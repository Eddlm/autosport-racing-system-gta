"""Check the settings keys the code touches against the schema that owns them, and the schema itself.

The owned ini files are already enforced at load: `SettingsRepair.PruneOwnedFiles` drops any key the
schema does not declare, and `CompleteOwnedKeys` writes every declared key that is missing or holds a
value the schema rejects. What neither can see is whether the CODE agrees with the schema:

  * a key written by the menu but not declared is pruned on the next load, so the setting silently
    reverts to its default - the bug that made the governor switch look like it persisted;
  * a key READ but not declared is worse: the store seeds it into the file, so it looks like it works
    for the session and is deleted the next time the game loads;
  * and `Migrate` seeds too, so a migration target that is not declared is carried over and lost.

And whether the SCHEMA agrees with itself: a default its own repair would reject makes the completion
pass rewrite that key and log it on every single load, forever, without ever settling.

Exit code is 1 when any ERROR is found, so this can gate a sweep. Only literal key names, literal
defaults and literal option lists are checkable - the Debug menu's keys come from its toggle table and
several domains are `Enum.GetNames`, so those are counted and reported rather than guessed at.

Run: python docs/check-settings-keys.py
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "src"

STORE_FILES = {
    "SettingsMenuStore": "Menu-Settings.ini",
    "RaceMenuStore": "Menu-Race.ini",
    "DebugMenuStore": "Menu-Debug.ini",
}

READ_RE = re.compile(r"(SettingsMenuStore|RaceMenuStore|DebugMenuStore)\.(Get|GetInt|GetFloat|GetBool|TryGetFloat)\(\"([A-Za-z0-9_]+)\"")
WRITE_RE = re.compile(r"(SettingsMenuStore|RaceMenuStore|DebugMenuStore)\.(Set|Migrate)\(\"([A-Za-z0-9_]+)\"")
RACER_WRITE_RE = re.compile(r"SaveRacerSetting\(\"([A-Za-z0-9_]+)\"")
SPEC_RE = re.compile(r"\.Specs\.Add\(new KeySpec\(\"([^\"]+)\",\s*([^,]+?)\s*(?:,\s*([^;]+?))?\s*\)\);")
LITERAL_LIST_RE = re.compile(r"new\[\]\s*\{([^}]*)\}")
QUOTED_RE = re.compile(r"^\"([^\"]*)\"$")
RANGE_RE = re.compile(r"(-?[0-9.]+)f?\s*,\s*(-?[0-9.]+)f?\s*$")


def strip_comments(text):
    return re.sub(r"//[^\n]*", "", text)


def parse_literal_list(text):
    """['a','b'] for a literal `new[] { ... }`, else None."""
    found = LITERAL_LIST_RE.search(text or "")
    if not found:
        return None
    return re.findall(r"\"([^\"]*)\"", found.group(1))


def parse_specs():
    """Per file: (key, default or None, kind, literal domain or None, (min, max) or None, raw default)."""
    text = strip_comments((SRC / "SettingsRepair.cs").read_text(encoding="utf-8-sig"))
    per_file = {}
    current = None
    for line in text.splitlines():
        opened = re.search(r"FileSpec\s+(\w+)\s*=\s*Owned\(\"([^\"]+)\"\)", line)
        if opened:
            current = opened.group(2)
            per_file.setdefault(current, [])
            continue
        if current is None:
            continue
        found = SPEC_RE.search(line)
        if not found:
            continue
        key, default_raw, rest = found.group(1), found.group(2).strip(), (found.group(3) or "")
        quoted = QUOTED_RE.match(default_raw)
        default = quoted.group(1) if quoted else None
        kind_match = re.search(r"Kind\.(Text|Bool|Number)", rest)
        kind = kind_match.group(1) if kind_match else "Text"
        rng = RANGE_RE.search(rest)
        bounds = (float(rng.group(1)), float(rng.group(2))) if rng else None
        per_file[current].append((key, default, kind, parse_literal_list(rest), bounds, default_raw))
    return per_file


def code_keys():
    reads, writes = set(), set()
    for path in sorted(SRC.glob("*.cs")):
        text = strip_comments(path.read_text(encoding="utf-8-sig"))
        for store, _method, key in READ_RE.findall(text):
            reads.add((STORE_FILES[store], key))
        for store, _method, key in WRITE_RE.findall(text):
            writes.add((STORE_FILES[store], key))
        for key in RACER_WRITE_RE.findall(text):
            writes.add(("Menu-Settings.ini", key))
    return reads, writes


def close_enough(value, entries):
    return any(entry == value or entry.lower() == value.lower() for entry in entries)


def check_schema(file, specs, errors):
    """Defaults the schema's own repair would reject, and duplicate declarations."""
    seen = set()
    computed = [0, 0]
    for key, default, kind, domain, bounds, default_raw in specs:
        if key in seen:
            print(f"ERROR  {file}: '{key}' is declared twice")
            errors[0] += 1
        seen.add(key)
        if default_raw != "null" and default is None:
            computed[0] += 1
            continue
        if default is None:
            continue
        if kind == "Bool" and default.lower() not in ("true", "false"):
            print(f"ERROR  {file}: '{key}' is Bool but its default '{default}' does not parse")
            errors[0] += 1
        if kind == "Text":
            if domain is None:
                computed[1] += 1
            elif not close_enough(default, domain):
                print(f"ERROR  {file}: '{key}' defaults to '{default}', which its own domain rejects - the completion pass rewrites it on every load")
                errors[0] += 1
        if kind == "Number":
            try:
                number = float(default)
            except ValueError:
                print(f"ERROR  {file}: '{key}' is Number but its default '{default}' does not parse")
                errors[0] += 1
                continue
            if domain is None:
                computed[1] += 1
            elif not close_enough(default, domain):
                print(f"ERROR  {file}: '{key}' defaults to '{default}', which is not one of its own offered values - the completion pass rewrites it on every load")
                errors[0] += 1
            if bounds and not (bounds[0] <= number <= bounds[1]):
                print(f"ERROR  {file}: '{key}' defaults to '{default}', outside its own range {bounds[0]}..{bounds[1]}")
                errors[0] += 1
    return computed


def main():
    per_file = parse_specs()
    reads, writes = code_keys()
    errors = [0]
    warnings = 0
    computed_defaults = computed_domains = 0

    for file in sorted(per_file):
        declared = {spec[0] for spec in per_file[file]}
        for key in sorted(k for f, k in writes if f == file):
            if key not in declared:
                print(f"ERROR  {file}: '{key}' is WRITTEN but not declared - it will be pruned on the next load")
                errors[0] += 1
        for key in sorted(k for f, k in reads if f == file):
            if key not in declared:
                print(f"ERROR  {file}: '{key}' is READ but not declared - it seeds a key the next load deletes")
                errors[0] += 1
        for key in sorted(declared):
            if (file, key) not in reads and (file, key) not in writes:
                print(f"warn   {file}: '{key}' is declared but nothing reads or writes it")
                warnings += 1
        computed = check_schema(file, per_file[file], errors)
        computed_defaults += computed[0]
        computed_domains += computed[1]

    total = sum(len(v) for v in per_file.values())
    print(f"\n{errors[0]} error(s), {warnings} warning(s) over {total} declared keys")
    if computed_defaults or computed_domains:
        print(f"not statically checkable: {computed_defaults} computed default(s), {computed_domains} computed domain(s)")
    return 1 if errors[0] else 0


if __name__ == "__main__":
    sys.exit(main())
