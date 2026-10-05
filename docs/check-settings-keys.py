"""Check the settings keys the code touches against the schema that owns them.

The owned ini files are already enforced at load: `SettingsRepair.PruneOwnedFiles` drops any key the
schema does not declare, and `CompleteOwnedKeys` writes every declared key that is missing. What is
NOT enforced is the agreement between that schema and the code:

  * a key written by the menu but not declared is pruned on the next load, so the setting silently
    reverts to its default - the bug that made the governor switch look like it persisted;
  * a key READ but not declared is worse: `MenuSettings.SeedIfMissing` writes it into the file, so it
    looks like it works for the session and is deleted the next time the game loads;
  * and `Migrate` seeds too, so a migration target that is not declared is carried over and lost.

Exit code is 1 when any ERROR is found, so this can gate a sweep.

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

# Get* seeds on read; Set and Migrate both write. SaveRacerSetting wraps SettingsMenuStore.Set.
READ_RE = re.compile(r"(SettingsMenuStore|RaceMenuStore|DebugMenuStore)\.(Get|GetInt|GetFloat|GetBool|TryGetFloat)\(\"([A-Za-z0-9_]+)\"")
WRITE_RE = re.compile(r"(SettingsMenuStore|RaceMenuStore|DebugMenuStore)\.(Set|Migrate)\(\"([A-Za-z0-9_]+)\"")
RACER_WRITE_RE = re.compile(r"SaveRacerSetting\(\"([A-Za-z0-9_]+)\"")


def declared_keys():
    """{ini file name: set of keys} from the FileSpec blocks in SettingsRepair.cs."""
    text = (SRC / "SettingsRepair.cs").read_text(encoding="utf-8-sig")
    per_file = {}
    current = None
    dynamic = {}
    for line in text.splitlines():
        opened = re.search(r"FileSpec\s+(\w+)\s*=\s*Owned\(\"([^\"]+)\"\)", line)
        if opened:
            current = opened.group(2)
            per_file.setdefault(current, set())
            dynamic.setdefault(current, 0)
            continue
        if current is None:
            continue
        literal = re.search(r"\b\w+\.Specs\.Add\(new KeySpec\(\"([^\"]+)\"", line)
        if literal:
            per_file[current].add(literal.group(1))
            continue
        if re.search(r"\b\w+\.Specs\.Add\(new KeySpec\(", line):
            dynamic[current] += 1
    return per_file, dynamic


def code_keys():
    """(reads, writes) as {(file, key)} from every store use in src/."""
    reads, writes = set(), set()
    for path in sorted(SRC.glob("*.cs")):
        text = path.read_text(encoding="utf-8-sig")
        for store, _method, key in READ_RE.findall(text):
            reads.add((STORE_FILES[store], key))
        for store, _method, key in WRITE_RE.findall(text):
            writes.add((STORE_FILES[store], key))
        for key in RACER_WRITE_RE.findall(text):
            writes.add(("Menu-Settings.ini", key))
    return reads, writes


def main():
    per_file, dynamic = declared_keys()
    reads, writes = code_keys()
    errors = warnings = 0

    for file in sorted(per_file):
        declared = per_file[file]
        for key in sorted(k for f, k in writes if f == file):
            if key not in declared:
                print(f"ERROR  {file}: '{key}' is WRITTEN but not declared - it will be pruned on the next load")
                errors += 1
        for key in sorted(k for f, k in reads if f == file):
            if key not in declared:
                print(f"ERROR  {file}: '{key}' is READ but not declared - it seeds a key the next load deletes")
                errors += 1
        for key in sorted(declared):
            used = (file, key) in reads or (file, key) in writes
            if not used:
                print(f"warn   {file}: '{key}' is declared but nothing reads or writes it")
                warnings += 1

    for file, count in sorted(dynamic.items()):
        if count:
            print(f"note   {file}: {count} key(s) declared dynamically - not statically checkable")

    print(f"\n{errors} error(s), {warnings} warning(s) over {sum(len(v) for v in per_file.values())} declared keys")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
