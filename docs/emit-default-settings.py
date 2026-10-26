"""Emit the four menu inis at their schema defaults, for the shipped package.

The mod writes these itself on first run, but the package should arrive with them so a player can read
what the defaults ARE without starting the game. The values come from the same schema the repair pass
uses - `SettingsRepair`'s `KeySpec` table - plus the Debug menu's toggle table, which is generated in a
loop and therefore not visible to a literal parse.

Run: python docs/emit-default-settings.py [output_dir]
Default output: Dist/AutosportRacingSystem/Settings
"""

import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from importlib.util import module_from_spec, spec_from_file_location  # noqa: E402

spec = spec_from_file_location("checker", HERE / "check-settings-keys.py")
if spec is None or spec.loader is None:
    raise SystemExit("cannot load check-settings-keys.py")
checker = module_from_spec(spec)
spec.loader.exec_module(checker)

ROOT = HERE.parent
SRC = ROOT / "src"
DEBUG_FILE = "Menu-Debug.ini"


def debug_toggle_defaults():
    """`{ Options.ShowLeaderboard, true }` -> [('ShowLeaderboard', 'True'), ...] in enum order."""
    text = (SRC / "AutosportRacingSystem.cs").read_text(encoding="utf-8-sig")
    pairs = re.findall(r"\{\s*Options\.(\w+),\s*(true|false)\s*\}", text)
    return [(name, "True" if value == "true" else "False") for name, value in pairs]


def main():
    out_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "Dist" / "AutosportRacingSystem" / "Settings"
    out_dir.mkdir(parents=True, exist_ok=True)
    per_file, dynamic = checker.parse_specs()
    written = []
    for name, specs in per_file.items():
        lines = ["[MENU]"]
        emitted = set()
        skipped = []
        for key, default, kind, domain, dyn, bounds, raw in specs:
            if default is None and not domain:
                # A code-computed default (a menu choice, an enum name): writing "" would ship a broken value, so the
                # key is left out and the mod seeds the real default on first load.
                skipped.append(key)
                continue
            lines.append(f"{key}={default if default is not None else domain[0]}")
            emitted.add(key)
        if name == DEBUG_FILE:
            for key, value in debug_toggle_defaults():
                if key not in emitted:
                    lines.append(f"{key}={value}")
        path = out_dir / name
        path.write_text("\r\n".join(lines) + "\r\n", encoding="utf-8", newline="")
        written.append((name, len(lines) - 1, skipped))
    for name, count, skipped in written:
        print(f"{name}: {count} keys" + (f", skipped (code-computed default): {', '.join(skipped)}" if skipped else ""))
    print(f"written to {out_dir}")


if __name__ == "__main__":
    main()
