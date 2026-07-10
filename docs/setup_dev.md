# ARS — Build, Run, and Develop

> How to set up a working dev environment for the Autosport Racing
> System GTA V script.

## Stack

| | |
| --- | --- |
| Language | C# |
| Target framework | .NET Framework 4.8 (legacy, not .NET Core) |
| Host | ScriptHookV .NET 2 (SHVDN) inside GTA V |
| Build | MSBuild 14.0 (`Microsoft.Common.props`) |
| IDE | Visual Studio 2015+ (any version that supports `ToolsVersion=14.0` and .NET 4.8) |

`NewRacingSystem.csproj` is a legacy-style csproj. There is no
`Microsoft.NET.Sdk`, no PackageReference, no `dotnet build`. Use
`msbuild`, `devenv`, or open the `.sln` in Visual Studio.

## Prerequisites

1. **GTA V** installed and working.
2. **ScriptHookV** by Alexander Blade — drop `ScriptHookV.dll` into
   the GTA V folder.
3. **ScriptHookVDotNet2** — drop `ScriptHookVDotNet2.dll` into the
   GTA V folder. ARS references this assembly (see the csproj's
   `<Reference Include="ScriptHookVDotNet2">`).
4. **Visual Studio 2015 or newer** with the .NET desktop workload.
   Community Edition is fine.

Verify GTA V is runnable *without* ARS first. If GTA V crashes on
start, ARS can't help you diagnose it.

## Project layout

```
NewRacingSystem/
├── AutosportRacingSystem.cs   # main script
├── Racer.cs                   # per-racer AI
├── DataStructures.cs          # types
├── SkillSet.cs                # (empty, reserved)
├── PersonalitySet.cs          # (empty, type lives in DataStructures.cs)
├── Properties/AssemblyInfo.cs
├── NewRacingSystem.csproj
├── NewRacingSystem.sln
├── README.md                  # 2019 author notice
├── Racer.md                   # refactor TODO
├── Racer.CallTree.md          # method-level call graph
└── docs/                      # these docs
    ├── architecture.md
    ├── ai_system.md
    └── setup_dev.md
```

## Build

### Visual Studio

1. Open `NewRacingSystem.sln`.
2. Set configuration to **Debug | Any CPU** (or x64 — both work).
3. Build → Build Solution (`Ctrl+Shift+B`).
4. The post-build target `CopyArsDll` (see csproj) automatically
   copies `bin\Debug\ARS.dll` to
   `D:\SteamLibrary\steamapps\common\Grand Theft Auto V\Scripts\ARS\`
   *if that path exists*.

### Command line

```bash
# From a Developer Command Prompt for VS:
msbuild NewRacingSystem.sln /p:Configuration=Debug /p:Platform="Any CPU"
```

`msbuild` is preferred over `dotnet build` because the project uses
`ToolsVersion="14.0"`. `dotnet build` will complain.

## Where the build expects to land

The csproj hard-codes the GTA V path:

```xml
<GtaScriptsRoot>D:\SteamLibrary\steamapps\common\Grand Theft Auto V\Scripts</GtaScriptsRoot>
<GtaArsScriptsDir>$(GtaScriptsRoot)\ARS</GtaArsScriptsDir>
```

**If your GTA V is anywhere else, the auto-copy silently does
nothing.** The build still succeeds and `ARS.dll` ends up in
`bin\Debug\`. You then have to copy it manually.

The recommended fix: edit the `<GtaScriptsRoot>` in
`NewRacingSystem.csproj` to match your install, or remove the
`CopyArsDll` target and copy `bin\Debug\ARS.dll` to
`Grand Theft Auto V\Scripts\ARS\ARS.dll` by hand.

The runtime expects this exact directory layout under
`Grand Theft Auto V\Scripts\ARS\`:

```
Scripts\ARS\
├── ARS.dll
└── Tracks\                 # your track files
```

## Run

1. Launch GTA V.
2. ARS will show a blue "ARSe" notification in the bottom-right
   corner once the script loads.
3. Press `Sprint + E` to open the menu (the message is added to
   `HelpMessages` after `Loaded` flips to true).
4. Select a track, set grid size, and start a race.

There are two ways ARS starts loading:

- **Auto-load**: the `GENERAL.LoadAtStart` setting in the dev INI
  defaults to `true`. ARS loads on first tick.
- **Manual load**: type the cheat code `arson` in-game (see
  `WasCheatStringJustEntered` in `OnTick`).

If ARS doesn't show up at all:

- Check `scripts\ARS\Log.log` — the constructor wipes and rewrites
  this file on every load. An empty file after a launch attempt
  means SHVDN didn't see the script.
- Verify `ARS.dll` is in `Scripts\ARS\`, not just `Scripts\`.

## Development loop

A typical iteration:

1. Edit code in VS.
2. Build (Ctrl+Shift+B).
3. The post-build copies `ARS.dll` to `Scripts\ARS\`.
4. In GTA V, press the abort combo for SHVDN (default `Insert`,
   reloads scripts). Or fully quit and relaunch GTA V.
5. Test in-game.

SHVDN can hot-reload scripts if you abort and re-enter the menu
combination. For a full reset, quit GTA V and relaunch — the
`TrackPoints` list and any race state live in memory only.

## Settings files

ARS reads its own INI from `scripts\ARS\`. The exact path is set
in `LoadSettings()`. The relevant keys for development:

```
[GENERAL]
LoadAtStart = 1            ; auto-load on first tick

[RACERS]
AIRacerAutofix = 1         ; 0 = realistic, 1 = strong axles, 2 = god mode
```

The other runtime options (`ShowAggro`, `ShowInputs`,
`ShowTrackAnalysis`, `ShowPhysics`, `UseNearbyCars`, `ReverseRoute`)
are in-game toggles in the menu, not INI keys.

## Debugging

### In-game debug overlays

Four overlay modes, selectable from the menu or by setting
`DebugVisual` directly:

- `None` — no overlay.
- `Inputs` — shows the live steer / throttle / brake.
- `Speed` — shows speed, grip, Gs.
- `Positioning` — shows track point, lookaheads, lane offset.
- `PropEdit` — track creator mode (also enters `routeEditMode`).

These are gated by `OptionValuesList`. Enable them in the menu and
they'll show in the world.

### Logs

`scripts\ARS\Log.log` is the only persistent log. The constructor
truncates it; subsequent calls to `ARS.Log(LogImportance.Info, ...)`
and `Log(LogImportance.Error, ...)` append.

Wrap your debug prints in `ARS.Log` rather than `Console.WriteLine` —
GTA V is a Win32 game with no console attached.

### The `OnTick` catch

`OnTick` is wrapped in a single `try/catch` that logs and
continues. If you introduce a bug that throws every frame, the
script will spam the log but keep running. To see real crashes,
narrow the `try` to a specific subsystem, or comment the catch
during development. The user has previously called out that
"seeing the error and crash is preferred over swallowing it this
early."

### Reading the source

A few conventions to know:

- All important code lives in the `ARS` namespace.
- `Racer.cs` is structured roughly top-to-bottom by call order
  (perception → steer → speed → corrections → recovery → draw),
  with `RunTimedCore` and `ProcessAI` near the middle. This is
  not a hard rule; see `Racer.md` for the intended refactor.
- `Racer.CallTree.md` is the most accurate method map.
- `GTA.Native.Function.Call` is used heavily. Numeric hashes
  (e.g. `(Hash)0xA132FB5370554DB0`) are inlined natives; you
  will need a native DB to decode them.
- `ARS.TrackPoints`, `ARS.Path`, `ARS.WideDict`, etc. are
  static. They are populated once at load and shared across
  all racers. Do not mutate from inside `Racer` unless you mean
  to mutate for everyone.

## Common issues

| Symptom | Likely cause | Fix |
| --- | --- | --- |
| `ARS` does not appear in-game | `ARS.dll` not in `Scripts\ARS\` | Copy by hand; verify GTA V path in csproj |
| Log is empty after launch | SHVDN didn't load the DLL | Check GTA V is non-Steam or that ScriptHookV is current |
| AI racers drive into walls | `BaseMechanicalGrip` reads zero (e.g. modded vehicle) | Set `RACERS.AIRacerAutofix = 1` for now |
| Tracks don't show up in the menu | Folder structure wrong | Tracks must be `Scripts\ARS\Tracks\*.xml` (subfolders allowed) |
| Stuck AI racers everywhere | Recovery state-machine bug | See `Racer.md` §9 — the recovery logic is on the refactor list |
| Build error: "missing ScriptHookVDotNet2.dll" | HintPath in csproj points at a path you don't have | Edit the `<HintPath>` in `NewRacingSystem.csproj` |

## License and contribution

The `README.md` (2019, Eddlm) places restrictions:

- View and study the source — allowed.
- Pull requests — allowed.
- Re-release in any form — **not allowed**.
- Derivatives — **not allowed.**

Treat this as a code-reading / patch-back project, not a fork.
