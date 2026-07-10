# Autosport Racing System — Architecture

> A high-level map of the ARS GTA V racing script.
> Source of truth: the C# code, not the README. The README is a 2019
> author notice; everything below is derived from the current build.

## What it is

ARS is a single-player C# script for **Grand Theft Auto V** that adds
arcade-style road racing on top of stock GTA traffic and physics. It
spawns opponent AI drivers, runs a full race lifecycle (grid → countdown
→ laps → finish), reads its own XML track format, and writes its output
controls directly to the underlying ScriptHookV natives.

It does **not** introduce a custom physics engine. AI driving is a
planner layered on top of GTA's vehicle model, and "racing behavior" is
expressed as `(throttle, brake, steering input)` applied to a normal
`Vehicle`.

## Runtime shape

```
┌──────────────────────────────────────────────────────────┐
│                    ScriptHookV host                       │
│                  (the GTA process)                        │
│                                                            │
│   ┌──────────────────────────────────────────────────┐   │
│   │          ARS : Script (AutosportRacingSystem)    │   │
│   │                                                   │   │
│   │  • Loads settings + tracks (XML, on background)  │   │
│   │  • Owns the menu (GTA scaleforms + tick UI)      │   │
│   │  • Owns the per-frame `OnTick` loop              │   │
│   │  • Owns global state (TrackPoints, Racer list)   │   │
│   │  └──── runs ────► per-Racer `ProcessTick()`       │   │
│   │                  per-Racer `ProcessAI()`          │   │
│   │                  per-Racer `TranslateSteer()`     │   │
│   │                  per-Racer `SpeedToThrottleBrake` │   │
│   └──────────────────────────────────────────────────┘   │
│                           │                                │
│                           ▼                                │
│   ┌──────────────────────────────────────────────────┐   │
│   │   Racer[] — one per car in the current race      │   │
│   │   (player included if joined)                     │   │
│   │   Each Racer owns:                                │   │
│   │   • Vehicle + Ped handles                         │   │
│   │   • VehicleControl (steer/throttle/brake/targets) │   │
│   │   • Memory (intention, rivals, corner, personality)│   │
│   │   • PID lane controller (steer follow)            │   │
│   │   • BehaviorVariance (per-instance jitter)        │   │
│   └──────────────────────────────────────────────────┘   │
│                           │                                │
│                           ▼                                │
│   ┌──────────────────────────────────────────────────┐   │
│   │  Static track model (ARS.TrackPoints,            │   │
│   │  ARS.CornerPoints, ARS.Path, ARS.WideDict, …)    │   │
│   │  Built once from XML, mutated at runtime when     │   │
│   │  the first racer populates grip per node.        │   │
│   └──────────────────────────────────────────────────┘   │
│                           │                                │
│                           ▼                                │
│   Native GTA memory / Vehicle (steer, throttle, brake)    │
└──────────────────────────────────────────────────────────┘
```

## Files

| File | Role | LOC |
| --- | --- | --- |
| `AutosportRacingSystem.cs` | Script entry, settings, menu, race lifecycle, track load, freecam, debug drawing, helpers | ~6500 |
| `Racer.cs` | Per-vehicle AI pipeline (perceive → decide → apply) | ~1555 |
| `DataStructures.cs` | `TrackPoint`, `CornerPoint`, `Memory`, `VehicleControl`, `Rival`, `PersonalitySet` | ~285 |
| `SkillSet.cs` | Reserved for skill configurations (currently empty) | 9 |
| `PersonalitySet.cs` | Reserved for personality files (currently empty — the live type lives in `DataStructures.cs`) | 10 |
| `Properties/AssemblyInfo.cs` | Standard assembly metadata | — |

`Racer.CallTree.md` already exists and gives a detailed method-level
call graph for `Racer.cs`. Treat it as the implementation reference.
`Racer.md` is a refactor TODO, not documentation; read it before
editing `Racer.cs`.

## Core subsystems

### 1. Script host and lifecycle

`ARS` extends ScriptHookV's `Script` class. Its constructor wires two
events:

- `Tick += OnTick` — runs every game frame.
- `Aborted += OnAbort` — runs when the script is unloaded.

The constructor also resets `scripts\ARS\Log.log`, calls `LoadSettings()`,
and prints a `~b~ARSe~w~` startup notification with the assembly
version.

`OnTick(object, EventArgs)` is the master loop. It runs the load task,
handles track-creator UI, the freecam, draw-time debug overlays, scaleform
countdown, and the timescaled traffic system. It is wrapped in
`try/catch` that logs `OnTick error: <ex>` on failure — see
*Notes on the tick* below.

`LoadSettings()` reads the INI file once; `StartLoadScript()` kicks
off `FillKnownDisciplines`, `FillKnownTracks`, and `ReFilterKnownTracks`
on a background `Task`. The actual model-spawning
(`FillCachedCandidates`) must then run on the script thread, and
`HandleLoadScriptTask()` does that once the task completes. This is the
only place in the codebase that has to be careful about the GTA thread.

### 2. Track model

Tracks are XML files under `scripts\ARS\Tracks\` (subfolders allowed).
`FillKnownTracks` walks them, indexes tags into `TrackTags`, and stores
each file's first point as an "immersive join" location in
`ImmersiveJoins`. The `<Tags>`, `<Disciplines>`, and `<Route><Point>`
elements are read by `GetTrackTags`, `GetRacerTags`, and
`GetTrackStartPos`.

At runtime, the track is a flat array of `TrackPoint` objects
(`ARS.TrackPoints`) plus derived dictionaries:

- `Path` — ordered `Vector3` of node positions.
- `Angles` — signed-angle-per-node used by the AI planner.
- `WideDict` — track width per node.
- `MultiplierInTerrain` — grip multiplier per node, filled by the first
  racer to cross it.
- `NodeScalarData` — generic per-node float; the corner lane profile
  stores its `DeviationFromCenter` values here.

`CornerPoint` is a higher-level entity (a series of consecutive
`TrackPoint`s). The corner list is computed once, and each `Racer` keeps
an "active corner" in `Brain.Corner`.

### 3. Race lifecycle

`RaceState` is a flat enum: `None`, `NotInitiated`, `Countdown`,
`InProgress`, `PostRace`, `Finished`. Transitions are driven by the
menu (the `Options` enum) and by the timer/countdown path in `OnTick`.

`Launch()` on each `Racer` snapshots the next key corner into
`Brain.Corner`, zeroes lap times, starts a per-racer
`LapStartTime = Game.GameTime`, and gives a randomized handbrake
hold-off (`HandBrakeTime = Game.GameTime + GetRandomInt(100, 400)`).
The handbrake is a feature, not a bug — it staggers the start so the
player has a fighting chance.

`LeaderboardFinish` is filled in arrival order and is what the post-race
screen reads.

### 4. Per-racer AI pipeline

A `Racer` updates on two clocks:

- `ProcessTick` — every game frame, does lightweight perception and
  applies last-frame control outputs.
- `RunTimedCore` — at a slower cadence (≈100 ms, see
  `LastCoreTick`), does the heavy work: nearest-node resolution,
  lookahead projection, grip rebuild, corner scan, rival refresh.

`ProcessAI` is the inner decision loop. The intended call order
(captured in `Racer.md`) is:

1. perceive (follow track, dynamic bbox, perceived grip, corner info)
2. decide steer (`SteerTrack` → `SteerApplyCorrections`)
3. decide speed (`SpeedTrack` → `SpeedToThrottleBrake`)
4. corrections (traction control, ABS lock-up limiter)
5. translate inputs (`TranslateSteer`)
6. stuck/recovery overrides (`ApplyStuckRecoveryOverride`)

`SteerTrack` produces a *desired lane offset*; a `PID` controller turns
that into a steer angle. `SteerApplyCorrections` then physics-clamps
the angle (max yaw rate vs expected yaw rate), applies a slide
counter-steer, and writes the final value. `TranslateSteer` is the
last-mile "degrees → stick input" and includes the only rate limiter.

### 5. Rival awareness

Each `Racer.Memory` keeps three `Rival` slots. `UpdateRivals` picks the
nearest three other racers by a combined score (distance + relative
position), and `UpdateRivalInfo` recomputes relative position, lane
occupancy, time-to-reach/time-to-rear, and the rival's lane offset.
These feed the `PersonalityRivals` block, which controls overtaking
risk (dive-bomb willingness, side-to-side minimum distance, aggression
buildup rate).

### 6. Vehicle dynamics and grip

Grip is *not* read from the vehicle's `handling.meta` directly. It is
estimated at runtime from `BaseMechanicalGrip`, terrain (sand / tarmac
/ grass are detected via GTA surface hashes), and observed Gs. The
estimate flows back into `SteerApplyCorrections`, the corner-speed
planner, and the traction-control throttle cap.

Terrain hashes are stored as integer lists on `ARS`:

- `ARS.Other` — surfaces with no special handling.
- `ARS.Road` — tarmac / concrete variants.
- `ARS.Dirt` — dirt, gravel, loose gravel.
- `ARS.Sand` — sand only.

`ARS.TerrainTypes` is the typed alias of these, used inside the AI.

### 7. Race creator / freecam / debug

These are the "while not racing" modes. The track creator
(`HandleTrackCreator`, `CreateTrack`, `SaveTrack`) is a full in-game
editor: you can drive a route, place track-limit props, save to XML,
and reload it. The freecam is a `ba_prop_battle_drone_quad_static`
spawned above the player that the script attaches a cinematic camera to.

`DebugDisplay` selects the in-world overlay (none / inputs / speed /
positioning / prop edit). All four are gated by options in
`OptionValuesList`.

## Settings and persistence

`LoadSettings()` reads the INI; the path is hard-coded to
`scripts\ARS\`. The relevant runtime toggles are:

- `DevSettingsFile` — `GENERAL.LoadAtStart` (auto-load on game start,
  default true) and `RACERS.AIRacerAutofix` (0 = crash-realistic, 1 =
  strong axles, 2 = god-mode invulnerability).
- `OptionValuesList` — per-run toggles (aggro display, input display,
  track-analysis display, physics display, use nearby cars, reverse
  route).
- `HiddenDebugVisual` and `DebugVisual` — debug overlay level and
  category.

There is no persistence layer for races themselves — track files and
INI settings are the only persistent state. The session is rebuilt
from those on every game start.

## Data flow summary

```
        ┌────────────────────────────┐
        │  scripts\ARS\Tracks\*.xml  │   (on disk)
        └────────────┬───────────────┘
                     │ FillKnownTracks
                     ▼
        ┌────────────────────────────┐
        │  ARS.TrackPoints / Path /  │   (static, in-memory)
        │  WideDict / CornerPoints   │
        └────────────┬───────────────┘
                     │  per-frame ProcessTick
                     ▼
        ┌────────────────────────────┐
        │   Racer.CurrentTrackPoint  │   (per-racer, per-frame)
        │   Racer.LookAheads         │
        │   Racer.Memory             │
        └────────────┬───────────────┘
                     │ SteerTrack / SpeedTrack
                     ▼
        ┌────────────────────────────┐
        │  VehicleControl.{Steer,    │   (per-racer, per-frame)
        │  Throttle, Brake}          │
        └────────────┬───────────────┘
                     │ TranslateSteer + SpeedToThrottleBrake
                     ▼
        ┌────────────────────────────┐
        │  Native GTA Vehicle inputs │   (the actual car)
        └────────────────────────────┘
```

## Notes on the tick

- `OnTick` is wrapped in a single `try { ... } catch (Exception ex) { Log(LogImportance.Error, "OnTick error: " + ex, true); }` near
  the bottom. Any uncaught exception is logged but does not crash the
  script — it can swallow real bugs. Be careful when debugging.
- `IsLoadingScript` + `LoadScriptTask` is the one place where threading
  is meaningful: track discovery runs on a `Task.Run`, but any code
  that touches `GTA.Model` or natives must run on the script thread.
  `HandleLoadScriptTask()` is the bridge.
- `GTA.Native.Function.Call(...)` is used heavily and bypasses the
  ScriptHookV managed wrappers. Several known hashes are inlined as
  numeric casts (e.g. `(Hash)0xA132FB5370554DB0`). Searching by hex is
  the way to find them.

## Where to look first

If you are new to the code:

1. Read `AutosportRacingSystem.cs` constructor + `OnTick` — the loop
   shape.
2. Skim `Racer.cs::RunTimedCore` and `Racer.cs::ProcessAI` — the AI
   loop shape.
3. Read `Racer.CallTree.md` for the method-level map.
4. Read `DataStructures.cs` — every important type lives here.
5. Read `Racer.md` — it is a refactor backlog, not docs, but reading
   it tells you which methods are about to change.
