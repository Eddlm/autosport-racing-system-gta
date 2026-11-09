# Plan — N.O.S. compatibility and the PR #11 adoption triage

**Status: proposal, not implemented.** Written 2026-11-05 against `master` @ `463988c`
(`src/AutosportRacingSystem.cs` 4,1xx lines, `src/Racer.cs` 4,897 lines, `src/FreeCamController.cs` 180 lines,
`src/AutosportRacingSystem.TrackCreator.cs`, `src/VehicleCatalog.cs`).
Source under review: PR #11 by VirtuOzZ (`VirtuOzZ-Mods`), branch `feature/ars-nos-integration`,
one commit `707e92f`, 8 files, +761/−128, branched from `c991306` — **40 commits behind** this master.

**Credit.** Every item adopted from this PR carries his name wherever the code is explained: the adapter
header, `docs/FEATURES.md`, and the release note. The comment budget (~5,000 `//` chars in `src/`) is the
reason the attribution is a header line plus a doc row, never a paragraph per function.

## Scope and non-goals

Scope: (1) a N.O.S. integration designed to be optional and to keep one owner of the nitro effect;
(2) a verdict per remaining feature of the PR, with the ones worth taking specified to the line.

- **Non-goal: merging the PR branch.** It predates the steering/brake overhaul (`Racer.cs` moved 1,165
  lines under it). A merge is a rewrite of his diff; only the ideas are reusable.
- **Non-goal: a second roster pipeline.** The captured-grid feature builds a parallel source of truth for
  "who is on the grid". That collides with the one-writer rule and is deferred (item 6).
- **Non-goal: any hard reference to the N.O.S. assembly.** ARS must load and race with N.O.S. absent.
- **Non-goal: N.O.S. Simulator mode for AI.** His own page limits AI nitro to Infinite and Regenerating.

## Shared plumbing

**The nitrous ask.** ARS owns *when*, and that does not change: `NitrousDurationMs = 3000`
(`Racer.cs:476`), `_nitrousActiveUntil`, one bottle per lap (`_nitrousLapUsed` `:512`, gate `:2925`), and
the card that decides a burn is worth it (`TryPlayNitrousCard` `:2923`). Only the **ask** is rerouted — and
the ask is not one call, it is two:

| the ask | where | what it does |
|---|---|---|
| `REQUEST_NAMED_PTFX_ASSET` + `FullyChargeNitrousHash` + `OverrideNitrousLevelHash(true…)` | `Racer.cs:3121-3123` | the game's own nitro: flames, exhaust, boost |
| `CheatPowerIncreaseHash(Car, multiplier)` with `NitrousPowerMultiplier = 5f` | `Racer.cs:3674`, from `ApplyPowerMultiplier` `:3645` | ARS's ×5 engine power while the burn is live |

`StopNitrous` (`:3157`) undoes the first; the second falls back to `_powerCounter` once
`_nitrousActiveUntil` passes. With N.O.S. present **both** must hand over, or the car gets N.O.S.'s
acceleration multiplied by five.

No interface, no backend field, no new concept: one static adapter whose `TryStart(Vehicle)` answers whether
it took the ask, and today's bodies kept as the fallback.

```csharp
void StartNitrous()
{
    _nosBurning = !ControlledByPlayer && NosNitro.TryStart(Car);
    if (!_nosBurning) { the three natives, verbatim }
    _nitrousActiveUntil = Game.GameTime + NitrousDurationMs;
    _nitrousLapUsed = Lap;
}

void StopNitrous()
{
    if (_nosBurning) NosNitro.Stop(Car); else { the native, verbatim }
    _nosBurning = false;
    _nitrousActiveUntil = 0;
}
```

`TryStart` returning false *is* the "business as usual" rule: no branch at the call site, and a missing,
older, or refusing API degrades to today's behaviour by construction.

## Items

### 1. N.O.S. AI backend (the compatibility ask)

**Contract, as used by his working code.** Assembly `N.O.S - Nitro Overdrive System.API`, namespace
`NitroOverdriveSystem.API`: `NOSApi.API_VERSION` (≥2), `NOSApi.SupportsAiNitro`,
`RegisterAiVehicle(int handle, NOSAiMode, NOSAiOptions) -> bool`, `SetAiNitro(int handle, bool)`,
`ReleaseAiVehicle(int handle)`; `NOSAiOptions { FlameStyle, Audio }`.

**Verification before any code.** These signatures come from reflection strings in a diff, not from the
API. One command settles it once the mod is installed (or from the Developer SDK, which he distributes on
Discord — not in the mod download):

```
ilspycmd "N.O.S - Nitro Overdrive System.API.dll"    # ilspycmd 9.1.0 is installed at ~/.dotnet/tools
```

Read the public surface only: parameter types and the `NOSAiMode` values. A wrong enum value registers the
car in a mode that regenerates on its own, and the symptom is a rival nitro-ing outside its card.

**Where it plugs in.** Three call sites, all in `Racer.cs`: `StartNitrous` (`:3118`), `StopNitrous`
(`:3154`), and the multiplier in `ApplyPowerMultiplier` (`:3645`). Register on the first ask for that car;
`Release` in `Racer.Delete` (`:4878`) **before** `Car.Delete()` (`:4893`), or N.O.S. keeps a handle to a
dead car.

- `SetAiNitro(handle, pressed)` is a *state*, so call it on the edge only — cache the last value. Every
  tick per car is ~480 boxed reflection invokes a second on an 8-car grid, for no gain.
- `FlameStyle` chosen once per car at registration and kept; `Audio = true`.
- Mode: the one whose AI reserve cannot refill on its own. ARS's bottle is one per lap, and a mode that
  regenerates hands out a second burn the card never authorised.
- `_nosBurning` also gates the ×5 in `ApplyPowerMultiplier`, so the two boosts never stack.

**Two consequences, named now rather than discovered in game.**

- `Ai Nitro %` (`AutosportRacingSystem.cs:67`, menu row `:1265`, seed `SettingsRepair.cs:168`) goes inert
  for N.O.S.-routed cars: N.O.S. owns the strength, so its own setting is the tune. Annotate the row
  instead of leaving a knob that does nothing.
- The player's ask (`TryFireNitrous`, bound to `X` / `VehicleDuck`) **cannot** be rerouted: `RegisterAiVehicle`
  is an AI entry point, and N.O.S. already gives the player's car its own nitro, key, HUD and charge. So
  ARS's player burn stays business-as-usual; whether that reads as one system too many is a judgement for
  after a drive, not a rule to add now.

**Guards.** Probe lazily and cache the success; a failed probe stays retryable so a load-order race
resolves itself. Every invoke in a try/catch that degrades to "no API" rather than to an exception in a
tick. Assembly lookup scans `AppDomain.CurrentDomain.GetAssemblies()` first, `Assembly.Load` only as a
fallback.

**Acceptance test.** Same build, two runs: with N.O.S. 1.9 present an AI burn logs `nitrous backend: Nos`
and its flames/audio are N.O.S.'s; with the mod removed the same DLL logs `Native` and burns exactly as
today. That the *same artifact* does both is the proof of optionality.

**Effort S (~90 lines adapter, ~6 changed in `Racer.cs`). Risk: medium** — third-party API, unverified
until decompiled.

### 2. Fair race start — the player's vehicle during the countdown

**Real gap, verified.** `StartCountdown` (`AutosportRacingSystem.cs:2659`) sets `RaceState.Countdown` and
nothing else; the only `FreezePosition` in the tree is the DNF-under-track case (`Racer.cs:4519`). The AI
waits in `GridWait` and the player can floor it at "3".

**His version does too much:** a second `FreezePosition` owner that re-asserts every frame, teleports the
car back when it drifts >8 cm, disables six control actions, and runs twice per tick (before and after the
racer updates). A sticky `FreezePosition` is exactly the failure that survives a crashed race.

**Take instead:** while `RaceStatus == RaceState.Countdown`, disable accelerate/brake/left-right/handbrake
at one call site, and zero the velocity once per tick. No teleport, no freeze flag, no second call site.
`ReleasePlayerCountdownLock` then has nothing to release, and an aborted countdown cannot leave the car
paralysed.

**Acceptance test.** Holding W through the countdown does not move the car; at GO it launches normally;
aborting the countdown (menu → end) leaves the car fully drivable.

**Effort XS (~30 lines). Risk: low.**

### 3. Military / tank filter — one line, real

**Verified:** `VehicleCatalog.BuildPowerCache` (`VehicleCatalog.cs:96`) drops models whose
`GET_VEHICLE_CLASS_FROM_NAME` is in `BlacklistedVehicleClasses` (`AutosportRacingSystem.cs:107`), which
lists Motorcycles, Cycles, Boats, Helicopters, Planes, Trains — **not Military (19)**. A Rhino passes
`model.IsCar` and enters the pool.

**Take:** add `(VehicleClass)19` to that set. **Drop** the name-based second guard (`IsRoadRaceCarModel`'s
eight hardcoded model names) — with the class in, it filters nothing the class does not already filter,
and it re-derives `new Model(name).Hash` per candidate.

**Acceptance test.** The pool log prints `dropped <rhino>: class Military is blacklisted`.

**Effort XS. Risk: low** (it also drops weaponised road cars of that class — his eye on the pool list).

### 4. Free camera — take the rewrite, keep a floor

`src/FreeCamController.cs` is unchanged between the PR's base and this master, so the rewrite applies
cleanly. His complaint is the one real one: drone inertia (`_movement` accumulated and decayed per frame,
`ApplyMovementDecay`) and altitude driven by an `_intendedHeight` follower.

**Take:** camera-relative input, no inertia, `Sprint` = 45 m/s, 12 m/s otherwise, `dt` from `Game.FPS`
clamped to 0.1 s. **Keep one thing he removed:** a floor ~1.5 m above ground. Free vertical motion with no
floor lets the editing camera sink through terrain, which is the bug the old `_intendedHeight` clamp
existed to prevent.

**Acceptance test.** The camera answers a key press within one frame, Shift is 45 m/s, and pointing down
into a slope stops ~1.5 m above it instead of entering it.

**Effort S (~120 lines replaced). Risk: low.**

### 5. Track-editor undo ramp and ESC exit — take with the guard he lost

**Real defect in his code.** The old block kept a floor of two nodes (`if (RouteNodes.Count > 2)`) and
cleared the route below it. The new block calls `RouteNodes.RemoveAt(RouteNodes.Count - 1)` with **no
count guard**, so an empty route plus a held Aim throws `ArgumentOutOfRangeException` in a tick — the
script dies. Reachable on the very first frame of the editor (the route is empty until the start line is
placed) and again 350 ms after the last node is deleted while the button is still held.

**Take:** the ramp (350 ms delay, then 145 ms → 45 ms over 2.2 s), the section/preview invalidation on each
deletion, and ESC/Back to leave without saving — **with a `Count > 0` guard on the removal**, and the exit
key not fighting the `FrontendCancel` instruction already shown.

**Acceptance test.** Hold Aim on an empty route: no exception, the script survives. Hold Aim over a full
route: the first node goes after 350 ms, then accelerates. ESC leaves the editor and discards the route.

**Effort S. Risk: low.**

### 6. Captured racing grid — deferred, not taken

366 new lines (`AutosportRacingSystem.CapturedGrid.cs`) plus spawn-pipeline changes, four enum modes for
three behaviours (`CustomOnly` is migrated to `CustomClones` at `AutosportRacingSystem.cs:748`), and a
`CapturedGridMode` key written through both `RaceMenuStore` and a `SettingsRepair` `KeySpec`.

Why not: it is a **second source of truth** for grid membership (the roster pipeline already owns that),
it borrows live world cars and the peds standing within 7 m of them — random civilians become rivals — and
the "copy appearance" routine immediately overwrites the paint it copied. It also assumes the spawn slot
`RouteNodes[(Racers.Count + 1) * 10]`, which is a guess about node spacing.

The idea underneath is honest and separate: *race the cars that are around you*. If it is wanted, it is its
own design doc with its own question — where a borrowed car's pace and its release rules come from — not a
rider on a compatibility change. **Do not start before the nitrous seam exists.**

**Effort L. Risk: high.**

## Order and effort

| # | Item | Effort | Risk | Why here |
|---|---|---|---|---|
| 3 | Military class in the blacklist | XS | low | one line, no dependency |
| 5 | Editor undo ramp + ESC, guarded | S | low | self-contained quality of life |
| 2 | Fair start, simplified | XS | low | fixes a real fairness gap |
| 4 | Free camera rewrite + floor | S | low | the piece he asked for |
| 1 | N.O.S. backend behind the seam | S–M | medium | needs the DLL decompiled first |
| 6 | Captured grid | L | high | deferred; own design doc |

Items 3, 5, 2 and 4 are independent and land in one build. Item 1 is one build of its own, verified in
both configurations (with and without N.O.S.), because a backend that only works when the mod is present
is the failure this item exists to prevent.

## Open questions

1. **Player nitro ownership with N.O.S. installed** (item 1): yield ARS's `X` to N.O.S., keep both, or
   drive the player through the API too. Recommendation: yield.
2. **AI mode**: which `NOSAiMode` value leaves the reserve under ARS's control rather than regenerating on
   its own. Needs the decompile; if no mode gates it, the AI backend only registers for cars whose card
   fired, and releases immediately after `Stop`.
3. **Borrowed cars in the captured grid**: is "race what is parked around me" wanted at all, and if so does
   a borrowed car keep the pace of its model or of the ARS pool?
4. **Credit shape**: release-note line + `docs/FEATURES.md` row + adapter header, as assumed here, or a
   `CREDITS.md` entry as well.
