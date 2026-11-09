# 413-T4 — State lifetime: per-car and per-race latches, timers, counters

- Date: 2026-10-30
- Build: 1376
- Read: `src/Racer.cs` (4708 lines, md5 `36242085b0feb5458d6e33f9b589fbcb`), `src/AutosportRacingSystem.cs` (3943 lines, md5 `68f13ee5b7da9af267392f9671ffdfb5`), `src/DataStructures.cs` (427 lines)
- Mode: read-only. No source edit, no build, no git, no game.

**Note on the baseline.** `Racer.cs` was edited by another task while this review was running (4704 → 4708 lines; the four lines now at `Racer.cs:652-655` — `ActiveManeuver.LastEnabled = 0;` and `Array.Clear(_cardCooldownUntil, ...)` — landed mid-review). Every line number and quote below was re-checked against the md5 above, not against an earlier read. Findings that the new lines settle (`ActiveManeuver`, `_cardCooldownUntil`) are therefore *not* reported as leaks any more; they were leaks an hour ago.

## The paths

| Tag | Path | Entry point |
|---|---|---|
| **I** | new race / restart, per car | `Racer.Initialize()` `Racer.cs:609`, called from `SetupRace` `ARS:2517`, `GridBuilder.Place` `GridBuilder.cs:29` |
| **L** | lights out, per car | `Racer.Launch()` `Racer.cs:1753`, called from the countdown tick `ARS:2164` |
| **D** | teardown, per car | `Racer.Delete()` `Racer.cs:4689`, called from `CleanRacers` `ARS:2475` |
| **C** | normal end / End Race menu | `CleanEverything()` `ARS:1802` (via race-complete `ARS:2239`, menu `ARS:844`, `ARS:873`, `TrackCreator.cs:31`) |
| **A** | script abort | `OnAbort()` `ARS:3250`, wired `Aborted += OnAbort` `ARS:357` |
| **S** | new race sets up | `StartRace()` `ARS:2424` → `SetupRace()` `ARS:2509` |

**A is not a peer of C.** `Aborted += OnAbort` (`ARS:357`) is the ScriptHookVDotNet script-unload event: the domain goes away with the assembly, so *in-memory* statics do not need clearing there — only world state (props, peds, natives on entities that survive) does. I therefore do not treat "static not reset in `OnAbort`" as a defect, and I say so where it matters. The real third path is **S**: `SetupRace` is reachable straight from the "Restart Race" menu item (`ARS:826-828`) *without* C, so C's resets are skipped entirely on a restart.

## Field table

Per-car (`Racer`). I = `Initialize`, L = `Launch`, D = `Delete`. "self" = cleared inside the race by its own logic.

| field | I | L | D | self | verdict |
|---|---|---|---|---|---|
| `_nitrousActiveUntil` (492) | no | no | **yes 4692** `if (_nitrousActiveUntil > 0) StopNitrous();` | `StopNitrous` 3049 + tick guard 2795 | OK — see nitrous verdict |
| `_nitrousLapUsed` (494) | yes 645 | no (L sets `Lap = 1`, 1772) | no | 3016 | OK |
| `_kickPlaying/_kickPlayingSince/_kickWindowEnd/_kickArmSince/_kickCooldownUntil/_kickSteerSign/_kickAskDegrees` (140-147) | no | yes 1774-1780 | no | fold on end 4508-4542 | OK (L always precedes the race) |
| `_stuckPhase/_stuckPhaseEndTime/_stuckRecoveryStartTime/_stuckStationarySince/_stuckOffTrackSince/_stuckAgainSince/_stuckExitSince/_stuckRecoveryCooldownEndTime/_stuckMoveSampleTime` (105-115) | no | yes 1813-1821 | no | 4263-4266, 4561-4568 | OK |
| `_stuckArmAllowedTime` (112) | no | **set** 1773 (`now + 15000`) | no | — | OK (armed by L, not by I) |
| **`Control.HandBrakeTime` (DS:340)** | **no** | **no** | **no** | only expiry; refreshed 4228, 4304 | **F2** |
| `Pressure` (513) | no | yes 1785 | no | falls 4295 | OK |
| `ActiveManeuver.Type/.Target/.LastEnabled` (478) | yes 652-654 | yes 1757-1758 | no | `EndCard` 483-489 | OK (fixed mid-review) |
| `_cardCooldownUntil[]` (479) | yes 655 | no | no | absolute, expires | OK (fixed mid-review) |
| `_avoidHoldActive/_avoidHoldLane/_avoidHoldUntil` (500-507) | no | yes 1804-1806 | no | 1269-1289, 4565 | OK |
| `_carriedLane` (179) | no | no | no | `_laneSnap` 1823 → snap at 1124-1127 | OK, by design |
| `_laneSnap` (205) | no | yes 1823 | no | consumed 1126 | OK |
| `_lineBiasNode/_lineCornerNode/_lineBiasMeters` (188-191) | no | yes 1808-1809 | no | re-rolled per apex 1015-1022 | OK |
| `_skippedApexes` (319) | yes 687 | no | no | removed on pass 3865/3896 | OK |
| `_cornerContexts` / `CornerContext.RequiresBraking` | yes 685-686 | no | no | one-way per race (3895) | OK for a race; a restart re-seeds it |
| `_brakeSample*` (320-339) | no | no | no | re-latched on apex change 2044-2069 | OK |
| `_brakeCommitApexNode` (341) | yes 688 | no | no | 2033, 2177 | OK |
| `Lap/LapTimes/LapStartTime/CanRegisterNewLap/_previousNode` (77-86) | yes 641-651 | partial 1772-1784 | no | 3707-3740 | OK |
| `RacePosition/FinalPosition/IsDNF/FinishedPointToPoint/BaseBehavior` (32-87) | yes 646-667 | partial | no | 4319-4331 | OK |
| `VehicleData.Peak*` (DS:62-65) | yes 643 | yes 1782 | no | per lap 3724/3734 | OK |
| **`_lastCoreTick` (98)** | **no** | **no** | no | 3408 | **F8** |
| **`_appliedThrottleLastFrame` (509)** | yes 638 | no | no | written 3147, never read | **F9** |
| `_isPassengerized` (414) | no | no | no | two-way 3441/3445 | OK |
| `_avoidWallsInitialized` (177) | no | no | no | one-shot 1400 | OK (per object) |
| `VehicleData.AccelSamples/Head/Count/LastAccelSampleTime`, `_lastSpeed` (DS:19-22, Racer:54) | no | no | no | ring buffer | OK in practice: a stale `LastAccelSampleTime` makes `dt` huge, so the first sample is ~0 and the window refills in 200 ms |

Per-race (`ARS` statics). C = `CleanEverything`, A = `OnAbort`, S = `SetupRace`/`StartRace`.

| field | C | A | S | verdict |
|---|---|---|---|---|
| `RaceStatus` (219) | 1822 | 3295 (finally) | 2536 | **F6**: `Finished` (2238) is written and never read |
| `PlayerParticipating` (126) | 1823 | 3296 | no (set true 2493) | **F1**-rank: `CleanRacers` (2470-2485) leaves it true |
| `ParkedDNFs` (224) | **1813 only** | no | no | **F7** |
| `LeaderboardFinish` (213) | 1812 | 3297 | 2513 | OK |
| `_raceTimedFinishMs` (275) | 1815 | 3298 | 2512, 2480, 827 | OK |
| `_countdown` (616) | 1824 | no | 2554, 2166 | OK |
| `_countdownTickMs` (614) | no | no | **2553** (rule differs from 2132) | **F4/F5** |
| `_trackInstanced/_gridInstanced` (230-231) | 1825-1826 | no (unload) | 2464-2465, 1449/1496, 1503 | OK (A irrelevant) |
| `_flareFx` (251) | 1809-1810 | 3264 (stops, never `Clear`) | no | OK (A irrelevant) |
| `Bumps/RouteNodes/NodeHalfWidths/EditNodeHalfWidths` | 1817-1820 | no | no | OK (A irrelevant; `TrackLoader` clears both Bumps and Crests on load, `TrackLoader.cs:266/527/705`) |
| `_routeEditorActive` (TrackCreator:14) | 1814 | 3299 | no | OK |
| `RaceStartTime` (222) | no | no | 2168 | OK: `RaceIsRunning`-gated readers, re-stamped when the countdown hits zero |
| `RaceReward` (117) | no | no | 2526 | OK (rewritten before it is paid, 2237) |
| `DebugFocusRacer/PlayerRacer` | no | no | 2196/2198, 1889 | OK (recomputed per tick) |

## Findings

### F1 (rank 1) — `PlayerParticipating` survives `CleanRacers`, and it is the only guard in front of `SetupRace` on the restart path
`src/AutosportRacingSystem.cs:2470-2485`
```csharp
public void CleanRacers()
{
    ...
    _nextInLine = 0;
    _raceTimedFinishMs = 0;
    _gameTimeNextInLine = 0;
    LeaderboardFinish.Clear();
    RaceStatus = RaceState.None;
    _gridInstanced = false;
}
```
`CleanEverything` clears the flag (`1823 PlayerParticipating = false;`) and so does `OnAbort` (`3296`); `CleanRacers` — which empties `Racers` on its own — does not. "Load Track" (`ARS:722-729` → `InstanceTrack()`, which calls `CleanRacers()` at `1429`) and "Build Grid" (`802-809` → `1467`) are menu items that can be pressed mid-race. After one, `Racers` is empty and `PlayerParticipating` is still true, so "Restart Race" skips its own guard:
`ARS:826` — `if (!PlayerParticipating) { StartRaceFromMenu(); return; }`
and takes `SetupRace(true, true)` (`828`), which reaches
`ARS:2524` — `Racer mostPower = Racers.OrderBy(v => Function.Call<float>(Hash.GET_VEHICLE_ACCELERATION, v.Car)).ToList()[0];`
on an empty list: `ArgumentOutOfRangeException`, caught by `OnTick`'s handler (`2259`) and logged as "OnTick error", with `RaceStatus` still `None` because `2536` is never reached.
Falsify: race once with the player, press "Load Track", press "Restart Race" — one logged `OnTick error`, `ArgumentOutOfRangeException` at `ARS:2524`.

### F2 (rank 2) — the grid handbrake is a latch that only a branch that stops running clears
`src/Racer.cs:4228`
```csharp
if (BaseBehavior == RacerBaseBehavior.GridWait && Control.HandBrakeTime < now) Control.HandBrakeTime = now + (100 * ARS.GetRandomInt(2, 6));
```
`src/Racer.cs:3463` — `Car.HandbrakeOn = Control.HandBrakeTime > Game.GameTime;`
`Launch()` resets fourteen `Control` fields and both reason pairs (`1789-1812`, `1822`) and does not touch `HandBrakeTime`:
```csharp
Control.MaxThrottleFromOffTrack = 1f;
Control.MaxThrottleFromRecovery = 1f;
Control.ThrottleReason = ThrottleReason.Plan;
...
Control.BrakeReasonLevel = 1f;
```
`Initialize` does not either (`636-658`), and neither does `Delete`. The GridWait writer refreshes the deadline every core tick, so at the instant `Launch` runs (from `ARS:2164`, between core ticks) the deadline is 200-600 ms in the future; `BaseBehavior` flips to `Race` at `1771`, the writer stops, and the car is held by the handbrake while the mod commands throttle — a random 0-600 ms handicap off the line for every AI car. The same field has a second writer with a different window, `Racer.cs:4304` (`Control.HandBrakeTime = Game.GameTime + 1000;` for a DNF car): one latch, two laws, and no path that clears it at a phase boundary.
Falsify: log `Control.HandBrakeTime - Game.GameTime` in `ApplyInputs` for the first 30 ticks of a race; non-zero handbrake with `BaseBehavior == Race` proves it.

### F3 (rank 3) — `ApplyNoCollision(false)` is called after the list it iterates has been emptied
`src/AutosportRacingSystem.cs:1804` then `1811`
```csharp
void CleanEverything()
{
    CleanRacers();
    ...
    _flareFx.Clear();
    ApplyNoCollision(false);
```
`ApplyNoCollision` walks `Racers` (`1778` — `for (int i = 0; i < Racers.Count; i++)`), and `CleanRacers` cleared it three lines earlier (`2477`). The undo therefore iterates zero racers and never disables a single `SET_ENTITY_NO_COLLISION_ENTITY` pair; `ApplyNoCollision(true)` was applied at `2169`/`2190`. This is a reset that is behind a statement that destroys the state it needs. World-side effect is bounded (the AI cars are deleted by `r.Delete()` inside `CleanRacers`, so the pairs die with them), but the player's car keeps its pair entries against handles the game is then free to recycle.
Falsify: count loop iterations in `ApplyNoCollision` at teardown — 0 at `1811`, N at `2169`.

### F4 (rank 4) — the countdown's out-of-race reset branch is dead: the subtraction is reversed
`src/AutosportRacingSystem.cs:2120`
```csharp
if (RaceStatus != RaceState.Countdown && _countdownTickMs - Game.GameTime > 5000) _countdown = _maxCountdown - 1;
```
Both writers put `_countdownTickMs` at or below `now + 1000` (`2132`, `2553`), so `_countdownTickMs - Game.GameTime` is ≤ 1000 and, once a tick has passed, negative. The branch can never run; `_countdown = _maxCountdown - 1` is never executed. Written as `Game.GameTime - _countdownTickMs` it would have meant what it says. This is the "guard on the wrong thing" shape: the sign is not the timer's.
Falsify: breakpoint on `2120` with the operands printed — the condition is false on every frame of a session.

### F5 (rank 5) — two writers to `_countdownTickMs` with two different rules; the countdown is a second short
`src/AutosportRacingSystem.cs:2553` vs `2132`
```csharp
// StartCountdown, 2553
_countdownTickMs = Game.GameTime;
// OnTick, 2132
_countdownTickMs = Game.GameTime + 1000;
```
`StartCountdown` is called from inside `OnTick` at `2125`/`2127`, i.e. before the `2130` test in the same frame; with the deadline set to *now* the guard `_countdownTickMs < Game.GameTime` is true on the very next frame, so the first decrement (7 → 6) lands ~16 ms after the countdown begins instead of 1 s after. `_maxCountdown` is 7 (`615`), so the sequence 6,5,4,3,2,1,GO lasts ~6 s. One timer, two rules: the writer that starts it uses "due now", the writer that advances it uses "due in one second".
Falsify: log `(Game.GameTime, _countdown)` from `2142`; the first `6` is within one frame of `StartCountdown`.

### F6 (rank 6) — `RaceState.Finished` is written and never read, and is overwritten two lines later
`src/AutosportRacingSystem.cs:2238-2239`
```csharp
RaceStatus = RaceState.Finished;
CleanEverything();
```
`CleanEverything` sets `RaceStatus = RaceState.None;` at `1822`, in the same statement list, so the value lives for one call. Grep for `RaceState.Finished` in `src/` returns exactly one hit — this write. Nothing can observe the finished state, so any future `RaceStatus == Finished` test would be dead on arrival. House rule 8 (a value written and never read) plus rule 1 (one law per value: five writers, `1822/2483/2536/3295/3936`).
Falsify: `grep -rn "RaceState.Finished" src/` → one line, a write.

### F7 (rank 7) — `ParkedDNFs` is reset on exactly one of the three ends
`src/AutosportRacingSystem.cs:1813` — `ParkedDNFs = 0;`
`src/Racer.cs:4313` — `int slot = ARS.ParkedDNFs++;`
The counter is the DNF parking slot, monotone for the life of the race and beyond: `CleanEverything` is its only zeroing site, and the restart path (`ARS:826-828` → `SetupRace`) never passes through it, nor does `StartRace` (`2424`) or `SetupRace` (`2509`). A race that ends by any route other than C leaves the counter advanced, so the next race's dead cars are parked further along the route (`4315` — `(slot * DNFSlotSpacingMeters) % nodeCount`, and the side alternates on `slot % 2` at `4321`), eventually at a different node than slot 0 and on the wrong side of the road for the slot the designers meant.
Falsify: race to two DNFs, "Restart Race", kill two cars — the second pair parks at slots 3/4, not 0/1 (log `slot` at `4313`).

### F8 (rank 8) — `_lastCoreTick` is seeded to a magic 100, so every racer's first core tick integrates ~10 s
`src/Racer.cs:98-99`
```csharp
int _lastCoreTick = 100;
int TimeSince_lastCoreTick => (int)ARS.Clamp(Game.GameTime - _lastCoreTick, 1, 9999);
```
`src/Racer.cs:2244` — `float TickScale => (0.001f * TimeSince_lastCoreTick);`
`_lastCoreTick` is only ever set to `Game.GameTime` at the end of a core tick (`3408`). It is not set by the constructor, not by `Initialize` (`609-691`), not by `Launch`, so on the first `RunTimedCore` (`ARS:2184`) `Game.GameTime - 100` clamps to 9999 and `TickScale` is 9.999 for every rate-based fold in that frame — e.g. `1912 _projectionOutsideSeconds = projectionOutside ? _projectionOutsideSeconds + TickScale : 0f;` takes a 10-second step, and `2311 _cornerLiftMps += ... * TickScale;`. Most consumers are clamped or recomputed next frame, and `_projectionOutsideSeconds`/`_cornerLiftMps` reset the moment their condition flips, so the honest blast radius is one bad frame; the defect is that a timer whose whole job is "how long since the last tick" is initialised to a constant that is not a time.
Falsify: log `TickScale` in `ComputeSteering` on the first tick after a racer is constructed — 9.999, not ~0.03.

### F9 (rank 9) — `_appliedThrottleLastFrame` is written and never read
`src/Racer.cs:3147`
```csharp
_appliedThrottleLastFrame = VehicleMemory.GetThrottle(Car);            if (!ControlledByPlayer)
```
Declaration `509`, initialised `638`, assigned again every `ProcessTick` — and no reader anywhere (`grep -n _appliedThrottleLastFrame src/Racer.cs` → three hits, all writes). House rule 8. The missing newline also shows the write was appended to an existing line, which is where "written for a reason that never landed" usually comes from.
Falsify: the same grep; three hits, no read.

## Nitrous verdict (prior audit finding)

**Settled: fixed.** `Racer.Delete` today tests presence, not liveness:

`src/Racer.cs:4692` — `if (_nitrousActiveUntil > 0) StopNitrous();`

`StopNitrous` (`3042-3050`) is idempotent in the sense the prior finding assumed: it re-issues the `OVERRIDE_NITROUS_LEVEL` release with `false` and zeroes the field (`3049 _nitrousActiveUntil = 0;`), so calling it for a burn that expired between the racer's last core tick and teardown is safe and necessary. The `> GameTime` form would have missed exactly that window; `> 0` closes it. The in-race expiry path is also intact: `2795 if (_nitrousActiveUntil > 0 && (!ARS.NitrousEnabled || Game.GameTime >= _nitrousActiveUntil)) StopNitrous();`.

One residual, which I judge **by design**: `Initialize` resets `_nitrousLapUsed` (`645`) but not `_nitrousActiveUntil`, so a "Restart Race" during a live burn keeps the boost until it expires (≤ 3 s). Nothing else reads it into the new race: `ApplyPowerMultiplier` (`3503`) multiplies power only when throttle is commanded, and the grid is held at `Control.Throttle = 0`, and `Delete` — the only path that can leave a *native* override applied to a car that survives — is called before any new race except the restart, which never deletes. Not reported as a finding for that reason.

## What I could not decide

1. **Whether F3 has a world-side victim.** `SET_ENTITY_NO_COLLISION_ENTITY` pairs are per (entity, entity); the AI cars are deleted inside `CleanRacers`, so the pairs are moot unless GTA V recycles those handles to new entities while the player's car is still in the world. I did not have a way to observe handle recycling without running the game, so I state the defect (the undo never executes) and not the symptom.
2. **`CornerContext.RequiresBraking` is a one-way latch per race** (`DataStructures.cs:412`, set at `Racer.cs:3895`, never set back to `false`; only `_cornerContexts.Clear()` at `Racer.cs:685` re-seeds it). Whether a corner that once needed braking can stop needing it inside the same race — after a tyre/grip change, or on the second lap at a different speed — I could not settle from the code; the surrounding comment ("A corner that asks for nothing gets no card", `Racer.cs:376-379`) reads as if the answer is meant to be "no".
3. **`Rival` fields outlive their rival.** `UpdateRivals` nulls `RivalRacer` for every slot (`4655`/`4673`) but leaves `Distance`, `RelativeOffset`, `RouteGapAhead`, `DirectionDiff` holding the last rival's numbers. Every reader I found filters `RivalRacer == null` first (`NearestRival` `2648`, `2375`, `2364`, `2870`), so I found no reachable stale read — but the type documents `Distance = 99`, `DirectionDiff = 99f`, `TimeToReach = +inf` as "no rival", and nothing restores those sentinels, which means the invariant is enforced only by caller discipline.
4. **`Racer.FinalPosition`/`RacePosition` are written only by `ARS`** (`2208-2209`, `2228`, `2236`) while `Racer` reads them (`454`, `2417`). That is one writer for the pair, so it is not a house-rule-1 violation, but ranking depends on `RacePosition` being non-zero, and during the first 200 ms of a race it is 0 for everyone (`ARS:2217-2220`); `ComputeRubberBandFactor` bails on `<= 0` (`2411`), `RivalsWithinDistance` does not (`454` — every `r.RacePosition < 0` is false). I could not show that this changes a decision, so I am not filing it.
