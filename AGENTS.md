# ARS — GTA V Racing Mod

Source: `F:\Archivos Seguros\Mis Archivos\Codigo\GTAV\NewRacingSystem`
Target: C# / .NET Framework 4.8 / ScriptHookVDotNet 2

## Writing this file — no specific values
Constants, thresholds, conditions, windows and knob names get tuned constantly and go stale. Keep this doc to **stable architecture**: structure, override order, invariants, and design intent (the *why*, not the *what number*). The code is the source of truth for specifics — describe behavior, don't enumerate current values.

## Workflow
- Compile, test, then commit each logical change. Ask before pushing.
- Build: `& "C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe" NewRacingSystem.csproj /v:minimal /nologo /p:Configuration=Release`
- Build output used by the game: `D:\SteamLibrary\steamapps\common\Grand Theft Auto V\Scripts\ARS\`
- Crash log: `D:\SteamLibrary\steamapps\common\Grand Theft Auto V\ScriptHookVDotNet2.log`
- Sub-200 ms lags don't matter: one-frame ordering quirks are acceptable — don't restructure call order or add plumbing just to kill them.

## Dependencies and UI
- LemonUI SHVDN2 replaces the hand-drawn menu. Root actions: Start Race and Freecam; Debug submenu exposes debug toggles.
- Reference: `C:\Users\Usuario\Downloads\LemonUI\SHVDN2\LemonUI.SHVDN2.dll`; deployed beside `ARS.dll`.
- The menu opens through the existing Sprint + Context hotkey or `arsmenu`; LemonUI controls navigation/cancel/input suppression.

## Code map — the files
- `AutosportRacingSystem.cs` — the big orchestration file: race setup/flow, track/corner generation, grid, leaderboard, helpers (`Remap`/`Clamp`/`Circumradius`, native-call wrappers), `GetNextCorner`/`FindNextCorner`/`CornerApexSpeed`/`MaxSpeedForBrakingDistance` (static AI math that Racer calls).
- `Racer.cs` — per-car intelligence: the whole steering/speed pipeline (`ComputeSteering`, `ComputeTargetSpeed`, `ComputeHighSpeedLane`, `ComputeCornerTargetLane`, `ComputeAvoidAheadLane`, `ApplyRivalWalls`, `ApplySteerLimits`, `ConvertSpeedToPedals`, `TranslateSteerToInput`), plus pressure, maneuvers, traction control, stuck recovery, debug drawing.
- `DataStructures.cs` — shared state: `RacerBrain` (perception, corner, rivals, intention), `Rival`, `TrackPoint`, `CornerPoint`/`Corner`, `VehicleControl`, `VehicleState`, `HandlingData`.
- `PersonalitySet.cs` / `SkillSet.cs` — leftover per-racer personality scaffolding.

## The per-frame pipeline (Racer, timed core)
Rough order, each step consuming the last:
1. `UpdateTrackPosition` — local ±6-node search (global rescan past ~25 m), builds time-based lookaheads (0.25s→2s), computes the route radii, ticks laps.
2. `ComputeTargetSpeed` — builds the **intended speed** (see Speed pipeline) and sets `_cornerSpd` = current corner's apex speed.
3. `ComputeSteering` — resolves the lane (see Lane systems), derives heading/lane/recovery angles, runs the PD to `Control.SteerDegrees`.
4. `ApplySteerLimits` — speed-based clamp on steer (only when steer and yaw share sign).
5. `ConvertSpeedToPedals` — the speed loop: intended speed → throttle/brake, clamped by the caps.
6. `TranslateSteerToInput` — smooths steer toward the target and remaps degrees → -1..1 input.
7. Traction control, maneuvers, stuck recovery run around these.

## Lane systems — where on the track the car aims
Resolved in `ComputeSteering` in this override order (later overrides earlier):
1. **System 2 — high-speed line (`ComputeHighSpeedLane`)**: always runs. Positions the car on the **inside edge of the track's own curvature**, derived purely from track geometry (signed angle over ±20 nodes, small deadzone) — fully independent of the corner system. Disabled past a radius ceiling (wider = effectively straight).
2. **System 1 — corner outside approach (`ComputeCornerTargetLane`)**: temporary override. Holds the **outside line** only during the approach window before a key corner's apex, then lets go (`0f`) so System 2's inside takes over naturally.
3. **Avoidance (`ComputeAvoidAheadLane`)**: overrides both when a rival ahead needs passing.
4. **`ApplyRivalWalls`**: clamps the final lane between track bounds and rival walls. **Invariants: the car's full width always stays on track; the buffer always includes the average of both cars' bounding boxes (no physical contact possible) + an aggression extra.**

Design rules:
- **Gain discipline**: each system has a pursuit gain (avoidance fixed; corner systems radius-scaled). The gain scales how *hard* the target is pursued — the *target itself* is always a track edge. Aggressive steer-**out** is dangerous, so the outside approach is deliberately gentler than the inside commit.
- **Corner approach**: only engages if the car entered the window needing to brake (entry latch); let-go time scales with track width (wider track → release earlier, more time to cross to the inside).
- The corner line phases are a **hard switch** (outside → inside), no lerp.

## Speed pipeline — how fast the car goes
- **`cornerSpd`** — the "ballpark": kinematic braking plan `√(v²+2·a·d)` targeting the **apex speed**, distance to the **apex node**, plus slope-grip loss and a flat offset.
- **`followTrackSpd`** — the "precise": route curvature `√(g·grip·radius)` from the speed-based lookahead window, plus slope-grip, crest/dip, and pressure overspeed.
- **`Intention.Speed` = min(cornerSpd, followTrackSpd)**, then clamped by engine top speed, `MaxSpeed`, and `_speedCap`.
- **Two cap domains, deliberately split:**
  - `_accelerationCap` (input domain, ±1): throttle/brake scalar from avoidance (rival-distance smooth map). Lifts/brakes proportionally; only lowers via `Math.Min`.
  - `_speedCap` (speed domain, m/s): the projection off-track source pins it at most to the corner speed, then deducts **incrementally** (per-second rate, not an instant pin) so a single frame of air/off-track doesn't slam the cap.
- **Asymmetry is by design**: corner is the loose target, route is the tight one. Do not add flat offsets to `followTrackSpd` to "balance" them.
- **High-grip cars don't respond to braking-side tuning**: their braking is so strong that the braking plan rarely binds — route-curvature speed control wins. Adjustments that act on the braking plan barely register on them; tune the route-speed side instead.

## Corner lifecycle
1. **Generation** (AutosportRacingSystem): scan nodes → radius below the candidate threshold marks key candidates → second pass demotes any key corner closer than **full track width × 10** to its predecessor (uses the wider of the pair) → spans + min precise radius computed per corner. Spans are for radius scan/raceline/debug only — not culling.
2. **Targeting**: `FindNextCorner` resolves the first key corner after the current node every timed core (no speed/range gate); circuits wrap; braking distance wraps across the seam.
3. **Approach**: 5s window; outside hold (System 1) → let-go → inside (System 2) through the apex.
4. **Apex**: `GetNextCorner` flips to the next key corner once the apex is passed — the corner speed guard releases here (relevant to blow-through behavior in packs).

Track facts: **1 node = 1 m**. Circuit lookaheads use modulo; point-to-point clamps.

## Per-racer state
- **Aggression** (0–100): grid-assigned (first=0 … last=100, rounded to 10); player stays 50. Scales avoidance extra buffer and allowed TCS wheelspin.
- **Pressure** (0–100): pure proximity × aggression — target ramps from the nearest racer within 100 m; rises toward target, falls quickly. Adds a fixed overspeed to `followTrackSpd`.
- **Maneuvers** (`Maneuver` on the racer): one-off tactical behaviors with a periodic arm check and a fixed lifecycle (arm → act → off). Nitrous (temporary speed boost), Yield (lift off while trailing a faster rival near a corner), and Divebomb (pressure-driven commit to the inside of a rival ahead at a corner).
- **Passengerize**: AI drivers shift to the passenger seat while a rival overlaps within combined half-length + 0.5 m, preventing contact swerves. Never applied to the player.
- Ghosting was removed during avoidance cleanup; rebuild later with the avoidance system.

## Debug (LemonUI Debug submenu)
- **ShowInputs** — per-car speed readout (current/intended, corner/route speed, `_speedCap`/`_accelerationCap`) + pedal trail.
- **ShowTrackAnalysis** — lane-aim spheres to the final target, track-ahead radius, wall lines, corner chevrons.
- **ShowAggro** — pressure chevron/text + 1s projection (white line/sphere, red when off-track).
- **ShowPhysics** — G-force sphere/vector.

## Durable gotchas — do not "fix" these
- **Remap with a descending output range + `clamp=true` is inverted** by `Clamp(val, min, max)` when `min > max` (NaN also compares less-than-anything). Keep output clamps ascending — use a descending *input* range when you want a reversed map. (This bit us repeatedly.)
- **NaN discipline**: `Clamp(NaN, -limit, +limit)` returns the min bound → instant full-lock. Guard steering outputs and any clamp input that can be non-finite.
- **Sub-200 ms lag** acceptable; don't add plumbing for one-frame quirks.
- **Speed asymmetry** (above) is intentional.

## Known TODOs / open items
- Walls-collapsed avoidance (empty TODO in `ApplyRivalWalls`) — fold into `_accelerationCap` as a separate source if needed.
- Two closely spaced corners where the second is slower — planner only targets the first key corner.
- Corner hugging (inside-line hold through the apex) still being evaluated.
- Pressure-driven lookahead: reverted; hardcoded window pending investigation.
- Grid car selection: see `grid_rework.md` (performance-bracket brainstorm, not implemented).
- **Stability awareness**: wheels off the ground = unstable → the car should drive more carefully (lift-off/unweighting should bias speed down beyond the current crest/dip factor). No implementation yet.
- **Prevent rear-ends**: cars need to brake before hitting the car ahead from behind, and avoid leaning on each other — specifically, the *inside* car should brake to close its own trajectory in (don't rely on the outside car to open up). No implementation yet.
