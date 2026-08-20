# ARS — GTA V Racing Mod

Source: `F:\Archivos Seguros\Mis Archivos\Codigo\GTAV\NewRacingSystem`
Target: C# / .NET Framework 4.8 / ScriptHookVDotNet 2

## Persistent cross-session memory for the agent
**This file is the agent's long-term memory.** The agent has no memory between sessions, so this doc is the durable record it must rely on. **Aggressively save durable memories here**: quirks of the code, non-obvious design decisions and their *why*, high-level concepts, invariants, override order, workflow rules, architecture intent. **Never** record volatile things like exact numeric values/constants/thresholds/knob names that get tuned constantly — the code is the source of truth for those. When a meaningful durable fact is established, write it here (or append to the relevant section) before the session ends. When uncertain from memory, read this file first.

## Writing this file — no specific values
Constants, thresholds, conditions, windows and knob names get tuned constantly and go stale. Keep this doc to **stable architecture**: structure, override order, invariants, and design intent (the *why*, not the *what number*). The code is the source of truth for specifics — describe behavior, don't enumerate current values.

## Workflow
- **Every file change → compile (with autocopy) → user verifies in-game → then commit.** Never commit before the human confirms the change works. The human is the gatekeeper for verification; do not treat a successful compile as "verified."
- **Documentation-only changes:** when a change is strictly comments or other non-executable documentation and the build passes, in-game verification is not required before committing. Git is the backup while the work remains undistributed.
- **The project auto-copies on build** (`NewRacingSystem.csproj` has both a `PostBuildEvent` and a `CopyArsDll` target): every build drops `ARS.dll` into `D:\SteamLibrary\...\Scripts\ARS\`. **Quirk:** both Debug and Release fire the copy, so *whichever configuration builds last wins* in the game folder. When you need Release deployed, run Release last.
- Build (MSBuild, Release): `& "C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe" NewRacingSystem.csproj /v:minimal /nologo /p:Configuration=Release`
- Build (dotnet, agent-tuned — suppresses 1300+ SHVDN2 deprecation warnings, errors only): `dotnet build "F:\Archivos Seguros\Mis Archivos\Codigo\GTAV\NewRacingSystem\NewRacingSystem.sln" /target:NewRacingSystem /property:GenerateFullPaths=true /p:Configuration=Debug /p:Platform="Any CPU" /consoleloggerparameters:ErrorsOnly`
- Build output used by the game: `D:\SteamLibrary\steamapps\common\Grand Theft Auto V\Scripts\ARS\`
- Crash log: `D:\SteamLibrary\steamapps\common\Grand Theft Auto V\ScriptHookVDotNet2.log`
- Sub-200 ms lags don't matter: one-frame ordering quirks are acceptable — don't restructure call order or add plumbing just to kill them.
- **Avoidance experiments checkpoint:** `3087663` — last commit before further avoidance/speed tuning experiments. Roll back here if experiments go sideways.
- **Corner experiments checkpoint:** `54b33a7` — last commit before the **live corner creation** experiments. Known-good baseline (hill grip-loss exponential, radius-mapped crest aggression, later apex gate, power-matched grid). Roll back here if live-corner work goes sideways.
- **Gs-aware preview checkpoint:** tag `checkpoint-pre-gs-aware` (commit `183aeda`) — state right before the Gs-aware preview steering experiment.

## Dependencies and UI
- LemonUI SHVDN2 replaces the hand-drawn menu. Root actions: Start Race and Freecam; the race setup area exposes track selection, target grid size, power target, and power bracket; Debug submenu exposes debug toggles. If available, LSIA Test Track is selected as the default track.
- Reference: `C:\Users\Usuario\Downloads\LemonUI\SHVDN2\LemonUI.SHVDN2.dll`; deployed beside `ARS.dll`.
- The menu opens through the existing Sprint + Context hotkey or `arsmenu`; LemonUI controls navigation/cancel/input suppression.

## Code map — the files
- `AutosportRacingSystem.cs` — the big orchestration file: race setup/flow, track/corner generation, grid, leaderboard, helpers (`Remap`/`Clamp`/`Circumradius`, native-call wrappers), `FindNextCorner` (live rolling scan)/`CornerApexSpeed`/`MaxSpeedForBrakingDistance` (static AI math that Racer calls).
- `Racer.cs` — per-car intelligence: the whole steering/speed pipeline (`ComputeSteering`, `ComputeTargetSpeed`, `ComputeHighSpeedLane`, `ComputeCornerTargetLane`, `ComputeAvoidAheadLane`, `ApplyRivalWalls`, `ApplySteerLimits`, `ConvertSpeedToPedals`, `TranslateSteerToInput`), plus pressure, maneuvers, traction control, stuck recovery, debug drawing.
- `DataStructures.cs` — shared state: `RacerBrain` (perception, corner, rivals, intention), `Rival`, `TrackPoint`, `CornerPoint`/`Corner`, `VehicleControl`, `VehicleState`, `HandlingData`.
- `PersonalitySet.cs` / `SkillSet.cs` — leftover per-racer personality scaffolding.

## The per-frame pipeline (Racer, timed core)
Rough order, each step consuming the last:
1. `UpdateTrackPosition` — local ±6-node search (global rescan past ~25 m), builds time-based lookaheads (0.25s→2s), computes the route radii, ticks laps.
2. `ComputeTargetSpeed` — builds the **intended speed** (see Speed pipeline) and sets `_cornerSpd` = current corner's apex speed.
3. `ComputeSteering` — resolves the lane (see Lane systems), derives heading/lane/recovery angles, runs the PD to `Control.SteerDegrees`. **The heading-error term is load-bearing — do not remove or merge it into the lane term.** It anchors the car to face the track direction, which is what catches lateral-drift overshoot at the lane center. Merging heading + lane into a single "aim at the point" angle was tried and reverted: without a separate heading term the car sails past the lane and oscillates.
4. `ApplySteerLimits` — speed-based clamp on steer (only when steer and yaw share sign).
5. `ConvertSpeedToPedals` — the speed loop: intended speed → throttle/brake, clamped by the caps.
6. `TranslateSteerToInput` — smooths steer toward the target and remaps degrees → -1..1 input.
7. Traction control, maneuvers, stuck recovery run around these.

## Lane systems — where on the track the car aims
Resolved in `ComputeSteering` in this override order (later overrides earlier):
1. **System 2 — high-speed line (`ComputeHighSpeedLane`)**: always runs. Positions the car on the **inside edge of the track's own curvature**, derived purely from track geometry (signed angle over ±20 nodes, small deadzone) — fully independent of the corner system. Disabled past a radius ceiling (wider = effectively straight).
2. **System 1 — corner outside approach (`ComputeCornerTargetLane`)**: temporary override. Holds the **outside line** only during the approach window before a key corner's apex, then lets go (`0f`) so System 2's inside takes over naturally.
3. **Avoidance (`ComputeAvoidAheadLane`)**: overrides both when a rival ahead needs passing.
4. **`ApplyRivalWalls`**: clamps the final lane between track bounds and rival walls. **Invariants: the car's full width always stays on track; the buffer always includes the average of both cars' bounding boxes (no physical contact possible) + an aggression extra.** Overlap classification uses an asymmetric longitudinal window by design: more margin from a rival ahead (the car moves off them earlier), nearly none from one just passed (it crosses back over to the racing line as soon as it's clear).

Design rules:
- **Gain discipline**: each system has a pursuit gain (avoidance fixed; corner systems radius-scaled). The gain scales how *hard* the target is pursued — the *target itself* is always a track edge. Aggressive steer-**out** is dangerous, so the outside approach is deliberately gentler than the inside commit.
- **Corner approach**: only engages if the car entered the window needing to brake (entry latch); let-go time scales with track width (wider track → release earlier, more time to cross to the inside).
- **Corner-commit maneuvers** (Divebomb / DefendLane): inside the outside-approach window they replace the outside hold with a lane right beside the target rival on the corner's inside — the divebomber to out-brake an ahead rival, the defender to block a behind rival from diving. Same lane formula, opposite target frame; both clean up when their armed apex is passed (defend also drops on overtake).
- The corner line phases are a **hard switch** (outside → inside), no lerp.

## Gs-aware preview steering (lane error blend)
- **The concept**: pure-pursuit lane steering measures the lane error at the *car's current position*, implicitly assuming the path follows the nose. At speed, centrifugal force (already present in `ProjectAhead`, whose acceleration term is measured, not commanded) curves the real path, so nose-aimed steering overshoots. The preview fix: measure the lane error at the **1s Gs-aware projection** instead — steering then anticipates centrifugal drift rather than reacting to it.
- **Implementation** (in `ComputeSteering`'s lane-bias block): the lane error is a blend of the legacy error (measured at the car) and the preview error (measured at `ProjectAhead(1f)` against the `LookAhead.OneSec` node frame). Blend 0 = exact legacy behavior (the toggle is the A/B switch); blend 1 = full preview. The `atan2` lookahead denominator and the `_laneGainDivisor` exp-gain curve are shared by both — untouched.
- **Menu**: Debug submenu — "Gs-Aware Preview" checkbox (`Options.GsAwarePreview` in `DebugToggles`) plus a "Preview Blend" list item (steps between 0.1 and 1.0) writing `ARS.GsAwarePreviewBlend`. Default off.
- The projection read in steering is one frame stale (`RunTimedCore` runs before `ProcessTick` refreshes `AccelerationVector`) — same staleness ConfidenceMPS lives with; acceptable by design.

## Speed pipeline — how fast the car goes
- **`cornerSpd`** — the "ballpark": kinematic braking plan `√(v²+2·a·d)` targeting the **apex speed**, distance to the **apex node**, plus a flat +5 m/s offset.
- **`followTrackSpd`** — the "precise": route curvature `√(g·grip·radius)` from the speed-based lookahead window, plus a flat +5 m/s offset and pressure overspeed.
- **Slope grip loss is TEMPORARILY DISABLED** for speed tuning. The slope-grip factor (applied to corner and route speed) is commented out in `ComputeTargetSpeed`. Re-enable when tuning is settled. The crest/dip vertical curvature factor (route speed only) is re-enabled.
- **Hill/crest grip floors**: `HillGripMin` and the crest/dip vertical grip factors are all floored at 0.8 — hills and crests can never remove more than 20% of grip. (Previously could zero out corner speed at sharp crests.)
- **`Intention.Speed` = min(cornerSpd, followTrackSpd)**, then clamped by engine top speed, `MaxSpeed`, and `_speedCap`. The pressure speed bias (a post-min `±2 m/s` nudge) is **commented out** for late-braking isolation testing — re-enable when root cause is confirmed.
- **Pedal gain**: speed error (in Gs) → pedals uses `* 1f` (1 G = full throttle/brake). Was `* 5f` (0.2 G saturated) — the old gain caused over-braking.
- **Two cap domains, deliberately split:**
  - `_accelerationCap` (input domain, ±1): throttle/brake scalar from avoidance (rival-distance smooth map). Lifts/brakes proportionally; only lowers via `Math.Min`.
  - `_speedCap` (speed domain, m/s): the projection off-track source pins it at most to the corner speed, then deducts **incrementally** (per-second rate, not an instant pin) so a single frame of air/off-track doesn't slam the cap.
- **Asymmetry is by design**: corner is the loose target, route is the tight one. Do not add flat offsets to `followTrackSpd` to "balance" them.
- **High-grip cars don't respond to braking-side tuning**: their braking is so strong that the braking plan rarely binds — route-curvature speed control wins. Adjustments that act on the braking plan barely register on them; tune the route-speed side instead.

## Corner lifecycle (precomputed apex table + four held targets)
1. **Apex table at generation time** (`BuildApexCorners`, end of `GenerateRouteInfo`): a second pass walks the track **sequentially in chunks**; in each chunk the node with the **smallest precise curve radius** is the chunk's apex, kept **only if its radius is under a corner limit**. Each `CornerPoint` in `ARS.Corners` carries only the node, that radius, and a tentative 1g corner speed (`√(g·r)`). Nothing else — no spans, angles, thresholds. (Earlier local-minimum criteria over ±1 then ±4 neighbors produced ~every node as a corner on noisy recorded tracks; the chunked pass fixed it.)
2. **Held apex queue per racer** (`UpdateNextApexes`): targets persist until passed, then leapfrog forward and refill by scanning after the last held apex. Redundant close apexes are skipped unless materially slower; the queue invalidates when route data changes or the car suffers a low-speed incident. Per-car apex speed = `√(grip · gravity · radius)`.
3. **Braking map (ApexBrakingSpeed)**: each held apex feeds the kinematic map `v = √(vApex² + 2·a·d)` over the usable distance (apex distance minus a buffer of `ApexBufferSeconds` × corner speed). `cornerSpd = min(map(apex1..4))` — whichever corner demands the lowest speed wins. The buffer is pressure-scaled (0 pressure = brake earlier, 100 = later) and divebomb shortens it further.
4. **Route-speed split**: the near window (three sample points car+velocity÷grip / midpoint / car+(velocity×3)÷grip, circumradius through them) gauges `followTrackSpd` and is the authority for **sweeping corners**; the four-apex braking map handles tight ("killer") corners from far away. `Intention.Speed = min(cornerSpd, followTrackSpd)`. The route speed is **no longer gated off** near the apex — both speeds always run and the min governs (the old 2s gate caused unawareness in complex curves).
5. The old live per-racer rolling scan (`FindNextCorner`, `Brain.Corner`) and the far-killer 4s→8s window scan are **superseded and dormant** — `Brain.Corner` consumers (outside-approach lane, divebomb/defend, chevrons) still read it when a corner is instanced, but nothing instances corners anymore.

Track facts: **1 node = 1 m**. Circuit lookaheads use modulo; point-to-point clamps.

## Grid car selection (power-matched)
- **Vehicle stats can be read from the model without spawning**: natives like `GET_VEHICLE_MODEL_ACCELERATION` (power), `GET_VEHICLE_MODEL_MAX_TRACTION` (grip), and the estimated-max-speed native take a **model hash** (as opposed to the spawned-handle versions, e.g. `GET_VEHICLE_ACCELERATION` / `_0x53AF99BAA671CA47`). This is what makes pre-race vehicle evaluation cheap — no car needs to exist yet.
- **Model-power cache (`ModelPowerCache`)**: scans the `Vehicles\*.xml` pool for which models exist (no natives on the background thread), then `BuildPowerCache()` runs on the **main script thread** after loading and fills model → power via the native. **GTA natives must not be called on the background load thread (`Task.Run`) — that hard-crashes the game**; always do native-per-car reads on the main thread. Grid selection later is a pure cache lookup — no natives called at race start for the matched set.
- **Selection** (`FillCachedCandidates`): candidates are kept if their cached power falls within the user-selected power bracket around the user-selected power target; the qualified pool is shuffled and trimmed to the requested grid size. The old Disciplines tag filter is bypassed so the whole `Vehicles\` pool is considered. Watch the log for the candidate count to debug the band width.
- The leaderboard displays each car's combined power scale rather than the former three-part performance string. It uses the same model acceleration plus scaled estimated-top-speed formula as grid selection.

## Per-racer state
- **Aggression** (0–100): grid-assigned (first=0 … last=100, rounded to 10); player stays 50. Scales avoidance extra buffer and allowed TCS wheelspin.
- **Pressure** (0–100): pure proximity × aggression — target ramps from the nearest racer within 100 m; rises toward target, falls quickly. Adds a fixed overspeed to `followTrackSpd`.
- **Maneuvers** (`Maneuver` on the racer): one-off tactical behaviors with a periodic arm check and a fixed lifecycle (arm → act → off). Nitrous (temporary speed boost), Yield (lift off while trailing a faster rival near a corner), Divebomb (pressure-driven commit to the inside of a rival ahead at a corner), and DefendLane (cover the inside against a rival behind so they can't dive — they take the corner wide).
- **Passengerize**: AI drivers shift to the passenger seat while a rival overlaps within combined half-length + 0.5 m, preventing contact swerves. Never applied to the player.
- Ghosting was removed during avoidance cleanup; rebuild later with the avoidance system.

## Per-racer randomization
- **`_laneGainDivisor`** (90–110): set once in the constructor at spawn. Scales the lane-pursuit gain curve's input — a lower divisor means the car saturates the gain at a smaller degree error (snappier lane pursuit), a higher divisor means it needs more error to max out (lazier). Subtle per-car variation so the field doesn't all steer identically.

## Debug (LemonUI Debug submenu)
- **ShowInputs** — per-car speed readout (current/intended, corner/route speed, `_speedCap`/`_accelerationCap`/`_confidenceMPS`) + pedal trail.
- **ShowTrackAnalysis** — lane-aim spheres to the final target, track-ahead radius, wall lines, corner chevrons.
- **ShowAggro** — pressure chevron/text + 0.5s and 1s projections (white line/sphere, red when off-track).
- **ShowPhysics** — G-force sphere/vector.

## Durable gotchas — do not "fix" these
- **Remap with a descending output range + `clamp=true` is inverted** by `Clamp(val, min, max)` when `min > max` (NaN also compares less-than-anything). Keep output clamps ascending — use a descending *input* range when you want a reversed map. (This bit us repeatedly.)
- **NaN discipline**: `Clamp(NaN, -limit, +limit)` returns the min bound → instant full-lock. Guard steering outputs and any clamp input that can be non-finite.
- **Sub-200 ms lag** acceptable; don't add plumbing for one-frame quirks.
- Synchronous setup/loading work can pause `OnTick` and temporarily suppress per-frame debug visuals; revisit those operations later if setup stalls need to become incremental.
- **Speed asymmetry** (above) is intentional.

## Known TODOs / open items
- Walls-collapsed avoidance (empty TODO in `ApplyRivalWalls`) — fold into `_accelerationCap` as a separate source if needed.
- Two closely spaced corners where the second is slower — **now handled by design**: the live scan skips everything within the first 5s after an apex (exit-zone rule), so a close second corner is never instanced as a separate target.
- Corner hugging (inside-line hold through the apex) still being evaluated.
- Pressure-driven lookahead: reverted; hardcoded window pending investigation.
- Grid car selection: **now implemented as power-matched selection** (see the "Grid car selection" section below). See `grid_rework.md` for the earlier performance-bracket brainstorm.
- **Stability awareness**: **now implemented** — two rules: (1) if both left or both right wheels are off the ground, steer into that side to regain all four wheels; (2) if not all wheels are on the ground (3 Hz check), reduce `MaxThrottle` at 0.5/s (floors at 0.1, recovers on its own at 2/s). Also: if steer angle exceeds the grip-based limit by 10º, reduce `MaxThrottle` at 1/s (same floor/recovery).
- **Stuck recovery**: the old velocity punt (setting `Car.Velocity` to throw the car toward the track) is **replaced** with a smooth position lerp to the track edge over 1.5s (smoothstep). Triggers on the 3rd+ recovery attempt instead of the punt.
- **TCS**: simplified to a P-controller on `MaxThrottleFromTCS` targeting ideal wheelspin (-1). Max 0.5 on-track, 0.15 off-track (`OutOfTrackDistance() > 0`). Wheelspin is signed: negative = spin, positive = lockup.
- **Projection off-track throttle kill**: **REMOVED** — replaced by ConfidenceMPS (see below).
- **ConfidenceMPS**: a drifting speed bias added to `Intention.Speed` right after `min(cornerSpd, followTrackSpd)`. It evaluates short-horizon projections against the track edges and only applies when lateral load and speed-error stability indicate that confidence is appropriate. An off-track projection independently caps throttle as an emergency response.
- **Steering limit**: multiplier mapped to throttle (0 throttle = 1.0, full = 0.8). Lateral traction curve floor at 0.5.
- **Prevent rear-ends**: cars need to brake before hitting the car ahead from behind, and avoid leaning on each other — specifically, the *inside* car should brake to close its own trajectory in (don't rely on the outside car to open up). No implementation yet.
