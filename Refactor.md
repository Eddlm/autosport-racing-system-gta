Good. Let's dissect from smallest to biggest change, with risk/effort notes. I'll list them in ascending order of invasiveness. You pick which ones you want.

## 1. Cache `Model.GetDimensions()` per racer
**Size:** tiny, ~3 lines.
**Where:** `Racer` constructor, store `Vector3 ModelDimensions`.
**Used in:** `UpdateRivalInfo`, `ComputeSecondsToHit`, `UpdatePassengerSeat`, stuck checks.
**Risk:** very low — pure read-only cache of a static value.
**Win:** small but removes repeated native-ish model lookups.

## 2. Replace `TrackPoints.OrderBy(...).First()` in `ComputeOffshootSpeedCap`
**Size:** small, ~10–15 lines.
**Where:** `Racer.cs` line 843.
**Current:** full sort of entire track to find nearest node.
**Replace:** local ±N node scan like `UpdateTrackPosition` already does.
**Risk:** very low — same result, faster.
**Win:** removes O(N log N) per core tick.

## 3. Add `_phaseOffsetMs` and de-clump 0.5s/1s gates
**Size:** small, ~10 lines.
**Where:** `Racer` constructor + `ProcessTimedAI`.
**Current:** `_halfSecondTick`, `_oneSecondTick` reset to `Game.GameTime + interval`, so all cars fire together.
**Change:** reset to `Game.GameTime + interval + _phaseOffsetMs`.
**Risk:** low — just spreads the same work across frames.
**Win:** prevents frame spikes when all cars hit the same tick.

## 4. Gate `UpdatePressure` at 0.5s
**Size:** small, ~5 lines.
**Where:** `ProcessAI` call to `UpdatePressure`.
**Risk:** low — slow integrator, 0.5s is invisible.
**Win:** removes O(N) per core tick per car.

## 5. Gate `UpdateRivalInfo` at 0.5s
**Size:** small, ~5 lines.
**Where:** `ProcessAI` call to `UpdateRivalInfo`.
**Risk:** low-medium — avoidance reads this data, but 0.5s staleness is acceptable (sub-200ms, already tolerated per AGENTS.md).
**Win:** removes the dominant per-core-tick cost (swept-box math per rival).

## 6. Gate `UpdateNextApexes` / `UpdateTrackPosition` expensive tail at 0.5s
**Size:** medium, ~20–30 lines.
**Where:** split `UpdateTrackPosition` into cheap (local node search) and expensive (lookahead/radius/apex rebuild) parts.
**Risk:** medium — more code movement, but lookaheads are time-based and apex only changes on pass.
**Win:** removes all-corners scan + sort per core tick.

## 7. Replace `AccelerationVector` List with fixed ring buffer
**Size:** medium, ~20 lines.
**Where:** `UpdateTickData` + `ProjectAhead`.
**Risk:** medium — touches every-frame path; must preserve exactly the same values.
**Win:** removes O(N) shift and LINQ every frame per car.

## 8. Orchestrator computes nearest-rival once (kill O(N²) pressure)
**Size:** medium-large, ~30–40 lines.
**Where:** `AutosportRacingSystem.OnTick` or new helper, plus `Racer.UpdatePressure` reads shared array.
**Risk:** medium — changes where pressure data comes from.
**Win:** biggest algorithmic win at 20 cars.

## 9. Reuse `List<TrackPoint>` scratch buffer in `UpdateTrackPosition`
**Size:** small, ~10 lines.
**Where:** `Racer` field + `UpdateTrackPosition`.
**Risk:** low — just stops allocating.
**Win:** reduces GC churn.

My recommendation for the September WIP: do **1, 2, 3, 4, 5, 9** as one batch. They are small, safe, and directly reduce per-core-tick cost. Then verify in-game with 20 cars.

Save **6, 7, 8** for after release unless you hit a hard perf wall.

Which ones do you want to do?