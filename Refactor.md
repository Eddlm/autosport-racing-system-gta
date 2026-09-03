## Pending performance/refactor items (not yet done)

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

## Done
- 1. Cache `Model.GetDimensions()` per racer
- 2. Replace `TrackPoints.OrderBy(...).First()` in `ComputeOffshootSpeedCap`
- 3. Add `_phaseOffsetMs` and de-clump 0.5s/1s gates
- 4. Gate `UpdatePressure` at 0.5s
- 5. Gate `UpdateRivalInfo` at 0.5s
- 6. Split apex update: leapfrog every tick, refill gated 0.5s
- 9. Reuse `List<TrackPoint>` scratch buffer in `UpdateTrackPosition`