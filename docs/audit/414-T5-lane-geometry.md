# 414-T5 — lane controller and the geometry it reads

- Date: 2026-10-30
- Build: 1377 (tree snapshot `src/Racer.cs` md5 `2687d185…`, 4709 lines)
- Reviewer scope: read-only. No source edit, no build, no run, no git.
- Severity: **blocker** = can cost a race or crash; **release-note** = a decision or a line needed before the tag; **file** = hardening, the next reader's problem.

## Scope read (today's line numbers)

| File | Read | Contains |
|---|---|---|
| `src/Racer.cs` | 693–890 | `ComputeSteering`: bounds 708–721, demand chain 712–738, aim point 744–750, pursuit 970–1000 |
| `src/Racer.cs` | 1002–1044 | `TryGetOutsideLane` (the apex line) |
| `src/Racer.cs` | 1046–1097 | `HeadingLeansOut`, `TryComputeCurveLean` |
| `src/Racer.cs` | 1099–1193 | the mover: `UpdateLaneMover` |
| `src/Racer.cs` | 1195–1240 | constants, `NearestApexAhead` |
| `src/Racer.cs` | 1260–1471 | `ComputeAvoidanceLane`, `ComputeAvoidAheadLane`, `TryPickAvoidanceSide`, `ClampLaneToWalls`, `ApplyRivalWalls` |
| `src/Racer.cs` | 2211–2242, 2757–2790, 4026–4048, 4458–4461 | `CornerEntranceNode`, `ForwardNodeDistance`, `MetresToNode`, `OutsideGateMeters`, `IsWithinCorner`, `AdvancePastApex`, `HasPassedApex`, `NearestEdgeLane` |
| `src/Racer.cs` | 3599–3695 | `UpdateTrackPosition` (the node the lane is measured against) |
| `src/DataStructures.cs` | 140–257, 344–378 | `Rival` route-frame fields, `TrackPoint`, `CornerPoint` |
| `src/TrackLoader.cs` | 120–314, 430–523, 740–830, 930–1039 | loader order, `DensifyRoute`, width walk, corner detection, `PreciseCurveRadius`, `Circumradius2D/3D` |
| `src/AutosportRacingSystem.cs` | 551–602, 2742–2767 | `SignedLaneOffset`, `Remap`, `Clamp`, `IsBetween`, `FindNearestTrackPoint` |

`ComputeSteering` is 693–890 and its local `TryGetSteerContext` 879–889. `_laneRequest` has one writer chain (721 → 727 → 732 → 734 → 735 → 738) — no two-writer defect in the demand itself.

## 3D-read sweep (24 sites)

The question at every site: is a vector that carries elevation feeding a read whose answer is meant to be planar?

| # | Site | Vectors / axis | Sampled at different heights? | Verdict |
|---|---|---|---|---|
| 1 | `AutosportRacingSystem.cs:553` `Vector3 right = Vector3.Cross(refDir, Vector3.WorldUp);` | `refDir` = `TrackPoint.Direction`, which is a **3D** unit vector (`TrackLoader.cs:221` normalises a 3D difference, Z kept) | no (same node) | **FLAGGED** — the cross cancels Z but is *not* normalised, so `Dot` returns metres scaled by `cos(slope)`: 7 cm short on a 10 m offset at a 12 % grade, 19 cm at 20 %. Sibling site `Racer.cs:744` normalises the identical cross. See F5. |
| 2 | `Racer.cs:710`, `745`, `1033` (all `SignedLaneOffset`) | as #1 | no | inherits #1 — same scale error |
| 3 | `Racer.cs:750` `SignedLaneOffset(aimPoint, Car.Position, Car.ForwardVector)` | `refDir` = car forward, which carries pitch | yes: `aimPoint` is a road position, `Car.Position` is the car | inherits #1; diagnostic only (read at 845) |
| 4 | `Racer.cs:744` `Vector3.Cross(steerRefPoint.Direction, Vector3.WorldUp).Normalized` | same as #1 | no | clean — **and this is the proof #1 is an outlier** |
| 5 | `Racer.cs:757` `Vector3.SignedAngle(CurrentTrackPoint.Direction, Car.ForwardVector, Vector3.WorldUp)` | two **full 3D** directions, neither flattened | yes (track direction vs car attitude) | **FLAGGED (latent)** — the only true 3D-angle read in the steering path; on a crest the car's pitch enters a heading measure. Dead today (`LaneCrossingScale = false`, 956). See F10. |
| 6 | `Racer.cs:782` `Dot(delta, velDir)` | `delta` 3D (Z gated ≤ `RivalHeightGateMeters` at 781), `velDir` 3D unit | yes | minor — longitudinal gap inflated by the grade; ≤ ~0.4 m against a ~6 m gate |
| 7 | `Racer.cs:783` `Dot(delta, velRight)` | `velRight = Cross(velDir, WorldUp).Normalized` → horizontal unit | yes | clean (Z of `delta` cannot project onto a horizontal axis) |
| 8 | `Racer.cs:799` `SignedAngle(myHoriz, rivalHoriz, …)` | both flattened at 775/795 | no | clean |
| 9 | `Racer.cs:920` `SignedAngle(rivalFlat, courseFlat, …)` | both flattened at 918–919 | no | clean |
| 10 | `Racer.cs:988` `SignedAngle(headingFlat, toAimFlat, …)`, `distance = toAimFlat.Length()` | flattened at 980–982 | no | clean (2D distance, 2D angle, consistent) |
| 11 | `Racer.cs:1035–1038` `Vector3.Angle(steerFlat, entranceFlat)` | flattened, with a `LengthSquared > 0.01` guard | no | clean |
| 12 | `Racer.cs:1048–1052` `Dot(heading.Normalized, right)` | flattened at 1048–1049, `right = Cross(track.Normalized, up)` | no | clean |
| 13 | `Racer.cs:1353` `Dot(Cross(predDir, WorldUp), rivalPosition − predHalf)` | `predDir` 3D unit, cross unnormalised | yes | clean *in effect* — the result is used only as `> 0f` (1354); the foreshortening cannot change the sign |
| 14 | `Racer.cs:3659` `AlongTrackSpeed = Dot(velocity, Direction)` | 3D · 3D | no | clean by intent — along-track speed should include the grade |
| 15 | `TrackLoader.cs:223–225` `SignedAngle(left, forward, WorldUp)` | `left.Z = 0; forward.Z = 0` first | no | clean — the model the rest should follow |
| 16 | `TrackLoader.cs:457–461` chicane `SignedAngle` | flattened at 457–459 | no | clean |
| 17 | `TrackLoader.cs:890` `AimsSameWay` | flattened inline | no | clean |
| 18 | `TrackLoader.cs:504` `points[node].DistanceTo(points[node + 1])` (densify gap) | 3D | no | minor — node spacing becomes 1 m *on the grade*, not 1 m plan; see F6 |
| 19 | `TrackLoader.cs:252` `CumulativeDistance` from 3D `DistanceTo` | 3D | no | minor — `MetresToNode` returns slope-inflated metres, compared against planar gates (300 m at 1013) |
| 20 | `TrackLoader.cs:259` `RouteNodes[0].DistanceTo(last) > 20f` | 3D | a lap's first/last node are the same point, so no | clean in practice |
| 21 | `TrackLoader.cs:986–988` `Circumradius3D` → `Circumradius2D(X,Y)` | drops Z | yes | clean **math**, dishonest **name** (rule 5) — a road radius should be planar and is |
| 22 | `TrackLoader.cs:1023–1038` `Circumradius2D` | planar by construction | n/a | clean |
| 23 | `DataStructures.cs:241` `SignedAngle(flat, flat, me.Car.UpVector)` | flattened; axis is the car's *tilted* up | vectors flat, axis tilted | clean — for coplanar-with-horizontal vectors the axis only sets the sign |
| 24 | `AutosportRacingSystem.cs:2754` `DistanceTo2D` in `FindNearestTrackPoint` | 2D | no | clean — contrast case: the reacquire path at `Racer.cs:3643` uses 3D for the same job |

Two substantive 3D findings (#1 → F5, #5 → F10); the rest are clean or second-order.

## Findings, ranked

### F1 — `_laneRequest` has no single bound, and the doc block claiming one is falsified twice
`src/Racer.cs:717–720`, `1296`, `1379`, `1394`, `1021`

```
717|            // The one bound every writer obeys: half the road, less the car's half width, less the buffer. Inside the corner the
720|            float laneLimit = Math.Max(roadWide - carHalfWidth - (IsWithinCorner(lineCorner) ? 0f : EdgeEmptySpaceMeters), 0f);
1296|            float trackBound = roadWide - carHalfWidth;
1379|            float trackBound = roadWide;
1394|            float trackBound = roadWide;
```

Five places build "the drivable lane bound" and three different values come out: `W − hc − 0.5` (720, and again rebuilt at 1021 for `safeBound`), `W − hc` (1296), and the bare `W` (1379, 1394). 717 says every writer obeys one bound; 1296 and 1379 do not. Consequence: `TryPickAvoidanceSide` may return a pass lane up to 0.5 m wider than the mover will ever deliver, because `UpdateLaneMover` re-clamps at 1119 to `±laneLimit`; the mover's own log prints both numbers, so the gap shows up as `asked W−hc, aimed W−hc−0.5`. Also `ClampLaneToWalls` bounds at the raw half width, which contradicts the comment at 1201 ("the aim keeps the car's half width plus this buffer from the edge") — at 1379/1380 nothing subtracts the car's half width. **release-note.**

*Falsify*: with `ARS.SteerTrace` or the mover log (1191), find a tick where `LaneRequest` = ±(`TrackHalfWidth − carHalfWidth`) during an avoidance pass and `CarriedLane` sat 0.5 m inside it; or read 1296 against 1379 — one of them must be wrong, and both are live on the same tick.

### F2 — the aim point mixes two node frames, and the 0.5 blend only hides half of it
`src/Racer.cs:710`, `745`, `748–749`

```
710|            float steerRefOffset = ARS.SignedLaneOffset(Car.Position, CurrentTrackPoint.Position, CurrentTrackPoint.Direction);
745|            float offsetAtProjection = ARS.SignedLaneOffset(ProjectAhead(SteerPreviewSeconds), steerRefPoint.Position, steerRefPoint.Direction);
748|            float aimPointOffset = aimLane - GsPreviewBlend * (offsetAtProjection - steerRefOffset);
749|            Vector3 aimPoint = steerRefPoint.Position + steerRight * aimPointOffset;
```

`steerRefOffset` (and `_carriedLane`, `aimLane`, `laneLimit`) are lateral offsets from the **current** node's centreline; `offsetAtProjection` is a lateral offset from the **steer-ref** node's centreline, 8–90 nodes ahead (157–160, `0.5 × speed`). Line 748 subtracts the two, so the term is not "how the lane changes ahead" but that change *plus* the arc offset between two different origin lines: `S = R(1 − cos(d/R))`, which is 2.23 m at 30 m/s in R 50 and **3.95 m at 40 m/s**. Because `SteerPreviewSeconds` 0.5 equals the lookahead in seconds, the projected point lands on the ref node, so the difference measures ≈ `S` itself. Line 749 then applies the result as an offset from `steerRefPoint`, i.e. in the other frame — the lane a car is asked to hold is not the lane it is measured against, and the residual is of the order of `S` (metres) against a 0.1 m hold band (742) and a 1 m release band (1216). **release-note** (this is prior finding B, restated at the line where it bites).

*Falsify*: `ARS.SteerTrace` already prints `aimoff/proj/ref` (839–841). On a constant-radius corner entry, `proj − ref` should hover near 0 if the frames agreed; it will instead track the arc offset and grow with speed. Compare against `2*(R − sqrt(R² − (d/2)²))` computed from `PreciseCurveRadius` at the steer-ref node.

### F3 — the avoidance-lane hold is switched off by a hardcoded `false`, against its own justification
`src/Racer.cs:1246–1248`, `1263`, `1265`

```
1246|        // The same five seconds the avoidance lane arms on: a rival we are catching has to keep the corridor closed until
1247|        // the avoidance lane has taken over from it. Retiring the wall at three left a two second band in which the corridor
1248|        // was open, the avoidance lane was not yet the request, and the lean was free to aim at the car ahead.
1263|            bool avoidHoldEnabled = false;
1265|            if (!avoidHoldEnabled) return passLane;
```

`avoidHoldEnabled` is a non-const local, so the compiler cannot elide it; 1266–1290 — the latch, the 8 s timeout, the re-check loop — is unreachable on every tick. The comment three lines above states the hold exists because without it there is a band where the corridor is open and the lean aims at the car ahead; that band is what ships. `_avoidHoldActive/_avoidHoldLane/_avoidHoldUntil` (501–507) are then written only by resets (1805–1807, 4566–4567) and by the dead block — rule 8 (written and never read) and rule 6 (comment contradicts code). **release-note** — either the constant is a leftover debug kill switch or a regression that was never noticed.

*Falsify*: `grep -n "avoidHoldEnabled" src/Racer.cs` → 1263 and 1265 only. Then grep why the comment says otherwise; no other reader of `_avoidHoldActive` exists outside 1266–1290 plus the resets.

### F4 — the whole-route re-snap has no heading or continuity test (prior D)
`src/Racer.cs:3640–3651`

```
3640|            float reacquireDistance = Math.Max(CurrentTrackPoint.TrackHalfWidth + 10f, 25f);
3641|            if (closestDistance > reacquireDistance)
3643|                foreach (TrackPoint point in ARS.TrackPoints)
3645|                    float distance = point.Position.DistanceTo(carPosition);
```

The ±6-node window scan above it (3606–3611) is bounded and safe; this fallback scans **every** node with a 3D `DistanceTo` and takes the nearest with no heading test, no continuity test, no "did I move more than N nodes" test. On a leg that overlaps another (a figure-of-eight, an out-and-back, a pit lane beside the straight) the snap can land on the wrong leg; `CurrentTrackPoint` then jumps, `MetresToNode(corner.Node)` flips sign (2767–2769), and `NearestApexAhead` (1219–1240) picks a corner that reads ahead when it is behind. Everything downstream of the lane is measured against `CurrentTrackPoint` (710, 1033, 3655). **release-note.**

*Falsify*: on any overlapping-leg track, drive off the road (> `max(halfWidth+10, 25)` m) then back on and watch `Lap`/`RaceProgress` (3596) jump, or add a log of `CurrentTrackPoint.Node` before/after 3643. Static check: 3643–3651 contains no `Direction`, `Angle`, or index-delta term.

### F5 — `SignedLaneOffset` returns foreshortened metres because its axis is never normalised
`src/AutosportRacingSystem.cs:553–556`

```
553|            Vector3 right = Vector3.Cross(refDir, Vector3.WorldUp);
554|            Vector3 rPos = pos - refPoint;
556|            return Vector3.Dot(right, rPos);
```

`refDir` is a 3D unit vector (`TrackLoader.cs:221`), so `Cross(dir, up) = (dir.y, −dir.x, 0)` has length `cos(slope) < 1`. Every lane quantity in this scope is this function's output (710, 745, 750, 1033, 3655, and `Rival.NodeOffset` via `DataStructures.cs:191`), and every bound they are compared against — `laneLimit` 720, `LaneOverrideHoldMeters` 0.1 at 742/1099, `OutReleaseReachedMeters` 1 at 1216, `LaneCrossMeters` 0.1 at 1107 — is built with no slope factor. 7 cm short per 10 m of offset on a 12 % grade, 19 cm on 20 %: the second exceeds the 0.1 m release bands. `Racer.cs:744` normalises the identical cross product, so the omission is not a convention. **release-note** (small, but it is the measurement under every band).

*Falsify*: place the car on a 20 % grade (the Vinewood hills) at a known lane offset, then log `SignedLaneOffset` against `DistanceTo2D` to the nearest point on the node's centreline. The ratio will be `cos(atan(grade))`, and `Racer.cs:744` will disagree with it by that factor.

### F6 — `ForwardNodeDistance` returns nodes and five call sites treat it as metres or seconds
`src/Racer.cs:2757–2761`, `2828`, `2840`, `2864`, `2981`

```
2758|            int fwd = targetNode - CurrentTrackPoint.Node;
2759|            if (!ARS.IsPointToPoint && fwd < 0) fwd += ARS.TrackPoints.Count;
2760|            return fwd;
2840|                && ForwardNodeDistance(NextApexNode) > NitrousLonelyMinApexDistance;
2828|                if (ForwardNodeDistance(entranceNode) / Math.Max(speed, 1f) < NitrousCornerLookaheadSeconds) return false;
```

`ForwardNodeDistance` is a node count; `MetresToNode` (2765) exists precisely because the two are not interchangeable, and its comment says so. Yet 2840 compares nodes to `NitrousLonelyMinApexDistance = 500f` (a metres-named constant, 426) and 2828/2864/2906/2981 divide nodes by m/s and compare to `NitrousCornerLookaheadSeconds = 8f`. The identity holds only under an undocumented invariant — `DensifyRoute` (487–523) forces ~1 m spacing for gaps above 1.5 m, but leaves every gap ≤ 1.5 m alone, and returns early when `dense.Count == count`. On a hand-drawn route with 1.4 m nodes the "8 seconds" is 11, and the "500 m" is 700. Rule 5: the name does not carry the unit and the arithmetic assumes it does. **release-note** (nitrous/dive timing, not the lane itself).

*Falsify*: log `ARS.RouteLengthMeters / ARS.TrackPoints.Count` (252/254) on any track; if it is not 1.0, every one of those five comparisons is off by that ratio, linearly.

### F7 — `999` is the sentinel for "huge radius", "no solve", "unknown" and "uncomputed"
`src/TrackLoader.cs:237–238`, `1034`, `1038`, and `src/Racer.cs:1079`

```
237|                point.GeneralCurveRadius = 999f;
238|                point.PreciseCurveRadius = 999f;
1034|            if (denominator == 0f || float.IsNaN(denominator) || float.IsInfinity(denominator)) return 999f;
1079|            if (!(ahead.PreciseCurveRadius <= CurveLeanRadiusMeters)) return false;
```

Four meanings share one number, and 999 m is also a legal radius for a real road. Two consequences in this scope. (a) `Racer.cs:1079` treats "the solve degenerated" and "this is straight" identically, so a node whose circumcentre failed reports a number that reads as a straight — the lean silently skips it and `_leanApproachMps` is never re-armed. (b) `TrackLoader.cs:213–214` marks the first and last `PointToPointEndBandNodes` (50) of a point-to-point as uncomputed: `Angle = 0`, `PreciseCurveRadius = 999` (234–238), and `Direction` is set to the *global* `(RouteNodes[10] − RouteNodes[5])` for all of them. A corner inside the first or last 50 m of a point-to-point is therefore invisible to `NearestApexAhead` (1230 filters `DetectedRadius >= 50`), and the car runs it on `steerRefOffset` alone, measured against a direction that belongs to another part of the route. **file.**

*Falsify*: for (a), log which branch of 1034/1038 fires on a known-straight and a known-tight node — the value is identical. For (b), build a point-to-point whose first corner is at node 30 and watch `ARS.Corners` come back empty near the start, and `TrackPoints[30].Direction` equal to `TrackPoints[0].Direction`.

### F8 — the mover comment says 24 m/s; the constant is 99
`src/Racer.cs:1101–1103`

```
1101|        // The mover: 40 m/s^2, 24 m/s, half of both outward. The cap bounds the lunge, the acceleration the reversal.
1102|        const float LaneMoverAccelMps2 = 40f;
1103|        const float LaneMoverSpeedCapMps = 99f;
```

Rule 6. The acceleration half of the sentence matches; the speed half is 4× out. The cap is what bounds the lunge (1166, 1169), and 99 m/s is effectively "no cap" for a lane offset bounded by ~8 m, so the sentence describes a mover that does not exist. Either the constant or the comment is stale, and there is no third witness. **file.**

*Falsify*: read 1102–1103 — the numbers are literal. Then read 1159–1161 to see the cap is the live limiter (halved when `toOutside`, further `Min`'d with `_leanApproachMps`).

### F9 — the lean-rate comment promises 10 m/s at R = W; the constant gives 20
`src/Racer.cs:932–933`, `1084`

```
932|        // The base rate: width over radius, 10 m/s when the radius equals the width.
933|        const float CurveLeanWidthScale = 20f;
1084|            _leanApproachMps = CurveLeanWidthScale * trackWidthMeters / Math.Max(ahead.PreciseCurveRadius, 5f);
```

`trackWidthMeters` is the **full** width (`roadWide * 2f` at 721). With `R = W` the expression evaluates to `20 × W / W = 20 m/s`, not the 10 the comment states. Rule 6; the two disagree by exactly the factor of two that the comment was presumably written to explain. This is the cap that limits how fast the aim can run to the inside on every corner entry (1161), so a reader tuning from the comment will be off by 2×. **file.**

*Falsify*: substitute `trackWidthMeters = 2 × halfWidth` into 1084; at `R = 2 × halfWidth` the rate is 20, and the comment's "radius equals the width" is exactly that case.

### F10 — the only genuine 3D `SignedAngle` in the steering path is a dead branch
`src/Racer.cs:754–760`, `956`

```
956|        static readonly bool LaneCrossingScale = false;
757|                float crossingDeg = Vector3.SignedAngle(CurrentTrackPoint.Direction, Car.ForwardVector, Vector3.WorldUp);
759|                if (!turningIn && laneSteerDeg * crossingDeg > 0f)
760|                    laneSteerDeg *= LaneCrossingShare(Math.Abs(crossingDeg), speedMps);
```

`CurrentTrackPoint.Direction` and `Car.ForwardVector` both carry elevation, so `crossingDeg` is the 3D angle between them: on a crest the car's nose-up attitude is counted as a heading difference. `Vector3.WorldUp` only sets the sign. Every sibling measure in this file flattens first (775/795, 918–919, 980–981, 1035–1036, 1048–1049). The block is unreachable behind a `static readonly false`, so it is latent rather than live — but it is the exact trap item 1 describes, left armed with a `Read`-able switch, and `LaneCrossingThresholdDegrees`/`LaneCrossingShare` (958–968) exist only to serve it. **file.**

*Falsify*: flip 956 to `true` in a local build and log `cref` (838, already computed flat) against `crossingDeg` on a crest — they will diverge by the pitch angle.

### F11 — `MetresToNode` answers "0 m, right here" for a node that does not exist
`src/Racer.cs:2765–2766`

```
2765|        float MetresToNode(int node)
2766|        {
2767|            if (node < 0 || node >= ARS.TrackPoints.Count) return 0f;
```

Comment at 2764 promises metres, wrapped, negative behind. An out-of-range node silently becomes "at the car", which is the most aggressive value the callers can receive: `NearestApexAhead` accepts `metres >= 0` (1232–1233) and `IsWithinCorner` gates the lane buffer on the same corner (720, 1088), and `TryGetOutsideLane` tests `metresToApex <= OutsideGateMeters` (1042). The only reason this is not already visible is that `_defendApexNode = -1` (2720) yields `0 > gate` = false at 727, so the DefendLane demand fails safe by accident. A stale or edited corner index is not safe by accident. **file.**

*Falsify*: call `MetresToNode(-1)` and `MetresToNode(TrackPoints.Count)`; both return 0, indistinguishable from a corner under the wheels.

### F12 — an unguarded `Clamp` turns a NaN lane request into the opposite road edge
`src/Racer.cs:1119` with `src/AutosportRacingSystem.cs:594`

```
1119|            float request = ARS.Clamp(_laneRequest, -laneEdge, laneEdge);
594|            if (val.CompareTo(min) < 0) return min;
```

`float.NaN.CompareTo(x)` is −1 for every x, so `Clamp(NaN, −L, +L)` returns `−L` — a full-width request to the right edge, not a hold at centre. `_laneRequest` is written from five sources (721–738) with no NaN filter, and the two neighbouring reads *do* guard (`1122` for `gap`, `989` for the bearing), so the omission is inconsistent as well as unsafe. Same shape at 1145 (`Clamp(steerRefOffset, …)`), 1166, 1186, and 1388. **file** — latent, but the failure mode is a hard lunge at the road edge rather than a neutral.

*Falsify*: unit-call `ARS.Clamp(float.NaN, -8f, 8f)` in a scratch console — it returns −8. Then find the writer that can produce NaN: 1084/1089 propagate `laneEdge` untouched, and `ClampLaneToWalls` at 1388 propagates `lane` untouched.

## Prior findings re-checked

| # | Claim | Verdict today |
|---|---|---|
| A | `drivableEdge`/`trackBound` in five places under two meanings | **TRUE**, with the count now five and the values three: 720 `W − hc − 0.5`, 1021 `W − hc − 0.5` rebuilt, 1296 `W − hc`, 1379 and 1394 bare `W`; plus `OutOfTrackDistance` at 3555 comparing `|offset|` to `TrackHalfWidth` raw. See F1. |
| B | `ownLane` (car frame) vs `steerRefOffset` (reference frame), ~2.3 m at R 50, against a 0.1 m band | **TRUE, at a different site than recorded.** The latch/guard pair is consistent today — `ownLane` at 721 *is* `steerRefOffset`, and the mover's `aimFromCar` (1134) uses the same frame. The real frame crossing is the aim point: 748 subtracts a reference-frame offset (745) from a car-frame one (710) and 749 applies the result in the reference frame. Magnitude `R(1 − cos(d/R))`: 1.00 m at 20 m/s, 2.23 m at 30 m/s, 3.95 m at 40 m/s, all at R 50. See F2. |
| C | `999` is both "huge radius" and "degenerate solve" | **TRUE** at `TrackLoader.cs:1034` and `1038` (both return 999), at `237–238` (end bands and out-of-range), and consumed as a threshold at `Racer.cs:1079` and `TrackLoader.cs:978` (`radius >= 999f … continue`), which also discards a legal 999 m road. See F7. |
| D | whole-route re-scan with no heading test mis-snaps on an overlapping leg | **TRUE** at `Racer.cs:3643–3651` — the `foreach` over all `ARS.TrackPoints` with a bare 3D `DistanceTo`, no `Direction`/`Angle`/continuity term, entered whenever `closestDistance > max(halfWidth + 10, 25)`. The bounded ±6-node scan above it (3606–3611) is not the problem. See F4. |

## Checked and deliberately not reported

- `ARS.Remap(Aggression, 100f, 0f, 0.2f, 1.2f, true)` at 1297 and 1417 — inverted *input* bounds with `clamp` on correctly ordered *output* bounds. The formula handles it (578) and the clamp range is `[0.2, 1.2]` in the same order as the arguments. By design: high aggression ⇒ small buffer.
- `IsWithinCorner` (2776–2790) handles the corner that straddles the start line by testing both orders. Correct.
- The demand chain 721 → 727 → 732 → 734 → 735 → 738 is a single writer to `laneRequest`; the last writer wins, and `UpdateLaneMover` smooths the request through `requestRate` (1166), so a source flip cannot step the aim.
- `_laneRequestOutward` and `_leanApproachMps` are last-tick latches but are read only behind `_laneRequestReason == LaneReason.CurveLean` (1155, 1161), so they cannot leak into another source's mover law.
- The defend-lane gate is read at two nodes on purpose: comment at 2712–2716 ("The writer reads the same gate at the steer node and this one reads it at the car's node … which costs nothing because this only frees the slot"). By design, quoted.
- `NearestEdgeLane` (4458–4461) returns the edge on the car's own side, and since the aim is a *lane value* the car outside it is pulled back in. Correct sign, not a bug.
- `LaneMoverAccelMps2` halved for `toOutside` plus the 6 m/s OUT cap (1156–1161) against the comment at 1101 "half of both outward": the accel half is true, the speed half is F8.
- `OffsetCornerNode` (3540–3549) and `ResolveLookAhead` (3669–3674) both modulo a circuit route and clamp a point-to-point; `AdvancePastApex` (4030–4040) modulos `since` at 4035. **No missing-modulo defect found** — item 6 comes back clean.
- Lap-0 gate at 721: `Lap` is set to 1 when behaviour becomes `Race` (1772), and `GridWait` already returns from `TryGetSteerContext` at 883, so the gate is redundant, not a first-lap blind spot.

## Could not decide

1. **F2's residual sign and size.** The algebra says the aim lands roughly 1.5 × the arc offset away from the physical lane it is given, but `offsetAtProjection` depends on where `ProjectAhead(0.5)` actually falls relative to the ref node, which depends on `AverageAcceleration`. Decidable only from the existing `steertrace` line (839–845) on a constant-radius corner.
2. **F3's intent.** Nothing in the tree says whether `avoidHoldEnabled = false` is a debug kill switch or a reverted regression; the 8 s timeout, the resets at 1805 and 4566, and the comment at 1246–1248 all say the feature was built and then switched off.
3. **F1's correct bound.** Whether the wall clamp should use `laneLimit` (drivable) or the raw half width (walls may reach the edge) is a design question, not a defect. The defect is that both are live and the doc block claims only one exists.
4. **F7(b)'s practical bite.** Whether a point-to-point's first/last 50 nodes matter depends on how tracks are authored; I did not have a track file to check whether any shipped route has a corner inside that band.
5. **F8/F9's direction.** Which side of each comment/constant pair is stale cannot be read from the source; only the driver's tuning notes can settle it.
