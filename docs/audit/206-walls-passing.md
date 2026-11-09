# AUDIT 206 — rival walls, passing side, and the avoidance latch

**Surface**: the body-to-body rules between rivals as they are compiled today — `ApplyRivalWalls` and
`ClampLaneToWalls` (`Racer.cs:1673-1797`), the record they decide from (`Rival.Update`,
`DataStructures.cs:169-273`), the pass picker and its latch (`ComputeAvoidanceLane`,
`ComputeAvoidAheadLane`, `TryPickAvoidanceSide`, `Racer.cs:1544-1669`), the dead avoidance hold
(`Racer.cs:1531-1579`), and the two timers that publish the record (`Racer.cs:4698-4701`,
`:4784-4788`, `:5279`).

**Date**: 2026-10-25.

**Build audited**: **1092**. `build/build-number.txt` reads `1092`; `bin/Release/ARS.dll` is sha256
`8e3ab22dc3b3ff8a1dda4dfd3cd8a7f5c8017f36206032ffa3210dea6c983285`; source `HEAD e851a09`.

**Method**: read-only. Source read, `git blame` read, and the constants as written put through
`awk`. No build, deploy, commit or game run. No log line was used: this surface has **no log
instrument** — nothing in `ApplyRivalWalls`, `ClampLaneToWalls` or the pass picker writes one — so
every reproduction below names a line that would have to be added. That absence is itself a finding
the next drive should close (see "What could not be verified").

**Status vocabulary**: **VERIFIED** = read in the code, or reproduced read-only (arithmetic, source
read). **PROBABLE** = one step rests on a behaviour not re-observed today. Nothing here is
driver-verified.

**Verdict**: **two blockers, both carried from audit 04 and both re-verified on 1092 — neither has
moved.** 206-01 is 04-W1 with its period corrected (the record is not 500 ms old, it is 250-749 ms
old, so the staleness band is both wider at one end and narrower at the other than the 500 ms figure
audit 04 tuned against); 206-02 is 04-W2 unchanged, with the
three-abreast threshold now computed instead of transcribed. What is new is the *side*: both systems
that answer "which side is he on" read the rival in this car's **body frame**, and in a corner that
frame reports a car directly ahead in the same lane as being up to four metres to one side
(206-03). The walls are what starts an overtake, so the walls start every overtake on the outside of
the corner, on both hands — and the pass picker agrees with them, which is why it has never read as a
conflict. Three smaller items follow: a one-sided deadband where the picker and the wall name
opposite sides (206-04), the car-width inset that steps on release rather than on the wall itself
(206-05, the mechanism the deleted avoid-hold was covering for), and a latch that lets go at the
first centimetre of daylight (206-06). The brief's own question about `AvoidHoldTimeoutMs` closes
with no defect: the whole hold is unreachable (206-07).

---

## Findings

Severity: **blocker** = must change before the tag; **release-note** = true as shipped, but it will
mislead on day one or needs a decision; **file** = a defect in a tracked or shipped file.
**Blockers: 2 (206-01, 206-02), ranked first.**

| ID | Severity | Claim | Evidence | State | How to reproduce |
|---|---|---|---|---|---|
| 206-01 | **blocker** | **Every wall gate is decided from a rival record that is 250-749 ms old, and the gap it tests moves further than the overlap window in that time.** The record's beat is `RivalRefreshMs + _phaseOffsetMs`, not `RivalRefreshMs`: `_rivalTick = now + 250` and the gate tests `_rivalTick + _phaseOffsetMs < now`, so the period is `250 + phase` with `phase = GetRandomInt(0, 500)` = 0-499 ms — **250-749 ms, mean 500 ms, i.e. 2 Hz on average and 1.3 Hz at worst**, not the four times a second the comment claims. `_phaseOffsetMs` is seeded once per racer in the constructor. It is the faster of the two publishers of this record while `BaseBehavior == Race` (`:4701`); outside a race the 500 ms path is the only one and the record is older still (206-08). The gates that consume the record: `overlaps` (`:1715`), `approaching` (`:1719`), the wall magnitude (`:1736`, `:1741`), the side (`:1732`), the latch (`:1596`) and the avoidance picker's candidate gate (`:4047`). The overlap window is `±CombinedSize.Y` = 5.40 m ahead / 4.65 m behind for two 4.4 m cars, 10.05 m full (`DataStructures.cs:196-197`, buffer 1.0 m ahead and 0.25 m behind). At a 10 m/s closure one beat steps the route gap **2.5-7.5 m** — the whole half-window at the high end — so the gate can go from "six metres ahead" to "a metre and a half behind" without ever reading "alongside", which is the tick the wall exists for. The lane term moves at the rival's own lateral rate: 3 m/s against a 1.9 m corridor gate (`DataStructures.cs:265`) is 0.75-2.25 m per beat, so a rival mid-lane-change enters or leaves the corridor between samples. Carried from `04-W1`, whose verdict the verify pass recorded as "parcial" and whose 500 ms figure is superseded (see Corrections). | `Racer.cs:102-104`, `:559`, `:4698-4701`, `:5279`, `:1715`, `:1719`, `:1732`, `:1736`, `:1741`; `AutosportRacingSystem.cs:3389-3391` (`Random.Next(min,max)`, upper exclusive); `DataStructures.cs:196-197`, `:265`. Arithmetic: period 250+p, p∈[0,499]; gap step = closure × period. `Racer.cs:103` states in its own comment that the freshness of these records "is a safety property, not a convenience". | VERIFIED (code + arithmetic). Carried from 04-W1. | Add one line at the rival tick only: print `_rivalTick`, `_phaseOffsetMs`, `now - _rivalTick` and `r.RouteGapAhead` beside a live recomputation of the same gap from both `CurrentTrackPoint.CumulativeDistance`. The steps land 250+phase apart, and a rival closing at 10 m/s is sampled with nothing in between either side of the overlap band. |
| 206-02 | **blocker** | **When the clamp-space corridor inverts, both walls are discarded and the whole road is returned — and the band where it happens is the ordinary three-abreast gap, not a pathological one.** `ClampLaneToWalls` adds `carHalfWidth` on each *constrained* side (`:1681-1682`), while the ordering guard (`:1756-1762`) runs on the targets, where no inset exists. Targets that are ordered in target space therefore produce `clampLeft > clampRight`, which takes the branch at `:1683` and returns `Clamp(lane, -trackBound, trackBound)` — no constraint at all, and no relabel, because `walledLane == laneRequest`. Threshold: inversion needs each rival closer than `rivalBuffer + carHalf`, i.e. **2.10 m per side (4.20 m separation) at Aggression 100 and 3.10 m per side (6.20 m separation) at Aggression 0** for a 1.9 m car; with 1.2 m half-widths at Aggression 100 the threshold is 5.20 m separation — exactly the "zero-width" point audit 04's transcription measured, which is the cross-check that this arithmetic is the code's arithmetic. At Aggression 0 the two rivals are **6.20 m apart, leaving 4.30 m of daylight between two 1.90 m cars** and the car 1.90 m wide still has no walls. Three abreast is precisely this case, and it is the case the contact steer-away (`:929`) has to answer alone. | `Racer.cs:1681-1683`, `:1756-1762`, `:1764-1765`, `:1796`, `:929`, `:1722` (per-rival `aggroBuffer`). Arithmetic above; `rivalBuffer = rivalBoundingBox/2 + aggroBuffer`, `aggroBuffer = Remap(Aggression, 100, 0, 0.2, 1.2, true)` (`:1722`, `:1586`). | VERIFIED (code + arithmetic; the 5.20 m match is 04's transcription, not re-run). Carried from 04-W2. | Two rivals one each side at ±2.1 m with Show Rivalry on: both walls draw red while the lane request crosses from one side to the other unclamped, and the `Lane mover` line keeps naming other reasons. |
| 206-03 | release-note | **"Which side is he on" is read in this car's body frame while the wall is placed in the track frame, so in a corner a car directly ahead in the same lane is reported as being metres to one side — and the follower is pushed to the outside of every corner.** `LateralGap = RelativeOffset.X`, taken from `GET_OFFSET_FROM_ENTITY_GIVEN_WORLD_COORDS` (`DataStructures.cs:192-194`, `AutosportRacingSystem.cs:608-611`), is the rival in this car's local frame; `+X` is right, as the project's own mapping shows (`DataStructures.cs:237-238`). The wall's position comes from `r.NodeOffset` — the rival's own **track-frame** lane (`:1736`, `:1741`) — and the same record's corridor gate is also track-frame (`DataStructures.cs:265`). A point on the arc a distance *d* ahead of a car on radius *R* sits `d²/2R` off the car's forward axis: at **R = 50 m, d = 20 m gives 4.00 m**, and the 0.3 m deadband is passed at 5.5 m (R=50), 7.7 m (R=100), 11.0 m (R=200). So a rival that the corridor gate admits as same-lane (`\|ΔNodeOffset\| ≤ ~1.9 m`) is simultaneously read as 4 m to the right, and the **right** wall closes on a right-hand corner while the car holds the racing line — the request is pinned 2.10 m (Aggression 100) to **3.10 m** (Aggression 0) to the rival's left, i.e. to the outside of the corner. Mirror it on a left-hander: the sagitta changes sign, the **left** wall closes, and the car is pinned to the outside again. The pass picker does not disagree, which is why this has never read as a conflict: `TryPickAvoidanceSide` measures the same cross product from `predHalf` (`:1641-1647`), a point 0.5 s ahead, which shrinks the magnitude about four-fold and keeps the sign, so `passLeft = rivalSide > 0` picks the outside too. Net: `LaneReason` reads `WallClamp`, the pass runs on the outside on both hands, and the inside line is unavailable for as long as a car is within the 3 s approach window (`WallApproachSeconds`, `:1525`) in the same lane. | `DataStructures.cs:192-194`, `:237-238`, `:265`, `:159`; `AutosportRacingSystem.cs:608-611`; `Racer.cs:1732`, `:1530`, `:1736`, `:1741`, `:1641-1647`, `:1649`, `:1719`, `:1525`, `:769-771`. Arithmetic: `d²/2R` table at R = 30/50/100/200 and d = 10/15/20/30/40. | VERIFIED (code + arithmetic); the visible consequence is PROBABLE — not driven. | A rival holding the racing line in a ~50 m corner, closing at ~11 mph: Show Rivalry draws the wall on the corner's inside red while `\|NodeOffset - myNodeOffset\|` is under a car width, and the `Lane mover` line names `WallClamp` with no `Avoidance` before it. Log `r.LateralGap`, `r.NodeOffset - nodeOffset` and `r.RouteGapAhead` on one line: the first grows with the corner, the second stays near zero. |
| 206-04 | release-note | **The side deadband is one-sided, and inside it the pass picker and the wall name opposite sides 4.20 m apart.** `rivalOnRight = r.LateralGap > WallSideDeadbandMeters` (`:1732`, 0.3 m at `:1530`) maps **every** rival with `LateralGap ≤ 0.3` onto "on my left", so a rival 0.29 m to my **right** closes the **left** wall and forces this car to that rival's right — through its own front corner if the car is already right of it. The comment above it states the rule the code does not implement: "only a rival on the centreline counts as straight ahead" is a band of ±0.3 m, and the code has (−∞, 0.3]. Inside the band the two systems disagree: the picker uses `passLeft = rivalSide > 0f` (`:1647`) with no deadband, so a rival 0.2 m to the right is passed on the **left** (`:1649`), and the wall then clamps that request to the rival's **right** (`:1736`). The span between the two answers is `2 × (rivalHalf + aggro + carHalf)` = **4.20 m** at Aggression 100, and because the clamp runs last (`:769-771`) the car follows the wall while `LaneReason` reads `WallClamp` and the picker's choice is discarded. On a straight this is also the tie-break for a collinear rival: it is always "pass on the right". | `Racer.cs:1732`, `:1530`, `:1729-1743`, `:1647`, `:1649`, `:769-771`, `:768`. Arithmetic: picker lane `= rivalLane - (rivalHalf + aggro + carHalf)`, wall floor `= rivalLane + (rivalHalf + aggro + carHalf)`. | VERIFIED (code + arithmetic). | On a straight, a rival 0.2 m right of centre and closing: log the picker's `_passLaneValue` and the returned `laneRequest` on the same tick — the first is left of the rival, the second right of it, and the `Lane mover` line names `WallClamp`. |
| 206-05 | release-note | **The car-width inset is keyed to this tick's constraint flag rather than to the wall itself, so the permitted lane steps outward by half a car in one tick at the instant a passed rival leaves the overlap gate — the exact instant the deleted avoid-hold existed for.** `clampLeft/clampRight` add `carHalfWidth` only while `leftConstrained`/`rightConstrained` are true (`:1681-1682`, set at `:1764-1765`), while the walls themselves are rate-limited and keep moving after the flag drops: the target jumps to the track edge and the stored wall walks there at the 5 m/s open rate (`:1772-1773`, `:1779-1793`). The gate releases when `\|RouteGapAhead\|` exceeds `CombinedSize.Y`, which for a rival **behind** is the body length plus a **0.25 m** buffer (`DataStructures.cs:195-197`) — so at the instant the passed car's bumper is 0.25 m behind, the constraint on this car's lane relaxes by `carHalfWidth` = **0.95 m in a single tick**, and it takes 0.19 s at 5 m/s for the wall itself to account for any of it. The mover absorbs most of it (14 m/s² from rest reaches about 0.25 m in those 0.19 s), so the mechanism is a 0.25-0.95 m step outward, in the direction of the car just passed, while its nose is 0.25 m from this car's rear corner. Audit 04 filed this as W6; the argument for raising it is the hold deleted on 2026-10-25 (`:1531-1538`, `:1546-1551`), whose comment now asserts that "ApplyRivalWalls covers the nose clip on its own". The drive that retired the hold is evidence the *frequency* is low, not that the step is absent, and this is the one mechanism by which the wall's guarantee is weaker than the flag says. | `Racer.cs:1681-1682`, `:1764-1765`, `:1772-1773`, `:1779-1793`, `:1531-1538`, `:1546-1551`, `:1120-1174`; `DataStructures.cs:195-197`. Arithmetic: `carHalf = BoundingBox/2` (`Racer.cs:1679`, `:3873`); 0.95/5 = 0.19 s. | VERIFIED (code + arithmetic); the clip is PROBABLE. | Log `leftConstrained`, `_avoidLeftWall`, `clampLeft` and `carHalfWidth` on one line through a pass: one tick after `RouteGapAhead` crosses `-CombinedSize.Y`, `clampLeft` drops by 0.95 m while the wall drawn by Show Rivalry has not moved. |
| 206-06 | release-note | **The pass-lane latch releases at the first centimetre of lateral daylight, so the committed side is not held during the half of the pass where it matters.** `bodiesSideBySide = \|target.NodeOffset - nodeOffset\| < target.CombinedWidth` (`:1596`) compares two lane offsets against `CombinedSize.X = (w_me + w_rival)/2` (`DataStructures.cs:198-199`), which is exactly the centre separation at which two bodies **just touch**. So the latch is armed while the bodies *overlap* laterally and disarmed the moment they stop touching — which is the opposite half of the window from the one RULE 2 describes ("once the two bodies are side by side … there is no room to cross over without touching"). A pass with 0.4 m of daylight between the doors, which is what a pass looks like, has the latch **off** and the picker re-asked every core tick. The band is narrow (it opens at the touching distance and the alternative side needs another car width), so the practical exposure is small — but the guard is named for a state it does not cover, and the code that reads it will. | `Racer.cs:1592-1603`, `:1596-1597`; `DataStructures.cs:198-199`, `:162`. | VERIFIED (code + arithmetic). | During a pass, log `\|target.NodeOffset - nodeOffset\|` beside `target.CombinedWidth`: the latch flag goes false within the first 0.1 m of lateral clearance, while `TryPickAvoidanceSide` is re-run on every core tick after it. |
| 206-07 | release-note | **`AvoidHoldTimeoutMs` cannot trap a rival: the hold it times is unreachable, so that question has no code path to answer.** `avoidHoldEnabled` is a local `false` (`:1549`) and the very next statement returns on it (`:1551`), so every line from `:1552` to `:1579` is dead: the latch write, the timeout backstop and the geometric release never execute. `AvoidHoldTimeoutMs` (`:1538`), `AvoidHoldBehindMeters` (`:1534`), `AvoidHoldLateralMeters` (`:1535`), `_avoidHoldLane`, `_avoidHoldActive` and `_avoidHoldUntil` therefore have no live reader or writer, and the "ZOMBIE" markers at `:1533`/`:1537`/`:1546-1548` date the decision to today. Nothing is trapped, and nothing is held: the post-pass lane protection the hold was built for is now 206-05's job. | `Racer.cs:1531-1579`, `:530-536`; `grep -n "_avoidHold" src/Racer.cs` — every site is inside the disabled block. | VERIFIED (code). | Delete `:1552-1579` and the six fields: the build is unchanged, and no `_avoidHold` symbol survives. |
| 206-08 | file | **The 500 ms `UpdateRivalInfo` timer is redundant: the rival refresh already republishes the record on its own beat, so the records are filled twice per beat and one of the two is thrown away.** `UpdateRivals` ends with `UpdateRivalInfo()` (`:5279`, in the tree since `601313ce`, 2026-10-06), and `UpdateRivals` is called from the rival tick (`:4698-4701`); `ProcessAI` then calls `UpdateRivalInfo()` again on a separate 500 ms timer (`:4784-4788`). The second path runs `Rival.Update` for every slot — two entity vectors, two positions and one offset native per rival — for a result the next beat overwrites. It is also the path the verify pass read as the *only* source of the record, which is how 206-01's period came to be recorded as 500 ms. | `Racer.cs:4784-4788`, `:5279`, `:4698-4701`, `:4038-4053`; `DataStructures.cs:169-184`. `git blame -L 5276,5280` → `601313ce`. | VERIFIED (code + git). | `git log -S "UpdateRivalInfo();" -- src/Racer.cs` and read `601313ce`; or log the game time of both calls for one car for two seconds — they interleave on every other rival tick. |
| 206-09 | file | **`FrontGap` is documented as a front-to-rear distance but is published negative whenever the rival overlaps, and it is inert only because two different guards happen to sit in front of the two consumers.** `FrontGap = RouteGapAhead - CombinedSize.Y` is assigned at `:267` **before** the `if (FrontGap <= 2f) return 0f` at `:270`, so the field keeps the negative value while `TimeToReach` reports zero. The two readers: `IsBetween(r.FrontGap, 0f, 3f)` (`Racer.cs:1611`) correctly rejects a negative, and `Remap(r.FrontGap, 0, RivalLiftMeters, RivalLiftFloor, 1f, true)` (`:2943`) is protected by `RelativePosition == Ahead` (`:2942`) — an overlapping rival is classified Left/Right (`DataStructures.cs:227-239`), and `RelativeOffset.Y > CombinedSize.Y` cannot co-exist with `RouteGapAhead < CombinedSize.Y` because an arc is never shorter than its chord. Both are right today by construction, not by contract. | `DataStructures.cs:159`, `:196-197`, `:225-239`, `:261-273`; `Racer.cs:1611`, `:2939-2945`, `:475-476`. | VERIFIED (code + geometry). | Log `FrontGap` beside a rival that is alongside: it reads about −3.0 m while the comment calls it the front-to-rear distance. |

---

## Questions that closed with no defect

The brief's remaining questions all close, with a source each; recording them so the next reader does
not re-open them.

- **Division by zero, or by a difference that can be negative, in `TimeToReach` / `FrontGap` /
  `CombinedSize`.** No. `ComputeTimeToReach` (`DataStructures.cs:261-273`) screens twice before the
  only division: `if (FrontGap <= 2f) return 0f` (`:270`) and `if (closingAlong <= 0.001f) return
  float.PositiveInfinity` (`:269`), so the divisor is always above `0.001` and the dividend above 2 —
  the quotient is finite, positive, and bounded by `FrontGap/0.001`. A negative `FrontGap` (an
  overlapping rival) is caught by the same `<= 2f` screen, so it never reaches the divide; it only
  survives in the published field, which is 206-09. `CombinedSize` is never a divisor anywhere in
  the tree. The degenerate zero-width corridor is handled before the clamp (`:1683`), not by
  dividing.
- **Can a rival be trapped, never able to pass?** No, and the reason is structural rather than
  lucky: `TimeToReach` returns infinity for any rival that is not *ahead* (`RouteGapAhead <= 0f`,
  `DataStructures.cs:264`), and `approaching` is its only gate (`Racer.cs:1719`), so a car behind
  never walls the car in front of it. The pair cannot lock; only the follower is constrained, and
  the leader is free. A picker that bails — a pass lane outside the track (`:1663`) or already past
  it (`:1665-1666`) — is also not a trap, because the wall then produces a legal side on its own
  (`:1736`/`:1741`) and `LaneReason` reads `WallClamp`. The cost of 206-03 is *which* side, not
  whether.
- **A rival half a length behind.** Correct on all four counts. `RouteGapAhead ≈ −2.2 m` is well
  inside the overlap gate (`|gap| < CombinedSize.Y` ≈ 4.65 m behind, `Racer.cs:1715`), so the wall
  applies; `ClassifyRelativePosition` reads Left/Right rather than Behind
  (`DataStructures.cs:231-239`), which is what a body beside the rear wheel is; `TimeToReach` is
  infinite behind (`:264`), so no approach wall is built from it; and `isAvoidanceCandidate`
  requires `RelativePos.Ahead` (`Racer.cs:4047`), so it is never a pass target. The only asymmetry
  is the deliberate 0.75 m tighter buffer behind (`DataStructures.cs:195-196`).
- **Are the wall rates per second?** Yes. `TickScale` is seconds (`:2582`), so `wallCloseRate = 10 *
  TickScale` (`:1772`) and `wallOpenRate = 5 * TickScale` (`:1773`) are 10 m/s and 5 m/s, and the
  `wallSnapMeters` jump is a distance, not a rate.

---

## Corrections to the record

- ⚠️ **`201-verify-01-06.md`, row `04-W1`**: the row says the record "did not move" and that
  `Racer.cs:4784-4786` "is what fills `Rival`". It is also filled by `UpdateRivals` →
  `UpdateRivalInfo` (`:5279`, in the tree since `601313ce`, 2026-10-06, i.e. before that verify
  pass), on the 250 ms rival tick. So the beat is `min(250+p, 500+p)` = **250-749 ms**, not 500 ms —
  the verdict "still stale" stands, but the figure is fresher than that row says at one end (250 ms)
  and staler than the code's own comment claims at the other (749 ms). The corrected figure is
  206-01's.
- ⚠️ **`Racer.cs:4695-4697`**: "UpdateRivals is three records and a handful of native reads, cheap
  at four times a second". With `_phaseOffsetMs` added to the deadline the period is `250 + phase`
  ms, so the record refreshes **1.3-4 times a second, 2 Hz on average** — 206-01.
- ⚠️ **`Racer.cs:1126-1127`**: "the demand handed to the mover is already `roadWide -
  carHalfWidth`". It is not: `ClampLaneToWalls` bounds the request to `±roadWide` (`:1681-1683`,
  `:1691`), and the mover is where the `- carHalfWidth` is applied (`:1128`). The value the mover
  acts on is the same, so nothing behaves differently — but the sentence states as a guarantee an
  invariant the wall clamp does not hold, and it is the invariant a future reader will trust.
- ⚠️ **`04-W3`** ("a wall may name a lane the car cannot occupy") is still true of the wall clamp
  and still harmless, for the reason `:1128` gives: the mover re-clamps. Worth keeping as written
  and not re-raising.

---

## What could not be verified

- **No behavioural claim here is driven.** Every consequence that needs the game (the outside-line
  pass, the 4.2 m side disagreement, the nose clip, the three-abreast blackout) is marked PROBABLE
  at that step and rests on the arithmetic that precedes it.
- **This surface has no log line.** Nothing in `ApplyRivalWalls`, `ClampLaneToWalls`,
  `ComputeAvoidAheadLane` or `TryPickAvoidanceSide` writes to the log, and no debug readout carries
  the wall flags, the two clamp bounds or the picker's answer. Every "how to reproduce" above
  therefore begins with an instrument that does not exist yet. `Show Rivalry` draws the walls and
  their held/released colour (`AutosportRacingSystem.cs:2066-2106`) but not the numbers. The
  cheapest instrument for all four items is one throttled line per car: `LateralGap`,
  `r.NodeOffset - nodeOffset`, `RouteGapAhead`, `leftConstrained/rightConstrained`,
  `_avoidLeftWall/_avoidRightWall`, `clampLeft/clampRight`, `clampLeft - clampRight` and
  `_passLaneLatched`, printed at 1 Hz and once on every state change.
- **The exact band of 206-02 at the timid end is arithmetic, not transcription.** The 5.20 m
  cross-check against 04's measured "zero-width" point is at 1.2 m half-widths and Aggression 100;
  the 6.20 m figure at Aggression 0 was not run through the offline transcription.
- **Nothing was invented to fill a row.** Where a claim needed a factor this engine does not model
  (tyre state, fuel, wear), it was left out; every number above is a constant read in the source or
  arithmetic on those constants.
