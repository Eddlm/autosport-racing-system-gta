# VERIFY IN GAME — outstanding

Build deployed: **2090** (`4b1ee31c966b`). **AS** is verified (2026-11-26) and **AR** visually verified; every other section from **AF** up is **compile-only** — **AQ**, **AP**, **AO**, **AN**, **AM**, **AL**, **AJ**, **AI**, **AH**, **AG**, **AF**. What is ✅ is yours, verified in game; what is ⬜ is still **compile-only**.
Tick a line with `[x]` or tell me «verified N» and I update this file. Quotes in «…» are your own words,
translated where you spoke Spanish.

**Section P is closed** (all eighteen verified). **78 open rows below**; everything already driven or disposed of
sits in **Answered** at the bottom, out of the way, with **81** closed rows under it. The detail of the 30-10 hunt
stays in `docs/audit/410-hunt-2026-10-30.md`.

---

## AS · 2026-11-25 (2087 → 2090) — the route term becomes a per-section target speed

✅ **Verified in game 2026-11-26** (your words: «Testing the build, the system now works as
intended, so the core is proven»; specifically: «the car does follow the chosen speedpoint's
speed, and only when close enough. the same car that crashed before, does not crash now, as it
slows down appropriately» — seeded targets and the 2 s trust both read in the drive).

Behind the same persisted `Route Speed Points` checkbox — default OFF as shipped in 2087 (static default, `KeySpec`
seed and stored ini line all False, so the A/B was a menu flip; you flipped it back ON to test) — the route term is
no longer the physics base plus an offset: it IS the per-section speed. Each seeded point (every 50th node) takes
the **tightest swept radius within half the seed spacing on each side** (25 nodes each way at spacing 50 — adjacent
points' windows tile the route wherever points survive; pruned ground near corner apexes falls back to the old
base) as its own `Radius` — your overshoot law, sharpened in
2088→2089: seed points were leapfrogging the tight stretches between them because each trusted only its own node,
and a first 25-node-total window still left the middle of every gap unseen — and stores `TargetSpeedMps` from
`RouteIdealSpeedForRadius(point.Radius)` — the route's own ideal-speed law: apex speed with downforce below the
cap radius, above it rising with radius at the dialled exponent (`RouteSpeedCapRadiusM` 100 m anchors the pure
apex speed, `RouteSpeedRiseExponent` 0.25 the step; the cap collapsed the five-knob taper in 2098, the exponent
replaced the flat floor in 2099 — a cap could not tell 200 m from 400 m) — written once per route revision,
bounded 0 to the intended-speed ceiling. The seeding is
**generate-then-prune** (your restructure, 2090): every candidate seeds unconditionally — deep straights carry the
999 no-bend sentinel and clamp to the ceiling (the old [150, 900] band ate 22 of 39 LSIA candidates and left a
~1.3 km straight dead zone, measured by replicating the pass on LSIA Test Track) — then a prune pass removes
points with a **detected corner apex within the sweep's own reach plus one node (±26 at spacing 50)** — your rule,
confined from the first whole-spacing shape: a point near a corner hands that ground to the corner system. Each
baked target also carries the **downhill only**: the most inclined trackpoint in the same ±half-spacing window the
radius lookup uses, travelled forward, damps the target through the same hill law the old route term carries
(√factor, the law the pitch menus tune); uphill never raises the target and crest stays out (your call, 2101).
The prune logs one `[ROUTESEED]` line (seeded → kept) per racer per route revision, so the density is visible in
ARS.log. The respected speed is smoothed: it rides a SmoothFloat at rate 6/s (the pedal gauges'
law) toward the section target, so point switches taper instead of stepping; first service snaps
and the apex handover ramp chases the smoothed value. While the switch is on the old
route speed system (its physics base, its scalar offset row and the surface/slope/crest scaling it carried) is
decoupled; a tick with no active point falls back to the old base, so the car never stands down to 0. Off is the
old term, untouched.

Readouts: the four world labels and the subtitle's third line print the point's target speed in m/s (the advance
log prints `tgt` where it printed `off`); the subtitle's `r` now reads the window's tightest radius. The cursor
still keys to `LookAheads[LookAhead.OneSec]` per your ruling; on top of it a point is **trusted only inside the
car's 2 s horizon** (your rule, 2090): the selected point must sit within [0, the `LookAhead.TwoSec` distance —
rebuilt each tick from the car's speed] and not behind it; outside, or when a passed point is stuck on a
point-to-point tail, `ActiveRouteSpeedPoint` returns null and the old base owns the term. That ends the
straight-cap defect — a point whose radius bakes a curve's begin can no longer cap a straight seconds early, and
the subtitle shows `route points: none` during the gap. The speed-to-input chain
(metric check you asked for) is m/s end to end: `TargetSpeedMps` → `routeTerm` → `Speed` → `ConvertSpeedToPedals`
comparing against `VehicleData.SpeedVectorLocal.Y`, no single mph call inside it — the only mph conversions are the
menu rows', made once.

Pass condition: switch on — the car holds the seeded speeds through the tight stretches between seed nodes too (no
more leapfrog overshoot), straights seed and let the ceiling speak for them, curve approaches cap in only ~2 s
before the trusted point (the subtitle reads `route points: none` until then), the labels show real speeds, and
the subtitle's third line shows `n/x r/tgt` advancing; switch off — today's baseline returns unchanged;
`[ROUTEPOINT]` lines keep firing on advances, now as `tgt`, and `[ROUTESEED]` names the table size per revision.

---

## AR · 2026-11-25 (2086) — per-node route speed offsets, wired inert

◐ Visually verified in game (your words, 2026-11-25: «visuals working well»); the readout half of the pass
condition is met, the no-behaviour-change clause below is still yours to judge.

The AI's route term now follows a per-racer table of node-keyed speed offsets alongside the existing global
scalar. Every offset is 0 and the persisted `Route Speed Points` checkbox (on by default) gates only the
addition, so the speed plan reads exactly the number it read before. Phase 2 is the learner that writes the
offsets; the old global route learner stays off.

Seeding walks every 50th node, keeping those with `SweepCurveRadius` in [150, 900] m, in track order. One
predicate picks the active point: the first one the 1 s lookahead has not crossed; the cursor advances while
crossed, wraps on circuits, stops at the last point on point-to-point tracks. Cleared at race init, rebuilt on
`ARS.CornersRevision` changes.

A new `Show Route Speed Offsets` debug toggle draws the active point's offset and the next three as text at each
point's position (wrap-aware on circuits, stops at the last point on point-to-point tracks).

Pass condition: with `Show Curve Radius` on, the subtitle's third line shows an advancing
`#index / node / radius / +0.0mps`, version-stamped `[ROUTEPOINT]` lines appear on advances and wraps, and
the car drives exactly as it did — no pedal pulse or speed step at a handover.

---

## AQ · 2026-11-24 (2050) — the route offset is one number, seeded by the row and bounded at ±10

⬜ Not driven. Compile-only.

The route learner used to move a share that the menu row was added to, floored at zero: the row was a baseline
and the learner could only add to it, with no ceiling at all. It is one number now — the row seeds it when the
racer initialises, the learner moves it from there, and the route term reads that single value. A row set at the
top of its ladder no longer starts the car twice as far up as the row says.

Both moves ride one bound, `RouteSpeedLearnLimitMph`, the row's own ladder in the row's own unit, applied in one
setter (`SetRouteSpeedOffset`) so the rise and the fall cannot disagree about it. **The fall can cross zero now**,
which it could not before: a car that keeps washing out under a raised row walks the offset back down past the
row instead of parking on it.

The row stays live: moving it mid-race shifts the offset by the difference and carries whatever the learner had
earned (`_routeOffsetSeedMph` holds the value that shift was last taken from), so a menu change lands without a
reload.

What to judge: whether the car now holds a raised row instead of drifting past it, and whether a fall that can go
under the row over-slows a car that washes out once.

---

## AO · 2026-11-24 (2027 → 2048) — the route speed rise reads the projection, not a radius band

⬜ Not driven. Compile-only.

The rise had a proxy: the car's own path radius against the track's under it, rising while that sat in a middle
band. It now reads the same quantity the fall reads, from the other side. `ProjectionWideningWithinLimit` takes the car's own
projection against the car in lateral metres — the same `_projectionWidenMeters` the fall reads, measured a second
ahead — and rises while that number is above zero and under `RouteSpeedLearnWidenMetres`: the projection outside the
car, so the car is using the road, but short of the wash-out line. At or past that line the fall answers. One
measurement, one threshold, and its two sides.

**The two offsets are lane against lane (2032).** The widen first compared the projection against the car's own
node frame, where the road's curvature dominates: a ballistic projection a second out on a tight bend reads several
metres off the centreline while the car reads near it, so the number sat past the widen line on every corner, the
rise band was unreachable, and the subtitle stayed red while the fall stayed armed. Both offsets now come from their
own nearest node — the projection's lane against the car's lane — so the number moves only when the car's own line
does. The subtitle changed with it, since it prints the same number.

The band, its two constants and the car-over-track percentage are gone with it: the radius pair, its clamped-open sentinel and the helper that measured them are deleted, and the tuning subtitle prints the widen itself in metres — green when it is inside the rise threshold, red when it is past the fall's.

The gates, in order: off corner mode, the route term in charge, `ThrottleDemand` under `RouteSpeedLearnThrottle`
(the ask, never the pedal, and nothing already limited by a cap), brake off, the projection inside, a slide of at
least `RouteSpeedLearnMinSlideShare` of the peak — a car that is not loading its tyres is not evidence — and the
stability cap whole. The rise is `RouteSpeedLearnRateMph` per second with no upper clamp.

The fall is untouched: washing out, at half the rise's rate, floored at zero. It still runs in corner mode, which
is where washing out mostly happens; only the rise is kept off the corners.

What to judge: whether the offset now climbs on roads it used to leave alone, and whether a car merely pointing
inside at low slide earns speed it has not demonstrated.

**What changed after the first drives (2035 → 2048):** the demand gate became "the route is lifting the car" — a
demand pinned at full means the car is still chasing its target — and the steering gate is gone, because the
projection test already implies the wheel is doing something. The widen itself is now read at
`SteerPreviewSeconds`, the horizon the steering aims at, instead of a full second: a ballistic second cuts metres
inside any bend (the sagitta, `v²t²/2R`), which made the band unreachable and the subtitle red. The node is the
nearest in 2D within two seconds of travel. Measured after the change: the offset climbs 0.0 → 0.2 mph in 37 s of
ordinary driving — it works, it is simply starved of windows, which is the open question.

---

## AP · 2026-11-24 (2046) — the verdict's metre is a deadzone, and the width stops being subtracted twice

⬜ Not driven. Compile-only.

`CornerSpeedLearnVerdictMarginMetres` no longer shifts the balance, it silences it: a pass whose worst clearance sits
between zero and the margin — inside the paint, within a metre of it — writes nothing and announces nothing. Past the
margin the room beyond it is the rise, at `CornerSpeedLearnMetreToOffset`; a clearance under zero, the car's outer side
over the line, is the deduction at the same scale. Off the track is clearance under zero, not under the margin: the
clearance is already measured from the outer flank.

The old expression also took `VehicleData.SlidingWidthMetres` off the clearance a second time — `outsideReach` already carries
half the bounding box — so the width is now counted once, where it is measured, and the metre is the only band left.

What to judge: whether a car holding the paint now learns nothing at all, which is the point, and whether the
deductions begin where the tyre actually crosses the line.

---

## AN · 2026-11-24 (2025) — one verdict moves a corner by 2 m/s at most

⬜ Not driven. Compile-only.

The per-pass step is `CornerSpeedLearnDeltaLimitMps`, now 2 m/s in both directions, so a deduction is capped at 2
as well — the off-track hit pays the same limit and deducts 2 for it. The clean-pass credit
`CornerSpeedLearnNoSlideRiseMps` comes down with it: it was already clamped by the step limit, so the clamp did the
work while the constant claimed a rise it never got.

The scale underneath is unchanged in shape and halved in rate: the verdict reads the pass's worst outside clearance in metres — the gap from the car's outer side to the paint — and converts at `CornerSpeedLearnMetreToOffset` — half a metre per second for each metre of room past the margin, so it takes six metres of clearance to reach the step limit. The stored offset keeps its ±10 m/s clamp.

What to judge: whether a corner now takes too many passes to walk up to its limit, and whether a genuinely wide
pass still reads as a deduction rather than a nudge.

---

## AM · 2026-11-24 (2020 → 2024) — the throttle stability cap: per wheel down, a bounded hold, one rate up

⬜ Not driven. Compile-only.

The cap falls at `StabilityFallPerWheelPerSecond` for every wheel off the ground, so one lifted wheel costs a
quarter of the range per second and four of them cost the whole range at once. Its bottom is `StabilityFloor`
below zero, and that depth is not a pedal but a memory: the deeper it goes the longer the throttle stays down
once the wheels are back, at one second per unit of `StabilityRisePerSecond`, which is the only rate the return
has. Stability's own output is floored at `StabilityThrottleFloor` where it enters the composition, so however
deep the hold goes the pedal it asks for is never below a tenth and never negative; every other cap keeps its
own right to cut to nothing.

Both bounds answer the same case: off the road a supercar keeps a wheel up much of the time, so an unbounded cut
would hold the pedal at nothing for seconds at a stretch, and a cut with no depth to it would forget the lift
the moment the wheel landed.

What to judge: whether the hold now reads as a real cut after a kerb without strangling the car off the road,
and whether the per-wheel fall is too slow to notice when only one wheel is skipping.

---

## AL · 2026-11-23 (2011 → 2014) — the route ask is capped by a speed-ramped angle, and the outside metre cap is gone

⬜ Not driven. Compile-only.

Two changes on one pass. **The route ask may not point more than a speed-ramped cap off the route at the aim's own node**:
inert below `20 mph` (`RouteSteerAngleCapMinMph`), `45°` there (`RouteSteerAngleCapWideDegrees`),
tightening linearly to `5°` at `60 mph` (`RouteSteerAngleCapTightMph`, `RouteSteerAngleCapTightDegrees`) and flat
above it. It stands down while the car is off the track (`SignedMetresBeyondTrackEdge() > 0`) and while corner mode holds,
and it is **absolute**: the angle is measured at the aim's own node, whatever angle the car itself carries,
so a car already crossways has its aim pulled back inside the band rather than earning a wider one. The angle is measured against the track direction at the **aim's own node** (`steerRefPoint.Direction`), not the
tangent under the car: on a 100 m bend the tangent turns 7.7° across a 13 m lookahead at 60 mph, 12.8° on a 60 m
bend, so capping against the car's node trimmed the racing line itself and held every car out of the corner — the
reported "can't steer in". Against the aim's node, following the road costs nothing and only real lateral
positioning is capped. The pursuit consumes a bearing measured off the car's course, so the capped angle is
converted afterwards (`bearing = clamp(aimVsRoute) − headingVsRoute`).

Both ranges are written low-to-high because `Remap`'s clamp is `Clamp(r, out_min, out_max)`: passing a descending
output range (`45` → `5`) makes the clamp collapse to a single value. Lateral budget = speed × sin(cap): `7.7 m/s`
at 30-40 mph, `2.3 m/s` at 60, `3.9` at 100 — the tightest point is 60 mph, and above it the flat 5° floor lets the
budget creep back up with speed.

**The outside metre cap and its creep are deleted**, not parked: `LanePrepOutsideCapMetres` (`2`),
`LanePrepCreepMetres` (`0.1`), `LanePrepRampSeconds` (`3`), `_laneOutCap`, `_laneLead`, `_laneAskCornerNode`,
`_laneAskStartMs`, `_laneAskLastMs` and `LaneAskGapMs` are all gone, and the following branch's ask is simply the
aim. The lane log line lost its `lead`/`cap` fields with them. How fast a lane is taken is now one law, in degrees.

⚠️ **Overlap the owner should know about:** `LaneCrossingScale` (`false`, `Racer.cs`) already implements a
sideways-*speed* allowance — `LaneCrossingThresholdDegrees` = `asin(LaneCrossingLateralSpeedMps / speed)`, applied
as a scale rather than a boundary, and the DESIGN crossing section argues a scale reads softer than a cap. The two
shapes cover the same quantity: if the 15° cap reads harsh, the parked scale is the alternative, not a new build.

**The inside lean is an angle ramped by radius** (build 2017): nothing at `LaneLeanMaxRadius` (`400 m`),
`LaneLeanMaxDegrees` (`20°`) by `LaneLeanMinRadius` (`100 m`) and flat inside it — so the old 400 m gate is now a
zero rather than the cliff a whole-lane ask turned it into. The angle is walked into an offset over the lookahead
(`lookahead × sin(angle)`), so the ask scales with speed and saturates at `laneLimit`; the wall clamps and
`ApplyRivalWalls` are unchanged. `LaneInsetPerRadius` (`125 / radius`) and the TEMPORARY full-lean switch are both
gone. `Remap` is called with the input descending (400 → 100) so its clamp receives an ascending output pair.

AI only, as with every steer law: `ComputeSteering` runs inside the `!ControlledByPlayer` branch.

---

## AJ · 2026-11-23 (2004 → 2007) — the overspeed detector arms on forward acceleration, one piece of code for player and AI

⬜ Not driven. Compile-only.

The arm gate was wrong twice over. It branched on `ControlledByPlayer` — measured lateral Gs for the player, steer
angle for the AI (`OverspeedArmMaxSteerDegrees` `2`) — so the human and the field could disagree about what
“straight” means on the same road; and it asked whether the car was *going straight* at all, when the quantity
being judged is a **forward** one. Both cars now arm on the same axis and the same measurement as the excess
itself: `GetLongitudinalGs > OverspeedArmMinForwardGs` (`0.1 g`, genuinely pulling), slide under
`OverspeedArmMaxSlideDegrees` (`2°`), speed over `OverspeedArmMinSpeedMph` (`30`). Nothing in the gate asks about
straightness or about who is driving; the steer and lateral-Gs constants are deleted.

`GetLongitudinalGs` is the mean of the last ten 20 ms velocity differences — a 200 ms window, so roughly 100 ms
stale — projected on the car's current horizontal forward axis, which is also the number the excess is built
from. The gate and the measurement are therefore the same quantity, one frame-late but coherent.

The level is still *spent* differently, and that is structural rather than stylistic: the AI's level caps its
pedal (`UpdateThrottleReasonCaps` runs for AI only, where `Control.MaxThrottleFromOverspeed` is one of the
ceilings), while the player's cuts engine power (`UpdatePowerCompensation`, the `if (ControlledByPlayer)` branch),
because the mod never writes a player's pedals — `ApplyInputs` is AI-only. Same detector, different lever.

Recovery is its own law since 2007: the penalty heals at `OverspeedRecoveryPerSecond` (`1.0`/s) on the AI's pedal
cap and at `OverspeedPlayerRecoveryPerSecond` (`0.5`/s) on the player's power cut, where every other reason cap
heals at `1.75`/s (`ReasonCapSlewRate` `3.5` × `ReasonCapRecoveryScale` `0.5`). `GlideCap` gained a rise-rate
overload for it; no other cap moved, and the onset is still `3.5`/s.

---

## AI · 2026-11-23 (2003 → 2008) — the throttle stability cap, parked off for a few builds and put back

`TrackSpeed.StabilityCap` went `false` in 2003: lifting a wheel stopped lowering
`Control.MaxThrottleFromStability`, so the cap sat at 1, never bound against TCS, and the route learner's
stability test always read stable. Back `true` on his word after driving that build — the mechanism answers a real
case and stays as written. Nothing about it changed, so there is nothing new to judge beyond the cap being live.

---

## AH · 2026-11-23 (2000) — a hit voids the pass, and a slide past 20° voids the deduction

⬜ Not driven. Compile-only.

Two ways a pass stops meaning anything, and neither was being noticed. A **contact** is a step of more than three
g across the two horizontal axes inside one 20 ms acceleration sample (`ContactJumpGs` `3`) — grip alone never
produces that, a hit does. It stamps `Racer.LastContactTime`, and `MsSinceContact()` reports how old it is. The
test at the verdict is one number: `MsSinceContact()` under `4000` ms (`CornerSpeedLearnQuietContactMs`) means
the pass is poisoned and no verdict is given — nothing written, no lock taken, the notice reading `hit - no
change`. Four quiet seconds and the pass is good, and never having been hit reads as infinite and passes.

**The exception (2019):** a hit that landed while the car was **off the road** is not bad data to discard — it is
the corner being too fast, and voiding it is what keeps a wrong speed stuck on that corner for good. So
`MsSinceContact() < 1000` (`CornerSpeedLearnHitOffTrackMs`) together with off the track is **latched** for the
pass, and at the commit it pays the step limit (`-2 m/s`) instead of being voided; the notice reads
`hit off track - -2.00 m/s`. It is latched while the window ticks rather than tested at the commit, because by
then the car may be back on the surface and the contact older than a second. No lock is taken, so a later pass
still judges the corner normally. It cannot tell a wall the car chose from a rival that punted it off — both pay.

A **slide past 20°** during the window (`CornerSpeedLearnLostControlDegrees`) means the car was out of control
rather than out of corner, so the pass cannot take speed away: the delta is floored at zero and a rise still
stands. The reason line reads `lost control N°` and the log adds `lost control, deduction voided`.

---

## AG · 2026-11-23 (1995 → 1999) — the corner verdict's balance sits a car's width short of the line, and the side comes from the corner

⬜ Not driven. Compile-only.

The offset could only fall once the car's outer flank had reached the white line, so a pass half a metre off the
paint still earned a rise: the balance rewarded being at the very limit. The comparison now carries a car's width
plus one metre (`CornerSpeedLearnVerdictMarginMetres` `1`, read with `VehicleData.SlidingWidthMetres`), so the zero sits
with the flank that far short of the line, and inside that room the pass is already a deduction. The stored
clearance stays raw — it is what the reason line quotes — and the margin is applied once, where the delta is
computed.

The cadence is coarse on purpose (1996, 1997): one metre of room is one metre per second of offset in either
direction (`CornerSpeedLearnMetreToOffset` `1`, was 0.66), and a pass that never slid at all earns `2 m/s`
(`CornerSpeedLearnNoSlideRiseMps` `2`, was 2.5 from 1999, 4 for one build) — a corner that produced no slide taught nothing
about its limit, so it moves by the coarse step rather than by a measured clearance. Since 1998 one pass may only
move a corner by `±2 m/s` (`CornerSpeedLearnDeltaLimitMps`, 2.5 until 2025): a step limit on the move, separate from the `±10`
the stored offset keeps, because a wide track hands a single pass enough clearance to swing a corner by a dozen
metres a second at once. The no-slide credit is set to that same limit, so the number in the code is the number
applied.

The side is read from the corner doing the judging (`CornerOutsideSign`, the corner's own `Angle` through
`ARS.InsideSign`), never from the curvature the car happens to sit on when the sample lands: an S-bend flips the
track's sign under the car between one sample and the next, and the corner that asked for the lift still owns
which side is outside. Where the apex angle reads zero the sign falls back to the corner's own span, the
`SignedAngle` of the direction at its `StartNode` and its `EndNode`.

The notice is unchanged in shape: the reason is still the physical metres the outer flank had to spare, and the
red follows the sign of the move rather than offtrack. The log line now carries both numbers
(`closest N.N m · verdict N.N m · <delta> m/s`), so a deduction the margin caused can be told from one the road
caused.

---

## AF · 2026-11-23 (1970 → 1971) — the corner owns the speed only to its apex, then a 1 s ramp hands it to the route

The corner-mode latch was one flag doing three jobs. Build 1971 splits it: the *speed authority* (`_cornerSpeedAuthority`) is the corner owning the speed, from the tick it latches until its apex; the *corner state* (`_cornerMode`) is the verdict window, alive `CornerVerdictMs` `2000` ms past the apex — the corner-speed learner's sampling window, and the reason `_brakePlanActive` stays set. At the apex the authority ends and a ramp takes the number over: over `ApexRampMs` `1000` ms the speed is `held + (route − held) × (elapsed / 1000)`, so the handover is a slope, not a step, and at the ramp's start the number still is the held speed. While the ramp runs the term reads `SpeedTerm.CornerToRoute` — `corner-route` in the log and subtitle — so the route-speed learner, which skips unless the term is `Route`, cannot learn off a blend. The verdict commits when the window closes, so the learner now gets 2 s of post-apex samples instead of 750 ms. The entry gate moved from `_cornerMode` to `_cornerSpeedAuthority`, so a following corner — a chicane, an S-bend — still latches while the previous corner's verdict window is open; the old special case 'a nearer corner that wants less takes over' is gone, and the corner already being served is excluded by name (`cornerSourceNode != _cornerModeNode`), because it sits inside the picker's six-metre tail right after its apex and re-entering it would aim the apex count at a node behind the car. Build 1980 rewrote the trigger — his words, in full: «As soon as the rampdown asks for a lift, we forego the route speed. That's it. While ignoring route speed, IF the car dips 25 m/s below the ramp/target, not timed, then we deselect the corner.» The entry test is `cornerSpd < ARS.GetForwardSpeed(Car)` (with `cornerSpd < 998f` excluding the no-ramp sentinel and `cornerSourceNode >= 0` requiring a source node): the corner takes over on exactly the tick its own braking envelope — `cornerSpd`, the ramp down to the latched apex — drops below the car's forward speed, i.e. the moment the ramp asks for a lift; the envelope is below the car's own speed on that tick, so taking over can only ever slow the car, and the route term plays no part in the entry. While it holds, the route term is not consulted at all. The only pre-apex exit is `cornerTerm - ARS.GetForwardSpeed(Car) > CornerDeselectMps` (`25` m/s), untimed — no duration, read every tick — logged as `EXIT to route (car fell well under the corner)` and committing no corner-speed verdict (`judge: false`), because the corner was never driven at its own speed. The deselect is not '25 below the ramp' in the old sense: the envelope sits far above the car for most of the approach — a measured log read the envelope at `121` m/s with the car at `0`, `475` m ahead of the apex — so at takeover the gap starts near zero, and a 25 m/s gap means the car has since fallen well below what the corner asks, something else already keeping it slow. `RouteHoldReleaseMs` (the `250` ms rule) and its `EXIT to route (route is the higher number)` release are deleted in this build. A corner taking over cancels the ramp. Why the ramp: the route term is computed from the tightest radius in a 1-2 s band ahead, so just past the apex that band still spans the corner and the route term sits at its tightest — handing back at the apex without a ramp would step the speed down on every exit; the ramp removes the step in both directions, because the route term can also sit above the held speed. `CornerModePostApexHoldMs` is deleted, the exit line's `why` reads `verdict window closed`, and the `APEX` line now prints `route <n> m/s` beside `held <n> m/s`, so the ramp's endpoints are checkable from `ARS.log`. The throttle cause and the source name both read `corner-route` while the ramp runs, so the state is visible from the seat without the log.

| # | What | How | Passes if | State |
|---|---|---|---|---|
| AF1 | **the handover is a smooth ramp, not a step** (1971) | watch the speed readout through one braking apex, then read the `mode … APEX` and `EXIT` lines in `ARS.log` | the readout leaves the held speed continuously and lands on the route term over the 1 s after the apex instead of stepping; the `APEX` line prints `held <n> m/s` and `route <n> m/s`, and those are the ramp's endpoints | ⬜ |
| AF2 | **a chicane still latches while the previous corner's verdict window is open** (1971) | an S-bend or a chicane taken at speed, `mode` lines visible | the second `ENTER` lands while the first corner's 2 s verdict window is still open — never a tick on the pure route term in between — and taking over cancels the first corner's ramp | ⬜ |
| AF3 | **the corner verdict still lands once, 2 s after the apex** (1971) | one corner on consecutive laps, `Corner Speed Learn Notify` on and `mode` lines visible | exactly one `corner speed` line per pass, landing when the verdict window closes 2 s after the apex with the `EXIT` line reading `verdict window closed`, and the next pass starts from the number just written | ⬜ |
| AF4 | **the corner takes over the tick its ramp asks for a lift** (1980) | a braking corner approached from above the corner's speed, `mode` lines visible in `ARS.log`, the `pre`/`ENTER` pair read against the timing | the `ENTER corner #n` lands before the apex on the tick the corner's own braking envelope (`cornerSpd`) drops below the car's forward speed — the `pre` line at that tick shows `corner` below `car` — and `route` takes no part: an `ENTER` whose `pre` line shows `corner` above `route` is still a legal takeover, and no route-term movement on the approach starts or defers it | ⬜ |
| AF5 | **the corner lets go only when the car falls 25 m/s under its demand** (1980) | a corner whose hold the route term alone would have lifted before the apex — an open bend whose route band reads wider, or an entry that eases — with `mode` and `corner speed` lines in `ARS.log` | the corner holds through a `pre` showing `route` the higher number, and no `EXIT` lands on that alone; the only pre-apex exit reads `EXIT to route (car fell well under the corner)` with the car sitting 25 m/s (`CornerDeselectMps`) below the corner's demand, needing no timer, and the pass commits no `corner speed` verdict line — `judge: false`, the corner was never driven at its own speed | ⬜ |

## AE · 2026-11-23 (1967 → 1970) — the route offset falls when the projection lands outside the car

The route learner had a rise and a fall, but the fall sat behind `const bool RouteSpeedLearnFallEnabled = false`: the offset could only climb, for the whole session, with no reset anywhere. Build 1967 gave it a live fall; build 1970 replaced the law with his — the car's own projection is more to the outside than the car while the wheel is turned inside. `MeasureProjectionOutside` (`Racer.cs:2020`) already runs at `:1896`, before the learner at `:1949`, and now also publishes `_projectionWidenMeters` (the ballistic projection's distance from the centreline minus the car's) and `_projectionWidenSign` (the side it drifted to); `_projectionWidening` is that difference being positive. `IsWashingOut` asks for `Control.ThrottleDemand >= RouteSpeedLearnFullThrottle`, `|Control.SteerInput| >= RouteSpeedLearnSteerMin` and `_projectionWidenMeters >= RouteSpeedLearnWidenMetres`, with the steering sign opposite `_projectionWidenSign` — wheel toward the inside, trajectory toward the outside. `ProjectAhead` curves with the car's own `AverageAcceleration`, a full 3D vector, so the widening is real drift rather than the chord-versus-arc residue, which is centimetres. The fall runs at `RouteSpeedLearnFallShareOfRate` of the rise (half), floored at zero, and is tested before the rise so one tick cannot do both. The throttle read is the demand, per the 2026-11-22 backlog note: a cap that pulled the pedal down did not make the plan want less.

| # | What | How | Passes if | State |
|---|---|---|---|---|
| AE1 | **the offset comes back down when the projection widens** (1967 → 1970) | a corner the AI arrives at too fast: full demand, wheel inside, the projection drifting out; `Show Curve Radius` on for the offset subtitle | the offset falls at half the rise's rate while the projection is more to the outside than the car, and the next pass runs less wide | ⬜ |
| AE2 | **ordinary cornering never lowers it** (1967 → 1970) | any clean committed corner, part throttle | the offset does not move; only the full-demand widening state pays | ⬜ |
| AE3 | **it cannot go below zero** (1970) | a corner the AI cannot take at any speed, driven repeatedly | the offset bottoms out at 0.0 and never goes negative | ⬜ |

## AD · 2026-11-23 (1960 → 1962) — the apex pass is wrap-aware, and both pickers stop at the last corner

`HasPassedApex` now keeps the lap-local odometer test and adds the occurrence test: the apex must also be behind the car (`ForwardNodeDistance(apexNode) > TrackPoints.Count / 2`), so the first corner of the next lap is not read as already passed while the car is still on the run to the line. `NearestApexAhead` also finds the current lap's last corner and refuses to consider corners beyond it until that last corner's apex has passed; once it has, the first corner of the next lap is allowed. `ComputeTargetSpeed` applies the same last-corner gate to the queue's second and third corners, and its empty-table fallback now prefers the queue head's ramp, so a stale braking flag on the next lap's first corner cannot skip the last corner of the current lap.

| # | What | How | Passes if | State |
|---|---|---|---|---|
| AD1 | **no next-lap corner is released as passed** (1960) | a circuit whose last corner sits just before the line; drive the final corner and the run to the line with `Plan/Lane chose/released apex` visible | the lane chooses the next lap's first corner only after the last corner's apex; no repeated choose/released pair for the same upcoming corner at the line | ⬜ |
| AD2 | **the line still lets go at the apex** (1960) | a corner followed by a long straight | the lane releases on the apex tick, as before; the extra occurrence test does not hold the line past the apex | ⬜ |
| AD3 | **the speed plan does not skip the last corner** (1962) | a circuit whose last corner sits just before the line; drive the last corner with `mode` and `Plan chose apex` visible | the plan enters corner mode on the last lap corner (for this track, node 2164) while it is still ahead, not on the next lap's first corner (252); no `ENTER corner #252` with the last apex still ahead | ✅ — «You fixed it now, very nice» |

## AC · 2026-11-23 (1954 → 1956) — corner speed learning measures the car's actual clearance past the apex

The peak-slip-share verdict is gone. `UpdateCornerSpeedLearning` and `CommitCornerSpeedLearning` (`Racer.cs:2125-2197`) keep one offset per corner (`CornerContext.CornerSpeedOffsetMps`): it starts at 0, is held inside ±10 m/s, and is added to that corner's planned speed. The window is the corner-mode latch itself (see **X**): the learner tracks the corner that corner mode entered, under the same lock rules, and commits when corner mode lifts or hands off to a stricter corner. Inside it a sample is taken only once `HasPassedApex(_cornerSpeedApexNode)` says the car is past the apex, on a new acceleration sample (`VehicleData.LastAccelSampleTime` has advanced), with `TRLateralAtSpeed` usable (`> 0.01`) and `|SlideAngle| / TRLateralAtSpeed >= 0.75` (`CornerSpeedLearnMinSlideShare`). The sample is the car's own position, not a projection: `outsideSign = Math.Sign(context.Point.Angle)` picks the corner's outside, `signedOffset` is `SignedLaneOffset(Car.Position, CurrentTrackPoint.Position, CurrentTrackPoint.Direction)`, `outsideReach = signedOffset * outsideSign + VehicleData.SlidingWidthMetres * 0.5`, and `clearance = max(CurrentTrackPoint.TrackHalfWidth, 0.1) - outsideReach`; the pass keeps the smallest clearance. At commit `delta = leastClearance * CornerSpeedLearnMetreToOffset` (0.66 m/s per metre; negative when the car's outer side was off the road, positive when inside); the resulting offset is clamped to ±10 m/s, and a pass with no qualifying slide sample earns `+2 m/s` (`CornerSpeedLearnNoSlideRiseMps`). There is no upper slip-share cap, no six-second cap, no back-on-power test, no ±3 m/s step and no traffic or off-track veto. `CornerSpeedLearning` gates the learner and `CornerSpeedLearnNotify` only the notice — the log always receives the line. The line is brief: `<corner> · closest N.N m · <signed delta> m/s`, where the metres are the least clearance over the qualifying samples and the delta is the move actually applied after the clamp; the red `· ~r~offtrack~w~` suffix follows when that least clearance was negative, and `· ahead +N m/s` when a rise was shared. A pass with no qualifying sample reads `<corner> · no slide · +2.00 m/s`. The lock clears 40 m past the corner.

| # | What | How | Passes if | State |
|---|---|---|---|---|
| AC1 | **the verdict is the car's actual clearance** (1956) | one corner driven twice, once held comfortably inside and once with the car's outer side running off the edge past the apex, `Corner Speed Learn Notify` on | each line names the corner, then `closest N.N m` and a signed delta; the pass whose `closest` figure is negative appends the red `~r~offtrack~w~` marker; no `projection` or `projected` wording and no `peak slip`, percentage or band word appears | ⬜ |
| AC2 | **off the road costs speed, room earns it** (1956) | the same corner on consecutive laps, the first pass sliding wide of the edge and the second sliding with room to spare inside | the off-road pass announces a negative delta with the offtrack marker, the inside pass a positive one; each move is the least clearance times 0.66 m/s per metre, added to the offset and clamped to ±10 m/s, and the next pass starts from the number just written | ⬜ |
| AC3 | **no slide sample still nudges** (1956) | a corner taken at ordinary pace with the slide under three quarters of the modelled peak | the line reads `<corner> · no slide · +2.00 m/s`, and the offset rises from 0 to +2.00 | ⬜ |
| AC4 | **the clamp holds at ±10** (1956) | a corner already at +10 m/s whose pass keeps a positive clearance, and one at −10 m/s whose pass still runs its outer side off the road | both announce `+0.00 m/s` — the move is clamped away — and stay at ±10 m/s; the −10 pass still carries the offtrack marker | ⬜ |
| AC5 | **the verdict lands at the corner-mode lift, once** (1956) | one corner on consecutive laps; compare the notice with the `mode … EXIT to route` lines | exactly one `corner speed` line per pass, at the lift or the handoff to a stricter corner — not at a six-second mark, not on throttle pickup — and the next pass starts from the number just written | ⬜ |

## AA · 2026-11-23 (1951 → 1952) — superseded by 1954, see **AC**

The peak-slip-share verdict these rows tested is gone; none was driven, and all five sit in **Answered** as superseded.

## AB · 2026-11-23 (1953) — the apex-pass test is a lap odometer, not a half-route split

`HasPassedApex` no longer asks whether the apex lies in the rear half of the route (`ForwardNodeDistance(apexNode) > TrackPoints.Count / 2`). Each car now keeps a lap-local node odometer: `UpdateLapOdometer` adds the wrapped node delta between consecutive nodes to `_lapOdometer`, remembers the node and the odometer reading the lap started on (`_lapStartNode`, `_lapStartOdometer`), and resets that start on the 90%→10% line crossing (`_lapStartNode = 0`, `_lapStartOdometer = _lapOdometer`); the reset that prepares a car zeroes all four. `HasPassedApex` compares the odometer advance since the lap start with the apex's wrapped node offset from the start node, so a corner counts as passed only once the car has driven past it on this lap. The old test read any apex more than half a route ahead as already passed — just after the line, the corners from the end of the previous lap read passed for the first part of the new lap — and the lane's held-corner latch, the crossing-scale turn-in exemption, the apex queue shift (with `Tips.ApexPassed`), the skip-flag clear in `UpdateCornerRequirements` and the two card exits all read it — the corner-speed learner's window is the corner-mode latch, and since 1956 its samples wait for `HasPassedApex` too. The corner-mode counter in **X** is a different instrument and is unchanged, and `RaceOrderKey` is only the leaderboard sort key; point-to-point still compares nodes directly. Before the first line crossing the lap starts at the node the car initialized on, so a corner behind that node reads not-passed until the car comes round to it. Build 1960 adds the missing occurrence check: the same node repeats next lap, so the lap-local pass must also be behind the car (`ForwardNodeDistance(apexNode) > TrackPoints.Count / 2`), and the picker stops at the current lap's last corner until that apex has passed.

| # | What | How | Passes if | State |
|---|---|---|---|---|
| AB1 | **a lap boundary is not a corner boundary** (1953) | a circuit whose last corner sits just before the line; drive two laps at racing speed with the corner line visible and `Show Track Analysis` on | the corner line arms and releases at the same points on both laps, and no corner ahead of the car is treated as passed at the seam — the second lap reads like the first, with no dropped line and no skipped approach | ⬜ |
| AB2 | **the line still lets go at the apex** (1953) | a corner followed by a long straight | the corner line releases on the apex tick and the lean resumes, as before — the lap odometer neither holds the line past the apex nor drops it early | ⬜ |

## Y · 2026-11-23 (1934) — the brake aim is a fixed 15 m node

`BrakeAimDepth` is gone from the menu and settings; the plan brakes toward the first node centre at least 15 m behind the apex, selected by route distance. The old lag correction is gone as of build 1940; the ramp is a straight constant-deceleration solve from the selected node distance, with a provisional 4.5 m reduction applied to that distance. A stale `BrakeAimDepth` key is pruned by the settings repair.

| # | What | How | Passes if | State |
|---|---|---|---|---|
| Y1 | **the fixed target changes the braking entry** (1934) | one familiar braking corner | the plan starts from the fixed 15 m-back node; the car is not given the old speed-derived aim | ⬜ |
| Y2 | **the retired row stays gone** (1934) | open Rivals > Rival Pace after load | no `Brake Aim Depth` row is present, and `Menu-Settings.ini` has no `BrakeAimDepth` | ⬜ |

## Z · 2026-11-23 (1939 → 1944) — brake ramp learner with a single ideal

The old three-band learner is gone. Each braking corner now keeps its own `BrakeFactor`: seed `0.75` (2090, driver call from `0.90`), clamp `0.33`–`1.30`. The ideal overshoot is the shared hard-coded `Racer.BrakeRampIdealDeltaMps`, currently `0.5 m/s` over target at the fixed 15 m ramp-end node. The adjustment is `(BrakeRampIdealDeltaMps - delta) / 50`, so an arrival exactly at the ideal overshoot holds the factor; hotter arrivals lower it, cooler arrivals raise it. `BrakeLearning` gates it and `BrakeLearningNotify` gates the notice. Build 1940 removed the old lag compensation and solved the ramp directly from the remaining node distance, after a provisional 4.5 m reduction; build 1941 made zero the ideal, made the learning step `error / 50`, and changed the seed to `0.60`; build 1942 made the ideal `0.5 m/s` over target; build 1943 shared that value through one code constant used by the learner and menu description; build 1944 lowered the clamp floor to `0.50` so a fresh `0.60` seed can be corrected downward.

| # | What | How | Passes if | State |
|---|---|---|---|---|
| Z1 | **the learner fires and adjusts one braking corner** (1939 → 1944) | drive one braking corner with `Brake Ramp Notify` on, then revisit it | one line per pass with signed delta and `before → after`; below the shared ideal overshoot raises factor, above it lowers it, each by `(ideal - delta) / 50` | ⬜ |
| Z2 | **non-braking corners stay silent** (1939) | drive a fast sweeper that does not ask for braking | no `brake ramp` line and factor stays at seed | ⬜ |

## X · 2026-11-23 (1926 → 1950) — route is the default, a corner takes over on its ramp's lift demand

The two speed terms no longer meet in a min(): the route term decides where no corner holds the car, and a corner becomes the sole authority on the tick its own braking envelope drops below the car's forward speed — the ramp demanding a lift — not by any comparison against the route term (the entry law in **AF**). Build 1945 made the hold a fixed delay after the car passes the apex — `CornerModePostApexHoldMs`, three quarters of a second in 1969 (it was 15 m in 1926-1944); build 1971 deleted it — the speed authority now ends at the apex, a 1 s ramp over `ApexRampMs` hands the speed back, and the corner state runs on `CornerVerdictMs` as the verdict window (see **AF**). Build 1946 added forced `Info` lines for the apex-pass and corner-mode lift. The apex test is now a corner-local node odometer (build 1950; the global `_raceOdometer` of 1948/1949 is gone): `EnterCornerMode` zeroes `_cornerModeDistanceNodes`, latches `_cornerModeLastNode` at the car's current node and sets `_cornerModeApexTargetNodes = ForwardNodeDistance(apexNode)`, with a negative point-to-point target clamped to 0; each tick `_cornerModeDistanceNodes` is advanced by `WrappedNodeDelta(_cornerModeLastNode, currentNode)`, and the apex counts as passed once that count reaches the target. It is counted in track nodes, not metres, and it is not `RaceOrderKey`, lap progress or any global odometer. The shared `HasPassedApex` is a different test and was rebuilt on its own lap-local odometer in 1953 — see **AB**. The public `RaceProgress` field is now `RaceOrderKey` (lap-aware leaderboard sort only). Every takeover and release is logged as a pair, forced past `LogLevel=Error`: `mode <car>: pre route … corner … car … apex at …` then `ENTER corner #n, drinking … held …`; and `post route … car … apex at …` then `EXIT to route (why)` — the `why` reads `verdict window closed` since 1971, or `car fell well under the corner` on the untimed 25 m/s deselect (1980), and the `APEX` line prints `car`, `held` and `route` m/s.

| # | What | How | Passes if | State |
|---|---|---|---|---|
| X1 | **a corner takes the car and keeps its state through the exit** (1926-1932; hold retimed 1945, split into authority + verdict window 1971, entry on the ramp's lift demand 1980 — see **AF**) | any braking corner at speed; `mode` lines in `ARS.log` | `pre` shows `corner` below `car` (the envelope demanding a lift) at the takeover, `ENTER` names that corner, the speed leaves the corner at its apex through the 1 s ramp, and `EXIT` lands when the verdict window closes, 2 s after the apex | ⬜ |
| X2 | **taking over only ever slows** (1926; the term in force is the ramp over its 1 s run, 1971; entry waits for the envelope, 1980) | a fast corner taken from above the corner's ask — on bare route and, for a second pass, during the previous corner's `corner-route` ramp | no speed rise at the `ENTER` tick — the takeover waits until the corner's braking envelope sits below the car's forward speed, so the number it hands over already reads under the car whatever `route` shows; `route` is not the trigger on bare route or on a live ramp | ⬜ |
| X3 | **the exit is not held into the straight** (1932; retimed 1945, ramped 1971) | the corner before the longest straight | the route term takes the speed back as a slope over the 1 s ramp past the apex (`corner-route` on the readout while it runs), not a flat hold and not a step — the car does not carry the corner speed down the straight | ⬜ |
| X4 | **two close corners hand over without a gap** (1926; entry gate on `_cornerSpeedAuthority` since 1971) | an S-bend or a chicane | a second `ENTER` while the first corner's verdict window is still open, or an `EXIT` followed immediately by `ENTER` on the next tick — never a tick on the pure route term in between, and the takeover cancels the first corner's ramp | ⬜ |

## R · 2026-11-22 (1886 → 1889) — all five verified in game, see **Answered**


## V · 2026-11-22 (1893) — the brake factor is pinned

| # | What | How | Passes if | State |
|---|---|---|---|---|
| V1 | **each corner seeds at 0.75** (1893, learned in 1944, seed moved to 0.75 in 2090) | the apex debug text before any brake-learn pass | every untouched corner shows `b 0.75`; changed corners show their learned factor | ⬜ |

## W · 2026-11-22 (1896, retuned 1966) — the route band averages its tightest nodes

The band no longer reports its single tightest node: it averages its tightest nodes (no cap) — five from 1896, two as of build 1966 (`RouteRadiusMeanCount`) — so one noisy radius cannot drive the number. Built deliberately to be tested against the prediction that a tight piece narrower than five nodes gets averaged with open road and is under-protected.

| # | What | How | Passes if | State |
|---|---|---|---|---|
| W1 | **a real tight feature is still safe** (1896) | a short kink or chicane whose tight piece is narrower than the band's tightest set, with open road in the same band | the car still makes the kink; running wide or understeering there confirms the average under-protects it | ⬜ |
| W2 | **the number stops jittering** (1896) | the route radius readout (`ShowRouteRadius`) through a corner | the radius reads steady node to node instead of flicking between the tight node and the open road | ⬜ |

## U · 2026-11-22 (1892 → 1895) — corner speed learning, live

**Superseded by build 1951, and the verdict was rewritten again in 1954 and 1956 — see AC.** These rows tested the 1892–1895 verdict: a clean pass raised the corner 2 m/s, a slide of 10° or more or a wheel off dropped 2, and traffic voided the change. U1 and U4 were signed off under that rule; all five are superseded and none is open. The current rule is in **AC**.

| # | What | How | Passes if | State |
|---|---|---|---|---|
| U1 | **a clean corner pays 2 m/s** (1892) | a corner taken cleanly, brakes released before the entry, no traffic nearby | a notice reads `corner #n · clean · +0.0 → +2.0 m/s`, and the apex debug line for that car shows `+2.0 m/s` | — superseded (1951) |
| U2 | **a slide or a wheel off takes 2 back** (1892) | a corner where the car reaches 10 degrees of slide or more, or puts a wheel off the road | `slide at 10 deg or more` or `left the road`, and the offset drops 2 m/s | — superseded (1951) |
| U3 | **traffic voids the verdict** (1892) | a corner taken while avoiding a car: a lane taken to pass, or the side-by-side push on the nose | `traffic · offset stays`; the corner's number does not move either way | — superseded (1951) |
| U4 | **the number is used next pass, and stops at 10** (1892) | the same corner on following laps, offset at +2 or more | the car carries visibly more speed through it, and repeated clean passes stop at +10 m/s (`= at the limit`) | — superseded (1951) |
| U5 | **the window ends when the car is back on the power** (1895) | a corner the car takes and then goes above half throttle out of | the verdict lands at the throttle pickup, not at the six-second cap; a corner that never lifts still runs the full window | — superseded (1951) |

**Left as-is at his word, 2026-11-22 (drive verdict on 1895, the old rule; superseded 1951):** it works — the number moves and the car uses it — but where the route curvature term is the smaller of the two the offset is superseded and nothing in the car changes. U2, U3 and U5 were never exercised. Parked means shipped and left alone: the switch stays on by default, nothing is disabled.


## T · 2026-11-22 (1891) — the corner speed switch, before anything learns

| # | What | How | Passes if | State |
|---|---|---|---|---|
| T1 | **the switch exists and holds** (1891) | Rivals menu, pace page: toggle Corner Speed Learning off, restart, reopen | the checkbox reads off after the restart; the learner is live now (see AC), so on/off racing is no longer identical | ⬜ |

## S · 2026-11-22 (1890) — the boost waits until the corner is behind

| # | What | How | Passes if | State |
|---|---|---|---|---|
| S1 | **a corner exit is not open road** (1890) | a rival unwinding out of a corner onto a long straight, next entrance far away | no burn until about a second after the corner is behind it, then the burn can fire as before | ⬜ |
| S2 | **the straights still burn** (1890) | a race with several long straights, a fight and a finish | burns still land on the straights, in fights and near the finish: the fix narrows the window, it does not silence the bottle | ⬜ |

## Q · 2026-11-16 (1878 → 1885) — the blend is gone, the aim leads further, every verdict speaks

| # | What | How | Passes if | State |
|---|---|---|---|---|
| Q1 | **the widest-arc blend is gone** (1878) | drive the wide fast sweepers that used to launch you too hot into the apex | cars hold the speed the line asks for; no corner feels out-cornered by its own radius | ⬜ |
| Q2 | **brake aim leads 1.25 s** (1880) | the same entry you used to brake at | braking starts visibly earlier (~1.79× the old lead distance); the car reaches corner speed with more room to spare | — obsolete (1936) |
| Q3 | **no verdict is rate-limited away** (1882) | `Brake Learn Notify` on, drive a chicane or any two corners < 1.2 s apart | **both** verdicts reach the UI — before the fix the second one was silently dropped | — obsolete (1936) |
| Q4 | **an aborted corner still closes** (1882) | engage a corner (within 50 m of the apex), then spin/stop before the apex so the queue moves on | `corner #n · apex not reached · factor stays x.xx` appears once — no silence, and no second message if the same corner is re-held and passed | — obsolete (1936) |
| Q5 | **a pending verdict survives learning off** (1882) | brake-learn notify on, cross an apex, and untick Brake Learning within 0.75 s | the verdict still fires (saying `learning off · factor stays`), instead of vanishing with the toggle | — obsolete (1936) |
| Q6 | **overshoot deducts** (1884) | a corner where a rival crosses the brakepoint clearly hot (> 3 m/s over target) | one message `overshot by X m/s`, factor **−0.02** exactly (0.75 → 0.73), clamped at 0.25 | — obsolete (1936) |
| Q7 | **undershoot adds** (1884) | a rival that was above target+3 on the approach, then drops below it while still 7+ nodes (≈7 m) before the brakepoint | one message `undershoot, below target+3 with N nodes spare`, factor **+0.02** exactly, clamped at 1.25 | — obsolete (1936) |
| Q8 | **the band holds and still speaks** (1884) | a corner whose crossing is within ±3 m/s of target, or one approached from below without ever being above the bar | message `in the band · 0.75 = holds` — no change, never silent; and never two verdicts for one corner | — obsolete (1936) |

Note (1882): the old brake learner ran only for **AI racers** — its update lived inside `if (!ControlledByPlayer)`.
The 1936 ramp analyser retains that same player boundary: it is driven inside the AI path.
Known limit (1884): superseded in 1936 — the old undershoot latch no longer exists.

---

## P · 2026-11-14 (1841 → 1872) — all eighteen verified in game, see **Answered**


## O · 2026-11-14 (1808 → 1820) — the finish line and the quick join

| # | What | How | Passes if | State |
|---|---|---|---|---|
| O1 | **a failed start heals itself** (1808, 1809) | get a start to fail (empty grid), then press Start again without ending anything | the log says the grid was flagged instanced with no racers and rebuilds, instead of repeating the roster message | ⬜ |
| O2 | **the save is free after a race** (1811) | run a race, end it (or reload mid-race), then open the pause menu | Save Game is available; the mod no longer marks itself a mission script | ⬜ |
| O5 | **nothing race-shaped outlives the race** (1820) | every debug toggle on, finish a race | checkpoints, chevrons, bumps, G-line, rival cards, route radius and track analysis all stop when the race does | ⬜ |

---

## N · 2026-11-13 (1784 → 1802) — Nearby Cars

Ten builds since the section below was written; every row here is **compile-only**.

| # | What | How | Passes if | State |
|---|---|---|---|---|
| N2 | **adoption runs after the pool, and a ghost seat gets a driver** (1790) | kill a parked car's ambient driver with a clean headshot — the corpse counts as occupied until the engine culls the body, then the seat is a ghost; Start Race once the body is gone | it is adopted, a driver is spawned and ends up **in the seat** (`Racer.cs:584`), the car drives, and the log's `ghostSeat` count rises | ⬜ unprovoked: no invalid ped yet. ARS never deletes a preview car's ped itself, so the only producer is the engine culling a corpse; walk-away (car despawns too) is the fallback |

---

## L · the racing line (1301 → 1312) — all three verified, see **Answered**

## A · Without driving (log)

| # | What | How | Passes if | State |
|---|---|---|---|---|
| A2 | **atomic settings** | change one row, then make the ini **read-only** and change another | the value persists; with the file locked an `Error` lands in the log instead of the value being lost | ⬜ |

## B · One normal lap

| # | What | How | Passes if | State |
|---|---|---|---|---|
| B2 | **brake learner** (1126) | an entry overshot under braking with `Brake Learn Notify` | the notice **comes out without Show Track Analysis** and the cut is **0.15 at most** | — obsolete (1936) |
| B3 | **walls at three** | Show Rivalry, a rival on each side | both walls stay drawn | ⬜ |
| B6 | **board · position** (1125) | look at the top-left corner | `POS` no longer overlaps the hints/tips | ⬜ |
| B7 | **slow allowance** (1127) | a 5-15 mph corner with `Show Front Slip` | the car turns and the token `x1.5`/`x2.0` appears | ⬜ |
| B8 | **Peak Slip Target** (1128) | token `x0.9` and `F` in a fast corner | the cut starts before the peak (F 100 = 90 % of the peak) | ⬜ |
| B9 | **Deepest Cut (%)** (1385) | raise it from 60 to 80 in a long slide | it cuts more than before; at 0 it cuts nothing; at 60 identical to 1384 | ⬜ |
| B10 | **the reference is the outer wheel** (1391, alive in `MeasureSlipFromMemory`) | `Show Front Slip` with `Slip From Memory` **on**: steer one way in a fast corner and watch the token's `F` | the number follows the **outer** wheel (the one slipping less); with the wheel straight it is the average of the two | ⬜ |
| B13 | **Brake Aim Depth** (1397) | a corner where it already braked, raising it to 0.4 and then 0.8 | it brakes **later** and enters faster; if after a few laps it brakes where it used to, that is the learner lowering the factor | — obsolete (1936) |
| B17 | **exponential close** (1132) | `Slip From Memory` **off**: the readout is the proportion | at 500 % it reaches 100 % in ~0.27 s (8 ticks); at 1000 % ~0.3 s | ⬜ |

## C · Deliberate abuse

| # | What | How | Passes if | State |
|---|---|---|---|---|
| C1 | **nitro at the finish** | fire it and **End Race within 3 s** | your car does not come out with the bottle armed; unticking Nitrous mid-burn **stops** the push | ⬜ |
| C2 | **reload** | **Insert** mid-race | no crash, the race disappears, neutral pedal | ⬜ |
| C3 | **race after race** | finish one and launch another before the «Ns to end» expires | the new one **does not die on its own lights** | ⬜ |
| C4 | **creator** | start line + **one** section, save | it refuses and says how many nodes are missing, with no black screen | ⬜ |
| C6 | **freecam** | enable it and **get into a car** | the freecam **drops out by itself** | ⬜ |

## D · Decisions and paperwork — both decided, see **Answered**

## E · that day's work, not yet tested

| # | What | How | Passes if | State |
|---|---|---|---|---|
| E2 | **horizontal slip angle** (1165) | corners with **elevation change** | it detects the slide the same on a slope as on the flat; before, the gradient inflated it | ⬜ |
| E3 | **chicanes** (1165) | a track with gradient and esses | chicanes are still chicanes (+10 mph); none gets classified by the elevation | ⬜ |
| E4 | **grid slots** (1165) | look at the cars on the start | none sunk or floating on tracks with gradient | ⬜ |

## F · The feedback batch (1294)

| # | What | How | Passes if | State |
|---|---|---|---|---|
| F5 | **10 min parked, no storm** (came from A1) | leave the mod standing 10 min, no race, and `grep -c "OnTick error" ARS.log` | **0**. The «208» figure came from a session on the 25th; every session since closes at 0 | ⬜ |

---

### Off this list, already decided

- **rival-log lag** (the old B5, no longer a row) and **R12** (leaderboard per frame) → parked for **0.11**, in `AGENTS-BACKLOG.md`.
- **B5 · board · toggle (1112)** → covered by **F4** (verified 28-10 · 1294); row withdrawn 13-11.
- **`SpeedSteerCeiling`** → zombie for **0.11**: revive the law or delete it with its menu row.
- **`corner-chevron-exit`** → 0.9.1.
- **301** · 43 real legacy rows → debt, not a blocker. **303-F2** → left alone on purpose.
- **The crash when changing the roster**: cause **unknown**. The 1167/1169 commits remove real hazards but do not explain it. If it comes back, `ARS.prev.log` (in the game root) already keeps the previous session.

---

## H · the 30-10 bug hunt (1375 → 1380)

Six commits, all **compile-only**. Every row carries how to provoke it and what must happen.

| # | What | How | Passes if | State |
|---|---|---|---|---|
| H9 | **the lane on a gradient** (1379) | a stretch with elevation change, at speed | it behaves **the same** as on the flat; the change is < 1 cm and you should not feel it | ⬜ outstanding: has not found the stretch yet |

### Waiting on your decision (deliberately untouched)

1. **`No Collision` is inverted** at its call sites: `ApplyNoCollision(enable)` hands `!enable` to `SET_ENTITY_NO_COLLISION_ENTITY` (`AutosportRacingSystem.cs:1906`), so the menu row asks for the opposite of what it says, and the cleanup asks for the opposite again. It also pairs each car with its **nearest rival only**, so half the field keeps colliding anyway. D-T4-1.
2. **«Open Up on Throttle»: withdrawn from the tree** (1389) — the row, the key and the method are gone; it never reached the wheel any more. The formula lives in the backlog (`open-up-inert`) in case it comes back. D-T6-1.
3. **`avoid-hold-timeout`** (the test I owed you) lives behind `avoidHoldEnabled = false` (`Racer.cs:1209`): the test points at dead code. D-T5-2.
4. **`FEATURES.md` sells the player's nitro as «coming soon»** (`docs/FEATURES.md:139`) and the AI's as a percentage of yours: yours runs on X/duck, and the AI carries 5× against 1×. D-T8-1, D-T8-2.

Two items left this list on 13-11 because the code moved under them: `LaneMoverSpeedCapMps` no longer exists anywhere in the tree, and the ABS tooltip was rewritten in 1803.

## I · 2026-11-02 (1569)

| # | What | How | Passes if | State |
|---|---|---|---|---|
| I1 | **Projection brake floor** (`ProjectionBrakeFloorMph`) | go wide in a fast bend with `Show Gs Projection` on and read the `Offshoot:` log line | `floor=` moves with the bend — 20 in a tight corner, higher in a fast one — and the car **keeps rolling**: the projection never holds a full brake with the speed under its own floor | ⬜ |
| I3 | **lane timings: identity, two clocks, and no glide** | a lap | the absolute ends when the lane preview crosses the apex and the request **snaps** to the curvature ask (the 1 s and 3 s glides are gone); the next corner's preparation does not claim the car on the apex | ⬜ |
| I5 | **drive kept off track** (`OffTrackThrottleLevel` 0.35, now a **floor**) | put a car well past the edge at 40+ mph and let it go | off the track the pedal cannot be cut below the ramp by TCS/stability/plan: the car keeps rolling and only arms the reverse recovery if it is genuinely stopped | ⬜ |
| I6 | **projection is a speed offset** (`ProjectionSlowdownMphPerSecond` 2, floor 30) | run wide on the approach to a fast bend, `Show Gs Projection` on | the car slows **gradually** (2 mph/s on the target, then the pedal law brakes) instead of the old 0.25 s throttle drop + 0.8 brake, and it never goes under 30 mph or the bend's speed less 30; the `Offshoot:` log line now prints `drop=` and `floor=` | ⬜ |
| I7 | **route turn-in is a pure inset** (`LaneInsetPerRadius` 125, no speed factor) | a slow corner taken fast, and the same corner taken slowly | the aim sits the **same distance inside** in both (2.5 m at R=50, 1.25 at R=100, 5 m clamp under ~25 m); it no longer widens with speed | ⬜ |
| I8 | **a corner is forgotten at the lookahead** (`ApexAheadOfLookahead` gates both selection and forgetting) | drive out of a corner on a stretch with no corner the lane will work | the lane never asks for the **outside** after the exit and never re-arms on the corner behind it; it holds the field until the apex crosses the preview, then lets go | ⬜ |

## J · 2026-11-09 (1755) — the corner pipeline

The ten bench flags are **compile-time constants**, all `true` in this build (`Racer.cs:16-25`, `TrackSpeed`) —
there is no switch and no ini key, so nothing here can be restored or forgotten. Ini as this bench was written:
`CornerCeiling` 500, `CornerOffset` 0, `RouteOffset` 0, `BrakeAimDepth` 0. The blend setting (`CornerRadiusBlendPct`) was removed from the build — the widest-arc speed model lost to the detected radius everywhere it was driven.

| # | What | How | Passes if | State |
|---|---|---|---|---|
| J3 | **full brake asks for a 4.5 m/s error** (fe71f53) | overspeed a bend by ~5 mph, then ~10 | the pedal stays off full while the error is under 4.5 m/s (≈10 mph); the learner band, the projection share and the ramp shift with it | ⬜ |

Open on this bench: the exit asks for nothing outward — the dynamic lane only ever aims inside, so the car
leaves a corner on the inside line unless `DefendLane` is working it (his ruling: the exit is fluid, default
outward). A span holding two corners of different size is refused at the 1.6 radius ratio, not split. Grapeseed
2968 sits at 1.97.

## K · 2026-11-09 (1768) — the corner is spent when the car coasts

| # | What | How | Passes if | State |
|---|---|---|---|---|
| K2 | **a lift on the straight releases nothing** | coast a hundred metres before a bend, both pedals up | the corner stays selected and the shape still starts at the entrance four seconds out — the release is gated on being past the entrance | ⬜ |
| K3 | **the input gate is gone** | trail brake lightly into a bend | the pedals no longer matter: only the longitudinal Gs decide, so a light brake above 0.05 g keeps the corner selected | ⬜ |
| K4 | **the lane lets go on its own corner** | the shaped line through a bend taken off the pedals | the shape stops once the car is through the entrance instead of running to the apex: the lane tests the corner it holds, because the queue's head has already moved on by the time steering runs | ⬜ |
| K5 | **the creep is a clock, not an angle** (1770) | the S where a left corner is approached through a right-hander | the cap opens at 0.1 m and reaches 2 m three seconds after the corner starts being asked, so a counter-turning prep no longer gets the tightest creep for being perfectly placed | ⬜ |
| K6 | **the line outlives the coast** (1836) | coast through an entrance: the speed ask stays spent, but the line is held to its apex node | the shape does not stop at the entrance any more — coasting spends the speed ask only, and the corner is kept to its apex (`Racer.cs:729`). This is what the K2 above was re-scoped to in 1836 | ⬜ |

The signal is `VehicleData.GetLongitudinalGs`, the same one `UpdateCornerRequirements` already reads.
`CoastSpentGs` is the only knob (0.05 g ≈ 0.49 m/s², so the tail of a soft brake release counts as spent).

## M · 2026-11-09 (1776) — what the two auditors found, and what shipped

DS Flash and GLM 5.3 Flash audited `fc8dd13..HEAD` read-only (`docs/audit/500-session-ds.md`,
`docs/audit/501-session-glm.md`). The rows below are the ones I re-checked in the code and fixed; the rest,
including the ones I judged false or not worth a change, are in the reports.

| # | What | Passes if | State |
|---|---|---|---|
| M1 | **the lane really lets go** (1772) | brake into a bend, coast from the entrance: the shape stops there instead of running to the apex — the selection no longer re-adopts a corner it has just declared spent | ⬜ |
| M2 | **a stopped car is not coasting** (1773) | sitting on the grid, or stopped past an entrance, no corner is dropped from the plan | ⬜ |
| M3 | **the creep survives a race** (1774) | second race on the same track: the cap still opens at 0.1 m and climbs, instead of starting at 2 m | ⬜ |
| M4 | **a skipped shape has no entrance** (1775) | a corner the shape builder dropped no longer reports the start line as its entrance | ⬜ |
| M5 | **the lane looks past a skipped corner** (1777) | a wide corner the plan ignored gets no lean from the lane either — one selection, both readers | ⬜ |

Left alone on purpose, each one a verdict rather than a fix: the refill's `first = 0` fallback still commits a corner
the plan skipped — a queue needs a head, and `CornerRequiresBraking` keeps that head out of the braking table, so it is
a placeholder and not a decision to brake. Build 1962 uses that head in the empty-table fallback as a speed ceiling, so a
skipped wide head cannot hide the next tight corner; its ramp is high enough not to bind. The lane can adopt the next corner while the plan still brakes for the
current apex: that is what an S asks for, and the ramp is what keeps the first ask small. `UpdateCornerRequirements`
runs every frame because it is the detector, not the refill. `_cornerContexts` does prune on a revision change
(`SyncCornerContexts`), so that audit row was false. AGENTS/ now carries dated CURRENT blocks; its history stays as
history.

---

## Answered — every row already driven or disposed of

Kept for provenance: these are the in-game verdicts and the dispositions. Nothing here is work.

**AK · 2026-11-23 (2010) — airborne, the wheel is worth a quarter of the slide** — ✅ verified in game by the owner,
23-11: «I see the cars not insta-snapping». With all four wheels off the ground both steer limits are trimmed to
`|SlideAngle| × 0.25` (`AirborneSteerShareOfSlide`) before the one clamp, so a car landing at 40° arrives with 10° of
wheel instead of a full countersteer. AI only: `ApplySteerLimits` runs in the AI branch and the mod never writes a
player's steering (`ApplyInputs` is AI-only).

**I · 2026-11-02 (1569)**
- **I2** `ars` cheat — verified 16-11: the console opens the menu, and the code registers only `ars` (`AutosportRacingSystem.cs:390,2388`) — `arsmenu` is gone
- **I4** apex merge raised 5 s → 6 s — superseded — the old 5→6 s merge window is gone; the live law (Nov-09 corner work) merges on `closeCornerSeconds = 2 s`, or on similar radii within `SpanMergeRadiusGate = 50 m` with the same aim (`TrackLoader.cs:1295-1298`). Compile-verified, not driven

**C · Deliberate abuse**
- **C5** short solo — verified 16-11: Grid Size 0 on a short circuit starts

**O · the finish line and the quick join (16-11)**
- **O3** the chevron is a quick race (1814) — verified 16-11
- **O4** corner blips live and die with the race (1817-1819) — verified 16-11
- **O6** the corner radius blend reads right (1805) — **superseded: the blend is gone from the tree.** The row verified a setting that no longer exists; the corner speed radius is the detected radius. Kept for the record; nothing to test anymore.
- **O7** the declaration sweep changed nothing but where things sit (1814-1834) — verified 16-11: the static proof (identical line multiset in `Racer.cs` and `DataStructures.cs`, six enums re-wrapped, the menu rows in the same order (41 at the time; the blend row has since been removed)) plus a 12-minute race on 1876 — **0** `OnTick error`, 2,989 lines all `(Info)`, no Error or Fatal line in the log

**P · the debug panel (14-11 → 16-11)**
- **P1** six throttle bars in the panel (1843) — verified 14-11 · 1843
- **P2** the bars split: three throttle, three brake (1844) — verified 14-11 · 1844
- **P3** the bars tell the throttle chain (1845) — verified 14-11 · 1845
- **P4** the TCS cap pulls from the top (1846) — verified 16-11
- **P5** the throttle bar rides a smooth follower (1847) — verified 14-11 · 1847
- **P6** brake and the TCS ceiling ride followers too (1848) — verified 14-11 · 1848
- **P7** corner and route margins as centred white bars (1849) — verified 14-11 · 1849 — bars move as designed; the drive caught the corner bar riding the braking *ramp*, not the arrival speed → P8
- **P8** the corner margin measures the arrival speed (1850) — verified 14-11 · 1850
- **P9** the panel rides the Show Pedal Trail toggle (1851) — verified 14-11 · 1851
- **P10** the columns are labelled (1852) — verified 14-11 · 1852
- **P11** APEX and ROUT: orange in range, gray when irrelevant (1853) — verified 14-11 · 1853
- **P12** the Gs floor is a lane lerp (1854) — verified 14-11 · 1854
- **P13** STB: the stability throttle cap (1855) — verified 16-11
- **P14** the floor gate rides the slide angle (1856) — verified 14-11 · 1856
- **P15** the floor: a ±2 m window over the car lane, closed off-road (1857) — verified 14-11 · 1857
- **P16** the TCS ladders: 20-400, step 20 (1859) — verified 14-11 · 1859
- **P17** the TCS ladders reach the game's ceiling (1860) — verified 16-11
- **P18** Show TRlateral drawn (1872) — verified 16-11

**N · Nearby Cars (13-11 · 1802)**
- **N1** the cars around you join the grid (1784) — verified
- **N3** the toggle is a three-way choice (1794) — verified: «Works, they tune if the setting is chosen»
- **N4** adopted cars are sorted, not pinned (1799) — verified
- **N5** the chevrons share one angle (1800, 1802) — verified
- **N6** the chevron says which car is adoptable (1801) — verified

**L · the racing line**
- **L1** phase by distance (1302) — verified 28-10
- **L2** lean by width and radius (1308) — verified 28-10
- **L3** out-in removed (1312) — verified 28-10

**A · Without driving**
- **A3** 304 · empty grid — verified 26-10
- **A4** preflight — verified 26-10 (1124)
- **A5** layout · names — verified 26-10

- **A6** new roster (1169) — verified 16-11 — the roster indexes and the grid builds (the row's 643 was the old roster; the current one holds 568)
- **A7** rotated log (1171) — verified 16-11: `ARS.prev.log` sits in the game root with the previous session
**B · One normal lap**
- **B1** corner seam — verified 26-10: 14 corners at El Burro / 18 at Oct1, `apex=0` only in the `skipped`
- **B4** board · names (1117) — verified 26-10
- **B11** Target Under Power (1393) — ⛔ parked in 1396, the row no longer exists
- **B12** the lane's throttle floor (1396) — verified 31-10: «tested, works great»
- **B14** Car-Centred Radius (1399) — verified 31-10 · 1400: «we have it, this has helped»; that the entry has not got slower was never called out
- **B15** Slid Radius Lead + the radius toggle (1404) — verified 31-10
- **B16** an inactive lane plants itself (1408) — ❌ dropped in 1409: `LaneCrossMinMeters` turned a continuous quantity into a binary state and flickered

**Pace · the surface under the road**
- **the route term reads the surface** (1806) — it was the only speed answer blind to grip: the corner term had always carried its own. Verified 13-11: «the entire track is driven better now … a flat improvement everywhere»
- **an unsampled surface inherits the one the car is on** (1807) — at runtime it takes the grip under the car, and at load the previous sample; a dead ray no longer writes tarmac

**D · Decisions and paperwork**
- **D1** ABS unlocked (1114) — the description was replaced in 1803 (`AutosportRacingSystem.cs:1135`); the checkbox itself was never driven, so it stays compile-only
- **D2** `Oprahs` in `names.txt` — renamed to `Oprah`, live install and Dist mirror both
- **D3** tag `v0.9.0` — ✅ 30-10: release published 27-10, build 1293, tag at `1d8b91e`
- **D4** PI relative, on foot (1124) — verified 26-10

**E · that day's work**
- **E1** off-track: 40 under, floor 20 (1171) — ⛔ superseded by I1 (1569)
- **E5** the wheel dither — resolved in 1163: it was the delay measured in 3D

- **E6** air offroad — verified 16-11: the signal works — stability reduces mid-air; the feature itself stays parked, never built
**F · The feedback batch (1294)**
- **F1** build gate — ✅ 28-10: the race ran, the gate let it through
- **F2** no storm — ✅ 28-10: 0 across 14 log lines (1.5 KB)
- **F3** tips vs board — ✅ «all good», 28-10
- **F4** car name — ✅ «all good», 28-10 — this is what retired B5

**H · the 30-10 bug hunt**
- **H1** track refused (1375) — verified 30-10 · 1380 with `Chianski Long only Five.xml`; the door exercised was the preflight, the menu path is still unreachable
- **H2** the abort leaves the ped standing (1375) — verified 30-10 · 1380
- **H3** a failing background load speaks up (1375) — verified 30-10 · 1380: `OnTick error` = 0
- **H4** the prune keeps comments (1376) — ❌ fails 30-10 · 1380: `MenuSettings.Save()` rewrites the file, so the comment goes. D-T2-3
- **H5** the prune says what it drops (1376) — verified 30-10 · 1380: `dropped stale ClaveInventada`
- **H6** restart with no inherited card (1377) — verified 30-10 · 1380: «the plays disappear and are played in the new race normally»
- **H7** Restart after Load Track (1378) — ⛔ unreachable: `StagedSpawns` is forced to `false` (`:3432`), so there is no door today
- **H8** the handbrake at launch (1378) — verified 30-10 · 1380: «everyone gets away fine»
- **H10** clamp after a slide + Restart (1380) — verified 30-10 · 1380: «I see no problem»

**J · the corner pipeline**
- **J1** the prep cap hangs off where the prep started (47e9efd) — verified 09-11 · 1755
- **J2** release at the apex, four seconds to position (1e8e57c) — 09-11 · 1755, same pass as J1

**R · nitrous straights (22-11 · 1889)** — «all works»
- **R1** nitrous needs a near-straight (1886) — verified 22-11 · 1889
- **R2** 2nd gear is enough now (1886) — verified 22-11 · 1889
- **R3** the faster-rival gate is a 60 m window (1888) — verified 22-11 · 1889
- **R4** the 8 s gate is honest now (1889) — verified 22-11 · 1889: «the 8 s gate seems to work well»
- **R5** slide under 1° (1889) — verified 22-11 · 1889

**K · the corner is spent when the car coasts — for speed only**
- **K1** coasting inside the corner releases it early (`CornerSpent`) — verified 09-11 · 1769/1770: «the 0.05gs thing works well»

**AA · corner speed learning, the peak slip share (23-11)**
- **AA1** the verdict is the peak slip share (1951) — superseded (1954): the share verdict is gone, never driven; see **AC**
- **AA2** the two thresholds move it, the band holds (1951) — superseded (1954): there is no ±3 m/s step any more
- **AA3** traffic and a wheel off no longer void it (1951) — superseded (1954): no traffic or off-track veto exists, but the row was never driven and its subject is now AC1-AC5
- **AA4** one verdict per pass (1951) — superseded (1954): the once-per-pass rule survives as AC5, still undriven
- **AA5** the line says why it stayed (1952) — superseded (1954): `stays at the limit` survives in AC4

