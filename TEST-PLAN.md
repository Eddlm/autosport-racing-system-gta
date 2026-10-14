# ARS — test plan: build 483

**Status: driven, except the batch at the very end of the file.** Everything this file listed as outstanding has since been driven and held in one session. The recovery's reverse phase and its teleport, gated on the escape budget, were driver-verified on build 450 (`666c89b`), and the rest of the recovery went with them — trigger B, the exit back into normal racing, the start-line arm delay and the flooded-engine rescue — after which the drive phase was rebuilt again (coast instead of brake, nearest-edge aim, exit on any part of the car on track; `b50dfef`, section at the end) and that rebuild has been driven too. The same drive covered the maneuver cards on the fast core pass with a clean restart (`601313c`, last section), the steer ceiling down to one governor, the slide blend after the slew, the crest entrance move, DNF parking in both options with a race ending on DNFs, the route-frame rival detection, and the bump scan overlay, whose cyan marker was confirmed in game. The limiter's slide raise then went off, was driven in that state, and its retirement was kept, so that decision is settled; the changes after it — the narrowed ramp band, the damper's speed setting and the ceiling factor (`489`, last section) — are **undriven**, and the ladder's `steer-ramp-band` is that drive. What the drive did not settle: the crest's gating constants still wait on an instrumented run, and the two residuals the ladder names for the route-frame detection were not provoked. No Collision's one-shot mode and its per-tick all-pairs form were both
driven and **failed** against the engine's one-slot limit, and its nearest-rival form is now **driver-verified**
(`cbae845`); the slew is driven — 45 read as less stable and **180 removed most of the stability problems**
(`393bfdd`, driver-verified); the damper's speed scale is capped at 1 and **driver-verified as an improvement**
(`5bb94bd`), and its two subtraction rules are **driver-verified as working very well** (`a0134d9`); the damper bypass and the maneuvering ramp are new; **build 473 collapsed the steer governor
to the slide**, so a car at turn-in now gets the full cornering law where
the retired yaw cut allowed it about a fifth of that — driven with the rest of the list, so that authority is in. Build 471's state was accepted on the drive — the simplification
batch and the governor-key fix regressed nothing. The deployed DLL is dev build **489**; a build + reload is enough
(SHVDN reloads the scripts live, no game restart).

| commit | what it is | section |
|---|---|---|
| `0623276` | recovery redesign, DNF parking, No Collision one-shot, route-frame rival detection | §1–§13 |
| `27ee097` | the corner entrance moves to the crest's entry | Crest entrance move |
| `666c89b` | the recovery teleport now waits for the escape budget, so the reverse runs | §5 |
| `f84fe9a` | the steering slew is one rate for both directions | Steering slew and slide blend |
| `d521d1d` | the no-collision pairs are re-asserted every tick (one-shot failed) | §12 |
| `cbae845` | each car is ghosted against its nearest rival only (engine has one slot) | §12 |
| `f3f318f` → `148db70` | the bump/lip scan and its `Show Bumps` overlay, then the rise-walk fixes | Bump scan overlay |

Start a race with several AI cars. `Show Pedal Trail` on the Debug menu helps for the input trail, and `Log.log` in the
game folder carries the init banner and lap lines.

---

## Superseded during the session — do NOT test these

- The **500 → 1000 ms reverse**: still live, but now part of the redesign below, so test it there.
- The **join gate / wait-for-traffic**, its **5 mph creep**, the **edge→+4 m band** and the **contact stop**:
  all deleted when the redesign landed. Nothing to test.
- The old **`HasRegainedControl`** verdict (20 mph / heading / slide / 500 ms): deleted.
- The **`RealisticRecovery` menu option**: deleted; its stale ini key is pruned on next load.

---

## 1. Off-track throttle ramp (general, every car)

1. Take a car well off the track at speed (past the raw edge).
   **Expect:** pedal lifted (throttle 0) above 20 mph, ramping back to full at 15 mph — it sits slow but firm.
2. Watch the **white override sphere** on the white steer line while off track (Show Track Analysis).
3. Put the car half-off (centre still on the track). **Expect: no cap** — the rule is centre-past-edge.

## 2. Recovery trigger A — stuck against something

**Driver-verified on build 450:** the 1 s reverse backs off as designed instead of being teleported mid-phase.

4. Wedge a car against a tree/wall and let it stop.
   **Expect:** after 2 s below 2 mph with the plan asking for more → **Reverse** (1 s, straight, backwards).
5. Then **Drive**: forward at ≤20 mph, steering back toward the route.
6. **Is 1 s of reverse enough to clear a tree?** It is ~4× the distance of 500 ms. Watch it does not back into the car behind.

## 3. Recovery trigger B — off track (driver-verified)

7. Send a car off the track and keep it there 2 s.
   **Expect:** it enters **Drive directly** (no reverse), slows to ~20 mph, steers back and rejoins.
8. Confirm a car off track but **moving** does not enter Reverse.

## 4. Recovery exit (driver-verified)

9. A car rejoining must end recovery only when **on the drivable bound AND signed forward > 4 mph**, held 0.5 s.
   **Expect:** a car still rolling backwards out of the reverse does NOT exit.
10. Confirm a car that crawls back onto the surface at ~3 mph stays in recovery, then exits as it speeds up.

## 5. Re-stuck loop + snap

**Driver-verified on build 450:** the teleport fires, but only after the escape budget rather than one second in.

11. In Drive, stop the car again (or wedge it).
    **Expect:** another 1 s Reverse (same A predicate, 2 s).
12. A car that **cannot move at all** (no position change over 1 s) is **teleported** to the drivable bound — but only
    once the 6 s recovery budget is spent. Before that the 1 s Reverse and the Drive phase run uninterrupted; the
    snap no longer fires on the first static second.
13. A car re-stuck in **Reverse** after the 6 s recovery budget → teleported.
14. A car that keeps **moving** is never teleported at the budget (by design).

## 6. The 15 s arm delay (start line) — no false entry seen after a launch

15. At the green, a heavy / wheelspinning car still under 2 mph must **NOT** reverse — no recovery at all for the
    first 15 s of the race.
16. A car pushed off track inside those first 15 s must **NOT** enter Drive either (it still gets the off-track
    throttle cap and the normal lane law).
17. After 15 s, recovery arms normally.

## 7. DNF parking — option OFF (shoulder)

**Driver-observed:** falling in the water did **not** DNF, and the car stayed immovable even after the teleport.
Water does not zero `EngineHealth` — it floods the engine, and nothing in ARS ever restarted one — so the
`EngineHealth <= 0` predicate never saw it. The fix is a restart, not a DNF: an AI car whose engine is off with
health left is started again (`EnsureEngineRunning`, `Racer.cs`), so the recovery's own teleport now rescues it.
Watch a flooded car come back instead of sitting on the throttle. **Driver-verified: it does.**

18. Kill an engine (water). **Expect:** the car is parked on the shoulder, slot 0 at the start/finish line, the next
    5 m further along, alternating sides, 1 m past the edge, heading along the route, handbrake held.
19. Confirm it never arms recovery again and does not move.
20. **Check the slot is not inside a wall / over water** — there is no ground test (known caveat).
21. Confirm it is still seen as a rival: other AI should avoid it rather than drive through it.

## 8. DNF parking — option ON (under track, default)

22. Toggle **"Hide DNFs"** in the AI menu (may sit below the 10-row fold — scroll).
23. Kill an engine. **Expect:** the car is dropped 5 m below the slot and **frozen in the air**, and the AI ignores
    it entirely (no swerve, no avoidance, no rival walls).
24. Confirm the freeze does **not** carry into the next race (cleared in `Initialize`).
25. Confirm the choice persists across a reload (`Menu-Settings.ini` → `DNFUnderTrack`).

## 9. Race end with DNFs

26. With 1+ DNF cars, the race must end when all **surviving** cars have finished, not wait on the parked car.
27. Extreme: if **every** car ends up parked, the race must still end (all-parked guard) — and no crash on the
    reward line.
28. The reward still goes to the player only if a real finisher won.

## 10. Slow / stopped rival direction

29. A stopped or barely-rolling rival ahead (under 1 m/s) must still be treated as pointing the same way — the
    detector should avoid it, not drive into it.
30. A shoulder-parked DNF car: the AI should avoid it correctly.

## 11. Regressions to glance at

31. AI menu: the new **Hide DNFs** item appears, toggles live, and does not disturb the items after it.
32. Fresh install: `Settings\` still creates only the three `Menu-*.ini`, now including `DNFUnderTrack`.
33. The retired `RealisticRecovery` key is gone from `Menu-Settings.ini` after a load.
34. Pedal bar still shows the reason spheres correctly alongside the new off-track one.

## 12. No Collision option — test this one hardest

**Known history, and the limit the engine imposes.** The third argument is a **mode** (verified in the engine,
`commands_entity.cpp:5839`): `false` is `NO_COLLISION_PERMENANT`, `true` is `NO_COLLISION_RESET_WHEN_NO_IMPACTS`,
which lets the entry clear itself. **The one-shot application was driven and failed, and so was the per-tick
all-pairs re-assert — because the engine stores exactly one partner per entity** (`fwDynamicEntityComponent::
m_pNoCollisionEntity`, a single `fwRegdRef`), and `PERMENANT` is never cleared by the reset path. So an all-pairs
loop leaves each car holding only its last loop partner. **A whole grid cannot be ghosted pairwise with this
native.** A car is instead paired with its **nearest rival**, one slot each, re-asserted every tick.

**Driver-verified (`cbae845`):** the nearest-rival pairing works, and it ghosts more than one pair at once. Three
abreast, the **middle car passes through both neighbours** — each neighbour stores it as its nearest, so both
pairs are rejected even though the middle car itself has one slot. The constraint is one **outgoing** slot per
car, not one ghosted pair per car.

The residual: a pair is ghosted only when at least one of the two stored the other, so a car whose sole close
rival points elsewhere can still be touched. Judge the option on whether ordinary racing stays contact-free, not
on the full matrix below being literally achievable.

The matrix below is a test of how far the nearest-rival pairing holds; a single contact is still the falsification.

35. Toggle **No Collision** in the AI menu.
36. With it ON: racers must **not detect each other** — no avoidance, no repulsion, no rival walls, no
    Yield / DiveBomb / ChillOut, no contested-Nitro, no rival throttle cap. The pack should behave as if each
    car were alone.
37. With it ON: cars must **physically pass through each other** (no contact) while still colliding with the
    **world** — check a car still hits a wall and does not fall through the road.
38. Toggling it OFF mid-race must restore collisions immediately.
39. It must persist across a reload (`NoCollision` in `Menu-Settings.ini`).
40. After the race the player's car must NOT still be phasing through leftovers — `CleanEverything` clears the
    pairs.

### Contact matrix — every case that can produce a bump

Drive each of these with the mode ON and watch for any contact. These are the historical failure cases:

- [ ] AI vs AI, side by side through a corner.
- [ ] AI rear-ends a slower AI.
- [ ] Head-on: a spun car facing the wrong way.
- [ ] Player drives into an AI.
- [ ] AI drives into the player.
- [ ] Start-line pile-up: overlapping cars at the green.
- [ ] **Respawn — the most likely leak**: a destroyed-and-respawned car is a *new entity*, so its pair entries are
      gone. Confirm whether it collides again. If it does, the fix is a re-apply on respawn, not a per-tick loop.
- [ ] **Dummy range**: two cars close past the ~40 m LOD swap with no contact.
- [ ] **Whole grid, not just the near cars**: with one shot at the green, spread-out pairs are ghosted too.
- [ ] Toggle OFF → ON mid-race: the whole field goes ghosted at once.
- [ ] Toggle ON → OFF mid-race: contact returns at once.
- [ ] Script reload (Insert): the race restarts — confirm no stale ghosting survives into the new race.
- [ ] With the mode ON, a **shoulder-parked DNF car** is passed through rather than hit.

Report any single contact as a failure — that is the exact symptom the old system had.

## 13. Rival detection — now the ROUTE frame

The old hit test measured a straight line through the car, so a corner bent the racing line away from it and the
call bailed out; `FrontGap` was also written only after the closing guards, so it was a remembered value carried
across rivals and races. Both are gone in **`0623276`**: the gap is the route arc (`CumulativeDistance`, exact metres,
lap-agnostic), the closure is each car's own along-track speed (`Dot(Velocity, Direction)`), and the corridor is the
difference of the two cars' track-relative offsets — so no corner geometry can distort any of it.

42. **The reported symptom**: follow a rival until it pulls away, then take a corner. No swerve and no lift for it
    once it is past ~3 m.
43. In a corner, a rival 1-3 m in front must still be avoided and must still cap the throttle.
44. On a straight, rear-end prevention must still work: close on a slower car and confirm the lift/avoidance.
45. Rival walls must not appear from a rival more than 30 m of route ahead.
46. **The discriminator**: follow an AI into a ~50 m corner at ~90 km/h holding ~20 m of arc, and have it brake at
    the apex. The blue Rival sphere on the white steer line must appear and the throttle glide down. On the old model this
    was invisible — 3.99 m of lateral offset, bail-out, `FrontGap` 14.5 m — and the follower closed to contact.
47. **Negative control for 46**: the same 20 m of arc at matched speed → no sphere. Without this, "it lifted" does
    not distinguish the model from "always reacts to a car ahead".
48. **Nose-to-tail through the same corner**: two cars enter on the racing line one behind the other. The rear must
    keep tracking the front through the whole arc, and must lift for real if the front one brakes mid-corner.
49. **A stopped car on a tight hairpin's opposite leg** — known residual. `DirectionDiff` is forced to 0 below
    1 m/s, so a wreck on the other leg's line within 30 m of route can become an avoidance target. Watch for
    avoidance across the infield; the guard is a route-direction gate for slow cars.
50. **Stacked track (bridge / underpass / near-parallel legs)** — the other known residual: the off-track node
    rescan is nearest-in-3D with no continuity term, so a car that leaves the track there can lock onto the wrong
    deck, and every route quantity then reads against the wrong centreline.

Two things are deliberately **not** in this build: the 0.5 s two-path G projection (accurate to ~0.13 m at
R=50/25 m/s, but worth little until the rival refresh is lifted) and that 2 Hz `UpdateRivalInfo` cadence change.

---

## Open decisions still not coded

- **Etiquette exclusion**: a car held by the pack (Rival/ChillOut/Yield caps) can still reverse into the car behind.
- **B on fast excursions**: any 2 s off-track enters Drive, though the off-track cap already limits it.
---

## Crest entrance move (built `27ee097`, driver-verified)

The question is whether braking now starts where the track stops being flat, with the car still loaded when it does.

1. **A crest before a braking corner** — pick one with a long obvious rise. The plan should begin earlier than the previous build, at the base of the crest rather than its top; the corner-entry speed should read lower because more braking distance is being spent before the unload.
2. **The same corner twice** — once at pace, once deliberately slower. The move is a fixed node walk, so a slow approach spends the extra distance idling; if that looks like braking far too early, note the speed, because the gate should be braking-span-dependent rather than fixed.
3. **Log check** — generation now prints `entry=` beside `node=` per crest. Expect `entry` earlier in travel order than `node` (a lower index on a circuit), and the move distance on the `Crest` lines; a skip names its reason.
4. **A track with no crest before its corners** — nothing should change.

## Bump scan overlay (built `f3f318f`, rise walk revised through `148db70`, driver-verified — the cyan marker drew as expected)

The question is whether the scan finds the lips a driver can feel, and whether the ones it finds are the ones that launch a car. It reads the **route line only** — the raycast over the real surface, and the lane-local case with it, are a later stage.

1. **The overlay** - enable `Show Bumps` in the Debug submenu and drive the track. A cyan 3 m vertical tick and a line across the road mark every lip within 400 m of the player (`AutosportRacingSystem.cs:1717`). Expect a marker where the rise ends and the road drops away, not in the middle of a slope and not on every undulation.
2. **Log check** - generation prints one `Bump:` line per lip with `lip=`, `rise=`, `grade=` and `curvature=`, then a closing `Bumps: N lips, minimum departure grade …, run 2.0-8.0m` (`TrackLoader.cs:421-424`). Expect a handful to a few dozen on a circuit; a count near the node count means the detector floor is too low.
3. **A known launcher** - a jump the cars already take must carry a marker, and its `grade` should be the steepest of the set.
4. **A long climb** - a rise longer than eight metres must produce no marker at all. If it does, the run limit is not biting.
5. **The seam** - a circuit whose first/last node edge steps in height prints `Bumps: the route seam steps at grade …, so no lip is taken within 8.0m of it`; within that stretch no lip may be taken, while the rest of the circuit scans normally. A step-only seam is not a real lip.
6. **Nothing drives on it** - behaviour and lap times must match the build before `f3f318f`: the `RequiresEarlyBrake` / `RampEndNode` hook the braking plan reads is still unset, so a marked lip must not move anyone's braking point.

## Steering slew and slide blend (both driven)

The countersteer doubling was never validated and the base rate read as instant, so both became **one rate**
(`SteerSlewRate`) applied at `TranslateSteerToInput`. **Driven at 45: the AI was clearly less stable than before**,
a correction arriving too slowly, and **driven at 180: most of the stability problems went away** — so the
flattening was the defect, and the single rate is the 180 the old countersteer doubling used, full lock in about two
tenths of a second. The blend was re-cut (`Racer.cs:594`) at the same time:

- the weight ramps from zero at **0.25 × the authored peak slip** to full at **0.5 ×** it — the static
  `LateralTractionCurve`, not the speed-scaled peak, so it no longer moves with speed;
- the rear-led sign gate is **gone**: a slide of either sign now gets countersteer;
- the countersteer's share of the slide then ramps from **half to all of it** across **0.5 × to 1.0 ×** the peak.

Watch:

1. **A slide** — the countersteer should come in earlier and reach the whole slide. A car that still spins means
   the blend is not the limit.
2. **A deep understeer** — the rear-led gate was there because body slip alone cannot tell an oversteer from an
   understeer, and steering against the latter deepens it. Watch a front washing wide for exactly that.
3. **Brake release** — the pedal override is still gated on `_slidePriority >= 1`, which now arrives at **0.5 ×**
   the peak instead of 2.5 × the speed-scaled peak, so the brake lets go earlier in a slide. If that reads
   premature, re-anchor it to the share reaching full.
4. **A straight** — the actuator is fast again, so any weaving on the straight is the damper's and not the slew's;
   watch the straight and fast direction changes (the survey's 0.6 rad/s warning).

## The damper's low-speed gain (driven — improved)

`SteerDampingFor` scaled the dial by 25 divided by forward speed, floored at 1 m/s, so the damper ran at **25× the
dial** at a standstill, 11× at 5 mph, 3.7× at 15, and only reached 1× at 56 mph. It was born that way in `97cbc34`
and never had a cap. Below roughly 10 mph the term therefore saturates past the lock, and since the aim reference is
gated off at a standstill, a stopped car's damper opposes *all* rotation rather than the excess over what the corner
needs. The scale is now capped at 1, so the dial is the gain at every speed below 25 m/s. **Driver-verified: this
improved the system.**

Watch: a car stopped and pointing across the route should now steer toward lock and rotate onto it, where before it
sat and would not turn; and low-speed cornering should stop fighting itself.

**The damper's two subtraction rules (driven — works very well).** It may only ever *subtract*: a term that would
add steer in the sign the command already has is zeroed, so a left command never gets more left from it. And past
neutral — the countersteer region, where the wheel points against the car's own rotation rather than merely less
into it — it keeps half its authority. The axle gate that used to guard that halving (`understeerDeg < 0`, and
`UndersteerDegrees` with it) is deleted. Deliberately **not** a cap tied to the slide: the code already records that
such a cap fell to zero with the slide and took the rate feedback off a straight car, which set it oscillating.
**Driver-verified at `a0134d9`: works very well.** The no-add rule was expected to halve the damper's straight-line
action, and that cost has not shown up yet — watch a long straight for a weave or a limit cycle before calling it
settled for good.

## Steer ceiling — one governor (driver-verified)

The limiter has **one** governing quantity now: the slide. The yaw-usage governor, both Turn-In dials, the Steer Limit
Mode item and the mode switch are **gone**, because the two governors were answering the same question — *is this car
already committed to a rotation?* — one by proxying through yaw rate and the other by measuring the slide directly.
The proxy lost on its own evidence: the yaw cut granted authority for yaw the car already had, so under understeer,
where yaw is low *because* the steering is not working, it took steering away from the car that needed it most. Yaw
rate also cannot tell "turning in nicely" from "the rear has stepped out", and slide angle with its sign can.

The parts, unchanged:

- the **cornering ceiling** from the at-speed peak slip, with the geometry fallback when the live peak reads unusable;
- the **maneuvering ramp** under it — a raise only, easing up to the car's own lock at 5 mph and below;
- the slide **no longer adds** to the cornering law: that raise was retired by decision after the drive that showed the base law alone reads better, and is now deleted (`ebb3178`), so a sliding car is held to the peak-slip ceiling unless its command is the answering one;
- the **countersteer allowance** on the answering side, in proportion to the blend weight the slide has earned;
- the **damper bypass** — raised to full lock when the damper term *and* the command both oppose the rotation.

Reverse and standing still keep full lock, since the ramp is keyed to forward speed.

Watch:

1. **Straight line — this is the point of the drive.** The weave that only Slide-Governed showed is now universal,
   because the yaw cap that held a not-rotating car at about a fifth of the law is gone. One caveat before reading
   anything into it: the aligned-wheel deadband still zeroes the command when the aim bearing is small and nothing is
   sliding, so a weave can only be one whose bearing stays above that threshold. If it appears at the law's authority,
   the limiter is exonerated and the oscillation belongs to the loop under it — the pursuit and the damper.
2. **Corner entry — the behaviour change.** A not-yet-rotating car now gets the full cornering law at turn-in, where
   the yaw cut allowed it about a fifth of that. Watch for over-rotation on entry rather than understeer: this is what
   buying the authority costs, and it is the one thing to judge.
3. **Catching a slide** — a slide past the cornering law opens extra authority, and the answering side can reach the
   slide angle itself, so the counterbalancer must never be clipped.
4. **The bypass** — it needs both the damper term and the command to oppose the rotation, so a limiter that reads as
   absent while countersteering is working as intended.
5. **The retired settings** — Steer Limit Mode, Turn-In Minimum and Turn-In Maximum are gone from the menu, and
   `SettingsRepair` prunes them from the ini on the next load. That pruning is the intended behaviour this time, not
   the bug it was when the governor key was undeclared.
6. **Reverse** — a reversing car holds full lock. If a recovery reverse still reads as steering straight, the ramp is
   not the reason.
7. **Settings — a load check with a falsifier.** Every key the menu touches is declared, so `Log.log` must hold **no
   "declares no key" line** after a load. A line there means something wrote or read a key that `SettingsRepair` would
   prune, which is the bug class the store now refuses and reports instead of tolerating.

The law is plotted in `docs/steer-ceiling.png`, drawn by `docs/steer-ceiling.py`.

## Recovery drive — the coast, the edge aim and the exit (built `b50dfef`, dev build 478, driver-verified)

The Drive phase no longer forces its speed: the intention is the plan's, and a throttle cap lifts the pedal a band above the drive speed and opens it at or below, so the car coasts down rather than braking. Off track it aims at the nearest edge instead of the centre line. The exit test asks for any part of the car inside the bound, not the whole car.

1. **The coast** — send a car off the track and let it enter Drive. Expect the pedal lifted, a cyan sphere on the bar tagged `StuckRecovery`, and the car slowing to about the drive speed with **no brake**: the red pedal sphere should not appear while it does.
2. **The aim** — the car heads for the edge it left by, not across the track to the centre, and rejoins on that side.
3. **The exit** — recovery ends as soon as any part of the car is back on the track and it has been rolling forward over 4 mph for 0.5 s, which can now happen while its centre is still outside the bound. Normal racing must resume at once.
4. **Stuck on the surface (trigger A)** — reverse out of a wall while still on the track: the car must **not** steer at the edge (the edge aim is gated on being off track) and must return to the plan's speed rather than being held at the drive speed.
5. **The cap's floor** — below the drive speed the pedal is unrestricted, so a car in a ditch can still climb back out under power.

## Cards on the fast core pass (built `601313c`, dev build 483, driver-verified)

The four tactical cards — nitro, defend, divebomb, yield — and the maneuver cleanups used to be decided once a second. They now decide on the fast core pass, so a window that opens and closes between two slow beats is no longer missed. What they decide on: track state and perception from that same core tick, control state one core tick old, and rival records from the half-second publish — so the rival side is up to half a second old, which is the deliberate trade. DefendLane is the card where that can flip a live-versus-snapshot arrival comparison, and moving the publish itself onto the core pass is the next step. The crowd card and the rival scan kept their slow beats, and the card rate is not a fixed number: the pass serves six cars a frame, so a car's card rate falls as the grid grows.

1. **Something gets played** — with a full grid in a competitive pack, at least one AI should take a card it would previously have missed: a defend or a dive at an entrance it used to arrive at passively. Watch the debug focus car, not the player's.
2. **Nothing doubles** — nitro is still once per lap per car; a car must not burn twice inside one lap.
3. **No stutter** — a full grid must not cost frames. The moved work is a few loops over three rivals per car per core tick, so a hitch at the start or in a pack is the thing to suspect.
4. **The dive's braking is steady through a corner** — the bonus used to be redrawn at every evaluation on a circuit, so jittery dive braking was the symptom; it should now be one draw per dive.
5. **The crowd card is unchanged** — ChillOut should still come and go on its slow beat rather than flicker.
6. **A restart is clean** — arm a card or two, then Restart Race from the menu: nothing may carry over. A divebomb armed in the race just abandoned must not brake the new race's first corners, and the new race must not start with the pressure the last one built, whose symptom is cars refusing to yield through the opening laps.

## The slide's ceiling raise — retired by decision, then deleted (`ebb3178`; driven as build 487)

The limiter used to keep the larger of the cornering law and half the slide angle plus a half degree of free play as its ceiling. That raise was driven with the answering side left on, and the base law alone read better, so the decision is a keeper rather than an experiment, and nothing missed it: the raise, its two constants and the switch behind them are gone (`ebb3178`), leaving the limiter behaviour-identical to the build that was driven. Everything else in the function is in force — the base ceiling with its geometry fallback and low-speed ramp, the countersteer allowance on the answering side, and the damper bypass to full lock — so a driver answering a rotation still reaches past the ceiling; what is gone is the general loosening the slide granted both sides.

The checks that produced the decision, kept as the record:

1. **Steering into a slide** — a car sliding while its command is *not* the answering one is capped at the peak-slip ceiling instead of the raised one, so it asks for more wheel than it gets. Driven: the base reads better than the raise.
2. **The answering side** — unchanged, as intended: a countersteering car still reaches past the ceiling toward the slide angle, and the damper still lifts that side to full lock when its term opposes the rotation.
3. **Turn-in at speed** — unchanged: the raise never applied going into a corner, and the base ceiling is the same number.
4. **A slow car** — the maneuvering ramp is untouched.
5. **Restoring** — one flip, and nothing else in the function changed.

## The ramp band narrowed, the damper's speed is a setting, and the ceiling takes a factor (dev build 489, not driven)

Three changes on the speed axis. The limiter's maneuvering ramp still rises from the car's full lock at its start speed to the law's ceiling at its end speed, but the end speed moved down (30 to 15 mph), so the band it governs is short and above it the ceiling is the tyre's peak-slip law alone where the old chord held it several degrees higher. The damper's gain was defined at a hard-coded speed; that speed is now the **Steer Damping Speed (mph)** item in Rivals > Rival Steering (10 to 250 mph in steps of 30, default 70 — the old constant's nearest offer in the new domain; an ini holding 60 snaps to 70 on load), the speed at which the gain is the dial and above which it falls as that speed over the car's. And the **Steer Limit At Speed** item (0.20 / 0.50 / 0.75 / 1.00, default 1.00) scales the at-speed ceiling the limiter computes from the tyre's live peak slip — the corner-geometry fallback is not scaled, and the ramp's end anchor scales with it so the band stays continuous.

1. **Corner entry in the narrowed band** — the case the ramp change bites: the wheel is capped tighter than the old chord allowed, so watch a slow corner for a car that cannot place itself, or one that turns in more lazily than it did. Above the old end speed nothing moved.
2. **Below the ramp's start** — placement is still near full lock: a car nosing out of a ditch or off the line must still steer.
3. **The damper knob's range** — at 10 the damper gives up early and a fast car should read lively and loose; at 250 it is the dial all the way to racing speed. The surprise that motivated the wider range was 90 holding at ×1 — so check whether 130 or 190 reads any different from 90, or whether the damper stops being the limit well below either.
4. **The ceiling factor** — at 0.20 the at-speed ceiling is a fifth of the tyre law: the plan's target speeds do not read the limiter, so expect cars asking for the same corners and washing visibly wide for want of wheel. At 0.75 the loss should be subtle. At 1.00 the build must drive exactly like 488.
5. **The menu and the file** — both items sit under Steer Damping in Rivals > Rival Steering, their values survive a reload, and `Log.log` holds no "declares no key" line after a load.

The ceiling law is plotted in `docs/steer-ceiling.png`, regenerated with the narrower band (the factor is a flat scale on top of it).

## The debug streamline, the overspeed ramp and the lane base (dev build 521 — the overspeed ramp driven, three owed)

1. **The lane base** (`dd484d3`) — with no corner line, no avoidance and no high-speed answer, the target lane is now the lane the car is already in rather than the centre. On a straight a car that arrives off-line should keep its offset instead of drifting to the middle, and it must still take the inside line the moment a corner's window opens. The failure to watch for is the opposite of the old bug: a car left at the edge of the track with nothing pulling it back — the rival walls and the off-track recovery still bound it, but nothing else does.
2. **The overspeed ramp** (`2dcbbcc`) — **driven, reads right**: the cars do not overspeed, and they do not lift too readily either, the pair the half-throttle ladder could not hold at once, so the two thresholds stay where they are. The throttle cap falls linearly between them instead of stepping in half-throttle rungs, so a closed throttle now needs a far larger surplus than the ladder demanded; if it ever turns weak, the tell is a car on a long climb gaining speed its wheels cannot explain.
3. **The ceiling band's continuity** — `e59e0a9` gives the ramp's end anchor the same factor the cap in force takes, which is what the section above already claims. At Steer Limit At Speed 1.00 nothing should change; at 0.50 watch for the ceiling no longer stepping down as a car accelerates through the ramp's end (it did step, by the factor's shortfall).
4. **The debug visuals the streamline changed** — the reason limits riding the white steer line, the corner rings and their label gone, and the trail's chevrons. Only the trail has been reported on in game; the limit spheres have not.

## The TCS and ABS taper, and its two knobs (dev build 523 — the TCS half driven, the ABS half owed)

**Driven:** the TCS taper reads right at the live pair (`TcsSlipThreshold` 0.2 with `TcsTaperSpan` 0.3) — the cut fades in and settles rather than stepping. Still owed: the ABS half of the same shape (`f028a2e`), which cuts at the knee and floors at mingrip with no knobs of its own, and the knob extremes, which are the point of the range. **The ABS half needs a setup rather than a drive**: a car with no natural ABS of its own on a low-grip surface, so the wheels actually lock, and the instrument is the pedal bar's limit sphere for the cap (`Racer.cs:2566`).

1. **The shape** — both caps now cut on a tanh between the threshold and one span past it, so the first response fades in instead of starting at a slope step and the floor is eased onto rather than hit. What should be felt is the *middle* of the span: a tanh runs above its own chord, so the car is trimmed a little earlier than the old straight ramp was at the same slip, and settles onto the floor rather than arriving at it.
2. **The knobs** — Rival Traction holds **TCS Wheelspin Limit** (0.1 to 2.5 in steps of 0.1, default 0.5) and **TCS Cut Softness** (0.1 to 0.5 in steps of 0.1, default 0.5). Both are in the tyre's slip-ratio units, the same ones the traction curve is: the curve peaks at 1.0 and is flat past the knee at 2.5, which is the top of the threshold's range. **The ceiling is the sum of the two**, so the shipped pair regulates to half a slip unit and floors the cut at exactly 1.0 — the traction peak. Raise the threshold to let the AI spin more before it intervenes; widen the span to make the cut more gradual. The extremes are the point rather than a side effect: at 0.1 + 0.1 the field drives more safely than anything the old law could express.
3. **Off-track halves it, and nothing else moves it** — the threshold is the setting on every surface and at every speed now: the surface-grip scaling is gone (it tightened the regulation where grip was scarce, tracking the lower peak slip a loose surface has, and it went for simplicity) and so is the launch ramp. The off-track half is the only modulation left, and the surface no longer reads into the value at all. Expect the cars to be a shade more permissive than before wherever grip is scarce and off the line — 0.5 everywhere, where the old law gave more slip at launch and less on dirt. A launch allowance, as a setting or a per-racer personality, is filed as `launch-slip-allowance`.
4. **ABS** — the same shape with no knobs: the cut still starts at the knee and floors at mingrip, which are the traction curve's own numbers.
5. **The menu and the file** — both items sit under TCS in Rival Traction, and their keys appear in `Menu-Settings.ini` on the first load after this build; `Log.log` must hold no "declares no key" line, and both values must survive a reload.

## The ceiling's reference is the wheel midpoint (driven: 1.00 squeals, 0.75 mostly clean)

**Driven:** at 1.00 the tyres squeal through a corner; at 0.75 the squeal is mostly gone, which is the crossing this change was for — the cap sits just under the peak. The wheel-slip spheres agree, and put a number on it: with them on, the tyres read near full green — the top of the traction curve — at 1.00, and below the peak at 0.75. The tight-slow-corner check below is the one still owed.

The peak-slip ceiling was stated at the *outer* front wheel. The engine scales one front wheel to `TEMP_STEER_WHEEL_MULT` (`wheel.cpp:6445`, `CWheel::SetSteerAngle`) and leaves the other at the command, and the ceiling divided the peak slip by that 0.75 so the outer wheel landed on it. A two-wheel model wants the *axle* angle, which is the midpoint of the two — `2 / (1 + 0.75)` — so the ceiling is that now (`PeakSlipMidpointCommandShare`, `Racer.cs`), and the menu says what 1.00 means.

1. **The size of the change.** The cap is a flat 14% tighter at every speed and every factor, so the new 1.00 sits between the old 1.00 and the old 0.75. Judge the drive against that bracketing, not against a setting that reproduces 488 exactly.
2. **The tight stuff.** The tightened cap bites hardest above the ramp's end speed, where the ramp no longer reaches: watch a slow corner for a car that cannot place itself, or that turns in more lazily than it did.
3. **The item and the file.** The description now names what 1.00 means; confirm it shows, the value survives a reload, and `Log.log` holds no "declares no key" line after a load.

## The measured-slip allowance under the ceiling (PARKED — built, driven once, switched off; do not drive)

**Parked** by decision after the first drive: the swerves read slower, but what the allowance should *be* — a rate limit on authority or an amplitude reference — is a design call, filed as `steer-slip-allowance`. `SlipPeakCeiling` is off, so the list below is what a future session resumes from rather than what is owed now.

The ceiling in force is now the lower of the model law and an allowance read from the tyre: the wheel may lead the side slip it already carries by `SteerSlipLeadShare`, and the allowance climbs to the whole peak over `SteerSlipClimbSeconds` while the command keeps asking for more than it grants, falling at the same rate once it stops. The climb is timed to the peak, so a lower Steer Limit At Speed cuts the plateau without changing when it arrives.

1. **The cyan walls are the instrument.** `ResolveSteerCeiling` draws them, so the ceiling in force is visible: it should widen over about a second from a turn-in off a straight, then hold. Snapping wide instantly means the climb is not scaling with the core-tick time; never widening means the demand test is not firing.
2. **The timing** — the first drive read the swerves as slower, which is the effect the law exists for, but the climb as a bit fast; judge whether a second is now too slow rather than too fast.
3. **The hunt** — the cap chases the ask, so when the ask sits near the cap the allowance can oscillate at the climb rate. The tell is the walls breathing on a straight, or a weave there. The fix is asymmetric rates: climb in a second, fall over two or three.
4. **A slide** — the allowance reaches the whole peak at the climb rate rather than instantly, so the model ceiling returns over that same second while the countersteer allowance and the damper bypass keep raising on top throughout.

## The outside approach is armed per approach, and a skipped approach is not asked for (dev build 535 — the arming half driven, the skip and the braking check owed)

**Driven:** the outside line resets per approach (`7619347`) — no locked-in outsides carried from a previous corner's exit. Still owed: the skip itself (`1395c30`), which should show as no swing to the outer edge after an apex and no swing-out between the two corners of a chicane or S inside the 2–3.95 s band; and item 4 below, that braking still holds through the apex.

Both gates are in time, and both live in `UpdateCornerRequirements`. The arming flag `RequiresPositioning` used to latch for the race, so the 3.95 s window bounded only the first approach and every later one had the outside line in force from the previous corner's exit; it now clears once the corner's apex is behind, so each approach asks again inside its own window. The second gate is the approach itself: a corner promoted to queue slot 0 while it is *already* inside that window has no approach left to drive, and it is skipped for the whole approach, because asking for the outer edge there is the step the window exists to avoid. `RequiresBraking` is deliberately left latched, because its range is measured to the entrance and the flag has to outlive that to hold a plan through the apex.

Promotion happens at the previous corner's apex (`UpdateApexLeapfrog`), so the second gate **is** the successive-corner rule: "promoted already inside the window" and "the second corner is within the window of the first corner's apex" are the same measurement. The generation merges pairs closer than 2 s, or 3 s when they aim the same way (`TrackLoader.cs`), so the band this gate bites is what survives a merge: roughly 2 to 3.95 s. The blend itself is unchanged — inside the window the target is the outer edge immediately and the ramp inward begins 34 to 55 m before the apex, which is the intended outside approach rather than a defect.

1. **The long straight** — a car that used to sit on the outer edge from the previous corner onward should now stay where it was and take the outside only inside its own window, about four seconds out. The tell of the arming gate working is a difference between the first approach to a corner and every later one, where the old behaviour was identical.
2. **The skipped approach** — a corner the car meets already inside its window must now get **no** outside line, so it takes the inside line from wherever it is rather than swinging out. The tell of the gate working is the absence of a step to the outer edge just after a previous apex; the tell of it being too eager is a tight corner taken with no outside line when the car visibly had room and time for one.
3. **Successive corners** — the band this gate bites is 2 to 3.95 s between a corner's apex and the next corner's entrance, which is what survives the generation's merge. A chicane or an S inside that band must not have the car swing out between the two; if it still does, the promotion the gate judged was not the one expected.
4. **Braking is untouched** — the two flags share one loop and only the positioning one is cleared, so a corner's braking plan must still hold through its apex; the tell of a mistake is a car that stops slowing just before an apex.

## The 0.9 menu and PI batch (NOT DRIVEN — every item here is compile-verified only)

The whole session's UI work shipped without a game session: the menu tree, the two copy sweeps, the Reset gate, the named autofix options, the PI deviation limit and the Start Race line. Nothing below has been seen in game.

1. **The tree loads at all.** The root should be seven submenus in frequency order — Race, Rivals, Race Rules, Interface, Controls, Debug, Track Creator — plus Reset To Track as the last row. A constructor fault would have stopped the script loading, so check `Log.log` for a load error and open every menu once.
2. **Every settings item survives a reload.** The homes all moved and one ini domain changed, so change one item per menu, reload with Insert, and confirm each value came back; `Log.log` must hold no "declares no key" line.
3. **Racer Autofix reads as names.** The row should offer Off / Damage Resistant / Invincible, and a previously stored `0`/`1`/`2` must have migrated rather than reset. The open question rides this row: under Invincible the player's own car is repaired during a race, because the repair half has no player guard (`Racer.cs:3350`).
4. **Max PI Deviation actually caps the grid.** Set a target and a window, build a grid, and check the field's PIs sit inside the window and match the log's `PI limit N around X: …` line. A window nothing fits must give the explicit no-vehicles-in-window error rather than a silent solo grid.
5. **Reset To Track is race-only.** In free roam the row must be greyed and must not teleport; mid-race it behaves as it always did. If it is still clickable, the guard's notification should say why it refused.
6. **Start Race reads right and spectates.** In a car the race starts with you in it; on foot it must run with you watching from the sideline; an empty grid must notify instead of returning silently.
7. **The copy reads correctly in place** — the Rivals family, the two renamed PI offsets, the traction and steering rows, the renamed options and the Racer Autofix list. Every line was written so the name stands alone and the tooltip carries only what the name cannot.
8. **One item cannot be driven at all**: the update notice now prefers the release title, so it only fires against a newer release — it waits for the first cut after this one.

## The lane ask is scaled down as the car crosses (DRIVEN — softer, still confident)

**Switchable (`b390029`):** `LaneCrossingScale` gates the multiply, so the unscaled ask can be driven beside the scaled one — shipped off for that comparison. The verdict below was reached with the scale on and no lane inertia upstream of it, so the two have never been compared in the order they now sit, and that drive is for the comparison itself rather than a decision. **First read with it off (`b390029`):** turn-in reads more decisive without too much aggression — but that is the approach side, while the scale's own case is a car crossing hard toward a lane it would reach anyway, and this read did not single that out.

**Driven:** the reads are softer than either rejected shape — the hard cap on the wheel and the step on the crossing — while the cars still steer with confidence, which is the scale doing its job with the allowance left where it was. What that drive did not cover: the fleet-wide read, since the car in hand was high grip, and the corner-line delivery at speed, which is item 3 below.

The lane steer is pure pursuit on the bearing to its aim point, and the first answer to an over-large ask was a ceiling on the wheel angle — the wrong quantity and the wrong shape. What is bounded now is the **crossing angle**: the car's heading against the track direction at the car. `LaneCrossingLateralSpeedMps` sets the crossing that costs half the ask, and because it is defined as a sideways rate it follows `1/speed`; `LaneCrossingShare` (`Racer.cs`) is `allowance / (allowance + crossing)`, a scale that fades with the crossing and never reaches nothing. Two shapes were tried and rejected on the way here — a hard cap on the wheel, and a step on the crossing — which is why this one is soft by construction and has no width to tune. **Corner entry is exempt, and only the entry** — from the braking target to the apex, because turn-in is a crossing on purpose: the car has to point into the turn to rotate, so a bound on the crossing fights turn-in whatever its shape or schedule. Everything before the braking target is still lane work and is scaled, which is the point: the first cut of this gate keyed on the corner owning the lane at all, and the positioning window opens seconds early, so on a twisty track it was off for most of a lap.

1. **Mid-straight, a car well off its lane** — the case it exists for. The car should stop angling itself across its own direction to reach a lane it would reach anyway: the yellow pursuit line in Show Track Analysis should keep nearly all its authority until the crossing approaches the allowance and then fade — half at the allowance, a third at twice it — rather than switching off.
2. **The unwind must never be scaled** — the deliberate asymmetry, a sign test rather than a fade. A car already crossed and steering back keeps its whole ask; if it struggles to straighten, that test is the first thing to check.
3. **The cost** — at speed the same term delivers the corner's inside line, the outside approach and the avoid-ahead pass, so lane moves slow once the crossing passes the threshold. A car reaching its line late into a corner, or a pass that no longer completes before it, is this one. `LaneCrossingLateralSpeedMps` is the knob, and scaling speed up or down in the threshold is the intended fiddle.
4. **Fast corners** — a car at the limit genuinely carries a few degrees of crossing, so the scale bites there too; the tell is a car that cannot hold its inside line through a quick bend. **The fade is what to judge here**: there is no width to tune and no cliff, so chatter around the allowance is not expected — if the cars still cross too hard, the allowance is the knob rather than a new shape.
5. **Low speed must be unchanged** — the threshold grows as speed falls, so placing a crawling car, an off-line start or a recovery must still turn as it did.

## Apex speed reads the smoothed radius (driven on high-grip cars: faster and more consistent — the fleet-wide read is still owed)

Corner speed is `√(grip · g · R)` and `R` was `SupposedRadius`, the region's **minimum precise** radius — a short-chord measurement, so a minimum over many samples sat on the low tail of the noise and held every corner speed down. The speed path now reads `DetectedRadius`, the smoothed region radius the rest of the system already used: the tight flag (`Racer.cs:2986`), the entrance and exit spans and the merge survivor (`TrackLoader.cs:698-699`, `:737`). Six sites moved: `CornerApexSpeed` (`AutosportRacingSystem.cs:2779`), the outside-line entry gate (`Racer.cs:833`), the braking-requirement flag's intended speed (`Racer.cs:2989`) and the three held-apex radii (`Racer.cs:3039`, `:3055`, `:3069`).

1. **Cars brake later everywhere.** This raises every corner speed, so the first thing to expect is later, deeper braking into slow corners. The steering pipeline did not change at all; a car that turns in lazily *at the same speed* is not this.
2. **The failure to watch is over-confidence.** A smoothed radius is also a larger one, so a car that was conservative now carries more speed to the apex — the tell is a car washing wide or sliding on entry in a tight corner, especially one whose apex sits on a noise spike. The tight stuff is where this holds or does not.
3. **Judge it against the offsets at 0.** Both offsets ship at **+6 mph** (`AutosportRacingSystem.cs:153`) and the plan takes the lower of the two terms (`Racer.cs:1629`), so the +6 has been masking this bias. At 0 the plan's own number is on show; at the default the fudge absorbs part of the change.
4. **The radius pair is no longer visible in game** — the circle label that printed `R plan/detected` is gone, so the per-corner gap cannot be read off the screen and behaviour is the evidence.

## Contact avoidance is its own term, added after the damper and the blend (driver-observed, gate read back)

The steer-away correction used to be folded into the lane steer, so the lane system's own stages acted on it: the yaw damper modulated it, the slide handover replaced the command with a countersteer target that deliberately omits the lane term, and the on-aim rule zeroed the wheel. It is now its own `contactSteerDeg` (`Racer.cs:591-612`), added at `:657` after all three; only the steer limiter and the final slew bound it.

**The gate, read back.** A rival is skipped when it is itself slower than 0.1 m/s, when it sits outside a box of half the two lengths plus a metre by half the two widths plus three, or unless its lateral offset and its lateral closing rate in this car's frame carry **opposite signs** (`:607`) — the term is armed only while the pair closes sideways, so a rival drifting away laterally is excluded by that same line, and an exactly aligned one is excluded too, which is why a car dead ahead gets nothing from it. The closure is *relative*, so this car's own sideways movement arms it as much as the rival's, and the strength remap floors at 0.3 m/s of closing (`:610`), so a closing below that floor passes the gate and contributes exactly zero rather than a jitter.

**The sibling term does not share that gate.** The side-by-side heading match (`:679-711`) tests the rival's *own* lateral velocity toward this car (`:702`) and not the relative one, which its comment states deliberately — this car's own closure is the rival walls' job — so it can stay armed while the pair separates under this car's movement, and at zero lateral separation it stays on by design where the contact term goes quiet. Both terms read only the three rivals in `Brain.Rivals` (`DataStructures.cs:115`), so a fourth car alongside is invisible to either.

Observed reading fine in a race, but not at any of the four checks below — none of them was driven deliberately.

1. **A slide alongside a rival** — the case the change exists for. A car sliding with someone closing on its flank must now steer away instead of handing the wheel entirely to the countersteer. The tell of the old behaviour is the wheel going quiet exactly as the slide peaked.
2. **On its line with a rival closing** — a car sitting on its aim with a rival closing laterally used to have its command zeroed. It must now step away. Too little and nothing changes; too much reads as a twitch on a straight.
3. **The margin** — a rival closing just past the remap's floor (0.3 m/s) gives a fraction of a degree with nothing suppressing it any more, and at the floor itself the clamped remap gives exactly none, so watch for a small wheel jitter in side-by-side traffic that the old on-aim zeroing hid. The floor is the knob if it shows.
4. **What must not change** — the limiter and the slew still close the steering last, so the push cannot exceed the tyre's allowance within one tick; and with no rival closing the term is zero, so every other behaviour should be identical.

## Every racer brakes from its own confidence (NOT DRIVEN)

The factor each corner starts from was one per-corner draw around 0.75, so a fresh grid differed corner by corner but every racer averaged the same centre. It is two draws now: `_brakeConfidenceBias`, taken once in the constructor (`Racer.cs:362`) and added to every corner that racer seeds, plus the per-corner draw unchanged (`SeededBrakeFactor` `Racer.cs:275`). Both are ±0.05, so a racer starts anywhere in 0.65–0.85 with a centre of its own, and the bias is the half that moves a lap because it repeats across every corner while the per-corner draw averages out.

1. **The draw is in the log, so this needs no drive.** Starting a race must write one `Brake confidence bias <car>: <value>` line per racer to `Log.log`, spread across ±0.05 rather than all reading the same number. A grid of identical lines means the draw is per corner after all, or the cars never reach the constructor.
2. **Brake Learning off stays uniform.** With the toggle off `BrakeFactorForApex` returns the flat seed and no corner context is ever seeded, so the field must brake identically again — that is what the option promises, and it is the baseline for judging the on state.
3. **The opening lap is the only place to see it.** The first commit moves a corner's factor up to ±0.2 against a ±0.05 seed, so the bias is a lap-one difference and the field converges after that; a spread that has gone by lap two is the design working, not the draw failing.
4. **The feel** — with one centre for the field, the pack used to brake as a single car into the first corner; each racer should now be slightly earlier or later. A car *visibly* overshooting corners is the ±0.05 bias being too much rather than a fault in the draw, and `BrakeConfidenceJitterHundredths` (`Racer.cs:206`) is the knob.

## The lane mover — the aim is carried onto the request (driver-observed as right)

The override chain still picks the lane, but its answer is now a request (`_laneRequest`) and the aimed lane (`_targetLane`) is carried onto it by `UpdateLaneMover` at 14 m/s^2 with a 12 m/s cap, so a requirement that is valid for half a second moves the lane a fraction of the way instead of stepping it. The request's own rate feeds forward, so a line that moves smoothly is still tracked without the lag a pure arrival law would leave; the rival walls bound the aim as well as the request; the green and the recovery snap re-seed it; and a core tick arriving after a stall is bounded at 100 ms. **Driven: the lanes read right — smooth, and the flapping this exists for is gone.**

1. **The numbers are settled for one geometry.** `B` is the safe bound (half the track width less half the car) and the line's own motion scales with it, so the settling case of a 5 m half-width does not cover a wider track, whose line moves further and would want a little more acceleration. `LaneMoverAccelMps2` is the knob, and the cap has to stay above the line's own peak rate or the line itself is clipped.
2. **If it is retuned, the log is the instrument** — `Lane mover <car>: asked …, aimed …, rate …`, at most once a second per car while the lane is actually moving.
3. **What the mover deliberately does not touch** — contact avoidance stays its own steer term added after the damper, while the recovery's nearest-edge lane goes through the mover like every other source, so a turn-back that reads late is the case to watch for if one is ever seen.

## The inside line is a pull, not the edge (driver-verified — reads right)

**Driven:** the lean reads right. The checks below stay as the record of what the shape is judged on if it is ever doubted.

The high-speed source no longer answers the full inside edge on a bend that passes its 500 m radius gate. It answers the car's own lane plus X metres toward the inside, recomputed every tick, with X one metre times a linear fade from 500 m down to 250 m of radius — nothing at the gate, half a metre at 375, the full metre at 250 and held below. Because the target is relative it recedes as the car moves, so the crossing stays alive for as long as the bend lasts and the drivable bound is what ends it; the lane mover stays the only rate bound.

**Rejected on the way here**: a proportional *destination*, where the car aims X inside and stops, which leaves a long bend short of the inside; and a speed-driven allowance, because speed is the result of the curvature — it misreads a straight as a bend and the slowest corner as the one needing least line.

1. **Judge it where it is visible, not at the gate.** X below the aligned-wheel deadband steers nothing, so the answer first appears near 465 m at the lower lookahead floor and near 370 m once the lookahead caps; a no-op read at 500 m is the deadband, not the change failing.
2. **The corner line still wins.** The pull is replaced wherever the corner line answers, so only bends the corner detector has not claimed show it, and judging it on a registered corner is the likeliest misattribution.
3. **The ask and the crossing are different readings.** On a tight bend at the limit the limiter clips the extra ask, so "a tight bend pulls harder" can be true of the intended lane and false of the realised crossing. Read the intended lane against the live deviation, not the crossing alone.
4. **A bend long enough to traverse ends at the edge and holds it.** The traverse is computed from the observed crossing rate rather than a stopwatch, because X sets a rate and not a position; stopping short on a long bend, or crossing past the edge, is the falsifier.
5. **No step, and no self-excitation.** No aim step when the chord node's radius jumps or across the gate; and since the request is now anchored on the car's own lane, the tell of the aim and the crossing trading through the mover's feedforward is the intended line sawtoothing against the live one.
6. **The magnitude does not move with speed.** Only the deadband should make the onset differ between a slow and a fast pass; a speed-dependent X is a falsifier.
7. **On exit it holds.** When the bend ends the source reports nothing, the request returns to the car's own lane, and the car holds the position it reached rather than unwinding.

## The lane carries the reason it changed (compile-verified, drive owed)

The lane was a per-tick decision with no record of why it moved, which made every handoff an inference. Whoever writes the request now writes a reason beside it — own lane, curve lean, outside line, apex line, commit line, avoidance, recovery edge, wall clamp, debug lock — and that tag also replaces the aimed flag, since aimed is now simply that the reason is not own lane. The corner line reports its two phases separately: the outer hold is the profile's far clamp, and everything below it is the blend toward the apex.

1. **The log names it** — `Lane mover <car>: asked …, (reason), aimed …, rate …`, at most once a second per car while the lane is moving; a *static* reason is never logged, so the always-on read-out is the lane line.
2. **The lane line carries it** — the intended-lane colour is the reason, own lane keeping red and black still meaning no aim at all, plus a label at the head of the line for the focus racer. Wall clamp and the debug pin are grey on purpose: a bound is not an opinion about where to go.
3. **One behaviour moved, and it is the flag.** The wall clamp is now a writer where it was not, so a car whose lane sits outside the drivable edge arms the aimed flag — and through it the outside-projection throttle cap — from the first tick rather than once the mover has brought the aim in. That is the state the cap exists to police, so it reads as intended rather than as a fault; it is on this list because it is a change and not a refactor.
4. **The reason describes the request, not the aim.** The mover carries the aim onto the request at a bounded rate, so during that lag the label names a lane the car has not reached yet. That is why the property is named for the request.
5. **A zero-valued opinion reads as silence.** The profile passes exactly through zero at the midpoint of its blend and the chain treats a zero answer as no answer, so the tag can flicker for a tick there. It predates the tag, which only makes it visible.
