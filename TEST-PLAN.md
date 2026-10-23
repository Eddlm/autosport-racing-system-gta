# ARS — test plan: build 483

**Status: the steering chain, the lane laws and the contact-avoidance term are closed by the driver, and the sections still marked owed below are the cards, the pedals, the debug visuals, the menu batch and the fleet-wide radius read.** Everything else this file listed as outstanding has since been driven and held in one session. The recovery's reverse phase and its teleport, gated on the escape budget, were driver-verified on build 450 (`666c89b`), and the rest of the recovery went with them — trigger B, the exit back into normal racing, the start-line arm delay and the flooded-engine rescue — after which the drive phase was rebuilt again (coast instead of brake, nearest-edge aim, exit on any part of the car on track; `b50dfef`, section at the end) and that rebuild has been driven too. The same drive covered the maneuver cards on the fast core pass with a clean restart (`601313c`, last section), the steer ceiling down to one governor, the slide blend after the slew, the crest entrance move, DNF parking in both options with a race ending on DNFs, the route-frame rival detection, and the bump scan overlay, whose cyan marker was confirmed in game. The limiter's slide raise then went off, was driven in that state, and its retirement was kept, so that decision is settled; the changes after it — the narrowed ramp band, the damper's speed setting and the ceiling factor (`489`, last section) — are **driver-verified**, which closes the ladder's `steer-ramp-band`. What the drive did not settle: the crest's gating constants still wait on an instrumented run, and the two residuals the ladder names for the route-frame detection were not provoked. No Collision's one-shot mode and its per-tick all-pairs form were both
driven and **failed** against the engine's one-slot limit, and its nearest-rival form is now **driver-verified**
(`cbae845`); the slew is driven — 45 read as less stable and **180 removed most of the stability problems**
(`393bfdd`, driver-verified); the damper's speed scale is capped at 1 and **driver-verified as an improvement**
(`5bb94bd`), and its two subtraction rules are **driver-verified as working very well** (`a0134d9`); the damper bypass and the maneuvering ramp are new; **build 473 collapsed the steer governor
to the slide**, so a car at turn-in now gets the full cornering law where
the retired yaw cut allowed it about a fifth of that — driven with the rest of the list, so that authority is in. Build 471's state was accepted on the drive — the simplification
batch and the governor-key fix regressed nothing. The deployed DLL writes its build number to `build-number.txt` beside it, and the counter moves on every compile, so read that file rather than trusting a number written here; a build + reload is enough
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

## The ramp band narrowed, the damper's speed is a setting, and the ceiling takes a factor (dev build 489 — driver-verified)

Three changes on the speed axis. The limiter's maneuvering ramp still rises from the car's full lock at its start speed to the law's ceiling at its end speed, but the end speed moved down (30 to 15 mph), so the band it governs is short and above it the ceiling is the tyre's peak-slip law alone where the old chord held it several degrees higher. The damper's gain was defined at a hard-coded speed; that speed is now the **Steer Damping Speed (mph)** item in Rivals > Rival Steering (10 to 250 mph in steps of 30, default 70 — the old constant's nearest offer in the new domain; an ini holding 60 snaps to 70 on load), the speed at which the gain is the dial and above which it falls as that speed over the car's. And the **Steer Limit At Speed** item (0.05 to 1.25 in 0.05 steps, default 1.00) scales the at-speed ceiling the limiter computes from the tyre's live peak slip — the corner-geometry fallback is not scaled, and the ramp's end anchor scales with it so the band stays continuous.

**Revised: the Steer Damping Speed item is REMOVED (driver, build 711).** At a dial of 1.0 the cars damped cleanly at every speed, so the gain no longer falls with speed at all: the item, `ARS.SteerDampingSpeedMph` and the `1/|v|` schedule are gone and `SteerDampingFor` is the dial over the grip divisor at every speed. A stored `SteerDampingSpeed` is dropped on load as an undeclared key; item 3 above and the speed half of this section no longer apply.

1. **Corner entry in the narrowed band** — the case the ramp change bites: the wheel is capped tighter than the old chord allowed, so watch a slow corner for a car that cannot place itself, or one that turns in more lazily than it did. Above the old end speed nothing moved.
2. **Below the ramp's start** — placement is still near full lock: a car nosing out of a ditch or off the line must still steer.
3. **The damper knob's range** — at 10 the damper gives up early and a fast car should read lively and loose; at 250 it is the dial all the way to racing speed. The surprise that motivated the wider range was 90 holding at ×1 — so check whether 130 or 190 reads any different from 90, or whether the damper stops being the limit well below either.
4. **The ceiling factor** — at 0.20 the at-speed ceiling is a fifth of the tyre law: the plan's target speeds do not read the limiter, so expect cars asking for the same corners and washing visibly wide for want of wheel. At 0.75 the loss should be subtle. At 1.00 the build must drive exactly like 488. The range was widened after that batch (`29b267b`); the outer-reference change (`0dfbf53`) sits on top of it: at 1.25 the ceiling is about 1.67× the modelled peak slip, so the wheel is asked well past the tyre's peak on purpose and the read is whether the rivals visibly slide; on a car whose lock is below that product the top of the range changes nothing.
5. **The menu and the file** — both items sit under Steer Damping in Rivals > Rival Steering, their values survive a reload, and `Log.log` holds no "declares no key" line after a load.

The ceiling law is plotted in `docs/steer-ceiling.png`, regenerated with the narrower band (the factor is a flat scale on top of it).

## The debug streamline, the overspeed ramp and the lane base (dev build 521 — the ramp, the lane base and the band continuity driven; the debug visuals owed)

1. **The lane base** (`dd484d3`) — with no corner line, no avoidance and no lean answer, the carried lane is now the lane the car is already in rather than the centre. On a straight a car that arrives off-line should keep its offset instead of drifting to the middle, and it must still take the inside line the moment a corner's window opens. The failure to watch for is the opposite of the old bug: a car left at the edge of the track with nothing pulling it back — the rival walls and the off-track recovery still bound it, but nothing else does.
2. **The overspeed ramp** (`2dcbbcc`) — **driven, reads right**: the cars do not overspeed, and they do not lift too readily either, the pair the half-throttle ladder could not hold at once, so the two thresholds stay where they are. The throttle cap falls linearly between them instead of stepping in half-throttle rungs, so a closed throttle now needs a far larger surplus than the ladder demanded; if it ever turns weak, the tell is a car on a long climb gaining speed its wheels cannot explain.
3. **The ceiling band's continuity** — `e59e0a9` gives the ramp's end anchor the same factor the cap in force takes, which is what the section above already claims. At Steer Limit At Speed 1.00 nothing should change; at 0.50 watch for the ceiling no longer stepping down as a car accelerates through the ramp's end (it did step, by the factor's shortfall).
4. **The debug visuals the streamline changed** — the reason limits riding the white steer line, the corner rings and their label gone, and the trail's chevrons. Only the trail has been reported on in game; the limit spheres have not.

## The TCS and ABS taper, and its two knobs (dev build 523 — the TCS half driven, the ABS half owed)

**Superseded by the share restatement at the end of this file**: the pair driven here, 0.2 with 0.3, is 50% and 75% of the peak in the units the knobs carry now.

**Driven:** the TCS taper reads right at the live pair (`TcsSlipThreshold` 0.2 with `TcsTaperSpan` 0.3) — the cut fades in and settles rather than stepping. Still owed: the ABS half of the same shape (`f028a2e`), which cuts at the knee and floors at mingrip with no knobs of its own, and the knob extremes, which are the point of the range. **The ABS half needs a setup rather than a drive**: a car with no natural ABS of its own on a low-grip surface, so the wheels actually lock, and the instrument is the pedal bar's limit sphere for the cap (`Racer.cs:2566`).

1. **The shape** — both caps now cut on a tanh between the threshold and one span past it, so the first response fades in instead of starting at a slope step and the floor is eased onto rather than hit. What should be felt is the *middle* of the span: a tanh runs above its own chord, so the car is trimmed a little earlier than the old straight ramp was at the same slip, and settles onto the floor rather than arriving at it.
2. **The knobs** — Rival Traction holds **TCS Wheelspin Limit** (0.1 to 2.5 in steps of 0.1, default 0.5) and **TCS Cut Softness** (0.1 to 0.5 in steps of 0.1, default 0.5). Both are in the tyre's slip-ratio units, the same ones the traction curve is: the curve peaks at 1.0 and is flat past the knee at 2.5, which is the top of the threshold's range. **The ceiling is the sum of the two**, so the shipped pair regulates to half a slip unit and floors the cut at exactly 1.0 — the traction peak. Raise the threshold to let the AI spin more before it intervenes; widen the span to make the cut more gradual. The extremes are the point rather than a side effect: at 0.1 + 0.1 the field drives more safely than anything the old law could express.
3. **Off-track halves it, and nothing else moves it** — the threshold is the setting on every surface and at every speed now: the surface-grip scaling is gone (it tightened the regulation where grip was scarce, tracking the lower peak slip a loose surface has, and it went for simplicity) and so is the launch ramp. The off-track half is the only modulation left, and the surface no longer reads into the value at all. Expect the cars to be a shade more permissive than before wherever grip is scarce and off the line — 0.5 everywhere, where the old law gave more slip at launch and less on dirt. A launch allowance, as a setting or a per-racer personality, is filed as `launch-slip-allowance`.
4. **ABS** — the same shape with no knobs: the cut still starts at the knee and floors at mingrip, which are the traction curve's own numbers.
5. **The menu and the file** — both items sit under TCS in Rival Traction, and their keys appear in `Menu-Settings.ini` on the first load after this build; `Log.log` must hold no "declares no key" line, and both values must survive a reload.
6. **The cut floors at 0.2 throttle** (driver-decided, no row of its own): a car held at the bottom of a TCS cut keeps a fifth of the throttle, not a quarter. It is the floor the cut lands on at the far end of the taper, so it shows on a long wheelspin exit rather than in the middle of the span — a car that now reads as bogged or slow to recover is this number, not the taper.

## The ceiling's reference is the outer wheel (driver-verified)

**Driven:** at 1.00 the tyres squeal through a corner; at 0.75 the squeal is mostly gone, which is the crossing this change was for — the cap sits just under the peak. The wheel-slip spheres agree, and put a number on it: with them on, the tyres read near full green — the top of the traction curve — at 1.00, and below the peak at 0.75. The tight-slow-corner check below was covered in the same drive.

The peak-slip ceiling is stated at the *outer* front wheel — the loaded wheel in a corner, whose grip matters most. The engine scales one front wheel to `TEMP_STEER_WHEEL_MULT` (`wheel.cpp:6445`, `CWheel::SetSteerAngle`) and leaves the other at the command, so the command is sized at `1 / 0.75 = 4/3` of the peak for the outer wheel to land on it (`PeakSlipOuterCommandShare`, `Racer.cs`); the inner, steered to the full command, runs past the peak by design. The axle midpoint was the reference before this (`2 / (1 + 0.75) = 8/7`, driven: 1.00 squeals, 0.75 mostly clean) and was reverted to the outer so the loaded wheel is the one at the peak; the outer reference owes its own drive.

1. **The size of the change.** The cap is a flat 14% tighter at every speed and every factor, so the new 1.00 sits between the old 1.00 and the old 0.75. Judge the drive against that bracketing, not against a setting that reproduces 488 exactly.
2. **The tight stuff.** The tightened cap bites hardest above the ramp's end speed, where the ramp no longer reaches: watch a slow corner for a car that cannot place itself, or that turns in more lazily than it did.
3. **The item and the file.** The description now names what 1.00 means; confirm it shows, the value survives a reload, and `Log.log` holds no "declares no key" line after a load.

## The measured-slip allowance under the ceiling (CLOSED — built, driven once, switched off; do not drive)

**Closed (driver)**: the allowance is a switch — `SlipPeakCeiling` is off and the update returns early — and the question it left open, what the ceiling is stated against, was settled at the outer wheel (`0dfbf53`), so nothing here is owed. The list below is what a future session resumes from if the switch is ever turned on.

The ceiling in force is now the lower of the model law and an allowance read from the tyre: the wheel may lead the side slip it already carries by `SteerSlipLeadShare`, and the allowance climbs to the whole peak over `SteerSlipClimbSeconds` while the command keeps asking for more than it grants, falling at the same rate once it stops. The climb is timed to the peak, so a lower Steer Limit At Speed cuts the plateau without changing when it arrives.

1. **The cyan walls are the instrument.** `ResolveSteerCeiling` draws them, so the ceiling in force is visible: it should widen over about a second from a turn-in off a straight, then hold. Snapping wide instantly means the climb is not scaling with the core-tick time; never widening means the demand test is not firing.
2. **The timing** — the first drive read the swerves as slower, which is the effect the law exists for, but the climb as a bit fast; judge whether a second is now too slow rather than too fast.
3. **The hunt** — the cap chases the ask, so when the ask sits near the cap the allowance can oscillate at the climb rate. The tell is the walls breathing on a straight, or a weave there. The fix is asymmetric rates: climb in a second, fall over two or three.
4. **A slide** — the allowance reaches the whole peak at the climb rate rather than instantly, so the model ceiling returns over that same second while the countersteer allowance and the damper bypass keep raising on top throughout.

## The outside approach is armed per approach, and a skipped approach is not asked for (dev build 535 — driver-verified)

**Driven:** the outside line resets per approach (`7619347`) — no locked-in outsides carried from a previous corner's exit — the skip itself (`1395c30`), which shows as no swing to the outer edge after an apex and no swing-out between the two corners of a chicane or S inside the 2–3.95 s band, and item 4 below, that braking still holds through the apex.

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

## The lane ask is scaled down as the car crosses (driver-verified — softer and still confident, the comparison included)

**Switchable (`b390029`):** `LaneCrossingScale` gates the multiply, so the unscaled ask can be driven beside the scaled one — shipped off for that comparison. The verdict below was reached with the scale on and no lane inertia upstream of it, so the two have never been compared in the order they now sit, and that drive is for the comparison itself rather than a decision. **First read with it off (`b390029`):** turn-in reads more decisive without too much aggression — but that is the approach side, while the scale's own case is a car crossing hard toward a lane it would reach anyway, and this read did not single that out.

**Driven:** the reads are softer than either rejected shape — the hard cap on the wheel and the step on the crossing — while the cars still steer with confidence, which is the scale doing its job with the allowance left where it was. Since the close-out: the comparison and item 3 were driven and accepted; the fleet-wide read, taken on a high-grip car here, is the `apex-radius` item the ladder still owes.

The lane steer is pure pursuit on the bearing to its aim point, and the first answer to an over-large ask was a ceiling on the wheel angle — the wrong quantity and the wrong shape. What is bounded now is the **crossing angle**: the car's heading against the track direction at the car. `LaneCrossingLateralSpeedMps` sets the crossing that costs half the ask, and because it is defined as a sideways rate it follows `1/speed`; `LaneCrossingShare` (`Racer.cs`) is `allowance / (allowance + crossing)`, a scale that fades with the crossing and never reaches nothing. Two shapes were tried and rejected on the way here — a hard cap on the wheel, and a step on the crossing — which is why this one is soft by construction and has no width to tune. **Corner entry is exempt, and only the entry** — from the braking target to the apex, because turn-in is a crossing on purpose: the car has to point into the turn to rotate, so a bound on the crossing fights turn-in whatever its shape or schedule. Everything before the braking target is still lane work and is scaled, which is the point: the first cut of this gate keyed on the corner owning the lane at all, and the positioning window opens seconds early, so on a twisty track it was off for most of a lap.

1. **Mid-straight, a car well off its lane** — the case it exists for. The car should stop angling itself across its own direction to reach a lane it would reach anyway: the yellow pursuit line in Show Track Analysis should keep nearly all its authority until the crossing approaches the allowance and then fade — half at the allowance, a third at twice it — rather than switching off.
2. **The unwind must never be scaled** — the deliberate asymmetry, a sign test rather than a fade. A car already crossed and steering back keeps its whole ask; if it struggles to straighten, that test is the first thing to check.
3. **The cost** — at speed the same term delivers the corner's inside line, the outside approach and the avoid-ahead pass, so lane moves slow once the crossing passes the threshold. A car reaching its line late into a corner, or a pass that no longer completes before it, is this one. `LaneCrossingLateralSpeedMps` is the knob, and scaling speed up or down in the threshold is the intended fiddle.
4. **Fast corners** — a car at the limit genuinely carries a few degrees of crossing, so the scale bites there too; the tell is a car that cannot hold its inside line through a quick bend. **The fade is what to judge here**: there is no width to tune and no cliff, so chatter around the allowance is not expected — if the cars still cross too hard, the allowance is the knob rather than a new shape.
5. **Low speed must be unchanged** — the threshold grows as speed falls, so placing a crawling car, an off-line start or a recovery must still turn as it did.

## Apex speed reads the smoothed radius (driven on high-grip cars: faster and more consistent — the fleet read is owed, non-blocking: one car under 1.5 G, one over 2.5 G)

Corner speed is `√(grip · g · R)` and `R` was `SupposedRadius`, the region's **minimum precise** radius — a short-chord measurement, so a minimum over many samples sat on the low tail of the noise and held every corner speed down. The speed path now reads `DetectedRadius`, the smoothed region radius the rest of the system already used: the tight flag (`Racer.cs:2986`), the entrance and exit spans and the merge survivor (`TrackLoader.cs:698-699`, `:737`). Six sites moved: `CornerApexSpeed` (`AutosportRacingSystem.cs:2779`), the outside-line entry gate (`Racer.cs:833`), the braking-requirement flag's intended speed (`Racer.cs:2989`) and the three held-apex radii (`Racer.cs:3039`, `:3055`, `:3069`).

1. **Cars brake later everywhere.** This raises every corner speed, so the first thing to expect is later, deeper braking into slow corners. The steering pipeline did not change at all; a car that turns in lazily *at the same speed* is not this.
2. **The failure to watch is over-confidence.** A smoothed radius is also a larger one, so a car that was conservative now carries more speed to the apex — the tell is a car washing wide or sliding on entry in a tight corner, especially one whose apex sits on a noise spike. The tight stuff is where this holds or does not.
3. **Judge it against the offsets at 0.** The offsets ship at **+2 mph** (corner) and **+4 mph** (route) (`AutosportRacingSystem.cs:153`) and the plan takes the lower of the two terms (`Racer.cs:1629`), so the pair has been masking this bias. At 0 the plan's own number is on show; at the shipped values the fudge absorbs part of the change.
4. **The radius pair is no longer visible in game** — the circle label that printed `R plan/detected` is gone, so the per-corner gap cannot be read off the screen and behaviour is the evidence.

## Contact avoidance is its own term, added after the damper and the blend (re-grounded, driver-verified)

The steer-away correction used to be folded into the lane steer, so the lane system's own stages acted on it: the yaw damper modulated it, the slide handover replaced the command with a countersteer target that deliberately omits the lane term, and the on-aim rule zeroed the wheel. It is now its own `contactSteerDeg`, added after all three; only the steer limiter and the final slew bound it.

**The magnitude is the pair's relative course angle, scaled by the daylight between the two sides.** The gap is measured between the surfaces rather than between the centres — the lateral separation minus the two half-widths — so zero means the sides are touching, which is what the ramp's full end has to mean. The ramp gives nothing at five metres of daylight and the whole angle at touching, so half the angle at two and a half metres. One constant owns both that outer end and the lateral gate, and they have to move together or the ramp's tail is clipped into a step. The angle is taken on horizontal velocities rather than the published three-dimensional one, so a rival lifted on a crest beside the car reads nothing, and the term is silent below walking pace, where a car's own direction is noise.

**The gate is load-bearing for the sign, not only for restraint.** A rival is skipped unless its lateral offset and its lateral closing rate in this car's frame carry **opposite signs** — the term is armed only while the pair closes sideways. With that gate on, the course angle's sign and the side the rival is on are equal by construction, which is why the angle can be used directly as a steer-away. A separating pair inverts that relation, so without the gate the term would push toward the rival while the slide blend is recovering away from it. The closure is *relative*, so this car's own sideways movement arms it as much as the rival's.

**The sibling term does not share that gate.** The side-by-side heading match tests the rival's *own* lateral velocity toward this car and not the relative one, which its comment states deliberately — this car's own closure is the rival walls' job — so it can stay armed while the pair separates under this car's movement, and at zero lateral separation it stays on by design where the contact term goes quiet. The two are prevention and reaction: the match stops a convergence developing, this term answers when it did not. Both read only the three rivals in `Brain.Rivals` (`DataStructures.cs:115`), so a fourth car alongside is invisible to either.

**Nothing here reacts to proximity.** A stationary rival alongside gives nothing, at any distance, under this law as under the last one: the term answers relative movement.

**The read-out, and what it cannot isolate.** The push rides the final command after the on-aim zeroing, so it appears as the debug steer fan's white line — the command after the limiter and the slew — diverging from the red one, which is pursuit plus damper. That divergence is *not* the push alone: the side-by-side heading match enters the command but never the red line, the slide blend substitutes its own target while a slide is on and dominates the difference, and the limiter and the slew act on the sum. So the fan reads this term cleanly only with no slide and no match present, and on a slide it cannot be attributed at all. Nothing logs it, and there is no debug draw of the term itself — which is the change that would make it legible.

1. **A parallel pass gives nothing** — same course, any daylight: the angle is zero, so a deliberate side-by-side pass with no relative course must produce no divergence *beyond* the match and the limiter, which is why the check needs a pair with the match quiet and no slide.
2. **A separating pair gives nothing** — the gate, and the case that would invert the sign if it were dropped.
3. **A closing pair ramps** — nothing at about five metres of daylight, half the angle at two and a half, the whole clipped angle in a genuine rub. Read the behaviour rather than the fan here: a degree of command at racing speed asks for more curvature than the tyres hold, so the small end is not negligible to the car even where it is hard to see. At 60 m/s a one metre per second squeeze is about 0.95° at contact and 0.48° at the half point, roughly a fifth and a tenth of the grant; five metres per second reaches the grant at contact; and below 20 m/s the same one metre per second is 2.9°. Too strong is a clear step away while the pair is *not* closing sideways, or beyond the half point, and a twitch on a straight is the classic tell; too weak is a real side rub with nothing felt at all. The shapes are drawn in `docs/avoidance-push.png`, produced by `docs/avoidance-push.py`.
4. **No reaction to a rival in the air** — a car lifted on a crest beside another must give nothing, which is what the horizontal angle buys.
5. **A three-wide moment** — the sum is still uncapped per rival, so two or three conflicted rivals saturate the few-degree grant again; the finding should say whether the per-rival proportionality is enough on its own. Bounding it is a separate filed item, not this change.

## Every racer brakes from its own confidence (NOT DRIVEN)

The factor each corner starts from was one per-corner draw around 0.75, so a fresh grid differed corner by corner but every racer averaged the same centre. It is two draws now: `_brakeConfidenceBias`, taken once in the constructor (`Racer.cs:362`) and added to every corner that racer seeds, plus the per-corner draw unchanged (`SeededBrakeFactor` `Racer.cs:275`). Both are ±0.05, so a racer starts anywhere in 0.65–0.85 with a centre of its own, and the bias is the half that moves a lap because it repeats across every corner while the per-corner draw averages out.

1. **The draw is in the log, so this needs no drive.** Starting a race must write one `Brake confidence bias <car>: <value>` line per racer to `Log.log`, spread across ±0.05 rather than all reading the same number. A grid of identical lines means the draw is per corner after all, or the cars never reach the constructor.
2. **Brake Learning off stays uniform.** With the toggle off `BrakeFactorForApex` returns the flat seed and no corner context is ever seeded, so the field must brake identically again — that is what the option promises, and it is the baseline for judging the on state.
3. **The opening lap is the only place to see it.** The first commit moves a corner's factor up to ±0.2 against a ±0.05 seed, so the bias is a lap-one difference and the field converges after that; a spread that has gone by lap two is the design working, not the draw failing.
4. **The feel** — with one centre for the field, the pack used to brake as a single car into the first corner; each racer should now be slightly earlier or later. A car *visibly* overshooting corners is the ±0.05 bias being too much rather than a fault in the draw, and `BrakeConfidenceJitterHundredths` (`Racer.cs:206`) is the knob.

## The lane mover — the aim is carried onto the request (driver-observed as right, the aim's interval included)

The override chain still picks the lane, but its answer is now a request (`_laneRequest`) and the carried lane (`_carriedLane`) is carried onto it by `UpdateLaneMover` at 14 m/s^2 with a 12 m/s cap, so a requirement that is valid for half a second moves the lane a fraction of the way instead of stepping it. The request's own rate feeds forward, so a line that moves smoothly is still tracked without the lag a pure arrival law would leave; the rival walls bound the request alone; the green and the recovery snap re-seed it; and a core tick arriving after a stall is bounded at 100 ms. **Driven: the lanes read right — smooth, and the flapping this exists for is gone.**

**The aim may not chase a lag of its own making.** With no source asking for a place the request *is* the car's own lane, so a lagged aim stops being a target and becomes something moving with the car: the steering turns into a delayed function of the car's own motion and the two chase each other. The aim is therefore clamped into the interval between the car's own lane and the request, after the integration. With the own lane requested the interval is a point on the car, so the aim tracks it exactly and the error is zero; with the car centred while the aim sits on the far side, the aim lands on the car and then travels out with its normal inertia; and where the aim already trails between the car and the request nothing changes at all, which is the case that was working. The clamp takes the car's lane from the call site rather than the published centre deviation, because those are two different frames — the steer reference sits a lookahead ahead — and it zeroes the mover's rate when it moves the aim, or the rate winds up to the car's own lateral speed and carries the aim the wrong way on the first tick a real request arrives.

**Read-out (`75038ab`).** The own-lane wobble is gone, which is the case the interval exists for: the car no longer chases a lag the mover gave it. Nothing else was retuned, so the ordinary lag is unchanged — the interval only speaks when the aim is outside it.

1. **The numbers are settled for one geometry.** `B` is the safe bound (half the track width less half the car) and the line's own motion scales with it, so the settling case of a 5 m half-width does not cover a wider track, whose line moves further and would want a little more acceleration. `LaneMoverAccelMps2` is the knob, and the cap has to stay above the line's own peak rate or the line itself is clipped.
2. **If it is retuned, the log is the instrument** — `Lane mover <car>: asked …, aimed …, rate …`, at most once a second per car while the lane is actually moving.
3. **What the mover deliberately does not touch** — contact avoidance stays its own steer term added after the damper, while the recovery's nearest-edge lane goes through the mover like every other source, so a turn-back that reads late is the case to watch for if one is ever seen.
4. **The own-lane wobble is gone** — the tell is the yaw and steer wobble that used to persist while the request was the car's own lane. The log is silent in exactly that case, because its line is gated on a rate above 0.2 and the clamp zeroes the rate, so the read-out is the debug aim line sitting steady instead of wandering. And the debug red line is the aim in the *steer-reference* frame while the green line is the car's offset at the current node: different frames, so red will not sit exactly on green in a corner even when the aim is exactly the car's lane.
5. **The cross-side case** — from an aim on the far side, it must land on the car and then travel out to the request under the normal inertia. The log prints while it travels, so this case is legible there.
6. **The ordinary lag survives** — a request beyond the car with the aim between them must still trail it, with the log showing asked greater than aimed and a rate below the cap. Falsifiers: an aim ever outside the interval between the car and the request, or an aim reaching the request instantly while it is being driven *outward*.
7. **The one place to watch** — the lean reports on a 0.1 m threshold, so its opinion can come and go around the car's lane and step the request by that much, and the aim follows it: about 0.4 degrees of bearing at a 15 m lookahead, self-limiting as the aim re-enters the interval. No deadband unless a drive sees it.
8. **The walls no longer touch the aim (driver-observed: the avoidance reads smoother than the placement it replaced).** A corridor that closes now steps the *ask*, and the aim is carried onto it at the mover's rate, so a wall-driven jump of the solid line is the fault and a grey `WallClamp` tag with the solid line walking after it is the change working. The deliberate loss is that the aim is no longer bounded to the corridor: while the car itself is outside one that has just closed, the aim sits between the car's own lane and the clamped ask, so the line may read outside the corridor until the car arrives. The other instant path is untouched and is not a wall — the interval clamp places the aim whenever the ask lands on the far side of it, so check the ask number in the label before blaming a corridor.

## The inside line is a pull, not the edge (driver-verified — reads right)

**Driven:** the lean reads right. The checks below stay as the record of what the shape is judged on if it is ever doubted.

The lean no longer answers the full inside edge on a bend that passes its 500 m radius gate. It answers the car's own lane plus X metres toward the inside, recomputed every tick, with X one metre times a linear fade from 500 m down to 250 m of radius — nothing at the gate, half a metre at 375, the full metre at 250 and held below. Because the target is relative it recedes as the car moves, so the crossing stays alive for as long as the bend lasts and the drivable bound is what ends it; the lane mover stays the only rate bound.

**Rejected on the way here**: a proportional *destination*, where the car aims X inside and stops, which leaves a long bend short of the inside; and a speed-driven allowance, because speed is the result of the curvature — it misreads a straight as a bend and the slowest corner as the one needing least line.

1. **Judge it where it is visible, not at the gate.** X below the aligned-wheel deadband steers nothing, so the answer first appears near 465 m at the lower lookahead floor and near 370 m once the lookahead caps; a no-op read at 500 m is the deadband, not the change failing.
2. **The corner line still wins.** The pull is replaced wherever the corner line answers, so only bends the corner detector has not claimed show it, and judging it on a registered corner is the likeliest misattribution.
3. **The ask and the crossing are different readings.** On a tight bend at the limit the limiter clips the extra ask, so "a tight bend pulls harder" can be true of the intended lane and false of the realised crossing. Read the intended lane against the live deviation, not the crossing alone.
4. **A bend long enough to traverse ends at the edge and holds it.** The traverse is computed from the observed crossing rate rather than a stopwatch, because X sets a rate and not a position; stopping short on a long bend, or crossing past the edge, is the falsifier.
5. **No step, and no self-excitation.** No aim step when the chord node's radius jumps or across the gate. The case this used to warn about — the request anchored on the car's own lane letting the aim and the crossing trade through the mover's feedforward — is removed by the mover's own interval clamp, so what is left to watch is the lean's transient: its request is the car plus up to a metre, and the clamp binds there only when the car moves inward faster than the aim follows, clearing within a tick or two because the request's rate feeds forward.
6. **The magnitude does not move with speed.** Only the deadband should make the onset differ between a slow and a fast pass; a speed-dependent X is a falsifier.
7. **On exit it holds.** When the bend ends the source reports nothing, the request returns to the car's own lane, and the car holds the position it reached rather than unwinding.

## The lane carries the reason it changed (driver-verified)

The lane was a per-tick decision with no record of why it moved, which made every handoff an inference. Whoever writes the request now writes a reason beside it — own lane, curve lean, outside line, apex line, commit card, avoidance, recovery edge, wall clamp, debug lock — and that tag also replaces the boolean the writers kept, so the flag a consumer reads is that reason together with the aim clause it already had (the request-and-aim split is in `DESIGN.md`). The corner line reports its two phases separately: the outer hold is the profile's far clamp, and everything below it is the blend toward the apex.

1. **The log names it** — `Lane mover <car>: asked …, (reason), aimed …, rate …`, at most once a second per car while the lane is moving; a *static* reason is never logged, so the always-on read-out is the lane line.
2. **The lane line carries it** — both ribbons take the reason colour: the ask dashes and dims at the request's own offset, the solid line sits at the aim, own lane keeping red and black still meaning zero, and the near end of the line names the reason for the focus racer with `ask` and `aim` beside it in metres and the mover's rate in m/s. The colour is the secondary channel: where two reasons can alternate within a second — the corner's two phases, a card ending mid-corner, the blend's zero crossing — the name is what settles it. Wall clamp and the debug pin are grey on purpose: a bound is not an opinion about where to go.
3. **One behaviour moved, and it is the flag.** The wall clamp is now a writer where it was not, so a car whose lane sits outside the drivable edge arms the aimed flag from the first tick rather than once the mover has brought the aim in. The cap that reads that flag then acts only if its own gates agree — speed above its floor and the projection on the turn side — so this arms the policing rather than firing it, and it is the state the cap exists for. It is on this list because it is a change and not a refactor.
4. **The reason describes the request, not the aim.** The mover carries the aim onto the request at a bounded rate, so during that lag the solid line sits short of the dashed ask and the two numbers at the near end separate. The tag rides both ribbons and names the ask, which is why the property is named for the request. A dashed ask that changes side while the solid line swings after it is the chain changing its mind, not the mover misbehaving, and the reverse — the ask steady while the solid line wanders — is the mover's own work; that is the reading the second ribbon exists to separate.
5. **A zero-valued opinion reads as silence.** The profile passes exactly through zero at the midpoint of its blend and the chain treats a zero answer as no answer, so the tag can flicker for a tick there. It predates the tag, which only makes it visible.

**First read-out.** The commit card's lane was seen with no rival in view, which is the card working rather than the corner line drifting: a defend-lane card plays only in the one-to-three-second window before a corner entrance, against a rival that is already behind the car and closing on that entrance, so its target is out of sight by construction. The line it writes sits on the inside of the corner, one rival width plus a margin off that rival's own lane, clamped by the same bound as every other lane target. Worth knowing on the next read too: the lane line is drawn for the debug focus racer, which is never the player, so a committed line can belong to a different car than the one being watched.

## The divebomb claims the last band before the brakes (compile-verified, drive owed)

The card armed up to five seconds before a corner entrance — hundreds of metres at speed — on a straight-axis contact time that said nothing about where the two cars would actually meet, and once armed it held the committed lane and its brake bonus until the apex or an eight-second backstop. On a car whose rival had already gone, that reads as a lone car leaning semi-inside, far from the corner, which is how it was found with the reason tag.

1. **The window is the braking point.** The card arms when the entrance is within the plan's own braking time for that apex plus a small lead, instead of a fixed five seconds, so the commitment is the approach it is about to brake into rather than a slice of the straight.
2. **The reach is an arrival band.** A partner's own time to that entrance must be within a band of the car's, so a slower car just ahead counts — which is the dive worth taking — and a car the car would merely catch on the straight does not; the target must also be on this piece of the route, which the published route gap answers.
3. **The card releases with the partner.** It now ends when the target is gone, has left the perceived set, is off this piece of the route, or is travelling another way — not only at the armed apex. The commit lane already dropped silently when the target left the perceived set, because that lookup is by reference; the brake effects did not, and they bias the next three apexes, which is why the release matters more than the lane does.
4. **What the drive must separate.** "Still too early" is a CommitCard with no partner in the band, or with the car still gaining speed more than about twice the braking distance out; "now too strict" is a partner held through the braking zone with no CommitCard at all, counted over several cars and laps because that failure is silent. The card also needs full throttle and at most one rival wall, so the armable interval is narrow by construction: if it stops firing, the lead is the knob.
5. **The brake side is a second read the tag cannot show.** The dive's brake effects are not scoped to its own apex, so they reach the next three braking targets; the observable is the *following* braking zones after a release, where the car must brake as its own plan says rather than later.
6. **Flagged rather than folded in.** The commit branch answers for whichever corner is current without checking that it is the armed one, so a tick where the corner has moved on can compute the lane for the next corner; and scoping the brake effects to the armed apex would remove the class of item 5 outright. Both are their own change, and neither is built here.

**First read-out (`3656c03`).** Fewer false arming events than the five-second window produced, read from the cars in a race. That settles the "too early" half of item 4 and leaves the other half open: nothing yet says the card still fires when a genuine partner is held through the braking zone, and that failure is the silent one, so it needs counting over cars and laps rather than judging by eye.

## The outward lane move is bounded in position, not scaled in gain (driver-verified — the outside lunge is gone)

The lane steer is one pure-pursuit bearing, so it carries the lane error and the road's own geometry together, and the ask is a small angle per metre but a large lateral acceleration per metre: a 2.7 m wheelbase on a half-second preview asks 8 m/s^2 for every metre of lane error between about 58 and 216 km/h, buying less below that where the 8 m floor binds and more above it where the 30 m preview cap does. No lateral-g figure for the tyres is measured anywhere in the tree — the limiter's ceiling is a slip-angle proxy (`ResolveSteerCeiling`) and the grip is handling.meta authored — so the ceiling an ask runs into is the limiter, not a measured grip.

What is bounded is the move and not the ask. Where the lane was asked for by the **outside line** or by **avoidance** (`OutwardLeadBounded`, `Racer.cs`), the aim may not lead the car's own lane by more than `LaneOutwardLeadMeters`, 1.5 m: one clamp in the mover beside the interval clamp, zeroing the mover's rate in the same branch or the rate winds up against a position the bound refuses to move. It is measured from the car's own lane, so it travels out with the car and no lane change is cut short — only its bite. The gate is the reason the lane was asked for rather than a comparison of the two lanes, because that comparison clamps an off-track car's return, which needs the mover's whole reach; the recovery edge, a commit card's lane, a wall clamp, the apex crossing and the debug pin are exempt, the first three because they are not preferences and the crossing because it is lane work the corner owns. The re-seed (`_laneSnap`) bypasses every bound, as it already bypassed the interval clamp and the walls, so the green is one tick at the full request.

The share that scaled the lane error's own degrees is reverted (`73756c7`; it lived at `9decee7` as a half and `a555ea9` as three quarters). Its own bracket is why the bound replaces it: the unscaled ask rings onto the outside line, a half is smooth but lazy — the car trails the aim, so a lane change and the entry land late — and the ring is that share's own term, so scaling the gain could not tell an outside lunge from a turn-in. Recover it from either commit if a ring shows up where the bound does not reach.

1. **The falsifier is a late or short outer edge**: the car reaches the outside line after the braking target instead of before it, or holds a steady offset short of it. That says the bound is the cause and 1.5 m is too small rather than the share.
2. **The second tell is free and visual**: the red aim line pinned at the same gap from the green car line while the car is still moving outward. The mover's own log line is silent while the bound bites, because both bounded branches zero the rate before the line's own `Math.Abs(_laneRate) > 0.2` predicate, so the line is this bound's read-out and the log is not. If a ring survives *below* that gap, where the law is linear again, the fault is damping — the next lever is the damper or the preview length, not this number.
3. **What the exempt list is judged on**: an off-track recovery must still cross back at the mover's whole rate and a commit card's lane must still be taken without the bound; a slow one of those means a reason tag is missing rather than the bound being wrong.
4. **The avoidance pass is bounded on purpose**, since it is the other outside move reported, so a pass that now reads timid is the trade being felt and the number is the knob.
5. **The lane error is unscaled again**, so the centring is the responsive one the driver first judged aggressive and oscillating; the bound only truncates the error at which that can bite.
6. **Coverage is what separates two clean drives**: only the corner line's far clamp (`OutsideLine`, `ComputeCornerLane`) and an avoidance pass carry the two bounded reasons, so a car prepositioning under `ApexLine` — already inside the blend window — or on a straight with no lean (`OwnLane`) is untouched, and a drive with no lunge is consistent with the bound working *and* with it never engaging. The debug line's colour and the name at its near end are the instrument for which one it was.

**Read-out (`73756c7`).** Driven, and the outside lunge is gone, which was the whole complaint. The shape it reads on the line: the aim runs out to 1.5 m from the car's own lane at the mover's own arrival rate, about 6.5 m/s at a metre and a half left, stops there, and thereafter moves only as fast as the car does — because the bound is re-anchored to the car's lane every tick, so it advances exactly when the car approaches. That stop is a kink in the aim's velocity and not in the ask: the gap the pursuit reads sits at a steady 1.5 m, so the ask is a steady 12 m/s² rather than a stop-go command. The smoother shape — bounding the target the mover chases instead of clamping the aim — was offered and not taken, and it is the first lever if the hard stop is ever wanted gone. Item 6's coverage question is answered in the direction that matters: the bound did engage on the outside preposition, which is what the hard stop at 1.5 m shows.

**Reshape — the hard stop wanted gone (build 703, driver-verified).** The driver wants the stop out, so the cap moves to 3 m and the taper opens at 1 m: below 1 m of lead the move is untouched, and across the band each outward tick is scaled by `tanh(remaining lead)` normalised over the band, so the glide leaves 1 m at full speed and approaches 3 m asymptotically instead of striking a wall and freezing. Driven: the lane switch reads smoother, no more start-and-stop. The clamps from the first bound stay as the failsafe, so the cap the glide approaches is still a real bound — the lead is measured from the car's own lane, and the exempt reasons and the gate are unchanged. Judge the glide with the same coverage rule item 6 states: only `OutsideLine` and `Avoidance` carries it, and a drive with no bite is consistent with the taper working *and* with it never engaging.

## The debug visual batch — readout dropped, wheel slip ungated, the Gs projection drawn (dev build 523+, compile-verified, drive owed)

1. **The lane lines' world-space text readout is gone** (`45ccbd4`) — the ribbons still draw and their colours still name the lane reason; nothing but the tag with the ask/aim/rate numbers should disappear. It sat at the near end of the focus racer's 40-node lane lines under Show Track Analysis.
2. **Show Wheel Slip is standalone** (`6ae4db9`) — the toggle joins the debug-focus scan gate, so toggling it alone must draw the spheres with no fan, no lane lines and nothing else appearing. Before this it drew only when Show Inputs or Show Track Analysis was also on, because those were the only triggers that selected a focus racer.
3. **Show Gs Projection is the new drawing** (`55b800d`) — toggled alone it draws, from the roof of the closest AI car (the fan's full-model-height convention), one white line to where the car projects at 0.5 s and one cyan line connected from that joint to the 1 s projection, both lifted to roof height. The bend at the joint is the acceleration the car is committing to; on a straight the path reads as one straight ray. Default off, and the rename starts a fresh persisted key because toggle persistence is keyed by the enum name's spelling — the old `GsAwarePreview` value dies at its next repair pass.
4. **The gs-aware lead is always on, and that is invisible if you never turned the toggle off** (`55b800d`) — the only behaviour change is for a saved copy that had `Gs-Aware Preview` off, which now leads the aim again. Anyone driving on the shipped default should read no difference from before, which is the regression to look for: nothing at all.

## The throttle kick card — rotation on an over-the-limiter ask (compile-verified, drive owed)

The card, by the drive's own shaping (KISS, no band): **arm** = AI-only, signed forward roll, RWD (bias ≤ 0.01, live `0x48`), cooldown clear, no slide (the blend's own onset threshold), the ask handed to the limiter last core tick sitting past the ceiling it enforced, and **the gear gate**: the burst's computed thrust per rear wheel — the drive-force percent read at handling `0x60` (the count verified from the `0x48` bias to the `0x80` lock) times the current gear's ratio from the leak's own constants (first `1/0.3`, top `0.9`, the middles on the `1.1` progression, so the unread close-ratio and low-speed-torque flags shift the whole ladder on flagged cars) over the two drive wheels — beating the live grip times the static wheel share by the margin. The unread flags make flagged cars read green on multiplied thrust; the fold's slip is their verdict. **Play** = the plan's throttle floored to full for up to 1.25 s (the floor raises the base before the caps' argmin; the tag reads `Kick` unless a cap binds tighter), TCS pinned to 1 at the field for the window, the slide blend and `IsFullCountersteer` standing down for the rotation sharing the arm-side sign. **Fold** = the window cap, or a body slide past 30° under the burst (`rotated` — the delivery bound, either side, the machinery resumes to catch whatever the burst left); every play and fold is a forced log line with its duration, ask and slip.

The evidence so far, from four shaped versions driven: the burst itself is proven — every play across two laps broke the drive axle (slips to the traction peak and past, one to the read's floor in 146 ms); the no-gear-gate version fired fourteen plays on the offroad track, all full-window, most pushing the wheels past the peak; and the arm's reading has been the shaped thing: the *delivered* command armed on the countersteer's replacement (85° of a 30° car while the pursuit asked 21°), the un-gated composite then showed the counter side of that door (a 165° ask on a body-straight, wheels-spinning state), and the *pursuit's* domain with no band armed on the corner-entry catch-up state (sixteen plays, every fold `limiter replaced`). This version keeps the composite read but adds the thrust verdict and the slide bound; what it hopes to prove: the gear gate prunes the no-rotation plays, the 1.25 s window makes the rotation legible, and the 30° fold hands a delivered rotation to the machinery before it becomes a spin.

1. **Count and class the plays on a lap.** The log's play lines carry ask/granted/speed/slip — the entry-band class (20-60 mph, granted 5-10°) will dominate with the wide-open arm by decision; the read that matters is the outcome, not the count: a car that exits its entry better or worse than the unbuilt would.
2. **The gear gate's first test**: the zero-thrust class (the old `no wheelspin`'s cases — the offroad lap's `slip 0.01` fold) should now arm only where the computed thrust beats the budget; a car shown kicking with a shallow fold (`slip > −0.3` at 1.25 s) is the gate over-permissive for its flagged ratio — its model's meta is then the thing to check.
3. **The 30° bound**: a kick that reaches a 30° body slide must fold on `rotated` inside the window and hand to the machinery without a crash or a spin-out — this is the overshoot's stop, tuned by the constant.
4. **The TCS signature** stays: the cap pinned to 1 through the play, returning cutting-fast at the fold.

## The blend refuses an outward ask (driver-observed: the entry reads better)

Below the corner line's far clamp — the `ApexLine` phase — the demand is refused when it lies outward of the car's own lane, so a car already inside the line is never sent back out to meet it and an entry cannot read as out and then in. The profile itself is untouched and still absolute; what is dropped is its outward half once the blend has begun (`ComputeCornerLane`, `Racer.cs`). Three exemptions are deliberate: the hold at the far clamp (`OutsideLine`), because reaching the outer edge is what the outside approach is for and the driver-verified 1.5 m bound above lives on it; every phase that is not the blend; and a car off the road, whose way back to the track is this line. The comparison is against the car's own lane — `CurrentNodeOffset`, the green debug line — and not the mover's `steerRefOffset`, which reads inward by the bend's sagitta (about 2.25 m at 30 m/s and 6.25 m at 50 m/s on a 50 m radius) and would refuse the line outright on the fast tight corners it exists for. One imprecision is inherent and left in: the demand is computed at the steer reference's station while `own` is the car's, so on a tight corner the two differ by the profile's own change over the lookahead — the reason the top-edge refusals below carry gaps of centimetres.

1. **The refusal is in the log**, at most once a second per car while it bites, and never as a tag: `Corner line refused <car>: demand X, own Y, Zm to apex`. A tag was rejected because a refusal leaves the request where the chain had it, and `_laneOverride` arms on the tag — tagging it would arm the offshoot pedal cap for a car that is holding its own line.
2. **On the line it reads as the dashed ask sitting on the green own-lane line** while the car is inside the demand, with `ApexLine` still appearing and stepping the ask inward once the sweep crosses the car.
3. **Falsifier**: a car that stays inside on an entry it had room and time to open up, or an armed approach where `ApexLine` never appears at all. The first says the refusal is too aggressive, the second that it is refusing the whole blend.
4. **A confound to separate on the drive**: this and the `outside-approach-window` skip gate produce the same tell, no outside line on a corner with room for one. The refusal logs; the skip gate does not.

## The TCS pair stated as a share of the peak slip, and the ABS knee on that peak (compile-verified — drives PARKED on a lower-grip handling)

**Parked before the drive**, by the driver's call: neither the TCS percent extremes nor the brake re-scale can be judged on the grip the fleet runs today, because the AI neither spins nor locks on it. Both resume when a lower-grip handling is in use, and the readings below are what that drive picks up.

The slip axis was settled by a live probe (`b889222`, evidence on the ladder item `slip-axis-scale`): `wheel+0x174` is the raw signed slip ratio, so the traction curve peaks at 0.4 in the field and goes flat at 1.0 — every threshold ARS compared against that curve was 2.5× too high. Both TCS knobs are now shares of the peak (`TcsSlipLimitPct`, `TcsTaperSpanPct`: 10 to 200 in tens, default 80 and 80), and the ABS knee moved from 2.5 to the peak, its floor a span later at twice the peak.

1. **The file converts itself once.** A `Menu-Settings.ini` holding the old raw pair (0.4 / 0.4 on this install) must come back as 100 / 100 with `TcsSlipThreshold` and `TcsTaperSpan` gone; 0.5 / 0.5 comes back as 120 / 120, because a tie rounds to the lower, tighter offer. `Log.log` must hold no "declares no key" line, and both new values must survive a reload.
2. **The menu reads in shares.** `TCS Wheelspin Limit (%)` and `TCS Cut Softness (%)` under Rival Traction, and both descriptions name the peak as their reference. The ceiling is the limit plus the softness, so 80 with 80 bottoms the cut at 160% of the peak.
3. **The extremes are the point of the range**: 10 with 10 should read as a field slower and safer than anything the old law could express, and 200 with 200 as traction control that barely intervenes, its floor landing at four times the peak — past the point the curve goes flat, where the tyres are simply spinning.
4. **The ABS re-scale owes a setup rather than a drive** — a car with no ABS of its own on a low-grip surface, so the wheels actually lock, and the instrument the section above names. What to watch: the cap trimming the brake from the peak slip onward, with full authority before the wheel is locked. The falsifier is a car that can no longer brake to the threshold, which would say the taper is too tight for the plan. On a car whose own ABS is active the engine forces the field to ±5, so its cap sits at the brake floor either way and the re-scale cannot be judged there.
5. **What the comparison already says, before the drive** (`docs\abs-comparison.png`, script beside it): vanilla's own standard ABS holds the modelled brake slip at 0.6 — 1.5× the peak — with the pedal untouched, and this fleet's traction curve is shallow (max 1.50 against min 1.30), so 97% of peak force is still there at that clamp and 95% at the slip where our pedal floor lands. Ours therefore removes three quarters of the brake where the tyre has lost a twentieth of its force, and the case for the cap is the wheel staying rolling and the plan's balance rather than longitudinal force. The vanilla alternative is the crude one — nothing until the wheel is already locked, then a 62.5 ms full release once the slip passes 1.5 — and no fleet car carries that flag pair.

## The offroad gravity flag has one home again (dev build 613, compile-verified, drive owed)

The flag was applied twice: `Initialize` multiplied gravity by the engine's own ratio and `UpdatePerceivedGrip` baked the same ratio into the base grip, so the speed laws took the flag 1.44× in v² while the two steering laws — hard-coding `9.8f` — took only the grip half. The bake is gone, so grip is the coefficient again, and `AckermannCeilingDegrees` and `SteerLimitedSpeed` now read `Handling.Gravity` like every other grip law. This follows the driver's rule: a quantity about actual grip takes the car's own gravity, while a theoretical or reporting one (`ComputePaceIndex`, the curvature-to-delta-G conversion) keeps the assumed 9.8.

1. **This moves nearly the whole grid, not a subset.** 39 of the 41 fleet cars carry the ×1.2 off-road gravity bit, so expect a fleet-wide change: cornering speed down by about 9.5% (the grip term loses the baked 1.2 while gravity keeps its own) and grip-limited braking down with it. A car that does not carry the bit must be unchanged, and that is the control.
2. **The two laws must now agree with each other.** The steer-limited speed and the cornering speed were reading different gravity, so the plan asked for an entry speed its own wheel could not hold; after this they share one gravity, and a corner entry should read as the wheel's limit rather than above it.
3. **The precedence trap is untouched and still has nothing to bite.** `HF_OFFROAD_ABILITIES` (the ×1.1 tier) is still untested and no fleet car carries both tiers, which is the only case where ARS's ×1.2 overstates the engine's ×1.1.

## The overspeed correction is a rate ladder (dev build 616, compile-verified, drive owed)

The checkbox became a list: `Off`, then `0.1` to `1.0` in 0.1 steps, default `0.2`. The number is the throttle lifted per 0.1 G of surplus over the fixed 0.2 G start threshold, so the cut completes at `0.2 + 0.1 / value` — `0.1` reproduces the shape driven at build 612, `0.5` completes at 0.4 G, `1.0` at 0.30 G. The arm gate, the detector, its ~3 Hz sample and 0.1 G deadband, and the glide rates are all unchanged; a stored `OverspeedEnabled` False carries to `Off` and True to `0.2`.

1. **The one that matters: `0.2` must not bite into normal driving.** A car climbing or cresting at racing speed must keep its throttle where it is legitimately pulling, with the cut appearing only where the engine's own acceleration exceeds what the wheels can explain. If the field reads as hesitant on climbs, the default is too sharp.
2. **`0.1` is the control** — it reproduces build 614's behaviour exactly, so a car that behaved well there must behave identically at `0.1`. A difference means the rate is not the old slope.
3. **The ends** — `Off` must leave the climb speed standing, and `1.0` should read as a near-switch, a full lift arriving in about a third of a second, with no crash and no stuttery car.

## Stability owns the throttle cut (dev build 617, driver-verified)

The throttle cut was a binary declared by two arms — the chassis riding above its rest height, and yaw demand past the grip tolerance. Both are gone. The cut is a live target from one signal now: `ARS.WheelsOffGround` counts wheels reading exactly zero on the side-slip axis (`wheel+0x1C4`), the target is `1 − 0.25 × off`, and it glides at 2/s in and 1/s out on stability's own rate pair. Four wheels down is full throttle, two down is half, none down is closed.

1. **The historical failure first: decompression must not count.** This was built once before on the longitudinal ratio and retired because a decompressed tyre still read ~0 there. On a straight, over a kerb and through a compression the count must stay at zero wheels off, and the violet pedal-bar sphere must not appear; a wipe of throttle with all four wheels visibly down is this test failing, and the fix is a floor above the noise rather than the exact-zero test.
2. **A jump reads as airborne.** Over a crest or a launch the count should reach its maximum and the violet limit sphere should walk down about a quarter per wheel, arriving over roughly half a second.
3. **Recovery is half the rate.** After landing the cap must climb back over about a second, slower than it fell, with the sphere following down rather than snapping back.
4. **The arm that was lost.** A four-wheels-down slide no longer draws a throttle cut of its own — the TCS cap and the steering blend are what answer it now. A slide that reads unmanaged is this trade, not a defect in the count.

## The creator's arc has a tightened middle (dev build 620, compile-verified, drive owed)

`GenerateArc` emitted a constant-radius section, which left the apex search with no minimum to find: every node read the same radius, the first one won, and the apex landed at the arc's entrance. The section's middle is now pulled 8% tighter, radially about the arc's centre so both endpoints and both end tangents are untouched, with a flat band over the first and last 15% so the pull cannot register as a tighter bend at the joins.

1. **The apex lands mid-arc.** Draw a section in the creator, save the track, load it and watch where the cars turn in: they should clip the middle of the drawn arc rather than arriving at its entrance.
2. **The trade, when driven.** The middle is a genuinely tighter road, so the recorded apex radius is 8% smaller and that corner's 1 g speed about 4% lower on created tracks. That is what the tighter middle is for, but it is the thing to judge: if a drawn corner now reads as too slow, the depth is the number.
3. **No kink at the joins.** A section joined to a straight or to another section must show no step and no twitch, which is what the end bands are for.

## The creator rebuilds the route table as you draw (dev build 624, compile-verified, drive owed)

Applying a section or deleting nodes now rebuilds the derived table on the same tick — the node geometry around the change, the apex table whole — and the circuit/point-to-point flag is recomputed from the route's end gap rather than inherited from the last track loaded. Entering the creator clears the table it would otherwise carry over.

1. **The rebuild agrees with the load.** With `Log.log` at Info, every apply and delete prints `Route rebuild: incremental matches the full pass (N nodes, local Xms, full Yms)`. `INCREMENTAL DIFFERS` is the check failing rather than a cosmetic message: the line carries both signatures, so copy them out.
2. **The cost number.** That same line is the measurement that decides whether the local rebuild was worth building — if the local time is a rounding error beside the full pass, the locality is doing its job, and if it is not, the corner cut is where the time went.
3. **Nothing throws on a clear.** With Show Bumps on, press Aim until the route empties: the log must not fill with the same exception every frame, which is what the missing bump clear would have caused.
4. **The table survives the edit.** After several sections, the corners reported at draw time must be the ones the saved track loads with.

## Merged corners own the whole bend (dev build 625, compile-verified, drive owed)

When two corners merge, the survivor's span was re-walked on the radius criterion around its own apex. Since the survivor is usually the tighter corner, that threshold is the smaller one, so the merged corner could cover *less* road than the two corners had separately — leaving the first corner's entrance with no corner record, and therefore no braking zone. The survivor now takes the union of the two spans, bounded only by the hard node limit.

1. **Two close corners.** Take a track with a pair that merges — a fast kink a second or two before a slower corner — and watch the approach: the cars must begin preparing at the *first* corner's entrance rather than somewhere between the two.
2. **The braking zone is covered.** The tell of the old behaviour was a stretch at the first corner's entrance with no braking requirement on it; with the fix the plan should ask for that speed from the union's entrance onward.
3. **No over-preparation.** The union is a superset of both spans, so a car must not now crawl through the second corner: if the merged bend reads too slow, the union is too generous rather than the merge rule being wrong.
4. **The unmerged case.** Two corners that do *not* merge must be untouched — their spans still come from the region walk, which this change does not touch.

## The track list survives a shrinking list (dev build 630, compile-verified, drive owed)

`RefreshTrackList` swapped `_trackListItem.Items` while the selected index could sit past the new end, and LemonUI's `Items` setter reads `Items[index]` as it lands with no clamp, so a shorter list threw inside the setter; an emptied list threw at the assignment that follows it. The index is now parked on a valid entry before the swap, as the car-pool refresh already did.

1. **A shrinking list** — with the game running, delete several `Tracks\*.xml` files from the folder, then make `RefreshTrackList` run against the shorter list (save a created track, or open the creator and apply a section): the Select Track row must rebuild with no exception popup, and the selection must land on a real entry.
2. **An empty list** — move every track out of the folder and trigger the same refresh: the menu must still open with an empty row rather than throwing `'There are no available items.'`, and Instance Track must say no tracks were found rather than hand a deleted path to the loader.
3. **Empty, then refilled** — with the list still empty, put one track file back and refresh again: the row must come back selected on a real entry. This is the case the first cut of the fix missed — an empty pass leaves the index field stale while the getter reports −1, so the refill read `Items[stale]` and threw inside the setter.
4. **Nothing moves at rest** — with the file set unchanged, opening the menu and saving a track must leave the same track selected, and `Menu-Race.ini`'s `Track` value must not change.

## The slide response batch (driver-verified — the slides answered)

Builds 694–699 cut the damper's authority back and moved the blend's band and the countersteer's share in one run, driven as one batch. The blend ramps linearly across the car's own `LateralTractionCurve`, from half of the authored peak slip to four-fifths of it, and the countersteer answers the whole slide angle, so the blend's weight is the only modulator and the limiter's countersteer allowance reads the same weight. The damper's whole term is halved (`DamperEffectScale`) while its crossing overshoot past neutral is kept whole (`DamperCrossingShare`); the pair is one decision, because halving only the crossing left the base term at full strength and the slide still did not answer.

**The driver's verdict: good enough — the batch is crossed.** What it does not claim: the band and the share are a taste cut rather than a derivation, so a car whose tyres read differently may want a different band; and the damper's scale is a global trade, since the term is load-bearing for holding centre — a car that reads unstable at half authority is answered with the damping dial, not by restoring the scale.

**Revised on a later drive: the damper's scale returned to full authority.** The slides kept their answer, but the field read a smidge unstable at half the term, so `DamperEffectScale` sits at 1 and the batch's keepers are the blend band and the countersteer share, with the crossing overshoot whole.

## The picker looks past an ignored corner (build 704, driver-verified on the reported corner; review fixes shipped in build 706, drive owed)

A wide corner the car passes flat was still taken as the promotion reference for the tight corner behind it: at the sweeper's apex the tight corner was promoted already inside its own window, so the whole approach was skipped and its outside line never ran — every lap, not only the first. Now the requirements scan flags a corner that cannot ask for positioning by its own radius (`DetectedRadius >= CornerTightRadius`) and has not asked for braking by the time the car is inside its 3.95 s window; `RefillApexQueue` gives slot one to the next corner instead, and the flag holds until that apex is behind. The driver confirmed the fix on the corner that showed the bug.

1. **The hairpin behind the sweeper takes its approach, every lap.** On the track that showed it, the tight corner after the fast one must show the outside line for the same number of seconds it has on the laps where it worked.
2. **The log names the skip.** With `Log.log` at Info, one `Queue skip <name>: corner N ignored (wide R m, no braking asked)` per approach, once — a repeat each scan would be the flag failing to latch.
3. **No regression late in the approach.** A skipped corner must not become slot one again unless it armed braking — that revocation is deliberate and retakes slot one the same pass — and a skipped corner's log line should not reappear within the same approach.
4. **An armed corner is never skipped.** A wide corner that arms a braking plan must show its usual braking: the flag is revoked in the same pass and the queue re-picks it, so the braking plan appears the tick it is asked for rather than half a second later.
5. **The consumer shifts, from slot one no longer being the nearest corner.** DefendLane and Yield now arm in the 1-3 s and 0.5-2 s windows before the *next* corner rather than the skipped one; a nitrous shot that used to be vetoed while approaching or inside the skipped corner can now fire; the lane-crossing taper (`turningIn`) stays active through the skipped corner's turn-in where it used to stand down; brake learning does not sample the skipped corner's approach. None of these is a defect to "fix" blind — they are the shape of the trade and are judged on the drive.
6. **Nothing else moves.** A tight corner behind another tight one keeps the successive-corner suppression exactly as before.

## A corner is anything under 50 m (build 706, compile-verified, drive owed)

The table's ceiling and the region scan both drop from 100 m to 50 m (they already sat equal): a bend wider than 50 m on its tightest node is no longer a corner at all, so it gets no apex entry, no braking plan, no chevron and no card anchor, and route speed is the only law that slows a car for it. At 50 m every corner the table keeps is under `CornerTightRadius`, so the "wide" class the picker skip exists for disappears — the skip stays as the guard for a ceiling that produces one. The cost to watch: bends of 50-100 m lose their braking plan and are driven on route speed alone.

1. **The corner table shrinks.** At Info, the `Apex table:` aggregate line should count fewer corners on a track with sweepers; the ones left are hairpins, kinks and tight bends.
2. **The sweepers are driven on route speed.** A bend that used to have a braking plan should now come on the route limiter's speed; the tell of a problem is a late, sharp brake rather than a prepared approach.
3. **The tight corners keep their braking and their outside lines.** No corner the table still holds may read slower or miss its approach — that would be the ceiling cutting into the population it should keep.
4. **The lap times tell the trade.** Fewer braking plans can read faster (less over-preparation) or slower (more reactive braking); which way this track goes is the judgement.

**Shipped default: 200 m (build 724).** The ladder was written at 50, moved to 80, and settled at 200 once the field drove the range: at 80 m real corners were missing, at 180 m fewer problems, and 200 — the top of the ladder — has every corner the AI needs. Corners between `CornerTightRadius` (50 m) and 200 m are the "wide" class the picker skip is live for, and nothing wider than 200 m reaches the table at all.

**Observed on By The Highway (build 722)**: the 90° left the field expected is absent at 80 m and returns at 100 m, and the widest apex the table kept at 80 m was 70 m — so that stretch is driven on route speed alone with no braking plan, which is exactly the cost this section named. The merge was cleared as the cause in the same run: zero `Corners merged` and zero `Corner skipped` lines, all nine corners logged as `Corner add`.

**The rung, measured by the driver (build 723)**: 80 m breaks the table, 180 m leaves fewer problems, **200 m — the top of the ladder — has every corner the AI needs**. The lesson is a change of philosophy rather than a bigger number: **cribar curvas por radio da más problemas que soluciona; la tabla debe describir la carretera y la IA debe aprender a ignorar las curvas que no necesita**.

## The G line over the player's own car (build 721, compile-verified, drive owed)

A Debug checkbox, `Show G Line`, draws one line from the player's own car's roof in any state: it samples its own speed because a racer's 20 ms window lives on `Racer`, and it averages 200 ms like every other acceleration read in the mod. Direction is the horizontal acceleration the car is pulling, length is that magnitude at one metre per G with the top clamped, and a perpendicular tick at the tail makes the direction readable — a bare line looks the same in both directions.

1. **It draws outside a race.** In free roam, with no track, no grid and no racers, the line floats at roof height over the player's car and follows it. This is the whole point: the existing Gs Projection needs a race and an AI car.
2. **Accelerating points forward, braking points back.** The tick marks the tail and the tip the direction, so hard throttle and hard braking must swap the two ends.
3. **A hard corner points at the corner's centre.** Through a committed turn the line should stand roughly across the car's heading, not along it; a line that lies down the car's nose through a corner means the lateral term is not reaching the average.
4. **Length tracks magnitude, and the clamp holds.** A gentle bend reads short, a hard one long, and a crash or a kerb strike must not throw it into a spike — the 200 ms mean and the clamp are what answer that.
5. **It survives getting out and back in.** Leaving the car and re-entering must restart the window instead of differencing against the old speed, so the line reads nothing for the first 200 ms and then settles. A line that jumps on re-entry is the restart gate failing.
6. **The tip reads the number.** The label sits at the line's end as `2.5Gs`, one decimal, and it reports the real magnitude even where the line itself is clamped — past 4 G the stroke stops growing and the number keeps climbing.
