# ARS — test plan: build 469

**Status: partially driven.** The recovery's reverse phase and its teleport, now gated on the escape budget, are
driver-verified on build 450 (`666c89b`); No Collision's one-shot mode and its per-tick all-pairs form were both
driven and **failed** against the engine's one-slot limit, and its nearest-rival form is now **driver-verified**
(`cbae845`); the slew is driven — 45 read as less stable and **180 removed most of the stability problems**
(`393bfdd`, driver-verified); the damper's speed scale is capped at 1 and **driver-verified as an improvement**
(`5bb94bd`); the engine restart, the steer limit governor, its damper bypass and the maneuvering ramp are new and
undriven. Everything else below is undriven. The deployed DLL is dev build **469**; a build + reload is enough
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

Start a race with several AI cars. `Show Inputs` on the Debug menu helps for the pedal bar, and `Log.log` in the
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
2. Watch the **white sphere** on the pedal bar while off track (Show Inputs).
3. Put the car half-off (centre still on the track). **Expect: no cap** — the rule is centre-past-edge.

## 2. Recovery trigger A — stuck against something

**Driver-verified on build 450:** the 1 s reverse backs off as designed instead of being teleported mid-phase.

4. Wedge a car against a tree/wall and let it stop.
   **Expect:** after 2 s below 2 mph with the plan asking for more → **Reverse** (1 s, straight, backwards).
5. Then **Drive**: forward at ≤20 mph, steering back toward the route.
6. **Is 1 s of reverse enough to clear a tree?** It is ~4× the distance of 500 ms. Watch it does not back into the car behind.

## 3. Recovery trigger B — off track

7. Send a car off the track and keep it there 2 s.
   **Expect:** it enters **Drive directly** (no reverse), slows to ~20 mph, steers back and rejoins.
8. Confirm a car off track but **moving** does not enter Reverse.

## 4. Recovery exit

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

## 6. The 15 s arm delay (start line)

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
Watch a flooded car come back instead of sitting on the throttle.

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
    the apex. The blue Rival sphere on the pedal bar must appear and the throttle glide down. On the old model this
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
- **AGENTS.md** still carries the failed intel-gather section and is unedited.
---

## Crest entrance move (built `27ee097`, not yet driven)

The question is whether braking now starts where the track stops being flat, with the car still loaded when it does.

1. **A crest before a braking corner** — pick one with a long obvious rise. The plan should begin earlier than the previous build, at the base of the crest rather than its top; the corner-entry speed should read lower because more braking distance is being spent before the unload.
2. **The same corner twice** — once at pace, once deliberately slower. The move is a fixed node walk, so a slow approach spends the extra distance idling; if that looks like braking far too early, note the speed, because the gate should be braking-span-dependent rather than fixed.
3. **Log check** — generation now prints `entry=` beside `node=` per crest. Expect `entry` earlier in travel order than `node` (a lower index on a circuit), and the move distance on the `Crest` lines; a skip names its reason.
4. **A track with no crest before its corners** — nothing should change.

## Bump scan overlay (built `f3f318f`, rise walk revised through `148db70`, not yet driven)

The question is whether the scan finds the lips a driver can feel, and whether the ones it finds are the ones that launch a car. It reads the **route line only** — the raycast over the real surface, and the lane-local case with it, are a later stage.

1. **The overlay** - enable `Show Bumps` in the Debug submenu and drive the track. A cyan 3 m vertical tick and a line across the road mark every lip within 400 m of the player (`AutosportRacingSystem.cs:1717`). Expect a marker where the rise ends and the road drops away, not in the middle of a slope and not on every undulation.
2. **Log check** - generation prints one `Bump:` line per lip with `lip=`, `rise=`, `grade=` and `curvature=`, then a closing `Bumps: N lips, minimum departure grade …, run 2.0-8.0m` (`TrackLoader.cs:421-424`). Expect a handful to a few dozen on a circuit; a count near the node count means the detector floor is too low.
3. **A known launcher** - a jump the cars already take must carry a marker, and its `grade` should be the steepest of the set.
4. **A long climb** - a rise longer than eight metres must produce no marker at all. If it does, the run limit is not biting.
5. **The seam** - a circuit whose first/last node edge steps in height prints `Bumps: the route seam steps at grade …, so no lip is taken within 8.0m of it`; within that stretch no lip may be taken, while the rest of the circuit scans normally. A step-only seam is not a real lip.
6. **Nothing drives on it** - behaviour and lap times must match the build before `f3f318f`: the `RequiresEarlyBrake` / `RampEndNode` hook the braking plan reads is still unset, so a marked lip must not move anyone's braking point.

## Steering slew and slide blend (the slew is driven; the blend is not)

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

**The damper's two subtraction rules (new, undriven).** It may only ever *subtract*: a term that would add steer in
the sign the command already has is zeroed, so a left command never gets more left from it. And past neutral — the
countersteer region, where the wheel points against the car's own rotation rather than merely less into it — it keeps
half its authority. The axle gate that used to guard that halving (`understeerDeg < 0`, and `UndersteerDegrees` with
it) is deleted. Deliberately **not** a cap tied to the slide: the code already records that such a cap fell to zero
with the slide and took the rate feedback off a straight car, which set it oscillating.

Watch: the no-add rule makes the damper's straight-line action intermittent, since it now acts only when it opposes
the command's sign rather than roughly always. If a straight-line weave or a limit cycle appears, that rule is the
cause and not the gain cap; if the cars feel *more* planted, the rule is doing what it was asked to.

## Steer limit governor — A/B (not driven)

The limiter now takes its governing quantity from **Steer Limit Mode** in the Settings menu, a two-item list. The
slip-balance knee (a degree past the applied command, capped by the ceiling) is **removed from both**: it was a
misreading of the intended "slide angle plus one degree", which belongs in the slide-governed limit.

- **Yaw-Governed** (default, the previous behaviour minus the knee): the cornering ceiling from the at-speed peak
  slip, scaled by yaw usage from Turn-In Minimum to Turn-In Maximum, plus the countersteer allowance (slide angle
  times blend weight) on the answering side.
- **Slide-Governed**: the cornering law stands **and the slide adds to it** — the ceiling is the larger of the two.
  No yaw turn-in share. It was a *replacement* until `467`, which put the ceiling at one degree above 30 mph and left
  cars unable to steer in or rejoin; as an addition a car keeps the full cornering law and a slide only ever opens
  more. The addition was **halved in `468`** to half the slide angle plus half a degree, which makes it bind only
  past a **~20° slide above 30 mph** — a spin rather than a corner — so a test of its effect should expect little.
- **Driven in Slide-Governed at `467`**: off-track rejoin at speed, corner entry above 35 mph and the
  stopped-and-perpendicular rotation all pass; the slide addition in a corner reads as working but is hard to tell
  apart from over-rotation countersteer; and there is **mild straight-line weaving that Yaw-Governed does not show**,
  which the halving cannot address because the addition never binds on a straight.
- The **addition versus replacement** distinction is the whole A/B: Yaw-Governed *caps* steer-in by yaw usage, while
  Slide-Governed never caps it and adds as the slide grows. The curves are plotted in
  `docs/steer-limit-modes.png`, drawn by `docs/steer-limit-modes.py`.

The answering side is now **one shared path in both modes**: the countersteer allowance (slide angle times blend
weight) **and a damper bypass**. When the damper's term pushes *against* the rotation **and** the command does too,
that side is raised to full lock — the rotation leads the body slip, so a snap is answered before the slide-governed
ceiling can see it. The command half of that test is load-bearing: without it the raise landed on steer-in, which
shares the side with the correction. The mode switch therefore moves only steer-in authority.

A **maneuvering ramp** sits under both modes: the limit eases from whatever the mode computed at 30 mph up to the
car's own lock at 5 mph and below, and it is a raise only, so neither the mode nor the yaw share can cut it back.
**Reverse is full lock**, and so is standing still, since the ramp is keyed to forward speed — backing up is placing
the car, not cornering.

Watch:

1. **Slide-Governed, corner entry** — the cornering law stands untouched, so the car should steer in exactly as
   Yaw-Governed does at full yaw share. If it still ploughs straight on, the limit is not the reason.
2. **Slide-Governed, catching a slide** — a slide past the cornering law opens extra authority, and the answering
   side can reach the slide angle plus a degree, so the counterbalancer must never be clipped.
3. **Yaw-Governed, corner entry** — dropping the knee took away the one place that gave a degree back. Watch
   whether cars now understeer where the knee used to help.
4. **Either mode** — the switch persists in the menu settings, so confirm the mode you think you are driving is the
   one the menu shows.
5. **The bypass is countersteer-only now** — it needs both the damper term and the command to oppose the rotation.
   So a limiter that reads as absent while countersteering is working as intended, and steer-in should be capped
   again. The sign test still flickers near zero yaw in a steady corner; if the limiter reads as intermittently
   absent there, the tightenings are the allowance's own slide gate or a magnitude floor on the damper term.
6. **A snap in Slide-Governed** — the thing the bypass is meant to buy: the car should now catch an over-rotation
   whose body slip is still too small to open the slide-governed ceiling. If it still spins, the bypass is not the
   missing authority and the target, not the limit, is what is short.
7. **Inside the band the modes do not differ** — from 5 to 30 mph the limit is the ramp's value whichever governor
   is set, so the two only diverge above 30 mph. Judge the A/B there.
8. **Reverse** — a reversing car should hold full lock in both modes. If a recovery reverse still reads as steering
   straight, the ramp is not the reason.
