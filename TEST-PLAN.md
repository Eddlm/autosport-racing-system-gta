# ARS — test plan: the uncommitted recovery / off-track / DNF work

**Status: NOTHING here has been driven.** The working tree is uncommitted (5 files) and the deployed DLL is
dev build **#433**. A build + reload is enough (SHVDN reloads the scripts live, no game restart).

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

11. In Drive, stop the car again (or wedge it).
    **Expect:** another 1 s Reverse (same A predicate, 2 s).
12. A car that **cannot move at all** (no position change over 1 s) → **teleported** to the drivable bound.
13. A car re-stuck in **Reverse** after the 6 s recovery budget → teleported.
14. A car that keeps **moving** is never teleported at the budget (by design).

## 6. The 15 s arm delay (start line)

15. At the green, a heavy / wheelspinning car still under 2 mph must **NOT** reverse — no recovery at all for the
    first 15 s of the race.
16. A car pushed off track inside those first 15 s must **NOT** enter Drive either (it still gets the off-track
    throttle cap and the normal lane law).
17. After 15 s, recovery arms normally.

## 7. DNF parking — option OFF (shoulder)

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

**Known history:** the old ghosting was reported as *precarious* — contact sometimes happened with the mode on.
The cause is now known from the engine source: the native's third argument is a **mode**, and the old code passed
`true`, which selects `NO_COLLISION_RESET_WHEN_NO_IMPACTS` — the entry **clears itself** once the pair stops
touching. The flag is now requested in the `PERMENANT` mode, so a single application should hold. If contact still
appears, the cause is an entity swap (respawn, or the AI dummy-vehicle conversion past ~40 m), not the mode.

**This is the ONE-SHOT build.** The mode is applied exactly once — at the green, and whenever the option is
toggled on — for every pair regardless of spacing. Nothing re-asserts it per tick. The matrix below is therefore a
genuine test of whether the entry holds, and a single contact is the falsification.

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

## 13. Rival detection — the stale front-gap fix

## 13. Rival detection — now the ROUTE frame

The old hit test measured a straight line through the car, so a corner bent the racing line away from it and the
call bailed out; `FrontGap` was also written only after the closing guards, so it was a remembered value carried
across rivals and races. Both are gone in **#441**: the gap is the route arc (`CumulativeDistance`, exact metres,
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
