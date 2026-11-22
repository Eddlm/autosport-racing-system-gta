# VERIFY IN GAME — outstanding

Build deployed: **1896**. What is ✅ is yours, verified in game; what is ⬜ is still **compile-only**.
Tick a line with `[x]` or tell me «verified N» and I update this file. Quotes in «…» are your own words,
translated where you spoke Spanish.

**Section P is closed** (all eighteen verified). **58 open rows below**; everything already driven or disposed of
sits in **Answered** at the bottom, out of the way, with **75** closed rows under it. The detail of the 30-10 hunt
stays in `docs/audit/410-hunt-2026-10-30.md`.

---

## R · 2026-11-22 (1886 → 1889) — all five verified in game, see **Answered**


## V · 2026-11-22 (1893) — the brake factor is pinned

| # | What | How | Passes if | State |
|---|---|---|---|---|
| V1 | **every corner reads 0.70, and holds** (1893) | the apex debug text, with Brake Learning off in the menu | every corner shows `b 0.70`; a restart keeps it and no corner drifts | ⬜ |

## W · 2026-11-22 (1896) — the route band averages its five tightest nodes

The band no longer reports its single tightest node: it averages the five tightest (no cap), so one noisy radius cannot drive the number. Built deliberately to be tested against the prediction that a tight piece narrower than five nodes gets averaged with open road and is under-protected.

| # | What | How | Passes if | State |
|---|---|---|---|---|
| W1 | **a real tight feature is still safe** (1896) | a short kink or chicane whose tight piece is narrower than the band's five-tightest set, with open road in the same band | the car still makes the kink; running wide or understeering there confirms the average under-protects it | ⬜ |
| W2 | **the number stops jittering** (1896) | the route radius readout (`ShowRouteRadius`) through a corner | the radius reads steady node to node instead of flicking between the tight node and the open road | ⬜ |

## U · 2026-11-22 (1892 → 1895) — corner speed learning, live

| # | What | How | Passes if | State |
|---|---|---|---|---|
| U1 | **a clean corner pays 2 m/s** (1892) | a corner taken cleanly, brakes released before the entry, no traffic nearby | a notice reads `corner #n · clean · +0.0 → +2.0 m/s`, and the apex debug line for that car shows `+2.0 m/s` | ✅ |
| U2 | **a slide or a wheel off takes 2 back** (1892) | a corner where the car reaches 10 degrees of slide or more, or puts a wheel off the road | `slide at 10 deg or more` or `left the road`, and the offset drops 2 m/s | ⬜ |
| U3 | **traffic voids the verdict** (1892) | a corner taken while avoiding a car: a lane taken to pass, or the side-by-side push on the nose | `traffic · offset stays`; the corner's number does not move either way | ⬜ |
| U4 | **the number is used next pass, and stops at 10** (1892) | the same corner on following laps, offset at +2 or more | the car carries visibly more speed through it, and repeated clean passes stop at +10 m/s (`= at the limit`) | ✅ |
| U5 | **the window ends when the car is back on the power** (1895) | a corner the car takes and then goes above half throttle out of | the verdict lands at the throttle pickup, not at the six-second cap; a corner that never lifts still runs the full window | ⬜ |

**Left as-is at his word, 2026-11-22 (drive verdict on 1895):** it works — the number moves and the car uses it — but where the route curvature term is the smaller of the two the offset is superseded and nothing in the car changes. U2, U3 and U5 were never exercised. Parked means shipped and left alone: the switch stays on by default, nothing is disabled.


## T · 2026-11-22 (1891) — the corner speed switch, before anything learns

| # | What | How | Passes if | State |
|---|---|---|---|---|
| T1 | **the switch exists and holds** (1891) | Rivals menu, pace page: toggle Corner Speed Learning off, restart, reopen | the checkbox reads off after the restart, and the racing is identical with it on or off — this build's learner is inert by design | ⬜ |

## S · 2026-11-22 (1890) — the boost waits until the corner is behind

| # | What | How | Passes if | State |
|---|---|---|---|---|
| S1 | **a corner exit is not open road** (1890) | a rival unwinding out of a corner onto a long straight, next entrance far away | no burn until about a second after the corner is behind it, then the burn can fire as before | ⬜ |
| S2 | **the straights still burn** (1890) | a race with several long straights, a fight and a finish | burns still land on the straights, in fights and near the finish: the fix narrows the window, it does not silence the bottle | ⬜ |

## Q · 2026-11-16 (1878 → 1885) — the blend is gone, the aim leads further, every verdict speaks

| # | What | How | Passes if | State |
|---|---|---|---|---|
| Q1 | **the widest-arc blend is gone** (1878) | drive the wide fast sweepers that used to launch you too hot into the apex | cars hold the speed the line asks for; no corner feels out-cornered by its own radius | ⬜ |
| Q2 | **brake aim leads 1.25 s** (1880) | the same entry you used to brake at | braking starts visibly earlier (~1.79× the old lead distance); the car reaches corner speed with more room to spare | ⬜ |
| Q3 | **no verdict is rate-limited away** (1882) | `Brake Learn Notify` on, drive a chicane or any two corners < 1.2 s apart | **both** verdicts reach the UI — before the fix the second one was silently dropped | ⬜ |
| Q4 | **an aborted corner still closes** (1882) | engage a corner (within 50 m of the apex), then spin/stop before the apex so the queue moves on | `corner #n · apex not reached · factor stays x.xx` appears once — no silence, and no second message if the same corner is re-held and passed | ⬜ |
| Q5 | **a pending verdict survives learning off** (1882) | brake-learn notify on, cross an apex, and untick Brake Learning within 0.75 s | the verdict still fires (saying `learning off · factor stays`), instead of vanishing with the toggle | ⬜ |
| Q6 | **overshoot deducts** (1884) | a corner where a rival crosses the brakepoint clearly hot (> 3 m/s over target) | one message `overshot by X m/s`, factor **−0.02** exactly (0.75 → 0.73), clamped at 0.25 | ⬜ |
| Q7 | **undershoot adds** (1884) | a rival that was above target+3 on the approach, then drops below it while still 7+ nodes (≈7 m) before the brakepoint | one message `undershoot, below target+3 with N nodes spare`, factor **+0.02** exactly, clamped at 1.25 | ⬜ |
| Q8 | **the band holds and still speaks** (1884) | a corner whose crossing is within ±3 m/s of target, or one approached from below without ever being above the bar | message `in the band · 0.75 = holds` — no change, never silent; and never two verdicts for one corner | ⬜ |

Note (1882): the learner runs only for **AI racers** — `UpdateBrakeLearning` sits inside `if (!ControlledByPlayer)`,
so the player's own car never announces. Pre-existing and by design (the row text says *rivals*), just now checked in code.
Known limit (1884): the undershoot latch reads speed and distance only — traffic that slows a car below the bar
more than 6 nodes out, without a spin or contact, can still latch and read as margin. A collision gate is the
one-line fix; only worth it if it shows up in a drive.

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
| B2 | **brake learner** (1126) | an entry overshot under braking with `Brake Learn Notify` | the notice **comes out without Show Track Analysis** and the cut is **0.15 at most** | ⬜ |
| B3 | **walls at three** | Show Rivalry, a rival on each side | both walls stay drawn | ⬜ |
| B6 | **board · position** (1125) | look at the top-left corner | `POS` no longer overlaps the hints/tips | ⬜ |
| B7 | **slow allowance** (1127) | a 5-15 mph corner with `Show Front Slip` | the car turns and the token `x1.5`/`x2.0` appears | ⬜ |
| B8 | **Peak Slip Target** (1128) | token `x0.9` and `F` in a fast corner | the cut starts before the peak (F 100 = 90 % of the peak) | ⬜ |
| B9 | **Deepest Cut (%)** (1385) | raise it from 60 to 80 in a long slide | it cuts more than before; at 0 it cuts nothing; at 60 identical to 1384 | ⬜ |
| B10 | **the reference is the outer wheel** (1391, alive in `MeasureSlipFromMemory`) | `Show Front Slip` with `Slip From Memory` **on**: steer one way in a fast corner and watch the token's `F` | the number follows the **outer** wheel (the one slipping less); with the wheel straight it is the average of the two | ⬜ |
| B13 | **Brake Aim Depth** (1397) | a corner where it already braked, raising it to 0.4 and then 0.8 | it brakes **later** and enters faster; if after a few laps it brakes where it used to, that is the learner lowering the factor | ⬜ |
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
a placeholder and not a decision to brake. The lane can adopt the next corner while the plan still brakes for the
current apex: that is what an S asks for, and the ramp is what keeps the first ask small. `UpdateCornerRequirements`
runs every frame because it is the detector, not the refill. `_cornerContexts` does prune on a revision change
(`SyncCornerContexts`), so that audit row was false. AGENTS/ now carries dated CURRENT blocks; its history stays as
history.

---

## Answered — every row already driven or disposed of

Kept for provenance: these are the in-game verdicts and the dispositions. Nothing here is work.

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

