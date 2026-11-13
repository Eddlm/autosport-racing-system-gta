# VERIFY IN GAME — outstanding

Build deployed: **1802**. What is ✅ is yours, verified in game; what is ⬜ is still **compile-only**.
Tick a line with `[x]` or tell me «verified N» and I update this file. Quotes in «…» are your own words,
translated where you spoke Spanish.

**Today's work is section N**, at the top: Nearby Cars, builds 1784 → 1802. N1 and N3 were verified on
13-11; the rest is still compile-only. Count: **48 open** (44 old + 4 from Nearby Cars). The detail of the
30-10 hunt stays in `docs/audit/410-hunt-2026-10-30.md`.

---

## N · 2026-11-13 (1784 → 1802) — Nearby Cars

Ten builds since the section below was written; every row here is **compile-only**.

| # | What | How | Passes if | State |
|---|---|---|---|---|
| N1 | **the cars around you join the grid** (1784) | Nearby Cars on, one empty car in range, Start Race | it joins the grid and runs with the pack, not as a prop | ✅ **verified 13-11 · 1802** |
| N2 | **adoption runs after the pool, and a ghost seat gets a driver** (1790) | kill a parked car's ambient driver with a clean headshot — the corpse counts as occupied until the engine culls the body, then the seat is a ghost; Start Race once the body is gone | it is adopted, a driver is spawned and ends up **in the seat** (`Racer.cs:584`), the car drives, and the log's `ghostSeat` count rises | ⬜ unprovoked: no invalid ped yet. ARS never deletes a preview car's ped itself, so the only producer is the engine culling a corpse; walk-away (car despawns too) is the fallback |
| N3 | **the toggle is a three-way choice** (1794) | switch Ignore → Join As Is → Join And Tune | Ignore draws nothing and adopts nothing; Join As Is adopts and leaves Auto Styling alone; Join And Tune lets the tuner see them | ✅ **verified 13-11 · 1802**: «Works, they tune if the setting is chosen» |
| N4 | **adopted cars are sorted, not pinned** (1799) | adopt mid-field, then look at the grid | they take their place by pace with the rest; nobody is nailed to a slot | ⬜ |
| N5 | **the chevrons share one angle** (1800, 1802) | a track start and a nearby car in the same view | both chevrons face the same way: one constant, `TrackChevronRotation` | ⬜ |
| N6 | **the chevron says which car is adoptable** (1801) | one healthy empty car and one unhealthy one in range | blue for race-fit, orange for not; an **occupied** car draws nothing — a corpse in the driver's seat counts as occupied | ⬜ |

---

## L · the racing line (1301 → 1312)

| # | What | How | Passes if | State |
|---|---|---|---|---|
| L1 | **phase by distance** (1302) | V and U corners, watching the line | it no longer accelerates over the last nodes | ✅ **verified 28-10 · 1302** |
| L2 | **lean by width and radius** (1308) | a narrow track and a wide one | on the wide one it attacks the inside with more pace, with no jerk of the wheel | ✅ **verified 28-10 · 1308** |
| L3 | **out-in removed** (1312) | any corner | the outside orange is still there on the approach, **with no** magenta, and the entry feels the same as in 1308 | ✅ **verified 28-10 · 1312** |

## A · Without driving (log)

| # | What | How | Passes if | State |
|---|---|---|---|---|
| A2 | **atomic settings** | change one row, then make the ini **read-only** and change another | the value persists; with the file locked an `Error` lands in the log instead of the value being lost | ⬜ |
| A3 | **304 · empty grid** | `Max PI Deviation` low enough that nobody is in band, Start Race | it **does not start**, it says `No car within N PI of X`, and **no flares are left behind** | ✅ **verified 26-10** |
| A4 | **preflight** | the same case as A3, watching the log | `Start Race refused: …` appears **before** anything is instanced | ✅ **verified 26-10 (1124)** |
| A5 | **layout · names** | start a race | rivals carry a name (`Drivers\names.txt`) and tips come out | ✅ **verified 26-10** |
| A6 | **new roster** (1169) | change the Rivals roster to `cars.txt` and ask for a grid | it indexes **643 models** and builds the grid. Before, it used to hang (or appear to) when the roster changed | ⬜ |
| A7 | **rotated log** (1171) | start twice in a row | `ARS.prev.log` exists in the **game root** with the previous session | ⬜ |

## B · One normal lap

| # | What | How | Passes if | State |
|---|---|---|---|---|
| B1 | **corner seam** | **El Burro Rally** with `LogLevel = Info` | the first `Corner add` gives a sane `start=` and nitro is **not** vetoed for the whole lap | ✅ **verified 26-10**: 14 corners at El Burro / 18 at Oct1, `apex=0` only in the `skipped`, and it brakes for the first corner |
| B2 | **brake learner** (1126) | an entry overshot under braking with `Brake Learn Notify` | the notice **comes out without Show Track Analysis** and the cut is **0.15 at most** | ⬜ |
| B3 | **walls at three** | Show Rivalry, a rival on each side | both walls stay drawn | ⬜ |
| B4 | **board · names** (1117) | look at the column | **no** PI brackets, and the PI with **one decimal** | ✅ **verified 26-10** |
| B6 | **board · position** (1125) | look at the top-left corner | `POS` no longer overlaps the hints/tips | ⬜ |
| B7 | **slow allowance** (1127) | a 5-15 mph corner with `Show Front Slip` | the car turns and the token `x1.5`/`x2.0` appears | ⬜ |
| B8 | **Peak Slip Target** (1128) | token `x0.9` and `F` in a fast corner | the cut starts before the peak (F 100 = 90 % of the peak) | ⬜ |
| B9 | **Deepest Cut (%)** (1385) | raise it from 60 to 80 in a long slide | it cuts more than before; at 0 it cuts nothing; at 60 identical to 1384 | ⬜ |
| B10 | **the reference is the outer wheel** (1391, alive in `MeasureSlipFromMemory`) | `Show Front Slip` with `Slip From Memory` **on**: steer one way in a fast corner and watch the token's `F` | the number follows the **outer** wheel (the one slipping less); with the wheel straight it is the average of the two | ⬜ |
| B11 | ~~Target Under Power~~ (1393) | — | parked in 1396: the row no longer exists | ⛔ |
| B12 | **the lane's throttle floor** (1396) | a long bend aimed at the **inside**, with the plan asking for a lift | the throttle **does not drop**: it accelerates even though the plan wanted a lift; hugging the outer edge the floor disappears | ✅ **verified 31-10 · 1396**: «tested, works great» |
| B13 | **Brake Aim Depth** (1397) | a corner where it already braked, raising it to 0.4 and then 0.8 | it brakes **later** and enters faster; if after a few laps it brakes where it used to, that is the learner lowering the factor | ⬜ |
| B14 | **Car-Centred Radius** (1399) | Figureight between apex 2 and 3, one lap with the toggle **off** and one **on** | off goes wide as it does today; on holds the corner until it really leaves, and does **not** brake earlier on the entry | ✅ **verified 31-10 · 1400**: «we have it, this has helped» — the entry not having got slower is still to confirm |
| B15 | **Slid Radius Lead + the radius toggle** (1404) | the same stretch: toggle **off**, then lead 0.0 / 0.5 / 1.5 | off = reacts late to the entry; 0.0 = measures only what has been travelled, the most cautious on the exit; 1.5 = almost the same number as the main one | ✅ **verified 31-10 · 1404** |
| B16 | ~~an inactive lane plants itself~~ (1408) | — | **dropped in 1409**: the `LaneCrossMinMeters` threshold turned a continuous quantity into a binary state and flickered. The block was removed; the lane comes back through the mover, as before | ❌ dropped |
| B17 | **exponential close** (1132) | `Slip From Memory` **off**: the readout is the proportion | at 500 % it reaches 100 % in ~0.27 s (8 ticks); at 1000 % ~0.3 s | ⬜ |

## C · Deliberate abuse

| # | What | How | Passes if | State |
|---|---|---|---|---|
| C1 | **nitro at the finish** | fire it and **End Race within 3 s** | your car does not come out with the bottle armed; unticking Nitrous mid-burn **stops** the push | ⬜ |
| C2 | **reload** | **Insert** mid-race | no crash, the race disappears, neutral pedal | ⬜ |
| C3 | **race after race** | finish one and launch another before the «Ns to end» expires | the new one **does not die on its own lights** | ⬜ |
| C4 | **creator** | start line + **one** section, save | it refuses and says how many nodes are missing, with no black screen | ⬜ |
| C5 | **short solo** | **Grid Size 0** on a short circuit | it starts | ⬜ |
| C6 | **freecam** | enable it and **get into a car** | the freecam **drops out by itself** | ⬜ |

## D · Decisions and paperwork

| # | What | Notes | State |
|---|---|---|---|
| D1 | **ABS unlocked** (1114) | row alive, it starts with the stored value. **The new description is still missing** — the one you were going to give me | ⬜ |
| D2 | **`Oprahs`** in `names.txt` | does it stay, become `Oprah`, or go? | ⬜ |
| D3 | **tag `v0.9.0`** | **it was already cut**: release published 27-10 with build 1293 and that day's zip, tag at `1d8b91e`. Today's build goes separately, as `v0.9.0.1381` in draft. `docs/audit/` is 1140 KB of internals | ✅ 30-10 |
| D4 | **PI relative, on foot** (1124) | immediate error, nothing instanced, no teleport, no fade | ✅ **verified 26-10** |

## E · that day's work, not yet tested

| # | What | How | Passes if | State |
|---|---|---|---|---|
| E1 | **off-track: 40 under, floor 20** (1171) | take yourself off the line in a fast corner | the car really drops, not 10 mph less: a 35 mph corner asks 20, a 70 mph one asks 30 | ⛔ superseded by I1 — no longer behind a switch, allowance now 20 mph (1569) |
| E2 | **horizontal slip angle** (1165) | corners with **elevation change** | it detects the slide the same on a slope as on the flat; before, the gradient inflated it | ⬜ |
| E3 | **chicanes** (1165) | a track with gradient and esses | chicanes are still chicanes (+10 mph); none gets classified by the elevation | ⬜ |
| E4 | **grid slots** (1165) | look at the cars on the start | none sunk or floating on tracks with gradient | ⬜ |
| E5 | **the wheel dither** | no longer has to be tested | **resolved** in 1163: it was the delay measured in 3D. The failed attempts are in `AGENTS-BACKLOG.md` | ✅ |
| E6 | **air offroad** | not implemented | the signal exists and is reliable (`ARS.WheelsOffGround`, 0-4 wheels reading `0x1C4`), but it only feeds the stability factor. Parked | ⬜ |

## F · The feedback batch (1294)

| # | What | How | Passes if | State |
|---|---|---|---|---|
| F1 | **build gate** | any normal race on 3889 | it **starts the same** as before: the gate only refuses when a signature is missing | ✅ **28-10 (1294)**: the race ran; the gate let it through |
| F2 | **no storm** | a whole race and `grep -c "OnTick error" ARS.log` | **0** | ✅ **28-10 (1294)**: 0 across 14 log lines (1.5 KB) |
| F3 | **tips vs board** | pass an apex with tips and the leaderboard visible | the tip box **does not cover** the `POS` header | ✅ «all good», 28-10 (1294) |
| F4 | **car name** | Interface > *Show Driver Names* off | the header says `CAR` and the model shows | ✅ «all good», 28-10 (1294) |
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
| H1 | **track refused** (1375) | Tracks → an XML with fewer than 6 route nodes | it does **not** say «Track instanced», no grid starts, the warning comes out | ✅ **verified 30-10 · 1380** with `Chianski Long only Five.xml`: `Start Race refused: That track has 5 route nodes`, with no `Track instanced`. Note: the door exercised was the **preflight**, which already worked; the one I fixed (Load Track in the menu) is still unreachable |
| H2 | **the abort leaves the ped standing** (1375) | go into freecam and then Insert (reload) | the ped **does not** hang without gravity; the next race starts normally | ✅ **verified 30-10 · 1380**: reloading in freecam no longer leaves the ped floating in ragdoll; the bug existed and can no longer be reproduced |
| H3 | **a failing background load speaks up** (1375) | start and read the log | a single `Initialization complete`, with no `OnTick error` | ✅ **verified 30-10 · 1380**: `build 1380`, `Initialization complete.`, and `grep -c "OnTick error"` = **0** |
| H4 | **the prune keeps comments** (1376) | put `; prueba = 1` inside `[MENU]` of `Menu-Race.ini` and reload | the comment **survives** the load | ❌ **fails 30-10 · 1380**: the line disappears. The prune does keep it, but `MenuSettings.Save()` rewrites the file as `[MENU]` plus the keys declared by the first `Set()`, and the load does half a dozen of those before you look. It is D-T2-3, archived; I verified the pass wrong myself: I checked the prune, not the file |
| H5 | **the prune says what it drops** (1376) | add `ClaveInventada = 1` in `[MENU]` and reload with `LogLevel = Error` | `ARS.log` carries `dropped stale` | ✅ **verified 30-10 · 1380**: `Settings repair: Menu-Race.ini dropped stale ClaveInventada`, with the default log level |
| H6 | **restart with no inherited card** (1377) | with a manoeuvre active (rival label visible), Restart Race | the new race **does not** drag the old card's throttle cap | ✅ **verified 30-10 · 1380**: «the plays disappear and are played in the new race normally» |
| H7 | **Restart after Load Track** (1378) | race → Load Track → Restart Race | it does **not** throw `ArgumentOutOfRangeException`; it starts or refuses cleanly | ⛔ **unreachable**: `Load Track` and `Build Grid` are only added to the menu with `StagedSpawns`, which the load forces to `false` (`:3432`). The hole is real and is closed, but today there is no door. For 0.11 / 0.9.1 |
| H8 | **the handbrake at launch** (1378) | standing start, watch the first metres | no car leaves nailed for 0.2–0.6 s; the pack leaves together | ✅ **verified 30-10 · 1380**: «everyone gets away fine» |
| H9 | **the lane on a gradient** (1379) | a stretch with elevation change, at speed | it behaves **the same** as on the flat; the change is < 1 cm and you should not feel it | ⬜ outstanding: has not found the stretch yet |
| H10 | **clamp after a slide + Restart** (1380) | slide, Restart Race, first corner | it turns like a new race, not with the wheel already shut | ✅ **verified 30-10 · 1380**: «I see no problem» |

### Waiting on your decision (deliberately untouched)

1. **`No Collision` is inverted** at all three call sites: the setting does nothing and its cleanup points the wrong way. `docs/audit/410-hunt-2026-10-30.md` → D-T4-1.
2. **«Open Up on Throttle»: withdrawn from the tree** (1389) — the row, the key and the method are gone; it never reached the wheel any more. The formula lives in the backlog (`open-up-inert`) in case it comes back. D-T6-1.
3. **`LaneMoverSpeedCapMps` = 99** where the comment and the design say 24: the whip cap cannot bite. D-T5-4.
4. **`avoid-hold-timeout`** (the test I owed you) lives behind `avoidHoldEnabled = false`: the test points at dead code. D-T5-2.
5. **ABS tooltip**: «I'm working on undoing that» on a row that does work. D-T7-1.
6. **`FEATURES.md` sells the player's nitro as «coming soon»** and the AI's as a percentage of yours: yours runs on X/duck, and the AI carries 5× against 1×. D-T8-1, D-T8-2.

## I · 2026-11-02 (1569)

| # | What | How | Passes if | State |
|---|---|---|---|---|
| I1 | **Projection brake floor** (`ProjectionBrakeFloorMph`) | go wide in a fast bend with `Show Gs Projection` on and read the `Offshoot:` log line | `floor=` moves with the bend — 20 in a tight corner, higher in a fast one — and the car **keeps rolling**: the projection never holds a full brake with the speed under its own floor | ⬜ |
| I2 | **`ars` cheat** | type `ars` in the console | the ARS menu opens; `arsmenu` no longer does | ⬜ |
| I3 | **lane timings: identity, two clocks, and no glide** | a lap | the absolute ends when the lane preview crosses the apex and the request **snaps** to the curvature ask (the 1 s and 3 s glides are gone); the next corner's preparation does not claim the car on the apex | ⬜ |
| I4 | **apex merge raised 5 s → 6 s** | load a track with a close pair of corners, read `Apex table: N corners` | the pair now merges into one corner (count drops by one) and the second no longer runs dynamic-only | ⬜ |
| I5 | **drive kept off track** (`OffTrackThrottleLevel` 0.35, now a **floor**) | put a car well past the edge at 40+ mph and let it go | off the track the pedal cannot be cut below the ramp by TCS/stability/plan: the car keeps rolling and only arms the reverse recovery if it is genuinely stopped | ⬜ |
| I6 | **projection is a speed offset** (`ProjectionSlowdownMphPerSecond` 2, floor 30) | run wide on the approach to a fast bend, `Show Gs Projection` on | the car slows **gradually** (2 mph/s on the target, then the pedal law brakes) instead of the old 0.25 s throttle drop + 0.8 brake, and it never goes under 30 mph or the bend's speed less 30; the `Offshoot:` log line now prints `drop=` and `floor=` | ⬜ |
| I7 | **route turn-in is a pure inset** (`LaneInsetPerRadius` 125, no speed factor) | a slow corner taken fast, and the same corner taken slowly | the aim sits the **same distance inside** in both (2.5 m at R=50, 1.25 at R=100, 5 m clamp under ~25 m); it no longer widens with speed | ⬜ |
| I8 | **a corner is forgotten at the lookahead** (`ApexAheadOfLookahead` gates both selection and forgetting) | drive out of a corner on a stretch with no corner the lane will work | the lane never asks for the **outside** after the exit and never re-arms on the corner behind it; it holds the field until the apex crosses the preview, then lets go | ⬜ |

## J · 2026-11-09 (1755) — the corner pipeline

Ten bench flags all on in this build (`CornerPlan`, `RouteCurvature`, `ChicaneBoost`, `ProjectionCut`,
`OffTrackThrottle`, `CrestGate`, `StabilityCap`, `GsLaneFloor`, `OverspeedCorrection`, `SlopeGrip`). Ini:
`CornerCeiling` 500, `CornerLineShare` 20, `CornerOffset` 0, `RouteOffset` 0, `BrakeAimDepth` 0.

| # | What | How | Passes if | State |
|---|---|---|---|---|
| J1 | **the prep cap hangs off where the prep started** (47e9efd) | the bend where the pack used to be thrown off the road | no sudden steer-out: the creep is a fixed distance from the offset the prep began at, not a lead re-based on the car every tick | ✅ **verified 09-11 · 1755**: «Oh yeah much smoother. The same point cars got whacked off the road by their sudden steer out, they dont anymore.» |
| J2 | **release at the apex, four seconds to position** (1e8e57c) | the same pass, on the exit | the held corner lets go at the apex and the dynamic lane takes over, no fight against the shaped exit | ✅ 09-11 · 1755, same pass as J1 |
| J3 | **full brake asks for a 4.5 m/s error** (fe71f53) | overspeed a bend by ~5 mph, then ~10 | the pedal stays off full while the error is under 4.5 m/s (≈10 mph); the learner band, the projection share and the ramp shift with it | ⬜ |

Open on this bench: the exit asks for nothing outward — the dynamic lane only ever aims inside, so the car
leaves a corner on the inside line unless `DefendLane` is working it (his ruling: the exit is fluid, default
outward). A span holding two corners of different size is refused at the 1.6 radius ratio, not split. Grapeseed
2968 sits at 1.97.

## K · 2026-11-09 (1768) — the corner is spent when the car coasts

| # | What | How | Passes if | State |
|---|---|---|---|---|
| K1 | **coasting inside the corner releases it early** (`CornerSpent`) | brake into a bend as usual, then roll through the entrance off both pedals | once past the entrance with `long G` under 0.05 the corner leaves the selection: the line stops shaping and the plan moves on, instead of holding it to the apex | ✅ **verified 09-11 · 1769/1770**: «the 0.05gs thing works well» |
| K2 | **a lift on the straight releases nothing** | coast a hundred metres before a bend, both pedals up | the corner stays selected and the shape still starts at the entrance four seconds out — the release is gated on being past the entrance | ⬜ |
| K3 | **the input gate is gone** | trail brake lightly into a bend | the pedals no longer matter: only the longitudinal Gs decide, so a light brake above 0.05 g keeps the corner selected | ⬜ |
| K4 | **the lane lets go on its own corner** | the shaped line through a bend taken off the pedals | the shape stops once the car is through the entrance instead of running to the apex: the lane tests the corner it holds, because the queue's head has already moved on by the time steering runs | ⬜ |
| K5 | **the creep is a clock, not an angle** (1770) | the S where a left corner is approached through a right-hander | the cap opens at 0.1 m and reaches 2 m three seconds after the corner starts being asked, so a counter-turning prep no longer gets the tightest creep for being perfectly placed | ⬜ |

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

