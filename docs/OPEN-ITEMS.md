# ARS — open items register

**What this is**: the loose ends, one dated section per run of the 16:00 cron (`e737d5394da0`). It records what is decided but not applied, what was raised and never decided, and which audit findings are still filed and unfixed. Finished work belongs in `AGENTS-WIP.md` and `AGENTS-STEERING.md`, not here.

**Read this when**: starting a session, and you need to know what the last run left open before picking work.

## 2026-10-24 — run 1 · commits `648d9b1` (00:02) … `66e4f41` (16:01), 90 in the day

**Decided and applied today**
- The corner lift climbs at 10 mph/s, down from 20: `648d9b1`, `b1f6185`.
- The kick card is off, and every nearby rival names the card it is playing: `fcab863`.
- Card exits are looser than entries — dwell floor plus cooldown — a card holds a second and rests a second, and both fight cards get a 2–6 s window: `ac29d42`, `8317e65`, `b628e92`, `c3b4be2`.
- DefendLane arms 3–5 s out on a rival 3–6 s behind, reads only the chaser's own time-to-reach (mirrored when absent), and releases when the chaser stops coming: `20458c0`, `3627787`, `e87eade`, `5bdf1c6`.
- DiveBomb is the last option, considered 3–5 s out with its arm on its release band, keeps its braking point, asks who arrives first within 0.75 s, logs its lifecycle, and no card arms against a target already playing it: `e9fcf97`, `2331d21`, `8223968`, `652123f`, `265529a`, `3e94aaa`, `c05f0f4`.
- The rival lift is a 3 m ramp to half throttle, its remap ascends, and the rule is written down: `0465bb1`, `e46664a`, `d154410`.
- The damper countersteers 5° and no further, behind a 0–10 knob in twos, and the ceiling is documented as a slide-propensity dial: `e8b66e7`, `1b3f9e7`, `18b67f8`.
- The slide band reads the tyre peak at the current speed, opens at the peak and closes 1.5 peaks past it; the assist ceiling is a share of the slide, grows on the blend's gate and resets on reversal alone: `4b61a41`, `aa8c97c`, `1331fda`, `9e66b9b`, `07e21c0`.
- A sliding car gets 10°/s of extra countersteer, dropped on reversal, and both pedals cap at half instead of cutting: `af305c5`, `4924f50`.
- Stability is a per-wheel rate — down hard, back slowly — emptying in 0.5 s and refilling in 2 s, with a lost wheel costing 1/s of throttle cap: `a775a8a`, `05ea488`, `d97a566`.
- Terrain speed cuts apply to the route alone and never to a corner plan, behind a temporary flag; the projection brakes to half and stops at the curvature speed; the brake learner is scored on arrival speed: `ce7212b`, `bc0f794`, `a73cdd3`, `efebb8d`, `a7e0e95`.
- The apex line is metres of route: a 250 m outside plateau, a ramp from 50 m to 5 m out, the raw half width five nodes early, a latch once the demand is inside the car, held to the apex and following the nearest unpassed apex — and only corners under `IdealLineMaxRadiusMeters` 50 m ask for it: `ce895b2`, `51d8bff`, `fb15d89`, `57e97c7`, `3c16023`, `ab30b8c`, `f6bd315`.
- The lean stands down only while the line is active, it yields to a corner asking for the outside, and the line owns the lane whenever a tight corner is ahead: `d78cbb4`, `f6cf63b`, `8ec7c43`.
- The lane mover is sized from the traversal and drops momentum on reversal, at 26 m/s² and 24 m/s: `7bd27d2`, `5635848`.
- The inside guard is re-armed each pass and logs when it holds; four audit fixes land (wall inset, metres from the car, card window, latch reset); the avoidance hold times out and does not survive a re-init: `837c4a7`, `4e8fb3d`, `5ad52c1`.
- The positioning-requirement machinery is deleted: `18d5a39`.
- Debug lane heights step by a quarter of a decimetre, every colour on its own height: `4482553`, `8f45a99`.
- ABS: the wheel-lock reading is published and the guard logged, the guard becomes a proportional taper on slip ratio (knee 0.5, zero at lock, no glide), and the switch then goes off and greyed because the engine re-grants `WF_ABS` to every AI car: `870a601`, `3cf74bc`, `66e4f41`.
- Docs: the traction-curve pair scales about 11°/G, the `STATUS_PLAYER`/`STATUS_PHYSICS` table is recorded site by site, the register opens, the session closes at build 880: `8484dd4`, `dd820ce`, `4f898ff`, `172b19f`.

**Decided but not applied**
- `outside-leg-half-rate` — the outside leg runs at half the mover rate (13 m/s², 12 m/s), compile-verified in the deployed build and not driven; drive a corner that asks for the outside and watch the lane settle without lingering (`650cab9`, AGENTS-WIP.md:184).
- `avoid-hold-timeout` — the 8 s `AvoidHoldTimeoutMs` backstop (`Racer.cs:1281`) is compile-verified and not driven; hold a rival alongside past the timeout and check the allow reopens (`5ad52c1`, AGENTS-WIP.md:185).
- `abs-taper` — the taper is behind a forced-off, greyed `AbsEnabled`; not driven in either shape, and it returns with one line (`3cf74bc`, `66e4f41`, AGENTS-TECHNOTES.md:119).
- Card batch — today's defend/dive windows and the dwell/cooldown pair carry no drive recorded, and the divebomb drive owed stands (AGENTS-BACKLOG.md:184).
- `stability-speed-scaling` — recorded, not started; the first decision is which law, the speed-scaled fall or the 100–150 ms wheel-off debounce (`52cc47b`, AGENTS-BACKLOG.md:78).
- `personality` — the spec is written and nothing is built (AGENTS-PERSONALITY.md, `b52b1b4`).
- Older drives still owed, each with its TEST-PLAN section: `merge-span-gap` (625), `feature-entrance-pull` (727), `overspeed-thresholds-setting` (616), `gravity-grip` (613), the creator's route-table rebuild (624), the fleet-wide `apex-radius` read (AGENTS-BACKLOG.md).
- `pi-controls-merge` — DECIDED, not built: one typed row writing two keys behind it (AGENTS-BACKLOG.md:133).
- `weaponized-grid-filter` — DECIDED no code; the manual edit to `Dist\AutosportRacingSystem\Vehicles\cars.txt` is still pending (AGENTS-BACKLOG.md:23).
- `player-nitrous-fire` — parked; reopen only from a run at Log Level `Info`, where no `Player nitrous fired on lap N` line means the X path is never reached (AGENTS-BACKLOG.md:143).

**Not decided**
- How to beat the engine's forced ABS on AI cars: `Automobile.cpp:3775` re-grants `WF_ABS` every tick to a car in `STATUS_PHYSICS`, so clearing the model bits changes nothing on an AI car and the menu now says so outright (AGENTS-VEHICLE-STATUS.md, `66e4f41`). Settle it by deciding whether ARS writes `STATUS_PLAYER` into the AI vehicle's status word — which also flips damage thresholds, population culling and collision response — or accepts a player-only ABS.
- `slide-brake-rampdown` — what the brake reduction is a share of: the plan's brake, or the cap the ABS guard leaves (AGENTS-BACKLOG.md:54). One line settles it; the shape stays instant in, ramped out.
- `track-node-editor` — how much re-derives from a moved node: the whole section, or the two halves either side of it (AGENTS-BACKLOG.md:99).
- `feature-entrance-pull` — the 100-node pull is provisional and the 40-node apex cap binds on nearly every corner, so the entrance lands 40 nodes before the apex wherever a feature qualifies; settle whether the pull stays or `SpanMaxDistanceNodes` moves (AGENTS-BACKLOG.md:91).
- The aftermath of agenda A: a bend before a chicane now registers as its own corner (AGENTS-BACKLOG.md:130). Read that track's `Apex table:` line and decide corner or merge case.
- `LaneCrossingScale` — the crossing scale is off while a comparison is owed (AGENTS-STEERING.md:670); drive one corner with the scale on and off at the same entry speed and compare the lane's settling.
- Recorded ideas with no decision, each needing a yes/no before it can be ranked: `manual-apex-placement`, `creator-apex-preview`, `surface-stats-revival`, `section-roughness-sweep`, `launch-slip-allowance` (AGENTS-BACKLOG.md).

**Filed findings, not fixed**
- `rival-walls-500ms-snapshot` — the wall corridor keys off rival fields refreshed only on the 500 ms beat while the walls run every core tick, so the outside request stays clipped for up to half a second after a pass. Falsify by correlating each `asked X (WallClamp)` second against the nearest rival's `RouteGapAhead`/`NodeOffset`: a constrained corridor with a negative or over-30 m gap is the signature (`d3aa239`, AGENTS-BACKLOG.md:58).
- `track-point-resnap` — a whole-route re-scan with no heading test, so one mis-snap on an overlapping leg makes the approached corner read as behind and the picker jumps to another corner, possibly a lap away. Falsify by watching `d=` step from tens to hundreds of metres with another corner's radius in the same second (`d3aa239`, AGENTS-BACKLOG.md:66).
- `lane-two-frames` — `ownLane` is the car frame's node offset and `steerRefOffset` the reference frame's, ~2.3 m of sagitta at R 50 against a 0.1 m release band, so the inside latch and the mover guard can disagree tick to tick. Falsify with `Lane guard ... held at X, request was Y` where `Y − X` is 1–2 m and its sign flips at constant steer (`d3aa239`, AGENTS-BACKLOG.md:72).
- The pedal law is proportional on speed error with no deceleration term (`FullPedalSpeedErrorMps` 3, ×2 for brake, `Racer.cs:1867`): a car 1 m/s over at 200 m out and one 1 m/s over at 10 m out get the same pedal. Falsify by logging the speed error and the pedal at both distances in one braking zone (AGENTS-WIP.md:148).
- The curvature term has no time dimension while the corner does, so its target can step 12.84 km/h in one node — 59 % of the brake range in about 26 ms. Falsify by logging the route term per node against the corner plan's ramp (AGENTS-WIP.md:149).
- `999` is the sentinel for both "huge radius" and "degenerate solve", so the route can be blind in a curve and report a number that looks like a straight. Falsify by reading the route radius through a genuinely tight curve and at the viaduct (AGENTS-WIP.md:150).
- `grip-native-vs-lateral-g` — the grip native has never been checked against measured lateral g, and every grip law scales off it. Falsify by logging both at a steady-state limit corner (AGENTS-BACKLOG.md:82).
- The traction pair is unverified across the shipped fleet: `fTractionCurveMax` and `fTractionCurveLateral` must sit on the ~11°/G line, and a car off it invalidates the slide band, `SlidingFraction`, the steer ceiling and the ABS taper (`8484dd4`, AGENTS-TECHNOTES.md:109). Falsify by dumping the pair for every car in `cars.txt` and plotting it against the line.
- `AGENTS-TECHNOTES.md:126` still describes the ABS guard as gliding at `AbsBrakeFallPerSecond` 0.2/s over four seconds and its log line without `slip=`; `3cf74bc` deleted the glide and added the field. Falsify with `grep -r AbsBrakeFallPerSecond src` — it returns nothing.
- `AGENTS-WIP.md:151` still lists the brake learner's pedal-share defect as open; `a7e0e95` replaced that verdict with arrival speed. Falsify by reading `Racer.cs`'s brake learner for an arrival-speed comparison.
