# AUDIT 200 — ARS 0.9.0: the audit, ranked

**Surface**: the whole 0.9.0 audit — the eleven hunt reports of this pass (`201`–`211`) and the verifier
(`212`) that re-checked every blocker in the code. Each finding below names the report that owns it.

**Date**: 2026-10-25, late session. The live log read for this report ends at 23:50:47.

**Build audited**: **1092**. `build/build-number.txt` = `1092`; `bin/Release/ARS.dll` and the live
`Scripts\AutosportRacingSystem\ARS.dll` are one file, sha256
`8e3ab22dc3b3ff8a1dda4dfd3cd8a7f5c8017f36206032ffa3210dea6c983285`; source `HEAD e851a09`, `src/` clean.
⚠️ **1092 is a local development axis, not the release identity**: the workflow passes
`github.run_number` (`deploy.yml:19`) and the last run is #276, so the tag lands at **≈277**. See `202`
§"Two premises in the card".

**Method**: read-only synthesis. Nothing was built, edited, deployed, committed or driven; the only file
written is this one. **Every blocker row was re-read in the source today**, at the line quoted — the
numbers here are mine at `e851a09`, not copied from `212`. One artefact is new since `212`: the live
`Log.log` was re-read at 23:52 and it carries **208** exceptions on this dll (B1).

**Status**: **VERIFIED** = read in the code today, or reproduced read-only (arithmetic, replica,
decompile), or **observed in the live log**. **VERIFIED (2xx)** = the report read it and this pass did
not re-open it. **PROBABLE** = the mechanism is read and one step rests on engine, SHVDN or drive
behaviour not observed. **NOT ESTABLISHED** = the artefact exists and was not read here. **NOT
VERIFIABLE** = the artefact that would settle it is gone. **Nothing in this audit is driver-verified.**

## How to read this in five minutes

- **§1** is the ten blockers, ordered by what shipping them costs. B1 is not a prediction — it is firing
  about 28 times a minute on the installed build.
- **§2** is what to know before the tag, but what does not stop a race: fourteen rows a release decision
  actually touches.
- **§3** is what looked like a blocker and is not, one line each — including eight closures that hold and
  four doc blocks that do not belong on the gate.
- **§4** is what this audit never looked at. Read it before treating anything above as coverage.

---

## 1. Blockers — ten, ordered by the cost of leaving them

The order is not `212`'s. It ranks by **exposure × silence × what the player loses when it lands**: the
exception that is already firing comes first, and a defect that is wrong on every lap of a bundled track
outranks one that ends a race only in a narrow window.

| # | Cost if shipped | Claim | Evidence at `e851a09` | Status | How to reproduce |
|---|---|---|---|---|---|
| **B1**<br>`08-B1` | **Already firing.** ~28 abandoned frames a minute, every session, and the record says the defect is closed. | A non-finite `SlideAngle` reaches `Math.Sign` in `ComputeSteering`; the only NaN test sits **after** both calls, so it cannot help. | `Racer.cs:910` (`slideDeg = VehicleData.SlideAngle`), `:912` and `:922` (the two `Math.Sign(slideDeg)`), `:931-932` (the `IsNaN(Control.SteerDegrees)` test — after the throw); the named producer is gated at `:3876-3878`. Live `Log.log`: **208** identical blocks `System.ArithmeticException … at System.Math.Sign … at ARS.Racer.ComputeSteering`, 23:43:26 → 23:50:47 (7 min 21 s), on this dll. Each throw unwinds out of `RunTimedCore` into `OnTick`'s catch (`AutosportRacingSystem.cs:2243`, `:2326-2328`) — the rest of the frame is skipped and the batch cursor does not advance. | **VERIFIED (code) + OBSERVED (208 in the live log)**. The producer of the non-finite value is **not identified**: finite `Velocity` with a non-finite forward or up vector is the surviving path (`202`, "What this pass could not establish", 2 — PROBABLE). | Leave ARS running ten minutes at the shipped `LogLevel = Error` and `grep -c "OnTick error" Log.log`. Compare `202`'s closure ledger, which records it closed. |
| **B2**<br>`02-B2` = `207-B1` | The first race started after finishing one can die **at its own lights**: the commonest menu sequence in the mod. | A race started from the Race menu inherits the previous race's grace deadline and ends itself on it. | `AutosportRacingSystem.cs:320` (the field), `:2275` (armed at the first finisher), `:2301-2302` (the test — outside every `RaceStatus` gate), `:905` and `:1831` (the only two resets). `CleanRacers` (`:2550-2565`) and `SetupRace` (`:2592-2640`) reset neither. | VERIFIED | Run to the first AI crossing, then Race → Start Race before the "Ns to end the race" line expires: the new race dies when the **old** number reaches zero, sometimes before its countdown ends. |
| **B3**<br>`07-T3` = `209-B2` | **Every lap** of a bundled track: nitrous silenced for the whole circuit and one braking plan armed past its own apex, silently. | The width walk subtracts an **absolute** node from a **rotated scan list's** absolute node, so a region containing node 0 walks its entrance to position 0. | `TrackLoader.cs:869` (the walk), rotation `:371-391`, publish `:899-901` (`StartNode = scanNodes[startPosition]`). Consumers: `Racer.cs:3223-3243` (`IsWithinCorner` wrap branch), `:3279` (in-corner nitrous veto), `:4374` (braking entrance). Replica on the shipped file (`209`): El Burro Rally `apex=0 … scanStart=37 … StartNode=37 … LengthStart=1664`, which makes `IsWithinCorner` true for **1696 of 1701** nodes. | VERIFIED (code + arithmetic). The shipped-track reachability rests on `209`'s calibrated replica, not on a lap. | Load El Burro Rally at `LogLevel=Info`: the first `Corner add` reports `start=37` with the apex at node 0. |
| **B4**<br>`04-W2` = `206-02` | The walls vanish without a word **exactly when there is a car on each side**. | When the clamp-space corridor inverts, `ClampLaneToWalls` discards both walls and returns the whole road; the ordering guard runs on the targets, where no inset exists. | `Racer.cs:1681-1683` (inset per constrained side, then `if (clampLeft > clampRight) return Clamp(lane, -trackBound, trackBound)`), guard `:1756-1762`, targets `:1736`/`:1741`. Inversion below **4.20 m** separation at Aggression 100, **6.20 m** at Aggression 0. | VERIFIED (code); the 4.20 m / 6.20 m thresholds are `206`'s arithmetic. | Two rivals one each side at ±2.1 m, Show Rivalry on: both walls draw red while the request crosses between them unclamped. |
| **B5**<br>`04-W1` = `206-01` | The wall gate can miss the alongside tick entirely — the one tick the wall exists for. | The record the gates decide from is **250-749 ms** old, and at 10 m/s closure the tested gap moves 2.5-7.5 m per beat against a 10.05 m window. | `Racer.cs:104` (`RivalRefreshMs = 250`), `:559` (`_phaseOffsetMs = GetRandomInt(0, 500)`), `:4698-4701` (the beat is `250 + phase`), `:5279` (`UpdateRivals` republishes), `:4784-4788` (the redundant 500 ms path); window `DataStructures.cs:195-197`. | VERIFIED (code). `201`'s 500 ms is wrong; `206`'s period is the code's — the 2.5-7.5 m step is its arithmetic. | Log `r.RouteGapAhead` beside a live recomputation at 60 Hz: the published value steps on the beat. |
| **B6**<br>`07-T2` = `209-B1` | A **black screen** and a lost route, on a track the player just saved, from two clicks. | The creator accepts a 2-node route and the loader indexes `[2]`, `[3]`, `[5]` — after the world has been torn down. | `AutosportRacingSystem.TrackCreator.cs:48` (`< 2` is the only guard), `:230-234` (one Attack click adds exactly 2 nodes); `TrackLoader.cs:152`, `:156-157`, `:166`; fade-out before the throw `TrackLoader.cs:30-41`; `InstanceTrack` `AutosportRacingSystem.cs:1502-1523`, catch `:2326-2328`; `DensifyRoute` returns below 4 nodes `TrackLoader.cs:504-507`. | VERIFIED (code; the throw is arithmetic) | Creator → one Attack (start line) → Save Track → Load Track: `ArgumentOutOfRangeException`, screen stays faded. |
| **B7**<br>`05-P2` | The player's own car leaves the race with GTA's nitrous **armed**, and keeps it for the session — this one changes the car outside the race. | Ending a race during a burn leaves `SET_OVERRIDE_NITROUS_LEVEL` set, and teardown deliberately keeps the player's car. | `Racer.cs:3460-3461` (charge + override), `:3496-3499` (`StopNitrous`, the only clear), `:3241` (returns when `NitrousEnabled` is off), `:3243-3244` (returns while the burn is live; the only caller), `:5282-5299` (`Delete` keeps a player-owned car); `AutosportRacingSystem.cs:1820` and `:921` (the teardowns); `Initialize` `:642-699` resets no nitro state. | VERIFIED (state). The *thrust* an armed bottle then permits is `205-02`'s question, not this one. | Nitrous on, player on the grid, fire, **End Race inside the 3 s window**: the car leaves the race with the system armed. |
| **B8**<br>`03-G2` | One blocked approach ruins that corner **for the rest of the race**; recovery is 14 clean arrivals. | A hot entry floors a corner's brake factor; the cut is uncapped per event and the clamp is applied afterwards. | `Racer.cs:2506` (`entryCut = 0.2 × max(0, excess − 0.894)/2.2352`), `:2526` (clamp to `[0.5, 1.0]`), seed `0.75 ± 0.05` (`:341`, `:352`), recovery `0.025` per clean arrival (`:373`). 5 m/s over = **0.367** — the floor from the seed. | VERIFIED (code + arithmetic) | Brake Learning + Brake Learn Notify on; one approach 11 mph hot and the notice prints `0.50` and stays there. |
| **B9**<br>`12-B1` | Silent data loss: a torn settings file is **accepted**, and the only symptom is a setting reverting. | Settings are written truncate-then-rewrite and the result is discarded; the atomic writer the repo already owns has one caller, the repair pass. | `MenuSettings.cs:73-78` (`SeedIfMissing`) and `:80-86` (`Set`) both `File.SetValue(...); File.Save();` with the result discarded; `ScriptSettings.Save` is `File.CreateText` then a line-by-line write (decompiled from `libs/ScriptHookVDotNet2.dll`); `WriteAtomic` callers: `SettingsRepair.cs:237` only, defined `:367-379`; a torn value that is a ladder rung is returned unchanged and unlogged (`SettingsRepair.cs:79`). | VERIFIED (code); the `ScriptSettings.Save` decompile is `212`'s. | Make `Menu-Settings.ini` read-only and change any row: the menu shows the new value, the file keeps the old one, nothing is logged. |
| **B10**<br>`13-B2` | A **crash on the reload key**. The player never presses it; the dev loop presses it constantly — and it is one decision with `207-B2`. | Reload/abort while ARS is executing is a crash path, and the record calls reload free. | `AutosportRacingSystem.cs:3461-3498` (`OnAbort`, no `try`), `:2814` (`Script.Yield()` inside the roster's enum walk); `AGENTS.md:65` states the reload is free. Dump `ceb5b698-dee0-479e-98f8-bcc9edfa6869.dmp`: `System.ObjectDisposedException`, stack through `ScriptHookVDotNet!Unknown` and **`ARS!Unknown+0x35c1`**. | VERIFIED (dump); that ten `Insert` presses reproduce it is PROBABLE | `cdbX64 -z <dump> -y "" -c ".ecxr; k; q"`; in game, press `Insert` mid-race ten times and count the launches that die with no ARS line. |

**Counts**: 10 blockers, of which seven are player-facing (B2-B8), one is the live exception (B1), one
corrupts a save (B9), one is a crash on a dev key (B10). `212` counted the same ten; the order here is the
only thing that differs.

---

## 2. Release notes — fourteen rows a release decision touches

True as shipped; each needs a decision or a line in the notes; none of them can cost a race on its own.
The four reports that declared no blocker alone carry **44** release-note and file rows (`203` 10,
`204` 10, `208` 10, `210` 14); the fourteen below are the ones where a release decision changes
something.

| # | Sev | Claim | Evidence | Status | How to reproduce |
|---|---|---|---|---|---|
| R1<br>`10-B2` | release-note | The release body states the stability cut at twice the code's rate, and asymmetrically where the code is symmetric. | `docs/RELEASE-NOTES.md:15` ("closes a **quarter** of the throttle per wheel … gives it back at **half the rate**") against `Racer.cs:2864-2865` (`0.5f` / `0.5f`, applied `:2920`/`:2922`). | VERIFIED (this pass) | `grep -n "StabilityFallPerWheelPerSecond\|StabilityRisePerSecond" src/Racer.cs docs/RELEASE-NOTES.md` |
| R2<br>`211-B5` | release-note | The release body sells the nitrous pair as a **measured 1:2.5**; the code is 5 against 1, and parity is at `AiNitroPct` **50**. | `docs/release-0.9.0.md:33` ("1:2.5 as measured on track"); `Racer.cs:443` (`NitrousPowerMultiplier = 5f`), `:447` (`PlayerNitrousPowerMultiplier = 1f`), applied `:3984-3985`; the comment above them still says "2 for the player". | VERIFIED (this pass) | `grep -n "1:2.5" docs/release-0.9.0.md src/Racer.cs` |
| R3<br>`205-02` | release-note | **The player's burn carries no drive force of its own**: it is a charge plus a HUD switch, and the force belongs to GTA's native nitro, whose trigger ARS never pulls. | `Racer.cs:3457-3471` (charge + override only), `:3985`/`:4011` (`SET_VEHICLE_CHEAT_POWER_INCREASE(car, 1.0)` — ×1.0 is a no-op); the native force needs `bNitrous`, set only by `CVehicle::ProcessNitrousControl` on a ≤250 ms release of `INPUT_VEH_TRANSFORM` (`Vehicle.cpp:37239-37251`); nothing in `src/` calls `SET_NITROUS_IS_ACTIVE`. | VERIFIED (mechanism); whether keyboard `X` is also bound to `INPUT_VEH_TRANSFORM` is **NOT ESTABLISHED** — if it is, the keyboard bottle works by accident and a pad's never does. | `LogLevel = Info`, fire from ~40 mph on a straight: the `Burn … (player), power x1.00` line prints every 500 ms and the speed stays flat for the 3 s. |
| R4<br>`207-R3` | release-note | The final classification is computed and **thrown away in the same tick**; the purse is paid with no message. | `AutosportRacingSystem.cs:2304-2308` (stamp, reward, `Finished`, then `CleanEverything`), `:1838-1839`, `:1820`, `:2117` (the HUD gate), `:2643` (the only prize line). | VERIFIED (2xx) | Finish a race: the board disappears the instant the winner is decided, the balance rises, nothing says where you finished. |
| R5<br>`207-R1` | release-note | The player's racer is bound to **one vehicle for the life of the race**; nothing watches the player die, respawn, leave or change car. | `Racer.cs:24`, `:551` (the only assignment in 5304 lines), `:4780-4782` (`UpdateDNFCheck` inside `!ControlledByPlayer`); `AutosportRacingSystem.cs:1923` (re-resolves the same object). | VERIFIED (2xx; `grep -rn "IsDead\|IsWasted" src/` empty) | Race, get out and jack another car: the board keeps the old car's position and the lap time stops moving. |
| R6<br>`206-03` | release-note | "Which side is he on" is read in **this car's body frame** while the wall is placed in the **track frame**, so a car directly ahead in the same lane reads metres to one side — and the follower is pushed to the **outside of every corner**. | `DataStructures.cs:192-194` (`RelativeOffset.X`), `AutosportRacingSystem.cs:608-611`; the wall's position from `r.NodeOffset` (`Racer.cs:1736`, `:1741`); corridor gate `DataStructures.cs:265`. Sagitta `d²/2R`: 4.00 m at 20 m ahead on R=50. | VERIFIED (`206`: code + arithmetic); the consequence needs a drive (PROBABLE) | A rival within the 3 s approach window in the same lane: `LaneReason` reads `WallClamp` and the pass runs on the outside on both hands. |
| R7<br>`207-B2` | release-note | A reload mid-race **resumes** the previous race on a half-reset state, and that race can no longer be timed out. | Statics `AutosportRacingSystem.cs:149`, `:243`, `:251`, `:256`, `:305`, `:308`; instance `:320`, `:1845-1846`; `OnAbort` `:3461-3498` clears racers and props but none of `RaceStatus`, `LeaderboardFinish`, `PlayerParticipating` or the route. | **PROBABLE**. `207` read a live log at `Info` (racers acting with no spawn line); that log is gone with the truncation at `:396`, and **today's log cannot replace it** — the live `Menu-Settings.ini:27` is `LogLevel = Error`, so no spawn line would print either way. | Reload mid-race and read `Log.log` for `Card`/`Kick` lines with no `Loading Racer` line. Needs the level at `Info` first. |
| R8<br>`208-R1` + `208-R2` | release-note | **Two migrations that never run**, so an upgrader's settings silently change: `CrestEffect` 50 becomes 0.4 g, and `AiNitro = Never` becomes `NitrousEnabled = True`. | `AutosportRacingSystem.cs:3607` (the seeding getter writes the key before the legacy read at `:3613`; `MenuSettings.cs:99-103` then refuses), `:3614` (the `> 0` guard skips a stored 0); `:3583` (the only `AiNitro` read), `:3542` (the prune runs first), `SettingsRepair.cs:344` (`Settings.ini` deleted), `git show e950506`. | VERIFIED (2xx) | Set `CrestEffect = 50`, delete `MaxCrestLift`, load: the file comes back at 0.4 with no repair line. |
| R9<br>`208-R4` | release-note | A value the repair **snaps to the ladder's first rung** reaches the file but never the session — the menu reseeds with `SelectedIndex`, which fires nothing when the index is already 0. | Statics read `AutosportRacingSystem.cs:3589-3648`, repair `:3653`, menu built after; LemonUI 1.5.2 `SelectedIndex` setter is `if (index != value) { … TriggerEvent(…) }` (decompiled); two items have since been re-seeded by hand (`:1245-1246`, `:1381-1382`). | VERIFIED (2xx) | Set `CornerOffset = -100` and load: the log says `-10`, the row shows `-10`, and the rivals brake with −100 for the session until the row is touched. |
| R10<br>`204-01` | release-note | A car that comes to a **dead stop** loses its forward throttle within half a second: the stability cap has no floor and the only branch that raises it needs the car moving. | `AutosportRacingSystem.cs:3304-3313` (a wheel is airborne when its side-slip field is exactly `0f`), `wheel.cpp:5303-5309` (the engine assigns exactly `0.0f` at ≤0.1 m/s); `Racer.cs:2918-2923` (fall 2.0/s, rise only when no wheel reads zero), `:3044`, `:4928` (recovery starts a stopped car in `Reverse`). | VERIFIED (`204`: engine + code); PROBABLE in game | Stop an AI car dead with Show Inputs on: `MaxThrottleFromStability` walks to 0 in ~0.5 s and stays there until the reverse phase moves the car. |
| R11<br>`205-04` | release-note | **A fire during the countdown is free**, so the player gets two bottles in the first lap. | The gate is `Lap <= _nitrousLapUsed` (`Racer.cs:3478-3481`); `Initialize` sets `Lap = 0`, `_nitrousLapUsed = -1` (`:677-679`); the racer exists before `RaceStatus = Countdown` (`AutosportRacingSystem.cs:2526`, `:2657`) and the fire check is not gated on race state (`:2113-2115`). The AI's identical gate is protected by a 6 s arm delay (`Racer.cs:3260`). | VERIFIED (2xx) | Press X at the countdown — the log reads "fired on lap 0" — then cross the line and press again. |
| R12<br>`210-P1` | release-note | The leaderboard is rebuilt and redrawn **every frame**, and it is the largest cost on the surface: 672 native dispatches and ~1,092 heap allocations per frame at 20 cars. | `AutosportRacingSystem.cs:2117`, `:2356`, `:2380-2448`, `:2425`, `:2281-2293` (the 200 ms ranking pass), `:3828-3838` (8 natives per `DrawText`); 4 header + 80 rows = 84 draws × 8 = 672, × 13 arguments = 1,092 allocations — **40,320 natives/s, ~1.5 MB/s** at 60 fps. | VERIFIED (2xx; counted from the source) | Breakpoint at `:3830` with 20 cars: 84 hits per frame. A 5 Hz rebuild removes ~98 % of it and no information. |
| R13<br>`209-R10` | release-note | The shipped package carries **three tracks twice** and the dev's own working files: `Dist` holds `NURBGP.xml`, `Oct1.xml` and `TestOct.xml` at the root and again under `testing\`, which the discovery walk lists. | `ls Dist/AutosportRacingSystem/Tracks` today: 18 `.xml` at the root, `testing/` holding `NURBGP.xml`, `Oct1.xml` and `TestOct.xml` a second time, an untracked `F8.xml` (the dev's own track), a `Figureight.xml.bak`, and `Figureight.xml` modified; discovery walks subfolders `AutosportRacingSystem.cs:465-467` and shows only the bare name `:1642`; the workflow copies `Dist\AutosportRacingSystem\*` **verbatim** (`deploy.yml:27`). | VERIFIED (this pass) | Pack from the working tree and open the Track list: three pairs of identical names, plus the dev's `F8`. |
| R14<br>`211-R1` | release-note | The **injected** root memory names a steer ceiling of 1.00 with a menu top of 1.25; the build ships **0.90** and the ladder reaches **2.00**, where the sentence's own conclusion ("never bounds the slip the tyre does") stops holding. | `AGENTS.md:132`; `AutosportRacingSystem.cs:180` (`= 0.9f`), `SettingsRepair.cs:160` (default `"0.90"`, ladder `0.05 … 2.00`), `Racer.cs:2034` (ceiling = peak × 4/3). | VERIFIED (this pass) | `grep -n "SteerCeilingFactor" src/AutosportRacingSystem.cs src/SettingsRepair.cs AGENTS.md` |

---

## 3. What looked like a blocker and is not

### 3.1 Eight closures that hold (`212` §2)

| Closed | By | What holds |
|---|---|---|
| `02-B1` player's own car deleted at teardown | `bd55c24` | `Racer.cs:5282-5299` — the branch is `if (ControlledByPlayer \|\| Game.Player.Character.IsInVehicle(Car))`; only the `else` deletes. |
| `05-P1` player's ped left with proofs and 1000 HP | `bd55c24` | `Racer.cs:629-638` — the proofs/health block sits inside `if (!Driver.IsPlayer)`. |
| `05-P5` `SlideAngle` 90° at rest | `bd55c24` | `Racer.cs:3876-3878` — `Velocity.LengthSquared() > 0.01f ? … : 0f`. This is the producer B1's record blamed; gating it did not close B1. |
| `07-T1` one unparseable file stopped startup | `bd55c24` | `AutosportRacingSystem.cs:472-490` — `*.xml` filter and a per-file `try`/`catch` logging `Track skipped`. |
| `10-B1` release note sold ABS as shipped | `9757f09` | `docs/RELEASE-NOTES.md:15` — "shipped **off** and greyed"; `AutosportRacingSystem.cs:3602` forces `AbsEnabled = false`. |
| `10-F4` `AbsEnabled` declared `True` while forced false | `9757f09` | `SettingsRepair.cs:150` — `KeySpec("AbsEnabled", "False", Kind.Bool)`. |
| `13-B1` wheel-pointer miss became an unbounded dereference | `9757f09` | `AutosportRacingSystem.cs:3129-3135` (miss → forced `Error`, `return 0`), `:3168` (`wheelPtr == 0 \|\| numWheels <= 0 \|\| numWheels > 8`). |
| `08-M2` `ShowRivalry` alone drew nothing | `13599db` | `AutosportRacingSystem.cs:2257` — the focus scan lists `Options.ShowRivalry`. |

### 3.2 Nine rows filed as blockers that are real at a lower grade

| Row | Filed | Now | One line |
|---|---|---|---|
| `03-G1` | blocker | **release-note** (§2 has no row; see `201`) | The route brake cap really is inert after the first armed corner (`Racer.cs:4421` writes `RequiresBraking` once; the cap at `:2729` is skipped while it is set) — but the consequence is a full-authority brake on a curvature step: a slam, not a lost race, a crash or a lost value. |
| `05-P3` | blocker | release-note | The apex and braking solves are built from the grip at the car (`Racer.cs:4644`, `:4653`, `:4616`), but they are re-solved every tick (`:4530-4538`), so the error is a lag that converges as the car slows — not an open-loop plan. |
| `207-B2` | blocker | release-note (R7) | It needs the reload key, which the player never presses, and it is the same keypress as the crash in B10 — one decision, not two fixes. Evidence stays `AutosportRacingSystem.cs:3461-3498`. |
| `10-B2` | blocker | release-note (R1) | A sentence in the release body (`docs/RELEASE-NOTES.md:15` against `Racer.cs:2864-2865`), not code. |
| `211-B5` | blocker | release-note (R2) | A sentence in the release body (`docs/release-0.9.0.md:33` against `Racer.cs:443`, `:447`), not code. |
| `211-B1` | blocker | **file** | `AGENTS/AGENTS-STEERING.md:183`, `:361`, `:660`, `:686` and `TEST-PLAN.md:498` say the lane mover runs at 14 m/s² with a 12 m/s cap; `Racer.cs:1102-1103` says 26 and 24, and the same doc says 26/24 at `:388`/`:676`. |
| `211-B2` | blocker | **file** | Five symbols the tree does not contain (`LeadBounded`, `LaneInwardLeadMeters`, `LaneOutwardLeadMeters`, `LaneInwardLeadTaperMeters`, `IdealLineOffset`) and a "no lateral leash" claim the live leash at `Racer.cs:1110-1114`, `:1201-1202` contradicts. |
| `211-B3` | blocker | **file** | `DamperCrossingMaxDeg` has 0 hits; the cap is `ARS.DamperCountersteerMaxDeg` = 2° (`Racer.cs:883-889`, `AutosportRacingSystem.cs:179`), and `AGENTS/AGENTS-STEERING.md:71` states both the cap and its removal. |
| `211-B4` | blocker | **file** | Three numbers for one dial: doc 1.5 (`AGENTS/AGENTS-STEERING.md:603`), code 2.5 (`Racer.cs:1805`), comment 0.5/0.8 (`:1801-1803`). |

The four `file` rows are graded there because they cannot cost a race, a file or a crash — the next
session that reads them is the one that re-tunes the number. Fix them in the same pass as the release
body; they are cheap and they are wrong.

### 3.3 Three premises in the brief that the files do not support

- **"720 commits"** since build 958: `git rev-list --count 9bdb9af..HEAD` = **126** (`201`).
- **"build 1092 desplegada"**: true of this install, but the tag will carry the CI run number, **≈277**
  (`202`; `deploy.yml:19`).
- **"los cuatro puntos R"** in `10-release-readiness.md`: the report has **eight** (`R1`–`R8`) — picking
  four would have been an invention (`202`).

### 3.4 Two candidates dropped for lack of a source

`206-03`'s "outside on both hands" consequence (the geometry is arithmetic, the consequence needs a
drive — it is kept as R6 at PROBABLE) and `205-03`'s balance claim (a track measurement, not a code
fact). Neither was written up as a finding to fill a row.

---

## 4. What this audit has NOT looked at

Honest scope, in the order it matters.

1. **Nothing was driven.** No row in 200–212 is driver-verified. Where a row says "the field has no walls
   three abreast" or "the follower is pushed outside", that is arithmetic on the code; the behaviour was
   not watched in the game. `207`, `206`, `209` and `203` each say the same in their own closing section.
2. **Nothing was built or profiled.** `e851a09` compiling is inferred from CI run #276 (`success`), not
   from a local compile (`202`, "What this pass could not establish", 1). Every performance figure in §2
   is a count times a frame rate — **no milliseconds are claimed** (`210`).
3. **The producer of B1's NaN is not identified.** The log proves a non-finite value reaches
   `Math.Sign`; which field is non-finite, and why, is open. Gating one producer was mistaken for closing
   the defect once already.
4. **The 44 non-blocker rows of `203`, `204`, `208` and `210` were not re-opened** by `212`, and this pass
   did not re-open them either. Their grades are inherited, not adjudicated. A second verifier is what
   that would take.
5. **Seven surfaces were never audited at all**: `SmartTuner` (menu label Auto Styling),
   `MenyooAppearance`, `VehicleSelector`/`VehicleCatalog`'s read path, `GridBuilder`,
   `FreeCamController`, `UpdateChecker` and `Tips`. `grep -l` across 201–212: `MenyooAppearance`,
   `VehicleSelector`, `GridBuilder` and `FreeCamController` appear in **none** of the twelve;
   `SmartTuner`, `UpdateChecker` and `Tips` appear only as passing mentions. `TrackVisuals` is cited
   only as a consumer (`209`).
6. **The design companions were not opened by the doc audit**: `AGENTS/AGENTS-DUEL.md`,
   `AGENTS/AGENTS-PERSONALITY.md`, `AGENTS/AGENTS-SMARTTUNING.md`, `AGENTS/AGENTS-FLAGS.md`, `AGENTS/AGENTS-VANILLA-STEERING.md`,
   `AGENTS/AGENTS-TEAMS.md`, plus half of `AGENTS/AGENTS-STEERING.md` (`211` §"Coverage"). The Duel model and the
   personality spec are designs, not code — nothing there can be out of date with a build, but nothing
   there is checked either.
7. **The FiveM tree** (`S:\FiveM\server-data\resources\[eddlm]\[ars-fivem]`) was not touched by any report.
8. **Three log lines are untriaged**: `No vehicles within 2 PI of the target` at 23:46:00 and
   `No vehicles within 8 PI of the target` at 23:46:18 and 23:46:23, all at `Error`, on this build. The
   grid could not be filled at the requested pace three times tonight. No audit row covers the
   grid-selection failure path, and this pass did not read it.
9. **Whether the reload crashes or resumes** is still one open decision, not a measurement (B10 / R7).
   `202` found the dumps are not from build 1092; which of `Aborted`-not-firing and `OnAbort` throwing is
   live is not decidable read-only.

**Nothing in this report was invented to fill a row.** Where a number would have needed a factor this
engine does not model, it is absent, not estimated.

---

## 5. What this means for the tag

**Eight fixes are code, and they are all small:**

1. Reset the grace deadline where a race is created — one line in `SetupRace` (B2).
2. A minimum route length at the creator's save, matched to the loader's lowest index (B6).
3. Clear the override at teardown, `StopNitrous` behind `_nitrousActiveUntil > 0`, from
   `Racer.Delete` (B7).
4. Compare the seam walk in scan order rather than in absolute node order (B3).
5. Move the corridor guard into clamp space, or inset the targets before the ordering test (B4).
6. Move the `IsNaN(SlideAngle)` test **before** the two `Math.Sign` calls (B1) — this fixes the symptom
   while the producer hunt continues.
7. Route `MenuSettings.Save` through the `WriteAtomic` the repo already has (B9).
8. Cap the brake factor's cut per event, or let the recovery step scale with it (B8).

**Two decisions are the owner's, and no code settles them:**

- Whether the rival record beats fast enough for the overlap gate, or the gate is widened to the
  record's own period (B5).
- Whether a reload ends a race or resumes it (B10 / R7) — with the crash path on the same keypress.

**Two sentences must be fixed before the archive ships**, because they are the release body telling the
player a number the code does not have: the stability rate (R1) and the nitrous pair (R2). Neither is a
code change; both are wrong in the artefact the player reads first.

**And one identity fact**: the tag will not say 1092. Whatever the notes say, the build the release
carries is the CI run number, ≈277.

---

*Every finding id carries its report: `03-G2` is `docs/audit/03-speed-governors.md`, `207-R1` is
`docs/audit/207-race-cycle.md`, and so on. The full index — `01` build-toolchain · `02` race-cycle ·
`03` speed-governors · `04` lane-avoidance · `05` physics-tyres · `06` menu-settings · `07` track-data ·
`08` runtime-robustness · `09` performance · `10` release-readiness · `11` docs-vs-code · `12`
data-integrity · `13` crash-environment · `201` verify 01–06 · `202` verify 07–13 · `203` steering ·
`204` pedals · `205` power · `206` walls-passing · `207` race-cycle · `208` menu-settings · `209`
track-data · `210` performance · `211` docs-vs-code · `212` the verifier. This file supersedes none of
them: it ranks them.*
