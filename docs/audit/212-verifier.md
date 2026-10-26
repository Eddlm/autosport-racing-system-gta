# AUDIT 212 — the verifier: every blocker of audits 201–211, re-checked in the code

**Surface**: every finding graded **blocker** by the eleven audits of the 0.9.0 pass — `201`, `202`, `203`, `204`, `205`,
`206`, `207`, `208`, `209`, `210`, `211` — plus every closure those audits claim for a finding that was a blocker in the
report it came from. Findings graded release-note or file below the blocker line were **not** re-opened (see §5).

**Date**: 2026-10-25 (late session).

**Build audited**: **1092**. `build/build-number.txt` = `1092`; the live
`Scripts\AutosportRacingSystem\ARS.dll` and `bin/Release/ARS.dll` are one file, sha256
`8e3ab22dc3b3ff8a1dda4dfd3cd8a7f5c8017f36206032ffa3210dea6c983285`, mtime 22:34:25. Source `HEAD e851a09`
(commits `bd55c24` 14:51, `9757f09` 15:24 and `237fbf5` 18:55 are all ancestors, so all three fixes are in the
deployed binary).

**Method**: read-only. Source read at the line quoted, `git log`/`git blame` read, `ScriptHookVDotNet2.dll` and
`ScriptSettings.Save` decompiled with `ilspycmd`, one minidump re-read with `cdbX64.exe`, one raw-geometry probe over a
shipped track (`ET` + circumradius, scratch script). **No build, deploy, commit, code edit or game run.**

**Status vocabulary**: **VERIFIED** = read in the code today, or reproduced read-only (arithmetic, decompile, dump,
geometry). **PROBABLE** = the mechanism is certain and one step rests on engine, SHVDN or drive behaviour not observed
here. **NOT VERIFIABLE** = the artefact that would settle it no longer exists. Nothing here is driver-verified.

**Verdict**: **19 blocker rows across the eleven reports collapse to 13 distinct defects** (six are the same defect
filed twice). **Ten stay at blocker grade, five are real at a lower grade, four are doc/file repairs — and one of the
nine closures re-checked is false.** The false closure is the most serious single result: `08-B1` (a NaN reaching
`Math.Sign` in `ComputeSteering`) is recorded as closed by `202`, while three other reports read 173 identical
`Math.Sign` exceptions out of the `ComputeSteering` frame **on the deployed 1092** — and the code still has the path.
Two reports contradict each other on the rival record's period (`201` says 500 ms, `206` says 250–749 ms): the code
says 250–749 ms. The live `Log.log` that carried the 173 exceptions has since been truncated by the next script load,
so that half is no longer reproducible from the file — the code path is, and it is not closed.

---

## 1. Blockers, ordered by real severity

Severity in **my** column is the release-gate grade: `blocker` = must change before the tag; `release-note` = true as
shipped, needs a decision or a line in the notes, cannot cost a race on its own; `file` = a defect in a tracked file
with no runtime consequence. The "report" column is the grade the audit gave, for the record.

| # | Report(s) | Report | Mine | Claim, re-checked | Evidence at `e851a09` | Status | How to reproduce |
|---|---|---|---|---|---|---|---|
| 1 | `02-B2` = `207-B1` | blocker | **blocker** | A race started from the Race menu inherits the previous race's grace deadline and ends itself on it — sometimes before its own countdown finishes. | `AutosportRacingSystem.cs:320` (the field), `:2275` (armed at the first finisher), `:2301-2302` (the test, outside every `RaceStatus` gate), `:905` and `:1831` (the only two resets). `CleanRacers` `:2550-2565` clears `LeaderboardFinish` and sets `RaceStatus = None` but **not** the deadline; `SetupRace` does not either. | VERIFIED | Run to the first AI crossing, then Race → Start Race before the "Ns to end the race" line expires. The new race dies when the *old* number reaches zero. |
| 2 | `07-T2` = `209-B1` | blocker | **blocker** | The creator saves a 2-node route the loader cannot read, and the throw lands after the world is torn down. | `AutosportRacingSystem.TrackCreator.cs:48` (`< 2` is the only guard), `:230-234` (one Attack click adds exactly 2 nodes); `TrackLoader.cs:152` (`RouteNodes[0]`), `:156-157` (`[2]`, `[3]`), `:166` (`[5]`); `AutosportRacingSystem.cs:1502-1523` (`InstanceTrack`, no `try`), `:2326-2328` (the catch). `DensifyRoute` returns below 4 nodes (`TrackLoader.cs:504-507`). | VERIFIED | Creator → one Attack (start line) → Save Track → Load Track: `ArgumentOutOfRangeException`, screen stays faded. |
| 3 | `05-P2` = `205-01` | blocker | **blocker** | Ending a race during a burn leaves `SET_OVERRIDE_NITROUS_LEVEL` armed on the player's car, which teardown deliberately keeps. | `Racer.cs:3460-3461` (charge + override), `:3496-3499` (`StopNitrous`, the only clear), `:3241` (returns when `NitrousEnabled` is off), `:3243` (returns while the burn is live), `:3244` (the only caller), `:5282-5299` (`Delete` keeps a player-owned car); `AutosportRacingSystem.cs:1820` (`CleanEverything` → `CleanRacers` → `Delete`), `:921` (End Race row). `Initialize` `:642-699` resets no nitro state. | VERIFIED (state). The *thrust* the armed bottle then permits is `205-02`'s question, not this one. | Nitrous on, player on the grid, fire, End Race inside the 3 s window: the car leaves the race with the system armed. |
| 4 | `07-T3` = `209-B2` | blocker | **blocker** | The width rule subtracts an **absolute** node from a **rotated scan list's** absolute node, so a region containing node 0 walks its entrance to `startPosition = 0`. | `TrackLoader.cs:869` (`while (startPosition > 0 && apexNode - scanNodes[startPosition] < widthNodes)`), rotation `:371-391`, publish `:899-901`. Consumers: `Racer.cs:3223-3243` (`IsWithinCorner` wrap branch) and `:3279` (in-corner nitrous veto), `:4374` (entrance node for braking). | VERIFIED (code + arithmetic). The shipped-track reachability is corroborated independently: El Burro Rally's start line sits in a real bend — R ≈ 66–71 m over raw nodes 0–30, opening to 203 m at node 40 (ET + circumradius probe today) — which is exactly the replica's `scanStart = 37`. | Load El Burro Rally at `LogLevel=Info`: the first `Corner add` reports `start=37` with the apex at node 0. |
| 5 | `04-W2` = `206-02` | blocker | **blocker** | An inverted clamp-space corridor discards both walls and returns the whole road, silently, at an ordinary three-abreast gap. | `Racer.cs:1681-1683` (inset per constrained side, then `if (clampLeft > clampRight) return Clamp(lane, -trackBound, trackBound)`), ordering guard `:1756-1762` runs on the **targets**, where no inset exists; targets `:1736`/`:1741`. Arithmetic: `rivalBuffer = 0.95 + 0.20` and `carHalf = 0.95` → inversion below **4.20 m** separation at Aggression 100, **6.20 m** at Aggression 0. | VERIFIED (code + arithmetic) | Two rivals one each side at ±2.1 m, Show Rivalry on: both walls draw red while the lane request crosses between them unclamped. |
| 6 | `04-W1` = `206-01` | parcial → blocker | **blocker** | The wall gates decide from a record that is 250–749 ms old, and the route gap moves further than the overlap window in that time. | `Racer.cs:104` (`RivalRefreshMs = 250`), `:559` (`_phaseOffsetMs = GetRandomInt(0, 500)` once per racer), `:4698-4701` (`if (_rivalTick + _phaseOffsetMs < now) { _rivalTick = now + RivalRefreshMs; … UpdateRivals(); }`), `:5279` (`UpdateRivals` ends with `UpdateRivalInfo()`), `:4784-4788` (the second, 500 ms publisher). Overlap window `DataStructures.cs:195-197` (±5.40 / 4.65 m). | VERIFIED (code). `201`'s 500 ms is wrong; `206`'s period is the code's. | At 10 m/s closure the gate can step 2.5–7.5 m per beat and skip the alongside tick; log `r.RouteGapAhead` beside the live recomputation. |
| 7 | `08-B1` (declared closed by `202`) | blocker → closed | **blocker — closure false** | A NaN still reaches `Math.Sign` inside `ComputeSteering` on the deployed build. | The 958 producer is dead: `Racer.cs:3876-3878` returns `0f` below 0.1 m/s. But `:912` and `:922` take `Math.Sign(slideDeg)` on every tick, and the only NaN test is *after* both (`:931-932`). Three reports (`207-R7`, `209`'s closing note, `210`'s "not verifiable") each read **173** `OnTick error: System.ArithmeticException … at System.Math.Sign … at ARS.Racer.ComputeSteering` lines from the 22:47–23:20 session of this 22:34 binary. | VERIFIED (code); the log witness is now **NOT VERIFIABLE** — `AutosportRacingSystem.cs:396` truncated `Log.log` to 4 lines at 23:41:57, and the file is 210 B today. | Leave ARS running 30 min at the shipped `LogLevel = Error` and `grep -c "OnTick error" Log.log`; each throw skips the rest of that frame and parks the batch cursor on the thrower (`:2243-2244`). |
| 8 | `12-B1` | blocker | **blocker** | Settings are written truncate-then-rewrite and the result is discarded; a torn file is then accepted. | `MenuSettings.cs:76-77` (`SetValue` + `Save()`, bool discarded) and `:84-85`; `ScriptSettings.Save` is `File.CreateText(_fileName)` (truncate) then a line-by-line write — decompiled from `libs/ScriptHookVDotNet2.dll`; `WriteAtomic` has exactly one caller, the repair pass (`SettingsRepair.cs:237`, `:367-379`); a torn value that happens to be a ladder rung is returned unchanged and unlogged (`SettingsRepair.cs:79`). | VERIFIED (mechanism) | Make `Menu-Settings.ini` read-only and change any row: the menu shows the new value, the file keeps the old one, nothing is logged. |
| 9 | `03-G2` | blocker | **blocker** | One hot entry can floor a corner's brake factor for the rest of the race; the cut is uncapped per event and the clamp is applied afterwards. | `Racer.cs:2509-2512` (`entryCut = 0.2 × max(0, excess − 0.894)/2.2352`), `:2524-2526` (clamp to `[0.5, 1.0]`), seed 0.75 ± 0.05 (`:341`, `:352`), recovery 0.025 per clean arrival (`:373`). 5 m/s over = **0.367**, i.e. the floor from the seed; recovery is **14 clean corners**. | VERIFIED (code + arithmetic) | Brake Learning + Brake Learn Notify on; one blocked approach 11 mph hot — the notice prints `0.50` and stays there. |
| 10 | `13-B2` | blocker | **blocker** | Reload/abort while ARS is executing is a crash path, and the record calls reload free. | `AutosportRacingSystem.cs:3461-3498` (`OnAbort`, no `try`), `:2814` (`Script.Yield()` inside `BuildRosterFromEnum`'s enum walk), `AGENTS.md:65` (no caveat). Dump `ceb5b698-dee0-479e-98f8-bcc9edfa6869.dmp` re-read today with `cdbX64`: `CLR exception type: System.ObjectDisposedException`, stack through `ScriptHookVDotNet!Unknown` and **`ARS!Unknown+0x35c1`**. | VERIFIED (dump); that ten Insert presses reproduce it is PROBABLE | `cdbX64 -z <dump> -y "" -c ".ecxr; k; q"`; in game, press `Insert` mid-race ten times and count the launches that die with no ARS line. |
| 11 | `03-G1` | blocker | **release-note** | The route brake cap is inert on almost every racing lap after the first that arms a corner. | `RequiresBraking` is written true once (`Racer.cs:4421`) and never false; `cornerSpd` is below the 999 sentinel for any armed corner in slots 1-3 (`:2631-2634`), `cornerPlansABrake` reads that (`:2669`), and the cap is skipped when it is set (`:2729`). | VERIFIED | Lap 1 vs lap 2 on a curvature-only stretch: the `BrakeReason.Route` ceiling reads 0.2 then 1.0. **Downgraded**: the defect is real and the mechanism is exactly as reported, but the consequence is a full-authority brake on a curvature step — a slam, not a lost race, a crash or a lost value. |
| 12 | `05-P3` | blocker | **release-note** | Every apex and braking solve is built from the grip sampled at the car, not at the feature ahead. | `Racer.cs:4644` (`RouteIdealSpeedForRadius`), `:4653` (`ApexSpeedWithDownforce`), `:4616` (`BrakingDecelBase`) all read `VehicleData.CurrentMechanicalGrip`, written at the car every core tick (`:3861`, `:5199-5201`), and the queue is re-solved from it each tick (`:4530-4538`). | VERIFIED | Log `CurrentMechanicalGrip` and `NextApexSpeed` at 2 Hz over a lap: the apex speed moves while the radius is constant. **Downgraded**: the value is recomputed every tick and therefore converges as the car slows, so the error is a lag, not an open-loop plan; it degrades precision, it does not break a race. |
| 13 | `207-B2` | blocker | **release-note** | A script reload mid-race resumes the previous race on a half-reset state, and that race can no longer be timed out. | Statics `AutosportRacingSystem.cs:149`, `:243`, `:251`, `:256`, `:305`, `:308`; instance `:320` (the deadline), `:1845-1846` (the batch cursor); `OnAbort` `:3461-3498` clears racers and props but none of `RaceStatus`, `LeaderboardFinish`, `PlayerParticipating` or the route. | VERIFIED (structure) / PROBABLE (that SHVDN re-instantiates over live statics). The log witness (`207`'s 22:47–23:20 read) is gone with the truncation at `:396`. | Reload mid-race and read `Log.log` for `Card`/`Kick` lines with no `Loading Racer` line. **Downgraded**: it needs the reload key, and `13-B2` says the same press may crash instead — one decision, not two fixes. |
| 14 | `10-B2` | blocker | **release-note** | The release body states the stability cut at twice the code's rate, and asymmetrically where the code is symmetric. | `docs/RELEASE-NOTES.md:15` ("closes a **quarter** of the throttle per wheel out of contact and gives it back at **half the rate**") against `Racer.cs:2864-2865` (`0.5f` / `0.5f`, applied `:2920` / `:2922`). | VERIFIED | `grep -n "StabilityFallPerWheelPerSecond\|StabilityRisePerSecond" src/Racer.cs docs/RELEASE-NOTES.md`. |
| 15 | `211-B5` | blocker | **release-note** | The release draft sells the nitrous pair as a measured 1:2.5; the code is 5 against 1. | `docs/release-0.9.0.md:33` ("1:2.5 as measured on track"); `Racer.cs:443` (`NitrousPowerMultiplier = 5f`), `:447` (`PlayerNitrousPowerMultiplier = 1f`), applied `:3984-3985` — parity is at `AiNitroPct` **50**. `AGENTS/AGENTS-WIP.md:480` and `:538` say 2.5 and "player 2.0". | VERIFIED | `grep -n "1:2.5" docs/release-0.9.0.md AGENTS/AGENTS-WIP.md src/Racer.cs`. |
| 16 | `211-B1` | blocker | **file** | Four sites in `AGENTS/AGENTS-STEERING.md` and one in `TEST-PLAN.md` state the lane mover at 14 m/s² with a 12 m/s cap; the code runs 26 and 24. | `Racer.cs:1102-1103` (`26f`, `24f`); `AGENTS/AGENTS-STEERING.md:183`, `:361`, `:660`, `:686` and `TEST-PLAN.md:498` say 14/12, while the same file at `:388` and `:676` says 26/24. | VERIFIED | `grep -n "LaneMoverAccelMps2" src/Racer.cs AGENTS/AGENTS-STEERING.md TEST-PLAN.md`. |
| 17 | `211-B2` | blocker | **file** | The lane section names five symbols the tree does not contain and says the surviving law has no lateral leash — the leash is live. | `LeadBounded`, `LaneInwardLeadMeters`, `LaneOutwardLeadMeters`, `LaneInwardLeadTaperMeters`, `IdealLineOffset`: **0 hits** in `src/`. Live leash: `Racer.cs:1110-1114` (`LaneTaperNearMeters` 2, `LaneTaperFarMeters` 4, `LaneTaperCapShare` 0.5) applied at `:1201-1202`; `AGENTS/AGENTS-STEERING.md:683-687` ("All of it is gone, together with `LeadBounded`") and `:699`. | VERIFIED | `grep -rnw "LeadBounded\|LaneInwardLeadMeters\|LaneOutwardLeadMeters\|IdealLineOffset" src/` → 0; `grep -n "LaneTaper" src/Racer.cs`. |
| 18 | `211-B3` | blocker | **file** | The damper paragraph caps the crossing at a constant that does not exist (5°), then declares the cap removed; the code caps at 2°. | `DamperCrossingMaxDeg` → 0 hits; `Racer.cs:883` and `:888-889` clamp to `ARS.DamperCountersteerMaxDeg`, `AutosportRacingSystem.cs:179` (`2f`), `SettingsRepair.cs:159` (default `"2.00"`, ladder 0…10); `AGENTS/AGENTS-STEERING.md:71` states both halves of the contradiction. | VERIFIED | `grep -rnw "DamperCrossingMaxDeg" src/` → 0. |
| 19 | `211-B4` | blocker | **file** | Three numbers for one dial: doc 1.5, code 2.5, comment 0.5/0.8. | `Racer.cs:1805` (`SlideBlendFullFraction = 2.5f`), `:1804` (`1f`), comment at `:1801-1803` ("peak × 0.5 … peak × 0.8"), `AGENTS/AGENTS-STEERING.md:603` ("`SlideBlendFullFraction` 1.5"). | VERIFIED | `grep -n "SlideBlend" src/Racer.cs AGENTS/AGENTS-STEERING.md`. |

**Counts**: 10 blocker · 5 release-note · 4 file. Of the 10 blockers, seven are player-facing (1–5, 8, 9), one is a
crash on the reload key (10), one is the false closure of a live exception (7), and one is the stale record the walls
decide from (6).

**The four doc blocks (16–19) are graded `file`, not `blocker`**: they cannot cost a race, a file or a crash, and the
next session that reads them is the one that re-tunes the number. `15` is the one doc row that reaches the player
through the release body, and that is why it is a release-note rather than a file.

---

## 2. Closures re-checked

Nine closures were re-checked: the six findings that were **blockers** in the report that raised them (`02-B1`, `05-P1`,
`07-T1`, `08-B1`, `10-B1`, `13-B1`) plus three the verify passes closed in passing (`05-P5`, `10-F4`, `08-M2`).
Eight hold; **one does not**.

| Closed | By | What I read | Verdict |
|---|---|---|---|
| `02-B1` player's own car deleted at teardown | `bd55c24` | `Racer.cs:5282-5299` — the branch is `if (ControlledByPlayer \|\| Game.Player.Character.IsInVehicle(Car))`, and only the `else` deletes. | holds |
| `05-P1` player's ped left with proofs and 1000 HP | `bd55c24` | `Racer.cs:629-638` — the proofs/health block is inside `if (!Driver.IsPlayer)`. | holds |
| `05-P5` `SlideAngle` 90° at rest | `bd55c24` | `Racer.cs:3876-3878` — `Velocity.LengthSquared() > 0.01f ? … : 0f`. | holds |
| `07-T1` one unparseable file stopped startup | `bd55c24` | `AutosportRacingSystem.cs:472-490` — `EnumerateFiles(dir, "*.xml")` and a per-file `try`/`catch` with `Track skipped (...)` at `Error`. | holds |
| `10-B1` release note sold ABS as shipped | `9757f09` | `docs/RELEASE-NOTES.md:15` — "shipped **off** and greyed"; `AutosportRacingSystem.cs:3602` forces `AbsEnabled = false`. | holds |
| `10-F4` `AbsEnabled` declared `True` while forced false | `9757f09` | `SettingsRepair.cs:150` — `KeySpec("AbsEnabled", "False", Kind.Bool)`. | holds |
| `13-B1` wheel-pointer miss became an unbounded dereference | `9757f09` | `AutosportRacingSystem.cs:3129-3135` (miss → forced `Error`, `return 0`), `:3168` (`wheelPtr == 0 \|\| numWheels <= 0 \|\| numWheels > 8`). | holds |
| `08-M2` `ShowRivalry` alone drew nothing | `13599db` | `AutosportRacingSystem.cs:2257` — the focus scan's condition now lists `Options.ShowRivalry`. | holds |
| **`08-B1` NaN at `Math.Sign` in `ComputeSteering`** | **`bd55c24`** | Row 7 above: the named producer is gated, but the exception the log recorded on this same binary is the same signature, and `:912`/`:922` can still see a non-finite `SlideAngle` with the NaN test after them (`:931-932`). | **does not hold** |

A closure that rests on "the producer I named is gone" is a half-closure when the symptom has another producer and the
symptom was still observed. `202` wrote the verdict from the code alone; `207`, `209` and `210` each read the log of the
same session and each counted 173 throws.

---

## 3. Where the reports contradict each other

| Pair | Contradiction | Which is right, and why |
|---|---|---|
| `201` `04-W1` vs `206-01` | The rival record's period: **500 ms** vs **250–749 ms**. | **`206`.** `_rivalTick` is armed `now + 250` (`Racer.cs:4700`) and the gate adds the per-racer `_phaseOffsetMs` (`:559`, `:4698`), so the beat is `250 + phase` with `phase ∈ [0,499]`. `201` also names the 500 ms path as what fills the record; `UpdateRivals` ends with `UpdateRivalInfo()` (`:5279`), so it is the 500 ms path that is redundant. |
| `202`'s `09-P5`, `210`'s P5 vs `206-01` | The rival scan runs at **4 Hz** (both) vs **1.3–4 Hz, 2 Hz mean**. | **`206`.** Same arithmetic; `202`'s "3,480 natives/s, 4× the audited figure" is 2× on average, 4× only at `phase = 0`. |
| `201` `04-W2` vs `206-02` | Inversion arithmetic: `201` says two rivals at 4.0 m give targets "about ±0.35 m" and a 0.95 m half width turns that into "+0.60/−0.60". | **`206`.** With a 1.9 m car the targets are ±0.85 and the clamps ±0.10; 4.0 m is still inside the 4.20 m inversion threshold, so the finding stands and the numbers in `201` do not. |
| `205-01` vs `205-02` (same report) | `205-01` says the kept car "pulls, repeatedly, with no race running"; `205-02` proves the player's bottle carries no drive force because ARS never triggers the native activation. | Both true in their own half: the **state** leak is real and permanent (`205-01`), the **thrust** is conditional on a control ARS does not press (`205-02`). `201`'s `05-P2` impact sentence ("the kept car keeps the system") is the accurate one. |
| `207-B2` vs `13-B2` | A reload mid-race **resumes** the race (207) vs a reload mid-execution is a **crash** (202/13). | Not resolvable from the tree: they are two outcomes of one undefined action, and `207` itself files the decision ("does a reload end the race, or resume it?"). Both are ranked as one item here. |
| `202` `08-B1` vs `207-R7`/`209`/`210` | `08-B1` **CLOSED** vs 173 identical `Math.Sign` exceptions in `ComputeSteering` on the deployed build. | **`207`/`209`/`210`.** See row 7. |

---

## 4. Reports that declared no blocker — what I checked

`203` (steering), `204` (pedals), `208` (menu/settings) and `210` (performance) each declare **no blocker**, and this
pass agrees with the grade: nothing in them is a must-change-before-the-tag. Three of their rows are load-bearing for
a blocker and were re-read here rather than trusted:

- **`203-05`** (the pedal reads the previous tick's `_slip`) — order confirmed: `ConvertSpeedToPedals` `Racer.cs:4796`
  before `ApplySteerLimits` `:4804`; `_slip` written at `:2091`. Immaterial, as reported.
- **`204-01`** (a dead stop drains the stability cap) — `Racer.cs:2918-2923` with `WheelsOffGround` at
  `AutosportRacingSystem.cs:3304-3313`. The engine half (`wheel.cpp:5303-5309`) is the reported source, not re-read
  here; it stays PROBABLE.
- **`210`'s own correction that `12-B1`'s write is per menu change** — consistent with row 8.

`203`, `204`, `208` and `210`'s remaining release-note and file rows (10 + 10 + 10 + 14) were **not re-opened**. Their
grades are not in dispute here; a later pass that wants them verified needs a second verifier.

---

## 5. What this pass could not establish

1. **The 173 exceptions are no longer in the file.** `Log.log` was truncated at 23:41:57 by the constructor
   (`AutosportRacingSystem.cs:396`) and now holds four lines, two of them `Info`. Row 7 rests on three independent
   reads of the session that preceded the truncation, and on the code path — not on a file I can show you today. That
   truncation is itself `13-R5`'s finding.
2. **Whether the reload path crashes or resumes.** The three dumps named in `202` exist (Aug 19, Sep 10, Sep 11) and the
   Sep 10 one re-read today does carry an `ARS!Unknown` frame under an `ObjectDisposedException`; none of them is from
   build 1092, and which of `Aborted`-not-firing and `OnAbort` throwing is the live one is not decidable read-only.
3. **Whether the seam corner on El Burro Rally kills the lap in the game.** The code defect is verified and the track's
   start-line bend is corroborated by raw geometry; the span numbers (`StartNode=37`, `LengthStart=1664`) come from
   `209`'s calibrated replica, not from a driven lap.
4. **Any driving consequence.** No row here is driver-verified. Where a row says "the field has no walls three abreast",
   that is arithmetic on the code — the wall gate itself was not watched in the game.
5. **Whether `203`, `204`, `208`, `210`'s non-blocker rows are individually right.** Out of scope by the brief; the
   covered set is the blocker rows plus the nine closures in §2.

**Nothing in this report was invented to fill a row.** Two candidate rows were dropped for lack of a source: `206-03`'s
"outside on both hands" (the geometry is right — sagitta `d²/2R` = 4.0 m at 20 m ahead on a 50 m radius — but the
consequence needs a drive and the report itself grades it release-note), and `205-03`'s balance claim (a track
measurement, not a code fact).

---

## 6. What this means for the tag

Ten rows are marked blocker. Eight are code fixes and two are decisions the owner has to make first:

- **Fix before the tag, code**: the grace deadline reset in `SetupRace` (1); the creator's minimum route length (2); the
  override cleared at teardown (3); the seam walk compared in scan order (4); the corridor guard moved to clamp space
  (5); the `Math.Sign` NaN test moved before the calls (7); `WriteAtomic` used by `MenuSettings.Save` (8); the brake
  factor's per-event cut capped (9).
- **Decide before the tag**: whether the rival record is refreshed often enough for the overlap gate, or the gate is
  widened to the record's own period (6); whether a reload ends a race or resumes it (10/13).
- **Leave as a release note**: the route cap's latch (11), the grip sampling (12), the nitrous parity wording — 100 or
  50, pick one and say it (15) — and the four internal doc blocks (16–19), with `14` and `15` corrected in the release
  body before it ships.
