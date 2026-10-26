# AUDIT 11 — docs against code: stale refs, wrong units, closed-but-open

**Surface**: every claim in the doc set that names a `src/*.cs` line, a constant with a value or unit, a state tag
(`closed`, `drive owed`, `not fixed`), or a behaviour a reader would act on: `AGENTS.md`, `AGENTS/AGENTS-STEERING.md`,
`AGENTS/AGENTS-TECHNOTES.md`, `AGENTS/AGENTS-BACKLOG.md`, `AGENTS/AGENTS-WIP.md`, `docs/OPEN-ITEMS.md`, `docs/KNOWN-BUGS.md`.
**Date**: 2026-10-25.
**Build audited**: **958** (`build/build-number.txt` = `958`; source `HEAD 9bdb9af`, 31 commits ahead of
`origin/master`). Every line number below is against that tree.
**Method**: read-only — no edit, build, deploy, commit or game run. Two instruments: a reference extractor over the
doc set (287 `file.cs:line` citations pulled with their sentence, each resolved against `src/`), and a constant
extractor (1 549 backticked identifiers in the docs, 189 of which resolve to a declared constant with a numeric
initialiser, each printed beside its declaration and its doc sites). The candidate list came from those two, then
every row below was opened by hand on both sides. Arithmetic was recomputed, never recalled.

**Verdict**: the code is in better shape than its notes. Of **56 claims checked**, **16 are wrong in a way that would
cost a session real time** — four places still describe the lane mover at 14 m/s²/12 m/s when it has run at **26/24**
since `5635848`, two describe the brake learner with numbers two tuning passes old (one of them mis-stating the effect
by **4.8×**), one names a damper constant that does not exist at a value the shipped one does not have, and one calls
a dead branch (behind `bool avoidHoldEnabled = false`) an outstanding drive. **22 more cite a line that has moved
(270–1 000 lines) or a symbol that is gone**, which reads as a broken claim because the target now holds someone
else's code, and **2 are state tags the code contradicts**. The remaining 16 hold as written (14) or are
**probable**-only (2). **No claim was found where the code is the one that is wrong** — every mismatch resolved in the
code's favour, which is the direction this check is supposed to find.

---

## Group 1 — wrong value or unit (the expensive class; 15 rows)

These are the rows where a reader would tune, drive or package against a number the code does not use.

| # | Doc (file:line) | Claim as written | Code says (file:line) | Verdict | Conf. |
|---|---|---|---|---|---|
| 1 | `AGENTS/AGENTS-STEERING.md:183` | "the lane mover carries the carried lane onto the chain's request at **14 m/s^2 with a 12 m/s cap**" | `LaneMoverAccelMps2 = 26f`, `LaneMoverSpeedCapMps = 24f` (`Racer.cs:1054-1055`) | **wrong** | VERIFIED |
| 2 | `AGENTS/AGENTS-STEERING.md:361` | item 3, marked "BUILT and driver-verified": "carried onto it at **14 m/s^2 with a 12 m/s cap**" | same two constants (`Racer.cs:1054-1055`); the `OutsideLine` half is 13/12 (`Racer.cs:1141-1146`) | **wrong** (stale; see #5 for the half) | VERIFIED |
| 3 | `AGENTS/AGENTS-STEERING.md:660` | "carries the carried lane onto that request at **14 m/s^2 with a 12 m/s cap**" | `Racer.cs:1054-1055` = 26 / 24 | **wrong** | VERIFIED |
| 4 | `AGENTS/AGENTS-STEERING.md:684` | "What is left is the acceleration limit (`LaneMoverAccelMps2` **14**) and the speed cap (`LaneMoverSpeedCapMps` **12**)" | `Racer.cs:1054-1055` = 26 / 24; `5635848` ("lane mover at 26 and 24") is the commit that moved them, and it rewrote exactly one paragraph of `AGENTS/AGENTS-STEERING.md` — the one now at `:686`; the four rows above were missed | **wrong** | VERIFIED |
| 5 | `AGENTS/AGENTS-STEERING.md:686` | "halved to **13 and 12** while the reason is `OutsideLine`" | `Racer.cs:1141-1146` halves the pair only for `LaneReason.OutsideLine`; 26/2 = 13, 24/2 = 12 — the halves are right while the four rows above state the unhalved pair wrongly in the same file | **verified** | VERIFIED |
| 6 | `docs/OPEN-ITEMS.md:23` | states the mover correctly — "The lane mover is sized from the traversal and drops momentum on reversal, at **26 m/s² and 24 m/s**" | `Racer.cs:1054-1055` = 26 / 24 | **verified** — and it is the doc that contradicts rows 1-4: two files describe one law with different numbers | VERIFIED |
| 7 | `AGENTS/AGENTS-STEERING.md:897` | "twenty miles an hour over costs a fifth of the factor (`BrakeEntryFactorStep` 0.2 over `BrakeEntryExcessMps` **8.9408**) … the first five miles an hour dead (`BrakeEntryDeadbandMps` **2.2352**)" | `BrakeEntryExcessMps = 2.2352f` (**five** mph, `Racer.cs:347`), `BrakeEntryDeadbandMps = 0.89408f` (**two** mph, `Racer.cs:349`), `BrakeEntryFactorStep = 0.2f` (`:348`), used at `Racer.cs:2330` as `step × (excess − deadband) / anchor` | **wrong** — both anchors are one step stale, and the derived number is 4.8× off: at a 20 mph overshoot the doc's own law gives **0.15**, the code gives **0.72**, which from the 0.75 seed saturates `BrakeMinFactor 0.5`. The same file's `:411-416` carries the current numbers (2/5 mph, −0.12 at 5, −0.32 at 10) — **the file contradicts itself** | VERIFIED |
| 8 | `AGENTS/AGENTS-TECHNOTES.md:140-142` | "inside `BrakeEntryDeadbandMps` (**5 mph**) … gains a flat `BrakeGoodArrivalStep` **0.05**; above the deadband … `BrakeEntryFactorStep` 0.2 for **20 mph** over" | `BrakeEntryDeadbandMps` is **2 mph** (`Racer.cs:349`), `BrakeGoodArrivalStep` is **0.025** (`:340`), the anchor is **5 mph** (`:347`) | **wrong** ×3 — the paragraph is two tuning passes old (`1e287b7` and `d22cad6`/`0e86433` both skipped this file) | VERIFIED |
| 9 | `AGENTS/AGENTS-TECHNOTES.md:130` | "`AbsBrakeFallPerSecond` **0.2/s**, so from 1.0 to the `AbsBrakeFloor` **0.2** takes four seconds" | no `AbsBrakeFallPerSecond` exists anywhere in `src/` (`grep -rn AbsBrakeFallPerSecond src` → 0 hits); the guard is a stateless taper on slip ratio (`AbsKneeSlip 0.5`, `AbsLockSlip 1.0`, `AbsBrakeFloor = 0f`, `Racer.cs:2809-2811`) | **wrong** — the field and the floor both; `docs/OPEN-ITEMS.md:60` already has this filed and it is still true | VERIFIED |
| 10 | `AGENTS/AGENTS-STEERING.md:602-603` | "the band starts at the peak and ends past it: **1.0 to 2.5** … `SlideBlendStartFraction` 1.0 and `SlideBlendFullFraction` **1.5**" | `SlideBlendStartFraction = 1f`, `SlideBlendFullFraction = 2.5f` (`Racer.cs:1754-1755`), read at `Racer.cs:818` | **wrong** — the sentence's own opening ("1.0 to 2.5") is right and its named value ("1.5") is wrong, so a reader who trusts the symbol tunes a band the code does not have | VERIFIED |
| 11 | `AGENTS/AGENTS-STEERING.md:71` | "capped at `DamperCrossingMaxDeg` **5 degrees** past neutral" | `DamperCrossingMaxDeg` has **0 hits** in `src/`; the field is `DamperCountersteerMaxDeg = 4f` (`AutosportRacingSystem.cs:178`), default `"4.00"` with a 0/2/4/6/8/10 ladder (`SettingsRepair.cs:159`) | **wrong** — wrong name and wrong value | VERIFIED |
| 12 | `docs/OPEN-ITEMS.md:16` | "The damper countersteers **5°** and no further, behind a 0–10 knob in twos" | ladder form is right (0–10 in twos), the shipped default is **4** (`SettingsRepair.cs:159`, `AutosportRacingSystem.cs:178`) | **wrong** (value) | VERIFIED |
| 13 | `AGENTS/AGENTS-BACKLOG.md:78` | `stability-speed-scaling`: "falls at `StabilityFallPerWheelPerSecond` **1.0** per wheel per second … recovers at **0.25/s**" | `StabilityFallPerWheelPerSecond = 0.5f`, `StabilityRisePerSecond = 0.5f` (`Racer.cs:2683-2684`) | **wrong** — and self-defeating: the paragraph's closing "**the 4:1 asymmetry** … stays" only holds for the *code's* numbers (4 × 0.5 = 2.0/s down against 0.5/s up); the numbers it prints give 16:1 | VERIFIED (`git blame` → `05ea488`, whose message is literally "stability empties in half a second and refills in two"; it edited `AGENTS/AGENTS-STEERING.md` only) |
| 14 | `docs/OPEN-ITEMS.md:19` | "Stability is a per-wheel rate … with a lost wheel costing **1/s** of throttle cap" | `StabilityFallPerWheelPerSecond = 0.5f` (`Racer.cs:2683`) → **0.5/s** per wheel | **wrong** — same constant, second document, still 1.0 | VERIFIED |
| 15 | `AGENTS/AGENTS-WIP.md:447` | the ordering guard's "**corridor half a metre** clamped inside the edges" | `float corridorHalf = Math.Min(1f, trackBound);` (`Racer.cs:1708`), i.e. **±1 m** on any road, with the pair placed at `mid ∓ 1` (`:1709-1711`) | **wrong** (unit/value) — and the doc's own quoted subagent figures in the same paragraph (5.625/7.625) are the ±1 ones, so the prose and its evidence disagree | VERIFIED |

**The quantified half of #7-8** (skill §4 — two laws, two numbers): the learner's cut is
`0.2 × (excess − 0.89408) / 2.2352`, so *5 mph over* costs 0.12, *10 mph* 0.32, *20 mph* 0.72 (floor), *40 mph* 1.52
(floor). The doc's stale form (anchor 8.9408, deadband 2.2352) yields 0.15 at 20 mph. **The learner is 4.8× sharper
than `AGENTS/AGENTS-STEERING.md:897` and `AGENTS/AGENTS-TECHNOTES.md:141` describe, and saturates the factor floor at a 10 mph
overshoot rather than a 20 mph one** — that is a different tuning regime, not a different rounding.

---

## Group 2 — stale line reference (target line now holds other code; 23 rows)

Anchors, not contracts — but a reader who jumps to one and finds someone else's statement has no way to tell
"moved" from "never existed".

| # | Doc (file:line) | Claim as written | Code says (file:line) | Verdict | Conf. |
|---|---|---|---|---|---|
| 16 | `AGENTS/AGENTS-STEERING.md:664` | "`CurveLeanChordSeconds` (`Racer.cs:675`), beside `CurveLeanRadiusMeters` (`:674`)" | declarations at `Racer.cs:945` and `:944` (Δ ≈ 270 lines); `:674-675` are the tail of `UpdateSlidePriority` | **stale ref** | VERIFIED |
| 17 | `AGENTS/AGENTS-STEERING.md:665` | "`ComputeCornerLane` `Racer.cs:762`" | `float ComputeCornerLane(...)` at `Racer.cs:1262` (Δ 500); `:762` is a comment inside the steer assembly | **stale ref** | VERIFIED |
| 18 | `AGENTS/AGENTS-STEERING.md:665` | "`IdealLineOffset` `Racer.cs:753`" | **`IdealLineOffset` has 0 hits case-insensitively in `src/`**; `:753` is `laneSteerDeg *= LaneCrossingShare(…)` | **stale ref + absent symbol** — no mechanism to verify; the offset profile is `ComputeCornerLane`'s own arithmetic (`Racer.cs:1262-1466`) | VERIFIED |
| 19 | `AGENTS/AGENTS-STEERING.md:847` | "The inside line aims at the inside edge — `−cornerDir × (roadWide − half the car)` (`Racer.cs:808-809`)" | `lane = -cornerDir * Math.Max(laneEdge - LaneInsideGapMeters, 0f)` (`Racer.cs:1036`): the operand is the **lane edge less an inset**, not `roadWide` less the car's half width; `:809` is `const float steerKP = 1.0f` | **wrong formula + stale ref** | VERIFIED |
| 20 | `AGENTS/AGENTS-STEERING.md:661` | "`OutOfTrackDistance` (`Racer.cs:2807`) is centre-based on the raw half width" | `float OutOfTrackDistance()` at `Racer.cs:3821` (Δ 1 014); `:2807` is a comment in the ABS block | **stale ref** (the behaviour claim itself holds) | VERIFIED |
| 21 | `AGENTS/AGENTS-STEERING.md:897` | "`ApexBrakingSpeed` `Racer.cs:3165`" | `float ApexBrakingSpeed(int apexNode, float apexSpeed)` at `Racer.cs:4359`; `:3165` is `ActiveManeuver.LastEnabled = Game.GameTime;` | **stale ref** | VERIFIED |
| 22 | `AGENTS/AGENTS-STEERING.md:897` | "the `Math.Min` chain at `Racer.cs:1564`" | the three-apex min chain is `Racer.cs:2455-2459`; `:1564` is a bare `continue;` inside the wall loop | **stale ref** | VERIFIED |
| 23 | `AGENTS/AGENTS-STEERING.md:899` | "and **min governs** (`Racer.cs:1544`)" | `cornerSpd = Math.Min(cornerSpd, …)` at `Racer.cs:2457` and `:2459`, assembled at `:2454`; `:1544` is a comment about a latched target | **stale ref** | VERIFIED |
| 24 | `AGENTS/AGENTS-STEERING.md:61` | "`LookAhead.SteerRef` is `speed × SteerPreviewSeconds` clamped to a floor and a cap in metres (`Racer.cs:2670`, `:129-131`)" | `SteerPreviewSeconds = 0.5f` (`Racer.cs:167`), floor `SteerLookaheadMinMeters 8` (`:168`), cap `SteerLookaheadMaxMeters 30` (`:169`); `:2670` is a closing brace, `:129-131` are the recovery speed constants | **stale ref** ×3 | VERIFIED |
| 25 | `AGENTS/AGENTS-STEERING.md:70` | the sliding gate "(`Racer.cs:589-590`, `:714`, `:942`)" and the reference "(`SteerDampingAimReference`, `Racer.cs:960`)" | the gate is `Math.Abs(SlideAngle) < Handling.LateralTractionCurve * SlidingFraction` at `Racer.cs:822`; the damper block is `:824-836`; `SteerDampingAimReference` is declared at `Racer.cs:1792`; `:589-590` is `SET_ENTITY_PROOFS` | **stale ref** ×4 — and `:589-590` is in the proof/HP block, so a reader chasing the damper ends up editing god-mode | VERIFIED |
| 26 | `AGENTS/AGENTS-BACKLOG.md:21` (`stale-track-path-load`) | "`InstanceTrack` hands `_selectedTrackPath` straight to the loader (`AutosportRacingSystem.cs:1356-1367`)" | `public void InstanceTrack()` at `AutosportRacingSystem.cs:1436`; `:1356-1367` is development-menu construction | **stale ref** (the defect statement is unaffected) | VERIFIED |
| 27 | `AGENTS/AGENTS-BACKLOG.md:42` (`gravity-grip`) | "a theoretical or reporting one (`ComputePaceIndex` `AutosportRacingSystem.cs:97` …)" | `public static float ComputePaceIndex(...)` at `AutosportRacingSystem.cs:109`; `:97` is a comment above `ModelElectricCache` | **stale ref** | VERIFIED |
| 28 | `AGENTS/AGENTS-BACKLOG.md:94` (`immersive-join-points`) | "the chevron draws at the nearest start (`AutosportRacingSystem.cs:1880`), Context joins within 5 m (`:1885`), the join takes the mode's anchor… (`:1887`)" | `:1880-1887` are spectate lip drawing / `DrawLine` calls; the only surviving text on that claim is a code comment, `// Pace Mode governs this grid like any other — the join adds no pace override of its own.` (`:2058`) | **stale ref** ×3 — item is marked **closed** on the strength of three refs that no longer point at the join at all | VERIFIED |
| 29 | `AGENTS/AGENTS-BACKLOG.md:86` (`overspeed-correction`) | "the wheel-pushed Gs are sampled at ~3 Hz with a 0.1 G deadband (`Racer.cs:4109-4123`)" | the block is `Racer.cs:4970-4984` (`_lastGsCheck` gate, `excess = measuredGs - wheelGs - 0.1f` at `:4981`); `:4109-4123` is `UpdateApexLeapfrog` | **stale ref** (deadband 0.1 confirmed) | VERIFIED |
| 30 | `docs/OPEN-ITEMS.md:32` | "the 8 s `AvoidHoldTimeoutMs` backstop (`Racer.cs:1281`)" | `AvoidHoldTimeoutMs = 8000` at `Racer.cs:1488`; `:1281` is a comment inside `ComputeCornerLane` | **stale ref** — and **closed-but-open** in the worse direction: the whole hold is unreachable (`bool avoidHoldEnabled = false`, `Racer.cs:1499`, early return `:1501`), so this is a drive owed on code that cannot run | VERIFIED |
| 31 | `docs/OPEN-ITEMS.md:55` | "`FullPedalSpeedErrorMps` 3, ×2 for brake, `Racer.cs:1867`" | `FullPedalSpeedErrorMps = 3f` at `Racer.cs:266`; `:1867` is a bare `{` inside the slip-ceiling method | **stale ref** (the law itself is as described) | VERIFIED |
| 32 | `docs/OPEN-ITEMS.md:31` and `:32` | two drives "owed" anchored at `AGENTS/AGENTS-WIP.md:184` and `:185` | both targets are **blank lines**; the build-951 block they belong to is `AGENTS/AGENTS-WIP.md:456-464` | **stale ref** ×2 | VERIFIED |
| 33 | `docs/OPEN-ITEMS.md:48` | "`LaneCrossingScale` … (AGENTS/AGENTS-STEERING.md:670)" | `:670` is the corner-line entry-only bullet; the crossing-scale text is at `AGENTS/AGENTS-STEERING.md:845-848` | **stale ref** | VERIFIED |
| 34 | `docs/OPEN-ITEMS.md:61` | "`AGENTS/AGENTS-WIP.md:151` still lists the brake learner's pedal-share defect as open" | `:151` is the curvature/speed-step paragraph; **`grep -n "pedal share\|pedal-share" AGENTS/AGENTS-WIP.md` returns nothing** — the defect it says is still listed is not in the file at that line or any other | **stale ref + false claim** — the filed item should be closed: `a7e0e95` did replace the verdict (see `AGENTS/AGENTS-STEERING.md:411-416`) | VERIFIED |
| 35 | `AGENTS/AGENTS-TECHNOTES.md:413` | "whose gate is gone and whose blend now rides ungated (`Racer.cs:579`)" | `GsPreviewBlend = 0.5f` (`Racer.cs:171`) applied at `:738`; `:579` is a `_SET_BLIP_SHOW_HEADING_INDICATOR` call | **stale ref** | VERIFIED |
| 36 | `AGENTS/AGENTS-TECHNOTES.md:414` | "`DrawGsProjection` `Racer.cs:2615`" | `void DrawGsProjection()` at `Racer.cs:3513` | **stale ref** | VERIFIED |
| 37 | `AGENTS/AGENTS-TECHNOTES.md:414` | "`UpdatePlayerGLine` `AutosportRacingSystem.cs:3160`" | `void UpdatePlayerGLine(Vehicle car)` at `AutosportRacingSystem.cs:3335`; `:3160` is a closing brace | **stale ref** | VERIFIED |
| 38 | `AGENTS/AGENTS-TECHNOTES.md:414` | "Joins the debug-focus scan gate (`AutosportRacingSystem.cs:2004`)" | `:2004` is `Color leftColor = racer.LeftWallHeld ? Color.Red : Color.DodgerBlue;` — the rival-wall draw, not the scan gate | **stale ref** | VERIFIED |

---

## Group 3 — the claim is a mechanism the code no longer has (4 rows)

| # | Doc (file:line) | Claim as written | Code says (file:line) | Verdict | Conf. |
|---|---|---|---|---|---|
| 39 | `AGENTS/AGENTS-STEERING.md:660` and `:847` | "**The aim is clamped into the interval between the car's own lane and the request** (`Racer.cs:907-914`), after the integration" — given a *justification* (limit cycle), not just a citation | there is no such clamp: `_carriedLane += step;` (`Racer.cs:1170`) is the last write to the aim, and the comment directly below it states the opposite as deliberate — "**Nothing below this line can move the aim without inertia**" (`:1172-1175`). `:907-914` is `ComputeSideBySideSteerCorrection`. The file's own history at `AGENTS/AGENTS-STEERING.md:399-402` records that the aim-side clamp "went too (build 894)" | **wrong** (mechanism deleted; doc still argues for its necessity) | VERIFIED |
| 40 | `AGENTS/AGENTS-STEERING.md:361` | "so a request that flips inside half a second moves the lane **3.4 m**" | from rest under the live law: ½ · 26 · 0.5² = **3.25 m**; the paragraph's own 14 m/s² would give **1.75 m**. Neither is 3.4 — the figure matches no build of the pair this file states | **probable** (derived number, no live instrument) | PROBABLE |
| 41 | `Racer.cs:1752-1753` (src comment) | "The blend is linear from peak × **0.5** … to peak × **0.8** (the countersteer is at full authority)" | the live pair is `SlideBlendStartFraction 1f` / `SlideBlendFullFraction 2.5f` (`Racer.cs:1754-1755`), consumed at `:818`. The comment describes a band that was replaced and is the only place in `src/` that still states it | **wrong** (src comment) | VERIFIED |
| 42 | `Racer.cs:347` (src comment) | "five miles an hour over costs…" + `:348` "…a fifth of the factor" | `Racer.cs:2330` computes `0.2 × (2.2352 − 0.89408) / 2.2352 = 0.12` at five mph over — **0.6 of a fifth**. The comment's own arithmetic does not produce its own claim | **probable** (comment, not behaviour) | PROBABLE |

---

## Group 4 — state tags (closed / owed) against the code (7 rows)

| # | Doc (file:line) | Claim as written | Code says (file:line) | Verdict | Conf. |
|---|---|---|---|---|---|
| 43 | `AGENTS/AGENTS-BACKLOG.md:20` (`track-list-index-throw`) | "**closed**": the index is parked at 0 before the swap, guarded on the outgoing count | `if (_trackListItem.Items.Count > 0) _trackListItem.SelectedIndex = 0;` immediately before `_trackListItem.Items = names;` (`AutosportRacingSystem.cs:1580-1581`) — parked, guarded, and an empty list cannot set an index | **verified closed** | VERIFIED |
| 44 | `AGENTS/AGENTS-BACKLOG.md:38` (`rival-publish-core`) vs `:58` (`rival-walls-500ms-snapshot`) vs `:167` | `:38` **CLOSED (driver)**: "the half-second-old rival record is fine as it stands, so the publish stays on its own 500 ms beat" — and `:167`, in the relocated index, files the same item as "**not built**"; `:58`, two entries later in the ladder, names that same record as "the most likely remaining cause of 'never the same corner, never the same lap'" and is **filed, not fixed** | the beat is real and unchanged: `_rivalInfoTick + _phaseOffsetMs < now` → `_rivalInfoTick = now + 500` (`Racer.cs:4542-4545`), while every consumer it feeds (`ApplyRivalWalls`) runs each core tick | **closed-but-open** — three statements about one mechanism across two files, and the code supports the *open* one | VERIFIED |
| 45 | `docs/OPEN-ITEMS.md:60` | "`AGENTS/AGENTS-TECHNOTES.md:126` still describes the ABS guard as gliding at `AbsBrakeFallPerSecond` 0.2/s … Falsify with `grep -r AbsBrakeFallPerSecond src` — it returns nothing" | grep returns **0 hits**; `AGENTS/AGENTS-TECHNOTES.md:126-134` still carries the paragraph | **verified, still open** — a correctly filed drift; it is the one doc that names its own falsification test and passes it | VERIFIED |
| 46 | `docs/OPEN-ITEMS.md:32` | `avoid-hold-timeout` "compile-verified and **not driven**; hold a rival alongside past the timeout and check the allow reopens" | `bool avoidHoldEnabled = false` → `if (!avoidHoldEnabled) return passLane;` (`Racer.cs:1499-1501`), and every write to `_avoidHoldActive` sits below that return (`:1505`, `:1516`, `:1528`) or in the clears (`:2057`, `:4904`). The constants carry `// ZOMBIE (2026-10-25): unused while avoidHoldEnabled is false` (`:1483`, `:1487`) | **closed-but-open** — an owed drive on an unreachable branch. Either the drive is dead or the zombie is; the ladder should say which | VERIFIED |
| 47 | `AGENTS/AGENTS-BACKLOG.md:29` (`constant-radius-generator`) | "**closed (driver)** … the generator is `GenerateBezier` (`AutosportRacingSystem.TrackCreator.cs:268`)" | `public static List<Vector3> GenerateBezier(...)` at `AutosportRacingSystem.TrackCreator.cs:268` | **verified** | VERIFIED |
| 48 | `AGENTS/AGENTS-BACKLOG.md:83` (`saveroute-precision`) | "implemented (`Math.Round(v.X, 4)` at `TrackFile.cs:132,137,142`)" | `Math.Round(v.X, 4)`, `(v.Y, 4)`, `(v.Z, 4)` at `AutosportRacingSystem.TrackFile.cs:132,137,142` | **verified** | VERIFIED |
| 49 | `AGENTS/AGENTS-BACKLOG.md:15` (`settings-key-symmetry`) | "they cannot drift silently — the checker cross-checks all four **on every build**" | `NewRacingSystem.csproj:154-157`: `<Target Name="CheckSettingsKeys" BeforeTargets="CoreCompile">` runs `python docs\check-settings-keys.py`; a non-zero exit fails the compile | **verified** (mechanism); the checker's own coverage was not re-scoped here — `AUDIT 06` owns that | VERIFIED |

---

## Group 5 — checked and sound (7 rows)

Recorded so the covered set is honest; these cost nothing to leave alone.

| # | Doc (file:line) | Claim as written | Code says (file:line) | Verdict | Conf. |
|---|---|---|---|---|---|
| 50 | `AGENTS.md:121` | "`Handling.Downforce` reads `0x0014` — the code is right; don't restore the curve-fit" | `public static float GetDownforce(Vehicle vehicle) => ReadHandlingFloat(vehicle, 0x0014);` (`VehicleMemory.cs:37`) | **verified** | VERIFIED |
| 51 | `AGENTS.md:131` | "The grip divisor in `SteerDampingFor` is `max(BaseMechanicalGrip, 1)` **as a denominator**, so the term is weaker on high-grip cars" | `return SteerDampingEnabled ? ARS.SteerDampingGain / Math.Max(VehicleData.BaseMechanicalGrip, 1f) : 0f;` (`Racer.cs:1797`) | **verified** | VERIFIED |
| 52 | `AGENTS.md:132` | "`SteerCeilingFactor` 1.00 puts the outer wheel on the first [crossing]: it sizes the command at **4/3 of the peak**" | `EngineSteerWheelOuterShare = 0.75f`, `PeakSlipOuterCommandShare = 1f / EngineSteerWheelOuterShare` = 1.333 (`Racer.cs:1810-1811`), applied by `PeakSlipCeilingAt` (`:1866-1868`) | **verified** | VERIFIED |
| 53 | `AGENTS.md:133` | "Driven once: swerves read slower, then parked off (`SlipPeakCeiling`)" | `static readonly bool SlipPeakCeiling = false;` (`Racer.cs:1899`) with the allowance update returning early on it (`:1911`) | **verified** | VERIFIED |
| 54 | `AGENTS.md:124` | "Offroad gravity needs its baseline reset before the multiply, or it stacks across races" | `Handling.Gravity = 9.8f;` then `if (hasOffroad) Handling.Gravity *= 1.2f;` (`Racer.cs:644-645`) — assigned, then multiplied | **verified** | VERIFIED |
| 55 | `docs/KNOWN-BUGS.md:27` (`apex-stays-referenced`) | "`NearestApexAhead` only offers corners with `DetectedRadius < 50 m` and `metres >= 0`, so … it returns `null`, nothing replaces the node … A null corner makes `ComputeCornerLane` return 0" | `if (corner.DetectedRadius >= IdealLineMaxRadiusMeters) continue;` (`Racer.cs:1243`, `IdealLineMaxRadiusMeters = 50f`, `:1218`); `if (metres < 0f \|\| metres >= bestMeters) continue;` (`:1249`); the adopted path also requires `metres >= 0f` (`:1251`); `CornerPoint c = lineCorner; if (c == null) return 0f;` (`:1276-1277`); `_lineCornerNode` is written only at `:1351` (under `wantsPosition`) and cleared at `:2069` | **verified** — the bug is described exactly, including its release condition | VERIFIED |
| 56 | `docs/KNOWN-BUGS.md:9` (`first-corner-eagerness`) | "The apex line's window is **300 m of route** plus the `IdealLineSpeedMarginMph` speed floor. The floor is `apex speed − 15 mph`" | `IdealLineWindowMeters = 300f` (`Racer.cs:1203`), gated at `:1318`, `:1347`, `:1364`, `:1377`; `fastEnoughForLine = speedMps > ApexSpeedWithDownforce(...) - ARS.MphToMps(IdealLineSpeedMarginMph)` (`:1346`, `:1221`) | **verified** | VERIFIED |

---

## What is not provable from files

1. **Which value the driver's last tuning pass intended**, where a doc and its own commit disagree (`#1-4`, `#7-8`).
   The commit messages are the only evidence and they side with the code (`5635848` "lane mover at 26 and 24",
   `d22cad6`/`0e86433` "the brake learner corrects in one lap", `05ea488` "stability empties in half a second"). A
   `git show` of the commit that edited the doc without editing its sentence is what makes this a doc defect rather
   than a code defect; no drive can add to that.
2. **`#40`'s 3.4 m** — derivable from the law but not attributable to a build: the log line `Lane mover` prints
   `rate` and `aimed` once a second per car (`Racer.cs:1176-1179`), so a single second of a half-second request
   during a drive would settle it. Nothing in the tree records the measurement the sentence claims.
3. **`#44`** — whether the 500 ms record is "fine" is a driving verdict, not a file one. The reading settles only
   that the two items contradict each other on the same mechanism; `AUDIT 04` (W1) already carries the reproduction
   argument for the open side.

## Residuals this pass did not cover

- `AGENTS/AGENTS-STEERING.md` is 926 lines and `AGENTS/AGENTS-TECHNOTES.md` 584; **the constants were swept mechanically for all
  189 doc-named declarations, but the prose was not read end to end** — rows were found by following citations, so a
  wrong claim that cites nothing is outside this pass's net.
- `AGENTS/AGENTS-FLAGS.md` (459 lines) and `AGENTS/AGENTS-VANILLA-STEERING.md` (191) were not opened: they cite engine source, not
  `src/*.cs`, so the same extractor does not reach them.
- `docs/STEERING-CONTROLLER-SURVEY.md` (2 747 lines) was excluded deliberately — it is an external survey, and its
  `file.cs:line` hits belong to other projects' code.
- The settings checker's own store allowlist was not re-scoped here (`AUDIT 06` / `check-settings-keys.py`); the
  claim verified in `#49` is only that it runs on every build.
