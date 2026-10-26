# The lane vocabulary pass — the working map

One file, written in turns. The Lead seeds the map; the reviewer annotates it **in place** — inline, or in
its own `> REV:` blocks — and the Lead resolves every annotation before a single rename lands. Only one of
us writes this file at a time; the other reads it first. The reviewer's usual read-only brief is relaxed
**for this file only**; everything else in the tree stays read-only to it.

> **Read the last table first.** The map's own "Proposed" column is the draft that was put to the reviewer,
> and four of its proposals were refused in favour of better names. **The settled rename at the end of this
> file is what landed**; the resolutions table above it records what happened to every annotation. The old
> names survive here on purpose, as the draft and the record. A review pass found a handful elsewhere, and they
> are dealt with: three parameter names in `Racer.cs` that the pass had missed, a few historical or stale
> mentions in the docs, and a second copy of the source under `Util\shvdn-compat\build-2022api\`, which is a lab
> copy and not the tree.

## Why this pass

Four different things were called "lane", two of them were called "target", and the car's own offset exists
in two frames without either name saying which frame. The worst one: `TargetLane` is the **follower**, not
the goal — so "target" in this codebase means the aim rather than what anyone asked for, which reads
backwards to a newcomer and is why the reason tag (`LaneRequestReason`) and the property (`TargetLane`)
look like they describe the same number when they describe two.

## The quantities, and the names proposed

| # | What it is | Today | Proposed | Note |
|---|---|---|---|---|
| 1 | Where the car is, in the steer reference's frame | `carOffset` (`Racer.cs:547`, passed at `:553`, `:568`) | `referenceOffset` | the chain, the mover, the lean and the pursuit all use this one |
| 2 | Where the car is, in the current track node's frame | `Brain.CurrentPerception.DeviationFromCenter` (`DataStructures.cs:123`, set `Racer.cs:2966`, read `:998`, `:2865`, `:3815`, `:3918`, `DataStructures.cs:200`, `:276`, `AutosportRacingSystem.cs:1808`) | `CurrentNodeOffset` | the debug green line and a rival's occupied lane read this one; in a bend the two frames differ by about the lookahead squared over the radius |
| 3 | What the chain asks for | `_laneRequest` / `LaneRequestReason` (`Racer.cs:155`) | unchanged | already the right word, and the reason tag belongs to it |
| 4 | What the mover carries, and what steering aims at | `_targetLane` / `TargetLane` (`Racer.cs:154`) | `_aimedLane` / `AimedLane` | the prose already says "the aimed lane" everywhere |
| 5 | The offset the aim point carries, after the Gs lead | `aimLane` (`Racer.cs:575`, `:579`, `:582`) | `aimPointOffset` | a fifth thing, absent from the user's list of three |
| 6 | The world point the pursuit steers to | `laneAimPoint` (`Racer.cs:583`) | `aimPoint` | a point, not a lane |
| 7 | Whether the lane is an instruction rather than the car's own lane | `_targetLaneAimed` (`Racer.cs:569`) | `_laneIsInstruction` (?) | its only reader is the offshoot cap |
| 8 | The high-speed source's answer | `TryComputeHighSpeedLane` (`:813`), `HighSpeedLane*` (`:742`, `:743`, `:746`, `:747`, `:750`), `highSpeedLane` local (`:553`) | `...CurveLean...` (?) | the docs and the reason enum call this "the lean"; only the code says "high speed" |
| 9 | The corner line's answer | `ComputeCornerTargetLane` (`:937`) | `ComputeCornerLane` (?) | "TargetLane" in the name carries the stale sense |
| 10 | The commit card's phase | `LaneReason.CommitLine` | `CornerCommit` (?) | names the card while its siblings name phases |
| 11 | Bounds, not lanes | `insideBound` (`:836`), `drivableEdge`, `ClampLaneToWalls`, `ApplyRivalWalls`, `_avoidLeftWall` / `_avoidRightWall` | unchanged | note only: these are limits that lanes are clamped into |
| 12 | The mover, its flag, its rate, its snap | `UpdateLaneMover` (`:860`), `LaneOverrideHoldMeters`, `_laneRate`, `_laneSnap` | unchanged | not lanes |
| 13 | The debug line's own locals | `currentDeviation` / `intendedDeviation` (`AutosportRacingSystem.cs:1808`, `:1809`) | `carOffsetAtNode` / `aimedOffset` (?) | "intended" is the stale word for the aim, and the pair should read as the two rows 1 and 4 draw |

## Open questions for the reviewer

1. **What is missing?** Enumerate the lane-ish identifiers per file — `Racer.cs`, `DataStructures.cs`,
   `AutosportRacingSystem.cs`, and the docs — and say for each whether it is a *quantity* (rows 1-6), a
   *bound* (row 11) or *neither* (row 12). Include the locals: `currentLaneMeters` (`:998`),
   `laneAtProjection` (`:579`), `laneRequest` and `laneReason` in the chain, and anything reached from them.
2. **Which proposals are wrong?** Name any that collide with a term already spent elsewhere in the tree, or
   that a reader would misread in the other direction.
3. **Rows 7, 8, 9, 10** — rename, leave, or a better name? Give one recommendation each with the reason.
4. **The reach.** Which of these names appear in the docs (`docs/DESIGN.md`, `AGENTS/AGENTS-STEERING.md`,
   `AGENTS/AGENTS-TECHNOTES.md`, `docs/TEST-PLAN.md`, `AGENTS/AGENTS-BACKLOG.md`) as *symbols* rather than prose, so the rename
   has to carry them too? And does a pure rename move any line, invalidating an anchor? It should not.
5. **What should be refused?** Which renames would leave a reader worse off than today, and why?
6. **The word "lane" itself.** Is it carrying two senses that should be split — the *position* a car holds
   versus the *target* it is sent to — or is that distinction already carried by the words above?

## Reviewer annotations — task-14

Everything below is the reviewer's, appended only; nothing above is edited, so `grep '^> REV:'` is the set the
Lead has to resolve. Each block ends in the call it asks for.

> REV: map errors. (a) Row 11 cites `insideBound` at `:836`; it is at `:835` — `:836` is the `lane = ARS.Clamp(...)` line. Every other line cite in the table checks out against the file as it stands, rows 1, 2 (including the `:2966` set), 4, 5, 6, 8 (`:813` and all five constants at `:742`/`:743`/`:746`/`:747`/`:750`), 9 (`:937`) and 12 (`:860`) included, so this is one off-by-one rather than drift. (b) Row 10's note asks for a name that "names the card", but its proposal `CornerCommit` names the corner — the proposal contradicts its own note; see REV-row10. Call: fix the `:836` cite and settle row 10.

> REV: row 1 — right quantity, wrong name. `carOffset` at `:547` is correct and the note is right that the chain, the mover, the lean and the pursuit all use it. But `referenceOffset` collides in a direction the map does not check: "reference" is already spent twice in this file's own tree — `SteerDampingAimReference` (`Racer.cs:1147`) and `SteerCapGripReference` (`:1164`) — while the frame's own local is `steerRefPoint`, so `steerRefOffset` says which reference and matches the code. Consumer list for the rename: `:551` (the seed), `:553` (the lean), `:561` (the recovery), `:568` (the mover), `:569` (the flag), `:579` (the preview), `:892` (the interval clamp). Call: rename to `steerRefOffset`.

> REV: row 2 — right, with two additions. The frame diagnosis and the "lookahead squared over the radius" note are both correct and the line set checks out. Add: the quantity has a third name in the tree, `currentLaneMeters` (`Racer.cs:1024`, a parameter of `TryPickAvoidanceSide`, passed from `ComputeAvoidAheadLane`), so a rename has three names to fix, not one. `CurrentNodeOffset` is acceptable — the project does say "node" (`CurrentTrackPoint.Node`, `OffsetCornerNode`) — though the field it renames lives on `CurrentPerception`; the code's own word would be `CurrentTrackPointOffset`. Call: accept `CurrentNodeOffset`, and note `currentLaneMeters` in the row.

> REV: row 3 — right, leave it. `_laneRequest`/`LaneRequestReason` is the one place the word is exact, and folding the tag into this row is right: the tag is not a lane at all, only its owner is. Call: no change.

> REV: row 4 — right diagnosis, name refused. `TargetLane` being the follower is the worst of the fourteen and the complaint is correct. But `_aimedLane`/`AimedLane` is the proposal I would refuse first: "aim" is already a family — `aimLane` (`:575`, row 5), `laneAimPoint` (`:583`, row 6), the `aimPoint` parameter of `PursuitSteerDegrees`, `_steerAimCurvature`, `SteerDampingAimReference` — and the old flag `_targetLaneAimed` (row 7) says "aimed" too, so `AimedLane` would put a fourth "aim" on the very quantity the flag is about. Recommend `_carriedLane`/`CarriedLane`: the docs already use the verb ("the aim is carried onto the request") and it is unused as an identifier; `_appliedLane`/`AppliedLane` is the second option, matching `Control.LastAppliedSteerDegrees` ("the value actually sent"). The property's consumers are only the debug line (`AutosportRacingSystem.cs:1809`) and the flag (`:569`), so the name matters more than the churn. Call: rename to `_carriedLane`/`CarriedLane`, refuse `AimedLane`.

> REV: row 5 — right, keep. `aimPointOffset` is honest, and this row is what makes row 4's "aimed" unnecessary. Add the local it is fed from: `laneAtProjection` (`:578`) is a *third frame* — the car's offset measured at `ProjectAhead(SteerPreviewSeconds)` against the same steer reference — and it has no row; see REV-gaps. Call: no rename; carry the `laneAtProjection` note into the gaps.

> REV: row 6 — right, keep. `aimPoint` matches the parameter it is passed to, `PursuitSteerDegrees(Vector3 aimPoint, ...)`, and it drops "lane" from a point, which is the whole complaint. No collision. Call: rename to `aimPoint`.

> REV: row 7 — rename, but not to `_laneIsInstruction`. The flag's own mechanism already has the right noun two lines away: `LaneOverrideHoldMeters`, and the assignment's comment reads "The aim carries an **override** until it has come back this close to the car's own lane". Naming the flag for "instruction" splits one mechanism across two nouns. Recommend `_laneOverride` (or `_laneOverrideActive`): it pairs with the hold constant and it survives row 4's rename because it no longer needs "target" or "aimed". Reader count is one (`OffshootInputCap`, `Racer.cs:1488`) with the assignment at `:569`. Call: rename to `_laneOverride`.

> REV: row 8 — rename, and the *direction* matters. The docs (`AGENTS/AGENTS-STEERING.md:404`) and the design file call the answer "the lean", and the reason enum's member is `CurveLean`; only the code says "high speed". The rename should move the code to the docs' word, not the reverse: "high-speed line" promises an absolute place, which is the shape the pull replaced. Recommend `TryComputeCurveLean`, the five constants as `CurveLean*` (`:742`-`:750`), the local `curveLean`, and the chord constant left naming the chord (`CurveLeanChordSeconds` — the chord is the lookahead, not the answer). Do not touch `PreciseCurveRadius` (not ours). Alternative if "curve" feels crowded beside `CurveRadius`: `BendLean*` — "bend" is the design file's other word and is unused as an identifier. Refuse the reverse (`CurveLean` → `HighSpeedLine`): it names the answer a line, which the pull is not. Call: rename in the docs' direction.

> REV: row 9 — rename, yes. `ComputeCornerLane` drops the stale sense and lines up with its siblings `ComputeAvoidAheadLane` (`:993`) and `NearestEdgeLane`, which is the shape a reader expects; "TargetLane in the name carries the stale sense" is exactly right. It is cited in four docs (see REV-reach). Call: rename to `ComputeCornerLane`.

> REV: row 10 — the diagnosis is right, the proposal is wrong. `CornerCommit` names the *corner*, so it would read as a third corner-line phase beside `OutsideLine` and `ApexLine` — the confusion the rename exists to remove. The member's job is to say *which writer* wrote the request, and the writer is the card (`ManeuverType.DiveBomb`/`DefendLane`), so recommend `CommitCard`, which also sits with `Avoidance`, `WallClamp`, `DebugLock` and `RecoveryEdge`, the writer/action names. Deeper option, out of this map's scope and its own decision: if every member named its writer, `OutsideLine`/`ApexLine` would become `CornerOutside`/`CornerApex`; I am not proposing that here, only noting that `CommitCard` moves toward it and `CornerCommit` away. Call: rename to `CommitCard`, refuse `CornerCommit`.

> REV: row 11 — right classification, and the row hides the real problem. `insideBound` (`:835`) recomputes `drivableEdge` (`:546`) one line for one; `trackBound` does it again in `ComputeAvoidAheadLane` (`:996`) and as a parameter of `TryPickAvoidanceSide` (`:1024`), and `ClampLaneToWalls` builds `trackBound`/`clampLeft`/`clampRight` a fourth time. One bound, four names — a compute-once defect rather than a vocabulary one, and the vocabulary pass should not paper over it. Call: leave the names, and file the duplication separately (`AUDIT-COMPUTE-ONCE.md` is the pile for it).

> REV: row 12 — right classification, incomplete list. The machinery is wider than the four names: `LaneMoverAccelMps2` (`:850`), `LaneMoverSpeedCapMps` (`:851`), `LaneMoverMaxStepSeconds` (`:854`), `_laneRequestPrev` (`:149`, the feedforward's history), `_laneMoverLoggedAt` (`:151`), `LaneOverrideHoldMeters`; plus, if the crossing rule counts as lane machinery, which I think it does, `LaneCrossingScale` (`:770`), `LaneCrossingShare` (`:778`), `LaneCrossingThresholdDegrees` (`:772`) and `LaneCrossingLateralSpeedMps` (`:767`) — compiled and switched off. Add `LaneLockTestOffsetMeters` (`:753`) with its toggle `Options.LockLaneCentre` (`:565`), and the palette `LaneReasonColor` with its three colour fields (near `:2587`-`:2593`). None of them is a lane; all of them read as one to a newcomer unless the row says so. Call: extend the list, no renames.

> REV: row 13 — right pair, and the names must track whatever rows 2 and 4 become. `carOffsetAtNode` should mirror row 2's choice and `aimedOffset` row 4's (so `carriedOffset` if row 4 is `CarriedLane`), or the debug locals and the quantities part company at the rename. The row also misses the rest of that draw block: `intendedColor`, and the label's `labelNode`/`labelRight`/`labelPos` (`AutosportRacingSystem.cs:1833`-`:1843`), of which `labelPos` is what the on-screen name is anchored to. Call: rename the pair, and note the label locals.

> REV: gaps — the enumeration for open q1, by file, each classified as quantity (rows 1-6), bound (row 11) or neither (row 12). `Racer.cs`: *quantities* — `carOffset` (row 1); `_laneRequest` (row 3) with the chain's own locals `laneRequest` (`:551`), `laneReason` (`:552`) and `cornerReason` (`:556`); `_laneRequestPrev` (`:149`, the request's history that the feedforward reads); `_targetLane` (row 4); `aimLane` (row 5) with `laneAtProjection` (`:578`); `laneAimPoint` (row 6). *Source answers*, only two of which have rows — `highSpeedLane` (row 8), `cornerLane` (`:555`), `avoidAheadLane` (`:559`, from `ComputeAvoidAheadLane` `:993`, no row), `NearestEdgeLane` (called `:561`, no row), `walledLane` (`:562`, the post-wall request, no row), `passLane` (`:1041`, the pass manoeuvre's lane, no row). *Bounds* — `drivableEdge`, `insideBound`, `trackBound` twice, `clampLeft`/`clampRight`, `_avoidLeftWall`/`_avoidRightWall`. *Neither* — the enum `LaneReason` itself (`DataStructures.cs:322`) and its nine members; `laneSteerDeg`, `_steerPursuitDeg`, `crossingDeg` (angles); the mover's constants and state (row 12's list above); `LaneLockTestOffsetMeters` and `Options.LockLaneCentre`; the crossing machinery; the palette. `DataStructures.cs`: `DeviationFromCenter` (row 2); `OccupiedLane` (`:165`) and `OccupiedLaneWidth` (`:166`) — a rival's lane and the pair's combined width, **with no row at all**, and they are what the commit lane and the rival walls actually read; the enum and its members. `AutosportRacingSystem.cs`: `SignedLaneOffset` (`:550`, the measurement itself and the home of the `+ = right` convention, cited in `AGENTS/AGENTS-TECHNOTES.md:79` — neither a quantity nor a bound, and the one name here that must not move without that doc); `currentDeviation`/`intendedDeviation` (row 13) with `intendedColor` and the label locals; `Options.LockLaneCentre`. The largest omission is `laneAtProjection` and the third frame it represents — the preview mixes an offset measured at the car with one measured at the projected point. The second is `OccupiedLane`/`OccupiedLaneWidth`. The third is the sources with no row: `ComputeAvoidAheadLane`, `NearestEdgeLane`, `passLane`, `walledLane`. Call: resolve these into the table before renaming anything, or the pass will leave the same four things called "lane".

> REV: collisions and refusals (open q2, q5). Refuse, worst first: `AimedLane` (row 4) — collides with the whole aim family and with the old flag's word; `referenceOffset` (row 1) — "reference" is spent on two constants; `CornerCommit` (row 10) — names the corner, not the card; `_laneIsInstruction` (row 7) — splits the flag from `LaneOverrideHoldMeters`. Also refuse renaming `LaneReason` and `SignedLaneOffset` (both cited as symbols in docs, and the second is the sign convention's own name), and refuse renaming the bound locals as part of a vocabulary pass — they need deduplication, not new words. Accept: rows 2, 5, 6, 9 and 13's direction, and row 8's direction. Call: the Lead resolves each of these four as accept/refuse.

> REV: reach (open q4). Symbols the rename has to carry, from a tree-wide grep of the docs: `TargetLane` — `docs/TEST-PLAN.md:493`, `AGENTS/AGENTS-STEERING.md:332`, `AGENTS/AGENTS-TECHNOTES.md:366`, `STEERING-SIMPLIFICATION-FINDINGS.md:112`; `_targetLane` — `docs/TEST-PLAN.md:493`; `_laneRequest` — `docs/TEST-PLAN.md:493`; `LaneRequestReason` — `AGENTS/AGENTS-STEERING.md:412`; `carOffset` — `AGENTS/AGENTS-STEERING.md:332-333`; `DeviationFromCenter` — `PLAN-RATE-PRECISION.md:298-299`, `AUDIT-COMPUTE-ONCE.md:109`; `TryComputeHighSpeedLane` and `HighSpeedLane*` — `AGENTS/AGENTS-STEERING.md:404`, `:416`, `:502`, `STEERING-SIMPLIFICATION-FINDINGS.md:186`; `ComputeCornerTargetLane` — `AGENTS/AGENTS-STEERING.md:405`, `:420`, `AGENTS/AGENTS-BACKLOG.md:166`, `AGENTS/AGENTS-TECHNOTES.md:266`, `:323`; `UpdateLaneMover` — `AGENTS/AGENTS-STEERING.md:412`, `docs/TEST-PLAN.md:493`; `aimLane` — `AGENTS/AGENTS-STEERING.md:332`, `STEERING-SIMPLIFICATION-FINDINGS.md:390`; `LaneReason` and `CommitLine` — `AGENTS/AGENTS-STEERING.md:412`, `docs/TEST-PLAN.md:542`; `OccupiedLane` — `PLAN-RATE-PRECISION.md:298`, `AUDIT-COMPUTE-ONCE.md:104`, `:109`; `SignedLaneOffset` — `AGENTS/AGENTS-TECHNOTES.md:79`, `AGENTS/AGENTS-BACKLOG.md:346`, `STEERING-SIMPLIFICATION-FINDINGS.md:54`. `laneAimPoint`, `insideBound`, `laneAtProjection`, `currentLaneMeters` and `LaneOverrideHoldMeters` appear in no doc, so they are code-only. And yes: a pure rename moves no line — identifiers do not change line counts and the docs' anchors are mostly `path:line` numbers a rename leaves alone; the one way to break that is a length change re-wrapping a wrapped condition or comment, so substitute tokens without reformatting. Call: the rename commit carries the docs above, and no re-wrapping.

> REV: the word "lane" (open q6). It carries two senses, but not the two the question names. What the names above already separate is *quantity versus machinery*: the offset sense appears as `...Offset` or a plain local (`carOffset`, `aimPointOffset`, `passLane`), while "lane" as a subsystem or role appears as a prefix (`UpdateLaneMover`, `LaneReason`, `LaneCrossingScale`, `ClampLaneToWalls`). The genuinely overloaded pair is a *place on the track a car holds* (`OccupiedLane`, "the car's own lane") versus *the lateral target the chain asks for* (the request) — but both are the same kind of number, a signed offset in metres, and the request-versus-aim distinction is already carried by `Request`/`Target`/`Carried`. Call: do not split the word; make the quantity names carry the unit, as rows 1, 2 and 5 already do, and leave "lane" to the subsystem the way rows 11 and 12 do.

> REV: already right, plainly. Rows 3, 5, 6, 11 and 12 are right as they stand — row 3 unchanged, `aimPointOffset`, `aimPoint`, the bounds classified as bounds, the mover classified as machinery. "Why this pass" is right as a statement about concepts: the four things are the car's own offset, the request, the aim and the aim-point offset, and the two "targets" are `_targetLane`/`TargetLane` and `ComputeCornerTargetLane`. Row 2's two-frame diagnosis and row 8's "only the code says high speed" are correct, row 13's "`intended` is the stale word" is correct, and the table's line cites are correct except the single off-by-one in row 11. Call: nothing to resolve here beyond the off-by-one.

## Resolutions — the Lead

Every block above is resolved here, in the file's own order. Where a proposal was refused, the block says so.

| Block | Disposition |
|---|---|
| map errors | fixed: row 11's `insideBound` cite is `:835`, and row 10 is settled below |
| row 1 | accepted `steerRefOffset`; refused `referenceOffset`, because "reference" is already spent twice |
| row 2 | accepted `CurrentNodeOffset`; `currentLaneMeters` stands as the third name and is renamed with it |
| row 3 | no change |
| row 4 | accepted `CarriedLane` / `_carriedLane`; refused `AimedLane`, which would put a fourth "aim" on the quantity the old flag was about |
| row 5 | no rename; `laneAtProjection` is recorded as a quantity of its own and named `offsetAtProjection` |
| row 6 | accepted `aimPoint` |
| row 7 | accepted `_laneOverride`; refused `_laneIsInstruction`, which split one mechanism across two nouns |
| row 8 | accepted the direction, to the docs' word: `TryComputeCurveLean`, the five constants `CurveLean*`, the local `curveLean`; refused the reverse, which would name the answer a line |
| row 9 | accepted `ComputeCornerLane` |
| row 10 | accepted `CommitCard`; refused `CornerCommit`, which names the corner and would read as a third phase |
| row 11 | names left; the one-bound-four-locals duplication is filed in `AUDIT-COMPUTE-ONCE.md`, not papered over |
| row 12 | the list extended, no renames |
| row 13 | the pair renamed to track rows 2 and 4 — `nodeOffset` and `carriedOffset` — with `intendedColor` → `carriedColor`; the label locals noted |
| gaps | the enumeration accepted. `OccupiedLane` → `NodeOffset` and `OccupiedLaneWidth` → `CombinedWidth`, since it is the pair's width and not a lane; `SignedLaneOffset`, the unlisted sources, `passLane`, `walledLane` and the enum itself are left alone |
| collisions and refusals | as above; `LaneReason` and `SignedLaneOffset` are left, and so are the bound locals |
| reach | the seven docs are carried, and no line is re-wrapped, so no anchor moves |
| the word "lane" | not split; the quantity names carry the unit and "lane" stays with the subsystem |
| already right | nothing beyond the off-by-one, which is fixed |

## The settled rename

| Today | Becomes |
|---|---|
| `carOffset` | `steerRefOffset` |
| `Brain.CurrentPerception.DeviationFromCenter` | `CurrentNodeOffset` |
| `Rival.OccupiedLane` | `Rival.NodeOffset` |
| `Rival.OccupiedLaneWidth` | `Rival.CombinedWidth` |
| `currentLaneMeters` (parameter) | `nodeOffset` |
| `_targetLane` / `TargetLane` | `_carriedLane` / `CarriedLane` |
| `_targetLaneAimed` | `_laneOverride` |
| `aimLane` | `aimPointOffset` |
| `laneAtProjection` | `offsetAtProjection` |
| `laneAimPoint` | `aimPoint` |
| `TryComputeHighSpeedLane` | `TryComputeCurveLean` |
| `HighSpeedLaneRadiusMeters`, `HighSpeedLaneChordSeconds`, `HighSpeedLanePullMeters`, `HighSpeedLaneFullPullRadiusMeters`, `HighSpeedLanePullReportMeters` | the same five with `CurveLean` in place of `HighSpeedLane` |
| `highSpeedLane` (local) | `curveLean` |
| `ComputeCornerTargetLane` | `ComputeCornerLane` |
| `LaneReason.CommitLine` | `LaneReason.CommitCard` |
| debug locals `currentDeviation` / `intendedDeviation` / `intendedColor` | `nodeOffset` / `carriedOffset` / `carriedColor` |

Left alone on purpose: `_laneRequest` and `LaneRequestReason` (already exact), `LaneReason` and its other members, `SignedLaneOffset`, the bounds and their four locals, the mover's machinery, the crossing machinery, the palette, `ComputeAvoidAheadLane`, `NearestEdgeLane`, `passLane`, `walledLane`, and the debug label's own locals.
