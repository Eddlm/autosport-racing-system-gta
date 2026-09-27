# ARS code audit — clarity, method size, CPU load, comments

Scope: `src\` — 22 files, 11,313 lines, 372 methods. Written read-only; the status table below tracks the
implementation that followed.
Method: brace-matched method spans + comment-block scan (line spans, not just sentence counts), a full
read of the hot spots, and per-file review passes. Framework cost claims were verified by resolving the
IL of `libs\ScriptHookVDotNet2.dll`; a sample of the review-pass findings was re-verified at the source
before being written down here, and the ones re-verified are those carrying the sharp claims (the RNG
bug, the two native storms, the IL premise, the dead members). Line numbers are anchors, not contracts —
grep the symbol if one misses.

## Status — where the work stopped

| Batch | Commit | State |
|---|---|---|
| 1. Defects (§5.1 items 1–3, §1.4) | `9517976` | landed; its `Tips` change was **reverted** in `342d5d0` |
| 2. Dead members (§1.3) | `b586070` | landed, unverified in game |
| 3. Stale comments (§4.2–4.3) | `f4f5d05` | landed |
| 4a. Perf: repeated natives, hoists, dead writes (§3.1–3.4, §3.6) | `15e9847` | landed, wants a drive |
| 4b. Creator preview loop + `ClosestNodeToPlace` (§3.7–3.8) | `01d5344` | landed, creator wants a check |
| 5. Dead methods + the route probe, 406 lines (§5.2) | `c2ad2de` | landed |
| 7a. `Rival.Update` split into four methods (§2) | `141d712` | landed, wants a drive |
| 6. Comment prose (§4.1) | `e705bf9` + this commit | **partial** — TCS, `SmartTuner`'s brand block and the downforce block done |
| 6b. Whitespace residue (§5.3) | — | **not started** — needs a script go-ahead or a line-range tool; see below |
| 4b. Repeated `Game.GameTime` reads (§3) | `e1f948e`, `c2ee51a` | **partial** — `ProcessAI`, `ProcessTimedAI` and `UpdateStuckCheck` done (14 reads → 3 per car per tick). **`UpdateStuckRecovery` is NOT a site**: its two reads are on mutually exclusive paths and the common case reads the clock zero times, so hoisting there *adds* a read. |
| 4c. Leaderboard per-frame rank + `BestLap()` (§3) | — | **deliberately NOT changed.** `DrawLeaderboard`'s `OrderByDescending(v => v.RaceProgress)` (`AutosportRacingSystem.cs:1970`) re-ranks the *racing* cars every frame, but `RaceProgress` moves every frame, so the re-rank is required rather than accidental — `LeaderboardFinish` (`:1958`) is appended in finishing order and never sorted. `BestLap()` (`Racer.cs:1809`) is a loop over at most a handful of `LapTimes`, i.e. negligible. Only the LINQ allocations (Where iterator + OrderBy buffer + ToList) are avoidable, and swapping in an in-place `List.Sort` would drop LINQ's *stable* ordering so cars tied on progress could flicker row order in the HUD. Not worth that risk for three allocations. |
| 7b. `ComputeTargetSpeed`'s crest/dip law (§2) | `6e2847a` | landed, wants a drive |
| 7c. The remaining splits (§2) | `4146c43` | **partial** — lap counter and `HandleCheats` done (104 → 9 lines: three named handlers behind a dispatcher); `OnTick`, `LoadTrack`, `SaveRoute`, `InitializeMenu` left |

**Still open, in the order I would take it:**

1. **The rest of the comment prose (§4.1).** Compressed so far: the TCS block (`Racer.cs`, 12 lines → 4),
   `SmartTuner.cs`'s brand-palette evidence (10 → 5) and the downforce derivation
   (`AutosportRacingSystem.cs`, 9 → 4). Untouched: the seven steer-ceiling blocks in `Racer.cs`,
   `TrackCreator`'s ground-probe note, and the `SettingsRepair` / `MenyooAppearance` / `VehicleSelector` /
   `DataStructures` / `MenuSettings` / `VehicleCatalog` one-offs. The depth goes to `AGENTS-STEERING.md` /
   `AGENTS-TECHNOTES.md` / `AGENTS-SMARTTUNING.md` with a one-line pointer left behind.
2. **Whitespace residue (§5.3).** 319 lines in 67 runs, untouched — including six whitespace-only lines left
   where `DrawRouteNodes` was (in `TrackCreator.cs`, just above `DrawSection`). The trap: many of those lines
   carry spaces, so an exact-match edit needs their real space counts, which the scan reports.
3. **The remaining splits (§2)**, safest first: `HandleCheats`' three handlers → `OnTick`'s draw/cheat/HUD
   blocks → `LoadTrack` → `SaveRoute` (the 22× `InnerText` idiom first) → **`InitializeMenu` last**, because
   its submenu locals are captured by the item handlers and the staged-spawn slot indices are load-bearing.
   Done so far: `Rival.Update` into four, `ComputeTargetSpeed`'s crest law into `CrestGripSpeedFactor`, and
   `UpdateTrackPosition`'s lap counter into `UpdateLapRegistration`.
4. **CPU items deferred out of batch 4**, each verified real but left for their own pass: caching
   `Game.GameTime` per tick (~20 call sites), the leaderboard's per-frame re-sort and per-row `BestLap()`, and
   the creator's one-`DrawMarker`-per-preview-point.
5. **One deliberate leftover:** `ARS.FindNextCorner` and the `LiveCorner` / `CornerScanNode` pair it drives are
   unreferenced, but `FindNextCorner` is named in `AGENTS.md`'s code map as live static AI math, so removing it
   wants a decision rather than a cleanup side effect.

Deferred out of batch 4 on purpose, each checked first: caching `Game.GameTime` per tick (§3.1, real but
~20 call sites — worth its own pass), the leaderboard's per-frame re-sort and per-row `BestLap()` (§3.6),
the track-creator raycast storm (§3.7, batch 4b), and the flip-only handbrake write — which was **not**
done because `Vehicle.HandbrakeOn` has no getter (verified by IL), so a cached flag could never notice
the game clearing the handbrake on a respawn, and the grid-wait handbrake is load-bearing.

Two corrections to this report, found while implementing it:

1. §1.3 said `using System.Linq;` was unused in `TrackFile.cs`. It was used — for two
   `World.GetAllProps().ToList()` copies. Those copies are gone (§3.10), which is what makes the using
   genuinely unused.
2. AGENTS.md calls `VehicleData.TextPerformanceIndex` write-only. It is **read** — the leaderboard's PI
   column prints it (`AutosportRacingSystem.cs:1955`, `:1968`), so only `LapTimes` is still write-only.

---

## 0. Headline — what is actually worth acting on

Ranked by value per unit of risk. Items 1–5 are defects or work whose result nobody reads.

| # | Item | Where | Kind |
|---|---|---|---|
| 1 | `random(0, Count - 1)` against an **exclusive-max** RNG — the last style / livery / mod option / palette colour of every list is unreachable | `SmartTuner.cs:336,338,361,479,540` | **bug** |
| 2 | `UpdateRivals` sorts the field with **4 natives per comparison** to fill 3 slots | `Racer.cs:3244-3264` | CPU |
| 3 | `Rival.Update` spends ~10 `Velocity` natives to obtain 2 vectors | `DataStructures.cs:190-248` | CPU |
| 4 | `DrawText` measures every string with 3 natives whose result **no caller uses** — hundreds of wasted natives per frame with the leaderboard on | `AutosportRacingSystem.cs:3315-3318` | CPU, free win |
| 5 | Per-frame work whose result nobody reads: `activeCorner`, `CurveRadiusAfterFollowPoint`, `SpeedVector`, `Vehicle v = player.LastVehicle` | `Racer.cs:1460,2461,1929`, `AutosportRacingSystem.cs:1586` | dead + CPU |
| 6 | `Car.Position` / `Car.Velocity` read **inside loops** — up to one native per track node | `Racer.cs:2294-2302,2345-2366,3107-3113` | CPU |
| 7 | Sort-and-allocate to find a minimum, called per frame from 3 live sites | `AutosportRacingSystem.cs:2808` | CPU + clarity |
| 8 | Creator rebuilds the whole preview arc + up to 4 raycasts per point + 1 `DrawMarker` per point, every frame | `TrackCreator.cs:173-178` | CPU |
| 9 | ~370 lines of unreferenced methods, plus ~15 dead locals/fields | §5.2, §1.3 | dead code |
| 10 | `InitializeMenu` is 536 lines / 309 statements | `AutosportRacingSystem.cs:678` | size |
| 11 | Comment prose: 77 blocks of 3+ lines, 36 with 3+ sentences; the design rationale belongs in the companions | §4 | comments |

---

## 1. Variable and method clarity

### 1.1 Dishonest method names

The project's own rule — *"Method names must be honest — extra filtering/computing → refactor"* — has
these live violations:

- **`Racer.cs:2259 ApplyRivalThrottleCap`** also writes `Brain.CurrentIntention.Speed` (`:2282`). It is a
  follow-distance *limit*, not a throttle cap. Widen the name (`ApplyRivalFollowLimits`) or split the
  speed write out.
- **`TrackLoader.cs:463 BuildTrackLimits`** builds no limits at all: it re-detects point-to-point via a
  `ref bool` (duplicating `LoadTrack:143`), reads the flare colour, spawns the start-line flare pair and
  clears focus. Rename `SpawnStartLineDecorations` and drop the `ref`.
- **`TrackCreator.cs:352 PlayerOrCameraNearPos`** tests exactly one position per branch, and the branches
  read inverted for the name: when the freecam **is** active it measures the **player**; when it is not,
  it measures the **camera**. Given the freecam is the editing surface, either the live branch measures
  the wrong thing or the name is wrong — decide which.
- **`FreeCamController.cs:166 DrawInstructions`** returns "the scaleform is loaded", and the caller (`:41`)
  uses that return to disable every freecam control for the frame. `TryDrawInstructions`, or split the
  load check out.
- **`SmartTuner.cs:322 PickStyle(..., out int liveryIndex)`** returns the style *and* a livery index out of
  band — two unrelated results — and the caller then indexes a *different* list with it (`options[livery]`,
  `:310`). That 1:1 coupling between `names` and `options` is stated nowhere.
- **`TrackRepository.cs:51 fullWidth`** holds a **half**-width (`XML Wide` is the half-width —
  `TrackLoader.cs:223`, `TrackCreator.cs:204`). The name is the inverse of the truth.
- **`VehicleCatalog.cs:93-97` vs `:128-133`** are two copies of key canonicalisation that have already
  diverged (in-place pool rewrite + null slot vs a null return). Have the loop call `CanonicalKey`.
- **`TrackFile.cs:56 UpdateRoute(bool path, bool raceline, bool props)`** — `raceline` only ever ORs with
  `path` (`:59`), so one job is split into two names plus three boolean params. (Also dead, §5.2.)

### 1.2 Cryptic or misleading locals

- **`TrackCreator.cs:59,66-68,79,95` — `cool`** is a tri-state closure status (−1/0/1) named like a
  temperature. `enum CircuitClosure { None, Open, Closed }` would remove the comments that explain it.
- **`FreeCamController.cs:69,77-78,128,208` — `speed`** is a per-frame *step* (0.01/0.02 m, +0.05 while
  boosting), later fed to `ApplyMovementDecay`. `reduce` (`:74`) means "no movement key held this frame".
- **`Racer.cs:1447-1452`** — `ccStart/ccMid/ccEnd`, `cornerAggression`, `cornerEffectiveDelta` duplicate the
  route-crest block at `:1404-1413` which names the same quantities `crestStart/crestMid/crestEnd`,
  `routeAggression`. The duplication is the real problem (§2).
- **`DataStructures.cs:194,199,266`** — three divergent "speed gap" quantities with near-homonym names
  (`ForwardSpeedGap`, `absoluteSpeedGap`, `closingLong`) feeding three different laws (`SecondsToReach`,
  `TimeToContact`, `SecondsToHit`). Nothing says which law is which, or which is legacy.
- **`Racer.cs:1591 IdealWheelspinBase`, `:1594 deepest`** — PascalCase locals against the file's camelCase.
- **`TrackCreator.cs:541 GetPerpendicular(a, b, length, bool clockwise)`** — callers pass `(pos, oldpos)`, so
  the reference direction is backwards, and the bool means "which side of the track". `SideOffset(from,
  towards, distance, bool rightSide)`.
- **`GridBuilder.cs:10-13` — `PowerDescendent` / `TopSpeedDescendent`** — a typo **shown to the user** in
  the grid-sorting menu (`AutosportRacingSystem.cs:899-902`), and bare `Power` / `TopSpeed` do not state a
  direction. The store keeps the enum *name*, so a rename needs a `SettingsRepair` migration.
- **`Racer.cs:1908-1930` — `UpdateTickData` is a grab-bag of per-frame state** with no stated scope; several
  of its writes are dead (§3).

### 1.3 Dead locals and fields (verified: no reader anywhere in `src\`)

- `AutosportRacingSystem.cs:1846 LapPos` — a `List<Racer>` allocated in `OnTick` every 200 ms, never read.
- `AutosportRacingSystem.cs:3474 result` + `:3473 lastCar` — `LoadGrid`'s `List<dynamic>`; its only write is
  `result.Add(lastCar)` at `:3651` and nothing reads it. `dynamic` drags the C# binder in for nothing.
- `Racer.cs:1460 activeCorner` — a LINQ scan of `ARS.Corners` per core tick, discarded (§3).
- `Racer.cs:1538-1539 SlopeGripLossK/SlopeGripLossExp` — dead constants; the slope model that used them is gone.
- `AutosportRacingSystem.cs:95 ElectricDriveAtTopSpeed`, `:1548 _shortTickMs`, `:626-627 _freeCamMovement/_freeCamRotation`.
- `TrackCreator.cs:443 chevcolor` and the discarded `col.ToArgb()` at `:441`; dead locals `countmax`/`count`/`lastline`
  (`:368-372`, `:466-470`); the `fidelity` parameter (`:360`) is unused; `dd` (`:370/453`) is redundant with `ph`;
  `oldrWidepos`/`oldlWidepos` (`:495-496`, and `:401-402` in the dead twin) are computed with two
  `GetPerpendicular` calls per node and never read.
- `TrackCreator.cs:18 _bezierStartAnchor` — assigned at `:217`, never read. `:200-201` — the identical
  `World.DrawMarker(ChevronUpx3, …)` issued **twice** with identical arguments, per frame.
- `TrackFile.cs:93` — the dead serialiser assigns `_pathWidth`, the **creator's live width knob**
  (`TrackCreator.cs:19`): a cross-file mutant. Make it a local.
- `Racer.cs:1699 BehindNodeDistance` — private, zero call sites.
- `GridBuilder.cs:22 positions.Reverse()` and `Place` mutate the caller's lists; the caller happens to
  rebuild them first (`AutosportRacingSystem.cs:2138`), so it is latent, not live.
- `TrackVisuals.cs:11 editorActive` cannot vary — the only call site gates on the same flag (`:1629`).
- Unused `using System.Linq;`: `TrackLoader.cs:9`, `TrackFile.cs:7`, `TrackRepository.cs:6`.

### 1.4 Latent null / unguarded dereferences

- **`Racer.cs:1373`** — `ARS.CornerApexSpeed(Brain.Corner.Point, this)` runs when `cornerSpd <= 5`, but the
  only null guard on `Brain.Corner` is the `else if` at `:1366`, which that branch did *not* take. The
  author guards it properly 49 lines later at `:1422`. An NRE here surfaces as an `OnTick error` log.
- **`TrackCreator.cs:355`** — `World.RenderingCamera.Position` with no null check, while the only writer in
  the tree is `AutosportRacingSystem.cs:3020 = null` and `FreeCamController.cs:26` explicitly null-checks
  the same property. Reachable only if a section is drawn with the freecam off (entering the creator turns
  it on at `:33`), so latent — but it is a crash, not a nit.
- **`LoadGrid`'s `SpawnRacer`** (`:3599`) — the `World.CreateVehicle` result is dereferenced (`car.Heading`,
  `car.InstallModKit()`) with no null check inside a `try` that swallows into a log, so a failed spawn reads
  as "Failed to load racer" rather than "no vehicle".

### 1.5 Consistency with the project's own conventions — verified clean

Worth stating so it is not re-checked: `ARS.Clamp` is used 78× against exactly **one** `Math.Min(Math.Max(`
idiom; `ARS.IsBetween` is used 15× against 6 hand-rolled range checks (4 of which are int key-code tests,
where the float API does not fit); and **no `Remap` call site has a descending output range with
`clamp: true`** — the inverted-clamp hazard the memory warns about is not present, including
`Racer.cs:1596-1597`, where `deepest = base − deepening` keeps the output ascending on purpose.

---

## 2. Method size — too many jobs at once

372 methods; **31 over 60 lines, 14 over 100, 7 over 150**. The top of the list, with the jobs each one is
actually doing:

| Lines | Method | Jobs |
|---|---|---|
| **536** | `AutosportRacingSystem.cs:678 InitializeMenu` | Builds **every** menu in one body: Race (track/laps/reverse/spawn/PI trio/grid/start/restart/end), root actions, Other, Track Creator, Settings, General Settings, car pool, nitrous, Menyoo, tips, AI Settings, Advanced, the staged-spawn slot restore, and the final `MenuPool` wiring. |
| **313** | `AutosportRacingSystem.cs:1575 OnTick` | 66 branch points: update check, load task, freecam prop, freecam tick, 4 debug draw groups, nitro probe, nitro top-up, HUD, join marker, countdown, the time-sliced racer batch, per-racer `ProcessTick` + finish handling, position sorting, race end, help queue, cheats. |
| **211** | `TrackFile.cs:204 SaveRoute` | Filename resolve/sanitise/collision; XML scaffolding; UI prompts + input; route writing; prop discovery inline; prop writing; save; list refresh. The `Math.Round(...).ToString().Replace(",", ".")` idiom appears **22×**. |
| **193** | `AutosportRacingSystem.cs:3466 LoadGrid` | Already split into 5 local functions (`LoadVehicleModel`, `RandomizeCarColour`, `CreateDriverPed`, `AddRacer`, `SpawnRacer`) — good names, but nested, so the outer method is still a 193-line scroll. |
| **189** | `Racer.cs:1335 ComputeTargetSpeed` | Apex plan (4 apexes), route speed, NaN guards, hill grip, crest grip **twice**, apex-speed reuse, steer-limited blend, chicane boost, the final `min`, yield cap, chillout standoff, rubber band. |
| **186** | `TrackCreator.cs:55 HandleTrackCreator` | Closure status + visuals, pending-section draw, width keys, per-state help text, node deletion, section apply, aim-preview arc + terrain + markers, start-line preview, half-width sync. |
| **153** | `TrackLoader.cs:23 LoadTrack` | Guards/prompt/fade; teardown of 3 prop lists + 6 statics; a **thread-culture switch**; prop spawn (4 natives each); route parse + width clamp; debug reverse; circuit detection; `GenerateRouteInfo`; `BuildTrackLimits`; player/camera placement. |
| **149** | `Racer.cs:2314 UpdateTrackPosition` | Windowed node search, global rescan fallback, lookahead build, deviation, the whole **lap-registration block** (blips, lap times, peak logging), route radius, apex leapfrog, refill tick, two more radius windows. |
| **149** | `Racer.cs:421 ComputeSteering` | Course error, lane override chain, off-track recovery, lane P + rival repulsion, side-by-side, PD assembly, slide blend, NaN guard — plus a `TryGetSteerContext` local function declared at the *bottom* (`:558`) and called at the *top* (`:423`). |
| **119** | `TrackLoader.cs:230 BuildApexTable` | Region-scan state machine + flush + log, then a wholly separate chicane pass (`:323-347`). |
| **106** | `FreeCamController.cs:24 Update` | HUD reset, ride keep-alive teleport, hide-HUD native, toggle, instruction gate, 6 `DisableControlThisFrame`, time-scale toggle, position/rotation integration, height control, decay. |
| **104** | `AutosportRacingSystem.cs:2243 HandleCheats` | Three unrelated cheat handlers inline; the first is 47 lines of tabular dump. Called **every frame**. |
| **79** | `DataStructures.cs:172 Rival.Update` | 9 jobs: relative offset, combined size, lane occupancy, distance, forward speed gap, `SecondsToReach`, `TimeToContact`, position classification, `SecondsToHit` + `DirectionDiff`. |
| **78** | `Racer.cs:2750 UpdateRouteTarget` | Dead (§5.2) — 78 lines with no caller. |

Structural notes:

- **The section-banner comments in `ComputeSteering` and `ComputeTargetSpeed` are a symptom, not a style.**
  `// --- Lane steer ---`, `// --- Off-track recovery ---` exist because each block is an extractable job.
  Extracting them into honestly-named methods is what lets the banners go.
- **`ComputeTargetSpeed`'s crest/dip grip block is duplicated**: `:1389-1417` (route) and `:1425-1457`
  (corner) repeat the same three steps — point-to-point-vs-modulo window selection, three position
  lookups, then `MapGamma` × 2 + `Max` + floor + `sqrt`. One
  `ApplyVerticalGripFactor(speed, radius, startNode, midNode, endNode)` removes ~25 lines and the
  `cc*`/`crest*` naming divergence with it.
- **`InitializeMenu`'s natural seams already exist** as its banners, and they match the ini groups the
  project already has (`Menu-Race.ini`, `Menu-Settings.ini`, `Menu-Debug.ini`): Race `:689-812`, root/Other
  `:821-845`, Creator `:846-876`, Settings/General `:877-903`, pool `:904-954`, nitrous `:955-964`, Menyoo
  `:965-973`, tips `:974-983`, AI `:984-1104`, Advanced `:1105-1126`, staged-spawn restore `:1127-1180`,
  wiring `:1181-1212`. **Constraint:** `settingsMenu`, `aiMenu`, `debugMenu`, `racersMenu`, `advancedMenu`,
  `cameraMenu`, `creatorMenu` are **locals** captured by the handlers, so a split must promote them to
  fields or pass them through — and the two load-bearing invariants at `:778` (assigning `SelectedIndex`
  fires `ItemChanged`) and `:1127-1128` (the staged-spawn slots) must survive the move.
- **`LoadGrid`/`ComputeSteering`/`UpdateTrackPosition` all use nested local functions** instead of private
  methods. The jobs are named (good) but invisible from the member list, which is part of why these files
  read as huge.`LoadVehicleModel` also re-constructs `new Model(modelName)` twice (`:3482`, `:3486`) for no
  effect.

---

## 3. CPU load

**Framework premise — verified by resolving the IL of `libs\ScriptHookVDotNet2.dll` (2.11.6.0):**
`Entity.Position` / `Velocity` / `ForwardVector` / `Model` / `Exists`, `Ped.IsPlayer`,
`Player.LastVehicle` and `Game.GameTime` are all `Function.Call` natives; `Game.Player` and
`Player.Character` are **not**. `GTA.Native.InputArgument` is a **class**, and every getter above shows
`InputArgument.op_Implicit` in its IL — so **each value-type argument to a native call allocates an
`InputArgument` object**. There is *no* `params`-array allocation for ≤16 arguments (fixed-arity overloads
exist up to 16), and enum dictionary keys do not box on .NET Framework 4.8 (`EnumEqualityComparer` uses a
constrained `callvirt`) — two plausible findings checked and rejected. `Vehicle.HandbrakeOn` has **no
getter** at all: it is a write-only property.

Net: **call count is the currency, and the allocation is one object per argument.** Caching an entity
vector in a local is not micro-optimisation — it removes a native transition *and* two or three heap
objects.

**Frequency, which decides everything below.** `ProcessTick` runs for **all ~20 racers every frame**.
`RunTimedCore` — and therefore `ComputeSteering`, `ComputeTargetSpeed`, `UpdateTrackPosition`,
`UpdatePerceivedGrip` — runs for **6 racers per frame** round-robin (`AutosportRacingSystem.cs:1804`,
`count = Clamp(Racers.Count, 1, 6)`; the `_gameTimeNextInLine` gate at `:1812` is always satisfied at
60 fps, so it gates nothing), i.e. each car's core tick lands every ~4 frames at 20 cars. Rival info is
~2 Hz, maneuvers 1 Hz, apex refill 0.5 Hz. The static native count on the AI path is on the order of
**700–900 per frame** at 20 cars.

### 3.1 Native storms — fix these first

- **`Racer.cs:3244 UpdateRivals` sorts the whole field with 4 natives per comparison, to fill 3 slots.**
  It builds a `List<Racer>` (a native `Position` distance per racer, `:3249`), then
  `candidates.Sort((a, b) => Vector3.Distance(a.Car.Position, hoodPos).CompareTo(Vector3.Distance(b.Car.Position, hoodPos)))`
  (`:3259`) — that is **four `Car.Position` natives per comparison**, ~86 comparisons at 20 cars, plus a
  delegate and a sort buffer, plus `Car.ForwardVector` at `:3258`. Runs 1 Hz per car × 20 cars.
  Fix: precompute `hoodPos` once and keep the 3 smallest **squared** distances in a single pass; no sort,
  no delegate, no `Vector3.Distance` sqrt.
- **`DataStructures.cs:172-250 Rival.Update` spends ~10 velocity natives on 2 vectors.**
  `me.Car.Velocity` is read at `:190, :191, :193, :199, :240, :248`; `RivalRacer.Car.Velocity` at
  `:193, :199, :241, :248`; plus `Position` twice per side (`:186`, and again inside
  `EntityRelativeOffset:177`) and `ForwardVector`/`UpVector` (`:192`, `:248`). ~20 natives per rival × 3
  rivals at 2 Hz per car. Hoisting `mePos`, `meVel`, `rivalPos`, `rivalVel`, `meForward`, `meUp` into
  locals (or passing them in from `UpdateRivalInfo`) removes ~75% of it with **zero** behaviour change.
- **`AutosportRacingSystem.cs:3315-3318 DrawText(Vector2, …)` measures every string with 3 natives and
  returns the size, which all 21 call sites discard.** With `ShowLeaderboard` on and a full grid this is
  on the order of **~100 calls and ~320 wasted natives per frame**. Delete `:3315-3318` and make the
  method `void` — the cheapest real win in the audit.
- **`Game.GameTime` is a native call, read ~4× per car per frame and ~15× per car per core tick**
  (`Racer.cs:1913,2153,2187,2198` on the per-frame path; `2878,2880,2883,2885,2929,2931,2935,2939,2941,
  3024,3050,3053,3060,3079,3085,3097,3133,3202,3204` on the core path; plus 8 in `OnTick`) — on the order
  of **130 native clock reads per frame**. `int now = Game.GameTime;` once per tick, or one static
  refreshed in `OnTick`, is identical within a frame and removes all of them.
- **`Driver.IsPlayer` is called 3–4× per car per frame while `ControlledByPlayer` is already cached**
  (`Racer.cs:1945,1948,1969,2042,2149`; the constructor sets `ControlledByPlayer` from exactly this
  native). `:2149`'s `!Driver.IsPlayer` is additionally redundant with its caller's `if (!Driver.IsPlayer)`
  at `:2042`. Tens of natives per frame for a value that cannot change mid-race.
- **`Racer.cs:2153` writes `Car.HandbrakeOn` every frame per car** — a native `SET_VEHICLE_HANDBRAKE` for
  a value that rarely changes (and, per the IL, it cannot be read back). Keep the last written bool and
  call the setter only on a flip. The line is also the `if/else` that should just be an assignment:
  `Car.HandbrakeOn = Control.HandBrakeTime > Game.GameTime;`.
- **The player racer is re-found by LINQ with a native predicate per racer, twice per frame**:
  `AutosportRacingSystem.cs:1630` (unconditional) and `:1910` (`DrawRaceHud`), both
  `Racers.FirstOrDefault(r => r.Driver != null && r.Driver.IsPlayer)`. Cache it when the grid is built,
  or read `ControlledByPlayer`.

### 3.2 Per-frame work whose result nobody reads

- **`Racer.cs:1460-1462`** — `activeCorner` is assigned from an `ARS.Corners` LINQ scan and never used, per
  core tick (closure + scan). The comment above it claims it "invalidates the corner map"; it does nothing.
- **`Racer.cs:2461`** — `Brain.CurrentPerception.CurveRadiusAfterFollowPoint` is written from a full
  `ComputeRouteRadius(2.5s…4.5s)` node scan **and two** native `Car.Velocity.Length()` calls, and the field
  (`DataStructures.cs:133`) has **no reader anywhere**. Its sibling at `:2460` *is* read.
- **`Racer.cs:1929`** — `Brain.CurrentPerception.SpeedVector` is written from a native and never read;
  `VehicleData.SpeedVectorGlobal` (`:1927`) likewise. Two natives per car per frame for nothing.
- **`AutosportRacingSystem.cs:1586`** — `Vehicle v = player.LastVehicle;` costs a
  `GET_PLAYERS_LAST_VEHICLE` native every frame and `v` is never used in `OnTick`.

### 3.3 Natives inside loops — the hitch class

- **`Racer.cs:3107-3113 ApplyStuckRecoveryOverride` scans the entire route with a native `Car.Position`
  read per node**, and it does so **before** the `_stuckRecoveryAttempts >= 5` and timeout exits, so it is
  paid on every recovery frame while a car is stuck. One node is ~1 m, so that is thousands of
  `GET_ENTITY_COORDS` calls per recovering car per core tick. Search a window around
  `CurrentTrackPoint.Node` instead — the same 13-node window `UpdateTrackPosition` uses.
- **`Racer.cs:2345-2353` and `:2358-2366`** — `Car.Position` read *inside* the loops. The windowed scan
  pays one native per candidate (13–19); the **global rescan pays one native per track node**. The rescan
  only fires past 25 m (`:2355`), so it is the recovery path, but when it fires it is a hitch.
- **`Racer.cs:2294-2302 InitializeTrackPosition`** has the same shape *unconditionally*: `Car.Position` per
  node over the whole route, for every car, during grid setup. Hoisting one `Vector3 carPos` fixes all
  three loops at once.
- **`Racer.cs:2460-2461`** — `Car.Velocity.Length()` is called 4× across two adjacent lines, and `speed`
  was already read at `:2373` in the same method. (`:2472-2473` add more.) Cache it once.
- **`Racer.cs:2274`** — `Car.Velocity.Length()` inside the rival loop in `ApplyRivalThrottleCap`; hoist it
  next to `nearestThrottleCap`, which is already hoisted.

### 3.4 Allocations per car per core tick

- **`Racer.cs:2392-2403`** — a 7-element `ValueTuple` array is allocated per core tick per racer, solely to
  be iterated into a dictionary that was just cleared. Seven direct `LookAheads[...] = …` assignments
  remove the array. (`Clear()`+`Add` itself does not allocate — capacity is kept — so the array is the
  whole cost.)
- **`Racer.cs:2504-2505 UpdateApexLeapfrog` allocates an `int[]` and a `float[]` every core tick**, and
  `:2507` nests `heldNodes.Any(node => … ARS.Corners.Any(corner => corner.Node == node))` — a display class
  per held node plus up to 4 full corner scans. Four `int`/`float` locals passed into `CommitApexQueue`,
  plus the node→corner index below, removes all of it.
- **`Racer.cs:3178` → `AutosportRacingSystem.cs:2698-2709`** — `WheelGripMultipliers(Car).Average()` builds a
  `List<ulong>` **and** a `List<float>` and then boxes a `List<float>` enumerator, per car per core tick.
  The only caller wants the mean: sum the offsets in the existing wheel loop and return a float.
- **`Racer.cs:2561-2568 RefillApexQueue`** (0.5 s per car) rescans all corners and then
  `Sort((l, r) => ForwardNodeDistance(...).CompareTo(...))`, recomputing `ForwardNodeDistance` twice per
  comparison, plus a `List.Contains` per candidate. Precompute the distance once, then select the 4
  nearest in one pass.
- **`Racer.cs:1531`** (conditional) — `ARS.Racers.FirstOrDefault(r => r.Car.Exists() && r.Car == Game.Player.Character.CurrentVehicle)`
  evaluates a `DOES_ENTITY_EXIST` native per racer until it matches, per car per core tick — but only when
  `RubberbandingPct > 0`, and the default is `0` (`AutosportRacingSystem.cs:179`), so normally free.
  Compare handles.

### 3.5 The corner lookup family — one map, five call sites

`ARS.Corners.FirstOrDefault(c => c.Node == X)` / `.Exists(...)` appears at `Racer.cs:1294`
(`HasPassedBrakingTarget`, called 3×), `:2674` (`ApexBrakingSpeed`, 1–4× per core tick from
`ComputeTargetSpeed:1358-1364`), `:2627`, `:664`, `:1740`, and at `:1461` where the result is thrown away.
Each is a closure allocation plus an O(corners) scan. `ARS.Corners` is one entry per **corner region**
(`BuildApexTable` is region-based, `TrackLoader.cs:230-318`) — tens, not thousands — so each individual
scan is small and this is **not** a headline item. It is worth doing because a `Dictionary<int, CornerPoint>`
built beside `BuildApexTable` fixes the whole family at once and removes the closures and the float
comparisons. The same map serves §3.4's nested scan.

### 3.6 Orchestration (`OnTick`) — per-frame LINQ and formatting

- **`AutosportRacingSystem.cs:1819`** — `DebugFocusRacer = Racers.Where(...).OrderBy(r => r.Car.Position.DistanceTo(focusPos)).FirstOrDefault()`
  sorts the field with a native position read per car, **every frame, unconditionally**, while its only
  consumers are three debug draws behind toggles (`:1647-1655`, `Racer.cs:1948/1969`). Gate it on those
  toggles, or move it to the existing 200 ms tick at `:1842`.
- **`AutosportRacingSystem.cs:1974`** — the leaderboard's `Racers.Where(...).OrderByDescending(...).ToList()`
  runs per frame and duplicates the ordering already computed at `:1846-1855` (200 ms). `DrawLapStats`
  (`:1988-1996`) then builds 4 formatted strings per row per frame, and `BestLap()` (`Racer.cs:1863-1871`)
  enumerates `LapTimes` per row per frame. Cache the sorted rows in the 200 ms tick.
- **`AutosportRacingSystem.cs:1847-1854`** — LINQ `Where(...).ToList()` + `OrderByDescending(...).ToList()`
  every 200 ms where one pass would do; `unfinished.Any()` after `ToList()` re-walks the list.
- **`AutosportRacingSystem.cs:1861`** runs its `Where/OrderByDescending` every frame once one car has
  finished. **`:1797`** `Racers.Any()` allocates a boxed enumerator per frame (`Count > 0` is free).
- **`AutosportRacingSystem.cs:1681`** rebuilds `"~y~" + seconds + "s~w~ …"` and issues 3 help-text natives
  every frame while the finish timer runs.
- **`AutosportRacingSystem.cs:1617-1623`** — `World.CreateProp` inline in `OnTick` when the freecam ride is
  missing; it belongs in the freecam's own tick.
- **`AutosportRacingSystem.cs:1725-1730`** issues 6 density natives every frame with a track loaded — this
  is required (`*_THIS_FRAME` natives), not a finding.
- **`HandleCheats`** (`:1881` → `:2248, 2302, 2319`) computes `Game.GenerateHash` plus an `IS_*` native 4×
  per frame for input that arrives once a session. Small, but it is per-frame work for an event.

### 3.7 The track creator — the worst per-frame offender in the tree

While the creator is active (`HandleTrackCreator` runs unconditionally from `OnTick:1613`):

- **`TrackCreator.cs:173 GenerateArc` is rebuilt from scratch every tick**, and inside it `:304-311` runs
  `TryResolveGround` **per preview point** — at 1 m spacing — while `TryResolveGround` (`:322-350`) fires up
  to **4 stacked `World.Raycast` probes** per point. A 100 m aim is ~100 points ⇒ up to 400 raycasts in one
  frame, plus a fresh `List<Vector3>`. Cache the section against the aim point behind a ~1 m deadband, or
  resolve ground at a coarser stride.
- **`TrackCreator.cs:175-178`** — one `World.DrawMarker` per preview point, per frame: hundreds of markers
  competing under the per-frame budget that silently truncates and is shared with everything else. The
  visible symptom will be *other* debug visuals disappearing. `DrawLine` between consecutive points is the
  documented substitute.
- **`TrackCreator.cs:184-188`** — `EditNodeHalfWidths` is cleared and rebuilt with one insert per node every
  frame; track the last synced index instead (as the half-width sync at `:223-237` also should — it
  re-tests the count inside its loop).
- **`TrackCreator.cs:200-201`** — the duplicated `DrawMarker` (§1.3).
- **`TrackCreator.cs:95 → :461 DrawSection → ClosestNodeToPlace`** — the sort of §3.8, per frame.

### 3.8 Sort-and-allocate to find a minimum (live, per frame)

**`AutosportRacingSystem.cs:2808 ClosestNodeToPlace`**:
`PathRoute.OrderBy(p => p.DistanceTo(v)).ToList()[0]` allocates a closure, a comparer and a buffer, and
sorts the whole route to obtain one element; it then **linearly scans again** to recover the index by exact
`Vector3` **float equality** (`:2811`), and `Count - 1` means the last node can never be returned. Live call
sites: `TrackVisuals.cs:14` (per frame, editor route draw), `TrackVisuals.cs:216` (per frame, spectator
chevrons), `TrackCreator.cs:461` (per frame via `DrawSection`). One single-pass min loop returning the index
fixes the CPU, the float-equality fragility and the off-by-one together.

### 3.9 Marker volume (toggle-gated, but budget-greedy)

`TrackVisuals.cs:18-27 DrawRoute` draws up to ~41 markers per frame and `:225-242 DrawEdgeChevronsAround`
up to ~62. Both are gated, so this is a budget problem rather than a CPU one — but per the memory the
marker budget is shared with every other marker the script draws, so these are what starves something
else. `DrawLine` stubs or a cached index window. Aside: `:241` casts `(MarkerType)20` — an unnamed magic
enum.

### 3.10 One-shot / setup — real but bounded

- **`MenyooAppearance.cs:79-114` re-parses the whole Menyoo XML library for every spawned car**
  (`FilesMatching` loads *every* file, then `Apply` loads the chosen one again), inline per car during race
  start (`AutosportRacingSystem.cs:3608`). With N cars × M files this is the biggest single hitch in the
  support files. Build a hash → files index once, and reuse the parsed document.
- **`SettingsRepair.cs:238-267 CompleteOwnedKeys` saves per repaired key** — ~40 create/write/close cycles
  on a fresh install instead of 3. Startup time the user sees.
- **`MenuSettings.cs:57-68 Set`** rewrites the whole file (`File.CreateText` + a `WriteLine` per key +
  `Close`) on every change, and ~22 list/checkbox handlers call it on index change — the known "save per
  scroll". Per keypress, not per frame; a dirty flag flushed on menu close is the fix.
- **`TrackFile.cs:334-344` (LIVE)** — `World.GetAllProps().ToList()` allocates a duplicate of every prop in
  the world, then `AutoGeneratedProps.Contains` / `StartLineFlares.Contains` are **linear scans per prop** =
  O(props × generated). A `HashSet<Prop>` fixes it. The dead twin (`:39-51`) is O(props × nodes).
- **`GridBuilder.cs:45-48`** — sort comparators call a native **inside the comparison** (~2·n·log n natives,
  each allocating its `InputArgument`s). `ARS.ModelAccelCache` / `ModelTopSpeedMphCache` already hold both
  values, and mph/m·s ordering is monotonic, so ranking is unchanged if the comparators read the caches.
  `:58`'s `new Random()` per shuffle is also worth replacing with `ARS.GetRandomInt`: two shuffles in the
  same tick seed identically.
- **`TrackLoader.cs:47` sets `Thread.CurrentThread.CurrentCulture` to en-US and never restores it** — and
  `AutosportRacingSystem.cs:3066` does the same at script init. The effect is that every culture-sensitive
  `ToString`/`TryParse` in the mod silently depends on that global side effect, which is exactly what makes
  the writers' `Replace(",", ".")` workaround dead weight and the `TrackRepository` invariant parses
  inconsistent. Pick one convention (invariant, explicitly) and drop the side effect.
- **`VehicleMemory.cs:109-122 ScanModule`** walks the module byte-by-byte; it is correctly memoised
  (including misses), but its first call happens on the main thread inside a per-frame control write
  (`:46`) — worst case a multi-hundred-ms hitch if a pattern is absent. Anchor the scan.
- **`SetSPLVisibility` (`AutosportRacingSystem.cs:2001-2003`) calls `World.GetAllProps()`** — one-shot per
  track load (`TrackLoader.cs:172`), but a whole-world enumeration during setup.
- **`AutosportRacingSystem.cs:434-435` / `TrackRepository.cs:22-67`** — each track file is loaded and parsed
  3× per list build, and `SaveRoute` triggers the whole-folder refresh with no `Script.Yield()`
  (`TrackFile.cs:411`), so the hitch lands inside the menu's process step.
- **`GetDownforceGsAtSpeed` (`AutosportRacingSystem.cs:3343-3347`)** re-reads `Car.HasBone("spoiler")` (up to
  3 `GET_ENTITY_BONE_INDEX_BY_NAME` natives) and the model's top speed every core tick for facts that cannot
  change mid-race — the same class as the deliberately-cached handling reads at `Racer.cs:351-366`. Cache
  both at `Initialize`.

### 3.11 Measured and cleared — do not spend effort here

- **No disk I/O in any tick path.** `MenuSettings.Get*` only touches the in-memory `ScriptSettings` after
  the first `Load` (`GetValue` is a dictionary lookup + `Convert.ChangeType`; only `Save` creates a file),
  and all three store reads in `Racer.cs` are gated (lap crossing, 1 Hz, 1 Hz). **The backlog line about
  "per-tick store reads in `Racer.cs`" is stale** — the reads are cheap and infrequent; say so before
  spending a cycle on it. The `OnTick:1820` per-frame read is likewise negligible.
- **`Racer.cs:2892` + `:2942`** — `UpdateRivalInfo()` does run twice within the same second (the 1 s and
  500 ms gates coincide), so one of the two calls per second is duplicated work. Low impact, but it is a
  real redundancy rather than a cost.
- **`ProjectAhead`** (`Racer.cs:1934-1937`) re-sums the 10-sample acceleration window on every call
  (`DataStructures.cs:35-44`), and the debug block calls it 3× per frame — toggle-gated, so a running sum
  maintained where the ring advances is the honest fix rather than an urgent one.
- **`Racer.cs:2023 VehicleMemory.GetThrottle(Car)`** runs for every racer every frame but is consumed by
  the launch test only when `ControlledByPlayer && PlayerLaunchTestActive` (`:2028`). Not worth
  restructuring.
- **Clean and cheap, verified:** `TrackLoader.TickQueuedFlares` (empty-queue early-out, no allocations,
  natives only while a flare is pending); `SmartTuner.Tick` (one car per tick); `FreeCamController.Update`'s
  early-outs; `UpdateChecker.TryNotify` (network work on a background thread, **no native in it** — the
  correct threading split); `VehicleCatalog`'s roster discovery (file I/O, no natives, on the load thread —
  correct per the threading contract); `VehicleMemory.FindPattern`'s key concatenation (gated on
  `offset == 0`); `ComputeSteering`'s sub-helpers, `ApplySteerLimits`, `ResolveSteerCeiling`,
  `TranslateSteerToInput`, `TractionControl` and `ConvertSpeedToPedals` (no allocations; 3 rival slots);
  every debug visual is toggle-gated before any geometry is built.
- **Dead and therefore harmless:** `DrawText(Vector3, …)` (`:3292`) and `World3DToScreen2d` (`:3277`, which
  leaks two `Marshal.AllocCoTaskMem` `OutputArgument`s and never disposes them) have **no callers** — a
  live leak avoided only by being unreachable. If either is ever revived, fix the disposal.

---

## 4. Comments

Counts (comment-only line **blocks**, so a 6-line block is one entry): **502 blocks, 77 of 3+ lines, 76 over
200 characters, 36 with 3+ sentences.** By file: `Racer.cs` 196, `AutosportRacingSystem.cs` 170, then a long
tail (`SmartTuner` 23, `DataStructures` 19, `TrackCreator` 15, `SettingsRepair` 13, `TrackLoader`/`Tips` 10 …).

Volume itself is not the problem — the ratio is fine (~8% of lines) and most of it is genuinely
load-bearing rationale. Three specific things are:

### 4.1 Blocks that exceed the project's own one-line rule

Worst first:

| Where | Size | What it is |
|---|---|---|
| `Racer.cs:1557-1568` | 12 lines, 1210 chars, 7 sentences | The TCS wheelspin scale: free-rolling/peak/flat landmarks, why the target sits past the peak, what the plateau drop costs, and the grip scaling. `AGENTS-TECHNOTES.md` material with a one-line pointer here. |
| `SmartTuner.cs:92-101` | 10 lines, 6 sentences | Brand-palette evidence, including a dated note. Belongs in `AGENTS-SMARTTUNING.md`. |
| `AutosportRacingSystem.cs:3327-3335` | 9 lines, 7 sentences | Downforce model derivation with engine line references. `AGENTS-TECHNOTES.md`. |
| `TrackCreator.cs:315-321` | 7 lines, 5 sentences | Three separate facts about the ground probe; one line is enough. |
| `SettingsRepair.cs:10-16` | 7 lines, 5 sentences | Header. |
| `MenyooAppearance.cs:12-18` | 7 lines, 5 sentences | Header. |
| `Racer.cs:871-874`, `:886-888`, `:904-907`, `:923-928`, `:936-937`, `:952-954`, `:960-961` | 4–6 lines each | The whole steer-ceiling family carries a paragraph per method. Correct and hard-won — and exactly what `AGENTS-STEERING.md` exists for. |
| `AutosportRacingSystem.cs:166-171` | 6 lines, 4 sentences | `SteerSlipCeiling`, including negative-value behaviour: rationale, not code. |
| `VehicleSelector.cs:10-15`, `DataStructures.cs:63`, `MenuSettings.cs:47-48`, `VehicleCatalog.cs:68-70/78-80`, `SmartTuner.cs:55-58/129-137/201-203/375-380` | 3–6 lines | Same class. |

### 4.2 Stale comments that now contradict the code

These are the ones that will actively mislead:

- **`AutosportRacingSystem.cs:158`** — `// On = route curvature limits speed …; off = the corner braking plan
  alone.` describes a **boolean toggle that no longer exists** (removed in `70e177c`) and sits above
  `CornerOffsetMph`, an int. Delete.
- **`AutosportRacingSystem.cs:189`** — `// Flat mph added to every racer's intended speed plan…` is an
  **orphaned comment**: the field it described is gone, and the next declaration at `:192` has its own.
- **`AutosportRacingSystem.cs:143`** — `(Settings.ini [RACERS])` names a section that no longer exists
  (per-menu `Menu-*.ini` now).
- **`TrackFile.cs:12-17`** — three false claims in a 6-line header: "UpdateRoute and SaveRoute have no
  callers" (SaveRoute is the creator's live save), "SaveRoute is the dead 'new track from scratch' path",
  and "Nothing in this file may write to `Tracks\`" (`:406` does, deliberately).
- **`TrackCreator.cs:12`** — "Saving the recorded route is still unwired" is false; `:38-53` call `SaveRoute`.
- **`Racer.cs:1542`** — `// Reduces crest-induced grip loss as curvature allows more aggressive traversal.`
  is orphaned: it sits between two dead constants, and the method that follows is `GetFollowPointSlopeAngle`,
  about which it says nothing.
- **`Racer.cs:2006-2008`** — the comment describing the *white clamp marker* sits above the *yaw damper*
  block; the marker it describes is drawn 9 lines later at `:2017`. Move it.
- **`DataStructures.cs:279`** — the input trail is kept "every metre of travel"; `SampleInputTrail` samples
  every **0.5 m** (`Racer.cs:2062`).
- **`DataStructures.cs:121`** — names `RivalInfoUpdate()`; the method is `UpdateRivalInfo` (`Racer.cs:2242`).
  Grep finds no `RivalInfoUpdate`.
- **`MenuSettings.cs:7`** — "Never `Load()` the same file twice" while `LoadSettings` loads
  `Menu-Settings.ini` twice (`AutosportRacingSystem.cs:3075`, and again through the store at `:3081`).
  Harmless today, but it is the exact invariant the comment exists to protect.
- **`TrackVisuals.cs:30-32`** — the second sentence, "Unchanged racing visuals.", is diff residue that
  states nothing.

### 4.3 Comments that restate the code

Sweepable in one pass: `TrackVisuals.cs:115-117` (`// green = entrance` above `startColor`) — and the same
three lines are **missing** from the identical block at `:139-141`; `SmartTuner.cs:532-533` repeats the
rationale already given at `:412-413`; `SmartTuner.cs:352-353` restates the `>= 75` skip directly below it;
`MenyooAppearance.cs:55` restates the `Directory.GetParent` two lines down; `DataStructures.cs:180`
restates the ternary; `TrackVisuals.cs:229`'s trailing `// odd nodes only…` is the one that earns its keep.

**Commented-out code**: exactly two places — `Racer.cs:2090-2091` (the disabled corner scan and route
probe) and the corresponding empty scaffolding in the creator (`:419-426`, an `if` with an empty body).
Nowhere else in `src\`, which is unusually disciplined.

### 4.4 Comments worth keeping as they are

`TrackFile.cs:20-22`, `:409-410`; `TrackLoader.cs:139-142`, `:472-475`, `:495`, `:541`;
`TrackCreator.cs:36-37`, `:103-105`; `Racer.cs:136-144`, `:1104-1107`; `VehicleCatalog.cs:135-136`;
`SettingsRepair.cs:42/63/71-72`; `TrackVisuals.cs:229`; and `AutosportRacingSystem.cs:3005`
(`// Random.Next semantics: max is exclusive…`) — which is not only accurate but is the source that proves
the `SmartTuner` bug below.

---

## 5. Findings outside the four dimensions

Surfaced because the audit read the code; each is worth a decision.

### 5.1 Defects

1. **`SmartTuner` treats an exclusive-max RNG as inclusive-max** — the last element of every list is
   systematically unreachable. `ARS.GetRandomInt(min, max)` is `_random.Next(min, max)`
   (`AutosportRacingSystem.cs:3006-3009`) and its own comment states the convention: *"max is exclusive, so
   an index bound is the length itself, never length - 1."* Against that:
   - `SmartTuner.cs:336` `styles[random(0, styles.Count - 1)]` → the last style is never picked
   - `:338` `options[random(0, options.Count - 1)]` → the last livery in each style group
   - `:361` `veh.SetMod(slot, random(0, count - 1), false)` → the last mod option of every slot
   - `:479`, `:540` → the last palette colour, the last matte colour
   - `:360` `random(0, 99) >= 75` skips 24 of 99 values (24.2%), not 25%

   `VehicleSelector.cs:58` and `Racer.cs:414` get the convention right and document it, so both conventions
   currently coexist. Fix: pass the length; for `:360` pass `100`.
2. **`Tips.cs:135` (and the `Denominator` at `:113-124`) is off by one.** `GetRandomInt(1, N) != 1` yields
   1/(N−1), so Medium is 1-in-49 rather than 1-in-50. Either `GetRandomInt(1, N + 1)` or rename so the
   off-by-one is explicit.
3. **`TrackFile.cs:322-323` (LIVE) — `int.TryParse(NodeHalfWidths[i].ToString(), out W)`** round-trips a
   float through a string to get an int: a fractional half-width fails to parse and `W` becomes **0**, and
   because `W` is declared outside the loop, a node **missing** from the dictionary silently inherits the
   *previous node's* width. Use `(int)NodeHalfWidths[i]` with an explicit fallback and no carry-over.
4. **`TrackFile.cs:91-105` vs `:321-323` — two divergent `<Wide>` writers.** `SaveRoute` writes node *i*'s
   width; `UpdateRoute` writes node *i−1*'s (the off-by-one the memory records), and its `wide == 0` branch
   (`:94-99`) is unreachable because `W` starts at 5 and `_pathWidth` is floored at 1 (`TrackCreator.cs:108`).
   Since `UpdateRoute` is dead (§5.2), deleting it removes the divergence; if it ever returns, both should
   share one writer.
5. **`TrackLoader.cs:219` and `LoadTrack:159-169` hardcode node indices** (`RouteNodes[10]`, `[5]`,
   `[1][2][3]`), so a route shorter than 11 nodes throws — while `SaveTrackFromCreator` only requires 2
   (`TrackCreator.cs:45`). A hand-made or truncated track file can crash the load path.
6. **`UpdateChecker.cs:42/55`** — `_latestTag` is written on the background thread and read on the main
   thread with no barrier (benign: a stale read only delays the notify); `:29` sets
   `ServicePointManager.SecurityProtocol = Tls12`, replacing the process-wide default rather than OR-ing it.
7. **`MenuSettings.cs`'s stated invariant vs `LoadSettings`** — see §4.2.

### 5.2 Dead code inventory (all verified: no call site anywhere in `src\`)

| Method | Lines | Notes |
|---|---|---|
| `TrackFile.cs:56 UpdateRoute` | 148 | Referenced only from comments. Its callee `FindCustomProps` (`:35`) is reachable only from here. |
| `TrackCreator.cs:360 DrawRouteNodes` | 96 | A near-duplicate of the live `DrawSection:458`, differing in range (±50/±100), stride (5/6) and whether crossing lines are drawn — keep one. |
| `Racer.cs:2750 UpdateRouteTarget` | 78 | Referenced only from the commented-out line at `:2091`. |
| `TrackRepository.cs:69 ReadTrackStartPosition` | ~17 | No caller. |
| `TrackFile.cs:35 FindCustomProps` | ~20 | Only from dead `UpdateRoute`. |
| `Racer.cs:2105 UpdateCornerValidity` | 4 | Referenced only from the comment at `:2090`. |
| `Racer.cs:1699 BehindNodeDistance` | ~7 | No caller. |

**≈ 370 lines.** The dormant state they drive (`LiveCorner`, `CornerScanNode`, `RouteTargetNode`,
`RouteTargetRadius` — `Racer.cs:34-38`) is annotated as deliberately retained, so this is a decision, not an
oversight: either cut the lot (git keeps it) or delete the call-less wrappers and leave the state with an
honest comment. What should not stay is the current state of affairs, where the dead twin of a live method
sits 100 lines away with a subtly different rule.

Related: `AutosportRacingSystem.cs:52`'s `Options` enum still declares **`FindCustomProps`** with no menu
item and no handler, which implies to a reader that the method is wired. It is not.

### 5.3 Formatting residue

**67 runs of 3+ blank lines, 319 lines consumed**; 121 whitespace-only lines; 20 lines with trailing
whitespace. Worst: `AutosportRacingSystem.cs:2151` (23 lines), `:2792` (16), `:1597` (15), `:1711` (11),
`:2005` (10). Inside `OnTick` alone there are ~35 lines of it, and `LoadGrid:3572-3578` is a 7-line gap in
the middle of a method. It is almost all residue from deleted code, and it actively hides things — e.g. the
empty `if (ph == end - 1)` body in the creator. `TrackCreator.cs` carries ~90 interior blank/whitespace
lines and `TrackFile.cs` similar runs (`74,140,186,304,354,394,400`).

---

## 6. Suggested order of work

Each step is separately committable. Steps 1–3 need no in-game verification beyond a compile — they cannot
change behaviour except where they fix a bug.

1. **Defects** (§5.1 items 1–3, §1.4): the `SmartTuner` RNG bounds, the `Tips` denominator, the two
   unguarded dereferences, the `TrackFile` width round-trip. Small, isolated, and the RNG one is a real
   user-visible bias.
2. **Free CPU wins** (§3.1–§3.3): `DrawText`'s dead measurement, the dead per-frame writes
   (`activeCorner`, `CurveRadiusAfterFollowPoint`, `SpeedVector`, `LastVehicle`), `Game.GameTime` and
   `Driver.IsPlayer` caching, the hoists out of the three loops, and the two native storms
   (`UpdateRivals`, `Rival.Update`). All mechanical; none can change behaviour.
3. **Dead members and residue** (§5.2, §1.3, §5.3): the ~370 dead lines, the ~15 dead locals/fields, the
   319 blank lines. Doing this first makes every later diff smaller — and the memory records that
   exact-match edits driven from a `line=length` map are the fail-safe way to cut large regions.
4. **Stale comments** (§4.2): the ones actively costing future sessions time.
5. **Comment prose migration** (§4.1): move the paragraphs into `AGENTS-STEERING.md`,
   `AGENTS-TECHNOTES.md` and `AGENTS-SMARTTUNING.md`, leaving one-line pointers. Biggest readability win
   per edit, zero runtime risk.
6. **Method splits** (§2), in this order of value/risk: `ComputeTargetSpeed`'s duplicated crest block →
   `Rival.Update`'s three jobs → `HandleCheats`' three handlers → `OnTick`'s draw/cheat/HUD blocks →
   `UpdateTrackPosition`'s lap block → `LoadTrack` → `SaveRoute` (the 22× `InnerText` idiom first) →
   `InitializeMenu` last, and carefully (see §2's constraint on the menu locals).
7. **Two structural fixes that want their own build and a drive**: the node→corner index for
   `ARS.Corners` (§3.5), and a once-per-frame cached entity snapshot per car (`pos`/`vel`/`forward`/`up`)
   that the whole per-racer pipeline reads instead of re-calling natives. The second is the only change
   here that touches the entire pipeline.
8. **The creator's preview loop** (§3.7) — worth its own pass, since it is the one place a user can feel the
   frame cost directly, but it is also the code most likely to be rewritten for other reasons.
