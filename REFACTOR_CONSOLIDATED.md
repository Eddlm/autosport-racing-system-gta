# ARS Refactor — Consolidated Considerations

This is a synthesis of four independent audits (GLM-5.2, DeepSeek V4 Pro, MiniMax M3, Qwen 3.7 Max). It is NOT a plan to implement — it is a list of things worth keeping in mind during the refactor. Items are kept because at least one audit flagged them and they survived a cross-check; where the four disagree, the disagreement is noted.

The three goals remain: (1) simplify, keep behavior; (2) rename for clarity; (3) remove all comments. Bugs and architecture observations are parked in an appendix because they are out of scope.

---

## A. Goal 1 — Simplify (behavior-preserving)

### A.1 Disabled-by-constant blocks — unanimous, delete all
Every audit caught these. They are dead weight and obscure real logic.
- `Racer.cs:743–764` — `#if false` steer-angle speed limiter.
- `Racer.cs:766–781` — `if (Brain.Corner.Valid && 1==2)` outward limiter.
- `Racer.cs:786–798` — "Risk factor for grip" block, result (`speedAdjust`) discarded.
- `Racer.cs:865–887` — `if (… && 1 == 0)` catchup force/grip block.
- `ARS.cs:1372` — `if (1 == 1 || !Brain.Corner.Valid || …)` rocket-boost guard; the `1==1` makes the test tautological.
- `ARS.cs:1570, 1616` — `… || 1 == 1` "radar enabled" gates; the native call is dead, the `||` is always true.
- `ARS.cs:1638–1644` — `if (1==2 && CanWeUse(playerVeh)…)` add-known-model.
- `ARS.cs:2560–2602` — `if (… && 1 == 2)` ghosting block (matches project memory: ghosting was removed).
- `ARS.cs:3805–3831` — `if (hash == … && 1 == 2)` garage/repair block.
- `ARS.cs:5528–5541` — `if (1 == 1) { if (1==2 && …) {…} else {…} }` collapses to the `else` body.
- `ARS.cs:2071, 2083`, `DrawPath`/`DrawSection` `|| 1 == 1` and `if (1 == 1)` wrappers — collapse to unconditional.
- `SetThrottle`/`SetBrakes` `else if (1 == 1)` — just `else`.

### A.2 Dead enums — delete
- `Decision`, `Mistake`, `OptionValues`, `CameraTransition` (unanimous: zero references).
- `BrakeStabilityStrat` — only `.Invalid` ever appears; delete with its lone field (`Corner.stabilityStrat`).
- `eLookAheads.SteerInRef` — declared, never added to the dictionary.
- `RaceState.PostRace` — never assigned or tested (Qwen only; verify before deleting the enum member — it is a public API surface).

### A.3 Dead fields — delete (verify each before deletion, but all four audits agree on the bulk)
Racer.cs: `Traffic`, `MaxLeftLane`, `MaxRightLane`, `LocalSpdLimiter`, `SteerTarget` (write-only), `TorqueMult`, `DebugText`+`AddDebugText`, `HeadingPID` (+ its `SetValue` call in `Launch`), `_countersteerBlend` + `CountersteerFullSeverityMult` + `CountersteerRecoveryRate`, `Random` (the per-Racer one; callers use `ARS.GetRandomInt`), `DistToInside` (never called), `DistToOutside` (only called from the `1==2` block), `RiskFactorForGrip`/`RiskFactorForBrake` (stubs), the entire follow-lane-trail subsystem (`UpdateFollowLaneTrail`, `DrawFollowLaneTrail`, `DrawRawFollowLaneTrail`, `GetFollowLaneTrailPoint`, `GetRawFollowLaneTrailPoint`, `FollowLaneTrail`, `RawFollowLaneTrail`, `LastFollowLaneTrailNode` — note `TrailSamples`/`DrawInputTrails` ARE live; keep those).

DataStructures.cs: `VehicleControl.SteerStabilityCorrection`, `.SteerManeuver`, `.SteerMax`, `.FollowLane`, `.ThrottleOffset`, `.CurrentLockupLimiter`; `VehData.Understeer`, `.CurrentDownforce`, `.CurveRadiusPhysicalGs` (written, never read); `Rival.AvoidStr`, `.OvertakeLane`, `.sToRear` (computed, never read); `Corner.sToEntrance` (default 0, the only reader is gated by `1==1`); `Approach` class + `Corner.Approach` field (all three members unread); `CornerPoint.FullAngle` (never written); `AIData.SpeedToInput` (never called).

ARS.cs: `TracjectoryProjectionSeconds`; duplicate `AccelerationVector`/`lSpeed`/`GsClamp`/`directionClamp` instance fields (Racer has its own); `debugTrailer` + `HandlePlayerDebugStuff` (call site commented); `ImmersiveJoins`/`NeabyImmersiveJoins`/`HandleImmersiveJoins`/`secInteval` (call commented); `MiniaturizedPath`/`DrawMiniaturizedPath`; static terrain-hash lists `Other`/`Road`/`Dirt`/`Sand` (never enumerated — only `TerrainTypes` enum is used); `FlareFX` is used, keep; `ExhaustOffsets`, `DistToPercent`.

### A.4 Dead methods — delete
ARS.cs (no callers found by any audit): `GetMoreShit` (also vulgar), `GetWheelsPower`, `GetWheelInternalDownforceMod`, `GetWheelsWetgrip`, `GetWheelSkidmark`, `GetWheelSlippage`, `GetWheelsAvgWheelspin`, `GetTRCurveMax`, `GetModelFlags`, `SetDefMultiplier`, `SetGearRatio`, `GetDownforceGsAtSpeed`, `DrawStats` (both overloads), `DrawDirectionalBoundingBox`, `OffsetByAngle`, `LaneSelectionApproachCorner` (returns 0), `Lerp`, `LerpDelta`, `LerpByDistance`, `RotateDir`, `GetXfromPosInDirection`, `IsRoadBusy`, `GetRoadHeading`, `GetRoadOutOfBoundsX`, `GetRoadPos` (verify — Qwen says no live callers; MiniMax/DeepSeek did not flag), `IsAhead`, `HasArrived`, `IsBitSet`, `IsBetweenRange`, `Project`, `GetColorFromRedYellowGreenGradient`, `GradientAtoB` (only `GradientAtoBtoC` is used), `mapGamma`, `WarnPlayer`, `Notify` overload, `DrawBlackBars`, `DrawMiniaturizedPath`, `PlaceOnGround(Prop)` (empty body), `AdjustMenuOption` (empty stub called only as the null-`OnAdjust` fallback), `HandleImmersiveJoins`, `HandlePlayerDebugStuff`, `LoadVehicle(string,Vector3)`, `GetSteerInput` (verify), `Bezier3`, `GenerateCubicBezier` (only `Bezier2`/`QuadraticBezier` used).

PID class: if `HeadingPID` is the only consumer and is deleted, the entire `PID` class plus its methods becomes dead — but check whether any other PID instance exists. None was found; the class is a candidate for deletion alongside `HeadingPID`. **Disagreement:** DeepSeek lists PID methods as dead individually; Qwen flags the whole class as behaviorally dead; MiniMax lists `HeadingPID` dead but does not commit on the class. Recommendation: delete `HeadingPID`; if no other PID users, delete the class too.

### A.5 Copy-paste → one helper — unanimous, high-value
1. **Wheel-scalar readers** → `ReadWheelFloats(Vehicle, ulong offset, int round)` + a reduction variant for max/avg. Cuts ~9 near-identical functions to 2 helpers + thin wrappers for the live ones (`GetWheelsGrip`, `GetWheelsMaxWheelspin`).
2. **Native pattern-scan + offset cache** in `SetSteerInput`/`SetSteerAngle`/`SetThrottle`/`SetBrakes` → `ResolveOffset(pattern, mask, delta)` + `MemWriteFloat`/`MemReadFloat`. Same `\x74\x0A\xF3…` pattern, same learn-then-use shape.
3. **XML load-with-null-retry** in `GetTrackTags`/`GetTrackStartPos`/`GetRacerTags` → one `LoadXmlOrThrow(path)`. The `while (document == null && pat < 1000)` loop is unreachable (`Load` never returns null). Delete the loop everywhere.
4. **Hill-grip quartet** (`GetHillGsLossAt{TrackPoint,VelocityVector}`, `GetHillGripMultiplierAt{TrackPoint,VelocityVector}`) → parameterize on the angle source; one pair instead of four. Keep only the `…VelocityVector` variants (the `…TrackPoint` ones are unused per DeepSeek).
5. **`DrawPath` vs `DrawSection`** → one `DrawPathSegment(nodes, widedict, edgeStep, drawEndCaps)`; the two methods share their entire skeleton.
6. **XML route-point + prop write boilerplate** in `SaveRoute`/`UpdateRoute`/`LoadTrack` → `WriteRoutePoint(doc, pos, wide)` + `ParseRoutePoint(XmlElement)` + `WritePropElement(doc, Prop)`.
7. **`CleanEverything` double-clear** — `Path/WideDict/TrackLimits` cleared twice (ARS ~1077–1096); collapse to one pass.
8. **Track-corner chevron drawing** — AI branch and player branch of `DrawStuff` render the same three chevrons; extract `DrawCornerChevrons(CornerPoint)`.
9. **Handling-address readers** (`GetTRCurveLat`, `GetSteerLock`, `GetDownforce`, `GetHandlingFlags`) share a `T ReadHandlingField<T>(Vehicle, ulong offset)` shape (Qwen). Lower value than the wheel readers; optional.

### A.6 Other pure simplifications
- `Racer.UpdateFollowTrack` lookahead `ResolveLookAhead`: the seven `LookAheads.Add` calls can become a loop over `(enum, multiplier)` pairs. (The wraparound bug in the same local is a bug, not a simplification — see appendix.)
- `SpeedToThrottleBrake` symmetric clamp: `if (newBrake > 0.0) newThrottle = 0;` then `if (Math.Abs(newThrottle) > 0.0f) newBrake = 0;` → collapse to one "whichever is stronger wins" expression.
- `EnsureMenuItemDefinitions` for `Options.CreateTrack`: nested `if (routeEditMode) { if (routeEditMode) {…} }` tautology — drop the inner check.
- `Memory` constructor hardcodes three `Rivals.Add(new Rival())` — initializer or `Enumerable.Repeat` (no behavior change).

---

## B. Goal 2 — Rename inventory (per file)

Agreement is high across all four audits. Below is the merged list; where audits proposed different target names, the most descriptive is kept and alternatives noted.

### B.1 Typos (unanimous — fix)
| Current | Fix |
|---|---|
| `CornerPoint.LenghtEnd` | `LengthEnd` |
| `TracjectoryProjectionSeconds` | `TrajectoryProjectionSeconds` (or delete — dead) |
| `UpdatePercievedGrip` (and comment "percieved") | `UpdatePerceivedGrip` |
| `FreecCamRide` | `FreeCamRide` |
| `GametimerefLong` | `GameTimeRefLong` |
| `GametimeCountDown` | `GameTimeCountdown` |
| `StartCoundown` | `StartCountdown` |
| `MaxCountDown` | `MaxCountdown` |
| `CountDown` | `Countdown` |
| `secInteval` | `SecondInterval` |
| `NeabyImmersiveJoins` | `NearbyImmersiveJoins` |
| `menOffexts`/`memOffsets` local | `memoryOffsets` |
| `frecuency` (local + XML node) | `frequency` / `<Frequency>` — **caution: changes saved XML attribute name; back-compat consideration** |
| `OnAbort(object sourc, …)` | `OnAbort(object sender, …)` |
| `GetMoreShit` | delete |
| `SetloadingPromptText` | `SetLoadingPromptText` |
| `FinishedStandStill` | `FinishedStandstill` |

### B.2 Vague names — types/classes (DataStructures.cs)
| Current | Proposed | Note |
|---|---|---|
| `Memory` | `RacerBrain` | matches how it's referenced (`Brain`); all audits agree |
| `Memory.data` | `RacerBrain.Perception` or `State` | lowercase public field; "data" is vague |
| `Memory.intention` | `RacerBrain.Intention` | lowercase → PascalCase |
| `Intention.IntendedSpdChangeGs` | `IntendedSpeedChangeGs` or `TargetLongitudinalGs` | `Spd` inconsistent with sibling `Speed` |
| `VehData` | `VehicleState` | matches `Racer.VehicleData` field |
| `HandlingData` | `HandlingProfile` | optional |
| `VehicleControl` | `ControlInputs` | optional |
| `Rival.relativePos` | `Rival.RelativePosition` (or keep enum `RelativePos` → `RelativePosition`) | lowercase public field |
| `Rival.rPos` | `Rival.RelativeOffset` | cryptic |
| `Rival.sToReach` / `sToRear` | `SecondsToReach` / `SecondsToRear` | `s` ambiguous |
| `Rival.BoundingBoxTotal` | `Rival.CombinedSize` or `CombinedBounds` | "total" of what? |
| `Rival.DirectionDiff` | `Rival.VelocityAngleDiffDeg` | add unit |
| `Corner.OG` | `Corner.Point` (or `Definition`) | two-letter name |
| `Corner.sToEntrance` | `SecondsToEntrance` (or delete — dead) | |
| `CornerPoint.LengthStart`/`LenghtEnd` | `LengthStart`/`LengthEnd` | typo on End |
| `TrackPoint.TrackWide` | `TrackHalfWidth` | **semantic clarification:** all audits note it is used as a half-width in clamps (`roadWide - carHalfWidth`). DeepSeek/MiniMax propose `TrackWidth`; Qwen proposes `TrackWidth`. Confirm semantics during rename — if it is genuinely half-width, `TrackHalfWidth` is clearer. |
| `Approach` class | delete (all members unread) | |
| `BrakeStabilityStrat` enum | delete | |
| `AIData` | `AiConstants` (Qwen) | optional; only holds Max/MinSpeed |
| `HandlingData.TRlateral` | `LateralTractionCurve` or `TractionCurveLateral` | "TR" unreadable in callers |
| `HandlingData.Power` | `Acceleration` | it's `GET_VEHICLE_ACCELERATION` (DeepSeek) |

### B.3 Vague names — methods (Racer.cs)
| Current | Proposed |
|---|---|
| `SteerTrack()` | `ComputeSteering()` |
| `SpeedTrack()` | `ComputeTargetSpeed()` |
| `SpeedToThrottleBrake()` | `ApplySpeedToThrottleBrake()` or `ConvertSpeedToPedals()` |
| `SteerTranslateInput()` | `TranslateSteerToInput()` |
| `SteerApplyCorrections()` | `ApplySteerLimits()` (it only applies the speed-based limiter) |
| `DrawStuff()` | `DrawRacerDebug()` |
| `ClampTargetLaneForAvoidance()` | `ApplyRivalWalls()` (it maintains walls AND clamps) |
| `UpdateFollowTrack()` | `UpdateTrackPosition()` (it does lap counting + lookahead, not "follow lane") |
| `UpdateDynamicBoundingBox()` | keep, or `UpdateSlideAndBoundingBox()` |
| `UpdateCornerInfo()` | `UpdateCornerValidity()` |

### B.4 Vague names — methods/fields (ARS.cs)
| Current | Proposed |
|---|---|
| `map()` | `Remap()` (unanimous) |
| `mapGamma()` | `RemapGamma()` (or delete — dead) |
| `rad2deg()`/`deg2rad()` | `RadToDeg()`/`DegToRad()` |
| `GetWheelsMaxWheelspin` | `MaxWheelSlip` (descriptive) |
| `GetWheelsGrip` | `WheelGripMultipliers` |
| `GetDirectionalBoundingBox` | `SlidingBoundingBoxWidth` |
| `LeftOrRight` | `SignedLaneOffset` |
| `GetOffset` | `EntityRelativeOffset` |
| `MStoMPH`/`MPHtoMS` | `MpsToMph`/`MphToMps` |
| `GripGainLossElChange` | `HillGripDeltaGs` or `GripDeltaFromElevationChange` |
| `GetHillGripMultiplierAtCurrentVelocityVector` | `HillGripMultiplierFromVelocity` |
| `MapIdealSpeedForDistance` | `MaxSpeedForBrakingDistance` or keep |
| `GetSpeedForCorner` | `CornerApexSpeed` or keep |
| `LookForCornerAhead` | `FindNextCorner` |
| `GetCurveRadius`/`GetCurveRadius2D` | `Circumradius3D`/`Circumradius2D` |
| `Cheats()` | `HandleCheats()` |
| `CleanEverything` | `TearDownRace` (optional) |
| `SetSPLVisibility` | `SetSplVisibility` (acronym casing) or expand |
| `ParsedEnum` | `FormatEnumName` |
| `ReFilterKnownTracks` | `FilterKnownTracks` |
| `LoadDriver` | `LoadDriverXml` (clarify returns XmlDocument) |
| `TracjectoryProjectionSeconds` | delete (dead) |
| `catchupPos` | `CatchupPosition` |
| `intendedOpponents` | `IntendedOpponents` or `_intendedOpponents` |
| `routeEditMode` | `_routeEditorActive` |
| `listenmode` | `_listenMode` |
| `SteerOffset`/`steeroffset` | `SteerOffset` (PascalCase public static) |
| `throttleOffset`/`brakeOffset`/`handlingPtr`/`wheelsPtr`/`numwheelsoffset` | PascalCase uniformly |
| `static ulong steerAngle` | `SteerAngleOffset` (it's an offset, not an angle) |
| `scaleform` (field shadows type) | `InstructionalButtonsScaleform` or `_instructionalButtons` |
| `racertext`/`debugFrontend`/`floatingtext`/`SCCountdown` | PascalCase or delete (most are dead) |
| `Random rnd` | `_random` |
| `randomcolors` | `_randomColors` |
| `RacerBaseBehavior` | `RacerPhase` (Qwen: the values are phases, not behaviors) — optional, touches the enum name across files |
| `RCStatus` | `RacerRaceState` (collides visually with static `ARS.RaceStatus`) |
| `eLookAheads` | `LookAhead` (drop Hungarian `e` prefix) |

### B.5 Convention drift to fix in one pass
The older code uses PascalCase private instance fields (`HalfSecondTick`, `LastSpeed`, `HeadingPID`, `TorqueMult`); newer code uses `_camelCase` (`_cornerPhase`, `_avoidLeftWall`, `_isPassengerized`). Standardize on `_camelCase` for private instance fields, PascalCase for public. This is a large mechanical pass — do it per-file after the dead-code purge so renames don't fight dead code. Public static fields currently lowercase (`scaleform`, `steeroffset`) → PascalCase.

### B.6 Disagreements / judgment calls
- `TrackWide` semantics: half-width or full-width? DeepSeek/MiniMax/Qwen all flag it. **Resolve the semantics first**, then name it. My read: it is used as a half-width (`roadWide - carHalfWidth`), so `TrackHalfWidth`.
- `Corner.Speed` vs `CornerPoint.Speed`: `CornerPoint.Speed` is dead (never assigned meaningfully per Qwen); `Corner.Speed` is alive. Keep `Corner.Speed` (rename to `TargetSpeed` to distinguish — Qwen).
- `RacerBaseBehavior` → `RacerPhase`: Qwen proposes; others keep. Optional; touches the enum name across files.
- PID gains `kP/kD/kI` → `proportionalGain` etc.: Qwen proposes; others don't address. Lower priority; only matters if the PID class survives (it likely doesn't).
- `FinishedStandStill` → `FinishedStandstill`: trivial casing fix; Qwen only.

---

## C. Goal 3 — Remove all comments

All four audits agree: delete every comment. The rename pass (Goal 2) makes the code self-documenting. Categories present, with representative instances:

1. **Empty/stub XML `<summary>`** — `Racer.cs:543-545` (DistToOutside), `:662-663` (SteerTranslateInput), `:1221-1223` (OutOfTrackDistance); `DataStructures.cs:24-25`, `:28-31`, `:60-62` ("The AI brain."), `:180-182` ("Pure, precise data"), `:195-197` ("Context aware data").
2. **Content XML summaries that restate the method** — `Racer.cs:1332-1334` ("Processes AI logic on slow cycles"), `:1387-1389` ("Runs Speeding and Steering logic"), `:802-804` (TractionControl), `:849-851` (ProcessTick); `ARS.cs:4258-4261` and the hill-grip family.
3. **Stale "removed X" notes** — `ARS.cs:844` (OpenRaceOptionsMenu removed), `Racer.cs:744` (disabled limiter "Kept for reference"), `Racer.cs:1120-1122` (lane-offset trail TODO).
4. **Section banners** — pervasive in both files: `Racer.cs:23,32,37,43,53,61,67,78,96,100,104,108,113,118,122` and `:344` (`// ── Local helpers ──`). `ARS.cs:217,228,236,247,252,810,1246,1330,1343,1349,1352,1369,1405,1427,1442,1499,1515,1546,1615,4890`.
5. **Obvious restatements** — the entire PID block `ARS.cs:60-176` (`// Fields`, `// Constructor`, `// Set target value with clamping`, `// Get current value`, etc.).
6. **Disabled-feature notes** — die with the dead code in A.1.
7. **Inline math narration** — densest in `SteerTrack` (`// 1) Heading error…` through `// 8) Slide countersteer…`), `ComputeCornerTargetLane`, `ClampTargetLaneForAvoidance`, `SpeedTrack`, `GenerateRouteInfo` (`// 1) Compute a span…`, `// 2) Remove…`, `// 3) Build per-node lane bias…`), `GetCurveRadius2D` (perpendicular-bisector explanation).
8. **Terrain-hash inline notes** — `ARS.cs:236-241` (`//-1286696947<grass`, `//Concrete`, etc.) and `TerrainTypes` enum trailing comments. Die with the static hash lists if those are deleted.
9. **External-project breadcrumbs** — `Racer.cs:108` ("GodotRace-style"), `:259` ("Matches GodotRace"), `:325` ("GodotRace-style PD"). Remove.
10. **Catch justifications** — `Racer.cs:191` (`/* blip creation failed, non-critical */`). Remove; the empty catch is intentional.

### The one-exception question
Three audits say **zero comments kept**. Two (MiniMax, mine) name one candidate: the kinematic braking equation on `MapIdealSpeedForDistance` (`ARS.cs:4780-4782`), `v₀ = √(vTarget² + 2·decel·distance)`, as a non-obvious physics identity. MiniMax would keep it; DeepSeek and Qwen would fold it into the method name (`MaxSpeedForBrakingDistance`) and drop the comment. **Decision to make:** zero comments, or one. If one, the braking equation is the strongest candidate. Note: even that can be encoded in the method name, so "zero" is defensible.

---

## Appendix — Bugs and architecture (out of scope, noticed in passing)

These are NOT part of the three goals. Do not fix them during the refactor; track separately.

### Bugs (flagged by one or more audits)
1. **`PlaceCars` GridSort loop is a no-op.** `ARS.cs:2432`: `g.ToString().ToLowerInvariant().Contains(setting);` — the result is discarded, so `sort = g` runs every iteration and `sort` ends as the last enum member (`Random`). DeepSeek, MiniMax, Qwen all caught it. Real bug.
2. **`UpdateFollowTrack` lookahead wrap.** `Racer.cs:1266`: when `Node + offset >= Count`, returns `TrackPoints[offset]` instead of the wrapped node. Corrupts `LookAheads` on laps ≥ 2 of closed circuits. DeepSeek, MiniMax, Qwen caught it.
3. **`UpdateRivals` off-by-one.** `Racer.cs:1594`: `for (int i = 0; i < Brain.Rivals.Count - 1; i++)` never writes the last Rival slot. MiniMax flagged; verify whether intentional "free slot."
4. **`UpdateFollowTrack` loop on tiny tracks.** `Racer.cs:1244-1248`: `points.Count <= 12` with `i = 0` reset can loop forever if `TrackPoints.Count < 13`. Qwen flagged.
5. **`GetCurveRadius2D` divide-by-zero.** `ARS.cs:4217-4220`: vertical chord (equal X) → `±Infinity`/`NaN` radius. Callers NaN-guard, but `Infinity` clamps to `MaxSpeed`. Qwen flagged.
6. **`Rival.Update` `BoundingBoxTotal.Y` fudge.** `+2f` mixed with half-lengths; `RelativePos.Ahead` threshold may be too loose. Qwen flagged; verify the fudge is meters.
7. **`VehicleData.Gs` stray `/ 2`.** `Racer.cs:1574`: `AccelerationVector.Aggregate(...) / Count / 2` — unexplained halving. DeepSeek flagged.
8. **`GetWheelsGrip(Car).Average()`** throws on empty list (no count check). DeepSeek flagged.
9. **`SpeedTrack` `cornerSpd <= 5` recompute.** overrides the kinematic braking plan with grip-1.0 re-evaluation. DeepSeek flagged.
10. **`Bezier2` uses `Vector3.Zero` as control point** — geometrically a Bezier through world origin. Works in practice because callers add an offset, but misleading/brittle. DeepSeek flagged.
11. **Memory vs code drift on braking distance.** Project memory said "30m safety"; code uses 10m (we changed it this session). DeepSeek flagged the mismatch — this is expected, not a bug.
12. **`Intention.Direction` never assigned** — `Vector3.Zero` forever. Qwen flagged; likely dead field.

### Architecture observations (explicitly NOT a goal — parked for later)
- `AutosportRacingSystem.cs` is a 6280-line god class. Splitting into ~10 files by responsibility (TrackLoader, CornerBaking, RaceManager, MenuSystem, MemoryAccess, VehicleFactory, Drawing, MathUtils, Freecam, TrackCreator) is a separate, larger project. Do NOT mix into the three-goal refactor.
- `Racer.cs` debug drawing (`DrawStuff`, `DrawInputTrails`, trail helpers) is a candidate for a `RacerDebugDrawer` — separate task.
- `OnTick` is ~370 lines; after extracting leaderboard/countdown/freecam/traffic-density into named methods, it should be ~40 lines of orchestration. Separate task.
- Magic numbers → named `Tuning` constants (avoidance buffer, TCS base, braking safety, phase time thresholds, etc.). Adjacent to Goal 1 but not strictly simplification; consider a follow-up pass.
- `VehData.YawRotationPerSecondDegrees = 1f` and `SlideAngle = 22f` defaults leak fake values into the first tick before `UpdateGrip` overwrites — should be `0f`. Borderline bug; flagged by my audit.

---

## Suggested execution order (preserves playability at every step)

1. Dead-code purge (A.1–A.4). Compile. Commit.
2. Comment removal (C). Compile. Commit.
3. Rename DataStructures.cs (B.2). Compile. Commit.
4. Rename Racer.cs (B.3 + private-field convention). Compile. Commit.
5. Rename ARS.cs (B.4 + static-field convention). Compile. Commit.
6. Copy-paste collapse (A.5). Compile. Commit.
7. Other simplifications (A.6). Compile. Commit.

Each step compiles independently. Stop anytime, mod stays playable. Bug fixes and file-splitting are separate tracks.