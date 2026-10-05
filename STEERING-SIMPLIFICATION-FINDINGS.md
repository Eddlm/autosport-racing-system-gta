# ARS — steering simplification: findings and ranked candidates

**Provenance.** Read against `48c8d6f`, working tree clean. `src/Racer.cs` is the revision left by `a0134d9` (3755
lines); every `file:line` anchor below was opened in that file rather than recalled, and where a doc's anchor has
drifted the checked line is given beside it. **No build was run and nothing was driven** — every statement here is a
static reading of the source. Nothing in this file is a claim that a behaviour works; only the driver can say that.

**Scope.** The steering chain as `PLAN-STEERING-SIMPLIFICATION.md` names it: `ComputeSteering`, `ApplySteerLimits` and
its helpers, `TranslateSteerToInput`, plus the settings and debug surfaces that reach them. The speed pipeline, the
corner lifecycle and the recovery path are touched only where they read steering state, and no candidate proposes
changing them.

**What the chain is attached to.** `ProcessAI` runs it for AI cars only (`Racer.cs:3356`), and `ProcessAI` is called
from `RunTimedCore` (`Racer.cs:2524`), which `AutosportRacingSystem.cs:1922-1938` paces at six racers per frame. So
the "tick" below is that racer's timed-core tick, and `TickScale` (`Racer.cs:1531`) is the time since that racer's
own last core tick, not a frame.

---

## 1. The live chain, stage by stage

### 1.1 Where it sits, and the order the contract states

The contract (`PLAN-STEERING-SIMPLIFICATION.md:16`) fixes the order as **track position → target speed → steering →
steer limits → pedals → steer slew**. What the code does inside one core tick (`Racer.cs:3343-3393`):

| # | stage | anchor | input | consumer |
|---|---|---|---|---|
| 1 | track position | `UpdateTrackPosition` `Racer.cs:2711` | lookaheads, route data | lookahead table the steering reads |
| 2 | target speed | `ComputeTargetSpeed` `Racer.cs:3365` call, body `:1556` | apex plan, route speed | `Brain.CurrentIntention.Speed`; **reads `Control.SteerDegrees` at `:1626`** |
| 3 | steering | `ComputeSteering` `:3366` call, body `:494` | lane systems + pursuer + damper + blend | `Control.SteerDegrees` |
| 4 | throttle/brake reason caps | `:3368-3369` | pedal reasons | `MaxThrottle*` / `MaxBrake*`; **reads `Control.SteerDegrees` at `:1780`** |
| 5 | pedals | `ConvertSpeedToPedals` `:3370`, body `:1302` | intended speed | `Control.Throttle`/`Brake` |
| 6 | stuck recovery | `:3372-3375` | stuck state | throttle/brake overrides; no steering write |
| 7 | steer limits | `ApplySteerLimits` `:3378`, body `:1161` | `Control.SteerDegrees` | clamps it in place |
| 8 | steer slew | `TranslateSteerToInput` `:3379`, body `:1536` | `Control.SteerDegrees`, `TickScale` | `Control.SteerInput` → `ApplyInputs` `:2587` |

The limiter runs **after** the pedals, not before them — see **Q1**, because the two stages between them read
`Control.SteerDegrees` and therefore see the previous core tick's post-slew steer rather than this tick's clamped
request.

### 1.2 `ComputeSteering` (`Racer.cs:494`) — resolution, aim, command

| stage | anchor | input | output / consumer |
|---|---|---|---|
| priority reset | `:496` | — | `_slidePriority = 0` before the gate, so a failed gate leaves the blend and the allowance off |
| context gate | `:498`, body `:630-640` | `BaseBehavior`, `CurrentTrackPoint.Node >= 3`, `LookAheads[SteerRef]` | returns `SteerDegrees = 0` (`:500`) |
| travel direction | `:509-510` | `Car.Velocity` | `courseDir` — velocity when `LengthSquared() > 0.01`, forward otherwise |
| high-speed lane | `:517` → `:791-811` | `roadWide`, speed, node at `speed × 1.01` m, its `PreciseCurveRadius < 500` (`:748`, `:802`) | `defaultLane`; **0 on a straight** |
| corner lane | `:520` → `:835`, `IdealLineOffset` `:826` | `Brain.Corner`, `Lap > 0`, `steerRefPoint` | overrides `defaultLane` when non-zero (`:521`) |
| avoidance lane | `:523` → `:886` | `Brain.AvoidanceTarget` and the rest of `Brain.Rivals` | overrides when non-zero (`:524`) |
| rival walls | `:526` → `:952-1011` | `_avoidLeftWall`/`_avoidRightWall`, `roadWide` | clamps the lane to the walls |
| debug lane override | `:527` | `Options.LockLaneCentre` (default false, `AutosportRacingSystem.cs:194`) | replaces the whole lane with 0.1 m |
| aim assembly | `:533-542` | `targetLane`, `GsAwarePreview` toggle (`:535`), `ProjectAhead`, `SignedLaneOffset` | `_debugLaneAimPoint`; clamped to the drivable edge at `:541` |
| corner log | `:544` → `:703-745` | corner table, current node | log lines only |
| **lane pursuit** | `:548` → `:771-789` → `:765-769` | `_debugLaneAimPoint`, `courseDir`, `WheelBase`, `PursuitGain` (`:755`) | `laneSteerDeg`, `_steerAimBearingDegrees`, `_steerAimCurvature` |
| rival repulsion | `:554-572` | rivals within the long/lat gates, their relative lateral velocity | **added** into `laneSteerDeg` (`:571`) |
| side-by-side | `:577` → `:645-677` | overlapping rivals, `courseDir`, `SideBySide*` (`:302-304`) | `sideBySideSteerDeg` |
| forward speed | `:585` | `ARS.GetForwardSpeed` (`AutosportRacingSystem.cs:3365`) | damper and blend gating |
| blend weight | `:589-592` | `Handling.LateralTractionCurve`, `SlideAngle`, `SlideBlendStartFraction`/`FullFraction` (`:1018-1019`), `fwdSpeed >= 2` | `_slidePriority` → blend `:617`, limiter `:1193`, `IsFullCountersteer` `:1238` |
| damper reference | `:593-595` | `SteerDampingAimReference`, `fwdSpeed`, `SlideAngle` vs `LateralTractionCurve × SlidingFraction` (`:1015`), `_steerAimCurvature` | `yawTarget` |
| damper gain | `:596` → `:1040-1045` | `SteerDampingGain`, `SteerDampingReferenceMps` (`:1039`), `fwdSpeed`, `BaseMechanicalGrip`, `SteerDampingEnabled` | `damperGain` |
| damper term + 2 subtraction rules | `:601-609` | `YawRotationPerSecondDegrees`, `yawTarget`, `nonDamperSteerDeg`, `DamperCrossingShare` (`:1028`) | `damperTermDeg`, `_damperTermDeg` (limiter bypass `:1217`, debug `:2508`) |
| assembly | `:600`, `:610-611` | side-by-side, damper, lane | `Control.SteerDegrees` |
| slide blend | `:613-618` | `_slidePriority`, `sideBySideSteerDeg`, `SlideAngle`, `CountersteerShare()` (`:1228-1233`) | lerps the whole command |
| aligned-wheel deadband | `:623` | `_steerAimBearingDegrees < SmallAimBearingDegrees` (`:760`), no slide | zeroes the command |
| NaN guard | `:625-626` | — | zeroes the command |

Two fields carry the lane stage's own state out: `_steerAimBearingDegrees` and `_steerAimCurvature` are written by
`PursuitSteerDegrees` (`:786-787`) and read at `:594` and `:623`. On its two early returns (`:775-779`, `:781-785`)
it clears the bearing but **not** the curvature, so `yawTarget` in that tick is built from the previous tick's
curvature. It is currently inert: the same early return leaves `|bearing| < 1°`, so `:623` zeroes the command, and the
limiter's bypass needs `requestedSteer > 0f` / `< 0f` (`:1219-1220`) which a zero command cannot satisfy. Recorded
here so the next reader does not have to re-derive why it is harmless.

### 1.3 `ApplySteerLimits` (`Racer.cs:1161`)

| stage | anchor | input | effect |
|---|---|---|---|
| NaN guard | `:1164-1168` | — | command zeroed, returns |
| reverse / standstill | `:1171`, `:1178-1179` | `Dot(Velocity, ForwardVector)` | not > 0 → `speedCeiling = SteeringLock` |
| cornering ceiling | `:1181` → `:1133-1144` | `TRLateralAtSpeed` (`:1088`), `PeakSlipCeilingAt` (`:1109-1112`), `GeometrySteerCeiling` fallback (`:1103-1107`), `ManeuverRamp` (`:1148-1153`) | base ceiling |
| slide addition | `:1184` | `SlideAngle`, `SlideLimitSlideShare`/`FreeplayDegrees` (`:1158-1159`) | **raise**, Slide-Governed only |
| both sides set | `:1186-1187` | — | `SteerLimitLeft`/`SteerLimitRight` |
| countersteer allowance | `:1191-1196` | `countersteering` (`:1176`), `_slidePriority`, `SlideAngle` | **raise** to `|slide| × weight` on the answering side |
| yaw envelope | `:1199-1211` | `YawUsagePercent` (`:1117-1128`), `YawTurnInMinimumPercent`/`Maximum` | **cut** of the commanded side, then `ManeuverRamp` (`:1208`) |
| damper bypass | `:1217-1221` | `countersteering`, `_damperTermDeg`, yaw rate | **raise** to full lock |
| one clamp | `:1223` | both side limits | `Control.SteerDegrees` |

### 1.4 `TranslateSteerToInput` (`Racer.cs:1536`)

NaN guard `:1539`; slew of `SteerDegrees − LastAppliedSteerDegrees` by `SteerSlewRate × TickScale` (`:1541-1545`,
rate at `:1026`, both directions one rate); degrees → `[-1,1]` remap against `VehicleData.SteeringLock` (`:1549`).
Consumed by `ApplyInputs` (`:2587`). `Control.LastAppliedSteerDegrees` is reset on race init (`:1294`).

### 1.5 Per-core inputs the chain reads

`VehicleData.YawRotationPerSecondDegrees` is a raw native read written in `UpdatePerceivedGrip` (`:3696`), which
`RunTimedCore` calls at `:2521` — once per core tick, before `ProcessAI`. `VehicleData.SlideAngle` and
`VehicleData.BoundingBox` are written by `UpdateSlideAndBoundingBox` (`:2520`, body `:2531-2535`), and
`TRLateralAtSpeed` by `UpdateTRLateralAtSpeed` (`:2522`, body `:1090-1094`). All three are therefore fresh for the
chain within its own tick, and stale for every consumer outside it — `IsUnstable` (`:1812`) and `YawUsagePercent` are
in the same core-tick world; nothing reads them per frame.

### 1.6 Consumers of steering state outside the chain

| consumer | anchor | reads |
|---|---|---|
| steer-limited speed | `ComputeTargetSpeed` `:1626-1631` | `Control.SteerDegrees` — the previous tick's **post-slew** value |
| overspeed arm gate | `UpdateThrottleReasonCaps` `:1780` | `Control.SteerDegrees` vs `OverspeedArmMaxSteerDegrees` (`:1761`) |
| nitrous gate | `:2024` | `Control.SteerDegrees` vs `NitrousMaxSteerDegrees` (`:279`) |
| debug lane lines | `AutosportRacingSystem.cs:1760` | `TargetLane` property (`Racer.cs:141`) |
| debug steer fan | `Racer.cs:2274-2280` | `_steerPursuitDeg`, `_damperTermDeg`, `Control.SteerDegrees` |
| debug yaw panel | `Racer.cs:2502-2515` | `_debugYawTargetPerSecond`, `_debugDamperGainSeconds`, `_debugDamperSpeedScale`, `_damperTermDeg`, `YawUsagePercent()` |
| pedal bar reason | `Racer.cs:2454`, `:1323`, `:1850` | `IsFullCountersteer()` → `_slidePriority` |

---

## 2. Interaction map — raise, cut, or both

The Lead's map (`PLAN-STEERING-SIMPLIFICATION.md:72-74`) counts the limiter's four raisers and one cutter. Adding the
command half gives the full picture:

**Raise-only**
- rival repulsion `:571` (adds a signed magnitude away from a closing rival);
- slide addition `:1184` (Slide-Governed only);
- countersteer allowance `:1194-1195`;
- damper bypass `:1219-1220`;
- maneuvering ramp `:1152` (declared raise-only at `:1142` and `:1147`).

**Cut-only**
- yaw envelope `:1209-1210` — the only cut that shapes authority to a non-zero value (the deadband below cuts to
  exactly zero, and the clamp and the slew only bound);
- aligned-wheel deadband `:623` (to exactly zero, not to a smaller value);
- the final clamp `:1223` (bounds, does not shape);
- the slew `:1543` (bounds the rate, either direction).

**Both**
- the damper term `:601-608`: subtract-only in the command's sign (`:603`), but past neutral it keeps half its
  authority `:608`, so its net effect can sit on the far side of the command — the Lead's map already has this;
- the slide blend `:617-618`: a lerp toward `sideBySide − slide × share`, so it can raise or cut;
- the lane pursuit `:548`: signed, and the only term that moves with position rather than rotation;
- side-by-side `:674`: signed, summed with the pursuit.

**Two shapes worth naming.**

1. **The ramp is applied twice in Yaw-Governed and once in Slide-Governed.** `ResolveSteerCeiling` ramps its own
   ceiling (`:1143`), and Yaw-Governed then ramps the share-scaled value again (`:1208`). Slide-Governed's addition
   (`:1184`) is unramped but is `max`'d against the ramped resolution, so the ramp still governs. This is why the two
   modes differ *inside* the 5–30 mph band rather than meeting on the ramp's value — see **Q3**.
2. **Two different slide-derived ramps feed the countersteer.** The command's share ramps 0.5 → 1.0 across
   0.5× → 1.0× the **static** authored peak (`CountersteerShare` `:1232`, constants `:1019-1021`), while the
   limiter's allowance is `|slide| × _slidePriority`, whose weight is already full at 0.5× the peak
   (`:590`, `:1193`). At every slide the allowance is a superset of the command's own countersteer component; the two
   laws are not one law — see **Q8**.

---

## 3. Dials and constants

Values are not recorded here (the code owns them); what is recorded is whether each has a live consumer, a menu
reader, or only a debug draw.

| dial / constant | anchor | consumer class |
|---|---|---|
| `SteerDampingGain` | `AutosportRacingSystem.cs:154`, menu `:1059-1066`, schema `SettingsRepair.cs:151` | **menu**, live at `Racer.cs:1044` |
| `SteerLimitMode` | `AutosportRacingSystem.cs:160`, menu `:1069-1078` | **menu**, live at `Racer.cs:1177` — but not declared in the schema, see **Q2** |
| `YawTurnInMinimumPercent` / `Maximum` | `AutosportRacingSystem.cs:155-156`, menu `:1080-1100`, schema `:152-153` | **menu**, live at `Racer.cs:1203-1204` |
| `DebugToggles[GsAwarePreview]` | `AutosportRacingSystem.cs:195` (default true), menu `:890` | **debug menu**, live at `Racer.cs:535-538` |
| `DebugToggles[LockLaneCentre]` | `AutosportRacingSystem.cs:194` (default false), menu `:889` | **debug menu**, live at `Racer.cs:527` |
| `SteerDampingEnabled` | `Racer.cs:1031`, rationale `:1029-1030` | compile-time switch, fixed true; **A/B handle per `AGENTS-STEERING.md:52`** |
| `SteerDampingAimReference` | `Racer.cs:1034`, rationale `:1032-1033` | compile-time switch, fixed true; the code comment keeps the zero reference "one flip away for A/B", and `AGENTS-STEERING.md:218` treats the reference as the lever if the standing-offset toll returns |
| `SteerSlewRate` | `Racer.cs:1026` | live at `:1542`; driver-verified at 180 (`TEST-PLAN.md:242-245`) |
| `SteerDampingReferenceMps` | `Racer.cs:1039` | live at `:1043` |
| `DamperCrossingShare` | `Racer.cs:1028` | live at `:608`; rule driver-verified (`TEST-PLAN.md:277-285`) |
| `SlidingFraction` | `Racer.cs:1015` | live at `:594` (damper reference gate) |
| `SlideBlendStartFraction` / `SlideBlendFullFraction` | `Racer.cs:1018-1019` | live at `:590` and (the full one) `:1232` |
| `CountersteerSlideShare` | `Racer.cs:1020` | live at `:1232` |
| `CountersteerFullSlideFraction` | `Racer.cs:1021` | live at `:1232` **as a factor of exactly 1** |
| `SlideLimitSlideShare` / `SlideLimitFreeplayDegrees` | `Racer.cs:1158-1159` | live at `:1184` |
| `PeakSlipOuterWheelCommandShare` | `Racer.cs:1054` | live at `:1111` |
| `SteerLimitRampStartMph` / `EndMph` | `Racer.cs:1057-1058` | live at `:1137-1152` |
| `VanillaSteerReductionPerMps`, `SteerCapGripReference`, `SteerCapGripFloor` | `Racer.cs:1050-1053` | reachable only through `GeometrySteerCeiling` — see **C6** |
| `PursuitGain`, `MaxPursuitBearingDegrees`, `SmallAimBearingDegrees` | `Racer.cs:755-760` | live at `:767`, `:623` |
| `SteerPreviewSeconds`, `SteerLookaheadMinMeters`, `MaxMeters`, `GsPreviewBlend` | `Racer.cs:129-133` | live at `:537`, `:2775` |
| `HighSpeedLaneRadiusMeters`, `IdealLineLeadMultiple`, `RequirementLookaheadSeconds` | `Racer.cs:748`, `:819`, `:814` | live in the lane systems |
| `SideBySide*` | `Racer.cs:302-304` | live at `:657-670` |
| `LaneLockTestOffsetMeters` | `Racer.cs:751` | debug toggle only (`:527`) |
| `CornerLogApexBandNodes` | `Racer.cs:681` | diagnostic only — see **C1** |

---

## 4. Dead, write-only and unreachable (zero-behaviour-risk inventory)

| item | anchor | why it is safe |
|---|---|---|
| `_rawCornerLane` | decl `:143`, write `:522` | **write-only** — no reader anywhere in the tree (repo-wide check) |
| `SteerLimitLeft` / `SteerLimitRight` | decl `:168-169` | public fields whose only readers are inside `ApplySteerLimits` (`:1187`, `:1194`, `:1209`, `:1219`, `:1223`); nothing outside draws or reads them |
| `CountersteerFullSlideFraction` | decl `:1021`, use `:1232` | multiplies `peak` by exactly 1f — cannot change the expression |
| `SteerDampingEnabled` / `SteerDampingAimReference` | `:1031`, `:1034` | `const … = true`, so the guarded branches are fixed; behaviour-identical to inline the kept path |
| corner diagnostic | call `:544`, body `:693-745`, `ForwardNodes` `:685-691` | log-only; its own comment says to remove it with its call (`:679-680`); `ForwardNodes` has no other caller (checked) |
| `GeometrySteerCeiling` block | `:1064-1073`, `:1077-1084`, `:1103-1107`, constants `:1050-1053`, callers `:1135`, `:1141` | taken only when `TRLateralAtSpeed <= 0.01f` / `endPeak <= 0.01f`; `LateralTractionCurve` is clamped to 22 outside 1..100 at `:426-427` and defaults to 22 (`DataStructures.cs:105`), so both conditions need a NaN velocity length (`:1092-1093`) |
| `_steerPursuitDeg`, `_debugYawTargetPerSecond`, `_debugDamperGainSeconds`, `_debugDamperSpeedScale` | `:178-180`, `:172-174` | written for the debug draws at `:2274-2280` and `:2502-2515` only; dead only if those drawings go |
| `CurveRadiusAfterFollowPoint` | `DataStructures.cs:130` | adjacent, not in this chain: marked `NEVER USED` in place and read nowhere |

---

## 5. The two steer-limit governors, compared as a design

**What they share.** Both run the same stages: `ResolveSteerCeiling` as the base (`:1181`), the maneuver ramp inside
it (`:1143`), the countersteer allowance (`:1191-1196`), the damper bypass (`:1217-1221`), and the single clamp
(`:1223`). Reverse and standstill keep the raw lock in both (`:1178-1179`). The mode switch therefore does **not**
choose between two limiters — it changes exactly two things:

1. `:1184` — Slide-Governed raises the ceiling to `max(cornering law, half the slide + half a degree)`;
2. `:1199-1211` — Yaw-Governed cuts the commanded side to the yaw-usage share and ramps part of the way back.

**Measured against the repo's own model.** `docs/steer-limit-modes.py` mirrors `:1133-1153` and `:1199-1211`; I
checked its formulas line by line against those spans and evaluated them (no file written) for the fleet-typical car
it assumes — 40° lock, 22° authored peak slip, Turn-In 20%/100%, no slide, no yaw, no damper:

| forward speed | Yaw-Governed, zero yaw usage | cornering law = Slide-Governed, no slide | Yaw-Governed at full yaw usage |
|---|---|---|---|
| 5 mph | 40.0° | 40.0° | 40.0° |
| 10 mph | 33.4° | 34.5° | 34.5° |
| 15 mph | 26.3° | 29.0° | 29.0° |
| 20 mph | 18.8° | 23.5° | 23.5° |
| 25 mph | 10.9° | 18.0° | 18.0° |
| 30 mph | 2.5° | 12.5° | 12.5° |
| 40 mph | 2.1° | 10.5° | 10.5° |
| 60 mph | 1.6° | 8.0° | 8.0° |

**Is one redundant?** As code, no: they act on different quantities, and the yaw envelope is the chain's **only**
steer-in cutter (the Lead's map says the same). What the A/B actually tests is narrower and sharper than
"two governors": **should a car's own yaw usage gate how much steer-in it is allowed at all?** Everything else —
the countersteer allowance, the bypass, the ramp, the cornering ceiling — is common to both.

**Is the switch earning its place?** The evidence that bears on it: the cut is computed from
`|YawRotationPerSecondDegrees|` (`:1127`), i.e. from a state the steering itself produces, so a car not yet
rotating is granted the least authority to start (2.5° at 30 mph, 2.1° at 40); the value climbs only as the car
already rotates. `AGENTS-STEERING.md:92` records the field's warning about closing a loop through a signal the loop
produced, which is the shape this has. That is a design question, not a finding — **the A/B is undriven**
(`TEST-PLAN.md:287`), and nothing here can say which shape drives better. What can be said is that the two modes are
not redundant and the switch selects one design decision, not a tuning.

---

## 6. Ranked candidates

Ranked by behaviour risk first, then by how much is removed. Every candidate names what it removes, what it is
expected to cost behaviourally, and what the driver would watch. No candidate is a re-tune, and none proposes moving
a stage.

### C1 — Remove the temporary corner diagnostic from `ComputeSteering` — zero risk, ~62 lines

**Removes:** the call at `:544`; `LogCornerCrossings` `:703-745`; `LogCornerPhase` `:693-701`; `ForwardNodes`
`:685-691` (its only callers are `:716-718`); `CornerLogApexBandNodes` `:681`; `_logCornerIndex` / `_logCornerPhase`
`:682-683`; and the comment at `:679-680`, which is the removal instruction itself.
**Behavioural cost:** none. It only reads `CurrentTrackPoint.Node` and the corner table, and writes log lines at
phase transitions. The per-tick work removed is a loop over `ARS.Corners` that breaks at the containing corner.
**How the driver would know:** no on-track symptom — the observable is negative: `[CORNER] …` lines stop appearing in
`Log.log`. That is the read-out the corner-miss diagnosis uses (`:679-680`), so if any planned drive still reads
corners off the log, this lands after it.

### C2 — Delete `_rawCornerLane` — zero risk, 2 lines

**Removes:** the field `:143` and its assignment `:522`. The value it mirrors is still used at `:521`.
**Behavioural cost:** none — write-only.
**How the driver would know:** no symptom; nothing displays or reads it.

### C3 — Make the two steer-limit fields locals — zero risk

**Removes:** `SteerLimitLeft` / `SteerLimitRight` `:168-169` (with their `= 40f` initialisers), turning the clamp at
`:1223` into one built from locals; and with them the first half of the comment at `:166-167` — "Public so a rule can
grant an allowance to one side **and the debug view can draw them**" — because no debug view reads them.
**Behavioural cost:** none. Every reader is inside `ApplySteerLimits`.
**How the driver would know:** no symptom. The only observable would be a compile error if some consumer I could not
find exists — which is itself the check.

### C4 — Delete the factor-of-one dial — zero risk

**Removes:** `CountersteerFullSlideFraction` `:1021` and its use at `:1232`, where it multiplies `peak` by exactly 1f.
**Behavioural cost:** none; the remap's high input bound is unchanged by an exact ×1.
**How the driver would know:** no symptom.

### C5 — Retire the two fixed-true switches — zero behaviour, but a ruling is needed

**Removes:** `SteerDampingEnabled` `:1031` with the ternary at `:1044`, and `SteerDampingAimReference` `:1034` with
the guard at `:594`; both are `const … = true`, so inlining the kept path is behaviour-identical.
**Behavioural cost:** none by construction.
**How the driver would know:** no symptom.
**Ruling needed — Q6:** `AGENTS-STEERING.md:52` names the kill switch as "the handle for re-running the experiment";
the reference switch's own comment keeps the zero reference "one flip away for A/B" (`Racer.cs:1032-1033`), and
`AGENTS-STEERING.md:218` treats the reference as the lever if the standing-offset toll returns. Removing both
retires those handles, which contradicts the docs' intent even though it changes no behaviour.

### C6 — Remove the unreachable geometry ceiling fallback

**Removes:** `SteerReductionPerMps` `:1064-1073`, `AckermannCeilingDegrees` `:1077-1084`, `GeometrySteerCeiling`
`:1103-1107`, `VanillaSteerReductionPerMps` / `SteerCapGripReference` / `SteerCapGripFloor` `:1050-1053`, the two
fallback ternaries `:1135` and `:1141`, and the comments that tie them to `SteerLimitedSpeed` (`:1046-1049`,
`:1060-1063`, `:1075-1076`). Roughly 30 lines and 3 constants.
**Reachability, checked:** the fallback fires only when `TRLateralAtSpeed <= 0.01f` (`:1135`) or `endPeak <= 0.01f`
(`:1141`). `TRLateralAtSpeed` is `LateralPeakAtSpeed(Velocity.Length())` (`:1090-1094`, `:1097-1101`), which is zero
only if `Handling.LateralTractionCurve <= 0.01f` — impossible, because `Initialize` clamps it to 22 when outside
1..100 (`:426-427`) and the field defaults to 22 (`DataStructures.cs:105`) — or if the velocity length is NaN
(`:1093`). So no finite state reaches it.
**Behavioural cost:** none for every finite state. In the NaN-velocity state the fallback currently yields a
geometry ceiling; without it `PeakSlipCeilingAt(0)` returns 0 (`:1109-1112`) and the ceiling collapses to zero,
leaving only the ramp, the allowance and the bypass to raise it. So the removal is free only if that state is
declared impossible or replaced by an explicit guard — **Q7**.
**How the driver would know:** nothing in normal running, because the path is not taken. If the degenerate state
ever occurred without a guard, the symptom is a car that briefly stops steering (full authority loss for a tick or
more) — a glitch, not a handling trait.

### C7 — Collapse the third forward-speed expression onto `ARS.GetForwardSpeed`

**Removes:** the inline `Vector3.Dot(Car.Velocity, Car.ForwardVector)` at `:1171`, replaced by the call the command
side already uses at `:585` (`AutosportRacingSystem.cs:3365-3373`).
**Behavioural cost:** not zero. The helpers differ on a pitched car: `GetForwardSpeed` flattens the forward vector to
XY and normalises it (and returns 0 when the vehicle is unusable or the flattening is degenerate), while the inline
dot keeps the pitch component. On a climb the limiter's speed would read slightly higher, moving the ceiling and the
ramp position, and the `fwdSpeed > 0f` reverse gate (`:1179`) would be decided on the flattened vector.
**How the driver would know:** a small shift in allowed steer-in on steep climbs and dips — a fraction of a degree at
speed. The more visible case is a car reversing on a slope, where the sign of the gate could differ between the two
expressions.

### C8 — Collapse the `nonLaneSteerDeg` intermediate

**Removes:** the local at `:610`; `:611` becomes `nonDamperSteerDeg + damperTermDeg`, since the two lines already
compute that sum.
**Behavioural cost:** the floating-point addition order changes from `(side + damper) + lane` to
`(side + lane) + damper` — a last-bit difference on angles of order a few degrees, orders of magnitude below the
slew's per-tick step.
**How the driver would know:** no symptom.

### C9 — Drop the Yaw-Governed turn-in cut, leaving one governor — **a deliberate behaviour change**

**Removes:** the yaw envelope block `:1199-1211`; `YawUsagePercent` `:1117-1128` (its only other consumer is the
debug panel at `:2509`); the two menu dials `Turn-In Minimum` / `Turn-In Maximum` (`AutosportRacingSystem.cs:1080-1100`)
and their schema entries (`SettingsRepair.cs:152-153`); and the `SteerLimitMode` switch itself (`Racer.cs:1177`,
`AutosportRacingSystem.cs:160`, menu `:1069-1078`) — because with the cut gone the switch would only be choosing
between the slide addition and the slide addition, so it selects nothing, and the addition at `:1184` becomes
unconditional. The countersteer allowance, the bypass and the ramp are untouched.
**Expected behavioural cost:** above 30 mph the steer-in ceiling rises from the cut value to the cornering law —
2.5° → 12.5° at 30 mph, 2.1° → 10.5° at 40, 1.6° → 8.0° at 60 — and inside the band it rises to the ramp's value
(18.8° → 23.5° at 20 mph). Below 5 mph nothing changes (both are the lock). In other words the limiter stops being a
steer-in governor and becomes a cornering ceiling with its raisers. This is a behaviour that is currently
undriven either way (`TEST-PLAN.md:287`), and it is the one candidate here that changes what the wheel can ask for.
**How the driver would know:** the same corners taken with visibly more steer-in at speed — a sharper turn-in, and
the symptoms that would follow it: entry over-rotation, more slide-blend activity at high speed, or a weave on fast
sweeps. The specific check is a fast corner where the car currently ploughs with the wheel at its limit; with this
change the wheel would be allowed past that limit. The `YawUsagePercent` debug read-out on the yaw panel
(`:2513`) is the instrument for watching the usage that currently gates it.

---

## 7. Decisions for the Lead

Nothing below is settled here; each is named with its evidence.

**Q1 — the pipeline order the contract fixes is not the order the code runs.** The contract
(`PLAN-STEERING-SIMPLIFICATION.md:16`) says "steer limits → pedals"; the code runs pedals (`:3370`) before the
limiter (`:3378`), with the throttle/brake reason caps between them (`:3368-3369`). That is not cosmetic: the
overspeed arm gate reads `Control.SteerDegrees` at `:1780`, and `SteerLimitedSpeed` reads it at `:1626`; in the
code's order both see the previous core tick's post-slew steer, while the contract's order would have them see this
tick's clamped request. The doc's own pipeline list (`AGENTS-STEERING.md:230-236`) is in the contract's order too.
Decision: fix the contract to match the code, or move the limiter above the pedal stage (a real behaviour change to
those two gates), or state which readers were intended.

**Q2 — `SteerLimitMode` cannot survive a reload, so the A/B procedure in `TEST-PLAN.md` cannot hold as written.**
The menu writes the key (`AutosportRacingSystem.cs:1074` → `SaveRacerSetting` `:1256-1258` → `SettingsMenuStore`,
which is `Menu-Settings.ini`, `:3074`). `SettingsRepair.PruneOwnedFiles` (`SettingsRepair.cs:195-232`) drops every
`[MENU]` key of an owned file that is not declared in `BuildSchema` (`:214`, `:218`, `:224`), and `BuildSchema`
declares `SteerDampingGain` (`:151`) and the two turn-in dials (`:152-153`) but **no `SteerLimitMode`**. `LoadSettings`
runs the prune (`:3071`) before reading the value with a `"Yaw-Governed"` default (`:3103`), and the menu rebuilds its
selection from the same store (`:1076`). So within one session the switch holds; across a script reload it is pruned
and both the code and the menu return to Yaw-Governed. `TEST-PLAN.md:328-329` tells the tester to "confirm the mode
you think you are driving is the one the menu shows" — that check passes, but the mode it shows will be the default.
Decision: declare the key (a fix, outside this pass's write scope) or amend the test note.

**Q3 — `TEST-PLAN.md`'s "inside the band the modes do not differ" is false.** `TEST-PLAN.md:337-338`: "from 5 to
30 mph the limit is the ramp's value whichever governor is set". The ramp is applied to a *different input* per mode
(`:1143` on the ceiling, then `:1208` on the Yaw share-scaled value), so the outputs differ throughout the band —
33.4° vs 34.5° at 10 mph, 18.8° vs 23.5° at 20, 10.9° vs 18.0° at 25 — and coincide only at ≤5 mph, where both are
the lock. Decision: amend the note, or confirm that the two modes were intended to meet on the ramp.

**Q4 — the docs disagree with each other about straight-line lane control, and the code contradicts one side.**
With no lane demand, `targetLane` is 0 and the aim follows it (`:534`); the aim point is
`steerRefPoint.Position + steerRight × aimLane` (`:542`) and the pursuit runs unconditionally once the context gate
passes (`:548`; no `|targetLane|` gate on the command). So with no lane demand the aim is the **route centre
at the lookahead**, not the car's own offset, and a car offset laterally on a straight has a non-zero bearing and
gets a non-zero pursuit command. `AGENTS-STEERING.md:210` ("a car displaced laterally on a straight is therefore
corrected by nothing") and `:222` (ranked item 5's premise that a straight-line offset is uncontrolled) say the
opposite; `:255` says the gate was removed, which matches the code; and `:256` says "with no lane demand the aim is
the car's own live offset" — no such line exists in the code, and the Gs-aware preview shifts the aim by half the
*projected minus current* offset (`:537-538`), which is a drift-lead term, zero at steady offset, not a hold.
Decision: which statement is the record, and whether ranked item 5's premise needs correcting.

**Q5 — doc anchors and claims that no longer match the code (hygiene batch).**
- `AGENTS-STEERING.md:230-236`'s pipeline anchors: `UpdateTrackPosition` is at `:2711` (doc 2295),
  `ComputeTargetSpeed` at `:1556` (doc 1315), `ComputeSteering` at `:494` (doc 416), `ConvertSpeedToPedals` at
  `:1302` (doc 1072), `TranslateSteerToInput` at `:1536` (doc 1537).
- `AGENTS-STEERING.md:283` cites `AutosportRacingSystem.cs:1888` for the timed-core pacing; that line is
  `_countdown--`. The batch is `AutosportRacingSystem.cs:1922-1938`.
- `AGENTS-STEERING.md:126` says "ARS already has `_steerLimitedThisFrame` for the saturation gate" — no such symbol
  exists in the tree.
- `AGENTS-STEERING.md:292` says the Show Inputs fan draws each limit on its true side; nothing outside
  `ApplySteerLimits` reads either limit field (C3).
- `AGENTS.md`'s durable-gotcha line "`if (1 == 2) return;` is an INVERTED gate that disables nothing" — no
  `1 == 2` (spaced or not) exists in any `.cs` file.
- Two menu descriptions contradict their consumers: "Steer Damping" says "against zero yaw"
  (`AutosportRacingSystem.cs:1059`) while the live damper is aim-referenced (`Racer.cs:594`), and "Steer Limit Mode"
  says Slide-Governed "sets the limit to the slide angle plus a degree" (`:1070`) while `:1184` takes the **max** of
  the cornering law and **half** the slide **plus half a degree** (`TEST-PLAN.md:299-300` records the halving;
  `AGENTS-STEERING.md:234` records the addition).

**Q6 — retiring the two fixed-true switches (C5) retires the documented A/B handles.** Decision: inline the kept
path, or keep the switches as the record says they are kept.

**Q7 — the degenerate state behind C6.** If `Car.Velocity.Length()` is ever NaN, `TRLateralAtSpeed` becomes 0
(`:1092-1093`) and the geometry fallback is what keeps a ceiling on the car. Removing the block without a guard turns
that state into zero steer-in authority. Decision: declare the state impossible, or replace the block with a named
guard before removing it.

**Q8 — should the limiter's countersteer allowance and the blend's countersteer share be one law?** They are two
slide-derived ramps on different scales (`:1193` with `:590`; `:1232` with `:1230`), and the allowance is a superset
of the command's own countersteer component at every slide. Not proposed as a candidate because the magnitude of the
change is hard to state and the allowance is the limiter's guarantee that the correction is never clipped
(`TEST-PLAN.md:324-325` wants exactly that). Decision: leave two laws, or unify and re-drive.

---

## 8. What could not be settled from here

- **No drive, no log, no build.** Every symptom above is what to watch, not what happens. Whether the Yaw-Governed
  cut helps or hurts on track (Q on the governor's design), and whether C9's extra steer-in is an improvement or an
  over-rotation source, are questions only a drive can answer.
- **The NaN-velocity reachability in C6/Q7 is static reasoning**, not an observation; I did not find a path that
  produces it, and I cannot rule one out from the source alone.
- **The governor curve table is arithmetic**, computed from the repo's own model in `docs/steer-limit-modes.py`
  after checking its formulas against `Racer.cs:1133-1153` and `:1199-1211`. It assumes the fleet-typical car the
  script documents; a car whose authored peak slip or lock differs scales differently.
- **The A/B cannot be trusted across a reload** until Q2 is decided, so a drive that spans a script reload may be
  judging a mode the menu is showing but the code is not running.
