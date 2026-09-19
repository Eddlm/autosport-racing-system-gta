# Vanilla GTA V steering — what the reference source actually does

**Read this when** a question or a change touches *how the game itself* steers a car: "how does vanilla do it", GTA's AI steering, `CCarAI` / `FindMaxSteerAngle` / `CVehicleIntelligence`, `CVehControls` / `m_steerAngle` / `m_fSteerInput` / `m_fSteerInputBias`, `HumaniseCarControlInput`, `SET_VEHICLE_STEER_BIAS`, driving-style flags, stick curves, auto-centre, steer-to-throttle coupling, or **any claim that "the real GTA V code" does X**. It also owns the correction several ARS notes need: the `/= 1 + 0.075 × (fwdSpeed − 5)` speed reduction is **player-assist only** — it is *not* the AI's steering model.

**Trees — read-only reference, always cited as `file:line` + symbol.** Line numbers drift between copies, so re-grep the symbol rather than trusting a line here:
- **engine** (all implementations live here): `E:\GTA\GTAVSP\GTAV Source\src\dev_ng\game` — `Vehicles`, `vehicleAi`, `task`, and `script\` (`commands_task.cpp`, `commands_vehicle.cpp`)
- **script declarations only**: `G:\P1\P1\gta5\script\dev_ng` is the RAGE *script* project — it carries the `.sch` native declarations (`core\common\native\commands_vehicle.sch`, `commands_task.sch`) and **no implementations**. Do not go looking for `commands_vehicle.cpp` there.

**NEVER take a native hash from either tree — they are outdated, and a wrong hash fails silently.** Proof: the source's `SCR_REGISTER_SECURE` hash for `SET_VEHICLE_CHEAT_POWER_INCREASE` is `0x8f7d5ed5832ac0aa`, while the hash that actually works in ARS is `0xB59E4BD37AE292DB` — same native, different numbers. Use a hash already proven in `src\`, a SHVDN enum name, or verify in game. The **behaviour, symbol names and `file:line` structure** are what this tree is good for; the hashes are not — treat every hash below as source-declared only.

## One control struct, two writers

`CVehControls` (`Vehicles\vehicle.h:97–155`) = `m_steerAngle`, `m_secondSteerAngle`, `m_throttle`, `m_brake`, `m_handBrake`, `m_nitrous`, `m_KERS`. Wheels consume it in `Automobile.cpp` (`float fApplySteerAngle = GetSteerAngle();`, ~`:3488`/`:3692`). Player and AI are only two *writers* of that struct; everything that differs between them is upstream.

**Units: radians, not degrees.** `Vehicles\handlingMgr.cpp:1751` converts on load — `m_fSteeringLock = DtoR * m_fSteeringLock` — so the runtime handling struct, `CVehControls::m_steerAngle` and every AI steering constant are radians (a meta lock of 40° becomes ~0.7 rad). Corroborating: `FindMaxSteerAngle` returns 0.2–0.7, `GetTurnRadiusAtCurrentSpeed` computes `fWheelBase / sin(maxSteerAngle)`, and the automobile task asserts its angles inside ±HALF_PI. **ARS is unaffected** — it converts back at read time (`Racer.cs:317` lock, `:308` lateral traction, both `RadToDeg`), so its degrees stay self-consistent; treat any vanilla number copied into ARS as radians until converted.

## The AI/NPC path — a plain bearing law

`vehicleAi\task\TaskVehicleGoToAutomobile.cpp:10466+` (`GoToPoint_OnDeferredTask`) is the steering law, and it is smaller than ARS's:

- `vehDriveOrientation = atan2(forward)` — the **body** heading, *not* the velocity heading
- `dirToTargetOrientation = atan2(targetPos − carPos)`
- `desiredSteerAngle = SubtractAngleShorterFast(dirToTarget, driveOri)` — a **relative bearing to the target point**, sign-flipped under `DF_DriveInReverse`

So vanilla's AI is a heading-error law with **no cross-track term and no distance/lookahead scaling at the steering level.** The comment claims it aims ahead a little; the code aims at `GetTargetPosition()`. The lookahead lives in whoever produced the target (route / junction / pathfind), never in the steering law.

**It is not entirely slip-blind, but the slip term is chase-only** (`:10525–10544`): when the driver is in a vehicle chase or carries `CPED_RESET_FLAG_SteerIntoSkids` (and is not handbrake-turning), vanilla adds `fSideSlip × sfSideSlipSteerInfluence` to the desired angle — `fSideSlip` is lateral velocity normalised by `|forward speed| + 1`, and the tunable's default is **negative (−0.4)**, i.e. steer *against* the slide. Two details ARS should note: it is added **before** the clamp, so vanilla's countersteer is **not exempt** from the steering cap (unlike ARS's `ApplySteerLimits`), and its gain is an order of magnitude smaller than ARS's `2 × SlideAngle` blend. Normal cruising gets no slip term at all.

`AdjustControls` (`:10630–10856`) then, in order: clamp to `±FindMaxSteerAngle()`; a throttle **ceiling** from steering usage; `gasDownMult`, a second speed-gated throttle cut; a slip-based traction ceiling (`CalculateMaximumThrottleBasedOnTraction`); apply, flipping the steer sign when momentum opposes the desired direction. Two things ARS should take from it:

- **Vanilla couples steering to throttle through a `Min`-style ceiling that is re-applied every tick** (~30% gas at full steering angle above 10 m/s, 50% below). That is precisely the shape `AGENTS-BACKLOG.md` concluded a revived ARS tie-in must use — ARS's removed version was a *decaying decrement*, which the recovery erased.
- **The AI has no countersteer exemption** — the clamp is symmetric, and the slip term above is inside it. ARS's exemption is a deliberate ARS deviation, not something vanilla does.

## The AI's speed-based steering cap

`CVehicleIntelligence::FindMaxSteerAngle` (`vehicleAi\VehicleIntelligence.cpp:2663–2677`), applied at `TaskVehicleGoToAutomobile.cpp:10655–10656`:

`min(0.7, speed > 42 ? 0.2 : 0.9 − speed/60)` rad ⇒ **40° up to ~12 m/s, 34° at 18, 23° at 30, 11.5° above 42 m/s.**

It is **speed-only** — no driving flag, no grip, no TRlat, no downforce, no personality input (the comment only muses that bigger vehicles may want a larger value).

**Caveat — a second, older limiter exists but is not in this drop.** The comment that points here says "carai.cpp"; `CCarAI` appears in this tree only as call sites (`Automobile.cpp:5173,5181`, `Bike.cpp:716`, `train.cpp:6262`) and as a **link-order symbol list** (`VS_Project\LinkOrder\BankRelease_LinkOrder.txt`, e.g. `?FindMaxSteerAngle@CCarAI@@SAMPAVCVehicle@@@Z` and `?ClipSteerAngleToMaxSteerAngle@CCarAI@@...`) — its implementation file is absent. That symbol takes a `CVehicle*` and so is **not** the no-argument `CVehicleIntelligence::FindMaxSteerAngle()` traced above. The cap we can prove is applied in the task path is the `CVehicleIntelligence` one; whether a legacy `CCarAI` path still runs is **unverifiable from this tree** — don't assert it either way.

## The player path

`vehicleAi\task\TaskVehiclePlayer.cpp`:

- pad axis → `fDesiredSteerInput`, shaped by a **stick curve** `Sign(x)·|x|^1.5` (`ms_fCAR_STEERING_CURVE_POW`, `:71`); mouse-steering has its own deadzone/multiplier/auto-centre branch (`:696–722`), while a real wheel device bypasses the curve entirely because it is already time-based (`:742–749`)
- a **speed-blended first-order lag** into `m_fSteerInput`: `current += (desired − current) × fSmoothFrac × dt`, 12/s stopped → 6/s at ≥30 m/s on one platform and 10/5 on the other (`:56–60`, `:70`), **×0.2 for rear-wheel-steer cars** (`:629–633`)
- `m_fSteerInput += m_fSteerInputBias`, then clamped to ±1 (`:799–800`)
- `fSteerAngle = m_fSteerInput × m_fSteeringLock` (`:801`), plus a stationary auto-centre capped at 10° (`:72`, `:854`)

## Player-only assists — the correction the ARS notes need

`Vehicles\Automobile.cpp:3692–3746` sits **inside `if (bDriverIsPlayer)`**, and the comment above it says the AI is handled elsewhere ("carai.cpp") *so that you can see what AI cars are trying to do*. Inside that block:

- the speed reduction `fApplySteerAngle /= 1 + 0.075 × (fwdSpeed − 5)`, gated on `!(sideSpeed × steer < −0.1 × |fwdSpeed|)` (`:3715–3718`; constants at `:2962–2964`) — a countersteer carve-out, but **only for the player's car**
- a **velocity-referenced auto-centre**, `atan2(−sideSpeed, fwdSpeed)` clamped to ±15° (`:2966`, `:3720–3725`) — vanilla's yaw/slip correction for the player

**So: never cite the 0.075 formula as "what GTA V's AI does".** The AI's authority limit is `FindMaxSteerAngle` above. Distinct from both: `CVehicle::m_fSteeringBias` (`Automobile.h:406`) is a life-decayed PIT/side-hit bias applied and clamped to ±`m_fSteeringLock` (`Automobile.cpp:3491`, `:3750`) — **not** the script bias below.

**ARS borrows this curve anyway, deliberately.** `ApplySteerLimits` uses it as the AI's live steer *ceiling* — `SteeringLock / (1 + 0.075 × (v − 5))` — not as the player's attenuation, and takes only the reduction (vanilla's auto-centre is not applied). The attenuation form was driven too and felt stable, but it removes gain at every speed; the ceiling form leaves the AI's full gain below the limit and clips only the extremes, and the driver judged that decisive rather than twitchy. Two properties carried over and one didn't: the countersteer exemption survives (by the yaw test rather than vanilla's side-speed one), the curve's knee at 41 mph is where the steering allowance halves — but the ceiling is **grip-blind**, which is the one thing the TRlat limiter it replaced had and this does not (`AGENTS.md` pipeline step 4).

## Rate limiting is a traffic behaviour, not a racing one

`HumaniseCarControlInput` (`vehicleAi\task\TaskVehicleMissionBase.cpp:251–329`; automobile variant `TaskVehicleGoToAutomobile.cpp:10864–10921`) **passes the AI's decided controls straight through when it is neither conservative-driving nor going slowly** — normal driving has *no* steer rate limit at all. The smoothed branch clamps the change to ~2.0 rad/s (0.5 when stopped), and `bConservativeDriving` comes from `GetIntelligence()->GetHumaniseControls()`, set while cruising. ARS's fixed 180°/s slew on the angle is ARS's own construct; the nearest vanilla analogue is the player-side exponential lag on the *input*, not a constant-rate slew on the angle.

## The script layer — and the traps in it

Natives are registered in the **engine** tree (`game\script\commands_task.cpp`, `commands_vehicle.cpp`; tables at `SCR_REGISTER_SECURE`, `commands_task.cpp:9033+` / `commands_vehicle.cpp:15488+`) and reach steering through one chain:

native → `sVehicleMissionParams` (`TaskVehicleMissionBase.h:28`; flags field `:213`) on a `CTaskVehicleMissionBase`, wrapped by `CTaskControlVehicle` → per-tick `CTaskVehicleGoToPointAutomobile` → `CVehControls` → `CVehicle::SetSteerAngle` → `CWheel::SetSteerAngle`.

- **`SET_VEHICLE_STEER_BIAS` is PLAYER-ONLY** (source-declared hash `0x77451b49db5a200a`, **do not use** — see the hash warning). It writes `CVehicle::m_fSteerInputBias` (`commands_vehicle.cpp:7695`), and the **only steering consumer is the player drive task** (`TaskVehiclePlayer.cpp:799`) — no AI goto/mission task reads it (the rest are bike lean `:2304`, plane yaw `:3497`, lowrider `:887`; zeroed in `Vehicle.cpp:1265,25230`, `TaskDamageDeath.cpp:5632`). Anything that would use it to steer an *AI* car will do nothing.
- **`TASK_VEHICLE_DRIVE_TO_COORD` never reads `DRIVINGSTYLE_RACING`** (`commands_task.cpp:443–478`) — it only ORs `DF_DriveInReverse` when the mode is 2. The script `DRIVINGSTYLE` enum is also degenerate: `NORMAL`/`ACCURATE`/`STRAIGHTLINE` are all `0` and only `RACING=1`, `REVERSING=2` differ (`commands_vehicle.sch:315–321`).
- **`SET_DRIVE_TASK_DRIVING_STYLE` takes the `DF_` bitfield, not the 0/1/2 style** (source-declared hash `0x27be5555cdf6f983`, **do not use**) — it ends at `SetAllFlags(flags)` (`TaskVehicleMissionBase.h:293`).
- **`SET_DRIVE_TASK_MAX_CRUISE_SPEED` truncates to a byte and clamps nothing** (`commands_task.cpp:6541`); `SET_DRIVE_TASK_CRUISE_SPEED` clamps to `MAX_CRUISE_SPEED` = 120.0f (`TaskVehicleMissionBase.cpp:33` — its header comment still says 63; the code wins).
- **These do not exist in either tree**: `SET_DRIVING_STYLE`, `SET_DRIVE_TASK_HANDLING_FLAGS`, `SET_DRIVE_TASK_ACTUAL_VEHICLE_SPEED`. The only `SET_DRIVE_TASK_*` natives are CRUISE_SPEED, MAX_CRUISE_SPEED and DRIVING_STYLE.
- **`TASK_VEHICLE_MISSION` / `_PED_TARGET` / `_COORS_TARGET`** build flags from booleans in `CVehicleIntelligence::GetTaskFromMissionIdentifier` (`VehicleIntelligence.cpp:3305+`), where `MISSION_GOTO` and `MISSION_GOTO_RACING` both land on `GetGotoTaskForVehicle` — so the "racing" mission variant is not a distinct driving law.
- **`CDriverPersonality` reaches throttle, not steering** — its only public entry is `FindMaxAcceleratorInput` (`driverpersonality.h:24`), used at `TaskVehicleMissionBase.cpp:319` and `TaskVehicleGoToAutomobile.cpp:10903`.

**Steering-relevant driving flags** (`enum DrivingFlags`, `vehicleAi\VehMission.h:93`, `BIT(n)`): `DF_SwerveAroundAllCars` BIT(2), `DF_SteerAroundStationaryCars` BIT(3), `DF_SteerAroundPeds` BIT(4), `DF_SteerAroundObjects` BIT(5), `DF_DontSteerAroundPlayerPed` BIT(6), `DF_GoOffRoadWhenAvoiding` BIT(8) (avoidance may leave the road), `DF_DriveIntoOncomingTraffic` BIT(9), `DF_DriveInReverse` BIT(10) (negates the steering angle and flips the bonnet/drive reference), `DF_ForceStraightLine` BIT(24), `DF_AdjustCruiseSpeedBasedOnRoadSpeed` BIT(14), `DF_UseShortCutLinks` BIT(18) (this single bit *is* `DRIVINGMODE_PLOUGHTHROUGH`), `DF_ChangeLanesAroundObstructions` BIT(19), `DF_AvoidTurns` BIT(27), `DF_ForceJoinInRoadDirection` BIT(30). Composites with intent comments: `VehMission.h:143–150`; **"ignore road speed" is the *absence* of BIT(14) — there is no positive ignore bit.**

## ARS vs vanilla, at a glance

| | vanilla AI | ARS |
|---|---|---|
| steer error | bearing to a point, no distance | lateral distance in metres |
| reference | body heading | velocity (course-over-ground) |
| authority limit | speed-only, 11.5–40° | grip/TRlat-scaled, ~6–9° at speed |
| countersteer | inside the same symmetric clamp | exempt from the clamp |
| slip term | chase-only, gain −0.4 on normalised sideslip | always, `2 × SlideAngle` |
| rate limit | none in normal driving | fixed 180°/s, doubled countersteering |
| throttle tie-in | two `Min` ceilings (steer usage, slip) | none (removed as a no-op) |
| script steering lever | `m_fSteerInputBias`, player-only | ARS writes its own input |

**Take-aways.** ARS's steering allowance is 3–4× tighter at speed than vanilla's AI — deliberate, a different law, and driver-verified, but it means vanilla is not the authority for that number. Vanilla's steer→throttle coupling is the reference implementation for the tie-in ARS removed. Vanilla keeps its AI nearly slip-blind (chase-only term) and buys stability from the speed cap plus the throttle ceilings; ARS's split — slip handling inside the AI's steering law, throttle tie-in removed — is the inverse, and it was a choice, not an accident.

## Not in this drop (do not guess)

`carai.cpp` / `CCarAI` implementations; the racing-line layer (`racingline.cpp`, held by `CTaskVehicleGoToPointAutomobile` per `TaskVehicleGoToAutomobile.h:68–70`); internals of `CTaskVehicleGotoLongRange`; and the non-deferred / LOD / super-dummy steering paths of the goto task.
