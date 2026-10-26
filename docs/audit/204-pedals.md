# AUDIT 204 — the pedal chain: eight caps, one law, and the number the brief got wrong

**Surface**: the AI throttle and brake from the speed error to the control float — `ConvertSpeedToPedals`
(`Racer.cs:2264-2338`), `ComputeCombinedInput` (`:2341-2345`), `SplitCombinedInput` (`:2347-2356`), the
front-slip cap on the combined input (`:2281-2285`), the eight throttle caps (`UpdateThrottleReasonCaps`
`:2913-2964`, `ComposeThrottleCap` `:3037-3071`), the three brake caps (`UpdateBrakeReasonCaps`
`:3001-3019`, `ComposeBrakeCap` `:3075-3091`), the shared glide (`:2883-2893`), the projection clock
(`:2303-2328`), `TickScale` (`:109-110`, `:2582`) and the two writers that reach the control page
(`:2333-2334`, `:5152-5163`) through `VehicleMemory.SetThrottle`/`SetBrakes` (`:3933-3934`).

**Date**: 2026-10-25.

**Build audited**: **1092**. `build/build-number.txt` reads `1092`; `bin/Release/ARS.dll` is sha256
`8e3ab22dc3b3ff8a1dda4dfd3cd8a7f5c8017f36206032ffa3210dea6c983285`; source `HEAD e851a09`.

**Method**: read-only. Source read, `git log` read, the engine's own side-slip assignment re-read in the
leaked tree (`E:/GTA/GTAVSP/GTAV Source/src/dev_ng/game/Vehicles/wheel.cpp:5303-5309`), and the constants
put through `python -c`. No build, deploy, commit or game run.

**Status vocabulary**: **VERIFIED** = read in the code or reproduced read-only (arithmetic, source read).
**PROBABLE** = one link rests on a behaviour not observed today. Nothing here is driver-verified.

**Verdict**: **no proven blocker on this surface.** The eleven caps compose as a plain `Math.Min` and every
level is inside `[0,1]`, so no cap can manufacture a pedal that no cap asked for; a zero can only come
from a cap that states zero as its intent. What is wrong is elsewhere: one cap is never rewritten after
the flag (**204-02**), the stability cap has no floor and its only release is a measurement of motion
(**204-01**), the glides and the projection clock scale with an unguarded `TickScale` that reaches 9.999 s
while the pedal slew alone is protected (**204-03**), and the brief's own 1:6 is a **1:3** in this build
(**204-09**).

---

## The chain, in the order it runs

1. `SpeedVectorLocal.Y` against `Brain.CurrentIntention.Speed` → one error in m/s; the divisor is 1 m/s
   under the target and 3 m/s over it (`:2266-2275`, `:2343`).
2. `ComputeCombinedInput` clamps that to `[-1,1]`; the front-slip cap then lowers the positive side only
   (`:2277-2285`).
3. `SplitCombinedInput` turns the sign into one pedal (`:2347-2356`); full countersteer clamps both to 0.5
   (`:2289-2295`).
4. `ComposeThrottleCap` takes the minimum of the plan's throttle and the eight reason fields
   (`:3044-3051`); `ComposeBrakeCap` the minimum of the split brake and ABS, countersteer and route
   (`:3077-3078`).
5. The projection outranks both, on its own clock (`:2303-2328`).
6. Both pedals then slew at 6/s, capped at 0.5 per tick (`:2261-2267`, `:2333-2334`), and are written into
   the vehicle's control page (`:3933-3934`).

The whole chain is **AI-only**: `ProcessAI`'s player branch calls `UpdateNitrous()` and nothing else
(`:4811-4818`), and `ApplyInputs` is guarded by `if (!ControlledByPlayer)` (`:3611-3614`). Every number
below is a rival's.

---

## The eleven caps, and where each can land

All levels are inside `[0,1]` and compose by `Math.Min` (`:3044-3051`, `:3077-3078`), so the question
"can one cap leave no pedal" is the question "which cap can ask for zero, and what holds it there".

| Cap | Field | Level it asks for | Can reach 0 | What holds it there |
|---|---|---|---|---|
| TCS | `MaxThrottleFromTCS` | 1 down to `TcsCapFloor` 0.2 | no | wheelspin past the slip target (`:2975-2984`, floor `:2841`); setting off returns 1 (`:2977`) |
| Stability | `MaxThrottleFromStability` | 1 down to 0, additive | **yes** | any wheel whose side slip reads exactly 0 (`:2918-2923`) — see 204-01 |
| Overspeed | `MaxThrottleFromOverspeed` | 1 down to 0 | yes | armed only straight, slide under 2 deg, above 30 mph (`:2898-2911`) |
| Rival | `MaxThrottleFromRival` | 1 down to `RivalLiftFloor` 0.1 | no | a rival ahead inside 3 m (`:2939-2943`, `:475-476`) |
| ChillOut | `MaxThrottleFromChillOut` | 0.5 | no | the card is parked at `const ChillOutEnabled = false` (`:3202`), so the field reads 1 (`:2947`, `:473`) |
| Yield | `MaxThrottleFromYield` | 0.5 | no | the card with a live target (`:2948`, `:2852`) |
| Off-track | `MaxThrottleFromOffTrack` | 1 at 15 mph to 0 at 20 mph; 0.75 while recovering | yes | off the edge above 20 mph (`:2950-2959`, `:2853-2858`) |
| Recovery | `MaxThrottleFromRecovery` | 1 at 20 mph to 0 at 25 mph, only in `Drive` | yes | the stuck phase (`:2961-2963`) |
| ABS | `MaxBrakeFromABS` | 1 when `AbsEnabled` is false | only with ABS on | the slip taper and the lock backstop (`:3021-3032`); off by load |
| Countersteer | `MaxBrakeFromCountersteer` | 0.5 | no | full countersteer (`:3005`, `:1823`) |
| Route | `MaxBrakeFromRoute` | 0.2 | no | route is the limiting term and no corner plans a brake (`:2729`, `:256`) — and it freezes at the flag, 204-02 |

---

## Findings

No blockers. Release-notes first, in the order they should be read; then the files. Every row carries its source.

| # | Sev | Claim | Evidence | Status | How to reproduce |
|---|---|---|---|---|---|
| 204-01 | release-note | **A car that comes to a dead stop loses its forward throttle within half a second, because `MaxThrottleFromStability` has no floor and the only branch that raises it needs the car to be moving.** `WheelsOffGround` calls a wheel airborne when its side-slip field is exactly `0f` (`AutosportRacingSystem.cs:3304-3313`), and the engine assigns exactly `0.0f` whenever `Abs(fSpeedAlongFwd) <= 0.1f` and the lateral spring is unloaded (`wheel.cpp:5303-5309`, the assignment at `:5303` and the gate at `:5304`). Four such wheels drain the cap at 2.0/s to zero in 0.5 s, and it rises at 0.5/s only in the `else` branch where no wheel reads zero (`Racer.cs:2918-2923`); the composed throttle is then 0 (`:3044`). The recovery state machine starts a *stopped* car in `Reverse` (`:4928`), and that phase writes the pedal directly (`:5161`), so the escape from the missing pedal is backing up, not the throttle the plan is asking for. | Engine: `wheel.cpp:5303-5309`. Code: `AutosportRacingSystem.cs:3304-3313`; `Racer.cs:2918-2923`, `:3044`, `:4928`, `:5161`. | VERIFIED (engine + code); PROBABLE in game (a stop on camber may carry a lateral-spring term, which makes the angle non-zero) | Stop an AI car dead mid-race (wall, spin) with Show Inputs on: `MaxThrottleFromStability` walks to 0 in ~0.5 s and the throttle stays 0 while the plan asks 1, until the recovery's reverse phase moves the car. One log line of `ARS.WheelsOffGround` plus the caps on a car held at 0 mph for 2 s settles it. |
| 204-02 | release-note | **`MaxBrakeFromRoute` is written only while a race is being planned, so the finish freezes whatever it was — a 0.2 brake ceiling that outlives the flag.** It has exactly two writers: the `Launch()` reset (`Racer.cs:2215`) and `ComputeTargetSpeed` (`:2729`). That method returns early for `GridWait`, `FinishedRace` and `FinishedStandStill` (`:2609-2623`), all before `:2729`, so a car that crosses the line on a stretch where the route term is the limiting one and no corner plans a brake keeps `RouteBrakeCap` **0.2** (`:256`) for the rest of the session. `ComposeBrakeCap` then caps the pedal there and names `BrakeReason.Route` (`:3078`, `:3083`). On a point-to-point the finished target is 0 m/s (`:2621`): the stop takes five times the full-pedal time — roughly 25 s at 60 m/s on a 1.2 g car, against the 30 s race timeout (`AutosportRacingSystem.cs:2275`). | `Racer.cs:2215` and `:2729` are the only writes (grep `MaxBrakeFromRoute`); early returns `:2609-2623`; cap `:256`; composition `:3077-3078`, `:3083`; point-to-point target `:2621`. | VERIFIED (mechanism); PROBABLE (that the last pre-finish tick had 0.2) | Finish on a straight and log `Control.MaxBrakeFromRoute`, `Control.BrakeReason` and `Control.Brake` after the flag: cap 0.20, reason Route, pedal pinned near 0.2 while the target is 0. |
| 204-03 | release-note | **`TickScale` cannot be 0 but can be 9.999 s, and only the pedal slew is guarded against it; the glides and the projection clock scale linearly and saturate in one tick.** `TimeSince_lastCoreTick` clamps to `[1, 9999]` ms (`:110`), so `TickScale` lives in `[0.001, 9.999]` (`:2582`) — a zero would freeze both pedals at their last value (`inputChange = 0`, `:2267`), and the floor is what prevents that, not a separate guard. The slew is protected twice, by `PedalSlewMaxPerTick = 0.5` (`:2262`), which binds above 83 ms per core tick. Nothing else is: `Glide` steps `3.5 x TickScale` (`:2885-2887`), so a 286 ms stall already crosses a cap's whole range in one tick and 9.999 s gives 35; the projection clock adds `TickScale` per tick (`:2304`) and jumps past both the 0.5 s zero and the 0.75 s half-brake floor (`:2305`, `ProjectionInputPerSecond = 2` `:259`, `ProjectionBrakeFloor = 0.5` `:260`), so the first tick back from an alt-tab or a load hitch applies the full floor with no ramp; stability takes `0.5 x wheels x TickScale` (`:2920`), i.e. 20 at 9.999 s, clamped to 0 (`:2923`). A failed core tick does not update `_lastCoreTick` either, because the write is the last line of `RunTimedCore` (`:3866`), so repeated failures inflate the interval until one tick completes. | `Racer.cs:109-110`, `:2582`, `:2261-2267`, `:2883-2893`, `:2304-2305`, `:259-260`, `:2920-2923`, `:3857-3867`. | VERIFIED (arithmetic from the constants) | Alt-tab or force a ~1 s hitch while the projection is outside and widening: on the first tick back `sec` reads about 1 and `input` sits at -0.5, and any reason cap reaches its target in that same tick. |
| 204-04 | release-note | **With `TcsEnabled = False` only the wheelspin taper goes; seven caps still act on the throttle, and the one that behaves like traction control is not gated on the setting.** `TcsCapLevel` returns 1 on its first line (`:2977`), so `MaxThrottleFromTCS` sits at 1 — the card that could pin it is disabled outright (`KickCardEnabled = false`, `:5064`; `UpdateThrottleKick` returns on its first line, `:5068`). Still live and able to cut: stability (`:2918`), overspeed (`:2928`), rival lift (`:2939-2945`), ChillOut (`:2947`), Yield (`:2948`), off-track (`:2950-2959`), recovery (`:2961-2963`), the projection (`:2303-2327`) — and the **front-slip cap on the combined input** (`:2281-2285`), which reaches zero throttle at 2.5x the front peak and never reads `TcsEnabled`. The brake side has no ABS at all: `AbsEnabled` is false at load, so `AbsCapLevel` returns 1 (`:3023`). | `Racer.cs:2975-2984`, `:2281-2285`, `:2918-2963`, `:3021-3024`, `:5064-5068`; the band's origin `docs/steering-and-pedals.md:23-29`. | VERIFIED | Run `TcsEnabled = False`, put an AI car into a front-led understeer past 250 % (readout `F`), and watch the throttle close anyway. The setting removes the wheelspin taper, not the front-slip cut. |
| 204-05 | file | **The throttle reason names a cap that is not the binding one whenever the projection is in play.** `projectionBinds` is computed on the pre-composition throttle (`:2306`), `ComposeThrottleCap` runs its argmin and then overwrites the tag with `Offtrack` unconditionally when that flag is set (`:3063`), and the trim itself happens after the composition (`:2327`). If another reason had already lowered the pedal below the projection level, the value is that reason's and the tag still says `Offtrack`, with `ThrottleReasonLevel` holding the argmin's number (`:3069`). The projection has no `ThrottleReason` of its own, so the standing answer to "why isn't the pedal full" is wrong in exactly the case the WIP note calls the term that outranks every reason field (`AGENTS/AGENTS-WIP.md:147`). | `Racer.cs:2306-2307`, `:2327`, `:3053-3070`. | VERIFIED | Hold a car outside and widening while a second cap also binds (TCS, stability or the rival lift); the readout names Offtrack while the bar sits on the other cap's level. |
| 204-06 | file | **`Control.MaxThrottle` and `Control.MaxBrake` are published and never read, and the projection brake can exceed the ceiling they publish.** Both are assignments only (`:2212-2213`, `:3051`, `:3077-3078`); no reader exists in `src/`. The applied brake can be 0.5 while `Control.MaxBrake` is 0.2, because the projection branch takes `MaxBrakeFromABS` alone (`:2319`) after the composition has already min'd in the route cap. Harmless today — nothing reads either field — and a trap for the next reader who trusts them. | `Racer.cs:2319`, `:3044-3051`, `:3077-3078`; exhaustive grep for `.MaxThrottle` and `.MaxBrake` in `src/` returns assignments only. | VERIFIED | Apply the projection floor while `MaxBrakeFromRoute` is 0.2: the written brake is 0.5 and `Control.MaxBrake` reads 0.2. |
| 204-07 | file | **The reverse override is a second pedal writer that bypasses all eleven caps, and it publishes a level of 0 for a -0.5 throttle.** `ApplyStuckRecoveryOverride` (`:5152-5163`) runs after `ConvertSpeedToPedals` (`:4801`) and writes `Control.Throttle = -0.5` and `Control.Brake = 0`, with `ThrottleReasonLevel = 0` (`:5160`) while the applied throttle is -0.5. It is the only path that can produce a negative throttle, so the exception is deliberate; the level is a lie either way. | `Racer.cs:4801`, `:5152-5163`; the only other writer of `Control.Throttle` is the slew at `:2334`. | VERIFIED | Trigger a recovery; the throttle reads -0.5 while the reason level reads 0.00. |
| 204-08 | file | **`FindLowestIntendedSpeed` finds no lowest: it clamps one value.** `:2360-2366` is `if (Speed >= 0) Speed = Min(Speed, MaxSpeed)`. The name promises a reduction over several candidates, of which there is one; under the project's naming rule it should say what it does (clamp to the model ceiling) or move into the caller. | `Racer.cs:2360-2366`; single call site `:2271`. | VERIFIED | Read the body against the name. |
| 204-09 | file | ⚠️ **The brief's 1:6 is not this build's law: the pair is 1:3, and the comment above the constants and the line pointers in the notes still say otherwise.** `FullPedalSpeedErrorMps = 1` (`:272`) and `BrakeErrorMultiplier = 3` (`:274`) feed the only divisor at `:2343`, so full brake is 3 m/s = 6.71 mph of error, not 13.42. The comment at `:269-271` says "Braking keeps its six, so the pair is now 1:6"; `AGENTS/AGENTS-WIP.md:544-548` already says 1:3, so the comment is the stale side. The same notes point at `Racer.cs:1375` for `ConvertSpeedToPedals`, `:1672` for `UpdateThrottleReasonCaps` and `:1765` for `ComposeThrottleCap` (`AGENTS/AGENTS-STEERING.md:559`, `:879-880`); in this build those methods are at `:2264`, `:2913` and `:3037`. | `Racer.cs:269-274`, `:2343`; `AGENTS/AGENTS-WIP.md:544-548`; `AGENTS/AGENTS-STEERING.md:559`, `:879-880`; current line numbers from the source. | VERIFIED | Read the constant and divide: 3 x 2.23694 = 6.71 mph. |
| 204-10 | file | **When the plan flips from full throttle to full brake in one tick, both pedals are non-zero for ~167 ms, because the two slews are independent.** The split guarantees one pedal per tick (`:2349-2355`), but `Control.Throttle` falls at 6/s while `Control.Brake` rises at the same rate (`:2333-2334`), so a +1 to -1 flip leaves ~167 ms in which both are applied — 4.5 m at 60 mph. The composition comment already knows the throttle can still read at or above 1.0 while the car is braking (`:2309-2310`); it does not say that the brake is equally late in the same window. | `Racer.cs:2333-2334`, `:2349-2355`, `:2309-2310`, slew rate `:2261`. | VERIFIED (mechanism and arithmetic); PROBABLE (the effect on braking) | Log both pedals for one corner where the plan flips from throttle to brake; there are ~10 frames at 60 fps with both above zero. |

---

## Questions that closed with no defect

- **Can one cap step on another and leave the car with no pedal?** Not by composition. Every throttle
  level is in `[0,1]` and the ceiling is a chain of `Math.Min` over all eight (`:3044-3051`); the brake is
  the same over its three (`:3077-3078`). A zero can only come from a cap that states zero: stability,
  off-track above 20 mph (`:2853`, `:2953`) and recovery above 25 mph (`:2962`) — each a stated intent.
  Ties go to the earlier reason in a fixed order (`:3055-3062`) while the published ceiling is the plain
  min, so tag and value agree except in 204-05.
- **Can a cap stay stuck after a repair, a respawn or the flag?** Repair (`:4759`) and `SnapToTrack`
  (`:5165-5178`) touch no cap. The eight glide fields recover at 1.75-3.5/s, i.e. 0.29-0.57 s from any
  level, and stability at 0.5/s, i.e. 2 s (`:2840`, `:2881`, `:2922`); `Launch()` resets every cap field —
  the eleven reason caps plus the two published ceilings — and both reason tags and levels (`:2212-2245`).
  The one that does not recover is `MaxBrakeFromRoute` after the flag — 204-02.
- **Does the projection clock survive a race restart?** It is not reset by `Launch()`, but it cannot
  outlive the first tick: the grid is below `OffshootMinSpeedMph` 20, so `MeasureProjectionOutside`
  leaves `_projectionWidening` false (`:2378`, `:2384`), and `:2304` zeroes the clock before the input is
  computed. Closed, and worth remembering as a near miss rather than a bug.
- **Is `TickScale` ever 0?** No. `Clamp(..., 1, 9999)` ms at `:110`; a zero would freeze both pedals
  because each slew step is 0 (`:2267`, `:2333-2334`). The floor is the guard.
- **Is the chain live for the player?** No — see the header. The player's `Control.Throttle`/`Brake` are
  never written, so the whole of this report describes the AI field.

---

## Where this report stopped rather than invent

- **The live value of `wheel+0x1C4` on a stationary but loaded car.** The engine writes `0.0f` when
  forward speed is at or under 0.1 m/s and the lateral spring is unloaded; whether a stopped car always
  meets both is what 204-01 turns on. A one-line log of `ARS.WheelsOffGround` and
  `Control.MaxThrottleFromStability` on a car held at 0 mph for 2 s settles it, and nothing read-only can.
- **Whether the shipped finish lines sit on a stretch where the route term limits** (204-02). One log line
  after a flag names it; the code path is proven, the trigger is not.
- **The feel of any of it.** Nothing here is driver-verified. The reproduction column names the readout
  fields to watch, not a result seen.
- **GTA's behaviour with throttle and brake applied together** (204-10). The engine's brake-versus-drive
  arbitration was not read, so the cost is stated in metres and frames, not in lost deceleration.
