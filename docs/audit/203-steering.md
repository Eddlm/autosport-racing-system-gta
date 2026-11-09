# AUDIT 203 — the front-slip guard from the inside

**Surface**: the whole front-slip chain as it is compiled today — `MeasureFrontSlip` and its two sources
(`Racer.cs:1944-2024`), the chase state and the two-sided clamp (`:2073-2149`), the actuator it writes
into (`TranslateSteerToInput`, `:2587-2601`), the pedal that shares its signal (`:2277-2286`), the
readout (`:3711-3723`) and the one other consumer of the granted ceiling (`:5103`). Wheel memory:
`AutosportRacingSystem.cs:3161-3327`; the published yaw and slide it reads: `DataStructures.cs:25-26`.

**Date**: 2026-10-25.

**Build audited**: **1092**. `build/build-number.txt` reads `1092`; `bin/Release/ARS.dll` is sha256
`8e3ab22dc3b3ff8a1dda4dfd3cd8a7f5c8017f36206032ffa3210dea6c983285`; source `HEAD e851a09`.

**Method**: read-only. Source read, `git log` read, the engine's own traction curve re-read in the
leaked tree (`E:/GTA/GTAVSP/GTAV Source/src/dev_ng/game/Vehicles/wheel.cpp:94-96`, `:3170-3186`), and
the constants as written put through `awk`. No build, deploy, commit or game run.

**Status vocabulary**: **VERIFIED** = read in the code, or reproduced read-only (arithmetic, source
read). **PROBABLE** = one step rests on a behaviour not re-observed today. Nothing here is
driver-verified.

**Verdict**: **no blocker on this surface.** The three places the brief suspected a hole — a NaN
`chasingClamp`, a ceiling left over from another car, a left/right asymmetry in the countersteer and
damper branches — all close, with a source each (see "Questions that closed with no defect"). What is
wrong is elsewhere: the chase's authority at the threshold (203-01), the 180°/s slew sitting between the
limiter's write and the input it believes it has set (203-02), and a ceiling that outlives the ticks on
which the guard cannot speak (203-03).

---

## Findings

Release-notes first, in the order they should be read; then the files. Every row carries its source.

| # | Sev | Claim | Evidence | Status | How to reproduce |
|---|---|---|---|---|---|
| 203-01 | release-note | **Below ~119.5 % of the target the gate is a net opener, so it cannot hold the wheel at the peak; and because `clampTarget` steps from the full lock to `abs(request) × factor` across the same 100 % line, there is no value for it to settle on.** At 100 % the closing rate is 1.00 °/s against a flat 4 °/s opening; they cross at 119.5 %; only above ~150 % does closing reach 8 °/s. The wheel is therefore held *above* the peak, by however much the slip lags the command, and the loop chatters around the step instead of arriving. | Rates: `Racer.cs:1936-1942` (tanh over `SlipCeilingCloseSpanPct = 75`), constants `:316-321`; the step: `:2105-2107`; the opening branch: `:2111-2113`. Cross-over 119.47 % computed from those constants. The engine curve the saturation is tied to is real: `wheel.cpp:94-96` (`sfTractionPeakAngle` 1.0, `sfTractionMinAngle` 2.5), linear fall `:3170-3186`. | VERIFIED (code + arithmetic) | ShowFrontSlip; hold a corner where `F` sits at 100-120 — `ratio` keeps crossing 1.00 without parking, and `factor` moves while the mph does not. The doc's own good-corner row (`docs/steering-and-pedals.md:61-64`, ratio 1.82-2.50 with factor 0.40-0.42) is the same effect at a bigger slip. |
| 203-02 | release-note | **The limiter's output is only a target for the 180 °/s slew, which starts from the angle already applied — so the wheel runs above the ceiling for ~8-13 frames after a cut, and reaches a raised ceiling just as late.** `ApplySteerLimits` writes the clamped angle, then `TranslateSteerToInput` re-writes the same field as `LastApplied + clamped(error)`, where `LastApplied` is unconstrained by this tick's ceiling. A 40°→16° cut takes 133 ms (8 frames at 60 fps); a collapse to 2° takes 211 ms (13 frames). The guard's next measurement is taken from a wheel still above the ceiling it believes it has set. The same lag applies in the dangerous direction: the countersteer/damper whitelist raises its side to the full lock and the slew needs the same 0.13-0.21 s to deliver it, during a slide. | `Racer.cs:2146` (the clamp) then `:2592-2596` (the slew, `SteerSlewRate = 180` at `:1826`, `TickScale` at `:2582`); whitelist raise `:2127-2144`. Arithmetic: 24°/180 °/s = 133 ms; 38°/180 = 211 ms; 180/60 = 3°/frame. | VERIFIED | Log `Control.SteerDegrees` immediately after `ApplySteerLimits` and again after `TranslateSteerToInput` for one hard cut; the second value is the first only after 8-13 ticks. |
| 203-03 | release-note | **`chasingClamp` is written in exactly one place and never reset, so its value survives every tick on which the guard could not speak — reverse, a wheel-memory miss, the theory source below 3 m/s.** Those ticks apply no limit (correct) but leave the ceiling where it was; the next readable tick resumes from it and reopens at 4 °/s. Bounded (the target is the lock whenever the tyre is under 100 %), but a ceiling parked at 2° after a spin needs 9 s to reach a 38° lock, and the readout divides by it as though it were current. | Only assignment: `Racer.cs:2115`; declaration `:308`; the unreadable returns `:1948`, `:1971`, `:2007-2008`; the guard gate `:2096`; the readout's use `:3717`. No reset in the class. | VERIFIED (exhaustive assignment check) | Spin the car with the wheel centred, let it stop, then drive on: the readout's `ratio` starts high and decays as the ceiling walks back up at 4 °/s. |
| 203-04 | release-note | ⚠️ **`docs/steering-and-pedals.md` documents a law that is not the one compiled.** Three drifts: the slow-corner allowance is presented as live at 30/10 mph (`:9`, `:48-55`) while the code parks it and puts the band at 15/5; `maxSlip` is given as `LateralPeakAtSpeed × FrontSlipCapPeakShare` while the shipped (memory) source never calls `LateralPeakAtSpeed` and no `maxSlip` symbol exists in `src/`; and two comments contradict the constants under them. | Doc `:9`, `:48-55`, `:52`, `:55`; code `SlowCornerSlipAllowance = false` `Racer.cs:1839` with `15/5` `:1840-1841`; `LateralPeakAtSpeed` callers are `:1915` and the parked ramp `:2054` only (memory source `:1967-1993` uses `FrontSlipCapPeakShare` directly, `:1987-1988`). Stale comments: `:1830-1831` ("falls at 2 deg/s and rises at twice that" vs 1-10/4 at `:316-318`), `:1845-1847` ("five degrees past neutral" vs `DamperCountersteerMaxDeg = 2f`, `AutosportRacingSystem.cs:179`). The release notes are right: `docs/release-0.9.0.md:69`. | VERIFIED | Read the doc's "Slow corners" section against `Racer.cs:1839-1841`; flip `SlowCornerSlipAllowance` and nothing changes, because nothing reads it. |
| 203-05 | release-note | **The two actuators that share one threshold read measurements one core tick apart, and the pedal reads the older one.** Within a tick the pedal law runs before `ApplySteerLimits` refreshes `_slip`, so `ConvertSpeedToPedals` trims the throttle on the previous tick's front slip while the wheel uses this tick's. 16 ms at 60 fps — immaterial at speed, but it is the reverse of the safe order: the slower actuator gets the staler signal, and the first tick after a reset gets an unreadable one. | Pipeline order `Racer.cs:4789-4805`; pedal reads `_slip` at `:2281-2285`; `_slip` is assigned at `:2091`, inside `ApplySteerLimits`. | VERIFIED | Log `_slip.InnerPct` at the pedal and at the limiter for one frame; the pedal's value is the previous frame's limiter value. |
| 203-06 | file | **The memory source trusts the first two wheel entries as the front pair and only rejects a count below 2.** `GetWheelPtrs` returns the engine's array unfiltered; `[0]`/`[1]` are read as the fronts, and `RearPct` needs four or reads 0 — which the readout prints as `R 0`, i.e. "the rears are fine". On any vehicle whose wheel count is not 4 the "front pair" contains something else. Inert for shipped grids — motorcycles and cycles are blacklisted — so this is debt, not a live defect. The ordering of a 6-wheeled array was **not** verified (see the note at the end). | `AutosportRacingSystem.cs:3161-3175` (count guard `:3168`), `:3318-3327`; `Racer.cs:1971` (`< 2`), `:1974-1975`, `:1989-1991`; readout `:3722`; blacklist `AutosportRacingSystem.cs:131-136`. | VERIFIED (path); the 6-wheel order is UNVERIFIED | Force a 6-wheeled or 3-wheeled model onto the grid; `R` reads 0 in a slide that has both axles gone. |
| 203-07 | file | **The memory source has no NaN test while the theory source does.** `MeasureSlipFromTheory` tests speed, grip and wheelbase for NaN before its floors (`:2006`); `MeasureSlipFromMemory` takes `Math.Abs` and `Math.Max` on raw floats — both propagate NaN — and then sets `Readable = true`. A NaN slip fails the `InnerPct > 0.01f` gate, the guard goes silent, `factor` reads 1.00 and the readout prints `F NaN`. No crash: it fails open, with nothing logged. | `Racer.cs:2005-2007` (the guard that exists) vs `:1972-1988` (the source without one); gate `:2096`; `Math.Abs`/`Math.Max` NaN propagation; `ARS.Clamp` returns the *min* bound on NaN (`AutosportRacingSystem.cs:645-650`, `CompareTo` puts NaN below everything). | VERIFIED (path), PROBABLE (that the field can hold NaN) | Nothing observed produces one; a debugger write of NaN into `wheel+0x1C4` is the only reproduction, and the mod keeps running with the guard off. |
| 203-08 | file | **The readout's `ratio` divides by the chase, not by the ceiling actually granted, so it reads above 1 while the wheel is not being cut at all.** During a slide the damper branch sets its side to the full lock and the countersteer branch to the slide angle; the readout still divides by `chasingClamp`, and `_steerGrantedCeiling` — the value that knows — is already read by the kick. | `Racer.cs:3717-3720` (divides by `_chasingClampDeg`) vs `:2129-2131`, `:2142-2143` (the raises) and `:2148` (`_steerGrantedCeiling`), consumed at `:5103`. | VERIFIED | ShowFrontSlip in a countersteer: `ratio` stays above 1.00 while the car answers the lock and the wheels are visibly ahead of the clamp. |
| 203-09 | file | **On the NaN early return the limiter leaves its published state stale.** The return at `:2076-2080` skips `_slip`, `_slipFactor`, `_finalCeilingDeg`, `_steerPreLimitDegrees` and `_steerGrantedCeiling`, so the kick's arm test reads the previous tick's pair, and the readout draws last tick's slip with a zeroed command. | `Racer.cs:2076-2080` (the return) against the writes at `:2091-2092`, `:2101`, `:2120`, `:2147-2148`; the reader `:5103`. | VERIFIED | One tick, no crash; visible only if the NaN is forced. |
| 203-10 | file | **The same wheel array is walked three times per car per frame and the guard is the third.** `WheelsOffGround`, `MaxWheelSlip` and now `WheelSideSlipAngles` each build their own `List<ulong>` inside `GetWheelPtrs`; the guard adds a `List<float>` on top. At 20 racers and 60 fps that is ~2,400 list allocations a second for one array, against the codebase's own "compute once, publish" rule. The impact is **not measurable read-only**, so it is not graded above `file`. | `Racer.cs:2918`, `:2978`, `:1970`; `AutosportRacingSystem.cs:3161-3175`, `:3318-3327`. | VERIFIED (the allocations), PROBABLE (the impact) | Count `GetWheelPtrs` calls per frame; three per racer, all returning the same pointers. |

---

## Questions that closed with no defect

Four of the brief's suspicions did not survive the code. Stated so the next session does not re-open them.

- **`chasingClamp` cannot go NaN.** It has exactly one assignment (`Racer.cs:2115`), and on that path
  `clampTarget` is either the lock or `|request| × factor` with a finite factor (the gate `:2096`
  excludes NaN and infinity), while `rate` is a sum of two constants times a bounded tanh. It also
  cannot exceed the car's lock: `speedCeiling = min(speedCeiling, chase)` at `:2118` sits under
  `speedCeiling = ResolveSteerCeiling(...) = SteeringLock` at `:2090`.
- **It cannot be left over from another car.** `Car` is written only in the constructor
  (`Racer.cs:551`) and racers only ever come from `new Racer(...)`
  (`AutosportRacingSystem.cs:2573`, `:4075`), so a different car means a different object, whose chase
  is NaN and is born at that car's own target (`:2108`). The persistence that *does* exist is across
  unreadable ticks on the same car, and that is 203-03.
- **The countersteer and damper branches are genuinely sign-symmetric.** `countersteering` is a sign
  inequality (`:2088`) over a yaw rate published with the same left-positive convention as the command
  (`:5241`, `GET_ENTITY_ROTATION_VELOCITY.Z`, converted by `RadToDeg`); each whitelist mirrors its two
  sides (`:2129-2131`, `:2142-2143`); the damper's side choice is the product test
  `_damperTermDeg * yawRate < 0` (`:2140`), which is symmetric by construction; the damper term itself
  is killed on the command's own side (`:876`), crosses neutral symmetrically (`:880-884`) and is
  clamped symmetrically (`:888-889`). The only two asymmetries are cosmetic and cancel: the `else` at
  `:2131` grants the *right* side when the request is exactly zero, and `requestedSteer >= 0` at
  `:2148` then reports the *left* ceiling — a zero request clamps to zero either way. **The reported
  left/right difference is not explained by these two branches**; the remaining suspects are the
  slide's sign chain upstream (`:901-922`) and the pursuit, neither of which this audit covered.
- **The theory source is the geometry ceiling re-derived, not a second sensor.** Its fixed point is
  `arg × δ/tan δ` radians with `arg = L·grip·g/v²`, i.e. `AckermannCeilingDegrees` (`:1899-1906`) to
  within 0.2 % for any request up to 5° (2.11° at 60 mph, L 2.7, grip 1.0). That is why parking
  `SpeedSteerCeiling` costs the theory path nothing — and why it costs the shipped path something: with
  `SlipFromMemory = true` (`AutosportRacingSystem.cs:170`) the limiter is a real feedback law, not the
  same law twice.

---

## Where this report stopped rather than invent

- **The wheel-array order for a six-wheeled vehicle.** `RearPct` reads `[2]` and `[3]`; whether those are
  the rear pair or the middle pair on a 6×4 is an engine detail I did not read, and 203-06 turns on it.
- **Any in-game effect.** Nothing here is driver-verified; the reproduction column names the readout
  fields to watch, not a result seen.
- **The tyre curve's numbers are cited, not re-measured on track.** `sfTractionPeakAngle` 1.0 and
  `sfTractionMinAngle` 2.5 *were* re-read today in `wheel.cpp:94-96`; `fLoss` and the per-surface scaling
  around them were not followed, so the claim "ARS reads the same axis" rests on the field being the
  curve's own argument, as its consumers already assume.
