# Steering control — how this is actually done elsewhere

**Read this when** the request or the bug touches: steering controller, PID, P/I/D, pure pursuit, Stanley, aim point, lookahead, preview, gain, gain scheduling, feedforward, anti-windup, integral clamp, countersteer, opposite lock, oscillation, weaving, "why does it oscillate", "how do other games do it", "is a PID the right tool", **loop order, damping ratio, "why is it stable"**, standing error, straight-line drift, steering limiter, steer authority, **grip-normalised gain**, velocity-vector vs heading reference.

**Depth lives in `STEERING-CONTROLLER-SURVEY.md`**:
36 implementations, 10 documented disagreements, every claim carrying its source URL, inaccessible sources listed rather than papered over.
This file is the distillation; the survey is the evidence.
ARS comparisons here are the agent's analysis and go stale — **the code wins**.
**The goal behind a system — the idea, the shape it takes and the shapes already rejected, with no line numbers — is `DESIGN.md`.** That file says what a system is *for* and this one says what is *live*; read it before reshaping anything here.

## ⚠️ STATUS — the PID / measured-yaw-rate law in this file was tried and ABANDONED

**Everything below about a P/I/D law applied to the *aim error*, and about a correction reading the *measured yaw rate*, records a design that was built, driven through many variants, and then rolled back.**
The live steering system is the **pre-PID authority chain, restored from `1f4516d`** and simplified since (`cd6efea`):
**pure pursuit** on the bearing to the aim point (the bearing clamped past abeam) + a side-by-side heading correction + a damper whose reference is that same aim's curvature + the countersteer blend, then the steer limiter and the slew.
There is no course-error term, no off-track recovery term and no outer PD, and the rival walls bound the *lane request* and never the aim (`Racer.cs` `ApplyRivalWalls`).

Four supersessions, each stated at its own section below:
- the live **course** error is **velocity**-referenced (not nose);
- there is **no PID** — no `SteerI`, no `SteerD`/`SteerTrim`; the steering knob is `SteerDampingGain`;
- the damper targets the yaw the **aim point** requires (`SteerDampingAimReference`), the zero reference kept one flip away for A/B rather than deleted;
- the steerable ceiling has one governor (the peak-slip cap): the yaw-usage turn-in share that replaced the Bias/Slip dials was itself retired, and the slide's `Math.Max` loosening was retired by decision and has since been deleted outright (see the limiter paragraph).

**The lessons worth carrying forward** (they are why this section is kept):
- a reference that contains its own correction closes a measurement loop through its own input;
- a veto that **drops** rather than **clamps** turns a monotonic dial into a cliff;
- `v·tan(δ)/L` is a fiction once the tyres leave their linear range;
- the field's "read yaw rate directly instead of differentiating" advice is sound in principle but **did not beat a well-tuned PD chain in this game**.

## What the restored chain actually is, and where its stability comes from

**This section reads the live chain as a control loop; the design it replaced is kept only where a paragraph is marked as history.**

**The analysis of the live law (`ComputeSteering`).** The terms that are always on:

- There is **no course-error term, no recovery term and no `laneBias` term**.
- The lane becomes a steer by pure pursuit, and pursuit is the loop's only position law.

**The aim point is the pursuit's input, not a debug ornament.**
It is built at `Racer.cs:574-582` and handed straight to `PursuitSteerDegrees` (`Racer.cs:586`), whose law is an `atan` on the bearing from the car's travel direction to it (`PursuitSteerFromBearing` `Racer.cs:786-790`) —
so the live chain does carry a geometric gain and an aim angle, and the lane becomes a steer here and nowhere else.
The bearing clamps at `MaxPursuitBearingDegrees` so `sin` cannot fade past abeam (`Racer.cs:684`, `:694`), and inside `SmallAimBearingDegrees` with no slide the command is zeroed outright (`Racer.cs:687`, `:617`).
The aim's lateral coordinate also carries the **Gs-aware lead**: the aim is the carried lane minus half of the lane error measured at the car's own 0.5 s kinematic projection — position + velocity·t + half the smoothed acceleration·t² (`ProjectAhead` `Racer.cs:2442`, read as a lane offset `Racer.cs:578`, blend `GsPreviewBlend` 0.5 `:137`) — so the correction leads the lateral drift instead of reacting after it lands, and the lead is nil at steady state. **Always on** — the behaviour toggle behind it was de-gated, and its menu slot is repurposed as the `ShowGsProjection` debug drawing of that same path (`AutosportRacingSystem.cs:1259`, drawn from the roof `Racer.cs:2615`).

**The stability is loop *order*, not the setpoint: pursuit is position feedback driving a plant whose input is a rate, and the damper is the loop's only rate feedback.**
To first order the pursuit acts on *where the car is*, not on where it points, and nothing but the damper closes that second-order loop (`Racer.cs:596`, `:602-603`).

- **There is no integrator anywhere, and no curvature feedforward either.**
  Nothing integrates against the command `ApplySteerLimits` caps (`Racer.cs:1120`), so the anti-windup item below is moot while there is no integral; the literature's standing-error job falls to the pursuit, because it acts on **position** — a car offset from its aim carries a bearing back to it and converges rather than settling at a permanent offset.
- **Saturation, not algebra, is what the fast case meets.**
  `sin` caps what a swinging aim may ask for (`Racer.cs:684`), and the limiter closes the command last on the peak-slip cap, with corner geometry only as its fallback (`Racer.cs:1045`) — the reason a limit corner's steer must also carry the slip angle that generates the force, which is the quantity `TRLateralAtSpeed` feeds `PeakSlipCeilingAt` (`Racer.cs:1014`, `:1035`).
- **Rejected design, kept as history.**
  The abandoned PID law put P on the *aim error* with **no position term at all**, so its standing line error could only be fought by raising P — the "P 1.0 swerves / P 0.5 is stable but lazy" trade — and an I term would have added the very phase lag it was meant to remove.
  The lever for "lazy" was never P: the live lane's one gain is `PursuitGain` (`Racer.cs:682`), and the dial that looks like a steering gain is the damper's (`Racer.cs:970`).

**Two properties of the heading reference, usually mis-stated:**

1. **The reference is the velocity vector, and it is the bearing's origin, not a course error** (`courseDir = Car.Velocity.Normalized`, `Racer.cs:508`).
   The pursuit measures the aim's bearing from the car's *travel* direction (`Racer.cs:707`), so a body rotated hard against its own velocity reads as perfectly tracked — the slide-blindness that made this reference worth keeping.
2. **The preview is a constant *time*, clamped in distance, and grip never enters it.**
   `LookAhead.SteerRef` is `speed × SteerPreviewSeconds` clamped to a floor and a cap in metres (`Racer.cs:2670`, `:129-131`): below the floor the time stretches, above the cap it shortens, and inside the band the same lateral error asks for less steer the faster the car goes — the loop's gain falls with speed without a gain schedule being written anywhere.
   Nothing in the steer reference or the aim reads grip, so no part of the live lane law is grip-scheduled.

**The straight-line regime, and why an offset on it is held rather than converged away.**
With no corner line and no avoidance, the aim is the lane the car is already in (the Lane systems base rule), so the lane steer is the pursuit bearing to that point: **zero only when the car is already on the aim**, and a real correction for a car holding a lateral offset — the pursuit acts on position.
The pursuit is not gated either (see the ungated-pursuit section), so a straight is lane work, not a regime where the lane law is switched off.

**One body-yaw feedback path, and it is the loop's only rate term.**
The signal is the raw native yaw rate — `GET_ENTITY_ROTATION_VELOCITY.Z` in °/s, unfiltered and undifferentiated (`Racer.cs:3591`).
The term is `−gain·(yawRate − yawTarget)`, where `yawTarget` is the yaw the **aim point** requires, `fwdSpeed × _steerAimCurvature`, dropped above `LateralTractionCurve × SlidingFraction` because a sliding car's wanted rotation is the countersteer's (`Racer.cs:589-590`, `:714`, `:942`) — the reference is live (`SteerDampingAimReference`, `Racer.cs:960`).
The gain is `SteerDampingFor`, and the term never adds lock: one that agrees in sign with the rest of the command is zeroed, and one that crosses neutral answers with its overshoot whole (`DamperCrossingShare` 1) **capped at `DamperCountersteerMaxDeg` 2 degrees past neutral**, plus an unconditional magnitude bound so a straight-line command cannot be steered by the term with no ceiling. Driven history: the plain law at gain 0.5 held, and so did a /3 and a /2 slope — this is the amplitude form of the same idea, back by driver request once the slide blend was relaxed and the damper was again the loosest source of opposite lock. **The crossing is not capped and its slope is not cut, and that is a driven result**: three shapes were built and driven — a flat 5° ceiling, a third of the overshoot across the whole range, and a 2° free band with half of the excess beyond it — and each did its job only because the gain was wrong. With `SteerDampingGain` at 0.5 the plain law does all of it, so the shapes were removed rather than tuned. The gain is the one knob; if a snap returns, lower it before re-inventing a ceiling.
**The whole term is scaled by `DamperEffectScale`, and it sits back at 1 — full authority.** The slide batch drove the scale at 0.5 and the slides answered, but that field read a smidge unstable, so the scale returns to whole and what stays from the batch is the blend's band and the countersteer's share; halving only the crossing while the base term ran at full strength never helped the slide at all.
`steerKP` is unity and dimensionless, so the pursuit and the side-by-side correction enter at 1.0 against that gain (`Racer.cs:577`, `:595`), and there is no course feedback anywhere in the sum.

- **Rejected design, kept as history — the zero-reference damper, and the toll it paid.**
  Until the term damped the *excess over* the yaw the aim requires, it damped against zero, so in a steady corner it removed `gain × ω` of command the corner needed — paid as a **standing lateral offset from the carried lane**, growing with speed and corner tightness, which is the mechanism behind "stable but a little lazy".
  Re-referencing removes the setpoint error without weakening the damping.
  The durable trap that build exposed is a convention, not a number: a reference built in the steer command's own frame (positive = left) must **not** be negated, or the damper doubles its opposition in corners instead of removing the toll.

- **Removing the damper is not the fix for that toll — driven, and the term is load-bearing.**
  With `SteerDampingEnabled` off the gain is zero and the sum falls back to the pursuit plus the side-by-side match (`Racer.cs:957`, `:970`, `:595`), and the observed behaviour is that the cars **cannot hold centre**: a limit cycle around zero steer — steer in, overshoot, steer back.
  Deleting the term leaves position feedback with no rate feedback at all on a rate plant, which is why it reads as stabilisation rather than trim (`Racer.cs:955-956`); past the tyres' linear range the damper, not the proportional term, is likewise what restrains the loop.

**Two couplings and one sign test on a raw signal:**
- The limiter's countersteer decision is `Math.Sign(requestedSteer) != Math.Sign(VehicleData.YawRotationPerSecondDegrees)` — a sign test on the unfiltered native read (`Racer.cs:1086`, `:3591`), so a car dithering across zero yaw flickers it, and `Math.Sign(0)` matches nothing.
- The damper is not private to the sum: the limiter reads `_damperTermDeg` to decide whether the answering side may run to full lock (`Racer.cs:1114-1118`).
- The slide blend starts from the damper-inclusive command while its countersteer target carries no damper at all, so moving the damper moves where the blend starts (`Racer.cs:605`, `:611-612`).

- **Rejected design, kept as history — the track-curvature reference (`f8304f9`).**
  A damper referenced to the track's own required yaw (`r_des = v·κ`) was built and then deleted, along with its switch (`SteerDampingTrackReference`) and the steer variable it fed; what it left behind is the convention trap above.
  Its cost estimate also flattered the zero form: "a corner only needs `L/R` of steer" counts the kinematic angle alone, while a limit corner must also carry the slip angle that generates the force — the quantity `TRLateralAtSpeed` feeds `PeakSlipCeilingAt` (`Racer.cs:1035`, see the saturation bullet above).

**The gain is grip-normalised and constant with speed (`SteerDampingFor`, `Racer.cs`).**
`SteerDampingGain` is an **absolute gain in seconds**: the dial *is* the gain, divided by the floored `BaseMechanicalGrip` — read rather than the live perceived grip, so at a fixed dial the term is a **uniform multiplier** (every car's term moves by the same ratio) and a low grip reading cannot blow it up.
**The speed schedule is REMOVED (driver):** the term used to fall as a reference speed over the car's above that speed, and at a dial of 1.0 the cars damped cleanly at every speed, so the axis, its menu item and `ARS.SteerDampingSpeedMph` are gone; a stored `SteerDampingSpeed` is dropped by the repair pass as an undeclared key.
The divisor is grip itself rather than its square root, so the term falls faster with grip than a constant-damping-ratio law would need.
Speed therefore enters the live loop only through the pursuit's growing `range`.

## The dominant architecture is feedforward + feedback, not a lone PID

Every serious implementation surveyed splits the steering command into an open-loop **feedforward** term and a closed-loop **feedback** term — a two-degree-of-freedom architecture, explicitly named as such in the racing literature:

```
delta = delta_feedforward(curvature of the line ahead)  +  delta_feedback(PID on cross-track error)
```

- [Control-Theoretic PID Steering](https://www.emergentmind.com/topics/control-theoretic-steering-pid) states the law outright: `delta(t) = delta_pp(t) + delta_pid(t)`, feedforward geometric (pure-pursuit) term plus summed PID feedback.
- [Speedgoat, autonomous racing](https://www.speedgoat.com/Portals/0/adam/Content/0weSCmFk8U6yqmpJ_9cnug/Text/Sensor%20Fusion%20and%20Motion%20Control%20for%20Autonomous%20Racing%20Cars.pdf): "All controllers are based upon a two degrees of freedom architecture, separating the control request generation into a feedforward part and a feedback part".
- [Stanford, Kapania ch.2 "Feedforward-Feedback Steering Controller"](https://ddl.stanford.edu/sites/g/files/sbiybj25996/files/media/file/2016_thesis_kapania_0.pdf); [TUM's feedforward benchmark](https://arxiv.org/abs/2605.21111).
- The richest shipped example is **Ziggy Racer** (Forza Horizon 6, virtual gamepad, ~71 Hz): `steer = k_ff·κ·load_comp + k_head·α + PID(cross-track) + countersteer(sideslip)`.

**Why the feedforward exists:** a feedback-only law must carry a *standing error* to produce a steady command, so P alone settles a permanent few degrees of error — a permanent lane offset or corner cut.
The literature's first fix is the feedforward; the integrator is the fallback.

## Pure pursuit, stated properly

Pure pursuit is **itself a proportional controller**: the [Sensors 2025 pure-pursuit paper](https://pmc.ncbi.nlm.nih.gov/articles/PMC11820862/) gives the gain as `2/l_d²` on lateral error (so `2L/l_d²` in steering-angle terms), equivalently `(2L/l_d)·α` on the aim angle. The [autonomous-driving-book](https://github.com/YangyangFu/autonomous-driving-book/blob/main/book/3-trajectory-tracking/lateral-control/pure-pursuit.md) notes it "can be tuned at different speeds by creating a relationship between the speed and the lookahead distance."

**ARS's live lane law *is* the geometry form — `atan(2 × wheelbase × sin(bearing) / aimDistance)` — and no `SteerTrim` survives to trim it.**
What was tried and abandoned was the **PID applied to the aim error**, together with the trim and differentiator machinery around it; the geometry itself was never the casualty and is the live pursuit.
The live chain (see the STATUS banner) pursues the lane with pure pursuit on the aim point's bearing, plus a side-by-side heading correction, a damper referenced to that aim's curvature and the slide blend.
**The pure-pursuit gain lesson still holds**: `sin` rather than the raw angle saturates the demand as the aim point swings abeam instead of asking for full lock, and the gain self-schedules only because the aim distance moves with speed.
**Two repair-policy lessons from that era, still load-bearing:**
- the repair pass **snaps an in-list numeric key to the nearest offer** rather than resetting it, so an off-list stored value becomes a permanent and *invisible* error;
- **every declared default must be a member of its own domain list**, or the repair rewrites it on the next launch.

And `SettingsRepair` **prunes any declared-then-undeclared key** — that is the mechanism that retires a key from every existing ini, which is how `SteerTrim` / `SteerD` / `SteerI` went when the PID was abandoned and how `SteerKD` came back — renamed to `SteerDampingScale` for the grip-normalised era, and to `SteerDampingGain` when the dial became the gain directly.

## What is universal is *preview* scheduling, not gain scheduling

**Correction to an earlier claim in this file.** "Gain scheduling by speed is universal" is too strong.
Most implementations **never touch the gain** — TORCS `berniw`/`bt`/`damned`, USR default and Assetto Corsa hold a flat `1/steerLock` or per-car `STEER_GAIN` — and the *effective* gain moves only because the **aim distance** moves.
Explicit gain scheduling is the minority, and it contradicts itself in direction:

- **More gain at low speed**: Game AI Pro ch.40 — "at low speed the vehicle may require **much larger** K values".
- **Less authority at high speed**: Unity Standard Assets (`maxSteerAngle` 28° → 23%), Unity's own tutorial (`steeringRange` 30° → 10°), NwliZz, and ForzaETH's `k_speed`. ForzaETH even ships **both** an upscale and a downscale in the same code, silently fighting each other.
- **A band-pass**: USR `SteerMod` reduces gain both above target speed and below it, with the reasons stated verbatim ("stop bouncing over curbs"; "avoiding or have just recovered from avoiding").

**So ARS's arrangement — flat trim × a gain derived from a speed-scaled preview — is the dominant pattern, not an oddity.**

**The one real derivation, and it is about the preview:** ForzaETH's stability criterion is that the guidance loop's natural frequency `ω_n = √2·v/L_d` (with `τ = L_d/v`) must stay below half the vehicle-plus-**delay** bandwidth.
As `L_d` grows, `ω_n` falls — so **`L_d` must grow with speed** to hold the loop inside the bandwidth.
That is the proper justification for a speed-affine preview: not comfort, but delay margin.

**Four sources name the hazard of scheduling at all.**
Game AI Pro ch.40: "if the coupling is too great, a hidden positive feedback loop can be set up, resulting in instability".
Ziggy Racer hit exactly that: feeding the *planner's own* curvature into the steering feedforward "created a planner ↔ tracker limit cycle … the two rang together into a bang-bang steering oscillation."
**Never close the loop through a signal the loop itself produced.**

## The reference vector: body forward dominates; ARS's velocity reference is near-unique

| Reference | Count | Who |
|---|---|---|
| **Body forward / heading** | 13 | TORCS berniw/bt/tutorial, Unreal RacingAI, Unity CarAIControl + AIVehicleRoutingBuddy, Unreal TrafficAI, TUM example PP (body frame), ForzaETH MAP **shipped code**, ForzaETH PP, Game AI Pro ch.39, and others |
| **Velocity vector** | 3 | TUM countersteer (`atan2(vy, max(1,vx))`), TUM ESC (`beta = atan2(vy, vx)`), Ziggy Racer (sideslip catch) |
| **Track tangent** | 2 | TORCS bt `speedangle` (**throttle gate only**), TORCS SCR `angle` sensor |

**Load-bearing consequences:**

- **The velocity-referenced aim error is genuinely rare, and where it appears it is a stability layer, not a path-following error.** ARS uses it as the primary error — **nobody in the corpus does that**, so this design has essentially no precedent to lean on, in either direction.
  Not a bug: an original choice whose behaviour ARS has to establish empirically.
  Note what it buys and what it does not: referencing **velocity** rather than the nose makes the error a pure *course* measure (which is why it needs no separate heading term), but it still says nothing about slip angle — ARS keeps that in the separate countersteer blend, and so does the field.
- **Do not cite ForzaETH/MAP code as precedent for it.** The MAP ICRA paper specifies the velocity vector; ForzaETH's stack paper and its shipped code use heading. The same group contradicts itself across its own publications.
- **The track tangent is never the steering error** in any source — it appears only on throttle gates. ARS's `speedangle`-style velocity-vs-tangent quantity is, in TORCS bt, a *throttle* input: the same primitive on a different channel.

## Look-ahead: speed-affine nearly everywhere, grip nowhere

16+ sources scale preview with speed; one with **track width** (SuperTuxKart's corridor search); one with **driver skill** (simplix).
**Not one ties preview to grip or friction** — including stacks that have a full Pacejka model or per-wheel slip estimates available.

**The two criteria are different things, and no source resolves them.**
The ForzaETH derivation argues against grip: `ω_n = √2·v/L_d` is checked against a bandwidth that **does not contain grip**, so scaling `L_d` by grip moves `ω_n` without moving the constraint — but that is a *stability* criterion, and low grip genuinely does need an earlier turn-in for the same curvature, a *path-tracking* criterion.
History: the grip trend was found to drive corner cutting, which is why the full `1/grip` became a compressed manual `grip→leadScale` — an era the ranked list shows withdrawn, since the live steer reference carries no grip at all.
If a grip-scaled preview is ever proposed again, settle it by A/B'ing `speed/grip` against a speed-only preview, measured on cross-track RMS and steer-reversal count rather than by feel.

## Treat the preview as a state, not a memoryless function

TORCS `bt` and `damned` both floor the preview against `oldlookahead − v·dt` — a **rate limit on the preview itself**, described as preventing "snap back" of lookahead on harsh braking.
It is the only such guard in the corpus, and it matters because it makes the preview a state with dynamics.

**This is the field's answer to a hazard ARS had**: the lane systems are a **hard switch** (outside → inside, no lerp) and the chain's answer could step, which stepped the **lane error**, and so the command.
The old D-term amplification is gone; the step now reaches the wheel as a pursuit-plus-damper jump, ramped off by the slew rate at a fixed °/s rather than spiking in a single frame.
**Rate-limiting the lane at the source was the better fix, and it is built and driver-verified**: the lane mover carries the carried lane onto the chain's request at 26 m/s^2 with a 24 m/s cap, which removes the step from the P term instead of relying on the downstream slew to hide it, and it survives a slew-rate retune (ranked list item 3).

## Anti-windup is mandatory when the actuator saturates — and ARS's steering saturates

Only **five** surveyed sources carry an integral term, and **four of them guard it**.
**ARS has none today** — the live chain is P and D only — so read this as the prerequisite for turning an I on, not as a description of the code.
(The abandoned PID law's integrator *was* unguarded: unbounded, integrating unconditionally, while `ApplySteerLimits` caps the command to a small allowance at speed — so enabling it wound the integrator against a cap it could not overcome.)

Documented remedies, all cheap:
- **clamp** the integral (Ziggy: `cte_int` ±3; TUM: ±1325 on both PIDs);
- **suppress integration while the actuator is saturated** (Ziggy's steer-clip anti-windup bleeds the integrator when the wheel is pinned at full lock and still winding);
- Game AI Pro ch.40's third remedy: a **leaky rolling integral** — each frame reduce the integral by `(1−T)%` and add `T%` of the current error — which needs no saturation signal and self-heals.

ARS already has `_steerLimitedThisFrame` for the saturation gate.

## Damping belongs on yaw rate, not on a D term over the path error

**Almost every implementation that needed damping put it on yaw rate or curvature, not on a D term:**
- TORCS berniw/bt (a yaw-rate error term, gain 0.1);
- Ziggy (a *filtered* yaw-rate comparison, `r_des = v·κ`);
- TUM PP and Eelis03 (low-pass curvature filters);
- Unity CarAIControl (a yaw-rate speed-caution term);
- NwliZz (a yaw-acceleration stabiliser).

Game AI Pro ch.40 warns about the alternative in the same breath as recommending it: "If the input data (R) is noisy, the derivative term can fluctuate in an undesirable manner… **this is effectively adding another integrator.**"

**The bullets below were acted on *inside the abandoned PID law* — this block is the record of a rejected design and of what it taught, not a description of the code (see the STATUS banner):**

- Our D differentiated the *aim error* — the nose-to-aim measure (body forward since `6f4dc45`), a raw per-tick divide — so it was noisy; the field's answer is not to smooth it but to **read yaw rate directly**, and that is what the abandoned law did (`GET_ENTITY_ROTATION_VELOCITY.Z`, a native read), so the filter and the numerical derivative were both gone.
  USR's cheapest trick folds the damping into the aim *bearing* instead of adding a term: `targetAngle − (yaw + yaw_rate/15)`.
- **Reading yaw rate directly does not drop the speed mis-scheduling**: contamination and scheduling are two separate defects needing two separate fixes — with a fixed damping gain the loop over-damps at speed and under-damps at walking pace, so the schedule is the second fix.
- The dead law's correction referenced the rotation the *command* asked for, not zero — against raw `ω` the term would reduce steer permanently in every corner (the zero-reference toll above).
- **The `2L/d` form rather than `1/speed` is deliberate**: the aim distance already has a floor, so the gain self-limits at low speed instead of running away at rest — which matters for the stuck-recovery crawl.
- **A derived gain is a floor, not a target** — the ideal-model cancellation point was weaker than what the car actually needed, and the driver's confirmed value beat it: treat any derived gain here as a lower bound to start above, and **do not "correct" a working value down to a computed one**.
- **The dead law's D needed a gate only because the aim error's derivative carried the aim point's own swing**; any steer-vs-yaw gate leaks exactly in the countersteer case (steer and yaw disagree during a slide) and starves a corrective `I` as well (the same gate zeroed it).
- **Driver-observed, and structural rather than tuning: a high `SteerD` read as a brief damping burst at the *end* of the steer** — full-aggression turn-in, sharply damped arrival, the car steers at full aggression and is only damped in its final moments (the driver's stated preference).
  The correction kills its own trigger — paying back the excess lowers `ω`, which removes the excess, which removes the correction — so the **higher** the gain, the **shorter** the burst.
  **Do not "fix" the burstiness by clamping the correction — the brevity is the mechanism working, not a defect.**
- **The correction's reference can be the raw geometric command or the post-limit pre-slew one (`_limitedSteerDeg`, captured at the end of `ApplySteerLimits` and read one frame later because the limiter runs after `ComputeSteering`)**, and in normal driving the two are nearly the same number — verified.
  The live ceiling is vanilla's **grip-blind** speed curve (~20° at 41 mph and ~15° at 60 mph on a 40° lock) while the pure-pursuit command in a normal corner sits far below it, so the limiter **rarely clamps**; the variant referencing the post-limit steer was built and driven, and its behaviour was very similar to the raw reference.
  The earlier prediction that "D goes blind at speed because the limiter prevents the car reaching the wanted yaw" was therefore largely untested by normal driving — **the blindness needs the limiter actually biting**.
  The TRlat-form ceiling this replaced *did* scale with grip, so on a low-grip car its ceiling was the same order as the command, and that divergence case is gone; **the remaining divergence case is the abeam-aim-point transient — test there, not on a grippy car**.
  Reverting to the raw reference is one line (substitute `pSteer` in the yaw-rate computation); **the veto tests `pSteer` either way, deliberately**, so the reference is the only changed variable.
- **The veto must CLAMP at centre, not drop — dropping made D non-monotonic, and that is why `SteerD` 1.0 felt the same as 0.5** (found by the driver, then fixed).
  The drop form discarded any correction that would take the command past zero, so a larger `SteerD` asked for larger cuts, crossed the threshold more often, and therefore did **less** work — the dose response was linear, then a cliff to zero, and any meaningful overrotation put every setting past the cliff: the two settings were indistinguishable.
  The final form clamped instead, so more gain meant more damping, saturating at wheels-straight.
  **The ceiling is structural, not a tuning limit**: the most such a term can ever do is *neutralise* the lock, because it may never steer the other way — the price of unwind-only — and above saturation every gain value looks identical, so **tune in the mid-range, never at the extremes**; the clamp also covers `pSteer == 0` exactly, where a sign comparison cannot fire.
  (The additive half was vetoed *by choice* — a direction decision, not a guard against a contaminated input.)
  **General lesson worth keeping: a "never exceed" rule must clamp, not veto — a veto turns a monotonic dial into a cliff and inverts the top of its range.**

## Course error and slip angle are different states — and which the loop sees depends on its reference vector

**Every survey claim here is about the field, and stays true; each "ARS" contrast below is the live chain as written.**
The **restored chain's course error is velocity-referenced** (course-over-ground) — the *blind* one below — while the **nose-referenced variant was the abandoned PID law's**, and it was that law that was A/B'd and dropped in favour of velocity before the whole thing was abandoned in turn.
Both are kept because the difference between them is the durable insight.
With `θ` the direction to the aim point, `ψ` the heading, `v` the velocity heading and `SlideAngle = ψ − v`:

| reference | error | on a slide |
|---|---|---|
| **velocity** (course-over-ground) | `e_vel = θ − v` | a **course** error only. For a pure body rotation with the course intact, `e_vel = 0` — the car reads as *perfectly tracked while it is sideways*, and only the blend countersteers. |
| **body forward** (textbook pure pursuit) | `e_nose = θ − ψ` | the slide **directly**, since `e_nose = e_vel − SlideAngle`. A slide *subtracts* from the error, so the law **countersteers on its own** — the right sign, but see the magnitude trap below. |

- A car pointed 20° off while tracking straight produces a course error, and **either** reference corrects it by steering. Ordinary.
- A car **sliding with its velocity vector still pointing at the aim point**: the velocity reference reads zero error and does nothing; the nose reference reads the full slip as error and counters it. That is *why* 13 of ~20 surveyed implementations are nose-referenced, and why nose-referenced controllers so often need no separate slide term at all.

**Neither reference *reads* slip — `e_nose` merely contains it.**

**ARS runs two layers, and how they relate depends entirely on which reference is selected:**

- The live course-error P chain in `ComputeSteering` → the line, from whichever reference is active.
- The **slide countersteer blend** → **slip**, via `VehicleData.SlideAngle`, ramping in against TRlat and **overriding** the path command at full priority — the only consumer of slip in the entire steering path.

With the **velocity** reference the two layers are **complementary** — course vs slip, the field's shape.

### The magnitude trap: the sign is right, the gain is ~5× too small

**Do not conclude from the table above that a slip-containing error gives you countersteer worth having.**
The error passes through the pure-pursuit gain, so a genuine slide yields opposite lock of invisible magnitude — right sign, well inside the slew and the limiter; the velocity reference has the same gain and the *wrong* sign (in a developed slide it commands *into* the corner).

**And in a corner the nose-referenced error does not even reach zero.**
The aim point sits at the chord angle off the tangent while the nose sits at the slip angle β, so `e_nose = θ − ψ` stays positive until the slide exceeds the chord angle.
Below that the nose reference merely **unwinds** steer as the body rotates — self-reinforcing: less steer means less yaw, a smaller β, and `|SlideAngle|` stays under the blend's ramp onset, leaving the blend contributing nothing.

**So the blend is not redundant with the nose reference — it is the same signal at roughly 5× the gain**, and that is why the reference choice barely affected slide recovery: at full ramp the blend *replaces* the command outright.
**The reference choice is a line-tracking decision, not a slide-handling one.**
Making the path term a real slide participant would need the slip term entering with a gain near 1, not routed through `2L/ld`.
**ARS has no understeer detector or sideslip model anywhere** — the blend's bare `|SlideAngle|` magnitude gate is the only slip consumer in the steering path, so nothing "classifies" a car as understeering; it merely fails to trip the gate.

**The countersteer magnitude has a physical neutral value, and it is 1.**
A steer angle equal to the slip angle points the front wheels along the **velocity vector**, which removes the front tyres' contribution to the rotation — that *neutralises* a slide.
Any multiple above 1 points the wheels *past* the velocity vector and actively rotates the nose back — a deliberate over-correction rather than a stabilisation.
ARS *exposed* it as the `SlideCountersteer` knob for exactly this reason — a taste dial around a derived value, not an arbitrary gain — **but that knob is now gone**: the blend answers a share of the slide angle (`CountersteerShare`), and the limiter raises that side's limit to `min(|SlideAngle| × blend weight, lock)`, so the countersteer can reach the velocity vector but not pass it.
Two bounds agree with the physics: the share's top at 1, and the grant at the slide angle.

**Driven: 1 is neutral on paper and too much in the car — the shipped share is 0.5.**
Countersteer as wide as the slide (33° of wheel for 33° of slide) is the exact geometry, and it is also the state where the correction finishes level and the released tyre throws the car back the other way: **snap oversteer**, a full reverse rotation with no warning.
Half the slide still holds the car and still recovers it; the process is simply slower, which is the trade taken deliberately.
A higher share buys that speed back, and 1.0 is past the edge — anything between the two is a comfort dial, not a new law.
The share is a **ceiling only**: the ramp that earns it is untouched, so the blend still engages at the same slip and still reaches its full weight on the same nodes. Scaling the ramp instead (which the first cut did) softens the transition rather than lowering the reach, and that is not the same change.

That is TUM's architecture — a path-tracking controller with a slip-gated countersteer layer on top (`δ += k·(α_f − α_r)`, gated on `|α_r| > |α_f|`).
**The survey mis-characterised ARS as folding the slide into the path error; it does not.**
The two terms are not competing for the same job, so their interaction is a *priority* choice (save the car first) — the honest justification for the blend getting the last word.
Two consequences to keep in mind: while the blend holds full priority, **course tracking is suspended**; and a slide that never develops enough slip to trip the ramp is invisible to the blend, so only the PD chain's indirect response to it exists.

**At full priority the pedals are capped, not cut.** `CountersteerPedalCap` 0.5 limits **both** throttle and brake while `IsFullCountersteer` holds, and the plan keeps its own direction up to that cap: the tyre is already spending its grip diagonally, so neither a full brake nor a full throttle belongs in that state, but coasting at 0.05 was the other extreme and left the car unable to keep the wheels rolling or to settle its speed.

## Slides: the field mostly does nothing; ARS is ahead of it

**The survey's strongest negative result.** Of ~20 implementations, **17 do nothing about slides at all**; only **three** control them:
TUM's countersteer — the only real published law, `δ += k·(α_f − α_r)` gated on `|α_r| > |α_f|` — plus Ziggy's yaw-rate + sideslip detectors and USR's `getAvoidSteer`.
The rest avoid the problem by giving the AI extra grip (Assetto Corsa's 20% overhead "so the AI's weird and twitchy inputs don't make them spin", AMS2, rF2's steering-lock multiplier), deleting the slide kinematically (Unreal TrafficAI rewrites heading from wheelbase geometry), or penalising it in a reward (GT Sophy).

**Correction to an earlier claim in this file.** The oversteer-countersteer / understeer-slow-down split was recorded as an "independent confirmation" of ARS's design; it is one project (Ziggy), not a norm.
What *is* common is the **understeer half**: three sources measure sideslip and respond by **slowing down rather than steering** — the ESC idiom with the countersteer half simply absent.
So ARS's asymmetry (oversteer gets the wheel, understeer gets the speed pipeline) is a minority position that one strong source shares.

**ARS's steering limit no longer has a countersteer exemption — and that closes the gap this section used to record.**
USR reaches the intent by making the clamp *wider* under rear skid; ARS now does the same thing explicitly, raising only that side's limit (and only up to the slide angle).
Two independent codebases converge on *the limit must widen for the correction that saves you, without disappearing for it*.

**ARS's live configuration is therefore the field's shape, not a deviation**: a course controller for the line, plus a slip-gated countersteer layer with priority over it, plus a limiter that widens on the countersteer side instead of exempting it.
The piece still missing relative to TUM: TUM reads **per-wheel slip angles** (`α_f`, `α_r`) and gates on the *rear* exceeding the *front*, while ARS reads a single body-level `SlideAngle` — ARS cannot distinguish front-axle from rear-axle slip, and its gate is on slip magnitude alone rather than on the front/rear imbalance that actually defines oversteer.

## Speed-dependent steering limits are normal — but check how hard they bite

TORCS's own **human** driver ships a "steer speed sensitivity… the factor of reduction of steering values when the speed grows. the default value is 1.0 which corresponds to **half the steering at 360 km/h**" — a mild, feel-oriented reduction on the input path.
Structurally the same idea as ARS's `ApplySteerLimits` but far gentler than ARS's allowance.

Ziggy's headline diagnosis is the cautionary tale: it burned a long campaign trying steering-law levers against a corner where the wheel was **already correctly saturated** — "you cannot fix a saturation by asking for more of a resource that is exhausted." The fix was reducing demand (a speed cut in the approach), not raising authority.

**Diagnostic for ARS:** if cars run wide in a corner *identically at every gain*, the limiter's allowance is the binding constraint, not the controller — measure commanded steer against the allowance before touching gains.

**On the slew rate — read the code for its value; the design question is whether it should exist at all.**
Exactly one shipped racing game publishes a steering rate limit, and its stated reason is **comfort** ("to reduce shaking of karts", because "otherwise the karts counter-steer to fast") — while Eelis03 documents a **0.6 rad/s (34°/s)** actuator limit *causing* a limit cycle.
So 34°/s is the number to measure any rate limit against, and a rate limit can be the oscillation *source* rather than a cure; ARS's rate should be justified on stability grounds, not comfort.
Note also that ARS's **faster countersteer path was a guess, never validated in game, and has since been removed** — the slew is a single rate, so a slide correction now arrives no faster than any other steer.

## Run the integral in distance, not in time

"The integrator AND the derivative gain need to be scaled by the speed. Normally the PID updates every millisecond but what you really want is to update the PID as a function of distance travelled or rate of speed."
Integrating `error × dt` makes the integral speed-dependent for free; `error × ds` removes that.
ARS already adopted time-sampled discipline for the Gs calculator; the steering loop has the same argument in distance terms — **moot today, since there is no integral to sample** (see the no-integrator bullet above), and live again the moment one is added.
The *derivative* half is already half-answered: the live damper's gain carries the reciprocal speed schedule above its reference speed (see the gain paragraph above).

## Reading constants out of this literature

- **Code beats papers.** Three documented traps: TUM's paper and code disagree on the **sign** of the countersteer (porting the paper steers *into* the slide); TUM's code defaults and shipped YAML disagree in the dangerous direction (`max_brake_pressure` is declared **twice in one file**); and one repo's README contradicts its own YAML.
- **Cite the right publication.** MAP's paper and MAP's code disagree on the reference vector (above).
- **Cross-track error as the *primary* steering signal appears in exactly one shipped default in the whole corpus** (simplix), and there its authority is capped at **15%** of total steer.
- TORCS berniw caps its lateral-error correction at 0.1 for the same reason.
- **External support for ARS's split**: the lane systems decide *where* to aim, the pursuit law carries the authority, lateral error is a bounded trim.

## The lane pursuit is ungated, and a straight-line offset converges

`ComputeSteering` calls the pursuit whenever `TryGetSteerContext` passes (`Racer.cs:496`, `:624-633`): the old `|targetLane|` guard went with the error-based lane-bias era (`dcef913`), so a straight or gentle sweep — where the lean has no answer — still steers on a real bearing rather than on nothing.

**What the aim is decides the rest.**
The aim offset is the carried lane (`aimPointOffset = _carriedLane`), and the chain's answer is a **request** that the lane mover carries it onto: the chain is seeded with the car's own offset (`steerRefOffset`), so with no corner line, no avoidance and no lean answer the request **is** that offset — a straight-line standing offset is **held, not converged onto the centre** (the Lane systems base rule).
`steerRefOffset` is read in three places: that seed, the optional Gs-aware preview (zero at steady state), and the lane mover's interval clamp, which takes it as a parameter rather than reading the centre deviation.
**The gate removal still has not had its own drive.**

## Where ARS sits, and the ranked list

ARS runs pure pursuit on the aim point's bearing (unity `steerKP`), a side-by-side heading correction, a yaw-rate damper referenced to that aim's own curvature, and a limit that widens on the countersteer side; no integral, no curvature feedforward.
That is the field's feedforward-plus-feedback *intent* reached by a different route — the pursuit's position term does the standing-error job the curvature feedforward usually does.
Two unusual choices: the bearing is measured from the **velocity** vector (the loop is slide-blind by construction), and nothing but the damper's divisor is grip-scheduled.

Ranked by expected payoff:

1. **The yaw damper's reference and gain are both settled and live** — repaired and reverted twice (`a9603c4`), and the gain's cap now driver-verified (`5bb94bd`). The lever stays here if the standing-offset toll ever returns.
   The signal half stays done: the live damper reads measured yaw rate directly, with no numerical differentiation and no aim-point swing, as `(yaw − target) × gain`.
   The reference is **settled, and no longer zero**: it targets the yaw the **aim point** requires (`SteerDampingAimReference`), the track form deleted with its switch (see the rejected-design note above), centring driver-confirmed with it.
   Damping against **zero** charged a standing lane offset in every corner (the zero-reference toll above); the drive that preferred zero predates the course error's deletion, so it was confounded.
   The gain side's speed tracking returned uncapped (`97cbc34`) and is now **capped** — the dial stays the gain below its speed gate — which the drive confirms improved the system (`5bb94bd`, driver-verified).
   Grip normalisation (`SteerDampingScale`) stays retired (`a9603c4`); the grip divisor does that job.
   If the toll ever shows up again in the lane line, the reference is the lever that removes it.
2. **Clamp and gate an integral** before I is ever turned up — **a precondition, not a task**: there is no integral to clamp today. Prefer clamp + saturation gate, or Game AI Pro ch.40's leaky rolling integral.
3. ~~Rate-limit the carried lane~~ — **BUILT and driver-verified (`5e9426a`, built `0f825b8`, `8e8884e`): the lane mover.** The lane reads right on a drive — smooth, and the flapping this exists for is gone. The chain's answer is a request and the carried lane is carried onto it at 26 m/s^2 with a 24 m/s cap, so a request that flips inside half a second moves the lane 3.4 m instead of stepping the whole way. The numbers come from the line's own motion, not from feel: two safe bounds over `2R·leadAngle` asks about 8 m/s and 21 m/s^2 at a supercar's apex speed and 6.4 and 13.6 at a vanilla car's, near enough the same at every radius because a car takes each corner at its own limit — so the cap must sit above the line's own rate and the acceleration is the only lunge knob. At 14 the aim tracks the line to 0.27 m and 0.06 m respectively; at 6 it lagged 2.9 m, over a third of the traverse, which would have read as cars refusing the inside line. The request's own rate feeds forward, the aim is clamped into the interval between the car's own lane and the request, and the green re-seeds it. **The rival walls were then taken off the aim entirely (`3d13279`, driver-observed: the avoidance reads smoother)** — a bound laid on the aim is placed in one tick rather than carried onto, which is the step the mover exists to remove — so the walls bound the request alone and a switch to the legal side of a rival costs the arrival law's time (0.38 s for a metre) instead of arriving at once. Its own log line is the instrument if it is ever retuned.
4. ~~A/B the grip-scaled preview against a speed-only one~~ — **WITHDRAWN: the premise does not exist.**
   Grip enters neither the steer reference nor the aim (see the preview properties above), so there is no grip-scaled preview to A/B and the constant-time arithmetic this item rested on described a form the code never had.
   What is still testable is the **band**, not the grip: outside it the preview time stretches below the floor and shortens above the cap, so the live question is whether those two bounds should move.
5. **Add the curvature feedforward (`wheelbase × curvature`)** if a standing aim error shows up as a permanent lane offset.
   **Partly answered already**: where a lane source is engaged, the live cross-track pursuit nulls an offset, because it acts on position rather than course — the job the feedforward exists to do is currently done by the lane term.
   Revisit only if the lane term's authority is deliberately reduced, or if a *straight-line* standing offset ever matters — a straight-line offset is **held** today (see the ungated-pursuit section).
6. **Sharpen the slide layer's input.** The blend is live again, so this is no longer "reinstate slide handling" but "give it a better signal" — see the Slides section: TUM gates on the **front/rear slip-angle imbalance** over `|α_r| > |α_f|` (the actual definition of oversteer) while ARS gates on the magnitude of a single body-level `SlideAngle` against TRlat.
   Per-wheel slip is available in the handling struct's wheel data if it is worth the plumbing.
7. Consider a distance-based integral, a lookahead fallback (PX4: target the closest point on the path when the lateral error exceeds the lookahead) and error-adaptive aggression (JPL/Kelly: let the gain grow with lateral error).
   The last two are honest upgrades to the **live** off-track recovery term — a severity-scaled push-back once past the safe edge, not a lookahead fallback.

## The live pipeline, the lane laws, the speed plan and the corner lifecycle

Moved whole out of the project `AGENTS.md` to keep the instruction chain inside its cap. The order and the invariants stay in the parent; the systems live here.

## Per-frame pipeline (`Racer`)
1. `UpdateTrackPosition` (`Racer.cs:2809`) — local node search with a global rescan fallback, time-based lookaheads, route radii, lap ticks.
2. `ComputeTargetSpeed` (`Racer.cs:1629`) — builds the intended speed; see the Speed pipeline.
3. `ComputeSteering` (`Racer.cs:522`) — resolves the lane into a target offset and sums the authority chain into `Control.SteerDegrees` (the aim point is the pursuit's input — see the chain analysis above).
4. `ApplySteerLimits` (`Racer.cs` `ApplySteerLimits`) — the two side limits closed by a single `Clamp`, and it runs **last among the writers of the steering request**.
   The base is the peak-slip cap, the corner geometry being only a fallback when the peak reads unusable, eased up by the maneuvering ramp below its own end speed (what the peak reads is the saturation bullet above).
   The slide's `Math.Max` loosening of the base is gone — retired by decision, then deleted (`ebb3178`) — and the only thing the slide still opens is the side answering a rotation, through the countersteer allowance.
   **The retired yaw-usage governor is the cautionary tale**: it scaled the ceiling by the car's own yaw rate, so a car not yet rotating got about a fifth of the law — least authority at exactly the moment it needed most — and under understeer, where yaw is low *because* the steering is not working, it took steering away from the car it exists to rescue.
   Yaw rate also cannot tell a clean turn-in from a stepped-out rear, and the slide's sign can.
   The **slip-balance knee is deleted** (`d434b86`); `d434b86` added the second governor, and `cd6efea` removed it.
   **A steering writer added below it is a silent bypass** — the bug class this shape removes.
4b. `UpdateLaneMover` (`Racer.cs` `UpdateLaneMover`) — carries the aim onto the lane request under one law: `LaneMoverAccelMps2 26`
   and `LaneMoverSpeedCapMps 24`, both **halved while the reason is `OutsideLine`** (positioning outwards is a choice made with room;
   the apex side is where the car is already turning in), an arrival term `sqrt(2·a·gap)` that decelerates onto the request, the
   request's own rate as feed-forward, a rate reset whenever the desired rate reverses sign, and `dt` capped at
   `LaneMoverMaxStepSeconds 0.1` so a stalled tick cannot integrate a whole gap.
   **Every lane reason passes through it** — `CurveLean`, `OutsideLine`, `ApexLine`, `CommitCard`, `Avoidance`, `RecoveryEdge` — so
   inertia is a property of the mover, not of any producer. The two escapes are `_laneSnap` (one tick, `SnapToTrack` and the per-car
   reset) and `ClampLaneToWalls`, which bounds the **request** only while the aim still rides the rate.
   **The guard lives on the objective, not on the aim point (driver, 2026-10-24, build 893).** `UpdateLaneMover` takes `laneEdge`
(`roadWide - carHalfWidth`) and clamps `_laneRequest` to it before anything else, so a lane can never name the track's very edge - only
where the car's CENTRE may sit. Every path that writes the aim writes a guarded value: the snap writes the clamped request, the
cross-through teleport writes `Clamp(steerRefOffset)`, and the inside-mode guard's substitution is clamped too. The `aimPointOffset`
clamp against `drivableEdge` **went too** (build 894): a bound on the point anchors the pursuit at a maximum distance and pulls it
back inside, when all that is wanted is for the lane to stop there. The objective's own clamp is the only bound, so the preview
correction is free to lead the point past the aim without moving the aim itself.
Before this the outside line asked for `+-roadWide` in its plateau, which `drivableEdge` then trimmed by half a car every tick - the
same unsatisfiable-demand defect the audit found in the rival walls.

**The lean carries no bound of its own (driver, 2026-10-24, build 891).** `lane = steerRefOffset - cornerDir * pull` and nothing else:
the `roadWide` clamp it used to end with was the aim point's own clamp repeated 700 lines early, so the only bound left on the lean is
`aimPointOffset` against `drivableEdge`, and `ClampLaneToWalls` on the request. Its `roadWide` parameter went with it.

**The brake learner, tuned by driving (driver, 2026-10-24, build 922).** `BrakeEntryDeadbandMps` **2 mph**,
`BrakeEntryExcessMps` **5 mph**, `BrakeEntryFactorStep` **0.2** per five miles an hour over, `BrakeGoodArrivalStep` **0.025** flat,
`BrakeMinFactor 0.5`, and the ceiling moved from 1.5 to **1.0** - no step above the nominal brake point. Both steps are absolute in the
factor's own units, never a percentage. By overshoot: 2 mph nothing, 5 mph -0.12, 10 mph -0.32 (floor), 20 mph -0.72 (floor). The car under
test took four laps to settle at the gentler settings; this is one lap for anything past ~10 mph. Cost: a single overshoot saturates the
floor and every clean lap buys back only 0.025, so a corner can sit at 0.5 for ten laps after one mistake - the symptom is brakes far too
early on a corner the AI got wrong once.

**The brake learner is calibrated and VERIFIED (driver, 2026-10-24, build 922).** One lap to learn a corner - a single pass through
the apex - and once settled the factor moves at most **+/-0.02 per lap**. The driver's verdict: *"este balance esta bien"*. That closes
the thread; the numbers in the paragraph above are the calibrated set and should not be nudged without a fresh on-track reason. The
two earlier tunings are kept in the log because each failing one says what the next needed: 0.2/20 mph took four laps to settle, 0.05/20
mph was a trim that could not correct at all.

**The clock cushion was swept and removed (driver, 2026-10-24, build 926).** `LineClockBoostMph` was tried at +5, +2 and -2 mph and
the verdict was *"no hay mejoria consistente"* - the whole sweep moves the blend 6.2 m and 0.18 s at 120 km/h, which is inside the noise of
how a car happens to enter. The constant and its term are deleted rather than left at zero; `clock=` stays in the `Corner line` log, which
is what let the sweep be read at all. The lesson: **the entry-speed latch is the mando that matters; the cushion around it is not a mando.**
If the crossing still reads as too quick, the knob is `IdealLineBlendStartSeconds`, whose steps are whole seconds.

**A corner's entry and exit are floored at one track width from its apex (driver, 2026-10-24, build 932).** In `AddCornerRegion` the
green entry and the red exit are walked outward until they are at least `2 x TrackHalfWidth` TRACK NODES from the apex (nodes are
metres here), after the region clamp and before the corner is built. A corner whose entry sat a few metres from its own apex describes a
bend the car is already inside, and every distance the manoeuvre later measures against that apex inherited the shrink. This is the
corner table's rule, not the blend's - an earlier attempt to floor the BLEND in metres was reverted (builds 930/931) because the wrong
end of the chain was being constrained. **VERIFIED on F8 (driver): the cars take the corners better - there is more corner to
manoeuvre in.** Note that it binds broadly, not rarely: the logged walks returned 6, 9, 11, 13, 19, 64 list positions, so most F8 corners
had their entry pushed outward. That is the point - those entries were the strange apexes.

**The clock leans on the car's own braking (driver, 2026-10-24, build 927).** `LineClockBrakeMphPerG 2` adds two miles an hour of clock per g
(build 928, doubled from 1), clamped to `LineClockBrakeCapMph 10` (raised from 1 at build 929 to see the effect uncapped), measured from `AlongTrackSpeed`'s fall each tick. A car shedding speed
reaches the apex later than the cached entry speed says, so the line is allowed to slow with it - by a mile an hour at most, which is
the whole point of the cap. Reading fixed up front: the driver said 0.1 per g, and at 0.1 mph/g the cap could never bind, so the law is
0.1 mph per 0.1 g. `Corner line` logs the effective clock and `br=` in g.

**The entry-latched clock still depends on how the car enters (driver, 2026-10-24, build 917).** One frozen number cannot be right for
two different approaches: a car that enters slow and accelerates keeps a low clock, so `2 × v_entry` metres arrive almost immediately
once it is up to speed; a car that enters fast and brakes hard keeps a high clock, so the same 2 s of clock stretch over more real time
than intended. The latch point is the window (300 m), which is a *distance* and therefore already late or early depending on speed.

**The candidate fix, not applied**: latch at the moment the blend OPENS rather than at the window, with the trigger itself in live
speed - `metresToApex <= IdealLineBlendStartSeconds * Math.Max(speedMps, 1f)` - and freeze the clock at that tick. The blend then always
begins ~2 s of actual driving before the apex, whatever the speed, and freezing stops it reopening under braking. Cost: the trigger
moves with speed, so the opening distance is not identical between laps - only the time is.

**`cornerDir * x` is not a side-neutral coordinate (driver, 2026-10-24, build 915).** `demandInside` compared `cornerDir * offset` with
`cornerDir * ownLane`, and that product grows toward the OUTSIDE on a left-hand corner and toward the INSIDE on a right-hand one. So the
test could only ever read *inside* on lefts: `_apexLineInsideNode` and `_apexLineInsideCornerDir` were never set on a right, the inside
mode never armed there, and the whole guard chain was left-hand only. The driver found it by running the same circuit **backwards** - the
corners that misbehaved swapped sides, which is the signature of a sign asymmetry, not of a timing one. The side-neutral form is
`(demand - ownLane) * -cornerDir > 0`, because `-cornerDir` points at the apex side on either hand. Every other use of `cornerDir` is a
multiplication by a positive magnitude and is correct: `lane = -cornerDir * (laneEdge - gap)`,
`commitLane = target.NodeOffset + (-cornerDir) * gap`, `inward = -_apexLineInsideCornerDir`.

**Rule**: a comparison between two lane values must be made in a frame that does not depend on the corner's handedness. Multiplying
both sides by `cornerDir` does not neutralise the sides - it mirrors one and not the other.

**The last second is a commitment (driver, 2026-10-24, build 914).** Once `secondsToApex <= IdealLineBlendEndSeconds 1`, `linePhase`
is forced to 1 and `lineActive` to true whatever the blend managed: the target IS the inside extreme. It closes every way the line could
arrive short - a 2 s window at speed, a lean contesting the lane, a speed floor that dropped out under braking - and it is what the
magenta line could not do on F8, where it reached the centre and hovered without ever committing to the inside. `entryActive` still
gates the plateau and the exit; the commitment overrides only the final second.

**The adopted apex is drawn (build 913).** `ShowTrackAnalysis` now draws a white diagonal from the car to the **centre of the node**
of the corner the lane has adopted (`Racer.LineCornerNode`), at `ApexMarkHeight 1.6` so it flies above every lane line and chevron, with
the node number beside it. The lane's own geometry is gradual and hard to read; the identity is not - a diagonal that jumps to a distant
corner is the whole class of bug in one glance.

**The cross rule only fires on a reversal (build 910).** Comparing aim and demand against the CAR's lane alone teleported the aim
onto the car's lane every tick of an out-in: with the car mid-track, the demand is already inside while the aim still leads on the
outside of it, so the two straddled the centre and the line reached the middle and stopped - it could never cross to the inner half.
A teleport now also needs `gap * _laneRate < 0`, i.e. the request behind the aim's own motion - a genuine reversal, which is what the
rule exists for (a lean flipping handedness). Normal chasing, including crossing the centre, is left to the mover's own law.
`Lane cross` logs the event once a second.

**The adopted corner owns the line until its apex is passed (driver, 2026-10-24, build 909).** `NearestApexAhead` now keeps returning
`_lineCornerNode` while it is still ahead and no other corner is more than `LineApexStickyMeters 30` nearer. Without it, two apexes a
few metres apart - a chicane, a bend detected twice - swapped identity tick after tick, and every swap reset the blend: the magenta
line reverted to the outside mid-manoeuvre on the last two apexes of F8. The ratchet's `else`-clear is **gone**: it ran whenever the
blend window flickered, and that window contains the speed floor, which moves with mechanical grip and downforce - i.e. it flickered
exactly when the car was unsettled. The latch is released on adoption of a different corner (always from the outside, whatever its
handedness) and in `Launch`. **A line, once started, is never interrupted; only a passed apex ends it.**

**Two apexes closer together than the out-in window (F8, driver, 2026-10-24).** When the next apex is born already inside
`IdealLineBlendStartSeconds`, there is no room for an outside leg between the two corners and there is no point asking for one. The
adoption reset handles it without a special case: the latch goes to 0 and the same tick's `linePhase` immediately lifts it to the
value its own clock gives (0.8 at 1.2 s), so the demand crosses from the first apex's inside extreme to the second's remaining blend
instead of jumping outwards first. A pair like that should read as one continuous inside line across both apexes.

**Edge clearances and the per-car outside bias (driver, 2026-10-24, build 907).** The outside line now sits `LaneOutsideGapMeters
0.5` inside the guard and the lean sits `LaneInsideGapMeters 1.0` inside it, so neither names the very edge of the tarmac. On top of the
outside gap each car rolls its own pull-in of `0..LaneBiasMaxMeters 4` metres, **once per corner** (`_lineBiasNode`) and held until it
passes the apex, so a field does not stack itself on one line; the bound is floored at zero, so the bias can reach the centre of the road
and never cross it. The inside extreme of the apex line keeps the raw guard - it is the one place the car is meant to use the whole
width. The bias is cleared in `Launch`: a restart must not inherit the previous race's pull-in.

**The car's lane is measured in the car's own node frame (build 905, audit `deleg_7fee6020`).** `steerRefOffset` used to be
`SignedLaneOffset(Car.Position, steerRefPoint.Position, steerRefPoint.Direction)` against the steer reference, which sits up to 30 m
ahead: the car's position then picks up the sagitta of that chord, `L^2/2R` - 9 m on a 50 m corner, sign following the corner, so a
car dead centre read as being at the outer edge. Every same-side test in the mover (cross, overshoot, leash, the inside guard's hold
value) is made against this number. It is now measured against `CurrentTrackPoint`.

**Three pieces of latch state ended a race and could start the next one (build 905).** `_linePhaseNode/_linePhaseLatched/_linePhaseDir`
are cleared in `Launch` (they are keyed by node INDEX, which means nothing across tracks); `_apexLineInsideNode` is now cleared whenever
its apex is behind the car, outside the `ApexLine` branch - the clear inside that branch was unreachable, because the branch only runs
while the apex is still ahead, which is how lap 2 armed the guard before its own demand had gone inside; and the blend's armour is gated
on `inBlendWindow` (window, radius, apex ahead, time) and NOT on the speed floor, because that floor moves with mechanical grip and one
tick under it swept the ratchet away mid-corner.

**The lean names the inside; the curvature sets the approach rate (driver, 2026-10-24, build 899).** `TryComputeCurveLean` used to
return `steerRefOffset - cornerDir * pull` with `pull` up to 1 m: a RELATIVE target, so the line followed the car's own drift - a hug,
not a correction - and the sampled-node sign made it move erratically. Now it returns the inside LANE, `-cornerDir * laneEdge`
(`roadWide - carHalfWidth`), and publishes an **absolute approach speed**, `_leanApproachMps = 5 * exp(-max(R - 200, 0)/100)`, which the mover applies as
`min(cap, _leanApproachMps)` while the reason is `CurveLean`. The linear share it replaced was still too eager (a 499 m bend moved as
fast as a 250 m one at 9.6 m/s); an exponential in m/s is what the driver asked for. Speed by radius: 499 m 0.25 m/s, 400 m 0.68,
300 m 1.84, 250 m 3.03, 200 m and tighter 5.00 - the cap of 5 m/s is the mover's, not the car's. `CurveLeanPullMeters`,
`CurveLeanPullReportMeters` and `CurveLeanFullRateRadiusMeters` are gone.
Cost: the lean is no longer self-limiting - a bend inside 500 m now walks the car all the way to the inside lane, where before it
asked for one metre.

**The apex blend is a TIME law; the outside is a distance law (driver, 2026-10-24, build 896).** `IdealLineWindowMeters 300` still says
where the line begins to be asked for, because placing the car on the outer edge is done with room to spare and does not change with
speed. The outside-inside blend is `IdealLineBlendStartSeconds 2` to `IdealLineBlendEndSeconds 0.25` (three-to-one was tried first and was too early, build 897; the end moved
from 1 s to 0.25 s at build 928 so the manoeuvre has 1.75 s of clock and the inside is pinned only at the last quarter second), phase computed from
`metresToApex / _lineClockMps`, where `_lineClockMps` is **the speed the car carried when it entered the window**, latched once per
corner (`_lineClockNode`, build 917, driver's call). Three variants were tried and each failed for its own reason: the **live speed**
moved the blend opening with braking, traffic and throttle cuts, so the line behaved differently every lap; the **apex speed**
(`ApexSpeedWithDownforce`, build 916) was too slow - 2 s of a 40 km/h apex reached at 120 km/h is 22 m and 0.67 s of real time, a third
of the intended duration; and the **distance blend** before them arrived early when slow and late when fast. The entry speed is honest
about the approach and frozen for the corner. The inside extreme is now reached ONE SECOND before the apex instead of five metres. `Corner line` logs `t=` beside `d=`.
The clock runs on `speedMps`, which braking lengthens, so the phase is **ratcheted per apex** (`_linePhaseNode` /
`_linePhaseLatched`, build 898), the corner's direction latched with it (`_linePhaseDir`, build 902), and both **gated on the blend
itself** (build 903): outside `secondsToApex <= IdealLineBlendStartSeconds` the latch is cleared, because before the blend the line
is asking for the outside and there is nothing to ratchet - an armour latched 300 m out would follow the car into situations the
corner knows nothing about. Inside the blend the demand can only move toward the apex.

**The aim is leashed to the car** (`build 889`, `LaneTaperNearMeters 2`, `LaneTaperFarMeters 4`): the speed cap is full up to
   2 m from the car lane and falls linearly to **half** (`LaneTaperCapShare 0.5`) at 4 m, on top of the reason-based halving, and
   stays at half beyond. It slows the lane as it leaves the car; it never stops it, or the car could not be led anywhere at all
   (builds 889 stop, 895 half). Cap by distance, inward / outward: 2 m 24/12, 3 m 18/9, 3.5 m 15/7.5, 4 m and beyond 12/6.
   **The aim lives on the segment [car lane, demand]** - two violations, two rules. Crossing (aim and demand on opposite sides
   of the car lane) is the teleport above. **Overshoot** (same side, but the demand already inside the aim) sets `_laneOvershoot`:
   the mover's acceleration doubles - the speed cap is untouched, so it returns twice as promptly without ever moving faster than
   any other request - and the car stops following the lane, keeping its own (`aimLane = steerRefOffset`), because following an aim
   on its way back drives the car past the goal and back again. Build 887.
   **A request on the far side of the car is placed ON the car, rate zeroed** (`build 886`, `LaneCrossMinMeters 0.1`): the stretch
   between the aim and the car's own lane is ground the car already occupies, so sweeping it is invisible work that costs a full
   reversal — which is what made a curve changing handedness read as half a second of frozen lane. What remains visible, and keeps
   its own rate, is the outbound leg from the car onto the new side.
5. `ConvertSpeedToPedals` (`Racer.cs:1375`) — one `[-1,1]` combined input, split, then the **reason-cap compose** (also pre-slew) and the slew.
   **GridWait lives in three places** (`ComputeTargetSpeed`, `ProcessAI` handbrake pulses, `ComputeSteering` hold-straight) — keep them in sync.
6. `TranslateSteerToInput` (`Racer.cs:1609`) — slews steer toward target at `SteerSlewRate`, then remaps degrees → −1..1.

**The maneuver cards decide at the top of the core pass**, after `UpdateTrackPosition` and before `ComputeTargetSpeed` (`ProcessAI`).
The four tactical cards and the maneuver cleanups ride that pass on rival records the publish owns; the crowd card and the rival scan keep their own slower beats.
The pass serves a slice of the grid per frame, so a car's card rate falls as the grid grows.

## The steering authority chain (`ComputeSteering`)
**The aim point is the whole geometry**: the lane steer is pure pursuit — the bearing from the car's travel direction to the aim point (`PursuitSteerFromBearing` `Racer.cs`), with pure pursuit's factor of two carried in `PursuitGain`.
**The course error is deleted**: referenced to the road's direction at the lookahead node it over-commanded by roughly `lookahead/radius`, which *was* the corner cutting, and the bearing already carries that information.
**Off-track recovery is deleted**: off-track it and the lane call were the same law at the same point at gain 1 each, so it doubled the command rather than recovering anything distinct, and the lane pursuit alone recovers the car — driver-verified, and it can oscillate because the car keeps enough speed to overshoot.
Then rival repulsion **added** to the lane steer and never overwriting it (`Racer.cs:685`), the side-by-side heading match (**one-way** — only while the overlapping rival moves toward this car's side, `Racer.cs:707`; it is the heading difference between the two cars' forward vectors, `Racer.cs:733`), and the **yaw damper referenced to the aim point's required yaw** — the loop's only rate feedback, **load-bearing** (with it off the cars limit-cycle around centre; against *zero* it taxed every steady corner into a standing lateral offset — see the damper bullets above).
The repulsion term itself is now the pair's relative course angle taken on horizontal velocities, ramped by the daylight between the two surfaces — nothing at five metres of that gap, the whole angle when the sides touch — and its coming-at-my-side gate is what keeps the angle's sign pointing away rather than into the rival (`Racer.cs:603-640`).

**No term is gated** — `ApplySteerLimits` and the slew rate only cap the output; the pursuit's gating story is the ungated-pursuit section above.

**The slide batch hands the countersteer to a blend and halves the damper's whole term (`d5a643f`, re-cut in `7297050`, settled by the slide drive).**
The blend's weight ramps linearly from a fraction of the **static** `LateralTractionCurve` to full at a higher fraction of it — not the speed-scaled peak, and with no rear-led sign gate — so the slide is caught before the tyres' limit is gone and the countersteer reaches full authority inside the band.
The target answers the whole slide angle (`Racer.cs` `CountersteerShare`); the blend weight is the only modulator, and the limiter's countersteer allowance reads the same weight.
The weight is computed before the damper because the limiter's allowance reads it, and the old gate constant is renamed `SlidingFraction`, whose only consumer is now the damper's reference gate (`Racer.cs:589`).
The axle gate that used to guard that halving, and `UndersteerDegrees` with it, are deleted.
**One deliberate silence**: the recovery rejoin's throttle kick card (`cc469ee`) buys a rotation on purpose — the slide sharing the arm-time ask's sign is wanted, so the blend and `IsFullCountersteer` stand down for it while the card plays (`KickWantsSlide` `Racer.cs`) and the ordinary handover resumes at the fold. A slide on the opposite side is not carded and gets no silence.

## Lane systems — where on the track the car aims
Resolved in `ComputeSteering`, later overriding earlier, over a base of **the lane the car is already in**:
**curve lean** (`Racer.cs` `TryComputeCurveLean`, runs every tick and answers only where the node `speed × CurveLeanChordSeconds` ahead carries a curve radius under the gate (`Racer.cs:725`, `:726`, `:805`), and it answers a **relative pull** — the car's own lane plus up to a metre toward the inside, faded linearly from nothing at the gate to the full metre at 250 m (`:812`, `:814`) — rather than the edge, so the crossing stays alive while the bend lasts and the drivable bound is what ends it; below the hold threshold it reports no opinion, because the answer is then the car's own lane),
**corner line** (`Racer.cs` `ComputeCornerLane`, the distance-parameterised ideal line),
**avoidance** (`Racer.cs` `ComputeAvoidAheadLane`),
then **`ApplyRivalWalls`** (`Racer.cs`) clamps the result between track bounds and rival walls — walls that sit exactly at `±roadWide` when nothing constrains them (`Racer.cs:884`, `:922-923`, `:938`).

**The base is load-bearing**: zero is the *centre* lane, not the absence of an answer, so a chain seeded with zero steered every car back to the middle of the road wherever no source had an opinion — which is every straight (`dd484d3`).
The corner and avoidance sources still report silence as zero, and that is what lets the lean take over inside a corner the car has not asked to position for.

**The damper's countersteer ceiling is a slide-propensity dial, not just an amplitude (driver, 2026-10-23).** The damper
is the only term that reads **yaw rate**; the blend and the assist read **slip angle**, so they act after the car is
already crossed. That makes `DamperCountersteerMaxDeg` the car's resistance to *continuing* to rotate: at 10 it cuts the
rotation early and the slide barely opens, at 0 it can only cancel the command, the rotation keeps going, the slide
passes the assist's 10-degree onset and the assistance arrives at 10 degrees per second against a car that may be
rotating at 60 — late by construction, and only decisive if the slide persists. Lowering the ceiling therefore does not
only reduce the correction: **it increases the occasions**. It is also a personality lever before the personality system
exists — a tail-happy car is one with this knob low.

**The blend's band starts at the peak and ends past it: 1.0 to 2.5 of the live peak slip.** `SlideBlendStartFraction` 1.0 and
`SlideBlendFullFraction` 2.5.

**Both bands read `TRLateralAtSpeed`, the peak at the current speed (driver, 2026-10-23).** They used to read the authored
`Handling.LateralTractionCurve` so the response was the same at every speed, and that was wrong for the reason the engine
itself imposes: GTA scales the lateral curve by speed, so a tyre peaking at 11.6 degrees at 20 mph peaks at 3.7 at 200.
A fixed angle therefore measured a different part of the curve at every speed — at 200 mph the tyre was already at the
floor of its curve from four degrees while the blend did not commit until eleven. The band is now a fraction of the live
peak, so it narrows as the speed rises and the same slide fraction is answered at both ends. The throttle-kick gate reads
the same reference. The start is the peak itself, which is the physical boundary between gripping and
sliding — correcting at half the peak, as it does at 0.5, is correcting while the tyre still grips, and with the reference
live that was 2.1 degrees at 200 km/h, a car that never slides at all. At 200 km/h the band is **4.2 to 10.5 degrees**; at
20 mph, **11.6 to 29**. What that changes: `IsFullCountersteer`
needs priority 1, so the pedal handover and the projection's yield now begin at 33 degrees rather than 17.6; and the
assist's own onset of 10 degrees now **precedes** the blend in the early part of every slide, which is the intended
shape — the gentle response first, the earned authority later.

**The slide gets a careful extra, and its own release is what keeps it safe (driver, 2026-10-23).** Half the slide
recovers most cars; the ones it does not are the ones still **rotating into** the slide. Past
`CountersteerAssistOnsetDegrees` 10 and while `sign(slide) == sign(yawRate)`, the command gains
`CountersteerAssistDegreesPerSecond` **10 degrees per second** — a rate, so it builds rather than jumping. **Its two gates
are separate**: it grows while the **blend is committed** (`_slidePriority > 0`), and it is reset only when the **rotation
turns back against the slide**. Reading one test for both — grow only while rotating into the slide — left a car sliding
steadily, with no rotation to measure, on less than half the correction its slide asked for and indefinitely. Its ceiling is **a share of the slide, not an angle**: `CountersteerTotalShare` 1.0 is the whole answer the wheel may ever give, the blend already spends `CountersteerSlideShare` 0.5 of it, and the assist may add the difference — so at 33 degrees of slide the wheel can reach 33, which points it along the velocity vector and no further. 0.8 leaves it short of neutral. **The moment the rotation changes sign the degrees disappear**, and that instant
drop is the whole safety argument: the assist is removed exactly when the car stops turning into the slide, which is
before the correction can become the next snap. Adding amplitude without that release is what the 5-degree ceiling
tried to fix and could not.

**The inward lead is bounded like the outward one.** The tanh bound was written outward-only because the outside line
is a preference it can decline; the apex line needs the same shape for the opposite sign, because an aim that runs
ahead of the car toward the vertex asks for a nose the car cannot place. `LaneInwardLeadMeters` 3 and
`LaneInwardLeadTaperMeters` 1 mirror the outward pair, and the same tanh is applied through `LeadBounded`, which now
names the three preference reasons (outside line, avoidance, apex line) instead of two. The request clamp uses the
cap of the reason it is bounding, so the two directions keep their own numbers.

**The apex line is geometric, and it is pulled toward the apex by half a second of travel.** `IdealLineOffset` walks a
smoothstep from the outer edge at `t = 1` to the inner edge at `t = 0`, with `t = |distance to apex| / (radius x leadAngle
x IdealLineLeadMultiple)`. No time enters the profile, so a car arriving fast met the same shape it would have met slowly
— and arrived at it late, because steering and the lane mover take time to carry the aim across. `IdealLineLeadSeconds` 0.5
subtracts the distance the car covers in half a second from the profile's argument, so the line reaches the inside half a
second **before** the apex rather than exactly at it. The lead is in metres of the car's own travel, so it scales with
speed rather than with the corner.

**Side-by-side means the bodies, not the boxes.** `CombinedSize.Y` carries the reactive buffers (+1 m ahead, +0.25
behind) because it answers "is this rival near enough to matter"; the heading match used the same number, so a car
following two metres off another's tail was inside the gate and got its heading matched — turning toward the leader made
the leader turn with it. The match now measures `ModelDimensions.Y` halves, the real bodies, so it needs an actual
longitudinal overlap at any model size. The same-line case stays intentional: with no lateral separation there is no
"away" to close, and matching there is what keeps a nose-to-tail pair on one arc.

**The pass lane has a release, and it is the half that was missing.** `ComputeAvoidAheadLane` returns the offset
beside a slower rival ahead; the chain drops the request the tick `Brain.AvoidanceTarget` clears, which is the tick the
overlap ends — and that is precisely when the rear is still beside the car that was passed. The mover's rate keeps the
*speed* of the turn sane but not its *timing*, so the car turned in on top of the rival behind it. `ComputeAvoidanceLane`
wraps it: the offset holds while any rival is still within `AvoidHoldBehindMeters` 14 m behind and `AvoidHoldLateralMeters`
1.5 m of the held lane beyond the pair's combined width, and releases only when nothing is left alongside. The hold is
geometric, not timed — it ends when the cars are actually clear, at any speed.

**The chain's answer is a request, not the aim.** The lane mover (`Racer.cs` `UpdateLaneMover`) carries the carried lane onto that request at 26 m/s^2 with a 24 m/s cap, so a requirement that is valid for half a second cannot step the lane; the request's own rate feeds forward, so a line that moves smoothly is still tracked without the v^2/2a lag a pure arrival law would leave. The numbers are settled against the line's own motion in the ranked list, item 3. **Every writer of the request also records why**: `LaneReason` tags it own lane, curve lean, the corner's outside or apex phase, a commit card, avoidance, the recovery edge, a wall that moved it, or the debug pin (`Racer.LaneRequestReason`), and the debug ribbons take their colour from that tag — the ask dashed and dimmed, the aim solid — with the reason and both offsets named at the near end. The tag describes the request and never the aim, which the mover's bounded carry keeps different for a moment. **The aim is clamped into the interval between the car's own lane and the request** (`Racer.cs:907-914`), after the integration, because a lag is only harmless while the request holds still: with no source answering, the request *is* the car's own lane and a lagged aim turns the correction into a delayed function of the car's own motion, which is a limit cycle rather than a lag. The clamp takes the car's lane from the call site and not from the published centre deviation — two different frames, the steer reference sitting a lookahead ahead — and it zeroes the mover's rate when it moves the aim, or that rate winds up to the car's own lateral speed and carries the aim the wrong way on the first tick with a real request. **The walls are not among the mover's clamps**: they bound the request alone, so a corridor that closes on the car is carried onto like any other request.

**Lane systems only *set* the request; how hard it is pursued is the authority chain's job.** Every lane answer insets by half the car's width — the inside line, the avoidance bound and the corner line alike; `OutOfTrackDistance` (`Racer.cs:2807`) is centre-based on the raw half width, which is why a car pushed off is judged by its centre.

- **Two steering lookaheads, and both are named — do not conflate them**: `TryComputeCurveLean`'s chord is `CurveLeanChordSeconds` (`Racer.cs:675`), beside `CurveLeanRadiusMeters` (`:674`),
while `LookAhead.SteerRef` — which feeds every lane bound, the aim point and the debug line — is a **preview time** (`SteerPreviewSeconds` `Racer.cs`), floored and capped in metres, rather than a fixed distance.
Why: a fixed distance raises the loop's natural frequency with speed while the actuator lag stays put. `SteerLookaheadMinMeters` is its floor and is **live** again.
The consequence is worth knowing before it reads as a fault: for one lane offset the commanded steer goes as one over the square of the lookahead, so the same ask turns the car in harder the slower it goes, flat below `SteerLookaheadMinMeters` and above `SteerLookaheadMaxMeters`, and that rides on top of the ceiling rising as speed falls.
- **The corner line is a distance-parameterised offset profile from the apex** (`ComputeCornerLane` `Racer.cs:762`, `IdealLineOffset` `Racer.cs:753`), so grip never enters the lane path: a low-grip car needs a lower speed, not an earlier turn-in.
Its lead is the geometric one stretched by a constant multiple.
- **The corner line is entry-only (`d5a643f`)** — the exit half and the passed/active-apex bookkeeping are deleted, so past the apex the lane comes from the next corner's entry profile or the lean, and its offset applies only while the car has asked to position and is above apex speed.
Adjacent profiles still merge by taking the further-in of the two, and an opposite-direction pair is still excluded as a chicane.
  **The mover's acceleration is sized from the line, and a reversal drops its momentum (driver, 2026-10-23).** The outside-inside
traversal is `2 * safeBound` - **18 m** on a wide track - and it has to fit inside the blend's 1.5 s, which needs
`4 * 18 / 1.5^2 = 32 m/s^2` just to start and stop in the window. It ran at **14**, so the aim could not follow the ramp at
all, and a direction change first had to spend `24 / 14 = 1.7 s` killing its own speed - longer than the whole blend. That is
the "inertia" the driver felt. Now `LaneMoverAccelMps2` **26** and `LaneMoverSpeedCapMps` **24** — **halved to 13 and 12 while the reason is `OutsideLine`**,
because opening the line to the outer edge is a positioning choice made with room to spare and at the full rate it arrived as a
sideways lunge; the apex side keeps the full pair (45/25 was the first, deliberately over-sized cut; 26 covers 18 m from rest in 1.66 s against a 1.5 s window, and the window does not start from rest), and when the desired rate
reverses sign `_laneRate` is zeroed instead of being carried across: momentum the other way is a lag with nothing worth
keeping.

**The lane mover has no lateral leash to the car (driver, 2026-10-23).** `LaneInwardLeadMeters` and `LaneOutwardLeadMeters`
(3 m each) capped how far the aim could sit from `steerRefOffset`, with a tanh taper from 1 m and two interval clamps that
pinned `_carriedLane` between the car's own lane and the request. All of it is gone, together with `LeadBounded`: the aim was
tethered to the car, so a geometric line could not run ahead of it and every request read as a follower rather than a target.
What is left is the acceleration limit (`LaneMoverAccelMps2` 26) and the speed cap (`LaneMoverSpeedCapMps` 24) - inertia,
not distance - and the inside-mode guard, which is a direction test and not a leash.

**The line is measured in METRES of route, and reaches the extreme five metres early (driver, 2026-10-23).** On a bend a node is
more path than the straight line it stands for, so a node count measures a different corner at every radius. `metresToApex`
is `TrackPoints[apex].CumulativeDistance - reference.CumulativeDistance`, wrapped shortest-way-round with `RouteLengthMeters`
(the same pattern as the rival gap); `CumulativeDistance` is already metres and is rebuilt with the route. The stations are
`IdealLineOutsideMeters` **250**, `IdealLineRampStartMeters` **50** and `IdealLineApexMeters` **5**: the line opens 300 m out,
the ramp runs from 50 m to 5 m, and `linePhase` is `clamp((50 - d) / 45)`.

**The extreme is the raw half width, and the line reaches it five metres early.** `safeBound` is the track
half width **less half a car**, which is the right bound for a car that must stay inside the paint but the wrong one for a
line: the line is where the car's CENTRE goes, and the body overhanging the white line is the point of an apex - it was
stopping a metre short of the inside on every corner and reading as "the line never reaches the extreme". `IdealLineOffset`
is gone with it: the caller computes `linePhase = clamp((rampNodes + 5 - fwdToApex) / rampNodes)`, `offset = halfWidth *
(1 - 2 * linePhase)`, and the phase also names the reason (`<= 0` is `OutsideLine`, anything above is `ApexLine`) - one law
and one place. The five-node lead (`IdealLineApexLeadNodes`) puts the car ON the inside before the apex instead of arriving
at it on the way out. `safeBound` stays for the commit lane, which really does need the car's width.

**The line is pure node geometry now (driver, 2026-10-23).** `fwdToApex <= IdealLineOutsideNodes + rampNodes` (**250 plus the ramp**) opens the line, and inside
it `IdealLineOffset` is `t = clamp((ramp - d) / ramp)` with `ramp = 10 * TrackHalfWidth`: the outer clamp at the ramp's start,
the inside clamp on the apex, straight between. The ramp length is the traversal itself, so the line crosses the road at
forty-five degrees on any track and there is nothing to tune. Every time clock is gone - the live speed, the apex speed and
the latched approach speed that replaced both - with `IdealLineOutsideSeconds`, `IdealLineApexSeconds`, `_blendRefNode` and
`_blendRefMps`. Time was the wrong axis: room for an outside is room on the road, and the road is nodes.

**The avoidance hold is a pass in progress, not a lane the car owns (driver, 2026-10-23).** `_avoidHoldLane`/`_avoidHoldActive`
were written only inside `ComputeAvoidanceLane` and released only by a geometric test that needs a rival still alongside - all
of it from the 500 ms rival snapshot. Nothing cleared them in `Initialize`, so a hold from a previous race outranked the corner
line on the next one's first corners, and a rival that despawned or a snapshot that still said "alongside" for a car already
behind kept the held lane alive indefinitely. Now: a hard `AvoidHoldTimeoutMs` **8000** backstop, a drop when the held lane is
outside the current road, a clear in `Initialize` (with the inside guard, which had the same problem) and a clear in
`FinishStuckRecovery`.

**Four fixes from the audit (2026-10-23).** (1) `ClampLaneToWalls` added `carHalfWidth` to **both** walls, so with no rival
the corridor was `+- (roadWide - halfCar)` while the plateau demand is `+- roadWide`: every plateau tick the request was
trimmed ~0.95 m and relabelled `WallClamp`, which is why the outside looked like it never fired. The inset now applies only
to a side a rival constrains. (2) `metresToApex` is measured from the **car**, not the steering reference - the reference sits
a lookahead ahead, so the window opened early and died early by a distance that moved with speed. (3) The card's commit lane
required only that a Defend/Dive card be alive, so **every** corner returned `CommitCard` before the apex line was computed:
it now also requires the corner to be inside the line's window. (4) `_apexLineInsideNode` is dropped when its apex is passed,
so lap 2 no longer starts with the inside guard pre-armed at a corner lap 1 drove normally.

**The inside guard is re-armed every pass, and its firing is logged (driver, 2026-10-23).** `_apexInsideNow` is READ
unconditionally by `UpdateLaneMover` but was only WRITTEN inside `ComputeCornerLane`'s `ApexLine` branch - so a value latched
at one corner survived the apex (where the reason is no longer `ApexLine`, so the branch never runs), survived the gap to the
next corner, and then refused that corner's outward request using the PREVIOUS corner's direction. That is the intermittent
"the outside never fires", never the same corner because it depends on which corner last latched and which way the next one
turns. The flag is now cleared at the top of `ComputeCornerLane` and again in the chain when no corner is evaluated, and the
guard logs `Lane guard <car>: held at X, request was Y, inside dir Z, carrier W` once a second so the next session proves
whether it is still firing.

**The line picks its own corner, and the lean is out whenever one exists (driver, 2026-10-23).** The picker used to return the
**nearest** apex, which is usually a wide one, so `lineOwnsLane` went false between corners and the lean took the lane back -
the yellow line on top of a stretch that was supposed to be holding the outside. `NearestApexAhead` now only considers corners
under `IdealLineMaxRadiusMeters`, measures with `CumulativeDistance` (metres, wrapped) and requires a positive forward distance.
The lean's gate is the line's **activity**, not its existence: `ComputeCornerLane` returns `lineActive = wantsPosition` (inside
the window, apex still ahead) and the chain skips `CurveLean` exactly then. Gating on "a tight corner exists somewhere ahead"
had switched the lean off for most of a lap; the rule is with an outside there is no hug-inside, and outside that window the
lean is free.

**The two protocols travel together: without an outside there is no outside-inside (driver, 2026-10-23).** `CurveLean` pulls
**inward** from the car's own lane on any bend under `CurveLeanRadiusMeters` **500** by up to `CurveLeanPullMeters` 1, and it is
relative, so it can never ask for the outer edge: the inside half of the protocol with the outside half missing. Worse, it ran
BEFORE the apex window opened - so a car under 50 m of apex radius was pulled inside by the lean, and six seconds out the corner
asked for the outside, a reversal that read as the lane's inertia. Now the nearest apex decides first: if its radius is under
`IdealLineMaxRadiusMeters`, the corner owns the lane and the lean is skipped entirely, neutral until the window opens. Above the
threshold the lean is the only opinion there is, exactly as before. `NearestApexAhead` is computed once in the chain and passed
into `ComputeCornerLane`, so both halves read the same corner.

**The line has a speed floor (driver, 2026-10-24).** `wantsPosition` also requires `speedMps > ApexSpeedWithDownforce(radius) -
IdealLineSpeedMarginMph` (**15 mph**). Without it the window, being metres, keeps an apex near the start line inside it for ten
seconds at grid speed and every car dives for the outer edge off the line. The margin is measured UNDER the apex speed, so a car
that has already slowed for the corner keeps the line it committed to; a car crawling is simply not asked for one.

**The outside is asked for by TIME to the apex, not by being overspeed (driver, 2026-10-23).** The whole `RequiresPositioning` machinery used to
gate the corner line and it armed only on a corner that was **tight** (`R < 50`) and where the car arrived **10 mph over**
the apex speed. It is gone: the field, `ApproachStarted`, the promotion pass that set it, `inPositionRange` and
`_promotedApexNode` all deleted with it. Once the braking plan got good enough to arrive on speed, `excess` never crossed the threshold and the
line stopped existing: measured in one session, **65 % of samples read `armed=False`, `started=False`** and only 14 % had the
line at all. The outside is the first half of the line, not a correction for overspeed. The gate is now
`fwdToApex <= IdealLineWindowNodes` (**200 nodes**, which is both halves' room: the outside plateau plus the ramp),
measured in nodes, plus `DetectedRadius < IdealLineMaxRadiusMeters` (**50**, the same bound as `CornerTightRadius` but its own constant) and
`!HasPassedApex`. The radius gate replaces what the excess gate was really doing: deciding which bends are corners the car
positions for at all. Past that the bend is a kink the car takes without leaving its lane, and an outside line there was noise.

**The blend is one straight ramp in TIME to the apex, and nothing else (driver, 2026-10-23).** `IdealLineOffset` is
`t = clamp((IdealLineOutsideSeconds - secondsToApex) / (IdealLineOutsideSeconds - IdealLineApexSeconds))`, `offset = safeBound * (1 - 2t)`: the outer clamp at **1.5 s** out, the inside clamp
at the apex (**0 s**), linear between, and it holds the inside below 0.5 s and past the apex.  Both obvious choices fail: the **live** speed
makes `d / v` grow every time the car brakes, so the countdown expands under a braking car and the line reopens outward
mid-corner; the **apex** speed fixes that but measures the window with the speed the car will leave with, so in a slow corner
reached fast the whole two seconds fit inside one real second and the car never reaches the outside - the outside phase was
eaten. Entry speed is the honest clock: wide enough to open, and it can only ratchet up, never down. Time is what makes one corner read the same at any speed; a fixed node count is a different number of
seconds at 40 km/h and at 200, so the line arrived early when slow and late when fast. Shape: **apex T0, movement T-1.5 to T0**. The outside needs no constant of its own - everything beyond the ramp's start is
already the outer clamp, so the platform IS the clamp, and it holds for as long as the car is further out than two seconds. `IdealLineLeadSeconds`,
`IdealLineLeadMultiple`, the `leadAngle` geometry and `IdealLineBlendNodes` are **gone**. The distance is now **signed** (`+-count/2` on a
circuit, negative once the apex is behind): the old fallback swapped a passed apex for a wrapped forward distance, which read
as a near-full lap of "nodes to go" and opened the line outward on the exit.

**The apex is chosen by absolute distance, so the blend has an exit half (driver, 2026-10-23).** `NearestApexAhead` kept only
corners with a positive forward distance, so the tick after the apex the request jumped from the inside clamp of that corner to
the **outer** clamp of the next - a step from -safeBound to +safeBound at the moment the tyre is loaded hardest, which is the
"outside-inside is not blending" the driver reported. `IdealLineOffset` is symmetric in `|d|`, so the exit half already existed:
it was being cut off at the apex. The pick is now the corner with the smallest **absolute** distance, passed or not (wrapped to
`+-count/2` on a circuit), so the profile opens back outward through the exit and the hand-over happens where the two corners
are equidistant. `_apexInsideNow` therefore also carries `!HasPassedApex(apexNode)`: the inside guard has to end exactly at the
apex, or it would pin the car to the inside through the exit that the same profile is now asking it to open out of.

**The direction guard lives at the mover, not only in the chain (driver, 2026-10-23).** The demand-side latch was not enough:
the request is rebuilt every tick, so any reason along the chain can hand the mover a value the corner would never have asked
for - a re-read corner, a wrapped distance, another reason taking the chain for one pass. `UpdateLaneMover` therefore reads the
request, and in inside mode asks the only question that matters before anything moves: **is this ask toward the apex or away
from it?** Away, and it is not applied - the request is replaced by the car's own lane, so the step, the feed-forward and both
interval clamps all see a value that cannot widen the line. `_apexInsideNow` is republished by `ComputeCornerLane` every pass
and keyed to the apex node, so a new corner starts clean and passing the apex still ends the inside line.

**The line may open the car outward, but only until the demand crosses to its inside (driver, 2026-10-23).** Two halves, one
latch. While the demand is still **outside** the car the pull is allowed - that is the approach, the half of the outside-inside
line that puts the car wide. The first tick the demand is **inside** the car, `_apexLineInsideNode` latches that apex, and from
then on a demand that reads outward only holds the car where it is: no later re-read of a corner, wrapped distance or profile
step may drag it back out, which is what reads from the seat as the line snapping the car outward mid-corner. The latch is
keyed by apex node, so a new corner starts with the approach it is owed, and passing the apex is still the only thing that
ends the inside line.

**A demand behind the car's own lane holds the lane, it does not drop the line (driver, 2026-10-23).** The refusal used to
`return 0f`, which reads to the caller as "this branch has no opinion": `cornerLane != 0f` failed, the chain fell through to
`TryComputeCurveLean` and the apex line was replaced by the lean's yellow mid-corner. Measured in the log: 92 refusals in one
session, all of them near the apex (`demand 4.13, own -3.92, 18m to apex`), because the profile can still be sitting on its
outer clamp while the car is already inside. The branch now returns `ownLane` - the request becomes exactly where the car
already is, so nothing pulls outward - and keeps `LaneReason.ApexLine`, so the line lives until the apex as the only end.
The log line was renamed `Corner line held` and now carries the radius and the speed, which are the two numbers that decide
how far out the profile still is: `t = (|d| - v*0.5) / (R * leadAngle * 2)`.

**The apex line belongs to the nearest unpassed apex, not to `Brain.Corner` (driver, 2026-10-23).** `Brain.Corner` is the head of
the **plan's** queue, and `RefillApexQueue` deliberately looks past a corner it has flagged in `_skippedApexes` - a wide corner
with no braking plan, once the car is inside its window - so the head swaps to the next apex while the car is still inside the
flagged one's window. `ComputeCornerLane` read that head, so the profile suddenly measured a corner far down the road: `t` went to
1, the offset returned to the outer clamp and the reason became `OutsideLine`, which is the orange line. The magenta line did not
fade - it was replaced, and the car kept holding the outside through the corner it was actually driving. `NearestApexAhead()`
walks every corner and takes the smallest positive `ForwardNodeDistance`, flagged or not: the skip is a statement about the
braking plan and has no business taking the line away.

**The line ends at the apex and nowhere earlier (driver, 2026-10-23).** `entryActive` is `RequiresPositioning && !HasPassedApex`; the
old second term was `speed > ApexSpeedWithDownforce(radius) - 20 mph`, which dropped the inside line twenty miles an hour before the
apex speed was reached — that is, while the car was still closing. The symptom was the car giving up the outside-inside turn-in
early: the offset went to zero and the lane returned to the centre, so the car stopped steering for the apex with its speed still to
lose. The profile itself is unchanged: `IdealLineOffset` holds the outer clamp far out and smooth-steps to the inside as the apex
arrives, so it now reaches the inside at the apex instead of being cut twenty short.

The entry profile's far end is a **clamp, not a gate** — the outer edge is its value at every distance past the blend — so the window, not the profile, is what bounds it in time. Two time gates arm it, both in `UpdateCornerRequirements`: `RequiresPositioning` clears once the corner's apex is behind, so every approach re-arms in its own window (`Racer.cs:2965`); and the approach has to have **started**, judged once at promotion — a corner promoted to slot 0 while already inside the window is skipped for that approach (`Racer.cs:2945`), which is also the successive-corner rule, because promotion *is* the previous corner's apex. `RequiresBraking` is deliberately **not** cleared, because its range is measured to the entrance and it must outlive that to hold a plan through the apex. Filed as `outside-approach-window`, gate chain in `AGENTS-BACKLOG.md`.
- **The picker looks past a corner the plan will ignore, so the start rule above cannot misfire on a sweeper.** A corner too wide to ever ask for positioning (`DetectedRadius >= CornerTightRadius`) that has not asked for braking by the time the car is inside its 3.95 s window is flagged in `UpdateCornerRequirements`; `RefillApexQueue` gives slot one to the next corner instead, and the flag holds until that apex is behind. Without it a fast sweeper the car passes flat promotes the tight corner behind it *at its own apex* — already inside the tight corner's window — and the whole approach is lost, every lap. A corner that asks for a braking plan **revokes its own flag in that same pass** and retakes slot one, so an armed corner is never skipped, and a flag landing forces that tick's refill so the picker cannot switch a gate late; slot one therefore stops being "the nearest corner", and the consumers that ride it shift with it (cards, the nitro corner veto, the crossing taper, brake learning — `TEST-PLAN.md`). The table's own ceiling is a Debug setting (`Corner Ceiling`, 20–200 m, `ARS.CornerCeilingMeters`, shipped at 200 m since build 724) whose change rebuilds the table in place, mid-race included; the ceiling sits above `CornerTightRadius`, so the corners between the two are the "wide" class the skip exists for, and a ceiling at or under the tightness gate parks it. The two halves are one flag, written by the scan and read by the picker (`picker-skip-ignored-corner` in `AGENTS-BACKLOG.md`).
- **The blend refuses an ask that would move the car outward.** Below the far clamp, i.e. in the `ApexLine` phase, a demand lying outside the car's own lane is dropped and the chain falls through to the lean or to the car's own lane, so a car already inside the line is never sent back out to meet it and an entry cannot read as out and then in. Exempt by design: the hold at the clamp, because reaching the outer edge is what the outside approach is for and the outward lead lives on it; every other phase; and a car off the road, whose way back is this line. The comparison uses the car's own lane (`CurrentNodeOffset`, the green line) and not the mover's `steerRefOffset`, which reads inward by the bend's sagitta — metres on a fast tight corner — and would refuse the line outright there. The refusal **logs** rather than taking a reason tag, because it leaves the request where the chain had it and `_laneOverride` arms on the tag. Driver-observed as right, with 25 refusals logged across a race and the entry no longer reading out and then in.
- **A sweeping corner suppresses its own corner line through the positioning decision**: the flag requires a tight corner (`Racer.cs:2853`) and route speed governs a sweeper's speed either way.
That regime is no longer switchable (the Route Speed Limit toggle removal is the speed pipeline's `70e177c` note); the lever that shifts it is **Road Curvature Offset (mph)**.
- **Corner-commit maneuvers** replace the corner line with a lane beside the target rival on the corner's inside, and clean up when their apex is passed.
- **The lane pursuit's gate is REMOVED** — the pursuit runs ungated (see the ungated-pursuit section), driving from the car's live position.
- **A lane becomes a steer by pure pursuit off its aim point** — the bearing from the car's travel direction to `steerRefPoint.Position` offset along `steerRight` by the carried lane, turned into an angle by the wheelbase (`PursuitSteerDegrees` `Racer.cs`), then scaled by how far the car is crossed, except across turn-in itself — the braking target to the apex (`LaneCrossingShare` `Racer.cs`) — **that scale is off behind `LaneCrossingScale` while a comparison is owed, so the ask reaches the assembly unscaled**; **no radius produces a steer**, and every lane source feeds the request the mover carries the aim from.
  The inside line aims at the **inside edge** — `−cornerDir × (roadWide − half the car)` (`Racer.cs:808-809`).
  The steer carries pure pursuit's factor of two (`PursuitGain`, `Racer.cs`); a target that steps from wherever the car is instead holds a constant error and never converges.
  The ordering is load-bearing — car's-heading-to-the-aim is the steering command's own convention, and a minus inverts the lane.
  **The lane ask is scaled by the crossing, the ruling against a ceiling is why it is a scale, and the scale itself is off behind `LaneCrossingScale` while a comparison is owed.** That verdict — a ceiling flattens a lane law into a constant-command phase that builds lateral velocity the short tail cannot shed — was driven against the flat degrees-per-metre P, and both shapes tried since run into it: a step on the crossing, and a hard cap on the wheel. The share `allowance / (allowance + crossing)` fades instead and never reaches nothing. What keeps it out of the ceiling's way is also where it applies: only the ask that crosses the car further, never the unwinding one, and never during turn-in. The lookahead's length and the step into the line stay the levers for *how fast* a lane is taken (`AGENTS-TECHNOTES.md`). With the lane mover upstream, the owed drive is whether this scale is redundant or still the better answer for the case it was built for. **Reviving it as a 5° limit is a re-tune, not a restoration**: as compiled the allowance is `asin(1 m/s ÷ speed)`, about 1.6° at 130 km/h, and the same value by construction at every speed, while the gate is the crossing *direction* with the turn-in window exempt — not the lane moving outward.
  **The lane error is unscaled and the outward move is bounded instead** (`73756c7`; reshaped, build 703): the ask is the pursuit's own, and where the lane was asked for by the outside line or by avoidance the aim leads the car's own lane at full send until 1 m, then each outward tick carries the band's tanh share and the approach to the **3 m cap** is asymptotic — the hard stop the first bound drove at 1.5 m is gone (driver). The bound still travels out with the car, no lane change is cut short, and the clamps behind the glide are the failsafe. A recovery edge, a commit card's lane, a wall clamp, the apex crossing and the debug pin are exempt, and the gate is the reason the lane was asked for rather than a comparison of the two lanes — that comparison clamps an off-track car's return, which needs the mover's whole reach.
  **The share on those degrees was driven and reverted** (`9decee7` at a half, `a555ea9` at three quarters): the unscaled ask rings onto the outside line and a half leaves the following lazy, the car trailing the aim so a lane change and the entry land late. The ring is that share's own term, which is why scaling the gain could never separate an outside lunge from a turn-in.
  **The centring's own ratio**: `114.59·L/D²` degrees per metre of lane error, with the wheelbase `L` and `D = clamp(0.5 × speed, 8, 30)` m, so a metre asks a flat 8 m/s² across the half-second band — the degrees fall with speed and the lateral acceleration they buy does not. No lateral-g figure for the tyres exists in the tree: the limiter's ceiling is a slip-angle proxy and the grip is authored.
  The aim's Gs-aware preview (`ProjectAhead` `Racer.cs`) is described in the chain analysis; note that it steers *earlier still* when the car is being pushed away from its target — the case that falsifies "it reels back aggression" as a general claim.
  **The aim offset is clamped to the drivable edge** (raw half-width less half the car, not the raw edge the rival walls bound to), because the preview shift is applied after the wall clamp and could otherwise aim the car off the track.
  **With no lane demand the aim is the lane the car is already in** (see the base rule above), and it is the recovery's nearest-edge lane, not the lane aim, that turns an off-track car back; the preview only damps the drift the correction gives up rather than bounding it.
- **The bearing's own soft-compression is retired, kept as a lever.**
  The pursuit bearing was soft-capped — `threshold + scale × tanh((|bearing| − threshold) / scale)` — so an extreme bearing asymptotes instead of following the angle; the driver liked it enough to ask that it be kept in mind.
  It went in build #365 with its two menu knobs, because a threshold near the sine clamp's own bound made it nearly inert and the clamp already handles the sine's own non-monotonicity past a right angle.
  **Recover `CompressBearingAngle` from `879585e`** if extreme bearings ever need a lever — the candidate being an off-track recovery that proves too violent — and put it on the bearing *before* the pursuit law, never on the assembled steer.

## Speed pipeline — how fast the car goes
`Intention.Speed` is `min(cornerSpd + ARS.CornerOffsetMph, followTrackSpd + ARS.RouteOffsetMph)`, then clamped by `MaxSpeed` (with the standoff-rival term and artificial rubberbanding on top). The projection cap acts on throttle only and never on `Speed`.
**The offsets are split, one per authority, and each is applied inside the `min`** — so `CornerOffsetMph` also lifts apex-limited speed while apex speed itself stays pure physics (`Racer.cs:1544`).
`Brain.CurrentIntention.CorneringSpeedLimit` is computed alongside the offsets and **read nowhere**, not even by the HUD (`Racer.cs`) — a known write-only field, so a plan that expects it to bite is wrong.

- **`cornerSpd`** — the kinematic braking plan toward the apex speed over the distance to the apex node (`Racer.cs:3030`).
Its decel (`Racer.cs:3072`) is a grip-limited base **plus the gravity component of the mean grade over the braking span**, sampled along the same trimmed span the solve integrates.
**A crest before the corner is in that decel too**: `CornerPoint` carries a crest node, its curvature at a fixed probe and the extent its unload ramps over (`AssignCrestNodes` `TrackLoader.cs`), the solve scales **only the grip-limited base** by the clipped-triangle unload (`CrestDecelFactor` `Racer.cs`) — vertical curvature is a normal-load effect while grade is along-slope gravity — and the entrance walks back so the plan starts while the car is loaded.
**Its target is the crest's entry — where the track stops being flat and the vertical curvature begins — not the unload's peak (built, `27ee097`)**: braking from the peak spends the ramp already unloaded, so the entry is the last point at which the car is still fully loaded, and this exists so a car never arrives airborne with nothing to brake with.
- **Gs and degrees are not interchangeable, and a threshold censors its own evidence.**
  `HillGripDeltaGs` returns speed-scaled Gs (`v²κ/g`) while a pitch break is pure geometry, so one feature reads a couple of degrees and a couple of G depending on unit and window — a launch test must name the speed it assumes.
  A scan that stops at its threshold can never log what sits below it, so "the distribution shows a clean gap" is not evidence the threshold is safe; log the unthresholded value first.
- **Never smooth on top of smoothing, and state baselines in metres.**
  `TrackPoint.Elevation` is already a central difference of the route (`TrackLoader.cs:208`) and `Direction` a nearer one, while the crest curvature reads the raw positions — two vertical reads of the same road through different filters, so building curvature by differencing `Elevation` would compound the filter and shift the onset it is meant to find.
  The reads carry an unstated ladder (`ExactRadius`, `PreciseCurveRadius`, `GeneralCurveRadius`, crest): it wants one convention with two named rungs — short to locate a feature, long to measure its strength — counted in metres, because a node count is a different filter on every track (`AGENTS-BACKLOG.md`; spacing per track is in the Track facts below).
- **`followTrackSpd`** — route curvature from a time-based lookahead window (`Racer.cs:2760` sampled, `Racer.cs:3140` the law), the authority for sweeping corners, and **always in the `min`**: the Route Speed Limit toggle was removed in `70e177c`, replaced by the additive `RouteOffsetMph` (see the sweeper bullet above).
Its one live modifier is the **`SteerLimitedSpeed` blend** (`Racer.cs:1522`), a steer-implied/route-speed mix fed the *clamped* steer of the previous pass — so a tighter steer ceiling **raises** route speed.
- **Slope grip loss is LIVE** (exponential pitch model on corner + route speed) — ignore any older note claiming otherwise.
The hill/crest grip floors are tuned as one knob; read the code for current values, and note the known issue that the model reads an upcoming hill and brakes before the car reaches it.
- **Pedal gain is one linear map, no feedforward** (`Racer.cs:1247`), with the brake side divided by `BrakeErrorMultiplier` on top because braking is deliberately the softer side.
Both pedals then ramp to that target at a fixed slew rate.
- **Throttle limits are per-reason (`1ebbd2c`)**: every limiter owns a `MaxThrottleFrom<X>` level field in `UpdateThrottleReasonCaps` (`Racer.cs:1672`), and `ComposeThrottleCap` (`Racer.cs:1765`) takes `min(base, reasons)` **pre-slew**, naming the winner into `ThrottleReason` — the standing answer to "why isn't the pedal full".
The fields cut at the shared rate and recover more slowly (`GlideCap` `Racer.cs:1665`); the brake side mirrors them (`UpdateBrakeReasonCaps` `Racer.cs:1745`, `ComposeBrakeCap` `Racer.cs:1795`), and the composed `MaxThrottle`/`MaxBrake` are no longer drawn.
Overrides (grid wait, countersteer, stuck recovery) re-tag after; offtrack re-tags the throttle side only, since the brake path stopped taking an off-track command in `6a3dfa3`.
- **The overspeed reason is the incline-boost anti-cheat**: it compares measured forward Gs against the **total** wheel-pushed Gs with a deadband (`OverspeedExcessGs` `Racer.cs:3581`), so it fires whenever the car accelerates harder than its wheels are asking.
It arms only on a straight line above its speed gate (`Racer.cs:1677`), which deliberately leaves the launch alone — the engine's assist there is exactly the excess it would otherwise read.
- **Off-track projection cap (`Racer.cs:1611`)** is a **throttle-only** ceiling: 1.0 at the track edge, ramping to 0 across the five metres beyond it, and it fires only when the projection is outside the edge *and* further out than the car already is.
Applied with `Math.Min` after `ComposeThrottleCap`, it never reaches the brake and has no negative branch; it yields to the full-countersteer floor, because a slide is exactly when the projection is widest.
- **The throttle/brake avoidance override was REMOVED.** **Asymmetry is by design** — corner is the loose target, route the tight one, so don't "balance" them with flat offsets; on high-grip cars the route term wins.

## Corner lifecycle (apex table + three held targets)
1. **Apex table at generation** — `BuildApexTable` (`TrackLoader.cs:231`) detects corner regions on the smoothed radius, and `AddCornerRegion` takes each region's **smoothed minimum** as its apex, keeping the region's smallest precise radius alongside it as the 1g speed's radius.
   - Two apexes the car reaches close in time are one complex, and so is a pair whose exit and entrance aim the same way inside a short window — that branch catches a fast kink the car clears before it can react to the slow corner behind it, so the window must stay under the time a U-turn's two apexes take to be planned separately, or the second absorbs the first and the merged span never covers the lead-in.
   - It keeps the **slower by smoothed radius** and re-walks the survivor's span.
   - Both span ends derive from the apex, walked outward to where the smoothed radius opens past a multiple of the apex's own — entrance tighter than exit so the two ends cannot chatter — bounded by the region the detector already found and by a node cap that stays a node count, because a radius multiple there gave a sweeper twice its own radius.
   - **The apex is the smoothed minimum, not the region's tightest node** — a coarse chord measures a wide corner below the file's own decimal quantum.
   - A region must also run a minimum length and carry a tight enough apex to register at all, which is what drops the blips and shallow bends; no radius flag survives, and the racer decides per corner (`UpdateCornerRequirements` `Racer.cs:2840`).
2. **Held apex queue per racer** — `UpdateApexLeapfrog` (`Racer.cs:2797`) prunes stale entries and leapfrogs passed ones each tick, while `RefillApexQueue` (`Racer.cs:2883`) **derives the queue fresh on every refill**: slot 1 is always the nearest corner ahead, the rest the most restrictive corners this car wants a braking plan for; the queue invalidates on a route-data change or a low-speed incident.
Per-car apex speed is physics, `√(grip · gravity · radius)`.
3. **Braking map** — every held apex the car has flagged for braking feeds a kinematic solve over the usable distance and `cornerSpd` takes the lowest demand (`ApexBrakingSpeed` `Racer.cs:3165`, the `Math.Min` chain at `Racer.cs:1564`); its decel is grip-limited times a **per-apex learned factor**, seeded by two draws of ±0.05 — one per racer, taken once in the constructor, and one per corner — and corrected from the share of the braking phase spent at full pedal, with a flat cut for a corner taken with a real slide (`CommitBrakeLearning` `Racer.cs:1446`). **A second correction lands at the entrance pass**: the speed the plan asked for there against the speed the car carried. The step is linear and anchored where it bites: **twenty miles an hour over costs a fifth of the factor** (`BrakeEntryFactorStep` 0.2 over `BrakeEntryExcessMps` 8.9408), uncapped in total, with the **first five miles an hour dead** (`BrakeEntryDeadbandMps` 2.2352) because below that the gap is measurement noise rather than evidence. A quadratic was built and rejected the same day — it left a moderate overshoot untouched and only punished the gross one, which is the wrong end of the range. The notice is gated on `Show Track Analysis` as well as the notify setting: it is a diagnostic, not race furniture. It is the rule that catches a car that came in hot without ever braking — an approach the pedal share scores as perfect, because it never pressed hard. The factor runs 0.5 to 1.5.
The buffer is a flat time × speed, pressure does **not** scale it, and **a car whose ABS holds the pedal down reads as under-braking and drifts its factor up** — the learner watches pedal share, not achieved decel.
**A constant offset on the learned value cancels within a lap, wherever it is applied**: the commit reads that value and writes it back multiplied, so the only fixed point is the share target being met, and a per-racer base survives as a seed only — a trait that lasts needs the target or the gain, not the seed.
4. **Route-speed split** — route speed governs sweeping corners while the three-apex map handles tight ones from far away, and **min governs** (`Racer.cs:1544`).
**Braking anchors on the corner entrance less a flat lead** (`BrakeTargetBaseNode`), which retires the turn-in lead that scaled with speed and radius — the same node the release and the learned factor's sampler use, so plan, release and learning all name one node.
The apex plan is released only once braking is genuinely done, down at apex speed (`Racer.cs:1493`).
5. The old live rolling and far-killer scans are **superseded**, but `Brain.Corner` is **live again**: `CommitApexQueue` (`Racer.cs:2975`) re-instances it each tick from the nearest held apex, so the corner line, the commit lane and the maneuver gates read live state.

Track facts: node counts are treated as metres; measured spacing on `Airport Quick 1` averaged 1.034 m/node over 1491 gaps (0.877–2.394 m), so the convention is count-as-metres, not exact spacing. Circuit lookaheads use modulo and point-to-point clamps.

## Stability — the throttle cut's landing state (moved out of the project memory, verbatim)

- **Stability owns the throttle cut, and it is one signal (`Racer.cs`)**: the count of wheels reading zero on the side-slip axis (`ARS.WheelsOffGround`, `wheel+0x1C4`) is a **rate, not a target**: each wheel out of contact costs `StabilityFallPerWheelPerSecond` **0.5** of the cap **per second, additive** — four wheels empty it in exactly **half a second**, one wheel in two — and only with every wheel down does the cap recover, at one flat `StabilityRisePerSecond` 0.5/s, which is zero to whole in **two seconds**, clamped to 0..1. The four-to-one asymmetry of the worst case is the point: being out of contact is the emergency, coming back is the thing to be careful about. Read on every **timed core** pass, never per frame.
- **The arms it replaced are gone**: the ride-height-above-rest test and the yaw-demand test are **removed**, with `InstabilityRideHeightMargin`, `InstabilityMinSpeedMps` and `InstabilityYawTolerance`. That costs the four-wheels-down slide its throttle cut, so the TCS cap and the steering blend are what answer a slide now.
- **The lateral axis is load-bearing**: this was built once before (`cead3dd`), counting near-zeroes on the **longitudinal** ratio, and retired (`c18444a`) because a decompressed tyre still reads ~0 there. `CurrentMechanicalGrip` carries no stability factor, and nothing models roll, pitch, load or suspension.

## Durable gotchas — steering and pipeline (moved out of the project memory, verbatim)

**Trigger**: steering trap, pipeline trap, clamp, NaN, steer sign, limiter, gravity, pursuit, "do not fix this".

- **The pursuit's steer saturates at the bearing it clamps to, and there is no separate "recovery law" to preserve.**
  `PursuitSteerFromBearing` (`Racer.cs`) is the `atan` of the pursuit's own sine term, so past the clamp the command would fold back down instead of flattening, and any saturation or crossover angle is a derivation off that form rather than a value the code holds.
  The `ARS.Clamp` NaN trap is already the NaN discipline bullet in `AGENTS-TECHNOTES.md`, and the line once cited for a "recovery law" (`Racer.cs:731`) is `Math.Sign` of the corner direction inside `TryComputeCurveLean` — neither a clamp nor a recovery term.
- **Offroad gravity needs a baseline reset before the multiplier** — `Initialize` (`Racer.cs:417`) runs every race and on respawn, so setting gravity immediately before the offroad multiply is load-bearing or it stacks across restarts.
- **Do not go hunting for an old limiter that used the static TRlat — the ceiling never reads it.**
  `Handling.LateralTractionCurve` is only the *input* to the speed-scaled peak (`LateralPeakAtSpeed`, `Racer.cs:1023`) plus the countersteer gate thresholds, the slide blend, and every limiter function (`AckermannCeilingDegrees`, `GeometrySteerCeiling`, `PeakSlipCeilingAt`, `ResolveSteerCeiling`, `ApplySteerLimits`) has a caller.
- **A POSITIVE steer command steers LEFT — do not re-derive this.**
  `Racer.cs`'s own lane law gives it away (`laneSteerDeg = -laneErrorMeters * laneGain` against the `+ = right` **lane** convention, which is a different convention), the rival repulsion and corner-commit lane agree, and `TrackPoint.Angle` is already in the steer's convention (a left-hand corner is positive), so a reference built from it must be used as-is.
  `SteerLimitLeft` bounds positive commands and `SteerLimitRight` negative ones, which is what `Racer.cs`'s clamp now reads (`92d5867`), while the Show Pedal Trail fan draws each limit on its true side — orange left, red right.
  A rename here is a *side* swap, not a symbol swap, so check every use by side rather than by name.
