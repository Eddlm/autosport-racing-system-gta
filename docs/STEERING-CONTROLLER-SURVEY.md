# Steering Controllers in Racing Games and Sim-Racing AI — An Exhaustive Source-Cited Survey

**Scope.** How the *lateral* controller actually works in racing-game AI and sim-racing AI: what is driven to zero, the law, gain scheduling, look-ahead choice, slide handling, actuation limits, and tuning lore. Academic robotics canon (pure pursuit, Stanley, MPC as taught in robotics courses) is included only where a game/sim-specific source states a concrete implementation with constants.

**Method.** Primary sources only: actual driver source code, actual competition-paper equations, actual config YAMLs, actual tutorial code. Every claim carries its URL. Where a source is vague, this document says *"source does not state"*. Where a source is folklore with no derivation, it is flagged.

**One structural note that organises everything below.** Almost every working implementation resolves to the same two-degree-of-freedom shape:

```
steer = geometric_aim_angle_law( heading_error_to_aim_point )     <- the "pursuit" term
      + correction_terms( lateral_error, yaw-rate error, sideslip )<- the "feedback" terms
```

and the argument between implementations is almost never *which* of these to use — it is (a) whether the pursuit term's reference vector is the **body forward** or the **velocity** vector, (b) how the **aim-point distance is scheduled**, and (c) what happens when the tires let go.

---

## 1. TORCS `berniw` / TORCS 1.3.7 default driver — pure pursuit + lateral-error term + yaw-rate damping

- **Source code (driver core):** https://sources.debian.org/data/main/t/torcs/1.3.7+dfsg-5/src/drivers/berniw/berniw.cpp
- **Source code (constants + angle helper):** https://sources.debian.org/data/main/t/torcs/1.3.7+dfsg-5/src/drivers/berniw/berniw.h

`berniw` is the canonical TORCS reference driver ("the best hand-coded controller provided in TORCS (Berniw)", per https://citeseerx.ist.psu.edu/document?doi=2fe98d034cd955afa06200bbb2621196e05250bf&repid=rep1&type=pdf). It is the reference implementation for most TORCS-derived sims, including Speed Dreams.

**CONTROLLED VARIABLE / ERROR SIGNAL.** Three distinct signals, summed:

1. **Heading error from the car's BODY FORWARD vector to the car→aim-point direction.** The aim point comes from a path planner (`mpf->plan(...)`), and:

```c
/* steer to next target point */
targetAngle = atan2(myc->destpathseg->getLoc()->y - car->_pos_Y, myc->destpathseg->getLoc()->x - car->_pos_X);
targetAngle -= car->_yaw;
NORM_PI_PI(targetAngle);
steer = targetAngle / car->_steerLock;
```

Note the reference is `car->_yaw` — the **body forward** vector, *not* the velocity vector.

2. **A lateral-path-error correction**, gated by speed and off-path distance:

```c
if (!mpf->getPitStop()) {
    steer = steer + MIN(0.1, myc->derror*0.02)*myc->getErrorSgn();
    if (fabs(steer) > 1.0) steer/=fabs(steer);
}
```

`derror` is the pathfinder's lateral deviation from the planned path; `getErrorSgn()` is which side. The correction is **capped at 0.1** (i.e. 10% of full steer) and its gain is **0.02 per metre of path error**. This is the first hard number in the survey: *the pursuit term dominates and the lateral-error term is a bounded trim.*

3. **Yaw-rate error (a stability damping term)**:

```c
/* try to control angular velocity */
double omega = myc->getSpeed()/myc->currentpathseg->getRadius();
steer += 0.1*(omega - myc->getCarPtr()->_yaw_rate);
```

The reference yaw rate is `omega = v / R` — the **kinematic yaw rate the car ought to have for the path radius at its current speed**. Gain `0.1`. This is a textbook reference-yaw-rate stability loop and is the same primitive TUM's ESC uses 20+ years later (§10).

**CONTROLLER TYPE.** Not a named controller: **feedforward geometric pursuit (P on aim-point heading error) + bounded P on lateral error + P on yaw-rate error**. Two of the three terms are `P`; there is no I term anywhere in the steering path.

**GAIN SCHEDULING.** The pursuit term's gain is **`1/car->_steerLock`** — i.e. it normalises by the car's own steering lock, so the same law produces a bigger normalised command on a car with less lock. No speed scheduling of the steering gain. The *lateral-error* correction is **speed-gated but not speed-scaled** in the steering path (it is speed-scaled in the *brake* path — see below).

**LOOK-AHEAD / PREVIEW.** The look-ahead lives in the path planner, and the TORCS-bt variant states the schedule explicitly (§2). In `berniw` the target is `myc->destpathseg` — the next planned path segment — with no explicit look-ahead formula in this file; **source does not state** the aim distance here.

**SLIDE / DRIFT HANDLING.** Handled by the yaw-rate damping term above plus `CFRICTION` (a hand-tuned friction derate, `BERNIW_ATT_FMAGIC` / `caero` magic-number attributes read from the driver's XML per https://sources.debian.org/data/main/t/torcs/1.3.7+dfsg-5/src/drivers/berniw/berniw.h). **No countersteer controller, no sideslip term.** The car simply understeers out if the yaw-rate error grows.

**LIMITS / ACTUATION.** Hard clamp on the summed command:

```c
if (fabs(steer) > 1.0) steer/=fabs(steer);
```

The `steer` command is documented as `[-1.0, 1.0]` in the API (`tdble steer; /**< Steer command [-1.0, 1.0] */`, https://github.com/jzbontar/torcs/blob/master/src/interfaces/car.h). **No explicit steer-rate limit** — the rate limit is an emergent property of the physics and the `0.1` yaw-rate gain.

There *is* a speed/load-dependent **braking** multiplier in the same driver, which is a steer-adjacent load-sensitivity model worth recording:

```c
float weight = myc->mass*G;
float maxForce = weight + myc->ca*myc->MAX_SPEED*myc->MAX_SPEED;
float force = weight + myc->ca*myc->getSpeedSqr();
brake = brake*MIN(1.0, force/maxForce);
```

i.e. **brake authority scales with (weight + downforce·v²)/(weight + downforce·v_max²)**: more downforce, more braking. The identical pattern appears in the `bt` driver as `filterBrakeSpeed` (§2).

**WHAT THE SOURCE SAYS ABOUT TUNING.** The only tuning commentary in the source is via magic constants pulled from XML (`cfriction`, `caero`, `fuelperlap`) — `berniw.h` documents them as `BERNIW_ATT_FMAGIC "cfriction"` etc. There is **no comment in the source about oscillation or speed-dependent gain failure**. The tuning advice for this family of drivers lives in the *robot tutorial* (§3), not in the code.

---

## 2. TORCS `bt` driver (Bernhard Wymann) — pure pursuit + angular-velocity term + load-scaled braking, with the `speedangle` velocity reference

- **Source:** https://sources.debian.org/data/main/t/torcs/1.3.7+dfsg-5/src/drivers/bt/driver.cpp

This is the most *explicit* TORCS driver and the best-documented one in open source. It is a direct ancestor of the Speed Dreams robots.

**CONTROLLED VARIABLE / ERROR SIGNAL.** The steering law drives the **angle from the car's BODY FORWARD vector to the car→look-ahead-point direction**, divided by steering lock:

```c
float Driver::getSteer()
{
	float targetAngle;
	vec2f target = getTargetPoint();

	targetAngle = atan2(target.y - car->_pos_Y, target.x - car->_pos_X);
	targetAngle -= car->_yaw;
	NORM_PI_PI(targetAngle);
	return targetAngle / car->_steerLock;
}
```

But the same driver *also* maintains a **velocity-referenced** angle and uses it as a gate on the throttle — a genuinely important detail, because it shows a 2002-era driver distinguishing the two reference vectors:

```c
speedangle = mycardata->getTrackangle() - atan2(car->_speed_Y, car->_speed_X);
NORM_PI_PI(speedangle);
```

`atan2(car->_speed_Y, car->_speed_X)` is the **velocity vector's** direction relative to the car frame, so `speedangle` is the angle **between the velocity vector and the track tangent**. It is consumed by `filterTrk`:

```c
// Hold car on the track.
float Driver::filterTrk(float accel)
{
	tTrackSeg* seg = car->_trkPos.seg;

	if (car->_speed_x < MAX_UNSTUCK_SPEED ||		// Too slow.
		pit->getInPit() ||							// Pit stop.
		car->_trkPos.toMiddle*speedangle > 0.0f)	// Speedvector points to the inside of the turn.
	{
		return accel;
	}
	...
```

So: **steering is nose-referenced; the safety/throttle gate is velocity-referenced.** The steering aim point is a *track point* with an **overtaking offset** applied laterally (`getOffset()`), which is how this generation of drivers leaves the racing line — the lane is chosen by offsetting the aim point off the track centreline, exactly the architecture AGENTS-STEERING-style lane systems use.

**CONTROLLER TYPE.** Feedforward/geometric pursuit + a collision filter that blends toward a parallel-heading command. The collision filter is a real, quotable law:

```c
// Steer delta required to drive parallel to the opponent.
float psteer = diffangle/car->_steerLock;
...
if (car->_trkPos.seg->type == TR_STR) {
    if (fabs(car->_trkPos.toMiddle) > fabs(ocar->_trkPos.toMiddle)) {
        // Its me, I do correct not that much.
        psteer = steer*(d/c) + 1.5f*psteer*(1.0f - d/c);
    } else {
        // Its the opponent, so I correct more.
        psteer = steer*(d/c) + 2.0f*psteer*(1.0f - d/c);
    }
} else { ... 1.5f ... / 2.0f ... }
```

with `c = SIDECOLL_MARGIN/2.0f` and `SIDECOLL_MARGIN = 3.0f` [m]. This is a **distance-weighted blend between the path-following steer and a "drive parallel to the other car" steer**, with weights 1.5× (self) / 2.0× (opponent) — an early and explicit avoidance-override design.

**GAIN SCHEDULING.** The steering gain is again `1/car->_steerLock`. The **braking** side is explicitly speed-and-downforce scheduled:

```c
// Reduces the brake value such that it fits the speed (more downforce -> more braking).
float Driver::filterBrakeSpeed(float brake)
{
	float weight = (CARMASS + car->_fuel)*G;
	float maxForce = weight + CA*MAX_SPEED*MAX_SPEED;
	float force = weight + CA*currentspeedsqr;
	return brake*force/maxForce;
}
```
with `MAX_SPEED = 84.0f` [m/s].

**LOOK-AHEAD / PREVIEW.** This is the most exact look-ahead schedule in the survey, and it includes a **stability mechanism that is rarely mentioned in tutorials**:

```c
const float Driver::LOOKAHEAD_CONST = 17.0f;				// [m]
const float Driver::LOOKAHEAD_FACTOR = 0.33f;				// [-]
...
	lookahead = LOOKAHEAD_CONST + car->_speed_x*LOOKAHEAD_FACTOR;
	// Prevent "snap back" of lookahead on harsh braking.
	float cmplookahead = oldlookahead - car->_speed_x*RCM_MAX_DT_ROBOTS;
	if (lookahead < cmplookahead) {
		lookahead = cmplookahead;
	}
}
oldlookahead = lookahead;
```

So: **`lookahead = 17 m + 0.33 × speed`**, with a **slew-rate limit on the look-ahead itself** — when the car brakes hard, the speed-proportional term would collapse and yank the aim point back toward the car; the `oldlookahead − v·dt` floor prevents that "snap back". The source's comment is explicit about the failure mode it is preventing. Pit look-ahead is separately scheduled:

```c
if (currentspeedsqr > pit->getSpeedlimitSqr()) {
    lookahead = PIT_LOOKAHEAD + car->_speed_x*LOOKAHEAD_FACTOR;   // PIT_LOOKAHEAD = 6.0f
} else {
    lookahead = PIT_LOOKAHEAD;
}
```

The loop that walks the track to the aim point is a plain segment-walk (`while (length < lookahead) { seg = seg->next; ... }`) with **no search cap** other than `DISTCUTOFF = 200.0f` [m] applied elsewhere.

**SLIDE / DRIFT HANDLING.** No oversteer/sideslip term at all. The only grip-related machinery is:
- `TIREMU` = **minimum `mu` across the four wheels** (`initTireMu`), a conservative grip estimate;
- `MU_FACTOR` per-driver friction derate (default `0.69f`, read from the driver XML `BT_ATT_MUFACTOR`);
- `getAllowedSpeed` derives corner speed from `mu`, `G`, `r` and downforce, with the **learned-radius** (`SegLearn`) correction:

```c
float mu = segment->surface->kFriction*TIREMU*MU_FACTOR;
float r = radius[segment->id];
float dr = learn->getRadius(segment);
if (dr < 0.0f) { r += dr; } else {
    float tdr = dr*(1.0f - MIN(1.0f, fabs(myoffset)*2.0f/segment->width));
    r += tdr;
}
r = MAX(1.0, r);
return sqrt((mu*G*r)/(1.0f - MIN(1.0f, r*CA*mu/mass)));
```

The radius heuristic itself is in `computeRadius`:

```c
radius[currentseg->id] = (currentseg->radius + currentseg->width/2.0)/lastturnarc;
```
where `lastturnarc` is the accumulated arc of the turn normalised by `PI/2` — i.e. **a short, sharp turn gets a smaller effective radius** (more speed reduction) than a long, gentle one of the same nominal radius. The source comments the more conservative alternative as `// README: the outcommented code is the more save version.`

The stuck-recovery path *does* use a pure pursuit toward the *track centre*:

```c
car->_steerCmd = -mycardata->getCarAngle() / car->_steerLock;
```
with `getCarAngle()` defined (per the same codebase family) as `RtTrackSideTgAngleL(&car->_trkPos) - car->_yaw` — the **heading error to the track tangent**, i.e. body-forward vs track tangent.

**LIMITS / ACTUATION.** Command range is `[-1, 1]`; the code divides by `car->_steerLock` (radians) rather than clamping in the steering path, relying on the `[-1,1]` API contract. `isStuck()` uses explicit gates: `MAX_UNSTUCK_ANGLE = 15°/180°·PI`, `MAX_UNSTUCK_SPEED = 5.0f` [m/s], `MIN_UNSTUCK_DIST = 3.0f` [m], `UNSTUCK_TIME_LIMIT = 2.0f` [s].

**WHAT THE SOURCE SAYS ABOUT TUNING.** Directly relevant comments, verbatim:
- `const float Driver::FULL_ACCEL_MARGIN = 1.0f; // [m/s] Margin reduce oscillation of brake/acceleration.` — oscillation is attributed to the throttle/brake split, not to steering.
- `const float Driver::ABS_MINSPEED = 3.0f; // [m/s] Below this speed the ABS is disabled (numeric, division by small numbers).` — an explicit low-speed numerical-instability guard.
- `const float Driver::WIDTHDIV = 3.0f; // [-] Defines the percentage of the track to use (2/WIDTHDIV).`
- The look-ahead snap-back comment quoted above.

---

## 3. TORCS Robot Tutorial (Bernhard Wymann, `doc/tutorials/robot/`) — the taught pure pursuit, `17 + 0.33·v`

- **The tutorial's own constants and law, as restated verbatim by a tutorial that follows it:** https://godidier.github.io/drafts/torcs-ai-tuto-04.html
- **Tutorial referenced from the project docs:** https://torcs.sourceforge.net/docs/

**CONTROLLED VARIABLE / ERROR SIGNAL.** The angle from the car's **body forward** vector to the **car→target-point** direction:

```cpp
float CarController::GetSteering(float car_angle)
{
    float target_angle;
    Vector2D target = GetTargetPoint();

    target_angle = atan2(target.y - car->_pos_Y, target.x - car->_pos_X)
            - car->_yaw;
    target_angle = remainder(target_angle, 2*PI);
    return target_angle / car->_steerLock;
}
```

The earlier chapter's baseline is a different signal — heading error to the **track tangent** plus a normalized lateral offset:

```cpp
float CarController::GetSteering(float car_angle)
{
    float steering_angle;
    steering_angle = car_angle - car->_trkPos.toMiddle / car->_trkPos.seg->width;
    return steering_angle / car->_steerLock;
}

float CarController::CurrentCarAngle(tSituation* situation)
{
    float car_angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    car_angle = remainder( car_angle, 2*PI);
    return car_angle;
}
```
(https://godidier.github.io/torcs-ai-tuto-01.html). Note that `toMiddle / width` is a *normalized lateral offset in track-widths*, added directly to a *radian* heading error — a units-mixing shortcut that works only because `steerLock` is small. **Flag: this is tutorial folklore with no derivation**; it is presented as the simple baseline before being replaced.

**CONTROLLER TYPE.** Geometric pure pursuit, P-only (`target_angle / steerLock`). No I, no D, no yaw-rate term.

**GAIN SCHEDULING.** None in the steering law — the only scaling is `1/steerLock`.

**LOOK-AHEAD / PREVIEW.** Explicit, constant + speed:

```cpp
const float CarController::LOOK_AHEAD_CONST = 17.0; // [m]
const float CarController::LOOK_AHEAD_FACTOR = 0.33; // [1/s]
...
float look_ahead = LOOK_AHEAD_CONST + car->_speed_x * LOOK_AHEAD_FACTOR;
```

This is **the same `17 + 0.33·v` as the `bt` driver** (§2) — the tutorial is documenting the production driver's own numbers. The aim point is then constructed *on the track geometry*, not by intersecting a circle: on a straight, `target + direction*length`; on a curve, `target.Rotate(center, arc)` with `arc = (length/radius) * arcsign`. **The source does not state a stability rule tying look-ahead to stability**, but the companion `bt` code does (the anti-snap-back clause).

**SLIDE / DRIFT HANDLING.** None in the steering law. The tutorial's vehicle-dynamics chapter discusses understeer/oversteer, and the mitigation is on the **throttle**, not the wheel:

```cpp
float CarController::FilterTrack(float acceleration)
{
    ...
    if (seg->type == TR_STR){
        float to_middle = fabs(car->_trkPos.toMiddle);
        float safe_width = seg->width / WIDTH_DIV;
        if (to_middle > safe_width) return 0.0;
        else return acceleration;
    } else {
        float sign = (seg->type == TR_RGT) ? -1.0 : 1.0;
        if (car->_trkPos.toMiddle * sign > 0.0){ // inside a turn
            return acceleration;
        } else { ... if (to_middle > safe_width) return 0.0; ... }
    }
}
```
with `WIDTH_DIV = 4.0` and `MAX_UNSTUCK_SPEED = 5.0`. This is an **asymmetric throttle cut: no penalty on the inside of the turn, cut to zero on the outside past `width/4`.** The design intent is stated: *"when we are far away from the middle at the outside of a turn, we set our acceleration to zero. In contrast when we are inside, we can accelerate further, because the centrifugal force pushes the car outside."*

**LIMITS / ACTUATION.** `[-1,1]` via the API; no rate limit.

**WHAT THE SOURCE SAYS ABOUT TUNING.** Unusually honest, and worth quoting because it is a rare self-aware tutorial:
- On the heuristic corner-speed formula: *"Remember this is a heuristic and don't try to make full sense of why it works."* (the formula being `r = (segment->radius + segment->width/2) / sqrt(arc)`).
- *"Incidentally, this heuristic may lead the car to slightly go off track on some turns. We are about to fix that behavior."*
- On the whole approach: *"The problem with heuristics is that they may work great on some tracks but also horribly fail on some other tracks or dislike certain features."*

**Flag:** the `GetAllowedSpeed` formula `sqrt((mu*G*r)/(1.0 - MIN(1.0, r*CA*mu/full_car_mass)))` is copied verbatim from the production `bt` driver with the tutorial author's own admission that the corner-radius part is unjustified.

---

## 4. Game AI Pro, Chapter 40 — "Racing Vehicle Control Systems using PID Controllers" (Melder & Tomlinson)

- **Source:** https://www.gameaipro.com/GameAIPro/GameAIPro_Chapter40_Racing_Vehicle_Control_Systems_using_PID_Controllers.pdf

This is the only *game-industry* chapter in the canon that states the PID form for a racing controller explicitly, and its definition of the steering error is the one every later tutorial paraphrases.

**CONTROLLED VARIABLE / ERROR SIGNAL.** Quoted exactly:

> *"When used in a steering controller, the input may be the vehicle's required position, the scaled output is fed into the vehicle's steering wheel, and **the error is the perpendicular distance of the vehicle to the racing line**."*

And earlier, in the proportional-control discussion:

> *"For a vehicle driving around a circular track (**the error is the distance from the center of the vehicle to the center of the road**), if Kp is too large, a small error will result in the vehicle turning too much, then overcorrecting and ultimately weaving across the line uncontrollably with larger and larger amplitude."*

So: **cross-track / perpendicular distance to the racing line.** The reference is neither body-forward nor velocity — it is the racing-line geometry. The chapter does **not** state which point on the car the perpendicular is measured from.

**CONTROLLER TYPE.** PID, textbook form:

```
e(t) = Y(t) - R(t)                                                    (40.1)
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·(d/dt)e(t)                           (40.2)
```

**GAIN SCHEDULING.** Yes, and stated as the general remedy:

> *"Varying the K-coefficients. It may be the case that in different situations, different K values work better. For example, **at low speed the vehicle may require much larger K values to get it to start to move, compared to when it is running at high speed.** In this case, it would be useful to **linearly vary the K values as a function of speed**. Be careful with this, though, because **if the coupling is too great, a hidden positive feedback loop can be set up, resulting in instability.**"*

And under Adaptive Control:

> *"One adaptive control method is to use gain scheduling. This is where the controller parameters are directly adjusted by an external factor, such as the vehicle's speed or the surface grip, through a simple linear relationship or a more complex polynomial equation. Gain scheduling can work well when the system is largely dependent upon only one value, but **can become particularly difficult to tune when the values are dependent upon multiple factors**."*

This is the survey's clearest statement of both the *why* of gain scheduling and its *hazard* (gain scheduling can create an unintended feedback path).

**LOOK-AHEAD / PREVIEW.** The chapter itself gives **no look-ahead distance for the PID steering controller** — the error is the perpendicular distance, which is a *preview-free* signal. The look-ahead/preview architecture is deferred to the *predictive control* section, where it is described as a "runner":

> *"a simpler form, we can use a runner that guides the vehicle: the input to the steering controller becomes **the angle between the vehicle's forward direction and the vector from the vehicle to the runner**; the controller's error is then the angle difference."*

(quoted from the chapter summary at https://kenneil.medium.com/racing-vehicle-control-systems-using-pid-controllers-884748edde49, which mirrors the chapter). This is a **body-forward-referenced angular error to a moving aim point** — i.e. exactly a pure-pursuit-style error expressed as a PID input.

**SLIDE / DRIFT HANDLING.** Not covered. The chapter's control targets are position and speed; there is no sideslip or countersteer discussion anywhere. **Source does not state.**

**LIMITS / ACTUATION.** Not stated as a number. It states only the output scaling convention:

> *"In this case, the scaled controller output might vary from −1.0 (need to slow down) to +1.0 (need to accelerate)."*

for the speed controller; for steering it says only *"the scaled output is fed into the vehicle's steering wheel."*

**WHAT THE SOURCE SAYS ABOUT TUNING.** This is the chapter's main contribution and it is unusually explicit:

- Integral: *"Once the steady-state error has been reduced to zero, the integral error will still be nonzero and contribute to the controller output. It is this residual error (or memory) that causes the overshoot. Furthermore, as the integral and proportional term try to correct for the overshoot, oscillation around the target value can develop."*
- Integral wind-up: *"Because the integral error builds over time, if the system has a constant error (such as waiting for the start lights to change) the integral error will be constantly increasing. To counter this, it is desirable to be able to reset the integral error based on external events (such as the lights changing). Similarly, **capping the integral error** may also be desirable to limit the effects of the error memory."* — **anti-windup, stated for a racing controller.**
- Integral as a rolling average: *"This is best implemented by calculating the integral error as a rolling average, that is, on each frame the current integral is reduced by (1–T%) and T% of the current error is added back."*
- Noise and the D term: *"If the input data (R) is noisy, the derivative term can fluctuate in an undesirable manner. Smoothing the error data with a low-pass filter (i.e., a rolling average of the last few frames) can help to eliminate this. However, **this is effectively adding another integrator**."*
- Steering character target: *"steering should be reasonably responsive, but smooth, for normal driving. Additionally, we should certainly avoid over-steering and zigzagging, so we should aim for an **overshoot free characteristic**."*
- Tuning procedure: *"1. Set all gains (K) to 0 and increase Kp until the system behaves in a desirable manner with no overshoot or oscillation. ... 2. Increase Ki to eliminate the steady-state error. ... 3. Adjust Kd to reduce any overshoot or reduce the settling time as required."*
- The rule of thumb: *"In practice, it is often found that **Kd and Ki are approximately half of Kp**. As a general rule, if the system is unable to reach its desired value, increase Ki. If it oscillates, reduce Kp and Ki. If it is too slow to change, then increase Kd."*
- Metrics: *"For a steering controller, this would involve recording **the vehicle position perpendicular to the racing line**."*
- Cheating the line for variety: *"a steering controller would normally try to minimize the distance of the vehicle to the racing line, but by using an **error offset**, this would cause the vehicle to drive a short distance to the left or right of the racing line."*

**Flag.** The chapter states the PID for a *position* error but never resolves the obvious objection that a pure PID on perpendicular distance to a *curving* line has a speed-dependent, curvature-blind steady-state error — no feedforward term is given. The engagement with the problem is confined to "vary K with speed", which is precisely the coupling the chapter itself warns creates hidden feedback.

---

## 5. Game AI Pro, Chapter 41 — "The Heat Vision System for Racing AI" (Nic Melder) — where the steering *target* comes from

- **Source:** https://www.gameaipro.com/GameAIPro/GameAIPro_Chapter41_The_Heat_Vision_System_for_Racing_AI.pdf

Not a steering law — a **lane-selection** system that feeds one — but included because it is the canonical game-industry answer to "how do you decide *where* to aim when there is traffic", and it is the direct ancestor of the lane-offset systems in §1/§2 and of modern utility-based lane picking.

**CONTROLLED VARIABLE / ERROR SIGNAL.** The system produces a **track offset** ("desired track position"), which is then handed to the steering controller. *"Once the ideal track position has been found, this can be converted into a track offset that can then be passed to the steering controllers."*

The heat line is *"a one-dimensional 'heat line' which spans the width of the track at the car's position"*, stored as *"a one-dimensional fixed sized array of floats that is scaled to the width of the track at the vehicle's position."*

**CONTROLLER TYPE.** Not a controller — a cost field plus a descent rule. Heat writing passes include *Position* (large heat — cannot occupy), *Block* (remove heat), *Draft* (remove heat in a drafting cone). Minimum-seeking is explicitly a **physical simulation rather than an argmin**:

> *"in order to find the ideal track position, the target position should be moved from the vehicle's position along the line with decreasing heat. In order to avoid local minima (i.e., where a small heat hill exists) **momentum and friction should be applied to the target point's movement**. This is analogous to rolling a ball down the hill... Where the ball settles is the target vehicle's ideal track offset position."*

**GAIN SCHEDULING.** No — the offset field is recomputed per frame; the only scheduling is of the *heat signature sizes*: *"The actual size of the different heat signatures that the vehicles can add is determined by a number of factors that can include relative speed, driver characteristics, and difficulty."* And: *"for a vehicle in front, an aggressive driver will add heat with a smaller lateral spread than a less aggressive driver."*

**LOOK-AHEAD / PREVIEW.** Not stated as a distance. The heat line is at the car's position; observed vehicles are culled by *"all the cars within a short distance (e.g., 50 m) of the target"*. The heat footprint has along-track extent: *"for a square-based heat signature, the width across and the distance along the track should be independently scalable, whereas for a drafting cone the length, angle, and fall off should be controllable."*

**SLIDE / DRIFT HANDLING.** Explicitly **excluded**, with the recovery case named:

> *"it is likely that the heat vision system will not be used at all times... To illustrate this, **if the vehicle is off track or facing the wrong way after spinning, it does not make sense to use the heat vision system**, but the tests 'on track' and 'is spun' are still required to aid in the vehicle's recovery to track."*

**LIMITS / ACTUATION.** Not applicable — the output is an offset, not a steer command.

**WHAT THE SOURCE SAYS ABOUT TUNING.** Smoothing is presented as mandatory and its failure mode named:

> *"the resultant heat line may be quite rough with many discontinuities. Since the heat line is a simple array of floating-point values, graphical techniques can be used to smooth it. **Smoothing the line is important as it will remove any small discontinuities that may cause 'snagging' when determining the desired track position.**"*

And the honest scope limit: *"This system works well when driving on the track, especially when in a pack, but doesn't work in other situations such as recovering back to the track."*

---

## 6. Forza Horizon 6 self-taught controller ("Ziggy Racer") — the richest documented feedforward+feedback steering law in the wild

- **Source (README, which states the laws and the constants):** https://github.com/talontownsend/ziggy-racer (rendered): https://raw.githubusercontent.com/talontownsend/ziggy-racer/main/README.md

This is a *non-commercial hobby* project, but it is the single most explicit public statement of a complete modern steering law for a racing game, with constants, fitted exponents, and a documented experimental log including *failed* approaches. It uses only Forza's UDP telemetry and a virtual gamepad — no game internals.

**CONTROLLED VARIABLE / ERROR SIGNAL.** Three terms, quoted verbatim:

```
steer = k_ff · κ_ahead · load_comp        ← FEEDFORWARD: bend the wheel for the corner you can see
      + k_head · α                          ← pursuit heading alignment
      + kp·e + ki·∫e + kd·ė                 ← FEEDBACK: PID on cross-track error e
      + counter-steer(sideslip)             ← slide catch (applied last)
```

- `α` is the **pursuit heading** error — the angle to the aim point.
- `e` is the **cross-track offset** in a Frenet frame: *"Frenet state (d, d', d'') where d = cross-track offset"*, against a precomputed racing line: *"Localize itself on a precomputed racing line every tick, in a Frenet (arc-length / lateral-offset) frame."*
- `κ_ahead` is curvature read from the **stable racing line**, not the plan — and the source explains why this distinction is load-bearing:

> *"The feedforward reads the **stable racing-line curvature** a preview distance ahead (speed-scaled), not the per-tick planned-path curvature. That distinction was load-bearing: feeding the wobbling merge-path curvature back into the feedforward created a **planner ↔ tracker limit cycle** — the plan re-anchored to the car, the car chased the plan, and the two rang together into a bang-bang steering oscillation through the hairpin. Anchoring the feedforward to ground-truth line geometry broke the loop. This is a textbook lesson in *not closing a fast inner loop through a signal that itself depends on the loop's output.*"*

This is the survey's best-documented instance of a **feedforward that closed an unintended loop and oscillated** — the same class of hazard Game AI Pro ch.40 warns about for gain scheduling, here attached to a concrete mechanism.

**CONTROLLER TYPE.** **Curvature feedforward + pursuit heading P + cross-track PID + countersteer**, where the feedforward is the primary authority and the PID trims. Explicitly a two-degree-of-freedom design.

**GAIN SCHEDULING.** Yes, on a **vertical-load** basis rather than a speed basis:

```
load_factor = 1 + a_y / g          grip_scale = load_factor^0.705
```

> *"The 0.705 exponent is a fitted tire load-sensitivity. Over a crest the car goes light (`load_factor` → ~0.66), grip drops ~21 %, and the foot lifts *before* the rear steps out... **The steering feedforward carries the same load term (`ff · load_comp`) so the wheel angle grows as load falls** — the extra steer arrives before the wash, like a driver who sees the crest coming."*

The grip envelope is speed-scheduled and **identified from telemetry, not assumed**:

> *"The grip envelope slope `k ≈ 0.0025` and intercept `a0 ≈ 26 m/s²` were fit from the reference driver's lateral-g-per-speed-bin data... Downforce is folded into the cornering model as `a_lat(v) = a0 + k·v²`."*

And **anti-windup** is present and described:

> *"The cross-track integrator (`cte_int`) is clamped (`±3`), and integration is **suppressed while the actuator saturates** — the throttle integral stops accumulating once the grip cap binds, and an optional steer-clip anti-windup bleeds the integrator when the wheel is pinned at full lock and the integrator is winding the same direction. Classic integrator anti-windup, in two channels."*

**LOOK-AHEAD / PREVIEW.** Speed-scaled, and the *reason* is the classic trade-off:

> *"The feedforward reads the stable racing-line curvature **a preview distance ahead (speed-scaled)**."*

The merge horizon is separately adaptive: *"a QUINTIC merge trajectory `(d0,d0',0) → (0,0,0)` over a **speed/offset-adaptive horizon**; sample 5 horizons, score by cost, pick the best (temporal hysteresis)"*. **The source does not state the numeric preview-distance constant or its speed law.**

**SLIDE / DRIFT HANDLING.** This is the most developed slide story in the survey, and it uses **two independent detectors**:

> *"A yaw-rate reference `r_des = v·κ` is compared against filtered measured yaw rate. **Oversteer** (`|r_meas| > |r_des|`, same sign) triggers a **counter-steer damping term**; **understeer** (`|r_meas| < 0.75·|r_des|`) **eases the speed target** so the front regains grip. A separate **sideslip-angle detector** catches four-wheel slides the yaw-rate flag misses (a slide where the car rotates *less* than commanded while skating wide) and **blends the steering from path-lock toward counter-lock as the slide develops**."*

Constants stated: `r_des = v·κ`; understeer threshold `0.75·|r_des|`; `A_BRAKE ≈ 25 m/s²`; full-pedal decel `30 m/s²`. Countersteer is applied **last** in the sum.

The critical operational lesson is stated as a *diagnosis*, not a tuning tip:

> *"**The real diagnosis: steering-authority exhaustion.** The car arrives at the crest already too far *inside* the turn, and the wheel is **already correctly maxed outward**... Under the *light-crest grip*, the correction the car needs simply exceeds ±1.0 of available steering. **The budget is already spent, and spent correctly.** So **no steering-law lever can help** — you cannot fix a saturation by asking for more of a resource that's exhausted. ... **The fix that worked: an arrival-geometry lever.** If you can't add steering authority, reduce the demand: **shave the target speed by ~10 % in the crest *approach*** — *ending 20 m short of the hazard so it never slows during the light crest itself* (slowing *in* the crest is catastrophic — a mis-placed in-crest mask produced a 51 % slide rate)."*

Recorded statistics: baseline `13.3` incidents/1k for an anti-windup attempt with *no effect*; slew-rate limiting on the crest was **catastrophic at 40/1k**; the shipped approach-overslow gave **0 kill-zone incident-laps out of 56 vs 7/55**, `p ≈ 0.0001` Fisher exact, at `+0.05 s`.

**LIMITS / ACTUATION.** *"slew-limited stick"* — the steering command is slew limited at the actuator. **The numeric slew rate is not stated.** Steer is normalised to the stick's `[-1, 1]`. The friction circle bounds the pedals, not the wheel:

```
fc_frac = √( 1 − (a_lat_now / a_lat_max(v))² )
```

**WHAT THE SOURCE SAYS ABOUT TUNING.** Failures are recorded with numbers, and the methodology is the interesting part:

- *"Steering slew-rate limiting on the crest — **catastrophic**, 40/1k."*
- *"Anti-windup on the saturating wheel — no effect (`13.3` incidents/1k)."*
- *"P-term restoration in the grip-return window — slower, +3.8."*
- *"Anticipatory throttle-hold — kill-zone incidents *up* to 20 %."*
- *"Heading de-weighting — 20 %."*
- Acceptance metric: *"**Incident-lap metrics, not just section times.** A 130 km/h slalom costs little *time* but is wild and off-track-risky — so section time alone is *not* an acceptance criterion. Every re-measure now also reports cross-track RMS/p90, steer-reversal counts, sideslip p90/p99, and off-track %."*
- Standing-error diagnosis on the pedals, which transfers directly to steering: *"Both pedals were originally pure-proportional and so needed a *standing error* to hold pedal — the brake equilibrated at half-pedal, 40 m late... The fix is a **brake feedforward**."*

---

## 7. Pure pursuit as a proportional law — the geometry, stated with the gain

- **Source (derivation + gain-scheduling statement):** https://github.com/YangyangFu/autonomous-driving-book/blob/main/book/3-trajectory-tracking/lateral-control/pure-pursuit.md
- **Source (real implementation, tuned, with look-ahead schedule and clip bands):** https://github.com/Eelis03/mpc-lateral-controller — file https://raw.githubusercontent.com/Eelis03/mpc-lateral-controller/main/src/mpc_lateral/algorithm/pure_pursuit.py
- **Source (front-axle Stanley, same testbed):** https://raw.githubusercontent.com/Eelis03/mpc-lateral-controller/main/src/mpc_lateral/algorithm/stanley.py
- **Source (F1TENTH paper deriving PP from L1 guidance):** https://f1tenth.github.io/publications/Pursuit_Controller.pdf

**CONTROLLED VARIABLE / ERROR SIGNAL.** The angle `α` between the vehicle **heading** and the **vehicle→look-ahead-point** vector, where the reference point is the **rear axle**. From the F1TENTH paper: *"The L1 guidance of [22] results in a desired centripetal acceleration ac of the vehicle... `ac = vt · Ψ̇ = 2 (vt²/Ld) sin(η)` (2) ... where **Ld is the lookahead distance and η is the angle between the velocity vector and lookahead point**."* Note the paper's own definition of η is **velocity-referenced**, while the derived pure-pursuit form is normally applied with the heading — the paper collapses the two by assuming `vx = vt` and no sideslip: *"This assumes that no side slip occurs and the longitudinal velocity in the car's frame vx = vt."* **That assumption is exactly what breaks at the limit, and MAP (§8) exists to remove it.**

The steering law:

```
δ = tan⁻¹( 2 sin(η) · l_wb / L_d )          (Eq. 4, F1TENTH paper)
δ = tan⁻¹( 2 L sin(α) / l_d )               (autonomous-driving-book, identical form)
curvature = 2 sin(α) / distance
steering  = atan(wheelbase · curvature)     (Eelis03 implementation, exact code)
```

**As a proportional controller on cross-track error**, the autonomous-driving-book makes the gain explicit:

> *"If crosstrack error (e) is defined here as lateral distance between the heading vector and the goal point, then `sin α = e / L_d`. Thus the steering angle is `δ = arctan(2L sin(α)/L_d) = arctan(2Le/L_d²)`. **Pure pursuit is a proportional controller. The proportional gain `2L/L_d²` can be tuned at different speeds by creating a relationship between the speed and the lookahead distance.**"*

So the **pure-pursuit gain is `≈ 2 × wheelbase / L_d²`** for small angles on the *cross-track* error (equivalently `2·wheelbase/L_d` per radian on the *angle* error).

**CONTROLLER TYPE.** Geometric (P). No integral, no derivative. In the tuned implementation the derivative-like damping comes from **filtering the curvature**, not from a D term on the error:

```python
double curvature_raw = 2 * dy_vehicle / (lookahead_distance * lookahead_distance);
double curvature = curvature_filter.step(curvature_raw);
double steering_request_fb = std::atan(p_.wheelbase_m * curvature);
```
with `curvature_filter_Ts = 0.05 s` and `set_tf_pole(std::exp(-p_.tS / p_.curvature_filter_Ts))` — a first-order low-pass with pole derived from the loop time step. (TUM, §10, same file family.)

**GAIN SCHEDULING.** The gain is scheduled **implicitly, by scheduling the look-ahead**, and the source states the objective:

> *"`L_d = k_v · v_r`... The lookahead distance is usually chosen to be a function of the speed of the vehicle, **so that ω will not become more sensitive to α when v_r is higher**. The higher the speed, the higher the lookahead distance. This is because at higher speeds, the vehicle will cover more distance in the time it takes to react to the path."*

The tuned implementation is `lookahead = gain·speed + offset`, **clipped to a band**, with the two failure modes named in the docstring:

> *"The lookahead distance is scheduled with speed, `lookahead = gain * speed + offset` clipped to a fixed band, which is the standard remedy for **the two failure modes of a fixed lookahead: oscillation when it is short relative to speed, and corner cutting when it is long relative to the curvature of the path.**"*

Exact defaults (grid-search optimum over the benchmark):

```python
lookahead_gain: float = 0.15      # s
lookahead_offset: float = 2.0     # m
min_lookahead: float = 2.0        # m
max_lookahead: float = 25.0       # m

def lookahead(self, speed): 
    nominal = self.lookahead_gain * abs(speed) + self.lookahead_offset
    return float(min(max(nominal, self.min_lookahead), self.max_lookahead))
```

and the actual chord distance (not the nominal look-ahead) is used in the law:

> *"The actual straight-line distance to that sample, not the nominal lookahead, is used in the steering law, because **the geometry that derives the law is a chord**. Near the end of the path no such sample exists and the final sample is used, which makes the effective lookahead shrink as the vehicle runs out the last few metres."*

**LOOK-AHEAD / PREVIEW — the human-factors anchor.** Dean Pomerleau's ALVINN paper states the empirically derived rule that everything above is a rediscovery of:

> *"Empirically, I have found that **over the speed range of 5 to 55 mph, accurate and stable vehicle control can be achieved using the following rule: look ahead the distance the vehicle will travel in 2-3 seconds.** Interestingly, with this empirically determined rule for choosing the lookahead distance, the pure pursuit model of steering is a fairly good approximation to how people actually steer."*
> (https://www.cs.bu.edu/faculty/betke/cs440/fall2003-cs440/papers/pomerleau_dean_1995_1.pdf)

Note this is a **time-based** rule (2–3 s of travel), whereas `berniw`/`bt` use `17 m + 0.33 s·v` — a 0.33 s term, roughly 6–10× shorter than the human 2–3 s rule. The two rules are not reconcilable as stated, and neither source discusses the discrepancy. **This is a genuine disagreement worth flagging** (see *Disagreements*).

**SLIDE / DRIFT HANDLING.** **None — and this is the central admitted limitation of geometric pure pursuit.** From the F1TENTH/MAP paper:

> *"When driving corners at higher speeds, tire slip starts to occur and **the underlying assumption of Ackermann steering from Pure Pursuit no longer holds**. The desired acceleration ac is no longer correctly commanded and **the car starts to drift away from the trajectory**."*

Measured consequence, from the tuned comparison run at `1.0 m` initial offset (2× the tuning offset) and a `0.6 rad/s` actuator rate limit:

| Speed | Controller | Recovery distance (m) | Overshoot (m) | Steering effort (rad/s) |
|---|---|---|---|---|
| 5 | Stanley | 118.07 | +3.7263 | 0.5023 |
| 5 | pure-pursuit | 52.47 | +6.4457 | 0.4023 |
| 5 | MPC | 10.98 | +0.1689 | 0.1939 |
| 12 | pure-pursuit | 45.64 | +20.2088 | 0.5042 |
| 20 | pure-pursuit | 60.43 | +20.0957 | 0.5546 |

with the interpretation stated: *"An overshoot of about 20 m is the divergence limit at which a run is stopped, so those rows mean the vehicle left the path rather than that it overshot by that amount and came back... **both geometric laws demand more steering rate than the 0.6 rad/s actuator can deliver. The resulting lag turns the loop into a limit cycle**... Stanley oscillates out to 3.7 m and never settles inside the 120 m path, **pure pursuit leaves the path entirely**, and both of them sit on the rate limit for long stretches of the run."*

**LIMITS / ACTUATION.** `clip_steering(steering, parameters)` in both controllers; the `0.6 rad/s` actuator rate limit is a **simulator constraint, not a controller clamp**, which is the point the author draws:

> *"The decisive advantage is robustness, and it comes from one structural fact. **The steering rate limit is a constraint inside the optimisation rather than a clip applied to the output afterwards**, so the trajectory the controller plans is one the actuator can actually execute."*

**WHAT THE SOURCE SAYS ABOUT TUNING.** Best single paragraph in the survey on the geometric-vs-MPC decision:

> *"If tracking error on a well behaved benchmark is all you need, **a geometric law is a defensible choice and it is two orders of magnitude simpler**. ... The honest caveat is that this is partly a result about the tuning criterion."*

Grid-searched gains, with the selected values:

| Controller | Grid searched | Selected |
|---|---|---|
| stanley | gain in {1,2,3,4,5,6,6.5,7}, softening speed in {0.5,1,2,3,4} m/s | gain **6.0**, softening speed **3.0 m/s** |
| pure-pursuit | lookahead gain in {0.1,0.15,0.2,0.25,0.3,0.5,0.7} s, offset in {0.5,1,1.5,2,2.5,4,6} m | gain **0.15 s**, offset **2.0 m** |
| mpc | heading weight in {0,0.1,0.5,2,10}, steering-rate weight in {0.002,0.01,0.05,0.2,1.0}, horizon in {20,30,40,60} | heading **0.1**, rate **0.002**, horizon **40** |

Mean RMS error over the grid: stanley `0.1326 m`, pure-pursuit `0.1482 m`, mpc `0.1208 m`. Mean steering effort: `0.1745`, `0.1786`, `0.1934 rad/s`. Mean step: `0.047`, `0.063`, `0.523 ms`; worst mpc step `6.998 ms`.

And the pure-pursuit-specific failure: *"Pure pursuit does not settle at all on this path, **it hunts for the entire run**, which a root mean square number reports as the same kind of quantity as Stanley's steady offset even though it is not the same behaviour at all. Its RMS error of 0.2201 m and Stanley's 0.2028 m are close enough to read as a near tie and describe two different failures."*

**Stanley (for contrast, same testbed).** The Stanley law as implemented:

```python
denominator = self.config.softening_speed + abs(state.longitudinal_speed)
cross_track_term = math.atan(
    self.config.cross_track_gain * projection.lateral_error / denominator
)
steering = clip_steering(-heading_error - cross_track_term, self.parameters)
```
with the docstring: *"Stanley steers the front wheel to remove the heading error and adds a term that turns the front wheel towards the path in proportion to the arctangent of the cross-track error measured at the front axle, **softened by a speed term so the gain does not diverge at low speed**."* Reference point: **front axle**. This is the canonical **speed-divided gain** (`gain / (softening + v)`) — the one explicit "divide by speed" gain schedule in the survey, and its purpose is stated as *finite gain at standstill*, i.e. the **opposite** of the "more gain is needed at low speed" reasoning in Game AI Pro ch.40. See *Disagreements*.

---

## 8. ForzaETH / ETH-PBL **MAP** controller — L1 guidance + a steering lookup table (model-based pure pursuit)

- **Paper:** https://arxiv.org/abs/2209.04346 (Model- and Acceleration-based Pursuit Controller for High-Performance Autonomous Racing, ICRA 2023) — full text as scraped: https://f1tenth.github.io/publications/Pursuit_Controller.pdf
- **Code:** https://github.com/ETH-PBL/MAP-Controller
- **Config with the exact numbers:** https://raw.githubusercontent.com/ETH-PBL/MAP-Controller/main/map_controller/cfg/map_params.yaml
- **Integration in the race stack:** https://arxiv.org/abs/2403.11784 (v2 HTML: https://arxiv.org/html/2403.11784v2)

**⚠ The `η` reference vector disagrees between the paper and the shipped code — and this is the single most important discrepancy in the survey.**

- **Original MAP paper:** *"**L_d is the lookahead distance and η is the angle between the velocity vector and lookahead point**."* → **velocity-referenced.**
- **ForzaETH race-stack paper (the integration):** *"**η denotes the angle between the heading and the lookahead point** as illustrated in Fig. 23a."* (https://arxiv.org/abs/2403.11784) → **heading-referenced.**
- **The shipped code implements HEADING.** Verbatim from `controller/map/src/MAP_Controller.py`:

```python
eta = np.arcsin(np.dot([-np.sin(yaw), np.cos(yaw)], L1_vector)/np.linalg.norm(L1_vector))
```

`[-sin(yaw), cos(yaw)]` is the **body heading vector** derived from pose yaw. So the racing-track implementation of the "velocity-referenced" law is actually nose-referenced.

**Consequence for anyone porting this:** ARS's aim-error PID measures from the **velocity** vector (with a nose fallback below ~3 m/s). That matches the **original MAP paper**, not ForzaETH's shipped code. **It is not a bug on ARS's side** — the two published statements of the same controller contradict each other, and the code sided with the heading version.

**CONTROLLER TYPE.** **Feedforward model-based guidance + static inverse-model lookup.** The law is:

1. Compute desired centripetal acceleration `a_c = 2 v_t² / L_d · sin(η)` (L1 guidance, from fixed-wing UAV guidance).
2. Convert to steering **not** by the Ackermann inversion `δ = tan⁻¹(a_c · l_wb / v_x²)` — which is what pure pursuit uses and which *"assumes that no side slip occurs"* — but by a **precomputed lookup table** generated offline by simulating the identified single-track + Pacejka model:

> *"Since Eq. (5) cannot be solved analytically for the steady-state centripetal acceleration, **a LUT was generated to obtain a mapping from steering angle to acceleration**. For this, the system was simulated using the single-track dynamic model. The state was propagated with a range of constant longitudinal velocities and steering angles for which the resulting steady-state centripetal acceleration was recorded. With this, the controller is able to retrieve the required steering angle for a certain velocity and desired acceleration **by interpolating between the closest elements in the table**."*

LUT sweep, stated: *"These simulations, conducted for two seconds of simulated time, are repeated for **velocities from 0.5 to 7 m/s at 0.1 m/s intervals**."*

The swing-radius (steady-state) assumption is stated and defended:

> *"For a set of fixed steering angles at a certain speed v_x, the dynamics of v_y and Ψ̇ in Eq. (6) and Eq. (5) converge to a steady-state, where the car turns at a constant radius and with constant centripetal acceleration... Therefore, the proposed method is able to **neglect the transient phase**, commanding steering angles resulting in the desired steady-state centripetal acceleration. **The feedback loop and intrinsic stability of the guidance law [22] mitigate the inaccuracies during the transient phase.**"*

**GAIN SCHEDULING.** The LUT **is** the gain schedule: it is indexed by `(speed, desired lateral acceleration)` and therefore returns a different steering angle per speed for the same requested acceleration. There is no scalar gain to schedule. The look-ahead is separately speed-scheduled (below).

**LOOK-AHEAD / PREVIEW.** An affine function of speed, floored and capped — *and, in the shipped ForzaETH code, floored by a **lateral-error-dependent term***:

```
# Minimum and maximum lookahead distance in meters
t_clip_min: 0.3 
t_clip_max: 5

# Tuning parameters for lookahead distance as m*target_speed + q
m_map: 0.3
q_map: 0.15
```

> *"The lookahead distance is calculated with following formula: `m_map · targetspeed + q_map`. ... **`t_clip_min` is the minimum distance for the lookahead distance in meters to prevent oscillations**; **`t_clip_max` is the maximum distance for the lookahead distance in meters to prevent corner cutting**."*

So from the reference repo: `L_d = clamp(0.3·v_target + 0.15, 0.3, 5.0)` metres. Note the **gain is on target speed (the commanded speed), not measured speed**.

**The race-stack integration is different, and better**, and the difference is the single most transferable look-ahead idea in this survey. Shipped law (https://raw.githubusercontent.com/ForzaETH/race_stack/main/controller/map/src/MAP_Controller.py, default branch `ros2-humble` → https://raw.githubusercontent.com/ForzaETH/race_stack/ros2-humble/controller/controller/map.py):

```python
L1_distance = np.clip(self.q_l1 + self.speed_now*self.m_l1,
                      max(self.t_clip_min, np.sqrt(2)*lateral_error),
                      self.t_clip_max)
```

The lower clip bound is **`max(t_clip_min, √2 · |lateral_error|)`**, and the code comment states why: `# clip lower bound to avoid ultraswerve when far away from mincurv`. **When the car is far off the line, the look-ahead is forced to grow with the error, which reduces the pursuit gain** (gain `∝ 1/L_d`) and prevents the controller from demanding an impossibly tight arc to recover. This is a *stability* rule tying look-ahead to state, not just to speed — and it is implemented as a clip, not a mode switch.

Constants, per platform (all from the repo's configs):
| platform | `t_clip_min` | `t_clip_max` | `m_l1` | `q_l1` |
|---|---|---|---|---|
| NUC5 / NUC6 | 0.9 | 5 | 0.55 | −0.03 |
| NUC2 | 0.8 | — | 0.583 | −0.167 |
| paper Appendix A.1.4 ("tuned") | — | — | 0.6 | −0.18 |

**A negative `q_l1` with a positive `m_l1` means the affine term is short at low speed and grows steeply** — `L_d` at 5 m/s is `0.55·5 − 0.03 = 2.72 m`, at 10 m/s `5.47 m` (clipped to 5). The floor rather than the offset does the low-speed work.

**The stated stability rule — the best in the survey.** The repo's tuning README gives the reason L_d must be affine in v, with real numbers:

> *"the natural frequency of the controller must be smaller than half the frequency of the entire vehicle dynamics including delays"*, with `ω_n = √2·v_x/L_d`, `τ = L_d/v_x`.

That is: the pure-pursuit/guidance loop is modelled as a second-order system whose natural frequency **falls** as `L_d` grows, so `L_d` must grow with `v` merely to hold `ω_n` in the same relationship to the vehicle-plus-delay bandwidth. **This is the only derivation in the entire survey that derives the look-ahead schedule from a stability criterion rather than from a felt trade-off.**

**GAIN SCHEDULING — four independent schedules in the shipped controller**, which is more than any other source here:
1. **Longitudinal-acceleration scaling** with a hard deadband: mean `acc_now ≥ +1` → ×1.2; `≤ −1` → ×0.9. (Steering authority changes with whether the car is on the power or the brakes.)
2. **Speed-based steering downscale**: `factor = 1 - np.clip((speed - start_scale_speed)/(speed_diff), 0, 1) * downscale_factor`, with `start_scale_speed = 7.0`, end `8.0`, `downscale_factor = 0.2`.
3. **An undocumented extra speed upscale** — `steering_angle *= np.clip(1 + (self.speed_now/10), 1, 1.25)` — which **increases** steer with speed and saturates at ≥2.5 m/s.
4. The look-ahead itself (which sets the guidance gain).

**⚠ Schedules 2 and 3 push in opposite directions** (one downscales steering above 7–8 m/s, the other upscales it, saturating at 2.5 m/s). The extraction flags this as an unresolved conflict in the shipped code; the source states no rationale for either, and there is **no comment reconciling them**. Anyone reading the reference repo's config alone (`m_map`/`q_map`) sees only schedule 4.

**SLIDE / DRIFT HANDLING.** This is the *entire point* of MAP: it handles slip by replacing the no-slip Ackermann inversion with the tire-model inversion.

> *"The proposed MAP controller **incorporates the highly non-linear Pacejka tire model [13] into lateral geometric control**... This results in a more accurate control strategy that would need to be newly classified between geometric and model-based methods... When driving corners at higher speeds, tire slip starts to occur and the underlying assumption of Ackermann steering from Pure Pursuit no longer holds. The desired acceleration a_c is no longer correctly commanded and **the car starts to drift away from the trajectory**. Therefore, we propose a method of incorporating **the tire slip into the conversion from centripetal acceleration to the steering angle**."*

There is **no separate countersteer controller** — slide is absorbed in the inverse model. There is a hard consequence recorded: *"there is a lack [where] the model did not converge to a steady state acceleration but rather **an unstable drift, resulting in an upper bound for the achievable lateral acceleration at a given speed v_x**."* That upper bound is a **grip-ceiling constraint baked into the LUT** — the table simply has no entry above the achievable acceleration, and the lookup **truncates at the first NaN** rather than clamping, so the grip ceiling is enforced by the table running out.

The model was **identified, not assumed** — Pacejka parameters were fitted by *"steady-state cornering experiments… in which the car was driven at a constant speed and **the steering angle was increased slowly (0.02 rad/s)**"*, with *"an average residual of 0.87 N for the front and 1.35 N for the rear tires"* and *"25.5 % of points being rejected as outliers"*; shape factor `C` limited to `1.5` and curvature factor `E` limited to `1.1` *"to prevent the function to curve back towards zero for high tire slip angles."*

The only other slide-adjacent logic is **longitudinal** — a speed reduction keyed to lateral error × curvature:

```python
        lat_e_norm *= 2 
        curv = np.clip(2*(np.mean(self.curvature_waypoints)/0.8) - 2, a_min = 0, a_max = 1) # 0.8 ca. max curvature mean
        global_speed *= (1 - lat_e_coeff + lat_e_coeff*np.exp(-lat_e_norm*curv))
```
i.e. paper Eq. 11, `v_des = (1 + λ_lat(−1 + e^(−d_norm·c_norm)))·v_traj(s_ego + s_la)` with `λ_lat = 1` tuned and `0.8` a **hardcoded** "ca. max curvature mean" normaliser. So the shipped controller does have a **"you are off the line — slow down" term on the speed side**; it just has no corresponding steering-side countersteer term.

**LIMITS / ACTUATION — and this is the survey's most surprising finding on field 7. The shipped ForzaETH MAP controller has NO absolute steering-angle clamp at all. Its only steering limit is a slew limit on the *change*:**

```python
        # limit change of steering angle
        threshold = 0.4
        if abs(steering_angle - self.curr_steering_angle) > threshold:
            self.logger_info(f"[MAP Controller] steering angle clipped")
        steering_angle = np.clip(steering_angle, self.curr_steering_angle - threshold, self.curr_steering_angle + threshold) 
        self.curr_steering_angle = steering_angle
        return steering_angle
```

**`threshold = 0.4` rad per control cycle**, stateful (`curr_steering_angle` initialised 0 and **never reset**, including across state-machine transitions). Loop rate `self.rate = 40` (`self.create_timer(1/self.rate, self.control_loop)`) → **0.4 rad at 40 Hz ≈ 16 rad/s**. (ARS's equivalent slew is 360°/s ≈ 6.3 rad/s, i.e. ~2.5× tighter.)

**There is no `max_steer(v)` remap.** The speed dependence is `k_speed`, a *scale on the command*, not a bound — and it acts against the `clip(1 + v/10, 1, 1.25)` term (§GAIN SCHEDULING). The de-facto speed-dependent maximum steer is **LUT saturation**: the table's edge cells are NaN past the point where *"the simulated model did not converge to a steady state acceleration but rather an unstable drift"*, and the lookup truncates at the first NaN. **So the grip ceiling is enforced by the table running out, not by a clamp.**

Two further details worth recording:
- **The LUT is indexed by a delay-compensated speed, not the raw one.** Steering-path preview is separate from the L1 lookahead: the LUT's speed is sampled at a **forward-propagated position** to compensate actuator delay — `la_position_steer = position + v · speed_lookahead_for_steer`, then the waypoint's speed at that point. The code comment is explicit: `# lookahead for steer (steering delay incorporation by propagating position)`. Shipped `speed_lookahead_for_steer = 0.0` (safe) / `0.175` (aggressive); longitudinal `speed_lookahead = 0.25`.
- **There is a watchdog, not a limit**: if waypoints stop arriving, `waypoint_safety_counter >= rate/state_machine_rate * 10` → *"Received no local wpnts. STOPPING!!"* → `speed = 0; steering_angle = 0`.

**Also flagged — a probable latent bug in the waypoint indexing:** `d_index = int(L1_distance/0.1 + 0.5)` uses a **hardcoded 0.1 m waypoint spacing with no interpolation**. Unless the local planner emits 10 cm spacing, `L1_distance` and the real geometric distance diverge, and the L1 point is not where the formula thinks it is. The controller logs no warning for this.

**WHAT THE SOURCE SAYS ABOUT TUNING.** The repo ships the best-documented tuning procedure in the survey — a **two-point identification of the look-ahead line**, verbatim from `stack_master/config/README.md`:

> *"Set up a track where the car can safely drive up to 4 m/s (no sharp corners)… start a time trials session with the driving speed to constant 2 m/s. Set `m_l1` and `q_l1` to zero and `t_clip_max` to a high number. With that we make sure, that the L1 distance is kept constant at the value of `t_clip_min`. **While driving slowly increase `t_clip_min` until the car doesn't oscillate anymore.** Save this number for later as **D_2**. Now set the driving speed to 4 m/s and repeat the same process. Save the number set in `t_clip_min` as **D_4**. With these two datapoints gathered, we will interpolate to get the linear function representing the l1/velocity relation."*

with the arithmetic given explicitly:

> *"`m_l1` = (D_4 − D_2)/2, `q_l1` = D_2 − 2·`m_l1`, `t_clip_min` = D_2 − 1·`m_l1` (as the car will never drive slower than 1 m/s)"*

**That is a tunable look-ahead schedule derived from two measured oscillation thresholds** — the only procedure in the survey that turns "the speed at which it starts weaving" into a formula rather than a guess.

Race-day rules, verbatim:
> *"**If the car tends to cut corners decrease `q_l1` and/or `m_l1`, if it oscillates increase them.**"*
> *"`t_clip_min` and `t_clip_max` should not be changed unless the track is extremely fast or slow."*

**⚠ Internal inconsistency worth flagging:** the same README then says *"Proven good values are `m_l1 = 0.5`, `q_l1 = 0.2`"* — a **positive** `q_l1`, whereas **every shipped YAML and the paper's Appendix A.1.4 use negative `q_l1`** (−0.03 … −0.18). The code and the paper agree against that README line. Anyone tuning from the README alone would use the wrong sign.

Scaler tuning, verbatim:
> *"For `acc_scaler_for_steer` and `dec_scaler_for_steer` start out with both values at 1. Increase the `acc_scaler_for_steer` slowly, looking at the laptime and average/max lateral error. **Increase until the values stop improving or you see the car skidding.** It is generally good to keep the values in the range [1,1.5]… slowly decrease `dec_scaler_for_steer`… keep the values in the range [0.7,1]"*

Oscillation causes, and this is the survey's strongest *derivation* of why gain scheduling is necessary:

> **MAP §II:** *"In the traditional Pure Pursuit method [17], the L_d is a chosen constant. **This inherently presents a trade-off; a too-short L_d produces oscillations at high velocities; too high L_d results in inaccurate tracking and the cutting of corners.**"*
> **MAP §III-D, the stability derivation:** *"Park et al. showed that the evolution of the lateral distance to the trajectory as a result of the guidance law in Eq. 2 could be approximated by a **second order system with a time constant τ = L_d/v_x and natural frequency ω_n = √2·v_x/L_d**. Therefore, to converge to the trajectory as quickly as possible, L_d should be chosen as small as possible. On the other hand, the natural frequency increases with the velocity. **For stability, the natural frequency of the controller must be smaller than half the frequency of the entire vehicle dynamics including delays. Tests showed that for higher speeds this criterion was no longer met and the system became unstable.** To address this, L_d was scaled with the velocity with the affine mapping L_d = m + q·v_ctrl… **Relating the lookahead distance with speed made the natural frequency of the guidance logic independent of the velocity and ensured stability at higher speeds.**"*
> **ForzaETH §7.3.2 restatement:** *"If the distance is set too short, especially at high speeds, it can result in undesirable vehicle oscillations. Conversely, setting the distance too large may tempt the vehicle to cut corners and potentially lead to collisions. Therefore, **the goal is to keep L_d as small as possible without it causing oscillations**, striking a balance between stability and performance."*

**This is the survey's only look-ahead schedule derived from a stability criterion rather than from a felt trade-off, and it is the strongest available answer to "why is the look-ahead always proportional to speed?"**

Finally, an empirical argument for the lookup table over the analytic inversion, which is a *gain-tuning* argument in disguise:

> **MAP §IV-C:** *"**The linear version fails to complete a lap at higher speeds, as oscillations cause it to collide with the track boundaries**, and Pure Pursuit crashes due to under-steering in the third corner."* … *"Using the linear tire model resulted in a **ten-fold increase in average lateral error** during the two laps driven before the crash."*

Measured results (MAP Table I, at the higher grip coefficient): pure pursuit `8.74 s / 0.115 m mean lateral error / 0.33 m max`; MAP-linear `8.10 / 0.074 / 0.20` (**unstable — crashed after 2 laps**); MAP-Pacejka `7.39 / 0.055 / 0.23` at up to 11 m/s. ForzaETH's full-stack ablation: MAP `6.47 s` and `9.23 cm` lateral deviation, **~4.5 % faster and ~50 % less lateral deviation than the same stack with pure pursuit.**

In the ForzaETH race stack the same controller is described as: *"The primary lateral controller is a Model- and Acceleration-based Pursuit (MAP) controller that uses a lookup table approach to determine optimal steering angles based on racing conditions."* (https://github.com/ForzaETH/race_stack)

---

## 9. TUM `tam-stability-control` — the countersteer / ESC layer that sits *on top of* an arbitrary steering controller

- **Code (countersteer law):** https://raw.githubusercontent.com/TUMFTM/tam-stability-control/main/packages/stability_control_tam_cpp/src/countersteer.cpp
- **Config (all constants):** https://raw.githubusercontent.com/TUMFTM/tam-stability-control/main/config/stability_controller_config.yml
- **Reference pure-pursuit controller + its look-ahead formula:** https://raw.githubusercontent.com/TUMFTM/tam-stability-control/main/packages/tracking_controller_pure_pursuit/src/tracking_controller_pure_pursuit.cpp
- **README:** https://github.com/TUMFTM/tam-stability-control
- **Paper:** https://arxiv.org/abs/2608.17779

This is the survey's cleanest example of the **separation-of-concerns architecture**: a path-tracking controller that knows nothing about slides, wrapped by a stability layer that can override it. It is also the source of the survey's only **explicit rear-sideslip-dominance countersteer law with source code**.

**(a) The reference controller (pure pursuit, TUM).**

**CONTROLLED VARIABLE.** Cross-track deviation expressed as a **vehicle-frame lateral offset to the look-ahead point on the path**, converted to curvature:

```cpp
double dy_vehicle =
    -std::sin(odometry_buffer_.orientation_rad.z) * (x_traj - odometry_buffer_.position_m.x) +
     std::cos(odometry_buffer_.orientation_rad.z) * (y_traj - odometry_buffer_.position_m.y);
double curvature_raw = 2 * dy_vehicle / (lookahead_distance * lookahead_distance);
double curvature = curvature_filter.step(curvature_raw);
double steering_request_fb = std::atan(p_.wheelbase_m * curvature);
```

`dy_vehicle` is the **lateral component of the car→aim-point vector in the body frame** — i.e. the reference for the *look-ahead selection* is the body yaw, while the underlying arc construction is the rear-axle-to-aim-point chord. Then a **feedforward** on path curvature is added:

```cpp
double kappa_traj_lookahead_ = interp(s_current_m_ + current_velocity_mps_ * p_.lookahead_time_lat_ff_s, s_vec, kappa_traj_vec);
double steering_request_ff = p_.enable_lat_feedforward_perc * std::atan(p_.wheelbase_m * kappa_traj_lookahead_);
steering_request_rad_ = steering_request_ff + steering_request_fb + p_.static_steering_offset_rad;
```

**GAIN SCHEDULING / LOOK-AHEAD.** The look-ahead is a **three-term schedule including a curvature term** — the only curvature-dependent look-ahead in the survey:

```cpp
double lookahead_distance =
    p_.minimum_lookahead_distance_lat_m + p_.lookahead_time_lat_s * current_velocity_mps_ +
    p_.curvature_lookahead_gain * 1.0 / std::max(std::abs(kappa_traj_current), 1e-8);
lookahead_distance = std::max(lookahead_distance, 1e-6);
```

Defaults: `min_lookahead_distance_lat_m = 1.0` m, `lookahead_time_lat_s = 0.1` s, `curvature_lookahead_gain = 0.0` (i.e. **the curvature term ships disabled**), `enable_lat_feedforward_perc = 0.0` (**feedforward also ships disabled**), `static_steering_offset_rad = 0.0`, `curvature_filter_Ts = 0.05`, `wheelbase_m = 2.971`, `steering_angle_min_rad/max_rad = ∓0.43`.

Note the shipping look-ahead is **`1.0 m + 0.1 s · v`** — a time constant an order of magnitude shorter than Pomerleau's 2–3 s human rule and ~3× shorter than `berniw`'s 0.33 s term. See *Disagreements*.

There is also a **longitudinal** look-ahead for path matching, using a speed floor:

```cpp
odometry_lookahead.position_m.x += std::cos(orientation.z) * std::max(velocity_mps.x, 3.0) * p_.lookahead_time_long_ff_s;  // 0.05 s
```

**LIMITS.** `steering_request_rad_ = std::clamp(steering_request_rad_, p_.steering_angle_min_rad, p_.steering_angle_max_rad);` — a **tire-angle clamp of ±0.43 rad (±24.6°)**, stated as *"Enforce tire-steering limits."*

**(b) The countersteer layer.**

**CONTROLLED VARIABLE / ERROR SIGNAL.** **Load-weighted axle sideslip angle**, `(alpha_front − alpha_rear)`, gated on rear dominance:

```cpp
// Countersteer when rear sideslip dominates.
if (std::abs(sideslip_rear) > std::abs(sideslip_front) &&
    std::abs(sideslip_rear) > p_.countersteer_enable_sideslip && p_.countersteer_enabled) {
    countersteer_active_ = true;
    steering_request_rad_countersteer +=
        p_.countersteer_steering_factor * (sideslip_front - sideslip_rear);
}
```

**This is the exact law:** `δ_corrected = δ_request + k_cs · (α_front − α_rear)`, applied only when `|α_rear| > |α_front|` **and** `|α_rear| > enable_sideslip`. Rear slip must *dominate* front slip before any correction is added — a clean oversteer detector that needs no yaw-rate reference. `k_cs = 1.0`.

**⚠ SIGN DISCREPANCY BETWEEN PAPER AND CODE — flagged, not resolved by guessing. This is the most dangerous single item in the survey.**

The **paper** (arXiv:2608.17779, Eqs. 5–7) states:

> δ = ψ̇·l/v + α_f − α_r  **(5)**
> Δδ = ᾱ_r − α_f  **(6)**
> δ̄ = δ − Δδ  **(7)**
> *"In a simplified abstraction assuming linear tire behavior and applying a small-angle approximation, **the steering angle must be reduced by Δδ = ᾱ_r − α_f**, to achieve a neutral steering condition, where α_f = α_r."*

The **code** does the opposite sign:

```cpp
steering_request_rad_countersteer += p_.countersteer_steering_factor * (sideslip_front - sideslip_rear);
```

Substituting Eq. (6) into Eq. (7) gives `δ̄ = δ + (α_f − ᾱ_r)` — **the same expression the code computes.** But the paper's own prose says the steering angle *"must be **reduced** by Δδ"* while its Eq. (7) writes a **minus**, so Eq. (6)+(7) together add rather than subtract. Deriving from the code's own slip convention (`slip_angle_def = -atan2(vy, max(1.0, vx))`, giving `α_f = −δ + β` and `α_r = β + l_r·ψ̇/v`), we get `α_f − α_r = −δ − l_r·ψ̇/v`, so with `k = 1` **the code *reinforces* steering exactly where the paper's prose says it should *reduce* it.**

**Trigger, gain and magnitude all agree between paper and code; only the applied sign disagrees.** The extraction flags this explicitly as a trap: **port the code's convention, not the paper's algebra, or the countersteer steers *into* the slide.** Anyone implementing from the paper alone has a 50 % chance of inverting their countersteer — and the failure will look like an understeer problem, not a sign problem.

**Two design choices stated as policy, both directly relevant to field 6:**
- *"A **neutral or slightly understeering condition is desired**, as motion controllers are generally able to maintain stability in these conditions."*
- *"Intervention thresholds for the cs are therefore set to be **more sensitive** than in the esc system."*
- And the delay argument for preferring the wheel to the brake: *"the steering actuator exhibits a shorter delay of **30 ms** compared to the brake actuator, which has about **150 ms**, enabling a more rapid response in dynamic oversteer scenarios."*

And the deliberate anti-model stance, which is the exact opposite of MAP's (§8):

> *"Although the cs is designed for stationary circular motion and linear tire dynamics, our experiments demonstrate improved stability across diverse driving conditions. The feedforward estimation error due to simplifications is mitigated by a high-frequency feedback control loop. This is highlighted in the presented ablation studies, where the cs can stabilize the vehicle even in highly nonlinear tire regions. **We argue that neglecting nonlinear tire dynamics represents a strategic advantage in this case, as it enhances system robustness against external disturbances and model mismatches.**"*

The sideslip is **load-weighted per axle and filtered** — and note the gate tests the **filtered** value, which is easy to miss:

```cpp
weighted_sideslip_front_ = (tire_loads.front_left * alpha_rad.front_left +
                            tire_loads.front_right * alpha_rad.front_right) / front_tire_load;
weighted_sideslip_rear_ = (tire_loads.rear_left * alpha_rad.rear_left +
                           tire_loads.rear_right * alpha_rad.rear_right) / rear_tire_load;
weighted_sideslip_front_filtered_ = slip_angle_filter_front_.step(weighted_sideslip_front_);
```

with the filter pole built from the config time constant: `set_tf_pole(std::exp(-p_.tS / p_.slip_angle_filter_Ts))`, `slip_angle_filter_Ts = 0.15` s, `tS = 0.01` s.

**Constants, all from the config:** `steering_factor: 1.0`, `enable_sideslip: 0.009` [rad, ≈0.52°].

**LIMITS.** After the countersteer is added, the **same tire-angle clamp applies**, so countersteer cannot exceed the physical lock:

```cpp
steering_request_rad_ = std::clamp(steering_request_rad_, p_.steering_angle_min_rad, p_.steering_angle_max_rad);
```
(`vehicle.steering.min_angle = -0.43`, `max_angle = 0.43` rad by default.)

**Fail-safe behaviour**, which matters for a racing bot: if steering feedback or slip angles are invalid, the countersteer **passes the motion controller's request through unchanged** rather than guessing:

```cpp
} else {
    // Pass through the motion-control request.
    countersteer_active_ = false;
    steering_request_rad_ = steering_request_motion_control_rad_;
}
```

**(c) The ESC layer (yaw-rate + sideslip PID).** Config, verbatim:

```yaml
ESC:
  enabled: True
  max_brake_pressure: 60.0
  # --- Yaw Rate PID ---
  tS_psi_dot: 0.92
  kp_psi_dot: 8.0
  ki_psi_dot: 1.0
  kd_psi_dot: 0.6
  # --- Slip Angle PID ---
  tS_beta: 0.9
  kp_beta: 25.0
  ki_beta: 8.0
  kd_beta: 1.0
  # --- Maximum Values ---
  max_integrator_yaw_moment: 1325.0
  max_brake_pressure: 80.0
  # --- Activation and Deactivation Thresholds ---
  threshold_velocities: [25.0, 50.0]
  beta_threshold_activate: [0.035, 0.0174]
  beta_threshold_deactivate: 0.035
  psi_dot_error_threshold_activate: [0.0401, 0.0349]
  psi_dot_error_threshold_deactivate: 0.026
  beta_error_threshold_activate: [0.061, 0.035]
  beta_error_threshold_deactivate: 0.018
  min_activation_velocity: 7.0
```

Two PIDs (yaw-rate error and slip-angle error), each with a **time constant** acting as a rolling window (`tS_psi_dot: 0.92`, `tS_beta: 0.9`), an **integrator clamp** (`max_integrator_yaw_moment: 1325.0`), and **hysteresis thresholds that are speed-scheduled as two-element arrays interpolated on `threshold_velocities: [25.0, 50.0]` m/s** — e.g. `beta_threshold_activate: [0.035, 0.0174]` means the activation slip-angle threshold **tightens at higher speed**. Note the asymmetry between activate and deactivate thresholds on every channel (e.g. `psi_dot_error_threshold_activate: [0.0401, 0.0349]` vs `psi_dot_error_threshold_deactivate: 0.026`) — deliberate hysteresis to stop the ESC chattering: **deactivation thresholds are constants while activation thresholds fall with speed, so the band widens as speed rises.**

**⚠ Config defect worth flagging: `max_brake_pressure` is declared TWICE in the same YAML block — `60.0` under `# --- General ---` and `80.0` under `# --- Maximum Values ---`.** The later key wins, so the operating value is `80.0`, but anyone reading the first occurrence gets the wrong number. Also note the **code defaults differ from the shipped YAML in the *direction* of the thresholds** — code default `min_activation_velocity` is `10.0` vs YAML `7.0`, and code default deactivation thresholds are much larger (`0.0524 / 0.0873 / 0.1745` vs YAML `0.035 / 0.018 / 0.026`). **Only the YAML is the operating point.**

**The activation predicate is the most sophisticated in the survey — four magnitude tests AND three directional-consistency sign products:**

```cpp
  // Require large, directionally consistent yaw-rate and sideslip errors.
  return esc_enabled_ && velocity_ > p_.min_activation_velocity &&
         std::abs(beta_) > beta_threshold_activate_ &&
         std::abs(beta_error_filtered_) > beta_error_threshold_activate_ &&
         std::abs(psi_dot_error_filtered_) > psi_dot_error_threshold_activate_ &&
         psi_dot_ * psi_dot_error_filtered_ < 0.0 && beta_ * beta_error_filtered_ < 0.0 &&
         beta_error_filtered_ * psi_dot_error_filtered_ < 0.0;
```

The three sign products require that the yaw-rate error **opposes** the current yaw rate, the sideslip error **opposes** the current sideslip, and the two errors **agree with each other** — i.e. **intervene only on a coherent, self-consistent departure, never on noise or on a single outlying channel.** Release is separate and requires *all three* errors to collapse simultaneously:

```cpp
bool ESC::deactivate_esc() const
{
  return !esc_enabled_ || velocity_ < p_.min_activation_velocity ||
         ((std::abs(beta_) < p_.beta_threshold_deactivate) &&
          (std::abs(beta_error_filtered_) < p_.beta_error_threshold_deactivate) &&
          (std::abs(psi_dot_error_filtered_) < p_.psi_dot_error_threshold_deactivate));
}
```

**How the steering request is modified: NOT AT ALL by the ESC.** The ESC shifts **brake pressure only** — differential front braking. Order in the node, verbatim:

```cpp
  // Calculate slips before supplying them to the stability-control components.
  slip_calculation_->step();
  ...
  esc_->step();
  ...
  slip_control_->set_esc_active(esc_->get_esc_active());
  slip_control_->set_target_brake_pressure(esc_->get_brake_pressure_target_bar());
  slip_control_->step();

  countersteer_system_->step();
```
**slip estimation → ESC (brakes) → slip control (sees ESC's pressures) → countersteer (overwrites the steering request last).** Output: `ctrl_out.lateral.steering_tire_angle = countersteer_system_->get_steering_request_rad();` — **the countersteer layer is the only thing that touches steering.** Loop 100 Hz (`tS = 0.01`).

**The yaw moment and its conversion to brake pressure**, verbatim:

```cpp
    yaw_moment = p_.yaw_inertia * (pid_feedback_psi_dot.feedback + pid_feedback_beta.feedback);
    double additional_brake_pressure =
      yaw_moment * 2.0 / p_.trackwidth_front * p_.force_to_frictionless_brake_pressure_bar_per_N;
```
with `force_to_frictionless_brake_pressure_bar_per_N = tire_radius_front / (brake_pad_mean_radius · brake_pads_number · π · (brake_piston_diameter/2)² · pascal_per_bar)`. So **`M_z = I_zz·(PID_ψ̇ + PID_β)`**, converted to a pressure delta and **shifted between the two front wheels** with real minimum-pressure and headroom limits — the β PID is stepped on the **negated** error (`pid_beta_.step(-beta_error_filtered_, ...)`).

**The anti-windup idiom is conditional integration, not back-calculation**: `error_integrator_ += (integrator_update - error_integrator_) * (integrator_update < saturation_high_ && integrator_update > saturation_low_)`.

**Two design choices stated as policy:**
- *"The brakes on the rear axle are not used to contribute a moment, **since the rear tires are already saturated in an oversteer situation**. **Understeer situations are not addressed by the esc system**, as they are considered less critical due to slower dynamics and are assumed to be managed by the motion controller."*
- *"Corrections using the steering angle rather than the brake pressure command are generally preferred because they have less influence on the vehicle's longitudinal dynamics and result in less performance degradation."*

**The longitudinal side (ABS/TC) uses the same idiom — threshold state machines with tuned reduce/increase step asymmetry, not PID** (e.g. front ABS `reduce_step: 0.038`, `increase_step: 0.023`, `slip_threshold_hold_reduce: -10.0`, `slip_threshold_reduce_hold: -12.0`, `slip_threshold_hold_increase: -25.0`, `slip_threshold_increase_hold: -15.0`, `slip_threshold_safe: -7.0`, `slip_angle_max: 0.07`; TC with `operation_mode: 3`, `min_activation_velocity: 1.0`, `max_slip_throttle_cut: 25.0`). The slip definition is `slip_def = clamp((rot_vel - vel_over_ground) / max(3.0, vel_over_ground), -1.0, 1.0)` — note the **3.0 m/s denominator floor**, the same low-speed numerical guard as TORCS's `ABS_MINSPEED` (§2).
The paper's rationale for the state machine over a controller: *"The sc is implemented as a **model-free finite-state machine**, as such systems demonstrate robustness and have been extensively validated in real-world applications."* And the differential subtlety: *"in the acceleration case, the rear axle is controlled as a single unit… because the vehicle's **locking differential** counteracts wheel speed differences… controlling the individual rear-wheel brake torques during traction events would induce undesired yaw moments due to the locking differential."*
Throttle intervention is **binary**, the only one: `throttle_request = throttle_target * (max(all four slips) <= tc_max_slip_throttle_cut)`. The engine is otherwise deliberately untouched: *"This command is not influenced, since counteracting the engine via brake pressure modulation usually offers faster overall system responses if only engine airflow can be influenced."*

**Vehicle constants** (from `esc.cpp` declarations and `config/vehicle_handler/DummyVehicle/vehicle_config.yaml`): `cf` (front tire stiffness) **100000.0 N/rad**, `cr` **200000.0 N/rad** — **these are code defaults only and the keys are absent from the shipped YAML, so they are live**; `mass` **800.0 kg**; `yaw_inertia` **1000.0 kg·m²**; `wheelbase` **2.971 m** (= 7.4 ft, Indycar-scale); `lf` **1.724 m** → `lr` **1.247 m**; `trackwidth_front` (ESC) **1.606 m** but **1.639 / 1.524 m** in `slip_calculation.cpp` — **an inconsistency between two subsystems using "the same" track width.** Tire radius front/rear `0.293475 / 0.3074348 m`. Slip calculation is faded out below 5 m/s (`P_VDC_MinVelSlipCalc_mps: 5.0`, code default 3.0).

**Brake-bias scheduling** by speed, also in config:

```yaml
brake_bias_shift:
    # relative to current brake bias
    values: [-0.101, -0.0475, 0.00]
    vel_interp_mps: [20.0, 35.0, 60.0]
```
i.e. the brake bias is shifted **forward-biased at low speed and progressively less so at high speed**, interpolated on speed. Plus `P_VDC_brake_bias_front: 0.58`.

**WHAT THE SOURCE SAYS ABOUT TUNING.** The README states the architectural intent: *"This repository provides a vehicle stability-control system that can **safeguard an arbitrary motion controller** for real-world testing in autonomous racing."* The source comments are sparse; `// README:`-style caveats do not appear here. The countersteer's own comment is the design rule: `// Countersteer when rear sideslip dominates.` **Nothing is stated about oscillation causes in the steering path** — the oscillation concern is visible only in the ABS/TC threshold hysteresis and the look-ahead clip comments in the sibling controllers.

---

## 10. DonkeyCar `path_follow` — the clearest statement of the cross-track-error PID and its tuning procedure

- **Source:** https://docs.donkeycar.com/guide/path_follow/path_follow/

Used here because it is the most complete public *procedure* for tuning a cross-track-error steering PID, and because it states the error definition, the gain-vs-curve trade-off, and the sign-debugging step that every other source omits.

**CONTROLLED VARIABLE / ERROR SIGNAL.** **Signed cross-track error to a locally-fitted track line**, where the line is built from two waypoints around the nearest point:

> *"Choose the waypoint `PATH_LOOK_AHEAD` points ahead of the closest point on the path. Choose the waypoint `PATH_LOOK_BEHIND` points behind the closest point on the path. Use behind and ahead waypoints to create a line that represents the desired track. **Calculate the cross-track error between the vehicle's current position and the desired track. The cross-track error is a signed value that represents the distance from the line and which side of the line we are on.** Use the cross-track error as the error input into the PID controller that controls steering."*

Default look-ahead/behind, in points not metres — note this is a *different kind of look-ahead* from every other implementation in this survey (a **line-fit half-length in waypoint indices**, not an aim distance):

```python
PATH_SEARCH_LENGTH = None   # number of points to search for closest point, None to search entire path
PATH_LOOK_AHEAD = 1         # number of points ahead of the closest point to include in cte track
PATH_LOOK_BEHIND = 1        # number of points behind the closest point to include in cte track
```

**CONTROLLER TYPE.** PID on cross-track error, output = steering value.

**GAIN SCHEDULING.** None — the gains are constants in `myconfig.py`, changed only by the operator at runtime via `INC_PID_P_BTN` / `DEC_PID_P_BTN` / `INC_PID_D_BTN` / `DEC_PID_D_BTN`. **The doc explicitly recommends scheduling look-ahead by speed instead**:

> *"Generally, **if you are driving very fast you might want the look ahead to be larger than if driving slowly so that your steering can anticipate upcoming curves.**"*

**LOOK-AHEAD / PREVIEW.** As above, in waypoints. The trade-off is stated with its failure mode named as **understeer**:

> *"Increasing the length of the resulting track line, by increasing the look behind and/or look ahead, also acts as a **noise filter**; it smooths out the track. This reduces the amount of **jitter** in the controller. However, this must be balanced with the true curves in the path; **longer track segments effectively 'flatten' curves and so can result in understeer; not steering enough when on a curve.**"*

**SLIDE / DRIFT HANDLING.** **Source does not state.** There is no sideslip, yaw-rate or countersteer discussion anywhere in the path-follow documentation. Slide is implicitly handled by the low `PID_THROTTLE` constant and the recommendation to start in *Autosteering* mode: *"In **Autosteering** mode the car will try to follow the set of recorded waypoints, but it will only control steering; you still control throttle manually. This is a good mode to start in when following the path as you can safely stop the car by letting off the throttle. **It's also helpful in determining the maximum speed at which the car can reliably follow the waypoints.**"*

**LIMITS / ACTUATION.** Not stated numerically. Throttle is either a constant (`PID_THROTTLE`, with `USE_CONSTANT_THROTTLE = True`) or *"the throttle saved with the closest point on the path scaled by the `PID_THROTTLE` value"*.

**WHAT THE SOURCE SAYS ABOUT TUNING.** The most operationally specific tuning text in the survey:

- On P: *"**If this is too small then car will not turn enough when it reaches a curve. If this to too large then it will over-react to small changes in the path and may start turning in circles; especially when it gets to a curve.**"*
- On D: *"This parameter can be useful in reducing **oscillations and overshoot**."*
- On I: *"This may be useful in reducing offsets caused by accumulated error; **such as if one wheel is slightly smaller in diameter than another.**"* — a mechanical-asymmetry bias, not a curvature bias.
- The tuning procedure, verbatim: *"First determine the P coefficient. zero out the D and the I coefficients. Use a kind of 'binary' search to find a value where the vehicle will roughly follow a recorded straight line; probably oscillating around it. **It will be weaving like it is under the influence.** To do this, record a short straight line, maybe 6 meters... put the car in autopilot mode and stand in the middle of the line holding the car parallel to the line; the car's front wheels should stay stable and straight. Now slowly move the car off the line, keeping the car parallel to the line; the car should start to turn back towards the line. ... **If the car turns away from the line rather than towards the line then change the sign of the P value.** If the car turns very little then increase the P value. If the car turns very abruptly when off the line then reduce the P value."*
- And the failure of PID tuning generally: *"The PID coefficients are the most important (and time consuming) parameters to configure. **If they are not correct for your car then it will not follow the path.**"*

**Flag.** The doc calls this *"the Hello World of path following"* and it is exactly that — the sign convention of the cross-track error relative to the steering output is the only nontrivial part, and the doc handles it with a physical test rather than a derivation.

---

## 11. Habrador / Unity — the canonical cross-track-error PID tutorial, with the "drunk car" and "seasick" failure modes named

- **Source:** https://www.habrador.com/tutorials/pid-controller/1-car-follow-path/
- **Source code:** https://github.com/Habrador/Unity-Control-systems-Tutorial

**CONTROLLED VARIABLE / ERROR SIGNAL.** Cross-track error, **signed by a separate left/right test**:

```csharp
//Get the cross track error, which is what we want to minimize with the pid controller
float CTE = Math.GetCrossTrackError(steerPosition, previousWaypoint, currentWaypoint);
//But we still need a direction to steer
CTE *= Math.SteerDirection(transform, steerPosition, currentWaypoint);
float steeringAngle = PIDControllerScript.GetSteerFactorFromPIDController(CTE);
```

with

```csharp
public static float GetCrossTrackError(Vector3 carPos, Vector3 goingFromPos, Vector3 goingToPos)
{
    Vector3 a = carPos - goingFromPos;
    Vector3 b = goingToPos - goingFromPos;
    float progress = (a.x*b.x + a.y*b.y + a.z*b.z) / (b.x*b.x + b.y*b.y + b.z*b.z);
    Vector3 errorPos = goingFromPos + progress * b;
    float error = (errorPos - carPos).magnitude;
    return error;
}
```

Note the **sign is applied outside the error** by `SteerDirection`, which is computed from the **body `transform.right`** vector:

```csharp
Vector3 youDir = carTrans.right;
Vector3 waypointDir = waypointPos - steerPosition;
float dotProduct = Vector3.Dot(youDir, waypointDir);
float steerDirection = 0f;
if (dotProduct > 0f) steerDirection = 1f; else steerDirection = -1f;
```

So: **the cross-track magnitude is geometric; the sign reference is the car's BODY RIGHT vector.** The tutorial also exposes *where on the car* the error is measured as a tunable:

> *"So we can experiment with the position where the car is checking if it should steer left/right — doesn't have to be where the wheels are — especially if we are reversing."*

```csharp
steerPosition = transform.position + transform.forward * centerSteerDifference;
```

**CONTROLLER TYPE.** PID on cross-track error, with **the rolling-average and the derivative implemented distinctly**:

```csharp
//P
alpha = tau_P * CTE;
//I
CTE_sum += Time.fixedDeltaTime * CTE;
//Sometimes better to just sum the last errors
float averageAmount = 20f;
CTE_sum = CTE_sum + ((CTE - CTE_sum) / averageAmount);
alpha += tau_I * CTE_sum;
//D
float d_dt_CTE = (CTE - CTE_old) / Time.fixedDeltaTime;
alpha += tau_D * d_dt_CTE;
```

Note the integral is **not** `Σ e·dt` — it is a **leaky/rolling-average accumulator with a 20-frame time constant** (the same "rolling average" the Game AI Pro chapter recommends, §4). The derivative is a raw per-frame difference **with no filtering** despite the chapter's warning that this is exactly where noise bites.

**GAIN SCHEDULING.** None. `tau_P`, `tau_I`, `tau_D` are `public float` fields set in the inspector; the tutorial gives **no numeric defaults at all**. **Source does not state** any speed dependence.

**LOOK-AHEAD / PREVIEW.** There is none in the steering law — the error is measured **at the car**, against the line between the *previous* and *current* waypoint. The only forward-looking behaviour is waypoint switching:

```csharp
if (Math.HasPassedWaypoint(steerPosition, previousWaypoint, currentWaypoint)) { currentWaypointIndex += 1; ... }
```
using a **projection-progress test** (`progress > 1.0f` where `progress = (a·b)/|b|²`). The tutorial does not add distance-based preview.

**SLIDE / DRIFT HANDLING.** **Source does not state.** The only wheel-level machinery is a configuration recommendation:

```csharp
//To get a more realistic behavior
public Vector3 centerOfMassChange;
...
transform.GetComponent<Rigidbody>().centerOfMass = transform.GetComponent<Rigidbody>().centerOfMass + centerOfMassChange;
```
i.e. *"Move the center of mass"* — a physics-setup trick to change how the car rotates, not a controller.

**LIMITS / ACTUATION.** A **steering-angle clamp** plus an explicit **rate limiter implemented as an exponential average**, both quoted:

```csharp
float maxSteeringAngle = 40f;
...
//Limit the steering angle
steeringAngle = Mathf.Clamp(steeringAngle, -maxSteeringAngle, maxSteeringAngle);

//Average the steering angles to simulate the time it takes to turn the steering wheel
float averageAmount = 30f;
averageSteeringAngle = averageSteeringAngle + ((steeringAngle - averageSteeringAngle) / averageAmount);
```

The rate limit is explicitly justified as actuator realism: *"Average the steering angles to simulate the time it takes to turn the steering wheel."* `maxMotorTorque = 500f`, `maxSteeringAngle = 40f`.

**WHAT THE SOURCE SAYS ABOUT TUNING.** This tutorial's value is that it names the *symptoms* of the naive designs it replaces:

- On binary steering: *"This is working, but **the wheels will move really fast to the left or right when the car is driving straight towards the waypoint, which is not looking good.**"*
- On averaging instead of PID: *"To minimize this behavior, a good way is to take the rolling average of the steering angles, but **now the car will look like it's drunk.** So we need a better way, which is a PID controller."*
- Closing line: *"And that's it, your passengers in your self-driving car don't need to get **seasick** anymore!"*

**Numeric defaults: none.** This is the survey's clearest case of a widely-copied tutorial that supplies the *architecture* and *zero constants* — every implementation derived from it invents its own gains.

---

## 12. TORCS Simulated Car Racing Championship — a client-side PD on track angle + trackPos, with damping added after oscillation

- **Source (the code and the oscillation history, as a verbatim prompt/response log):** https://dev.to/dglezz/log-1-building-the-base-control-model-18om
- **Context (the competition and its API):** https://ns2.matf.bg.ac.rs/~vladaf/Courses/PmfBl%20I%20UVI/Materijali/Igre/Aktive/CarRacing/1304.1672v2.pdf

Included **as the survey's designated folklore specimen**, because it is a fully transparent record of an LLM-suggested controller being tuned by its symptom, and because the resulting code is representative of a large family of Python TORCS bots. The author labels the post *"a raw development log storing the exact prompts and responses used with the local LLM (IBM Granite-3.2-8B)"*.

**CONTROLLED VARIABLE / ERROR SIGNAL.** Two terms, both from the SCR sensor vector: `angle` (*"Angle between the car direction and the direction of the track axis"*) and `trackPos` (*"Distance between the car and the track axis... it is 0 when car is on the axis, -1 when the car is on the right edge of the track and +1 when it is on the left edge"*, per the sensor table). The law:

```python
R['steer'] = steer_kp * S['angle'] / math.pi - steer_kd * S['trackPos'] - 0.05 * R['steer']  # Added damping term
```

**This is not a PD controller**, despite the source's claim: `S['angle']` is a **heading error** (proportional), `S['trackPos']` is a **position error** (also proportional), and the third term is a **low-pass on the previous output** (a first-order lag / output filter, not a derivative). The variable named `steer_kd` multiplies a *position*, not a derivative. **Flag explicitly: the naming is wrong and the maths does not match the label.** The `- 0.05 * R['steer']` term is doing the damping job.

The reference for `angle` is the **track axis** (track tangent), not body-forward or velocity — the distinction matters because the car's own yaw is implicit in the sensor.

**CONTROLLER TYPE.** P (heading) + P (lateral position) + **output low-pass**. Gains, both versions, exactly as given:

```python
steer_kp = 30  # Proportional Gain for steering based on track angle
steer_kd = 0.20  # Derivative Gain for centering based on track position
```
then after the oscillation report:
```python
steer_kp = 15  # Proportional Gain for steering based on track angle (reduced from 30)
steer_kd = 0.15  # Derivative Gain for centering based on track position (adjusted to be less aggressive)
```

**GAIN SCHEDULING.** None. The gains are constants; the only speed-dependent behaviour is in the throttle.

**LOOK-AHEAD / PREVIEW.** **The steering law has none.** The *braking* side is given a look-ahead by scanning the `track` sensor array, first crudely:

```python
if len(track_sensors) > 1 and (track_sensors[-1] - track_sensors[0]) < -20:  # Check for upcoming sharp turn
    R['brake'] = 0.5
```
then as a "predictive look-ahead":

```python
lookahead_sensors = S['track'][-5:]  # Use last five sensors to anticipate track curvature change
max_distance_change = 20  # Threshold for detecting a sharp curve ahead
for i in range(1, len(lookahead_sensors)):
    if lookahead_sensors[i] - lookahead_sensors[i-1] < max_distance_change:
        distance_reduction = min(1.0, (max_distance_change - (lookahead_sensors[i] - lookahead_sensors[i-1])) / max_distance_change)
        R['brake'] = distance_reduction * 0.6  # Apply partial brakes gradually as a sharp turn is detected
```

**Flag.** The difference test `track_sensors[-1] - track_sensors[0] < -20` and then `lookahead_sensors[i] - lookahead_sensors[i-1] < 20` are **inconsistent in sign** (the first compares a difference of 20 against a negative threshold; the second treats *any* increase smaller than +20 as "a sharp curve"). The sensory indices are asserted, not explained: *"These sensors are chosen as they represent positions near the car's current and future trajectory, providing a window into what lies ahead"* — no mapping from sensor index to angle is given, and in TORCS's SCR `track` array the sensors run from −90° to +90°, so `[-5:]` and `[0]` are at **opposite ends of the sensor fan**, not a front-facing window. **This is folklore with no derivation and a probable indexing error.**

**SLIDE / DRIFT HANDLING.** A single hard-coded **braking** heuristic, and it is presented as slide prevention:

```python
brake_threshold = 0.9  # Braking threshold angle
if abs(S['angle']) > brake_threshold:
    R['brake'] = 1.0  # Apply full brakes for sharp turns
```

No countersteer, no sideslip.

**LIMITS / ACTUATION.** Only a final clip, and only on the throttle: `R['accel'] = max(0.0, min(1.0, R['accel']))`. **The steering output is not clamped in the published code** — a real hazard given `steer_kp = 30` and `angle` in `[-π, π]`.

**WHAT THE SOURCE SAYS ABOUT TUNING.** The oscillation story, verbatim, and it is a textbook case of P-too-large:

> *"The current `drive(c)` function has two major issues: **In straight sections, the car oscillates (zig-zags) excessively. This suggests that the proportional steering gain `steer_kp = 30` is too aggressive** or that we need a damping term. Please adjust the PD controller to be more stable in straights while maintaining responsiveness. The car crashes in the first corner because **the braking system is too reactive** (it only brakes at angle > 0.9)."*

and the fix:

> *"`steer_kp` was reduced from 30 to 15 to decrease the aggressiveness of the steering control, addressing the zig-zagging in straight sections. ... **A damping term `-0.05 * R['steer']` was added to counteract oscillations by dampening large changes in steering angle, promoting stability.**"*

Also of note, the throttle is **steering-dependent**, which is a recognisable idiom:

```python
target_speed = 250  # Target speed in km/h, adjust as needed
throttle_gain = 0.4  # Gain for acceleration based on steering angle
if S['speedX'] < target_speed - (R['steer'] * throttle_gain): R['accel'] += 0.2
else: R['accel'] -= 0.1
```
reduced to `throttle_gain = 0.3` in the second iteration.

---

## 13. Onieva et al., TORCS Car Racing Competition 2009 — fuzzy/parametric steering from 19 range-finder sensors

- **Paper (full text):** https://autopia.car.upm-csic.es/wp-content/papercite-data/pdf/onieva2009_modularparametricarchitecture.pdf
- **Record:** https://scienceportal.tecnalia.com/en/publications/a-modular-parametric-architecture-for-the-torcs-racing-engine/

A five-module architecture (gear, desired speed, low-level gas & brake, steering, opponents modifier) built for the *Simulated Car Racing Championship*, i.e. the client/server TORCS API where the controller only sees sensors. Included because its steering module is a **weighted-sensor blend** rather than a pursuit law — a genuinely different architecture.

**CONTROLLED VARIABLE / ERROR SIGNAL.** **Not a single error signal — a weighted average of directional range-finder readings, plus a frontal-distance correction factor.** Verbatim:

> *"The first one is when the car is inside the track axis and is not using the reverse gear. In this situation **9 Track sensors that go from -40 to 40 degrees are used. The output value is calculated as a weighted average from all the sensors' values**, additionally, **a correction factor F₀ which will depend on the value of the frontal sensor** is used. This factor will be useful, for example, to reduce the steering action in straight segments. We define **F₀ = 0.2 if T₀ = 100 and F₀ = 1 in other cases**. Equation 3 shows the calculation, where **wₜ, t = −40, ..., 40 represents the weight to apply to each sensor value**, it is used **wₜ = t/20**."*

The equation, as printed:

```
F_insideStraight(T_−40, …, T_40) = F₀ × ( Σ Tₜ·wₜ ) / ( Σ Tₜ )        (3)
```

So the steering command is a **sensor-weighted centroid whose weight is linear in the sensor's bearing angle** (`wₜ = t/20`, so the outermost sensors get weight 2.0 and the centre gets 0.0). The steady-state behaviour is: aim at the bearing that maximises `Tₜ·t` mass — i.e. **turn toward the side with more room, in proportion to how far off-axis that room is.** No explicit heading or lateral-error term exists.

**Correction factor:** `F₀ = 0.2` when the frontal sensor reads the maximum possible value (`T₀ = 100`) — i.e. **on a straight, steering authority is cut to 20%.** Otherwise `F₀ = 1`. This is a **discrete, sensor-triggered gain schedule**, the only one of its kind in the survey.

**CONTROLLER TYPE.** **Weighted-average (fuzzy-adjacent) controller**, not P/PD/PID/pursuit. The paper's own framing is a fuzzy system, but the *steering* module is the linear blend above; the TSK fuzzy system is used for the **target speed**, not the wheel:

> *"the allowed speed in a certain track segment is managed by a simple TSK fuzzy system"*, with consequents `TSᵢ = {200,175,150,125,100,75,50}, i=1...7` (in km/h, given `TargetSpeed` values up to `300`) and rules over `Front`, `Max₁₀`, `Max₂₀` with three trapezoidal membership functions `{Low, Medium, High}`. An exception: *"when Front = 100 (the maximum possible value) where TargetSpeed = 300 is used."*

**GAIN SCHEDULING.** Only the discrete `F₀` switch above. **No speed-dependent steering gain.**

**LOOK-AHEAD / PREVIEW.** **None for steering.** All sensor inputs are instantaneous bearings. The "look-ahead" is entirely in the speed module (using the ±10° and ±20° sensors to anticipate corners).

**SLIDE / DRIFT HANDLING.** **None in the steering module.** Grip is handled on the brake side by an ABS-shaped filter:

> *"the ABS filter function is implemented in order to avoid slips of the car when a brake signal is applied. The filter is implemented by reducing the brake signal (if AccelBrake<0) as showed in (2)."*

```
AccelBrake = AccelBrake − (speed − speed_wheels − 1.5)/5 ,  if (speed − speed_wheels) > 1.5      (2)
```
*i.e. brake is reduced in proportion to wheel-vs-vehicle speed error above a 1.5 m/s slip deadband, at a rate of 1/5 per unit slip.*

The stuck/reverse path is worth recording as a **state machine on angle and lateral position**, because it is the only source here that defines "stuck" numerically:

> *"We define a counter Stuck_time to record the number of consecutive game turns while the condition **(|angle| > π/6)** is satisfied. When **Stuck_time = 25**, it is considered that the car is stuck and reverse gear is applied until **(angle × trackPos > 0)** or **(front > 10 and |angle| < π/2)** is satisfied; in this moment the gear change from reverse to first gear."*

**LIMITS / ACTUATION.** The output is a normalised steer; the API maps it as *"steer [-1,1] Steering value: -1 and +1 means respectively full right and left, that corresponds to an angle of **0.785398 rad**"* (Table II). **No explicit clamp or rate limit is stated** — the weighted average is bounded by construction since it is a convex combination of bounded sensor values scaled by `F₀ ≤ 1`.

Gear-change constants are given (`GIᵢ = 8000` rpm for all upshift gears; `GDᵢ = {2500, 3000, 3000, 3500, 3500}` for downshifts from 2nd through 6th; *"a gear must to be maintained during, at least 20 time steps"*).

**WHAT THE SOURCE SAYS ABOUT TUNING.** The paper is candid about being hand-tuned and about what is *not* justified:

> *"As a first approach, we provide a **'hand-tuned' version of the controllers** that allow to achieve very good results... The modules are highly intuitive and these preliminary results **open the way to apply soft computing techniques to perform an automatic parameters adjustment for future competitions.**"*

and the design rationale, which is a statement of the modular philosophy:

> *"The main idea behind the architecture is to have **a small set of simple and interpretable modules whose interactions lead to a good driving**."*

**Flag.** `F₀ = 0.2 if T₀ = 100` is a **hard threshold on a sensor reading**, presented with the justification *"This factor will be useful, for example, to reduce the steering action in straight segments"* — no derivation, and it will chatter if `T₀` sits at 100. The paper does not discuss this.

---

## 14. GRID Autosport (Codemasters) — production practice: hand-authored lines and zones, offline-trained AI, no published law

- **Source:** https://www.pcgamesn.com/how-codemasters-teach-ai-drive (interview with James Nicholls and Clive Moody, Codemasters)

Included because it is the only credible public account of how a *shipped, commercially successful* racing game actually structures the steering problem, and because what it describes is **not a controller** in the control-theory sense. Every claim below is a direct quote.

**CONTROLLED VARIABLE / ERROR SIGNAL.** **A hand-authored racing line plus per-corner authored braking zones**, followed by a steel thread; the AI's steering authority is measured against track extents rather than against a computed geometry:

> *"When the level design guys build their tracks they **lay down the information the AI need to drive, they lay down the lines the cars will actually need to follow**, which are quite organic in their nature. They **lay down the braking zones for each corner**. They tune things like **track extents, so the map of the track that the AI could ever possibly use**, so, now, if there's a collision up front the AI can go around that collision, maybe go on the grass for a bit and then rejoin."*

**CONTROLLER TYPE.** Unspecified — and the source is explicit that the controller is **learned offline, not written**:

> *"We have a bespoke tool that we use that we've built and modified over many years that **actually trains the AI**. The AI system then runs with all that raw data, so the AI **runs lap after lap after lap at an accelerated super test, normally overnight using many different tracks and cars. It's driving, it's making mistakes, it's correcting them, it's training them up, it's improving them gradually.** That's how we end up at the trained AI data that you see in the game."*

**GAIN SCHEDULING.** Not addressed as a gain. What is scheduled instead is **driver personality** and **discipline behaviour**:

> *"we lay on top of that a set of information from each driver in the game. **Each driver has their own characteristics. How consistent they are, how aggressive they can be.**"*
> *"We also tune behaviours on a discipline basis... This is where you start to see unique behaviours such as... well, **the extreme examples would be how to drift a car**, but also things like in Touring Cars guys will protect the inside line and jostle for position on the track, whereas the opposite would be true in Open Wheel where they'll try to avoid contact."*

**LOOK-AHEAD / PREVIEW.** Not stated as a distance. The authored braking zones are the look-ahead proxy.

**SLIDE / DRIFT HANDLING.** Named as a per-discipline behaviour (*"how to drift a car"*) and left there. **Source does not state** a mechanism.

**LIMITS / ACTUATION.** Only the *track extents* map: *"the map of the track that the AI could ever possibly use."* No steering clamp or rate limit stated.

**WHAT THE SOURCE SAYS ABOUT TUNING.** Two statements worth keeping:

- On why not to follow splines, which is a direct criticism of the pure-pursuit family in this survey: *"**You can get cars to follow splines very easily by comparison but that's how you get that robotic inhuman AI behaviour.**"*
- On rubber-banding, i.e. on the acceptance criterion for a racing AI: *"if there's a pack of cars they will be bleeding time off each other because they're fighting for track position. There's other variables in there like AI making mistakes, either minor ones or proper off-track incidents... **If we did anything like the AI always slowed to a crawl when you were falling back it would shatter that illusion.**"*
- And the tuning-scale reality: *"we're pushing 80… 85 cars in this game and we've got over 100 routes. That's a hell of a lot of combinations of car and track... **The car handling team spend ages getting the car handling just right for every car.**"* — plus *"We have **dedicated guys on the team who do nothing but set the benchmark times for the AI**."*

**Flag.** This source contains **no formula, no constant, and no controller structure** — it is a human-interest feature, not an engineering account. It is included precisely to document that the most commercially successful approach in this survey is *authored data + offline learning*, and that this approach is not describable in the 8-field format. Anyone citing it for a steering law is over-reading it.

---

## 15. Production sim racing: what is and is not public (iRacing, Forza, GT Sophy, F1)

Recorded here so the survey's gaps are explicit rather than papered over.

**iRacing.** The AI is documented only through **driver-attribute UI descriptions**, not a controller. The one attribute that names the steering behaviour is: *"**Smoothness – A measure of the AI Driver's steering behavior.**"* (https://support.iracing.com/support/solutions/articles/31000153531-ai-rosters, https://www.iracing.com/airoster-/). Release notes confirm the AI is developed *"per discipline, per car ánd per track"* (community report, https://www.reddit.com/r/iRacing/comments/1v4ihvr/how_is_adaptive_ai_working_out_anyone_using_it_on/) and that AI tires are being brought *"closer aligned with exactly how a player-driver car functions"* (https://iracing.freshdesk.com/support/solutions/articles/31000177717-2026-season-1-initial-release-notes-2025-12-08-03-) — i.e. the historical difference between AI and player was partly in the **plant**, not the controller. **No steering law, error signal, gain schedule, look-ahead or clamp is public.** Developer updates are content-focused (https://www.iracing.com/iracing-development-update-august-2025/).

**Forza / Drivatar.** The published architecture is: a **neural network that predicts the deviation of a specific player** from an existing reference driving model, with the "AI controller" underneath:

> *"The AI controller is how the game uses throttle, brake and steering inputs to move a car around the track."* and *"Therefore, in Forza Motorsport we have evolved the Drivatar system to use machine learning."*
> (https://forza.net/news/forza-motorsport-drivatars-tire-physics)

And the layering, from a contemporaneous analysis: *"Drivatar will then **tweak that AI controller layer** to drive like a specific human's behaviour."* (https://www.gamedeveloper.com/design/how-forza-s-drivatar-actually-works). **This is the survey's cleanest statement of the "learned policy on top of a classical controller" architecture — but the classical layer's law is not public.** Wikipedia's summary of the RL paradigm describes it as tracking *"the player's car position and speed and the consistency of the behavior and guess their turn angle and speed for a given segment"* (https://en.wikipedia.org/wiki/Forza) — a **segment-keyed turn-angle lookup**, which is what a gains/look-ahead schedule looks like from outside. Forza Motorsport's Update 20 describes a move to *"a multi-line AI system trained on multiple trajectories"* (https://forzamotorsport.it/2025/04/30/modifiche-all-ia-dei-drivatar-in-arrivo-con-laggiornamento-20-di-forza-motorsport/).

**Gran Turismo Sophy.** Published: QR-SAC reinforcement learning, with *"agent-understandable encodings of the rules of racing"* and a complex reward function (https://www.gran-turismo.com/us/gran-turismo-sophy/technology/). The commercial pages **do not state the action space**. The Nature paper (*Outracing champion Gran Turismo drivers with deep reinforcement learning*, https://www.nature.com/articles/s41586-021-04357-7, PDF mirror https://www.cs.utexas.edu/~pstone/Papers/bib2html-links/nature22.pdf) is the primary source for `"The agent sends an action, a, for each car it controls to the game"` and for the continuous steering/throttle action space — **but that PDF could not be retrieved in this session (see Limitations), so its exact action-space bounds and control frequency are not reported here.** One derivative project records the practical detail that a similar GT-style agent is trained on *"steering sensitivity factors while using speed data for smooth control"* (https://www.theseus.fi/bitstream/10024/890966/2/Alali_Sammy.pdf) — **second-hand, not a Sophy statement.**

**F1 (Codemasters/EA).** The public record is *"AI is retrained whenever handling changes"* (https://www.thesixthaxis.com/2015/04/16/codemasters-steven-embling-on-rebuilding-f1-2015-from-the-ground-up/: *"Obviously, whenever we make any changes to the handling, **the AI has to be trained to drive to those new changes at the maximum of their ability**"*) and a denial of scripting (https://gamingbolt.com/f1-2010-codemasters-responds-to-issues: *"The AI system implemented in F1 2010 is very complex, and **is certainly not scripted in any way**"*). Also relevant: F1 23's *"Precision Drive"* is a **gamepad assist** replacing an older steering assist (https://www.reddit.com/r/F1Game/comments/13eq651) — i.e. the same steering-correction problem solved for human input. **No steering law is public.**

**Conclusion for this section:** for closed-source sims, the *only* reliably public lever is the **data pipeline** (authored lines, braking zones, track extents, offline training, player-imitation networks). None of the eight fields in this survey can be filled from public sources for iRacing, Forza, GT Sophy or F1.

---

## 16. Unreal Engine — the **RacingAI** plugin behind "Simple Racer with AI in Unreal Engine 5" (Christina Piberger/Charlier): a real PID on aim-heading error

- **Tutorial post:** https://chriscalation.com/posts/simple-racer/
- **Code:** https://github.com/ChrisVifzack/unreal-simple-racer — plugin `Plugins/RacingAI/`
- **PID law + defaults:** https://raw.githubusercontent.com/ChrisVifzack/unreal-simple-racer/main/Plugins/RacingAI/Source/RacingAI/Private/Utils/PIDController.cpp and `.../Public/Utils/PIDController.h`
- **Error signal + look-ahead:** https://raw.githubusercontent.com/ChrisVifzack/unreal-simple-racer/main/Plugins/RacingAI/Source/RacingAI/Private/Spline/SplineFollowComponent.cpp
- **Unit constants:** https://raw.githubusercontent.com/ChrisVifzack/unreal-simple-racer/main/Plugins/RacingAI/Source/RacingAI/Public/Utils/MathUtils.h

The blog post states only: *"Two PID controllers are used to control steering and speed respectively. (PID control works by injecting inputs to minimize the error between actual and desired state. This enables the AI to follow the spline.)"* — everything below is the actual C++.

**CONTROLLED VARIABLE / ERROR SIGNAL.** A **signed heading error between the car's BODY FORWARD vector and the car→aim-point direction**, both **projected onto the actor's up vector**:

```cpp
const auto OwnerDirection = GetOwner()->GetActorForwardVector();
const FVector OwnerUpVector = GetOwner()->GetActorUpVector();
const float Sign = FMath::Sign(FVector::DotProduct(FVector::CrossProduct(OwnerDirection, OwnerToTargetDirection), OwnerUpVector));

const float Angle = UMathUtils::GetSmallestAngleBetweenVectorsInDegrees(
    FVector::VectorPlaneProject(OwnerDirection, OwnerUpVector),
    FVector::VectorPlaneProject(OwnerToTargetDirection, OwnerUpVector));

LateralAngleAlignmentError = Sign * Angle;
```

**The reference vector is the body forward — not the velocity vector, not the spline tangent.** The spline enters only through the aim point. Sign comes from `dot(cross(forward, toTarget), up)`.

Notably, a lateral *location* error is computed but **not fed to the steering PID** — a latent cross-track term that is measured and discarded:

```cpp
const auto LocalOwnerToTarget = GetOwner()->GetActorTransform().InverseTransformVector(OwnerToClosestSplineLocation) * UMathUtils::Cm2Meter;
LateralLocationError = LocalOwnerToTarget.Y;
```
and only the angle error reaches the controller: `LateralCV = UPIDController::UpdatePID(LateralPIDSettings, LateralPIDState, LateralAngleAlignmentError, DeltaTime);`

**CONTROLLER TYPE.** Full parallel PID, textbook form:

```cpp
CV += PIDConfig.Kp * Error;
PIDState.AccumulatedError += Error * DeltaSeconds;
CV += PIDConfig.Ki * PIDState.AccumulatedError;
const float ErrorRate = (Error - PIDState.PreviousError) / DeltaSeconds;
CV += PIDConfig.Kd * ErrorRate;
PIDState.PreviousError = Error;
return FMath::Clamp(CV, PIDConfig.MinOutput, PIDConfig.MaxOutput);
```

Defaults: `Kp = 0.9f; Ki = 0.001f; Kd = 0.05f; MinOutput = -1.f; MaxOutput = 1.f;`. Note the integral is a **raw accumulator with no leak and no clamp** — the opposite of the Game AI Pro recommendation (§4) and of the DonkeyCar/Habrador rolling-average treatment (§10, §11). Ki is small enough (`0.001`) that this is probably survivable in practice, but the wind-up path is unguarded.

The complementary longitudinal PID **reuses the same class and is not implemented**: `ApplyInputs` is an empty virtual with the comment `// needs to be implemented specific to your pawn.`

**GAIN SCHEDULING.** **None on the gains** — `Kp/Ki/Kd` are fixed `UPROPERTY(EditAnywhere)` values. Speed enters only through the look-ahead distance.

**LOOK-AHEAD / PREVIEW.** Speed-scheduled aim distance:

```cpp
// the actual desired/target location should be a bit ahead of current spline param (for smoother steering). the look ahead distance is a function of velocity.
float TargetParam = CurrentSplineParam;
const float SpeedKmh = GetOwner()->GetVelocity().Size() * UMathUtils::Cms2Kmh;
const float LookAheadDistance = UKismetMathLibrary::MapRangeClamped(SpeedKmh, 30.f, 100.f, 2.f, 15.f);
const FVector TargetLocation = Spline->MoveParam(TargetParam, LookAheadDistance * UMathUtils::Meter2Cm);
```

Constants: `Cms2Kmh = 0.036f`, `Meter2Cm = 100.0f`. So **2 m at ≤30 km/h, 15 m at ≥100 km/h, linear between, and clamped outside the band** — floored *and* capped by `MapRangeClamped`. The only stated justification is the code comment *"for smoother steering"*. **No stability rule is stated.** Note the **mixed reference**: the *speed* for scheduling comes from `GetVelocity()`, while the *error* comes from the forward vector.

**SLIDE / DRIFT HANDLING.** **Source does not state.** No countersteer term, no sideslip term, no yaw-rate term, no grip term. The PID's derivative acts on the **heading-error rate**, which is at best indirect yaw damping. Critically, because the error is measured against body forward, a sliding car's error is measured against its **nose**, so the controller will not automatically unwind toward opposite lock — the same structural limitation as ARS's nose-based legacy cascade, and the reason ARS's pipeline notes insist on the velocity reference.

**LIMITS / ACTUATION.** The audit of the plugin finds **only one clamp**: `FMath::Clamp(CV, PIDConfig.MinOutput, PIDConfig.MaxOutput)` with defaults `-1 .. +1` — a **normalised steering input**, not an angle. **No steer-rate limit, and no speed-dependent maximum steer in the plugin.** Steering-angle clamping is left to the vehicle Blueprint (`BP_SR_BaseVehicle.uasset`, `BP_SR_Sedan.uasset` are binary and not readable as text, so what they do is unstated). The only guard is `ensure(DeltaSeconds > 0.f)` returning `0.f`.

**WHAT THE SOURCE SAYS ABOUT TUNING.** **Source does not state** any pitfall text, oscillation warning, or "works at low speed not high" note. The only tuning affordances are that `LateralPIDSettings` is `EditAnywhere` and `bDebugDraw` draws the orange closest-point sphere, the red target sphere and a red error arrow for on-screen tuning. **Verdict: real, clean, minimal PID — but it is a *heading-error* PID with a speed-scheduled pursuit point, no slide handling, and no tuning narrative at all.**

---

## 17. Unity Standard Assets `CarAIControl.cs` — the single most-copied Unity car-AI script, and a pure P with an unexplained gain

- **Source (full verbatim file):** https://raw.githubusercontent.com/KatVHarris/GravityInfiniteRunner/master/Unity/Assets/Sample%20Assets/Vehicles/Car/Scripts/CarAIControl.cs
- **Paired actuator, same repo:** https://raw.githubusercontent.com/KatVHarris/GravityInfiniteRunner/master/Unity/Assets/Sample%20Assets/Vehicles/Car/Scripts/CarController.cs
- **Forum copies of the same class:** https://discussions.unity.com/t/car-ai-help/679803 and https://discussions.unity.com/t/ai-car-script/869175 (the two vintages rename the fields — shipped build uses `steerSensitivity`, the forum copy `m_SteerSensitivity`; the shipped build is quoted here)

**CONTROLLED VARIABLE / ERROR SIGNAL.** The **angle from the car to the aim point in local body coordinates**:

```csharp
// calculate the local-relative position of the target, to steer towards
Vector3 localTarget = transform.InverseTransformPoint(offsetTargetPos);
// work out the local angle towards the target
float targetAngle = Mathf.Atan2( localTarget.x, localTarget.z ) * Mathf.Rad2Deg;
```

i.e. a **nose-referenced aim-angle error** (`InverseTransformPoint` is equivalent to body forward). The script *does* obtain the velocity vector, but uses it **only for the braking/caution calculation**, never for the steering error:

```csharp
Vector3 fwd = transform.forward;
if (rigidbody.velocity.magnitude > carController.MaxSpeed*0.1f)
{
    fwd = rigidbody.velocity;
}
```

**This is the exact inverse of ARS's choice** (ARS measures aim error from the velocity vector and falls back to nose below ~3 m/s; this script measures aim error from the nose always, and swaps to velocity only for the *speed-caution* term above 10 % of max speed). Both designs contain the same idea — "the velocity vector matters at speed, the nose matters when slow" — applied to different signals.

**CONTROLLER TYPE.** **Pure proportional, saturating, with a direction flip:**

```csharp
float steer = Mathf.Clamp ( targetAngle * steerSensitivity, -1, 1 )  * Mathf.Sign(carController.CurrentSpeed);
```

with `[SerializeField] float steerSensitivity = 0.05f;`. **No I and no D.** Accel/brake is a separate saturated P law with two gains chosen by direction:

```csharp
float accel = Mathf.Clamp((desiredSpeed-carController.CurrentSpeed)*accelBrakeSensitivity,-1,1);
```
with `accelSensitivity = 0.04f`, `brakeSensitivity = 1f` (i.e. **braking is 25× more aggressive per m/s of error than accelerating** — the opposite asymmetry from ARS's deliberately softer brake side, §Speed pipeline).

**GAIN SCHEDULING.** **None on the steering gain** — `0.05f` is fixed. Speed schedules the **speed target, not the gain**:

```csharp
desiredSpeed = Mathf.Lerp(carController.MaxSpeed, carController.MaxSpeed*cautiousSpeedFactor, cautiousnessRequired);
```
with `cautiousSpeedFactor = 0.05f` and `cautiousnessRequired = Mathf.InverseLerp(0, cautiousMaxAngle, Mathf.Max(spinningAngle, approachingCornerAngle))`, `cautiousMaxAngle = 50f`, `spinningAngle = rigidbody.angularVelocity.magnitude * cautiousAngularVelocityFactor`, `cautiousAngularVelocityFactor = 30f`. **So yaw rate does appear — but as a speed reduction, not a steering term.** That is the "oversteer/understeer → slow down" half of the ESC idiom with the countersteer half missing.

**LOOK-AHEAD / PREVIEW.** **There is no look-ahead distance at all.** The aim point is a target transform, laterally offset only by wandering or collision evasion:

```csharp
offsetTargetPos += target.right * (Mathf.PerlinNoise( Time.time * lateralWanderSpeed, randomPerlin )*2-1) * lateralWanderDistance;
```
`lateralWanderDistance = 3f`, `lateralWanderSpeed = 0.1f`. The "preview" is the classic hack of parenting an empty transform ahead of the car and aiming at it; braking triggers off the *target's own forward vector*: `float approachingCornerAngle = Vector3.Angle(target.forward,fwd);`. **No stability rule tying preview to stability is stated.**

**SLIDE / DRIFT HANDLING.** **None in the AI** — and what replaces it is collision response, not countersteer:

```csharp
// evasive action for 1 second
avoidOtherCarSlowdown = 0.5f ...
avoidPathOffset = lateralWanderDistance * -Mathf.Sign(otherCarAngle);
```
i.e. a collision-triggered lateral path offset. Yaw rate appears only in `spinningAngle` for the speed-caution term. **The nearest thing to countersteer lives in the actuator, not the AI**, as a *rate boost* under opposite lock:

```csharp
if (Mathf.Sign (steerInput) != Mathf.Sign (CurrentSteerAngle)) {
    currentSteerSpeed *= advanced.oppositeLockSteeringCorrection;
}
```
with the comment *"increase steering speed if steering input is in opposite direction to current wheel direction (for faster response)"* and `oppositeLockSteeringCorrection = 4f`. **This is a genuinely interesting design: rather than detecting a slide, it detects the *driver's intent to catch one* and gives the actuator 4× the slew rate to execute it.** It is the cheapest countersteer mechanism in the survey and it requires no sideslip estimate.

**LIMITS / ACTUATION.** Two layers.
AI side: `Mathf.Clamp(targetAngle * steerSensitivity, -1, 1)` — input space only.
Actuator side, and this is the canonical Unity speed-dependent steer limit:

```csharp
var currentSteerSpeed = Mathf.Lerp (steeringResponseSpeed, steeringResponseSpeed * maxSpeedSteerResponse, curvedSpeedFactor);
var currentMaxAngle = Mathf.Lerp (maxSteerAngle, maxSteerAngle * maxSpeedSteerAngle, curvedSpeedFactor);
...
CurrentSteerAngle = Mathf.MoveTowards (CurrentSteerAngle, steerInput * currentMaxAngle, Time.deltaTime * currentSteerSpeed);
```

Constants: `maxSteerAngle = 28`, `maxSpeedSteerAngle = 0.23f`, `steeringResponseSpeed = 200`, `maxSpeedSteerResponse = 0.5f`, `maxSpeed = 60` m/s. The speed normalisation uses a **quadratic bias**, not a linear one:

```
curvedSpeedFactor = CurveFactor(SpeedFactor),  CurveFactor(f) = 1 - (1 - f)*(1 - f)
SpeedFactor = Mathf.InverseLerp(0, reversing ? maxReversingSpeed : maxSpeed, Mathf.Abs(CurrentSpeed))
```

So max steer collapses to **23 % of 28° ≈ 6.4°** at top speed and the steer *rate* halves. `Mathf.MoveTowards` **is** the steer-rate limit. A third correction: `advanced.steeringCorrection = 2f` multiplies steer speed when `steerInput == 0` (*"How fast the steering returns to centre with no steering input"*).
**Why**: stated only by the field comments — `maxSpeedSteerAngle` is *"the reduction in steering angle at max speed"*, `maxSpeedSteerResponse` is *"the reduction in steer response at max speed."* No derivation.

**WHAT THE SOURCE SAYS ABOUT TUNING.** **Nothing normative.** The only "pitfall" text is mechanical: `lateralWanderDistance/Speed` and `accelWanderAmount/accelWanderSpeed` exist so that *"cars don't all wander in the same pattern"* and *"to give the cars a more human, less robotic feel"*, and `accelWanderAmount` *"can introduce jostling and bumps between AI cars in a race"*. **Verdict: the archetype of the folklore pattern.** The gain is an unexplained `0.05f`, the law is `angle × constant` with no derivation, and the tuning advice that circulates for this script is entirely "try numbers until it stops weaving". The interesting engineering is in the *actuator* (`CarController.cs`), not the AI.

---

## 18. Unity official "Create a car with Wheel colliders" tutorial — engine-blessed speed-dependent steer reduction

- **Source:** https://docs.unity3d.com/6000.0/Documentation/Manual/WheelColliderTutorial.html (mirror: https://docs.unity.com/en-us/engine/6000.7/manual/physics-section/physics-overview/collision-section/collider-shapes/wheel-colliders/wheel-collider-tutorial)

A *player* controller, but the canonical Unity statement of the speed/steer relationship that AI tutorials copy — and the engine's own recommended answer.

**CONTROLLED VARIABLE / ERROR SIGNAL.** **Source does not state** (no AI, no error signal). The steering variable is simply `hInput`, documented as *"Steering input"*.

**CONTROLLER TYPE.** None (open-loop player input) with one speed-scheduled scaling term.

**GAIN SCHEDULING.** The whole reason to cite it:

```csharp
// Calculate current speed along the car's forward axis
float forwardSpeed = Vector3.Dot(transform.forward, rigidBody.linearVelocity);
float speedFactor = Mathf.InverseLerp(0, maxSpeed, Mathf.Abs(forwardSpeed)); // Normalized speed factor

// Reduce motor torque and steering at high speeds for better handling
float currentMotorTorque = Mathf.Lerp(motorTorque, 0, speedFactor);
float currentSteerRange = Mathf.Lerp(steeringRange, steeringRangeAtMaxSpeed, speedFactor);
...
wheel.WheelCollider.steerAngle = hInput * currentSteerRange;
```

Defaults: `maxSpeed = 20f`, `steeringRange = 30f`, `steeringRangeAtMaxSpeed = 10f`. **Linear interpolation on normalised |forward speed|**, with the speed measure being the **projection of velocity onto the car's forward axis** (`Vector3.Dot(transform.forward, …)`) — the same projection ARS uses rather than raw velocity magnitude, and for the same reason (a car sliding sideways should not read as fast).

Stated reason, verbatim: *"Reduce motor torque and steering at high speeds for better handling."*

**LOOK-AHEAD / PREVIEW.** **Source does not state** (no preview exists).

**SLIDE / DRIFT HANDLING.** **Source does not state.** Nothing addresses oversteer, drifting or grip loss. The only stability affordance is a plant change: *"Adjust center of mass to improve stability and prevent rolling"*, `centreOfGravityOffset = -1f`.

**LIMITS / ACTUATION.** The speed-scheduled `currentSteerRange` is the only limit (30° → 10°, floored at 10° because `InverseLerp` clamps at 1). **No steer-rate limit** — the tutorial assigns `steerAngle` directly. **Why**: only the comment above.

**WHAT THE SOURCE SAYS ABOUT TUNING.** **Source does not state** any pitfall or oscillation guidance. The only tuning sentence is: *"Now that you have a basic setup, you can try changing different settings to observe how they affect the movement of the car."* **Verdict: authoritative provenance for "reduce steer angle with speed", but it is documentation of an actuator, not of a controller — one line of justification and no derivation.**

---

## 19. Unity `deye1986/AIVehicleRoutingBuddy` — waypoint AI with a units-aware gain, a deadzone, and a README that documents code it does not ship

- **Source:** https://github.com/deye1986/AIVehicleRoutingBuddy (MIT, David Ikin)
- **AI code:** https://raw.githubusercontent.com/deye1986/AIVehicleRoutingBuddy/main/AICarContoller.cs (filename misspelled in the repo)
- **README:** https://raw.githubusercontent.com/deye1986/AIVehicleRoutingBuddy/main/README.md

**CONTROLLED VARIABLE / ERROR SIGNAL.** Signed angle from the car's **BODY FORWARD** to the car→waypoint direction, both flattened to horizontal, with a **5° deadzone**:

```csharp
Vector3 targetDirection = (waypoints[currentWaypointIndex].position - transform.position).normalized;
targetDirection.y = 0;

Vector3 carForward = transform.forward;
carForward.y = 0;
carForward.Normalize();

float angleToTarget = Vector3.SignedAngle(carForward, targetDirection, Vector3.up);

if (Mathf.Abs(angleToTarget) < 5f)
    angleToTarget = 0f;

float targetSteerInput = Mathf.Clamp(angleToTarget / carControl.steeringRange, -1f, 1f);
targetSteerInput *= steeringPower;
```

The aim point is the **raw waypoint position** — no geometric aim construction. Note the normalisation: the error is divided by the car's own `steeringRange` (degrees per unit input) before the gain is applied, so **the gain is dimensionally a trim, not the whole controller** — unlike the Unity Standard Assets `0.05f`. The deadzone is the survey's only statement of one.

**CONTROLLER TYPE.** **P + deadzone + first-order smoothing:**

```csharp
currentSteerInput = Mathf.Lerp(currentSteerInput, targetSteerInput, Time.fixedDeltaTime * 5f);
return Mathf.Clamp(currentSteerInput, -1f, 1f);
```

No I, no D. The smoothing rate `5f` is hardcoded.

**GAIN SCHEDULING.** **None.** `public float steeringPower = 1.5f;`. Speed never multiplies or divides the steering law. The README claims a UI parameter `maxSteeringAngleDampener = 25` described as *"Reduces steering sensitivity on straights"* — **that parameter does not appear in the shipped code.** Any speed/straight-line gain scheduling is documented but not implemented. **This is a documentation/code drift worth flagging to anyone who adopts this project.**

**LOOK-AHEAD / PREVIEW.** **None on steering** — the aim point is the current waypoint and the car advances on `distanceToWaypoint < waypointReachDistance` (`5f`). The only preview is *corner* preview for speed: `public float cornerLookahead = 20f;` used as `float cornerProximity = 1f - Mathf.Clamp01(distanceToWaypoint / cornerLookahead);`. **No stability rule stated.**

**SLIDE / DRIFT HANDLING.** **None.** No yaw rate, no lateral velocity, no slip, no countersteer, no clamp-for-slide. The mitigation is recovery, not control — a **wiggle-reverse stuck routine**:

```csharp
float wiggleSteering = Mathf.Sin(reverseTimer * 3f) * 0.5f;
carControl.aiHorizontalInput = wiggleSteering;
carControl.aiVerticalInput = -1f;
```
with `stuckSpeedThreshold = 1f`, `stuckTimeThreshold = 3f`, `reverseTime = 2.5f`. The README names the gap itself under *"Known Limitations"*: *"Collision avoidance not implemented."*

**LIMITS / ACTUATION.** Input clamped `[-1, 1]` twice (angle/`steeringRange` clamp, then the smoothing clamp). The angle-range normalisation is the de-facto angle limit and the README requires the vehicle to expose `public float steeringRange; // Maximum steering angle`. **No steer-rate limit beyond the `Lerp`. Why: not stated.**

**WHAT THE SOURCE SAYS ABOUT TUNING.** This source *does* give verbatim per-scenario advice, which is itself the finding — **gain scheduling by track type, done by hand**:

- *"For Faster, More Aggressive AI: Increase `targetSpeed`; **Increase `steeringPower` (1.8-2.0)**; Increase `throttleStrength` (1.2-1.5); Reduce `cornerLookahead` (15-18)."*
- *"For Smoother, More Realistic AI: **Increase `steerSmoothingFactor` (4-6)**; Increase `cornerLookahead` (25-30); … **Lower `steeringPower` (1.0-1.3)**."*
- *"For High-Speed Oval Tracks: Increase `targetSpeed` significantly; **Reduce `steeringPower` (0.8-1.0)**; Increase `maxSteeringAngleDampener` (30-40); Minimal or no brake zones needed."*
- Pitfalls, verbatim: *"**Pitfalls** — IMPORTANT - ANGULAR DAMPING SHOULD BE SET TO 1.6 - 2.1. Reduce at own risk."*

That last line is the file's only stability statement, and it is a **rigid-body angular-drag constraint on the plant, not a controller statement** — a real, quotable example of a tutorial pushing stability out of the controller and into the physics settings. The "high-speed oval → reduce `steeringPower`" advice is the closest thing to gain scheduling anywhere in this source, and it is manual and per-track, not automatic.

Speed-derived corner braking is documented with hard constants: *"Sharp corners (>45°): Reduces speed to 50% of target; Moderate corners (>25°): Reduces speed to 70% of target; Gentle curves (<25°): Maintains full speed"*, matching `sharpCornerAngle = 45f`, `moderateCornerAngle = 25f`, `cornerSpeedMultiplier` values `0.5f`/`0.7f` in code.

**Verdict: better than most folklore — units-aware gain, deadzone, smoothing, documented per-scenario tuning — but zero slide handling, zero automatic gain scheduling, and a README that documents features the code doesn't have.**

---

## 20. Unity `NwliZz/Self-Driving-Bot` — arbitration + PID, with the lateral law unpublished and the gain scheduling pushed into the plant

- **Source:** https://github.com/NwliZz/Self-Driving-Bot — described as *"Unity-based self-driving car system - featuring road scanning, spline path planning, Pure Pursuit steering, PID speed control, and behavior arbitration for traffic lights and vehicle following."*
- **Code read:** https://raw.githubusercontent.com/NwliZz/Self-Driving-Bot/main/Assets/Scripts/Bot/Driver.cs and https://raw.githubusercontent.com/NwliZz/Self-Driving-Bot/main/Assets/Scripts/Bot/Mechanisms/PIDController.cs

**CONTROLLED VARIABLE / ERROR SIGNAL.** The architecture is arbitration-first: `Driver.FixedUpdate` evaluates three commands and picks by priority, then **overrides the winner's steering with the path command's**:

```csharp
ControlCommand chosen = allCommands.OrderByDescending(cmd => cmd.Priority).First();
chosen.SteeringAngle = pathCommand.SteeringAngle;
```

The path steering itself is computed in an `Actions.EvaluatePath()` that **is not present in the published tree**. The PID that *is* published is **longitudinal, not lateral**:

```csharp
public float UpdatePID(float setpoint, float measured, float deltaTime) {
    float error = setpoint - measured; ...
}
```
(that is, speed-setpoint-minus-speed). **Lateral error is not computed in any file present in the repo.** This is an access limit, reported as such rather than filled in by inference.

**CONTROLLER TYPE.** Classical parallel PID (speed) plus a geometric pure-pursuit steering path (steering computed elsewhere; only the interface is public). The PID:

```csharp
float error = setpoint - measured;
integral += error * deltaTime;
float derivative = (error - lastError) / deltaTime;
lastError = error;
return Kp * error + Ki * integral + Kd * derivative;
```

**Anti-windup exists but only as a priority-change reset, not a clamp**: `void ResetPIDControllers() { actions.pathSpeedPID.Reset(); }`, called from `HasChangedPriority(currentPriority)` *"if priority changes update uninformed Controllers"*. `PIDController.Reset()` zeroes `integral` and `lastError`. **No integral clamp is present** — an event-driven reset only, which is exactly the pattern Game AI Pro ch.40 describes as the *first* of two anti-windup measures (§4).

**GAIN SCHEDULING.** **The steering gain is scheduled by speed — in the plant, not in the controller**:

```csharp
float x = horizontalInput * (maxSteerAngle - (currSpeed / topSpeed) * steerAngleLimitingFactor);
float steerSpeed = steerSensitivity + (currSpeed / topSpeed) * speedDependencyFactor;
steerAngle = Mathf.SmoothStep (steerAngle, x, steerSpeed);
```

i.e. **max steer decreases linearly with normalised speed** (`maxSteerAngle − (v/v_max)·steerAngleLimitingFactor`) while the **steer response rate increases** with speed (`steerSensitivity + (v/v_max)·speedDependencyFactor`). **Note the sign flip versus Unity's Standard Assets (§17), which *reduces* response at speed** — the two most-copied Unity implementations schedule the response rate in opposite directions, and neither argues for its choice. No PID-gain scheduling is stated; `Kp/Ki/Kd` are plain public fields with no defaults in the published class.

**LOOK-AHEAD / PREVIEW.** **Source does not state** in the published files (`EvaluatePath`/`Actions` are absent; only `Driver.cs`, `PIDController.cs`, `Calculations.cs`, `CntrlCmnd.cs`, `Navigation/*`, `Scan.cs` are present). The README claims *"road scanning, spline path planning, Pure Pursuit steering"*, so a look-ahead almost certainly exists — **but its form and constants cannot be quoted.**

**SLIDE / DRIFT HANDLING.** Present, and unusual: a **velocity-vector correction keyed to yaw rate** rather than a countersteer term.

```csharp
void steerHelper ()
{
    localSteerHelper = Mathf.SmoothStep (localSteerHelper, _steerHelper * Mathf.Abs (horizontalInput), 0.1f);
    ...
    if (Mathf.Abs (oldRotation - transform.eulerAngles.y) < 10) {
        float turnAdjust = (transform.eulerAngles.y - oldRotation) * _steerHelper;
        Quaternion velRotation = Quaternion.AngleAxis (turnAdjust, Vector3.up);
        car.velocity = velRotation * car.velocity;
    }
    oldRotation = transform.eulerAngles.y;
}
```

**The whole velocity vector is rotated by the car's frame-to-frame yaw change, scaled by `_steerHelper × |steer input|`, gated to <10° of yaw per frame.** This is not measuring error from the velocity vector (ARS's approach) — it is *correcting the velocity vector itself* toward the nose. Functionally it suppresses the sideways component that a slide produces, which is why it doubles as drift handling. There is also a **yaw-acceleration stabiliser**:

```csharp
void rotationalStabilizer ()
{
    calcAngularAccl ();
    float reverseTorque = -1 * Mathf.Abs (angularAcclY) * revTorquePower * Mathf.Sign (car.angularVelocity.y) * (currSpeed / topSpeed);
    car.AddRelativeTorque (transform.up * reverseTorque);
}
```
with `angularAcclY = (prevAngularVelocity - car.angularVelocity.y) / Time.deltaTime` — a **PD-in-yaw-acceleration stabiliser scaled by normalised speed**, i.e. a derivative term applied in the *plant* rather than the controller.

Grip loss is otherwise addressed in the tire model, not the controller: `adjustSidewaysFriction` raises sideways friction with lateral slip — `driftX = Mathf.Abs (transform.InverseTransformVector (car.velocity).x); float driftFactor = driftX * driftVelocityFactor;` — **the more sideways the car is, the *more* lateral grip it gets** (an anti-drift curve).

**LIMITS / ACTUATION.** `maxSteerAngle`, reduced by speed as in GAIN SCHEDULING. **No explicit `Mathf.Clamp` on `steerAngle` in `steerCar`** — the speed term does the limiting and `Mathf.SmoothStep` provides rate limiting. Traction control is a separate decrement loop: `if (forwardSlip >= slipLimit && currentTorque >= 0) { currentTorque -= 1000 * traction; }` with `slipLimit` and `traction` public fields. **Why for the speed-dependent limit: not stated.**

**WHAT THE SOURCE SAYS ABOUT TUNING.** **Source does not state** any tuning narrative; no README tuning section in the fetched material. **Verdict: the most "autonomous-driving-shaped" Unity project found — arbitration + a PID class + a velocity-vector correction + a yaw stabiliser — but the lateral controller itself is not published, so its law and constants cannot be quoted.**

---

## 21. Unreal `HappySapeta/TrafficAI` — kinematic bicycle + IDM, and the "delete the slide" answer to drift

- **Source:** https://github.com/HappySapeta/TrafficAI (MIT)
- **Code:** https://raw.githubusercontent.com/HappySapeta/TrafficAI/main/Source/TrafficAI/Simulation/TrSimulationSystem.cpp

**CONTROLLED VARIABLE / ERROR SIGNAL.** The **signed planar angle between the car's simulated heading and a blended goal direction**, computed as a 2D cross/dot `atan2` with an explicit **`− heading·0.9` bias**:

```cpp
const FVector GoalDirection = (Goals[Index] - Positions[Index]).GetSafeNormal();
...
const FVector TargetHeading = (GoalDirection - CurrentHeading * 0.9f).GetSafeNormal();
const float TargetSteerAngle = FMath::Atan2
(
    CurrentHeading.X * TargetHeading.Y - CurrentHeading.Y * TargetHeading.X,
    CurrentHeading.X * TargetHeading.X + CurrentHeading.Y * TargetHeading.Y
);
```

Reference vector: **the simulated body heading**. The `− CurrentHeading * 0.9f` term is a constant pull-back on the current heading that shapes the response (a fixed understeer bias); **the source does not justify it.** The goal is a path point, not a tangent:

```cpp
const FVector Future = Positions[Index] + Velocities[Index].GetSafeNormal() * PathFollowingConfig.LookAheadDistance;
...
const FVector PositionOnPath = ProjectPointOnPathClamped(Positions[Index], OffsetPath);
const float Distance = FVector::Distance(Positions[Index], PositionOnPath);
if (Distance < PathFollowingConfig.PathFollowThreshold) { Goals[Index] = OffsetPath.End; PathFollowingStates[Index] = true; }
else { Goals[Index] = FutureOnPath; PathFollowingStates[Index] = false; }
```

Note the **velocity-vector projection for the future point** (predictive, not purely geometric) and a **path-offset lane bias**: `const FVector PathLeft = PathDirection.RotateAngleAxis(-90.0f, FVector::UpVector); const FVector PathOffset = PathLeft * PathFollowingConfig.PathFollowOffset;` applied to both path ends.

**CONTROLLER TYPE.** **P with geometric (arctangent) error shaping, saturating, driving a kinematic bicycle:**

```cpp
float SteerAngle = FMath::Clamp(TargetSteerAngle * VehicleConfig.SteeringSpeed, -VehicleConfig.MaxSteeringAngle, VehicleConfig.MaxSteeringAngle);
```

i.e. `θ_steer = clamp(gain · atan2_error, ±maxSteer)`. The plant is then pure geometry:
```cpp
FrontWheelPosition += CurrentVelocity.Length() * CurrentHeading.RotateAngleAxis(FMath::RadiansToDegrees(SteerAngle), FVector::UpVector) * TickRate;
CurrentHeading = (FrontWheelPosition - RearWheelPosition).GetSafeNormal();
```
— the true kinematic bicycle. Longitudinal is full IDM.

**GAIN SCHEDULING.** **Source does not state** for steering (`SteeringSpeed` is a config constant; the clamp is speed-independent). Speed scheduling exists only on the IDM longitudinal side through `VehicleConfig.DesiredSpeed`.

**LOOK-AHEAD / PREVIEW.** `PathFollowingConfig.LookAheadDistance`, a config constant applied **along the velocity direction**, with **no speed proportionality** as published. `GoalUpdateDistance` gates path renewal; `PathFollowThreshold` switches the goal from "future point" to "path end". **No stability rule is stated.**

**SLIDE / DRIFT HANDLING.** **None — and structurally so.** The steering consumes no slip, no lateral acceleration and no yaw rate, and the plant **rewrites the heading from wheelbase geometry every tick**, then re-aligns velocity to heading: `CurrentVelocity = CurrentHeading * CurrentVelocity.Length();`. **Any true drift is physically impossible in this model, so there is nothing to countersteer.** That is the "**kinematic substitution**" answer to slide handling: instead of controlling a slide, remove it. Worth recording as a distinct strategy. (The IDM side reacts to other cars, not to grip: `const float InteractionTerm = -VehicleConfig.MaximumAcceleration * FMath::Square(GapTerm); float Acceleration = FreeRoadTerm + InteractionTerm;`)

**LIMITS / ACTUATION.** `FMath::Clamp(..., -VehicleConfig.MaxSteeringAngle, VehicleConfig.MaxSteeringAngle)` — a **speed-independent** angle clamp, applied inside the orientation update so the bicycle model never sees more than the max angle. **No steer-rate limit.** Accel clamp: `FMath::Clamp(Acceleration, -VehicleConfig.ComfortableBrakingDeceleration * 2.0f, VehicleConfig.MaximumAcceleration);`. A documented *sensing*-range rationale exists but is not about steering: `constexpr float DETECTION_RANGE_SCALE = 2.0f; // Values smaller than 2 would result in failure to detect other vehicles properly.`

**WHAT THE SOURCE SAYS ABOUT TUNING.** The only tuning text is code comments (`constexpr float AMBER_DURATION = 5.0f; // This duration is used for the timer that switches the signal state from green to amber.` and the `DETECTION_RANGE_SCALE` note). **No oscillation or low-vs-high-speed guidance.** **Verdict: a clean, honest geometric implementation — the only source in this set whose steering limit is *not* speed-scheduled, and the only one that sidesteps slide handling by construction rather than by omission.**

---

## 22. Unreal "Realistic Vehicle AI" (Dragon Li Software) — a commercial cascaded PID whose documentation exists but is not machine-readable

- **Epic forum product thread:** https://forums.unrealengine.com/t/dragon-li-software-ltd-realistic-vehicle-ai/2583009
- **Fab listing:** https://www.fab.com/listings/31c40245-ab61-4b73-90ef-5682779d52ff
- **Manual:** *RealisticVehicleAiManual.pdf* v1.1.0 (52 pp., 2023) via https://drive.google.com/file/d/1bHc4AUgxHemFD8RbFfloS3157fwerVJp/view

**Access limitation, stated honestly:** the PDF body is rendered by the Drive viewer and could not be text-extracted. What was retrieved is the **table of contents, pages 1–3 body text, and captions**. Everything below is what is verified; the rest is marked unavailable rather than paraphrased.

**CONTROLLED VARIABLE / ERROR SIGNAL.** Forum text: *"It controls vehicles only through throttle, brake, steer and handbrake, just same as human players. No data cheating will be done."* The manual's tuning section is `1.1.4 Tuning PID parameters` with subsections `Calibration Lab Level`, `PID Visualization and Tuning UI`, `Additional features comparing to standard PID`, `General Tuning Guidance`. **The exact error definition is not retrievable — source does not state (in accessible text).**

**CONTROLLER TYPE.** Forum, verbatim: *"provides a PID(Proportional band, integral and derivative)-based driving solution"* … *"Its **cascaded PID** system provides a flexible and agile control interface."* … *"As the cascaded PID system provides a flexible control interface, many different work modes (or 'strategy') can be implemented above it. … You can write your own customized 'strategy' by overriding the CustomStrategy function."* The manual does contain `2.1 Core Control Process` and `2.2 Modules and Source Code Structure` (verified in the TOC) — that is where the cascade topology is defined, and it is not in the extractable text.

**GAIN SCHEDULING.** The manual lists a full section titled **`Additional features comparing to standard PID`** (p. 9) plus **`General Tuning Guidance`** (p. 10). **This is the strongest signal in the whole survey that a shipped Unreal racing-AI product treats "standard PID is not enough" as a first-class topic** — but the content is behind the viewer. **Source does not state (in accessible text).**

**LOOK-AHEAD / PREVIEW.** Not accessible. The plugin includes `1.2.1 Create Track` / `Set Track by Landscape Spline` / `Set Track by Actors with USplineComponent` and a *"track-planning actor which can generate racing-line based on various standards"* (forum quote), implying a **generated** rather than hand-placed racing line — but the preview rule is not stated.

**SLIDE / DRIFT HANDLING.** Forum only: *"It can drive vehicles to run on racing-lines, overtake opponent vehicles, and try to avoid collisions."* The presence of a **`handbrake` actuation channel** is the only hint that slides are modelled at all. The manual has `1.1.5 Test and set physical parameters` with `About the Steer Test` and `About the Brake Test` subsections — the vendor's calibration procedure is a **per-vehicle steer/brake test bench.** Explicit slide-handling policy: **source does not state (in accessible text).**

**LIMITS / ACTUATION.** Not accessible beyond the actuation-channel statement, quoted from manual p.3: *"RVA controls vehicles through steer-throttle-brake-handbrake-gear, just like human player, so **the vehicles' physical features also influence RVA**."* That sentence is the reason the vendor ships a **Calibration Lab level** and a **PID Visualization and Tuning UI** — an unusual, quotable admission that the controller cannot be separated from the vehicle's physics.

**WHAT THE SOURCE SAYS ABOUT TUNING.** Verbatim from the forum: *"To integrate to physically customized vehicles, you need to have some basic knowledge about PID tuning."* From the manual: the workflow is *"Create and tune a vehicle actor class … Tuning its physical parameters until it satisfies the needs of your game and is good for human player"* — i.e. **tune the car for a human first, then let the PID follow** — followed by `Calibration Lab Level`, `PID Visualization and Tuning UI`, `Additional features comparing to standard PID`, `General Tuning Guidance`, then `Test and set physical parameters` and `Write Configuration to uasset`. Appendix items retrieved from the TOC are directly relevant: **"1. Does this plugin drive vehicles in the best ways from the viewpoint of mathematics? Is it possible to drive faster and better?"**, **"3 (Known Issue - with walk-around solution) Velocity drop to 0 when being culled"**, **"4 (Known Issue) Physics simulation give smaller acceleration when tick delta-time is long"**.

**Verdict: a documented *claim* of a cascaded PID with non-standard extras and a formal tuning methodology. The constants and the law could not be verified, and they are not guessed here. Anyone who needs this one must open the PDF in a real reader.**

---

## 23. ML-Agents racers and the tutorial long tail — the negative findings, which are findings

**23a. Unity ML-Agents racing: the policy *replaces* the steering controller.** `Tinker-Twins/AutoRACE-Simulator` (https://github.com/Tinker-Twins/AutoRACE-Simulator), `Assets/Scripts/AIAgent.cs`:

```csharp
public override void OnActionReceived(float[] vectorAction)
{
    // DISCRETE ACTION SPACE
    var SteerAction = Mathf.FloorToInt(vectorAction[0]);
    switch (SteerAction) { case 0: SteeringCommand = 0; break; case 1: SteeringCommand = -1; break; case 2: SteeringCommand = 1; break; }
```

Observations are a single float: `sensor.AddObservation(carController.currSpeed);`. The only "controller" text is the manual heuristic (A/D → 1/2). The car controller it drives *does* have a steering-angle abstraction — `[HideInInspector]public float turnRange = 4f;` and `float turnAngle;` with `turnAngle = Mathf.Atan2 (localFlatForward.x, localFlatForward.z) * turnCheckSense;` (`turnCheckSense = 10000`) — but it is used **only for a turning *flag*** (`isTurning()`), never for steering. **Fields 2–8: there is no classical controller to extract, and the project states nothing about slide handling, look-ahead, or gain scheduling.** The same pattern holds for `tsukuri0/Kart-Racing`, `alonshoa/Karting` (Unity's Karting template + PPO) and `maxiwoj/car_racer_ml_agents` — all take a `steerInput = Mathf.Clamp(vectorAction[2], -1f, 1f)`-style action and hand it straight to the wheel colliders (https://discussions.unity.com/t/racing-simulator-ml-agents/808611). **This is the canonical "policy *is* the controller" case, and it is the useful contrast against a hybrid design where a classical controller sits under the policy.**

**23b. The most-requested and least-specified pattern: waypoint-angle → steering with an unexplained constant.** The Unity forums are full of it. One posted solution is literally a two-parameter curve fit with no derivation: `steer = 0.0235149f * Angle + 0.0517327f`, produced by an online function-equation finder (https://www.reddit.com/r/Unity3D/comments/l7dhmw/). The Unity Discussions thread on steer sensitivity produces the archetypal folklore answers, including `SteerAngle = Input.GetAxis("Horizontal")*1/rigidBody.velocity` — criticised **in the thread itself** as dimensionally wrong: *"rigidbody.velocity is Vector3, Axis input is a float … this can't work"* — and the divide-by-magnitude variant `myWheelCollider.steerAngle = Input.GetAxis("Horizontal") * maxSteerAngle * 1 / rv;` with an explicit *"but I've not tested this - so don't blame me"* (https://discussions.unity.com/t/how-do-i-correct-the-sensitivity-of-steerangle/9305). **Verdict: divide-by-speed is community folklore with no derivation and acknowledged dimensional sloppiness. The engine's own answer (§18) is a `Lerp` between two angles, not a division.**

**23c. Neither engine ships a path-following steering controller.** Unreal's own documentation only wires input events to `Set Throttle Input` / `Set Brake Input` / `Set Steering Input` (https://dev.epicgames.com/documentation/unreal-engine/how-to-set-up-vehicles-in-unreal-engine), and the API reference for `UChaosVehicleMovementComponent` exposes `GetSteeringInput()` with the note *"Get the user input for the vehicle steering - can use this to feed control to a connected trailer"* (https://dev.epicgames.com/documentation/unreal-engine/API/Plugins/ChaosVehicles/UChaosVehicleMovementComponent). Unity's equivalent is `WheelCollider.steerAngle` plus the tutorial in §18. **Both engines ship the *actuator* and leave the path-following controller entirely to the developer — which is precisely why the tutorial space is dominated by `angle × constant`.**

**23d. Unreal tutorial sources located but not text-extractable (documented, not guessed).**
- **jourverse's "Unreal Engine AI Vehicle Tutorial" series** — the largest UE5 vehicle-AI tutorial corpus found (21+ parts): index at https://forums.unrealengine.com/t/community-tutorial-unreal-engine-ai-vehicle-tutorial-6-advanced-move-to-target-location/1966897; [Tutorial 1: Spline Path Following](https://www.youtube.com/watch?v=HNpA9ArbZok) has chapters *"Calculate Steering Input: (14:29) / Control Throttle of Vehicle: (21:17) / Calculate Brake Input: (26:19)"*; [Tutorial 11](https://www.patreon.com/posts/project-file-ai-113373774) is described as *"we will enhance vehicle speed control by calculating the **curvature of the spline path**"* — the same primitive as the `√(g·grip·radius)` route-speed term. The Epic dev-community articles are JavaScript-rendered and returned no body text; Blueprint logic is not text.
- **DriveSim (UE5 vehicle-physics plugin)** — its help site states: *"This class manages AI controller to follow a spline on a map. This AI Controller interact with ADriveSimVehicle to drive the car and adhust input command s (steering, throttle, brake...) according spline curves and vehicle speed"* (https://sites.google.com/view/drivesim-ue5). That sentence is the entire accessible steering description: **speed is named as an input to the input-command computation, but the law, gains, look-ahead and limits are not published.**
- **Racing AI plugin (gfx-station/Fab)** — feature list only: *"Calculate throttle, brake, and steering … Driver profile classes Rookie, Advanced, and Pro with different AI/NPC steering accuracy and throttle intensity"* (https://gfx-station.com/racing-ai/). Relevant as an example of **difficulty implemented as per-profile steering inaccuracy**, not as gain scheduling.

---

## 24. Game AI Pro, Chapter 39 — "Representing and Driving a Race Track for AI Controlled Vehicles" (Tomlinson & Melder): the canonical AAA statement of *why* preview exists

- **Source:** https://www.gameaipro.com/GameAIPro/GameAIPro_Chapter39_Representing_and_Driving_a_Race_Track_for_AI_Controlled_Vehicles.pdf

This is the survey's most important **game-industry** source after ch.40, because it is the only one that explains the aim-point architecture and its failure modes in engineering prose.

**CONTROLLED VARIABLE / ERROR SIGNAL.** Exact:

> *"it is better to steer towards an **aiming point** some distance ahead of the vehicle, with **steering based on the angle between the vehicle's current direction and the vector between the car center and the aiming point**."*

**Reference vector: the car's own current direction (body heading). Error: the angle to the car→aim-point vector.** Explicitly **not** a track tangent and **not** a cross-track offset. This is the same law as TORCS (§2/§3), Unity's CarAIControl (§17) and the Unreal RacingAI plugin (§16) — **the aim-point angle is the universal game-industry error signal.**

**CONTROLLER TYPE.** Geometric aim-point steering (*"steer the Line Using a Look-Ahead or Runner"*) **plus a PID layer mixed in as a correction**:

> *"This kind of short-range adjustment works particularly well with a **PID-based control layer** [Melder and Tomlinson 13], **when mixed into the main steering target as a correction**."*

Note the architecture: **the aim-point term is the authority and the PID is the trim** — the same relationship as the Ziggy Racer design (§6), and the inverse of a naive "PID on cross-track error" reading of ch.40.

**GAIN SCHEDULING.** **Source does not state** any gain schedule. The *look-ahead* is what is scheduled.

**LOOK-AHEAD / PREVIEW — the survey's canonical statement of the failure mode and the reason for scheduling.** Verbatim, and it should be quoted in full wherever preview is discussed:

> *"If the steering is calculated, based on the racing line immediately in front of the car's track registration position, **the AI will tend to make a lot of small corrections, which results in weaving left and right across the line. In the worst case, this can build up, become noticeable to the user, and ultimately cause the vehicle to spin out.** Instead, it is better to steer towards an aiming point some distance ahead of the vehicle… This tends to smooth out the steering. However, it can lead to unusual behavior; **on a sharp corner, a larger look-ahead will make the AI cut across the inside of the corner. To counter this, the look-ahead distance should be related to the track curvature and/or the current speed; the exact formulation tends to be a matter of trial and error.**"*

Two things are notable: the mechanism is named (**look-ahead too short → weaving → build-up → spin**), and the remedy is explicitly admitted to be empirical (*"a matter of trial and error"*). **No formula is given** — which is exactly why TORCS's `17.0 + 0.33·v` (§2/§3) is the citable numeric instance and this chapter is the citable *reason*.

The **runner** mechanism (the same idea as ch.40's "runner", §4) is stated operationally:

> *"On every frame, the runner is updated along the racing line by a distance equal to the current speed of the vehicle, with a correction for any re-evaluation of the look-ahead distance."*

That is a **time-parameterised aim point** (`d_runner += v·dt`) rather than a re-searched distance each frame — a genuinely different implementation choice from every "search along the spline for lookahead metres" in this survey, and it makes the aim point a stateful integrator rather than a stateless lookup.

**SLIDE / DRIFT HANDLING.** **No countersteer controller.** What is stated is a **spare-grip-aware authority rule**, which is the closest thing to a principled oversteer policy in the game-industry sources:

> *"any turning should be 'under control,' that is, within grip limits. Furthermore, **the corrective action should be based on available spare grip; if extra steering would force the vehicle beyond the grip limit then braking might be preferred.**"*

And a deliberate non-hard boundary, which is a design decision worth recording:

> *"the reaction might be less severe, thus allowing the car to run off and recover rather than risk a spin."*

**LIMITS / ACTUATION.** **Source does not state** an explicit steering clamp or rate limit. Steering lock appears only as a **corner-speed factor**: *"there are other factors—the vehicle suspension, weight distribution, down force, and even **steering lock** can play a roll [sic]."* A **lateral wall-buffer steering correction** is described (the same idea as ARS's `ApplyRivalWalls`): *"it is also a good idea to look directly sideways as well as ahead; that is, make small corrections based on the proximity to any wall alongside the car in order to maintain a safe buffer distance."*

Corner speed is given as physics: `mv²/r ≤ G_max` (Eq. 39.4), with radius `r = d/sin(θ)` (Eq. 39.3).

**WHAT THE SOURCE SAYS ABOUT TUNING.** Beyond the look-ahead passage above:
- On track data: *"sudden changes in length between adjacent segments will lead to difficulties with the tangent/normal evaluations."*
- On the intended use of the line: *"the AI uses the racing line as a **guide only**."*

---

## 25. Game AI Pro 2, Chapter 18 — "Context Steering" (Andrew Fray, **F1 2011**): the lane arbiter that F1 used, and the explicit delegation of the steering law

- **Source:** https://www.gameaipro.com/GameAIPro2/GameAIPro2_Chapter18_Context_Steering_Behavior-Driven_Steering_at_the_Macro_Scale.pdf
- **GDC 2013 talk by the same author:** https://gdcvault.com/play/1018262/The-Next-Vector-Improvements-in (*"In another example, we will present the method used in a AAA racing game, which fixed some of the common problem that occur with traditional steering algorithms"*)
- **Author's blog on the technique:** https://andrewfray.wordpress.com/2013/03/26/context-behaviours-know-how-to-share/
- **Author's page confirming provenance:** https://andrewfray.dev/publicspeaking.html (*"Andrew developed Context Steering for F1 2011"*)

**FLAG FIRST: this is a lane/decision arbiter, not a steering controller, and the chapter says so.** The F1 2011 steering loop is explicitly delegated:

> *"a **low-level driver system followed a hand-placed racing line spline**"*

**CONTROLLED VARIABLE / ERROR SIGNAL.** **Not an angle error — a scalar lane offset against the racing line:**

> *"the behavior system only needed to manage **position on the track**, rather than driving. This was done with **a scalar left or right offset from the racing line**."*

The context map *"scales with the width of the track, with the left and right edges of the map lining up with the track edges."* **Reference geometry: the racing-line spline, expressed as lateral offset** — the same primitive as Game AI Pro ch.41's heat line (§5) and as ARS's lane systems. **Three independent sources converge on "the steering controller receives a lane offset, not a desired heading."**

**CONTROLLER TYPE.** **Search/arbitration over a one-dimensional context map** — not P/PD/PID/pursuit:

> *"we traverse the danger map to find the lowest danger and mask out all slots that have higher danger… apply it to the interest map… pick the interest map slot with the highest remaining interest"*

F1's variant, which is a **boundary-walk rather than an argmin**:

> *"First, we find the slot of the danger map corresponding to the car's current position… **walk left and right along the map, continuing as long as the danger in the next slot is less than the current.**"*

Sub-slot resolution is interpolated:

> *"we can evaluate the **gradients of the interest around it and estimate where those gradients would have met**… back-project this virtual slot index into world space"*

**GAIN SCHEDULING.** **Source does not state** (no speed/heading gain exists — the output is a lane index). Behaviour magnitude is the only scaling: *"The speed we move is proportional to the strength of interest in the slot."* The racing-line behaviour is deliberately weak so that arbitration can override it: *"write the **most interest at the racing line, but never very much**… it always has an idea of which way is closer to the racing line and can tuck in tightly."*

**LOOK-AHEAD / PREVIEW.** **Source does not state** any steering preview distance — preview lives in the delegated low-level driver, and the chapter does not describe it. Per-vehicle geometry is stated: avoidance *"wrote high danger into the map over the width of the other car, but also **a decreasing skirt of danger at the edges**. This kept a minimum lateral separation between cars."*

**SLIDE / DRIFT HANDLING.** **Source does not state** for steering. The only emergency mechanism is braking:

> *"we look at the highest danger across the planned journey from our current slot to the most interesting. If any slot is over some threshold of danger, we ask for braking with intensity proportional to the danger strength. **We use a threshold because some danger can be informative without being a real issue**"*

**No countersteer term.**

**LIMITS / ACTUATION.** **Source does not state** steering angle or rate limits. The guarantee claimed is about *constraints on the lane choice*, not the wheel: *"Context steering behaviors are small and stateless and **guarantee any desired movement constraint**"* and *"It also stops us remaining in high danger because of high interest when there's an obvious escape route."*

**WHAT THE SOURCE SAYS ABOUT TUNING.** Three failure modes with fixes:

- **Chase flip-flopping** — fixed by temporal blending: *"we can take the last update's context map and **blend it with the current one**, making high values emerge over time rather than instantly. This is a kind of **global hysteresis**."*
- **Spikes/troughs** — *"To avoid sharp spikes or troughs, we can apply a **blurring function** over the context maps."*
- **Judder at few slots** — *"This can lead to juddery behaviour if there aren't a lot of slots"*, fixed by sub-slot interpolation.

And the *architectural* lesson, which is the chapter's real contribution and a cautionary tale about building steering out of if/else:

> **F1 2010's avoidance** *"became monolithic… had decomposed into an **old-school sequence of if/else blocks with a thin steering behavior wrapper** and was a maintenance nightmare."*

Plus the write rule that makes arbitration order-independent: *"**Context maps are not cumulative.** When a behaviour wants to add strength to a slot, it is only written if it is **stronger** than the value already in the slot."*

---

## 26. Assetto Corsa (Kunos) — three bullets and a config file: the whole public record

- **Primary (first-party):** Stefano Casillo, *"The challenges in developing Assetto Corsa A.I."*, Codemotion Milan 2017 — https://www.slideshare.net/Codemotion/stefano-casillo-the-challenges-in-developing-assetto-corsa-ai-codemotion-milan-2017
- **Config glossary (community, NOT Kunos):** https://www.overtake.gg/threads/ai-ini-configuration.149771/
- **Grip-overhead comment source (community transcription):** https://www.scribd.com/document/813097469/Physics-Modding
- **CSP custom-AI interface spec:** https://docs.assetto.cn/en/custom-ai/

**CONTROLLED VARIABLE / ERROR SIGNAL.** The deck's slide 7, **in full**:

> **"AI Steering ● PID based ● Target dynamic distance ● Lateral offset"**

and slide 8 *"Steer target"* is an **image**: a single point on the racing line ahead, circled. That is the entire first-party technical statement. **Controlled variable: a spline aim point plus a "Lateral offset"** — the deck's own words. The reference line is the *"Fast Lane" dynamic spline* (slide 5: *"AI 'World' ● 'Fast Lane' dynamic spline ● Track borders ● Car data and status ● Opponents status"*). **No equation appears anywhere in the deck.**

**CONTROLLER TYPE.** *"PID based"* — verbatim, and nothing more. The deck explicitly excludes alternatives (slide 3: *"This talk is buzzword free, you won't find: ● Neural networks ● Fuzzy logic ● Expert systems"*).

**GAIN SCHEDULING.** **Source does not state** in the deck. Crucially, the **shipped config contradicts a speed-scheduled gain**: `STEER_GAIN` is a flat per-car constant documented as *"how hard the AI needs to turn the wheel"*. So Assetto Corsa is a **PID whose gain is fixed per car, with the scheduling done entirely by the aim distance** — structurally the same choice the Unreal RacingAI plugin makes (§16) and the same choice TORCS makes (§2/§3).

**LOOK-AHEAD / PREVIEW.** The deck says only *"Target dynamic distance"*. The shipped config documents the law as **affine in speed**, per the community glossary (marked here as community, not first-party):
- `BASE` → *"how far in front of the car the AI looks at the recorded line to know what to do"*
- `SPEED_GAIN` → *"how much it increases the distance according to speed (needs to look farther ahead to be ready for corners at higher speeds)"*

Witnessed shipped pairs: `BASE=18.6`; `BASE=20 / SPEED_GAIN=0.2 / GAS_BRAKE_LOOKAHEAD=3`; `BASE=18 / SPEED_GAIN=0.10 / STEER_GAIN=1.6`; `BASE=26.6 / SPEED_GAIN=0.8 / STEER_GAIN=1.5`. Live developer-app knobs confirm these are the real controls: `set aiSteerGain / set aiPush / set aiLookAhead / set aiLookAheadSpeed` (https://www.scribd.com/document/813097469/Physics-Modding). **Note the `BASE` values (18.6–26.6 m) are of the same order as TORCS's `17.0` m constant** — two independent sims converged on ~17–27 m of preview at low speed.

**SLIDE / DRIFT HANDLING — the survey's strongest negative finding, and it comes with the mechanism used instead.** **No countersteer controller exists in any public Assetto Corsa record.** Instead the AI is given *undeclared grip overhead* plus an artificial stability system. Shipped `ULTRA_GRIP` comment, verbatim:

> *"VALUE=1.2 ; how much lateral grip the AI is simulated within braking zones. This helps AIs brake closer to the limit"*
> *"**AI cars have a 20% (1.2 * 100 = 120%) tire grip overhead vs the player and some form of (artificial) stability control… The extra is there so the AI's weird and twitchy inputs don't make them spin.**"*

**That is the answer Assetto Corsa gives to the slide problem: make the plant more forgiving rather than control the slide.** Throttle-side slip handling does exist (deck slide 9: *"AI Throttle — Target speed / 'Zero throttle' / 'Torque/Grip Factor' / **Understeer Factor** / **Slip correction**"*), so there is an understeer parameter — on the throttle, not the wheel. Deliberately *inducing* slide is a tyre-model edit, not a controller mode: *"the only major difference was the 'Friction Angle Limit' was much higher, 9.3 vs 58.6… increasing the value (up to 90…) makes the car behave more loose"* (https://www.overtake.gg/threads/ai-drift-ini-manipulation.182358/).

**LIMITS / ACTUATION.** **Source does not state** any steering clamp or rate limit — also absent from the CSP custom-AI spec, which only normalises the interface: `float steer; /* normalize steer value from -1 to 1 */`.

**WHAT THE SOURCE SAYS ABOUT TUNING.** All of the following is **community documentation, not Kunos** — flagged as such:
- `STEER_GAIN`: *"the sensitivity of the steering wheel when the AI is in control. **If you see it making huge fast rotations to compensate on a corner, reduce it.** If it doesn't turn the wheel fast enough, increase."*
- Look-ahead: *"Tweak tip : +/-0.5, test. If a car cuts a corner too sharp, decrease, if it's ending on the outside line, increase."*
- *"`AERO_HINTS` being too high will also cause cars to miss corner entry."*
- *"different cars steer locks and steer ratios affect the steer gain within the ai.ini."* — an explicit statement that the gain is **not** portable across cars, which is the practical argument for normalising by steering lock the way TORCS does.

**MARKETING flag.** Deck slides 2 and 12 are non-technical. **For Assetto Corsa Competizione there is no first-party technical statement about AI steering at all** — official notes are behavioural only (*"Reworked pitlane spline logic for AI pit entry"*).

**One more first-party AC source calibrates what "PID based" means in this codebase** — not the AI, but the ABS/TC system, which uses the *same* idiom and documents its anti-windup and gain scheduling. From https://docs.assetto.cn/en/car/physics/brakes.html:
- *"The basic system is a **variable-gain PID controller with an additional constant controller**"* — i.e. AC's PID variant is **gain-scheduled PID + a feedforward constant**, which is the same two-degree-of-freedom shape the AI almost certainly uses.
- **Gain scheduling implemented as a LUT multiplied in**: `[ABS_GAIN_CONTROLLER_2] … LUT=(0=0.0|50=0.2|100=1) INPUT=SPEEDKMH COMBINATOR=MULT` — a **three-point table on speed**, multiplied rather than added.
- **Anti-windup with the reason stated**: `INTEGRAL_ERROR_LIM=-0.01,10000 ; this clamps the integral controller's integral calculation. **Without it, you can run into undershoot situations.**`
- **Derivative filtering with the reason stated**: `FILTER_TIME_CONSTANT=0.03 ; a filter is added here to model the filtering that would need to be done to a derivative signal in real life` — a *physical-realism* justification for the exact measure Game AI Pro ch.40 recommends for noise (§4).

**This is the survey's best evidence for what a shipped sim's PID actually contains**, and it confirms the two recommendations from the literature: **clamp the integral, filter the derivative.** Neither is stated for the AI, but the idiom is first-party and shared.

**Contradiction worth flagging:** Kunos's Casillo states *"One particularity of Assetto Corsa is that we're using the same physics for the artificial intelligence cars as for you"* (https://www.gtplanet.net/forum/threads/assetto-corsa-news-and-general-discussion.236693/page-258/), while the shipped config gives AI cars a hardcoded 20 % lateral-grip overhead in braking zones. **No source reconciles the two statements** — and it is the same class of contradiction as D9.

---

## 27. Gran Turismo Sophy — the exact action space, the ~6 s speed-scheduled observation window, and the absence of any classical controller

- **Primary:** Wurman et al., *"Outracing champion Gran Turismo drivers with deep reinforcement learning"*, Nature 602:223 (2022) — https://www.nature.com/articles/s41586-021-04357-7 (author PDF: https://www.cs.utexas.edu/users/pstone/Papers/bib2html-links/nature22.pdf)
- **Vendor pages (flagged marketing):** https://www.gran-turismo.com/us/gran-turismo-sophy/technology/ and https://www.gran-turismo.com/us/gran-turismo-sophy/project/

**CONTROLLED VARIABLE / ERROR SIGNAL.** **There is none — no classical error signal, an end-to-end policy:**

> *"The core actions of the agent were mapped to **two continuous-valued dimensions: changing velocity (accelerating or braking) and steering (left or right)**."*

Observed *"in the egocentric frame of reference of the agent"*. **Reference vector: the car's own frame, but as a policy input, not as an error term.**

**CONTROLLER TYPE.** *"model-free, off-policy deep RL… **quantile regression soft actor-critic (QR-SAC)**"*; *"GT Sophy interacted with the game at **10 Hz**"*. Note: opponents *inside the training scenarios* were driven by *"**simple PID controllers**"* — Sophy itself is not. Training hyperparameters, stated: policy LR `2.5e-5`, critic LR `5.0e-5`, **n-step 7** (explicitly revised: *"the 5-step return used on 2 July 2021 was not the best choice; this was changed to a 7-step return"*), discount `0.9896`, `α 0.01`, **32 quantiles**, 4 hidden layers × 2048 units.

**GAIN SCHEDULING / LOOK-AHEAD.** No gain exists, but the **observation horizon is speed-scheduled, and this is the cleanest statement of a time-based preview in the survey**:

> *"We encoded the approaching course segment as **60 equally spaced 3D points** along each edge of the track and the centre line… **The span of the points in any given observation was a function of the current velocity, so as to always represent approximately the next 6 s of travel.**"*

**~6 seconds of travel.** This is the same *kind* of rule as Pomerleau's empirical 2–3 s for human drivers (§7) and is the only modern, shipped, documented instance of it. Also: *"the agent was given a static map defining the left and right edges and the centre line of the track."*

**SLIDE / DRIFT HANDLING.** **Reward-side, not controller-side:** *"penalties if it went out of bounds, hit a wall or **lost traction**."* (The chapter also names the classic trailing-car failure: *"If the agent following does not anticipate the possibility of the opponent braking early, it will not be able to avoid rear-ending the human driver."*)

**LIMITS / ACTUATION.** No steering clamp (it is an input-space policy). The physics bound is stated, and it is a good sanity anchor for AI-vs-human comparisons: *"**GT Sophy cannot brake harder than humans but it can learn more precisely when to brake**"*; humans produce *"smooth, 60-Hz signals compared with GT Sophy's 10-Hz action rate."*

**WHAT THE SOURCE SAYS ABOUT TUNING.** ML-specific, but two lessons generalise:
- **Self-play was inadequate**: *"the straightforward application of self-play was inadequate in this setting… **By racing against only copies of itself, the agent was ill-prepared for the imprecision it would see with human opponents**"* — a reminder of the *exposure problem*.
- **Rule-modelling fought aggression**: *"as we tried to more accurately model blame assignment, **the resulting policies were judged much too aggressive by stewards and test drivers**."*

**MARKETING flag.** The vendor pages are non-technical: *"revolutionary superhuman racing agent"*, skills shown only as graphics, the RL description reduced to *"the agent takes an action in the world, is given a reward (or penalty) and receives an updated description of the world state."* **Cite the Nature paper, never the vendor page, for anything technical.**

---

## 28. Game AI Pro 3, Chapter 17 — "Fast Cars, Big City: The AI of Driver San Francisco" (Jenner & Ocio)

- **Source:** https://www.gameaipro.com/GameAIPro3/GameAIPro3_Chapter17_The_AI_of_Driver_San_Francisco.pdf

**CONTROLLED VARIABLE / ERROR SIGNAL.** Path-following to a **time-advanced target pose**:

> *"The action of the AI in any frame is based on the **desired position and orientation of the vehicle, as defined in the path it is following, at a time in the future**. Internally, the AI used a simple finite-state machine… **Each state had some heuristics that allowed the control values to be calculated based on the differences between the current heading and velocity and the target position and orientation of the vehicle.**"*

**⚠ Note the reference is explicitly TWO vectors — "the differences between the current heading *and velocity* and the target position and orientation."** This is the only game-industry source in the survey that names **both** the heading and the velocity as error inputs. It does not say how they are combined.

**CONTROLLER TYPE.** *"simple AI path-following module"*; an FSM of maneuver states with heuristics producing *"controller input based on these data"*. **No P/PD/PID/pursuit is named** — the algebraic law is **not stated.**

**GAIN SCHEDULING / LOOK-AHEAD.** No steering gain stated. The preview is the **path horizon**, and it is stated in both space and time:

> *"This rectangle was wide enough to encompass the widest road in the network, and it was **long enough to allow the vehicle to travel at full speed for a couple of seconds**"*
> paths *"represented the predicted movement of the vehicle for the **next couple of seconds**… path updating happened approximately **every second**."*

**SLIDE HANDLING.** No countersteer — instead, **feasibility by simulation**:

> *"the turns have no representation of the momentum of the vehicle or the limits of friction at the wheels. **These problems are resolved by the low-level path optimizer**"*, which runs *"a simplified 2D physics simulation… **driven by the same inputs as the game vehicle**."*

That is a fourth distinct answer to the slide problem (alongside: control the slide, exclude it, delete it kinematically, and make the plant forgiving — §9, §8, §21, §26).

**LIMITS / ACTUATION.** **Source does not state** a steering clamp. Actuation honesty is stated as a constraint the AI could not escape: *"**We were not allowed to cheat by giving the AI vehicles more power or tighter grip between the tires and the road.**"* (Directly opposite to Assetto Corsa's 20 % grip overhead, §26 — an explicit disagreement.)

**WHAT THE SOURCE SAYS ABOUT TUNING.** Verbatim: *"The three terms were **scaled by factors we arrived at empirically** to give the best results."*; *"we found that **three or four iterations of the optimization loop were enough** to approach a local minimum in scoring"*. Path score terms are named: dot product with the potential-field gradient, `|speed − desired speed|`, and a collision penalty.

---

## 29. Drift King — "Scully" (indie dev deep dive): the only source in the survey with real **drift** control mathematics

- **Source:** https://mellowarpeggiation.itch.io/drift-king/devlog/641400/scully-slot-cars-tangent-bars-and-automobiles-ai-deep-dive

This is the standout for field 6. Everything else in the survey either omits slide handling or handles it analytically; Scully has a **two-state Line/Drift controller with an explicit drift-sustaining policy**.

**CONTROLLED VARIABLE / ERROR SIGNAL.** A signed angle from the car's **projected future position** to a sampled point on the line, expressed in the car's frame:

> *"**applying steering to reduce the Steering Offset angle to zero**… a signed angle that points from the projected position to the sampled position, relative to our cars current angle (so if the car needs to turn right to match the offset angle, it's positive, and for left, negative)"*

Inputs include *"**Vehicle drift angle** (can be derived from the vehicles velocity and rotation)"*. **So the reference is the projected-position-to-line direction, and the velocity direction enters through the drift angle.** This is the closest thing in any game-dev source to ARS's velocity-referenced aim error — it does not measure the error *from* the velocity vector, but it *uses* the velocity direction as a control input.

**CONTROLLER TYPE.** **Two-state machine (Line / Drift) with projection-based P control on an angle offset**, outputting *"steering, handbrake and boost. With **100% throttle at all times**."*

**GAIN SCHEDULING.** **Source does not state** explicit gains. The effective scheduling is **state-dependent preview**: Line mode uses the 0.5 s projection, Drift mode the 1.2 s projection.

**LOOK-AHEAD / PREVIEW — the survey's only explicit ballistic projection with a stated formula.** Verbatim:

> *"Project where our vehicle is going to end up given its current velocity and acceleration — for both **0.5 seconds and 1.2 seconds** into the future. The formula to find this is **`p + v·t + 0.5·a·(t·t)`**."*

Then: *"Find the distance from the vehicle to these two projected points, **add this distance to the distance given by the closest sample**, and plug this into the curve… In our implementation, we also add an extra bit of distance when sampling the curve, keeping the sampled points a set distance ahead of our projected points."*

**This is a fundamentally different look-ahead construction from every other source here**: instead of "how far ahead along the line do I aim", it asks "**where will I be, and which line sample is nearest that point**" — a *ballistic* look-ahead. It is also the only source that uses **acceleration** in the projection. ARS's disabled `ProjectAhead` idea (a ballistic + pessimistic projection replacing geometric route speed) is the same primitive, and the survey contains no other game source that does it.

**SLIDE / DRIFT HANDLING — the standout, quoted in full:**

> *"If our Drifting Offset goes too far off the track and we're in Line mode, **switch to Drift mode**"*
> *"In Drift mode, **align car to 1.2s projected angle, and use the handbrake/boost to resolve 0.5s projected angle**"*
> *"apply handbrake **until the vehicle drift angle is high enough to sustain the drift without further handbraking**"*
> *"Continue holding the handbrake **until the Steering Offset is near the outside of the corner**. When the Steering Offset falls to the outside of the corner, **apply boost to bring it back to the line**."*

And the outside/inside detection, which is a genuinely novel use of the two projections:

> *"We use the tangent of the Steering Offset sample to determine 'outside'. **If both projected points fall on the same side, we are travelling towards the outside of the corner. If projected points fall on opposing sides, we are travelling too far on the inside of the corner, AKA cutting the corner.**"*

So **two horizons of the same projection are used as a differential test to diagnose corner-cutting vs running wide.** No other source in this survey does that.

**LIMITS / ACTUATION.** Outputs normalised (`Desired steering (float, -1 to 1)`); **no explicit clamp or rate limit stated.** One interesting operational constraint is recorded: *"If your game does not have any way of increasing engine power at will (boost, NOS, magic), the vehicle only be able to maintain the line from the inside of the corner… you may be able to **modulate throttle instead, using only 0.5 throttle in the middle of a corner**."*

**WHAT THE SOURCE SAYS ABOUT TUNING.** The best experimental-methodology passage in the survey from a solo developer:

- *"the first and biggest problem was **failing to control variables while developing the algorithm**"*
- *"we were **overfitting to the bumps and peculiarities of this track**"*
- *"we couldn't quickly iterate, and were once again overfitting. This time however, we were **overfitting to the first corner**… watching the vehicle take the first corner reasonably well and then **absolutely suck at the remainder of the track!**"*
- The fix: **per-corner test suites with exact start position and velocity.**
- The general rule extracted: *"in basic terms, if you want to create effective state machine based AI systems, **you must reduce the problem space**"*
- And on difficulty: *"for creating difficulty levels… **Always make your hardest difficulty first.**"*
- Residual known flaw: *"we do however have some **overfitting issues for cases where the vehicle can accelerate and turn considerably faster than stock**."*

---

## 30. Radu Angelescu — "Implementing a simple top down racing game AI controller": a bang-bang relay on aim angle

- **Source (full code in the post):** https://raduangelescu.com/post/classictopdownracinggameai/

Included as the survey's clearest **non-proportional** steering implementation — a relay (bang-bang) controller — and because it is the only source that stores its parameters in an array explicitly to hand them to an optimiser.

**CONTROLLED VARIABLE / ERROR SIGNAL.** The angle between the car's forward vector and the car→racing-line-point direction:

```cpp
forwardCar, dir, dot, angleInDegrees
```
Reference = **body forward**. Aim point = `getSectorPoint(getCurrentRaceSectorIdx() + floor(EBASICAI_LOOKAHEAD_DISTANCE))` — i.e. **look-ahead expressed in NODE COUNT, not metres.**

**CONTROLLER TYPE.** **Bang-bang (relay) on an angle threshold — not proportional:**

```cpp
if (diffAngleToTarget > ANGLETOTURN) RIGHT = 1.0f;
else if (diffAngleToTarget < ANGLETOTURN) LEFT = 1.0f;
```

`ANGLETOTURN` therefore acts simultaneously as the **deadband** and the **full-lock threshold** — the output is always 0 or ±1. **This is the only relay steering controller in the survey**, and it is worth noting that it is viable only because the aim point moves with the car; a relay on a fixed target would chatter.

The parameters are deliberately data, not code: they are stored in an array *"so we can later generalize and use the basic ai for our machine learning/optimization algorithms."*

**GAIN SCHEDULING.** No steering gain. Speed is scheduled **by the aim angle**, used as a curvature proxy:

```cpp
maxSpeedPercent = MAXSPEED * (1 - diffAngleToTarget * ANGLETOTURNSPEEDINFLUENCE);
```

i.e. **the same signal that drives the steering relay also caps the speed** — a neat one-signal design, and structurally the same "steering-angle-scaled throttle" idiom as the SCR folklore bot (§12) and the Unreal RacingAI plugin's dynamic throttle.

**LOOK-AHEAD.** **Constant in node count** (`EBASICAI_LOOKAHEAD_DISTANCE`). **Source does not state** any speed relation, floor, cap, or stability rule. The racing line is a 3-point moving average of track centres, smoothed `smoothIterations` times.

**SLIDE HANDLING.** **Source does not state.** No countersteer. A wall raycast only stops acceleration: `sensorData[IS_RAYCAST0] > DISTANCETOFRONTWALL_STOP`.

**LIMITS.** `ANGLETOTURN` is the deadband that bounds the relay. **No steering clamp or rate limit** — the output is inherently ±1/0, so the "rate" is whatever the physics does with full lock.

**WHAT THE SOURCE SAYS ABOUT TUNING.** Verbatim: *"The parameters can only be determined based on our car physics model, so we may see this as an opportunity to test a **differential evolution algorithm** for the problem: 'pick the best parameters so the racer gets the best time in a race'."* — i.e. the author's own position is that these thresholds are **not derivable and should be searched**.

---

## 31. Shipped-sim patch notes as evidence: rFactor 2 and Automobilista 2

These fill the survey's gap on **what production sims actually fix and expose**, when they publish no law.

**(a) rFactor 2 (Studio 397).**

- **Oscillation attributed to look-ahead, verbatim (Oct 2022 RC notes):** *"**Fixed waypoint lookahead to fix jerky steering motion**"* — alongside *"Fixed erratic throttle and brake inputs"* and *"Fixed an issue with driveline noise that could cause AI to brake partially off track and spin, or cut too much."* (https://support.studio-397.com/hc/en-us/articles/5688464366095-October-2022-Release-Candidate; text verified verbatim via mirrors https://www.overtake.gg/news/rfactor-2-publishes-october-release-candidate.807/ and https://traxion.gg/rfactor-2s-test-release-candidate-previews-updated-ai-and-integrated-store/.) November 2022 release post: *"**Smoothed out steering inputs and car control**."* (https://www.studio-397.com/2022/11/announcing-ai-improvements/)
- **The only documented AI steering-limit mechanism in the survey, verbatim:** *"Add **AI max steering lock multiplier** to deal with their lower quality physics which can be off at times (**defaults to 1.0**)."* Plus: *"Separated AI Spring Rate Multiplier. Just in case you need to tune it manually."* and *"Automatic AI front/rear roll stiffness calculation, should help their balance with their simplified physics. **A compensation of sorts.**"* (https://docs.studio-397.com/display/DG/Physics%20Calculation%20Tool)
  **What this means architecturally:** rFactor 2's AI does not merely receive a handicap in grip or power — it is given a **steering-lock multiplier applied to the controller's output**, explicitly to compensate for the AI's own simplified physics being wrong. That is a *controller→plant mismatch* correction, in the same family as Assetto Corsa's 20 % grip overhead (§26) and Driver SF's forbidden cheats (§28), but implemented on the *steering authority* rather than the tire.
- **CONTROLLED VARIABLE / CONTROLLER TYPE / GAINS.** **Source does not state** anywhere in public rF2 documentation. They publish knobs and symptoms, never the law. The one structural hint: *"Fixed **waypoint lookahead**"* — the aim is a waypoint, and look-ahead is the tuning knob.
- **WHAT THE SOURCE SAYS ABOUT TUNING.** Verbatim from the track-AI docs (https://docs.studio-397.com/display/DG/Track+AI+Tutorial): *"If the branches are not smooth and 'seamless' to the fast path, the AI cars will **brake heavily and might do weird steering movements** before entering the fast path."*; on a racing-line defect: *"This **kink** makes the AI hit the brakes at the start-finish line, ruining their lap times and smooth racing."*; and the world model: *"**Main Corridors**: The AI uses corridors to get information on the width of the track - what is legal racing surface as opposed to illegal surface."* Driving-line variants shipped: *"Fastest - fastest, optimal racing line; Left and right - side by side paths for formation lap; Block - defending line"*.

**(b) Automobilista 2 (Reiza).** Verbatim AI entries (https://www.bsimracing.com/automobilista-2-v0-9-5-1-released/, https://forum.reizastudios.com/threads/automobilista-2-v0-9-5-1-released.11126/):

> *"**Fixed AI tendency to oscillate steering erratically running down longer straights**"*
> *"**Adjust AI steer smoothing to improve pitlane mobility**"*
> *"Increased AI lateral rate of movement when calm"*
> *"This build introduces the new AI Strength logic - now the setting works more similarly to how it did in AMS1, as a **more straight-forward AI Grip multiplier**, without affecting other aspects of AI performance & behavior"*
> *"Added customized parameter per car for AI brake application"*
> *"Improved AI skill when driving off the racing line & reduced range for factoring cars up ahead"*

**Two findings from this.** First, **oscillation on straights is treated as a smoothing problem**, not a gain-scheduling problem — the fix is *"steer smoothing"*, the same lever Unity's Standard Assets actuator uses (§17) and the same lever DonkeyCar recommends via look-ahead (§10). Second, **AI difficulty is a grip multiplier, not a controller gain** — the shipped answer to "make the AI faster" is to change the *plant*, not the *law*. That is now the third independent instance of the same choice (Assetto Corsa, AMS2, and rF2's lock multiplier being its steering-side analogue).

**Fields 2–5 and 7 (error signal, law, gains, look-ahead, clamps): source does not state** — these are vendor patch notes that name symptoms only.

---

## 32. Two prominent sources that are **not** about steering — flagged so they are not miscited

**(a) "The Pure Advantage: Advanced Racing Game AI"** — https://www.gamedeveloper.com/design/the-pure-advantage-advanced-racing-game-ai (and its GDC talk *"Advanced Racing Game AI in PURE"*, https://www.gdcvault.com/play/1011998/Advanced-Racing-Game-AI-in).

**Read in full (all five pages plus the GDC deck): there is no steering controller, no look-ahead, no preview, no gain, no clamp, and no constant pertaining to steering in the entire article or deck.** It is a **race-management / difficulty-balancing** article. Its actual content: rejecting rubber-banding in favour of per-driver **"skills… represented as a real number within the range [0..1]"**, **"Dynamic Competition Balancing (DCB)"**, and a **"race script"**; skill clamps; and per-lap skill bonuses (*"+0.05 skill bonus during the first lap, then a +0.025… and no bonus the last lap"*). The only steering-adjacent item is a personality effect on *"the probability for him to oversteer/understeer the corners"* — no controller. Other constants: aim points ±0/250/500 m from the player, 150 m skill-saturation distance, 10–20 s top-skill start, ~5 s interpolation.

**This source is technical but off-topic for a steering survey. Anyone citing "The Pure Advantage" as a steering-controller reference is misciting it** — the title's "pure" refers to *PURE*, the game, not to pure pursuit.

**(b) "Artificial Intelligence and Games" (Yannakakis & Togelius), §6.4.1 "Racing Games"** — https://gameaibook.org/wp-content/uploads/2025/08/AI_and_Games_2nd_Edition.pdf (1st ed. §3.4.5: https://gameaibook.org/book.pdf).

**Whole-book searches: "steer" occurs on exactly two lines (the same sentence), "PID" = 0 hits, "pure pursuit" = 0 hits, "throttle" = 0 hits, "racing line" = 1 hit (the Drivatar sentence).** The book contains **no steering controller and no control law.** Fields 2–8: **source does not state** in every case.

What it does contribute, and it is worth keeping:
- On inputs, verbatim: *"The vast majority of racing games take a continuous input signal as a steering input, similar to a steering wheel."*
- On planning depth: *"At the most basic level, the agent needs to control for the position of the vehicle and adjust the acceleration or braking, using fine-tuned continuous input, so as to traverse the track as fast as possible. **Doing this optimally requires at least short-term planning, one or two turns (of the track) forward.**"*
- On Drivatar (Forza), verbatim: *"The early Drivatar agents are built on a form of supervised lazy learning… all tracks in the game need to be composed of segments drawn from the same 'alphabet'. During driving, the agent **selects the driving actions that most closely approximate the racing line taken by the players on the relevant segment**… but posed restrictions on the design of the tracks."* — i.e. **a segment-keyed action lookup with an authored alphabet constraint.**
- On the Simulated Car Racing Championship, verbatim and directly relevant to the folklore-vs-engineering question: *"**A general trend observed over the course of the competition was that the winning agents incorporate more and more domain knowledge as hand-coded mechanisms, with learning algorithms generally only used for tuning parameters of these mechanisms.**"* — the competition record's own verdict is that **hand-coded structure beat learning for steering**, with learning used for gains.
- Input-representation rule of thumb: *"it is much easier to learn a good driving policy if the inputs are represented in the **frame of reference of the car** rather than that of the track."*

---

## 33. **SuperTuxKart** (shipped open-source kart racer) — geometric pure pursuit with a deliberate ×2 over-steer and **no P/I/D at all**

- **Source (commit-pinned):** `github.com/supertuxkart/stk-code` @ `734f180f8cb7a8ebca171579e82b0ad158fb684f` — `src/karts/controller/ai_base_controller.cpp`, `ai_base_lap_controller.cpp`, `skidding_ai.cpp`, `ai_properties.hpp`

This is the survey's cleanest example of **shipped, executable, readable steering code in a real commercial-quality game**, and it is materially different from every other source here in two ways: **the look-ahead is not speed-based**, and **there is no PID**.

**CONTROLLED VARIABLE / ERROR SIGNAL.** The aim point **in kart-local coordinates**:

```cpp
Vec3 lc = m_kart->getTrans().inverse()(point);
```
Local X is lateral, Z is forward. **Not** a track tangent, **not** a cross-track offset, **not** the velocity vector. The steering law is the pure geometric arc construction (see below).

**CONTROLLER TYPE — the exact law, `steerToPoint()` @86, lines 127–149:**

```cpp
float radius = (lc.getX()*lc.getX() + lc.getZ()*lc.getZ()) / (2.0f*lc.getX());
...
float sin_steer_angle = m_kart->getKartProperties()->getWheelBase()/radius;
...
float steer_angle = asinf(sin_steer_angle);
...
return steer_angle*2.0f;
```

so `r = (x² + z²)/(2x)`, `δ = asin(L/r)`, and the command is **`2δ`**. The code comment states the reasoning and is worth quoting because it is the only explicit justification for a deliberate over-command in the survey:

> *"After doing the exact computation, we now return an 'oversteered' value. **This actually helps in making tighter turns**, and also in very tight turns on narrow roads (where following the circle might actually take the kart off track) it forces smaller turns. **It does not actually hurt to steer too much, since the steering will be adjusted every frame.**"*

**No P, no I, no D.** The ×2 is a fixed, deliberate gain on a geometric law — and note the justification is *re-solution frequency*, i.e. the same argument for why a proportional law re-evaluated every frame tolerates over-command.

**The gain is not `2·wheelbase/L_d` in the usual sense** because there is no `L_d`: the radius is derived from the actual local offset of the aim point each frame, so the effective look-ahead *is* that distance. **This corroborates ARS's geometric form `atan(2·L·sin(err)/d)` as the same family, with `sin`-saturation.**

**LOOK-AHEAD / PREVIEW — the survey's outlier: a track-width corridor search, not a speed schedule.** `skidding_ai.cpp findNonCrashingPoint()` @2695, lines 2721–2773:

```cpp
unsigned int steps = (unsigned int)( len / m_kart_length );
if( steps < 3 ) steps = 3;
if( steps>1000) steps = 1000;
...
if ( distance + m_kart_width * 0.5f > DriveGraph::get()->getNode(*last_node)->getPathWidth() )
{ *aim_position = ...getCenter(); return; }
```

The aim point is the **furthest node centre reachable in a straight line without the kart's half-width leaving the track.** The look-ahead distance is therefore an *emergent property of track width and geometry*, not a tuned constant or a speed function.

**STATED STABILITY RULE — the sharp-turn guard**, which is the closest thing to a stability criterion in this file:

```cpp
float diff = normalizeAngle(angle1-angle);
if(fabsf(diff)>1.5f) { *aim_position = DriveGraph::get()->getNode(target_sector)->getCenter(); return; }
```
**~1.5 rad ≈ 86°**: if the aim point swings more than ~86° off, fall back to the sector centre.

*Note for readers:* `const unsigned int look_ahead = 10;` @`ai_base_lap_controller.cpp:171` is **node-list bookkeeping, NOT the steering preview** — the same naming trap as TORCS's `SteerLookaheadMinMeters`.

**SLIDE HANDLING — deliberate, not corrected.** Skid control is *requested alongside* steer: `canSkid()` @2899 with `const float MIN_SKID_SPEED = 5.0f;`, and an arc duration `float duration = length / m_kart->getSpeed(); duration *= 1.5f;` where the `1.5f` is documented as an *"experimentally found factor"*. There is a no-skid-on-straights rule and a against-track-direction veto. **There is NO countersteer or opposite-lock anywhere — source does not state.**

There is, however, a **diagnosed failure mode** for the aim-point-behind-the-axle case, @100–118, which is worth recording because it is a concrete, observed instability:

> *"it will result in the kart **doing slaloms, not driving straight**"*

**LIMITS / ACTUATION — an explicit steer-rate limit, and the industry's stated reason for it.** `setSteering()` @185, lines 187–215:

```cpp
float steer_fraction = angle / m_kart->getMaxSteerAngle();   // then clamped to ±1.0
float max_steer_change = dt/m_ai_properties->m_time_full_steer;
```

The documentation comment is the survey's best statement of *why* a rate limit exists:

> *"uses a 'time till full steer' value … The parameter is defined in the kart properties and helps somewhat to make AI karts more **'pushable'** (since **otherwise the karts counter-steer to fast**)."*

and `ai_properties.hpp:70-72`:

> *"Time for AI karts to reach full steer angle (**used to reduce shaking of karts**)."*

**So SuperTuxKart's fix for steering oscillation is a rate limit, not a gain tweak** — and the stated reason is *countersteering too fast*, i.e. the rate limit is deliberately slowing the countersteer the AI would otherwise apply. **This is the closest thing in the survey to a rationale for the rate limit ARS applies at 360°/s, and it points the *opposite* way from a slide response: the limit exists partly to blunt countersteer.**

There is also a **saturation detector that brakes instead of scheduling a gain**:

```cpp
if(m_kart->getSpeed() > max_turn_speed && m_kart->getSpeed()>min_speed && fabsf(m_controls->getSteer()) > 0.95f)
    m_controls->setBrake(true);
```
i.e. *"I am at the steering limit and still too fast — slow down"*, with the steer threshold at 95 % of lock. This is the same insight Ziggy Racer reached (§6) — saturation means no steering lever remains, so reduce demand — implemented as a simple detector rather than a speed plan.

**WHAT THE SOURCE SAYS ABOUT TUNING.** Verbatim, and unusually honest about shipping a known-imperfect algorithm:

- *"This results in this algorithm **often picking points to aim at that would actually force the kart off track**. But in reality the kart has to turn (and does not immediately in one frame change its direction) which takes some time - so it is actually mostly on track. **Since this algoritm (so far) ends up with by far the best AI behaviour, it is for now the default.**"*
- `/** TODO: ONLY USE FOR OLD SKIDDING! … The minimum steering angle at which the AI adds skidding. **Lower values tend to improve the line the AI is driving.** This is used to adjust for different AI levels. */`
- Open defects left in code: `// FIXME - requires fixing of the turn radius bugs`, `//TODO : make acceleration steering aware`

**Verdict: the survey's best counter-example to "you need a PID and a speed-scheduled look-ahead."** A geometric pure pursuit, ×2, with a track-width corridor preview and a steer-rate limit, ships as the default AI in a real game and is described by its own authors as producing *"by far the best AI behaviour"* among the alternatives they tried.

---

## 34. TORCS / Speed Dreams shipped robot family — four drivers, one real two-degree-of-freedom controller (and it ships **off**)

- **Source (commit-pinned mirrors, because the official hosts were unreachable):** `github.com/jpbruyere/speed-dreams` @ `0bfb1053a28c69169842e93ed5f0e0708bb1020f`; `github.com/jzbontar/torcs` @ `f84e113231bd64754df1e7529f415e4ba20a11ef`
- Constants are **commit-specific**; current 2.4.x values were not re-verified.

**This closes the Speed Dreams gap flagged in §33** and materially changes three conclusions in the synthesis.

**(a) USR default — pure pursuit with the yaw-rate term folded into the reference and a steer-into-the-skid term** (`raceline.cpp getAvoidSteer()` @2221):

```cpp
GetSteerPoint(5.0 + car->_speed_x/10, &target, offset, steertime * time_mod);
double steer_direction = targetAngle - (car->_yaw + car->_yaw_rate/15);
...
steer = steer_direction / car->_steerLock;
double nextangle = data->angle + car->_yaw_rate/4;
if (fabs(nextangle) > fabs(data->speedangle)) {
    double anglediff = (data->speedangle - nextangle) * (0.1 + fabs(nextangle)/6);
    steer += anglediff * (1.0 + MAX(1.0, 1.0 - car->_accel_x/5));
}
```

Three things to note:
- **The yaw-rate damping is inside the *reference*.** The target bearing is compared against `car->_yaw + car->_yaw_rate/15` rather than against the nose — so the aim error is measured from a **yaw-rate-lead-corrected heading**, which is a compact way to build damping into a P law without a D term. **This is a distinct third option from "add a D term" and "add a separate yaw-rate term": predistort the reference.**
- **The second term is a genuine steer-into-the-skid term**, gated on the *velocity* angle (`data->speedangle` is velocity-vs-track, §2) versus a yaw-rate-predicted angle, with a gain `(0.1 + |nextangle|/6)` that **grows with the size of the predicted angle** and a further multiplier `(1 + MAX(1.0, 1 − accel_x/5))` that **grows under braking.** This is the only shipped TORCS-family term that steers *with* a developing slide.
- **Look-ahead `5.0 + speed_x/10` m** — a 5 m floor with a 0.1 s speed term, then a **ballistic re-projection** (`double Time = deltaTime*3 + MAX(0.0, time/2);`). **Gain is flat (`1/steerLock`); no integral.**

**(b) TORCS classic — a speed-proportional preview with an explicit sub-linear cap** (`driver.cpp getTargetPoint()` @2078):

```cpp
lookahead = (LOOKAHEAD_CONST * 1.5 + speed * 0.45);
lookahead = MIN(lookahead, (LOOKAHEAD_CONST + ((speed*(speed/10)) * 0.15)));
lookahead *= SteerLookahead;
```
with `LOOKAHEAD_CONST = 18.0f; // [m]` and `LOOKAHEAD_FACTOR = 0.33f`.

**The preview grows fast at low speed and then flattens** — `MIN(27 + 0.45v, 18 + 0.015v²)` — rather than growing without bound. **This is the only sub-linear look-ahead cap in the survey** and it is a cheap, directly reusable rule: at 30 m/s the first form would give 40.5 m but the cap gives 31.5 m.

**(c) TORCS `damned` — the clean pure-pursuit baseline, and the source of the snap-back guard.** P with geometric gain and nothing else:

```cpp
targetAngle = atan2(target.y - car->_pos_Y, target.x - car->_pos_X);
targetAngle -= car->_yaw;
NORM_PI_PI(targetAngle);
return targetAngle / car->_steerLock;
```
with `LOOKAHEAD_CONST = 17.0f; LOOKAHEAD_FACTOR = 0.33f;` — **the same constants as the tutorial (§3), now confirmed as the shipped `damned` baseline** — plus the preview **rate** guard which is a stated stability rule on the *preview itself*:

```cpp
float cmplookahead = oldlookahead - car->_speed_x*RCM_MAX_DT_ROBOTS;
if (lookahead < cmplookahead) { lookahead = cmplookahead; }
```
(*"Prevent 'snap back' of lookahead on harsh braking"* — the same mechanism as `bt`, §2, now confirmed in a second driver.)

Slide handling: **source does not state** — none. The side-collision blend (`psteer = steer*(d/c) + 1.5f*psteer*(1.0f - d/c);`) only substitutes when it does **not** reverse the command's sign. Limits: **none stated**.

**(d) USR `SteerMod` — the survey's most complete two-degree-of-freedom controller, and it ships OFF.** `raceline.cpp` 1997–2208; `SteerMod(0)` ctor, ini `PRV_STEER_MOD` default **`0.0f`**.

Verbatim, in four parts — **feedforward Ackermann on target curvature**:

```cpp
k1999steer = atan(wheelbase * steergain * TargetCurvature) / car->_steerLock;
```

**cross-track error and velocity-angle error** (note the **speed-divided gain**, `300/(carspeed+300)/15`):

```cpp
Error = (dx*(Y-SRL[SRLidx].ty[Prev]) - dy*(X-SRL[SRLidx].tx[Prev]))/Mag(dx,dy);
double sError = (dx*car->_speed_Y - dy*car->_speed_X)/(carspeed+0.01);
VnError = asin(sError);
k1999steer -= (atan(Error*(300/(carspeed+300))/15) + VnError)/car->_steerLock;
```

**sideslip term with its own speed schedule**:

```cpp
Skid = (dirx*vy - vx*diry)/(carspeed+1.0);
double skidfactor = 1.0 - MIN(0.6, carspeed/120);
k1999steer += (asin(Skid*sc)/car->_steerLock)*skidfactor;
```

**yaw-rate error against the kinematic reference**:

```cpp
double yr = carspeed*TargetCurvature;
double diff = (car->_yaw_rate*(data->mode==mode_normal?1.0:1.2)) - yr;
...
k1999steer -= steerskid;
```

**and TWO STATED GAIN SCHEDULES, verbatim from the comments:**

> *"// decrease steergain as speed is slower than targetspeed and we're either avoiding or have just recovered from avoiding."*
> *"// decrease steergain at high speed to stop bouncing over curbs"*
> `if (carspeed > 60) steergain = MAX(MIN(1.0, SteerGain), steergain - (carspeed-60.0)/40);`

**This is the survey's only implementation containing `k_ff·κ` + cross-track + sideslip + yaw-rate error together** — i.e. exactly the shape of the modern feedforward+feedback architecture — and it is **disabled by default** in favour of the simpler `getAvoidSteer()` pure pursuit. The author's own uncertainty is left in the source: `// what does this do???` @2048, and two `//Unused code?` markers. **Sideslip drives a SPEED cut rather than more steer.**

**(e) simplix (Speed Dreams default) — first-order model + feedback, and the only shipped default in the corpus whose primary signal is cross-track:**

```cpp
double AheadDist = oLookBase + oCurrSpeed * oLookScale;
...
double Angle = AheadPointInfo.Angle - CarYaw;
DOUBLE_NORM_PI_PI(Angle);
if (oCurrSpeed < SLOWSPEED) return Angle;
...
Angle += 0.08 * (Omega - CarYawRate);          // yaw-rate P, gain 0.08
Angle += AvgK * oScaleSteer;                    // curvature feedforward
...
oPIDCLine.oP = 1.2;  oPIDCLine.oD = 12;         // preceded by commented-out oP = 1.0; oD = 10;
...
Angle -= Factor * atan(oPIDCLine.Sample(Delta));
```
where **`Factor = MIN(0.15, oStartSteerFactor)`** — the cross-track correction's authority is **capped at 15 % of the total steer angle.** That is a direct, quotable answer to "how much should the cross-track term be allowed to contribute": **≤15 %**, with the aim-point pursuit carrying the rest.

Look-ahead: `oLookAhead(5.0)` m + `oLookAheadFactor(0.05)` s, with the factor **hard-capped** `MIN(0.2, …)` — and the **preview SHRINKS with skill**:

```cpp
oLookAhead = oLookAhead / (1+oSkillGlobal/24);
```
**Better AI previews *shorter*.** This is the only source in the survey that treats look-ahead as a *difficulty* parameter, and it does so in the direction that gives the stronger driver less smoothing — which is the opposite of how difficulty is implemented everywhere else (grip multipliers, §31).

Its PID class has explicit **anti-windup on the integral state**: `oMaxTotal(100), oMinTotal(-100)`, `oI(0)` default. The recorded tuning history is preserved in comments: `oP = 1.2; oD = 12;` preceded by commented-out `oP = 1.0; oD = 10;` — **a recorded 20 %/20 % increase**, which is the only instance in the survey of a tuning change left in the source as evidence.

Limits: `oSteer = FilterSteerSpeed(oSteer);` with `const float MaxSteerSpeed = 0.1f;` — **a steer-rate limit of 0.1** (units per tick, partially unverified) — plus a Range clamp.

Slide: **yaw-rate term only; no countersteer — source does not state.**

**(f) Correction to §15 — the full iRacing "Smoothness" gloss.** The attribute is documented as:

> *"**Smoothness – A measure of the AI Driver's steering behavior. A lower value results in more oversteer. A higher value results in more understeer.**"* (https://support.iracing.com/support/solutions/articles/31000153531-ai-rosters)
> *"This won't make them any faster or slower, but **the attitude of the car will be slightly different.**"* (https://www.iracing.com/airoster-/)

**So iRacing's one exposed steering parameter is a single scalar that trades oversteer against understeer** — a direct, if undocumented, statement that their AI's steering law has an oversteer/understeer balance axis. All other fields remain **source does not state**; this is still UI-level, not controller-level.

---

## 35. A note on the remaining explicit gaps

Not extracted, and **no claims are made about them**:
- **TORCS `berniw` / `bt` / `olethros` / `kilo2008` / `K1999`** steering — `berniw` and `bt` are covered in §1–§2 from the Debian source mirror; the others are not.
- **SuperTuxKart numeric defaults** (`m_skidding_threshold`, `m_time_full_steer`) and `getSteeringWhenSkidding` — the *mechanism* is documented in §34, the numbers are not.
- **The Speed Dreams `shadow` robot's dedicated `PidController.cpp`** — a promising unread lead; Shadow is described as *"extremely fast and aggressive in single-stint races"* (https://sourceforge.net/p/speed-dreams/wiki/ListOfRobots/), so its controller would be worth reading.
- **BeamNG, art of rally, KartKraft, Circuit Superstars, Split/Second, Sonic & All-Stars Racing** dev write-ups — not attempted.
- **Assetto Corsa Competizione internals** — no first-party technical statement exists (§26).

---

## 36. Sources that were located but could not be retrieved (audit trail)

Recorded so the survey's gaps are explicit rather than silently filled:

- **Speed Dreams robot source.** The modern tree is Forgejo-hosted and returns **HTTP 403 to automated fetch** (`https://forge.a-lec.org/speed-dreams/speed-dreams-code/raw/branch/main/src/drivers/shadow/src/Driver.cpp` → `403 Forbidden`, nginx). The GitHub mirror `argos-research/speed-dreams` is a **2015 pre-fork snapshot whose `usr.cpp` no longer contains the steering code** (steering moved into `driver.cpp`/`unitdriver.cpp`), and `unitdriver.cpp` returns `unsupported content type "application/octet-stream"`. **What is documented about Speed Dreams steering in this survey is therefore limited to:** (i) the robot family and its provenance — Simplix, USR, Shadow and DanDroid all use **Tim Foden's shared Raceline** (https://sourceforge.net/p/speed-dreams/wiki/ListOfRobots/); (ii) USR is described as having *"used originally the legendary K1999 raceline calculations (currently uses the Tim Foden's Raceline, like Simplix, Shadow or DanDroid)"*; (iii) a **real, sourced tuning symptom** from the Speed Dreams development list, verbatim: *"I did see problems in traffic: as soon as it is forced outside the raceline, **it tries to correct too strong and spin after the correcting escalated**."* (https://sourceforge.net/p/speed-dreams/mailman/speed-dreams-devel/?page=669). That last quote is a first-party description of exactly the failure mode this survey's *Disagreements* section is about — **an over-strong off-line correction escalating into a spin** — and it is the same diagnosis Game AI Pro ch.39 gives for a too-short look-ahead (§24) and the same one Ziggy Racer reaches via steering-authority exhaustion (§6). **The Speed Dreams driver's own law could not be retrieved; do not attribute a formula to it.**
- **GDC Vault "The Next Vector"** — the session page shows metadata only behind a login; the slide PDF returns **HTTP 403** (`ubm-twvideo01.s3.amazonaws.com/.../824050Pentheny_Graham_The_Next_Vector.pdf`) and the `media.gdcvault.com` copy is `application/pdf` (unsupported). **Its racing content is effectively available as Game AI Pro 2 ch.18 by the same speaker (§25), which is cited instead.**
- **Gran Turismo Sophy Nature PDF** — direct PDF fetch was unavailable during part of this session; the numbers above come from the author-hosted PDF mirror at `cs.utexas.edu`. If re-verifying, use that mirror or the DOI.
- **Gran Turismo / Forza / iRacing / F1 AI steering laws** — no public source states them (§15). For iRacing specifically, the only two technical statements located are the steering-behaviour attribute *"**Smoothness – A measure of the AI Driver's steering behavior**"* (https://support.iracing.com/support/solutions/articles/31000153531-ai-rosters) and an unrelated pit-lane PID: *"**The car now uses a PID controller targeting the maximum allowable pit speed.**"* (https://www.iracing.com/iracing-development-update-may-2026/).

---

# CROSS-CUTTING PATTERNS

## Which architectures dominate

**Counted across the 30+ implementations in this survey, the distribution is lopsided:**

| Architecture | Count | Instances |
|---|---|---|
| **Aim-point / pursuit angle → steering, with the aim point set by a racing line** | **15+** | TORCS berniw, TORCS bt, TORCS tutorial, Unreal RacingAI, Unity CarAIControl, Unity AIVehicleRoutingBuddy, Unreal TrafficAI, TUM example PP, F1TENTH PP, Eelis03 PP, ForzaETH PP, MAP, Radu Angelescu, Drift King, Assetto Corsa ("Steer target" + lateral offset), Game AI Pro ch.39, Game AI Pro ch.40's "runner" |
| **PID/PID-cascade on an error signal** | **7** | Game AI Pro ch.40, Unreal RacingAI, Habrador/Unity, DonkeyCar, NwliZz, Assetto Corsa (claimed), Dragon Li (claimed) |
| **Lane arbiter feeding a pursuit controller** | **3** | Game AI Pro ch.41 heat vision, Game AI Pro 2 ch.18 context steering, TORCS bt `getOffset()` |
| **Fuzzy / weighted-sensor** | **1** | Onieva TORCS 2009 |
| **Model-inverse / LUT** | **1** | ForzaETH MAP |
| **Relay / bang-bang** | **1** | Radu Angelescu |
| **Learned policy (no classical controller)** | **4+** | GT Sophy, Forza Drivatar, Unity ML-Agents racers, Driver SF's path optimizer |
| **Kinematic substitution (slide deleted by construction)** | **1** | Unreal TrafficAI |

**The single dominant architecture is: aim at a point ahead on the racing line, steer by the angle to it, and schedule the aim distance.** Game AI Pro ch.39 states the error definition that 15 sources independently reproduce:

> *"it is better to steer towards an **aiming point** some distance ahead of the vehicle, with **steering based on the angle between the vehicle's current direction and the vector between the car center and the aiming point**."*

**Pure pursuit is not a separate architecture — it is the special case where the aim point is placed by intersecting a look-ahead circle, and the gain becomes `2·wheelbase/L_d`.** This is worth stating explicitly because the literature treats them as different algorithms:

> *"If crosstrack error (e) is defined here as lateral distance between the heading vector and the goal point, then `sin α = e / L_d`. Thus the steering angle is `δ = arctan(2L sin(α)/L_d) = arctan(2Le/L_d²)`. **Pure pursuit is a proportional controller.**"*
> (https://github.com/YangyangFu/autonomous-driving-book/blob/main/book/3-trajectory-tracking/lateral-control/pure-pursuit.md)

So **the entire game-industry "aim-point angle" family and the robotics "pure pursuit" family are the same controller**; the differences are (a) how the aim point is chosen, (b) whether `sin` or the raw angle is used, and (c) whether the gain is `1/steerLock`, `2L/L_d`, `0.05`, or a PID output. **ARS's current shipped form — `atan(2·wheelbase·sin(aimError)/aimDistance)` with a dimensionless trim — is the unified form, expressed once.**

## The reference vector: body forward dominates, and the velocity reference is rare

| Reference vector for the steering error | Count | Sources |
|---|---|---|
| **Body forward / heading** | **13** | TORCS berniw, TORCS tutorial, TORCS bt, Unreal RacingAI, Unity CarAIControl, Unity AIVehicleRoutingBuddy, Unreal TrafficAI, Radu Angelescu, TUM example PP (body frame), ForzaETH MAP **shipped code**, ForzaETH PP, ForzaETH paper restatement, Game AI Pro ch.39 |
| **Velocity vector** | **3** | TUM countersteer (`-atan2(vy, max(1.0, vx))` per wheel), TUM ESC (`beta_ = atan2(vy, vx)`), Ziggy Racer (`counter-steer(sideslip)` + yaw-rate reference) |
| **Track tangent** | **2** | TORCS bt `speedangle` (**throttle gate only, not steering**), TORCS SCR folklore (`angle` sensor) |
| **Both heading and velocity named** | **1** | Game AI Pro 3 ch.17 (Driver San Francisco) — *"the differences between the current heading **and velocity** and the target position and orientation"* |
| **Traction/per-wheel velocity** | **1** | TUM CS (per-wheel, rotated by steering and toe) |

**Three findings here are load-bearing:**

1. **Body forward is the overwhelming norm (13 of ~20).** The velocity-referenced aim error is genuinely rare, and **the only three velocity-referenced implementations are (a) an autonomous-racing stability layer that measures slip rather than path error, (b) an ESC, and (c) a project that blends it in as a slide-catch term.** No game-industry source in this survey references the velocity vector for its *primary* path-following error. **ARS's velocity-referenced aim error is close to unique among the sources surveyed.**

2. **But the original MAP paper — the source ARS would most likely be compared against — also specifies the velocity vector**, and only ForzaETH's shipped code and restatement changed it to heading. **So ARS matches the MAP paper and contradicts MAP's shipped implementation.** The discrepancy is inside the same research group (§8).

3. **The track-tangent reference appears only on throttle gates and in sensor-based folklore, never on the steering error** in any source surveyed. Two independent stacks confirm this explicitly: *"Track tangent is never the steering error reference in either stack"* (ForzaETH + TUM). **This is a strong negative result.** ARS's `speedangle`-style velocity-vs-tangent quantity would be, in TORCS bt, a **throttle** input — the same primitive on a different channel.

## Look-ahead: speed-scheduled nearly everywhere it is quantified, and always affine

| Source | Look-ahead law | Constants |
|---|---|---|
| TORCS bt / berniw / tutorial / **`damned`** | `LOOKAHEAD_CONST + v·LOOKAHEAD_FACTOR` | **17.0 m + 0.33·v**, plus a slew floor `oldlookahead − v·dt` |
| **TORCS classic** | `MIN(const·1.5 + 0.45v, const + 0.015v²)` | `LOOKAHEAD_CONST = 18.0`; **a sub-linear cap** — 27 + 0.45v until the quadratic takes over |
| Assetto Corsa | `BASE + SPEED_GAIN·v` | BASE 18.6–26.6 m, SPEED_GAIN 0.10–0.8 |
| ForzaETH MAP | `clip(q_l1 + v·m_l1, max(t_clip_min, √2·|d|), t_clip_max)` | 0.9 / 5 m; m 0.55, q −0.03 (paper tuned: 0.6 / −0.18) |
| MAP reference repo | `clip(m_map·v_target + q_map, t_clip_min, t_clip_max)` | 0.3 / 5 m; m 0.3, q 0.15 |
| Eelis03 pure pursuit | `clip(0.15·v + 2.0, 2.0, 25.0)` | gain 0.15 s, offset 2.0 m, band 2–25 m |
| **USR default** | `5.0 + speed_x/10` m, then **ballistic re-projection** | 5 m floor, 0.1 s term |
| TUM example PP | `min_lookahead_distance_lat_m + 0.1·v + k_κ/|κ|` | 1.0 m + 0.1 s (curvature term ships 0.0) |
| **TUM shipped YAML** | same form | **0.1 m + 0.45·v + 0.00004/|κ|** |
| **simplix** | `oLookAhead + oLookAheadFactor·v`, factor capped, **preview ÷ (1 + skill/24)** | 5.0 m + 0.05 s, factor cap 0.2 |
| Unreal RacingAI | `MapRangeClamped(v_kmh, 30, 100, 2, 15)` | 2 m at ≤30 km/h → 15 m at ≥100 km/h |
| **SuperTuxKart** | **track-width corridor search** — furthest node centre reachable without the kart's half-width leaving the track | emergent, not tuned; `steps = len/kart_length` clamped to [3, 1000] |
| DonkeyCar | `PATH_LOOK_AHEAD` / `PATH_LOOK_BEHIND` **in waypoint indices** | 1 / 1 points |
| Radu Angelescu | `EBASICAI_LOOKAHEAD_DISTANCE` **in node count** | constant |
| Unreal TrafficAI | `PathFollowingConfig.LookAheadDistance` | constant, applied **along the velocity vector** |
| Unity CarAIControl | **none** (aims at a target transform) | — |
| Unity AIVehicleRoutingBuddy | **none** (aims at the current waypoint) | — |
| GT Sophy | **~6 s of travel**, span ∝ velocity | 60 points per edge |
| Pomerleau (human-driver anchor) | **2–3 s of travel** | empirical |
| Drift King | **ballistic: `p + v·t + 0.5·a·t²`** at **0.5 s and 1.2 s** | two horizons, used differentially |

**Patterns:**
- **Affine in speed, not `speed/grip`.** **Not one source in this survey ties look-ahead to grip.** Sixteen-plus implementations scale it with speed; one with **track width** (SuperTuxKart); one with **driver skill** (simplix). ARS's `speed / grip × leadScale` is the outlier.
- **A sub-linear cap is a real design choice, not an oversight.** TORCS classic's `MIN(27 + 0.45v, 18 + 0.015v²)` grows fast then flattens; at 30 m/s the linear form would give 40.5 m and the cap gives 31.5 m. **This is the only preview in the survey that is deliberately not affine in speed**, and it is cheap to reuse.
- **Floors and caps are common but rarely universal.** ForzaETH (both), Eelis03, TUM, Unreal RacingAI and simplix all clip to a band; TORCS bt and DonkeyCar do not state one; MAP's reference repo states the reason for each bound explicitly.
- **The floor is a stability device, stated as such.** ForzaETH: *"`t_clip_min` is the minimum distance for the lookahead distance in meters **to prevent oscillations**"*; and the tuning rule *"increase `t_clip_min` until the car doesn't oscillate anymore."*
- **Three sources have a non-speed look-ahead term**: ForzaETH's `√2·|lateral_error|` floor (*"to avoid ultraswerve when far away from mincurv"*), TUM's `k_κ/|κ|` curvature extension (*"more lookahead on straight"*, ships at ~0), and SuperTuxKart's track-width corridor (the whole preview).
- **Preview *rate* is separately guarded in two drivers**: TORCS `bt` and `damned` both floor the preview against `oldlookahead − v·dt` (*"Prevent 'snap back' of lookahead on harsh braking"*). **This is a stability rule on the preview's rate, not its value** — the only such guard in the survey, and worth noting because it means the *preview* is treated as a state with dynamics rather than a memoryless function of speed.
- **The time-based rule is the human-factors anchor and it is 6–20× larger than the engineering constants.** Pomerleau: 2–3 s. Sophy: ~6 s. TORCS: 0.33 s. TUM shipped: 0.1 s (0.45 s in a commented YAML variant). Eelis03: 0.15 s. USR: 0.1 s. **The game-dev and F1TENTH constants sit an order of magnitude below the human and RL observation windows**, and no source addresses the gap. The likely reconciliation is that the human/Sophy windows are *perceptual/planning* horizons while the controller constants are *tracking* horizons — but **no source says this**, so it remains an open question (see *Disagreements*).

## Gain scheduling: universal in intent, and contradictory in direction

**Every source that quantifies a gain schedules it in *some* way — but the schedule is far more often on the look-ahead than on the gain, five distinct schedules appear, and they disagree about which way the gain should move.**

**The dominant choice is to schedule the look-ahead and leave the gain flat.** TORCS `berniw`/`bt`/`damned` (§1, §2, §35c) and USR default (§35a) **never touch the gain at all** — it is a flat `1/steerLock`, and the *effective* gain moves only because the aim distance moves. Assetto Corsa keeps `STEER_GAIN` flat per car. The Unreal RacingAI plugin keeps `Kp/Ki/Kd` fixed. **So "gain scheduling is universal" would be too strong a claim:** what is universal is *preview* scheduling; explicit gain scheduling is the minority.

**The five actual schedules:**

1. **Schedule the look-ahead, hold the gain** (the dominant choice): TORCS family, USR default, Unreal RacingAI, Assetto Corsa, ForzaETH PP, Eelis03.
2. **Schedule the gain up with speed** — Game AI Pro ch.40: *"at low speed the vehicle may require **much larger** K values"*; ForzaETH's undocumented `steering_angle *= clip(1 + v/10, 1, 1.25)`.
3. **Schedule the gain down with speed** — Unity Standard Assets (`maxSteerAngle` 28° → 23 %, rate ×0.5); NwliZz (`maxSteerAngle − (v/v_max)·factor`); Unity's official tutorial (`steeringRange` 30° → 10°); ForzaETH's documented `k_speed = 1 − 0.2·ramp(v)`; **USR `SteerMod`** twice, with both reasons stated verbatim (`"// decrease steergain as speed is slower than targetspeed and we're either avoiding or have just recovered from avoiding."` and `"// decrease steergain at high speed to stop bouncing over curbs"`).
4. **Divide by speed** — Stanley: `atan(gain · e / (softening + v))`, purpose stated as *"finite at standstill"*; **USR `SteerMod`'s cross-track gain** is also `300/(carspeed+300)/15`, on exactly that principle.
5. **Schedule by load/grip, and by skill** — Ziggy Racer: `load_factor^0.705` folded into both the steering feedforward and the cornering speed; **simplix shrinks the look-ahead with driver skill** (`oLookAhead = oLookAhead / (1+oSkillGlobal/24)`) and gates all feedback off below a slow-speed threshold (`if (oCurrSpeed < SLOWSPEED) return Angle;`).

**The contradiction is real and unresolved.** Game AI Pro ch.40 says low speed needs *more* gain; Unity's and NwliZz's shipped code *reduce* steer authority with speed; USR `SteerMod` explicitly reduces gain at high speed *and* below target speed (i.e. a **band-pass** on gain); and ForzaETH's own code contains **both** a downscale above 7–8 m/s and an upscale that saturates at 2.5 m/s, which **fight each other** with no comment reconciling them. There is no consensus, and **the only sources that derive their direction state a purpose, not a proof** — USR `SteerMod`'s reasons are *"stop bouncing over curbs"* and *"we're either avoiding or have just recovered from avoiding."*

**The one derivation that exists is about the look-ahead's effect on gain**, and it is ForzaETH/MAP's:

> *"the evolution of the lateral distance… could be approximated by a second order system with a time constant τ = L_d/v_x and natural frequency **ω_n = √2·v_x/L_d**… **For stability, the natural frequency of the controller must be smaller than half the frequency of the entire vehicle dynamics including delays.** Tests showed that for higher speeds this criterion was no longer met and the system became unstable. To address this, L_d was scaled with the velocity with the affine mapping L_d = m + q·v_ctrl."*

That is *why* look-ahead must grow with speed: not for comfort, but to hold the guidance loop's natural frequency in a fixed relationship to the vehicle-plus-delay bandwidth.

**Four sources name the hazard of scheduling at all.** Game AI Pro ch.40: *"Be careful with this, though, because **if the coupling is too great, a hidden positive feedback loop can be set up, resulting in instability.**"* Ziggy Racer documents exactly that failure in the feedforward channel: feeding the *planner's own* curvature back in *"created a **planner ↔ tracker limit cycle** … the two rang together into a bang-bang steering oscillation."* Game AI Pro ch.40 again: gain scheduling *"can become particularly difficult to tune when the values are dependent upon multiple factors."* And **SuperTuxKart's author notes that the steering law's aim points *"would actually force the kart off track"* about as often as not, and ships it anyway because the effective schedule — re-solved every frame against real geometry — beats the alternatives.**

**A sixth, quite different lever for the same problem: predistort the reference.** USR default folds the yaw-rate damping *into the aim bearing* (`targetAngle - (car->_yaw + car->_yaw_rate/15)`) rather than adding a D term or a separate yaw-rate term. **This is neither schedule 1 nor schedule 3 — it is a way to get damping inside a flat-gain P law**, and it is the cheapest option in the survey for a controller that has no room for another term.

## Anti-windup and integral treatment

Integral terms appear in only **five** sources, and four of them guard the integral:

| Source | Integral handling |
|---|---|
| Game AI Pro ch.40 | **Both remedies named**: *"reset the integral error based on external events"* and *"**capping the integral error** may also be desirable to limit the effects of the error memory"*; also a rolling-average integral: *"on each frame the current integral is reduced by (1–T%) and T% of the current error is added back"* |
| Habrador/Unity | Leaky rolling-average accumulator, `averageAmount = 20f` frames — **not** `Σe·dt` |
| DonkeyCar | No clamp; `PID_I` used to correct *mechanical* asymmetry (*"if one wheel is slightly smaller in diameter than another"*) |
| Ziggy Racer | `cte_int` clamped **±3**; *"integration is **suppressed while the actuator saturates**… an optional steer-clip anti-windup bleeds the integrator when the wheel is pinned at full lock and the integrator is winding the same direction"* |
| TUM ESC | `max_integrator_yaw_moment = ±1325.0` on both PIDs; **conditional integration** anti-windup, not back-calculation |
| Unreal RacingAI | **None** — raw accumulator, no leak, no clamp (mitigated by a tiny `Ki = 0.001`) |
| NwliZz | **Reset on priority change only** — an event-driven reset, no clamp |

**Pattern: the game/R&D sources that ship an I term almost always clamp it or leak it, and the two that ship it unguarded use a tiny `Ki` or an event reset instead.** ARS's PID having no D term is the *only* structural difference from the majority — but see the disagreement on D below.

## Slide / drift handling: the field that is almost always missing

**This is the survey's strongest negative result.** Sorted by what the implementation actually does:

| Strategy | Sources | Mechanism |
|---|---|---|
| **Nothing at all** | TORCS berniw, TORCS bt, TORCS tutorial, TORCS SCR folklore, Unity CarAIControl (AI layer), Unity AIVehicleRoutingBuddy, Unreal RacingAI, Onieva fuzzy, DonkeyCar, Habrador, Radu Angelescu, ForzaETH MAP, F1TENTH PP, Eelis03 PP, Game AI Pro ch.38/39/40 | — |
| **Countersteer from measured slip** | **TUM countersteer** (the only one), Ziggy Racer (yaw-rate + sideslip detectors) | `δ += k·(α_f − α_r)`, gated on `|α_r| > |α_f|` |
| **Grip ceiling enforced by table exhaustion** | ForzaETH MAP | LUT truncates at the first NaN past *"an unstable drift"* |
| **Make the plant forgiving** | **Assetto Corsa** (20 % grip overhead + *"some form of (artificial) stability control… so the AI's weird and twitchy inputs don't make them spin"*), **Automobilista 2** (AI strength = *"a more straight-forward AI Grip multiplier"*), **rFactor 2** (*"AI max steering lock multiplier to deal with their lower quality physics"*) | change the vehicle, not the controller |
| **Delete the slide kinematically** | Unreal TrafficAI | heading rewritten from wheelbase geometry each tick; `velocity = heading · |velocity|` |
| **Simulate feasibility instead** | Game AI Pro 3 ch.17 (Driver SF) | *"a simplified 2D physics simulation… driven by the same inputs as the game vehicle"* |
| **Separate drift state with handbrake + boost** | **Drift King / Scully** | Line/Drift FSM; *"apply handbrake until the vehicle drift angle is high enough to sustain the drift without further handbraking"* |
| **Prefer braking over more steering** | Game AI Pro ch.39 | *"if extra steering would force the vehicle beyond the grip limit then **braking might be preferred**"* |
| **Penalise it in the reward** | GT Sophy | *"penalties if it went out of bounds, hit a wall or lost traction"* |
| **Rate-boost the actuator under opposite lock** | Unity Standard Assets `CarController.cs` | `if (sign(steerInput) != sign(CurrentSteerAngle)) currentSteerSpeed *= 4f` — *"for faster response"* |
| **Clamp exempting countersteer** | **ARS only** | `ApplySteerLimits` applies the speed-based allowance only when steer and yaw share sign |

**Count: 17 sources do nothing about slides; only 3 control them; the rest avoid the problem by changing the plant, the model, or the state space.** Of the ~9 that address it at all, **6 address it by *not having* slides** rather than by controlling them.

**Three important sub-patterns:**
- **Exactly one implementation detects a slide and *steers into it*: USR default's `getAvoidSteer()`** (§34a) — gated on the projected heading still exceeding the velocity angle, with a gain that **grows with the size of the predicted angle** and **grows further under braking** (`(0.1 + |nextangle|/6)` and `(1 + MAX(1.0, 1 − accel_x/5))`). Everything else either measures slip for a countersteer, or measures it for a *speed* reduction, or does nothing.
- **Three implementations measure sideslip and respond by slowing rather than steering**: TUM's ESC (brakes, never the wheel — §9), simplix and USR `SteerMod` (*"sideslip drives a speed cut rather than more steer"*). **This is the "understeer → slow down" half of the ESC idiom with the countersteer half replaced by nothing**, and it is common.
- **No source in this survey clamps the steering and *exempts* countersteer except ARS** — but **USR gets the same effect from the opposite direction**, by making the clamp itself *wider under rear skid* (§Limits). The TUM and Unity implementations clamp everything uniformly; ForzaETH clamps nothing absolutely. **So "the limit must not bind the correction that saves you" is a recognised design intent in two independent codebases, reached by two different mechanisms.**
- **No game-industry source uses a velocity-referenced error as its slide mechanism.** Instead they use: slip angles (TUM), a sideslip detector (Ziggy), a velocity-vector *correction* (NwliZz's `steerHelper` — *"the whole velocity vector is rotated by the car's frame-to-frame yaw change, scaled by `_steerHelper × |steer input|`"*), a separate drift state (Drift King), or a yaw-rate reference (Ziggy's `r_des = v·κ`, TORCS berniw's `omega = v/R`). **ARS lands the slide in the error itself; every other source measures it as a separate quantity and adds a correction.**

## Limits and actuation: what the field actually does

| Limit type | Sources |
|---|---|
| **Slew / rate limit** | **ForzaETH MAP** (0.4 rad/cycle at 40 Hz ≈ 16 rad/s), **Unity CarAIControl** (`MoveTowards` with a speed-scheduled rate), Unity Standard Assets (4× under opposite lock), Unreal RacingAI (none), Ziggy Racer (*"slew-limited stick"*, rate unstated), ARS (360°/s) |
| **Absolute angle clamp** | Unity Standard Assets (28° → 23 %), Unity official tutorial (30° → 10°), NwliZz (`maxSteerAngle` − speed), TUM (±0.43 rad), Unreal TrafficAI (`MaxSteeringAngle`, speed-independent), Habrador (40°), Onieva (±0.785 rad = the API range) |
| **Speed-dependent max steer** | Unity Standard Assets (**quadratic** bias), NwliZz (**linear**, but *response rate increases* with speed), Unity official (linear `Lerp`), ForzaETH (`k_speed`, a **scale not a bound**) |
| **No clamp at all** | **ForzaETH MAP** (slew only), Unity CarAIControl AI layer (input space only; the clamp lives in `CarController.cs`), TORCS (relies on the `[-1,1]` API contract) |
| **Grip/LUT ceiling instead of a clamp** | ForzaETH MAP |
| **Steering-lock multiplier** | **rFactor 2** — *"Add AI max steering lock multiplier to deal with their lower quality physics which can be off at times (defaults to 1.0)"* |

**Three patterns:**
1. **Angle clamps and rate limits are usually implemented in the *plant* (the `CarController`), not the AI.** Unity's CarAIControl AI side clamps only to `[-1,1]`; the real 28°→6.4° collapse is in `CarController.cs`. TUM's ±0.43 rad is in the countersteer layer, which is a wrapper, not the tracking controller. **This is the same separation of concerns as ARS's `ApplySteerLimits` + `TranslateSteerToInput` split, and it is standard practice.**
2. **The *reason* for a speed-dependent steer limit is essentially never stated.** Unity says only *"Reduce motor torque and steering at high speeds for better handling"*; the field comment says *"the reduction in steering angle at max speed."* **The only source that gives a *mechanical* justification is TUM, and it is about normalising the error, not the maximum** — Stanley's `atan(gain·e/(softening + v))`, *"softened by a speed term so the gain does not diverge at low speed."*
3. **Rate limits are rarer than angle clamps — and exactly one shipped racing game in the survey publishes one *and* states why.** SuperTuxKart's is `max_steer_change = dt/m_time_full_steer`, documented as *"Time for AI karts to reach full steer angle (**used to reduce shaking of karts**)"* and *"helps somewhat to make AI karts more 'pushable' (**since otherwise the karts counter-steer to fast**)"* (§34). **That rationale is the opposite of a slide-response argument: the limit exists partly to blunt the countersteer the AI would otherwise apply.** ForzaETH logs its clip but never explains the number; Unity's `Mathf.MoveTowards` is described as *"simulate the time it takes to turn the steering wheel"* (Habrador, same idea). **No source states a target steer rate in rad/s except ForzaETH's implied ≈16 rad/s, and no source ties the rate limit to a stability criterion** — even though Eelis03 demonstrates that *"both geometric laws demand more steering rate than the 0.6 rad/s actuator can deliver. **The resulting lag turns the loop into a limit cycle**."* **That is a rate limit causing oscillation, stated with numbers, and it is the best available evidence that a steer-rate limit deserves a stability-based justification rather than a comfort-based one.**
4. **USR stacks *three* speed-dependent slew limits and widens them under rear skid** (§34a): a per-frame slew `limit = ((90 − clamp(speed_x, 40, 60)) / 120) · SmoothSteer`, a deviation-around-raceline clamp, and an asymmetric `smoothSteering()` limit `lstlimit = MAX(40, 80 − speed_x)·0.004 − MIN(0, MAX(−0.5, angle_error))` — plus a speed-dependent maximum steer that **opens up when the rear is sliding.** **This is the only implementation in the survey that treats the steering limit as a slide-dependent quantity in the permissive direction**, which is the same design intent as ARS's countersteer-exempt limiter (§9) — arrived at independently, in 2002-era TORCS code.

## What the sources say about tuning — the consensus list of failure modes

Aggregated, with the source that states each:

| Failure mode | Stated by |
|---|---|
| **Look-ahead too short → weaving → build-up → spin** | Game AI Pro ch.39; MAP §II/III-D; ForzaETH tuning README; Assetto Corsa community; Eelis03 (*"pure pursuit… hunts for the entire run"*) |
| **Look-ahead too long → corner cutting** | Game AI Pro ch.39; MAP §II; ForzaETH README; TUM config comment; a course deck (*"Looking too far ahead during a sharp turn will cause A.I. to cut corners"*) |
| **P too high → zig-zag on straights** | TORCS SCR folklore (`steer_kp = 30` → 15); AMS2 (*"Fixed AI tendency to oscillate steering erratically running down longer straights"*); DonkeyCar (*"over-react to small changes in the path and may start turning in circles"*); Game AI Pro ch.40 (*"weaving across the line uncontrollably with larger and larger amplitude"*) |
| **P too low → steady-state offset / won't turn in** | Game AI Pro ch.40 (*"the car will settle a fixed distance away from the racing line"*); DonkeyCar (*"car will not turn enough when it reaches a curve"*) |
| **Integral → overshoot from residual memory** | Game AI Pro ch.40 |
| **Gain scheduling itself → hidden positive feedback** | Game AI Pro ch.40 |
| **Feedforward closing an unintended loop → limit cycle** | Ziggy Racer (planner↔tracker) |
| **Over-strong off-line correction → escalation → spin** | Speed Dreams dev list |
| **Saturation → no steering lever helps; reduce demand instead** | Ziggy Racer (crest case: slew limiting was *"catastrophic, 40/1k"*; the fix was slowing the approach) |
| **Rate-limited actuator → limit cycle** | Eelis03 (`0.6 rad/s` limit; PP *"leaves the path entirely"*) |
| **Bad track data → jerky steering** | rFactor 2 (*"Fixed waypoint lookahead to fix jerky steering motion"*; *"sudden changes in length between adjacent segments will lead to difficulties"*) |
| **Overfitting the controller to one track/corner** | Drift King (*"overfitting to the first corner… absolutely suck at the remainder of the track"*) |
| **Uncontrolled variables during development** | Drift King (*"the first and biggest problem was failing to control variables"*) |
| **Noise → D-term spikes** | Game AI Pro ch.40 (*"the derivative term can fluctuate in an undesirable manner… **this is effectively adding another integrator**"*) |
| **Balancing the AI by cheating grip/power** | Assetto Corsa (20 % grip overhead), AMS2 (grip multiplier), rF2 (lock multiplier) — all *deliberate*; Driver SF (*"We were not allowed to cheat"*) — the opposite policy |

**And the most useful methodological statement in the survey, from the Simulated Car Racing Championship's own record (via the AI & Games textbook):**

> *"**A general trend observed over the course of the competition was that the winning agents incorporate more and more domain knowledge as hand-coded mechanisms, with learning algorithms generally only used for tuning parameters of these mechanisms.**"*

That is the competition record's verdict: **hand-coded structure beat learning for steering, with learning used for the gains.** It is the strongest available support for the architecture this survey documents.

---

# DISAGREEMENTS / OPEN QUESTIONS

Where sources genuinely conflict. Each is flagged with what would settle it.

### D1. Body-forward vs velocity-vector reference — an unresolved fork, and one group contradicts itself

**The conflict:**
- **13 sources** reference the steering error to **body forward/heading** (§24, §16, §17, §19, §21, §30, TORCS, ForzaETH code).
- **3 sources** reference it to the **velocity vector**: TUM countersteer and ESC (slip *is* the velocity-frame angle), Ziggy Racer (sideslip into the countersteer term), ARS.
- **The ForzaETH MAP controller disagrees with itself.** The original ICRA paper: *"η is the angle between the **velocity vector** and lookahead point."* The race-stack paper restatement: *"η denotes the angle between the **heading** and the lookahead point."* The shipped code: **heading** (`[-sin(yaw), cos(yaw)]`).
- **The only game-industry source that names both** is Driver San Francisco (*"the differences between the current heading **and velocity**"*) — and it does not say how they combine.

**Why it matters:** a velocity-referenced error carries sideslip **as error** and therefore needs no separate slide handling; a nose-referenced one does not, which is *why* every nose-referenced implementation in this survey either omits slide handling or adds a separate term. **The two choices predict the slide-handling field almost perfectly** (§ *Slide / drift handling*: the nose-referenced majority does nothing; the velocity-referenced minority either *is* a stability system or adds a countersteer term).

**What would settle it:** a source that runs both against the same car and track. ForzaETH has the testbed (PP vs MAP, 4× lateral error improvement) but changed the reference **and** the tire model at once, so the two effects are confounded. **No source in this survey isolates the reference-vector choice.**

### D2. Gain direction with speed — directly contradictory, and mostly avoided by scheduling the preview instead

- Game AI Pro ch.40: *"**at low speed the vehicle may require much larger K values** to get it to start to move, compared to when it is running at high speed."*
- Unity Standard Assets, Unity's official tutorial, NwliZz, and ForzaETH's `k_speed`: **reduce** steering authority as speed rises.
- Stanley: **divide** the gain by `(softening + speed)` to keep it finite at standstill; USR `SteerMod`'s cross-track term does the same (`300/(carspeed+300)/15`).
- **USR `SteerMod` reduces gain at *both* ends** — below target speed while avoiding, and above 60 in `carspeed` units — i.e. a **band-pass on gain**, the only such schedule found.
- ForzaETH MAP: **both at once** — `k_speed` downscales above 7–8 m/s while `clip(1 + v/10, 1, 1.25)` upscales, saturating at 2.5 m/s, with **no comment reconciling them**.
- **TORCS `berniw`/`bt`/`damned`, USR default, Unreal RacingAI, Assetto Corsa: schedule nothing at all** — flat gain, and the effective gain moves only through the preview. **This is the majority position**, and it means the "contradiction" is largely confined to the sources that *do* schedule a gain explicitly.
- Ziggy Racer: schedule by **load**, not speed (and by fitted grip-vs-speed for the cornering cap).
- simplix: schedule **look-ahead by driver skill**, and gate all feedback off below a slow-speed threshold.

**The reconciliation that seems likely, but that almost no source states:** these are schedules on *different quantities*. Reducing the **maximum steer angle** with speed (Unity) is a plant limit for stability; increasing the **error-to-steer gain** at low speed (ch.40) is a deadband/authority argument; and growing the **look-ahead** with speed (TORCS, MAP, AC) reduces the *pursuit* gain. **A complete controller can do all three without contradiction, and ARS does** (fixed `SteerTrim`, geometric gain `∝ 1/L_d`, separate `ApplySteerLimits` allowance). **USR `SteerMod` is the only surveyed implementation that states two of the three separately and gives a reason for each** — and it ships disabled. That ForzaETH ships two opposed schedules suggests the industry does not consistently distinguish them either.

### D3. Look-ahead time constant: 0.1 s (engineering) vs 2–6 s (human and RL)

- **Tracking constants:** TORCS 0.33 s; TUM shipped 0.1 s (0.45 s in a commented YAML variant); Eelis03 0.15 s; ForzaETH `m_l1` 0.55–0.6 s.
- **Perceptual/planning constants:** Pomerleau 2–3 s of travel for stable human steering; GT Sophy *"approximately the next 6 s of travel."*

**A 6–20× gap, and nobody addresses it.** The likely-but-unstated explanation is that the two are different quantities — a *tracking* preview vs a *perceptual/planning* horizon — and Sophy's window is an observation encoding rather than a control target. **Also note the discrepancy cuts both ways:** TUM's *code default* is 0.1 s but its *shipped YAML and the config comment* say 0.45 s, and the code's `min_lookahead_distance_lat_m` default (1.0 m) differs from the YAML's (0.1 m) — **so even one repo contains a 4.5× disagreement about its own look-ahead time, depending on which file is authoritative.** That is a caution for anyone reading constants from a repo without checking which layer wins.

### D4. Does the gain belong in the goal or the error? (The "pure pursuit is a P controller" problem)

Two mathematically identical formulations are used as if they were different designs:

- **Angular:** `δ = heading_error_to_aim / steerLock` or `δ = k·α`.
- **Cross-track:** `δ = arctan(2·L·e / L_d²)`, i.e. **P on cross-track error with gain `2L/L_d²`.**

**Sources disagree about which is "the" pure pursuit.** The F1TENTH paper derives the cross-track form (`δ = tan⁻¹(2·sin(η)·l_wb/L_d)`) and then says *"Pure Pursuit can be derived from this equation"*. The autonomous-driving-book states both and insists *"Pure pursuit is a proportional controller."* Game AI Pro ch.40 treats the error as a **perpendicular distance** and the controller as a **PID**. Eelis03 implements the chord form and calls the derived gain *"the standard remedy."*

**The practical consequence is a units trap:** switching between formulations silently changes the gain's units, and a constant tuned in one frame is wrong in the other. **ARS hit exactly this**: the docs record that `SteerTrim`'s ini key is deliberately **not** `SteerP`, because the unit change from constant-gain to geometric required retiring a stale value, and the repair pass **snaps an in-list numeric key to the nearest offer** rather than resetting it. **No surveyed source discusses this migration hazard** — which suggests most of them never changed formulation after shipping, or silently retuned.

### D5. Should the slide be controlled, avoided, or designed out? — four mutually exclusive answers, all shipping

| Answer | Source | Statement |
|---|---|---|
| **Control it** | TUM countersteer | `δ += k·(α_f − α_r)` gated on rear dominance; separate layer wrapping an arbitrary controller |
| **Prevent the model from needing it** | ForzaETH MAP | Pacejka **inside** the controller — and TUM explicitly rejects this: *"neglecting nonlinear tire dynamics represents a **strategic advantage**… it enhances system robustness against external disturbances and model mismatches"* |
| **Make the plant forgiving** | Assetto Corsa (20 % grip + artificial stability), AMS2 (grip multiplier), rF2 (lock multiplier) | *"so the AI's weird and twitchy inputs don't make them spin"* |
| **Forbid it** | Driver San Francisco | *"We were not allowed to cheat by giving the AI vehicles more power or tighter grip"* |
| **Delete it kinematically** | Unreal TrafficAI | heading rewritten from geometry, velocity re-aligned to heading every tick |
| **Simulate it away** | Driver SF | *"a simplified 2D physics simulation… driven by the same inputs as the game vehicle"* |
| **Give it its own state and a handbrake** | Drift King | Line/Drift FSM |

**ForzaETH and TUM are the sharpest conflict**, and they are both competition-winning autonomous-racing stacks in the same survey: **MAP puts the identified nonlinear tire model inside the controller and drives tracking error to zero; TUM deliberately excludes it for robustness and leans on a 100 Hz loop.** Both outperform their baselines. **Neither resolves the other's case.**

### D6. Is the aim-point lane a steering input or a separate decision layer? — three sources, three answers

- **Game AI Pro ch.41 (heat vision)** — the lane is chosen by a cost field and *"converted into a track offset that can then be passed to the steering controllers"*; the steering law is not its business.
- **Game AI Pro 2 ch.18 (F1 2011 context steering)** — the lane is chosen by a context-map walk, and the steering loop is **explicitly delegated** to *"a low-level driver system followed by a hand-placed racing line spline."*
- **TORCS bt** — the lane is a **lateral offset added to the aim point inside the steering function** (`myoffset`, `getOffset()`), with no separation.
- **ARS** — resolved into `_steerAimPoint` by the lane systems, then a single PID.

**Game AI Pro ch.18's stated reason for separating is architectural, not mathematical**: F1 2010's monolithic version *"had decomposed into an old-school sequence of if/else blocks with a thin steering behavior wrapper and was a maintenance nightmare."* **But TORCS bt demonstrates that folding the lane into the aim point works and is far simpler.** No source compares the two.

### D7. Is a PID on cross-track error adequate, or is feedforward mandatory?

- **Game AI Pro ch.40** presents a PID on perpendicular line distance as *"good enough"* and says predictive control / MPC is *"too big for games."*
- **Ziggy Racer** states flatly that proportional-only control fails and requires feedforward: *"Both pedals were originally pure-proportional and so needed a **standing error** to hold pedal… The fix is a **brake feedforward**."* Its steering law is **feedforward-first** (curvature) with the PID as trim.
- **TUM's example PP** ships `enable_lat_feedforward_perc = 0.5` in the YAML — **half** the command is curvature feedforward.
- **ForzaETH MAP** is *entirely* feedforward in this sense: `a_lat = 2v²/L_d·sin(η)` **is** the curvature demand, inverted through the model.
- **Eelis03** shows a pure-PID-free geometric law is competitive: PP mean RMS `0.1482 m` vs MPC `0.1208 m`, and *"a geometric law is a defensible choice and it is two orders of magnitude simpler."*

**So the field splits: PID-on-cross-track (ch.40, Habrador, DonkeyCar) vs feedforward-first (MAP, TUM, Ziggy, Eelis03's MPC comparison).** The strongest specific argument for feedforward is the Ziggy standing-error observation, and the strongest specific argument against pure PID is that a PID on cross-track error has a **curvature-blind steady-state error at speed** — which no PID source in this survey addresses, and which ch.40 handles only by suggesting the gains be varied with speed, i.e. exactly the coupling ch.40 itself warns creates hidden feedback.

### D8. Is a D term on the steering error useful or harmful?

- **Supports D:** Game AI Pro ch.40 (*"Adjust Kd to reduce any overshoot"*); DonkeyCar (*"useful in reducing oscillations and overshoot"*); TUM ESC (both PIDs have `kd` of 0.6 and 1.0); Unreal RacingAI (`Kd = 0.05`); TORCS SCR folklore (added a damping term after oscillation).
- **Warns against D:** Game AI Pro ch.40, in the same section: *"If the input data (R) is noisy, the derivative term can fluctuate in an undesirable manner… **this is effectively adding another integrator.**"*
- **Gets damping elsewhere instead:** TORCS berniw/bt (a **yaw-rate error term**, gain 0.1); Ziggy Racer (a **filtered** yaw-rate comparison, `r_des = v·κ`); TUM PP and Eelis03 (a **low-pass filter on the curvature**, `curvature_filter_Ts` 0.05 s / 0.005 s); Unity CarAIControl (a **speed-caution** term from yaw rate); NwliZz (a yaw-acceleration stabiliser in the plant).

**Pattern: almost every implementation that needed damping added it on yaw rate or on curvature, not as a D term on the path error.** ARS's docs record *"Snap-oversteer counter (TODO) — no D-term exists to catch yaw-rate spikes."* **The survey suggests the field's answer is a yaw-rate term, not a D term** — which is what ARS's *disabled* inner yaw loop was, and what TORCS berniw has had since 2002.

### D9. Should the AI be given help the player does not have?

- **Yes, deliberately:** Assetto Corsa (*"AI cars have a 20% tire grip overhead vs the player and some form of (artificial) stability control"*); AMS2 (*"AI Strength… a more straight-forward AI Grip multiplier"*); rFactor 2 (*"AI max steering lock multiplier to deal with their lower quality physics"*, plus *"AI Spring Rate Multiplier"* and *"Automatic AI front/rear roll stiffness calculation… A compensation of sorts"*).
- **No, by policy:** Driver San Francisco (*"We were not allowed to cheat by giving the AI vehicles more power or tighter grip between the tires and the road"*); Forza's Drivatar difficulty is trained, not multiplied.
- **Player-visible complaint:** the Forza AI's grip advantage is a recurring community grievance (*"they have straight up nonsensical levels of grip… they never seem to break loose even on wet grass"*, https://www.reddit.com/r/ForzaHorizon/comments/1th1a9c/why-the-drivatars-in-forza-are-so-awful/).

**The mechanism matters for a survey reader:** Assetto Corsa's overhead is applied **"within braking zones"** (`ULTRA_GRIP`'s comment), and rF2's is on **steering lock**, not grip. So the same "AI cheat" idea is implemented on three different quantities (lateral grip, longitudinal grip, steering authority) — and **none of the three sources explains why it chose that channel over the others.** ARS's approach (separate `Handling.Gravity`, TCS, and a speed-dependent steer allowance) is closer to rF2's on the steering side, which is worth noting: **a steering-lock/allowance compensation and a grip compensation are not equivalent**, because only the former leaves the tire model honest.

### D10. Does look-ahead belong on grip at all? — a systematic gap, not a disagreement

**Sixteen-plus sources schedule look-ahead with speed. Zero schedule it with grip or friction.** ARS's `LookAhead.SteerRef = speed / grip × leadScale` is the only grip-scaled preview in the survey — including the pure-robotics sources, which is notable because grip *is* available to several of them (MAP has an identified Pacejka model; TUM estimates per-wheel slip). **The one source that schedules preview with anything other than speed schedules it with *track width*** (SuperTuxKart's corridor search, §34) — and one schedules it with **driver skill** (simplix, §35e). So the field has three preview drivers — speed, track geometry, and skill — and grip is not among them.

**The strongest theoretical statement against `speed/grip` is in the ForzaETH derivation**: the stability criterion is `ω_n = √2·v/L_d` against the vehicle-plus-delay bandwidth, and **grip does not appear in it**. So a grip-scaled look-ahead changes `ω_n` without changing the bandwidth it must respect — which is why ForzaETH scales with velocity alone.

**But there is a counter-argument the survey cannot resolve from sources:** at low grip a car *does* need to start turning earlier for the same curvature, which is a path-tracking argument rather than a stability one — and the two are different criteria. **ARS's own notes record that this was the observed problem** (`07d5e57`: the grip-scaled lead was replaced by a **manual grip→lead scale** because *"the grip trend was driving corner cutting"*) — i.e. ARS **tested the grip dependence and found it behaved wrongly in the direction the theory predicts it should help.** That is a real, in-game data point that no surveyed source replicates. **An A/B of `speed/grip` vs `speed`-only look-ahead against ARS's cross-track RMS and steer-reversal metrics would settle it, and would be the single highest-value experiment this survey identifies.**

---

## Methodological note: how to read a constant from these sources

Three concrete traps found in this survey, all of which produced a wrong answer during research:

1. **Paper vs shipped code can disagree on the sign of the correction** (TUM countersteer: same trigger, same gain, opposite sign, §9). **Always prefer code over algebra, and derive the sign from the code's own slip convention.**
2. **Code defaults vs shipped config can disagree in the dangerous direction** (TUM: `min_activation_velocity` 10.0 default vs 7.0 shipped; deactivation thresholds differ by ~2×; `max_brake_pressure` **declared twice in one file**, 60 then 80; look-ahead time 0.1 s code vs 0.45 s YAML; `min_lookahead_distance` 1.0 m vs 0.1 m). **Check which layer wins before quoting.**
3. **A source's own documentation can contradict its own code** (ForzaETH README says *"proven good values are `q_l1 = 0.2`"* while every YAML and the paper use **negative** `q_l1`; `deye1986`'s README documents `steerSmoothingFactor` and `maxSteeringAngleDampener` **parameters that do not exist in the shipped source**). **Prefer code.**

**A fourth trap specific to this domain:** a tutorial's field names may not describe what the code computes. The TORCS SCR folklore bot (§12) names its gains `steer_kp` / `steer_kd` while `steer_kd` multiplies a **position** (`trackPos`) and the actual damping is an unnamed `-0.05 * R['steer']` output low-pass. **Read the arithmetic, not the identifiers.**

---

## Summary: the one-paragraph version

Across TORCS, Speed Dreams' lineage, SuperTuxKart, Unity, Unreal, Game AI Pro, DonkeyCar, F1TENTH, ForzaETH, TUM, rFactor 2, Assetto Corsa, Automobilista 2, GT Sophy, Forza, Driver San Francisco and a dozen hobby projects, **the dominant steering architecture is: choose an aim point ahead on a racing line, drive the angle between the car's body-forward vector and the car→aim-point vector to zero, and schedule the aim distance with speed while holding the gain per car.** The gain is `2 × wheelbase / lookahead` when the aim point is placed by a look-ahead circle (pure pursuit) and `1 / steerLock` when it is placed by track geometry — the same controller, two unit conventions; SuperTuxKart ships `δ = 2·asin(L/r)` with `r = (x²+z²)/(2x)` and no P, I or D at all. **Look-ahead is scheduled with speed in every quantified source, with track width in one, with driver skill in one, and with grip in none**; the reason given, when one is given at all, is stability (`ω_n = √2·v/L_d` must stay below half the vehicle-plus-delay bandwidth), not comfort. **Slide handling is the field the industry does not have**: seventeen of the surveyed implementations do nothing, exactly one detects a slide and steers into it, three measure sideslip and slow down instead, and the rest avoid the problem by making the plant forgiving, deleting the slide kinematically, penalising it in a reward, or giving drifting its own state machine. **The two reference-vector camps are close to perfectly predictive of that split** — nose-referenced controllers need a separate slide term and mostly don't have one; velocity-referenced ones either are stability systems or fold the slide into the error. **Cross-track error as the *primary* steering signal appears in exactly one shipped default in the entire corpus (Speed Dreams' simplix), and even there its authority is capped at 15 % of the total steer angle** — everyone else pursues an aim point, which is where ARS's aim-error PID sits. ARS is in the velocity-referenced minority, and it matches the **original MAP paper** rather than MAP's shipped code — a contradiction inside the MAP authors' own two publications. **The two things ARS does that the survey could not find elsewhere are the grip-scaled preview (which ARS itself tested and backed away from) and the countersteer-exempt steering limiter (for which USR gets the same effect by widening the clamp under rear skid instead).**

