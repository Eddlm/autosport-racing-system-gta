# Steering control — how this is actually done elsewhere

**Read this when** the request or the bug touches: steering controller, PID, P/I/D, pure pursuit, Stanley, aim point, lookahead, preview, gain, gain scheduling, feedforward, anti-windup, integral clamp, countersteer, opposite lock, oscillation, weaving, "why does it oscillate", "how do other games do it", "is a PID the right tool", steering limiter, steer authority, velocity-vector vs heading reference.

**Depth lives in `STEERING-CONTROLLER-SURVEY.md`** (36 implementations, 10 documented disagreements, every claim carrying its source URL, inaccessible sources listed rather than papered over). This file is the distillation; the survey is the evidence. ARS comparisons here are mine and go stale — **the code wins**.

## The dominant architecture is feedforward + feedback, not a lone PID

Every serious implementation surveyed splits the steering command into an open-loop **feedforward** term and a closed-loop **feedback** term — a two-degree-of-freedom architecture, explicitly named as such in the racing literature:

```
delta = delta_feedforward(curvature of the line ahead)  +  delta_feedback(PID on cross-track error)
```

- [Control-Theoretic PID Steering](https://www.emergentmind.com/topics/control-theoretic-steering-pid) states the law outright: `delta(t) = delta_pp(t) + delta_pid(t)`, feedforward geometric (pure-pursuit) term plus summed PID feedback.
- [Speedgoat, autonomous racing](https://www.speedgoat.com/Portals/0/adam/Content/0weSCmFk8U6yqmpJ_9cnug/Text/Sensor%20Fusion%20and%20Motion%20Control%20for%20Autonomous%20Racing%20Cars.pdf): "All controllers are based upon a two degrees of freedom architecture, separating the control request generation into a feedforward part and a feedback part".
- [Stanford, Kapania ch.2 "Feedforward-Feedback Steering Controller"](https://ddl.stanford.edu/sites/g/files/sbiybj25996/files/media/file/2016_thesis_kapania_0.pdf); [TUM's feedforward benchmark](https://arxiv.org/abs/2605.21111).
- The richest shipped example is **Ziggy Racer** (Forza Horizon 6, virtual gamepad, ~71 Hz): `steer = k_ff·κ·load_comp + k_head·α + PID(cross-track) + countersteer(sideslip)`.

**Why the feedforward exists:** a feedback-only law must carry a *standing error* to produce a steady command, so P alone settles a permanent few degrees of error — a permanent lane offset or corner cut. The literature's first fix is the feedforward; the integrator is the fallback.

## Pure pursuit, stated properly

Pure pursuit is **itself a proportional controller**: the [Sensors 2025 pure-pursuit paper](https://pmc.ncbi.nlm.nih.gov/articles/PMC11820862/) gives the gain as `2/l_d²` on lateral error (so `2L/l_d²` in steering-angle terms), equivalently `(2L/l_d)·α` on the aim angle. The [autonomous-driving-book](https://github.com/YangyangFu/autonomous-driving-book/blob/main/book/3-trajectory-tracking/lateral-control/pure-pursuit.md) notes it "can be tuned at different speeds by creating a relationship between the speed and the lookahead distance."

**ARS now computes this gain from geometry** (`atan(2 × wheelbase × sin(error) / aimDistance)`, trimmed by `SteerTrim`), which is the unified form — pure pursuit proper is the same law with the aim point placed by a look-ahead circle. `sin` rather than the raw angle is deliberate: the demand saturates as the aim point swings abeam instead of asking for full lock. **Knobs and units:** the menu's *Steer P* is `SteerTrim`, a **dimensionless trim on the geometry** (1.0 = the geometry itself), and the gain still schedules itself because the aim distance is `speed / grip × leadScale`; **`SteerI` runs on the angle error; `SteerD` is a lead time in seconds applied through that same geometry gain**, so the damping no longer changes with speed (see the D section below). Two naming traps to keep: the ini key is `SteerTrim`, deliberately *not* `SteerP`, because changing a raw gain into a trim meant a stale value had to be retired — the repair pass **snaps an in-list numeric key to the nearest offer** rather than resetting it, so a stored 0.25 would have become a permanent 0.2 trim (~80% too little steer) with no visible error. And **every declared default must be a member of its own domain list**, or the repair rewrites it on the next launch.

## What is universal is *preview* scheduling, not gain scheduling

**Correction to an earlier claim in this file.** "Gain scheduling by speed is universal" is too strong. Most implementations **never touch the gain** — TORCS `berniw`/`bt`/`damned`, USR default and Assetto Corsa hold a flat `1/steerLock` or per-car `STEER_GAIN` — and the *effective* gain moves only because the **aim distance** moves. Explicit gain scheduling is the minority, and it contradicts itself in direction:

- **More gain at low speed**: Game AI Pro ch.40 — "at low speed the vehicle may require **much larger** K values".
- **Less authority at high speed**: Unity Standard Assets (`maxSteerAngle` 28° → 23%), Unity's own tutorial (`steeringRange` 30° → 10°), NwliZz, and ForzaETH's `k_speed`. ForzaETH even ships **both** an upscale and a downscale in the same code, silently fighting each other.
- **A band-pass**: USR `SteerMod` reduces gain both above target speed and below it, with the reasons stated verbatim ("stop bouncing over curbs"; "avoiding or have just recovered from avoiding").

**So ARS's arrangement — flat trim × a gain derived from a speed-scaled preview — is the dominant pattern, not an oddity.**

**The one real derivation, and it is about the preview:** ForzaETH's stability criterion is that the guidance loop's natural frequency `ω_n = √2·v/L_d` (with `τ = L_d/v`) must stay below half the vehicle-plus-**delay** bandwidth. As `L_d` grows, `ω_n` falls — so **`L_d` must grow with speed** to hold the loop inside the bandwidth. That is the proper justification for a speed-affine preview: not comfort, but delay margin.

**Four sources name the hazard of scheduling at all** — Game AI Pro ch.40: "if the coupling is too great, a hidden positive feedback loop can be set up, resulting in instability". Ziggy Racer hit exactly that: feeding the *planner's own* curvature into the steering feedforward "created a planner ↔ tracker limit cycle … the two rang together into a bang-bang steering oscillation." **Never close the loop through a signal the loop itself produced.**

## The reference vector: body forward dominates; ARS's velocity reference is near-unique

| Reference | Count | Who |
|---|---|---|
| **Body forward / heading** | 13 | TORCS berniw/bt/tutorial, Unreal RacingAI, Unity CarAIControl + AIVehicleRoutingBuddy, Unreal TrafficAI, TUM example PP (body frame), ForzaETH MAP **shipped code**, ForzaETH PP, Game AI Pro ch.39, and others |
| **Velocity vector** | 3 | TUM countersteer (`atan2(vy, max(1,vx))`), TUM ESC (`beta = atan2(vy, vx)`), Ziggy Racer (sideslip catch) |
| **Track tangent** | 2 | TORCS bt `speedangle` (**throttle gate only**), TORCS SCR `angle` sensor |

**Load-bearing consequences:**

- **The velocity-referenced aim error is genuinely rare, and where it appears it is a stability layer, not a path-following error.** ARS uses it as the primary error. **Nobody in the corpus does that** — so this design has essentially no precedent to lean on, in either direction. Not a bug; an original choice whose behaviour ARS has to establish empirically. Note what it buys and what it does not: referencing **velocity** rather than the nose makes the error a pure *course* measure (which is why it needs no separate heading term), but it still says nothing about slip angle — ARS keeps that in the separate countersteer blend, and so does the field.
- **Do not cite ForzaETH/MAP code as precedent for it.** The MAP ICRA paper specifies the velocity vector; ForzaETH's stack paper and its shipped code use heading. The same group contradicts itself across its own publications.
- **The track tangent is never the steering error** in any source — it appears only on throttle gates. ARS's `speedangle`-style velocity-vs-tangent quantity is, in TORCS bt, a *throttle* input: the same primitive on a different channel.

## Look-ahead: speed-affine nearly everywhere, grip nowhere

16+ sources scale preview with speed; one with **track width** (SuperTuxKart's corridor search); one with **driver skill** (simplix). **Not one ties preview to grip or friction** — including stacks that have a full Pacejka model or per-wheel slip estimates available. ARS's `speed / grip × leadScale` is the only grip-scaled preview in the corpus.

**This is the one real open technical question about ARS's existing steering geometry.** The ForzaETH derivation is a genuine argument against grip: `ω_n = √2·v/L_d` is checked against a bandwidth that **does not contain grip**, so scaling `L_d` by grip moves `ω_n` without moving the constraint. But that is a *stability* criterion, and low grip genuinely does need an earlier turn-in for the same curvature — a *path-tracking* criterion, and the two are different things. No source resolves it.

**ARS has an in-game data point no source replicates**: the grip trend was already found to drive corner cutting, which is why the full `1/grip` was replaced by a compressed manual `grip→leadScale`. **The experiment that would settle it: A/B `speed/grip` against a speed-only preview, measured on cross-track RMS and steer-reversal count rather than by feel.** That is the highest-value experiment this research identifies.

## Treat the preview as a state, not a memoryless function

TORCS `bt` and `damned` both floor the preview against `oldlookahead − v·dt` — a **rate limit on the preview itself**, described as preventing "snap back" of lookahead on harsh braking. It is the only such guard in the corpus, and it matters because it makes the preview a state with dynamics.

**This is the field's answer to a hazard ARS has**: the lane systems are a **hard switch** (outside → inside, no lerp) and `targetLane` can step, which steps the aim point, which steps the aim error, which a D term turns into a one-frame steer spike. **Rate-limiting `targetLane` fixes that at the source** — better than smoothing the D term, because it removes the step for P as well.

## Anti-windup is mandatory when the actuator saturates — and ARS's steering saturates

Only **five** surveyed sources carry an integral term, and **four of them guard it**. ARS's is unguarded (unbounded, integrating unconditionally) while `ApplySteerLimits` caps the command to a small allowance at speed — so enabling I winds the integrator against a cap it cannot overcome.

Documented remedies, all cheap: **clamp** the integral (Ziggy: `cte_int` ±3; TUM: ±1325 on both PIDs) and **suppress integration while the actuator is saturated** (Ziggy's steer-clip anti-windup bleeds the integrator when the wheel is pinned at full lock and still winding). Game AI Pro ch.40 names both and adds a third: a **leaky rolling integral** — each frame reduce the integral by `(1−T)%` and add `T%` of the current error — which needs no saturation signal and self-heals. ARS already has `_steerLimitedThisFrame` for the saturation gate.

## Damping belongs on yaw rate, not on a D term over the path error

**Almost every implementation that needed damping put it on yaw rate or curvature, not on a D term**: TORCS berniw/bt (a yaw-rate error term, gain 0.1), Ziggy (a *filtered* yaw-rate comparison, `r_des = v·κ`), TUM PP and Eelis03 (low-pass curvature filters), Unity CarAIControl (a yaw-rate speed-caution term), NwliZz (a yaw-acceleration stabiliser).

Game AI Pro ch.40 warns about the alternative in the same breath as recommending it: "If the input data (R) is noisy, the derivative term can fluctuate in an undesirable manner… **this is effectively adding another integrator.**"

**Three consequences for ARS:**

- Our D differentiates the aim error, and that error is measured from the car's nose to the aim point (body forward since `6f4dc45`) with a raw per-tick divide, so it is noisy. The field's answer is not to smooth it but to **read yaw rate directly**. USR's cheapest trick is to fold the damping into the aim *bearing* rather than add a term: `targetAngle − (yaw + yaw_rate/15)`. ARS's D is mathematically a yaw-rate term *plus the aim point's own swing* (`ė = θ̇_aim − ω`, so the D term carries both `−D·v/L·δ` and `+D·θ̇_aim`), so writing it explicitly keeps the behaviour and drops the numerical differentiation and the aim-point half at once. **It does not drop the speed mis-scheduling**: the closed loop with a yaw-rate term `−Kw·ω` is damped by the identical `1/(1 + Kw·v/L)` factor, so a fixed gain still over-damps at speed and under-damps at walking pace — contamination/noise and scheduling are two separate defects needing two separate fixes, and the scaling is the second. **The whole speed story was one identity: `D/P = SteerD·v/L`** — the flat-gain form this replaced — at any error and any grip; equivalently, D's lead time over the loop's own response time, which put the crossover at `v = L/SteerD` (D stopped being a trim and became the dominant term above it, around 60 mph with the old gain). **ARS now applies D through the geometry gain** (`Kd = SteerD × SteerTrim × 2L/d`, `SteerD` in seconds), so the lead is constant in time and `D/P = 2·SteerD·grip/leadScale` — speed-invariant. Using `2L/d` rather than `1/speed` is deliberate: `aimDistance` already has a floor, so the gain self-limits at low speed instead of running away at rest, which matters for the stuck-recovery crawl. The driver confirmed the flat gain was too weak in slow corners *before* this fix, which is the evidence the identity was right.
- **ARS's disabled inner yaw loop was the field's answer, badly implemented.** The *shape* (a yaw-rate error term) is what TORCS berniw has shipped since 2002; what was wrong was the expected-yaw reference (derived from applied steer, and hence meaningless once the car slides), not the choice of signal.
- **The gate D needs is itself a symptom of the contaminated input.** Because `ė = θ̇_aim − ω` carries the aim point's swing, D could add lock when the *request* moved while the car had done nothing — the aim point is pinned to a 1 m node, so `θ̇_aim` steps at every crossing. Gating on the yaw rate leaked twice: steer and yaw disagree during a slide (exactly the countersteer case), which let the additive correction through, and `Math.Sign(0)` matches nothing, so a straight (yaw ≈ 0, or dithering across it) held the gate open. The gate now compares against `pSteer`, making the correction strictly unwind-only — honest, but it also means D supplies no lead at all and `I` is the only term that could trim a steady-state following error. Reading yaw rate directly (item 1) removes the need for the gate entirely.

## Course error and slip angle are different states — and which the loop sees depends on its reference vector

**This is the most error-prone point in the steering design, so always state it with the reference attached, never as a general property.** **ARS uses the body-forward reference; the velocity variant was A/B'd in game and dropped** — both are documented here because the difference between them is the durable insight. With `θ` the direction to the aim point, `ψ` the heading, `v` the velocity heading and `SlideAngle = ψ − v`:

| reference | error | on a slide |
|---|---|---|
| **velocity** (course-over-ground) | `e_vel = θ − v` | a **course** error only. For a pure body rotation with the course intact, `e_vel = 0` — the car reads as *perfectly tracked while it is sideways*, and only the blend countersteers. |
| **body forward** (textbook pure pursuit) | `e_nose = θ − ψ` | the slide **directly**, since `e_nose = e_vel − SlideAngle`. A slide *subtracts* from the error, so the law **countersteers on its own** — the right sign, but see the magnitude trap below. |

- A car pointed 20° off while tracking straight produces a course error, and **either** reference corrects it by steering. Ordinary.
- A car **sliding with its velocity vector still pointing at the aim point**: the velocity reference reads zero error and does nothing; the nose reference reads the full slip as error and counters it. That is *why* 13 of ~20 surveyed implementations are nose-referenced, and why nose-referenced controllers so often need no separate slide term at all.

**Neither reference *reads* slip — `e_nose` merely contains it.** The blend via `VehicleData.SlideAngle` remains the only explicit consumer of that state.

**ARS runs two layers, and how they relate depends entirely on which reference is selected:**

- The aim-error PID in `ComputeSteering` → the line, from whichever reference is active.
- The **slide countersteer blend** → **slip**, via `VehicleData.SlideAngle`, ramping in against TRlat and **overriding** the PID at full priority. It is the only consumer of slip in the entire steering path.

With the **velocity** reference the two layers are **complementary** — course vs slip, the field's shape, and the honest justification for the blend getting the last word.

### The magnitude trap: the sign is right, the gain is ~5× too small

**Do not conclude from the table above that a slip-containing error gives you countersteer worth having.** That error passes through the pure-pursuit gain `2L/aimDistance` ≈ **0.225° of steer per degree of error** at 30 m/s, so a genuine 10° slide yields `atan(2 × 2.7 × sin 10° / 24)` ≈ **2.2° of opposite lock** — right sign, invisible magnitude, well inside the slew and the limiter. The velocity reference has the same gain and the *wrong* sign (in a developed slide it commands *into* the corner).

**And in a corner the nose-referenced error does not even reach zero.** The aim point sits at the chord angle `c ≈ ld/(2R)` off the tangent (~6.9° at 30 m/s in a 100 m corner, more in tight ones) while the nose sits at the slip angle β, so `e_nose = c − β` stays **positive** until the slide exceeds the chord angle. Below that the nose reference merely **unwinds** steer as the body rotates — and that is self-reinforcing: less steer means less yaw, which means a *smaller* β, which keeps `|SlideAngle|` under the blend's ramp onset (~`0.3 × TRlat`) and leaves the blend contributing nothing.

**So the blend is not redundant with the nose reference — it is the same signal at roughly 5× the gain**, and it is why the reference choice barely affects slide recovery at all: at a 10° slide the PID offers ~2.2° and the blend ~6.2°, and at full ramp the blend *replaces* the PID outright. **The reference choice is a line-tracking decision, not a slide-handling one.** Making the PID a real slide participant would need the slip term entering with a gain near 1, not routed through `2L/ld`. Note also that **ARS has no understeer detector or sideslip model anywhere** — the blend's bare `|SlideAngle|` magnitude gate is the only thing in the steering path that looks at slip, so nothing "classifies" a car as understeering; it merely fails to trip the gate.

**The countersteer magnitude has a physical neutral value, and it is 1.** A steer angle equal to the slip angle points the front wheels along the **velocity vector**, which removes the front tyres' contribution to the rotation — that *neutralises* a slide. Any multiple above 1 points the wheels *past* the velocity vector and actively rotates the nose back, which is a deliberate over-correction rather than a stabilisation. ARS exposes it as the `SlideCountersteer` knob for exactly this reason — it is a taste dial around a derived value, not an arbitrary gain. Note also that nothing else limits this path: `ApplySteerLimits` exempts countersteer, so the knob is the only bound on it.

That is TUM's architecture — a path-tracking controller with a slip-gated countersteer layer on top (`δ += k·(α_f − α_r)`, gated on `|α_r| > |α_f|`). **The survey mis-characterised ARS as folding the slide into the path error; it does not.** The two terms are not competing for the same job, so their interaction is a *priority* choice (save the car first), not redundancy — which is the honest justification for the blend getting the last word.

**Two consequences to keep in mind:** while the blend holds full priority, **course tracking is suspended**; and a slide that never develops enough slip to trip the ramp is invisible to the blend, so only the PID's second-order response to it exists.

## Slides: the field mostly does nothing; ARS is ahead of it

**The survey's strongest negative result.** Of ~20 implementations, **17 do nothing about slides at all**; only **three** control them (TUM's countersteer — the only real published law, `δ += k·(α_f − α_r)` gated on `|α_r| > |α_f|` — plus Ziggy's yaw-rate + sideslip detectors and USR's `getAvoidSteer`); the rest avoid the problem by giving the AI extra grip (Assetto Corsa's 20% overhead "so the AI's weird and twitchy inputs don't make them spin", AMS2, rF2's steering-lock multiplier), deleting the slide kinematically (Unreal TrafficAI rewrites heading from wheelbase geometry), or penalising it in a reward (GT Sophy).

**Correction to an earlier claim in this file.** I recorded the oversteer-countersteer / understeer-slow-down split as an "independent confirmation" of ARS's design. It is one project (Ziggy), not a norm. What *is* common is the **understeer half**: three sources measure sideslip and respond by **slowing down rather than steering** — the ESC idiom with the countersteer half simply absent. So ARS's asymmetry (oversteer gets the wheel, understeer gets the speed pipeline) is a minority position that one strong source shares.

**ARS's countersteer-exempt steering limit has no exact counterpart anywhere** — but **USR reaches the same intent from the other direction, by making the clamp *wider* under rear skid.** Two independent codebases, two mechanisms, one intent: *the limit must not bind the correction that saves you.*

**ARS's live configuration is therefore the field's shape, not a deviation**: a course controller for the line, plus a slip-gated countersteer layer with priority over it, plus a countersteer-exempt limiter. The piece still missing relative to TUM is that TUM reads **per-wheel slip angles** (`α_f`, `α_r`) and gates on the *rear* exceeding the *front*, while ARS reads a single body-level `SlideAngle` — so ARS cannot distinguish front-axle from rear-axle slip, and its gate is on slip magnitude alone rather than on the front/rear imbalance that actually defines oversteer.

## Speed-dependent steering limits are normal — but check how hard they bite

TORCS's own **human** driver ships a "steer speed sensitivity… the factor of reduction of steering values when the speed grows. the default value is 1.0 which corresponds to **half the steering at 360 km/h**" — a mild, feel-oriented reduction on the input path, structurally the same idea as ARS's `ApplySteerLimits` but far gentler than ARS's allowance.

Ziggy's headline diagnosis is the cautionary tale: it burned a long campaign trying steering-law levers against a corner where the wheel was **already correctly saturated** — "you cannot fix a saturation by asking for more of a resource that is exhausted." The fix was reducing demand (a speed cut in the approach), not raising authority.

**Diagnostic for ARS:** if cars run wide in a corner *identically at every gain*, the limiter's allowance is the binding constraint, not the PID — measure commanded steer against the allowance before touching gains.

**On the slew rate — read the code for its value; the design question is whether it should exist at all.** Exactly one shipped racing game publishes a steering rate limit, and its stated reason is **comfort** ("to reduce shaking of karts", because "otherwise the karts counter-steer to fast") — while Eelis03 documents a **0.6 rad/s (34°/s)** actuator limit *causing* a limit cycle. So 34°/s is the number to measure any rate limit against, and a rate limit can be the oscillation *source* rather than a cure. ARS's rate should be justified on stability grounds, not comfort. Note also that ARS's **faster countersteer path was a guess, never validated in game** — do not treat it as a designed safety asymmetry when weighing it.

## Run the integral in distance, not in time

"The integrator AND the derivative gain need to be scaled by the speed. Normally the PID updates every millisecond but what you really want is to update the PID as a function of distance travelled or rate of speed." Integrating `error × dt` makes the integral speed-dependent for free; `error × ds` removes that. ARS already adopted time-sampled discipline for the Gs calculator; the steering PID has the same argument available in distance terms.

## Reading constants out of this literature

- **Code beats papers.** Three documented traps: TUM's paper and code disagree on the **sign** of the countersteer (porting the paper steers *into* the slide); TUM's code defaults and shipped YAML disagree in the dangerous direction (`max_brake_pressure` is declared **twice in one file**); and one repo's README contradicts its own YAML.
- **Cite the right publication.** MAP's paper and MAP's code disagree on the reference vector (above).
- **Cross-track error as the *primary* steering signal appears in exactly one shipped default in the whole corpus** (simplix), and there its authority is capped at **15%** of total steer. TORCS berniw caps its lateral-error correction at 0.1 for the same reason. **External support for ARS's split**: the lane systems decide *where* to aim, the pursuit law carries the authority, lateral error is a bounded trim.

## Where ARS sits, and the ranked list

ARS runs **pure pursuit on course-over-ground with the geometric gain computed in code**, a flat trim knob, a yaw-rate-shaped D term, and a saturating countersteer-exempt limit. That is the dominant architecture with two unusual choices (a velocity-referenced error; grip in the preview) and one missing piece (anti-windup). Ranked by expected payoff:

1. **Move the damping onto measured yaw rate**, scaled by speed, instead of differentiating the aim error. Fixes the noise fragility *and* the `D·v/L` speed mis-scheduling together. This supersedes the "damping ratio" framing — it is the same insight, with the field's standard implementation.
2. **Clamp and gate the integral** before I is ever turned up. Prefer clamp + saturation gate, or Game AI Pro ch.40's leaky rolling integral.
3. **Rate-limit `targetLane`** so a hard lane switch cannot step the aim point — the TORCS preview-rate guard, applied to the lateral target. Fixes the D-spike hazard for P too.
4. **A/B the grip-scaled preview** against a speed-only one, on cross-track RMS and steer-reversal count. The survey's highest-value open experiment.
5. **Add the curvature feedforward** (`wheelbase × curvature`) if a standing aim error shows up as a permanent lane offset.
6. **Sharpen the slide layer's input.** The blend is live again, so this is no longer "reinstate slide handling" but "give it a better signal": TUM gates on the **front/rear slip-angle imbalance** (`|α_r| > |α_f|` — the actual definition of oversteer) while ARS gates on the magnitude of a single body-level `SlideAngle` against TRlat, which cannot tell a rear-axle slide from a front-axle one. Per-wheel slip is available in the handling struct's wheel data if it is worth the plumbing.
7. Consider a distance-based integral, and a lookahead fallback (PX4: target the closest point on the path when the lateral error exceeds the lookahead) plus error-adaptive aggression (JPL/Kelly: let the gain grow with lateral error) as the honest replacement for the disabled off-track recovery term.
