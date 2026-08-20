# Steering-Limit Subsystem Audit

Scope: the empirical "peak-memory" steer limiter in `ApplySteerLimits` (`src\Racer.cs`), its
state fields, and its debug rendering in `DrawDebugPanel`. Audited against the code at the
time of writing; line numbers refer to `Racer.cs` unless noted.

## Verdict

The core idea is sound and the unit bug that plagued earlier revisions is now fixed, but the
system has **three Major correctness issues** (counter-steer starvation, an unconditional
throttle-cap latch, and an explore-mode that lets the gate be a no-op) plus a subtle
dimensionally-correct-but-semantically-wrong spike cap. It is a good *heuristic*, not a
rigorous controller, and it will misbehave precisely in the high-slip, low-grip conditions it
exists to handle.

---

## Findings

### MAJOR-1 — The sign gate can starve counter-steering `Racer.cs:701`

```csharp
if (Math.Sign(Control.SteerDegrees) == Math.Sign((int)VehicleData.YawRotationPerSecondDegrees))
```

The intent is "only limit steering *into* a corner; never limit counter-steer." But the test
uses the **commanded steer sign** against **yaw-rate sign**, and the two are not the same
thing when the car is sliding. In a drift/catch scenario the AI commands steer *against* the
yaw (counter-steer) and that sign condition flips false — which *is* the intended bypass,
so far correct. The real failure is the opposite case: **when the car is pointing the nose
into a corner but already yawing before the steer command is issued**, a large corrective
counter-steer command can share the yaw sign and be clamped by the memory limit, suppressing
exactly the recovery input. There is no negative allowance in `_bestSteerDeg` (it is
`Math.Abs` at line 737), so a counter-steer command whose magnitude happens to exceed the
limit gets clipped to `±speedBasedSteeringLimit` at line 765 while the car is genuinely
over-rotating.

**Fix:** gate on the *vehicle's* slip/yaw error ("is the car rotating beyond target?"),
not on the commanded-input sign — or, minimally, never clamp when `|SlideAngle|` exceeds a
recovery threshold. As written the limiter's claim "counter-steer never enters this gate"
is false: it depends on a fragile sign coincidence.

### MAJOR-2 — `MaxThrottle` clamp is a one-way ratchet with no recovery path `Racer.cs:766-767, 899`

```csharp
if (Math.Abs(Control.SteerDegrees) >= speedBasedSteeringLimit)
    Control.MaxThrottle = Math.Min(Control.MaxThrottle, FullSteerThrottleCap);   // = 0.9
```

`FullSteerThrottleCap = 0.9`. The only recovery is line 899 in `ConvertSpeedToPedals`:

```csharp
if (Control.MaxThrottle < 1.00f) Control.MaxThrottle += 2 * TickScale;
```

so `MaxThrottle` recovers at 2/s **only while below 1.0**, and this recovery line runs in the
*speed* phase. If a car spends a long corner continuously at the steer limit, `MaxThrottle`
is pinned to 0.9 the whole way. That is plausibly intended (it is a stability throttle cap),
but it is **not specific to the empirical system** and interacts badly with it: the empirical
limiter's exploit clamp deliberately *holds* steer at the limit (`>= limit` is true at the
clamp), so the 0.9 throttle cap is now effectively **always on during exploit corners**. This
binds the two subsystems together in a way neither comment documents.

**Fix:** decouple — either derive the throttle cap from a separate stability signal, or make
the exploit target land just *inside* the limit (`bestSteer + margin`, clamped to
`limit - epsilon`) so "at the limit" is a transient, not a steady state.

### MAJOR-3 — Explore mode makes the whole gate a no-op `Racer.cs:750-759`

```csharp
if (_empiricalExploit) speedBasedSteeringLimit = Math.Max(_bestSteerDeg + 2f, 8f);
else                   speedBasedSteeringLimit = VehicleData.SteeringLock;
```

In explore mode the limit is full steering lock, and `TranslateSteerToInput` (line 923) maps
`±SteeringLock → ∓1`, so **the clamp at line 765 can never bind** in explore mode. The
explore branch is therefore equivalent to "no limiter at all": the only thing the empirical
system contributes during exploration is *recording* peaks. That is defensible as "probe
philosophy," but it means every corner entry (where utilization is naturally low) runs
entirely un-limited, and the system's protective value is confined to the exploit window.
If the recorded peak is stale (see MINOR-2), explore mode can persist through an entire
low-grip corner with zero authority reduction — the exact wash-out failure this system was
built to prevent.

**Fix:** bound explore authority too (e.g. `min(SteeringLock, bestSteer * 1.5f)`), or at
least require the probe to be monotonic before granting full lock.

### MINOR-1 — Spike cap is dimensionally correct but semantically wrong `Racer.cs:720, 734`

```csharp
float gripCeiling = Math.Max(VehicleData.CurrentMechanicalGrip, 0.1f);
...
if (latG > _bestLatG && latG <= gripCeiling * 1.1f) { ... record ... }
```

This is now in consistent Gs units (good — the earlier `×Handling.Gravity` bug is gone), but
`CurrentMechanicalGrip` is the **declared** grip, which the project itself has established
reads ~2× what cars actually achieve. A measured lateral G that genuinely peaks at ~2.0 G with
a declared grip of ~3.9 G passes the cap happily, so the cap does **not** reject ordinary
over-achievement — it only rejects *collision-scale* impulses (>4.3 G). That is still a useful
spike guard, but it is misnamed and does not protect against a fast-but-legit-looking bump in
the 2–4 G band. A hard pavement transition at speed can produce a 3 G impulse that is not
sustainable grip and *will* enter the memory.

**Fix:** either (a) make the cap adaptive (reject values > k× the running median of recent
lateral G rather than > declared grip), or (b) accept that it is only a gross spike filter and
add a separate rate-of-change guard (`(latG - prevLatG)/dt` above a threshold ⇒ reject).

### MINOR-2 — `_bestSteerDeg` goes stale but is never decoupled from a decaying peak `Racer.cs:736-742`

`_bestSteerDeg` updates **only** on rise (line 737). When the peak decays (line 741) the
remembered steer does *not* decay with it. So the exploit limit `bestSteer + 2` can clamp
steer to an angle that achieved a *higher* (now-stale) G than the car can currently reach.
With `decayTau = 0.33s` the peak falls fast, but `_bestSteerDeg` stays pinned at its old value,
so for a window after grip loss the exploit limit is the *old corner*'s best steer applied to
the *new* (lower-grip) situation.

**Fix:** when the decay branch fires, also decay `_bestSteerDeg` toward a conservative default
(or toward `abs(SteerDegrees)` at the moment of decay), so the remembered steer tracks the
remembered G.

### MINOR-3 — The decay deadband's `0.98` is undocumented and interacts with `u` `Racer.cs:739, 744-747`

`else if (latG < _bestLatG * 0.98f)` means a 2% sag does **not** decay the peak. Then
`utilization = latG / Max(_bestLatG, 0.2f)` and the hysteresis at 0.6/0.9 is *measured against
the un-decayed peak*. In a slowly-degrading corner, `latG` can sit in the 0.6–0.98 band for
many frames: no decay, but utilization *just below* the 0.9 exploit latch, so the car stays in
explore (full lock) while the peak is quietly ratcheting. The deadband is small enough that
this is usually transient, but it is the one place the "fast correction" story breaks.

**Fix:** state the deadband rationale in a comment and confirm the 0.98/0.6/0.9 triple is
intended; consider making decay proportional to `(1 - u)` instead of a fixed deadband so the
very-slightly-below-peak case drains deterministically.

### NIT-1 — `_bestLatG` and `_bestSteerDeg` are never reset per-corner/session `Racer.cs:147-148`

Initial values `0` and `15`. On first frames `_bestLatG` rises from 0, so `utilization` starts
at `latG / 0.2` — an enormous initial value that forces the `>= 0.9` exploit latch on the
*first* frame with any lateral G, before any sensible peak is learned. (The 0.2 floor in the
denominator is a Gs floor, not a "no-measurement" sentinel.) The transient resolves within a
few frames thanks to the fast rise, but it is an avoidable spike at spawn/launch.

**Fix:** initialize `_bestLatG` to a plausible floor (e.g. `0.4f`) or gate the exploit latch
until a minimum number of samples have been recorded.

### NIT-2 — `AverageAcceleration` mixing in longitudinal G weakens the lateral measure `DataStructures.cs:24-32`

`AverageAcceleration` is the plain mean of the full 3D acceleration vector (surfaced over the
last ≤10 samples). Dotting it with `Car.RightVector` projects out the longitudinal component
mathematically, but the *average* is dominated by throttle/brake transients, so high-frequency
longitudinal noise inflates the vector's norm and, under a tilted chassis, leaks a small
longitudinal contribution into the lateral dot via the world-right projection. A dedicated
lateral-channel accumulator (already partially present as `GetLateralGs` at
`DataStructures.cs:42-49`, which the limiter does **not** use) would be cleaner and is already
written.

**Fix:** have the limiter call `VehicleData.GetLateralGs(...)` (or a motion-axes version) so
the measurement is laterally-committed by construction rather than by projection of a mixed
mean.

### NIT-3 — Yaw sign is an `(int)` cast, losing precision at low yaw `Racer.cs:701`

`Math.Sign((int)VehicleData.YawRotationPerSecondDegrees)` truncates yaw to an integer; any
`|yaw| < 1°/s` casts to `0`, whose `Sign` is `0`. A command with `Sign(steer) == +1/-1` then
never equals `0`, so **at near-zero yaw the gate is always false and the limiter never runs**
(never clamps, but also never records). Near-zero yaw is exactly low-speed corner entry, where
the memory should be seeded. Practically it means the system is dormant at low speed.

**Fix:** compare against a deadzone on the float (`yaw > +eps` / `< -eps`), not an int cast,
and decide explicitly what sign-0 should do.

---

## Units contract (should-be-true invariants)

1. **Lateral G is always real G = `m/s² ÷ 9.8`.** `AverageAcceleration` is m/s²; any consumer
   must divide by 9.8 once. Never multiply a grip-from-native value by `Handling.Gravity` and
   then treat the result as G — that was the recurring 9.8× bug.
2. `_bestLatG` is **Gs**, always, everywhere (memory, ceiling, utilization denominator,
   display).
3. `_bestSteerDeg` is **degrees (absolute)**; it must grow/shrink in lock-step with the peak it
   belongs to.
4. `speedBasedSteeringLimit` is **degrees (absolute, ±)**; it feeds `ARS.Clamp(steer, -limit,
   +limit)` and must stay positive.
5. The 0.2 floors are **Gs floors**, not "no data" sentinels; a zero-measurement state must be
   handled separately from a genuine 0.2 G.
6. `YawRotationPerSecondDegrees` is a **world Z** rotation rate in °/s (from
   `GET_ENTITY_ROTATION_VELOCITY .Z`, line 2753); positive = CCW. Any sign comparison against
   steer must account for the local↔world heading transform, or it is meaningless.

---

## Open questions for the game author

1. What is the sign convention of `Control.SteerDegrees` vs `YawRotationPerSecondDegrees`
   (does positive steer = left = CCW-world-yaw)? The MAJOR-1 gate cannot be validated without
   confirming this in-game (e.g. steer right while watching yaw sign).
2. Is the 0.9 `MaxThrottle` cap *intended* to be active for the entire exploit window, or only
   as a transient anti-spin nudge? The two subsystems' coupling (MAJOR-2) is currently
   undocumented.
3. Is "full-lock authority while exploring" an accepted design risk, or should explore also be
   bounded (MAJOR-3)? The author has stated reprobing from scratch is acceptable, but the
   un-limited explore phase means a full lap's worth of corners run without any authority cap
   whenever below 0.6 utilization.
4. Is `CurrentMechanicalGrip`'s ~2× over-reading (established elsewhere in the project) expected
   to stay, or will declared grip be re-scaled? It directly affects the usefulness of the
   spike cap (MINOR-1).
