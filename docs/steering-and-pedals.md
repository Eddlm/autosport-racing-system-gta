# Steering and pedal limits

How the front-slip guard is built today, what each number means, the evidence behind it, what was measured on track, and what is still open. Written after the 2026-10-25 session (builds 1050-1089) and rewritten after the 2026-10-26 session, which turned the actuator rates into the law described below (builds 1127-1140), and again after 2026-10-31 (build 1382), which split the single share into a target and an absolute tolerance without changing the shipped behaviour.

## The four pieces

| Name | What it is | Rate |
|---|---|---|
| `capShare` | what counts as "the peak" this tick, as a multiple of the tyre's own peak: `Peak Slip Target (0.90) x slow-corner allowance + Peak Slip Tolerance (°) / peak(v)`. `FrontPct` is normalised against it. | none — physics and settings |
| `factor` | how much of the request survives: `Clamp(100 / FrontPct, 1 - Deepest Cut %, 1)`. | none — recomputed every tick |
| `clampTarget` | the angle the wheel is allowed. Above the target (`FrontPct <= 100`) it is the steering lock, i.e. no limit; past it, `request x factor`. **`Open Up on Throttle` is out of the tree** (build 1389): the row, its key and `OpensUpClampScale()` are gone because the scale never reached the wheel — `open-up-inert` in the backlog keeps the formula. When it comes back it must scale the clamp and not `capShare`, because `FrontPct` also feeds the throttle cap and scaling it there closed the pedal instead of the wheel. | none — recomputed every tick |
| `chasingClamp` | the ceiling actually applied, `min(lock, chasingClamp)`. Born at `clampTarget` the first time the guard can speak, and the **only state** in the system. | closes exponentially, opens linearly (below) |

The steering factor:

```csharp
factor = Clamp(100 / FrontPct, 1 - DeepestCutPct / 100, 1)
```

- `FrontPct` is the **outer** front tyre's side slip, as a percentage of `capShare`: the loaded wheel, steered to 0.75 of the command by Ackermann, so the one that lands on its peak while the inner runs past by design. It is picked by magnitude (`min(|left|, |right|)`), so no index-to-side assumption is baked in.
- The cut stops at **`Deepest Cut (%)`** (`Rival Steering`, 0-100 in steps of 5, ships **60**): the share of the request the guard may remove at the bottom of a slide, so 60 still leaves 40 % of what was asked. There is no second clamp behind it. The row is stated as a *depth* rather than a floor so that its ladder runs with the guard's strength — 0 turns the cut off and 100 leaves nothing — instead of against it, which is what made the old numbers read backwards.
- The shipped 60 is `1/2.5`, the point where the traction curve stops falling, so the wheel bottoms out just before a deeper cut would buy no more grip (2.25x the physical peak at the shipping 0.90 target, because the floor is measured against the allowance). That is a stop the tyre chooses, not a preference: below it the cut only takes direction away, and it also bounds how far each pulse of the loop can swing when the slip reading trails the wheel by a tick or two.

The pedal, applied to `combinedInput` before it splits into throttle and brake:

```csharp
if (FrontPct > 100) combinedInput = Min(combinedInput, Max(0, 1 - (FrontPct - 100) / 150));
```

Only the throttle, never below zero, closed between 1x and 2.5x the target. Braking is left to the plan.

## The actuator: close and open

The two directions are deliberately different laws, and the asymmetry is the design:

```csharp
if (clampTarget >= chase)                     // OPEN: linear, careful
    chase = Min(clampTarget, chase + 72f * TickScale);
else {                                        // CLOSE: exponential in the gap, decisive
    gap   = chase - clampTarget;
    chase = Max(clampTarget, chase - Min(gap * 15f, 90f) * TickScale);
}
```

**Closing** is exponential in the *gap*, not in the severity of the slip:

- The rate is the distance to the target times **15/s**, so the gap halves every 30 Hz tick. Its own shape, no tuning per target size.
- **Ceiling 90 deg/s** (3 deg a tick). It binds above a 6 deg gap, which is where a collapse would otherwise read as a teleport (without it a 36 deg drop is covered at 500 deg/s).
- **No snap.** The exponential closes the gap and `Math.Max` stops the last step from crossing the target, so the tail is visible in the readout instead of jumping to 100 %.

Measured, 30 Hz, to 1 % of the target:

| target | 200 % | 400 % | 500 % | 1000 % |
|---|---|---|---|---|
| 2 deg | 233 ms | 300 ms | 300 ms | 433 ms |
| 5 deg | 233 ms | 333 ms | 400 ms | 667 ms |
| 10 deg | 267 ms | 467 ms | 600 ms | 1.13 s |

A full collapse, 38 deg to 2 deg: **633 ms in 19 ticks**, stepped and visible.

**Opening** is linear: **72 deg/s** (2.4 deg a tick). The full travel from 2 deg to the lock (36 deg) takes 0.5 s; any shorter gap proportionally less.

| jump | time |
|---|---|
| 2 deg -> 38 deg (lock) | 0.50 s |
| 2 deg -> 20 deg | 0.25 s |
| 10 deg -> 20 deg | 0.14 s |
| 18 deg -> 20 deg | 0.03 s |

Rules the calibration has to respect, learned the hard way:

- **The closing gain cannot approach the tick rate.** At 30 Hz the factor per tick is `1 - k/30`, so `k = 60` overshoots and is clamped back in ONE tick (a snap, not a law) and `k = 30` arrives exactly in one. The usable band is roughly `10-25`.
- **A relative rule and an absolute cap do not agree.** "500 % arrives in 0.1 s" is true for a 2 deg target and false for a 10 deg one, because 500 % of 10 deg is a 40 deg gap and 40 deg in 0.1 s is 400 deg/s. The measured table above is the honest statement; the percentage alone is not.
- **Exponential opening was tried and rejected**: its time varied with the gap, so the same manoeuvre looked different depending on how far the clamp had fallen. Linear makes the time proportional to the distance, which is what the car feels.

## Why the floor, the target and the tolerance are settings

- **`Peak Slip Target`** (0.05-2.00, ships **0.90**) is how much of the tyre's peak slip the guard lets it use before it cuts. 1.00 aims at the peak, below it the car never reaches its grip, above it the tyre slides, 2.00 barely cuts at all. Being a share it moves with the car and with the speed: the same value means the same thing on an 11 degree car and a 22 degree one.
- **`Peak Slip Tolerance (°)`** (0-6, steps of 0.5, ships **0**) is extra slip allowed on top, in **absolute degrees**: 0 cuts at the target, 2 lets the tyre run two degrees past it, 6 is past the 2.5x flat point, i.e. no guard at all. Being absolute it weighs more the smaller the peak is, so at speed it dominates the target — 4 degrees against a 2.7 degree peak is 148 % of it.
- **`Deepest Cut (%)`** (0-100 in steps of 5, ships **60**) is how much of the request the guard may remove at the very bottom: 60 leaves 40 %, the point where the tyre's force stops falling, so a deeper cut costs direction and buys no grip. 0 turns the cut off; 100 leaves nothing. Being a share of the *ask* it is the one row here that does not speak in the tyre's own units, which is why its label states the depth instead of a fraction of what is left.

All three live in `Rival Steering`, and the first two are **one** quantity:

```
capShare = Peak Slip Target x SlowCornerSlipShare + Peak Slip Tolerance / peak(v)
```

One writer, one denominator: `capShare` feeds the wheel cut, the throttle band and the readout from the same expression (`AllowedSlipShare`, `Racer.cs`). The tolerance is converted to a share by dividing it by the peak at that speed, which is why it is added *after* the slow-corner allowance rather than scaled by it. **At the shipped `tolerance = 0` the expression is algebraically the old `Peak Slip Share x allowance`**, so the row's arrival changed no behaviour; only moving it does. The row was named `Peak Slip Share` and before that `SteerCeilingFactor`, and the ini key is still `SteerCeilingFactor` — renaming the store would need a migration for a value that already exists in every install.

The speed-derived ceiling the target once scaled is the parked `SpeedSteerCeiling` branch (see below); it now reads the same `AllowedSlipDegrees` so that flipping it carries this pair rather than a second arithmetic.

## Slow corners

```csharp
slow  = Clamp((15 - mph) / (15 - 5), 0, 1);
share = 1 + slow;        // SlowCornerSlipAllowance = true
```

1x at 15 mph and above, 1.5x at 10, **2x at 5 and below**, for both the wheel and the pedal. Below 15 mph the readout's 100 % therefore means "the peak times the allowance".

The allowance is the only part of the law that moves with speed. It corrects an axis the tyre model assumes flat: at walking pace the tyre does not behave like it does at 60 mph, and holding the wheel at the high-speed peak there is what made the car refuse to turn in hairpins.

In the **theory** source the allowance is nearly inert: that source cannot be read below 6.7 mph (`FrontSlipTheoryMinSpeedMps = 3`) and only cuts when the demand at full lock exceeds the grip available, which happens at 12-15 mph. All of the allowance's real effect is in the memory source, which is the default.

## The two sources

`Slip From Memory` (Development submenu) picks where `FrontPct` comes from:

- **Memory (default)**: the wheel's own side-slip axis, offset `0x1C4`, normalised so 1.0 is the peak and 2.5 the plateau. The **outer** front wheel is the signal — `min(|left|, |right|)`, no index-to-side assumption — because Ackermann gives the inner the full command and the outer 0.75, and both peak at the same angle: the outer is the wheel that lands on its peak, the inner runs past it. With the wheel neutral the pair is averaged.
- **Theory**: lateral demand against available grip, `demand = v^2 / R` with `R` from the *inner* wheel's radius, `available = grip x g x capShare`, the tolerance riding inside the same share. Unreadable below 6.7 mph and when the geometry is degenerate.

Two different laws on the same scale, by decision. Normalising them means turning a lateral demand back into an equivalent slip angle, i.e. reintroducing a tyre model.

## Why the plateau is at 2.5x

The game's own tyre curve, `game/Vehicles/wheel.cpp:3170-3190`:

```
sfTractionPeakAngle = 1.0     // the coefficient peaks here
sfTractionMinAngle  = 2.5     // and reaches its floor here, flat from then on
```

So the lateral force rises to 1.0 of the slip angle, falls linearly to 2.5, and is constant past that:

- Past 2.5x, taking more wheel away buys **no grip** — the tyre is already on the flat.
- Between 1x and 2.5x the force is falling, so the useful signal is exactly that band.
- Cutting the wheel below the peak angle *reduces* force: the cut cannot fix an understeer on its own, which is why the pedal also has to respond.

Drive-side evidence for the pedal: `Automobile.cpp` splits drive force by `fDriveBiasFront`, and `wheel.cpp:4460` limits the drive coefficient against the traction available — the throttle eats the same grip budget. On a front-drive car the pedal therefore causes understeer; on a rear-drive one it also rotates.

## Measurements on track (2026-10-25, 31-32 mph)

Same car, same `factor` 0.40-0.42 (saturated), two corners with completely different behaviour:

| Corner | ratio | factor | mph | F | R |
|---|---|---|---|---|---|
| good | 1.82 - 2.50 | 0.40 - 0.42 | 31-32 | 237-257 | 197-204 |
| bad | 3.12 | 0.40 | 32 | 312 | 118 |

Reading: `R` at ~200 in both is ~0.78 of `F`, i.e. both axles are past their peak — a four-wheel slide, not a front-led understeer. The wheel clamp is then the wrong actuator: it removes 60 % of the angle while the force stays flat. The car is simply too fast for the corner, and the pedal is the only thing that can fix it.

## What the readout shows

One line, the same for both slip sources — `Debug > Show Front Slip`:

```
Slip vs Allowed
x1.9   47mph   cap 6.2°
```

- `x` = `FrontPct / 100`: how far the tyre is from what the guard allows. 1.0 sits exactly on the allowance, 1.9 is 90 % past it. Under the memory source that is measured side slip; under the theory source it is lateral demand, which is the slip the model predicts — the same question, and the same heading, either way.
- `mph` = forward speed, the context that gives a corner its name.
- `cap` = the ceiling **in force**, in command degrees: the bound the command was clamped against on the side asked (`_steerGrantedCeiling`), which is `min(gate, chasingClamp)` until a countersteer allowance or the damper branch lifts it to the slide angle or to the lock. It is the rate-limited value, not the law's target, so watching it walk is how the rate gets tuned.

When there is nothing to read, the same slot names the state instead of showing numbers: `inactive - stopped`, `no reading` (the source cannot answer — no wheel field, or the theory source under 3 m/s), `inactive - straight` (readable, below 0.5 % of the allowance). One token, one meaning, and the heading never moves.

What is **not** on the line any more: the factor, the asked/allowed ratio, the per-wheel front and rear percentages, and the allowance share. Each answered a question the first three numbers now answer together, and four of them were what made the line unreadable. `RearPct` is still computed in `SlipState` because the four-wheel-slide brake is the next thing to gate on it.

Only an AI racer draws it: `DrawFrontSlipReadout` returns early for the player, and `ApplySteerLimits` lives inside `if (!ControlledByPlayer)`. **The whole system is AI-only** — the player's steering is never written by ARS.

## Open items

1. **Brake when both axles are sliding.** The data says a four-wheel slide needs speed removed, not wheel angle. Proposal: if `R > 180` and `F > 200`, let the combined input go negative (to -1 as the excess grows). The earlier brake experiment was rejected as "slow" because it braked in *every* corner, including front-led understeer where it was unnecessary; gating it on `R` fixes that. `R` is normalised against `capShare` too, so the threshold moves with `Peak Slip Target` and the tolerance.
2. **Never cut below the peak angle.** `Deepest Cut` can be pushed past the angle that produces peak force on a car asking for little — 100 removes the whole request and the car goes straight on. The row now says so in its label, but nothing in the law stops it.
3. **Hysteresis.** The close now starts at any gap (exponential) and the open runs at 72 deg/s, so the old 1-2 Hz oscillation at 100 % should be gone — unverified in the car. A dead band (close above 105, open below 95) remains the fallback.
4. **The measurement lag.** The slip trails the command by 100-200 ms and nothing compensates. A first-order filter on `FrontPct` would remove most of it.
5. **`SpeedSteerCeiling` is zombie code** (`Racer.cs`, `static readonly bool = false`): `ResolveSteerCeiling`, `PeakSlipCeilingAt`, `GeometrySteerCeiling`, `ManeuverRamp` and the 5-30 mph ramp are compiled but inert, while the target/tolerance pair drives the live limiter. Decided for **0.11**: revive the law or delete it **and** its leftovers together.
6. **Left/right asymmetry** was reported and never confirmed. The wheel pick is assumption-free (`max` of both fronts), so if it persists the suspects are the countersteer/damper sign path and the AI's own steering.
7. **The guard disappears when the source cannot answer.** If `FrontPct <= 0.01` or the source is unreadable (theory below 6.7 mph), the block is skipped and the applied ceiling is the lock — i.e. no limit at all, which reads as an instant release. Keeping the last ceiling instead is a one-line change with real consequences in the car, so it waits for a decision.

## Failures worth remembering
- **A bearing measured in three dimensions (build 1163).** `PursuitSteerDegrees` took `Vector3.SignedAngle(heading, toAim, WorldUp)` with the raw
  3D vectors. The aim is 8 m ahead and adjacent route nodes differ by ~1 m in height, so the angle included the road's grade: 6.4 deg that
  flipped sign with every node step and read as a lateral tremor. The tell was in the numbers - the aim's lateral offset was 0.04 m while the
  bearing swung 6.5 deg - and it went unnoticed for a whole session of filters. Steering is a horizontal problem: flatten both vectors first.



- **Starting the chase at the lock.** A ceiling that begins at 38 deg and closes at 6 deg/s needs five seconds to reach a 7 deg target, so it never moved.
- **Seeding it to the request.** That made the ceiling follow the wheel, a ratchet dragged down by every small angle that could only climb back at the open rate.
- **Recomputing the cut from the request each tick.** The request is regenerated by the plan or the pad every tick, so "request minus 6 deg/s" never accumulates and the clamp looks like it does nothing. The continuity lives in the angle actually applied.
- **Letting the target track the request when there is no overshoot.** In the straights the target fell to a couple of degrees and the chase went with it; on corner entry it needed seconds to climb back, which is what held the wheel down while the factor read 0.75.
- **Gating the close on the severity of the slip (tanh, 1-10 deg/s).** The rate ignored the distance to the target, so a clamp at 1000 % of a small target took 1.35 s to travel 13.5 deg — the driver saw "the clamp never comes back". Replaced by the exponential in the gap.
- **A 45 deg/s cap calibrated by feel.** It made a 10 deg target take over a second on a serious slide, because the cap governs everything above a 3 deg gap. 90 deg/s bounds the worst case without shaping the normal one.
- **Filtering the steering input to calm a dither (build 1143, reverted in 1144).** An exponential slew capped at 180 deg/s reached at a 10 deg error did slow a 1-2 deg ask from one tick to four, and it made the car oscillate MORE: as a first-order filter with tau = 55.6 ms it adds 46 deg of phase lag at 3 Hz, inside the pursuit loop. A dither in the request is the loop's own signal; lagging the actuator does not damp it, it moves it. The fix, if it is ever wanted, is in the controller or a dead band on the ask, not in the slew.
- **Soft centre on the ask (build 1145, reverted in 1146).** Halving the pursuit's request at 4 deg and below, untouched from 8 deg, adds no phase lag and halves the amplitude of a small dither - and the dither stayed. Together with the input filter above, that rules out both the amplitude of the ask and the actuator as the cause: what is left is the term that flips sign with the rotation (the yaw damper), the pursuit gain near a zero bearing, and the aim point's own jitter. Parked in `AGENTS/AGENTS-BACKLOG.md`.
- **A snap at small gaps pretending to be the law.** Forcing "arrived" at a 0.05 deg gap hid the exponential's tail and made the last ticks read as a teleport; removed so the approach stays visible.
