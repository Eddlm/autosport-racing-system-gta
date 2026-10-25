# Steering and pedal limits

How the front-slip guard is built today, what each number means, the evidence behind it, what was measured on track, and what is still open. Written after the 2026-10-25 session (builds 1050-1089).

## The three pieces

| Name | What it is | Rate |
|---|---|---|
| `maxSlip` | the peak slip at this speed, `LateralPeakAtSpeed(speed) x FrontSlipCapPeakShare`, times the slow-corner allowance. The scale of everything else. | none — physics |
| `clampTarget` | the angle the wheel is allowed this tick. Above the target (`InnerPct <= 100`) it is the steering lock, i.e. no limit at all; past it, `|request| x factor`. | none — recomputed every tick |
| `chasingClamp` | the ceiling actually applied, `min(speedCeiling, chasingClamp)`. Born at `clampTarget` the first time the guard can speak, and the only state in the system. | shuts at 1-10 deg/s, opens at 4 deg/s |

The steering factor:

```csharp
factor = Clamp(100 / Min(InnerPct, 250), FrontSlipFactorFloor, 1)
```

- `InnerPct` is the worse front tyre's slip as a percentage of `maxSlip` (the readout's `F`).
- The cut saturates at **2.5x** because that is where the tyre's own curve stops falling (see below).
- `FrontSlipFactorFloor = 0.05` is currently **inert**: with the saturation at 250 the factor cannot fall below 0.40.

The pedal, applied to `combinedInput` before it splits into throttle and brake:

```csharp
if (InnerPct > 100) combinedInput = Min(combinedInput, Max(0, 1 - (InnerPct - 100) / 150));
```

Only the throttle, never below zero, closed between x1 and x2.5 of the target. Braking is left to the plan.

## Why x2.5 and not some round number

The game's own tyre curve, `game/Vehicles/wheel.cpp:3170-3190`:

```
sfTractionPeakAngle = 1.0     // the coefficient peaks here
sfTractionMinAngle  = 2.5     // and reaches its floor here, flat from then on
```

So the lateral force rises to 1.0 of the slip angle, falls linearly to 2.5, and is constant past that. Consequences that drove the design:

- Past 2.5x, taking more wheel away buys **no grip** — the tyre is already on the flat. That is why the cut saturates there.
- Between 1x and 2.5x the force is falling, so the useful signal is exactly that band.
- Cutting the wheel below the peak angle *reduces* force: the cut cannot fix an understeer on its own, which is why the pedal also has to respond.

Drive-side evidence for the pedal: `Automobile.cpp` splits drive force by `fDriveBiasFront`, and `wheel.cpp:4460` limits the drive coefficient against the traction available — the throttle eats the same grip budget. On a front-drive car the pedal therefore causes understeer; on a rear-drive one it also rotates.

## Slow corners

```csharp
slow = Clamp((SlowCornerSlipFullMph - mph) / (SlowCornerSlipFullMph - SlowCornerSlipDoubleMph), 0, 1);
maxSlip *= 1 + slow;     // SlowCornerSlipFullMph = 30, SlowCornerSlipDoubleMph = 10
```

The allowance is 1x at 30 mph and 2x at 10 mph, for both the wheel and the pedal. Below 30 mph the readout's 100% therefore means "the peak times the allowance", not "at the peak".

## Measurements on track (2026-10-25)

Two corners at 31-32 mph, same car, same `factor` 0.40-0.42 (saturated), completely different behaviour:

| Corner | ratio | factor | mph | F | R |
|---|---|---|---|---|---|
| good | 1.82 - 2.50 | 0.40 - 0.42 | 31-32 | 237-257 | 197-204 |
| bad | 3.12 | 0.40 | 32 | 312 | 118 |

Reading: `R` at ~200 in both is ~0.78 of `F`, i.e. both axles are past their peak — a four-wheel slide, not a front-led understeer. The wheel clamp is then the wrong actuator: it removes 60% of the angle while the force stays flat. The car is simply too fast for the corner, and the pedal is the only thing that can fix it.

## What the readout shows

```
ratio   factor   mph   F n   R n
```

- `ratio` = `|request| / chasingClamp`: above 1 the guard is holding the wheel back. Spikes when the request rises quickly, because the chase opens at 4 deg/s.
- `factor` = the share of the request the cut leaves; 1.00 means no cut.
- `F` / `R` = worst front and worst rear tyre, as a percentage of their target.

Only an AI racer draws it: `DrawFrontSlipReadout` returns early for the player, and `ApplySteerLimits` lives inside `if (!ControlledByPlayer)`. **The whole system is AI-only** — the player's steering is never written by ARS.

## Open items

1. **Brake when both axles are sliding.** The data says a four-wheel slide needs speed removed, not wheel angle. Proposal: if `R > 180` and `F > 200`, let the combined input go negative (to -1 as the excess grows). The earlier brake experiment was rejected as "slow" because it braked in *every* corner, including front-led understeer where it was unnecessary; gating it on `R` fixes that.
2. **Never cut below the peak angle.** With the saturation the factor bottoms at 0.40, but the ceiling is still `request x 0.40`, which can be under the angle that produces peak force on a car asking for little.
3. **Hysteresis.** Closing starts at 1 deg/s exactly at the target and opening runs at 4 deg/s; sitting at 100% can oscillate a few degrees at 1-2 Hz. A dead band (close above 105, open below 95) would settle it.
4. **The measurement lag.** The slip trails the command by 100-200 ms and nothing compensates: at 10 deg/s the gate over-closes by ~1.5 deg. A first-order filter on `InnerPct` would remove most of it.
5. **`FrontSlipFactorFloor` is inert** — decide whether the deep cut (0.05) matters or delete the constant.
6. **Two slip sources.** `Slip From Memory` (Development submenu) reads the wheel memory; off derives the demand from the inner wheel's radius against the grip. They are two different laws on the same scale (at 60 mph the ceilings are 7.97 and 2.81 deg), by decision. Normalising them means turning a lateral demand back into an equivalent slip angle, i.e. reintroducing a tyre model.
7. **`SpeedSteerCeiling`** is parked at `false`: `ResolveSteerCeiling`, `ManeuverRamp` and `GeometrySteerCeiling` are compiled but inert. Decide whether to delete them or revive the law.
8. **Left/right asymmetry** was reported and never confirmed. The wheel pick is now assumption-free (`max` of both fronts), so if it persists the suspects are the countersteer/damper sign path and the AI's own steering.

## Failures worth remembering

- **Starting the chase at the lock.** A ceiling that begins at 38 deg and closes at 6 deg/s needs five seconds to reach a 7 deg target, so it never moved.
- **Seeding it to the request.** That made the ceiling follow the wheel, a ratchet dragged down by every small angle that could only climb back at the open rate.
- **Recomputing the cut from the request each tick.** The request is regenerated by the plan or the pad every tick, so "request minus 6 deg/s" never accumulates and the clamp looks like it does nothing. The continuity lives in the angle actually applied.
- **Letting the target track the request when there is no overshoot.** In the straights the target fell to a couple of degrees and the chase went with it; on corner entry it needed seconds to climb back, which is what held the wheel down while the factor read 0.75.
