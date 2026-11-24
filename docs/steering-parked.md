# Steering and pedal limits: what is parked or revised

`steering-and-pedals.md` states the laws that run today. This file holds what does not: the switches parked or
removed, the pieces compiled but inert, and the revisions that built that file. One question: *what here does not
run, and what would bring it back?*

## Parked switches

**`WheelTrimUnderPower` (Target Under Power).** Blended the wheel's allowance down to `TargetUnderPowerPct` % of
the peak past half pedal, gated by the projection's room. Parked at `false` with its menu row removed on
2026-10-31. Restore: one flag and one row. The projection-room geometry it measured now drives the throttle floor
instead.

**`SlipPedalCut` (the slip pedal band).** Parked at `false` on 2026-10-31 (build 1394, driver: "envenena el
test"). With it on, the throttle closed from 100 % to 250 % of the allowance:

```csharp
if (FrontPct > 100) combinedInput = Min(combinedInput, Max(0, 1 - (FrontPct - 100) / 150));
```

It went out because it was a third thing moving the pedal in one test: with the trim handing back angle under
power, a band on the same measurement left no way to attribute what the seat felt. Its 1×–2.5× span is the
engine's flat point, `sfTractionMinAngle`, so the flag has no setting of its own. What would bring it back: a
four-wheel slide — both axles past the allowance — where removing angle buys nothing and only speed does (that is
open item 1 in the law file; before it ships, its gate needs re-measurement — the law file's measurement section
tells why).

**`SpeedSteerCeiling` and its helpers.** Compiled but inert: `ResolveSteerCeiling`, `PeakSlipCeilingAt`,
`GeometrySteerCeiling`, `SteerLimitRamp` and the ramp `SteerLimitRampStartMph`→`SteerLimitRampEndMph` return the
full lock before any of their law is reached (`Racer.cs:1601`, re-verified 2026-11-14). Decided for 0.11: revive
the law or delete it and its leftovers together (open item 5 in the law file).

## The lane throttle floor's revisions

- 2026-10-25 → 31: parked at build 1389 ("para que no envenenen el test"), revived on 2026-10-31 at build 1396
  with `GsLaneOutsideFloor` in force, given its checkbox row in `Rival Pace` at build 1398 (ships on).
- 2026-11-14: the five-metre absolute fall became a lane-to-inside-edge lerp (build 1854); the lateral-Gs gate
  became the slide band — 0.5–1.5 × the traction curve's peak at speed — the same day (build 1856); that lerp
  became the ±2 m window over the car lane, with the projection-or-car off-road closers and the widening
  qualifier dropped (build 1857). The law file carries only the final shape; this is the chain.

## How the law file was rewritten

Written after the 2026-10-25 session (builds 1050-1089). Rewritten after 2026-10-26 (builds 1127-1140), which
turned the actuator rates into the law. Rewritten again after 2026-10-31 (build 1382), which split the single
share into a target and an absolute tolerance without changing the shipped behaviour. On 2026-11-14 the floor's
mechanic changed twice in one day and its dated parentheticals moved here, so the law stays in the present tense.
