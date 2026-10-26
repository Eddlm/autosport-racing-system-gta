# 0.9.0 — release notes (draft)

Draft for the `v0.9.0` tag. The workflow generates its own body from the full commit list (700+), which is
noise; this is the curated version to attach instead.

## What it is

Autosport Racing System: an AI race field for GTA V that drives the car through its own pedal and
steering laws instead of the game's. Rivals race the route you build, brake on their own, defend and
pass, and the player's car can be raced against them.

## Requirements

- ScriptHookV (Alexander Blade) — not bundled.
- .NET Framework 4.8 and the latest Visual C++ Redistributable.
- ScriptHookVDotNet **nightly**, not stable v3.6.0: the stable bridge cannot start on current game
  builds and every script dies. Verified against `v3.7.0-nightly.188`; copy the `.asi`, the `2.dll` and
  the `3.dll` from the same download.
- LemonUI.SHVDN2 1.5.2 ships inside the package (MIT).

## Highlights

**Steering comes from the tyres, not from a table.** The front axle's measured side slip is read per
wheel and compared against the peak that axle has at that speed; past it the steering command is
capped, and the cap closes at a rate set by how far past the tyre is. The saturation and the rates
come from the game's own curve (`sfTractionPeakAngle` 1.0, `sfTractionMinAngle` 2.5 in `wheel.cpp`),
not from a tuned constant.

**The pedal answers the same signal.** The throttle is trimmed by the same overshoot, closed across
the x1-x2.5 band of the target, never below zero — braking stays the plan's decision.

**Nitrous is finally symmetrical.** The player rides the game's native nitrous (its own ×4); rivals
get a scripting multiplier on the same measured scale: `NitrousPowerMultiplier = 5` against the player's
`PlayerNitrousPowerMultiplier = 1`, so the AI's bottle is five times the player's at the default `AiNitroPct = 100`
and they are at parity at 50. The AI's burn is
exempt from the packing factor and from the anti-overspeed cap, so a leader no longer brakes against
its own bottle.

**Rival walls are geometry, not prediction.** A wall exists while the bodies overlap on the route
axis, picks its side from the car's lane, holds it while the cars are abreast, and moves at a fixed
rate. The predictive branch, the angle gate and the latch are gone — they were the source of the
oscillation.

**Top speed reads the game's own drag-limited estimate.** `GET_VEHICLE_MODEL_ESTIMATED_MAX_SPEED`
replaces the geared ceiling divided by 0.75, which inflated the reference by 1.78x.

**Menu and settings.** One ini per menu group, a repair pass that fills every owned key, a root
subtitle of `MAIN MENU` with trails built from the tree, and the menu build wrapped so a failure
logs instead of taking the mod down.

## Behaviour to know about

- The steering and pedal guard is **AI-only**: it lives inside the non-player branch, so the player's
  own steering is never written by ARS.
- Debug readouts always describe `DebugFocusRacer` — the rival nearest the player — never the player's
  car.
- A front-slip limiter cannot fix a four-wheel slide: when both axles are past their peak, the corner
  is simply faster than the grip and only speed helps. See `docs/steering-and-pedals.md`.
- `Slip From Memory` (Development submenu) switches the limiter between the measured wheel slip and a
  demand-versus-grip model. They are two different laws on one scale, by decision.

## Known open items

Tracked in `docs/steering-and-pedals.md` (items 1-8) and `AGENTS-WIP.md`. The load-bearing ones:

- The 0.40 cut floor is an accident of the 2.5x saturation; the constant that states it says 0.05 and
  never binds.
- No hysteresis around the target, and no compensation for the 100-200 ms measurement lag.
- `SpeedSteerCeiling` is parked at `false`, leaving `ResolveSteerCeiling`, `ManeuverRamp` and
  `GeometrySteerCeiling` compiled but inert.
- Under 15 mph the slip allowance grows to twice the peak share by 5 mph, so a hairpin can still ask the wheel for more; above 15 mph it is flat.

## Upgrade notes

- Settings live in `Scripts\AutosportRacingSystem\Settings\`, one ini per menu. Old keys are migrated
  and pruned by the repair pass; a clean start is safe and lands on the shipped defaults.
- Tracks built before this release load as they are.
