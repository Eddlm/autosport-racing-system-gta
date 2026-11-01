# Autosport Racing System 0.9.0 — first WIP

The first public WIP of the reworked mod: the playable race loop, the bundled tracks, the flat car roster, the in-game
track creator, Auto Styling and the reworked AI.

## Required

- **ScriptHookV** (Alexander Blade) — not bundled.
- **ScriptHookVDotNet Enhanced (SHVDNE)**, not upstream's stable v3.6.0: the stable bridge cannot start on current
  game builds and every script dies. ARS is built against the v2 API, `ScriptHookVDotNet2.dll` **2.11.6**, which
  SHVDNE ships — copy the `.asi` and the `2.dll` from the same install, because they are a matched pair.
- .NET Framework 4.8 and the latest Visual C++ Redistributable.
- LemonUI.SHVDN2 ships inside the package (MIT).
- If ARS does not appear, `ARS.log` at the game root names the reason; a build the mod cannot drive on logs a forced
  error rather than crashing.

## In this release

- **Steering comes from the tyres, not from a table.** The front axle's measured side slip is read per wheel and
  compared against the peak that axle has at that speed; past it the command is capped, and the cap closes at a rate
  set by how far past the tyre is. The saturation and the rates come from the game's own traction curve, not from a
  tuned constant.
- **The pedal answers the same signal.** The throttle is trimmed by the same overshoot and closed across the band
  above the target, never below zero — braking stays the plan's decision.
- **Steering and slides are one control chain**: pure pursuit on the aim point, the yaw damper against the yaw the aim
  requires, a slide blend that hands the countersteer to the car inside the tyre's slip band, and a limiter whose
  ceiling reads the slip axis rather than a fixed angle.
- **Pedals as reasons**: traction control restated as shares of the peak slip; an overspeed correction that lifts
  throttle when the road pulls harder than the wheels explain; a stability cut that closes throttle per wheel out of
  contact and gives it back at its own rate; and ABS built as a wheel-rotation lock guard but shipped **off and
  greyed**, because GTA re-grants native ABS to every AI car and no script clamp can out-brake the engine.
- **Nitrous is symmetrical.** The player rides the game's native nitrous; rivals get a scripting multiplier on the
  same measured scale, which at the default settings is several times the player's bottle and reaches parity when the
  rival percentage is halved. The AI's burn is exempt from the packing factor and from the anti-overspeed cap, so a
  leader no longer brakes against its own bottle.
- **Rival walls are geometry, not prediction.** A wall exists while the bodies overlap on the route axis, picks its
  side from the car's lane, holds it while the cars are abreast, and moves at a fixed rate. The predictive branch, the
  angle gate and the latch are gone — they were the source of the oscillation.
- **Top speed reads the game's own drag-limited estimate** instead of a geared ceiling divided by an arbitrary
  factor, which inflated the speed reference the AI planned against.
- **The AI races the field, not a script**: overtaking pressure, divebomb, defend-lane, yield and chill-out
  manoeuvres, one bottle of nitrous per lap fired by the AI itself, and a stuck-recovery cycle that reverses, drives
  out and only teleports as a last resort.
- **Tracks**: the in-game creator is live — record a route, save it, and the derived table rebuilds as you draw. A
  corner is a region that opens to twice its radius within a short span of nodes, and apex speeds read the smoothed
  radius, so cars brake later.
- **Grid and board**: pace-ranked car selection for the grid, with PI and best-lap columns on the frozen leaderboard.
- **Auto Styling**: the cosmetic auto-tuner, with optional Menyoo appearance support.
- **Every four-wheeled class can race.** The pool no longer filters off-road, industrial, utility, commercial,
  service, emergency or military cars; only motorcycles, bicycles, boats, helicopters, planes and trains stay out. A
  grid can mix types it never mixed before — the pace band decides who races whom, not the body style.
- **Menu and settings**: one ini per menu group, a repair pass that fills every owned key and prunes retired ones, and
  the menu build wrapped so a failure logs instead of taking the mod down.

## Behaviour to know about

- The steering and pedal guard is **AI-only**: it lives inside the non-player branch, so the player's own steering is
  never written by ARS.
- Debug readouts describe the focus rival — the one nearest the player — never the player's car.
- A front-slip limiter cannot fix a four-wheel slide: when both axles are past their peak, the corner is simply
  faster than the grip and only speed helps.
- `Slip From Memory` switches the limiter between the measured wheel slip and a demand-versus-grip model. They are two
  different laws on one scale, by decision.

## Known limitations

- **Bikes are out of scope** — the loader drops every bike line.
- **No track editing or deleting** — a created track can be saved, not updated.
- No per-car XML saves, saved drivers, personalities or discipline filtering.
- The grid uses **model-level pace**, not the installed upgrades of a particular vehicle.
- The beater/rust matte paint rule ships unverified in game.
- The slip guard has no hysteresis and no compensation for its measurement lag, its deepest cut is a share of the
  request rather than of the tyre's own limit, and the speed-derived ceiling it once used is parked: compiled, inert.
- Low-grip versus high-grip behaviour across the fleet is a non-blocking check; report a car that turns in lazily or
  washes wide at the edge of the grip range.

## Upgrade notes

- Settings live in `Scripts\AutosportRacingSystem\Settings\`, one ini per menu. Old keys are migrated and pruned by
  the repair pass; a clean start is safe and lands on the shipped defaults.
- Tracks built before this release load as they are.
