# ARS — Duel model, full deferred design (companion to AGENTS.md)

**Read this when** the work touches **rivals, overtaking, side-by-side racing, maneuvers/cards (DiveBomb, DefendLane, Yield, ChillOut) or nitro shots**. **Status: designed via Council, not implemented** — the live facts of the shipped cards are below, the design under them.

## Live per-racer state — rivals, overtaking, cards, maneuvers, nitro, Passengerize

- **Aggression** is grid-assigned by position (first lowest, last highest, the player mid) and scales only the avoidance buffer. The old note that it scaled TCS wheelspin is retired: `TcsCapLevel` reads no aggression. **Pressure** is proximity × aggression, rising slowly and falling quickly, and drives divebomb/defend/arming.
- **Maneuvers are cards** (`Maneuver` on the racer, no legacy arm blocks). A card **plays** into the slot and **folds** on its own condition, priority **ChillOut → DefendLane → DiveBomb → Yield**, with **Nitro slotless**. A played card is **never reconsidered mid-play** — unplaying causes thrash.
- **ChillOut** halves throttle and holds station behind the closest rival ahead until the field thins, arming only above a minimum speed so a slow car cannot become a roadblock.
- **DiveBomb** targets the closest reachable rival with a deeper braking target while its card lives. **DefendLane** covers the inside against a faster chaser; **Yield** lets a faster overlapping rival by near the entrance. The corner-commit lane (`Racer.cs:727`) engages inside the outside-approach window and ignores the one-way hold latch and the per-corner positioning decision while the card lives.
- **Nitro** has four situations — contested / defended / lonely / finish-spender — behind vetoes on braking, off-track, big steer angle and low gear on high-geared RWD cars. **AWD is exempt** via the handling struct's drive bias (full rationale: `AGENTS-TECHNOTES.md`). The player fires by key press (`TryFireNitrous`), the AI via `TryPlayNitrousCard`; both share one shot per lap, one charge cycle and one `SetOverrideNitrousLevel` path.
- **Passengerize** shifts AI drivers to the passenger seat while a rival overlaps, so they cannot swerve into contact, and is never applied to the player. **The ghosting removed in the avoidance cleanup is back as the No Collision option, untested (`0623276`)**: it clears rival detection outright and makes every racer pair pass through by calling the native in its permanent mode — the mode the removed ghosting got wrong.

## Duel model — the deferred design (not implemented)

Every card becomes physics-aware instead of reacting to instantaneous rival speed, on one shared primitive: **predicted time-to-apex per rival**, closed-form and cheap at the 1 Hz `ConsiderManeuvers` cadence.

**Inputs come from each car's own live plan**, never model-stat recomputation: rival `Racer` objects expose `NextApexSpeed` (which already bakes in downforce and the per-car learned corner-speed offsets), `CurrentMechanicalGrip`, `Handling.BrakingAbility`, `Handling.EstimatedTopSpeed`, `Handling.Acceleration`, `Gravity` and their own `ForwardNodeDistance` — all populated in `Initialize()` for every spawned racer.

**Decel is the existing `Racer.BrakingDecel(apexNode, spanMeters)` helper** that `ApexBrakingSpeed` and the `MaxSpeedForBrakingDistance` fallback already share — reuse it rather than re-extracting it. Model-hash natives (braking `0xDC53FD41B4ED944C`, …) are the **player** fallback only, since no AI plan exists for them, and against the player demand a wider margin — never dive the player on a coin-flip. Straights use the segment variant, per-car end speed over the distance to the next corner entrance; nitro effectively multiplies our accel (calibrate the approximation in game once).

Card upgrades this enables (all Council-endorsed):
- **DiveBomb gate**: arm only when the exchange completes — predicted arrival level-or-ahead at the braking target AND our apex speed carries the corner beside the target. Today the dive shortens the braking target with *our* decel and never validates against the rival's braking map, so "I'm faster right now" conflates a drag race with a braking duel.
- **DefendLane threat check**: defend only when the chaser's own grip/braking predicts they genuinely pass (beat or match us to this apex). A chaser who merely out-drag-races but under-brakes is beaten by the normal outside hold plus our own braking plan.
- **Yield corner-superiority**: arm only when the rival out-brakes OR out-corners us *for the specific upcoming corner*; a straight-line rocket that loses every corner gets no yield. Plus a race-progress-ahead gate — today Yield arms on any overlapping rival, even one behind on race progress.
- **Nitro segment advantage**: replace "rival faster right now" with "the burn converts a losing exchange into a pass completed *before* the entrance" (a pass completing inside the braking window is a dive in progress → veto). **Known bug to fix with it**: `rivalNearbyFaster` never filters `RelativePosition`, so a faster rival behind double-counts with the defended rule. Also gate "defended" on the chaser being straight-superior over the burn window.

Hazards / guard rails (Council consensus — respect all of these):
- **Self-consistency**: predictions must model the *post-commit* state on our side — a dive shortens its own braking target, Yield halves our decel — or dives get green-lit that overshoot and Yield is under-armed.
- **Line-radius correction**: the commit lane is about half a track width inside, so corner-phase comparisons need radius ± the lateral gap (decisive at hairpins, negligible at 200 m).
- **Slipstream is invisible** to the duel, so defender predictions systematically under-estimate chasers: inflate chaser closing estimates or add a cheap tow term (close behind + near-zero lateral → effective top-speed bonus), which also improves the nitro logic. Never resurrect the old 30 m artificial-traffic follow mistake.
- **Surface transients**: a rival's `CurrentMechanicalGrip` carries their one-tick `GroundGripMultiplier` (dirt/kerb) — mitigate with margins, or compare on `BaseMechanicalGrip` + `DownforceGripBonus`.
- **Latch verdicts** once per (apex, rival), like the existing arm-once pattern, so two AIs running the same duel cannot flip-flop at each other.
- **Mutual nitro**: the rival's `_nitrousLapUsed` is readable — include their possible burn in the prediction.
- **Margin floors**: declare a winner only beyond a tunable margin; ties take the conservative action. NaN-guard radii and speeds (the `Clamp(NaN)` → min-bound gotcha).
- **Downforce is speed-dependent** — accept it as uncertainty and demand margin rather than modelling it.

Sequencing (each step verified in-game before the next): 1) nitro segment-advantage + the `RelativePosition` fix; 2) shared decel helper + time-to-apex helper + DiveBomb gate + DefendLane threat check; 3) Yield corner-superiority + the self-consistency amendment; 4) later: a "pick the fight where we're superior" playbook, dive-target selection ordered by duel margin instead of raw distance, the tow term, yield exit on confirmed pass, and a duel line in the ShowInputs debug panel.
