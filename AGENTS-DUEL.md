# ARS — Duel model, full deferred design (companion to AGENTS.md)
> **Recorded idea, not built — the dive is the last option, not the first (2026-10-23, driver).** The next corners are
> already known (`NextApexNode2` / `NextApexNode3`), each car's top speed is known (`Handling.EstimatedTopSpeed`), and
> the straight between two corners is a `CumulativeDistance` difference. That is enough to ask whether the pass is
> *already bought*: ground gained on the next straight is roughly `(myTop - hisTop) x (straightLength / meanSpeed)`, and
> if that exceeds the gap to him, a dive adds nothing but the chance of going off. It also reads the other way — a small
> advantage or a short straight is exactly when the dive is worth it, because the road ahead will not give the place
> back. What makes this better than the current gates is that they decide from what is true **now** (who reaches this
> entrance first) while this decides from what happens **next**, which is how a driver reasons: not here, because I have
> him on the straight.
>
> **Recorded idea, not built — the corner is intel (2026-10-23).** Two cars fighting for one corner carry more than
> geometry: each already computes the speed it *wants* there. `ApexSpeedWithDownforce(DetectedRadius)` is what the car
> would carry through the apex, `BrakeFactorForApex` is how much it trusts that corner, `BrakingDecelBase` is what it
> can actually shed, and `NextApexSpeed` is the apex it currently holds. All of it is per racer and `r.RivalRacer`
> reaches the rival's copy, so the comparison is one access modifier away — `ApexSpeedWithDownforce` is private today.
> What it would buy: whether a dive is an investment or a lost half-lap on the outside; whether yielding is free when
> the rival is quicker through the corner and this car is quicker down the next straight; whether defending is worth
> the tyre when the rival's apex speed is far higher; and, later, the input that lets personality decide to fight or
> concede. The line it draws: today the cards decide from **geometry** (who reaches the entrance first); this would
> let them decide from **the likely result** (who wins the corner).
>
> **Defend watches its own threat, 2026-10-23 (driver).** The card armed on "he is closing and will arrive no later",
> and its release never looked at either: a chaser who lifted stayed covered until the apex, holding the inside for
> a fight that had ended. The release now also ends the card when the threat is gone — his own `TimeToReach` reads
> infinite (he stopped closing) or his time to the entrance exceeds this car's by `DefendThreatMarginSeconds` 0.5.
> The entrance node is stored at arm (`_defendEntranceNode`) because the release runs outside the block that computed
> it. Margin rather than a bare comparison: an early release costs the card its 1.5 s cooldown, so the wrong one is
> the expensive one.
>
> **A card is a decision, so its exits are looser than its entries (2026-10-23).** Three rules, all in `Racer`: every
> continuous release threshold is **wider** than the arm that admitted the card (`DivebombReleaseGapMeters` 40 against
> the 30 m arm, `DivebombReleaseAngleDegrees` 25 against 20, chill-out releasing at 50 m and one car fewer); no card may
> end before `CardMinDwellMs` 1000; and after ending, that card cannot re-arm for `CardCooldownMs` 1000 through
> `EndCard`. Yield is exempt from the cooldown because it is reactive and has nothing to protect. The point is that a
> card must read as intent: identical thresholds on both sides make a value hovering at the boundary flicker at the
> core rate, and every re-arm re-rolls whatever random the card carries.
>
> **Arm and release must bound the same thing (driven, 2026-10-23).** The time-window rewrite dropped the `+30 m` band from
> the arm while the release kept it, so every dive armed beyond thirty metres died on the next tick — the bonus was
> re-rolled each frame and the card read as flickering. Observed in the log as `armed ... gap 42.0` / `released partner
> lost after 16 ms` pairs, while every dive inside the band ran 2-3 s. `SameSectionMaxGapMeters` is back on the arm:
> any release test that is not also an arm test is a re-arm loop with a fresh random in it.
>
> **Divebomb gates, 2026-10-23:** the arm now asks *who arrives at the entrance first*, signed. `RouteGapAhead > 0`
> (he is ahead on the road), `rivalTimeToEntrance < myTimeToEntrance` (he gets there first), the difference no more
> than `DivebombReachSeconds` **0.75**, and `ForwardSpeedGap > 0` (this car is the one closing). The old absolute
> difference armed a car that was going to arrive first anyway, and the old `-CombinedSize.Y .. +30 m` band allowed a
> dive against a car alongside or behind. The distance band is gone: the time window bounds the gap by itself.
>
> **One card per situation, 2026-10-23:** a card may not arm against a target that is already playing *that same*
> card (`CardFree`) — two cars diving each other, or two cars yielding to each other, reads as a glitch rather than
> as racecraft. Applied at the three arming sites that pick a target: divebomb, yield, chill-out; defend already
> excluded both dive and defend targets before this rule existed.
>
> **Divebomb, 2026-10-23:** the card no longer moves the braking target. `BrakingTargetNode` and its 0.5 factor are gone; `ApexBrakingSpeed` uses `BrakeTargetBaseNode` for every card. The card now carries only its brake bonus (+2 % to +8 %, rolled per dive) and the commit lane to the inside of its target, which is the part that still reads on track. The kick card is switched off behind `KickCardEnabled`.


**Read this when** the work touches **rivals, overtaking, side-by-side racing, maneuvers/cards (DiveBomb, DefendLane, Yield, ChillOut) or nitro shots**. **Status: designed via Council, not implemented** — the live facts of the shipped cards are below, the design under them.

## Live per-racer state — rivals, overtaking, cards, maneuvers, nitro, Passengerize

- **Aggression** is grid-assigned by position (first lowest, last highest; the player is skipped and takes the last grid slot), and it scales the avoidance buffer — the higher it is, the less room the car gives — and the pressure target. The old note that it scaled TCS wheelspin is retired: `TcsCapLevel` reads no aggression. **Pressure** is proximity × aggression to the nearest car, rising slowly and falling quickly, and its only consumer is Yield.
- **Maneuvers are cards** (`Maneuver` on the racer, no legacy arm blocks). A card **plays** into the slot and **folds** on its own condition, priority **ChillOut → DefendLane → DiveBomb → Yield**, with **Nitro slotless**. A played card is **never reconsidered mid-play** — unplaying causes thrash.
- **ChillOut** halves throttle and holds station behind the closest rival ahead until the field thins, arming only above a minimum speed so a slow car cannot become a roadblock.
- **DiveBomb** targets the closest reachable rival with a deeper braking target while its card lives. **DefendLane** covers the inside against a faster chaser; **Yield** lets a faster overlapping rival by near the entrance. The corner-commit lane (`Racer.cs:727`) engages inside the outside-approach window and ignores the one-way hold latch and the per-corner positioning decision while the card lives.
- **Nitro** has four situations — contested / defended / lonely / finish-spender — behind vetoes on braking, off-track, big steer angle and low gear on high-geared RWD cars. **AWD is exempt** via the handling struct's drive bias (full rationale: `AGENTS-TECHNOTES.md`). The player fires by key press (`TryFireNitrous`), the AI via `TryPlayNitrousCard`; both share one shot per lap, one charge cycle and one `SetOverrideNitrousLevel` path.
- **Throttle kick** (limiter-saturation rotation, slotless like nitro, `cc469ee`, current contract `09295b3`): arms on the **ask the limiter was handed sitting past the ceiling it enforced** (stashed one core tick old in `ApplySteerLimits`), held 1 s of consecutive game-time calls, with no slide in the progress (the blend's own onset), RWD (the `0x48` bias ≤ 0.01) and a signed forward roll — **no speed band, no context gates, by decision** — plus the **gear gate**, computed from the leak's transmission: the burst's per-wheel thrust (`fInitialDriveForce` at handling `0x60`, the count verified between the `0x48` and `0x80` anchors) times the current gear's ratio (the leak's constants: first `1/0.3`, top `0.9`, the middles on the `1.1` progression) over two drive wheels, beating the live grip times the static wheel share by margin; the close-ratio and low-speed-torque flag multipliers are unread, so flagged cars read green on multiplied thrust and the fold's slip is their verdict. The window floors the pedal for 1.25 s — TCS pinned to 1 at the field, the slide blend and `IsFullCountersteer` standing down for the rotation sharing the arm-side sign (`KickWantsSlide`) — and folds at the **window cap or a body slide past 30°** (`rotated`); instability, the recovery override and the TCS's return cutting-fast answer the rest. `ThrottleReason.Kick` tags the floor; every play, fold and recovery entry logs forced; the burst's rotation is driver-proven (every recorded play broke the axle). Review: two glm-5.3-flash passes, findings folded; the drive judges the entry-band plays' outcomes (TEST-PLAN).
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
