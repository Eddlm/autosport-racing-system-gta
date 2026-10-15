# ARS — Deferred backlog (companion to AGENTS.md)

**Read this when**: the work touches anything deferred / "later" / "TODO" / "open question", you need to know whether something was already decided, tried or fixed, or the user asks to add, resume or rank a feature.

> Conventions. Every item carries a stable kebab-case slug beside its rank number — the number is rank and may move, the slug never does; other files cite the slug only and write nothing else about the item. Tiers rank simplest → most complex; 0-2 need no wheel time, 3+ need the user driving or a design call first. **Docs drift — the code wins**: check a claim against the source, and correct the line you are touching rather than reconciling the file wholesale. Renumbering the ladder must remap its own `item N` references in the same pass — park every old reference on a placeholder first, because a direct replace cascades when one entry's target string is another's source.

## The ladder — open items, simplest → most complex

**Tier 0 — decisions, no code**
- `lane-repulsion-ceiling` — the contact push stays unbounded. It is not a lane law: the request is settled above it, and the lane-side contact term is `ApplyRivalWalls`. **DECIDED (driver)**
- `player-special-ability` — ignore the engine's player grip and steer advantage. **DECIDED (driver)**
- `ai-free-abs` — ABS stays the car's flags plus ARS's own cap. **DECIDED (driver)**
- `crest-gate` — entry move built and driver-verified (`27ee097`). The corner must account for the crest so cars brake earlier than an airborne arrival allows; the gate tests whether the crest falls inside the braking span and clears a severity floor — err generous. The two constants wait on the instrumented run.
- `lane-guidance-gate` — REMOVED: the lane steer is ungated and the aim is the carried lane, so a straight-line standing offset is held. **driver-verified**
- `settings-key-symmetry` — shipped (`fb2b684`, `4852d7d`); the key name still has four homes.
- `settings-doc-anchor-drift` — anchors DONE; the seven claim-level re-derivations remain.

**Tier 1 — localised edits, low risk**
- `council-minors` — save-per-scroll dirty flag (**verified open**: no dirty or scroll-guard symbol exists); per-tick store reads; join chevron; `GridSize` (**verified done**: the item stores the value and snaps it on read, `AutosportRacingSystem.cs:731-739`); pace fallback.
- `pace-index-cache` — restored from the compute-once audit, which still calls it its highest-value item and which the backlog compression had dropped: `Racer.Initialize` probes the three model natives live (`Racer.cs:531-543`) while the selection path is cache-first (`TryComputePlayerCarPaceIndex`, `AutosportRacingSystem.cs:3375`), so the player and the AI can reach the same PI by different routes, and only the live path has no non-finite filter. **partially settled (verified)**: the audit's quoted "Cache first…" comment no longer exists and the code now states the live probe as deliberate, so the open question is whether to accept that or make the cache first-class.
- `weaponized-grid-filter` — weaponized cars must not reach the grid; mechanism settled, **not built**, wanted before 0.9.
- `update-checker` — extract the update checker into a separate DLL. **not built** (verified: `src\UpdateChecker.cs` compiles into the single library, `NewRacingSystem.csproj:93`).
- `slide-limit-raise-removal` — deleted (`ebb3178`); the countersteer allowance and damper bypass stay.

**Tier 2 — verify in game, then tune**
- `apex-radius` — driver-observed on high-grip cars (`aa302e1`); the fleet-wide read is owed.
- `constant-radius-generator` — a creator-built corner can collapse the span; watch created tracks specifically, with the stability tolerance as the lever.
- `steer-ramp-band` — the limiter's ramp band is narrower; the ramp's end anchor scales with Steer Limit At Speed so the band stays continuous. **driver-verified (`489`, `e59e0a9`)**
- `side-slip-scale` — the side-slip field the wheel-slip spheres read was never probed the way the longitudinal one was: the leak writes it on a scale whose peak is 2.5 while the sphere colours call 1.0 the peak, and the layout lesson of `slip-axis-scale` applies to this block too. The live steering ceiling reads the model, not this field, so nothing in the ceiling moves either way. **unsettled**

**Tier 3 — single-method changes**
- `electric-pace` — slow-electrics class question, **DECIDED**: class membership must not influence an electric's treatment (`3d589d6`, user decision), and the ramp's peak stays the scored value, measured and deliberately left (`f3fc274`, `e6c0370`).

**Tier 4 — coupled systems**
- `corner-approach-braking` — start the outside-line move about a second before the car must brake for the apex, computed from its own speed profile rather than a fixed seconds-to-apex gate.
- `rival-publish-core` — the rival publish moves onto the core pass over a per-frame pose snapshot, so a card stops deciding on a record up to half a second old. **not built**: verified, `UpdateRivalInfo` still runs on its own 500 ms beat (`Racer.cs:3695-3699`) while the cards run on the core pass below it (`:3700`), and the second call (`:4183`) is the re-own path, not a cadence change.

**Tier 5 — new controllers and design decisions**
- `snap-oversteer` — a D-term that does not exist yet; dedicated session. **not built** (verified: no derivative term anywhere in the steering chain).
- `gravity-grip` — gravity carries the offroad multiplier and grip stays the coefficient, both published. **BUILT (compile-verified, dev build 613), drive owed**: the double-count is gone — the grip bake in `UpdatePerceivedGrip` is deleted, so grip is the coefficient again, and `AckermannCeilingDegrees` and `SteerLimitedSpeed` read `Handling.Gravity` like every other grip law. The rule stands: a quantity about actual grip takes the car's own gravity; a theoretical or reporting one (`ComputePaceIndex` `AutosportRacingSystem.cs:97`, the curvature-to-G conversion `:2670`) keeps the assumed 9.8.
- `pace-model-theoretical` — pre-race selection is spawn-free by design, the player's car alone the live gap. Preserve the invariant that the leaderboard number equals the selection metric.
- `overrotation-pedal` — a rotation-driven throttle authority; the pedal acts on rear grip, and the driver-observed case that puts it on the map is full throttle while the car slides. **not built** (verified: no such symbol in `src`).
- `yaw-damper` — reference and speed/grip schedule shipped (`d5a643f`); the crossing past neutral while the rear leads remains, with the floor-plus-ceiling shape as the fix.

**Tier 6 — subsystem reworks**
- `two-projection` — two-projection route speed replacing the geometric route speed. **not built** (verified: no such symbol in `src`).

**Tier 7 — shipped, wants a second look**
- `downhill-braking-term` — span consistency fixed and verified in game. The per-node decel profile is **not** a defect — the mean is the exact quantity — so do not "fix" it. Open: the brake-learning/grade coupling.

**Tier 8 — removed for the WIP, reimplement later**
- `slide-brake-rampdown` — taken out so the WIP ships a tuned steering/braking pair. **parked**, v0.9 and over; anything reimplemented must respect the pair **instant release, ramped recovery**.

**Tier 9 — recorded ideas, not started**
- `brake-learning` — a slide-learned ceiling per corner; a design call first, then wheel time, and the ceiling has to be per corner rather than per car.

**Relocated open items — ranked on entry**
- `low-speed-ramp-peak` — closed by reading the source; refile only from a symptom the driver describes. **closed**
- `grip-native-vs-lateral-g` — low-grip gate driver-verified; one unverified number under it, the grip native never checked against measured lateral g.
- `saveroute-precision` — 4-decimal route writes. **DECIDED and implemented** (`Math.Round(v.X, 4)` at `TrackFile.cs:132,137,142`; the props keep their own 2 decimals).
- `reverse-throttle` — removed by design; nitro charges at launch and per lap, no free-roam refill.
- `instability-tuning` — tuned by driving, not derived.
- `overspeed-correction` — driver-verified (`2dcbbcc`); the refresh rate and grade term remain.
- `tcs-controller` — a reason cap, not a P-integrator (`1ebbd2c`); its two knobs are shares of the peak slip (`b11e52b`).
- `steering-batch-residuals` — closed: the countersteer allowance and the damper's crossing were driven and stand as they are. **closed**
- `merge-span-gap` — a surviving merge does not reach its own first corner's entrance.
- `merge-rule-stress-test` — the two corner-merge rules need a stress test, the aligned window validated on one track only.
- `dnf-final-death` — DNF is final death, driver-verified (`0623276`).
- `rival-detection-route-frame` — route-frame detection, driver-verified (`0623276`); two residuals, both unobserved.
- `immersive-join-points` — **built and verified**: the chevron draws at the nearest start (`AutosportRacingSystem.cs:1880`), Context joins within 5 m (`:1885`), and the join takes the mode's anchor like any other start ("the join adds no pace override of its own", `:1887`). **closed**
- `manual-apex-placement` — place apexes by hand. **(idea)**
- `creator-apex-preview` — preview the apex table inside the creator. **(idea)**
- `surface-stats-revival` — track surface composition. **(idea)**
- `bump-ramp-window` — bumps are ramps, the airborne window; Stage 0 scan built and its overlay driver-verified (`f3f318f`). Detection must not reuse the crest instrument, ramps must be classified rather than treated alike, a bump on a crest takes the `min` of the two speed effects and never their sum, the classification lives in the consumer and never the detector, and both behaviours ship as a setting with an off state.
- `section-roughness-sweep` — a lateral normal sweep giving a section's roughness, which would make a one-lane bump visible. Call it roughness or section quality — **not** stability, which is the car's own state. **(idea)**
- `ceiling-factor-review` — closed (`0dfbf53`): reverted the midpoint to the outer wheel; the outer reference has since been driven. **closed**
- `steer-slip-allowance` — the measured-slip allowance under the ceiling; built, driven once, parked off, and what it should be is a design call. **parked**
- `wheel-contact-count` — a per-wheel ground-contact count for the stability check; the near-zero test is the part to fix, since a rolling tyre always reads a little noise. **not built**: verified, no contact-count symbol exists in `src`.
- `ceiling-factor-range` — closed (`29b267b`): the domain is 0.05 to 1.25 in 0.05 steps. **closed**
- `tcs-cap-floor-setting` — verified: `TcsCapFloor` (`Racer.cs:1938`, read `:2050`) has no menu item and no key while the threshold and the span above it both have one, so it is the one part of the policy the driver cannot reach.
- `overspeed-thresholds-setting` — verified: `OverspeedEnabled` is the only exposed setting (`AutosportRacingSystem.cs:1023`, `SettingsRepair.cs:147`); the five thresholds are consts with no key and no item (`Racer.cs:1966-1972`).
- `corner-record-dormant-fields` — verified: `RequiresEarlyBrake` (`DataStructures.cs:387`) and `RampEndNode` (`:388`) have no writer anywhere, and their only reader is the dead `MaxSpeedForBrakingDistance` (`AutosportRacingSystem.cs:3075`), so `speed-plan-dead-ends` takes both fields with it. Decide per field: re-arm it, or delete it with its branch.
- `grip-learner` — DISMISSED (driver). Do not rebuild it from memory: the two-sided bracket, its cleanliness filters and the grip-memory persistence were the dismissed design.
- `surface-path-unread` — verified dead: `ARS.TerrainGripMultipliers` (`AutosportRacingSystem.cs:287`) is cleared on load and written per node by the first two cars on track (`Racer.cs:4140-4142`), and read nowhere; the live grip path is `MeanWheelGripMultiplier` (`AutosportRacingSystem.cs:2901`) → `GroundGripMultiplier` → `CurrentMechanicalGrip`. Delete the table with its writer — deletion is the honest default.
- `intention-maxspeed-ramp` — closed (`f894228`): the remnant is deleted; compile-verified, not drivable. **closed**
- `speed-plan-dead-ends` — verified, all four: the `else if (Brain.Corner != null)` branch (`Racer.cs:1791`) is unreachable because `Brain.Corner` is set only inside `if (NextApexNode >= 0)` (`Racer.cs:3426-3443`), which kills `ARS.MaxSpeedForBrakingDistance` (`AutosportRacingSystem.cs:3064`), its only caller; plus the write-only `VehicleData.PerformanceIndex` (`DataStructures.cs:94`, written `Racer.cs:533`) and `TrackProgress` (`Racer.cs:64`, written `:3019`). Delete rather than annotate.
- `amputated-remnants` — closed: trimmed and compiled, the keeper decided.
- `slip-axis-scale` — settled by live probe and fixed (`b11e52b`); the disproved reading it overturns is corrected in `AGENTS-TECHNOTES.md`. Parked before the drive.
- `launch-slip-allowance` — allow more slip off the line, as a setting or a per-racer personality. **(idea)**
- `outside-approach-window` — the outside line is in force from far too far back; both time gates are driver-verified, and `RequiresBraking` is deliberately not cleared so a plan holds through the apex. **closed**
- `radius-variants` — audit the radius family. Verified rungs: per node `GeneralCurveRadius` and `PreciseCurveRadius` (`TrackLoader.cs:213-214`), the corner's `SupposedRadius` (tightest precise) and `DetectedRadius` (smoothed, `:713-714`), and the route's `CurveRadiusToFollowPoint` / `HighSpeedCurveRadius` (`Racer.cs:3125`, `:3135`). The dead rung: `HighSpeedCurveRadius` feeds only `Brain.CurrentIntention.CorneringSpeedLimit` (`Racer.cs:1854`), which no consumer reads.
- `lane-crossing-share` — the lane ask is scaled down as the car crosses, except into a corner (driver-verified, `da63a0d`); a soft scale that never reaches nothing, with turn-in exempt for the entry alone. **closed**
- `outward-lead-bound` — the aim may not lead the car's own lane by more than 1.5 m; driver-verified (`73756c7`). **closed**

## Agenda — the workstreams named for the coming days
- **A. Corner context, from the ground up** — one context-generation system naming brake point, turn-in and apex; diagnosed, partly shipped.
- **B. TCS and ABS as smooth curves** — the driver suspects `tanh` is simpler than the current remaps; independent of A.
- **C. Car-to-car avoidance, every angle and behaviour** — nearest-rival form driver-verified (`cbae845`); read `AGENTS-DUEL.md` first.
- **D. The card system** — refine the live maneuvers toward the designed duel model; priority ChillOut → DefendLane → DiveBomb → Yield, nitro slotless, and a played card is never reconsidered. Coupled to C, and A is upstream of both.
- **E. The route-frame ranking migration** — deferred to its own session; six consumers still rank by world `Distance`.
- **F. A central perception manager** — deferred, own session, needs the measurement first; it contains E.
- **G. Countersteer** — **closed (driver)**: taken with the steering chain, the slew left at one flat rate (`f84fe9a`).
- **Aftermath of A, driver-observed and to revisit** — a bend before a chicane now registers as its own corner.

## Held for a future version
- `pi-controls-merge` — merge `PI Offset` and `PI Target` into one row that takes a typed number, interpreted by Grid PI Mode. **DECIDED, not built.** Two things must not be re-derived: the row writes **two keys behind it** (`PaceOffset` in Relative, `PaceTarget` in Absolute), or a mode switch would reinterpret a target as an offset; and the number is **typed** through `Game.GetUserInput`, not picked from the ladder. The no-renaming alternative is two typed rows.
- `raceable-bikes` — bikes have never been tested, so 0.9.0 ships no bikes pool; the loader's `IsCar` gate and the class blacklist drop every line, and there is no tool to trial one with. **parked, post-0.9.0**

## Standing notes — no slug, still binding
- **Track creator — revived and driver-verified.** Creating a track is deliberately on again, so the WIP's old no-track-mutation property no longer holds, and updating a loaded track is impossible: the dead `UpdateRoute` was cut (`c2ad2de`). **Do not vendor the game's `materials.dat`** — a stale copy misaligns silently and a row reorder would be invisible, so read the live table instead — and **never feed a surfaced material back into `CurrentMechanicalGrip`**, which would make the prior self-validating.
- **Front slip angle is uncosted** — bounded by the steer ceiling, never priced in the pedal: TCS reads the longitudinal slip only, and body slip stays small in pure understeer.
- **Corrections are corrections — never reuse one as an absolute target.** The rule is embodied in the code: countersteer is rebuilt as a full target and the steer blends toward it, never toward the bare correction term.
- **Nitro pacing** — charged once at race launch and again on each lap increase, AI bottles gated on `AiNitro` with the player's never gated, and no free-roam refill.
- **Stability awareness — partially implemented**: `IsUnstable` reads on every timed core pass, firing when the chassis rides above its at-rest height or when its yaw rate demands more lateral acceleration than the grip allows.
- **Launch model — designed, not built (parked)**: start at a corner entrance at a multiple of the intended speed and integrate a ballistic projection forward, so a crest that unloads the tyres shows up as an airborne run.
- **Rear-end prevention — open ideas**, separate from the corner-specific Yield maneuver, which overrides the speed plan only when trailing a faster rival near an entrance.
- **Route read baselines** — the driver's proposal: fix a smoothing standard in metres instead of letting each read choose its own node count.
- **GitHub #3** — the reporter confirmed the fix, so it is a waiting game: close it once a week passes with no further reports. **Do not write a date here** — the commit that records the decision is the anchor.
- **Council review backlog** (`9783143`, `b845170`) — deferred findings to check when the area is touched again; the menu save-per-scroll scrub is the live one, its safe half done.

## The decision-ready top — each open item's state tag
- `dnf-final-death` — **driver-verified (`0623276`)**
- `rival-detection-route-frame` — **driver-verified (`0623276`)**, residuals unobserved
- `lane-repulsion-ceiling`, `player-special-ability`, `ai-free-abs` — **all DECIDED**
- `outside-approach-window` — **driver-verified (`7619347`, `1395c30`)**, both gates
- `crest-gate` — entry move driver-verified (`27ee097`), gating constants pending
- `steer-ramp-band` — **driver-verified (`489`, `e59e0a9`)**
- `side-slip-scale` — **unsettled**; probe the side-slip field beside the longitudinal one on a single drive before moving the sphere landmarks or the parked allowance's constants
- `ceiling-factor-review` — **closed (`0dfbf53`)**, the outer reference driven
- `tcs-cap-floor-setting`, `overspeed-thresholds-setting` — two open ranges; `ceiling-factor-range` is **closed (`29b267b`)**
- `launch-slip-allowance` — **(idea)**; the removed launch ramp last existed in `a6fcf7d`
- `rival-publish-core` — **not built**
- `apex-radius` — **driver-observed (`aa302e1`)** on high-grip cars; the fleet-wide read is owed
- `radius-variants` — the radius-family audit, asked for later
- `corner-record-dormant-fields` — filed, unowned; the early-brake branch is dead and the behaviour does not exist
- `grip-learner` — **DISMISSED (driver)**
- `slip-axis-scale` — **settled and fixed (`b11e52b`)**, the TCS share restatement and the ABS knee with it, drives parked
- `intention-maxspeed-ramp` — **closed (`f894228`)**, not drivable
- `lane-crossing-share` — **driver-verified (`da63a0d`)**, the comparison included (`b390029`)
- `outward-lead-bound` — **driver-verified (`73756c7`)**
- `slide-brake-rampdown` — **parked**
- `steer-slip-allowance` — **parked** after one drive
- `pi-controls-merge` — **DECIDED, not built**; wanted at the forefront of the next version
- `raceable-bikes` — **parked, post-0.9.0**
- `wheel-contact-count` — **not built**
- `amputated-remnants` — **closed (`8d1888e`)**
- **Steering is closed (driver)** — the limiter, its ramp band, the ceiling and its factor, the damper, the slew and the blend, the lane laws with the lane mover and its target timing, and the contact-avoidance term are all driven and accepted.
- **Pushed**, newest first: `053146e..ea87461` (the slip axis settled by probe, the TCS pair as shares of the peak, the ABS knee on that peak, the kick and debug trains); `da8a407..053146e` (the lane chain's run); `c2e7e6b..7872758` (the ceiling's midpoint reference, the wheel-slip spheres, the measured-slip allowance built then parked, the settled steering pair); `ac5514b..c2e7e6b` (the debug-visual train, the slide-raise retirement, the ramp/damper batch, the overspeed ramp, the lane base and the Council fixes).
- **Undriven across those runs**: the divebomb, the kick card and the debug-visual train — the drives owed are the last section of `TEST-PLAN.md`.
