# Rate & precision plan — spending the CPU headroom

**Status: plan only. Nothing built, nothing committed.** Open questions at the end are unresolved on purpose.

## Decisions taken in discussion

- **The adaptive signal is measured cost, not frame rate.** Driving the batch from fps closes a loop
  around the thing being perturbed — AI work raises frame time, frame time lowers the allowed count —
  so it hunts unless heavily damped, and a damped loop cannot protect a single heavy frame. It also
  cannot distinguish a CPU-bound frame from a GPU-bound one. Measured cost is feedforward, adapts to
  the machine, and measures what is actually being spent.
- **A fixed batch bump is conditional, not the plan.** A budget that affords every car every frame beats
  a larger fixed count outright, so the constant is only worth touching if the measurement says a budget
  cannot buy full rate.
- **Measure before deciding.** One instrumented run with a 30-car grid turns this from a design argument
  into arithmetic.
- **The first step keeps the round-robin deterministic.** Capping the batch at half the grid buys most of
  the rate an adaptive schedule would, while remaining a fixed rule: identical behaviour on every machine
  and every replay. Adaptation stays on the table only for hardware where a fixed two-batch cycle cannot
  hold the frame.
- **No precision change ships without an A/B on frame rate.** At 30 cars, the achieved frame rate is
  measured before and after. The AI's rate is allowed to change; the frame rate is not. This is the guard
  against the one real danger — more AI work, a lower frame rate, and therefore a *slower* AI, which is
  the opposite of the goal.

## Why this exists

A 30-car grid holds frame rate (driver-observed). That means the per-frame budget is not the binding
constraint, but it also means the per-frame budget is currently being *spent on safety*, not on
accuracy. This plan inventories where resolution is being thrown away and stages the work that buys it
back.

The driver's own example was the precision of the G forces. The audit answered that question — and the
answer is that **the G window is not the problem**; the problem is one layer below it.

## The finding

There are two layers, and they run at different rates:

- **The sensor layer is already full-rate.** `GET_ENTITY_SPEED_VECTOR` is read every frame for every
  car, regardless of the sampling gate (`Racer.cs:2221`, `Racer.cs:2238`). The native is paid for.
- **The decision layer is not.** `RunTimedCore` (`Racer.cs:2520`) is the whole perception-and-control
  pipeline — track position, slide/bounding box, perceived grip, `TRLateralAtSpeed`, then `ProcessAI`
  → target speed, steering, both pedal caps, pedals — and it is fed by a fixed batch of six racers per
  frame (`AutosportRacingSystem.cs:1882-1891`).

So each car's core ticks at `6 · fps / cars`:

| cars | core rate at 60 fps | command held for |
|---|---|---|
| ≤6 | 60 Hz | 17 ms |
| 12 | 30 Hz | 33 ms |
| 20 | 20 Hz | 50 ms |
| **30** | **12 Hz** | **83 ms** |

The frame rate holds because the per-frame *cost* is constant; the AI's *rate* silently divides with
grid size. At 30 cars a car travels ~2 m at 25 m/s between one steering decision and the next.

The design already compensates for the varying interval — every rate-dependent term is scaled by
`TickScale` (`Racer.cs:1532`): pedal and steer slew (`:1306`, `:1544`), wall opening (`:999`), max-speed
ramp (`:1346`), brake-learning sample time (`:1448`), reason-cap glides (`:1773`), pressure
(`:3412`). That is why behaviour survives at 12 Hz instead of falling apart. But `TickScale` fixes
**rates**; it cannot fix the **sampling delay**, which is pure phase lag in the control loop. That is
the real limit, and it is why a big grid reads as the AI turning in late rather than moving slowly.

**One edge to know:** the pedal slew's hitch guard sits exactly at the 30-car rate. `PedalSlewRate *
TickScale` = 6 × 0.0833 = 0.5, and `PedalSlewMaxPerTick` is 0.5 — so at 30 cars the ramp is at its
designed full-range-in-167 ms and not a hair further. Any further cadence drop (more cars, or a frame
rate under 60) and the guard binds and the pedal ramp stretches.

## The G-force verdict: keep the window, fix two free defects

The ten-sample mean **telescopes**:

```
(1/10) · Σ (v_k − v_{k−1}) / d  =  (v_now − v_{now−W}) / W
```

so it is a single two-point secant over the window, not ten independent looks. The ten samples buy no
averaging: noise is `√2·σ_v / W` and lag is `W/2`, regardless of how many samples fill the window.

And the quantity it feeds is small. The Gs-aware preview shifts the aim by half the projected lateral
motion, `0.25 · a · t²`, which at the preview time is `0.0625 · a` — **0.61 m at 1 g**. Pure pursuit
turns a 0.61 m lateral aim offset into **0.47° of steer at a 20 m lookahead, 0.21° at 30 m**. The whole
acceleration term is worth under half a degree; the window's delay costs roughly 0.2° of that.

**Do not shorten the window.** Shortening to ~100 ms multiplies the noise 3.3× to buy a fifth of a
degree on a term capped by the above.

Two things *are* wrong, and both fixes are free:

1. **The window lies.** `AccelWindow = 10` × `AccelIntervalMs = 20` (`DataStructures.cs:14-15`) reads
   as 200 ms, but the gate is `elapsed >= 20` on an integer-ms clock (`Racer.cs:2224-2226`), so the
   sample gap is `ceil(20 / T_frame) · T_frame`. At 60 fps that is 33.4 ms → **W ≈ 334 ms**, not 200.
   Worse, the window then *changes with frame rate* (at 144 fps it falls toward 208 ms), so the AI's
   turn-in depends on the renderer's load.
2. **The estimator is never reset.** `Launch` clears the peaks but not `_lastSpeed` or the sample ring,
   so a respawn or a `SnapToTrack` teleport injects a multi-G spike into the mean for a full window,
   and into the 3 Hz overspeed read (`Racer.cs:3664`).

The estimator is also **not** what feeds the per-lap peaks — those take the raw single difference
(`Racer.cs:2235`), which is display-only and gated at grip × 1.33.

## Ranked inventory

Cheapest / highest value first. All quantities refresh per core tick unless a timer says otherwise, so
the batch rate multiplies through everything below it.

| # | Item | Cadence | Anchor | Buys | Cost | Hazard |
|---|---|---|---|---|---|---|
| 1 | Cache the native property reads per core tick | per core tick | `Racer.cs:416` (~15 reads in `ComputeSteering` alone), `:2734`, `:3645` | nothing directly — **frees** the budget that pays for #3 | negative; removes ~40-60 natives/tick | none; values cannot change inside one tick |
| 2 | Reset the accel estimator on `Launch` / `SnapToTrack` | once | `Racer.cs:1243`, `:3622` | removes a multi-G spike polluting a whole window | zero | none |
| 3 | Window the accel samples by elapsed time | per tick | `Racer.cs:2224-2233` | the constant means what it says, at every fps; removes the frame-rate dependence of turn-in | zero natives (the sample is already read each frame) | noise unchanged; only the window becomes honest |
| 4 | Batch size / budget-aware batching | 6 per frame | `AutosportRacingSystem.cs:1882` | **the main lever** — 12 → 24 Hz at 30 cars; steering command held 83 → 42 ms | linear in the new count | control-loop ZOH changes (drive it); the pedal guard above |
| 5 | `UpdateRivalInfo` at core rate | 500 ms | `Racer.cs:3354-3357` (and redundantly at `:3303`) | rival gap/closure data 0.5 s → ≤83 ms; ~30 m → ~5 m of gap error at 60 m/s closure | ~+5-6 k natives/s at 30 cars | avoidance/card gates were tuned on 2 Hz data — **behaviour** |
| 6 | Overspeed sample at core rate | 333 ms | `Racer.cs:3664` | measured-Gs lag 333 → 83 ms (20 m → 5 m at 60 m/s) | a memory walk per tick | the excess ladder is coarse and `GlideCap` smooths, but it can flap |
| 7 | Apex-queue refill | 500 ms | `Racer.cs:2811-2814` | braking-plan refill 500 → 83 ms | a managed sort/filter | queue churn; partly moot — `UpdateCornerRequirements()` forces a refill on a flip |
| 8 | Speed-scale the track-node search window | per core tick | `Racer.cs:2713-2714` | removes a systematic node lag: a car covers `v · TickScale` per tick (11 m at 134 m/s) against a ±6 m window, so it loses the car above ~72 m/s | ~0 natives (managed compares) | wrong deck on a stacked track — clamp to half a lap |
| 9 | HUD standings (`_posUpdateTickMs`) | 200 ms | `AutosportRacingSystem.cs:1924` | standings lag 200 → 16 ms | a managed sort of 30 | none, display-only; **low value** |
| 10 | `_halfSecondTick` | 500 ms | `Racer.cs:3289` | **nothing — write-only; set and never read** | — | free deletion |

Also dead: `GetLateralGs` (`DataStructures.cs:52`) has no callers.

## Staged plan

Order matters: stage 2 is what makes stage 3 affordable, and stage 3 is the one that risks behaviour.

### Stage 0 — measure (no behaviour change, and it decides everything below)

Time the batch at `AutosportRacingSystem.cs:1878-1892` with `Stopwatch.GetTimestamp()` and log once per
second, on a 30-car grid:

- cores admitted per frame, and the achieved core interval per car;
- the milliseconds spent in the batch that frame;
- the mean **and the worst** cost of a single core tick.

The worst tick matters as much as the mean, because a core tick is not homogeneous work: the 1 s and
500 ms timers put `UpdateRivals` and `UpdatePressure` on some ticks. Those scans are O(N²) **across a
second, not inside one tick** — each racer scans the field once when its own timer fires, so a tick
carrying one pays for a single scan of the field, not for the whole grid's. The distribution still has a
tail, and a budget sized on the mean would under-provision on the ticks that carry a scan.

Acceptance: a quotable cost per core tick and a visible tail. Copy `Log.log` out live — it is truncated
at script init (`AGENTS.md`).

### Stage 1 — free and honest (no natives, no driving change)
- Reset `_lastSpeed` and the acceleration ring in `Launch` and `SnapToTrack`.
- Window the acceleration samples by elapsed time so the configured window is the real window at every
  frame rate.
- Delete `_halfSecondTick`.

### Stage 2 — pay for stage 3 (cost-negative)
- In `RunTimedCore` (`Racer.cs:2520-2530`), read `Car.Position`, `Car.Velocity` and `Car.ForwardVector`
  once per tick into locals and thread them through `ComputeSteering`, `UpdateTrackPosition` and
  `UpdatePerceivedGrip`, instead of re-reading the native at every use.
- `UpdatePressure` re-reads `Car.Position` once per candidate *inside* its loop (`Racer.cs:3397`). The
  car cannot move mid-tick, so it belongs in a local above the loop — at 30 cars that is one redundant
  native read per rival, twice a second, per car.
- `UpdateRivals` allocates two lists per call and removes from their middles (`Racer.cs:3701-3702`,
  `:3722-3723`); the pick needs neither.
- **This stage is the point of the plan**: the AI's cost is dominated by native reads, so precision is
  bought cheapest by deleting redundant reads rather than by deleting rate.

### Stage 3 — the real lever: a two-batch cycle (own build, own drive)

Cap the batch at half the grid instead of the fixed six, so the cycle is always two frames:

- `count = min(Racers.Count, max(6, ceil(Racers.Count / 2)), MaxCoresPerFrame)`.

At 30 cars that is 15 and 15: the AI's rate doubles to 30 Hz at 60 fps, the index skew between two cars
falls from four frames to one, and the pedal slew's hitch guard stops binding. The floor of six matters —
at six cars every car already ticks every frame, so a naive "always two batches" rule would *cut* that to
30 Hz for nothing. The absolute ceiling matters on very large grids, where half the grid stops being a
small number: at 48 cars the cycle lengthens to three batches rather than two. Read its value off stage 0.

The rule changes what the rate depends on, which is the real prize: with `count ≈ N/2` a car's core rate
is roughly **half the frame rate at any grid size**, instead of `6 · fps / N` collapsing as the grid
grows. That is the half of the problem that is ours to fix; the other half is in the section below.

Verify on a 30-car grid that the frame still holds, and watch a hard turn-in for the earlier, smoother
command.

## The scheduler, if stage 0 says a budget cannot buy full rate

Conditional on the measurement. If 30 core ticks fit comfortably inside a slice, skip this and raise the
constant instead.

- **Budget** — spend at most a fixed slice of the frame on AI work.
- **Smoothed cost** — drive admission from an exponential average of ms per core tick, never the
  instantaneous measurement: one OS preemption or one heavy tick would otherwise collapse the batch to a
  single car. The tail above is why the average must be respected rather than raced against.
- **Round-robin admission** — reuse the existing `_nextInLine` cursor so no car is favoured.
- **Ceilings and floors** — always at least one core per frame, and never more than `Racers.Count` per
  frame, so no car is double-ticked on a machine with time to spare.
- **Staleness deadline** — a pure budget starves: if the slice affords four of thirty cars, every car's
  cycle stretches and a slow machine degrades without limit. Any car past a maximum staleness is
  admitted regardless of budget, bounding staleness at the cost of the occasional over-budget frame.
  That trade is deliberate — the AI must not starve.

## Outside our control

The AI's rate is bounded above by the frame rate: a car cannot tick more than once per frame, and no
engineering here creates ticks the renderer does not produce. On a machine that cannot exceed its capped
frame rate, that ceiling is the whole story.

- **At 30 fps** the two-batch rule gives the AI ~15 Hz, at any grid size — above the rate already proven
  acceptable, since 30 cars at 12 Hz is the configuration being driven today.
- **Below ~24 fps** the rate falls under that proven floor and the AI degrades qualitatively. Nothing in
  ARS recovers it. The honest response is to say so rather than degrade silently.
- The same bound applies to everything else loading the machine: other mods, traffic, peds, the renderer.

Also not ours: the player's driving is unpredictable by definition, so the AI must tolerate it rather
than predict it; the player's special ability is a real grip and steering advantage the AI cannot match;
and the engine grants the AI free ABS while withholding it from the player.

**Therefore: detect and state the condition.** Stage 0 already measures the achieved core interval, so it
should also log once when that interval falls under the viability floor. The difference between "the mod
is broken" and "the hardware is the limit" is worth one line in a log.

## Deliberately out of scope

- **`UpdateRivals` or `UpdatePressure` at a higher rate.** Both are O(N²) in native position reads —
  at 30 cars that is ~900 reads per refresh, ~54 k/s if run per frame against ~900/s now. 2-4 Hz is
  the ceiling without a design change.
- **The mean's length** for the overspeed detector and the flag extrapolation. The lag is the safe
  side of that trade and the ladder is coarse.
- **`AccumulateLapPeaks`.** Display-only, hard-gated; "fixing" the peak G is a HUD change.
- **`TickScale`.** Correct as-is; per-tick constants would break every grid size.
- **`_phaseOffsetMs`.** Deliberate desync so rivals do not all tick together.

## Noted, not scheduled — the route-frame ranking migration

**Deferred to its own session. Do not fold it into the stages above.** It is a behaviour change, and the
precision work must stay separable from it.

The route frame is only a third migrated. Three pieces, and only the first is done:

1. **The hit test** (`TimeToReach`, `FrontGap`, `RouteGapAhead`) — route frame, done.
2. **The candidate scan** (`UpdateRivals`, `Racer.cs:3699-3711`) — world distance, 39 native position reads
   per racer per second at a 40-car grid. The route value is already cached per racer, so the filter costs
   no natives at all.
3. **The ranking** — six consumers still order rivals by `Rival.Distance`, which `UpdateOffsets` sets as
   world-space straight-line distance (`DataStructures.cs:152`, `:201`): `Racer.cs:1657`, `:1969`,
   `:2033`, `:2084`, `:2113`, `:2138`. No consumer trusts slot 0, so the slot order itself is free.

Doing 2 without 3 would filter by route and then rank by world, consistent only by accident.

Mechanics: `|Δ CumulativeDistance|` with the shorter-arc wrap on a circuit, as `UpdateRouteGap` already
does; metres rather than node index, because nodes are not evenly spaced; keep the range gate; guard a
racer with no `CurrentTrackPoint` yet and `RouteLengthMeters` at zero; and pick three by insertion into
three registers instead of allocating the two lists the current pick builds.

Consequence to accept deliberately: it drops physically-close but route-far cars — a hairpin's opposite
leg, a bridge from the road beneath — which is correct for racing and a real loss for collision. It also
removes the known route-frame residual by construction, since that residual is a world-space scan putting
the car in the list. And it changes which rival each maneuver targets, so it needs its own drive.

## Open questions — to settle before building

1. **Adapt at all, or stay deterministic?** The two-batch cap is a fixed rule, so behaviour is identical
   on every machine and every replay; the adaptive scheduler is not, and that reproducibility was its
   real cost. The scheduler only earns its place if the measurement shows a fixed two-batch cycle cannot
   hold the frame on weaker hardware.
2. **What is the budget in milliseconds, and does the staleness deadline override it?** Absolute ms or a
   share of the frame? The number wants stage 0's tail, not its mean.
3. **Should the adaptive count be visible in the UI at all?** The menu window is fixed at ten rows, and
   the value is grid-global — worth showing, or keep it internal?
4. **`UpdateRivalInfo` at core rate is a behaviour change.** The avoidance and card gates were tuned on
   2 Hz data. Do we accept the change, or keep the cadence and only drop the redundant call?
5. **Do we want the short `ProjectAhead`-only window?** It restores preview lead for ~0.2° more steer
   at 1 g, but it is a live-steering change on a term worth under half a degree.
6. **The node-search window at speed** — worth the stacked-track exposure?
7. **Verification of the finished work.** A once-per-second line with the achieved core interval and the
   achieved sample gap, plus the same corner driven at a capped and an uncapped frame rate.

## Verification notes

- Nothing here should be bundled with a behaviour change. Precision work must be separable, so a
  regression has one suspect.
- Every code change follows the standing gate: build → reload → drive → then commit.
