# ARS AI System — How the AI Actually Drives

> Walkthrough of the per-racer decision pipeline.
> File: `Racer.cs`. Companion: `Racer.CallTree.md` for the method map.

## Mental model

A `Racer` is a finite state machine wrapped around a planner. The state
machine is intentionally small (`RacerBaseBehavior`:
`GridWait → Race → FinishedRace → FinishedStandStill`); the
intelligence lives in the *planner*, which runs every frame.

The planner answers three questions in this order:

1. **Where should I be on the road?** → lane offset along the track's
   normal axis.
2. **How fast should I be going?** → scalar target speed.
3. **How do I translate that into pedal and stick?** → `throttle`,
   `brake`, `steer` inputs to the vehicle.

Lane and speed are computed first; corrections (grip, slide, traction
control) and recovery (stuck, off-track) come second; the last step is
just a unit conversion to stick degrees.

---

## The two clocks

A `Racer` runs two loops:

- **`ProcessTick`** — every frame. Cheap perception, applies last
  frame's `VehicleControl`, and checks for state changes.
- **`RunTimedCore`** — every ~100 ms (`LastCoreTick`). Expensive work:
  rebuild track context, run the corner scan, refresh rival slots,
  rebuild grip, look at handbrake.

Most of the interesting logic is in the timed core. The per-frame
`ProcessTick` is mostly "actuate what I decided last time".

---

## Perception

### 1. Track following

`UpdateFollowTrack` does three things in sequence:

- `ResolveCurrentTrackPoint` — finds the nearest `TrackPoint` by
  projecting the car's position onto the polyline. The current point is
  held in `Racer.CurrentTrackPoint`.
- `BuildLookAheads` — populates `Racer.LookAheads`, a dictionary keyed
  by `eLookAheads` (`SteerRef`, `QuarterSec`, `HalfSec`,
  `ThreeQuarterSec`, `OneSec`, `OneHalfSec`, `TwoSec`, `SteerInRef`).
  Each lookahead is a `TrackPoint` projected to the time-to-reach
  distance at the current speed.
- `UpdateLapState` — increments lap counter when the racer crosses the
  start node.

Lookaheads are *time-based*, not distance-based. They adapt to speed:
at 200 km/h the half-second lookahead is much further down the road
than at 50 km/h, which is what the planner wants.

### 2. Dynamic bounding box

`UpdateDynamicBoundingBox` refreshes the car's directional size and
slide angle, both of which feed the rival logic and the corner-speed
planner. The bounding box is needed because some cars are nearly twice
as wide as others; using a fixed value would under- or over-estimate
the gap in tight corners.

### 3. Perceived grip

`UpdatePercievedGrip` (sic) is the most important perception step.
Grip is rebuilt from three sources, weighted by stability:

- `BaseMechanicalGrip` — read once from the vehicle's handling.
- Surface terrain — the car's current surface hash is checked against
  `ARS.Road`, `ARS.Dirt`, `ARS.Sand`, etc. Each terrain has its own
  multiplier.
- Observed Gs — the G-meter on the body is used as a feedback signal
  to detect wheelspin and lockup.

The result lives in `VehicleData.AvgGroundStability` and is used by:

- `SteerApplyCorrections` — slide counter-steer magnitude.
- `SpeedTrack` — corner speed cap.
- `TractionControl` — throttle cap.

### 4. Corner info

`UpdateCornerInfo` is responsible for "what is the next corner, and
how am I approaching it?". It maintains `Brain.Corner`, which is the
currently-active corner and the time-to-entrance in seconds. The corner
becomes valid on lap > 0; the very first lap the racer just follows
the path.

The key fields on a `Corner`:

- `OG` — the underlying `CornerPoint` (the apex, in effect).
- `Speed` — the speed the planner wants at the apex.
- `sToEntrance` — seconds to the corner start, used as the brake
  countdown.
- `Approach` — whether the approach has been sanity-checked (no
  rival in the way, no off-track danger, etc.).
- `stabilityStrat` — `BrakeStabilityStrat`: `Invalid` → `Checking` →
  `Checked`, a small state machine for "is it safe to floor the brake
  here?".

---

## Decision: steering

`SteerTrack` is the lane planner. The shape is:

1. **Gather context** — read the three required lookaheads
   (`SteerRef`, `HalfSec`, `OneHalfSec`) and the road width at the
   planning node. Bail out early if any are missing (e.g. during
   `GridWait` or in the first few nodes).
2. **Pick a base lane offset** —
   - On lap 0, follow the center of the road.
   - On lap 1+, follow a *learned* lane profile: `Brain.data.DeviationFromCenter`.
   - In a corner, apply `ApplyCornerLaneProfile`, which interpolates
     from the entrance lane to the apex lane and back out, using
     values stored in `ARS.NodeScalarData` (the
     "corner lane profile" learned from prior racers or from
     self-driving).
3. **Apply inside-line bias** — the racer hugs the inside of a
   corner. The magnitude is `keepInside / Handling.Grip`, so a
   high-grip car uses a sharper inside line.
4. **Clamp to road width** — `FollowLane` is clamped to
   `[-roadWide + BoundingBox, roadWide - BoundingBox]`.
5. **PID to steer target** — `LanePID.SetTarget(FollowLane)`, then
   project a target point to the side of the road and use the angle
   between that point and the car's forward vector as the desired
   steer angle.

`SteerApplyCorrections` then physics-clamps that angle:

- **Speed-based steer limit** — a kinematic max
  (`v² / (g * wheelbase)`). At 300 km/h, the car physically cannot
  turn more than ~3° without flipping.
- **Yaw vs expected yaw** — the actual yaw rate is compared to the
  expected yaw rate at the requested angle. If oversteering, the
  steer angle is reduced proportionally; if understeering, the
  script leaves it alone (the comment in code notes this is
  deliberate).
- **Slide counter-steer** — if the car is sliding and the slide
  direction agrees with the yaw direction, subtract a small
  counter-steer.

`TranslateSteer` is the last-mile mapping from `SteerTrackDegrees` to
`SteerInput` (the actual -1..1 stick). This is the **only** place a
rate limiter is applied — `Racer.md` notes that future refactors must
not add a second one.

---

## Decision: speed

`SpeedTrack` produces `Brain.intention.Speed`. The intent is the
*minimum* of four caps, in increasing order of pessimism:

- `EngineTopSpeed` * 1.3 (GTA lets cars exceed their meta top speed).
- `Brain.intention.MaxSpeed` (per-racer cap, set by personality and
  aggression).
- A corner-speed cap derived from the next corner's radius and the
  current grip.
- An "outward corner" cap — if the racer is on the wrong side of the
  road heading into a corner, the cap drops, because the inside line
  is no longer reachable.

`SpeedToThrottleBrake` then converts the speed error (in Gs) to
throttle and brake:

- Positive error → throttle, with a soft cap from `TCSThrottle` and
  the per-frame stability limit.
- Negative error → brake (if going forward) or reverse (if the
  intent is actually negative).
- The output is rate-limited: `Brake += (target - Brake) * 5 *
  TickScale` per frame. That's a critically-damped feel.

A corner needs a brake-then-rotate-then-throttle pattern. The
`stabilityStrat` state machine on `Corner` is what enforces that
order: while the racer is still "checking" the corner, the brake is
allowed to be high; once "checked", the racer commits to a
trajectory and the planner refuses to add more brake.

---

## Decision: rivals and aggression

`UpdateRivals` selects the three nearest racers into `Memory.Rivals`
slots. The selection prefers:

- Direct ahead / direct behind (longitudinal relevance).
- Adjacent lateral (overtaking relevance).

`UpdateRivalInfo` then computes for each rival:

- `relativePos` — `Ahead`, `Left`, `Right`, `Behind`, `Unreachable`.
- `Distance` — center-to-center, used for overtake commitment.
- `sToReach` / `sToRear` — if I'm going faster, how many seconds to
  reach and clear the rival.
- `OccupiedLane` — the rival's current lane offset; this is what
  the steer planner uses to avoid T-boning.
- `DirectionDiff` — are we going the same way, or is the rival
  crossing?

`PersonalityRivals` controls *how* the racer reacts:

- `SideToSideMinDist` — the minimum gap it will leave on the side
  before aborting an overtake.
- `BehindRivalMinDistance` — how close it dares to sit in a slipstream.
- `AggressionBuildup` — how quickly `Brain.intention.Aggression`
  rises while stuck behind a rival.
- `ManeuverRiskFactor` — how much `Aggression` shortens the brake
  distance.

`Aggression` is itself stateful. It rises when blocked, decays when
free, and is reset on a successful pass. It also caps the
`LateBrake` / `Flatout` / `AttackInside` decisions in the `Decision`
enum.

---

## Decision: corrections

Three correction modules run after steer and speed have been planned:

- **`TractionControl`** — caps throttle when wheelspin is detected.
- **`ABS`** (lock-up limiter) — releases brake if the wheels are
  locking.
- **`SteerApplyCorrections`** — already covered above.

These are not optional. They are the only thing keeping the AI from
spinning out at 200 km/h.

---

## Stuck and off-track recovery

If the racer is moving but the throttle is fully open and the speed
hasn't changed for 2 seconds (`StuckCheckTimeMs`), the racer is
considered "stuck by throttle". It then enters a recovery sequence
(`StuckRecoveryTimeMs` = 2 s) which:

1. Releases throttle.
2. Applies brake.
3. If still stuck after the window, escalates (`StuckRecoveryAttempts`).

`Racer.md` notes that the escalation logic is currently duplicated in
two call sites (`ProcessAI` and `ProcessTick`) and is a refactor
target. The state machine is informal; `Idle`, `Tracking`,
`Recovering`, `EscalationKick` are the intended future states.

Off-track recovery is more forgiving: when `OutOfTrackDistance() >
0.5f`, the throttle floor is raised to 0.45 so the racer can
self-extract.

---

## Personality and variance

`PersonalitySet` is the per-racer style. The fields that matter for
feel:

- `Name` / `ProbToUse` / `SkillRange` — the menu shows these; the
  script picks one at random per spawn.
- `Stability.Skill` (0–100) — caps the risk the racer will take.
  Higher = faster and more willing to slide.
- `Stability.UndersteerFactor` — used in the corner speed planner;
  a high value pushes the racer to slow down more aggressively.
- `Stability.NoSlide` / `MaxSlide` — bounds on how much lateral
  movement the racer will tolerate.

`BehaviorVariance` is a per-instance jitter (80%–120% of 1.0) for
`BrakeDistance`, `SteeringStrictness`, `SpeedAggroVariance`, and
`SpeedBaseVariance`. It is the simplest way to keep twenty AI racers
from driving in lockstep.

---

## What the AI cannot do

- It cannot plan *multi-corner* sequences. Each corner is decided
  independently based on the next one.
- It cannot recover from a serious crash. If a racer is on its roof
  and not moving, the stuck recovery will try once and then give up.
- It cannot adapt to weather or time-of-day; the surface hash is
  enough for normal play.
- It does not know about elevation changes well. `TrackPoint.Elevation`
  is recorded but the planner does not yet use it for the corner
  speed cap.

These are all visible in the code as the obvious "next thing to
improve" features. See `Racer.md` for the prioritized refactor
backlog.
