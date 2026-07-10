# Corner AI Review — Three Angles

> Review of how ARS detects, plans speed for, and chooses lines through corners.
> Each section is one angle; together they form a full picture.

---

## 1. Corner Detection and Validity — by Ministral

The corner detection system in ARS is a two-phase pipeline:
1. **Track preprocessing** (static, at load time)
2. **Per-racer corner validity** (dynamic, per-frame)

### 1.1 Track Preprocessing

Every `CornerPoint` gets:
- `Angle` (signed angle between 20 nodes ahead/behind)
- `Elevation` (track elevation at the node)
- `ElevationChange` (vertical slope)
- Default `LengthStart`/`LenghtEnd` = 20 nodes (~100m for a typical track)

**Key corner selection** (lines 4233–4244): a corner is "key" if its radius is a
local minimum (smaller than ±5 neighbors). This filters minor bends, focusing on
apex points (hairpins, tight sweepers).

**Span expansion** (lines 4253–4279): for each key corner, expand start/end nodes
until the radius exceeds `thresholdRadius = radius * 50`. Clamped to 5–300 nodes
and capped at `trackWide * 4`.

**Lane profile build** (lines 4302–4417): for each key corner, generate a quadratic
Bézier with anchors at outside (factor 0.9) → inside apex (factor tuned
iteratively up to 300 attempts, range [-5, 5]) → outside. Sample the curve at
24–300 points, project each track node to the closest Bézier sample, store the
lateral offset in `NodeScalarData`.

**Overlap handling** (lines 4283–4300): if two key corners are too close, the
later one is discarded. No span adjustment.

**Robustness**:
- Works well for **sweepers**.
- Struggles with **chicanes, esses, and tight hairpins** (radius threshold too high).
- No **multi-corner planning** — each corner is treated independently.
- **Elevation changes** are recorded but not used in corner-speed planning (only in grip adjustment).

### 1.2 Per-Racer Corner Validity

`Brain.Corner` is the active corner. `UpdateCornerInfo()` (Racer.cs:724) and
`LookForCornerAhead()` (ARS:4973) maintain it.

**Validity rules**:
- Lap 0: `Brain.Corner.Valid = false`.
- Lap ≥ 1: valid until the racer passes `endNode`. Then variance is reset to 80–120% of 1.0.

**Time-to-entrance**: `Brain.Corner.sToEntrance = distance_to_start / speed`,
clamped to [0, 99]. Used to gate brake authority via `BrakeStabilityStrat`.

### 1.3 BrakeStabilityStrat State Machine

| State | Transition | Purpose |
|-------|-----------|---------|
| `Invalid` | Corner detected → `Checking` | Corner exists but not approved for braking |
| `Checking` | Brake applied → `Checked` | Brake allowed to slow for the corner |
| `Checked` | Corner passed → `Invalid` | Brake clamped to avoid late braking |

**Problem**: Once `Checked`, the racer cannot add more brake even if conditions
change (e.g., a rival blocks the apex). The state machine is **too rigid**.

### 1.4 Weaknesses and Edge Cases

| Issue | Impact | Potential Fix |
|-------|--------|---------------|
| Chicanes | Not detected as key corners | Lower `radiusFactorForCornerBounds` or add a chicane detector |
| Esse curves | Treated as two separate corners | Use curvature continuity to merge nearby corners |
| Hairpins | May be missed if radius threshold is too high | Dynamically adjust threshold by track type |
| Elevation ignored in speed | Speed spikes on crests | Use `GripGainLossElChange` to reduce speed on crests |
| Outward corner limiter | Disabled (`1==2`), racers ignore wrong-side positioning | Re-enable and tune `outwardThresholdDeg` |
| Fixed lane profiles | Inherit bad lines from first racer | Add grip-based line adjustment |
| No multi-corner planning | Each corner independent | Predict next corner's radius and adjust speed earlier |

---

## 2. Corner Speed Planning and Execution — by Minimax

The speed target is computed in `SpeedTrack()` (Racer.cs:540–627):

```csharp
Brain.intention.Speed = Math.Min(Math.Min(cornerSpd, followTrackSpd), maxSpeedForSteerAngle);
```

### 2.1 Corner Speed Derivation — `GetSpeedForCorner`

`v = sqrt(grip * g * radius)` — a textbook formula assuming 1G of lateral
acceleration at the limit and a perfect circular corner.

**Ignores**:
- Speed-dependent grip (GTA's grip curve is nonlinear)
- Weight transfer during braking/acceleration
- Elevation changes (the code calculates `elDeltaGs` but **comments out** the
  grip adjustment at line 4654)

### 2.2 Braking Distance — `MapIdealSpeedForDistance`

```csharp
float targetDistance = (c.Node - c.LengthStart - r.CurrentTrackPoint.Node) - 25 - r.Car.Velocity.Length();
float timeToReachTarget = Math.Max((targetDistance / velCurrent), 0.01f);
float spd = velTarget + (float)((brakingAbility * r.Handling.Gravity) * timeToReachTarget);
```

**Problem**: When `targetDistance` is large (racer is far from the corner),
`timeToReachTarget` is large, and `spd` overshoots the target speed. This is why
racers **brake too early** on long straightaways.

### 2.3 Follow Track Speed

Uses `CurveRadiusToFollowPoint` (a lookahead node's curve radius). More generic
than corner-specific speed, but doesn't account for drift (the actual path
radius is larger than the track radius).

### 2.4 Max Speed for Steer Angle

Reverse-engineers the max speed from the current steer angle:
`v = sqrt(grip * g * (wheelbase / tan(steer)))`.

**Problem**: This is a **self-fulfilling prophecy**:
- Hard turn → speed capped to match.
- Gentle turn → more speed allowed, encouraging over-speed.
- No lookahead — reacts to the **current** steer angle, not the **planned** corner.

### 2.5 The `Decision` Enum

Defined (ARS:27) as `{ LateBrake, Flatout, NoOvertake, FastCorner, EarlyExit, AttackInside }`
but **never referenced** in `Racer.cs` or `SpeedTrack()`. Planned but not
implemented, or used in a different module.

### 2.6 The Disabled Outward Corner Limiter

Racer.cs:590–604 — the condition `1==2` hardcodes this to `false`. A debug
artifact left behind. If a racer gets pushed wide, it **won't slow down** to
get back to the racing line.

### 2.7 Failure Modes

| Symptom | Root Cause |
|---------|------------|
| Brake too early | `MapIdealSpeedForDistance` overshoot |
| Brake too late | No lookahead curvature; reacts to current steer |
| Speed spikes on crests | `GripGainLossElChange` commented out |
| Wide in corners | Outward limiter disabled |
| Inconsistent lines | `NodeScalarData` not recalibrated |

### 2.8 Recommendations

**Quick fixes**:
1. **Re-enable the outward corner limiter** (change `1==2` to `true`).
2. **Un-comment the grip adjustment** (line 4654).
3. **Add a lookahead curvature buffer** — predict steer angle 50m ahead.

**Medium-term**:
4. **Fix the `MapIdealSpeedForDistance` overshoot** — cap the delta added to
   `velTarget` (e.g., max +20 m/s); use a decaying function.
5. **Add weight transfer to grip calculation**.
6. **Implement the `Decision` enum** — personality-driven speed bias.

---

## 3. Line Choice — by Primary (minimax-m3)

Two systems determine the racing line:
1. **Static lane profile** — built once at track load, stored in `NodeScalarData`.
2. **Dynamic inside-line bias** — computed per-frame in `SteerTrack()`.

### 3.1 The Static Lane Profile (Build Time)

A quadratic Bézier from **outside → inside apex → outside**:

```
        Bezier: outside → apex → outside
              ╱─────╲
             ╱   •   ╲        ← control point (apex, inside)
            •         •
   start ─•            •─ end
   (outside)          (outside)
```

- **Start/End**: `outsideAnchorFactor = 0.9` of track width on the outside.
- **Apex**: `apexAnchorFactor` of track width on the inside, tuned iteratively
  (300 attempts max, range [-5, 5]).

**What gets stored**: for every node in `[startNode, endNode]`, the signed
lateral offset from track center to the closest Bézier point.

### 3.2 The Dynamic Bias (Run Time)

`SteerTrack()` (Racer.cs:249–260) computes target lane per frame:

**Regime A — inside a known corner (lap ≥ 1)**: use the learned Bézier profile
via `ApplyCornerLaneProfile()`.

**Regime B — otherwise (lap 0, straight, exit)**: apply a dynamic inside bias:
`targetLane = Brain.data.DeviationFromCenter - (keepInside / Handling.Grip)`.

**Problem with Regime A**: The profile is **fixed once per track**. If the first
racer takes a bad line, every racer inherits it. No recalibration based on actual
grip conditions.

**Problem with Regime B**: If the car is on the wrong line, this bias
**reinforces the wrong choice** — the sign convention assumes the car is heading
roughly toward the corner.

### 3.3 The Lane Application

`ApplyCornerLaneProfile()` (Racer.cs:299–335) has three states:

1. **Pre-corner** (`followBaseNode < startNode`): use the **entrance lane** (outside).
2. **In-corner** (`startNode ≤ followBaseNode ≤ endNode`): use the **Bézier profile**.
3. **Post-corner**: not handled — falls through with `localTargetLane` unchanged.

**Problem**: There's no **exit** handling. Once the racer passes `endNode`, it
stays at whatever the Bézier gave it for the last node (which is the *outside*
of the corner exit — correct by accident).

### 3.4 Why the Line Looks "Off"

| Symptom | Root Cause |
|---------|------------|
| Racer misses apex on first attempt | `apexAnchorFactor` tuning is iterative but local — only checks the apex node |
| Racer runs wide on exit | No explicit exit-node lane |
| Racer fights the lane on lap 1 | `Brain.data.DeviationFromCenter` starts at 0; PID overshoots |
| Same line for all cars | `NodeScalarData` is static and shared |
| No grip-based adjustment | No `AvgGroundStability` input to the profile |

### 3.5 The Hidden Smell: `apexAnchorFactor` Too Large

On a tight hairpin (`LengthStart = 5`), the apex factor of 5 means the apex is
`5 × trackWide` inside — **way past the inside edge**. The iterative solver
clamps to `[-5, 5]`, so on a narrow hairpin the apex lands **on the curb or
beyond**.

### 3.6 Recommendations

**Quick wins**:
1. **Clamp `apexAnchorFactor` to `[-1, 1]`**:
   ```csharp
   apexAnchorFactor = Clamp(apexAnchorFactor - factorDelta, -1f, 1f);
   ```
2. **Add a post-corner exit ramp**:
   ```csharp
   if (followBaseNode > endNode)
   {
       float exitT = (followBaseNode - endNode) / 20f;
       localTargetLane = profileLane * (1f - exitT);  // Ramp to center
   }
   ```
3. **Per-car profile scaling** — multiply the profile by
   `VehicleData.BaseMechanicalGrip / 1.0f` to scale lane offsets by grip.

**Medium-term**:
4. **Recalibrate the profile each lap** — least-squares fit of the Bézier to
   the actual driven line of the fastest racer.
5. **Add elevation awareness** — on crests, shift apex earlier (late apex); on
   dips, shift later (early apex).
6. **Grip-based line choice** — if `GroundGripMultiplier < 0.7`, take a tighter
   line to reduce cornering demand.

---

## Synthesis

### Key Findings

1. **Corner detection** is robust for sweepers but struggles with chicanes,
   esses, and hairpins. No multi-corner planning.
2. **Speed planning** is mathematically sound but tuned poorly. The
   `MapIdealSpeedForDistance` overshoot causes early braking; the disabled
   outward limiter and commented-out elevation adjustment leave easy wins on
   the table.
3. **Line choice** has three real problems: the Bézier solver doesn't respect
   track width on tight corners, the exit phase is undefined, and the profile
   is static and shared.

### Quick Wins (Low Risk, High Impact)

| Fix | File | Effort |
|-----|------|--------|
| Re-enable outward limiter (`1==2` → `true`) | Racer.cs:590 | 1 line |
| Clamp `apexAnchorFactor` to `[-1, 1]` | AutosportRacingSystem.cs:4385 | 1 line |
| Un-comment grip adjustment for elevation | AutosportRacingSystem.cs:4654 | 1 line |
| Add exit ramp after corner endNode | Racer.cs:ApplyCornerLaneProfile | 5 lines |

### Medium-Term Improvements

- Chicane detector (`Angle > 45°` and `Radius < 10m`)
- Curvature continuity (merge nearby corners for esses)
- Recalibrate lane profiles per lap
- Grip-based line adjustments
- `Decision` enum implementation
- Multi-corner planning (predict next corner's radius)

### Long-Term Vision

- Dynamic corner spans (adjust by track type)
- Personality-driven line choice (`Decision` profiles)
- Wet/dry/sand-specific line selection
- Elevation-aware apex shifting (late apex on crests)

---

## File References

| File | Relevant Lines |
|------|----------------|
| `AutosportRacingSystem.cs` | 4208–4424 (corner detection + lane profile build) |
| `AutosportRacingSystem.cs` | 4626–4662 (`GetSpeedForCorner`) |
| `AutosportRacingSystem.cs` | 5019–5038 (`MapIdealSpeedForDistance`) |
| `Racer.cs` | 234–337 (`SteerTrack`, `ApplyCornerLaneProfile`) |
| `Racer.cs` | 340–386 (`SteerApplyCorrections`) |
| `Racer.cs` | 540–627 (`SpeedTrack`) |
| `Racer.cs` | 724–778 (`UpdateCornerInfo`) |
| `DataStructures.cs` | 190–240 (`TrackPoint`, `CornerPoint`, `Corner`) |
