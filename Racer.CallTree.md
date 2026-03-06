# Racer Method Call Tree

```text
Racer.cs : Main racer behavior and control pipeline.
├── RunTimedCore() : Periodic core AI update loop.
│   ├── UpdateFollowTrack() : Updates nearest node, lookaheads, lap progress, and curve target.
│
│   ├── UpdateDynamicBoundingBox() : Refreshes directional car size and slide angle.
│
│   ├── UpdatePercievedGrip() : Rebuilds grip/stability model from traction, terrain, and motion.
│
│   ├── UpdateCornerInfo() : Maintains corner validity and collision/ghost race behavior.
│
│   └── ProcessAI() : Executes steering, speed, rival, traction, and stuck-recovery logic.
│       ├── ProcessTimedAI() : Runs low-frequency checks (corner search, rivals, re-entry, boosts, autofix).
│       │   └── UpdateRivals() : Reassigns nearest racers into rival slots.
│       │
│       ├── UpdateRivalInfo() : Updates rival telemetry and aggression target.
│       │
│       ├── SteerTrack() : Computes desired lane and raw steering angle from lookaheads/corner profile.
│       │   ├── TryBuildSteerTrackContext() : Validates required lookahead references and road width.
│       │   │
│       │   ├── ApplyCornerLaneProfile(...) : Applies precomputed lane offsets through the active corner.
│       │   │
│       │   └── AngleToTrackDir(first, second) : Measures directional change between two track points.
│       │
│       ├── SteerApplyCorrections() : Applies physics-based steering limits and slide/yaw corrections.
│       │
│       ├── SpeedTrack() : Produces target speed from corner, grip, yaw, and lane-outward risk.
│       │   ├── DistToOutside(direction) : Computes distance to outside track edge for limiter logic.
│       │   │
│       │   └── RiskFactorForGrip() : Returns grip risk multiplier used in speed planning.
│       │
│       ├── SpeedToThrottleBrake() : Converts target speed into smoothed throttle/brake commands.
│       │   └── OutOfTrackDistance() : Measures how far vehicle footprint is beyond track width.
│       │
│       ├── SteerTranslateInput() : Smooths steering delta and maps steer degrees to input [-1..1].
│       │
│       ├── UpdateStuckCheck() : Detects prolonged low-speed stuck state and starts recovery.
│       │   └── HandleRecoveryAttemptEscalation() : Escalates failed recovery by nudging velocity trackward.
│       │
│       ├── UpdateStuckRecovery() : Maintains or ends timed stuck-recovery state.
│       │   └── HandleRecoveryAttemptEscalation() : Escalates if repeated recoveries keep failing.
│       │
│       ├── TractionControl() : Adjusts throttle cap from current wheelspin state.
│       │
│       └── ApplyStuckRecoveryOverride() : Forces reverse and centered steering during recovery window.
│
├── ProcessTick() : Per-frame update path outside timed-core cadence.
│   ├── UpdateTickData() : Updates frame kinematics, PID state, and trail sample buffers.
│   │
│   ├── DrawStuff() : Renders optional debug overlays and trail visualizations.
│   │   ├── UpdateFollowLaneTrail() : Stores latest PID/raw lane target trail points.
│   │   │   ├── GetFollowLaneTrailPoint(node, laneOffset) : Converts PID lane offset to world position.
│   │   │   │
│   │   │   └── GetRawFollowLaneTrailPoint(node, laneOffset) : Converts raw lane offset to world position.
│   │   │
│   │   ├── DrawFollowLaneTrail() : Draws smoothed PID lane trail segments.
│   │   │
│   │   ├── DrawRawFollowLaneTrail() : Draws raw lane trail segments.
│   │   │
│   │   └── DrawInputTrails() : Draws throttle/brake history as colored markers.
│   │
│   └── ApplyInputs() : Sends computed controls to vehicle or zeros controls when inactive.
│
├── Launch() : Resets runtime race state and transitions from grid behavior to race behavior.
│
├── Initialize() : Initializes handling/performance baselines and race state fields.
│
├── UpdateRivals() : Public helper to rebuild nearest rival assignments.
│
├── AddDebugText(s) : Adds a deduplicated debug line to on-screen debug text.
│
├── Delete() : Cleans up ped/vehicle ownership and blip state.
│
├── DistToInside(direction) : Computes distance to inside track edge (currently not used by call chain).
│
└── RiskFactorForBrake() : Returns brake risk multiplier (currently not used by call chain).
```
