using GTA;
using GTA.Math;
using GTA.Native;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;

namespace ARS
{
    public enum RacerBaseBehavior
    {
        GridWait, Race, FinishedRace, FinishedStandStill
    }
    public class Racer
    {


        public string Name = "Racer";
        internal string _baseName = "Racer";
        // The name without the PI suffix: what a board or a label should print.
        public string DriverName = "Racer";
        internal string CarModelName = "";
        public Ped Driver;
        public Vehicle Car;
        public Team TeamRole = Team.None;
        public bool ControlledByPlayer = false;
        public RacerBaseBehavior BaseBehavior = RacerBaseBehavior.GridWait;
        public RaceState RCStatus = RaceState.NotInitiated;


        public VehicleControl Control = new VehicleControl();
        public RacerBrain Brain = new RacerBrain();

        // Held apexes ahead: the nearest first, then the ones the car would have to brake earliest for.
        public int NextApexNode = -1;
        public float NextApexRadius = 999f;
        public float NextApexSpeed = 999f;
        public int NextApexNode2 = -1;
        public float NextApexRadius2 = 999f;
        public float NextApexSpeed2 = 999f;
        public int NextApexNode3 = -1;
        public float NextApexRadius3 = 999f;
        public float NextApexSpeed3 = 999f;

        // The corner NextApexNode names, published at the queue commit so the cards never rescan the table.
        CornerPoint _nextApexCorner;
        const int HeldApexCount = 3;



        public VehicleState VehicleData = new VehicleState();
        public HandlingData Handling = new HandlingData();
        public float GroundGripMultiplier = 1f;
        Vector3 _lastSpeed;

        
        int _lastGsCheck = 0;
        bool _gripLogged = false;
        // -1 until Launch captures the at-rest ride height; the height test is skipped while it has no value.
        float _restHeightAboveGround = -1f;

        // Racer progress along the route.
        public TrackPoint CurrentTrackPoint = new TrackPoint();
        public float AlongTrackSpeed = 0f;
        public int RaceProgress = 0;
        // The two nodes the live crest-grip system is currently reading on the route: the
        // 0.5s-ahead point (centre of the Menger-curvature triangle) and the 1.0s-ahead point.
        // Set by CrestGripSpeedFactor's caller and read by the track-analysis visual so the
        // driver can see which sample the no-fly gate is checking this tick. (The previous name
        // LiveCornerCrestNode was a holdover from a design that read the next corner's apex;
        // the new design samples relative to the car, so the variable now holds the 1.0s-ahead
        // route point instead.)
        public int LiveRouteCrestNode = -1;
        public int LiveRouteCrestFarNode = -1;
        // The two lookahead nodes RouteRadiusSampled measured this tick, for the route-radius debug lines.
        public int LiveRouteRadiusNearNode = -1;
        public int LiveRouteRadiusMidNode = -1;
        public int LiveRouteRadiusFarNode = -1;
        // The radius the last sample produced, for the readout over the car.
        public float LiveRouteRadius = -1f;

        // Time-based route references used by steering and speed calculations.
        public enum LookAhead { SteerRef, QuarterSec, HalfSec, ThreeQuarterSec, OneSec, OneHalfSec, TwoSec };
        public Dictionary<LookAhead, TrackPoint> LookAheads = new Dictionary<LookAhead, TrackPoint>();


        public List<TimeSpan> LapTimes = new List<TimeSpan>();
        public int LapStartTime = 0;
        public int Lap = 0;
        public int RacePosition = 0;
        // A car whose engine is dead cannot rejoin. It is parked on the shoulder and never races again this race.
        public bool IsDNF = false;
        // Frozen finish rank, assigned once when the racer crosses the line; 0 = still racing.
        public int FinalPosition = 0;
        public bool CanRegisterNewLap = false;
        int _previousNode = -1;
        public bool FinishedPointToPoint = false;


        int _halfSecondTick = 0;
        int _oneSecondTick = 0;
        int _phaseOffsetMs = 0;
        // Rival records feed the rival walls, so their freshness is a safety property, not a convenience.
        const int RivalRefreshMs = 250;
        int _rivalTick = 0;
        int _pressureTick = 0;
        int _apexUpdateTick = 0;
        int _rivalInfoTick = 0;
        int _lastCoreTick = 100;
        int TimeSince_lastCoreTick => (int)ARS.Clamp(Game.GameTime - _lastCoreTick, 1, 9999);


        List<TrackPoint> _trackPositionScratch = new List<TrackPoint>(13);
        const int StuckRecoveryTimeMs = 6000;
        const int StuckRecoveryCooldownMs = 2000;
        const int RecoveryArmDelayMs = 15000;
        StuckPhase _stuckPhase = StuckPhase.None;
        int _stuckPhaseEndTime = 0;
        int _stuckRecoveryStartTime = 0;
        int _stuckStationarySince = 0;
        int _stuckOffTrackSince = 0;
        int _stuckAgainSince = 0;
        int _stuckExitSince = 0;
        int _stuckArmAllowedTime = 0;
        int _stuckRecoveryCooldownEndTime = 0;
        Vector3 _stuckMoveSamplePosition = Vector3.Zero;
        int _stuckMoveSampleTime = 0;
        // Backing off is a short, straight phase. Drive is not a pedal override: it is a speed intention, so the plan,
        // the off-track cap and the lane law all keep owning the car while it rejoins.
        const int StuckReverseMs = 1000;
        const int RecoveryTriggerMs = 2000;
        const float RecoveryTriggerMph = 2f;
        const float RecoveryPlanGapMph = 1f;
        const float RecoveryDriveMph = 20f;
        const float RecoveryCoastBandMph = 5f;
        const float RecoveryExitMph = 4f;
        const int RecoveryExitHoldMs = 500;
        const int StuckMoveSampleMs = 1000;
        const float StuckMoveMeters = 0.5f;
        // DNF cars are parked on the shoulder, spaced along the route from the start line and alternating sides.
        const int DNFSlotSpacingMeters = 5;
        const float DNFSlotOffsetMeters = 1f;
        const float DNFUnderTrackMeters = 5f;

        // Throttle kick (limiter-saturation rotation): the pursuit's ask sitting past the steering limiter's
        // granted ceiling with no slide in progress, held through a run of consecutive core calls, plays a
        // full-throttle window — TCS pinned off for it, the slide blend standing down for the rotation the
        // ask's own side wants. Gated to RWD and a forward roll, and to the gear gate: the computed burst
        // thrust per rear wheel must beat the grip budget by the margin. The fold is the window cap or a
        // slide the burst has delivered past its bound (either side: the machinery resumes to catch it).
        const float KickMaxDriveBiasFront = 0.01f;
        const float KickPedalLevel = 1f;
        const int KickArmSaturateMs = 1000;
        const int KickWindowMs = 1250;
        const int KickCooldownMs = 1500;
        const float KickFoldSlideDegrees = 30f;
        const float KickGripMargin = 1.2f;
        const float KickWheelLoadShare = 0.25f;
        bool _kickPlaying = false;
        int _kickPlayingSince = 0;
        int _kickWindowEnd = 0;
        int _kickArmSince = 0;
        int _kickCooldownUntil = 0;
        int _kickSteerSign = 0;
        // The raw pursuit ask, captured before the crossing scale: the kick's deficit is the pursuit's own
        // demand against the lock, not what the scale and the limiter deliver of it.
        float _kickAskDegrees = 0f;
        public float RouteLookaheadSizeSeconds = 2.0f;
        // The steer reference is a preview *time*: a fixed distance raises the loop's natural frequency with speed.
        const float SteerPreviewSeconds = 0.5f;
        const float SteerLookaheadMinMeters = 8f;
        const float SteerLookaheadMaxMeters = 30f;
        // The Gs-aware preview: how much of the car's projected lateral motion leads the lane error.
        const float GsPreviewBlend = 0.5f;


        float _avoidLeftWall = 0f;
        float _avoidRightWall = 0f;
        float _wallRoadWide = 0f;
        // Read by the Show Rivalry debug lines: the corridor the car is allowed to aim into.
        public float AvoidLeftWall => _avoidLeftWall;
        public float AvoidRightWall => _avoidRightWall;
        public float WallRoadWide => _wallRoadWide;
        bool _leftWallHeld;
        bool _rightWallHeld;
        // True while a rival is what keeps that wall inside the track bound (as opposed to it relaxing back out).
        public bool LeftWallHeld => _leftWallHeld;
        public bool RightWallHeld => _rightWallHeld;
        int _activeRivalWallCount = 0;
        bool _avoidWallsInitialized = false;
        // The carried lane and the sources' request for it are separate: the mover carries one onto the other with
        // lateral inertia, so a request that is valid for half a second cannot step the lane.
        float _carriedLane = 0f;
        // Set by the mover when the aim has overshot the demand on the same side of the car: the car stops following it and it
        // returns at twice the acceleration until it is back on the segment.
        bool _laneOvershoot;


        // The bearing before the de-glitch and the distance to the aim point: together they say whether the flip is the aim moving
        // or the same aim seen from a very short baseline.
        float _pursuitRawBearingDeg;
        float _pursuitAimDistanceM;
        // Lateral offset of the aim from the car's own axis, raw and smoothed: the step size in METRES, which is what tells a node
        // step (about a metre) from the lookahead index jumping (metres).
        float _pursuitAimLateralM;
        // The apex blend only ever closes: the latched phase of the corner currently being approached, and which corner it belongs to.
        int _linePhaseNode = -1;
        float _linePhaseLatched;
        // The apex this line last seeded its phase for, and when. A corner the picker drops and re-adopts a tick later must not be re-seeded:
        // that would push the demand back outward mid-manoeuvre.
        int _linePhaseSeededNode = -1;
        int _linePhaseSeededAt = 0;
        const int LinePhaseReseedBlockMs = 5000;
        // This car's own pull-in for the outside line, rolled once per corner and held until the car passes it.
        int _lineBiasNode = -1;
        float _lineBiasMeters;
        // The corner that owns the apex line until its apex is passed. NearestApexAhead keeps handing this back while it is the
        // nearest, which is what stops two close apexes from taking the line off each other mid-manoeuvre.
        int _lineCornerNode = -1;
        // The speed the car carried when it entered this corner's out-in window, frozen for the whole manoeuvre: the clock then says the
        // same thing every lap, and braking, traffic or a throttle cut can no longer move where the blend opens.
        int _lineClockNode = -1;
        // The clock follows the car's own braking, a little: a car that is shedding speed will reach the apex later than the cached entry
        // speed says, so the line is allowed to slow with it. 0.1 mph per 0.1 g, and never more than a mile an hour either way.
        const float LineClockBrakeMphPerG = 2f;
        const float LineClockBrakeCapMph = 10f;
        float _lineAlongPrev;
        float _lineDecelG;
        float _lineClockMps;
        // One log a second at most, so a rule that fires rarely does not flood the file when it does.
        int _laneCrossLoggedAt;
        // The direction the latched phase belongs to: a second apex in the same bend keeps the latch, a bend the other way starts
        // again from the outside.
        float _linePhaseDir;
        // The lean's approach speed in m/s, set from the sampled curvature: near zero at the 500 m gate, at most CurveLeanApproachMps.
        float _leanApproachMps;
        float _laneRequest = 0f;
        LaneReason _laneRequestReason = LaneReason.OwnLane;
        float _laneRequestPrev = 0f;
        float _laneRate = 0f;
        float _laneMoverLoggedAt = 0f;
        // The apex whose demand has already crossed to the inside of the car: from then on it may hold but never pull
        // outward. Keyed by node rather than reset, so a new corner starts with the approach it is owed.
        int _apexLineInsideNode = -1;
        float _apexLineInsideCornerDir = 0f;
        bool _apexInsideNow = false;
        int _laneGuardLoggedAt = 0;
        int _cornerLineLoggedAt = 0;
        float _cornerRefusalLoggedAt = 0f;
        bool _laneSnap = false;
        bool _laneOverride = false;
        public float CarriedLane { get { return _carriedLane; } }
        // The corner that owns the apex line right now: the debug diagonal is drawn to its node, so a swap of identity is visible.
        public int LineCornerNode { get { return _lineCornerNode; } }
        public LaneReason LaneRequestReason { get { return _laneRequestReason; } }
        public float LaneRequest { get { return _laneRequest; } }
        public float LaneRate { get { return _laneRate; } }

        float _cornerSpd = 999f;

        int _divebombApexNode = -1;
        int _defendApexNode = -1;

        // Sagitta below this share of the chord means the sample is a straight: at 121 m it is 0.24 m, which needs a
        // radius past 7.5 km. The tightest genuine corner on the nine shipped tracks sits at 0.006, three times it.
        const float RouteStraightSagittaShare = 0.002f;

        // How much brake the route's curvature is allowed to ask for. A curvature-driven target is a step, not a
        // ramp: braking on it at full authority is what reads as a slam. A corner owns its own ramp and is exempt.
        const float RouteBrakeCap = 0.2f;

        // The most throttle the development Gs floor will hold: full pedal at the inside edge, fading to nothing at the centre.
        // 1.00 is deliberate - at the edge the car is not allowed to lift at all, which is the extreme the owner asked to feel.
        const float GsLaneThrottleMaxFloor = 1f;

        // Two input units per second while outside, down to a floor that leaves two thirds of the tyre for steering.
        const float ProjectionInputPerSecond = 2f;
        const float ProjectionBrakeFloor = 0.5f;
        // Off the line the car does not need to stop, only to be slow enough to turn back. The projection escalates down to the
        // speed the curvature asks for minus this much, and never below the absolute floor: a 35 mph corner asks 35-40 = -5, so
        // the car is taken to exactly 20 and no slower - which is also where the widening gate turns itself off.
        const float OffTrackSpeedSlackMph = 40f;
        const float OffTrackSpeedFloorMph = 20f;
        // A node deflection below this is a straight, which has no outside to judge.
        const float OffshootMinRouteAngleDeg = 2f;
        // Line discipline belongs to speed: below this the problem is traction, and a cap would hold an off-edge car
        // at zero, because the projection of a stationary car is the car.
        const float OffshootMinSpeedMph = 20f;
        // Throttle saturates at one metre per second of deficit: the pedal is there to reach the plan, not to approach it
        // gently. Braking keeps its three, so the pair is 1:3 - full gas the moment the car is a hair under its speed,
        // full brake only when it is three metres a second over.
        const float FullPedalSpeedErrorMps = 1f;
        // Braking is the softer side: it takes this many times the speed error to command full brake.
        const float BrakeErrorMultiplier = 3f;

        // Applied pedal input sampled every half metre of travel, for the debug trail: the cap holds 40 m.
        readonly List<InputTrailSample> _inputTrail = new List<InputTrailSample>();
        const float InputTrailSampleSpacing = 0.5f;
        const int InputTrailMaxSamples = 80;
        // The trail draws chevrons pitched by the pedal — nose down for throttle, nose up for brake, 45° at full —
        // so the pitch is the gauge and the marker's bulk keeps it visible where a hairline was not. A site is
        // chosen as its sample is taken, never by a stride over the list: a list-anchored stride re-forms every
        // time a sample arrives or the oldest drops, and the trail strobes at frame rate.
        const float InputTrailChevronSpacing = 1f;
        const float InputTrailChevronGroundClearance = 1f;
        const float InputTrailChevronHeightGain = 0.5f;
        const float InputTrailChevronSize = 2f;
        const float InputTrailChevronPitch = 45f;
        const int InputTrailFadeCount = 5;
        float _inputTrailChevronDistance = 0f;


        // Yaw damper term in degrees: read by the steer sum below.
        float _damperTermDeg = 0f;
        // How much of the sliding countersteer blend the current slide has earned, 0 to 1; read by the limiter
        // and the slew within the same frame.
        float _slidePriority = 0f;
        float _steerPursuitDeg = 0f;
        float _steerAimBearingDegrees = 0f;
        float _steerAimCurvature = 0f;
        // The limiter's before-and-after: the ask it was handed and the granted ceiling it enforced, one core
        // tick old when the kick card reads them, so the over-the-limiter test is the delivery's own.
        float _steerPreLimitDegrees = 0f;
        // The ceiling actually in force after every limit, and the wheel angle granted, both for the readout.
        float _finalCeilingDeg = 0f;
        // The chase that is actually applied: follows clampTarget at one rate in both directions, and is born at whatever the
        // target is the first time the guard can speak. NaN until then.
        float _chasingClampDeg = float.NaN;
        // What the gate is aiming at this tick, kept only so the readout can say whether the applied clamp has got there yet.
        float _clampTargetDeg = float.NaN;
        // The share of the request the cut is leaving this tick, 1 when it is not cutting. Written here so the readout can show it.
        float _slipFactor = 1f;

        // Opening is LINEAR by decision: a fixed 72 deg/s, which covers the whole travel from 2 deg to the lock (36 deg) in half a
        // second and any shorter gap proportionally less. Exponential opening was tried and rejected - the time varied with the gap,
        // so the same manoeuvre looked different depending on how far the clamp had fallen. The old 4 deg/s needed 4.5 s for the same
        // travel, which read as 'the clamp never comes back'.
        const float SlipOpenDegPerSec = 72f;
        // Closing is exponential in the gap, not in the severity: the rate is the distance to the target times this gain, so the
        // time to arrive is a property of the system and not of how big the drop happened to be. Calibration (owner's rule): a
        // clamp sitting at 500% of its target - a gap of four times the target - arrives in about 0.2 s. There is NO ceiling on the
        // rate, so the gain has to stay well under the tick rate: at 15/s the gap keeps half of itself per 30 Hz tick, and any gain
        // above 30/s would overshoot the target and be clamped back in a single tick, which is a snap, not a law.
        const float SlipClosePerSecond = 15f;
        // And a ceiling on the close: without one, a target that collapses 36 deg is covered at 500 deg/s, which is a teleport rather
        // than a law. 90 deg/s is 3 deg a tick: it binds above a 6 deg gap, so a big drop is visibly stepped rather than instantaneous,
        // and everything short of that keeps the exponential's own curve.
        const float SlipCloseMaxDegPerSec = 90f;
        // One measurement of the front axle per tick, filled by MeasureFrontSlip and read by the limiter and the readout.
        SlipState _slip;
        // Below this command there is no inside of the corner, so the pair is averaged instead of picked.
        const float FrontSlipNeutralSteerDeg = 0.5f;
        // The theoretical source's floors: below this the kinematic radius is noise, and g is g.
        const float FrontSlipTheoryMinSpeedMps = 3f;
        // Below this command the radius is a straight line and its inverse is noise.
        const float FrontSlipTheoryMinCommandDeg = 0.05f;
        const float FrontSlipGravity = 9.81f;
        float _steerGrantedCeiling = 0f;
        // Last off-track projection cap, kept for the pedal bar's override sphere.
        float _projectionOutsideMeters = 0f;
        bool _projectionWidening = false;
        float _projectionOutsideSeconds = 0f;
        // The colour the trajectory is drawn in, decided where the trajectory is measured rather than in the drawing code:
        // the floor, the edge and the widening all live here, and DrawDebug only has to paint what it is handed.
        Color _projectionPathColor = Color.Blue;
        // What the inside floor added to the pedal this tick, as a share of full throttle: the difference between what the plan,
        // the caps and the slew had composed and what the car was finally given. Read by the development subtitle.
        float _throttleFloorGain = 0f;
        // Whether the lane floor was holding any throttle on the last tick it ran. Read from the brake sample as the car crosses a
        // corner entrance, so the report can say whether the floor was live when the corner began.
        bool _gsFloorActiveThisTick = false;
        // True only on the tick the projection actually cut the throttle, which is what the red lines report.
        bool _offtrackThrottleHeld = false;
        int _offshootLogAt = 0;

        // Brake learning: the factor multiplies the decel the plan assumes it has - `brakingAbility * g * factor` - so 1.0 is the
        // grip-limited maximum. Driven at 1.0 the cars brake exactly as the plan asks and at 1.5 they overshoot everything, which
        // says the model is calibrated and the factor has nothing global to learn. Held at 1.0 with no draw while the learner is
        // measured: a seeded range would make two runs of the same corner incomparable, and the exercise is to read the excess
        // speed, not to watch the seed drift.
        const float BrakeFactorSeed = 1f;
        // The radius under which a corner counts as tight, i.e. where the outside line is worth having.
        const float CornerTightRadius = 50f;
        // Arrival is extrapolated from the live forward Gs at this discount: holding a G flat across the window
        // over-predicts, because acceleration tapers with speed. Tuned by driving.
        const float RequirementExtrapolationScale = 0.9f;
        const float PositionExcessMph = 10f;
        const float BrakeExcessMph = 25f;
        // Braking is judged from the distance it actually needs, plus this much road for the plan to take hold.
        const float BrakeHorizonMargin = 40f;
        // The learner writes. Only the metres verdict moves the factor: a slide and a jump are the speed system's errors, reported in the
        // notice and never charged here.
        const bool BrakeLearningApplies = true;
        readonly Dictionary<int, CornerContext> _cornerContexts = new Dictionary<int, CornerContext>();
        int _cornersRevisionSeeded = -1;
        // Apex nodes the scan has judged worth nothing this approach (wide and unarmed inside their window). The
        // picker looks past them, so a fast corner the car passes flat no longer makes the tight one behind it
        // promote inside its own window and lose the approach.
        readonly HashSet<int> _skippedApexes = new HashSet<int>();
        float _brakeSampleMaxSlideDeg = 0f;  // largest slip seen between this apex's entrance and the apex itself
        int _brakeSampleApexNode = -1;
        bool _brakeSampleWasAirborne = false;
        // Metres from the corner entrance to the node where the car dropped below the corner speed plus the full-brake error, so the
        // sign reads as a position on the road: negative with road still to run before the entrance, positive once past it.
        float _brakeSampleEntryMetres = 0f;
        bool _brakeSampleEntryMeasured = false;
        bool _brakeSampleSawAboveApex = false;
        // The corner speed frozen when the sample was opened. NextApexSpeed is rebuilt every tick from grip and a smoothed radius,
        // and grip carries the per-tick downforce bonus, so a threshold read live jitters by about a metre a second and decides the
        // crossing on noise instead of on the car.
        float _brakeSampleApexSpeed = 0f;
        int _brakeSampleBelowTicks = 0;
        bool _brakeSampleEntryHasCandidate = false;
        // The floor state read as the car crossed this corner's entrance.
        bool _brakeSampleFloorAtEntrance = false;
        bool _brakeSampleEntranceCrossed = false;
        // Room left to the edge at the closest approach to this corner's apex. Two floats, nothing else.
        const float BrakeApexNearMetres = 50f;
        float _brakeSampleApexGap = 0f;
        int _brakeLastApexAnnounced = -1;
        float _brakeSampleApexClosest = float.MaxValue;
        int _brakeOverNextSampleAt = 0;
        // The commit is deferred past the exit, because a car that entered too hot slides on the way out and that is
        // the same verdict as sliding in. The sample is handed over when the apex is passed and then spends
        // BrakeCommitDelaySeconds gathering the exit's slide before it is allowed to teach anything.
        int _brakeCommitApexNode = -1;
        int _brakeCommitAtGameTime = 0;
        float _brakeCommitMaxSlideDeg = 0f;
        bool _brakeCommitWasAirborne = false;
        float _brakeCommitEntryMetres = 0f;
        bool _brakeCommitEntryMeasured = false;
        bool _brakeCommitSawAboveApex = false;
        bool _brakeCommitFloorAtEntrance = false;
        bool _brakeCommitEntranceCrossed = false;
        // A slide only counts as the braking's fault while the car is actually being asked to brake hard: a rear-engined car loses its
        // rear at full pedal, which is a real effect of the demand and not of the entry speed. Below this share of brake there is no
        // braking to blame, and the slide belongs to the speed system.
        const float BrakeSkidBrakeShare = 0.25f;
        const float BrakeSkidFactorStep = 0.1f;      // a skid under that demand costs this much factor, flat
        const float BrakeSkidPeakMultiple = 2.5f;   // skid gate sits at the peak x this, where grip hits its floor
        const float BrakeCommitDelaySeconds = 0.75f; // wait this long after the apex before the verdict lands
        const float BrakeMinFactor = 0.5f;       // learned factor range floor
        // The learner aims at a *duration*, not a magnitude: one second spent over the ramp. Time is the unit that makes the two
        // directions symmetric - a short corner and a long one are both given the same second - and it is what a magnitude could
        // never deliver. A car braking at the limit is always fractionally over its own theory, so a speed or a ratio scored "over"
        // on every corner and the factor could only ever fall: the learner could never risk more. Under the target the entry may be
        // braver, over it the entry is relaxed.
        // The verdict aims at a position: the car should reach the corner speed exactly at the entrance, so the mark wants to land on
        // zero metres. This is the factor moved per metre of error - three metres costs 0.06, tuned by driving - capped per event so one
        // bad corner cannot spend the whole range.
        const float BrakeEntryFactorPerMetre = 0.02f;
        const float BrakeEntryMaxStepPerEvent = 0.25f;
        // How far before the entrance the floor state is watched, in seconds of travel.
        const float BrakeFloorLatchSeconds = 1f;
        // How much of the law's own brake error the ramp is allowed before the clock starts. At 1.0 a stop that is going well still
        // counted time over - the car trails the falling ramp by more than the law's steady error, and every measurement is a sample
        // of a moving target - so the floor is raised to 1.5: four and a half metres a second, about ten miles an hour, above the
        // ramp before a single tenth of a second is counted.
        const float BrakeOverAllowanceShare = 1.5f;
        const int BrakeOverSampleMs = 100;
        const float BrakeMaxFactor = 1.5f;   // ceiling above the 1.0 seed so a learned cut can be earned back
        public float BrakeFactorForApex(int apexNode) => ARS.BrakeLearning && TryGetCornerContext(apexNode, out CornerContext context) ? context.BrakeFactor : BrakeFactorSeed;
        float _divebombBrakeBonus; // temp brake boost while diving, whole hundredths 2-8 drawn per dive, never committed to learning

        // The car's own view of a corner: keyed by node and rebuilt only when the apex table changes, so a learned
        // factor survives a mid-session rebuild and a corner that no longer exists cannot linger.
        bool TryGetCornerContext(int apexNode, out CornerContext context)
        {
            SyncCornerContexts();
            return _cornerContexts.TryGetValue(apexNode, out context);
        }

        // Gates the braking plan, not the held queue: the queue must keep the real next corners or Brain.Corner,
        // NextApexSpeed and the low-speed invalidation all read a corner that is a lap away.
        bool CornerRequiresBraking(int apexNode) => apexNode >= 0 && TryGetCornerContext(apexNode, out CornerContext context) && context.RequiresBraking;

        CornerContext CornerContextForWrite(int apexNode)
        {
            if (!TryGetCornerContext(apexNode, out CornerContext context))
            {
                context = new CornerContext { BrakeFactor = SeededBrakeFactor() };
                _cornerContexts[apexNode] = context;
            }
            return context;
        }

        void SyncCornerContexts()
        {
            if (_cornersRevisionSeeded == ARS.CornersRevision) return;
            _cornersRevisionSeeded = ARS.CornersRevision;
            foreach (CornerPoint corner in ARS.Corners)
                if (!_cornerContexts.ContainsKey(corner.Node))
                    _cornerContexts[corner.Node] = new CornerContext { Point = corner, BrakeFactor = SeededBrakeFactor() };

            List<int> stale = new List<int>();
            foreach (int node in _cornerContexts.Keys)
                if (!ARS.Corners.Exists(corner => corner.Node == node)) stale.Add(node);
            foreach (int node in stale) _cornerContexts.Remove(node);
        }

        // The racer's own bias plus one independent draw per corner. The bias repeats across all of that racer's
        // corners where the per-corner draw averages out, so the bias is the half that moves a lap. The +1 makes
        // each draw symmetric: GetRandomInt's max is exclusive.
        float SeededBrakeFactor() => BrakeFactorSeed;

        // While diving, a temp bonus is appended to the learned factor; the stored factor itself never changes.
        public float EffectiveBrakeFactor(int apexNode)
        {
            return ActiveManeuver.Type == ManeuverType.DiveBomb
                ? BrakeFactorForApex(apexNode) + _divebombBrakeBonus
                : BrakeFactorForApex(apexNode);
        }



        bool _isPassengerized = false;
        const float RivalSearchRangeMeters = 200f;

        // Feature gate for nitrous (ARS.NitrousEnabled). Both sides burn the same multiplier; the AI's share of it
        // is the Race Rules setting ARS.AiNitroPct, which ships at 100 so the two are equal until tuned.
        const float NitrousPowerMultiplier = 5f;
        // The player's own factor. Measured on track: their 1.0 and the rivals' 2.5 produced the same shove, because the player's
        // burn rides GTA's own nitrous force (1 + 3.0 x powerMod) while the rivals only get this one. The pair is doubled here
        // instead of bridged: 2 for the player, 5 for a rival at AiNitroPct 100, which keeps the observed 1:2.5.
        const float PlayerNitrousPowerMultiplier = 1f;
        const float NitrousMaxSteerDegrees = 5f;
        const float NitrousMinThrottle = 0.9f;
        // The card may not fire inside the first six seconds of the race: at the launch the field is still bunched, so a
        // bottle spent there buys a gap off the line rather than a pass.
        const int NitrousArmDelayMs = 6000;
        const float NitrousCornerLookaheadSeconds = 8f;
        const float NitrousLonelySpeedFraction = 0.5f;
        const float NitrousLonelyMinApexDistance = 500f;
        const float NitrousDefenseReachSeconds = 4f;
        const float NitrousNearbyRivalDistance = 80f;
        const float NitrousFinishExtraDistance = 100f;
        const float ChillRivalCrowdDistance = 40f;
        const int ChillCrowdCount = 5;
        const float ChillReleaseRadiusMeters = 50f;

        // A card is a decision, and a decision does not change at the core rate. Every release is looser than its
        // arm, no card may end before it has had time to read, and no card may re-arm straight after ending.
        const float DivebombReleaseGapMeters = 40f;
        // How late the chaser may become before the threat is gone: the arm admitted him because he arrived no later, so
        // the release must let go once he clearly will not.
        const float DefendThreatMarginSeconds = 0.5f;
        const float DivebombReleaseAngleDegrees = 25f;
        const int CardMinDwellMs = 1000;
        const int CardCooldownMs = 1000;
        public const float ChillStandoffDistance = 20f;
        const float ChillThrottleCap = 0.5f;
        // Anti-collision lift: full gas at this bumper gap, this much at contact, linear between.
        const float RivalLiftMeters = 3f;
        const float RivalLiftFloor = 0.1f;
        const float ChillStandoffEscapeMargin = 2f;
        const float ChillMinSpeedMph = 30f;

        // Counts cars ahead on race progress only; cars behind don't crowd us.
        int RivalsWithinDistance(float distance)
        {
            if (ARS.NoCollision) return 0;
            return ARS.Racers.Count(r => r.Car.Handle != Car.Handle && r.RacePosition < RacePosition && r.Car.Position.DistanceTo(Car.Position) <= distance);
        }
        const int NitrousDurationMs = 3000;
        const float RocketBoostMinimumCurveRadius = 600f;
        const float SideBySideAssistRangeExtra = 3f;
        const float SideBySideFullAssistExtraGap = 1f;
        const float SideBySideMinimumAssist = 0.1f;
        const string NitrousPtfxAsset = "veh_xs_vehicle_mods";
        const ulong CheatPowerIncreaseHash = 0xB59E4BD37AE292DB;
        const ulong MaxDriveGearHash = 0x24910C3D66BA770D;
        const ulong CurrentDriveGearHash = 0x56185A25D45A0DCD;
        const ulong FullyChargeNitrousHash = 0x1A2BCC8C636F9226;
        const ulong OverrideNitrousLevelHash = 0xC8E9B6B71B8E660D;
        const float PowerCounterMax = 1.5f;
        const float PowerCounterMinCommanded = 0.1f;
        const float PowerCounterMinApplied = 0.05f;
        const float TopSpeedCounterOnsetScale = 1f;
        const float TopSpeedCounterBand = 15f;
        const float TopSpeedCounterMax = 1.35f;



        public Maneuver ActiveManeuver = new Maneuver();
        readonly int[] _cardCooldownUntil = new int[8];

        bool CardCooled(ManeuverType type) => Game.GameTime >= _cardCooldownUntil[(int)type];
        bool CardMayEnd => Game.GameTime - ActiveManeuver.LastEnabled >= CardMinDwellMs;
        void EndCard()
        {
            // Yield is reactive by nature: it lets a faster car by and has nothing to protect against re-arming.
            if (ActiveManeuver.Type != ManeuverType.Yield) _cardCooldownUntil[(int)ActiveManeuver.Type] = Game.GameTime + CardCooldownMs;
            ActiveManeuver.Type = ManeuverType.None;
            ActiveManeuver.Target = null;
        }

        // Read by the world card label: the dive's own brake bonus is the only number that card carries.
        public float DivebombBrakeBonus => _divebombBrakeBonus;
        int _nitrousActiveUntil = 0;
        int _nitrousGsLoggedAt = 0;
        int _nitrousLapUsed = -1;
        int _nitroRefusalLoggedAt = 0;
        float _powerCounter = 1f;
        float _overspeedPowerLevel = 1f;
        float _countersteerAssistDeg = 0f;
        int _cornerLiftApex = -1;
        int _defendEntranceNode = -1;
        float _avoidHoldLane = 0f;
        bool _avoidHoldActive = false;
        // The pass lane the car committed to, and the rival it committed to it for. Held while the bodies overlap.
        float _passLaneValue;
        bool _passLaneLatched;
        Racer _passLaneTarget;
        int _avoidHoldUntil = 0;
        float _cornerLiftMps = 0f;
        float _appliedThrottleLastFrame = 1f;
        int _powerCounterLoggedAt = 0;


        public float Aggression = 50f;
        public float Pressure = 0f;
        const float PressureRange = 100f;
        const float PressureProximityRange = 100f;
        const float PressureRisePerSecond = 2f;
        const float PressureFallPerSecond = 30f;

        public Racer(Vehicle RacerCar, Ped RacerPed)
        {
            Car = RacerCar;
            Driver = RacerPed;
            try { Name = RacerCar.FriendlyName; } catch (Exception) { Name = "Racer"; }
            if (Name == "NULL" || Name == null) { try { Name = Car.DisplayName.ToString()[0].ToString().ToUpper() + Car.DisplayName.ToString().Substring(1).ToLowerInvariant(); } catch (Exception) { Name = "Racer"; } }
            CarModelName = Name;

            if (Driver.IsPlayer) ControlledByPlayer = true;
            _halfSecondTick = Game.GameTime + (ARS.GetRandomInt(10, 50));
            _phaseOffsetMs = ARS.GetRandomInt(0, 500);
            // The confidence draw is gone with the fixed seed: the learner now starts every corner from the same number.
            _pressureTick = Game.GameTime;
            VehicleData.ModelDimensions = Car.Model.GetDimensions();

            if (!ControlledByPlayer)
            {

                try { Driver.BlockPermanentEvents = true; } catch (Exception) { }
                try { Driver.AlwaysKeepTask = true; } catch (Exception) { }
                Function.Call(GTA.Native.Hash.SET_DRIVER_ABILITY, Driver, 0f);
                Function.Call(GTA.Native.Hash.SET_DRIVER_AGGRESSIVENESS, Driver, 0f);

                AutofixMode autofix = ARS.Autofix;
                if (autofix == AutofixMode.Invincible)
                {
                    Function.Call(GTA.Native.Hash.SET_ENTITY_PROOFS, Car, true, true, true, true, true, true, true, true);
                    Function.Call(GTA.Native.Hash.SET_ENTITY_PROOFS, Driver, true, true, true, true, true, true, true, true);

                    Car.IsInvincible = true;
                    Car.IsCollisionProof = true;
                    Car.IsOnlyDamagedByPlayer = true;
                    Function.Call(GTA.Native.Hash.SET_VEHICLE_STRONG, Car, true);
                    Function.Call(GTA.Native.Hash.SET_VEHICLE_HAS_STRONG_AXLES, Car, true);
                    try { Car.EngineCanDegrade = false; } catch (Exception) { }
                }
                else if (autofix == AutofixMode.AutoRepair)
                {
                    Function.Call(GTA.Native.Hash.SET_VEHICLE_STRONG, Car, true);
                    Function.Call(GTA.Native.Hash.SET_VEHICLE_HAS_STRONG_AXLES, Car, true);
                    try { Car.EngineCanDegrade = false; } catch (Exception) { }
                }
                else
                {
                    Car.IsInvincible = false;
                    Car.IsCollisionProof = false;
                }

                Car.EngineRunning = true;
                Driver.SetIntoVehicle(Car, VehicleSeat.Driver);
                VehicleMemory.SetSteerAngle(Car, 0.5f);
                VehicleMemory.SetThrottle(Car, 0f);
                VehicleMemory.SetBrakes(Car, 0f);

                try { Car.IsRadioEnabled = false; } catch (Exception) { }
            }

            try { Car.IsPersistent = true; } catch (Exception) { }
            if (!Driver.IsPlayer)
            {
                try
                {
                    if (Car.CurrentBlip == null || Car.CurrentBlip.Exists() == false)
                    {
                        Car.AddBlip();
                        Car.CurrentBlip.Color = BlipColor.Blue;
                        Car.CurrentBlip.Scale = 0.75f;
                        Function.Call(Hash._SET_BLIP_SHOW_HEADING_INDICATOR, Car.CurrentBlip, true);
                        Function.Call(Hash._0x2B6D467DAB714E8D, Car.CurrentBlip, true);
                        Car.CurrentBlip.Name = Name;
                    }
                }
                catch (Exception) {  }
            }

            // AI racers must survive a race; the player's own ped must not. AddPlayerToGrid passes the player as Driver, and
            // this block ran for every racer, so a race left the player bullet, fire, explosion, melee and water proof at
            // 1000 HP with critical hits disabled - for good.
            if (!Driver.IsPlayer)
            {
                Function.Call(GTA.Native.Hash._0x0DC7CABAB1E9B67E, Car, true, 1);
                Function.Call(GTA.Native.Hash._0x0DC7CABAB1E9B67E, Driver, true, 1);
                Function.Call(GTA.Native.Hash.SET_ENTITY_PROOFS, Driver, true, true, true, false, true, true, 1, true);

                Driver.MaxHealth = 1000;
                Driver.Health = 1000;
                Driver.CanSufferCriticalHits = false;
            }

            if (Car.ClassType == VehicleClass.Emergency) TeamRole = Team.Cop;

        }
        public void Initialize()
        {
            Handling.Downforce = VehicleMemory.GetDownforce(Car);

            Handling.LateralTractionCurve = ARS.RadToDeg(VehicleMemory.GetLateralTraction(Car));
            if (Handling.LateralTractionCurve < 1 || Handling.LateralTractionCurve > 100) Handling.LateralTractionCurve = 22;
            ARS.Log(ARS.LogImportance.Info, "TRlat for " + Car.DisplayName + ":" + Handling.LateralTractionCurve + "º");

            Handling.BrakingAbility = Car.MaxBraking;
            Handling.EstimatedTopSpeed = ARS.EngineTopSpeed(Car);
            // Two numbers, one line: the geared ceiling the vehicle flavour reports (m/s, before drag - see EngineTopSpeed)
            // and the drag-limited estimate the mod now keeps, in mph. The second is what every threshold built on it uses.
            ARS.Log(ARS.LogImportance.Info, "Top speed " + Car.DisplayName + ": geared "
                + Function.Call<float>(Hash._0x53AF99BAA671CA47, Car).ToString("0.0") + " m/s, estimate "
                + ARS.MpsToMph(Handling.EstimatedTopSpeed).ToString("0") + " mph");
            Handling.Acceleration = Function.Call<float>(Hash.GET_VEHICLE_ACCELERATION, Car);


            VehicleData.SteeringLock = ARS.RadToDeg(VehicleMemory.GetSteerLock(Car));
            if (VehicleData.SteeringLock < 1 || VehicleData.SteeringLock > 100) VehicleData.SteeringLock = 40;
            ARS.Log(ARS.LogImportance.Info, "Steerlock for " + Car.DisplayName + ":" + VehicleData.SteeringLock + "º");

            VehicleData.WheelBase = ARS.GetWheelBase(Car);
            ARS.Log(ARS.LogImportance.Info, "Wheelbase for " + Car.DisplayName + ":" + VehicleData.WheelBase + " m");
            Control.SteerDegrees = 0f;
            CurrentTrackPoint = ARS.TrackPoints.Last();
            Control.Brake = 0f;
            Control.Throttle = 0f;
            _powerCounter = 1f;
            _appliedThrottleLastFrame = 1f;
            Function.Call((Hash)CheatPowerIncreaseHash, Car, 1.0f);

            LapTimes.Clear();
            LapStartTime = 0;
            VehicleData.ResetLapPeaks();
            Lap = 0;
            _nitrousLapUsed = -1;
            RacePosition = 0;
            FinalPosition = 0;
            IsDNF = false;
            Car.FreezePosition = false;
            CanRegisterNewLap = false;
            _previousNode = -1;
            _inputTrail.Clear();
            _inputTrailChevronDistance = 0f;
            _restHeightAboveGround = -1f;

            string flags = VehicleMemory.GetHandlingFlags(Car).ToString("X");
            int flagsHex = Convert.ToInt32(flags, 16);
            bool hasOffroad = (flagsHex & 0x800000) != 0 || (flagsHex & 0x200000) != 0;
            Handling.Gravity = 9.8f;
            if (hasOffroad) Handling.Gravity *= 1.2f;

            BaseBehavior = RacerBaseBehavior.GridWait;
            FinishedPointToPoint = false;

            Handling.Grip = Function.Call<float>((Hash)0xA132FB5370554DB0, Car);

            float modelGrip = Function.Call<float>((Hash)0x539DE94D44FDFD0D, Car.Model.Hash);
            float modelTopSpeedMph = ARS.MpsToMph(Function.Call<float>((Hash)0xF417C2502FFFED43, Car.Model.Hash));
            float modelAccel = Function.Call<float>(Hash.GET_VEHICLE_MODEL_ACCELERATION, Car.Model.Hash);
            bool modelElectric;
            if (!ARS.ModelElectricCache.TryGetValue(Car.Model.Hash.ToString(), out modelElectric)) modelElectric = ARS.IsElectricModel(Car.Model.Hash);
            // Deliberate: the three model natives are probed live rather than read from the cache the grid was ranked on, so
            // the cache answers only for the electric flag and for a probe that came back non-finite.
            float livePace = ARS.ComputePaceIndex(modelTopSpeedMph, modelGrip, modelAccel, modelElectric);
            if (float.IsNaN(livePace) || float.IsInfinity(livePace)) ARS.ModelPaceIndexCache.TryGetValue(Car.Model.Hash.ToString(), out livePace);
            VehicleData.PowerScale = livePace;
            VehicleData.TextPerformanceIndex = VehicleData.PowerScale.ToString("0.0");
            // The PI used to be glued to the name for the logs; the board has a column for it, so the name stays clean and the suffix
            // survives only where a log line needs to tell two identical names apart.
            if (!ControlledByPlayer) Name = _baseName + " (" + VehicleData.PowerScale.ToString("0.00") + ")";
            DriverName = _baseName;

            _cornerContexts.Clear();
            _cornersRevisionSeeded = -1;
            _skippedApexes.Clear();
            _brakeCommitApexNode = -1;

            Car.Repair();
        }

        public void ComputeSteering()
        {
            _slidePriority = 0f;

            if (!TryGetSteerContext(out TrackPoint steerRefPoint, out float roadWide))
            {
                Control.SteerDegrees = 0f;
                return;
            }

            float speedMps = Math.Max(Car.Velocity.Length(), 1f);


            // --- Travel direction: the pursuit bearing is taken against it, and every term that steers shares it ---

            Vector3 courseDir = Car.ForwardVector;
            if (Car.Velocity.LengthSquared() > 0.01f) courseDir = Car.Velocity.Normalized;


            // --- Lane resolution: override chain (lean → corner → avoidance → recovery → walls → debug pin) ---

            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            float drivableEdge = Math.Max(roadWide - carHalfWidth, 0f);
            // Measured in the CAR's own node frame. Against the steer reference - up to thirty metres ahead - the car's position
            // picks up the sagitta of that chord, L^2/2R: nine metres on a fifty-metre corner, and its sign follows the corner,
            // so a car dead centre read as being at the outer edge. Every same-side test in the mover is made against this value.
            float steerRefOffset = ARS.SignedLaneOffset(Car.Position, CurrentTrackPoint.Position, CurrentTrackPoint.Direction);
            // A lane nobody defines holds the one the car is already in. Zero is the centre lane, not the absence of
            // an answer, so seeding the chain with it would steer every car back to the middle of the road on a
            // straight, where neither the lean nor a corner line has an opinion.
            float laneRequest = steerRefOffset;
            LaneReason laneReason = LaneReason.OwnLane;
            // The two protocols never overlap: the lean is skipped exactly while the apex line is ACTIVE - inside its window and
            // with the apex still ahead - and free everywhere else. Gating it on "a tight corner exists somewhere ahead" switched
            // the lean off for most of a lap; gating it on the line's own activity is the rule the driver stated: with an outside
            // there is no hug-inside.
            CornerPoint lineCorner = NearestApexAhead();
            float cornerLane = 0f;
            LaneReason cornerReason = LaneReason.OwnLane;
            bool cornerLineActive = false;
            if (lineCorner != null && Lap > 0)
                cornerLane = ComputeCornerLane(steerRefPoint, speedMps, Brain.CurrentPerception.CurrentNodeOffset, lineCorner, out cornerReason, out cornerLineActive);
            else _apexInsideNow = false;   // no corner is being evaluated, so nothing may keep the inside guard armed
            if (!cornerLineActive && TryComputeCurveLean(speedMps, steerRefOffset, drivableEdge, out float curveLean)) { laneRequest = curveLean; laneReason = LaneReason.CurveLean; }
            if (cornerLane != 0f) { laneRequest = cornerLane; laneReason = cornerReason; }
            float avoidAheadLane = ComputeAvoidanceLane(roadWide);
            if (avoidAheadLane != 0f) { laneRequest = avoidAheadLane; laneReason = LaneReason.Avoidance; }
            if (_stuckPhase == StuckPhase.Drive && OutOfTrackDistance() > 0f) { laneRequest = NearestEdgeLane(steerRefOffset, drivableEdge); laneReason = LaneReason.RecoveryEdge; }
            float walledLane = ApplyRivalWalls(laneRequest, roadWide);
            if (walledLane != laneRequest) laneReason = LaneReason.WallClamp;
            laneRequest = walledLane;
            if (ARS.DebugToggles[Options.LockLaneCentre]) { laneRequest = LaneLockTestOffsetMeters; laneReason = LaneReason.DebugLock; }
            _laneRequest = laneRequest;
            _laneRequestReason = laneReason;
            UpdateLaneMover(steerRefOffset, drivableEdge);
            _laneOverride = laneReason != LaneReason.OwnLane || Math.Abs(_carriedLane - steerRefOffset) > LaneOverrideHoldMeters;

            // Aim at the lookahead distance, offset by the carried lane. The Gs preview shifts
            // that offset by the lateral motion the car is already committing to, so the correction leads the drift
            // instead of reacting to it; shifting a real point keeps the pursuit's chord real. Always on.
            Vector3 steerRight = Vector3.Cross(steerRefPoint.Direction, Vector3.WorldUp).Normalized;
            float offsetAtProjection = ARS.SignedLaneOffset(ProjectAhead(SteerPreviewSeconds), steerRefPoint.Position, steerRefPoint.Direction);
            // While the aim has overshot the demand the car keeps its own lane: following an aim that is on its way back
            // would drive the car past the goal and back again, which is the wiggle this state exists to avoid.
            float aimLane = _laneOvershoot ? steerRefOffset : _carriedLane;
            float aimPointOffset = aimLane - GsPreviewBlend * (offsetAtProjection - steerRefOffset);
            Vector3 aimPoint = steerRefPoint.Position + steerRight * aimPointOffset;
            _pursuitAimLateralM = ARS.SignedLaneOffset(aimPoint, Car.Position, Car.ForwardVector);

            // --- Lane steer: pure pursuit toward the carried lane ---


            float laneSteerDeg = PursuitSteerDegrees(aimPoint, courseDir);
            _kickAskDegrees = laneSteerDeg;
            if (LaneCrossingScale)
            {
                // Turn-in is a crossing on purpose, so the taper stands down for the entry itself — braking target to
                // apex — and no earlier, because everything before that is still lane work. Damping the ask that
                // unwinds a crossing would be the damper's one-way mistake in reverse.
                float crossingDeg = Vector3.SignedAngle(CurrentTrackPoint.Direction, Car.ForwardVector, Vector3.WorldUp);
                bool turningIn = _nextApexCorner != null && HasPassedBrakingTarget() && !HasPassedApex(_nextApexCorner.Node);
                if (!turningIn && laneSteerDeg * crossingDeg > 0f)
                    laneSteerDeg *= LaneCrossingShare(Math.Abs(crossingDeg), speedMps);
            }
            _steerPursuitDeg = laneSteerDeg;
            // --- Heading assist: match a side-by-side rival's heading ---

            float sideBySideSteerDeg = ComputeSideBySideSteerCorrection(courseDir);

            // --- Contact avoidance: a direct steer-away, not a lane intent ---
            // Inside the "no touching" box, steer away from rivals actually closing laterally; it never moves the
            // request, and parallel traffic must not kill the lane steer. It is added in the assembly after the
            // damper, the slide blend and the on-aim zeroing, so none of them can fade the one term that must not
            // fade — only the steer limiter bounds it, and that closes the steering last.
            // The offset is the pair's relative course angle, ramped by the daylight between the two sides: nothing at
            // the outer gap, the whole angle when the sides touch. The coming-at-my-side gate below is what makes that
            // angle's sign mean "away" — a separating rival turns it into a push toward the rival, which is the slide
            // blend's job to answer and not this term's.
            const float ContactPushOuterGapMeters = 5f;
            // How far ahead of the boxes the push still looks, as a share of a second of the car's own travel. The box
            // is what touching means; a rival that has just left it is still the one that can be crossed by, and the
            // gap to react in is distance, not overlap. Ahead only: a rival behind is the rival's problem.
            const float ContactPushReachSeconds = 0.3f;
            const float ContactPushReachMaxMeters = 20f;
            const float ContactPushMinSpeedMps = 1f;
            float contactSteerDeg = 0f;
            Vector3 velDir = speedMps > 0.5f ? Car.Velocity / speedMps : courseDir;
            // Normalised: Cross(velDir, WorldUp) shrinks by cos(pitch), which shortened every lateral reading on a grade.
            Vector3 velRight = Vector3.Cross(velDir, Vector3.WorldUp).Normalized;
            Vector3 myHoriz = new Vector3(Car.Velocity.X, Car.Velocity.Y, 0f);
            bool myDirectionKnown = myHoriz.Length() >= ContactPushMinSpeedMps;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || !r.RivalRacer.Car.Exists()) continue;
                Vector3 delta = r.RivalRacer.Car.Position - Car.Position;
                float longDist = Math.Abs(Vector3.Dot(delta, velDir));
                float latDist = Math.Abs(Vector3.Dot(delta, velRight));
                float longGate = (VehicleData.ModelDimensions.Y + r.RivalRacer.VehicleData.ModelDimensions.Y) * 0.5f + 1f;
                if (Vector3.Dot(delta, velDir) > 0f)
                    longGate += Math.Min(speedMps * ContactPushReachSeconds, ContactPushReachMaxMeters);
                // Read live rather than off the half-second publish, because the touching point anchors the ramp below.
                float combinedHalfWidth = (VehicleData.BoundingBox + r.RivalRacer.VehicleData.BoundingBox) * 0.5f;
                if (longDist > longGate || latDist > combinedHalfWidth + ContactPushOuterGapMeters) continue;
                Vector3 rivalVel = r.RivalRacer.Car.Velocity;
                float latSide = Vector3.Dot(delta, velRight);
                float latRelVel = Vector3.Dot(rivalVel - Car.Velocity, velRight);
                if (latSide * latRelVel >= 0f) continue;
                // Horizontal only: the published angle is three-dimensional, so a rival lifted on a crest beside the car
                // would read a course difference with no lateral conflict in it.
                Vector3 rivalHoriz = new Vector3(rivalVel.X, rivalVel.Y, 0f);
                if (!myDirectionKnown || rivalHoriz.Length() < ContactPushMinSpeedMps) continue;
                float surfaceGap = Math.Max(latDist - combinedHalfWidth, 0f);
                float ramp = ARS.Remap(surfaceGap, ContactPushOuterGapMeters, 0f, 0f, 1f, true);
                contactSteerDeg += Vector3.SignedAngle(myHoriz, rivalHoriz, Vector3.WorldUp) * ramp;
            }


            // --- Assembly: lane pursuit + corrections + damper + slide blend ---

            const float steerKP = 1.0f;
            // Damp the excess over the yaw the aim point requires, not over zero which taxes every steady corner; the
            // slide blend keeps the zero reference, since its wanted rotation is the countersteer's.
            float fwdSpeed = ARS.GetForwardSpeed(Car);
            // The blend's weight: the slide is answered from half the tyre's peak slip angle and at full weight by one
            // and a half times it. The reference is the peak **at the current speed**, because GTA scales the lateral
            // curve by speed: a tyre peaking at 11.6 degrees at 20 mph peaks at 3.7 at 200, so a fixed angle measures a
            // different part of the curve at every speed.
            if (TRLateralAtSpeed > 0.01f && fwdSpeed >= 2f)
                _slidePriority = ARS.Remap(Math.Abs(VehicleData.SlideAngle), TRLateralAtSpeed * SlideBlendStartFraction, TRLateralAtSpeed * SlideBlendFullFraction, 0f, 1f, true);
            else
                _slidePriority = 0f;
            float yawTarget = 0f;
            if (SteerDampingAimReference && fwdSpeed > 0f && Math.Abs(VehicleData.SlideAngle) < Handling.LateralTractionCurve * SlidingFraction) yawTarget = ARS.RadToDeg(fwdSpeed * _steerAimCurvature);
            float yawRateToDamp = VehicleData.YawRotationPerSecondDegrees - yawTarget;
            float damperGain = SteerDampingFor();
            float nonDamperSteerDeg = (steerKP * sideBySideSteerDeg) + (steerKP * laneSteerDeg);
            float damperTermDeg = -damperGain * yawRateToDamp * DamperEffectScale;
            // The damper only ever subtracts: a left command never gets more left from it.
            if (damperTermDeg * nonDamperSteerDeg > 0f) damperTermDeg = 0f;
            // Past neutral it is countersteering — the wheel pointing against the car's own rotation rather than
            // merely less into it. Not a cap tied to the slide: that fell to zero with the slide and took the rate
            // feedback off a car going straight, which set it oscillating.
            float damperOvershootDeg = Math.Abs(damperTermDeg) - Math.Abs(nonDamperSteerDeg);
            if (damperTermDeg * nonDamperSteerDeg < 0f && damperOvershootDeg > 0f)
            {
                float crossingDeg = Math.Min(damperOvershootDeg * DamperCrossingShare, ARS.DamperCountersteerMaxDeg);
                damperTermDeg = -Math.Sign(nonDamperSteerDeg) * (Math.Abs(nonDamperSteerDeg) + crossingDeg);
            }
            // The crossing above only fires when the pursuit has a side to cross from. Aimed straight the command is zero
            // and the term would steer with no ceiling at all, so the magnitude is bounded here unconditionally.
            float damperCeilingDeg = Math.Abs(nonDamperSteerDeg) + ARS.DamperCountersteerMaxDeg;
            damperTermDeg = ARS.Clamp(damperTermDeg, -damperCeilingDeg, damperCeilingDeg);
            _damperTermDeg = damperTermDeg;
            Control.SteerDegrees = nonDamperSteerDeg + damperTermDeg;

            // Steer trace (Development > Steer Trace): the four signals a loop delay can be measured from - what was asked, what the
            // game got, the yaw it produced and the slip the tyre reports. One line per tick, off by default.
            // Only the racer the player is driving: the trace exists to measure the loop on the car whose hands are on the
            // wheel, and printing it for every rival multiplied the volume by the size of the grid (394 lines/s, 2026-10-26).
            if (ARS.SteerTrace && ControlledByPlayer)
                ARS.Log(ARS.LogImportance.Info, "steertrace t=" + Game.GameTime
                    + " ask=" + Control.SteerDegrees.ToString("0.00")
                    + " in=" + ARS.Remap(Control.SteerDegrees, -VehicleData.SteeringLock, VehicleData.SteeringLock, -1f, 1f, true).ToString("0.000")
                    + " yaw=" + VehicleData.YawRotationPerSecondDegrees.ToString("0.0")
                    + " slip=" + _slip.InnerPct.ToString("0")
                    + " mph=" + ARS.MpsToMph(fwdSpeed).ToString("0")
                    + " cref=" + Vector3.SignedAngle(new Vector3(Car.ForwardVector.X, Car.ForwardVector.Y, 0f), new Vector3(courseDir.X, courseDir.Y, 0f), Vector3.WorldUp).ToString("0.0")
                    + " aimoff=" + aimPointOffset.ToString("0.00")
                    + " proj=" + offsetAtProjection.ToString("0.00")
                    + " ref=" + steerRefOffset.ToString("0.00")
                    + " over=" + (_laneOvershoot ? 1 : 0)
                    + " rawbear=" + _pursuitRawBearingDeg.ToString("0.0")
                    + " dist=" + _pursuitAimDistanceM.ToString("0.0")
                    + " lat=" + _pursuitAimLateralM.ToString("0.00"));


            if (_slidePriority > 0f)
            {
                // The correction term only - not the slidePriority ramp - and the share of the slide the countersteer
                // answers, which reaches all of it at the top of the band. The kick card's rotation is the wanted
                // one while it shares the ask's side, so the blend is silent for it and the ordinary handover
                // resumes at the fold — the same stand-down shape as the turn-in exemption above.
                if (!KickWantsSlide())
                {
                    float countersteerTarget = (steerKP * sideBySideSteerDeg) - (VehicleData.SlideAngle * CountersteerShare());
                    Control.SteerDegrees += (countersteerTarget - Control.SteerDegrees) * _slidePriority;
                }
            }

            // The careful extra: it grows while the blend is committed, at a rate, so it builds instead of jumping. The
            // reset is its own gate and a much narrower one - the rotation turning back against the slide - and that instant
            // drop is what keeps the assist from becoming the next snap. A car sliding steadily has no rotation to reverse,
            // so it accumulates; a car being caught is dropped to zero at once.
            float slideDeg = VehicleData.SlideAngle;
            // A non-finite slide reaching Math.Sign throws ArithmeticException out of the tick: 208 of those in seven minutes on
            // build 1092, each one abandoning the rest of the frame. The guard at the end of this method ran after both calls, so it
            // could never help. Sanitising at the read keeps every consumer below on a number; what produces the non-finite value
            // is still unidentified, so this closes the symptom and not the cause.
            if (float.IsNaN(slideDeg) || float.IsInfinity(slideDeg)) slideDeg = 0f;
            bool blendCommitted = _slidePriority > 0f;
            bool rotationReversed = VehicleData.YawRotationPerSecondDegrees * Math.Sign(slideDeg) < 0f;
            if (!blendCommitted || rotationReversed)
            {
                _countersteerAssistDeg = 0f;
            }
            else
            {
                float assistCeilingDeg = Math.Max(0f, (CountersteerTotalShare - CountersteerSlideShare) * Math.Abs(slideDeg));
                _countersteerAssistDeg = Math.Min(_countersteerAssistDeg + CountersteerAssistDegreesPerSecond * TickScale, assistCeilingDeg);
            }
            Control.SteerDegrees -= Math.Sign(slideDeg) * _countersteerAssistDeg;

            // On its aim to within the noise the command has nothing to correct and only flips sign from tick to
            // tick, so the wheel is left still — unless a slide is steering it, which must not be zeroed. Contact
            // avoidance is not an aiming correction, so it is added after this and after everything above it: a
            // rival closing alongside still moves the wheel while the car is on its line, or sliding.
            if (Math.Abs(_steerAimBearingDegrees) < SmallAimBearingDegrees && _slidePriority <= 0f) Control.SteerDegrees = 0f;
            Control.SteerDegrees += steerKP * contactSteerDeg;

            if (float.IsNaN(Control.SteerDegrees) || float.IsInfinity(Control.SteerDegrees))
                Control.SteerDegrees = 0f;

            // --- Local function: TryGetSteerContext ---

            bool TryGetSteerContext(out TrackPoint localSteerRef, out float localRoadWide)
            {
                localSteerRef = null;
                localRoadWide = 0f;
                if (BaseBehavior == RacerBaseBehavior.GridWait || BaseBehavior == RacerBaseBehavior.FinishedStandStill || CurrentTrackPoint.Node < 3)
                    return false;
                if (!LookAheads.TryGetValue(LookAhead.SteerRef, out localSteerRef) || localSteerRef == null)
                    return false;
                localRoadWide = localSteerRef.TrackHalfWidth;
                return true;
            }
        }

        // Phase 1: match an overlapping rival's heading so side-by-side cars follow the same arc, but only while
        // that rival is moving toward this car's side - one drifting away is leaving room, not asking to be followed.
        float ComputeSideBySideSteerCorrection(Vector3 courseDir)
        {
            float correction = 0f;
            foreach (Rival rival in Brain.Rivals)
            {
                if (rival.RivalRacer == null || !rival.RivalRacer.Car.Exists()) continue;

                Vector3 relativeOffset = ARS.EntityRelativeOffset(Car, rival.RivalRacer.Car);
                // The bodies, not the boxes: CombinedSize.Y carries the one-metre-ahead buffer, and matching the heading
                // of a car that is merely following this one turns this car with it. Side-by-side means the bodies
                // actually overlap along the road, whatever the model's length turns out to be.
                float bodyHalfSum = (VehicleData.ModelDimensions.Y + rival.RivalRacer.VehicleData.ModelDimensions.Y) * 0.5f;
                if (Math.Abs(relativeOffset.Y) > bodyHalfSum) continue;

                float lateralDistance = Math.Abs(relativeOffset.X);

                float fullAssistDistance = rival.CombinedSize.X + SideBySideFullAssistExtraGap;
                float wideDistance = rival.CombinedSize.X + SideBySideAssistRangeExtra;
                if (lateralDistance > wideDistance) continue;

                // Only while the rival is moving toward this car's side, taken on the axis that separates the two:
                // one drifting away is leaving room, and matching their heading there would steer this car across
                // the gap after them. This car's own closure is the rival walls' job, not this term's.
                Vector3 toRival = rival.RivalRacer.Car.Position - Car.Position;
                Vector3 lateralAxis = toRival - Car.ForwardVector * Vector3.Dot(toRival, Car.ForwardVector);
                float lateralGap = lateralAxis.Length();
                // No lateral separation to close - a same-lane overlap has no "away", so the match stays on.
                if (lateralGap > 0.01f && Vector3.Dot(rival.RivalRacer.Car.Velocity, lateralAxis / lateralGap) >= 0f) continue;

                float proximity = ARS.Remap(lateralDistance, wideDistance, fullAssistDistance, SideBySideMinimumAssist, 1f, true);
                Vector3 rivalFlat = new Vector3(rival.RivalRacer.Car.ForwardVector.X, rival.RivalRacer.Car.ForwardVector.Y, 0f);
                Vector3 courseFlat = new Vector3(courseDir.X, courseDir.Y, 0f);
                float headingDifference = -Vector3.SignedAngle(rivalFlat, courseFlat, Vector3.WorldUp);
                if (float.IsNaN(headingDifference) || float.IsInfinity(headingDifference)) continue;

                correction += headingDifference * proximity;
            }
            return correction;
        }

        // Lane Control System 2: positions the car on the inside of the track curvature. The lean names the inside LANE; the curvature
        // decides SPEED, exponentially in metres per second, because how fast a car may be placed is not linear in radius: a 499 m
        // bend must barely move the aim and a 200 m one gets CurveLeanApproachMps and no more, whatever the mover's own cap is.
        const float CurveLeanRadiusMeters = 500f;
        const float CurveLeanChordSeconds = 1.01f;
        const float CurveLeanApproachMps = 5f;
        const float CurveLeanApproachFullRadiusMeters = 200f;
        // E-folding distance: every tau of extra radius multiplies the approach speed by 1/e. At 100 the 500 m gate lands near
        // 0.25 m/s and the 250 m bend near 3 m/s.
        const float CurveLeanApproachTauMeters = 100f;

        // Debug lock for the centring test: a near-centre aim offset, pinned for every racer.
        const float LaneLockTestOffsetMeters = 0.1f;

        // Pure pursuit's steer carries a factor of two the plain bearing misses: the curvature to a point at a given
        // bearing is 2 sin(angle) over the distance, not the angle over it. This is the knob if the lane is still shy.
        const float PursuitGain = 2f;
        // sin falls again past a right angle, so an aim point abeam or behind the car would fade instead of saturating.
        const float MaxPursuitBearingDegrees = 90f;
        // Inside this bearing the car is on its aim to within the noise, so the command only flips sign from tick to
        // tick. The aim bearing carries the lateral error, so a car merely parallel to the road is not inside it.
        const float SmallAimBearingDegrees = 1f;
        // The crossing-angle allowance: the heading that carries the car sideways at LaneCrossingLateralSpeedMps, so it
        // follows 1/speed and falls away to almost nothing at speed. It is a softness scale, not a boundary — the ask
        // that would cross the car further keeps the share threshold/(threshold + crossing), so it fades with the
        // crossing instead of stopping at a line, and the ask that unwinds a crossing is never scaled at all.
        const float LaneCrossingLateralSpeedMps = 1f;
        // Off for a comparison still owed, not parked by decision: the mover upstream may make this scale redundant,
        // and the two have never been driven in that order. The law stays compiled, so reviving it is this flag.
        static readonly bool LaneCrossingScale = false;

        static float LaneCrossingThresholdDegrees(float speedMps)
        {
            float sine = ARS.Clamp(LaneCrossingLateralSpeedMps / Math.Max(speedMps, 0.1f), 0f, 1f);
            return ARS.RadToDeg((float)Math.Asin(sine));
        }

        static float LaneCrossingShare(float crossingDegrees, float speedMps)
        {
            float threshold = LaneCrossingThresholdDegrees(speedMps);
            return threshold / (threshold + crossingDegrees);
        }

        // Pure pursuit's curvature law: the bearing to an aim point is the chord of the circle through the car, so the
        // path's curvature is 2 sin(bearing) over the distance and the steer it demands is that curvature times the
        // wheelbase.
        float PursuitSteerFromBearing(float bearing, float distance)
        {
            bearing = ARS.Clamp(bearing, -MaxPursuitBearingDegrees, MaxPursuitBearingDegrees);
            return ARS.RadToDeg((float)Math.Atan(PursuitGain * Math.Sin(ARS.DegToRad(bearing)) * VehicleData.WheelBase / distance));
        }

        float PursuitSteerDegrees(Vector3 aimPoint, Vector3 heading)
        {
            Vector3 toAim = aimPoint - Car.Position;
            // FLAT, both of them: the aim is 8 m ahead and the terrain between the two is not level, so the 3D angle between them
            // carries the road's own grade. One metre of difference over eight is 7 degrees of 'bearing', which flipped sign with
            // every node step and was read as a lateral tremor for a whole session. Steering is a horizontal problem.
            Vector3 toAimFlat = new Vector3(toAim.X, toAim.Y, 0f);
            Vector3 headingFlat = new Vector3(heading.X, heading.Y, 0f);
            float distance = toAimFlat.Length();
            if (distance <= 0.5f)
            {
                _steerAimBearingDegrees = 0f;
                return 0f;
            }
            float bearing = Vector3.SignedAngle(headingFlat, toAimFlat, Vector3.WorldUp);
            if (float.IsNaN(bearing) || float.IsInfinity(bearing))
            {
                _steerAimBearingDegrees = 0f;
                return 0f;
            }

            _pursuitRawBearingDeg = bearing;
            _pursuitAimDistanceM = distance;
            _steerAimBearingDegrees = bearing;
            _steerAimCurvature = 2f * (float)Math.Sin(ARS.DegToRad(bearing)) / distance;
            return PursuitSteerFromBearing(bearing, distance);
        }

        bool TryComputeCurveLean(float speedMps, float steerRefOffset, float laneEdge, out float lane)
        {
            lane = 0f;
            int count = ARS.TrackPoints.Count;
            int fwdNode;
            int fwdOffset = (int)(speedMps * CurveLeanChordSeconds);
            if (ARS.IsPointToPoint)
                fwdNode = (int)ARS.Clamp(CurrentTrackPoint.Node + fwdOffset, 0, count - 1);
            else
                fwdNode = ((CurrentTrackPoint.Node + fwdOffset) % count + count) % count;

            TrackPoint ahead = ARS.TrackPoints[fwdNode];
            if (!(ahead.PreciseCurveRadius < CurveLeanRadiusMeters)) return false;

            float cornerDir = Math.Sign(ahead.Angle);
            if (cornerDir == 0f) return false;
            // Clamp answers a NaN with its min bound, so an unreadable car lane would ask for the wrong edge.
            if (float.IsNaN(steerRefOffset) || float.IsInfinity(steerRefOffset)) return false;

            // The inside LANE, named absolutely, with the approach speed falling off exponentially with radius: full
            // CurveLeanApproachMps at 200 m and tighter, 1/e per extra tau of radius, effectively nothing at the 500 m gate.
            float overFullRadius = Math.Max(ahead.PreciseCurveRadius - CurveLeanApproachFullRadiusMeters, 0f);
            _leanApproachMps = CurveLeanApproachMps * (float)Math.Exp(-overFullRadius / CurveLeanApproachTauMeters);
            lane = -cornerDir * Math.Max(laneEdge - LaneInsideGapMeters, 0f);
            return true;
        }

        // The aim carries an override until it has come back this close to the car's own lane, so a consumer gated on
        // that fact is released with the steering rather than with the request.
        const float LaneOverrideHoldMeters = 0.1f;

        // The lane mover's settled numbers for a 10 m track (5 m a side, so the safe bound B is 4 m). The ideal line
        // crosses two bounds over 2R·leadAngle metres, which asks about 8 m/s and 21 m/s^2 at a supercar's own apex
        // speed and 6.4 m/s and 13.6 m/s^2 at a vanilla car's — self-similar across radii, because a car takes each
        // corner at its own limit — so the cap has to sit above the line's own rate and the acceleration is the only
        // lunge knob. At the shipped 26 the aim tracks the line inside a centimetre, and a 6 m request valid for half a second moves 3.4 m
        // where it used to step the whole way.
        // Sized from the line's own geometry, not by feel: the outside-inside traversal is 2 * safeBound - 18 m on a wide
        // track - and it has to happen inside the blend's 1.5 s, which needs 4 * 18 / 1.5^2 = 32 m/s^2 to start and stop
        // within it. At 14 the aim could not follow the ramp at all, and a reversal first had to spend 24 / 14 = 1.7 s
        // killing its own speed, longer than the whole window.
        const float LaneMoverAccelMps2 = 26f;
        const float LaneMoverSpeedCapMps = 24f;
        // A tick that arrives after a stall must not integrate the whole gap, or the aim is thrown to the edge clamp
        // and walks back: the pedal path bounds its own step for the same reason.
        const float LaneMoverMaxStepSeconds = 0.1f;
        // Under this a request is too close to the car's own lane for "the other side" to mean anything, and the aim would be
        // flipped to the car on noise.
        const float LaneCrossMinMeters = 0.1f;
        // The aim is leashed to the car: full speed up to LaneTaperNearMeters away, slowing to LaneTaperCapShare by LaneTaperFarMeters.
        // The leash slows the lane down as it leaves the car; it never stops it, or the car could not be led anywhere at all.
        const float LaneTaperNearMeters = 2f;
        const float LaneTaperFarMeters = 4f;
        const float LaneTaperCapShare = 0.5f;

        // Carries the aim onto the request. The rate is bounded by the acceleration, which is what stops a
        // request that flips inside half a second from stepping the lane; the arrival term decelerates on the rate
        // that just stops in the distance left; and the request's own rate feeds forward, so a line that moves
        // smoothly is tracked without the v^2/2a lag a pure arrival law would leave (2 m at 5 m/s).
        void UpdateLaneMover(float steerRefOffset, float laneEdge)
        {
            // Inside mode, applied at the last point before the car moves: once the demand has crossed to the apex side
            // of the car, a request that points outward is not applied - whatever produced it. That is the guard the
            // chain cannot give, because every tick the request is rebuilt and any reason along the way can hand over a
            // value the corner would not have asked for.
            // The objective carries the guard: a lane may never name the track's very edge, only where the car's CENTRE can
            // sit, so the demand handed to the mover is already roadWide - carHalfWidth and every step toward it is legal.
            float request = ARS.Clamp(_laneRequest, -laneEdge, laneEdge);
            if (_apexInsideNow && _apexLineInsideCornerDir != 0f && OutOfTrackDistance() <= 0f)
            {
                float inward = -_apexLineInsideCornerDir;
                if ((request - steerRefOffset) * inward < 0f)
                {
                    if (Game.GameTime > _laneGuardLoggedAt)
                    {
                        _laneGuardLoggedAt = Game.GameTime + 1000;
                        ARS.Log(ARS.LogImportance.Info, "Lane guard " + Name + ": held at " + steerRefOffset.ToString("0.00") + ", request was " + request.ToString("0.00") + ", inside dir " + inward.ToString("0") + ", carrier " + _carriedLane.ToString("0.00"));
                    }
                    request = ARS.Clamp(steerRefOffset, -laneEdge, laneEdge);
                }
            }

            float gap = request - _carriedLane;
            if (float.IsNaN(gap) || float.IsInfinity(gap)) return;

            if (_laneSnap)
            {
                _laneSnap = false;
                _carriedLane = request;
                _laneRequestPrev = request;
                _laneRate = 0f;
                _laneOvershoot = false;
                return;
            }

            // A request on the far side of the car is placed ON the car instead of being swept across it. The stretch between
            // the aim and the car's own lane is ground the car already occupies: none of it can be seen to move, and paying for
            // it costs a full reversal - which is what made a curve changing handedness read as half a second of frozen lane.
            // Teleporting to the car and starting from rest keeps the two movements that ARE visible: away from the car, onto
            // the new side.
            float aimFromCar = _carriedLane - steerRefOffset;
            float askFromCar = request - steerRefOffset;
            // Only a REVERSAL crosses to the car: the request is on the far side of the car AND behind the aim's own motion. Requiring just
            // the far side pinned the aim to the car's lane during every out-in - the demand is inside while the aim still leads on the
            // outside of a car sitting mid-track, so the two straddled the centre every tick and the line could never cross it.
            bool reversing = gap * _laneRate < 0f;
            if (reversing && aimFromCar * askFromCar < 0f && Math.Abs(aimFromCar) > LaneCrossMinMeters && Math.Abs(askFromCar) > LaneCrossMinMeters)
            {
                if (Game.GameTime > _laneCrossLoggedAt)
                {
                    _laneCrossLoggedAt = Game.GameTime + 1000;
                    ARS.Log(ARS.LogImportance.Info, "Lane cross " + Name + ": aim " + _carriedLane.ToString("0.00") + " to car " + steerRefOffset.ToString("0.00") + ", request " + request.ToString("0.00") + " (" + _laneRequestReason + "), rate " + _laneRate.ToString("0.00"));
                }
                _carriedLane = ARS.Clamp(steerRefOffset, -laneEdge, laneEdge);
                _laneRate = 0f;
            }

            // Same side of the car, but the demand already inside the aim: the aim has overshot the goal, so the car would be
            // steered past it and back again. It returns at twice the acceleration - the speed cap is untouched - and the car
            // stops following the lane at all until the aim is back on the segment.
            aimFromCar = _carriedLane - steerRefOffset;
            askFromCar = request - steerRefOffset;
            _laneOvershoot = aimFromCar * askFromCar > 0f && Math.Abs(aimFromCar) > Math.Abs(askFromCar) + LaneCrossMinMeters;

            float dt = Math.Min(TickScale, LaneMoverMaxStepSeconds);
            // The move OUT to the plateau is half the speed and half the acceleration of everything else: opening the line to the
            // outer edge is a positioning choice the car makes with room to spare, and at the full rate it arrived as a sideways
            // lunge. The apex side keeps the full pair, because there the car is being placed for a corner it is already turning into.
            bool toOutside = _laneRequestReason == LaneReason.OutsideLine;
            float moverAccel = toOutside ? LaneMoverAccelMps2 * 0.5f : LaneMoverAccelMps2;
            // Half the inertia on the way back, and only the acceleration: the cap is what bounds the visible lunge, so an
            // overshoot returns twice as promptly without ever moving faster than any other request.
            if (_laneOvershoot) moverAccel *= 2f;
            float moverSpeedCap = toOutside ? LaneMoverSpeedCapMps * 0.5f : LaneMoverSpeedCapMps;
            // The lean's approach rate is the bend's own: share 0 in a 500 m curve, the full cap at 250 m or tighter.
            // The lean's approach rate is its own absolute speed, not a share of the mover's cap: 5 m/s at 200 m and tighter.
            if (_laneRequestReason == LaneReason.CurveLean) moverSpeedCap = Math.Min(moverSpeedCap, _leanApproachMps);
            // Full cap within the leash's near end, falling to nothing at its far end: the lane leads the car, it does not run
            // away from it.
            float awayFromCar = Math.Abs(_carriedLane - steerRefOffset);
            float leash = ARS.Clamp((awayFromCar - LaneTaperNearMeters) / (LaneTaperFarMeters - LaneTaperNearMeters), 0f, 1f);
            moverSpeedCap *= 1f + leash * (LaneTaperCapShare - 1f);
            float requestRate = ARS.Clamp((request - _laneRequestPrev) / dt, -moverSpeedCap, moverSpeedCap);
            _laneRequestPrev = request;

            // The two terms are clamped apart: the arrival never exceeds the cap, but a request already moving at the
            // cap must not lose the arrival's headroom, or the gap can never close while the line keeps moving.
            float arrivalRate = Math.Sign(gap) * Math.Min(moverSpeedCap, (float)Math.Sqrt(2f * moverAccel * Math.Abs(gap)));
            float desiredRate = requestRate + arrivalRate;

            // A reversal drops the momentum instead of spending it: carrying speed one way while the line asks for the other
            // is the lag that reads as the lane taking a moment to change direction, and there is nothing worth keeping.
            if (desiredRate * _laneRate < 0f) _laneRate = 0f;
            float rateStep = moverAccel * dt;
            _laneRate += ARS.Clamp(desiredRate - _laneRate, -rateStep, rateStep);
            float step = _laneRate * dt;

            _carriedLane += step;

            // The walls bound the request alone. A bound laid on the aim is placed in one tick rather than carried
            // onto, which is the step this mover exists to remove, so a corridor that closes is carried onto like any
            // other request. Nothing below this line can move the aim without inertia: the acceleration and the speed
            // cap are the whole law, and the aim answers the geometry rather than the car's own position.

            if (Math.Abs(_laneRate) > 0.2f && Game.GameTime > _laneMoverLoggedAt)
            {
                _laneMoverLoggedAt = Game.GameTime + 1000;
                ARS.Log(ARS.LogImportance.Info, "Lane mover " + Name + ": asked " + _laneRequest.ToString("0.00") + " (" + _laneRequestReason + "), aimed " + _carriedLane.ToString("0.00") + ", rate " + _laneRate.ToString("0.00"));
            }
        }

        // How far ahead of a corner the car decides whether it wants to position or brake for it.
        const float RequirementLookaheadSeconds = 3.95f;

        // The chord meets the outside edge at the lead, and a car cannot change heading at the rate that implies, so
        // the blend is stretched past the geometric lead: the endpoints stay where the chord put them, the rate of
        // change falls as one over the multiple, and the path's curvature as one over its square.
        // The profile is geometric - no time in it - but the car arrives late to it, so the whole line is pulled this
        // far toward the apex in travel: half a second at the car's own speed, which is what the steering needs to
        // actually reach the inside before the apex passes.
        // Apex at T0, the outside-inside movement at T-1.5. That is the whole shape: the ramp runs from two seconds out to
        // the apex, and the outside needs no constant of its own because everything beyond the ramp's start is already the
        // outer clamp - the platform IS the clamp. It was being eaten because the clock ran on the apex's speed, which
        // measures a slow corner's window in metres too short for a fast car to reach the clamp in time.
        // How long before the apex the corner starts asking for the outside at all: the 1.5 s ramp plus the 4.5 s the car
        // needs to get out there and hold it.
        // The two halves need room, and room on this road is measured in nodes: the line exists inside two hundred of them
        // and the outside plateaus for those 250 metres. The ramp runs from 50 m out to 5 m out.
        // The outside is a distance decision and the apex is not. Placing the car on the outer edge is done with room to spare
        // and does not change with speed, so the window and the plateau stay in metres of route.
        const float IdealLineWindowMeters = 300f;
        // Clearance from the edges, and the personal bias. The outside line keeps half a metre from the outer edge; the lean keeps a
        // full metre from the inner one. Each car also rolls its own pull-in of 0..LaneBiasMaxMeters, once per corner and held until
        // it passes, so a field does not stack itself on one line - and never enough to reach the centre, which the clamp guarantees.
        const float LaneOutsideGapMeters = 0.5f;
        const float LaneInsideGapMeters = 1f;
        const float LaneBiasMaxMeters = 4f;
        // How much nearer another corner must be before it takes the line away from the one already adopted. A few metres of
        // hysteresis is enough: two apexes that swap places tick after tick do so by less than that.
        const float LineApexStickyMeters = 30f;
        // Moving from the outside to the inside is a time decision: 3 s to the apex the car starts coming in, 1 s it is on the
        // inside. In metres that stretch would be 20 m at 25 km/h and 80 m at 100, which is why the distance blend arrived
        // early when slow and late when fast.
        const float IdealLineBlendStartSeconds = 2f;
        const float IdealLineBlendEndSeconds = 0.25f;
        const float IdealLineMaxRadiusMeters = 50f;
        // How far under the apex speed a car may be and still be given the outside. Off the grid every car is under it, which is
        // the point: they crawl to the first corner instead of diving for the outer edge.
        const float IdealLineSpeedMarginMph = 15f;

        // The ideal line is the widest one that still grazes the inside edge at the apex, and widening the arc until it
        // touches the outside answers that with a straight chord - whose outer end meets the outside edge at an angle,
        // which puts a corner in the target exactly where the car is settling onto it. So the chord supplies the two
        // ends - the outside edge a lead before the apex, the inside edge at it - and the blend between them leaves and
        // meets both edges flat, which is the part a car at speed can actually follow.
        // Nearest corner by absolute distance, passed or not. The profile is symmetric in the distance to the apex, so it
        // opens back outward by itself on the way out - which is the exit half of the outside-inside line. Choosing only
        // corners still ahead cut that half off at the apex and handed the request straight to the next corner, whose
        // demand at that distance is the outer clamp: a step from the inside edge to the outside edge, taken at the one
        // moment the car is loading the tyre hardest. The crossover is where the two are equidistant, so the blend is
        // continuous across the whole corner instead of resetting on it.
        CornerPoint NearestApexAhead()
        {
            int count = ARS.TrackPoints.Count;
            if (count < 10) return null;
            // Only the corners the line owns are candidates: past IdealLineMaxRadiusMeters the bend is a kink the car takes
            // without leaving its lane, and picking one of those here handed the lane to the lean (yellow) while a tight
            // corner was still two hundred metres away.
            CornerPoint best = null;
            float bestMeters = float.MaxValue;
            CornerPoint adopted = null;
            float adoptedMeters = float.MaxValue;
            foreach (CornerPoint corner in ARS.Corners)
            {
                if (corner.DetectedRadius >= IdealLineMaxRadiusMeters) continue;
                float metres = ARS.TrackPoints[corner.Node].CumulativeDistance - CurrentTrackPoint.CumulativeDistance;
                if (!ARS.IsPointToPoint && ARS.RouteLengthMeters > 0f && Math.Abs(metres) > ARS.RouteLengthMeters * 0.5f)
                    metres -= Math.Sign(metres) * ARS.RouteLengthMeters;
                if (corner.Node == _lineCornerNode && metres >= 0f) { adopted = corner; adoptedMeters = metres; }
                if (metres < 0f || metres >= bestMeters) continue;
                bestMeters = metres;
                best = corner;
            }
            // The corner the line is already working on keeps it until another is this much nearer. Without it a pair of apexes a
            // few metres apart - a chicane, a bend detected twice - swap identity tick after tick, and a swap resets the blend.
            if (adopted != null && adoptedMeters <= bestMeters + LineApexStickyMeters) return adopted;
            return best;
        }

        float ComputeCornerLane(TrackPoint steerRefPoint, float speedMps, float ownLane, CornerPoint lineCorner, out LaneReason reason, out bool lineActive)
        {
            reason = LaneReason.OwnLane;
            lineActive = false;
            // Reset every pass, then let the inside branch set it again. The flag is READ unconditionally by the mover, so a
            // value left over from the last corner kept refusing outward asks on a corner the car had not even reached yet.
            _apexInsideNow = false;
            // The lane belongs to the corner the car is approaching, never to the head of the plan's queue. That queue
            // looks past a corner it has flagged as trivial, which swapped the head for the next apex while the car was
            // still inside the flagged one's window: the profile then read as "far away" and the line went back to the
            // outer clamp, so the car stopped turning in with the apex still ahead. Only passing the apex ends a line.
            float alongNow = AlongTrackSpeed;
            if (TickScale > 0.0001f) _lineDecelG = Math.Max(0f, (_lineAlongPrev - alongNow) / TickScale / 9.81f);
            _lineAlongPrev = alongNow;
            CornerPoint c = lineCorner;
            if (c == null) return 0f;
            int apexNode = c.Node;

            // The line is a function of the distance to the apex, so no time enters the geometry - and it is measured
            // at the point the offset is applied to, or the profile is one lookahead stale. The reference sits ahead
            // of the car, though, so in the last lookahead before an apex it is already past one while the car is
            // not: falling back to the car's own distance there keeps the entry branch driving to the inside, where
            // reading the reference would open the profile outward at the apex.
            // Signed, and negative once the apex is behind: the blend reads it as "nodes to go", so a wrapped forward
            // distance would hand it a near-full lap and open the line outward on the exit half.
            int count = ARS.TrackPoints.Count;
            // Metres along the route, not nodes, and measured from the CAR: the reference point sits a lookahead ahead, so
            // reading the window there opened it early and killed it a lookahead before the apex - a distance that moved with
            // speed, which is the one thing this window must not do. `CumulativeDistance` is metres and is rebuilt with the
            // route; the wrap is the same shortest-way-round the rival gap uses.
            float metresToApex = ARS.TrackPoints[apexNode].CumulativeDistance - CurrentTrackPoint.CumulativeDistance;
            if (!ARS.IsPointToPoint && ARS.RouteLengthMeters > 0f && Math.Abs(metresToApex) > ARS.RouteLengthMeters * 0.5f)
                metresToApex -= Math.Sign(metresToApex) * ARS.RouteLengthMeters;

            float cornerDir = Math.Sign(c.Angle);
            if (cornerDir == 0f) return 0f;

            float halfWidth = steerRefPoint.TrackHalfWidth;
            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            float safeBound = Math.Max(halfWidth - carHalfWidth, 0f);

            // The car's own line for this corner: a pull-in rolled once when the corner is adopted and held until the car passes its
            // apex. The outside bound keeps the half-metre gap and the bias; floored at zero so the bias can reach the centre of the
            // road but never cross it.
            if (_lineBiasNode != apexNode)
            {
                _lineBiasNode = apexNode;
                _lineBiasMeters = ARS.GetRandomInt(0, (int)(LaneBiasMaxMeters * 100f) + 1) / 100f;
            }
            float outsideBound = Math.Max(safeBound - LaneOutsideGapMeters - _lineBiasMeters, 0f);
            // Corner-commit: hold the defend/dive line for the card's whole life, but only on the corner the card was armed
            // for. Without the window test EVERY corner came back as CommitCard while any card was alive, because the branch
            // returns before the apex line is ever computed - three to four seconds of no outside on whatever corner the car
            // happened to reach, which is exactly the intermittent blackout the driver reported.
            bool isCornerCommit = ActiveManeuver.Target != null
                && (ActiveManeuver.Type == ManeuverType.DefendLane || ActiveManeuver.Type == ManeuverType.DiveBomb)
                && metresToApex >= 0f && metresToApex <= IdealLineWindowMeters;
            if (isCornerCommit)
            {
                Rival target = Brain.Rivals.FirstOrDefault(r => r.RivalRacer == ActiveManeuver.Target);
                if (target != null && target.RivalRacer.Car.Exists())
                {
                    float gap = target.CombinedWidth + 0.6f;
                    float commitLane = target.NodeOffset + (-cornerDir) * gap;
                    reason = LaneReason.CommitCard;
                    return ARS.Clamp(commitLane, -safeBound, safeBound);
                }
            }

            // The line lives for the whole approach and stops at the apex, nothing else. It used to end twenty miles an
            // hour above the apex speed, which dropped it exactly while the car was still closing: the entry went out to
            // the clamp, reached the inside and then let go twenty short, so the car stopped turning toward the apex with
            // its speed still to lose.
            // The window is TIME to the apex, and the only thing that opens it is being inside it. `RequiresPositioning` used
            // to gate this as well and it armed only on a corner that was TIGHT and where the car arrived ten miles an hour
            // over - so once the braking plan got good enough to arrive on speed, the line stopped existing: 65 % of the
            // samples in one session read armed=False, started=False. The outside is not a correction for overspeed, it is
            // the first half of the line, and a corner asks for it because it is a corner.
            // Only corners with a real apex get a line: past this radius the bend is a kink the car takes without moving
            // off its lane, and asking for an outside there was noise.
            // A car that is not going to reach the corner fast enough to need an outside does not get one. Without this, an apex
            // near the start line has every car diving for the outer edge off the grid: the window is metres, and a car at 30 km/h
            // is inside it for ten seconds. The margin is measured under the apex speed, so a car already slowed for the corner
            // keeps the line it is committed to.
            bool fastEnoughForLine = speedMps > ApexSpeedWithDownforce(c.DetectedRadius) - ARS.MphToMps(IdealLineSpeedMarginMph);
            bool wantsPosition = metresToApex >= 0 && metresToApex <= IdealLineWindowMeters
                && c.DetectedRadius < IdealLineMaxRadiusMeters && fastEnoughForLine;
            bool entryActive = wantsPosition && !HasPassedApex(apexNode);
            lineActive = wantsPosition;
            if (wantsPosition) _lineCornerNode = apexNode;
            // The inside guard ends with its apex - including on the tick the car passes it, when this corner stops being the nearest
            // one ahead and the ApexLine branch below no longer runs. Clearing it there alone was unreachable, so the field survived
            // the race and armed lap 2's guard before lap 2's demand had gone inside.
            if (_apexLineInsideNode >= 0 && HasPassedApex(_apexLineInsideNode)) _apexLineInsideNode = -1;
            // The phase names the offset: +halfWidth is the outer edge, -halfWidth the inner one. Both are the RAW half width;
            // the mover's own guard (roadWide - carHalfWidth) is what finally places the car's CENTRE, so the body may still
            // overhang the white line at the apex - which is the point of an apex. The inside extreme is reached ONE SECOND
            // before the apex, so the car is already at the inside when it gets there instead of arriving on the way out.
            // The clock runs on the speed the car carried when it ENTERED the window, latched once per corner. The live speed moved the
            // blend opening with braking and traffic; the apex speed was too slow and shortened the manoeuvre to a third of its real time.
            // The entry speed is the one number that is both honest about the approach and stable for the whole corner.
            if (_lineClockNode != apexNode) { _lineClockNode = apexNode; _lineClockMps = 0f; }
            if (_lineClockMps <= 0f && metresToApex >= 0f && metresToApex <= IdealLineWindowMeters)
                _lineClockMps = Math.Max(speedMps, 1f);
            float clockBoostMph = ARS.Clamp(_lineDecelG * LineClockBrakeMphPerG, -LineClockBrakeCapMph, LineClockBrakeCapMph);
            float secondsToApex = metresToApex / Math.Max(_lineClockMps + ARS.MphToMps(clockBoostMph), 1f);
            float linePhase = ARS.Clamp((IdealLineBlendStartSeconds - secondsToApex)
                / Math.Max(IdealLineBlendStartSeconds - IdealLineBlendEndSeconds, 0.01f), 0f, 1f);
            // The line never backs out of the apex. The clock is the car's own speed, so braking lengthens the time to the apex and
            // would reopen a blend that has already closed; the phase is ratcheted per apex, so the demand can only move inward and
            // a new corner starts from the outside.
            // The armour exists only where the blend exists: inside the last IdealLineBlendStartSeconds before the apex. Before that
            // the line is asking for the outside and there is nothing to ratchet, and a direction latched 300 m out would follow the
            // car into situations the corner knows nothing about. The SPEED floor is deliberately not part of this test: it moves with
            // mechanical grip and downforce, and one tick under it used to sweep the ratchet away mid-corner.
            bool inBlendWindow = metresToApex >= 0 && metresToApex <= IdealLineWindowMeters
                && c.DetectedRadius < IdealLineMaxRadiusMeters && !HasPassedApex(apexNode)
                && secondsToApex <= IdealLineBlendStartSeconds;
            if (inBlendWindow)
            {
                // A different corner always starts from the outside. The old rule kept the latch when the handedness matched, so
                // that two apexes of the same bend could not reset each other - NearestApexAhead's hysteresis does that now, and it
                // does it before the identity can change, so keeping the phase here would only make a real next corner start inside.
                if (_linePhaseNode != apexNode)
                {
                    bool seedFromLane = secondsToApex <= IdealLineBlendStartSeconds
                        && !(_linePhaseSeededNode == apexNode && Game.GameTime - _linePhaseSeededAt < LinePhaseReseedBlockMs);
                    _linePhaseNode = apexNode;
                    _linePhaseLatched = 0f;
                    // A corner adopted already inside the blend has no outside left to run: seeding the latch from the car's own lane stops
                    // the line asking for an outside it has already passed, which is a swerve out on a corner just met. The lane is read in
                    // the same frame as the demand - cornerDir * ownLane - so the two are comparable.
                    // The seed is refused for an apex this line already seeded recently: a picker that drops a corner and comes back a tick
                    // later would otherwise re-inject the phase mid-manoeuvre. That is what the timestamp is for.
                    if (seedFromLane)
                    {
                        _linePhaseLatched = ARS.Clamp((outsideBound - cornerDir * ownLane) / Math.Max(outsideBound + safeBound, 0.01f), 0f, 1f);
                        _linePhaseSeededNode = apexNode;
                        _linePhaseSeededAt = Game.GameTime;
                    }
                    _linePhaseDir = cornerDir;
                }
                _linePhaseLatched = Math.Max(_linePhaseLatched, linePhase);
                linePhase = _linePhaseLatched;
                // Closer to the apex only: a sign that flips - a second corner of the other handedness adopted for a tick, an angle
                // read on the wrong node - cannot throw the demand to the far side of the road once the blend has started.
                cornerDir = _linePhaseDir != 0f ? _linePhaseDir : cornerDir;
            }
            // No else-clear: the ratchet is released when a DIFFERENT corner is adopted, or in Launch. Clearing it whenever the
            // blend window flickered was how the line reset mid-manoeuvre - and the window contains the speed floor, which moves
            // with grip, so it flickers precisely when the car is unsettled.
            // The last second is a commitment, not a preference. Whatever the blend managed by now - a short window at speed, a lean
            // contesting the lane, a speed floor that dropped out under braking - inside the final second the target IS the inside
            // extreme, and the line is active so that nothing else can claim the lane from it.
            bool apexCommitted = secondsToApex <= IdealLineBlendEndSeconds && !HasPassedApex(apexNode);
            if (apexCommitted)
            {
                linePhase = 1f;
                lineActive = true;
            }
            float offset = (entryActive || apexCommitted) ? outsideBound - (outsideBound + safeBound) * linePhase : 0f;
            // Temporary: why the apex line is or is not being asked for, for the cars the driver can see, once a second.
            if (ARS.DebugToggles[Options.ShowTrackAnalysis] && Car.Position.DistanceTo(Game.Player.Character.Position) < 60f
                && Game.GameTime > _cornerLineLoggedAt)
            {
                _cornerLineLoggedAt = Game.GameTime + 1000;
                ARS.Log(ARS.LogImportance.Info, "Corner line " + Name + ": R " + c.DetectedRadius.ToString("0.0") + (c.DetectedRadius < CornerTightRadius ? " tight" : " WIDE")
                    + " wants=" + wantsPosition
                    + " entry=" + entryActive + " t=" + secondsToApex.ToString("0.0") + " d=" + metresToApex.ToString("0.0") + " demand=" + (cornerDir * offset).ToString("0.00")
                    + " own=" + ownLane.ToString("0.00") + " out=" + outsideBound.ToString("0.00")
                    + " bias=" + _lineBiasMeters.ToString("0.00") + " in=" + safeBound.ToString("0.00")
                    + " clock=" + ARS.MpsToMph(_lineClockMps + ARS.MphToMps(clockBoostMph)).ToString("0") + "mph"
                    + " br=" + _lineDecelG.ToString("0.00") + "g"
                    + " apex=" + apexNode + " dir=" + cornerDir.ToString("0"));
            }
            // The profile's far clamp is the outer edge, so the offset itself names the phase: at the clamp it holds the
            // outside line, and everything below it is the blend toward the apex.
            if (entryActive) reason = linePhase <= 0f ? LaneReason.OutsideLine : LaneReason.ApexLine;

            // The blend may only pull toward the apex: a car already inside the demand is never sent back out to meet it,
            // or the entry reads as out and then in. Three things are exempt by design - the hold at the clamp, because
            // reaching the outer edge is what the outside approach is for; every reason that is not the blend; and a car
            // off the road, whose way back is this line. The comparison is made against the car's own lane rather than
            // the reference frame's, which reads inward by the bend's sagitta and would refuse the line outright on the
            // fast tight corners it exists for.
            // The pull may open the car outward only while the demand is still outside it: that is the approach, the
            // half of the line that puts the car wide. The moment the demand crosses to the inside of the car, the
            // apex side is where it lives, and no later demand may drag the car back out - which is what a re-read of
            // a corner, a wrapped distance or a profile step up can produce, and what reads from the seat as the line
            // snapping the car outward mid-corner. The latch is keyed by apex node, so a new corner starts clean and
            // the only thing that ends the inside line is passing its apex.
            if (reason == LaneReason.ApexLine)
            {
                // Inside, measured in the corner's own frame: -cornerDir points at the apex side on either hand. Comparing
                // cornerDir*offset with cornerDir*ownLane only means "inside" on a LEFT corner - on a right one it means the
                // opposite, so the guard never armed and the whole inside mode was left-hand only. Running a track backwards
                // swapped the corners that misbehaved, which is exactly what that asymmetry looks like from the seat.
                bool demandInside = (cornerDir * offset - ownLane) * -cornerDir > 0f;
                if (demandInside)
                {
                    _apexLineInsideNode = apexNode;
                    _apexLineInsideCornerDir = cornerDir;
                }
                // The guard ends with the apex, and must: past it the same profile opens back outward, which is the
                // exit half of the line. Holding the guard there would pin the car to the inside all the way out.
                _apexInsideNow = _apexLineInsideNode == apexNode;
                if (!demandInside && _apexInsideNow && OutOfTrackDistance() <= 0f)
                {
                    if (Game.GameTime > _cornerRefusalLoggedAt)
                    {
                        _cornerRefusalLoggedAt = Game.GameTime + 1000;
                        ARS.Log(ARS.LogImportance.Info, "Corner line held " + Name + ": demand " + (cornerDir * offset).ToString("0.00") + ", own " + ownLane.ToString("0.00") + ", " + metresToApex.ToString("0.0") + "m to apex, R " + c.DetectedRadius.ToString("0.0") + ", " + ARS.MpsToMph(speedMps).ToString("0") + " mph");
                    }
                    return ownLane;
                }
            }

            return cornerDir * offset;
        }

        // A rival ahead is only a pass target while the two velocity vectors sit within this angle; past it the
        // rival is not in a valid position to avoid, and the car falls back to follow-behind.
        const float AvoidAngleGateDegrees = 20f;
        // The wall and pass geometry only trust an adjacency when the pair also sits on neighbouring route nodes,
        // so an overlapping car on the far leg of a U is never mistaken for a trackside blockage.
        const float SameSectionMaxGapMeters = 30f;
        // A rival this many seconds away puts its wall up now, so the car has moved over before it arrives. The walls are
        // what starts an overtake, not only what survives one.
        const float WallApproachSeconds = 3f;
        // Half of the 1000 an engine's health tops out at, which is not the body's ceiling.
        const float EngineHealthHalf = 500f;
        // How far off the centreline a rival must sit before its wall takes the right side. A body width sent rivals that were
        // clearly to the right onto the left wall, which pushed the left target past the far edge on a narrow road.
        const float WallSideDeadbandMeters = 0.3f;
        // After a pass the lane must not snap back to the line: the rear is still beside the car that was passed, and
        // turning in there is what puts it in the other car's nose. The request holds until nothing is left alongside.
        // ZOMBIE (2026-10-25): unused while avoidHoldEnabled is false - see ComputeAvoidanceLane.
        const float AvoidHoldBehindMeters = 14f;
        const float AvoidHoldLateralMeters = 1.5f;
        // A pass that has taken longer than this is not a pass any more: the hold dies and the corner line gets the lane back.
        // ZOMBIE (2026-10-25): unused while avoidHoldEnabled is false - see ComputeAvoidanceLane.
        const int AvoidHoldTimeoutMs = 8000;

        // Pick a lane to pass a rival ahead. If two rivals trigger on opposite sides, thread the needle.
        // The pass lane, plus its release. The release is the half that matters: a car that has just cleared the overlap
        // still has its rear beside the passed car, so the offset is held until that car is well behind rather than
        // dropped the tick the target clears.
        float ComputeAvoidanceLane(float roadWide)
        {
            // ZOMBIE (2026-10-25): the hold below is kept, not used. Its claim - that the lane snapped back and the car
            // turned in on the car it had just passed - was invalidated by build 934, driven with the hold off: ApplyRivalWalls
            // covers the nose clip on its own. Due to be revisited and removed if it truly is of no use.
            bool avoidHoldEnabled = false;
            float passLane = ComputeAvoidAheadLane(roadWide);
            if (!avoidHoldEnabled) return passLane;
            if (passLane != 0f)
            {
                _avoidHoldLane = passLane;
                _avoidHoldActive = true;
                _avoidHoldUntil = Game.GameTime + AvoidHoldTimeoutMs;
                return passLane;
            }
            if (!_avoidHoldActive) return 0f;
            // The hold is a pass in progress, not a lane the car owns: it dies with the pass. The geometric release below
            // needs a rival still alongside, and when there is none - a car that despawned, a race restarted, a snapshot
            // that says "alongside" for a rival already behind - nothing released it and the held lane outranked the corner
            // line for the rest of the race. This is the backstop, and Initialize/Launch clear it too.
            if (Game.GameTime > _avoidHoldUntil || Math.Abs(_avoidHoldLane) > roadWide)
            {
                _avoidHoldActive = false;
                return 0f;
            }

            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || !r.RivalRacer.Car.Exists()) continue;
                if (r.RouteGapAhead > 0f || r.RouteGapAhead < -AvoidHoldBehindMeters) continue;
                if (Math.Abs(r.NodeOffset - _carriedLane) > r.CombinedWidth + AvoidHoldLateralMeters) continue;
                return _avoidHoldLane;
            }

            _avoidHoldActive = false;
            return 0f;
        }

        float ComputeAvoidAheadLane(float roadWide)
        {
            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            float trackBound = roadWide - carHalfWidth;
            float aggroBuffer = ARS.Remap(Aggression, 100f, 0f, 0.2f, 1.2f, true);
            float nodeOffset = Brain.CurrentPerception.CurrentNodeOffset;

            Rival target = Brain.AvoidanceTarget;
            if (target == null || target.RivalRacer == null) { _passLaneLatched = false; return 0f; }

            // RULE 2 - once the two bodies are side by side the side is committed: there is no room to cross over without
            // touching, so the lane the pass began on is the lane it finishes on and the geometry is not asked again until
            // the lateral overlap ends. Latched per target: a different rival ahead starts a fresh decision. The lateral test
            // is deliberate - a car behind this one does not pin it, only one alongside.
            bool bodiesSideBySide = Math.Abs(target.NodeOffset - nodeOffset) < target.CombinedWidth;
            if (bodiesSideBySide && _passLaneLatched && _passLaneTarget == target.RivalRacer) return _passLaneValue;

            if (!TryPickAvoidanceSide(target, trackBound, aggroBuffer, carHalfWidth, nodeOffset, out float avoidanceLane, out bool targetGoLeft))
            {
                _passLaneLatched = false;
                return 0f;
            }

            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || r == target) continue;
                if (r.RelativePosition != RelativePos.Ahead) continue;
                if (!ARS.IsBetween(r.RouteGapAhead, 0f, SameSectionMaxGapMeters)) continue;
                if (!ARS.IsBetween(Math.Abs(r.DirectionDiff), 0f, AvoidAngleGateDegrees)) continue;
                if (!ARS.IsBetween(r.FrontGap, 0f, 3f) && !ARS.IsBetween(r.TimeToReach, 0f, 5f)) continue;

                if (!TryPickAvoidanceSide(r, trackBound, aggroBuffer, carHalfWidth, nodeOffset, out float secondTarget, out bool secondGoLeft))
                    continue;

                if (secondGoLeft == targetGoLeft) continue;
                _passLaneValue = (avoidanceLane + secondTarget) * 0.5f;
                _passLaneLatched = true;
                _passLaneTarget = target.RivalRacer;
                return _passLaneValue;
            }

            _passLaneValue = avoidanceLane;
            _passLaneLatched = true;
            _passLaneTarget = target.RivalRacer;
            return _passLaneValue;
        }

        bool TryPickAvoidanceSide(Rival rival, float trackBound, float aggroBuffer, float carHalfWidth, float nodeOffset, out float passLane, out bool passLeft)
        {
            passLane = 0f;
            passLeft = false;

            float rivalLane = rival.NodeOffset;
            // The rival's half width only: the lane built below adds the car's own half width, and CombinedWidth
            // already carries both of them (it is the average of the two full widths), so using it here counted the
            // car's body twice and left a full half width of air between the two.
            float buffer = rival.RivalRacer.VehicleData.BoundingBox * 0.5f + aggroBuffer;

            // Pass on the side the rival sits on relative to our predicted line (0.5s→1s projection segment).
            Vector3 predHalf = ProjectAhead(0.5f);
            Vector3 predDir = ProjectAhead(1f) - predHalf;
            float predLen = predDir.Length();
            if (predLen > 1f) predDir /= predLen;
            else predDir = Car.Velocity.Length() > 0.5f ? Car.Velocity.Normalized : Car.ForwardVector;
            float rivalSide = Vector3.Dot(Vector3.Cross(predDir, Vector3.WorldUp), rival.RivalRacer.Car.Position - predHalf);
            passLeft = rivalSide > 0f;

            passLane = passLeft ? rivalLane - buffer - carHalfWidth : rivalLane + buffer + carHalfWidth;

            // Side flip only on near-straights: route radius above the floor, never inside corners.
            // RULE 1 - no side change within a metre nose to tail: the crossing itself is the risk, because the rear of
            // the car ahead is still in the way while this one goes round. Further back the flip is free.
            const float SideFlipRadiusFloor = 200f;
            const float SideFlipMinClearanceMeters = 1f;
            bool canChangeSide = Math.Abs(rival.RouteGapAhead) >= SideFlipMinClearanceMeters;
            if (canChangeSide && Math.Abs(passLane) > trackBound && Brain.CurrentPerception.CurveRadiusToFollowPoint > SideFlipRadiusFloor)
            {
                passLane = passLeft ? rivalLane + buffer + carHalfWidth : rivalLane - buffer - carHalfWidth;
                passLeft = !passLeft;
            }

            if (Math.Abs(passLane) > trackBound) return false;

            if (passLeft && nodeOffset <= passLane) return false;
            if (!passLeft && nodeOffset >= passLane) return false;

            return true;
        }

        // One clamp site for the rival walls, and it bounds the request alone: the mover carries the aim onto the
        // request rather than placing it, so a wall that closes on the car cannot step the lane.
        float ClampLaneToWalls(float lane, float roadWide, bool leftConstrained, bool rightConstrained)
        {
            // The car's half width is the inset a RIVAL's wall needs; the track edge needs none, because the lane request is
            // already where the car's CENTRE may go. Adding it to an unconstrained wall made the plateau demand (+-halfWidth)
            // unsatisfiable by half a car on every tick, so the clamp rewrote the request and relabelled it WallClamp - which
            // then read in the log as the outside never firing.
            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            float trackBound = roadWide;
            float clampLeft = leftConstrained ? Math.Max(_avoidLeftWall + carHalfWidth, -trackBound) : Math.Max(_avoidLeftWall, -trackBound);
            float clampRight = rightConstrained ? Math.Min(_avoidRightWall - carHalfWidth, trackBound) : Math.Min(_avoidRightWall, trackBound);
            if (clampLeft > clampRight)
            {
                // The insets are what invert here, not the targets: the corridor has been narrowed below one car's width.
                // Returning the whole road is what made both walls vanish exactly when a rival sat on each side; collapsing to
                // the corridor's midpoint keeps the request between them, which is the best a car of this size can do.
                float collapsed = (clampLeft + clampRight) * 0.5f;
                return ARS.Clamp(collapsed, -trackBound, trackBound);
            }
            return ARS.Clamp(lane, clampLeft, clampRight);
        }

        float ApplyRivalWalls(float requestLane, float roadWide)
        {
            // The outer bound here is the raw track edge with no car-width inset. A rival's wall carries that rival's
            // half width, and ClampLaneToWalls adds this car's own, so the body edge lands on the other body's edge.
            float trackBound = roadWide;

            if (!_avoidWallsInitialized)
            {
                _avoidLeftWall = -trackBound;
                _avoidRightWall = trackBound;
                _avoidWallsInitialized = true;
            }

            float targetLeftWall = -trackBound;
            float targetRightWall = trackBound;
            bool leftConstrained = false;
            bool rightConstrained = false;

            // A pure fold over the rivals: Max on the left wall and Min on the right one are commutative, so the result
            // cannot depend on the order they are visited in, and no rival can open a wall another has closed.
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null) continue;

                // Only a body that is actually alongside constrains a wall, and the overlap is read on the ROUTE axis, the
                // same one rival detection uses: the car's own frame under-reads the gap for a rival at an angle and for every
                // corner, so a car beside this one could drop out of overlap without moving. One axis, and no same-section
                // guard: an arc shorter than a body cannot reach the far leg of a U.
                bool overlaps = Math.Abs(r.RouteGapAhead) < r.CombinedSize.Y;
                // TimeToReach already carries every gate that matters - ahead in arc, inside this car's lateral corridor and
                // actually closing - and returns infinity otherwise, so the approach needs no angle test of its own. The
                // angle and the RelativePos classification were what made the old branch flicker, not the approach itself.
                bool approaching = r.TimeToReach < WallApproachSeconds;
                if (!overlaps && !approaching) continue;

                float aggroBuffer = ARS.Remap(Aggression, 100f, 0f, 0.2f, 1.2f, true);
                // Same rule as the pass lane: ClampLaneToWalls adds the car's own half width on a constrained side, so
                // the wall carries the rival's half width alone and puts the car's body edge at the rival's edge.
                float rivalBuffer = r.RivalRacer.VehicleData.BoundingBox * 0.5f + aggroBuffer;

                // Which side of THIS car the rival is on, read in the car's own frame (+ is right). The track-centre sign
                // let a rival drifting across the road's midpoint swap walls in one tick, and a remembered side made the
                // answer depend on where the car had been. The deadband is deliberately narrow - only a rival on the
                // centreline counts as straight ahead - because a wide one sent rivals that were clearly to the right onto the
                // left wall, and that target then pushed past the far edge on a narrow road.
                bool rivalOnRight = r.LateralGap > WallSideDeadbandMeters;

                if (!rivalOnRight)
                {
                    targetLeftWall = Math.Max(targetLeftWall, r.NodeOffset + rivalBuffer);
                    leftConstrained = true;
                }
                else
                {
                    targetRightWall = Math.Min(targetRightWall, r.NodeOffset - rivalBuffer);
                    rightConstrained = true;
                }
            }

            // A wall is a lane offset inside the road and nothing else: neither a target nor a stored value may leave the
            // edges. Without this a rival near - or past - one edge produced a target beyond the opposite bound, the
            // corridor inverted, and the ordering guard wrote both walls off the road for a tick, then the rate pulled them
            // back: the one-tick spike the Show Rivalry lines showed.
            targetLeftWall = Math.Min(targetLeftWall, trackBound);
            targetRightWall = Math.Max(targetRightWall, -trackBound);

            // The ordering guard belongs on the TARGETS. Writing a collapsed corridor into the state is what made the pair
            // oscillate: the guard jumped both walls 1.6 m out in one tick, the mover pulled them back at its rate, the targets
            // re-crossed a few ticks later, and the whole thing repeated forever. A guard on the targets is a fixed point.
            if (targetLeftWall > targetRightWall)
            {
                float corridorHalf = Math.Min(1f, trackBound);
                float mid = ARS.Clamp((targetLeftWall + targetRightWall) * 0.5f, -trackBound + corridorHalf, trackBound - corridorHalf);
                targetLeftWall = mid - corridorHalf;
                targetRightWall = mid + corridorHalf;
            }

            _leftWallHeld = leftConstrained;
            _rightWallHeld = rightConstrained;
            _wallRoadWide = roadWide;
            // Two rates, one per direction. Opening is the comfort half at 5 m/s: slower than the car can unwind, so a wall
            // that has just let go is still the limit while the corner line starts asking. Closing runs at 10 m/s, but with a
            // jump: a wall further than one car width from where it belongs is teleported to a car width short of it first.
            // The guard is then immediate - it is never a car width late - while the only stretch the car can feel, the last
            // one, is always travelled at the rate instead of arriving in a single tick and snapping the aim.
            float wallCloseRate = 10f * TickScale;
            float wallOpenRate = 5f * TickScale;
            float wallSnapMeters = VehicleData.BoundingBox;
            // The stored walls are measured against the previous tick's width: a road that narrows leaves them outside the
            // new edge until the rate walks them in, which is a slow version of the same spike.
            _avoidLeftWall = Math.Min(_avoidLeftWall, trackBound);
            _avoidRightWall = Math.Max(_avoidRightWall, -trackBound);
            float leftTarget = leftConstrained ? targetLeftWall : -trackBound;
            float rightTarget = rightConstrained ? targetRightWall : trackBound;

            // Closing only, and only when the gap is wider than a car: the wall jumps to a car width from its target and
            // walks the rest. A wall that is opening is never teleported - that half has no urgency and its smoothness is
            // the whole point.
            if (leftTarget > _avoidLeftWall + wallSnapMeters)
                _avoidLeftWall = Math.Max(leftTarget - wallSnapMeters, -trackBound);
            if (rightTarget < _avoidRightWall - wallSnapMeters)
                _avoidRightWall = Math.Min(rightTarget + wallSnapMeters, trackBound);
            // The left wall closes by moving right (target above the wall), the right wall by moving left (target below it).
            float leftStep = leftTarget > _avoidLeftWall ? wallCloseRate : wallOpenRate;
            float rightStep = rightTarget < _avoidRightWall ? wallCloseRate : wallOpenRate;
            _avoidLeftWall = ARS.Clamp(leftTarget, _avoidLeftWall - leftStep, _avoidLeftWall + leftStep);
            _avoidRightWall = ARS.Clamp(rightTarget, _avoidRightWall - rightStep, _avoidRightWall + rightStep);
            _activeRivalWallCount = (_avoidLeftWall > -trackBound ? 1 : 0) + (_avoidRightWall < trackBound ? 1 : 0);

            return ClampLaneToWalls(requestLane, roadWide, leftConstrained, rightConstrained);
        }

        // Above the authored peak slip × this the car counts as sliding, so the damper drops its aim reference.
        const float SlidingFraction = 0.3f;
        // The blend ramps in across these multiples of the authored peak slip. The blend is linear from
        // peak × 0.5 (the slide is just past the traction peak) to peak × 0.8 (the countersteer is at full
        // authority), and the countersteer answers the full slide — the blend is the only modulator.
        const float SlideBlendStartFraction = 1f;
        const float SlideBlendFullFraction = 2.5f;
        // The share of the slide the countersteer answers. Half, not all: a wheel as wide as the slide itself
        // (33° of countersteer for 33° of slide) is what snaps the car the other way at the end of the correction.
        const float CountersteerSlideShare = 0.5f;
        // Half the slide recovers most cars; the ones it does not are the ones still rotating into the slide, so the
        // extra is a rate that only grows while the car keeps turning that way and is dropped the instant the rotation
        // turns back. The cap is a safety: a long slide must not wind the wheel to lock on a car that is already gone.
        // Two gates, not one: the extra grows whenever the blend is committed, and is reset only when the rotation turns
        // back. Reading the same test for both left a car sliding steadily - no rotation, so no growth - with less than
        // half the correction its slide asked for, indefinitely.
        const float CountersteerAssistDegreesPerSecond = 10f;
        // The ceiling is a share of the slide, not an angle: the blend and the assist are two halves of one question -
        // how much of the slide may the wheel answer. At 1.0 the wheel can reach the velocity vector and no further, which
        // is the neutral point; anything above it is a deliberate over-correction. 0.8 leaves it short of neutral.
        const float CountersteerTotalShare = 1f;
        // Pedal level held at full countersteer: enough throttle to keep the wheels rolling and no brake.
        // Full countersteer caps both pedals: the tyre is already spending its grip diagonally, so neither a full
        // brake nor a full throttle belongs in that state - but the plan keeps its own direction, up to this much.
        const float CountersteerPedalCap = 0.5f;
        // Below this forward speed the velocity direction is numerical noise, so the slide angle means nothing.
        const float CountersteerMinSpeedMph = 10f;
        // 120 deg/s, not 180: it still outruns the yaw the car can generate (a car rotating at 90 deg/s would out-turn the 90 deg/s this
        // used to be tried at), but it takes the edge off the small corrections the pursuit asks for. 4 deg a tick at 30 Hz.
        const float SteerSlewRate = 120f;

        // A cap on the steer command driven by the front tyres' measured side slip. While the car is on the throttle the
        // command may not push the fronts past this share of their own peak (the side-slip axis peaks at 1.0); the cap
        // falls at 2 deg/s and rises at twice that, so it lets go before it binds. The floor keeps the wheel steerable:
        // a cap that reached zero would drive the car straight out of the corner.
        // From this speed down the slip allowance starts doubling, and it is fully doubled at the second one.
        static readonly bool SlowCornerSlipAllowance = true;
        const float SlowCornerSlipFullMph = 15f;
        const float SlowCornerSlipDoubleMph = 5f;
        // How much of the damper's overshoot past neutral survives. 1.0 keeps the full overshoot when the
        // signs do not match: the damper is allowed to countersteer at full authority on a slide.
        const float DamperCrossingShare = 1f;
        // The damper's countersteer is a micro-adjustment by design: five degrees past neutral, no more. The term had no
        // bound of its own and the limiter opens its side to full lock while it pushes against the rotation, so without a
        // ceiling the stabiliser is the loosest source of opposite lock in the car.
        // The damper's authority against the car's own rotation. The slide batch drove it at 0.5 and the slides
        // answered, but that field read a smidge unstable, so the knob sits back at full — the slide response to keep
        // is the blend's band and share, with the crossing overshoot whole.
        const float DamperEffectScale = 1f;
        // Kill switch for the yaw-rate damper. Driven with it off the cars cannot hold centre — the term is the
        // only thing opposing a rotation the course chain has already started, so it is load-bearing, not trim.
        const bool SteerDampingEnabled = true;
        // The damper's reference: the yaw the aim point requires removes the standing-offset toll, and the zero
        // reference stays one flip away for A/B — the drive that preferred it was confounded by the course error.
        const bool SteerDampingAimReference = true;
        float SteerDampingFor()
        {
            // Grip is the only divisor: the gain used to fall as a reference speed over the car's above that speed,
            // and a full-authority dial damped cleanly at every speed, so the schedule was removed by direction.
            return SteerDampingEnabled ? ARS.SteerDampingGain / Math.Max(VehicleData.BaseMechanicalGrip, 1f) : 0f;
        }
        // Vanilla's player steering limiter used as a ceiling (AGENTS.md pipeline step 4): vanilla divides by
        // 1 + 0.075 × (forward speed − 5) in m/s and skips it while the car is sliding. The 5 m/s shift and its
        // gate are deliberately dropped here, so the ceiling starts closing from a standstill instead of
        // holding full lock to ~11 mph. Vanilla's auto-centre term beside it is not applied.
        const float VanillaSteerReductionPerMps = 0.075f;
        // Grip is read as a ratio to this reference, floored so a low-grip car cannot blow the coefficient up.
        const float SteerCapGripReference = 1f;
        const float SteerCapGripFloor = 0.5f;
        // The game scales one front wheel to this share of the command (TEMP_STEER_WHEEL_MULT, CWheel::SetSteerAngle,
        // wheel.cpp:6445). The outer wheel carries the corner's load, so the ceiling caps the command so that wheel
        // lands on the peak slip; the inner, steered to the full command, runs past it.
        const float EngineSteerWheelOuterShare = 0.75f;
        const float PeakSlipOuterCommandShare = 1f / EngineSteerWheelOuterShare;
        // Below the ramp's end speed the ceiling eases back to the car's full lock, so a slow car can steer in
        // fully; above it the ceiling is the law's own. The band is in mph because that is how it is judged.
        const float SteerLimitRampStartMph = 5f;
        const float SteerLimitRampEndMph = 30f;

        // Live ceiling coefficient: the useful steer angle at speed is ~ grip × g × wheelbase / v², so grip
        // belongs in that numerator and the cap loosens as √grip — the same √grip the speed maths uses.
        // Reads CurrentMechanicalGrip, not the base: BaseMechanicalGrip has the downforce term divided out
        // (1 + 0.035 × downforce) and only gets it back through Current, so a downforce car's base reads low.
        float SteerReductionPerMps
        {
            get
            {
                float grip = VehicleData.CurrentMechanicalGrip;
                if (float.IsNaN(grip)) grip = SteerCapGripReference;
                float gripRatio = Math.Max(grip / SteerCapGripReference, SteerCapGripFloor);
                return VanillaSteerReductionPerMps / (float)Math.Sqrt(gripRatio);
            }
        }

        // Deliberately the same grip and gravity that SteerLimitedSpeed uses, so the ceiling and the steer-limited
        // speed stay one law read two ways.
        float AckermannCeilingDegrees(float fwdSpeed)
        {
            float grip = VehicleData.CurrentMechanicalGrip;
            if (float.IsNaN(grip) || fwdSpeed <= 0f) return VehicleData.SteeringLock;
            float ratio = VehicleData.WheelBase * grip * Handling.Gravity / (fwdSpeed * fwdSpeed);
            if (float.IsNaN(ratio) || ratio <= 0f) return VehicleData.SteeringLock;
            return ARS.RadToDeg((float)Math.Atan(ratio));
        }

        // The authored fTractionCurveLateral is the ZERO-SPEED peak — the tyre stiffens with speed, which is what
        // LateralPeakAtSpeed models. Refreshed once per timed core so every consumer in a frame reads the same value.
        public float TRLateralAtSpeed = 22f;

        void UpdateTRLateralAtSpeed()
        {
            float speed = Car.Velocity.Length();
            TRLateralAtSpeed = float.IsNaN(speed) ? 0f : LateralPeakAtSpeed(speed);
        }

        // The tyre's peak slip angle at an arbitrary speed; the live field is this value at the current speed.
        float LateralPeakAtSpeed(float speed)
        {
            float lat = Handling.LateralTractionCurve;
            return lat <= 0.01f ? 0f : lat / (1f + Math.Min(5f, 0.1f * speed));
        }

        // One measurement of the front axle, filled once per tick and read by both the limiter and the readout, so the two can
        // never disagree about what the tyres are doing. Two sources fill it: the wheel memory (measured side slip) or the car's
        // own geometry against its grip (theoretical demand). Both speak the same 100 = at the target scale.
        public struct SlipState
        {
            public float InnerPct;   // the inner front tyre: the signal the limit acts on
            public float RearPct;    // the worse rear tyre, same scale: tells an understeer from a slide the front cannot see
            public bool Readable;    // false when this source could not answer this tick
        }

        SlipState MeasureFrontSlip(float fwdSpeed)
        {
            // Reverse is always the full lock, as the speed ceiling above already states: neither source answers going backwards,
            // where the inside of the corner is not even defined the same way.
            if (fwdSpeed <= 0f) return new SlipState();
            return ARS.SlipFromMemory ? MeasureSlipFromMemory(fwdSpeed) : MeasureSlipFromTheory(fwdSpeed);
        }

        // The measured source: the tyres' own side slip, in the field's units, whose peak is 1.0. No handling peak is needed to
        // compare them, which is the 1:1 fallback - without a readable peak the guard still knows where the limit is and only
        // the steered-angle ceiling is lost.
        // Slow corners: below 15 mph the car is asked to turn tighter than the peak allows, and holding the wheel down there just
        // makes it push wide. The allowance doubles by 5 mph, where the geometry needs the lock and the slip is harmless.
        float SlowCornerSlipShare(float fwdSpeed)
        {
            // ON: this is the only part of the law that moves with speed. Under 15 mph the effective cap share grows to 2.0 at 5 mph,
            // so the wheel is allowed past the tyre's own peak exactly where a hairpin needs it; above 15 mph it is identically 1.0.
            // The 2.0 plateau is deliberate: the curve is still falling at 2x the peak, and 2.5x is where it goes flat.
            if (!SlowCornerSlipAllowance) return 1f;
            float mph = ARS.MpsToMph(fwdSpeed);
            float slow = ARS.Clamp((SlowCornerSlipFullMph - mph) / (SlowCornerSlipFullMph - SlowCornerSlipDoubleMph), 0f, 1f);
            return 1f + slow;
        }

        SlipState MeasureSlipFromMemory(float fwdSpeed)
        {
            SlipState s = new SlipState();
            List<float> slips = ARS.WheelSideSlipAngles(Car);
            if (slips.Count < 2) return s;
            s.Readable = true;

            float rawLeft = Math.Abs(slips[0]);
            float rawRight = Math.Abs(slips[1]);

            // The INNER wheel is the signal: Ackermann steers it to the full command while the outer only takes
            // EngineSteerWheelOuterShare of it, so the inner reaches its peak first and is the tyre that warns. With the wheel
            // neutral there is no inside, so the pair is averaged; a positive command steers left, so a left turn's inside is 0.
            // The wheel that complains most, not a wheel chosen by the sign of the command: the index-to-side mapping was an
            // assumption nobody ever verified, and a wrong guess would make the two directions behave differently. Taking the
            // larger of the two is assumption-free, and in a corner the inner tyre is the one with the larger angle anyway.
            float rawInner = Math.Abs(Control.SteerDegrees) < FrontSlipNeutralSteerDeg
                ? 0.5f * (rawLeft + rawRight)
                : Math.Max(rawLeft, rawRight);

            // One owner for how much of the tyre's peak slip is allowed: the menu's Peak Slip Share. 1.00 cuts at the peak, below it
            // cuts before the peak, above it lets the tyre slide, and 2.00 is past anything but a real slide. Clamped because a
            // hand-edited ini could carry a zero and the division would poison the whole reading with NaN.
            float capShare = Math.Max(0.05f, ARS.SteerCeilingFactor) * SlowCornerSlipShare(fwdSpeed) * OpensUpSlipScale();
            s.InnerPct = 100f * rawInner / capShare;
            s.RearPct = slips.Count >= 4
                ? 100f * Math.Max(Math.Abs(slips[2]), Math.Abs(slips[3])) / capShare
                : 0f;
            return s;
        }

        // The theoretical source, no wheel memory: the radius the current command asks for gives the lateral acceleration the
        // front axle demands, and the ceiling is the command whose demand lands on a share of the grip the car actually has.
        // Both sides of the comparison are m/s^2, so there is no peak angle and no outer-wheel share to reconcile - the inner
        // wheel's tighter radius (half the track) is the only asymmetry, and it is the same one Ackermann puts on the command.
        SlipState MeasureSlipFromTheory(float fwdSpeed)
        {
            SlipState s = new SlipState();
            float gripGs = VehicleData.CurrentMechanicalGrip;
            float speed = Math.Abs(fwdSpeed);
            float wheelBase = Math.Max(VehicleData.WheelBase, 0.5f);
            // NaN fails every comparison, so the floors alone would let it through and poison the ceiling: test it first.
            if (float.IsNaN(speed) || float.IsNaN(gripGs) || float.IsNaN(wheelBase)) return s;
            if (speed < FrontSlipTheoryMinSpeedMps || gripGs <= 0.01f || wheelBase <= 0.5f) return s;

            s.Readable = true;
            float available = gripGs * FrontSlipGravity * Math.Max(0.05f, ARS.SteerCeilingFactor) * SlowCornerSlipShare(fwdSpeed) * OpensUpSlipScale();

            // The inner wheel is the signal, as in the measured source, and its angle IS the command: the file's own convention
            // gives the outer wheel 0.75 of it (EngineSteerWheelOuterShare), so the inner turns more and takes the tighter line.
            // Its radius is therefore L/tan(command) - not the axle-centre radius offset by half a track, which understated the
            // demand by 13-30% and hid exactly the saturation this source exists to see.
            float commandDeg = Math.Abs(Control.SteerDegrees);
            float innerRadius = commandDeg > FrontSlipTheoryMinCommandDeg
                ? wheelBase / (float)Math.Tan(commandDeg * (float)Math.PI / 180f)
                : float.MaxValue;
            float demand = speed * speed / Math.Max(innerRadius, 0.5f);

            s.InnerPct = 100f * demand / available;
            return s;
        }

        // Development experiment: with Open Up on Throttle on, the slip the limit allows shrinks with the pedal, from what it is today at
        // zero throttle down to OpensUpSlipFloorShare at full. The pedal read here is the previous tick's - it is computed after the wheel -
        // which is one frame against a rule that is meant to be driven by feel.
        const float OpensUpSlipFloorShare = 0.6f;

        float OpensUpSlipScale()
        {
            if (!ARS.OpenUpOnThrottle) return 1f;
            return 1f - (1f - OpensUpSlipFloorShare) * ARS.Clamp(Control.Throttle, 0f, 1f);
        }

        float GeometrySteerCeiling(float fwdSpeed)
        {
            float vanillaCeiling = VehicleData.SteeringLock / (1f + SteerReductionPerMps * fwdSpeed);
            return Math.Min(vanillaCeiling, AckermannCeilingDegrees(fwdSpeed));
        }

        float PeakSlipCeilingAt(float peakSlipDeg)
        {
            return ARS.Clamp(peakSlipDeg * PeakSlipOuterCommandShare, 0f, VehicleData.SteeringLock);
        }

        // The ceiling in force at this speed: the peak-slip cap, with the corner geometry as a fallback only when the
        // live peak reads unusable. Eased back towards full lock below the ramp's end speed, where it meets what
        // the car gets at that end speed anyway. The menu's SteerCeilingFactor scales the at-speed peak-slip cap
        // wherever it is read - the cap in force and the ramp's end anchor alike, so the band stays continuous as
        // the car crosses the ramp end - and it never scales the geometry fallback nor the ramp's low-speed end,
        // which is full lock.
        float ResolveSteerCeiling(float fwdSpeed)
        {
            // Parked by decision while the measured-slip factor is being evaluated: the speed-derived ceiling used to be the
            // only steering law, and the only way to judge the new one is to let it work alone. The law stays compiled, so
            // reviving it is this one flag. Full lock means the tyre is the limit, not a precomputed angle.
            if (!SpeedSteerCeiling) return VehicleData.SteeringLock;
            float ceiling = TRLateralAtSpeed > 0.01f ? PeakSlipCeilingAt(TRLateralAtSpeed) * ARS.SteerCeilingFactor : GeometrySteerCeiling(fwdSpeed);
            float speedMph = ARS.MpsToMph(fwdSpeed);
            if (speedMph >= SteerLimitRampEndMph) return ceiling;

            float endSpeed = ARS.MphToMps(SteerLimitRampEndMph);
            float endPeak = LateralPeakAtSpeed(endSpeed);
            float endCeiling = endPeak > 0.01f ? PeakSlipCeilingAt(endPeak) * ARS.SteerCeilingFactor : GeometrySteerCeiling(endSpeed);
            // max() keeps the ramp a raise only: the straight line sits a degree under the curved law near 25 mph.
            return Math.Max(ceiling, ManeuverRamp(speedMph, endCeiling));
        }

        // The speed-derived steer ceiling (peak slip angle at speed, scaled by the menu's SteerCeilingFactor). Off while the
        // measured-slip factor is on trial; with it off, nothing but the tyre's own slip limits the wheel.
        static readonly bool SpeedSteerCeiling = false;

        // Between the ramp's start and end speeds the ceiling eases up to the car's own lock, so a slow car can place
        // itself; at or above the end speed the law's own ceiling stands. A raise only, never a cut.
        float ManeuverRamp(float speedMph, float ceiling)
        {
            if (speedMph >= SteerLimitRampEndMph) return ceiling;
            // Descending *input* with ascending output, because Remap's own clamp inverts a descending output.
            return ARS.Remap(speedMph, SteerLimitRampEndMph, SteerLimitRampStartMph, ceiling, VehicleData.SteeringLock, true);
        }

        void ApplySteerLimits()
        {
            // NaN guard: Clamp would turn NaN into full-lock.
            if (float.IsNaN(Control.SteerDegrees) || float.IsInfinity(Control.SteerDegrees))
            {
                Control.SteerDegrees = 0f;
                return;
            }

            float requestedSteer = Control.SteerDegrees;
            float fwdSpeed = Vector3.Dot(Car.Velocity, Car.ForwardVector);

            // Two limits, one per side, closed by one clamp: the ceiling is granted and then raised on the answering
            // side rather than the clamp being skipped — except for the stabiliser below. Reverse is always the full
            // lock: backing up is placing the car, and the geometry law inverts below −13 m/s anyway.
            bool countersteering = Math.Sign(requestedSteer) != Math.Sign(VehicleData.YawRotationPerSecondDegrees);
            float speedCeiling = VehicleData.SteeringLock;
            if (fwdSpeed > 0f) speedCeiling = ResolveSteerCeiling(fwdSpeed);
            _slip = MeasureFrontSlip(fwdSpeed);
            _slipFactor = 1f;
            _clampTargetDeg = float.NaN;
            // maxSlip is the peak at this speed and it is the only thing that moves with speed. clampTarget is the old proportional
            // law - the request scaled by maxSlip/slip - so it carries the percentage of reduction, and it is recomputed every
            // tick. chasingClamp is the value actually applied and the only state in the system.
            if (_slip.Readable && _slip.InnerPct > 0.01f)
            {
                // One stop, the menu's Slip Cut Floor: it is the deepest the cut may go, and there is no second clamp behind it. At the
                // shipping 0.40 this is arithmetically the old 2.5x saturation (100/250 = 0.40), so the default changes nothing, but a
                // value below 0.40 now actually reaches the wheel instead of dying against a constant nobody could move.
                float factor = ARS.Clamp(100f / _slip.InnerPct, ARS.SlipCutFloor, 1f);
                _slipFactor = factor;
                // Only an overshoot lowers the target. Below it the gate has no business being under the lock, and a target that
                // tracked every small request collapsed the ceiling in the straights and then needed seconds to climb back - which is
                // exactly what held the wheel down on corner entry while the factor itself was barely cutting.
                float clampTarget = _slip.InnerPct > 100f
                    ? Math.Abs(requestedSteer) * factor
                    : VehicleData.SteeringLock;
                _clampTargetDeg = clampTarget;
                float chase = float.IsNaN(_chasingClampDeg) ? clampTarget : _chasingClampDeg;
                // The closing rate follows the severity of the overshoot, the opening is a flat rate: the harder the tyre is past
                // its peak the faster the gate shuts, and it always reopens carefully rather than snapping wide.
                if (clampTarget >= chase)
                {
                    _chasingClampDeg = Math.Min(clampTarget, chase + SlipOpenDegPerSec * TickScale);
                }
                else
                {
                    float gap = chase - clampTarget;
                    float step = Math.Min(gap * SlipClosePerSecond, SlipCloseMaxDegPerSec) * TickScale;
                    // No snap: the exponential closes the gap on its own and Math.Max stops the last step from crossing the target,
                    // so the tail is visible in the readout instead of jumping to 100 %.
                    _chasingClampDeg = Math.Max(clampTarget, chase - step);
                }
                speedCeiling = Math.Min(speedCeiling, _chasingClampDeg);
            }
            _finalCeilingDeg = speedCeiling;
            // LEFT bounds positive commands and RIGHT negative ones, because a positive command steers left.
            float steerLimitRight = speedCeiling;
            float steerLimitLeft = speedCeiling;

            // The one whitelisted allowance: the side answering a slide reaches past the ceiling towards the slide
            // angle itself, in proportion to how much of the blend that slide has earned.
            if (countersteering && _slidePriority > 0f)
            {
                float countersteerAllowance = Math.Min(Math.Abs(VehicleData.SlideAngle) * _slidePriority, VehicleData.SteeringLock);
                if (requestedSteer > 0f) steerLimitLeft = Math.Max(steerLimitLeft, countersteerAllowance);
                else steerLimitRight = Math.Max(steerLimitRight, countersteerAllowance);
            }

            float yawRate = VehicleData.YawRotationPerSecondDegrees;

            // The damper is the car's stabiliser and the rotation leads the slide, so while its term pushes against
            // the rotation it is answering a slide the ceiling cannot see yet: that side is not capped at all, after
            // the envelope has had its say. Both the term and the command must be against the rotation, so this is a
            // raise for a correction and never for steer-in, which shares the side.
            if (countersteering && _damperTermDeg * yawRate < 0f)
            {
                if (requestedSteer > 0f) steerLimitLeft = VehicleData.SteeringLock;
                else if (requestedSteer < 0f) steerLimitRight = VehicleData.SteeringLock;
            }

            Control.SteerDegrees = ARS.Clamp(requestedSteer, -steerLimitRight, steerLimitLeft);
            _steerPreLimitDegrees = requestedSteer;
            _steerGrantedCeiling = requestedSteer >= 0f ? steerLimitLeft : steerLimitRight;
        }


        // The share of the slide the countersteer answers is a constant — the blend is the only modulator.
        float CountersteerShare() => CountersteerSlideShare;


        // True when the slide has saturated the band at four fifths of the authored peak: the wheel blend is at its
        // full weight and the pedals hand over — no brake, and only the throttle that keeps the wheels rolling.
        // Forward speed gates it: reversing reads as a ~180° slide, and a reversed car must never be starved.
        bool IsFullCountersteer()
        {
            if (KickWantsSlide()) return false;
            if (Vector3.Dot(Car.Velocity, Car.ForwardVector) < ARS.MphToMps(CountersteerMinSpeedMph)) return false;
            return _slidePriority >= 1f;
        }

        // The kick card's wanted rotation: the slide the burst buys rotates toward the aim, so it carries the
        // delivered steer's own sign — anything opposite is a spin the ordinary machinery must catch. Math.Sign
        // throws on NaN, so an unreadable slide reads as unwanted and leaves the blend live.
        bool KickWantsSlide()
        {
            float slide = VehicleData.SlideAngle;
            if (float.IsNaN(slide) || float.IsInfinity(slide)) return false;
            return _kickPlaying && _kickSteerSign * Math.Sign(slide) > 0f;
        }


        public void Launch()
        {
            Brain.Corner = null;
            _nextApexCorner = null;
            ActiveManeuver.Type = ManeuverType.None;
            ActiveManeuver.Target = null;
            _divebombApexNode = -1;
            _defendApexNode = -1;
            NextApexNode = -1;
            NextApexRadius = 999f;
            NextApexSpeed = 999f;
            NextApexNode2 = -1;
            NextApexRadius2 = 999f;
            NextApexSpeed2 = 999f;
            NextApexNode3 = -1;
            NextApexRadius3 = 999f;
            NextApexSpeed3 = 999f;
            BaseBehavior = RacerBaseBehavior.Race;
            Lap = 1;
            _stuckArmAllowedTime = Game.GameTime + RecoveryArmDelayMs;
            _kickPlaying = false;
            _kickPlayingSince = 0;
            _kickWindowEnd = 0;
            _kickArmSince = 0;
            _kickCooldownUntil = 0;
            _kickSteerSign = 0;
            _kickAskDegrees = 0f;
            LapStartTime = ARS.IsPointToPoint ? Game.GameTime : 0;
            VehicleData.ResetLapPeaks();
            CanRegisterNewLap = false;
            _previousNode = -1;
            Pressure = 0f;
            // The grid is the one moment the car is known to be at rest, so this is where the baseline comes from.
            float restHeight = Car.HeightAboveGround;
            if (restHeight > 0f) _restHeightAboveGround = restHeight;
            Control.MaxThrottle = 1f;
            Control.MaxBrake = 1f;
            Control.MaxBrakeFromABS = 1f;
            Control.MaxBrakeFromRoute = 1f;
            Control.MaxBrakeFromCountersteer = 1f;
            Control.MaxThrottleFromTCS = 1f;
            Control.MaxThrottleFromStability = 1f;
            Control.MaxThrottleFromOverspeed = 1f;
            Control.MaxThrottleFromRival = 1f;
            Control.MaxThrottleFromChillOut = 1f;
            Control.MaxThrottleFromYield = 1f;
            Control.MaxThrottleFromOffTrack = 1f;
            Control.MaxThrottleFromRecovery = 1f;
            Control.ThrottleReason = ThrottleReason.Plan;
            // Nothing survives a re-initialise: a held pass lane and an armed inside guard both belong to a corner that is
            // not on this track any more, and read from the next race they override its first corners.
            _avoidHoldActive = false;
            _avoidHoldLane = 0f;
            _avoidHoldUntil = 0;
            _apexInsideNow = false;
            _apexLineInsideNode = -1;
            // The blend's ratchet is state too, and it is keyed by node INDEX: kept across a restart it can arm the inside extreme
            // on the first corner of the next race from an index that belongs to a different track.
            _linePhaseNode = -1;
            _linePhaseLatched = 0f;
            _linePhaseDir = 0f;
            // The outside bias is rolled per corner, so a restart must not inherit the previous race's pull-in.
            _lineBiasNode = -1;
            _lineCornerNode = -1;
            _lineClockNode = -1;
            _lineClockMps = 0f;
            Control.ThrottleReasonLevel = 1f;
            Control.BrakeReason = BrakeReason.Plan;
            Control.BrakeReasonLevel = 1f;
            _stuckPhase = StuckPhase.None;
            _stuckPhaseEndTime = 0;
            _stuckRecoveryStartTime = 0;
            _stuckStationarySince = 0;
            _stuckOffTrackSince = 0;
            _stuckAgainSince = 0;
            _stuckExitSince = 0;
            _stuckRecoveryCooldownEndTime = 0;
            _stuckMoveSampleTime = 0;
            Control.LastAppliedSteerDegrees = 0f;
            _laneSnap = true;
            if (TeamRole == Team.Cop) Car.SirenActive = true;

        }

        const float PedalSlewRate = 6f;             // pedal slew rate (units/second: full range in ~167ms)
        const float PedalSlewMaxPerTick = 0.5f;     // hitch guard: never step more than this in one tick

        // Development experiment (owner, 2026-10-26): a car already pulling more than half the lateral grip it has, with the
        // projected line heading for the inside of the road, may not lift off the throttle. The offset is measured from the
        // CAR's node and direction: the projection has a node of its own with its own direction, and offsets taken in two
        // frames put the line on the wrong side of the centre on noise. What the car itself is doing is not part of the test.
        //
        // The inside test is what makes it liveable: halfway through a 90 degree bend the projection already points down the
        // straight, the projected lane is not further inside than the car, and the rule lets go exactly where lifting would
        // cost the exit.
        // Development experiment (owner, 2026-10-26): a car already pulling more than half the lateral grip it has, with the
        // projected line heading for the inside of the road, may not lift off the throttle. The offset is measured from the
        // CAR's node and direction: the projection has a node of its own with its own direction, and offsets taken in two
        // frames put the line on the wrong side of the centre on noise. What the car itself is doing is not part of the test.
        //
        // The floor is a ramp, not a switch: it scales with how deep the projection sits toward the inside, from nothing at the
        // track centre to the full GsLaneThrottleMaxFloor at the inside edge. Past the centre toward the outside it stays zero,
        // where the off-track limit already lives. Halfway through a 90 degree bend the projection points down the straight,
        // the inside test fails and the rule lets go exactly where lifting would cost the exit.
        // One colour for the whole trajectory, decided from the numbers the tick already computed:
        //   green to white - the inside floor is holding, scaled by how hard (green at the inside edge, white as it fades);
        //   red            - the path leaves the road and runs further out than the car, or drifts out with the car on the road;
        //   yellow         - it leaves the road but cuts inside the car;
        //   blue           - nothing to say.
        // Green wins because the floor is the reason the line is on screen; the two families cannot overlap, since the floor
        // returns zero while the off-track clock is running.
        Color ProjectionPathColour(float gsFloor, bool insideRegime)
        {
            if (insideRegime)
            {
                float share = ARS.Clamp(gsFloor / GsLaneThrottleMaxFloor, 0f, 1f);
                int dim = (int)(255f * (1f - share));
                return Color.FromArgb(255, dim, 255, dim);
            }
            bool outside = _projectionOutsideMeters > 0f;
            if (outside && _projectionWidening) return Color.Red;
            if (outside) return Color.Yellow;
            if (_projectionWidening) return Color.Red;
            return Color.Blue;
        }

        // The share of the theoretical lateral Gs the car must already be pulling before the floor holds any throttle. A third, so
        // the floor is live through the whole of a committed corner instead of only its loaded half.
        const float GsLaneFloorLateralShare = 0.33f;

        float GsLaneThrottleFloorValue(out bool insideRegime)
        {
            insideRegime = false;
            if (!ARS.GsLaneThrottleFloor) return 0f;
            // The off-track system owns the pedal while its clock runs: this floor works the inside of the road and must
            // never lift a throttle the projection is pulling down, nor stand in front of its brake. Two laws on one
            // pedal is one too many, and the outside one has seniority because it is the one that knows the plan failed.
            if (_projectionOutsideSeconds > 0f) return 0f;
            float theoreticalGs = VehicleData.CurrentMechanicalGrip * Handling.Gravity / 9.8f;
            if (!(theoreticalGs > 0f)) return 0f;
            if (Math.Abs(VehicleData.GetLateralGs(Car.ForwardVector)) <= GsLaneFloorLateralShare * theoreticalGs) return 0f;
            // The two conventions point opposite ways and mixing them cost a build of white lines: TrackPoint.Angle is
            // SignedAngle(previous, next, WorldUp), so POSITIVE is a LEFT turn, while SignedLaneOffset is built on
            // Cross(direction, WorldUp), so POSITIVE is the RIGHT of travel. MeasureProjectionOutside pins the pair by
            // calling sign(offset) == sign(Angle) the OUTSIDE, which leaves the inside on the negative side of a left-hander.
            // The side still comes from the car's node, as the owner asked - only its sign is flipped.
            int insideSign = -Math.Sign(CurrentTrackPoint.Angle);
            if (insideSign == 0) return 0f;
            // Every gate above is about the car and the moment, not about where the line points: passing them is what makes
            // the regime "live", and it is reported even when the line sits at the centre and the floor is therefore zero.
            // Without that distinction the gradient had no white end - the value that paints white was the value that
            // switched the branch off (2026-10-26).
            insideRegime = true;
            // The side comes from the CAR's corner - the direction of the turn is the car's business - but the distance comes
            // from the node the projection actually falls on. Measured against the car's own node the two get mixed: half a
            // second at 30 m/s is 15 m ahead, which in a 30 m radius is 14 degrees round the corner and about 3.6 m of fake
            // lateral offset. That is what made the depth collapse or saturate depending on where the apex was - a colour
            // that went white deep inside the inside.
            Vector3 projected = ProjectAhead(SteerPreviewSeconds);
            int projectedWindow = (int)(Car.Velocity.Length() * SteerPreviewSeconds) + 10;
            TrackPoint projectedNode = ARS.FindNearestTrackPoint(projected, CurrentTrackPoint.Node, 0, projectedWindow);
            float projectedOffset = ARS.SignedLaneOffset(projected, projectedNode.Position, projectedNode.Direction);
            // Room to the inside measured in metres: the centre is zero, the inner edge is 1, and the car's own width is not
            // lane the projection can use.
            float room = Math.Max(projectedNode.TrackHalfWidth - VehicleData.ModelDimensions.X * 0.5f, 1f);
            float depth = ARS.Clamp(projectedOffset * insideSign / room, 0f, 1f);
            return GsLaneThrottleMaxFloor * depth;
        }

        void ConvertSpeedToPedals()
        {
            float currentForwardSpeed = VehicleData.SpeedVectorLocal.Y;
            float inputChange = Math.Min(PedalSlewRate * TickScale, PedalSlewMaxPerTick);
            float newThrottle = 0f;
            float newBrake = 0f;

            FindLowestIntendedSpeed();

            Brain.CurrentIntention.IntendedSpeedChange = Brain.CurrentIntention.Speed - currentForwardSpeed;

            float intendedSpeedChange = Brain.CurrentIntention.IntendedSpeedChange;

            float combinedInput = ComputeCombinedInput(intendedSpeedChange);
            // The throttle follows the tyre's own curve, whose useful band is x1 to x2.5: the game's traction coefficient peaks at
            // 1.0 of the slip angle and reaches its floor at 2.5 (sfTractionPeakAngle / sfTractionMinAngle, wheel.cpp), after which
            // more slip changes nothing. So the pedal closes across that band and never below zero: the brake stays the plan's.
            if (_slip.Readable && _slip.InnerPct > 100f)
            {
                float over = ARS.Clamp((_slip.InnerPct - 100f) / 150f, 0f, 1f);
                combinedInput = Math.Min(combinedInput, Math.Max(0f, 1f - over));
            }
            MeasureProjectionOutside(1f);
            // Full countersteer: no brake, just enough throttle to keep the wheels rolling. The reason caps still
            // apply on top; the release is instant.
            bool countersteering = IsFullCountersteer();
            SplitCombinedInput(combinedInput, ref newThrottle, ref newBrake);
            if (countersteering)
            {
                newThrottle = Math.Min(newThrottle, CountersteerPedalCap);
                newBrake = Math.Min(newBrake, CountersteerPedalCap);
            }
            // The projection is the only thing that slows the car while it is still on the track: the distance
            // ramp that used to sit beside this clock asked for the same pedal from the other direction, and two
            // laws on one pedal is one too many. Time is the escalation — full throttle, coast, then brake — and
            // full countersteer is the one branch it yields to: the tyre there is already working diagonally, so
            // brake on top saturates the circle instead of sharing it.
            float projectionSpeedMph = ARS.MpsToMph(Car.Velocity.Length());
            float projectionTargetMph = Math.Max(OffTrackSpeedFloorMph, ARS.MpsToMph(RouteIdealSpeedForRadius(Brain.CurrentPerception.CurveRadiusToFollowPoint)) - OffTrackSpeedSlackMph);
            bool projectionOutside = _projectionWidening && _projectionOutsideMeters > 0f && projectionSpeedMph > projectionTargetMph;
            _projectionOutsideSeconds = projectionOutside ? _projectionOutsideSeconds + TickScale : 0f;
            float projectionInput = Math.Max(1f - ProjectionInputPerSecond * _projectionOutsideSeconds, -ProjectionBrakeFloor);
            bool projectionBinds = !countersteering && newThrottle > Math.Max(projectionInput, 0f);
            newThrottle = ComposeThrottleCap(newThrottle, projectionBinds, countersteering);
            _offtrackThrottleHeld = projectionBinds;
            // Brake composes here on the raw split target, pre-slew: in the throttle-to-brake transition
            // Control.Throttle slews down slowly and could still read >= 1.0 while the car is already braking.
            newBrake = ComposeBrakeCap(newBrake, countersteering);
            // The projection outranks the plan and the route: it is the only term that knows the plan has already
            // failed. It does not outrank the ABS, which is the physical ceiling this brake floor exists to respect,
            // nor the countersteer release above, which is already spending the tyre in two directions at once.
            if (!countersteering)
            {
                if (projectionInput < 0f)
                {
                    float projectionBrake = Math.Min(-projectionInput, Control.MaxBrakeFromABS);
                    if (projectionBrake > newBrake)
                    {
                        newBrake = projectionBrake;
                        Control.BrakeReason = BrakeReason.Projection;
                        Control.BrakeReasonLevel = projectionBrake;
                    }
                }
                else if (newThrottle > projectionInput) newThrottle = projectionInput;
            }
            // Artificial rubberbanding no longer borrows the pedal: a throttle above 1.0 is a number the engine cannot
            // apply, so its boost arrived only through the power compensation and the two features were multiplying.
            // It now sets the power outright, in ApplyPowerMultiplier, next to the burn.

            Control.Brake += ARS.Clamp(newBrake - Control.Brake, -inputChange, inputChange);
            Control.Throttle += ARS.Clamp(newThrottle - Control.Throttle, -inputChange, inputChange);
            // The development floor bounds this pedal and nothing else: the brake keeps whatever the plan and the projection
            // asked for, because the rule is about not lifting, not about not slowing.
            float gsFloor = GsLaneThrottleFloorValue(out bool gsInsideRegime);
            // The floor asks the same question as any other cap - does the tyre allow it? - so it composes through the ceiling
            // ComposeThrottleCap has just built (TCS, stability, overspeed, rival, offtrack, recovery) and stands down entirely
            // while the countersteer release is working. Those are physics, not intentions; the floor is a preference about the
            // line and has no business outvoting a spinning tyre.
            if (countersteering) gsFloor = 0f;
            gsFloor = Math.Min(gsFloor, Control.MaxThrottle);
            float throttleBeforeFloor = Control.Throttle;
            if (gsFloor > 0f) Control.Throttle = Math.Max(Control.Throttle, gsFloor);
            _throttleFloorGain = Control.Throttle - throttleBeforeFloor;
            _gsFloorActiveThisTick = Control.Throttle > throttleBeforeFloor;
            // Development read-out, on the focused racer and only while the toggle is on: what the floor added, in percent of
            // full throttle, measured against everything that had already had its say.
            //
            // It cannot live on the player's car: this whole method only runs under !ControlledByPlayer, because the human
            // drives that one. The floor is an AI rule, so the number has to be read on an AI - the same car whose trajectory
            // line is on screen.
            if (ARS.GsLaneThrottleFloor && ARS.DebugFocusRacer == this)
                UI.ShowSubtitle("Gs floor: +" + (100f * _throttleFloorGain).ToString("0") + "% throttle", 50);
            _projectionPathColor = ProjectionPathColour(gsFloor, gsInsideRegime);

            UpdateBrakeLearning();

        }

        // One linear map for both pedals: a direction swap needs no case of its own, because its error is the sum of the magnitudes and saturates anyway.
        float ComputeCombinedInput(float intendedSpeedChange)
        {
            float divisor = intendedSpeedChange < 0f ? FullPedalSpeedErrorMps * BrakeErrorMultiplier : FullPedalSpeedErrorMps;
            return ARS.Clamp(intendedSpeedChange / divisor, -1f, 1f);
        }

        void SplitCombinedInput(float combinedInput, ref float newThrottle, ref float newBrake)
        {
            if (combinedInput > 0f)
            {
                newThrottle = combinedInput;
                return;
            }

            if (combinedInput < 0f) newBrake = -combinedInput;
        }



        void FindLowestIntendedSpeed()
        {
            if (Brain.CurrentIntention.Speed >= 0f)
            {
                Brain.CurrentIntention.Speed = Math.Min(Brain.CurrentIntention.Speed, AiConstants.MaxSpeed);
            }
        }

        // How far the one-second projection has left the track, in metres beyond the edge, for the clock that decides
        // how much pedal to take. Zero unless the projection is heading further from the centre than the car already
        // is: a car coming back in gets no response however wide it currently sits. It needs the car moving and the
        // node actually turning; it does not need a carried lane, because the projection is pure kinematics.
        void MeasureProjectionOutside(float seconds)
        {
            bool laneOverride = _laneOverride;
            float mph = ARS.MpsToMph(Car.Velocity.Length());
            float signedOffset = 0f;
            _projectionOutsideMeters = 0f;
            _projectionWidening = false;
            float halfWidth = 0f;
            float outsideOffset = 0f;
            float carOffset = 0f;
            float turnDirection = 0f;

            if (mph >= OffshootMinSpeedMph)
            {
                Vector3 proj = ProjectAhead(seconds);
                // 1 node ≈ 1 m: the window has to reach the projected node.
                int projectionWindow = (int)(Car.Velocity.Length() * seconds) + 10;
                TrackPoint tp = ARS.FindNearestTrackPoint(proj, CurrentTrackPoint.Node, 0, projectionWindow);
                signedOffset = ARS.SignedLaneOffset(proj, tp.Position, tp.Direction);
                halfWidth = Math.Max(tp.TrackHalfWidth, 0.1f);

                // Outside is the side the corner the car is in turns away from.
                turnDirection = Math.Abs(CurrentTrackPoint.Angle) > OffshootMinRouteAngleDeg ? Math.Sign(CurrentTrackPoint.Angle) : 0f;

                if (Math.Sign(signedOffset) == turnDirection)
                {
                    outsideOffset = signedOffset * turnDirection;
                    // The car's own offset is read in its own node's frame, not the projected one: measured against a
                    // frame tens of nodes ahead, a corner turns the along-track separation into a fake lateral offset
                    // and the comparison below can never fail. Same frame as turnDirection, so the sides compare.
                    carOffset = ARS.SignedLaneOffset(Car.Position, CurrentTrackPoint.Position, CurrentTrackPoint.Direction) * turnDirection;
                    // The distance is a fact and the colour reads it; the clock is the thing that must not fire on a
                    // car already coming back, so widening is tracked beside it rather than folded into it.
                    _projectionOutsideMeters = outsideOffset - halfWidth;
                    _projectionWidening = outsideOffset > carOffset;
                }
            }

            // Every gate in one line, twice a second while the projection is on screen: guessing which one is shut
            // has already cost two builds.
            if (ARS.DebugToggles[Options.ShowGsProjection] && ARS.DebugFocusRacer == this && Game.GameTime >= _offshootLogAt)
            {
                _offshootLogAt = Game.GameTime + 500;
                ARS.Log(ARS.LogImportance.Info, "Offshoot: override=" + laneOverride + " mph=" + mph.ToString("0") + " turn=" + turnDirection + " signed=" + signedOffset.ToString("0.00") + " car=" + carOffset.ToString("0.00") + " half=" + halfWidth.ToString("0.00") + " outside=" + outsideOffset.ToString("0.00") + " sec=" + _projectionOutsideSeconds.ToString("0.00") + " input=" + (Math.Max(1f - ProjectionInputPerSecond * _projectionOutsideSeconds, -ProjectionBrakeFloor)).ToString("0.00"));
            }

        }

        // True when the one-second projection sits closer to the track's centre line than the car does, each measured
        // in its own node's frame — both are distances from the centre at their own point, so they compare directly.
        // It answers "is this car being steered back in", which is the only case where the off-track limiter should
        // give way to a recovery throttle.
        bool ProjectionClosingOnCentre()
        {
            Vector3 proj = ProjectAhead(1f);
            // 1 node ≈ 1 m, and the projection is one second out.
            int projectionWindow = (int)(Car.Velocity.Length()) + 10;
            TrackPoint tp = ARS.FindNearestTrackPoint(proj, CurrentTrackPoint.Node, 0, projectionWindow);
            float carOffset = Math.Abs(ARS.SignedLaneOffset(Car.Position, CurrentTrackPoint.Position, CurrentTrackPoint.Direction));
            float projectionOffset = Math.Abs(ARS.SignedLaneOffset(proj, tp.Position, tp.Direction));
            return projectionOffset < carOffset;
        }

        // Samples braking quality across the approach to the current apex. The verdict waits: the sample is parked
        // at the apex pass and only lands a couple of seconds later, once the exit has had its say.
        void UpdateBrakeLearning()
        {
            if (!ARS.BrakeLearning)
            {
                _brakeCommitApexNode = -1;
                return;
            }

            // Deferred commit: still watching, now for the slide a too-hot entry shows on the way out.
            if (_brakeCommitApexNode >= 0)
            {
                float exitSlide = Math.Abs(VehicleData.SlideAngle);
                if (!float.IsNaN(exitSlide) && exitSlide > _brakeCommitMaxSlideDeg) _brakeCommitMaxSlideDeg = exitSlide;
                if (Game.GameTime >= _brakeCommitAtGameTime) CommitBrakeLearning();
            }

            // The apex this sample belongs to, reset here rather than after the gates below, so a corner change
            // during a maneuver or past its own entrance cannot leave the sample pointing at an older apex.
            if (NextApexNode != _brakeSampleApexNode)
            {
                // The previous apex has just gone past, so this is the frame where the passed corner is still known by name. The reading comes
                // from the closest approach tracked while approaching, not from here: by now the apex is tens of metres behind, and the
                // perpendicular to its old tangent means nothing in a tight corner.
                // Fires once per apex, and only when that apex really went past: a picker that oscillates between candidates, or drops to -1 and
                // comes back, changes the sample several times over one corner and used to announce each time.
                if (_brakeSampleApexNode >= 0 && _brakeSampleApexNode != _brakeLastApexAnnounced
                    && _brakeSampleApexClosest < float.MaxValue && HasPassedApex(_brakeSampleApexNode))
                {
                    _brakeLastApexAnnounced = _brakeSampleApexNode;
                    AnnounceBrakeAdjustment("Apex " + _brakeSampleApexGap.ToString("0.0") + "m");
                }
                _brakeSampleApexNode = NextApexNode;
                _brakeSampleMaxSlideDeg = 0f;
                _brakeSampleWasAirborne = false;
                _brakeSampleEntryMetres = 0f;
                _brakeSampleEntryMeasured = false;
                _brakeSampleSawAboveApex = false;
                _brakeSampleApexSpeed = Math.Max(2f, NextApexSpeed);
                _brakeSampleBelowTicks = 0;
                _brakeSampleEntryHasCandidate = false;
                _brakeSampleFloorAtEntrance = false;
                _brakeSampleEntranceCrossed = false;
                _brakeSampleApexClosest = float.MaxValue;
                _brakeOverNextSampleAt = 0;
                // A corner that never reaches its apex loses its entrance cut here, with the sample. That is
                // deliberate: one verdict per corner and one exit from the commit. Settling the cut on the spot
                // would be a second place the factor is written, judged without the slide and the pedal share.
            }


            // Room to the edge at the closest approach to the apex: one comparison, one subtraction, read where the car is nearest the apex.
            if (_brakeSampleApexNode >= 0)
            {
                TrackPoint apexTrack = ARS.TrackPoints[_brakeSampleApexNode];
                float approach = Car.Position.DistanceTo(apexTrack.Position);
                // Only from close range: at the start of a race the tracker sits on the route's last node, and the lane offset to a distant
                // apex is a chord of hundreds of metres. That is where the -129 m came from.
                if (approach < BrakeApexNearMetres && approach < _brakeSampleApexClosest)
                {
                    _brakeSampleApexClosest = approach;
                    _brakeSampleApexGap = apexTrack.TrackHalfWidth - Math.Abs(ARS.SignedLaneOffset(Car.Position, apexTrack.Position, apexTrack.Direction));
                }
            }

            // Sampled on the learner's own clock, and the air penalty is checked over the whole corner rather than inside the braking
            // window: a car that takes off has already passed its braking target, which is exactly where the window ends.
            if (_brakeSampleApexNode >= 0 && !HasPassedApex(_brakeSampleApexNode) && Game.GameTime >= _brakeOverNextSampleAt)
            {
                _brakeOverNextSampleAt = Game.GameTime + BrakeOverSampleMs;
                // The ramp carries the pedal law's own brake error: a proportional law needs a sustained error to hold any decel, so a car
                // braking at the limit sits about BrakeErrorMultiplier above the bare ramp by design, and 1.5x that is where the floor of
                // the comparison sits before "over" means anything.
                float ramp = ApexBrakingSpeed(_brakeSampleApexNode, NextApexSpeed) + BrakeErrorMultiplier * BrakeOverAllowanceShare;
                float carried = ARS.GetForwardSpeed(Car);
                bool over = ramp < 100f && carried > ramp;

                // Airborne and still over the ramp: recorded for the notice, never charged to the factor.
                if (over && Car.IsInAir) _brakeSampleWasAirborne = true;

            }

            // Where the car actually reaches the corner speed, measured against its braking target: positive is late, negative is early,
            // zero is the corner taken exactly where the plan asked. A position rather than a speed difference, so the pedal law's
            // steady error - it needs up to BrakeErrorMultiplier above the target to hold full brake - cannot contaminate the reading.
            // The reference is the speed frozen when the sample opened, and the crossing takes three ticks in a row below it: the speed
            // signal crosses a live threshold several times per corner on its own noise, and one tick of it was enough to place the mark
            // a hundred metres out. The car also has to have been above it first - one already slow out of the previous corner never
            // crossed it, and has nothing to say.
            if (!_brakeSampleEntryMeasured && _brakeSampleApexNode >= 0 && !HasPassedApex(_brakeSampleApexNode))
            {
                // The bar is the corner speed plus the error the pedal law needs to reach full brake: below it the car is asking for the
                // whole pedal, and that is the moment worth marking. Same allowance the over-clock uses, one place.
                float apexSpeed = _brakeSampleApexSpeed + BrakeErrorMultiplier;
                float forward = ARS.GetForwardSpeed(Car);
                if (forward <= apexSpeed + 0.2f && _brakeSampleSawAboveApex)
                {
                    // Marked on the FIRST tick below, then confirmed by the next two: waiting for three ticks before taking the position
                    // would push the mark late by v * 0.05 s - two metres and a half at 180 km/h - and that bias would grow with speed,
                    // which is exactly the sort of inconsistency this reading is meant not to have.
                    _brakeSampleBelowTicks++;
                    if (_brakeSampleBelowTicks == 1)
                    {
                        // The corner is looked up by apex node, exactly as ApexBrakingSpeed does: _nextApexCorner can be an older corner
                        // whose entrance sits tens of metres away, and the target would then be placed from that one.
                        CornerPoint corner = ARS.Corners.FirstOrDefault(c => c.Node == _brakeSampleApexNode);
                        int entranceNode = CornerEntranceNode(corner, _brakeSampleApexNode);
                        if (entranceNode >= 0)
                        {
                            // Signed from the entrance: negative is the mark sitting before it, positive is the car still on the bar past it.
                            float metres = CurrentTrackPoint.CumulativeDistance - ARS.TrackPoints[entranceNode].CumulativeDistance;
                            // The wrap the rest of the route code applies: on a circuit an entrance behind the car reads as most of a lap.
                            if (!ARS.IsPointToPoint && ARS.RouteLengthMeters > 0f && Math.Abs(metres) > ARS.RouteLengthMeters * 0.5f)
                                metres -= Math.Sign(metres) * ARS.RouteLengthMeters;
                            _brakeSampleEntryMetres = metres;
                            _brakeSampleEntryHasCandidate = true;
                        }
                    }
                    else if (_brakeSampleBelowTicks >= 3 && _brakeSampleEntryHasCandidate) _brakeSampleEntryMeasured = true;
                }
                else
                {
                    _brakeSampleBelowTicks = 0;
                    _brakeSampleEntryHasCandidate = false;
                    if (forward > apexSpeed + 1f) _brakeSampleSawAboveApex = true;
                }
            }

            // Whether the lane floor was holding throttle on the way into this corner. It is latched over every tick of the last second of
            // travel before the entrance, not read on the crossing frame: a single tick samples a rule that ramps with depth, so ON at the
            // entrance can be OFF a metre earlier and the verdict would turn on when it happened to be sampled.
            if (!_brakeSampleEntranceCrossed && _brakeSampleApexNode >= 0)
            {
                CornerPoint entranceCorner = ARS.Corners.FirstOrDefault(c => c.Node == _brakeSampleApexNode);
                int entranceNode = CornerEntranceNode(entranceCorner, _brakeSampleApexNode);
                if (entranceNode >= 0)
                {
                    float toEntrance = ARS.TrackPoints[entranceNode].CumulativeDistance - CurrentTrackPoint.CumulativeDistance;
                    if (!ARS.IsPointToPoint && ARS.RouteLengthMeters > 0f && Math.Abs(toEntrance) > ARS.RouteLengthMeters * 0.5f)
                        toEntrance -= Math.Sign(toEntrance) * ARS.RouteLengthMeters;
                    // The pedal state is one tick old (the pedals are computed after this method), which against a one-second window is nothing.
                    if (toEntrance > 0f && toEntrance <= Car.Velocity.Length() * BrakeFloorLatchSeconds && _gsFloorActiveThisTick)
                        _brakeSampleFloorAtEntrance = true;
                    if (toEntrance <= 0f) _brakeSampleEntranceCrossed = true;
                }
            }
            // Runs on the corner proper, entrance to apex — exactly where the sample has stopped. The slide is only watched while the car
            // is being asked for more than BrakeSkidBrakeShare of brake: a car that slides under light pedal is being carried too fast,
            // and that is the speed system's error, not the braking point's.
            if (_brakeSampleApexNode >= 0 && HasPassedBrakingTarget() && !HasPassedApex(_brakeSampleApexNode)
                && Control.Brake > BrakeSkidBrakeShare)
            {
                float slide = Math.Abs(VehicleData.SlideAngle);
                if (!float.IsNaN(slide) && slide > _brakeSampleMaxSlideDeg) _brakeSampleMaxSlideDeg = slide;
            }

            // A passed corner always gets its verdict, whatever the held queue did this tick. A corner-table rebuild or a car slowed to
            // half the apex speed invalidates that queue, and the notice used to be swallowed with it - which is exactly what made the
            // reports look random. Every corner reports now, and a corner with no mark says so instead of staying silent.
            if (_brakeCommitApexNode < 0 && _brakeSampleApexNode >= 0 && HasPassedApex(_brakeSampleApexNode))
                ScheduleBrakeCommit();

            if (ActiveManeuver.Type != ManeuverType.None) return;

            if (HasPassedBrakingTarget()) return;

        }

        // Park the sample at the apex pass; the verdict lands BrakeCommitDelaySeconds later.
        void ScheduleBrakeCommit()
        {
            // Already parked for this apex: the leapfrog and the sample can both ask in the same tick, and the second must not
            // commit what the first only just scheduled - the slide window would be cut to a single frame.
            if (_brakeCommitApexNode >= 0 && _brakeCommitApexNode == _brakeSampleApexNode) return;
            // Two apexes inside the delay (a chicane) settle the older one now rather than lose it.
            if (_brakeCommitApexNode >= 0) CommitBrakeLearning();
            _brakeCommitApexNode = _brakeSampleApexNode;
            _brakeCommitAtGameTime = Game.GameTime + (int)(BrakeCommitDelaySeconds * 1000f);
            _brakeCommitMaxSlideDeg = _brakeSampleMaxSlideDeg;
            _brakeCommitWasAirborne = _brakeSampleWasAirborne;
            _brakeCommitEntryMetres = _brakeSampleEntryMetres;
            _brakeCommitEntryMeasured = _brakeSampleEntryMeasured;
            _brakeCommitSawAboveApex = _brakeSampleSawAboveApex;
            _brakeCommitFloorAtEntrance = _brakeSampleFloorAtEntrance;
            _brakeCommitEntranceCrossed = _brakeSampleEntranceCrossed;
        }

        void CommitBrakeLearning()
        {
            if (!ARS.BrakeLearning || _brakeCommitApexNode < 0) return;
            int apexNode = _brakeCommitApexNode;
            float maxSlideDeg = _brakeCommitMaxSlideDeg;
            _brakeCommitApexNode = -1;
            float before = BrakeFactorForApex(apexNode);
            bool skid = TRLateralAtSpeed > 0.01f && maxSlideDeg > TRLateralAtSpeed * BrakeSkidPeakMultiple;
            // The notice reads the verdict, not the numbers behind it: how far from the entrance the corner speed was reached, and what that
            // cost the factor. Events that are not the braking point's - a held floor, a jump - are named but never charged.
            //
            // A slide seen while the pedal was asking for more than BrakeSkidBrakeShare of brake is the braking point's own fault: a
            // rear-engined car loses its rear at that demand. The position verdict is thrown away and the factor pays the flat cut.
            if (skid)
            {
                float skidFactor = ARS.Clamp(before - BrakeSkidFactorStep, BrakeMinFactor, BrakeMaxFactor);
                if (BrakeLearningApplies) CornerContextForWrite(apexNode).BrakeFactor = skidFactor;
                AnnounceBrakeAdjustment("Skid detected, assuming instability adjust " + (skidFactor - before).ToString("+0.00;-0.00"));
                return;
            }
            if (!_brakeCommitEntryMeasured)
            {
                AnnounceBrakeAdjustment(_brakeCommitSawAboveApex ? "Never slowed" : "No braking");
                return;
            }
            string where = _brakeCommitEntryMetres < -0.5f
                ? "Early by " + Math.Abs(_brakeCommitEntryMetres).ToString("0") + "m"
                : (_brakeCommitEntryMetres > 0.5f
                    ? "Late by " + _brakeCommitEntryMetres.ToString("0") + "m"
                    : "On the mark");
            // The target is a position: the car should reach the corner speed exactly at the entrance. Negative is early, so the factor
            // climbs and the corner is braked later; positive is late and it gives factor back. No dead zone, capped per event.
            // The floor no longer holds the verdict: it is reported in the line, and the metres still decide, because a corner the floor
            // kept quick still has a braking point that can be measured wrong.
            float step = ARS.Clamp(-_brakeCommitEntryMetres * BrakeEntryFactorPerMetre, -BrakeEntryMaxStepPerEvent, BrakeEntryMaxStepPerEvent);
            float factor = ARS.Clamp(before + step, BrakeMinFactor, BrakeMaxFactor);
            if (BrakeLearningApplies) CornerContextForWrite(apexNode).BrakeFactor = factor;
            AnnounceBrakeAdjustment(where + " → " + factor.ToString("0.00"));
        }

        // One line per verdict: the car and how many miles an hour over the speed the corner asks for it arrived at the entrance.
        // Nothing else - the rest of the numbers belong in the log, not on the screen.
        void AnnounceBrakeAdjustment(string detail)
        {
            if (!ARS.BrakeLearningNotify) return;
            UI.Notify("~b~[ARS]~w~ (" + Car.DisplayName + ") " + detail);
        }

        int CornerEntranceNode(CornerPoint corner, int apexNode)
        {
            return corner == null ? apexNode : (corner.StartNode >= 0 ? corner.StartNode : OffsetCornerNode(apexNode, -corner.LengthStart));
        }

        int CornerExitNode(CornerPoint corner)
        {
            if (corner == null || corner.Node < 0) return -1;
            return corner.EndNode >= 0 ? corner.EndNode : OffsetCornerNode(corner.Node, corner.LengthEnd);
        }

        const int BrakeTargetLeadMeters = 2;
        // Past the braking target the plan holds its own number until the car has actually reached it, then lets it
        // grow: five miles an hour a second, so the arc is driven rather than sat on. The route term still votes
        // through the min, so a radius that says slower wins anyway.
        const float CornerLiftMphPerSecond = 10f;
        const float CornerLiftGraceMph = 5f;

        // Where the braking target is anchored: a fixed lead before the corner entrance, which the learned factor moves
        // from there toward the apex.
        int BrakeTargetBaseNode(CornerPoint corner, int fallbackNode)
        {
            int entranceNode = CornerEntranceNode(corner, fallbackNode);
            return entranceNode < 0 ? fallbackNode : OffsetCornerNode(entranceNode, -BrakeTargetLeadMeters);
        }

        // Past the braking target the plan expects apex speed; later braking is corner-exit scrub, not approach.
        bool HasPassedBrakingTarget()
        {
            if (NextApexNode < 0) return true;
            CornerPoint corner = _nextApexCorner;
            int targetNode = BrakeTargetBaseNode(corner, NextApexNode);
            if (targetNode < 0) return true;
            int targetDistance = ForwardNodeDistance(targetNode);
            int apexDistance = ForwardNodeDistance(NextApexNode);
            return targetDistance <= 0 || (!ARS.IsPointToPoint && targetDistance > apexDistance);
        }

        float TickScale => (0.001f * TimeSince_lastCoreTick);




        void TranslateSteerToInput()
        {

            if (float.IsNaN(Control.SteerDegrees) || float.IsInfinity(Control.SteerDegrees)) Control.SteerDegrees = 0f;

            float error = Control.SteerDegrees - Control.LastAppliedSteerDegrees;
            // Flat slew, not exponential: filtering the input to calm a small dither only adds phase lag inside the pursuit loop, and the
            // car oscillated MORE with it (build 1143, reverted the same day).
            float maxDeltaPerTick = SteerSlewRate * TickScale;
            float delta = ARS.Clamp(error, -maxDeltaPerTick, maxDeltaPerTick);
            Control.SteerDegrees = Control.LastAppliedSteerDegrees + delta;
            Control.LastAppliedSteerDegrees = Control.SteerDegrees;

            if (float.IsNaN(Control.SteerInput) || float.IsInfinity(Control.SteerInput)) Control.SteerInput = 0f;

            Control.SteerInput = ARS.Remap(Control.SteerDegrees, -VehicleData.SteeringLock, VehicleData.SteeringLock, -1, 1, true);
            
        }




        public void ComputeTargetSpeed()
        {
            if (BaseBehavior == RacerBaseBehavior.GridWait)
            {
                Brain.CurrentIntention.Speed = 200f;
                return;
            }
            if (BaseBehavior == RacerBaseBehavior.FinishedRace)
            {
                Brain.CurrentIntention.Speed = 20f;
                return;
            }
            if (BaseBehavior == RacerBaseBehavior.FinishedStandStill)
            {
                Brain.CurrentIntention.Speed = 0f;
                return;
            }
            Brain.CurrentIntention.Speed = AiConstants.MaxSpeed;

            float cornerSpd = 999f;
            if (NextApexNode >= 0)
            {
                // Held apexes set the plan, but only the ones this car decided it needs one for; every other
                // corner is route speed's job.
                if (CornerRequiresBraking(NextApexNode)) cornerSpd = Math.Max(2, ApexBrakingSpeed(NextApexNode, NextApexSpeed));
                if (NextApexNode2 >= 0 && CornerRequiresBraking(NextApexNode2))
                    cornerSpd = Math.Min(cornerSpd, Math.Max(2, ApexBrakingSpeed(NextApexNode2, NextApexSpeed2)));
                if (NextApexNode3 >= 0 && CornerRequiresBraking(NextApexNode3))
                    cornerSpd = Math.Min(cornerSpd, Math.Max(2, ApexBrakingSpeed(NextApexNode3, NextApexSpeed3)));
            }

            // Route speed from the triple-check circumradius window.
            float followTrackSpd = RouteIdealSpeedForRadius(Brain.CurrentPerception.CurveRadiusToFollowPoint);

            if (float.IsNaN(cornerSpd) || float.IsInfinity(cornerSpd)) cornerSpd = 999f;
            if (float.IsNaN(followTrackSpd) || float.IsInfinity(followTrackSpd)) followTrackSpd = 999f;
            if (cornerSpd <= 5 && Brain.Corner != null) cornerSpd = ARS.CornerApexSpeed(Brain.Corner.Point, this);

            // The corner holds its own number past the braking target until that number has been achieved — reducing to
            // within the grace band — and only then starts to grow. Lifting on position alone let a car that arrived
            // thirty over keep going, and holding the number flat to the apex sat on it through the whole arc.
            if (NextApexNode != _cornerLiftApex)
            {
                _cornerLiftApex = NextApexNode;
                _cornerLiftMps = 0f;
            }
            if (cornerSpd < 999f)
            {
                // The gate reads the lifted number: once the plan starts growing, the car has to keep up with it, and
                // a car that falls behind pauses the growth rather than losing what it already earned.
                float liftTarget = cornerSpd + _cornerLiftMps;
                if (HasPassedBrakingTarget()
                    && ARS.GetForwardSpeed(Car) <= liftTarget + ARS.MphToMps(CornerLiftGraceMph))
                    _cornerLiftMps += ARS.MphToMps(CornerLiftMphPerSecond) * TickScale;
                cornerSpd += _cornerLiftMps;
            }
            // Past the apex the corner is over and the plan lets go entirely.
            if (NextApexNode >= 0 && HasPassedApex(NextApexNode)) cornerSpd = 999f;

            // A corner inside its braking distance owns the brake: its ramp is a planned deceleration and the route's
            // instantaneous radius must not disarm it. Read before the slope and crest factors, which scale the 999
            // sentinel down into a number that would otherwise look like a live plan.
            bool cornerPlansABrake = cornerSpd < 999f;

            // Hill grip loss: exponential model, 15 degrees halves grip. It is read ahead of the car and it applies to the
            // route speed, which is a statement about this moment - never to the corner plan, whose apex is a property of
            // its own geometry. Inheriting it made a corner two hundred metres away rise and fall with the road under the
            // wheels, and a crest sampled on the way in sank a target that had nothing to do with it.
            {
                float slopeAngleDeg = ARS.RadToDeg(GetFollowPointSlopeAngle());
                float slopeGripFactor = ARS.HillGripFactorFromPitchAngle(slopeAngleDeg, this);
                followTrackSpd *= (float)Math.Sqrt(slopeGripFactor);
            }

            // Vertical-curvature no-fly gate: a single sample of the road centred on the car, with
            // the near sample 1.0s ahead and the far sample 2.0s ahead at the live car speed. The
            // factor is read once and applied to both route and corner speeds; the corner's apex
            // speed already flows from the route speed, so the corner inherits the slow-down.
            int count = ARS.TrackPoints.Count;
            int carNode = CurrentTrackPoint.Node;
            float routeCrestFactor = CrestGripSpeedFactor(carNode, count, out int routeNear, out int routeFar, out _);
            // Visualise the two lookahead nodes the live system just sampled: routeNear (1.0s ahead)
            // and routeFar (2.0s ahead). The car itself is at carNode, implicit.
            LiveRouteCrestNode = routeNear;
            LiveRouteCrestFarNode = routeFar;
            followTrackSpd *= routeCrestFactor;

            // The apex speed before vertical grip, and the only input the corner's crest call takes as its entry speed. The
            // corner plan takes no terrain factor, for the same reason the slope does not: the crest is measured under the
            // car.
            _cornerSpd = NextApexNode >= 0 ? NextApexSpeed : (Brain.Corner != null ? ARS.CornerApexSpeed(Brain.Corner.Point, this) : 999f);

            // Steer-limited speed: max speed for current steer angle before sliding. Off while the route curvature is
            // being isolated: the blend made 70 % of the curvature term a function of the car's own steering angle,
            // which closed a loop — the less it turns, the more speed the radius allows. Kept computed for readout.
            bool steerLimitedSpeedGovernsRoute = false;
            float steerRad = Math.Abs(Control.SteerDegrees) * (float)Math.PI / 180f;
            if (steerRad > 0.001f)
            {
                float turnRadius = VehicleData.WheelBase / (float)Math.Tan(steerRad);
                Brain.CurrentIntention.SteerLimitedSpeed = (float)Math.Sqrt(Handling.Gravity * VehicleData.CurrentMechanicalGrip * Math.Max(turnRadius, 1f));
                if (steerLimitedSpeedGovernsRoute) followTrackSpd = 0.7f * Brain.CurrentIntention.SteerLimitedSpeed + 0.3f * followTrackSpd;
            }
            else
            {
                Brain.CurrentIntention.SteerLimitedSpeed = 999f; // Straight = no steer limit
            }

            // Chicane boost: +10 mph to both corner and route speed while the chicane corner
            // is the active target. No node gate — Brain.Corner updates naturally on apex pass.
            if (Brain.Corner != null && Brain.Corner.Point.IsChicane)
            {
                float chicaneBoost = ARS.MphToMps(10f);
                cornerSpd += chicaneBoost;
                followTrackSpd += chicaneBoost;
            }

            float cornerTerm = cornerSpd + ARS.MphToMps(ARS.CornerOffsetMph);
            float routeTerm = followTrackSpd + ARS.MphToMps(ARS.RouteOffsetMph);
            Brain.CurrentIntention.Speed = Math.Min(cornerTerm, routeTerm);
            // Which of the two is asking for the speed decides whether the pedal may brake on it at all: a corner
            // plans its own deceleration, the road's curvature does not.
            Control.MaxBrakeFromRoute = routeTerm <= cornerTerm && !cornerPlansABrake ? RouteBrakeCap : 1f;

            // ChillOut: hold a standoff behind the closest rival ahead.
            if (ActiveManeuver.Type == ManeuverType.ChillOut)
            {
                Rival standoffRival = Brain.Rivals
                    .Where(r => r.RivalRacer != null && r.RelativePosition == RelativePos.Ahead && r.RivalRacer.Car.Exists())
                    .OrderBy(r => r.Distance)
                    .FirstOrDefault();
                if (standoffRival != null && standoffRival.Distance < ChillStandoffDistance)
                {
                    float standoffMargin = ARS.Remap(standoffRival.Distance, 0f, ChillStandoffDistance, 0f, ChillStandoffEscapeMargin, true);
                    Brain.CurrentIntention.Speed = Math.Min(Brain.CurrentIntention.Speed, standoffRival.RivalRacer.Car.Velocity.Length() + standoffMargin);
                }
            }

            // Rubber-banding: slow leaders via a lower speed target (the laggard boost is in ConvertSpeedToPedals).
            // Natural mode skips the penalty while any rival is nearby, so they can race without interference.
            float rbFactor = ComputeRubberBandFactor();
            bool rbNearbyRival = Brain.Rivals.Any(r => r.RivalRacer != null && r.Distance < 100f);
            bool rbFreeZone = Lap <= 1 && CurrentTrackPoint != null && CurrentTrackPoint.Node < 500;
            // The burn outranks the packing. Rubberbanding trims a lonely leader's plan to 0.67 - and the nitro card
            // fires precisely when the car is lonely, so the burn used to push the car into its own trimmed target and
            // the controller braked against it. Spending the bottle is a decision: while it lasts, the plan is the
            // road's, not the field's.
            bool burning = Game.GameTime < _nitrousActiveUntil;
            if (!burning && rbFactor < 1f && !rbFreeZone && !(ARS.CurrentRubberbandMode == RubberbandMode.Natural && rbNearbyRival))
                Brain.CurrentIntention.Speed *= rbFactor;
        }

        // Vertical-curvature grip factor over a triangle the car is currently inside, as a speed multiplier.
        // The triangle's three points are the car (start), 1.0s ahead at the current speed (midpoint), and
        // 2.0s ahead. The car always slows enough to bring the gs at the triangle's midpoint down to
        // ARS.MaxCrestLift -- the speed factor is sqrt(MaxCrestLift / |deltaGs|) so a 1.2g reading on a
        // 1.0g cap forces the car to 0.91 of its current speed, and a 2.0g reading on a 0.5g cap forces
        // 0.5. The gs is the gs the car will feel in 1.0s (the curvature's centre), not the gs the car
        // is currently on -- that is the lookahead lead time, not the current state. Dips load the
        // tyres, not unload, so they're ignored. The function takes the live car speed as the probe
        // velocity, so a faster car sees a longer sample arc and a slower one a shorter one -- the
        // same 1.0s/2.0s time window expressed in metres.
        const float nearSeconds = 1.0f;
        const float farSeconds = 2.0f;
        float CrestGripSpeedFactor(int carNode, int count, out int nearNode, out int farNode, out float rawDeltaGs)
        {
            nearNode = -1;
            farNode = -1;
            rawDeltaGs = 0f;
            if (carNode < 0 || carNode >= count) return 1f;
            float v = Car.Velocity.Length();
            if (v < 0.5f) return 1f;     // parked: no forward motion, no sample

            // Walk forward in metres, accumulating along the route, until the target seconds are reached.
            // On a circuit the wrap is implicit (mod count), on point-to-point the walk clamps to count-1.
            int near = TrackLoader.NodeAhead(carNode, nearSeconds * v, count);
            int far = TrackLoader.NodeAhead(carNode, farSeconds * v, count);
            if (near < 0 || far < 0) return 1f;
            if (near == carNode || far == carNode || near == far) return 1f;
            nearNode = near;
            farNode = far;

            float deltaGs = ARS.HillGripDeltaGs(ARS.TrackPoints[carNode].Position, ARS.TrackPoints[near].Position, ARS.TrackPoints[far].Position, v);
            rawDeltaGs = deltaGs;
            if (deltaGs > 0f) deltaGs = 0f;
            float magGs = Math.Abs(deltaGs);
            if (magGs <= ARS.MaxCrestLift) return 1f;
            return (float)Math.Sqrt(ARS.MaxCrestLift / magGs);
        }

        // Rubber-band factor: <1 for leaders (penalty), >1 for laggards (boost), 1 at center.
        float ComputeRubberBandFactor()
        {
            if (ARS.RubberbandingPct <= 0 || RacePosition <= 0) return 1f;
            int fieldSize = ARS.Racers.Count;
            int centerPos = (fieldSize + 1) / 2;
            Racer playerRacer = ARS.Racers.FirstOrDefault(r => r.Car.Exists() && r.Car == Game.Player.Character.CurrentVehicle);
            if (playerRacer != null) centerPos = playerRacer.RacePosition;
            int maxDistance = Math.Max(Math.Max(centerPos - 1, fieldSize - centerPos), 1);
            float norm = (RacePosition - centerPos) / (float)maxDistance;
            return 1f + norm * 0.33f * (ARS.RubberbandingPct * 0.01f);
        }

        // TrackPoint.Elevation is 90·sin(pitch), so this converts it back to the slope sine.
        const float ElevationToSlopeSine = 1f / 90f;

        // Slope angle at the lookahead point: average grade over the 1.0s-to-2.0s window ahead of the
        // car, matching the no-fly gate's triangle so the slope and the crest read the same road
        // sample. Two-point linear fit: slope = (zFar - zNear) / (sFar - sNear). The angle is the
        // atan of that ratio; the caller takes the magnitude via the absolute value.
        float GetFollowPointSlopeAngle()
        {
            float v = Car.Velocity.Length();
            if (v < 0.5f) return 0f;
            int near = TrackLoader.NodeAhead(CurrentTrackPoint.Node, 1.0f * v, ARS.TrackPoints.Count);
            int far = TrackLoader.NodeAhead(CurrentTrackPoint.Node, 2.0f * v, ARS.TrackPoints.Count);
            if (near < 0 || far < 0 || near == far) return 0f;

            Vector3 from = ARS.TrackPoints[near].Position;
            Vector3 to = ARS.TrackPoints[far].Position;
            float horizDist = Vector2.Distance(new Vector2(from.X, from.Y), new Vector2(to.X, to.Y));
            if (horizDist < 0.01f) return 0f;
            return Math.Abs((float)Math.Atan2(to.Z - from.Z, horizDist));
        }

        // TCS and ABS are measured on the game's per-wheel slip ratio (wheel+0x174): 0 free rolling, 0.4 at the
        // traction curve's peak, 1.0 where it goes flat, spin negative and lock positive. Read live: the slot is
        // that raw signed ratio, and the engine's own ABS forces it to +/-5 while it is active (wheel.cpp:5061).
        const float OffTrackTargetShare = 0.5f;

        // One home for the throttle reason caps: each reason states the level it wants, its field glides toward
        // that level at the shared rate both ways, and ConvertSpeedToPedals takes the minimum against the plan's
        // own throttle and tags the winner. The level is the logic half, the shared rate the clock half.
        const float ReasonCapSlewRate = 3.5f;
        const float TcsCapFloor = 0.2f;

        // The taper the TCS cap cuts on: nothing at the threshold, the cap's floor one span of slip past it, tanh
        // between so the onset has no slope step, normalised so the floor lands where the span says it does. A tanh
        // runs above its own chord, so the cut leads a straight ramp through the middle of the span.
        static readonly float TaperNormaliser = (float)(1.0 / Math.Tanh(1.0));

        static float TaperShare(float depth, float span)
        {
            return ARS.Clamp((float)Math.Tanh(depth / span) * TaperNormaliser, 0f, 1f);
        }
        public const float YieldThrottleLevel = 0.5f;
        const float OffTrackThrottleLevel = 0f;
        const float OffTrackSafeSpeedMph = 20f;
        const float OffTrackFullThrottleMph = 15f;
        // The throttle an off-track car gets while it is pointed back at the track: three quarters, not the limiter.
        // Half was too little to pull away from a stop in the grass.
        const float OffTrackRecoverThrottle = 0.75f;

        // Stability: each wheel out of contact costs a quarter of the throttle, since it cannot use the grip it has.
        // Half the throttle cap per wheel per second, additive: four wheels empty it in exactly half a second, one in two.
        // Recovery only runs with every wheel down and takes the cap from zero to whole in two seconds - slower than the
        // worst-case fall by four, because being out of contact is the emergency and coming back is what to be careful with.
        const float StabilityFallPerWheelPerSecond = 0.5f;
        const float StabilityRisePerSecond = 0.5f;

        // The overspeed correction only arms while the car is going straight and up to speed: steering or slide of
        // its own puts lateral work and bump transients into the longitudinal read, and below the speed gate the
        // launch assist is what it would otherwise be reading.
        const float OverspeedArmMaxSteerDegrees = 2f;
        const float OverspeedArmMaxSlideDegrees = 2f;
        const float OverspeedArmMinSpeedMph = 30f;
        // The player has no steer command for ARS to read, so their arm test reads what the car is doing: a quarter
        // of a g of lateral is the same "barely turning" the two-degree command allows an AI car at speed.
        const float OverspeedArmMaxLateralGs = 0.25f;

        // Where the correction starts; how hard it then cuts is the player's rate.
        const float OverspeedCutStartGs = 0.2f;

        // Cutting is fast, recovering is deliberate: the shared rate going down, half of it coming back.
        const float ReasonCapRecoveryScale = 0.5f;

        static float Glide(float current, float level, float fallRate, float riseRate, float tickScale)
        {
            float rate = level < current ? fallRate : riseRate;
            float step = rate * tickScale;
            return ARS.Clamp(current + ARS.Clamp(level - current, -step, step), 0f, 1f);
        }

        float GlideCap(float current, float level)
        {
            return Glide(current, level, ReasonCapSlewRate, ReasonCapSlewRate * ReasonCapRecoveryScale, TickScale);
        }

        // The Gs correction's target level. GTA lets an uphill car accelerate beyond what its wheel power should
        // produce; the AI pays that as a pedal cap, and the player's pedal is not ours to cap, so the same level is
        // spent as drive force instead (UpdatePowerCompensation).
        float OverspeedThrottleLevel()
        {
            bool straight = ControlledByPlayer
                ? Math.Abs(VehicleData.GetLateralGs(Car.ForwardVector)) < OverspeedArmMaxLateralGs
                : Math.Abs(Control.SteerDegrees) < OverspeedArmMaxSteerDegrees;
            bool armed = straight
                && Math.Abs(VehicleData.SlideAngle) < OverspeedArmMaxSlideDegrees
                && Car.Velocity.Length() > ARS.MphToMps(OverspeedArmMinSpeedMph);
            // A positive accept, not an absence test: Clamp returns its min bound for a NaN, so a non-finite excess
            // would otherwise command a full cut instead of no cut at all.
            if (ARS.OverspeedCutPerTenthG <= 0f || !armed || !(VehicleData.OverspeedExcessGs > 0f)) return 1f;
            float cut = ARS.OverspeedCutPerTenthG * (VehicleData.OverspeedExcessGs - OverspeedCutStartGs) / 0.1f;
            return ARS.Clamp(1f - cut, 0f, 1f);
        }

        void UpdateThrottleReasonCaps()
        {
            // The kick card plays on exactly the wheelspin this cut moderates: the card owns the pedal for its
            // window, so the field is pinned for the window rather than glided back.
            Control.MaxThrottleFromTCS = _kickPlaying ? 1f : GlideCap(Control.MaxThrottleFromTCS, TcsCapLevel());
            int wheelsOffGround = ARS.WheelsOffGround(Car);
            if (wheelsOffGround > 0)
                Control.MaxThrottleFromStability -= StabilityFallPerWheelPerSecond * wheelsOffGround * TickScale;
            else
                Control.MaxThrottleFromStability += StabilityRisePerSecond * TickScale;
            Control.MaxThrottleFromStability = ARS.Clamp(Control.MaxThrottleFromStability, 0f, 1f);

            // The burn is exempt. This cap exists to catch the physics GTA gets wrong on its own - an uphill car pulling
            // more Gs than its wheels justify - and a nitro burn produces exactly that signature on purpose. Left in, the
            // cap ate the AI's burn and never the player's, because only the AI's pedal is ours to trim.
            Control.MaxThrottleFromOverspeed = Game.GameTime < _nitrousActiveUntil
                ? 1f
                : GlideCap(Control.MaxThrottleFromOverspeed, OverspeedThrottleLevel());

            // The lift is a distance, not a countdown: no ramp on time-to-reach, so a closing rival never cuts the throttle
            // to nothing. Three metres out is full gas and contact is the floor, and the floor is a tenth of throttle, not
            // zero — this term yields the acceleration, it does not stop the car.
            //
            // Written ascending on both sides on purpose: Remap's clamp runs Clamp(r, out_min, out_max) and Clamp tests the
            // minimum first, so a descending output range returns out_min for everything. Ascending in and out, and the
            // clamp is honest.
            float rivalLevel = 1f;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || r.RelativePosition != RelativePos.Ahead) continue;
                rivalLevel = Math.Min(rivalLevel, ARS.Remap(r.FrontGap, 0f, RivalLiftMeters, RivalLiftFloor, 1f, true));
            }
            Control.MaxThrottleFromRival = GlideCap(Control.MaxThrottleFromRival, rivalLevel);

            Control.MaxThrottleFromChillOut = GlideCap(Control.MaxThrottleFromChillOut, ActiveManeuver.Type == ManeuverType.ChillOut ? ChillThrottleCap : 1f);
            Control.MaxThrottleFromYield = GlideCap(Control.MaxThrottleFromYield, ActiveManeuver.Type == ManeuverType.Yield && ActiveManeuver.Target != null ? YieldThrottleLevel : 1f);

            float offTrackThrottle = 1f;
            if (OutOfTrackDistance() > 0f)
            {
                offTrackThrottle = ARS.Remap(ARS.MpsToMph(Car.Velocity.Length()), OffTrackSafeSpeedMph, OffTrackFullThrottleMph, OffTrackThrottleLevel, 1f, true);
                // A car whose projection is closing on the centre line is driving itself back in, so the speed
                // limiter is replaced by a flat half. That is looser than the limiter above 17.5 mph and tighter
                // below it: recovery happens under power, but never a full-throttle launch off a loose surface.
                if (ProjectionClosingOnCentre()) offTrackThrottle = OffTrackRecoverThrottle;
            }
            Control.MaxThrottleFromOffTrack = GlideCap(Control.MaxThrottleFromOffTrack, offTrackThrottle);

            float recoveryThrottle = 1f;
            if (_stuckPhase == StuckPhase.Drive) recoveryThrottle = ARS.Remap(ARS.MpsToMph(Car.Velocity.Length()), RecoveryDriveMph + RecoveryCoastBandMph, RecoveryDriveMph, 0f, 1f, true);
            Control.MaxThrottleFromRecovery = GlideCap(Control.MaxThrottleFromRecovery, recoveryThrottle);
        }

        // Both signals are read every tick: the old system latched them at 3 Hz and missed crests shorter than the
        // sample period. Riding above the at-rest height means the chassis is unloaded, bouncing or airborne;
        // demanding more lateral acceleration than the grip allows means the car is rotating, not cornering.
        // Both TCS knobs are shares of the peak slip; the field the wheels publish is the peak times that share.
        static float SlipForShare(int sharePercent)
        {
            return sharePercent * 0.01f * ARS.SlipPeakRatio;
        }

        float TcsCapLevel()
        {
            if (!ARS.TcsEnabled) return 1f;
            float wheelspin = ARS.MaxWheelSlip(Car);

            float slipTarget = -SlipForShare(ARS.TcsSlipLimitPct) * (OutOfTrackDistance() > 0f ? OffTrackTargetShare : 1f);
            float spinDepth = Math.Max(0f, slipTarget - wheelspin);
            float depthShare = TaperShare(spinDepth, SlipForShare(ARS.TcsTaperSpanPct));
            return 1f - depthShare * (1f - TcsCapFloor);
        }

        // ABS: a proportional taper on the slip-ratio axis, with the rotation guard as its backstop. The field runs 0 free
        // rolling, 0.4 at the traction peak, 1.0 locked; full brake to the knee at 0.5, then a straight line to nothing at
        // the lock point, so the wheel is never allowed to stay locked and the answer is immediate - no glide, no state.
        const float AbsLockRadPerSecond = 2f;
        const float AbsMinSpeedMph = 15f;
        // The taper, on the slip-ratio axis the curve is drawn on: 0 free rolling, 0.4 the peak, 1.0 locked. Full brake to
        // the knee, then a straight line to nothing at the lock point - proportional, so the wheel is never allowed to stay
        // locked and there is no glide to wait for. The knee sits past the peak on purpose (0.5 against 0.4): the criterion
        // is keeping the wheel turning with margin, not extracting the last newton.
        const float AbsKneeSlip = 0.5f;
        const float AbsLockSlip = 1f;
        const float AbsBrakeFloor = 0f;
        int _absLoggedAt = 0;
        bool _absWasLocked = false;

        void UpdateBrakeReasonCaps()
        {
            bool countersteering = IsFullCountersteer();
            Control.MaxBrakeFromABS = AbsCapLevel();
            Control.MaxBrakeFromCountersteer = GlideCap(Control.MaxBrakeFromCountersteer, countersteering ? CountersteerPedalCap : 1f);
            // The ABS guard is otherwise invisible: its only sign is the pedal bar. Log the moment a wheel crosses the lock
            // floor and the reading that proves it, with the cap the glide has actually reached - the difference between the
            // detector firing and the pedal letting go is the whole question on a car that has no ABS of its own.
            if (ARS.AbsEnabled && Math.Abs(Control.Brake) > 0.2f && Game.GameTime > _absLoggedAt)
            {
                bool locked = ARS.MpsToMph(Car.Velocity.Length()) >= AbsMinSpeedMph && ARS.AnyWheelRotationBelow(Car, AbsLockRadPerSecond);
                if (locked || locked != _absWasLocked)
                {
                    _absLoggedAt = Game.GameTime + 500;
                    ARS.Log(ARS.LogImportance.Info, "ABS " + Name + ": locked=" + locked + " slip=" + ARS.BrakeLockSlip(Car).ToString("0.00") + " minWheel=" + ARS.MinWheelRotation(Car).ToString("0.00") + " rad/s cap=" + Control.MaxBrakeFromABS.ToString("0.00") + " pedal=" + Control.Brake.ToString("0.00") + " mph=" + ARS.MpsToMph(Car.Velocity.Length()).ToString("0"));
                }
                _absWasLocked = locked;
            }
        }

        float AbsCapLevel()
        {
            if (!ARS.AbsEnabled) return 1f;
            if (ARS.MpsToMph(Car.Velocity.Length()) < AbsMinSpeedMph) return 1f;
            // Backstop, below the taper: a wheel that has actually stopped turning is past the curve altogether, whatever
            // the slip axis says, so the brake goes to the floor outright.
            if (ARS.AnyWheelRotationBelow(Car, AbsLockRadPerSecond)) return AbsBrakeFloor;
            float slip = ARS.BrakeLockSlip(Car);
            if (slip <= AbsKneeSlip) return 1f;
            float overKnee = (slip - AbsKneeSlip) / Math.Max(AbsLockSlip - AbsKneeSlip, 0.01f);
            return Math.Max(AbsBrakeFloor, 1f - overKnee);
        }

        // The one throttle composition: the plan's throttle against every reason field, the argmin named into the
        // tag (first reason wins ties). Offtrack's cut and the countersteer replacement pre-empt the ordinary
        // reasons; grid wait owns the pedal outright and stuck recovery re-tags after.
        float ComposeThrottleCap(float baseThrottle, bool offtrackLimited, bool countersteering)
        {
            // The kick card owns a floor, not a cap: its window raises the base before the argmin, and the only
            // cap it defeats is TCS, pinned at the field — stability and the live caps still bound the result.
            bool kickRaises = _kickPlaying && baseThrottle < KickPedalLevel;
            if (kickRaises) baseThrottle = KickPedalLevel;

            float ceiling = Math.Min(Control.MaxThrottleFromTCS, Control.MaxThrottleFromStability);
            ceiling = Math.Min(ceiling, Control.MaxThrottleFromOverspeed);
            ceiling = Math.Min(ceiling, Control.MaxThrottleFromRival);
            ceiling = Math.Min(ceiling, Control.MaxThrottleFromChillOut);
            ceiling = Math.Min(ceiling, Control.MaxThrottleFromYield);
            ceiling = Math.Min(ceiling, Control.MaxThrottleFromOffTrack);
            ceiling = Math.Min(ceiling, Control.MaxThrottleFromRecovery);
            Control.MaxThrottle = ceiling;

            float binding = baseThrottle;
            ThrottleReason reason = ThrottleReason.Plan;
            if (Control.MaxThrottleFromTCS < binding) { binding = Control.MaxThrottleFromTCS; reason = ThrottleReason.Tcs; }
            if (Control.MaxThrottleFromStability < binding) { binding = Control.MaxThrottleFromStability; reason = ThrottleReason.Stability; }
            if (Control.MaxThrottleFromOverspeed < binding) { binding = Control.MaxThrottleFromOverspeed; reason = ThrottleReason.Overspeed; }
            if (Control.MaxThrottleFromRival < binding) { binding = Control.MaxThrottleFromRival; reason = ThrottleReason.Rival; }
            if (Control.MaxThrottleFromChillOut < binding) { binding = Control.MaxThrottleFromChillOut; reason = ThrottleReason.ChillOut; }
            if (Control.MaxThrottleFromYield < binding) { binding = Control.MaxThrottleFromYield; reason = ThrottleReason.Yield; }
            if (Control.MaxThrottleFromOffTrack < binding) { binding = Control.MaxThrottleFromOffTrack; reason = ThrottleReason.Offtrack; }
            if (Control.MaxThrottleFromRecovery < binding) { binding = Control.MaxThrottleFromRecovery; reason = ThrottleReason.StuckRecovery; }
            if (offtrackLimited) reason = ThrottleReason.Offtrack;
            if (countersteering) reason = ThrottleReason.Countersteer;
            if (BaseBehavior == RacerBaseBehavior.GridWait) reason = ThrottleReason.GridWait;
            if (_kickPlaying && reason == ThrottleReason.Plan) reason = ThrottleReason.Kick;

            Control.ThrottleReason = reason;
            Control.ThrottleReasonLevel = binding;
            return binding;
        }

        // The brake-side composition: the plan's braking against the ABS and countersteer caps, the argmin named
        // into the tag; grid wait owns the tag while waiting.
        float ComposeBrakeCap(float baseBrake, bool countersteering)
        {
            Control.MaxBrake = Math.Min(Control.MaxBrakeFromABS, Control.MaxBrakeFromCountersteer);
            Control.MaxBrake = Math.Min(Control.MaxBrake, Control.MaxBrakeFromRoute);

            float binding = baseBrake;
            BrakeReason reason = BrakeReason.Plan;
            if (Control.MaxBrakeFromABS < binding) { binding = Control.MaxBrakeFromABS; reason = BrakeReason.Abs; }
            if (Control.MaxBrakeFromRoute < binding) { binding = Control.MaxBrakeFromRoute; reason = BrakeReason.Route; }
            if (Control.MaxBrakeFromCountersteer < binding) { binding = Control.MaxBrakeFromCountersteer; reason = BrakeReason.Countersteer; }
            if (countersteering) reason = BrakeReason.Countersteer;
            if (BaseBehavior == RacerBaseBehavior.GridWait) reason = BrakeReason.GridWait;

            Control.BrakeReason = reason;
            Control.BrakeReasonLevel = binding;
            return binding;
        }
        Rival NearestRival()
        {
            Rival nearest = null;
            float best = float.MaxValue;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || !r.RivalRacer.Car.Exists()) continue;
                if (r.Distance < best) { best = r.Distance; nearest = r; }
            }
            return nearest;
        }

        // The cards ride the fast core pass. Track state, perception and the car's own velocity were rebuilt earlier in
        // this same core tick; the rival records are the half-second publish's, owned by the slower rival scan, and
        // Yield's pressure belongs to the pressure beat. The crowd card keeps the slow beat because it counts the field.
        void ConsiderManeuvers()
        {
            if (ControlledByPlayer || BaseBehavior != RacerBaseBehavior.Race || ARS.Racers.Count < 1) return;

            // Force-disable maneuvers armed for more than 8s without firing.
            if (ActiveManeuver.Type != ManeuverType.None && Game.GameTime - ActiveManeuver.LastEnabled > 8000)
            {
                EndCard();
                _divebombApexNode = -1;
            }

            // Divebomb cleanup: off once the partner stops being one — gone, out of the perceived set, off this piece
            // of the route or travelling another way — and off once we pass the armed apex. The commit lane already
            // drops when the target leaves the perceived set, because that lookup is by reference there; the brake
            // effects do not, and they reach the next three apexes, so the release cannot wait for the apex.
            if (ActiveManeuver.Type == ManeuverType.DiveBomb)
            {
                Rival divePartner = null;
                foreach (Rival r in Brain.Rivals)
                {
                    if (r.RivalRacer != ActiveManeuver.Target) continue;
                    divePartner = r;
                    break;
                }
                bool partnerLost = ActiveManeuver.Target == null || !ActiveManeuver.Target.Car.Exists() || divePartner == null || Math.Abs(divePartner.RouteGapAhead) > DivebombReleaseGapMeters || Math.Abs(divePartner.DirectionDiff) > DivebombReleaseAngleDegrees;

                if ((partnerLost || HasPassedApex(_divebombApexNode)) && CardMayEnd)
                {
                    ARS.Log(ARS.LogImportance.Info, "Card divebomb: " + Name + " released"
                        + (HasPassedApex(_divebombApexNode) ? " at apex" : " partner lost")
                        + " after " + (Game.GameTime - ActiveManeuver.LastEnabled) + " ms", true);
                    EndCard();
                    _divebombApexNode = -1;
                }
            }

            // DefendLane fold: off once we pass the defended apex or the target gets past us.
            if (ActiveManeuver.Type == ManeuverType.DefendLane)
            {
                bool targetTrails = false;
                bool threatGone = false;
                foreach (Rival r in Brain.Rivals)
                {
                    if (r.RivalRacer != ActiveManeuver.Target) continue;
                    targetTrails = r.RelativePosition != RelativePos.Ahead;
                    // The threat is over when he stops coming: his own time to reach this car reads infinite once he is no
                    // longer closing, or he will now arrive at the entrance clearly after it does. Read with a margin, because
                    // a release costs the card its cooldown and a wrong one is expensive.
                    float hisTimeToReachMe = float.PositiveInfinity;
                    foreach (Rival back in r.RivalRacer.Brain.Rivals)
                    {
                        if (back.RivalRacer != this) continue;
                        hisTimeToReachMe = back.TimeToReach;
                        break;
                    }
                    float myTime = _defendEntranceNode >= 0 ? ForwardNodeDistance(_defendEntranceNode) / Math.Max(Car.Velocity.Length(), 1f) : 0f;
                    float hisTime = _defendEntranceNode >= 0 ? r.RivalRacer.ForwardNodeDistance(_defendEntranceNode) / Math.Max(r.Speed, 1f) : 0f;
                    threatGone = float.IsPositiveInfinity(hisTimeToReachMe) || hisTime > myTime + DefendThreatMarginSeconds;
                    break;
                }
                bool lostTarget = ActiveManeuver.Target == null || !ActiveManeuver.Target.Car.Exists() || !targetTrails;

                if ((lostTarget || threatGone || HasPassedApex(_defendApexNode)) && CardMayEnd)
                {
                    EndCard();
                    _defendApexNode = -1;
                    _defendEntranceNode = -1;
                }
            }

            // Card model: while no card is in play, the hand is evaluated in priority order.
            // Nitro resolves instantly (burn lives in _nitrousActiveUntil), so it never occupies the slot.
            if (ActiveManeuver.Type == ManeuverType.None) TryPlayNitrousCard();

            if (ActiveManeuver.Type == ManeuverType.None) TryPlayDefendLaneCard();

            if (ActiveManeuver.Type == ManeuverType.None) TryPlayDivebombCard();

            if (ActiveManeuver.Type == ManeuverType.None) TryPlayYieldCard();
        }

        // The crowd card counts the whole field, so it keeps the slow beat.
        void ConsiderCrowdCard()
        {
            if (ControlledByPlayer || BaseBehavior != RacerBaseBehavior.Race || ARS.Racers.Count < 1) return;

            // ChillOut cleanup: off once the pack around us thins out.
            if (ActiveManeuver.Type == ManeuverType.ChillOut && CardMayEnd && RivalsWithinDistance(ChillReleaseRadiusMeters) < ChillCrowdCount - 1)
            {
                EndCard();
            }

            // Off for now. The card holds a standoff behind the nearest rival and pays for it in throttle; parked until the
            // pack behaviour it was written for earns that cost. The cleanup above still runs, so a card armed before this
            // flag was flipped still ends cleanly.
            const bool ChillOutEnabled = false;
            // ChillOut: only when fast enough for bunching to matter, in a dense pack of better-placed cars.
            if (ChillOutEnabled && ActiveManeuver.Type == ManeuverType.None && ARS.MpsToMph(Car.Velocity.Length()) >= ChillMinSpeedMph && RivalsWithinDistance(ChillRivalCrowdDistance) >= ChillCrowdCount)
            {
                Rival closestRival = NearestRival();
                if (closestRival != null && CardFree(closestRival.RivalRacer, ManeuverType.ChillOut))
                {
                    ActiveManeuver.Type = ManeuverType.ChillOut;
                    ActiveManeuver.Target = closestRival.RivalRacer;
                    ActiveManeuver.LastEnabled = Game.GameTime;
                }
            }
        }

        int ForwardNodeDistance(int targetNode)
        {
            int fwd = targetNode - CurrentTrackPoint.Node;
            if (!ARS.IsPointToPoint && fwd < 0) fwd += ARS.TrackPoints.Count;
            return fwd;
        }

        bool IsWithinCorner(CornerPoint corner)
        {
            if (corner == null || corner.StartNode < 0 || corner.EndNode < 0) return false;

            if (ARS.IsPointToPoint)
                return CurrentTrackPoint.Node >= corner.StartNode
                    && CurrentTrackPoint.Node <= corner.EndNode;

            if (corner.StartNode <= corner.EndNode)
                return CurrentTrackPoint.Node >= corner.StartNode
                    && CurrentTrackPoint.Node <= corner.EndNode;

            return CurrentTrackPoint.Node >= corner.StartNode
                || CurrentTrackPoint.Node <= corner.EndNode;
        }

        void UpdateNitrous()
        {
            // The burn is shut down whatever the setting says now: an override left armed is not a racing state, it is the
            // car keeping a bottle it was never given, and it outlives the toggle that spawned it.
            // Unticking the setting cuts a live burn: the player asked for no nitrous, and an override that keeps applying until its
            // own window expires is not what the toggle means.
            if (_nitrousActiveUntil > 0 && (!ARS.NitrousEnabled || Game.GameTime >= _nitrousActiveUntil)) StopNitrous();
        }

        // Valid when a shot is available and the straight is long enough; appropriate when contested, defended,
        // lonely, or spent near the finish with a rival close.
        // Two cars reading the same situation must not both commit to it: a dive answered by a dive, or two cars
        // yielding to each other, reads as a glitch rather than as racecraft. A card may not arm against a target
        // that is already playing that same card.
        static bool CardFree(Racer target, ManeuverType type)
        {
            return target == null || target.ActiveManeuver.Type != type;
        }

        bool TryPlayNitrousCard()
        {
            if (!ARS.NitrousEnabled || Lap <= _nitrousLapUsed) return false;
            if (ARS.RaceStartTime > 0 && Game.GameTime - ARS.RaceStartTime < NitrousArmDelayMs) return false;
            if (Control.Brake > 0f) return false;
            if (OutOfTrackDistance() > 0f) return false;
            if (Math.Abs(Control.SteerDegrees) >= NitrousMaxSteerDegrees) return false;
            if (Control.Throttle < NitrousMinThrottle) return false;
            if (!IsAwd() && Function.Call<int>((Hash)MaxDriveGearHash, Car) > 3 && Function.Call<int>((Hash)CurrentDriveGearHash, Car) <= 2) return false;
            if (NextApexNode < 0) return false;

            float speed = Car.Velocity.Length();
            Rival closestRival = NearestRival();

            // Finish spender: with a rival nearby the burn near the line is always worth it,
            // so the 8s corner gate no longer applies.
            bool finishSpender = closestRival != null && closestRival.Distance <= NitrousNearbyRivalDistance
                && RemainingRaceDistanceMeters() <= speed * (NitrousDurationMs / 1000f) + NitrousFinishExtraDistance;
            if (!finishSpender)
            {
                CornerPoint corner = _nextApexCorner;
                // Circuit wrap makes a just-behind entrance read a lap away; veto in-corner shots.
                if (corner != null && IsWithinCorner(corner)) return false;
                int entranceNode = corner == null
                    ? NextApexNode
                    : (corner.StartNode >= 0 ? corner.StartNode : OffsetCornerNode(NextApexNode, -corner.LengthStart));
                if (ForwardNodeDistance(entranceNode) / Math.Max(speed, 1f) < NitrousCornerLookaheadSeconds) return false;
            }

            bool rivalNearbyFaster = closestRival != null && closestRival.Speed > speed;
            bool rivalBehindIncoming = false;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || !r.RivalRacer.Car.Exists() || r.RelativePosition != RelativePos.Behind || r.Speed <= speed) continue;
                if (r.Distance / (r.Speed - speed) < NitrousDefenseReachSeconds) { rivalBehindIncoming = true; break; }
            }
            bool lonelyClear = closestRival == null
                && speed < Handling.EstimatedTopSpeed * NitrousLonelySpeedFraction
                && ForwardNodeDistance(NextApexNode) > NitrousLonelyMinApexDistance;
            if (!rivalNearbyFaster && !rivalBehindIncoming && !lonelyClear && !finishSpender) return false;

            StartNitrous();
            return true;
        }

        // Both fight cards are considered over the same span to the entrance. The hand is evaluated on the half-second
        // beat, so this window is the number of chances a card gets: four at 3-5 s, eight at 2-6. Narrowing it is not free.
        const float FightWindowMinSeconds = 2f;
        const float FightWindowMaxSeconds = 6f;
        // A partner reaches that entrance within this of the car: a wider band accepts a car the car will not fight
        // there, a narrower one drops the slower car just ahead, which is the dive worth taking.
        const float DivebombReachSeconds = 0.75f;
        const float DivebombFullThrottle = 0.99f;

        // Divebomb card: at full throttle in the last band before the brakes, with an open side to commit into (at
        // most one rival wall active), dive the rival arriving at that entrance with us and still on this piece of
        // the route.
        bool TryPlayDivebombCard()
        {
            if (!CardCooled(ManeuverType.DiveBomb)) return false;
            if (Brain.Corner == null) return false;
            if (Control.Throttle < DivebombFullThrottle) return false;
            if (_activeRivalWallCount > 1) return false;

            int apexNode = Brain.Corner.Point.Node;
            int entranceNode = CornerEntranceNode(Brain.Corner.Point, apexNode);
            if (entranceNode < 0) return false;
            float timeToEntrance = ForwardNodeDistance(entranceNode) / Math.Max(Car.Velocity.Length(), 1f);
            if (!ARS.IsBetween(timeToEntrance, FightWindowMinSeconds, FightWindowMaxSeconds)) return false;

            Rival diveTarget = null;
            float nearestDive = float.MaxValue;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || !r.RivalRacer.Car.Exists()) continue;
                // Arriving together is the question, not who arrives first: a one-sided test drops the slower car just
                // ahead, which is the dive worth taking. The route band keeps the target on this piece of the road,
                // and the forward distance wraps past the apex, so another stretch cannot answer for it.
                // He is the one arriving first, and by no more than the reach: a car this one will beat to the entrance
                // needs no dive, and one it cannot reach needs no dive either. The window is signed on purpose - the
                // old absolute difference also armed the car that was going to arrive first anyway.
                float rivalTimeToEntrance = r.RivalRacer.ForwardNodeDistance(entranceNode) / Math.Max(r.Speed, 1f);
                if (rivalTimeToEntrance >= timeToEntrance) continue;
                if (timeToEntrance - rivalTimeToEntrance > DivebombReachSeconds) continue;
                // Ahead on the road, and closing on him: without closure braking later buys nothing. The upper bound is
                // the release's own: a card armed beyond it is released on the next tick, which re-rolls the bonus every
                // frame and reads as a card flickering rather than a move being made.
                if (r.RouteGapAhead <= 0f || r.RouteGapAhead > SameSectionMaxGapMeters) continue;
                if (r.ForwardSpeedGap <= 0f) continue;
                // The release tests direction, so the arm has to as well: a target diverging past the gate would otherwise
                // arm, release and re-arm inside the same tick, re-rolling the dive's brake bonus at the core rate.
                if (Math.Abs(r.DirectionDiff) > AvoidAngleGateDegrees) continue;
                if (!CardFree(r.RivalRacer, ManeuverType.DiveBomb)) continue;
                if (r.Distance < nearestDive) { nearestDive = r.Distance; diveTarget = r; }
            }
            if (diveTarget == null) return false;

            ActiveManeuver.Type = ManeuverType.DiveBomb;
            ActiveManeuver.Target = diveTarget.RivalRacer;
            ActiveManeuver.LastEnabled = Game.GameTime;
            _divebombApexNode = apexNode;
            _divebombBrakeBonus = ARS.GetRandomInt(2, 9) / 100f;
            ARS.Log(ARS.LogImportance.Info, "Card divebomb: " + Name + " armed on " + diveTarget.RivalRacer.Name
                + " apex " + apexNode + " bonus +" + _divebombBrakeBonus.ToString("0.00") + " gap " + diveTarget.RouteGapAhead.ToString("0.0")
                + " dT " + (timeToEntrance - diveTarget.RivalRacer.ForwardNodeDistance(entranceNode) / Math.Max(diveTarget.Speed, 1f)).ToString("0.00"), true);
            return true;
        }

        // DefendLane card: cover the inside so a faster chaser that reaches the entrance no later
        // than we do can't dive underneath.
        bool TryPlayDefendLaneCard()
        {
            if (!CardCooled(ManeuverType.DefendLane)) return false;
            if (Brain.Corner == null) return false;

            int apexNode = Brain.Corner.Point.Node;
            int entranceNode = CornerEntranceNode(Brain.Corner.Point, apexNode);
            if (entranceNode < 0) return false;
            float myTimeToEntrance = ForwardNodeDistance(entranceNode) / Math.Max(Car.Velocity.Length(), 1f);
            if (!ARS.IsBetween(myTimeToEntrance, FightWindowMinSeconds, FightWindowMaxSeconds)) return false;

            Rival defenderTarget = null;
            float nearestDefender = float.MaxValue;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || !r.RivalRacer.Car.Exists()) continue;
                if (r.RivalRacer.ActiveManeuver.Type == ManeuverType.DiveBomb || r.RivalRacer.ActiveManeuver.Type == ManeuverType.DefendLane) continue;
                bool closes = r.RelativePosition == RelativePos.Behind && r.ForwardSpeedGap < 0f;
                bool arrivesNoLater = r.RivalRacer.ForwardNodeDistance(entranceNode) / Math.Max(r.Speed, 1f) <= myTimeToEntrance;
                if (!closes || !arrivesNoLater) continue;
                // His own view of this car carries the number directly: from his side this car is the one ahead, so his
                // TimeToReach is defined where ours reads infinity, and it is measured on his FrontGap - bumper to bumper
                // - rather than between centres. It only exists while this car is among his three nearest, so the mirror
                // below is the fallback rather than the rule.
                // Only his own view answers this: his TimeToReach is measured on his FrontGap, bumper to bumper, and
                // carries his lane gate. When this car is not among his three nearest there is no number, and no
                // defend - a fallback measured between centres disagreed with him on the only quantity that matters.
                float timeToReachMe = float.PositiveInfinity;
                foreach (Rival back in r.RivalRacer.Brain.Rivals)
                {
                    if (back.RivalRacer != this) continue;
                    timeToReachMe = back.TimeToReach;
                    break;
                }
                if (!ARS.IsBetween(timeToReachMe, 3f, 6f)) continue;
                if (r.Distance < nearestDefender) { nearestDefender = r.Distance; defenderTarget = r; }
            }
            if (defenderTarget == null) return false;

            ActiveManeuver.Type = ManeuverType.DefendLane;
            ActiveManeuver.Target = defenderTarget.RivalRacer;
            ActiveManeuver.LastEnabled = Game.GameTime;
            _defendApexNode = apexNode;
            _defendEntranceNode = entranceNode;
            return true;
        }

        // Yield card: let a faster overlapping rival by when they carry far more pressure into the entrance.
        bool TryPlayYieldCard()
        {
            if (!CardCooled(ManeuverType.Yield)) return false;
            if (Brain.Corner == null) return false;

            Rival closestRival = NearestRival();
            if (closestRival == null) return false;

            float pressureDiff = closestRival.RivalRacer.Pressure - Pressure;
            bool inOverlap = closestRival.RelativePosition == RelativePos.Left || closestRival.RelativePosition == RelativePos.Right;
            int entranceNode = CornerEntranceNode(Brain.Corner.Point, Brain.Corner.Point.Node);
            float timeToEntrance = ForwardNodeDistance(entranceNode) / Math.Max(Car.Velocity.Length(), 1f);
            if (pressureDiff <= 30f || !inOverlap || !ARS.IsBetween(timeToEntrance, 0.5f, 2f)
                || closestRival.Distance > 20f
                || closestRival.Speed <= Car.Velocity.Length()
                || !CardFree(closestRival.RivalRacer, ManeuverType.Yield)) return false;

            ActiveManeuver.Type = ManeuverType.Yield;
            ActiveManeuver.Target = closestRival.RivalRacer;
            ActiveManeuver.LastEnabled = Game.GameTime;
            return true;
        }

        float RemainingRaceDistanceMeters()
        {
            int nodeCount = ARS.TrackPoints.Count;
            if (ARS.IsPointToPoint) return nodeCount - CurrentTrackPoint.Node;
            float totalLaps = ARS.RaceMenuStore.GetInt("Laps", 6);
            return Math.Max(0f, (totalLaps + 1f - Lap) * nodeCount - CurrentTrackPoint.Node);
        }

        // Fastest completed lap, or null when none was timed. Lap 1 is never timed, for anyone.
        public TimeSpan? BestLap()
        {
            TimeSpan? best = null;
            foreach (TimeSpan lap in LapTimes)
            {
                if (!best.HasValue || lap < best.Value) best = lap;
            }
            return best;
        }

        void StartNitrous()
        {
            Function.Call(Hash.REQUEST_NAMED_PTFX_ASSET, NitrousPtfxAsset);
            Function.Call((Hash)FullyChargeNitrousHash, Car);
            Function.Call((Hash)OverrideNitrousLevelHash, Car, true, 1.0f, 1.0f, 100.0f, false);
            _nitrousActiveUntil = Game.GameTime + NitrousDurationMs;
            _nitrousLapUsed = Lap;
            // The AI's half of the story is invisible without this: the plan it is about to be pushed past, and the
            // packing factor it is escaping. Raise LogLevel to Info to read it.
            if (!ControlledByPlayer)
            {
                float planMph = Brain.CurrentIntention.Speed >= 0f ? ARS.MpsToMph(Brain.CurrentIntention.Speed) : -1f;
                ARS.Log(ARS.LogImportance.Info, "AI nitrous: " + Car.DisplayName + " lap " + Lap + ", plan " + planMph.ToString("0") + " mph, packing x" + ComputeRubberBandFactor().ToString("0.00"));
            }
        }

        public bool TryFireNitrous()
        {
            // The player's only route to the burn, and the AI's card is its twin: the same two gates and no bottle
            // bookkeeping — StartNitrous charges the bottle itself, so a fire can never be refused for want of one
            // (that gate was the bug: the player was refused in the second after every start or finish line).
            bool live = Game.GameTime < _nitrousActiveUntil;
            string refusal = live
                ? "the burn is still live"
                : Lap <= _nitrousLapUsed ? "the bottle was spent on this lap" : null;
            if (refusal != null)
            {
                if (Game.GameTime > _nitroRefusalLoggedAt)
                {
                    _nitroRefusalLoggedAt = Game.GameTime + 1000;
                    ARS.Log(ARS.LogImportance.Info, "Player nitrous refused: " + refusal);
                }
                return false;
            }
            ARS.Log(ARS.LogImportance.Info, "Player nitrous fired on lap " + Lap);
            StartNitrous();
            return true;
        }

        void StopNitrous()
        {
            Function.Call((Hash)OverrideNitrousLevelHash, Car, false, 10.0f, 0.0f, 100.0f, true);
            _nitrousActiveUntil = 0;
        }

        void UpdateYield()
        {
            if (ActiveManeuver.Type != ManeuverType.Yield) return;

            // Exit: target is >10m away
            if (ActiveManeuver.Target != null && ActiveManeuver.Target.Car.Exists())
            {
                float dist = Car.Position.DistanceTo(ActiveManeuver.Target.Car.Position);
                if (dist > 10f) EndCard();
            }
            else EndCard();
        }
        public void UpdateTickData()
        {
            Vector3 cSpeed = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, false);

            // Sample acceleration at a fixed 20ms interval (10 samples × 20ms = 200ms window).
            int now = Game.GameTime;
            int elapsed = now - VehicleData.LastAccelSampleTime;
            if (elapsed >= VehicleState.AccelIntervalMs)
            {
                VehicleData.LastAccelSampleTime = now;
                float dt = Math.Max(elapsed * 0.001f, 0.001f);
                Vector3 accel = (cSpeed - _lastSpeed) / dt;
                VehicleData.AccelSamples[VehicleData.AccelHead] = accel;
                VehicleData.AccelHead = (VehicleData.AccelHead + 1) % VehicleState.AccelWindow;
                if (VehicleData.AccelCount < VehicleState.AccelWindow) VehicleData.AccelCount++;
                _lastSpeed = cSpeed;
                VehicleData.AccumulateLapPeaks(accel, cSpeed, Car.ForwardVector);
            }

            VehicleData.SpeedVectorLocal = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true);
        }


        // Kinematic projection: pos + v*t + 0.5*a*t^2. Default t=1 (1s).
        public Vector3 ProjectAhead(float seconds = 1f)
        {
            return Car.Position + Car.Velocity * seconds + 0.5f * VehicleData.AverageAcceleration * seconds * seconds;
        }

        static Vector3 RotateZ(Vector3 v, float degrees)
        {
            float rad = degrees * (float)Math.PI / 180f;
            float cos = (float)Math.Cos(rad);
            float sin = (float)Math.Sin(rad);
            return new Vector3(v.X * cos - v.Y * sin, v.X * sin + v.Y * cos, v.Z);
        }



        // Every debug visual of the focus car, behind one gate: these blocks used to sit in ProcessTick with the
        // same focus test written out twice.
        void DrawDebug()
        {
            if (ControlledByPlayer || ARS.DebugFocusRacer != this) return;

            if (ARS.DebugToggles[Options.ShowInputs]) DrawInputTrail();
            if (ARS.DebugToggles[Options.ShowTrackAnalysis]) DrawSteerAngles();
            if (ARS.DebugToggles[Options.ShowWheelSlip]) DrawWheelSlip();
            if (ARS.DebugToggles[Options.ShowFrontSlip]) DrawFrontSlipReadout();
            if (ARS.DebugToggles[Options.ShowGsProjection]) DrawGsProjection();
        }

        const float FanLineLength = 5f;
        const float FanPursuitHeight = 0.05f;
        const float FanDamperHeight = 0.10f;
        const float FanSteerHeight = 0.15f;
        const float FanCeilingHeight = 0.20f;

        // The steer-angle fan over the roof. The model origin sits near the ground, so the roof is at the full
        // model height, not half of it — half-height floats at mid-body and the relative height varies per car.
        void DrawSteerAngles()
        {
            Vector3 origin = Car.Position + new Vector3(0, 0, VehicleData.ModelDimensions.Z + 0.2f);
            Vector3 fwd = Car.ForwardVector;

            // Yellow: pursuit angle (what the lane tracking wants)
            ARS.DrawLine(origin + new Vector3(0, 0, FanPursuitHeight), origin + new Vector3(0, 0, FanPursuitHeight) + RotateZ(fwd, _steerPursuitDeg) * FanLineLength, Color.Yellow);

            // Red: pursuit angle + damper
            ARS.DrawLine(origin + new Vector3(0, 0, FanDamperHeight), origin + new Vector3(0, 0, FanDamperHeight) + RotateZ(fwd, _steerPursuitDeg + _damperTermDeg) * FanLineLength, Color.Red);

            // White: the final slewed steer (what the wheels actually request), carrying the pedals and their limits.
            Vector3 steerBase = origin + new Vector3(0, 0, FanSteerHeight);
            Vector3 steerDir = RotateZ(fwd, Control.SteerDegrees);
            ARS.DrawLine(steerBase, steerBase + steerDir * FanLineLength, Color.White);

            // Green throttle (reverse included) and red brake: both run back-to-front as their input rises, so the
            // two spheres read against each other on one axis.
            DrawSteerLineSphere(steerBase, steerDir, FanLineLength, Math.Abs(Control.Throttle), SteerLinePedalSize, Color.Green);
            DrawSteerLineSphere(steerBase, steerDir, FanLineLength, Control.Brake, SteerLinePedalSize, Color.Red);
            DrawPedalLimits(steerBase, steerDir, FanLineLength);

            // Cyan: the limiter's ceiling in either direction, the walls the steer lines are constrained against.
            float ceilingDeg = ResolveSteerCeiling(ARS.GetForwardSpeed(Car));
            ARS.DrawLine(origin + new Vector3(0, 0, FanCeilingHeight), origin + new Vector3(0, 0, FanCeilingHeight) + RotateZ(fwd, ceilingDeg) * FanLineLength, Color.Cyan);
            ARS.DrawLine(origin + new Vector3(0, 0, FanCeilingHeight), origin + new Vector3(0, 0, FanCeilingHeight) + RotateZ(fwd, -ceilingDeg) * FanLineLength, Color.Cyan);
        }

        public void ProcessTick()
        {
            UpdateTickData();

            // The trail is only ever drawn for the focus car (DrawDebug's own gate), so sampling every racer filled a
            // ring buffer nobody would read: same condition, one place that decides.
            if (ARS.DebugToggles[Options.ShowInputs] && !ControlledByPlayer && ARS.DebugFocusRacer == this) SampleInputTrail();
            DrawDebug();

            _appliedThrottleLastFrame = VehicleMemory.GetThrottle(Car);            if (!ControlledByPlayer)
            {
                ApplyInputs();
            }

            UpdatePowerCompensation();
            ApplyPowerMultiplier();
        }

        // One sample every half metre travelled measured against the last recorded point; the oldest drops off at the cap.
        void SampleInputTrail()
        {
            Vector3 position = Car.Position;
            if (_inputTrail.Count > 0 && position.DistanceTo2D(_inputTrail[_inputTrail.Count - 1].Position) < InputTrailSampleSpacing) return;

            bool chevron = _inputTrail.Count == 0;
            if (!chevron)
            {
                _inputTrailChevronDistance += position.DistanceTo2D(_inputTrail[_inputTrail.Count - 1].Position);
                chevron = _inputTrailChevronDistance >= InputTrailChevronSpacing;
            }
            if (chevron) _inputTrailChevronDistance = 0f;

            _inputTrail.Add(new InputTrailSample { Position = position, Input = Control.Throttle - Control.Brake, Chevron = chevron });
            if (_inputTrail.Count > InputTrailMaxSamples) _inputTrail.RemoveAt(0);
        }

        void DrawInputTrail()
        {
            int sites = 0;
            for (int i = 1; i < _inputTrail.Count; i++) if (_inputTrail[i].Chevron) sites++;
            if (sites == 0) return;

            // A sample carries the car's origin, whose height over the road differs per model, so the rest height
            // cached at the grid comes off first and every car's trail sits the same distance over the tarmac.
            float restHeight = _restHeightAboveGround > 0f ? _restHeightAboveGround : 0f;
            int site = 0;
            for (int i = 1; i < _inputTrail.Count; i++)
            {
                InputTrailSample sample = _inputTrail[i];
                if (!sample.Chevron) continue;

                int fade = Math.Min(site, sites - 1 - site);
                site++;

                Vector3 heading = sample.Position - _inputTrail[i - 1].Position;
                heading.Z = 0f;
                if (heading.LengthSquared() < 0.0001f) continue;
                heading.Normalize();

                float input = ARS.Clamp(sample.Input, -1f, 1f);
                // The rotation's middle slot sits at 90 to lie the chevron flat, which leaves the first slot a yaw and
                // the last a roll - no slot left to pitch with. So the pedal tilts the direction vector instead, and
                // the rotation stays at the flat, pointing configuration every marker site shares.
                float pitchRad = input * InputTrailChevronPitch * ((float)Math.PI / 180f);
                Vector3 direction = heading * (float)Math.Cos(pitchRad) - Vector3.WorldUp * (float)Math.Sin(pitchRad);
                float groundZ = sample.Position.Z - restHeight + InputTrailChevronGroundClearance;
                Vector3 position = new Vector3(sample.Position.X, sample.Position.Y, groundZ - input * InputTrailChevronHeightGain);

                // Both ends fade over the first and last few sites, so a site arriving and the oldest leaving do not pop.
                Color colour = fade < InputTrailFadeCount ? Color.FromArgb(255 * (fade + 1) / (InputTrailFadeCount + 1), InputColour(input)) : InputColour(input);
                World.DrawMarker(MarkerType.ChevronUpx1, position, direction, new Vector3(89f, 90f, -90f), new Vector3(InputTrailChevronSize, InputTrailChevronSize, InputTrailChevronSize), colour, false, false, 2, false, "", "", false);
            }
        }

        const float SteerLinePedalSize = 0.08f;

        // One sphere at a level on the steer line: the base is no input, the tip is full, so a level reads as its
        // distance along the line whatever the sphere marks.
        void DrawSteerLineSphere(Vector3 basePt, Vector3 dir, float length, float level, float size, Color color)
        {
            World.DrawMarker(MarkerType.DebugSphere, basePt + dir * length * ARS.Clamp(level, 0f, 1f), Vector3.Zero, Vector3.Zero, new Vector3(size, size, size), color);
        }

        // A sphere over each wheel, coloured by the tyre's side slip on the axis in AGENTS/AGENTS-TECHNOTES.md: white at
        // rest, green at the traction peak, yellow at the engine's own skid line, red where the curve goes flat.
        static readonly Color SlipRestColor = Color.White;
        static readonly Color SlipPeakColor = Color.Lime;
        static readonly Color SlipSkidColor = Color.Yellow;
        static readonly Color SlipFloorColor = Color.Red;
        const float SlipSkidLevel = 2f;
        const float SlipFloorLevel = 2.5f;
        const float WheelSlipSphereHeight = 1f;
        const float WheelSlipSphereSize = 0.35f;

        // Where the trajectory starts: the engine bone, not the model's bounding box. The box is the tallest part of the shell -
        // mirrors, spoilers, roof racks - so the same line began three metres up on a truck and one and a half on a coupe, and a
        // body kit moved it. The bone is where the mass that produces the acceleration sits, and it is the same reference the
        // wheelbase already reads. The lift clears the bodywork, because the bone itself sits inside the engine bay.
        // Off road the projection's vertical component is mostly the car's own pitching, and the line jumped enough to be hard to
        // follow - the eye loses it among the bumps. Half of the vertical deviation from the origin survives: enough to read a
        // real grade, little enough that the bumps stop throwing the end of the line around.
        const float ProjectionVerticalShare = 0.5f;

        Vector3 DampedVertical(Vector3 point, float baseZ)
        {
            return new Vector3(point.X, point.Y, baseZ + (point.Z - baseZ) * ProjectionVerticalShare);
        }

        Vector3 ProjectionOrigin()
        {
            const float lift = 0.2f;
            if (Car.HasBone("engine")) return Car.GetBoneCoord("engine") + Vector3.WorldUp * lift;
            if (Car.HasBone("chassis")) return Car.GetBoneCoord("chassis") + Vector3.WorldUp * lift;
            // Bikes and trailers have neither; the roof is the honest last resort.
            return Car.Position + Vector3.WorldUp * (VehicleData.ModelDimensions.Z + lift);
        }

        void DrawGsProjection()
        {
            // The path hangs off the engine bone rather than the model box, so its height is a property of the car's mass
            // and not of its tallest trim.
            Vector3 origin = ProjectionOrigin();
            float lift = origin.Z - Car.Position.Z;
            Vector3 half = DampedVertical(ProjectAhead(0.5f) + Vector3.WorldUp * lift, origin.Z);
            Vector3 full = DampedVertical(ProjectAhead(1f) + Vector3.WorldUp * lift, origin.Z);
            // One colour for both halves, decided in the tick that measured them: green fading to white while the inside floor
            // holds, red when the path runs wide, yellow when it runs off the road but cuts inside, blue otherwise.
            ARS.DrawLine(origin, half, _projectionPathColor);
            ARS.DrawLine(half, full, _projectionPathColor);
        }

        // Both front wheels' measured side slip in degrees against the limit, and the factor the request is being scaled by
        // this tick. 1.00 leaves the steering alone; 0.60 removes two fifths of it.
        void DrawFrontSlipReadout()
        {
            // One dimensionless number: what the car asked for over what the ceiling allows. 1.00 means the clamp is just
            // touching, 2.00 means the request was twice the allowed angle, and the value falls below 1 once it stops biting.
            // Theory source: the only number worth seeing is whether the applied clamp has arrived at the target. Over 100% the
            // gate is still closing onto it; 100% means it sits where the law says. Nothing else is printed in this mode.
            if (!ARS.SlipFromMemory)
            {
                float target = _clampTargetDeg;
                float applied = _chasingClampDeg;
                string proportion = (float.IsNaN(target) || float.IsNaN(applied) || target < 0.01f)
                    ? "--"
                    : (100f * applied / target).ToString("0") + "%";
                UI.ShowSubtitle(proportion, 50);
                return;
            }

            float clamp = float.IsNaN(_chasingClampDeg) ? _finalCeilingDeg : _chasingClampDeg;
            float asked = Math.Abs(_steerPreLimitDegrees);
            float mph = ARS.MpsToMph(Math.Abs(Vector3.Dot(Car.Velocity, Car.ForwardVector)));
            float share = Math.Max(0.05f, ARS.SteerCeilingFactor) * SlowCornerSlipShare(ARS.MphToMps(mph));
            UI.ShowSubtitle((clamp > 0.01f ? asked / clamp : 0f).ToString("0.00") + "   "
                + _slipFactor.ToString("0.00") + "   " + mph.ToString("0") + "mph"
                + "   F " + _slip.InnerPct.ToString("0") + "   R " + _slip.RearPct.ToString("0")
                // The percentages are normalised against the share in force: the menu's factor times the slow-corner allowance. The token
                // appears whenever that is not 1.0, so F 100 never silently means something other than the tyre's peak.
                + (Math.Abs(share - 1f) > 0.001f ? "   x" + share.ToString("0.0") : ""), 50);
        }

        void DrawWheelSlip()
        {
            List<float> slips = ARS.WheelSideSlipAngles(Car);
            if (slips.Count < 4) return;

            float halfWidth = VehicleData.ModelDimensions.X * 0.5f;
            float halfBase = VehicleData.WheelBase * 0.5f;
            for (int i = 0; i < 4; i++)
            {
                float lateral = i % 2 == 0 ? -halfWidth : halfWidth;
                float longitudinal = i < 2 ? halfBase : -halfBase;
                Vector3 position = Car.Position + Car.ForwardVector * longitudinal + Car.RightVector * lateral + Vector3.WorldUp * WheelSlipSphereHeight;
                World.DrawMarker(MarkerType.DebugSphere, position, Vector3.Zero, Vector3.Zero, new Vector3(WheelSlipSphereSize, WheelSlipSphereSize, WheelSlipSphereSize), SlipColor(slips[i]));
            }
        }

        static Color SlipColor(float slip)
        {
            slip = Math.Abs(slip);
            if (slip <= 1f) return BlendColor(SlipRestColor, SlipPeakColor, slip);
            if (slip <= SlipSkidLevel) return BlendColor(SlipPeakColor, SlipSkidColor, slip - 1f);
            return BlendColor(SlipSkidColor, SlipFloorColor, (slip - SlipSkidLevel) / (SlipFloorLevel - SlipSkidLevel));
        }

        static Color BlendColor(Color from, Color to, float t)
        {
            t = ARS.Clamp(t, 0f, 1f);
            return Color.FromArgb((int)(from.A + (to.A - from.A) * t), (int)(from.R + (to.R - from.R) * t), (int)(from.G + (to.G - from.G) * t), (int)(from.B + (to.B - from.B) * t));
        }

        // The reason limits ride the white steer line with the applied pedals, so every sphere on the line shares
        // one scale: the base is no input and the tip is full. A white sphere is an override command; a coloured
        // sphere is a per-reason limit, drawn only where it bites. Highest level goes down first, so the binding
        // cap paints last.
        const float PedalLimitReasonSize = 0.1f;
        const float PedalLimitCapSize = 0.09f;
        // Etiquette limits (rival, chill-out, yield) are harmless, so they read cool; grip limits yellow; the
        // overspeed cut black; countersteer orange; stability violet; the recovery coast cyan.
        static readonly Color NonDangerousReasonColor = Color.FromArgb(255, 120, 200, 255);
        static readonly Color GripReasonColor = Color.Yellow;
        static readonly Color OverspeedReasonColor = Color.Black;
        static readonly Color CountersteerReasonColor = Color.Orange;
        static readonly Color StabilityReasonColor = Color.FromArgb(255, 190, 80, 255);
        static readonly Color RecoveryReasonColor = Color.Cyan;
        // The lane line's own hues, plus the one colour for a bound rather than a source: a wall clamp and the debug
        // pin are not opinions about where to go, so they must not read as one. The commit card takes the hue furthest
        // from both corner phases, because a card ending and a corner blending can alternate within a second.
        static readonly Color LaneReasonOutsideColor = Color.Orange;
        static readonly Color LaneReasonApexColor = Color.FromArgb(255, 255, 60, 200);
        static readonly Color LaneReasonCommitColor = Color.FromArgb(255, 0, 200, 120);
        static readonly Color LaneReasonBoundColor = Color.FromArgb(255, 170, 170, 170);

        // The pedal palette is reused only where the meaning is the same; the rest are the lane line's own, and the
        // name drawn at the line is what settles a pair that hue alone cannot separate.
        public static Color LaneReasonColor(LaneReason reason)
        {
            switch (reason)
            {
                case LaneReason.CurveLean: return GripReasonColor;
                case LaneReason.OutsideLine: return LaneReasonOutsideColor;
                case LaneReason.ApexLine: return LaneReasonApexColor;
                case LaneReason.CommitCard: return LaneReasonCommitColor;
                case LaneReason.Avoidance: return NonDangerousReasonColor;
                case LaneReason.RecoveryEdge: return RecoveryReasonColor;
                case LaneReason.WallClamp: return LaneReasonBoundColor;
                case LaneReason.DebugLock: return LaneReasonBoundColor;
            }
            return Color.Red;
        }

        struct PedalLimitSphere
        {
            public float Level;
            public Color Color;
            public float Size;
        }

        readonly PedalLimitSphere[] _pedalLimits = new PedalLimitSphere[12];

        void DrawPedalLimits(Vector3 basePt, Vector3 dir, float length)
        {
            int count = 0;
            AddPedalLimit(Control.MaxThrottleFromTCS, GripReasonColor, PedalLimitReasonSize, ref count);
            AddPedalLimit(Control.MaxThrottleFromStability, StabilityReasonColor, PedalLimitReasonSize, ref count);
            AddPedalLimit(Control.MaxThrottleFromOverspeed, OverspeedReasonColor, PedalLimitReasonSize, ref count);
            AddPedalLimit(Control.MaxThrottleFromRival, NonDangerousReasonColor, PedalLimitReasonSize, ref count);
            AddPedalLimit(Control.MaxThrottleFromChillOut, NonDangerousReasonColor, PedalLimitReasonSize, ref count);
            AddPedalLimit(Control.MaxThrottleFromYield, NonDangerousReasonColor, PedalLimitReasonSize, ref count);
            AddPedalLimit(Control.MaxThrottleFromOffTrack, Color.White, PedalLimitReasonSize, ref count);
            AddPedalLimit(Control.MaxThrottleFromRecovery, RecoveryReasonColor, PedalLimitReasonSize, ref count);
            AddPedalLimit(Control.MaxBrakeFromABS, GripReasonColor, PedalLimitReasonSize, ref count);
            AddPedalLimit(Control.MaxBrakeFromCountersteer, CountersteerReasonColor, PedalLimitReasonSize, ref count);
            if (IsFullCountersteer()) AddPedalLimit(CountersteerPedalCap, Color.White, PedalLimitCapSize, ref count);
            if (_offtrackThrottleHeld) AddPedalLimit(Math.Max(1f - ProjectionInputPerSecond * _projectionOutsideSeconds, 0f), Color.White, PedalLimitCapSize, ref count);

            SortPedalLimits(count);
            for (int i = 0; i < count; i++)
                DrawSteerLineSphere(basePt, dir, length, _pedalLimits[i].Level, _pedalLimits[i].Size, _pedalLimits[i].Color);
        }

        // A limit at full authority is not limiting anything, so it is not queued.
        void AddPedalLimit(float level, Color color, float size, ref int count)
        {
            if (level >= 1f || count >= _pedalLimits.Length) return;
            _pedalLimits[count].Level = level;
            _pedalLimits[count].Color = color;
            _pedalLimits[count].Size = size;
            count++;
        }

        void SortPedalLimits(int count)
        {
            for (int i = 1; i < count; i++)
            {
                PedalLimitSphere cap = _pedalLimits[i];
                int j = i - 1;
                while (j >= 0 && _pedalLimits[j].Level < cap.Level)
                {
                    _pedalLimits[j + 1] = _pedalLimits[j];
                    j--;
                }
                _pedalLimits[j + 1] = cap;
            }
        }

        // Full throttle green, neutral yellow, full brake red.
        static Color InputColour(float input)
        {
            float v = ARS.Clamp(input, -1f, 1f);
            return Color.FromArgb((int)(255f * (1f - Math.Max(v, 0f))), (int)(255f * (1f + Math.Min(v, 0f))), 0);
        }

        public void RunTimedCore()
        {
            UpdateTrackPosition();
            UpdateSlideAndBoundingBox();
            UpdatePerceivedGrip();
            UpdateTRLateralAtSpeed();

            ProcessAI();

            _lastCoreTick = Game.GameTime;
        }



        void UpdateSlideAndBoundingBox()
        {
            VehicleData.BoundingBox = ARS.SlidingBoundingBoxWidth(Car);
            // A stationary car has no direction of travel: Normalized() of a zero velocity is NaN, SignedAngle carries it, and
            // the first Math.Sign that reads SlideAngle throws out of the tick (38 OnTick errors in two minutes on build 958).
            VehicleData.SlideAngle = Car.Velocity.LengthSquared() > 0.01f
                ? (float)Math.Round(Vector3.SignedAngle(new Vector3(Car.Velocity.X, Car.Velocity.Y, 0f).Normalized, new Vector3(Car.ForwardVector.X, Car.ForwardVector.Y, 0f).Normalized, Car.UpVector), 3)
                : 0f;
        }

        void UpdatePassengerSeat()
        {
            if (ControlledByPlayer) return;

            float myHalfLen = Math.Abs(VehicleData.ModelDimensions.Y) * 0.5f;

            bool shouldPassengerize = false;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null) continue;
                float rivalHalfLen = Math.Abs(r.RivalRacer.VehicleData.ModelDimensions.Y) * 0.5f;
                float threshold = myHalfLen + rivalHalfLen + 0.5f;
                if (r.Distance < threshold)
                {
                    shouldPassengerize = true;
                    break;
                }
            }

            if (shouldPassengerize && !_isPassengerized)
            {
                try { Driver.SetIntoVehicle(Car, VehicleSeat.Passenger); _isPassengerized = true; } catch (Exception) { }
            }
            else if (!shouldPassengerize && _isPassengerized)
            {
                try { Driver.SetIntoVehicle(Car, VehicleSeat.Driver); _isPassengerized = false; } catch (Exception) { }
            }
        }

        // A flooded engine is off, not dead: the game cuts it in water and never turns it back on, so without this an
        // AI car sits immovable on the throttle forever. Health gates it, so a wreck is not raised.
        void EnsureEngineRunning()
        {
            bool engineRunning = Function.Call<bool>((Hash)0xAE31E7DF9B5B132E, Car);   // GET_IS_VEHICLE_ENGINE_RUNNING
            if (Car.EngineHealth > 0f && !engineRunning) Function.Call(Hash.SET_VEHICLE_ENGINE_ON, Car, true, true, false);
        }

        public void ApplyInputs()
        {


            if (Driver.IsSittingInVehicle(Car))
            {
                UpdatePassengerSeat();
                EnsureEngineRunning();

                Car.HandbrakeOn = Control.HandBrakeTime > Game.GameTime;

                // The pedal goes out whole, 1.00 included, and that is deliberate. GTA trims grip at a literal full
                // throttle - the engine's own launch behaviour - and a 0.99 cap used to sit here to keep the car out of
                // that range. It is disabled: the cap was also the last thing between the law and the tyre, and the
                // driver wants the native behaviour gone rather than dodged.
                VehicleMemory.SetThrottle(Car, ARS.Clamp(Control.Throttle, -1, 1));
                VehicleMemory.SetBrakes(Car, ARS.Clamp(Control.Brake, 0f, 1f));
                VehicleMemory.SetSteerAngle(Car, Control.SteerInput);

            }
            else
            {
                VehicleMemory.SetThrottle(Car, 0f);
                VehicleMemory.SetBrakes(Car, 0f);
                VehicleMemory.SetSteerInput(Car, 0f);
            }
        }

        // The engine resets the cheat multiplier every physics step, so this re-writes it each frame — the single
        // per-frame writer, with nitro riding on the same write. Force side only: a fade is a multiplication so its
        // inverse restores it, whereas a cut to zero cannot be, and grip-side losses only add wheelspin.
        void UpdatePowerCompensation()
        {
            // This channel can only take power AWAY. Exactly one reason at a time may ask for more than stock - the burn or
            // artificial rubberbanding - and both of them set the multiplier outright in ApplyPowerMultiplier. The pedal gap
            // and the over-top-speed fade used to inflate this counter, so three features multiplied through one number.
            float counter = 1f;

            // The player's side of the Gs correction. Their pedal is theirs, so the same level is taken out of the drive
            // force instead: for an input the engine reads as 1.0 this is as close to capping it as the mod can get.
            if (ControlledByPlayer)
            {
                _overspeedPowerLevel = GlideCap(_overspeedPowerLevel, OverspeedThrottleLevel());
                counter *= _overspeedPowerLevel;
            }

            _powerCounter = counter;

            if (ControlledByPlayer && _overspeedPowerLevel < 0.99f && Game.GameTime > _powerCounterLoggedAt)
            {
                _powerCounterLoggedAt = Game.GameTime + 2000;
                ARS.Log(ARS.LogImportance.Info, "Player power cut " + _overspeedPowerLevel.ToString("0.00") + " (excess Gs " + VehicleData.OverspeedExcessGs.ToString("0.00") + ")");
            }
            // The counter can no longer exceed 1, so the 'power counter raised' line it used to print has nothing left to
            // report: a boost is the burn or the rubberbanding now, and both announce themselves.
        }

        void ApplyPowerMultiplier()
        {
            // One reason at a time. The burn is its own multiplier; artificial rubberbanding is its own, and only for an AI
            // car; everything else can only be _powerCounter, which is <= 1. They never multiply with each other, so the
            // number the car gets on this tick is a number someone chose.
            float multiplier = _powerCounter;
            if (Game.GameTime < _nitrousActiveUntil)
            {
                multiplier = ControlledByPlayer
                    ? PlayerNitrousPowerMultiplier
                    : NitrousPowerMultiplier * ARS.AiNitroPct * 0.01f;
            }
            else if (!ControlledByPlayer && ARS.RubberbandingPct > 0 && ARS.CurrentRubberbandMode == RubberbandMode.Artificial)
            {
                float rbTorque = ComputeRubberBandFactor();
                if (rbTorque > 1f) multiplier *= rbTorque;
            }

            if (float.IsNaN(multiplier) || float.IsInfinity(multiplier) || multiplier < 0f || multiplier > 10f) multiplier = 1f;

            // The burn's own readout, independent of every setting: the power asked for, the pedal that receives it and
            // the three caps trimming that pedal. A small burn is a pedal story long before it is a power story.
            if (Game.GameTime < _nitrousActiveUntil && Game.GameTime > _nitrousGsLoggedAt)
            {
                _nitrousGsLoggedAt = Game.GameTime + 500;
                string planMph = Brain.CurrentIntention.Speed >= 0f ? ARS.MpsToMph(Brain.CurrentIntention.Speed).ToString("0") : "?";
                ARS.Log(ARS.LogImportance.Info, "Burn " + Car.DisplayName + (ControlledByPlayer ? " (player)" : " (AI)")
                    + ", power x" + multiplier.ToString("0.00")
                    + ", throttle " + Control.Throttle.ToString("0.00")
                    + " caps TCS " + Control.MaxThrottleFromTCS.ToString("0.00")
                    + " stab " + Control.MaxThrottleFromStability.ToString("0.00")
                    + " over " + Control.MaxThrottleFromOverspeed.ToString("0.00")
                    + ", plan " + planMph + " mph, speed " + ARS.MpsToMph(Car.Velocity.Length()).ToString("0")
                    + " mph, slip " + ARS.MaxWheelSlip(Car).ToString("0.00"));
            }

            Function.Call((Hash)CheatPowerIncreaseHash, Car, multiplier);
        }

        bool IsAwd()
        {
            float bias = VehicleMemory.GetDriveBiasFront(Car);
            return bias > 0.01f && bias < 0.99f;
        }

        int OffsetCornerNode(int node, int offset)
        {
            int count = ARS.TrackPoints.Count;
            if (count == 0) return -1;

            if (ARS.IsPointToPoint)
                return (int)ARS.Clamp(node + offset, 0, count - 1);

            int wrapped = (node + offset) % count;
            return wrapped < 0 ? wrapped + count : wrapped;
        }


        float OutOfTrackDistance()
        {
            return Math.Abs(Brain.CurrentPerception.CurrentNodeOffset) - CurrentTrackPoint.TrackHalfWidth;
        }

        void UpdateRivalInfo()
        {
            Brain.AvoidanceTarget = null;
            foreach (Rival r in Brain.Rivals)
            {
                r.Update(this);
                // No heading test: whether a car is in the way is a question about its body, not about where it points. A spun
                // rival in this car's corridor is exactly the thing to go round, and TimeToReach already refuses any rival that
                // is outside that corridor, so the gate was filtering out targets without protecting anything.
                bool isAvoidanceCandidate = r.RelativePosition == RelativePos.Ahead && ARS.IsBetween(r.RouteGapAhead, 0f, SameSectionMaxGapMeters) && (ARS.IsBetween(r.FrontGap, 0f, 3f) || ARS.IsBetween(r.TimeToReach, 0f, 5f));
                if (Brain.AvoidanceTarget == null && isAvoidanceCandidate)
                {
                    Brain.AvoidanceTarget = r;
                }
            }
        }

        public void InitializeTrackPosition()
        {
            if (ARS.TrackPoints.Count == 0) return;

            TrackPoint closestPoint = ARS.TrackPoints[0];
            Vector3 carPosition = Car.Position;
            float closestDistance = closestPoint.Position.DistanceTo(carPosition);
            foreach (TrackPoint point in ARS.TrackPoints)
            {
                float distance = point.Position.DistanceTo(carPosition);
                if (distance < closestDistance)
                {
                    closestPoint = point;
                    closestDistance = distance;
                }
            }

            CurrentTrackPoint = closestPoint;
            UpdateRaceProgress();
        }

        void UpdateRaceProgress()
        {
            RaceProgress = Lap * ARS.TrackPoints.Count + CurrentTrackPoint.Node;
        }

        public void UpdateTrackPosition()
        {


            int refTrackpoint = (int)ARS.Clamp(CurrentTrackPoint.Node, 0, ARS.TrackPoints.Count - 1);

            _trackPositionScratch.Clear();
            int lastNode = ARS.TrackPoints.Count - 1;
            int firstCandidate = Math.Max(refTrackpoint - 6, 0);
            int lastCandidate = Math.Min(refTrackpoint + 6, lastNode);
            for (int i = firstCandidate; i <= lastCandidate; i++)
            {
                _trackPositionScratch.Add(ARS.TrackPoints[i]);
            }

            if (!ARS.IsPointToPoint)
            {
                if (refTrackpoint <= 6)
                {
                    for (int i = Math.Max(lastNode - 6, 0); i <= lastNode; i++)
                        _trackPositionScratch.Add(ARS.TrackPoints[i]);
                }
                else if (refTrackpoint >= lastNode - 6)
                {
                    for (int i = 0; i <= Math.Min(6, lastNode); i++)
                        _trackPositionScratch.Add(ARS.TrackPoints[i]);
                }
            }

            Vector3 carPosition = Car.Position;
            TrackPoint closestPoint = _trackPositionScratch[0];
            float closestDistance = closestPoint.Position.DistanceTo(carPosition);
            foreach (TrackPoint point in _trackPositionScratch)
            {
                float distance = point.Position.DistanceTo(carPosition);
                if (distance < closestDistance)
                {
                    closestPoint = point;
                    closestDistance = distance;
                }
            }

            float reacquireDistance = Math.Max(CurrentTrackPoint.TrackHalfWidth + 10f, 25f);
            if (closestDistance > reacquireDistance)
            {
                foreach (TrackPoint point in ARS.TrackPoints)
                {
                    float distance = point.Position.DistanceTo(carPosition);
                    if (distance < closestDistance)
                    {
                        closestPoint = point;
                        closestDistance = distance;
                    }
                }
            }

            CurrentTrackPoint = closestPoint;
            Brain.CurrentPerception.CurrentNodeOffset = ARS.SignedLaneOffset(carPosition, CurrentTrackPoint.Position, CurrentTrackPoint.Direction);

            LookAheads.Clear();
            Vector3 velocity = Car.Velocity;
            float speed = velocity.Length();
            AlongTrackSpeed = Vector3.Dot(velocity, CurrentTrackPoint.Direction);

            int steerRef = (int)ARS.Clamp(speed * SteerPreviewSeconds, SteerLookaheadMinMeters, SteerLookaheadMaxMeters);
            int quarterSec = (int)(speed * 0.25f);
            int halfSec = (int)(speed * 0.5f);
            int threeQuarterSec = (int)(speed * 0.75f);
            int oneSec = (int)(speed);
            int oneHalfSec = (int)(speed * 1.5f);
            int twoSec = (int)(speed * 2f);

            TrackPoint ResolveLookAhead(int offset)
            {
                int node = CurrentTrackPoint.Node + offset;
                if (ARS.IsPointToPoint) return ARS.TrackPoints[Math.Min(node, lastNode)];
                return ARS.TrackPoints[node % ARS.TrackPoints.Count];
            }

            LookAheads[LookAhead.SteerRef] = ResolveLookAhead(steerRef);
            LookAheads[LookAhead.QuarterSec] = ResolveLookAhead(quarterSec);
            LookAheads[LookAhead.HalfSec] = ResolveLookAhead(halfSec);
            LookAheads[LookAhead.ThreeQuarterSec] = ResolveLookAhead(threeQuarterSec);
            LookAheads[LookAhead.OneSec] = ResolveLookAhead(oneSec);
            LookAheads[LookAhead.OneHalfSec] = ResolveLookAhead(oneHalfSec);
            LookAheads[LookAhead.TwoSec] = ResolveLookAhead(twoSec);




            int nodeCount = ARS.TrackPoints.Count;
            int currentNode = CurrentTrackPoint.Node;
            float currentPct = ARS.GetPercent(currentNode, nodeCount);
            float previousPct = _previousNode >= 0 ? ARS.GetPercent(_previousNode, nodeCount) : 0f;
            bool wrappedStartLine = !ARS.IsPointToPoint && _previousNode >= 0 && previousPct > 90f && currentPct < 10f;
            UpdateLapRegistration(currentNode, nodeCount, currentPct, wrappedStartLine);

            UpdateRaceProgress();

            _previousNode = currentNode;

            // Route radius from three sample points.
            Brain.CurrentPerception.CurveRadiusToFollowPoint = RouteRadiusSampled();
            UpdateApexLeapfrog();
            // Requirements run ahead of the refill and force one on a flip, so a corner that has just asked for a
            // plan gets it this tick rather than up to 500 ms later.
            if (UpdateCornerRequirements() || _apexUpdateTick + _phaseOffsetMs < Game.GameTime)
            {
                _apexUpdateTick = Game.GameTime + 500;
                RefillApexQueue();
            }
            // High-speed lane radius: short 0.5s to 1.0s window.
            Brain.CurrentPerception.HighSpeedCurveRadius = ComputeRouteRadius((int)(speed * 0.5f), (int)(speed * 1.0f));
        }

        // The lap counter: closed by crossing the line, and reopened mid-track so the next crossing can register.
        void UpdateLapRegistration(int currentNode, int nodeCount, float currentPct, bool wrappedStartLine)
        {
            if (CanRegisterNewLap)
            {
                if (wrappedStartLine || (ARS.IsPointToPoint && currentPct > 99f && ARS.EntityRelativeOffset(Car, ARS.TrackPoints.Last().Position).Y < 0f))
                {
                    CanRegisterNewLap = false;
                    Lap++;
                    ARS.Log(ARS.LogImportance.Info, "Lap++ " + Name + " -> lap " + Lap + " (node " + currentNode + ")");
                    if (Lap > ARS.RaceMenuStore.GetInt("Laps", 6))
                    {
                        if (Car.CurrentBlip != null) Car.CurrentBlip.Color = BlipColor.Green;
                    }

                    if (Lap == 2 && !ARS.IsPointToPoint)
                    {
                        LapStartTime = Game.GameTime;
                        VehicleData.ResetLapPeaks();
                    }
                    else if (Lap > 1)
                    {
                        TimeSpan lapTime = ARS.ParseToTimeSpan(Game.GameTime - LapStartTime);
                        string peaks = "accel " + VehicleData.PeakAccelG.ToString("0.00") + "G decel " + Math.Abs(VehicleData.PeakDecelG).ToString("0.00") + "G lat " + VehicleData.PeakLateralG.ToString("0.00") + "G top " + ARS.MpsToMph(VehicleData.PeakTopSpeedMps).ToString("0") + "mph";
                        ARS.Log(ARS.LogImportance.Info, "Laptime " + CarModelName + ": " + lapTime.ToString("m':'ss'.'f") + " | " + peaks);
                        LapTimes.Add(lapTime);
                        if (ARS.DebugToggles[Options.ShowAiLapTimes]) UI.Notify("~b~[ARS]~w~ " + Name + " lap " + (Lap - 1) + ": " + lapTime.ToString("m':'ss'.'f"));
                        LapStartTime = Game.GameTime;
                        VehicleData.ResetLapPeaks();
                    }
                }
            }
            else if (BaseBehavior == RacerBaseBehavior.Race && ARS.IsBetween(currentPct, 40f, 60f))
            {
                CanRegisterNewLap = true;
            }
        }

        // Circumradius through three route-window sample points.
        // Four points, not three. The exact circumradius is dominated by the angle at its middle point, and near
        // collinearity that angle makes the radius hypersensitive: one node moving a metre swings it hundreds of
        // metres, which the route speed turns into a step. Four points fitted by least squares spread that influence,
        // so the reading moves with the road instead of with one vertex. The car's own node is still the first sample
        // and the farthest is still twice the per-grip lookahead, as before; the two middles split that span into
        // thirds so the fit sees the shape of the bend rather than one apex angle.
        float RouteRadiusSampled()
        {
            int count = ARS.TrackPoints.Count;
            if (count < 10) return 999f;

            float traction = Math.Max(VehicleData.CurrentMechanicalGrip, 0.1f);
            // Grip is floored at 0.1, so an unclamped span can be twenty times the speed in nodes and wrap past the
            // start line on a short track - the four samples then stop being monotone in road order, which the
            // thirds-split assumption needs. Clamp to the route.
            int far = Math.Min((int)(Car.Velocity.Length() * 2f / traction), count - 1);
            int middleNear = far / 3;
            int middleFar = (far * 2) / 3;

            int carNode = CurrentTrackPoint.Node;
            int n0 = WrappedNode(carNode);
            int n1 = WrappedNode(carNode + middleNear);
            int n2 = WrappedNode(carNode + middleFar);
            int n3 = WrappedNode(carNode + far);

            LiveRouteRadiusNearNode = n1;
            LiveRouteRadiusMidNode = n2;
            LiveRouteRadiusFarNode = n3;

            Vector3 p0 = ARS.TrackPoints[n0].Position;
            Vector3 p1 = ARS.TrackPoints[n1].Position;
            Vector3 p2 = ARS.TrackPoints[n2].Position;
            Vector3 p3 = ARS.TrackPoints[n3].Position;

            float r;
            float chord = Vector2.Distance(new Vector2(p0.X, p0.Y), new Vector2(p3.X, p3.Y));
            if (middleNear == 0 || middleFar == middleNear || chord < 1f)
            {
                // A slow car collapses the thirds onto its own node and leaves a duplicated vertex; the span is too
                // short to say anything, and the exact solve over it is just as singular. Straight is the honest read.
                r = 999f;
            }
            else if (Sagitta(p0, p1, p2, p3, chord) < RouteStraightSagittaShare * chord)
            {
                // Four points this close to their own chord describe no circle a car can turn on: the fit's answer
                // would be the RMS size of the sampling window, not a radius. Below this share a straight (999) is
                // the only honest number, and it is above any car's top speed once clamped.
                r = 999f;
            }
            else
            {
                r = TrackLoader.FittedCircleRadius(p0, p1, p2, p3);
                if (r >= 999f) r = ARS.Circumradius3D(p0, p3, p2);
            }

            if (float.IsNaN(r) || float.IsInfinity(r)) r = 999f;
            LiveRouteRadius = ARS.Clamp(r, 5f, 999f);
            return LiveRouteRadius;
        }

        // How far the two middle samples sit off the chord joining the outer two, normalised by that chord. It is the
        // geometric test for "these four points describe a straight", and it is deliberately not a test on the fit
        // residual or on the normal equations: on a uniform node spacing the samples are centrally symmetric and the
        // algebraic fit's curvature terms cancel exactly, while the matrix stays perfectly conditioned.
        float Sagitta(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float chord)
        {
            float dx = (p3.X - p0.X) / chord;
            float dy = (p3.Y - p0.Y) / chord;
            float near = Math.Abs((p1.X - p0.X) * dy - (p1.Y - p0.Y) * dx);
            float far = Math.Abs((p2.X - p0.X) * dy - (p2.Y - p0.Y) * dx);
            return Math.Max(near, far);
        }

        int WrappedNode(int node)
        {
            int count = ARS.TrackPoints.Count;
            return ARS.IsPointToPoint
                ? (int)ARS.Clamp(node, 0, count - 1)
                : ((node % count) + count) % count;
        }

        // Braking map close-corner filtering and entrance timing.
        const float ApexBufferSeconds = 5f;
        const float EntranceBrakeBufferSeconds = 0.25f;
        const float EntranceBrakeExtraDistance = 0f;
        const float SecondaryApexSpeedDifference = 5f;

        // Cheap: drop passed apexes and invalidate stale entries every tick.
        void UpdateApexLeapfrog()
        {
            int[] heldNodes = { NextApexNode, NextApexNode2, NextApexNode3 };
            float[] heldRadii = { NextApexRadius, NextApexRadius2, NextApexRadius3 };

            bool heldTableChanged = heldNodes.Any(node => node >= 0 && !ARS.Corners.Any(corner => corner.Node == node));
            bool lowSpeedInvalidation = heldNodes[0] >= 0 && Car.Velocity.Length() < NextApexSpeed * 0.5f;
            if (heldTableChanged || lowSpeedInvalidation)
            {
                for (int i = 0; i < heldNodes.Length; i++) heldNodes[i] = -1;
            }
            else
            {
                // Leapfrog passed apexes forward through the held queue.
                int shift = 0;
                while (shift < heldNodes.Length && heldNodes[shift] >= 0 && HasPassedApex(heldNodes[shift])) shift++;
                if (shift > 0) ScheduleBrakeCommit();
                if (shift > 0 && ControlledByPlayer) Tips.ApexPassed(shift);
                if (shift > 0)
                {
                    for (int i = 0; i < heldNodes.Length - shift; i++)
                    {
                        heldNodes[i] = heldNodes[i + shift];
                        heldRadii[i] = heldRadii[i + shift];
                    }
                    for (int i = heldNodes.Length - shift; i < heldNodes.Length; i++)
                    {
                        heldNodes[i] = -1;
                        heldRadii[i] = 999f;
                    }
                }
            }

            CommitApexQueue(heldNodes, heldRadii);
        }

        // Expensive: scan all corners and refill empty queue slots. Gated to 0.5s.
        // Decide, per car and per corner, whether the corner wants a braking plan and an outside line. It reads the
        // car's own predicted arrival against its own apex speed, so it never depends on the held queue — a corner
        // the queue has not selected still gets evaluated, which is what lets a flag turn on in the first place.
        // The two flags answer different questions and so are judged on different axes: braking is a v²/2a distance
        // problem and is tested from the road it actually needs, while positioning is a lateral move and is tested
        // inside the time window the move belongs in.
        bool UpdateCornerRequirements()
        {
            float speed = Math.Max(Car.Velocity.Length(), 1f);
            float forwardGs = VehicleData.GetLongitudinalGs(Car.ForwardVector);
            bool flipped = false;

            foreach (CornerPoint corner in ARS.Corners)
            {
                int entranceNode = corner.StartNode >= 0 ? corner.StartNode : OffsetCornerNode(corner.Node, -corner.LengthStart);
                int distance = ForwardNodeDistance(entranceNode);
                if (!TryGetCornerContext(corner.Node, out CornerContext context)) continue;

                // The skip flag clears once the apex is behind; braking is not cleared, because its range is measured to the
                // entrance and the flag must outlive it to hold the plan.
                if (HasPassedApex(corner.Node))
                {
                    _skippedApexes.Remove(corner.Node);
                }

                if (distance <= 0) continue;

                bool tight = corner.DetectedRadius < CornerTightRadius;
                // The intended speed must be the one the plan itself will target, or the flag reads a wider corner
                // than the car is actually asked to take and never fires. That is the smoothed radius, which is the
                // one the speed path reads.
                float intended = ApexSpeedWithDownforce(corner.DetectedRadius);
                float timeToEntrance = distance / speed;
                float decel = Math.Max(BrakingDecelBase(corner.Node), 0.1f);
                float brakingDistance = (speed * speed - intended * intended) / (2f * decel);
                bool inBrakingRange = distance <= brakingDistance + BrakeHorizonMargin;

                // A corner too wide to ask for positioning that has not asked for braking by the time the car is
                // inside its window is one the car is passing through: it is flagged here and the picker looks past
                // it so the corner behind is promoted from further out. The flag holds until the apex is behind, and
                // the arming below revokes it the same pass a braking plan is asked for — an armed corner is never
                // skipped. A flag landing forces this tick's refill, or the picker would switch a gate late.
                if (!tight && !context.RequiresBraking && timeToEntrance <= RequirementLookaheadSeconds)
                {
                    if (_skippedApexes.Add(corner.Node))
                    {
                        flipped = true;
                        ARS.Log(ARS.LogImportance.Info, "Queue skip " + Name + ": corner " + corner.Node + " ignored (wide "
                            + corner.DetectedRadius.ToString("0") + " m, no braking asked), the picker looks past it");
                    }
                }

                if (!inBrakingRange) continue;

                float arrival = speed + forwardGs * Handling.Gravity * timeToEntrance * RequirementExtrapolationScale;
                float excess = arrival - intended;

                // A tight corner that wants positioning is braking regardless, because the held queue is the only
                // route to an outside line — so its braking threshold is the lower one.
                if (!context.RequiresBraking && inBrakingRange && excess > ARS.MphToMps(tight ? PositionExcessMph : BrakeExcessMph))
                {
                    context.RequiresBraking = true;
                    _skippedApexes.Remove(corner.Node);
                    flipped = true;
                }
            }
            return flipped;
        }

        void RefillApexQueue()
        {
            int count = ARS.TrackPoints.Count;
            if (count < 10 || ARS.Corners.Count == 0)
            {
                CommitApexQueue(new[] { -1, -1, -1 }, new[] { 999f, 999f, 999f });
                return;
            }

            List<int> upcoming = new List<int>();
            for (int i = 0; i < ARS.Corners.Count; i++)
            {
                if (ForwardNodeDistance(ARS.Corners[i].Node) > 0) upcoming.Add(i);
            }
            upcoming.Sort((left, right) => ForwardNodeDistance(ARS.Corners[left].Node).CompareTo(ForwardNodeDistance(ARS.Corners[right].Node)));

            // Derived fresh every refill and never carried over: slot one must be the nearest corner the car is not
            // already passing through, or the line, the commit lane and the chevron aim past it the moment the other
            // slots are occupied. A corner the scan flagged as worth nothing this approach is looked past, so the one
            // behind it becomes slot one while the car is still far from it — that is what keeps its approach alive,
            // since promotion happens on the slot-one change. If nothing ahead is unflagged the nearest still takes it.
            int first = 0;
            while (first < upcoming.Count && _skippedApexes.Contains(ARS.Corners[upcoming[first]].Node)) first++;
            if (first >= upcoming.Count) first = 0;

            List<int> selectedNodes = new List<int>();
            List<float> selectedRadii = new List<float>();
            if (first < upcoming.Count)
            {
                selectedNodes.Add(ARS.Corners[upcoming[first]].Node);
                selectedRadii.Add(ARS.Corners[upcoming[first]].DetectedRadius);
            }

            // The remaining slots only feed the braking plan, so they go to the most restrictive corners this car
            // has decided it needs one for — the ones that make it brake earliest.
            float speed = Car.Velocity.Length();
            while (selectedNodes.Count < HeldApexCount)
            {
                int best = -1;
                float bestBrakingSpeed = float.MaxValue;
                for (int i = 0; i < upcoming.Count; i++)
                {
                    int node = ARS.Corners[upcoming[i]].Node;
                    if (selectedNodes.Contains(node)) continue;
                    if (!CornerRequiresBraking(node)) continue;

                    float radius = ARS.Corners[upcoming[i]].DetectedRadius;
                    float apexSpeed = ApexSpeedWithDownforce(radius);
                    if (apexSpeed >= speed) continue;
                    if (!CornerWorthHolding(node, radius, selectedNodes, selectedRadii)) continue;

                    float brakingSpeed = ApexBrakingSpeed(node, apexSpeed);
                    if (brakingSpeed < bestBrakingSpeed)
                    {
                        bestBrakingSpeed = brakingSpeed;
                        best = i;
                    }
                }
                if (best < 0) break;
                selectedNodes.Add(ARS.Corners[upcoming[best]].Node);
                selectedRadii.Add(ARS.Corners[upcoming[best]].DetectedRadius);
            }

            CommitApexQueue(
                new[]
                {
                    selectedNodes.Count > 0 ? selectedNodes[0] : -1,
                    selectedNodes.Count > 1 ? selectedNodes[1] : -1,
                    selectedNodes.Count > 2 ? selectedNodes[2] : -1
                },
                new[]
                {
                    selectedRadii.Count > 0 ? selectedRadii[0] : 999f,
                    selectedRadii.Count > 1 ? selectedRadii[1] : 999f,
                    selectedRadii.Count > 2 ? selectedRadii[2] : 999f
                });
        }

        // A candidate that belongs to a corner already held adds nothing unless it is materially slower: the pair is
        // one complex, and the tighter of the two is the one that has to be planned for.
        bool CornerWorthHolding(int node, float radius, List<int> selectedNodes, List<float> selectedRadii)
        {
            int distance = ForwardNodeDistance(node);
            float candidateSpeed = RouteIdealSpeedForRadius(radius);
            float nearestGap = float.MaxValue;
            float nearestSpeed = 999f;
            for (int i = 0; i < selectedNodes.Count; i++)
            {
                float gap = Math.Abs(distance - ForwardNodeDistance(selectedNodes[i]));
                if (gap < nearestGap)
                {
                    nearestGap = gap;
                    nearestSpeed = RouteIdealSpeedForRadius(selectedRadii[i]);
                }
            }
            if (nearestGap > Math.Max(5f, nearestSpeed * ApexBufferSeconds)) return true;
            return nearestSpeed - candidateSpeed >= SecondaryApexSpeedDifference;
        }

        void CommitApexQueue(int[] nodes, float[] radii)
        {
            NextApexNode = nodes[0];
            NextApexRadius = radii[0];
            NextApexSpeed = NextApexNode >= 0 ? ApexSpeedWithDownforce(NextApexRadius) : 999f;
            NextApexNode2 = nodes[1];
            NextApexRadius2 = radii[1];
            NextApexSpeed2 = NextApexNode2 >= 0 ? ApexSpeedWithDownforce(NextApexRadius2) : 999f;
            NextApexNode3 = nodes[2];
            NextApexRadius3 = radii[2];
            NextApexSpeed3 = NextApexNode3 >= 0 ? ApexSpeedWithDownforce(NextApexRadius3) : 999f;

            if (NextApexNode >= 0)
            {
                // Instance Brain.Corner from the nearest apex.
                CornerPoint original = ARS.Corners.FirstOrDefault(c => c.Node == NextApexNode);
                _nextApexCorner = original;
                CornerPoint cp = original ?? new CornerPoint();
                if (original == null)
                {
                    cp.Node = NextApexNode;
                    cp.Angle = ARS.TrackPoints[NextApexNode].Angle;
                    cp.DetectedRadius = NextApexRadius;
                }
                Brain.Corner = new Corner(NextApexSpeed, cp);
            }
            else
            {
                _nextApexCorner = null;
                Brain.Corner = null;
            }
        }

        bool HasPassedApex(int apexNode)
        {
            if (apexNode < 0) return false;
            if (ARS.IsPointToPoint) return CurrentTrackPoint.Node >= apexNode;
            // On circuits, the apex is behind us when the forward distance exceeds half the track.
            return ForwardNodeDistance(apexNode) > ARS.TrackPoints.Count / 2;
        }

        // Kinematic braking map reaches apex speed at the braking target: a fixed lead before the corner's turn-in.

        float ApexBrakingSpeed(int apexNode, float apexSpeed)
        {
            if (apexNode < 0) return 999f;
            float velTarget = apexSpeed;

            CornerPoint corner = ARS.Corners.FirstOrDefault(c => c.Node == apexNode);
            int fallbackNode = CornerEntranceNode(corner, apexNode);
            int targetNode = BrakeTargetBaseNode(corner, fallbackNode);
            if (targetNode < 0) targetNode = fallbackNode;

            int targetDistance = ForwardNodeDistance(targetNode);
            int apexDistance = ForwardNodeDistance(apexNode);
            // On circuits, a passed braking target wraps to the next lap. Once the apex is
            // still ahead but the target is farther away, the target has been passed.
            float rawDistance = !ARS.IsPointToPoint && targetDistance > apexDistance
                ? 0f
                : targetDistance;
            if (rawDistance < 0f) rawDistance = 0f;
            float distance = rawDistance > 0f
                ? Math.Max(0f, rawDistance
                    - Car.Velocity.Length() * EntranceBrakeBufferSeconds
                    - EntranceBrakeExtraDistance)
                : 0f;

            // Grade over the same span the solve integrates, or the mean and the distance measure different zones.
            float decel = BrakingDecel(apexNode, distance);

            float spd = (float)Math.Sqrt(velTarget * velTarget + 2f * decel * distance);
            if (float.IsNaN(spd) || float.IsInfinity(spd)) spd = 999f;
            return spd;
        }

        // Braking decel over a span: grip-limited base plus the gravity component of the span's mean grade.
        // The solve is v² = vApex² + 2∫a·ds, so the mean decel over the span is the exact quantity.
        // (CrestDecelFactor was removed: the scanner no longer produces a per-feature gs reading, so the
        // brake decel cannot integrate an unload term. The crest/dip table is for visualisation only.)
        public float BrakingDecel(int apexNode, float spanMeters)
        {
            float decel = BrakingDecelBase(apexNode) + Handling.Gravity * BrakingGradeSine(spanMeters);
            return Math.Max(decel, 0.1f);
        }

        // The grade-free half, so a horizon test can ask the same question without walking the span.
        float BrakingDecelBase(int apexNode)
        {
            float brakingAbility = Math.Min(Handling.BrakingAbility * 4, VehicleData.CurrentMechanicalGrip);
            float decel = brakingAbility * Handling.Gravity * EffectiveBrakeFactor(apexNode);
            if (ActiveManeuver.Type == ManeuverType.Yield) decel *= 0.5f;
            return decel;
        }

        // Mean grade over the braking span as sin(pitch): negative downhill loses decel, positive uphill gains it.
        float BrakingGradeSine(float spanMeters)
        {
            int spanNodes = (int)spanMeters;
            if (spanNodes < 1) return 0f;
            int count = ARS.TrackPoints.Count;
            int samples = Math.Min(spanNodes, 8);
            float sum = 0f;
            for (int i = 1; i <= samples; i++)
            {
                int node = CurrentTrackPoint.Node + (int)(spanNodes * (float)i / samples);
                int sample = ARS.IsPointToPoint ? (int)ARS.Clamp(node, 0, count - 1) : ((node % count) + count) % count;
                sum += ARS.TrackPoints[sample].Elevation;
            }
            return sum / samples * ElevationToSlopeSine;
        }

        // Centripetal speed limit for a radius.
        float RouteIdealSpeedForRadius(float r)
        {
            if (float.IsNaN(r) || float.IsInfinity(r)) r = 999f;
            r = ARS.Clamp(r, 5f, 999f);
            return (float)Math.Sqrt((VehicleData.CurrentMechanicalGrip * Handling.Gravity) * r);
        }

        float ApexSpeedWithDownforce(float r)
        {
            // CurrentMechanicalGrip already includes the per-tick downforce bonus (see UpdatePerceivedGrip),
            // so this is just the cornering-speed-from-grip formula with a sanity clamp on the radius.
            if (float.IsNaN(r) || float.IsInfinity(r)) r = 999f;
            r = ARS.Clamp(r, 5f, 999f);
            return (float)Math.Sqrt((VehicleData.CurrentMechanicalGrip * Handling.Gravity) * r);
        }

        float ComputeRouteRadius(int startOffset, int endOffset)
        {
            int count = ARS.TrackPoints.Count;
            int routeStartNode, routeEndNode, routeMidNode;
            if (ARS.IsPointToPoint)
            {
                routeStartNode = (int)ARS.Clamp(CurrentTrackPoint.Node + startOffset, 0, count - 1);
                routeEndNode = (int)ARS.Clamp(CurrentTrackPoint.Node + endOffset, 0, count - 1);
                routeMidNode = (int)((routeStartNode + routeEndNode) * 0.5f);
            }
            else
            {
                routeStartNode = ((CurrentTrackPoint.Node + startOffset) % count + count) % count;
                routeEndNode = ((CurrentTrackPoint.Node + endOffset) % count + count) % count;
                int span = ((endOffset - startOffset) % count + count) % count;
                if (span == 0) return 999f;
                routeMidNode = (((CurrentTrackPoint.Node + startOffset + span / 2) % count) + count) % count;
            }

            if (routeEndNode == routeStartNode) return 999f;
            // Circumradius through the start, midpoint, and end of the route window.
            return ARS.Circumradius3D(
                ARS.TrackPoints[routeStartNode].Position,
                ARS.TrackPoints[routeEndNode].Position,
                ARS.TrackPoints[routeMidNode].Position);
        }




        void ProcessTimedAI()
        {
            int now = Game.GameTime;

            if (_halfSecondTick + _phaseOffsetMs < now)
            {
                _halfSecondTick = now + 500 + (int)ARS.Remap(Car.Velocity.Length(), 0, 100, -250, 250, true);
            }

            // 04: the rival records used to refresh once a second, so the rival walls decided from a picture up to a second
            // old - sixty metres at racing speed. They now have their own timer, and the one-second work below keeps its
            // cadence; UpdateRivals is three records and a handful of native reads, cheap at four times a second.
            if (_rivalTick + _phaseOffsetMs < now)
            {
                _rivalTick = now + RivalRefreshMs;
                if (!ControlledByPlayer && BaseBehavior == RacerBaseBehavior.Race && ARS.Racers.Count >= 1) UpdateRivals();
            }

            if (_oneSecondTick + _phaseOffsetMs < now)
            {
                _oneSecondTick = now + 1000;

                if (!ControlledByPlayer)
                {
                    if (BaseBehavior == RacerBaseBehavior.Race && ARS.Racers.Count >= 1)
                    {
                        ConsiderCrowdCard();
                    }

                    // AutoRepair: past half damage on the body or the engine the car puts itself back together. Once a
                    // second is enough at this threshold, and a repair leaves the health full, so it cannot loop.
                    if (ARS.Autofix == AutofixMode.AutoRepair)
                    {
                        // Body and engine do not share a ceiling: MaxHealth is the body's, while an engine's health tops out at
                        // 1000 by GTA's own table and goes negative when it is on fire. Comparing the engine against MaxHealth
                        // made the threshold wrong on any car whose body health is not 1000.
                        float bodyFloor = Math.Max(Car.MaxHealth, 1) * 0.5f;
                        if (Car.EngineHealth < EngineHealthHalf || Car.BodyHealth < bodyFloor)
                        {
                            // Before the repair: afterwards both read full and the line proves nothing.
                            ARS.Log(ARS.LogImportance.Info, "AutoRepair " + Name + ": body " + Car.BodyHealth.ToString("0") + ", engine " + Car.EngineHealth.ToString("0"));
                            Car.Repair();
                        }
                    }

                    if (!Driver.IsSittingInVehicle(Car) && Car.IsStopped && Driver.IsStopped)
                    {
                        if (Driver.TaskSequenceProgress == -1)
                        {
                            TaskSequence enter = new TaskSequence();
                            Function.Call(Hash.TASK_ENTER_VEHICLE, 0, Car, 6000, -1, 2f, 0, 0);
                            enter.Close();
                            Driver.Task.PerformSequence(enter);
                            return;
                        }
                    }
                }

                // Independent rocket/boost control: enable boost on a stable, full-throttle run.
                if (BaseBehavior == RacerBaseBehavior.Race && Brain.CurrentPerception.HighSpeedCurveRadius > RocketBoostMinimumCurveRadius && Math.Abs(VehicleData.SlideAngle) < 0.5f && Math.Abs(Control.Throttle) > 0.9f) Function.Call((Hash)0x81E1552E35DC3839, Car, true);

                // Independent rocket/boost control: disable boost when braking.
                if (Function.Call<bool>((Hash)0x3D34E80EED4AE3BE, Car) && Control.Brake > 0.1f) Function.Call((Hash)0x81E1552E35DC3839, Car, false);


                // Rivals only: this sits outside the !ControlledByPlayer block that carries the proofs, so without the guard
                // the player's own car was repaired every tick in this mode - and the probe below would have measured it.
                if (!ControlledByPlayer && ARS.Autofix == AutofixMode.Invincible && Function.Call<bool>(Hash._IS_VEHICLE_DAMAGED, Car))
                {
                    // Info, so it only shows when someone raises Log Level to Info on purpose: does an invincible car ever take
                    // damage at all? If it never appears when the level is up, the proofs hold and this repair is dead weight.
                    ARS.Log(ARS.LogImportance.Info, "Invincible repair " + Name + ": body " + Car.BodyHealth.ToString("0") +
                        ", engine " + Car.EngineHealth.ToString("0"));
                    Car.Repair();
                }
            }
        }




        public void ProcessAI()
        {
            int now = Game.GameTime;

            ProcessTimedAI();
            if (_pressureTick + _phaseOffsetMs < now)
            {
                _pressureTick = now + 500;
                UpdatePressure();
            }

            if (BaseBehavior == RacerBaseBehavior.GridWait && Control.HandBrakeTime < now) Control.HandBrakeTime = now + (100 * ARS.GetRandomInt(2, 6));

            if (!ControlledByPlayer)
            {
                UpdateDNFCheck();

                if (_rivalInfoTick + _phaseOffsetMs < now)
                {
                    _rivalInfoTick = now + 500;
                    UpdateRivalInfo();
                }
                ConsiderManeuvers();
                ComputeTargetSpeed();
                ComputeSteering();

                UpdateThrottleReasonCaps();
                UpdateBrakeReasonCaps();
                UpdateThrottleKick();
                ConvertSpeedToPedals();

                UpdateStuckCheck();
                UpdateStuckRecovery();

                ApplyStuckRecoveryOverride();

                // The limiter closes the steering last, after every writer above, so nothing escapes it.
                ApplySteerLimits();
                TranslateSteerToInput();

                UpdateNitrous();
                UpdateYield();

            }
            else
            {
                UpdateNitrous();
                _stuckPhase = StuckPhase.None;
                _stuckStationarySince = 0;
                _stuckOffTrackSince = 0;
                _stuckRecoveryCooldownEndTime = 0;
            }
        }

        void UpdatePressure()
        {
            float closestDistance = float.MaxValue;
            if (BaseBehavior == RacerBaseBehavior.Race)
            {
                foreach (Racer racer in ARS.Racers)
                {
                    if (racer == this || racer.Car == null || !racer.Car.Exists()) continue;
                    float dist = racer.Car.Position.DistanceTo(Car.Position);
                    if (dist < closestDistance)
                        closestDistance = dist;
                }
            }

            float targetPressure = 0f;
            if (closestDistance <= PressureProximityRange)
            {
                // Map nearby-rival proximity to an aggression-scaled pressure target.
                float t = ARS.Clamp((PressureProximityRange - closestDistance) / (PressureProximityRange - 20f), 0f, 1f);
                targetPressure = Aggression * t;
            }

            if (targetPressure > Pressure)
                Pressure = Math.Min(Pressure + PressureRisePerSecond * TickScale, targetPressure);
            else
                Pressure = Math.Max(Pressure - PressureFallPerSecond * TickScale, targetPressure);

            Pressure = ARS.Clamp(Pressure, 0f, PressureRange);
        }
 
 
        // A car whose engine is gone cannot rejoin whatever the recovery does, so it is taken out of the race and
        // parked instead of being teleported at forever.
        void UpdateDNFCheck()
        {
            if (IsDNF)
            {
                Control.HandBrakeTime = Game.GameTime + 1000;
                return;
            }
            if (BaseBehavior != RacerBaseBehavior.Race || Car.EngineHealth > 0f) return;
            ParkAtDNFSlot();
        }

        void ParkAtDNFSlot()
        {
            int slot = ARS.ParkedDNFs++;
            int nodeCount = ARS.TrackPoints.Count;
            int node = ARS.IsPointToPoint ? Math.Min(slot * DNFSlotSpacingMeters, nodeCount - 1) : (slot * DNFSlotSpacingMeters) % nodeCount;
            TrackPoint point = ARS.TrackPoints[node];
            Vector3 direction = new Vector3(point.Direction.X, point.Direction.Y, 0f);
            if (direction == Vector3.Zero) direction = Vector3.WorldNorth;
            direction.Normalize();
            Vector3 right = Vector3.Cross(direction, Vector3.WorldUp);
            float side = slot % 2 == 0 ? 1f : -1f;
            Car.Position = point.Position + right * side * (point.TrackHalfWidth + DNFSlotOffsetMeters) + new Vector3(0f, 0f, 0.5f);
            Car.Heading = direction.ToHeading();
            Car.Velocity = Vector3.Zero;
            if (ARS.DNFUnderTrack)
            {
                Car.Position = new Vector3(Car.Position.X, Car.Position.Y, Car.Position.Z - DNFUnderTrackMeters);
                Car.FreezePosition = true;
            }
            IsDNF = true;
            BaseBehavior = RacerBaseBehavior.FinishedStandStill;
            FinishStuckRecovery();
        }

        void UpdateStuckCheck()
        {
            if (_stuckPhase != StuckPhase.None)
            {
                ResetStuckArmTimers();
                return;
            }

            int now = Game.GameTime;

            if (now < _stuckRecoveryCooldownEndTime || BaseBehavior != RacerBaseBehavior.Race || !Driver.IsSittingInVehicle(Car))
            {
                ResetStuckArmTimers();
                return;
            }

            // Not in the opening seconds of a race: a car still crawling off the line is launching, not stuck, and the
            // pack around it is not a place to reverse into.
            if (now < _stuckArmAllowedTime)
            {
                ResetStuckArmTimers();
                return;
            }

            if (PlanWantsToMove())
            {
                if (_stuckStationarySince == 0) _stuckStationarySince = now;
            }
            else _stuckStationarySince = 0;

            if (OutOfTrackDistance() > 0f)
            {
                if (_stuckOffTrackSince == 0) _stuckOffTrackSince = now;
            }
            else _stuckOffTrackSince = 0;

            bool stationary = _stuckStationarySince != 0 && now - _stuckStationarySince >= RecoveryTriggerMs;
            bool offTrack = _stuckOffTrackSince != 0 && now - _stuckOffTrackSince >= RecoveryTriggerMs;

            if (stationary) StartStuckRecovery(now, StuckPhase.Reverse);
            else if (offTrack) StartStuckRecovery(now, StuckPhase.Drive);
        }

        // A car wants to be somewhere it is not: too slow, with its own plan asking for more. That is the stuck case,
        // as opposed to being held still on purpose.
        bool PlanWantsToMove()
        {
            return ARS.MpsToMph(Car.Velocity.Length()) < RecoveryTriggerMph && Brain.CurrentIntention.Speed - Car.Velocity.Length() > ARS.MphToMps(RecoveryPlanGapMph);
        }

        void ResetStuckArmTimers()
        {
            _stuckStationarySince = 0;
            _stuckOffTrackSince = 0;
        }

        void StartStuckRecovery(int now, StuckPhase phase)
        {
            _stuckPhase = phase;
            if (phase == StuckPhase.Reverse) _stuckPhaseEndTime = now + StuckReverseMs;
            _stuckRecoveryStartTime = now;
            _stuckAgainSince = 0;
            _stuckExitSince = 0;
            _stuckMoveSamplePosition = Car.Position;
            _stuckMoveSampleTime = now;
            ResetStuckArmTimers();
            // Forced, once per entry: the kick card's decision inputs at the moment they matter, so a kick that
            // did not play explains itself in the log instead of staying silent while the situation was real.
            if (phase == StuckPhase.Drive)
            {
                int entryGear = Function.Call<int>((Hash)CurrentDriveGearHash, Car);
                int entryGearCount = Function.Call<int>((Hash)MaxDriveGearHash, Car);
                float entryThrust = VehicleMemory.GetInitialDriveForce(Car) * KickGearRatio(Math.Max(entryGear, 1), entryGearCount) / 2f;
                float entryBudget = VehicleData.CurrentMechanicalGrip * KickWheelLoadShare * KickGripMargin;
                ARS.Log(ARS.LogImportance.Info, "Kick " + Name + ": recovery drive — bearing " + _steerAimBearingDegrees.ToString("0") + " deg, lock " + VehicleData.SteeringLock.ToString("0") + " deg, bias " + VehicleMemory.GetDriveBiasFront(Car).ToString("0.00") + ", close " + KickCloseSeconds().ToString("0.0") + " s, forward " + ARS.MpsToMph(ARS.GetForwardSpeed(Car)).ToString("0") + " mph, gear " + entryGear + "/" + entryGearCount + ", thrust " + entryThrust.ToString("0.00") + ", budget " + entryBudget.ToString("0.00"), true);
            }
        }

        void UpdateStuckRecovery()
        {
            if (BaseBehavior != RacerBaseBehavior.Race || !Driver.IsSittingInVehicle(Car))
            {
                if (_stuckPhase != StuckPhase.None) FinishStuckRecovery();
                return;
            }

            if (_stuckPhase == StuckPhase.None) return;

            int now = Game.GameTime;

            if (now - _stuckMoveSampleTime >= StuckMoveSampleMs)
            {
                bool couldNotMove = Car.Position.DistanceTo(_stuckMoveSamplePosition) < StuckMoveMeters;
                bool escapeSpent = now - _stuckRecoveryStartTime >= StuckRecoveryTimeMs;
                _stuckMoveSamplePosition = Car.Position;
                _stuckMoveSampleTime = now;
                if (couldNotMove && escapeSpent)
                {
                    SnapToTrack();
                    return;
                }
            }

            if (_stuckPhase == StuckPhase.Reverse)
            {
                if (now - _stuckRecoveryStartTime >= StuckRecoveryTimeMs) SnapToTrack();
                else if (now >= _stuckPhaseEndTime) _stuckPhase = StuckPhase.Drive;
                return;
            }

            if (PlanWantsToMove())
            {
                if (_stuckAgainSince == 0) _stuckAgainSince = now;
            }
            else _stuckAgainSince = 0;

            if (_stuckAgainSince != 0 && now - _stuckAgainSince >= RecoveryTriggerMs)
            {
                _stuckPhase = StuckPhase.Reverse;
                _stuckPhaseEndTime = now + StuckReverseMs;
                _stuckMoveSamplePosition = Car.Position;
                _stuckMoveSampleTime = now;
                return;
            }

            if (RecoveryExitHolds()) FinishStuckRecovery();
        }

        // Off the track the nearest edge is the way back; the normal aim is the centre, which crosses the whole width.
        float NearestEdgeLane(float steerRefOffset, float drivableEdge)
        {
            return steerRefOffset >= 0f ? drivableEdge : -drivableEdge;
        }

        // The steering's own reach: how many seconds the bearing to the aim takes to close on the yaw that full
        // lock's minimum radius buys at the current speed — the recovery-entry log's informative read of how far
        // the situation was from placeable by steering alone.
        float KickCloseSeconds()
        {
            if (float.IsNaN(VehicleData.SteeringLock) || float.IsInfinity(VehicleData.SteeringLock)) return 0f;
            float lockRad = ARS.DegToRad(VehicleData.SteeringLock);
            float yawDegPerSecAtLock = ARS.RadToDeg(ARS.GetForwardSpeed(Car) * (float)Math.Tan(lockRad) / Math.Max(VehicleData.WheelBase, 0.1f));
            return Math.Abs(_steerAimBearingDegrees) / Math.Max(yawDegPerSecAtLock, 1f);
        }

        // The transmission's gear ratio, ported from the leaked CTransmission::CalculateGearRatios: first is
        // 1/0.3, top is 0.9, and the middles walk the 1.1 progression from a baseRatio seeded by the first's
        // ratio — so the unread flag multipliers (the close-ratio gearbox and low-speed-torque flags halve the
        // first) shift the whole ladder on flagged cars, not just first: those cars read a green light on
        // multiplied thrust, which the fold's slip line exposes as a shallow burst.
        static float KickGearRatio(int gear, int maxGear)
        {
            if (maxGear < 2 || gear >= maxGear) return 0.9f;
            if (gear == 1) return 1.0f / 0.3f;
            float first = 1.0f / 0.3f;
            double baseRatio = Math.Pow((1.0 / Math.Pow(1.1, 0.5 * (maxGear - 1) * (maxGear - 2))) * first, 1.0 / (maxGear - 1));
            return (float)(0.9 * Math.Pow(baseRatio, maxGear - gear) * Math.Pow(1.1, 0.5 * (maxGear - gear) * (maxGear - gear - 1.0)));
        }

        // Gear eligibility, from the leak's own thrust math: at full throttle the per-wheel force is throttle
        // (1) x drive-force percent x the gear's ratio, split over the two drive wheels; the tyre budget is the
        // live grip x the wheel's static load share. Mass and gravity cancel across the two sides, so the
        // comparison is a single line. The clutch and rev factors only shrink the real burst, so this is the
        // ceiling — the right side of the gate to be.
        bool GearCanKick()
        {
            int currentGear = Function.Call<int>((Hash)CurrentDriveGearHash, Car);
            int maxGear = Function.Call<int>((Hash)MaxDriveGearHash, Car);
            if (currentGear < 1 || maxGear < 2) return false;
            float thrustPerWheel = VehicleMemory.GetInitialDriveForce(Car) * KickGearRatio(currentGear, maxGear) / 2f;
            return thrustPerWheel > VehicleData.CurrentMechanicalGrip * KickWheelLoadShare * KickGripMargin;
        }

        // Off by decision: the card owns the pedal for its window and pins the TCS, and from the outside the car just
        // reads as unpredictable. Reviving it is this constant.
        const bool KickCardEnabled = false;

        void UpdateThrottleKick()
        {
            if (!KickCardEnabled) return;

            if (!_kickPlaying)
            {
                TryArmThrottleKick();
                return;
            }

            int now = Game.GameTime;
            bool rotated = Math.Abs(VehicleData.SlideAngle) > KickFoldSlideDegrees;
            if (now < _kickWindowEnd && !rotated) return;

            // Forced: the kick's lifecycle is the card's own read-out and the drive's instrument, written
            // whatever the Log Level; the event is rare, so the noise is bounded.
            string foldReason = rotated ? "rotated" : "window spent";
            ARS.Log(ARS.LogImportance.Info,
                "Kick " + Name + ": folded " + foldReason + " after " + (now - _kickPlayingSince) + " ms, ask " + _kickAskDegrees.ToString("0") + " deg, slip " + ARS.MaxWheelSlip(Car).ToString("0.00"), true);
            _kickPlaying = false;
            _kickCooldownUntil = now + KickCooldownMs;
        }

        void TryArmThrottleKick()
        {
            if (ControlledByPlayer || ARS.GetForwardSpeed(Car) <= 0f
                || VehicleMemory.GetDriveBiasFront(Car) > KickMaxDriveBiasFront
                || Game.GameTime < _kickCooldownUntil || !GearCanKick()
                || Math.Abs(VehicleData.SlideAngle) >= TRLateralAtSpeed * SlideBlendStartFraction)
            {
                _kickArmSince = 0;
                return;
            }
            // The arm's one condition, read where it lands: the ask handed to the limiter one core tick ago sat
            // past the ceiling it enforced — the steering cannot deliver what the pursuit wants, and with no
            // slide there is no rotation being had. One tick stale by construction, which the hold's own second
            // absorbs.
            bool overTheLimiter = Math.Abs(_steerPreLimitDegrees) > _steerGrantedCeiling;
            if (!overTheLimiter)
            {
                _kickArmSince = 0;
                return;
            }

            int now = Game.GameTime;
            if (_kickArmSince == 0)
            {
                _kickArmSince = now;
                return;
            }
            if (now - _kickArmSince < KickArmSaturateMs) return;

            _kickPlaying = true;
            _kickPlayingSince = now;
            _kickWindowEnd = now + KickWindowMs;
            _kickArmSince = 0;
            _kickSteerSign = Math.Sign(_steerPreLimitDegrees);
            ARS.Log(ARS.LogImportance.Info, "Kick " + Name + ": playing, ask " + _steerPreLimitDegrees.ToString("0") + " deg past granted " + _steerGrantedCeiling.ToString("0") + " deg at " + ARS.MpsToMph(Car.Velocity.Length()).ToString("0") + " mph, slip " + ARS.MaxWheelSlip(Car).ToString("0.00"), true);
        }

        // Any part of the car on the drivable bound and moving forward, held: a reverse roll is not a recovered car.
        bool RecoveryExitHolds()
        {
            bool onTrack = Math.Abs(Brain.CurrentPerception.CurrentNodeOffset) <= CurrentTrackPoint.TrackHalfWidth + VehicleData.BoundingBox * 0.5f;
            bool movingForward = ARS.MpsToMph(Vector3.Dot(Car.Velocity, Car.ForwardVector)) > RecoveryExitMph;
            if (!onTrack || !movingForward)
            {
                _stuckExitSince = 0;
                return false;
            }
            if (_stuckExitSince == 0) _stuckExitSince = Game.GameTime;
            return Game.GameTime - _stuckExitSince >= RecoveryExitHoldMs;
        }

        void FinishStuckRecovery()
        {
            _stuckPhase = StuckPhase.None;
            _stuckPhaseEndTime = 0;
            _stuckRecoveryStartTime = 0;
            _stuckAgainSince = 0;
            _avoidHoldActive = false;
            _avoidHoldLane = 0f;
            _stuckExitSince = 0;
            _stuckRecoveryCooldownEndTime = Game.GameTime + StuckRecoveryCooldownMs;
        }

        void ApplyStuckRecoveryOverride()
        {
            if (_stuckPhase != StuckPhase.Reverse) return;

            Control.Brake = 0f;
            Control.BrakeReason = BrakeReason.StuckRecovery;
            Control.BrakeReasonLevel = 0f;
            Control.ThrottleReason = ThrottleReason.StuckRecovery;
            Control.ThrottleReasonLevel = 0f;
            Control.Throttle = -0.5f;
            Control.SteerDegrees = 0f;
        }

        void SnapToTrack()
        {
            Vector3 direction = new Vector3(CurrentTrackPoint.Direction.X, CurrentTrackPoint.Direction.Y, 0f);
            if (direction == Vector3.Zero) direction = Vector3.WorldNorth;
            direction.Normalize();
            Vector3 right = Vector3.Cross(direction, Vector3.WorldUp);
            float side = ARS.SignedLaneOffset(Car.Position, CurrentTrackPoint.Position, CurrentTrackPoint.Direction) >= 0f ? 1f : -1f;
            float laneOffset = Math.Max(CurrentTrackPoint.TrackHalfWidth - VehicleData.BoundingBox * 0.5f, 0f) * side;
            Car.Position = CurrentTrackPoint.Position + right * laneOffset + new Vector3(0f, 0f, 0.5f);
            Car.Heading = direction.ToHeading();
            Car.Velocity = direction * ARS.MphToMps(10f);
            _laneSnap = true;
            FinishStuckRecovery();
        }

        void UpdatePerceivedGrip()
        {


            float handlingGrip = Function.Call<float>((Hash)0xA132FB5370554DB0, Car);
            handlingGrip = ARS.Clamp(handlingGrip, 0.1f, 100f);
            handlingGrip /= 1f + 0.035f * Handling.Downforce;

            GroundGripMultiplier = ARS.MeanWheelGripMultiplier(Car);

            // Centripetal acceleration v²/r stands in for the cornering load the engine scales downforce by; see
            // GetDownforceGsAtSpeed for why world-frame lateral velocity cannot be used here.
            float forwardMs = ARS.GetForwardSpeed(Car);
            float routeRadius = Brain.CurrentPerception.CurveRadiusToFollowPoint;
            float lateralMs = 0f;
            if (routeRadius > 1f && !float.IsNaN(routeRadius) && !float.IsInfinity(routeRadius))
                lateralMs = (forwardMs * forwardMs) / routeRadius;
            float dfGs = ARS.GetDownforceGsAtSpeed(this, forwardMs, lateralMs);

            VehicleData.BaseMechanicalGrip = handlingGrip;
            VehicleData.DownforceGripBonus = dfGs;
            VehicleData.CurrentMechanicalGrip = (VehicleData.BaseMechanicalGrip + VehicleData.DownforceGripBonus) * GroundGripMultiplier;

            if (!_gripLogged)
            {
                _gripLogged = true;
                ARS.Log(ARS.LogImportance.Info, "Grip for " + Car.DisplayName + ": base " + VehicleData.BaseMechanicalGrip
                    + ", current " + VehicleData.CurrentMechanicalGrip + " (steer cap k " + SteerReductionPerMps + ")");
            }

            // Sampling the wheel-pushed Gs at ~3 Hz so the overspeed reason can glide on a cheap read.
            if (Game.GameTime - _lastGsCheck >= 333) // ~3 Hz
            {
                _lastGsCheck = Game.GameTime;

                // Compare measured forward Gs against wheel-pushed Gs: GTA lets an uphill car accelerate beyond
                // what its wheel power should produce, and this is the correction for that.
                if (ARS.OverspeedCutPerTenthG > 0f)
                {
                    List<float> wheelPowers = ARS.WheelPowers(Car);
                    float wheelGs = 0f;
                    foreach (float p in wheelPowers) wheelGs += p;
                    float measuredGs = VehicleData.GetLongitudinalGs(Car.ForwardVector);
                    float excess = measuredGs - wheelGs - 0.1f;
                    VehicleData.OverspeedExcessGs = excess;

                    // Inside a burn only, and twice a second: the burn is the one moment the excess is expected to be
                    // large, so this is the pair of numbers that says whether the wheel reading follows the cheat
                    // multiplier or not. If wheel does not rise with measured, the reference is on a different scale.
                    if (Game.GameTime < _nitrousActiveUntil && Game.GameTime > _nitrousGsLoggedAt)
                    {
                        _nitrousGsLoggedAt = Game.GameTime + 500;
                        ARS.Log(ARS.LogImportance.Info, "Nitro Gs: measured " + measuredGs.ToString("0.00")
                            + ", wheel " + wheelGs.ToString("0.00") + ", excess " + excess.ToString("0.00")
                            + (ControlledByPlayer ? " (player)" : " (AI)"));
                    }
                }
            }



            VehicleData.YawRotationPerSecondDegrees = ARS.RadToDeg(Function.Call<Vector3>(Hash.GET_ENTITY_ROTATION_VELOCITY, Car).Z);
        }
        public void UpdateRivals()
        {
            if (ARS.NoCollision)
            {
                foreach (Rival r in Brain.Rivals) r.RivalRacer = null;
                return;
            }

            Vector3 myPosition = Car.Position;
            Vector3 hoodPosition = myPosition + Car.ForwardVector;
            List<Racer> candidates = new List<Racer>();
            List<float> candidateDistances = new List<float>();
            foreach (Racer r in ARS.Racers)
            {
                if (r.Car.Handle == Car.Handle) continue;
                if (ARS.DNFUnderTrack && r.IsDNF) continue;
                Vector3 rivalPosition = r.Car.Position;
                if (Vector3.Distance(rivalPosition, myPosition) >= RivalSearchRangeMeters) continue;
                candidates.Add(r);
                candidateDistances.Add((rivalPosition - hoodPosition).LengthSquared());
            }

            foreach (Rival r in Brain.Rivals) r.RivalRacer = null;
            for (int slot = 0; slot < Brain.Rivals.Count && candidates.Count > 0; slot++)
            {
                int nearest = 0;
                for (int i = 1; i < candidates.Count; i++)
                {
                    if (candidateDistances[i] < candidateDistances[nearest]) nearest = i;
                }
                Brain.Rivals[slot].RivalRacer = candidates[nearest];
                candidates.RemoveAt(nearest);
                candidateDistances.RemoveAt(nearest);
            }

            // A re-owned slot must not keep the last occupant's numbers: every consumer reads the record, not the car.
            UpdateRivalInfo();
        }

        public void Delete()
        {
            // Ownership decides, not where the player is standing. This used to test IsInVehicle, so a player who stepped
            // out before the race ended had their own car deleted at teardown - and the arm that ran when they were inside
            // teleported them into the driver's seat for no reason. A player-owned racer keeps its vehicle; an AI car is
            // deleted unless the player happens to be sitting in it, which would take them with it.
            // A burn still live when the race ends would leave SET_OVERRIDE_NITROUS_LEVEL armed on a car the player keeps, and
            // nothing in Initialize clears it: the bottle would follow them out of the race for the rest of the session.
            // Presence, not liveness: a burn that expired since the last core tick is still armed in the vehicle, and StopNitrous is
            // idempotent, so the wider test is the correct one.
            if (_nitrousActiveUntil > 0) StopNitrous();

            if (!ControlledByPlayer) Driver.Delete();

            if (Car.CurrentBlip != null && Car.CurrentBlip.Exists()) Car.CurrentBlip.Remove();

            if (ControlledByPlayer || Game.Player.Character.IsInVehicle(Car))
            {
                Car.IsPersistent = false;
            }
            else
            {
                Car.Delete();
            }
        }
    }
}


