// One racer's laws: brake plan, corner authority, route speed points, lane and learner.
using GTA;
using GTA.Math;
using GTA.Native;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.Linq;
using System.Windows.Forms;

namespace ARS
{

        // Every track-derived speed law has a bench switch: off leaves the code, removes the effect.
    internal static class TrackSpeed
    {
        public static readonly bool CornerPlan = true;
        public static readonly bool RouteCurvature = true;
        public static readonly bool ChicaneBoost = true;
        public static readonly bool ProjectionCut = true;
        public static readonly bool OffTrackThrottle = true;
        public static readonly bool CrestGate = true;
        public static readonly bool StabilityCap = true;
        public static readonly bool GsLaneFloor = true;
        public static readonly bool OverspeedCorrection = true;
        public static readonly bool SlopeGrip = true;
    }

    public enum RacerBaseBehavior
    {
        GridWait, Race, FinishedRace, FinishedStandStill
    }
    public class Racer
    {

        public string Name = "Racer";
        internal string _baseName = "Racer";
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

        public int NextApexNode = -1;
        public float NextApexRadius = 999f;
        public float NextApexSpeed = 999f;
        public int NextApexNode2 = -1;
        public float NextApexRadius2 = 999f;
        public float NextApexSpeed2 = 999f;
        public int NextApexNode3 = -1;
        public float NextApexRadius3 = 999f;
        public float NextApexSpeed3 = 999f;
        CornerPoint _nextApexCorner;
        const int HeldApexCount = 3;

        public VehicleState VehicleData = new VehicleState();
        public HandlingData Handling = new HandlingData();
        public float GroundGripMultiplier = 1f;
        Vector3 _lastSpeed;
        Vector3 _lastAccel;
        int _lastGsCheck = 0;
        bool _gripLogged = false;
        float _restHeightAboveGround = -1f;

        public TrackPoint CurrentTrackPoint = new TrackPoint();
        public float AlongTrackSpeed = 0f;

        public int RaceOrderKey = 0;

        int _cornerModeDistanceNodes = 0;
        int _cornerModeApexTargetNodes = 0;
        int _cornerModeLastNode = -1;
        int _lapOdometer = 0;
        int _lastLapNode = -1;
        int _lapStartNode = -1;
        int _lapStartOdometer = 0;
        public int LiveRouteCrestNode = -1;
        public int LiveRouteCrestFarNode = -1;
        public int LiveRouteRadiusNearNode = -1;
        public int LiveRouteRadiusMidNode = -1;
        public int LiveRouteRadiusFarNode = -1;
        public int LiveSlidRadiusNearNode = -1;
        public int LiveSlidRadiusMidNode = -1;
        public int LiveSlidRadiusFarNode = -1;
        public float LiveSlidRadius = -1f;
        public float LiveRouteRadius = -1f;
        public enum LookAhead { SteerRef, OneSec };
        public Dictionary<LookAhead, TrackPoint> LookAheads = new Dictionary<LookAhead, TrackPoint>();

        public List<TimeSpan> LapTimes = new List<TimeSpan>();
        public int LapStartTime = 0;
        public int Lap = 0;
        public int RacePosition = 0;
        public bool IsDNF = false;
        public int FinalPosition = 0;
        public bool CanRegisterNewLap = false;
        int _previousNode = -1;
        public bool FinishedPointToPoint = false;
        int _halfSecondDueMs = 0;
        int _oneSecondDueMs = 0;
        int _phaseOffsetMs = 0;
        const int RivalRefreshMs = 250;
        int _rivalTick = 0;
        int _pressureTick = 0;
        int _apexUpdateTick = 0;
        int _rivalInfoTick = 0;
        int _lastCoreTick = 100;
        int TimeSinceLastCoreTick => (int)ARS.Clamp(Game.GameTime - _lastCoreTick, 1, 9999);

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
        const int DNFSlotSpacingMeters = 5;
        const float DNFSlotOffsetMeters = 1f;
        const float DNFUnderTrackMeters = 5f;

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
        float _kickAskDegrees = 0f;

        const float GsPreviewBlend = 0.5f;
        const float SteerPreviewSeconds = 0.5f;
        const float SteerLookaheadMinNodes = 8f;
        public const float SteerLookaheadMaxNodes = 90f;

        public static float SteerLookaheadNodes(float speedMps)
        {
            return ARS.Clamp(speedMps * SteerPreviewSeconds, SteerLookaheadMinNodes, SteerLookaheadMaxNodes);
        }

        float _avoidLeftWall = 0f;
        float _avoidRightWall = 0f;
        float _wallRoadWide = 0f;
        bool _leftWallHeld;
        bool _rightWallHeld;
        int _activeRivalWallCount = 0;
        bool _avoidWallsInitialized = false;

        public float AvoidLeftWall => _avoidLeftWall;
        public float AvoidRightWall => _avoidRightWall;
        public float WallRoadWide => _wallRoadWide;
        public bool LeftWallHeld => _leftWallHeld;
        public bool RightWallHeld => _rightWallHeld;

        float _laneRequest = 0f;
        LaneReason _laneRequestReason = LaneReason.OwnLane;
        float _carriedLane = 0f;

        float _pursuitRawBearingDeg;
        float _pursuitAimDistanceM;
        float _pursuitAimLateralM;

        int _probeApexNode = -1;
        int _laneApexNode = -1;
        int _laneShapeApexNode = -1;

        int _laneTraceAt;
        int _throttleLogAt;

        float _thrWanted;
        float _brkWanted;
        float _brkFinal;
        readonly SmoothFloat _brkSmooth = new SmoothFloat(0f, BrakeSmoothRate);
        readonly SmoothFloat _tcsSmooth = new SmoothFloat(1f, TcsSmoothRate);
        readonly SmoothFloat _stabSmooth = new SmoothFloat(1f, StabSmoothRate);
        float _thrFinal;
        readonly SmoothFloat _thrSmooth = new SmoothFloat(0f, ThrottleSmoothRate);
        readonly SmoothFloat _routeSmooth = new SmoothFloat(0f, RouteSmoothRate);
        float _thrDelta;
        float _thrTarget;
        float _thrActualMps;
        float _thrFloor;
        bool _thrSlew;
        bool _brakePlanActive;
        SpeedTerm _thrTerm;
        float _speedCorner;
        float _speedCornerArrival;
        float _speedRoute;
        float _routeSpeedOffsetMph;
        float _routeOffsetSeedMph;
        float _cornerOffsetSeedMph;

        public float ThrottleWanted { get { return _thrWanted; } }
        public float ThrottleFinal { get { return _thrFinal; } }
        public float BrakeFinal { get { return _brkFinal; } }
        public float BrakeSmooth { get { return _brkSmooth.Value; } }
        public float TcsSmooth { get { return _tcsSmooth.Value; } }
        public float StabSmooth { get { return _stabSmooth.Value; } }
        public float ThrottleSmooth { get { return _thrSmooth.Value; } }
        public float ThrottleDelta { get { return _thrDelta; } }
        public float ThrottleTargetMps { get { return _thrTarget; } }
        public float ThrottleActualMps { get { return _thrActualMps; } }
        public float ThrottleFloorLevel { get { return _thrFloor; } }
        public bool ThrottleFloorActive { get { return _gsFloorActiveThisTick; } }
        public bool ThrottleSlewLimited { get { return _thrSlew; } }
        public SpeedTerm ThrottleTerm { get { return _thrTerm; } }
        public int SpeedSourceNode { get { return Brain.CurrentIntention.TermNode; } }
        public float SpeedSourceMetres { get { return Brain.CurrentIntention.TermMetres; } }
        public float SpeedSourceRadius { get { return Brain.CurrentIntention.TermRadius; } }
        public float CornerTermMps { get { return _speedCorner; } }
        public float CornerArrivalMps { get { return _speedCornerArrival; } }
        public float RouteTermMps { get { return _speedRoute; } }
        float _probeMaxInwardOffsetMetres;
        int _probeNode = -1;
        float _probeOffsetAtApexMetres;
        bool _probeAtApex;

        void UpdateApexProbe(CornerPoint corner, float ownOffset)
        {
            int turn = TrackTurnSign(corner.StartNode, corner.Node, Math.Sign(corner.Angle));
            if (turn == 0) return;
            float inward = ownOffset * ARS.LaneInsideSign * turn;
            if (inward > _probeMaxInwardOffsetMetres)
            {
                _probeMaxInwardOffsetMetres = inward;
                _probeNode = CurrentTrackPoint.Node;
            }
            if (!_probeAtApex && Math.Abs(CurrentTrackPoint.Node - corner.Node) <= 1)
            {
                _probeOffsetAtApexMetres = ownOffset;
                _probeAtApex = true;
            }
        }

        void LogApexProbe(int apexNode)
        {
            ARS.Log(ARS.LogImportance.Info, "apex probe " + apexNode
                + ": offset at the apex " + (_probeAtApex ? _probeOffsetAtApexMetres.ToString("0.00") : "missed")
                + ", deepest inward " + _probeMaxInwardOffsetMetres.ToString("0.00")
                + " at node " + _probeNode
                + (_probeNode >= 0 ? " (" + (_probeNode - apexNode) + " nodes past the apex)" : ""));
        }
        float _laneField;
        int _laneTurnSign = 0;
        bool _laneOverride = false;
        public float CarriedLane { get { return _carriedLane; } }

        public int LineCornerNode { get { return _laneApexNode; } }
        public LaneReason LaneRequestReason { get { return _laneRequestReason; } }
        public float LaneRequest { get { return _laneRequest; } }

        int _divebombApexNode = -1;
        int _defendApexNode = -1;
        const float GsLaneThrottleMaxFloor = 1f;
        const float GsLaneOutsideFloor = 0f;

        const float GsLaneFloorSpreadMeters = 2f;
        const float ProjectionFullLiftMetres = 4f;
        const float ProjectionLiftPerSecond = 2f;
        const float ProjectionArmSlideShare = 0.6f;
        const float ProjectionMaxLift = 1f + 0.5f;
        const float OffTrackSpeedFloorMph = 0f;
        const float ProjectionMinSpeedMph = 20f;
        public const float FullThrottleSpeedErrorMps = 1f;
        public const float FullBrakeSpeedErrorMps = 4.5f;

        const float RouteSpeedBrakeCap = 0.33f;

        readonly List<InputTrailSample> _inputTrail = new List<InputTrailSample>();
        const float InputTrailSampleSpacing = 0.5f;
        const int InputTrailMaxSamples = 80;
        const float InputTrailChevronSpacing = 1f;
        const float InputTrailChevronGroundClearance = 1f;
        const float InputTrailChevronHeightGain = 0.5f;
        const float InputTrailChevronSize = 2f;
        const float InputTrailChevronPitch = 45f;
        const int InputTrailFadeCount = 5;
        float _inputTrailChevronDistance = 0f;

        float _damperTermDeg = 0f;

        float _countersteerDemand01 = 0f;
        float _slideShare = 0f;
        float _steerPursuitDeg = 0f;
        float _steerAimBearingDegrees = 0f;
        float _steerAimCurvature = 0f;

        float _steerPreLimitDegrees = 0f;

        float _chasingClampDeg = float.NaN;

        static readonly bool SlipPedalCut = false;

        const float SlipOpenDegPerSec = 72f;
        const float SlipClosePerSecond = 15f;
        const float SlipCloseMaxDegPerSec = 90f;

        SlipState _slip;
        const float FrontSlipNeutralSteerDeg = 0.5f;
        const float FrontSlipTheoryMinSpeedMps = 3f;
        const float FrontSlipTheoryMinCommandDeg = 0.05f;
        const float FrontSlipGravity = 9.81f;
        float _steerGrantedCeiling = 0f;

        float _projectionOutsideMeters = 0f;
        float _projectionCenterOffset = 0f;
        float _projectionHalfWidth = 0.1f;
        int _projectionMeasuredAt = -1;
        float _carOutsideMeters = 0f;
        bool _projectionWidening = false;
        float _projectionWidenMeters;
        int _projectionWidenSign;
        bool _projectionCutActive = false;
        bool _projectionArmed = false;
        float _gsLaneFloorShare01;

        float _gsFloorApplied;
        bool _exitRoomArmed;
        float _projectionLift = 0f;
        float _projectionDropMph = 0f;

        Color _projectionPathColor = Color.Blue;

        bool _gsFloorActiveThisTick = false;

        int _projectionLogAtMs = 0;
        const float BrakeFactorMin = 0.33f;
        const float BrakeFactorMax = 1.30f;
        // A factor rise means the car arrived under the commanded speed, so the corner's lead is longer than its
        // decel needs: the aim node steps toward the apex and the ramp ends later. One direction only - the factor
        // owns the other correction, so the two knobs cannot both lengthen the window.
        const float BrakeBufferStepNodes = 5f;
        const float BrakeBufferMinNodes = 1f;
        const float BrakeBufferMaxNodes = 30f;
        public const float BrakeRampIdealDeltaMps = 0.5f;
        const float BrakeRampErrorDivisor = 20f;
        const float CornerTightRadiusMetres = 50f;
        const float RequirementExtrapolationScale = 0.9f;
        const float TightCornerArmExcessMph = 10f;
        const float WideCornerArmExcessMph = 25f;
        const float BrakeHorizonMarginMetres = 40f;
        readonly Dictionary<int, CornerContext> _cornerContexts = new Dictionary<int, CornerContext>();
        int _cornersRevisionSeeded = -1;
        const int RouteSpeedPointSpacingNodes = 40;
        readonly List<RouteSpeedPoint> _routeSpeedPoints = new List<RouteSpeedPoint>();
        int _routeSpeedCursor = -1;
        int _routeSpeedRevisionSeeded = -1;
        RouteSpeedPoint _activeRouteSpeedPoint;
        int _planLogApexNode = -1;

        readonly HashSet<int> _skippedApexes = new HashSet<int>();
        int _brakeAnalysisApexNode = -1;
        int _brakeAnalysisLap = -1;
        bool _brakeAnalysisArmed;
        bool _brakeAnalysisFired;

        const float CornerSpeedLearnLimitMps = 10f;

        const float CornerSpeedLearnDeltaLimitMps = 2f;
        const float CornerSpeedLearnMinSlideShare = 0.75f;

        const float CornerSpeedLearnLostControlDegrees = 20f;

        const int CornerSpeedLearnQuietContactMs = 4000;

        const int CornerSpeedLearnHitOffTrackMs = 1000;

        const float CornerSpeedLearnMetreToOffset = 0.5f;

        const float CornerSpeedLearnVerdictMarginMetres = 1f;

        const float CornerSpeedLearnNoSlideRiseMps = 2f;
        const float CornerSpeedLearnReleaseMetres = 40f;

        const float ApexReachedMetres = 3f;

        const float RouteSpeedLearnBrake = 0.5f;
        const float RouteSpeedLearnFullThrottle = 0.9f;
        const float RouteSpeedLearnRateMph = 1f;
        const float RouteSpeedLearnMinSlideShare = 0.2f;
        const float RouteSpeedLearnSteerMin = 0.05f;
        const float RouteSpeedLearnWidenMetres = 0.5f;
        const float RouteSpeedLearnFallShareOfRate = 0.5f;
        const float RouteSpeedLearnLimitMph = 20f;
        static readonly bool RouteSpeedLearningEnabled = false; // parked while the trim is dialled: the offset then sits on its row
                const float RouteSpeedLearnMinStabilityCap = 0.999f;
        int _cornerSpeedApexNode = -1;
        int _cornerSpeedLastSampleMs = -1;
        float _cornerSpeedMinOutsideClearanceMetres = float.MaxValue;
        float _cornerSpeedWorstSlideDeg;
        bool _cornerSpeedHitOffTrack;
        int _cornerSpeedOutsideSamples;

        const int CornerVerdictMs = 2000;

        const float CornerDeselectMps = 25f;
        bool _cornerMode;
        bool _cornerSpeedAuthority;
        int _cornerModeNode = -1;
        float _cornerModeSpeed = 999f;
        int _cornerModeApexPassedAt = -1;
        readonly HashSet<int> _cornerSpeedLocked = new HashSet<int>();
        int _laneAvoidAt;
        int _sideBySideSteerAt;
        public float BrakeFactorForApex(int apexNode) => TryGetCornerContext(apexNode, out CornerContext context) ? context.BrakeFactor : CornerContext.BrakeFactorSeed;

        // Seed and learning are one number: the setting is where it starts, the learner moves it. The toggle only
        // stops the moving — turning Corner Speed Learning off leaves the menu's offset in force on every corner.
        public float CornerSpeedOffsetFor(int apexNode) => TryGetCornerContext(apexNode, out CornerContext context) ? context.CornerSpeedOffsetMps : 0f;
        float _divebombBrakeBonus;

        bool TryGetCornerContext(int apexNode, out CornerContext context)
        {
            SyncCornerContexts();
            return _cornerContexts.TryGetValue(apexNode, out context);
        }

        bool CornerRequiresBraking(int apexNode) => apexNode >= 0 && TryGetCornerContext(apexNode, out CornerContext context) && context.RequiresBraking;

        bool CornerAsksForBraking() => Brain.Corner != null && ARS.CornerApexSpeed(Brain.Corner.Point, this) < Car.Velocity.Length();

        CornerContext CornerContextForWrite(int apexNode)
        {
            if (!TryGetCornerContext(apexNode, out CornerContext context))
            {
                context = new CornerContext { BrakeFactor = CornerContext.BrakeFactorSeed };
                _cornerContexts[apexNode] = context;
            }
            return context;
        }

        // Every 40th node seeds the lattice; then each apex pulls its two nearest points onto half the cadence
        // either side of it — the half being where the numbers actually matter: the rear flank reads the approach
        // road up to 20 nodes back, the front flank the exit road from 20 nodes out. One behaviour, every corner.
        void SyncRouteSpeedPoints()
        {
            if (_routeSpeedRevisionSeeded == ARS.CornersRevision) return;
            _routeSpeedRevisionSeeded = ARS.CornersRevision;
            _routeSpeedPoints.Clear();
            _routeSpeedCursor = -1;
            _activeRouteSpeedPoint = null;

            int count = ARS.TrackPoints.Count;
            int spacing = RouteSpeedPointSpacingNodes;
            int flank = RouteSpeedPointSpacingNodes / 2;
            List<int> nodes = new List<int>();
            for (int node = 0; node < count; node += spacing) nodes.Add(node);

            foreach (CornerPoint corner in ARS.Corners)
            {
                int backIndex = -1, aheadIndex = -1;
                int backDelta = int.MinValue, aheadDelta = int.MaxValue;
                for (int i = 0; i < nodes.Count; i++)
                {
                    int delta = nodes[i] - corner.Node;
                    if (!ARS.IsPointToPoint)
                    {
                        delta = ((delta % count) + count) % count;
                        if (delta > count / 2) delta -= count;
                    }
                    if (delta < 0 && delta > backDelta) { backDelta = delta; backIndex = i; }
                    else if (delta > 0 && delta < aheadDelta) { aheadDelta = delta; aheadIndex = i; }
                }
                if (backIndex >= 0)
                    nodes[backIndex] = ARS.IsPointToPoint
                        ? (int)ARS.Clamp(corner.Node - flank, 0, count - 1)
                        : ((corner.Node - flank) % count + count) % count;
                if (aheadIndex >= 0)
                    nodes[aheadIndex] = ARS.IsPointToPoint
                        ? (int)ARS.Clamp(corner.Node + flank, 0, count - 1)
                        : ((corner.Node + flank) % count) % count;
            }

            nodes.Sort();
            for (int i = nodes.Count - 1; i > 0; i--)
                if (nodes[i] == nodes[i - 1]) nodes.RemoveAt(i);

            foreach (int node in nodes)
            {
                RouteSpeedPoint entry = new RouteSpeedPoint(ARS.TrackPoints[node].Node, TightestSweepRadiusNear(node, count));
                entry.SetTargetSpeedMps(RouteIdealSpeedForRadius(entry.Radius) * NearbyDownhillSpeedFactor(node, count));
                _routeSpeedPoints.Add(entry);
            }
            ARS.Log(ARS.LogImportance.Error, "[ROUTESEED build " + ARS.ReadBuildNumber() + "] " + Name
                + " seeded " + _routeSpeedPoints.Count + " points, flanks at half cadence");
        }

        // Half the seed spacing each side, so adjacent windows tile the route and no node falls between them —
        // except across an apex, where the window stops at the point's own node (ApexWindowClamp).
        float TightestSweepRadiusNear(int node, int count)
        {
            float tightest = float.MaxValue;
            int from = -RouteSpeedPointSpacingNodes / 2;
            int to = RouteSpeedPointSpacingNodes / 2;
            ApexWindowClamp(node, count, ref from, ref to);
            for (int offset = from; offset <= to; offset++)
            {
                int sample = ARS.IsPointToPoint ? (int)ARS.Clamp(node + offset, 0, count - 1)
                    : ((node + offset) % count + count) % count;
                float radius = ARS.TrackPoints[sample].SweepCurveRadius;
                if (radius < tightest) tightest = radius;
            }
            return tightest;
        }

        // The point bakes the worst of what is nearby, mirroring the radius lookup's window: the most inclined
        // trackpoint each way of the node, travelled forward, damped by the same hill law the old route term
        // carries. Downhill only - uphill never raises the target, and crest stays out.
        float NearbyDownhillSpeedFactor(int node, int count)
        {
            int from = -RouteSpeedPointSpacingNodes / 2;
            int to = RouteSpeedPointSpacingNodes / 2;
            ApexWindowClamp(node, count, ref from, ref to);
            float steepestDescent = 0f;
            for (int offset = from; offset <= to; offset++)
            {
                int sample = ARS.IsPointToPoint ? (int)ARS.Clamp(node + offset, 0, count - 1)
                    : ((node + offset) % count + count) % count;
                float descent = -ARS.TrackPoints[sample].SlopeSineTimes90 * ElevationToSlopeSine;
                if (descent > steepestDescent) steepestDescent = descent;
            }
            if (steepestDescent <= 0f) return 1f;
            float pitchDeg = ARS.RadToDeg((float)Math.Asin(ARS.Clamp(steepestDescent, 0f, 1f)));
            return (float)Math.Sqrt(ARS.HillGripFactorFromPitchAngle(pitchDeg, this));
        }

        // The apex is a boundary the sweep may not cross, not a reason to delete the point. An apex behind the
        // point means it reads only ahead of itself, an apex ahead means only behind — so no point ever bakes
        // ground the corner system owns, and the first point past a corner reads the road it is leaving on
        // instead of the corner it just left. A point boxed between two apexes keeps its own node alone.
        void ApexWindowClamp(int node, int count, ref int from, ref int to)
        {
            int reach = RouteSpeedPointSpacingNodes / 2 + 1;
            foreach (CornerPoint corner in ARS.Corners)
            {
                int delta = corner.Node - node;
                if (!ARS.IsPointToPoint)
                {
                    delta = ((delta % count) + count) % count;
                    if (delta > count / 2) delta -= count;
                }
                if (delta > reach || delta < -reach) continue;
                if (delta <= 0) from = 0;
                else to = 0;
            }
        }

        float RouteSpeedPointTargetSpeedMps(float fallbackMps)
        {
            SyncRouteSpeedPoints();
            RouteSpeedPoint point = ActiveRouteSpeedPoint();

            // No active point: the old base still owns the speed, never a 0 m/s stand-down.
            return point == null ? fallbackMps : point.TargetSpeedMps;
        }

        RouteSpeedPoint ActiveRouteSpeedPoint()
        {
            if (_routeSpeedPoints.Count == 0) return null;
            if (!LookAheads.TryGetValue(LookAhead.OneSec, out TrackPoint oneSecLook) || oneSecLook == null) return null;
            int oneSecFwd = ForwardNodeDistance(oneSecLook.Node);
            int nodeCount = ARS.TrackPoints.Count;
            if (_routeSpeedCursor < 0) _routeSpeedCursor = 0;
            int startCursor = _routeSpeedCursor;
            for (int steps = 0; steps < _routeSpeedPoints.Count; steps++)
            {
                // Signed car-to-point distance: a just-passed point folds to a small negative on circuits,
                // and on point-to-point the raw subtraction already reads negative.
                int ahead = ForwardNodeDistance(_routeSpeedPoints[_routeSpeedCursor].Node);
                if (!ARS.IsPointToPoint && ahead > nodeCount / 2) ahead -= nodeCount;
                if (ahead < 0)
                {
                    if (_routeSpeedCursor + 1 < _routeSpeedPoints.Count) _routeSpeedCursor++;
                    else if (ARS.IsPointToPoint) break;
                    else _routeSpeedCursor = 0;
                    continue;
                }
                // The next point takes over the moment it fits the 1 s horizon; only when the horizon cannot
                // reach it yet does the current point hold to its own node.
                int nextCursor = _routeSpeedCursor + 1 < _routeSpeedPoints.Count
                    ? _routeSpeedCursor + 1
                    : (ARS.IsPointToPoint ? -1 : 0);
                if (nextCursor < 0) break;
                int nextAhead = ForwardNodeDistance(_routeSpeedPoints[nextCursor].Node);
                if (!ARS.IsPointToPoint && nextAhead > nodeCount / 2) nextAhead -= nodeCount;
                if (nextAhead < 0 || nextAhead <= oneSecFwd)
                {
                    _routeSpeedCursor = nextCursor;
                    continue;
                }
                break;
            }
            RouteSpeedPoint selected = _routeSpeedPoints[_routeSpeedCursor];
            int toPoint = ForwardNodeDistance(selected.Node);
            if (!ARS.IsPointToPoint && toPoint > nodeCount / 2) toPoint -= nodeCount;
            if (_routeSpeedCursor != startCursor) LogRouteSpeedPointAdvance(startCursor, selected);
            // Trusted only within the car's 1 s horizon: a farther point's curve-baked speed would cap the straight.
            if (toPoint < 0 || toPoint > oneSecFwd)
            {
                _activeRouteSpeedPoint = null;
                return null;
            }
            _activeRouteSpeedPoint = selected;
            return selected;
        }

        void LogRouteSpeedPointAdvance(int fromCursor, RouteSpeedPoint point)
        {
            ARS.Log(ARS.LogImportance.Error, "[ROUTEPOINT build " + ARS.ReadBuildNumber() + "] "
                + Name + " cursor " + fromCursor + "->" + _routeSpeedCursor
                + (_routeSpeedCursor == 0 ? " wrap" : "")
                + " node " + point.Node
                + " r " + point.Radius.ToString("0", CultureInfo.InvariantCulture)
                + " tgt " + point.TargetSpeedMps.ToString("0.0", CultureInfo.InvariantCulture));
        }

        void SyncCornerContexts()
        {
            // The setting moved: carry every corner by the same delta so learning keeps its distance from the
            // seed. This runs before the revision gate and before any context is born, so a fresh corner is
            // seeded once and never shifted on top of that.
            if (ARS.CornerOffsetMph != _cornerOffsetSeedMph)
            {
                float shift = ARS.MphToMps(ARS.CornerOffsetMph - _cornerOffsetSeedMph);
                foreach (CornerContext context in _cornerContexts.Values)
                    context.CornerSpeedOffsetMps = ARS.Clamp(context.CornerSpeedOffsetMps + shift, -CornerSpeedLearnLimitMps, CornerSpeedLearnLimitMps);
                _cornerOffsetSeedMph = ARS.CornerOffsetMph;
            }

            if (_cornersRevisionSeeded == ARS.CornersRevision) return;
            _cornersRevisionSeeded = ARS.CornersRevision;
            foreach (CornerPoint corner in ARS.Corners)
                if (!_cornerContexts.ContainsKey(corner.Node))
                    _cornerContexts[corner.Node] = new CornerContext { Point = corner, BrakeFactor = CornerContext.BrakeFactorSeed };

            List<int> stale = new List<int>();
            foreach (int node in _cornerContexts.Keys)
                if (!ARS.Corners.Exists(corner => corner.Node == node)) stale.Add(node);
            foreach (int node in stale) _cornerContexts.Remove(node);
        }

        public float EffectiveBrakeFactor(int apexNode)
        {
            return ActiveManeuver.Type == ManeuverType.DiveBomb
                ? BrakeFactorForApex(apexNode) + _divebombBrakeBonus
                : BrakeFactorForApex(apexNode);
        }

        bool _isPassengerized = false;
        const float RivalSearchRangeMeters = 200f;
        const float NitrousPowerMultiplier = 5f;
        const float PlayerNitrousPowerMultiplier = 1f;
        const float NitrousMaxSteerDegrees = 5f;
        const float NitrousMinThrottle = 0.9f;
        const float NitrousMinRouteRadiusMeters = 800f;
        const int NitrousArmDelayMs = 6000;
        const int NitrousDurationMs = 3000;
        const float NitrousCornerLookaheadSeconds = 8f;
        const float NitrousLonelySpeedFraction = 0.5f;
        const float NitrousLonelyMinApexDistance = 500f;
        const float NitrousDefenseReachSeconds = 4f;
        const float NitrousNearbyRivalDistance = 80f;
        const float NitrousFasterReachMeters = 30f;
        const float NitrousMaxSlideDegrees = 1f;
        const float NitrousFinishExtraDistance = 100f;
        const string NitrousPtfxAsset = "veh_xs_vehicle_mods";
        const ulong CheatPowerIncreaseHash = 0xB59E4BD37AE292DB;
        const ulong MaxDriveGearHash = 0x24910C3D66BA770D;
        const ulong CurrentDriveGearHash = 0x56185A25D45A0DCD;

        const ulong FullyChargeNitrousHash = 0x1A2BCC8C636F9226;
        const ulong OverrideNitrousLevelHash = 0xC8E9B6B71B8E660D;

        const float ChillRivalCrowdDistance = 40f;
        const int ChillCrowdCount = 5;
        const float ChillReleaseRadiusMeters = 50f;
        public const float ChillStandoffDistance = 20f;
        const float ChillThrottleCap = 0.5f;
        const float ChillStandoffEscapeMargin = 2f;
        const float ChillMinSpeedMph = 30f;

        const float DivebombReleaseGapMeters = 40f;
        const float DivebombReleaseAngleDegrees = 25f;
        const float DefendLaterMarginSeconds = 1f;
        const float DefendCloseBehindMeters = 20f;
        const int CardMinDwellMs = 1000;
        const int CardCooldownMs = 1000;

        const float RivalLiftMeters = 3f;
        const float RivalLiftFloor = 0.1f;
        const float RivalHeightGateMeters = 2f;
        const float RocketBoostMinimumCurveRadius = 600f;
        const float SideBySideAssistRangeExtra = 3f;
        const float SideBySideFullAssistExtraGap = 1f;
        const float SideBySideMinimumAssist = 0.1f;
        const float SideBySideHeadingShare = 0.5f;

        int RivalsWithinDistance(float distance)
        {
            if (ARS.NoCollision) return 0;
            return ARS.Racers.Count(r => r.Car.Handle != Car.Handle && r.RacePosition < RacePosition && r.Car.Position.DistanceTo(Car.Position) <= distance);
        }

        public Maneuver ActiveManeuver = new Maneuver();
        readonly int[] _cardCooldownUntil = new int[8];
        public float DivebombBrakeBonus => _divebombBrakeBonus;

        bool CardCooled(ManeuverType type) => Game.GameTime >= _cardCooldownUntil[(int)type];
        bool CardMayEnd => Game.GameTime - ActiveManeuver.EnabledAtMs >= CardMinDwellMs;
        void EndCard()
        {

            if (ActiveManeuver.Type != ManeuverType.Yield) _cardCooldownUntil[(int)ActiveManeuver.Type] = Game.GameTime + CardCooldownMs;
            ActiveManeuver.Type = ManeuverType.None;
            ActiveManeuver.Target = null;
        }

        int _nitrousActiveUntil = 0;
        int _nitrousGsLoggedAt = 0;
        int _nitrousLapUsed = -1;
        int _nitroRefusalLoggedAt = 0;
        float _powerMultiplier = 1f;
        float _overspeedPowerLevel = 1f;
        const float SlideAverageSeconds = 0.5f;
        const int SlideAverageSlots = 128;
        readonly float[] _slideSamples = new float[SlideAverageSlots];
        readonly float[] _steerSamples = new float[SlideAverageSlots];
        readonly int[] _slideSampleAt = new int[SlideAverageSlots];
        int _slideSampleIndex;
        int _slideSampleCount;
        float _defendLaneOffset = 0f;
        float _avoidHoldLane = 0f;
        bool _avoidHoldActive = false;

        float _passLaneValue;
        bool _passLaneLatched;
        Racer _passLaneTarget;
        int _avoidHoldUntil = 0;
        float _appliedThrottleLastFrame = 1f;
        int _powerCounterLoggedAt = 0;

        public float Aggression = 50f;
        public float Pressure = 0f;
        const float PressureRange = 100f;
        const float PressureProximityRange = 100f;
        const float PressureRisePerSecond = 2f;
        const float PressureFallPerSecond = 30f;

        public bool FromNearby;

        public Racer(Vehicle RacerCar, Ped RacerPed)
        {
            Car = RacerCar;
            Driver = RacerPed;
            try { Name = RacerCar.FriendlyName; } catch (Exception) { Name = "Racer"; }
            if (Name == "NULL" || Name == null) { try { Name = Car.DisplayName.ToString()[0].ToString().ToUpper() + Car.DisplayName.ToString().Substring(1).ToLowerInvariant(); } catch (Exception) { Name = "Racer"; } }
            CarModelName = Name;

            if (Driver.IsPlayer) ControlledByPlayer = true;
            _halfSecondDueMs = Game.GameTime + (ARS.GetRandomInt(10, 50));
            _phaseOffsetMs = ARS.GetRandomInt(0, 500);

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

            Handling.LateralTractionCurveDegrees = ARS.RadToDeg(VehicleMemory.GetLateralTraction(Car));
            if (Handling.LateralTractionCurveDegrees < 1 || Handling.LateralTractionCurveDegrees > 100) Handling.LateralTractionCurveDegrees = 22;
            ARS.Log(ARS.LogImportance.Info, "TRlat for " + Car.DisplayName + ":" + Handling.LateralTractionCurveDegrees + "º");

            Handling.BrakingAbility = Car.MaxBraking;
            Handling.EstimatedTopSpeedMps = ARS.EngineTopSpeed(Car);

            ARS.Log(ARS.LogImportance.Info, "Top speed " + Car.DisplayName + ": geared "
                + Function.Call<float>(Hash._0x53AF99BAA671CA47, Car).ToString("0.0") + " m/s, estimate "
                + ARS.MpsToMph(Handling.EstimatedTopSpeedMps).ToString("0") + " mph");
            Handling.Acceleration = Function.Call<float>(Hash.GET_VEHICLE_ACCELERATION, Car);

            VehicleData.SteeringLock = ARS.RadToDeg(VehicleMemory.GetSteerLock(Car));
            if (VehicleData.SteeringLock < 1 || VehicleData.SteeringLock > 100) VehicleData.SteeringLock = 40;
            ARS.Log(ARS.LogImportance.Info, "Steerlock for " + Car.DisplayName + ":" + VehicleData.SteeringLock + "º");

            VehicleData.WheelBaseMetres = ARS.GetWheelBase(Car);

            ARS.Log(ARS.LogImportance.Info, "Wheelbase for " + Car.DisplayName + ":" + VehicleData.WheelBaseMetres + " m");
            Control.SteerDegrees = 0f;
            CurrentTrackPoint = ARS.TrackPoints.Last();
            Control.Brake = 0f;
            Control.Throttle = 0f;
            _powerMultiplier = 1f;
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
            _cornerModeDistanceNodes = 0;
            _cornerModeApexTargetNodes = 0;
            _cornerModeLastNode = -1;
            _cornerMode = false;
            _cornerModeNode = -1;
            _cornerModeSpeed = 999f;
            _cornerModeApexPassedAt = -1;
            _cornerSpeedAuthority = false;
            _routeOffsetSeedMph = ARS.Clamp(ARS.RouteOffsetMph, -RouteSpeedLearnLimitMph, RouteSpeedLearnLimitMph);
            SetRouteSpeedOffset(_routeOffsetSeedMph);
            // Every corner is about to be born from this seed, so the stamp starts on it: a reset must not then
            // read as a setting change and shift the fresh contexts off their own seed.
            _cornerOffsetSeedMph = ARS.CornerOffsetMph;
            _lapOdometer = 0;
            _lastLapNode = -1;
            _lapStartNode = -1;
            _lapStartOdometer = 0;
            ActiveManeuver.Type = ManeuverType.None;
            ActiveManeuver.Target = null;
            ActiveManeuver.EnabledAtMs = 0;
            Array.Clear(_cardCooldownUntil, 0, _cardCooldownUntil.Length);
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

            float livePace = ARS.ComputePaceIndex(modelTopSpeedMph, modelGrip, modelAccel, modelElectric);
            if (float.IsNaN(livePace) || float.IsInfinity(livePace)) ARS.ModelPaceIndexCache.TryGetValue(Car.Model.Hash.ToString(), out livePace);
            VehicleData.PaceIndex = livePace;
            VehicleData.TextPerformanceIndex = VehicleData.PaceIndex.ToString("0.0");

            if (!ControlledByPlayer) Name = _baseName + " (" + VehicleData.PaceIndex.ToString("0.00") + ")";
            DriverName = _baseName;

            _cornerContexts.Clear();
            _routeSpeedPoints.Clear();
            _routeSpeedCursor = -1;
            _routeSpeedRevisionSeeded = -1;
            _activeRouteSpeedPoint = null;
            _cornersRevisionSeeded = -1;
            _skippedApexes.Clear();
            _brakeAnalysisApexNode = -1;
            _brakeAnalysisLap = -1;
            _brakeAnalysisArmed = false;
            _brakeAnalysisFired = false;
            _laneHeldCorner = null;
            _laneTurnSign = 0;

            Car.Repair();
        }

        public void ComputeSteering()
        {
            _countersteerDemand01 = 0f;
            _slideShare = 0f;

            if (!TryGetSteerContext(out TrackPoint steerRefPoint, out float roadHalfWidthMetres))
            {
                Control.SteerDegrees = 0f;
                return;
            }

            float speedMps = Math.Max(Car.Velocity.Length(), 1f);

            Vector3 courseDir = Car.ForwardVector;
            if (Car.Velocity.LengthSquared() > 0.01f) courseDir = Car.Velocity.Normalized;

            float carHalfWidth = VehicleData.SlidingWidthMetres * 0.5f;

            float steerRefOffset = ARS.SignedLaneOffset(Car.Position, CurrentTrackPoint.Position, CurrentTrackPoint.Direction);

            float laneRequest = steerRefOffset;
            LaneReason laneReason = LaneReason.OwnLane;

            CornerPoint candidate = NearestApexAhead();

            CornerPoint previousLaneCorner = _laneHeldCorner;
            if (previousLaneCorner == null || HasPassedApex(previousLaneCorner.Node))
            {
                if (previousLaneCorner != null)
                {
                    FinishShapeError(previousLaneCorner);
                    LogApexEvent("Lane", false, previousLaneCorner.Node,
                        "apex passed · lap " + Lap + " · node " + CurrentTrackPoint.Node);
                }
                _laneHeldCorner = candidate;
                if (_laneHeldCorner != null)
                    LogApexEvent("Lane", true, _laneHeldCorner.Node, null);
            }
            CornerPoint lineCorner = _laneHeldCorner;

            float laneLimit = Math.Max(roadHalfWidthMetres - carHalfWidth - (IsWithinCorner(lineCorner) ? 0f : EdgeEmptySpaceMeters), 0f);

            int probeApexNode = lineCorner != null ? lineCorner.Node : -1;
            if (probeApexNode != _probeApexNode)
            {
                if (_probeApexNode >= 0) LogApexProbe(_probeApexNode);
                _probeApexNode = probeApexNode;
                _probeMaxInwardOffsetMetres = 0f;
                _probeNode = -1;
                _probeOffsetAtApexMetres = 0f;
                _probeAtApex = false;
            }
            if (lineCorner != null) UpdateApexProbe(lineCorner, steerRefOffset);

            int laneNode = TrackLoader.NodeAhead(CurrentTrackPoint.Node, speedMps * ARS.LanePreviewSeconds, ARS.TrackPoints.Count);
            float laneEdge = Math.Max(roadHalfWidthMetres - carHalfWidth, 0f);
            _laneField = ARS.TrackPoints[laneNode].LaneLineShare * laneEdge;
            int laneShapeNode = TrackLoader.NodeAhead(CurrentTrackPoint.Node, speedMps * ARS.LaneShapePreviewSeconds, ARS.TrackPoints.Count);
            float laneShapeNow = ClampShapedLane(ARS.TrackPoints[laneShapeNode].LaneLineShare * laneEdge, laneEdge, lineCorner);
            float aimRadius = ARS.TrackPoints[steerRefPoint.Node].PreciseCurveRadius;
            CornerPoint laneApex = lineCorner;
            float toApex = laneApex != null ? SignedMetresToNode(laneApex.Node) : 0f;
            _laneApexNode = laneApex != null ? laneApex.Node : -1;
            _laneShapeApexNode = _laneApexNode;

            int entranceNode = laneApex != null ? CornerEntranceNode(laneApex, laneApex.Node) : -1;
            float toEntrance = entranceNode >= 0 ? SignedMetresToNode(entranceNode) : 0f;
            bool following = ARS.AbsoluteLaneEnabled && Lap > 0
                && lineCorner != null && SignedMetresToNode(lineCorner.Node) > 0f
                && entranceNode >= 0 && toEntrance <= speedMps * ARS.LanePrepSeconds
                && _laneApexNode >= 0 && ARS.TrackPoints[_laneApexNode].LaneShapeStart >= 0;

            if (ARS.LaneErrorTelemetry)
            {
                int hereNode = CurrentTrackPoint.Node;
                TrackPoint here = ARS.TrackPoints[hereNode];
                if (hereNode != _shapeErrLastNode && here.LaneShaped && here.LaneShapeApex == _laneApexNode
                    && lineCorner != null && SignedMetresToNode(lineCorner.Node) > 0f)
                {
                    float half = Math.Max(here.TrackHalfWidth - VehicleData.SlidingWidthMetres * 0.5f, 0f);
                    Vector3 hereRight = Vector3.Normalize(Vector3.Cross(here.Direction, Vector3.WorldUp));
                    float carLane = Vector3.Dot(Car.Position - here.Position, hereRight);
                    _shapeErrSum += Math.Abs(carLane - here.LaneLineShare * half);
                    _shapeErrCount++;
                    _shapeErrLastNode = hereNode;
                }
            }

            float leanDeg = ARS.Remap(aimRadius, LaneLeanMaxRadius, LaneLeanMinRadius, 0f, LaneLeanMaxDegrees, true);
            Vector3 toSteerRef = steerRefPoint.Position - Car.Position;
            toSteerRef.Z = 0f;
            float leanLookahead = Math.Max(toSteerRef.Length(), 1f);
            float dynamicInset = Math.Min(laneLimit,
                leanLookahead * (float)Math.Sin(ARS.DegToRad(leanDeg)));
            bool dynamicOwns = false;
            float dynamicAsk = steerRefOffset + ARS.LaneInsideSign * LaneTurnSignToAim(ARS.TrackPoints[steerRefPoint.Node]) * dynamicInset;
            dynamicOwns = !following && Lap > 0 && aimRadius <= ARS.LaneDynamicRadius;
            laneRequest = dynamicOwns ? dynamicAsk : steerRefOffset;
            if (following)
            {

                bool beforeEntrance = toEntrance > speedMps * ARS.LaneShapePreviewSeconds;
                float aim = beforeEntrance
                    ? ClampShapedLane(ARS.TrackPoints[entranceNode].LaneLineShare * laneEdge, laneEdge, laneApex)
                    : laneShapeNow;

                laneRequest = aim;
                laneReason = LaneReason.CurveLean;
            }
            else if (dynamicOwns) laneReason = LaneReason.CurveLean;
            CornerPoint laneRef = laneApex;
            if (laneRef != null && !following)
            {

                float outward = TrackTurnSign(CurrentTrackPoint.Node, laneRef.Node, 0);
                if (outward != 0f && (laneRequest - steerRefOffset) * outward > 0f) laneRequest = steerRefOffset;
            }
            float requestLimit = following ? laneEdge : laneLimit;
            laneRequest = ARS.Clamp(laneRequest, -requestLimit, requestLimit);

            if (ActiveManeuver.Type == ManeuverType.DefendLane && SignedMetresToNode(_defendApexNode) > OutsideGateMeters(steerRefPoint))
            {
                laneRequest = ARS.Clamp(_defendLaneOffset, -laneLimit, laneLimit);
                laneReason = LaneReason.CommitCard;
            }
            float avoidAheadLane = ComputeAvoidanceLane(roadHalfWidthMetres);
            if (avoidAheadLane != 0f) { laneRequest = avoidAheadLane; laneReason = LaneReason.Avoidance; }
            if (_stuckPhase == StuckPhase.Drive && SignedMetresBeyondTrackEdge() > 0f) { laneRequest = NearestEdgeLane(steerRefOffset, laneLimit); laneReason = LaneReason.RecoveryEdge; }
            float walledLane = ApplyRivalWalls(laneRequest, roadHalfWidthMetres);
            if (walledLane != laneRequest) laneReason = LaneReason.WallClamp;
            laneRequest = walledLane;
            if (ARS.DebugToggles[Options.LockLaneCentre]) { laneRequest = LaneLockTestOffsetMeters; laneReason = LaneReason.DebugLock; }
            _laneRequest = laneRequest;
            _laneRequestReason = laneReason;
            if (laneReason == LaneReason.Avoidance) _laneAvoidAt = Game.GameTime;
            _carriedLane = laneRequest;
            if (Game.GameTime > _laneTraceAt)
            {
                _laneTraceAt = Game.GameTime + 1000;
                string mode = following ? "ABS" : (laneReason == LaneReason.CurveLean ? "DYN" : "OWN");
                if (_thrFinal < ThrottleLogBelow && Game.GameTime > _throttleLogAt)
                {
                    _throttleLogAt = Game.GameTime + 1000;
                    ARS.Log(ARS.LogImportance.Info, "thr " + Name + ": "
                        + _thrFinal.ToString("0.00") + " final, " + _thrWanted.ToString("0.00") + " wanted, "
                        + " | brk " + _brkFinal.ToString("0.00") + " wanted " + _brkWanted.ToString("0.00")
                        + " (" + Control.BrakeReason + " cap " + Control.MaxBrake.ToString("0.00") + ")"
                        + " | reason " + ARS.ThrottleCause(Control.ThrottleReason, _thrDelta, _thrWanted,
                            ThrottleSlewLimited, ThrottleFloorActive, _thrFloor, _thrTerm)
                        + " | src " + ARS.SpeedSourceName(this)
                        + " | d " + _thrDelta.ToString("+0.00;-0.00") + " tgt " + _thrTarget.ToString("0.0")
                        + " corner " + (_speedCorner >= 900f ? "-" : _speedCorner.ToString("0.0"))
                        + " route " + (_speedRoute >= 900f ? "-" : _speedRoute.ToString("0.0"))
                        + " now " + _thrActualMps.ToString("0.0") + " (" + _thrTerm + ")"
                        + " | floor " + _thrFloor.ToString("0.00") + (ThrottleSlewLimited ? " slew" : "")
                        + " | tcs " + Control.MaxThrottleFromTCS.ToString("0.00")
                        + " stab " + Control.MaxThrottleFromStability.ToString("0.00")
                        + " over " + Control.MaxThrottleFromOverspeed.ToString("0.00")
                        + " rival " + Control.MaxThrottleFromRival.ToString("0.00")
                        + " offt " + Control.MaxThrottleFromOffTrack.ToString("0.00")
                        + " rec " + Control.MaxThrottleFromRecovery.ToString("0.00"));
                }
                ARS.Log(ARS.LogImportance.Info, "lane " + Name + ": node " + CurrentTrackPoint.Node
                    + " offset " + steerRefOffset.ToString("0.00")
                    + ", field " + _laneField.ToString("0.00") + " @lane node " + laneNode
                    + ", asked " + _laneRequest.ToString("0.00") + " (" + _laneRequestReason + ")"
                    + " | " + mode
                    + " apex " + (lineCorner != null ? lineCorner.Node.ToString() + " in " + toApex.ToString("0") : "-")
                    + " gs " + VehicleData.GetLongitudinalGs(Car.ForwardVector).ToString("+0.00;-0.00"));
            }
            _laneOverride = laneReason != LaneReason.OwnLane || Math.Abs(_carriedLane - steerRefOffset) > LaneOverrideHoldMeters;

            Vector3 steerRight = Vector3.Cross(steerRefPoint.Direction, Vector3.WorldUp).Normalized;
            float offsetAtProjection = ARS.SignedLaneOffset(ProjectAhead(SteerPreviewSeconds), steerRefPoint.Position, steerRefPoint.Direction);

            float aimLane = _carriedLane;
            float aimPointOffset = aimLane - GsPreviewBlend * (offsetAtProjection - steerRefOffset);
            Vector3 aimPoint = steerRefPoint.Position + steerRight * aimPointOffset;
            _pursuitAimLateralM = ARS.SignedLaneOffset(aimPoint, Car.Position, Car.ForwardVector);

            float laneSteerDeg = PursuitSteerDegrees(aimPoint, courseDir, steerRefPoint.Direction);
            _kickAskDegrees = laneSteerDeg;
            if (LaneCrossingScale)
            {

                float crossingDeg = Vector3.SignedAngle(CurrentTrackPoint.Direction, Car.ForwardVector, Vector3.WorldUp);
                bool turningIn = _nextApexCorner != null && HasPassedBrakingTarget() && !HasPassedApex(_nextApexCorner.Node);
                if (!turningIn && laneSteerDeg * crossingDeg > 0f)
                    laneSteerDeg *= LaneCrossingShare(Math.Abs(crossingDeg), speedMps);
            }
            _steerPursuitDeg = laneSteerDeg;

            float sideBySideSteerDeg = ComputeSideBySideSteerCorrection(courseDir);
            if (Math.Abs(sideBySideSteerDeg) > 0.001f) _sideBySideSteerAt = Game.GameTime;

            const float ContactPushOuterGapMeters = 5f;

            const float ContactPushReachSeconds = 0.3f;
            const float ContactPushReachMaxMeters = 20f;
            const float ContactPushMinSpeedMps = 1f;
            float contactSteerDeg = 0f;
            Vector3 velDir = speedMps > 0.5f ? Car.Velocity / speedMps : courseDir;

            Vector3 velRight = Vector3.Cross(velDir, Vector3.WorldUp).Normalized;
            Vector3 myHoriz = new Vector3(Car.Velocity.X, Car.Velocity.Y, 0f);
            bool myDirectionKnown = myHoriz.Length() >= ContactPushMinSpeedMps;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || !r.RivalRacer.Car.Exists()) continue;
                Vector3 delta = r.RivalRacer.Car.Position - Car.Position;
                if (Math.Abs(delta.Z) > RivalHeightGateMeters) continue;
                float longDist = Math.Abs(Vector3.Dot(delta, velDir));
                float latDist = Math.Abs(Vector3.Dot(delta, velRight));
                float longGate = (VehicleData.ModelDimensions.Y + r.RivalRacer.VehicleData.ModelDimensions.Y) * 0.5f + 1f;
                if (Vector3.Dot(delta, velDir) > 0f)
                    longGate += Math.Min(speedMps * ContactPushReachSeconds, ContactPushReachMaxMeters);

                float combinedHalfWidth = (VehicleData.SlidingWidthMetres + r.RivalRacer.VehicleData.SlidingWidthMetres) * 0.5f;
                if (longDist > longGate || latDist > combinedHalfWidth + ContactPushOuterGapMeters) continue;
                Vector3 rivalVel = r.RivalRacer.Car.Velocity;
                float latSide = Vector3.Dot(delta, velRight);
                float latRelVel = Vector3.Dot(rivalVel - Car.Velocity, velRight);
                if (latSide * latRelVel >= 0f) continue;

                Vector3 rivalHoriz = new Vector3(rivalVel.X, rivalVel.Y, 0f);
                if (!myDirectionKnown || rivalHoriz.Length() < ContactPushMinSpeedMps) continue;
                float surfaceGap = Math.Max(latDist - combinedHalfWidth, 0f);
                float ramp = ARS.Remap(surfaceGap, ContactPushOuterGapMeters, 0f, 0f, 1f, true);
                contactSteerDeg += Vector3.SignedAngle(myHoriz, rivalHoriz, Vector3.WorldUp) * ramp;
            }

            const float steerKP = 1.0f;

            float fwdSpeed = ARS.GetForwardSpeed(Car);

            float yawTarget = 0f;
            if (SteerDampingAimReference && fwdSpeed > 0f && Math.Abs(VehicleData.SlideAngle) < Handling.LateralTractionCurveDegrees * SlidingFraction) yawTarget = ARS.RadToDeg(fwdSpeed * _steerAimCurvature);
            float yawRateToDamp = VehicleData.YawRotationPerSecondDegrees - yawTarget;
            float damperGain = SteerDampingFor();
            float nonDamperSteerDeg = (steerKP * sideBySideSteerDeg) + (steerKP * laneSteerDeg);
            float damperTermDeg = -damperGain * yawRateToDamp * DamperEffectScale;

            if (damperTermDeg * nonDamperSteerDeg > 0f) damperTermDeg = 0f;

            float damperOvershootDeg = Math.Abs(damperTermDeg) - Math.Abs(nonDamperSteerDeg);
            if (damperTermDeg * nonDamperSteerDeg < 0f && damperOvershootDeg > 0f)
            {
                float crossingDeg = Math.Min(damperOvershootDeg * DamperCrossingShare, ARS.DamperCountersteerMaxDeg);
                damperTermDeg = -Math.Sign(nonDamperSteerDeg) * (Math.Abs(nonDamperSteerDeg) + crossingDeg);
            }

            float damperCeilingDeg = Math.Abs(nonDamperSteerDeg) + ARS.DamperCountersteerMaxDeg;
            damperTermDeg = ARS.Clamp(damperTermDeg, -damperCeilingDeg, damperCeilingDeg);
            _damperTermDeg = damperTermDeg;
            Control.SteerDegrees = nonDamperSteerDeg + damperTermDeg;

            if (ARS.SteerTrace && ControlledByPlayer)
                ARS.Log(ARS.LogImportance.Info, "steertrace t=" + Game.GameTime
                    + " ask=" + Control.SteerDegrees.ToString("0.00")
                    + " in=" + ARS.Remap(Control.SteerDegrees, -VehicleData.SteeringLock, VehicleData.SteeringLock, -1f, 1f, true).ToString("0.000")
                    + " yaw=" + VehicleData.YawRotationPerSecondDegrees.ToString("0.0")
                    + " slip=" + _slip.FrontPct.ToString("0")
                    + " mph=" + ARS.MpsToMph(fwdSpeed).ToString("0")
                    + " cref=" + Vector3.SignedAngle(new Vector3(Car.ForwardVector.X, Car.ForwardVector.Y, 0f), new Vector3(courseDir.X, courseDir.Y, 0f), Vector3.WorldUp).ToString("0.0")
                    + " aimoff=" + aimPointOffset.ToString("0.00")
                    + " proj=" + offsetAtProjection.ToString("0.00")
                    + " ref=" + steerRefOffset.ToString("0.00")
                    + " rawbear=" + _pursuitRawBearingDeg.ToString("0.0")
                    + " dist=" + _pursuitAimDistanceM.ToString("0.0")
                    + " lat=" + _pursuitAimLateralM.ToString("0.00"));

            float slideDeg = VehicleData.SlideAngle;
            if (float.IsNaN(slideDeg) || float.IsInfinity(slideDeg)) slideDeg = 0f;

            float slideShare = TRLateralAtSpeed > 0.01f ? Math.Abs(slideDeg) / TRLateralAtSpeed : 0f;
            _slideShare = slideShare;
            float countersteerAmount = 0f;
            if (slideShare > CountersteerBlendStart)
                countersteerAmount = 0.5f * Math.Min(slideShare - CountersteerBlendStart,
                    CountersteerBlendMid - CountersteerBlendStart) / (CountersteerBlendMid - CountersteerBlendStart);
            if (slideShare > CountersteerBlendMid)
                countersteerAmount += 0.5f * Math.Min(slideShare - CountersteerBlendMid,
                    CountersteerBlendFull - CountersteerBlendMid) / (CountersteerBlendFull - CountersteerBlendMid);

            bool rotationReversed = VehicleData.YawRotationPerSecondDegrees * Math.Sign(slideDeg) < 0f;
            if (rotationReversed && countersteerAmount > 0f) countersteerAmount *= CountersteerRecoilShare;

            _countersteerDemand01 = countersteerAmount;
            if (countersteerAmount > 0f && !KickWantsSlide())
                Control.SteerDegrees -= Math.Sign(slideDeg) * countersteerAmount * Math.Abs(slideDeg);

            if (Math.Abs(_steerAimBearingDegrees) < SmallAimBearingDegrees && _countersteerDemand01 <= 0f) Control.SteerDegrees = 0f;
            Control.SteerDegrees += steerKP * contactSteerDeg;

            if (float.IsNaN(Control.SteerDegrees) || float.IsInfinity(Control.SteerDegrees))
                Control.SteerDegrees = 0f;

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

        float ComputeSideBySideSteerCorrection(Vector3 courseDir)
        {
            float correction = 0f;
            foreach (Rival rival in Brain.Rivals)
            {
                if (rival.RivalRacer == null || !rival.RivalRacer.Car.Exists()) continue;

                Vector3 relativeOffset = ARS.EntityRelativeOffset(Car, rival.RivalRacer.Car);
                if (Math.Abs(relativeOffset.Z) > RivalHeightGateMeters) continue;

                float bodyHalfSum = (VehicleData.ModelDimensions.Y + rival.RivalRacer.VehicleData.ModelDimensions.Y) * 0.5f;
                if (Math.Abs(relativeOffset.Y) > bodyHalfSum) continue;

                float lateralDistance = Math.Abs(relativeOffset.X);

                float fullAssistDistance = rival.CombinedSize.X + SideBySideFullAssistExtraGap;
                float wideDistance = rival.CombinedSize.X + SideBySideAssistRangeExtra;
                if (lateralDistance > wideDistance) continue;

                Vector3 toRival = rival.RivalRacer.Car.Position - Car.Position;
                Vector3 lateralAxis = toRival - Car.ForwardVector * Vector3.Dot(toRival, Car.ForwardVector);
                float lateralGap = lateralAxis.Length();

                if (lateralGap > 0.01f && Vector3.Dot(rival.RivalRacer.Car.Velocity, lateralAxis / lateralGap) >= 0f) continue;

                float proximity = ARS.Remap(lateralDistance, wideDistance, fullAssistDistance, SideBySideMinimumAssist, 1f, true);
                Vector3 rivalFlat = new Vector3(rival.RivalRacer.Car.ForwardVector.X, rival.RivalRacer.Car.ForwardVector.Y, 0f);
                Vector3 courseFlat = new Vector3(courseDir.X, courseDir.Y, 0f);
                float headingDifference = -Vector3.SignedAngle(rivalFlat, courseFlat, Vector3.WorldUp);
                if (float.IsNaN(headingDifference) || float.IsInfinity(headingDifference)) continue;

                correction += headingDifference * SideBySideHeadingShare * proximity;
            }
            return correction;
        }

        const float LaneLockTestOffsetMeters = 0.1f;
        const float PursuitGain = 2f;
        const float MaxPursuitBearingDegrees = 90f;

        const float RouteSteerAngleCapMinMph = 20f;
        const float RouteSteerAngleCapTightMph = 60f;
        const float RouteSteerAngleCapWideDegrees = 45f;
        const float RouteSteerAngleCapTightDegrees = 5f;
        const float SmallAimBearingDegrees = 1f;
        const float LaneCrossingLateralSpeedMps = 1f;
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

        float PursuitSteerFromBearing(float bearing, float distance)
        {
            bearing = ARS.Clamp(bearing, -MaxPursuitBearingDegrees, MaxPursuitBearingDegrees);
            return ARS.RadToDeg((float)Math.Atan(PursuitGain * Math.Sin(ARS.DegToRad(bearing)) * VehicleData.WheelBaseMetres / distance));
        }

        float PursuitSteerDegrees(Vector3 aimPoint, Vector3 heading, Vector3 refDirection)
        {
            Vector3 toAim = aimPoint - Car.Position;

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

            float reach = headingFlat.Length() > 0.01f
                ? Math.Max(1f, Vector3.Dot(toAimFlat, Vector3.Normalize(headingFlat)))
                : 1f;

            _pursuitRawBearingDeg = bearing;

            float speedMph = ARS.MpsToMph(Car.Velocity.Length());
            if (!_cornerMode && speedMph >= RouteSteerAngleCapMinMph && SignedMetresBeyondTrackEdge() <= 0f)
            {
                Vector3 refFlat = new Vector3(refDirection.X, refDirection.Y, 0f);
                if (refFlat.LengthSquared() > 0.0001f && headingFlat.LengthSquared() > 0.0001f)
                {

                    float capDeg = ARS.Remap(speedMph, RouteSteerAngleCapTightMph, RouteSteerAngleCapMinMph,
                        RouteSteerAngleCapTightDegrees, RouteSteerAngleCapWideDegrees, true);
                    float aimVsRoute = Vector3.SignedAngle(refFlat, toAimFlat, Vector3.WorldUp);
                    float headingVsRoute = Vector3.SignedAngle(refFlat, headingFlat, Vector3.WorldUp);
                    bearing = ARS.Clamp(aimVsRoute, -capDeg, capDeg) - headingVsRoute;
                }
            }
            _pursuitAimDistanceM = distance;
            _steerAimBearingDegrees = bearing;
            _steerAimCurvature = 2f * (float)Math.Sin(ARS.DegToRad(bearing)) / reach;
            return PursuitSteerFromBearing(bearing, reach);
        }

        static readonly string[] LineVerdictNames = { "taken", "no apex", "apex passed", "radius 50 m or wider", "not fast enough", "beyond the window", "past the entrance", "aligned out", "inside the gate" };
        static readonly string[] LeanVerdictNames = { "lean in", "lean out", "lean off" };

const float LaneOverrideHoldMeters = 0.1f;

        public const float LaneTaperNearMeters = 2.5f;

const float RequirementLookaheadSeconds = 3.95f;
        const float EdgeEmptySpaceMeters = 0.5f;
        const float IdealLineBlendWidthFactor = 6f;
        const float IdealLineBlendMinMeters = 20f;
        public const float IdealLineMaxRadiusMeters = 50f;

        const float LaneLeanMaxRadius = 800f;
        const float LaneLeanMinRadius = 200f;
        const float LaneLeanMaxDegrees = 20f;

        bool CornerIsLaned(CornerPoint corner) => corner.DetectedRadius < IdealLineMaxRadiusMeters;

        CornerPoint _laneHeldCorner;

        float _shapeErrSum;
        int _shapeErrCount;
        int _shapeErrLastNode = -1;

        void FinishShapeError(CornerPoint corner)
        {
            if (ARS.LaneErrorTelemetry && _shapeErrCount > 0)
            {
                float average = _shapeErrSum / _shapeErrCount;
                ARS.Log(ARS.LogImportance.Info, "lane error " + Name + " apex " + corner.Node + ": mean "
                    + average.ToString("0.00") + " m over " + _shapeErrCount + " shaped nodes");
            }
            _shapeErrSum = 0f;
            _shapeErrCount = 0;
            _shapeErrLastNode = -1;
        }

        float ClampShapedLane(float metres, float laneEdge, CornerPoint apex)
        {
            if (apex == null) return metres;
            float outsideSign = -ARS.InsideSign(apex.Angle);
            if (outsideSign == 0f) return metres;
            float limit = Math.Max(laneEdge - ARS.LaneShapeOuterMarginMetres, 0f);
            if (metres * outsideSign > limit) return outsideSign * limit;
            return metres;
        }

        CornerPoint LastCornerOfLap()
        {
            CornerPoint last = null;
            foreach (CornerPoint corner in ARS.Corners)
                if (last == null || corner.Node > last.Node) last = corner;
            return last;
        }

        bool CornerBeyondLastLapCorner(int node, CornerPoint lastLapCorner, bool lastLapCornerPassed)
        {
            if (node < 0 || lastLapCorner == null || lastLapCornerPassed) return false;
            return ForwardNodeDistance(node) > ForwardNodeDistance(lastLapCorner.Node);
        }

        CornerPoint NearestApexAhead()
        {
            int count = ARS.TrackPoints.Count;
            if (count < 10) return null;

            CornerPoint lastLapCorner = LastCornerOfLap();
            bool lastLapCornerPassed = lastLapCorner == null || HasPassedApex(lastLapCorner.Node);

            CornerPoint best = null;
            float bestMeters = float.MaxValue;
            foreach (CornerPoint corner in ARS.Corners)
            {
                if (!CornerIsLaned(corner)) continue;
                float metres = SignedMetresToNode(corner.Node);
                float reach = Math.Max(Math.Max(Car.Velocity.Length(), 1f) * ARS.LanePreviewSeconds, ARS.LaneApexMinReachMeters);
                if (!ApexAheadOfLookahead(corner, reach)) continue;
                if (metres < 0f || metres >= bestMeters) continue;
                if (CornerBeyondLastLapCorner(corner.Node, lastLapCorner, lastLapCornerPassed)) continue;
                if (_skippedApexes.Contains(corner.Node)) continue;
                bestMeters = metres;
                best = corner;
            }

            return best;
        }
        const float AvoidAngleGateDegrees = 20f;
        const float SameSectionMaxGapMeters = 30f;
        const float WallApproachSeconds = 5f;
        internal const float EngineHealthHalf = 500f;
        internal const float BodyHealthShare = 0.5f;
        const float WallSideDeadbandMeters = 0.3f;
        const float AvoidHoldBehindMeters = 14f;
        const float AvoidHoldLateralMeters = 1.5f;
        const int AvoidHoldTimeoutMs = 8000;

        internal static bool IsRaceFit(Vehicle car) =>
            car.EngineHealth >= EngineHealthHalf && car.BodyHealth >= Math.Max(car.MaxHealth, 1) * BodyHealthShare;

        float ComputeAvoidanceLane(float roadHalfWidthMetres)
        {

            bool avoidHoldEnabled = false;
            float passLane = ComputeAvoidAheadLane(roadHalfWidthMetres);
            if (!avoidHoldEnabled) return passLane;
            if (passLane != 0f)
            {
                _avoidHoldLane = passLane;
                _avoidHoldActive = true;
                _avoidHoldUntil = Game.GameTime + AvoidHoldTimeoutMs;
                return passLane;
            }
            if (!_avoidHoldActive) return 0f;

            if (Game.GameTime > _avoidHoldUntil || Math.Abs(_avoidHoldLane) > roadHalfWidthMetres)
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

        float ComputeAvoidAheadLane(float roadHalfWidthMetres)
        {
            float carHalfWidth = VehicleData.SlidingWidthMetres * 0.5f;
            float trackBound = roadHalfWidthMetres - carHalfWidth;
            float aggroBuffer = ARS.Remap(Aggression, 100f, 0f, 0.2f, 1.2f, true);
            float nodeOffset = Brain.CurrentPerception.CurrentLaneOffsetMetres;

            Rival target = Brain.AvoidanceTarget;
            if (target == null || target.RivalRacer == null) { _passLaneLatched = false; return 0f; }

            bool latchHolds = _passLaneLatched && _passLaneTarget == target.RivalRacer
                && Math.Abs(_passLaneValue - target.NodeOffset) >= target.CombinedWidth;
            if (latchHolds) return _passLaneValue;

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
                if (!ARS.IsBetween(Math.Abs(r.DirectionDiffDegrees), 0f, AvoidAngleGateDegrees)) continue;
                if (!ARS.IsBetween(r.FrontGapMetres, 0f, 3f) && !ARS.IsBetween(r.TimeToReachSeconds, 0f, 5f)) continue;

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

            float buffer = rival.RivalRacer.VehicleData.SlidingWidthMetres * 0.5f + aggroBuffer;

            Vector3 predHalf = ProjectAhead(0.5f);
            Vector3 predDir = ProjectAhead(1f) - predHalf;
            float predLen = predDir.Length();
            if (predLen > 1f) predDir /= predLen;
            else predDir = Car.Velocity.Length() > 0.5f ? Car.Velocity.Normalized : Car.ForwardVector;
            float rivalSide = Vector3.Dot(Vector3.Cross(predDir, Vector3.WorldUp), rival.RivalRacer.Car.Position - predHalf);
            passLeft = rivalSide > 0f;

            passLane = passLeft ? rivalLane - buffer - carHalfWidth : rivalLane + buffer + carHalfWidth;

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

        float ClampLaneToWalls(float lane, float roadHalfWidthMetres, bool leftConstrained, bool rightConstrained)
        {

            float carHalfWidth = VehicleData.SlidingWidthMetres * 0.5f;
            float trackBound = roadHalfWidthMetres;
            float clampLeft = leftConstrained ? Math.Max(_avoidLeftWall + carHalfWidth, -trackBound) : Math.Max(_avoidLeftWall, -trackBound);
            float clampRight = rightConstrained ? Math.Min(_avoidRightWall - carHalfWidth, trackBound) : Math.Min(_avoidRightWall, trackBound);
            if (clampLeft > clampRight)
            {

                float collapsed = (clampLeft + clampRight) * 0.5f;
                return ARS.Clamp(collapsed, -trackBound, trackBound);
            }
            return ARS.Clamp(lane, clampLeft, clampRight);
        }

        float ApplyRivalWalls(float requestLane, float roadHalfWidthMetres)
        {

            float trackBound = roadHalfWidthMetres;

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

            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null) continue;

                bool overlaps = Math.Abs(r.RouteGapAhead) < r.CombinedSize.Y;

                bool approaching = r.TimeToReachSeconds < WallApproachSeconds;
                if (!overlaps && !approaching) continue;

                float aggroBuffer = ARS.Remap(Aggression, 100f, 0f, 0.2f, 1.2f, true);

                float rivalBuffer = r.RivalRacer.VehicleData.SlidingWidthMetres * 0.5f + aggroBuffer;

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

            targetLeftWall = Math.Min(targetLeftWall, trackBound);
            targetRightWall = Math.Max(targetRightWall, -trackBound);

            if (targetLeftWall > targetRightWall)
            {
                float corridorHalf = Math.Min(1f, trackBound);
                float mid = ARS.Clamp((targetLeftWall + targetRightWall) * 0.5f, -trackBound + corridorHalf, trackBound - corridorHalf);
                targetLeftWall = mid - corridorHalf;
                targetRightWall = mid + corridorHalf;
            }

            _leftWallHeld = leftConstrained;
            _rightWallHeld = rightConstrained;
            _wallRoadWide = roadHalfWidthMetres;

            float wallCloseRate = 10f * TickScale;
            float wallOpenRate = 5f * TickScale;
            float wallSnapMeters = VehicleData.SlidingWidthMetres;

            _avoidLeftWall = Math.Min(_avoidLeftWall, trackBound);
            _avoidRightWall = Math.Max(_avoidRightWall, -trackBound);
            float leftTarget = leftConstrained ? targetLeftWall : -trackBound;
            float rightTarget = rightConstrained ? targetRightWall : trackBound;

            if (leftTarget > _avoidLeftWall + wallSnapMeters)
                _avoidLeftWall = Math.Max(leftTarget - wallSnapMeters, -trackBound);
            if (rightTarget < _avoidRightWall - wallSnapMeters)
                _avoidRightWall = Math.Min(rightTarget + wallSnapMeters, trackBound);

            float leftStep = leftTarget > _avoidLeftWall ? wallCloseRate : wallOpenRate;
            float rightStep = rightTarget < _avoidRightWall ? wallCloseRate : wallOpenRate;
            _avoidLeftWall = ARS.Clamp(leftTarget, _avoidLeftWall - leftStep, _avoidLeftWall + leftStep);
            _avoidRightWall = ARS.Clamp(rightTarget, _avoidRightWall - rightStep, _avoidRightWall + rightStep);
            _activeRivalWallCount = (_avoidLeftWall > -trackBound ? 1 : 0) + (_avoidRightWall < trackBound ? 1 : 0);

            return ClampLaneToWalls(requestLane, roadHalfWidthMetres, leftConstrained, rightConstrained);
        }
        const float SlidingFraction = 0.3f;

        const float CountersteerBlendStart = 1f;
        const float CountersteerBlendMid = 2f;
        const float CountersteerBlendFull = 4f;
        const float CountersteerRecoilShare = 0.5f;
        const float CountersteerPedalCap = 0.5f;
        const float CountersteerMinSpeedMph = 10f;

        const float SlideBrakeCapStartShare = 0.5f;
        const float SlideBrakeCapEndShare = 1.5f;
        const float SlideBrakeCap = 0.33f;
        const float SteerSlewRate = 120f;

        const int AirborneSteerMinWheelsOff = 4;
        const float AirborneSteerShareOfSlide = 0.25f;
        static readonly bool SlowCornerSlipAllowance = true;
        const float SlowCornerSlipFullMph = 15f;
        const float SlowCornerSlipDoubleMph = 5f;
        const float DamperCrossingShare = 1f;
        const float DamperEffectScale = 1f;
        const bool SteerDampingEnabled = true;
        const bool SteerDampingAimReference = true;
        float SteerDampingFor()
        {

            return SteerDampingEnabled ? ARS.SteerDampingGain / Math.Max(VehicleData.BaseMechanicalGrip, 1f) : 0f;
        }
        const float VanillaSteerReductionPerMps = 0.075f;
        const float SteerCapGripReference = 1f;
        const float SteerCapGripFloor = 0.5f;
        const float EngineSteerWheelOuterShare = 0.75f;
        const float PeakSlipOuterCommandShare = 1f / EngineSteerWheelOuterShare;
        const float SteerLimitRampStartMph = 5f;
        const float SteerLimitRampEndMph = 30f;

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

        float AckermannCeilingDegrees(float fwdSpeed)
        {
            float grip = VehicleData.CurrentMechanicalGrip;
            if (float.IsNaN(grip) || fwdSpeed <= 0f) return VehicleData.SteeringLock;
            float ratio = VehicleData.WheelBaseMetres * grip * Handling.Gravity / (fwdSpeed * fwdSpeed);
            if (float.IsNaN(ratio) || ratio <= 0f) return VehicleData.SteeringLock;
            return ARS.RadToDeg((float)Math.Atan(ratio));
        }

        public float TRLateralAtSpeed = 22f;
        public float SteerAskDegrees;

        void UpdateTRLateralAtSpeed()
        {
            float speed = Car.Velocity.Length();
            TRLateralAtSpeed = float.IsNaN(speed) ? 0f : LateralPeakAtSpeed(speed);
        }

        float LateralPeakAtSpeed(float speed)
        {
            float lat = Handling.LateralTractionCurveDegrees;
            return lat <= 0.01f ? 0f : lat / (1f + Math.Min(5f, 0.1f * speed));
        }

        public struct SlipState
        {
            public float FrontPct;
            public float RearPct;
            public bool Readable;
        }

        SlipState MeasureFrontSlip(float fwdSpeed)
        {

            if (fwdSpeed <= 0f) return new SlipState();
            return ARS.SlipFromMemory ? MeasureSlipFromMemory(fwdSpeed) : MeasureSlipFromTheory(fwdSpeed);
        }

        float SlowCornerSlipShare(float fwdSpeed)
        {

            if (!SlowCornerSlipAllowance) return 1f;
            float mph = ARS.MpsToMph(fwdSpeed);
            float slow = ARS.Clamp((SlowCornerSlipFullMph - mph) / (SlowCornerSlipFullMph - SlowCornerSlipDoubleMph), 0f, 1f);
            return 1f + slow;
        }

        float AllowedSlipShare(float fwdSpeed)
        {
            float peakDeg = LateralPeakAtSpeed(fwdSpeed);
            float toleranceShare = peakDeg > 0.01f ? ARS.PeakSlipToleranceDeg / peakDeg : 0f;
            return Math.Max(0.05f, ARS.PeakSlipTarget) * SlowCornerSlipShare(fwdSpeed) + toleranceShare;
        }

        float AllowedSlipDegrees(float fwdSpeed)
        {
            return LateralPeakAtSpeed(fwdSpeed) * AllowedSlipShare(fwdSpeed);
        }

        const float ExitTrimPedalStart = 0.5f;

        static readonly bool WheelTrimUnderPower = false;

        float AllowedSlipTrim(float fwdSpeed)
        {
            if (!WheelTrimUnderPower) return 1f;

            float ramp = ARS.Clamp((Control.Throttle - ExitTrimPedalStart) / (1f - ExitTrimPedalStart), 0f, 1f);
            ramp *= _exitRoomArmed ? _gsLaneFloorShare01 : 0f;
            if (ramp <= 0f) return 1f;

            float baseShare = Math.Max(0.05f, AllowedSlipShare(fwdSpeed));
            float exitShare = ARS.Clamp(ARS.TargetUnderPowerPct, 5, 100) * 0.01f;
            float blended = baseShare + (exitShare - baseShare) * ramp;
            return baseShare / Math.Max(0.05f, blended);
        }

        SlipState MeasureSlipFromMemory(float fwdSpeed)
        {
            SlipState s = new SlipState();
            List<float> slips = ARS.WheelSideSlipAngles(Car);
            if (slips.Count < 2) return s;
            s.Readable = true;

            float rawLeft = Math.Abs(slips[0]);
            float rawRight = Math.Abs(slips[1]);

            float rawOuter = Math.Abs(Control.SteerDegrees) < FrontSlipNeutralSteerDeg
                ? 0.5f * (rawLeft + rawRight)
                : Math.Min(rawLeft, rawRight);

            float capShare = AllowedSlipShare(fwdSpeed);
            s.FrontPct = 100f * rawOuter / capShare;
            s.RearPct = slips.Count >= 4
                ? 100f * Math.Max(Math.Abs(slips[2]), Math.Abs(slips[3])) / capShare
                : 0f;
            return s;
        }

        SlipState MeasureSlipFromTheory(float fwdSpeed)
        {
            SlipState s = new SlipState();
            float gripGs = VehicleData.CurrentMechanicalGrip;
            float speed = Math.Abs(fwdSpeed);
            float wheelBase = Math.Max(VehicleData.WheelBaseMetres, 0.5f);

            if (float.IsNaN(speed) || float.IsNaN(gripGs) || float.IsNaN(wheelBase)) return s;
            if (speed < FrontSlipTheoryMinSpeedMps || gripGs <= 0.01f || wheelBase <= 0.5f) return s;

            s.Readable = true;
            float available = gripGs * FrontSlipGravity * AllowedSlipShare(fwdSpeed);

            float commandDeg = Math.Abs(Control.SteerDegrees);
            float innerRadius = commandDeg > FrontSlipTheoryMinCommandDeg
                ? wheelBase / (float)Math.Tan(commandDeg * (float)Math.PI / 180f)
                : float.MaxValue;
            float demand = speed * speed / Math.Max(innerRadius, 0.5f);

            s.FrontPct = 100f * demand / available;
            return s;
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

        float ResolveSteerCeiling(float fwdSpeed)
        {

            if (!SpeedSteerCeiling) return VehicleData.SteeringLock;
            float ceiling = LateralPeakAtSpeed(fwdSpeed) > 0.01f ? PeakSlipCeilingAt(AllowedSlipDegrees(fwdSpeed)) : GeometrySteerCeiling(fwdSpeed);
            float speedMph = ARS.MpsToMph(fwdSpeed);
            if (speedMph >= SteerLimitRampEndMph) return ceiling;

            float endSpeed = ARS.MphToMps(SteerLimitRampEndMph);
            float endCeiling = LateralPeakAtSpeed(endSpeed) > 0.01f ? PeakSlipCeilingAt(AllowedSlipDegrees(endSpeed)) : GeometrySteerCeiling(endSpeed);

            return Math.Max(ceiling, SteerLimitRamp(speedMph, endCeiling));
        }
        static readonly bool SpeedSteerCeiling = false;

        float SteerLimitRamp(float speedMph, float ceiling)
        {
            if (speedMph >= SteerLimitRampEndMph) return ceiling;

            return ARS.Remap(speedMph, SteerLimitRampEndMph, SteerLimitRampStartMph, ceiling, VehicleData.SteeringLock, true);
        }

        void ApplySteerLimits()
        {

            if (float.IsNaN(Control.SteerDegrees) || float.IsInfinity(Control.SteerDegrees))
            {
                Control.SteerDegrees = 0f;
                return;
            }

            float requestedSteer = Control.SteerDegrees;
            SteerAskDegrees = requestedSteer;
            float fwdSpeed = Vector3.Dot(Car.Velocity, Car.ForwardVector);

            bool countersteering = Math.Sign(requestedSteer) != Math.Sign(VehicleData.YawRotationPerSecondDegrees);
            float speedCeiling = VehicleData.SteeringLock;
            if (fwdSpeed > 0f) speedCeiling = ResolveSteerCeiling(fwdSpeed);
            _slip = MeasureFrontSlip(fwdSpeed);

            if (_slip.Readable && _slip.FrontPct > 0.01f)
            {

                float wheelPct = _slip.FrontPct * AllowedSlipTrim(fwdSpeed);
                float factor = ARS.Clamp(100f / wheelPct, 1f - ARS.SlipCutDepthPct * 0.01f, 1f);

                float clampTarget = wheelPct > 100f
                    ? Math.Abs(requestedSteer) * factor
                    : VehicleData.SteeringLock;

                float chase = float.IsNaN(_chasingClampDeg) ? clampTarget : _chasingClampDeg;

                if (clampTarget >= chase)
                {
                    _chasingClampDeg = Math.Min(clampTarget, chase + SlipOpenDegPerSec * TickScale);
                }
                else
                {
                    float gap = chase - clampTarget;
                    float step = Math.Min(gap * SlipClosePerSecond, SlipCloseMaxDegPerSec) * TickScale;

                    _chasingClampDeg = Math.Max(clampTarget, chase - step);
                }
                speedCeiling = Math.Min(speedCeiling, _chasingClampDeg);
            }
            float steerLimitRight = speedCeiling;
            float steerLimitLeft = speedCeiling;

            if (countersteering && _countersteerDemand01 > 0f)
            {
                float countersteerAllowance = Math.Min(Math.Abs(VehicleData.SlideAngle) * _countersteerDemand01, VehicleData.SteeringLock);
                if (requestedSteer > 0f) steerLimitLeft = Math.Max(steerLimitLeft, countersteerAllowance);
                else steerLimitRight = Math.Max(steerLimitRight, countersteerAllowance);
            }

            float yawRate = VehicleData.YawRotationPerSecondDegrees;

            if (countersteering && _damperTermDeg * yawRate < 0f)
            {
                if (requestedSteer > 0f) steerLimitLeft = VehicleData.SteeringLock;
                else if (requestedSteer < 0f) steerLimitRight = VehicleData.SteeringLock;
            }

            if (ARS.WheelsOffGround(Car) >= AirborneSteerMinWheelsOff)
            {
                float airLimit = Math.Abs(VehicleData.SlideAngle) * AirborneSteerShareOfSlide;
                steerLimitRight = Math.Min(steerLimitRight, airLimit);
                steerLimitLeft = Math.Min(steerLimitLeft, airLimit);
            }

            Control.SteerDegrees = ARS.Clamp(requestedSteer, -steerLimitRight, steerLimitLeft);
            _steerPreLimitDegrees = requestedSteer;
            _steerGrantedCeiling = requestedSteer >= 0f ? steerLimitLeft : steerLimitRight;
        }

        bool IsFullCountersteer()
        {
            if (KickWantsSlide()) return false;
            if (Vector3.Dot(Car.Velocity, Car.ForwardVector) < ARS.MphToMps(CountersteerMinSpeedMph)) return false;
            return _countersteerDemand01 >= 1f;
        }

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
            _defendLaneOffset = 0f;
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
            _chasingClampDeg = float.NaN;

            float restHeight = Car.HeightAboveGround;
            if (restHeight > 0f) _restHeightAboveGround = restHeight;
            Control.HandBrakeUntilMs = 0;
            Control.MaxThrottle = 1f;
            Control.MaxBrake = 1f;
            Control.MaxBrakeFromABS = 1f;
            Control.MaxBrakeFromCountersteer = 1f;
            Control.MaxBrakeFromSlide = 1f;
            Control.MaxThrottleFromTCS = 1f;
            Control.MaxThrottleFromStability = 1f;
            Control.MaxThrottleFromOverspeed = 1f;
            Control.MaxThrottleFromRival = 1f;
            Control.MaxThrottleFromChillOut = 1f;
            Control.MaxThrottleFromYield = 1f;
            Control.MaxThrottleFromOffTrack = 1f;
            Control.MaxThrottleFromRecovery = 1f;
            Control.ThrottleReason = ThrottleReason.Plan;

            _avoidHoldActive = false;
            _avoidHoldLane = 0f;
            _avoidHoldUntil = 0;

            Control.ThrottleReasonCap = 1f;
            Control.BrakeReason = BrakeReason.Plan;
            Control.BrakeReasonCap = 1f;
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
            if (TeamRole == Team.Cop) Car.SirenActive = true;

        }
        const float ThrottleLogBelow = 0.9f;
        const float PedalSlewRate = 6f;
        const float PedalSlewMaxPerTick = 0.5f;
        // The UI follower's exponential rate per second: 95% after about half a second.
        const float ThrottleSmoothRate = 6f;
        const float BrakeSmoothRate = 6f;
        // The route speed chases at half the gauges' rate: section switches take about a second, and the
        // target chain's 2 s anticipation carries the lag.
        const float RouteSmoothRate = 3f;
        const float TcsSmoothRate = 6f;
        const float StabSmoothRate = 6f;

        Color ProjectionPathColour(float gsFloor, bool insideRegime)
        {
            if (insideRegime)
            {
                float share = ARS.Clamp(gsFloor / GsLaneThrottleMaxFloor, 0f, 1f);
                int dim = (int)(255f * (1f - share));
                return Color.FromArgb(255, dim, 255, dim);
            }
            if (_projectionCutActive) return Color.Red;
            if (_projectionOutsideMeters > 0f) return Color.Yellow;
            return Color.Blue;
        }

        const float GsLaneFloorSlipLowShare = 0.5f;
        const float GsLaneFloorSlipOverShare = 1.5f;

        void UpdateExitRoom()
        {
            _exitRoomArmed = false;
            _gsLaneFloorShare01 = 0f;
            if (!ARS.GsLaneThrottleFloor && !WheelTrimUnderPower) return;

            if (_projectionOutsideMeters > 0f || _carOutsideMeters > 0f) return;
            float slipPeakDeg = LateralPeakAtSpeed(ARS.GetForwardSpeed(Car));
            if (!(slipPeakDeg > 0f)) return;
            float slipDeg = Math.Abs(VehicleData.SlideAngle);
            if (slipDeg < GsLaneFloorSlipLowShare * slipPeakDeg || slipDeg > GsLaneFloorSlipOverShare * slipPeakDeg) return;

            Vector3 projected = ProjectAhead(SteerPreviewSeconds);
            int projectedWindow = (int)(Car.Velocity.Length() * SteerPreviewSeconds) + 10;
            TrackPoint projectedNode = ARS.FindNearestTrackPoint(projected, CurrentTrackPoint.Node, 0, projectedWindow);
            float carHalfWidth = VehicleData.ModelDimensions.X * 0.5f;
            float laneHere = CurrentTrackPoint.LaneLineShare * (CurrentTrackPoint.TrackHalfWidth - carHalfWidth);
            float laneAhead = projectedNode.LaneLineShare * (projectedNode.TrackHalfWidth - carHalfWidth);
            float laneVsLane = laneAhead - laneHere;

            float floor01 = ARS.Clamp((laneVsLane + GsLaneFloorSpreadMeters) / (2f * GsLaneFloorSpreadMeters), 0f, 1f);
            _exitRoomArmed = floor01 > 0f;
            _gsLaneFloorShare01 = floor01;
        }

        float GsLaneThrottleFloorValue(out bool insideRegime)
        {
            insideRegime = TrackSpeed.GsLaneFloor && ARS.GsLaneThrottleFloor && _exitRoomArmed;
            if (!insideRegime) return 0f;
            return GsLaneOutsideFloor + (GsLaneThrottleMaxFloor - GsLaneOutsideFloor) * _gsLaneFloorShare01;
        }

        void ConvertSpeedToPedals()
        {
            float currentForwardSpeed = VehicleData.SpeedVectorLocal.Y;
            float inputChange = Math.Min(PedalSlewRate * TickScale, PedalSlewMaxPerTick);
            float newThrottle = 0f;
            float newBrake = 0f;

            ClampIntendedSpeedToCeiling();
            ApplyProjectionSpeedCut();

            float intendedSpeedChange = Brain.CurrentIntention.Speed - currentForwardSpeed;
            Brain.CurrentIntention.IntendedSpeedChange = intendedSpeedChange;
            _thrDelta = intendedSpeedChange;
            _thrTarget = Brain.CurrentIntention.Speed;
            _thrActualMps = currentForwardSpeed;
            _thrTerm = Brain.CurrentIntention.Term;

            float combinedInput = ComputeCombinedInput(intendedSpeedChange);

            combinedInput = Math.Min(combinedInput, 1f - _projectionLift);

            if (SlipPedalCut && _slip.Readable && _slip.FrontPct > 100f)
            {
                float over = ARS.Clamp((_slip.FrontPct - 100f) / 150f, 0f, 1f);
                combinedInput = Math.Min(combinedInput, Math.Max(0f, 1f - over));
            }
            bool countersteering = IsFullCountersteer();

            UpdateExitRoom();
            float gsFloor = GsLaneThrottleFloorValue(out bool gsInsideRegime);
            if (countersteering) gsFloor = 0f;

            SplitCombinedInput(combinedInput, ref newThrottle, ref newBrake);
            if (countersteering)
            {
                newThrottle = Math.Min(newThrottle, CountersteerPedalCap);
                newBrake = Math.Min(newBrake, CountersteerPedalCap);
            }

            newThrottle = ComposeThrottleCap(newThrottle, countersteering, gsFloor);
            _thrWanted = newThrottle;

            newBrake = ComposeBrakeCap(newBrake, countersteering);
            _brkWanted = newBrake;

            _thrSlew = Math.Abs(newThrottle - Control.Throttle) > inputChange * 0.99f;
            Control.Brake += ARS.Clamp(newBrake - Control.Brake, -inputChange, inputChange);
            Control.Throttle += ARS.Clamp(newThrottle - Control.Throttle, -inputChange, inputChange);

            _thrFloor = _gsFloorApplied;
            _thrFinal = Control.Throttle;
            _thrSmooth.Target = _thrFinal;
            _thrSmooth.Advance(TickScale);
            _brkFinal = Control.Brake;
            _brkSmooth.Target = _brkFinal;
            _brkSmooth.Advance(TickScale);
            _projectionPathColor = ProjectionPathColour(_gsFloorApplied, gsInsideRegime);

            UpdateBrakeRampAnalysis();
            UpdateCornerSpeedLearning();
            UpdateRouteSpeedLearning();

        }

        float ComputeCombinedInput(float intendedSpeedChange)
        {
            return PedalFromSpeedError(intendedSpeedChange);
        }

        public static float PedalSpeedScale(float speedErrorMps) => speedErrorMps >= 0f ? FullThrottleSpeedErrorMps : FullBrakeSpeedErrorMps;

        public static float PedalFromSpeedError(float speedErrorMps) => ARS.Clamp(speedErrorMps / PedalSpeedScale(speedErrorMps), -1f, 1f);

        void SplitCombinedInput(float combinedInput, ref float newThrottle, ref float newBrake)
        {
            if (combinedInput > 0f)
            {
                newThrottle = combinedInput;
                return;
            }

            if (combinedInput < 0f) newBrake = -combinedInput;
        }

        void ClampIntendedSpeedToCeiling()
        {
            if (Brain.CurrentIntention.Speed >= 0f)
            {
                Brain.CurrentIntention.Speed = Math.Min(Brain.CurrentIntention.Speed, AiConstants.MaxIntendedSpeedMps);
            }
        }

        void ApplyProjectionSpeedCut()
        {
            MeasureProjectionOutside(1f);
            if (!TrackSpeed.ProjectionCut)
            {
                _projectionCutActive = false;
                _projectionDropMph = 0f;
                return;
            }
            float projectionSlide = Math.Abs(VehicleData.SlideAngle);
            if (float.IsNaN(projectionSlide) || float.IsInfinity(projectionSlide)) projectionSlide = 0f;
            bool steeringIntoSlide = Math.Abs(SteerAskDegrees) > 0.01f
                && Math.Abs(VehicleData.YawRotationPerSecondDegrees) > 0.01f
                && Math.Sign(SteerAskDegrees) == Math.Sign(VehicleData.YawRotationPerSecondDegrees);
            bool projectionSliding = TRLateralAtSpeed > 0.01f
                && projectionSlide >= TRLateralAtSpeed * ProjectionArmSlideShare
                && steeringIntoSlide;
            bool projectionMovingAhead = ARS.MpsToMph(Car.Velocity.Length()) >= ProjectionMinSpeedMph
                && ARS.GetForwardSpeed(Car) > 0f;
            _projectionArmed = projectionSliding && projectionMovingAhead;
            _projectionCutActive = _projectionArmed && _projectionOutsideMeters > 0f;

            float projectionTargetLift = _projectionCutActive
                ? Math.Min(_projectionOutsideMeters / ProjectionFullLiftMetres, 1f) * ProjectionMaxLift
                : 0f;

            float projectionStep = ProjectionLiftPerSecond * TickScale;
            _projectionLift = ARS.Clamp(projectionTargetLift, _projectionLift - projectionStep, _projectionLift + projectionStep);

            float projectionBrakeShare = Math.Max(0f, _projectionLift - 1f);
            _projectionDropMph = ARS.MpsToMph(projectionBrakeShare * FullBrakeSpeedErrorMps);
        }

        void MeasureProjectionOutside(float seconds)
        {
            bool laneOverride = _laneOverride;
            float mph = ARS.MpsToMph(Car.Velocity.Length());
            float signedOffset = 0f;
            _projectionOutsideMeters = 0f;
            _projectionCenterOffset = 0f;
            _projectionHalfWidth = 0.1f;
            _projectionMeasuredAt = Game.GameTime;
            _carOutsideMeters = 0f;
            _projectionWidening = false;
            _projectionWidenMeters = 0f;
            _projectionWidenSign = 0;
            float halfWidth = 0f;
            float outsideOffset = 0f;
            float carOffset = 0f;
            float turnDirection = 0f;

            if (mph >= ProjectionMinSpeedMph)
            {
                Vector3 proj = ProjectAhead(seconds);

                int projectionWindow = (int)(Car.Velocity.Length() * 2f);
                TrackPoint tp = ARS.FindNearestTrackPoint(proj, CurrentTrackPoint.Node, 0, projectionWindow);
                signedOffset = ARS.SignedLaneOffset(proj, tp.Position, tp.Direction);
                halfWidth = Math.Max(tp.TrackHalfWidth, 0.1f);

                turnDirection = Math.Sign(signedOffset);
                outsideOffset = Math.Abs(signedOffset);
                carOffset = Math.Abs(ARS.SignedLaneOffset(Car.Position, CurrentTrackPoint.Position, CurrentTrackPoint.Direction));
                _carOutsideMeters = carOffset - Math.Max(CurrentTrackPoint.TrackHalfWidth, 0.1f);
                _projectionOutsideMeters = outsideOffset - halfWidth;
                _projectionCenterOffset = signedOffset;
                _projectionHalfWidth = halfWidth;

                Vector3 widenProj = ProjectAhead(SteerPreviewSeconds);
                int widenWindow = (int)(Car.Velocity.Length() * SteerPreviewSeconds * 2f);
                TrackPoint widenNode = ARS.FindNearestTrackPoint(widenProj, CurrentTrackPoint.Node, 0, widenWindow);
                float widenSigned = ARS.SignedLaneOffset(widenProj, widenNode.Position, widenNode.Direction);
                _projectionWidenMeters = Math.Abs(widenSigned) - carOffset;
                _projectionWidenSign = Math.Sign(widenSigned);
                _projectionWidening = _projectionWidenMeters > 0f;
            }

            if (ARS.DebugToggles[Options.ShowGsProjection] && ARS.DebugFocusRacer == this && Game.GameTime >= _projectionLogAtMs)
            {
                _projectionLogAtMs = Game.GameTime + 500;
                ARS.Log(ARS.LogImportance.Info, "Offshoot: override=" + laneOverride + " mph=" + mph.ToString("0") + " turn=" + turnDirection + " signed=" + signedOffset.ToString("0.00") + " car=" + carOffset.ToString("0.00") + " half=" + halfWidth.ToString("0.00") + " outside=" + outsideOffset.ToString("0.00") + " drop=" + _projectionDropMph.ToString("0.0") + " floor=" + ProjectionSpeedFloorMph().ToString("0"));
            }

        }

        float ProjectionSpeedFloorMph()
        {
            return OffTrackSpeedFloorMph;
        }

        bool ProjectionClosingOnCentre()
        {
            Vector3 proj = ProjectAhead(1f);

            int projectionWindow = (int)(Car.Velocity.Length() * 2f);
            TrackPoint tp = ARS.FindNearestTrackPoint(proj, CurrentTrackPoint.Node, 0, projectionWindow);
            float carOffset = Math.Abs(ARS.SignedLaneOffset(Car.Position, CurrentTrackPoint.Position, CurrentTrackPoint.Direction));
            float projectionOffset = Math.Abs(ARS.SignedLaneOffset(proj, tp.Position, tp.Direction));
            return projectionOffset < carOffset;
        }

        string CornerId(int apexNode)
        {
            for (int i = 0; i < ARS.Corners.Count; i++)
                if (ARS.Corners[i].Node == apexNode) return "corner #" + (i + 1);
            return "corner " + apexNode;
        }

        void LogApexEvent(string source, bool chose, int apexNode, string reason)
        {
            string detail = string.IsNullOrEmpty(reason) ? string.Empty : " (" + reason + ")";
            ARS.Log(ARS.LogImportance.Info, source + (chose ? " chose apex " : " released apex ")
                + apexNode + " at " + Game.GameTime + " ms" + detail);
        }

        float PlannedCornerArrivalSpeed(CornerPoint corner)
        {
            // CornerApexSpeed already carries this corner's offset — the menu setting seeds it, the learner moves
            // it. Nothing is added here: the offset has one writer.
            float speed = ARS.CornerApexSpeed(corner, this);
            if (TrackSpeed.ChicaneBoost && Brain.Corner != null && Brain.Corner.Point.IsChicane)
                speed += ARS.MphToMps(10f);
            return speed;
        }

        void UpdateBrakeRampAnalysis()
        {
            if (!ARS.BrakeLearning) return;

            int apex = NextApexNode;
            if (_brakeAnalysisApexNode != apex || _brakeAnalysisLap != Lap)
            {
                _brakeAnalysisApexNode = apex;
                _brakeAnalysisLap = Lap;
                _brakeAnalysisArmed = false;
                _brakeAnalysisFired = false;
            }
            if (apex < 0 || _brakeAnalysisFired) return;

            CornerPoint corner = ARS.Corners.FirstOrDefault(c => c.Node == apex);
            int targetNode = BrakeTargetBaseNode(apex, CornerEntranceNode(corner, apex));
            if (targetNode >= 0)
            {
                float distance = SignedMetresToNode(targetNode);
                if (distance > 0f)
                {
                    _brakeAnalysisArmed = true;
                    return;
                }
            }

            if (!_brakeAnalysisArmed) return;
            _brakeAnalysisFired = true;
            if (!CornerRequiresBraking(apex)) return;

            float targetSpeed = PlannedCornerArrivalSpeed(corner);
            float delta = ARS.GetForwardSpeed(Car) - targetSpeed;
            CornerContext context = CornerContextForWrite(apex);
            float before = context.BrakeFactor;
            float after = ARS.Clamp(before + (BrakeRampIdealDeltaMps - delta) / BrakeRampErrorDivisor,
                BrakeFactorMin, BrakeFactorMax);
            context.BrakeFactor = after;
            string bufferText = string.Empty;
            if (after > before + 0.0005f && context.ApexSpeedBuffer > BrakeBufferMinNodes)
            {
                context.ApexSpeedBuffer = ARS.Clamp(context.ApexSpeedBuffer - BrakeBufferStepNodes,
                    BrakeBufferMinNodes, BrakeBufferMaxNodes);
                bufferText = " · buffer -> " + Math.Round(context.ApexSpeedBuffer).ToString(CultureInfo.InvariantCulture) + " nodes";
            }
            string deltaText = (delta >= 0f ? "+" : string.Empty) + delta.ToString("0.0", CultureInfo.InvariantCulture);
            string factorText = Math.Abs(after - before) < 0.0005f
                ? "factor " + after.ToString("0.00", CultureInfo.InvariantCulture) + " holds"
                : "factor " + before.ToString("0.00", CultureInfo.InvariantCulture) + " -> " + after.ToString("0.00", CultureInfo.InvariantCulture);

            string brakeReason = deltaText + " m/s";
            if (delta > 0f) brakeReason = "~r~" + brakeReason + "~w~";
            AnnounceBrakeRamp(CornerId(apex) + " · " + deltaText + " m/s vs target · " + factorText + bufferText,
                brakeReason + " - " + after.ToString("0.00", CultureInfo.InvariantCulture));
        }

        void AnnounceBrakeRamp(string detail, string notified)
        {
            ARS.Log(ARS.LogImportance.Info, "brake ramp " + Name + " (" + Car.DisplayName + "): " + detail);
            if (!ARS.BrakeLearningNotify) return;
            UI.Notify("~b~[BRAKE]~w~ " + Car.DisplayName + "~n~" + notified);
        }

        static float CornerOutsideSign(CornerPoint corner)
        {
            float angle = corner.Angle;
            if (Math.Sign(angle) == 0f && corner.StartNode >= 0 && corner.EndNode >= 0)
            {
                Vector3 entry = ARS.TrackPoints[corner.StartNode].Direction;
                Vector3 exit = ARS.TrackPoints[corner.EndNode].Direction;
                angle = Vector3.SignedAngle(entry, exit, Vector3.WorldUp);
            }
            return -ARS.InsideSign(angle);
        }

        void UpdateCornerSpeedLearning()
        {
            if (!ARS.CornerSpeedLearning) return;

            ReleaseCornerSpeedLocks();

            if (_cornerSpeedApexNode < 0) return;

            if (!_cornerSpeedHitOffTrack && MsSinceContact() < CornerSpeedLearnHitOffTrackMs && SignedMetresBeyondTrackEdge() > 0f)
                _cornerSpeedHitOffTrack = true;
            if (!HasPassedApex(_cornerSpeedApexNode)) return;
            if (VehicleData.LastAccelSampleTime == _cornerSpeedLastSampleMs || TRLateralAtSpeed <= 0.01f) return;

            if (!TryGetCornerContext(_cornerSpeedApexNode, out CornerContext context) || context.Point == null) return;
            float outsideSign = CornerOutsideSign(context.Point);
            if (outsideSign == 0f) return;

            _cornerSpeedLastSampleMs = VehicleData.LastAccelSampleTime;
            float slide = Math.Abs(VehicleData.SlideAngle);
            if (float.IsNaN(slide) || float.IsInfinity(slide)) return;
            if (slide > _cornerSpeedWorstSlideDeg) _cornerSpeedWorstSlideDeg = slide;
            if (slide / TRLateralAtSpeed < CornerSpeedLearnMinSlideShare) return;

            float signedOffset = ARS.SignedLaneOffset(Car.Position, CurrentTrackPoint.Position, CurrentTrackPoint.Direction);
            float outsideReach = signedOffset * outsideSign + VehicleData.SlidingWidthMetres * 0.5f;
            float clearance = Math.Max(CurrentTrackPoint.TrackHalfWidth, 0.1f) - outsideReach;
            _cornerSpeedMinOutsideClearanceMetres = Math.Min(_cornerSpeedMinOutsideClearanceMetres, clearance);
            _cornerSpeedOutsideSamples++;
        }

        void StartCornerSpeedLearning(int apexNode)
        {
            if (apexNode < 0 || _cornerSpeedLocked.Contains(apexNode))
            {
                _cornerSpeedApexNode = -1;
                return;
            }
            _cornerSpeedApexNode = apexNode;
            _cornerSpeedLastSampleMs = -1;
            _cornerSpeedMinOutsideClearanceMetres = float.MaxValue;
            _cornerSpeedOutsideSamples = 0;
            _cornerSpeedWorstSlideDeg = 0f;
            _cornerSpeedHitOffTrack = false;
        }

        void CommitCornerSpeedLearningFor(int apexNode)
        {
            if (_cornerSpeedApexNode != apexNode) return;
            CommitCornerSpeedLearning();
        }

        void CommitCornerSpeedLearning()
        {
            int apexNode = _cornerSpeedApexNode;
            _cornerSpeedApexNode = -1;
            if (apexNode < 0) return;

            if (MsSinceContact() < CornerSpeedLearnQuietContactMs)
            {

                if (_cornerSpeedHitOffTrack)
                {
                    float hitBefore = TryGetCornerContext(apexNode, out CornerContext hitContext) ? hitContext.CornerSpeedOffsetMps : 0f;
                    float hitAfter = ARS.Clamp(hitBefore - CornerSpeedLearnDeltaLimitMps,
                        -CornerSpeedLearnLimitMps, CornerSpeedLearnLimitMps);
                    CornerContextForWrite(apexNode).CornerSpeedOffsetMps = hitAfter;
                    float hitApplied = hitAfter - hitBefore;
                    string hitDelta = (hitApplied >= 0f ? "+" : "")
                        + hitApplied.ToString("0.00", CultureInfo.InvariantCulture) + " m/s";
                    AnnounceCornerSpeedAdjustment(CornerId(apexNode) + " · hit off track, " + MsSinceContact()
                        + " ms ago · " + hitDelta, "~r~hit off track~w~ - " + hitDelta);
                    return;
                }

                AnnounceCornerSpeedAdjustment(CornerId(apexNode) + " · voided, contact " + MsSinceContact() + " ms ago",
                    "hit - no change");
                return;
            }

            _cornerSpeedLocked.Add(apexNode);

            float before = TryGetCornerContext(apexNode, out CornerContext context) ? context.CornerSpeedOffsetMps : 0f;
            string id = CornerId(apexNode);
            bool hadSlipSample = _cornerSpeedOutsideSamples > 0;
            float clearance = hadSlipSample ? _cornerSpeedMinOutsideClearanceMetres : float.NaN;

            float verdictClearance = float.NaN;
            float delta = CornerSpeedLearnNoSlideRiseMps;
            if (hadSlipSample)
            {
                if (clearance > CornerSpeedLearnVerdictMarginMetres)
                    verdictClearance = clearance - CornerSpeedLearnVerdictMarginMetres;
                else if (clearance < 0f)
                    verdictClearance = clearance;
                else
                    return;
                delta = verdictClearance * CornerSpeedLearnMetreToOffset;
            }

            delta = ARS.Clamp(delta, -CornerSpeedLearnDeltaLimitMps, CornerSpeedLearnDeltaLimitMps);

            bool lostControl = _cornerSpeedWorstSlideDeg > CornerSpeedLearnLostControlDegrees && delta < 0f;
            if (lostControl) delta = 0f;
            float after = ARS.Clamp(before + delta, -CornerSpeedLearnLimitMps, CornerSpeedLearnLimitMps);

            CornerContextForWrite(apexNode).CornerSpeedOffsetMps = after;
            float applied = after - before;

            string deltaText = (applied >= 0f ? "+" : "") + applied.ToString("0.00", CultureInfo.InvariantCulture) + " m/s";
            string peakText = hadSlipSample
                ? "closest " + clearance.ToString("0.0", CultureInfo.InvariantCulture) + " m"
                : "no slide";
            string offtrack = hadSlipSample && clearance < 0f ? " · ~r~offtrack~w~" : "";

            string reasonText = hadSlipSample ? clearance.ToString("0.0", CultureInfo.InvariantCulture) + " m" : "no slide";
            if (lostControl)
                reasonText = "lost control " + _cornerSpeedWorstSlideDeg.ToString("0", CultureInfo.InvariantCulture) + "°";
            string verdictText = hadSlipSample
                ? " · verdict " + verdictClearance.ToString("0.0", CultureInfo.InvariantCulture) + " m"
                : "";

            if (applied < 0f) reasonText = "~r~" + reasonText + "~w~";
            AnnounceCornerSpeedAdjustment(id + " · " + peakText + verdictText + " · " + deltaText + offtrack
                + (lostControl ? " · lost control, deduction voided" : ""),
                reasonText + " - " + deltaText);
        }

        CornerPoint NearestCornerAhead()
        {
            CornerPoint best = null;
            float bestMetres = float.MaxValue;
            foreach (CornerPoint corner in ARS.Corners)
            {
                float metres = SignedMetresToNode(corner.Node);
                if (metres < -ApexReachedMetres * 2f) continue;
                if (best != null && metres >= bestMetres) continue;
                best = corner;
                bestMetres = metres;
            }
            return best;
        }

        void EnterCornerMode(int apexNode, float heldSpeed, float cornerTerm, float routeTerm)
        {
            if (apexNode < 0) return;

            if (SignedMetresToNode(apexNode) < 0f) return;
            if (_cornerMode && _cornerModeNode >= 0 && _cornerModeNode != apexNode)
                CommitCornerSpeedLearningFor(_cornerModeNode);
            ARS.Log(ARS.LogImportance.Info, "mode " + Name + ": pre  route " + routeTerm.ToString("0.0", CultureInfo.InvariantCulture)
                + " corner " + cornerTerm.ToString("0.0", CultureInfo.InvariantCulture)
                + " car " + Car.Velocity.Length().ToString("0.0", CultureInfo.InvariantCulture)
                + " apex at " + SignedMetresToNode(apexNode).ToString("0.0", CultureInfo.InvariantCulture) + " m");
            _cornerMode = true;
            _cornerSpeedAuthority = true;
            _cornerModeNode = apexNode;
            _cornerModeSpeed = heldSpeed;
            _cornerModeApexPassedAt = -1;
            _cornerModeDistanceNodes = 0;
            _cornerModeLastNode = CurrentTrackPoint.Node;
            _cornerModeApexTargetNodes = ForwardNodeDistance(apexNode);
            if (ARS.IsPointToPoint && _cornerModeApexTargetNodes < 0) _cornerModeApexTargetNodes = 0;
            StartCornerSpeedLearning(apexNode);
            ARS.Log(ARS.LogImportance.Info, "mode " + Name + ": ENTER corner #" + apexNode + ", drinking "
                + cornerTerm.ToString("0.0", CultureInfo.InvariantCulture) + " m/s, held "
                + heldSpeed.ToString("0.0", CultureInfo.InvariantCulture) + " m/s");
        }

        // `judge: false` is the early release: the corner stopped asking for a lift before its apex.
        void ExitCornerMode(float routeTerm, string why, bool judge = true)
        {
            if (!_cornerMode) return;
            if (judge) CommitCornerSpeedLearningFor(_cornerModeNode);
            else _cornerSpeedApexNode = -1;
            int postApexMs = _cornerModeApexPassedAt >= 0 ? Game.GameTime - _cornerModeApexPassedAt : -1;
            ARS.Log(ARS.LogImportance.Info, "mode " + Name + ": corner mode lifted for #" + _cornerModeNode
                + (postApexMs >= 0 ? " " + postApexMs + " ms after apex" : " before apex was marked"));
            ARS.Log(ARS.LogImportance.Info, "mode " + Name + ": post route " + routeTerm.ToString("0.0", CultureInfo.InvariantCulture)
                + " car " + Car.Velocity.Length().ToString("0.0", CultureInfo.InvariantCulture)
                + " apex at " + SignedMetresToNode(_cornerModeNode).ToString("0.0", CultureInfo.InvariantCulture) + " m");
            _cornerMode = false;
            _cornerSpeedAuthority = false;
            ARS.Log(ARS.LogImportance.Info, "mode " + Name + ": EXIT to route (" + why + ")");
            _cornerModeNode = -1;
            _cornerModeSpeed = 999f;
            _cornerModeApexPassedAt = -1;
            _cornerModeDistanceNodes = 0;
            _cornerModeApexTargetNodes = 0;
            _cornerModeLastNode = -1;
        }

        void ReleaseCornerSpeedLocks()
        {
            if (_cornerSpeedLocked.Count == 0) return;
            List<int> released = null;
            foreach (int node in _cornerSpeedLocked)
            {
                if (SignedMetresToNode(node) >= -CornerSpeedLearnReleaseMetres) continue;
                if (released == null) released = new List<int>();
                released.Add(node);
            }
            if (released == null) return;
            foreach (int node in released) _cornerSpeedLocked.Remove(node);
        }

        void AnnounceCornerSpeedAdjustment(string detail, string notified)
        {
            ARS.Log(ARS.LogImportance.Info, "corner speed " + Name + " (" + Car.DisplayName + "): " + detail);
            if (!ARS.CornerSpeedLearnNotify) return;
            UI.Notify("~b~[CORNER SPEED]~w~ " + Car.DisplayName + "~n~" + notified);
        }

        int CornerEntranceNode(CornerPoint corner, int apexNode)
        {
            int apex = corner != null && corner.Node >= 0 ? corner.Node : apexNode;
            if (apex >= 0 && apex < ARS.TrackPoints.Count)
            {
                int shaped = ARS.TrackPoints[apex].LaneShapeStart;
                if (shaped >= 0) return shaped;
            }
            if (corner == null) return apexNode;
            return corner.StartNode >= 0 ? corner.StartNode : OffsetCornerNode(apexNode, -corner.EntranceNodes);
        }

        int CornerExitNode(CornerPoint corner)
        {
            if (corner == null || corner.Node < 0) return -1;
            return corner.EndNode >= 0 ? corner.EndNode : OffsetCornerNode(corner.Node, corner.ExitNodes);
        }

        // The ramp's aim node, in nodes behind the apex: the corner's own buffer, so the ramp and the learner that
        // grades it always read the same node — and the learner can move it once its brake factor has settled.
        int BrakeAimNode(int apexNode)
        {
            int count = ARS.TrackPoints.Count;
            if (apexNode < 0 || apexNode >= count || count == 0) return -1;
            float buffer = TryGetCornerContext(apexNode, out CornerContext context)
                ? context.ApexSpeedBuffer
                : CornerContext.ApexSpeedBufferSeed;
            int back = (int)Math.Round(buffer);
            if (back < 1) back = 1;
            if (back >= count) back = count - 1;
            return ARS.IsPointToPoint
                ? Math.Max(0, apexNode - back)
                : ((apexNode - back) % count + count) % count;
        }

        int BrakeTargetBaseNode(int apexNode, int fallbackNode)
        {
            int aimNode = BrakeAimNode(apexNode);
            return aimNode < 0 ? fallbackNode : aimNode;
        }

        bool HasPassedBrakingTarget()
        {
            if (NextApexNode < 0) return true;
            CornerPoint corner = _nextApexCorner;
            int targetNode = BrakeTargetBaseNode(NextApexNode, CornerEntranceNode(corner, NextApexNode));
            if (targetNode < 0) return true;
            int targetDistance = ForwardNodeDistance(targetNode);
            int apexDistance = ForwardNodeDistance(NextApexNode);
            return targetDistance <= 0 || (!ARS.IsPointToPoint && targetDistance > apexDistance);
        }

        float TickScale => (0.001f * TimeSinceLastCoreTick);

        void TranslateSteerToInput()
        {

            if (float.IsNaN(Control.SteerDegrees) || float.IsInfinity(Control.SteerDegrees)) Control.SteerDegrees = 0f;

            float error = Control.SteerDegrees - Control.LastAppliedSteerDegrees;

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
                Brain.CurrentIntention.Term = SpeedTerm.GridWait;
                return;
            }
            if (BaseBehavior == RacerBaseBehavior.FinishedRace)
            {
                Brain.CurrentIntention.Speed = 20f;
                Brain.CurrentIntention.Term = SpeedTerm.Finished;
                return;
            }
            if (BaseBehavior == RacerBaseBehavior.FinishedStandStill)
            {
                Brain.CurrentIntention.Speed = 0f;
                Brain.CurrentIntention.Term = SpeedTerm.Finished;
                return;
            }
            Brain.CurrentIntention.Speed = AiConstants.MaxIntendedSpeedMps;
            Brain.CurrentIntention.Term = SpeedTerm.None;

            float cornerSpd = 999f;
            float cornerArrival = 999f;
            int cornerSourceNode = -1;
            CornerPoint lastLapCorner = LastCornerOfLap();
            bool lastLapCornerPassed = lastLapCorner == null || HasPassedApex(lastLapCorner.Node);
            if (TrackSpeed.CornerPlan && NextApexNode >= 0)
            {

                if (NextApexNode >= 0 && !CornerBeyondLastLapCorner(NextApexNode, lastLapCorner, lastLapCornerPassed) && CornerRequiresBraking(NextApexNode))
                {
                    cornerSpd = Math.Max(2, ApexBrakingSpeed(NextApexNode, NextApexSpeed));
                    cornerSourceNode = NextApexNode;
                    cornerArrival = NextApexSpeed;
                }
                if (NextApexNode2 >= 0 && !CornerBeyondLastLapCorner(NextApexNode2, lastLapCorner, lastLapCornerPassed) && CornerRequiresBraking(NextApexNode2))
                {
                    float second = Math.Max(2, ApexBrakingSpeed(NextApexNode2, NextApexSpeed2));
                    if (second < cornerSpd) { cornerSpd = second; cornerSourceNode = NextApexNode2; cornerArrival = NextApexSpeed2; }
                }
                if (NextApexNode3 >= 0 && !CornerBeyondLastLapCorner(NextApexNode3, lastLapCorner, lastLapCornerPassed) && CornerRequiresBraking(NextApexNode3))
                {
                    float third = Math.Max(2, ApexBrakingSpeed(NextApexNode3, NextApexSpeed3));
                    if (third < cornerSpd) { cornerSpd = third; cornerSourceNode = NextApexNode3; cornerArrival = NextApexSpeed3; }
                }
            }

            if (cornerSpd >= 998f)
            {
                CornerPoint owned = NextApexNode >= 0 && !CornerBeyondLastLapCorner(NextApexNode, lastLapCorner, lastLapCornerPassed)
                    ? ARS.Corners.FirstOrDefault(corner => corner.Node == NextApexNode)
                    : null;
                if (owned == null) owned = NearestCornerAhead();
                if (owned != null)
                {
                    cornerArrival = ARS.CornerApexSpeed(owned, this);
                    cornerSpd = Math.Max(2f, ApexBrakingSpeed(owned.Node, cornerArrival));
                    cornerSourceNode = owned.Node;
                }
            }

            float followTrackSpd = TrackSpeed.RouteCurvature
                ? RouteIdealSpeedForRadius(Brain.CurrentPerception.CurveRadiusToFollowPoint) * (float)Math.Sqrt(RouteSurfaceGrip())
                : 999f;

            if (float.IsNaN(cornerSpd) || float.IsInfinity(cornerSpd)) cornerSpd = 999f;
            if (float.IsNaN(followTrackSpd) || float.IsInfinity(followTrackSpd)) followTrackSpd = 999f;
            if (cornerSpd <= 5 && Brain.Corner != null)
            {
                cornerSpd = ARS.CornerApexSpeed(Brain.Corner.Point, this);
                cornerArrival = cornerSpd;
            }

            bool chicane = TrackSpeed.ChicaneBoost && Brain.Corner != null && Brain.Corner.Point.IsChicane;
            if (chicane)
            {
                cornerSpd += ARS.MphToMps(10f);
                cornerArrival += ARS.MphToMps(10f);
                Brain.CurrentIntention.Term = SpeedTerm.Chicane;
            }

            if (TrackSpeed.SlopeGrip)
            {
                float slopeAngleDeg = ARS.RadToDeg(GetFollowPointSlopeAngle());
                float slopeGripFactor = ARS.HillGripFactorFromPitchAngle(slopeAngleDeg, this);
                followTrackSpd *= (float)Math.Sqrt(slopeGripFactor);
            }

            int count = ARS.TrackPoints.Count;
            int carNode = CurrentTrackPoint.Node;
            float routeCrestFactor = CrestGripSpeedFactor(carNode, count, out int routeNear, out int routeFar, out _);

            LiveRouteCrestNode = routeNear;
            LiveRouteCrestFarNode = routeFar;
            followTrackSpd *= routeCrestFactor;

            if (chicane) followTrackSpd += ARS.MphToMps(10f);

            bool steerLimitedSpeedGovernsRoute = false;
            float steerRad = Math.Abs(Control.SteerDegrees) * (float)Math.PI / 180f;
            if (steerRad > 0.001f)
            {
                float turnRadius = VehicleData.WheelBaseMetres / (float)Math.Tan(steerRad);
                Brain.CurrentIntention.SteerLimitedSpeed = (float)Math.Sqrt(Handling.Gravity * VehicleData.CurrentMechanicalGrip * Math.Max(turnRadius, 1f));
                if (steerLimitedSpeedGovernsRoute) followTrackSpd = 0.7f * Brain.CurrentIntention.SteerLimitedSpeed + 0.3f * followTrackSpd;
            }
            else
            {
                Brain.CurrentIntention.SteerLimitedSpeed = 999f;
            }

            // Route is the default authority, a corner an override: it latches only lower and owns the speed to
            // that corner's apex, where it ramps back into the live route term. Each term already carries its own
            // offset (corner: the seeded learner value, route: _routeSpeedOffsetMph) — no offset is added here.
            float cornerTerm = cornerSpd;
            float cornerArrivalTerm = cornerArrival;

            // Route Speed Points on: the seeded per-section target IS the route term; off: the old term, untouched.
            float routeTerm = ARS.RouteSpeedPoints
                ? RouteSpeedPointTargetSpeedMps(followTrackSpd + ARS.MphToMps(_routeSpeedOffsetMph))
                : followTrackSpd + ARS.MphToMps(_routeSpeedOffsetMph);
            // The respected speed rides the pedal smoother toward the section target: switches taper instead
            // of stepping. First service snaps (no ramp from the 0 seed); off leaves the old term untouched.
            if (ARS.RouteSpeedPoints)
            {
                _routeSmooth.Target = routeTerm;
                if (_routeSmooth.Value <= 0f) _routeSmooth.Value = routeTerm;
                _routeSmooth.Advance(TickScale);
                routeTerm = _routeSmooth.Value;
            }

            if (_cornerMode)
            {
                if (_cornerModeApexPassedAt < 0 && _cornerModeDistanceNodes >= _cornerModeApexTargetNodes)
                {
                    _cornerModeApexPassedAt = Game.GameTime;
                    _cornerSpeedAuthority = false;
                    // The corner's held speed becomes the respected speed's own starting value, and the smoother —
                    // already chasing the next speed point — climbs from it at its own rate. No blend, no clock, and
                    // no step in either direction: a target below the held speed descends just as smoothly.
                    // Speed points off means there is no smoother to seed (it only advances under that flag), so the
                    // corner releases straight onto the curvature term and the handover is the step it always was.
                    if (ARS.RouteSpeedPoints)
                    {
                        _routeSmooth.Value = _cornerModeSpeed;
                        routeTerm = _routeSmooth.Value;
                    }
                    ARS.Log(ARS.LogImportance.Info, "mode " + Name + ": APEX corner #" + _cornerModeNode
                        + " passed, car " + Car.Velocity.Length().ToString("0.0", CultureInfo.InvariantCulture)
                        + " m/s, held " + _cornerModeSpeed.ToString("0.0", CultureInfo.InvariantCulture)
                        + " m/s, " + (ARS.RouteSpeedPoints
                            ? "seeded route " + routeTerm.ToString("0.0", CultureInfo.InvariantCulture)
                              + " chasing " + _routeSmooth.Target.ToString("0.0", CultureInfo.InvariantCulture)
                            : "route " + routeTerm.ToString("0.0", CultureInfo.InvariantCulture))
                        + " m/s");
                }

                if (_cornerModeNode < 0 || (_cornerModeApexPassedAt >= 0 && Game.GameTime - _cornerModeApexPassedAt >= CornerVerdictMs))
                {
                    ExitCornerMode(routeTerm, "verdict window closed");
                }
                else if (_cornerSpeedAuthority)
                {

                    float toApex = SignedMetresToNode(_cornerModeNode);
                    cornerArrival = _cornerModeSpeed;
                    cornerSpd = toApex < 0f
                        ? _cornerModeSpeed
                        : Math.Max(2f, ApexBrakingSpeed(_cornerModeNode, _cornerModeSpeed));
                    cornerSourceNode = _cornerModeNode;
                    cornerTerm = cornerSpd;
                    cornerArrivalTerm = cornerArrival;

                    if (cornerTerm - ARS.GetForwardSpeed(Car) > CornerDeselectMps)
                        ExitCornerMode(routeTerm, "car fell well under the corner", false);
                }
            }

            if (!_cornerSpeedAuthority && cornerSpd < 998f && cornerSourceNode >= 0
                && cornerSourceNode != _cornerModeNode && cornerSpd < ARS.GetForwardSpeed(Car))
                EnterCornerMode(cornerSourceNode, cornerArrival, cornerTerm, routeTerm);

            _brakePlanActive = _cornerMode || (NextApexNode >= 0 && CornerRequiresBraking(NextApexNode));
            _speedCorner = cornerTerm;
            _speedCornerArrival = cornerArrivalTerm;
            _speedRoute = routeTerm;
            Brain.CurrentIntention.Speed = _cornerSpeedAuthority ? cornerTerm : routeTerm;
            if (_cornerSpeedAuthority)
            {
                Brain.CurrentIntention.Term = SpeedTerm.Corner;
                Brain.CurrentIntention.TermNode = cornerSourceNode;
                Brain.CurrentIntention.TermMetres = cornerSourceNode >= 0 ? SignedMetresToNode(cornerSourceNode) : 0f;
                CornerPoint source = cornerSourceNode >= 0 ? ARS.Corners.Find(c => c.Node == cornerSourceNode) : null;
                Brain.CurrentIntention.TermRadius = source != null ? source.DetectedRadius : 0f;
            }
            else
            {
                Brain.CurrentIntention.Term = SpeedTerm.Route;
                Brain.CurrentIntention.TermNode = -1;
                Brain.CurrentIntention.TermMetres = 0f;
                Brain.CurrentIntention.TermRadius = Brain.CurrentPerception.CurveRadiusToFollowPoint;
            }

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
                    Brain.CurrentIntention.Term = SpeedTerm.Standoff;
                }
            }

            float rbFactor = ComputeRubberBandFactor();
            bool rbNearbyRival = Brain.Rivals.Any(r => r.RivalRacer != null && r.Distance < 100f);
            bool rbFreeZone = Lap <= 1 && CurrentTrackPoint != null && CurrentTrackPoint.Node < 500;

            bool burning = Game.GameTime < _nitrousActiveUntil;
            if (!burning && rbFactor < 1f && !rbFreeZone && !(ARS.CurrentRubberbandMode == RubberbandMode.Natural && rbNearbyRival))
                Brain.CurrentIntention.Speed *= rbFactor;
        }
        const float CrestNearSeconds = 1.0f;
        const float CrestFarSeconds = 2.0f;
        float CrestGripSpeedFactor(int carNode, int count, out int nearNode, out int farNode, out float rawDeltaGs)
        {
            nearNode = -1;
            farNode = -1;
            rawDeltaGs = 0f;
            if (!TrackSpeed.CrestGate) return 1f;
            if (carNode < 0 || carNode >= count) return 1f;
            float v = Car.Velocity.Length();
            if (v < 0.5f) return 1f;

            int near = TrackLoader.NodeAhead(carNode, CrestNearSeconds * v, count);
            int far = TrackLoader.NodeAhead(carNode, CrestFarSeconds * v, count);
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
        const float ElevationToSlopeSine = 1f / 90f;

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
        const float OffTrackTargetShare = 0.5f;
        const float ReasonCapSlewRate = 3.5f;
        const float TcsCapFloor = 0.2f;
        static readonly float TaperNormaliser = (float)(1.0 / Math.Tanh(1.0));

        static float TaperShare(float depth, float span)
        {
            return ARS.Clamp((float)Math.Tanh(depth / span) * TaperNormaliser, 0f, 1f);
        }
        public const float YieldThrottleLevel = 0.5f;

        const float OffTrackWideningThrottle = 0f;
        const float OffTrackSafeSpeedMph = 20f;
        const float OffTrackFullThrottleMph = 15f;
        const float OffTrackRecoverThrottle = 0.25f;
        const float StabilityFallPerWheelPerSecond = 0.5f;
        const float StabilityRisePerSecond = 1f;

        const float StabilityFloor = -0.5f;
        const float StabilityThrottleFloor = 0.1f;
        const float OverspeedArmMaxSlideDegrees = 2f;
        const float OverspeedArmMinSpeedMph = 30f;
        const float OverspeedArmMinForwardGs = 0.1f;
        const float OverspeedCutStartGs = 0.2f;
        const float ReasonCapRecoveryScale = 0.5f;

        const float OverspeedRecoveryPerSecond = 1f;
        const float OverspeedPlayerRecoveryPerSecond = 0.5f;

        static float Glide(float current, float level, float fallRate, float riseRate, float tickScale)
        {
            float rate = level < current ? fallRate : riseRate;
            float step = rate * tickScale;
            return ARS.Clamp(current + ARS.Clamp(level - current, -step, step), 0f, 1f);
        }

        float GlideCap(float current, float level)
        {
            return GlideCap(current, level, ReasonCapSlewRate * ReasonCapRecoveryScale);
        }

        float GlideCap(float current, float level, float risePerSecond)
        {
            return Glide(current, level, ReasonCapSlewRate, risePerSecond, TickScale);
        }

        float SlideBrakePedalCap()
        {
            if (!ARS.SlideBrakeCap || TRLateralAtSpeed <= 0.01f) return 1f;
            float t = ARS.Clamp((_slideShare - SlideBrakeCapStartShare) / (SlideBrakeCapEndShare - SlideBrakeCapStartShare), 0f, 1f);
            return 1f + (SlideBrakeCap - 1f) * t;
        }

        float OverspeedThrottleLevel()
        {
            if (!TrackSpeed.OverspeedCorrection) return 1f;

            bool pulling = VehicleData.GetLongitudinalGs(Car.ForwardVector) > OverspeedArmMinForwardGs;
            bool armed = pulling
                && Math.Abs(VehicleData.SlideAngle) < OverspeedArmMaxSlideDegrees
                && Car.Velocity.Length() > ARS.MphToMps(OverspeedArmMinSpeedMph);

            if (ARS.OverspeedCutPerTenthG <= 0f || !armed || !(VehicleData.OverspeedExcessGs > 0f)) return 1f;
            float cut = ARS.OverspeedCutPerTenthG * (VehicleData.OverspeedExcessGs - OverspeedCutStartGs) / 0.1f;
            return ARS.Clamp(1f - cut, 0f, 1f);
        }

        void UpdateThrottleReasonCaps()
        {

            Control.MaxThrottleFromTCS = _kickPlaying ? 1f : GlideCap(Control.MaxThrottleFromTCS, TcsCapLevel());
            _tcsSmooth.Target = Control.MaxThrottleFromTCS;
            _tcsSmooth.Advance(TickScale);
            int wheelsOffGround = ARS.WheelsOffGround(Car);
            if (TrackSpeed.StabilityCap && wheelsOffGround > 0)
                Control.MaxThrottleFromStability -= StabilityFallPerWheelPerSecond * wheelsOffGround * TickScale;
            else
                Control.MaxThrottleFromStability += StabilityRisePerSecond * TickScale;
            Control.MaxThrottleFromStability = ARS.Clamp(Control.MaxThrottleFromStability, StabilityFloor, 1f);
            _stabSmooth.Target = Control.MaxThrottleFromStability;
            _stabSmooth.Advance(TickScale);

            Control.MaxThrottleFromOverspeed = Game.GameTime < _nitrousActiveUntil
                ? 1f
                : GlideCap(Control.MaxThrottleFromOverspeed, OverspeedThrottleLevel(), OverspeedRecoveryPerSecond);

            float rivalLevel = 1f;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || r.RelativePosition != RelativePos.Ahead) continue;
                rivalLevel = Math.Min(rivalLevel, ARS.Remap(r.FrontGapMetres, 0f, RivalLiftMeters, RivalLiftFloor, 1f, true));
            }
            Control.MaxThrottleFromRival = GlideCap(Control.MaxThrottleFromRival, rivalLevel);

            Control.MaxThrottleFromChillOut = GlideCap(Control.MaxThrottleFromChillOut, ActiveManeuver.Type == ManeuverType.ChillOut ? ChillThrottleCap : 1f);
            Control.MaxThrottleFromYield = GlideCap(Control.MaxThrottleFromYield, ActiveManeuver.Type == ManeuverType.Yield && ActiveManeuver.Target != null ? YieldThrottleLevel : 1f);

            float offTrackThrottle = 1f;
            if (TrackSpeed.OffTrackThrottle && SignedMetresBeyondTrackEdge() > 0f)
            {
                float level = ProjectionClosingOnCentre() ? OffTrackRecoverThrottle : OffTrackWideningThrottle;
                offTrackThrottle = ARS.Remap(ARS.MpsToMph(Car.Velocity.Length()), OffTrackSafeSpeedMph, OffTrackFullThrottleMph, level, 1f, true);
            }
            Control.MaxThrottleFromOffTrack = GlideCap(Control.MaxThrottleFromOffTrack, offTrackThrottle);

            float recoveryThrottle = 1f;
            if (_stuckPhase == StuckPhase.Drive) recoveryThrottle = ARS.Remap(ARS.MpsToMph(Car.Velocity.Length()), RecoveryDriveMph + RecoveryCoastBandMph, RecoveryDriveMph, 0f, 1f, true);
            Control.MaxThrottleFromRecovery = GlideCap(Control.MaxThrottleFromRecovery, recoveryThrottle);
        }

        static float SlipForShare(int sharePercent)
        {
            return sharePercent * 0.01f * ARS.SlipPeakRatio;
        }

        float TcsCapLevel()
        {
            if (!ARS.TcsEnabled) return 1f;
            float wheelspin = ARS.MaxWheelSlip(Car);

            float slipTarget = -SlipForShare(ARS.TcsSlipLimitPct) * (SignedMetresBeyondTrackEdge() > 0f ? OffTrackTargetShare : 1f);
            float spinDepth = Math.Max(0f, slipTarget - wheelspin);
            float depthShare = TaperShare(spinDepth, SlipForShare(ARS.TcsTaperSpanPct));
            return 1f - depthShare * (1f - TcsCapFloor);
        }
        const float AbsLockRadPerSecond = 2f;
        const float AbsMinSpeedMph = 15f;
        const float AbsKneeSlip = 0.5f;
        const float AbsLockSlip = 1f;
        const float AbsBrakeFloor = 0f;
        int _absLoggedAt = 0;
        bool _absWasLocked = false;

        void UpdateBrakeReasonCaps()
        {
            bool countersteering = IsFullCountersteer();
            Control.MaxBrakeFromABS = AbsCapLevel();
            Control.MaxBrakeFromCountersteer = GlideCap(Control.MaxBrakeFromCountersteer,
                countersteering && !_brakePlanActive ? CountersteerPedalCap : 1f);
            Control.MaxBrakeFromSlide = GlideCap(Control.MaxBrakeFromSlide, SlideBrakePedalCap());

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

            if (ARS.AnyWheelRotationBelow(Car, AbsLockRadPerSecond)) return AbsBrakeFloor;
            float slip = ARS.BrakeLockSlip(Car);
            if (slip <= AbsKneeSlip) return 1f;
            float overKnee = (slip - AbsKneeSlip) / Math.Max(AbsLockSlip - AbsKneeSlip, 0.01f);
            return Math.Max(AbsBrakeFloor, 1f - overKnee);
        }

        float ComposeThrottleCap(float baseThrottle, bool countersteering, float floor)
        {

            bool kickRaises = _kickPlaying && baseThrottle < KickPedalLevel;
            if (kickRaises) baseThrottle = KickPedalLevel;

            float stabilityCap = Math.Max(StabilityThrottleFloor, Control.MaxThrottleFromStability);
            float ceiling = Math.Min(Control.MaxThrottleFromTCS, stabilityCap);
            ceiling = Math.Min(ceiling, Control.MaxThrottleFromOverspeed);
            ceiling = Math.Min(ceiling, Control.MaxThrottleFromRival);
            ceiling = Math.Min(ceiling, Control.MaxThrottleFromChillOut);
            ceiling = Math.Min(ceiling, Control.MaxThrottleFromYield);
            ceiling = Math.Min(ceiling, Control.MaxThrottleFromRecovery);
            ceiling = Math.Min(ceiling, Control.MaxThrottleFromOffTrack);
            Control.MaxThrottle = ceiling;

            float raised = Math.Min(floor, ceiling);
            _gsFloorApplied = raised;
            _gsFloorActiveThisTick = raised > baseThrottle;
            if (_gsFloorActiveThisTick) baseThrottle = raised;
            Control.ThrottleDemand = baseThrottle;

            float binding = baseThrottle;
            ThrottleReason reason = ThrottleReason.Plan;
            if (Control.MaxThrottleFromTCS < binding) { binding = Control.MaxThrottleFromTCS; reason = ThrottleReason.Tcs; }
            if (stabilityCap < binding) { binding = stabilityCap; reason = ThrottleReason.Stability; }
            if (Control.MaxThrottleFromOverspeed < binding) { binding = Control.MaxThrottleFromOverspeed; reason = ThrottleReason.Overspeed; }
            if (Control.MaxThrottleFromRival < binding) { binding = Control.MaxThrottleFromRival; reason = ThrottleReason.Rival; }
            if (Control.MaxThrottleFromChillOut < binding) { binding = Control.MaxThrottleFromChillOut; reason = ThrottleReason.ChillOut; }
            if (Control.MaxThrottleFromYield < binding) { binding = Control.MaxThrottleFromYield; reason = ThrottleReason.Yield; }
            if (Control.MaxThrottleFromRecovery < binding) { binding = Control.MaxThrottleFromRecovery; reason = ThrottleReason.StuckRecovery; }
            if (Control.MaxThrottleFromOffTrack < binding) { binding = Control.MaxThrottleFromOffTrack; reason = ThrottleReason.Offtrack; }
            if (countersteering) reason = ThrottleReason.Countersteer;
            if (BaseBehavior == RacerBaseBehavior.GridWait) reason = ThrottleReason.GridWait;
            if (_kickPlaying && reason == ThrottleReason.Plan) reason = ThrottleReason.Kick;

            Control.ThrottleReason = reason;
            Control.ThrottleReasonCap = binding;
            Control.ThrottleLimited = binding < baseThrottle - 0.001f;
            return binding;
        }

        float ComposeBrakeCap(float baseBrake, bool countersteering)
        {
            float routeCap = Brain.CurrentIntention.Term == SpeedTerm.Route ? RouteSpeedBrakeCap : 1f;

            Control.MaxBrake = Math.Min(Control.MaxBrakeFromABS, Control.MaxBrakeFromCountersteer);
            Control.MaxBrake = Math.Min(Control.MaxBrake, Control.MaxBrakeFromSlide);
            Control.MaxBrake = Math.Min(Control.MaxBrake, routeCap);

            float binding = baseBrake;
            BrakeReason reason = BrakeReason.Plan;
            if (Control.MaxBrakeFromABS < binding) { binding = Control.MaxBrakeFromABS; reason = BrakeReason.Abs; }
            if (Control.MaxBrakeFromCountersteer < binding) { binding = Control.MaxBrakeFromCountersteer; reason = BrakeReason.Countersteer; }
            if (Control.MaxBrakeFromSlide < binding) { binding = Control.MaxBrakeFromSlide; reason = BrakeReason.Slide; }
            if (routeCap < binding) { binding = routeCap; reason = BrakeReason.RouteSpeed; }
            if (countersteering) reason = BrakeReason.Countersteer;
            if (BaseBehavior == RacerBaseBehavior.GridWait) reason = BrakeReason.GridWait;

            Control.BrakeReason = reason;
            Control.BrakeReasonCap = binding;
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

        void ConsiderManeuvers()
        {
            if (ControlledByPlayer || BaseBehavior != RacerBaseBehavior.Race || ARS.Racers.Count < 1) return;

            if (ActiveManeuver.Type != ManeuverType.None && Game.GameTime - ActiveManeuver.EnabledAtMs > 8000)
            {
                EndCard();
                _divebombApexNode = -1;
            }

            if (ActiveManeuver.Type == ManeuverType.DiveBomb)
            {
                Rival divePartner = null;
                foreach (Rival r in Brain.Rivals)
                {
                    if (r.RivalRacer != ActiveManeuver.Target) continue;
                    divePartner = r;
                    break;
                }
                bool partnerLost = ActiveManeuver.Target == null || !ActiveManeuver.Target.Car.Exists() || divePartner == null || Math.Abs(divePartner.RouteGapAhead) > DivebombReleaseGapMeters || Math.Abs(divePartner.DirectionDiffDegrees) > DivebombReleaseAngleDegrees;

                if ((partnerLost || HasPassedApex(_divebombApexNode)) && CardMayEnd)
                {
                    ARS.Log(ARS.LogImportance.Info, "Card divebomb: " + Name + " released"
                        + (HasPassedApex(_divebombApexNode) ? " at apex" : " partner lost")
                        + " after " + (Game.GameTime - ActiveManeuver.EnabledAtMs) + " ms");
                    EndCard();
                    _divebombApexNode = -1;
                }
            }

            if (ActiveManeuver.Type == ManeuverType.DefendLane)
            {
                bool targetTrails = false;
                bool threatGone = false;
                foreach (Rival r in Brain.Rivals)
                {
                    if (r.RivalRacer != ActiveManeuver.Target) continue;
                    targetTrails = r.RelativePosition != RelativePos.Ahead;

                    float hisTimeToReachMe = float.PositiveInfinity;
                    foreach (Rival back in r.RivalRacer.Brain.Rivals)
                    {
                        if (back.RivalRacer != this) continue;
                        hisTimeToReachMe = back.TimeToReachSeconds;
                        break;
                    }
                    float combinedLength = Math.Abs(VehicleData.ModelDimensions.Y) + Math.Abs(r.RivalRacer.VehicleData.ModelDimensions.Y);

                    bool stillClosing = !float.IsPositiveInfinity(hisTimeToReachMe);
                    bool stillClose = r.Distance < DefendCloseBehindMeters + combinedLength;
                    threatGone = !stillClosing && !stillClose;
                    break;
                }
                bool lostTarget = ActiveManeuver.Target == null || !ActiveManeuver.Target.Car.Exists() || !targetTrails;

                bool outsideOver = SignedMetresToNode(_defendApexNode) <= OutsideGateMeters(CurrentTrackPoint);

                if ((lostTarget || threatGone || outsideOver || HasPassedApex(_defendApexNode)) && CardMayEnd)
                {
                    EndCard();
                    _defendApexNode = -1;
                }
            }

            if (ActiveManeuver.Type == ManeuverType.None) TryPlayNitrousCard();

            if (ActiveManeuver.Type == ManeuverType.None) TryPlayDefendLaneCard();

            if (ActiveManeuver.Type == ManeuverType.None) TryPlayDivebombCard();

            if (ActiveManeuver.Type == ManeuverType.None) TryPlayYieldCard();
        }

        void ConsiderCrowdCard()
        {
            if (ControlledByPlayer || BaseBehavior != RacerBaseBehavior.Race || ARS.Racers.Count < 1) return;

            if (ActiveManeuver.Type == ManeuverType.ChillOut && CardMayEnd && RivalsWithinDistance(ChillReleaseRadiusMeters) < ChillCrowdCount - 1)
            {
                EndCard();
            }

            const bool ChillOutEnabled = false;

            if (ChillOutEnabled && ActiveManeuver.Type == ManeuverType.None && ARS.MpsToMph(Car.Velocity.Length()) >= ChillMinSpeedMph && RivalsWithinDistance(ChillRivalCrowdDistance) >= ChillCrowdCount)
            {
                Rival closestRival = NearestRival();
                if (closestRival != null && CardFree(closestRival.RivalRacer, ManeuverType.ChillOut))
                {
                    ActiveManeuver.Type = ManeuverType.ChillOut;
                    ActiveManeuver.Target = closestRival.RivalRacer;
                    ActiveManeuver.EnabledAtMs = Game.GameTime;
                }
            }
        }

        int ForwardNodeDistance(int targetNode)
        {
            int fwd = targetNode - CurrentTrackPoint.Node;
            if (!ARS.IsPointToPoint && fwd < 0) fwd += ARS.TrackPoints.Count;
            return fwd;
        }

        float SignedMetresToNode(int node)
        {
            if (node < 0 || node >= ARS.TrackPoints.Count) return 0f;
            float metres = ARS.TrackPoints[node].CumulativeDistance - CurrentTrackPoint.CumulativeDistance;
            if (!ARS.IsPointToPoint && ARS.RouteLengthMeters > 0f && Math.Abs(metres) > ARS.RouteLengthMeters * 0.5f)
                metres -= Math.Sign(metres) * ARS.RouteLengthMeters;
            return metres;
        }

        float OutsideGateMeters(TrackPoint at) => Math.Max(2f * at.TrackHalfWidth * IdealLineBlendWidthFactor, IdealLineBlendMinMeters);

        const float LaneTurnSignDeadband = 0.002f;
        int LaneTurnSignToAim(TrackPoint aim)
        {
            Vector3 a = CurrentTrackPoint.Direction;
            Vector3 b = aim.Direction;
            float turn = a.X * b.Y - a.Y * b.X;
            if (Math.Abs(turn) > LaneTurnSignDeadband) _laneTurnSign = Math.Sign(turn);
            return _laneTurnSign;
        }

        int TrackTurnSign(int fromNode, int toNode, int fallback)
        {
            Vector3 a = ARS.TrackPoints[fromNode].Direction;
            Vector3 b = ARS.TrackPoints[toNode].Direction;
            float turn = a.X * b.Y - a.Y * b.X;
            if (Math.Abs(turn) <= LaneTurnSignDeadband) return fallback;
            return Math.Sign(turn);
        }

        bool ApexAheadOfLookahead(CornerPoint corner, float lookaheadMetres)
        {
            return corner != null && corner.Node >= 0 && SignedMetresToNode(corner.Node) - lookaheadMetres > 0f;
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

            if (_nitrousActiveUntil > 0 && (!ARS.NitrousEnabled || Game.GameTime >= _nitrousActiveUntil)) StopNitrous();
        }

        static bool CardFree(Racer target, ManeuverType type)
        {
            return target == null || target.ActiveManeuver.Type != type;
        }

        bool TryPlayNitrousCard()
        {
            if (!ARS.NitrousEnabled || Lap <= _nitrousLapUsed) return false;
            if (ARS.RaceStartTime > 0 && Game.GameTime - ARS.RaceStartTime < NitrousArmDelayMs) return false;
            if (Control.Brake > 0f) return false;
            if (SignedMetresBeyondTrackEdge() > 0f) return false;
            if (Math.Abs(Control.SteerDegrees) >= NitrousMaxSteerDegrees) return false;

            float nitrousSlide = VehicleData.SlideAngle;
            if (float.IsNaN(nitrousSlide) || Math.Abs(nitrousSlide) >= NitrousMaxSlideDegrees) return false;
            if (Control.Throttle < NitrousMinThrottle) return false;
            if (Function.Call<int>((Hash)CurrentDriveGearHash, Car) <= 1) return false;
            if (NextApexNode < 0) return false;

            float speed = Car.Velocity.Length();
            Rival closestRival = NearestRival();

            bool finishSpender = closestRival != null && closestRival.Distance <= NitrousNearbyRivalDistance
                && RemainingRaceDistanceMeters() <= speed * (NitrousDurationMs / 1000f) + NitrousFinishExtraDistance;
            if (!finishSpender)
            {
                CornerPoint corner = _nextApexCorner;

                if (NitrousRouteRadius() <= NitrousMinRouteRadiusMeters) return false;
                int entranceNode = corner == null
                    ? NextApexNode
                    : (corner.StartNode >= 0 ? corner.StartNode : OffsetCornerNode(NextApexNode, -corner.EntranceNodes));

                if (SignedMetresToNode(entranceNode) / Math.Max(speed, 1f) < NitrousCornerLookaheadSeconds) return false;
            }

            bool rivalNearbyFaster = false;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || !r.RivalRacer.Car.Exists()) continue;
                r.UpdateRouteGap(this);
                if (Math.Abs(r.RouteGapAhead) <= NitrousFasterReachMeters && r.Speed > speed) { rivalNearbyFaster = true; break; }
            }
            bool rivalBehindIncoming = false;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || !r.RivalRacer.Car.Exists() || r.RelativePosition != RelativePos.Behind || r.Speed <= speed) continue;
                if (r.Distance / (r.Speed - speed) < NitrousDefenseReachSeconds) { rivalBehindIncoming = true; break; }
            }
            bool lonelyClear = closestRival == null
                && speed < Handling.EstimatedTopSpeedMps * NitrousLonelySpeedFraction
                && ForwardNodeDistance(NextApexNode) > NitrousLonelyMinApexDistance;
            if (!rivalNearbyFaster && !rivalBehindIncoming && !lonelyClear && !finishSpender) return false;

            StartNitrous();
            return true;
        }
        const float FightWindowMinSeconds = 2f;
        const float FightWindowMaxSeconds = 6f;
        const float DivebombReachSeconds = 0.75f;
        const float DivebombFullThrottle = 0.99f;

        bool TryPlayDivebombCard()
        {
            if (!CardCooled(ManeuverType.DiveBomb)) return false;
            if (Brain.Corner == null) return false;
            if (!CornerAsksForBraking()) return false;
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

                float rivalTimeToEntrance = r.RivalRacer.ForwardNodeDistance(entranceNode) / Math.Max(r.Speed, 1f);
                if (rivalTimeToEntrance >= timeToEntrance) continue;
                if (timeToEntrance - rivalTimeToEntrance > DivebombReachSeconds) continue;

                if (r.RouteGapAhead <= 0f || r.RouteGapAhead > SameSectionMaxGapMeters) continue;
                if (r.ForwardSpeedGap <= 0f) continue;

                if (Math.Abs(r.DirectionDiffDegrees) > AvoidAngleGateDegrees) continue;
                if (!CardFree(r.RivalRacer, ManeuverType.DiveBomb)) continue;
                if (r.Distance < nearestDive) { nearestDive = r.Distance; diveTarget = r; }
            }
            if (diveTarget == null) return false;

            ActiveManeuver.Type = ManeuverType.DiveBomb;
            ActiveManeuver.Target = diveTarget.RivalRacer;
            ActiveManeuver.EnabledAtMs = Game.GameTime;
            _divebombApexNode = apexNode;
            _divebombBrakeBonus = ARS.GetRandomInt(2, 9) / 100f;
            ARS.Log(ARS.LogImportance.Info, "Card divebomb: " + Name + " armed on " + diveTarget.RivalRacer.Name
                + " apex " + apexNode + " bonus +" + _divebombBrakeBonus.ToString("0.00") + " gap " + diveTarget.RouteGapAhead.ToString("0.0")
                + " dT " + (timeToEntrance - diveTarget.RivalRacer.ForwardNodeDistance(entranceNode) / Math.Max(diveTarget.Speed, 1f)).ToString("0.00"));
            return true;
        }

        bool TryPlayDefendLaneCard()
        {
            if (!CardCooled(ManeuverType.DefendLane)) return false;
            if (Brain.Corner == null) return false;
            if (!CornerAsksForBraking()) return false;

            int apexNode = Brain.Corner.Point.Node;
            int entranceNode = CornerEntranceNode(Brain.Corner.Point, apexNode);
            if (entranceNode < 0) return false;
            float myTimeToEntrance = ForwardNodeDistance(entranceNode) / Math.Max(Car.Velocity.Length(), 1f);
            if (!ARS.IsBetween(myTimeToEntrance, FightWindowMinSeconds, FightWindowMaxSeconds)) return false;

            if (SignedMetresToNode(apexNode) <= OutsideGateMeters(CurrentTrackPoint)) return false;

            Rival defenderTarget = null;
            float nearestDefender = float.MaxValue;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || !r.RivalRacer.Car.Exists()) continue;
                if (r.RivalRacer.ActiveManeuver.Type == ManeuverType.DiveBomb || r.RivalRacer.ActiveManeuver.Type == ManeuverType.DefendLane) continue;
                if (r.RelativePosition != RelativePos.Behind) continue;

                float timeToReachMe = float.PositiveInfinity;
                foreach (Rival back in r.RivalRacer.Brain.Rivals)
                {
                    if (back.RivalRacer != this) continue;
                    timeToReachMe = back.TimeToReachSeconds;
                    break;
                }

                float combinedLength = Math.Abs(VehicleData.ModelDimensions.Y) + Math.Abs(r.RivalRacer.VehicleData.ModelDimensions.Y);
                bool closesSoon = ARS.IsBetween(timeToReachMe, 3f, 6f);
                bool closeBehind = r.Distance < DefendCloseBehindMeters + combinedLength;
                if (!closesSoon && !closeBehind) continue;

                float hisTimeToEntrance = r.RivalRacer.ForwardNodeDistance(entranceNode) / Math.Max(r.Speed, 1f);
                if (hisTimeToEntrance > myTimeToEntrance + DefendLaterMarginSeconds) continue;

                if (r.Distance < nearestDefender) { nearestDefender = r.Distance; defenderTarget = r; }
            }
            if (defenderTarget == null) return false;

            int defendTurn = TrackTurnSign(Brain.Corner.Point.StartNode, Brain.Corner.Point.Node, Math.Sign(Brain.Corner.Point.Angle));
            if (defendTurn == 0) return false;

            Racer defender = defenderTarget.RivalRacer;
            float hisLane = ARS.SignedLaneOffset(defender.Car.Position, defender.CurrentTrackPoint.Position, defender.CurrentTrackPoint.Direction);
            float combinedWidth = Math.Abs(VehicleData.ModelDimensions.X) + Math.Abs(defender.VehicleData.ModelDimensions.X);
            float clearance = (Math.Abs(VehicleData.SlidingWidthMetres) + Math.Abs(defender.VehicleData.SlidingWidthMetres)) * 0.5f;
            float reachable = Math.Abs(CurrentTrackPoint.TrackHalfWidth) - Math.Abs(VehicleData.SlidingWidthMetres) * 0.5f - EdgeEmptySpaceMeters;
            float inward = -defendTurn;

            if (Math.Abs(hisLane + inward * clearance) > reachable) return false;

            ActiveManeuver.Type = ManeuverType.DefendLane;
            ActiveManeuver.Target = defender;
            ActiveManeuver.EnabledAtMs = Game.GameTime;
            _defendApexNode = apexNode;

            _defendLaneOffset = ARS.Clamp(hisLane + inward * combinedWidth, -reachable, reachable);
            return true;
        }

        bool TryPlayYieldCard()
        {
            if (!CardCooled(ManeuverType.Yield)) return false;
            if (Brain.Corner == null) return false;
            if (!CornerAsksForBraking()) return false;

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
            ActiveManeuver.EnabledAtMs = Game.GameTime;
            return true;
        }

        float RemainingRaceDistanceMeters()
        {
            int nodeCount = ARS.TrackPoints.Count;
            if (nodeCount < 2) return 0f;
            float nodes = ARS.IsPointToPoint
                ? nodeCount - CurrentTrackPoint.Node
                : (ARS.RaceMenuStore.GetInt("Laps", 6) + 1f - Lap) * nodeCount - CurrentTrackPoint.Node;

            float routeLength = ARS.RouteLengthMeters;
            if (routeLength <= 0f) routeLength = ARS.TrackPoints[nodeCount - 1].CumulativeDistance;
            float metresPerNode = routeLength > 0f ? routeLength / (nodeCount - 1) : 1f;
            return Math.Max(0f, nodes) * metresPerNode;
        }

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

            if (!ControlledByPlayer)
            {
                float planMph = Brain.CurrentIntention.Speed >= 0f ? ARS.MpsToMph(Brain.CurrentIntention.Speed) : -1f;
                ARS.Log(ARS.LogImportance.Info, "AI nitrous: " + Car.DisplayName + " lap " + Lap + ", plan " + planMph.ToString("0") + " mph, packing x" + ComputeRubberBandFactor().ToString("0.00"));
            }
        }

        public bool TryFireNitrous()
        {

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

            if (ActiveManeuver.Target != null && ActiveManeuver.Target.Car.Exists())
            {
                float dist = Car.Position.DistanceTo(ActiveManeuver.Target.Car.Position);
                if (dist > 10f) EndCard();
            }
            else EndCard();
        }

        const float ContactJumpGs = 3f;

        public int LastContactTime = -1;
        public int MsSinceContact() => LastContactTime < 0 ? int.MaxValue : Game.GameTime - LastContactTime;

        public void UpdateTickData()
        {
            Vector3 cSpeed = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, false);

            int now = Game.GameTime;
            int elapsed = now - VehicleData.LastAccelSampleTime;
            if (elapsed >= VehicleState.AccelIntervalMs)
            {
                VehicleData.LastAccelSampleTime = now;
                float dt = Math.Max(elapsed * 0.001f, 0.001f);
                Vector3 accel = (cSpeed - _lastSpeed) / dt;

                if (VehicleData.AccelCount > 0)
                {
                    float jumpX = accel.X - _lastAccel.X;
                    float jumpY = accel.Y - _lastAccel.Y;
                    if (Math.Sqrt(jumpX * jumpX + jumpY * jumpY) > ContactJumpGs * 9.8f) LastContactTime = now;
                }
                _lastAccel = accel;
                VehicleData.AccelSamples[VehicleData.AccelHead] = accel;
                VehicleData.AccelHead = (VehicleData.AccelHead + 1) % VehicleState.AccelWindow;
                if (VehicleData.AccelCount < VehicleState.AccelWindow) VehicleData.AccelCount++;
                _lastSpeed = cSpeed;
                VehicleData.AccumulateLapPeaks(accel, cSpeed, Car.ForwardVector);
            }

            VehicleData.SpeedVectorLocal = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true);
        }

        public Vector3 ProjectAhead(float seconds = 1f)
        {
            return Car.Position + Car.Velocity * seconds + 0.5f * VehicleData.AverageAcceleration * seconds * seconds;
        }

        void DrawDebug()
        {

            if (!ControlledByPlayer && ARS.DebugToggles[Options.ShowTrLateral] && ARS.Racers.Count > 0
                && this == ARS.Racers.OrderBy(c => c.Car.Position.DistanceTo(Game.Player.Character.Position)).First())
                DrawTrLateral();

            if (ControlledByPlayer || ARS.DebugFocusRacer != this) return;

            CornerPoint shapedCorner = _laneShapeApexNode >= 0 ? ARS.Corners.FirstOrDefault(c => c.Node == _laneShapeApexNode) : null;
            if (ARS.DebugToggles[Options.ShowTrackAnalysis] && _laneShapeApexNode >= 0
                && (shapedCorner == null || TrackLoader.ApexAllowsOutsideApproach(shapedCorner)))
            {
                int count = ARS.TrackPoints.Count;
                for (int direction = -1; direction <= 1; direction += 2)
                {
                    int lastShaped = _laneShapeApexNode;
                    for (int step = (direction < 0 ? 0 : 1); step < count; step++)
                    {
                        int n = direction < 0
                            ? (_laneShapeApexNode - step)
                            : (_laneShapeApexNode + step);
                        if (ARS.IsPointToPoint) { if (n < 0 || n >= count) break; }
                        else n = ((n % count) + count) % count;
                        TrackPoint nodePoint = ARS.TrackPoints[n];
                        if (!nodePoint.LaneShaped || nodePoint.LaneShapeApex != _laneShapeApexNode) break;
                        lastShaped = n;
                        if ((n & 1) == 0) continue;
                        float half = Math.Max(nodePoint.TrackHalfWidth - VehicleData.SlidingWidthMetres * 0.5f, 0f);
                        Vector3 right = Vector3.Normalize(Vector3.Cross(nodePoint.Direction, Vector3.WorldUp));
                        World.DrawMarker(MarkerType.DebugSphere, nodePoint.Position + Vector3.WorldUp * 0.6f + right * (nodePoint.LaneLineShare * half), Vector3.Zero, Vector3.Zero, new Vector3(0.25f, 0.25f, 0.25f), Color.Black);
                    }
                    if (lastShaped != _laneShapeApexNode)
                    {
                        TrackPoint end = ARS.TrackPoints[lastShaped];
                        float half = Math.Max(end.TrackHalfWidth - VehicleData.SlidingWidthMetres * 0.5f, 0f);
                        Vector3 right = Vector3.Normalize(Vector3.Cross(end.Direction, Vector3.WorldUp));
                        Vector3 at = end.Position + Vector3.WorldUp * 0.6f + right * (end.LaneLineShare * half);
                        World.DrawMarker(MarkerType.DebugSphere, at, Vector3.Zero, Vector3.Zero, new Vector3(0.9f, 0.9f, 0.9f), direction < 0 ? Color.Red : Color.Green);
                    }
                }
            }

            if (ARS.DebugToggles[Options.ShowInputs]) DrawInputTrail();
            if (ARS.DebugToggles[Options.ShowWheelSlip]) DrawWheelSlip();
            if (ARS.DebugToggles[Options.ShowFrontSlip]) DrawFrontSlipReadout();
            if (ARS.DebugToggles[Options.ShowSlideAngle]) DrawSlideAngleReadout();
            if (ARS.DebugToggles[Options.ShowGsProjection])
            {
                DrawGsProjection();
                DrawProjectionGateSubtitle();
            }
        }
        public void ProcessTick()
        {
            UpdateTickData();

            if (ARS.DebugToggles[Options.ShowInputs] && !ControlledByPlayer && ARS.DebugFocusRacer == this) SampleInputTrail();
            DrawDebug();

            _appliedThrottleLastFrame = VehicleMemory.GetThrottle(Car);            if (!ControlledByPlayer)
            {
                ApplyInputs();
            }

            UpdatePowerCompensation();
            ApplyPowerMultiplier();
        }

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

                float pitchRad = input * InputTrailChevronPitch * ((float)Math.PI / 180f);
                Vector3 direction = heading * (float)Math.Cos(pitchRad) - Vector3.WorldUp * (float)Math.Sin(pitchRad);
                float groundZ = sample.Position.Z - restHeight + InputTrailChevronGroundClearance;
                Vector3 position = new Vector3(sample.Position.X, sample.Position.Y, groundZ - input * InputTrailChevronHeightGain);

                Color colour = fade < InputTrailFadeCount ? Color.FromArgb(255 * (fade + 1) / (InputTrailFadeCount + 1), InputColour(input)) : InputColour(input);
                World.DrawMarker(MarkerType.ChevronUpx1, position, direction, new Vector3(89f, 90f, -90f), new Vector3(InputTrailChevronSize, InputTrailChevronSize, InputTrailChevronSize), colour, false, false, 2, false, "", "", false);
            }
        }
        static readonly Color SlipRestColor = Color.White;
        static readonly Color SlipPeakColor = Color.Lime;
        static readonly Color SlipSkidColor = Color.Yellow;
        static readonly Color SlipFloorColor = Color.Red;
        const float SlipSkidLevel = 2f;
        const float SlipFloorLevel = 2.5f;
        const float WheelSlipSphereHeight = 1f;
        const float WheelSlipSphereSize = 0.35f;
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

            return Car.Position + Vector3.WorldUp * (VehicleData.ModelDimensions.Z + lift);
        }

        void DrawGsProjection()
        {

            Vector3 origin = ProjectionOrigin();
            float lift = origin.Z - Car.Position.Z;
            Vector3 half = DampedVertical(ProjectAhead(0.5f) + Vector3.WorldUp * lift, origin.Z);
            Vector3 full = DampedVertical(ProjectAhead(1f) + Vector3.WorldUp * lift, origin.Z);

            ARS.DrawLine(origin, half, _projectionPathColor);
            ARS.DrawLine(half, full, _projectionPathColor);

            if (_projectionDropMph > 0.5f)
                ARS.DrawText(full + Vector3.WorldUp * 0.5f,
                             "-" + ((int)Math.Round(_projectionDropMph)).ToString(CultureInfo.InvariantCulture) + " MPH",
                             _projectionCutActive ? Color.Red : Color.Yellow, 0.45f);
            DrawGReadout();
        }

        public void DrawGReadout()
        {
            Vector3 accel = VehicleData.AverageAcceleration;
            Vector3 fwd = Car.ForwardVector;
            fwd.Z = 0f;
            fwd.Normalize();
            Vector3 right = Vector3.Cross(fwd, Vector3.WorldUp);
            float latG = Vector3.Dot(accel, right) / 9.8f;
            float lonG = Vector3.Dot(accel, fwd) / 9.8f;
            Vector3 pos = Car.Position + Vector3.WorldUp * (VehicleData.ModelDimensions.Z + 0.5f);
            string text = "L" + latG.ToString("0.0", CultureInfo.InvariantCulture)
                        + " D" + lonG.ToString("0.0", CultureInfo.InvariantCulture);
            ARS.DrawText(pos, text, Color.White, 0.5f);
        }

        // Learner rules: the fall is the wash-out; the rise needs route demand under the full-throttle line,
        // the projection nearer the inside than the car, a slide worth a fifth of the peak and an uncut cap.
        void UpdateRouteSpeedLearning()
        {
            float step = RouteSpeedLearnRateMph * TickScale;

            if (ARS.RouteOffsetMph != _routeOffsetSeedMph)
            {
                SetRouteSpeedOffset(_routeSpeedOffsetMph + ARS.RouteOffsetMph - _routeOffsetSeedMph);
                _routeOffsetSeedMph = ARS.RouteOffsetMph;
            }

            if (!RouteSpeedLearningEnabled) return;

            if (IsWashingOut())
            {
                if (Brain.CurrentIntention.Term != SpeedTerm.Route) return;
                SetRouteSpeedOffset(_routeSpeedOffsetMph - step * RouteSpeedLearnFallShareOfRate);
                return;
            }

            if (_cornerMode || Brain.CurrentIntention.Term != SpeedTerm.Route) return;

            if (Control.ThrottleDemand >= 1f || Control.ThrottleLimited) return;
            if (Control.Brake > RouteSpeedLearnBrake) return;
            if (!ProjectionWideningWithinLimit()) return;
            float slideAbs = Math.Abs(VehicleData.SlideAngle);
            if (TRLateralAtSpeed <= 0.01f || float.IsNaN(slideAbs)) return;
            if (slideAbs < TRLateralAtSpeed * RouteSpeedLearnMinSlideShare) return;
            if (Control.MaxThrottleFromStability < RouteSpeedLearnMinStabilityCap) return;
            SetRouteSpeedOffset(_routeSpeedOffsetMph + step);
        }

        // One writer for the offset, so its bound lives in one place: the row seeds it, the learner moves it.
        void SetRouteSpeedOffset(float value)
        {
            _routeSpeedOffsetMph = ARS.Clamp(value, -RouteSpeedLearnLimitMph, RouteSpeedLearnLimitMph);
        }

        bool ProjectionWideningWithinLimit()
        {
            return _projectionWidenMeters > 0f && _projectionWidenMeters < RouteSpeedLearnWidenMetres;
        }

        bool IsWashingOut()
        {
            if (Control.ThrottleDemand < RouteSpeedLearnFullThrottle) return false;
            float steer = Control.SteerInput;
            if (Math.Abs(steer) < RouteSpeedLearnSteerMin) return false;
            if (_projectionWidenMeters < RouteSpeedLearnWidenMetres) return false;
            return Math.Sign(steer) == _projectionWidenSign;
        }

        void DrawProjectionGateSubtitle()
        {
            float slide = Math.Abs(VehicleData.SlideAngle);
            if (float.IsNaN(slide) || float.IsInfinity(slide)) slide = 0f;
            float share = TRLateralAtSpeed > 0.01f ? slide / TRLateralAtSpeed : 0f;
            string percent = ((int)Math.Round(share * 100f)).ToString(CultureInfo.InvariantCulture) + "%";
            UI.ShowSubtitle(_projectionArmed ? "~r~" + percent : "~b~" + percent, 50);
        }

        public void DrawRouteSpeedPointOffsets()
        {
            // The cursor's own point plus the next four, always on screen: lime only while the cursor's
            // point actually drives, white through trust gaps - the overlay never hides with the trust.
            if (_routeSpeedPoints.Count == 0 || _routeSpeedCursor < 0) return;
            for (int i = 0; i < 5; i++)
            {
                int index = _routeSpeedCursor + i;
                if (index >= _routeSpeedPoints.Count)
                {
                    if (ARS.IsPointToPoint) break;
                    index %= _routeSpeedPoints.Count;
                }
                RouteSpeedPoint point = _routeSpeedPoints[index];
                if (point.Node < 0 || point.Node >= ARS.TrackPoints.Count) continue;
                string text = "#" + index + " " + point.TargetSpeedMps.ToString("0.0", CultureInfo.InvariantCulture) + "mps~n~r"
                    + point.Radius.ToString("0", CultureInfo.InvariantCulture) + "m";
                ARS.DrawText(ARS.TrackPoints[point.Node].Position + Vector3.WorldUp * 2.5f,
                    text, i == 0 && _activeRouteSpeedPoint != null ? Color.LimeGreen : Color.White, 0.5f);
            }
        }

        public void DrawRouteSpeedSubtitle()
        {
            string first = "~w~" + _routeSpeedOffsetMph.ToString("0.0", CultureInfo.InvariantCulture);
            string colour = _projectionWidenMeters > 0f && _projectionWidenMeters < RouteSpeedLearnWidenMetres ? "~g~"
                : _projectionWidenMeters >= RouteSpeedLearnWidenMetres ? "~r~" : "~w~";
            string second = colour + _projectionWidenMeters.ToString("0.0", CultureInfo.InvariantCulture) + " m";
            string third = _activeRouteSpeedPoint == null
                ? "~w~route points: none"
                : "~w~#" + _routeSpeedCursor + " n" + _activeRouteSpeedPoint.Node
                    + " r" + _activeRouteSpeedPoint.Radius.ToString("0", CultureInfo.InvariantCulture)
                    + " " + _activeRouteSpeedPoint.TargetSpeedMps.ToString("0.0", CultureInfo.InvariantCulture) + "mps";
            UI.ShowSubtitle(first + "~n~" + second + "~n~" + third, 50);
        }

        void DrawFrontSlipReadout()
        {
            float fwdSpeed = Vector3.Dot(Car.Velocity, Car.ForwardVector);
            float mph = ARS.MpsToMph(Math.Abs(fwdSpeed));

            string state = fwdSpeed <= 0f ? "inactive - stopped"
                : !_slip.Readable ? "no reading"
                : _slip.FrontPct < 0.5f ? "inactive - straight"
                : null;

            if (state != null)
            {
                UI.ShowSubtitle("Slip vs Allowed~n~" + state, 50);
                return;
            }

            float cap = _steerGrantedCeiling > 0.01f ? _steerGrantedCeiling : VehicleData.SteeringLock;

            UI.ShowSubtitle("Slip vs Allowed~n~x" + (_slip.FrontPct * 0.01f).ToString("0.0")
                + "   " + mph.ToString("0") + "mph"
                + "   cap " + cap.ToString("0.0") + "°", 50);
        }

        void DrawSlideAngleReadout()
        {
            int now = Game.GameTime;
            _slideSamples[_slideSampleIndex] = VehicleData.SlideAngle;
            _steerSamples[_slideSampleIndex] = Control.SteerDegrees;
            _slideSampleAt[_slideSampleIndex] = now;
            _slideSampleIndex = (_slideSampleIndex + 1) % SlideAverageSlots;
            if (_slideSampleCount < SlideAverageSlots) _slideSampleCount++;

            int windowMs = (int)(SlideAverageSeconds * 1000f);
            float slideSum = 0f;
            float steerSum = 0f;
            int used = 0;
            for (int i = 0; i < _slideSampleCount; i++)
            {
                if (now - _slideSampleAt[i] > windowMs) continue;
                slideSum += _slideSamples[i];
                steerSum += _steerSamples[i];
                used++;
            }

            float slide = used > 0 ? slideSum / used : VehicleData.SlideAngle;
            float steer = used > 0 ? steerSum / used : Control.SteerDegrees;
            UI.ShowSubtitle(slide.ToString("0.0") + "°~n~" + steer.ToString("0.0") + "°", 50);
        }

        public void DrawTrLateral()
        {
            float halfWidth = VehicleData.SlidingWidthMetres * 0.5f;
            float bodyLength = VehicleData.ModelDimensions.Y;
            Vector3 fwd = Car.ForwardVector;
            fwd.Z = 0f;
            fwd.Normalize();
            Vector3 back = -fwd;
            Vector3 right = Vector3.Cross(fwd, Vector3.WorldUp);
            Vector3 rear = Car.Position + Vector3.WorldUp * 0.1f + back * (bodyLength * 0.5f);
            float backCos = (float)Math.Cos(TRLateralAtSpeed * Math.PI / 180.0);
            float openSin = (float)Math.Sin(TRLateralAtSpeed * Math.PI / 180.0);
            float slideDeg = VehicleData.SlideAngle;
            if (float.IsNaN(slideDeg) || float.IsInfinity(slideDeg)) slideDeg = 0f;
            slideDeg = Math.Abs(slideDeg);
            float slideCos = (float)Math.Cos(slideDeg * Math.PI / 180.0);
            float slideSin = (float)Math.Sin(slideDeg * Math.PI / 180.0);
            for (int i = 0; i < 2; i++)
            {
                Vector3 side = i == 0 ? right : -right;
                Vector3 rearSide = rear + side * halfWidth;
                Vector3 frontSide = rearSide - back * bodyLength;
                ARS.DrawLine(rearSide, frontSide, Color.White);
                Vector3 hornDir = back * backCos + side * openSin;
                ARS.DrawLine(frontSide, frontSide + hornDir * bodyLength, Color.Yellow);
                Vector3 slideHornDir = back * slideCos + side * slideSin;
                ARS.DrawLine(frontSide, frontSide + slideHornDir * bodyLength, Color.Red);
            }
            float share = TRLateralAtSpeed > 0.01f ? slideDeg / TRLateralAtSpeed : 0f;
            UI.ShowSubtitle("~b~" + ((int)Math.Round(share * 100f)).ToString(CultureInfo.InvariantCulture) + "%", 50);
        }

        void DrawWheelSlip()
        {
            List<float> slips = ARS.WheelSideSlipAngles(Car);
            if (slips.Count < 4) return;

            float halfWidth = VehicleData.ModelDimensions.X * 0.5f;
            float halfBase = VehicleData.WheelBaseMetres * 0.5f;
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
        const float LaneStillMps = 0.2f;

        public static Color LaneMotionColor(LaneReason reason, float rateMps, int cornerDir)
        {
            if (reason == LaneReason.OwnLane) return Color.Gray;
            if (cornerDir == 0 || Math.Abs(rateMps) < LaneStillMps) return Color.DodgerBlue;
            return rateMps * cornerDir < 0f ? Color.Lime : Color.Yellow;
        }

        public static Color InputColour(float input)
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
            VehicleData.SlidingWidthMetres = ARS.SlidingBoundingBoxWidth(Car);

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

        void EnsureEngineRunning()
        {
            bool engineRunning = Function.Call<bool>((Hash)0xAE31E7DF9B5B132E, Car);
            if (Car.EngineHealth > 0f && !engineRunning) Function.Call(Hash.SET_VEHICLE_ENGINE_ON, Car, true, true, false);
        }

        public void ApplyInputs()
        {

            if (Driver.IsSittingInVehicle(Car))
            {
                UpdatePassengerSeat();
                EnsureEngineRunning();

                Car.HandbrakeOn = Control.HandBrakeUntilMs > Game.GameTime;

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

        void UpdatePowerCompensation()
        {

            float counter = 1f;

            if (ControlledByPlayer)
            {
                _overspeedPowerLevel = GlideCap(_overspeedPowerLevel, OverspeedThrottleLevel(),
                    OverspeedPlayerRecoveryPerSecond);
                counter *= _overspeedPowerLevel;
            }

            _powerMultiplier = counter;

            if (ControlledByPlayer && _overspeedPowerLevel < 0.99f && Game.GameTime > _powerCounterLoggedAt)
            {
                _powerCounterLoggedAt = Game.GameTime + 2000;
                ARS.Log(ARS.LogImportance.Info, "Player power cut " + _overspeedPowerLevel.ToString("0.00") + " (excess Gs " + VehicleData.OverspeedExcessGs.ToString("0.00") + ")");
            }

        }

        void ApplyPowerMultiplier()
        {

            float multiplier = _powerMultiplier;
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

        float SignedMetresBeyondTrackEdge()
        {
            return Math.Abs(Brain.CurrentPerception.CurrentLaneOffsetMetres) - CurrentTrackPoint.TrackHalfWidth;
        }

        void UpdateRivalInfo()
        {
            Brain.AvoidanceTarget = null;
            foreach (Rival r in Brain.Rivals)
            {
                r.Update(this);

                bool isAvoidanceCandidate = r.RelativePosition == RelativePos.Ahead && ARS.IsBetween(r.RouteGapAhead, 0f, SameSectionMaxGapMeters) && (ARS.IsBetween(r.FrontGapMetres, 0f, 3f) || ARS.IsBetween(r.TimeToReachSeconds, 0f, 5f));
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
            UpdateRaceOrderKey();
        }

        int WrappedNodeDelta(int fromNode, int toNode)
        {
            int delta = toNode - fromNode;
            if (ARS.IsPointToPoint) return delta;

            int count = ARS.TrackPoints.Count;
            if (delta > count / 2) delta -= count;
            else if (delta < -count / 2) delta += count;
            return delta;
        }

        void UpdateLapOdometer(int node)
        {
            int count = ARS.TrackPoints.Count;
            if (count <= 0) return;
            if (_lastLapNode < 0 || _lapStartNode < 0)
            {
                _lapOdometer = 0;
                _lastLapNode = node;
                _lapStartNode = node;
                _lapStartOdometer = 0;
                return;
            }

            _lapOdometer += WrappedNodeDelta(_lastLapNode, node);
            float previousPct = ARS.GetPercent(_lastLapNode, count);
            float currentPct = ARS.GetPercent(node, count);
            if (!ARS.IsPointToPoint && previousPct > 90f && currentPct < 10f)
            {
                _lapStartNode = 0;
                _lapStartOdometer = _lapOdometer;
            }

            _lastLapNode = node;
        }

        void UpdateCornerModeOdometer(int node)
        {
            if (!_cornerMode)
            {
                _cornerModeLastNode = -1;
                return;
            }

            if (_cornerModeLastNode >= 0)
                _cornerModeDistanceNodes += WrappedNodeDelta(_cornerModeLastNode, node);
            _cornerModeLastNode = node;
        }

        void UpdateRaceOrderKey()
        {
            RaceOrderKey = Lap * ARS.TrackPoints.Count + CurrentTrackPoint.Node;
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
            UpdateLapOdometer(closestPoint.Node);
            UpdateCornerModeOdometer(closestPoint.Node);
            Brain.CurrentPerception.CurrentLaneOffsetMetres = ARS.SignedLaneOffset(carPosition, CurrentTrackPoint.Position, CurrentTrackPoint.Direction);

            LookAheads.Clear();
            Vector3 velocity = Car.Velocity;
            float speed = velocity.Length();
            AlongTrackSpeed = Vector3.Dot(velocity, CurrentTrackPoint.Direction);

            int steerRef = (int)SteerLookaheadNodes(speed);
            int oneSec = (int)(speed);

            TrackPoint ResolveLookAhead(int offset)
            {
                int node = CurrentTrackPoint.Node + offset;
                if (ARS.IsPointToPoint) return ARS.TrackPoints[Math.Min(node, lastNode)];
                return ARS.TrackPoints[node % ARS.TrackPoints.Count];
            }

            LookAheads[LookAhead.SteerRef] = ResolveLookAhead(steerRef);
            LookAheads[LookAhead.OneSec] = ResolveLookAhead(oneSec);

            int nodeCount = ARS.TrackPoints.Count;
            int currentNode = CurrentTrackPoint.Node;
            float currentPct = ARS.GetPercent(currentNode, nodeCount);
            float previousPct = _previousNode >= 0 ? ARS.GetPercent(_previousNode, nodeCount) : 0f;
            bool wrappedStartLine = !ARS.IsPointToPoint && _previousNode >= 0 && previousPct > 90f && currentPct < 10f;
            UpdateLapRegistration(currentNode, nodeCount, currentPct, wrappedStartLine);

            UpdateRaceOrderKey();

            _previousNode = currentNode;

            Brain.CurrentPerception.CurveRadiusToFollowPoint = RouteRadiusSampled();
            UpdateApexLeapfrog();

            if (UpdateCornerRequirements() || _apexUpdateTick + _phaseOffsetMs < Game.GameTime)
            {
                _apexUpdateTick = Game.GameTime + 500;
                RefillApexQueue();
            }

            Brain.CurrentPerception.HighSpeedCurveRadius = ComputeRouteRadius((int)(speed * 0.5f), (int)(speed * 1.0f));
        }

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

        const float RouteRadiusFarEndSeconds = 2f;
        const float RouteRadiusSpanSeconds = 1f;

        const int RouteRadiusMeanCount = 2;

        float RouteRadiusWindow(float endSeconds, ref int nearNode, ref int midNode, ref int farNode)
        {
            int count = ARS.TrackPoints.Count;
            float speed = Car.Velocity.Length();
            int span = Math.Min((int)(speed * RouteRadiusSpanSeconds), count - 1);
            int end = Math.Min((int)(speed * Math.Max(endSeconds, 0f)), count - 1);
            int start = end - span;
            if (span < 8) return 999f;

            int carNode = CurrentTrackPoint.Node;
            float r = 999f;
            int tightestNode = -1;
            float[] tightest = new float[RouteRadiusMeanCount];
            for (int i = 0; i < tightest.Length; i++) tightest[i] = 999f;
            for (int offset = start; offset <= end; offset++)
            {
                int node = WrappedNode(carNode + offset);
                float radius = ARS.TrackPoints[node].SweepCurveRadius;
                if (radius < r)
                {
                    r = radius;
                    tightestNode = node;
                }
                if (radius >= tightest[RouteRadiusMeanCount - 1]) continue;
                int slot = RouteRadiusMeanCount - 1;
                while (slot > 0 && tightest[slot - 1] > radius)
                {
                    tightest[slot] = tightest[slot - 1];
                    slot--;
                }
                tightest[slot] = radius;
            }

            float sum = 0f;
            int used = 0;
            for (int i = 0; i < tightest.Length; i++)
            {
                if (tightest[i] >= 999f) break;
                sum += tightest[i];
                used++;
            }
            float bandRadius = used > 0 ? sum / used : r;

            nearNode = WrappedNode(carNode + start);
            midNode = tightestNode;
            farNode = WrappedNode(carNode + end);
            return bandRadius;
        }

        float NitrousRouteRadius()
        {
            int count = ARS.TrackPoints.Count;
            if (count < 10) return 999f;
            float speed = Car.Velocity.Length();
            int span = Math.Min(Math.Max((int)(speed * RouteRadiusSpanSeconds), 8), count - 1);
            int carNode = CurrentTrackPoint.Node;
            float r = 999f;
            for (int offset = -span; offset <= span; offset++)
            {
                float radius = ARS.TrackPoints[WrappedNode(carNode + offset)].SweepCurveRadius;
                if (radius < r) r = radius;
            }
            return r;
        }

        float RouteRadiusSampled()
        {
            int count = ARS.TrackPoints.Count;
            if (count < 10)
            {
                LiveRouteRadius = 999f;
                LiveRouteRadiusNearNode = -1;
                LiveRouteRadiusMidNode = -1;
                LiveRouteRadiusFarNode = -1;
                LiveSlidRadiusNearNode = -1;
                LiveSlidRadiusMidNode = -1;
                LiveSlidRadiusFarNode = -1;
                LiveSlidRadius = 999f;
                return 999f;
            }

            float r = 999f;
            if (ARS.RouteRadiusFar)
            {
                r = RouteRadiusWindow(RouteRadiusFarEndSeconds, ref LiveRouteRadiusNearNode, ref LiveRouteRadiusMidNode, ref LiveRouteRadiusFarNode);
            }
            else
            {
                LiveRouteRadiusNearNode = -1;
                LiveRouteRadiusMidNode = -1;
                LiveRouteRadiusFarNode = -1;
            }

            float slid = RouteRadiusWindow(ARS.SlidRadiusLeadSeconds, ref LiveSlidRadiusNearNode, ref LiveSlidRadiusMidNode, ref LiveSlidRadiusFarNode);
            LiveSlidRadius = slid;
            if (slid < r) r = slid;

            LiveRouteRadius = ARS.Clamp(r, 5f, 999f);
            return LiveRouteRadius;
        }

        int WrappedNode(int node)
        {
            int count = ARS.TrackPoints.Count;
            return ARS.IsPointToPoint
                ? (int)ARS.Clamp(node, 0, count - 1)
                : ((node % count) + count) % count;
        }
        const float ApexBufferSeconds = ARS.LanePrepSeconds;
        const float SecondaryApexMinSpeedGapMps = 5f;

        void UpdateApexLeapfrog()
        {
            int[] heldNodes = { NextApexNode, NextApexNode2, NextApexNode3 };
            float[] heldRadii = { NextApexRadius, NextApexRadius2, NextApexRadius3 };

            bool heldTableChanged = heldNodes.Any(node => node >= 0 && !ARS.Corners.Any(corner => corner.Node == node));
            bool lowSpeedInvalidation = heldNodes[0] >= 0 && Car.Velocity.Length() < NextApexSpeed * 0.5f;
            if (heldTableChanged || lowSpeedInvalidation)
            {
                for (int i = 0; i < heldNodes.Length; i++)
                {
                    if (heldNodes[i] >= 0) LogApexEvent("Plan", false, heldNodes[i], lowSpeedInvalidation ? "queue invalidated by low speed" : "queue invalidated by missing corner");
                    heldNodes[i] = -1;
                }
            }
            else
            {

                int shift = 0;
                while (shift < heldNodes.Length && heldNodes[shift] >= 0 && HasPassedApex(heldNodes[shift])) shift++;
                if (shift > 0 && ControlledByPlayer) Tips.ApexPassed(shift);
                if (shift > 0)
                {
                    for (int i = 0; i < shift; i++)
                        if (heldNodes[i] >= 0) LogApexEvent("Plan", false, heldNodes[i], "apex passed");
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

        bool UpdateCornerRequirements()
        {
            float speed = Math.Max(Car.Velocity.Length(), 1f);
            float forwardGs = VehicleData.GetLongitudinalGs(Car.ForwardVector);
            bool flipped = false;

            foreach (CornerPoint corner in ARS.Corners)
            {
                int entranceNode = corner.StartNode >= 0 ? corner.StartNode : OffsetCornerNode(corner.Node, -corner.EntranceNodes);
                int distance = ForwardNodeDistance(entranceNode);
                if (!TryGetCornerContext(corner.Node, out CornerContext context)) continue;

                if (HasPassedApex(corner.Node))
                {
                    _skippedApexes.Remove(corner.Node);
                }

                if (distance <= 0) continue;

                bool tight = corner.DetectedRadius < CornerTightRadiusMetres;

                float intended = ARS.CornerApexSpeed(corner, this);
                float timeToEntrance = distance / speed;
                float decel = Math.Max(BrakingDecelBase(corner.Node), 0.1f);
                float brakingDistance = (speed * speed - intended * intended) / (2f * decel);
                bool inBrakingRange = distance <= brakingDistance + BrakeHorizonMarginMetres;

                if (!tight && !context.RequiresBraking && timeToEntrance <= RequirementLookaheadSeconds)
                {
                    if (_skippedApexes.Add(corner.Node))
                    {
                        flipped = true;
                        ARS.Log(ARS.LogImportance.Info, "Queue skip " + Name + ": corner " + corner.Node + " ignored (wide "
                            + corner.DetectedRadius.ToString("0") + " m, no braking asked), the picker looks past it");
                        LogApexEvent("Plan", false, corner.Node, "wide, no braking asked");
                    }
                }

                if (!inBrakingRange) continue;

                float arrival = speed + forwardGs * Handling.Gravity * timeToEntrance * RequirementExtrapolationScale;
                float excess = arrival - intended;

                if (!context.RequiresBraking && inBrakingRange && excess > ARS.MphToMps(tight ? TightCornerArmExcessMph : WideCornerArmExcessMph))
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

            int first = 0;
            while (first < upcoming.Count && _skippedApexes.Contains(ARS.Corners[upcoming[first]].Node)) first++;
            if (first >= upcoming.Count) first = 0;

            if (upcoming.Count == 0)
            {
                if (NextApexNode >= 0)
                    ARS.Log(ARS.LogImportance.Info, "plan " + Name + ": refill found no corner ahead, kept head "
                        + NextApexNode + " (node " + CurrentTrackPoint.Node + " of " + count + ")");
                return;
            }

            List<int> selectedNodes = new List<int>();
            List<float> selectedRadii = new List<float>();
            if (first < upcoming.Count)
            {
                selectedNodes.Add(ARS.Corners[upcoming[first]].Node);
                selectedRadii.Add(ARS.Corners[upcoming[first]].DetectedRadius);
            }

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
                    float apexSpeed = ARS.CornerApexSpeed(ARS.Corners[upcoming[i]], this);
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

        bool CornerWorthHolding(int node, float radius, List<int> selectedNodes, List<float> selectedRadii)
        {
            int distance = ForwardNodeDistance(node);
            float candidateSpeed = ApexSpeedFor(node, radius);
            float nearestGap = float.MaxValue;
            float nearestSpeed = 999f;
            for (int i = 0; i < selectedNodes.Count; i++)
            {
                float gap = Math.Abs(distance - ForwardNodeDistance(selectedNodes[i]));
                if (gap < nearestGap)
                {
                    nearestGap = gap;
                    nearestSpeed = ApexSpeedFor(selectedNodes[i], selectedRadii[i]);
                }
            }
            if (nearestGap > Math.Max(5f, nearestSpeed * ApexBufferSeconds)) return true;
            return nearestSpeed - candidateSpeed >= SecondaryApexMinSpeedGapMps;
        }

        void CommitApexQueue(int[] nodes, float[] radii)
        {
            int previousHead = NextApexNode;
            NextApexNode = nodes[0];
            NextApexRadius = radii[0];
            NextApexSpeed = NextApexNode >= 0 ? ApexSpeedFor(NextApexNode, NextApexRadius) : 999f;
            NextApexNode2 = nodes[1];
            NextApexRadius2 = radii[1];
            NextApexSpeed2 = NextApexNode2 >= 0 ? ApexSpeedFor(NextApexNode2, NextApexRadius2) : 999f;
            NextApexNode3 = nodes[2];
            NextApexRadius3 = radii[2];
            NextApexSpeed3 = NextApexNode3 >= 0 ? ApexSpeedFor(NextApexNode3, NextApexRadius3) : 999f;
            if (NextApexNode != previousHead)
            {
                if (NextApexNode >= 0)
                    LogApexEvent("Plan", true, NextApexNode, previousHead >= 0 ? "from " + previousHead : null);
                else if (previousHead >= 0)
                    LogApexEvent("Plan", false, previousHead, "queue emptied");
            }

            if (NextApexNode >= 0)
            {

                CornerPoint original = ARS.Corners.FirstOrDefault(c => c.Node == NextApexNode);
                _nextApexCorner = original;

                if (original != null && original.SurfaceGrip < 1f && _planLogApexNode != NextApexNode)
                {
                    _planLogApexNode = NextApexNode;
                    float grip = original.SurfaceGrip;
                    float decel = BrakingDecelBase(NextApexNode);
                    ARS.Log(ARS.LogImportance.Info, "Plan " + Name + ": apex " + NextApexNode + " grip " + grip.ToString("0.00")
                        + ", " + NextApexSpeed.ToString("0.0") + " m/s vs " + ApexSpeedWithDownforce(original.DetectedRadius).ToString("0.0") + " on tarmac"
                        + ", brake " + decel.ToString("0.0") + " vs " + (decel / grip).ToString("0.0"));
                }
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

            int count = ARS.TrackPoints.Count;
            if (count <= 0) return false;
            int startNode = _lapStartNode >= 0 ? _lapStartNode : CurrentTrackPoint.Node;
            int delta = apexNode - startNode;
            delta %= count;
            if (delta < 0) delta += count;
            if (_lapOdometer - _lapStartOdometer < delta) return false;

            return ForwardNodeDistance(apexNode) > count / 2;
        }

        // One writer, so the ramp and the learner that grades it cannot aim at different nodes.
        int BrakeTargetDistance(int apexNode, int fallbackNode)
        {
            int targetNode = BrakeTargetBaseNode(apexNode, fallbackNode);
            if (targetNode < 0) targetNode = fallbackNode;
            return ForwardNodeDistance(targetNode);
        }

        float ApexBrakingSpeed(int apexNode, float apexSpeed)
        {
            if (apexNode < 0) return 999f;
            float velTarget = apexSpeed;

            CornerPoint corner = ARS.Corners.FirstOrDefault(c => c.Node == apexNode);
            int fallbackNode = CornerEntranceNode(corner, apexNode);

            int targetDistance = BrakeTargetDistance(apexNode, fallbackNode);
            int apexDistance = ForwardNodeDistance(apexNode);

            float rawDistance = !ARS.IsPointToPoint && targetDistance > apexDistance ? 0f : targetDistance;
            rawDistance -= FullBrakeSpeedErrorMps;
            if (rawDistance < 0f) rawDistance = 0f;

            float decel = BrakingDecel(apexNode, rawDistance);
            float spd = (float)Math.Sqrt((velTarget * velTarget) + (2f * decel * rawDistance));
            if (float.IsNaN(spd) || float.IsInfinity(spd)) spd = 999f;
            return spd;
        }

        public float BrakingDecel(int apexNode, float spanMeters)
        {
            float decel = BrakingDecelBase(apexNode) + Handling.Gravity * BrakingGradeSine(spanMeters);
            return Math.Max(decel, 0.1f);
        }

        float BrakingDecelBase(int apexNode)
        {
            float brakingAbility = Math.Min(Handling.BrakingAbility * 4, VehicleData.CurrentMechanicalGrip);

            CornerPoint corner = apexNode >= 0 ? ARS.Corners.FirstOrDefault(c => c.Node == apexNode) : null;
            float surfaceGrip = corner != null ? corner.SurfaceGrip : 1f;
            float decel = brakingAbility * surfaceGrip * Handling.Gravity * EffectiveBrakeFactor(apexNode);
            if (ActiveManeuver.Type == ManeuverType.Yield) decel *= 0.5f;
            return decel;
        }

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
                sum += ARS.TrackPoints[sample].SlopeSineTimes90;
            }
            return sum / samples * ElevationToSlopeSine;
        }

        // The tyre root over-asks on a wide radius: apex physics reads pure below the cap radius, and above it
        // the speed keeps rising with radius but in dialled steps - the cap radius anchors the pure apex speed
        // and RouteSpeedRiseExponent is the exponent of the rise (0.5 = the tyre root's own steps, lower =
        // smaller steps, 0 = flat). 999 m stays the open-road code that never damps.
        const float RouteSpeedCapRadiusM = 100f;
        const float RouteSpeedRiseExponent = 0.125f;
        const float RouteRadiusOpenM = 999f;

        float RouteIdealSpeedForRadius(float r)
        {
            float root = ApexSpeedWithDownforce(r);
            if (r >= RouteRadiusOpenM) return root;
            if (r <= RouteSpeedCapRadiusM) return root;
            return ApexSpeedWithDownforce(RouteSpeedCapRadiusM) * (float)Math.Pow(r / RouteSpeedCapRadiusM, RouteSpeedRiseExponent);
        }

        float RouteSurfaceGrip()
        {
            int count = ARS.TrackPoints.Count;
            if (count == 0) return 1f;

            int here = CurrentTrackPoint.Node;
            float known = here >= 0 && here < count ? ARS.TrackPoints[here].SurfaceGrip : 1f;
            float Grip(int node) => node >= 0 && node < count ? ARS.TrackPoints[node].SurfaceGrip : known;
            float grip = Math.Min(Grip(LiveRouteRadiusNearNode), Grip(LiveRouteRadiusFarNode));
            grip = Math.Min(grip, Math.Min(Grip(LiveRouteRadiusMidNode), Grip(LiveSlidRadiusNearNode)));
            grip = Math.Min(grip, Math.Min(Grip(LiveSlidRadiusMidNode), Grip(LiveSlidRadiusFarNode)));
            return grip;
        }

        float ApexSpeedFor(int node, float fallbackRadius)
        {
            CornerPoint corner = node >= 0 ? ARS.Corners.FirstOrDefault(c => c.Node == node) : null;
            return corner != null ? ARS.CornerApexSpeed(corner, this) : ApexSpeedWithDownforce(fallbackRadius);
        }

        float ApexSpeedWithDownforce(float r)
        {

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

            return ARS.Circumradius3D(
                ARS.TrackPoints[routeStartNode].Position,
                ARS.TrackPoints[routeEndNode].Position,
                ARS.TrackPoints[routeMidNode].Position);
        }

        void ProcessTimedAI()
        {
            int now = Game.GameTime;

            if (_halfSecondDueMs + _phaseOffsetMs < now)
            {
                _halfSecondDueMs = now + 500 + (int)ARS.Remap(Car.Velocity.Length(), 0, 100, -250, 250, true);
            }

            if (_rivalTick + _phaseOffsetMs < now)
            {
                _rivalTick = now + RivalRefreshMs;
                if (!ControlledByPlayer && BaseBehavior == RacerBaseBehavior.Race && ARS.Racers.Count >= 1) UpdateRivals();
            }

            if (_oneSecondDueMs + _phaseOffsetMs < now)
            {
                _oneSecondDueMs = now + 1000;

                if (!ControlledByPlayer)
                {
                    if (BaseBehavior == RacerBaseBehavior.Race && ARS.Racers.Count >= 1)
                    {
                        ConsiderCrowdCard();
                    }

                    if (ARS.Autofix == AutofixMode.AutoRepair && !IsRaceFit(Car))
                    {

                        ARS.Log(ARS.LogImportance.Info, "AutoRepair " + Name + ": body " + Car.BodyHealth.ToString("0") + ", engine " + Car.EngineHealth.ToString("0"));
                        Car.Repair();
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

                if (BaseBehavior == RacerBaseBehavior.Race && Brain.CurrentPerception.HighSpeedCurveRadius > RocketBoostMinimumCurveRadius && Math.Abs(VehicleData.SlideAngle) < 0.5f && Math.Abs(Control.Throttle) > 0.9f) Function.Call((Hash)0x81E1552E35DC3839, Car, true);

                if (Function.Call<bool>((Hash)0x3D34E80EED4AE3BE, Car) && Control.Brake > 0.1f) Function.Call((Hash)0x81E1552E35DC3839, Car, false);

                if (!ControlledByPlayer && ARS.Autofix == AutofixMode.Invincible && Function.Call<bool>(Hash._IS_VEHICLE_DAMAGED, Car))
                {

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

            if (BaseBehavior == RacerBaseBehavior.GridWait && Control.HandBrakeUntilMs < now) Control.HandBrakeUntilMs = now + (100 * ARS.GetRandomInt(2, 6));

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

                float t = ARS.Clamp((PressureProximityRange - closestDistance) / (PressureProximityRange - 20f), 0f, 1f);
                targetPressure = Aggression * t;
            }

            if (targetPressure > Pressure)
                Pressure = Math.Min(Pressure + PressureRisePerSecond * TickScale, targetPressure);
            else
                Pressure = Math.Max(Pressure - PressureFallPerSecond * TickScale, targetPressure);

            Pressure = ARS.Clamp(Pressure, 0f, PressureRange);
        }

        void UpdateDNFCheck()
        {
            if (IsDNF)
            {
                Control.HandBrakeUntilMs = Game.GameTime + 1000;
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

            if (SignedMetresBeyondTrackEdge() > 0f)
            {
                if (_stuckOffTrackSince == 0) _stuckOffTrackSince = now;
            }
            else _stuckOffTrackSince = 0;

            bool stationary = _stuckStationarySince != 0 && now - _stuckStationarySince >= RecoveryTriggerMs;
            bool offTrack = _stuckOffTrackSince != 0 && now - _stuckOffTrackSince >= RecoveryTriggerMs;

            if (stationary) StartStuckRecovery(now, StuckPhase.Reverse);
            else if (offTrack) StartStuckRecovery(now, StuckPhase.Drive);
        }

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

            if (phase == StuckPhase.Drive)
            {
                int entryGear = Function.Call<int>((Hash)CurrentDriveGearHash, Car);
                int entryGearCount = Function.Call<int>((Hash)MaxDriveGearHash, Car);
                float entryThrust = VehicleMemory.GetInitialDriveForce(Car) * KickGearRatio(Math.Max(entryGear, 1), entryGearCount) / 2f;
                float entryBudget = VehicleData.CurrentMechanicalGrip * KickWheelLoadShare * KickGripMargin;
                ARS.Log(ARS.LogImportance.Info, "Kick " + Name + ": recovery drive — bearing " + _steerAimBearingDegrees.ToString("0") + " deg, lock " + VehicleData.SteeringLock.ToString("0") + " deg, bias " + VehicleMemory.GetDriveBiasFront(Car).ToString("0.00") + ", close " + KickBearingCloseSeconds().ToString("0.0") + " s, forward " + ARS.MpsToMph(ARS.GetForwardSpeed(Car)).ToString("0") + " mph, gear " + entryGear + "/" + entryGearCount + ", thrust " + entryThrust.ToString("0.00") + ", budget " + entryBudget.ToString("0.00"));
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

        float NearestEdgeLane(float steerRefOffset, float laneLimit)
        {
            return steerRefOffset >= 0f ? laneLimit : -laneLimit;
        }

        float KickBearingCloseSeconds()
        {
            if (float.IsNaN(VehicleData.SteeringLock) || float.IsInfinity(VehicleData.SteeringLock)) return 0f;
            float lockRad = ARS.DegToRad(VehicleData.SteeringLock);
            float yawDegPerSecAtLock = ARS.RadToDeg(ARS.GetForwardSpeed(Car) * (float)Math.Tan(lockRad) / Math.Max(VehicleData.WheelBaseMetres, 0.1f));
            return Math.Abs(_steerAimBearingDegrees) / Math.Max(yawDegPerSecAtLock, 1f);
        }

        static float KickGearRatio(int gear, int maxGear)
        {
            if (maxGear < 2 || gear >= maxGear) return 0.9f;
            if (gear == 1) return 1.0f / 0.3f;
            float first = 1.0f / 0.3f;
            double baseRatio = Math.Pow((1.0 / Math.Pow(1.1, 0.5 * (maxGear - 1) * (maxGear - 2))) * first, 1.0 / (maxGear - 1));
            return (float)(0.9 * Math.Pow(baseRatio, maxGear - gear) * Math.Pow(1.1, 0.5 * (maxGear - gear) * (maxGear - gear - 1.0)));
        }

        bool GearCanKick()
        {
            int currentGear = Function.Call<int>((Hash)CurrentDriveGearHash, Car);
            int maxGear = Function.Call<int>((Hash)MaxDriveGearHash, Car);
            if (currentGear < 1 || maxGear < 2) return false;
            float thrustPerWheel = VehicleMemory.GetInitialDriveForce(Car) * KickGearRatio(currentGear, maxGear) / 2f;
            return thrustPerWheel > VehicleData.CurrentMechanicalGrip * KickWheelLoadShare * KickGripMargin;
        }
        static readonly bool KickCardEnabled = false;

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

            string foldReason = rotated ? "rotated" : "window spent";
            ARS.Log(ARS.LogImportance.Info,
                "Kick " + Name + ": folded " + foldReason + " after " + (now - _kickPlayingSince) + " ms, ask " + _kickAskDegrees.ToString("0") + " deg, slip " + ARS.MaxWheelSlip(Car).ToString("0.00"));
            _kickPlaying = false;
            _kickCooldownUntil = now + KickCooldownMs;
        }

        void TryArmThrottleKick()
        {
            if (ControlledByPlayer || ARS.GetForwardSpeed(Car) <= 0f
                || VehicleMemory.GetDriveBiasFront(Car) > KickMaxDriveBiasFront
                || Game.GameTime < _kickCooldownUntil || !GearCanKick()
                || Math.Abs(VehicleData.SlideAngle) >= TRLateralAtSpeed * CountersteerBlendStart)
            {
                _kickArmSince = 0;
                return;
            }

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
            ARS.Log(ARS.LogImportance.Info, "Kick " + Name + ": playing, ask " + _steerPreLimitDegrees.ToString("0") + " deg past granted " + _steerGrantedCeiling.ToString("0") + " deg at " + ARS.MpsToMph(Car.Velocity.Length()).ToString("0") + " mph, slip " + ARS.MaxWheelSlip(Car).ToString("0.00"));
        }

        bool RecoveryExitHolds()
        {
            bool onTrack = Math.Abs(Brain.CurrentPerception.CurrentLaneOffsetMetres) <= CurrentTrackPoint.TrackHalfWidth + VehicleData.SlidingWidthMetres * 0.5f;
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
            Control.BrakeReasonCap = 0f;
            Control.ThrottleReason = ThrottleReason.StuckRecovery;
            Control.ThrottleReasonCap = 0f;
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
            float laneOffset = Math.Max(CurrentTrackPoint.TrackHalfWidth - VehicleData.SlidingWidthMetres * 0.5f, 0f) * side;
            Car.Position = CurrentTrackPoint.Position + right * laneOffset + new Vector3(0f, 0f, 0.5f);
            Car.Heading = direction.ToHeading();
            Car.Velocity = direction * ARS.MphToMps(10f);
            FinishStuckRecovery();
        }

        void UpdatePerceivedGrip()
        {

            float handlingGrip = Function.Call<float>((Hash)0xA132FB5370554DB0, Car);
            handlingGrip = ARS.Clamp(handlingGrip, 0.1f, 100f);
            handlingGrip /= 1f + 0.035f * Handling.Downforce;

            GroundGripMultiplier = ARS.MeanWheelGripMultiplier(Car);

            float forwardMs = ARS.GetForwardSpeed(Car);
            float routeRadius = Brain.CurrentPerception.CurveRadiusToFollowPoint;
            float lateralMs = 0f;
            if (routeRadius > 1f && !float.IsNaN(routeRadius) && !float.IsInfinity(routeRadius))
                lateralMs = (forwardMs * forwardMs) / routeRadius;
            float dfGs = ARS.GetDownforceGsAtSpeed(this, forwardMs, lateralMs);

            VehicleData.BaseMechanicalGrip = handlingGrip;
            VehicleData.DownforceGripBonusGs = dfGs;
            VehicleData.CurrentMechanicalGrip = (VehicleData.BaseMechanicalGrip + VehicleData.DownforceGripBonusGs) * GroundGripMultiplier;

            if (!_gripLogged)
            {
                _gripLogged = true;
                ARS.Log(ARS.LogImportance.Info, "Grip for " + Car.DisplayName + ": base " + VehicleData.BaseMechanicalGrip
                    + ", current " + VehicleData.CurrentMechanicalGrip + " (steer cap k " + SteerReductionPerMps + ")");
            }

            if (Game.GameTime - _lastGsCheck >= 333)
            {
                _lastGsCheck = Game.GameTime;

                if (ARS.OverspeedCutPerTenthG > 0f)
                {
                    List<float> wheelPowers = ARS.WheelPowers(Car);
                    float wheelGs = 0f;
                    foreach (float p in wheelPowers) wheelGs += p;
                    float measuredGs = VehicleData.GetLongitudinalGs(Car.ForwardVector);
                    float excess = measuredGs - wheelGs - 0.1f;
                    VehicleData.OverspeedExcessGs = excess;

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

            UpdateRivalInfo();
        }

        public void Delete()
        {

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
