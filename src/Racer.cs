// Lane law: one line in two states. OUT names the outer edge at a fixed 6 m/s between the 300 m window and the gate six
// track widths out; the base state names the inside at 10 x track width / radius, and holds to a quarter of a metre of lead
// while the car still leans 10 deg out of the corner. The mover shapes every move: 40 m/s^2 and 24 m/s, halved outward.
// Corridor, walls, avoidance and recovery write the same request, last writer winning.
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

        int _lastGsCheck = 0;
        bool _gripLogged = false;

        float _restHeightAboveGround = -1f;

        public TrackPoint CurrentTrackPoint = new TrackPoint();
        public float AlongTrackSpeed = 0f;
        public int RaceProgress = 0;

        public int LiveRouteCrestNode = -1;
        public int LiveRouteCrestFarNode = -1;

        public int LiveRouteRadiusNearNode = -1;
        public int LiveRouteRadiusMidNode = -1;
        public int LiveRouteRadiusFarNode = -1;

        public float LiveRouteRadius = -1f;

        public enum LookAhead { SteerRef, QuarterSec, HalfSec, ThreeQuarterSec, OneSec, OneHalfSec, TwoSec };
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

        int _halfSecondTick = 0;
        int _oneSecondTick = 0;
        int _phaseOffsetMs = 0;

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
        public float RouteLookaheadSizeSeconds = 2.0f;

        const float SteerPreviewSeconds = 0.5f;
        const float SteerLookaheadMinNodes = 8f;
        public const float SteerLookaheadMaxNodes = 90f;

        // How far ahead the car is aimed: half a second of travel, floored and capped, read as a NODE count by the aim and by
        // the debug ribbon, so the two ends agree with each other. Nodes are at most 1.5 m apart, so the same speed aims
        // further on a coarsely sampled route: the number is nodes, not metres, and always was.
        public static float SteerLookaheadNodes(float speedMps)
        {
            return ARS.Clamp(speedMps * SteerPreviewSeconds, SteerLookaheadMinNodes, SteerLookaheadMaxNodes);
        }

        const float GsPreviewBlend = 0.5f;

        float _avoidLeftWall = 0f;
        float _avoidRightWall = 0f;
        float _wallRoadWide = 0f;

        public float AvoidLeftWall => _avoidLeftWall;
        public float AvoidRightWall => _avoidRightWall;
        public float WallRoadWide => _wallRoadWide;
        bool _leftWallHeld;
        bool _rightWallHeld;

        public bool LeftWallHeld => _leftWallHeld;
        public bool RightWallHeld => _rightWallHeld;
        int _activeRivalWallCount = 0;
        bool _avoidWallsInitialized = false;

        float _carriedLane = 0f;

        bool _laneOvershoot;

        float _pursuitRawBearingDeg;
        float _pursuitAimDistanceM;

        float _pursuitAimLateralM;

        int _lineBiasNode = -1;
        float _lineBiasMeters;

        int _lineCornerNode = -1;

        int _laneCrossLoggedAt;

        float _leanApproachMps;
        float _laneRequest = 0f;
        LaneReason _laneRequestReason = LaneReason.OwnLane;

        bool _laneRequestOutward;
        int _laneCornerDir;
        float _laneRequestPrev = 0f;
        float _laneRate = 0f;
        float _laneMoverLoggedAt = 0f;
        int _cornerLineLoggedAt = 0;
        bool _laneSnap = false;
        bool _laneOverride = false;
        public float CarriedLane { get { return _carriedLane; } }

        public int LineCornerNode { get { return _lineCornerNode; } }
        public LaneReason LaneRequestReason { get { return _laneRequestReason; } }
        public float LaneRequest { get { return _laneRequest; } }
        public float LaneRate { get { return _laneRate; } }
        public int LaneCornerDir { get { return _laneCornerDir; } }

        float _cornerSpd = 999f;

        int _divebombApexNode = -1;
        int _defendApexNode = -1;

        const float RouteStraightSagittaShare = 0.002f;

        const float RouteBrakeCap = 0.2f;

        const float GsLaneThrottleMaxFloor = 1f;
        // At the outer edge the floor keeps half a pedal: lifting there is the one thing that pulls a car further out.
        const float GsLaneOutsideFloor = 0.5f;

        const float ProjectionInputPerSecond = 2f;
        const float ProjectionBrakeFloor = 0.5f;

        const float OffTrackSpeedSlackMph = 40f;
        const float OffTrackSpeedFloorMph = 20f;

        const float OffshootMinRouteAngleDeg = 2f;

        const float OffshootMinSpeedMph = 20f;

        const float FullPedalSpeedErrorMps = 1f;

        const float BrakeLearnerBarMps = FullPedalSpeedErrorMps * BrakeErrorMultiplier;

        const float BrakeErrorMultiplier = 3f;

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

        float _slidePriority = 0f;
        float _steerPursuitDeg = 0f;
        float _steerAimBearingDegrees = 0f;
        float _steerAimCurvature = 0f;

        float _steerPreLimitDegrees = 0f;

        float _finalCeilingDeg = 0f;

        float _chasingClampDeg = float.NaN;

        float _clampTargetDeg = float.NaN;

        float _slipFactor = 1f;

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
        bool _projectionWidening = false;
        float _projectionOutsideSeconds = 0f;

        Color _projectionPathColor = Color.Blue;

        bool _gsFloorActiveThisTick = false;

        bool _offtrackThrottleHeld = false;
        int _offshootLogAt = 0;

        const float BrakeFactorSeed = 0.8f;

        const int BrakeFactorSeedJitterHundredths = 5;

        const int BrakeConfidenceJitterHundredths = 5;
        float _brakeConfidenceBias;

        const float CornerTightRadius = 50f;

        const float RequirementExtrapolationScale = 0.9f;
        const float PositionExcessMph = 10f;
        const float BrakeExcessMph = 25f;

        const float BrakeHorizonMargin = 40f;

        const bool BrakeLearningApplies = true;
        readonly Dictionary<int, CornerContext> _cornerContexts = new Dictionary<int, CornerContext>();
        int _cornersRevisionSeeded = -1;

        readonly HashSet<int> _skippedApexes = new HashSet<int>();
        float _brakeSampleMaxSlideDeg = 0f;
        int _brakeSampleApexNode = -1;
        bool _brakeSampleWasAirborne = false;

        float _brakeSampleEntryMetres = 0f;
        bool _brakeSampleEntryMeasured = false;
        bool _brakeSampleSawAboveApex = false;

        float _brakeSampleApexSpeed = 0f;
        int _brakeSampleBelowTicks = 0;
        bool _brakeSampleEntryHasCandidate = false;

        bool _brakeSampleFloorAtEntrance = false;
        bool _brakeSampleEntranceCrossed = false;

        const float BrakeApexNearMetres = 50f;
        float _brakeSampleApexGap = 0f;
        int _brakeLastApexAnnounced = -1;
        float _brakeSampleApexClosest = float.MaxValue;
        int _brakeOverNextSampleAt = 0;

        int _brakeCommitApexNode = -1;
        int _brakeCommitAtGameTime = 0;
        float _brakeCommitMaxSlideDeg = 0f;
        bool _brakeCommitWasAirborne = false;
        float _brakeCommitEntryMetres = 0f;
        bool _brakeCommitEntryMeasured = false;
        bool _brakeCommitSawAboveApex = false;
        bool _brakeCommitFloorAtEntrance = false;
        bool _brakeCommitEntranceCrossed = false;

        const float BrakeSkidBrakeShare = 0.25f;
        const float BrakeSkidFactorStep = 0.1f;
        const float BrakeSkidPeakMultiple = 2.5f;
        const float BrakeCommitDelaySeconds = 0.75f;
        const float BrakeMinFactor = 0.5f;

        const float BrakeEntryFactorPerMetre = 0.02f;
        const float BrakeEntryMaxStepPerEvent = 0.25f;

        const float BrakeFloorLatchSeconds = 1f;

        const float BrakeOverAllowanceShare = 1.5f;
        const int BrakeOverSampleMs = 100;
        const float BrakeMaxFactor = 1.5f;
        public float BrakeFactorForApex(int apexNode) => ARS.BrakeLearning && TryGetCornerContext(apexNode, out CornerContext context) ? context.BrakeFactor : BrakeFactorSeed;
        float _divebombBrakeBonus;

        bool TryGetCornerContext(int apexNode, out CornerContext context)
        {
            SyncCornerContexts();
            return _cornerContexts.TryGetValue(apexNode, out context);
        }

        bool CornerRequiresBraking(int apexNode) => apexNode >= 0 && TryGetCornerContext(apexNode, out CornerContext context) && context.RequiresBraking;

        // A corner that asks for nothing gets no card: no draft, no defence, no concession. Read from the apex speed the
        // picker already worked out against the speed we carry now, so it answers on approach. The braking latch cannot:
        // it only trips once we are already inside braking range and arriving too fast, which is most of the way in.
        bool CornerAsksForBraking() => Brain.Corner != null && Brain.Corner.Speed < Car.Velocity.Length();

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

        float SeededBrakeFactor() => BrakeFactorSeed + _brakeConfidenceBias + ARS.GetRandomInt(-BrakeFactorSeedJitterHundredths, BrakeFactorSeedJitterHundredths + 1) / 100f;

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

        const float DivebombReleaseGapMeters = 40f;

        // How much later than me a rival may still reach the corner entrance and remain worth defending against.
        const float DefendLaterMarginSeconds = 1f;
        // A rival this close is worth defending against even if it is not closing: 20 metres plus both cars' length.
        const float DefendCloseBehindMeters = 20f;
        const float DivebombReleaseAngleDegrees = 25f;
        const int CardMinDwellMs = 1000;
        const int CardCooldownMs = 1000;
        public const float ChillStandoffDistance = 20f;
        const float ChillThrottleCap = 0.5f;

        const float RivalLiftMeters = 3f;
        const float RivalLiftFloor = 0.1f;
        const float ChillStandoffEscapeMargin = 2f;
        const float ChillMinSpeedMph = 30f;

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
        const float SideBySideHeadingShare = 0.5f;
        // Two cars are only in the same race for space if they are at the same level: on a figure-eight the car on the bridge
        // passes a couple of metres above the one underneath, and proximity alone reads that as a car alongside.
        const float RivalHeightGateMeters = 2f;
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

            if (ActiveManeuver.Type != ManeuverType.Yield) _cardCooldownUntil[(int)ActiveManeuver.Type] = Game.GameTime + CardCooldownMs;
            ActiveManeuver.Type = ManeuverType.None;
            ActiveManeuver.Target = null;
        }

        public float DivebombBrakeBonus => _divebombBrakeBonus;
        int _nitrousActiveUntil = 0;
        int _nitrousGsLoggedAt = 0;
        int _nitrousLapUsed = -1;
        int _nitroRefusalLoggedAt = 0;
        float _powerCounter = 1f;
        float _overspeedPowerLevel = 1f;
        float _countersteerAssistDeg = 0f;
        int _cornerLiftApex = -1;
        float _defendLaneOffset = 0f;
        float _avoidHoldLane = 0f;
        bool _avoidHoldActive = false;

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

            Handling.LateralTractionCurve = ARS.RadToDeg(VehicleMemory.GetLateralTraction(Car));
            if (Handling.LateralTractionCurve < 1 || Handling.LateralTractionCurve > 100) Handling.LateralTractionCurve = 22;
            ARS.Log(ARS.LogImportance.Info, "TRlat for " + Car.DisplayName + ":" + Handling.LateralTractionCurve + "º");

            Handling.BrakingAbility = Car.MaxBraking;
            Handling.EstimatedTopSpeed = ARS.EngineTopSpeed(Car);

            ARS.Log(ARS.LogImportance.Info, "Top speed " + Car.DisplayName + ": geared "
                + Function.Call<float>(Hash._0x53AF99BAA671CA47, Car).ToString("0.0") + " m/s, estimate "
                + ARS.MpsToMph(Handling.EstimatedTopSpeed).ToString("0") + " mph");
            Handling.Acceleration = Function.Call<float>(Hash.GET_VEHICLE_ACCELERATION, Car);

            VehicleData.SteeringLock = ARS.RadToDeg(VehicleMemory.GetSteerLock(Car));
            if (VehicleData.SteeringLock < 1 || VehicleData.SteeringLock > 100) VehicleData.SteeringLock = 40;
            ARS.Log(ARS.LogImportance.Info, "Steerlock for " + Car.DisplayName + ":" + VehicleData.SteeringLock + "º");

            VehicleData.WheelBase = ARS.GetWheelBase(Car);

            _brakeConfidenceBias = ARS.GetRandomInt(-BrakeConfidenceJitterHundredths, BrakeConfidenceJitterHundredths + 1) / 100f;
            ARS.Log(ARS.LogImportance.Info, "Brake confidence bias " + Name + ": " + _brakeConfidenceBias.ToString("0.00"));
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

            float livePace = ARS.ComputePaceIndex(modelTopSpeedMph, modelGrip, modelAccel, modelElectric);
            if (float.IsNaN(livePace) || float.IsInfinity(livePace)) ARS.ModelPaceIndexCache.TryGetValue(Car.Model.Hash.ToString(), out livePace);
            VehicleData.PowerScale = livePace;
            VehicleData.TextPerformanceIndex = VehicleData.PowerScale.ToString("0.0");

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

            Vector3 courseDir = Car.ForwardVector;
            if (Car.Velocity.LengthSquared() > 0.01f) courseDir = Car.Velocity.Normalized;

            float carHalfWidth = VehicleData.BoundingBox * 0.5f;

            float steerRefOffset = ARS.SignedLaneOffset(Car.Position, CurrentTrackPoint.Position, CurrentTrackPoint.Direction);

            float laneRequest = steerRefOffset;
            LaneReason laneReason = LaneReason.OwnLane;

            AdvancePastApex();
            CornerPoint lineCorner = NearestApexAhead();
            // The one bound every writer obeys: half the road, less the car's half width, less the buffer. Inside the corner the
            // car's own width is the whole buffer, so the lane may ask for the inner extreme with its edge on the line; on the
            // way in the half metre still applies.
            float laneLimit = Math.Max(roadWide - carHalfWidth - (IsWithinCorner(lineCorner) ? 0f : EdgeEmptySpaceMeters), 0f);
            if (Lap > 0 && TryComputeCurveLean(lineCorner, steerRefPoint, speedMps, steerRefOffset, laneLimit, roadWide * 2f, out float curveLean, out bool leanOutward))
            {
                laneRequest = curveLean;
                laneReason = LaneReason.CurveLean;
                _laneRequestOutward = leanOutward;
            }
            if (ActiveManeuver.Type == ManeuverType.DefendLane && MetresToNode(_defendApexNode) > OutsideGateMeters(steerRefPoint))
            {
                laneRequest = ARS.Clamp(_defendLaneOffset, -laneLimit, laneLimit);
                laneReason = LaneReason.CommitCard;
            }
            float avoidAheadLane = ComputeAvoidanceLane(roadWide);
            if (avoidAheadLane != 0f) { laneRequest = avoidAheadLane; laneReason = LaneReason.Avoidance; }
            if (_stuckPhase == StuckPhase.Drive && OutOfTrackDistance() > 0f) { laneRequest = NearestEdgeLane(steerRefOffset, laneLimit); laneReason = LaneReason.RecoveryEdge; }
            float walledLane = ApplyRivalWalls(laneRequest, roadWide);
            if (walledLane != laneRequest) laneReason = LaneReason.WallClamp;
            laneRequest = walledLane;
            if (ARS.DebugToggles[Options.LockLaneCentre]) { laneRequest = LaneLockTestOffsetMeters; laneReason = LaneReason.DebugLock; }
            _laneRequest = laneRequest;
            _laneRequestReason = laneReason;
            UpdateLaneMover(steerRefOffset, laneLimit);
            _laneOverride = laneReason != LaneReason.OwnLane || Math.Abs(_carriedLane - steerRefOffset) > LaneOverrideHoldMeters;

            Vector3 steerRight = Vector3.Cross(steerRefPoint.Direction, Vector3.WorldUp).Normalized;
            float offsetAtProjection = ARS.SignedLaneOffset(ProjectAhead(SteerPreviewSeconds), steerRefPoint.Position, steerRefPoint.Direction);

            float aimLane = _laneOvershoot ? steerRefOffset : _carriedLane;
            float aimPointOffset = aimLane - GsPreviewBlend * (offsetAtProjection - steerRefOffset);
            Vector3 aimPoint = steerRefPoint.Position + steerRight * aimPointOffset;
            _pursuitAimLateralM = ARS.SignedLaneOffset(aimPoint, Car.Position, Car.ForwardVector);

            float laneSteerDeg = PursuitSteerDegrees(aimPoint, courseDir);
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

                float combinedHalfWidth = (VehicleData.BoundingBox + r.RivalRacer.VehicleData.BoundingBox) * 0.5f;
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

                if (!KickWantsSlide())
                {
                    float countersteerTarget = (steerKP * sideBySideSteerDeg) - (VehicleData.SlideAngle * CountersteerShare());
                    Control.SteerDegrees += (countersteerTarget - Control.SteerDegrees) * _slidePriority;
                }
            }

            float slideDeg = VehicleData.SlideAngle;

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

            if (Math.Abs(_steerAimBearingDegrees) < SmallAimBearingDegrees && _slidePriority <= 0f) Control.SteerDegrees = 0f;
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

                // Half his heading error, not a fixed nudge: the assist scales with how hard he is angling, and there is no cap.
                correction += headingDifference * SideBySideHeadingShare * proximity;
            }
            return correction;
        }

        // A line exists at all only under this radius.
        const float CurveLeanRadiusMeters = 900f;
        const float CurveLeanChordSeconds = 1.25f;
        // The base rate: width over radius, 10 m/s when the radius equals the width.
        const float CurveLeanWidthScale = 20f;

        // OUT ignores the bend: 6 m/s, straight or curve.
        const float CurveLeanOutMps = 6f;

        // How far the heading must lean out before the line stops pulling in. At 10 deg the car is genuinely going wide:
        // 5.2 m/s of lateral drift at 30 m/s, well past a car merely following the bend.
        const float CurveLeanInwardHoldDegrees = 10f;
        // How far inside the car the aim may still sit while that hold is on: a quarter of a metre, so the line keeps
        // pointing at the inside without pulling the car there.
        const float CurveLeanInwardHoldLeadMeters = 0.25f;

        const float LaneLockTestOffsetMeters = 0.1f;

        const float PursuitGain = 2f;

        const float MaxPursuitBearingDegrees = 90f;

        const float SmallAimBearingDegrees = 1f;

        // Crossing scale for the steering ask. Off: the mover upstream may make it redundant.
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
            return ARS.RadToDeg((float)Math.Atan(PursuitGain * Math.Sin(ARS.DegToRad(bearing)) * VehicleData.WheelBase / distance));
        }

        float PursuitSteerDegrees(Vector3 aimPoint, Vector3 heading)
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

            _pursuitRawBearingDeg = bearing;
            _pursuitAimDistanceM = distance;
            _steerAimBearingDegrees = bearing;
            _steerAimCurvature = 2f * (float)Math.Sin(ARS.DegToRad(bearing)) / distance;
            return PursuitSteerFromBearing(bearing, distance);
        }

        bool TryGetOutsideLane(CornerPoint lineCorner, TrackPoint steerRefPoint, float speedMps, out float lane)
        {
            lane = 0f;
            if (lineCorner == null) return false;
            int apexNode = lineCorner.Node;
            if (HasPassedApex(apexNode)) return false;

            if (!(lineCorner.DetectedRadius < IdealLineMaxRadiusMeters)) return false;
            if (!(speedMps > ApexSpeedWithDownforce(lineCorner.DetectedRadius) - ARS.MphToMps(IdealLineSpeedMarginMph))) return false;

            float metresToApex = MetresToNode(apexNode);
            if (!(metresToApex <= IdealLineWindowMeters)) return false;

            if (_lineBiasNode != apexNode)
            {
                _lineBiasNode = apexNode;
                _lineBiasMeters = ARS.GetRandomInt(-(int)(LaneBiasMaxMeters * 100f), (int)(LaneBiasMaxMeters * 100f) + 1) / 100f;
            }
            _lineCornerNode = apexNode;
            float safeBound = Math.Max(steerRefPoint.TrackHalfWidth - VehicleData.BoundingBox * 0.5f - EdgeEmptySpaceMeters, 0f);
            float outsideLane = Math.Sign(lineCorner.Angle) * Math.Max(safeBound - LaneOutsideGapMeters - _lineBiasMeters, 0f);
            lane = outsideLane;

            int outsideEntranceNode = CornerEntranceNode(lineCorner, apexNode);
            if (outsideEntranceNode >= 0)
            {
                if (ForwardNodeDistance(steerRefPoint.Node) >= ForwardNodeDistance(outsideEntranceNode)) return false;

                // The one release the width can veto: the corner has already begun by heading, so the outside line only lets go
                // once the car has used the width. What it must reach is whatever the walls leave reachable, not the bare
                // outside lane: a rival wall can close that lane for the whole corner and waiting for it would never end.
                float ownLane = ARS.SignedLaneOffset(Car.Position, CurrentTrackPoint.Position, CurrentTrackPoint.Direction);
                float reachable = ClampLaneToWalls(outsideLane, steerRefPoint.TrackHalfWidth, _leftWallHeld, _rightWallHeld);
                Vector3 steerFlat = new Vector3(steerRefPoint.Direction.X, steerRefPoint.Direction.Y, 0f);
                Vector3 entranceFlat = new Vector3(ARS.TrackPoints[outsideEntranceNode].Direction.X, ARS.TrackPoints[outsideEntranceNode].Direction.Y, 0f);
                if (steerFlat.LengthSquared() > 0.01f && entranceFlat.LengthSquared() > 0.01f
                    && Vector3.Angle(steerFlat, entranceFlat) < OutReleaseAlignDegrees
                    && Math.Abs(ownLane - reachable) < OutReleaseReachedMeters) return false;
            }

            if (metresToApex <= OutsideGateMeters(steerRefPoint)) return false;
            return true;
        }

        bool HeadingLeansOut(float cornerDir)
        {
            Vector3 track = new Vector3(CurrentTrackPoint.Direction.X, CurrentTrackPoint.Direction.Y, 0f);
            Vector3 heading = new Vector3(Car.ForwardVector.X, Car.ForwardVector.Y, 0f);
            if (track.LengthSquared() < 0.0001f || heading.LengthSquared() < 0.0001f) return false;
            Vector3 right = Vector3.Cross(track.Normalized, Vector3.WorldUp);
            float lateral = Vector3.Dot(heading.Normalized, right);
            return lateral * cornerDir > (float)Math.Sin(ARS.DegToRad(CurveLeanInwardHoldDegrees));
        }

        bool TryComputeCurveLean(CornerPoint lineCorner, TrackPoint steerRefPoint, float speedMps, float ownLane, float laneEdge, float trackWidthMeters, out float lane, out bool outward)
        {
            lane = 0f;
            outward = false;
            _laneCornerDir = 0;

            if (TryGetOutsideLane(lineCorner, steerRefPoint, speedMps, out float outsideLane))
            {
                lane = outsideLane;
                outward = true;
                _laneCornerDir = Math.Sign(lineCorner.Angle);
                _leanApproachMps = CurveLeanOutMps;
                return true;
            }
            int count = ARS.TrackPoints.Count;
            int fwdNode;
            int fwdOffset = (int)(speedMps * CurveLeanChordSeconds);
            if (ARS.IsPointToPoint)
                fwdNode = (int)ARS.Clamp(CurrentTrackPoint.Node + fwdOffset, 0, count - 1);
            else
                fwdNode = ((CurrentTrackPoint.Node + fwdOffset) % count + count) % count;

            TrackPoint ahead = ARS.TrackPoints[fwdNode];
            if (!(ahead.PreciseCurveRadius <= CurveLeanRadiusMeters)) return false;

            float cornerDir = Math.Sign(ahead.Angle);
            if (cornerDir == 0f) return false;

            _leanApproachMps = CurveLeanWidthScale * trackWidthMeters / Math.Max(ahead.PreciseCurveRadius, 5f);
            _laneCornerDir = (int)cornerDir;
            // Inside the corner the lane may ask for the inner extreme; on the way in it keeps the gap, so the line does not hug
            // the edge a hundred metres before the turn-in.
            float insideGap = IsWithinCorner(lineCorner) ? 0f : LaneInsideGapMeters;
            lane = -cornerDir * Math.Max(laneEdge - insideGap, 0f);

            if (lane * cornerDir < ownLane * cornerDir && HeadingLeansOut(cornerDir))
            {
                float held = ownLane - cornerDir * CurveLeanInwardHoldLeadMeters;
                if (held * cornerDir > lane * cornerDir) lane = held;
            }
            return true;
        }

        const float LaneOverrideHoldMeters = 0.1f;

        // The mover: 40 m/s^2, 24 m/s, half of both outward. The cap bounds the lunge, the acceleration the reversal.
        const float LaneMoverAccelMps2 = 40f;
        const float LaneMoverSpeedCapMps = 99f;

        const float LaneMoverMaxStepSeconds = 0.1f;

        const float LaneCrossMinMeters = 0.1f;

        // Elastic: the lane leads the car at full rate up to 2.5 m, fades to a stop at 5 m, and past 5 it comes back, at full
        // pull by 6 m. The aim is a leash, never a rope.
        public const float LaneTaperNearMeters = 2.5f;
        const float LaneTaperFarMeters = 5f;
        const float LaneTaperCapShare = 0f;
        const float LaneElasticReturnEnd = 6f;

        void UpdateLaneMover(float steerRefOffset, float laneEdge)
        {

            float request = ARS.Clamp(_laneRequest, -laneEdge, laneEdge);

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

            float aimFromCar = _carriedLane - steerRefOffset;
            float askFromCar = request - steerRefOffset;

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

            aimFromCar = _carriedLane - steerRefOffset;
            askFromCar = request - steerRefOffset;
            _laneOvershoot = aimFromCar * askFromCar > 0f && Math.Abs(aimFromCar) > Math.Abs(askFromCar) + LaneCrossMinMeters;

            float dt = Math.Min(TickScale, LaneMoverMaxStepSeconds);

            bool toOutside = _laneRequestOutward && _laneRequestReason == LaneReason.CurveLean;
            float moverAccel = toOutside ? LaneMoverAccelMps2 * 0.5f : LaneMoverAccelMps2;

            if (_laneOvershoot) moverAccel *= 2f;
            float moverSpeedCap = toOutside ? LaneMoverSpeedCapMps * 0.5f : LaneMoverSpeedCapMps;

            if (_laneRequestReason == LaneReason.CurveLean) moverSpeedCap = Math.Min(moverSpeedCap, _leanApproachMps);

            float awayFromCar = Math.Abs(_carriedLane - steerRefOffset);
            float leash = ARS.Clamp((awayFromCar - LaneTaperNearMeters) / (LaneTaperFarMeters - LaneTaperNearMeters), 0f, 1f);
            moverSpeedCap *= 1f + leash * (LaneTaperCapShare - 1f);
            float requestRate = ARS.Clamp((request - _laneRequestPrev) / dt, -moverSpeedCap, moverSpeedCap);
            _laneRequestPrev = request;

            float arrivalRate = Math.Sign(gap) * Math.Min(moverSpeedCap, (float)Math.Sqrt(2f * moverAccel * Math.Abs(gap)));
            float desiredRate = requestRate + arrivalRate;

            // Past the far end the aim stops being led and returns to the car, harder the further out it sits.
            if (awayFromCar > LaneTaperFarMeters)
            {
                float pull = ARS.Clamp((awayFromCar - LaneTaperFarMeters) / (LaneElasticReturnEnd - LaneTaperFarMeters), 0f, 1f);
                float returnMps = Math.Min(LaneMoverSpeedCapMps, (float)Math.Sqrt(2f * moverAccel * (awayFromCar - LaneTaperFarMeters)));
                desiredRate = Math.Sign(steerRefOffset - _carriedLane) * pull * returnMps;
            }

            if (desiredRate * _laneRate < 0f) _laneRate = 0f;
            float rateStep = moverAccel * dt;
            _laneRate += ARS.Clamp(desiredRate - _laneRate, -rateStep, rateStep);
            float step = _laneRate * dt;

            _carriedLane += step;
            _carriedLane = ARS.Clamp(_carriedLane, -laneEdge, laneEdge);

            if (Math.Abs(_laneRate) > 0.2f && Game.GameTime > _laneMoverLoggedAt)
            {
                _laneMoverLoggedAt = Game.GameTime + 1000;
                ARS.Log(ARS.LogImportance.Info, "Lane mover " + Name + ": asked " + _laneRequest.ToString("0.00") + " (" + _laneRequestReason + "), aimed " + _carriedLane.ToString("0.00") + ", rate " + _laneRate.ToString("0.00"));
            }
        }

        const float RequirementLookaheadSeconds = 3.95f;

        // OUT window: from here to the gate the line names the outer edge.
        const float IdealLineWindowMeters = 300f;

        const float LaneOutsideGapMeters = 0.5f;
        // The lane never leaves the road: whatever asks for it, the aim keeps the car's half width plus this buffer from the edge.
        const float EdgeEmptySpaceMeters = 0.5f;
        const float LaneInsideGapMeters = 1f;
        const float LaneBiasMaxMeters = 0.5f;

        const float LineApexStickyMeters = 30f;

        // The gate: six track widths before the apex, floored at 20 m.
        const float IdealLineBlendWidthFactor = 6f;
        const float IdealLineBlendMinMeters = 20f;
        // Beyond this radius a bend is a kink and gets no outside.
        const float IdealLineMaxRadiusMeters = 50f;
        // How close the steering lookahead's direction has to be to the entrance's before the out state calls the corner begun.
        const float OutReleaseAlignDegrees = 10f;
        // How close to the outside lane the car has to be before the out state is allowed to let go of it.
        const float OutReleaseReachedMeters = 1f;
        const float IdealLineSpeedMarginMph = 15f;

        CornerPoint NearestApexAhead()
        {
            int count = ARS.TrackPoints.Count;
            if (count < 10) return null;

            CornerPoint best = null;
            float bestMeters = float.MaxValue;
            CornerPoint adopted = null;
            float adoptedMeters = float.MaxValue;
            foreach (CornerPoint corner in ARS.Corners)
            {
                if (corner.DetectedRadius >= IdealLineMaxRadiusMeters) continue;
                float metres = MetresToNode(corner.Node);
                if (corner.Node == _lineCornerNode && metres >= 0f) { adopted = corner; adoptedMeters = metres; }
                if (metres < 0f || metres >= bestMeters) continue;
                bestMeters = metres;
                best = corner;
            }

            if (adopted != null && adoptedMeters <= bestMeters + LineApexStickyMeters) return adopted;
            return best;
        }

        const float AvoidAngleGateDegrees = 20f;

        const float SameSectionMaxGapMeters = 30f;

        // The same five seconds the avoidance lane arms on: a rival we are catching has to keep the corridor closed until
        // the avoidance lane has taken over from it. Retiring the wall at three left a two second band in which the corridor
        // was open, the avoidance lane was not yet the request, and the lean was free to aim at the car ahead.
        const float WallApproachSeconds = 5f;

        const float EngineHealthHalf = 500f;

        const float WallSideDeadbandMeters = 0.3f;

        const float AvoidHoldBehindMeters = 14f;
        const float AvoidHoldLateralMeters = 1.5f;

        const int AvoidHoldTimeoutMs = 8000;

        float ComputeAvoidanceLane(float roadWide)
        {

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

            // A pass lane is a decision, not a shadow: once chosen it holds while the target persists, so a rival drifting down
            // the road no longer drags our line with him. It is recomputed when he is no longer clear of the latched lane.
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

            float buffer = rival.RivalRacer.VehicleData.BoundingBox * 0.5f + aggroBuffer;

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

        float ClampLaneToWalls(float lane, float roadWide, bool leftConstrained, bool rightConstrained)
        {

            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            float trackBound = roadWide;
            float clampLeft = leftConstrained ? Math.Max(_avoidLeftWall + carHalfWidth, -trackBound) : Math.Max(_avoidLeftWall, -trackBound);
            float clampRight = rightConstrained ? Math.Min(_avoidRightWall - carHalfWidth, trackBound) : Math.Min(_avoidRightWall, trackBound);
            if (clampLeft > clampRight)
            {

                float collapsed = (clampLeft + clampRight) * 0.5f;
                return ARS.Clamp(collapsed, -trackBound, trackBound);
            }
            return ARS.Clamp(lane, clampLeft, clampRight);
        }

        float ApplyRivalWalls(float requestLane, float roadWide)
        {

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

            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null) continue;

                bool overlaps = Math.Abs(r.RouteGapAhead) < r.CombinedSize.Y;

                bool approaching = r.TimeToReach < WallApproachSeconds;
                if (!overlaps && !approaching) continue;

                float aggroBuffer = ARS.Remap(Aggression, 100f, 0f, 0.2f, 1.2f, true);

                float rivalBuffer = r.RivalRacer.VehicleData.BoundingBox * 0.5f + aggroBuffer;

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
            _wallRoadWide = roadWide;

            float wallCloseRate = 10f * TickScale;
            float wallOpenRate = 5f * TickScale;
            float wallSnapMeters = VehicleData.BoundingBox;

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

            return ClampLaneToWalls(requestLane, roadWide, leftConstrained, rightConstrained);
        }

        const float SlidingFraction = 0.3f;

        const float SlideBlendStartFraction = 1f;
        const float SlideBlendFullFraction = 2.5f;

        const float CountersteerSlideShare = 0.5f;

        const float CountersteerAssistDegreesPerSecond = 10f;

        const float CountersteerTotalShare = 1f;

        const float CountersteerPedalCap = 0.5f;

        const float CountersteerMinSpeedMph = 10f;

        const float SteerSlewRate = 120f;

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
            float ratio = VehicleData.WheelBase * grip * Handling.Gravity / (fwdSpeed * fwdSpeed);
            if (float.IsNaN(ratio) || ratio <= 0f) return VehicleData.SteeringLock;
            return ARS.RadToDeg((float)Math.Atan(ratio));
        }

        public float TRLateralAtSpeed = 22f;

        void UpdateTRLateralAtSpeed()
        {
            float speed = Car.Velocity.Length();
            TRLateralAtSpeed = float.IsNaN(speed) ? 0f : LateralPeakAtSpeed(speed);
        }

        float LateralPeakAtSpeed(float speed)
        {
            float lat = Handling.LateralTractionCurve;
            return lat <= 0.01f ? 0f : lat / (1f + Math.Min(5f, 0.1f * speed));
        }

        public struct SlipState
        {
            public float InnerPct;
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

        SlipState MeasureSlipFromMemory(float fwdSpeed)
        {
            SlipState s = new SlipState();
            List<float> slips = ARS.WheelSideSlipAngles(Car);
            if (slips.Count < 2) return s;
            s.Readable = true;

            float rawLeft = Math.Abs(slips[0]);
            float rawRight = Math.Abs(slips[1]);

            float rawInner = Math.Abs(Control.SteerDegrees) < FrontSlipNeutralSteerDeg
                ? 0.5f * (rawLeft + rawRight)
                : Math.Max(rawLeft, rawRight);

            float capShare = Math.Max(0.05f, ARS.SteerCeilingFactor) * SlowCornerSlipShare(fwdSpeed);
            s.InnerPct = 100f * rawInner / capShare;
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
            float wheelBase = Math.Max(VehicleData.WheelBase, 0.5f);

            if (float.IsNaN(speed) || float.IsNaN(gripGs) || float.IsNaN(wheelBase)) return s;
            if (speed < FrontSlipTheoryMinSpeedMps || gripGs <= 0.01f || wheelBase <= 0.5f) return s;

            s.Readable = true;
            float available = gripGs * FrontSlipGravity * Math.Max(0.05f, ARS.SteerCeilingFactor) * SlowCornerSlipShare(fwdSpeed);

            float commandDeg = Math.Abs(Control.SteerDegrees);
            float innerRadius = commandDeg > FrontSlipTheoryMinCommandDeg
                ? wheelBase / (float)Math.Tan(commandDeg * (float)Math.PI / 180f)
                : float.MaxValue;
            float demand = speed * speed / Math.Max(innerRadius, 0.5f);

            s.InnerPct = 100f * demand / available;
            return s;
        }

        const float OpensUpThrottleSpanShare = 0.5f;

        float OpensUpClampScale()
        {
            float floorPct = ARS.Clamp(ARS.OpenUpOnThrottleFloorPct, 0, 100) / 100f;
            if (floorPct >= 1f) return 1f;

            float travel = ARS.Clamp(Control.Throttle / OpensUpThrottleSpanShare, 0f, 1f);
            return 1f - (1f - floorPct) * travel;
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
            float ceiling = TRLateralAtSpeed > 0.01f ? PeakSlipCeilingAt(TRLateralAtSpeed) * ARS.SteerCeilingFactor : GeometrySteerCeiling(fwdSpeed);
            float speedMph = ARS.MpsToMph(fwdSpeed);
            if (speedMph >= SteerLimitRampEndMph) return ceiling;

            float endSpeed = ARS.MphToMps(SteerLimitRampEndMph);
            float endPeak = LateralPeakAtSpeed(endSpeed);
            float endCeiling = endPeak > 0.01f ? PeakSlipCeilingAt(endPeak) * ARS.SteerCeilingFactor : GeometrySteerCeiling(endSpeed);

            return Math.Max(ceiling, ManeuverRamp(speedMph, endCeiling));
        }

        static readonly bool SpeedSteerCeiling = false;

        float ManeuverRamp(float speedMph, float ceiling)
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
            float fwdSpeed = Vector3.Dot(Car.Velocity, Car.ForwardVector);

            bool countersteering = Math.Sign(requestedSteer) != Math.Sign(VehicleData.YawRotationPerSecondDegrees);
            float speedCeiling = VehicleData.SteeringLock;
            if (fwdSpeed > 0f) speedCeiling = ResolveSteerCeiling(fwdSpeed);
            _slip = MeasureFrontSlip(fwdSpeed);
            _slipFactor = 1f;
            _clampTargetDeg = float.NaN;

            if (_slip.Readable && _slip.InnerPct > 0.01f)
            {

                float factor = ARS.Clamp(100f / _slip.InnerPct, ARS.SlipCutFloor, 1f);
                _slipFactor = factor;

                float clampTarget = _slip.InnerPct > 100f
                    ? Math.Abs(requestedSteer) * factor
                    : VehicleData.SteeringLock;
                _clampTargetDeg = clampTarget;

                _clampTargetDeg *= OpensUpClampScale();
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
            _finalCeilingDeg = speedCeiling;

            float steerLimitRight = speedCeiling;
            float steerLimitLeft = speedCeiling;

            if (countersteering && _slidePriority > 0f)
            {
                float countersteerAllowance = Math.Min(Math.Abs(VehicleData.SlideAngle) * _slidePriority, VehicleData.SteeringLock);
                if (requestedSteer > 0f) steerLimitLeft = Math.Max(steerLimitLeft, countersteerAllowance);
                else steerLimitRight = Math.Max(steerLimitRight, countersteerAllowance);
            }

            float yawRate = VehicleData.YawRotationPerSecondDegrees;

            if (countersteering && _damperTermDeg * yawRate < 0f)
            {
                if (requestedSteer > 0f) steerLimitLeft = VehicleData.SteeringLock;
                else if (requestedSteer < 0f) steerLimitRight = VehicleData.SteeringLock;
            }

            Control.SteerDegrees = ARS.Clamp(requestedSteer, -steerLimitRight, steerLimitLeft);
            _steerPreLimitDegrees = requestedSteer;
            _steerGrantedCeiling = requestedSteer >= 0f ? steerLimitLeft : steerLimitRight;
        }

        float CountersteerShare() => CountersteerSlideShare;

        bool IsFullCountersteer()
        {
            if (KickWantsSlide()) return false;
            if (Vector3.Dot(Car.Velocity, Car.ForwardVector) < ARS.MphToMps(CountersteerMinSpeedMph)) return false;
            return _slidePriority >= 1f;
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

            _avoidHoldActive = false;
            _avoidHoldLane = 0f;
            _avoidHoldUntil = 0;

            _lineBiasNode = -1;
            _lineCornerNode = -1;
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

        const float PedalSlewRate = 6f;
        const float PedalSlewMaxPerTick = 0.5f;

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

        const float GsLaneFloorLateralShare = 0.525f;

        float GsLaneThrottleFloorValue(out bool insideRegime)
        {
            insideRegime = false;
            if (!ARS.GsLaneThrottleFloor) return 0f;

            // The projection owns the pedals once it is outside the edge, because it is the one braking for the line: the floor
            // yields to it whether or not the car is still over its target speed. Waiting on the outside timer was not enough,
            // since that timer only runs while the car is also faster than the target and re-arms the moment it is not.
            if (_projectionOutsideMeters > 0f && _projectionWidening) return 0f;
            float theoreticalGs = VehicleData.CurrentMechanicalGrip * Handling.Gravity / 9.8f;
            if (!(theoreticalGs > 0f)) return 0f;
            if (Math.Abs(VehicleData.GetLateralGs(Car.ForwardVector)) <= GsLaneFloorLateralShare * theoreticalGs) return 0f;

            int insideSign = -Math.Sign(CurrentTrackPoint.Angle);
            if (insideSign == 0) return 0f;

            insideRegime = true;

            Vector3 projected = ProjectAhead(SteerPreviewSeconds);
            int projectedWindow = (int)(Car.Velocity.Length() * SteerPreviewSeconds) + 10;
            TrackPoint projectedNode = ARS.FindNearestTrackPoint(projected, CurrentTrackPoint.Node, 0, projectedWindow);
            float projectedOffset = ARS.SignedLaneOffset(projected, projectedNode.Position, projectedNode.Direction);

            float room = Math.Max(projectedNode.TrackHalfWidth - VehicleData.ModelDimensions.X * 0.5f, 1f);
            // Full pedal from the inside through the centre, falling to half at the outer edge. Past that edge the off-track
            // projection owns the braking, so the floor stays put and lets it act.
            float inward = projectedOffset * insideSign / room;
            float outerShare = ARS.Clamp(-inward, 0f, 1f);
            return GsLaneThrottleMaxFloor - (GsLaneThrottleMaxFloor - GsLaneOutsideFloor) * outerShare;
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

            if (_slip.Readable && _slip.InnerPct > 100f)
            {
                float over = ARS.Clamp((_slip.InnerPct - 100f) / 150f, 0f, 1f);
                combinedInput = Math.Min(combinedInput, Math.Max(0f, 1f - over));
            }
            MeasureProjectionOutside(1f);

            bool countersteering = IsFullCountersteer();
            SplitCombinedInput(combinedInput, ref newThrottle, ref newBrake);
            if (countersteering)
            {
                newThrottle = Math.Min(newThrottle, CountersteerPedalCap);
                newBrake = Math.Min(newBrake, CountersteerPedalCap);
            }

            float projectionSpeedMph = ARS.MpsToMph(Car.Velocity.Length());
            float projectionTargetMph = Math.Max(OffTrackSpeedFloorMph, ARS.MpsToMph(RouteIdealSpeedForRadius(Brain.CurrentPerception.CurveRadiusToFollowPoint)) - OffTrackSpeedSlackMph);
            bool projectionOutside = _projectionWidening && _projectionOutsideMeters > 0f && projectionSpeedMph > projectionTargetMph;
            _projectionOutsideSeconds = projectionOutside ? _projectionOutsideSeconds + TickScale : 0f;
            float projectionInput = Math.Max(1f - ProjectionInputPerSecond * _projectionOutsideSeconds, -ProjectionBrakeFloor);
            bool projectionBinds = !countersteering && newThrottle > Math.Max(projectionInput, 0f);
            newThrottle = ComposeThrottleCap(newThrottle, projectionBinds, countersteering);
            _offtrackThrottleHeld = projectionBinds;

            newBrake = ComposeBrakeCap(newBrake, countersteering);

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

            Control.Brake += ARS.Clamp(newBrake - Control.Brake, -inputChange, inputChange);
            Control.Throttle += ARS.Clamp(newThrottle - Control.Throttle, -inputChange, inputChange);

            float gsFloor = GsLaneThrottleFloorValue(out bool gsInsideRegime);

            if (countersteering) gsFloor = 0f;
            gsFloor = Math.Min(gsFloor, Control.MaxThrottle);
            float throttleBeforeFloor = Control.Throttle;
            if (gsFloor > 0f) Control.Throttle = Math.Max(Control.Throttle, gsFloor);
            _gsFloorActiveThisTick = Control.Throttle > throttleBeforeFloor;
            _projectionPathColor = ProjectionPathColour(gsFloor, gsInsideRegime);

            UpdateBrakeLearning();

        }

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

                int projectionWindow = (int)(Car.Velocity.Length() * seconds) + 10;
                TrackPoint tp = ARS.FindNearestTrackPoint(proj, CurrentTrackPoint.Node, 0, projectionWindow);
                signedOffset = ARS.SignedLaneOffset(proj, tp.Position, tp.Direction);
                halfWidth = Math.Max(tp.TrackHalfWidth, 0.1f);

                turnDirection = Math.Abs(CurrentTrackPoint.Angle) > OffshootMinRouteAngleDeg ? Math.Sign(CurrentTrackPoint.Angle) : 0f;

                if (Math.Sign(signedOffset) == turnDirection)
                {
                    outsideOffset = signedOffset * turnDirection;

                    carOffset = ARS.SignedLaneOffset(Car.Position, CurrentTrackPoint.Position, CurrentTrackPoint.Direction) * turnDirection;

                    _projectionOutsideMeters = outsideOffset - halfWidth;
                    _projectionWidening = outsideOffset > carOffset;
                }
            }

            if (ARS.DebugToggles[Options.ShowGsProjection] && ARS.DebugFocusRacer == this && Game.GameTime >= _offshootLogAt)
            {
                _offshootLogAt = Game.GameTime + 500;
                ARS.Log(ARS.LogImportance.Info, "Offshoot: override=" + laneOverride + " mph=" + mph.ToString("0") + " turn=" + turnDirection + " signed=" + signedOffset.ToString("0.00") + " car=" + carOffset.ToString("0.00") + " half=" + halfWidth.ToString("0.00") + " outside=" + outsideOffset.ToString("0.00") + " sec=" + _projectionOutsideSeconds.ToString("0.00") + " input=" + (Math.Max(1f - ProjectionInputPerSecond * _projectionOutsideSeconds, -ProjectionBrakeFloor)).ToString("0.00"));
            }

        }

        bool ProjectionClosingOnCentre()
        {
            Vector3 proj = ProjectAhead(1f);

            int projectionWindow = (int)(Car.Velocity.Length()) + 10;
            TrackPoint tp = ARS.FindNearestTrackPoint(proj, CurrentTrackPoint.Node, 0, projectionWindow);
            float carOffset = Math.Abs(ARS.SignedLaneOffset(Car.Position, CurrentTrackPoint.Position, CurrentTrackPoint.Direction));
            float projectionOffset = Math.Abs(ARS.SignedLaneOffset(proj, tp.Position, tp.Direction));
            return projectionOffset < carOffset;
        }

        void UpdateBrakeLearning()
        {
            if (!ARS.BrakeLearning)
            {
                _brakeCommitApexNode = -1;
                return;
            }

            if (_brakeCommitApexNode >= 0)
            {
                float exitSlide = Math.Abs(VehicleData.SlideAngle);
                if (!float.IsNaN(exitSlide) && exitSlide > _brakeCommitMaxSlideDeg) _brakeCommitMaxSlideDeg = exitSlide;
                if (Game.GameTime >= _brakeCommitAtGameTime) CommitBrakeLearning();
            }

            if (NextApexNode != _brakeSampleApexNode)
            {

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

            }

            if (_brakeSampleApexNode >= 0)
            {
                TrackPoint apexTrack = ARS.TrackPoints[_brakeSampleApexNode];
                float approach = Car.Position.DistanceTo(apexTrack.Position);

                if (approach < BrakeApexNearMetres && approach < _brakeSampleApexClosest)
                {
                    _brakeSampleApexClosest = approach;
                    _brakeSampleApexGap = apexTrack.TrackHalfWidth - Math.Abs(ARS.SignedLaneOffset(Car.Position, apexTrack.Position, apexTrack.Direction));
                }
            }

            if (_brakeSampleApexNode >= 0 && !HasPassedApex(_brakeSampleApexNode) && Game.GameTime >= _brakeOverNextSampleAt)
            {
                _brakeOverNextSampleAt = Game.GameTime + BrakeOverSampleMs;

                float ramp = ApexBrakingSpeed(_brakeSampleApexNode, NextApexSpeed) + BrakeLearnerBarMps * BrakeOverAllowanceShare;
                float carried = ARS.GetForwardSpeed(Car);
                bool over = ramp < 100f && carried > ramp;

                if (over && Car.IsInAir) _brakeSampleWasAirborne = true;

            }

            if (!_brakeSampleEntryMeasured && _brakeSampleApexNode >= 0 && !HasPassedApex(_brakeSampleApexNode))
            {

                float apexSpeed = _brakeSampleApexSpeed + BrakeLearnerBarMps;
                float forward = ARS.GetForwardSpeed(Car);
                if (forward <= apexSpeed + 0.2f && _brakeSampleSawAboveApex)
                {

                    _brakeSampleBelowTicks++;
                    if (_brakeSampleBelowTicks == 1)
                    {

                        CornerPoint corner = ARS.Corners.FirstOrDefault(c => c.Node == _brakeSampleApexNode);
                        int entranceNode = CornerEntranceNode(corner, _brakeSampleApexNode);
                        if (entranceNode >= 0)
                        {

                            float metres = CurrentTrackPoint.CumulativeDistance - ARS.TrackPoints[entranceNode].CumulativeDistance;

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

            if (!_brakeSampleEntranceCrossed && _brakeSampleApexNode >= 0)
            {
                CornerPoint entranceCorner = ARS.Corners.FirstOrDefault(c => c.Node == _brakeSampleApexNode);
                int entranceNode = CornerEntranceNode(entranceCorner, _brakeSampleApexNode);
                if (entranceNode >= 0)
                {
                    float toEntrance = ARS.TrackPoints[entranceNode].CumulativeDistance - CurrentTrackPoint.CumulativeDistance;
                    if (!ARS.IsPointToPoint && ARS.RouteLengthMeters > 0f && Math.Abs(toEntrance) > ARS.RouteLengthMeters * 0.5f)
                        toEntrance -= Math.Sign(toEntrance) * ARS.RouteLengthMeters;

                    if (toEntrance > 0f && toEntrance <= Car.Velocity.Length() * BrakeFloorLatchSeconds && _gsFloorActiveThisTick)
                        _brakeSampleFloorAtEntrance = true;
                    if (toEntrance <= 0f) _brakeSampleEntranceCrossed = true;
                }
            }

            if (_brakeSampleApexNode >= 0 && HasPassedBrakingTarget() && !HasPassedApex(_brakeSampleApexNode)
                && Control.Brake > BrakeSkidBrakeShare)
            {
                float slide = Math.Abs(VehicleData.SlideAngle);
                if (!float.IsNaN(slide) && slide > _brakeSampleMaxSlideDeg) _brakeSampleMaxSlideDeg = slide;
            }

            if (_brakeCommitApexNode < 0 && _brakeSampleApexNode >= 0 && HasPassedApex(_brakeSampleApexNode))
                ScheduleBrakeCommit();
        }

        void ScheduleBrakeCommit()
        {

            if (_brakeCommitApexNode >= 0 && _brakeCommitApexNode == _brakeSampleApexNode) return;

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

            float step = ARS.Clamp(-_brakeCommitEntryMetres * BrakeEntryFactorPerMetre, -BrakeEntryMaxStepPerEvent, BrakeEntryMaxStepPerEvent);
            float factor = ARS.Clamp(before + step, BrakeMinFactor, BrakeMaxFactor);
            if (BrakeLearningApplies) CornerContextForWrite(apexNode).BrakeFactor = factor;
            AnnounceBrakeAdjustment(where + " → " + factor.ToString("0.00"));
        }

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

        const float CornerLiftMphPerSecond = 10f;
        const float CornerLiftGraceMph = 5f;

        int BrakeTargetBaseNode(CornerPoint corner, int fallbackNode)
        {
            int entranceNode = CornerEntranceNode(corner, fallbackNode);
            return entranceNode < 0 ? fallbackNode : OffsetCornerNode(entranceNode, -BrakeTargetLeadMeters);
        }

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

                if (CornerRequiresBraking(NextApexNode)) cornerSpd = Math.Max(2, ApexBrakingSpeed(NextApexNode, NextApexSpeed));
                if (NextApexNode2 >= 0 && CornerRequiresBraking(NextApexNode2))
                    cornerSpd = Math.Min(cornerSpd, Math.Max(2, ApexBrakingSpeed(NextApexNode2, NextApexSpeed2)));
                if (NextApexNode3 >= 0 && CornerRequiresBraking(NextApexNode3))
                    cornerSpd = Math.Min(cornerSpd, Math.Max(2, ApexBrakingSpeed(NextApexNode3, NextApexSpeed3)));
            }

            float followTrackSpd = RouteIdealSpeedForRadius(Brain.CurrentPerception.CurveRadiusToFollowPoint);

            if (float.IsNaN(cornerSpd) || float.IsInfinity(cornerSpd)) cornerSpd = 999f;
            if (float.IsNaN(followTrackSpd) || float.IsInfinity(followTrackSpd)) followTrackSpd = 999f;
            if (cornerSpd <= 5 && Brain.Corner != null) cornerSpd = ARS.CornerApexSpeed(Brain.Corner.Point, this);

            if (NextApexNode != _cornerLiftApex)
            {
                _cornerLiftApex = NextApexNode;
                _cornerLiftMps = 0f;
            }
            if (cornerSpd < 999f)
            {

                float liftTarget = cornerSpd + _cornerLiftMps;
                if (HasPassedBrakingTarget()
                    && ARS.GetForwardSpeed(Car) <= liftTarget + ARS.MphToMps(CornerLiftGraceMph))
                    _cornerLiftMps += ARS.MphToMps(CornerLiftMphPerSecond) * TickScale;
                cornerSpd += _cornerLiftMps;
            }

            if (NextApexNode >= 0 && HasPassedApex(NextApexNode)) cornerSpd = 999f;

            bool cornerPlansABrake = cornerSpd < 999f;

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

            _cornerSpd = NextApexNode >= 0 ? NextApexSpeed : (Brain.Corner != null ? ARS.CornerApexSpeed(Brain.Corner.Point, this) : 999f);

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
                Brain.CurrentIntention.SteerLimitedSpeed = 999f;
            }

            if (Brain.Corner != null && Brain.Corner.Point.IsChicane)
            {
                float chicaneBoost = ARS.MphToMps(10f);
                cornerSpd += chicaneBoost;
                followTrackSpd += chicaneBoost;
            }

            float cornerTerm = cornerSpd + ARS.MphToMps(ARS.CornerOffsetMph);
            float routeTerm = followTrackSpd + ARS.MphToMps(ARS.RouteOffsetMph);
            Brain.CurrentIntention.Speed = Math.Min(cornerTerm, routeTerm);

            Control.MaxBrakeFromRoute = routeTerm <= cornerTerm && !cornerPlansABrake ? RouteBrakeCap : 1f;

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

            float rbFactor = ComputeRubberBandFactor();
            bool rbNearbyRival = Brain.Rivals.Any(r => r.RivalRacer != null && r.Distance < 100f);
            bool rbFreeZone = Lap <= 1 && CurrentTrackPoint != null && CurrentTrackPoint.Node < 500;

            bool burning = Game.GameTime < _nitrousActiveUntil;
            if (!burning && rbFactor < 1f && !rbFreeZone && !(ARS.CurrentRubberbandMode == RubberbandMode.Natural && rbNearbyRival))
                Brain.CurrentIntention.Speed *= rbFactor;
        }

        const float nearSeconds = 1.0f;
        const float farSeconds = 2.0f;
        float CrestGripSpeedFactor(int carNode, int count, out int nearNode, out int farNode, out float rawDeltaGs)
        {
            nearNode = -1;
            farNode = -1;
            rawDeltaGs = 0f;
            if (carNode < 0 || carNode >= count) return 1f;
            float v = Car.Velocity.Length();
            if (v < 0.5f) return 1f;

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
        const float OffTrackThrottleLevel = 0f;
        const float OffTrackSafeSpeedMph = 20f;
        const float OffTrackFullThrottleMph = 15f;

        const float OffTrackRecoverThrottle = 0.75f;

        const float StabilityFallPerWheelPerSecond = 0.5f;
        const float StabilityRisePerSecond = 0.5f;

        const float OverspeedArmMaxSteerDegrees = 2f;
        const float OverspeedArmMaxSlideDegrees = 2f;
        const float OverspeedArmMinSpeedMph = 30f;

        const float OverspeedArmMaxLateralGs = 0.25f;

        const float OverspeedCutStartGs = 0.2f;

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

        float OverspeedThrottleLevel()
        {
            bool straight = ControlledByPlayer
                ? Math.Abs(VehicleData.GetLateralGs(Car.ForwardVector)) < OverspeedArmMaxLateralGs
                : Math.Abs(Control.SteerDegrees) < OverspeedArmMaxSteerDegrees;
            bool armed = straight
                && Math.Abs(VehicleData.SlideAngle) < OverspeedArmMaxSlideDegrees
                && Car.Velocity.Length() > ARS.MphToMps(OverspeedArmMinSpeedMph);

            if (ARS.OverspeedCutPerTenthG <= 0f || !armed || !(VehicleData.OverspeedExcessGs > 0f)) return 1f;
            float cut = ARS.OverspeedCutPerTenthG * (VehicleData.OverspeedExcessGs - OverspeedCutStartGs) / 0.1f;
            return ARS.Clamp(1f - cut, 0f, 1f);
        }

        void UpdateThrottleReasonCaps()
        {

            Control.MaxThrottleFromTCS = _kickPlaying ? 1f : GlideCap(Control.MaxThrottleFromTCS, TcsCapLevel());
            int wheelsOffGround = ARS.WheelsOffGround(Car);
            if (wheelsOffGround > 0)
                Control.MaxThrottleFromStability -= StabilityFallPerWheelPerSecond * wheelsOffGround * TickScale;
            else
                Control.MaxThrottleFromStability += StabilityRisePerSecond * TickScale;
            Control.MaxThrottleFromStability = ARS.Clamp(Control.MaxThrottleFromStability, 0f, 1f);

            Control.MaxThrottleFromOverspeed = Game.GameTime < _nitrousActiveUntil
                ? 1f
                : GlideCap(Control.MaxThrottleFromOverspeed, OverspeedThrottleLevel());

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

                if (ProjectionClosingOnCentre()) offTrackThrottle = OffTrackRecoverThrottle;
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

            float slipTarget = -SlipForShare(ARS.TcsSlipLimitPct) * (OutOfTrackDistance() > 0f ? OffTrackTargetShare : 1f);
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
            Control.MaxBrakeFromCountersteer = GlideCap(Control.MaxBrakeFromCountersteer, countersteering ? CountersteerPedalCap : 1f);

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

        float ComposeThrottleCap(float baseThrottle, bool offtrackLimited, bool countersteering)
        {

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

        void ConsiderManeuvers()
        {
            if (ControlledByPlayer || BaseBehavior != RacerBaseBehavior.Race || ARS.Racers.Count < 1) return;

            if (ActiveManeuver.Type != ManeuverType.None && Game.GameTime - ActiveManeuver.LastEnabled > 8000)
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
                        hisTimeToReachMe = back.TimeToReach;
                        break;
                    }
                    float combinedLength = Math.Abs(VehicleData.ModelDimensions.Y) + Math.Abs(r.RivalRacer.VehicleData.ModelDimensions.Y);
                    // Gone when he is neither closing nor close: the same pair that let him in. Closure alone cannot decide it,
                    // since a rival holding station behind never shows a closing time, and arrival time cannot either, since a
                    // car behind always reaches the entrance later than us and the test would fire on the very first tick.
                    bool stillClosing = !float.IsPositiveInfinity(hisTimeToReachMe);
                    bool stillClose = r.Distance < DefendCloseBehindMeters + combinedLength;
                    threatGone = !stillClosing && !stillClose;
                    break;
                }
                bool lostTarget = ActiveManeuver.Target == null || !ActiveManeuver.Target.Car.Exists() || !targetTrails;
                // The defence ends where the out state does, so the base behaviour takes the line back and the car turns in for
                // the apex when it should. The writer reads the same gate at the steer node and this one reads it at the car's
                // node: where the road changes width between them the slot can free a few metres early or late, which costs
                // nothing because this only frees the slot.
                bool outsideOver = MetresToNode(_defendApexNode) <= OutsideGateMeters(CurrentTrackPoint);

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

        // Metres from the car to a node along the route, wrapped for a loop and negative once it is behind us.
        float MetresToNode(int node)
        {
            if (node < 0 || node >= ARS.TrackPoints.Count) return 0f;
            float metres = ARS.TrackPoints[node].CumulativeDistance - CurrentTrackPoint.CumulativeDistance;
            if (!ARS.IsPointToPoint && ARS.RouteLengthMeters > 0f && Math.Abs(metres) > ARS.RouteLengthMeters * 0.5f)
                metres -= Math.Sign(metres) * ARS.RouteLengthMeters;
            return metres;
        }

        // Where the out state hands the line back: the same gate the lean uses, read at the node each caller has at hand.
        float OutsideGateMeters(TrackPoint at) => Math.Max(2f * at.TrackHalfWidth * IdealLineBlendWidthFactor, IdealLineBlendMinMeters);

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
            if (OutOfTrackDistance() > 0f) return false;
            if (Math.Abs(Control.SteerDegrees) >= NitrousMaxSteerDegrees) return false;
            if (Control.Throttle < NitrousMinThrottle) return false;
            if (!IsAwd() && Function.Call<int>((Hash)MaxDriveGearHash, Car) > 3 && Function.Call<int>((Hash)CurrentDriveGearHash, Car) <= 2) return false;
            if (NextApexNode < 0) return false;

            float speed = Car.Velocity.Length();
            Rival closestRival = NearestRival();

            bool finishSpender = closestRival != null && closestRival.Distance <= NitrousNearbyRivalDistance
                && RemainingRaceDistanceMeters() <= speed * (NitrousDurationMs / 1000f) + NitrousFinishExtraDistance;
            if (!finishSpender)
            {
                CornerPoint corner = _nextApexCorner;

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
            // Armed inside the out state's own gate the card could never move the car: the lane writer holds the defend
            // offset only while the apex is still beyond that gate.
            if (MetresToNode(apexNode) <= OutsideGateMeters(CurrentTrackPoint)) return false;

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
                    timeToReachMe = back.TimeToReach;
                    break;
                }

                // He counts if he is closing on me, or if he is simply close behind: the case the card exists for. A rival
                // that is merely close never shows a closing time, so the two tests have to be alternatives, not a chain.
                float combinedLength = Math.Abs(VehicleData.ModelDimensions.Y) + Math.Abs(r.RivalRacer.VehicleData.ModelDimensions.Y);
                bool closesSoon = ARS.IsBetween(timeToReachMe, 3f, 6f);
                bool closeBehind = r.Distance < DefendCloseBehindMeters + combinedLength;
                if (!closesSoon && !closeBehind) continue;

                // And he has to still be arriving: a rival who reaches the entrance long after me threatens nothing.
                float hisTimeToEntrance = r.RivalRacer.ForwardNodeDistance(entranceNode) / Math.Max(r.Speed, 1f);
                if (hisTimeToEntrance > myTimeToEntrance + DefendLaterMarginSeconds) continue;

                if (r.Distance < nearestDefender) { nearestDefender = r.Distance; defenderTarget = r; }
            }
            if (defenderTarget == null) return false;

            int defendTurn = Math.Sign(Brain.Corner.Point.Angle);
            if (defendTurn == 0) return false;

            Racer defender = defenderTarget.RivalRacer;
            float hisLane = ARS.SignedLaneOffset(defender.Car.Position, defender.CurrentTrackPoint.Position, defender.CurrentTrackPoint.Direction);
            float combinedWidth = Math.Abs(VehicleData.ModelDimensions.X) + Math.Abs(defender.VehicleData.ModelDimensions.X);
            float clearance = (Math.Abs(VehicleData.BoundingBox) + Math.Abs(defender.VehicleData.BoundingBox)) * 0.5f;
            float reachable = Math.Abs(CurrentTrackPoint.TrackHalfWidth) - Math.Abs(VehicleData.BoundingBox) * 0.5f - EdgeEmptySpaceMeters;
            float inward = -defendTurn;

            // If the road leaves no room to be clear of him on the inside, the card is not played: a target that clamps back
            // onto his outside would hand him the apex. The target itself may still be cut back to the road edge, which only
            // brings the line nearer the gate he is already on.
            if (Math.Abs(hisLane + inward * clearance) > reachable) return false;

            ActiveManeuver.Type = ManeuverType.DefendLane;
            ActiveManeuver.Target = defender;
            ActiveManeuver.LastEnabled = Game.GameTime;
            _defendApexNode = apexNode;

            // His lane, one combined width towards the corner's inside, so the line sits nearer the apex than he does.
            // Taken once when the card is played: a defence is a promise about where the car will be, not a chase.
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

        static Vector3 RotateZ(Vector3 v, float degrees)
        {
            float rad = degrees * (float)Math.PI / 180f;
            float cos = (float)Math.Cos(rad);
            float sin = (float)Math.Sin(rad);
            return new Vector3(v.X * cos - v.Y * sin, v.X * sin + v.Y * cos, v.Z);
        }

        // Input visuals are parked: the steer fan and the throttle and brake bars that hang off it are off, while the pedal
        // trail of chevrons stays. Flip this one flag to bring all three back; no setting has to change.
        const bool DebugInputVisuals = false;

        void DrawDebug()
        {
            if (ControlledByPlayer || ARS.DebugFocusRacer != this) return;

            if (ARS.DebugToggles[Options.ShowInputs]) DrawInputTrail();
            if (DebugInputVisuals && ARS.DebugToggles[Options.ShowTrackAnalysis]) DrawSteerAngles();
            if (ARS.DebugToggles[Options.ShowWheelSlip]) DrawWheelSlip();
            if (ARS.DebugToggles[Options.ShowFrontSlip]) DrawFrontSlipReadout();
            if (ARS.DebugToggles[Options.ShowGsProjection]) DrawGsProjection();
        }

        const float FanLineLength = 5f;
        const float FanPursuitHeight = 0.05f;
        const float FanDamperHeight = 0.10f;
        const float FanSteerHeight = 0.15f;
        const float FanCeilingHeight = 0.20f;

        void DrawSteerAngles()
        {
            Vector3 origin = Car.Position + new Vector3(0, 0, VehicleData.ModelDimensions.Z + 0.2f);
            Vector3 fwd = Car.ForwardVector;

            ARS.DrawLine(origin + new Vector3(0, 0, FanPursuitHeight), origin + new Vector3(0, 0, FanPursuitHeight) + RotateZ(fwd, _steerPursuitDeg) * FanLineLength, Color.Yellow);

            ARS.DrawLine(origin + new Vector3(0, 0, FanDamperHeight), origin + new Vector3(0, 0, FanDamperHeight) + RotateZ(fwd, _steerPursuitDeg + _damperTermDeg) * FanLineLength, Color.Red);

            Vector3 steerBase = origin + new Vector3(0, 0, FanSteerHeight);
            Vector3 steerDir = RotateZ(fwd, Control.SteerDegrees);
            ARS.DrawLine(steerBase, steerBase + steerDir * FanLineLength, Color.White);

            DrawSteerLineSphere(steerBase, steerDir, FanLineLength, Math.Abs(Control.Throttle), SteerLinePedalSize, Color.Green);
            DrawSteerLineSphere(steerBase, steerDir, FanLineLength, Control.Brake, SteerLinePedalSize, Color.Red);
            DrawPedalLimits(steerBase, steerDir, FanLineLength);

            float ceilingDeg = ResolveSteerCeiling(ARS.GetForwardSpeed(Car));
            ARS.DrawLine(origin + new Vector3(0, 0, FanCeilingHeight), origin + new Vector3(0, 0, FanCeilingHeight) + RotateZ(fwd, ceilingDeg) * FanLineLength, Color.Cyan);
            ARS.DrawLine(origin + new Vector3(0, 0, FanCeilingHeight), origin + new Vector3(0, 0, FanCeilingHeight) + RotateZ(fwd, -ceilingDeg) * FanLineLength, Color.Cyan);
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

        const float SteerLinePedalSize = 0.08f;

        void DrawSteerLineSphere(Vector3 basePt, Vector3 dir, float length, float level, float size, Color color)
        {
            World.DrawMarker(MarkerType.DebugSphere, basePt + dir * length * ARS.Clamp(level, 0f, 1f), Vector3.Zero, Vector3.Zero, new Vector3(size, size, size), color);
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
        }

        void DrawFrontSlipReadout()
        {

            if (!ARS.SlipFromMemory)
            {
                float target = _clampTargetDeg;
                float applied = _chasingClampDeg;
                if (float.IsNaN(target) || float.IsNaN(applied) || target < 0.01f)
                {
                    float forwardSpeed = Vector3.Dot(Car.Velocity, Car.ForwardVector);
                    float speedMph = ARS.MpsToMph(Math.Abs(forwardSpeed));
                    string state = forwardSpeed <= 0f ? "inactive - stopped"
                        : speedMph < 7f ? "inactive - below 7 mph"
                        : !_slip.Readable ? "no wheel data"
                        : "inactive - straight";
                    UI.ShowSubtitle("Slip vs Allowed~n~" + state, 50);
                    return;
                }
                UI.ShowSubtitle("Slip vs Allowed~n~x" + (applied / target).ToString("0.0")
                    + " of " + target.ToString("0.0") + "°", 50);
                return;
            }

            float clamp = float.IsNaN(_chasingClampDeg) ? _finalCeilingDeg : _chasingClampDeg;
            float asked = Math.Abs(_steerPreLimitDegrees);
            float mph = ARS.MpsToMph(Math.Abs(Vector3.Dot(Car.Velocity, Car.ForwardVector)));
            float share = Math.Max(0.05f, ARS.SteerCeilingFactor) * SlowCornerSlipShare(ARS.MphToMps(mph));

            UI.ShowSubtitle((clamp > 0.01f ? asked / clamp : 0f).ToString("0.0") + "   "
                + _slipFactor.ToString("0.00") + "   " + mph.ToString("0") + "mph"
                + "   F " + _slip.InnerPct.ToString("0") + "   R " + _slip.RearPct.ToString("0")

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

        const float PedalLimitReasonSize = 0.1f;
        const float PedalLimitCapSize = 0.09f;

        static readonly Color NonDangerousReasonColor = Color.FromArgb(255, 120, 200, 255);
        static readonly Color GripReasonColor = Color.Yellow;
        static readonly Color OverspeedReasonColor = Color.Black;
        static readonly Color CountersteerReasonColor = Color.Orange;
        static readonly Color StabilityReasonColor = Color.FromArgb(255, 190, 80, 255);
        static readonly Color RecoveryReasonColor = Color.Cyan;


        // The lane's colour says what it is doing, not who asked: grey with nothing asking, blue with an aim that is not
        // moving, green while it moves inward, yellow while it moves outward.
        // Blue means "asking, but the aim is not going anywhere": the same rate the mover treats as quiet in its own log,
        // so the two never disagree about whether the lane is moving.
        const float LaneStillMps = 0.2f;

        public static Color LaneMotionColor(LaneReason reason, float rateMps, int cornerDir)
        {
            if (reason == LaneReason.OwnLane) return Color.Gray;
            if (cornerDir == 0 || Math.Abs(rateMps) < LaneStillMps) return Color.DodgerBlue;
            return rateMps * cornerDir < 0f ? Color.Lime : Color.Yellow;
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
            VehicleData.BoundingBox = ARS.SlidingBoundingBoxWidth(Car);

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

                Car.HandbrakeOn = Control.HandBrakeTime > Game.GameTime;

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
                _overspeedPowerLevel = GlideCap(_overspeedPowerLevel, OverspeedThrottleLevel());
                counter *= _overspeedPowerLevel;
            }

            _powerCounter = counter;

            if (ControlledByPlayer && _overspeedPowerLevel < 0.99f && Game.GameTime > _powerCounterLoggedAt)
            {
                _powerCounterLoggedAt = Game.GameTime + 2000;
                ARS.Log(ARS.LogImportance.Info, "Player power cut " + _overspeedPowerLevel.ToString("0.00") + " (excess Gs " + VehicleData.OverspeedExcessGs.ToString("0.00") + ")");
            }

        }

        void ApplyPowerMultiplier()
        {

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

            int steerRef = (int)SteerLookaheadNodes(speed);
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

        float RouteRadiusSampled()
        {
            int count = ARS.TrackPoints.Count;
            if (count < 10) return 999f;

            float traction = Math.Max(VehicleData.CurrentMechanicalGrip, 0.1f);

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

                r = 999f;
            }
            else if (Sagitta(p0, p1, p2, p3, chord) < RouteStraightSagittaShare * chord)
            {

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

        const float ApexBufferSeconds = 5f;
        const float EntranceBrakeBufferSeconds = 0.25f;
        const float EntranceBrakeExtraDistance = 0f;
        const float SecondaryApexSpeedDifference = 5f;

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

                if (HasPassedApex(corner.Node))
                {
                    _skippedApexes.Remove(corner.Node);
                }

                if (distance <= 0) continue;

                bool tight = corner.DetectedRadius < CornerTightRadius;

                float intended = ApexSpeedWithDownforce(corner.DetectedRadius);
                float timeToEntrance = distance / speed;
                float decel = Math.Max(BrakingDecelBase(corner.Node), 0.1f);
                float brakingDistance = (speed * speed - intended * intended) / (2f * decel);
                bool inBrakingRange = distance <= brakingDistance + BrakeHorizonMargin;

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

        // The corner the line works is let go once the car is this many nodes past its apex: until then it keeps the reference
        // it earned on the way in, and after it the label and the next corner's stickiness move on together.
        const int ApexAdvanceNodes = 10;

        void AdvancePastApex()
        {
            if (_lineCornerNode < 0 || !HasPassedApex(_lineCornerNode)) return;
            int count = ARS.TrackPoints.Count;
            int since = CurrentTrackPoint.Node - _lineCornerNode;
            if (!ARS.IsPointToPoint) since = ((since % count) + count) % count;
            if (since <= ApexAdvanceNodes) return;
            _lineCornerNode = -1;
            CornerPoint nextAhead = NearestApexAhead();
            _lineCornerNode = nextAhead != null ? nextAhead.Node : -1;
        }

        bool HasPassedApex(int apexNode)
        {
            if (apexNode < 0) return false;
            if (ARS.IsPointToPoint) return CurrentTrackPoint.Node >= apexNode;

            return ForwardNodeDistance(apexNode) > ARS.TrackPoints.Count / 2;
        }

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

            float rawDistance = !ARS.IsPointToPoint && targetDistance > apexDistance
                ? 0f
                : targetDistance;
            if (rawDistance < 0f) rawDistance = 0f;
            float distance = rawDistance > 0f
                ? Math.Max(0f, rawDistance
                    - Car.Velocity.Length() * EntranceBrakeBufferSeconds
                    - EntranceBrakeExtraDistance)
                : 0f;

            float decel = BrakingDecel(apexNode, distance);

            float spd = (float)Math.Sqrt(velTarget * velTarget + 2f * decel * distance);
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
            float decel = brakingAbility * Handling.Gravity * EffectiveBrakeFactor(apexNode);
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
                sum += ARS.TrackPoints[sample].Elevation;
            }
            return sum / samples * ElevationToSlopeSine;
        }

        float RouteIdealSpeedForRadius(float r)
        {
            if (float.IsNaN(r) || float.IsInfinity(r)) r = 999f;
            r = ARS.Clamp(r, 5f, 999f);
            return (float)Math.Sqrt((VehicleData.CurrentMechanicalGrip * Handling.Gravity) * r);
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

            if (_halfSecondTick + _phaseOffsetMs < now)
            {
                _halfSecondTick = now + 500 + (int)ARS.Remap(Car.Velocity.Length(), 0, 100, -250, 250, true);
            }

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

                    if (ARS.Autofix == AutofixMode.AutoRepair)
                    {

                        float bodyFloor = Math.Max(Car.MaxHealth, 1) * 0.5f;
                        if (Car.EngineHealth < EngineHealthHalf || Car.BodyHealth < bodyFloor)
                        {

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

        float NearestEdgeLane(float steerRefOffset, float laneLimit)
        {
            return steerRefOffset >= 0f ? laneLimit : -laneLimit;
        }

        float KickCloseSeconds()
        {
            if (float.IsNaN(VehicleData.SteeringLock) || float.IsInfinity(VehicleData.SteeringLock)) return 0f;
            float lockRad = ARS.DegToRad(VehicleData.SteeringLock);
            float yawDegPerSecAtLock = ARS.RadToDeg(ARS.GetForwardSpeed(Car) * (float)Math.Tan(lockRad) / Math.Max(VehicleData.WheelBase, 0.1f));
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
