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
        public float TrackProgress = 0f;
        public int RaceProgress = 0;

        // Time-based route references used by steering and speed calculations.
        public enum LookAhead { SteerRef, QuarterSec, HalfSec, ThreeQuarterSec, OneSec, OneHalfSec, TwoSec };
        public Dictionary<LookAhead, TrackPoint> LookAheads = new Dictionary<LookAhead, TrackPoint>();


        public List<TimeSpan> LapTimes = new List<TimeSpan>();
        public int LapStartTime = 0;
        public int Lap = 0;
        public int NitroChargedLap = -1;
        public int RacePosition = 0;
        // Frozen finish rank, assigned once when the racer crosses the line; 0 = still racing.
        public int FinalPosition = 0;
        public bool CanRegisterNewLap = false;
        int _previousNode = -1;
        public bool FinishedPointToPoint = false;


        int _halfSecondTick = 0;
        int _oneSecondTick = 0;
        int _phaseOffsetMs = 0;
        int _pressureTick = 0;
        int _apexUpdateTick = 0;
        int _rivalInfoTick = 0;
        int _lastCoreTick = 100;
        int TimeSince_lastCoreTick => (int)ARS.Clamp(Game.GameTime - _lastCoreTick, 1, 9999);


        int _lastStuckGameTime = 0;
        List<TrackPoint> _trackPositionScratch = new List<TrackPoint>(13);
        public bool IsStuckByThrottle = false;
        const int StuckCheckTimeMs = 800;
        bool _isRecoveringFromStuck = false;
        int _stuckRecoveryEndTime = 0;
        const int StuckRecoveryTimeMs = 1000;
        int _stuckRecoveryCooldownEndTime = 0;
        const int StuckRecoveryCooldownMs = 2000;
        int _stuckRecoveryAttempts = 0;
        int _stuckReverseUntil = 0;
        int _regainedControlSince = 0;
        // Backing off is a short, straight phase; the way out is the forward phase after it, which runs until the safe
        // predicate holds rather than for a fixed time.
        const int StuckReverseMs = 500;
        const int RegainedControlHoldMs = 500;
        const float StuckRouteAimMeters = 10f;
        const float RecoveredHeadingDeg = 25f;
        public int StuckRecoveryAttemptsNow => _stuckRecoveryAttempts;
        public bool IsRecoveringFromStuckNow => _isRecoveringFromStuck;
        const float RecoveredMinSpeedMph = 20f;
        const float RecoveredSlideFraction = 0.3f;


        public float RouteLookAheadSeconds = 0.5f;
        public float RouteLookaheadSizeSeconds = 2.0f;
        // The steer reference is a preview *time*: a fixed distance raises the loop's natural frequency with speed.
        const float SteerPreviewSeconds = 0.5f;
        const float SteerLookaheadMinMeters = 8f;
        const float SteerLookaheadMaxMeters = 30f;
        // The Gs-aware preview: how much of the car's projected lateral motion leads the lane error.
        const float GsPreviewBlend = 0.5f;


        float _avoidLeftWall = 0f;
        float _avoidRightWall = 0f;
        int _activeRivalWallCount = 0;
        bool _avoidWallsInitialized = false;
        float _targetLane = 0f;
        public float TargetLane { get { return _targetLane; } }
        Vector3 _debugLaneAimPoint = Vector3.Zero;
        float _rawCornerLane = 0f;

        float _cornerSpd = 999f;

        // Latched on corner approach when entry speed warrants holding the outside line.
        int _activeApexNode = -1;
        float _activeApexRadius = 0f;
        float _activeApexDirection = 0f;
        int _passedApexNode = -1;
        float _passedApexRadius = 0f;
        float _passedApexDirection = 0f;
        int _divebombApexNode = -1;
        int _defendApexNode = -1;

        const float OffshootRangeMeters = 2f;
        const float OffshootBlendBrake = 0.25f; // brake floor at the outer limit
        // A node deflection below this is a straight, which has no outside to judge.
        const float OffshootMinRouteAngleDeg = 2f;
        // Line discipline belongs to speed: below this the problem is traction, and a cap would hold an off-edge car
        // at zero, because the projection of a stationary car is the car.
        const float OffshootMinSpeedMph = 20f;
        const float FullPedalSpeedErrorMps = 3f;
        // Braking is the softer side: it takes this many times the speed error to command full brake.
        const float BrakeErrorMultiplier = 2f;

        // Applied pedal input sampled every metre of travel, for the debug trail.
        readonly List<InputTrailSample> _inputTrail = new List<InputTrailSample>();
        const int InputTrailMaxSamples = 40;


        // Public so a rule can grant an allowance to one side and the debug view can draw them. LEFT bounds positive
        // commands and RIGHT negative ones, because a positive command steers left (AGENTS.md's steer-sign gotcha).
        public float SteerLimitRight = 40f;
        public float SteerLimitLeft = 40f;
        // Yaw damper term in degrees: read by the steer sum below and by the parked yaw HUD when re-armed.
        float _debugDamperTermDeg = 0f;
        float _debugYawTargetPerSecond = 0f;
        float _debugDamperGainSeconds = 0f;
        float _steerPursuitDeg = 0f;
        float _steerAimCurvature = 0f;
        // Last off-track projection cap, kept for the pedal bar's override sphere.
        float _offtrackInputCap = 1f;

        // Brake learning (Phase 1): learn the effective decel factor per corner apex.
        const float BrakeFactorSeed = 0.75f;
        // The radius under which a corner counts as tight, i.e. where the outside line is worth having.
        const float CornerTightRadius = 50f;
        // Arrival is extrapolated from the live forward Gs at this discount: holding a G flat across the window
        // over-predicts, because acceleration tapers with speed. Tuned by driving.
        const float RequirementExtrapolationScale = 0.9f;
        const float PositionExcessMph = 10f;
        const float BrakeExcessMph = 25f;
        // Braking is judged from the distance it actually needs, plus this much road for the plan to take hold.
        const float BrakeHorizonMargin = 40f;
        // Each corner's initial factor is the seed plus a draw of +/- this many hundredths.
        const int BrakeFactorSeedJitterHundredths = 5;
        readonly Dictionary<int, CornerContext> _cornerContexts = new Dictionary<int, CornerContext>();
        int _cornersRevisionSeeded = -1;
        float _brakeSampleSeconds = 0f;      // sampled braking time: the denominator of the full-pedal share
        float _brakeSampleFullSeconds = 0f;  // of which, the time at full pedal: the numerator
        float _brakeSampleMaxSlideDeg = 0f;  // largest slip seen between this apex's entrance and the apex itself
        int _brakeSampleApexNode = -1;
        // The commit is deferred past the exit, because a car that entered too hot slides on the way out and that is
        // the same verdict as sliding in. The sample is handed over when the apex is passed and then spends
        // BrakeCommitDelaySeconds gathering the exit's slide before it is allowed to teach anything.
        int _brakeCommitApexNode = -1;
        int _brakeCommitAtGameTime = 0;
        float _brakeCommitSampleSeconds = 0f;
        float _brakeCommitFullSeconds = 0f;
        float _brakeCommitMaxSlideDeg = 0f;
        const float BrakeSampleThreshold = 0.25f;   // only pedal above this counts as braking at all
        const float BrakeFullPedalThreshold = 0.9f; // pedal at or above this counts as full brake
        const float BrakeFullFractionTarget = 0.2f; // target share of the braking phase spent at full pedal
        const float BrakeAdjustGain = 1f;           // step per unit of share error: 0.1 for every 10 points off
        const float BrakeSkidFactorStep = 0.1f;     // a skidded corner costs this much factor, flat
        const float BrakeSkidPeakMultiple = 2.5f;   // skid gate sits at the peak x this, where grip hits its floor
        const float BrakeCommitDelaySeconds = 0.75f; // wait this long after the apex before the verdict lands
        const float BrakeMinFactor = 0.5f;       // learned factor range floor
        const float BrakeMaxFactor = 1.2f;
        // Read by ARS.MaxSpeedForBrakingDistance (static) to scale its decel plan.
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
            // A rebuilt table renumbers the route, so a remembered apex is a node of the old one.
            _activeApexNode = -1;
            _activeApexDirection = 0f;
            _passedApexNode = -1;
            _passedApexDirection = 0f;

            foreach (CornerPoint corner in ARS.Corners)
                if (!_cornerContexts.ContainsKey(corner.Node))
                    _cornerContexts[corner.Node] = new CornerContext { Point = corner, BrakeFactor = SeededBrakeFactor() };

            List<int> stale = new List<int>();
            foreach (int node in _cornerContexts.Keys)
                if (!ARS.Corners.Exists(corner => corner.Node == node)) stale.Add(node);
            foreach (int node in stale) _cornerContexts.Remove(node);
        }

        // One independent draw per corner, so a fresh grid does not brake every corner on one assumption.
        // The +1 makes the draw symmetric: GetRandomInt's max is exclusive.
        static float SeededBrakeFactor() => BrakeFactorSeed + ARS.GetRandomInt(-BrakeFactorSeedJitterHundredths, BrakeFactorSeedJitterHundredths + 1) / 100f;

        // While diving, a temp bonus is appended to the learned factor; the stored factor itself never changes.
        public float EffectiveBrakeFactor(int apexNode)
        {
            return ActiveManeuver.Type == ManeuverType.DiveBomb
                ? BrakeFactorForApex(apexNode) + _divebombBrakeBonus
                : BrakeFactorForApex(apexNode);
        }



        bool _isPassengerized = false;
        const float RivalSearchRangeMeters = 200f;

        // Feature gate for nitrous (ARS.NitrousEnabled).
        const float NitrousPowerMultiplier = 2.5f;
        const float NitrousMaxSteerDegrees = 5f;
        const float NitrousMinThrottle = 0.9f;
        const float NitrousCornerLookaheadSeconds = 8f;
        const float NitrousLonelySpeedFraction = 0.5f;
        const float NitrousLonelyMinApexDistance = 500f;
        const float NitrousDefenseReachSeconds = 4f;
        const float NitrousNearbyRivalDistance = 80f;
        const float NitrousFinishExtraDistance = 100f;
        const float ChillRivalCrowdDistance = 40f;
        const int ChillCrowdCount = 5;
        const float ChillStandoffDistance = 20f;
        const float ChillThrottleCap = 0.5f;
        const float ChillStandoffEscapeMargin = 2f;
        const float ChillMinSpeedMph = 30f;

        // Counts cars ahead on race progress only; cars behind don't crowd us.
        int RivalsWithinDistance(float distance)
        {
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
        const float LaunchShapeMaxSpeed = 12f;
        const float ModulatedThrottleCap = 0.99f;
        const float ModulatedThrottleLow = 0.05f;
        const float ModulatedThrottleHigh = 0.95f;
        const float LaunchThrottleBinarySplit = 0.1f;

        public Maneuver ActiveManeuver = new Maneuver();
        int _nitrousActiveUntil = 0;
        int _nitrousLapUsed = -1;
        float _powerCounter = 1f;
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

                if (ARS.SettingsMenuStore.GetInt("AIRacerAutofix", 2) == 2)
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
                else if (ARS.SettingsMenuStore.GetInt("AIRacerAutofix", 2) == 1)
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

            Function.Call(GTA.Native.Hash._0x0DC7CABAB1E9B67E, Car, true, 1);
            Function.Call(GTA.Native.Hash._0x0DC7CABAB1E9B67E, Driver, true, 1);
            Function.Call(GTA.Native.Hash.SET_ENTITY_PROOFS, Driver, true, true, true, false, true, true, 1, true);

            Driver.MaxHealth = 1000;
            Driver.Health = 1000;
            Driver.CanSufferCriticalHits = false;

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
            if (ControlledByPlayer) ARS.PlayerModulatesThrottle = false;
            Function.Call((Hash)CheatPowerIncreaseHash, Car, 1.0f);

            LapTimes.Clear();
            LapStartTime = 0;
            VehicleData.ResetLapPeaks();
            Lap = 0;
            NitroChargedLap = -1;
            _nitrousLapUsed = -1;
            RacePosition = 0;
            FinalPosition = 0;
            CanRegisterNewLap = false;
            _previousNode = -1;
            _inputTrail.Clear();
            _restHeightAboveGround = -1f;

            string flags = VehicleMemory.GetHandlingFlags(Car).ToString("X");
            int flagsHex = Convert.ToInt32(flags, 16);
            bool hasOffroad = (flagsHex & 0x800000) != 0 || (flagsHex & 0x200000) != 0;
            Handling.Gravity = 9.8f;
            if (hasOffroad) Handling.Gravity *= 1.2f;

            BaseBehavior = RacerBaseBehavior.GridWait;
            FinishedPointToPoint = false;

            Handling.Grip = Function.Call<float>((Hash)0xA132FB5370554DB0, Car);

            VehicleData.PerformanceIndex = (int)((Handling.EstimatedTopSpeed * 5) + (Handling.Grip * 100) + (Handling.Acceleration * 500));
            float modelGrip = Function.Call<float>((Hash)0x539DE94D44FDFD0D, Car.Model.Hash);
            float modelTopSpeedMph = ARS.MpsToMph(Function.Call<float>((Hash)0xF417C2502FFFED43, Car.Model.Hash));
            float modelAccel = Function.Call<float>(Hash.GET_VEHICLE_MODEL_ACCELERATION, Car.Model.Hash);
            // Cache first so the spawn-time PI matches the metric the grid was selected on; live probe only for an unscored car.
            bool modelElectric;
            if (!ARS.ModelElectricCache.TryGetValue(Car.Model.Hash.ToString(), out modelElectric)) modelElectric = ARS.IsElectricModel(Car.Model.Hash);
            VehicleData.PowerScale = ARS.ComputePaceIndex(modelTopSpeedMph, modelGrip, modelAccel, modelElectric);
            VehicleData.TextPerformanceIndex = VehicleData.PowerScale.ToString("0.00");
            if (!ControlledByPlayer) Name = _baseName + " (" + VehicleData.PowerScale.ToString("0.00") + ")";

            _cornerContexts.Clear();
            _cornersRevisionSeeded = -1;
            _activeApexNode = -1;
            _activeApexDirection = 0f;
            _passedApexNode = -1;
            _passedApexDirection = 0f;
            _brakeCommitApexNode = -1;

            Car.Repair();
        }

        public void ComputeSteering()
        {
            if (!TryGetSteerContext(out TrackPoint steerRefPoint, out float roadWide))
            {
                Control.SteerDegrees = 0f;
                return;
            }

            float speedMps = Math.Max(Car.Velocity.Length(), 1f);


            // --- Travel direction: the pursuit bearing is taken against it, and every term that steers shares it ---

            Vector3 courseDir = Car.ForwardVector;
            if (Car.Velocity.LengthSquared() > 0.01f) courseDir = Car.Velocity.Normalized;


            // --- Lane resolution: override chain (high-speed → corner → avoidance → walls) ---

            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            float drivableEdge = Math.Max(roadWide - carHalfWidth, 0f);
            float defaultLane = ComputeHighSpeedLane(roadWide, speedMps);
            bool gotActiveCorner = Brain.Corner != null && Lap > 0;
            float cornerLane = 0f;
            if (gotActiveCorner) cornerLane = ComputeCornerTargetLane(steerRefPoint, speedMps);
            if (cornerLane != 0f) defaultLane = cornerLane;
            _rawCornerLane = cornerLane;
            float avoidAheadLane = ComputeAvoidAheadLane(roadWide);
            if (avoidAheadLane != 0f) defaultLane = avoidAheadLane;
            // With no lane demand the car aims at its own live offset, so it holds its line instead of chasing the road
            // centre. Off the surface that flips to the centre - the only thing that turns an off-track car around,
            // with no recovery term behind it - or the aim would follow the car off the track.
            bool hasLaneDemand = defaultLane != 0f;
            float carOffset = ARS.SignedLaneOffset(Car.Position, steerRefPoint.Position, steerRefPoint.Direction);
            bool onTrack = Math.Abs(carOffset) <= drivableEdge;
            if (!hasLaneDemand) defaultLane = onTrack ? carOffset : 0f;
            float targetLane = ApplyRivalWalls(defaultLane, roadWide);
            if (ARS.DebugToggles[Options.LockLaneCentre]) targetLane = LaneLockTestOffsetMeters;
            _targetLane = targetLane;

            // Aim at the lookahead distance, offset by the target lane. The Gs-aware preview shifts
            // that offset by the lateral motion the car is already committing to, so the correction leads the drift
            // instead of reacting to it; shifting a real point keeps the pursuit's chord real.
            Vector3 steerRight = Vector3.Cross(steerRefPoint.Direction, Vector3.WorldUp).Normalized;
            float aimLane = targetLane;
            if (ARS.DebugToggles[Options.GsAwarePreview])
            {
                float laneAtProjection = ARS.SignedLaneOffset(ProjectAhead(SteerPreviewSeconds), steerRefPoint.Position, steerRefPoint.Direction);
                aimLane = targetLane - GsPreviewBlend * (laneAtProjection - carOffset);
            }
            // The aim targets the car's centre, so it locks to the drivable edge; the rival walls bound to the raw edge.
            aimLane = ARS.Clamp(aimLane, -drivableEdge, drivableEdge);
            _debugLaneAimPoint = steerRefPoint.Position + steerRight * aimLane;

            LogCornerCrossings();

            // --- Lane steer: pure pursuit toward the target lane ---

            float laneSteerDeg = PursuitSteerDegrees(_debugLaneAimPoint, courseDir);
            _steerPursuitDeg = laneSteerDeg;
            // Physical repulsion: inside the "no touching" box, steer away from rivals
            // actually closing laterally; parallel traffic must not kill the lane steer.
            Vector3 velDir = speedMps > 0.5f ? Car.Velocity / speedMps : courseDir;
            Vector3 velRight = Vector3.Cross(velDir, Vector3.WorldUp);
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || !r.RivalRacer.Car.Exists()) continue;
                Vector3 delta = r.RivalRacer.Car.Position - Car.Position;
                float longDist = Math.Abs(Vector3.Dot(delta, velDir));
                float latDist = Math.Abs(Vector3.Dot(delta, velRight));
                float longGate = (VehicleData.ModelDimensions.Y + r.RivalRacer.VehicleData.ModelDimensions.Y) * 0.5f + 1f;
                float latGate = (VehicleData.BoundingBox + r.RivalRacer.VehicleData.BoundingBox) * 0.5f + 3f;
                if (longDist > longGate || latDist > latGate) continue;
                Vector3 rivalVel = r.RivalRacer.Car.Velocity;
                if (rivalVel.LengthSquared() < 0.01f) continue;
                float latSide = Vector3.Dot(delta, velRight);
                float latRelVel = Vector3.Dot(rivalVel - Car.Velocity, velRight);
                if (latSide * latRelVel >= 0f) continue;
                float dist = delta.Length();
                float distScale = ARS.Remap(dist, 6f, 2f, 0.5f, 2f, true);
                float strength = ARS.Remap(Math.Abs(latRelVel), 0.3f, 3f, 0f, 15f, true) * distScale;
                laneSteerDeg += Math.Sign(latSide) * strength;
            }


            // --- Heading assist: match a side-by-side rival's heading ---

            float sideBySideSteerDeg = ComputeSideBySideSteerCorrection(courseDir);


            // --- Assembly: lane pursuit + corrections + damper + slide blend ---

            const float steerKP = 1.0f;
            // Damp the excess over the yaw the aim point requires, not over zero which taxes every steady corner; the
            // slide blend keeps the zero reference, since its wanted rotation is the countersteer's.
            float fwdSpeed = ARS.GetForwardSpeed(Car);
            float yawTarget = 0f;
            if (SteerDampingAimReference && fwdSpeed > 0f && Math.Abs(VehicleData.SlideAngle) < Handling.LateralTractionCurve * CountersteerBlendStartFraction) yawTarget = ARS.RadToDeg(fwdSpeed * _steerAimCurvature);
            float yawRateToDamp = VehicleData.YawRotationPerSecondDegrees - yawTarget;
            float damperGain = SteerDamping;
            _debugYawTargetPerSecond = yawTarget;
            _debugDamperGainSeconds = damperGain;
            _debugDamperTermDeg = -damperGain * yawRateToDamp;
            float nonLaneSteerDeg = (steerKP * sideBySideSteerDeg) + _debugDamperTermDeg;
            Control.SteerDegrees = nonLaneSteerDeg + (steerKP * laneSteerDeg);

            if (Handling.LateralTractionCurve > 1f)
            {
                float forwardMs = ARS.GetForwardSpeed(Car);
                if (forwardMs >= 2f)
                {
                    float slidePriority = ARS.Remap(Math.Abs(VehicleData.SlideAngle), Handling.LateralTractionCurve * CountersteerBlendStartFraction, Handling.LateralTractionCurve * CountersteerFullFraction, 0f, 1f, true);
                    if (slidePriority > 0f)
                    {
                        // Countersteer equals the slide angle - the correction term only, not the slidePriority ramp.
                        float countersteerTarget = nonLaneSteerDeg - VehicleData.SlideAngle;
                        Control.SteerDegrees += (countersteerTarget - Control.SteerDegrees) * slidePriority;
                    }
                }
            }

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
                if (Math.Abs(relativeOffset.Y) > rival.CombinedSize.Y) continue;

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
                float headingDifference = -Vector3.SignedAngle(rival.RivalRacer.Car.ForwardVector, courseDir, Vector3.WorldUp);
                if (float.IsNaN(headingDifference) || float.IsInfinity(headingDifference)) continue;

                correction += headingDifference * proximity;
            }
            return correction;
        }

        // TEMPORARY diagnostic: logs entrance, apex and exit per corner with the car's offset, so a corner that is
        // consistently missed can be read off the log instead of the screen. Remove with its call in ComputeSteering.
        const int CornerLogApexBandNodes = 2;
        int _logCornerIndex = -1;
        int _logCornerPhase = -1;

        int ForwardNodes(int fromNode, int toNode)
        {
            int count = ARS.TrackPoints.Count;
            int d = toNode - fromNode;
            if (!ARS.IsPointToPoint) d = ((d % count) + count) % count;
            return d;
        }

        void LogCornerPhase(int index, string phase)
        {
            CornerPoint c = ARS.Corners[index];
            ARS.Log(ARS.LogImportance.Info, "[CORNER] " + (index + 1) + "/" + ARS.Corners.Count
                + " apex=" + c.Node + " sign=" + Math.Sign(c.Angle)
                + " R=" + c.SupposedRadius.ToString("0") + " Rdet=" + c.DetectedRadius.ToString("0")
                + " " + phase + " off=" + Brain.CurrentPerception.DeviationFromCenter.ToString("0.00")
                + " steer=" + Control.SteerDegrees.ToString("0.0"));
        }

        void LogCornerCrossings()
        {
            if (ARS.Corners.Count == 0 || ARS.TrackPoints.Count == 0) return;
            int node = CurrentTrackPoint.Node;
            int foundIndex = -1;
            int foundPhase = -1;
            for (int i = 0; i < ARS.Corners.Count; i++)
            {
                CornerPoint c = ARS.Corners[i];
                if (c == null || c.Node < 0) continue;
                int entrance = CornerEntranceNode(c, c.Node);
                int exit = CornerExitNode(c);
                if (entrance < 0 || exit < 0) continue;
                int along = ForwardNodes(entrance, node);
                if (along > ForwardNodes(entrance, exit)) continue;
                int toApex = ForwardNodes(entrance, c.Node);
                foundIndex = i;
                foundPhase = along < toApex - CornerLogApexBandNodes ? 0 : (along <= toApex + CornerLogApexBandNodes ? 1 : 2);
                break;
            }
            if (foundIndex < 0)
            {
                if (_logCornerIndex >= 0)
                {
                    LogCornerPhase(_logCornerIndex, "EXIT");
                    _logCornerIndex = -1;
                    _logCornerPhase = -1;
                }
                return;
            }
            if (foundIndex != _logCornerIndex)
            {
                _logCornerIndex = foundIndex;
                _logCornerPhase = foundPhase;
                LogCornerPhase(foundIndex, "ENTRANCE");
                return;
            }
            if (foundPhase == 1 && _logCornerPhase != 1)
            {
                _logCornerPhase = foundPhase;
                LogCornerPhase(foundIndex, "APEX");
            }
        }

        // Lane Control System 2: positions the car on the inside edge of the track curvature.
        const float HighSpeedLaneRadiusMeters = 500f;

        // Debug lock for the centring test: a near-centre aim offset, pinned for every racer.
        const float LaneLockTestOffsetMeters = 0.1f;

        // Pure pursuit's steer carries a factor of two the plain bearing misses: the curvature to a point at a given
        // bearing is 2 sin(angle) over the distance, not the angle over it. This is the knob if the lane is still shy.
        const float PursuitGain = 2f;
        // sin falls again past a right angle, so an aim point abeam or behind the car would fade instead of saturating.
        const float MaxPursuitBearingDegrees = 90f;

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
            float distance = new Vector3(toAim.X, toAim.Y, 0f).Length();
            if (distance <= 0.5f) return 0f;
            float bearing = Vector3.SignedAngle(heading, toAim, Vector3.WorldUp);
            if (float.IsNaN(bearing) || float.IsInfinity(bearing)) return 0f;
            _steerAimCurvature = 2f * (float)Math.Sin(ARS.DegToRad(bearing)) / distance;
            return PursuitSteerFromBearing(bearing, distance);
        }

        float ComputeHighSpeedLane(float roadWide, float speedMps)
        {
            int count = ARS.TrackPoints.Count;
            int fwdNode;
            int fwdOffset = (int)(speedMps * 1.01f);
            if (ARS.IsPointToPoint)
                fwdNode = (int)ARS.Clamp(CurrentTrackPoint.Node + fwdOffset, 0, count - 1);
            else
                fwdNode = ((CurrentTrackPoint.Node + fwdOffset) % count + count) % count;

            TrackPoint ahead = ARS.TrackPoints[fwdNode];
            if (!(ahead.PreciseCurveRadius < HighSpeedLaneRadiusMeters)) return 0f;

            float cornerDir = Math.Sign(ahead.Angle);
            if (cornerDir == 0f) return 0f;

            // The edge, absolute: a target that steps from wherever the car is holds a constant error and so never
            // converges on the line. The gentleness belongs in the bearing, which grows as the car falls short.
            float insideBound = Math.Max(roadWide - VehicleData.BoundingBox * 0.5f, 0f);
            return -cornerDir * insideBound;
        }

        // How far ahead of a corner the car decides whether it wants to position or brake for it.
        const float RequirementLookaheadSeconds = 3.95f;

        // The chord meets the outside edge at the lead, and a car cannot change heading at the rate that implies, so
        // the blend is stretched past the geometric lead: the endpoints stay where the chord put them, the rate of
        // change falls as one over the multiple, and the path's curvature as one over its square.
        const float IdealLineLeadMultiple = 2f;

        // The ideal line is the widest one that still grazes the inside edge at the apex, and widening the arc until it
        // touches the outside answers that with a straight chord - whose outer end meets the outside edge at an angle,
        // which puts a corner in the target exactly where the car is settling onto it. So the chord supplies the two
        // ends - the outside edge a lead before the apex, the inside edge at it - and the blend between them leaves and
        // meets both edges flat, which is the part a car at speed can actually follow.
        float IdealLineOffset(int distanceFromApex, float apexRadius, float safeBound)
        {
            float innerRadius = apexRadius - safeBound;
            if (!(innerRadius > 0f) || apexRadius >= 999f) return 0f;
            float leadAngle = (float)Math.Acos(ARS.Clamp(innerRadius / (apexRadius + safeBound), -1f, 1f));
            float t = ARS.Clamp(Math.Abs(distanceFromApex) / Math.Max(apexRadius * leadAngle * IdealLineLeadMultiple, 1f), 0f, 1f);
            return safeBound * (2f * t * t * (3f - 2f * t) - 1f);
        }

        float ComputeCornerTargetLane(TrackPoint steerRefPoint, float speedMps)
        {
            CornerPoint c = Brain.Corner.Point;
            int apexNode = c.Node;

            // The line is a function of the distance to the apex, so no time enters the geometry - and it is measured
            // at the point the offset is applied to, or the profile is one lookahead stale. The reference sits ahead
            // of the car, though, so in the last lookahead before an apex it is already past one while the car is
            // not: falling back to the car's own distance there keeps the entry branch driving to the inside, where
            // reading the reference would open the profile outward at the apex.
            int fwdToApex = apexNode - steerRefPoint.Node;
            if (fwdToApex < 0) fwdToApex = ForwardNodeDistance(apexNode);

            float cornerDir = Math.Sign(c.Angle);
            if (cornerDir == 0f) return 0f;

            if (apexNode != _activeApexNode)
            {
                _passedApexNode = _activeApexNode;
                _passedApexRadius = _activeApexRadius;
                _passedApexDirection = _activeApexDirection;
                _activeApexNode = apexNode;
                _activeApexRadius = c.DetectedRadius;
                _activeApexDirection = cornerDir;
            }

            float halfWidth = steerRefPoint.TrackHalfWidth;
            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            float safeBound = Math.Max(halfWidth - carHalfWidth, 0f);

            // Corner-commit: hold the defend/dive line for the card's whole life, not just the outside phase.
            bool isCornerCommit = ActiveManeuver.Target != null && (ActiveManeuver.Type == ManeuverType.DefendLane || ActiveManeuver.Type == ManeuverType.DiveBomb);
            if (isCornerCommit)
            {
                Rival target = Brain.Rivals.FirstOrDefault(r => r.RivalRacer == ActiveManeuver.Target);
                if (target != null && target.RivalRacer.Car.Exists())
                {
                    float gap = target.OccupiedLaneWidth + 0.6f;
                    float commitLane = target.OccupiedLane + (-cornerDir) * gap;
                    return ARS.Clamp(commitLane, -safeBound, safeBound);
                }
            }

            // The car decides: a corner it has not asked to position for keeps the inside line, and so does one it
            // arrives at far below its own apex speed.
            bool wantsPosition = TryGetCornerContext(apexNode, out CornerContext apexContext) && apexContext.RequiresPositioning;
            bool aboveApexSpeed = speedMps > ApexSpeedWithDownforce(c.SupposedRadius) - ARS.MphToMps(20f);
            bool entryActive = wantsPosition && aboveApexSpeed;
            float offset = entryActive ? IdealLineOffset(fwdToApex, c.DetectedRadius, safeBound) : 0f;

            // A passed apex still owns its exit half until the profile has opened back to the outside edge, and the
            // next corner's own profile saturates at that same edge, so the handover is the lesser of the two. An
            // opposite-direction pair is a chicane: two profiles would demand opposite edges with no room to change
            // sides, so it keeps the apex aim instead.
            if (_passedApexDirection != 0f && _passedApexDirection == cornerDir)
            {
                int pastApex = steerRefPoint.Node - _passedApexNode;
                if (!ARS.IsPointToPoint && pastApex < 0) pastApex += ARS.TrackPoints.Count;
                float exitOffset = IdealLineOffset(pastApex, _passedApexRadius, safeBound);
                offset = entryActive ? Math.Min(offset, exitOffset) : exitOffset;
            }

            return cornerDir * offset;
        }

        // A rival ahead is only a pass target while the two velocity vectors sit within this angle; past it the
        // rival is not in a valid position to avoid, and the car falls back to follow-behind.
        const float AvoidAngleGateDegrees = 20f;
        // The wall and pass geometry only trust an adjacency when the pair also sits on neighbouring route nodes,
        // so an overlapping car on the far leg of a U is never mistaken for a trackside blockage.
        const float SameSectionMaxGapMeters = 30f;

        // Pick a lane to pass a rival ahead. If two rivals trigger on opposite sides, thread the needle.
        float ComputeAvoidAheadLane(float roadWide)
        {
            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            float trackBound = roadWide - carHalfWidth;
            float aggroBuffer = ARS.Remap(Aggression, 100f, 0f, 0.2f, 1.2f, true);
            float currentLaneMeters = Brain.CurrentPerception.DeviationFromCenter;

            Rival target = Brain.AvoidanceTarget;
            if (target == null || target.RivalRacer == null) return 0f;

            if (!TryPickAvoidanceSide(target, trackBound, aggroBuffer, carHalfWidth, currentLaneMeters, out float targetLane, out bool targetGoLeft))
                return 0f;

            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || r == target) continue;
                if (r.RelativePosition != RelativePos.Ahead) continue;
                if (r.RouteGapMeters > SameSectionMaxGapMeters) continue;
                if (!ARS.IsBetween(Math.Abs(r.DirectionDiff), 0f, AvoidAngleGateDegrees)) continue;
                if (!ARS.IsBetween(r.FrontGap, 0f, 3f) && !ARS.IsBetween(r.SecondsToHit, 0f, 5f)) continue;

                if (!TryPickAvoidanceSide(r, trackBound, aggroBuffer, carHalfWidth, currentLaneMeters, out float secondTarget, out bool secondGoLeft))
                    continue;

                if (secondGoLeft == targetGoLeft) continue;
                return (targetLane + secondTarget) * 0.5f;
            }

            return targetLane;
        }

        bool TryPickAvoidanceSide(Rival rival, float trackBound, float aggroBuffer, float carHalfWidth, float currentLaneMeters, out float passLane, out bool passLeft)
        {
            passLane = 0f;
            passLeft = false;

            float rivalLane = rival.OccupiedLane;
            float buffer = rival.OccupiedLaneWidth + aggroBuffer;

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
            const float SideFlipRadiusFloor = 200f;
            if (Math.Abs(passLane) > trackBound && Brain.CurrentPerception.CurveRadiusToFollowPoint > SideFlipRadiusFloor)
            {
                passLane = passLeft ? rivalLane + buffer + carHalfWidth : rivalLane - buffer - carHalfWidth;
                passLeft = !passLeft;
            }

            if (Math.Abs(passLane) > trackBound) return false;

            if (passLeft && currentLaneMeters <= passLane) return false;
            if (!passLeft && currentLaneMeters >= passLane) return false;

            return true;
        }

        float ApplyRivalWalls(float targetLane, float roadWide)
        {
            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            // The outer bound here is the raw track edge with no car-width inset; the per-rival walls below carry
            // both cars' widths instead.
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

                bool overlaps = Math.Abs(r.LongitudinalGap) < r.CombinedSize.Y && r.RouteGapMeters <= SameSectionMaxGapMeters;
                bool aheadAndClose = r.RelativePosition == RelativePos.Ahead && r.SecondsToReach < 3f && r.RouteGapMeters <= SameSectionMaxGapMeters && Math.Abs(r.DirectionDiff) <= AvoidAngleGateDegrees;
                if (!overlaps && !aheadAndClose) continue;

                float aggroBuffer = ARS.Remap(Aggression, 100f, 0f, 0.2f, 1.2f, true);
                float rivalBuffer = r.OccupiedLaneWidth + aggroBuffer;

                if (r.OccupiedLane <= 0f)
                {
                    targetLeftWall = Math.Max(targetLeftWall, r.OccupiedLane + rivalBuffer);
                    leftConstrained = true;
                }
                else
                {
                    targetRightWall = Math.Min(targetRightWall, r.OccupiedLane - rivalBuffer);
                    rightConstrained = true;
                }
            }

            float openRate = 10f * TickScale;
            _avoidLeftWall = leftConstrained ? targetLeftWall : Math.Max(_avoidLeftWall - openRate, -trackBound);
            _avoidRightWall = rightConstrained ? targetRightWall : Math.Min(_avoidRightWall + openRate, trackBound);
            _activeRivalWallCount = (_avoidLeftWall > -trackBound ? 1 : 0) + (_avoidRightWall < trackBound ? 1 : 0);

            // Rival walls must remain ordered; collapse overlap to a narrow centered corridor.
            if (_avoidLeftWall > _avoidRightWall)
            {
                float mid = (_avoidLeftWall + _avoidRightWall) * 0.5f;
                _avoidLeftWall = mid - 1f;
                _avoidRightWall = mid + 1f;
            }

            float clampLeft = Math.Max(_avoidLeftWall + carHalfWidth, -trackBound);
            float clampRight = Math.Min(_avoidRightWall - carHalfWidth, trackBound);

            if (clampLeft > clampRight) return ARS.Clamp(targetLane, -trackBound, trackBound);
            return ARS.Clamp(targetLane, clampLeft, clampRight);
        }

        // Countersteer blend: starts at TRlat × this, fully engaged at TRlat × the full fraction.
        const float CountersteerBlendStartFraction = 0.3f;
        const float CountersteerFullFraction = 0.6f;
        // Pedal level held at full countersteer: enough throttle to keep the wheels rolling and no brake.
        const float CountersteerRollThrottle = 0.05f;
        // Below this forward speed the velocity direction is numerical noise, so the slide angle means nothing.
        const float CountersteerMinSpeedMph = 10f;
        const float SteerSlewRate = 90f;
        const float SteerSlewRateCountersteer = 180f;
        // Kill switch for the yaw-rate damper. Driven with it off the cars cannot hold centre — the term is the
        // only thing opposing a rotation the course chain has already started, so it is load-bearing, not trim.
        const bool SteerDampingEnabled = true;
        // The damper's reference: the yaw the aim point requires removes the standing-offset toll, and the zero
        // reference stays one flip away for A/B — the drive that preferred it was confounded by the course error.
        const bool SteerDampingAimReference = true;
        float SteerDamping => SteerDampingEnabled ? ARS.SteerDampingGain / Math.Max(VehicleData.BaseMechanicalGrip, 1f) : 0f;
        // Vanilla's player steering limiter used as a ceiling (AGENTS.md pipeline step 4): vanilla divides by
        // 1 + 0.075 × (forward speed − 5) in m/s and skips it while the car is sliding. The 5 m/s shift and its
        // gate are deliberately dropped here, so the ceiling starts closing from a standstill instead of
        // holding full lock to ~11 mph. Vanilla's auto-centre term beside it is not applied.
        const float VanillaSteerReductionPerMps = 0.075f;
        // Grip is read as a ratio to this reference, floored so a low-grip car cannot blow the coefficient up.
        const float SteerCapGripReference = 1f;
        const float SteerCapGripFloor = 0.5f;
        const float PeakSlipOuterWheelCommandShare = 1f / 0.75f;
        // Below the ramp's end speed the ceiling eases back to the car's full lock, so a slow car can steer in
        // fully; above it the ceiling is the law's own. The band is in mph because that is how it is judged.
        const float SteerLimitRampStartMph = 5f;
        const float SteerLimitRampEndMph = 40f;

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
            float ratio = VehicleData.WheelBase * grip * 9.8f / (fwdSpeed * fwdSpeed);
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

        float GeometrySteerCeiling(float fwdSpeed)
        {
            float vanillaCeiling = VehicleData.SteeringLock / (1f + SteerReductionPerMps * fwdSpeed);
            return Math.Min(vanillaCeiling, AckermannCeilingDegrees(fwdSpeed));
        }

        float PeakSlipCeilingAt(float peakSlipDeg)
        {
            return ARS.Clamp(peakSlipDeg * PeakSlipOuterWheelCommandShare, 0f, VehicleData.SteeringLock);
        }

        // Maximum sustained yaw rate the car can hold at the current speed: the slip ceiling's radius from the
        // Ackermann relation, then v / R. Above ~100% the car is over-rotating — the slide blend or the limiter
        // owns what happens next. Returns 0 when no grip data is available yet (early init).
        float YawUsagePercent()
        {
            if (TRLateralAtSpeed <= 0.01f) return 0f;
            float fwdSpeed = ARS.GetForwardSpeed(Car);
            if (fwdSpeed <= 0.1f) return 0f;
            float steerLockRad = PeakSlipCeilingAt(TRLateralAtSpeed) * (float)Math.PI / 180f;
            if (steerLockRad <= 0.001f) return 0f;
            float turnRadius = VehicleData.WheelBase / (float)Math.Tan(steerLockRad);
            float maxYawRadPerSec = fwdSpeed / Math.Max(turnRadius, 1f);
            float maxYawDegPerSec = maxYawRadPerSec * 180f / (float)Math.PI;
            return Math.Abs(VehicleData.YawRotationPerSecondDegrees) / maxYawDegPerSec * 100f;
        }

        // The ceiling in force at this speed: the peak-slip cap, with the corner geometry as a fallback only when the
        // live peak reads unusable. Eased back towards full lock below the ramp's end speed, where it meets what
        // the car gets at that end speed anyway.
        float ResolveSteerCeiling(float fwdSpeed)
        {
            float ceiling = TRLateralAtSpeed > 0.01f ? PeakSlipCeilingAt(TRLateralAtSpeed) : GeometrySteerCeiling(fwdSpeed);
            float speedMph = ARS.MpsToMph(fwdSpeed);
            if (speedMph >= SteerLimitRampEndMph) return ceiling;

            float endSpeed = ARS.MphToMps(SteerLimitRampEndMph);
            float endPeak = LateralPeakAtSpeed(endSpeed);
            float endCeiling = endPeak > 0.01f ? PeakSlipCeilingAt(endPeak) : GeometrySteerCeiling(endSpeed);
            // Descending *input* with ascending output, because Remap's own clamp inverts a descending output.
            float ramped = ARS.Remap(speedMph, SteerLimitRampEndMph, SteerLimitRampStartMph, endCeiling, VehicleData.SteeringLock, true);
            // max() keeps the ramp a raise only: the straight line sits a degree under the curved law near 25 mph.
            return Math.Max(ceiling, ramped);
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

            // Two limits, one per side, closed by one clamp: nothing is exempt from being limited, so an allowance
            // is granted to a side rather than a check skipped. A reversing car keeps the raw lock — vanilla's
            // 1 + k × v goes negative below −13 m/s and would invert the ceiling.
            bool countersteering = Math.Sign(requestedSteer) != Math.Sign(VehicleData.YawRotationPerSecondDegrees);
            float speedCeiling = VehicleData.SteeringLock;
            if (fwdSpeed > 0f) speedCeiling = ResolveSteerCeiling(fwdSpeed);
            SteerLimitRight = speedCeiling;
            SteerLimitLeft = speedCeiling;

            // The one whitelisted allowance: the side answering a slide reaches past the ceiling up to the slide
            // angle itself — a raise, never a reduction.
            if (countersteering && Math.Abs(VehicleData.SlideAngle) >= Handling.LateralTractionCurve * CountersteerBlendStartFraction)
            {
                float countersteerAllowance = Math.Min(Math.Abs(VehicleData.SlideAngle), VehicleData.SteeringLock);
                if (requestedSteer > 0f) SteerLimitLeft = Math.Max(SteerLimitLeft, countersteerAllowance);
                else SteerLimitRight = Math.Max(SteerLimitRight, countersteerAllowance);
            }

            float yawRate = VehicleData.YawRotationPerSecondDegrees;
            if (fwdSpeed > 0f && requestedSteer * yawRate >= 0f)
            {
                float yawUsage = YawUsagePercent() * 0.01f;
                if (float.IsNaN(yawUsage) || float.IsInfinity(yawUsage)) yawUsage = 0f;
                float maximumShare = ARS.YawTurnInMaximumPercent * 0.01f;
                float minimumShare = Math.Min(ARS.YawTurnInMinimumPercent * 0.01f, maximumShare);
                float turnInShare = ARS.Remap(yawUsage, 0f, maximumShare, minimumShare, maximumShare, true);
                float turnInCeiling = Math.Min(speedCeiling * turnInShare, VehicleData.SteeringLock);
                float speedMph = ARS.MpsToMph(fwdSpeed);
                turnInCeiling = ARS.Remap(speedMph, SteerLimitRampEndMph, SteerLimitRampStartMph, turnInCeiling, VehicleData.SteeringLock, true);
                if (requestedSteer > 0f) SteerLimitLeft = Math.Min(SteerLimitLeft, turnInCeiling);
                else if (requestedSteer < 0f) SteerLimitRight = Math.Min(SteerLimitRight, turnInCeiling);
            }

            Control.SteerDegrees = ARS.Clamp(requestedSteer, -SteerLimitRight, SteerLimitLeft);
        }


        // True when the slide has saturated the countersteer blend (same threshold ComputeSteering uses).
        // Forward speed gates it: reversing reads as a ~180° slide, and a reversed car must never be starved.
        bool IsFullCountersteer()
        {
            if (Vector3.Dot(Car.Velocity, Car.ForwardVector) < ARS.MphToMps(CountersteerMinSpeedMph)) return false;
            return Math.Abs(VehicleData.SlideAngle) >= Handling.LateralTractionCurve * CountersteerFullFraction;
        }


        public void Launch()
        {
            // Open the launch window at the green: the player's input is judged during the launch itself.
            if (ControlledByPlayer) ARS.PlayerLaunchTestActive = true;

            Brain.Corner = null;
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
            LapStartTime = ARS.IsPointToPoint ? Game.GameTime : 0;
            VehicleData.ResetLapPeaks();
            CanRegisterNewLap = false;
            _previousNode = -1;
            // The grid is the one moment the car is known to be at rest, so this is where the baseline comes from.
            float restHeight = Car.HeightAboveGround;
            if (restHeight > 0f) _restHeightAboveGround = restHeight;
            Control.MaxThrottle = 1f;
            Control.MaxBrake = 1f;
            Control.MaxBrakeFromABS = 1f;
            Control.MaxBrakeFromCountersteer = 1f;
            Control.MaxThrottleFromTCS = 1f;
            Control.MaxThrottleFromInstability = 1f;
            Control.MaxThrottleFromOverspeed = 1f;
            Control.MaxThrottleFromRival = 1f;
            Control.MaxThrottleFromChillOut = 1f;
            Control.MaxThrottleFromYield = 1f;
            Control.ThrottleReason = ThrottleReason.Plan;
            Control.ThrottleReasonLevel = 1f;
            Control.BrakeReason = BrakeReason.Plan;
            Control.BrakeReasonLevel = 1f;
            IsStuckByThrottle = false;
            _lastStuckGameTime = 0;
            _isRecoveringFromStuck = false;
            _stuckRecoveryEndTime = 0;
            _stuckRecoveryCooldownEndTime = 0;
            _stuckRecoveryAttempts = 0;
            Control.LastAppliedSteerDegrees = 0f;
            if (TeamRole == Team.Cop) Car.SirenActive = true;

        }

        const float PedalSlewRate = 6f;             // pedal slew rate (units/second: full range in ~167ms)
        const float PedalSlewMaxPerTick = 0.5f;     // hitch guard: never step more than this in one tick

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
            float preBlendInput = combinedInput;
            _offtrackInputCap = OffshootInputCap(1f);
            combinedInput = Math.Min(combinedInput, _offtrackInputCap);
            bool offtrackLimited = combinedInput < preBlendInput;
            bool offtrackBrakeCommand = offtrackLimited && combinedInput < 0f;
            // Full countersteer: no brake, just enough throttle to keep the wheels rolling. The reason caps still
            // apply on top; the release is instant.
            bool countersteering = IsFullCountersteer();
            if (countersteering)
            {
                combinedInput = CountersteerRollThrottle;
            }
            SplitCombinedInput(combinedInput, ref newThrottle, ref newBrake);
            newThrottle = ComposeThrottleCap(newThrottle, offtrackLimited, countersteering);
            // Brake composes here on the raw split target, pre-slew: in the throttle-to-brake transition
            // Control.Throttle slews down slowly and could still read >= 1.0 while the car is already braking.
            newBrake = ComposeBrakeCap(newBrake, offtrackBrakeCommand, countersteering);
            bool rbFreeZonePedal = Lap <= 1 && CurrentTrackPoint != null && CurrentTrackPoint.Node < 500;
            if (ARS.RubberbandingPct > 0 && ARS.CurrentRubberbandMode == RubberbandMode.Artificial && newThrottle >= 1.00f && !rbFreeZonePedal)
            {
                float rbTorque = ComputeRubberBandFactor();
                if (rbTorque > 1f) newThrottle *= rbTorque;
            }

            Control.Brake += ARS.Clamp(newBrake - Control.Brake, -inputChange, inputChange);
            Control.Throttle += ARS.Clamp(newThrottle - Control.Throttle, -inputChange, inputChange);

            UpdateBrakeLearning();

            if (Brain.CurrentIntention.MaxSpeed < AiConstants.MaxSpeed) Brain.CurrentIntention.MaxSpeed += 15 * TickScale;

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
                Brain.CurrentIntention.Speed = Math.Min(Brain.CurrentIntention.Speed, Brain.CurrentIntention.MaxSpeed);
            }
        }

        // Projection response: on a corner's outside, cap the maximum combined input by how far off-centre
        // the projection lands — full throttle on the centre line, none at the track edge, light brake beyond.
        float OffshootInputCap(float seconds)
        {
            // Only meaningful when the car is aiming at a lane; with no target lane there is no
            // hug-inside expectation to enforce, so the outside sanity check must not fire.
            if (_targetLane == 0f) return 1f;

            if (ARS.MpsToMph(Car.Velocity.Length()) < OffshootMinSpeedMph) return 1f;

            Vector3 proj = ProjectAhead(seconds);
            // 1 node ≈ 1 m: the window has to reach the projected node.
            int projectionWindow = (int)(Car.Velocity.Length() * seconds) + 10;
            TrackPoint tp = ARS.FindNearestTrackPoint(proj, CurrentTrackPoint.Node, 0, projectionWindow);
            float signedOffset = ARS.SignedLaneOffset(proj, tp.Position, tp.Direction);
            float halfWidth = Math.Max(tp.TrackHalfWidth, 0.1f);

            // Outside is the side the corner the car is in turns away from.
            float turnDirection = Math.Abs(CurrentTrackPoint.Angle) > OffshootMinRouteAngleDeg ? Math.Sign(CurrentTrackPoint.Angle) : 0f;

            if (Math.Sign(signedOffset) != turnDirection) return 1f;

            // Inside half keeps full throttle; the cap ramps from the centre line to the edge, then brakes.
            float outsideOffset = signedOffset * turnDirection;
            if (outsideOffset > halfWidth) return -OffshootBlendBrake * Math.Min((outsideOffset - halfWidth) / OffshootRangeMeters, 1f);
            if (outsideOffset > 0f) return 1f - outsideOffset / halfWidth;
            return 1f;
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
                _brakeSampleApexNode = NextApexNode;
                _brakeSampleSeconds = 0f;
                _brakeSampleFullSeconds = 0f;
                _brakeSampleMaxSlideDeg = 0f;
            }

            // Runs on the corner proper, entrance to apex — exactly where the sample has stopped. A car that slid on
            // the way in was on a tyre that had already let go, so the commit below throws that sample away.
            if (_brakeSampleApexNode >= 0 && HasPassedBrakingTarget() && !HasPassedApex(_brakeSampleApexNode))
            {
                float slide = Math.Abs(VehicleData.SlideAngle);
                if (!float.IsNaN(slide) && slide > _brakeSampleMaxSlideDeg) _brakeSampleMaxSlideDeg = slide;
            }

            if (ActiveManeuver.Type != ManeuverType.None) return;

            if (HasPassedBrakingTarget()) return;

            if (Control.Brake <= BrakeSampleThreshold) return;
            _brakeSampleSeconds += TickScale;
            // The numerator is time at full pedal: the target is a share of the phase, not an amount.
            if (Control.Brake >= BrakeFullPedalThreshold) _brakeSampleFullSeconds += TickScale;
        }

        // Park the sample at the apex pass; the verdict lands BrakeCommitDelaySeconds later.
        void ScheduleBrakeCommit()
        {
            // Two apexes inside the delay (a chicane) settle the older one now rather than lose it.
            if (_brakeCommitApexNode >= 0) CommitBrakeLearning();
            _brakeCommitApexNode = _brakeSampleApexNode;
            _brakeCommitAtGameTime = Game.GameTime + (int)(BrakeCommitDelaySeconds * 1000f);
            _brakeCommitSampleSeconds = _brakeSampleSeconds;
            _brakeCommitFullSeconds = _brakeSampleFullSeconds;
            _brakeCommitMaxSlideDeg = _brakeSampleMaxSlideDeg;
        }

        void CommitBrakeLearning()
        {
            if (!ARS.BrakeLearning || _brakeCommitApexNode < 0) return;
            int apexNode = _brakeCommitApexNode;
            float sampleSeconds = _brakeCommitSampleSeconds;
            float maxSlideDeg = _brakeCommitMaxSlideDeg;
            float fullShare = sampleSeconds > 0f ? _brakeCommitFullSeconds / sampleSeconds : 0f;
            _brakeCommitApexNode = -1;
            // The notices ride the track-analysis debug view, so a normal race is quiet.
            bool announce = ARS.DebugToggles[Options.ShowTrackAnalysis];

            float before = BrakeFactorForApex(apexNode);
            // A slide is a verdict of its own, but it takes a real one: 2.5x the live peak is where the lateral
            // curve has fallen all the way to fTractionCurveMin, so that is sliding rather than merely cornering at
            // the limit. The factor then takes a flat cut — sample or no sample, the slide is the evidence.
            if (TRLateralAtSpeed > 0.01f && maxSlideDeg > TRLateralAtSpeed * BrakeSkidPeakMultiple)
            {
                float skidFactor = ARS.Clamp(before - BrakeSkidFactorStep, BrakeMinFactor, BrakeMaxFactor);
                CornerContextForWrite(apexNode).BrakeFactor = skidFactor;
                if (announce) UI.Notify("~b~[ARS]~w~ skid detected > " + skidFactor.ToString("0.00"));
                return;
            }
            // Nothing to score: the car never braked above the sample threshold on the way in.
            if (sampleSeconds <= 0f) return;
            // The error is the *share* of the braking phase spent at full pedal, not the length of the phase:
            // a corner the AI takes without ever pressing hard scores 0, so it asks for the largest correction.
            float step = (BrakeFullFractionTarget - fullShare) * BrakeAdjustGain;
            float factor = ARS.Clamp(before * (1f + step), BrakeMinFactor, BrakeMaxFactor);
            CornerContextForWrite(apexNode).BrakeFactor = factor;
            // The learned factor is otherwise invisible, and this is the only read-out of what the AI decided
            // braking that corner costs.
            if (announce) UI.Notify("~b~[ARS]~w~ " + (int)Math.Round(fullShare * 100f) + "% brake > " + factor.ToString("0.00"));
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
            CornerPoint corner = ARS.Corners.FirstOrDefault(c => c.Node == NextApexNode);
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
            bool countersteering = Math.Sign(Control.SteerDegrees) != Math.Sign(VehicleData.YawRotationPerSecondDegrees);
            float rate = countersteering ? SteerSlewRateCountersteer : SteerSlewRate;
            float maxDeltaPerTick = rate * TickScale;
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
            else if (Brain.Corner != null) cornerSpd = Math.Max(2, ARS.MaxSpeedForBrakingDistance(Brain.Corner.Point, this));

            // Route speed from the triple-check circumradius window.
            float followTrackSpd = RouteIdealSpeedForRadius(Brain.CurrentPerception.CurveRadiusToFollowPoint);

            if (float.IsNaN(cornerSpd) || float.IsInfinity(cornerSpd)) cornerSpd = 999f;
            if (float.IsNaN(followTrackSpd) || float.IsInfinity(followTrackSpd)) followTrackSpd = 999f;
            if (cornerSpd <= 5 && Brain.Corner != null) cornerSpd = ARS.CornerApexSpeed(Brain.Corner.Point, this);

            // Hold the apex braking plan until the braking target (entrance) is reached AND the car has
            // actually braked down to the corner speed; route speed takes over inside the corner.
            if (NextApexNode >= 0 && HasPassedBrakingTarget() && Car.Velocity.Length() <= NextApexSpeed + ARS.MphToMps(1f)) cornerSpd = 999f;

            // Hill grip loss: exponential model, 15 degrees halves grip.
            {
                float slopeAngleDeg = ARS.RadToDeg(GetFollowPointSlopeAngle());
                float slopeGripFactor = ARS.HillGripFactorFromPitchAngle(slopeAngleDeg, this);
                float slopeSpeedFactor = (float)Math.Sqrt(slopeGripFactor);
                cornerSpd *= slopeSpeedFactor;
                followTrackSpd *= slopeSpeedFactor;
            }

            // Crest/dip vertical curvature grip effect (route speed only).
            int count = ARS.TrackPoints.Count;
            int followNode = (int)ARS.Clamp(CurrentTrackPoint.Node + (int)(Car.Velocity.Length() * RouteLookAheadSeconds), 0, count - 1);
            followTrackSpd *= CrestGripSpeedFactor(OffsetCornerNode(followNode, -3), followNode, OffsetCornerNode(followNode, 3), Brain.CurrentPerception.CurveRadiusToFollowPoint, Car.Velocity.Length(), out _);

            // Pure apex speed for the corner-approach gate.
            _cornerSpd = NextApexNode >= 0 ? NextApexSpeed : (Brain.Corner != null ? ARS.CornerApexSpeed(Brain.Corner.Point, this) : 999f);
            float cornerApexSpeedWithVerticalGrip = _cornerSpd;

            // Corner crest/dip: same check as route, centered on the apex node.
            if (Brain.Corner != null)
            {
                CornerPoint crestCorner = Brain.Corner.Point;
                float cornerCrestFactor = CrestGripSpeedFactor(OffsetCornerNode(crestCorner.Node, -3), crestCorner.Node, OffsetCornerNode(crestCorner.Node, 3), NextApexRadius, _cornerSpd, out _);
                cornerSpd *= cornerCrestFactor;
                cornerApexSpeedWithVerticalGrip *= cornerCrestFactor;
            }

            // Steer-limited speed: max speed for current steer angle before sliding. Blended into route speed so an outside car (less steering) may carry more speed.
            float steerRad = Math.Abs(Control.SteerDegrees) * (float)Math.PI / 180f;
            if (steerRad > 0.001f)
            {
                float turnRadius = VehicleData.WheelBase / (float)Math.Tan(steerRad);
                Brain.CurrentIntention.SteerLimitedSpeed = (float)Math.Sqrt(9.8f * VehicleData.CurrentMechanicalGrip * Math.Max(turnRadius, 1f));
                followTrackSpd = 0.7f * Brain.CurrentIntention.SteerLimitedSpeed + 0.3f * followTrackSpd;
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

            Brain.CurrentIntention.Speed = Math.Min(cornerSpd + ARS.MphToMps(ARS.CornerOffsetMph), followTrackSpd + ARS.MphToMps(ARS.RouteOffsetMph));
            // Physics-limited cornering speed for the current high-speed curve radius.
            Brain.CurrentIntention.CorneringSpeedLimit = (float)Math.Sqrt(9.8f * VehicleData.CurrentMechanicalGrip * Brain.CurrentPerception.HighSpeedCurveRadius) + ARS.MphToMps(ARS.CornerOffsetMph);

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
            if (rbFactor < 1f && !rbFreeZone && !(ARS.CurrentRubberbandMode == RubberbandMode.Natural && rbNearbyRival))
                Brain.CurrentIntention.Speed *= rbFactor;
        }

        // Vertical-curvature grip factor over the three-node window centred on a node, as a speed multiplier.
        // 1 means the window is degenerate or a straight; both the aggression and the floor scale with the local
        // radius, so a tight corner is cautious and a straight is aggressive. A corner reads its own entrance and exit
        // as the outer samples, the route a short window around its lookahead point.
        float CrestGripSpeedFactor(int startNode, int centreNode, int endNode, float radius, float entrySpeed, out float rawDeltaGs)
        {
            rawDeltaGs = 0f;
            if (startNode < 0 || centreNode < 0 || endNode < 0) return 1f;
            if (startNode == endNode || startNode == centreNode || endNode == centreNode) return 1f;

            float deltaGs = ARS.HillGripDeltaGs(ARS.TrackPoints[startNode].Position, ARS.TrackPoints[centreNode].Position, ARS.TrackPoints[endNode].Position, entrySpeed);
            rawDeltaGs = deltaGs;
            if (deltaGs > 0f) deltaGs = 0f;
            float aggression = ARS.MapGamma(radius, 100f, 300f, 0f, 1f, 0.5f, true);
            float crestFloor = ARS.MapGamma(radius, 100f, 500f, 0.4f, 0.8f, 0.5f, true);
            if (deltaGs < 0f) deltaGs *= (1f - aggression);
            float verticalGripFactor = Math.Max(1f + deltaGs, crestFloor);
            if (verticalGripFactor < 1f) verticalGripFactor = 1f - Math.Min((1f - verticalGripFactor) * ARS.CrestEffect, 0.9f);
            return (float)Math.Sqrt(verticalGripFactor);
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

        float GetFollowPointSlopeAngle()
        {
            int followNode = (int)ARS.Clamp(CurrentTrackPoint.Node + (int)(Car.Velocity.Length() * RouteLookAheadSeconds), 0, ARS.TrackPoints.Count - 1);
            int aheadNode = (int)ARS.Clamp(followNode + 5, 0, ARS.TrackPoints.Count - 1);
            if (aheadNode == followNode) return 0f;

            Vector3 from = ARS.TrackPoints[followNode].Position;
            Vector3 to = ARS.TrackPoints[aheadNode].Position;
            float horizDist = Vector2.Distance(new Vector2(from.X, from.Y), new Vector2(to.X, to.Y));
            if (horizDist < 0.01f) return 0f;
            return Math.Abs((float)Math.Atan2(to.Z - from.Z, horizDist));
        }

        // TCS slip-ratio targets, on the game's per-wheel rotation-slip ratio at wheel+0x174: 0 free rolling, the
        // curve peaks at 1.0, degrades linearly to CurveMin at 2.5 (sfTractionPeakAngle / sfTractionMinAngle, leaked
        // wheel.cpp:94-97) and is flat past it. More negative = more spin. Scale confirmed by the driver against
        // live values: the lock ratio runs to ~15 at a stopped wheel, mingrip onset at 2.5.
        const float IdealWheelspinLaunchRatio = 1.75f;   // standstill: mid-traction, the CurveMax-CurveMin midpoint
        const float IdealWheelspinPeakRatio = 1.0f;      // normal driving: the traction peak
        const float IdealWheelspinOffTrackRatio = 0.5f;  // off-track: half the peak coefficient
        const float IdealWheelspinLaunchTaperEndMph = 30f;
        const float SlipTargetGripFloor = 0.3f;

        // One home for the throttle reason caps: each reason states the level it wants, its field glides toward
        // that level at the shared rate both ways, and ConvertSpeedToPedals takes the minimum against the plan's
        // own throttle and tags the winner. The level is the logic half, the shared rate the clock half.
        const float ReasonCapSlewRate = 3.5f;
        const float TcsCapFloor = 0.25f;
        // The curve is flat from here (CurveMin at 2.5): both cap levels bottom exactly at the knee — past it no
        // deeper slip earns a deeper cut.
        const float SlipCurveKnee = 2.5f;
        const float YieldThrottleLevel = 0.5f;

        // Instability, the replacement's first cut: off-road the chassis is thrown around and the two consequences
        // are losing grip and losing control. The ride height is captured at Launch, where the car is known to be at
        // rest, so a rise above it is the chassis unloading off its suspension; the yaw side is the friction circle,
        // since at this speed the tyres allow at most grip*g of lateral acceleration.
        const float InstabilityRideHeightMargin = 0.1f;
        const float InstabilityMinSpeedMps = 8f;
        const float InstabilityYawTolerance = 1.2f;
        const float InstabilityThrottleLevel = 0f;

        // The overspeed correction only arms while the car is going straight and up to speed: steering or slide of
        // its own puts lateral work and bump transients into the longitudinal read, and below the speed gate the
        // launch assist is what it would otherwise be reading.
        const float OverspeedArmMaxSteerDegrees = 2f;
        const float OverspeedArmMaxSlideDegrees = 2f;
        const float OverspeedArmMinSpeedMph = 30f;

        // Cutting is fast, recovering is deliberate: the shared rate going down, half of it coming back.
        const float ReasonCapRecoveryScale = 0.5f;

        float GlideCap(float current, float level)
        {
            float rate = level < current ? ReasonCapSlewRate : ReasonCapSlewRate * ReasonCapRecoveryScale;
            float step = rate * TickScale;
            return ARS.Clamp(current + ARS.Clamp(level - current, -step, step), 0f, 1f);
        }

        void UpdateThrottleReasonCaps()
        {
            Control.MaxThrottleFromTCS = GlideCap(Control.MaxThrottleFromTCS, TcsCapLevel());
            Control.MaxThrottleFromInstability = GlideCap(Control.MaxThrottleFromInstability, IsUnstable() ? InstabilityThrottleLevel : 1f);

            bool overspeedArmed = Math.Abs(Control.SteerDegrees) < OverspeedArmMaxSteerDegrees && Math.Abs(VehicleData.SlideAngle) < OverspeedArmMaxSlideDegrees && Car.Velocity.Length() > ARS.MphToMps(OverspeedArmMinSpeedMph);
            float overspeedLevel = 1f;
            if (ARS.OverspeedEnabled && overspeedArmed && VehicleData.OverspeedExcessGs > 0f)
                overspeedLevel = ARS.Clamp(1f - (float)Math.Floor(VehicleData.OverspeedExcessGs / 0.1f) * 0.5f, 0f, 1f);
            Control.MaxThrottleFromOverspeed = GlideCap(Control.MaxThrottleFromOverspeed, overspeedLevel);

            float rivalLevel = 1f;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || r.RelativePosition != RelativePos.Ahead) continue;
                if (ARS.IsBetween(r.SecondsToHit, 0f, 3f)) rivalLevel = Math.Min(rivalLevel, ARS.Remap(r.SecondsToHit, 0f, 3f, 0f, 1f, true));
                if (ARS.IsBetween(r.FrontGap, 0f, 1f)) rivalLevel = Math.Min(rivalLevel, ARS.Remap(r.FrontGap, 0f, 1f, 0f, 1f, true));
            }
            Control.MaxThrottleFromRival = GlideCap(Control.MaxThrottleFromRival, rivalLevel);

            Control.MaxThrottleFromChillOut = GlideCap(Control.MaxThrottleFromChillOut, ActiveManeuver.Type == ManeuverType.ChillOut ? ChillThrottleCap : 1f);
            Control.MaxThrottleFromYield = GlideCap(Control.MaxThrottleFromYield, ActiveManeuver.Type == ManeuverType.Yield && ActiveManeuver.Target != null ? YieldThrottleLevel : 1f);
        }

        // Both signals are read every tick: the old system latched them at 3 Hz and missed crests shorter than the
        // sample period. Riding above the at-rest height means the chassis is unloaded, bouncing or airborne;
        // demanding more lateral acceleration than the grip allows means the car is rotating, not cornering.
        bool IsUnstable()
        {
            if (_restHeightAboveGround > 0f && Car.HeightAboveGround - _restHeightAboveGround > InstabilityRideHeightMargin) return true;

            float speed = Car.Velocity.Length();
            if (speed < InstabilityMinSpeedMps) return false;
            float lateralDemand = ARS.DegToRad(Math.Abs(VehicleData.YawRotationPerSecondDegrees)) * speed;
            return lateralDemand > VehicleData.CurrentMechanicalGrip * Handling.Gravity * InstabilityYawTolerance;
        }

        float TcsCapLevel()
        {
            if (!ARS.TcsEnabled) return 1f;
            float wheelspin = ARS.MaxWheelSlip(Car);

            float IdealWheelspin;
            if (OutOfTrackDistance() > 0f)
            {
                // The half-peak point is a deliberate policy, not a grip-scaled setpoint: off-track halves the target itself.
                IdealWheelspin = -IdealWheelspinOffTrackRatio;
            }
            else
            {
                float gripScale = ARS.Clamp(GroundGripMultiplier, SlipTargetGripFloor, 1f);
                IdealWheelspin = -ARS.Remap(ARS.MpsToMph(Car.Velocity.Length()),
                    IdealWheelspinLaunchTaperEndMph, 0f, IdealWheelspinPeakRatio * gripScale, IdealWheelspinLaunchRatio * gripScale, true);
            }

            float absTarget = -IdealWheelspin;
            float spinDepth = Math.Max(0f, IdealWheelspin - wheelspin);
            float depthShare = ARS.Clamp(spinDepth / (SlipCurveKnee - absTarget), 0f, 1f);
            return 1f - depthShare * (1f - TcsCapFloor);
        }

        // ABS: the brake-side twin, containment where TCS regulates: TCS holds a curve point, ABS guards the flat
        // region past mingrip where full lock lives — a stopped wheel reads ~15 (driver-observed), so the cut starts
        // at the knee (2.5) and floors by 5. Lock reads positive on the same +0x174 ratio (leaked wheel.cpp:5233
        // spin negative, :5268 lock positive); the engine's own free ABS clamps coarsely; FLAG_WD_ABS is left alone;
        // the driver sign check is the first test.
        const float AbsFloorSlip = 5f;
        const float AbsBrakeFloor = 0.25f;

        void UpdateBrakeReasonCaps()
        {
            bool countersteering = IsFullCountersteer();
            Control.MaxBrakeFromABS = GlideCap(Control.MaxBrakeFromABS, AbsCapLevel());
            Control.MaxBrakeFromCountersteer = GlideCap(Control.MaxBrakeFromCountersteer, countersteering ? 0f : 1f);
        }

        float AbsCapLevel()
        {
            if (!ARS.AbsEnabled) return 1f;
            float lockSlip = ARS.MaxWheelLockSlip(Car);

            float lockDepth = Math.Max(0f, lockSlip - SlipCurveKnee);
            float depthShare = ARS.Clamp(lockDepth / (AbsFloorSlip - SlipCurveKnee), 0f, 1f);
            return 1f - depthShare * (1f - AbsBrakeFloor);
        }

        // The one throttle composition: the plan's throttle against every reason field, the argmin named into the
        // tag (first reason wins ties). Offtrack's cut and the countersteer replacement pre-empt the ordinary
        // reasons; grid wait owns the pedal outright and stuck recovery re-tags after.
        float ComposeThrottleCap(float baseThrottle, bool offtrackLimited, bool countersteering)
        {
            float ceiling = Math.Min(Control.MaxThrottleFromTCS, Control.MaxThrottleFromInstability);
            ceiling = Math.Min(ceiling, Control.MaxThrottleFromOverspeed);
            ceiling = Math.Min(ceiling, Control.MaxThrottleFromRival);
            ceiling = Math.Min(ceiling, Control.MaxThrottleFromChillOut);
            ceiling = Math.Min(ceiling, Control.MaxThrottleFromYield);
            Control.MaxThrottle = ceiling;

            float binding = baseThrottle;
            ThrottleReason reason = ThrottleReason.Plan;
            if (Control.MaxThrottleFromTCS < binding) { binding = Control.MaxThrottleFromTCS; reason = ThrottleReason.Tcs; }
            if (Control.MaxThrottleFromInstability < binding) { binding = Control.MaxThrottleFromInstability; reason = ThrottleReason.Instability; }
            if (Control.MaxThrottleFromOverspeed < binding) { binding = Control.MaxThrottleFromOverspeed; reason = ThrottleReason.Overspeed; }
            if (Control.MaxThrottleFromRival < binding) { binding = Control.MaxThrottleFromRival; reason = ThrottleReason.Rival; }
            if (Control.MaxThrottleFromChillOut < binding) { binding = Control.MaxThrottleFromChillOut; reason = ThrottleReason.ChillOut; }
            if (Control.MaxThrottleFromYield < binding) { binding = Control.MaxThrottleFromYield; reason = ThrottleReason.Yield; }
            if (offtrackLimited) reason = ThrottleReason.Offtrack;
            if (countersteering) reason = ThrottleReason.Countersteer;
            if (BaseBehavior == RacerBaseBehavior.GridWait) reason = ThrottleReason.GridWait;

            Control.ThrottleReason = reason;
            Control.ThrottleReasonLevel = binding;
            return binding;
        }

        // The brake-side composition: the plan's braking against the ABS and countersteer caps, the argmin named
        // into the tag; offtrack's brake command pre-empts and grid wait owns the tag while waiting.
        float ComposeBrakeCap(float baseBrake, bool offtrackBrakeCommand, bool countersteering)
        {
            Control.MaxBrake = Math.Min(Control.MaxBrakeFromABS, Control.MaxBrakeFromCountersteer);

            float binding = baseBrake;
            BrakeReason reason = BrakeReason.Plan;
            if (Control.MaxBrakeFromABS < binding) { binding = Control.MaxBrakeFromABS; reason = BrakeReason.Abs; }
            if (Control.MaxBrakeFromCountersteer < binding) { binding = Control.MaxBrakeFromCountersteer; reason = BrakeReason.Countersteer; }
            if (offtrackBrakeCommand) reason = BrakeReason.Offtrack;
            if (countersteering) reason = BrakeReason.Countersteer;
            if (BaseBehavior == RacerBaseBehavior.GridWait) reason = BrakeReason.GridWait;

            Control.BrakeReason = reason;
            Control.BrakeReasonLevel = binding;
            return binding;
        }
        void ConsiderManeuvers()
        {
            if (ControlledByPlayer) return;

            // Force-disable maneuvers armed for more than 8s without firing.
            if (ActiveManeuver.Type != ManeuverType.None && Game.GameTime - ActiveManeuver.LastEnabled > 8000)
            {
                ActiveManeuver.Type = ManeuverType.None;
                ActiveManeuver.Target = null;
            }

            // Divebomb cleanup: off once we pass the armed apex.
            if (ActiveManeuver.Type == ManeuverType.DiveBomb && _divebombApexNode >= 0)
            {
                int passed = CurrentTrackPoint.Node - _divebombApexNode;
                if (!ARS.IsPointToPoint && passed < 0) passed += ARS.TrackPoints.Count;
                if (passed >= 0)
                {
                    ActiveManeuver.Type = ManeuverType.None;
                    ActiveManeuver.Target = null;
                    _divebombApexNode = -1;
                }
            }

            // DefendLane fold: off once we pass the defended apex or the target gets past us.
            if (ActiveManeuver.Type == ManeuverType.DefendLane)
            {
                bool lostTarget = ActiveManeuver.Target == null
                    || !ActiveManeuver.Target.Car.Exists()
                    || !Brain.Rivals.Any(r => r.RivalRacer == ActiveManeuver.Target && r.RelativePosition != RelativePos.Ahead);

                int passed = CurrentTrackPoint.Node - _defendApexNode;
                if (!ARS.IsPointToPoint && passed < 0) passed += ARS.TrackPoints.Count;

                if (lostTarget || (_defendApexNode >= 0 && passed >= 0))
                {
                    ActiveManeuver.Type = ManeuverType.None;
                    ActiveManeuver.Target = null;
                    _defendApexNode = -1;
                }
            }

            // ChillOut cleanup: off once the pack around us thins out.
            if (ActiveManeuver.Type == ManeuverType.ChillOut && RivalsWithinDistance(ChillRivalCrowdDistance) < ChillCrowdCount)
            {
                ActiveManeuver.Type = ManeuverType.None;
                ActiveManeuver.Target = null;
            }

            // ChillOut: only when fast enough for bunching to matter, in a dense pack of better-placed cars.
            if (ActiveManeuver.Type == ManeuverType.None && ARS.MpsToMph(Car.Velocity.Length()) >= ChillMinSpeedMph && RivalsWithinDistance(ChillRivalCrowdDistance) >= ChillCrowdCount)
            {
                Rival closestRival = Brain.Rivals.Where(r => r.RivalRacer != null).OrderBy(r => r.Distance).FirstOrDefault();
                if (closestRival != null)
                {
                    ActiveManeuver.Type = ManeuverType.ChillOut;
                    ActiveManeuver.Target = closestRival.RivalRacer;
                    ActiveManeuver.LastEnabled = Game.GameTime;
                }
            }

            // Card model: while no card is in play, the hand is evaluated in priority order.
            // Nitro resolves instantly (burn lives in _nitrousActiveUntil), so it never occupies the slot.
            if (ActiveManeuver.Type == ManeuverType.None) TryPlayNitrousCard();

            if (ActiveManeuver.Type == ManeuverType.None) TryPlayDefendLaneCard();

            if (ActiveManeuver.Type == ManeuverType.None) TryPlayDivebombCard();

            if (ActiveManeuver.Type == ManeuverType.None) TryPlayYieldCard();
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
            if (!ARS.NitrousEnabled) return;

            if (Game.GameTime < _nitrousActiveUntil) return;   // the burn's power rides ApplyPowerMultiplier
            if (_nitrousActiveUntil > 0) StopNitrous();
        }

        // Valid when a shot is available and the straight is long enough; appropriate when contested, defended,
        // lonely, or spent near the finish with a rival close.
        bool TryPlayNitrousCard()
        {
            if (!ARS.NitrousEnabled || Lap <= _nitrousLapUsed) return false;
            if (Control.Brake > 0f) return false;
            if (OutOfTrackDistance() > 0f) return false;
            if (Math.Abs(Control.SteerDegrees) >= NitrousMaxSteerDegrees) return false;
            if (Control.Throttle < NitrousMinThrottle) return false;
            if (!IsAwd() && Function.Call<int>((Hash)MaxDriveGearHash, Car) > 3 && Function.Call<int>((Hash)CurrentDriveGearHash, Car) <= 2) return false;
            if (NextApexNode < 0) return false;

            float speed = Car.Velocity.Length();
            Rival closestRival = Brain.Rivals
                .Where(r => r.RivalRacer != null)
                .OrderBy(r => r.Distance)
                .FirstOrDefault();

            // Finish spender: with a rival nearby the burn near the line is always worth it,
            // so the 8s corner gate no longer applies.
            bool finishSpender = closestRival != null && closestRival.Distance <= NitrousNearbyRivalDistance
                && RemainingRaceDistanceMeters() <= speed * (NitrousDurationMs / 1000f) + NitrousFinishExtraDistance;
            if (!finishSpender)
            {
                CornerPoint corner = ARS.Corners.FirstOrDefault(c => c.Node == NextApexNode);
                // Circuit wrap makes a just-behind entrance read a lap away; veto in-corner shots.
                if (corner != null && IsWithinCorner(corner)) return false;
                int entranceNode = corner == null
                    ? NextApexNode
                    : (corner.StartNode >= 0 ? corner.StartNode : OffsetCornerNode(NextApexNode, -corner.LengthStart));
                if (ForwardNodeDistance(entranceNode) / Math.Max(speed, 1f) < NitrousCornerLookaheadSeconds) return false;
            }

            bool rivalNearbyFaster = closestRival != null && closestRival.RivalRacer.Car.Velocity.Length() > speed;
            bool rivalBehindIncoming = Brain.Rivals.Any(r => r.RivalRacer != null && r.RelativePosition == RelativePos.Behind
                && r.RivalRacer.Car.Velocity.Length() > speed
                && r.Distance / (r.RivalRacer.Car.Velocity.Length() - speed) < NitrousDefenseReachSeconds);
            bool lonelyClear = closestRival == null
                && speed < Handling.EstimatedTopSpeed * NitrousLonelySpeedFraction
                && ForwardNodeDistance(NextApexNode) > NitrousLonelyMinApexDistance;
            if (!rivalNearbyFaster && !rivalBehindIncoming && !lonelyClear && !finishSpender) return false;

            StartNitrous();
            return true;
        }

        const float DivebombEntranceSeconds = 5f;
        const float DivebombFullThrottle = 0.99f;
        const float DivebombOverlapReachSeconds = 2f;

        // Divebomb card: at full throttle into a corner under 5s away, with an open side to commit
        // into (at most one rival wall active), dive the closest rival we overlap or will reach within 2s.
        bool TryPlayDivebombCard()
        {
            if (Brain.Corner == null) return false;
            if (Control.Throttle < DivebombFullThrottle) return false;
            if (_activeRivalWallCount > 1) return false;

            int apexNode = Brain.Corner.Point.Node;
            int entranceNode = CornerEntranceNode(Brain.Corner.Point, apexNode);
            if (entranceNode < 0) return false;
            float timeToEntrance = ForwardNodeDistance(entranceNode) / Math.Max(Car.Velocity.Length(), 1f);
            if (timeToEntrance > DivebombEntranceSeconds) return false;

            Rival diveTarget = Brain.Rivals
                .Where(r => r.RivalRacer != null
                    && r.RivalRacer.Car.Exists()
                    && (r.RelativePosition == RelativePos.Left || r.RelativePosition == RelativePos.Right
                        || r.TimeToContact <= DivebombOverlapReachSeconds))
                .OrderBy(r => r.Distance)
                .FirstOrDefault();
            if (diveTarget == null) return false;

            ActiveManeuver.Type = ManeuverType.DiveBomb;
            ActiveManeuver.Target = diveTarget.RivalRacer;
            ActiveManeuver.LastEnabled = Game.GameTime;
            _divebombApexNode = apexNode;
            _divebombBrakeBonus = ARS.GetRandomInt(2, 9) / 100f;
            return true;
        }

        // DefendLane card: cover the inside so a faster chaser that reaches the entrance no later
        // than we do can't dive underneath.
        bool TryPlayDefendLaneCard()
        {
            if (Brain.Corner == null) return false;

            int apexNode = Brain.Corner.Point.Node;
            int entranceNode = CornerEntranceNode(Brain.Corner.Point, apexNode);
            if (entranceNode < 0) return false;
            float myTimeToEntrance = ForwardNodeDistance(entranceNode) / Math.Max(Car.Velocity.Length(), 1f);
            if (!ARS.IsBetween(myTimeToEntrance, 1f, 3f)) return false;

            Rival defenderTarget = Brain.Rivals
                .Where(r => r.RivalRacer != null
                    && r.RivalRacer.Car.Exists()
                    && r.RelativePosition == RelativePos.Behind
                    && r.RivalRacer.ActiveManeuver.Type != ManeuverType.DiveBomb
                    && r.RivalRacer.ActiveManeuver.Type != ManeuverType.DefendLane
                    && r.Distance <= 30f
                    && r.ForwardSpeedGap < 0f
                    && r.RivalRacer.ForwardNodeDistance(entranceNode) / Math.Max(r.RivalRacer.Car.Velocity.Length(), 1f) <= myTimeToEntrance)
                .OrderBy(r => r.Distance)
                .FirstOrDefault();
            if (defenderTarget == null) return false;

            ActiveManeuver.Type = ManeuverType.DefendLane;
            ActiveManeuver.Target = defenderTarget.RivalRacer;
            ActiveManeuver.LastEnabled = Game.GameTime;
            _defendApexNode = apexNode;
            return true;
        }

        // Yield card: let a faster overlapping rival by when they carry far more pressure into the entrance.
        bool TryPlayYieldCard()
        {
            if (Brain.Corner == null) return false;

            Rival closestRival = Brain.Rivals
                .Where(r => r.RivalRacer != null && r.RivalRacer.Car.Exists())
                .OrderBy(r => r.Distance)
                .FirstOrDefault();
            if (closestRival == null) return false;

            float pressureDiff = closestRival.RivalRacer.Pressure - Pressure;
            bool inOverlap = closestRival.RelativePosition == RelativePos.Left || closestRival.RelativePosition == RelativePos.Right;
            int entranceNode = CornerEntranceNode(Brain.Corner.Point, Brain.Corner.Point.Node);
            float timeToEntrance = ForwardNodeDistance(entranceNode) / Math.Max(Car.Velocity.Length(), 1f);
            if (pressureDiff <= 30f || !inOverlap || !ARS.IsBetween(timeToEntrance, 0.5f, 2f)
                || closestRival.Distance > 20f
                || closestRival.RivalRacer.Car.Velocity.Length() <= Car.Velocity.Length()) return false;

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
        }

        public bool TryFireNitrous()
        {
            if (Game.GameTime < _nitrousActiveUntil || NitroChargedLap < Lap || Lap <= _nitrousLapUsed) return false;
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
                if (dist > 10f)
                {
                    ActiveManeuver.Type = ManeuverType.None;
                    ActiveManeuver.Target = null;
                }
            }
            else
            {
                ActiveManeuver.Type = ManeuverType.None;
                ActiveManeuver.Target = null;
            }
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



        public void ProcessTick()
        {
            UpdateTickData();

            if (ARS.DebugToggles[Options.ShowInputs] && !ControlledByPlayer) SampleInputTrail();

            // Corner rings + the steer-angle fan (track analysis).
            if (ARS.DebugToggles[Options.ShowTrackAnalysis] && !ControlledByPlayer && ARS.DebugFocusRacer == this)
            {
                DrawCornerTable();
                DrawCornerCircle();

                // Steer angle visualization: three lines from the top of the car.
                float halfZ = Car.Model.GetDimensions().Z * 0.5f;
                Vector3 origin = Car.Position + new Vector3(0, 0, halfZ);
                Vector3 fwd = Car.ForwardVector;
                float lineLen = 5f;

                // Yellow: pursuit angle (what the lane tracking wants)
                ARS.DrawLine(origin + new Vector3(0, 0, 0.05f), origin + new Vector3(0, 0, 0.05f) + RotateZ(fwd, _steerPursuitDeg) * lineLen, Color.Yellow);

                // Red: pursuit angle + damper
                ARS.DrawLine(origin + new Vector3(0, 0, 0.10f), origin + new Vector3(0, 0, 0.10f) + RotateZ(fwd, _steerPursuitDeg + _debugDamperTermDeg) * lineLen, Color.Red);

                // White: final slewed steer (what the wheels actually request)
                ARS.DrawLine(origin + new Vector3(0, 0, 0.15f), origin + new Vector3(0, 0, 0.15f) + RotateZ(fwd, Control.SteerDegrees) * lineLen, Color.White);
            }

            // Input trail + pedal bar (inputs).
            if (ARS.DebugToggles[Options.ShowInputs] && !ControlledByPlayer && ARS.DebugFocusRacer == this)
            {
                DrawInputTrail();

                DrawPedalBar();

                DrawYawDamperHud();
            }

            _appliedThrottleLastFrame = VehicleMemory.GetThrottle(Car);

            // Judge the player's launch input once per race: sample while the window is open (a keyboard pedal is a
            // hard 0/1, a gamepad is not), then freeze the verdict so the AI's shaping cannot change mid-race.
            if (ControlledByPlayer && ARS.PlayerLaunchTestActive)
            {
                float playerThrottle = Math.Abs(_appliedThrottleLastFrame);
                if (playerThrottle > ModulatedThrottleLow && playerThrottle < ModulatedThrottleHigh) ARS.PlayerModulatesThrottle = true;

                if (ARS.GetForwardSpeed(Car) > LaunchShapeMaxSpeed)
                {
                    ARS.PlayerLaunchTestActive = false;
                    ARS.Log(ARS.LogImportance.Info, ARS.PlayerModulatesThrottle
                        ? "Player launch input: modulatable — AI launch throttle capped at " + ModulatedThrottleCap
                        : "Player launch input: binary 0/1 — AI launch throttle quantised");
                }
            }

            if (!ControlledByPlayer)
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
            if (_inputTrail.Count > 0 && position.DistanceTo2D(_inputTrail[_inputTrail.Count - 1].Position) < 0.5f) return;
            _inputTrail.Add(new InputTrailSample { Position = position, Input = Control.Throttle - Control.Brake });
            if (_inputTrail.Count > InputTrailMaxSamples) _inputTrail.RemoveAt(0);
        }

        void DrawInputTrail()
        {
            if (_inputTrail.Count == 0) return;
            foreach (InputTrailSample sample in _inputTrail)
                World.DrawMarker(MarkerType.DebugSphere, sample.Position, Vector3.Zero, Vector3.Zero, new Vector3(0.105f, 0.105f, 0.105f), InputColour(sample.Input));
            ARS.DrawLine(Car.Position, _inputTrail[_inputTrail.Count - 1].Position, Color.White);
        }

        // Every noted corner, dimmed, so a close pair can be read off the road; eight segments places a ring well
        // enough. The held corner gets the full twenty plus its label.
        void DrawCornerTable()
        {
            foreach (CornerPoint corner in ARS.Corners)
            {
                if (Brain.Corner != null && corner.Node == Brain.Corner.Point.Node) continue;
                if (!TryCornerCircle(corner, out Vector3 centre, out float radius, out float z)) continue;
                DrawCornerRing(centre, radius, z, 8, Color.FromArgb(150, 150, 150));
            }
        }

        // The active corner's own circle, so its fitted radius can be measured against the road: 20 plan-view
        // segments plus the diameter through the apex. DRAW_LINE, not markers, so it spends no marker budget.
        void DrawCornerCircle()
        {
            if (Brain.Corner == null) return;
            CornerPoint corner = Brain.Corner.Point;
            if (!TryCornerCircle(corner, out Vector3 centre, out float radius, out float z)) return;

            DrawCornerRing(centre, radius, z, 20, Color.Magenta);

            string carContext = TryGetCornerContext(corner.Node, out CornerContext context)
                ? (context.RequiresBraking ? " B+" : " B-") + (context.RequiresPositioning ? " P+" : " P-") + "  BF " + context.BrakeFactor.ToString("0.00")
                : "";
            int crestStart = CornerEntranceNode(corner, corner.Node);
            int crestEnd = CornerExitNode(corner);
            float crestFactor = CrestGripSpeedFactor(crestStart, corner.Node, crestEnd, NextApexRadius, _cornerSpd, out float crestDeltaGs);
            int crestSpan = crestEnd - crestStart;
            if (!ARS.IsPointToPoint && crestSpan < 0) crestSpan += ARS.TrackPoints.Count;
            string crestContext = "  crest " + crestDeltaGs.ToString("0.00") + "G x" + crestFactor.ToString("0.00") + " / " + crestSpan + "m";
            ARS.DrawText(new Vector3(centre.X, centre.Y, z + 1f), "R " + radius.ToString("0.0") + "/" + corner.DetectedRadius.ToString("0.0") + " m" + carContext + crestContext, Color.Magenta, 0.45f);

            Vector3 apexPosition = ARS.TrackPoints[corner.Node].Position;
            Vector3 toApex = new Vector3(apexPosition.X - centre.X, apexPosition.Y - centre.Y, 0f);
            if (toApex.LengthSquared() < 0.0001f) return;
            toApex.Normalize();
            Vector3 from = centre - toApex * radius;
            Vector3 to = centre + toApex * radius;
            ARS.DrawLine(new Vector3(from.X, from.Y, z), new Vector3(to.X, to.Y, z), Color.Magenta);
        }

        // The centre sits a radius to the inside: a positive angle is a left-hand corner, whose inside is -right.
        bool TryCornerCircle(CornerPoint corner, out Vector3 centre, out float radius, out float z)
        {
            radius = corner.SupposedRadius;
            z = 0f;
            centre = Vector3.Zero;
            if (!(radius > 0.1f) || radius > 500f) return false;

            Vector3 apexPosition = ARS.TrackPoints[corner.Node].Position;
            Vector3 heading = ARS.TrackPoints[corner.Node].Direction;
            Vector3 right = Vector3.Cross(heading, Vector3.WorldUp).Normalized;
            centre = apexPosition - right * (radius * Math.Sign(corner.Angle));
            z = apexPosition.Z + 0.5f;
            return true;
        }

        void DrawCornerRing(Vector3 centre, float radius, float z, int segments, Color colour)
        {
            Vector3 previous = Vector3.Zero;
            for (int i = 0; i <= segments; i++)
            {
                float step = i * 2f * (float)Math.PI / segments;
                Vector3 point = new Vector3(centre.X + (float)Math.Cos(step) * radius, centre.Y + (float)Math.Sin(step) * radius, z);
                if (i > 0) ARS.DrawLine(previous, point, colour);
                previous = point;
            }
        }

        // Pedal bar over the car along its forward axis: centre neutral, front end full throttle, back end full
        // brake, reverse throttle placed ahead by magnitude. The applied pedal is green (throttle) or red (brake);
        // a white sphere is an override command; a coloured sphere is a per-reason limit, drawn only where it bites.
        // The inputs go down first and then every cap from highest to lowest, so the binding cap paints last.
        const float PedalBarHalfLength = 1.25f;
        // The applied pedal sits a hair under both cap classes, so a cap that coincides with it still rings it.
        const float PedalBarReasonSize = 0.1f;
        const float PedalBarCapSize = 0.09f;
        const float PedalBarInputSize = 0.08f;
        // Etiquette limits (rival, chill-out, yield) are harmless, so they read cool; grip limits yellow; the
        // overspeed cut black; countersteer orange; instability violet.
        static readonly Color NonDangerousReasonColor = Color.FromArgb(255, 120, 200, 255);
        static readonly Color GripReasonColor = Color.Yellow;
        static readonly Color OverspeedReasonColor = Color.Black;
        static readonly Color CountersteerReasonColor = Color.Orange;
        static readonly Color InstabilityReasonColor = Color.FromArgb(255, 190, 80, 255);

        struct PedalCapSphere
        {
            public Vector3 Axis;
            public float Level;
            public Color Color;
            public float Size;
        }

        readonly PedalCapSphere[] _pedalCaps = new PedalCapSphere[12];

        void DrawPedalBar()
        {
            Vector3 center = Car.Position + new Vector3(0f, 0f, Car.Model.GetDimensions().Z + 0.375f);
            Vector3 fwd = Car.ForwardVector;
            ARS.DrawLine(center + fwd * PedalBarHalfLength, center - fwd * PedalBarHalfLength, Color.White);

            float throttle = Math.Abs(Control.Throttle);
            if (throttle > 0f) DrawPedalBarSphere(center, fwd, throttle, Color.Green, PedalBarInputSize);
            if (Control.Brake > 0f) DrawPedalBarSphere(center, -fwd, Control.Brake, Color.Red, PedalBarInputSize);

            int count = 0;
            AddPedalCap(fwd, Control.MaxThrottleFromTCS, GripReasonColor, PedalBarReasonSize, ref count);
            AddPedalCap(fwd, Control.MaxThrottleFromInstability, InstabilityReasonColor, PedalBarReasonSize, ref count);
            AddPedalCap(fwd, Control.MaxThrottleFromOverspeed, OverspeedReasonColor, PedalBarReasonSize, ref count);
            AddPedalCap(fwd, Control.MaxThrottleFromRival, NonDangerousReasonColor, PedalBarReasonSize, ref count);
            AddPedalCap(fwd, Control.MaxThrottleFromChillOut, NonDangerousReasonColor, PedalBarReasonSize, ref count);
            AddPedalCap(fwd, Control.MaxThrottleFromYield, NonDangerousReasonColor, PedalBarReasonSize, ref count);
            AddPedalCap(-fwd, Control.MaxBrakeFromABS, GripReasonColor, PedalBarReasonSize, ref count);
            AddPedalCap(-fwd, Control.MaxBrakeFromCountersteer, CountersteerReasonColor, PedalBarReasonSize, ref count);
            if (IsFullCountersteer()) AddPedalCap(fwd, CountersteerRollThrottle, Color.White, PedalBarCapSize, ref count);
            if (_offtrackInputCap < 0f) AddPedalCap(-fwd, -_offtrackInputCap, Color.White, PedalBarCapSize, ref count);
            else if (_offtrackInputCap < 1f) AddPedalCap(fwd, _offtrackInputCap, Color.White, PedalBarCapSize, ref count);

            DrawPedalCaps(center, count);
        }

        // A limit at full authority is not limiting anything, so it is not queued.
        void AddPedalCap(Vector3 axis, float level, Color color, float size, ref int count)
        {
            if (level >= 1f || count >= _pedalCaps.Length) return;
            _pedalCaps[count].Axis = axis;
            _pedalCaps[count].Level = level;
            _pedalCaps[count].Color = color;
            _pedalCaps[count].Size = size;
            count++;
        }

        void DrawPedalCaps(Vector3 center, int count)
        {
            for (int i = 1; i < count; i++)
            {
                PedalCapSphere cap = _pedalCaps[i];
                int j = i - 1;
                while (j >= 0 && _pedalCaps[j].Level < cap.Level)
                {
                    _pedalCaps[j + 1] = _pedalCaps[j];
                    j--;
                }
                _pedalCaps[j + 1] = cap;
            }

            for (int i = 0; i < count; i++)
                DrawPedalBarSphere(center, _pedalCaps[i].Axis, _pedalCaps[i].Level, _pedalCaps[i].Color, _pedalCaps[i].Size);
        }

        void DrawPedalBarSphere(Vector3 center, Vector3 axis, float fraction, Color color, float size)
        {
            World.DrawMarker(MarkerType.DebugSphere, center + axis * ARS.Clamp(fraction, 0f, 1f) * PedalBarHalfLength, Vector3.Zero, Vector3.Zero, new Vector3(size, size, size), color);
        }

        // Full throttle green, neutral yellow, full brake red.
        static Color InputColour(float input)
        {
            float v = ARS.Clamp(input, -1f, 1f);
            return Color.FromArgb((int)(255f * (1f - Math.Max(v, 0f))), (int)(255f * (1f + Math.Min(v, 0f))), 0);
        }

        // Yaw HUD parked: the false gate skips the red text; flip it to re-arm. A guard-return disables nothing.
        void DrawYawDamperHud()
        {
            if (ControlledByPlayer) return;
            if (ARS.DebugFocusRacer != this) return;
            if (1 == 2)
            {
                float yaw = VehicleData.YawRotationPerSecondDegrees;
                float yawError = yaw - _debugYawTargetPerSecond;
                float damper = _debugDamperTermDeg;
                float usagePct = YawUsagePercent();
                Color red = Color.FromArgb(255, 230, 30, 30);
                ARS.DrawText(new Vector2(0.5f, 0.085f), "YAW " + yaw.ToString("0.0") + " / TARGET " + _debugYawTargetPerSecond.ToString("0.0") + " deg/s", red, ARS.DrawTextFont.Standard, ARS.DrawTextAlign.Center, 0.45f);
                ARS.DrawText(new Vector2(0.5f, 0.110f), "ERROR " + yawError.ToString("0.0") + " x GAIN " + _debugDamperGainSeconds.ToString("0.00") + " s = STEER " + damper.ToString("0.0") + " deg", red, ARS.DrawTextFont.Standard, ARS.DrawTextAlign.Center, 0.45f);
                ARS.DrawText(new Vector2(0.5f, 0.135f), "YAW USAGE " + usagePct.ToString("0") + "%", red, ARS.DrawTextFont.Standard, ARS.DrawTextAlign.Center, 0.45f);
            }
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
            VehicleData.SlideAngle = (float)Math.Round(Vector3.SignedAngle(Car.Velocity.Normalized, Car.ForwardVector, Car.UpVector), 3);
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

        public void ApplyInputs()
        {


            if (Driver.IsSittingInVehicle(Car))
            {
                UpdatePassengerSeat();

                Car.HandbrakeOn = Control.HandBrakeTime > Game.GameTime;

                VehicleMemory.SetThrottle(Car, ShapeLaunchThrottle(ARS.Clamp(Control.Throttle, -1, 1)));
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
            float counter = 1f;

            float commanded = Math.Abs(Control.Throttle);
            if (!ControlledByPlayer && commanded > PowerCounterMinCommanded && _appliedThrottleLastFrame < commanded)
                counter *= ARS.Clamp(commanded / Math.Max(_appliedThrottleLastFrame, PowerCounterMinApplied), 1f, PowerCounterMax);

            // Past the car's own top speed the engine fades drive force and adds an over-speed brake. No
            // throttle gate is needed: this scales fDriveForce, which is already zero off-throttle.
            float overTopSpeed = ARS.GetForwardSpeed(Car) - Handling.EstimatedTopSpeed * TopSpeedCounterOnsetScale;
            if (overTopSpeed > 0f) counter *= ARS.Remap(overTopSpeed, 0f, TopSpeedCounterBand, 1f, TopSpeedCounterMax, true);

            _powerCounter = counter;

            if (_powerCounter > 1.001f && Game.GameTime > _powerCounterLoggedAt)
            {
                _powerCounterLoggedAt = Game.GameTime + 2000;
                string throttlePart = ControlledByPlayer ? "" : ", throttle " + _appliedThrottleLastFrame.ToString("0.00") + " of " + commanded.ToString("0.00");
                ARS.Log(ARS.LogImportance.Info, "Power counter " + Car.DisplayName + " x" + _powerCounter.ToString("0.00") + " (over top speed " + overTopSpeed.ToString("0.0") + " m/s" + throttlePart + ")");
            }
        }

        void ApplyPowerMultiplier()
        {
            float multiplier = _powerCounter;
            if (Game.GameTime < _nitrousActiveUntil) multiplier *= NitrousPowerMultiplier;

            if (float.IsNaN(multiplier) || float.IsInfinity(multiplier) || multiplier < 0f || multiplier > 10f) multiplier = 1f;

            Function.Call((Hash)CheatPowerIncreaseHash, Car, multiplier);
        }

        // Match the player's launch capability: a keyboard pedal is a hard 0/1 and 1.0 is exactly what sets the
        // engine's full-throttle grip loss, so a binary player's cars are quantised the same way, while a player who
        // can modulate gets a cap that stays under it. Reverse, braking and the post-fade speed stay unshaped.
        float ShapeLaunchThrottle(float throttle)
        {
            if (throttle <= 0f || ARS.GetForwardSpeed(Car) > LaunchShapeMaxSpeed) return throttle;

            bool modulates = ARS.PlayerModulatesThrottle || !ARS.PlayerParticipating;
            if (modulates) return Math.Min(throttle, ModulatedThrottleCap);
            return throttle > LaunchThrottleBinarySplit ? 1f : 0f;
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
            return Math.Abs(Brain.CurrentPerception.DeviationFromCenter) - CurrentTrackPoint.TrackHalfWidth;
        }

        void UpdateRivalInfo()
        {
            Brain.AvoidanceTarget = null;
            foreach (Rival r in Brain.Rivals)
            {
                r.Update(this);
                bool isAvoidanceCandidate = r.RelativePosition == RelativePos.Ahead && r.RouteGapMeters <= SameSectionMaxGapMeters && (ARS.IsBetween(r.FrontGap, 0f, 3f) || ARS.IsBetween(r.SecondsToHit, 0f, 5f)) && ARS.IsBetween(Math.Abs(r.DirectionDiff), 0f, AvoidAngleGateDegrees);
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
            TrackProgress = Lap * ARS.TrackPoints.Count + CurrentTrackPoint.Node;
            RaceProgress = (int)TrackProgress;
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
            Brain.CurrentPerception.DeviationFromCenter = ARS.SignedLaneOffset(carPosition, CurrentTrackPoint.Position, CurrentTrackPoint.Direction);

            LookAheads.Clear();
            float speed = Car.Velocity.Length();

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
        float RouteRadiusSampled()
        {
            int count = ARS.TrackPoints.Count;
            if (count < 10) return 999f;

            float traction = Math.Max(VehicleData.CurrentMechanicalGrip, 0.1f);
            int o1 = 0;
            int o2 = (int)(Car.Velocity.Length() / traction);
            int o3 = (int)(Car.Velocity.Length() * 2f / traction);

            int n1, n2, n3;
            if (ARS.IsPointToPoint)
            {
                n1 = (int)ARS.Clamp(CurrentTrackPoint.Node + o1, 0, count - 1);
                n2 = (int)ARS.Clamp(CurrentTrackPoint.Node + o2, 0, count - 1);
                n3 = (int)ARS.Clamp(CurrentTrackPoint.Node + o3, 0, count - 1);
            }
            else
            {
                n1 = ((CurrentTrackPoint.Node + o1) % count + count) % count;
                n2 = ((CurrentTrackPoint.Node + o2) % count + count) % count;
                n3 = ((CurrentTrackPoint.Node + o3) % count + count) % count;
            }

            float r = ARS.Circumradius3D(ARS.TrackPoints[n1].Position, ARS.TrackPoints[n3].Position, ARS.TrackPoints[n2].Position);
            if (float.IsNaN(r) || float.IsInfinity(r)) r = 999f;
            return ARS.Clamp(r, 5f, 999f);
        }

        // Braking map close-corner filtering and entrance timing.
        const float ApexBufferSeconds = 5f;
        const float EntranceBrakeBufferSeconds = 0.25f;
        const float EntranceBrakeExtraDistance = 0f;
        const float SecondaryApexSpeedDifference = 5f;
        const float BrakingTargetFactor = 0.5f;

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
                if (distance <= 0) continue;
                if (!TryGetCornerContext(corner.Node, out CornerContext context)) continue;

                bool tight = corner.DetectedRadius < CornerTightRadius;
                // The intended speed must be the one the plan itself will target, or the flag reads a wider corner
                // than the car is actually asked to take and never fires. That is SupposedRadius, not the smoothed one.
                float intended = ApexSpeedWithDownforce(corner.SupposedRadius);
                float timeToEntrance = distance / speed;
                float decel = Math.Max(BrakingDecelBase(corner.Node), 0.1f);
                float brakingDistance = (speed * speed - intended * intended) / (2f * decel);
                bool inBrakingRange = distance <= brakingDistance + BrakeHorizonMargin;
                bool inPositionRange = tight && timeToEntrance <= RequirementLookaheadSeconds;
                if (!inBrakingRange && !inPositionRange) continue;

                float arrival = speed + forwardGs * Handling.Gravity * timeToEntrance * RequirementExtrapolationScale;
                float excess = arrival - intended;

                // A tight corner that wants positioning is braking regardless, because the held queue is the only
                // route to an outside line — so its braking threshold is the lower one.
                if (!context.RequiresBraking && inBrakingRange && excess > ARS.MphToMps(tight ? PositionExcessMph : BrakeExcessMph))
                {
                    context.RequiresBraking = true;
                    flipped = true;
                }
                if (!context.RequiresPositioning && inPositionRange && excess > ARS.MphToMps(PositionExcessMph))
                {
                    context.RequiresPositioning = true;
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

            // Derived fresh every refill and never carried over: slot one must be the nearest corner each time, or
            // the line, the commit lane and the chevron aim past it the moment the other slots are occupied.
            List<int> selectedNodes = new List<int>();
            List<float> selectedRadii = new List<float>();
            if (upcoming.Count > 0)
            {
                selectedNodes.Add(ARS.Corners[upcoming[0]].Node);
                selectedRadii.Add(ARS.Corners[upcoming[0]].SupposedRadius);
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

                    float radius = ARS.Corners[upcoming[i]].SupposedRadius;
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
                selectedRadii.Add(ARS.Corners[upcoming[best]].SupposedRadius);
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
                CornerPoint cp = original ?? new CornerPoint();
                if (original == null)
                {
                    cp.Node = NextApexNode;
                    cp.Angle = ARS.TrackPoints[NextApexNode].Angle;
                    cp.SupposedRadius = NextApexRadius;
                }
                Brain.Corner = new Corner(NextApexSpeed, cp);
            }
            else
            {
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
        int BrakingTargetNode(CornerPoint corner, float apexSpeed, float factor)
        {
            if (corner == null || corner.Node < 0) return -1;

            int targetNode = BrakeTargetBaseNode(corner, -1);
            if (targetNode < 0) return -1;

            factor = ARS.Clamp(factor, 0f, 1f);
            int distance = corner.Node - targetNode;
            if (!ARS.IsPointToPoint && distance < 0) distance += ARS.TrackPoints.Count;
            if (ARS.IsPointToPoint && distance < 0) distance = 0;

            return OffsetCornerNode(targetNode, (int)Math.Round(distance * factor));
        }

        float ApexBrakingSpeed(int apexNode, float apexSpeed)
        {
            if (apexNode < 0) return 999f;
            float velTarget = apexSpeed;

            CornerPoint corner = ARS.Corners.FirstOrDefault(c => c.Node == apexNode);
            int fallbackNode = CornerEntranceNode(corner, apexNode);
            int targetNode = ActiveManeuver.Type == ManeuverType.DiveBomb
                ? BrakingTargetNode(corner, apexSpeed, BrakingTargetFactor)
                : BrakeTargetBaseNode(corner, fallbackNode);
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

        // A crest unloads the tyres before a corner and the span's mean grade cannot see it: an up-then-down bump
        // averages to no grade at all. Geometry is measured once at generation at this probe, so the live speed
        // rescales it, and the unload is floored the way the hill grip model floors its own.
        const float CrestProbeSpeed = ARS.CrestProbeSpeed;
        const float CrestDecelFloor = 0.5f;

        // Braking decel over a span: grip-limited base plus the gravity component of the span's mean grade.
        // The solve is v² = vApex² + 2∫a·ds, so the mean decel over the span is the exact quantity.
        public float BrakingDecel(int apexNode, float spanMeters)
        {
            float decel = BrakingDecelBase(apexNode) * CrestDecelFactor(apexNode, spanMeters) + Handling.Gravity * BrakingGradeSine(spanMeters);
            return Math.Max(decel, 0.1f);
        }

        // Vertical curvature changes normal load, so it scales the grip-limited base only -- the grade term is the
        // along-slope gravity component, which a crest does not change. The crest covers its extent, so the unload is
        // the area of that ramp clipped to the solve span rather than a point test: a binary in/out steps the planned
        // decel mid-braking, and those samples are far too sparse to integrate it.
        float CrestDecelFactor(int apexNode, float spanMeters)
        {
            CornerPoint corner = ARS.Corners.FirstOrDefault(c => c.Node == apexNode);
            if (corner == null || corner.CrestNode < 0 || corner.CrestSpanNodes <= 0 || spanMeters < 1f) return 1f;
            if (corner.CrestGs >= 0f) return 1f;
            int count = ARS.TrackPoints.Count;
            int toCrest = corner.CrestNode - CurrentTrackPoint.Node;
            if (!ARS.IsPointToPoint) toCrest = ((toCrest % count) + count) % count;
            float halfExtent = Math.Max(1f, corner.CrestSpanNodes * 0.5f);
            float start = toCrest - halfExtent;
            float end = toCrest + halfExtent;
            float lo = Math.Max(start, 0f);
            float hi = Math.Min(end, spanMeters);
            if (end <= 0f || start >= spanMeters) return 1f;
            float peak = ARS.Clamp(toCrest, lo, hi);
            float risingUnscaled = RampIntegralUnscaled(start, lo, peak);
            float fallingUnscaled = RampIntegralUnscaled(end, hi, peak);
            float speedRatio = Car.Velocity.Length() / CrestProbeSpeed;
            float unload = 1f + corner.CrestGs * speedRatio * speedRatio * (risingUnscaled + fallingUnscaled) / (2f * halfExtent * spanMeters);
            if (float.IsNaN(unload) || float.IsInfinity(unload)) return 1f;
            unload = Math.Max(unload, CrestDecelFloor);
            return 1f - Math.Min((1f - unload) * ARS.CrestEffect, 0.9f);
        }

        // The raw expression before the 1/(2h) normalisation: 2h times the area under one half of the ramp. Its endpoints
        // go in distance order so the result stays positive; swapping them negates the area and the unload with it.
        static float RampIntegralUnscaled(float zeroAt, float from, float to)
        {
            return (to - zeroAt) * (to - zeroAt) - (from - zeroAt) * (from - zeroAt);
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

            if (_oneSecondTick + _phaseOffsetMs < now)
            {
                _oneSecondTick = now + 1000;

                if (!ControlledByPlayer)
                {
                    if (BaseBehavior == RacerBaseBehavior.Race && ARS.Racers.Count >= 1)
                    {
                        UpdateRivals();
                        UpdateRivalInfo();
                        ConsiderManeuvers();
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


                if (ARS.SettingsMenuStore.GetInt("AIRacerAutofix", 2) == 2 && Function.Call<bool>(Hash._IS_VEHICLE_DAMAGED, Car))
                {
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
                if (_rivalInfoTick + _phaseOffsetMs < now)
                {
                    _rivalInfoTick = now + 500;
                    UpdateRivalInfo();
                }
                ComputeTargetSpeed();
                ComputeSteering();

                UpdateThrottleReasonCaps();
                UpdateBrakeReasonCaps();
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
                IsStuckByThrottle = false;
                UpdateNitrous();
                _lastStuckGameTime = 0;
                _isRecoveringFromStuck = false;
                _stuckRecoveryEndTime = 0;
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

            // Pressure-driven lookahead is intentionally disabled.
            RouteLookAheadSeconds = 0.5f;
        }
 
 
        // A car is out of the woods only when it is on the track, driving forward at a real pace, pointing along the
        // route and not sliding - and has held all of that, because one good sample off a bounce is not recovery.
        bool HasRegainedControl()
        {
            bool tracking = OutOfTrackDistance() <= 0f && ARS.MpsToMph(Vector3.Dot(Car.Velocity, Car.ForwardVector)) >= RecoveredMinSpeedMph && Math.Abs(VehicleData.SlideAngle) < Handling.LateralTractionCurve * RecoveredSlideFraction && Math.Abs(Vector3.SignedAngle(Car.ForwardVector, CurrentTrackPoint.Direction, Vector3.WorldUp)) <= RecoveredHeadingDeg;

            if (!tracking)
            {
                _regainedControlSince = 0;
                return false;
            }

            if (_regainedControlSince == 0) _regainedControlSince = Game.GameTime;
            return Game.GameTime - _regainedControlSince >= RegainedControlHoldMs;
        }

        void UpdateStuckCheck()
        {
            if (HasRegainedControl()) _stuckRecoveryAttempts = 0;

            if (_isRecoveringFromStuck)
            {
                IsStuckByThrottle = false;
                _lastStuckGameTime = 0;
                return;
            }

            int now = Game.GameTime;

            // Cooldown after each recovery ends: the car must get a real chance to drive away.
            if (now < _stuckRecoveryCooldownEndTime)
            {
                IsStuckByThrottle = false;
                _lastStuckGameTime = 0;
                return;
            }

            if (BaseBehavior != RacerBaseBehavior.Race || !Driver.IsSittingInVehicle(Car))
            {
                IsStuckByThrottle = false;
                _lastStuckGameTime = 0;
                return;
            }

            bool lowLongitudinalGs = Math.Abs(VehicleData.GetLongitudinalGs(Car.ForwardVector)) < 0.25f;
            bool stuckCondition = lowLongitudinalGs && ARS.MpsToMph(Car.Velocity.Length()) < 5f;

            if (!stuckCondition)
            {
                IsStuckByThrottle = false;
                _lastStuckGameTime = 0;
                return;
            }

            if (_lastStuckGameTime == 0)
            {
                _lastStuckGameTime = now;
            }

            bool stuckForLongEnough = (now - _lastStuckGameTime) >= StuckCheckTimeMs;
            IsStuckByThrottle = stuckForLongEnough;

            if (stuckForLongEnough && !_isRecoveringFromStuck)
            {
                _isRecoveringFromStuck = true;
                _stuckRecoveryAttempts++;
                _stuckRecoveryEndTime = now + StuckRecoveryTimeMs;
                IsStuckByThrottle = false;
                _lastStuckGameTime = 0;
            }
        }

        void UpdateStuckRecovery()
        {
            if (BaseBehavior != RacerBaseBehavior.Race || !Driver.IsSittingInVehicle(Car))
            {
                _isRecoveringFromStuck = false;
                _stuckRecoveryEndTime = 0;
                return;
            }

            if (!_isRecoveringFromStuck && IsStuckByThrottle)
            {
                _isRecoveringFromStuck = true;
                _stuckRecoveryAttempts++;
                _stuckRecoveryEndTime = Game.GameTime + StuckRecoveryTimeMs;
                IsStuckByThrottle = false;
            }

            if (!_isRecoveringFromStuck) return;

            if (HasRegainedControl())
            {
                FinishStuckRecovery();
                return;
            }

            // The clock is the escalation, not the verdict: a manoeuvre still freeing the car is not cut off, and one
            // that is not counts as another failed attempt so the teleport can take over.
            if (Game.GameTime >= _stuckRecoveryEndTime)
            {
                _stuckRecoveryAttempts++;
                _stuckRecoveryEndTime = Game.GameTime + StuckRecoveryTimeMs;
            }
        }

        void FinishStuckRecovery()
        {
            _isRecoveringFromStuck = false;
            _stuckRecoveryEndTime = 0;
            _stuckReverseUntil = 0;
            _lastStuckGameTime = 0;
            _stuckRecoveryCooldownEndTime = Game.GameTime + StuckRecoveryCooldownMs;
        }

        void ApplyStuckRecoveryOverride()
        {
            if (!_isRecoveringFromStuck) return;

            // Find nearest track point (shared by teleport and steering-align).
            Vector3 carPosition = Car.Position;
            TrackPoint nearest = ARS.TrackPoints[0];
            float best = float.MaxValue;
            foreach (TrackPoint point in ARS.TrackPoints)
            {
                float distance = point.Position.DistanceTo(carPosition);
                if (distance >= best) continue;
                best = distance;
                nearest = point;
            }

            // After 5 failed reverse attempts, teleport to the nearest track edge.
            if (_stuckRecoveryAttempts >= 5)
            {
                Vector3 direction = new Vector3(nearest.Direction.X, nearest.Direction.Y, 0f);
                if (direction == Vector3.Zero) direction = Vector3.WorldNorth;
                direction.Normalize();
                Vector3 right = Vector3.Cross(direction, Vector3.WorldUp);
                float side = ARS.SignedLaneOffset(Car.Position, nearest.Position, nearest.Direction) >= 0f ? 1f : -1f;
                Car.Position = nearest.Position + right * (nearest.TrackHalfWidth * side) + new Vector3(0f, 0f, 0.5f);
                Car.Heading = direction.ToHeading();
                Car.Velocity = direction * ARS.MphToMps(10f);

                _stuckRecoveryAttempts = 0;
                IsStuckByThrottle = false;
                FinishStuckRecovery();
                return;
            }

            if (Game.GameTime >= _stuckRecoveryEndTime)
            {
                FinishStuckRecovery();
                return;
            }

            Control.Brake = 0f;
            Control.BrakeReason = BrakeReason.StuckRecovery;
            Control.BrakeReasonLevel = 0f;
            Control.ThrottleReason = ThrottleReason.StuckRecovery;
            Control.ThrottleReasonLevel = 0f;

            if (_stuckReverseUntil == 0) _stuckReverseUntil = Game.GameTime + StuckReverseMs;

            // Back off straight first: reversing with the wheels turned swings the nose away from where the car is
            // going. Then drive out toward the route AHEAD, where the steering convention is unambiguous.
            if (Game.GameTime < _stuckReverseUntil)
            {
                Control.Throttle = -0.5f;
                Control.SteerDegrees = 0f;
                return;
            }

            Control.Throttle = 0.5f;
            Vector3 toRoute = nearest.Position + nearest.Direction * StuckRouteAimMeters - Car.Position;
            toRoute.Z = 0f;
            Control.SteerDegrees = toRoute.LengthSquared() > 0.01f ? Vector3.SignedAngle(Car.ForwardVector, toRoute.Normalized, Vector3.WorldUp) : 0f;
        }

        void UpdatePerceivedGrip()
        {


            float handlingGrip = Function.Call<float>((Hash)0xA132FB5370554DB0, Car);
            handlingGrip = ARS.Clamp(handlingGrip, 0.1f, 100f);
            handlingGrip /= 1f + 0.035f * Handling.Downforce;

            // TEMP experiment: a car carrying the off-road FLAG has permanently raised gravity (set once in
            // Initialize, not a surface state), and its grip is scaled with it.
            float gravityGs = Handling.Gravity / 9.8f;
            if (gravityGs > 1f) handlingGrip *= gravityGs;

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
                if (ARS.OverspeedEnabled)
                {
                    List<float> wheelPowers = ARS.WheelPowers(Car);
                    float wheelGs = 0f;
                    foreach (float p in wheelPowers) wheelGs += p;
                    float measuredGs = VehicleData.GetLongitudinalGs(Car.ForwardVector);
                    float excess = measuredGs - wheelGs - 0.1f;
                    VehicleData.OverspeedMeasuredGs = measuredGs;
                    VehicleData.OverspeedWheelGs = wheelGs;
                    VehicleData.OverspeedExcessGs = excess;
                }
            }


            if (Math.Abs(Brain.CurrentPerception.DeviationFromCenter) < CurrentTrackPoint.TrackHalfWidth && RacePosition <= 2 && !ARS.TerrainGripMultipliers.ContainsKey(CurrentTrackPoint.Node))
            {
                ARS.TerrainGripMultipliers.Add(CurrentTrackPoint.Node, GroundGripMultiplier);
            }

            VehicleData.YawRotationPerSecondDegrees = ARS.RadToDeg(Function.Call<Vector3>(Hash.GET_ENTITY_ROTATION_VELOCITY, Car).Z);
        }
        public void UpdateRivals()
        {
            Vector3 myPosition = Car.Position;
            Vector3 hoodPosition = myPosition + Car.ForwardVector;
            List<Racer> candidates = new List<Racer>();
            List<float> candidateDistances = new List<float>();
            foreach (Racer r in ARS.Racers)
            {
                if (r.Car.Handle == Car.Handle) continue;
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
        }

        public void Delete()
        {
            if (!Driver.IsPlayer)
            {
                Driver.Delete();
            }

            if (Game.Player.Character.IsInVehicle(Car))
            {
                Game.Player.Character.SetIntoVehicle(Car, VehicleSeat.Driver);
                Car.IsPersistent = false;
            }
            else
            {
                if (Car.CurrentBlip != null && Car.CurrentBlip.Exists()) Car.CurrentBlip.Remove();
                Car.Delete();
            }
        }
    }
}


