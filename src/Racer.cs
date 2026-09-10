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
        public Ped Driver;
        public Vehicle Car;
        public Team TeamRole = Team.None;
        public bool ControlledByPlayer = false;
        public RacerBaseBehavior BaseBehavior = RacerBaseBehavior.GridWait;
        public RaceState RCStatus = RaceState.NotInitiated;


        public VehicleControl Control = new VehicleControl();
        public RacerBrain Brain = new RacerBrain();

        // Dormant legacy state retained for the disabled live-corner and route-probe systems.
        public CornerPoint LiveCorner = new CornerPoint();
        public int CornerScanNode = -1;
        public int RouteTargetNode = -1;
        public float RouteTargetRadius = 999f;

        // Four nearest precomputed apexes ahead, nearest first.
        public int NextApexNode = -1;
        public float NextApexRadius = 999f;
        public float NextApexSpeed = 999f;
        public int NextApexNode2 = -1;
        public float NextApexRadius2 = 999f;
        public float NextApexSpeed2 = 999f;
        public int NextApexNode3 = -1;
        public float NextApexRadius3 = 999f;
        public float NextApexSpeed3 = 999f;
        public int NextApexNode4 = -1;
        public float NextApexRadius4 = 999f;
        public float NextApexSpeed4 = 999f;
        int _lastApexProgressNode = -1;


        public VehicleState VehicleData = new VehicleState();
        public HandlingData Handling = new HandlingData();
        public float GroundGripMultiplier = 1f;
        Vector3 _lastSpeed;

        
        int _lastStabilityCheck = 0;
        int _wheelsOffGround = 0;

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
        int _stuckRecoveryAttempts = 0;
        public int StuckRecoveryAttemptsNow => _stuckRecoveryAttempts;
        public bool IsRecoveringFromStuckNow => _isRecoveringFromStuck;


        struct TrailSample
        {
            public Vector3 Position;
            public float CombinedInput;

            public TrailSample(Vector3 position, float combinedInput)
            {
                Position = position;
                CombinedInput = combinedInput;
            }
        }
        List<TrailSample> _trailSamples = new List<TrailSample>();


        public float RouteLookAheadSeconds = 0.5f;
        public float RouteLookaheadSizeSeconds = 2.0f;


        float _avoidLeftWall = 0f;
        float _avoidRightWall = 0f;
        int _activeRivalWallCount = 0;
        bool _avoidWallsInitialized = false;
        float _targetLane = 0f;
        float _rawCornerLane = 0f;
        float _slideCountersteerDegrees = 0f;
        float _cornerSpd = 999f;
        float _debugCornerSpd = 999f;
        float _debugFollowTrackSpd = 999f;
        float _debugHillPitch = 0f;

        // Latched on corner approach when entry speed warrants holding the outside line.
        bool _approachOutsideDecided = false;
        bool _approachHoldsOutside = false;
        int _approachCornerNode = -1;
        int _divebombApexNode = -1;
        int _defendApexNode = -1;

        const float OffshootRangeMeters = 2f;
        const float OffshootBlendBrake = 0.5f; // projection blend target: 0.5 brake
        const float FullPedalSpeedErrorMps = 3f;
        static readonly float StationarySpeedThresholdMps = ARS.MphToMps(5f);

        // Empirical steer-limit memory: peak lateral G reference and the steer that produced it.
        float _latGRef = 0f;
        float _peakSteerDeg = 15f;
        bool _nearGripPeak = false;

        // True when the speed-based steering limiter actually reduced the steer this frame.
        bool _steerLimitedThisFrame = false;
        float _steerLimitDegrees = 999f;
        float _requestedSteerDegrees = 0f;

        // Brake learning (Phase 1): learn the effective decel factor per corner apex.
        const float BrakeFactorDefault = 0.95f;
        readonly Dictionary<int, float> _brakeFactorsByApex = new Dictionary<int, float>();
        float _brakeSampleSeconds = 0f;
        float _brakeSampleFullInput = 0f;
        int _brakeSampleApexNode = -1;
        const float BrakeSampleThreshold = 0.5f; // only samples above this count toward the average
        const float BrakeFullTimeTarget = 0.33f; // target seconds at full brake per braking phase
        const float BrakeAdjustGain = 0.3f;      // proportional factor step on the full-brake-time error
        const float BrakeMinFactor = 0.5f;       // learned factor range floor
        const float BrakeMaxFactor = 1.2f;
        // Read by ARS.MaxSpeedForBrakingDistance (static) to scale its decel plan.
        public float BrakeFactorForApex(int apexNode) => _brakeFactorsByApex.TryGetValue(apexNode, out float f) ? f : BrakeFactorDefault;
        float _divebombBrakeBonus; // temp brake boost while diving, whole hundredths 2-8 drawn per dive, never committed to learning

        // While diving, a temp bonus is appended to the learned factor; the stored factor itself never changes.
        public float EffectiveBrakeFactor(int apexNode)
        {
            return ActiveManeuver.Type == ManeuverType.DiveBomb
                ? BrakeFactorForApex(apexNode) + _divebombBrakeBonus
                : BrakeFactorForApex(apexNode);
        }



        bool _isPassengerized = false;

        // Feature gate for AI nitrous (permission resolved via ARS.AiNitro setting).
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

        public Maneuver ActiveManeuver = new Maneuver();
        int _nitrousActiveUntil = 0;
        int _nitrousLapUsed = -1;


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

                if (ARS.RacersMenuStore.GetInt("AIRacerAutofix", 1) == 2)
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
                else if (ARS.RacersMenuStore.GetInt("AIRacerAutofix", 1) == 1)
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
                ARS.SetSteerAngle(Car, 0.5f);
                ARS.SetThrottle(Car, 0f);
                ARS.SetBrakes(Car, 0f);

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
            Control.SteerDegrees = 0f;
            CurrentTrackPoint = ARS.TrackPoints.Last();
            Control.Brake = 0f;
            Control.Throttle = 0f;

            LapTimes.Clear();
            LapStartTime = 0;
            Lap = 0;
            NitroChargedLap = -1;
            _nitrousLapUsed = -1;
            RacePosition = 0;
            CanRegisterNewLap = false;
            _previousNode = -1;

            string flags = ARS.GetHandlingFlags(Car).ToString("X");
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
            bool modelElectric = Function.Call<int>((Hash)0xD839450756ED5A80, Car.Model.Hash) != 0;
            VehicleData.PowerScale = ARS.ComputePaceIndex(modelTopSpeedMph, modelGrip, modelAccel, modelElectric);
            VehicleData.TextPerformanceIndex = VehicleData.PowerScale.ToString("0.00");
            if (!ControlledByPlayer) Name = _baseName + " (" + VehicleData.PowerScale.ToString("0.00") + ")";

            _brakeFactorsByApex.Clear();
            foreach (CornerPoint corner in ARS.Corners)
                _brakeFactorsByApex[corner.Node] = ARS.Remap(corner.SupposedRadius, 25f, 250f, 0.5f, 1f, true);

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

            // Heading error is kept separate from lane bias; merging them caused oscillation.
            Vector3 carForward = Car.ForwardVector;
            if (Car.Velocity.LengthSquared() > 0.01f) carForward = Car.Velocity.Normalized;
            float headingErrorDeg = -Vector3.SignedAngle(steerRefPoint.Direction, carForward, Vector3.WorldUp);
            if (float.IsNaN(headingErrorDeg) || float.IsInfinity(headingErrorDeg)) headingErrorDeg = 0f;
            headingErrorDeg *= 1.0f;


            float defaultLane = ComputeHighSpeedLane(roadWide, speedMps);
            bool gotActiveCorner = Brain.Corner != null && Lap > 0;
            float cornerLane = 0f;
            if (gotActiveCorner) cornerLane = ComputeCornerTargetLane(steerRefPoint, speedMps);
            if (cornerLane != 0f) defaultLane = cornerLane;
            _rawCornerLane = cornerLane;
            float avoidAheadLane = ComputeAvoidAheadLane(roadWide);
            float avoidLookaheadDist = 0f;
            if (avoidAheadLane != 0f)
            {
                defaultLane = avoidAheadLane;
                if (Brain.AvoidanceTarget != null) avoidLookaheadDist = Brain.AvoidanceTarget.SecondsToReach * speedMps;
            }
            float targetLane = ApplyRivalWalls(defaultLane, roadWide);
            _targetLane = targetLane;


            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            float absDev = Math.Abs(Brain.CurrentPerception.DeviationFromCenter);
            float safeEdge = roadWide - carHalfWidth;
            float overshoot = absDev - safeEdge;
            float recoveryDeg = 0f;
            if (overshoot > 0f)
            {
                float maxRecoveryDeg = ARS.Remap(ARS.MpsToMph(speedMps), 100f, 10f, 3f, 45f, true);
                float severity = ARS.Clamp(overshoot / Math.Max(carHalfWidth, 0.1f), 0f, 1f);
                recoveryDeg = Math.Sign(Brain.CurrentPerception.DeviationFromCenter) * maxRecoveryDeg * severity;
            }


            float trackBound = roadWide - carHalfWidth;
            bool hasActiveGuidance = Math.Abs(targetLane) > 0.01f || _avoidLeftWall > -trackBound || _avoidRightWall < trackBound;
            float laneBiasDeg = 0f;
            if (hasActiveGuidance)
            {
                float currentLane = Brain.CurrentPerception.DeviationFromCenter;
                float lookaheadDist = steerRefPoint.Position.DistanceTo(Car.Position);
                if (avoidLookaheadDist > 0f) lookaheadDist = avoidLookaheadDist;
                if (lookaheadDist < 1f) lookaheadDist = speedMps * 1.5f;
                float laneError = targetLane - currentLane;
                if (ARS.DebugToggles[Options.GsAwarePreview]
                    && LookAheads.TryGetValue(LookAhead.HalfSec, out TrackPoint halfSecPoint)
                    && halfSecPoint != null)
                {
                    Vector3 projection = ProjectAhead(0.5f);
                    float projectedLane = ARS.SignedLaneOffset(projection, halfSecPoint.Position, halfSecPoint.Direction);
                    float blend = ARS.Clamp(ARS.GsAwarePreviewBlend, 0f, 1f);
                    laneError = laneError * (1f - blend) + (targetLane - projectedLane) * blend;
                }
                laneBiasDeg = -(float)(Math.Atan2(laneError, lookaheadDist) * (180.0 / Math.PI)) * 1.0f;
                Vector3 velDir = Car.ForwardVector;
                if (Car.Velocity.LengthSquared() > 0.01f) velDir = Car.Velocity.Normalized;
                Vector3 velRight = Vector3.Cross(Vector3.WorldUp, velDir);
                foreach (Rival r in Brain.Rivals)
                {
                    if (r.RivalRacer == null || !r.RivalRacer.Car.Exists()) continue;
                    Vector3 delta = r.RivalRacer.Car.Position - Car.Position;
                    float longComp = Vector3.Dot(delta, velDir);
                    float latComp = Vector3.Dot(delta, velRight);
                    float combinedHalfLen = (VehicleData.BoundingBox + r.RivalRacer.VehicleData.BoundingBox) * 0.5f;
                    if (Math.Abs(longComp) > combinedHalfLen) continue;
                    float rivalHalfWidth = r.RivalRacer.VehicleData.BoundingBox * 0.5f;
                    float penetration = (carHalfWidth + rivalHalfWidth) - Math.Abs(latComp);
                    if (penetration <= 0f) continue;
                    float awayDir = -Math.Sign(latComp);
                    laneBiasDeg += awayDir * penetration * 2f;
                }
            }

            float sideBySideHeadingDeg = ComputeSideBySideHeadingCorrection(carForward);

            const float steerKP = 1.0f;
            // TEMP: hardcoded 0.66 — testing yaw damping.
            const float steerKD = 0.66f;
            float trajectorySteer = (steerKP * (headingErrorDeg + recoveryDeg + sideBySideHeadingDeg)) - (steerKD * VehicleData.YawRotationPerSecondDegrees);
            Control.SteerDegrees = trajectorySteer + (steerKP * laneBiasDeg);
            bool sameSignSlideYaw = Math.Sign(VehicleData.SlideAngle) == Math.Sign(VehicleData.YawRotationPerSecondDegrees);
            _slideCountersteerDegrees = 0f;
            if (sameSignSlideYaw)
            {
                float slideScale = ARS.Remap(Math.Abs(VehicleData.SlideAngle), 0f, Handling.LateralTractionCurve * 1.2f, 0.5f, 1.2f, true);
                _slideCountersteerDegrees = VehicleData.SlideAngle * slideScale;
                Control.SteerDegrees -= _slideCountersteerDegrees;
                // Slide priority: blend toward trajectory steering + full countersteer, dropping the lane pursuit.
                if (Handling.LateralTractionCurve > 1f)
                {
                    float slidePriority = ARS.Remap(Math.Abs(VehicleData.SlideAngle), Handling.LateralTractionCurve * 0.5f, Handling.LateralTractionCurve * 1.2f, 0f, 1f, true);
                    float countersteerTarget = trajectorySteer - _slideCountersteerDegrees;
                    Control.SteerDegrees += (countersteerTarget - Control.SteerDegrees) * slidePriority;
                }
            }


            // NaN/Inf guard: ApplySteerLimits would turn NaN into full-lock.
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

        // Phase 1: match an overlapping rival's heading so side-by-side cars follow the same arc.
        float ComputeSideBySideHeadingCorrection(Vector3 carForward)
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

                float proximity = ARS.Remap(lateralDistance, wideDistance, fullAssistDistance, SideBySideMinimumAssist, 1f, true);
                float headingDifference = -Vector3.SignedAngle(rival.RivalRacer.Car.ForwardVector, carForward, Vector3.WorldUp);
                if (float.IsNaN(headingDifference) || float.IsInfinity(headingDifference)) continue;

                correction += headingDifference * proximity;
            }
            return correction;
        }

        // Lane Control System 2: positions the car on the inside edge of the track curvature.
        float ComputeHighSpeedLane(float roadWide, float speedMps)
        {
            int count = ARS.TrackPoints.Count;
            int fwdNode;
            int fwdOffset = (int)(speedMps * 0.9f);
            if (ARS.IsPointToPoint)
                fwdNode = (int)ARS.Clamp(CurrentTrackPoint.Node + fwdOffset, 0, count - 1);
            else
                fwdNode = ((CurrentTrackPoint.Node + fwdOffset) % count + count) % count;

            Vector3 currentDir = CurrentTrackPoint.Direction;
            Vector3 futureDir = ARS.TrackPoints[fwdNode].Direction;

            float signedAngle = Vector3.SignedAngle(currentDir, futureDir, Vector3.WorldUp);
            if (float.IsNaN(signedAngle) || float.IsInfinity(signedAngle)) return 0f;
            if (Math.Abs(signedAngle) <= 5f) return 0f;

            float cornerDir = Math.Sign(signedAngle);
            return -cornerDir * roadWide;
        }

        // Hold the outside line on entry, then release it for the high-speed inside line.
        float ComputeCornerTargetLane(TrackPoint steerRefPoint, float speedMps)
        {
            CornerPoint c = Brain.Corner.Point;
            int apexNode = c.Node;

            float distToApexNodes = Math.Abs(apexNode - CurrentTrackPoint.Node);
            float timeToApex = distToApexNodes / Math.Max(speedMps, 1f);
            float releaseSeconds = steerRefPoint.TrackHalfWidth * 0.33f;
            float approachStartTime = releaseSeconds + 2f;

            if (apexNode != _approachCornerNode || timeToApex > approachStartTime)
            {
                _approachCornerNode = apexNode;
                _approachOutsideDecided = false;
                _approachHoldsOutside = false;
                if (timeToApex > approachStartTime) return 0f;
            }

            // Flagged corners are too close to the previous one: no outside hold, no corner-commit.
            bool suppressOutside = ARS.Corners.Exists(cp => cp.Node == apexNode && cp.SuppressOutsideApproach);
            bool shouldHoldOutside = !suppressOutside; // TEMP: hold outside unless flagged, for testing
            if (!_approachOutsideDecided || (!_approachHoldsOutside && shouldHoldOutside))
            {
                _approachHoldsOutside = shouldHoldOutside;
                _approachOutsideDecided = true;
            }

            // Entrance-direction gate: don't hold the outside line if the direction at the
            // entrance diverges too far from the current track direction (the approach is misaligned).
            int entranceNode = c.StartNode >= 0 ? c.StartNode : OffsetCornerNode(apexNode, -c.LengthStart);
            if (entranceNode >= 0 && entranceNode < ARS.TrackPoints.Count)
            {
                float entranceHeading = Vector3.SignedAngle(ARS.TrackPoints[entranceNode].Direction, CurrentTrackPoint.Direction, Vector3.WorldUp);
                if (!float.IsNaN(entranceHeading) && !float.IsInfinity(entranceHeading) && Math.Abs(entranceHeading) > 60f)
                    return 0f;
            }

            float cornerDir = Math.Sign(c.Angle);
            if (cornerDir == 0f) return 0f;

            float halfWidth = steerRefPoint.TrackHalfWidth;
            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            float safeBound = halfWidth - carHalfWidth;

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

            if (_approachHoldsOutside && timeToApex > releaseSeconds)
            {
                return cornerDir * halfWidth;
            }
            return 0f;
        }

        // Pick a lane to pass a rival ahead. If two rivals trigger on opposite sides, thread the needle.
        float ComputeAvoidAheadLane(float roadWide)
        {
            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            float trackBound = roadWide - carHalfWidth;
            float aggroBuffer = ARS.Remap(Aggression, 100f, 0f, 0.2f, 1.2f, true);
            float currentLane = Brain.CurrentPerception.DeviationFromCenter;

            Rival target = Brain.AvoidanceTarget;
            if (target == null || target.RivalRacer == null) return 0f;

            if (!TryPickAvoidanceSide(target, trackBound, aggroBuffer, carHalfWidth, currentLane, out float targetLane, out bool targetGoLeft))
                return 0f;

            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || r == target) continue;
                if (r.RelativePosition != RelativePos.Ahead) continue;
                if (!ARS.IsBetween(Math.Abs(r.DirectionDiff), 0f, 30f)) continue;
                if (!ARS.IsBetween(r.FrontGap, 0f, 3f) && !ARS.IsBetween(r.SecondsToHit, 0f, 5f)) continue;

                if (!TryPickAvoidanceSide(r, trackBound, aggroBuffer, carHalfWidth, currentLane, out float secondTarget, out bool secondGoLeft))
                    continue;

                if (secondGoLeft == targetGoLeft) continue;
                return (targetLane + secondTarget) * 0.5f;
            }

            return targetLane;
        }

        bool TryPickAvoidanceSide(Rival rival, float trackBound, float aggroBuffer, float carHalfWidth, float currentLane, out float passLane, out bool passLeft)
        {
            passLane = 0f;
            passLeft = false;

            float rivalLane = rival.OccupiedLane;
            float buffer = rival.OccupiedLaneWidth + aggroBuffer;

            float roomLeft = rivalLane - buffer + trackBound;
            float roomRight = trackBound - (rivalLane + buffer);
            passLeft = roomLeft > roomRight;

            passLane = passLeft ? rivalLane - buffer - carHalfWidth : rivalLane + buffer + carHalfWidth;

            if (Math.Abs(passLane) > trackBound)
            {
                passLane = passLeft ? rivalLane + buffer + carHalfWidth : rivalLane - buffer - carHalfWidth;
                passLeft = !passLeft;
            }

            if (Math.Abs(passLane) > trackBound) return false;

            if (passLeft && currentLane <= passLane) return false;
            if (!passLeft && currentLane >= passLane) return false;

            return true;
        }

        float ApplyRivalWalls(float targetLane, float roadWide)
        {
            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            float trackBound = roadWide - carHalfWidth;

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
                if (r.RelativePosition != RelativePos.Left && r.RelativePosition != RelativePos.Right) continue;

                float aggroBuffer = ARS.Remap(Aggression, 100f, 0f, 0.2f, 1.2f, true);
                float rivalBuffer = r.OccupiedLaneWidth + aggroBuffer;

                if (r.RelativePosition == RelativePos.Left)
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

            float openRate = 2f * TickScale;
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

        const float FullSteerThrottleCap = 0.9f;
        const float GripRefGain = 5f;               // proportional reference time constant (~0.2s)
        const float MinSteerLimit = 8f;             // floor for the steer clamp (degrees)
        const float ExploitUtilization = 0.9f;       // utilization at/above → exploit mode
        const float UtilizationFloorFactor = 0.5f;   // fraction of traction curve as utilization floor
        const float SpikeCapFactor = 1.1f;           // × declared grip = spike-rejection cap
        const float OversteerCutMargin = 10f;        // degrees past limit before throttle cut
        const float SteerSlewRate = 180f;                // fixed steering slew rate (degrees/second)
        const float SteerSlewRateCountersteer = 360f;    // doubled when countersteering (steer opposes yaw)
        // Game's player steering limiter (Automobile.cpp): speed-based reduction.
        const float PlayerSpeedSteerFwdThreshold = 0.001f;   // effectively always on
        // Steer reduction multiplier: 0.04 at throttle 0.5, 0.08 at throttle 0.99.
        // Floor: TRlat/3 (degrees) so high-speed steering doesn't collapse.


        void ApplySteerLimits()
        {
            _steerLimitedThisFrame = false;

            // NaN guard: Clamp would turn NaN into full-lock.
            if (float.IsNaN(Control.SteerDegrees) || float.IsInfinity(Control.SteerDegrees))
            {
                Control.SteerDegrees = 0f;
                return;
            }

            // Slide-angle steer limit ramps in with speed: full lock at standstill,
            // collapsing to |slide angle| + 2 by 10 m/s, staying at that value above.
            // Applies in either steering direction (steering-in or countersteer).
            float requestedSteer = Control.SteerDegrees;
            _requestedSteerDegrees = Math.Abs(requestedSteer);
            float fwdSpeed = Vector3.Dot(Car.Velocity, Car.ForwardVector);
            float fwdMph = ARS.MpsToMph(Math.Max(fwdSpeed, 0f));
            float slideAngle = Math.Abs(VehicleData.SlideAngle);
            // Max steer angle = 3° base, plus slide, capped at TRlat × 0.5.
            float maxSteerAngle = Math.Min(3f + slideAngle, Handling.LateralTractionCurve * 0.5f);
            // Brake rampdown: once slide exceeds the grip-based steer allowance, ease brake so tires regain lateral grip.
            float gripSteerAngle = 2f + Handling.LateralTractionCurve * 0.2f;
            Control.MaxBrake = slideAngle > gripSteerAngle ? ARS.Remap(slideAngle, gripSteerAngle * 2f, gripSteerAngle, 0.8f, 1f, true) : 1f;
            float maxSteer = ARS.Remap(fwdMph, 50f, 0f, maxSteerAngle, VehicleData.SteeringLock, true);
            _steerLimitDegrees = maxSteer;
            if (Math.Abs(requestedSteer) > maxSteer)
            {
                Control.SteerDegrees = Math.Sign(requestedSteer) * maxSteer;
                _steerLimitedThisFrame = true;
                if (Control.MaxThrottle >= 0.1) Control.MaxThrottle -= (float)(2 * TickScale);
            }

            /* ZOMBIE — speed-based reduction, disabled while trialing slide-angle steer limit.

            // Only limit when steering and yaw agree (car turning into the steer).
            if (Math.Sign(Control.SteerDegrees) != Math.Sign((int)VehicleData.YawRotationPerSecondDegrees))
                return;

            // Game's player steering limiter (Automobile.cpp): speed-based reduction.
            float fwdSpeed = Vector3.Dot(Car.Velocity, Car.ForwardVector);
            float preLimitSteer = Math.Abs(Control.SteerDegrees);
            _requestedSteerDegrees = preLimitSteer;
            float absoluteSteerLimit = VehicleData.SteeringLock;
            _steerLimitDegrees = absoluteSteerLimit;

            if (fwdSpeed > PlayerSpeedSteerFwdThreshold)
            {
                float speedSteerReduction = 0.5f;
                float divisor = 1f + speedSteerReduction * fwdSpeed;
                Control.SteerDegrees /= divisor;
                if (Math.Abs(Control.SteerDegrees) < preLimitSteer) _steerLimitedThisFrame = true;
                _steerLimitDegrees = absoluteSteerLimit / divisor;
            }

            const float MinSteerAngleFloorFactor = 0.3f;
            float MinSteerAngleFloor = Handling.LateralTractionCurve * MinSteerAngleFloorFactor;
            if (Control.SteerDegrees != 0f && Math.Abs(Control.SteerDegrees) < MinSteerAngleFloor)
                Control.SteerDegrees = Math.Sign(Control.SteerDegrees) * MinSteerAngleFloor;

            if (_steerLimitedThisFrame && Control.MaxThrottle>=0.1) Control.MaxThrottle -= (float)(2 * TickScale);
            */
        }


        public void Launch()
        {
            Brain.Corner = null;
            CornerScanNode = -1;
            RouteTargetNode = -1;
            RouteTargetRadius = 999f;
            NextApexNode = -1;
            NextApexRadius = 999f;
            NextApexSpeed = 999f;
            NextApexNode2 = -1;
            NextApexRadius2 = 999f;
            NextApexSpeed2 = 999f;
            NextApexNode3 = -1;
            NextApexRadius3 = 999f;
            NextApexSpeed3 = 999f;
            NextApexNode4 = -1;
            NextApexRadius4 = 999f;
            NextApexSpeed4 = 999f;
            _lastApexProgressNode = -1;
            ResetRouteProbe();
            VehicleData.AvgGroundStability = 1;
            BaseBehavior = RacerBaseBehavior.Race;
            Lap = ARS.IsPointToPoint ? 1 : 0;
            LapStartTime = ARS.IsPointToPoint ? Game.GameTime : 0;
            CanRegisterNewLap = false;
            _previousNode = -1;
            Control.HandBrakeTime = Game.GameTime + ARS.GetRandomInt(100, 400);
            Control.MaxThrottle = 1f;
            Control.MaxBrake = 1f;
            IsStuckByThrottle = false;
            _lastStuckGameTime = 0;
            _isRecoveringFromStuck = false;
            _stuckRecoveryEndTime = 0;
            _stuckRecoveryAttempts = 0;
            Control.LastAppliedSteerDegrees = 0f;
            if (TeamRole == Team.Cop) Car.SirenActive = true;

        }

        void ConvertSpeedToPedals()
        {
            float currentForwardSpeed = VehicleData.SpeedVectorLocal.Y;
            float inputChange = 2f * TickScale;
            float newThrottle = 0f;
            float newBrake = 0f;

            FindLowestIntendedSpeed();

            Brain.CurrentIntention.IntendedSpeedChange = Brain.CurrentIntention.Speed - currentForwardSpeed;

            float intendedSpeedChange = Brain.CurrentIntention.IntendedSpeedChange;

            float combinedInput = ComputeCombinedInput(intendedSpeedChange, currentForwardSpeed);
            combinedInput = ApplyOffshootBlend(combinedInput);
            combinedInput = ApplyThrottleCap(combinedInput);
            SplitCombinedInput(combinedInput, ref newThrottle, ref newBrake);

            Control.Brake += ARS.Clamp(newBrake - Control.Brake, -inputChange, inputChange);
            Control.Throttle += ARS.Clamp(newThrottle - Control.Throttle, -inputChange, inputChange);

            UpdateBrakeLearning();
            Control.Brake = Math.Min(Control.Brake, Control.MaxBrake);
            Control.Throttle = Math.Min(Control.Throttle, Control.MaxThrottle);
            if (Control.MaxThrottle < 1.00f) Control.MaxThrottle += 2 * TickScale;

            if (Brain.CurrentIntention.MaxSpeed < AiConstants.MaxSpeed) Brain.CurrentIntention.MaxSpeed += 15 * TickScale;

        }

        float ComputeCombinedInput(float intendedSpeedChange, float currentForwardSpeed)
        {
            if (intendedSpeedChange > 0f && ShouldBrakeBeforeDrivingForward(currentForwardSpeed))
                return -ARS.Clamp(intendedSpeedChange / FullPedalSpeedErrorMps, 0f, 1f);

            return ARS.Clamp(intendedSpeedChange / FullPedalSpeedErrorMps, -1f, 1f);
        }

        float ApplyThrottleCap(float combinedInput)
        {
            float throttleCap = Math.Min(Control.MaxThrottleFromTCS, 1f);

            if (combinedInput > 0f) return Math.Min(combinedInput, throttleCap);
            return combinedInput;
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

        bool ShouldBrakeBeforeDrivingForward(float speed) => speed < -StationarySpeedThresholdMps;

        void FindLowestIntendedSpeed()
        {
            if (Brain.CurrentIntention.Speed >= 0f)
            {
                Brain.CurrentIntention.Speed = Math.Min(Brain.CurrentIntention.Speed, ARS.EngineTopSpeed(Car) * 1.3f);
                Brain.CurrentIntention.Speed = Math.Min(Brain.CurrentIntention.Speed, Brain.CurrentIntention.MaxSpeed);
            }
        }

        // Projection response: when the 1s projection nears the outside edge of a corner, cap the
        // maximum combined input — full throttle at -2m inside the edge, 0.5 brake at +2m past it.
        float ApplyOffshootBlend(float combinedInput)
        {
            // Only meaningful when the car is aiming at a lane; with no target lane there is no
            // hug-inside expectation to enforce, so the outside sanity check must not fire.
            if (_targetLane == 0f) return combinedInput;

            Vector3 proj = ProjectAhead(1f);
            TrackPoint tp = ARS.FindNearestTrackPoint(proj, CurrentTrackPoint.Node);
            float signedOffset = ARS.SignedLaneOffset(proj, tp.Position, tp.Direction);
            float safeBound = tp.TrackHalfWidth - VehicleData.BoundingBox * 0.5f;
            float offTrackDistance = Math.Abs(signedOffset) - safeBound;

            // Outside is judged from the track angle 1s behind, so it stays relevant through the corner.
            int count = ARS.TrackPoints.Count;
            int behindOffset = (int)(Car.Velocity.Length() * 1f);
            int behindNode = ARS.IsPointToPoint
                ? (int)ARS.Clamp(CurrentTrackPoint.Node - behindOffset, 0, count - 1)
                : ((CurrentTrackPoint.Node - behindOffset) % count + count) % count;
            bool isOutsideCorner = Math.Sign(signedOffset) == Math.Sign(ARS.TrackPoints[behindNode].Angle);

            if (!isOutsideCorner) return combinedInput;

            float maxInput = ARS.Remap(offTrackDistance, OffshootRangeMeters, -OffshootRangeMeters, -OffshootBlendBrake, 1f, true);
            return Math.Min(combinedInput, maxInput);
        }

        // Samples braking quality across the approach to the current apex; the factor is
        // only committed when that apex is passed — in-progress braking never adjusts live.
        void UpdateBrakeLearning()
        {
            if (!ARS.DebugToggles[Options.BrakeLearning])
            {
                _brakeFactorsByApex.Clear();
                return;
            }

            if (ActiveManeuver.Type != ManeuverType.None) return;

            if (HasPassedBrakingTarget()) return;

            if (NextApexNode != _brakeSampleApexNode)
            {
                _brakeSampleApexNode = NextApexNode;
                _brakeSampleSeconds = 0f;
                _brakeSampleFullInput = 0f;
            }

            if (Control.Brake <= BrakeSampleThreshold) return;
            _brakeSampleSeconds += TickScale;
            if (Control.Brake >= 1f) _brakeSampleFullInput += Control.Brake * TickScale;
        }

        void CommitBrakeLearning()
        {
            if (!ARS.DebugToggles[Options.BrakeLearning]) return;
            if (_brakeSampleSeconds < MinimumBrakeSampleSeconds || _brakeSampleFullInput <= 0f || _brakeSampleApexNode < 0) return;
            float fullTime = _brakeSampleFullInput; // seconds at full brake
            float step = (BrakeFullTimeTarget - fullTime) * BrakeAdjustGain;
            float factor = BrakeFactorForApex(_brakeSampleApexNode) * (1f + step);
            _brakeFactorsByApex[_brakeSampleApexNode] = ARS.Clamp(factor, BrakeMinFactor, BrakeMaxFactor);
        }

        int CornerEntranceNode(CornerPoint corner, int apexNode)
        {
            return corner == null ? apexNode : (corner.StartNode >= 0 ? corner.StartNode : OffsetCornerNode(apexNode, -corner.LengthStart));
        }

        // Past the braking target (entrance node) the plan expects apex speed; later braking is corner-exit scrub, not approach.
        bool HasPassedBrakingTarget()
        {
            if (NextApexNode < 0) return true;
            CornerPoint corner = ARS.Corners.FirstOrDefault(c => c.Node == NextApexNode);
            int entranceNode = CornerEntranceNode(corner, NextApexNode);
            if (entranceNode < 0) return true;
            int entranceDistance = ForwardNodeDistance(entranceNode);
            int apexDistance = ForwardNodeDistance(NextApexNode);
            return entranceDistance <= 0 || (!ARS.IsPointToPoint && entranceDistance > apexDistance);
        }

        float TickScale => (0.001f * TimeSince_lastCoreTick);




        void TranslateSteerToInput()
        {

            if (float.IsNaN(Control.SteerDegrees) || float.IsInfinity(Control.SteerDegrees)) Control.SteerDegrees = 0f;

            // Fixed slew-rate limiter: the applied steer moves toward the target at a
            // fixed rate (180°/s), doubled to 360°/s when countersteering (steer opposes yaw).
            float error = Control.SteerDegrees - Control.LastAppliedSteerDegrees;
            bool countersteering = Math.Sign(Control.SteerDegrees) != Math.Sign(VehicleData.YawRotationPerSecondDegrees);
            float rate = countersteering ? SteerSlewRateCountersteer : SteerSlewRate;
            float maxDeltaPerTick = rate * TickScale;
            float delta = ARS.Clamp(error, -maxDeltaPerTick, maxDeltaPerTick);
            Control.SteerDegrees = Control.LastAppliedSteerDegrees + delta;

            // Average with the previous frame's applied steer to soften twitch.
            Control.SteerDegrees = (Control.SteerDegrees + Control.LastAppliedSteerDegrees) * 0.5f;

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
                // Plan braking against all held apexes; the most restrictive target governs.
                cornerSpd = Math.Max(2, ApexBrakingSpeed(NextApexNode, NextApexSpeed));
                if (NextApexNode2 >= 0)
                    cornerSpd = Math.Min(cornerSpd, Math.Max(2, ApexBrakingSpeed(NextApexNode2, NextApexSpeed2)));
                if (NextApexNode3 >= 0)
                    cornerSpd = Math.Min(cornerSpd, Math.Max(2, ApexBrakingSpeed(NextApexNode3, NextApexSpeed3)));
                if (NextApexNode4 >= 0)
                    cornerSpd = Math.Min(cornerSpd, Math.Max(2, ApexBrakingSpeed(NextApexNode4, NextApexSpeed4)));
            }
            else if (Brain.Corner != null) cornerSpd = Math.Max(2, ARS.MaxSpeedForBrakingDistance(Brain.Corner.Point, this));

            // Route speed from the triple-check circumradius window.
            float followTrackSpd = RouteSpeedEnabled
                ? RouteIdealSpeedForRadius(Brain.CurrentPerception.CurveRadiusToFollowPoint)
                : 999f;

            if (float.IsNaN(cornerSpd) || float.IsInfinity(cornerSpd)) cornerSpd = 999f;
            if (float.IsNaN(followTrackSpd) || float.IsInfinity(followTrackSpd)) followTrackSpd = 999f;
            if (cornerSpd <= 5) cornerSpd = ARS.CornerApexSpeed(Brain.Corner.Point, this);

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

                if (ARS.DebugToggles[Options.ShowTrackAnalysis]) _debugHillPitch = slopeAngleDeg;
            }

            // Crest/dip vertical curvature grip effect (route speed only).
            int count = ARS.TrackPoints.Count;
            int followNode = (int)ARS.Clamp(CurrentTrackPoint.Node + (int)(Car.Velocity.Length() * RouteLookAheadSeconds), 0, count - 1);
            int crestStartNode, crestEndNode;
            if (ARS.IsPointToPoint)
            {
                crestStartNode = (int)ARS.Clamp(followNode - 3, 0, count - 1);
                crestEndNode = (int)ARS.Clamp(followNode + 3, 0, count - 1);
            }
            else
            {
                crestStartNode = ((followNode - 3) % count + count) % count;
                crestEndNode = ((followNode + 3) % count + count) % count;
            }
            if (crestStartNode != crestEndNode && crestStartNode != followNode && crestEndNode != followNode)
            {
                Vector3 crestStart = ARS.TrackPoints[crestStartNode].Position;
                Vector3 crestMid = ARS.TrackPoints[followNode].Position;
                Vector3 crestEnd = ARS.TrackPoints[crestEndNode].Position;
                float deltaGs = ARS.HillGripDeltaGs(crestStart, crestMid, crestEnd, Car.Velocity.Length());
                // Crest aggression scales with route curvature: tight = cautious, straight = aggressive.
                float routeRadius = Brain.CurrentPerception.CurveRadiusToFollowPoint;
                float routeAggression = ARS.MapGamma(routeRadius, 100f, 300f, 0f, 1f, 0.5f, true);
                float routeCrestFloor = ARS.MapGamma(routeRadius, 100f, 500f, 0.4f, 0.8f, 0.5f, true);
                float effectiveDeltaGs = deltaGs;
                if (effectiveDeltaGs < 0f) effectiveDeltaGs *= (1f - routeAggression);
                float verticalGripFactor = Math.Max(1f + effectiveDeltaGs, routeCrestFloor);
                followTrackSpd *= (float)Math.Sqrt(verticalGripFactor);
            }



            // Pure apex speed for the corner-approach gate.
            _cornerSpd = NextApexNode >= 0 ? NextApexSpeed : (Brain.Corner != null ? ARS.CornerApexSpeed(Brain.Corner.Point, this) : 999f);
            float cornerApexSpeedWithVerticalGrip = _cornerSpd;

            // Corner crest/dip: same check as route, centered on the apex node.
            if (Brain.Corner != null)
            {
                int apexNode = Brain.Corner.Point.Node;
                int cornerCrestStart, cornerCrestEnd;
                if (ARS.IsPointToPoint)
                {
                    cornerCrestStart = (int)ARS.Clamp(apexNode - 3, 0, count - 1);
                    cornerCrestEnd = (int)ARS.Clamp(apexNode + 3, 0, count - 1);
                }
                else
                {
                    cornerCrestStart = ((apexNode - 3) % count + count) % count;
                    cornerCrestEnd = ((apexNode + 3) % count + count) % count;
                }
                if (cornerCrestStart != cornerCrestEnd && cornerCrestStart != apexNode && cornerCrestEnd != apexNode)
                {
                    Vector3 ccStart = ARS.TrackPoints[cornerCrestStart].Position;
                    Vector3 ccMid = ARS.TrackPoints[apexNode].Position;
                    Vector3 ccEnd = ARS.TrackPoints[cornerCrestEnd].Position;
                    float cornerDeltaGs = ARS.HillGripDeltaGs(ccStart, ccMid, ccEnd, _cornerSpd);
                    float cornerEffectiveDelta = cornerDeltaGs;
                    float cornerRadius = NextApexRadius;
                    float cornerAggression = ARS.MapGamma(cornerRadius, 100f, 300f, 0f, 1f, 0.5f, true);
                    float cornerCrestFloor = ARS.MapGamma(cornerRadius, 100f, 500f, 0.4f, 0.8f, 0.5f, true);
                    if (cornerEffectiveDelta < 0f) cornerEffectiveDelta *= (1f - cornerAggression);
                    float cornerVerticalGrip = Math.Max(1f + cornerEffectiveDelta, cornerCrestFloor);
                    float cornerVerticalSpeedFactor = (float)Math.Sqrt(cornerVerticalGrip);
                    cornerSpd *= cornerVerticalSpeedFactor;
                    cornerApexSpeedWithVerticalGrip *= cornerVerticalSpeedFactor;
                }
            }

            // During the generated corner region, let route speed govern by invalidating the corner map.
            CornerPoint activeCorner = NextApexNode >= 0
                ? ARS.Corners.FirstOrDefault(c => c.Node == NextApexNode)
                : null;


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

            followTrackSpd += ARS.MphToMps(6f); // TEMP diagnostic: push follow-track speed out

            _debugCornerSpd = cornerSpd;
            _debugFollowTrackSpd = followTrackSpd;
            Brain.CurrentIntention.Speed = Math.Min(cornerSpd, followTrackSpd) + ARS.MphToMps(8f);
            // Physics-limited cornering speed for the current high-speed curve radius.
            Brain.CurrentIntention.CorneringSpeedLimit = (float)Math.Sqrt(9.8f * VehicleData.CurrentMechanicalGrip * Brain.CurrentPerception.HighSpeedCurveRadius);

            // Yield: cap throttle to 0.5 to stay behind.
            if (ActiveManeuver.Type == ManeuverType.Yield && ActiveManeuver.Target != null)
            {
                Control.MaxThrottle = Math.Min(Control.MaxThrottle, 0.5f);
            }

            // ChillOut: half throttle and hold a standoff behind the closest rival ahead.
            if (ActiveManeuver.Type == ManeuverType.ChillOut)
            {
                Control.MaxThrottle = Math.Min(Control.MaxThrottle, ChillThrottleCap);
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

            // Temporarily neutralized: keep the acceleration cap at 1 until rear-end
            // avoidance has a dedicated speed-control implementation.
        }

        const float SlopeGripLossK = 3f;
        const float SlopeGripLossExp = 2f;
        // Reduces crest-induced grip loss as curvature allows more aggressive traversal.

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

        void TractionControl()
        {
            float wheelspin = ARS.MaxWheelSlip(Car);

            float IdealWheelspin;
            if (OutOfTrackDistance() > 0f)
            {
                IdealWheelspin = -0.25f;  // off-track: tame target
            }
            else
            {
                // On-track: allow more wheelspin as the car slides (slide angle in degrees, /10).
                IdealWheelspin = -3f - Math.Abs(VehicleData.SlideAngle) / 10f;
                IdealWheelspin = ARS.Clamp(IdealWheelspin, -6f, 0f);  // magnitude capped at 6
            }

            float error = wheelspin - IdealWheelspin;
            float change = error * TickScale * 2f;
            Control.MaxThrottleFromTCS = ARS.Clamp(Control.MaxThrottleFromTCS + change, 0.25f, 1);
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

        int BehindNodeDistance(int targetNode)
        {
            int behind = CurrentTrackPoint.Node - targetNode;
            if (!ARS.IsPointToPoint && behind < 0) behind += ARS.TrackPoints.Count;
            return behind;
        }

        void UpdateNitrous()
        {
            if (ControlledByPlayer || !ARS.AiNitroAllowed()) return;

            if (Game.GameTime < _nitrousActiveUntil)
            {
                Function.Call((Hash)CheatPowerIncreaseHash, Car, NitrousPowerMultiplier);
                return;
            }
            if (_nitrousActiveUntil > 0) StopNitrous();
        }

        // Nitro card: valid when the shot is available and the straight is long enough;
        // appropriate when contested (faster rival ahead), defended (rival behind closing in
        // within the burn's reach), lonely (empty endless straight while slow), or spent
        // near the finish with a rival nearby.
        bool TryPlayNitrousCard()
        {
            if (!ARS.AiNitroAllowed() || Lap <= _nitrousLapUsed) return false;
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
            float totalLaps = ARS.SettingsFile.GetValue("GENERAL_SETTINGS", "Laps", 5);
            return Math.Max(0f, (totalLaps + 1f - Lap) * nodeCount - CurrentTrackPoint.Node);
        }

        void StartNitrous()
        {
            Function.Call(Hash.REQUEST_NAMED_PTFX_ASSET, NitrousPtfxAsset);
            Function.Call((Hash)FullyChargeNitrousHash, Car);
            Function.Call((Hash)OverrideNitrousLevelHash, Car, true, 1.0f, 50.0f, 100.0f, false);
            _nitrousActiveUntil = Game.GameTime + NitrousDurationMs;
            _nitrousLapUsed = Lap;
        }

        void StopNitrous()
        {
            Function.Call((Hash)CheatPowerIncreaseHash, Car, 1.0f);
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

            Vector3 accel = (cSpeed - _lastSpeed) / Game.LastFrameTime;
            VehicleData.AccelSum += accel - VehicleData.AccelerationVector[VehicleData.AccelHead];
            VehicleData.AccelerationVector[VehicleData.AccelHead] = accel;
            VehicleData.AccelHead = (VehicleData.AccelHead + 1) % VehicleState.AccelWindow;
            if (VehicleData.AccelCount < VehicleState.AccelWindow) VehicleData.AccelCount++;

            _lastSpeed = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, false);
            VehicleData.SpeedVectorGlobal = cSpeed;
            VehicleData.SpeedVectorLocal = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true);
            Brain.CurrentPerception.SpeedVector = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true);

            if (ARS.DebugToggles[Options.ShowInputs] && !Driver.IsPlayer)
            {
                float combinedInput = ARS.Clamp(Control.Throttle - Control.Brake, -1f, 1f);
                if (_trailSamples.Count == 0)
                {
                    _trailSamples.Add(new TrailSample(Car.Position, combinedInput));
                }
                else if (Car.Position.DistanceTo(_trailSamples[_trailSamples.Count - 1].Position) > 1f)
                {
                    _trailSamples.Add(new TrailSample(Car.Position, combinedInput));
                }
            }

            while (_trailSamples.Count > 100) _trailSamples.RemoveAt(0);
        }


        // Kinematic projection: pos + v*t + 0.5*a*t^2. Default t=1 (1s).
        public Vector3 ProjectAhead(float seconds = 1f)
        {
            return Car.Position + Car.Velocity * seconds + 0.5f * VehicleData.AverageAcceleration * seconds * seconds;
        }



        public void ProcessTick()
        {
            UpdateTickData();
            DrawRacerDebug();

            if (!Driver.IsPlayer)
            {
                ApplyInputs();
            }
        }

        public void RunTimedCore()
        {
            UpdateTrackPosition();
            UpdateSlideAndBoundingBox();
            UpdatePerceivedGrip();
            // Legacy live-corner scan and route probe remain disabled.
            // UpdateNextApexes supplies corner state.
            // UpdateCornerValidity();
            // UpdateRouteTarget();

            ProcessAI();
            if (Driver.IsPlayer && ARS.SettingsFile.GetValue("CATCHUP", "OnlyBehindPlayer", true)) ARS.CatchupPosition = RacePosition;

            _lastCoreTick = Game.GameTime;
        }



        void UpdateSlideAndBoundingBox()
        {
            VehicleData.BoundingBox = ARS.SlidingBoundingBoxWidth(Car);
            VehicleData.SlideAngle = (float)Math.Round(Vector3.SignedAngle(Car.Velocity.Normalized, Car.ForwardVector, Car.UpVector), 3);
        }
        void UpdateCornerValidity()
        {
            ARS.FindNextCorner(this);
        }







        void UpdatePassengerSeat()
        {
            if (Driver.IsPlayer) return;

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


            if (Driver.IsSittingInVehicle(Car) && !Driver.IsPlayer)
            {
                UpdatePassengerSeat();

                if (Control.HandBrakeTime > Game.GameTime) Car.HandbrakeOn = true; else Car.HandbrakeOn = false;

                VehicleMemory.SetThrottle(Car, ARS.Clamp(Control.Throttle, -1, 1));
                VehicleMemory.SetBrakes(Car, Control.Brake);
                VehicleMemory.SetSteerAngle(Car, Control.SteerInput);

            }
            else
            {
                VehicleMemory.SetThrottle(Car, 0f);
                VehicleMemory.SetBrakes(Car, 0f);
                VehicleMemory.SetSteerInput(Car, 0f);
            }
        }
        void DrawRacerDebug()
        {
            bool requestedInputs = ARS.DebugToggles[Options.ShowInputs];
            bool requestedTrack = ARS.DebugToggles[Options.ShowTrackAnalysis];
            bool requestedAggro = ARS.DebugToggles[Options.ShowAggro];
            if (!requestedInputs && !requestedTrack && !requestedAggro) return;

            // Card-state chevrons stay per-car, within 50m of the player.
            if (requestedAggro && !Driver.IsPlayer)
            {
                if (Car.Position.DistanceTo(Game.Player.Character.Position) <= 50f) DrawManeuverStateChevron();
            }

            // Panel and lane/projection visuals belong to the AI racer closest to the player.
            if (ARS.DebugFocusRacer != this) return;

            if (requestedInputs)
            {
                DrawInputTrails();
                DrawWheelDirectionLine();
                DrawSteerTargetLine();
                DrawFollowPointLine();
            }

            if (requestedTrack)
            {
                DrawCornerDebug();
                DrawProjectionDebug();
                DrawCollisionThreatDebug();
            }

            DrawDebugPanel(requestedInputs, requestedTrack);
        }

        // Maneuver-state chevron above the car: green = no card, blue = passive (Yield/ChillOut), orange = active (DiveBomb/DefendLane).
        void DrawManeuverStateChevron()
        {
            if (Driver.IsPlayer) return;
            Color stateColor;
            switch (ActiveManeuver.Type)
            {
                case ManeuverType.Yield:
                case ManeuverType.ChillOut:
                    stateColor = Color.Blue;
                    break;
                case ManeuverType.DiveBomb:
                case ManeuverType.DefendLane:
                    stateColor = Color.Orange;
                    break;
                default:
                    stateColor = Color.Green;
                    break;
            }
            Vector3 pos = Car.Position + new Vector3(0, 0, Car.Model.GetDimensions().Z + 0.3f);
            bool isAwd = IsAwd();
            MarkerType marker = isAwd ? MarkerType.ChevronUpx2 : MarkerType.ChevronUpx1;
            World.DrawMarker(marker, pos, Vector3.Zero, Vector3.Zero, new Vector3(0.5f, 0.5f, 0.5f), stateColor, false, true, 0, false, "", "", false);
        }

        // Handling fDriveBiasFront: 0 = RWD, 1 = FWD, anything between = AWD.
        bool IsAwd()
        {
            float bias = VehicleMemory.GetDriveBiasFront(Car);
            return bias > 0.01f && bias < 0.99f;
        }

        void DrawProjectionDebug()
        {
            // Projection debug: car to 0.5s to 1s. Red if off-track.
            Vector3 projectedHalf = ProjectAhead(0.5f);
            Vector3 projected = ProjectAhead();
            Vector3 lineStart = Car.Position + new Vector3(0, 0, Car.Model.GetDimensions().Z * 0.6f);
            ARS.DrawLine(lineStart, projectedHalf, Color.White);
            ARS.DrawLine(projectedHalf, projected, Color.White);

            TrackPoint projectedTrackPoint = ARS.TrackPoints.OrderBy(t => t.Position.DistanceTo2D(projected)).First();
            float projectedLateralOffset = Math.Abs(ARS.SignedLaneOffset(projected, projectedTrackPoint.Position, projectedTrackPoint.Direction));
            float projectedSafeBound = projectedTrackPoint.TrackHalfWidth - VehicleData.BoundingBox * 0.5f;
            bool willGoOffTrack = projectedLateralOffset > projectedSafeBound;

            Color projectionColor = willGoOffTrack ? Color.Red : Color.White;
            World.DrawMarker(MarkerType.DebugSphere, projected, Vector3.Zero, Vector3.Zero, new Vector3(0.5f, 0.5f, 0.5f), projectionColor, false, false, 0, false, "", "", false);
            World.DrawMarker(MarkerType.DebugSphere, projectedHalf, Vector3.Zero, Vector3.Zero, new Vector3(0.4f, 0.4f, 0.4f), projectionColor, false, false, 0, false, "", "", false);

            // Track edges at the projected progress.
            Vector3 trackRight = Vector3.Cross(projectedTrackPoint.Direction, Vector3.WorldUp).Normalized;
            Vector3 leftEdge = projectedTrackPoint.Position - trackRight * projectedTrackPoint.TrackHalfWidth;
            Vector3 rightEdge = projectedTrackPoint.Position + trackRight * projectedTrackPoint.TrackHalfWidth;
            ARS.DrawLine(leftEdge, rightEdge, willGoOffTrack ? Color.Red : Color.Green);
        }

        void DrawCollisionThreatDebug()
        {
            Rival threat = Brain.Rivals.FirstOrDefault(r => r.RivalRacer != null && r.RivalRacer.Car.Exists() && ARS.IsBetween(r.FrontGap, 0f, 50f));
            if (threat == null) return;
            Vector3 from = Car.Position + new Vector3(0, 0, Car.Model.GetDimensions().Z * 0.6f);
            Vector3 to = threat.RivalRacer.Car.Position + new Vector3(0, 0, threat.RivalRacer.Car.Model.GetDimensions().Z * 0.6f);
            ARS.DrawLine(from, to, Color.Magenta);
        }

        void DrawCornerDebug()
        {
            Vector3 from = Car.Position + new Vector3(0, 0, 0.6f);
            DrawApexDebug(NextApexNode, Color.Yellow, from);
            DrawApexDebug(NextApexNode2, Color.Orange, from);

            foreach (CornerPoint corner in ARS.Corners)
            {
                int startNode = corner.StartNode >= 0
                    ? corner.StartNode
                    : OffsetCornerNode(corner.Node, -corner.LengthStart);
                int endNode = corner.EndNode >= 0
                    ? corner.EndNode
                    : OffsetCornerNode(corner.Node, corner.LengthEnd);
                DrawCornerBoundary(startNode, Color.Cyan);
                DrawCornerBoundary(endNode, Color.Magenta);
            }
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

        void DrawCornerBoundary(int node, Color color)
        {
            if (node < 0 || node >= ARS.TrackPoints.Count) return;

            TrackPoint point = ARS.TrackPoints[node];
            Vector3 right = Vector3.Cross(point.Direction, Vector3.WorldUp).Normalized;
            Vector3 center = point.Position + new Vector3(0, 0, 0.35f);
            ARS.DrawLine(center - right * point.TrackHalfWidth, center + right * point.TrackHalfWidth, color);
        }

        void DrawApexDebug(int apexNode, Color color, Vector3 from)
        {
            if (apexNode < 0 || apexNode >= ARS.TrackPoints.Count) return;

            Vector3 apexPosition = ARS.TrackPoints[apexNode].Position + new Vector3(0, 0, 0.6f);
            ARS.DrawLine(from, apexPosition, color);
            World.DrawMarker(MarkerType.DebugSphere, apexPosition, Vector3.Zero, Vector3.Zero, new Vector3(0.35f, 0.35f, 0.35f), color, false, false, 0, false, "", "", false);
        }

        void DrawDebugPanel(bool showInputs, bool showTrack)
        {
            int lineCount = (showInputs ? 8 : 0) + (showTrack ? 2 : 0);
            if (lineCount == 0) return;

            float lineHeight = 0.026f;
            float top = 0.045f;
            float height = lineCount * lineHeight + 0.02f;
            bool limiterActive = _steerLimitedThisFrame;
            int r = 0;
            int g = 0;
            int b = 0;
            int a = 120;
            Function.Call(Hash.DRAW_RECT, 0.89f, top + height * 0.5f, 0.22f, height, r, g, b, a);

            float y = top + 0.01f;
            if (showInputs)
            {
                float allowedSteer = _steerLimitDegrees;
                bool requestingMore = _requestedSteerDegrees > allowedSteer + 0.5f;
                string steerText = "STEER " + allowedSteer.ToString("0.0") + "º";
                ARS.DrawText(new Vector2(0.79f, y), steerText,
                    requestingMore ? Color.Red : Color.White, ARS.DrawTextFont.Standard, ARS.DrawTextAlign.Left, 0.35f);
                y += lineHeight;

                float gravityOverEarth = Handling.Gravity / 9.8f;
                ARS.DrawText(new Vector2(0.79f, y), "GRV " + gravityOverEarth.ToString("0.00"),
                    Color.White, ARS.DrawTextFont.Standard, ARS.DrawTextAlign.Left, 0.35f);
                y += lineHeight;

                ARS.DrawText(new Vector2(0.79f, y), "GRP " + VehicleData.BaseMechanicalGrip.ToString("0.00"),
                    Color.White, ARS.DrawTextFont.Standard, ARS.DrawTextAlign.Left, 0.35f);
                y += lineHeight;

                ARS.DrawText(new Vector2(0.79f, y), "GMP " + VehicleData.CurrentMechanicalGrip.ToString("0.00"),
                    Color.White, ARS.DrawTextFont.Standard, ARS.DrawTextAlign.Left, 0.35f);
                y += lineHeight;

                ARS.DrawText(new Vector2(0.79f, y), "DF  " + Handling.Downforce.ToString("0.00"),
                    Color.White, ARS.DrawTextFont.Standard, ARS.DrawTextAlign.Left, 0.35f);
                y += lineHeight;

                float forwardMsPanel = ARS.GetForwardSpeed(Car);
                float routeRadiusPanel = Brain.CurrentPerception.CurveRadiusToFollowPoint;
                float lateralMsPanel = (routeRadiusPanel > 1f && !float.IsNaN(routeRadiusPanel) && !float.IsInfinity(routeRadiusPanel))
                    ? (forwardMsPanel * forwardMsPanel) / routeRadiusPanel
                    : 0f;
                float dfGs = ARS.GetDownforceGsAtSpeed(this, forwardMsPanel, lateralMsPanel);
                ARS.DrawText(new Vector2(0.79f, y), "DFG " + dfGs.ToString("0.00"),
                    Color.White, ARS.DrawTextFont.Standard, ARS.DrawTextAlign.Left, 0.35f);
                y += lineHeight;

                float speedDiff = _debugFollowTrackSpd - Car.Velocity.Length();
                ARS.DrawText(new Vector2(0.79f, y), "DIFF " + ARS.MpsToMph(speedDiff).ToString("0") + " mph",
                    Color.White, ARS.DrawTextFont.Standard, ARS.DrawTextAlign.Left, 0.35f);
                y += lineHeight;

                // Learned brake strength for the next apex (dive bonus included): >1 leans braking on, <1 eases off.
                bool learning = ARS.DebugToggles[Options.BrakeLearning];
                float brakeFactor = EffectiveBrakeFactor(NextApexNode);
                Color brakeColor = !learning ? Color.Gray : brakeFactor > 1.01f ? Color.Green : brakeFactor < 0.99f ? Color.Red : Color.White;
                ARS.DrawText(new Vector2(0.79f, y), "BRK  " + brakeFactor.ToString("0.00") + " x",
                    brakeColor, ARS.DrawTextFont.Standard, ARS.DrawTextAlign.Left, 0.35f);
                y += lineHeight;
            }

            if (showTrack)
            {
                DrawApexPanelLine(ref y, "A1", NextApexNode, NextApexSpeed, NextApexRadius, Color.Yellow, lineHeight);
                DrawApexPanelLine(ref y, "A2", NextApexNode2, NextApexSpeed2, NextApexRadius2, Color.Orange, lineHeight);
            }
        }

        void DrawFollowPointLine()
        {
            int count = ARS.TrackPoints.Count;
            if (count < 10) return;

            float traction = Math.Max(VehicleData.CurrentMechanicalGrip, 0.1f);
            int o2 = (int)(Car.Velocity.Length() / traction);
            int n2 = ARS.IsPointToPoint
                ? (int)ARS.Clamp(CurrentTrackPoint.Node + o2, 0, count - 1)
                : ((CurrentTrackPoint.Node + o2) % count + count) % count;

            Vector3 from = Car.Position + new Vector3(0, 0, 0.6f);
            Vector3 to = ARS.TrackPoints[n2].Position + new Vector3(0, 0, 0.6f);
            ARS.DrawLine(from, to, Color.Yellow);
        }

        void DrawApexPanelLine(ref float y, string label, int apexNode, float apexSpeed, float apexRadius, Color color, float lineHeight)
        {
            string value = apexNode >= 0
                ? label + "   " + ARS.MpsToMph(apexSpeed).ToString("0") + " mph | " + apexRadius.ToString("0") + " m"
                : label + "   --";
            ARS.DrawText(new Vector2(0.79f, y), value, color, ARS.DrawTextFont.Standard, ARS.DrawTextAlign.Left, 0.35f);
            y += lineHeight;
        }

        // Debug: lines to the route-window sample nodes.
        void DrawRouteFollowLine()
        {
            float speed = Car.Velocity.Length();
            int count = ARS.TrackPoints.Count;
            if (count < 10) return;

            float grip = Math.Max(VehicleData.CurrentMechanicalGrip, 0.1f);
            int o1 = (int)(speed / grip);        // start: car + velocity divided by grip
            int o3 = (int)(speed * 3f / grip);   // end: car + (velocity times 3) divided by grip
            if (o3 < o1 + 2) o3 = o1 + 2;
            int o2 = o1 + (o3 - o1) / 2;         // middle: midpoint between 1 and 3

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

            Vector3 from = Car.Position + new Vector3(0, 0, 0.6f);
            int[] nodes = { n1, n2, n3 };
            for (int i = 0; i < 3; i++)
            {
                Vector3 to = ARS.TrackPoints[nodes[i]].Position + new Vector3(0, 0, 0.6f);
                ARS.DrawLine(from, to, Color.White);
            }

            // Red lines to the two held next apexes (the braking-plan targets).
            if (NextApexNode >= 0)
            {
                Vector3 to = ARS.TrackPoints[NextApexNode].Position + new Vector3(0, 0, 0.6f);
                ARS.DrawLine(from, to, Color.Red);
            }
            if (NextApexNode2 >= 0)
            {
                Vector3 to = ARS.TrackPoints[NextApexNode2].Position + new Vector3(0, 0, 0.6f);
                ARS.DrawLine(from, to, Color.Red);
            }
            if (NextApexNode3 >= 0)
            {
                Vector3 to = ARS.TrackPoints[NextApexNode3].Position + new Vector3(0, 0, 0.6f);
                ARS.DrawLine(from, to, Color.Red);
            }
            if (NextApexNode4 >= 0)
            {
                Vector3 to = ARS.TrackPoints[NextApexNode4].Position + new Vector3(0, 0, 0.6f);
                ARS.DrawLine(from, to, Color.Red);
            }
        }

        void DrawInputTrails()
        {
            if (_trailSamples.Count < 2) return;

            for (int i = 1; i < _trailSamples.Count; i++)
            {
                TrailSample fromSample = _trailSamples[i - 1];
                TrailSample toSample = _trailSamples[i];
                Vector3 from = fromSample.Position;
                Vector3 to = toSample.Position;
                Vector3 segment = to - from;
                if (segment.Length() < 0.05f) continue;

                float inputFrom = fromSample.CombinedInput;
                float inputTo = toSample.CombinedInput;
                 Vector3 point = to;
                Vector3 away = segment.Normalized;
                float dimension = VehicleData.ModelDimensions.Y + 1f;
                Vector3 chevronScale = new Vector3(dimension / 2f, dimension / 4f, -(dimension / 2f));
                float value = ARS.Clamp((inputFrom + inputTo) * 0.5f, -1f, 1f);
                Color baseColor;
                if (value >= 0f)
                {

                    baseColor = ARS.GradientAtoBtoC(Color.White, Color.GreenYellow, Color.Green, value * 100f);
                }
                else
                {

                    float brake = -value;
                    baseColor = ARS.GradientAtoBtoC(Color.White, Color.Orange, Color.Red, brake * 100f);
                }
                Color finalColor = Color.FromArgb(255, baseColor.R, baseColor.G, baseColor.B);

                World.DrawMarker(MarkerType.ChevronUpx1, point, -away, new Vector3(90, 0, 0), chevronScale, finalColor, false, false, 0, false, "", "", false);
            }
        }

        // Debug: line at the front of the car showing where the front wheels point.
        // Angled by the steering angle; red when the speed-based steering limiter
        // actually reduced the steer that frame (or last).
        void DrawWheelDirectionLine()
        {
            if (Car == null || !Car.Exists()) return;

            // The actual wheel angle is the steering input (what's written to the
            // game) converted to degrees: SteerInput * SteeringLock.
            float steerDeg = Control.SteerInput * VehicleData.SteeringLock;
            if (float.IsNaN(steerDeg) || float.IsInfinity(steerDeg)) steerDeg = 0f;

            // Positive steer = left (CCW from above). Rotate the car's forward
            // vector by the steering angle around the up axis.
            Vector3 fwd = Car.ForwardVector;
            Vector3 DirAt(float deg)
            {
                float rad = deg * (float)Math.PI / 180f;
                float c = (float)Math.Cos(rad);
                float s = (float)Math.Sin(rad);
                return new Vector3(fwd.X * c - fwd.Y * s, fwd.X * s + fwd.Y * c, fwd.Z);
            }

            // Start at the front of the car, slightly above the ground.
            float halfLen = VehicleData.ModelDimensions.Y * 0.5f;
            Vector3 start = Car.Position + Car.ForwardVector * halfLen + new Vector3(0, 0, 0.3f);

            // Red when the speed-based steering limiter actually reduced the steer
            // this frame (or last, if the draw runs before ApplySteerLimits).
            ARS.DrawLine(start, start + DirAt(steerDeg) * 2f, _steerLimitedThisFrame ? Color.Red : Color.White);
        }

        void DrawSteerTargetLine()
        {
            if (Car == null || !Car.Exists()) return;
            if (!LookAheads.TryGetValue(LookAhead.SteerRef, out TrackPoint steerRefPoint) || steerRefPoint == null) return;

            Vector3 right = Vector3.Cross(steerRefPoint.Direction, Vector3.WorldUp).Normalized;
            Vector3 target = steerRefPoint.Position + right * _targetLane + new Vector3(0, 0, 0.3f);
            float halfLen = VehicleData.ModelDimensions.Y * 0.5f;
            Vector3 start = Car.Position + Car.ForwardVector * halfLen + new Vector3(0, 0, 0.3f);
            ARS.DrawLine(start, target, Color.Cyan);
        }




        float OutOfTrackDistance()
        {
            return (Math.Abs(Brain.CurrentPerception.DeviationFromCenter) + (VehicleData.BoundingBox / 2)) - CurrentTrackPoint.TrackHalfWidth;
        }

        void UpdateRivalInfo()
        {
            Brain.AvoidanceTarget = null;
            foreach (Rival r in Brain.Rivals)
            {
                r.Update(this);
                bool isAvoidanceCandidate = r.RelativePosition == RelativePos.Ahead
                    && (ARS.IsBetween(r.FrontGap, 0f, 3f)
                        || ARS.IsBetween(r.SecondsToHit, 0f, 5f))
                    && ARS.IsBetween(Math.Abs(r.DirectionDiff), 0f, 30f);
                if (Brain.AvoidanceTarget == null && isAvoidanceCandidate)
                {
                    Brain.AvoidanceTarget = r;
                }
            }
        }

        void ApplyRivalThrottleCap()
        {
            float nearestThrottleCap = 1f;
            float nearestSpeedLimit = float.PositiveInfinity;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || r.RelativePosition != RelativePos.Ahead) continue;

                if (ARS.IsBetween(r.SecondsToHit, 0f, 3f))
                    nearestThrottleCap = Math.Min(nearestThrottleCap, ARS.Remap(r.SecondsToHit, 0f, 3f, 0f, 1f, true));
                if (ARS.IsBetween(r.FrontGap, 0f, 1f))
                    nearestThrottleCap = Math.Min(nearestThrottleCap, ARS.Remap(r.FrontGap, 0f, 1f, 0f, 1f, true));

                if (ARS.IsBetween(r.FrontGap, 0f, 2f))
                {
                    float rivalSpeed = r.RivalRacer.Car.Velocity.Length();
                    float ourSpeed = Car.Velocity.Length();
                    float safeSpeedLimit = ARS.Remap(r.FrontGap, 0f, 2f, rivalSpeed, Math.Max(ourSpeed, rivalSpeed), true);
                    nearestSpeedLimit = Math.Min(nearestSpeedLimit, safeSpeedLimit);
                }
            }
            Control.MaxThrottle = Math.Min(Control.MaxThrottle, nearestThrottleCap);
            if (nearestSpeedLimit < float.PositiveInfinity)
                Brain.CurrentIntention.Speed = Math.Min(Brain.CurrentIntention.Speed, nearestSpeedLimit);
        }




        public void InitializeTrackPosition()
        {
            if (ARS.TrackPoints.Count == 0) return;

            TrackPoint closestPoint = ARS.TrackPoints[0];
            float closestDistance = closestPoint.Position.DistanceTo(Car.Position);
            foreach (TrackPoint point in ARS.TrackPoints)
            {
                float distance = point.Position.DistanceTo(Car.Position);
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

            TrackPoint closestPoint = _trackPositionScratch[0];
            float closestDistance = closestPoint.Position.DistanceTo(Car.Position);
            foreach (TrackPoint point in _trackPositionScratch)
            {
                float distance = point.Position.DistanceTo(Car.Position);
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
                    float distance = point.Position.DistanceTo(Car.Position);
                    if (distance < closestDistance)
                    {
                        closestPoint = point;
                        closestDistance = distance;
                    }
                }
            }

            CurrentTrackPoint = closestPoint;
            Brain.CurrentPerception.DeviationFromCenter = ARS.SignedLaneOffset(Car.Position, CurrentTrackPoint.Position, CurrentTrackPoint.Direction);

            LookAheads.Clear();
            float speed = Car.Velocity.Length();

            int steerRef = (int)ARS.Clamp((int)(speed / Math.Max(VehicleData.CurrentMechanicalGrip, 0.1f)), (int)(CurrentTrackPoint.TrackHalfWidth * 2f), 500);
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

            var lookAheadOffsets = new (LookAhead key, int offset)[]
            {
                (LookAhead.SteerRef, steerRef),
                (LookAhead.QuarterSec, quarterSec),
                (LookAhead.HalfSec, halfSec),
                (LookAhead.ThreeQuarterSec, threeQuarterSec),
                (LookAhead.OneSec, oneSec),
                (LookAhead.OneHalfSec, oneHalfSec),
                (LookAhead.TwoSec, twoSec),
            };
            foreach (var (key, offset) in lookAheadOffsets)
                LookAheads.Add(key, ResolveLookAhead(offset));




            int nodeCount = ARS.TrackPoints.Count;
            int currentNode = CurrentTrackPoint.Node;
            float currentPct = ARS.GetPercent(currentNode, nodeCount);
            float previousPct = _previousNode >= 0 ? ARS.GetPercent(_previousNode, nodeCount) : 0f;
            bool wrappedStartLine = !ARS.IsPointToPoint && _previousNode >= 0 && previousPct > 90f && currentPct < 10f;

            if (CanRegisterNewLap)
            {
                if (wrappedStartLine || (ARS.IsPointToPoint && currentPct > 99f && ARS.EntityRelativeOffset(Car, ARS.TrackPoints.Last().Position).Y < 0f))
                {
                    CanRegisterNewLap = false;
                    Lap++;
                    ARS.Log(ARS.LogImportance.Info, "Lap++ " + Name + " -> lap " + Lap + " (node " + currentNode + ")");
                    if (Lap > ARS.SettingsFile.GetValue("GENERAL_SETTINGS", "Laps", 5))
                    {
                        if (Car.CurrentBlip != null) Car.CurrentBlip.Color = BlipColor.Green;
                    }

                    if (Lap == 1 && !ARS.IsPointToPoint)
                    {
                        LapStartTime = Game.GameTime;
                    }
                    else if (Lap > 1)
                    {
                        TimeSpan lapTime = ARS.ParseToTimeSpan(Game.GameTime - LapStartTime);
                        UI.Notify(Name + "'s laptime: ~b~" + lapTime.ToString("m':'ss'.'f"));
                        LapTimes.Add(lapTime);
                        LapStartTime = Game.GameTime;
                    }
                }
            }
            else if (BaseBehavior == RacerBaseBehavior.Race && ARS.IsBetween(currentPct, 40f, 60f))
            {
                CanRegisterNewLap = true;
            }

            UpdateRaceProgress();

            _previousNode = currentNode;
            _lastApexProgressNode = currentNode;

            // Route radius from three sample points.
            Brain.CurrentPerception.CurveRadiusToFollowPoint = RouteRadiusSampled();
            UpdateApexLeapfrog();
            if (_apexUpdateTick + _phaseOffsetMs < Game.GameTime)
            {
                _apexUpdateTick = Game.GameTime + 500;
                RefillApexQueue();
            }
            // High-speed lane radius: short 0.5s to 1.0s window.
            Brain.CurrentPerception.HighSpeedCurveRadius = ComputeRouteRadius((int)(Car.Velocity.Length() * 0.5f), (int)(Car.Velocity.Length() * 1.0f));
            Brain.CurrentPerception.CurveRadiusAfterFollowPoint = ComputeRouteRadius((int)(Car.Velocity.Length() * 2.5f), (int)(Car.Velocity.Length() * 4.5f));
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
        const float ApexBufferSeconds = 2f;
        const float MinimumBrakeSampleSeconds = 0.25f;
        const float EntranceBrakeBufferSeconds = 0.25f;
        const float EntranceBrakeExtraDistance = 0f;
        const float SecondaryApexSpeedDifference = 5f;
        const float BrakingTargetFactor = 0.5f;
        const bool RouteSpeedEnabled = true;

        // Cheap: drop passed apexes and invalidate stale entries every tick.
        void UpdateApexLeapfrog()
        {
            int[] heldNodes = { NextApexNode, NextApexNode2, NextApexNode3, NextApexNode4 };
            float[] heldRadii = { NextApexRadius, NextApexRadius2, NextApexRadius3, NextApexRadius4 };

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
                if (shift > 0) CommitBrakeLearning();
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
        void RefillApexQueue()
        {
            int[] heldNodes = { NextApexNode, NextApexNode2, NextApexNode3, NextApexNode4 };
            float[] heldRadii = { NextApexRadius, NextApexRadius2, NextApexRadius3, NextApexRadius4 };

            int count = ARS.TrackPoints.Count;
            if (count < 10 || ARS.Corners.Count == 0)
            {
                CommitApexQueue(new[] { -1, -1, -1, -1 }, new[] { 999f, 999f, 999f, 999f });
                return;
            }

            List<int> selectedNodes = new List<int>();
            List<float> selectedRadii = new List<float>();
            for (int i = 0; i < heldNodes.Length && selectedNodes.Count < 4; i++)
            {
                if (heldNodes[i] < 0) break;
                selectedNodes.Add(heldNodes[i]);
                selectedRadii.Add(heldRadii[i]);
            }

            // Scan forward from the last held apex, leapfrogging each accepted target.
            List<int> upcoming = new List<int>();
            for (int i = 0; i < ARS.Corners.Count; i++)
            {
                int d = ForwardNodeDistance(ARS.Corners[i].Node);
                if (d > 0) upcoming.Add(i);
            }

            upcoming.Sort((left, right) => ForwardNodeDistance(ARS.Corners[left].Node).CompareTo(ForwardNodeDistance(ARS.Corners[right].Node)));
            for (int i = 0; i < upcoming.Count && selectedNodes.Count < 4; i++)
            {
                int candidate = upcoming[i];
                int distance = ForwardNodeDistance(ARS.Corners[candidate].Node);
                if (selectedNodes.Contains(ARS.Corners[candidate].Node)) continue;
                float candidateSpeed = RouteIdealSpeedForRadius(ARS.Corners[candidate].SupposedRadius);

                if (selectedNodes.Count > 0)
                {
                    int previousDistance = ForwardNodeDistance(selectedNodes[selectedNodes.Count - 1]);
                    float previousSpeed = RouteIdealSpeedForRadius(selectedRadii[selectedRadii.Count - 1]);
                    if (distance <= previousDistance) continue;

                    float closeCornerDistance = Math.Max(5f, previousSpeed * ApexBufferSeconds);
                    bool closeToPrevious = distance - previousDistance <= closeCornerDistance;
                    bool materiallySlower = previousSpeed - candidateSpeed >= SecondaryApexSpeedDifference;
                    if (closeToPrevious && !materiallySlower) continue;
                }

                selectedNodes.Add(ARS.Corners[candidate].Node);
                selectedRadii.Add(ARS.Corners[candidate].SupposedRadius);
            }

            CommitApexQueue(
                new[]
                {
                    selectedNodes.Count > 0 ? selectedNodes[0] : -1,
                    selectedNodes.Count > 1 ? selectedNodes[1] : -1,
                    selectedNodes.Count > 2 ? selectedNodes[2] : -1,
                    selectedNodes.Count > 3 ? selectedNodes[3] : -1
                },
                new[]
                {
                    selectedRadii.Count > 0 ? selectedRadii[0] : 999f,
                    selectedRadii.Count > 1 ? selectedRadii[1] : 999f,
                    selectedRadii.Count > 2 ? selectedRadii[2] : 999f,
                    selectedRadii.Count > 3 ? selectedRadii[3] : 999f
                });
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
            NextApexNode4 = nodes[3];
            NextApexRadius4 = radii[3];
            NextApexSpeed4 = NextApexNode4 >= 0 ? ApexSpeedWithDownforce(NextApexRadius4) : 999f;

            if (NextApexNode >= 0)
            {
                // Instance Brain.Corner from the nearest apex.
                CornerPoint cp = new CornerPoint();
                cp.Node = NextApexNode;
                cp.Angle = ARS.TrackPoints[NextApexNode].Angle;
                cp.SupposedRadius = NextApexRadius;
                cp.Speed = NextApexSpeed;
                Brain.Corner = new Corner(cp.Speed, cp);
            }
            else
            {
                Brain.Corner = null;
            }
        }

        bool HasPassedApex(int apexNode)
        {
            if (apexNode < 0 || _lastApexProgressNode < 0) return false;
            if (ARS.IsPointToPoint) return CurrentTrackPoint.Node >= apexNode;

            int count = ARS.TrackPoints.Count;
            int moved = CurrentTrackPoint.Node - _lastApexProgressNode;
            if (moved < 0) moved += count;
            int distanceToApex = apexNode - _lastApexProgressNode;
            if (distanceToApex < 0) distanceToApex += count;
            return distanceToApex <= moved;
        }

        // Kinematic braking map reaches apex speed at the corner entrance.
        int BrakingTargetNode(CornerPoint corner, float factor)
        {
            if (corner == null || corner.Node < 0) return -1;

            int entranceNode = corner.StartNode >= 0
                ? corner.StartNode
                : OffsetCornerNode(corner.Node, -corner.LengthStart);
            if (entranceNode < 0) return -1;

            factor = ARS.Clamp(factor, 0f, 1f);
            int distance = corner.Node - entranceNode;
            if (!ARS.IsPointToPoint && distance < 0) distance += ARS.TrackPoints.Count;
            if (ARS.IsPointToPoint && distance < 0) distance = 0;

            return OffsetCornerNode(entranceNode, (int)Math.Round(distance * factor));
        }

        float ApexBrakingSpeed(int apexNode, float apexSpeed)
        {
            if (apexNode < 0) return 999f;
            float velTarget = apexSpeed;

            CornerPoint corner = ARS.Corners.FirstOrDefault(c => c.Node == apexNode);
            int entranceNode = CornerEntranceNode(corner, apexNode);
            int targetNode = ActiveManeuver.Type == ManeuverType.DiveBomb
                ? BrakingTargetNode(corner, BrakingTargetFactor)
                : entranceNode;
            if (targetNode < 0) targetNode = entranceNode;

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

            float brakingAbility = Math.Min(Handling.BrakingAbility * 4, VehicleData.CurrentMechanicalGrip);
            float decel = brakingAbility * Handling.Gravity * EffectiveBrakeFactor(apexNode);
            if (ActiveManeuver.Type == ManeuverType.Yield) decel *= 0.5f;

            float spd = (float)Math.Sqrt(velTarget * velTarget + 2f * decel * distance);
            if (float.IsNaN(spd) || float.IsInfinity(spd)) spd = 999f;
            return spd;
        }

        // Dormant legacy route-probe state; the static apex table now supplies braking targets.
        const float RouteProbeSeconds = 5f;
        int _probeLastNode = -1;
        float _probeLastRadius = 999f;
        bool _probeShrinking = false;
        bool _probeInitialized = false;
        int _probeMinNode = -1;
        float _probeMinRadius = 999f;

        void ResetRouteProbe()
        {
            _probeLastNode = -1;
            _probeLastRadius = 999f;
            _probeShrinking = false;
            _probeInitialized = false;
            _probeMinNode = -1;
            _probeMinRadius = 999f;
        }

        void UpdateRouteTarget()
        {
            // Hold the locked target until its node is crossed.
            if (RouteTargetNode >= 0 && CurrentTrackPoint.Node <= RouteTargetNode)
                return;

            // Re-arm when the target is crossed.
            if (RouteTargetNode >= 0)
            {
                RouteTargetNode = -1;
                RouteTargetRadius = 999f;
                ResetRouteProbe();
            }

            float speed = Car.Velocity.Length();
            int count = ARS.TrackPoints.Count;
            if (speed < 1f || count < 10) return;

            int probeNode = CurrentTrackPoint.Node + (int)(speed * RouteProbeSeconds);
            if (ARS.IsPointToPoint)
                probeNode = (int)ARS.Clamp(probeNode, 0, count - 1);
            else
                probeNode = ((probeNode % count) + count) % count;

            float r = ARS.TrackPoints[probeNode].PreciseCurveRadius;
            if (float.IsNaN(r) || float.IsInfinity(r)) r = 999f;
            r = ARS.Clamp(r, 5f, 999f);

            // First read after arming: baseline.
            if (!_probeInitialized)
            {
                _probeInitialized = true;
                _probeLastNode = probeNode;
                _probeLastRadius = r;
                _probeMinNode = probeNode;
                _probeMinRadius = r;
                return;
            }

            // Only judge when the probe advanced (speed drops can pull it backwards).
            bool advanced = probeNode > _probeLastNode;
            if (advanced)
            {
                if (r < _probeLastRadius)
                {
                    _probeShrinking = true;
                    if (r < _probeMinRadius) { _probeMinRadius = r; _probeMinNode = probeNode; }
                }
                else if (_probeShrinking)
                {
                    // If next node is larger, the descent ended. Lock the minimum as the apex.
                    int nextNode = probeNode + 1;
                    if (ARS.IsPointToPoint)
                        nextNode = (int)ARS.Clamp(nextNode, 0, count - 1);
                    else
                        nextNode = ((nextNode % count) + count) % count;

                    float nextR = ARS.TrackPoints[nextNode].PreciseCurveRadius;
                    if (float.IsNaN(nextR) || float.IsInfinity(nextR)) nextR = 999f;
                    nextR = ARS.Clamp(nextR, 5f, 999f);

                    if (nextR > r)
                    {
                        RouteTargetNode = _probeMinNode;
                        RouteTargetRadius = _probeMinRadius;
                        return;
                    }
                }
            }

            _probeLastNode = probeNode;
            _probeLastRadius = r;
            if (!_probeShrinking)
            {
                _probeMinNode = probeNode;
                _probeMinRadius = r;
            }
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
            if (_halfSecondTick + _phaseOffsetMs < Game.GameTime)
            {
                _halfSecondTick = Game.GameTime + 500 + (int)ARS.Remap(Car.Velocity.Length(), 0, 100, -250, 250, true);
            }

            if (_oneSecondTick + _phaseOffsetMs < Game.GameTime)
            {
                _oneSecondTick = Game.GameTime + 1000;

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


                if (ARS.RacersMenuStore.GetInt("AIRacerAutofix", 1) == 2 && Function.Call<bool>(Hash._IS_VEHICLE_DAMAGED, Car))
                {
                    Car.Repair();
                }
            }
        }




        public void ProcessAI()
        {
            ProcessTimedAI();
            if (_pressureTick + _phaseOffsetMs < Game.GameTime)
            {
                _pressureTick = Game.GameTime + 500;
                UpdatePressure();
            }

            if (BaseBehavior == RacerBaseBehavior.GridWait && Control.HandBrakeTime < Game.GameTime) Control.HandBrakeTime = Game.GameTime + (100 * ARS.GetRandomInt(2, 6));

            if (!ControlledByPlayer)
            {
                if (_rivalInfoTick + _phaseOffsetMs < Game.GameTime)
                {
                    _rivalInfoTick = Game.GameTime + 500;
                    UpdateRivalInfo();
                }
                ApplyRivalThrottleCap();

                ComputeTargetSpeed();
                ComputeSteering();

                // Two-wheel stability: steer into the airborne side to regain all four wheels.
                // TEMPORARILY DISABLED while tuning the speed-based steering limiter.
                if (1 == 2)
                {
                    List<bool> wg = ARS.WheelsOnGround(Car);
                    if (wg.Count >= 4)
                    {
                        bool leftDown = wg[0] && wg[2];
                        bool rightDown = wg[1] && wg[3];
                        if (!leftDown && rightDown)
                            Control.SteerDegrees = -VehicleData.SteeringLock;
                        else if (!rightDown && leftDown)
                            Control.SteerDegrees = VehicleData.SteeringLock;
                    }
                }

                ApplySteerLimits();

                ConvertSpeedToPedals();
                TranslateSteerToInput();

                UpdateStuckCheck();
                UpdateStuckRecovery();
                
                TractionControl();
                ApplyStuckRecoveryOverride();

                UpdateNitrous();
                UpdateYield();

            }
            else
            {
                IsStuckByThrottle = false;
                _lastStuckGameTime = 0;
                _isRecoveringFromStuck = false;
                _stuckRecoveryEndTime = 0;
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
 
 
        void UpdateStuckCheck()
        {
            if (ARS.MpsToMph(Car.Velocity.Length()) > 10f && Math.Abs(Brain.CurrentPerception.DeviationFromCenter) < CurrentTrackPoint.TrackHalfWidth)
            {
                _stuckRecoveryAttempts = 0;
            }

            if (_isRecoveringFromStuck)
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
                _lastStuckGameTime = Game.GameTime;
            }

            bool stuckForLongEnough = (Game.GameTime - _lastStuckGameTime) >= StuckCheckTimeMs;
            IsStuckByThrottle = stuckForLongEnough;

            if (stuckForLongEnough && !_isRecoveringFromStuck)
            {
                _isRecoveringFromStuck = true;
                _stuckRecoveryAttempts++;
                _stuckRecoveryEndTime = Game.GameTime + StuckRecoveryTimeMs;
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

            if (Game.GameTime >= _stuckRecoveryEndTime)
            {
                _isRecoveringFromStuck = false;
                _stuckRecoveryEndTime = 0;
                _lastStuckGameTime = 0;
                return;
            }
        }

        void ApplyStuckRecoveryOverride()
        {
            if (!_isRecoveringFromStuck) return;

            if (Game.GameTime >= _stuckRecoveryEndTime)
            {
                _isRecoveringFromStuck = false;
                _stuckRecoveryEndTime = 0;
                _lastStuckGameTime = 0;
                return;
            }


            Control.SteerInput = 0f;
            Control.Throttle = -0.33f;
            Control.Brake = 0f;
        }

        void UpdatePerceivedGrip()
        {


            float handlingGrip = Function.Call<float>((Hash)0xA132FB5370554DB0, Car);
            handlingGrip = ARS.Clamp(handlingGrip, 0.1f, 100f);
            handlingGrip /= 1f + 0.035f * Handling.Downforce;

            // TEMP experiment: bake the gravity multiplier into base grip when the off-road
            // flag pushes gravity in Gs above 1 (normally 1.2).
            float gravityGs = Handling.Gravity / 9.8f;
            if (gravityGs > 1f) handlingGrip *= gravityGs;

            GroundGripMultiplier = ARS.WheelGripMultipliers(Car).Average();

            // Centripetal acceleration v²/r is the proxy for cornering load (engine applies
            // downforce scaled by lateral speed, but pure-pursuit driving keeps world-frame lateral
            // velocity near zero — centripetal accel captures the same load physically).
            float forwardMs = ARS.GetForwardSpeed(Car);
            float routeRadius = Brain.CurrentPerception.CurveRadiusToFollowPoint;
            float lateralMs = 0f;
            if (routeRadius > 1f && !float.IsNaN(routeRadius) && !float.IsInfinity(routeRadius))
                lateralMs = (forwardMs * forwardMs) / routeRadius;
            float dfGs = ARS.GetDownforceGsAtSpeed(this, forwardMs, lateralMs);

            VehicleData.BaseMechanicalGrip = handlingGrip;
            VehicleData.DownforceGripBonus = dfGs;
            // AvgGroundStability is currently hardcoded to 1f: the old wheels-off-ground detector
            // (WheelSlips ~0) was unreliable and triggered on decompression, so it has been removed
            // until a trustworthy replacement is found.
            VehicleData.AvgGroundStability = 1f;
            VehicleData.CurrentMechanicalGrip = (VehicleData.BaseMechanicalGrip + VehicleData.DownforceGripBonus) * GroundGripMultiplier * VehicleData.AvgGroundStability;

            // Airborne vehicles temporarily lose available throttle; normal pedal processing restores it.
            if (Game.GameTime - _lastStabilityCheck >= 333) // ~3 Hz
            {
                _lastStabilityCheck = Game.GameTime;
                List<bool> wheelsOnGround = ARS.WheelsOnGround(Car);
                bool allDown = wheelsOnGround.Count > 0 && wheelsOnGround.All(w => w);
                if (!allDown)
                    Control.MaxThrottle = Math.Max(Control.MaxThrottle - 0.5f * TickScale, 0.1f);
            }


            if (Math.Abs(Brain.CurrentPerception.DeviationFromCenter) < CurrentTrackPoint.TrackHalfWidth && RacePosition <= 2 && !ARS.TerrainGripMultipliers.ContainsKey(CurrentTrackPoint.Node))
            {
                ARS.TerrainGripMultipliers.Add(CurrentTrackPoint.Node, GroundGripMultiplier);
            }

            VehicleData.YawRotationPerSecondDegrees = ARS.RadToDeg(Function.Call<Vector3>(Hash.GET_ENTITY_ROTATION_VELOCITY, Car).Z);
        }
        public void UpdateRivals()
        {
            List<Racer> candidates = new List<Racer>();
            foreach (Racer r in ARS.Racers)
            {
                if (r.Car.Handle != Car.Handle && r.Car.Position.DistanceTo(Car.Position) < 200f)
                {
                    candidates.Add(r);
                }
            }

            foreach (Rival r in Brain.Rivals) r.RivalRacer = null;
            if (candidates.Count > 0)
            {
                Vector3 hoodPos = Car.Position + Car.ForwardVector;
                candidates.Sort((a, b) => Vector3.Distance(a.Car.Position, hoodPos).CompareTo(Vector3.Distance(b.Car.Position, hoodPos)));
                for (int i = 0; i < Brain.Rivals.Count; i++)
                {
                    if (i == candidates.Count) break;
                    Brain.Rivals[i].RivalRacer = candidates[i];
                }
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


