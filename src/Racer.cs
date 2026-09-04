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

        // Time-based route references used by steering and speed calculations.
        public enum LookAhead { SteerRef, QuarterSec, HalfSec, ThreeQuarterSec, OneSec, OneHalfSec, TwoSec };
        public Dictionary<LookAhead, TrackPoint> LookAheads = new Dictionary<LookAhead, TrackPoint>();


        public List<TimeSpan> LapTimes = new List<TimeSpan>();
        public int LapStartTime = 0;
        public int Lap = 0;
        public int RacePosition = 0;
        public bool CanRegisterNewLap = true;
        bool _hasLeftLapArmNode = false;
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
        const int StuckCheckTimeMs = 2000;
        bool _isRecoveringFromStuck = false;
        int _stuckRecoveryEndTime = 0;
        const int StuckRecoveryTimeMs = 2000;
        int _stuckRecoveryAttempts = 0;
        public int StuckRecoveryAttemptsNow => _stuckRecoveryAttempts;
        public bool IsRecoveringFromStuckNow => _isRecoveringFromStuck;

        // State for smooth recovery repositioning toward the track.
        bool _isLerpingToTrack = false;
        Vector3 _lerpStartPos = Vector3.Zero;
        Vector3 _lerpTargetPos = Vector3.Zero;
        int _lerpStartTime = 0;
        const int LerpToTrackMs = 1500;

        // Grip checker experiment: track peak lateral Gs through a corner.
        bool _gripCheckArmed = false;
        float _gripCheckPeakGs = 0f;
        int _gripCheckApexNode = -1;
        int _gripCheckLastApexNode = -1;
        Dictionary<int, float> _cornerSpeedOffsets = new Dictionary<int, float>();
        float _tempSpeedUp = 0f;
        bool _gripCheckLifted = false;


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
        bool _avoidWallsInitialized = false;
        float _targetLane = 0f;
        float _rawCornerLane = 0f;
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

        float InputForOffshoot = 1f;
        const float OffshootRangeMeters = 2f;
        const float FullPedalSpeedErrorMps = 6.47f;
        static readonly float StationarySpeedThresholdMps = ARS.MphToMps(5f);
        float _speedCap = 999f;
        const float SpeedCapRiseRate = 30f;

        // Empirical steer-limit memory: peak lateral G reference and the steer that produced it.
        float _latGRef = 0f;
        float _peakSteerDeg = 15f;
        bool _nearGripPeak = false;

        // True when the speed-based steering limiter actually reduced the steer this frame.
        bool _steerLimitedThisFrame = false;
        float _steerLimitDegrees = 999f;
        float _requestedSteerDegrees = 0f;

        // Brake learning (Phase 1): learn the effective decel factor that keeps the car
        // at full brake ~75% of each braking phase. Toggle off → factor pinned at 1.
        float _brakeDecelFactor = 1f;
        int _brakePhaseFrames = 0;
        int _brakePhaseFullFrames = 0;
        bool _inBrakePhase = false;
        const float FullBrakeThreshold = 0.9f;   // brake input at/above this counts as "full"
        const float BrakeFractionTarget = 0.75f; // target share of the phase at full brake
        const float BrakeEaseOffRate = 0.98f;    // reduce factor when fraction too high
        const float BrakeLeanOnRate = 1.02f;     // increase factor when fraction too low
        const float MinBrakeDecelFactor = 0.3f;
        const float MaxBrakeDecelFactor = 1.5f;
        // Read by ARS.MaxSpeedForBrakingDistance (static) to scale its decel plan.
        public float BrakeDecelFactor => _brakeDecelFactor;



        bool _isPassengerized = false;

        // Feature gate for AI nitrous.
        const bool AiNitrousEnabled = false;
        const float NitrousPowerMultiplier = 2.5f;
        const float NitrousCornerLookaheadSeconds = 8f;
        const int NitrousDurationMs = 3000;
        const string NitrousPtfxAsset = "veh_xs_vehicle_mods";
        const ulong CheatPowerIncreaseHash = 0xB59E4BD37AE292DB;
        const ulong FullyChargeNitrousHash = 0x1A2BCC8C636F9226;
        const ulong OverrideNitrousLevelHash = 0xC8E9B6B71B8E660D;

        public Maneuver ActiveManeuver = new Maneuver();
        int _nitrousActiveUntil = 0;


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

                if (ARS.RaceSettingsFile.GetValue<int>("RACERS", "AIRacerAutofix", 1) == 2)
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
                else if (ARS.RaceSettingsFile.GetValue<int>("RACERS", "AIRacerAutofix", 1) == 1)
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
            if (Handling.Downforce > 100) Handling.Downforce *= 0.01f;

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
            CanRegisterNewLap = true;
            _hasLeftLapArmNode = false;

            string flags = ARS.GetHandlingFlags(Car).ToString("X");
            int flagsHex = Convert.ToInt32(flags, 16);
            bool hasOffroad = (flagsHex & 0x800000) != 0 || (flagsHex & 0x200000) != 0;
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


            const float steerKP = 1.0f;
            // TEMP: hardcoded 0.66 — testing yaw damping.
            const float steerKD = 0.66f;
            Control.SteerDegrees = (steerKP * (headingErrorDeg + laneBiasDeg + recoveryDeg)) - (steerKD * VehicleData.YawRotationPerSecondDegrees);
            bool sameSignSlideYaw = Math.Sign((int)VehicleData.SlideAngle) == Math.Sign((int)VehicleData.YawRotationPerSecondDegrees);
            if (sameSignSlideYaw)
            {
                float slideScale = ARS.Remap(Math.Abs(VehicleData.SlideAngle), 0f, Handling.LateralTractionCurve * 1.2f, 0.5f, 1.2f, true);
                Control.SteerDegrees -= VehicleData.SlideAngle * slideScale;
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

        // Lane Control System 2: positions the car on the inside edge of the track curvature.
        float ComputeHighSpeedLane(float roadWide, float speedMps)
        {
            if (Brain.CurrentPerception.HighSpeedCurveRadius > 500f) return 0f;

            int count = ARS.TrackPoints.Count;
            int fwdNode;
            int fwdOffset = Math.Max((int)(speedMps * 1.0f), 5);
            if (ARS.IsPointToPoint)
                fwdNode = (int)ARS.Clamp(CurrentTrackPoint.Node + fwdOffset, 0, count - 1);
            else
                fwdNode = ((CurrentTrackPoint.Node + fwdOffset) % count + count) % count;

            Vector3 currentDir = CurrentTrackPoint.Direction;
            Vector3 futureDir = ARS.TrackPoints[fwdNode].Direction;

            float signedAngle = Vector3.SignedAngle(currentDir, futureDir, Vector3.WorldUp);
            if (float.IsNaN(signedAngle) || float.IsInfinity(signedAngle)) return 0f;

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
            const float approachStartTime = 5.0f;

            if (apexNode != _approachCornerNode || timeToApex > approachStartTime)
            {
                _approachCornerNode = apexNode;
                _approachOutsideDecided = false;
                _approachHoldsOutside = false;
                if (timeToApex > approachStartTime) return 0f;
            }

            bool shouldHoldOutside = true; // TEMP: always hold outside for testing
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
                if (!float.IsNaN(entranceHeading) && !float.IsInfinity(entranceHeading) && Math.Abs(entranceHeading) > 45f)
                    return 0f;
            }

            float cornerDir = Math.Sign(c.Angle);
            if (cornerDir == 0f) return 0f;

            float halfWidth = steerRefPoint.TrackHalfWidth;
            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            float safeBound = halfWidth - carHalfWidth;

            float releaseSeconds = steerRefPoint.TrackHalfWidth * 0.2f;
            if (_approachHoldsOutside && timeToApex > releaseSeconds)
            {
                // Corner-commit: sit beside the rival on the corner inside instead of the outside line.
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
                if (!ARS.IsBetween(r.SecondsToHit, 0f, 5f)) continue;
                if (!ARS.IsBetween(r.FrontGap, 0f, 5f)) continue;
                if (!ARS.IsBetween(r.DirectionDiff, -20f, 20f)) continue;

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
            // Max steer angle = 2 + max(slide angle, TRlat × 0.2), so low-slide cars keep a grip-based minimum allowance.
            float maxSteerAngle = 2f + Math.Max(slideAngle, Handling.LateralTractionCurve * 0.2f);
            float maxSteer = ARS.Remap(fwdMph, 60f, 0f, maxSteerAngle, VehicleData.SteeringLock, true);
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
            LapStartTime = Game.GameTime;
            Lap = 1;
            CanRegisterNewLap = false;
            _hasLeftLapArmNode = false;
            Control.HandBrakeTime = Game.GameTime + ARS.GetRandomInt(100, 400);
            Control.MaxThrottle = 1f;
            IsStuckByThrottle = false;
            _lastStuckGameTime = 0;
            _isRecoveringFromStuck = false;
            _stuckRecoveryEndTime = 0;
            _stuckRecoveryAttempts = 0;
            _isLerpingToTrack = false;
            _lerpStartTime = 0;
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
            bool wantsReverse = Brain.CurrentIntention.Speed < -0.1f;

            float combinedInput = ComputeCombinedInput(intendedSpeedChange, currentForwardSpeed, wantsReverse);
            combinedInput = ApplyThrottleCap(combinedInput, wantsReverse, currentForwardSpeed);
            SplitCombinedInput(combinedInput, wantsReverse, currentForwardSpeed, ref newThrottle, ref newBrake);

            Control.Brake += ARS.Clamp(newBrake - Control.Brake, -inputChange, inputChange);
            Control.Throttle += ARS.Clamp(newThrottle - Control.Throttle, -inputChange, inputChange);

            UpdateBrakeLearning();
            Control.Throttle = Math.Min(Control.Throttle, Control.MaxThrottle);
            if (Control.MaxThrottle < 1.00f) Control.MaxThrottle += 2 * TickScale;

            if (Brain.CurrentIntention.MaxSpeed < AiConstants.MaxSpeed) Brain.CurrentIntention.MaxSpeed += 15 * TickScale;

        }

        float ComputeCombinedInput(float intendedSpeedChange, float currentForwardSpeed, bool wantsReverse)
        {
            if (intendedSpeedChange > 0f && ShouldBrakeBeforeDrivingForward(currentForwardSpeed))
                return -ARS.Clamp(intendedSpeedChange / FullPedalSpeedErrorMps, 0f, 1f);

            if (intendedSpeedChange < 0f && wantsReverse && ShouldBrakeBeforeReversing(currentForwardSpeed))
                return -ARS.Clamp((-intendedSpeedChange) / FullPedalSpeedErrorMps, 0f, 1f);

            return ARS.Clamp(intendedSpeedChange / FullPedalSpeedErrorMps, -1f, 1f);
        }

        float ApplyThrottleCap(float combinedInput, bool wantsReverse, float currentForwardSpeed)
        {
            float throttleCap = Math.Min(Control.MaxThrottleFromTCS, 1f);
            bool isReverseThrottle = wantsReverse && !ShouldBrakeBeforeReversing(currentForwardSpeed);

            if (combinedInput > 0f) return Math.Min(combinedInput, throttleCap);
            if (combinedInput < 0f && isReverseThrottle) return -Math.Min(-combinedInput, throttleCap);
            return combinedInput;
        }

        void SplitCombinedInput(float combinedInput, bool wantsReverse, float currentForwardSpeed, ref float newThrottle, ref float newBrake)
        {
            if (combinedInput > 0f)
            {
                newThrottle = combinedInput;
                return;
            }

            if (combinedInput < 0f)
            {
                bool isReverseThrottle = wantsReverse && !ShouldBrakeBeforeReversing(currentForwardSpeed);
                if (isReverseThrottle) newThrottle = combinedInput;
                else newBrake = -combinedInput;
            }
        }

        bool ShouldBrakeBeforeDrivingForward(float speed) => speed < -StationarySpeedThresholdMps;

        bool ShouldBrakeBeforeReversing(float speed) => speed > StationarySpeedThresholdMps;

        void FindLowestIntendedSpeed()
        {
            if (Brain.CurrentIntention.Speed >= 0f)
            {
                Brain.CurrentIntention.Speed = Math.Min(Brain.CurrentIntention.Speed, ARS.EngineTopSpeed(Car) * 1.3f);
                Brain.CurrentIntention.Speed = Math.Min(Brain.CurrentIntention.Speed, Brain.CurrentIntention.MaxSpeed);
                Brain.CurrentIntention.Speed = Math.Min(Brain.CurrentIntention.Speed, _speedCap);
                Brain.CurrentIntention.Speed = Math.Min(Brain.CurrentIntention.Speed, ComputeOffshootSpeedCap());
            }
        }

        float ComputeOffshootSpeedCap()
        {
            Vector3 proj = ProjectAhead(1f);
            TrackPoint tp = ARS.FindNearestTrackPoint(proj, CurrentTrackPoint.Node);
            float signedOffset = ARS.SignedLaneOffset(proj, tp.Position, tp.Direction);
            float safeBound = tp.TrackHalfWidth - VehicleData.BoundingBox * 0.5f;
            float offTrackDistance = Math.Abs(signedOffset) - safeBound;
            bool isOutsideCorner = tp.PreciseCurveRadius < 400f && Math.Sign(signedOffset) == Math.Sign(CurrentTrackPoint.Angle);

            if (offTrackDistance <= 0f || !isOutsideCorner)
            {
                InputForOffshoot = 1f;
                return 999f;
            }

            _gripCheckLifted = true;
            InputForOffshoot = ARS.Remap(offTrackDistance, OffshootRangeMeters, -OffshootRangeMeters, -1f, 1f, true);
            float floorSpeed = 5f * VehicleData.CurrentMechanicalGrip;
            return ARS.Remap(InputForOffshoot, -1f, 1f, floorSpeed, 999f, true);
        }

        void UpdateBrakeLearning()
        {
            if (!ARS.DebugToggles[Options.BrakeLearning])
            {
                _inBrakePhase = false;
                _brakeDecelFactor = 1f;
                return;
            }

            if (ActiveManeuver.Type != ManeuverType.None)
            {
                _inBrakePhase = false;
                return;
            }

            if (Control.Brake > 0f && !_inBrakePhase)
            {
                _inBrakePhase = true;
                _brakePhaseFrames = 0;
                _brakePhaseFullFrames = 0;
            }
            if (_inBrakePhase)
            {
                _brakePhaseFrames++;
                if (Control.Brake >= FullBrakeThreshold) _brakePhaseFullFrames++;
            }
            if (Control.Brake <= 0f && _inBrakePhase)
            {
                _inBrakePhase = false;
                float fraction = _brakePhaseFrames > 0 ? (float)_brakePhaseFullFrames / _brakePhaseFrames : 0f;
                if (fraction > BrakeFractionTarget) _brakeDecelFactor *= BrakeEaseOffRate;
                else if (fraction < BrakeFractionTarget) _brakeDecelFactor *= BrakeLeanOnRate;
                _brakeDecelFactor = ARS.Clamp(_brakeDecelFactor, MinBrakeDecelFactor, MaxBrakeDecelFactor);
            }
        }

        float TickScale => (0.001f * TimeSince_lastCoreTick);




        void TranslateSteerToInput()
        {

            if (float.IsNaN(Control.SteerDegrees) || float.IsInfinity(Control.SteerDegrees)) Control.SteerDegrees = 0f;

            // Fixed slew-rate limiter: the applied steer moves toward the target at a
            // fixed rate (45°/s), doubled to 90°/s when countersteering (steer opposes yaw).
            float error = Control.SteerDegrees - Control.LastAppliedSteerDegrees;
            bool countersteering = Math.Sign(Control.SteerDegrees) != Math.Sign((int)VehicleData.YawRotationPerSecondDegrees);
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

            // Once the car has turned in, let route speed govern the corner instead of the apex braking plan.
            if (NextApexNode >= 0)
            {
                float timeToApex = ForwardNodeDistance(NextApexNode) / Math.Max(Car.Velocity.Length(), 1f);
                if (timeToApex <= 0.33f) cornerSpd = 999f;
            }

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


            // Temporary: reduce both speed targets by 3 m/s to combat consistent overshooting.
            cornerSpd -= 3f;
            followTrackSpd -= 3f;
            followTrackSpd += _tempSpeedUp;
            _debugCornerSpd = cornerSpd;
            _debugFollowTrackSpd = followTrackSpd;
            Brain.CurrentIntention.Speed = Math.Min(cornerSpd, followTrackSpd);

            // Yield: cap throttle to 0.5 to stay behind.
            if (ActiveManeuver.Type == ManeuverType.Yield && ActiveManeuver.Target != null)
            {
                Control.MaxThrottle = Math.Min(Control.MaxThrottle, 0.5f);
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
                IdealWheelspin = -1f - Math.Abs(VehicleData.SlideAngle) / 10f;
                IdealWheelspin = ARS.Clamp(IdealWheelspin, -3f, 0f);  // magnitude capped at 3
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

            // DefendLane cleanup: off once we pass the defended apex or the target overtakes us.
            if (ActiveManeuver.Type == ManeuverType.DefendLane)
            {
                bool overtaken = ActiveManeuver.Target == null
                    || !ActiveManeuver.Target.Car.Exists()
                    || !Brain.Rivals.Any(r => r.RivalRacer == ActiveManeuver.Target && r.RelativePosition == RelativePos.Behind);

                int passed = CurrentTrackPoint.Node - _defendApexNode;
                if (!ARS.IsPointToPoint && passed < 0) passed += ARS.TrackPoints.Count;

                if (overtaken || (_defendApexNode >= 0 && passed >= 0))
                {
                    ActiveManeuver.Type = ManeuverType.None;
                    ActiveManeuver.Target = null;
                    _defendApexNode = -1;
                }
            }

            // Nitrous: arm if nearby cars exist or position >= 3rd, and not recently used
            if (AiNitrousEnabled && ActiveManeuver.Type == ManeuverType.None && Game.GameTime - ActiveManeuver.LastEnabled > 20000)
            {
                bool hasNearbyCars = Brain.Rivals.Any(r => r.RivalRacer != null);
                if (hasNearbyCars || RacePosition >= 3)
                {
                    ActiveManeuver.Type = ManeuverType.Nitrous;
                    ActiveManeuver.LastEnabled = Game.GameTime;
                }
            }

            // Yield: arm if pressure is much lower than closest rival, in overlap, at the corner entrance.
            if (ActiveManeuver.Type == ManeuverType.None && Brain.Corner != null)
            {
                Rival closestRival = Brain.Rivals
                    .Where(r => r.RivalRacer != null)
                    .OrderBy(r => r.Distance)
                    .FirstOrDefault();
                if (closestRival != null)
                {
                    float pressureDiff = closestRival.RivalRacer.Pressure - Pressure;
                    bool inOverlap = closestRival.RelativePosition == RelativePos.Left || closestRival.RelativePosition == RelativePos.Right;
                    int entranceNode = Brain.Corner.Point.StartNode >= 0
                        ? Brain.Corner.Point.StartNode
                        : OffsetCornerNode(Brain.Corner.Point.Node, -Brain.Corner.Point.LengthStart);
                    float timeToEntrance = ForwardNodeDistance(entranceNode)
                        / Math.Max(Car.Velocity.Length(), 1f);

                    if (pressureDiff > 30f && inOverlap && ARS.IsBetween(timeToEntrance, 0.5f, 2f)
                        && closestRival.Distance <= 20f
                        && closestRival.RivalRacer.Car.Velocity.Length() > Car.Velocity.Length())
                    {
                        ActiveManeuver.Type = ManeuverType.Yield;
                        ActiveManeuver.Target = closestRival.RivalRacer;
                        ActiveManeuver.LastEnabled = Game.GameTime;
                        UI.Notify("~y~" + Name + "~w~ yields to ~y~" + closestRival.RivalRacer.Name);
                    }
                }
            }

            // Divebomb: commit to a rival's inside to out-brake them at the apex.
            if (ActiveManeuver.Type == ManeuverType.None && Brain.Corner != null)
            {
                int apexNode = Brain.Corner.Point.Node;
                int entranceNode = Brain.Corner.Point.StartNode >= 0
                    ? Brain.Corner.Point.StartNode
                    : OffsetCornerNode(apexNode, -Brain.Corner.Point.LengthStart);
                float myTimeToEntrance = ForwardNodeDistance(entranceNode)
                    / Math.Max(Car.Velocity.Length(), 1f);

                Rival diveTarget = Brain.Rivals
                    .Where(r =>
                        r.RivalRacer != null
                        && r.RelativePosition == RelativePos.Ahead
                        && r.RivalRacer.ActiveManeuver.Type != ManeuverType.DiveBomb
                        && r.RivalRacer.ActiveManeuver.Type != ManeuverType.DefendLane
                        && r.Distance <= 40f
                        && Math.Abs(r.LateralGap) <= 6f
                        && r.ForwardSpeedGap > 0f
                        && Math.Abs(r.RivalRacer.ForwardNodeDistance(entranceNode)
                            / Math.Max(r.RivalRacer.Car.Velocity.Length(), 1f) - myTimeToEntrance) <= 1f)
                    .OrderBy(r => r.Distance)
                    .FirstOrDefault();

                if (diveTarget != null && Pressure > 30f
                    && ARS.IsBetween(myTimeToEntrance, 1f, 3f))
                {
                    ActiveManeuver.Type = ManeuverType.DiveBomb;
                    ActiveManeuver.Target = diveTarget.RivalRacer;
                    ActiveManeuver.LastEnabled = Game.GameTime;
                    _divebombApexNode = apexNode;
                    UI.Notify("~y~" + Name + "~w~ divebombs ~y~" + diveTarget.RivalRacer.Name);
                }
            }

            // DefendLane: cover the inside so a rival behind can't dive underneath.
            if (ActiveManeuver.Type == ManeuverType.None && Brain.Corner != null)
            {
                int apexNode = Brain.Corner.Point.Node;
                int entranceNode = Brain.Corner.Point.StartNode >= 0
                    ? Brain.Corner.Point.StartNode
                    : OffsetCornerNode(apexNode, -Brain.Corner.Point.LengthStart);
                float myTimeToEntrance = ForwardNodeDistance(entranceNode)
                    / Math.Max(Car.Velocity.Length(), 1f);

                Rival defenderTarget = Brain.Rivals
                    .Where(r =>
                        r.RivalRacer != null
                        && r.RelativePosition == RelativePos.Behind
                        && r.RivalRacer.ActiveManeuver.Type != ManeuverType.DiveBomb
                        && r.RivalRacer.ActiveManeuver.Type != ManeuverType.DefendLane
                        && r.Distance <= 30f
                        && Math.Abs(r.LateralGap) <= 6f
                        && r.ForwardSpeedGap < 0f
                        && r.RivalRacer.ForwardNodeDistance(entranceNode)
                            / Math.Max(r.RivalRacer.Car.Velocity.Length(), 1f) <= myTimeToEntrance)
                    .OrderBy(r => r.Distance)
                    .FirstOrDefault();

                if (defenderTarget != null && ARS.IsBetween(myTimeToEntrance, 1f, 3f))
                {
                    ActiveManeuver.Type = ManeuverType.DefendLane;
                    ActiveManeuver.Target = defenderTarget.RivalRacer;
                    ActiveManeuver.LastEnabled = Game.GameTime;
                    _defendApexNode = apexNode;
                    UI.Notify("~y~" + Name + "~w~ defends the inside from ~y~" + defenderTarget.RivalRacer.Name);
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

        int BehindNodeDistance(int targetNode)
        {
            int behind = CurrentTrackPoint.Node - targetNode;
            if (!ARS.IsPointToPoint && behind < 0) behind += ARS.TrackPoints.Count;
            return behind;
        }

        void UpdateNitrous()
        {
            if (ControlledByPlayer || !AiNitrousEnabled) return;

            // Apply power boost while nitrous is active.
            if (Game.GameTime < _nitrousActiveUntil)
            {
                Function.Call((Hash)CheatPowerIncreaseHash, Car, NitrousPowerMultiplier);
                return;
            }
            if (_nitrousActiveUntil > 0)
            {
                StopNitrous();
                ActiveManeuver.Type = ManeuverType.None;
            }

            if (ActiveManeuver.Type != ManeuverType.Nitrous) return;

            // Stability: steering < 5 degrees, rotation < 30 degrees per second.
            if (Math.Abs(Control.SteerDegrees) >= 5f) return;
            if (Math.Abs(VehicleData.YawRotationPerSecondDegrees) >= 30f) return;

            // Corner check: more than 8s away.
            if (Brain.Corner != null)
            {
                int entranceNode = Brain.Corner.Point.StartNode >= 0
                    ? Brain.Corner.Point.StartNode
                    : OffsetCornerNode(Brain.Corner.Point.Node, -Brain.Corner.Point.LengthStart);
                float distanceToEntrance = ForwardNodeDistance(entranceNode);
                float timeToEntrance = distanceToEntrance * 2f / Math.Max(Car.Velocity.Length(), 1f);
                if (timeToEntrance <= NitrousCornerLookaheadSeconds) return;
            }

            // Target check: closest rival must be faster.
            Rival closestRival = Brain.Rivals
                .Where(r => r.RivalRacer != null)
                .OrderBy(r => r.Distance)
                .FirstOrDefault();
            if (closestRival == null) return;
            if (closestRival.RivalRacer.Car.Velocity.Length() <= Car.Velocity.Length()) return;

            StartNitrous();
            ActiveManeuver.Type = ManeuverType.None;
            ActiveManeuver.LastEnabled = Game.GameTime;
        }

        void StartNitrous()
        {
            Function.Call(Hash.REQUEST_NAMED_PTFX_ASSET, NitrousPtfxAsset);
            Function.Call((Hash)FullyChargeNitrousHash, Car);
            Function.Call((Hash)OverrideNitrousLevelHash, Car, true, 1.0f, 50.0f, 100.0f, false);
            _nitrousActiveUntil = Game.GameTime + NitrousDurationMs;
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

            RunGripChecker();
        }

        void RunGripChecker()
        {
            if (ControlledByPlayer) return;

            if (_tempSpeedUp > 0f) _tempSpeedUp = Math.Max(0f, _tempSpeedUp - 1f * Game.LastFrameTime);
            else if (_tempSpeedUp < 0f) _tempSpeedUp = Math.Min(0f, _tempSpeedUp + 1f * Game.LastFrameTime);

            float latGs = Math.Abs(VehicleData.GetLateralGs(Car.ForwardVector));

            if (!_gripCheckArmed)
            {
                if (NextApexNode < 0) return;
                float timeToApex = ForwardNodeDistance(NextApexNode) / Math.Max(Car.Velocity.Length(), 1f);
                if (timeToApex > 2f) { _gripCheckLastApexNode = -1; return; }
                if (NextApexNode == _gripCheckLastApexNode) return;
                _gripCheckArmed = true;
                _gripCheckApexNode = NextApexNode;
                _gripCheckPeakGs = latGs;
                _tempSpeedUp = GetCornerSpeedOffset(NextApexNode);
                _gripCheckLifted = false;
                return;
            }

            _gripCheckPeakGs = Math.Max(_gripCheckPeakGs, latGs);

            if (Control.Throttle > 0.9f && latGs < 0.2f)
            {
                float timeToArmedApex = ForwardNodeDistance(_gripCheckApexNode) / Math.Max(Car.Velocity.Length(), 1f);
                if (timeToArmedApex <= 2f) return;
                if (_gripCheckLifted)
                {
                    _gripCheckLastApexNode = _gripCheckApexNode;
                    _gripCheckArmed = false;
                    _gripCheckPeakGs = 0f;
                    return;
                }
                float offset = GetCornerSpeedOffset(_gripCheckApexNode);
                if (_gripCheckPeakGs > 5f) offset -= 5f;
                else if (_gripCheckPeakGs / Math.Max(VehicleData.CurrentMechanicalGrip, 0.1f) < 0.95f) offset += 2f;
                _cornerSpeedOffsets[_gripCheckApexNode] = offset;

                UI.Notify(Name + " did " + _gripCheckPeakGs.ToString("0.0") + "/" + VehicleData.CurrentMechanicalGrip.ToString("0.0") + " on apex " + _gripCheckApexNode + " (+" + offset.ToString("0") + "m/s)");
                _gripCheckLastApexNode = _gripCheckApexNode;
                _gripCheckArmed = false;
                _gripCheckPeakGs = 0f;
            }
        }

        float GetCornerSpeedOffset(int apexNode)
        {
            return _cornerSpeedOffsets.TryGetValue(apexNode, out float o) ? o : 0f;
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
            if (!requestedInputs && !requestedTrack) return;

            // During focused debugging, use the closest AI racer so input data is available.
            Racer closestAiToPlayer = ARS.Racers
                .Where(r => r != null && r.Driver != null && !r.Driver.IsPlayer && r.Car != null && r.Car.Exists())
                .OrderBy(r => r.Car.Position.DistanceTo(Game.Player.Character.Position))
                .FirstOrDefault();
            if (closestAiToPlayer != this) return;

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

            // Legacy debug visuals remain below for later reintroduction, but are not called for now.
            return;

            bool showAggro = ARS.DebugToggles[Options.ShowAggro];
            bool showInputs = ARS.DebugToggles[Options.ShowInputs];
            bool showTrack = ARS.DebugToggles[Options.ShowTrackAnalysis];
            bool showPhysics = ARS.DebugToggles[Options.ShowPhysics];
            bool showAny = showAggro || showInputs || showTrack || showPhysics;
            if (!showAny) return;

            Racer closestToPlayer = ARS.Racers
                .Where(r => r != null && r.Car != null && r.Car.Exists())
                .OrderBy(r => r.Car.Position.DistanceTo(Game.Player.Character.Position))
                .FirstOrDefault();
            if (closestToPlayer != this) return;


            if (showTrack && Driver.IsPlayer && Lap >= ARS.SettingsFile.GetValue<int>("GENERAL_SETTINGS", "Laps", 5) && CanRegisterNewLap)
            {
                World.DrawMarker(MarkerType.CheckeredFlagRect, ARS.TrackPoints.First().Position + new Vector3(0, 0, 5f), ARS.TrackPoints.First().Direction, new Vector3(0, 0, 0), new Vector3(5f, 5f, 5f), Color.White);
            }


            if (showPhysics)
            {


                World.DrawMarker(MarkerType.DebugSphere, Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.1f, 0.1f, 0.1f), Color.Green, false, false, 0, false, "", "", false);


                Vector3 avgGs = VehicleData.AverageAcceleration;
                avgGs.Z = 0f;

                float colorPercent = ARS.Remap(avgGs.Length() / 9.8f, 0, VehicleData.CurrentMechanicalGrip, 0, 100, true);
                Color gColor = ARS.GradientAtoBtoC(Color.White, Color.Yellow, Color.Red, colorPercent);

                World.DrawMarker(MarkerType.DebugSphere, Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)) + (avgGs / 9.8f), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.15f, 0.15f, 0.15f), gColor, false, false, 0, false, "", "", false);
                ARS.DrawLine(Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)) + (avgGs / 9.8f), Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)), gColor);


                Vector3 maxValues = new Vector3(VehicleData.CurrentMechanicalGrip * 9.8f, VehicleData.CurrentMechanicalGrip * 9.8f, VehicleData.CurrentMechanicalGrip * 9.8f);
                Vector3 max = Vector3.Clamp(avgGs, -maxValues, maxValues);



                Vector3 source = Car.Position + new Vector3(0, 0, 0.5f + (Car.Model.GetDimensions().Z * 0.6f));
                ARS.DrawText(source, "~b~" + Math.Round(ARS.MpsToMph(Car.Velocity.Length())).ToString() + "~w~mph~n~~y~" + (avgGs.Length() / 9.8f).ToString("0.0") + " Gs", Color.White, 0.5f);


            }

            if (!Driver.IsPlayer)
            {

                if (!Car.IsInRangeOf(Game.Player.Character.Position, 500)) return;
                if (showInputs)
                {
                    DrawInputTrails();

                    // Speed readout (mph).
                    ARS.DrawText(Car.Position + new Vector3(0, 0, 2f),
                        "~w~SPD ~g~" + ARS.MpsToMph(Car.Velocity.Length()).ToString("0") + "~w~/~b~" + ARS.MpsToMph(Brain.CurrentIntention.Speed).ToString("0") + "mph",
                        Color.White, 0.4f);
                    ARS.DrawText(Car.Position + new Vector3(0, 0, 2.4f),
                        "~w~corn ~o~" + ARS.MpsToMph(_debugCornerSpd).ToString("0") + "~w~ rte ~c~" + ARS.MpsToMph(_debugFollowTrackSpd).ToString("0"),
                        Color.White, 0.4f);
                    ARS.DrawText(Car.Position + new Vector3(0, 0, 2.8f),
                        "~w~spdCap ~p~" + ARS.MpsToMph(_speedCap).ToString("0"),
                        Color.White, 0.4f);
                }
                if (showTrack)
                {
                    Vector3 trackCenter = CurrentTrackPoint.Position;
                    Vector3 trackRight = Vector3.Cross(CurrentTrackPoint.Direction, Vector3.WorldUp);
                    Vector3 leftWallPos = trackCenter + trackRight * _avoidLeftWall;
                    Vector3 rightWallPos = trackCenter + trackRight * _avoidRightWall;
                    Vector3 up = new Vector3(0, 0, 0.1f);
                    float wallHeight = 1.5f;
                    Vector3 leftTop = leftWallPos + up + new Vector3(0, 0, wallHeight);
                    Vector3 rightTop = rightWallPos + up + new Vector3(0, 0, wallHeight);
                    ARS.DrawLine(leftWallPos + up, leftTop, Color.Blue);
                    ARS.DrawLine(rightWallPos + up, rightTop, Color.Red);
                    ARS.DrawLine(leftTop, rightTop, Color.White);

                    // Lane aim spheres: car to final clamped lane target.
                    if (LookAheads.TryGetValue(LookAhead.SteerRef, out TrackPoint laneRef) && laneRef != null)
                    {
                        Vector3 laneRight = Vector3.Cross(laneRef.Direction, Vector3.WorldUp).Normalized;
                        Vector3 laneAim = laneRef.Position + laneRight * _targetLane;
                        for (int s = 1; s <= 10; s++)
                        {
                            float t = s / 10f;
                            Vector3 spherePos = Car.Position + (laneAim - Car.Position) * t;
                            World.DrawMarker(MarkerType.DebugSphere, spherePos, Vector3.Zero, Vector3.Zero, new Vector3(0.2f, 0.2f, 0.2f), Color.White, false, false, 0, false, "", "", false);
                        }
                    }

                    ARS.DrawText(Car.Position + new Vector3(0, 0, 2.4f), "~o~R: ~w~" + Brain.CurrentPerception.HighSpeedCurveRadius.ToString("0"), Color.White, 0.4f);
                    ARS.DrawText(Car.Position + new Vector3(0, 0, 2.8f), "~p~pitch ~w~" + _debugHillPitch.ToString("0.0"), Color.White, 0.4f);

                    // Projection debug is also shown with Track Analysis.
                    DrawProjectionDebug();
                }

                if (showAggro)
                {
                    Color pressureColor = ARS.GradientAtoBtoC(Color.Green, Color.Yellow, Color.Red, Pressure);
                    World.DrawMarker(MarkerType.ChevronUpx1, Car.Position + new Vector3(0f, 0f, 1.5f), Vector3.Zero, Vector3.Zero, new Vector3(0.5f, 0.5f, -0.5f), pressureColor, false, true, 0, false, "", "", false);

                    ARS.DrawText(Car.Position + new Vector3(0, 0, 2f), ((int)Pressure).ToString(), Color.White, 0.4f);

                    DrawProjectionDebug();
                }

                if (showTrack)
                {
                    DrawRouteFollowLine();
                }

            }
            else
            {
                if (showTrack)
                {
                    DrawRouteFollowLine();
                }
            }
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
            Rival threat = Brain.Rivals.FirstOrDefault(r => r.RivalRacer != null && r.RivalRacer.Car.Exists() && !float.IsInfinity(r.SecondsToHit) && !float.IsNaN(r.SecondsToHit) && r.SecondsToHit < 5f);
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
            int lineCount = (showInputs ? 4 : 0) + (showTrack ? 3 : 0);
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
                    requestingMore ? Color.Red : Color.White, ARS.DrawTextFont.Default, ARS.DrawTextAlign.Left, 0.35f);
                y += lineHeight;

                ARS.DrawText(new Vector2(0.79f, y), "GRV " + Handling.Gravity.ToString("0.00"),
                    Color.White, ARS.DrawTextFont.Default, ARS.DrawTextAlign.Left, 0.35f);
                y += lineHeight;

                float nativeGrip = Function.Call<float>((Hash)0xA132FB5370554DB0, Car);
                ARS.DrawText(new Vector2(0.79f, y), "GRP " + nativeGrip.ToString("0.00"),
                    Color.White, ARS.DrawTextFont.Default, ARS.DrawTextAlign.Left, 0.35f);
                y += lineHeight;

                float speedDiff = _debugFollowTrackSpd - Car.Velocity.Length();
                ARS.DrawText(new Vector2(0.79f, y), "DIFF " + ARS.MpsToMph(speedDiff).ToString("0") + " mph",
                    Color.White, ARS.DrawTextFont.Default, ARS.DrawTextAlign.Left, 0.35f);
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
            ARS.DrawText(new Vector2(0.79f, y), value, color, ARS.DrawTextFont.Default, ARS.DrawTextAlign.Left, 0.35f);
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
                    && ARS.IsBetween(r.SecondsToHit, 0f, 5f)
                    && ARS.IsBetween(r.FrontGap, 0f, 5f)
                    && ARS.IsBetween(r.DirectionDiff, -20f, 20f);
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

            bool hasCrossedStartLine = !ARS.IsPointToPoint
                && CanRegisterNewLap
                && refTrackpoint >= lastNode - 6
                && Vector3.Dot(Car.Position - ARS.TrackPoints[0].Position, ARS.TrackPoints[0].Direction) > 0f;
            if (hasCrossedStartLine) _trackPositionScratch.Add(ARS.TrackPoints[0]);

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




            if (CanRegisterNewLap)
            {
                if (hasCrossedStartLine || (ARS.IsPointToPoint && ARS.GetPercent(CurrentTrackPoint.Node, ARS.TrackPoints.Count) > 99 && ARS.EntityRelativeOffset(Car, ARS.TrackPoints.Last().Position).Y < 0f))
                {
                    CanRegisterNewLap = false;
                    _hasLeftLapArmNode = false;
                    Lap++;
                    if (Lap > ARS.SettingsFile.GetValue("GENERAL_SETTINGS", "Laps", 5))
                    {
                        if (Car.CurrentBlip != null) Car.CurrentBlip.Color = BlipColor.Green;
                    }

                    if (Lap > 1)
                    {
                        TimeSpan lapTime = ARS.ParseToTimeSpan(Game.GameTime - LapStartTime);
                        UI.Notify(Name + "'s laptime: ~b~" + lapTime.ToString("m':'ss'.'f"));
                        LapTimes.Add(lapTime);
                        LapStartTime = Game.GameTime;
                    }
                }
            }
            else if (BaseBehavior == RacerBaseBehavior.Race)
            {
                if (CurrentTrackPoint.Node < 100) _hasLeftLapArmNode = true;
                else if (_hasLeftLapArmNode) CanRegisterNewLap = true;
            }

            _lastApexProgressNode = CurrentTrackPoint.Node;

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
        const float EntranceBrakeBufferSeconds = 1f;
        const float EntranceBrakeExtraDistance = 0f;
        const float SecondaryApexSpeedDifference = 5f;
        const float BrakingTargetFactor = 0.66f;
        const bool RouteSpeedEnabled = true;
        const bool ConfidenceEnabled = true;

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
            NextApexSpeed = NextApexNode >= 0 ? ApexSpeedWithDownforce(NextApexRadius) + GetCornerSpeedOffset(NextApexNode) : 999f;
            NextApexNode2 = nodes[1];
            NextApexRadius2 = radii[1];
            NextApexSpeed2 = NextApexNode2 >= 0 ? ApexSpeedWithDownforce(NextApexRadius2) + GetCornerSpeedOffset(NextApexNode2) : 999f;
            NextApexNode3 = nodes[2];
            NextApexRadius3 = radii[2];
            NextApexSpeed3 = NextApexNode3 >= 0 ? ApexSpeedWithDownforce(NextApexRadius3) + GetCornerSpeedOffset(NextApexNode3) : 999f;
            NextApexNode4 = nodes[3];
            NextApexRadius4 = radii[3];
            NextApexSpeed4 = NextApexNode4 >= 0 ? ApexSpeedWithDownforce(NextApexRadius4) + GetCornerSpeedOffset(NextApexNode4) : 999f;

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
            int entranceNode = corner == null
                ? apexNode
                : (corner.StartNode >= 0
                    ? corner.StartNode
                    : OffsetCornerNode(apexNode, -corner.LengthStart));
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
            float decel = brakingAbility * Handling.Gravity * _brakeDecelFactor;
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
            if (float.IsNaN(r) || float.IsInfinity(r)) r = 999f;
            r = ARS.Clamp(r, 5f, 999f);
            float baseSpeed = (float)Math.Sqrt((VehicleData.CurrentMechanicalGrip * Handling.Gravity) * r);
            if (Handling.Downforce < 1f) return baseSpeed;
            float dfGs = ARS.GetDownforceGsAtSpeed(this, baseSpeed);
            float newGrip = VehicleData.CurrentMechanicalGrip + dfGs;
            return (float)Math.Sqrt(newGrip * Handling.Gravity * r);
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

                if (BaseBehavior == RacerBaseBehavior.Race && Math.Abs(VehicleData.SlideAngle) < 0.5f && Math.Abs(Control.Throttle) > 0.9f) Function.Call((Hash)0x81E1552E35DC3839, Car, true);

                if (Function.Call<bool>((Hash)0x3D34E80EED4AE3BE, Car) && Control.Brake > 0.1f) Function.Call((Hash)0x81E1552E35DC3839, Car, false);


                if (ARS.RaceSettingsFile.GetValue<int>("RACERS", "AIRacerAutofix", 1) == 2 && Function.Call<bool>(Hash._IS_VEHICLE_DAMAGED, Car))
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

                // Re-rise the speed cap before concern sources pull it down.
                _speedCap = Math.Min(999f, _speedCap + SpeedCapRiseRate * TickScale);

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
                HandleRecoveryAttemptEscalation();
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
                HandleRecoveryAttemptEscalation();
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
            // Smoothly reposition to the track edge after repeated failed recovery attempts.
            if (_isLerpingToTrack)
            {
                float elapsed = Game.GameTime - _lerpStartTime;
                float t = ARS.Clamp(elapsed / LerpToTrackMs, 0f, 1f);
                float smooth = t * t * (3f - 2f * t);
                Car.Position = _lerpStartPos + (_lerpTargetPos - _lerpStartPos) * smooth;
                Car.Velocity = Vector3.Zero;
                if (t >= 1f)
                {
                    _isLerpingToTrack = false;
                    _lerpStartTime = 0;
                }
                return; // skip the reverse-rock while lerping
            }

            if (!_isRecoveringFromStuck) return;

            if (Game.GameTime >= _stuckRecoveryEndTime)
            {
                _isRecoveringFromStuck = false;
                _stuckRecoveryEndTime = 0;
                _lastStuckGameTime = 0;
                return;
            }


            Control.SteerInput = 0f;
            Control.Throttle = -1f;
            Control.Brake = 0f;
        }

        void HandleRecoveryAttemptEscalation()
        {
            if (_stuckRecoveryAttempts < 2) return;

            // Smoothly reposition to the track edge after repeated failed recovery attempts.
            Vector3 toTrack = CurrentTrackPoint.Position - Car.Position;
            if (toTrack.Length() < 0.01f) return;

            Vector3 trackRight = Vector3.Cross(CurrentTrackPoint.Direction, Vector3.WorldUp).Normalized;
            float sideOffset = Vector3.Dot(Car.Position - CurrentTrackPoint.Position, trackRight);
            float edgeOffset = Math.Sign(sideOffset) * (CurrentTrackPoint.TrackHalfWidth - 1f);
            _lerpTargetPos = CurrentTrackPoint.Position + trackRight * edgeOffset;
            _lerpStartPos = Car.Position;
            _lerpStartTime = Game.GameTime;
            _isLerpingToTrack = true;
        }

        void UpdatePerceivedGrip()
        {


            float handlingGrip = Function.Call<float>((Hash)0xA132FB5370554DB0, Car);
            handlingGrip = ARS.Clamp(handlingGrip, 0.1f, 5f);

            GroundGripMultiplier = ARS.WheelGripMultipliers(Car).Average();

            VehicleData.BaseMechanicalGrip = handlingGrip;
            // Stability-aware grip: AvgGroundStability (0.8..1.0) multiplies the mechanical grip so
            // grip loss from wheels lifting makes the AI more careful on corners and brake earlier.
            VehicleData.CurrentMechanicalGrip = VehicleData.BaseMechanicalGrip * GroundGripMultiplier * VehicleData.AvgGroundStability;

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

            // Slip-based stability: count wheels off the ground (slip ~0) while moving, then drop
            // stability 0.1/s per wheel off, rise 1/s, clamped to 0.8..1.0.
            bool moving = Car.Velocity.Length() > 1f;
            _wheelsOffGround = moving ? ARS.WheelSlips(Car).Count(s => Math.Abs(s) < 0.01f) : 0;

            if (_wheelsOffGround > 0)
            {
                VehicleData.AvgGroundStability -= 0.1f * _wheelsOffGround * TickScale;
            }
            else if (VehicleData.AvgGroundStability < 1f)
            {
                VehicleData.AvgGroundStability += 1f * TickScale;
            }
            VehicleData.AvgGroundStability = ARS.Clamp(VehicleData.AvgGroundStability, 0.8f, 1f);

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


