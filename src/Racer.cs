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
        int _lastCoreTick = 100;
        int TimeSince_lastCoreTick => (int)ARS.Clamp(Game.GameTime - _lastCoreTick, 1, 9999);


        int _lastStuckGameTime = 0;
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
        float _accelerationCap = 1f; 
        float _speedCap = 999f;
        const float SpeedCapRiseRate = 30f;

        // Speed bias that drifts according to projected track safety.
        float _confidence = 0f;
        const float ConfidenceNeutralWiggleroom = 2f;



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
        float _laneGainDivisor = 100f;
        const float PressureRange = 100f;
        const float PressureProximityRange = 100f;
        const float PressureRisePerSecond = 2f;
        const float PressureFallPerSecond = 30f;
        const float PressureMaxSpeedOffset = 5f;

        public Racer(Vehicle RacerCar, Ped RacerPed)
        {
            Car = RacerCar;
            Driver = RacerPed;
            try { Name = RacerCar.FriendlyName; } catch (Exception) { Name = "Racer"; }
            if (Name == "NULL" || Name == null) { try { Name = Car.DisplayName.ToString()[0].ToString().ToUpper() + Car.DisplayName.ToString().Substring(1).ToLowerInvariant(); } catch (Exception) { Name = "Racer"; } }

            if (Driver.IsPlayer) ControlledByPlayer = true;
            _halfSecondTick = Game.GameTime + (ARS.GetRandomInt(10, 50));
            _laneGainDivisor = (float)ARS.GetRandomInt(90, 110);

            if (!ControlledByPlayer)
            {

                try { Driver.BlockPermanentEvents = true; } catch (Exception) { }
                try { Driver.AlwaysKeepTask = true; } catch (Exception) { }
                Function.Call(GTA.Native.Hash.SET_DRIVER_ABILITY, Driver, 0f);
                Function.Call(GTA.Native.Hash.SET_DRIVER_AGGRESSIVENESS, Driver, 0f);

                if (ARS.DevSettingsFile.GetValue<int>("RACERS", "AIRacerAutofix", 1) == 2)
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
                else if (ARS.DevSettingsFile.GetValue<int>("RACERS", "AIRacerAutofix", 1) == 1)
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
            if (Handling.Downforce > 100) Handling.Downforce *= 0.1f;

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

            Handling.Grip = Function.Call<float>((Hash)0xA132FB5370554DB0, Car) * (Handling.Gravity / 9.8f);

            VehicleData.PerformanceIndex = (int)((Handling.EstimatedTopSpeed * 5) + (Handling.Grip * 100) + (Handling.Acceleration * 500));
            float modelPower = Function.Call<float>(Hash.GET_VEHICLE_MODEL_ACCELERATION, Car.Model.Hash);
            float modelTopSpeed = ARS.MpsToMph(Function.Call<float>((Hash)0xF417C2502FFFED43, Car.Model.Hash)) / ARS.TopSpeedScaleDivisor;
            VehicleData.PowerScale = modelPower + modelTopSpeed;
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



            Vector3 carForward = Car.Velocity.LengthSquared() > 0.01f
                ? Car.Velocity.Normalized
                : Car.ForwardVector;
            float headingErrorDeg = -Vector3.SignedAngle(
                steerRefPoint.Direction, carForward, Vector3.WorldUp);
            if (float.IsNaN(headingErrorDeg) || float.IsInfinity(headingErrorDeg))
                headingErrorDeg = 0f;


            headingErrorDeg *= 0.5f;

            float speedMps = Math.Max(Car.Velocity.Length(), 1f);
            float carHalfWidth = VehicleData.BoundingBox * 0.5f;


            float naturalLane = ComputeHighSpeedLane(roadWide);

            // Corner approach temporarily overrides the high-speed inside line.
            bool cornerActive = Brain.Corner != null && Lap > 0;
            float cornerLane = cornerActive ? ComputeCornerTargetLane(steerRefPoint, speedMps) : 0f;
            if (cornerLane != 0f) naturalLane = cornerLane;
            _rawCornerLane = cornerLane;

            // Avoidance overrides track-line targets, except during a committed divebomb.
            float avoidAheadLane = 0f;
            if (ActiveManeuver.Type != ManeuverType.DiveBomb)
            {
                avoidAheadLane = ComputeAvoidAheadLane(roadWide);
                if (avoidAheadLane != 0f) naturalLane = avoidAheadLane;
            }



            float clampedLane = ApplyRivalWalls(naturalLane, roadWide);
            _targetLane = clampedLane;


            float recoveryDeg = 0f;
            float absDev = Math.Abs(Brain.CurrentPerception.DeviationFromCenter);
            float safeEdge = roadWide - carHalfWidth;
            float overshoot = absDev - safeEdge;
            if (overshoot > 0f)
            {
                float maxRecoveryDeg = ARS.Remap(speedMps, 10f, 50f, 10f, 2f);
                float severity = Math.Min(overshoot / Math.Max(safeEdge, 1f), 1f);
                recoveryDeg = Math.Sign(Brain.CurrentPerception.DeviationFromCenter) * maxRecoveryDeg * severity;
            }



            float laneBiasDeg = 0f;
            float trackBound = roadWide - carHalfWidth;
            bool hasActiveGuidance = Math.Abs(clampedLane) > 0.01f
                || _avoidLeftWall > -trackBound
                || _avoidRightWall < trackBound;
            if (hasActiveGuidance)
            {
                float currentLane = Brain.CurrentPerception.DeviationFromCenter;
                float lookaheadDist = steerRefPoint.Position.DistanceTo(Car.Position);
                if (lookaheadDist < 1f) lookaheadDist = speedMps * 1.5f;


                float laneError = clampedLane - currentLane;
                laneBiasDeg = -(float)(Math.Atan2(laneError, lookaheadDist) * (180.0 / Math.PI));
                float expGain = (float)Math.Pow(Math.Min(Math.Abs(laneBiasDeg) / _laneGainDivisor, 1f), 0.66f);
                expGain = Math.Min(expGain, 0.3f);
                laneBiasDeg *= expGain;
            }


            float totalTargetDeg = headingErrorDeg + laneBiasDeg + recoveryDeg;


            const float steerKP = 1.0f;
            float gripForKD = VehicleData.CurrentMechanicalGrip;
            if (float.IsNaN(gripForKD) || float.IsInfinity(gripForKD) || gripForKD <= 0f) gripForKD = 1f;
            float steerKD = 0.5f / gripForKD;
            float pTerm = steerKP * totalTargetDeg;
            float dTerm = steerKD * VehicleData.YawRotationPerSecondDegrees;
            Control.SteerDegrees = pTerm - dTerm;


            if (Math.Sign((int)VehicleData.SlideAngle) == Math.Sign((int)VehicleData.YawRotationPerSecondDegrees))
            {
                float slideCounterSteer = VehicleData.SlideAngle * ARS.Remap(
                    Math.Abs(VehicleData.SlideAngle), 0, Handling.LateralTractionCurve * 1.2f, 0.5f, 1.2f, true);
                Control.SteerDegrees -= slideCounterSteer;
            }

            // NaN/Inf guard: ApplySteerLimits would turn NaN into full-lock.
            if (float.IsNaN(Control.SteerDegrees) || float.IsInfinity(Control.SteerDegrees))
                Control.SteerDegrees = 0f;



            bool TryGetSteerContext(out TrackPoint localSteerRef, out float localRoadWide)
            {
                localSteerRef = null;
                localRoadWide = 0f;

                if (BaseBehavior == RacerBaseBehavior.GridWait
                    || BaseBehavior == RacerBaseBehavior.FinishedStandStill
                    || CurrentTrackPoint.Node < 3)
                {
                    return false;
                }

                if (!LookAheads.TryGetValue(LookAhead.SteerRef, out localSteerRef)
                    || localSteerRef == null)
                {
                    return false;
                }

                localRoadWide = localSteerRef.TrackHalfWidth;
                return true;
            }
        }






        // Lane Control System 2: positions the car on the inside edge of the track curvature.
        float ComputeHighSpeedLane(float roadWide)
        {
            if (Brain.CurrentPerception.HighSpeedCurveRadius > 250f) return 0f;

            int count = ARS.TrackPoints.Count;
            int backNode, fwdNode;
            if (ARS.IsPointToPoint)
            {
                backNode = (int)ARS.Clamp(CurrentTrackPoint.Node - 20, 0, count - 1);
                fwdNode = (int)ARS.Clamp(CurrentTrackPoint.Node + 20, 0, count - 1);
            }
            else
            {
                backNode = ((CurrentTrackPoint.Node - 20) % count + count) % count;
                fwdNode = ((CurrentTrackPoint.Node + 20) % count + count) % count;
            }
            Vector3 backDir = ARS.TrackPoints[backNode].Direction;
            Vector3 fwdDir = ARS.TrackPoints[fwdNode].Direction;

            float signedAngle = Vector3.SignedAngle(backDir, fwdDir, Vector3.WorldUp);
            if (float.IsNaN(signedAngle) || float.IsInfinity(signedAngle)) return 0f;

            if (Math.Abs(signedAngle) < 1f) return 0f;

            float cornerDir = Math.Sign(signedAngle);
            return -cornerDir * roadWide;
        }
        // Hold the outside line on entry, then release it for the high-speed inside line.
        float ComputeCornerTargetLane(TrackPoint steerRefPoint, float speedMps)
        {
            CornerPoint c = Brain.Corner.Point;
            int apexNode = c.Node;

            if (apexNode != _approachCornerNode)
            {
                _approachCornerNode = apexNode;
                _approachOutsideDecided = false;
                _approachHoldsOutside = false;
            }

            float distToApexNodes = Math.Abs(apexNode - CurrentTrackPoint.Node);
            float timeToApex = distToApexNodes / Math.Max(speedMps, 1f);
            const float approachStartTime = 5.0f;
            if (timeToApex > approachStartTime) return 0f;

            if (!_approachOutsideDecided)
            {
                _approachHoldsOutside = speedMps >= _cornerSpd + ARS.MphToMps(20f);
                _approachOutsideDecided = true;
            }

            float cornerDir = Math.Sign(c.Angle);
            if (cornerDir == 0f) return 0f;

            float halfWidth = steerRefPoint.TrackHalfWidth;
            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            float safeBound = halfWidth - carHalfWidth;

            float holdOutsideUntil = (steerRefPoint.TrackHalfWidth * 2f) / 10f;
            if (_approachHoldsOutside && timeToApex > holdOutsideUntil)
            {
                // Corner-commit: sit beside the target rival on the corner inside instead of the outside line.
                bool isCornerCommit = ActiveManeuver.Target != null
                    && (ActiveManeuver.Type == ManeuverType.DiveBomb || ActiveManeuver.Type == ManeuverType.DefendLane);
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

            float firstTarget = 0f;
            bool firstGoLeft = false;
            bool foundFirst = false;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null) continue;
                if (r.RelativePosition != RelativePos.Ahead) continue;
                if (r.SecondsToReach < 0f || r.SecondsToReach > 3f) continue;
                if (Math.Abs(r.DirectionDiff) > 20f) continue;

                float rivalLane = r.OccupiedLane;
                float buffer = r.OccupiedLaneWidth + aggroBuffer;

                float roomLeft = rivalLane - buffer + trackBound;
                float roomRight = trackBound - (rivalLane + buffer);
                bool goLeft = roomLeft > roomRight;

                float targetLane = goLeft
                    ? rivalLane - buffer - carHalfWidth
                    : rivalLane + buffer + carHalfWidth;

                // If the chosen side is off-track, flip.
                if (Math.Abs(targetLane) > trackBound)
                {
                    targetLane = goLeft
                        ? rivalLane + buffer + carHalfWidth
                        : rivalLane - buffer - carHalfWidth;
                    goLeft = !goLeft;
                }

                if (Math.Abs(targetLane) > trackBound) continue;

                // Already on the far side, no avoidance needed.
                if (goLeft && currentLane <= targetLane) continue;
                if (!goLeft && currentLane >= targetLane) continue;

                if (!foundFirst)
                {
                    firstTarget = targetLane;
                    firstGoLeft = goLeft;
                    foundFirst = true;
                }
                else
                {
                    if (goLeft != firstGoLeft)
                        return (firstTarget + targetLane) * 0.5f;
                    return firstTarget;
                }
            }

            return foundFirst ? firstTarget : 0f;
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

                bool isRelevant = false;
                bool rivalIsLeft = false;

                if (r.RelativePosition == RelativePos.Left)
                {
                    isRelevant = true;
                    rivalIsLeft = true;
                }
                else if (r.RelativePosition == RelativePos.Right)
                {
                    isRelevant = true;
                    rivalIsLeft = false;
                }

                if (!isRelevant) continue;


                float aggroBuffer = ARS.Remap(Aggression, 100f, 0f, 0.2f, 1.2f, true);
                float rivalBuffer = r.OccupiedLaneWidth + aggroBuffer;

                if (rivalIsLeft)
                {
                    float wall = r.OccupiedLane + rivalBuffer;
                    if (wall > targetLeftWall) targetLeftWall = wall;
                    leftConstrained = true;
                }
                else
                {
                    float wall = r.OccupiedLane - rivalBuffer;
                    if (wall < targetRightWall) targetRightWall = wall;
                    rightConstrained = true;
                }
            }


            float openRate = 2f * TickScale;
            if (leftConstrained)
                _avoidLeftWall = targetLeftWall;
            else
                _avoidLeftWall = Math.Max(_avoidLeftWall - openRate, -trackBound);

            if (rightConstrained)
                _avoidRightWall = targetRightWall;
            else
                _avoidRightWall = Math.Min(_avoidRightWall + openRate, trackBound);

            // Rival walls must remain ordered; collapse overlap to a narrow centered corridor.
            if (_avoidLeftWall > _avoidRightWall)
            {
                float mid = (_avoidLeftWall + _avoidRightWall) * 0.5f;
                _avoidLeftWall = mid - 1f;
                _avoidRightWall = mid + 1f;
            }


            float clampLeft = _avoidLeftWall + carHalfWidth;
            float clampRight = _avoidRightWall - carHalfWidth;
            clampLeft = Math.Max(clampLeft, -trackBound);
            clampRight = Math.Min(clampRight, trackBound);
            // Guard: if walls crossed after car inset, Clamp(min>max) inverts. Fall back to track bound.
            if (clampLeft > clampRight) return ARS.Clamp(targetLane, -trackBound, trackBound);
            return ARS.Clamp(targetLane, clampLeft, clampRight);
        }

        void ApplySteerLimits()
        {
            // NaN guard: Clamp would turn NaN into full-lock.
            if (float.IsNaN(Control.SteerDegrees) || float.IsInfinity(Control.SteerDegrees))
            {
                Control.SteerDegrees = 0f;
                return;
            }

            if (Math.Sign(Control.SteerDegrees) == Math.Sign((int)VehicleData.YawRotationPerSecondDegrees))
            {
                float speedBasedSteeringLimit = (float)((VehicleData.BaseMechanicalGrip * Handling.Gravity * VehicleData.WheelBase) / Math.Pow(Car.Velocity.Length() + 0.01f, 2.0f));
                float steerMultiplier = ARS.Remap(Control.Throttle, 0f, 1f, 1f, 0.8f, true);
                speedBasedSteeringLimit = Math.Max(ARS.RadToDeg(speedBasedSteeringLimit) * steerMultiplier, Handling.LateralTractionCurve * 0.5f);

                // Oversteer throttle cut: steering 10 degrees past the grip limit, back off power.
                if (Math.Abs(Control.SteerDegrees) > speedBasedSteeringLimit + 10f)
                    Control.MaxThrottle = Math.Max(Control.MaxThrottle - 1f * TickScale, 0.1f);

                Control.SteerDegrees = ARS.Clamp(Control.SteerDegrees, -speedBasedSteeringLimit, speedBasedSteeringLimit);
            }
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
            float newThrottle = 0f;
            float newBrake = 0f;
            float throttleCap = Math.Min(Control.MaxThrottleFromTCS, 1f);
            float dirSwitchSpeed = ARS.MphToMps(5f);


            if (BaseBehavior == RacerBaseBehavior.GridWait)
            {
                Brain.CurrentIntention.Speed = 99f;
                Control.HandBrakeTime = Game.GameTime + 500;
            }



            if (Brain.CurrentIntention.Speed >= 0f)
            {
                Brain.CurrentIntention.Speed = Math.Min(Brain.CurrentIntention.Speed, ARS.EngineTopSpeed(Car) * 1.3f);
                Brain.CurrentIntention.Speed = Math.Min(Brain.CurrentIntention.Speed, Brain.CurrentIntention.MaxSpeed);
                Brain.CurrentIntention.Speed = Math.Min(Brain.CurrentIntention.Speed, _speedCap);
            }

            if ((Game.GameTime - LapStartTime) < 3000) Brain.CurrentIntention.IntendedSpeedChangeGs = 999;
            else
            {
                float currentLongitudinalSpeed = VehicleData.SpeedVectorLocal.Y;
                Brain.CurrentIntention.IntendedSpeedChangeGs = (Brain.CurrentIntention.Speed - currentLongitudinalSpeed) / 9.8f;
            }            

            float currentForwardSpeed = VehicleData.SpeedVectorLocal.Y;
            float speedErrorGs = Brain.CurrentIntention.IntendedSpeedChangeGs;
            bool wantsReverse = Brain.CurrentIntention.Speed < -0.1f;

            // Apply _accelerationCap: positive multiplies throttle, negative sets minimum brake.
            if (speedErrorGs > 0.0f)
            {

                if (currentForwardSpeed < -dirSwitchSpeed) newBrake = ARS.Clamp(speedErrorGs / 0.3f, 0f, 1f);
                else newThrottle = ARS.Clamp(speedErrorGs / 0.3f, 0f, throttleCap) * Math.Max(_accelerationCap, 0f);
            }
            else if (speedErrorGs < 0.0f)
            {
                    float reverseDemand = ARS.Clamp((-speedErrorGs) / 0.3f, 0f, throttleCap);
                    float brakeDemand = ARS.Clamp((-speedErrorGs) / 0.3f, 0f, 1f);

                if (wantsReverse)
                {

                    if (currentForwardSpeed > dirSwitchSpeed) newBrake = brakeDemand;
                    else newThrottle = -reverseDemand;
                }
                else
                {
                    newBrake = brakeDemand;
                }
            }


            // Brake floor from _accelerationCap (negative part = min brake).
            float accelCapBrakeMin = -Math.Min(_accelerationCap, 0f);
            if (accelCapBrakeMin > 0f)
            {
                newBrake = Math.Max(newBrake, accelCapBrakeMin);
            }

            if (newBrake > 0.0) newThrottle = 0; else newBrake = 0;

            // Confidence is an additive longitudinal input offset. It may not cancel braking.
            if (_confidence > 0f && newBrake <= 0f)
                newThrottle = ARS.Clamp(newThrottle + _confidence, 0f, throttleCap);
            else if (_confidence < 0f)
            {
                newThrottle = 0f;
                newBrake = ARS.Clamp(newBrake - _confidence, 0f, 1f);
            }


            float inputChange = 10f * TickScale;
            Control.Brake += ARS.Clamp(newBrake - Control.Brake, -inputChange, inputChange);
            Control.Throttle += ARS.Clamp(newThrottle - Control.Throttle, -inputChange, inputChange);
            Control.Throttle = Math.Min(Control.Throttle, Control.MaxThrottle);
            if (Control.MaxThrottle < 1.00f) Control.MaxThrottle += 2 * TickScale;

            if (Brain.CurrentIntention.MaxSpeed < AiConstants.MaxSpeed) Brain.CurrentIntention.MaxSpeed += 15 * TickScale;

        }
        float TickScale => (0.001f * TimeSince_lastCoreTick);




        void TranslateSteerToInput()
        {

            if (float.IsNaN(Control.SteerDegrees) || float.IsInfinity(Control.SteerDegrees)) Control.SteerDegrees = 0f;

            float maxDeltaPerTick = 90 * TickScale;
            float deltaToTarget = ARS.Clamp(Control.SteerDegrees - Control.LastAppliedSteerDegrees, -maxDeltaPerTick, maxDeltaPerTick);
             
            Control.SteerDegrees = Control.LastAppliedSteerDegrees+ deltaToTarget;

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
            float followTrackSpd = RouteIdealSpeedForRadius(Brain.CurrentPerception.CurveRadiusToFollowPoint);

            if (float.IsNaN(cornerSpd) || float.IsInfinity(cornerSpd)) cornerSpd = 999f;
            if (float.IsNaN(followTrackSpd) || float.IsInfinity(followTrackSpd)) followTrackSpd = 999f;
            if (cornerSpd <= 5) cornerSpd = ARS.CornerApexSpeed(Brain.Corner.Point, this);

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
                float routeAggression = ARS.Remap(Brain.CurrentPerception.CurveRadiusToFollowPoint, 100f, 300f, 0f, 1f, true);
                float effectiveDeltaGs = deltaGs;
                if (effectiveDeltaGs < 0f) effectiveDeltaGs *= (1f - routeAggression);
                float verticalGripFactor = Math.Max(1f + effectiveDeltaGs, 0.8f);
                followTrackSpd *= (float)Math.Sqrt(verticalGripFactor);
            }

            // Pressure overspeed on route speed.
            followTrackSpd += PressureMaxSpeedOffset * (Pressure / PressureRange);

            // Pure apex speed for the corner-approach gate.
            _cornerSpd = NextApexNode >= 0 ? NextApexSpeed : (Brain.Corner != null ? ARS.CornerApexSpeed(Brain.Corner.Point, this) : 999f);

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
                    if (cornerEffectiveDelta < 0f) cornerEffectiveDelta *= (1f - CrestAggression);
                    float cornerVerticalGrip = Math.Max(1f + cornerEffectiveDelta, 0.8f);
                    cornerSpd *= (float)Math.Sqrt(cornerVerticalGrip);
                }
            }

            _debugCornerSpd = cornerSpd;
            _debugFollowTrackSpd = followTrackSpd;
            Brain.CurrentIntention.Speed = Math.Min(cornerSpd, followTrackSpd);

            // ConfidenceMPS: drift toward a target based on the 0.5s and 1s projections.
            {
                Vector3 projHalf = ProjectAhead(0.5f);
                Vector3 proj1s = ProjectAhead(1f);
                bool anyOff = false;
                bool anyEdge = false;
                Vector3[] projections = { projHalf, proj1s };
                float carHalf = VehicleData.BoundingBox * 0.5f;
                for (int i = 0; i < projections.Length; i++)
                {
                    TrackPoint tp = ARS.TrackPoints.OrderBy(t => t.Position.DistanceTo2D(projections[i])).First();
                    float signedLateralOffset = ARS.SignedLaneOffset(projections[i], tp.Position, tp.Direction);
                    bool isInsideCorner = Math.Abs(tp.Angle) > 1f
                        && Math.Sign(signedLateralOffset) == -Math.Sign(tp.Angle);
                    if (isInsideCorner) continue;

                    float lateralOffset = Math.Abs(signedLateralOffset);
                    float edge = tp.TrackHalfWidth - carHalf;
                    if (lateralOffset > edge) anyOff = true;
                    else if (lateralOffset > edge - ConfidenceNeutralWiggleroom) anyEdge = true;
                }

                float target;
                float absoluteLateralGs = Math.Abs(VehicleData.GetLateralGs(Car.ForwardVector));
                float positiveLongitudinalGs = Math.Max(0f, VehicleData.GetLongitudinalGs(Car.ForwardVector));
                float halfGripGs = VehicleData.CurrentMechanicalGrip * 0.5f;
                bool confidenceGate = absoluteLateralGs > halfGripGs && positiveLongitudinalGs < halfGripGs;
                if (!confidenceGate)
                {
                    target = 0f;
                }
                else if (anyOff)
                {
                    target = -1f;
                }
                else if (anyEdge) target = 0f;
                else target = 1f;

                float confidenceDriftRate = Math.Max(0.5f, Car.Velocity.Length() * 0.01f);
                if (_confidence < target)
                    _confidence = Math.Min(_confidence + confidenceDriftRate * TickScale, target);
                else if (_confidence > target)
                    _confidence = Math.Max(_confidence - confidenceDriftRate * TickScale, target);
            }

            // Yield: cap throttle to 0.5 to stay behind.
            if (ActiveManeuver.Type == ManeuverType.Yield && ActiveManeuver.Target != null)
            {
                Control.MaxThrottle = Math.Min(Control.MaxThrottle, 0.5f);
            }

            // Avoidance: lower _accelerationCap as a rival ahead closes.
            if (ActiveManeuver.Type != ManeuverType.Yield)
            {
                Rival avoidThreat = Brain.Rivals.FirstOrDefault(r => r.RivalRacer != null && r.RelativePosition == RelativePos.Ahead);
                if (avoidThreat != null)
                {
                    float avoidScalar = ARS.Remap(avoidThreat.Distance, 0f, 5f, -1f, 1f, true);
                    _accelerationCap = Math.Min(_accelerationCap, avoidScalar);
                }
            }
        }

        const float SlopeGripLossK = 3f;
        const float SlopeGripLossExp = 2f;
        // Reduces crest-induced grip loss as curvature allows more aggressive traversal.
        const float CrestAggression = 0.5f;

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
            if (Control.Throttle <= 0.0f) return;

            float wheelspin = ARS.MaxWheelSlip(Car);
          float  IdealWheelspin = OutOfTrackDistance() > 0f ? -1f : -2f;

            float error = wheelspin - IdealWheelspin;
            float change = error * TickScale * 5f;
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

            // Yield: arm if pressure is much lower than closest rival, within 5s of corner, in overlap
            if (ActiveManeuver.Type == ManeuverType.None && Brain.Corner != null)
            {
                Rival closestRival = Brain.Rivals.FirstOrDefault(r => r.RivalRacer != null);
                if (closestRival != null)
                {
                    float pressureDiff = closestRival.RivalRacer.Pressure - Pressure;
                    bool inOverlap = closestRival.RelativePosition == RelativePos.Left || closestRival.RelativePosition == RelativePos.Right;
                    float distToApex = Math.Abs(Brain.Corner.Point.Node - CurrentTrackPoint.Node);
                    float timeToApex = distToApex / Math.Max(Car.Velocity.Length(), 1f);

                    if (pressureDiff > 30f && inOverlap && timeToApex <= 5f
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
                float myTimeToApex = ForwardNodeDistance(apexNode) / Math.Max(Car.Velocity.Length(), 1f);

                Rival diveTarget = Brain.Rivals.FirstOrDefault(r =>
                    r.RivalRacer != null
                    && r.RelativePosition == RelativePos.Ahead
                    && r.RivalRacer.ActiveManeuver.Type != ManeuverType.DiveBomb
                    && r.RivalRacer.ActiveManeuver.Type != ManeuverType.DefendLane
                    && Math.Abs(r.RivalRacer.ForwardNodeDistance(apexNode) / Math.Max(r.RivalRacer.Car.Velocity.Length(), 1f) - myTimeToApex) <= 1f);

                if (diveTarget != null && Pressure > 30f && myTimeToApex <= 8f)
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
                float myTimeToApex = ForwardNodeDistance(apexNode) / Math.Max(Car.Velocity.Length(), 1f);

                Rival defenderTarget = Brain.Rivals.FirstOrDefault(r =>
                    r.RivalRacer != null
                    && r.RelativePosition == RelativePos.Behind
                    && r.RivalRacer.ActiveManeuver.Type != ManeuverType.DiveBomb
                    && r.RivalRacer.ActiveManeuver.Type != ManeuverType.DefendLane
                    && r.RivalRacer.ForwardNodeDistance(apexNode) / Math.Max(r.RivalRacer.Car.Velocity.Length(), 1f) <= myTimeToApex);

                if (defenderTarget != null && myTimeToApex <= 6f)
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
            Rival closestRival = Brain.Rivals.FirstOrDefault(r => r.RivalRacer != null);
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

            VehicleData.AccelerationVector.Add((cSpeed - _lastSpeed) / Game.LastFrameTime);

            if (VehicleData.AccelerationVector.Count > 10) VehicleData.AccelerationVector.RemoveAt(0);

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

            while (_trailSamples.Count > 10) _trailSamples.RemoveAt(0);
        }


        // Kinematic projection: pos + v*t + 0.5*a*t^2. Default t=1 (1s).
        public Vector3 ProjectAhead(float seconds = 1f)
        {
            Vector3 avgAccel = VehicleData.AccelerationVector.Aggregate(
                new Vector3(0, 0, 0), (s, v) => s + v)
                / (float)VehicleData.AccelerationVector.Count;
            return Car.Position + Car.Velocity * seconds + 0.5f * avgAccel * seconds * seconds;
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

            float myHalfLen = Math.Abs(Car.Model.GetDimensions().Y) * 0.5f;

            bool shouldPassengerize = false;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null) continue;
                float rivalHalfLen = Math.Abs(r.RivalRacer.Car.Model.GetDimensions().Y) * 0.5f;
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
            }

            if (requestedTrack) DrawCornerDebug();
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


                Vector3 avgGs = VehicleData.AccelerationVector.Aggregate(new Vector3(0, 0, 0), (s, v) => s + v) / (float)VehicleData.AccelerationVector.Count;
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
                        "~w~spdCap ~p~" + ARS.MpsToMph(_speedCap).ToString("0") + "~w~ accCap ~r~" + _accelerationCap.ToString("0.00") + "~w~ conf ~y~" + _confidence.ToString("0.0"),
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
                }

                if (showAggro)
                {
                    Color pressureColor = ARS.GradientAtoBtoC(Color.Green, Color.Yellow, Color.Red, Pressure);
                    World.DrawMarker(MarkerType.ChevronUpx1, Car.Position + new Vector3(0f, 0f, 1.5f), Vector3.Zero, Vector3.Zero, new Vector3(0.5f, 0.5f, -0.5f), pressureColor, false, true, 0, false, "", "", false);

                    ARS.DrawText(Car.Position + new Vector3(0, 0, 2f), ((int)Pressure).ToString(), Color.White, 0.4f);

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
            int lineCount = (showInputs ? 2 : 0) + (showTrack ? 3 : 0);
            if (lineCount == 0) return;

            float lineHeight = 0.026f;
            float top = 0.045f;
            float height = lineCount * lineHeight + 0.02f;
            Function.Call(Hash.DRAW_RECT, 0.89f, top + height * 0.5f, 0.22f, height, 0, 0, 0, 120);

            float y = top + 0.01f;
            if (showInputs)
            {
                ARS.DrawText(new Vector2(0.79f, y),
                    "SPD   " + ARS.MpsToMph(Car.Velocity.Length()).ToString("0") + " / " + ARS.MpsToMph(Brain.CurrentIntention.Speed).ToString("0") + " mph",
                    Color.White, ARS.DrawTextFont.Default, ARS.DrawTextAlign.Left, 0.35f);
                y += lineHeight;
                ARS.DrawText(new Vector2(0.79f, y),
                    "ERR   " + (Brain.CurrentIntention.Speed - VehicleData.SpeedVectorLocal.Y).ToString("0.0") + " m/s | " + Math.Abs(Brain.CurrentIntention.IntendedSpeedChangeGs * 9.8f).ToString("0.0") + " m/s/s",
                    Color.White, ARS.DrawTextFont.Default, ARS.DrawTextAlign.Left, 0.35f);
                y += lineHeight;
            }

            if (showTrack)
            {
                DrawApexPanelLine(ref y, "A1", NextApexNode, NextApexSpeed, NextApexRadius, Color.Yellow, lineHeight);
                DrawApexPanelLine(ref y, "A2", NextApexNode2, NextApexSpeed2, NextApexRadius2, Color.Orange, lineHeight);
                ARS.DrawText(new Vector2(0.79f, y),
                    "CONF  " + _confidence.ToString("+0.00;-0.00;0.00"),
                    Color.Cyan, ARS.DrawTextFont.Default, ARS.DrawTextAlign.Left, 0.35f);
                y += lineHeight;
            }
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
                float dimension = Car.Model.GetDimensions().Y + 1f;
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




        float OutOfTrackDistance()
        {
            return (Math.Abs(Brain.CurrentPerception.DeviationFromCenter) + (VehicleData.BoundingBox / 2)) - CurrentTrackPoint.TrackHalfWidth;
        }

        void UpdateRivalInfo()
        {
            foreach (Rival r in Brain.Rivals) r.Update(this);
        }




        public void UpdateTrackPosition()
        {


            int refTrackpoint = (int)ARS.Clamp(CurrentTrackPoint.Node, 0, ARS.TrackPoints.Count - 1);

            List<TrackPoint> points = new List<TrackPoint>();
            int lastNode = ARS.TrackPoints.Count - 1;
            int firstCandidate = Math.Max(refTrackpoint - 6, 0);
            int lastCandidate = Math.Min(refTrackpoint + 6, lastNode);
            for (int i = firstCandidate; i <= lastCandidate; i++)
            {
                points.Add(ARS.TrackPoints[i]);
            }

            bool hasCrossedStartLine = !ARS.IsPointToPoint
                && CanRegisterNewLap
                && refTrackpoint >= lastNode - 6
                && Vector3.Dot(Car.Position - ARS.TrackPoints[0].Position, ARS.TrackPoints[0].Direction) > 0f;
            if (hasCrossedStartLine) points.Add(ARS.TrackPoints[0]);

            TrackPoint closestPoint = points[0];
            float closestDistance = closestPoint.Position.DistanceTo(Car.Position);
            foreach (TrackPoint point in points)
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






            // Route radius from three sample points.
            Brain.CurrentPerception.CurveRadiusToFollowPoint = RouteRadiusSampled();
            UpdateNextApexes();
            // High-speed lane radius: short 0.5s to 1.0s window.
            Brain.CurrentPerception.HighSpeedCurveRadius = ComputeRouteRadius((int)(Car.Velocity.Length() * 0.5f), (int)(Car.Velocity.Length() * 1.0f));
            Brain.CurrentPerception.CurveRadiusAfterFollowPoint = ComputeRouteRadius((int)(Car.Velocity.Length() * 2.5f), (int)(Car.Velocity.Length() * 4.5f));
        }

        // Circumradius through three route-window sample points.
        float RouteRadiusSampled()
        {
            float speed = Car.Velocity.Length();
            int count = ARS.TrackPoints.Count;
            if (count < 10) return 999f;

            float grip = Math.Max(VehicleData.CurrentMechanicalGrip, 0.1f);
            int o1 = (int)(speed / grip);
            int o3 = (int)(speed * 3f / grip);
            if (o3 < o1 + 2) o3 = o1 + 2;
            int o2 = o1 + (o3 - o1) / 2;

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
        const float EntranceBrakeExtraDistance = 30f;
        const float SecondaryApexSpeedDifference = 5f;

        // Refresh useful precomputed apexes ahead and instance Brain.Corner from the nearest.
        void UpdateNextApexes()
        {
            Brain.Corner = null;

            int count = ARS.TrackPoints.Count;
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

            if (count < 10 || ARS.Corners.Count == 0)
            {
                heldNodes = new[] { -1, -1, -1, -1 };
                heldRadii = new[] { 999f, 999f, 999f, 999f };
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

            NextApexNode = selectedNodes.Count > 0 ? selectedNodes[0] : -1;
            NextApexRadius = selectedRadii.Count > 0 ? selectedRadii[0] : 999f;
            NextApexSpeed = NextApexNode >= 0 ? RouteIdealSpeedForRadius(NextApexRadius) : 999f;
            NextApexNode2 = selectedNodes.Count > 1 ? selectedNodes[1] : -1;
            NextApexRadius2 = selectedRadii.Count > 1 ? selectedRadii[1] : 999f;
            NextApexSpeed2 = NextApexNode2 >= 0 ? RouteIdealSpeedForRadius(NextApexRadius2) : 999f;
            NextApexNode3 = selectedNodes.Count > 2 ? selectedNodes[2] : -1;
            NextApexRadius3 = selectedRadii.Count > 2 ? selectedRadii[2] : 999f;
            NextApexSpeed3 = NextApexNode3 >= 0 ? RouteIdealSpeedForRadius(NextApexRadius3) : 999f;
            NextApexNode4 = selectedNodes.Count > 3 ? selectedNodes[3] : -1;
            NextApexRadius4 = selectedRadii.Count > 3 ? selectedRadii[3] : 999f;
            NextApexSpeed4 = NextApexNode4 >= 0 ? RouteIdealSpeedForRadius(NextApexRadius4) : 999f;

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
            _lastApexProgressNode = CurrentTrackPoint.Node;
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
            int entranceDistance = ForwardNodeDistance(entranceNode);
            int apexDistance = ForwardNodeDistance(apexNode);
            // On circuits, a passed entrance wraps to the next lap. Once the apex is
            // still ahead but its entrance is farther away, the entrance has been passed.
            float rawDistance = !ARS.IsPointToPoint && entranceDistance > apexDistance
                ? 0f
                : entranceDistance;
            if (rawDistance < 0f) rawDistance = 0f;
            float distance = rawDistance > 0f
                ? rawDistance
                    + Car.Velocity.Length() * EntranceBrakeBufferSeconds
                    + EntranceBrakeExtraDistance
                : 0f;

            float brakingAbility = Math.Min(Handling.BrakingAbility * 4, VehicleData.CurrentMechanicalGrip);
            float decel = brakingAbility * Handling.Gravity;
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
            if (Brain.Corner == null) return;
            if (_halfSecondTick < Game.GameTime)
            {
                _halfSecondTick = Game.GameTime + 500 + (int)ARS.Remap(Car.Velocity.Length(), 0, 100, -250, 250, true);
            }



            if (_oneSecondTick < Game.GameTime)
            {
                _oneSecondTick = Game.GameTime + 1000;

                if (!ControlledByPlayer)
                {
                    if (BaseBehavior == RacerBaseBehavior.Race && ARS.Racers.Count >= 1)
                    {
                        UpdateRivals();
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


                if (ARS.DevSettingsFile.GetValue<int>("RACERS", "AIRacerAutofix", 1) == 2 && Function.Call<bool>(Hash._IS_VEHICLE_DAMAGED, Car))
                {
                    Car.Repair();
                }
            }
        }




        public void ProcessAI()
        {
            ProcessTimedAI();
            UpdatePressure();

            if (BaseBehavior == RacerBaseBehavior.GridWait && Control.HandBrakeTime < Game.GameTime) Control.HandBrakeTime = Game.GameTime + (100 * ARS.GetRandomInt(2, 6));

            if (!ControlledByPlayer)
            {
                UpdateRivalInfo();

                // Re-rise the caps before concern sources pull them down.
                _accelerationCap = Math.Min(1f, _accelerationCap + 0.33f * TickScale);
                _speedCap = Math.Min(999f, _speedCap + SpeedCapRiseRate * TickScale);

                ComputeTargetSpeed();
                ComputeSteering();

                // Two-wheel stability: steer into the airborne side to regain all four wheels.
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


            float handlingGrip = Function.Call<float>((Hash)0xA132FB5370554DB0, Car) * (Handling.Gravity / 9.8f);
            handlingGrip = ARS.Clamp(handlingGrip, 0.1f, 5f);

            GroundGripMultiplier = ARS.WheelGripMultipliers(Car).Average();

            VehicleData.BaseMechanicalGrip = handlingGrip;
            VehicleData.CurrentMechanicalGrip = ((VehicleData.BaseMechanicalGrip) * GroundGripMultiplier);

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

            float zSpeedDegreesFromHoriz = (Math.Abs(VehicleData.SpeedVectorLocal.Normalized.Z) * 90);


            if (zSpeedDegreesFromHoriz > 5f)
            {
                if (VehicleData.AvgGroundStability >= 0.1f) VehicleData.AvgGroundStability -= zSpeedDegreesFromHoriz * TickScale * 0.1f;
            }
            else if (VehicleData.AvgGroundStability < 1f) VehicleData.AvgGroundStability += 1f * TickScale;

            if (VehicleData.AvgGroundStability > 1.0f) VehicleData.AvgGroundStability = 1f;

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
                for (int i = 0; i < Brain.Rivals.Count - 1; i++)
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


