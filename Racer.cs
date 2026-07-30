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
    enum CornerPhase
    {
        None, Approach, TurnIn, Hold
    }
    public class Racer
    {

        // Identity and runtime references.
        public string Name = "Racer";
        public Ped Driver;
        public Vehicle Car;
        public Team TeamRole = Team.None;
        public bool ControlledByPlayer = false;
        public RacerBaseBehavior BaseBehavior = RacerBaseBehavior.GridWait;
        public RaceState RCStatus = RaceState.NotInitiated;

        // Core controller/memory.
        public VehicleControl Control = new VehicleControl();
        public Memory Brain = new Memory();
        PID HeadingPID = new PID(1.0f, 0.6f, 0);

        // Vehicle dynamics/perception state.
        public VehData VehicleData = new VehData();
        public HandlingData Handling = new HandlingData();
        public float GroundGripMultiplier = 1f;
        Vector3 LastSpeed;

        // Navigation and lane-following state.
        public enum eLookAheads { SteerRef, QuarterSec, HalfSec, ThreeQuarterSec, OneSec, OneHalfSec, TwoSec, SteerInRef };
        public TrackPoint CurrentTrackPoint = new TrackPoint();
        public Dictionary<eLookAheads, TrackPoint> LookAheads = new Dictionary<eLookAheads, TrackPoint>();
        public List<Vehicle> Traffic = new List<Vehicle>();
        public Vector3 SteerTarget = Vector3.Zero;
        public float MaxLeftLane = 0;
        public float MaxRightLane = 0;
        public int LocalSpdLimiter = 0;

        // Race lifecycle and position.
        public List<TimeSpan> LapTimes = new List<TimeSpan>();
        public int LapStartTime = 0;
        public int Lap = 0;
        public int RacePosition = 0;
        public bool CanRegisterNewLap = true;
        public bool FinishedPointToPoint = false;

        // Timers/ticks.
        int HalfSecondTick = 0; //500ms
        int OneSecondTick = 0; //1000ms
        int LastCoreTick = 100;
        int TimeSinceLastCoreTick => (int)ARS.Clamp(Game.GameTime - LastCoreTick, 1, 9999);

        // Stuck/recovery state.
        int LastStuckGameTime = 0;
        public bool IsStuckByThrottle = false;
        const int StuckCheckTimeMs = 2000;
        bool IsRecoveringFromStuck = false;
        int StuckRecoveryEndTime = 0;
        const int StuckRecoveryTimeMs = 2000;
        int StuckRecoveryAttempts = 0;
        public int StuckRecoveryAttemptsNow => StuckRecoveryAttempts;
        public bool IsRecoveringFromStuckNow => IsRecoveringFromStuck;

        // Debug and draw caches.
        public List<string> DebugText = new List<string>();
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
        List<TrailSample> TrailSamples = new List<TrailSample>();
        List<Vector3> FollowLaneTrail = new List<Vector3>();
        List<Vector3> RawFollowLaneTrail = new List<Vector3>();
        int LastFollowLaneTrailNode = -1;

        // Misc tuning/runtime helpers.
        float TorqueMult = 1.0f;
        Random Random = new Random();

        // Corner racing line state.
        CornerPhase _cornerPhase = CornerPhase.None;
        const float CornerOverDrive = 1.2f;

        // Countersteer state (GodotRace-style).
        float _countersteerBlend = 0f;
        const float CountersteerFullSeverityMult = 3f;
        const float CountersteerRecoveryRate = 3f;

        // Avoidance state.
        bool _avoidLiftOff = false;
        float _avoidLeftWall = 0f;
        float _avoidRightWall = 0f;

        // Passengerize state: move AI driver to passenger seat when a rival is
        // too close, so GTA V doesn't force an uncontrollable swerve on contact.
        bool _isPassengerized = false;

        public Racer(Vehicle RacerCar, Ped RacerPed)
        {
            Car = RacerCar;
            Driver = RacerPed;
            try { Name = RacerCar.FriendlyName; } catch (Exception) { Name = "Racer"; }
            if (Name == "NULL" || Name == null) { try { Name = Car.DisplayName.ToString()[0].ToString().ToUpper() + Car.DisplayName.ToString().Substring(1).ToLowerInvariant(); } catch (Exception) { Name = "Racer"; } }

            if (Driver.IsPlayer) ControlledByPlayer = true;
            HalfSecondTick = Game.GameTime + (ARS.GetRandomInt(10, 50));

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
                catch (Exception) { /* blip creation failed, non-critical */ }
            }

            Function.Call(GTA.Native.Hash._0x0DC7CABAB1E9B67E, Car, true, 1); //load collision
            Function.Call(GTA.Native.Hash._0x0DC7CABAB1E9B67E, Driver, true, 1);
            Function.Call(GTA.Native.Hash.SET_ENTITY_PROOFS, Driver, true, true, true, false, true, true, 1, true);

            Driver.MaxHealth = 1000;
            Driver.Health = 1000;
            Driver.CanSufferCriticalHits = false;

            if (Car.ClassType == VehicleClass.Emergency) TeamRole = Team.Cop;

        }
        public void Initialize()
        {
            Handling.Downforce = ARS.GetDownforce(Car);
            if (Handling.Downforce > 100) Handling.Downforce *= 0.1f;

            Handling.TRlateral = ARS.rad2deg(ARS.GetTRCurveLat(Car));
            if (Handling.TRlateral < 1 || Handling.TRlateral > 100) Handling.TRlateral = 22;
            ARS.Log(ARS.LogImportance.Info, "TRlat for " + Car.DisplayName + ":" + Handling.TRlateral + "º");

            Handling.BrakingAbility = Car.MaxBraking;
            Handling.TopSpeed = ARS.EngineTopSpeed(Car);
            Handling.Power = Function.Call<float>(Hash.GET_VEHICLE_ACCELERATION, Car);


            VehicleData.SteeringLock = ARS.rad2deg(ARS.GetSteerLock(Car));
            if (VehicleData.SteeringLock < 1 || VehicleData.SteeringLock > 100) VehicleData.SteeringLock = 40;
            ARS.Log(ARS.LogImportance.Info, "Steerlock for " + Car.DisplayName + ":" + VehicleData.SteeringLock + "º");
            Control.SteerTrackDegrees = 0f;
            CurrentTrackPoint = ARS.TrackPoints.Last();
            Control.Brake = 0f;
            Control.Throttle = 0f;

            LapTimes.Clear();
            LapStartTime = 0;
            Lap = 0;
            FollowLaneTrail.Clear();
            RawFollowLaneTrail.Clear();
            LastFollowLaneTrailNode = -1;

            string flags = ARS.GetHandlingFlags(Car).ToString("X");
            int flagsHex = Convert.ToInt32(flags, 16);
            bool hasOffroad = (flagsHex & 0x800000) != 0 || (flagsHex & 0x200000) != 0;
            if (hasOffroad) Handling.Gravity *= 1.2f;

            BaseBehavior = RacerBaseBehavior.GridWait;
            FinishedPointToPoint = false;

            Handling.Grip = Function.Call<float>((Hash)0xA132FB5370554DB0, Car) * (Handling.Gravity / 9.8f);

            VehicleData.PerformanceIndex = (int)((Handling.TopSpeed * 5) + (Handling.Grip * 100) + (Handling.Power * 500));
            VehicleData.TextPerformanceIndex = ((int)(Handling.TopSpeed / 1.2) + " | " + (int)(Handling.Power * 200) + " | " + (int)(Handling.Grip * 20));

            Car.Repair();
        }

        public void SteerTrack()
        {
            if (!TryGetSteerContext(out TrackPoint steerRefPoint, out float roadWide))
            {
                Control.SteerTrackDegrees = 0f;
                return;
            }

            // 1) Heading error: use velocity vector (where car is actually going),
            // not model forward (where car is pointing). Matches GodotRace.
            Vector3 carForward = Car.Velocity.LengthSquared() > 0.01f
                ? Car.Velocity.Normalized
                : Car.ForwardVector;
            float headingErrorDeg = -Vector3.SignedAngle(
                steerRefPoint.Direction, carForward, Vector3.WorldUp);
            if (float.IsNaN(headingErrorDeg) || float.IsInfinity(headingErrorDeg))
                headingErrorDeg = 0f;

            // Scale down the heading error so steering response is gentler.
            headingErrorDeg *= 0.5f;

            float speedMps = Math.Max(Car.Velocity.Length(), 1f);
            float carHalfWidth = VehicleData.BoundingBox * 0.5f;

            // 2) Compute target lane from corner bias or zero (no centering).
            float naturalLane;
            bool cornerActive = Brain.Corner != null && Brain.Corner.Valid && Lap > 0;
            if (cornerActive)
            {
                naturalLane = ComputeCornerTargetLane(steerRefPoint, speedMps);
            }
            else
            {
                // No centering nudge — car follows heading error and avoidance only.
                naturalLane = 0f;
            }

            // 3) Rival avoidance: clamp target lane away from rival's forbidden zone.
            // Only acts if the natural lane falls inside the zone.
            float clampedLane = ClampTargetLaneForAvoidance(naturalLane, roadWide, carHalfWidth);

            // 4) Off-track recovery: when car's edge exceeds track width, steer back.
            float recoveryDeg = 0f;
            float absDev = Math.Abs(Brain.data.DeviationFromCenter);
            float safeEdge = roadWide - carHalfWidth;
            float overshoot = absDev - safeEdge;
            if (overshoot > 0f)
            {
                float maxRecoveryDeg = ARS.map(speedMps, 10f, 50f, 10f, 2f);
                float severity = Math.Min(overshoot / Math.Max(safeEdge, 1f), 1f);
                recoveryDeg = Math.Sign(Brain.data.DeviationFromCenter) * maxRecoveryDeg * severity;
            }

            // 5) Convert target lane to a steering bias using the steering lookahead.
            // Only active when a corner or avoidance wall is constraining the lane.
            float laneBiasDeg = 0f;
            float trackBound = roadWide - carHalfWidth;
            bool hasActiveGuidance = cornerActive
                || _avoidLeftWall > -trackBound
                || _avoidRightWall < trackBound;
            if (hasActiveGuidance)
            {
                float currentLane = Brain.data.DeviationFromCenter;
                float lookaheadDist = steerRefPoint.Position.DistanceTo(Car.Position);
                if (lookaheadDist < 1f) lookaheadDist = speedMps * 1.5f;

                // Single bias from the clamped lane (respects walls).
                float laneError = clampedLane - currentLane;
                laneBiasDeg = -(float)(Math.Atan2(laneError, lookaheadDist) * (180.0 / Math.PI));
                laneBiasDeg *= 0.25f;
            }

            // 6) Total heading target = heading error + lane bias + recovery.
            float totalTargetDeg = headingErrorDeg + laneBiasDeg + recoveryDeg;

            // 7) GodotRace-style PD: P on heading error, D on yaw rate (subtracted).
            const float steerKP = 1.0f;
            float steerKD = 0.35f / VehicleData.CurrentMechanicalGrip;
            float pTerm = steerKP * totalTargetDeg;
            float dTerm = steerKD * VehicleData.YawRotationPerSecondDegrees;
            Control.SteerTrackDegrees = pTerm - dTerm;

            // 8) Slide countersteer: when sliding and yawing in the same direction,
            // steer opposite the slide to catch it.
            if (Math.Sign((int)VehicleData.SlideAngle) == Math.Sign((int)VehicleData.YawRotationPerSecondDegrees))
            {
                float slideCounterSteer = VehicleData.SlideAngle * ARS.map(
                    Math.Abs(VehicleData.SlideAngle), 0, Handling.TRlateral * 1.2f, 0.5f, 1.2f, true);
                Control.SteerTrackDegrees -= slideCounterSteer;
            }

            // Keep SteerTarget valid for debug drawing.
            SteerTarget = steerRefPoint.Position;

            // ── Local helpers ──────────────────────────────────────────────
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

                if (!LookAheads.TryGetValue(eLookAheads.SteerRef, out localSteerRef)
                    || localSteerRef == null)
                {
                    return false;
                }

                localRoadWide = localSteerRef.TrackWide;
                return true;
            }
        }

        /// <summary>
        /// Computes the target lane offset (meters from center) for the racing line.
        /// Three phases: Approach (outside) → TurnIn (apex) → Hold (keep apex).
        /// Positive = right of center, negative = left of center.
        /// </summary>
        float ComputeCornerTargetLane(TrackPoint steerRefPoint, float speedMps)
        {
            CornerPoint c = Brain.Corner.OG;
            int apexNode = c.Node;

            // Corner direction: SignedAngle(pre, fut, up) is negative for right turns.
            // cornerDir = -1 for right turn, +1 for left turn.
            float cornerDir = Math.Sign(c.Angle);

            // For a right turn (cornerDir = -1):
            //   Outside = left  = -halfWidth =  cornerDir * halfWidth
            //   Apex    = right = +halfWidth = -cornerDir * halfWidth
            // For a left turn (cornerDir = +1):
            //   Outside = right = +halfWidth =  cornerDir * halfWidth
            //   Apex    = left  = -halfWidth = -cornerDir * halfWidth
            if (cornerDir == 0f) return 0f;

            float halfWidth = steerRefPoint.TrackWide;
            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            float safeBound = halfWidth - carHalfWidth;

            // Always-on inside pull: intensity mapped from curve radius.
            float curveRadius = Math.Abs(CurrentTrackPoint.PreciseCurveRadius);
            float insideIntensity = ARS.map(curveRadius, 50f, 300f, 1f, 0.1f, true);
            insideIntensity = ARS.Clamp(insideIntensity, 0f, 1f);
            float insideTarget = -cornerDir * safeBound * insideIntensity;

            float distToApexNodes = Math.Abs(apexNode - CurrentTrackPoint.Node);
            float timeToApex = distToApexNodes / Math.Max(speedMps, 1f);

            float targetLane;
            const float approachStartTime = 5.0f;
            const float lerpStartTime = 2.0f;
            const float turnInTime = 1.0f;

            if (CurrentTrackPoint.Node >= apexNode)
            {
                _cornerPhase = CornerPhase.Hold;
                targetLane = insideTarget;
            }
            else if (timeToApex <= turnInTime)
            {
                _cornerPhase = CornerPhase.TurnIn;
                targetLane = insideTarget;
            }
            else if (timeToApex <= lerpStartTime)
            {
                // 2s→1s before apex: lerp from outside to inside.
                _cornerPhase = CornerPhase.Approach;
                float outsideTarget = cornerDir * safeBound;
                float t = (timeToApex - turnInTime) / (lerpStartTime - turnInTime);
                targetLane = insideTarget + (outsideTarget - insideTarget) * t;
            }
            else if (timeToApex <= approachStartTime)
            {
                // 5s→2s before apex: hold outside.
                _cornerPhase = CornerPhase.Approach;
                targetLane = cornerDir * safeBound;
            }
            else
            {
                _cornerPhase = CornerPhase.None;
                targetLane = insideTarget;
            }

            return targetLane;
        }

        /// <summary>
        /// Clamps a target lane offset to avoid rivals.
        /// Maintains independent left and right walls from all nearby rivals.
        /// Left wall = highest (most restrictive) from all left-side rivals.
        /// Right wall = lowest (most restrictive) from all right-side rivals.
        ///
        /// Walls slowly open toward track edges at 2m/s when not constrained,
        /// preventing snap-back when a rival moves away.
        ///
        /// If the resulting range is narrower than our car width + 1m,
        /// sets _avoidLiftOff to true so SpeedTrack can lift off.
        /// </summary>
        float ClampTargetLaneForAvoidance(float targetLane, float roadWide, float carHalfWidth)
        {
            _avoidLiftOff = false;

            float trackBound = roadWide - carHalfWidth;

            // Compute target walls from all rivals.
            float targetLeftWall = -trackBound;
            float targetRightWall = trackBound;
            bool leftConstrained = false;
            bool rightConstrained = false;

            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null) continue;

                bool isRelevant = false;
                bool rivalIsLeft = false;

                if (r.relativePos == RelativePos.Left)
                {
                    isRelevant = true;
                    rivalIsLeft = true;
                }
                else if (r.relativePos == RelativePos.Right)
                {
                    isRelevant = true;
                    rivalIsLeft = false;
                }
                else if (r.relativePos == RelativePos.Ahead && r.sToReach >= 0f && r.sToReach <= 4f)
                {
                    isRelevant = true;
                    rivalIsLeft = r.OccupiedLane < Brain.data.DeviationFromCenter;
                }

                if (!isRelevant) continue;

                float rivalBuffer = r.OccupiedLaneWidth * 0.5f;

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

            // Each side independently: snap when constrained, decay to track edge when clear.
            float openRate = 2f * TickScale; // 2m/s, framerate independent
            if (leftConstrained)
                _avoidLeftWall = targetLeftWall;
            else
                _avoidLeftWall = Math.Max(_avoidLeftWall - openRate, -trackBound);

            if (rightConstrained)
                _avoidRightWall = targetRightWall;
            else
                _avoidRightWall = Math.Min(_avoidRightWall + openRate, trackBound);

            // Check if there's room for our car.
            float carTotalWidth = carHalfWidth * 2f + 1f;
            if (_avoidRightWall - _avoidLeftWall < carTotalWidth)
                _avoidLiftOff = true;

            // Clamp target lane so our car's edge (not center) stays at the wall.
            float clampLeft = _avoidLeftWall + carHalfWidth;
            float clampRight = _avoidRightWall - carHalfWidth;
            return ARS.Clamp(targetLane, clampLeft, clampRight);
        }

        void SteerApplyCorrections()
        {
            // Speed-based steering limiter: only when turning in (steer and yaw same sign),
            // not when countersteering (opposite signs).
            if (Math.Sign(Control.SteerTrackDegrees) == Math.Sign((int)VehicleData.YawRotationPerSecondDegrees))
            {
                float speedBasedSteeringLimit = (float)((VehicleData.BaseMechanicalGrip * Handling.Gravity * VehicleData.WheelBase) / Math.Pow(Car.Velocity.Length() + 0.01f, 2.01f));
                speedBasedSteeringLimit = Math.Max(ARS.rad2deg(speedBasedSteeringLimit), 3f);
                Control.SteerTrackDegrees = ARS.Clamp(Control.SteerTrackDegrees, -speedBasedSteeringLimit, speedBasedSteeringLimit);
            }
        }

        /// <summary>
        /// </summary>
        /// <returns></returns>
        float DistToOutside(Vector3 direction)
        {
            if (Vector3.SignedAngle(CurrentTrackPoint.Direction, direction, Vector3.WorldUp) < 0.0f) return (float)Math.Round(CurrentTrackPoint.TrackWide + Brain.data.DeviationFromCenter, 1);
            else return (float)Math.Round(CurrentTrackPoint.TrackWide - Brain.data.DeviationFromCenter, 1);
        }

        float DistToInside(Vector3 direction)
        {
            if (Vector3.SignedAngle(CurrentTrackPoint.Direction, direction, Vector3.WorldUp) > 0.0f) return (float)Math.Round(CurrentTrackPoint.TrackWide + Brain.data.DeviationFromCenter, 1);
            else return (float)Math.Round(CurrentTrackPoint.TrackWide - Brain.data.DeviationFromCenter, 1);
        }

        public void Launch()
        {
            Brain.Corner = new Corner(5, ARS.CornerPoints.FirstOrDefault(c => c.IsKey));
            Brain.Corner.Valid = false;
            VehicleData.AvgGroundStability = 1;
            BaseBehavior = RacerBaseBehavior.Race;
            LapStartTime = Game.GameTime;
            Control.HandBrakeTime = Game.GameTime + ARS.GetRandomInt(100, 400);
            Control.MaxThrottle = 1f;
            IsStuckByThrottle = false;
            LastStuckGameTime = 0;
            IsRecoveringFromStuck = false;
            StuckRecoveryEndTime = 0;
            StuckRecoveryAttempts = 0;
            Control.LastAppliedSteerTrackDegrees = 0f;
            HeadingPID.SetValue(0f);
            FollowLaneTrail.Clear();
            RawFollowLaneTrail.Clear();
            LastFollowLaneTrailNode = -1;
            if (TeamRole == Team.Cop) Car.SirenActive = true;

        }

        void SpeedToThrottleBrake()
        {
            float newThrottle = 0f;
            float newBrake = 0f;
            float throttleCap = Math.Min(Control.TCSThrottle, 1f);
            float dirSwitchSpeed = ARS.MPHtoMS(5f);

            //Keep still with throttle up when waiting for the launch
            if (BaseBehavior == RacerBaseBehavior.GridWait)
            {
                Brain.intention.Speed = 99f;
                Control.HandBrakeTime = Game.GameTime + 500;
            }


            //Limit the th the input to the car's top speed. Cars can surpass defined engine top speed in V
            if (Brain.intention.Speed >= 0f)
            {
                Brain.intention.Speed = Math.Min(Brain.intention.Speed, ARS.EngineTopSpeed(Car) * 1.3f);
                Brain.intention.Speed = Math.Min(Brain.intention.Speed, Brain.intention.MaxSpeed);
            }

            if ((Game.GameTime - LapStartTime) < 3000) Brain.intention.IntendedSpdChangeGs = 999;
            else
            {
                float currentLongitudinalSpeed = VehicleData.SpeedVectorLocal.Y;
                Brain.intention.IntendedSpdChangeGs = (Brain.intention.Speed - currentLongitudinalSpeed) / 9.8f;
            }            

            float currentForwardSpeed = VehicleData.SpeedVectorLocal.Y;
            float speedErrorGs = Brain.intention.IntendedSpdChangeGs;
            bool wantsReverse = Brain.intention.Speed < -0.1f;

            if (speedErrorGs > 0.0f)
            {
                // If the car is still rolling backward, stop it first before applying forward throttle.
                if (currentForwardSpeed < -dirSwitchSpeed) newBrake = ARS.Clamp(speedErrorGs * 2f, 0f, 1f);
                else newThrottle = ARS.Clamp(speedErrorGs * 2f, 0f, throttleCap);
            }
            else if (speedErrorGs < 0.0f)
            {
                float reverseDemand = ARS.Clamp((-speedErrorGs) * 2f, 0f, throttleCap);
                float brakeDemand = ARS.Clamp((-speedErrorGs) * 2f, 0f, 1f);

                if (wantsReverse)
                {
                    // Transition to reverse: brake while moving forward, then release brake and apply negative throttle.
                    if (currentForwardSpeed > dirSwitchSpeed) newBrake = brakeDemand;
                    else newThrottle = -reverseDemand;
                }
                else
                {
                    newBrake = brakeDemand;
                }
            }


 
            if (newBrake > 0.0) newThrottle = 0;
            if (Math.Abs(newThrottle) > 0.0f) newBrake = 0;


            float stabilityThrottleLimit = VehicleData.AvgGroundStability;
            if (OutOfTrackDistance() > 0.5f)
            {
                // Off-track recovery needs some minimum throttle authority.
                stabilityThrottleLimit = Math.Max(stabilityThrottleLimit, 0.45f);
            }
            //Control.MaxThrottle = Math.Min(Control.MaxThrottle, stabilityThrottleLimit);

            Control.Brake += (newBrake - Control.Brake) * 5 * TickScale;
            Control.Throttle += (newThrottle - Control.Throttle) * 5 * TickScale;
            Control.Throttle = Math.Min(Control.Throttle, Control.MaxThrottle);
            if (Control.MaxThrottle < 1.00f) Control.MaxThrottle += 2 * TickScale;

            if (Brain.intention.MaxSpeed < AIData.MaxSpeed) Brain.intention.MaxSpeed += 15 * TickScale;

        }
        float TickScale => (0.001f * TimeSinceLastCoreTick);


        /// <summary>
        /// </summary>
        void SteerTranslateInput()
        {

            if (float.IsNaN(Control.SteerTrackDegrees) || float.IsInfinity(Control.SteerTrackDegrees)) Control.SteerTrackDegrees = 0f;
            if (float.IsNaN(Control.SteerManeuver) || float.IsInfinity(Control.SteerManeuver)) Control.SteerManeuver = 0f;

            
            float maxDeltaPerTick = 90 * TickScale;
            float deltaToTarget = ARS.Clamp(Control.SteerTrackDegrees - Control.LastAppliedSteerTrackDegrees, -maxDeltaPerTick, maxDeltaPerTick);
             
            Control.SteerTrackDegrees = Control.LastAppliedSteerTrackDegrees+ deltaToTarget;

            Control.LastAppliedSteerTrackDegrees = Control.SteerTrackDegrees;

            if (float.IsNaN(Control.SteerInput) || float.IsInfinity(Control.SteerInput)) Control.SteerInput = 0f;

            Control.SteerInput = ARS.map(Control.SteerTrackDegrees, -VehicleData.SteeringLock, VehicleData.SteeringLock, -1, 1, true);
            
        }

        /// <summary>
        /// Figures out the ideal speed to be at at the moment
        /// </summary>
        public void SpeedTrack()
        {
            if (BaseBehavior == RacerBaseBehavior.GridWait)
            {
                Brain.intention.Speed = 200f;
                return;
            }
            if (BaseBehavior == RacerBaseBehavior.FinishedRace)
            {
                Brain.intention.Speed = 20f;
                return;
            }
            if (BaseBehavior == RacerBaseBehavior.FinishedStandStill)
            {
                Brain.intention.Speed = 0f;
                return;
            }
            Brain.intention.Speed = AIData.MaxSpeed;

            float cornerSpd = 999f;
            if (Brain.Corner.Valid) cornerSpd = Math.Max(2, ARS.MapIdealSpeedForDistance(Brain.Corner.OG, this) * CornerOverDrive);

            float followRadius = Math.Max(Brain.data.CurveRadiusToFollowPoint, 0.1f);
            float followTrackSpd =(float)Math.Sqrt((VehicleData.CurrentMechanicalGrip * Handling.Gravity) * followRadius);            

            
            TrackPoint followMidpoint = ARS.TrackPoints[(int)((CurrentTrackPoint.Node + Brain.data.FollowPoint.Node) * 0.5f)];
            float hillGsDelta = ARS.GripGainLossElChange(CurrentTrackPoint.Position, followMidpoint.Position,  Brain.data.FollowPoint.Position, followTrackSpd);
            hillGsDelta= ARS.Clamp(hillGsDelta*0.5f, -0.3f, 0.3f);
            float adjustedFollowGrip = Math.Max(0.1f, VehicleData.CurrentMechanicalGrip + hillGsDelta);
            followTrackSpd=(float)Math.Sqrt((adjustedFollowGrip * Handling.Gravity) * followRadius);

            if (float.IsNaN(cornerSpd) || float.IsInfinity(cornerSpd)) cornerSpd = 999f;
            if (float.IsNaN(followTrackSpd) || float.IsInfinity(followTrackSpd)) followTrackSpd = 999f;
            if (cornerSpd <= 5) cornerSpd = ARS.GetSpeedForCorner(Brain.Corner.OG, this);
              
            Brain.intention.Speed = Math.Min(cornerSpd, followTrackSpd);

            // Avoidance lift-off: if there's no room to pass, match rival's speed.
            if (_avoidLiftOff)
            {
                Rival threat = Brain.Rivals.FirstOrDefault(r => r.RivalRacer != null && r.relativePos == RelativePos.Ahead);
                if (threat != null)
                {
                    float rivalSpeed = threat.RivalRacer.Car.Velocity.Length();
                    Brain.intention.Speed = Math.Min(Brain.intention.Speed, rivalSpeed);
                }
            }
#if false
            // Steer-angle speed limiter — disabled. Kept for reference.
            float maxSpeedForSteerAngle = AIData.MaxSpeed;
            float steerAngleDegAbs = Math.Abs(Control.SteerTrackDegrees);
            float steerAngleDegClamped = ARS.Clamp(steerAngleDegAbs, 0f, Math.Max(VehicleData.SteeringLock, 1f));
            float steerAngleRadAbs = Math.Abs(ARS.deg2rad(steerAngleDegClamped));
            bool steerAndSlideSameSign = Math.Sign(Control.SteerTrackDegrees) == Math.Sign(VehicleData.SlideAngle);
            if (steerAndSlideSameSign && steerAngleRadAbs > ARS.deg2rad(0.5f) && VehicleData.WheelBase > 0)
            {
                float tanSteer = (float)Math.Tan(steerAngleRadAbs);
                if (Math.Abs(tanSteer) > 0.0001f)
                {
                    float steerCurveRadius = Math.Abs(VehicleData.WheelBase / tanSteer);
                    if (!float.IsNaN(steerCurveRadius) && !float.IsInfinity(steerCurveRadius) && steerCurveRadius > 0f)
                    {
                        maxSpeedForSteerAngle = (float)Math.Sqrt((adjustedFollowGrip * Handling.Gravity) * steerCurveRadius);
                    }
                }
            }
            if (float.IsNaN(maxSpeedForSteerAngle) || float.IsInfinity(maxSpeedForSteerAngle)) maxSpeedForSteerAngle = AIData.MaxSpeed;
            Brain.intention.Speed = Math.Min(Brain.intention.Speed, maxSpeedForSteerAngle);
#endif
            // Extra limiter for cars pointing outward in a corner.
            if (Brain.Corner.Valid &&1==2)
            {           
                float angleToTrack = Vector3.SignedAngle(LookAheads[eLookAheads.HalfSec].Direction, Car.ForwardVector, Vector3.WorldUp);
                if (Math.Abs(angleToTrack) > 0.0f)
                {
                    float cornerDir = Brain.Corner.OG.Angle;
                    bool goingOutward = Math.Abs(cornerDir) > 0.01f && Math.Sign(angleToTrack) != Math.Sign(cornerDir);
                    float outwardThresholdDeg = ARS.map(DistToOutside(CurrentTrackPoint.Direction), CurrentTrackPoint.TrackWide, CurrentTrackPoint.TrackWide*2, 0,45,true);

                    if (goingOutward && Math.Abs(angleToTrack) > outwardThresholdDeg)
                    {
                        float speedReduction = (Math.Abs(angleToTrack) - outwardThresholdDeg) / 5f;
                        Brain.intention.MaxSpeed = Math.Max(followTrackSpd-10, followTrackSpd - speedReduction);                        
                    }
                }
            }


            // Risk factor for grip. The more we are at the limit, the more we should be careful with our speed target.
            // NOT active.
            if (Math.Sign((int)Control.SteerTrackDegrees) == Math.Sign((int)VehicleData.YawRotationPerSecondDegrees))
            {
                float velocity = Car.Velocity.Length();

                // Simple approximation (angle must be in RADIANS)
                float steeringAngleRad = Math.Abs(Control.SteerTrackDegrees * 1f) * (float)(Math.PI / 180);
                float intendedYawRate = (velocity * (float)Math.Tan(steeringAngleRad)) / VehicleData.WheelBase;
                intendedYawRate *= (180.0f / (float)Math.PI); // Convert to degrees/sec

                float maxYawRateFromGrip = ((VehicleData.CurrentMechanicalGrip * Handling.Gravity) / velocity) * (180.0f / (float)Math.PI);
                float speedAdjust = (Math.Abs(maxYawRateFromGrip * 0.99f) - Math.Abs(intendedYawRate)) / 10;
                if (float.IsNaN(speedAdjust)) speedAdjust = 1;
            }

        }

        /// <summary>
        /// Limits Control.Throttle and Control.Brake inputs to avoid wheelspin and lockups.
        /// </summary>
        void TractionControl()
        {
            if (Control.Throttle <= 0.0f) return;

            float wheelspin = ARS.GetWheelsMaxWheelspin(Car);
            bool lowGripOrLowGear = GroundGripMultiplier < 0.9f || Car.CurrentGear < 2;

            //float allowedWheelspin = lowGripOrLowGear ? 0.8f : 0.2f;
            float allowedWheelspin = 0.1f;

            float tcsValue = ARS.map(Math.Abs(wheelspin) - allowedWheelspin, 0.1f, -0.1f, -1f, 1f, true) * 8;
            float change = tcsValue * TickScale;
            Control.TCSThrottle = ARS.Clamp(Control.TCSThrottle + change, 0.2f, 1);
        }
        public void UpdateTickData()
        {
            VehicleData.LocalGs = (Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true) - VehicleData.SpeedVectorLocal) / Game.LastFrameTime;
            Vector3 cSpeed = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, false);

            VehicleData.AccelerationVector.Add((cSpeed - LastSpeed) / Game.LastFrameTime);

            if (VehicleData.AccelerationVector.Count > 10) VehicleData.AccelerationVector.RemoveAt(0);

            LastSpeed = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, false);
            VehicleData.SpeedVectorGlobal = cSpeed;
            VehicleData.SpeedVectorLocal = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true);
            Brain.data.SpeedVector = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true);

            while (TrailSamples.Count > 50) TrailSamples.RemoveAt(0);
        }
        /// <summary>
        /// Gathers and runs tick-sensitive stuff
        /// </summary>
        public void ProcessTick()
        {
            UpdateTickData();
            DrawStuff();

            if (!Driver.IsPlayer)
            {
                TorqueMult = ARS.map(Math.Abs(VehicleData.SlideAngle), 5f, 90f, 1f, 10);

                ApplyInputs();


                //Catchup
                if (ARS.Racers.Count > 1)
                {
                    if (RacePosition > ARS.catchupPos && 1 == 0)
                    {
                        if (!ARS.SettingsFile.GetValue("CATCHUP", "OnlyLoners", true) || !Brain.Rivals.Any(v => v.Distance < 50))
                        {
                            if (ARS.SettingsFile.GetValue("CATCHUP", "CatchupSpeed", 100) != 0 && Control.HandBrakeTime < Game.GameTime)
                            {
                                if (ARS.GetPercent(RacePosition, ARS.Racers.Count) >= 40)//&& !NearbyRivals.Any()
                                {
                                    float max = (float)Math.Round((float)ARS.SettingsFile.GetValue("CATCHUP", "CatchupSpeed", 100) / 1000, 2);
                                    Car.ApplyForceRelative(new Vector3(0, ARS.Clamp(Control.Throttle, -max, max), 0));
                                }
                            }
                            if (ARS.SettingsFile.GetValue("CATCHUP", "CatchupGrip", 100) != 0)
                            {
                                float max = (float)Math.Round((float)ARS.SettingsFile.GetValue("CATCHUP", "CatchupGrip", 100) / 1000, 2);
                                float v = ARS.Clamp(-Brain.data.SpeedVector.X, -max, max);
                                Car.ApplyForceRelative(new Vector3(v, 0, 0));
                            }
                        }
                    }
                }
            }
        }
        public void RunTimedCore()
        {
            UpdateFollowTrack();
            UpdateDynamicBoundingBox();
            UpdatePercievedGrip();
            UpdateCornerInfo();

            ProcessAI();
            if (Driver.IsPlayer && ARS.SettingsFile.GetValue("CATCHUP", "OnlyBehindPlayer", true)) ARS.catchupPos = RacePosition;

            LastCoreTick = Game.GameTime;
        }
        /// <summary>
        /// Widens this car's percieved boundingbox if its sliding, to cover its length instead of width.
        /// </summary>
        void UpdateDynamicBoundingBox()
        {
            VehicleData.BoundingBox = ARS.GetDirectionalBoundingBox(Car);
            VehicleData.SlideAngle = (float)Math.Round(Vector3.SignedAngle(Car.Velocity.Normalized, Car.ForwardVector, Car.UpVector), 3);
        }
        void UpdateCornerInfo()
        {
            if (Brain.Corner == null) return;

            if (Brain.Corner.Valid && Lap > 0)
            {
                int cornerEndNode = (int)ARS.Clamp(Brain.Corner.OG.Node + Brain.Corner.OG.LenghtEnd, 0, ARS.TrackPoints.Count - 1);
                if (CurrentTrackPoint.Node > cornerEndNode || (Math.Abs(CurrentTrackPoint.Node - Brain.Corner.OG.Node) > 1000))
                {
                    Brain.Corner.Valid = false;
                 }
            }

            // Rival detection only — no avoidance actions.
        }

        /// <summary>
        /// Moves the AI driver to the passenger seat when a rival is within
        /// combined half-length + 0.5m, preventing GTA V's forced swerve on
        /// contact. Returns the driver to the driver seat once clear.
        /// Never touches the player.
        /// </summary>
        void UpdatePassengerize()
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

            //AI Inputs
            if (Driver.IsSittingInVehicle(Car) && !Driver.IsPlayer)
            {
                UpdatePassengerize();

                if (Control.HandBrakeTime > Game.GameTime) Car.HandbrakeOn = true; else Car.HandbrakeOn = false;

                ARS.SetThrottle(Car, ARS.Clamp(Control.Throttle, -1, 1));
                ARS.SetBrakes(Car, Control.Brake);
                ARS.SetSteerAngle(Car, Control.SteerInput);

            }
            else
            {
                ARS.SetThrottle(Car, 0f);
                ARS.SetBrakes(Car, 0f);
                ARS.SetSteerInput(Car, 0f);
            }
        }
        void DrawStuff()
        {
            bool showAggro = ARS.OptionValuesList[Options.ShowAggro];
            bool showInputs = ARS.OptionValuesList[Options.ShowInputs];
            bool showTrack = ARS.OptionValuesList[Options.ShowTrackAnalysis];
            bool showPhysics = ARS.OptionValuesList[Options.ShowPhysics];
            bool showAny = showAggro || showTrack || showPhysics;
            if (!showAny) return;

            if ((Car.Position - Game.Player.Character.Position).Length() > 50) return;


            if (showTrack && Driver.IsPlayer && Lap >= ARS.SettingsFile.GetValue<int>("GENERAL_SETTINGS", "Laps", 5) && CanRegisterNewLap)
            {
                World.DrawMarker(MarkerType.CheckeredFlagRect, ARS.TrackPoints.First().Position + new Vector3(0, 0, 5f), ARS.TrackPoints.First().Direction, new Vector3(0, 0, 0), new Vector3(5f, 5f, 5f), Color.White);// DrawLine(vm,last, Color.Black);
            }


            if (showPhysics)
            {

                //Center of Gs
                World.DrawMarker(MarkerType.DebugSphere, Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.1f, 0.1f, 0.1f), Color.Green, false, false, 0, false, "", "", false);

                //Gs
                Vector3 avgGs = VehicleData.AccelerationVector.Aggregate(new Vector3(0, 0, 0), (s, v) => s + v) / (float)VehicleData.AccelerationVector.Count;
                avgGs.Z = 0f;

                float colorPercent = ARS.map(avgGs.Length() / 9.8f, 0, VehicleData.CurrentMechanicalGrip, 0, 100, true);
                Color gColor = ARS.GradientAtoBtoC(Color.White, Color.Yellow, Color.Red, colorPercent);

                World.DrawMarker(MarkerType.DebugSphere, Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)) + (avgGs / 9.8f), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.15f, 0.15f, 0.15f), gColor, false, false, 0, false, "", "", false);
                ARS.DrawLine(Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)) + (avgGs / 9.8f), Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)), gColor);


                Vector3 maxValues = new Vector3(VehicleData.CurrentMechanicalGrip * 9.8f, VehicleData.CurrentMechanicalGrip * 9.8f, VehicleData.CurrentMechanicalGrip * 9.8f);
                Vector3 max = Vector3.Clamp(avgGs, -maxValues, maxValues);



                Vector3 source = Car.Position + new Vector3(0, 0, 0.5f + (Car.Model.GetDimensions().Z * 0.6f));
                ARS.DrawText(source, "~b~" + Math.Round(ARS.MStoMPH(Car.Velocity.Length())).ToString() + "~w~mph~n~~y~" + (avgGs.Length() / 9.8f).ToString("0.0") + " Gs", Color.White, 0.5f);


            }

            if (!Driver.IsPlayer)
            {

                if (!Car.IsInRangeOf(Game.Player.Character.Position, 500)) return;
                if (showTrack)
                {
                   
                }

                if (showAggro)
                {
                    // Draw avoidance walls on the ground, relative to track center.
                    Vector3 trackCenter = CurrentTrackPoint.Position;
                    Vector3 trackRight = Vector3.Cross(CurrentTrackPoint.Direction, Vector3.WorldUp);
                    Vector3 leftWallPos = trackCenter + trackRight * _avoidLeftWall;
                    Vector3 rightWallPos = trackCenter + trackRight * _avoidRightWall;
                    Vector3 up = new Vector3(0, 0, 0.1f);
                    ARS.DrawLine(leftWallPos + up, leftWallPos + up + new Vector3(0, 0, 2f), Color.Blue);
                    ARS.DrawLine(rightWallPos + up, rightWallPos + up + new Vector3(0, 0, 2f), Color.Red);

                    // Debug: show wall values and lane info.
                    Vector3 textPos = Car.Position + new Vector3(0, 0, 2f);
                    ARS.DrawText(textPos, "~w~My lane: ~b~" + Brain.data.DeviationFromCenter.ToString("0.0") + " ~w~L: ~b~" + _avoidLeftWall.ToString("0.0") + " ~w~R: ~r~" + _avoidRightWall.ToString("0.0"), Color.White, 0.4f);
                    int ri = 0;
                    foreach (Rival r in Brain.Rivals)
                    {
                        if (r.RivalRacer == null) continue;
                        Vector3 rTextPos = Car.Position + new Vector3(0, 0, 2.5f + ri * 0.4f);
                        string side = r.relativePos == RelativePos.Left ? "L" : r.relativePos == RelativePos.Right ? "R" : r.relativePos == RelativePos.Ahead ? "A" : "?";
                        ARS.DrawText(rTextPos, "~w~R" + ri + "(" + side + ") lane: ~r~" + r.OccupiedLane.ToString("0.0") + " s: ~y~" + r.sToReach.ToString("0.0"), Color.White, 0.4f);
                        ri++;
                    }
                }

                if (showTrack)
                {
                    Vector3 source = Car.Position + new Vector3(0, 0, 0.5f + (Car.Model.GetDimensions().Z * 0.6f));


                    if (Brain.Corner.Valid && Lap > 0)
                    {
                        CornerPoint c = Brain.Corner.OG;
                        {

                            Vector3 wp = ARS.Path[c.Node];
                            float expectedSpeed = c.Speed;
                            Color gColor = ARS.GetColorFromRedYellowGreenGradient(ARS.map(expectedSpeed - Car.Velocity.Length(), -1, 1, 0, 100, true));


                            // Draw configured node offsets for the active corner (from NodeScalarData), mapped onto the track.
                            int startNode = (int)ARS.Clamp(c.Node - c.LengthStart, 0, ARS.TrackPoints.Count - 1);
                            int endNode = (int)ARS.Clamp(c.Node + c.LenghtEnd, 0, ARS.TrackPoints.Count - 1);
                            Vector3 oldOffsetPos = Vector3.Zero;
                            for (int node = startNode; node <= endNode; node++)
                            {
                                TrackPoint tNode = ARS.TrackPoints[node];
                                float laneOffset = 0f;
                                if (ARS.NodeScalarData.ContainsKey(node)) laneOffset = ARS.NodeScalarData[node];

                                float maxLane = Math.Max(0f, tNode.TrackWide - (VehicleData.BoundingBox / 2f));
                                laneOffset = ARS.Clamp(laneOffset, -maxLane, maxLane);

                                Vector3 offsetPos = tNode.Position - (Vector3.Cross(Vector3.WorldUp, tNode.Direction) * laneOffset) + (Vector3.WorldUp * 0.35f);
                                if (oldOffsetPos != Vector3.Zero) ARS.DrawLine(oldOffsetPos, offsetPos, Color.Blue);
                                oldOffsetPos = offsetPos;
                            }


                            World.DrawMarker(MarkerType.ChevronUpx1, wp, ARS.TrackPoints[c.Node].Direction, new Vector3(90, 0, 0), new Vector3(ARS.TrackPoints[c.Node].TrackWide * 2.5f, 5, 5), Color.FromArgb(50, gColor.R, gColor.G, gColor.B));
                            if (c.Node - c.LengthStart > 5)
                            {
                                World.DrawMarker(MarkerType.ChevronUpx1, ARS.TrackPoints[c.Node - c.LengthStart].Position, ARS.TrackPoints[c.Node - c.LengthStart].Direction, new Vector3(90, 0, 0), new Vector3(ARS.TrackPoints[c.Node].TrackWide * 2.5f, 5, 5), Color.FromArgb(50, gColor.R, gColor.G, gColor.B));
                                ARS.DrawLine(ARS.TrackPoints[c.Node - c.LengthStart].Position, ARS.TrackPoints[c.Node].Position, gColor);
                            }
                            if (c.Node + c.LenghtEnd < ARS.TrackPoints.Count - 5)
                            {
                                ARS.DrawLine(ARS.TrackPoints[c.Node + c.LenghtEnd].Position, ARS.TrackPoints[c.Node].Position, gColor);
                                World.DrawMarker(MarkerType.ChevronUpx1, ARS.TrackPoints[c.Node + c.LenghtEnd].Position, ARS.TrackPoints[c.Node + c.LenghtEnd].Direction, new Vector3(90, 0, 0), new Vector3(ARS.TrackPoints[c.Node].TrackWide * 2.5f, 5, 5), Color.FromArgb(50, gColor.R, gColor.G, gColor.B));
                            }

                            float followSpd = c.Speed;
                            float percent = ARS.map(ARS.MStoMPH(followSpd - Car.Velocity.Length()), -10, 10, 0, 100, true);
                            Color color = ARS.GradientAtoBtoC(Color.Red, Color.Yellow, Color.White, percent);
                            World.DrawMarker(MarkerType.DebugSphere, ARS.TrackPoints[c.Node].Position + Vector3.WorldUp * 2, Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.5f, 0.5f, 0.5f), color);

                        }
                    }


                    for (int i = 0; i < 1; i++)
                    {
                        if (i % 2 == 1) continue;
                        int nodeOffset = CurrentTrackPoint.Node + i;

                        if (CurrentTrackPoint.Node % 2 == 1) nodeOffset++;

                        if (nodeOffset < 0) continue;
                        if (nodeOffset >= ARS.TrackPoints.Count - 1) continue;


                        TrackPoint t = ARS.TrackPoints[nodeOffset];

                        float followSpd = (float)Math.Sqrt(VehicleData.CurrentMechanicalGrip * 9.8f * t.GeneralCurveRadius);
                        float percent = ARS.map(ARS.MStoMPH(followSpd - Car.Velocity.Length()), -10, 10, 0, 100, true);
                        Color c = ARS.GradientAtoBtoC(Color.Red, Color.Yellow, Color.White, percent);

                        Vector3 l = t.Position - (Vector3.Cross(Vector3.WorldUp, t.Direction) * t.TrackWide);
                        Vector3 r = t.Position + (Vector3.Cross(Vector3.WorldUp, t.Direction) * t.TrackWide);

                        World.DrawMarker(MarkerType.DebugSphere, l, Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), c);
                        World.DrawMarker(MarkerType.DebugSphere, r, Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), c);


                    }
                }

            }
            else
            {
                if (showTrack)
                {
                    // Track limits display
                    Color blue = Color.FromArgb(50, Color.LightSkyBlue);
                    int fNode = (int)ARS.Clamp(CurrentTrackPoint.Node + 500, 0, ARS.TrackPoints.Count() - 1);
                    for (int i = CurrentTrackPoint.Node; i < fNode; i++)
                    {
                        if (i % 2 == 1)
                        {
                            TrackPoint tPoint = ARS.TrackPoints[i];
                            Vector3 right = tPoint.Position + (Vector3.Cross(tPoint.Direction, Vector3.WorldUp) * tPoint.TrackWide) + (Vector3.WorldUp * 0.5f);
                            Vector3 left = tPoint.Position + (Vector3.Cross(tPoint.Direction, Vector3.WorldUp) * -tPoint.TrackWide) + (Vector3.WorldUp * 0.5f);

                            World.DrawMarker(MarkerType.ChevronUpx1, right, tPoint.Direction, new Vector3(89, 0, -90), new Vector3(1, 1f, 2f), blue);
                            World.DrawMarker(MarkerType.ChevronUpx1, left, tPoint.Direction, new Vector3(89, 0, -90), new Vector3(1, 1f, 2f), blue);
                        }
                    }
                }
            }
        }

        void UpdateFollowLaneTrail()
        {
            if (ARS.TrackPoints == null || ARS.TrackPoints.Count == 0) return;

            int pidNode = (int)ARS.Clamp(CurrentTrackPoint.Node, 0, ARS.TrackPoints.Count - 1);
            if (LookAheads.TryGetValue(eLookAheads.SteerRef, out TrackPoint steerRefPoint) && steerRefPoint != null)
            {
                pidNode = (int)ARS.Clamp(steerRefPoint.Node, 0, ARS.TrackPoints.Count - 1);
            }

            int rawNode = pidNode;

            // With angle-based steering there is no lane-offset PID output.
            // Draw centerline trail for now; restore lane-offset trail when
            // racing line bias is added.
            float pidLaneOffset = 0f;
            float rawLaneOffset = 0f;
            Vector3 pidPoint = GetFollowLaneTrailPoint(pidNode, pidLaneOffset);
            Vector3 rawPoint = GetRawFollowLaneTrailPoint(rawNode, rawLaneOffset);

            if (LastFollowLaneTrailNode < 0)
            {
                LastFollowLaneTrailNode = pidNode;
                FollowLaneTrail.Add(pidPoint);
                RawFollowLaneTrail.Add(rawPoint);
                return;
            }

            if (pidNode == LastFollowLaneTrailNode)
            {
                bool pidUnchanged = FollowLaneTrail.Count > 0 && FollowLaneTrail.Last().DistanceTo(pidPoint) < 0.05f;
                bool rawUnchanged = RawFollowLaneTrail.Count > 0 && RawFollowLaneTrail.Last().DistanceTo(rawPoint) < 0.05f;
                if (pidUnchanged && rawUnchanged) return;
            }

            LastFollowLaneTrailNode = pidNode;
            FollowLaneTrail.Add(pidPoint);
            RawFollowLaneTrail.Add(rawPoint);

            while (FollowLaneTrail.Count > 50) FollowLaneTrail.RemoveAt(0);
            while (RawFollowLaneTrail.Count > 50) RawFollowLaneTrail.RemoveAt(0);
        }

        Vector3 GetFollowLaneTrailPoint(int node, float laneOffset)
        {
            TrackPoint t = ARS.TrackPoints[node];
            return t.Position - (Vector3.Cross(Vector3.WorldUp, t.Direction) * laneOffset) + (Vector3.WorldUp * 0.3f);
        }
        Vector3 GetRawFollowLaneTrailPoint(int node, float laneOffset)
        {
            TrackPoint t = ARS.TrackPoints[node];
            return t.Position - (Vector3.Cross(Vector3.WorldUp, t.Direction) * laneOffset) + (Vector3.WorldUp * 0.35f);
        }

        void DrawFollowLaneTrail()
        {
            if (FollowLaneTrail.Count < 2) return;

            Color laneBlue = Color.Black;
            for (int i = 1; i < FollowLaneTrail.Count; i++)
            {
                ARS.DrawLine(FollowLaneTrail[i - 1], FollowLaneTrail[i], laneBlue);
            }
        }
        void DrawRawFollowLaneTrail()
        {
            if (RawFollowLaneTrail.Count < 2) return;

            Color rawLaneColor = Color.White;
            for (int i = 1; i < RawFollowLaneTrail.Count; i++)
            {
                ARS.DrawLine(RawFollowLaneTrail[i - 1], RawFollowLaneTrail[i], rawLaneColor);
            }
        }

        void DrawInputTrails()
        {
            if (TrailSamples.Count < 2) return;

            for (int i = 1; i < TrailSamples.Count; i++)
            {
                TrailSample fromSample = TrailSamples[i - 1];
                TrailSample toSample = TrailSamples[i];
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
                    // Throttle side: yellow -> lime green -> blue.
                    baseColor = ARS.GradientAtoBtoC(Color.White, Color.GreenYellow, Color.Green, value * 100f);
                }
                else
                {
                    // Brake side: yellow -> orange (half) -> red.
                    float brake = -value;
                    baseColor = ARS.GradientAtoBtoC(Color.White, Color.Orange, Color.Red, brake * 100f);
                }
                Color finalColor = Color.FromArgb(255, baseColor.R, baseColor.G, baseColor.B);

                World.DrawMarker(MarkerType.ChevronUpx1, point, -away, new Vector3(90, 0, 0), chevronScale, finalColor, false, false, 0, false, "", "", false);
            }
        }

        /// <summary>
        /// </summary>
        /// <returns>Distance in meters</returns>
        float OutOfTrackDistance()
        {
            return (Math.Abs(Brain.data.DeviationFromCenter) + (VehicleData.BoundingBox / 2)) - CurrentTrackPoint.TrackWide;
        }

        void UpdateRivalInfo()
        {
            foreach (Rival r in Brain.Rivals) r.Update(this);
        }

        /// <summary>
        /// Figure out our point in the track.
        /// </summary>
        public void UpdateFollowTrack()
        {

            //Get our current track point data
            int refTrackpoint = (int)ARS.Clamp(CurrentTrackPoint.Node, 0, ARS.TrackPoints.Count - 1);

            List<TrackPoint> points = new List<TrackPoint>();
            for (int i = refTrackpoint - 6; points.Count <= 12; i++)
            {
                if (i < 0 || i >= ARS.TrackPoints.Count) i = 0;
                points.Add(ARS.TrackPoints[i]);
            }

            CurrentTrackPoint = points.OrderBy(t => t.Position.DistanceTo(Car.Position)).First();
            Brain.data.DeviationFromCenter = ARS.LeftOrRight(Car.Position, CurrentTrackPoint.Position, CurrentTrackPoint.Direction);

            LookAheads.Clear();
            float speed = Car.Velocity.Length();

            int steerRef = (int)ARS.Clamp((int)(speed / Math.Max(VehicleData.CurrentMechanicalGrip, 0.1f)), (int)(CurrentTrackPoint.TrackWide * 2f), 500);
            int quarterSec = (int)(speed * 0.25f);
            int halfSec = (int)(speed * 0.5f);
            int threeQuarterSec = (int)(speed * 0.75f);
            int oneSec = (int)(speed);
            int oneHalfSec = (int)(speed * 1.5f);
            int twoSec = (int)(speed * 2f);

            TrackPoint ResolveLookAhead(int offset)
            {
                if (CurrentTrackPoint.Node + offset >= ARS.TrackPoints.Count) return ARS.TrackPoints[offset];
                return ARS.TrackPoints[CurrentTrackPoint.Node + offset];
            }

            LookAheads.Add(eLookAheads.SteerRef, ResolveLookAhead(steerRef));
            LookAheads.Add(eLookAheads.QuarterSec, ResolveLookAhead(quarterSec));
            LookAheads.Add(eLookAheads.HalfSec, ResolveLookAhead(halfSec));
            LookAheads.Add(eLookAheads.ThreeQuarterSec, ResolveLookAhead(threeQuarterSec));
            LookAheads.Add(eLookAheads.OneSec, ResolveLookAhead(oneSec));
            LookAheads.Add(eLookAheads.OneHalfSec, ResolveLookAhead(oneHalfSec));
            LookAheads.Add(eLookAheads.TwoSec, ResolveLookAhead(twoSec));



            //Lap counting
            if (CanRegisterNewLap)
            {
                if (ARS.GetPercent(CurrentTrackPoint.Node, ARS.TrackPoints.Count) < 10 || (ARS.IsPointToPoint && ARS.GetPercent(CurrentTrackPoint.Node, ARS.TrackPoints.Count) > 99 && ARS.GetOffset(Car, ARS.TrackPoints.Last().Position).Y < 0f))
                {
                    CanRegisterNewLap = false;
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
            else if (BaseBehavior == RacerBaseBehavior.Race && ARS.GetPercent(CurrentTrackPoint.Node, ARS.TrackPoints.Count) > 50) CanRegisterNewLap = true;



            int follow = CurrentTrackPoint.Node + (int)Car.Velocity.Length();
            if (follow < ARS.TrackPoints.Count - 1)
            {
                TrackPoint end = ARS.TrackPoints[CurrentTrackPoint.Node + (int)Car.Velocity.Length()*2];
                TrackPoint midpoint = ARS.TrackPoints[CurrentTrackPoint.Node + (int)Car.Velocity.Length() ];
                Brain.data.FollowPoint = end;
                Brain.data.CurveRadiusToFollowPoint = ARS.GetCurveRadius(CurrentTrackPoint.Position, end.Position, midpoint.Position)/2;
            }
            else
            {
                Brain.data.FollowPoint = CurrentTrackPoint;
                Brain.data.CurveRadiusToFollowPoint = 999;
            }
        }

        public void AddDebugText(string s)
        {
            s = "~w~" + s + "~w~";
            if (!DebugText.Contains(s)) DebugText.Add(s);
        }

        /// <summary>
        /// Processes AI logic that works on slow (500ms - 2000ms) cycles.
        /// </summary>
        void ProcessTimedAI()
        {
            if (Brain.Corner == null) return;
            if (HalfSecondTick < Game.GameTime)
            {
                HalfSecondTick = Game.GameTime + 500 + (int)ARS.map(Car.Velocity.Length(), 0, 100, -250, 250, true);
                if (!Brain.Corner.Valid && ARS.MStoMPH(Car.Velocity.Length()) > 10) ARS.LookForCornerAhead(this);

            }


            //Slow checks
            if (OneSecondTick < Game.GameTime)
            {
                OneSecondTick = Game.GameTime + 1000;

                if (!ControlledByPlayer)
                {
                    if (BaseBehavior == RacerBaseBehavior.Race && ARS.Racers.Count >= 1)
                    {
                        UpdateRivals();
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

                //Rocket boost
                if (1 == 1 || !Brain.Corner.Valid || Brain.Corner.sToEntrance > 4)
                {
                    if (BaseBehavior == RacerBaseBehavior.Race && Math.Abs(VehicleData.SlideAngle) < 0.5f && Math.Abs(Control.Throttle) > 0.9f) Function.Call((Hash)0x81E1552E35DC3839, Car, true);
                }

                if (Function.Call<bool>((Hash)0x3D34E80EED4AE3BE, Car) && Control.Brake > 0.1f) Function.Call((Hash)0x81E1552E35DC3839, Car, false);


                if (ARS.DevSettingsFile.GetValue<int>("RACERS", "AIRacerAutofix", 1) == 2 && Function.Call<bool>(Hash._IS_VEHICLE_DAMAGED, Car))
                {
                    Car.Repair();
                }
            }
        }

        /// <summary>
        /// Runs Speeding and Steering logic.
        /// </summary>
        public void ProcessAI()
        {
            ProcessTimedAI();

            if (BaseBehavior == RacerBaseBehavior.GridWait && Control.HandBrakeTime < Game.GameTime) Control.HandBrakeTime = Game.GameTime + (100 * ARS.GetRandomInt(2, 6));

            if (!ControlledByPlayer)
            {
                UpdateRivalInfo();
                
                SteerTrack();
                SteerApplyCorrections();
                
                SpeedTrack(); 
                SpeedToThrottleBrake();
                SteerTranslateInput();

                UpdateStuckCheck();
                UpdateStuckRecovery();
                
                TractionControl();
                ApplyStuckRecoveryOverride();

            }
            else
            {
                IsStuckByThrottle = false;
                LastStuckGameTime = 0;
                IsRecoveringFromStuck = false;
                StuckRecoveryEndTime = 0;
            }
        }
 
 
        void UpdateStuckCheck()
        {
            if (ARS.MStoMPH(Car.Velocity.Length()) > 10f && Math.Abs(Brain.data.DeviationFromCenter) < CurrentTrackPoint.TrackWide)
            {
                StuckRecoveryAttempts = 0;
            }

            if (IsRecoveringFromStuck)
            {
                IsStuckByThrottle = false;
                LastStuckGameTime = 0;
                return;
            }

            if (BaseBehavior != RacerBaseBehavior.Race || !Driver.IsSittingInVehicle(Car))
            {
                IsStuckByThrottle = false;
                LastStuckGameTime = 0;
                return;
            }

            bool lowLongitudinalGs = Math.Abs(VehicleData.LongitudinalGs) < 0.25f;
            bool stuckCondition = lowLongitudinalGs && ARS.MStoMPH(Car.Velocity.Length()) < 5f;

            if (!stuckCondition)
            {
                IsStuckByThrottle = false;
                LastStuckGameTime = 0;
                return;
            }

            if (LastStuckGameTime == 0)
            {
                LastStuckGameTime = Game.GameTime;
            }

            bool stuckForLongEnough = (Game.GameTime - LastStuckGameTime) >= StuckCheckTimeMs;
            IsStuckByThrottle = stuckForLongEnough;

            if (stuckForLongEnough && !IsRecoveringFromStuck)
            {
                IsRecoveringFromStuck = true;
                StuckRecoveryAttempts++;
                HandleRecoveryAttemptEscalation();
                StuckRecoveryEndTime = Game.GameTime + StuckRecoveryTimeMs;
                IsStuckByThrottle = false;
                LastStuckGameTime = 0;
            }
        }

        void UpdateStuckRecovery()
        {
            if (BaseBehavior != RacerBaseBehavior.Race || !Driver.IsSittingInVehicle(Car))
            {
                IsRecoveringFromStuck = false;
                StuckRecoveryEndTime = 0;
                return;
            }

            if (!IsRecoveringFromStuck && IsStuckByThrottle)
            {
                IsRecoveringFromStuck = true;
                StuckRecoveryAttempts++;
                HandleRecoveryAttemptEscalation();
                StuckRecoveryEndTime = Game.GameTime + StuckRecoveryTimeMs;
                IsStuckByThrottle = false;
            }

            if (!IsRecoveringFromStuck) return;

            if (Game.GameTime >= StuckRecoveryEndTime)
            {
                IsRecoveringFromStuck = false;
                StuckRecoveryEndTime = 0;
                LastStuckGameTime = 0;
                return;
            }
        }

        void ApplyStuckRecoveryOverride()
        {
            if (!IsRecoveringFromStuck) return;

            if (Game.GameTime >= StuckRecoveryEndTime)
            {
                IsRecoveringFromStuck = false;
                StuckRecoveryEndTime = 0;
                LastStuckGameTime = 0;
                return;
            }

            // Force reverse with centered steering during the whole recovery window.
            Control.SteerInput = 0f;
            Control.Throttle = -1f;
            Control.Brake = 0f;
        }

        void HandleRecoveryAttemptEscalation()
        {
            if (StuckRecoveryAttempts < 2) return;

            Vector3 toTrack = CurrentTrackPoint.Position - Car.Position;
            if (toTrack.Length() < 0.01f) toTrack = Car.ForwardVector;

            Vector3 nudgedVelocity = toTrack.Normalized * ARS.MPHtoMS(30f);
            nudgedVelocity.Z += 30f;
            Car.Velocity = nudgedVelocity;

        }

        void UpdatePercievedGrip()
        {

            //Base vehicle grip GetVehicleMaxTraction
            float handlingGrip = Function.Call<float>((Hash)0xA132FB5370554DB0, Car) * (Handling.Gravity / 9.8f);
            handlingGrip = ARS.Clamp(handlingGrip, 0.1f, 5f);

            GroundGripMultiplier = ARS.GetWheelsGrip(Car).Average(); //Surface grip
            Vector3 thisPoint = CurrentTrackPoint.Position;
            Vector3 toMidpoint = LookAheads[eLookAheads.HalfSec].Position;
            Vector3 toEndpoint = LookAheads[eLookAheads.OneSec].Position;

            float elChangeDegrees = (toEndpoint.Normalized.Z - toMidpoint.Normalized.Z) * 90;



            float hillGsLoss = ARS.GetHillGripMultiplierAtCurrentVelocityVector(this,4);

            VehicleData.BaseMechanicalGrip = handlingGrip;
            VehicleData.CurrentMechanicalGrip = ((VehicleData.BaseMechanicalGrip) * GroundGripMultiplier);
            VehicleData.CurrentMechanicalGrip *= hillGsLoss;
            float GsLoss=ARS.GripGainLossElChange(thisPoint, toMidpoint, toEndpoint, Car.Velocity.Length());
            //VehicleData.CurrentMechanicalGrip += GsLoss;
        
            if (Math.Abs(Brain.data.DeviationFromCenter) < CurrentTrackPoint.TrackWide && RacePosition <= 2 && !ARS.MultiplierInTerrain.ContainsKey(CurrentTrackPoint.Node))
            {
                ARS.MultiplierInTerrain.Add(CurrentTrackPoint.Node, GroundGripMultiplier);
            }

            float zSpeedDegreesFromHoriz = (Math.Abs(VehicleData.SpeedVectorLocal.Normalized.Z) * 90);

            //Checks for Z movement to judge wether the car is stable or not. Usually implies the vehicle is mid-air
            if (zSpeedDegreesFromHoriz > 5f)
            {
                if (VehicleData.AvgGroundStability >= 0.1f) VehicleData.AvgGroundStability -= zSpeedDegreesFromHoriz * TickScale * 0.1f;
            }
            else if (VehicleData.AvgGroundStability < 1f) VehicleData.AvgGroundStability += 1f * TickScale;

            if (VehicleData.AvgGroundStability > 1.0f) VehicleData.AvgGroundStability = 1f;

            VehicleData.Gs = VehicleData.AccelerationVector.Aggregate(new Vector3(0, 0, 0), (s, v) => s + v) / (float)(VehicleData.AccelerationVector.Count) / 2;
            Brain.data.CurveRadiusPhysicalGs = ARS.GetCurveRadius(Car.Position - Car.Velocity + (VehicleData.Gs / 2), Car.Position + Car.Velocity + (VehicleData.Gs / 2), Car.Position);

            VehicleData.YawRotationPerSecondDegrees = ARS.rad2deg(Function.Call<Vector3>(Hash.GET_ENTITY_ROTATION_VELOCITY, Car).Z);            
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
                candidates.Sort((a, b) => Vector3.Distance(a.Car.Position, Car.Position).CompareTo(Vector3.Distance(b.Car.Position, Car.Position)));
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

        public float RiskFactorForGrip()
        {
            return 1f;
        }
        public float RiskFactorForBrake()
        {
            return 1;
        }
    }
}



