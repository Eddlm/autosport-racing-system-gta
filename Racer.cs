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


        public VehicleState VehicleData = new VehicleState();
        public HandlingData Handling = new HandlingData();
        public float GroundGripMultiplier = 1f;
        Vector3 _lastSpeed;


        public enum LookAhead { SteerRef, QuarterSec, HalfSec, ThreeQuarterSec, OneSec, OneHalfSec, TwoSec };
        public TrackPoint CurrentTrackPoint = new TrackPoint();
        public Dictionary<LookAhead, TrackPoint> LookAheads = new Dictionary<LookAhead, TrackPoint>();


        public List<TimeSpan> LapTimes = new List<TimeSpan>();
        public int LapStartTime = 0;
        public int Lap = 0;
        public int RacePosition = 0;
        public bool CanRegisterNewLap = true;
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
        public int _stuckRecoveryAttemptsNow => _stuckRecoveryAttempts;
        public bool _isRecoveringFromStuckNow => _isRecoveringFromStuck;


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


        public float RouteWindowStart = 1.0f;
        public float RouteWindowSize = 1.0f;


        bool _avoidLiftOff = false;
        float _avoidLeftWall = 0f;
        float _avoidRightWall = 0f;



        bool _isPassengerized = false;


        public float Aggression = 50f;

        public Racer(Vehicle RacerCar, Ped RacerPed)
        {
            Car = RacerCar;
            Driver = RacerPed;
            try { Name = RacerCar.FriendlyName; } catch (Exception) { Name = "Racer"; }
            if (Name == "NULL" || Name == null) { try { Name = Car.DisplayName.ToString()[0].ToString().ToUpper() + Car.DisplayName.ToString().Substring(1).ToLowerInvariant(); } catch (Exception) { Name = "Racer"; } }

            if (Driver.IsPlayer) ControlledByPlayer = true;
            _halfSecondTick = Game.GameTime + (ARS.GetRandomInt(10, 50));

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
            Handling.Downforce = ARS.GetDownforce(Car);
            if (Handling.Downforce > 100) Handling.Downforce *= 0.1f;

            Handling.LateralTractionCurve = ARS.RadToDeg(ARS.GetTRCurveLat(Car));
            if (Handling.LateralTractionCurve < 1 || Handling.LateralTractionCurve > 100) Handling.LateralTractionCurve = 22;
            ARS.Log(ARS.LogImportance.Info, "TRlat for " + Car.DisplayName + ":" + Handling.LateralTractionCurve + "º");

            Handling.BrakingAbility = Car.MaxBraking;
            Handling.TopSpeed = ARS.EngineTopSpeed(Car);
            Handling.Acceleration = Function.Call<float>(Hash.GET_VEHICLE_ACCELERATION, Car);


            VehicleData.SteeringLock = ARS.RadToDeg(ARS.GetSteerLock(Car));
            if (VehicleData.SteeringLock < 1 || VehicleData.SteeringLock > 100) VehicleData.SteeringLock = 40;
            ARS.Log(ARS.LogImportance.Info, "Steerlock for " + Car.DisplayName + ":" + VehicleData.SteeringLock + "º");
            Control.SteerDegrees = 0f;
            CurrentTrackPoint = ARS._trackPoints.Last();
            Control.Brake = 0f;
            Control.Throttle = 0f;

            LapTimes.Clear();
            LapStartTime = 0;
            Lap = 0;

            string flags = ARS.GetHandlingFlags(Car).ToString("X");
            int flagsHex = Convert.ToInt32(flags, 16);
            bool hasOffroad = (flagsHex & 0x800000) != 0 || (flagsHex & 0x200000) != 0;
            if (hasOffroad) Handling.Gravity *= 1.2f;

            BaseBehavior = RacerBaseBehavior.GridWait;
            FinishedPointToPoint = false;

            Handling.Grip = Function.Call<float>((Hash)0xA132FB5370554DB0, Car) * (Handling.Gravity / 9.8f);

            VehicleData.PerformanceIndex = (int)((Handling.TopSpeed * 5) + (Handling.Grip * 100) + (Handling.Acceleration * 500));
            VehicleData.TextPerformanceIndex = ((int)(Handling.TopSpeed / 1.2) + " | " + (int)(Handling.Acceleration * 200) + " | " + (int)(Handling.Grip * 20));

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


            float naturalLane;
            bool cornerActive = Brain.Corner != null && Brain.Corner.Valid && Lap > 0;
            if (cornerActive)
            {
                naturalLane = ComputeCornerTargetLane(steerRefPoint, speedMps);
            }
            else
            {

                naturalLane = 0f;
            }



            float clampedLane = ApplyRivalWalls(naturalLane, roadWide, carHalfWidth);


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
            bool hasActiveGuidance = cornerActive
                || _avoidLeftWall > -trackBound
                || _avoidRightWall < trackBound;
            if (hasActiveGuidance)
            {
                float currentLane = Brain.CurrentPerception.DeviationFromCenter;
                float lookaheadDist = steerRefPoint.Position.DistanceTo(Car.Position);
                if (lookaheadDist < 1f) lookaheadDist = speedMps * 1.5f;


                float laneError = clampedLane - currentLane;
                laneBiasDeg = -(float)(Math.Atan2(laneError, lookaheadDist) * (180.0 / Math.PI));
                laneBiasDeg *= 0.25f;
            }


            float totalTargetDeg = headingErrorDeg + laneBiasDeg + recoveryDeg;


            const float steerKP = 1.0f;
            float steerKD = 0.35f / VehicleData.CurrentMechanicalGrip;
            float pTerm = steerKP * totalTargetDeg;
            float dTerm = steerKD * VehicleData.YawRotationPerSecondDegrees;
            Control.SteerDegrees = pTerm - dTerm;



            if (Math.Sign((int)VehicleData.SlideAngle) == Math.Sign((int)VehicleData.YawRotationPerSecondDegrees))
            {
                float slideCounterSteer = VehicleData.SlideAngle * ARS.Remap(
                    Math.Abs(VehicleData.SlideAngle), 0, Handling.LateralTractionCurve * 1.2f, 0.5f, 1.2f, true);
                Control.SteerDegrees -= slideCounterSteer;
            }



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






        float ComputeCornerTargetLane(TrackPoint steerRefPoint, float speedMps)
        {
            CornerPoint c = Brain.Corner.Point;
            int apexNode = c.Node;



            float cornerDir = Math.Sign(c.Angle);







            if (cornerDir == 0f) return 0f;

            float halfWidth = steerRefPoint.TrackHalfWidth;
            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            float safeBound = halfWidth - carHalfWidth;


            float curveRadius = Math.Abs(Brain.CurrentPerception.CurveRadiusToFollowPoint);
            float insideIntensity = ARS.Remap(curveRadius, 50f, 300f, 1f, 0.1f, true);
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
                targetLane = insideTarget;
            }
            else if (timeToApex <= turnInTime)
            {
                targetLane = insideTarget;
            }
            else if (timeToApex <= lerpStartTime)
            {
                float outsideTarget = cornerDir * safeBound;
                float t = (timeToApex - turnInTime) / (lerpStartTime - turnInTime);
                targetLane = insideTarget + (outsideTarget - insideTarget) * t;
            }
            else if (timeToApex <= approachStartTime)
            {
                targetLane = cornerDir * safeBound;
            }
            else
            {
                targetLane = 0f;
            }

            return targetLane;
        }













        float ApplyRivalWalls(float targetLane, float roadWide, float carHalfWidth)
        {
            _avoidLiftOff = false;

            float trackBound = roadWide - carHalfWidth;


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
                else if (r.RelativePosition == RelativePos.Ahead && r.SecondsToReach >= 0f && r.SecondsToReach <= 4f && Math.Abs(r.DirectionDiff) <= 20f)
                {
                    isRelevant = true;
                    rivalIsLeft = r.OccupiedLane < Brain.CurrentPerception.DeviationFromCenter;
                }

                if (!isRelevant) continue;


                float aggroBuffer = ARS.Remap(Aggression, 0f, 100f, 2f, 0.25f, true);
                float rivalBuffer = r.OccupiedLaneWidth * 0.5f + aggroBuffer;

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


            float carTotalWidth = carHalfWidth * 2f + 1f;
            if (_avoidRightWall - _avoidLeftWall < carTotalWidth)
                _avoidLiftOff = true;


            float clampLeft = _avoidLeftWall + carHalfWidth;
            float clampRight = _avoidRightWall - carHalfWidth;
            return ARS.Clamp(targetLane, clampLeft, clampRight);
        }

        void ApplySteerLimits()
        {


            if (Math.Sign(Control.SteerDegrees) == Math.Sign((int)VehicleData.YawRotationPerSecondDegrees))
            {
                float speedBasedSteeringLimit = (float)((VehicleData.BaseMechanicalGrip * Handling.Gravity * VehicleData.WheelBase) / Math.Pow(Car.Velocity.Length() + 0.01f, 2.01f));
                speedBasedSteeringLimit = Math.Max(ARS.RadToDeg(speedBasedSteeringLimit) * 0.9f, 1f);
                Control.SteerDegrees = ARS.Clamp(Control.SteerDegrees, -speedBasedSteeringLimit, speedBasedSteeringLimit);
            }
        }


        public void Launch()
        {
            Brain.Corner = new Corner(5, ARS._cornerPoints.FirstOrDefault(c => c.IsKey));
            Brain.Corner.Valid = false;
            VehicleData.AvgGroundStability = 1;
            BaseBehavior = RacerBaseBehavior.Race;
            LapStartTime = Game.GameTime;
            Control.HandBrakeTime = Game.GameTime + ARS.GetRandomInt(100, 400);
            Control.MaxThrottle = 1f;
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
            float newThrottle = 0f;
            float newBrake = 0f;
            float throttleCap = Math.Min(Control.TCSThrottle, 1f);
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

            if (speedErrorGs > 0.0f)
            {

                if (currentForwardSpeed < -dirSwitchSpeed) newBrake = ARS.Clamp(speedErrorGs * 2f, 0f, 1f);
                else newThrottle = ARS.Clamp(speedErrorGs * 2f, 0f, throttleCap);
            }
            else if (speedErrorGs < 0.0f)
            {
                float reverseDemand = ARS.Clamp((-speedErrorGs) * 2f, 0f, throttleCap);
                float brakeDemand = ARS.Clamp((-speedErrorGs) * 2f, 0f, 1f);

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


 
            if (newBrake > 0.0) newThrottle = 0; else newBrake = 0;


            float stabilityThrottleLimit = VehicleData.AvgGroundStability;
            if (OutOfTrackDistance() > 0.5f)
            {

                stabilityThrottleLimit = Math.Max(stabilityThrottleLimit, 0.45f);
            }


            Control.Brake += (newBrake - Control.Brake) * 5 * TickScale;
            Control.Throttle += (newThrottle - Control.Throttle) * 5 * TickScale;
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
            if (Brain.Corner.Valid) cornerSpd = Math.Max(2, ARS.MaxSpeedForBrakingDistance(Brain.Corner.Point, this) + 1f);

            float followRadius = Math.Max(Brain.CurrentPerception.CurveRadiusToFollowPoint, 0.1f);
            float followTrackSpd =(float)Math.Sqrt((VehicleData.CurrentMechanicalGrip * Handling.Gravity) * followRadius);            

            

            float hillSpeed = Car.Velocity.Length();
            int hillStartNode = (int)ARS.Clamp(CurrentTrackPoint.Node + (int)(hillSpeed * RouteWindowStart), 0, ARS._trackPoints.Count - 1);
            int hillEndNode = (int)ARS.Clamp(CurrentTrackPoint.Node + (int)(hillSpeed * (RouteWindowStart + RouteWindowSize)), 0, ARS._trackPoints.Count - 1);
            int hillMidNode = (int)((hillStartNode + hillEndNode) * 0.5f);
            if (hillMidNode < 0) hillMidNode = 0;
            if (hillMidNode >= ARS._trackPoints.Count) hillMidNode = ARS._trackPoints.Count - 1;

            float hillGsDelta = ARS.HillGripDeltaGs(ARS._trackPoints[hillStartNode].Position, ARS._trackPoints[hillMidNode].Position, ARS._trackPoints[hillEndNode].Position, followTrackSpd);
            hillGsDelta= ARS.Clamp(hillGsDelta*0.5f, -0.3f, 0.3f);
            float adjustedFollowGrip = Math.Max(0.1f, VehicleData.CurrentMechanicalGrip + hillGsDelta);
            followTrackSpd=(float)Math.Sqrt((adjustedFollowGrip * Handling.Gravity) * followRadius);

            if (float.IsNaN(cornerSpd) || float.IsInfinity(cornerSpd)) cornerSpd = 999f;
            if (float.IsNaN(followTrackSpd) || float.IsInfinity(followTrackSpd)) followTrackSpd = 999f;
            if (cornerSpd <= 5) cornerSpd = ARS.CornerApexSpeed(Brain.Corner.Point, this);
              
            Brain.CurrentIntention.Speed = Math.Min(cornerSpd, followTrackSpd);



            if (_avoidLiftOff)
            {
                Rival threat = Brain.Rivals.FirstOrDefault(r => r.RivalRacer != null && r.RelativePosition == RelativePos.Ahead);
                if (threat != null)
                {
                    float rivalSpeed = threat.RivalRacer.Car.Velocity.Length();
Brain.CurrentIntention.Speed = Math.Min(Brain.CurrentIntention.Speed, rivalSpeed);
                }
            }

        }




        void TractionControl()
        {
            if (Control.Throttle <= 0.0f) return;

            float wheelspin = ARS.MaxWheelSlip(Car);
            bool lowGripOrLowGear = GroundGripMultiplier < 0.9f || Car.CurrentGear < 2;


            float allowedWheelspin = 2.0f + ARS.Remap(Aggression, 0f, 100f, -0.2f, 0.2f, true);

            float tcsValue = ARS.Remap(Math.Abs(wheelspin) - allowedWheelspin, 0.1f, -0.1f, -1f, 1f, true) * 8;
            float change = tcsValue * TickScale;
            Control.TCSThrottle = ARS.Clamp(Control.TCSThrottle + change, 0.2f, 1);
        }
        public void UpdateTickData()
        {
            VehicleData.LocalGs = (Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true) - VehicleData.SpeedVectorLocal) / Game.LastFrameTime;
            Vector3 cSpeed = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, false);

            VehicleData.AccelerationVector.Add((cSpeed - _lastSpeed) / Game.LastFrameTime);

            if (VehicleData.AccelerationVector.Count > 10) VehicleData.AccelerationVector.RemoveAt(0);

            _lastSpeed = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, false);
            VehicleData.SpeedVectorGlobal = cSpeed;
            VehicleData.SpeedVectorLocal = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true);
            Brain.CurrentPerception.SpeedVector = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true);


            if (ARS._debugToggles[Options.ShowInputs] && !Driver.IsPlayer)
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

            while (_trailSamples.Count > 50) _trailSamples.RemoveAt(0);
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
            UpdateCornerValidity();

            ProcessAI();
            if (Driver.IsPlayer && ARS._settingsFile.GetValue("CATCHUP", "OnlyBehindPlayer", true)) ARS.CatchupPosition = RacePosition;

            _lastCoreTick = Game.GameTime;
        }



        void UpdateSlideAndBoundingBox()
        {
            VehicleData.BoundingBox = ARS.SlidingBoundingBoxWidth(Car);
            VehicleData.SlideAngle = (float)Math.Round(Vector3.SignedAngle(Car.Velocity.Normalized, Car.ForwardVector, Car.UpVector), 3);
        }
        void UpdateCornerValidity()
        {
            if (Brain.Corner == null) return;

            if (Brain.Corner.Valid && Lap > 0)
            {
                int apexNode = Brain.Corner.Point.Node;

                if (CurrentTrackPoint.Node >= apexNode || (Math.Abs(CurrentTrackPoint.Node - apexNode) > 1000))
                {
                    Brain.Corner.Valid = false;
                 }
            }


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
        void DrawRacerDebug()
        {
            bool showAggro = ARS._debugToggles[Options.ShowAggro];
            bool showInputs = ARS._debugToggles[Options.ShowInputs];
            bool showTrack = ARS._debugToggles[Options.ShowTrackAnalysis];
            bool showPhysics = ARS._debugToggles[Options.ShowPhysics];
            bool showAny = showAggro || showInputs || showTrack || showPhysics;
            if (!showAny) return;

            if ((Car.Position - Game.Player.Character.Position).Length() > 50) return;


            if (showTrack && Driver.IsPlayer && Lap >= ARS._settingsFile.GetValue<int>("GENERAL_SETTINGS", "Laps", 5) && CanRegisterNewLap)
            {
                World.DrawMarker(MarkerType.CheckeredFlagRect, ARS._trackPoints.First().Position + new Vector3(0, 0, 5f), ARS._trackPoints.First().Direction, new Vector3(0, 0, 0), new Vector3(5f, 5f, 5f), Color.White);
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
                }
                if (showTrack)
                {
                   
                }

                if (showAggro)
                {

                    Vector3 trackCenter = CurrentTrackPoint.Position;
                    Vector3 trackRight = Vector3.Cross(CurrentTrackPoint.Direction, Vector3.WorldUp);
                    Vector3 leftWallPos = trackCenter + trackRight * _avoidLeftWall;
                    Vector3 rightWallPos = trackCenter + trackRight * _avoidRightWall;
                    Vector3 up = new Vector3(0, 0, 0.1f);
                    ARS.DrawLine(leftWallPos + up, leftWallPos + up + new Vector3(0, 0, 2f), Color.Blue);
                    ARS.DrawLine(rightWallPos + up, rightWallPos + up + new Vector3(0, 0, 2f), Color.Red);


                    Vector3 textPos = Car.Position + new Vector3(0, 0, 2f);
                    ARS.DrawText(textPos, "~w~My lane: ~b~" + Brain.CurrentPerception.DeviationFromCenter.ToString("0.0") + " ~w~L: ~b~" + _avoidLeftWall.ToString("0.0") + " ~w~R: ~r~" + _avoidRightWall.ToString("0.0"), Color.White, 0.4f);
                    int ri = 0;
                    foreach (Rival r in Brain.Rivals)
                    {
                        if (r.RivalRacer == null) continue;
                        Vector3 rTextPos = Car.Position + new Vector3(0, 0, 2.5f + ri * 0.4f);
                        string side = r.RelativePosition == RelativePos.Left ? "L" : r.RelativePosition == RelativePos.Right ? "R" : r.RelativePosition == RelativePos.Ahead ? "A" : "?";
                        ARS.DrawText(rTextPos, "~w~R" + ri + "(" + side + ") lane: ~r~" + r.OccupiedLane.ToString("0.0") + " s: ~y~" + r.SecondsToReach.ToString("0.0"), Color.White, 0.4f);
                        ri++;
                    }
                }

                if (showTrack)
                {
                    DrawCornerChevrons(Brain.Corner);
                }

            }
            else
            {
                if (showTrack)
                {
                    DrawCornerChevrons(Brain.Corner);
                }
            }
        }

        void DrawCornerChevrons(Corner corner)
        {
            if (corner == null || !corner.Valid || Lap <= 0) return;
            CornerPoint c = corner.Point;
            int startNode = (int)ARS.Clamp(c.Node - c.LengthStart, 0, ARS._trackPoints.Count - 1);
            int endNode = (int)ARS.Clamp(c.Node + c.LengthEnd, 0, ARS._trackPoints.Count - 1);

            Vector3 chevScale = new Vector3(ARS._trackPoints[c.Node].TrackHalfWidth * 2.5f, 5, 5);

            World.DrawMarker(MarkerType.ChevronUpx1, ARS._trackPoints[startNode].Position, ARS._trackPoints[startNode].Direction, new Vector3(90, 0, 0), chevScale, Color.Green);
            World.DrawMarker(MarkerType.ChevronUpx1, ARS._trackPoints[c.Node].Position, ARS._trackPoints[c.Node].Direction, new Vector3(90, 0, 0), chevScale, Color.Green);
            World.DrawMarker(MarkerType.ChevronUpx1, ARS._trackPoints[endNode].Position, ARS._trackPoints[endNode].Direction, new Vector3(90, 0, 0), chevScale, Color.Green);
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


            int refTrackpoint = (int)ARS.Clamp(CurrentTrackPoint.Node, 0, ARS._trackPoints.Count - 1);

            List<TrackPoint> points = new List<TrackPoint>();
            for (int i = refTrackpoint - 6; points.Count <= 12; i++)
            {
                if (i < 0 || i >= ARS._trackPoints.Count) i = 0;
                points.Add(ARS._trackPoints[i]);
            }

            CurrentTrackPoint = points.OrderBy(t => t.Position.DistanceTo(Car.Position)).First();
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
                if (CurrentTrackPoint.Node + offset >= ARS._trackPoints.Count) return ARS._trackPoints[offset];
                return ARS._trackPoints[CurrentTrackPoint.Node + offset];
            }

            LookAheads.Add(LookAhead.SteerRef, ResolveLookAhead(steerRef));
            LookAheads.Add(LookAhead.QuarterSec, ResolveLookAhead(quarterSec));
            LookAheads.Add(LookAhead.HalfSec, ResolveLookAhead(halfSec));
            LookAheads.Add(LookAhead.ThreeQuarterSec, ResolveLookAhead(threeQuarterSec));
            LookAheads.Add(LookAhead.OneSec, ResolveLookAhead(oneSec));
            LookAheads.Add(LookAhead.OneHalfSec, ResolveLookAhead(oneHalfSec));
            LookAheads.Add(LookAhead.TwoSec, ResolveLookAhead(twoSec));




            if (CanRegisterNewLap)
            {
                if (ARS.GetPercent(CurrentTrackPoint.Node, ARS._trackPoints.Count) < 10 || (ARS._isPointToPoint && ARS.GetPercent(CurrentTrackPoint.Node, ARS._trackPoints.Count) > 99 && ARS.EntityRelativeOffset(Car, ARS._trackPoints.Last().Position).Y < 0f))
                {
                    CanRegisterNewLap = false;
                    Lap++;
                    if (Lap > ARS._settingsFile.GetValue("GENERAL_SETTINGS", "Laps", 5))
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
            else if (BaseBehavior == RacerBaseBehavior.Race && ARS.GetPercent(CurrentTrackPoint.Node, ARS._trackPoints.Count) > 50) CanRegisterNewLap = true;






            float routeSpeed = Car.Velocity.Length();
            int routeStartNode = (int)ARS.Clamp(CurrentTrackPoint.Node + (int)(routeSpeed * RouteWindowStart), 0, ARS._trackPoints.Count - 1);
            int routeEndNode = (int)ARS.Clamp(CurrentTrackPoint.Node + (int)(routeSpeed * (RouteWindowStart + RouteWindowSize)), 0, ARS._trackPoints.Count - 1);
            int routeMidNode = (int)((routeStartNode + routeEndNode) * 0.5f);
            if (routeMidNode < 0) routeMidNode = 0;
            if (routeMidNode >= ARS._trackPoints.Count) routeMidNode = ARS._trackPoints.Count - 1;

            if (routeEndNode != routeStartNode)
            {
                Brain.CurrentPerception.CurveRadiusToFollowPoint = ARS.Circumradius3D(ARS._trackPoints[routeStartNode].Position, ARS._trackPoints[routeEndNode].Position, ARS._trackPoints[routeMidNode].Position) / 2;
            }
            else
            {
                Brain.CurrentPerception.CurveRadiusToFollowPoint = 999;
            }
        }




        void ProcessTimedAI()
        {
            if (Brain.Corner == null) return;
            if (_halfSecondTick < Game.GameTime)
            {
                _halfSecondTick = Game.GameTime + 500 + (int)ARS.Remap(Car.Velocity.Length(), 0, 100, -250, 250, true);
                if (!Brain.Corner.Valid && ARS.MpsToMph(Car.Velocity.Length()) > 10) ARS.FindNextCorner(this);

            }



            if (_oneSecondTick < Game.GameTime)
            {
                _oneSecondTick = Game.GameTime + 1000;

                if (!ControlledByPlayer)
                {
                    if (BaseBehavior == RacerBaseBehavior.Race && ARS._racers.Count >= 1)
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

            if (BaseBehavior == RacerBaseBehavior.GridWait && Control.HandBrakeTime < Game.GameTime) Control.HandBrakeTime = Game.GameTime + (100 * ARS.GetRandomInt(2, 6));

            if (!ControlledByPlayer)
            {
                UpdateRivalInfo();
                
                ComputeSteering();
                ApplySteerLimits();
                
                ComputeTargetSpeed(); 
                ConvertSpeedToPedals();
                TranslateSteerToInput();

                UpdateStuckCheck();
                UpdateStuckRecovery();
                
                TractionControl();
                ApplyStuckRecoveryOverride();

            }
            else
            {
                IsStuckByThrottle = false;
                _lastStuckGameTime = 0;
                _isRecoveringFromStuck = false;
                _stuckRecoveryEndTime = 0;
            }
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

            bool lowLongitudinalGs = Math.Abs(VehicleData.LongitudinalGs) < 0.25f;
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

            Vector3 toTrack = CurrentTrackPoint.Position - Car.Position;
            if (toTrack.Length() < 0.01f) toTrack = Car.ForwardVector;

            Vector3 nudgedVelocity = toTrack.Normalized * ARS.MphToMps(30f);
            nudgedVelocity.Z += 30f;
            Car.Velocity = nudgedVelocity;

        }

        void UpdatePerceivedGrip()
        {


            float handlingGrip = Function.Call<float>((Hash)0xA132FB5370554DB0, Car) * (Handling.Gravity / 9.8f);
            handlingGrip = ARS.Clamp(handlingGrip, 0.1f, 5f);

            GroundGripMultiplier = ARS.WheelGripMultipliers(Car).Average();
            Vector3 thisPoint = CurrentTrackPoint.Position;
            Vector3 toMidpoint = LookAheads[LookAhead.HalfSec].Position;
            Vector3 toEndpoint = LookAheads[LookAhead.OneSec].Position;

            float elChangeDegrees = (toEndpoint.Normalized.Z - toMidpoint.Normalized.Z) * 90;



            float hillGsLoss = ARS.HillGripMultiplierFromVelocity(this,4);

            VehicleData.BaseMechanicalGrip = handlingGrip;
            VehicleData.CurrentMechanicalGrip = ((VehicleData.BaseMechanicalGrip) * GroundGripMultiplier);
            VehicleData.CurrentMechanicalGrip *= hillGsLoss;
            float GsLoss=ARS.HillGripDeltaGs(thisPoint, toMidpoint, toEndpoint, Car.Velocity.Length());

        
            if (Math.Abs(Brain.CurrentPerception.DeviationFromCenter) < CurrentTrackPoint.TrackHalfWidth && RacePosition <= 2 && !ARS._terrainGripMultipliers.ContainsKey(CurrentTrackPoint.Node))
            {
                ARS._terrainGripMultipliers.Add(CurrentTrackPoint.Node, GroundGripMultiplier);
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
            foreach (Racer r in ARS._racers)
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
    }
}


