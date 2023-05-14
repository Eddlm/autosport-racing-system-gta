using GTA;
using GTA.Math;
using GTA.Native;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

namespace ARS
{
    public enum RacerBaseBehavior
    {
        GridWait, Race, FinishedRace, FinishedStandStill
    }
    public class Racer
    {

        public Team team = Team.None;
        public VehicleControl vControl = new VehicleControl(); //Inputs for throttle(reverse), brake, steer and handbrake
        public Memory mem = new Memory(); //Holds live data about the track, other opponents nearby and its own personality        
        public bool ControlledByPlayer = false;

        //Decision-cooldown pairs
        public Dictionary<Decision, int> Decisions = new Dictionary<Decision, int>();
        public Dictionary<Decision, int> BannedDecisions = new Dictionary<Decision, int>();
        public Dictionary<Mistake, int> Mistakes = new Dictionary<Mistake, int>();
        public Dictionary<Mistake, int> BannedMistakes = new Dictionary<Mistake, int>();


        public List<TimeSpan> LapTimes = new List<TimeSpan>();
        public int LapStartTime = 0;
        public string Name = "Racer";
        public Ped Driver;
        public Vehicle Car;
        public List<string> DebugText = new List<string>();
        public List<Vector3> trail = new List<Vector3>();
        public Vector3 LastStuckPlace = Vector3.Zero;
        public RacerBaseBehavior BaseBehavior = RacerBaseBehavior.GridWait;
        public RaceState RCStatus = RaceState.NotInitiated;

        //Cheats
        float TorqueMult = 1.0f;

        //Race position
        public int Lap = 0;
        public int Pos = 0;
        public bool CanRegisterNewLap = true;

        //Ticks
        int HalfSecondTick = 0; //500ms
        int TenthSecondTick = 0; //100ms
        int OneSecondTick = 0; //1000ms
        int SpdBasedTick = 0; //velocity 1000-(m/s) 
        //Current Traction Curve Angle based off last direction changes
        float TRCurveAngle = 0f;

        //Stuck stuff
        int StuckGameTimeRef = 0;
        public bool StuckRecover = false;
        int TimeOutOfTrack = 0;

        //Perceived Info to make decisions
        List<Racer> NearbyRivals = new List<Racer>();
        public List<CornerPoint> KnownCorners = new List<CornerPoint>();

        //Handling stuff
        public float GroundGripMultiplier = 1f;
        public float BoundingBox = 0f;

        public Racer(Vehicle RacerCar, Ped RacerPed)
        {
            Car = RacerCar;
            Driver = RacerPed;
            Name = RacerCar.FriendlyName;
            if (Name == "NULL" || Name == null) Name = Car.DisplayName.ToString()[0].ToString().ToUpper() + Car.DisplayName.ToString().Substring(1).ToLowerInvariant();

            if (Driver.IsPlayer) ControlledByPlayer = true;
            HalfSecondTick = Game.GameTime + (ARS.GetRandomInt(10, 50));

            if (!ControlledByPlayer)
            {

                Driver.BlockPermanentEvents = true;
                Driver.AlwaysKeepTask = true;
                Function.Call(GTA.Native.Hash.SET_DRIVER_ABILITY, Driver, 1f);
                Function.Call(GTA.Native.Hash.SET_DRIVER_AGGRESSIVENESS, Driver, 100f);

                if (ARS.DevSettingsFile.GetValue<int>("RACERS", "AIRacerAutofix", 1) == 2)
                {
                    Function.Call(GTA.Native.Hash.SET_ENTITY_PROOFS, Car, true, true, true, true, true, true, true, true);
                    Function.Call(GTA.Native.Hash.SET_ENTITY_PROOFS, Driver, true, true, true, true, true, true, true, true);

                    Car.IsInvincible = true;
                    Car.IsCollisionProof = true;
                    Car.IsOnlyDamagedByPlayer = true;
                    Function.Call(GTA.Native.Hash.SET_VEHICLE_STRONG, Car, true);
                    Function.Call(GTA.Native.Hash.SET_VEHICLE_HAS_STRONG_AXLES, Car, true);
                    Car.EngineCanDegrade = false;

                }
                else if (ARS.DevSettingsFile.GetValue<int>("RACERS", "AIRacerAutofix", 1) == 1)
                {
                    Function.Call(GTA.Native.Hash.SET_VEHICLE_STRONG, Car, true);
                    Function.Call(GTA.Native.Hash.SET_VEHICLE_HAS_STRONG_AXLES, Car, true);
                    Car.EngineCanDegrade = false;
                }
                else
                {
                    Car.IsInvincible = false;
                    Car.IsCollisionProof = false;
                }

                Car.EngineRunning = true;
                ARS.SetBrakes(Car, 0f);
                Driver.SetIntoVehicle(Car, VehicleSeat.Driver);
                Car.IsRadioEnabled = false;
            }

            Car.IsPersistent = true;
            if ((Car.CurrentBlip == null || Car.CurrentBlip.Exists() == false) && !Driver.IsPlayer)
            {
                Car.AddBlip();
                Car.CurrentBlip.Color = BlipColor.Blue;
                Car.CurrentBlip.Scale = 0.75f;
                Function.Call(Hash._SET_BLIP_SHOW_HEADING_INDICATOR, Car.CurrentBlip, true);
                Function.Call(Hash._0x2B6D467DAB714E8D, Car.CurrentBlip, true);
                Car.CurrentBlip.Name = Name;
            }

            Function.Call(GTA.Native.Hash._0x0DC7CABAB1E9B67E, Car, true, 1);
            Function.Call(GTA.Native.Hash._0x0DC7CABAB1E9B67E, Driver, true, 1);
            Function.Call(GTA.Native.Hash.SET_ENTITY_PROOFS, Driver, true, true, true, false, true, true, 1, true);

            Driver.MaxHealth = 1000;
            Driver.Health = 1000;
            Driver.CanSufferCriticalHits = false;

            if (Car.ClassType == VehicleClass.Emergency) team = Team.Cop;
        }



        public void Initialize()
        {
            handlingData.Downforce = ARS.GetDownforce(Car);
            handlingData.TRlateral = ARS.rad2deg(ARS.GetTRCurveLat(Car));
            handlingData.BrakingAbility = Car.MaxBraking;
            handlingData.TopSpeed = ARS.EngineTopSpeed(Car);
            SteeringLock = ARS.rad2deg(ARS.GetSteerLock(Car));

            vControl.SteerTrack = 0f;
            trackPoint = ARS.TrackPoints.Last();
            mem.data.SteerAngle = 0f;
            vControl.Brake = 0f;
            vControl.Throttle = 0f;

            LapTimes.Clear();
            LapStartTime = 0;
            Lap = 0;

            BaseBehavior = RacerBaseBehavior.GridWait;
            FinishedPointToPoint = false;

            Car.Repair();
        }

        /// <summary>
        /// Neccesary calculations for the car to follow the track. The whole function results in vControl.SteerTrack.
        /// </summary>
        public void SteerTrack()
        {

            if (BaseBehavior == RacerBaseBehavior.GridWait || BaseBehavior == RacerBaseBehavior.FinishedStandStill)
            {
                vControl.SteerTrack = 0f;
                return;
            }

            float roadWide = FTHalfSec.TrackWide - (BoundingBox * 0.5f);

            //Base
            float baseAim = Vector3.SignedAngle(Car.Velocity.Normalized, FTQuarterSec.Direction, Vector3.WorldUp);

            /*
            //Get to the outside to approach the corner properly
            if (ARS.CornerPoints.Any() && trackPoint != null)
            {
                if (KnownCorners.Any())
                {
                    CornerPoint c = KnownCorners.First();
                    int dist = c.Node - c.LengthStart - trackPoint.Node;
                    float sToReach = 0;

                    if (dist > 0.0f) sToReach = (dist / (Car.Velocity.Length()));

                    //Creep to the outside of a corner if we need to brake to take it (better lineup)
                    if (sToReach < 10)
                    {
                        if (sToReach < 0) sToReach = 0.05f;

                        float Outside = followTrackPoint.TrackWide * (c.Angle > 0 ? 1 : -1);
                        float Inside = -Outside;

                        //float max = ARS.map(Math.Abs(c.AvgAngle), 30, 5, 1.25f, 1.75f, true);
                        intendedBias = ARS.map(sToReach, 1f, 0.5f, Outside, Inside);

                    }
                }
            }
            */
            float currentTotalGrip = (vehData.CurrentGrip + vehData.CurrentDownforce);
            float intendedLane = 0f;

            float Outside = 0;
            float Inside = 0;

            //Adjust the intended lane
            float differenceToLane = 0;

            //Bias to stay on the inside
            float angleFuture = 0;
            float baseStrictness = 1f / currentTotalGrip;
            float strictness = baseStrictness;
            if (FTOneSec != null)
            {
                angleFuture = Vector3.SignedAngle(trackPoint.Direction, FTHalfSec.Direction, Vector3.WorldUp);
                strictness *= ARS.map(Math.Abs(angleFuture), 5f, 22.5f*2, 1f, 2*2, true);

                Inside = roadWide * (angleFuture > 0 ? -1 : 1);
                Outside = -Inside;

                float threshold = 10;
                
                if (Math.Abs(angleFuture) > threshold)
                {
                    intendedLane = Inside;
                    differenceToLane = (mem.data.DeviationFromCenter - intendedLane) * strictness;
                }
            }

            //If there is a corner, go to the outside until we are close
            if (KnownCorners.Any())
            {
                CornerPoint c = KnownCorners.First();
                float timeToCorner = (c.Node - trackPoint.Node) / Car.Velocity.Length();
                float overSpeed = Car.Velocity.Length() - c.Speed;
                
                Inside = roadWide * (c.Angle > 0 ? -1 : 1);
                Outside = -Inside;

                if (timeToCorner < 5)
                {
                    intendedLane = ARS.map(timeToCorner, 1.75f, 1.25f, Outside, Inside);
                    intendedLane = ARS.Clamp(intendedLane, -roadWide, roadWide);

                    differenceToLane = (mem.data.DeviationFromCenter - intendedLane) * strictness;
                }
            }

            if (differenceToLane > 0 != angleFuture > 0) differenceToLane = ARS.Clamp(differenceToLane, -2, 2);
            
            mem.intention.LookaheadDeviationFromCenter = intendedLane;
            
            if (mem.Rivals.Any())
            {
                Rival rival = mem.Rivals.OrderByDescending(r => r.AvoidStr).First();

                if (mem.Rivals.Any(r => r.relativePos != RelativePos.Unreachable && r.rPos.X > 0 != differenceToLane > 0)) differenceToLane *= 0.5f;

                if (rival.AvoidStr > 0.0f && mem.AvoidAvgLane != 0)
                {
                    float extraLaneRequest = ARS.Lerp(intendedLane, mem.AvoidAvgLane, rival.AvoidStr);
                    differenceToLane = (mem.data.DeviationFromCenter - extraLaneRequest) * (strictness * (rival.AvoidStr + 1));
                }
            }


            //Max angle to move side to side
            float maxLatSpeed = 999;// ARS.map(ARS.MStoMPH(Car.Velocity.Length()), 60, 40, 3, 20, true);
            differenceToLane = ARS.Clamp(differenceToLane, -maxLatSpeed, maxLatSpeed);

            //Get back inside the track limits
            float outOfTrack = Math.Abs(mem.data.DeviationFromCenter) - roadWide; // OutOfTrackDistance() + 0.5f;
            if (outOfTrack > 0)
            {
                Vector2 back = new Vector2(0, 1);
                back.X = outOfTrack * (mem.data.DeviationFromCenter > 0 ? 0.75f : -0.75f);
                differenceToLane += ARS.Clamp(back.X, -45, 45);
            }


            float mult = currentTotalGrip;// * ARS.map(Math.Abs(angleFuture), 20, 40, 1, 2, true);
            float rAngle = baseAim / mult;


            //Steering behavior if we are going backwards.
            if (mem.data.SpeedVector.Y < 0f)
            {
                //Physical direction actually pointing to the track
                if (Math.Abs(rAngle) < 90f)
                {
                    vControl.SteerTrack = Vector3.SignedAngle(Car.ForwardVector, FTHalfSec.Direction, Car.UpVector);
                    return;
                }
                else //Physical direction pointing backwards
                {
                    vControl.SteerTrack = Vector3.SignedAngle(Car.ForwardVector, FTHalfSec.Direction, Car.UpVector);
                    return;
                }
            }


            differenceToLane *= ARS.map(Math.Abs(differenceToLane), 0.25f, 0.5f, 0, 1, true);

            vControl.SteerTrack = (rAngle + differenceToLane); //(vehData.CurrentGrip + vehData.CurrentDownforce)
            if (float.IsInfinity(vControl.SteerTrack) || float.IsNaN(vControl.SteerTrack)) vControl.SteerTrack = 0;


            //float maxSteer = ARS.map(ARS.MStoMPH(Car.Velocity.Length()), 120, 20, 3, 45, true);
            //vControl.SteerTrack = ARS.Clamp (vControl.SteerTrack, -maxSteer, maxSteer);
        }


        /// <summary>
        /// Corrects vControl.SteerAngle based off Slide and Spinout issues
        /// </summary>
        void SteerCorrections()
        {
            vControl.SteerCorrection = 0f;

            float mult = ARS.map(Math.Abs(vehData.SlideAngle), 0, handlingData.TRlateral, 0f, 2, true);
            if (mult > 1f) mult = 1f;

            float slideCorrection = vehData.SlideAngle * mult;
            float adjustedRot = (vehData.RotSpeedZ / vehData.CurrentGrip);
            float rotCorrection = adjustedRot * ARS.map(Math.Abs(adjustedRot), 5f, 90f, 0, 0.5f, true);
            vControl.SteerCorrection -= rotCorrection;

            if (vehData.RotSpeedZ > 0 != vehData.SlideAngle > 0) slideCorrection = 0f;
            vControl.SteerCorrection -= slideCorrection;
        }

        /// <summary>
        /// In meters. Negative = outside the track, on the outside of the current corner.
        /// </summary>
        /// <returns></returns>
        float DistToOutside()
        {
            if (Vector3.SignedAngle(trackPoint.Direction, FTHalfSec.Direction, Vector3.WorldUp) < 0.0f) return (float)Math.Round(trackPoint.TrackWide + mem.data.DeviationFromCenter, 1);
            else return (float)Math.Round(trackPoint.TrackWide - mem.data.DeviationFromCenter, 1);
        }

        float DistToInside()
        {
            if (Vector3.SignedAngle(trackPoint.Direction, FTHalfSec.Direction, Vector3.WorldUp) > 0.0f) return (float)Math.Round(trackPoint.TrackWide + mem.data.DeviationFromCenter, 1);
            else return (float)Math.Round(trackPoint.TrackWide - mem.data.DeviationFromCenter, 1);
        }

        public void Launch()
        {
            StuckRecover = false;
            StuckGameTimeRef = 0;
            BaseBehavior = RacerBaseBehavior.Race;
            LapStartTime = Game.GameTime;
            vControl.HandBrakeTime = 0;
            vControl.MaxThrottle = 1f;
            if (team == Team.Cop) Car.SirenActive = true;
        }

        /// <summary>
        /// Translates existing info into a throttle/brake/handbrake input for the car.
        /// </summary>
        void TranslateThrottleBrake()
        {
            //Slow down after finishing the race
            if (BaseBehavior == RacerBaseBehavior.FinishedRace || BaseBehavior == RacerBaseBehavior.FinishedStandStill)
            {
                if (BaseBehavior == RacerBaseBehavior.FinishedStandStill) vControl.Throttle = 0f; else mem.intention.Speed = ARS.MPHtoMS(60);
                if (vControl.Brake > 0.2f) vControl.Brake = 0.2f;
            }

            //Keep still with throttle up when waiting for the launch
            if (BaseBehavior == RacerBaseBehavior.GridWait)
            {
                float power = Function.Call<float>(Hash.GET_VEHICLE_ACCELERATION, Car) * 3.33f; //Torque in first gear
                vControl.Throttle = 0.25f;
                vControl.HandBrakeTime = Game.GameTime + 500;
                return;
            }

            //Limit the th the input to the car's top speed. Cars can surpass defined engine top speed in V
            if (mem.intention.Speed > ARS.EngineTopSpeed(Car) + ARS.MPHtoMS(25)) mem.intention.Speed = ARS.EngineTopSpeed(Car) + ARS.MPHtoMS(25);


            //Catchup - racers going in first will slow down if they get too far ahead
            if (Pos == 1 && ARS.Racers.Count > 1)
            {
                float top = ARS.EngineTopSpeed(Car);
                float bottom = ARS.EngineTopSpeed(Car) * 0.5f;
                float max = ARS.map(ARS.Racers[1].Car.Position.DistanceTo(Car.Position), 50, 25f, bottom, top, true);
                if (mem.intention.Speed > max) mem.intention.Speed = max;
            }

            if (team == Team.Cop)
            {
                Racer r = ARS.Racers.FindAll(a => a.team == Team.None).OrderByDescending(b => b.Pos).FirstOrDefault();
                if (r != null && r.Pos > Pos) mem.intention.Speed = ARS.EngineTopSpeed(Car) * 0.5f;
            }

            //Reverse when stuck
            if (StuckRecover && BaseBehavior == RacerBaseBehavior.Race)
            {
                mem.intention.Speed = -5f;
            }

            //Intention vs Current
            float spdDiff = (float)Math.Round(mem.intention.Speed - Car.Velocity.Length(), 2);

            //Scale the percieved speed difference to account for vehicle power. This partially normallizes expected accelerations across all powerlevels.
            float brakeScale = ARS.map(GroundGripMultiplier, 1, 0.5f, 0.1f, 1, true);
            float throttleScale = ARS.map(Function.Call<float>(Hash.GET_VEHICLE_ACCELERATION, Car), 0.5f, 0.1f, 0.25f, 1f, true);

            vControl.Throttle = (float)Math.Round((vControl.Throttle + (spdDiff * throttleScale)) / 2, 2);
            vControl.Brake = (float)Math.Round((vControl.Brake - (spdDiff * brakeScale)) / 2, 2);

            //Want to go forward
            if (mem.intention.Speed > 0f)
            {
                //Is going forward
                if (mem.data.SpeedVector.Y > 0f)
                {
                    vControl.Throttle = ARS.Clamp(vControl.Throttle, 0f, 1f);
                    vControl.Brake = ARS.Clamp(vControl.Brake, 0f, 1f);
                }
                else //Is going backward
                {
                    vControl.Throttle = 0f;
                    vControl.Brake = 1f;
                    vControl.SteerTrack *= -1;
                }
            }
            else //Want to go backward
            {
                //Is going backward
                if (mem.data.SpeedVector.Y < 0f)
                {
                    vControl.Throttle = ARS.Clamp(vControl.Throttle, -1f, 0);
                    vControl.Brake = 0f;

                    //Make them steer into the track's direction even in reverse
                    vControl.SteerTrack *= -1;
                }
                else //is going forward
                {
                    vControl.Throttle = 0f;
                    vControl.Brake = 1f;
                }
            }


            float scale = 0.10f;
            float steerRef = vControl.SteerCurrent;

            float steerWithRot = 0;
            if (vControl.SteerNoLimit > 0) steerWithRot = steerRef - (vehData.RotSpeedZ * scale);
            if (vControl.SteerNoLimit < 0) steerWithRot = -steerRef + (vehData.RotSpeedZ * scale);

            //Positive = understeer, Negative = oversteer
            vehData.Understeer = ARS.Lerp(vehData.Understeer, steerWithRot, 0.5f);

            //Only use it when actually steering into it (not countersteering)
            if (vehData.SpeedVectorLocal.X > 0 == steerRef > 0 && vehData.Understeer > 0)
            {
                float ogInput = vControl.Throttle;
                float spdAdjust = ARS.map(steerWithRot, 15, 5, -1f, 0, true);
                vControl.Throttle += spdAdjust;

                if (vControl.Throttle < 0)
                {
                    //vControl.Brake += -vControl.Throttle;
                    vControl.Throttle = 0;
                }
            }

            //Projected path method, tries to guess where the racer will be a second later
            int refPointNode = (int)(trackPoint.Node + Car.Velocity.Length());

            if (refPointNode < ARS.TrackPoints.Count())
            {
                TrackPoint refPoint = ARS.TrackPoints[refPointNode];

                float man = 1f;

                //Never brake below 10ms less than the original corner speed                 
                float noLift = refPoint.TrackWide;
                float maxLift = refPoint.TrackWide + 5f;
                float angleToFollowPoint = Vector3.SignedAngle(trackPoint.Direction, FTHalfSec.Direction, Vector3.WorldUp);

                //If we're going to the outside of this corner 
                if (Math.Abs(vControl.SteerCurrent) > 10) //mem.data.SpeedVector.X > 0 == angleToFollowPoint > 0
                {
                    Vector3 projected = Car.Position;
                    projected += Car.Velocity / 2;// (Car.Velocity + (vehData.AccelerationVector.Aggregate((v1, v2) => v1 + v2) / (float)vehData.AccelerationVector.Count));

                    float LoR = ARS.LeftOrRight(projected, refPoint.Position, refPoint.Direction);
                    float LoRCar = ARS.LeftOrRight(projected, Car.Position, Car.Velocity.Normalized);

                    //If our projection expect to be further outside than us right now
                    if (angleToFollowPoint > 0)
                    {
                        man = ARS.map(LoR, maxLift, noLift, -2, 0, true);
                    }
                    else
                    {
                        man = ARS.map(LoR, -maxLift, -noLift, -2, 0, true);
                    }


                    vControl.Throttle += man;

                    if (vControl.Throttle < 0)
                    {
                        //vControl.Brake += -vControl.Throttle;
                        vControl.Throttle = 0;
                    }
                }
            }
        }

        /// <summary>
        /// Translates existing info into a steering input for the car.
        /// </summary>
        void TranslateSteer()
        {

            if (!Mistakes.ContainsKey(Mistake.ForgetSteeringLimiter))
            {
                //maxSteerInput = ARS.map(vControl.Throttle, 1f, 0f,0.4f, 0.6f, true);
            }

            if (float.IsNaN(vControl.SteerTrack) || float.IsInfinity(vControl.SteerTrack)) vControl.SteerTrack = 0f;
            if (float.IsNaN(mem.data.SteerAngle) || float.IsInfinity(mem.data.SteerAngle)) mem.data.SteerAngle = 0f;
            if (float.IsNaN(vControl.SteerManeuver) || float.IsInfinity(vControl.SteerManeuver)) vControl.SteerManeuver = 0f;

            float maxSteer =  ARS.map(Car.Velocity.Length(), ARS.MPHtoMS(30), ARS.MPHtoMS(15), handlingData.TRlateral * 0.66f, SteeringLock, true);
            vControl.SteerTrack = ARS.Clamp(vControl.SteerTrack, -maxSteer, maxSteer);

            float finalSteer = vControl.SteerTrack + vControl.SteerManeuver + vControl.SteerCorrection;
            if (Math.Abs(finalSteer) > 360 || float.IsInfinity(finalSteer)) finalSteer = 0;


            float AngleDif = (float)Math.Round(finalSteer - mem.data.SteerAngle, 3);
            if (float.IsNaN(AngleDif)) AngleDif = 0f;


            float strSpeed = 12;
            if (Math.Abs(AngleDif) < strSpeed)
            {
                mem.data.SteerAngle = finalSteer;
            }
            else
            {
                if (mem.data.SteerAngle < finalSteer) mem.data.SteerAngle += strSpeed; else mem.data.SteerAngle -= strSpeed;
            }

            mem.data.SteerAngle = (float)Math.Round(ARS.Clamp(mem.data.SteerAngle, -SteeringLock, SteeringLock), 3);
            vControl.SteerInput = ARS.map(mem.data.SteerAngle, -SteeringLock, SteeringLock, -1, 1, true);

            if (float.IsNaN(vControl.SteerInput) || float.IsInfinity(vControl.SteerInput)) vControl.SteerInput = 0f;

            vControl.SteerCurrent = mem.data.SteerAngle;
        }

        float SteeringLock = 0f;

        /// <summary>
        /// Figures out the ideal speed to be at at the moment
        /// </summary>
        public void SpeedLogic()
        {
            if (BaseBehavior == RacerBaseBehavior.GridWait)
            {
                mem.intention.Speed = 200f;
                return;
            }
            if (BaseBehavior == RacerBaseBehavior.FinishedRace)
            {
                mem.intention.Speed = 20f;
                return;
            }
            if (BaseBehavior == RacerBaseBehavior.FinishedStandStill)
            {
                mem.intention.Speed = 0f;
                return;
            }


            mem.intention.Speed = AIData.MaxSpeed;

            //Aggro buildup
            float diff = mem.intention.AggroToReach - mem.intention.Aggression;
            if (diff > 0f) mem.intention.Aggression += mem.personality.Rivals.AggressionBuildup / 10; else mem.intention.Aggression -= mem.personality.Rivals.AggressionBuildup / 5;

            if (KnownCorners.Any())
            {
                if (ARS.DevSettingsFile.GetValue<bool>("RACERS", "AllowManeuvers", false) && NearbyRivals.Any() && (KnownCorners.First().Node - trackPoint.Node) / Car.Velocity.Length() < 5f)
                {
                    if (KnownCorners.First().Speed < Car.Velocity.Length() - ARS.MPHtoMS(20))
                    {
                        if (!BannedDecisions.ContainsKey(Decision.LateBrake)) MakeDecision(Decision.LateBrake, (int)(mem.intention.Aggression * 5), 10000, 5000, 5000);
                    }
                    else
                    {
                        if (!BannedDecisions.ContainsKey(Decision.FastCorner)) MakeDecision(Decision.FastCorner, (int)(mem.intention.Aggression * 5), 10000, 5000, 5000);
                    }
                }

                mem.intention.Speed = ARS.MapIdealSpeedForDistance(KnownCorners.First(), this);
            }


            if (Math.Abs(Vector3.SignedAngle(Car.ForwardVector, trackPoint.Direction, Vector3.WorldUp)) > 100f)
            {
                mem.intention.Speed = -5f;
                return;
            }


            if (mem.data.CurveRadiusToFollowPoint != 0f && !float.IsNaN(mem.data.CurveRadiusToFollowPoint))
            {
                float followSpd = ARS.GetSpeedFollowTrack(mem.data.CurveRadiusToFollowPoint, this);
                mem.intention.Speed = Math.Min(mem.intention.Speed, followSpd);
            }

            float elChange = (FTOneSec.Elevation - trackPoint.Elevation) * (Car.Velocity.Length() / 100);
            elChange *= Math.Abs(Vector3.SignedAngle(trackPoint.Direction, FTOneSec.Direction, Vector3.WorldUp)) / 10;
            
            if (float.IsNaN(elChange)) elChange = 0f;
            elChange = ARS.Clamp(elChange, -10, 2);

            mem.intention.Speed += elChange;
            mem.intention.Speed *= ARS.map(FTOneSec.Elevation, -45, 0, 0.5f,1, true);
            
            
        }

        void MakeDecision(Decision d, int chance, int duration, int sCooldown, int fCooldown)
        {
            if (chance == 0 || mem.personality.Rivals.ManeuverExtraGs == 0f || !ARS.DevSettingsFile.GetValue("RACERS", "AllowManeuvers", false)) return;

            if (!BannedDecisions.ContainsKey(d) && !Decisions.Any())
            {
                if (ARS.GetRandomInt(0, 100) <= chance) { Decisions.Add(d, Game.GameTime + duration); BannedDecisions.Add(d, Game.GameTime + duration + sCooldown); }
                else BannedDecisions.Add(d, Game.GameTime + fCooldown);
            }
        }

        void MakeMistake(Mistake m, int chance, int duration, int sCooldown, int fCooldown)
        {
            if (chance == 0 || !ARS.DevSettingsFile.GetValue("RACERS", "AllowMistakes", false)) return;
            if (!BannedMistakes.ContainsKey(m) && !Mistakes.ContainsKey(m))
            {
                if (ARS.GetRandomInt(0, 100) <= chance) { Mistakes.Add(m, Game.GameTime + duration); BannedMistakes.Add(m, Game.GameTime + duration + sCooldown); }
                else BannedMistakes.Add(m, Game.GameTime + fCooldown);
            }
        }

        /// <summary>
        /// Limits vControl.Throttle and vControl.Brake inputs to avoid wheelspin and lockups.
        /// </summary>
        void TractionControl()
        {
            float wheelspin = ARS.GetWheelsMaxWheelspin(Car);

            if (Mistakes.ContainsKey(Mistake.ForgetABS)) vControl.CurrentLockupLimiter = 1f;
            else if (vControl.Brake > 0.0f)
            {
                if (vControl.HandBrakeTime < Game.GameTime)
                {
                    vControl.CurrentLockupLimiter += ARS.map(wheelspin, 4f, 2f, -0.1f, 0.1f, true);
                    vControl.CurrentLockupLimiter = ARS.Clamp(vControl.CurrentLockupLimiter, 0.5f, 1f);

                    if (vControl.CurrentLockupLimiter < 1) MakeMistake(Mistake.ForgetABS, 100 - mem.personality.Stability.Skill, 1000, 4000, 2000);
                }
            }

            if (vControl.Throttle > 0f)
            {
                if (wheelspin < 0f)
                {
                    float min = mem.personality.Stability.WheelspinOnMinSlide;
                    float max = mem.personality.Stability.WheelspinOnMaxSlide;
                    if (Decisions.ContainsKey(Decision.Flatout)) { min += 1.0f; max += 1.0f; }

                    float maxSlide = (handlingData.TRlateral) * mem.personality.Stability.WheelspinMaxSlide;
                    float minSlide = (handlingData.TRlateral) * mem.personality.Stability.WheelspinNoSlide;

                    float allowedWheelspin = max;
                    if (min != max)
                    {
                        if (min > max) allowedWheelspin = ARS.map(Math.Abs(vehData.SlideAngle), maxSlide, minSlide, max, min, true);
                        else ARS.map(Math.Abs(vehData.SlideAngle), minSlide, maxSlide, min, max, true);
                    }

                    float throttleCorrection = ARS.map(Math.Abs(wheelspin), allowedWheelspin + 0.25f, allowedWheelspin - 0.25f, -1f, 1f, true) * (7.5f * Game.LastFrameTime);

                    //Add the correction
                    vControl.MaxThrottle += throttleCorrection;
                }
            }

            vControl.MaxThrottle = ARS.Clamp(vControl.MaxThrottle, 0.1f, 1f);

            //Throttle limiter
            if (!Mistakes.ContainsKey(Mistake.ForgetTCS) && vControl.Throttle > 0f && vControl.MaxThrottle < vControl.Throttle) vControl.Throttle = vControl.MaxThrottle;
        }

        public void UpdateTickData()
        {

            Vector3 cSpeed = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, false);
            vehData.AccelerationVector.Add((cSpeed - lSpeed) / Game.LastFrameTime);
            if (vehData.AccelerationVector.Count > 20) vehData.AccelerationVector.RemoveAt(0);

            lSpeed = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, false);
            vehData.SpeedVectorGlobal = cSpeed;
            vehData.SpeedVectorLocal = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true);
            mem.data.SpeedVector = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true);

            if (trail.Count > 50) trail.RemoveAt(0);
            if (DirChanges.Count >= 5) DirChanges.RemoveAt(0);
            if (!DirChanges.Any()) DirChanges.Add(Car.Velocity.Normalized);

            if (Car.Position.DistanceTo(DirChanges.Last()) >= 1f) DirChanges.Add(Car.Velocity.Normalized);

            float a = 0f;
            for (int i = 1; i < DirChanges.Count; i++)
            {
                a += (float)Math.Round(Vector3.SignedAngle(DirChanges[i], DirChanges[i - 1], Vector3.WorldUp), 10);
            }
            a /= DirChanges.Count;
            TRCurveAngle = a;
            if (float.IsNaN(TRCurveAngle)) TRCurveAngle = 0f;
        }

        public VehData vehData = new VehData();
        public HandlingData handlingData = new HandlingData();
        List<Vector3> DirChanges = new List<Vector3>();

        /// <summary>
        /// Gathers and runs tick-sensitive stuff
        /// </summary>
        public void ProcessTick()
        {
            UpdateTickData();
            DrawStuff();

            if (!Driver.IsPlayer)
            {
                TorqueMult = ARS.map(vehData.SlideAngle, 5f, 90f, 1f, 2 * vehData.CurrentGrip);
                if (TorqueMult > 5) TorqueMult = 5;
                if (TorqueMult > 1.0f && Car.CurrentGear > 0) Car.EngineTorqueMultiplier = TorqueMult;

                ApplyInputs();


                //Catchup
                if (ARS.Racers.Count > 1)
                {
                    if (Pos > ARS.catchupPos)
                    {
                        if (!ARS.SettingsFile.GetValue("CATCHUP", "OnlyLoners", true) || !NearbyRivals.Any())
                        {
                            if (ARS.SettingsFile.GetValue("CATCHUP", "CatchupSpeed", 100) != 0 && vControl.HandBrakeTime < Game.GameTime)
                            {
                                if (ARS.GetPercent(Pos, ARS.Racers.Count) >= 40)//&& !NearbyRivals.Any()
                                {
                                    float max = (float)Math.Round((float)ARS.SettingsFile.GetValue("CATCHUP", "CatchupSpeed", 100) / 1000, 2);
                                    Car.ApplyForceRelative(new Vector3(0, ARS.Clamp(vControl.Throttle, -max, max), 0));
                                }
                            }
                            if (ARS.SettingsFile.GetValue("CATCHUP", "CatchupGrip", 100) != 0)
                            {
                                float max = (float)Math.Round((float)ARS.SettingsFile.GetValue("CATCHUP", "CatchupGrip", 100) / 1000, 2);
                                float v = ARS.Clamp(-mem.data.SpeedVector.X, -max, max);
                                Car.ApplyForceRelative(new Vector3(v, 0, 0));
                            }
                        }
                    }
                }
            }


            //AI updates
            if (TenthSecondTick < Game.GameTime)
            {
                TenthSecondTick = Game.GameTime + 100;
                UpdateTrackInfo();
                UpdateVehicleInfo();
                UpdatePercievedGrip();

                ProcessOther();
                ProcessAI();
                if (Driver.IsPlayer && ARS.SettingsFile.GetValue("CATCHUP", "OnlyBehindPlayer", true)) ARS.catchupPos = Pos;
            }
        }
        void UpdateVehicleInfo()
        {
            BoundingBox = ARS.GetDirectionalBoundingBox(Car);
            vehData.SlideAngle = (float)Math.Round(Vector3.SignedAngle(Car.Velocity.Normalized, Car.ForwardVector, Car.UpVector), 3);
        }
        void ProcessOther()
        {
            if (KnownCorners.Any())
            {
                CornerPoint corner = KnownCorners.First();

                int spd = (int)Car.Velocity.Length();
                if (trackPoint.Node > corner.Node)
                {
                    KnownCorners.RemoveAt(0);
                    if (Decisions.ContainsKey(Decision.LateBrake)) Decisions[Decision.LateBrake] = 0;
                    if (ARS.DevSettingsFile.GetValue<bool>("RACERS", "AllowManeuvers", false) && NearbyRivals.Any())
                    {
                        if (!BannedDecisions.ContainsKey(Decision.Flatout) && NearbyRivals.First().Pos < Pos && Car.CurrentGear <= 3)
                        {
                            MakeDecision(Decision.Flatout, (int)(mem.intention.Aggression * 5), 4000, 8000, 5000);
                        }
                        if (KnownCorners.Any() && !BannedDecisions.ContainsKey(Decision.EarlyExit) && (KnownCorners.Last().Node - trackPoint.Node) / Car.Velocity.Length() < 4)
                        {
                            MakeDecision(Decision.EarlyExit, (int)(mem.intention.Aggression * 5), 4000, 8000, 5000);
                        }
                    }
                }
            }
            else
            {
                if (Decisions.ContainsKey(Decision.LateBrake)) Decisions[Decision.LateBrake] = 0;
                if (Decisions.ContainsKey(Decision.FastCorner)) Decisions[Decision.FastCorner] = 0;
            }


            //To avoid vanilla contact-swerves, sit the driver on the passenger seat if there's a potential colission. Also try and fix the helmet
            if (Car.Model.IsCar && !Driver.IsPlayer)
            {
                if (NearbyRivals.Any() && Car.IsInRangeOf(NearbyRivals.First().Car.Position, 5f))
                {
                    if (Driver.IsInVehicle(Car) && Car.IsSeatFree(VehicleSeat.RightFront))
                    {
                        Driver.Alpha = 0;
                        Driver.SetIntoVehicle(Car, VehicleSeat.RightFront);
                    }
                }
                else
                {
                    if (Driver.IsInVehicle(Car) && Car.IsSeatFree(VehicleSeat.Driver))
                    {
                        Driver.Alpha = 255;
                        Driver.SetIntoVehicle(Car, VehicleSeat.Driver);

                        Driver.CanWearHelmet = true;
                        Driver.GiveHelmet(true, HelmetType.RegularMotorcycleHelmet, 0);
                        Driver.RemoveHelmet(true);
                    }
                }
            }

            //Ghosting
            if (Car.Alpha != 255) Car.ResetAlpha();
            if (ARS.SettingsFile.GetValue<bool>("GENERAL_SETTINGS", "Ghosts", false) && NearbyRivals.Any())
            {
                foreach (Racer r in NearbyRivals)
                {
                    //if (Function.Call<bool>(Hash.IS_ENTITY_AT_ENTITY, Car, r.Car, (r.Car.Model.GetDimensions().X * 1.5f), (r.Car.Model.GetDimensions().Y * 1.5f), (r.Car.Model.GetDimensions().Z * 1.5f), true, true, true))
                    if (Car.IsInRangeOf(r.Car.Position, 6))
                    {
                        Function.Call(Hash.SET_ENTITY_NO_COLLISION_ENTITY, r.Car, Car, true);
                        Car.Alpha = 150;
                    }
                }
            }


            //Reset into track logic
            if ((Car.IsUpsideDown || Math.Abs(mem.data.DeviationFromCenter) > trackPoint.TrackWide) && !ControlledByPlayer && BaseBehavior == RacerBaseBehavior.Race)
            {
                if (TimeOutOfTrack == 0) TimeOutOfTrack = Game.GameTime;
                else if (Game.GameTime - TimeOutOfTrack > 4000 && Car.Velocity.Length() < 5f)
                {
                    TimeOutOfTrack = 0;
                    ResetIntoTrack();
                }
            }
            else
            {
                if (TimeOutOfTrack != 0) TimeOutOfTrack = 0;
            }
        }

        public void ApplyInputs()
        {
            //AI Inputs
            if (Driver.IsSittingInVehicle(Car) && !Driver.IsPlayer)
            {
                if (vControl.HandBrakeTime > Game.GameTime) Car.HandbrakeOn = true; else Car.HandbrakeOn = false;

                ARS.SetThrottle(Car, vControl.Throttle);
                ARS.SetBrakes(Car, vControl.Brake);
                ARS.SetSteerAngle(Car, vControl.SteerInput);
            }
            else
            {
                ARS.SetThrottle(Car, 0f);
                ARS.SetBrakes(Car, 0f);
                ARS.SetSteerInput(Car, 0f);
            }
        }

        Vector3 lSpeed;
        void DrawStuff()
        {

            if (!Car.IsInRangeOf(Game.Player.Character.Position, 100)) return;

            //Future 
            Vector3 projected = Car.Position;
            projected += Car.Velocity / 2;


            if (Driver.IsPlayer && Lap >= ARS.SettingsFile.GetValue<int>("GENERAL_SETTINGS", "Laps", 5) && CanRegisterNewLap)
            {
                World.DrawMarker(MarkerType.CheckeredFlagRect, ARS.TrackPoints.First().Position + new Vector3(0, 0, 5f), ARS.TrackPoints.First().Direction, new Vector3(0, 0, 0), new Vector3(5f, 5f, 5f), Color.White);// DrawLine(vm,last, Color.Black);
            }

            //ARS.DrawLine(trackPoint.Position + new Vector3(0, 0, 0.5f), FTHalfSec.Position + new Vector3(0, 0, 0.5f), Color.Red);
            //ARS.DrawLine(FTHalfSec.Position + new Vector3(0, 0, 0.5f), FTHalfSec.Position + (FTHalfSec.Direction * 5) + new Vector3(0, 0, 0.5f), Color.Blue);
            //if(oneSecAhead!= null) ARS.DrawLine(oneSecAhead.Position + new Vector3(0, 0, 0.5f), oneSecAhead.Position + (oneSecAhead.Direction * 5) + new Vector3(0, 0, 0.5f), Color.Green);

            /*
            if (oldNode != trackPoint.Node)
            {

                if (Driver.IsPlayer)
                {
                    List<int> PartTimeNodes = new List<int>();
                    PartTimeNodes.Add((int)((25 * ARS.TrackPoints.Count) / 100));
                    PartTimeNodes.Add((int)((50 * ARS.TrackPoints.Count) / 100));
                    PartTimeNodes.Add((int)((75 * ARS.TrackPoints.Count) / 100));

                    if (PartTimeNodes.Contains(trackPoint.Node))
                    {
                        TimeSpan lapTime = ARS.ParseToTimeSpan(Game.GameTime - LapStartTime);
                       UI.Notify("Part Time (" + Math.Round((ARS.GetPercent(trackPoint.Node, ARS.TrackPoints.Count))) + "%)~n~~b~" + lapTime.ToString("m':'ss'.'fff"));

                    }
                }
            }
            */

            if (ARS.OptionValuesList[Options.ShowPhysics])
            {

                //Center of Gs
                World.DrawMarker(MarkerType.DebugSphere, Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.1f, 0.1f, 0.1f), Color.Green, false, false, 0, false, "", "", false);

                //Gs
                Vector3 avgGs = vehData.AccelerationVector.Aggregate(new Vector3(0, 0, 0), (s, v) => s + v) / (float)vehData.AccelerationVector.Count;
                avgGs.Z = 0f;

                float colorPercent = ARS.map(avgGs.Length() / 9.8f, 0, vehData.CurrentGrip, 0, 100, true);
                Color gColor = ARS.GradientAtoBtoC(Color.White, Color.Yellow, Color.Red, colorPercent);

                World.DrawMarker(MarkerType.DebugSphere, Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)) + (avgGs / 9.8f), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.15f, 0.15f, 0.15f), gColor, false, false, 0, false, "", "", false);
                ARS.DrawLine(Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)) + (avgGs / 9.8f), Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)), gColor);


                Vector3 maxValues = new Vector3(vehData.CurrentGrip * 9.8f, vehData.CurrentGrip * 9.8f, vehData.CurrentGrip * 9.8f);
                Vector3 max = Vector3.Clamp(avgGs, -maxValues, maxValues);

                //if (avgGs.Length() / 9.8f > vehData.WheelsGrip) 

                Vector3 maxLength = (avgGs.Normalized * (vehData.CurrentGrip * 9.8f)) / 9.8f;
                //World.DrawMarker(MarkerType.DebugSphere, Car.Position + new Vector3(0, 0, Car.Model.GetDimensions().Z * 0.6f) + maxLength, Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.15f, 0.15f, 0.15f), Color.Red, false, false, 0, false, "", "", false);
                //ARS.DrawLine(Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)) + maxLength, Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)), gColor);


                //World.DrawMarker(MarkerType.DebugSphere, Car.Position + (Car.Velocity), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), Color.Blue, false, false, 0, false, "", "", false);
                //ARS.DrawLine(Car.Position, Car.Position + (Car.Velocity), Color.Blue);

                //World.DrawMarker(MarkerType.DebugSphere, Car.Position + (Car.Velocity + avgGs), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), Color.Green, false, false, 0, false, "", "", false);
                //ARS.DrawLine(Car.Position + (Car.Velocity), Car.Position + (Car.Velocity + avgGs), Color.Green);

                //ARS.DrawLine(Car.Position, visualSteerPoint, Color.Red);
                //World.DrawMarker(MarkerType.DebugSphere, visualSteerPoint, Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.15f, 0.15f, 0.15f), Color.Red, false, false, 0, false, "", "", false);

                Vector3 source = Car.Position + new Vector3(0, 0, 0.5f + (Car.Model.GetDimensions().Z * 0.6f));
                ARS.DrawText(source, "~b~" + Math.Round(ARS.MStoMPH(Car.Velocity.Length())).ToString() + "~w~mph~n~~y~" + Math.Round(avgGs.Length() / 9.8f, 2) + "Gs", Color.White, 0.5f);


            }

            if (!Driver.IsPlayer)
            {

                if (!Car.IsInRangeOf(Game.Player.Character.Position, 250f)) return;

                //ARS.DrawText(Car.Position + new Vector3(0, 0, 2), vehData.Understeer + "º", Color.SkyBlue, 0.5f);

                /*
                int z = 0;
                for (int i = this.followTrackPoint.Node; i > this.followTrackPoint.Node - 20; i--)
                {
                    if (i == 0 && ARS.TrackPoints.Count>i) break;
                    TrackPoint tr = ARS.TrackPoints[i];
                    ARS.DrawText(tr.Position + new Vector3(0,0,z),  ""+Math.Round(tr.AvgCurveRadius), Color.White, 0.5f);

                }
                */
                if (ARS.OptionValuesList[Options.ShowAggro] && !Driver.IsPlayer && Driver.IsInRangeOf(Game.Player.Character.Position, 100f))
                {
                    if (Decisions.Any()) World.DrawMarker(MarkerType.ChevronUpx2, Car.Position + new Vector3(0, 0, 1.5f), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.5f, 0.5f, -0.5f), ARS.GetColorFromRedYellowGreenGradient(100 - (mem.intention.Aggression * 100)), false, true, 0, false, "", "", false);
                    else World.DrawMarker(MarkerType.ChevronUpx1, Car.Position + new Vector3(0, 0, 1.5f), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.5f, 0.5f, -0.5f), ARS.GetColorFromRedYellowGreenGradient(100 - (mem.intention.Aggression * 100)), false, true, 0, false, "", "", false);
                }

                if (Decisions.Any() && ARS.OptionValuesList[Options.ShowInputs])
                {
                    string text = "";
                    foreach (Decision d in Decisions.Keys) text += d.ToString() + "~n~";
                    ARS.DrawText(Car.Position + new Vector3(0, 0, 2), text, Color.SkyBlue, 0.5f);
                }

                if (Mistakes.Any() && ARS.OptionValuesList[Options.ShowInputs])
                {
                    string text = "";
                    foreach (Mistake d in Mistakes.Keys) text += d.ToString() + "~n~";
                    ARS.DrawText(Car.Position + new Vector3(0, 0, 1), text, Color.Orange, 0.5f);
                }

                /*
                if (BannedMistakes.Any() && ARS.OptionValuesList[Options.ShowInputs])
                {
                    string text = "";
                    foreach (Mistake d in BannedMistakes.Keys) text += d.ToString() + "~n~";
                    ARS.DrawText(Car.Position + new Vector3(0, 0, 2), text, Color.White, 0.5f);
                }
                */

                if (trail.Count == 0) trail.Add(Car.Position);
                else if (Car.Position.DistanceTo(trail[trail.Count - 1]) > 2f) trail.Add(Car.Position);


                if (ARS.DebugVisual == (int)5 && NearbyRivals.Any()) ARS.DrawLine(Car.Position, NearbyRivals.First().Car.Position, Color.White);

                if (ARS.OptionValuesList[Options.ShowInputs])
                {


                    /*
                    World.DrawMarker(MarkerType.DebugSphere, LaneLookahead, Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), Color.Green);
                    ARS.DrawLine(Car.Position, LaneLookahead, Color.Green);

                    World.DrawMarker(MarkerType.DebugSphere, CornerPrepLookahead, Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), Color.Red);
                    ARS.DrawLine(LaneLookahead, CornerPrepLookahead, Color.Red);

                    World.DrawMarker(MarkerType.DebugSphere, HugCorner, Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), Color.Blue);
                    ARS.DrawLine(LaneLookahead, HugCorner, Color.Blue);
                    */

                    //Vector3 dev = Car.Position + (Vector3.Cross(Car.ForwardVector, Vector3.WorldUp) * (mem.intention.LookaheadDeviationPrepForCorner - mem.data.DeviationFromCenter)) ;
                    //if (mem.intention.LookaheadDeviationPrepForCorner != 0) World.DrawMarker(MarkerType.DebugSphere, dev, Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), Color.Red);

                    //dev = Car.Position + (Vector3.Cross(Car.ForwardVector, Vector3.WorldUp) * (mem.intention.LookaheadDeviationFromCenter-mem.data.DeviationFromCenter));
                    //if (mem.intention.LookaheadDeviationFromCenter != 0) World.DrawMarker(MarkerType.DebugSphere, dev, Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), Color.Green);

                    Color cc = Color.Green;
                    if (vControl.Brake > 0.0f) cc = Color.Yellow;
                    if (vControl.Brake > 0.5f) cc = Color.Orange;
                    if (vControl.Brake > 0.9f) cc = Color.Red;


                    Vector3 velocity = Car.Velocity.Normalized;
                    Vector3 back = Car.ForwardVector;
                    back.Z = velocity.Z;

                    Vector3 Dimensions = Car.Model.GetDimensions();
                    Vector3 inputplace = Car.Position + new Vector3(0, 0, -Car.HeightAboveGround);// new Vector3(0, 0, (Dimensions.Z * 0.6f));
                    Vector3 steergoal = Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * mem.data.SteerAngle) * -Car.ForwardVector; // Quaternion.RotationAxis takes radian angles                                                                                                                                                 
                    Vector3 idealsteergoal = Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * vControl.SteerTrack) * -Car.ForwardVector; // Quaternion.RotationAxis takes radian angles

                    float dimension = Car.Model.GetDimensions().Y + 1f;

                    if (vControl.Brake != 0f || true)
                    {
                        Vector3 inputBrake = inputplace + (Car.ForwardVector * -(Dimensions.Y * 0.5f));
                        Color c = Color.Red;
                        int alpha = (int)ARS.map(vControl.Brake, 0, 1f, 0, 255, true);
                        World.DrawMarker(MarkerType.ChevronUpx1, inputBrake, Car.ForwardVector, new Vector3(-90, 0, 0), new Vector3(dimension / 2, dimension / 4, (dimension / 2)), Color.FromArgb(alpha, c), false, false, 0, false, "", "", false);
                    }

                    if (vControl.Throttle != 0f || true)
                    {
                        Vector3 inputThrottle = inputplace + (Car.ForwardVector * (Dimensions.Y * 0.5f));
                        Color c = Color.GreenYellow;
                        int alpha = (int)ARS.map(vControl.Throttle, 0, 1, 0, 255, true);
                        //World.DrawMarker(MarkerType.ChevronUpx1, inputThrottle, -Car.ForwardVector, new Vector3(90, vControl.SteerAngle(), 0), new Vector3(dimension / 2, dimension / 4, -(dimension / 2)), Color.FromArgb(alpha, c), false, false, 0, false, "", "", false);
                        World.DrawMarker(MarkerType.ChevronUpx1, inputThrottle, -Car.ForwardVector, new Vector3(90, vControl.SteerCurrent, 0), new Vector3(dimension / 2, dimension / 4, -(dimension / 2)), Color.FromArgb(alpha, c), false, false, 0, false, "", "", false);
                    }
                }

                if (ARS.OptionValuesList[Options.ShowTrackAnalysis])
                {
                    Vector3 source = Car.Position + new Vector3(0, 0, 0.5f + (Car.Model.GetDimensions().Z * 0.6f));
                    //ARS.DrawText(source, "~b~" + Math.Round(ARS.MStoMPH(Car.Velocity.Length())).ToString() + "~w~mph~n~~y~" + Math.Round(avgGs.Length() / 9.8f, 2) + "Gs", Color.White, 0.5f);


                    if (KnownCorners.Any() && KnownCorners.Count > 0)
                    {
                        int d = 0;
                        foreach (CornerPoint c in KnownCorners)
                        {
                            //CornerPoint c = KnownCorners.First();
                            //if (c.Node % 2 == 1) continue;
                            //if (c.Node > KnownCorners.First().Node + KnownCorners.First().AvgScale) break;

                            d++;
                            Vector3 wp = ARS.Path[c.Node];
                            float expectedSpeed = c.Speed;
                            Color gColor = ARS.GetColorFromRedYellowGreenGradient(ARS.map(expectedSpeed - Car.Velocity.Length(), -1, 1, 0, 100, true));



                            if (c.IsKey)
                            {
                                World.DrawMarker(MarkerType.ChevronUpx1, ARS.TrackPoints[c.Node].Position + new Vector3(0, 0, 1.5f), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.5f, 0.5f, -0.5f), Color.SkyBlue, false, true, 0, false, "", "", false);
                            }


                            ARS.DrawLine(source, wp, gColor);
                            //ARS.DrawText(wp + new Vector3(0, 0, 1), Math.Round(c.Angle, 1) + "º~n~~b~" + Math.Round(ARS.MStoMPH(c.Speed)) + "~w~mph~n~~y~" + Math.Round(vehData.CurrentGrip + ARS.GetDownforceGsAtSpeed(this, c.Speed), 2) + "~w~Gs~n~~w~L: ~b~" + (c.Length), Color.White, ARS.map(d, 10, 1, 0.15f, 0.5f, true)); //*(c[5]+1f)
                            ARS.DrawText(wp + new Vector3(0, 0, 1),"Elevation Change: "+ Math.Round(c.ElevationChange, 1) + "º", Color.White, ARS.map(d, 10, 1, 0.15f, 0.5f, true)); //*(c[5]+1f)


                            World.DrawMarker(MarkerType.ChevronUpx1, wp, ARS.TrackPoints[c.Node].Direction, new Vector3(90, 0, 0), new Vector3(ARS.TrackPoints[c.Node].TrackWide * 2.5f, 5, 5), Color.FromArgb(50, gColor.R, gColor.G, gColor.B));
                            if (c.Node - c.LengthStart > 0) World.DrawMarker(MarkerType.ChevronUpx1, ARS.TrackPoints[c.Node - c.LengthStart].Position, ARS.TrackPoints[c.Node - c.LengthStart].Direction, new Vector3(90, 0, 0), new Vector3(ARS.TrackPoints[c.Node].TrackWide * 2.5f, 5, 5), Color.FromArgb(50, gColor.R, gColor.G, gColor.B));
                            if (c.Node + c.LenghtEnd < ARS.CornerPoints.Count()) World.DrawMarker(MarkerType.ChevronUpx1, ARS.TrackPoints[c.Node + c.LenghtEnd].Position, ARS.TrackPoints[c.Node + c.LenghtEnd].Direction, new Vector3(90, 0, 0), new Vector3(ARS.TrackPoints[c.Node].TrackWide * 2.5f, 5, 5), Color.FromArgb(50, gColor.R, gColor.G, gColor.B));

                            if (c.Node - c.Length > 0 && c.Node + c.Length < ARS.CornerPoints.Count()) ARS.DrawLine(ARS.TrackPoints[c.Node - c.Length].Position, ARS.TrackPoints[c.Node + c.Length].Position, Color.Black);
                        }

                    }

                    int node = trackPoint.Node;
                    int size = (FTHalfSec.Node - trackPoint.Node) * 4;
                    for (int i = node; i < node + size; i++)
                    {
                        if (i % 2 == 1) continue;

                        if (i > 0 && i < ARS.TrackPoints.Count)
                        {
                            TrackPoint cp = ARS.TrackPoints[i];
                            World.DrawMarker(MarkerType.ChevronUpx1, ARS.TrackPoints[cp.Node].Position - new Vector3(0, 0, 0.05f), ARS.TrackPoints[cp.Node].Direction, new Vector3(90, 0, 0), new Vector3(ARS.TrackPoints[cp.Node].TrackWide * 2.5f, 5, 5), Color.FromArgb(10, Color.White));

                        }
                    }
                }

                if (ARS.DebugVisual == (int)DebugDisplay.Positioning)
                {

                    Vector3 inLane = trackPoint.Position + (Vector3.Cross(trackPoint.Direction, Vector3.WorldUp) * mem.data.DeviationFromCenter) + Vector3.WorldUp;
                    ARS.DrawLine(inLane + (trackPoint.Direction * 5), inLane - (trackPoint.Direction * 5), Color.SkyBlue);

                    //Track limits display
                    /*
                    Color blue =  Color.FromArgb(50, Color.Blue);
                    int fNode = (int)ARS.Clamp(trackPoint.Node + 500, 0,ARS.TrackPoints.Count()-1);
                    for (int i = trackPoint.Node; i < fNode; i++)
                    {
                        if (i % 2 == 1)
                        {
                            TrackPoint tPoint = ARS.TrackPoints[i];
                            Vector3 right = tPoint.Position + (Vector3.Cross(tPoint.Direction, Vector3.WorldUp) * tPoint.TrackWide) + (Vector3.WorldUp * 2);
                            Vector3 left = tPoint.Position + (Vector3.Cross(tPoint.Direction, Vector3.WorldUp) * -tPoint.TrackWide) + (Vector3.WorldUp * 2);

                            World.DrawMarker(MarkerType.ChevronUpx1, right, tPoint.Direction, new Vector3(89, 0, -90), new Vector3(5, 1f, 2f), blue);
                            World.DrawMarker(MarkerType.ChevronUpx1, left, tPoint.Direction, new Vector3(89, 0, -90), new Vector3(5, 1f, 2f), blue);
                        }
                    }
                    */
                }
            }
        }

        /// <summary>
        /// How far from the outside of the track we're in. Negative values = we're inside the track.
        /// </summary>
        /// <returns>Distance in meters</returns>
        float OutOfTrackDistance()
        {
            return (Math.Abs(mem.data.DeviationFromCenter) + (BoundingBox / 2)) - trackPoint.TrackWide;
        }

        void VehicleInteractions()
        {
            DebugText.Clear();
            if (Car.Driver.IsPlayer || ARS.SettingsFile.GetValue<bool>("GENERAL_SETTINGS", "Ghosts", false)) return;

            if (NearbyRivals.Any())
            {
                //AI shouldn't be aggresive while crowded
                if (NearbyRivals.Where(r => r.Car.IsInRangeOf(Car.Position, 40)).Count() > 2)
                {
                    mem.intention.AggroToReach = 0f;
                }
                else
                {
                    Racer rival = NearbyRivals.First();
                    float aggroDist = rival.Car.Position.DistanceTo(Car.Position);

                    float Min = 50;
                    float Max = 100;

                    //If the rival is behind us, worry less about them
                    if (rival.Pos > Pos) { Min = 10; Max = 50; }

                    //Max aggro based on distance
                    mem.intention.AggroToReach = ARS.map(aggroDist, Max, Min, 0f, 1f, true);
                }
            }
            else mem.intention.AggroToReach = 0f;


            mem.AvoidAvgLane = 0;
            foreach (Rival r in mem.Rivals)
            {
                r.Update(this);

                r.AvoidStr = 0f;
                r.AvoidLane = 0;

                if (r.relativePos == RelativePos.Unreachable) continue;

                float personalSpace = Math.Abs(r.RivalRacer.mem.data.DeviationFromCenter - mem.data.DeviationFromCenter);

                //Base maneuver
                Vector2 man = new Vector2(0, 1);
                float cap = FTHalfSec.TrackWide * (BoundingBox * 0.5f);
                float offset = r.BoundingBoxTotal.X+2;

                if (r.relativePos == RelativePos.Ahead)
                {
                    //Speed control
                    if (personalSpace < offset)
                    {
                        float[] m = { 1, ARS.map(r.sToReach, 0, 4, 0, 1, true), ARS.map(r.Distance, 0, 5, 0, 1, true) };
                        //man.Y = m.Min();
                    }

                    //Steer away from the track limit
                    //if (str > 0 != r.RivalRacer.mem.data.DeviationFromCenter > 0 && Math.Abs(r.RivalRacer.mem.data.DeviationFromCenter) + r.BoundingBoxTotal.X > trackPoint.TrackWide) str *= -1f;

                    if (Math.Abs(r.DirectionDiff) < 15)
                    {

                        r.AvoidLane = r.RivalRacer.mem.data.DeviationFromCenter + (r.rPos.X > 0 ? -offset : offset);
                        
                        if (r.AvoidLane > cap || r.AvoidLane < -cap) r.AvoidLane = r.RivalRacer.mem.data.DeviationFromCenter + (r.rPos.X > 0 ? offset : -offset);

                        float[] m = { ARS.map(r.sToReach, 3, 1, 0, 1f, true), ARS.map(r.Distance, 6, 2, 0, 1f, true) };
                        r.AvoidStr = (float)Math.Round(m.Max(), 1);
                    }
                }
                else
                {
                    if (r.relativePos == RelativePos.Left || r.relativePos == RelativePos.Right)
                    {
                        
                        if (personalSpace < offset)
                        {
                            r.AvoidLane = r.RivalRacer.mem.data.DeviationFromCenter + (r.rPos.X > 0 ? -offset : offset);

                            r.AvoidStr =  ARS.map(personalSpace, offset, 0, 0.5f, 1.5f, true);

                            r.AvoidLane = ARS.Clamp(r.AvoidLane, -cap, cap);
                            //if (r.rPos.Y < 0) r.AvoidStr *= 0.5f;

                            //if (r.rPos.X < 0 != (mem.data.DeviationFromCenter - r.AvoidLane) > 0) r.AvoidLane = 0;

                            //if (mem.intention.LookaheadDeviationFromCenter > 0 == r.AvoidLane > 0) r.AvoidLane = 0;
                        }
                    }
                    if (r.AvoidLane == 0) r.AvoidStr = 0;
                }
                //mem.intention.Maneuvers.Add(man);

                //Average the lane stuff

                if (r.AvoidLane != 0f)
                {
                    if (mem.AvoidAvgLane == 0) mem.AvoidAvgLane = r.AvoidLane;
                    else mem.AvoidAvgLane = ARS.Lerp(mem.AvoidAvgLane, r.AvoidLane, 0.5f);
                }



                //if (Pos > 1) UI.ShowSubtitle(Name + ":~n~Rivals: "+mem.Rivals.Where(ri => ri.RivalRacer!=null).Count()+" ("+mem.Rivals.First().relativePos.ToString()+")~n~Maneuver: " + man.ToString()+"~n~"+r.sToReach);

            }

        }

        bool WouldMoveOutOfTrack(float maneuver)
        {
            //if we're already close to the outside and the maneuver turns us further there
            if (Math.Abs(mem.data.DeviationFromCenter) + Car.Model.GetDimensions().X > trackPoint.TrackWide && maneuver > 0 == mem.data.DeviationFromCenter > 0) return true;

            return false;
        }
        public bool FinishedPointToPoint = false;
        public int localSPDLimiter = 0;
        public float lookAheadHalfSec = 0;

        public TrackPoint trackPoint = new TrackPoint();
        public TrackPoint FTHalfSec = new TrackPoint();
        public TrackPoint FTQuarterSec = new TrackPoint();

        public TrackPoint FTOneSec = new TrackPoint();

        /// <summary>
        /// Figure out our point in the track.
        /// </summary>
        public void UpdateTrackInfo()
        {

            //Get our current track point data
            int cNode = trackPoint.Node;
            if (cNode > ARS.TrackPoints.Count - 20) trackPoint = ARS.TrackPoints.First();

            else trackPoint = ARS.TrackPoints.Skip(cNode).Take(20).OrderBy(t => t.Position.DistanceTo(Car.Position)).First();
            mem.data.DeviationFromCenter = ARS.LeftOrRight(Car.Position, trackPoint.Position, trackPoint.Direction);


            int quarter = (int)(Car.Velocity.Length() * 0.25f);
            if (trackPoint.Node + quarter >= ARS.TrackPoints.Count) FTQuarterSec = ARS.TrackPoints[(int)quarter];
            else FTQuarterSec = ARS.TrackPoints[trackPoint.Node + (int)quarter];


            lookAheadHalfSec = (int)(Car.Velocity.Length() * 0.5);
            lookAheadHalfSec = ARS.Clamp(lookAheadHalfSec, 1, 2000);

            if (trackPoint.Node + lookAheadHalfSec >= ARS.TrackPoints.Count) FTHalfSec = ARS.TrackPoints[(int)lookAheadHalfSec];
            else FTHalfSec = ARS.TrackPoints[trackPoint.Node + (int)lookAheadHalfSec];

            int reference = (int)(Car.Velocity.Length());
            if (trackPoint.Node + reference >= ARS.TrackPoints.Count) FTOneSec = ARS.TrackPoints[reference];
            else FTOneSec = ARS.TrackPoints[trackPoint.Node + reference];

            //Lap counting
            if (CanRegisterNewLap)
            {
                if (ARS.GetPercent(trackPoint.Node, ARS.TrackPoints.Count) < 10 || (ARS.IsPointToPoint && ARS.GetPercent(trackPoint.Node, ARS.TrackPoints.Count) > 99 && ARS.GetOffset(Car, ARS.TrackPoints.Last().Position).Y < 0f))
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
                        if (Driver.IsPlayer) UI.Notify("Laptime: ~b~" + lapTime.ToString("m':'ss'.'fff"));
                        LapTimes.Add(lapTime);
                        LapStartTime = Game.GameTime;
                    }
                }
            }
            else if (BaseBehavior == RacerBaseBehavior.Race && ARS.GetPercent(trackPoint.Node, ARS.TrackPoints.Count) > 50) CanRegisterNewLap = true;

            mem.data.CurveRadiusToFollowPoint = FTHalfSec.AvgCurveRadius;
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
            if (SpdBasedTick < Game.GameTime)
            {
                SpdBasedTick = Game.GameTime + (2000 - ((int)Car.Velocity.Length() * 20));
            }

            if (HalfSecondTick < Game.GameTime)
            {
                HalfSecondTick = Game.GameTime + 500 + (int)ARS.map(Car.Velocity.Length(), 0, 100, -250, 250, true);
                ARS.WorstCornerAhead(this);

                //Remove any corners too fast for us to account for
                if (KnownCorners.Any())
                {
                    KnownCorners = KnownCorners.OrderBy(v => ARS.MapIdealSpeedForDistance(v, this)).ToList();
                    KnownCorners.RemoveAll(v => v.Node < KnownCorners.First().Node || v.Speed > Car.Velocity.Length() + ARS.MPHtoMS(50)); //|| v.Speed > Car.Velocity.Length() + ARS.MPHtoMS(30)
                }

                //Decision cleaning
                if (Decisions.Any(de => de.Value < Game.GameTime)) Decisions.Remove(Decisions.First(de => de.Value < Game.GameTime).Key);
                if (BannedDecisions.Any(de => de.Value < Game.GameTime)) BannedDecisions.Remove(BannedDecisions.First(de => de.Value < Game.GameTime).Key);

                //Mistake cleaning
                if (Mistakes.Any(de => de.Value < Game.GameTime)) Mistakes.Remove(Mistakes.First(de => de.Value < Game.GameTime).Key);
                if (BannedMistakes.Any(de => de.Value < Game.GameTime)) BannedMistakes.Remove(BannedMistakes.First(de => de.Value < Game.GameTime).Key);

                if (ARS.DevSettingsFile.GetValue<bool>("RACERS", "AllowManeuvers", false) && !BannedDecisions.ContainsKey(Decision.NoOvertake) && KnownCorners.Any() && NearbyRivals.Where(r => r.Car.IsInRangeOf(Car.Position, BoundingBox * 4)).Count() >= 3)
                {
                    MakeDecision(Decision.NoOvertake, 100 - (int)(mem.intention.Aggression * 100), 2500, 2500, 2500);
                }

                if (!Driver.IsPlayer) if (NearbyRivals.Count > 0) Driver.Task.LookAt(NearbyRivals[0].Driver, 2000); else if (Car.Velocity.Length() > 5f) Driver.Task.LookAt(Car.Position + Car.Velocity, 2000);
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

                    //Task the driver to get back into the vehicle if they're out
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
                if (!KnownCorners.Any())
                {
                    if (BaseBehavior == RacerBaseBehavior.Race && Vector3.Angle(Car.ForwardVector, trackPoint.Direction) < 5f && Math.Abs(mem.data.SpeedVector.X) < 0.1f && Math.Abs(vControl.SteerCurrent) < 10f) Function.Call((Hash)0x81E1552E35DC3839, Car, true);
                }

                if (Function.Call<bool>((Hash)0x3D34E80EED4AE3BE, Car) && vControl.Brake > 0.1f) Function.Call((Hash)0x81E1552E35DC3839, Car, false);


                if (ARS.DevSettingsFile.GetValue<int>("RACERS", "AIRacerAutofix", 1) == 2 && Function.Call<bool>(Hash._IS_VEHICLE_DAMAGED, Car))
                {
                    Car.Repair();
                }


                //Stuck behavior
                if (vehData.Gs.Length() < 0.1f && Car.Velocity.Length() < 1f && BaseBehavior == RacerBaseBehavior.Race)
                {
                    if (StuckGameTimeRef == 0) StuckGameTimeRef = Game.GameTime;
                    if (!Driver.IsPlayer && BaseBehavior == RacerBaseBehavior.Race && Driver.IsSittingInVehicle(Car) && vControl.HandBrakeTime < Game.GameTime)
                    {
                        if (Game.GameTime - StuckGameTimeRef >= 2)
                        {
                            if (Game.GameTime - StuckGameTimeRef >= 4)
                            {
                                if (Driver.IsSittingInVehicle(Car) && !Car.IsInWater && Car.EngineHealth > 0)
                                {
                                    StuckGameTimeRef = 0;
                                    ResetIntoTrack();
                                    StuckRecover = false;
                                }
                            }
                            else if (!StuckRecover && !Car.Model.IsBike)
                            {
                                LastStuckPlace = Car.Position;
                                if (ARS.DebugVisual > 0) UI.Notify("~b~" + Car.FriendlyName + " tries to recover");
                                StuckRecover = true;
                            }
                        }
                    }
                }

                if (StuckRecover && (!Car.IsInRangeOf(LastStuckPlace, 5f) || mem.data.SpeedVector.Y > 3f))
                {
                    StuckRecover = false;
                    StuckGameTimeRef = 0;
                }
            }
        }


        /// <summary>
        /// Runs Speeding and Steering logic.
        /// To run every 100ms.
        /// </summary>
        public void ProcessAI()
        {
            ProcessTimedAI();

            if (BaseBehavior == RacerBaseBehavior.GridWait && vControl.HandBrakeTime < Game.GameTime) vControl.HandBrakeTime = Game.GameTime + (100 * ARS.GetRandomInt(2, 6));

            if (!ControlledByPlayer)
            {
                mem.intention.Corrections.Clear();
                mem.intention.Maneuvers.Clear();
                VehicleInteractions();


                //Base Steer
                SteerTrack();
                vControl.SteerNoLimit = vControl.SteerTrack;


                SpeedLogic();
                //SpeedCorrections();


                //Steer Maneuvers
                if (mem.data.SpeedVector.Y > 0f)
                {
                    //Maneuvers && Corrections
                    if (mem.intention.Maneuvers.Any() || mem.intention.Corrections.Any())
                    {
                        float ma = 0;
                        foreach (Vector2 m in mem.intention.Maneuvers.Concat(mem.intention.Corrections))
                        {
                            ma += m.X;
                        }
                        vControl.SteerTrack += ma;// (ma * ARS.map(Math.Abs(ma), 0, SteeringLock * 0.25f, 0f, 1f, true));
                    }
                }

                //float maxSteer = handlingData.TRlateral * ARS.map(Car.Velocity.Length(), 30f, 5f, 0.75f, 1, true);

                //vControl.SteerTrack = ARS.Clamp(vControl.SteerTrack, -maxSteer, maxSteer);


                SteerCorrections();
                TranslateThrottleBrake();
                TranslateSteer();
                TractionControl();
            }
        }


        void ResetIntoTrack()
        {
            vControl.SteerTrack = 0f;
            mem.data.SteerAngle = 0f;

            Car.Position = ARS.Path[trackPoint.Node] + new Vector3(0, 0, 3);

            Car.Heading = trackPoint.Direction.ToHeading();

            StuckRecover = false;
            LastStuckPlace = Vector3.Zero;

            Car.Speed = ARS.MPHtoMS(30);
        }

        void UpdatePercievedGrip()
        {

            //Base vehicle grip
            float HandlingGrip = Function.Call<float>((Hash)0xA132FB5370554DB0, Car); //GetVehicleMaxTraction
            HandlingGrip = ARS.Clamp(HandlingGrip, 0.5f, 3f);

            GroundGripMultiplier = (float)Math.Round(ARS.GetWheelsGrip(Car).Average(), 3); //Surface grip


            /*
            float wetavg = ARS.GetWheelsWetgrip(Car).Average();
            if (wetavg < 1.0 && 1 == 2)
            {
                //UI.ShowSubtitle("Wet: -" + ((1 - wetavg) * 4), 500);
                HandlingGrip *= (SurfaceGrip - ((1 - wetavg) * ARS.GetNumWheels(Car)));
            }
            */

            vehData.BaseGrip = HandlingGrip;
            vehData.CurrentGrip = HandlingGrip * GroundGripMultiplier;
            vehData.CurrentDownforce = ARS.GetDownforceGsAtSpeed(this, Car.Velocity.Length());


            //If you're in first, map terrain multipliers for the rest of the racers
            if (Pos <= 2 && !ARS.MultiplierInTerrain.ContainsKey(trackPoint.Node))
            {
                ARS.MultiplierInTerrain.Add(trackPoint.Node, GroundGripMultiplier);
            }


            //Checks for Z movement to judge wether the car is stable or not. Usually implies the vehicle is mid-air
            if (Math.Abs(vehData.SpeedVectorLocal.Normalized.Z) > 0.05f) // || TimeOutOfTrack != 0
            {
                if (vehData.AvgGroundStability >= 0.75f) vehData.AvgGroundStability -= 0.025f;
            }
            else if (vehData.AvgGroundStability < 1f) vehData.AvgGroundStability += 0.025f;

            if (vehData.AvgGroundStability > 1f) vehData.AvgGroundStability = 1f;

            vehData.Gs = vehData.AccelerationVector.Aggregate(new Vector3(0, 0, 0), (s, v) => s + v) / (float)vehData.AccelerationVector.Count;
            vehData.RotSpeedZ = ARS.rad2deg(Function.Call<Vector3>(Hash.GET_ENTITY_ROTATION_VELOCITY, Car).Z);
            mem.data.CurveRadiusPhysicalGs = ARS.GetCurveRadius(Car.Position - Car.Velocity + (vehData.Gs / 2), Car.Position + Car.Velocity + (vehData.Gs / 2), Car.Position);


        }


        /// <summary>
        /// Handles speed related corrections (maneuvers, overshooting)
        /// </summary>
        void SpeedCorrections()
        {

            /*
            //Overshooting            
            if (vehData.SlideAngle > 0 == vControl.SteerTrack > 0 && Car.Speed > 10)
            {
                float m = ARS.map(Math.Abs(vControl.SteerNoLimit), handlingData.TRlateral * 2f, handlingData.TRlateral * 0.5f, -1, 1, true);
                if (m < 1)
                {
                    UI.ShowSubtitle(m.ToString(), 200);
                    if (KnownCorners.Any())
                    {
                        float minimumSpd = KnownCorners.First().Speed - 5;
                        if (mem.intention.Speed + m < minimumSpd)
                        {
                            m = mem.intention.Speed - minimumSpd;
                            UI.ShowSubtitle("~o~"+m.ToString(), 200);
                        }
                    }
                    // 50 - X < 45
                    // X = 50 - 45
                    mem.intention.Maneuvers.Add(new Vector2(0, m));

                }
            }
            */



            /*
            //Projected path method, tries to guess where the racer will be a second later
            int refPointNode = (int)(trackPoint.Node + Car.Velocity.Length());

            if (false && refPointNode < ARS.TrackPoints.Count())
            {
                TrackPoint refPoint = ARS.TrackPoints[refPointNode];

                float man = 1f;

                //Never brake below 10ms less than the original corner speed            +(Car.Velocity.Length()/2)
                float minSpd = ARS.GetSpeedForCorner(ARS.CornerPoints[(int)(trackPoint.Node)], this) - 15f;
                float noLift = refPoint.TrackWide;
                float maxLift = refPoint.TrackWide + 5;

                //If we're going to the outside of this corner 
                if (mem.data.SpeedVector.X > 0 == ARS.CornerPoints[followTrackPoint.Node].Angle > 0)
                {
                    Vector3 projected = Car.Position + (Car.Velocity + ((vehData.AccelerationVector.Aggregate(new Vector3(0, 0, 0), (s, v) => s + v) / (float)vehData.AccelerationVector.Count)) / 2);
                    float LoR = ARS.LeftOrRight(projected, refPoint.Position, refPoint.Direction);
                    float LoRCar = ARS.LeftOrRight(projected, Car.Position, Car.Velocity.Normalized);

                    //If our projection expect to be further outside than us right now
                    if (Math.Abs(LoR) > Math.Abs(mem.data.DeviationFromCenter))
                    {
                        if (ARS.CornerPoints[trackPoint.Node].Angle > 0)
                        {
                            man = ARS.map(LoR, maxLift, noLift, -1, 1, true);
                        }
                        else
                        {
                            man = ARS.map(LoR, -maxLift, -noLift, -1, 1, true);
                        }
                    }



                    //Makes sure the maneuver does not try and stop the car below certain speed.
                    //Helps smoothing the behavior, and provides some common sense.
                    //(if overshooting you wouldn't try and brake to a stop, instead slowing down to something below the intended speed, but not THAT slow)
                    if (man != 1)
                    {
                        if (Car.Velocity.Length() + man < minSpd)
                        {
                            //   UI.ShowSubtitle("~o~Tryna go below min speed", 200);
                            float tSpd = (Car.Velocity.Length() + man) - minSpd;
                            man = ARS.map(tSpd, 1, -1, -1, 1, true);
                        }
                    }

                    //If we're on the outside and going further outside
                    if (Math.Abs(mem.data.DeviationFromCenter) > refPoint.TrackWide && Math.Abs(LoR) > Math.Abs(mem.data.DeviationFromCenter))
                    {
                        float safeTRLat = 1f;
                        if (Math.Abs(TRCurveAngle) > safeTRLat)
                        {
                            man = 1;
                            //   UI.ShowSubtitle("~g~Gs " + Math.Round(Math.Abs(TRCurveAngle), 1), 200);
                        }
                        else
                        {
                            //  UI.ShowSubtitle("~r~ Gs " + Math.Abs(TRCurveAngle), 200);
                            man = ARS.map(safeTRLat - Math.Abs(TRCurveAngle), 1f, 0, 0f, 1, true);
                        }
                    }
                    if (man != 1)
                    {
                        man = ARS.Clamp(man, Car.Velocity.Length() - AIData.MinSpeed, 900);
                        //UI.ShowSubtitle("~r~" + Math.Round(man, 1), 200);
                        mem.intention.Corrections.Add(new Vector2(0, man));
                    }
                }
            }


            */




            //Speed Maneuvers
            if (mem.intention.Maneuvers.Any() || mem.intention.Corrections.Any())
            {
                float speedMod = mem.intention.Maneuvers.Concat(mem.intention.Corrections).OrderBy(v => v.Y).First().Y;

                //If its less than full throttle
                if (speedMod < 1f)
                {
                    //If it results in actual reduction of speed
                    if (Car.Velocity.Length() + speedMod < mem.intention.Speed)
                    {
                        mem.intention.Speed = Car.Velocity.Length() + speedMod;
                    }
                }
                else
                {
                    //mem.intention.Speed += speedMod;
                }
            }
        }


        public List<Vehicle> Traffic = new List<Vehicle>();
        public void UpdateRivals()
        {

            NearbyRivals.Clear();
            List<Racer> Candidates = new List<Racer>();
            foreach (Racer r in ARS.Racers)
            {
                if (r.Car.Handle != Car.Handle && r.Car.Position.DistanceTo(Car.Position) < 100f)
                {
                    Candidates.Add(r);
                }
            }

            Candidates = Candidates.OrderBy(v => Vector3.Distance(v.Car.Position, Car.Position)).ToList();
            if (Candidates.Count() > 3) Candidates = Candidates.Take(3).ToList();

            NearbyRivals = Candidates;

            for (int i = 0; i < mem.Rivals.Count - 1; i++)
            {
                mem.Rivals[i].RivalRacer = null;
                if (NearbyRivals.Count - 1 < i) return;

                mem.Rivals[i].RivalRacer = NearbyRivals[i];

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
