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
        Vector3 visualSteerPoint = Vector3.Zero; //Debug, visual location where the car is aiming with the steer.
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

        //Current Traction Curve Angle based off last direction changes
        float TRCurveAngle = 0f;

        //Stuck stuff
        int StuckScore = 0;
        public bool StuckRecover = false;
        int TimeOutOfTrack = 0;

        //Perceived Info to make decisions
        List<Racer> NearbyRivals = new List<Racer>();
        public List<CornerPoint> KnownCorners = new List<CornerPoint>();

        //Handling stuff
        public float SurfaceGrip = 1f;

        float CurrentLockupLimiter = 1f;
        float CurrentWheelspinLimiter = 1f;

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

            if (Car.Model.IsBike) Driver.CanFlyThroughWindscreen = true; else Driver.CanFlyThroughWindscreen = false;

            Driver.MaxHealth = 1000;
            Driver.Health = 1000;
            Driver.CanSufferCriticalHits = false;
        }



        public void Initialize()
        {
            handlingData.Downforce = ARS.GetDownforce(Car);
            handlingData.TRlateral = ARS.rad2deg(ARS.GetTRCurveLat(Car));
            handlingData.BrakingAbility = Car.MaxBraking;
            SteeringLock = ARS.rad2deg(ARS.GetSteerLock(Car));

            vControl.SteerAngle = 0f;
            trackPoint = ARS.TrackPoints.Last();
            mem.data.SteerAngle = 0f;
            vControl.Brake = 0f;
            vControl.Throttle = 0f;
            LapTimes.Clear();
            LapStartTime = 0;
            BaseBehavior = RacerBaseBehavior.GridWait;
            Lap = 0;
            FinishedPointToPoint = false;

            Car.Repair();
        }

        /// <summary>
        /// Neccesary calculations for the car to follow the track. The whole function results in vControl.SteerAngle.
        /// </summary>
        public void SteerTrack()
        {
            float ogAngle = mem.data.SteerAngle;

            if (BaseBehavior == RacerBaseBehavior.GridWait || BaseBehavior == RacerBaseBehavior.FinishedStandStill)
            {
                vControl.SteerAngle = 0f;
                return;
            }

            //Offsets the aim to keep the car on the outside when approaching corners WHILE being in a curve already
            float creepToOutside = 0f;

            //Get to the outside to approach the corner properly
            if ((!mem.intention.Maneuvers.Any() || mem.intention.Maneuvers.OrderBy(v => v.X).First().X < 0.1f))
            {
                if (ARS.CornerPoints.Any() && trackPoint != null)
                {
                    if (KnownCorners.Any())
                    {
                        int dist = KnownCorners.First().Node - trackPoint.Node;
                        int sToReach = (int)(dist / (Car.Velocity.Length()));

                        //Only priorize it if the corner is at a reasonable distance and don't try to get to the outside while too close already KnownCorners.First().Speed < Car.Velocity.Length() + (vehData.GsAcceleration * sToReach) && 
                        if (sToReach < 5f && sToReach > 1f)
                        {
                            float Wide = followTrackPoint.TrackWide - (BoundingBox / 2) - 1;
                            float Outside = (Wide * (KnownCorners.First().Angle > 0 ? 1 : -1));                            
                            float lane = ARS.Clamp((Outside - mem.data.DeviationFromCenter)/2, -2, 2);

                            if (impedimentOnSide != 0f && impedimentOnSide > 0 != lane > 0f)
                            {
                                lane *= ARS.map(Math.Abs(impedimentOnSide), mem.personality.Rivals.SideToSideMinDist, mem.personality.Rivals.SideToSideMinDist + 4, 0f, 4f, true);
                            }
                            else if (fImpedimentOnSide != 0f && fImpedimentOnSide > 0 != lane > 0f)
                            {
                                lane *= ARS.map(Math.Abs(fImpedimentOnSide), mem.personality.Rivals.SideToSideMinDist, mem.personality.Rivals.SideToSideMinDist + 4, 0f, 4f, true);
                            }

                            //Account for acceleration but not deceleration
                            if (vehData.GsAcceleration > 0f) creepToOutside = lane * ARS.map(Car.Velocity.Length() + (vehData.GsAcceleration * sToReach), KnownCorners.First().Speed * 0.75f, KnownCorners.First().Speed * 1.25f, 0, 1, true);
                            else creepToOutside = lane * ARS.map(Car.Velocity.Length(), KnownCorners.First().Speed * 0.75f, KnownCorners.First().Speed * 1.25f, 0, 1, true);
                        }
                    }
                }
            }

            //Basic deviation>steer trnslation
            float dev = ARS.Clamp(mem.data.DeviationFromCenter, -(trackPoint.TrackWide - BoundingBox), trackPoint.TrackWide - BoundingBox);
            Vector3 inLane = followTrackPoint.Position + (Vector3.Cross(followTrackPoint.Direction, Car.UpVector) * (dev + creepToOutside));
            inLane.Z = followTrackPoint.Position.Z;
            if (mem.intention.LookaheadDeviationFromCenter != 0.0f) inLane = followTrackPoint.Position + (Vector3.Cross(followTrackPoint.Direction, Car.UpVector) * (mem.intention.LookaheadDeviationFromCenter + creepToOutside));


            //Steering to go for.
            float rAngle = Vector3.SignedAngle(Car.Velocity.Normalized, (inLane - Car.Position).Normalized, Vector3.WorldUp);
            if (Car.Model.IsBike) rAngle *= 0.5f;

            //Steering behavior if we are going backwards.
            if (mem.data.SpeedVector.Y < 0f)
            {
                //Physical direction actually pointing to the track
                if (Math.Abs(rAngle) < 90f)
                {
                    vControl.SteerAngle = Vector3.SignedAngle(Car.ForwardVector, followTrackPoint.Direction, Car.UpVector);
                    return;
                }
                else //Physical direction pointing backwards
                {
                    vControl.SteerAngle = Vector3.SignedAngle(Car.ForwardVector, followTrackPoint.Direction, Car.UpVector);
                    return;
                }
            }

            vControl.SteerAngle = (float)Math.Round((rAngle) / 2, 5);
            vControl.SteerAngle *= ARS.map(Math.Abs(vControl.SteerAngle), 0, 7.5f, 0, 1, true);
            vControl.SteerAngle = ARS.Clamp(vControl.SteerAngle, -SteeringLock, SteeringLock);

            visualSteerPoint = inLane;
        }

        /// <summary>
        /// Corrects vControl.SteerAngle based off Slide and Spinout issues
        /// </summary>
        void SteerCorrections()
        {
            if (!Car.Model.IsBicycle && !Car.Model.IsBike)
            {
                vehData.RotationVectorLocal.Z = (float)Math.Round(ARS.rad2deg(Function.Call<Vector3>(Hash.GET_ENTITY_ROTATION_VELOCITY, Car).Z), 1);

                if (Car.Velocity.Length() > 3f)
                {
                    //Slide                    
                    if (Math.Abs(vehData.SlideAngle) > 0.0f)
                    {
                        float minSlide = 0f;
                        float maxSlide = handlingData.TRlateral;
                        float maxCorrection = mem.personality.Stability.CounterFactor * 2;



                        //Degrees of correction
                        float correction = vehData.SlideAngle * ARS.map(Math.Abs(vehData.SlideAngle), minSlide, maxSlide, 0f, maxCorrection, false);
                        float correctionCap = Math.Abs(vehData.SlideAngle) * mem.personality.Stability.MaxAbsoluteCounter;

                        if (Mistakes.ContainsKey(Mistake.ForgetCounterSteer)) correction *= 0.25f;

                        //The overcorrection allows the driver to recover from the slide
                        correction = ARS.Clamp(correction, -correctionCap, correctionCap);

                        if (vehData.RotationVectorLocal.Z > 0 != vehData.SlideAngle > 0) correction *= 0f;
                        else vControl.SteerAngle -= correction;


 

                        if (Math.Abs(correction) > 5f) MakeMistake(Mistake.ForgetCounterSteer, 100 - mem.personality.Stability.Skill, 2000, 4000, 2000);
                    }



                    //Spinout
                    if (!Mistakes.ContainsKey(Mistake.ForgetSpinoutPrevention) && Math.Abs(vehData.RotationVectorLocal.Z) > Math.Abs(TRCurveAngle))
                    {
                        float degreesPerSecond = TRCurveAngle * Car.Velocity.Length();
                        float rotExceed = (float)Math.Round((Math.Abs(vehData.RotationVectorLocal.Z)) - (Math.Abs(degreesPerSecond)), 1) - mem.personality.Stability.SpinoutSafeRotSpeed;
                        if (vehData.RotationVectorLocal.Z < 0) rotExceed = -rotExceed;

                        float correct = (float)Math.Round(ARS.map(Math.Abs(rotExceed), 0f, mem.personality.Stability.SpinoutMaxExtraRotSpeed, 0f, mem.personality.Stability.SpinoutMaxCounterDegrees, true), 3);
                        correct *= ARS.map(vehData.WheelsGrip, 3, 2, 0.5f, 1f, true);

                        if (rotExceed > 0f == vehData.RotationVectorLocal.Z > 0f)
                        {
                            //UI.ShowSubtitle("Spinout of 20º+ ~o~" + Math.Round(rotExceed, 2)+"º", 200);

                            float correction = correct * (vehData.RotationVectorLocal.Z > 0 ? 1 : -1);
                            vControl.SteerAngle -= correction;

                            if (Math.Abs(rotExceed) > 5f) MakeMistake(Mistake.ForgetSpinoutPrevention, 100 - mem.personality.Stability.Skill, 2000, 4000, 2000);
                            /*
                            float throttleManeuver = ARS.map(Math.Abs(rotExceed), mem.personality.Stability.SpinoutUnsafeExtraRotSpeed, 0, 0, ARS.AIData.SpeedToInput, true);
                            if (throttleManeuver != ARS.AIData.SpeedToInput)
                            {
                                mem.intention.Maneuvers.Add(new Vector2(0, throttleManeuver));
                            }
                            */
                        }
                    }
                }
            }
        }

        //In meters. Negative = outside the track, on the outside of the current corner.
        float DistToOutside()
        {
            if (Vector3.SignedAngle(trackPoint.Direction, followTrackPoint.Direction, Vector3.WorldUp) < 0.0f) return (float)Math.Round(trackPoint.TrackWide + mem.data.DeviationFromCenter, 1);
            else return (float)Math.Round(trackPoint.TrackWide - mem.data.DeviationFromCenter, 1);
        }

        float DistToInside()
        {
            if (Vector3.SignedAngle(trackPoint.Direction, followTrackPoint.Direction, Vector3.WorldUp) > 0.0f) return (float)Math.Round(trackPoint.TrackWide + mem.data.DeviationFromCenter, 1);
            else return (float)Math.Round(trackPoint.TrackWide - mem.data.DeviationFromCenter, 1);
        }

        public void Launch()
        {
            StuckRecover = false;
            StuckScore = 0;
            BaseBehavior = RacerBaseBehavior.Race;
            LapStartTime = Game.GameTime;
            vControl.HandBrakeTime = 0;
            CurrentWheelspinLimiter = 1f;
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
                vControl.Throttle = ARS.GetPercent(vehData.WheelsGrip / 2, power) / 100; //Assumes its two wheel drive
                vControl.HandBrakeTime = Game.GameTime + 500;
                return;
            }

            //Limit the th the input to the car's top speed. Cars can surpass defined engine top speed in V
            if (mem.intention.Speed > ARS.EngineTopSpeed(Car)) mem.intention.Speed = ARS.EngineTopSpeed(Car) + ARS.MPHtoMS(10);

            //Catchup - racers going in first will slow down if they get too far ahead
            if (ARS.SettingsFile.GetValue("CATCHUP", "LeaderTopSpeed", 75) !=100 && Pos == 1 && ARS.Racers.Count > 1)
            {
                float max = ARS.map(ARS.Racers[1].Car.Position.DistanceTo(Car.Position), 200f, 50f, ARS.EngineTopSpeed(Car) * (ARS.SettingsFile.GetValue("CATCHUP", "LeaderTopSpeed", 75)/100), ARS.EngineTopSpeed(Car), true);
                if (mem.intention.Speed > max) mem.intention.Speed = max;
            }

            //Reverse when stuck
            if (StuckRecover && BaseBehavior == RacerBaseBehavior.Race)
            {
                mem.intention.Speed = -5f;
            }

            //Intention vs Current
            float spdDiff = (float)Math.Round(mem.intention.Speed - mem.data.SpeedVector.Y, 1);

            //Throttle to Intention
            vControl.Throttle = (float)Math.Round((vControl.Throttle + ARS.map(spdDiff, -ARS.AIData.SpeedToInput, ARS.AIData.SpeedToInput, -1f, 1f, true)) / 2, 2);
            vControl.Brake = (float)Math.Round((vControl.Brake + ARS.map(spdDiff, 0f, -ARS.AIData.SpeedToInput, 0f, 1f, true)) / 2, 2);

            //Input limits
            vControl.Brake = ARS.Clamp(vControl.Brake, 0f, 1f);

            //Want to go forward
            if (mem.intention.Speed > 0f)
            {
                //Is going forward
                if (mem.data.SpeedVector.Y > -2f)
                {
                    vControl.Throttle = ARS.Clamp(vControl.Throttle, 0f, 1f);
                    vControl.Brake = ARS.Clamp(vControl.Brake, 0f, 1f);
                }
                else //Is going backward
                {
                    vControl.Throttle = 0f;
                    vControl.Brake = 1f;
                }
            }
            else //Want to go backward
            {
                //Is going forward
                if (mem.data.SpeedVector.Y > 0f)
                {
                    vControl.Throttle = 0f;
                    vControl.Brake = 1f;
                }
                else //is going backward
                {
                    vControl.Throttle = ARS.Clamp(vControl.Throttle, -1f, 0);
                    vControl.Brake = 0f;


                    //Make them steer into the track's direction even in reverse
                    vControl.SteerAngle *= -1;
                }

            }
        }

        /// <summary>
        /// Translates existing info into a steering input for the car.
        /// </summary>
        void TranslateSteer()
        {
            if (float.IsNaN(vControl.SteerAngle)) vControl.SteerAngle = 0f;
            if (float.IsNaN(mem.data.SteerAngle)) mem.data.SteerAngle = 0f;

            float AngleDif = (float)Math.Round(vControl.SteerAngle - mem.data.SteerAngle, 10);
            if (float.IsNaN(AngleDif)) AngleDif = 0f;

            float aimSpeedThreshold = ARS.DevSettingsFile.GetValue<int>("RACERS", "SteeringSpeedLimit", 90) / 10;
            if (Car.Model.IsBike) aimSpeedThreshold = 60 / 10;
            if (Math.Abs(AngleDif) < aimSpeedThreshold)
            {
                mem.data.SteerAngle = vControl.SteerAngle;
            }
            else
            {
                if (mem.data.SteerAngle < vControl.SteerAngle) mem.data.SteerAngle += aimSpeedThreshold; else mem.data.SteerAngle -= aimSpeedThreshold;
            }
            mem.data.SteerAngle = (float)Math.Round(ARS.Clamp(mem.data.SteerAngle, -SteeringLock, SteeringLock), 10);

            vControl.SteerInput = ((float)Math.Pow((Math.Abs(mem.data.SteerAngle) / (SteeringLock)), 1f) * (mem.data.SteerAngle > 0 ? 1 : -1));
            vControl.SteerInput = (float)Math.Round(ARS.Clamp(vControl.SteerInput, -1f, 1f), 10);
            if (float.IsNaN(vControl.SteerInput) || float.IsInfinity(vControl.SteerInput)) vControl.SteerInput = 0f;
        }

        float SteeringLock = 0f;

        /// <summary>
        /// Figures out the ideal speed to be at at the moment
        /// </summary>
        public void SpeedLogic()
        {
            float Currentspeed = Car.Velocity.Length();

            mem.intention.Speed = ARS.AIData.MaxSpeed;

            //Aggro buildup
            float diff = mem.intention.AggroToReach - mem.intention.Aggression;
            if (diff > 0f) mem.intention.Aggression += mem.personality.Rivals.AggressionBuildup / 10; else mem.intention.Aggression -= mem.personality.Rivals.AggressionBuildup / 5;

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


            if (KnownCorners.Any())
            {
                KnownCorners = KnownCorners.OrderBy(v => ARS.MapIdealSpeedForDistance(v, this, true)).ToList();
                if (ARS.DevSettingsFile.GetValue<bool>("RACERS", "AllowManeuvers", false) && NearbyRivals.Any() && (KnownCorners.First().Node - trackPoint.Node) / Car.Velocity.Length() < 5f)
                {
                    if (KnownCorners.First().Speed < Car.Velocity.Length() - ARS.MPHtoMS(20))
                    {
                        if (!BannedDecisions.ContainsKey(Decision.LateBrake)) MakeDecision(Decision.LateBrake, (int)(mem.intention.Aggression * 75), 10000, 5000, 5000);
                    }
                    else
                    {
                        if (!BannedDecisions.ContainsKey(Decision.FastCorner)) MakeDecision(Decision.FastCorner, (int)(mem.intention.Aggression * 75), 10000, 5000, 5000);
                    }
                }
            }


            if (Math.Abs(Vector3.SignedAngle(Car.ForwardVector, trackPoint.Direction, Vector3.WorldUp)) > 100f)
            {
                mem.intention.Speed = -5f;
                return;
            }
            else
            {
                if (KnownCorners.Any())
                {
                    float finalSpd = ARS.MapIdealSpeedForDistance(KnownCorners.First(), this);

                    if (finalSpd < KnownCorners.First().Speed) finalSpd = KnownCorners.First().Speed;
                    mem.intention.Speed = finalSpd;
                }
            }
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
            if (Mistakes.ContainsKey(Mistake.ForgetABS)) CurrentLockupLimiter = 1f;
            else if (vControl.Brake > 0.0f)
            {
                if (vControl.HandBrakeTime < Game.GameTime)
                {
                    CurrentLockupLimiter += ARS.map(wheelspin, 4f, 2f, -0.1f, 0.1f, true);
                    CurrentLockupLimiter = ARS.Clamp(CurrentLockupLimiter, 0.5f, 1f);

                    if (CurrentLockupLimiter < 1) MakeMistake(Mistake.ForgetABS, 100 - mem.personality.Stability.Skill, 1000, 4000, 2000);

                }
            }


            float maxCorrection = 0.2f;

            if (Mistakes.ContainsKey(Mistake.ForgetTCS)) CurrentWheelspinLimiter = 1f;
            else if (vControl.Throttle > 0f)
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

                    float throttleCorrection = ARS.map(Math.Abs(wheelspin), allowedWheelspin + 0.5f, allowedWheelspin - 0.5f, -maxCorrection, maxCorrection, true);

                    //Add the correction
                    CurrentWheelspinLimiter += ARS.Clamp(throttleCorrection, -0.1f, 0.2f);

                    if (throttleCorrection < maxCorrection * 0.75f) MakeMistake(Mistake.ForgetTCS, 100 - mem.personality.Stability.Skill, 1000, 4000, 2000);
                }
                else CurrentWheelspinLimiter += maxCorrection;
            }

            CurrentWheelspinLimiter = ARS.Clamp(CurrentWheelspinLimiter, 0.25f, 1f);

            //Throttle limiter
            if (vControl.Throttle > 0f && CurrentWheelspinLimiter < vControl.Throttle) vControl.Throttle = CurrentWheelspinLimiter;
            if (CurrentWheelspinLimiter > vControl.Throttle) CurrentWheelspinLimiter = vControl.Throttle;
        }

        public void GatherTickData()
        {



            //Gather info
            Vector3 cSpeed = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, false);
            vehData.GsAcceleration = (vehData.GsAcceleration + ((cSpeed - lSpeed).Length() / Game.LastFrameTime) * (cSpeed.Length() > lSpeed.Length() ? 1 : -1)) / 2;
            vehData.AccelerationVector.Add((cSpeed - lSpeed) / Game.LastFrameTime);
            if (vehData.AccelerationVector.Count > 10) vehData.AccelerationVector.RemoveAt(0);

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
        public void ProcessTick()
        {
            GatherTickData();
            DrawStuff();

            //AI updates
            if (TenthSecondTick < Game.GameTime)
            {
                TenthSecondTick = Game.GameTime + 100 + (Pos * 2);
                GetTrackInfo();
                ProcessAI();
                if (Driver.IsPlayer && ARS.SettingsFile.GetValue("CATCHUP", "OnlyBehindPlayer", true)) ARS.catchupPos = Pos;
            }

            if (!Driver.IsPlayer)
            {
                //Apply catchup torque cheat
                //if (TorqueMult != 1.0f) Car.EngineTorqueMultiplier = TorqueMult;

                CornerDecisions();
                ApplyInputs();

                if (ARS.Racers.Count > 1)
                {
                    if (Pos>ARS.catchupPos)
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


        }

        public void CornerDecisions()
        {

            //Corner invalidation & Corner decisions
            if (KnownCorners.Any())
            {
                //int invalidateDist = (int)ARS.map(KnownCorners.First().Speed - Car.Velocity.Length(), -ARS.AIData.SpeedToInput, 0, 0, KnownCorners.First().Speed, true);
                //int invalidateDist = (int)Car.Velocity.Length() / 2;

                int invalidateDist = (int)(Car.Velocity.Length() * ARS.map(DistToInside(), trackPoint.TrackWide, trackPoint.TrackWide*0.25f, 0.2f, 0.8f, true));
                if (Decisions.ContainsKey(Decision.EarlyExit)) invalidateDist = (int)Car.Velocity.Length();

                if (KnownCorners.First().Node <= trackPoint.Node + invalidateDist && Vector3.Angle(Car.ForwardVector, ARS.TrackPoints[KnownCorners.First().Node].Direction) < 40)
                {
                    KnownCorners.RemoveAt(0);
                    if (Decisions.ContainsKey(Decision.LateBrake)) Decisions[Decision.LateBrake] = 0;
                    if (ARS.DevSettingsFile.GetValue<bool>("RACERS", "AllowManeuvers", false) && NearbyRivals.Any())
                    {
                        if (!BannedDecisions.ContainsKey(Decision.Flatout) && NearbyRivals.First().Pos < Pos && Car.CurrentGear <= 3)
                        {
                            MakeDecision(Decision.Flatout, (int)(mem.intention.Aggression * 75), 4000, 8000, 5000);
                        }
                        if (KnownCorners.Any() && !BannedDecisions.ContainsKey(Decision.EarlyExit) && (KnownCorners.Last().Node - trackPoint.Node) / Car.Velocity.Length() < 4)
                        {
                            MakeDecision(Decision.EarlyExit, (int)(mem.intention.Aggression * 75), 4000, 8000, 5000);
                        }
                    }
                }
            }
            else
            {
                if (Decisions.ContainsKey(Decision.LateBrake)) Decisions[Decision.LateBrake] = 0;
                if (Decisions.ContainsKey(Decision.FastCorner)) Decisions[Decision.FastCorner] = 0;
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

            if (Driver.IsPlayer && Lap >= ARS.SettingsFile.GetValue<int>("GENERAL_SETTINGS", "Laps", 5) && CanRegisterNewLap)
            {
                World.DrawMarker(MarkerType.CheckeredFlagRect, ARS.TrackPoints.First().Position + new Vector3(0, 0, 5f), ARS.TrackPoints.First().Direction, new Vector3(0, 0, 0), new Vector3(5f, 5f, 5f), Color.White);// DrawLine(vm,last, Color.Black);
            }

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

                //Center
                World.DrawMarker(MarkerType.DebugSphere, Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.1f, 0.1f, 0.1f), Color.Green, false, false, 0, false, "", "", false);


                //Gs
                Vector3 avgGs = vehData.AccelerationVector.Aggregate(new Vector3(0, 0, 0), (s, v) => s + v) / (float)vehData.AccelerationVector.Count;
                avgGs.Z = 0f;
                float colorPercent = ARS.map(avgGs.Length() / 9.8f, 0, vehData.WheelsGrip, 0, 100, true);
                Color gColor = ARS.GradientAtoBtoC(Color.White, Color.Yellow, Color.Red, colorPercent);

                World.DrawMarker(MarkerType.DebugSphere, Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)) + (avgGs / 9.8f), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.15f, 0.15f, 0.15f), gColor, false, false, 0, false, "", "", false);                
                ARS.DrawLine(Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)) + (avgGs / 9.8f), Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)), gColor);


                Vector3 maxValues = new Vector3(vehData.WheelsGrip * 9.8f, vehData.WheelsGrip * 9.8f, vehData.WheelsGrip * 9.8f);
                Vector3 max = Vector3.Clamp(avgGs, -maxValues, maxValues);

                //if (avgGs.Length() / 9.8f > vehData.WheelsGrip) 
                
                    Vector3 maxLength = (avgGs.Normalized * (vehData.WheelsGrip * 9.8f)) / 9.8f;
                    World.DrawMarker(MarkerType.DebugSphere, Car.Position + new Vector3(0, 0, Car.Model.GetDimensions().Z * 0.6f) + maxLength, Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.15f, 0.15f, 0.15f), Color.Red, false, false, 0, false, "", "", false);
                //ARS.DrawLine(Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)) + maxLength, Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)), gColor);


                World.DrawMarker(MarkerType.DebugSphere, Car.Position + (Car.Velocity), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), Color.Blue, false, false, 0, false, "", "", false);
                ARS.DrawLine(Car.Position, Car.Position + (Car.Velocity ), Color.Blue);

                World.DrawMarker(MarkerType.DebugSphere, Car.Position + (Car.Velocity+avgGs), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), Color.Green, false, false, 0, false, "", "", false);
                ARS.DrawLine(Car.Position +( Car.Velocity) , Car.Position + (Car.Velocity+avgGs), Color.Green);

                //ARS.DrawLine(Car.Position, visualSteerPoint, Color.Red);
                //World.DrawMarker(MarkerType.DebugSphere, visualSteerPoint, Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.15f, 0.15f, 0.15f), Color.Red, false, false, 0, false, "", "", false);

                Vector3 source = Car.Position + new Vector3(0, 0, 0.5f + (Car.Model.GetDimensions().Z * 0.6f));
                ARS.DrawText(source, "~b~" + Math.Round(ARS.MStoMPH(Car.Velocity.Length())).ToString() + "~w~mph~n~~y~" + Math.Round(avgGs.Length() / 9.8f, 2) + "Gs", Color.White, 0.5f);

            }

            if (!Driver.IsPlayer)
            {

                if (!Car.IsInRangeOf(Game.Player.Character.Position, 250f)) return;

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
                    Vector3 idealsteergoal = Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * vControl.SteerAngle) * -Car.ForwardVector; // Quaternion.RotationAxis takes radian angles

                    float dimension = Car.Model.GetDimensions().Y + 1f;

                    if (vControl.Brake != 0f)
                    {
                        Vector3 inputBrake = inputplace + (Car.ForwardVector * -(Dimensions.Y * 0.5f));
                        Color c = Color.Red;
                        int alpha = (int)ARS.map(vControl.Brake, 0, 1f, 0, 255, true);
                        World.DrawMarker(MarkerType.ChevronUpx1, inputBrake, Car.ForwardVector, new Vector3(-90, 0, 0), new Vector3(dimension / 2, dimension / 4, (dimension / 2)), Color.FromArgb(alpha, c), false, false, 0, false, "", "", false);
                    }

                    if (vControl.Throttle != 0f)
                    {
                        Vector3 inputThrottle = inputplace + (Car.ForwardVector * (Dimensions.Y * 0.5f));
                        Color c = Color.GreenYellow;
                        int alpha = (int)ARS.map(vControl.Throttle, 0, 1, 0, 255, true);
                        World.DrawMarker(MarkerType.ChevronUpx1, inputThrottle, -Car.ForwardVector, new Vector3(90, 0, 0), new Vector3(dimension / 2, dimension / 4, -(dimension / 2)), Color.FromArgb(alpha, c), false, false, 0, false, "", "", false);
                    }
                }

                if (ARS.OptionValuesList[Options.ShowTrackAnalysis])
                {
                    Vector3 source = Car.Position + new Vector3(0, 0, 0.5f + (Car.Model.GetDimensions().Z * 0.6f));
                    //ARS.DrawText(source, "~b~" + Math.Round(ARS.MStoMPH(Car.Velocity.Length())).ToString() + "~w~mph~n~~y~" + Math.Round(avgGs.Length() / 9.8f, 2) + "Gs", Color.White, 0.5f);


                    if (KnownCorners.Any() && KnownCorners.Count > 1)
                    {
                        int d = 0;
                        foreach (CornerPoint c in KnownCorners)
                        {
                            if (c.Node % 2 == 1) continue;
                            if (c.Node > KnownCorners.First().Node + KnownCorners.First().AvgScale) break;

                            d++;
                            Vector3 wp = ARS.Path[c.Node];
                            float expectedSpeed = c.Speed;
                            Color gColor = ARS.GetColorFromRedYellowGreenGradient(ARS.map(expectedSpeed - Car.Velocity.Length(), -ARS.AIData.SpeedToInput, ARS.AIData.SpeedToInput, 0, 100, true));

                            if (d == 1)
                            {
                                ARS.DrawLine(source, wp, gColor);
                                ARS.DrawText(wp + new Vector3(0, 0, 1), "~b~" + Math.Round(ARS.MStoMPH(c.Speed)) + "~w~mph~n~~y~" + Math.Round(vehData.WheelsGrip + ARS.GetDownforceGsAtSpeed(this, c.Speed), 2) + "~w~Gs", Color.White, ARS.map(d, 10, 1, 0.15f, 0.5f, true)); //*(c[5]+1f)
                            }
                            World.DrawMarker(MarkerType.ChevronUpx1, wp, ARS.TrackPoints[c.Node].Direction, new Vector3(90, 0, 0), new Vector3(ARS.TrackPoints[c.Node].TrackWide * 2.5f, 5, 5), Color.FromArgb(50, gColor.R, gColor.G, gColor.B));
                        }
                    }

                    int node = trackPoint.Node;
                    int size = (followTrackPoint.Node - trackPoint.Node) * 4;
                    for (int i = node; i < node + size; i++)
                    {
                        if (i % 2 == 1) continue;

                        if (i > 0 && i < ARS.TrackPoints.Count)
                        {
                            TrackPoint cp = ARS.TrackPoints[i];
                            World.DrawMarker(MarkerType.ChevronUpx1, ARS.TrackPoints[cp.Node].Position - new Vector3(0, 0, 0.05f), ARS.TrackPoints[cp.Node].Direction, new Vector3(90, 0, 0), new Vector3(ARS.TrackPoints[cp.Node].TrackWide * 2.5f, 5, 5), Color.FromArgb(50, Color.White));
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

        float OutOfTrackDistance()
        {
            return (Math.Abs(mem.data.DeviationFromCenter) + BoundingBox) - trackPoint.TrackWide;
        }

        #region Avoidance

        float impedimentOnSide = 0; //Directly by your side
        float fImpedimentOnSide = 0; //Ahead, not by yourside
        void VehicleInteractions()
        {
            DebugText.Clear();
            impedimentOnSide = 0.0f;
            fImpedimentOnSide = 0.0f;
            if (Car.Driver.IsPlayer || ARS.SettingsFile.GetValue<bool>("GENERAL_SETTINGS", "Ghosts", false)) return;


            if (NearbyRivals.Any())
            {
                //AI shouldn't be aggresive while crowded
                if (NearbyRivals.Where(r => r.Car.IsInRangeOf(Car.Position, 10)).Count() > 2)
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

            //Interaction with other vehicles            
            foreach (Racer r in NearbyRivals)
            {
                Vehicle refVehicle = r.Car;
                if (!ARS.CanWeUse(refVehicle)) continue;

                if (ARS.CanWeUse(refVehicle))
                {

                    //Position and direction data
                    Vector3 PositionRelativeToUs = ARS.GetOffset(Car, refVehicle);
                    Vector3 PositionRelativeToThem = ARS.GetOffset(refVehicle, Car);

                    float SideRelToTheirDirection = ARS.LeftOrRight(Car.Position, r.Car.Position, r.Car.Velocity.Normalized);
                    float SideRelToOurDirection = ARS.LeftOrRight(r.Car.Position, Car.Position, Car.Velocity.Normalized);//Positive=left of us;

                    float impedimentDist = ARS.Clamp(Math.Abs(SideRelToTheirDirection) - ((BoundingBox + r.BoundingBox) / 2), 0f, 10f) * (SideRelToTheirDirection > 0 ? 1 : -1);
                    float SpeedDiff = (float)Math.Round(Car.Velocity.Length() - refVehicle.Velocity.Length(), 1);// Negative=he is faster. Positive=he is slower;
                    float YBoundingBox = Math.Abs((Car.Model.GetDimensions().Y / 2) + (refVehicle.Model.GetDimensions().Y / 2)) - 0.25f;
                    float XDynamicBoundingBox = 0f;

                    //Their bounding box relative to our direction
                    XDynamicBoundingBox = ARS.map(Vector3.Angle(Car.Velocity.Normalized, refVehicle.ForwardVector), 0f, 90f, refVehicle.Model.GetDimensions().X, (refVehicle.Model.GetDimensions().Y));
                    XDynamicBoundingBox = ARS.Clamp(XDynamicBoundingBox, refVehicle.Model.GetDimensions().X, refVehicle.Model.GetDimensions().Y) / 2;

                    //Our own bounding box                    
                    XDynamicBoundingBox += BoundingBox / 2;

                    //How much the distance intersects with the sum of both BBs
                    float sideToSideDist = Math.Abs(SideRelToTheirDirection) - ((XDynamicBoundingBox));
                    float targetInsideDist = Math.Abs(SideRelToOurDirection) - ((XDynamicBoundingBox));

                    float DirDifference = Vector3.SignedAngle(Car.Velocity.Normalized, refVehicle.Velocity.Normalized, Car.UpVector);

                    //Adjust the speed relative to their direction relative to us
                    float targetRelativeSpeed = ARS.map(Math.Abs(DirDifference), 180f, 0f, -refVehicle.Velocity.Length(), refVehicle.Velocity.Length(), true);
                    SpeedDiff = (float)Math.Round(Car.Velocity.Length() - targetRelativeSpeed, 2);

                    //Distance and seconds to reach them
                    float sToHit = 10f;
                    float distance = (Car.Position.DistanceTo2D(refVehicle.Position) - YBoundingBox);
                    if (SpeedDiff > 0f) sToHit = (float)Math.Round(distance / SpeedDiff, 2);
                    if (float.IsNaN(sToHit) || float.IsInfinity(sToHit)) sToHit = 10f;
                    sToHit = ARS.Clamp(sToHit, 0f, 10f);

                    float safeSideDist = XDynamicBoundingBox + mem.personality.Rivals.SideToSideMinDist;

                    Vector3 offsetRight = r.Car.Position + (Vector3.Cross(r.Car.Velocity.Normalized, Vector3.WorldUp) * ((XDynamicBoundingBox / 2) + mem.personality.Rivals.SideToSideMinDist));
                    offsetRight = Vector3.Lerp(Car.Position, offsetRight, 2);

                    Vector3 offsetLeft = r.Car.Position - (Vector3.Cross(r.Car.Velocity.Normalized, Vector3.WorldUp) * ((XDynamicBoundingBox / 2) + mem.personality.Rivals.SideToSideMinDist));
                    offsetLeft = Vector3.Lerp(Car.Position, offsetLeft, 2);

                    Vector3 directionToCar = Car.Velocity.Normalized;
                    Vector3 offsetSelected = Vector3.Zero;

                    float SteerToIt = 0f;

                    //Closer to the right of the target
                    if (offsetRight.DistanceTo(Car.Position) < offsetLeft.DistanceTo(Car.Position))
                    {
                        if (ARS.GetOffset(Car, offsetRight).Y > 0f)
                        {
                            SteerToIt = Vector3.SignedAngle(directionToCar, (offsetRight - Car.Position).Normalized, Vector3.WorldUp);
                            offsetSelected = offsetRight;
                        }
                    }
                    else //Closer to the left of the target
                    {
                        if (ARS.GetOffset(Car, offsetLeft).Y > 0f)
                        {
                            SteerToIt = Vector3.SignedAngle(directionToCar, (offsetLeft - Car.Position).Normalized, Vector3.WorldUp);
                            offsetSelected = offsetLeft;
                        }
                    }

                    //Avoid steering to the contrary side
                    if (SteerToIt > 0 != PositionRelativeToUs.X > 0) SteerToIt = 0f;
                    else
                    {
                        //Don't steer out of the track - steer back the other side
                        if (mem.data.DeviationFromCenter > 0 != SteerToIt > 0 && r.mem.data.DeviationFromCenter > 0 != SteerToIt > 0)
                        {
                            if (Math.Abs(r.mem.data.DeviationFromCenter) + safeSideDist + 2 > trackPoint.TrackWide)
                            {
                                if (offsetSelected == offsetLeft) SteerToIt = Vector3.SignedAngle(directionToCar, (offsetRight - Car.Position).Normalized, Vector3.WorldUp);
                                else SteerToIt = Vector3.SignedAngle(directionToCar, (offsetLeft - Car.Position).Normalized, Vector3.WorldUp);
                            }
                        }
                    }

                    //Update Impediment side to side.
                    if (Math.Abs(PositionRelativeToUs.Y) < YBoundingBox)
                    {
                        if (impedimentOnSide == 0.0f || Math.Abs(impedimentDist) < Math.Abs(impedimentOnSide))
                        {
                            impedimentOnSide = impedimentDist;
                        }
                    }
                    else
                    {
                        //Update ImpedimentAhead. Serves us to know if we should switch lanes down the logic pipeline
                        if (sToHit < 3f)
                        {
                            if (fImpedimentOnSide == 0.0f || Math.Abs(impedimentDist) < Math.Abs(fImpedimentOnSide)) fImpedimentOnSide = impedimentDist;
                        }
                    }


                    //Side to side
                    //Ahead but within limits, or behind but within limits
                    if (mem.personality.Rivals.SideToSideMinDist != 0f && (PositionRelativeToUs.Y > 0 && PositionRelativeToUs.Y < YBoundingBox) || (PositionRelativeToUs.Y < 0 && PositionRelativeToUs.Y > -YBoundingBox))
                    {
                        if (sideToSideDist < 10)
                        {
                            List<Vector2> maneuvers = new List<Vector2>();
                            float maxSteer = 40f;

                            if (Car.IsTouching(refVehicle) && PositionRelativeToUs.Y > 0f)
                            {
                                maneuvers.Add(new Vector2((PositionRelativeToUs.X > 0 ? 3 : -3), ARS.AIData.SpeedToInput));
                            }

                            float str = 0f;
                            if (DirDifference > 0 == PositionRelativeToUs.X > 0f)
                            {
                                str = Vector3.Angle(Car.Velocity.Normalized, r.Car.Velocity.Normalized) * (DirDifference > 0 ? 1 : -1);                                
                                ARS.Clamp(str, -10, 10);
                                
                                //The closer we are, the higher the steer
                                str *= ARS.map(Math.Abs(PositionRelativeToUs.X), safeSideDist + 5f, safeSideDist, 0f, 1f, true);
                            }
                            else if (sideToSideDist < mem.personality.Rivals.SideToSideMinDist) maneuvers.Add(new Vector2((PositionRelativeToUs.X > 0 ? 1 : -1), ARS.AIData.SpeedToInput));



                            //If we would have to steer against another car on our side, reduce our steering gradually
                            if (impedimentOnSide != 0f)
                            {
                                if (impedimentOnSide > 0f == str > 0f)
                                {
                                    maxSteer = 10;
                                    maxSteer = ARS.map(Math.Abs(impedimentOnSide), safeSideDist, safeSideDist + 5f, 0f, maxSteer, true);
                                }
                            }

                            str = ARS.Clamp(str, -maxSteer, maxSteer);
                            if (str > 0 == PositionRelativeToUs.X > 0f && !float.IsNaN(str)) maneuvers.Add(new Vector2(str, ARS.AIData.SpeedToInput));

                            if (maneuvers.Any())
                            {
                                Vector2 m = maneuvers.OrderByDescending(x => Math.Abs(x.X)).First();

                                //If we are steering towards the outside of the track, limit our steering to avoid leaving
                                if (m.X > 0f == mem.data.DeviationFromCenter < 0f)
                                {
                                    float outMult = maxSteer;// ARS.map(OutOfTrackDistance(), 5f, 0f, 0, maxSteer, true);
                                    m.X = ARS.Clamp(m.X, -outMult, outMult);
                                }
                                mem.intention.Maneuvers.Add(m);
                            }
                        }
                    }
                    else if (mem.personality.Rivals.BehindRivalMinDistance != 0f)//Not side to side
                    {
                        //Ahead us
                        if (PositionRelativeToUs.Y > -YBoundingBox)
                        {
                            SteerToIt = ARS.Clamp(SteerToIt, -4, 4);

                            //Avoid rearends via brake
                            if (targetInsideDist < 0f)
                            {
                                if (Car.IsTouching(r.Car)) mem.intention.Maneuvers.Add(new Vector2(0, -ARS.MPHtoMS(5)));

                                if (SpeedDiff > -10 && mem.personality.Rivals.BehindRivalMinDistance > 0f)
                                {
                                    float d = PositionRelativeToUs.Y - mem.personality.Rivals.BehindRivalMinDistance;
                                    float maneuver = ARS.MapIdealSpeedForDistanceG(this, d, refVehicle.Velocity.Length()) - Car.Velocity.Length();
                                    maneuver -= ARS.map(d, -mem.personality.Rivals.BehindRivalMinDistance, 0, -ARS.MPHtoMS(1f), 0, true);

                                    if (!float.IsNaN(maneuver) && !float.IsInfinity(maneuver) && maneuver < ARS.AIData.SpeedToInput)
                                    {
                                        float[] minInput = {
                                            ARS.map(sToHit, 1f, 2f, -ARS.AIData.SpeedToInput, ARS.AIData.SpeedToInput, true),
                                            ARS.map(distance, mem.personality.Rivals.BehindRivalMinDistance, mem.personality.Rivals.BehindRivalMinDistance+5f, 0f, ARS.AIData.SpeedToInput, true),
                                            ARS.map(distance, 0f, mem.personality.Rivals.BehindRivalMinDistance, -ARS.AIData.SpeedToInput,0f, true)
                                        };
                                        maneuver = ARS.Clamp(maneuver, minInput.Min(), ARS.AIData.SpeedToInput);
                                        mem.intention.Maneuvers.Add(new Vector2(0, maneuver));
                                    }
                                }
                            }

                            //Avoid rearends via steer
                            if (!Decisions.ContainsKey(Decision.NoOvertake))
                            {
                                //Sanity steer limit
                                float maxSteer = 30f;
                                float tSteering = SteerToIt;

                                //Approaching them
                                if (SpeedDiff > -10)
                                {
                                    if (Math.Abs(DirDifference) < 30f)
                                    {
                                        float distAhead = PositionRelativeToUs.Y - mem.personality.Rivals.BehindRivalMinDistance;
                                        float[] mults = { 0, 0 };
                                        if (KnownCorners.Any())
                                        {
                                            mults[0] = ARS.map(sToHit, 3f, 1f, 0f, 1f, true);
                                            mults[1] = ARS.map(distAhead, 5f, 3f, 0f, 1f, true);
                                        }
                                        else
                                        {
                                            mults[0] = ARS.map(sToHit, 6f, 3f, 0f, 1f, true);
                                            mults[1] = ARS.map(distAhead, 20f, 5f, 0f, 1f, true);
                                        }
                                        tSteering = SteerToIt * mults.Max();
                                        tSteering *= ARS.map(ARS.MStoMPH(SpeedDiff), -10, 10, 0, 1, true);
                                    }
                                }

                                /*
                                else  //Not approaching, we are slower
                                {
                                    tSteering = SteerToIt * ARS.map(distance, mem.personality.Rivals.BehindRivalMinDistance * 2f, mem.personality.Rivals.BehindRivalMinDistance, 0f, 1f, true);
                                }
                                */
                                

                                //After making a steering decision, adjust for our context in the track
                                if (Math.Abs(tSteering) > 0.0f)
                                {
                                    //If we plan to overtake by one side, but there's a car ahead on that side already, reduce our steering gradually
                                    if (fImpedimentOnSide != 0)
                                    {
                                        if (fImpedimentOnSide > 0f == tSteering > 0f)
                                        {
                                            maxSteer = ARS.map(Math.Abs(fImpedimentOnSide), safeSideDist, safeSideDist + 1f, 0f, 2f, true);
                                        }
                                    }

                                    //If we would have to steer against another car on our side, reduce our steering gradually
                                    if (impedimentOnSide != 0f)
                                    {
                                        if (impedimentOnSide > 0f == tSteering > 0f)
                                        {
                                            maxSteer = ARS.map(Math.Abs(impedimentOnSide), safeSideDist, safeSideDist + 1f, 0f, 2f, true);
                                        }
                                    }
                                    tSteering = ARS.Clamp(tSteering, -maxSteer, maxSteer);
                                    if (tSteering != 0f && !float.IsNaN(tSteering) && !float.IsInfinity(tSteering)) mem.intention.Maneuvers.Add(new Vector2(tSteering, ARS.AIData.SpeedToInput));
                                }
                            }

                        }
                    }
                }
            }
        }
        #endregion

        bool WouldMoveOutOfTrack(float maneuver)
        {
            //if we're already close to the outside and the maneuver turns us further there
            if (Math.Abs(mem.data.DeviationFromCenter) + Car.Model.GetDimensions().X > trackPoint.TrackWide && maneuver > 0 == mem.data.DeviationFromCenter > 0) return true;

            return false;
        }
        float FollowTrackCornerAngle = 0f;
        public bool FinishedPointToPoint = false;
        public int localSPDLimiter = 0;
        public float lookAhead = 0;



        public TrackPoint trackPoint = new TrackPoint();
        public TrackPoint followTrackPoint = new TrackPoint();

        //temp
        float atoLookahead = 0f;

        public void GetTrackInfo()
        {

            //Get our current track point data
            int cNode = trackPoint.Node;
            int mNode = trackPoint.Node - 10;
            if (mNode < 0) mNode = 0;
            int nNode = mNode + 20;
            if (nNode > ARS.TrackPoints.Count) nNode = ARS.TrackPoints.Count;

            if (cNode > ARS.TrackPoints.Count - 5) trackPoint = ARS.TrackPoints.First();
            else trackPoint = ARS.TrackPoints.Where(p => p.Node >= cNode - 10 && p.Node <= cNode + 10).OrderBy(t => t.Position.DistanceTo(Car.Position)).First();
            mem.data.DeviationFromCenter = ARS.LeftOrRight(Car.Position, trackPoint.Position, trackPoint.Direction);

            //Look ahead
            mem.intention.LookaheadDeviationFromCenter = 0f;
            lookAhead = (int)((Car.Velocity.Length() / ARS.Clamp(vehData.WheelsGrip + vehData.DownforceGrip, 1, 3)));
            if (lookAhead < 5) lookAhead = 5;
            if (lookAhead > 150) lookAhead = 150;

            int fNode = trackPoint.Node + (int)(lookAhead * 1.5f);//*2is too much, even though it would make sense
            if (Car.Model.IsBike) fNode = trackPoint.Node + (int)(lookAhead);
            if (fNode < ARS.TrackPoints.Count())
            {
                //No lane changes planned
                if (1 == 1)
                {
                    float a = (float)Math.Round(Vector3.SignedAngle(trackPoint.Direction, ARS.TrackPoints[fNode].Direction, Vector3.WorldUp), 1);
                    atoLookahead = a;
                    if (Math.Abs(a) > 0.25f)
                    {

                        float laneOffset = (float)Math.Round(ARS.map(a, 10, -10, -followTrackPoint.TrackWide, followTrackPoint.TrackWide, true), 2);
                        float max = ARS.map(Math.Abs(a), 0.25f, 20, 0, 4, true);

                        //Actually the inside 
                        if (max > 0f && a > 0 == mem.data.DeviationFromCenter > laneOffset)
                        {
                            //Has nearby obstacles
                            if (impedimentOnSide != 0f && impedimentOnSide > 0 == a > 0f)
                            {
                                max = ARS.map(Math.Abs(impedimentOnSide), 0, mem.personality.Rivals.SideToSideMinDist + 2, 0f, 3f, true);
                            }
                            else if (fImpedimentOnSide != 0f && fImpedimentOnSide > 0 == a > 0f)
                            {
                                max = ARS.map(Math.Abs(impedimentOnSide), 0, mem.personality.Rivals.SideToSideMinDist + 2, 0f, 3f, true);
                            }

                            if (max > 0.0f)
                            {
                                float point = (laneOffset - mem.data.DeviationFromCenter);
                                float limit = ARS.map(Math.Abs(a), 45, 10, 0, BoundingBox, true);
                                mem.intention.LookaheadDeviationFromCenter = ARS.Clamp(mem.data.DeviationFromCenter + ARS.Clamp(point, -max, max), -followTrackPoint.TrackWide + limit, followTrackPoint.TrackWide - limit);
                            }
                        }
                    }
                }
            }

            //if (Car.Model.IsBicycle || Car.Model.IsBike) lookAhead *= 0.5f;

            if (lookAhead < 10) lookAhead = 10;



            if (trackPoint.Node + lookAhead >= ARS.TrackPoints.Count) followTrackPoint = ARS.TrackPoints[(int)lookAhead];
            else followTrackPoint = ARS.TrackPoints[trackPoint.Node + (int)lookAhead];

        }
        public void AddDebugText(string s)
        {
            s = "~w~" + s + "~w~";
            if (!DebugText.Contains(s)) DebugText.Add(s);
        }

        public void ProcessAI() //100ms
        {
            //Ghosting
            if (Car.Alpha != 255) Car.ResetAlpha();
            if (ARS.SettingsFile.GetValue<bool>("GENERAL_SETTINGS", "Ghosts", false) && NearbyRivals.Any())
            {
                foreach (Racer r in NearbyRivals)
                {
                    if (Function.Call<bool>(Hash.IS_ENTITY_AT_ENTITY, Car, r.Car, (r.Car.Model.GetDimensions().X * 1.5f), (r.Car.Model.GetDimensions().Y * 1.5f), (r.Car.Model.GetDimensions().Z * 1.5f), true, true, true))
                    {
                        Function.Call(Hash.SET_ENTITY_NO_COLLISION_ENTITY, Car, r.Car, false);
                        Car.Alpha = 150;
                    }
                }
            }

            //Checks for Z movement to judge wether the car is stable or not. Usually implies the vehicle is mid-air
            if (Math.Abs(vehData.SpeedVectorLocal.Normalized.Z) > 0.025f || TimeOutOfTrack != 0)
            {
                if (vehData.AvgGroundStability >= 0.75f) vehData.AvgGroundStability -= 0.05f;
            }
            else if (vehData.AvgGroundStability < 1f) vehData.AvgGroundStability += 0.05f;
            if (vehData.AvgGroundStability > 1f) vehData.AvgGroundStability = 1f;

            //Data dathering
            BoundingBox = ARS.GetDirectionalBoundingBox(Car);
            mem.intention.Aggression = ARS.Clamp(mem.intention.Aggression, 0, 1f);
            vehData.SlideAngle = (float)Math.Round(Vector3.SignedAngle(Car.Velocity.Normalized, Car.ForwardVector, Car.UpVector), 1);

            if (BaseBehavior == RacerBaseBehavior.GridWait && vControl.HandBrakeTime < Game.GameTime) vControl.HandBrakeTime = Game.GameTime + (100 * ARS.GetRandomInt(2, 6));

            if (HalfSecondTick < Game.GameTime)
            {
                HalfSecondTick = Game.GameTime + 500 + (Pos * 10);

                //Remove any corners too fast for us to account for
                if (KnownCorners.Any()) KnownCorners.RemoveAll(v => v.Node < KnownCorners.First().Node || v.Speed > Car.Velocity.Length() + ARS.MPHtoMS(30));

                UpdatePercievedGrip();
                if (BaseBehavior == RacerBaseBehavior.Race && ARS.Racers.Count >= 1) GetReferenceVehicle();

                //Decision cleaning
                if (Decisions.Any(de => de.Value < Game.GameTime)) Decisions.Remove(Decisions.First(de => de.Value < Game.GameTime).Key);
                if (BannedDecisions.Any(de => de.Value < Game.GameTime)) BannedDecisions.Remove(BannedDecisions.First(de => de.Value < Game.GameTime).Key);

                //Mistake cleaning
                if (Mistakes.Any(de => de.Value < Game.GameTime)) Mistakes.Remove(Mistakes.First(de => de.Value < Game.GameTime).Key);
                if (BannedMistakes.Any(de => de.Value < Game.GameTime)) BannedMistakes.Remove(BannedMistakes.First(de => de.Value < Game.GameTime).Key);

                if (ARS.DevSettingsFile.GetValue<bool>("RACERS", "AllowManeuvers", false) && !BannedDecisions.ContainsKey(Decision.NoOvertake) && KnownCorners.Any() && NearbyRivals.Where(r => r.Car.IsInRangeOf(Car.Position, BoundingBox * 2)).Count() >= 3)
                {
                    MakeDecision(Decision.NoOvertake, 100 - (int)(mem.intention.Aggression * 100), 2500, 2500, 2500);
                }

                //Reset into track logic
                if (Math.Abs(mem.data.DeviationFromCenter) > trackPoint.TrackWide && !ControlledByPlayer && BaseBehavior == RacerBaseBehavior.Race)
                {
                    if (TimeOutOfTrack == 0) TimeOutOfTrack = Game.GameTime;
                    else if (Game.GameTime - TimeOutOfTrack > 8000 && Car.Velocity.Length() < 5f)
                    {
                        TimeOutOfTrack = 0;
                        ResetIntoTrack();
                    }
                }
                else
                {
                    if (TimeOutOfTrack != 0) TimeOutOfTrack = 0;
                }

                if (!Driver.IsPlayer) if (NearbyRivals.Count > 0) Driver.Task.LookAt(NearbyRivals[0].Driver, 2000); else if (Car.Velocity.Length() > 5f) Driver.Task.LookAt(Car.Position + Car.Velocity, 2000);


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
            }

            //Slow checks
            if (OneSecondTick < Game.GameTime)
            {
                OneSecondTick = Game.GameTime + (1000 + (Pos * 20));

                if (!ControlledByPlayer)
                {


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
                    ARS.WorstCornerAhead(this);
                }

                //Rocket boost
                if (!KnownCorners.Any())
                {
                    if (BaseBehavior == RacerBaseBehavior.Race && Vector3.Angle(Car.ForwardVector, trackPoint.Direction) < 5f && Math.Abs(FollowTrackCornerAngle) < 0.5f && Math.Abs(mem.data.SpeedVector.X) < 0.1f && Math.Abs(vControl.SteerInput) < 0.2f) Function.Call((Hash)0x81E1552E35DC3839, Car, true);
                }
                if (Function.Call<bool>((Hash)0x3D34E80EED4AE3BE, Car) && vControl.Brake > 0.1f) Function.Call((Hash)0x81E1552E35DC3839, Car, false);


                if (ARS.DevSettingsFile.GetValue<int>("RACERS", "AIRacerAutofix", 1) == 2 && Function.Call<bool>(Hash._IS_VEHICLE_DAMAGED, Car))
                {
                    // UI.Notify("~b~" + Name + " auto repaired");
                    Car.Repair();
                }

                //Stuck behavior

                if (vehData.GsAcceleration < 0.1f && Car.Velocity.Length() < 1f)
                {
                    if (!Driver.IsPlayer && BaseBehavior == RacerBaseBehavior.Race && Driver.IsSittingInVehicle(Car) && vControl.HandBrakeTime < Game.GameTime)
                    {
                        StuckScore++;
                        if (!Car.IsOnAllWheels) StuckScore++;


                        if (StuckScore >= 2)
                        {
                            if (StuckScore >= 4)
                            {
                                if (Driver.IsSittingInVehicle(Car) && !Car.IsInWater && Car.EngineHealth > 0)
                                {
                                    StuckScore = 0;
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
                else if (StuckScore > 0) StuckScore--;
                StuckScore = (int)ARS.Clamp(StuckScore, 0, 10);

                if (StuckRecover && (!Car.IsInRangeOf(LastStuckPlace, 5f) || mem.data.SpeedVector.Y > 3f))
                {
                    StuckRecover = false;
                    StuckScore = 0;
                }
            }


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

            if (!ControlledByPlayer)
            {
                mem.intention.Corrections.Clear();
                mem.intention.Maneuvers.Clear();
                VehicleInteractions();
                ManageOvershoot();

                /*
                //Understeer
                if (!float.IsInfinity(TRCurveAngle) && vControl.SteerAngle >= 0 != TRCurveAngle > 0)
                {
                    float tr = (float)Math.Round(TRCurveAngle * 10, 1);
                    float str = (float)Math.Round(-vControl.SteerAngle, 1);
                    float UndersteerPercent = (float)Math.Round(ARS.GetPercent(str, tr));

                    float maxAcc = ARS.map(Math.Abs(vControl.SteerAngle), 5, 0, ARS.AIData.SpeedToInput * 0.5f, ARS.AIData.SpeedToInput, true);

                    if (maxAcc < ARS.AIData.SpeedToInput)
                    {
                        float maneuver = (ARS.map(UndersteerPercent, 500, 200, maxAcc, ARS.AIData.SpeedToInput, true));
                        mem.intention.Corrections.Add(new Vector2(0, maneuver));
                    }
                }
                */

                //Base Steer
                SteerTrack();

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
                        vControl.SteerAngle += (ma * ARS.map(Math.Abs(ma), 0, SteeringLock * 0.25f, 0f, 1f, true));
                    }

                    //Steering limiter

                    Vector3 currentTrackDir = trackPoint.Direction;
                    currentTrackDir.Z = 0f;
                    Vector3 followTrackDir = followTrackPoint.Direction;
                    followTrackDir.Z = 0f;

                    Vector3 carDir = Car.Velocity.Normalized;
                    carDir.Z = 0f;

                    float angle = Vector3.SignedAngle(currentTrackDir, followTrackDir, Vector3.WorldUp);
                    CarToCornerAngle = Vector3.SignedAngle(carDir, currentTrackDir, Vector3.WorldUp);
                    float CornerAngle = ARS.CornerPoints[trackPoint.Node].Angle;

                    float cSpd = ARS.MStoMPH(Car.Velocity.Length());
                    float middlePint =ARS.map(vControl.Throttle, 1f, 0f, 0.4f, 0.6f, true);

                    float maxExtra = 40f;

                    if (!Mistakes.ContainsKey(Mistake.ForgetSteeringLimiter))
                    {
                        if (cSpd < 25f) maxExtra = ARS.map(cSpd, 25f, 10f, handlingData.TRlateral * middlePint, SteeringLock, true);
                        else maxExtra = ARS.map(cSpd, 150f, 100f, 3f, handlingData.TRlateral * middlePint, true);

                        if (Math.Abs(CarToCornerAngle) > 30 && Math.Abs(CornerAngle) > 0f && CarToCornerAngle > 0f == CornerAngle > 0f)
                        {
                            if (ARS.MStoMPH(vehData.SpeedVectorLocal.Y) > 10) mem.intention.Maneuvers.Add(new Vector2(0, ARS.AIData.SpeedToInput * 0.2f));
                            maxExtra = ARS.map(cSpd, 25f, 5f, handlingData.TRlateral * 0.75f, SteeringLock, true);
                        }
                    }

                    if (mem.data.SpeedVector.X > 0 == vControl.SteerAngle > 0 && Math.Abs(vControl.SteerAngle) > maxExtra) MakeMistake(Mistake.ForgetSteeringLimiter, 100 - mem.personality.Stability.Skill, 1000, 2000, 4000);

                    vControl.SteerAngle = ARS.Clamp(vControl.SteerAngle, -maxExtra, maxExtra);
                    SteerCorrections();
                }

                //Base speed
                SpeedLogic();

                //Lift if its outside, going outside
                if (TimeOutOfTrack != 0 && CarToCornerAngle > 0 == mem.data.DeviationFromCenter > 0 && ARS.MStoMPH(vehData.SpeedVectorLocal.Y) > 25) mem.intention.Maneuvers.Add(new Vector2(0, ARS.AIData.SpeedToInput * 0.05f));


                //Speed Maneuvers
                if (mem.intention.Maneuvers.Any() || mem.intention.Corrections.Any())
                {
                    float speedMod = mem.intention.Maneuvers.Concat(mem.intention.Corrections).OrderBy(v => v.Y).First().Y;

                    //If its less than full throttle
                    if (speedMod <= ARS.AIData.SpeedToInput)
                    {
                        //If it results in actual reduction of speed
                        if (Car.Velocity.Length() + speedMod < mem.intention.Speed)
                        {
                            mem.intention.Speed = Car.Velocity.Length() + speedMod;
                        }
                    }
                    else
                    {
                        mem.intention.Speed += speedMod;
                    }
                }

                TranslateThrottleBrake();
                TranslateSteer();
                TractionControl();
            }
        }


        void ResetIntoTrack()
        {
            vControl.SteerAngle = 0f;
            mem.data.SteerAngle = 0f;

            Car.Position = ARS.Path[trackPoint.Node];

            Car.Heading = trackPoint.Direction.ToHeading();

            StuckRecover = false;
            LastStuckPlace = Vector3.Zero;
            Car.Speed = 20f;
        }

        void UpdatePercievedGrip()
        {
            vehData.DownforceGrip = ARS.GetDownforceGsAtSpeed(this, Car.Velocity.Length());

            float HandlingGrip = Function.Call<float>((Hash)0xA132FB5370554DB0, Car);//Car grip
            HandlingGrip = ARS.Clamp(HandlingGrip, 0.4f, 3f);
            float avg = ARS.GetWheelsGrip(Car).Average();
            float wetavg = ARS.GetWheelsWetgrip(Car).Average();

            SurfaceGrip = (float)Math.Round(avg, 1);
            if (HandlingGrip < 0.1f) HandlingGrip = 0.1f;
            HandlingGrip *= (SurfaceGrip * wetavg);
            vehData.WheelsGrip = HandlingGrip * vehData.AvgGroundStability;

            //If you're in first, map terrain multipliers for the rest of the racers
            if (Pos <= 2 && !ARS.MultiplierInTerrain.ContainsKey(trackPoint.Node))
            {
                ARS.MultiplierInTerrain.Add(trackPoint.Node, SurfaceGrip);
            }

        }


        float maxCarToTrackpointAngle = 0f;
        float CarToCornerAngle = 0f;

        /// <summary>
        /// Judges wether the racer is overshooting the corner and adds speed Maneuvers to correct it.
        /// </summary>
        void ManageOvershoot()
        {


            //Projected path method, tries to guess where the racer will be a second later
            int refPointNode = (int)(trackPoint.Node + Car.Velocity.Length());

            if (refPointNode < ARS.TrackPoints.Count() )
            {
                TrackPoint refPoint = ARS.TrackPoints[refPointNode];

                float man = ARS.AIData.SpeedToInput;

                //Never brake below 10ms less than the original corner speed            +(Car.Velocity.Length()/2)
                float minSpd = ARS.GetSpeedForCorner(ARS.CornerPoints[(int)(trackPoint.Node)], this) * 0.85f;
                float noLift = refPoint.TrackWide;
                float maxLift = refPoint.TrackWide+5;

                //If we're going to the outside of this corner 
                if (mem.data.SpeedVector.X > 0 == ARS.CornerPoints[followTrackPoint.Node].Angle > 0)
                {
                    Vector3 projected = Car.Position + (Car.Velocity + ((vehData.AccelerationVector.Aggregate(new Vector3(0, 0, 0), (s, v) => s + v) / (float)vehData.AccelerationVector.Count)) / 2);
                    float LoR = ARS.LeftOrRight(projected, refPoint.Position, refPoint.Direction);
                    float LoRCar = ARS.LeftOrRight(projected, Car.Position, Car.Velocity.Normalized);

                    //If our projection expect to be further outside than us right now
                    if ( Math.Abs(LoR) > Math.Abs(mem.data.DeviationFromCenter)) //LoRCar > 0 == TRCurveAngle > 0 &&
                    {
                        if (ARS.CornerPoints[trackPoint.Node].Angle > 0)
                        {
                            man = ARS.map(LoR, maxLift, noLift, -ARS.AIData.SpeedToInput, ARS.AIData.SpeedToInput, true);
                        }
                        else
                        {
                            man = ARS.map(LoR, -maxLift, -noLift, -ARS.AIData.SpeedToInput, ARS.AIData.SpeedToInput, true);
                        }
                    }



                    //Makes sure the maneuver does not try and stop the car below certain speed.
                    //Helps smoothing the behavior, and provides some common sense.
                    //(if overshooting you wouldn't try and brake to a stop, instead slowing down to something below the intended speed, but not THAT slow)
                    if (man != ARS.AIData.SpeedToInput)
                    {
                        if (Car.Velocity.Length() + man < minSpd)
                        {
                            //   UI.ShowSubtitle("~o~Tryna go below min speed", 200);
                            float tSpd = (Car.Velocity.Length() + man) - minSpd;
                            man = ARS.map(tSpd, ARS.AIData.SpeedToInput, -ARS.AIData.SpeedToInput, -ARS.AIData.SpeedToInput, ARS.AIData.SpeedToInput, true);
                        }
                    }

                    //If we're on the outside and going further outside
                    if (Math.Abs(mem.data.DeviationFromCenter) > refPoint.TrackWide && Math.Abs(LoR) > Math.Abs(mem.data.DeviationFromCenter))
                    {
                        float safeTRLat = 1f;
                        if (Math.Abs(TRCurveAngle) > safeTRLat)
                        {
                            man = ARS.AIData.SpeedToInput;
                            //   UI.ShowSubtitle("~g~Gs " + Math.Round(Math.Abs(TRCurveAngle), 1), 200);
                        }
                        else
                        {
                            //  UI.ShowSubtitle("~r~ Gs " + Math.Abs(TRCurveAngle), 200);
                            man = ARS.map(safeTRLat - Math.Abs(TRCurveAngle), 1f, 0, 0f, ARS.AIData.SpeedToInput, true);
                        }
                    }
                    if (man != ARS.AIData.SpeedToInput)
                    {
                        //UI.ShowSubtitle("~r~" + Math.Round(man, 1), 200);
                        mem.intention.Corrections.Add(new Vector2(0, man));
                    }
                }
            }
        }


        public List<Vehicle> Traffic = new List<Vehicle>();
        public void GetReferenceVehicle()
        {

            NearbyRivals.Clear();
            List<Racer> Candidates = new List<Racer>();
            foreach (Racer r in ARS.Racers as IEnumerable<Racer>)
            {

                if (r.Car.Handle != Car.Handle && r.Car.Position.DistanceTo(Car.Position) < 100f)
                {
                    Candidates.Add(r);
                }
            }

            Candidates = Candidates.OrderBy(v => Vector3.Distance(v.Car.Position, Car.Position)).ToList();
            if (Candidates.Count() > 3) Candidates = Candidates.Take(3).ToList();
            NearbyRivals = Candidates;

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
