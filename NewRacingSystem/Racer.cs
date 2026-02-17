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
    public enum RandomVariance
    {
        BrakeDistance, SteeringStrictness, SpeedAggroVariance, SpeedBaseVariance,
    }
    public class Racer
    {

        public Team team = Team.None;
        public VehicleControl vControl = new VehicleControl();
        public Memory mem = new Memory();
        public bool ControlledByPlayer = false;

        //Decision-cooldown pairs
        public Dictionary<Decision, int> Decisions = new Dictionary<Decision, int>();
        public Dictionary<Decision, int> BannedDecisions = new Dictionary<Decision, int>();
        public Dictionary<Mistake, int> Mistakes = new Dictionary<Mistake, int>();
        public Dictionary<Mistake, int> BannedMistakes = new Dictionary<Mistake, int>();

        public Dictionary<RandomVariance, float> BehaviorVariance = new Dictionary<RandomVariance, float>();


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
        public int RacePosition = 0;
        public bool CanRegisterNewLap = true;

        //Ticks
        int HalfSecondTick = 0; //500ms
        int OneSecondTick = 0; //1000ms

        PID SteerPID = new PID(6, 12f, 0);
        PID LanePID = new PID(0.2f, 0.5f, 0);

        //Stuck stuff
        int StuckGameTimeRef = 0;
        public bool StuckRecover = false;
        int GameTimeOutOfTrack = 0;

        //Handling stuff
        public float GroundGripMultiplier = 1f;

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
                Driver.SetIntoVehicle(Car, VehicleSeat.Driver);
                ARS.SetSteerAngle(Car, 0.5f);
                ARS.SetThrottle(Car, 0f);
                ARS.SetBrakes(Car, 0f);

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

            Function.Call(GTA.Native.Hash._0x0DC7CABAB1E9B67E, Car, true, 1); //load collision
            Function.Call(GTA.Native.Hash._0x0DC7CABAB1E9B67E, Driver, true, 1);
            Function.Call(GTA.Native.Hash.SET_ENTITY_PROOFS, Driver, true, true, true, false, true, true, 1, true);

            Driver.MaxHealth = 1000;
            Driver.Health = 1000;
            Driver.CanSufferCriticalHits = false;

            if (Car.ClassType == VehicleClass.Emergency) team = Team.Cop;


            Random _rand = new Random();
            foreach (RandomVariance name in Enum.GetValues(typeof(RandomVariance)).Cast<RandomVariance>())
            {
                float r = _rand.Next(80, 120);
                r *= 0.01f;
                BehaviorVariance.Add(name, r);
            }
        }
        Random Random = new Random();
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


            vData.SteeringLock = ARS.rad2deg(ARS.GetSteerLock(Car));
            if (vData.SteeringLock < 1 || vData.SteeringLock > 100) vData.SteeringLock = 40;
            ARS.Log(ARS.LogImportance.Info, "Steerlock for " + Car.DisplayName + ":" + vData.SteeringLock + "º");
            vControl.SteerTrackDegrees = 0f;
            CurrentTrackPoint = ARS.TrackPoints.Last();
            vControl.Brake = 0f;
            vControl.Throttle = 0f;

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

            vData.PerformanceIndex = (int)((Handling.TopSpeed * 5) + (Handling.Grip * 100) + (Handling.Power * 500));
            vData.TextPerformanceIndex = ((int)(Handling.TopSpeed / 1.2) + " | " + (int)(Handling.Power * 200) + " | " + (int)(Handling.Grip * 20));

            Car.Repair();
        }

        float AngleToTrackDir(TrackPoint first, TrackPoint second)
        {
            float a = Vector3.SignedAngle(first.Direction, second.Direction, Vector3.WorldUp);
            if (float.IsNaN(a)) a = 0;
            return a;
        }

        float TargetLane = 0;
        public float MaxLeftLane = 0;
        public float MaxRightLane = 0;

        /// <summary>
        /// Neccesary calculations for the car to follow the track. The whole function results in vControl.SteerTrack.
        /// </summary>
        public void SteerTrack()
        {
            if (LookAheads[eLookAheads.OneSec] == null || BaseBehavior == RacerBaseBehavior.GridWait ||
                BaseBehavior == RacerBaseBehavior.FinishedStandStill || CurrentTrackPoint.Node < 3)
            {
                vControl.SteerTrackDegrees = 0f;
                return;
            }


            float roadWide = LookAheads[eLookAheads.HalfSec].TrackWide;
            float Outside = 0;
            float Inside = 0;
            float AngleOneSec = AngleToTrackDir(CurrentTrackPoint, LookAheads[eLookAheads.OneSec]);


            MaxLeftLane = -roadWide + vData.BoundingBox;
            MaxRightLane = roadWide - vData.BoundingBox;

            //If the corner is too sharp, steer inside
            float KeepInside = AngleToTrackDir(CurrentTrackPoint, LookAheads[eLookAheads.OneHalfSec]);
            TargetLane = mem.data.DeviationFromCenter - (KeepInside / Handling.Grip);


            //If there is a corner, go to the outside until we are close
            if (mem.Corner.Valid && Lap > 0 && Car.Velocity.Length() > mem.Corner.Speed * 0.9f)
            {
                CornerPoint c = mem.Corner.OG;
                float distToApex = (ARS.TrackPoints[c.Node].Position - ARS.TrackPoints[CurrentTrackPoint.Node].Position).Length();
                float distToCornerEntrance = (ARS.TrackPoints[c.Node - c.LengthStart].Position - ARS.TrackPoints[CurrentTrackPoint.Node].Position).Length();
                float sToReachCornerStart = ARS.Clamp(distToCornerEntrance / Car.Velocity.Length(), 0, 99);
                float sToReachCornerApex = ARS.Clamp(distToApex / Car.Velocity.Length(), 0, 99);
                mem.Corner.sToEntrance = sToReachCornerStart;

                //Keep yourelf on the outside when approaching a corner.
                if (sToReachCornerStart < 5)
                {
                    float Fraction = DistToInside(ARS.TrackPoints[c.Node].Direction) / (CurrentTrackPoint.TrackWide * 2);
                    Outside = ARS.TrackPoints[c.Node].TrackWide * (c.Angle > 0 ? 1 : -1);
                    Inside = -Outside;
                    float sNeccesarytoTurnIn = (ARS.TrackPoints[c.Node].TrackWide * 0.4f);
                    float mult = ARS.map(Car.Velocity.Length() - mem.Corner.Speed, -15, -5, 0, 1, true);
                    if (sToReachCornerApex < 5 && sToReachCornerApex > sNeccesarytoTurnIn * Fraction) TargetLane = Outside * mult;
                }
            }


            //vs other cars
            if (1 == 1)
            {
                float urgentAvoid = 0;
                if (Car.Velocity.Length() > 10)
                {
                    foreach (Rival rInteract in mem.Rivals.OrderByDescending(r => r.Distance))
                    {
                        if (rInteract.RivalRacer == null) continue;

                        rInteract.OvertakeLane = 0;
                        float DistToRear = rInteract.rPos.Y - rInteract.BoundingBoxTotal.Y;

                        bool SideToSide = Math.Abs(rInteract.rPos.Y) < rInteract.BoundingBoxTotal.Y - 0.1f;
                        bool WeAreBehind = DistToRear > 0;
                        bool Approaching = Math.Abs(rInteract.DirectionDiff) < 15 && rInteract.sToRear < 3f;
                        bool WithinOurBoundsAhead = Math.Abs(rInteract.rPos.X) < vData.BoundingBox;
                        bool ShouldAvoid = SideToSide || (Approaching && ((mem.Corner.Valid && mem.Corner.sToEntrance < 2.0f) || Decisions.Any()));
                        bool AheadWouldCollide = WeAreBehind && Math.Abs(rInteract.rPos.X) < rInteract.BoundingBoxTotal.X;

                        if (mem.Corner.Valid && rInteract.relativePos == RelativePos.Ahead && mem.Corner.sToEntrance < 5)
                        {
                            float stoCorner = (mem.Corner.OG.Node - CurrentTrackPoint.Node) / Car.Velocity.Length();

                            if (!Decisions.ContainsKey(Decision.AttackInside) && stoCorner > 2.0f && Math.Abs(AngleToTrackDir(CurrentTrackPoint, rInteract.RivalRacer.CurrentTrackPoint)) < 30 && (rInteract.sToRear < stoCorner))
                            {
                                int activeRivals = mem.Rivals.Where(i => i.RivalRacer != null && i.Distance < 30).Count() - 1;
                                float rivalsClose = ARS.map(activeRivals, 3, 0, 0, 1, true);
                                MakeDecision(Decision.AttackInside, (int)((mem.intention.Aggression * 100) * rivalsClose), 6000, 3000, 3000);
                            }
                        }

                        if (AheadWouldCollide) mem.intention.MaxSpeed = rInteract.RivalRacer.Car.Velocity.Length() + ARS.map(DistToRear, 1, 5, 0, 30, true);

                        if (SideToSide)
                        {
                            if (rInteract.rPos.X > 0 == rInteract.DirectionDiff > 0)
                            {
                                float sideToMaxAngle = ARS.map(Math.Abs(rInteract.rPos.X), rInteract.BoundingBoxTotal.X + 2, rInteract.BoundingBoxTotal.X, 0, 1, true);
                                urgentAvoid = rInteract.DirectionDiff * sideToMaxAngle;

                                //If the dude is behind us we shouldn't swerve out as aggressively.
                                urgentAvoid *= ARS.map(rInteract.rPos.Y, -rInteract.BoundingBoxTotal.Y, 0, 0.5f, 1, true);
                            }
                        }

                        if (!SideToSide && vData.LongitudinalGs > 0.05f && Math.Abs(rInteract.DirectionDiff) < 20 && WeAreBehind && (!mem.Corner.Valid || mem.Corner.sToEntrance > 3 || Decisions.Count > 0))
                        {
                            float devDiff = rInteract.RivalRacer.mem.data.DeviationFromCenter - mem.data.DeviationFromCenter;
                            if (Math.Abs(devDiff) < vData.BoundingBox + 1)
                            {
                                float offset = ARS.Clamp((vData.BoundingBox + 1f - Math.Abs(devDiff)) * (devDiff < 0 ? 1 : -1), -10f, 10f);
                                if (offset > 0 == mem.data.DeviationFromCenter > 0 && CurrentTrackPoint.TrackWide - Math.Abs(mem.data.DeviationFromCenter) < vData.BoundingBox + 1) offset *= -0.5f;

                                float avoidDist = ARS.map(DistToRear, 5, 2.5f, 0, 1, true);
                                float avoidRearEnd = ARS.map(rInteract.sToRear, 3, 1f, 0, 1, true);
                                TargetLane += offset * Math.Max(avoidDist, avoidRearEnd);
                            }
                        }

                        //Side to side
                        if (ShouldAvoid)
                        {
                            float trackdirDiff = AngleToTrackDir(CurrentTrackPoint, rInteract.RivalRacer.CurrentTrackPoint);
                            //Right of us
                            if (rInteract.rPos.X > 0)
                            {
                                float extra = ARS.map(AngleOneSec, -45, 0, -1, 1, true);
                                float t = rInteract.OccupiedLane - rInteract.BoundingBoxTotal.X - extra;
                                if (t < MaxRightLane)
                                {
                                    MaxRightLane = t;
                                    if (!SideToSide && Math.Abs(trackdirDiff) < 10 && MaxRightLane < -roadWide) MaxRightLane = rInteract.OccupiedLane;
                                }
                            }
                            else
                            {
                                float extra = ARS.map(AngleOneSec, 45, 0f, -1, 1, true);
                                float t = rInteract.OccupiedLane + rInteract.BoundingBoxTotal.X + extra;
                                if (t > MaxLeftLane)
                                {
                                    MaxLeftLane = t;
                                    if (!SideToSide && Math.Abs(trackdirDiff) < 10 && MaxLeftLane > roadWide) MaxLeftLane = rInteract.OccupiedLane;
                                }
                            }
                        }
                    }
                }
            }
            TargetLane = ARS.Clamp(TargetLane, MaxLeftLane, MaxRightLane);
            vControl.FollowLane = TargetLane;


            TrackPoint FTSteerRef = LookAheads[eLookAheads.SteerRef];
            LanePID.SetTarget(vControl.FollowLane);

            SteerTarget = FTSteerRef.Position - (Vector3.Cross(Vector3.WorldUp, FTSteerRef.Direction) * LanePID.GetValue());
            vControl.SteerTrackDegrees = -Vector3.SignedAngle((SteerTarget - Car.Position).Normalized, Car.Velocity.Normalized, Vector3.WorldUp) ;


            if (float.IsNaN(vControl.SteerTrackDegrees) || float.IsInfinity(vControl.SteerTrackDegrees)) vControl.SteerTrackDegrees = 0f;
        }


        void SteerCorrections()
        {
            float currentSpeed = Car.Velocity.Length();
            float SpeedBasedSteeringLimit = (float)((vData.BaseMechanicalGrip * Handling.Gravity * vData.WheelBase) / Math.Pow(Car.Velocity.Length()+0.01f, 2f));            
            SpeedBasedSteeringLimit = Math.Max(ARS.rad2deg(SpeedBasedSteeringLimit), 3);
            
            vControl.SteerTrackDegrees*=ARS.map(Math.Abs(vControl.SteerTrackDegrees),0,45,0.25f,1,true);

            if (Math.Sign(vControl.SteerTrackDegrees) != Math.Sign(vData.YawRotationPerSecondDegrees))
            {
                vControl.SteerTrackDegrees *= 0.5f;
            }
            vControl.SteerTrackDegrees = ARS.Clamp(vControl.SteerTrackDegrees,-SpeedBasedSteeringLimit, SpeedBasedSteeringLimit);
            float expectedYawRate = ARS.rad2deg((float)(Car.Velocity.Length() * Math.Tan(ARS.deg2rad(vControl.SteerTrackDegrees))) / vData.WheelBase);

            float actualYawRate = vData.YawRotationPerSecondDegrees;

            //expectedYawRate = ARS.Clamp(expectedYawRate, -45, 45);
            float OversteerFactor = Math.Abs(actualYawRate) - Math.Abs(expectedYawRate);

            // If steering and rotation are opposite directions, maybe register as understeer?
            // Currently it stays registered as oversteer because it helps counter oversteer nicely, but it makes little sense.
            if (Math.Sign(expectedYawRate) != Math.Sign(actualYawRate) && Math.Abs(expectedYawRate) >=0.01f)
            {                
                //OversteerFactor = -Math.Abs(OversteerFactor); // Force negative (understeer)
            }
            
            bool isOversteering = OversteerFactor > 0f;
            bool isUndersteering = OversteerFactor < -0f;

            if (isOversteering)
            {
                float factor = OversteerFactor * 0.8f * Math.Sign(vData.YawRotationPerSecondDegrees);
                vControl.SteerTrackDegrees -= factor;                
            }


            // === UNUSED FO NOW ===
            float turnRadius = vData.WheelBase / (float)Math.Tan(Math.Abs(ARS.deg2rad(vControl.SteerTrackDegrees))); 
            float minTurnRadius = (float)Math.Pow(currentSpeed, 0.99f) / vData.BaseMechanicalGrip;
            float maxSteeringFromPhysics = ARS.rad2deg((float)Math.Atan((vData.WheelBase / minTurnRadius)));
 
            //Correct slides gradually. If we are already centering ourselves, don't correct as much, we good.
            float SlideCounterSteer = vData.SlideAngle* ARS.map(Math.Abs(vData.SlideAngle), 0, Handling.TRlateral*1.2f, 0.5f, 1.2f, true);
            if (Math.Sign((int)vData.SlideAngle) == Math.Sign((int)vData.YawRotationPerSecondDegrees)) vControl.SteerTrackDegrees -= SlideCounterSteer;


            SteerPID.SetTarget(vControl.SteerTrackDegrees,90);
        }

        /// <summary>
        /// In meters. Negative = outside the track, on the outside of the current corner.
        /// </summary>
        /// <returns></returns>
        float DistToOutside(Vector3 direction)
        {
            if (Vector3.SignedAngle(CurrentTrackPoint.Direction, direction, Vector3.WorldUp) < 0.0f) return (float)Math.Round(CurrentTrackPoint.TrackWide + mem.data.DeviationFromCenter, 1);
            else return (float)Math.Round(CurrentTrackPoint.TrackWide - mem.data.DeviationFromCenter, 1);
        }

        float DistToInside(Vector3 direction)
        {
            if (Vector3.SignedAngle(CurrentTrackPoint.Direction, direction, Vector3.WorldUp) > 0.0f) return (float)Math.Round(CurrentTrackPoint.TrackWide + mem.data.DeviationFromCenter, 1);
            else return (float)Math.Round(CurrentTrackPoint.TrackWide - mem.data.DeviationFromCenter, 1);
        }

        public void Launch()
        {
            foreach (Racer r in ARS.Racers) r.mem.Corner = new Corner(5, ARS.CornerPoints.FirstOrDefault(c => c.IsKey));

            mem.Corner.Valid = false;
            vData.AvgGroundStability = 1;
            StuckRecover = false;
            StuckGameTimeRef = 0;
            BaseBehavior = RacerBaseBehavior.Race;
            LapStartTime = Game.GameTime;
            vControl.HandBrakeTime = Game.GameTime + ARS.GetRandomInt(100, 400);
            vControl.MaxThrottle = 1f;
            if (team == Team.Cop) Car.SirenActive = true;

        }

        void SpeedToThrottleBrake()
        {
            float newThrottle = 0f;
            float newBrake = 0f;

            //Keep still with throttle up when waiting for the launch
            if (BaseBehavior == RacerBaseBehavior.GridWait)
            {
                //float power = Function.Call<float>(Hash.GET_VEHICLE_ACCELERATION, Car) * 3.33f; //Torque in first gear
                mem.intention.Speed = 99f;
                vControl.HandBrakeTime = Game.GameTime + 500;
            }


            //Limit the th the input to the car's top speed. Cars can surpass defined engine top speed in V
            mem.intention.Speed = Math.Min(mem.intention.Speed, ARS.EngineTopSpeed(Car) * 1.3f);
            mem.intention.Speed = Math.Min(mem.intention.Speed, mem.intention.MaxSpeed);

            if ((Game.GameTime - LapStartTime) < 3000) mem.intention.IntendedSpdChangeGs = 999;
            else
            {
                //Intention vs Current
                mem.intention.IntendedSpdChangeGs = (mem.intention.Speed - Car.Velocity.Length()) / 9.8f;
            }

            if (mem.intention.IntendedSpdChangeGs > 0.0f) newThrottle = ARS.Clamp((mem.intention.IntendedSpdChangeGs) * 2, 0, Math.Min(vControl.TCSThrottle, 1)); else newThrottle = 0f;
            if (mem.intention.IntendedSpdChangeGs < -0.0f) newBrake = ARS.Clamp(-(mem.intention.IntendedSpdChangeGs) * 2, 0, 1); else newBrake = 0f;


 
            if (newBrake > 0.0) newThrottle = 0;
            if (newThrottle > 0.0) newBrake = 0;


            vControl.MaxThrottle = Math.Min(vControl.MaxThrottle, vData.AvgGroundStability);

            vControl.Brake += (newBrake - vControl.Brake) * 5 * TickScale;
            vControl.Throttle += (newThrottle - vControl.Throttle) * 5 * TickScale;
            vControl.Throttle = Math.Min(vControl.Throttle, vControl.MaxThrottle);
            if (vControl.MaxThrottle < 1.00f) vControl.MaxThrottle += 2 * TickScale;

            if (mem.intention.MaxSpeed < AIData.MaxSpeed) mem.intention.MaxSpeed += 15 * TickScale;


        }
        float TickScale => (0.001f * TimeSinceLastCoreTick);


        /// <summary>
        /// Translates existing info into a steering input for the car.
        /// </summary>
        void TranslateSteer()
        {

            if (float.IsNaN(vControl.SteerTrackDegrees) || float.IsInfinity(vControl.SteerTrackDegrees)) vControl.SteerTrackDegrees = 0f;
            if (float.IsNaN(vControl.SteerManeuver) || float.IsInfinity(vControl.SteerManeuver)) vControl.SteerManeuver = 0f;

            float finalSteer = SteerPID.GetValue();


            if (float.IsNaN(vControl.SteerInput) || float.IsInfinity(vControl.SteerInput)) vControl.SteerInput = 0f;

            vControl.SteerInput = ARS.map(finalSteer, -vData.SteeringLock, vData.SteeringLock, -1, 1, true);
            vControl.SteerCurrent = finalSteer;
        }

        float DEVGripExtra = 0;
        /// <summary>
        /// Figures out the ideal speed to be at at the moment
        /// </summary>
        public void SpeedTrack()
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

            float cornerSpd = 999f;
            float followTrackSpd = 999f;
            if (mem.Corner.Valid)
            {
                cornerSpd = Math.Max(2, ARS.MapIdealSpeedForDistance(mem.Corner.OG, this) * RiskFactorForGrip());
            }

            followTrackSpd = (float)Math.Sqrt(((vData.CurrentMechanicalGrip) * Handling.Gravity) * mem.data.CurveRadiusToFollowPoint) * RiskFactorForGrip();

            float latGs = ARS.LeftOrRight(Car.Position + (vData.Gs / 9.8f * 2), Car.Position, Car.ForwardVector);
            if (CurrentTrackPoint.GeneralCurveRadius < 2000 && Math.Abs(latGs) > vData.CurrentMechanicalGrip * 0.5f)
            {
                float confidenceChange = 0;
                Vector3 fPosition = Car.Position + ((Car.Velocity + vData.Gs * ARS.TracjectoryProjectionSeconds) * ARS.TracjectoryProjectionSeconds);
                int rTrackpoint = CurrentTrackPoint.Node + (int)(Car.Velocity.Length() * ARS.TracjectoryProjectionSeconds);
                if (rTrackpoint > 100 && rTrackpoint < ARS.TrackPoints.Last().Node - 100)
                {
                    TrackPoint tr = ARS.TrackPoints[rTrackpoint];
                    float lane = ARS.LeftOrRight(fPosition, tr.Position, tr.Direction);
                    if (CurrentTrackPoint.Angle > 0f)
                    {
                        if (lane < 0) confidenceChange = ARS.map(lane, 0, -tr.TrackWide, 0, 1, true) * TickScale;
                        else confidenceChange = ARS.map(lane, tr.TrackWide * 2, 0, -1, 0, true) * TickScale;
                    }
                    else
                    {
                        if (lane > 0) confidenceChange = ARS.map(lane, 0, tr.TrackWide, 0, 1, true) * TickScale;
                        else confidenceChange = ARS.map(lane, -tr.TrackWide * 2, 0, -1, 0, true) * TickScale;
                    }
                }
             }
 

            //mem.intention.CornerConfidence = ARS.Clamp(mem.intention.CornerConfidence, -1.25f, 1);
            if (cornerSpd <= 5) cornerSpd = ARS.GetSpeedForCorner(mem.Corner.OG, this);
            if (followTrackSpd < 1) followTrackSpd = 1;
            mem.intention.Speed = Math.Min(cornerSpd * 1.1f, followTrackSpd * 1.1f);

            // Extra limiter for cars pointing outward in a corner.
            if (mem.Corner.Valid)
            {           
                float angleToTrack = Vector3.SignedAngle(LookAheads[eLookAheads.HalfSec].Direction, Car.ForwardVector, Vector3.WorldUp);
                if (Math.Abs(angleToTrack) > 0.0f)
                {
                    float cornerDir = mem.Corner.OG.Angle;
                    bool goingOutward = Math.Abs(cornerDir) > 0.01f && Math.Sign(angleToTrack) != Math.Sign(cornerDir);
                    float outwardThresholdDeg = ARS.map(DistToOutside(CurrentTrackPoint.Direction), CurrentTrackPoint.TrackWide, CurrentTrackPoint.TrackWide*2, 0,45,true);

                    if (goingOutward && Math.Abs(angleToTrack) > outwardThresholdDeg)
                    {
                        float speedReduction = (Math.Abs(angleToTrack) - outwardThresholdDeg) / 5f;
                        mem.intention.MaxSpeed = Math.Max(followTrackSpd-10, followTrackSpd - speedReduction);
                        
                    }
                }

            }

 
            if (Math.Sign((int)SteerPID.GetValue()) == Math.Sign((int)vData.YawRotationPerSecondDegrees))
            {
                float velocity = Car.Velocity.Length();

                //float angleToTrack = Vector3.SignedAngle(Car.Velocity.Normalized)
                // Simple approximation (angle must be in RADIANS)
                float steeringAngleRad = Math.Abs(SteerPID.GetValue() * 1f) * (float)(Math.PI / 180);

                float intendedYawRate = (velocity * (float)Math.Tan(steeringAngleRad)) / vData.WheelBase;
                intendedYawRate *= (180.0f / (float)Math.PI); // Convert to degrees/sec


                float maxYawRateFromGrip = ((vData.CurrentMechanicalGrip * Handling.Gravity) / velocity) * (180.0f / (float)Math.PI);

                float speedAdjust = (Math.Abs(maxYawRateFromGrip * 0.99f) - Math.Abs(intendedYawRate)) / 10;
                if (float.IsNaN(speedAdjust)) speedAdjust = 1;


                //mem.intention.MaxSpeed = Math.Max((int)(followTrackSpd * 0.8f), (int)(velocity + speedAdjust));

            }

            //Aggro buildup
            float aggroDiffToTarget = mem.intention.AggroToReach - mem.intention.Aggression;
            if (aggroDiffToTarget > 0.0f) mem.intention.Aggression += mem.personality.Rivals.AggressionBuildup * TickScale; else mem.intention.Aggression -= mem.personality.Rivals.AggressionBuildup * TickScale * 2;

        }

        void MakeDecision(Decision d, int chance, int duration, int sCooldown, int fCooldown)
        {
            if (chance == 0 || mem.personality.Rivals.ManeuverRiskFactor == 0f || !ARS.DevSettingsFile.GetValue("RACERS", "AllowManeuvers", false)) return;
            if (!BannedDecisions.ContainsKey(d))
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
            /*
            if (vControl.Brake > 0.0f)
            {
                float BrakeStabilityAdjust = 0;
                BrakeStabilityAdjust = ARS.map(wheelspin, 2f, 4f, -2f, 2f, true);
                vControl.CurrentLockupLimiter -= BrakeStabilityAdjust * TickScale;
                vControl.CurrentLockupLimiter = ARS.Clamp(vControl.CurrentLockupLimiter, 0.33f, 1f);
            }
            */
            if (vControl.Throttle > 0.0f)
            {
                float allowedWheelspin = 0.4f;
                if (GameTimeOutOfTrack != 0 && (Game.GameTime - GameTimeOutOfTrack) > 500) allowedWheelspin = 0.1f;
                if (Decisions.ContainsKey(Decision.Flatout)) { allowedWheelspin = 90; }

                float TCSValue = ARS.map(Math.Abs(wheelspin)- allowedWheelspin, 0.1f, -0.1f, -1f, 1f, true);
                float change = TCSValue * TickScale;
                vControl.TCSThrottle = ARS.Clamp(vControl.TCSThrottle + change, 0.1f, 1);
            }
        }
        public void UpdateTickData()
        {
            SteerPID.Update();
            LanePID.Update();
            vData.LocalGs = (Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true) - vData.SpeedVectorLocal) / Game.LastFrameTime;
            Vector3 cSpeed = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, false);

            vData.AccelerationVector.Add((cSpeed - lSpeed) / Game.LastFrameTime);

            if (vData.AccelerationVector.Count > 10) vData.AccelerationVector.RemoveAt(0);

            lSpeed = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, false);
            vData.SpeedVectorGlobal = cSpeed;
            vData.SpeedVectorLocal = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true);
            mem.data.SpeedVector = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true);

            if (trail.Count > 50) trail.RemoveAt(0);
        }
        public VehData vData = new VehData();
        public HandlingData Handling = new HandlingData();

        /// <summary>
        /// Gathers and runs tick-sensitive stuff
        /// </summary>
        public void ProcessTick()
        {
            UpdateTickData();
            DrawStuff();

            if (!Driver.IsPlayer)
            {
                TorqueMult = ARS.map(Math.Abs(vData.SlideAngle), 5f, 90f, 1f, 10);
                //if (TorqueMult > 1.0f && Car.CurrentGear > 0) Car.EngineTorqueMultiplier = TorqueMult;

                ApplyInputs();


                //Catchup
                if (ARS.Racers.Count > 1)
                {
                    if (RacePosition > ARS.catchupPos && 1 == 0)
                    {
                        if (!ARS.SettingsFile.GetValue("CATCHUP", "OnlyLoners", true) || !mem.Rivals.Any(v => v.Distance < 50))
                        {
                            if (ARS.SettingsFile.GetValue("CATCHUP", "CatchupSpeed", 100) != 0 && vControl.HandBrakeTime < Game.GameTime)
                            {
                                if (ARS.GetPercent(RacePosition, ARS.Racers.Count) >= 40)//&& !NearbyRivals.Any()
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
        int LastCoreTick = 100;
        int TimeSinceLastCoreTick => (int)ARS.Clamp(Game.GameTime - LastCoreTick, 1, 9999);
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
            vData.BoundingBox = ARS.GetDirectionalBoundingBox(Car);
            vData.SlideAngle = (float)Math.Round(Vector3.SignedAngle(Car.Velocity.Normalized, Car.ForwardVector, Car.UpVector), 3);
        }
        void UpdateCornerInfo()
        {
            if (mem.Corner == null) return;

            if (mem.Corner.Valid && Lap > 0)
            {
                if (CurrentTrackPoint.Node > mem.Corner.OG.Node || (Math.Abs(CurrentTrackPoint.Node - mem.Corner.OG.Node) > 1000))
                {
                    mem.Corner.Valid = false;
                    BehaviorVariance[RandomVariance.SpeedAggroVariance] = Random.Next(80, 120) * 0.01f;
                    BehaviorVariance[RandomVariance.BrakeDistance] = Random.Next(80, 120) * 0.01f;
                 }
            }

            //To avoid vanilla contact-swerves, sit the driver on the passenger seat if there's a potential colission. Also try and fix the helmet
            if (Car.Model.IsCar && !Driver.IsPlayer)
            {
                if (mem.Rivals.Any(v => v.Distance < v.BoundingBoxTotal.Y + 1))
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
            if (ARS.SettingsFile.GetValue<bool>("GENERAL_SETTINGS", "Ghosts", false) && mem.Rivals.Any(v => v.Distance < 50))
            {
                foreach (Rival r in mem.Rivals)
                {
                    //if (Function.Call<bool>(Hash.IS_ENTITY_AT_ENTITY, Car, r.Car, (r.Car.Model.GetDimensions().X * 1.5f), (r.Car.Model.GetDimensions().Y * 1.5f), (r.Car.Model.GetDimensions().Z * 1.5f), true, true, true))
                    if (Car.IsInRangeOf(r.RivalRacer.Car.Position, 6))
                    {
                        Function.Call(Hash.SET_ENTITY_NO_COLLISION_ENTITY, r.RivalRacer.Car, Car, true);
                        Car.Alpha = 150;
                    }
                }
            }


            //Reset into track logic
            if ((Car.IsUpsideDown || Math.Abs(mem.data.DeviationFromCenter) > CurrentTrackPoint.TrackWide) && !ControlledByPlayer && BaseBehavior == RacerBaseBehavior.Race)
            {
                if (GameTimeOutOfTrack == 0) GameTimeOutOfTrack = Game.GameTime;
                else if (Game.GameTime - GameTimeOutOfTrack > 5000 && Car.Velocity.Length() < 5f)
                {
                    GameTimeOutOfTrack = 0;
                    ResetIntoTrack();
                }
            }
            else
            {
                if (GameTimeOutOfTrack != 0) GameTimeOutOfTrack = 0;
            }
        }

        public void ApplyInputs()
        {

            //AI Inputs
            if (Driver.IsSittingInVehicle(Car) && !Driver.IsPlayer)
            {
                if (vControl.HandBrakeTime > Game.GameTime) Car.HandbrakeOn = true; else Car.HandbrakeOn = false;

                ARS.SetThrottle(Car, ARS.Clamp(vControl.Throttle, -1, 1));
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
            if ((Car.Position - Game.Player.Character.Position).Length() > 50) return;


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

                //Center of Gs
                World.DrawMarker(MarkerType.DebugSphere, Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.1f, 0.1f, 0.1f), Color.Green, false, false, 0, false, "", "", false);

                //Gs
                Vector3 avgGs = vData.AccelerationVector.Aggregate(new Vector3(0, 0, 0), (s, v) => s + v) / (float)vData.AccelerationVector.Count;
                avgGs.Z = 0f;

                float colorPercent = ARS.map(avgGs.Length() / 9.8f, 0, vData.CurrentMechanicalGrip, 0, 100, true);
                Color gColor = ARS.GradientAtoBtoC(Color.White, Color.Yellow, Color.Red, colorPercent);

                World.DrawMarker(MarkerType.DebugSphere, Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)) + (avgGs / 9.8f), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.15f, 0.15f, 0.15f), gColor, false, false, 0, false, "", "", false);
                ARS.DrawLine(Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)) + (avgGs / 9.8f), Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)), gColor);


                Vector3 maxValues = new Vector3(vData.CurrentMechanicalGrip * 9.8f, vData.CurrentMechanicalGrip * 9.8f, vData.CurrentMechanicalGrip * 9.8f);
                Vector3 max = Vector3.Clamp(avgGs, -maxValues, maxValues);



                Vector3 source = Car.Position + new Vector3(0, 0, 0.5f + (Car.Model.GetDimensions().Z * 0.6f));
                ARS.DrawText(source, "~b~" + Math.Round(ARS.MStoMPH(Car.Velocity.Length())).ToString() + "~w~mph~n~~y~" + (avgGs.Length() / 9.8f).ToString("0.0") + " Gs", Color.White, 0.5f);


            }

            if (!Driver.IsPlayer)
            {

                if (!Car.IsInRangeOf(Game.Player.Character.Position, 500)) return;

                if (ARS.OptionValuesList[Options.ShowAggro])
                {
                    if (Decisions.Any()) World.DrawMarker(MarkerType.ChevronUpx2, Car.Position + new Vector3(0, 0, 1.5f), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.5f, 0.5f, -0.5f), ARS.GetColorFromRedYellowGreenGradient(100 - (mem.intention.Aggression * 100)), false, true, 0, false, "", "", false);
                    else World.DrawMarker(MarkerType.ChevronUpx1, Car.Position + new Vector3(0, 0, 1.5f), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.5f, 0.5f, -0.5f), ARS.GetColorFromRedYellowGreenGradient(100 - (mem.intention.Aggression * 100)), false, true, 0, false, "", "", false);

                    string text = "";
                    foreach (Decision d in Decisions.Keys) text += d.ToString() + "~n~";
                    ARS.DrawText(Car.Position + new Vector3(0, 0, -1), text, Color.SkyBlue, 0.5f);

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


                if (ARS.OptionValuesList[Options.ShowInputs])
                {
                    Vector3 nSteer = SteerTarget + Vector3.WorldUp;
                    /*
                                        Vector3 fPosition = Car.Position + ((Car.Velocity + vehData.Gs * ARS.TracjectoryProjectionSeconds) * ARS.TracjectoryProjectionSeconds);
                                        for (float i = 0; i < 1; i += 0.1f)
                                        {

                                            Vector3 p2 = ARS.QuadraticBezier(Car.Position, Car.Position + Car.Velocity * (ARS.TracjectoryProjectionSeconds / 2), Car.Position + ((Car.Velocity + vehData.Gs) * ARS.TracjectoryProjectionSeconds), i);
                                            World.DrawMarker(MarkerType.DebugSphere, p2, Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.15f, 0.15f, 0.15f), Color.Azure, false, false, 0, false, "", "", false);

                                        }
                                        int rTrackpoint = CurrentTrackPoint.Node + (int)(Car.Velocity.Length() * ARS.TracjectoryProjectionSeconds);
                                        if(rTrackpoint<ARS.TrackPoints.Count-100) ARS.DrawLine(ARS.TrackPoints[rTrackpoint].Position, fPosition, Color.White);


                                        ARS.DrawLine(Car.Position, nSteer, Color.Gray);

                                        */

                    Vector3 dev = Car.Position + (Vector3.Cross(Car.ForwardVector, Vector3.WorldUp) * (LanePID.GetValue() - mem.data.DeviationFromCenter));
                    ARS.DrawLine(Car.Position, dev + Vector3.WorldUp, Color.White);

                    Color cc = Color.Green;
                    if (vControl.Brake > 0.0f) cc = Color.Yellow;
                    if (vControl.Brake > 0.5f) cc = Color.Orange;
                    if (vControl.Brake > 0.9f) cc = Color.Red;

                    Vector3 velocity = Car.Velocity.Normalized;
                    Vector3 back = Car.ForwardVector;
                    back.Z = velocity.Z;

                    Vector3 Dimensions = Car.Model.GetDimensions();
                    Vector3 inputplace = Car.Position + new Vector3(0, 0, -Car.HeightAboveGround);
                    Vector3 strCurent = Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * vControl.SteerCurrent) * (Car.ForwardVector * (Car.Position - SteerTarget).Length()); // Quaternion.RotationAxis takes radian angles
                                                                                                                                                                                                 //strCurent.Z = SteerTarget.Z;


                    Vector3 maxLeft = Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * vControl.SteerMax) * (Car.ForwardVector * (Car.Position - SteerTarget).Length()); // Quaternion.RotationAxis takes radian angles
                    Vector3 maxRight = Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * -vControl.SteerMax) * (Car.ForwardVector * (Car.Position - SteerTarget).Length()); // Quaternion.RotationAxis takes radian angles



                    float dimension = Car.Model.GetDimensions().Y + 1f;

                    Vector3 steerAim = Car.Position + strCurent;
                    steerAim.Z = nSteer.Z;

                    //ARS.DrawLine(Car.Position, steerAim, Color.Blue);
                    //World.DrawMarker(MarkerType.DebugSphere, steerAim, Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), Color.Blue);

                    steerAim = Car.Position + maxLeft;
                    steerAim.Z = nSteer.Z;
                    //ARS.DrawLine(Car.Position, steerAim, Color.Red);
                    //World.DrawMarker(MarkerType.DebugSphere, steerAim, Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), Color.Red);

                    steerAim = Car.Position + maxRight;
                    steerAim.Z = nSteer.Z;
                    //ARS.DrawLine(Car.Position, steerAim, Color.Red);
                    //World.DrawMarker(MarkerType.DebugSphere, steerAim, Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), Color.Red);


                    if (1 == 1)
                    {
                        Vector3 inputThrottle = inputplace + (Car.ForwardVector * (Dimensions.Y * 0.5f) * vControl.Throttle);
                        Vector3 inputBrake = inputplace - (Car.ForwardVector * (Dimensions.Y * 0.5f) * vControl.Brake);

                        Color cThrottle = ARS.GradientAtoB(Color.White, Color.Green, vControl.Throttle * 100);
                        Color cBrake = ARS.GradientAtoB(Color.White, Color.Red, vControl.Brake * 100);

                        if (vControl.Throttle > 0.05f) World.DrawMarker(MarkerType.ChevronUpx1, inputThrottle, -Car.ForwardVector, new Vector3(90, 0, 0), new Vector3(dimension / 2, dimension / 4, -(dimension / 2)), Color.FromArgb(250, cThrottle), false, false, 0, false, "", "", false);
                        if (vControl.Brake > 0.05f) World.DrawMarker(MarkerType.ChevronUpx1, inputBrake, Car.ForwardVector, new Vector3(90, 0, 0), new Vector3(dimension / 2, dimension / 4, -(dimension / 2)), Color.FromArgb(250, cBrake), false, false, 0, false, "", "", false);

                    }
                }

                if (ARS.OptionValuesList[Options.ShowTrackAnalysis])
                {
                    Vector3 source = Car.Position + new Vector3(0, 0, 0.5f + (Car.Model.GetDimensions().Z * 0.6f));
                    //ARS.DrawText(source, "~b~" + Math.Round(ARS.MStoMPH(Car.Velocity.Length())).ToString() + "~w~mph~n~~y~" + Math.Round(avgGs.Length() / 9.8f, 2) + "Gs", Color.White, 0.5f);


                    if (mem.Corner.Valid && Lap > 0)
                    {
                        CornerPoint c = mem.Corner.OG;
                        {

                            Vector3 wp = ARS.Path[c.Node];
                            float expectedSpeed = c.Speed;
                            Color gColor = ARS.GetColorFromRedYellowGreenGradient(ARS.map(expectedSpeed - Car.Velocity.Length(), -1, 1, 0, 100, true));


                            Vector3 oldpos = Vector3.Zero;
                            for (int i = c.Node - (int)(c.GetRadius() * 2); i <= c.Node; i++)
                            {
                                float howFarBack = ARS.TrackPoints[i].TrackWide / 4;// ARS.map(ARS.TrackPoints[i].TrackWide, 5, 20, 1, 4, false);

                                int dist = c.Node - i;
                                float DistPercent = ((dist / (c.GetRadius() * howFarBack)) * 100);


                                float Outside = (ARS.TrackPoints[i].TrackWide - (vData.BoundingBox / 2)) * (c.Angle > 0 ? 1 : -1);
                                float Inside = (ARS.TrackPoints[i].TrackWide - (vData.BoundingBox / 2)) * (c.Angle < 0 ? 1 : -1);

                                float laneApproach = ARS.Lerp(Inside, Outside, ARS.LaneApproach(DistPercent) * 0.01f);

                                //if (Outside > 0) laneApproach = ARS.Clamp(laneApproach, -Outside, Outside); else laneApproach = ARS.Clamp(laneApproach, Outside, -Outside);

                                Vector3 dot = ARS.TrackPoints[i].Position - (Vector3.Cross(Vector3.WorldUp, ARS.TrackPoints[i].Direction) * laneApproach) + (Vector3.WorldUp / 4);

                                if (oldpos != Vector3.Zero) ARS.DrawLine(oldpos, dot, Color.Red);
                                oldpos = dot;
                                //World.DrawMarker(MarkerType.DebugSphere, dot, Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), Color.Red, false, false, 0, false, "", "", false);
                                //ARS.DrawText(dot + new Vector3(0, 0, 1), DistPercent.ToString("0.00")+"%\n"+ laneApproach.ToString("0.00"), Color.Orange, 0.33f);

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


                        //World.DrawMarker(MarkerType.ChevronUpx1, ARS.TrackPoints[cp.Node].Position - new Vector3(0, 0, 0.05f), ARS.TrackPoints[cp.Node].Direction, new Vector3(90, 0, 0), new Vector3(ARS.TrackPoints[cp.Node].TrackWide * 2.66f, 5, 5), Color.FromArgb(75, Color.SkyBlue));
                        TrackPoint t = ARS.TrackPoints[nodeOffset];

                        float followSpd = (float)Math.Sqrt(vData.CurrentMechanicalGrip * 9.8f * t.GeneralCurveRadius);
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



                //Line showing track direction
                //Vector3 inLane = CurrentTrackPoint.Position + (Vector3.Cross(CurrentTrackPoint.Direction, Vector3.WorldUp) * mem.data.DeviationFromCenter) + Vector3.WorldUp;
                //ARS.DrawLine(inLane + (CurrentTrackPoint.Direction * 5), inLane - (CurrentTrackPoint.Direction * 5), Color.SkyBlue);

                //Track limits display
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

        /// <summary>
        /// How far from the outside of the track we're in. Negative values = we're inside the track.
        /// </summary>
        /// <returns>Distance in meters</returns>
        float OutOfTrackDistance()
        {
            return (Math.Abs(mem.data.DeviationFromCenter) + (vData.BoundingBox / 2)) - CurrentTrackPoint.TrackWide;
        }

        void UpdateRivalInfo()
        {
            foreach (Rival r in mem.Rivals) r.Update(this);
            if (Car.Driver.IsPlayer || ARS.SettingsFile.GetValue<bool>("GENERAL_SETTINGS", "Ghosts", false)) return;
            mem.intention.AggroToReach = 0f;

            //AI shouldn't be aggresive while crowded
            if (mem.Rivals.Where(r => r.Distance < 50).Count() < 3)
            {
                Rival closestRival = mem.Rivals.OrderBy(v => v.Distance).FirstOrDefault();
                if (closestRival.RivalRacer != null)
                {
                    float aggroDist = closestRival.Distance;

                    float Closest = 100;
                    float Furthest = 200;

                    //If the rival is behind us, worry less about them
                    if (closestRival.RivalRacer.RacePosition > RacePosition) { Closest = 15; Furthest = 30; }

                    mem.intention.AggroToReach = ARS.map(aggroDist, Furthest, Closest, 0f, 1f, true);
                }
            }
        }

        public bool FinishedPointToPoint = false;
        public int localSPDLimiter = 0;
        public TrackPoint CurrentTrackPoint = new TrackPoint();
        public Vector3 SteerTarget = Vector3.Zero;

        public enum eLookAheads { SteerRef, QuarterSec, HalfSec, ThreeQuarterSec, OneSec, OneHalfSec, SteerInRef };
        public Dictionary<eLookAheads, TrackPoint> LookAheads = new Dictionary<eLookAheads, TrackPoint>();

        /// <summary>
        /// Figure out our point in the track.
        /// </summary>
        public void UpdateFollowTrack()
        {

            //Get our current track point data
            int RefTrackpoint = (int)ARS.Clamp(CurrentTrackPoint.Node, 0, ARS.TrackPoints.Count - 1);

            List<TrackPoint> Points = new List<TrackPoint>();
            for (int i = RefTrackpoint - 6; Points.Count <= 12; i++)
            {
                if (i < 0 || i >= ARS.TrackPoints.Count) i = 0;
                Points.Add(ARS.TrackPoints[i]);
            }

            CurrentTrackPoint = Points.OrderBy(t => t.Position.DistanceTo(Car.Position)).First();
            mem.data.DeviationFromCenter = ARS.LeftOrRight(Car.Position, CurrentTrackPoint.Position, CurrentTrackPoint.Direction);

            LookAheads.Clear();
            TrackPoint lookAhead;


            int SteerRef = (int)ARS.Clamp((int)((Car.Velocity.Length() * 1.8f / vData.CurrentMechanicalGrip)), 1, 500);
            if (CurrentTrackPoint.Node + SteerRef >= ARS.TrackPoints.Count) lookAhead = ARS.TrackPoints[(int)SteerRef];
            else lookAhead = ARS.TrackPoints[CurrentTrackPoint.Node + (int)SteerRef];
            LookAheads.Add(eLookAheads.SteerRef, lookAhead);


            int QuarterSec = (int)(Car.Velocity.Length() * 0.25);
            if (CurrentTrackPoint.Node + QuarterSec >= ARS.TrackPoints.Count) lookAhead = ARS.TrackPoints[(int)QuarterSec];
            else lookAhead = ARS.TrackPoints[CurrentTrackPoint.Node + (int)QuarterSec];
            LookAheads.Add(eLookAheads.QuarterSec, lookAhead);

            int HalfSec = (int)(Car.Velocity.Length() * 0.5);
            if (CurrentTrackPoint.Node + HalfSec >= ARS.TrackPoints.Count) lookAhead = ARS.TrackPoints[(int)HalfSec];
            else lookAhead = ARS.TrackPoints[CurrentTrackPoint.Node + (int)HalfSec];
            LookAheads.Add(eLookAheads.HalfSec, lookAhead);


            int ThreeQuarterSec = (int)(Car.Velocity.Length() * 0.75);
            if (CurrentTrackPoint.Node + ThreeQuarterSec >= ARS.TrackPoints.Count) lookAhead = ARS.TrackPoints[(int)ThreeQuarterSec];
            else lookAhead = ARS.TrackPoints[CurrentTrackPoint.Node + (int)ThreeQuarterSec];
            LookAheads.Add(eLookAheads.ThreeQuarterSec, lookAhead);

            int OneSec = (int)(Car.Velocity.Length());
            if (CurrentTrackPoint.Node + OneSec >= ARS.TrackPoints.Count) lookAhead = ARS.TrackPoints[OneSec];
            else lookAhead = ARS.TrackPoints[CurrentTrackPoint.Node + OneSec];
            LookAheads.Add(eLookAheads.OneSec, lookAhead);


            int OneHalfSec = (int)(Car.Velocity.Length() * 1.5f);
            if (CurrentTrackPoint.Node + OneHalfSec >= ARS.TrackPoints.Count) lookAhead = ARS.TrackPoints[OneHalfSec];
            else lookAhead = ARS.TrackPoints[CurrentTrackPoint.Node + OneHalfSec];
            LookAheads.Add(eLookAheads.OneHalfSec, lookAhead);



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
                TrackPoint end = ARS.TrackPoints[CurrentTrackPoint.Node + (int)Car.Velocity.Length()];
                TrackPoint midpoint = ARS.TrackPoints[CurrentTrackPoint.Node + (int)Car.Velocity.Length() / 2];
                mem.data.CurveRadiusToFollowPoint = ARS.GetCurveRadius(CurrentTrackPoint.Position, end.Position, midpoint.Position) / 2;
            }
            else mem.data.CurveRadiusToFollowPoint = 999;
            //ARS.GetCurveRadius(Car.Position - Car.Velocity + (vData.Gs / 2), Car.Position + Car.Velocity + (vData.Gs / 2), Car.Position);

            //mem.data.CurveRadiusToFollowPoint = Math.Min(LookAheads[eLookAheads.OneHalfSec].GeneralCurveRadius, LookAheads[eLookAheads.HalfSec].GeneralCurveRadius);


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
            if (mem.Corner == null) return;
            if (HalfSecondTick < Game.GameTime)
            {
                HalfSecondTick = Game.GameTime + 500 + (int)ARS.map(Car.Velocity.Length(), 0, 100, -250, 250, true);
                if (!mem.Corner.Valid && ARS.MStoMPH(Car.Velocity.Length()) > 10) ARS.LookForCornerAhead(this);

                //Decision cleaning
                if (Decisions.Any(de => de.Value < Game.GameTime)) Decisions.Remove(Decisions.First(de => de.Value < Game.GameTime).Key);
                if (BannedDecisions.Any(de => de.Value < Game.GameTime)) BannedDecisions.Remove(BannedDecisions.First(de => de.Value < Game.GameTime).Key);

                //Mistake cleaning
                if (Mistakes.Any(de => de.Value < Game.GameTime)) Mistakes.Remove(Mistakes.First(de => de.Value < Game.GameTime).Key);
                if (BannedMistakes.Any(de => de.Value < Game.GameTime)) BannedMistakes.Remove(BannedMistakes.First(de => de.Value < Game.GameTime).Key);

                //if (!Driver.IsPlayer) if (NearbyRivals.Count > 0) Driver.Task.LookAt(NearbyRivals[0].Driver, 2000); else if (Car.Velocity.Length() > 5f) Driver.Task.LookAt(Car.Position + Car.Velocity, 2000);
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
                if (1 == 1 || !mem.Corner.Valid || mem.Corner.sToEntrance > 4)
                {
                    if (BaseBehavior == RacerBaseBehavior.Race && Math.Abs(vData.SlideAngle) < 0.5f && Math.Abs(vControl.Throttle) > 0.9f) Function.Call((Hash)0x81E1552E35DC3839, Car, true);
                }

                if (Function.Call<bool>((Hash)0x3D34E80EED4AE3BE, Car) && vControl.Brake > 0.1f) Function.Call((Hash)0x81E1552E35DC3839, Car, false);


                if (ARS.DevSettingsFile.GetValue<int>("RACERS", "AIRacerAutofix", 1) == 2 && Function.Call<bool>(Hash._IS_VEHICLE_DAMAGED, Car))
                {
                    Car.Repair();
                }


                //Stuck behavior
                if (vData.Gs.Length() < 0.1f && Car.Velocity.Length() < 1f && BaseBehavior == RacerBaseBehavior.Race)
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
        /// </summary>
        public void ProcessAI()
        {
            ProcessTimedAI();

            if (BaseBehavior == RacerBaseBehavior.GridWait && vControl.HandBrakeTime < Game.GameTime) vControl.HandBrakeTime = Game.GameTime + (100 * ARS.GetRandomInt(2, 6));

            if (!ControlledByPlayer)
            {
                UpdateRivalInfo();
                SteerTrack();
                SpeedTrack();

                SteerCorrections();

                SpeedToThrottleBrake();
                TranslateSteer();
                TractionControl();
            }
        }


        void ResetIntoTrack()
        {
            vControl.SteerTrackDegrees = 0f;

            Car.Position = ARS.Path[CurrentTrackPoint.Node] + new Vector3(0, 0, 3);

            Car.Heading = CurrentTrackPoint.Direction.ToHeading();

            StuckRecover = false;
            LastStuckPlace = Vector3.Zero;

            Car.Speed = ARS.MPHtoMS(15);
        }

        void UpdatePercievedGrip()
        {

            //Base vehicle grip GetVehicleMaxTraction
            float HandlingGrip = Function.Call<float>((Hash)0xA132FB5370554DB0, Car) * (Handling.Gravity / 9.8f);
            HandlingGrip = ARS.Clamp(HandlingGrip, 0.1f, 5f);

            GroundGripMultiplier = ARS.GetWheelsGrip(Car).Average(); //Surface grip
            Vector3 thisPoint = CurrentTrackPoint.Position;
            Vector3 toMidpoint = LookAheads[eLookAheads.HalfSec].Position;
            Vector3 toEndpoint = LookAheads[eLookAheads.OneSec].Position;

            float elChangeDegrees = (toEndpoint.Normalized.Z - toMidpoint.Normalized.Z) * 90;


            /*
            float wetavg = ARS.GetWheelsWetgrip(Car).Average();
            if (wetavg < 1.0 && 1 == 2)
            {
                //UI.ShowSubtitle("Wet: -" + ((1 - wetavg) * 4), 500);
                HandlingGrip *= (SurfaceGrip - ((1 - wetavg) * ARS.GetNumWheels(Car)));
            }
            */

            float hillGsLoss = ARS.GetHillGripMultiplierAtCurrentVelocityVector(this,4);

            vData.BaseMechanicalGrip = HandlingGrip;
            vData.CurrentMechanicalGrip = ((vData.BaseMechanicalGrip) * GroundGripMultiplier);
            vData.CurrentMechanicalGrip *= hillGsLoss;
            vData.CurrentMechanicalGrip += Math.Min(ARS.WouldLiftOffRoadAtSpeed(thisPoint, toMidpoint, toEndpoint, Car.Velocity.Length()),0.0f);
            UI.ShowSubtitle(hillGsLoss.ToString("0.00"), 500);
            //vData.CurrentDownforce = ARS.GetDownforceGsAtSpeed(this, Car.Velocity.Length()) / 2;

            //If you're in first, map terrain multipliers for the rest of the racers
            if (Math.Abs(mem.data.DeviationFromCenter) < CurrentTrackPoint.TrackWide && RacePosition <= 2 && !ARS.MultiplierInTerrain.ContainsKey(CurrentTrackPoint.Node))
            {
                ARS.MultiplierInTerrain.Add(CurrentTrackPoint.Node, GroundGripMultiplier);
            }

            float zSpeedDegreesFromHoriz = (Math.Abs(vData.SpeedVectorLocal.Normalized.Z) * 90);

            //Checks for Z movement to judge wether the car is stable or not. Usually implies the vehicle is mid-air
            if (zSpeedDegreesFromHoriz > 5f)
            {
                if (vData.AvgGroundStability >= 0.1f) vData.AvgGroundStability -= zSpeedDegreesFromHoriz * TickScale * 0.1f;
            }
            else if (vData.AvgGroundStability < 1f) vData.AvgGroundStability += 1f * TickScale;

            if (vData.AvgGroundStability > 1.0f) vData.AvgGroundStability = 1f;

            vData.Gs = vData.AccelerationVector.Aggregate(new Vector3(0, 0, 0), (s, v) => s + v) / (float)(vData.AccelerationVector.Count) / 2;
            mem.data.CurveRadiusPhysicalGs = ARS.GetCurveRadius(Car.Position - Car.Velocity + (vData.Gs / 2), Car.Position + Car.Velocity + (vData.Gs / 2), Car.Position);

            vData.YawRotationPerSecondDegrees = ARS.rad2deg(Function.Call<Vector3>(Hash.GET_ENTITY_ROTATION_VELOCITY, Car).Z);            
        }



        public List<Vehicle> Traffic = new List<Vehicle>();
        public void UpdateRivals()
        {
            List<Racer> Candidates = new List<Racer>();
            foreach (Racer r in ARS.Racers)
            {
                if (r.Car.Handle != Car.Handle && r.Car.Position.DistanceTo(Car.Position) < 100f)
                {
                    Candidates.Add(r);
                }
            }

            foreach (Rival r in mem.Rivals) r.RivalRacer = null;
            if (Candidates.Count > 0)
            {
                Candidates.Sort((a, b) => Vector3.Distance(a.Car.Position, Car.Position).CompareTo(Vector3.Distance(b.Car.Position, Car.Position)));
                for (int i = 0; i < mem.Rivals.Count - 1; i++)
                {
                    if (i == Candidates.Count) break;
                    mem.Rivals[i].RivalRacer = Candidates[i];
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
            return 1.025f;
            return ARS.map(mem.intention.Aggression, 0, 1, mem.personality.Stability.SpeedRiskFactorBase, mem.personality.Stability.SpeedRiskFactorAggro, true);
        }
        public float RiskFactorForBrake()
        {
            return 1;
            return ARS.map(mem.intention.Aggression, 0, 1, mem.personality.Stability.BrakeRiskFactorBase, mem.personality.Stability.BrakeRiskFactorAggro, true);
        }
    }
}
