using GTA;
using GTA.Math;
using GTA.Native;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NewRacingSystem
{
    public enum RacerBaseBehavior
    {
        GridWait, Race, DamagedAvoid, FinishedRace, FinishedStandStill
    }
    public class Racer
    {

        public bool ControlledByPlayer = false;
        //Racer Info
        public List<int> LapTimes = new List<int>();
        public int StartTime = 0;
        public string Name = "Racer";
        public Ped Driver;
        public Vehicle Car;
        public List<string> DebugText = new List<string>();
        public List<Vector3> trail = new List<Vector3>();
        public Vector3 LastStuckPlace = Vector3.Zero;
        public RacerBaseBehavior BaseBehavior = RacerBaseBehavior.GridWait;
        public RaceState RCStatus = RaceState.NotInitiated;




        //Modifiers
        public float Confidence = 1f;

        public int oldNode = 0;

        public int Lap = 0;
        public int Pos = 1;

        //Timers
        int BrakeAdjustCooldown = 0; //1000ms
        int ManeuverTimeRef = 0; //500ms, actually GetRefrenceVehicle timer
        int AITimeRef = 0; //100ms
        public int SpeedControlTimer = 0; //200ms
        public int SteerControlTime = 0; //200ms
        int DebugTextTime = 0;
        int OneSecondGameTime = 0;
        float TRCurveAngle = 0f;
        Vector3 TRCurveVector = Vector3.Zero;
        Vector3 ClosestPointInTrack = Vector3.Zero;
        int tempSPDip = 0;


        //Status
        int StuckScore = 0;
        public bool StuckRecover = false;
        int UseGunScore = 100;
        int GameTimeUseGun = 0;

        //Perceived Info to make decisions
        public List<Vector3> Path = new List<Vector3>();
        public List<dynamic> TrackInfo = new List<dynamic>();
        List<Racer> referenceVehs = new List<Racer>();
        public float HandlingGrip = 0f;
        public float SurfaceGrip = 1f;
        float maxOutofBounds = 0f;
        public int CurrentNode = 0;
        public float SideSlide = 0f;
        float DeviationFromTrackCenter = 2f;

        bool CanRegisterNewLap = true;

        //Cornering variables (speed  and steer)

        //General
        public Vector3 CManeuverDirection = Vector3.Zero;
        float RacingLineTreshold = 0f;
        float cSlide = 0f;

        //Steer       
        float RacingLineDeviation = 0.0f;
        float OvertakeDeviation = 0.0f;
        float SteerToDeviation = 0f;
        float CurrentCornerAngle = 0f;
        Vector3 CurrentCornerDir = Vector3.Zero;
        Vector3 CurrentTrackDir = Vector3.Zero;
        Vector3 FollowTrackDir = Vector3.Zero;

        float zRotSpeed = 0f;

        //Speed
        public float CurrentBrake = 0f;
        float CurrentLockupLimiter = 1f;
        float CurrentWheelspinLimiter = 1f;

        public Vector3 TractionCurve = Vector3.Zero;
        public Vector3 TractionCurvePos = Vector3.Zero;
        float spdDiff = 0f;

        public List<dynamic> WorstCorners = new List<dynamic>();
        // public List<dynamic> WorstCorner = null; //1=pos, 2=angle;
        public float IdealSpeed = 0f;

        int littlebrakescore = 0;

        //Vehicle control
        public float CurrentThrottle = 1f;
        public float CurrentSteering = 20f;
        public bool CurrentHandBrake = false;
        public int HandbrakeTime = 0;
        float MaxSteeringAngle = 40f; //ScriptTest.map(Car.Velocity.Length(), 0, 60, 40, 5);
        int OutOfTrack = 0;
        float modelw = 0;




        public Racer(Vehicle RacerCar, Ped RacerPed)
        {
            
            Car = RacerCar;
            Driver = RacerPed;
            Name= RacerCar.FriendlyName;
            Path = ARS.Path;

            if (Driver.IsPlayer) ControlledByPlayer = true;
            SteerControlTime = Game.GameTime + (ARS.GetRandomInt(10, 50));
            SpeedControlTimer = Game.GameTime + (ARS.GetRandomInt(10, 50));
           // Function.Call(Hash.SET_ENTITY_MOTION_BLUR, Car, true);
            //Function.Call(Hash.SET_ENTITY_MOTION_BLUR, Driver, true);

            //Entity proofs
            //Function.Call(GTA.Native.Hash.SET_ENTITY_PROOFS, Car, false, false, false, false, false, false, false, false);

            handlingFlags = ARS.GetHandlingFlags(Car).ToString("X");
            modelFlags = ARS.GetModelFlags(Car).ToString("X");
            downf = ARS.GetDownforce(Car);
            if (downf == 0f) downf = 1f;
            trcurveLat = ARS.rad2deg( ARS.GetTRCurveLat(Car));
            if (!ControlledByPlayer)
            {

                Driver.BlockPermanentEvents = true;
                Driver.AlwaysKeepTask = true;
                Function.Call(GTA.Native.Hash.SET_DRIVER_ABILITY, Driver, 1f);
                Function.Call(GTA.Native.Hash.SET_DRIVER_AGGRESSIVENESS, Driver, 100f);



                if (ARS.AIRacerAutofix == 2)
                {

                    Function.Call(GTA.Native.Hash.SET_ENTITY_PROOFS, Car, true, true, true, true, true, true, true, true);

                    Car.IsInvincible = true;
                    Car.IsCollisionProof = true;
                    Car.IsOnlyDamagedByPlayer = true;
                    Function.Call(GTA.Native.Hash.SET_VEHICLE_STRONG, Car, true);
                    Function.Call(GTA.Native.Hash.SET_VEHICLE_HAS_STRONG_AXLES, Car, true);
                    Car.EngineCanDegrade = false;
                }
                else if (ARS.AIRacerAutofix == 1)
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
                if(Car.Model.IsCar)
                {
                 //   Driver.SetIntoVehicle(Car, VehicleSeat.Driver);
               //     Driver.Alpha = 0;
                }

                 // Driver.GiveHelmet(false, HelmetType.RegularMotorcycleHelmet, 0);
                /*
                TaskSequence race = new TaskSequence();
                Function.Call(Hash.TASK_PAUSE, 0, 100);
                race.Close(true);
                Driver.Task.PerformSequence(race);
                */
                //Function.Call(GTA.Native.Hash.SET_PED_COMBAT_ATTRIBUTES, Driver, 1, false);

                Car.IsRadioEnabled = false;
            }
            Car.IsPersistent = true;
            HandlingGrip =  ARS.GetTRCurveMax(Car);
 
            float modelw = Car.Model.GetDimensions().X / 2;

            



            //if (Name == "NULL" || Name==null) Name = Car.DisplayName.ToString()[0].ToString().ToUpper() + Car.DisplayName.ToString().Substring(1).ToLowerInvariant();
            /*
            List<string> namewords = Car.FriendlyName.Split(' ').ToList();

            for(int i =0; i<namewords.Count; i++)
            {
                if (i  < namewords.Count-1)
                {                    
                    if (namewords[i].Length > 3) Name += namewords[i].Substring(0, 3) + ". ";
                    else Name += namewords[i] + " ";
                }
                else
                {
                    if (namewords[i].Length > 6) Name += namewords[i].Substring(0, 6) + ". ";
                    else Name += namewords[i] + " ";
                }

            }
            */

            //     Name = Car.FriendlyName;
            
            if ((Car.CurrentBlip == null || Car.CurrentBlip.Exists() == false) && !Driver.IsPlayer)
            {
                Car.AddBlip();
                Car.CurrentBlip.Color = BlipColor.Blue;
                Car.CurrentBlip.Scale = 0.75f;
                Function.Call(Hash._SET_BLIP_SHOW_HEADING_INDICATOR, Car.CurrentBlip, true);
                Function.Call(Hash._0x2B6D467DAB714E8D, Car.CurrentBlip, true);
                //Function.Call(Hash.SET_BLIP_AS_SHORT_RANGE, Car.CurrentBlip, true);
                Car.CurrentBlip.Name = Name;
            }

            
            Function.Call(GTA.Native.Hash._0x0DC7CABAB1E9B67E, Car, true, 1);
            Function.Call(GTA.Native.Hash._0x0DC7CABAB1E9B67E, Driver, true, 1);

            //Driver.IsInvincible = true;
            Function.Call(GTA.Native.Hash.SET_ENTITY_PROOFS, Driver, true, true, true, false, true, true, 1, true);

            if (Car.Model.IsBike) Driver.CanFlyThroughWindscreen = true; else Driver.CanFlyThroughWindscreen = false;

            Driver.MaxHealth = 1000;
            Driver.Health = 1000;

        }



        public void Initialize()
        {
            IdealSteeringAngle = 0f;
            CurrentSteeringAngle = 0f;
            CurrentBrake = 0f;
            CurrentThrottle = 0f;
            LapTimes.Clear();
            StartTime = 0;
            BaseBehavior = RacerBaseBehavior.GridWait;
            CurrentNode = 0;
            Lap = 0;
            FinishedPointToPoint = false;
        }

        public void Steer()
        {

            if (BaseBehavior == RacerBaseBehavior.GridWait || BaseBehavior == RacerBaseBehavior.FinishedStandStill)
            {

                CurrentStr = 0f;
                return;
            }


            float rAngle = Vector3.SignedAngle(Car.Velocity.Normalized, FollowTrackDir, Vector3.WorldUp);

            if (fwspd < 0f)
            {
                float angCorrection = 0f;
                if (Math.Abs(DeviationFromTrackCenter) > maxOutofBounds) angCorrection = DeviationFromTrackCenter > 0 ? 20f : -20f;
                if (Math.Abs(Vector3.Angle(Car.ForwardVector, CurrentCornerDir)) > 90f)
                {
                    rAngle = Vector3.SignedAngle(Car.ForwardVector, FollowTrackDir, Vector3.WorldUp);
                    IdealSteeringAngle = (float)Math.Round(rAngle, 2) * -1f;
                    return;

                }
                else
                {
                    rAngle = Vector3.SignedAngle(Car.ForwardVector, FollowTrackDir, Vector3.WorldUp) + angCorrection;
                    IdealSteeringAngle = (float)Math.Round(rAngle, 2) * -0.5f;
                    // IdealSteeringAngle = AutosportRacingSystem.Clamp(IdealSteeringAngle, -20f, 20f);

                    return;
                }
            }




            //BackToTrack 
            float weOut = (Math.Abs(DeviationFromTrackCenter) + modelw + 2f) - maxOutofBounds;

            if (weOut > 0f)//(DeviationC>0 != CurrentCornerAngle>0)
            {
                if (CManeuverDirection.X == 0.0f || CManeuverDirection.X > 0 != DeviationFromTrackCenter > 0)
                {
                    float getInside = ARS.map(weOut, 0f, 20f, 0f, 40f);
                    getInside = ARS.Clamp(getInside, 0f, 40f);

                    if (DeviationFromTrackCenter < 0) getInside = -getInside;


                    if (getInside != 0.0f) rAngle -= getInside;
                }

            }
            // else RacingLineDeviation = 0.1f;




            //rAngle = AutosportRacingSystem.Clamp(rAngle, -50, 50);
            IdealSteeringAngle = (float)Math.Round(rAngle, 2);
            IdealSteeringAngle = ARS.Clamp(IdealSteeringAngle, -90f, 90f);



            //if (MaxSteeringAngle < Math.Abs(CurrentCornerAngle)) MaxSteeringAngle = Math.Abs(CurrentCornerAngle);



            //intended deviation

            if (ARS.RacingLine.ContainsKey(FollowNode))
            {
                RacingLineDeviation = -ARS.RacingLine[FollowNode];

            }
            else RacingLineDeviation = 0f;

            

            OvertakeDeviation = 0.0f;
            // RacingLineDeviation = 0.0f;

            if (OvertakeDeviation != 0.0f)
            {
                RacingLineTreshold = maxOutofBounds;
                float modelwide = Car.Model.GetDimensions().X;
                if (OvertakeDeviation > maxOutofBounds - modelwide) OvertakeDeviation = maxOutofBounds - modelwide;
                if (OvertakeDeviation < -maxOutofBounds + modelwide) OvertakeDeviation = -maxOutofBounds + modelwide;



                float diff = DeviationFromTrackCenter - OvertakeDeviation;
                float treshold = 0.5f; // Car.Model.GetDimensions().X * 0.2f;


                if (Math.Abs(diff) > treshold)
                {
                    if (diff < 0) diff += treshold; else diff -= treshold;

                    SteerToDeviation = 0f;
                    float toDev = ARS.GetSteerOffsetToReachDeviation(Math.Abs(diff), 4f, 4f);
                    diff = ARS.Clamp(diff, -4f, 4f);

                    if (diff > 0) toDev = -toDev;

                    SteerToDeviation = toDev;

                    IdealSteeringAngle += SteerToDeviation;
                }
                else if (ARS.IsStable(Car, 0.2f, 10f))
                {
                    OvertakeDeviation = 0.0f;
                }
            }
            else if (RacingLineDeviation != 0.0f)// && Math.Abs(CurrentCornerAngle) < 1f && Math.Abs(FollowTrackCornerAngle) < 1f
            {

                // if (RacingLineDeviation > maxOutofBounds- modelwide) RacingLineDeviation = maxOutofBounds - modelwide;
                // if(RacingLineDeviation<-maxOutofBounds+ modelwide) RacingLineDeviation = -maxOutofBounds+ modelwide;


                float diff = DeviationFromTrackCenter - RacingLineDeviation;
                bool canSteer = true;
                if(impedimentOnSide!=0)
                {
                    if (impedimentOnSide > 0.0f == diff > 0.0f) canSteer = false;
                }

                if (canSteer)
                {
                    float treshold = Car.Model.GetDimensions().X + 3f; // Car.Model.GetDimensions().X * 0.2f;
                    treshold = RacingLineTreshold;// + ScriptTest.map(Math.Abs(FollowTrackCornerAngle), 0f, 3f, 3f, 0f);//0.5f;

                    if (Math.Abs(diff) > treshold)
                    {

                        // if (diff < 0) diff += treshold; else diff -= treshold;

                        diff = ARS.Clamp(diff, -4f, 4f);
                        //IdealSpeed += 8 - (Math.Abs(diff)*2);
                        SteerToDeviation = 0f;
                        float toDev = ARS.GetSteerOffsetToReachDeviation(diff, 4f, 4f);// 0.7f

                        //if (diff > 0 == CurrentCornerAngle > 0) toDev /= 2;
                        //if (diff > 0) toDev = -toDev;


                        SteerToDeviation = toDev * RacingLineTreshold * ARS.map(Math.Abs(SideSlide), 0.1f, 0.0f, 0.0f, 1.0f, true) ;

                        //if (SteerToDeviation > 0 == CurrentCornerAngle > 0) SteerToDeviation *= 0.75f ;

                        //      UI.ShowSubtitle(SteerToDeviation.ToString(), 500);
                        // UI.ShowSubtitle(Math.Round(diff,2).ToString()+"~n~"+ Math.Round(SteerToDeviation,2).ToString(), 500);

                        IdealSteeringAngle -= SteerToDeviation;
                    }
                    else
                    {
                        RacingLineDeviation = 0.0f;
                    }

                }
            }







            //Apply Maneuver steering, don't if it would make you go out of the road
            if (CManeuverDirection.X != 0.0f)
            {
                if (ARS.RacingLine.ContainsKey(FollowNode)) RacingLineTreshold = 0f;
                if (Math.Abs(DeviationFromTrackCenter) > maxOutofBounds)
                {
                    if (DeviationFromTrackCenter > 0 != CManeuverDirection.X > 0) IdealSteeringAngle += CManeuverDirection.X;

                }
                else IdealSteeringAngle += CManeuverDirection.X;

            }

            if (!Car.Model.IsBicycle && !Car.Model.IsBike)
            {
                Vector3 dir = Car.Velocity.Normalized;
                dir.Z = 0f;
                Vector3 head = Car.ForwardVector;
                head.Z = 0f;


                zRotSpeed = ARS.rad2deg(Function.Call<Vector3>(Hash.GET_ENTITY_ROTATION_VELOCITY, Car).Z);

                if (Car.Velocity.Length() > 10f)
                {
                    //Slide
                    float cSlide = Vector3.SignedAngle(dir, head, Car.UpVector);
                    if (Math.Abs(cSlide) > 2) 
                    {
                        float d = Vector3.SignedAngle(Car.ForwardVector, Car.Velocity.Normalized, Vector3.WorldUp) * 1f;
                        float correction = (cSlide * 1.1f);
                        if (zRotSpeed > 0 != cSlide > 0 && Math.Abs(zRotSpeed) > 2f) correction = 0;                        
                        IdealSteeringAngle -= correction;                        
                    }



                    //Stability
                    //Maximum allowed rotspeed based on velocity
                    float reference = ARS.map(Car.Velocity.Length(), 10f, 30f, 35f, 30f, true);
                    if (Math.Abs(zRotSpeed) > reference)  // Math.Abs(TRCurveAngle)+20
                    {
                        float diff = Math.Abs(zRotSpeed) - (reference);
                        if (zRotSpeed < 0) diff = -diff;

                        float correct = ARS.map(Math.Abs(diff), 0f, 40f, 0f, 1f, true);
                        //correct = ARS.Clamp(correct, 0f, 1f);
                        IdealSteeringAngle += -(diff * correct);

                    }
                }

            }
        }
        bool WorstCornerFound()
        {

            if (WorstCorners.Count > 0) return true;
            return false;
        }
        float IdealSteeringAngle = 0f;
        float CurrentSteeringAngle = 0f;

        float BrakeCornerSpeed = 200f;

        float CurrentStr = 0f;
        Vector3 BrakingPointRef = Vector3.Zero;
        int understeerScore = 0;

        float fwspd = 0f;

        Vector3 SPTStart = Vector3.Zero;
        public void Launch()
        {
            BaseBehavior = RacerBaseBehavior.Race;
            StartTime = Game.GameTime;
            HandbrakeTime = 0;
        }

        void ApplyThrottle()
        {

            //Apply throttle after modifications
            float fwspd = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true).Y;

            if (BaseBehavior == RacerBaseBehavior.GridWait)
            {
                CurrentThrottle = 0.75f;
                HandbrakeTime = Game.GameTime + 500;
                return;
            }


            if (StuckRecover && BaseBehavior == RacerBaseBehavior.Race)
            {
                IdealSpeed = -1f;
            }
            if (understeerScore >= 3 && fwspd > 5f)
            {
                CurrentThrottle = 0f;
            }
            if (IdealSpeed > ARS.MaxIdealSpeed) IdealSpeed = ARS.MaxIdealSpeed;
            spdDiff = (float)Math.Round(IdealSpeed - Car.Speed, 1);



            CurrentThrottle = (float)Math.Round(ARS.map(spdDiff, -ARS.SpeedToInput, ARS.SpeedToInput, -1f, 1f), 2);
            CurrentThrottle = ARS.Clamp(CurrentThrottle, -1f, 1f);
            if (IdealSpeed > 0f && CurrentThrottle < 0f) CurrentThrottle = 0f;


            CurrentBrake = (float)Math.Round(ARS.map(spdDiff, 0f, -ARS.SpeedToInput, 0f, 1f), 2);
            CurrentBrake = ARS.Clamp(CurrentBrake, 0f, 1f);

            if (IdealSpeed < 0f)
            {
                if (CurrentThrottle < -0.5f) CurrentThrottle = -0.5f;
                if (fwspd > 10f) CurrentBrake = 1f; else CurrentBrake = 0f;
            }
            else
            {
                if (fwspd < -10f) CurrentBrake = 1f;
            }



            if (BaseBehavior == RacerBaseBehavior.FinishedRace || BaseBehavior == RacerBaseBehavior.FinishedStandStill)
            {
                if (BaseBehavior == RacerBaseBehavior.FinishedStandStill) CurrentThrottle = 0f;
                if (CurrentBrake > 0.2f) CurrentBrake = 0.2f;
            }
            if (CurrentBrake > CurrentLockupLimiter) CurrentBrake = CurrentLockupLimiter;
        }

        void ApplySteer()
        {
            float steerMax = 15f;
            if (understeerScore > 0) understeerScore--;

            if (TRCurveAngle > 0 != IdealSteeringAngle > 0 && Math.Abs(SideSlide) < 0.2f)
            {
                if (Math.Abs(TRCurveAngle) + steerMax < Math.Abs(IdealSteeringAngle))
                {
                    if (Math.Abs(IdealSteeringAngle) > 10f) understeerScore += 2;
                    while (Math.Abs(TRCurveAngle) + steerMax < Math.Abs(IdealSteeringAngle)) IdealSteeringAngle *= 0.8f;
                }
            }

            understeerScore = (int)ARS.Clamp(understeerScore, 0, 7);

            float AngleDif = IdealSteeringAngle - CurrentSteeringAngle;

            if (AngleDif / 1 != AngleDif || IdealSteeringAngle / 1 != IdealSteeringAngle || CurrentSteeringAngle / 1 != CurrentSteeringAngle)
            {
                IdealSteeringAngle = 0f;
                CurrentSteeringAngle = 0f;
                AngleDif = 0f;
            }
            float ttreshold = 12f; //3f
            if (Math.Abs(AngleDif) > 10f) ttreshold = 12f;
            if ((modelFlags.Length > 5 && modelFlags[5] == '8')) ttreshold = 100f;

            if (Math.Abs(AngleDif) < ttreshold)
            {
                CurrentSteeringAngle = IdealSteeringAngle;
            }
            else
            {
                if (CurrentSteeringAngle < IdealSteeringAngle) CurrentSteeringAngle += ttreshold; else CurrentSteeringAngle -= ttreshold;
            }

            CurrentSteeringAngle = ARS.Clamp(CurrentSteeringAngle, -MaxSteeringAngle, MaxSteeringAngle);

            //   if (Math.Abs(CurrentSteeringAngle) > MaxSteeringAngle + 1f) return;
            CurrentStr = ARS.GetRatioForAngle(Math.Abs(CurrentSteeringAngle), 1.4f); //1.2f

            if (CurrentStr / 1 != CurrentStr) CurrentStr = 0f;


            //Fix for double steering for monster trucks and tracked vehicles
            if ((handlingFlags.Length > 6 && handlingFlags[6] == '8') || (modelFlags.Length > 5 && modelFlags[5] == '8')) CurrentStr *= 0.5f;


            CurrentStr = ARS.Clamp(CurrentStr, -1f, 1f);

            if (CurrentSteeringAngle < 0f) CurrentStr = -CurrentStr;
        }

        float MapIdealSpeedForDistance(float distance, float speedThere)
        {
            float result= speedThere + ((mphStopInTenMeters / 10) * (distance));
            if (result < speedThere) return speedThere;
            else return result;

        }


        float cornerdip = -4;
        public void Speed()
        {

            float Currentspeed = Car.Velocity.Length();
            IdealSpeed = ARS.MaxIdealSpeed;


            if (BaseBehavior == RacerBaseBehavior.GridWait)
            {
                IdealSpeed = 200f;
                return;
            }
            if (BaseBehavior == RacerBaseBehavior.FinishedRace)
            {
                IdealSpeed = 20f;
                return;
            }
            if (BaseBehavior == RacerBaseBehavior.FinishedStandStill)
            {
                IdealSpeed = 0f;
                return;
            }

            BrakeCornerSpeed = GetBrakecornerSpeed();



            if (Math.Abs(Vector3.SignedAngle(Car.ForwardVector, CurrentCornerDir, Vector3.WorldUp)) > 120f)
            {
                IdealSpeed = -2f;
            }
            else
            {
                if (WorstCornerFound())
                {



                    float dist = Car.Position.DistanceTo(WorstCorners[0][1]);
                    dist += ARS.extradistancedip;
                    int dip = ARS.extracurvedip + tempSPDip;
                    //if (Math.Abs(CurrentCornerAngle) > 1) dip = 0;

                    IdealSpeed =  MapIdealSpeedForDistance(dist+ cornerdip, BrakeCornerSpeed + dip);



                }

                if (RacingLineDeviation != 0.0f)
                {
                    float diff = DeviationFromTrackCenter - RacingLineDeviation;
                     IdealSpeed += ARS.map(Math.Abs(diff), 2.0f,0.0f, 0.0f, ARS.MPHtoMS(3), true);
                }

                if (CManeuverDirection.Y != 0.0f)
                {
                    float maneuverspeed = Car.Velocity.Length() + (CManeuverDirection.Y);
                    if (maneuverspeed < IdealSpeed)
                    {
                        IdealSpeed = maneuverspeed;
                    }
                }

                

                //if (IdealSpeed < BrakeCornerSpeed - 25f) IdealSpeed = BrakeCornerSpeed - 25f;

            }
        }
        void HandleWheelIssues()
        {

            bool isOnWheelspin = false;
            bool isOnLockup = false;
            float skid = ARS.GetWheelsMaxWheelspin(Car);
            if (CurrentBrake > 0f)
            {
                if (!CurrentHandBrake)
                {


                    if (skid > 1.5f)
                    {
                        isOnLockup = true;
                    }

                }
            }
            if (!isOnLockup) CurrentLockupLimiter += 0.05f; else CurrentLockupLimiter -= 0.2f;
            CurrentLockupLimiter = ARS.Clamp(CurrentLockupLimiter, 0.01f, 1f);

            if (ARS.TCSLevel >= 1)
            {

                if (1 == 1)
                {

                    if (CurrentThrottle > 0f)
                    {
                        if (skid < 0f)
                        {

                            float correctedMaxwheelspin = ARS.map(Math.Abs(cSlide), 10, ARS.idealSld + 10, ARS.maxWheelspinAllowed, -0.2f);
                            correctedMaxwheelspin = ARS.Clamp(correctedMaxwheelspin, ARS.maxWheelspinAllowed, -0.2f);

                            correctedMaxwheelspin = ARS.maxWheelspinAllowed;


                            if (ARS.maxWheelspinAllowed < -5f) correctedMaxwheelspin = -100f; //actual max 15f
                            if (Car.Speed < 5f && correctedMaxwheelspin > -1f) correctedMaxwheelspin = -1f;

                            if (skid < correctedMaxwheelspin)
                            {
                                float change = Math.Abs(skid) * 0.2f;
                                change = ARS.Clamp(change, 0.02f, 0.1f);

                                CurrentWheelspinLimiter -= change; //0.05f
                                isOnWheelspin = true;
                            }
                        }
                    }
                }
                if (CurrentThrottle > 0f && CurrentWheelspinLimiter < CurrentThrottle) CurrentThrottle = CurrentWheelspinLimiter;

            }

            //Anti-Wheelspin 
            if (ARS.TCSLevel >= 2)
            {
                //Pros will prevent wheelspin only raising the throttle slowly, which will keep wheelspin to a minimum when it happens
                if (!isOnWheelspin)
                {
                    if (CurrentWheelspinLimiter <= CurrentThrottle) CurrentWheelspinLimiter += 0.175f;
                    else if (CurrentWheelspinLimiter > CurrentThrottle) CurrentWheelspinLimiter = CurrentThrottle;
                }
            }
            else if (ARS.TCSLevel >= 1)
            {
                //Amateurs will gain confidence right away, which won't prevent wheelspin
                if (!isOnWheelspin) CurrentWheelspinLimiter = 1f;
            }

            CurrentWheelspinLimiter = ARS.Clamp(CurrentWheelspinLimiter, 0.1f, 1f);

        }
        float GetBrakecornerSpeed()
        {

            //Calc brakedistance if there's a valid worst angle ahead
            if (!WorstCornerFound() || StuckRecover) return 0f;

            int node = WorstCorners[0][3];
            float downhill = 0f;
            float expectedGrip = 1f;
            if (ARS.MultiplierInTerrain.ContainsKey(node)) expectedGrip = ARS.MultiplierInTerrain[node];
            if (expectedGrip < 0.9f) expectedGrip *= SurfaceGrip;

            float worstAngle = Math.Abs(WorstCorners[0][0]);// + clampedlimit; // ScriptTest.GetFutureWorstCorner(Car, Path, info[0], Math.Abs(SteeringAngle), CurrentBrakeDist);

            float bspeed = ARS.GetSpeedForAngle(Math.Abs(worstAngle), Confidence * expectedGrip);// ScriptTest.map(Math.Abs(worstAngle), 0f, ScriptTest.max_angle, ScriptTest.max_speed, ScriptTest.min_speed); //; 


            bspeed = ARS.Clamp(bspeed, ARS.min_speed, ARS.max_speed);

            if (bspeed < ARS.min_speed * 0.75f) bspeed = ARS.min_speed * 0.75f;
            BrakeCornerSpeed = bspeed;
            float downfConfidence = 0f;
            if(BrakeCornerSpeed>30.0f) downfConfidence = ((BrakeCornerSpeed - ARS.MPHtoMS(30f))* 0.1f) * downf * (ARS.map(trcurveLat, 16f,24f, 0.0f, 1.0f, true));
            BrakeCornerSpeed += downfConfidence; // map(, 5f, 100f, 0f, 8f, true);

          //  AutosportRacingSystem.DisplayHelpTextTimed(Math.Round(downfConfidence, 1).ToString()+"mph",3000);
            
            return BrakeCornerSpeed;
        }

        int FastEntryScore = 0;
        public void ProcessTick()
        {
            //Draw
            DrawDebug();


            //Important variable updates
            SideSlide = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true).Normalized.X;
            fwspd = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true).Y;

            if (trail.Count > 50) trail.RemoveAt(0);
            if (TractionCurvePos.DistanceTo(Car.Position) > 1f)
            {
                if (TractionCurve == Vector3.Zero) TractionCurve = Car.Velocity.Normalized;
                else
                {
                    TractionCurve = ((TractionCurve.Normalized - Car.Velocity.Normalized)).Normalized;
                    TRCurveAngle = Vector3.SignedAngle(Car.Velocity.Normalized, -TractionCurve.Normalized, Vector3.WorldUp);
                }
                TractionCurvePos = Car.Position;
            }

            //AI updates
            if (AITimeRef < Game.GameTime)
            {
                AITimeRef = Game.GameTime + 100;
                if (BaseBehavior == RacerBaseBehavior.Race && ARS.Racers.Count >= 1) GetReferenceVehicle(); //refVehicle =
                ProcessAI();
            }


            if (!Driver.IsPlayer)
            {

                if (WorstCorners.Count > 0)
                {
                    if (WorstCorners[0][3] <= CurrentNode+3)
                    {
                        if ((spdDiff < -2))
                        {
                             if(spdDiff<-10)  HandbrakeTime = Game.GameTime + 500;
                            if (FastEntryScore > 3)
                            {

                                littlebrakescore = 0;
                                BrakeAdjustCooldown = Game.GameTime + 1000;
                                mphStopInTenMeters -= 0.2f;
                               // ARS.Log(ARS.LogImportance.FATAL, Name + " reduces his brake confidence to " + mphStopInTenMeters+".(-0.2)");
                                FastEntryScore = 0;

                            }
                            else
                            {
                                FastEntryScore++;
                            }
                        }
                         //if (FastEntryScore > 0) FastEntryScore--;
                        WorstCorners.RemoveAt(0);
                        if (Math.Abs(SideSlide) > 0.2f && 1==2)
                        {
                            if (mphStopInTenMeters > 1.5f)
                            {
                                ARS.Log(ARS.LogImportance.Fatal, Name + " reduces his brake confidence to " + mphStopInTenMeters + ".(-0.05) (sliding)");
                                mphStopInTenMeters -= 0.05f;
                                // UI.ShowSubtitle(Name + "is sliding while entering a corner. Deducing 0.02 to " + mphStopInTenMeters);

                            }

                        }
                        else if (FastEntryScore == 0 && WorstCorners.Count == 0)
                        {
                            mphStopInTenMeters += 0.05f;
                          //  ARS.Log(ARS.LogImportance.FATAL, Name + " increases his brake confidence to " + mphStopInTenMeters + ".(+0.1) (good entry)");

                        }
                    }
                }
                else if (FastEntryScore > 0) FastEntryScore = 0;





                //AI Inputs
                if (Driver.IsSittingInVehicle(Car) && !Driver.IsPlayer)
                {

                    /*
                    float Current = Car.SteeringAngle;
                    float Expected = CurrentSteeringAngle;


                    float d = 0f;
                    if (Current > 0 == Expected > 0) d = Math.Max(Current, Expected) - Math.Min(Current, Expected);
                    else if (Current > Expected) d = Current - Expected; else d = Expected - Current;
                    //UI.ShowSubtitle(Math.Round(d, 1).ToString());

                    if (d > 15f)
                    {
                        UI.ShowSubtitle("~r~" + Math.Round(d, 1).ToString());
                        Driver.Task.ClearAll();
                    }
                    */
                    if (HandbrakeTime > Game.GameTime) CurrentHandBrake = true; else CurrentHandBrake = false;
                    if (CurrentHandBrake) Car.HandbrakeOn = true; else Car.HandbrakeOn = false;

                   ARS.SetThrottle(Car, CurrentThrottle);
                   ARS.SetBrakes(Car, CurrentBrake);

                 

                    /*
                    if (Car.Speed > 10f && Math.Abs(d) > 15f && Driver.IsSittingInVehicle()) //Function.Call<bool>(Hash.GET_IS_TASK_ACTIVE, Driver, 462)
                    {

                        UI.ShowSubtitle(Name + " had a steering issue " + Current + " " + Expected, 1000);
                        //Driver.Task.ClearAll();
                        Driver.Task.ClearAllImmediately();
                    }
                    */
                    
                    ARS.SetTrueSteer(Car, CurrentStr);


                }
                else
                {
                    ARS.SetThrottle(Car, 0f);
                    ARS.SetBrakes(Car, 0f);
                    ARS.SetTrueSteer(Car, 0f);
                }

            }



        }

        void DrawDebug()
        {


            if (DebugText.Count > 0)
            {
                if (ARS.DebugLevel == (int)DebugMode.Interactions)
                {
                    string d = "~g~- INFO -~w~";
                    //d+=""
                    foreach (string st in DebugText) d += "~w~~n~" + st;
                    ARS.DrawText(Car.Position + new Vector3(0, 0, 2), d, System.Drawing.Color.White, 0.4f);
                }
            }

            if (!Car.IsInRangeOf(Game.Player.Character.Position, 100f)) return;

            if (trail.Count == 0) trail.Add(Car.Position);
            else if (Car.Position.DistanceTo(trail[trail.Count - 1]) > 2f) trail.Add(Car.Position);


           if(ARS.DebugLevel==(int)DebugMode.Interactions) foreach (Vehicle v in Traffic) ARS.DrawLine(Car.Position, v.Position, Color.Orange);// World.DrawMarker(MarkerType.ChevronUpx1, v.Position+new Vector3(0,0, v.Model.GetDimensions().Z), Vector3.Zero, Vector3.Zero, new Vector3(1f,1f, -1f), Color.Red, false, false, 0, false, "", "", false);



            if (ARS.DebugLevel == (int)DebugMode.Trails)
            {
                if (trail.Count > 0)
                {
                    foreach (Vector3 d in trail)
                    {

                        Color c = Color.Red;
                        if (Pos % 2 == 1) c = Color.Blue;
                        World.DrawMarker(MarkerType.DebugSphere, d, Vector3.Zero, Vector3.Zero, new Vector3(0.15f, 0.15f, 0.15f), c);

                    }
                }

            }

            /*
            if (DebugTextTime < Game.GameTime)
            {
                DebugTextTime = Game.GameTime + 2000;
                if (DebugText.Count > 0) DebugText.RemoveAt(0);
            }
            if (DebugText.Count == 0) DebugTextTime = Game.GameTime + 2000;
            */

            if (ARS.DebugLevel == (int)DebugMode.Route)
            {
                if (RacingLineDeviation != 0.0f)
                {
                    float d = DeviationFromTrackCenter - RacingLineDeviation;


                    World.DrawMarker(MarkerType.ChevronUpx1, Car.Position + (Car.RightVector * d), Vector3.Zero, new Vector3(90, Car.Heading, 0), new Vector3(0.8f, 0.8f, -3.8f), Color.Green, false, false, 0, false, "", "", false);


                    // ScriptTest.DrawLine(Car.Position + (Car.RightVector * d)+ Car.ForwardVector * -2f, Car.Position + (Car.RightVector * d) + Car.ForwardVector * 2f, Color.Green);
                }

                if (OvertakeDeviation != 0.0f)
                {
                    float d = DeviationFromTrackCenter - OvertakeDeviation;


                    World.DrawMarker(MarkerType.ChevronUpx1, Car.Position + (Car.RightVector * d), Vector3.Zero, new Vector3(90, Car.Heading, 0), new Vector3(0.8f, 0.8f, -3.8f), Color.Black, false, false, 0, false, "", "", false);


                }
            }
            if (ARS.DebugLevel == (int)DebugMode.Inputs)//&& Car.IsInRangeOf(Game.Player.Character.Position, 100f)
            {

                Color cc = Color.Green;
                if (CurrentBrake > 0.0f) cc = Color.Yellow;
                if (CurrentBrake > 0.5f) cc = Color.Orange;
                if (CurrentBrake > 0.9f) cc = Color.Red;

                if (ARS.DebugLevel > 0)
                {
                    Vector3 Dimensions = Car.Model.GetDimensions();
                    Vector3 inputplace = Car.Position + new Vector3(0, 0, (Dimensions.Z * 0.6f));
                    Vector3 steergoal = Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * CurrentSteeringAngle) * -Car.ForwardVector; // Quaternion.RotationAxis takes radian angles
                    Vector3 idealsteergoal = Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * IdealSteeringAngle) * -Car.ForwardVector; // Quaternion.RotationAxis takes radian angles

                    float dimension = Car.Model.GetDimensions().Y + 1f;//new Vector3(90, Car.Heading + CurrentSteeringAngle, 0)
                    World.DrawMarker(MarkerType.ChevronUpx1, Car.Position + (steergoal * -(Dimensions.Y / 2)) + new Vector3(0, 0, -Car.HeightAboveGround), steergoal, new Vector3(90, 0, 0), new Vector3(dimension / 4, dimension / 2, -(dimension / 2)), Color.SkyBlue, false, false, 0, false, "", "", false);

                    if (CurrentSteeringAngle != IdealSteeringAngle) World.DrawMarker(MarkerType.ChevronUpx1, Car.Position + (idealsteergoal * -(Dimensions.Y / 2)) + new Vector3(0, 0, -Car.HeightAboveGround), idealsteergoal, new Vector3(90, 0, 0), new Vector3(dimension / 4, dimension / 2, -(dimension / 2)), Color.FromArgb(50, Color.SkyBlue), false, false, 0, false, "", "", false);

                    World.DrawMarker(MarkerType.DebugSphere, inputplace + (Car.ForwardVector * ((Dimensions.Y / 2) * (-CurrentLockupLimiter))), Car.ForwardVector, new Vector3(90, 0, 0), new Vector3(0.05f, 0.05f, 0.05f), Color.Black, false, false, 0, false, "", "", false);
                    World.DrawMarker(MarkerType.DebugSphere, inputplace + (Car.ForwardVector * ((Dimensions.Y / 2) * (CurrentWheelspinLimiter))), Car.ForwardVector, new Vector3(90, 0, 0), new Vector3(0.05f, 0.05f, 0.05f), Color.Black, false, false, 0, false, "", "", false);

                    World.DrawMarker(MarkerType.ChevronUpx1, inputplace + (Car.ForwardVector * ((Dimensions.Y / 2) * (CurrentThrottle - CurrentBrake))), Car.ForwardVector, new Vector3(90, 0, 0), new Vector3(1f, 1f, 1f), cc, false, false, 0, false, "", "", false);
                    ARS.DrawLine(inputplace + (Car.ForwardVector * -(Dimensions.Y / 2)), inputplace + (Car.ForwardVector * (Dimensions.Y / 2)), Color.White);
                    ARS.DrawLine(inputplace + (Car.RightVector * 0.2f), inputplace - (Car.RightVector * 0.2f), Color.White);
                }
            }


            if (ARS.DebugLevel == (int)DebugMode.Interactions)
            {
                int i = 0;
                foreach (Racer r in referenceVehs)
                {
                    Vehicle refVehicle = r.Car;
                    if (ARS.CanWeUse(refVehicle))
                    {
                        if (i == 0) ARS.DrawLine(Car.Position + new Vector3(0, 0, Car.Model.GetDimensions().Z * 0.6f), refVehicle.Position, Color.Orange); else ARS.DrawLine(Car.Position + new Vector3(0, 0, Car.Model.GetDimensions().Z * 0.6f), refVehicle.Position, Color.SkyBlue);
                    }
                    i++;
                }
            }
          //  AutosportRacingSystem.DrawText(Car.Position + Vector3.WorldUp * (Car.Model.GetDimensions().Z + 0.25f), Math.Round(Car.Velocity.Length()).ToString(), Color.White, 0.5f);//mphStopInTenMeters.ToString()

            if (ARS.DebugLevel == (int)DebugMode.Cornering && WorstCornerFound())
            {
                ARS.DrawText(Car.Position + Vector3.WorldUp * (Car.Model.GetDimensions().Z + 0.25f), Math.Round(ARS.MStoMPH(Car.Velocity.Length())).ToString() + "mph", Color.White, 0.5f);//mphStopInTenMeters.ToString()


                if (Pos == 1)
                {
                    Vector3 front = Car.Position;
                    Vector3 back = Car.Position;

                    front += Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * cThreshold) * (CurrentCornerDir * 5); // Quaternion.RotationAxis takes radian angles
                    back += Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * cThreshold) * -(CurrentCornerDir * 5); // Quaternion.RotationAxis takes radian angles

                    World.DrawMarker(MarkerType.DebugSphere, front, Vector3.Zero, Vector3.Zero, new Vector3(0.05f, 0.05f, 0.05f), Color.Red);
                    World.DrawMarker(MarkerType.DebugSphere, back, Vector3.Zero, Vector3.Zero, new Vector3(0.05f, 0.05f, 0.05f), Color.Red);
                    ARS.DrawLine(back, front, Color.Red);

                    Vector3 ctocorner = Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * cToCorner) * (CurrentCornerDir * 5);
                    Vector3 ctocornerb = Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * cToCorner) * -(CurrentCornerDir * 5);

                    //Corner
                    ARS.DrawLine(Car.Position + ((CurrentCornerDir * 5)), Car.Position - ((CurrentCornerDir * 5)), Color.White);

                    //Difference

                    ARS.DrawLine(Car.Position + ((ctocorner)), Car.Position + ((CurrentCornerDir * 5)), Color.Orange);
                    //Real
                    ARS.DrawLine(Car.Position + ((ctocorner)), Car.Position + ((ctocornerb)), Color.Blue);

                }
                int d = 0;
                foreach (dynamic c in WorstCorners)
                {


                    d++;
                    Vector3 wp = c[1];

                    if (d < 50)
                    {
                        if (ARS.MultiplierInTerrain.ContainsKey(c[3]))
                        {
                          //  AutosportRacingSystem.DrawText(wp + new Vector3(0, 0, 0.4f), AutosportRacingSystem.MultiplierInTerrain[c[3]].ToString(), Color.Red, 0.5f);

                        }
                        ARS.DrawText(wp, c[0].ToString()+ "º~n~" + ARS.MStoMPH(c[5]).ToString()+"mph", Color.White, 0.15f); //*(c[5]+1f)
                    }

                    float expectedSpeed = MapIdealSpeedForDistance(wp.DistanceTo(Path[CurrentNode])+ cornerdip, BrakeCornerSpeed);
                    if (Car.Velocity.Length() < BrakeCornerSpeed)
                    {
                        ARS.DrawLine(Car.Position, wp, Color.Green);
                    }
                    else
                    {
                        if (Car.Velocity.Length() > expectedSpeed + (ARS.SpeedToInput) / 2)
                        {
                            if (Car.Velocity.Length() > expectedSpeed + ARS.SpeedToInput) ARS.DrawLine(Car.Position, wp, Color.Orange);
                            else ARS.DrawLine(Car.Position, wp, Color.Red);

                        }
                        else
                        {
                            ARS.DrawLine(Car.Position, wp, Color.Yellow);
                        }
                    }
                }


            }
        }

        float OutOfTrackDistance()
        {

            //(Math.Abs(DeviationFromTrackCenter) + modelw + 0.5f) - maxOutofBounds;
            return Math.Abs(DeviationFromTrackCenter) - maxOutofBounds;
        }
        int impedimentOnSide = 0;
        void VehicleInteractions()
        {
            DebugText.Clear();
            impedimentOnSide = 0;
            CManeuverDirection = Vector3.Zero;
            if (Car.Driver.IsPlayer || ARS.SettingsFile.GetValue<bool>("GENERAL_SETTINGS", "Ghosts", false)) return;
            Traffic.Clear();
            foreach(Vehicle refVehicle in Traffic)
            {
                Vector3 Offset = (ARS.GetOffset(Car, refVehicle));
                Vector3 OffsetfromHim = (ARS.GetOffset(refVehicle, Car));
                float sideOffset = ARS.LeftOrRight(Car.Position, refVehicle.Position, refVehicle.Velocity.Normalized);
                

                //if (Car.IsTouching(refVehicle) && Offset.Y > Car.Model.GetDimensions().Y*0.5f) ManeuverThrottle(-5f);

                //Vector3 OffsetToAxleY = Offset + new Vector3(0, -AxleY, 0);
                float DirDifference = Vector3.SignedAngle(Car.Velocity.Normalized, refVehicle.Velocity.Normalized, Car.UpVector);


                if (refVehicle.Velocity.Length() < 5f) DirDifference = 0f;



                float YBoundingBox = ((Car.Model.GetDimensions().Y / 2) + (refVehicle.Model.GetDimensions().Y / 2));
                float XBoundingBox = (Car.Model.GetDimensions().X / 2);

                float XDynamicBoundingBox = 0f;
                XDynamicBoundingBox = ARS.map(Vector3.Angle(Car.ForwardVector, refVehicle.ForwardVector), 0f, 90f, (refVehicle.Model.GetDimensions().X / 2), (refVehicle.Model.GetDimensions().Y / 2));

                XDynamicBoundingBox = ARS.Clamp(XDynamicBoundingBox, refVehicle.Model.GetDimensions().X / 2, refVehicle.Model.GetDimensions().Y / 2);
            //    XDynamicBoundingBox =+ AutosportRacingSystem.map(DirDifference, 0f, 45f, 0f, 2f, true);
                XDynamicBoundingBox += XBoundingBox;

                float SpeedDiff = refVehicle.Velocity.Length() - Car.Velocity.Length();// Positive=he is faster. Negative=he is slower;
                
                float sideDist = Math.Abs(Offset.X) - XDynamicBoundingBox;
                if (Offset.Y > 0)
                {
                    /*
                    //Avoid rearends via brake
                    if (Offset.Y > YBoundingBox && SpeedDiff < 0f)
                    {
                        float mappedspeed = MapIdealSpeedForDistance((Offset.Y - YBoundingBox), refVehicle.Velocity.Length());
                        float maneuver = mappedspeed - refVehicle.Velocity.Length();
                        float mult = AutosportRacingSystem.map(Math.Abs(DirDifference), 30f, 10f, 0f, 1f);
                        mult = AutosportRacingSystem.Clamp(mult, 0f, 1f);
                        mult=1;
                        if (refVehicle.Velocity.Length() < 10f) mult = 1;
                        maneuver *= mult;

                        if (maneuver < AutosportRacingSystem.SpeedToInput)
                        {
                            if (Car.Speed - maneuver < 15f) maneuver = 0.25f;
                            ManeuverThrottle(maneuver);
                        }
                    }*/


                    //Avoid rearends via steer
                    if (Offset.Y > YBoundingBox && Math.Abs(sideOffset) < XDynamicBoundingBox + 1f)
                    {
                        if (Math.Abs(DirDifference) < 30f)
                        {
                            float speedDistance = ARS.map(SpeedDiff, -10f, 5f, 50f, 0f);
                            speedDistance = ARS.Clamp(speedDistance, 0f, 50f);
                            if (speedDistance < Offset.Y) continue;
                        }
                        float distInside = XDynamicBoundingBox - Math.Abs(sideOffset);


                        float mult = 0f;
                        //Throttle
                        if (Math.Abs(DirDifference) < 40f && sideDist<-0.2f)
                        {
                            
                            float mappedspeed = MapIdealSpeedForDistance((Offset.Y - YBoundingBox), refVehicle.Velocity.Length());
                            float maneuver = mappedspeed - Car.Velocity.Length();
                            mult = ARS.mapGamma(Math.Abs((Offset.Y - YBoundingBox)), 0f, 50f, 1f, 0f, 0.25f);
                            mult = ARS.Clamp(mult, 0f, 1f);
                            if (refVehicle.Velocity.Length() < 10f) mult = 1;
                            maneuver *= mult;

                            if (maneuver < ARS.SpeedToInput)
                            {
                                if (Car.Velocity.Length() - maneuver < 15f) maneuver = 0.1f;
                                ManeuverThrottle(maneuver);
                            }

                        }


                        //Steer
                        float st = ARS.map(sideDist, 0.5f, -1.0f, .5f, 4.0f);
                        st = ARS.Clamp(st, .5f, 4.0f);
                        
                        if (Offset.X < 0) st = -st;
                        // if (Math.Abs(DirDifference) > 90f) st = -st;

                        ManeuverSteer(st);
                        /*
                        //Allows the AI to realize they cannot squeeze through the gap between their target
                        //And the track limits, so they slowly steer the other way, assuming there's a better opportunity over there
                        if (st > 0 == DeviationFromTrackCenter > 0 && OutOfTrackDistance() > -(XDynamicBoundingBox * 2))
                        {
                            //ManeuverThrottle(((SpeedDiff + 2f) * 0.5f));
                            ManeuverSteer(3 * (st > 0 ? -1 : 1));
                        }
                        else
                        {

                        }*/
                    }
                }

            }
            foreach (Racer r in referenceVehs)
            {
                Vehicle refVehicle = r.Car;
                //Interaction with other vehicles            
                if (!ARS.CanWeUse(refVehicle)) continue;

                if (ARS.CanWeUse(refVehicle))
                {
                    //if (Car.IsTouching(r.Car)) Car.Alpha = 0; 
                    float AxleY = (Car.Model.GetDimensions().Y * 0.25f);

                    Vector3 Offset = (ARS.GetOffset(Car, refVehicle));
                    Vector3 OffsetfromHim = (ARS.GetOffset(refVehicle, Car));
                    float sideOffset = ARS.LeftOrRight(Car.Position, refVehicle.Position, refVehicle.Velocity.Normalized);


                    if (Car.IsTouching(refVehicle) && Offset.Y>0) ManeuverThrottle(-0.1f);


                    //Vector3 OffsetToAxleY = Offset + new Vector3(0, -AxleY, 0);
                    float YBoundingBox = ((Car.Model.GetDimensions().Y / 2) + (refVehicle.Model.GetDimensions().Y / 2));
                    float XBoundingBox = (Car.Model.GetDimensions().X / 2);

                    float XDynamicBoundingBox = 0f;
                     XDynamicBoundingBox= ARS.map(Vector3.Angle(Car.ForwardVector, refVehicle.ForwardVector), 0f, 90f, (refVehicle.Model.GetDimensions().X / 2), (refVehicle.Model.GetDimensions().Y / 2));
                    
                    XDynamicBoundingBox = ARS.Clamp(XDynamicBoundingBox, refVehicle.Model.GetDimensions().X / 2, refVehicle.Model.GetDimensions().Y / 2);
                    XDynamicBoundingBox += XBoundingBox;

                    float SpeedDiff =(float)Math.Round( refVehicle.Velocity.Length() - Car.Velocity.Length(),1);// Positive=he is faster. Negative=he is slower;
                    float DirDifference = Vector3.SignedAngle(Car.Velocity.Normalized, refVehicle.Velocity.Normalized, Car.UpVector);
                    float sideDist = Math.Abs(Offset.X) - XDynamicBoundingBox;

                    //Side to side
                    if (Math.Abs(OffsetfromHim.Y) < YBoundingBox+0.5f)// && 
                    {
                         impedimentOnSide = (Offset.X > 0.0f ? 1 : -1); //For future reference. AI wont steer towards the racing line if there is someone on their side
                        if(OffsetfromHim.Y < 1f)
                        {

                            float directionDiff = (float)Math.Round(DirDifference, 3);
                            if (Offset.X > 0) directionDiff = -directionDiff;
                            float steer = directionDiff;
                            //if (steer < 3f) steer = 3f;
                            bool forcedOut = OutOfTrackDistance() > -((Car.Model.GetDimensions().X / 2) + 1f);
                            if (directionDiff < 0.0f)
                            {
                                float mult = ARS.map(sideDist, 2f, 5f, 1.25f, 0.25f, false);
                                if (mult > 1.0f) mult = 1f;
                                if (mult < 0.1f) mult = 0.1f;

                                steer *= mult;
                                steer = ARS.Clamp(steer, -5f, 5f);
                                if (Offset.X > 0) steer = -steer;

                                //Allows the AI to realize they're being squeezed, and thus they stop trying to keep their distance.
                                if (forcedOut)
                                {
                                    if (OffsetfromHim.X > 0 != DeviationFromTrackCenter > 0 && Math.Abs(sideDist) < (Math.Abs(directionDiff) / 2.5f) && Math.Abs(SpeedDiff) < 30f)
                                    {
                                        //If they're being squeezed hard enough, give way slowing down a bit.
                                        float man = ARS.map(Offset.Y, 0f, YBoundingBox, -(ARS.SpeedToInput*0.2f), 1f);
                                        man = ARS.Clamp(man, 0f, 1f);
                                        ManeuverThrottle(man);
                                        AddDebugText("~r~ |V| ");
                                    }
                                    else AddDebugText("~y~ | V | ");
                                }
                                else
                                {
                                    steer = (float)Math.Round(steer, 2);
                                    ManeuverSteer(steer);
                                    AddDebugText("~y~ \\ V / ");
                                }
                            }

                            if (sideDist < 1f && !forcedOut)
                            {
                                float avoid = (float)Math.Round((sideDist - 0.5f) * 2, 1);
                                if (avoid < 0.0f)
                                {
                                    AddDebugText("~b~ / V \\ ");
                                    ManeuverSteer(Math.Abs(avoid) * (Offset.X > 0.0f ? 1 : -1));
                                }
                            }
                        }
                    }

   

                    if (Offset.Y > 0)
                    {
                        //Avoid rearends via brake
                        if (Offset.Y > YBoundingBox && SpeedDiff < 0f && Math.Abs(Offset.X) < XBoundingBox+0.5f)
                        {
                            float mappedspeed = MapIdealSpeedForDistance((Offset.Y - YBoundingBox), refVehicle.Velocity.Length());
                            mappedspeed = ARS.Clamp(mappedspeed, refVehicle.Velocity.Length(), Car.Velocity.Length());
                            float maneuver = (mappedspeed - Car.Velocity.Length())*0.4f;
                            float mult = ARS.map(Math.Abs(DirDifference), 10f, 30f,1f, 0f);
                            mult = ARS.Clamp(mult, 0f, 1f);

                            if (refVehicle.Velocity.Length() < 10f) mult = 1;
                           maneuver *= mult;

                            if (maneuver < ARS.SpeedToInput)
                            {
                               // if (Car.Speed - maneuver < 15f) maneuver = 0.25f;
                                ManeuverThrottle(maneuver);
                                //AutosportRacingSystem.DisplayHelpText(Math.Round(maneuver,1).ToString());
                            }
                        }


                        //Avoid rearends via steer
                        if (Offset.Y > YBoundingBox +0.5f && Math.Abs(sideOffset) < XDynamicBoundingBox + 1.5f )
                        {


                            float st = ARS.map(XDynamicBoundingBox+1f - Math.Abs(sideOffset), 0.0f, 4.0f, 0.25f, 3.0f, true);

                            float distModifier = ARS.map(Offset.Y-YBoundingBox, 1f, 10f, 1f, 0f);
                            distModifier = ARS.Clamp(distModifier, 0.0f, 1.0f);

                            st *= distModifier;
                            

                            if (sideOffset > 0) st = -st;

                           
                            //Allows the AI to realize they cannot squeeze through the gap between their target
                            //And the track limits, so they slowly steer the other way, assuming there's a better opportunity over there

                            if (impedimentOnSide == 0||impedimentOnSide>0.0f != st>0.0f)
                            {
                                if (st > 0 == r.DeviationFromTrackCenter > 0 && r.OutOfTrackDistance() > -(XDynamicBoundingBox * 2))
                                {
                                    ManeuverThrottle(3f);
                                    ManeuverSteer(3 * (st > 0 ? -1 : 1));
                                }
                                else
                                {
                                    ManeuverSteer(st);
                                }
                            }

                        }
                    }
                }
            }
        }
        bool WouldMoveOutOfTrack(float maneuver)
        {
            //if we're already close to the outside and the maneuver turns us further there
            if (Math.Abs(DeviationFromTrackCenter) + Car.Model.GetDimensions().X > maxOutofBounds && maneuver > 0 == DeviationFromTrackCenter > 0) return true;

            return false;
        }
        void ManeuverSteer(float x)
        {
            if (Math.Abs(CManeuverDirection.X) < Math.Abs(x)) CManeuverDirection.X = x;
        }
        void ManeuverThrottle(float y)
        {

            if (y < CManeuverDirection.Y || CManeuverDirection.Y < ARS.SpeedToInput) CManeuverDirection.Y = y;
        }

        float FollowTrackCornerAngle = 0f;
        public bool FinishedPointToPoint = false;
        public int localSPDLimiter = 0;
        public int FollowNode = 0;

        public float LowestReasonableSpeed = 0f;
        public void GetTrackInfo()
        {
            int followdist = (int)ARS.map(Car.Velocity.Length(), 0f, ARS.MPHtoMS(70f), 4, 8, true);// (int) Car.Velocity.Length();// 10
            //followdist = (int)ARS.Clamp(followdist, 3, 7);
            
            Vector3 carpos = Car.Position;
            if (CurrentNode >= Path.Count - 1)
            {
                if (ARS.IsPointToPoint)
                {
                    CurrentNode = Path.Count - 1;
                }
                else CurrentNode = 0;
            }
            if (CurrentNode <= 0)
            {
                if (ARS.IsPointToPoint) CurrentNode = 0;
                else CurrentNode = Path.Count - 1;
            }


            int selectedNode = CurrentNode - 2;
            if (selectedNode < 0)
            {
                if (ARS.IsPointToPoint) selectedNode = 0;
                else selectedNode = Path.Count - 1;
            }

            Vector3 selectedNodePos = Path[selectedNode];
            int rtries = 0;


            int point = CurrentNode - 2;
            if (point < 0)
            {
                if (ARS.IsPointToPoint) point = 0;
                else point = Path.Count - 1;
            }
            if (!FinishedPointToPoint)
            {

                //int min = (int)AutosportRacingSystem.Clamp(CurrentNode - 10, 0, Path.Count - 1);

                //List<Vector3> localp = Path.GetRange(min,20);
                //selectedNode = localp.IndexOf(localp.OrderBy(v => v.DistanceTo(Car.Position)).ToList()[0]);
                //selectedNodePos = Path[selectedNode];

                while (rtries < 20)
                {
                    point++;
                    rtries++;


                    if (point >= Path.Count - 1) if (FinishedPointToPoint) point = Path.Count - 5; else point = 0;

                    if (Path[point].DistanceTo(carpos) < Path[selectedNode].DistanceTo(carpos))
                    {
                        selectedNode = point;
                        selectedNodePos = Path[selectedNode];
                    }
                }
            }
            else
            {
                selectedNode = Path.Count - 3;
                selectedNodePos = Path[selectedNode];

            }

            if (selectedNodePos != Vector3.Zero)
            {
                CurrentNode = selectedNode;
                ClosestPointInTrack = selectedNodePos;


                if (FinishedPointToPoint)
                {
                    CurrentTrackDir = (Path[Path.Count - 1] - Path[Path.Count - 2]).Normalized;
                    CurrentCornerDir = CurrentTrackDir;
                    CurrentCornerAngle = 0f;
                    DeviationFromTrackCenter = 0f;

                    FollowTrackDir = (Path[Path.Count - 1] - Path[Path.Count - 2]).Normalized;
                    FollowTrackCornerAngle = 0f;
                }
                else
                {
                    int fRnode = selectedNode + 1;
                    if (fRnode > Path.Count - 1) fRnode = 0;
                    CurrentTrackDir = (Path[fRnode] - Path[selectedNode]).Normalized;


                    int ffRnode = selectedNode + 2;
                    if (ffRnode > Path.Count - 1) ffRnode = ffRnode - (Path.Count - 1);
                    CurrentCornerDir = ((Path[ffRnode] - Path[fRnode])).Normalized;//- CurrentTrackDir


                    float d = ARS.LeftOrRight(carpos, Path[fRnode], (Path[fRnode] - Path[selectedNode]));
                    float dev = carpos.DistanceTo(ClosestPointInTrack);



                    float cangle = Vector3.SignedAngle(CurrentTrackDir.Normalized, CurrentCornerDir.Normalized, Vector3.WorldUp);
                    CurrentCornerAngle = (float)Math.Round(cangle, 2);


                    if (d > 0) dev *= -1;
                    DeviationFromTrackCenter = dev;


                    rtries = 0;

                    int followNode = CurrentNode + followdist;
                    if (followNode < 0) followNode = 0;
                    if (followNode >= Path.Count - 3) followNode = 1;// followNode-(Path.Count-1);

                    Vector3 fdir = Path[followNode];

                    FollowTrackDir = (Path[followNode + 1] - fdir).Normalized;
                    Vector3 fFollowTrackDir = (Path[followNode + 2] - Path[followNode + 1]).Normalized;

                    FollowTrackCornerAngle = (float)Math.Round(Vector3.SignedAngle(FollowTrackDir.Normalized, fFollowTrackDir.Normalized, Vector3.WorldUp), 2);
                    FollowNode = followNode;

                }


            }
            if (ARS.WideDict.ContainsKey(CurrentNode))
            {
                maxOutofBounds = ARS.WideDict[CurrentNode];
            }
            else
            {
                for (int i = 0; i < Path.Count - 1; i++)
                {
                    if (ARS.WideDict.ContainsKey(i))
                    {
                        maxOutofBounds = ARS.WideDict[i];
                        //  break;
                    }
                }
            }
        }
        public void AddDebugText(string s)
        {
            //s += "~w~";
            s = "~w~" + s + "~w~";
            if (!DebugText.Contains(s)) DebugText.Add(s);
        }

        public void ProcessAI() //100ms
        {
            if (RacingLineDeviation != 0.0f) RacingLineTreshold += 0.1f; else RacingLineTreshold = 0f;
            if (RacingLineTreshold > 1.0f) RacingLineTreshold = 1.0f;
            if (RacingLineTreshold < 0.0f) RacingLineTreshold = 0.0f;

            if (BaseBehavior == RacerBaseBehavior.GridWait && HandbrakeTime < Game.GameTime) HandbrakeTime = Game.GameTime + (100 * ARS.GetRandomInt(2, 6));



            if (Math.Abs(DeviationFromTrackCenter) > maxOutofBounds && !ControlledByPlayer && BaseBehavior == RacerBaseBehavior.Race)
            {

                if (1 == 1)//BaseBehavior == RacerBaseBehavior.Race
                {
                    if (OutOfTrack == 0) OutOfTrack = Game.GameTime;
                    else if (Game.GameTime - OutOfTrack > 13000)
                    {
                        OutOfTrack = 0;
                        Car.Position = Path[CurrentNode];

                        Car.Heading = CurrentTrackDir.ToHeading();
                        StuckScore = 0;
                        StuckRecover = false;
                        LastStuckPlace = Vector3.Zero;
                        Car.Speed = 20f;
                    }
                }
            }
            else
            {
                if (OutOfTrack != 0) OutOfTrack = 0;
            }


            if (mphStopInTenMeters < 2f) mphStopInTenMeters = 2f;
            if (mphStopInTenMeters > 5f) mphStopInTenMeters = 5f;
            if (ARS.SettingsFile.GetValue<bool>("GENERAL_SETTINGS", "Ghosts", false))
            {

                Car.Alpha = 255;
                foreach (Racer r in referenceVehs)
                {
                    Function.Call(Hash.SET_ENTITY_NO_COLLISION_ENTITY, Car, r.Car, false);
                    if (Function.Call<bool>(Hash.IS_ENTITY_AT_ENTITY, Car, r.Car, r.Car.Model.GetDimensions().X, r.Car.Model.GetDimensions().Y, r.Car.Model.GetDimensions().Z, true, true, true)) Car.Alpha = 150;

                }

            }


            VehicleInteractions();
            if(!Driver.IsPlayer) if (referenceVehs.Count > 0) Driver.Task.LookAt(referenceVehs[0].Driver, 2000); else if (Car.Velocity.Length() > 5f) Driver.Task.LookAt(Car.Position + Car.Velocity, 2000);

            if (WorstCorners.Count == 0)
            {
                if (BaseBehavior == RacerBaseBehavior.Race && Vector3.Angle(Car.ForwardVector, CurrentCornerDir) < 5f && Math.Abs(FollowTrackCornerAngle) < 0.5f && Math.Abs(SideSlide) < 0.1f && Math.Abs(CurrentStr) < 0.2f) Function.Call((Hash)0x81E1552E35DC3839, Car, true);

            }

            if (Function.Call<bool>((Hash)0x3D34E80EED4AE3BE, Car) && (CurrentThrottle < 0.5f || CurrentBrake > 0.1f || WorstCorners.Count != 0)) Function.Call((Hash)0x81E1552E35DC3839, Car, false);
            if (!WorstCornerFound())
            {
                IdealBrakeDist = 150f;
            }



            //Mid-Corner Traction Curve calcs

            if (Car.Velocity.Length() > 10f)
            {
                HandleOvershoot();
            }



            //Slow checks
            if (OneSecondGameTime < Game.GameTime)
            {
                if (tempSPDip > 0) tempSPDip -= 1;

                OneSecondGameTime = Game.GameTime + (1000 + (ARS.GetRandomInt(-10, 10) * 10));

                if (!ControlledByPlayer)
                {

                    if (!Driver.IsSittingInVehicle(Car) && Car.IsStopped && Driver.IsStopped)
                    {
                        if (Driver.TaskSequenceProgress == -1)
                        {
                            //Driver.Task.EnterVehicle(Car, VehicleSeat.Driver, 500);


                            TaskSequence enter = new TaskSequence();
                            Function.Call(Hash.TASK_ENTER_VEHICLE, 0, Car, 6000, -1, 2f, 0, 0);
                            enter.Close();
                            Driver.Task.PerformSequence(enter);

                            return;
                        }



                    }
                }

                if (ARS.AIRacerAutofix == 2 && Function.Call<bool>(Hash._IS_VEHICLE_DAMAGED, Car))
                {
                    if(ARS.DebugLevel>0) UI.Notify("~b~"+Name + " auto repaired");
                    Car.Repair();
                }

                //Stuck
                if (BaseBehavior == RacerBaseBehavior.Race && Driver.IsSittingInVehicle(Car))
                {
                    if (Car.Velocity.Length() < 1f && CurrentThrottle != 0.0f && CurrentBrake == 0 && !CurrentHandBrake) StuckScore++; else StuckScore--;
                    StuckScore = (int)ARS.Clamp(StuckScore, 0, 10);

                    if (StuckScore >= 2)
                    {

                        if (StuckScore >= 5)//5
                        {
                            if (Driver.IsSittingInVehicle(Car) && Car.EngineHealth > 100)
                            {
                                Car.Rotation = Vector3.WorldUp;
                                Car.Position = Path[CurrentNode];
                                Car.Heading = CurrentCornerDir.ToHeading();
                                Car.Speed = 20f;
                            }
                        }
                        if (!StuckRecover && !Car.Model.IsBike)
                        {
                            LastStuckPlace = Car.Position;
                            if (ARS.DebugLevel > 0) UI.Notify("~b~" + Car.FriendlyName + " tries to recover");
                            StuckRecover = true;
                        }
                    }
                    if (StuckRecover && !Car.IsInRangeOf(LastStuckPlace, 5f))
                    {
                        StuckRecover = false;
                        StuckScore = 0;
                    }
                }

            }


            if (ManeuverTimeRef < Game.GameTime)
            {
                ManeuverTimeRef = Game.GameTime + 500;// (500 + (ScriptTest.GetRandomInt(-20, 20) * 10))

                
                ARS.WorstCornerAhead(this, Path, CurrentNode, CurrentCornerAngle,150);
                if (WorstCorners.Count > 0)
                {
                  //  WorstCorners = WorstCorners.OrderBy(v => Math.Abs(v[3])).ToList();

                    int index = WorstCorners.IndexOf(WorstCorners.OrderBy(v => MapIdealSpeedForDistance(v[3] - CurrentNode + cornerdip, v[5])).ToList()[0]);

                    //AutosportRacingSystem.Log(AutosportRacingSystem.LogImportance.Info, "[Corners] Index: " + index);


                   if(WorstCorners.Count>1) WorstCorners.RemoveRange(0, index);
                    //if(WorstCorners.Count-1<index) WorstCorners.RemoveRange(index,WorstCorners.Count-2);
                    while (WorstCorners.Count > index + 10) WorstCorners.RemoveAt(WorstCorners.Count - 1);


                }

                //WorstCorners = WorstCorners.OrderBy(v => MapIdealSpeedForDistance(v[3]-CurrentNode,v[5])).ToList();

                //float worst = MapIdealSpeedForDistance(WorstCorners[0][3] - CurrentNode, WorstCorners[0][5]);

                /*
                float spd = IdealSpeed;
                
                int lasti = 0;
                for (int i = 0; i < WorstCorners.Count; i++)
                {
                    if (i >= WorstCorners.Count) break;
                    float d = (float) Math.Round(Car.Position.DistanceTo(WorstCorners[i][1]), 1);
                    float newSpd = MapIdealSpeedForDistance(d, AutosportRacingSystem.GetSpeedForAngle(WorstCorners[i][0], Confidence));//WorstCorners[i][3]-CurrentNode
                    if (newSpd < spd && i > 0)
                    {
                        WorstCorners.RemoveAt(lasti);
                        AutosportRacingSystem.Log(AutosportRacingSystem.LogImportance.Info, "Pruned corner ("+i+" " + newSpd + " < "+ lasti+" "+ spd + ") "+ d.ToString());
                    }
                    spd = newSpd;
                    lasti = i;
                }
                */



                //if(WorstCorners.Count>10)  WorstCorners.RemoveRange(1, WorstCorners.Count - 10);
                //   for (int i = 0; i < 10; i++) if (WorstCorners.Count > 1 && WorstCorners[0][0] < WorstCorners[1][0]) WorstCorners.RemoveAt(0);





                //WorstCorners = WorstCorners.OrderBy(v => Math.Abs(v[0])).Reverse().ToList();
                //  WorstCorners = WorstCorners.OrderBy(v => Math.Abs(v[3])).ToList();


            }


            if (SteerControlTime < Game.GameTime)
            {
                SteerControlTime = Game.GameTime + 100;
                oldNode = CurrentNode;

                CurrentSteering = ARS.GetTrueSteer(Car); // Car.SteeringAngle;

                GetTrackInfo();
                if (BaseBehavior == RacerBaseBehavior.Race && CurrentNode > 20 && !CanRegisterNewLap) CanRegisterNewLap = true;

                if (FinishedPointToPoint)
                {
                    if (BaseBehavior == RacerBaseBehavior.Race)
                    {

                        LapTimes.Add(Game.GameTime);
                        Lap = 900;
                        CanRegisterNewLap = false;

                    }
                }
                else
                {

                    if (oldNode != CurrentNode && CurrentNode < oldNode - 100 || (CurrentNode >= Path.Count - 6 && ARS.IsPointToPoint))
                    {
                        CanRegisterNewLap = false;
                        Lap++;
                        if (Lap > ARS.SettingsFile.GetValue("GENERAL_SETTINGS","Laps", 5))
                        {
                            if (Car.CurrentBlip != null) Car.CurrentBlip.Color = BlipColor.Green;

                        }

                        if (Lap > 1) LapTimes.Add(Game.GameTime);

                    }
                }


                if (!ControlledByPlayer)
                {
                    Steer();
                    ApplySteer();
                }

            }


            if (SpeedControlTimer < Game.GameTime)
            {
                SpeedControlTimer = Game.GameTime + 200;
                HandleGrip();
                //  UI.ShowSubtitle(mphStopInTenMeters.ToString());

                if (!WorstCornerFound()) littlebrakescore = 0;
                //   spdDiff = IdealSpeed - Car.Velocity.Length();

                if (ARS.IsStable(Car, 0.2f, 50f) && BrakeAdjustCooldown < Game.GameTime && WorstCornerFound())// && CManeuverDirection.Y==0.0f && CurrentThrottle<0.75f&& Math.Abs(CurrentCornerAngle) <1
                {
                    if (spdDiff < -20)
                    {
                        if (littlebrakescore > 0) littlebrakescore--;
                    }
                    else if (spdDiff > -5 && spdDiff < 5)
                    {
                        littlebrakescore++;
                    }






                }



                if (!ControlledByPlayer)
                {

                    switch (BaseBehavior)
                    {
                        default:
                            {
                                Speed();
                                ApplyThrottle();
                                HandleWheelIssues();
                                break;
                            }


                    }
                }




            }
        }
        string handlingFlags = "";
        string modelFlags = "";
      public  float downf = 0f;
      public  float trcurveLat = 0f;
        void HandleGrip()
        {

            float avg = 0f;
            foreach (float gr in ARS.GetWheelsGrip(Car))
            {
                avg += gr;
            }
            avg /= ARS.GetWheelsGrip(Car).Count;

            float wetavg = 0f;
            foreach (float gr in ARS.GetWheelsWetgrip(Car))
            {
                wetavg += gr;
            }
            wetavg /= ARS.GetWheelsWetgrip(Car).Count;

            wetavg = (float)Math.Round(wetavg, 1);

            SurfaceGrip = (float)Math.Round(avg, 1);

            HandlingGrip = (float)Math.Round(ARS.GetTRCurveMax(Car), 1);
            HandlingGrip = ARS.Clamp(HandlingGrip, 0.4f, 3f);
            
            int nWheels = ARS.GetNumWheels(Car);
            float penalization = 0f;
            for (int i = -1; i < 6; i++)
            {

                if (Function.Call<bool>(Hash.IS_VEHICLE_TYRE_BURST, Car, i, true))
                {

                    float penalizationPercent = (float)Math.Round((1f / (float)nWheels) * 100, 2);
                    penalization += ((float)HandlingGrip / (float)100) * penalizationPercent;

                }
            }
            HandlingGrip -= penalization;
            if(HandlingGrip<0.1f) HandlingGrip=0.1f;


            HandlingGrip *= (SurfaceGrip * wetavg);



            Confidence = 1f;
            Confidence = ARS.map(HandlingGrip, 0.2f, 3f, 0.2f, 3f);
            Confidence = ARS.Clamp(Confidence, 0.2f, 3f);
            //Confidence += (Car.Velocity.Length() * 0.02f) * downf;// AutosportRacingSystem.map(Car.Velocity.Length(), 30f, 100f, 0, (0.2f * downf), true);

            if (Pos < 0 && !ARS.MultiplierInTerrain.ContainsKey(CurrentNode))
            {
                ARS.MultiplierInTerrain.Add(CurrentNode, SurfaceGrip);
            }



        }


        float cThreshold = 0f;
        float cToCorner = 0f;
        void HandleOvershoot()
        {
            cThreshold = 0f;
            Vector3 c = CurrentCornerDir;
            c.Z = 0f;
            Vector3 f = FollowTrackDir;
            f.Z = 0f;
            float angle = Vector3.SignedAngle(c, f, Vector3.WorldUp);

            float CarToCornerAngle = Vector3.SignedAngle(Car.Velocity.Normalized, CurrentCornerDir, Vector3.WorldUp);
            float CarHeadingToCornerAngle = Vector3.SignedAngle(Car.ForwardVector, CurrentCornerDir, Vector3.WorldUp);

            cToCorner = Math.Abs(CarToCornerAngle) * (angle < 0.0f ? 1 : -1); ;
            //Help AI not throttle too much if they would overshoot in a straight
            if (1==1)
            {
                float outOfBounds = maxOutofBounds;
                if (angle < 0.0f) outOfBounds = -outOfBounds;
                //if (CarToCornerAngle > 0) outOfBounds = -outOfBounds;
                float myDeviation = (float)Math.Round(DeviationFromTrackCenter,1);
                //myDeviation += AutosportRacingSystem.map(Math.Abs(CurrentCornerAngle), 0f, 1f, -1f, 1f);//Stricter if in a corner

                //float outTrack = myDeviation + outOfBounds;
                int Safety = (int)ARS.map(myDeviation + outOfBounds, 0f, outOfBounds *2, 0f, 100f, false);

                float threshold = (float)Math.Round(ARS.map(Safety, 35, 100, 0, 20, true));
                cThreshold = threshold * (angle < 0.0f ? 1 : -1);
                // AutosportRacingSystem.DisplayHelpText(Safety+"% Safe~n~Max"+threshold+"º");

                if (Safety < 0.0f)
                {
                    float expectedSpeed = ARS.GetSpeedForAngle(Math.Abs(CurrentCornerAngle), Confidence);
                    expectedSpeed += ARS.map(Safety, -10, 0, -24, 0, true);
                    float m =   Car.Velocity.Length()+ expectedSpeed;
                    ManeuverThrottle(m);

                }

                if (CarToCornerAngle > 0 == CurrentCornerAngle > 0 && Math.Abs(CurrentCornerAngle)>0.5) //Math.Abs(TRCurveAngle) < Math.Abs(CurrentCornerAngle) * 2f &&
                {

                  

                    float input = (float)Math.Round(ARS.map(Math.Abs(CarToCornerAngle), threshold, threshold + 10f, 1f, -1f), 1);//maxOutAng

                    input = ARS.Clamp(input, -1f, 1f);


                    float maneuver = ARS.SpeedToInput * input;
                    float nextspeed = Car.Velocity.Length()  +maneuver;

                    if (maneuver < ARS.SpeedToInput)
                    {
                        float minspeed = ARS.GetSpeedForAngle(Math.Abs(CurrentCornerAngle), Confidence) +ARS.map(Safety, -10, 10, -24, 0, true);// IdealSpeed - 5f;
                       // AutosportRacingSystem.DisplayHelpText(minspeed.ToString());
                     //   if (minspeed > Car.Velocity.Length()) minspeed = AutosportRacingSystem.map(myDeviation, maxOutofBounds, maxOutofBounds + 4, 25f, 5f);
                      //  if (minspeed < 5f) minspeed = 5f;

                        if (nextspeed < minspeed) maneuver = minspeed - Car.Velocity.Length();
                        ManeuverThrottle(maneuver);
                    //    if (Pos == 1) AutosportRacingSystem.DisplayHelpTextTimed("Current: " + Math.Round(Math.Abs(CarToCornerAngle), 1) + "º~n~Treshold:" + treshold + "º~n~Input " + input, 500);

                    }


                }
                
                /*
                else 
                
                if (CurrentThrottle > 0.0f)
                {
                    float max = CarToCornerAngle / 2;
                    if (max > 4) max = 4;

                    if (CurrentCornerAngle > 0) max = -max;
                }
                */

            }

            //if (CurrentCornerAngle < 0f) CarHeadingToCornerAngle = -CarHeadingToCornerAngle;


        }
        public float mphStopInTenMeters =2.5f;
        public float IdealBrakeDist = 150f; //default, changes on the function below


        public List<Vehicle> Traffic = new List<Vehicle>();
        public void GetReferenceVehicle()
        {
            if(ARS.DevSettingsFile.GetValue<bool>("TRACK", "Traffic", false)) {
                Traffic.Clear();
                Traffic = ARS.GlobalTraffic.Where(s => s.Position.DistanceTo(Car.Position) < 100f
                && (ARS.GetOffset(Car, s).Y > 0f)
                && Math.Abs(ARS.GetOffset(Car, s).X) < 10f
                && Math.Abs(ARS.GetOffset(Car, s).Z) < 5f).ToList();
                Traffic.OrderBy(v => v.Position.DistanceTo(Car.Position));
                if (Traffic.Count > 4) Traffic.RemoveRange(3, Traffic.Count - 4);

            }

            //List<Vehicle> racers= AutosportRacingSystem.Racers.Where(s => s.Car)).Tolist()
            /*
            RaycastResult cast = World.RaycastCapsule(Car.Position, CurrentTrackDir, 100f, (modelw*2f)+4f, (IntersectOptions)2+4+8, Car);//(Car.Velocity.Normalized)
            if (cast.DitHitEntity && cast.HitEntity.Model.IsVehicle)
            {
                bool isTr = true;
                foreach (Racer r in AutosportRacingSystem.Racers) if(r.Car == cast.HitEntity as Vehicle) { isTr = false; break; }
                if (isTr)
                {

                    UI.Notify("Added traffic");
                    Traffic.Add(cast.HitEntity as Vehicle);
                }
            }*/

            Vector3 refpoint = Car.Position + (Car.Velocity.Normalized * 2f);
            referenceVehs.Clear();
            List<Racer> Candidates = new List<Racer>();
            foreach (Racer r in ARS.Racers)
            {
                if (r.Car.Handle != Car.Handle && r.Car.Position.DistanceTo(refpoint) < 200f)
                {
                    float dist = r.Car.Position.DistanceTo(Car.Position);
                    if (dist > 10f)
                    {
                        float spdDiff = r.Car.Velocity.Length() - Car.Velocity.Length(); //Positive=target is faster
                        float maxDiff = (float)Math.Round(ARS.map(dist, 20f, 10f, 0.0f, 5.0f), 1);

                        
                        if (ARS.GetOffset(Car, r.Car).Y > 0f  && spdDiff > maxDiff) continue;
                        if (ARS.GetOffset(Car, r.Car).Y < 0f && spdDiff < -maxDiff) continue;
                    }
                    Candidates.Add(r);
                }
            }

            Candidates = Candidates.OrderBy(v => Vector3.Distance(v.Car.Position, refpoint)).ToList();
            int max = 3;
            while (Candidates.Count > max) Candidates.RemoveAt(Candidates.Count - 1);
            referenceVehs = Candidates;
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
