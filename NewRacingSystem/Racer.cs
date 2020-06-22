using GTA;
using GTA.Math;
using GTA.Native;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ARS
{


    public enum RacerBaseBehavior
    {
        GridWait, Race, DamagedAvoid, FinishedRace, FinishedStandStill
    }
    public class Racer
    {

        public VehicleControl vControl = new VehicleControl();
        public Memory mem = new Memory();

        public bool ControlledByPlayer = false;

        public Dictionary<Decision, int> Decisions = new Dictionary<Decision, int>();
        public Dictionary<Decision, int> BannedDecisions = new Dictionary<Decision, int>();

        public Dictionary<Mistake, int> Mistakes = new Dictionary<Mistake, int>();
        public Dictionary<Mistake, int> MistakesCooldown = new Dictionary<Mistake, int>();



        //Racer Info
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



        float CatchUpTorqueMult = 1.0f;


        //Modifiers
        public float Confidence = 1f;

        public int oldNode = 0;

        public int Lap = 0;
        public int Pos = 0;

        //Timers
        int HalfSecondCheck = 0; //1000ms
        int ProcessAITimeRef = 0; //100ms
        int OneSecondGameTime = 0;
        float TRCurveAngle = 0f;
        Vector3 TRCurveVector = Vector3.Zero;
        Vector3 ClosestPointInTrack = Vector3.Zero;


        //Status
        int StuckScore = 0;
        public bool StuckRecover = false;

        //Perceived Info to make decisions

        public List<dynamic> TrackInfo = new List<dynamic>();
        List<Racer> referenceVehs = new List<Racer>();
        public float HandlingGrip = 0f;
        public float SurfaceGrip = 1f;
        public float SideSlide = 0f;

        public bool CanRegisterNewLap = true;

        //Cornering variables (speed  and steer)

        //General
        float RacingLineTreshold = 0f;
        float SlideAngle = 0f;

        //Steer       
        float RacingLineDeviation = 0.0f;
        float OvertakeDeviation = 0.0f;
        float SteerToDeviation = 0f;

        float zRotSpeed = 0f;

        //Speed
        //public float vControl.Brake = 0f;
        float CurrentLockupLimiter = 1f;
        float CurrentWheelspinLimiter = 1f;

        public Vector3 TractionCurve = Vector3.Zero;
        public Vector3 TractionCurvePos = Vector3.Zero;
        float spdDiff = 0f;

        //public List<dynamic> WorstCorners = new List<dynamic>();
        // public List<dynamic> WorstCorner = null; //1=pos, 2=angle;




        //Vehicle control


        public bool CurrentHandBrake = false;
        public int HandbrakeTime = 0;
        float MaxSteeringAngle = 40f; //ScriptTest.map(Car.Velocity.Length(), 0, 60, 40, 5);
        int OutOfTrack = 0;

        //Avoidance info, how 'wide' the vehicle is
        //accounting for direction and sideways slide angle
        public float BoundingBox = 0f;

        float AngleToTrack = 0f;

        public List<CornerPoint> KnownCorners = new List<CornerPoint>();

        public Racer(Vehicle RacerCar, Ped RacerPed)
        {
            //mem.personality = ARS.personalities.Last();

            //if (ARS.DevSettingsFile.GetValue<bool>("RACERS", "UsePersonalities", false)) mem.personality = ARS.personalities[ARS.GetRandomInt(0, ARS.personalities.Count )]; else mem.personality = ARS.personalities.Last();

            AngleToTrack = 50f;
            Car = RacerCar;
            Driver = RacerPed;
            Name = RacerCar.FriendlyName;
            if (Name == "NULL" || Name == null) Name = Car.DisplayName.ToString()[0].ToString().ToUpper() + Car.DisplayName.ToString().Substring(1).ToLowerInvariant();


            if (Driver.IsPlayer) ControlledByPlayer = true;
            //SteerControlTime = Game.GameTime + (ARS.GetRandomInt(10, 50));
            HalfSecondCheck = Game.GameTime + (ARS.GetRandomInt(10, 50));
            // Function.Call(Hash.SET_ENTITY_MOTION_BLUR, Car, true);
            //Function.Call(Hash.SET_ENTITY_MOTION_BLUR, Driver, true);

            //Entity proofs
            //Function.Call(GTA.Native.Hash.SET_ENTITY_PROOFS, Car, false, false, false, false, false, false, false, false);

            handlingFlags = ARS.GetHandlingFlags(Car).ToString("X");
            modelFlags = ARS.GetModelFlags(Car).ToString("X");
            downf = ARS.GetDownforce(Car);
            if (downf == 0f) downf = 1f;
            //if (downf > 4f) downf = 4f;
            TRCurveLateral = ARS.rad2deg(ARS.GetTRCurveLat(Car));
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
                if (Car.Model.IsCar)
                {

                    //Driver.Alpha = 0;
                }

                Driver.SetIntoVehicle(Car, VehicleSeat.Driver);
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
            HandlingGrip = ARS.GetTRCurveMax(Car);

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

                // Function.Call(Hash._0xBFEFE3321A3F5015, Driver, Name, false,false, "Racer", 0);

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

            Car.Repair();

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

            mem.data.brakeSafety = ARS.map(Function.Call<float>((Hash)0xA132FB5370554DB0, Car), 2f, 1f, 0.5f, 1f, true);
        }


        float maxSteerSmooth = 50f;
        public void SteerLogic()
        {
            mem.intention.DeviationFromCenter = 0f;
            if (BaseBehavior == RacerBaseBehavior.GridWait || BaseBehavior == RacerBaseBehavior.FinishedStandStill)
            {
                vControl.SteerAngle = 0f;
                return;
            }

            //Unmodified angle to steer at, to follow the track.
            Vector3 carVel = Car.Velocity.Normalized;
            Vector3 track = followTrackPoint.Direction;

            float rAngle = Vector3.SignedAngle(Car.Velocity.Normalized, followTrackPoint.Direction, Vector3.WorldUp);


            //Steering behavior if we are going backwards.
            if (mem.data.SpeedVector.Y < 0f)
            {
                float angCorrection = 0f;
                if (Math.Abs(mem.data.DeviationFromCenter) > trackPoint.TrackWide) angCorrection = mem.data.DeviationFromCenter < 0 ? 20f : -20f;
                if (Math.Abs(AngleToTrack) > 90f)
                {
                    rAngle = Vector3.SignedAngle(Car.ForwardVector, followTrackPoint.Direction, Vector3.WorldUp);
                    vControl.SteerAngle = (float)Math.Round(rAngle, 2) * -1f;
                    return;

                }
                else
                {
                    rAngle = Vector3.SignedAngle(Car.ForwardVector, followTrackPoint.Direction, Vector3.WorldUp) + angCorrection;
                    vControl.SteerAngle = (float)Math.Round(rAngle, 2) * -0.5f;
                    return;
                }
            }


            //Approach the intended racing line deviation
            if ((!mem.intention.Maneuvers.Any() || mem.intention.Maneuvers.OrderBy(v => v.X).First().X < 0.1f))
            {

                if (ARS.CornerPoints.Any() && trackPoint != null)
                {
                    CornerPoint cornerPoint = ARS.CornerPoints[trackPoint.Node];
                    bool ApproachingCorner = false; 
                    if (KnownCorners.Any()) //Get to the outside to approach the corner properly
                    {
                        int dist = KnownCorners.First().Node - (trackPoint.Node+GetBrakePointInvalidateDist());
                        if (dist >= 30)
                        {
                            ApproachingCorner = true;

                            float Wide = followTrackPoint.TrackWide - BoundingBox - 2f;
                            float Outside = (Wide * (KnownCorners.First().Angle > 0 ? -1 : 1))*-1;
                            float lane = 0f;
                            float maxSteer = ARS.map(Math.Abs(cornerPoint.Angle), 3f, 1f, 0f, 5f, true) * ARS.map(ARS.MStoMPH(Car.Velocity.Length() - BrakeCornerSpeed), 0, ARS.AIData.SpeedToInput * 2, 0, 1, true);

                            if(Outside >0 != mem.data.DeviationFromCenter>0f && maxSteer>0.0f)
                            {
                                Vector2 maneuver = new Vector2(GetSteerToDeviation(lane, 0f, 1f, maxSteer), ARS.AIData.SpeedToInput);
                                if (maneuver.X != 0f)
                                {
                                    mem.intention.Maneuvers.Add(maneuver);
                                    mem.intention.DeviationFromCenter = lane;
                                }
                            }
                        }
                    }
                    if (!ApproachingCorner) //Get to the inside to take corner properly
                    {
                        float Wide = followTrackPoint.TrackWide - BoundingBox - 2f;
                        float Inside = Wide * (cornerPoint.Angle > 0 ? -1 : 1);
                        float lane = Inside;
                        float maxSteer = ARS.map(Math.Abs(cornerPoint.Angle), 0f, 2f, 0f, 3f, true);
                        lane = ARS.Clamp(lane, -Wide, Wide);

                        Vector2 maneuver = new Vector2(GetSteerToDeviation(lane, 0f, 3f, maxSteer), ARS.AIData.SpeedToInput);
                        if (maneuver.X != 0f)
                        {
                            mem.intention.Maneuvers.Add(maneuver);
                            mem.intention.DeviationFromCenter = lane;
                        }
                    }
                }                
            }

            //Out-Of-Track steering modifiers  
            float weOut = OutOfTrackDistance();
            if (weOut > 0f)
            {
                //!mem.intention.Maneuvers.Any()
                if (1 == 1)
                {
                    float max = ARS.map(ARS.MStoMPH(Car.Velocity.Length()), 50f, 10f, 10f, 50f, true);
                    float getInside = ARS.map(weOut, 0f, 5f, 0f, max, true) * (mem.data.DeviationFromCenter > 0 ? 1f : -1f);
                    if (getInside != 0.0f) mem.intention.Maneuvers.Add(new Vector2(getInside, ARS.AIData.SpeedToInput));
                }
            }

            //Center ourselves to avoid being on the side of the track
            if (!mem.intention.Maneuvers.Any() && trackPoint.TrackWide - (BoundingBox + Math.Abs(mem.data.DeviationFromCenter)) < 2f)
            {
                float centermyself = 1f * (mem.data.DeviationFromCenter > 0 ? 1f : -1f);
                mem.intention.Maneuvers.Add(new Vector2(centermyself, ARS.AIData.SpeedToInput));
            }

            vControl.SteerAngle = (float)Math.Round(rAngle, 2);
            vControl.SteerAngle = ARS.Clamp(vControl.SteerAngle, -90f, 90f);

            /*
            if (vControl.SteerAngle > 0f != zRotSpeed > 0f)
            {
                vControl.SteerAngle = ARS.Clamp(vControl.SteerAngle, Math.Abs(SlideAngle)-10f, Math.Abs(SlideAngle)+10f);
            }
            */


            if (!Car.Model.IsBicycle && !Car.Model.IsBike)
            {



                zRotSpeed = ARS.rad2deg(Function.Call<Vector3>(Hash.GET_ENTITY_ROTATION_VELOCITY, Car).Z);

                if (Car.Velocity.Length() > 10f)
                {
                    //Slide                    
                    if (Math.Abs(SlideAngle) > 0.0f)
                    {



                        float correction = (float)Math.Round(SlideAngle * ARS.map(Math.Abs(SlideAngle), mem.personality.Stability.CountersteerMinAngleSideways, mem.personality.Stability.CountersteerMaxAngleSideways, 0f, mem.personality.Stability.CountersteerSideways, true), 1);
                        if (zRotSpeed > 0 != SlideAngle > 0 && Math.Abs(zRotSpeed) > 0f) correction *= 0.25f;

                        float overC = mem.personality.Stability.CountersteerMaxOvercorrect;
                        if (Mistakes.ContainsKey(Mistake.SlideOvercorrect)) { overC = 90f; correction *= 2f; }
                        else if (Mistakes.ContainsKey(Mistake.SpinoutUndercorrect)) correction *= 0.25f;
                        correction = ARS.Clamp(correction, -Math.Abs(SlideAngle) - overC, Math.Abs(SlideAngle) + overC); //The overcorrection allows the driver to recover from the slide
                        vControl.SteerAngle -= correction;
                    }



                    //Stability
                    //Maximum allowed rotspeed based on velocity
                    float reference = Math.Abs(TRCurveAngle * 10f) + mem.personality.Stability.SpinoutSafeRotSpeed;
                    if (Math.Abs(zRotSpeed) > reference)
                    {
                        float diff = Math.Abs(zRotSpeed) - (reference);
                        if (zRotSpeed < 0) diff = -diff;



                        if (ARS.DevSettingsFile.GetValue<bool>("RACERS", "AllowMistakes", false) && zRotSpeed > 0 == SlideAngle > 0 && Math.Abs(zRotSpeed) > 0f && Math.Abs(diff) > mem.personality.Stability.SpinoutMaxExtraRotSpeed * 0.75f)
                        {
                            if (Math.Abs(diff) < mem.personality.Stability.SpinoutMaxExtraRotSpeed) MakeMistake(Mistake.SpinoutUndercorrect, 25, 750, 5000, 2000);
                            if (Math.Abs(diff) >= mem.personality.Stability.SpinoutMaxExtraRotSpeed) MakeMistake(Mistake.SlideOvercorrect, 25, 750, 5000, 2000);
                        }


                        float scaleMod = 1f;
                        if (Mistakes.ContainsKey(Mistake.SpinoutUndercorrect)) scaleMod = 0f;

                        float correct = ARS.map(Math.Abs(diff), 0f, mem.personality.Stability.SpinoutMaxExtraRotSpeed, 0f, mem.personality.Stability.SpinoutCounterScale * scaleMod, true);
                        vControl.SteerAngle += -(diff * correct);                        
                    }
                }
            }
            foreach (Vector2 m in mem.intention.Maneuvers)
            {
                vControl.SteerAngle += m.X;
            }
        }

        //In meters. Negative = outside the track, on the outside of the current corner.
        float DistToOutside()
        {
            if (trackPoint.Angle < 0.0f) return (float)Math.Round(trackPoint.TrackWide + mem.data.DeviationFromCenter, 1);
            else return (float)Math.Round(trackPoint.TrackWide - mem.data.DeviationFromCenter, 1);
        }

        float DistToOutsideCarToCorner()
        {
            float carTocorner = Vector3.SignedAngle(Car.Velocity.Normalized, trackPoint.Direction, Car.UpVector);

            if (carTocorner < 0.0f) return (float)Math.Round(trackPoint.TrackWide + mem.data.DeviationFromCenter, 1);
            else return (float)Math.Round(trackPoint.TrackWide - mem.data.DeviationFromCenter, 1);
        }

        float GetSteerToDeviation(float d, float noSteerDist = 1f, float smoothSteerDist = 5f, float maxSteer = 5f)
        {
            float r = (float)Math.Round(mem.data.DeviationFromCenter - d, 2);

            float str = (float)Math.Round(ARS.map(Math.Abs(r), noSteerDist, smoothSteerDist, 0f, maxSteer, true), 1) * (r > 0 ? 1 : -1);

            return str;
        }

        public float BrakeCornerSpeed = 200f;

        float CurrentStr = 0f;
        Vector3 BrakingPointRef = Vector3.Zero;


        Vector3 SPTStart = Vector3.Zero;
        public void Launch()
        {
            StuckRecover = false;
            StuckScore = 0;

            BaseBehavior = RacerBaseBehavior.Race;
            LapStartTime = Game.GameTime;
            HandbrakeTime = 0;
            CurrentWheelspinLimiter = 1f;
        }


        float theoreticalThrottle = 0f;
        void ApplyThrottle()
        {

            //Apply throttle after modifications
            float fwspd = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true).Y;

            if (BaseBehavior == RacerBaseBehavior.GridWait)
            {
                vControl.Throttle = 0.75f;
                HandbrakeTime = Game.GameTime + 500;
                return;
            }


            if (StuckRecover && BaseBehavior == RacerBaseBehavior.Race)
            {
                mem.intention.Speed = -4f;
            }

            if (mem.intention.Speed > ARS.AIData.MaxSpeed) mem.intention.Speed = ARS.AIData.MaxSpeed;

            spdDiff = (float)Math.Round(mem.intention.Speed - Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true).Y, 1);
            //spdDiff -= ARS.SpeedToInput;

            vControl.Throttle = (float)Math.Round(ARS.map(spdDiff, -ARS.AIData.SpeedToInput, ARS.AIData.SpeedToInput, -1f, 1f, true), 2);

            if (mem.intention.Speed > 0.0f)
            {

                vControl.Throttle = ARS.Clamp(vControl.Throttle, 0.001f, 1f);
            }
            //if (IdealSpeed > 0f && vControl.Throttle < 0f) vControl.Throttle = 0f;


            vControl.Brake = (float)Math.Round(ARS.map(spdDiff, 0f, -ARS.AIData.SpeedToInput, 0f, 1f), 2);
            vControl.Brake = ARS.Clamp(vControl.Brake, 0f, 1f);




            float slideAngle = ARS.map(Math.Abs(mem.data.SpeedVector.Normalized.X), 0f, 1f, 0f, 90f, true);
            float maxBrake = ARS.map(slideAngle, 90f, 25f, 0f, 1f, true);
            vControl.Brake = ARS.Clamp(vControl.Brake, 0f, maxBrake);



            if (mem.intention.Speed < 0f)
            {
                if (vControl.Throttle < -0.5f) vControl.Throttle = -0.5f;
                if (fwspd > 10f) vControl.Brake = 1f; else vControl.Brake = 0f;
            }
            else
            {
                if (fwspd < -1f) vControl.Brake = 1f;
            }



            if (BaseBehavior == RacerBaseBehavior.FinishedRace || BaseBehavior == RacerBaseBehavior.FinishedStandStill)
            {
                if (BaseBehavior == RacerBaseBehavior.FinishedStandStill) vControl.Throttle = 0f;
                if (vControl.Brake > 0.2f) vControl.Brake = 0.2f;
            }
            if (vControl.Brake > CurrentLockupLimiter) vControl.Brake = CurrentLockupLimiter;








            theoreticalThrottle = vControl.Throttle;

        }

        float maxStrTRCurve = 0f;
        void ApplySteer()
        {


            if (mem.data.SpeedVector.Y > 0)
            {

                /*
                maxStrTRCurve = Math.Abs(TRCurveAngle * 20f);
                if (float.IsNaN(maxStrTRCurve)) maxStrTRCurve = 0f;
                maxStrTRCurve += 0.5f;
                 */

                //if (Math.Abs(vControl.SteerAngle) > maxStrTRCurve) maxStrTRCurve += 0.5f; else if(maxStrTRCurve>1f) maxStrTRCurve -= 1f;
                maxStrTRCurve = (TRCurveLateral * 0.6f) + ARS.map(ARS.MStoMPH(Car.Velocity.Length()), 50f, 0f, 0f, TRCurveLateral * 0.4f, true);

                if (mem.data.SpeedVector.X > 0f == vControl.SteerAngle > 0f)
                {
                    vControl.SteerAngle = ARS.Clamp(vControl.SteerAngle, -maxStrTRCurve, maxStrTRCurve);
                }


            }




            if (float.IsNaN(vControl.SteerAngle)) vControl.SteerAngle = 0f;
            if (float.IsNaN(mem.data.SteerAngle)) mem.data.SteerAngle = 0f;

            float AngleDif = (float)Math.Round(vControl.SteerAngle - mem.data.SteerAngle, 2);
            if (float.IsNaN(AngleDif)) AngleDif = 0f;



            float aimSpeedThreshold = 15f;

            if (Math.Abs(AngleDif) < aimSpeedThreshold)
            {
                mem.data.SteerAngle = vControl.SteerAngle;
            }
            else
            {
                if (mem.data.SteerAngle < vControl.SteerAngle) mem.data.SteerAngle += aimSpeedThreshold; else mem.data.SteerAngle -= aimSpeedThreshold;
            }

            mem.data.SteerAngle = ARS.Clamp(mem.data.SteerAngle, -MaxSteeringAngle, MaxSteeringAngle);
            CurrentStr = ARS.GetRatioForAngle(Math.Abs(mem.data.SteerAngle), 1.3f); //1.2f


            if (float.IsNaN(CurrentStr)) CurrentStr = 0f;


            //Fix for double steering for monster trucks and tracked vehicles
            // if ((handlingFlags.Length > 6 && handlingFlags[6] == '8') || (modelFlags.Length > 5 && modelFlags[5] == '8')) CurrentStr *= 0.5f;


            CurrentStr = ARS.Clamp(CurrentStr, -1f, 1f) * (mem.data.SteerAngle > 0f ? 1 : -1);
            //if (mem.data.SteeringAngle < 0f) CurrentStr = -CurrentStr;
        }

        float MapIdealSpeedForDistance(float distance, float speedThere, float gamma = 1)
        {

            float maxSpd = (float)Math.Round((Function.Call<float>(Hash._0x53AF99BAA671CA47, Car) + (ARS.MPHtoMS(50f))));

            float spd = (float)Math.Round(ARS.mapGamma(distance, 0f, 250f, speedThere, maxSpd, gamma, true), 1);
            //ARS.DisplayHelpTextTimed("Max: " + Math.Round(ARS.MStoMPH(maxSpd)).ToString() +"~n~~b~Tgt: "+ Math.Round(ARS.MStoMPH(spd)).ToString(), 1000);
            return spd;

        }

        public void SpeedLogic()
        {
            float Currentspeed = Car.Velocity.Length();
            mem.intention.Speed = ARS.AIData.MaxSpeed;



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
                KnownCorners = KnownCorners.OrderBy(v => MapIdealSpeedForDistance(v.Node - trackPoint.Node, v.Speed)).ToList();
                KnownCorners.RemoveAll(v => v.Node < KnownCorners.First().Node);
                //if (KnownCorners.Count() > 100) KnownCorners = KnownCorners.Take(100).ToList();
            }


            if (KnownCorners.Any())
            {
                if (mem.intention.NeedAggressionChange)
                {
                    mem.intention.NeedAggressionChange = false;
                    mem.intention.Aggression = (ARS.GetRandomInt((int)(mem.intention.AggroToReach * 25), (int)(mem.intention.AggroToReach * 100)) * 0.01f);
                }
            }
            else if (!mem.intention.NeedAggressionChange)
            {
                mem.intention.NeedAggressionChange = true;
            }


            //Try and take the corner faster if it opens in the future
            if (KnownCorners.Any())
            {
                //Original speed
                BrakeCornerSpeed = GetBrakecornerSpeed(KnownCorners.First().Node, KnownCorners.First().Angle);

                //Future point reference
                int fNode = KnownCorners.First().Node + (int)(BrakeCornerSpeed * 0.25);
                if (ARS.CornerPoints.Count() > fNode)
                {
                    float p = ARS.GetPercent(Math.Abs(ARS.CornerPoints[fNode].Angle), Math.Abs(KnownCorners.First().Angle)); //Percentage of openness
                    float bonusSpeed = ARS.map(p, 100, 0, 0, ARS.MPHtoMS(10), true) * ARS.map(ARS.MStoMPH(BrakeCornerSpeed), 50, 100, 0, 1, true); //additional speed scaled to openness and by the original speed
                    BrakeCornerSpeed += bonusSpeed;
                }

                /*
                BrakeCornerSpeed = 0f;
                int nodesToCheck= (int)GetBrakecornerSpeed(KnownCorners.First().Node, KnownCorners.First().Angle)/2; 
                if (KnownCorners.Count >= nodesToCheck) { foreach (CornerPoint c in KnownCorners.Take(nodesToCheck)) BrakeCornerSpeed += c.Speed; BrakeCornerSpeed /= nodesToCheck; }
                else { foreach (CornerPoint c in KnownCorners) BrakeCornerSpeed += c.Speed; BrakeCornerSpeed /= KnownCorners.Count; }                
                */
            }
            else BrakeCornerSpeed = ARS.AngleToSpeed.First().Value;


            if (KnownCorners.Any() && ARS.DevSettingsFile.GetValue<bool>("RACERS", "AllowManeuvers", false) && KnownCorners.First().Node - trackPoint.Node < 75 && referenceVehs.Any() && Pos > 1)
            {
                if (BrakeCornerSpeed < Car.Velocity.Length() * 0.5)
                {
                    if (!BannedDecisions.ContainsKey(Decision.LateBrake)) MakeDecision(Decision.LateBrake, (int)(mem.intention.AggroToReach * 100), 1000, 4000, 5000);
                }
                else
                {
                    if (!BannedDecisions.ContainsKey(Decision.FastCorner)) MakeDecision(Decision.FastCorner, (int)(mem.intention.AggroToReach * 100), 4000, 6000, 10000);
                }
            }

            if (Math.Abs(Vector3.SignedAngle(Car.ForwardVector, trackPoint.Direction, Vector3.WorldUp)) > 70f)
            {
                mem.intention.Speed = -5f;
                return;
            }
            else
            {
                if (KnownCorners.Any())
                {

                    float dist = KnownCorners.First().Node - trackPoint.Node;
                    dist -= ARS.map(Math.Abs(KnownCorners.First().Angle), 5f, 0.5f, 5f, 30f, true);

                    float safety = ARS.DangerousCornerNodes.Contains(KnownCorners.First().Node) ? mem.data.brakeSafety * 1.15f : mem.data.brakeSafety;
                    if (Decisions.ContainsKey(Decision.LateBrake)) safety -= 0.05f;

                    float dSafety = ARS.map(KnownCorners.First().Elevation, 0, -10, 0, 20, true);
                    mem.intention.Speed = MapIdealSpeedForDistance(dist - (ARS.cornerDistSafety + dSafety), BrakeCornerSpeed, safety);

                    if (Decisions.ContainsKey(Decision.FastCorner)) mem.intention.Speed += ARS.MPHtoMS(2f);

                    //if (mem.intention.Speed < ARS.AIData.MinSpeed) mem.intention.Speed = ARS.AIData.MinSpeed;
                }

                if (RacingLineDeviation != 0.0f)
                {
                    float diff = mem.data.DeviationFromCenter - RacingLineDeviation;
                    mem.intention.Speed += ARS.map(Math.Abs(diff), 2.0f, 0.0f, 0.0f, ARS.MPHtoMS(3), true);
                }


                if (mem.intention.Maneuvers.Any())
                {
                    float speedMod = mem.intention.Maneuvers.OrderBy(v => v.Y).First().Y;



                    //If its less than full throttle
                    if (speedMod <= ARS.AIData.SpeedToInput)
                    {
                        //If it results in actual reduction of speed
                        if (Car.Velocity.Length() + speedMod < mem.intention.Speed)
                        {
                            //if(!Driver.IsPlayer) UI.ShowSubtitle("~o~" + Math.Round(speedMod, 1), 500);
                            mem.intention.Speed = Car.Velocity.Length() + speedMod;
                        }


                    }
                    else
                    {
                        //if (!Driver.IsPlayer) UI.ShowSubtitle("~g~" + Math.Round(speedMod, 1), 500);

                        mem.intention.Speed += speedMod;
                    }
                }
            }
        }


        void MakeDecision(Decision d, int chance, int duration, int sCooldown, int fCooldown)
        {
            if (!BannedDecisions.ContainsKey(d) && !Decisions.ContainsKey(d))
            {
                if (ARS.GetRandomInt(0, 100) <= chance) { Decisions.Add(d, Game.GameTime + duration); BannedDecisions.Add(d, Game.GameTime + duration + sCooldown); }
                else BannedDecisions.Add(d, Game.GameTime + fCooldown);
            }
        }

        void MakeMistake(Mistake m, int chance, int duration, int sCooldown, int fCooldown)
        {
            if (!MistakesCooldown.ContainsKey(m) && !Mistakes.ContainsKey(m))
            {
                if (ARS.GetRandomInt(0, 100) <= chance) { Mistakes.Add(m, Game.GameTime + duration); MistakesCooldown.Add(m, Game.GameTime + duration + sCooldown); }
                else MistakesCooldown.Add(m, Game.GameTime + fCooldown);
            }
        }

        void TractionControl()
        {

            float skid = ARS.GetWheelsMaxWheelspin(Car);
            if (vControl.Brake > 0f)
            {
                if (!CurrentHandBrake)
                {


                    if (skid > 2f)
                    {
                        CurrentLockupLimiter += ARS.map(skid, 4f, 2f, -0.2f, -0.025f, true);

                    }
                    else CurrentLockupLimiter += 0.1f;

                }
            }
            CurrentLockupLimiter = ARS.Clamp(CurrentLockupLimiter, 0.05f, 1f);






            float correction = 0.2f;

            if (vControl.Throttle > 0f)
            {
                if (skid < 0f)
                {

                    float min = mem.personality.Stability.WheelspinOnMinSlide;
                    float max = mem.personality.Stability.WheelspinOnMaxSlide;

                    if (Decisions.ContainsKey(Decision.Flatout))
                    {
                        min += 1; max += 1f;
                    }

                    float correctedMaxwheelspin = ARS.map(Math.Abs(SlideAngle), mem.personality.Stability.WheelspinMinSlide, mem.personality.Stability.WheelspinMaxSlide, min, max, true);
                    if (min > max) correctedMaxwheelspin = ARS.map(Math.Abs(SlideAngle), mem.personality.Stability.WheelspinMaxSlide, mem.personality.Stability.WheelspinMinSlide, max, min, true);

                    if (skid < correctedMaxwheelspin)
                    {
                        correction = ARS.map(Math.Abs(skid), correctedMaxwheelspin + 0.4f, correctedMaxwheelspin - 0.4f, -correction, correction, true);
                    }
                }
            }

            CurrentWheelspinLimiter += correction;


            CurrentWheelspinLimiter = ARS.Clamp(CurrentWheelspinLimiter, 0.2f, 1f);

            if (vControl.Throttle > 0f && CurrentWheelspinLimiter < vControl.Throttle) vControl.Throttle = CurrentWheelspinLimiter;
            if (CurrentWheelspinLimiter > vControl.Throttle) CurrentWheelspinLimiter = vControl.Throttle;
        }


        public float GetBrakecornerSpeed(int node = 0, float angle = 0f)
        {
            float result = 0f;
            float expectedGrip = 1f;
            if (ARS.MultiplierInTerrain.ContainsKey(node)) expectedGrip = ARS.MultiplierInTerrain[node];
            if (expectedGrip < 0.99f) expectedGrip *= SurfaceGrip;


            //Speed for Angle
            float cornerAngle = Math.Abs(angle);
            result = ARS.GetSpeedForAngle(Math.Abs(cornerAngle), (Confidence) * expectedGrip);


            //Relative Elevation penalty
            float rElevationPenalty = 0f;
            //if (cornerAngle > 4f && ARS.CornerPoints[node].RelativeElevation < 0f) rElevationPenalty = (ARS.map(ARS.CornerPoints[node].RelativeElevation, -0, -3, 0, 3, true) * 0.25f) * (result * 0.5f);


            if (cornerAngle > 1f && ARS.CornerPoints[node].RelativeElevation < 0f) rElevationPenalty = (ARS.map(ARS.CornerPoints[node].RelativeElevation, 4, -4f, -ARS.MPHtoMS(10), ARS.MPHtoMS(10), true));
            float HillPenalty = 0f;// ARS.map(ARS.CornerPoints[node].Elevation, -2, -30, 0, ARS.MPHtoMS(20), true);

            rElevationPenalty = (float)Math.Round(rElevationPenalty, 4);


            // UI.ShowSubtitle("~o~-"+rElevationPenalty + "ms",500);

            result -= rElevationPenalty + HillPenalty;

            //Downforce

            if (ARS.DevSettingsFile.GetValue<bool>("RACERS", "AccountForDownforce", false))
            {
                float mult = ARS.map(ARS.MStoMPH(result), 80f, 100f, 0f, 1f, true);

                if (mult > 0f)
                {

                    float cBonus = ARS.map(result, 0f, 10f, 0f, 0.1f) * (downf * mult);
                    result = ARS.GetSpeedForAngle(Math.Abs(cornerAngle), (Confidence + cBonus) * expectedGrip);
                }

            }



            //Elevation
            if (KnownCorners.Any() && KnownCorners.First().Elevation < 0f) result -= ARS.map(KnownCorners.First().Elevation, -45f, 0f, ARS.MPHtoMS(10f), 0, true);

            //Aggression
            mem.intention.RivalCornerBonusSpeed = ARS.map(mem.intention.Aggression, 0, 1, 0f, 3f, true);
            result += mem.intention.RivalCornerBonusSpeed;

            //if (result < (ARS.MPHtoMS(30))) result = ARS.MPHtoMS(30);
            return result;
        }

        float MaxOverShootSpeed = 0;
        bool CanLowerSafety = true;
        int FastEntryScore = 0;

        List<Vector3> DirChanges = new List<Vector3>();
        public void ProcessTick()
        {
            if (Driver.IsPlayer)
            {
                if (Lap >= ARS.SettingsFile.GetValue<int>("GENERAL_SETTINGS", "Laps", 5)  && CanRegisterNewLap)
                {
                    World.DrawMarker(MarkerType.CheckeredFlagRect, ARS.TrackPoints.First().Position + new Vector3(0, 0, 5f), ARS.TrackPoints.First().Direction, new Vector3(0, 0, 0), new Vector3(5f, 5f, 5f), Color.White);// DrawLine(vm,last, Color.Black);
                }
            }

            /*
            if (!Driver.IsPlayer && ARS.CanWeUse(Car) && Car.Model.IsCar)
            {
                float trcurve = ARS.rad2deg(ARS.GetTRCurveLat(Car));
                float angle = Vector3.Angle(Car.ForwardVector, Car.Velocity.Normalized);
                float mult = (float)Math.Round(ARS.map(angle, trcurve * 0.1f, trcurve * 0.5f, 1f, 2f, true), 2);
                if (mult > 1f) Car.EngineTorqueMultiplier = mult;
                //if (InverseTorqueDebug) if (mult > 1.0f) UI.ShowSubtitle("Angle: " + Math.Round(angle, 1) + "º /" + trcurve + "º~n~~b~x" + mult.ToString(), 500); else UI.ShowSubtitle("Angle: " + Math.Round(angle, 1) + "º /" + trcurve + "º~n~~w~x" + mult.ToString(), 500);

            }
            */
            //Draw
            if (!Driver.IsPlayer) DrawDebug();

            GetTrackInfo(ARS.Path);

            //Important variable updates
            SideSlide = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true).Normalized.X;
            mem.data.SpeedVector = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true);

            if (trail.Count > 50) trail.RemoveAt(0);
            if (DirChanges.Count >= 5) DirChanges.RemoveAt(0);
            if (!DirChanges.Any()) DirChanges.Add(Car.Velocity.Normalized);

            if (Car.Position.DistanceTo(DirChanges.Last()) >= 1f) DirChanges.Add(Car.Velocity.Normalized);


            /*
            if (TractionCurvePos.DistanceTo(Car.Position) >= 1f)
            {
                if (TractionCurve == Vector3.Zero) TractionCurve = Car.Velocity.Normalized;
                else
                {
                    
                    //Vector3 change = ((TractionCurve - Car.Velocity.Normalized));
                    TRCurveAngle = Vector3.SignedAngle(Car.Velocity.Normalized, TractionCurve, Vector3.WorldUp);
                    if (float.IsNaN(TRCurveAngle)) TRCurveAngle = 0f;
                }
                TractionCurve = Car.Velocity.Normalized;
                TractionCurvePos = Car.Position;

            }
            */



            float a = 0f;
            for (int i = 1; i < DirChanges.Count; i++)
            {
                a += (float)Math.Round(Vector3.SignedAngle(DirChanges[i], DirChanges[i - 1], Vector3.WorldUp), 5);
            }
            a /= DirChanges.Count;
            TRCurveAngle = a;


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

            //AI updates
            if (ProcessAITimeRef < Game.GameTime)
            {
                ProcessAITimeRef = Game.GameTime + 100;
                if (BaseBehavior == RacerBaseBehavior.Race && ARS.Racers.Count >= 1) GetReferenceVehicle(); //refVehicle =
                ProcessAI();

            }

            if (!Driver.IsPlayer)
            {
                if (CatchUpTorqueMult != 1.0f && ARS.SettingsFile.GetValue<bool>("GENERAL_SETTINGS", "Catchup", false)) Car.EngineTorqueMultiplier = CatchUpTorqueMult;
                if (KnownCorners.Any())
                {
                    if (!CanLowerSafety && MaxOverShootSpeed == 0f && BrakeCornerSpeed > 0f && (BrakeCornerSpeed < (Car.Velocity.Length() - (ARS.AIData.SpeedToInput * 3))))
                    {
                        CanLowerSafety = true;
                        if (ARS.DebugLevel > 0) UI.Notify("~b~[" + Name + "]~g~ BSA Enabled for node " + KnownCorners.First().Node);
                    }

                    float overShoot = Car.Velocity.Length() - mem.intention.Speed;

                    //Overshooting the brakepoint by a lot, raise our safety
                    if (CanLowerSafety)
                    {
                        if (overShoot > MaxOverShootSpeed && MaxOverShootSpeed > ARS.AIData.SpeedToInput * 2)
                        {
                            AdjustBrakeSafety(0.1f);
                            CanLowerSafety = false;
                            if (ARS.DebugLevel > 0) UI.Notify("~b~[" + Name + "]~y~ BSA cancelled - Too high potential speed.");
                        }
                    }

                    if (overShoot > MaxOverShootSpeed)
                    {
                        MaxOverShootSpeed = (float)Math.Round(overShoot, 1);
                    }

                    //Cancel safety reduction if we're unstable
                    if (CanLowerSafety)
                    {
                        if (Math.Abs(SideSlide) > 0.3f)
                        {
                            if (ARS.DebugLevel > 0) UI.Notify("~b~[" + Name + "]~y~ BSA cancelled - Slide.");
                            CanLowerSafety = false;
                        }
                    }
                    //int invalidateDist =IdealRelativeAngleExceed > 0f ? 0 : GetBrakePointInvalidateDist();
                    if (KnownCorners.First().Node <= trackPoint.Node + GetBrakePointInvalidateDist())
                    {
                        // if(BrakeCornerSpeed>Car.Velocity.Length() || KnownCorners.First().Node <= trackPoint.Node)
                        KnownCorners.RemoveAt(0);


                        if (ARS.DevSettingsFile.GetValue<bool>("RACERS", "AllowManeuvers", false) && !BannedDecisions.ContainsKey(Decision.Flatout) && referenceVehs.Any() && referenceVehs.First().Pos < Pos && (!KnownCorners.Any() || KnownCorners.First().Node - trackPoint.Node > 100) && Car.CurrentGear <= 3)
                        {
                            MakeDecision(Decision.Flatout, (int)(mem.intention.AggroToReach * 100), 4000, 2000, 4000);
                        }


                        //When reaching the brakepoint, we are going way too fast, adjust
                        if (overShoot > ARS.AIData.SpeedToInput)
                        {
                            AdjustBrakeSafety(0.1f);
                            CanLowerSafety = false;
                        }


                        //Only adjust if the braking has gone right
                        if (MaxOverShootSpeed != 0 && overShoot < ARS.AIData.SpeedToInput * 0.5f && MaxOverShootSpeed > 0f)
                        {
                            if (CanLowerSafety)
                            {
                                AdjustBrakeSafety(-0.05f);
                            }
                            else
                            {
                                //if (ARS.DebugLevel > 0) UI.Notify("~b~[" + Name + "]~o~ Cannot gain confidence (blocked, instability)");
                            }

                        }
                        MaxOverShootSpeed = 0f;
                        CanLowerSafety = false;
                    }
                }
                else if (FastEntryScore > 0) FastEntryScore--;


                //AI Inputs
                if (Driver.IsSittingInVehicle(Car) && !Driver.IsPlayer)
                {

                    if (HandbrakeTime > Game.GameTime) CurrentHandBrake = true; else CurrentHandBrake = false;
                    if (CurrentHandBrake) Car.HandbrakeOn = true; else Car.HandbrakeOn = false;

                    ARS.SetThrottle(Car, vControl.Throttle);
                    ARS.SetBrakes(Car, vControl.Brake);
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
        public int GetBrakePointInvalidateDist()
        {

            float[] spds = { Car.Velocity.Length(), BrakeCornerSpeed };

            //int inv = (int)ARS.map(ARS.MStoMPH(spds.Min()), 30, 100, 2, 10, true);

            int inv = (int)ARS.map(ARS.MStoMPH(spds.Min()), 50, 100, 5, 30, true);
            inv += (int)ARS.map(mem.intention.Aggression, 0, 1, 0, 10, true);
            return inv;
        }
        void AdjustBrakeSafety(float m)
        {
            if (BrakeSafetyAdjustCooldown < Game.GameTime)
            {
                mem.data.brakeSafety += m;
                mem.data.brakeSafety = (float)Math.Round(ARS.Clamp(mem.data.brakeSafety, 0.5f, 1.5f), 2);
                BrakeSafetyAdjustCooldown = Game.GameTime + 2000;

                if (ARS.DebugLevel > 0)
                {
                    if (m > 0f) UI.Notify("~b~[" + Name + "]~y~ Brake Safety raised to " + mem.data.brakeSafety);
                    else UI.Notify("~b~[" + Name + "]~g~ Brake Safety lowered to " + mem.data.brakeSafety);
                }
            }
        }
        void DrawDebug()
        {
            if (!Car.IsInRangeOf(Game.Player.Character.Position, 100f)) return;

            /*
            if (TRCurveAngle != 0f)
            {
                foreach (Vector3 v in ARS.Project(Car.Position, Car.Velocity.Normalized, -TRCurveAngle, (int)Car.Velocity.Length()))
                {
                    World.DrawMarker(MarkerType.DebugSphere, Car.Position + v, Vector3.Zero, Vector3.Zero, new Vector3(0.15f, 0.15f, 0.15f), Color.Blue);
                }
            }
            */




            if (Decisions.Any() && ARS.DebugLevel == 1)
            {
                string text = "";
                foreach (Decision d in Decisions.Keys) text += d.ToString() + "~n~";
                ARS.DrawText(Car.Position + new Vector3(0, 0, 2), text, Color.SkyBlue, 0.5f);
            }
            if (Mistakes.Any() && ARS.DebugLevel == 1)
            {
                string text = "";
                foreach (Mistake d in Mistakes.Keys) text += d.ToString() + "~n~";
                ARS.DrawText(Car.Position + new Vector3(0, 0, 1), text, Color.Orange, 0.5f);
            }

            /*
            if (!Driver.IsPlayer && Driver.IsInRangeOf(Game.Player.Character.Position, 100f))
            {
                if(Decisions.Any()) World.DrawMarker(MarkerType.ChevronUpx1, Car.Position + new Vector3(0, 0, 1.5f), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.5f, 0.5f, -0.5f), Color.Red, false, true, 0, false, "", "", false);
                else World.DrawMarker(MarkerType.ChevronUpx1, Car.Position + new Vector3(0, 0, 1.5f), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.5f, 0.5f, -0.5f), ARS.GetColorFromRedYellowGreenGradient(100 - (mem.intention.Aggression * 100)), false, true, 0, false, "", "", false);
            }
            */




            //if(BannedDecisions.Any()) ARS.DrawText(Car.Position + new Vector3(0, 0, 2), BannedDecisions.First().Key.ToString(), Color.Gray, 0.5f);
            /*
            if (followTrackPoint != null)
            {
                ARS.DrawLine(Car.Position,followTrackPoint.Position, Color.Black);
                World.DrawMarker(MarkerType.ChevronUpx1, followTrackPoint.Position, followTrackPoint.Direction,new Vector3(90,0,0), new Vector3(2f, 2f, 2f), Color.White, false, false, 0, false, "", "", false);
                Vector3 offset = followTrackPoint.Position + (ARS.RotateDir(followTrackPoint.Direction, 90) * mem.data.DeviationFromCenter);
                ARS.DrawLine(Car.Position, offset, Color.Black);
                World.DrawMarker(MarkerType.DebugSphere, offset, Vector3.Zero, Vector3.Zero, new Vector3(0.15f, 0.15f, 0.15f), Color.White);
            }
            */

            //if (Pos == 1) UI.ShowSubtitle(mphStopInTenMeters.ToString(), 1000);
            if (DebugText.Count > 0)
            {
                if (ARS.DebugLevel == (int)DebugMode.Interactions)
                {
                    string d = "~g~- INFO -~w~";
                    foreach (string st in DebugText) d += "~w~~n~" + st;
                    ARS.DrawText(Car.Position + new Vector3(0, 0, 2), d, System.Drawing.Color.White, 0.4f);
                }
            }

            if (trail.Count == 0) trail.Add(Car.Position);
            else if (Car.Position.DistanceTo(trail[trail.Count - 1]) > 2f) trail.Add(Car.Position);

            //if (ARS.DebugLevel == (int)DebugMode.Interactions) foreach (Vehicle v in Traffic) ARS.DrawLine(Car.Position, v.Position, Color.Orange);// World.DrawMarker(MarkerType.ChevronUpx1, v.Position+new Vector3(0,0, v.Model.GetDimensions().Z), Vector3.Zero, Vector3.Zero, new Vector3(1f,1f, -1f), Color.Red, false, false, 0, false, "", "", false);

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

            // ARS.DrawDirectionalBoundingBox(Car);


            /*
            if(Pos == 1 && ARS.Racers.Count >= 2)
            {
                Racer r = ARS.Racers[1];
                Vehicle refVehicle = r.Car;

                //float YBoundingBox = ((Car.Model.GetDimensions().Y / 2) + (refVehicle.Model.GetDimensions().Y / 2));
                //float XBoundingBox = (Car.Model.GetDimensions().X / 2);

                float myDBB = 0f;
                float hisDBB = 0f;
                //XDynamicBoundingBox = ARS.map(Vector3.Angle(Car.ForwardVector, refVehicle.ForwardVector), 0f, 90f, (refVehicle.Model.GetDimensions().X / 2), (refVehicle.Model.GetDimensions().Y / 2));
                myDBB = ARS.map(Vector3.Angle(Car.ForwardVector, Car.Velocity.Normalized), 0f, 90f, (Car.Model.GetDimensions().X / 2), (Car.Model.GetDimensions().Y / 2), true);
                hisDBB = ARS.map(Vector3.Angle(refVehicle.ForwardVector, refVehicle.Velocity.Normalized), 0f, 90f, (refVehicle.Model.GetDimensions().X / 2), (refVehicle.Model.GetDimensions().Y / 2), true);

                //XDynamicBoundingBox = ARS.Clamp(XDynamicBoundingBox, refVehicle.Model.GetDimensions().X / 2, refVehicle.Model.GetDimensions().Y / 2);
                //XDynamicBoundingBox =+ AutosportRacingSystem.map(DirDifference, 0f, 45f, 0f, 2f, true);

                float sideDist = ARS.LeftOrRight(Car.Position, r.Car.Position, Car.Velocity.Normalized);
                float insideDist = Math.Abs(sideDist) - ((myDBB+hisDBB));


                Vector3 myside = (Vector3.Cross(Car.Velocity.Normalized, Vector3.WorldUp) * myDBB);


                //ARS.DrawLine(Car.Position, Car.Position - (Car.RightVector * sideDist), Color.Red);

                ARS.DrawLine(Car.Position + myside + (Car.Velocity.Normalized * 5f), Car.Position + myside + (Car.Velocity.Normalized * -5f), Color.Black);
                ARS.DrawLine(Car.Position - myside + (Car.Velocity.Normalized * 5f), Car.Position - myside + (Car.Velocity.Normalized * -5f), Color.Black);
                ARS.DisplayHelpText(Math.Round(insideDist, 1).ToString());
            }
            */
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
                    float d = mem.data.DeviationFromCenter - RacingLineDeviation;


                    World.DrawMarker(MarkerType.ChevronUpx1, Car.Position + (Car.RightVector * d), Vector3.Zero, new Vector3(90, Car.Heading, 0), new Vector3(0.8f, 0.8f, -3.8f), Color.Green, false, false, 0, false, "", "", false);


                    // ScriptTest.DrawLine(Car.Position + (Car.RightVector * d)+ Car.ForwardVector * -2f, Car.Position + (Car.RightVector * d) + Car.ForwardVector * 2f, Color.Green);
                }

                if (OvertakeDeviation != 0.0f)
                {
                    float d = mem.data.DeviationFromCenter - OvertakeDeviation;


                    World.DrawMarker(MarkerType.ChevronUpx1, Car.Position + (Car.RightVector * d), Vector3.Zero, new Vector3(90, Car.Heading, 0), new Vector3(0.8f, 0.8f, -3.8f), Color.Black, false, false, 0, false, "", "", false);


                }
            }
            if (ARS.DebugLevel == (int)DebugMode.Inputs)//&& Car.IsInRangeOf(Game.Player.Character.Position, 100f)
            {

                Color cc = Color.Green;
                if (vControl.Brake > 0.0f) cc = Color.Yellow;
                if (vControl.Brake > 0.5f) cc = Color.Orange;
                if (vControl.Brake > 0.9f) cc = Color.Red;


                if (mem.intention.DeviationFromCenter != 0.0f)
                {
                    Vector3 lookAheadIntention = followTrackPoint.Position - (Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * 90f) * (followTrackPoint.Direction * mem.intention.DeviationFromCenter));

                    ARS.DrawLine(Car.Position, lookAheadIntention, Color.White);
                    World.DrawMarker(MarkerType.DebugSphere, lookAheadIntention, Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.05f, 0.05f, 0.05f), Color.White, false, false, 0, false, "", "", false);

                }
                else
                {

                    Vector3 lookAhead = followTrackPoint.Position - (Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * 90f) * (followTrackPoint.Direction * mem.data.DeviationFromCenter));

                    ARS.DrawLine(Car.Position, lookAhead, Color.GreenYellow);
                    World.DrawMarker(MarkerType.DebugSphere, lookAhead, Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.05f, 0.05f, 0.05f), Color.White, false, false, 0, false, "", "", false);

                }


                Vector3 Dimensions = Car.Model.GetDimensions();
                Vector3 inputplace = Car.Position + new Vector3(0, 0, (Dimensions.Z * 0.6f));
                Vector3 steergoal = Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * mem.data.SteerAngle) * -Car.ForwardVector; // Quaternion.RotationAxis takes radian angles
                                                                                                                                                  //Vector3 avoidGoal = Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * CManeuverDirection.X) * steergoal; // Quaternion.RotationAxis takes radian angles
                Vector3 idealsteergoal = Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * vControl.SteerAngle) * -Car.ForwardVector; // Quaternion.RotationAxis takes radian angles

                float dimension = Car.Model.GetDimensions().Y + 1f;//new Vector3(90, Car.Heading + mem.data.SteeringAngle, 0)
                World.DrawMarker(MarkerType.ChevronUpx1, Car.Position + (steergoal * -(Dimensions.Y / 2)) + new Vector3(0, 0, -Car.HeightAboveGround), steergoal, new Vector3(90, 0, 0), new Vector3(dimension / 4, dimension / 2, -(dimension / 2)), Color.SkyBlue, false, false, 0, false, "", "", false);
                //World.DrawMarker(MarkerType.ChevronUpx1, Car.Position + (avoidGoal * -(Dimensions.Y / 2)) + new Vector3(0, 0, -Car.HeightAboveGround), avoidGoal, new Vector3(90, 0, 0), new Vector3(dimension / 4, dimension / 2, -(dimension / 2)), Color.Yellow, false, false, 0, false, "", "", false);
                //World.DrawMarker(MarkerType.ChevronUpx1, Car.Position + (CManeuverDirection * -(Dimensions.Y / 2)) + new Vector3(0, 0, -Car.HeightAboveGround), CManeuverDirection, new Vector3(90, 0, 0), new Vector3(dimension / 4, dimension / 2, -(dimension / 2)), Color.Yellow, false, false, 0, false, "", "", false);

                if (mem.data.SteerAngle != vControl.SteerAngle) World.DrawMarker(MarkerType.ChevronUpx1, Car.Position + (idealsteergoal * -(Dimensions.Y / 2)) + new Vector3(0, 0, -Car.HeightAboveGround), idealsteergoal, new Vector3(90, 0, 0), new Vector3(dimension / 4, dimension / 2, -(dimension / 2)), Color.FromArgb(50, Color.SkyBlue), false, false, 0, false, "", "", false);


                World.DrawMarker(MarkerType.DebugSphere, inputplace + (Car.ForwardVector * ((Dimensions.Y / 2) * (-CurrentLockupLimiter))), Car.ForwardVector, new Vector3(90, 0, 0), new Vector3(0.05f, 0.05f, 0.05f), Color.Black, false, false, 0, false, "", "", false);
                World.DrawMarker(MarkerType.DebugSphere, inputplace + (Car.ForwardVector * ((Dimensions.Y / 2) * (CurrentWheelspinLimiter))), Car.ForwardVector, new Vector3(90, 0, 0), new Vector3(0.05f, 0.05f, 0.05f), Color.Black, false, false, 0, false, "", "", false);



                World.DrawMarker(MarkerType.ChevronUpx1, inputplace + (Car.ForwardVector * ((Dimensions.Y / 2) * (vControl.Throttle))), Car.ForwardVector, new Vector3(90, 0, 0), new Vector3(1f, 1f, 1f), Color.Green, false, false, 0, false, "", "", false);
                if (vControl.Brake > 0.0f) World.DrawMarker(MarkerType.ChevronUpx1, inputplace + (Car.ForwardVector * ((Dimensions.Y / 2) * (-vControl.Brake))), Car.ForwardVector, new Vector3(90, 0, 0), new Vector3(1f, 1f, 1f), cc, false, false, 0, false, "", "", false);
                ARS.DrawLine(inputplace + (Car.ForwardVector * -(Dimensions.Y / 2)), inputplace + (Car.ForwardVector * (Dimensions.Y / 2)), Color.White);
                ARS.DrawLine(inputplace + (Car.RightVector * 0.2f), inputplace - (Car.RightVector * 0.2f), Color.White);


                Vector3 steerG = Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * mem.data.SteerAngle) * Car.ForwardVector;
                Vector3 strMaxRight = Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * maxStrTRCurve) * Car.ForwardVector;
                Vector3 strMaxLeft = Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * -maxStrTRCurve) * Car.ForwardVector;

                ARS.DrawLine(Car.Position, Car.Position + (strMaxLeft * 5f), Color.Red);
                ARS.DrawLine(Car.Position, Car.Position + (strMaxRight * 5f), Color.Red);
                ARS.DrawLine(Car.Position, Car.Position + (steerG * 5f), Color.Blue);


                Vector3 smoothLeft = Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * maxSteerSmooth) * Car.ForwardVector;
                Vector3 smoothRight = Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * -maxSteerSmooth) * Car.ForwardVector;

                ARS.DrawLine(Car.Position, Car.Position + (smoothLeft * 5f), Color.White);
                ARS.DrawLine(Car.Position, Car.Position + (smoothRight * 5f), Color.White);


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

            if (ARS.DebugLevel == (int)DebugMode.Cornering)
            {
                Vector3 source = Car.Position + new Vector3(0, 0, Car.Model.GetDimensions().Z * 0.6f);

                ARS.DrawText(source, Math.Round(ARS.MStoMPH(Car.Velocity.Length())).ToString() + "mph", Color.White, 0.5f);//mphStopInTenMeters.ToString()

                if (1 == 2 && Pos <= 1)
                {
                    Vector3 front = Car.Position;
                    Vector3 back = Car.Position;

                    front += Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * maxCarToTrackpointAngle) * (trackPoint.Direction * 5);
                    back += Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * maxCarToTrackpointAngle) * -(trackPoint.Direction * 5);

                    World.DrawMarker(MarkerType.DebugSphere, front, Vector3.Zero, Vector3.Zero, new Vector3(0.05f, 0.05f, 0.05f), Color.Red);
                    World.DrawMarker(MarkerType.DebugSphere, back, Vector3.Zero, Vector3.Zero, new Vector3(0.05f, 0.05f, 0.05f), Color.Red);


                    Vector3 ctocorner = Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * CarToCornerAngle) * (trackPoint.Direction * 5);
                    Vector3 ctocornerb = Quaternion.RotationAxis(Vector3.WorldUp, (float)(Math.PI / 180f) * CarToCornerAngle) * -(trackPoint.Direction * 5);

                    //Corner angle
                    ARS.DrawLine(Car.Position + ((trackPoint.Direction * 5)), Car.Position - ((trackPoint.Direction * 5)), Color.White);

                    //Car to corner angle
                    ARS.DrawLine(Car.Position + ((ctocorner)), Car.Position + ((ctocornerb)), Color.Blue);

                    //Car to corner max angle
                    ARS.DrawLine(back, front, Color.Red);

                    //Difference
                    ARS.DrawLine(Car.Position + ((ctocorner)), Car.Position + ((trackPoint.Direction * 5)), Color.Orange);

                }
                int d = 0;
                foreach (CornerPoint c in KnownCorners)
                {
                    d++;
                    Vector3 wp = ARS.Path[c.Node];
                    float expectedSpeed = MapIdealSpeedForDistance(wp.DistanceTo(ARS.Path[trackPoint.Node]), c.Speed); //BrakeCornerSpeed;
                    Color gColor = ARS.GetColorFromRedYellowGreenGradient(ARS.map(expectedSpeed - Car.Velocity.Length(), -ARS.AIData.SpeedToInput, ARS.AIData.SpeedToInput, 0, 100, true));

                    if (d == 1)
                    {
                        ARS.DrawLine(source, wp, gColor);
                        //ARS.DrawText(wp+new Vector3(0,0,1), c.Angle.ToString() + "º~w~~n~~y~" + Math.Round(ARS.MStoMPH(c.Speed)) + "~w~mph~n~Elv:" + Math.Round(c.RelativeElevation, 1) + "º", textColor, ARS.map(d, 10, 1, 0.15f, 0.5f, true)); //*(c[5]+1f)
                        ARS.DrawText(wp + new Vector3(0, 0, 1), c.Angle.ToString() + "º~w~ > ~y~" + Math.Round(ARS.MStoMPH(c.Speed)) + "~w~mph~n~Relative Elevation:" + Math.Round(c.RelativeElevation, 1) + "º", Color.White, ARS.map(d, 10, 1, 0.15f, 0.5f, true)); //*(c[5]+1f)
                    }

                    World.DrawMarker(MarkerType.ChevronUpx1, wp, ARS.TrackPoints[c.Node].Direction, new Vector3(90, 0, 0), new Vector3(ARS.TrackPoints[c.Node].TrackWide * 2, 5, 2), Color.FromArgb(50, gColor.R, gColor.G, gColor.B));

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



            //Interaction with other vehicles            
            foreach (Racer r in referenceVehs)
            {
                Vehicle refVehicle = r.Car;
                if (!ARS.CanWeUse(refVehicle)) continue;

                if (ARS.CanWeUse(refVehicle))
                {
                    Vector3 PositionRelativeToUs = (ARS.GetOffset(Car, refVehicle));
                    Vector3 PositionRelativeToThem = (ARS.GetOffset(refVehicle, Car));

                    float SideRelToTheirDirection = ARS.LeftOrRight(Car.Position, r.Car.Position, r.Car.Velocity.Normalized);//r.Car.Velocity.Normalized
                    float SideRelToOurDirection = ARS.LeftOrRight(r.Car.Position, Car.Position, Car.Velocity.Normalized);//Positive=left of us;

                    float impedimentDist = ARS.Clamp(Math.Abs(SideRelToTheirDirection) - ((BoundingBox + r.BoundingBox)), 0f, 10f) * (SideRelToTheirDirection > 0 ? 1 : -1);

                    float SpeedDiff = (float)Math.Round(Car.Velocity.Length() - refVehicle.Velocity.Length(), 1);// Negative=he is faster. Positive=he is slower;

                    float YBoundingBox = Math.Abs((Car.Model.GetDimensions().Y / 2) + (refVehicle.Model.GetDimensions().Y / 2));
                    float XBoundingBox = BoundingBox;

                    float XDynamicBoundingBox = 0f;

                    //Their bounding box relative to our direction
                    XDynamicBoundingBox = ARS.map(Vector3.Angle(Car.Velocity.Normalized, refVehicle.ForwardVector), 0f, 90f, refVehicle.Model.GetDimensions().X / 2, (refVehicle.Model.GetDimensions().Y / 2));
                    XDynamicBoundingBox = ARS.Clamp(XDynamicBoundingBox, refVehicle.Model.GetDimensions().X / 2, refVehicle.Model.GetDimensions().Y / 2);

                    //Our own bounding box                    
                    XDynamicBoundingBox += XBoundingBox;

                    //How much the distance intersects with the sum of both BBs
                    float sideToSideDist = Math.Abs(SideRelToTheirDirection) - ((XDynamicBoundingBox));
                    float targetInsideDist = Math.Abs(SideRelToOurDirection) - ((XDynamicBoundingBox));


                    float DirDifference = Vector3.SignedAngle(Car.Velocity.Normalized, refVehicle.Velocity.Normalized, Car.UpVector);// + ((TRCurveAngle) - (r.TRCurveAngle));

                    //Adjust the speed relative to their direction relative to us
                    float targetRelativeSpeed = ARS.map(Math.Abs(DirDifference), 180f, 0f, -refVehicle.Velocity.Length(), refVehicle.Velocity.Length(), true);
                    SpeedDiff = (float)Math.Round(Car.Velocity.Length() - targetRelativeSpeed, 2);

                    float distance = (Car.Position.DistanceTo2D(refVehicle.Position) - YBoundingBox);

                    float sToHit = 10f;
                    if (SpeedDiff > 0f) sToHit = (float)Math.Round(distance / SpeedDiff, 2);
                    if (float.IsNaN(sToHit) || float.IsInfinity(sToHit)) sToHit = 10f;
                    sToHit = ARS.Clamp(sToHit, 0f, 10f);

                    //Behind them
                    if (PositionRelativeToUs.Y > 0f)
                    {
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
                            if (sToHit < 5f && Math.Abs(impedimentDist) < 10f)
                            {
                                if (fImpedimentOnSide == 0.0f || Math.Abs(impedimentDist) < Math.Abs(fImpedimentOnSide)) fImpedimentOnSide = impedimentDist;
                            }
                        }

                        float safeSideDist = (XDynamicBoundingBox / 2) + mem.intention.RivalSideDist;

                        //Avoid rearends via steer
                        if (!Decisions.ContainsKey(Decision.NoOvertake) && safeSideDist < 4f)
                        {
                            //Sanity steer limit
                            float maxSteer = 50f;
                            float tSteering = 0f;

                            //Lane to aim at
                            float lane = ARS.map(sideToSideDist, safeSideDist, 0f, 0f, safeSideDist, true) * (-SideRelToOurDirection > 0f ? 1 : -1);

                            if (lane > 0 == r.mem.data.DeviationFromCenter > 0) if (Math.Abs(r.mem.data.DeviationFromCenter + lane) + XDynamicBoundingBox + safeSideDist > r.trackPoint.TrackWide) lane *= -1; //If we cannot overtake by this side (too close to th track edge), steer to the other side

                            //Approaching them
                            if (SpeedDiff > 0.0f)
                            {
                                if (Math.Abs(DirDifference) < 30f)
                                {
                                    float SPDtoSteer = ARS.map(ARS.MStoMPH(Car.Velocity.Length()), 100f, 50f, 1f, 4f, true);
                                    tSteering = GetSteerToDeviation(mem.data.DeviationFromCenter + lane, 0f, 1f, SPDtoSteer);
                                    tSteering *= ARS.map(sToHit, 5f, 1f, 0f, 2f, true); //3s=x1
                                                                                        //UI.ShowSubtitle(tSteering.ToString(), 500);
                                }
                            }
                            else //Not approaching, we are slower
                            {
                                maxSteer = ARS.map(distance, mem.personality.Rivals.BehindRivalMinDistance + 10f, mem.personality.Rivals.BehindRivalMinDistance, 0f, 3f, true);
                                tSteering = GetSteerToDeviation(mem.data.DeviationFromCenter + lane, 0f, 2f, maxSteer);
                            }


                            //After making a steering decision, adjust for our context in the track
                            if (Math.Abs(tSteering) > 0.0f)
                            {
                                mem.intention.DeviationFromCenter = lane;

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
                                if (tSteering != 0f && !float.IsNaN(tSteering)) mem.intention.Maneuvers.Add(new Vector2(tSteering, ARS.AIData.SpeedToInput));
                            }
                        }

                        //Avoid rearends via brake
                        if (targetInsideDist < 0f)
                        {
                            if (SpeedDiff > -ARS.AIData.SpeedToInput)
                            {
                                float safeDistance = (Car.Position.DistanceTo2D(refVehicle.Position) - (YBoundingBox + mem.intention.RivalBehindDist));
                                float rSpeed = refVehicle.Velocity.Length(); // ARS.map(safeDistance, 0f, 1f, refVehicle.Velocity.Length() - 2f, refVehicle.Velocity.Length(), true);

                                float idealSpd = MapIdealSpeedForDistance(safeDistance, rSpeed, mem.data.brakeSafety * ARS.map(sToHit, 3f, 1f, 0.5f, 1f, true));
                                if (idealSpd < rSpeed) idealSpd = rSpeed;

                                float m = idealSpd - Car.Velocity.Length();
                                if (m > ARS.AIData.SpeedToInput) m = ARS.AIData.SpeedToInput;
                                if (!float.IsNaN(m) && !float.IsInfinity(m) && m < ARS.AIData.SpeedToInput) mem.intention.Maneuvers.Add(new Vector2(0, m));
                            }
                        }
                    }

                    //Side to side
                    if (Math.Abs(PositionRelativeToThem.Y) < YBoundingBox)//&& Offset.Y > 0f
                    {
                        if (sideToSideDist < 20f)
                        {
                            List<Vector2> maneuvers = new List<Vector2>();


                            float maxSteer = 10f; // ARS.map(ARS.MStoMPH(Car.Velocity.Length()), 100f, 30f, 5f, 20f, true);

                            float safeSideDist = (XDynamicBoundingBox / 2) + mem.intention.RivalSideDist;

                            if (Car.IsTouching(refVehicle) && PositionRelativeToThem.Y < 0f) maneuvers.Add(new Vector2(5f * (SideRelToOurDirection > 0f ? 1 : -1), 0f));

                            /*
                            else if (sideToSideDist < safeSideDist)
                            {                                
                                float steerAway = ARS.map(sideToSideDist, safeSideDist, 0f, 0f, 2f, true) * (SideRelToTheirDirection < 0f ? 1 : -1);                                                               
                                Vector2 keepAway = new Vector2(steerAway, ARS.AIData.SpeedToInput);                                
                                maneuvers.Add(keepAway);                                
                            }
                            */

                            //Closing in to us
                            if (sideToSideDist < 10.0f && DirDifference > 0 == PositionRelativeToUs.X > 0)
                            {
                                Vector3 aimTo = Vector3.Zero;
                                float maxAllowedAngle = ARS.map(Math.Abs(sideToSideDist), 0.25f, safeSideDist * 2, 0f, 20f, true);
                                //if(PositionRelativeToUs.Y<0f) maxAllowedAngle = ARS.map(Math.Abs(sideToSideDist), 0f, safeSideDist * 2, 0f, 20f, true);

                                float str = 0f;
                                if (maxAllowedAngle > 0.0f)
                                {
                                    if (Math.Abs(DirDifference) > maxAllowedAngle)
                                    {
                                        float dangAng = Math.Abs(DirDifference) - maxAllowedAngle;
                                        aimTo = Vector3.Lerp(Car.Velocity.Normalized, r.Car.Velocity.Normalized, ARS.map(dangAng, 20f, 0f, 0f, 1f, true));
                                    }
                                }
                                else
                                {
                                    aimTo = r.Car.Velocity.Normalized;// Vector3.Lerp(Car.Velocity.Normalized, r.Car.Velocity.Normalized, ARS.map(sideToSideDist, safeSideDist+2f, safeSideDist, 0f, 1f, true));
                                }

                                if (aimTo != Vector3.Zero)
                                {
                                    str = Vector3.SignedAngle(Car.Velocity.Normalized, aimTo, Car.UpVector);
                                }

                                // mem.intention.Maneuvers.Add(new Vector2(str, ARS.SpeedToInput));
                                if (str > 0 != SideRelToTheirDirection > 0f && !float.IsNaN(str)) maneuvers.Add(new Vector2(str, ARS.AIData.SpeedToInput));



                            }

                            if (maneuvers.Any())
                            {
                                Vector2 m = maneuvers.OrderByDescending(x => Math.Abs(x.X)).First();
                                m.X = ARS.Clamp(m.X, -maxSteer, maxSteer);


                                /*
                                //If we are steering towards the outside of the track, limit our steering to avoid leaving
                                if (m.X > 0f == mem.data.DeviationFromCenter < 0f)
                                {
                                    float mult = ARS.map(OutOfTrackDistance(), 0f, -2f, 0f, 3f, true);

                                    m.X = ARS.Clamp(m.X, -mult, mult);
                                    m.X *= mult;
                                }               
                                */
                                mem.intention.Maneuvers.Add(m);
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
        public int FollowNode = 0;



        public TrackPoint trackPoint = new TrackPoint();
        public TrackPoint followTrackPoint = new TrackPoint();
        public void GetTrackInfo(List<Vector3> Track)
        {
            int lookAhead = (int)(ARS.map(ARS.MStoMPH(Car.Velocity.Length()), 30f, 130f, 5, 15, true) * ARS.map(Confidence, 2, 1, 1f, 1.25f, true));
            if (Car.Model.IsBicycle || Car.Model.IsBike) lookAhead = 2;

            lookAhead += (int)Car.Model.GetDimensions().Y / 2;


            int cNode = trackPoint.Node;

            if (cNode > ARS.TrackPoints.Count - 5)
            {
                trackPoint = ARS.TrackPoints.First();
            }
            else
            {
                trackPoint = ARS.TrackPoints.Where(p => p.Node >= cNode - 10 && p.Node <= cNode + 10).OrderBy(t => t.Position.DistanceTo(Car.Position)).First();
            }
            if (trackPoint.Node + lookAhead >= ARS.TrackPoints.Count) followTrackPoint = ARS.TrackPoints[lookAhead];
            else followTrackPoint = ARS.TrackPoints[trackPoint.Node + lookAhead];

            mem.data.DeviationFromCenter = ARS.LeftOrRight(Car.Position, trackPoint.Position, trackPoint.Direction);
        }
        public void AddDebugText(string s)
        {
            //s += "~w~";
            s = "~w~" + s + "~w~";
            if (!DebugText.Contains(s)) DebugText.Add(s);
        }


        public void ProcessAI() //100ms
        {

            if (Car.Model.IsCar && !Driver.IsPlayer)
            {
                if (referenceVehs.Any() && Car.IsInRangeOf(referenceVehs.First().Car.Position, 5f))
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
            if (HalfSecondCheck < Game.GameTime)
            {
                HalfSecondCheck = Game.GameTime + 500 + (Pos * 10);

                //Decision handling
                if (Decisions.Any(de => de.Value < Game.GameTime)) Decisions.Remove(Decisions.First(de => de.Value < Game.GameTime).Key);
                if (BannedDecisions.Any(de => de.Value < Game.GameTime)) BannedDecisions.Remove(BannedDecisions.First(de => de.Value < Game.GameTime).Key);

                //Mistake handling
                if (Mistakes.Any(de => de.Value < Game.GameTime)) Mistakes.Remove(Mistakes.First(de => de.Value < Game.GameTime).Key);
                if (MistakesCooldown.Any(de => de.Value < Game.GameTime)) MistakesCooldown.Remove(MistakesCooldown.First(de => de.Value < Game.GameTime).Key);

                if (ARS.DevSettingsFile.GetValue<bool>("RACERS", "AllowManeuvers", false) && !BannedDecisions.ContainsKey(Decision.NoOvertake) && KnownCorners.Any() && referenceVehs.Any() && referenceVehs.Where(r => r.Car.IsInRangeOf(Car.Position, 30f) && r.Car.Velocity.Length() < Car.Velocity.Length() + ARS.MPHtoMS(10f)).Count() >= 3)
                {
                    MakeDecision(Decision.NoOvertake, 100 - (int)(mem.intention.AggroToReach * 100), 5000, 3000, 5000);
                }




                if (Math.Abs(mem.data.DeviationFromCenter) > trackPoint.TrackWide && !ControlledByPlayer && BaseBehavior == RacerBaseBehavior.Race)
                {

                    if (OutOfTrack == 0) OutOfTrack = Game.GameTime;
                    else if (Game.GameTime - OutOfTrack > 13000)
                    {
                        OutOfTrack = 0;
                        ResetIntoTrack();
                    }

                }
                else
                {
                    if (OutOfTrack != 0) OutOfTrack = 0;
                }

                if (!Driver.IsPlayer) if (referenceVehs.Count > 0) Driver.Task.LookAt(referenceVehs[0].Driver, 2000); else if (Car.Velocity.Length() > 5f) Driver.Task.LookAt(Car.Position + Car.Velocity, 2000);


                //Rocket boost
                if (!KnownCorners.Any())
                {
                    if (BaseBehavior == RacerBaseBehavior.Race && Vector3.Angle(Car.ForwardVector, trackPoint.Direction) < 5f && Math.Abs(FollowTrackCornerAngle) < 0.5f && Math.Abs(SideSlide) < 0.1f && Math.Abs(CurrentStr) < 0.2f) Function.Call((Hash)0x81E1552E35DC3839, Car, true);

                }
                if (Function.Call<bool>((Hash)0x3D34E80EED4AE3BE, Car) && vControl.Brake > 0.1f) Function.Call((Hash)0x81E1552E35DC3839, Car, false);

            }


            //Data dathering
            BoundingBox = ARS.GetDirectionalBoundingBox(Car);

            mem.intention.RivalSideDist = mem.personality.Rivals.SideToSideMinDist; // ARS.map(referenceVehs.Count, 1, 4, 0.5f, 2f, true);
            mem.intention.RivalBehindDist = mem.personality.Rivals.BehindRivalMinDistance; // = ARS.map(referenceVehs.Count, 1, 4, 0.25f, 3f, true);            

            mem.intention.Aggression = ARS.Clamp(mem.intention.Aggression, 0, 1f);
            SlideAngle = Vector3.SignedAngle(Car.Velocity.Normalized, Car.ForwardVector, Car.UpVector);

            if (BaseBehavior == RacerBaseBehavior.GridWait && HandbrakeTime < Game.GameTime) HandbrakeTime = Game.GameTime + (100 * ARS.GetRandomInt(2, 6));

            if (ARS.SettingsFile.GetValue<bool>("GENERAL_SETTINGS", "Ghosts", false))
            {
                Car.Alpha = 255;
                foreach (Racer r in referenceVehs)
                {
                    if (Function.Call<bool>(Hash.IS_ENTITY_AT_ENTITY, Car, r.Car, (r.Car.Model.GetDimensions().X * 2) + 0.2f, (r.Car.Model.GetDimensions().Y * 2) + 0.2f, (r.Car.Model.GetDimensions().Z * 2) + 0.2f, true, true, true))
                    {
                        Function.Call(Hash.SET_ENTITY_NO_COLLISION_ENTITY, Car, r.Car, false);
                        Car.Alpha = 150;
                    }
                }
            }





            //Slow checks
            if (OneSecondGameTime < Game.GameTime)
            {
                OneSecondGameTime = Game.GameTime + (1000 + (ARS.GetRandomInt(-10, 10) * 10));

                if (!ControlledByPlayer)
                {
                    //Cheats
                    CatchUpTorqueMult = 1.0f;
                    if (ARS.SettingsFile.GetValue<bool>("GENERAL_SETTINGS", "Catchup", false))
                    {
                        if (Pos == 1 && ARS.Racers.Count > 1)
                        {
                            CatchUpTorqueMult = ARS.map(ARS.Racers[1].Car.Position.DistanceTo(Car.Position), 100f, 50f, 0.5f, 1f, true);
                        }
                        if (Math.Abs(mem.data.SpeedVector.X) < 0.1f && ARS.GetPercent(Pos, ARS.Racers.Count) >= 75 && !referenceVehs.Any()) CatchUpTorqueMult = 2f;

                    }

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
                    ARS.WorstCornerAhead(this);

                }

                if (ARS.AIRacerAutofix == 2 && Function.Call<bool>(Hash._IS_VEHICLE_DAMAGED, Car))
                {
                    if (ARS.DebugLevel > 0) UI.Notify("~b~" + Name + " auto repaired");
                    Car.Repair();
                }

                //Stuck
                if (BaseBehavior == RacerBaseBehavior.Race && Driver.IsSittingInVehicle(Car))
                {
                    if (Car.Velocity.Length() < 1f && vControl.Throttle != 0.0f && vControl.Brake == 0 && !CurrentHandBrake) StuckScore++; else StuckScore--;
                    StuckScore = (int)ARS.Clamp(StuckScore, 0, 10);

                    if (StuckScore >= 2)
                    {

                        if (StuckScore >= 5)//5
                        {
                            if (Driver.IsSittingInVehicle(Car) && Car.EngineHealth > 100)
                            {
                                StuckScore = 0;
                                ResetIntoTrack();
                                StuckRecover = false;
                            }
                        }
                        if (!StuckRecover && !Car.Model.IsBike)
                        {
                            LastStuckPlace = Car.Position;
                            if (ARS.DebugLevel > 0) UI.Notify("~b~" + Car.FriendlyName + " tries to recover");
                            StuckRecover = true;
                        }
                    }
                    if (StuckRecover && (!Car.IsInRangeOf(LastStuckPlace, 5f) || mem.data.SpeedVector.Y > 3f))
                    {
                        StuckRecover = false;
                        StuckScore = 0;
                    }
                }

            }





            AngleToTrack = Vector3.SignedAngle(Car.ForwardVector, trackPoint.Direction, Vector3.WorldUp);

            if (BaseBehavior == RacerBaseBehavior.Race && ARS.GetPercent(trackPoint.Node, ARS.TrackPoints.Count) > 50 && !CanRegisterNewLap) CanRegisterNewLap = true;
            if (CanRegisterNewLap)
            {

                if (ARS.GetPercent(trackPoint.Node, ARS.TrackPoints.Count) < 10 || (ARS.IsPointToPoint && ARS.GetPercent(trackPoint.Node, ARS.TrackPoints.Count) > 90 && ARS.ConvertToLocalOffset(Car, ARS.TrackPoints.Last().Position).Y < 0f))
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

            oldNode = trackPoint.Node;

            HandleGrip();




            if (!ControlledByPlayer)
            {
                mem.intention.Maneuvers.Clear();
                VehicleInteractions();



                if (ARS.RaceStatus == RaceState.InProgress) HandleOvershoot();

                SpeedLogic();
                SteerLogic();

                ApplyThrottle();
                ApplySteer();
                TractionControl();

            }



        }
        int BrakeSafetyAdjustCooldown = 0;
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

        string handlingFlags = "";
        string modelFlags = "";

        public float downf = 0f;
        public float TRCurveLateral = 0f;
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

            HandlingGrip = Function.Call<float>((Hash)0xA132FB5370554DB0, Car);//Car grip
            HandlingGrip = ARS.Clamp(HandlingGrip, 0.4f, 3f);

            int nWheels = ARS.GetNumWheels(Car);
            float penalization = 0f;
            for (int i = -1; i < 6; i++)
            {

                if (Function.Call<bool>(Hash.IS_VEHICLE_TYRE_BURST, Car, i, true))
                {

                    float penalizationPercent = (float)Math.Round((1f / (float)nWheels) * 50, 2);
                    penalization += ((float)HandlingGrip / (float)100) * penalizationPercent;

                }
            }
            HandlingGrip -= penalization;
            if (HandlingGrip < 0.1f) HandlingGrip = 0.1f;


            HandlingGrip *= (SurfaceGrip * wetavg);


            Confidence = HandlingGrip;
            Confidence = ARS.Clamp(Confidence, 0.2f, 3f);


            if (Pos < 2 && !ARS.MultiplierInTerrain.ContainsKey(trackPoint.Node))
            {
                ARS.MultiplierInTerrain.Add(trackPoint.Node, SurfaceGrip);
            }
        }


        float maxCarToTrackpointAngle = 0f;
        float CarToCornerAngle = 0f;
        float IdealRelativeAngleExceed = 0f;

        float UndersteerPenalty = 0f;
        void HandleOvershoot()
        {
            //if (ARS.MStoMPH(Car.Velocity.Length()) < 30) return;

            Vector3 currentTrackDir = trackPoint.Direction;
            currentTrackDir.Z = 0f;
            Vector3 followTrackDir = followTrackPoint.Direction;
            followTrackDir.Z = 0f;

            Vector3 carDir = Car.Velocity.Normalized;
            carDir.Z = 0f;

            float angle = Vector3.SignedAngle(currentTrackDir, followTrackDir, Vector3.WorldUp);

            CarToCornerAngle = Vector3.SignedAngle(carDir, currentTrackDir, Vector3.WorldUp);
            //CarToCornerAngle *= (angle < 0.0f ? 1 : -1);

            float man = ARS.AIData.SpeedToInput;

            float CornerAngle = ARS.CornerPoints[trackPoint.Node].Angle;
            if (Math.Abs(CornerAngle) > 0f && CarToCornerAngle > 0f == CornerAngle > 0f)
            {

                float maxRAngle = 5f;//ARS.map(Math.Abs(CornerAngle), 10f, 1f, 20f, 20f, true);

                maxCarToTrackpointAngle = (float)Math.Round(ARS.map(DistToOutside(), 3f, 10f, 0f, maxRAngle, true)) * (angle < 0.0f ? 1 : -1);


                IdealRelativeAngleExceed = (float)Math.Round(Math.Abs(CarToCornerAngle) - Math.Abs(maxCarToTrackpointAngle), 1); //How many degress over the limit
                //float TRCurveOvershoot = Math.Abs(followTrackPoint.Angle) - Math.Abs(TRCurveAngle); //Positive: our TR curve is more open than the corner
                float TRCurveOvershoot = Math.Abs(ARS.CornerPoints[followTrackPoint.Node].Angle / 10) - Math.Abs(TRCurveAngle); //Positive: our TR curve is more open than the corner

                //If our TRCurve is better than the corner, have confidence and don't brake as much
                float minInput = ARS.map(TRCurveOvershoot, 2, -2, -ARS.AIData.SpeedToInput, ARS.AIData.SpeedToInput, true);

                //Never brake below 10mph less than the original corner speed
                float minSpd = BrakeCornerSpeed - ARS.MPHtoMS(10);

                man = ARS.map(IdealRelativeAngleExceed, 5, -5f, minInput, ARS.AIData.SpeedToInput, true);


                if (Car.Velocity.Length() + man < minSpd)
                {
                    minInput = ARS.map((Car.Velocity.Length() + man) - minSpd, 0, -10, -ARS.AIData.SpeedToInput, ARS.AIData.SpeedToInput, true);
                    man = ARS.map(IdealRelativeAngleExceed, 5, -5f, minInput, ARS.AIData.SpeedToInput, true);

                }


                if (man != ARS.AIData.SpeedToInput)
                {
                    mem.intention.Maneuvers.Add(new Vector2(0, man));

                }
                //string debug = "Overshoot: " + Math.Round(TRCurveOvershoot, 3)+"~n~Angle exceed:" + Math.Round(IdealRelativeAngleExceed, 3) ;
                //UI.ShowSubtitle(debug.ToString(), 500);

                if (Math.Abs(vControl.SteerAngle) >= maxStrTRCurve) UndersteerPenalty -= 1f; else UndersteerPenalty = ARS.AIData.SpeedToInput;
                /*
                if (UndersteerPenalty < ARS.AIData.SpeedToInput)
                {
                    if (TRCurveOvershoot < 0f) UndersteerPenalty = ARS.Clamp(UndersteerPenalty, 0f, ARS.AIData.SpeedToInput);
                    else UndersteerPenalty = ARS.Clamp(UndersteerPenalty, -ARS.AIData.SpeedToInput, ARS.AIData.SpeedToInput);

                    mem.intention.Maneuvers.Add(new Vector2(0, UndersteerPenalty));
                }
                */
            }
        }


        public List<Vehicle> Traffic = new List<Vehicle>();
        public void GetReferenceVehicle()
        {
            if (ARS.DevSettingsFile.GetValue<bool>("TRACK", "Traffic", false))
            {
                Traffic.Clear();
                Traffic = ARS.GlobalTraffic.Where(s => s.Position.DistanceTo(Car.Position) < 100f
                && (ARS.GetOffset(Car, s).Y > 0f)
                && Math.Abs(ARS.GetOffset(Car, s).X) < 10f
                && Math.Abs(ARS.GetOffset(Car, s).Z) < 5f).ToList();
                Traffic.OrderBy(v => v.Position.DistanceTo(Car.Position));
                if (Traffic.Count > 4) Traffic.RemoveRange(3, Traffic.Count - 4);
            }


            mem.intention.AggroToReach = 0f;
            Vector3 refpoint = Car.Position + (Car.Velocity.Normalized * 2f);
            referenceVehs.Clear();
            List<Racer> Candidates = new List<Racer>();
            foreach (Racer r in ARS.Racers as IEnumerable<Racer>)
            {

                if (r.Car.Handle != Car.Handle && r.Car.Position.DistanceTo(refpoint) < 120f)
                {
                    float dist = r.Car.Position.DistanceTo(Car.Position);

                    //Aggro
                    if (r.Pos < Pos && dist < 100f)
                    {
                        float aggro = 0f;
                        if (dist > 50f) aggro = ARS.map(dist, 100f, 50f, 0f, mem.personality.Rivals.AggressionCap * 0.75f, true);
                        else aggro = ARS.map(dist, 50f, 20f, mem.personality.Rivals.AggressionCap * 0.75f, mem.personality.Rivals.AggressionCap, true);

                        if (aggro > mem.intention.AggroToReach) mem.intention.AggroToReach = aggro;
                    }


                    if (ARS.GetOffset(Car, r.Car).Y < -5f) continue;
                    if (dist > 10f)
                    {
                        float spdDiff = Car.Velocity.Length() - r.Car.Velocity.Length(); //Positive=we are faster
                        float maxDiff = (float)Math.Round(ARS.map(dist, 30f, 10f, 0.0f, 10.0f), 1);
                        float sToHit = (float)Math.Round(dist / spdDiff, 2);
                        if (float.IsNaN(sToHit) || float.IsInfinity(sToHit)) sToHit = 10f;

                        if (ARS.GetOffset(Car, r.Car).Y < 0f) continue; //If he's behind, we do not care
                        if (ARS.GetOffset(Car, r.Car).Y > 0f)//If he is ahead
                        {
                            if (spdDiff < 0f && spdDiff < -maxDiff) continue;
                            if (spdDiff > 0f && sToHit > 5f) continue;
                        }
                    }
                    Candidates.Add(r);
                }
            }

            Candidates = Candidates.OrderBy(v => Vector3.Distance(v.Car.Position, refpoint)).ToList();
            int max = 3;
            while (Candidates.Count > max) Candidates.RemoveAt(Candidates.Count - 1);
            referenceVehs = Candidates;


            //if (Aggro) mem.personality.data.Aggression += mem.personality.Rivals.AggressionBuildup; else mem.personality.data.Aggression -= mem.personality.Rivals.AggressionBuildup;

            /*
            if (Candidates.Where(c=>c.Car.IsInRangeOf(Car.Position, 15f)).Count() >= 4) mem.intention.Aggression = 0f;
            else
            {
                if (ARS.DevSettingsFile.GetValue<bool>("RACERS", "UsePersonalities", false))
                {
                    if (mem.intention.Aggression<mem.personality.Rivals.AggressionCap && mem.intention.Aggression < AggroToReach) mem.intention.Aggression += mem.personality.Rivals.AggressionBuildup;
                    if (mem.intention.Aggression > AggroToReach) mem.intention.Aggression -= mem.personality.Rivals.AggressionBuildup;
                }
            }
            */
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
