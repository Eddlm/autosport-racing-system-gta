using GTA.Math;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ARS
{
    public static class AIData
    {
        public static float MaxSpeed = ARS.MPHtoMS(250f);
        public static float MinSpeed = ARS.MPHtoMS(5f);

        public static float SpeedToInput(float spd, float tSpd, float scale=1f)
        {
            return (tSpd-spd) / scale;
        }
    }

    public class VehData
    {
        public List<Vector3> AccelerationVector = new List<Vector3> { Vector3.Zero};
        public Vector3 SpeedVectorGlobal = Vector3.Zero;
        public Vector3 SpeedVectorLocal = Vector3.Zero;
        public float RotSpeedZ = 1f;
        
        public float SlideAngle = 22f;

        public float BaseGrip = 1f;
        public float CurrentGrip = 1f;
        public float CurrentDownforce = 1f;        
        public float AvgGroundStability = 1;
        

        public float Understeer = 0;
        public Vector3 Gs = Vector3.Zero;

        
    }
    public class HandlingData
    {
        public float TRlateral = 22f;
        public float Downforce = 1f;
        public float BrakingAbility = 1f;
        public float TopSpeed = 1f;
    }

    /// <summary>
    /// The AI brain.
    /// </summary>
    public class Memory
    {
        public Data data = new Data();
        public List<Rival> Rivals = new List<Rival>();

        public Memory()
        {
            Rivals.Add(new Rival());
            Rivals.Add(new Rival());
            Rivals.Add(new Rival());
            Rivals.Add(new Rival());
        }
        public float AvoidAvgLane = 0;
        public class Data
        {
            public float oldInlane = 0f;
            public float DeviationFromCenter = 0f;
            public float SteerAngle = 0f;
            public float CurveRadiusToFollowPoint = 0f;
            public float CurveAheadAngle = 0f;
            public float CurveRadiusPhysicalGs = 0f;
            public Vector3 SpeedVector = Vector3.Zero;
        }

        public Intention intention = new Intention();
        public class Intention
        {
            public List<Vector2> Maneuvers = new List<Vector2>();
            public List<Vector2> Corrections = new List<Vector2>();
            public float Speed;
            public Vector3 Direction;
            public float LookaheadDeviationFromCenter;
            public float MaxSteerToDeviation=3;
            public float DeviationAvoidanceStrength;
            //Rivals
            public float Aggression;
            public bool NeedAggressionChange = true;
            public float AggroToReach;

        }
        public PersonalitySet personality = new PersonalitySet();

    }
    public class Rival
    {
        public Racer RivalRacer = null;
        public RelativePos relativePos = RelativePos.Unreachable;
        public float Distance = 0;
       public float sToReach = 0f;
        public float DirectionDiff = 0f;
        public Vector3 rPos = Vector3.Zero;
        Vector3 pVector = Vector3.Zero;
        Vector3 vVector = Vector3.Zero;

        public Vector2 BoundingBoxTotal = Vector2.Zero;
        public float AvoidLane = 0f;
        public float AvoidStr = 0f;
        public void Update(Racer me)
        {
            relativePos = RelativePos.Unreachable;
            if (RivalRacer == null) return;

            rPos = ARS.GetOffset(me.Car, RivalRacer.Car);
            BoundingBoxTotal.Y = Math.Abs((me.Car.Model.GetDimensions().Y / 2) + (RivalRacer.Car.Model.GetDimensions().Y / 2));
            BoundingBoxTotal.X = (me.BoundingBox + RivalRacer.BoundingBox) / 2;

            Distance = me.Car.Position.DistanceTo2D(RivalRacer.Car.Position)-BoundingBoxTotal.Y;

            // Negative=he is faster. Positive=he is slower;
            float SpeedDiff = (float)Math.Round(me.Car.Velocity.Length() - RivalRacer.Car.Velocity.Length(), 4);
            
            sToReach = Distance / SpeedDiff;
            if (float.IsNaN(sToReach) || float.IsInfinity(sToReach) || SpeedDiff < 0) sToReach = 60;

            if (rPos.Y > BoundingBoxTotal.Y)
            {
                relativePos = RelativePos.Ahead;
            }
            else
            {

                if(rPos.Y < -BoundingBoxTotal.Y)
                {
                    relativePos = RelativePos.Behind;
                }
                else
                {
                    if (rPos.X > 0) relativePos = RelativePos.Right;
                    else relativePos = RelativePos.Left;
                }
                //if (ARS.LeftOrRight(RivalRacer.Car.Position, me.Car.Position, me.Car.Velocity.Normalized) > 0) relativePos = RelativePos.Right;

            }


            DirectionDiff = Vector3.SignedAngle(me.Car.Velocity, RivalRacer.Car.Velocity, me.Car.UpVector);
            pVector = RivalRacer.Car.Position - me.Car.Position;
            vVector = RivalRacer.Car.Velocity - me.Car.Velocity;
        }

    }
    //Inputs applied to achieve the intentions defined above
    public class VehicleControl
    {
        public float SteerTrack = 0f;
        public float SteerNoLimit = 0f;
        public float SteerCorrection = 0f;
        public float SteerManeuver = 0f;
        /// <summary>
        /// Actual Steering angle the vehicle is using right noe
        /// </summary>
        public float SteerCurrent = 0f;
        public float SteerInput = 0f;


        /// <summary>
        /// Returns the sum of steering angles stored in this object.
        /// </summary>
        /// <returns>SteerTrack + SteerCorrection + SteerManeuver</returns>
        public float SteerAngle() { return SteerTrack + SteerCorrection + SteerManeuver; }

        public float Throttle = 1f;
        public float MaxThrottle = 1f;
        public float CurrentLockupLimiter = 1f;

        public float Brake = 1f;

        public int HandBrakeTime = 0;


    }

    /// <summary>
    /// Pure, precise data
    /// </summary>
    public class TrackPoint
    {
        public int Node = 0;
        public Vector3 Position = Vector3.Zero;
        public float Angle = 0f;
        public Vector3 Direction = Vector3.Zero;
        public Vector3 AvgDirection = Vector3.Zero;
        public float CurveRadius = 999f;
        public float AvgCurveRadius = 999f;
        public Vector3 BezierMidPoint = Vector3.Zero;
        public float Elevation = 0f;
        public float ElevationChange = 0f;
        public float TrackWide = 5f;
    }

    /// <summary>
    /// Context aware data
    /// </summary>
    public class CornerPoint
    {
        public int Node = 0;
        public float Angle = 0f;
        public int LengthStart = 5;
        public int LenghtEnd = 5;
        public float FullAngle = 0f;
        public float Speed = 500f;
        public float Elevation = 0f;
        public float ElevationChange = 0f;
        public float Radius =0f;
        public bool IsKey = false;

        public int Length => LengthStart + LenghtEnd;
    }


    public class AvoidanceData
    {
        public float sToHit = 0f;
        public Vector2 relativeDistance = Vector2.Zero;
        public float relativeAngle = 0f;
    }


    public class PersonalityRivals
    {
        public float SideToSideMinDist = 1f;
        public float BehindRivalMinDistance = 1f;
        public float BehindRivalBrakeDeltaDare = 0f;
        public float AggressionBuildup = 0.01f;
        public float ManeuverExtraGs = 0.2f;
    }
    public class PersonalityStability
    {
        //Overdrive
        public float OverdriveCalm = -0.2f;
        public float OverdriveAggro = 0.2f;
        public float OverBrakeCalm = -0.2f;
        public float OverBrakeAggro = 0.2f;

        
        public int Skill = 100;
        public float WheelspinNoSlide = 0f;
        public float WheelspinMaxSlide = 0.5f;

        public float WheelspinOnMinSlide = 1f;
        public float WheelspinOnMaxSlide = 0.5f;

        //Slide
        public float MinAlowedSlideToTRLat = 0f;
        public float MaxAllowedSlideToTRLat = 1f;
        public float CounterFactor = 1f;
        public float MaxAbsoluteCounter = 10f;

        //Spinouts
        public float SpinoutSafeRotSpeed = 30f;
        public float SpinoutMaxExtraRotSpeed = 30f;
        public float SpinoutMaxCounterDegrees = 45f;

        public float SpinoutUnsafeExtraRotSpeed = 40f;
        public float SpinoutMaxWheelspinAtExtra = 0;

        public float UndersteerFactor = 0f;
    }

    public class PersonalitySet
    {
        public string Name = "Default";
        public int ProbToUse = 25;
        public string SkillRange = "80,100";
        public string Model = "";
        public PersonalityRivals Rivals = new PersonalityRivals();
        public PersonalityStability Stability = new PersonalityStability();

    }
}
