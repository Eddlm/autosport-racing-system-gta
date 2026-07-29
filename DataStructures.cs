using GTA.Math;
using System;
using System.Collections.Generic;

namespace ARS
{
    public static class AIData
    {
        public static float MaxSpeed = ARS.MPHtoMS(300f);
        public static float MinSpeed = ARS.MPHtoMS(15f);

        public static float SpeedToInput(float spd, float tSpd, float scale = 1f)
        {
            return (tSpd - spd) / scale;
        }
    }
    public enum BrakeStabilityStrat { Invalid, Checking, Checked }
    public class VehData
    {
        public List<Vector3> AccelerationVector = new List<Vector3> { Vector3.Zero};
        public Vector3 SpeedVectorGlobal = Vector3.Zero;
        public Vector3 SpeedVectorLocal = Vector3.Zero;
        public int WheelBase = 2;
        /// <summary>
        /// </summary>
        public float YawRotationPerSecondDegrees = 1f;        
        public float SlideAngle = 22f;
         /// <summary>
        /// Scales up percieved braking ability to brake as late as possible. 
        /// Hardcoded to reach an equilibrium of spending no more than 0.5s at full brake to approach corners.
        /// </summary>
        public float BaseMechanicalGrip = 1f;
        public float CurrentMechanicalGrip = 1f;
        public float CurrentDownforce = 0f;        
        public float AvgGroundStability = 1;
        

        public float Understeer = 0;
        public Vector3 LocalGs = Vector3.Zero;
        public Vector3 Gs = Vector3.Zero;
        public float LongitudinalGs => LocalGs.Y/9.8f;
        
        
        public float PerformanceIndex = 0;
        public string TextPerformanceIndex = "0";
        public float BoundingBox = 0f;
        public float SteeringLock = 0f;
    }
    public class HandlingData
    {
        public float TRlateral = 22f;
        public float Downforce = 1f;
        public float BrakingAbility = 1f;
        public float TopSpeed = 1f;
        public float Grip = 1f;
        public float Gravity = 9.8f;
        public float Power = 1;
    }

    /// <summary>
    /// The AI brain.
    /// </summary>
    public class Memory
    {
        public Data data = new Data();
        public List<Rival> Rivals = new List<Rival>();
        public Corner Corner =null;
        public Memory()
        {
            Rivals.Add(new Rival());
            Rivals.Add(new Rival());
            Rivals.Add(new Rival());
        }


        public class Data
        {
            public float DeviationFromCenter = 0f;
            public float CurveRadiusToFollowPoint = 0f;
            public TrackPoint FollowPoint = null;
            public float CurveRadiusPhysicalGs = 0f;
            public Vector3 SpeedVector = Vector3.Zero;
        }

        public Intention intention = new Intention();
        public class Intention
        {
            public float Speed;
            public float MaxSpeed;
            public Vector3 Direction;
            public float LookaheadDeviationFromCenter;
            public float DeviationAvoidanceStrength;
            public float IntendedSpdChangeGs;
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

        public float Distance = 99;
       public float sToReach = 99f;
       public float sToRear = 99f;
        public float DirectionDiff = 99f;
        public Vector3 rPos = Vector3.Zero;


        public Vector2 BoundingBoxTotal = Vector2.Zero;
        public float OvertakeLane = 0f;
        public float AvoidStr = 0f;
        public float OccupiedLane=0f;
        public float OccupiedLaneWidth = 0f;
        public void Update(Racer me)
        {
            relativePos = RelativePos.Unreachable;
            if (RivalRacer == null) return;

            rPos = ARS.GetOffset(me.Car, RivalRacer.Car);
            BoundingBoxTotal.Y = Math.Abs((me.Car.Model.GetDimensions().Y / 2) + (RivalRacer.Car.Model.GetDimensions().Y / 2)) + 2f;
            BoundingBoxTotal.X = (me.VehicleData.BoundingBox + RivalRacer.VehicleData.BoundingBox) / 2;
            OccupiedLaneWidth = BoundingBoxTotal.X;
            OccupiedLane = RivalRacer.Brain.data.DeviationFromCenter;
            Distance =  (me.Car.Position -RivalRacer.Car.Position).Length();// me.Car.Position.DistanceTo2D(RivalRacer.Car.Position);// -BoundingBoxTotal.Y;

            float SpeedDiff = (float)Math.Round(me.Car.Velocity.Length()- RivalRacer.Car.Velocity.Length(), 4);
            if (SpeedDiff < 0.001f)
            {
                sToReach = 909f;
                sToRear = 909f;
            }
            else
            {
                sToReach = Distance / SpeedDiff;
                sToRear = (Distance - BoundingBoxTotal.Y) / SpeedDiff;
            }

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
            }

            DirectionDiff = Vector3.SignedAngle(me.Car.Velocity, RivalRacer.Car.Velocity, me.Car.UpVector);
        }
    }


    public class VehicleControl
    {
        public float SteerTrackDegrees = 0f;
        public float SteerStabilityCorrection = 0f;
        public float SteerManeuver = 0f;
        public float SteerMax = 0f;
        public float FollowLane = 0f;
        public float SteerInput = 0f;
        public float LastAppliedSteerTrackDegrees = 0f;
        public float Throttle = 1f;
        public float Brake = 1f;
        public float ThrottleOffset = 0f;
        public float MaxThrottle = 1f;
        public float TCSThrottle = 1f;
        public float CurrentLockupLimiter = 1f;

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
        public float GeneralCurveRadius = 999f;
        public float PreciseCurveRadius = 999f;
        public float Elevation = 0f;
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
        public float Speed = 999;
        public float Elevation = 0f;
        public float ElevationChange = 0f;
        public bool IsKey = false;
        public float GetRadius() => ARS.TrackPoints[Node].GeneralCurveRadius;
        public float GetPreciseRadius() => ARS.TrackPoints[Node].PreciseCurveRadius;
    }
    public class Corner
    {
        public Approach Approach = new Approach();
        public float Speed = 0f;
        public CornerPoint OG;
        public bool Valid=true;
        public float sToEntrance;
        public BrakeStabilityStrat stabilityStrat=BrakeStabilityStrat.Invalid;
        public Corner  (float speed, CornerPoint oG)
        {
            Speed = speed;
            OG = oG;    
        }
    }
     
    public class Approach
    {
        public bool Valid = true;
        public bool CheckedForImpediment = false;
        public float HoldDeviation = 0;

    }

    public class PersonalityRivals
    {
        public float SideToSideMinDist = 1f;
        public float BehindRivalMinDistance = 1f;
        public float BehindRivalBrakeDeltaDare = 0f;
        public float AggressionBuildup = 0.01f;
        public float ManeuverRiskFactor = 0.2f;
    }
    public class PersonalityStability
    {
        //Overdrive
        public float SpeedRiskFactorBase = 1f;
        public float SpeedRiskFactorAggro = 1.2f;
        public float BrakeRiskFactorBase = 1;
        public float BrakeRiskFactorAggro = 1.2f;
        
        public int Skill = 100;
        public float NoSlide = 0.5f;
        public float MaxSlide = 1;

        public float WheelspinOnMinSlide = 0.2f;
        public float WheelspinOnMaxSlide = 0.2f;

        //Slide
        public float MinAlowedSlideToTRLat = 0f;
        public float MaxAllowedSlideToTRLat = 1f;

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




