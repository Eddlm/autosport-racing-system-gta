using GTA.Math;
using System;
using System.Collections.Generic;

namespace ARS
{
    public static class AIData
    {
        public static float MaxSpeed = ARS.MPHtoMS(300f);
        public static float MinSpeed = ARS.MPHtoMS(15f);
    }
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
        public float AvgGroundStability = 1;
        public Vector3 LocalGs = Vector3.Zero;
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
            public Vector3 SpeedVector = Vector3.Zero;
        }

        public Intention intention = new Intention();
        public class Intention
        {
            public float Speed;
            public float MaxSpeed;
            public float IntendedSpdChangeGs;

        }

    }
    public class Rival
    {
        public Racer RivalRacer = null;
        public RelativePos relativePos = RelativePos.Unreachable;

        public float Distance = 99;
       public float sToReach = 99f;
        public float DirectionDiff = 99f;
        public Vector3 rPos = Vector3.Zero;


        public Vector2 BoundingBoxTotal = Vector2.Zero;
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
            if (SpeedDiff <= 0.001f)
            {
                sToReach = 909f;
            }
            else
            {
                sToReach = Distance / SpeedDiff;
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
        public float SteerInput = 0f;
        public float LastAppliedSteerTrackDegrees = 0f;
        public float Throttle = 1f;
        public float Brake = 1f;
        public float MaxThrottle = 1f;
        public float TCSThrottle = 1f;

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
        public float Speed = 999;
        public float Elevation = 0f;
        public float ElevationChange = 0f;
        public bool IsKey = false;
        public float GetRadius() => ARS.TrackPoints[Node].GeneralCurveRadius;
        public float GetPreciseRadius() => ARS.TrackPoints[Node].PreciseCurveRadius;
    }
    public class Corner
    {
        public float Speed = 0f;
        public CornerPoint OG;
        public bool Valid=true;
        public float sToEntrance;
        public Corner  (float speed, CornerPoint oG)
        {
            Speed = speed;
            OG = oG;
        }
    }
}




