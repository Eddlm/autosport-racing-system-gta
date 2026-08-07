using GTA.Math;
using System;
using System.Collections.Generic;

namespace ARS
{
    public static class AiConstants
    {
        public static float MaxSpeed = ARS.MphToMps(300f);
        public static float MinSpeed = ARS.MphToMps(15f);
    }
    public class VehicleState
    {
        public List<Vector3> AccelerationVector = new List<Vector3> { Vector3.Zero};
        public Vector3 SpeedVectorGlobal = Vector3.Zero;
        public Vector3 SpeedVectorLocal = Vector3.Zero;
        public int WheelBase = 2;
        public float YawRotationPerSecondDegrees = 1f;        
        public float SlideAngle = 22f;
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
        public float LateralTractionCurve = 22f;
        public float Downforce = 1f;
        public float BrakingAbility = 1f;
            public float EstimatedTopSpeed = 1f;
        public float Grip = 1f;
        public float Gravity = 9.8f;
        public float Acceleration = 1;
    }

    public class RacerBrain
    {
        public Perception CurrentPerception = new Perception();
        public List<Rival> Rivals = new List<Rival> { new Rival(), new Rival(), new Rival() };
        public Corner Corner =null;


        public class Perception
        {
            public float DeviationFromCenter = 0f;
            public float CurveRadiusToFollowPoint = 0f;
            // Short-window (0.5s→1.0s) radius for the System 2 high-speed lane pursuit gain.
            // Shorter than CurveRadiusToFollowPoint (0.5s→2.5s) so the inside-edge commitment
            // responds to the imminent corner instead of the long approach.
            public float HighSpeedCurveRadius = 0f;
            // NEVER USED — kept for the future "two conflicting corners" work. Written in Racer.UpdateTrackPosition, never read.
            public float CurveRadiusAfterFollowPoint = 0f;
            public Vector3 SpeedVector = Vector3.Zero;
        }

        public Intention CurrentIntention = new Intention();
        public class Intention
        {
            public float Speed;
            public float MaxSpeed;
            public float IntendedSpeedChangeGs;

        }

    }
    public class Rival
    {
        public Racer RivalRacer = null;
        public RelativePos RelativePosition = RelativePos.Unreachable;

        public float Distance = 99;
        public float SecondsToReach = 99f;
        public float DirectionDiff = 99f;
        public Vector3 RelativeOffset = Vector3.Zero;


        public Vector2 CombinedSize = Vector2.Zero;
        public float OccupiedLane=0f;
        public float OccupiedLaneWidth = 0f;
        public void Update(Racer me)
        {
            RelativePosition = RelativePos.Unreachable;
            if (RivalRacer == null) return;

            RelativeOffset = ARS.EntityRelativeOffset(me.Car, RivalRacer.Car);
            CombinedSize.Y = Math.Abs((me.Car.Model.GetDimensions().Y / 2) + (RivalRacer.Car.Model.GetDimensions().Y / 2)) + 2f;
            CombinedSize.X = (me.VehicleData.BoundingBox + RivalRacer.VehicleData.BoundingBox) / 2;
            OccupiedLaneWidth = CombinedSize.X;
            OccupiedLane = RivalRacer.Brain.CurrentPerception.DeviationFromCenter;
            Distance =  (me.Car.Position -RivalRacer.Car.Position).Length();

            float SpeedDiff = (float)Math.Round(me.Car.Velocity.Length()- RivalRacer.Car.Velocity.Length(), 4);
            if (SpeedDiff <= 0.001f)
            {
                SecondsToReach = 909f;
            }
            else
            {
                SecondsToReach = Distance / SpeedDiff;
            }

            if (RelativeOffset.Y > CombinedSize.Y)
            {
                RelativePosition = RelativePos.Ahead;
            }
            else
            {

                if(RelativeOffset.Y < -CombinedSize.Y)
                {
                    RelativePosition = RelativePos.Behind;
                }
                else
                {
                    if (RelativeOffset.X > 0) RelativePosition = RelativePos.Right;
                    else RelativePosition = RelativePos.Left;
                }
            }

            DirectionDiff = Vector3.SignedAngle(me.Car.Velocity, RivalRacer.Car.Velocity, me.Car.UpVector);
        }
    }


    public class VehicleControl
    {
        public float SteerDegrees = 0f;
        public float SteerInput = 0f;
        public float LastAppliedSteerDegrees = 0f;
        public float Throttle = 1f;
        public float Brake = 1f;
        public float MaxThrottle = 1f;
        public float TCSThrottle = 1f;

        public int HandBrakeTime = 0;

    }

    public class TrackPoint
    {
        public int Node = 0;
        public Vector3 Position = Vector3.Zero;
        public float Angle = 0f;
        public Vector3 Direction = Vector3.Zero;
        public float GeneralCurveRadius = 999f;
        public float PreciseCurveRadius = 999f;
        public float Elevation = 0f;
        public float TrackHalfWidth = 5f;
    }

    public class CornerPoint
    {
        public int Node = 0;
        public float Angle = 0f;
        public int LengthStart = 5;
        public int LengthEnd = 5;
        public float Speed = 999;
        public float Radius = 999f;
        public float Elevation = 0f;
        public float ElevationChange = 0f;
        public bool IsKey = false;
        public float GetRadius() => ARS._trackPoints[Node].GeneralCurveRadius;
        public float GetPreciseRadius() => ARS._trackPoints[Node].PreciseCurveRadius;
    }
    public class Corner
    {
        public float Speed = 0f;
        public CornerPoint Point;
        public float SecondsToEntrance;
        public Corner  (float speed, CornerPoint point)
        {
            Speed = speed;
            Point = point;
        }
    }
    public enum ManeuverType { None, Nitrous, DiveBomb, DefendLane, Yield }
    public class Maneuver
    {
        public ManeuverType Type = ManeuverType.None;
        public bool Active = false;
        public int LastEnabled = 0;
        public Racer Target = null;
    }
}




