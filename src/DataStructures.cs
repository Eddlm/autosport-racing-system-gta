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
        public const int AccelWindow = 10;
        public Vector3[] AccelerationVector = new Vector3[AccelWindow];
        public int AccelHead = 0;
        public int AccelCount = 0;
        public Vector3 AccelSum = Vector3.Zero;
        public Vector3 SpeedVectorGlobal = Vector3.Zero;
        public Vector3 SpeedVectorLocal = Vector3.Zero;
        public int WheelBase = 2;
        public float YawRotationPerSecondDegrees = 1f;
        public float SlideAngle = 22f;
        public float BaseMechanicalGrip = 1f;
        public float DownforceGripBonus = 0f;
        public float CurrentMechanicalGrip = 1f;
        public float AvgGroundStability = 1;

        public Vector3 AverageAcceleration => AccelCount > 0 ? AccelSum / AccelCount : Vector3.Zero;

        public float GetLongitudinalGs(Vector3 forward)
        {
            Vector3 horizontalForward = new Vector3(forward.X, forward.Y, 0f);
            if (horizontalForward.LengthSquared() < 0.0001f) return 0f;
            horizontalForward.Normalize();
            return Vector3.Dot(AverageAcceleration, horizontalForward) / 9.8f;
        }

        public float GetLateralGs(Vector3 forward)
        {
            Vector3 horizontalForward = new Vector3(forward.X, forward.Y, 0f);
            if (horizontalForward.LengthSquared() < 0.0001f) return 0f;
            horizontalForward.Normalize();
            Vector3 right = Vector3.Cross(horizontalForward, Vector3.WorldUp).Normalized;
            return Vector3.Dot(AverageAcceleration, right) / 9.8f;
        }
        
        
        public float PerformanceIndex = 0;
        public float PowerScale = 0;
        public string TextPerformanceIndex = "0";
        public float BoundingBox = 0f;
        public Vector3 ModelDimensions = Vector3.Zero;
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
        // The ahead rival this racer is currently reacting to for avoidance. Set during
        // RivalInfoUpdate(); steering code consumes it without re-scanning the rival list.
        public Rival AvoidanceTarget = null;
        public Corner Corner =null;


        public class Perception
        {
            public float DeviationFromCenter = 0f;
            public float CurveRadiusToFollowPoint = 0f;
            // Short-window radius for the high-speed lane pursuit gain.
            public float HighSpeedCurveRadius = 0f;
            // NEVER USED. Kept for future "two conflicting corners" work.
            public float CurveRadiusAfterFollowPoint = 0f;
            public Vector3 SpeedVector = Vector3.Zero;
        }

        public Intention CurrentIntention = new Intention();
        public class Intention
        {
            public float Speed;
            public float MaxSpeed;
            public float IntendedSpeedChange;

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

        // Decomposed proximity (in me's local frame, SHVDN convention: +X right, +Y forward).
        public float LongitudinalGap = 99f;   // signed: + = rival ahead, - = rival behind
        public float LateralGap = 0f;          // signed: + = rival right, - = rival left
        public float ForwardSpeedGap = 0f;      // signed: + = me faster than rival (along me's forward axis)
        public float TimeToContact = float.PositiveInfinity; // longitudinal-only, forward rivals
        public float SecondsToHit = float.PositiveInfinity;    // physical swept bounding-box hit time
        public float FrontGap = float.PositiveInfinity;        // front-to-rear distance to rival

        public Vector2 CombinedSize = Vector2.Zero;
        public float OccupiedLane=0f;
        public float OccupiedLaneWidth = 0f;
        public void Update(Racer me)
        {
            RelativePosition = RelativePos.Unreachable;
            if (RivalRacer == null) return;

            RelativeOffset = ARS.EntityRelativeOffset(me.Car, RivalRacer.Car);
            LongitudinalGap = RelativeOffset.Y;
            LateralGap = RelativeOffset.X;
            // More margin from a rival ahead (+1m) than behind (+0.25m).
            float yBuffer = RelativeOffset.Y >= 0f ? 1f : 0.25f;
            CombinedSize.Y = Math.Abs((me.VehicleData.ModelDimensions.Y / 2) + (RivalRacer.VehicleData.ModelDimensions.Y / 2)) + yBuffer;
            CombinedSize.X = (me.VehicleData.BoundingBox + RivalRacer.VehicleData.BoundingBox) / 2;
            OccupiedLaneWidth = CombinedSize.X;
            OccupiedLane = RivalRacer.Brain.CurrentPerception.DeviationFromCenter;
            Distance =  (me.Car.Position -RivalRacer.Car.Position).Length();

            // Forward speed gap: project relative velocity onto me's forward axis.
            // Positive = me faster than rival; negative = rival faster.
            Vector3 meForward = me.Car.Velocity.LengthSquared() > 0.01f
                ? me.Car.Velocity.Normalized
                : me.Car.ForwardVector;
            Vector3 relativeVelocity = me.Car.Velocity - RivalRacer.Car.Velocity;
            ForwardSpeedGap = Vector3.Dot(relativeVelocity, meForward);

            // SecondsToReach and TimeToContact use longitudinal gap, not Euclidean.
            // Legacy SecondsToReach kept for existing consumers (avoidance filter expects 0..3 range).
            float longitudinalAbs = Math.Abs(LongitudinalGap);
            float absoluteSpeedGap = (float)Math.Round(me.Car.Velocity.Length() - RivalRacer.Car.Velocity.Length(), 4);
            if (absoluteSpeedGap <= 0.001f)
            {
                SecondsToReach = float.PositiveInfinity;
            }
            else
            {
                SecondsToReach = longitudinalAbs / Math.Abs(absoluteSpeedGap);
            }

            // TimeToContact: only meaningful for rivals ahead of me that I'm closing on.
            if (LongitudinalGap > 0f && ForwardSpeedGap > 0.001f)
            {
                TimeToContact = LongitudinalGap / ForwardSpeedGap;
            }
            else
            {
                TimeToContact = float.PositiveInfinity;
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

            SecondsToHit = ComputeSecondsToHit(me);

            // DirectionDiff: angle between velocity vectors. Guard against zero velocity (NaN).
            float meSpeed = me.Car.Velocity.LengthSquared();
            float rivalSpeed = RivalRacer.Car.Velocity.LengthSquared();
            if (meSpeed < 0.01f || rivalSpeed < 0.01f)
            {
                DirectionDiff = 0f;
            }
            else
            {
                DirectionDiff = Vector3.SignedAngle(me.Car.Velocity, RivalRacer.Car.Velocity, me.Car.UpVector);
            }
        }

        float ComputeSecondsToHit(Racer me)
        {
            float mySpeed = me.Car.Velocity.Length();
            if (mySpeed < 0.1f) return float.PositiveInfinity;

            Vector3 velDir = me.Car.Velocity.Normalized;
            Vector3 toRival = RivalRacer.Car.Position - me.Car.Position;
            if (RelativePosition != RelativePos.Ahead) return float.PositiveInfinity;

            float longGap = Vector3.Dot(toRival, velDir);
            float lateralOffset = (toRival - velDir * longGap).Length();
            if (lateralOffset > 2f) return float.PositiveInfinity;

            float rivalLongSpeed = Vector3.Dot(RivalRacer.Car.Velocity, velDir);
            float closingLong = mySpeed - rivalLongSpeed;
            if (closingLong <= 0.001f) return float.PositiveInfinity;

            float myHalfLen = me.VehicleData.ModelDimensions.Y * 0.5f;
            float rivalHalfLen = RivalRacer.VehicleData.ModelDimensions.Y * 0.5f;
            FrontGap = longGap - (myHalfLen + rivalHalfLen);
            if (FrontGap <= 2f) return 0f;

            return FrontGap / closingLong;
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
        public float MaxThrottleFromTCS = 1f;

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
        public int StartNode = -1;
        public int EndNode = -1;
        public int LengthStart = 5;
        public int LengthEnd = 5;
        public float Speed = 999;
        public float Elevation = 0f;
        public float ElevationChange = 0f;
        public bool IsKey = false;
        // True when a lifting lip sits before the apex. Car must brake before the lip.
        public bool RequiresEarlyBrake = false;
        public int RampEndNode = -1;
        // Vertical curvature Gs at the apex (negative = crest, positive = dip).
        public float CrestGs = 0f;
        // The corner radius from its region limits. Used for apex-speed calculation.
        public float SupposedRadius = 999f;
        public float GetRadius() => ARS.TrackPoints[Node].GeneralCurveRadius;
        public float GetPreciseRadius() => ARS.TrackPoints[Node].PreciseCurveRadius;
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
    public class TrackStartInfo
    {
        public string TrackPath;
        public Vector3 StartPosition;
        public Vector3 JoinPosition;
    }
    public enum ManeuverType { None, Nitrous, DiveBomb, DefendLane, Yield }
    public class Maneuver
    {
        public ManeuverType Type = ManeuverType.None;
        public int LastEnabled = 0;
        public Racer Target = null;
    }
}




