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
        public const int AccelIntervalMs = 20;
        public Vector3[] AccelSamples = new Vector3[AccelWindow];
        public int AccelHead = 0;
        public int AccelCount = 0;
        public int LastAccelSampleTime = 0;
        public Vector3 SpeedVectorLocal = Vector3.Zero;
        public float WheelBase = 2.7f;
        public float YawRotationPerSecondDegrees = 1f;
        public float SlideAngle = 22f;
        public float BaseMechanicalGrip = 1f;
        public float DownforceGripBonus = 0f;
        public float CurrentMechanicalGrip = 1f;

        // Overspeed debug — filled by the overspeed detector, read by the debug panel.
        public float OverspeedMeasuredGs;
        public float OverspeedWheelGs;
        public float OverspeedExcessGs;

        public Vector3 AverageAcceleration
        {
            get
            {
                if (AccelCount == 0) return Vector3.Zero;
                Vector3 sum = Vector3.Zero;
                for (int i = 0; i < AccelCount; i++) sum += AccelSamples[i];
                return sum / AccelCount;
            }
        }

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

        // Per-lap peaks ride the raw 20 ms sample, not the 200 ms mean; the G side is invalid past grip × margin, speed is uncapped.
        public const float LapPeakGripMargin = 1.33f;
        public float PeakAccelG;
        public float PeakDecelG;
        public float PeakLateralG;
        public float PeakTopSpeedMps;

        public void AccumulateLapPeaks(Vector3 accel, Vector3 velocity, Vector3 forward)
        {
            Vector3 horizontalForward = new Vector3(forward.X, forward.Y, 0f);
            if (horizontalForward.LengthSquared() < 0.0001f) return;
            horizontalForward.Normalize();
            float forwardSpeed = Vector3.Dot(new Vector3(velocity.X, velocity.Y, 0f), horizontalForward);
            if (forwardSpeed > PeakTopSpeedMps) PeakTopSpeedMps = forwardSpeed;

            float ceilingG = CurrentMechanicalGrip * LapPeakGripMargin;
            if (!(ceilingG > 0f)) return;
            float longitudinalG = Vector3.Dot(accel, horizontalForward) / 9.8f;
            Vector3 right = Vector3.Cross(horizontalForward, Vector3.WorldUp).Normalized;
            float lateralG = Math.Abs(Vector3.Dot(accel, right) / 9.8f);
            if (Math.Abs(longitudinalG) > ceilingG || lateralG > ceilingG) return;
            if (longitudinalG > PeakAccelG) PeakAccelG = longitudinalG;
            if (longitudinalG < PeakDecelG) PeakDecelG = longitudinalG;
            if (lateralG > PeakLateralG) PeakLateralG = lateralG;
        }

        public void ResetLapPeaks()
        {
            PeakAccelG = 0f;
            PeakDecelG = 0f;
            PeakLateralG = 0f;
            PeakTopSpeedMps = 0f;
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
        // The ahead rival this racer is reacting to for avoidance; UpdateRivalInfo sets it, steering reads it.
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
        }

        public Intention CurrentIntention = new Intention();
        public class Intention
        {
            public float Speed;
            public float MaxSpeed;
            public float IntendedSpeedChange;
            // Physics-limited cornering speed for the current track radius: v = √(g × grip × r)
            public float CorneringSpeedLimit;
            // Max speed for the current steering input before exceeding available grip: v = √(grip × g × L / tan(δ))
            public float SteerLimitedSpeed;

        }

    }
    public class Rival
    {
        public Racer RivalRacer = null;
        public RelativePos RelativePosition = RelativePos.Unreachable;

        public float Distance = 99;
        public float DirectionDiff = 99f;
        public Vector3 RelativeOffset = Vector3.Zero;

        // Decomposed proximity (in me's local frame, SHVDN convention: +X right, +Y forward).
        public float LongitudinalGap = 99f;   // signed: + = rival ahead, - = rival behind
        public float LateralGap = 0f;          // signed: + = rival right, - = rival left
        public float ForwardSpeedGap = 0f;      // signed: + = me faster than rival (along me's forward axis)
        public float TimeToContact = float.PositiveInfinity; // longitudinal-only, forward rivals
        public float TimeToReach = float.PositiveInfinity;     // seconds to close the route gap while closing on it
        public float FrontGap = float.PositiveInfinity;        // front-to-rear distance along the route
        public float RouteGapAhead = 0f;                       // signed route arc in metres, + = rival ahead

        public Vector2 CombinedSize = Vector2.Zero;
        public float OccupiedLane=0f;
        public float OccupiedLaneWidth = 0f;
        // A rival below this speed has no meaningful direction of travel; it is treated as facing this car's way.
        const float SlowRivalMps = 1f;
        public void Update(Racer me)
        {
            RelativePosition = RelativePos.Unreachable;
            if (RivalRacer == null) return;

            // One read each of the two entity vectors, handed to the parts that need them: every use below
            // used to call the native again.
            Vector3 myVelocity = me.Car.Velocity;
            Vector3 rivalVelocity = RivalRacer.Car.Velocity;

            UpdateOffsets(me);
            UpdateSpeedGaps(me, myVelocity, rivalVelocity);
            ClassifyRelativePosition();
            UpdateClosingTime(me, myVelocity, rivalVelocity);
        }

        // Where the rival sits relative to me, and how much room the pair takes up.
        void UpdateOffsets(Racer me)
        {
            Vector3 myPosition = me.Car.Position;
            Vector3 rivalPosition = RivalRacer.Car.Position;

            RelativeOffset = ARS.EntityRelativeOffset(me.Car, RivalRacer.Car);
            LongitudinalGap = RelativeOffset.Y;
            LateralGap = RelativeOffset.X;
            // More margin from a rival ahead (+1m) than behind (+0.25m).
            float yBuffer = RelativeOffset.Y >= 0f ? 1f : 0.25f;
            CombinedSize.Y = Math.Abs((me.VehicleData.ModelDimensions.Y / 2) + (RivalRacer.VehicleData.ModelDimensions.Y / 2)) + yBuffer;
            CombinedSize.X = (me.VehicleData.BoundingBox + RivalRacer.VehicleData.BoundingBox) / 2;
            OccupiedLaneWidth = CombinedSize.X;
            OccupiedLane = RivalRacer.Brain.CurrentPerception.DeviationFromCenter;
            Distance = (myPosition - rivalPosition).Length();
            UpdateRouteGap(me);
        }

        // Signed arc to the rival along the route, shortest way round on a circuit. CumulativeDistance is metres and
        // carries no lap, so a car a lap ahead but physically close still measures close.
        void UpdateRouteGap(Racer me)
        {
            float gap = RivalRacer.CurrentTrackPoint.CumulativeDistance - me.CurrentTrackPoint.CumulativeDistance;
            if (!ARS.IsPointToPoint && ARS.RouteLengthMeters > 0f && Math.Abs(gap) > ARS.RouteLengthMeters * 0.5f)
                gap -= Math.Sign(gap) * ARS.RouteLengthMeters;
            RouteGapAhead = gap;
        }

        // Speed gaps and the times they imply. ForwardSpeedGap projects the relative velocity onto my forward axis,
        // while TimeToContact uses the longitudinal gap.
        void UpdateSpeedGaps(Racer me, Vector3 myVelocity, Vector3 rivalVelocity)
        {
            float mySpeedSquared = myVelocity.LengthSquared();
            Vector3 meForward = mySpeedSquared > 0.01f ? myVelocity.Normalized : me.Car.ForwardVector;
            Vector3 relativeVelocity = myVelocity - rivalVelocity;
            ForwardSpeedGap = Vector3.Dot(relativeVelocity, meForward);

            // TimeToContact: only meaningful for rivals ahead of me that I'm closing on.
            if (LongitudinalGap > 0f && ForwardSpeedGap > 0.001f)
            {
                TimeToContact = LongitudinalGap / ForwardSpeedGap;
            }
            else
            {
                TimeToContact = float.PositiveInfinity;
            }
        }

        // Ahead, behind, or alongside: the longitudinal gap against the pair's combined length.
        void ClassifyRelativePosition()
        {
            if (RelativeOffset.Y > CombinedSize.Y)
            {
                RelativePosition = RelativePos.Ahead;
            }
            else if (RelativeOffset.Y < -CombinedSize.Y)
            {
                RelativePosition = RelativePos.Behind;
            }
            else
            {
                if (RelativeOffset.X > 0) RelativePosition = RelativePos.Right;
                else RelativePosition = RelativePos.Left;
            }
        }

        void UpdateClosingTime(Racer me, Vector3 myVelocity, Vector3 rivalVelocity)
        {
            TimeToReach = ComputeTimeToReach(me);

            // DirectionDiff: angle between velocity vectors, so a car that is stopped or barely rolling has no
            // direction of travel to read. Assume it points the way this car does, or the avoidance cannot see it.
            float mySpeedSquared = myVelocity.LengthSquared();
            if (mySpeedSquared < 0.01f || rivalVelocity.Length() < SlowRivalMps)
            {
                DirectionDiff = 0f;
            }
            else
            {
                DirectionDiff = Vector3.SignedAngle(myVelocity, rivalVelocity, me.Car.UpVector);
            }
        }

        // The gap and the closure both live in the route frame, so no straight axis can be laid across a corner to
        // distort them: a rival on the racing line keeps its lane however the road bends under it.
        float ComputeTimeToReach(Racer me)
        {
            FrontGap = float.PositiveInfinity;
            if (RouteGapAhead <= 0f) return float.PositiveInfinity;
            if (Math.Abs(OccupiedLane - me.Brain.CurrentPerception.DeviationFromCenter) > CombinedSize.X) return float.PositiveInfinity;

            FrontGap = RouteGapAhead - CombinedSize.Y;
            float closingAlong = me.AlongTrackSpeed - RivalRacer.AlongTrackSpeed;
            if (closingAlong <= 0.001f) return float.PositiveInfinity;
            if (FrontGap <= 2f) return 0f;

            return FrontGap / closingAlong;
        }
    }


    // One sample of the applied pedal input every half metre of travel, for the debug trail.
    public struct InputTrailSample
    {
        public Vector3 Position;
        public float Input;
    }

    public enum ThrottleReason
    {
        Plan,
        Tcs,
        Overspeed,
        Rival,
        ChillOut,
        Yield,
        Offtrack,
        GridWait,
        Countersteer,
        StuckRecovery,
        Instability
    }

    public enum BrakeReason
    {
        Plan,
        Abs,
        Countersteer,
        Offtrack,
        GridWait,
        StuckRecovery
    }

    public enum StuckPhase
    {
        None,
        Reverse,
        Drive
    }

    public class VehicleControl
    {
        public float SteerDegrees = 0f;
        public float SteerInput = 0f;
        public float LastAppliedSteerDegrees = 0f;
        public float Throttle = 1f;
        public float Brake = 1f;
        public float MaxThrottle = 1f;
        public float MaxBrake = 1f;
        public float MaxThrottleFromTCS = 1f;
        public float MaxBrakeFromABS = 1f;
        public float MaxBrakeFromCountersteer = 1f;
        public float MaxThrottleFromOverspeed = 1f;
        public float MaxThrottleFromRival = 1f;
        public float MaxThrottleFromChillOut = 1f;
        public float MaxThrottleFromYield = 1f;
        public float MaxThrottleFromInstability = 1f;
        public float MaxThrottleFromOffTrack = 1f;
        public ThrottleReason ThrottleReason = ThrottleReason.Plan;
        public float ThrottleReasonLevel = 1f;
        public BrakeReason BrakeReason = BrakeReason.Plan;
        public float BrakeReasonLevel = 1f;

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
        public float ExactRadius = 999f;
        public float Elevation = 0f;
        public float TrackHalfWidth = 5f;
        public float CumulativeDistance = 0f;
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
        // Vertical curvature Gs at the crest before this corner (negative = crest, positive = dip), measured at
        // generation at a fixed probe speed and rescaled by (v/probe)² at read. CrestNode is the unload's peak and
        // CrestEntryNode the onset the car meets first; both are -1 when there is no crest.
        public float CrestGs = 0f;
        public int CrestNode = -1;
        public int CrestEntryNode = -1;
        public int CrestSpanNodes = 0;
        // Part of a chicane: two close corners with opposite curvature signs.
        public bool IsChicane = false;
        // The corner radius from its region limits. Used for apex-speed calculation.
        public float SupposedRadius = 999f;
        // The region's tightest smoothed radius: what registration compares, and a far less noisy read than the
        // min-precise SupposedRadius.
        public float DetectedRadius = 999f;
        public float GetRadius() => ARS.TrackPoints[Node].GeneralCurveRadius;
        public float GetPreciseRadius() => ARS.TrackPoints[Node].PreciseCurveRadius;
    }
    // A lifting lip: the route rises over a short run and then drops away. Route geometry only for now -- nothing
    // drives on it, and the lane-local case needs the raycast instrument that reads the real surface.
    public class Bump
    {
        public int LipNode = -1;
        public int RiseStartNode = -1;
        public float DepartureGrade = 0f;
        public float LipCurvature = 0f;
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
    // Per-racer corner context: the shared corner plus what this car knows about it. Lives on the racer, never on
    // CornerPoint, because every racer holds a reference to the same table entry.
    public class CornerContext
    {
        public CornerPoint Point;
        public float BrakeFactor = 0f;
        public bool RequiresBraking = false;
        public bool RequiresPositioning = false;
    }
    public class TrackStartInfo
    {
        public string TrackPath;
        public Vector3 StartPosition;
        public Vector3 JoinPosition;
    }
    public enum ManeuverType { None, DiveBomb, DefendLane, Yield, ChillOut }
    public class Maneuver
    {
        public ManeuverType Type = ManeuverType.None;
        public int LastEnabled = 0;
        public Racer Target = null;
    }
}




