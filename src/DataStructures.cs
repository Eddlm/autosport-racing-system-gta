// Shared types: constants, vehicle state, racer brain.
using GTA.Math;
using System;
using System.Collections.Generic;

namespace ARS
{
    public static class AiConstants
    {

        public static float MaxIntendedSpeedMps = ARS.MphToMps(999f);
        public static float MinIntendedSpeedMps = ARS.MphToMps(10f);
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
        public float WheelBaseMetres = 2.7f;
        public float YawRotationPerSecondDegrees = 1f;
        public float SlideAngle = 22f;
        public float BaseMechanicalGrip = 1f;
        public float DownforceGripBonusGs = 0f;
        public float CurrentMechanicalGrip = 1f;

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

        public float PaceIndex = 0;
        public string TextPerformanceIndex = "0";
        public float SlidingWidthMetres = 0f;
        public Vector3 ModelDimensions = Vector3.Zero;
        public float SteeringLock = 0f;
    }
    public class HandlingData
    {
        public float LateralTractionCurveDegrees = 22f;
        public float Downforce = 1f;
        public float BrakingAbility = 1f;
            public float EstimatedTopSpeedMps = 1f;
        public float Grip = 1f;
        public float Gravity = 9.8f;
        public float Acceleration = 1;
    }

    public class RacerBrain
    {
        public Perception CurrentPerception = new Perception();
        public List<Rival> Rivals = new List<Rival> { new Rival(), new Rival(), new Rival() };

        public Rival AvoidanceTarget = null;
        public Corner Corner =null;

        public class Perception
        {
            public float CurrentLaneOffsetMetres = 0f;
            public float CurveRadiusToFollowPoint = 0f;

            public float HighSpeedCurveRadius = 0f;

        }

        public Intention CurrentIntention = new Intention();
        public class Intention
        {
            public float Speed;
            public SpeedTerm Term = SpeedTerm.None;
            public int TermNode = -1;
            public float TermMetres;
            public float TermRadius;
            public float IntendedSpeedChange;

            public float SteerLimitedSpeed;

        }

    }
    public class Rival
    {
        public Racer RivalRacer = null;
        public RelativePos RelativePosition = RelativePos.Unreachable;

        public float Distance = 99;
        public float DirectionDiffDegrees = 99f;
        public Vector3 RelativeOffset = Vector3.Zero;

        public float LongitudinalGap = 99f;
        public float LateralGap = 0f;
        public float ForwardSpeedGap = 0f;
        public float Speed;
        public float TimeToReachSeconds = float.PositiveInfinity;
        public float FrontGapMetres = float.PositiveInfinity;
        public float RouteGapAhead = 0f;

        public Vector2 CombinedSize = Vector2.Zero;

        public float NodeOffset=0f;
        public float CombinedWidth = 0f;
        const float SlowRivalMps = 1f;
        public void Update(Racer me)
        {
            RelativePosition = RelativePos.Unreachable;
            if (RivalRacer == null) return;

            Vector3 myVelocity = me.Car.Velocity;
            Vector3 rivalVelocity = RivalRacer.Car.Velocity;
            Speed = rivalVelocity.Length();

            UpdateOffsets(me);
            UpdateSpeedGaps(me, myVelocity, rivalVelocity);
            ClassifyRelativePosition();
            UpdateClosingTime(me, myVelocity, rivalVelocity);
        }

        void UpdateOffsets(Racer me)
        {
            Vector3 myPosition = me.Car.Position;
            Vector3 rivalPosition = RivalRacer.Car.Position;

            RelativeOffset = ARS.EntityRelativeOffset(me.Car, RivalRacer.Car);
            LongitudinalGap = RelativeOffset.Y;
            LateralGap = RelativeOffset.X;

            float yBuffer = RelativeOffset.Y >= 0f ? 1f : 0.25f;
            CombinedSize.Y = Math.Abs((me.VehicleData.ModelDimensions.Y / 2) + (RivalRacer.VehicleData.ModelDimensions.Y / 2)) + yBuffer;
            CombinedSize.X = (me.VehicleData.SlidingWidthMetres + RivalRacer.VehicleData.SlidingWidthMetres) / 2;
            CombinedWidth = CombinedSize.X;
            NodeOffset = RivalRacer.Brain.CurrentPerception.CurrentLaneOffsetMetres;
            Distance = (myPosition - rivalPosition).Length();
            UpdateRouteGap(me);
        }

        public void UpdateRouteGap(Racer me)
        {
            float gap = RivalRacer.CurrentTrackPoint.CumulativeDistance - me.CurrentTrackPoint.CumulativeDistance;
            if (!ARS.IsPointToPoint && ARS.RouteLengthMeters > 0f && Math.Abs(gap) > ARS.RouteLengthMeters * 0.5f)
                gap -= Math.Sign(gap) * ARS.RouteLengthMeters;
            RouteGapAhead = gap;
        }

        void UpdateSpeedGaps(Racer me, Vector3 myVelocity, Vector3 rivalVelocity)
        {
            float mySpeedSquared = myVelocity.LengthSquared();
            Vector3 meForward = mySpeedSquared > 0.01f ? myVelocity.Normalized : me.Car.ForwardVector;
            Vector3 relativeVelocity = myVelocity - rivalVelocity;
            ForwardSpeedGap = Vector3.Dot(relativeVelocity, meForward);
        }

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
            TimeToReachSeconds = ComputeTimeToReach(me);

            float mySpeedSquared = myVelocity.LengthSquared();
            if (mySpeedSquared < 0.01f || rivalVelocity.Length() < SlowRivalMps)
            {
                DirectionDiffDegrees = 0f;
            }
            else
            {

                DirectionDiffDegrees = Vector3.SignedAngle(new Vector3(myVelocity.X, myVelocity.Y, 0f), new Vector3(rivalVelocity.X, rivalVelocity.Y, 0f), me.Car.UpVector);
            }
        }

        float ComputeTimeToReach(Racer me)
        {
            FrontGapMetres = float.PositiveInfinity;
            if (RouteGapAhead <= 0f) return float.PositiveInfinity;
            if (Math.Abs(NodeOffset - me.Brain.CurrentPerception.CurrentLaneOffsetMetres) > CombinedSize.X) return float.PositiveInfinity;

            FrontGapMetres = RouteGapAhead - CombinedSize.Y;
            float closingAlong = me.AlongTrackSpeed - RivalRacer.AlongTrackSpeed;
            if (closingAlong <= 0.001f) return float.PositiveInfinity;
            if (FrontGapMetres <= 2f) return 0f;

            return FrontGapMetres / closingAlong;
        }
    }

    public struct InputTrailSample
    {
        public Vector3 Position;
        public float Input;
        public bool Chevron;
    }

    public enum SpeedTerm
    {
        None,
        GridWait,
        Finished,
        Corner,
        Route,
        Standoff,
        Chicane,
        Forced
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
        Stability,
        Kick
    }

    public enum BrakeReason
    {
        Plan,
        Abs,
        Countersteer,
        Slide,
        RouteSpeed,

        Projection,
        GridWait,
        StuckRecovery
    }

    public enum LaneReason
    {
        OwnLane,
        CurveLean,
        CommitCard,
        Avoidance,
        RecoveryEdge,
        WallClamp,
        DebugLock
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
        public float MaxBrakeFromABS = 1f;
        public float MaxBrakeFromCountersteer = 1f;
        public float MaxBrakeFromSlide = 1f;
        public float MaxThrottleFromOverspeed = 1f;
        public float MaxThrottleFromRival = 1f;
        public float MaxThrottleFromChillOut = 1f;
        public float MaxThrottleFromYield = 1f;
        public float MaxThrottleFromStability = 1f;
        public float MaxThrottleFromOffTrack = 1f;
        public float MaxThrottleFromRecovery = 1f;
        public float MaxThrottleFromTCS = 1f;
        public ThrottleReason ThrottleReason = ThrottleReason.Plan;
        public float ThrottleReasonCap = 1f;

        public float ThrottleDemand = 0f;
        public bool ThrottleLimited = false;
        public BrakeReason BrakeReason = BrakeReason.Plan;
        public float BrakeReasonCap = 1f;

        public int HandBrakeUntilMs = 0;
    }

    public class TrackPoint
    {
        public int Node = 0;
        public Vector3 Position = Vector3.Zero;
        public float Angle = 0f;
        public Vector3 Direction = Vector3.Zero;
        public float GeneralCurveRadius = 999f;
        public float PreciseCurveRadius = 999f;
        public float SweepCurveRadius = 999f;
        public float ExactRadius = 999f;
        public float SlopeSineTimes90 = 0f;
        public float CrestDipRadius = 0f;   // vertical radius at this node, crest positive, 0 = no vertical bend
        public float TrackHalfWidth = 5f;
        public float CumulativeDistance = 0f;
        public float LaneLineShare;
        public bool LaneShaped;
        public int LaneShapeApex = -1;
        public int LaneShapeStart = -1;
        public int LaneShapeExit = -1;

        public float SurfaceGrip = 1f;
    }

    public class CornerPoint
    {
        public int Node = 0;
        public float Angle = 0f;
        public int StartNode = -1;
        public int EndNode = -1;
        public int EntranceNodes = 5;
        public int ExitNodes = 5;
        public float Speed = 999;
        public float SlopeSineTimes90 = 0f;
        public bool IsChicane = false;
        public float SupposedRadius = 999f;
        public float DetectedRadius = 999f;
        public float WidestRadius = 0f;

        public float SurfaceGrip = 1f;

        public float GetRadius() => ARS.TrackPoints[Node].GeneralCurveRadius;
        public float GetPreciseRadius() => ARS.TrackPoints[Node].PreciseCurveRadius;
    }

    public class Bump
    {
        public int LipNode = -1;
        public int RiseStartNode = -1;
        public float DepartureGrade = 0f;
        public float LipCrestRadius = 0f;   // vertical radius at the lip, metres: the onset rung over the grade drop
    }

    // A crest as a feature: one contiguous stretch of crest-signed nodes with its peak, its extent and the radius that
    // peak reads. Static per track — nothing here depends on the car — and derived from TrackPoint.CrestDipRadius, so
    // elevation still has one author. LengthMeters spans FirstNode to LastNode (the hop out of the region is not
    // counted) while NodeCount counts both ends; a region crossing the index seam has FirstNode > LastNode and Wraps set.
    public class CrestRegion
    {
        public int FirstNode = -1;
        public int PeakNode = -1;
        public int LastNode = -1;
        public bool Wraps = false;
        public int NodeCount = 0;
        public float LengthMeters = 0f;
        public float Radius = 0f;
    }
    public class Corner
    {
        public float Speed = 0f;
        public CornerPoint Point;
        public Corner  (float speed, CornerPoint point)
        {
            Speed = speed;
            Point = point;
        }
    }

    public class CornerContext
    {
        // The seed every initialiser and fallback derives from: one value owns all of them.
        public const float BrakeFactorSeed = 0.6f;
        // Where the ramp aims to have arrived at corner speed, in nodes behind the apex. The brake factor learner
        // settles the aggression first; once it stops moving, this one moves the window earlier or later.
        public const float ApexSpeedBufferSeed = 15f;
        public CornerPoint Point;
        public float BrakeFactor = BrakeFactorSeed;
        public float ApexSpeedBuffer = ApexSpeedBufferSeed;
        // The menu's Corner Speed Offset is where every corner STARTS, not a second additive layered on the
        // learned value: one seed for all of them, and the corner speed learner moves each from there.
        public static float SeedSpeedOffsetMps => ARS.MphToMps(ARS.CornerOffsetMph);
        public float CornerSpeedOffsetMps = SeedSpeedOffsetMps;
        public bool RequiresBraking = false;
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
        public int EnabledAtMs = 0;
        public Racer Target = null;
    }

    public class SmoothFloat
    {
        public float Target;
        public float Value;
        readonly float _ratePerSecond;

        public SmoothFloat(float initial, float ratePerSecond)
        {
            Target = Value = initial;
            _ratePerSecond = ratePerSecond;
        }

        public void Advance(float dtSeconds)
        {
            Value += (Target - Value) * (1f - (float)Math.Exp(-_ratePerSecond * dtSeconds));
        }
    }
}
