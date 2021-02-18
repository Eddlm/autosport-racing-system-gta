using GTA.Math;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ARS
{
    public enum Context
    {
        OutOfTrack, Unstable
    }

    public class AIData
    {
        public float SpeedToInput = ARS.MPHtoMS(20); 

        public float MinAngle = 0;
        public float MaxAngle = 30;
        public float MaxSpeed = ARS.MPHtoMS(250);
        public float MinSpeed = ARS.MPHtoMS(10f);

        public float Delta = 0.12f;

    }
    public class VehData
    {
        public List<Vector3> AccelerationVector = new List<Vector3>();
        public Vector3 SpeedVectorGlobal = Vector3.Zero;
        public Vector3 SpeedVectorLocal = Vector3.Zero;
        public Vector3 RotationVectorLocal = Vector3.Zero;
        public float GsAcceleration = 0f;
        public float SlideAngle = 0f;
        public float DownforceGrip = 0f;
        public float WheelsGrip = 0f;
        public float AvgGroundStability = 1;
    }
    public class HandlingData
    {
        public float TRlateral = 22f;
        public float Downforce = 1f;
        public float BrakingAbility = 1f;
    }

    //The AI brain, both storing data and intentions
    public class Memory
    {
        public Data data = new Data();
        public class Data
        {
            public float oldInlane = 0f;
            public float DeviationFromCenter = 0f;
            public float SteerAngle = 0f;
            public Vector3 SpeedVector = Vector3.Zero;
            public List<Racer> Rivals = new List<Racer>();
        }

        public Intention intention = new Intention();
        public class Intention
        {
            public List<Vector2> Maneuvers = new List<Vector2>();
            public List<Vector2> Corrections = new List<Vector2>();
            public float Speed;
            public Vector3 Direction;
            public float LookaheadDeviationFromCenter;

            //Rivals
            public float Aggression;
            public bool NeedAggressionChange = true;
            public float AggroToReach;

        }
        public PersonalitySet personality = new PersonalitySet();

    }

    //Inputs applied to achieve the intentions defined above
    public class VehicleControl
    {
        public float SteerAngle = 0f;
        public float SteerInput = 0f;
        public float Throttle = 1f;
        public float Brake = 1f;
        public int HandBrakeTime = 0;
    }

    //Pure, precise data
    public class TrackPoint
    {
        public int Node = 0;
        public Vector3 Position = Vector3.Zero;
        public float Angle = 0f;
        public Vector3 Direction = Vector3.Zero;
        public float Elevation = 0f;
        public float RelativeElevation = 0f;
        public float TrackWide = 5f;
    }

    //Context aware data, based on a 10m long measurements
    public class CornerPoint
    {
        public int Node = 0;
        public float Angle = 0f;
        public float AvgAngle = 0f;
        public float Speed = 500f;
        public float Elevation = 0f;
        public float ElevationChange = 0f;
        public float Radius =0f;
        public float AvgRadius =0f;
        public float AvgScale = 10f;
        public float AvgElChange = 0f;
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
        public float CorrAtFullslide = 1f;
        public float MaxFullInput = 10f;

        //Spinouts
        public float SpinoutSafeRotSpeed = 20f;
        public float SpinoutMaxExtraRotSpeed = 80f;
        public float SpinoutMaxCounterDegrees = 20f;

        public float SpinoutUnsafeExtraRotSpeed = 40f;
        public float SpinoutMaxWheelspinAtExtra = 0;
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
