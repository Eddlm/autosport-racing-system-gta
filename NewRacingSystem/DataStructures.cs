using GTA.Math;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ARS
{

    
        public class AIData
    {
        public float SpeedToInput = ARS.MPHtoMS(5);

        public float MinAngle=0;
        public float MaxAngle=30;

        public float MaxSpeed = ARS.MPHtoMS(270);
        public float MinSpeed = ARS.MPHtoMS(30);

        public float Delta= 0.12f;
        
    }

    /*
     * 
    
        public class AIData
    {
        public float SpeedToInput = ARS.MPHtoMS(12);

        public float MinAngle=0;
        public float MaxAngle=35;

        public float MaxSpeed = ARS.MPHtoMS(400);
        public float MinSpeed = ARS.MPHtoMS(30);

        public float Delta= 0.065f;
        
    }

     */

    //The AI brain, both storing data and intentions
    public class Memory
    {
        public Data data = new Data();
        public class Data
        {
            public float DeviationFromCenter = 0f;
            public float brakeSafety =0f;
            public float SteerAngle=0f;
            public Vector3 SpeedVector = Vector3.Zero;
            public List<Racer> Rivals = new List<Racer>();
        }

        public Intention intention = new Intention();
        public class Intention
        {
            public List<Vector2> Maneuvers = new List<Vector2>(); 
            public float Speed;
            public Vector3 Direction;
            public float DeviationFromCenter;


            //Rivals
            public float Aggression;
            public bool NeedAggressionChange=true;
            public float AggroToReach;
            public float RivalSideDist;
            public float RivalBehindDist;
            public float RivalCornerBonusSpeed;

        }
        public PersonalitySet personality = new PersonalitySet();

    }

    //Inputs applied to achieve the intentions defined above
    public class VehicleControl
    {
            public float SteerAngle = 0f;
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

    //Context aware data, based on a 12m measurements
    public class CornerPoint
    {
        public int Node = 0;
        public float Angle = 0f;
        public float Speed = 20f;
        public float Elevation = 0f;
        public float RelativeElevation = 0f;
    }


    public class AvoidanceData
    {
        public float sToHit=0f;
        public Vector2 relativeDistance= Vector2.Zero;
        public float relativeAngle = 0f;
    }
    public class SkillSet
    {
        public float TractionControlPrecision; //0-100, they misjudge and overcorrect
        public float StabilityControlPrecision; //0-100, they misjudge and overcorrect/undercorrect
        public float BrakeCornerDeltaJudgement; //Faster or slower than ideal
        public float BrakeCornerSpeedJudgement; //Faster or slower than ideal
        public float SafetyAwareness;  //How much to respect common sense when dealing with unsafe situations
    }
    public enum Bravery
    {
        OvertakesInCorner,
        OvertakesApproachingCorner,
        OvertakesWhileCrowded,
        DoesNotLiftOnSlide,
        HandbrakeOnHotEntry
    }

    public class PersonalityRivals
    {
        public float SideToSideMinDist = 0.35f;
        public float BehindRivalMinDistance = 1f;
        public float BehindRivalBrakeDeltaDare = 0f;
        public float AggressionBuildup = 0.01f;
        public float AggressionCap = 0.5f;
    }
    public class PersonalityStability
    {

        public float WheelspinMinSlide = 1f;        
        public float WheelspinMaxSlide = 2f;        

        public float WheelspinOnMinSlide = 0.5f;        
        public float WheelspinOnMaxSlide = 1.5f;

        public float CountersteerSideways = 2f;        
        public float CountersteerMaxOvercorrect = 10f;        
        public float CountersteerMinAngleSideways = 0f;
        public float CountersteerMaxAngleSideways = 2f;

        public float SpinoutSafeRotSpeed = 30f;
        public float SpinoutMaxExtraRotSpeed = 30f;
        public float SpinoutCounterScale = 1f;

    }

    public class PersonalitySet
    {
        public string Name = "Default";
        public PersonalityRivals Rivals = new PersonalityRivals();
        public PersonalityStability Stability = new PersonalityStability();
        public PersonalityStability Mind = new PersonalityStability();

    }
    public class ContextInTrack
    {

        float RealToIntendedRelativeAngle = 0f;
        float RealToIntendedRelativeTCL = 0f; 

    }
}
