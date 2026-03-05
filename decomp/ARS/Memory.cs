using System.Collections.Generic;
using GTA.Math;

namespace ARS;

public class Memory
{
	public class Data
	{
		public float DeviationFromCenter = 0f;

		public float CurveRadiusToFollowPoint = 0f;

		public float CurveRadiusPhysicalGs = 0f;

		public Vector3 SpeedVector = Vector3.Zero;
	}

	public class Intention
	{
		public float Speed;

		public float MaxSpeed;

		public Vector3 Direction;

		public float LookaheadDeviationFromCenter;

		public float DeviationAvoidanceStrength;

		public float IntendedSpdChangeGs;

		public float Aggression;

		public bool NeedAggressionChange = true;

		public float AggroToReach;
	}

	public Data data = new Data();

	public List<Rival> Rivals = new List<Rival>();

	public Corner Corner = null;

	public Intention intention = new Intention();

	public PersonalitySet personality = new PersonalitySet();

	public Memory()
	{
		Rivals.Add(new Rival());
		Rivals.Add(new Rival());
		Rivals.Add(new Rival());
	}
}
