using System;
using GTA.Math;

namespace ARS;

public class Rival
{
	public Racer RivalRacer = null;

	public RelativePos relativePos = RelativePos.Unreachable;

	public float Distance = 99f;

	public float sToReach = 99f;

	public float sToRear = 99f;

	public float DirectionDiff = 99f;

	public Vector3 rPos = Vector3.Zero;

	public Vector2 BoundingBoxTotal = Vector2.Zero;

	public float OvertakeLane = 0f;

	public float AvoidStr = 0f;

	public float OccupiedLane = 0f;

	public float OccupiedLaneWidth = 0f;

	public void Update(Racer me)
	{
		relativePos = RelativePos.Unreachable;
		if (RivalRacer != null)
		{
			rPos = ARS.GetOffset(me.Car, RivalRacer.Car);
			BoundingBoxTotal.Y = Math.Abs(me.Car.Model.GetDimensions().Y / 2f + RivalRacer.Car.Model.GetDimensions().Y / 2f);
			BoundingBoxTotal.X = (me.vData.BoundingBox + RivalRacer.vData.BoundingBox) / 2f;
			OccupiedLaneWidth = BoundingBoxTotal.X;
			OccupiedLane = RivalRacer.mem.data.DeviationFromCenter;
			Distance = (me.Car.Position - RivalRacer.Car.Position).Length();
			float num = (float)Math.Round(me.Car.Velocity.Length() - RivalRacer.Car.Velocity.Length(), 4);
			if (num < 0.001f)
			{
				sToReach = 909f;
				sToRear = 909f;
			}
			else
			{
				sToReach = Distance / num;
				sToRear = (Distance - BoundingBoxTotal.Y) / num;
			}
			if (rPos.Y > BoundingBoxTotal.Y)
			{
				relativePos = RelativePos.Ahead;
			}
			else if (rPos.Y < 0f - BoundingBoxTotal.Y)
			{
				relativePos = RelativePos.Behind;
			}
			else if (rPos.X > 0f)
			{
				relativePos = RelativePos.Right;
			}
			else
			{
				relativePos = RelativePos.Left;
			}
			DirectionDiff = Vector3.SignedAngle(me.Car.Velocity, RivalRacer.Car.Velocity, me.Car.UpVector);
		}
	}
}
