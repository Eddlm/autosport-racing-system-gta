using System.Collections.Generic;
using GTA.Math;

namespace ARS;

public class VehData
{
	public List<Vector3> AccelerationVector = new List<Vector3> { Vector3.Zero };

	public Vector3 SpeedVectorGlobal = Vector3.Zero;

	public Vector3 SpeedVectorLocal = Vector3.Zero;

	public int WheelBase = 2;

	public float YawRotationPerSecondDegrees = 1f;

	public float SlideAngle = 22f;

	public float BaseMechanicalGrip = 1f;

	public float CurrentMechanicalGrip = 1f;

	public float CurrentDownforce = 0f;

	public float AvgGroundStability = 1f;

	public float Understeer = 0f;

	public Vector3 LocalGs = Vector3.Zero;

	public Vector3 Gs = Vector3.Zero;

	public float PerformanceIndex = 0f;

	public string TextPerformanceIndex = "0";

	public float BoundingBox = 0f;

	public float SteeringLock = 0f;

	public float LongitudinalGs => LocalGs.Y / 9.8f;
}
