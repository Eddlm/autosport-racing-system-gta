using GTA.Math;

namespace ARS;

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
