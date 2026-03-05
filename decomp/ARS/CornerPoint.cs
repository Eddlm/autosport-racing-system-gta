namespace ARS;

public class CornerPoint
{
	public int Node = 0;

	public float Angle = 0f;

	public int LengthStart = 5;

	public int LenghtEnd = 5;

	public float FullAngle = 0f;

	public float Speed = 999f;

	public float Elevation = 0f;

	public float ElevationChange = 0f;

	public bool IsKey = false;

	public float GetRadius()
	{
		return ARS.TrackPoints[Node].GeneralCurveRadius;
	}

	public float GetPreciseRadius()
	{
		return ARS.TrackPoints[Node].PreciseCurveRadius;
	}
}
