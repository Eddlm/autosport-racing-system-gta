namespace ARS;

public static class AIData
{
	public static float MaxSpeed = ARS.MPHtoMS(500f);

	public static float MinSpeed = ARS.MPHtoMS(15f);

	public static float SpeedToInput(float spd, float tSpd, float scale = 1f)
	{
		return (tSpd - spd) / scale;
	}
}
