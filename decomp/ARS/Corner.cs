namespace ARS;

public class Corner
{
	public Approach Approach = new Approach();

	public float Speed = 0f;

	public CornerPoint OG;

	public bool Valid = true;

	public float sToEntrance;

	public BrakeStabilityStrat stabilityStrat = BrakeStabilityStrat.Invalid;

	public Corner(float speed, CornerPoint oG)
	{
		Speed = speed;
		OG = oG;
	}
}
