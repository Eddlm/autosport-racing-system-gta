namespace ARS;

public class VehicleControl
{
	public float SteerTrackDegrees = 0f;

	public float SteerStabilityCorrection = 0f;

	public float SteerManeuver = 0f;

	public float SteerMax = 0f;

	public float FollowLane = 0f;

	public float SteerCurrent = 0f;

	public float SteerInput = 0f;

	public float Throttle = 1f;

	public float Brake = 1f;

	public float ThrottleOffset = 0f;

	public float MaxThrottle = 1f;

	public float TCSThrottle = 1f;

	public float CurrentLockupLimiter = 1f;

	public int HandBrakeTime = 0;
}
