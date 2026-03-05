using System;
using GTA;

namespace ARS;

public class PID
{
	private float value;

	private float target;

	private float velocity;

	private float maxSpeed;

	private float maxAccel;

	private float kP;

	private float kD;

	private float kI;

	private float integral;

	private int lastTick = Game.GameTime;

	public PID(float kP = 1f, float kD = 0.1f, float kI = 0f, float maxSpeed = 99f, float maxAccel = 99f)
	{
		this.kP = kP;
		this.kD = kD;
		this.kI = kI;
		this.maxSpeed = maxSpeed;
		this.maxAccel = maxAccel;
		value = 0f;
		target = 0f;
		velocity = 0f;
		integral = 0f;
	}

	public float Update()
	{
		float num = (float)(Game.GameTime - lastTick) * 0.01f;
		lastTick = Game.GameTime;
		if ((double)num <= 0.0)
		{
			return value;
		}
		float num2 = target - value;
		float num3 = num2 * kP;
		float num4 = (0f - velocity) * kD;
		integral += num2 * num;
		float num5 = integral * kI;
		float num6 = num3 + num4 + num5;
		if (Math.Abs(num6) > maxAccel)
		{
			num6 = maxAccel * (float)((num6 > 0f) ? 1 : (-1));
		}
		velocity += num6 * num;
		if (Math.Abs(velocity) > maxSpeed)
		{
			velocity = maxSpeed * (float)((velocity > 0f) ? 1 : (-1));
		}
		value += velocity * num;
		if (float.IsNaN(value) || ((double)Math.Abs(num2) < 0.1 && Math.Abs(velocity) < 0.1f))
		{
			value = target;
			velocity = 0f;
		}
		return value;
	}

	public void SetTarget(float target, float maxVal = 90f)
	{
		this.target = ARS.Clamp(target, 0f - maxVal, maxVal);
	}

	public void SetValue(float value)
	{
		this.value = value;
		velocity = 0f;
		integral = 0f;
	}

	public float GetValue()
	{
		return value;
	}

	public float GetTarget()
	{
		return target;
	}

	public void ModifyTarget(float delta, float maxVal = 20f)
	{
		target = ARS.Clamp(target + delta, 0f - maxVal, maxVal);
	}

	public bool IsClose(float threshold = 0.01f)
	{
		return Math.Abs(target - value) < threshold;
	}
}
