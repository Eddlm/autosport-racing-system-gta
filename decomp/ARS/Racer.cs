using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using GTA;
using GTA.Math;
using GTA.Native;

namespace ARS;

public class Racer
{
	public enum eLookAheads
	{
		SteerRef,
		QuarterSec,
		HalfSec,
		ThreeQuarterSec,
		OneSec,
		OneHalfSec,
		SteerInRef
	}

	public Team team = Team.None;

	public VehicleControl vControl = new VehicleControl();

	public Memory mem = new Memory();

	public bool ControlledByPlayer = false;

	public Dictionary<RandomVariance, float> BehaviorVariance = new Dictionary<RandomVariance, float>();

	public List<TimeSpan> LapTimes = new List<TimeSpan>();

	public int LapStartTime = 0;

	public string Name = "Racer";

	public Ped Driver;

	public Vehicle Car;

	public List<string> DebugText = new List<string>();

	public List<Vector3> trail = new List<Vector3>();

	public Vector3 LastStuckPlace = Vector3.Zero;

	public RacerBaseBehavior BaseBehavior = RacerBaseBehavior.GridWait;

	public RaceState RCStatus = RaceState.NotInitiated;

	private float TorqueMult = 1f;

	public int Lap = 0;

	public int RacePosition = 0;

	public bool CanRegisterNewLap = true;

	private int HalfSecondTick = 0;

	private int OneSecondTick = 0;

	private PID SteerPID = new PID(6f, 12f);

	private PID LanePID = new PID(0.2f, 0.5f);

	private int StuckGameTimeRef = 0;

	public bool StuckRecover = false;

	private int GameTimeOutOfTrack = 0;

	public float GroundGripMultiplier = 1f;

	public VehData vData = new VehData();

	public HandlingData Handling = new HandlingData();

	public bool FinishedPointToPoint = false;

	public int localSPDLimiter = 0;

	public TrackPoint CurrentTrackPoint = new TrackPoint();

	public Vector3 SteerTarget = Vector3.Zero;

	public Dictionary<eLookAheads, TrackPoint> LookAheads = new Dictionary<eLookAheads, TrackPoint>();

	public List<Vehicle> Traffic = new List<Vehicle>();

	private Random Random = new Random();

	private Vector3 lSpeed;

	private float TargetLane = 0f;

	public float MaxLeftLane = 0f;

	public float MaxRightLane = 0f;

	private float DEVGripExtra = 0f;

	private int LastCoreTick = 100;

	private int TimeSinceLastCoreTick => (int)ARS.Clamp(Game.GameTime - LastCoreTick, 1f, 9999f);

	private float TickScale => 0.001f * (float)TimeSinceLastCoreTick;

	public Racer(Vehicle RacerCar, Ped RacerPed)
	{
		Car = RacerCar;
		Driver = RacerPed;
		Name = RacerCar.FriendlyName;
		if (Name == "NULL" || Name == null)
		{
			Name = Car.DisplayName.ToString()[0].ToString().ToUpper() + Car.DisplayName.ToString().Substring(1).ToLowerInvariant();
		}
		if (Driver.IsPlayer)
		{
			ControlledByPlayer = true;
		}
		HalfSecondTick = Game.GameTime + ARS.GetRandomInt(10, 50);
		if (!ControlledByPlayer)
		{
			Driver.BlockPermanentEvents = true;
			Driver.AlwaysKeepTask = true;
			Function.Call(Hash._0xB195FFA8042FC5C3, Driver, 0f);
			Function.Call(Hash._0xA731F608CA104E3C, Driver, 0f);
			if (ARS.DevSettingsFile.GetValue("RACERS", "AIRacerAutofix", 1) == 2)
			{
				Function.Call(Hash._0xFAEE099C6F890BB8, Car, true, true, true, true, true, true, true, true);
				Function.Call(Hash._0xFAEE099C6F890BB8, Driver, true, true, true, true, true, true, true, true);
				Car.IsInvincible = true;
				Car.IsCollisionProof = true;
				Car.IsOnlyDamagedByPlayer = true;
				Function.Call(Hash._0x3E8C8727991A8A0B, Car, true);
				Function.Call(Hash._0x92F0CF722BC4202F, Car, true);
				Car.EngineCanDegrade = false;
			}
			else if (ARS.DevSettingsFile.GetValue("RACERS", "AIRacerAutofix", 1) == 1)
			{
				Function.Call(Hash._0x3E8C8727991A8A0B, Car, true);
				Function.Call(Hash._0x92F0CF722BC4202F, Car, true);
				Car.EngineCanDegrade = false;
			}
			else
			{
				Car.IsInvincible = false;
				Car.IsCollisionProof = false;
			}
			Car.EngineRunning = true;
			Driver.SetIntoVehicle(Car, VehicleSeat.Driver);
			ARS.SetSteerAngle(Car, 0.5f);
			ARS.SetThrottle(Car, 0f);
			ARS.SetBrakes(Car, 0f);
			Car.IsRadioEnabled = false;
		}
		Car.IsPersistent = true;
		if ((Car.CurrentBlip == null || !Car.CurrentBlip.Exists()) && !Driver.IsPlayer)
		{
			Car.AddBlip();
			Car.CurrentBlip.Color = BlipColor.Blue;
			Car.CurrentBlip.Scale = 0.75f;
			Function.Call(Hash._0x5FBCA48327B914DF, Car.CurrentBlip, true);
			Function.Call(Hash._0x2B6D467DAB714E8D, Car.CurrentBlip, true);
			Car.CurrentBlip.Name = Name;
		}
		Function.Call(Hash._0x0DC7CABAB1E9B67E, Car, true, 1);
		Function.Call(Hash._0x0DC7CABAB1E9B67E, Driver, true, 1);
		Function.Call(Hash._0xFAEE099C6F890BB8, Driver, true, true, true, false, true, true, 1, true);
		Driver.MaxHealth = 1000;
		Driver.Health = 1000;
		Driver.CanSufferCriticalHits = false;
		if (Car.ClassType == VehicleClass.Emergency)
		{
			team = Team.Cop;
		}
		Random random = new Random();
		foreach (RandomVariance item in Enum.GetValues(typeof(RandomVariance)).Cast<RandomVariance>())
		{
			float num = random.Next(80, 120);
			num *= 0.01f;
			BehaviorVariance.Add(item, num);
		}
	}

	public void Initialize()
	{
		Handling.Downforce = ARS.GetDownforce(Car);
		if (Handling.Downforce > 100f)
		{
			Handling.Downforce *= 0.1f;
		}
		Handling.TRlateral = ARS.rad2deg(ARS.GetTRCurveLat(Car));
		if (Handling.TRlateral < 1f || Handling.TRlateral > 100f)
		{
			Handling.TRlateral = 22f;
		}
		ARS.Log(ARS.LogImportance.Info, "TRlat for " + Car.DisplayName + ":" + Handling.TRlateral + "º");
		Handling.BrakingAbility = Car.MaxBraking;
		Handling.TopSpeed = ARS.EngineTopSpeed(Car);
		Handling.Power = Function.Call<float>(Hash._0x5DD35C8D074E57AE, new InputArgument[1] { Car });
		vData.SteeringLock = ARS.rad2deg(ARS.GetSteerLock(Car));
		if (vData.SteeringLock < 1f || vData.SteeringLock > 100f)
		{
			vData.SteeringLock = 40f;
		}
		ARS.Log(ARS.LogImportance.Info, "Steerlock for " + Car.DisplayName + ":" + vData.SteeringLock + "º");
		vControl.SteerTrackDegrees = 0f;
		CurrentTrackPoint = ARS.TrackPoints.Last();
		vControl.Brake = 0f;
		vControl.Throttle = 0f;
		LapTimes.Clear();
		LapStartTime = 0;
		Lap = 0;
		string value = ARS.GetHandlingFlags(Car).ToString("X");
		int num = Convert.ToInt32(value, 16);
		if ((num & 0x800000) != 0 || (num & 0x200000) != 0)
		{
			Handling.Gravity *= 1.2f;
		}
		BaseBehavior = RacerBaseBehavior.GridWait;
		FinishedPointToPoint = false;
		Handling.Grip = Function.Call<float>(Hash._0xA132FB5370554DB0, new InputArgument[1] { Car }) * (Handling.Gravity / 9.8f);
		vData.PerformanceIndex = (int)(Handling.TopSpeed * 5f + Handling.Grip * 100f + Handling.Power * 500f);
		vData.TextPerformanceIndex = (int)((double)Handling.TopSpeed / 1.2) + " | " + (int)(Handling.Power * 200f) + " | " + (int)(Handling.Grip * 20f);
		Car.Repair();
	}

	private float AngleToTrackDir(TrackPoint first, TrackPoint second)
	{
		float num = Vector3.SignedAngle(first.Direction, second.Direction, Vector3.WorldUp);
		if (float.IsNaN(num))
		{
			num = 0f;
		}
		return num;
	}

	public void SteerTrack()
	{
		if (LookAheads[eLookAheads.OneSec] == null || BaseBehavior == RacerBaseBehavior.GridWait || BaseBehavior == RacerBaseBehavior.FinishedStandStill || CurrentTrackPoint.Node < 3)
		{
			vControl.SteerTrackDegrees = 0f;
			return;
		}
		float trackWide = LookAheads[eLookAheads.HalfSec].TrackWide;
		float num = 0f;
		float num2 = 0f;
		float x = AngleToTrackDir(CurrentTrackPoint, LookAheads[eLookAheads.OneSec]);
		MaxLeftLane = 0f - trackWide + vData.BoundingBox;
		MaxRightLane = trackWide - vData.BoundingBox;
		float num3 = AngleToTrackDir(CurrentTrackPoint, LookAheads[eLookAheads.OneHalfSec]);
		TargetLane = mem.data.DeviationFromCenter - num3 / Handling.Grip;
		if (mem.Corner.Valid && Lap > 0 && Car.Velocity.Length() > mem.Corner.Speed * 0.9f)
		{
			CornerPoint oG = mem.Corner.OG;
			float num4 = (ARS.TrackPoints[oG.Node].Position - ARS.TrackPoints[CurrentTrackPoint.Node].Position).Length();
			float num5 = (ARS.TrackPoints[oG.Node - oG.LengthStart].Position - ARS.TrackPoints[CurrentTrackPoint.Node].Position).Length();
			float num6 = ARS.Clamp(num5 / Car.Velocity.Length(), 0f, 99f);
			float num7 = ARS.Clamp(num4 / Car.Velocity.Length(), 0f, 99f);
			mem.Corner.sToEntrance = num6;
			if (num6 < 5f)
			{
				float num8 = DistToInside(ARS.TrackPoints[oG.Node].Direction) / (CurrentTrackPoint.TrackWide * 2f);
				num = ARS.TrackPoints[oG.Node].TrackWide * (float)((oG.Angle > 0f) ? 1 : (-1));
				num2 = 0f - num;
				float num9 = ARS.TrackPoints[oG.Node].TrackWide * 0.4f;
				float num10 = ARS.map(Car.Velocity.Length() - mem.Corner.Speed, -15f, -5f, 0f, 1f, clamp: true);
				if (num7 < 5f && num7 > num9 * num8)
				{
					TargetLane = num * num10;
				}
			}
		}
		bool flag = true;
		float num11 = 0f;
		if (Car.Velocity.Length() > 10f)
		{
			foreach (Rival item in mem.Rivals.OrderByDescending((Rival r) => r.Distance))
			{
				if (item.RivalRacer == null)
				{
					continue;
				}
				item.OvertakeLane = 0f;
				float num12 = item.rPos.Y - item.BoundingBoxTotal.Y;
				bool flag2 = Math.Abs(item.rPos.Y) < item.BoundingBoxTotal.Y - 0.1f;
				bool flag3 = num12 > 0f;
				bool flag4 = Math.Abs(item.DirectionDiff) < 15f && item.sToRear < 3f;
				bool flag5 = Math.Abs(item.rPos.X) < vData.BoundingBox;
				bool flag6 = flag2 || (flag4 && mem.Corner.Valid && mem.Corner.sToEntrance < 2f);
				if (flag3 && Math.Abs(item.rPos.X) < item.BoundingBoxTotal.X)
				{
					mem.intention.MaxSpeed = item.RivalRacer.Car.Velocity.Length() + ARS.map(num12, 1f, 5f, 0f, 30f, clamp: true);
				}
				if (flag2 && item.rPos.X > 0f == item.DirectionDiff > 0f)
				{
					float num13 = ARS.map(Math.Abs(item.rPos.X), item.BoundingBoxTotal.X + 2f, item.BoundingBoxTotal.X, 0f, 1f, clamp: true);
					num11 = item.DirectionDiff * num13;
					num11 *= ARS.map(item.rPos.Y, 0f - item.BoundingBoxTotal.Y, 0f, 0.5f, 1f, clamp: true);
				}
				if (!flag2 && vData.LongitudinalGs > 0.05f && Math.Abs(item.DirectionDiff) < 20f && flag3 && (!mem.Corner.Valid || mem.Corner.sToEntrance > 3f))
				{
					float num14 = item.RivalRacer.mem.data.DeviationFromCenter - mem.data.DeviationFromCenter;
					if (Math.Abs(num14) < vData.BoundingBox + 1f)
					{
						float num15 = ARS.Clamp((vData.BoundingBox + 1f - Math.Abs(num14)) * (float)((num14 < 0f) ? 1 : (-1)), -10f, 10f);
						if (num15 > 0f == mem.data.DeviationFromCenter > 0f && CurrentTrackPoint.TrackWide - Math.Abs(mem.data.DeviationFromCenter) < vData.BoundingBox + 1f)
						{
							num15 *= -0.5f;
						}
						float val = ARS.map(num12, 5f, 2.5f, 0f, 1f, clamp: true);
						float val2 = ARS.map(item.sToRear, 3f, 1f, 0f, 1f, clamp: true);
						TargetLane += num15 * Math.Max(val, val2);
					}
				}
				if (!flag6)
				{
					continue;
				}
				float value = AngleToTrackDir(CurrentTrackPoint, item.RivalRacer.CurrentTrackPoint);
				if (item.rPos.X > 0f)
				{
					float num16 = ARS.map(x, -45f, 0f, -1f, 1f, clamp: true);
					float num17 = item.OccupiedLane - item.BoundingBoxTotal.X - num16;
					if (num17 < MaxRightLane)
					{
						MaxRightLane = num17;
						if (!flag2 && Math.Abs(value) < 10f && MaxRightLane < 0f - trackWide)
						{
							MaxRightLane = item.OccupiedLane;
						}
					}
					continue;
				}
				float num18 = ARS.map(x, 45f, 0f, -1f, 1f, clamp: true);
				float num19 = item.OccupiedLane + item.BoundingBoxTotal.X + num18;
				if (num19 > MaxLeftLane)
				{
					MaxLeftLane = num19;
					if (!flag2 && Math.Abs(value) < 10f && MaxLeftLane > trackWide)
					{
						MaxLeftLane = item.OccupiedLane;
					}
				}
			}
		}
		TargetLane = ARS.Clamp(TargetLane, MaxLeftLane, MaxRightLane);
		vControl.FollowLane = TargetLane;
		TrackPoint trackPoint = LookAheads[eLookAheads.SteerRef];
		LanePID.SetTarget(vControl.FollowLane);
		SteerTarget = trackPoint.Position - Vector3.Cross(Vector3.WorldUp, trackPoint.Direction) * LanePID.GetValue();
		vControl.SteerTrackDegrees = 0f - Vector3.SignedAngle((SteerTarget - Car.Position).Normalized, Car.Velocity.Normalized, Vector3.WorldUp);
		if (float.IsNaN(vControl.SteerTrackDegrees) || float.IsInfinity(vControl.SteerTrackDegrees))
		{
			vControl.SteerTrackDegrees = 0f;
		}
	}

	private void SteerCorrections()
	{
		float num = Car.Velocity.Length();
		float rad = (float)((double)(vData.BaseMechanicalGrip * Handling.Gravity * (float)vData.WheelBase) / Math.Pow(Car.Velocity.Length() + 0.01f, 2.0));
		rad = Math.Max(ARS.rad2deg(rad), 3f);
		vControl.SteerTrackDegrees *= ARS.map(Math.Abs(vControl.SteerTrackDegrees), 0f, 45f, 0.25f, 1f, clamp: true);
		if (Math.Sign(vControl.SteerTrackDegrees) != Math.Sign(vData.YawRotationPerSecondDegrees))
		{
			vControl.SteerTrackDegrees *= 0.5f;
		}
		vControl.SteerTrackDegrees = ARS.Clamp(vControl.SteerTrackDegrees, 0f - rad, rad);
		float value = ARS.rad2deg((float)((double)Car.Velocity.Length() * Math.Tan(ARS.deg2rad(vControl.SteerTrackDegrees))) / (float)vData.WheelBase);
		float yawRotationPerSecondDegrees = vData.YawRotationPerSecondDegrees;
		float num2 = Math.Abs(yawRotationPerSecondDegrees) - Math.Abs(value);
		if (Math.Sign(value) == Math.Sign(yawRotationPerSecondDegrees) || Math.Abs(value) >= 0.01f)
		{
		}
		bool flag = num2 > 0f;
		bool flag2 = num2 < -0f;
		if (flag)
		{
			float num3 = num2 * 0.8f * (float)Math.Sign(vData.YawRotationPerSecondDegrees);
			vControl.SteerTrackDegrees -= num3;
		}
		float num4 = (float)vData.WheelBase / (float)Math.Tan(Math.Abs(ARS.deg2rad(vControl.SteerTrackDegrees)));
		float num5 = (float)Math.Pow(num, 0.9900000095367432) / vData.BaseMechanicalGrip;
		float num6 = ARS.rad2deg((float)Math.Atan((float)vData.WheelBase / num5));
		float num7 = vData.SlideAngle * ARS.map(Math.Abs(vData.SlideAngle), 0f, Handling.TRlateral * 1.2f, 0.5f, 1.2f, clamp: true);
		if (Math.Sign((int)vData.SlideAngle) == Math.Sign((int)vData.YawRotationPerSecondDegrees))
		{
			vControl.SteerTrackDegrees -= num7;
		}
		SteerPID.SetTarget(vControl.SteerTrackDegrees);
	}

	private float DistToOutside(Vector3 direction)
	{
		if (Vector3.SignedAngle(CurrentTrackPoint.Direction, direction, Vector3.WorldUp) < 0f)
		{
			return (float)Math.Round(CurrentTrackPoint.TrackWide + mem.data.DeviationFromCenter, 1);
		}
		return (float)Math.Round(CurrentTrackPoint.TrackWide - mem.data.DeviationFromCenter, 1);
	}

	private float DistToInside(Vector3 direction)
	{
		if (Vector3.SignedAngle(CurrentTrackPoint.Direction, direction, Vector3.WorldUp) > 0f)
		{
			return (float)Math.Round(CurrentTrackPoint.TrackWide + mem.data.DeviationFromCenter, 1);
		}
		return (float)Math.Round(CurrentTrackPoint.TrackWide - mem.data.DeviationFromCenter, 1);
	}

	public void Launch()
	{
		foreach (Racer racer in ARS.Racers)
		{
			racer.mem.Corner = new Corner(5f, ARS.CornerPoints.FirstOrDefault((CornerPoint c) => c.IsKey));
		}
		mem.Corner.Valid = false;
		vData.AvgGroundStability = 1f;
		StuckRecover = false;
		StuckGameTimeRef = 0;
		BaseBehavior = RacerBaseBehavior.Race;
		LapStartTime = Game.GameTime;
		vControl.HandBrakeTime = Game.GameTime + ARS.GetRandomInt(100, 400);
		vControl.MaxThrottle = 1f;
		if (team == Team.Cop)
		{
			Car.SirenActive = true;
		}
	}

	private void SpeedToThrottleBrake()
	{
		float num = 0f;
		float num2 = 0f;
		if (BaseBehavior == RacerBaseBehavior.GridWait)
		{
			mem.intention.Speed = 99f;
			vControl.HandBrakeTime = Game.GameTime + 500;
		}
		mem.intention.Speed = Math.Min(mem.intention.Speed, ARS.EngineTopSpeed(Car) * 1.3f);
		mem.intention.Speed = Math.Min(mem.intention.Speed, mem.intention.MaxSpeed);
		if (Game.GameTime - LapStartTime < 3000)
		{
			mem.intention.IntendedSpdChangeGs = 999f;
		}
		else
		{
			mem.intention.IntendedSpdChangeGs = (mem.intention.Speed - Car.Velocity.Length()) / 9.8f;
		}
		num = ((!(mem.intention.IntendedSpdChangeGs > 0f)) ? 0f : ARS.Clamp(mem.intention.IntendedSpdChangeGs * 2f, 0f, Math.Min(vControl.TCSThrottle, 1f)));
		num2 = ((!(mem.intention.IntendedSpdChangeGs < -0f)) ? 0f : ARS.Clamp((0f - mem.intention.IntendedSpdChangeGs) * 2f, 0f, 1f));
		if ((double)num2 > 0.0)
		{
			num = 0f;
		}
		if ((double)num > 0.0)
		{
			num2 = 0f;
		}
		vControl.MaxThrottle = Math.Min(vControl.MaxThrottle, vData.AvgGroundStability);
		vControl.Brake += (num2 - vControl.Brake) * 5f * TickScale;
		vControl.Throttle += (num - vControl.Throttle) * 5f * TickScale;
		vControl.Throttle = Math.Min(vControl.Throttle, vControl.MaxThrottle);
		if (vControl.MaxThrottle < 1f)
		{
			vControl.MaxThrottle += 2f * TickScale;
		}
		if (mem.intention.MaxSpeed < AIData.MaxSpeed)
		{
			mem.intention.MaxSpeed += 15f * TickScale;
		}
	}

	private void TranslateSteer()
	{
		if (float.IsNaN(vControl.SteerTrackDegrees) || float.IsInfinity(vControl.SteerTrackDegrees))
		{
			vControl.SteerTrackDegrees = 0f;
		}
		if (float.IsNaN(vControl.SteerManeuver) || float.IsInfinity(vControl.SteerManeuver))
		{
			vControl.SteerManeuver = 0f;
		}
		float value = SteerPID.GetValue();
		if (float.IsNaN(vControl.SteerInput) || float.IsInfinity(vControl.SteerInput))
		{
			vControl.SteerInput = 0f;
		}
		vControl.SteerInput = ARS.map(value, 0f - vData.SteeringLock, vData.SteeringLock, -1f, 1f, clamp: true);
		vControl.SteerCurrent = value;
	}

	public void SpeedTrack()
	{
		if (BaseBehavior == RacerBaseBehavior.GridWait)
		{
			mem.intention.Speed = 200f;
			return;
		}
		if (BaseBehavior == RacerBaseBehavior.FinishedRace)
		{
			mem.intention.Speed = 20f;
			return;
		}
		if (BaseBehavior == RacerBaseBehavior.FinishedStandStill)
		{
			mem.intention.Speed = 0f;
			return;
		}
		mem.intention.Speed = AIData.MaxSpeed;
		float num = 999f;
		float num2 = 999f;
		if (mem.Corner.Valid)
		{
			num = Math.Max(2f, ARS.MapIdealSpeedForDistance(mem.Corner.OG, this) * RiskFactorForGrip());
		}
		num2 = (float)Math.Sqrt(vData.CurrentMechanicalGrip * Handling.Gravity * mem.data.CurveRadiusToFollowPoint) * RiskFactorForGrip();
		float value = ARS.LeftOrRight(Car.Position + vData.Gs / 9.8f * 2f, Car.Position, Car.ForwardVector);
		if (CurrentTrackPoint.GeneralCurveRadius < 2000f && Math.Abs(value) > vData.CurrentMechanicalGrip * 0.5f)
		{
			float num3 = 0f;
			Vector3 pos = Car.Position + (Car.Velocity + vData.Gs * ARS.TracjectoryProjectionSeconds) * ARS.TracjectoryProjectionSeconds;
			int num4 = CurrentTrackPoint.Node + (int)(Car.Velocity.Length() * ARS.TracjectoryProjectionSeconds);
			if (num4 > 100 && num4 < ARS.TrackPoints.Last().Node - 100)
			{
				TrackPoint trackPoint = ARS.TrackPoints[num4];
				float num5 = ARS.LeftOrRight(pos, trackPoint.Position, trackPoint.Direction);
				if (CurrentTrackPoint.Angle > 0f)
				{
					if (num5 < 0f)
					{
						num3 = ARS.map(num5, 0f, 0f - trackPoint.TrackWide, 0f, 1f, clamp: true) * TickScale;
					}
					else
					{
						num3 = ARS.map(num5, trackPoint.TrackWide * 2f, 0f, -1f, 0f, clamp: true) * TickScale;
					}
				}
				else if (num5 > 0f)
				{
					num3 = ARS.map(num5, 0f, trackPoint.TrackWide, 0f, 1f, clamp: true) * TickScale;
				}
				else
				{
					num3 = ARS.map(num5, (0f - trackPoint.TrackWide) * 2f, 0f, -1f, 0f, clamp: true) * TickScale;
				}
			}
		}
		if (num <= 5f)
		{
			num = ARS.GetSpeedForCorner(mem.Corner.OG, this);
		}
		if (num2 < 1f)
		{
			num2 = 1f;
		}
		mem.intention.Speed = Math.Min(num * 1.1f, num2 * 1.1f);
		if (mem.Corner.Valid)
		{
			float value2 = Vector3.SignedAngle(LookAheads[eLookAheads.HalfSec].Direction, Car.ForwardVector, Vector3.WorldUp);
			if (Math.Abs(value2) > 0f)
			{
				float angle = mem.Corner.OG.Angle;
				bool flag = Math.Abs(angle) > 0.01f && Math.Sign(value2) != Math.Sign(angle);
				float num6 = ARS.map(DistToOutside(CurrentTrackPoint.Direction), CurrentTrackPoint.TrackWide, CurrentTrackPoint.TrackWide * 2f, 0f, 45f, clamp: true);
				if (flag && Math.Abs(value2) > num6)
				{
					float num7 = (Math.Abs(value2) - num6) / 5f;
					mem.intention.MaxSpeed = Math.Max(num2 - 10f, num2 - num7);
				}
			}
		}
		if (Math.Sign((int)SteerPID.GetValue()) == Math.Sign((int)vData.YawRotationPerSecondDegrees))
		{
			float num8 = Car.Velocity.Length();
			float num9 = Math.Abs(SteerPID.GetValue() * 1f) * ((float)Math.PI / 180f);
			float num10 = num8 * (float)Math.Tan(num9) / (float)vData.WheelBase;
			num10 *= 180f / (float)Math.PI;
			float num11 = vData.CurrentMechanicalGrip * Handling.Gravity / num8 * (180f / (float)Math.PI);
			float f = (Math.Abs(num11 * 0.99f) - Math.Abs(num10)) / 10f;
			if (float.IsNaN(f))
			{
				f = 1f;
			}
		}
		float num12 = mem.intention.AggroToReach - mem.intention.Aggression;
		if (num12 > 0f)
		{
			mem.intention.Aggression += mem.personality.Rivals.AggressionBuildup * TickScale;
		}
		else
		{
			mem.intention.Aggression -= mem.personality.Rivals.AggressionBuildup * TickScale * 2f;
		}
	}

	private void TractionControl()
	{
		float wheelsMaxWheelspin = ARS.GetWheelsMaxWheelspin(Car);
		if (vControl.Throttle > 0f)
		{
			float num = 0.4f;
			if (GameTimeOutOfTrack != 0 && Game.GameTime - GameTimeOutOfTrack > 500)
			{
				num = 0.1f;
			}
			float num2 = ARS.map(Math.Abs(wheelsMaxWheelspin) - num, 0.1f, -0.1f, -1f, 1f, clamp: true);
			float num3 = num2 * TickScale;
			vControl.TCSThrottle = ARS.Clamp(vControl.TCSThrottle + num3, 0.1f, 1f);
		}
	}

	public void UpdateTickData()
	{
		SteerPID.Update();
		LanePID.Update();
		vData.LocalGs = (Function.Call<Vector3>(Hash._0x9A8D700A51CB7B0D, new InputArgument[2] { Car, true }) - vData.SpeedVectorLocal) / Game.LastFrameTime;
		Vector3 vector = Function.Call<Vector3>(Hash._0x9A8D700A51CB7B0D, new InputArgument[2] { Car, false });
		vData.AccelerationVector.Add((vector - lSpeed) / Game.LastFrameTime);
		if (vData.AccelerationVector.Count > 10)
		{
			vData.AccelerationVector.RemoveAt(0);
		}
		lSpeed = Function.Call<Vector3>(Hash._0x9A8D700A51CB7B0D, new InputArgument[2] { Car, false });
		vData.SpeedVectorGlobal = vector;
		vData.SpeedVectorLocal = Function.Call<Vector3>(Hash._0x9A8D700A51CB7B0D, new InputArgument[2] { Car, true });
		mem.data.SpeedVector = Function.Call<Vector3>(Hash._0x9A8D700A51CB7B0D, new InputArgument[2] { Car, true });
		if (trail.Count > 50)
		{
			trail.RemoveAt(0);
		}
	}

	public void ProcessTick()
	{
		UpdateTickData();
		DrawStuff();
		if (Driver.IsPlayer)
		{
			return;
		}
		TorqueMult = ARS.map(Math.Abs(vData.SlideAngle), 5f, 90f, 1f, 10f);
		ApplyInputs();
		if (ARS.Racers.Count > 1 && RacePosition > ARS.catchupPos && false && (!ARS.SettingsFile.GetValue("CATCHUP", "OnlyLoners", defaultvalue: true) || !mem.Rivals.Any((Rival v) => v.Distance < 50f)))
		{
			if (ARS.SettingsFile.GetValue("CATCHUP", "CatchupSpeed", 100) != 0 && vControl.HandBrakeTime < Game.GameTime && ARS.GetPercent(RacePosition, ARS.Racers.Count) >= 40f)
			{
				float num = (float)Math.Round((float)ARS.SettingsFile.GetValue("CATCHUP", "CatchupSpeed", 100) / 1000f, 2);
				Car.ApplyForceRelative(new Vector3(0f, ARS.Clamp(vControl.Throttle, 0f - num, num), 0f));
			}
			if (ARS.SettingsFile.GetValue("CATCHUP", "CatchupGrip", 100) != 0)
			{
				float num2 = (float)Math.Round((float)ARS.SettingsFile.GetValue("CATCHUP", "CatchupGrip", 100) / 1000f, 2);
				float x = ARS.Clamp(0f - mem.data.SpeedVector.X, 0f - num2, num2);
				Car.ApplyForceRelative(new Vector3(x, 0f, 0f));
			}
		}
	}

	public void RunTimedCore()
	{
		UpdateFollowTrack();
		UpdateDynamicBoundingBox();
		UpdatePercievedGrip();
		UpdateCornerInfo();
		ProcessAI();
		if (Driver.IsPlayer && ARS.SettingsFile.GetValue("CATCHUP", "OnlyBehindPlayer", defaultvalue: true))
		{
			ARS.catchupPos = RacePosition;
		}
		LastCoreTick = Game.GameTime;
	}

	private void UpdateDynamicBoundingBox()
	{
		vData.BoundingBox = ARS.GetDirectionalBoundingBox(Car);
		vData.SlideAngle = (float)Math.Round(Vector3.SignedAngle(Car.Velocity.Normalized, Car.ForwardVector, Car.UpVector), 3);
	}

	private void UpdateCornerInfo()
	{
		if (mem.Corner == null)
		{
			return;
		}
		if (mem.Corner.Valid && Lap > 0 && (CurrentTrackPoint.Node > mem.Corner.OG.Node || Math.Abs(CurrentTrackPoint.Node - mem.Corner.OG.Node) > 1000))
		{
			mem.Corner.Valid = false;
			BehaviorVariance[RandomVariance.SpeedAggroVariance] = (float)Random.Next(80, 120) * 0.01f;
			BehaviorVariance[RandomVariance.BrakeDistance] = (float)Random.Next(80, 120) * 0.01f;
		}
		if (Car.Model.IsCar && !Driver.IsPlayer)
		{
			if (mem.Rivals.Any((Rival v) => v.Distance < v.BoundingBoxTotal.Y + 1f))
			{
				if (Driver.IsInVehicle(Car) && Car.IsSeatFree(VehicleSeat.Passenger))
				{
					Driver.Alpha = 0;
					Driver.SetIntoVehicle(Car, VehicleSeat.Passenger);
				}
			}
			else if (Driver.IsInVehicle(Car) && Car.IsSeatFree(VehicleSeat.Driver))
			{
				Driver.Alpha = 255;
				Driver.SetIntoVehicle(Car, VehicleSeat.Driver);
				Driver.CanWearHelmet = true;
				Driver.GiveHelmet(canBeRemovedByPed: true, HelmetType.RegularMotorcycleHelmet, 0);
				Driver.RemoveHelmet(instantly: true);
			}
		}
		if (Car.Alpha != 255)
		{
			Car.ResetAlpha();
		}
		if (ARS.SettingsFile.GetValue("GENERAL_SETTINGS", "Ghosts", defaultvalue: false) && mem.Rivals.Any((Rival v) => v.Distance < 50f))
		{
			foreach (Rival rival in mem.Rivals)
			{
				if (Car.IsInRangeOf(rival.RivalRacer.Car.Position, 6f))
				{
					Function.Call(Hash._0xA53ED5520C07654A, rival.RivalRacer.Car, Car, true);
					Car.Alpha = 150;
				}
			}
		}
		if ((Car.IsUpsideDown || Math.Abs(mem.data.DeviationFromCenter) > CurrentTrackPoint.TrackWide) && !ControlledByPlayer && BaseBehavior == RacerBaseBehavior.Race)
		{
			if (GameTimeOutOfTrack == 0)
			{
				GameTimeOutOfTrack = Game.GameTime;
			}
			else if (Game.GameTime - GameTimeOutOfTrack > 5000 && Car.Velocity.Length() < 5f)
			{
				GameTimeOutOfTrack = 0;
				ResetIntoTrack();
			}
		}
		else if (GameTimeOutOfTrack != 0)
		{
			GameTimeOutOfTrack = 0;
		}
	}

	public void ApplyInputs()
	{
		if (Driver.IsSittingInVehicle(Car) && !Driver.IsPlayer)
		{
			if (vControl.HandBrakeTime > Game.GameTime)
			{
				Car.HandbrakeOn = true;
			}
			else
			{
				Car.HandbrakeOn = false;
			}
			ARS.SetThrottle(Car, ARS.Clamp(vControl.Throttle, -1f, 1f));
			ARS.SetBrakes(Car, vControl.Brake);
			ARS.SetSteerAngle(Car, vControl.SteerInput);
		}
		else
		{
			ARS.SetThrottle(Car, 0f);
			ARS.SetBrakes(Car, 0f);
			ARS.SetSteerInput(Car, 0f);
		}
	}

	private void DrawStuff()
	{
		if ((Car.Position - Game.Player.Character.Position).Length() > 50f)
		{
			return;
		}
		if (Driver.IsPlayer && Lap >= ARS.SettingsFile.GetValue("GENERAL_SETTINGS", "Laps", 5) && CanRegisterNewLap)
		{
			World.DrawMarker(MarkerType.CheckeredFlagRect, ARS.TrackPoints.First().Position + new Vector3(0f, 0f, 5f), ARS.TrackPoints.First().Direction, new Vector3(0f, 0f, 0f), new Vector3(5f, 5f, 5f), Color.White);
		}
		if (ARS.OptionValuesList[Options.ShowPhysics])
		{
			World.DrawMarker(MarkerType.DebugSphere, Car.Position + new Vector3(0f, 0f, Car.Model.GetDimensions().Z * 0.6f), Vector3.Zero, new Vector3(0f, 0f, 0f), new Vector3(0.1f, 0.1f, 0.1f), Color.Green, bobUpAndDown: false, faceCamY: false, 0, rotateY: false, "", "", drawOnEnt: false);
			Vector3 vector = vData.AccelerationVector.Aggregate(new Vector3(0f, 0f, 0f), (Vector3 s, Vector3 v) => s + v) / vData.AccelerationVector.Count;
			vector.Z = 0f;
			float percentage = ARS.map(vector.Length() / 9.8f, 0f, vData.CurrentMechanicalGrip, 0f, 100f, clamp: true);
			Color color = ARS.GradientAtoBtoC(Color.White, Color.Yellow, Color.Red, percentage);
			World.DrawMarker(MarkerType.DebugSphere, Car.Position + new Vector3(0f, 0f, Car.Model.GetDimensions().Z * 0.6f) + vector / 9.8f, Vector3.Zero, new Vector3(0f, 0f, 0f), new Vector3(0.15f, 0.15f, 0.15f), color, bobUpAndDown: false, faceCamY: false, 0, rotateY: false, "", "", drawOnEnt: false);
			ARS.DrawLine(Car.Position + new Vector3(0f, 0f, Car.Model.GetDimensions().Z * 0.6f) + vector / 9.8f, Car.Position + new Vector3(0f, 0f, Car.Model.GetDimensions().Z * 0.6f), color);
			Vector3 vector2 = new Vector3(vData.CurrentMechanicalGrip * 9.8f, vData.CurrentMechanicalGrip * 9.8f, vData.CurrentMechanicalGrip * 9.8f);
			Vector3 vector3 = Vector3.Clamp(vector, -vector2, vector2);
			Vector3 pos = Car.Position + new Vector3(0f, 0f, 0.5f + Car.Model.GetDimensions().Z * 0.6f);
			ARS.DrawText(pos, "~b~" + Math.Round(ARS.MStoMPH(Car.Velocity.Length())) + "~w~mph~n~~y~" + (vector.Length() / 9.8f).ToString("0.0") + " Gs", Color.White, 0.5f);
		}
		if (!Driver.IsPlayer)
		{
			if (!Car.IsInRangeOf(Game.Player.Character.Position, 500f))
			{
				return;
			}
			if (ARS.OptionValuesList[Options.ShowAggro])
			{
				World.DrawMarker(MarkerType.ChevronUpx1, Car.Position + new Vector3(0f, 0f, 1.5f), Vector3.Zero, new Vector3(0f, 0f, 0f), new Vector3(0.5f, 0.5f, -0.5f), ARS.GetColorFromRedYellowGreenGradient(100f - mem.intention.Aggression * 100f), bobUpAndDown: false, faceCamY: true, 0, rotateY: false, "", "", drawOnEnt: false);
			}
			if (trail.Count == 0)
			{
				trail.Add(Car.Position);
			}
			else if (Car.Position.DistanceTo(trail[trail.Count - 1]) > 2f)
			{
				trail.Add(Car.Position);
			}
			if (ARS.OptionValuesList[Options.ShowInputs])
			{
				Vector3 vector4 = SteerTarget + Vector3.WorldUp;
				Vector3 vector5 = Car.Position + Vector3.Cross(Car.ForwardVector, Vector3.WorldUp) * (LanePID.GetValue() - mem.data.DeviationFromCenter);
				ARS.DrawLine(Car.Position, vector5 + Vector3.WorldUp, Color.White);
				Color green = Color.Green;
				if (vControl.Brake > 0f)
				{
					green = Color.Yellow;
				}
				if (vControl.Brake > 0.5f)
				{
					green = Color.Orange;
				}
				if (vControl.Brake > 0.9f)
				{
					green = Color.Red;
				}
				Vector3 normalized = Car.Velocity.Normalized;
				Vector3 forwardVector = Car.ForwardVector;
				forwardVector.Z = normalized.Z;
				Vector3 dimensions = Car.Model.GetDimensions();
				Vector3 vector6 = Car.Position + new Vector3(0f, 0f, 0f - Car.HeightAboveGround);
				Vector3 vector7 = Quaternion.RotationAxis(Vector3.WorldUp, (float)Math.PI / 180f * vControl.SteerCurrent) * (Car.ForwardVector * (Car.Position - SteerTarget).Length());
				Vector3 vector8 = Quaternion.RotationAxis(Vector3.WorldUp, (float)Math.PI / 180f * vControl.SteerMax) * (Car.ForwardVector * (Car.Position - SteerTarget).Length());
				Vector3 vector9 = Quaternion.RotationAxis(Vector3.WorldUp, (float)Math.PI / 180f * (0f - vControl.SteerMax)) * (Car.ForwardVector * (Car.Position - SteerTarget).Length());
				float num = Car.Model.GetDimensions().Y + 1f;
				Vector3 vector10 = Car.Position + vector7;
				vector10.Z = vector4.Z;
				vector10 = Car.Position + vector8;
				vector10.Z = vector4.Z;
				vector10 = Car.Position + vector9;
				vector10.Z = vector4.Z;
				bool flag = true;
				Vector3 pos2 = vector6 + Car.ForwardVector * (dimensions.Y * 0.5f) * vControl.Throttle;
				Vector3 pos3 = vector6 - Car.ForwardVector * (dimensions.Y * 0.5f) * vControl.Brake;
				Color baseColor = ARS.GradientAtoB(Color.White, Color.Green, vControl.Throttle * 100f);
				Color baseColor2 = ARS.GradientAtoB(Color.White, Color.Red, vControl.Brake * 100f);
				if (vControl.Throttle > 0.05f)
				{
					World.DrawMarker(MarkerType.ChevronUpx1, pos2, -Car.ForwardVector, new Vector3(90f, 0f, 0f), new Vector3(num / 2f, num / 4f, 0f - num / 2f), Color.FromArgb(250, baseColor), bobUpAndDown: false, faceCamY: false, 0, rotateY: false, "", "", drawOnEnt: false);
				}
				if (vControl.Brake > 0.05f)
				{
					World.DrawMarker(MarkerType.ChevronUpx1, pos3, Car.ForwardVector, new Vector3(90f, 0f, 0f), new Vector3(num / 2f, num / 4f, 0f - num / 2f), Color.FromArgb(250, baseColor2), bobUpAndDown: false, faceCamY: false, 0, rotateY: false, "", "", drawOnEnt: false);
				}
			}
			if (!ARS.OptionValuesList[Options.ShowTrackAnalysis])
			{
				return;
			}
			Vector3 vector11 = Car.Position + new Vector3(0f, 0f, 0.5f + Car.Model.GetDimensions().Z * 0.6f);
			if (mem.Corner.Valid && Lap > 0)
			{
				CornerPoint oG = mem.Corner.OG;
				Vector3 pos4 = ARS.Path[oG.Node];
				float speed = oG.Speed;
				Color colorFromRedYellowGreenGradient = ARS.GetColorFromRedYellowGreenGradient(ARS.map(speed - Car.Velocity.Length(), -1f, 1f, 0f, 100f, clamp: true));
				Vector3 vector12 = Vector3.Zero;
				for (int num2 = oG.Node - (int)(oG.GetRadius() * 2f); num2 <= oG.Node; num2++)
				{
					float num3 = ARS.TrackPoints[num2].TrackWide / 4f;
					int num4 = oG.Node - num2;
					float distPercent = (float)num4 / (oG.GetRadius() * num3) * 100f;
					float b = (ARS.TrackPoints[num2].TrackWide - vData.BoundingBox / 2f) * (float)((oG.Angle > 0f) ? 1 : (-1));
					float a = (ARS.TrackPoints[num2].TrackWide - vData.BoundingBox / 2f) * (float)((oG.Angle < 0f) ? 1 : (-1));
					float num5 = ARS.Lerp(a, b, ARS.LaneApproach(distPercent) * 0.01f);
					Vector3 vector13 = ARS.TrackPoints[num2].Position - Vector3.Cross(Vector3.WorldUp, ARS.TrackPoints[num2].Direction) * num5 + Vector3.WorldUp / 4f;
					if (vector12 != Vector3.Zero)
					{
						ARS.DrawLine(vector12, vector13, Color.Red);
					}
					vector12 = vector13;
				}
				World.DrawMarker(MarkerType.ChevronUpx1, pos4, ARS.TrackPoints[oG.Node].Direction, new Vector3(90f, 0f, 0f), new Vector3(ARS.TrackPoints[oG.Node].TrackWide * 2.5f, 5f, 5f), Color.FromArgb(50, colorFromRedYellowGreenGradient.R, colorFromRedYellowGreenGradient.G, colorFromRedYellowGreenGradient.B));
				if (oG.Node - oG.LengthStart > 5)
				{
					World.DrawMarker(MarkerType.ChevronUpx1, ARS.TrackPoints[oG.Node - oG.LengthStart].Position, ARS.TrackPoints[oG.Node - oG.LengthStart].Direction, new Vector3(90f, 0f, 0f), new Vector3(ARS.TrackPoints[oG.Node].TrackWide * 2.5f, 5f, 5f), Color.FromArgb(50, colorFromRedYellowGreenGradient.R, colorFromRedYellowGreenGradient.G, colorFromRedYellowGreenGradient.B));
					ARS.DrawLine(ARS.TrackPoints[oG.Node - oG.LengthStart].Position, ARS.TrackPoints[oG.Node].Position, colorFromRedYellowGreenGradient);
				}
				if (oG.Node + oG.LenghtEnd < ARS.TrackPoints.Count - 5)
				{
					ARS.DrawLine(ARS.TrackPoints[oG.Node + oG.LenghtEnd].Position, ARS.TrackPoints[oG.Node].Position, colorFromRedYellowGreenGradient);
					World.DrawMarker(MarkerType.ChevronUpx1, ARS.TrackPoints[oG.Node + oG.LenghtEnd].Position, ARS.TrackPoints[oG.Node + oG.LenghtEnd].Direction, new Vector3(90f, 0f, 0f), new Vector3(ARS.TrackPoints[oG.Node].TrackWide * 2.5f, 5f, 5f), Color.FromArgb(50, colorFromRedYellowGreenGradient.R, colorFromRedYellowGreenGradient.G, colorFromRedYellowGreenGradient.B));
				}
				float speed2 = oG.Speed;
				float percentage2 = ARS.map(ARS.MStoMPH(speed2 - Car.Velocity.Length()), -10f, 10f, 0f, 100f, clamp: true);
				Color color2 = ARS.GradientAtoBtoC(Color.Red, Color.Yellow, Color.White, percentage2);
				World.DrawMarker(MarkerType.DebugSphere, ARS.TrackPoints[oG.Node].Position + Vector3.WorldUp * 2f, Vector3.Zero, new Vector3(0f, 0f, 0f), new Vector3(0.5f, 0.5f, 0.5f), color2);
			}
			for (int num6 = 0; num6 < 1; num6++)
			{
				if (num6 % 2 != 1)
				{
					int num7 = CurrentTrackPoint.Node + num6;
					if (CurrentTrackPoint.Node % 2 == 1)
					{
						num7++;
					}
					if (num7 >= 0 && num7 < ARS.TrackPoints.Count - 1)
					{
						TrackPoint trackPoint = ARS.TrackPoints[num7];
						float num8 = (float)Math.Sqrt(vData.CurrentMechanicalGrip * 9.8f * trackPoint.GeneralCurveRadius);
						float percentage3 = ARS.map(ARS.MStoMPH(num8 - Car.Velocity.Length()), -10f, 10f, 0f, 100f, clamp: true);
						Color color3 = ARS.GradientAtoBtoC(Color.Red, Color.Yellow, Color.White, percentage3);
						Vector3 pos5 = trackPoint.Position - Vector3.Cross(Vector3.WorldUp, trackPoint.Direction) * trackPoint.TrackWide;
						Vector3 pos6 = trackPoint.Position + Vector3.Cross(Vector3.WorldUp, trackPoint.Direction) * trackPoint.TrackWide;
						World.DrawMarker(MarkerType.DebugSphere, pos5, Vector3.Zero, new Vector3(0f, 0f, 0f), new Vector3(0.2f, 0.2f, 0.2f), color3);
						World.DrawMarker(MarkerType.DebugSphere, pos6, Vector3.Zero, new Vector3(0f, 0f, 0f), new Vector3(0.2f, 0.2f, 0.2f), color3);
					}
				}
			}
			return;
		}
		Color color4 = Color.FromArgb(50, Color.LightSkyBlue);
		int num9 = (int)ARS.Clamp(CurrentTrackPoint.Node + 500, 0f, ARS.TrackPoints.Count() - 1);
		for (int num10 = CurrentTrackPoint.Node; num10 < num9; num10++)
		{
			if (num10 % 2 == 1)
			{
				TrackPoint trackPoint2 = ARS.TrackPoints[num10];
				Vector3 pos7 = trackPoint2.Position + Vector3.Cross(trackPoint2.Direction, Vector3.WorldUp) * trackPoint2.TrackWide + Vector3.WorldUp * 0.5f;
				Vector3 pos8 = trackPoint2.Position + Vector3.Cross(trackPoint2.Direction, Vector3.WorldUp) * (0f - trackPoint2.TrackWide) + Vector3.WorldUp * 0.5f;
				World.DrawMarker(MarkerType.ChevronUpx1, pos7, trackPoint2.Direction, new Vector3(89f, 0f, -90f), new Vector3(1f, 1f, 2f), color4);
				World.DrawMarker(MarkerType.ChevronUpx1, pos8, trackPoint2.Direction, new Vector3(89f, 0f, -90f), new Vector3(1f, 1f, 2f), color4);
			}
		}
	}

	private float OutOfTrackDistance()
	{
		return Math.Abs(mem.data.DeviationFromCenter) + vData.BoundingBox / 2f - CurrentTrackPoint.TrackWide;
	}

	private void UpdateRivalInfo()
	{
		foreach (Rival rival2 in mem.Rivals)
		{
			rival2.Update(this);
		}
		if (Car.Driver.IsPlayer || ARS.SettingsFile.GetValue("GENERAL_SETTINGS", "Ghosts", defaultvalue: false))
		{
			return;
		}
		mem.intention.AggroToReach = 0f;
		if (mem.Rivals.Where((Rival r) => r.Distance < 50f).Count() >= 3)
		{
			return;
		}
		Rival rival = mem.Rivals.OrderBy((Rival v) => v.Distance).FirstOrDefault();
		if (rival.RivalRacer != null)
		{
			float distance = rival.Distance;
			float in_max = 100f;
			float in_min = 200f;
			if (rival.RivalRacer.RacePosition > RacePosition)
			{
				in_max = 15f;
				in_min = 30f;
			}
			mem.intention.AggroToReach = ARS.map(distance, in_min, in_max, 0f, 1f, clamp: true);
		}
	}

	public void UpdateFollowTrack()
	{
		int num = (int)ARS.Clamp(CurrentTrackPoint.Node, 0f, ARS.TrackPoints.Count - 1);
		List<TrackPoint> list = new List<TrackPoint>();
		int num2 = num - 6;
		while (list.Count <= 12)
		{
			if (num2 < 0 || num2 >= ARS.TrackPoints.Count)
			{
				num2 = 0;
			}
			list.Add(ARS.TrackPoints[num2]);
			num2++;
		}
		CurrentTrackPoint = list.OrderBy((TrackPoint t) => t.Position.DistanceTo(Car.Position)).First();
		mem.data.DeviationFromCenter = ARS.LeftOrRight(Car.Position, CurrentTrackPoint.Position, CurrentTrackPoint.Direction);
		LookAheads.Clear();
		float num3 = Car.Velocity.Length();
		int offset = (int)ARS.Clamp((int)(num3 * 1.8f / vData.CurrentMechanicalGrip), 1f, 500f);
		int offset2 = (int)(num3 * 0.25f);
		int offset3 = (int)(num3 * 0.5f);
		int offset4 = (int)(num3 * 0.75f);
		int offset5 = (int)num3;
		int offset6 = (int)(num3 * 1.5f);
		LookAheads.Add(eLookAheads.SteerRef, ResolveLookAhead(offset));
		LookAheads.Add(eLookAheads.QuarterSec, ResolveLookAhead(offset2));
		LookAheads.Add(eLookAheads.HalfSec, ResolveLookAhead(offset3));
		LookAheads.Add(eLookAheads.ThreeQuarterSec, ResolveLookAhead(offset4));
		LookAheads.Add(eLookAheads.OneSec, ResolveLookAhead(offset5));
		LookAheads.Add(eLookAheads.OneHalfSec, ResolveLookAhead(offset6));
		if (CanRegisterNewLap)
		{
			if (ARS.GetPercent(CurrentTrackPoint.Node, ARS.TrackPoints.Count) < 10f || (ARS.IsPointToPoint && ARS.GetPercent(CurrentTrackPoint.Node, ARS.TrackPoints.Count) > 99f && ARS.GetOffset(Car, ARS.TrackPoints.Last().Position).Y < 0f))
			{
				CanRegisterNewLap = false;
				Lap++;
				if (Lap > ARS.SettingsFile.GetValue("GENERAL_SETTINGS", "Laps", 5) && Car.CurrentBlip != null)
				{
					Car.CurrentBlip.Color = BlipColor.Green;
				}
				if (Lap > 1)
				{
					TimeSpan item = ARS.ParseToTimeSpan(Game.GameTime - LapStartTime);
					UI.Notify(Name + "'s laptime: ~b~" + item.ToString("m':'ss'.'f"));
					LapTimes.Add(item);
					LapStartTime = Game.GameTime;
				}
			}
		}
		else if (BaseBehavior == RacerBaseBehavior.Race && ARS.GetPercent(CurrentTrackPoint.Node, ARS.TrackPoints.Count) > 50f)
		{
			CanRegisterNewLap = true;
		}
		int num4 = CurrentTrackPoint.Node + (int)Car.Velocity.Length();
		if (num4 < ARS.TrackPoints.Count - 1)
		{
			TrackPoint trackPoint = ARS.TrackPoints[CurrentTrackPoint.Node + (int)Car.Velocity.Length()];
			TrackPoint trackPoint2 = ARS.TrackPoints[CurrentTrackPoint.Node + (int)Car.Velocity.Length() / 2];
			mem.data.CurveRadiusToFollowPoint = ARS.GetCurveRadius(CurrentTrackPoint.Position, trackPoint.Position, trackPoint2.Position) / 2f;
		}
		else
		{
			mem.data.CurveRadiusToFollowPoint = 999f;
		}
		TrackPoint ResolveLookAhead(int num5)
		{
			if (CurrentTrackPoint.Node + num5 >= ARS.TrackPoints.Count)
			{
				return ARS.TrackPoints[num5];
			}
			return ARS.TrackPoints[CurrentTrackPoint.Node + num5];
		}
	}

	public void AddDebugText(string s)
	{
		s = "~w~" + s + "~w~";
		if (!DebugText.Contains(s))
		{
			DebugText.Add(s);
		}
	}

	private void ProcessTimedAI()
	{
		if (mem.Corner == null)
		{
			return;
		}
		if (HalfSecondTick < Game.GameTime)
		{
			HalfSecondTick = Game.GameTime + 500 + (int)ARS.map(Car.Velocity.Length(), 0f, 100f, -250f, 250f, clamp: true);
			if (!mem.Corner.Valid && ARS.MStoMPH(Car.Velocity.Length()) > 10f)
			{
				ARS.LookForCornerAhead(this);
			}
		}
		if (OneSecondTick >= Game.GameTime)
		{
			return;
		}
		OneSecondTick = Game.GameTime + 1000;
		if (!ControlledByPlayer)
		{
			if (BaseBehavior == RacerBaseBehavior.Race && ARS.Racers.Count >= 1)
			{
				UpdateRivals();
			}
			if (!Driver.IsSittingInVehicle(Car) && Car.IsStopped && Driver.IsStopped && Driver.TaskSequenceProgress == -1)
			{
				TaskSequence taskSequence = new TaskSequence();
				Function.Call(Hash._0xC20E50AA46D09CA8, 0, Car, 6000, -1, 2f, 0, 0);
				taskSequence.Close();
				Driver.Task.PerformSequence(taskSequence);
				return;
			}
		}
		bool flag = true;
		if (BaseBehavior == RacerBaseBehavior.Race && Math.Abs(vData.SlideAngle) < 0.5f && Math.Abs(vControl.Throttle) > 0.9f)
		{
			Function.Call((Hash)9358855157613082681uL, Car, true);
		}
		if (Function.Call<bool>((Hash)4410405085910852542uL, new InputArgument[1] { Car }) && vControl.Brake > 0.1f)
		{
			Function.Call((Hash)9358855157613082681uL, Car, false);
		}
		if (ARS.DevSettingsFile.GetValue("RACERS", "AIRacerAutofix", 1) == 2 && Function.Call<bool>(Hash._0xBCDC5017D3CE1E9E, new InputArgument[1] { Car }))
		{
			Car.Repair();
		}
		if (vData.Gs.Length() < 0.1f && Car.Velocity.Length() < 1f && BaseBehavior == RacerBaseBehavior.Race)
		{
			if (StuckGameTimeRef == 0)
			{
				StuckGameTimeRef = Game.GameTime;
			}
			if (!Driver.IsPlayer && BaseBehavior == RacerBaseBehavior.Race && Driver.IsSittingInVehicle(Car) && vControl.HandBrakeTime < Game.GameTime && Game.GameTime - StuckGameTimeRef >= 2)
			{
				if (Game.GameTime - StuckGameTimeRef >= 4)
				{
					if (Driver.IsSittingInVehicle(Car) && !Car.IsInWater && Car.EngineHealth > 0f)
					{
						StuckGameTimeRef = 0;
						ResetIntoTrack();
						StuckRecover = false;
					}
				}
				else if (!StuckRecover && !Car.Model.IsBike)
				{
					LastStuckPlace = Car.Position;
					if (ARS.DebugVisual > 0)
					{
						UI.Notify("~b~" + Car.FriendlyName + " tries to recover");
					}
					StuckRecover = true;
				}
			}
		}
		if (StuckRecover && (!Car.IsInRangeOf(LastStuckPlace, 5f) || mem.data.SpeedVector.Y > 3f))
		{
			StuckRecover = false;
			StuckGameTimeRef = 0;
		}
	}

	public void ProcessAI()
	{
		ProcessTimedAI();
		if (BaseBehavior == RacerBaseBehavior.GridWait && vControl.HandBrakeTime < Game.GameTime)
		{
			vControl.HandBrakeTime = Game.GameTime + 100 * ARS.GetRandomInt(2, 6);
		}
		if (!ControlledByPlayer)
		{
			UpdateRivalInfo();
			SteerTrack();
			SpeedTrack();
			SteerCorrections();
			SpeedToThrottleBrake();
			TranslateSteer();
			TractionControl();
		}
	}

	private void ResetIntoTrack()
	{
		vControl.SteerTrackDegrees = 0f;
		Car.Position = ARS.Path[CurrentTrackPoint.Node] + new Vector3(0f, 0f, 3f);
		Car.Heading = CurrentTrackPoint.Direction.ToHeading();
		StuckRecover = false;
		LastStuckPlace = Vector3.Zero;
		Car.Speed = ARS.MPHtoMS(15f);
	}

	private void UpdatePercievedGrip()
	{
		float val = Function.Call<float>(Hash._0xA132FB5370554DB0, new InputArgument[1] { Car }) * (Handling.Gravity / 9.8f);
		val = ARS.Clamp(val, 0.1f, 5f);
		GroundGripMultiplier = ARS.GetWheelsGrip(Car).Average();
		Vector3 position = CurrentTrackPoint.Position;
		Vector3 position2 = LookAheads[eLookAheads.HalfSec].Position;
		Vector3 position3 = LookAheads[eLookAheads.OneSec].Position;
		float num = (position3.Normalized.Z - position2.Normalized.Z) * 90f;
		float hillGripMultiplierAtCurrentVelocityVector = ARS.GetHillGripMultiplierAtCurrentVelocityVector(this, 4f);
		vData.BaseMechanicalGrip = val;
		vData.CurrentMechanicalGrip = vData.BaseMechanicalGrip * GroundGripMultiplier;
		vData.CurrentMechanicalGrip *= hillGripMultiplierAtCurrentVelocityVector;
		vData.CurrentMechanicalGrip += Math.Min(ARS.WouldLiftOffRoadAtSpeed(position, position2, position3, Car.Velocity.Length()), 0f);
		UI.ShowSubtitle(hillGripMultiplierAtCurrentVelocityVector.ToString("0.00"), 500);
		if (Math.Abs(mem.data.DeviationFromCenter) < CurrentTrackPoint.TrackWide && RacePosition <= 2 && !ARS.MultiplierInTerrain.ContainsKey(CurrentTrackPoint.Node))
		{
			ARS.MultiplierInTerrain.Add(CurrentTrackPoint.Node, GroundGripMultiplier);
		}
		float num2 = Math.Abs(vData.SpeedVectorLocal.Normalized.Z) * 90f;
		if (num2 > 5f)
		{
			if (vData.AvgGroundStability >= 0.1f)
			{
				vData.AvgGroundStability -= num2 * TickScale * 0.1f;
			}
		}
		else if (vData.AvgGroundStability < 1f)
		{
			vData.AvgGroundStability += 1f * TickScale;
		}
		if (vData.AvgGroundStability > 1f)
		{
			vData.AvgGroundStability = 1f;
		}
		vData.Gs = vData.AccelerationVector.Aggregate(new Vector3(0f, 0f, 0f), (Vector3 s, Vector3 v) => s + v) / vData.AccelerationVector.Count / 2f;
		mem.data.CurveRadiusPhysicalGs = ARS.GetCurveRadius(Car.Position - Car.Velocity + vData.Gs / 2f, Car.Position + Car.Velocity + vData.Gs / 2f, Car.Position);
		vData.YawRotationPerSecondDegrees = ARS.rad2deg(Function.Call<Vector3>(Hash._0x213B91045D09B983, new InputArgument[1] { Car }).Z);
	}

	public void UpdateRivals()
	{
		List<Racer> list = new List<Racer>();
		foreach (Racer racer in ARS.Racers)
		{
			if (racer.Car.Handle != Car.Handle && racer.Car.Position.DistanceTo(Car.Position) < 100f)
			{
				list.Add(racer);
			}
		}
		foreach (Rival rival in mem.Rivals)
		{
			rival.RivalRacer = null;
		}
		if (list.Count > 0)
		{
			list.Sort((Racer a, Racer b) => Vector3.Distance(a.Car.Position, Car.Position).CompareTo(Vector3.Distance(b.Car.Position, Car.Position)));
			for (int num = 0; num < mem.Rivals.Count - 1 && num != list.Count; num++)
			{
				mem.Rivals[num].RivalRacer = list[num];
			}
		}
	}

	public void Delete()
	{
		if (!Driver.IsPlayer)
		{
			Driver.Delete();
		}
		if (Game.Player.Character.IsInVehicle(Car))
		{
			Game.Player.Character.SetIntoVehicle(Car, VehicleSeat.Driver);
			Car.IsPersistent = false;
			return;
		}
		if (Car.CurrentBlip != null && Car.CurrentBlip.Exists())
		{
			Car.CurrentBlip.Remove();
		}
		Car.Delete();
	}

	public float RiskFactorForGrip()
	{
		return 1.025f;
	}

	public float RiskFactorForBrake()
	{
		return 1f;
	}
}
