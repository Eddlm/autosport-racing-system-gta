using GTA;
using GTA.Math;
using GTA.Native;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;

namespace ARS
{
    public enum RacerBaseBehavior
    {
        GridWait, Race, FinishedRace, FinishedStandStill
    }
    public class Racer
    {


        public string Name = "Racer";
        internal string _baseName = "Racer";
        internal string CarModelName = "";
        public Ped Driver;
        public Vehicle Car;
        public Team TeamRole = Team.None;
        public bool ControlledByPlayer = false;
        public RacerBaseBehavior BaseBehavior = RacerBaseBehavior.GridWait;
        public RaceState RCStatus = RaceState.NotInitiated;


        public VehicleControl Control = new VehicleControl();
        public RacerBrain Brain = new RacerBrain();

        // Held apexes ahead: the nearest first, then the ones the car would have to brake earliest for.
        public int NextApexNode = -1;
        public float NextApexRadius = 999f;
        public float NextApexSpeed = 999f;
        public int NextApexNode2 = -1;
        public float NextApexRadius2 = 999f;
        public float NextApexSpeed2 = 999f;
        public int NextApexNode3 = -1;
        public float NextApexRadius3 = 999f;
        public float NextApexSpeed3 = 999f;

        // The corner NextApexNode names, published at the queue commit so the cards never rescan the table.
        CornerPoint _nextApexCorner;
        const int HeldApexCount = 3;



        public VehicleState VehicleData = new VehicleState();
        public HandlingData Handling = new HandlingData();
        public float GroundGripMultiplier = 1f;
        Vector3 _lastSpeed;

        
        int _lastGsCheck = 0;
        bool _gripLogged = false;
        // -1 until Launch captures the at-rest ride height; the height test is skipped while it has no value.
        float _restHeightAboveGround = -1f;

        // Racer progress along the route.
        public TrackPoint CurrentTrackPoint = new TrackPoint();
        public float TrackProgress = 0f;
        public float AlongTrackSpeed = 0f;
        public int RaceProgress = 0;

        // Time-based route references used by steering and speed calculations.
        public enum LookAhead { SteerRef, QuarterSec, HalfSec, ThreeQuarterSec, OneSec, OneHalfSec, TwoSec };
        public Dictionary<LookAhead, TrackPoint> LookAheads = new Dictionary<LookAhead, TrackPoint>();


        public List<TimeSpan> LapTimes = new List<TimeSpan>();
        public int LapStartTime = 0;
        public int Lap = 0;
        public int NitroChargedLap = -1;
        public int RacePosition = 0;
        // A car whose engine is dead cannot rejoin. It is parked on the shoulder and never races again this race.
        public bool IsDNF = false;
        // Frozen finish rank, assigned once when the racer crosses the line; 0 = still racing.
        public int FinalPosition = 0;
        public bool CanRegisterNewLap = false;
        int _previousNode = -1;
        public bool FinishedPointToPoint = false;


        int _halfSecondTick = 0;
        int _oneSecondTick = 0;
        int _phaseOffsetMs = 0;
        int _pressureTick = 0;
        int _apexUpdateTick = 0;
        int _rivalInfoTick = 0;
        int _lastCoreTick = 100;
        int TimeSince_lastCoreTick => (int)ARS.Clamp(Game.GameTime - _lastCoreTick, 1, 9999);


        List<TrackPoint> _trackPositionScratch = new List<TrackPoint>(13);
        const int StuckRecoveryTimeMs = 6000;
        const int StuckRecoveryCooldownMs = 2000;
        const int RecoveryArmDelayMs = 15000;
        StuckPhase _stuckPhase = StuckPhase.None;
        int _stuckPhaseEndTime = 0;
        int _stuckRecoveryStartTime = 0;
        int _stuckStationarySince = 0;
        int _stuckOffTrackSince = 0;
        int _stuckAgainSince = 0;
        int _stuckExitSince = 0;
        int _stuckArmAllowedTime = 0;
        int _stuckRecoveryCooldownEndTime = 0;
        Vector3 _stuckMoveSamplePosition = Vector3.Zero;
        int _stuckMoveSampleTime = 0;
        // Backing off is a short, straight phase. Drive is not a pedal override: it is a speed intention, so the plan,
        // the off-track cap and the lane law all keep owning the car while it rejoins.
        const int StuckReverseMs = 1000;
        const int RecoveryTriggerMs = 2000;
        const float RecoveryTriggerMph = 2f;
        const float RecoveryPlanGapMph = 1f;
        const float RecoveryDriveMph = 20f;
        const float RecoveryCoastBandMph = 5f;
        const float RecoveryExitMph = 4f;
        const int RecoveryExitHoldMs = 500;
        const int StuckMoveSampleMs = 1000;
        const float StuckMoveMeters = 0.5f;
        // DNF cars are parked on the shoulder, spaced along the route from the start line and alternating sides.
        const int DNFSlotSpacingMeters = 5;
        const float DNFSlotOffsetMeters = 1f;
        const float DNFUnderTrackMeters = 5f;


        public float RouteLookAheadSeconds = 0.5f;
        public float RouteLookaheadSizeSeconds = 2.0f;
        // The steer reference is a preview *time*: a fixed distance raises the loop's natural frequency with speed.
        const float SteerPreviewSeconds = 0.5f;
        const float SteerLookaheadMinMeters = 8f;
        const float SteerLookaheadMaxMeters = 30f;
        // The Gs-aware preview: how much of the car's projected lateral motion leads the lane error.
        const float GsPreviewBlend = 0.5f;


        float _avoidLeftWall = 0f;
        float _avoidRightWall = 0f;
        int _activeRivalWallCount = 0;
        bool _avoidWallsInitialized = false;
        float _targetLane = 0f;
        public float TargetLane { get { return _targetLane; } }
        Vector3 _debugLaneAimPoint = Vector3.Zero;

        float _cornerSpd = 999f;

        int _divebombApexNode = -1;
        int _defendApexNode = -1;

        const float OffshootRangeMeters = 2f;
        const float OffshootBlendBrake = 0.25f; // brake floor at the outer limit
        // A node deflection below this is a straight, which has no outside to judge.
        const float OffshootMinRouteAngleDeg = 2f;
        // Line discipline belongs to speed: below this the problem is traction, and a cap would hold an off-edge car
        // at zero, because the projection of a stationary car is the car.
        const float OffshootMinSpeedMph = 20f;
        const float FullPedalSpeedErrorMps = 3f;
        // Braking is the softer side: it takes this many times the speed error to command full brake.
        const float BrakeErrorMultiplier = 2f;

        // Applied pedal input sampled every half metre of travel, for the debug trail: the cap holds 40 m.
        readonly List<InputTrailSample> _inputTrail = new List<InputTrailSample>();
        const float InputTrailSampleSpacing = 0.5f;
        const int InputTrailMaxSamples = 80;
        // The trail draws chevrons pitched by the pedal — nose down for throttle, nose up for brake, 45° at full —
        // so the pitch is the gauge and the marker's bulk keeps it visible where a hairline was not. A site is
        // chosen as its sample is taken, never by a stride over the list: a list-anchored stride re-forms every
        // time a sample arrives or the oldest drops, and the trail strobes at frame rate.
        const float InputTrailChevronSpacing = 1f;
        const float InputTrailChevronGroundClearance = 1f;
        const float InputTrailChevronHeightGain = 0.5f;
        const float InputTrailChevronSize = 2f;
        const float InputTrailChevronPitch = 45f;
        const int InputTrailFadeCount = 5;
        float _inputTrailChevronDistance = 0f;


        // Yaw damper term in degrees: read by the steer sum below.
        float _damperTermDeg = 0f;
        // How much of the sliding countersteer blend the current slide has earned, 0 to 1; read by the limiter
        // and the slew within the same frame.
        float _slidePriority = 0f;
        float _steerPursuitDeg = 0f;
        float _steerAimBearingDegrees = 0f;
        float _steerAimCurvature = 0f;
        // Last off-track projection cap, kept for the pedal bar's override sphere.
        float _offtrackInputCap = 1f;

        // Brake learning (Phase 1): learn the effective decel factor per corner apex.
        const float BrakeFactorSeed = 0.75f;
        // The radius under which a corner counts as tight, i.e. where the outside line is worth having.
        const float CornerTightRadius = 50f;
        // Arrival is extrapolated from the live forward Gs at this discount: holding a G flat across the window
        // over-predicts, because acceleration tapers with speed. Tuned by driving.
        const float RequirementExtrapolationScale = 0.9f;
        const float PositionExcessMph = 10f;
        const float BrakeExcessMph = 25f;
        // Braking is judged from the distance it actually needs, plus this much road for the plan to take hold.
        const float BrakeHorizonMargin = 40f;
        // Each corner's initial factor is the seed plus a draw of +/- this many hundredths.
        const int BrakeFactorSeedJitterHundredths = 5;
        readonly Dictionary<int, CornerContext> _cornerContexts = new Dictionary<int, CornerContext>();
        int _cornersRevisionSeeded = -1;
        float _brakeSampleSeconds = 0f;      // sampled braking time: the denominator of the full-pedal share
        float _brakeSampleFullSeconds = 0f;  // of which, the time at full pedal: the numerator
        float _brakeSampleMaxSlideDeg = 0f;  // largest slip seen between this apex's entrance and the apex itself
        int _brakeSampleApexNode = -1;
        // The commit is deferred past the exit, because a car that entered too hot slides on the way out and that is
        // the same verdict as sliding in. The sample is handed over when the apex is passed and then spends
        // BrakeCommitDelaySeconds gathering the exit's slide before it is allowed to teach anything.
        int _brakeCommitApexNode = -1;
        int _brakeCommitAtGameTime = 0;
        float _brakeCommitSampleSeconds = 0f;
        float _brakeCommitFullSeconds = 0f;
        float _brakeCommitMaxSlideDeg = 0f;
        const float BrakeSampleThreshold = 0.25f;   // only pedal above this counts as braking at all
        const float BrakeFullPedalThreshold = 0.9f; // pedal at or above this counts as full brake
        const float BrakeFullFractionTarget = 0.2f; // target share of the braking phase spent at full pedal
        const float BrakeAdjustGain = 1f;           // step per unit of share error: 0.1 for every 10 points off
        const float BrakeSkidFactorStep = 0.1f;     // a skidded corner costs this much factor, flat
        const float BrakeSkidPeakMultiple = 2.5f;   // skid gate sits at the peak x this, where grip hits its floor
        const float BrakeCommitDelaySeconds = 0.75f; // wait this long after the apex before the verdict lands
        const float BrakeMinFactor = 0.5f;       // learned factor range floor
        const float BrakeMaxFactor = 1.2f;
        // Read by ARS.MaxSpeedForBrakingDistance (static) to scale its decel plan.
        public float BrakeFactorForApex(int apexNode) => ARS.BrakeLearning && TryGetCornerContext(apexNode, out CornerContext context) ? context.BrakeFactor : BrakeFactorSeed;
        float _divebombBrakeBonus; // temp brake boost while diving, whole hundredths 2-8 drawn per dive, never committed to learning

        // The car's own view of a corner: keyed by node and rebuilt only when the apex table changes, so a learned
        // factor survives a mid-session rebuild and a corner that no longer exists cannot linger.
        bool TryGetCornerContext(int apexNode, out CornerContext context)
        {
            SyncCornerContexts();
            return _cornerContexts.TryGetValue(apexNode, out context);
        }

        // Gates the braking plan, not the held queue: the queue must keep the real next corners or Brain.Corner,
        // NextApexSpeed and the low-speed invalidation all read a corner that is a lap away.
        bool CornerRequiresBraking(int apexNode) => apexNode >= 0 && TryGetCornerContext(apexNode, out CornerContext context) && context.RequiresBraking;

        CornerContext CornerContextForWrite(int apexNode)
        {
            if (!TryGetCornerContext(apexNode, out CornerContext context))
            {
                context = new CornerContext { BrakeFactor = SeededBrakeFactor() };
                _cornerContexts[apexNode] = context;
            }
            return context;
        }

        void SyncCornerContexts()
        {
            if (_cornersRevisionSeeded == ARS.CornersRevision) return;
            _cornersRevisionSeeded = ARS.CornersRevision;
            foreach (CornerPoint corner in ARS.Corners)
                if (!_cornerContexts.ContainsKey(corner.Node))
                    _cornerContexts[corner.Node] = new CornerContext { Point = corner, BrakeFactor = SeededBrakeFactor() };

            List<int> stale = new List<int>();
            foreach (int node in _cornerContexts.Keys)
                if (!ARS.Corners.Exists(corner => corner.Node == node)) stale.Add(node);
            foreach (int node in stale) _cornerContexts.Remove(node);
        }

        // One independent draw per corner, so a fresh grid does not brake every corner on one assumption.
        // The +1 makes the draw symmetric: GetRandomInt's max is exclusive.
        static float SeededBrakeFactor() => BrakeFactorSeed + ARS.GetRandomInt(-BrakeFactorSeedJitterHundredths, BrakeFactorSeedJitterHundredths + 1) / 100f;

        // While diving, a temp bonus is appended to the learned factor; the stored factor itself never changes.
        public float EffectiveBrakeFactor(int apexNode)
        {
            return ActiveManeuver.Type == ManeuverType.DiveBomb
                ? BrakeFactorForApex(apexNode) + _divebombBrakeBonus
                : BrakeFactorForApex(apexNode);
        }



        bool _isPassengerized = false;
        const float RivalSearchRangeMeters = 200f;

        // Feature gate for nitrous (ARS.NitrousEnabled).
        const float NitrousPowerMultiplier = 2.5f;
        const float NitrousMaxSteerDegrees = 5f;
        const float NitrousMinThrottle = 0.9f;
        const float NitrousCornerLookaheadSeconds = 8f;
        const float NitrousLonelySpeedFraction = 0.5f;
        const float NitrousLonelyMinApexDistance = 500f;
        const float NitrousDefenseReachSeconds = 4f;
        const float NitrousNearbyRivalDistance = 80f;
        const float NitrousFinishExtraDistance = 100f;
        const float ChillRivalCrowdDistance = 40f;
        const int ChillCrowdCount = 5;
        const float ChillStandoffDistance = 20f;
        const float ChillThrottleCap = 0.5f;
        const float ChillStandoffEscapeMargin = 2f;
        const float ChillMinSpeedMph = 30f;

        // Counts cars ahead on race progress only; cars behind don't crowd us.
        int RivalsWithinDistance(float distance)
        {
            if (ARS.NoCollision) return 0;
            return ARS.Racers.Count(r => r.Car.Handle != Car.Handle && r.RacePosition < RacePosition && r.Car.Position.DistanceTo(Car.Position) <= distance);
        }
        const int NitrousDurationMs = 3000;
        const float RocketBoostMinimumCurveRadius = 600f;
        const float SideBySideAssistRangeExtra = 3f;
        const float SideBySideFullAssistExtraGap = 1f;
        const float SideBySideMinimumAssist = 0.1f;
        const string NitrousPtfxAsset = "veh_xs_vehicle_mods";
        const ulong CheatPowerIncreaseHash = 0xB59E4BD37AE292DB;
        const ulong MaxDriveGearHash = 0x24910C3D66BA770D;
        const ulong CurrentDriveGearHash = 0x56185A25D45A0DCD;
        const ulong FullyChargeNitrousHash = 0x1A2BCC8C636F9226;
        const ulong OverrideNitrousLevelHash = 0xC8E9B6B71B8E660D;
        const float PowerCounterMax = 1.5f;
        const float PowerCounterMinCommanded = 0.1f;
        const float PowerCounterMinApplied = 0.05f;
        const float TopSpeedCounterOnsetScale = 1f;
        const float TopSpeedCounterBand = 15f;
        const float TopSpeedCounterMax = 1.35f;
        const float LaunchShapeMaxSpeed = 12f;
        const float ModulatedThrottleCap = 0.99f;
        const float ModulatedThrottleLow = 0.05f;
        const float ModulatedThrottleHigh = 0.95f;
        const float LaunchThrottleBinarySplit = 0.1f;

        public Maneuver ActiveManeuver = new Maneuver();
        int _nitrousActiveUntil = 0;
        int _nitrousLapUsed = -1;
        float _powerCounter = 1f;
        float _appliedThrottleLastFrame = 1f;
        int _powerCounterLoggedAt = 0;


        public float Aggression = 50f;
        public float Pressure = 0f;
        const float PressureRange = 100f;
        const float PressureProximityRange = 100f;
        const float PressureRisePerSecond = 2f;
        const float PressureFallPerSecond = 30f;

        public Racer(Vehicle RacerCar, Ped RacerPed)
        {
            Car = RacerCar;
            Driver = RacerPed;
            try { Name = RacerCar.FriendlyName; } catch (Exception) { Name = "Racer"; }
            if (Name == "NULL" || Name == null) { try { Name = Car.DisplayName.ToString()[0].ToString().ToUpper() + Car.DisplayName.ToString().Substring(1).ToLowerInvariant(); } catch (Exception) { Name = "Racer"; } }
            CarModelName = Name;

            if (Driver.IsPlayer) ControlledByPlayer = true;
            _halfSecondTick = Game.GameTime + (ARS.GetRandomInt(10, 50));
            _phaseOffsetMs = ARS.GetRandomInt(0, 500);
            _pressureTick = Game.GameTime;
            VehicleData.ModelDimensions = Car.Model.GetDimensions();

            if (!ControlledByPlayer)
            {

                try { Driver.BlockPermanentEvents = true; } catch (Exception) { }
                try { Driver.AlwaysKeepTask = true; } catch (Exception) { }
                Function.Call(GTA.Native.Hash.SET_DRIVER_ABILITY, Driver, 0f);
                Function.Call(GTA.Native.Hash.SET_DRIVER_AGGRESSIVENESS, Driver, 0f);

                if (ARS.SettingsMenuStore.GetInt("AIRacerAutofix", 2) == 2)
                {
                    Function.Call(GTA.Native.Hash.SET_ENTITY_PROOFS, Car, true, true, true, true, true, true, true, true);
                    Function.Call(GTA.Native.Hash.SET_ENTITY_PROOFS, Driver, true, true, true, true, true, true, true, true);

                    Car.IsInvincible = true;
                    Car.IsCollisionProof = true;
                    Car.IsOnlyDamagedByPlayer = true;
                    Function.Call(GTA.Native.Hash.SET_VEHICLE_STRONG, Car, true);
                    Function.Call(GTA.Native.Hash.SET_VEHICLE_HAS_STRONG_AXLES, Car, true);
                    try { Car.EngineCanDegrade = false; } catch (Exception) { }
                }
                else if (ARS.SettingsMenuStore.GetInt("AIRacerAutofix", 2) == 1)
                {
                    Function.Call(GTA.Native.Hash.SET_VEHICLE_STRONG, Car, true);
                    Function.Call(GTA.Native.Hash.SET_VEHICLE_HAS_STRONG_AXLES, Car, true);
                    try { Car.EngineCanDegrade = false; } catch (Exception) { }
                }
                else
                {
                    Car.IsInvincible = false;
                    Car.IsCollisionProof = false;
                }

                Car.EngineRunning = true;
                Driver.SetIntoVehicle(Car, VehicleSeat.Driver);
                VehicleMemory.SetSteerAngle(Car, 0.5f);
                VehicleMemory.SetThrottle(Car, 0f);
                VehicleMemory.SetBrakes(Car, 0f);

                try { Car.IsRadioEnabled = false; } catch (Exception) { }
            }

            try { Car.IsPersistent = true; } catch (Exception) { }
            if (!Driver.IsPlayer)
            {
                try
                {
                    if (Car.CurrentBlip == null || Car.CurrentBlip.Exists() == false)
                    {
                        Car.AddBlip();
                        Car.CurrentBlip.Color = BlipColor.Blue;
                        Car.CurrentBlip.Scale = 0.75f;
                        Function.Call(Hash._SET_BLIP_SHOW_HEADING_INDICATOR, Car.CurrentBlip, true);
                        Function.Call(Hash._0x2B6D467DAB714E8D, Car.CurrentBlip, true);
                        Car.CurrentBlip.Name = Name;
                    }
                }
                catch (Exception) {  }
            }

            Function.Call(GTA.Native.Hash._0x0DC7CABAB1E9B67E, Car, true, 1);
            Function.Call(GTA.Native.Hash._0x0DC7CABAB1E9B67E, Driver, true, 1);
            Function.Call(GTA.Native.Hash.SET_ENTITY_PROOFS, Driver, true, true, true, false, true, true, 1, true);

            Driver.MaxHealth = 1000;
            Driver.Health = 1000;
            Driver.CanSufferCriticalHits = false;

            if (Car.ClassType == VehicleClass.Emergency) TeamRole = Team.Cop;

        }
        public void Initialize()
        {
            Handling.Downforce = VehicleMemory.GetDownforce(Car);

            Handling.LateralTractionCurve = ARS.RadToDeg(VehicleMemory.GetLateralTraction(Car));
            if (Handling.LateralTractionCurve < 1 || Handling.LateralTractionCurve > 100) Handling.LateralTractionCurve = 22;
            ARS.Log(ARS.LogImportance.Info, "TRlat for " + Car.DisplayName + ":" + Handling.LateralTractionCurve + "º");

            Handling.BrakingAbility = Car.MaxBraking;
            Handling.EstimatedTopSpeed = ARS.EngineTopSpeed(Car);
            Handling.Acceleration = Function.Call<float>(Hash.GET_VEHICLE_ACCELERATION, Car);


            VehicleData.SteeringLock = ARS.RadToDeg(VehicleMemory.GetSteerLock(Car));
            if (VehicleData.SteeringLock < 1 || VehicleData.SteeringLock > 100) VehicleData.SteeringLock = 40;
            ARS.Log(ARS.LogImportance.Info, "Steerlock for " + Car.DisplayName + ":" + VehicleData.SteeringLock + "º");

            VehicleData.WheelBase = ARS.GetWheelBase(Car);
            ARS.Log(ARS.LogImportance.Info, "Wheelbase for " + Car.DisplayName + ":" + VehicleData.WheelBase + " m");
            Control.SteerDegrees = 0f;
            CurrentTrackPoint = ARS.TrackPoints.Last();
            Control.Brake = 0f;
            Control.Throttle = 0f;
            _powerCounter = 1f;
            _appliedThrottleLastFrame = 1f;
            if (ControlledByPlayer) ARS.PlayerModulatesThrottle = false;
            Function.Call((Hash)CheatPowerIncreaseHash, Car, 1.0f);

            LapTimes.Clear();
            LapStartTime = 0;
            VehicleData.ResetLapPeaks();
            Lap = 0;
            NitroChargedLap = -1;
            _nitrousLapUsed = -1;
            RacePosition = 0;
            FinalPosition = 0;
            IsDNF = false;
            Car.FreezePosition = false;
            CanRegisterNewLap = false;
            _previousNode = -1;
            _inputTrail.Clear();
            _inputTrailChevronDistance = 0f;
            _restHeightAboveGround = -1f;

            string flags = VehicleMemory.GetHandlingFlags(Car).ToString("X");
            int flagsHex = Convert.ToInt32(flags, 16);
            bool hasOffroad = (flagsHex & 0x800000) != 0 || (flagsHex & 0x200000) != 0;
            Handling.Gravity = 9.8f;
            if (hasOffroad) Handling.Gravity *= 1.2f;

            BaseBehavior = RacerBaseBehavior.GridWait;
            FinishedPointToPoint = false;

            Handling.Grip = Function.Call<float>((Hash)0xA132FB5370554DB0, Car);

            VehicleData.PerformanceIndex = (int)((Handling.EstimatedTopSpeed * 5) + (Handling.Grip * 100) + (Handling.Acceleration * 500));
            float modelGrip = Function.Call<float>((Hash)0x539DE94D44FDFD0D, Car.Model.Hash);
            float modelTopSpeedMph = ARS.MpsToMph(Function.Call<float>((Hash)0xF417C2502FFFED43, Car.Model.Hash));
            float modelAccel = Function.Call<float>(Hash.GET_VEHICLE_MODEL_ACCELERATION, Car.Model.Hash);
            bool modelElectric;
            if (!ARS.ModelElectricCache.TryGetValue(Car.Model.Hash.ToString(), out modelElectric)) modelElectric = ARS.IsElectricModel(Car.Model.Hash);
            // Deliberate: the three model natives are probed live rather than read from the cache the grid was ranked on, so
            // the cache answers only for the electric flag and for a probe that came back non-finite.
            float livePace = ARS.ComputePaceIndex(modelTopSpeedMph, modelGrip, modelAccel, modelElectric);
            if (float.IsNaN(livePace) || float.IsInfinity(livePace)) ARS.ModelPaceIndexCache.TryGetValue(Car.Model.Hash.ToString(), out livePace);
            VehicleData.PowerScale = livePace;
            VehicleData.TextPerformanceIndex = VehicleData.PowerScale.ToString("0.00");
            if (!ControlledByPlayer) Name = _baseName + " (" + VehicleData.PowerScale.ToString("0.00") + ")";

            _cornerContexts.Clear();
            _cornersRevisionSeeded = -1;
            _brakeCommitApexNode = -1;

            Car.Repair();
        }

        public void ComputeSteering()
        {
            _slidePriority = 0f;

            if (!TryGetSteerContext(out TrackPoint steerRefPoint, out float roadWide))
            {
                Control.SteerDegrees = 0f;
                return;
            }

            float speedMps = Math.Max(Car.Velocity.Length(), 1f);


            // --- Travel direction: the pursuit bearing is taken against it, and every term that steers shares it ---

            Vector3 courseDir = Car.ForwardVector;
            if (Car.Velocity.LengthSquared() > 0.01f) courseDir = Car.Velocity.Normalized;


            // --- Lane resolution: override chain (high-speed → corner → avoidance → walls) ---

            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            float drivableEdge = Math.Max(roadWide - carHalfWidth, 0f);
            float defaultLane = ComputeHighSpeedLane(roadWide, speedMps);
            bool gotActiveCorner = Brain.Corner != null && Lap > 0;
            float cornerLane = 0f;
            if (gotActiveCorner) cornerLane = ComputeCornerTargetLane(steerRefPoint, speedMps);
            if (cornerLane != 0f) defaultLane = cornerLane;
            float avoidAheadLane = ComputeAvoidAheadLane(roadWide);
            if (avoidAheadLane != 0f) defaultLane = avoidAheadLane;
            float carOffset = ARS.SignedLaneOffset(Car.Position, steerRefPoint.Position, steerRefPoint.Direction);
            if (_stuckPhase == StuckPhase.Drive && OutOfTrackDistance() > 0f) defaultLane = NearestEdgeLane(carOffset, drivableEdge);
            float targetLane = ApplyRivalWalls(defaultLane, roadWide);
            if (ARS.DebugToggles[Options.LockLaneCentre]) targetLane = LaneLockTestOffsetMeters;
            _targetLane = targetLane;

            // Aim at the lookahead distance, offset by the target lane. The Gs-aware preview shifts
            // that offset by the lateral motion the car is already committing to, so the correction leads the drift
            // instead of reacting to it; shifting a real point keeps the pursuit's chord real.
            Vector3 steerRight = Vector3.Cross(steerRefPoint.Direction, Vector3.WorldUp).Normalized;
            float aimLane = targetLane;
            if (ARS.DebugToggles[Options.GsAwarePreview])
            {
                float laneAtProjection = ARS.SignedLaneOffset(ProjectAhead(SteerPreviewSeconds), steerRefPoint.Position, steerRefPoint.Direction);
                aimLane = targetLane - GsPreviewBlend * (laneAtProjection - carOffset);
            }
            // The aim targets the car's centre, so it locks to the drivable edge; the rival walls bound to the raw edge.
            aimLane = ARS.Clamp(aimLane, -drivableEdge, drivableEdge);
            _debugLaneAimPoint = steerRefPoint.Position + steerRight * aimLane;

            // --- Lane steer: pure pursuit toward the target lane ---

            float laneSteerDeg = PursuitSteerDegrees(_debugLaneAimPoint, courseDir);
            _steerPursuitDeg = laneSteerDeg;
            // Physical repulsion: inside the "no touching" box, steer away from rivals
            // actually closing laterally; parallel traffic must not kill the lane steer.
            Vector3 velDir = speedMps > 0.5f ? Car.Velocity / speedMps : courseDir;
            Vector3 velRight = Vector3.Cross(velDir, Vector3.WorldUp);
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || !r.RivalRacer.Car.Exists()) continue;
                Vector3 delta = r.RivalRacer.Car.Position - Car.Position;
                float longDist = Math.Abs(Vector3.Dot(delta, velDir));
                float latDist = Math.Abs(Vector3.Dot(delta, velRight));
                float longGate = (VehicleData.ModelDimensions.Y + r.RivalRacer.VehicleData.ModelDimensions.Y) * 0.5f + 1f;
                float latGate = (VehicleData.BoundingBox + r.RivalRacer.VehicleData.BoundingBox) * 0.5f + 3f;
                if (longDist > longGate || latDist > latGate) continue;
                Vector3 rivalVel = r.RivalRacer.Car.Velocity;
                if (rivalVel.LengthSquared() < 0.01f) continue;
                float latSide = Vector3.Dot(delta, velRight);
                float latRelVel = Vector3.Dot(rivalVel - Car.Velocity, velRight);
                if (latSide * latRelVel >= 0f) continue;
                float dist = delta.Length();
                float distScale = ARS.Remap(dist, 6f, 2f, 0.5f, 2f, true);
                float strength = ARS.Remap(Math.Abs(latRelVel), 0.3f, 3f, 0f, 15f, true) * distScale;
                laneSteerDeg += Math.Sign(latSide) * strength;
            }


            // --- Heading assist: match a side-by-side rival's heading ---

            float sideBySideSteerDeg = ComputeSideBySideSteerCorrection(courseDir);


            // --- Assembly: lane pursuit + corrections + damper + slide blend ---

            const float steerKP = 1.0f;
            // Damp the excess over the yaw the aim point requires, not over zero which taxes every steady corner; the
            // slide blend keeps the zero reference, since its wanted rotation is the countersteer's.
            float fwdSpeed = ARS.GetForwardSpeed(Car);
            // The blend's weight: the slide is answered from a quarter of the authored peak slip and at full weight by
            // half of it, so a slide is caught well before the tyres' limit is gone. Both bands read the car's own
            // tyres, not the speed-scaled peak, so the response is the same at every speed.
            if (Handling.LateralTractionCurve > 0.01f && fwdSpeed >= 2f)
                _slidePriority = ARS.Remap(Math.Abs(VehicleData.SlideAngle), Handling.LateralTractionCurve * SlideBlendStartFraction, Handling.LateralTractionCurve * SlideBlendFullFraction, 0f, 1f, true);
            else
                _slidePriority = 0f;
            float yawTarget = 0f;
            if (SteerDampingAimReference && fwdSpeed > 0f && Math.Abs(VehicleData.SlideAngle) < Handling.LateralTractionCurve * SlidingFraction) yawTarget = ARS.RadToDeg(fwdSpeed * _steerAimCurvature);
            float yawRateToDamp = VehicleData.YawRotationPerSecondDegrees - yawTarget;
            float damperGain = SteerDampingFor(fwdSpeed, out _);
            float nonDamperSteerDeg = (steerKP * sideBySideSteerDeg) + (steerKP * laneSteerDeg);
            float damperTermDeg = -damperGain * yawRateToDamp;
            // The damper only ever subtracts: a left command never gets more left from it.
            if (damperTermDeg * nonDamperSteerDeg > 0f) damperTermDeg = 0f;
            // Past neutral it is countersteering — the wheel pointing against the car's own rotation rather than
            // merely less into it — and there it keeps half its authority. Not a cap tied to the slide: that fell to
            // zero with the slide and took the rate feedback off a car going straight, which set it oscillating.
            float damperOvershootDeg = Math.Abs(damperTermDeg) - Math.Abs(nonDamperSteerDeg);
            if (damperTermDeg * nonDamperSteerDeg < 0f && damperOvershootDeg > 0f) damperTermDeg = -Math.Sign(nonDamperSteerDeg) * (Math.Abs(nonDamperSteerDeg) + damperOvershootDeg * DamperCrossingShare);
            _damperTermDeg = damperTermDeg;
            Control.SteerDegrees = nonDamperSteerDeg + damperTermDeg;

            if (_slidePriority > 0f)
            {
                // The correction term only - not the slidePriority ramp - and the share of the slide the countersteer
                // answers, which reaches all of it at the top of the band.
                float countersteerTarget = (steerKP * sideBySideSteerDeg) - (VehicleData.SlideAngle * CountersteerShare());
                Control.SteerDegrees += (countersteerTarget - Control.SteerDegrees) * _slidePriority;
            }

            // On its aim to within the noise the command has nothing to correct and only flips sign from tick to
            // tick, so the wheel is left still — unless a slide is steering it, which must not be zeroed.
            if (Math.Abs(_steerAimBearingDegrees) < SmallAimBearingDegrees && _slidePriority <= 0f) Control.SteerDegrees = 0f;

            if (float.IsNaN(Control.SteerDegrees) || float.IsInfinity(Control.SteerDegrees))
                Control.SteerDegrees = 0f;

            // --- Local function: TryGetSteerContext ---

            bool TryGetSteerContext(out TrackPoint localSteerRef, out float localRoadWide)
            {
                localSteerRef = null;
                localRoadWide = 0f;
                if (BaseBehavior == RacerBaseBehavior.GridWait || BaseBehavior == RacerBaseBehavior.FinishedStandStill || CurrentTrackPoint.Node < 3)
                    return false;
                if (!LookAheads.TryGetValue(LookAhead.SteerRef, out localSteerRef) || localSteerRef == null)
                    return false;
                localRoadWide = localSteerRef.TrackHalfWidth;
                return true;
            }
        }

        // Phase 1: match an overlapping rival's heading so side-by-side cars follow the same arc, but only while
        // that rival is moving toward this car's side - one drifting away is leaving room, not asking to be followed.
        float ComputeSideBySideSteerCorrection(Vector3 courseDir)
        {
            float correction = 0f;
            foreach (Rival rival in Brain.Rivals)
            {
                if (rival.RivalRacer == null || !rival.RivalRacer.Car.Exists()) continue;

                Vector3 relativeOffset = ARS.EntityRelativeOffset(Car, rival.RivalRacer.Car);
                if (Math.Abs(relativeOffset.Y) > rival.CombinedSize.Y) continue;

                float lateralDistance = Math.Abs(relativeOffset.X);

                float fullAssistDistance = rival.CombinedSize.X + SideBySideFullAssistExtraGap;
                float wideDistance = rival.CombinedSize.X + SideBySideAssistRangeExtra;
                if (lateralDistance > wideDistance) continue;

                // Only while the rival is moving toward this car's side, taken on the axis that separates the two:
                // one drifting away is leaving room, and matching their heading there would steer this car across
                // the gap after them. This car's own closure is the rival walls' job, not this term's.
                Vector3 toRival = rival.RivalRacer.Car.Position - Car.Position;
                Vector3 lateralAxis = toRival - Car.ForwardVector * Vector3.Dot(toRival, Car.ForwardVector);
                float lateralGap = lateralAxis.Length();
                // No lateral separation to close - a same-lane overlap has no "away", so the match stays on.
                if (lateralGap > 0.01f && Vector3.Dot(rival.RivalRacer.Car.Velocity, lateralAxis / lateralGap) >= 0f) continue;

                float proximity = ARS.Remap(lateralDistance, wideDistance, fullAssistDistance, SideBySideMinimumAssist, 1f, true);
                float headingDifference = -Vector3.SignedAngle(rival.RivalRacer.Car.ForwardVector, courseDir, Vector3.WorldUp);
                if (float.IsNaN(headingDifference) || float.IsInfinity(headingDifference)) continue;

                correction += headingDifference * proximity;
            }
            return correction;
        }

        // Lane Control System 2: positions the car on the inside edge of the track curvature.
        const float HighSpeedLaneRadiusMeters = 500f;
        const float HighSpeedLaneChordSeconds = 1.01f;

        // Debug lock for the centring test: a near-centre aim offset, pinned for every racer.
        const float LaneLockTestOffsetMeters = 0.1f;

        // Pure pursuit's steer carries a factor of two the plain bearing misses: the curvature to a point at a given
        // bearing is 2 sin(angle) over the distance, not the angle over it. This is the knob if the lane is still shy.
        const float PursuitGain = 2f;
        // sin falls again past a right angle, so an aim point abeam or behind the car would fade instead of saturating.
        const float MaxPursuitBearingDegrees = 90f;
        // Inside this bearing the car is on its aim to within the noise, so the command only flips sign from tick to
        // tick. The aim bearing carries the lateral error, so a car merely parallel to the road is not inside it.
        const float SmallAimBearingDegrees = 1f;

        // Pure pursuit's curvature law: the bearing to an aim point is the chord of the circle through the car, so the
        // path's curvature is 2 sin(bearing) over the distance and the steer it demands is that curvature times the
        // wheelbase.
        float PursuitSteerFromBearing(float bearing, float distance)
        {
            bearing = ARS.Clamp(bearing, -MaxPursuitBearingDegrees, MaxPursuitBearingDegrees);
            return ARS.RadToDeg((float)Math.Atan(PursuitGain * Math.Sin(ARS.DegToRad(bearing)) * VehicleData.WheelBase / distance));
        }

        float PursuitSteerDegrees(Vector3 aimPoint, Vector3 heading)
        {
            Vector3 toAim = aimPoint - Car.Position;
            float distance = new Vector3(toAim.X, toAim.Y, 0f).Length();
            if (distance <= 0.5f)
            {
                _steerAimBearingDegrees = 0f;
                return 0f;
            }
            float bearing = Vector3.SignedAngle(heading, toAim, Vector3.WorldUp);
            if (float.IsNaN(bearing) || float.IsInfinity(bearing))
            {
                _steerAimBearingDegrees = 0f;
                return 0f;
            }
            _steerAimBearingDegrees = bearing;
            _steerAimCurvature = 2f * (float)Math.Sin(ARS.DegToRad(bearing)) / distance;
            return PursuitSteerFromBearing(bearing, distance);
        }

        float ComputeHighSpeedLane(float roadWide, float speedMps)
        {
            int count = ARS.TrackPoints.Count;
            int fwdNode;
            int fwdOffset = (int)(speedMps * HighSpeedLaneChordSeconds);
            if (ARS.IsPointToPoint)
                fwdNode = (int)ARS.Clamp(CurrentTrackPoint.Node + fwdOffset, 0, count - 1);
            else
                fwdNode = ((CurrentTrackPoint.Node + fwdOffset) % count + count) % count;

            TrackPoint ahead = ARS.TrackPoints[fwdNode];
            if (!(ahead.PreciseCurveRadius < HighSpeedLaneRadiusMeters)) return 0f;

            float cornerDir = Math.Sign(ahead.Angle);
            if (cornerDir == 0f) return 0f;

            // The edge, absolute: a target that steps from wherever the car is holds a constant error and so never
            // converges on the line. The gentleness belongs in the bearing, which grows as the car falls short.
            float insideBound = Math.Max(roadWide - VehicleData.BoundingBox * 0.5f, 0f);
            return -cornerDir * insideBound;
        }

        // How far ahead of a corner the car decides whether it wants to position or brake for it.
        const float RequirementLookaheadSeconds = 3.95f;

        // The chord meets the outside edge at the lead, and a car cannot change heading at the rate that implies, so
        // the blend is stretched past the geometric lead: the endpoints stay where the chord put them, the rate of
        // change falls as one over the multiple, and the path's curvature as one over its square.
        const float IdealLineLeadMultiple = 2f;

        // The ideal line is the widest one that still grazes the inside edge at the apex, and widening the arc until it
        // touches the outside answers that with a straight chord - whose outer end meets the outside edge at an angle,
        // which puts a corner in the target exactly where the car is settling onto it. So the chord supplies the two
        // ends - the outside edge a lead before the apex, the inside edge at it - and the blend between them leaves and
        // meets both edges flat, which is the part a car at speed can actually follow.
        float IdealLineOffset(int distanceFromApex, float apexRadius, float safeBound)
        {
            float innerRadius = apexRadius - safeBound;
            if (!(innerRadius > 0f) || apexRadius >= 999f) return 0f;
            float leadAngle = (float)Math.Acos(ARS.Clamp(innerRadius / (apexRadius + safeBound), -1f, 1f));
            float t = ARS.Clamp(Math.Abs(distanceFromApex) / Math.Max(apexRadius * leadAngle * IdealLineLeadMultiple, 1f), 0f, 1f);
            return safeBound * (2f * t * t * (3f - 2f * t) - 1f);
        }

        float ComputeCornerTargetLane(TrackPoint steerRefPoint, float speedMps)
        {
            CornerPoint c = Brain.Corner.Point;
            int apexNode = c.Node;

            // The line is a function of the distance to the apex, so no time enters the geometry - and it is measured
            // at the point the offset is applied to, or the profile is one lookahead stale. The reference sits ahead
            // of the car, though, so in the last lookahead before an apex it is already past one while the car is
            // not: falling back to the car's own distance there keeps the entry branch driving to the inside, where
            // reading the reference would open the profile outward at the apex.
            int fwdToApex = apexNode - steerRefPoint.Node;
            if (fwdToApex < 0) fwdToApex = ForwardNodeDistance(apexNode);

            float cornerDir = Math.Sign(c.Angle);
            if (cornerDir == 0f) return 0f;

            float halfWidth = steerRefPoint.TrackHalfWidth;
            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            float safeBound = Math.Max(halfWidth - carHalfWidth, 0f);

            // Corner-commit: hold the defend/dive line for the card's whole life, not just the outside phase.
            bool isCornerCommit = ActiveManeuver.Target != null && (ActiveManeuver.Type == ManeuverType.DefendLane || ActiveManeuver.Type == ManeuverType.DiveBomb);
            if (isCornerCommit)
            {
                Rival target = Brain.Rivals.FirstOrDefault(r => r.RivalRacer == ActiveManeuver.Target);
                if (target != null && target.RivalRacer.Car.Exists())
                {
                    float gap = target.OccupiedLaneWidth + 0.6f;
                    float commitLane = target.OccupiedLane + (-cornerDir) * gap;
                    return ARS.Clamp(commitLane, -safeBound, safeBound);
                }
            }

            // The car decides: a corner it has not asked to position for keeps the inside line, and so does one it
            // arrives at far below its own apex speed.
            bool wantsPosition = TryGetCornerContext(apexNode, out CornerContext apexContext) && apexContext.RequiresPositioning;
            bool aboveApexSpeed = speedMps > ApexSpeedWithDownforce(c.SupposedRadius) - ARS.MphToMps(20f);
            bool entryActive = wantsPosition && aboveApexSpeed;
            float offset = entryActive ? IdealLineOffset(fwdToApex, c.DetectedRadius, safeBound) : 0f;

            return cornerDir * offset;
        }

        // A rival ahead is only a pass target while the two velocity vectors sit within this angle; past it the
        // rival is not in a valid position to avoid, and the car falls back to follow-behind.
        const float AvoidAngleGateDegrees = 20f;
        // The wall and pass geometry only trust an adjacency when the pair also sits on neighbouring route nodes,
        // so an overlapping car on the far leg of a U is never mistaken for a trackside blockage.
        const float SameSectionMaxGapMeters = 30f;

        // Pick a lane to pass a rival ahead. If two rivals trigger on opposite sides, thread the needle.
        float ComputeAvoidAheadLane(float roadWide)
        {
            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            float trackBound = roadWide - carHalfWidth;
            float aggroBuffer = ARS.Remap(Aggression, 100f, 0f, 0.2f, 1.2f, true);
            float currentLaneMeters = Brain.CurrentPerception.DeviationFromCenter;

            Rival target = Brain.AvoidanceTarget;
            if (target == null || target.RivalRacer == null) return 0f;

            if (!TryPickAvoidanceSide(target, trackBound, aggroBuffer, carHalfWidth, currentLaneMeters, out float targetLane, out bool targetGoLeft))
                return 0f;

            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || r == target) continue;
                if (r.RelativePosition != RelativePos.Ahead) continue;
                if (!ARS.IsBetween(r.RouteGapAhead, 0f, SameSectionMaxGapMeters)) continue;
                if (!ARS.IsBetween(Math.Abs(r.DirectionDiff), 0f, AvoidAngleGateDegrees)) continue;
                if (!ARS.IsBetween(r.FrontGap, 0f, 3f) && !ARS.IsBetween(r.TimeToReach, 0f, 5f)) continue;

                if (!TryPickAvoidanceSide(r, trackBound, aggroBuffer, carHalfWidth, currentLaneMeters, out float secondTarget, out bool secondGoLeft))
                    continue;

                if (secondGoLeft == targetGoLeft) continue;
                return (targetLane + secondTarget) * 0.5f;
            }

            return targetLane;
        }

        bool TryPickAvoidanceSide(Rival rival, float trackBound, float aggroBuffer, float carHalfWidth, float currentLaneMeters, out float passLane, out bool passLeft)
        {
            passLane = 0f;
            passLeft = false;

            float rivalLane = rival.OccupiedLane;
            float buffer = rival.OccupiedLaneWidth + aggroBuffer;

            // Pass on the side the rival sits on relative to our predicted line (0.5s→1s projection segment).
            Vector3 predHalf = ProjectAhead(0.5f);
            Vector3 predDir = ProjectAhead(1f) - predHalf;
            float predLen = predDir.Length();
            if (predLen > 1f) predDir /= predLen;
            else predDir = Car.Velocity.Length() > 0.5f ? Car.Velocity.Normalized : Car.ForwardVector;
            float rivalSide = Vector3.Dot(Vector3.Cross(predDir, Vector3.WorldUp), rival.RivalRacer.Car.Position - predHalf);
            passLeft = rivalSide > 0f;

            passLane = passLeft ? rivalLane - buffer - carHalfWidth : rivalLane + buffer + carHalfWidth;

            // Side flip only on near-straights: route radius above the floor, never inside corners.
            const float SideFlipRadiusFloor = 200f;
            if (Math.Abs(passLane) > trackBound && Brain.CurrentPerception.CurveRadiusToFollowPoint > SideFlipRadiusFloor)
            {
                passLane = passLeft ? rivalLane + buffer + carHalfWidth : rivalLane - buffer - carHalfWidth;
                passLeft = !passLeft;
            }

            if (Math.Abs(passLane) > trackBound) return false;

            if (passLeft && currentLaneMeters <= passLane) return false;
            if (!passLeft && currentLaneMeters >= passLane) return false;

            return true;
        }

        float ApplyRivalWalls(float targetLane, float roadWide)
        {
            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            // The outer bound here is the raw track edge with no car-width inset; the per-rival walls below carry
            // both cars' widths instead.
            float trackBound = roadWide;

            if (!_avoidWallsInitialized)
            {
                _avoidLeftWall = -trackBound;
                _avoidRightWall = trackBound;
                _avoidWallsInitialized = true;
            }

            float targetLeftWall = -trackBound;
            float targetRightWall = trackBound;
            bool leftConstrained = false;
            bool rightConstrained = false;

            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null) continue;

                bool overlaps = Math.Abs(r.LongitudinalGap) < r.CombinedSize.Y && Math.Abs(r.RouteGapAhead) <= SameSectionMaxGapMeters;
                bool aheadAndClose = r.RelativePosition == RelativePos.Ahead && r.TimeToReach < 3f && ARS.IsBetween(r.RouteGapAhead, 0f, SameSectionMaxGapMeters) && Math.Abs(r.DirectionDiff) <= AvoidAngleGateDegrees;
                if (!overlaps && !aheadAndClose) continue;

                float aggroBuffer = ARS.Remap(Aggression, 100f, 0f, 0.2f, 1.2f, true);
                float rivalBuffer = r.OccupiedLaneWidth + aggroBuffer;

                if (r.OccupiedLane <= 0f)
                {
                    targetLeftWall = Math.Max(targetLeftWall, r.OccupiedLane + rivalBuffer);
                    leftConstrained = true;
                }
                else
                {
                    targetRightWall = Math.Min(targetRightWall, r.OccupiedLane - rivalBuffer);
                    rightConstrained = true;
                }
            }

            float openRate = 10f * TickScale;
            _avoidLeftWall = leftConstrained ? targetLeftWall : Math.Max(_avoidLeftWall - openRate, -trackBound);
            _avoidRightWall = rightConstrained ? targetRightWall : Math.Min(_avoidRightWall + openRate, trackBound);
            _activeRivalWallCount = (_avoidLeftWall > -trackBound ? 1 : 0) + (_avoidRightWall < trackBound ? 1 : 0);

            // Rival walls must remain ordered; collapse overlap to a narrow centered corridor.
            if (_avoidLeftWall > _avoidRightWall)
            {
                float mid = (_avoidLeftWall + _avoidRightWall) * 0.5f;
                _avoidLeftWall = mid - 1f;
                _avoidRightWall = mid + 1f;
            }

            float clampLeft = Math.Max(_avoidLeftWall + carHalfWidth, -trackBound);
            float clampRight = Math.Min(_avoidRightWall - carHalfWidth, trackBound);

            if (clampLeft > clampRight) return ARS.Clamp(targetLane, -trackBound, trackBound);
            return ARS.Clamp(targetLane, clampLeft, clampRight);
        }

        // Above the authored peak slip × this the car counts as sliding, so the damper drops its aim reference.
        const float SlidingFraction = 0.3f;
        // The blend ramps in across these multiples of the authored peak slip, then the countersteer answers this
        // share of the slide, growing from half to all of it over the next band.
        const float SlideBlendStartFraction = 0.25f;
        const float SlideBlendFullFraction = 0.5f;
        const float CountersteerSlideShare = 0.5f;
        // Pedal level held at full countersteer: enough throttle to keep the wheels rolling and no brake.
        const float CountersteerRollThrottle = 0.05f;
        // Below this forward speed the velocity direction is numerical noise, so the slide angle means nothing.
        const float CountersteerMinSpeedMph = 10f;
        const float SteerSlewRate = 180f;
        // How much of the damper's overshoot past neutral survives, so it may cross the sign, but at half authority.
        const float DamperCrossingShare = 0.5f;
        // Kill switch for the yaw-rate damper. Driven with it off the cars cannot hold centre — the term is the
        // only thing opposing a rotation the course chain has already started, so it is load-bearing, not trim.
        const bool SteerDampingEnabled = true;
        // The damper's reference: the yaw the aim point requires removes the standing-offset toll, and the zero
        // reference stays one flip away for A/B — the drive that preferred it was confounded by the course error.
        const bool SteerDampingAimReference = true;
        // The dial is defined at the damping speed the AI setting names: the damper's steer per unit yaw error is the
        // dial times that speed over the car's, so the term follows the steer a yaw rate actually needs - weaker above
        // it, and at most the dial below it. The cap is load-bearing: uncapped, a slow car ran many times the dial and
        // the term vetoed the steering instead of damping it.
        float SteerDampingFor(float forwardSpeed, out float speedScale)
        {
            // Abs, not the raw speed: a car travelling backwards needs the same steer per yaw rate as one going forwards.
            speedScale = Math.Min(1f, ARS.MphToMps(ARS.SteerDampingSpeedMph) / Math.Max(Math.Abs(forwardSpeed), 1f));
            return SteerDampingEnabled ? ARS.SteerDampingGain * speedScale / Math.Max(VehicleData.BaseMechanicalGrip, 1f) : 0f;
        }
        // Vanilla's player steering limiter used as a ceiling (AGENTS.md pipeline step 4): vanilla divides by
        // 1 + 0.075 × (forward speed − 5) in m/s and skips it while the car is sliding. The 5 m/s shift and its
        // gate are deliberately dropped here, so the ceiling starts closing from a standstill instead of
        // holding full lock to ~11 mph. Vanilla's auto-centre term beside it is not applied.
        const float VanillaSteerReductionPerMps = 0.075f;
        // Grip is read as a ratio to this reference, floored so a low-grip car cannot blow the coefficient up.
        const float SteerCapGripReference = 1f;
        const float SteerCapGripFloor = 0.5f;
        const float PeakSlipOuterWheelCommandShare = 1f / 0.75f;
        // Below the ramp's end speed the ceiling eases back to the car's full lock, so a slow car can steer in
        // fully; above it the ceiling is the law's own. The band is in mph because that is how it is judged.
        const float SteerLimitRampStartMph = 5f;
        const float SteerLimitRampEndMph = 15f;

        // Live ceiling coefficient: the useful steer angle at speed is ~ grip × g × wheelbase / v², so grip
        // belongs in that numerator and the cap loosens as √grip — the same √grip the speed maths uses.
        // Reads CurrentMechanicalGrip, not the base: BaseMechanicalGrip has the downforce term divided out
        // (1 + 0.035 × downforce) and only gets it back through Current, so a downforce car's base reads low.
        float SteerReductionPerMps
        {
            get
            {
                float grip = VehicleData.CurrentMechanicalGrip;
                if (float.IsNaN(grip)) grip = SteerCapGripReference;
                float gripRatio = Math.Max(grip / SteerCapGripReference, SteerCapGripFloor);
                return VanillaSteerReductionPerMps / (float)Math.Sqrt(gripRatio);
            }
        }

        // Deliberately the same grip and gravity that SteerLimitedSpeed uses, so the ceiling and the steer-limited
        // speed stay one law read two ways.
        float AckermannCeilingDegrees(float fwdSpeed)
        {
            float grip = VehicleData.CurrentMechanicalGrip;
            if (float.IsNaN(grip) || fwdSpeed <= 0f) return VehicleData.SteeringLock;
            float ratio = VehicleData.WheelBase * grip * 9.8f / (fwdSpeed * fwdSpeed);
            if (float.IsNaN(ratio) || ratio <= 0f) return VehicleData.SteeringLock;
            return ARS.RadToDeg((float)Math.Atan(ratio));
        }

        // The authored fTractionCurveLateral is the ZERO-SPEED peak — the tyre stiffens with speed, which is what
        // LateralPeakAtSpeed models. Refreshed once per timed core so every consumer in a frame reads the same value.
        public float TRLateralAtSpeed = 22f;

        void UpdateTRLateralAtSpeed()
        {
            float speed = Car.Velocity.Length();
            TRLateralAtSpeed = float.IsNaN(speed) ? 0f : LateralPeakAtSpeed(speed);
        }

        // The tyre's peak slip angle at an arbitrary speed; the live field is this value at the current speed.
        float LateralPeakAtSpeed(float speed)
        {
            float lat = Handling.LateralTractionCurve;
            return lat <= 0.01f ? 0f : lat / (1f + Math.Min(5f, 0.1f * speed));
        }

        float GeometrySteerCeiling(float fwdSpeed)
        {
            float vanillaCeiling = VehicleData.SteeringLock / (1f + SteerReductionPerMps * fwdSpeed);
            return Math.Min(vanillaCeiling, AckermannCeilingDegrees(fwdSpeed));
        }

        float PeakSlipCeilingAt(float peakSlipDeg)
        {
            return ARS.Clamp(peakSlipDeg * PeakSlipOuterWheelCommandShare, 0f, VehicleData.SteeringLock);
        }

        // The ceiling in force at this speed: the peak-slip cap, with the corner geometry as a fallback only when the
        // live peak reads unusable. Eased back towards full lock below the ramp's end speed, where it meets what
        // the car gets at that end speed anyway. The menu's SteerCeilingFactor scales the at-speed peak-slip cap
        // only - never the geometry fallback and never the ramp's low-speed end, so placement authority below the
        // ramp end is untouched; the factor bites only at and above it.
        float ResolveSteerCeiling(float fwdSpeed)
        {
            float ceiling = TRLateralAtSpeed > 0.01f ? PeakSlipCeilingAt(TRLateralAtSpeed) * ARS.SteerCeilingFactor : GeometrySteerCeiling(fwdSpeed);
            float speedMph = ARS.MpsToMph(fwdSpeed);
            if (speedMph >= SteerLimitRampEndMph) return ceiling;

            float endSpeed = ARS.MphToMps(SteerLimitRampEndMph);
            float endPeak = LateralPeakAtSpeed(endSpeed);
            float endCeiling = endPeak > 0.01f ? PeakSlipCeilingAt(endPeak) : GeometrySteerCeiling(endSpeed);
            // max() keeps the ramp a raise only: the straight line sits a degree under the curved law near 25 mph.
            return Math.Max(ceiling, ManeuverRamp(speedMph, endCeiling));
        }

        // Between the ramp's start and end speeds the ceiling eases up to the car's own lock, so a slow car can place
        // itself; at or above the end speed the law's own ceiling stands. A raise only, never a cut.
        float ManeuverRamp(float speedMph, float ceiling)
        {
            if (speedMph >= SteerLimitRampEndMph) return ceiling;
            // Descending *input* with ascending output, because Remap's own clamp inverts a descending output.
            return ARS.Remap(speedMph, SteerLimitRampEndMph, SteerLimitRampStartMph, ceiling, VehicleData.SteeringLock, true);
        }


        // The slide-governed addition is this share of the slide angle plus this much free play, so the wheel reaches
        // past the velocity vector to catch a rotation without the slide dictating the whole ceiling.
        const float SlideLimitSlideShare = 0.5f;
        const float SlideLimitFreeplayDegrees = 0.5f;
        // Retired by decision: with the raise on, half the slide angle plus the free play loosened the ceiling for every
        // command, and the base law alone was driven as the better of the two. Kept as a switch rather than deleted,
        // and deliberately not const so the raise stays compiled while it is off. The countersteer allowance and the
        // damper bypass are separate and stay; delete the raise, its constants and this switch together.
        static readonly bool SlideLimitRaise = false;

        void ApplySteerLimits()
        {
            // NaN guard: Clamp would turn NaN into full-lock.
            if (float.IsNaN(Control.SteerDegrees) || float.IsInfinity(Control.SteerDegrees))
            {
                Control.SteerDegrees = 0f;
                return;
            }

            float requestedSteer = Control.SteerDegrees;
            float fwdSpeed = Vector3.Dot(Car.Velocity, Car.ForwardVector);

            // Two limits, one per side, closed by one clamp: the ceiling is granted and then raised on the answering
            // side rather than the clamp being skipped — except for the stabiliser below. Reverse is always the full
            // lock: backing up is placing the car, and the geometry law inverts below −13 m/s anyway.
            bool countersteering = Math.Sign(requestedSteer) != Math.Sign(VehicleData.YawRotationPerSecondDegrees);
            float speedCeiling = VehicleData.SteeringLock;
            if (fwdSpeed > 0f)
            {
                speedCeiling = ResolveSteerCeiling(fwdSpeed);
                // The slide adds authority, it does not replace the cornering law: the ceiling is the larger of the
                // two, so a car can always steer in and rejoin, and a slide only ever opens more than it has.
                if (SlideLimitRaise) speedCeiling = Math.Max(speedCeiling, Math.Min(Math.Abs(VehicleData.SlideAngle) * SlideLimitSlideShare + SlideLimitFreeplayDegrees, VehicleData.SteeringLock));
            }
            // LEFT bounds positive commands and RIGHT negative ones, because a positive command steers left.
            float steerLimitRight = speedCeiling;
            float steerLimitLeft = speedCeiling;

            // The one whitelisted allowance: the side answering a slide reaches past the ceiling towards the slide
            // angle itself, in proportion to how much of the blend that slide has earned.
            if (countersteering && _slidePriority > 0f)
            {
                float countersteerAllowance = Math.Min(Math.Abs(VehicleData.SlideAngle) * _slidePriority, VehicleData.SteeringLock);
                if (requestedSteer > 0f) steerLimitLeft = Math.Max(steerLimitLeft, countersteerAllowance);
                else steerLimitRight = Math.Max(steerLimitRight, countersteerAllowance);
            }

            float yawRate = VehicleData.YawRotationPerSecondDegrees;

            // The damper is the car's stabiliser and the rotation leads the slide, so while its term pushes against
            // the rotation it is answering a slide the ceiling cannot see yet: that side is not capped at all, after
            // the envelope has had its say. Both the term and the command must be against the rotation, so this is a
            // raise for a correction and never for steer-in, which shares the side.
            if (countersteering && _damperTermDeg * yawRate < 0f)
            {
                if (requestedSteer > 0f) steerLimitLeft = VehicleData.SteeringLock;
                else if (requestedSteer < 0f) steerLimitRight = VehicleData.SteeringLock;
            }

            Control.SteerDegrees = ARS.Clamp(requestedSteer, -steerLimitRight, steerLimitLeft);
        }


        // The share of the slide the countersteer answers: half once the blend is full, all of it by the top of the band.
        float CountersteerShare()
        {
            float peak = Handling.LateralTractionCurve;
            if (peak <= 0.01f) return CountersteerSlideShare;
            return ARS.Remap(Math.Abs(VehicleData.SlideAngle), peak * SlideBlendFullFraction, peak, CountersteerSlideShare, 1f, true);
        }


        // True when the blend has reached full authority at half the authored peak slip.
        // Forward speed gates it: reversing reads as a ~180° slide, and a reversed car must never be starved.
        bool IsFullCountersteer()
        {
            if (Vector3.Dot(Car.Velocity, Car.ForwardVector) < ARS.MphToMps(CountersteerMinSpeedMph)) return false;
            return _slidePriority >= 1f;
        }


        public void Launch()
        {
            // Open the launch window at the green: the player's input is judged during the launch itself.
            if (ControlledByPlayer) ARS.PlayerLaunchTestActive = true;

            Brain.Corner = null;
            _nextApexCorner = null;
            ActiveManeuver.Type = ManeuverType.None;
            ActiveManeuver.Target = null;
            _divebombApexNode = -1;
            _defendApexNode = -1;
            NextApexNode = -1;
            NextApexRadius = 999f;
            NextApexSpeed = 999f;
            NextApexNode2 = -1;
            NextApexRadius2 = 999f;
            NextApexSpeed2 = 999f;
            NextApexNode3 = -1;
            NextApexRadius3 = 999f;
            NextApexSpeed3 = 999f;
            BaseBehavior = RacerBaseBehavior.Race;
            Lap = 1;
            _stuckArmAllowedTime = Game.GameTime + RecoveryArmDelayMs;
            LapStartTime = ARS.IsPointToPoint ? Game.GameTime : 0;
            VehicleData.ResetLapPeaks();
            CanRegisterNewLap = false;
            _previousNode = -1;
            Pressure = 0f;
            // The grid is the one moment the car is known to be at rest, so this is where the baseline comes from.
            float restHeight = Car.HeightAboveGround;
            if (restHeight > 0f) _restHeightAboveGround = restHeight;
            Control.MaxThrottle = 1f;
            Control.MaxBrake = 1f;
            Control.MaxBrakeFromABS = 1f;
            Control.MaxBrakeFromCountersteer = 1f;
            Control.MaxThrottleFromTCS = 1f;
            Control.MaxThrottleFromInstability = 1f;
            Control.MaxThrottleFromOverspeed = 1f;
            Control.MaxThrottleFromRival = 1f;
            Control.MaxThrottleFromChillOut = 1f;
            Control.MaxThrottleFromYield = 1f;
            Control.MaxThrottleFromOffTrack = 1f;
            Control.MaxThrottleFromRecovery = 1f;
            Control.ThrottleReason = ThrottleReason.Plan;
            Control.ThrottleReasonLevel = 1f;
            Control.BrakeReason = BrakeReason.Plan;
            Control.BrakeReasonLevel = 1f;
            _stuckPhase = StuckPhase.None;
            _stuckPhaseEndTime = 0;
            _stuckRecoveryStartTime = 0;
            _stuckStationarySince = 0;
            _stuckOffTrackSince = 0;
            _stuckAgainSince = 0;
            _stuckExitSince = 0;
            _stuckRecoveryCooldownEndTime = 0;
            _stuckMoveSampleTime = 0;
            Control.LastAppliedSteerDegrees = 0f;
            if (TeamRole == Team.Cop) Car.SirenActive = true;

        }

        const float PedalSlewRate = 6f;             // pedal slew rate (units/second: full range in ~167ms)
        const float PedalSlewMaxPerTick = 0.5f;     // hitch guard: never step more than this in one tick

        void ConvertSpeedToPedals()
        {
            float currentForwardSpeed = VehicleData.SpeedVectorLocal.Y;
            float inputChange = Math.Min(PedalSlewRate * TickScale, PedalSlewMaxPerTick);
            float newThrottle = 0f;
            float newBrake = 0f;

            FindLowestIntendedSpeed();

            Brain.CurrentIntention.IntendedSpeedChange = Brain.CurrentIntention.Speed - currentForwardSpeed;

            float intendedSpeedChange = Brain.CurrentIntention.IntendedSpeedChange;

            float combinedInput = ComputeCombinedInput(intendedSpeedChange);
            float preBlendInput = combinedInput;
            _offtrackInputCap = OffshootInputCap(1f);
            combinedInput = Math.Min(combinedInput, _offtrackInputCap);
            bool offtrackLimited = combinedInput < preBlendInput;
            bool offtrackBrakeCommand = offtrackLimited && combinedInput < 0f;
            // Full countersteer: no brake, just enough throttle to keep the wheels rolling. The reason caps still
            // apply on top; the release is instant.
            bool countersteering = IsFullCountersteer();
            if (countersteering)
            {
                combinedInput = CountersteerRollThrottle;
            }
            SplitCombinedInput(combinedInput, ref newThrottle, ref newBrake);
            newThrottle = ComposeThrottleCap(newThrottle, offtrackLimited, countersteering);
            // Brake composes here on the raw split target, pre-slew: in the throttle-to-brake transition
            // Control.Throttle slews down slowly and could still read >= 1.0 while the car is already braking.
            newBrake = ComposeBrakeCap(newBrake, offtrackBrakeCommand, countersteering);
            bool rbFreeZonePedal = Lap <= 1 && CurrentTrackPoint != null && CurrentTrackPoint.Node < 500;
            if (ARS.RubberbandingPct > 0 && ARS.CurrentRubberbandMode == RubberbandMode.Artificial && newThrottle >= 1.00f && !rbFreeZonePedal)
            {
                float rbTorque = ComputeRubberBandFactor();
                if (rbTorque > 1f) newThrottle *= rbTorque;
            }

            Control.Brake += ARS.Clamp(newBrake - Control.Brake, -inputChange, inputChange);
            Control.Throttle += ARS.Clamp(newThrottle - Control.Throttle, -inputChange, inputChange);

            UpdateBrakeLearning();

            if (Brain.CurrentIntention.MaxSpeed < AiConstants.MaxSpeed) Brain.CurrentIntention.MaxSpeed += 15 * TickScale;

        }

        // One linear map for both pedals: a direction swap needs no case of its own, because its error is the sum of the magnitudes and saturates anyway.
        float ComputeCombinedInput(float intendedSpeedChange)
        {
            float divisor = intendedSpeedChange < 0f ? FullPedalSpeedErrorMps * BrakeErrorMultiplier : FullPedalSpeedErrorMps;
            return ARS.Clamp(intendedSpeedChange / divisor, -1f, 1f);
        }

        void SplitCombinedInput(float combinedInput, ref float newThrottle, ref float newBrake)
        {
            if (combinedInput > 0f)
            {
                newThrottle = combinedInput;
                return;
            }

            if (combinedInput < 0f) newBrake = -combinedInput;
        }



        void FindLowestIntendedSpeed()
        {
            if (Brain.CurrentIntention.Speed >= 0f)
            {
                Brain.CurrentIntention.Speed = Math.Min(Brain.CurrentIntention.Speed, Brain.CurrentIntention.MaxSpeed);
            }
        }

        // Projection response: on a corner's outside, cap the maximum combined input by how far off-centre
        // the projection lands — full throttle on the centre line, none at the track edge, light brake beyond.
        float OffshootInputCap(float seconds)
        {
            // Only meaningful when the car is aiming at a lane; with no target lane there is no
            // hug-inside expectation to enforce, so the outside sanity check must not fire.
            if (_targetLane == 0f) return 1f;

            if (ARS.MpsToMph(Car.Velocity.Length()) < OffshootMinSpeedMph) return 1f;

            Vector3 proj = ProjectAhead(seconds);
            // 1 node ≈ 1 m: the window has to reach the projected node.
            int projectionWindow = (int)(Car.Velocity.Length() * seconds) + 10;
            TrackPoint tp = ARS.FindNearestTrackPoint(proj, CurrentTrackPoint.Node, 0, projectionWindow);
            float signedOffset = ARS.SignedLaneOffset(proj, tp.Position, tp.Direction);
            float halfWidth = Math.Max(tp.TrackHalfWidth, 0.1f);

            // Outside is the side the corner the car is in turns away from.
            float turnDirection = Math.Abs(CurrentTrackPoint.Angle) > OffshootMinRouteAngleDeg ? Math.Sign(CurrentTrackPoint.Angle) : 0f;

            if (Math.Sign(signedOffset) != turnDirection) return 1f;

            // Inside half keeps full throttle; the cap ramps from the centre line to the edge, then brakes.
            float outsideOffset = signedOffset * turnDirection;
            if (outsideOffset > halfWidth) return -OffshootBlendBrake * Math.Min((outsideOffset - halfWidth) / OffshootRangeMeters, 1f);
            if (outsideOffset > 0f) return 1f - outsideOffset / halfWidth;
            return 1f;
        }

        // Samples braking quality across the approach to the current apex. The verdict waits: the sample is parked
        // at the apex pass and only lands a couple of seconds later, once the exit has had its say.
        void UpdateBrakeLearning()
        {
            if (!ARS.BrakeLearning)
            {
                _brakeCommitApexNode = -1;
                return;
            }

            // Deferred commit: still watching, now for the slide a too-hot entry shows on the way out.
            if (_brakeCommitApexNode >= 0)
            {
                float exitSlide = Math.Abs(VehicleData.SlideAngle);
                if (!float.IsNaN(exitSlide) && exitSlide > _brakeCommitMaxSlideDeg) _brakeCommitMaxSlideDeg = exitSlide;
                if (Game.GameTime >= _brakeCommitAtGameTime) CommitBrakeLearning();
            }

            // The apex this sample belongs to, reset here rather than after the gates below, so a corner change
            // during a maneuver or past its own entrance cannot leave the sample pointing at an older apex.
            if (NextApexNode != _brakeSampleApexNode)
            {
                _brakeSampleApexNode = NextApexNode;
                _brakeSampleSeconds = 0f;
                _brakeSampleFullSeconds = 0f;
                _brakeSampleMaxSlideDeg = 0f;
            }

            // Runs on the corner proper, entrance to apex — exactly where the sample has stopped. A car that slid on
            // the way in was on a tyre that had already let go, so the commit below throws that sample away.
            if (_brakeSampleApexNode >= 0 && HasPassedBrakingTarget() && !HasPassedApex(_brakeSampleApexNode))
            {
                float slide = Math.Abs(VehicleData.SlideAngle);
                if (!float.IsNaN(slide) && slide > _brakeSampleMaxSlideDeg) _brakeSampleMaxSlideDeg = slide;
            }

            if (ActiveManeuver.Type != ManeuverType.None) return;

            if (HasPassedBrakingTarget()) return;

            if (Control.Brake <= BrakeSampleThreshold) return;
            _brakeSampleSeconds += TickScale;
            // The numerator is time at full pedal: the target is a share of the phase, not an amount.
            if (Control.Brake >= BrakeFullPedalThreshold) _brakeSampleFullSeconds += TickScale;
        }

        // Park the sample at the apex pass; the verdict lands BrakeCommitDelaySeconds later.
        void ScheduleBrakeCommit()
        {
            // Two apexes inside the delay (a chicane) settle the older one now rather than lose it.
            if (_brakeCommitApexNode >= 0) CommitBrakeLearning();
            _brakeCommitApexNode = _brakeSampleApexNode;
            _brakeCommitAtGameTime = Game.GameTime + (int)(BrakeCommitDelaySeconds * 1000f);
            _brakeCommitSampleSeconds = _brakeSampleSeconds;
            _brakeCommitFullSeconds = _brakeSampleFullSeconds;
            _brakeCommitMaxSlideDeg = _brakeSampleMaxSlideDeg;
        }

        void CommitBrakeLearning()
        {
            if (!ARS.BrakeLearning || _brakeCommitApexNode < 0) return;
            int apexNode = _brakeCommitApexNode;
            float sampleSeconds = _brakeCommitSampleSeconds;
            float maxSlideDeg = _brakeCommitMaxSlideDeg;
            float fullShare = sampleSeconds > 0f ? _brakeCommitFullSeconds / sampleSeconds : 0f;
            _brakeCommitApexNode = -1;
            // The notices ride the track-analysis debug view, so a normal race is quiet.
            bool announce = ARS.DebugToggles[Options.ShowTrackAnalysis];

            float before = BrakeFactorForApex(apexNode);
            // A slide is a verdict of its own, but it takes a real one: 2.5x the live peak is where the lateral
            // curve has fallen all the way to fTractionCurveMin, so that is sliding rather than merely cornering at
            // the limit. The factor then takes a flat cut — sample or no sample, the slide is the evidence.
            if (TRLateralAtSpeed > 0.01f && maxSlideDeg > TRLateralAtSpeed * BrakeSkidPeakMultiple)
            {
                float skidFactor = ARS.Clamp(before - BrakeSkidFactorStep, BrakeMinFactor, BrakeMaxFactor);
                CornerContextForWrite(apexNode).BrakeFactor = skidFactor;
                if (announce) UI.Notify("~b~[ARS]~w~ skid detected > " + skidFactor.ToString("0.00"));
                return;
            }
            // Nothing to score: the car never braked above the sample threshold on the way in.
            if (sampleSeconds <= 0f) return;
            // The error is the *share* of the braking phase spent at full pedal, not the length of the phase:
            // a corner the AI takes without ever pressing hard scores 0, so it asks for the largest correction.
            float step = (BrakeFullFractionTarget - fullShare) * BrakeAdjustGain;
            float factor = ARS.Clamp(before * (1f + step), BrakeMinFactor, BrakeMaxFactor);
            CornerContextForWrite(apexNode).BrakeFactor = factor;
            // The learned factor is otherwise invisible, and this is the only read-out of what the AI decided
            // braking that corner costs.
            if (announce) UI.Notify("~b~[ARS]~w~ " + (int)Math.Round(fullShare * 100f) + "% brake > " + factor.ToString("0.00"));
        }

        int CornerEntranceNode(CornerPoint corner, int apexNode)
        {
            return corner == null ? apexNode : (corner.StartNode >= 0 ? corner.StartNode : OffsetCornerNode(apexNode, -corner.LengthStart));
        }

        int CornerExitNode(CornerPoint corner)
        {
            if (corner == null || corner.Node < 0) return -1;
            return corner.EndNode >= 0 ? corner.EndNode : OffsetCornerNode(corner.Node, corner.LengthEnd);
        }

        const int BrakeTargetLeadMeters = 2;

        // Where the braking target is anchored: a fixed lead before the corner entrance, which the learned factor moves
        // from there toward the apex.
        int BrakeTargetBaseNode(CornerPoint corner, int fallbackNode)
        {
            int entranceNode = CornerEntranceNode(corner, fallbackNode);
            return entranceNode < 0 ? fallbackNode : OffsetCornerNode(entranceNode, -BrakeTargetLeadMeters);
        }

        // Past the braking target the plan expects apex speed; later braking is corner-exit scrub, not approach.
        bool HasPassedBrakingTarget()
        {
            if (NextApexNode < 0) return true;
            CornerPoint corner = _nextApexCorner;
            int targetNode = BrakeTargetBaseNode(corner, NextApexNode);
            if (targetNode < 0) return true;
            int targetDistance = ForwardNodeDistance(targetNode);
            int apexDistance = ForwardNodeDistance(NextApexNode);
            return targetDistance <= 0 || (!ARS.IsPointToPoint && targetDistance > apexDistance);
        }

        float TickScale => (0.001f * TimeSince_lastCoreTick);




        void TranslateSteerToInput()
        {

            if (float.IsNaN(Control.SteerDegrees) || float.IsInfinity(Control.SteerDegrees)) Control.SteerDegrees = 0f;

            float error = Control.SteerDegrees - Control.LastAppliedSteerDegrees;
            float maxDeltaPerTick = SteerSlewRate * TickScale;
            float delta = ARS.Clamp(error, -maxDeltaPerTick, maxDeltaPerTick);
            Control.SteerDegrees = Control.LastAppliedSteerDegrees + delta;
            Control.LastAppliedSteerDegrees = Control.SteerDegrees;

            if (float.IsNaN(Control.SteerInput) || float.IsInfinity(Control.SteerInput)) Control.SteerInput = 0f;

            Control.SteerInput = ARS.Remap(Control.SteerDegrees, -VehicleData.SteeringLock, VehicleData.SteeringLock, -1, 1, true);
            
        }




        public void ComputeTargetSpeed()
        {
            if (BaseBehavior == RacerBaseBehavior.GridWait)
            {
                Brain.CurrentIntention.Speed = 200f;
                return;
            }
            if (BaseBehavior == RacerBaseBehavior.FinishedRace)
            {
                Brain.CurrentIntention.Speed = 20f;
                return;
            }
            if (BaseBehavior == RacerBaseBehavior.FinishedStandStill)
            {
                Brain.CurrentIntention.Speed = 0f;
                return;
            }
            Brain.CurrentIntention.Speed = AiConstants.MaxSpeed;

            float cornerSpd = 999f;
            if (NextApexNode >= 0)
            {
                // Held apexes set the plan, but only the ones this car decided it needs one for; every other
                // corner is route speed's job.
                if (CornerRequiresBraking(NextApexNode)) cornerSpd = Math.Max(2, ApexBrakingSpeed(NextApexNode, NextApexSpeed));
                if (NextApexNode2 >= 0 && CornerRequiresBraking(NextApexNode2))
                    cornerSpd = Math.Min(cornerSpd, Math.Max(2, ApexBrakingSpeed(NextApexNode2, NextApexSpeed2)));
                if (NextApexNode3 >= 0 && CornerRequiresBraking(NextApexNode3))
                    cornerSpd = Math.Min(cornerSpd, Math.Max(2, ApexBrakingSpeed(NextApexNode3, NextApexSpeed3)));
            }
            else if (Brain.Corner != null) cornerSpd = Math.Max(2, ARS.MaxSpeedForBrakingDistance(Brain.Corner.Point, this));

            // Route speed from the triple-check circumradius window.
            float followTrackSpd = RouteIdealSpeedForRadius(Brain.CurrentPerception.CurveRadiusToFollowPoint);

            if (float.IsNaN(cornerSpd) || float.IsInfinity(cornerSpd)) cornerSpd = 999f;
            if (float.IsNaN(followTrackSpd) || float.IsInfinity(followTrackSpd)) followTrackSpd = 999f;
            if (cornerSpd <= 5 && Brain.Corner != null) cornerSpd = ARS.CornerApexSpeed(Brain.Corner.Point, this);

            // Past the braking target the corner's entrance is behind the car, so route speed owns the corner.
            if (NextApexNode >= 0 && HasPassedBrakingTarget()) cornerSpd = 999f;

            // Hill grip loss: exponential model, 15 degrees halves grip.
            {
                float slopeAngleDeg = ARS.RadToDeg(GetFollowPointSlopeAngle());
                float slopeGripFactor = ARS.HillGripFactorFromPitchAngle(slopeAngleDeg, this);
                float slopeSpeedFactor = (float)Math.Sqrt(slopeGripFactor);
                cornerSpd *= slopeSpeedFactor;
                followTrackSpd *= slopeSpeedFactor;
            }

            // Crest/dip vertical curvature grip effect (route speed only).
            int count = ARS.TrackPoints.Count;
            int followNode = (int)ARS.Clamp(CurrentTrackPoint.Node + (int)(Car.Velocity.Length() * RouteLookAheadSeconds), 0, count - 1);
            followTrackSpd *= CrestGripSpeedFactor(OffsetCornerNode(followNode, -3), followNode, OffsetCornerNode(followNode, 3), Brain.CurrentPerception.CurveRadiusToFollowPoint, Car.Velocity.Length(), out _);

            // Pure apex speed for the corner-approach gate.
            _cornerSpd = NextApexNode >= 0 ? NextApexSpeed : (Brain.Corner != null ? ARS.CornerApexSpeed(Brain.Corner.Point, this) : 999f);
            float cornerApexSpeedWithVerticalGrip = _cornerSpd;

            // Corner crest/dip: same check as route, centered on the apex node.
            if (Brain.Corner != null)
            {
                CornerPoint crestCorner = Brain.Corner.Point;
                float cornerCrestFactor = CrestGripSpeedFactor(OffsetCornerNode(crestCorner.Node, -3), crestCorner.Node, OffsetCornerNode(crestCorner.Node, 3), NextApexRadius, _cornerSpd, out _);
                cornerSpd *= cornerCrestFactor;
                cornerApexSpeedWithVerticalGrip *= cornerCrestFactor;
            }

            // Steer-limited speed: max speed for current steer angle before sliding. Blended into route speed so an outside car (less steering) may carry more speed.
            float steerRad = Math.Abs(Control.SteerDegrees) * (float)Math.PI / 180f;
            if (steerRad > 0.001f)
            {
                float turnRadius = VehicleData.WheelBase / (float)Math.Tan(steerRad);
                Brain.CurrentIntention.SteerLimitedSpeed = (float)Math.Sqrt(9.8f * VehicleData.CurrentMechanicalGrip * Math.Max(turnRadius, 1f));
                followTrackSpd = 0.7f * Brain.CurrentIntention.SteerLimitedSpeed + 0.3f * followTrackSpd;
            }
            else
            {
                Brain.CurrentIntention.SteerLimitedSpeed = 999f; // Straight = no steer limit
            }

            // Chicane boost: +10 mph to both corner and route speed while the chicane corner
            // is the active target. No node gate — Brain.Corner updates naturally on apex pass.
            if (Brain.Corner != null && Brain.Corner.Point.IsChicane)
            {
                float chicaneBoost = ARS.MphToMps(10f);
                cornerSpd += chicaneBoost;
                followTrackSpd += chicaneBoost;
            }

            Brain.CurrentIntention.Speed = Math.Min(cornerSpd + ARS.MphToMps(ARS.CornerOffsetMph), followTrackSpd + ARS.MphToMps(ARS.RouteOffsetMph));
            // Physics-limited cornering speed for the current high-speed curve radius.
            Brain.CurrentIntention.CorneringSpeedLimit = (float)Math.Sqrt(9.8f * VehicleData.CurrentMechanicalGrip * Brain.CurrentPerception.HighSpeedCurveRadius) + ARS.MphToMps(ARS.CornerOffsetMph);

            // ChillOut: hold a standoff behind the closest rival ahead.
            if (ActiveManeuver.Type == ManeuverType.ChillOut)
            {
                Rival standoffRival = Brain.Rivals
                    .Where(r => r.RivalRacer != null && r.RelativePosition == RelativePos.Ahead && r.RivalRacer.Car.Exists())
                    .OrderBy(r => r.Distance)
                    .FirstOrDefault();
                if (standoffRival != null && standoffRival.Distance < ChillStandoffDistance)
                {
                    float standoffMargin = ARS.Remap(standoffRival.Distance, 0f, ChillStandoffDistance, 0f, ChillStandoffEscapeMargin, true);
                    Brain.CurrentIntention.Speed = Math.Min(Brain.CurrentIntention.Speed, standoffRival.RivalRacer.Car.Velocity.Length() + standoffMargin);
                }
            }

            // Rubber-banding: slow leaders via a lower speed target (the laggard boost is in ConvertSpeedToPedals).
            // Natural mode skips the penalty while any rival is nearby, so they can race without interference.
            float rbFactor = ComputeRubberBandFactor();
            bool rbNearbyRival = Brain.Rivals.Any(r => r.RivalRacer != null && r.Distance < 100f);
            bool rbFreeZone = Lap <= 1 && CurrentTrackPoint != null && CurrentTrackPoint.Node < 500;
            if (rbFactor < 1f && !rbFreeZone && !(ARS.CurrentRubberbandMode == RubberbandMode.Natural && rbNearbyRival))
                Brain.CurrentIntention.Speed *= rbFactor;
        }

        // Vertical-curvature grip factor over the three-node window centred on a node, as a speed multiplier.
        // 1 means the window is degenerate or a straight; both the aggression and the floor scale with the local
        // radius, so a tight corner is cautious and a straight is aggressive. A corner reads its own entrance and exit
        // as the outer samples, the route a short window around its lookahead point.
        float CrestGripSpeedFactor(int startNode, int centreNode, int endNode, float radius, float entrySpeed, out float rawDeltaGs)
        {
            rawDeltaGs = 0f;
            if (startNode < 0 || centreNode < 0 || endNode < 0) return 1f;
            if (startNode == endNode || startNode == centreNode || endNode == centreNode) return 1f;

            float deltaGs = ARS.HillGripDeltaGs(ARS.TrackPoints[startNode].Position, ARS.TrackPoints[centreNode].Position, ARS.TrackPoints[endNode].Position, entrySpeed);
            rawDeltaGs = deltaGs;
            if (deltaGs > 0f) deltaGs = 0f;
            float aggression = ARS.MapGamma(radius, 100f, 300f, 0f, 1f, 0.5f, true);
            float crestFloor = ARS.MapGamma(radius, 100f, 500f, 0.4f, 0.8f, 0.5f, true);
            if (deltaGs < 0f) deltaGs *= (1f - aggression);
            float verticalGripFactor = Math.Max(1f + deltaGs, crestFloor);
            if (verticalGripFactor < 1f) verticalGripFactor = 1f - Math.Min((1f - verticalGripFactor) * ARS.CrestEffect, 0.9f);
            return (float)Math.Sqrt(verticalGripFactor);
        }

        // Rubber-band factor: <1 for leaders (penalty), >1 for laggards (boost), 1 at center.
        float ComputeRubberBandFactor()
        {
            if (ARS.RubberbandingPct <= 0 || RacePosition <= 0) return 1f;
            int fieldSize = ARS.Racers.Count;
            int centerPos = (fieldSize + 1) / 2;
            Racer playerRacer = ARS.Racers.FirstOrDefault(r => r.Car.Exists() && r.Car == Game.Player.Character.CurrentVehicle);
            if (playerRacer != null) centerPos = playerRacer.RacePosition;
            int maxDistance = Math.Max(Math.Max(centerPos - 1, fieldSize - centerPos), 1);
            float norm = (RacePosition - centerPos) / (float)maxDistance;
            return 1f + norm * 0.33f * (ARS.RubberbandingPct * 0.01f);
        }

        // TrackPoint.Elevation is 90·sin(pitch), so this converts it back to the slope sine.
        const float ElevationToSlopeSine = 1f / 90f;

        float GetFollowPointSlopeAngle()
        {
            int followNode = (int)ARS.Clamp(CurrentTrackPoint.Node + (int)(Car.Velocity.Length() * RouteLookAheadSeconds), 0, ARS.TrackPoints.Count - 1);
            int aheadNode = (int)ARS.Clamp(followNode + 5, 0, ARS.TrackPoints.Count - 1);
            if (aheadNode == followNode) return 0f;

            Vector3 from = ARS.TrackPoints[followNode].Position;
            Vector3 to = ARS.TrackPoints[aheadNode].Position;
            float horizDist = Vector2.Distance(new Vector2(from.X, from.Y), new Vector2(to.X, to.Y));
            if (horizDist < 0.01f) return 0f;
            return Math.Abs((float)Math.Atan2(to.Z - from.Z, horizDist));
        }

        // TCS slip-ratio targets, on the game's per-wheel rotation-slip ratio at wheel+0x174: 0 free rolling, the
        // curve peaks at 1.0, degrades linearly to CurveMin at 2.5 (sfTractionPeakAngle / sfTractionMinAngle, leaked
        // wheel.cpp:94-97) and is flat past it. More negative = more spin. Scale confirmed by the driver against
        // live values: the lock ratio runs to ~15 at a stopped wheel, mingrip onset at 2.5.
        const float IdealWheelspinLaunchRatio = 1.75f;   // standstill: mid-traction, the CurveMax-CurveMin midpoint
        const float IdealWheelspinPeakRatio = 1.0f;      // normal driving: the traction peak
        const float IdealWheelspinOffTrackRatio = 0.5f;  // off-track: half the peak coefficient
        const float IdealWheelspinLaunchTaperEndMph = 30f;
        const float SlipTargetGripFloor = 0.3f;

        // One home for the throttle reason caps: each reason states the level it wants, its field glides toward
        // that level at the shared rate both ways, and ConvertSpeedToPedals takes the minimum against the plan's
        // own throttle and tags the winner. The level is the logic half, the shared rate the clock half.
        const float ReasonCapSlewRate = 3.5f;
        const float TcsCapFloor = 0.25f;
        // The curve is flat from here (CurveMin at 2.5): both cap levels bottom exactly at the knee — past it no
        // deeper slip earns a deeper cut.
        const float SlipCurveKnee = 2.5f;
        const float YieldThrottleLevel = 0.5f;
        const float OffTrackThrottleLevel = 0f;
        const float OffTrackSafeSpeedMph = 20f;
        const float OffTrackFullThrottleMph = 15f;

        // Instability, the replacement's first cut: off-road the chassis is thrown around and the two consequences
        // are losing grip and losing control. The ride height is captured at Launch, where the car is known to be at
        // rest, so a rise above it is the chassis unloading off its suspension; the yaw side is the friction circle,
        // since at this speed the tyres allow at most grip*g of lateral acceleration.
        const float InstabilityRideHeightMargin = 0.1f;
        const float InstabilityMinSpeedMps = 8f;
        const float InstabilityYawTolerance = 1.2f;
        const float InstabilityThrottleLevel = 0f;

        // The overspeed correction only arms while the car is going straight and up to speed: steering or slide of
        // its own puts lateral work and bump transients into the longitudinal read, and below the speed gate the
        // launch assist is what it would otherwise be reading.
        const float OverspeedArmMaxSteerDegrees = 2f;
        const float OverspeedArmMaxSlideDegrees = 2f;
        const float OverspeedArmMinSpeedMph = 30f;

        // Cutting is fast, recovering is deliberate: the shared rate going down, half of it coming back.
        const float ReasonCapRecoveryScale = 0.5f;

        float GlideCap(float current, float level)
        {
            float rate = level < current ? ReasonCapSlewRate : ReasonCapSlewRate * ReasonCapRecoveryScale;
            float step = rate * TickScale;
            return ARS.Clamp(current + ARS.Clamp(level - current, -step, step), 0f, 1f);
        }

        void UpdateThrottleReasonCaps()
        {
            Control.MaxThrottleFromTCS = GlideCap(Control.MaxThrottleFromTCS, TcsCapLevel());
            Control.MaxThrottleFromInstability = GlideCap(Control.MaxThrottleFromInstability, IsUnstable() ? InstabilityThrottleLevel : 1f);

            bool overspeedArmed = Math.Abs(Control.SteerDegrees) < OverspeedArmMaxSteerDegrees && Math.Abs(VehicleData.SlideAngle) < OverspeedArmMaxSlideDegrees && Car.Velocity.Length() > ARS.MphToMps(OverspeedArmMinSpeedMph);
            float overspeedLevel = 1f;
            if (ARS.OverspeedEnabled && overspeedArmed && VehicleData.OverspeedExcessGs > 0f)
                overspeedLevel = ARS.Clamp(1f - (float)Math.Floor(VehicleData.OverspeedExcessGs / 0.1f) * 0.5f, 0f, 1f);
            Control.MaxThrottleFromOverspeed = GlideCap(Control.MaxThrottleFromOverspeed, overspeedLevel);

            float rivalLevel = 1f;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || r.RelativePosition != RelativePos.Ahead) continue;
                if (ARS.IsBetween(r.TimeToReach, 0f, 3f)) rivalLevel = Math.Min(rivalLevel, ARS.Remap(r.TimeToReach, 0f, 3f, 0f, 1f, true));
                if (ARS.IsBetween(r.FrontGap, 0f, 1f)) rivalLevel = Math.Min(rivalLevel, ARS.Remap(r.FrontGap, 0f, 1f, 0f, 1f, true));
            }
            Control.MaxThrottleFromRival = GlideCap(Control.MaxThrottleFromRival, rivalLevel);

            Control.MaxThrottleFromChillOut = GlideCap(Control.MaxThrottleFromChillOut, ActiveManeuver.Type == ManeuverType.ChillOut ? ChillThrottleCap : 1f);
            Control.MaxThrottleFromYield = GlideCap(Control.MaxThrottleFromYield, ActiveManeuver.Type == ManeuverType.Yield && ActiveManeuver.Target != null ? YieldThrottleLevel : 1f);

            float offTrackThrottle = 1f;
            if (OutOfTrackDistance() > 0f) offTrackThrottle = ARS.Remap(ARS.MpsToMph(Car.Velocity.Length()), OffTrackSafeSpeedMph, OffTrackFullThrottleMph, OffTrackThrottleLevel, 1f, true);
            Control.MaxThrottleFromOffTrack = GlideCap(Control.MaxThrottleFromOffTrack, offTrackThrottle);

            float recoveryThrottle = 1f;
            if (_stuckPhase == StuckPhase.Drive) recoveryThrottle = ARS.Remap(ARS.MpsToMph(Car.Velocity.Length()), RecoveryDriveMph + RecoveryCoastBandMph, RecoveryDriveMph, 0f, 1f, true);
            Control.MaxThrottleFromRecovery = GlideCap(Control.MaxThrottleFromRecovery, recoveryThrottle);
        }

        // Both signals are read every tick: the old system latched them at 3 Hz and missed crests shorter than the
        // sample period. Riding above the at-rest height means the chassis is unloaded, bouncing or airborne;
        // demanding more lateral acceleration than the grip allows means the car is rotating, not cornering.
        bool IsUnstable()
        {
            if (_restHeightAboveGround > 0f && Car.HeightAboveGround - _restHeightAboveGround > InstabilityRideHeightMargin) return true;

            float speed = Car.Velocity.Length();
            if (speed < InstabilityMinSpeedMps) return false;
            float lateralDemand = ARS.DegToRad(Math.Abs(VehicleData.YawRotationPerSecondDegrees)) * speed;
            return lateralDemand > VehicleData.CurrentMechanicalGrip * Handling.Gravity * InstabilityYawTolerance;
        }

        float TcsCapLevel()
        {
            if (!ARS.TcsEnabled) return 1f;
            float wheelspin = ARS.MaxWheelSlip(Car);

            float IdealWheelspin;
            if (OutOfTrackDistance() > 0f)
            {
                // The half-peak point is a deliberate policy, not a grip-scaled setpoint: off-track halves the target itself.
                IdealWheelspin = -IdealWheelspinOffTrackRatio;
            }
            else
            {
                float gripScale = ARS.Clamp(GroundGripMultiplier, SlipTargetGripFloor, 1f);
                IdealWheelspin = -ARS.Remap(ARS.MpsToMph(Car.Velocity.Length()),
                    IdealWheelspinLaunchTaperEndMph, 0f, IdealWheelspinPeakRatio * gripScale, IdealWheelspinLaunchRatio * gripScale, true);
            }

            float absTarget = -IdealWheelspin;
            float spinDepth = Math.Max(0f, IdealWheelspin - wheelspin);
            float depthShare = ARS.Clamp(spinDepth / (SlipCurveKnee - absTarget), 0f, 1f);
            return 1f - depthShare * (1f - TcsCapFloor);
        }

        // ABS: the brake-side twin, containment where TCS regulates: TCS holds a curve point, ABS guards the flat
        // region past mingrip where full lock lives — a stopped wheel reads ~15 (driver-observed), so the cut starts
        // at the knee (2.5) and floors by 5. Lock reads positive on the same +0x174 ratio (leaked wheel.cpp:5233
        // spin negative, :5268 lock positive); the engine's own free ABS clamps coarsely; FLAG_WD_ABS is left alone;
        // the driver sign check is the first test.
        const float AbsFloorSlip = 5f;
        const float AbsBrakeFloor = 0.25f;

        void UpdateBrakeReasonCaps()
        {
            bool countersteering = IsFullCountersteer();
            Control.MaxBrakeFromABS = GlideCap(Control.MaxBrakeFromABS, AbsCapLevel());
            Control.MaxBrakeFromCountersteer = GlideCap(Control.MaxBrakeFromCountersteer, countersteering ? 0f : 1f);
        }

        float AbsCapLevel()
        {
            if (!ARS.AbsEnabled) return 1f;
            float lockSlip = ARS.MaxWheelLockSlip(Car);

            float lockDepth = Math.Max(0f, lockSlip - SlipCurveKnee);
            float depthShare = ARS.Clamp(lockDepth / (AbsFloorSlip - SlipCurveKnee), 0f, 1f);
            return 1f - depthShare * (1f - AbsBrakeFloor);
        }

        // The one throttle composition: the plan's throttle against every reason field, the argmin named into the
        // tag (first reason wins ties). Offtrack's cut and the countersteer replacement pre-empt the ordinary
        // reasons; grid wait owns the pedal outright and stuck recovery re-tags after.
        float ComposeThrottleCap(float baseThrottle, bool offtrackLimited, bool countersteering)
        {
            float ceiling = Math.Min(Control.MaxThrottleFromTCS, Control.MaxThrottleFromInstability);
            ceiling = Math.Min(ceiling, Control.MaxThrottleFromOverspeed);
            ceiling = Math.Min(ceiling, Control.MaxThrottleFromRival);
            ceiling = Math.Min(ceiling, Control.MaxThrottleFromChillOut);
            ceiling = Math.Min(ceiling, Control.MaxThrottleFromYield);
            ceiling = Math.Min(ceiling, Control.MaxThrottleFromOffTrack);
            ceiling = Math.Min(ceiling, Control.MaxThrottleFromRecovery);
            Control.MaxThrottle = ceiling;

            float binding = baseThrottle;
            ThrottleReason reason = ThrottleReason.Plan;
            if (Control.MaxThrottleFromTCS < binding) { binding = Control.MaxThrottleFromTCS; reason = ThrottleReason.Tcs; }
            if (Control.MaxThrottleFromInstability < binding) { binding = Control.MaxThrottleFromInstability; reason = ThrottleReason.Instability; }
            if (Control.MaxThrottleFromOverspeed < binding) { binding = Control.MaxThrottleFromOverspeed; reason = ThrottleReason.Overspeed; }
            if (Control.MaxThrottleFromRival < binding) { binding = Control.MaxThrottleFromRival; reason = ThrottleReason.Rival; }
            if (Control.MaxThrottleFromChillOut < binding) { binding = Control.MaxThrottleFromChillOut; reason = ThrottleReason.ChillOut; }
            if (Control.MaxThrottleFromYield < binding) { binding = Control.MaxThrottleFromYield; reason = ThrottleReason.Yield; }
            if (Control.MaxThrottleFromOffTrack < binding) { binding = Control.MaxThrottleFromOffTrack; reason = ThrottleReason.Offtrack; }
            if (Control.MaxThrottleFromRecovery < binding) { binding = Control.MaxThrottleFromRecovery; reason = ThrottleReason.StuckRecovery; }
            if (offtrackLimited) reason = ThrottleReason.Offtrack;
            if (countersteering) reason = ThrottleReason.Countersteer;
            if (BaseBehavior == RacerBaseBehavior.GridWait) reason = ThrottleReason.GridWait;

            Control.ThrottleReason = reason;
            Control.ThrottleReasonLevel = binding;
            return binding;
        }

        // The brake-side composition: the plan's braking against the ABS and countersteer caps, the argmin named
        // into the tag; offtrack's brake command pre-empts and grid wait owns the tag while waiting.
        float ComposeBrakeCap(float baseBrake, bool offtrackBrakeCommand, bool countersteering)
        {
            Control.MaxBrake = Math.Min(Control.MaxBrakeFromABS, Control.MaxBrakeFromCountersteer);

            float binding = baseBrake;
            BrakeReason reason = BrakeReason.Plan;
            if (Control.MaxBrakeFromABS < binding) { binding = Control.MaxBrakeFromABS; reason = BrakeReason.Abs; }
            if (Control.MaxBrakeFromCountersteer < binding) { binding = Control.MaxBrakeFromCountersteer; reason = BrakeReason.Countersteer; }
            if (offtrackBrakeCommand) reason = BrakeReason.Offtrack;
            if (countersteering) reason = BrakeReason.Countersteer;
            if (BaseBehavior == RacerBaseBehavior.GridWait) reason = BrakeReason.GridWait;

            Control.BrakeReason = reason;
            Control.BrakeReasonLevel = binding;
            return binding;
        }
        Rival NearestRival()
        {
            Rival nearest = null;
            float best = float.MaxValue;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || !r.RivalRacer.Car.Exists()) continue;
                if (r.Distance < best) { best = r.Distance; nearest = r; }
            }
            return nearest;
        }

        // The cards ride the fast core pass. Track state, perception and the car's own velocity were rebuilt earlier in
        // this same core tick; the rival records are the half-second publish's, owned by the slower rival scan, and
        // Yield's pressure belongs to the pressure beat. The crowd card keeps the slow beat because it counts the field.
        void ConsiderManeuvers()
        {
            if (ControlledByPlayer || BaseBehavior != RacerBaseBehavior.Race || ARS.Racers.Count < 1) return;

            // Force-disable maneuvers armed for more than 8s without firing.
            if (ActiveManeuver.Type != ManeuverType.None && Game.GameTime - ActiveManeuver.LastEnabled > 8000)
            {
                ActiveManeuver.Type = ManeuverType.None;
                ActiveManeuver.Target = null;
            }

            // Divebomb cleanup: off once we pass the armed apex.
            if (ActiveManeuver.Type == ManeuverType.DiveBomb && HasPassedApex(_divebombApexNode))
            {
                ActiveManeuver.Type = ManeuverType.None;
                ActiveManeuver.Target = null;
                _divebombApexNode = -1;
            }

            // DefendLane fold: off once we pass the defended apex or the target gets past us.
            if (ActiveManeuver.Type == ManeuverType.DefendLane)
            {
                bool targetTrails = false;
                foreach (Rival r in Brain.Rivals)
                {
                    if (r.RivalRacer != ActiveManeuver.Target) continue;
                    targetTrails = r.RelativePosition != RelativePos.Ahead;
                    break;
                }
                bool lostTarget = ActiveManeuver.Target == null || !ActiveManeuver.Target.Car.Exists() || !targetTrails;

                if (lostTarget || HasPassedApex(_defendApexNode))
                {
                    ActiveManeuver.Type = ManeuverType.None;
                    ActiveManeuver.Target = null;
                    _defendApexNode = -1;
                }
            }

            // Card model: while no card is in play, the hand is evaluated in priority order.
            // Nitro resolves instantly (burn lives in _nitrousActiveUntil), so it never occupies the slot.
            if (ActiveManeuver.Type == ManeuverType.None) TryPlayNitrousCard();

            if (ActiveManeuver.Type == ManeuverType.None) TryPlayDefendLaneCard();

            if (ActiveManeuver.Type == ManeuverType.None) TryPlayDivebombCard();

            if (ActiveManeuver.Type == ManeuverType.None) TryPlayYieldCard();
        }

        // The crowd card counts the whole field, so it keeps the slow beat.
        void ConsiderCrowdCard()
        {
            if (ControlledByPlayer || BaseBehavior != RacerBaseBehavior.Race || ARS.Racers.Count < 1) return;

            // ChillOut cleanup: off once the pack around us thins out.
            if (ActiveManeuver.Type == ManeuverType.ChillOut && RivalsWithinDistance(ChillRivalCrowdDistance) < ChillCrowdCount)
            {
                ActiveManeuver.Type = ManeuverType.None;
                ActiveManeuver.Target = null;
            }

            // ChillOut: only when fast enough for bunching to matter, in a dense pack of better-placed cars.
            if (ActiveManeuver.Type == ManeuverType.None && ARS.MpsToMph(Car.Velocity.Length()) >= ChillMinSpeedMph && RivalsWithinDistance(ChillRivalCrowdDistance) >= ChillCrowdCount)
            {
                Rival closestRival = NearestRival();
                if (closestRival != null)
                {
                    ActiveManeuver.Type = ManeuverType.ChillOut;
                    ActiveManeuver.Target = closestRival.RivalRacer;
                    ActiveManeuver.LastEnabled = Game.GameTime;
                }
            }
        }

        int ForwardNodeDistance(int targetNode)
        {
            int fwd = targetNode - CurrentTrackPoint.Node;
            if (!ARS.IsPointToPoint && fwd < 0) fwd += ARS.TrackPoints.Count;
            return fwd;
        }

        bool IsWithinCorner(CornerPoint corner)
        {
            if (corner == null || corner.StartNode < 0 || corner.EndNode < 0) return false;

            if (ARS.IsPointToPoint)
                return CurrentTrackPoint.Node >= corner.StartNode
                    && CurrentTrackPoint.Node <= corner.EndNode;

            if (corner.StartNode <= corner.EndNode)
                return CurrentTrackPoint.Node >= corner.StartNode
                    && CurrentTrackPoint.Node <= corner.EndNode;

            return CurrentTrackPoint.Node >= corner.StartNode
                || CurrentTrackPoint.Node <= corner.EndNode;
        }

        void UpdateNitrous()
        {
            if (!ARS.NitrousEnabled) return;

            if (Game.GameTime < _nitrousActiveUntil) return;   // the burn's power rides ApplyPowerMultiplier
            if (_nitrousActiveUntil > 0) StopNitrous();
        }

        // Valid when a shot is available and the straight is long enough; appropriate when contested, defended,
        // lonely, or spent near the finish with a rival close.
        bool TryPlayNitrousCard()
        {
            if (!ARS.NitrousEnabled || Lap <= _nitrousLapUsed) return false;
            if (Control.Brake > 0f) return false;
            if (OutOfTrackDistance() > 0f) return false;
            if (Math.Abs(Control.SteerDegrees) >= NitrousMaxSteerDegrees) return false;
            if (Control.Throttle < NitrousMinThrottle) return false;
            if (!IsAwd() && Function.Call<int>((Hash)MaxDriveGearHash, Car) > 3 && Function.Call<int>((Hash)CurrentDriveGearHash, Car) <= 2) return false;
            if (NextApexNode < 0) return false;

            float speed = Car.Velocity.Length();
            Rival closestRival = NearestRival();

            // Finish spender: with a rival nearby the burn near the line is always worth it,
            // so the 8s corner gate no longer applies.
            bool finishSpender = closestRival != null && closestRival.Distance <= NitrousNearbyRivalDistance
                && RemainingRaceDistanceMeters() <= speed * (NitrousDurationMs / 1000f) + NitrousFinishExtraDistance;
            if (!finishSpender)
            {
                CornerPoint corner = _nextApexCorner;
                // Circuit wrap makes a just-behind entrance read a lap away; veto in-corner shots.
                if (corner != null && IsWithinCorner(corner)) return false;
                int entranceNode = corner == null
                    ? NextApexNode
                    : (corner.StartNode >= 0 ? corner.StartNode : OffsetCornerNode(NextApexNode, -corner.LengthStart));
                if (ForwardNodeDistance(entranceNode) / Math.Max(speed, 1f) < NitrousCornerLookaheadSeconds) return false;
            }

            bool rivalNearbyFaster = closestRival != null && closestRival.Speed > speed;
            bool rivalBehindIncoming = false;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || !r.RivalRacer.Car.Exists() || r.RelativePosition != RelativePos.Behind || r.Speed <= speed) continue;
                if (r.Distance / (r.Speed - speed) < NitrousDefenseReachSeconds) { rivalBehindIncoming = true; break; }
            }
            bool lonelyClear = closestRival == null
                && speed < Handling.EstimatedTopSpeed * NitrousLonelySpeedFraction
                && ForwardNodeDistance(NextApexNode) > NitrousLonelyMinApexDistance;
            if (!rivalNearbyFaster && !rivalBehindIncoming && !lonelyClear && !finishSpender) return false;

            StartNitrous();
            return true;
        }

        const float DivebombEntranceSeconds = 5f;
        const float DivebombFullThrottle = 0.99f;
        const float DivebombOverlapReachSeconds = 2f;

        // Divebomb card: at full throttle into a corner under 5s away, with an open side to commit
        // into (at most one rival wall active), dive the closest rival we overlap or will reach within 2s.
        bool TryPlayDivebombCard()
        {
            if (Brain.Corner == null) return false;
            if (Control.Throttle < DivebombFullThrottle) return false;
            if (_activeRivalWallCount > 1) return false;

            int apexNode = Brain.Corner.Point.Node;
            int entranceNode = CornerEntranceNode(Brain.Corner.Point, apexNode);
            if (entranceNode < 0) return false;
            float timeToEntrance = ForwardNodeDistance(entranceNode) / Math.Max(Car.Velocity.Length(), 1f);
            if (timeToEntrance > DivebombEntranceSeconds) return false;

            Rival diveTarget = null;
            float nearestDive = float.MaxValue;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || !r.RivalRacer.Car.Exists()) continue;
                bool overlaps = r.RelativePosition == RelativePos.Left || r.RelativePosition == RelativePos.Right || r.TimeToContact <= DivebombOverlapReachSeconds;
                if (!overlaps) continue;
                if (r.Distance < nearestDive) { nearestDive = r.Distance; diveTarget = r; }
            }
            if (diveTarget == null) return false;

            ActiveManeuver.Type = ManeuverType.DiveBomb;
            ActiveManeuver.Target = diveTarget.RivalRacer;
            ActiveManeuver.LastEnabled = Game.GameTime;
            _divebombApexNode = apexNode;
            _divebombBrakeBonus = ARS.GetRandomInt(2, 9) / 100f;
            return true;
        }

        // DefendLane card: cover the inside so a faster chaser that reaches the entrance no later
        // than we do can't dive underneath.
        bool TryPlayDefendLaneCard()
        {
            if (Brain.Corner == null) return false;

            int apexNode = Brain.Corner.Point.Node;
            int entranceNode = CornerEntranceNode(Brain.Corner.Point, apexNode);
            if (entranceNode < 0) return false;
            float myTimeToEntrance = ForwardNodeDistance(entranceNode) / Math.Max(Car.Velocity.Length(), 1f);
            if (!ARS.IsBetween(myTimeToEntrance, 1f, 3f)) return false;

            Rival defenderTarget = null;
            float nearestDefender = float.MaxValue;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null || !r.RivalRacer.Car.Exists()) continue;
                if (r.RivalRacer.ActiveManeuver.Type == ManeuverType.DiveBomb || r.RivalRacer.ActiveManeuver.Type == ManeuverType.DefendLane) continue;
                bool closes = r.RelativePosition == RelativePos.Behind && r.Distance <= 30f && r.ForwardSpeedGap < 0f;
                bool arrivesNoLater = r.RivalRacer.ForwardNodeDistance(entranceNode) / Math.Max(r.Speed, 1f) <= myTimeToEntrance;
                if (!closes || !arrivesNoLater) continue;
                if (r.Distance < nearestDefender) { nearestDefender = r.Distance; defenderTarget = r; }
            }
            if (defenderTarget == null) return false;

            ActiveManeuver.Type = ManeuverType.DefendLane;
            ActiveManeuver.Target = defenderTarget.RivalRacer;
            ActiveManeuver.LastEnabled = Game.GameTime;
            _defendApexNode = apexNode;
            return true;
        }

        // Yield card: let a faster overlapping rival by when they carry far more pressure into the entrance.
        bool TryPlayYieldCard()
        {
            if (Brain.Corner == null) return false;

            Rival closestRival = NearestRival();
            if (closestRival == null) return false;

            float pressureDiff = closestRival.RivalRacer.Pressure - Pressure;
            bool inOverlap = closestRival.RelativePosition == RelativePos.Left || closestRival.RelativePosition == RelativePos.Right;
            int entranceNode = CornerEntranceNode(Brain.Corner.Point, Brain.Corner.Point.Node);
            float timeToEntrance = ForwardNodeDistance(entranceNode) / Math.Max(Car.Velocity.Length(), 1f);
            if (pressureDiff <= 30f || !inOverlap || !ARS.IsBetween(timeToEntrance, 0.5f, 2f)
                || closestRival.Distance > 20f
                || closestRival.Speed <= Car.Velocity.Length()) return false;

            ActiveManeuver.Type = ManeuverType.Yield;
            ActiveManeuver.Target = closestRival.RivalRacer;
            ActiveManeuver.LastEnabled = Game.GameTime;
            return true;
        }

        float RemainingRaceDistanceMeters()
        {
            int nodeCount = ARS.TrackPoints.Count;
            if (ARS.IsPointToPoint) return nodeCount - CurrentTrackPoint.Node;
            float totalLaps = ARS.RaceMenuStore.GetInt("Laps", 6);
            return Math.Max(0f, (totalLaps + 1f - Lap) * nodeCount - CurrentTrackPoint.Node);
        }

        // Fastest completed lap, or null when none was timed. Lap 1 is never timed, for anyone.
        public TimeSpan? BestLap()
        {
            TimeSpan? best = null;
            foreach (TimeSpan lap in LapTimes)
            {
                if (!best.HasValue || lap < best.Value) best = lap;
            }
            return best;
        }

        void StartNitrous()
        {
            Function.Call(Hash.REQUEST_NAMED_PTFX_ASSET, NitrousPtfxAsset);
            Function.Call((Hash)FullyChargeNitrousHash, Car);
            Function.Call((Hash)OverrideNitrousLevelHash, Car, true, 1.0f, 1.0f, 100.0f, false);
            _nitrousActiveUntil = Game.GameTime + NitrousDurationMs;
            _nitrousLapUsed = Lap;
        }

        public bool TryFireNitrous()
        {
            if (Game.GameTime < _nitrousActiveUntil || NitroChargedLap < Lap || Lap <= _nitrousLapUsed) return false;
            StartNitrous();
            return true;
        }

        void StopNitrous()
        {
            Function.Call((Hash)OverrideNitrousLevelHash, Car, false, 10.0f, 0.0f, 100.0f, true);
            _nitrousActiveUntil = 0;
        }

        void UpdateYield()
        {
            if (ActiveManeuver.Type != ManeuverType.Yield) return;

            // Exit: target is >10m away
            if (ActiveManeuver.Target != null && ActiveManeuver.Target.Car.Exists())
            {
                float dist = Car.Position.DistanceTo(ActiveManeuver.Target.Car.Position);
                if (dist > 10f)
                {
                    ActiveManeuver.Type = ManeuverType.None;
                    ActiveManeuver.Target = null;
                }
            }
            else
            {
                ActiveManeuver.Type = ManeuverType.None;
                ActiveManeuver.Target = null;
            }
        }
        public void UpdateTickData()
        {
            Vector3 cSpeed = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, false);

            // Sample acceleration at a fixed 20ms interval (10 samples × 20ms = 200ms window).
            int now = Game.GameTime;
            int elapsed = now - VehicleData.LastAccelSampleTime;
            if (elapsed >= VehicleState.AccelIntervalMs)
            {
                VehicleData.LastAccelSampleTime = now;
                float dt = Math.Max(elapsed * 0.001f, 0.001f);
                Vector3 accel = (cSpeed - _lastSpeed) / dt;
                VehicleData.AccelSamples[VehicleData.AccelHead] = accel;
                VehicleData.AccelHead = (VehicleData.AccelHead + 1) % VehicleState.AccelWindow;
                if (VehicleData.AccelCount < VehicleState.AccelWindow) VehicleData.AccelCount++;
                _lastSpeed = cSpeed;
                VehicleData.AccumulateLapPeaks(accel, cSpeed, Car.ForwardVector);
            }

            VehicleData.SpeedVectorLocal = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true);
        }


        // Kinematic projection: pos + v*t + 0.5*a*t^2. Default t=1 (1s).
        public Vector3 ProjectAhead(float seconds = 1f)
        {
            return Car.Position + Car.Velocity * seconds + 0.5f * VehicleData.AverageAcceleration * seconds * seconds;
        }

        static Vector3 RotateZ(Vector3 v, float degrees)
        {
            float rad = degrees * (float)Math.PI / 180f;
            float cos = (float)Math.Cos(rad);
            float sin = (float)Math.Sin(rad);
            return new Vector3(v.X * cos - v.Y * sin, v.X * sin + v.Y * cos, v.Z);
        }



        // Every debug visual of the focus car, behind one gate: these blocks used to sit in ProcessTick with the
        // same focus test written out twice.
        void DrawDebug()
        {
            if (ControlledByPlayer || ARS.DebugFocusRacer != this) return;

            if (ARS.DebugToggles[Options.ShowInputs]) DrawInputTrail();
            if (ARS.DebugToggles[Options.ShowTrackAnalysis]) DrawSteerAngles();
        }

        const float FanLineLength = 5f;
        const float FanPursuitHeight = 0.05f;
        const float FanDamperHeight = 0.10f;
        const float FanSteerHeight = 0.15f;
        const float FanCeilingHeight = 0.20f;

        // The steer-angle fan over the roof. The model origin sits near the ground, so the roof is at the full
        // model height, not half of it — half-height floats at mid-body and the relative height varies per car.
        void DrawSteerAngles()
        {
            Vector3 origin = Car.Position + new Vector3(0, 0, VehicleData.ModelDimensions.Z + 0.2f);
            Vector3 fwd = Car.ForwardVector;

            // Yellow: pursuit angle (what the lane tracking wants)
            ARS.DrawLine(origin + new Vector3(0, 0, FanPursuitHeight), origin + new Vector3(0, 0, FanPursuitHeight) + RotateZ(fwd, _steerPursuitDeg) * FanLineLength, Color.Yellow);

            // Red: pursuit angle + damper
            ARS.DrawLine(origin + new Vector3(0, 0, FanDamperHeight), origin + new Vector3(0, 0, FanDamperHeight) + RotateZ(fwd, _steerPursuitDeg + _damperTermDeg) * FanLineLength, Color.Red);

            // White: the final slewed steer (what the wheels actually request), carrying the pedals and their limits.
            Vector3 steerBase = origin + new Vector3(0, 0, FanSteerHeight);
            Vector3 steerDir = RotateZ(fwd, Control.SteerDegrees);
            ARS.DrawLine(steerBase, steerBase + steerDir * FanLineLength, Color.White);

            // Green throttle (reverse included) and red brake: both run back-to-front as their input rises, so the
            // two spheres read against each other on one axis.
            DrawSteerLineSphere(steerBase, steerDir, FanLineLength, Math.Abs(Control.Throttle), SteerLinePedalSize, Color.Green);
            DrawSteerLineSphere(steerBase, steerDir, FanLineLength, Control.Brake, SteerLinePedalSize, Color.Red);
            DrawPedalLimits(steerBase, steerDir, FanLineLength);

            // Cyan: the limiter's ceiling in either direction, the walls the steer lines are constrained against.
            float ceilingDeg = ResolveSteerCeiling(ARS.GetForwardSpeed(Car));
            ARS.DrawLine(origin + new Vector3(0, 0, FanCeilingHeight), origin + new Vector3(0, 0, FanCeilingHeight) + RotateZ(fwd, ceilingDeg) * FanLineLength, Color.Cyan);
            ARS.DrawLine(origin + new Vector3(0, 0, FanCeilingHeight), origin + new Vector3(0, 0, FanCeilingHeight) + RotateZ(fwd, -ceilingDeg) * FanLineLength, Color.Cyan);
        }

        public void ProcessTick()
        {
            UpdateTickData();

            if (ARS.DebugToggles[Options.ShowInputs] && !ControlledByPlayer) SampleInputTrail();
            DrawDebug();

            _appliedThrottleLastFrame = VehicleMemory.GetThrottle(Car);

            // Judge the player's launch input once per race: sample while the window is open (a keyboard pedal is a
            // hard 0/1, a gamepad is not), then freeze the verdict so the AI's shaping cannot change mid-race.
            if (ControlledByPlayer && ARS.PlayerLaunchTestActive)
            {
                float playerThrottle = Math.Abs(_appliedThrottleLastFrame);
                if (playerThrottle > ModulatedThrottleLow && playerThrottle < ModulatedThrottleHigh) ARS.PlayerModulatesThrottle = true;

                if (ARS.GetForwardSpeed(Car) > LaunchShapeMaxSpeed)
                {
                    ARS.PlayerLaunchTestActive = false;
                    ARS.Log(ARS.LogImportance.Info, ARS.PlayerModulatesThrottle
                        ? "Player launch input: modulatable — AI launch throttle capped at " + ModulatedThrottleCap
                        : "Player launch input: binary 0/1 — AI launch throttle quantised");
                }
            }

            if (!ControlledByPlayer)
            {
                ApplyInputs();
            }

            UpdatePowerCompensation();
            ApplyPowerMultiplier();
        }

        // One sample every half metre travelled measured against the last recorded point; the oldest drops off at the cap.
        void SampleInputTrail()
        {
            Vector3 position = Car.Position;
            if (_inputTrail.Count > 0 && position.DistanceTo2D(_inputTrail[_inputTrail.Count - 1].Position) < InputTrailSampleSpacing) return;

            bool chevron = _inputTrail.Count == 0;
            if (!chevron)
            {
                _inputTrailChevronDistance += position.DistanceTo2D(_inputTrail[_inputTrail.Count - 1].Position);
                chevron = _inputTrailChevronDistance >= InputTrailChevronSpacing;
            }
            if (chevron) _inputTrailChevronDistance = 0f;

            _inputTrail.Add(new InputTrailSample { Position = position, Input = Control.Throttle - Control.Brake, Chevron = chevron });
            if (_inputTrail.Count > InputTrailMaxSamples) _inputTrail.RemoveAt(0);
        }

        void DrawInputTrail()
        {
            int sites = 0;
            for (int i = 1; i < _inputTrail.Count; i++) if (_inputTrail[i].Chevron) sites++;
            if (sites == 0) return;

            // A sample carries the car's origin, whose height over the road differs per model, so the rest height
            // cached at the grid comes off first and every car's trail sits the same distance over the tarmac.
            float restHeight = _restHeightAboveGround > 0f ? _restHeightAboveGround : 0f;
            int site = 0;
            for (int i = 1; i < _inputTrail.Count; i++)
            {
                InputTrailSample sample = _inputTrail[i];
                if (!sample.Chevron) continue;

                int fade = Math.Min(site, sites - 1 - site);
                site++;

                Vector3 heading = sample.Position - _inputTrail[i - 1].Position;
                heading.Z = 0f;
                if (heading.LengthSquared() < 0.0001f) continue;
                heading.Normalize();

                float input = ARS.Clamp(sample.Input, -1f, 1f);
                // The rotation's middle slot sits at 90 to lie the chevron flat, which leaves the first slot a yaw and
                // the last a roll - no slot left to pitch with. So the pedal tilts the direction vector instead, and
                // the rotation stays at the flat, pointing configuration every marker site shares.
                float pitchRad = input * InputTrailChevronPitch * ((float)Math.PI / 180f);
                Vector3 direction = heading * (float)Math.Cos(pitchRad) - Vector3.WorldUp * (float)Math.Sin(pitchRad);
                float groundZ = sample.Position.Z - restHeight + InputTrailChevronGroundClearance;
                Vector3 position = new Vector3(sample.Position.X, sample.Position.Y, groundZ - input * InputTrailChevronHeightGain);

                // Both ends fade over the first and last few sites, so a site arriving and the oldest leaving do not pop.
                Color colour = fade < InputTrailFadeCount ? Color.FromArgb(255 * (fade + 1) / (InputTrailFadeCount + 1), InputColour(input)) : InputColour(input);
                World.DrawMarker(MarkerType.ChevronUpx1, position, direction, new Vector3(89f, 90f, -90f), new Vector3(InputTrailChevronSize, InputTrailChevronSize, InputTrailChevronSize), colour, false, false, 2, false, "", "", false);
            }
        }

        const float SteerLinePedalSize = 0.08f;

        // One sphere at a level on the steer line: the base is no input, the tip is full, so a level reads as its
        // distance along the line whatever the sphere marks.
        void DrawSteerLineSphere(Vector3 basePt, Vector3 dir, float length, float level, float size, Color color)
        {
            World.DrawMarker(MarkerType.DebugSphere, basePt + dir * length * ARS.Clamp(level, 0f, 1f), Vector3.Zero, Vector3.Zero, new Vector3(size, size, size), color);
        }

        // The reason limits ride the white steer line with the applied pedals, so every sphere on the line shares
        // one scale: the base is no input and the tip is full. A white sphere is an override command; a coloured
        // sphere is a per-reason limit, drawn only where it bites. Highest level goes down first, so the binding
        // cap paints last.
        const float PedalLimitReasonSize = 0.1f;
        const float PedalLimitCapSize = 0.09f;
        // Etiquette limits (rival, chill-out, yield) are harmless, so they read cool; grip limits yellow; the
        // overspeed cut black; countersteer orange; instability violet; the recovery coast cyan.
        static readonly Color NonDangerousReasonColor = Color.FromArgb(255, 120, 200, 255);
        static readonly Color GripReasonColor = Color.Yellow;
        static readonly Color OverspeedReasonColor = Color.Black;
        static readonly Color CountersteerReasonColor = Color.Orange;
        static readonly Color InstabilityReasonColor = Color.FromArgb(255, 190, 80, 255);
        static readonly Color RecoveryReasonColor = Color.Cyan;

        struct PedalLimitSphere
        {
            public float Level;
            public Color Color;
            public float Size;
        }

        readonly PedalLimitSphere[] _pedalLimits = new PedalLimitSphere[12];

        void DrawPedalLimits(Vector3 basePt, Vector3 dir, float length)
        {
            int count = 0;
            AddPedalLimit(Control.MaxThrottleFromTCS, GripReasonColor, PedalLimitReasonSize, ref count);
            AddPedalLimit(Control.MaxThrottleFromInstability, InstabilityReasonColor, PedalLimitReasonSize, ref count);
            AddPedalLimit(Control.MaxThrottleFromOverspeed, OverspeedReasonColor, PedalLimitReasonSize, ref count);
            AddPedalLimit(Control.MaxThrottleFromRival, NonDangerousReasonColor, PedalLimitReasonSize, ref count);
            AddPedalLimit(Control.MaxThrottleFromChillOut, NonDangerousReasonColor, PedalLimitReasonSize, ref count);
            AddPedalLimit(Control.MaxThrottleFromYield, NonDangerousReasonColor, PedalLimitReasonSize, ref count);
            AddPedalLimit(Control.MaxThrottleFromOffTrack, Color.White, PedalLimitReasonSize, ref count);
            AddPedalLimit(Control.MaxThrottleFromRecovery, RecoveryReasonColor, PedalLimitReasonSize, ref count);
            AddPedalLimit(Control.MaxBrakeFromABS, GripReasonColor, PedalLimitReasonSize, ref count);
            AddPedalLimit(Control.MaxBrakeFromCountersteer, CountersteerReasonColor, PedalLimitReasonSize, ref count);
            if (IsFullCountersteer()) AddPedalLimit(CountersteerRollThrottle, Color.White, PedalLimitCapSize, ref count);
            if (_offtrackInputCap < 0f) AddPedalLimit(-_offtrackInputCap, Color.White, PedalLimitCapSize, ref count);
            else if (_offtrackInputCap < 1f) AddPedalLimit(_offtrackInputCap, Color.White, PedalLimitCapSize, ref count);

            SortPedalLimits(count);
            for (int i = 0; i < count; i++)
                DrawSteerLineSphere(basePt, dir, length, _pedalLimits[i].Level, _pedalLimits[i].Size, _pedalLimits[i].Color);
        }

        // A limit at full authority is not limiting anything, so it is not queued.
        void AddPedalLimit(float level, Color color, float size, ref int count)
        {
            if (level >= 1f || count >= _pedalLimits.Length) return;
            _pedalLimits[count].Level = level;
            _pedalLimits[count].Color = color;
            _pedalLimits[count].Size = size;
            count++;
        }

        void SortPedalLimits(int count)
        {
            for (int i = 1; i < count; i++)
            {
                PedalLimitSphere cap = _pedalLimits[i];
                int j = i - 1;
                while (j >= 0 && _pedalLimits[j].Level < cap.Level)
                {
                    _pedalLimits[j + 1] = _pedalLimits[j];
                    j--;
                }
                _pedalLimits[j + 1] = cap;
            }
        }

        // Full throttle green, neutral yellow, full brake red.
        static Color InputColour(float input)
        {
            float v = ARS.Clamp(input, -1f, 1f);
            return Color.FromArgb((int)(255f * (1f - Math.Max(v, 0f))), (int)(255f * (1f + Math.Min(v, 0f))), 0);
        }

        public void RunTimedCore()
        {
            UpdateTrackPosition();
            UpdateSlideAndBoundingBox();
            UpdatePerceivedGrip();
            UpdateTRLateralAtSpeed();

            ProcessAI();

            _lastCoreTick = Game.GameTime;
        }



        void UpdateSlideAndBoundingBox()
        {
            VehicleData.BoundingBox = ARS.SlidingBoundingBoxWidth(Car);
            VehicleData.SlideAngle = (float)Math.Round(Vector3.SignedAngle(Car.Velocity.Normalized, Car.ForwardVector, Car.UpVector), 3);
        }

        void UpdatePassengerSeat()
        {
            if (ControlledByPlayer) return;

            float myHalfLen = Math.Abs(VehicleData.ModelDimensions.Y) * 0.5f;

            bool shouldPassengerize = false;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null) continue;
                float rivalHalfLen = Math.Abs(r.RivalRacer.VehicleData.ModelDimensions.Y) * 0.5f;
                float threshold = myHalfLen + rivalHalfLen + 0.5f;
                if (r.Distance < threshold)
                {
                    shouldPassengerize = true;
                    break;
                }
            }

            if (shouldPassengerize && !_isPassengerized)
            {
                try { Driver.SetIntoVehicle(Car, VehicleSeat.Passenger); _isPassengerized = true; } catch (Exception) { }
            }
            else if (!shouldPassengerize && _isPassengerized)
            {
                try { Driver.SetIntoVehicle(Car, VehicleSeat.Driver); _isPassengerized = false; } catch (Exception) { }
            }
        }

        // A flooded engine is off, not dead: the game cuts it in water and never turns it back on, so without this an
        // AI car sits immovable on the throttle forever. Health gates it, so a wreck is not raised.
        void EnsureEngineRunning()
        {
            bool engineRunning = Function.Call<bool>((Hash)0xAE31E7DF9B5B132E, Car);   // GET_IS_VEHICLE_ENGINE_RUNNING
            if (Car.EngineHealth > 0f && !engineRunning) Function.Call(Hash.SET_VEHICLE_ENGINE_ON, Car, true, true, false);
        }

        public void ApplyInputs()
        {


            if (Driver.IsSittingInVehicle(Car))
            {
                UpdatePassengerSeat();
                EnsureEngineRunning();

                Car.HandbrakeOn = Control.HandBrakeTime > Game.GameTime;

                VehicleMemory.SetThrottle(Car, ShapeLaunchThrottle(ARS.Clamp(Control.Throttle, -1, 1)));
                VehicleMemory.SetBrakes(Car, ARS.Clamp(Control.Brake, 0f, 1f));
                VehicleMemory.SetSteerAngle(Car, Control.SteerInput);

            }
            else
            {
                VehicleMemory.SetThrottle(Car, 0f);
                VehicleMemory.SetBrakes(Car, 0f);
                VehicleMemory.SetSteerInput(Car, 0f);
            }
        }

        // The engine resets the cheat multiplier every physics step, so this re-writes it each frame — the single
        // per-frame writer, with nitro riding on the same write. Force side only: a fade is a multiplication so its
        // inverse restores it, whereas a cut to zero cannot be, and grip-side losses only add wheelspin.
        void UpdatePowerCompensation()
        {
            float counter = 1f;

            float commanded = Math.Abs(Control.Throttle);
            if (!ControlledByPlayer && commanded > PowerCounterMinCommanded && _appliedThrottleLastFrame < commanded)
                counter *= ARS.Clamp(commanded / Math.Max(_appliedThrottleLastFrame, PowerCounterMinApplied), 1f, PowerCounterMax);

            // Past the car's own top speed the engine fades drive force and adds an over-speed brake. No
            // throttle gate is needed: this scales fDriveForce, which is already zero off-throttle.
            float overTopSpeed = ARS.GetForwardSpeed(Car) - Handling.EstimatedTopSpeed * TopSpeedCounterOnsetScale;
            if (overTopSpeed > 0f) counter *= ARS.Remap(overTopSpeed, 0f, TopSpeedCounterBand, 1f, TopSpeedCounterMax, true);

            _powerCounter = counter;

            if (_powerCounter > 1.001f && Game.GameTime > _powerCounterLoggedAt)
            {
                _powerCounterLoggedAt = Game.GameTime + 2000;
                string throttlePart = ControlledByPlayer ? "" : ", throttle " + _appliedThrottleLastFrame.ToString("0.00") + " of " + commanded.ToString("0.00");
                ARS.Log(ARS.LogImportance.Info, "Power counter " + Car.DisplayName + " x" + _powerCounter.ToString("0.00") + " (over top speed " + overTopSpeed.ToString("0.0") + " m/s" + throttlePart + ")");
            }
        }

        void ApplyPowerMultiplier()
        {
            float multiplier = _powerCounter;
            if (Game.GameTime < _nitrousActiveUntil) multiplier *= NitrousPowerMultiplier;

            if (float.IsNaN(multiplier) || float.IsInfinity(multiplier) || multiplier < 0f || multiplier > 10f) multiplier = 1f;

            Function.Call((Hash)CheatPowerIncreaseHash, Car, multiplier);
        }

        // Match the player's launch capability: a keyboard pedal is a hard 0/1 and 1.0 is exactly what sets the
        // engine's full-throttle grip loss, so a binary player's cars are quantised the same way, while a player who
        // can modulate gets a cap that stays under it. Reverse, braking and the post-fade speed stay unshaped.
        float ShapeLaunchThrottle(float throttle)
        {
            if (throttle <= 0f || ARS.GetForwardSpeed(Car) > LaunchShapeMaxSpeed) return throttle;

            bool modulates = ARS.PlayerModulatesThrottle || !ARS.PlayerParticipating;
            if (modulates) return Math.Min(throttle, ModulatedThrottleCap);
            return throttle > LaunchThrottleBinarySplit ? 1f : 0f;
        }

        bool IsAwd()
        {
            float bias = VehicleMemory.GetDriveBiasFront(Car);
            return bias > 0.01f && bias < 0.99f;
        }

        int OffsetCornerNode(int node, int offset)
        {
            int count = ARS.TrackPoints.Count;
            if (count == 0) return -1;

            if (ARS.IsPointToPoint)
                return (int)ARS.Clamp(node + offset, 0, count - 1);

            int wrapped = (node + offset) % count;
            return wrapped < 0 ? wrapped + count : wrapped;
        }


        float OutOfTrackDistance()
        {
            return Math.Abs(Brain.CurrentPerception.DeviationFromCenter) - CurrentTrackPoint.TrackHalfWidth;
        }

        void UpdateRivalInfo()
        {
            Brain.AvoidanceTarget = null;
            foreach (Rival r in Brain.Rivals)
            {
                r.Update(this);
                bool isAvoidanceCandidate = r.RelativePosition == RelativePos.Ahead && ARS.IsBetween(r.RouteGapAhead, 0f, SameSectionMaxGapMeters) && (ARS.IsBetween(r.FrontGap, 0f, 3f) || ARS.IsBetween(r.TimeToReach, 0f, 5f)) && ARS.IsBetween(Math.Abs(r.DirectionDiff), 0f, AvoidAngleGateDegrees);
                if (Brain.AvoidanceTarget == null && isAvoidanceCandidate)
                {
                    Brain.AvoidanceTarget = r;
                }
            }
        }

        public void InitializeTrackPosition()
        {
            if (ARS.TrackPoints.Count == 0) return;

            TrackPoint closestPoint = ARS.TrackPoints[0];
            Vector3 carPosition = Car.Position;
            float closestDistance = closestPoint.Position.DistanceTo(carPosition);
            foreach (TrackPoint point in ARS.TrackPoints)
            {
                float distance = point.Position.DistanceTo(carPosition);
                if (distance < closestDistance)
                {
                    closestPoint = point;
                    closestDistance = distance;
                }
            }

            CurrentTrackPoint = closestPoint;
            UpdateRaceProgress();
        }

        void UpdateRaceProgress()
        {
            TrackProgress = Lap * ARS.TrackPoints.Count + CurrentTrackPoint.Node;
            RaceProgress = (int)TrackProgress;
        }

        public void UpdateTrackPosition()
        {


            int refTrackpoint = (int)ARS.Clamp(CurrentTrackPoint.Node, 0, ARS.TrackPoints.Count - 1);

            _trackPositionScratch.Clear();
            int lastNode = ARS.TrackPoints.Count - 1;
            int firstCandidate = Math.Max(refTrackpoint - 6, 0);
            int lastCandidate = Math.Min(refTrackpoint + 6, lastNode);
            for (int i = firstCandidate; i <= lastCandidate; i++)
            {
                _trackPositionScratch.Add(ARS.TrackPoints[i]);
            }

            if (!ARS.IsPointToPoint)
            {
                if (refTrackpoint <= 6)
                {
                    for (int i = Math.Max(lastNode - 6, 0); i <= lastNode; i++)
                        _trackPositionScratch.Add(ARS.TrackPoints[i]);
                }
                else if (refTrackpoint >= lastNode - 6)
                {
                    for (int i = 0; i <= Math.Min(6, lastNode); i++)
                        _trackPositionScratch.Add(ARS.TrackPoints[i]);
                }
            }

            Vector3 carPosition = Car.Position;
            TrackPoint closestPoint = _trackPositionScratch[0];
            float closestDistance = closestPoint.Position.DistanceTo(carPosition);
            foreach (TrackPoint point in _trackPositionScratch)
            {
                float distance = point.Position.DistanceTo(carPosition);
                if (distance < closestDistance)
                {
                    closestPoint = point;
                    closestDistance = distance;
                }
            }

            float reacquireDistance = Math.Max(CurrentTrackPoint.TrackHalfWidth + 10f, 25f);
            if (closestDistance > reacquireDistance)
            {
                foreach (TrackPoint point in ARS.TrackPoints)
                {
                    float distance = point.Position.DistanceTo(carPosition);
                    if (distance < closestDistance)
                    {
                        closestPoint = point;
                        closestDistance = distance;
                    }
                }
            }

            CurrentTrackPoint = closestPoint;
            Brain.CurrentPerception.DeviationFromCenter = ARS.SignedLaneOffset(carPosition, CurrentTrackPoint.Position, CurrentTrackPoint.Direction);

            LookAheads.Clear();
            Vector3 velocity = Car.Velocity;
            float speed = velocity.Length();
            AlongTrackSpeed = Vector3.Dot(velocity, CurrentTrackPoint.Direction);

            int steerRef = (int)ARS.Clamp(speed * SteerPreviewSeconds, SteerLookaheadMinMeters, SteerLookaheadMaxMeters);
            int quarterSec = (int)(speed * 0.25f);
            int halfSec = (int)(speed * 0.5f);
            int threeQuarterSec = (int)(speed * 0.75f);
            int oneSec = (int)(speed);
            int oneHalfSec = (int)(speed * 1.5f);
            int twoSec = (int)(speed * 2f);

            TrackPoint ResolveLookAhead(int offset)
            {
                int node = CurrentTrackPoint.Node + offset;
                if (ARS.IsPointToPoint) return ARS.TrackPoints[Math.Min(node, lastNode)];
                return ARS.TrackPoints[node % ARS.TrackPoints.Count];
            }

            LookAheads[LookAhead.SteerRef] = ResolveLookAhead(steerRef);
            LookAheads[LookAhead.QuarterSec] = ResolveLookAhead(quarterSec);
            LookAheads[LookAhead.HalfSec] = ResolveLookAhead(halfSec);
            LookAheads[LookAhead.ThreeQuarterSec] = ResolveLookAhead(threeQuarterSec);
            LookAheads[LookAhead.OneSec] = ResolveLookAhead(oneSec);
            LookAheads[LookAhead.OneHalfSec] = ResolveLookAhead(oneHalfSec);
            LookAheads[LookAhead.TwoSec] = ResolveLookAhead(twoSec);




            int nodeCount = ARS.TrackPoints.Count;
            int currentNode = CurrentTrackPoint.Node;
            float currentPct = ARS.GetPercent(currentNode, nodeCount);
            float previousPct = _previousNode >= 0 ? ARS.GetPercent(_previousNode, nodeCount) : 0f;
            bool wrappedStartLine = !ARS.IsPointToPoint && _previousNode >= 0 && previousPct > 90f && currentPct < 10f;
            UpdateLapRegistration(currentNode, nodeCount, currentPct, wrappedStartLine);

            UpdateRaceProgress();

            _previousNode = currentNode;

            // Route radius from three sample points.
            Brain.CurrentPerception.CurveRadiusToFollowPoint = RouteRadiusSampled();
            UpdateApexLeapfrog();
            // Requirements run ahead of the refill and force one on a flip, so a corner that has just asked for a
            // plan gets it this tick rather than up to 500 ms later.
            if (UpdateCornerRequirements() || _apexUpdateTick + _phaseOffsetMs < Game.GameTime)
            {
                _apexUpdateTick = Game.GameTime + 500;
                RefillApexQueue();
            }
            // High-speed lane radius: short 0.5s to 1.0s window.
            Brain.CurrentPerception.HighSpeedCurveRadius = ComputeRouteRadius((int)(speed * 0.5f), (int)(speed * 1.0f));
        }

        // The lap counter: closed by crossing the line, and reopened mid-track so the next crossing can register.
        void UpdateLapRegistration(int currentNode, int nodeCount, float currentPct, bool wrappedStartLine)
        {
            if (CanRegisterNewLap)
            {
                if (wrappedStartLine || (ARS.IsPointToPoint && currentPct > 99f && ARS.EntityRelativeOffset(Car, ARS.TrackPoints.Last().Position).Y < 0f))
                {
                    CanRegisterNewLap = false;
                    Lap++;
                    ARS.Log(ARS.LogImportance.Info, "Lap++ " + Name + " -> lap " + Lap + " (node " + currentNode + ")");
                    if (Lap > ARS.RaceMenuStore.GetInt("Laps", 6))
                    {
                        if (Car.CurrentBlip != null) Car.CurrentBlip.Color = BlipColor.Green;
                    }

                    if (Lap == 2 && !ARS.IsPointToPoint)
                    {
                        LapStartTime = Game.GameTime;
                        VehicleData.ResetLapPeaks();
                    }
                    else if (Lap > 1)
                    {
                        TimeSpan lapTime = ARS.ParseToTimeSpan(Game.GameTime - LapStartTime);
                        string peaks = "accel " + VehicleData.PeakAccelG.ToString("0.00") + "G decel " + Math.Abs(VehicleData.PeakDecelG).ToString("0.00") + "G lat " + VehicleData.PeakLateralG.ToString("0.00") + "G top " + ARS.MpsToMph(VehicleData.PeakTopSpeedMps).ToString("0") + "mph";
                        ARS.Log(ARS.LogImportance.Info, "Laptime " + CarModelName + ": " + lapTime.ToString("m':'ss'.'f") + " | " + peaks);
                        LapTimes.Add(lapTime);
                        if (ARS.DebugToggles[Options.ShowAiLapTimes]) UI.Notify("~b~[ARS]~w~ " + Name + " lap " + (Lap - 1) + ": " + lapTime.ToString("m':'ss'.'f"));
                        LapStartTime = Game.GameTime;
                        VehicleData.ResetLapPeaks();
                    }
                }
            }
            else if (BaseBehavior == RacerBaseBehavior.Race && ARS.IsBetween(currentPct, 40f, 60f))
            {
                CanRegisterNewLap = true;
            }
        }

        // Circumradius through three route-window sample points.
        float RouteRadiusSampled()
        {
            int count = ARS.TrackPoints.Count;
            if (count < 10) return 999f;

            float traction = Math.Max(VehicleData.CurrentMechanicalGrip, 0.1f);
            int o1 = 0;
            int o2 = (int)(Car.Velocity.Length() / traction);
            int o3 = (int)(Car.Velocity.Length() * 2f / traction);

            int n1, n2, n3;
            if (ARS.IsPointToPoint)
            {
                n1 = (int)ARS.Clamp(CurrentTrackPoint.Node + o1, 0, count - 1);
                n2 = (int)ARS.Clamp(CurrentTrackPoint.Node + o2, 0, count - 1);
                n3 = (int)ARS.Clamp(CurrentTrackPoint.Node + o3, 0, count - 1);
            }
            else
            {
                n1 = ((CurrentTrackPoint.Node + o1) % count + count) % count;
                n2 = ((CurrentTrackPoint.Node + o2) % count + count) % count;
                n3 = ((CurrentTrackPoint.Node + o3) % count + count) % count;
            }

            float r = ARS.Circumradius3D(ARS.TrackPoints[n1].Position, ARS.TrackPoints[n3].Position, ARS.TrackPoints[n2].Position);
            if (float.IsNaN(r) || float.IsInfinity(r)) r = 999f;
            return ARS.Clamp(r, 5f, 999f);
        }

        // Braking map close-corner filtering and entrance timing.
        const float ApexBufferSeconds = 5f;
        const float EntranceBrakeBufferSeconds = 0.25f;
        const float EntranceBrakeExtraDistance = 0f;
        const float SecondaryApexSpeedDifference = 5f;
        const float BrakingTargetFactor = 0.5f;

        // Cheap: drop passed apexes and invalidate stale entries every tick.
        void UpdateApexLeapfrog()
        {
            int[] heldNodes = { NextApexNode, NextApexNode2, NextApexNode3 };
            float[] heldRadii = { NextApexRadius, NextApexRadius2, NextApexRadius3 };

            bool heldTableChanged = heldNodes.Any(node => node >= 0 && !ARS.Corners.Any(corner => corner.Node == node));
            bool lowSpeedInvalidation = heldNodes[0] >= 0 && Car.Velocity.Length() < NextApexSpeed * 0.5f;
            if (heldTableChanged || lowSpeedInvalidation)
            {
                for (int i = 0; i < heldNodes.Length; i++) heldNodes[i] = -1;
            }
            else
            {
                // Leapfrog passed apexes forward through the held queue.
                int shift = 0;
                while (shift < heldNodes.Length && heldNodes[shift] >= 0 && HasPassedApex(heldNodes[shift])) shift++;
                if (shift > 0) ScheduleBrakeCommit();
                if (shift > 0 && ControlledByPlayer) Tips.ApexPassed(shift);
                if (shift > 0)
                {
                    for (int i = 0; i < heldNodes.Length - shift; i++)
                    {
                        heldNodes[i] = heldNodes[i + shift];
                        heldRadii[i] = heldRadii[i + shift];
                    }
                    for (int i = heldNodes.Length - shift; i < heldNodes.Length; i++)
                    {
                        heldNodes[i] = -1;
                        heldRadii[i] = 999f;
                    }
                }
            }

            CommitApexQueue(heldNodes, heldRadii);
        }

        // Expensive: scan all corners and refill empty queue slots. Gated to 0.5s.
        // Decide, per car and per corner, whether the corner wants a braking plan and an outside line. It reads the
        // car's own predicted arrival against its own apex speed, so it never depends on the held queue — a corner
        // the queue has not selected still gets evaluated, which is what lets a flag turn on in the first place.
        // The two flags answer different questions and so are judged on different axes: braking is a v²/2a distance
        // problem and is tested from the road it actually needs, while positioning is a lateral move and is tested
        // inside the time window the move belongs in.
        bool UpdateCornerRequirements()
        {
            float speed = Math.Max(Car.Velocity.Length(), 1f);
            float forwardGs = VehicleData.GetLongitudinalGs(Car.ForwardVector);
            bool flipped = false;

            foreach (CornerPoint corner in ARS.Corners)
            {
                int entranceNode = corner.StartNode >= 0 ? corner.StartNode : OffsetCornerNode(corner.Node, -corner.LengthStart);
                int distance = ForwardNodeDistance(entranceNode);
                if (distance <= 0) continue;
                if (!TryGetCornerContext(corner.Node, out CornerContext context)) continue;

                bool tight = corner.DetectedRadius < CornerTightRadius;
                // The intended speed must be the one the plan itself will target, or the flag reads a wider corner
                // than the car is actually asked to take and never fires. That is SupposedRadius, not the smoothed one.
                float intended = ApexSpeedWithDownforce(corner.SupposedRadius);
                float timeToEntrance = distance / speed;
                float decel = Math.Max(BrakingDecelBase(corner.Node), 0.1f);
                float brakingDistance = (speed * speed - intended * intended) / (2f * decel);
                bool inBrakingRange = distance <= brakingDistance + BrakeHorizonMargin;
                bool inPositionRange = tight && timeToEntrance <= RequirementLookaheadSeconds;
                if (!inBrakingRange && !inPositionRange) continue;

                float arrival = speed + forwardGs * Handling.Gravity * timeToEntrance * RequirementExtrapolationScale;
                float excess = arrival - intended;

                // A tight corner that wants positioning is braking regardless, because the held queue is the only
                // route to an outside line — so its braking threshold is the lower one.
                if (!context.RequiresBraking && inBrakingRange && excess > ARS.MphToMps(tight ? PositionExcessMph : BrakeExcessMph))
                {
                    context.RequiresBraking = true;
                    flipped = true;
                }
                if (!context.RequiresPositioning && inPositionRange && excess > ARS.MphToMps(PositionExcessMph))
                {
                    context.RequiresPositioning = true;
                    flipped = true;
                }
            }
            return flipped;
        }

        void RefillApexQueue()
        {
            int count = ARS.TrackPoints.Count;
            if (count < 10 || ARS.Corners.Count == 0)
            {
                CommitApexQueue(new[] { -1, -1, -1 }, new[] { 999f, 999f, 999f });
                return;
            }

            List<int> upcoming = new List<int>();
            for (int i = 0; i < ARS.Corners.Count; i++)
            {
                if (ForwardNodeDistance(ARS.Corners[i].Node) > 0) upcoming.Add(i);
            }
            upcoming.Sort((left, right) => ForwardNodeDistance(ARS.Corners[left].Node).CompareTo(ForwardNodeDistance(ARS.Corners[right].Node)));

            // Derived fresh every refill and never carried over: slot one must be the nearest corner each time, or
            // the line, the commit lane and the chevron aim past it the moment the other slots are occupied.
            List<int> selectedNodes = new List<int>();
            List<float> selectedRadii = new List<float>();
            if (upcoming.Count > 0)
            {
                selectedNodes.Add(ARS.Corners[upcoming[0]].Node);
                selectedRadii.Add(ARS.Corners[upcoming[0]].SupposedRadius);
            }

            // The remaining slots only feed the braking plan, so they go to the most restrictive corners this car
            // has decided it needs one for — the ones that make it brake earliest.
            float speed = Car.Velocity.Length();
            while (selectedNodes.Count < HeldApexCount)
            {
                int best = -1;
                float bestBrakingSpeed = float.MaxValue;
                for (int i = 0; i < upcoming.Count; i++)
                {
                    int node = ARS.Corners[upcoming[i]].Node;
                    if (selectedNodes.Contains(node)) continue;
                    if (!CornerRequiresBraking(node)) continue;

                    float radius = ARS.Corners[upcoming[i]].SupposedRadius;
                    float apexSpeed = ApexSpeedWithDownforce(radius);
                    if (apexSpeed >= speed) continue;
                    if (!CornerWorthHolding(node, radius, selectedNodes, selectedRadii)) continue;

                    float brakingSpeed = ApexBrakingSpeed(node, apexSpeed);
                    if (brakingSpeed < bestBrakingSpeed)
                    {
                        bestBrakingSpeed = brakingSpeed;
                        best = i;
                    }
                }
                if (best < 0) break;
                selectedNodes.Add(ARS.Corners[upcoming[best]].Node);
                selectedRadii.Add(ARS.Corners[upcoming[best]].SupposedRadius);
            }

            CommitApexQueue(
                new[]
                {
                    selectedNodes.Count > 0 ? selectedNodes[0] : -1,
                    selectedNodes.Count > 1 ? selectedNodes[1] : -1,
                    selectedNodes.Count > 2 ? selectedNodes[2] : -1
                },
                new[]
                {
                    selectedRadii.Count > 0 ? selectedRadii[0] : 999f,
                    selectedRadii.Count > 1 ? selectedRadii[1] : 999f,
                    selectedRadii.Count > 2 ? selectedRadii[2] : 999f
                });
        }

        // A candidate that belongs to a corner already held adds nothing unless it is materially slower: the pair is
        // one complex, and the tighter of the two is the one that has to be planned for.
        bool CornerWorthHolding(int node, float radius, List<int> selectedNodes, List<float> selectedRadii)
        {
            int distance = ForwardNodeDistance(node);
            float candidateSpeed = RouteIdealSpeedForRadius(radius);
            float nearestGap = float.MaxValue;
            float nearestSpeed = 999f;
            for (int i = 0; i < selectedNodes.Count; i++)
            {
                float gap = Math.Abs(distance - ForwardNodeDistance(selectedNodes[i]));
                if (gap < nearestGap)
                {
                    nearestGap = gap;
                    nearestSpeed = RouteIdealSpeedForRadius(selectedRadii[i]);
                }
            }
            if (nearestGap > Math.Max(5f, nearestSpeed * ApexBufferSeconds)) return true;
            return nearestSpeed - candidateSpeed >= SecondaryApexSpeedDifference;
        }

        void CommitApexQueue(int[] nodes, float[] radii)
        {
            NextApexNode = nodes[0];
            NextApexRadius = radii[0];
            NextApexSpeed = NextApexNode >= 0 ? ApexSpeedWithDownforce(NextApexRadius) : 999f;
            NextApexNode2 = nodes[1];
            NextApexRadius2 = radii[1];
            NextApexSpeed2 = NextApexNode2 >= 0 ? ApexSpeedWithDownforce(NextApexRadius2) : 999f;
            NextApexNode3 = nodes[2];
            NextApexRadius3 = radii[2];
            NextApexSpeed3 = NextApexNode3 >= 0 ? ApexSpeedWithDownforce(NextApexRadius3) : 999f;

            if (NextApexNode >= 0)
            {
                // Instance Brain.Corner from the nearest apex.
                CornerPoint original = ARS.Corners.FirstOrDefault(c => c.Node == NextApexNode);
                _nextApexCorner = original;
                CornerPoint cp = original ?? new CornerPoint();
                if (original == null)
                {
                    cp.Node = NextApexNode;
                    cp.Angle = ARS.TrackPoints[NextApexNode].Angle;
                    cp.SupposedRadius = NextApexRadius;
                }
                Brain.Corner = new Corner(NextApexSpeed, cp);
            }
            else
            {
                _nextApexCorner = null;
                Brain.Corner = null;
            }
        }

        bool HasPassedApex(int apexNode)
        {
            if (apexNode < 0) return false;
            if (ARS.IsPointToPoint) return CurrentTrackPoint.Node >= apexNode;
            // On circuits, the apex is behind us when the forward distance exceeds half the track.
            return ForwardNodeDistance(apexNode) > ARS.TrackPoints.Count / 2;
        }

        // Kinematic braking map reaches apex speed at the braking target: a fixed lead before the corner's turn-in.
        int BrakingTargetNode(CornerPoint corner, float apexSpeed, float factor)
        {
            if (corner == null || corner.Node < 0) return -1;

            int targetNode = BrakeTargetBaseNode(corner, -1);
            if (targetNode < 0) return -1;

            factor = ARS.Clamp(factor, 0f, 1f);
            int distance = corner.Node - targetNode;
            if (!ARS.IsPointToPoint && distance < 0) distance += ARS.TrackPoints.Count;
            if (ARS.IsPointToPoint && distance < 0) distance = 0;

            return OffsetCornerNode(targetNode, (int)Math.Round(distance * factor));
        }

        float ApexBrakingSpeed(int apexNode, float apexSpeed)
        {
            if (apexNode < 0) return 999f;
            float velTarget = apexSpeed;

            CornerPoint corner = ARS.Corners.FirstOrDefault(c => c.Node == apexNode);
            int fallbackNode = CornerEntranceNode(corner, apexNode);
            int targetNode = ActiveManeuver.Type == ManeuverType.DiveBomb
                ? BrakingTargetNode(corner, apexSpeed, BrakingTargetFactor)
                : BrakeTargetBaseNode(corner, fallbackNode);
            if (targetNode < 0) targetNode = fallbackNode;

            int targetDistance = ForwardNodeDistance(targetNode);
            int apexDistance = ForwardNodeDistance(apexNode);
            // On circuits, a passed braking target wraps to the next lap. Once the apex is
            // still ahead but the target is farther away, the target has been passed.
            float rawDistance = !ARS.IsPointToPoint && targetDistance > apexDistance
                ? 0f
                : targetDistance;
            if (rawDistance < 0f) rawDistance = 0f;
            float distance = rawDistance > 0f
                ? Math.Max(0f, rawDistance
                    - Car.Velocity.Length() * EntranceBrakeBufferSeconds
                    - EntranceBrakeExtraDistance)
                : 0f;

            // Grade over the same span the solve integrates, or the mean and the distance measure different zones.
            float decel = BrakingDecel(apexNode, distance);

            float spd = (float)Math.Sqrt(velTarget * velTarget + 2f * decel * distance);
            if (float.IsNaN(spd) || float.IsInfinity(spd)) spd = 999f;
            return spd;
        }

        // A crest unloads the tyres before a corner and the span's mean grade cannot see it: an up-then-down bump
        // averages to no grade at all. Geometry is measured once at generation at this probe, so the live speed
        // rescales it, and the unload is floored the way the hill grip model floors its own.
        const float CrestProbeSpeed = ARS.CrestProbeSpeed;
        const float CrestDecelFloor = 0.5f;

        // Braking decel over a span: grip-limited base plus the gravity component of the span's mean grade.
        // The solve is v² = vApex² + 2∫a·ds, so the mean decel over the span is the exact quantity.
        public float BrakingDecel(int apexNode, float spanMeters)
        {
            float decel = BrakingDecelBase(apexNode) * CrestDecelFactor(apexNode, spanMeters) + Handling.Gravity * BrakingGradeSine(spanMeters);
            return Math.Max(decel, 0.1f);
        }

        // Vertical curvature changes normal load, so it scales the grip-limited base only -- the grade term is the
        // along-slope gravity component, which a crest does not change. The crest covers its extent, so the unload is
        // the area of that ramp clipped to the solve span rather than a point test: a binary in/out steps the planned
        // decel mid-braking, and those samples are far too sparse to integrate it.
        float CrestDecelFactor(int apexNode, float spanMeters)
        {
            CornerPoint corner = ARS.Corners.FirstOrDefault(c => c.Node == apexNode);
            if (corner == null || corner.CrestNode < 0 || corner.CrestSpanNodes <= 0 || spanMeters < 1f) return 1f;
            if (corner.CrestGs >= 0f) return 1f;
            int count = ARS.TrackPoints.Count;
            int toCrest = corner.CrestNode - CurrentTrackPoint.Node;
            if (!ARS.IsPointToPoint) toCrest = ((toCrest % count) + count) % count;
            float halfExtent = Math.Max(1f, corner.CrestSpanNodes * 0.5f);
            float start = toCrest - halfExtent;
            float end = toCrest + halfExtent;
            float lo = Math.Max(start, 0f);
            float hi = Math.Min(end, spanMeters);
            if (end <= 0f || start >= spanMeters) return 1f;
            float peak = ARS.Clamp(toCrest, lo, hi);
            float risingUnscaled = RampIntegralUnscaled(start, lo, peak);
            float fallingUnscaled = RampIntegralUnscaled(end, hi, peak);
            float speedRatio = Car.Velocity.Length() / CrestProbeSpeed;
            float unload = 1f + corner.CrestGs * speedRatio * speedRatio * (risingUnscaled + fallingUnscaled) / (2f * halfExtent * spanMeters);
            if (float.IsNaN(unload) || float.IsInfinity(unload)) return 1f;
            unload = Math.Max(unload, CrestDecelFloor);
            return 1f - Math.Min((1f - unload) * ARS.CrestEffect, 0.9f);
        }

        // The raw expression before the 1/(2h) normalisation: 2h times the area under one half of the ramp. Its endpoints
        // go in distance order so the result stays positive; swapping them negates the area and the unload with it.
        static float RampIntegralUnscaled(float zeroAt, float from, float to)
        {
            return (to - zeroAt) * (to - zeroAt) - (from - zeroAt) * (from - zeroAt);
        }

        // The grade-free half, so a horizon test can ask the same question without walking the span.
        float BrakingDecelBase(int apexNode)
        {
            float brakingAbility = Math.Min(Handling.BrakingAbility * 4, VehicleData.CurrentMechanicalGrip);
            float decel = brakingAbility * Handling.Gravity * EffectiveBrakeFactor(apexNode);
            if (ActiveManeuver.Type == ManeuverType.Yield) decel *= 0.5f;
            return decel;
        }

        // Mean grade over the braking span as sin(pitch): negative downhill loses decel, positive uphill gains it.
        float BrakingGradeSine(float spanMeters)
        {
            int spanNodes = (int)spanMeters;
            if (spanNodes < 1) return 0f;
            int count = ARS.TrackPoints.Count;
            int samples = Math.Min(spanNodes, 8);
            float sum = 0f;
            for (int i = 1; i <= samples; i++)
            {
                int node = CurrentTrackPoint.Node + (int)(spanNodes * (float)i / samples);
                int sample = ARS.IsPointToPoint ? (int)ARS.Clamp(node, 0, count - 1) : ((node % count) + count) % count;
                sum += ARS.TrackPoints[sample].Elevation;
            }
            return sum / samples * ElevationToSlopeSine;
        }

        // Centripetal speed limit for a radius.
        float RouteIdealSpeedForRadius(float r)
        {
            if (float.IsNaN(r) || float.IsInfinity(r)) r = 999f;
            r = ARS.Clamp(r, 5f, 999f);
            return (float)Math.Sqrt((VehicleData.CurrentMechanicalGrip * Handling.Gravity) * r);
        }

        float ApexSpeedWithDownforce(float r)
        {
            // CurrentMechanicalGrip already includes the per-tick downforce bonus (see UpdatePerceivedGrip),
            // so this is just the cornering-speed-from-grip formula with a sanity clamp on the radius.
            if (float.IsNaN(r) || float.IsInfinity(r)) r = 999f;
            r = ARS.Clamp(r, 5f, 999f);
            return (float)Math.Sqrt((VehicleData.CurrentMechanicalGrip * Handling.Gravity) * r);
        }

        float ComputeRouteRadius(int startOffset, int endOffset)
        {
            int count = ARS.TrackPoints.Count;
            int routeStartNode, routeEndNode, routeMidNode;
            if (ARS.IsPointToPoint)
            {
                routeStartNode = (int)ARS.Clamp(CurrentTrackPoint.Node + startOffset, 0, count - 1);
                routeEndNode = (int)ARS.Clamp(CurrentTrackPoint.Node + endOffset, 0, count - 1);
                routeMidNode = (int)((routeStartNode + routeEndNode) * 0.5f);
            }
            else
            {
                routeStartNode = ((CurrentTrackPoint.Node + startOffset) % count + count) % count;
                routeEndNode = ((CurrentTrackPoint.Node + endOffset) % count + count) % count;
                int span = ((endOffset - startOffset) % count + count) % count;
                if (span == 0) return 999f;
                routeMidNode = (((CurrentTrackPoint.Node + startOffset + span / 2) % count) + count) % count;
            }

            if (routeEndNode == routeStartNode) return 999f;
            // Circumradius through the start, midpoint, and end of the route window.
            return ARS.Circumradius3D(
                ARS.TrackPoints[routeStartNode].Position,
                ARS.TrackPoints[routeEndNode].Position,
                ARS.TrackPoints[routeMidNode].Position);
        }




        void ProcessTimedAI()
        {
            int now = Game.GameTime;

            if (_halfSecondTick + _phaseOffsetMs < now)
            {
                _halfSecondTick = now + 500 + (int)ARS.Remap(Car.Velocity.Length(), 0, 100, -250, 250, true);
            }

            if (_oneSecondTick + _phaseOffsetMs < now)
            {
                _oneSecondTick = now + 1000;

                if (!ControlledByPlayer)
                {
                    if (BaseBehavior == RacerBaseBehavior.Race && ARS.Racers.Count >= 1)
                    {
                        UpdateRivals();
                        ConsiderCrowdCard();
                    }

                    if (!Driver.IsSittingInVehicle(Car) && Car.IsStopped && Driver.IsStopped)
                    {
                        if (Driver.TaskSequenceProgress == -1)
                        {
                            TaskSequence enter = new TaskSequence();
                            Function.Call(Hash.TASK_ENTER_VEHICLE, 0, Car, 6000, -1, 2f, 0, 0);
                            enter.Close();
                            Driver.Task.PerformSequence(enter);
                            return;
                        }
                    }
                }

                // Independent rocket/boost control: enable boost on a stable, full-throttle run.
                if (BaseBehavior == RacerBaseBehavior.Race && Brain.CurrentPerception.HighSpeedCurveRadius > RocketBoostMinimumCurveRadius && Math.Abs(VehicleData.SlideAngle) < 0.5f && Math.Abs(Control.Throttle) > 0.9f) Function.Call((Hash)0x81E1552E35DC3839, Car, true);

                // Independent rocket/boost control: disable boost when braking.
                if (Function.Call<bool>((Hash)0x3D34E80EED4AE3BE, Car) && Control.Brake > 0.1f) Function.Call((Hash)0x81E1552E35DC3839, Car, false);


                if (ARS.SettingsMenuStore.GetInt("AIRacerAutofix", 2) == 2 && Function.Call<bool>(Hash._IS_VEHICLE_DAMAGED, Car))
                {
                    Car.Repair();
                }
            }
        }




        public void ProcessAI()
        {
            int now = Game.GameTime;

            ProcessTimedAI();
            if (_pressureTick + _phaseOffsetMs < now)
            {
                _pressureTick = now + 500;
                UpdatePressure();
            }

            if (BaseBehavior == RacerBaseBehavior.GridWait && Control.HandBrakeTime < now) Control.HandBrakeTime = now + (100 * ARS.GetRandomInt(2, 6));

            if (!ControlledByPlayer)
            {
                UpdateDNFCheck();

                if (_rivalInfoTick + _phaseOffsetMs < now)
                {
                    _rivalInfoTick = now + 500;
                    UpdateRivalInfo();
                }
                ConsiderManeuvers();
                ComputeTargetSpeed();
                ComputeSteering();

                UpdateThrottleReasonCaps();
                UpdateBrakeReasonCaps();
                ConvertSpeedToPedals();

                UpdateStuckCheck();
                UpdateStuckRecovery();

                ApplyStuckRecoveryOverride();

                // The limiter closes the steering last, after every writer above, so nothing escapes it.
                ApplySteerLimits();
                TranslateSteerToInput();

                UpdateNitrous();
                UpdateYield();

            }
            else
            {
                UpdateNitrous();
                _stuckPhase = StuckPhase.None;
                _stuckStationarySince = 0;
                _stuckOffTrackSince = 0;
                _stuckRecoveryCooldownEndTime = 0;
            }
        }

        void UpdatePressure()
        {
            float closestDistance = float.MaxValue;
            if (BaseBehavior == RacerBaseBehavior.Race)
            {
                foreach (Racer racer in ARS.Racers)
                {
                    if (racer == this || racer.Car == null || !racer.Car.Exists()) continue;
                    float dist = racer.Car.Position.DistanceTo(Car.Position);
                    if (dist < closestDistance)
                        closestDistance = dist;
                }
            }

            float targetPressure = 0f;
            if (closestDistance <= PressureProximityRange)
            {
                // Map nearby-rival proximity to an aggression-scaled pressure target.
                float t = ARS.Clamp((PressureProximityRange - closestDistance) / (PressureProximityRange - 20f), 0f, 1f);
                targetPressure = Aggression * t;
            }

            if (targetPressure > Pressure)
                Pressure = Math.Min(Pressure + PressureRisePerSecond * TickScale, targetPressure);
            else
                Pressure = Math.Max(Pressure - PressureFallPerSecond * TickScale, targetPressure);

            Pressure = ARS.Clamp(Pressure, 0f, PressureRange);

            // Pressure-driven lookahead is intentionally disabled.
            RouteLookAheadSeconds = 0.5f;
        }
 
 
        // A car whose engine is gone cannot rejoin whatever the recovery does, so it is taken out of the race and
        // parked instead of being teleported at forever.
        void UpdateDNFCheck()
        {
            if (IsDNF)
            {
                Control.HandBrakeTime = Game.GameTime + 1000;
                return;
            }
            if (BaseBehavior != RacerBaseBehavior.Race || Car.EngineHealth > 0f) return;
            ParkAtDNFSlot();
        }

        void ParkAtDNFSlot()
        {
            int slot = ARS.ParkedDNFs++;
            int nodeCount = ARS.TrackPoints.Count;
            int node = ARS.IsPointToPoint ? Math.Min(slot * DNFSlotSpacingMeters, nodeCount - 1) : (slot * DNFSlotSpacingMeters) % nodeCount;
            TrackPoint point = ARS.TrackPoints[node];
            Vector3 direction = new Vector3(point.Direction.X, point.Direction.Y, 0f);
            if (direction == Vector3.Zero) direction = Vector3.WorldNorth;
            direction.Normalize();
            Vector3 right = Vector3.Cross(direction, Vector3.WorldUp);
            float side = slot % 2 == 0 ? 1f : -1f;
            Car.Position = point.Position + right * side * (point.TrackHalfWidth + DNFSlotOffsetMeters) + new Vector3(0f, 0f, 0.5f);
            Car.Heading = direction.ToHeading();
            Car.Velocity = Vector3.Zero;
            if (ARS.DNFUnderTrack)
            {
                Car.Position = new Vector3(Car.Position.X, Car.Position.Y, Car.Position.Z - DNFUnderTrackMeters);
                Car.FreezePosition = true;
            }
            IsDNF = true;
            BaseBehavior = RacerBaseBehavior.FinishedStandStill;
            FinishStuckRecovery();
        }

        void UpdateStuckCheck()
        {
            if (_stuckPhase != StuckPhase.None)
            {
                ResetStuckArmTimers();
                return;
            }

            int now = Game.GameTime;

            if (now < _stuckRecoveryCooldownEndTime || BaseBehavior != RacerBaseBehavior.Race || !Driver.IsSittingInVehicle(Car))
            {
                ResetStuckArmTimers();
                return;
            }

            // Not in the opening seconds of a race: a car still crawling off the line is launching, not stuck, and the
            // pack around it is not a place to reverse into.
            if (now < _stuckArmAllowedTime)
            {
                ResetStuckArmTimers();
                return;
            }

            if (PlanWantsToMove())
            {
                if (_stuckStationarySince == 0) _stuckStationarySince = now;
            }
            else _stuckStationarySince = 0;

            if (OutOfTrackDistance() > 0f)
            {
                if (_stuckOffTrackSince == 0) _stuckOffTrackSince = now;
            }
            else _stuckOffTrackSince = 0;

            bool stationary = _stuckStationarySince != 0 && now - _stuckStationarySince >= RecoveryTriggerMs;
            bool offTrack = _stuckOffTrackSince != 0 && now - _stuckOffTrackSince >= RecoveryTriggerMs;

            if (stationary) StartStuckRecovery(now, StuckPhase.Reverse);
            else if (offTrack) StartStuckRecovery(now, StuckPhase.Drive);
        }

        // A car wants to be somewhere it is not: too slow, with its own plan asking for more. That is the stuck case,
        // as opposed to being held still on purpose.
        bool PlanWantsToMove()
        {
            return ARS.MpsToMph(Car.Velocity.Length()) < RecoveryTriggerMph && Brain.CurrentIntention.Speed - Car.Velocity.Length() > ARS.MphToMps(RecoveryPlanGapMph);
        }

        void ResetStuckArmTimers()
        {
            _stuckStationarySince = 0;
            _stuckOffTrackSince = 0;
        }

        void StartStuckRecovery(int now, StuckPhase phase)
        {
            _stuckPhase = phase;
            if (phase == StuckPhase.Reverse) _stuckPhaseEndTime = now + StuckReverseMs;
            _stuckRecoveryStartTime = now;
            _stuckAgainSince = 0;
            _stuckExitSince = 0;
            _stuckMoveSamplePosition = Car.Position;
            _stuckMoveSampleTime = now;
            ResetStuckArmTimers();
        }

        void UpdateStuckRecovery()
        {
            if (BaseBehavior != RacerBaseBehavior.Race || !Driver.IsSittingInVehicle(Car))
            {
                if (_stuckPhase != StuckPhase.None) FinishStuckRecovery();
                return;
            }

            if (_stuckPhase == StuckPhase.None) return;

            int now = Game.GameTime;

            if (now - _stuckMoveSampleTime >= StuckMoveSampleMs)
            {
                bool couldNotMove = Car.Position.DistanceTo(_stuckMoveSamplePosition) < StuckMoveMeters;
                bool escapeSpent = now - _stuckRecoveryStartTime >= StuckRecoveryTimeMs;
                _stuckMoveSamplePosition = Car.Position;
                _stuckMoveSampleTime = now;
                if (couldNotMove && escapeSpent)
                {
                    SnapToTrack();
                    return;
                }
            }

            if (_stuckPhase == StuckPhase.Reverse)
            {
                if (now - _stuckRecoveryStartTime >= StuckRecoveryTimeMs) SnapToTrack();
                else if (now >= _stuckPhaseEndTime) _stuckPhase = StuckPhase.Drive;
                return;
            }

            if (PlanWantsToMove())
            {
                if (_stuckAgainSince == 0) _stuckAgainSince = now;
            }
            else _stuckAgainSince = 0;

            if (_stuckAgainSince != 0 && now - _stuckAgainSince >= RecoveryTriggerMs)
            {
                _stuckPhase = StuckPhase.Reverse;
                _stuckPhaseEndTime = now + StuckReverseMs;
                _stuckMoveSamplePosition = Car.Position;
                _stuckMoveSampleTime = now;
                return;
            }

            if (RecoveryExitHolds()) FinishStuckRecovery();
        }

        // Off the track the nearest edge is the way back; the normal aim is the centre, which crosses the whole width.
        float NearestEdgeLane(float carOffset, float drivableEdge)
        {
            return carOffset >= 0f ? drivableEdge : -drivableEdge;
        }

        // Any part of the car on the drivable bound and moving forward, held: a reverse roll is not a recovered car.
        bool RecoveryExitHolds()
        {
            bool onTrack = Math.Abs(Brain.CurrentPerception.DeviationFromCenter) <= CurrentTrackPoint.TrackHalfWidth + VehicleData.BoundingBox * 0.5f;
            bool movingForward = ARS.MpsToMph(Vector3.Dot(Car.Velocity, Car.ForwardVector)) > RecoveryExitMph;
            if (!onTrack || !movingForward)
            {
                _stuckExitSince = 0;
                return false;
            }
            if (_stuckExitSince == 0) _stuckExitSince = Game.GameTime;
            return Game.GameTime - _stuckExitSince >= RecoveryExitHoldMs;
        }

        void FinishStuckRecovery()
        {
            _stuckPhase = StuckPhase.None;
            _stuckPhaseEndTime = 0;
            _stuckRecoveryStartTime = 0;
            _stuckAgainSince = 0;
            _stuckExitSince = 0;
            _stuckRecoveryCooldownEndTime = Game.GameTime + StuckRecoveryCooldownMs;
        }

        void ApplyStuckRecoveryOverride()
        {
            if (_stuckPhase != StuckPhase.Reverse) return;

            Control.Brake = 0f;
            Control.BrakeReason = BrakeReason.StuckRecovery;
            Control.BrakeReasonLevel = 0f;
            Control.ThrottleReason = ThrottleReason.StuckRecovery;
            Control.ThrottleReasonLevel = 0f;
            Control.Throttle = -0.5f;
            Control.SteerDegrees = 0f;
        }

        void SnapToTrack()
        {
            Vector3 direction = new Vector3(CurrentTrackPoint.Direction.X, CurrentTrackPoint.Direction.Y, 0f);
            if (direction == Vector3.Zero) direction = Vector3.WorldNorth;
            direction.Normalize();
            Vector3 right = Vector3.Cross(direction, Vector3.WorldUp);
            float side = ARS.SignedLaneOffset(Car.Position, CurrentTrackPoint.Position, CurrentTrackPoint.Direction) >= 0f ? 1f : -1f;
            float laneOffset = Math.Max(CurrentTrackPoint.TrackHalfWidth - VehicleData.BoundingBox * 0.5f, 0f) * side;
            Car.Position = CurrentTrackPoint.Position + right * laneOffset + new Vector3(0f, 0f, 0.5f);
            Car.Heading = direction.ToHeading();
            Car.Velocity = direction * ARS.MphToMps(10f);
            FinishStuckRecovery();
        }

        void UpdatePerceivedGrip()
        {


            float handlingGrip = Function.Call<float>((Hash)0xA132FB5370554DB0, Car);
            handlingGrip = ARS.Clamp(handlingGrip, 0.1f, 100f);
            handlingGrip /= 1f + 0.035f * Handling.Downforce;

            // TEMP experiment: a car carrying the off-road FLAG has permanently raised gravity (set once in
            // Initialize, not a surface state), and its grip is scaled with it.
            float gravityGs = Handling.Gravity / 9.8f;
            if (gravityGs > 1f) handlingGrip *= gravityGs;

            GroundGripMultiplier = ARS.MeanWheelGripMultiplier(Car);

            // Centripetal acceleration v²/r stands in for the cornering load the engine scales downforce by; see
            // GetDownforceGsAtSpeed for why world-frame lateral velocity cannot be used here.
            float forwardMs = ARS.GetForwardSpeed(Car);
            float routeRadius = Brain.CurrentPerception.CurveRadiusToFollowPoint;
            float lateralMs = 0f;
            if (routeRadius > 1f && !float.IsNaN(routeRadius) && !float.IsInfinity(routeRadius))
                lateralMs = (forwardMs * forwardMs) / routeRadius;
            float dfGs = ARS.GetDownforceGsAtSpeed(this, forwardMs, lateralMs);

            VehicleData.BaseMechanicalGrip = handlingGrip;
            VehicleData.DownforceGripBonus = dfGs;
            VehicleData.CurrentMechanicalGrip = (VehicleData.BaseMechanicalGrip + VehicleData.DownforceGripBonus) * GroundGripMultiplier;

            if (!_gripLogged)
            {
                _gripLogged = true;
                ARS.Log(ARS.LogImportance.Info, "Grip for " + Car.DisplayName + ": base " + VehicleData.BaseMechanicalGrip
                    + ", current " + VehicleData.CurrentMechanicalGrip + " (steer cap k " + SteerReductionPerMps + ")");
            }

            // Sampling the wheel-pushed Gs at ~3 Hz so the overspeed reason can glide on a cheap read.
            if (Game.GameTime - _lastGsCheck >= 333) // ~3 Hz
            {
                _lastGsCheck = Game.GameTime;

                // Compare measured forward Gs against wheel-pushed Gs: GTA lets an uphill car accelerate beyond
                // what its wheel power should produce, and this is the correction for that.
                if (ARS.OverspeedEnabled)
                {
                    List<float> wheelPowers = ARS.WheelPowers(Car);
                    float wheelGs = 0f;
                    foreach (float p in wheelPowers) wheelGs += p;
                    float measuredGs = VehicleData.GetLongitudinalGs(Car.ForwardVector);
                    float excess = measuredGs - wheelGs - 0.1f;
                    VehicleData.OverspeedMeasuredGs = measuredGs;
                    VehicleData.OverspeedWheelGs = wheelGs;
                    VehicleData.OverspeedExcessGs = excess;
                }
            }


            if (Math.Abs(Brain.CurrentPerception.DeviationFromCenter) < CurrentTrackPoint.TrackHalfWidth && RacePosition <= 2 && !ARS.TerrainGripMultipliers.ContainsKey(CurrentTrackPoint.Node))
            {
                ARS.TerrainGripMultipliers.Add(CurrentTrackPoint.Node, GroundGripMultiplier);
            }

            VehicleData.YawRotationPerSecondDegrees = ARS.RadToDeg(Function.Call<Vector3>(Hash.GET_ENTITY_ROTATION_VELOCITY, Car).Z);
        }
        public void UpdateRivals()
        {
            if (ARS.NoCollision)
            {
                foreach (Rival r in Brain.Rivals) r.RivalRacer = null;
                return;
            }

            Vector3 myPosition = Car.Position;
            Vector3 hoodPosition = myPosition + Car.ForwardVector;
            List<Racer> candidates = new List<Racer>();
            List<float> candidateDistances = new List<float>();
            foreach (Racer r in ARS.Racers)
            {
                if (r.Car.Handle == Car.Handle) continue;
                if (ARS.DNFUnderTrack && r.IsDNF) continue;
                Vector3 rivalPosition = r.Car.Position;
                if (Vector3.Distance(rivalPosition, myPosition) >= RivalSearchRangeMeters) continue;
                candidates.Add(r);
                candidateDistances.Add((rivalPosition - hoodPosition).LengthSquared());
            }

            foreach (Rival r in Brain.Rivals) r.RivalRacer = null;
            for (int slot = 0; slot < Brain.Rivals.Count && candidates.Count > 0; slot++)
            {
                int nearest = 0;
                for (int i = 1; i < candidates.Count; i++)
                {
                    if (candidateDistances[i] < candidateDistances[nearest]) nearest = i;
                }
                Brain.Rivals[slot].RivalRacer = candidates[nearest];
                candidates.RemoveAt(nearest);
                candidateDistances.RemoveAt(nearest);
            }

            // A re-owned slot must not keep the last occupant's numbers: every consumer reads the record, not the car.
            UpdateRivalInfo();
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
            }
            else
            {
                if (Car.CurrentBlip != null && Car.CurrentBlip.Exists()) Car.CurrentBlip.Remove();
                Car.Delete();
            }
        }
    }
}


