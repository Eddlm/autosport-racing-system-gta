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
        public Ped Driver;
        public Vehicle Car;
        public Team TeamRole = Team.None;
        public bool ControlledByPlayer = false;
        public RacerBaseBehavior BaseBehavior = RacerBaseBehavior.GridWait;
        public RaceState RCStatus = RaceState.NotInitiated;


        public VehicleControl Control = new VehicleControl();
        public RacerBrain Brain = new RacerBrain();


        public VehicleState VehicleData = new VehicleState();
        public HandlingData Handling = new HandlingData();
        public float GroundGripMultiplier = 1f;
        Vector3 _lastSpeed;


        public enum LookAhead { SteerRef, QuarterSec, HalfSec, ThreeQuarterSec, OneSec, OneHalfSec, TwoSec };
        public TrackPoint CurrentTrackPoint = new TrackPoint();
        public Dictionary<LookAhead, TrackPoint> LookAheads = new Dictionary<LookAhead, TrackPoint>();


        public List<TimeSpan> LapTimes = new List<TimeSpan>();
        public int LapStartTime = 0;
        public int Lap = 0;
        public int RacePosition = 0;
        public bool CanRegisterNewLap = true;
        bool _hasLeftLapArmNode = false;
        public bool FinishedPointToPoint = false;


        int _halfSecondTick = 0;
        int _oneSecondTick = 0;
        int _lastCoreTick = 100;
        int TimeSince_lastCoreTick => (int)ARS.Clamp(Game.GameTime - _lastCoreTick, 1, 9999);


        int _lastStuckGameTime = 0;
        public bool IsStuckByThrottle = false;
        const int StuckCheckTimeMs = 2000;
        bool _isRecoveringFromStuck = false;
        int _stuckRecoveryEndTime = 0;
        const int StuckRecoveryTimeMs = 2000;
        int _stuckRecoveryAttempts = 0;
        public int StuckRecoveryAttemptsNow => _stuckRecoveryAttempts;
        public bool IsRecoveringFromStuckNow => _isRecoveringFromStuck;


        struct TrailSample
        {
            public Vector3 Position;
            public float CombinedInput;

            public TrailSample(Vector3 position, float combinedInput)
            {
                Position = position;
                CombinedInput = combinedInput;
            }
        }
        List<TrailSample> _trailSamples = new List<TrailSample>();


        public float RouteWindowStart = 0.5f; // seconds of lookahead; driven by Pressure in UpdatePressure (0..1)
        public float RouteWindowSize = 2.0f;


        float _avoidLeftWall = 0f;
        float _avoidRightWall = 0f;
        bool _avoidWallsInitialized = false; // walls start collapsed at center; snap fully open on first use
        float _targetLane = 0f; // final clamped lane target after all lane tweaks, set in ComputeSteering
        float _rawCornerLane = 0f; // System 1 corner lane (outside approach) before walls/avoidance, set in ComputeSteering (debug; also drives the outside-vs-inside gain split)
        float _cornerSpd = 999f;
        float _debugCornerSpd = 999f; // final cornerSpd after slope/offset, captured in ComputeTargetSpeed (debug)
        float _debugFollowTrackSpd = 999f; // final followTrackSpd after slope/pressure, captured in ComputeTargetSpeed (debug)
        // Outside-approach entry latch: decided once when the car enters the 5s window for a
        // given corner. The outside hold only makes sense if the car needs to brake — entries
        // below apex speed + 20 mph skip the outside line entirely (System 2 inside covers it).
        bool _approachOutsideDecided = false;
        bool _approachHoldsOutside = false;
        int _approachCornerNode = -1;
        int _divebombApexNode = -1; // apex the active divebomb armed against; off once passed
        int _defendApexNode = -1; // apex the active defend armed against; off once passed or target overtakes
        // Unified max-acceleration scalar in [-1, +1]. +1 = full throttle, 0 = coast, -1 = full brake.
        // Re-rises at 0.33/s toward +1 each frame (frame-independent via TickScale).
        // Each "lift off" source lowers it via Math.Min; applied once in ConvertSpeedToPedals.
        float _accelerationCap = 1f;
        // Unified speed ceiling (m/s). Self-rises at SpeedCapRiseRate toward 999 each tick.
        // Speed-based concern sources pull it down via Math.Min; ConvertSpeedToPedals clamps
        // the intended speed against it, so throttle/brake always come from the speed loop.
        float _speedCap = 999f;
        const float SpeedCapRiseRate = 30f;             // m/s the cap recovers per second
        const float ProjectionSteerDeadzoneDegrees = 2f; // projection cap ignored while |steer| is within this



        bool _isPassengerized = false;

        const bool AiNitrousEnabled = false;    // TEMP DISABLED - re-enable after tuning pass
        const float NitrousPowerMultiplier = 2.5f;
        const float NitrousCornerLookaheadSeconds = 8f;
        const int NitrousDurationMs = 3000;
        const string NitrousPtfxAsset = "veh_xs_vehicle_mods";
        const ulong CheatPowerIncreaseHash = 0xB59E4BD37AE292DB;
        const ulong FullyChargeNitrousHash = 0x1A2BCC8C636F9226;
        const ulong OverrideNitrousLevelHash = 0xC8E9B6B71B8E660D;

        public Maneuver ActiveManeuver = new Maneuver();
        int _nitrousActiveUntil = 0;


        public float Aggression = 50f;
        public float Pressure = 0f;
        float _laneGainDivisor = 100f; // per-racer variation of the lane-pursuit gain curve scale
        const float PressureRange = 100f;
        const float PressureProximityRange = 100f;
        const float PressureRisePerSecond = 2f;
        const float PressureFallPerSecond = 30f;
        const float PressureMaxSpeedOffset = 5f; // pressure overspeed at Pressure=100 (m/s)

        public Racer(Vehicle RacerCar, Ped RacerPed)
        {
            Car = RacerCar;
            Driver = RacerPed;
            try { Name = RacerCar.FriendlyName; } catch (Exception) { Name = "Racer"; }
            if (Name == "NULL" || Name == null) { try { Name = Car.DisplayName.ToString()[0].ToString().ToUpper() + Car.DisplayName.ToString().Substring(1).ToLowerInvariant(); } catch (Exception) { Name = "Racer"; } }

            if (Driver.IsPlayer) ControlledByPlayer = true;
            _halfSecondTick = Game.GameTime + (ARS.GetRandomInt(10, 50));
            _laneGainDivisor = (float)ARS.GetRandomInt(90, 110);

            if (!ControlledByPlayer)
            {

                try { Driver.BlockPermanentEvents = true; } catch (Exception) { }
                try { Driver.AlwaysKeepTask = true; } catch (Exception) { }
                Function.Call(GTA.Native.Hash.SET_DRIVER_ABILITY, Driver, 0f);
                Function.Call(GTA.Native.Hash.SET_DRIVER_AGGRESSIVENESS, Driver, 0f);

                if (ARS.DevSettingsFile.GetValue<int>("RACERS", "AIRacerAutofix", 1) == 2)
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
                else if (ARS.DevSettingsFile.GetValue<int>("RACERS", "AIRacerAutofix", 1) == 1)
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
                ARS.SetSteerAngle(Car, 0.5f);
                ARS.SetThrottle(Car, 0f);
                ARS.SetBrakes(Car, 0f);

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
            Handling.Downforce = ARS.GetDownforce(Car);
            if (Handling.Downforce > 100) Handling.Downforce *= 0.1f;

            Handling.LateralTractionCurve = ARS.RadToDeg(ARS.GetTRCurveLat(Car));
            if (Handling.LateralTractionCurve < 1 || Handling.LateralTractionCurve > 100) Handling.LateralTractionCurve = 22;
            ARS.Log(ARS.LogImportance.Info, "TRlat for " + Car.DisplayName + ":" + Handling.LateralTractionCurve + "º");

            Handling.BrakingAbility = Car.MaxBraking;
            Handling.EstimatedTopSpeed = ARS.EngineTopSpeed(Car);
            Handling.Acceleration = Function.Call<float>(Hash.GET_VEHICLE_ACCELERATION, Car);


            VehicleData.SteeringLock = ARS.RadToDeg(ARS.GetSteerLock(Car));
            if (VehicleData.SteeringLock < 1 || VehicleData.SteeringLock > 100) VehicleData.SteeringLock = 40;
            ARS.Log(ARS.LogImportance.Info, "Steerlock for " + Car.DisplayName + ":" + VehicleData.SteeringLock + "º");
            Control.SteerDegrees = 0f;
            CurrentTrackPoint = ARS.TrackPoints.Last();
            Control.Brake = 0f;
            Control.Throttle = 0f;

            LapTimes.Clear();
            LapStartTime = 0;
            Lap = 0;
            CanRegisterNewLap = true;
            _hasLeftLapArmNode = false;

            string flags = ARS.GetHandlingFlags(Car).ToString("X");
            int flagsHex = Convert.ToInt32(flags, 16);
            bool hasOffroad = (flagsHex & 0x800000) != 0 || (flagsHex & 0x200000) != 0;
            if (hasOffroad) Handling.Gravity *= 1.2f;

            BaseBehavior = RacerBaseBehavior.GridWait;
            FinishedPointToPoint = false;

            Handling.Grip = Function.Call<float>((Hash)0xA132FB5370554DB0, Car) * (Handling.Gravity / 9.8f);

            VehicleData.PerformanceIndex = (int)((Handling.EstimatedTopSpeed * 5) + (Handling.Grip * 100) + (Handling.Acceleration * 500));
            VehicleData.TextPerformanceIndex = ((int)(Handling.EstimatedTopSpeed / 1.2) + " | " + (int)(Handling.Acceleration * 200) + " | " + (int)(Handling.Grip * 20));

            Car.Repair();
        }

        public void ComputeSteering()
        {
            if (!TryGetSteerContext(out TrackPoint steerRefPoint, out float roadWide))
            {
                Control.SteerDegrees = 0f;
                return;
            }



            Vector3 carForward = Car.Velocity.LengthSquared() > 0.01f
                ? Car.Velocity.Normalized
                : Car.ForwardVector;
            float headingErrorDeg = -Vector3.SignedAngle(
                steerRefPoint.Direction, carForward, Vector3.WorldUp);
            if (float.IsNaN(headingErrorDeg) || float.IsInfinity(headingErrorDeg))
                headingErrorDeg = 0f;


            headingErrorDeg *= 0.5f;

            float speedMps = Math.Max(Car.Velocity.Length(), 1f);
            float carHalfWidth = VehicleData.BoundingBox * 0.5f;


            float naturalLane = ComputeHighSpeedLane(roadWide);

            // System 1: corner outside approach — temporarily overrides System 2's lane
            // while in its approach window. Returns 0f when it lets go, so System 2's
            // inside edge takes over naturally.
            bool cornerActive = Brain.Corner != null && Lap > 0;
            float cornerLane = cornerActive ? ComputeCornerTargetLane(steerRefPoint, speedMps) : 0f;
            if (cornerLane != 0f) naturalLane = cornerLane;
            _rawCornerLane = cornerLane;

            // Avoid-ahead: pick a lane to pass a rival ahead. Computed fresh each frame.
            // During a divebomb the committed inside lane IS the pass — generic avoidance
            // would just re-pick "the side with more room" and cancel the maneuver.
            float avoidAheadLane = 0f;
            if (ActiveManeuver.Type != ManeuverType.DiveBomb)
            {
                avoidAheadLane = ComputeAvoidAheadLane(roadWide);
                if (avoidAheadLane != 0f) naturalLane = avoidAheadLane;
            }



            float clampedLane = ApplyRivalWalls(naturalLane, roadWide);
            _targetLane = clampedLane;


            float recoveryDeg = 0f;
            float absDev = Math.Abs(Brain.CurrentPerception.DeviationFromCenter);
            float safeEdge = roadWide - carHalfWidth;
            float overshoot = absDev - safeEdge;
            if (overshoot > 0f)
            {
                float maxRecoveryDeg = ARS.Remap(speedMps, 10f, 50f, 10f, 2f);
                float severity = Math.Min(overshoot / Math.Max(safeEdge, 1f), 1f);
                recoveryDeg = Math.Sign(Brain.CurrentPerception.DeviationFromCenter) * maxRecoveryDeg * severity;
            }



            float laneBiasDeg = 0f;
            float trackBound = roadWide - carHalfWidth;
            bool hasActiveGuidance = Math.Abs(clampedLane) > 0.01f
                || _avoidLeftWall > -trackBound
                || _avoidRightWall < trackBound;
            if (hasActiveGuidance)
            {
                float currentLane = Brain.CurrentPerception.DeviationFromCenter;
                float lookaheadDist = steerRefPoint.Position.DistanceTo(Car.Position);
                if (lookaheadDist < 1f) lookaheadDist = speedMps * 1.5f;


                float laneError = clampedLane - currentLane;
                laneBiasDeg = -(float)(Math.Atan2(laneError, lookaheadDist) * (180.0 / Math.PI));
                // Gain from the degree error (atan2 output), not raw meters. Self-normalizes
                // via the lookahead distance — same meters produce fewer degrees at high speed.
                float expGain = (float)Math.Pow(Math.Min(Math.Abs(laneBiasDeg) / _laneGainDivisor, 1f), 0.66f);
                expGain = Math.Min(expGain, 0.3f);
                laneBiasDeg *= expGain;
            }


            float totalTargetDeg = headingErrorDeg + laneBiasDeg + recoveryDeg;


            const float steerKP = 1.0f;
            float gripForKD = VehicleData.CurrentMechanicalGrip;
            if (float.IsNaN(gripForKD) || float.IsInfinity(gripForKD) || gripForKD <= 0f) gripForKD = 1f;
            float steerKD = 0.5f / gripForKD;
            float pTerm = steerKP * totalTargetDeg;
            float dTerm = steerKD * VehicleData.YawRotationPerSecondDegrees;
            Control.SteerDegrees = pTerm - dTerm;



            if (Math.Sign((int)VehicleData.SlideAngle) == Math.Sign((int)VehicleData.YawRotationPerSecondDegrees))
            {
                float slideCounterSteer = VehicleData.SlideAngle * ARS.Remap(
                    Math.Abs(VehicleData.SlideAngle), 0, Handling.LateralTractionCurve * 1.2f, 0.5f, 1.2f, true);
                Control.SteerDegrees -= slideCounterSteer;
            }

            // Any NaN/Inf in the steering output would be turned into full-lock by ApplySteerLimits
            // (float.NaN.CompareTo(min) < 0 makes Clamp return the min bound). Kill it here, at the
            // source, before any downstream clamp can corrupt it.
            if (float.IsNaN(Control.SteerDegrees) || float.IsInfinity(Control.SteerDegrees))
                Control.SteerDegrees = 0f;



            bool TryGetSteerContext(out TrackPoint localSteerRef, out float localRoadWide)
            {
                localSteerRef = null;
                localRoadWide = 0f;

                if (BaseBehavior == RacerBaseBehavior.GridWait
                    || BaseBehavior == RacerBaseBehavior.FinishedStandStill
                    || CurrentTrackPoint.Node < 3)
                {
                    return false;
                }

                if (!LookAheads.TryGetValue(LookAhead.SteerRef, out localSteerRef)
                    || localSteerRef == null)
                {
                    return false;
                }

                localRoadWide = localSteerRef.TrackHalfWidth;
                return true;
            }
        }






        // System 2 — high-speed line. Positions the car on the inside edge of the track's
        // curvature, driven entirely by the track geometry (NOT the corner system). Always
        // evaluated; the corner system (System 1) only temporarily overrides this lane for
        // its outside approach. Direction uses the same ±20-node SignedAngle construction as
        // the corner generator, so inside/outside convention matches the corner system.
        float ComputeHighSpeedLane(float roadWide)
        {
            // Disabled on corners wider than 250 m — beyond that the inside-edge target is
            // meaningless (corners are effectively straights), so the lane is explicitly off.
            if (Brain.CurrentPerception.HighSpeedCurveRadius > 250f) return 0f;

            int count = ARS.TrackPoints.Count;
            int backNode, fwdNode;
            if (ARS._isPointToPoint)
            {
                backNode = (int)ARS.Clamp(CurrentTrackPoint.Node - 20, 0, count - 1);
                fwdNode = (int)ARS.Clamp(CurrentTrackPoint.Node + 20, 0, count - 1);
            }
            else
            {
                backNode = ((CurrentTrackPoint.Node - 20) % count + count) % count;
                fwdNode = ((CurrentTrackPoint.Node + 20) % count + count) % count;
            }
            Vector3 backDir = ARS.TrackPoints[backNode].Direction;
            Vector3 fwdDir = ARS.TrackPoints[fwdNode].Direction;

            float signedAngle = Vector3.SignedAngle(backDir, fwdDir, Vector3.WorldUp);
            if (float.IsNaN(signedAngle) || float.IsInfinity(signedAngle)) return 0f;

            // Deadzone: ignore near-straight sections so track-direction noise can't flip the
            // inside/outside target frame to frame.
            if (Math.Abs(signedAngle) < 1f) return 0f;

            float cornerDir = Math.Sign(signedAngle);
            return -cornerDir * roadWide;
        }
        // System 1 — corner outside approach. Its only job: hold the outside line during the
        // approach window (5s → 1s before the apex). It lets go within 1s of the apex so the
        // high-speed line (System 2) takes over the inside naturally. Returns 0f (no override)
        // outside the window.
        float ComputeCornerTargetLane(TrackPoint steerRefPoint, float speedMps)
        {
            CornerPoint c = Brain.Corner.Point;
            int apexNode = c.Node;

            // New corner target — reset the outside-hold entry latch.
            if (apexNode != _approachCornerNode)
            {
                _approachCornerNode = apexNode;
                _approachOutsideDecided = false;
                _approachHoldsOutside = false;
            }

            float distToApexNodes = Math.Abs(apexNode - CurrentTrackPoint.Node);
            float timeToApex = distToApexNodes / Math.Max(speedMps, 1f);
            const float approachStartTime = 5.0f;
            // Wait for the approach window (5s before the apex) before positioning. A slow car
            // can take the corner at current speed, so there's no reason to commit early; and
            // once inside the window we're committed — hold the line regardless of speed.
            if (timeToApex > approachStartTime) return 0f;

            // Entry decision: the outside hold only makes sense if the car needs to brake for
            // this corner. Latched once at window entry — if the car entered below apex speed
            // + 20 mph, skip the outside line entirely and let System 2's inside cover it.
            if (!_approachOutsideDecided)
            {
                _approachHoldsOutside = speedMps >= _cornerSpd + ARS.MphToMps(20f);
                _approachOutsideDecided = true;
            }

            float cornerDir = Math.Sign(c.Angle);
            if (cornerDir == 0f) return 0f;

            float halfWidth = steerRefPoint.TrackHalfWidth;
            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            float safeBound = halfWidth - carHalfWidth;

            // Hold the outside line until we're this many seconds before the apex, then let go —
            // the high-speed line (System 2) takes over the inside edge. The let-go time scales
            // with full track width: crossing from the outside edge to the inside edge takes
            // longer on a wide track, so the car must release earlier. 10 m full width → 1 s,
            // 20 m → 2 s, etc.
            float holdOutsideUntil = (steerRefPoint.TrackHalfWidth * 2f) / 10f;
            if (_approachHoldsOutside && timeToApex > holdOutsideUntil)
            {
                // Corner-commit maneuvers (Divebomb / DefendLane): instead of the outside line,
                // sit right beside the target rival on the corner's inside (-cornerDir). The
                // divebomber does it to out-brake an ahead rival; the defender to block a behind
                // rival from diving, forcing them around the outside.
                bool isCornerCommit = ActiveManeuver.Target != null
                    && (ActiveManeuver.Type == ManeuverType.DiveBomb || ActiveManeuver.Type == ManeuverType.DefendLane);
                if (isCornerCommit)
                {
                    Rival target = Brain.Rivals.FirstOrDefault(r => r.RivalRacer == ActiveManeuver.Target);
                    if (target != null && target.RivalRacer.Car.Exists())
                    {
                        // OccupiedLaneWidth = (myBox+rivalBox)/2 = exact edge-to-edge gap;
                        // small margin on top so we're beside, never touching.
                        float gap = target.OccupiedLaneWidth + 0.6f;
                        float commitLane = target.OccupiedLane + (-cornerDir) * gap;
                        return ARS.Clamp(commitLane, -safeBound, safeBound);
                    }
                }
                return cornerDir * halfWidth;
            }
            return 0f;
        }













        float ComputeAvoidAheadLane(float roadWide)
        {
            float carHalfWidth = VehicleData.BoundingBox * 0.5f;
            float trackBound = roadWide - carHalfWidth;
            float aggroBuffer = ARS.Remap(Aggression, 100f, 0f, 0.2f, 1.2f, true);
            float currentLane = Brain.CurrentPerception.DeviationFromCenter;

            // Loop through rivals (sorted by distance). Compute avoidance for each
            // qualifying rival. If a second one triggers on the opposite side, average
            // the two results (thread the needle) and stop. Same side — just keep the first.
            float firstTarget = 0f;
            bool firstGoLeft = false;
            bool foundFirst = false;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null) continue;
                if (r.RelativePosition != RelativePos.Ahead) continue;
                if (r.SecondsToReach < 0f || r.SecondsToReach > 3f) continue;
                if (Math.Abs(r.DirectionDiff) > 20f) continue;

                float rivalLane = r.OccupiedLane;
                float buffer = r.OccupiedLaneWidth + aggroBuffer;

                // Pick the side with more room
                float roomLeft = rivalLane - buffer + trackBound;
                float roomRight = trackBound - (rivalLane + buffer);
                bool goLeft = roomLeft > roomRight;

                float targetLane = goLeft
                    ? rivalLane - buffer - carHalfWidth
                    : rivalLane + buffer + carHalfWidth;

                // If the chosen side is off-track, flip to the other side
                if (Math.Abs(targetLane) > trackBound)
                {
                    targetLane = goLeft
                        ? rivalLane + buffer + carHalfWidth
                        : rivalLane - buffer - carHalfWidth;
                    goLeft = !goLeft;
                }

                // If both sides are off-track, skip this rival
                if (Math.Abs(targetLane) > trackBound) continue;

                // If this car is already on the far side, the rival is far off to the
                // side and our current line clears it — no avoidance needed for this one.
                if (goLeft && currentLane <= targetLane) continue;
                if (!goLeft && currentLane >= targetLane) continue;

                if (!foundFirst)
                {
                    firstTarget = targetLane;
                    firstGoLeft = goLeft;
                    foundFirst = true;
                }
                else
                {
                    // Second qualifying rival — only average if on the opposite side.
                    if (goLeft != firstGoLeft)
                        return (firstTarget + targetLane) * 0.5f;
                    // Same side — the first rival's avoidance already covers this one.
                    return firstTarget;
                }
            }

            return foundFirst ? firstTarget : 0f;
        }

        float ApplyRivalWalls(float targetLane, float roadWide)
        {
            float carHalfWidth = VehicleData.BoundingBox * 0.5f;

            float trackBound = roadWide - carHalfWidth;

            // Walls start collapsed at 0 (center) and would clamp every lane to a 0.X near-center
            // offset (Clamp with min>max inverts). Snap them fully open on first use so the clamp
            // window is the whole track from the very first frame.
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

                bool isRelevant = false;
                bool rivalIsLeft = false;

                if (r.RelativePosition == RelativePos.Left)
                {
                    isRelevant = true;
                    rivalIsLeft = true;
                }
                else if (r.RelativePosition == RelativePos.Right)
                {
                    isRelevant = true;
                    rivalIsLeft = false;
                }

                if (!isRelevant) continue;


                float aggroBuffer = ARS.Remap(Aggression, 100f, 0f, 0.2f, 1.2f, true);
                // Buffer = average of both bounding boxes (OccupiedLaneWidth = (myBox+rivalBox)/2,
                // the exact edge-to-edge touch distance — mandatory floor, cars must never touch)
                // + the aggro extra. Same buffer whether this car is ahead or behind the rival —
                // the ahead/behind cap was tried and removed (made overlap avoidance worse).
                float rivalBuffer = r.OccupiedLaneWidth + aggroBuffer;

                if (rivalIsLeft)
                {
                    float wall = r.OccupiedLane + rivalBuffer;
                    if (wall > targetLeftWall) targetLeftWall = wall;
                    leftConstrained = true;
                }
                else
                {
                    float wall = r.OccupiedLane - rivalBuffer;
                    if (wall < targetRightWall) targetRightWall = wall;
                    rightConstrained = true;
                }
            }


            float openRate = 2f * TickScale;
            if (leftConstrained)
                _avoidLeftWall = targetLeftWall;
            else
                _avoidLeftWall = Math.Max(_avoidLeftWall - openRate, -trackBound);

            if (rightConstrained)
                _avoidRightWall = targetRightWall;
            else
                _avoidRightWall = Math.Min(_avoidRightWall + openRate, trackBound);

            // Walls must never switch sides: collapse crossed walls onto their midpoint, ±1m.
            if (_avoidLeftWall > _avoidRightWall)
            {
                float mid = (_avoidLeftWall + _avoidRightWall) * 0.5f;
                _avoidLeftWall = mid - 1f;
                _avoidRightWall = mid + 1f;
            }


            float squeezeThreshold = VehicleData.BoundingBox + 0.2f;
            if (_avoidRightWall - _avoidLeftWall < squeezeThreshold)
            {
                // Squeeze speedcap disabled — revisit (was: cap speed just below velocity to lift/brake).
                // _speedCap = Math.Min(_speedCap, Math.Max(Car.Velocity.Length() - 0.5f, 0f));
            }


            float clampLeft = _avoidLeftWall + carHalfWidth;
            float clampRight = _avoidRightWall - carHalfWidth;
            // Full car-width safety: the inner wall can push the car toward the edge, but the
            // car's center must never cross the track limit (carHalfWidth is already applied
            // above, so its edges stay within roadWide). Clamp the window to the track bounds.
            clampLeft = Math.Max(clampLeft, -trackBound);
            clampRight = Math.Min(clampRight, trackBound);
            // Guard the collapsed-window case: if the walls have crossed (gap < 0 after car
            // inset), Clamp(min>max) would invert. Fall back to the track bound itself — never
            // the raw lane target, which could be the off-track outside edge.
            if (clampLeft > clampRight) return ARS.Clamp(targetLane, -trackBound, trackBound);
            return ARS.Clamp(targetLane, clampLeft, clampRight);
        }

        void ApplySteerLimits()
        {
            // NaN/Inf must never reach the clamp — float.NaN.CompareTo(min) < 0 makes Clamp
            // return the min bound, i.e. instant full-lock. Zero it and bail.
            if (float.IsNaN(Control.SteerDegrees) || float.IsInfinity(Control.SteerDegrees))
            {
                Control.SteerDegrees = 0f;
                return;
            }

            if (Math.Sign(Control.SteerDegrees) == Math.Sign((int)VehicleData.YawRotationPerSecondDegrees))
            {
                float speedBasedSteeringLimit = (float)((VehicleData.BaseMechanicalGrip * Handling.Gravity * VehicleData.WheelBase) / Math.Pow(Car.Velocity.Length() + 0.01f, 2.0f));
                speedBasedSteeringLimit = Math.Max(ARS.RadToDeg(speedBasedSteeringLimit) * 0.75f, Handling.LateralTractionCurve * 0.33f);
                Control.SteerDegrees = ARS.Clamp(Control.SteerDegrees, -speedBasedSteeringLimit, speedBasedSteeringLimit);
            }
        }


        public void Launch()
        {
            Brain.Corner = null;
            VehicleData.AvgGroundStability = 1;
            BaseBehavior = RacerBaseBehavior.Race;
            LapStartTime = Game.GameTime;
            Lap = 1;
            CanRegisterNewLap = false;
            _hasLeftLapArmNode = false;
            Control.HandBrakeTime = Game.GameTime + ARS.GetRandomInt(100, 400);
            Control.MaxThrottle = 1f;
            IsStuckByThrottle = false;
            _lastStuckGameTime = 0;
            _isRecoveringFromStuck = false;
            _stuckRecoveryEndTime = 0;
            _stuckRecoveryAttempts = 0;
            Control.LastAppliedSteerDegrees = 0f;
            if (TeamRole == Team.Cop) Car.SirenActive = true;

        }

        void ConvertSpeedToPedals()
        {
            float newThrottle = 0f;
            float newBrake = 0f;
            float throttleCap = Math.Min(Control.TCSThrottle, 1f);
            float dirSwitchSpeed = ARS.MphToMps(5f);


            if (BaseBehavior == RacerBaseBehavior.GridWait)
            {
                Brain.CurrentIntention.Speed = 99f;
                Control.HandBrakeTime = Game.GameTime + 500;
            }



            if (Brain.CurrentIntention.Speed >= 0f)
            {
                Brain.CurrentIntention.Speed = Math.Min(Brain.CurrentIntention.Speed, ARS.EngineTopSpeed(Car) * 1.3f);
                Brain.CurrentIntention.Speed = Math.Min(Brain.CurrentIntention.Speed, Brain.CurrentIntention.MaxSpeed);
                // Unified speed ceiling (projection and other speed-based sources).
                Brain.CurrentIntention.Speed = Math.Min(Brain.CurrentIntention.Speed, _speedCap);
            }

            if ((Game.GameTime - LapStartTime) < 3000) Brain.CurrentIntention.IntendedSpeedChangeGs = 999;
            else
            {
                float currentLongitudinalSpeed = VehicleData.SpeedVectorLocal.Y;
                Brain.CurrentIntention.IntendedSpeedChangeGs = (Brain.CurrentIntention.Speed - currentLongitudinalSpeed) / 9.8f;
            }            

            float currentForwardSpeed = VehicleData.SpeedVectorLocal.Y;
            float speedErrorGs = Brain.CurrentIntention.IntendedSpeedChangeGs;
            bool wantsReverse = Brain.CurrentIntention.Speed < -0.1f;

            // Apply the unified _accelerationCap (set by steer-in, avoidance and projection sources).
            // Positive part multiplies throttle; negative part's magnitude becomes a MINIMUM brake input.
            if (speedErrorGs > 0.0f)
            {

                if (currentForwardSpeed < -dirSwitchSpeed) newBrake = ARS.Clamp(speedErrorGs * 5f, 0f, 1f);
                else newThrottle = ARS.Clamp(speedErrorGs * 5f, 0f, throttleCap) * Math.Max(_accelerationCap, 0f);
            }
            else if (speedErrorGs < 0.0f)
            {
                float reverseDemand = ARS.Clamp((-speedErrorGs) * 5f, 0f, throttleCap);
                float brakeDemand = ARS.Clamp((-speedErrorGs) * 5f, 0f, 1f);

                if (wantsReverse)
                {

                    if (currentForwardSpeed > dirSwitchSpeed) newBrake = brakeDemand;
                    else newThrottle = -reverseDemand;
                }
                else
                {
                    newBrake = brakeDemand;
                }
            }


            // Brake floor from _accelerationCap (negative part = min brake).
            float accelCapBrakeMin = -Math.Min(_accelerationCap, 0f);
            if (accelCapBrakeMin > 0f)
            {
                newBrake = Math.Max(newBrake, accelCapBrakeMin);
            }

            if (newBrake > 0.0) newThrottle = 0; else newBrake = 0;


            Control.Brake += (newBrake - Control.Brake) * 5 * TickScale;
            Control.Throttle += (newThrottle - Control.Throttle) * 5 * TickScale;
            Control.Throttle = Math.Min(Control.Throttle, Control.MaxThrottle);
            if (Control.MaxThrottle < 1.00f) Control.MaxThrottle += 2 * TickScale;

            if (Brain.CurrentIntention.MaxSpeed < AiConstants.MaxSpeed) Brain.CurrentIntention.MaxSpeed += 15 * TickScale;

        }
        float TickScale => (0.001f * TimeSince_lastCoreTick);




        void TranslateSteerToInput()
        {

            if (float.IsNaN(Control.SteerDegrees) || float.IsInfinity(Control.SteerDegrees)) Control.SteerDegrees = 0f;

            float maxDeltaPerTick = 90 * TickScale;
            float deltaToTarget = ARS.Clamp(Control.SteerDegrees - Control.LastAppliedSteerDegrees, -maxDeltaPerTick, maxDeltaPerTick);
             
            Control.SteerDegrees = Control.LastAppliedSteerDegrees+ deltaToTarget;

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
            if (Brain.Corner != null) cornerSpd = Math.Max(2, ARS.MaxSpeedForBrakingDistance(Brain.Corner.Point, this));

            float followTrackSpd = ComputeRouteSpeed(Brain.CurrentPerception.CurveRadiusToFollowPoint);

            // Gate route speed off within 5s of the apex — the braking plan (cornerSpd) is
            // the only speed authority on corner approach. Route curvature speed reacts to
            // the corner too early and brakes sooner than the braking plan needs.
            if (Brain.Corner != null)
            {
                float timeToApex = ForwardNodeDistance(Brain.Corner.Point.Node) / Math.Max(Car.Velocity.Length(), 1f);
                if (timeToApex <= 5f)
                    followTrackSpd = 999f;
            }

            if (float.IsNaN(cornerSpd) || float.IsInfinity(cornerSpd)) cornerSpd = 999f;
            if (float.IsNaN(followTrackSpd) || float.IsInfinity(followTrackSpd)) followTrackSpd = 999f;
            if (cornerSpd <= 5) cornerSpd = ARS.CornerApexSpeed(Brain.Corner.Point, this);

            // Slope grip loss: TEMPORARILY DISABLED for speed tuning.
            // float slopeAngle = GetFollowPointSlopeAngle();
            // float slopeGripFactor = Math.Max(1f - SlopeGripLossK * (float)Math.Pow(slopeAngle, SlopeGripLossExp), 0.1f);
            // float slopeSpeedFactor = (float)Math.Sqrt(slopeGripFactor);
            // cornerSpd *= slopeSpeedFactor;
            // followTrackSpd *= slopeSpeedFactor;

            // Vertical curvature (crest/dip) grip effect - route speed only.
            // At a crest, the car needs centripetal acceleration v²/r. If that exceeds g, the car lifts off.
            // At a dip, the car is loaded, increasing grip temporarily.
            int count = ARS.TrackPoints.Count;
            int followNode = (int)ARS.Clamp(CurrentTrackPoint.Node + (int)(Car.Velocity.Length() * RouteWindowStart), 0, count - 1);
            int crestStartNode, crestEndNode;
            if (ARS._isPointToPoint)
            {
                crestStartNode = (int)ARS.Clamp(followNode - 3, 0, count - 1);
                crestEndNode = (int)ARS.Clamp(followNode + 3, 0, count - 1);
            }
            else
            {
                crestStartNode = ((followNode - 3) % count + count) % count;
                crestEndNode = ((followNode + 3) % count + count) % count;
            }
            if (crestStartNode != crestEndNode && crestStartNode != followNode && crestEndNode != followNode)
            {
                Vector3 crestStart = ARS.TrackPoints[crestStartNode].Position;
                Vector3 crestMid = ARS.TrackPoints[followNode].Position;
                Vector3 crestEnd = ARS.TrackPoints[crestEndNode].Position;
                float deltaGs = ARS.HillGripDeltaGs(crestStart, crestMid, crestEnd, Car.Velocity.Length());
                // deltaGs < 0 = crest (unloading), deltaGs > 0 = dip (loading)
                float effectiveDeltaGs = deltaGs;
                if (effectiveDeltaGs < 0f) effectiveDeltaGs *= (1f - CrestAggression); // scale only the unloading side
                float verticalGripFactor = Math.Max(1f + effectiveDeltaGs, 0f); // clamp at 0 for liftoff
                followTrackSpd *= (float)Math.Sqrt(verticalGripFactor);
            }

            // Same crest/dip check for the corner apex — sample ±3 nodes around the apex
            // using the apex speed as the velocity reference (that's the speed the car will
            // be doing at the apex). Apply the grip factor to cornerSpd.
            // (Moved after _cornerSpd is set — it needs the pure apex speed as input.)

            // Pressure overspeed: fixed offset (5 m/s at full pressure), applied only to route speed.
            followTrackSpd += PressureMaxSpeedOffset * (Pressure / PressureRange);

            // Store the apex speed (the actual corner constraint) for the corner-approach gate.
            // The braking-plan cornerSpd is the entry speed, which is naturally higher than the
            // current speed and would always skip the approach.
            _cornerSpd = Brain.Corner != null ? ARS.CornerApexSpeed(Brain.Corner.Point, this) : 999f;

            // Corner crest/dip: same vertical curvature check as route speed, but centered on
            // the apex node and using the apex speed as the velocity reference.
            if (Brain.Corner != null)
            {
                int apexNode = Brain.Corner.Point.Node;
                int cornerCrestStart, cornerCrestEnd;
                if (ARS._isPointToPoint)
                {
                    cornerCrestStart = (int)ARS.Clamp(apexNode - 3, 0, count - 1);
                    cornerCrestEnd = (int)ARS.Clamp(apexNode + 3, 0, count - 1);
                }
                else
                {
                    cornerCrestStart = ((apexNode - 3) % count + count) % count;
                    cornerCrestEnd = ((apexNode + 3) % count + count) % count;
                }
                if (cornerCrestStart != cornerCrestEnd && cornerCrestStart != apexNode && cornerCrestEnd != apexNode)
                {
                    Vector3 ccStart = ARS.TrackPoints[cornerCrestStart].Position;
                    Vector3 ccMid = ARS.TrackPoints[apexNode].Position;
                    Vector3 ccEnd = ARS.TrackPoints[cornerCrestEnd].Position;
                    float cornerDeltaGs = ARS.HillGripDeltaGs(ccStart, ccMid, ccEnd, _cornerSpd);
                    float cornerEffectiveDelta = cornerDeltaGs;
                    if (cornerEffectiveDelta < 0f) cornerEffectiveDelta *= (1f - CrestAggression);
                    float cornerVerticalGrip = Math.Max(1f + cornerEffectiveDelta, 0f);
                    cornerSpd *= (float)Math.Sqrt(cornerVerticalGrip);
                }
            }

            _debugCornerSpd = cornerSpd;
            _debugFollowTrackSpd = followTrackSpd;
            Brain.CurrentIntention.Speed = Math.Min(cornerSpd, followTrackSpd);
            // Pressure speed bias: 0..100 pressure maps to -2..+2 m/s on the final intended speed.
            // Applied after the min so it can push above or below the computed speeds; only
            // _speedCap (applied later in ConvertSpeedToPedals) can cap it.
            Brain.CurrentIntention.Speed += ARS.Remap(Pressure, 0f, 100f, -2f, 2f, true);

            // Yield: cap throttle to 0.5 to stay behind
            if (ActiveManeuver.Type == ManeuverType.Yield && ActiveManeuver.Target != null)
            {
                Control.MaxThrottle = Math.Min(Control.MaxThrottle, 0.5f);
            }


            // TODO: avoidance source — distance-based scalar in [-1, +1] (5m → 0m, +1 → -1).
            // Lowers _accelerationCap so the AI lifts off / brakes proportionally as a rival closes.
            // Suppressed while yielding — the yield speed cap handles staying behind.
            if (ActiveManeuver.Type != ManeuverType.Yield)
            {
                Rival avoidThreat = Brain.Rivals.FirstOrDefault(r => r.RivalRacer != null && r.RelativePosition == RelativePos.Ahead);
                if (avoidThreat != null)
                {
                    float avoidScalar = ARS.Remap(avoidThreat.Distance, 0f, 5f, -1f, 1f, true);
                    _accelerationCap = Math.Min(_accelerationCap, avoidScalar);
                }
            }

            // Projection off-track source (speed-based). Only the OUTSIDE of the upcoming
            // corner matters: uses the closest track node to the projected 1s position as the
            // reference frame. When the projection passes the actual track edge, the speed cap
            // is pinned at most to the corner speed, then deducted incrementally — 1 m/s per
            // meter of overshoot, per second — so a single frame of air/off-track projection
            // only nudges the cap instead of slamming it. Skipped while steering sits inside
            // the ±2° deadzone — a straight-running car projecting wide is not a cornering concern.
            if (Brain.Corner != null && Math.Abs(Control.SteerDegrees) > ProjectionSteerDeadzoneDegrees)
            {
                float cornerDir = Math.Sign(Brain.Corner.Point.Angle);
                if (cornerDir != 0f)
                {
                    Vector3 projected = ProjectAhead();
                    TrackPoint projectedTrackPoint = ARS.TrackPoints.OrderBy(t => t.Position.DistanceTo2D(projected)).First();
                    float outsideOffset = cornerDir * ARS.SignedLaneOffset(projected, projectedTrackPoint.Position, projectedTrackPoint.Direction);
                    float distanceFromInside = outsideOffset + projectedTrackPoint.TrackHalfWidth;
                    float trackWidth = projectedTrackPoint.TrackHalfWidth * 2f;

                    // Overshoot past the actual track edge (no 75% gate — the edge is the limit).
                    float overshoot = distanceFromInside - trackWidth;
                    if (overshoot > 0f)
                    {
                        // The safe speed is whichever is lower (corner or route). The cap pins
                        // to that, and the floor is safe speed - 4 m/s — the car can slow down
                        // to 4 m/s below the safe speed, but no further. At the floor, coast only.
                        float safeSpeed = Math.Min(_cornerSpd, followTrackSpd);
                        float floor = safeSpeed - 4f;
                        _speedCap = Math.Min(_speedCap, safeSpeed);
                        // Incremental deduction: 1 m/s per meter of overshoot, per second.
                        _speedCap = Math.Max(_speedCap - overshoot * 1f * TickScale, floor);
                        // If the floor was hit, kill throttle entirely — coast.
                        if (_speedCap <= floor)
                            Control.MaxThrottle = 0f;
                    }
                }
            }

        }

        float ComputeRouteSpeed(float radius)
        {
            radius = Math.Max(radius, 0.1f);
            return (float)Math.Sqrt((VehicleData.CurrentMechanicalGrip * Handling.Gravity) * radius);
        }

        const float SlopeGripLossK = 3f;    // multiplier for slope grip loss: loss = k · θ²
        const float SlopeGripLossExp = 2f;   // exponent
        // Crest aggression (route speed only): how hard the AI carries speed over crests. 0..1.
        // 0   = safe, no lift-off (cut bottoms out exactly at the 1g liftoff threshold)
        // 0.5 = a little lift-off (cut bottoms out beyond the liftoff threshold)
        // 1   = careless blow-through (no crest correction, keeps speed, gets airborne)
        const float CrestAggression = 0.5f;

        float GetFollowPointSlopeAngle()
        {
            int followNode = (int)ARS.Clamp(CurrentTrackPoint.Node + (int)(Car.Velocity.Length() * RouteWindowStart), 0, ARS.TrackPoints.Count - 1);
            int aheadNode = (int)ARS.Clamp(followNode + 5, 0, ARS.TrackPoints.Count - 1);
            if (aheadNode == followNode) return 0f;

            Vector3 from = ARS.TrackPoints[followNode].Position;
            Vector3 to = ARS.TrackPoints[aheadNode].Position;
            float horizDist = Vector2.Distance(new Vector2(from.X, from.Y), new Vector2(to.X, to.Y));
            if (horizDist < 0.01f) return 0f;
            return Math.Abs((float)Math.Atan2(to.Z - from.Z, horizDist));
        }

        void TractionControl()
        {
            if (Control.Throttle <= 0.0f) return;

            float wheelspin = ARS.MaxWheelSlip(Car);
            bool lowGripOrLowGear = GroundGripMultiplier < 0.9f || Car.CurrentGear < 2;


            // float allowedWheelspin = ARS.Remap(Pressure, 20f, 100f, 0.2f,2f, true);

            float tcsValue = ARS.Remap(Math.Abs(wheelspin) - 4f, 0.1f, -0.1f, -1f, 1f, true) * 8;
            float change = tcsValue * TickScale;
            Control.TCSThrottle = ARS.Clamp(Control.TCSThrottle + change, 0.01f, 1);
        }
        void ConsiderManeuvers()
        {
            if (ControlledByPlayer) return;

            // Force-disable any maneuver that's been armed for >8s without firing
            if (ActiveManeuver.Type != ManeuverType.None && Game.GameTime - ActiveManeuver.LastEnabled > 8000)
            {
                ActiveManeuver.Type = ManeuverType.None;
                ActiveManeuver.Target = null;
            }

            // Divebomb cleanup: off as soon as we pass the apex we armed for.
            if (ActiveManeuver.Type == ManeuverType.DiveBomb && _divebombApexNode >= 0)
            {
                int passed = CurrentTrackPoint.Node - _divebombApexNode;
                if (!ARS._isPointToPoint && passed < 0) passed += ARS.TrackPoints.Count;
                if (passed >= 0)
                {
                    ActiveManeuver.Type = ManeuverType.None;
                    ActiveManeuver.Target = null;
                    _divebombApexNode = -1;
                }
            }

            // DefendLane cleanup: off once we pass the defended apex, or the target overtakes us
            // (flips to Ahead / leaves the grid / is no longer a rival we track).
            if (ActiveManeuver.Type == ManeuverType.DefendLane)
            {
                bool overtaken = ActiveManeuver.Target == null
                    || !ActiveManeuver.Target.Car.Exists()
                    || !Brain.Rivals.Any(r => r.RivalRacer == ActiveManeuver.Target && r.RelativePosition == RelativePos.Behind);

                int passed = CurrentTrackPoint.Node - _defendApexNode;
                if (!ARS._isPointToPoint && passed < 0) passed += ARS.TrackPoints.Count;

                if (overtaken || (_defendApexNode >= 0 && passed >= 0))
                {
                    ActiveManeuver.Type = ManeuverType.None;
                    ActiveManeuver.Target = null;
                    _defendApexNode = -1;
                }
            }

            // Nitrous: arm if nearby cars exist or position >= 3rd, and not recently used
            if (AiNitrousEnabled && ActiveManeuver.Type == ManeuverType.None && Game.GameTime - ActiveManeuver.LastEnabled > 20000)
            {
                bool hasNearbyCars = Brain.Rivals.Any(r => r.RivalRacer != null);
                if (hasNearbyCars || RacePosition >= 3)
                {
                    ActiveManeuver.Type = ManeuverType.Nitrous;
                    ActiveManeuver.LastEnabled = Game.GameTime;
                }
            }

            // Yield: arm if pressure is much lower than closest rival, within 5s of corner, in overlap
            if (ActiveManeuver.Type == ManeuverType.None && Brain.Corner != null)
            {
                Rival closestRival = Brain.Rivals.FirstOrDefault(r => r.RivalRacer != null);
                if (closestRival != null)
                {
                    float pressureDiff = closestRival.RivalRacer.Pressure - Pressure;
                    bool inOverlap = closestRival.RelativePosition == RelativePos.Left || closestRival.RelativePosition == RelativePos.Right;
                    float distToApex = Math.Abs(Brain.Corner.Point.Node - CurrentTrackPoint.Node);
                    float timeToApex = distToApex / Math.Max(Car.Velocity.Length(), 1f);

                    if (pressureDiff > 30f && inOverlap && timeToApex <= 5f
                        && closestRival.RivalRacer.Car.Velocity.Length() > Car.Velocity.Length())
                    {
                        ActiveManeuver.Type = ManeuverType.Yield;
                        ActiveManeuver.Target = closestRival.RivalRacer;
                        ActiveManeuver.LastEnabled = Game.GameTime;
                        UI.Notify("~y~" + Name + "~w~ yields to ~y~" + closestRival.RivalRacer.Name);
                    }
                }
            }

            // Divebomb: pressure high, a rival ahead we'd reach the corner within 1s of —
            // commit to their inside and out-brake them at the apex.
            if (ActiveManeuver.Type == ManeuverType.None && Brain.Corner != null)
            {
                int apexNode = Brain.Corner.Point.Node;
                float myTimeToApex = ForwardNodeDistance(apexNode) / Math.Max(Car.Velocity.Length(), 1f);

                Rival diveTarget = Brain.Rivals.FirstOrDefault(r =>
                    r.RivalRacer != null
                    && r.RelativePosition == RelativePos.Ahead
                    && r.RivalRacer.ActiveManeuver.Type != ManeuverType.DiveBomb
                    && r.RivalRacer.ActiveManeuver.Type != ManeuverType.DefendLane
                    && Math.Abs(r.RivalRacer.ForwardNodeDistance(apexNode) / Math.Max(r.RivalRacer.Car.Velocity.Length(), 1f) - myTimeToApex) <= 1f);

                if (diveTarget != null && Pressure > 30f && myTimeToApex <= 8f)
                {
                    ActiveManeuver.Type = ManeuverType.DiveBomb;
                    ActiveManeuver.Target = diveTarget.RivalRacer;
                    ActiveManeuver.LastEnabled = Game.GameTime;
                    _divebombApexNode = apexNode;
                    UI.Notify("~y~" + Name + "~w~ divebombs ~y~" + diveTarget.RivalRacer.Name);
                }
            }

            // DefendLane: a rival behind would reach the corner alongside us (their time to apex
            // is equal or lower than ours) — cover the corner's inside so they can't dive
            // underneath. They're forced to take the corner on our outside.
            if (ActiveManeuver.Type == ManeuverType.None && Brain.Corner != null)
            {
                int apexNode = Brain.Corner.Point.Node;
                float myTimeToApex = ForwardNodeDistance(apexNode) / Math.Max(Car.Velocity.Length(), 1f);

                Rival defenderTarget = Brain.Rivals.FirstOrDefault(r =>
                    r.RivalRacer != null
                    && r.RelativePosition == RelativePos.Behind
                    && r.RivalRacer.ActiveManeuver.Type != ManeuverType.DiveBomb
                    && r.RivalRacer.ActiveManeuver.Type != ManeuverType.DefendLane
                    && r.RivalRacer.ForwardNodeDistance(apexNode) / Math.Max(r.RivalRacer.Car.Velocity.Length(), 1f) <= myTimeToApex);

                if (defenderTarget != null && myTimeToApex <= 6f)
                {
                    ActiveManeuver.Type = ManeuverType.DefendLane;
                    ActiveManeuver.Target = defenderTarget.RivalRacer;
                    ActiveManeuver.LastEnabled = Game.GameTime;
                    _defendApexNode = apexNode;
                    UI.Notify("~y~" + Name + "~w~ defends the inside from ~y~" + defenderTarget.RivalRacer.Name);
                }
            }
        }

        int ForwardNodeDistance(int targetNode)
        {
            int fwd = targetNode - CurrentTrackPoint.Node;
            if (!ARS._isPointToPoint && fwd < 0) fwd += ARS.TrackPoints.Count;
            return fwd;
        }

        int BehindNodeDistance(int targetNode)
        {
            int behind = CurrentTrackPoint.Node - targetNode;
            if (!ARS._isPointToPoint && behind < 0) behind += ARS.TrackPoints.Count;
            return behind;
        }

        void UpdateNitrous()
        {
            if (ControlledByPlayer || !AiNitrousEnabled) return;

            // Apply power boost while nitrous is active
            if (Game.GameTime < _nitrousActiveUntil)
            {
                Function.Call((Hash)CheatPowerIncreaseHash, Car, NitrousPowerMultiplier);
                return;
            }
            if (_nitrousActiveUntil > 0)
            {
                StopNitrous();
                ActiveManeuver.Type = ManeuverType.None;
            }

            // Check if nitrous maneuver is armed and conditions are right to fire
            if (ActiveManeuver.Type != ManeuverType.Nitrous) return;

            // Stability check: steering < 5°, rotation < 30°/s
            if (Math.Abs(Control.SteerDegrees) >= 5f) return;
            if (Math.Abs(VehicleData.YawRotationPerSecondDegrees) >= 30f) return;

            // Corner check: > 8s away
            if (Brain.Corner != null)
            {
                float distanceToEntrance = (Brain.Corner.Point.Node - Brain.Corner.Point.LengthStart) - CurrentTrackPoint.Node;
                float timeToEntrance = distanceToEntrance * 2f / Math.Max(Car.Velocity.Length(), 1f);
                if (timeToEntrance <= NitrousCornerLookaheadSeconds) return;
            }

            // Target check: closest rival must be faster
            Rival closestRival = Brain.Rivals.FirstOrDefault(r => r.RivalRacer != null);
            if (closestRival == null) return;
            if (closestRival.RivalRacer.Car.Velocity.Length() <= Car.Velocity.Length()) return;

            // All conditions met — fire!
            StartNitrous();
            ActiveManeuver.Type = ManeuverType.None;
            ActiveManeuver.LastEnabled = Game.GameTime;
        }

        void StartNitrous()
        {
            Function.Call(Hash.REQUEST_NAMED_PTFX_ASSET, NitrousPtfxAsset);
            Function.Call((Hash)FullyChargeNitrousHash, Car);
            Function.Call((Hash)OverrideNitrousLevelHash, Car, true, 1.0f, 50.0f, 100.0f, false);
            _nitrousActiveUntil = Game.GameTime + NitrousDurationMs;
        }

        void StopNitrous()
        {
            Function.Call((Hash)CheatPowerIncreaseHash, Car, 1.0f);
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
            VehicleData.LocalGs = (Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true) - VehicleData.SpeedVectorLocal) / Game.LastFrameTime;
            Vector3 cSpeed = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, false);

            VehicleData.AccelerationVector.Add((cSpeed - _lastSpeed) / Game.LastFrameTime);

            if (VehicleData.AccelerationVector.Count > 10) VehicleData.AccelerationVector.RemoveAt(0);

            _lastSpeed = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, false);
            VehicleData.SpeedVectorGlobal = cSpeed;
            VehicleData.SpeedVectorLocal = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true);
            Brain.CurrentPerception.SpeedVector = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, Car, true);


            if (ARS._debugToggles[Options.ShowInputs] && !Driver.IsPlayer)
            {
                float combinedInput = ARS.Clamp(Control.Throttle - Control.Brake, -1f, 1f);
                if (_trailSamples.Count == 0)
                {
                    _trailSamples.Add(new TrailSample(Car.Position, combinedInput));
                }
                else if (Car.Position.DistanceTo(_trailSamples[_trailSamples.Count - 1].Position) > 1f)
                {
                    _trailSamples.Add(new TrailSample(Car.Position, combinedInput));
                }
            }

            while (_trailSamples.Count > 10) _trailSamples.RemoveAt(0);
        }


        // 1-second kinematic projection of the car in world space.
        // Uses pos + v*t + 0.5*a*t² with t=1, where a is the running average of
        // the last ~10 frames' world-frame accelerations (m/s²) from UpdateTickData.
        public Vector3 ProjectAhead()
        {
            Vector3 avgAccel = VehicleData.AccelerationVector.Aggregate(
                new Vector3(0, 0, 0), (s, v) => s + v)
                / (float)VehicleData.AccelerationVector.Count;
            return Car.Position + Car.Velocity + 0.5f * avgAccel;
        }



        public void ProcessTick()
        {
            UpdateTickData();
            DrawRacerDebug();

            if (!Driver.IsPlayer)
            {
                ApplyInputs();
            }
        }
        public void RunTimedCore()
        {
            UpdateTrackPosition();
            UpdateSlideAndBoundingBox();
            UpdatePerceivedGrip();
            UpdateCornerValidity();

            ProcessAI();
            if (Driver.IsPlayer && ARS._settingsFile.GetValue("CATCHUP", "OnlyBehindPlayer", true)) ARS.CatchupPosition = RacePosition;

            _lastCoreTick = Game.GameTime;
        }



        void UpdateSlideAndBoundingBox()
        {
            VehicleData.BoundingBox = ARS.SlidingBoundingBoxWidth(Car);
            VehicleData.SlideAngle = (float)Math.Round(Vector3.SignedAngle(Car.Velocity.Normalized, Car.ForwardVector, Car.UpVector), 3);
        }
        void UpdateCornerValidity()
        {
            ARS.FindNextCorner(this);
        }







        void UpdatePassengerSeat()
        {
            if (Driver.IsPlayer) return;

            float myHalfLen = Math.Abs(Car.Model.GetDimensions().Y) * 0.5f;

            bool shouldPassengerize = false;
            foreach (Rival r in Brain.Rivals)
            {
                if (r.RivalRacer == null) continue;
                float rivalHalfLen = Math.Abs(r.RivalRacer.Car.Model.GetDimensions().Y) * 0.5f;
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

        public void ApplyInputs()
        {


            if (Driver.IsSittingInVehicle(Car) && !Driver.IsPlayer)
            {
                UpdatePassengerSeat();

                if (Control.HandBrakeTime > Game.GameTime) Car.HandbrakeOn = true; else Car.HandbrakeOn = false;

                ARS.SetThrottle(Car, ARS.Clamp(Control.Throttle, -1, 1));
                ARS.SetBrakes(Car, Control.Brake);
                ARS.SetSteerAngle(Car, Control.SteerInput);

            }
            else
            {
                ARS.SetThrottle(Car, 0f);
                ARS.SetBrakes(Car, 0f);
                ARS.SetSteerInput(Car, 0f);
            }
        }
        void DrawRacerDebug()
        {
            bool showAggro = ARS._debugToggles[Options.ShowAggro];
            bool showInputs = ARS._debugToggles[Options.ShowInputs];
            bool showTrack = ARS._debugToggles[Options.ShowTrackAnalysis];
            bool showPhysics = ARS._debugToggles[Options.ShowPhysics];
            bool showAny = showAggro || showInputs || showTrack || showPhysics;
            if (!showAny) return;

            if ((Car.Position - Game.Player.Character.Position).Length() > 50) return;


            if (showTrack && Driver.IsPlayer && Lap >= ARS._settingsFile.GetValue<int>("GENERAL_SETTINGS", "Laps", 5) && CanRegisterNewLap)
            {
                World.DrawMarker(MarkerType.CheckeredFlagRect, ARS.TrackPoints.First().Position + new Vector3(0, 0, 5f), ARS.TrackPoints.First().Direction, new Vector3(0, 0, 0), new Vector3(5f, 5f, 5f), Color.White);
            }


            if (showPhysics)
            {


                World.DrawMarker(MarkerType.DebugSphere, Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.1f, 0.1f, 0.1f), Color.Green, false, false, 0, false, "", "", false);


                Vector3 avgGs = VehicleData.AccelerationVector.Aggregate(new Vector3(0, 0, 0), (s, v) => s + v) / (float)VehicleData.AccelerationVector.Count;
                avgGs.Z = 0f;

                float colorPercent = ARS.Remap(avgGs.Length() / 9.8f, 0, VehicleData.CurrentMechanicalGrip, 0, 100, true);
                Color gColor = ARS.GradientAtoBtoC(Color.White, Color.Yellow, Color.Red, colorPercent);

                World.DrawMarker(MarkerType.DebugSphere, Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)) + (avgGs / 9.8f), Vector3.Zero, new Vector3(0, 0, 0), new Vector3(0.15f, 0.15f, 0.15f), gColor, false, false, 0, false, "", "", false);
                ARS.DrawLine(Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)) + (avgGs / 9.8f), Car.Position + new Vector3(0, 0, (Car.Model.GetDimensions().Z * 0.6f)), gColor);


                Vector3 maxValues = new Vector3(VehicleData.CurrentMechanicalGrip * 9.8f, VehicleData.CurrentMechanicalGrip * 9.8f, VehicleData.CurrentMechanicalGrip * 9.8f);
                Vector3 max = Vector3.Clamp(avgGs, -maxValues, maxValues);



                Vector3 source = Car.Position + new Vector3(0, 0, 0.5f + (Car.Model.GetDimensions().Z * 0.6f));
                ARS.DrawText(source, "~b~" + Math.Round(ARS.MpsToMph(Car.Velocity.Length())).ToString() + "~w~mph~n~~y~" + (avgGs.Length() / 9.8f).ToString("0.0") + " Gs", Color.White, 0.5f);


            }

            if (!Driver.IsPlayer)
            {

                if (!Car.IsInRangeOf(Game.Player.Character.Position, 500)) return;
                if (showInputs)
                {
                    DrawInputTrails();

                    // Speed readout — everything that shapes this moment's speed, in mph.
                    // Line 1: actual speed vs final intended speed.
                    ARS.DrawText(Car.Position + new Vector3(0, 0, 2f),
                        "~w~SPD ~g~" + ARS.MpsToMph(Car.Velocity.Length()).ToString("0") + "~w~/~b~" + ARS.MpsToMph(Brain.CurrentIntention.Speed).ToString("0") + "mph",
                        Color.White, 0.4f);
                    // Line 2: braking-plan corner speed and route curvature speed.
                    ARS.DrawText(Car.Position + new Vector3(0, 0, 2.4f),
                        "~w~corn ~o~" + ARS.MpsToMph(_debugCornerSpd).ToString("0") + "~w~ rte ~c~" + ARS.MpsToMph(_debugFollowTrackSpd).ToString("0"),
                        Color.White, 0.4f);
                    // Line 3: the two caps the speed loop clamps against.
                    ARS.DrawText(Car.Position + new Vector3(0, 0, 2.8f),
                        "~w~spdCap ~p~" + ARS.MpsToMph(_speedCap).ToString("0") + "~w~ accCap ~r~" + _accelerationCap.ToString("0.00"),
                        Color.White, 0.4f);
                }
                if (showTrack)
                {
                    Vector3 trackCenter = CurrentTrackPoint.Position;
                    Vector3 trackRight = Vector3.Cross(CurrentTrackPoint.Direction, Vector3.WorldUp);
                    Vector3 leftWallPos = trackCenter + trackRight * _avoidLeftWall;
                    Vector3 rightWallPos = trackCenter + trackRight * _avoidRightWall;
                    Vector3 up = new Vector3(0, 0, 0.1f);
                    float wallHeight = 1.5f;
                    Vector3 leftTop = leftWallPos + up + new Vector3(0, 0, wallHeight);
                    Vector3 rightTop = rightWallPos + up + new Vector3(0, 0, wallHeight);
                    ARS.DrawLine(leftWallPos + up, leftTop, Color.Blue);
                    ARS.DrawLine(rightWallPos + up, rightTop, Color.Red);
                    ARS.DrawLine(leftTop, rightTop, Color.White);

                    // Lane aim: 10 white spheres from the car to the final clamped lane target
                    // at the steer-ref distance. Shows where the car is steering (after all lane tweaks).
                    if (LookAheads.TryGetValue(LookAhead.SteerRef, out TrackPoint laneRef) && laneRef != null)
                    {
                        Vector3 laneRight = Vector3.Cross(laneRef.Direction, Vector3.WorldUp).Normalized;
                        Vector3 laneAim = laneRef.Position + laneRight * _targetLane;
                        for (int s = 1; s <= 10; s++)
                        {
                            float t = s / 10f;
                            Vector3 spherePos = Car.Position + (laneAim - Car.Position) * t;
                            World.DrawMarker(MarkerType.DebugSphere, spherePos, Vector3.Zero, Vector3.Zero, new Vector3(0.2f, 0.2f, 0.2f), Color.White, false, false, 0, false, "", "", false);
                        }
                    }

                    // Track-ahead high-speed curve radius (0.5s→1.0s window) — the value the pursuit gain reads.
                    ARS.DrawText(Car.Position + new Vector3(0, 0, 2.4f), "~o~R: ~w~" + Brain.CurrentPerception.HighSpeedCurveRadius.ToString("0"), Color.White, 0.4f);
                }

                if (showAggro)
                {
                    Color pressureColor = ARS.GradientAtoBtoC(Color.Green, Color.Yellow, Color.Red, Pressure);
                    World.DrawMarker(MarkerType.ChevronUpx1, Car.Position + new Vector3(0f, 0f, 1.5f), Vector3.Zero, Vector3.Zero, new Vector3(0.5f, 0.5f, -0.5f), pressureColor, false, true, 0, false, "", "", false);

                    ARS.DrawText(Car.Position + new Vector3(0, 0, 2f), ((int)Pressure).ToString(), Color.White, 0.4f);

                    // Projection debug — white line + sphere at the 1-second projected position.
                    Vector3 projected = ProjectAhead();
                    Vector3 lineStart = Car.Position + new Vector3(0, 0, Car.Model.GetDimensions().Z * 0.6f);
                    ARS.DrawLine(lineStart, projected, Color.White);

                    // Find the track point closest to the projected position and check if
                    // it falls inside the safe bound. If the projection is off-track, the
                    // car is going to leave the road in ~1 second.
                    TrackPoint projectedTrackPoint = ARS.TrackPoints.OrderBy(t => t.Position.DistanceTo2D(projected)).First();
                    float projectedLateralOffset = Math.Abs(ARS.SignedLaneOffset(projected, projectedTrackPoint.Position, projectedTrackPoint.Direction));
                    float projectedSafeBound = projectedTrackPoint.TrackHalfWidth - VehicleData.BoundingBox * 0.5f;
                    bool willGoOffTrack = projectedLateralOffset > projectedSafeBound;

                    Color projectionColor = willGoOffTrack ? Color.Red : Color.White;
                    World.DrawMarker(MarkerType.DebugSphere, projected, Vector3.Zero, Vector3.Zero, new Vector3(0.5f, 0.5f, 0.5f), projectionColor, false, false, 0, false, "", "", false);

                    // Draw the two track edges at the projected progress so the comparison is visible.
                    Vector3 trackRight = Vector3.Cross(projectedTrackPoint.Direction, Vector3.WorldUp).Normalized;
                    Vector3 leftEdge = projectedTrackPoint.Position - trackRight * projectedTrackPoint.TrackHalfWidth;
                    Vector3 rightEdge = projectedTrackPoint.Position + trackRight * projectedTrackPoint.TrackHalfWidth;
                    ARS.DrawLine(leftEdge, rightEdge, willGoOffTrack ? Color.Red : Color.Green);
                }

                if (showTrack)
                {
                    DrawCornerChevrons(Brain.Corner);
                }

            }
            else
            {
                if (showTrack)
                {
                    DrawCornerChevrons(Brain.Corner);
                }
            }
        }

        void DrawCornerChevrons(Corner corner)
        {
            if (corner == null || Lap <= 0) return;
            CornerPoint c = corner.Point;
            int startNode = (int)ARS.Clamp(c.Node - c.LengthStart, 0, ARS.TrackPoints.Count - 1);
            int endNode = (int)ARS.Clamp(c.Node + c.LengthEnd, 0, ARS.TrackPoints.Count - 1);

            Vector3 chevScale = new Vector3(ARS.TrackPoints[c.Node].TrackHalfWidth * 2.5f, 5, 5);

            World.DrawMarker(MarkerType.ChevronUpx1, ARS.TrackPoints[startNode].Position, ARS.TrackPoints[startNode].Direction, new Vector3(90, 0, 0), chevScale, Color.Green);
            World.DrawMarker(MarkerType.ChevronUpx1, ARS.TrackPoints[c.Node].Position, ARS.TrackPoints[c.Node].Direction, new Vector3(90, 0, 0), chevScale, Color.Green);
            World.DrawMarker(MarkerType.ChevronUpx1, ARS.TrackPoints[endNode].Position, ARS.TrackPoints[endNode].Direction, new Vector3(90, 0, 0), chevScale, Color.Green);

            // Text ~2m above the apex: corner radius + apex speed (single block, ~n~ newline)
            Vector3 apexPos = ARS.TrackPoints[c.Node].Position;
            ARS.DrawText(apexPos + new Vector3(0f, 0f, 2f),
                "R ~b~" + c.Radius.ToString("0.0") + "~n~~o~SPD " + corner.Speed.ToString("0.0"),
                Color.Blue, 0.4f);
        }

        void DrawInputTrails()
        {
            if (_trailSamples.Count < 2) return;

            for (int i = 1; i < _trailSamples.Count; i++)
            {
                TrailSample fromSample = _trailSamples[i - 1];
                TrailSample toSample = _trailSamples[i];
                Vector3 from = fromSample.Position;
                Vector3 to = toSample.Position;
                Vector3 segment = to - from;
                if (segment.Length() < 0.05f) continue;

                float inputFrom = fromSample.CombinedInput;
                float inputTo = toSample.CombinedInput;
                 Vector3 point = to;
                Vector3 away = segment.Normalized;
                float dimension = Car.Model.GetDimensions().Y + 1f;
                Vector3 chevronScale = new Vector3(dimension / 2f, dimension / 4f, -(dimension / 2f));
                float value = ARS.Clamp((inputFrom + inputTo) * 0.5f, -1f, 1f);
                Color baseColor;
                if (value >= 0f)
                {

                    baseColor = ARS.GradientAtoBtoC(Color.White, Color.GreenYellow, Color.Green, value * 100f);
                }
                else
                {

                    float brake = -value;
                    baseColor = ARS.GradientAtoBtoC(Color.White, Color.Orange, Color.Red, brake * 100f);
                }
                Color finalColor = Color.FromArgb(255, baseColor.R, baseColor.G, baseColor.B);

                World.DrawMarker(MarkerType.ChevronUpx1, point, -away, new Vector3(90, 0, 0), chevronScale, finalColor, false, false, 0, false, "", "", false);
            }
        }




        float OutOfTrackDistance()
        {
            return (Math.Abs(Brain.CurrentPerception.DeviationFromCenter) + (VehicleData.BoundingBox / 2)) - CurrentTrackPoint.TrackHalfWidth;
        }

        void UpdateRivalInfo()
        {
            foreach (Rival r in Brain.Rivals) r.Update(this);
        }




        public void UpdateTrackPosition()
        {


            int refTrackpoint = (int)ARS.Clamp(CurrentTrackPoint.Node, 0, ARS.TrackPoints.Count - 1);

            List<TrackPoint> points = new List<TrackPoint>();
            int lastNode = ARS.TrackPoints.Count - 1;
            int firstCandidate = Math.Max(refTrackpoint - 6, 0);
            int lastCandidate = Math.Min(refTrackpoint + 6, lastNode);
            for (int i = firstCandidate; i <= lastCandidate; i++)
            {
                points.Add(ARS.TrackPoints[i]);
            }

            bool hasCrossedStartLine = !ARS._isPointToPoint
                && CanRegisterNewLap
                && refTrackpoint >= lastNode - 6
                && Vector3.Dot(Car.Position - ARS.TrackPoints[0].Position, ARS.TrackPoints[0].Direction) > 0f;
            if (hasCrossedStartLine) points.Add(ARS.TrackPoints[0]);

            TrackPoint closestPoint = points[0];
            float closestDistance = closestPoint.Position.DistanceTo(Car.Position);
            foreach (TrackPoint point in points)
            {
                float distance = point.Position.DistanceTo(Car.Position);
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
                    float distance = point.Position.DistanceTo(Car.Position);
                    if (distance < closestDistance)
                    {
                        closestPoint = point;
                        closestDistance = distance;
                    }
                }
            }

            CurrentTrackPoint = closestPoint;
            Brain.CurrentPerception.DeviationFromCenter = ARS.SignedLaneOffset(Car.Position, CurrentTrackPoint.Position, CurrentTrackPoint.Direction);

            LookAheads.Clear();
            float speed = Car.Velocity.Length();

            int steerRef = (int)ARS.Clamp((int)(speed / Math.Max(VehicleData.CurrentMechanicalGrip, 0.1f)), (int)(CurrentTrackPoint.TrackHalfWidth * 2f), 500);
            int quarterSec = (int)(speed * 0.25f);
            int halfSec = (int)(speed * 0.5f);
            int threeQuarterSec = (int)(speed * 0.75f);
            int oneSec = (int)(speed);
            int oneHalfSec = (int)(speed * 1.5f);
            int twoSec = (int)(speed * 2f);

            TrackPoint ResolveLookAhead(int offset)
            {
                int node = CurrentTrackPoint.Node + offset;
                if (ARS._isPointToPoint) return ARS.TrackPoints[Math.Min(node, lastNode)];
                return ARS.TrackPoints[node % ARS.TrackPoints.Count];
            }

            var lookAheadOffsets = new (LookAhead key, int offset)[]
            {
                (LookAhead.SteerRef, steerRef),
                (LookAhead.QuarterSec, quarterSec),
                (LookAhead.HalfSec, halfSec),
                (LookAhead.ThreeQuarterSec, threeQuarterSec),
                (LookAhead.OneSec, oneSec),
                (LookAhead.OneHalfSec, oneHalfSec),
                (LookAhead.TwoSec, twoSec),
            };
            foreach (var (key, offset) in lookAheadOffsets)
                LookAheads.Add(key, ResolveLookAhead(offset));




            if (CanRegisterNewLap)
            {
                if (hasCrossedStartLine || (ARS._isPointToPoint && ARS.GetPercent(CurrentTrackPoint.Node, ARS.TrackPoints.Count) > 99 && ARS.EntityRelativeOffset(Car, ARS.TrackPoints.Last().Position).Y < 0f))
                {
                    CanRegisterNewLap = false;
                    _hasLeftLapArmNode = false;
                    Lap++;
                    if (Lap > ARS._settingsFile.GetValue("GENERAL_SETTINGS", "Laps", 5))
                    {
                        if (Car.CurrentBlip != null) Car.CurrentBlip.Color = BlipColor.Green;
                    }

                    if (Lap > 1)
                    {
                        TimeSpan lapTime = ARS.ParseToTimeSpan(Game.GameTime - LapStartTime);
                        UI.Notify(Name + "'s laptime: ~b~" + lapTime.ToString("m':'ss'.'f"));
                        LapTimes.Add(lapTime);
                        LapStartTime = Game.GameTime;
                    }
                }
            }
            else if (BaseBehavior == RacerBaseBehavior.Race)
            {
                if (CurrentTrackPoint.Node < 100) _hasLeftLapArmNode = true;
                else if (_hasLeftLapArmNode) CanRegisterNewLap = true;
            }






            Brain.CurrentPerception.CurveRadiusToFollowPoint = ComputeRouteRadius(RouteWindowStart, RouteWindowSize);
            // High-speed lane radius: short 0.5s→1.0s window so the inside-edge pursuit gain
            // reacts to the imminent corner, not the long approach.
            Brain.CurrentPerception.HighSpeedCurveRadius = ComputeRouteRadius(RouteWindowStart, 0.5f);
            Brain.CurrentPerception.CurveRadiusAfterFollowPoint = ComputeRouteRadius(RouteWindowStart + RouteWindowSize, RouteWindowSize); // NEVER USED — kept for future "two conflicting corners" work
        }

        float ComputeRouteRadius(float windowStart, float windowSize)
        {
            float routeSpeed = Car.Velocity.Length();
            int count = ARS.TrackPoints.Count;
            int startOffset = CurrentTrackPoint.Node + (int)(routeSpeed * windowStart);
            int endOffset = CurrentTrackPoint.Node + (int)(routeSpeed * (windowStart + windowSize));
            int routeStartNode, routeEndNode, routeMidNode;
            if (ARS._isPointToPoint)
            {
                routeStartNode = (int)ARS.Clamp(startOffset, 0, count - 1);
                routeEndNode = (int)ARS.Clamp(endOffset, 0, count - 1);
                routeMidNode = (int)((routeStartNode + routeEndNode) * 0.5f);
            }
            else
            {
                routeStartNode = ((startOffset % count) + count) % count;
                routeEndNode = ((endOffset % count) + count) % count;
                int span = ((endOffset - startOffset) % count + count) % count;
                if (span == 0) return 999f;
                routeMidNode = (((startOffset + span / 2) % count) + count) % count;
            }

            if (routeEndNode == routeStartNode) return 999f;
            return ARS.Circumradius3D(
                ARS.TrackPoints[routeStartNode].Position,
                ARS.TrackPoints[routeEndNode].Position,
                ARS.TrackPoints[routeMidNode].Position) / 2f;
        }




        void ProcessTimedAI()
        {
            if (Brain.Corner == null) return;
            if (_halfSecondTick < Game.GameTime)
            {
                _halfSecondTick = Game.GameTime + 500 + (int)ARS.Remap(Car.Velocity.Length(), 0, 100, -250, 250, true);
            }



            if (_oneSecondTick < Game.GameTime)
            {
                _oneSecondTick = Game.GameTime + 1000;

                if (!ControlledByPlayer)
                {
                    if (BaseBehavior == RacerBaseBehavior.Race && ARS._racers.Count >= 1)
                    {
                        UpdateRivals();
                        ConsiderManeuvers();
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

                if (BaseBehavior == RacerBaseBehavior.Race && Math.Abs(VehicleData.SlideAngle) < 0.5f && Math.Abs(Control.Throttle) > 0.9f) Function.Call((Hash)0x81E1552E35DC3839, Car, true);

                if (Function.Call<bool>((Hash)0x3D34E80EED4AE3BE, Car) && Control.Brake > 0.1f) Function.Call((Hash)0x81E1552E35DC3839, Car, false);


                if (ARS.DevSettingsFile.GetValue<int>("RACERS", "AIRacerAutofix", 1) == 2 && Function.Call<bool>(Hash._IS_VEHICLE_DAMAGED, Car))
                {
                    Car.Repair();
                }
            }
        }




        public void ProcessAI()
        {
            ProcessTimedAI();
            UpdatePressure();

            if (BaseBehavior == RacerBaseBehavior.GridWait && Control.HandBrakeTime < Game.GameTime) Control.HandBrakeTime = Game.GameTime + (100 * ARS.GetRandomInt(2, 6));

            if (!ControlledByPlayer)
            {
                UpdateRivalInfo();

                // Re-rise the caps; each concern source below pulls them down via Math.Min.
                _accelerationCap = Math.Min(1f, _accelerationCap + 0.33f * TickScale);
                _speedCap = Math.Min(999f, _speedCap + SpeedCapRiseRate * TickScale);

                ComputeTargetSpeed();
                ComputeSteering();
                ApplySteerLimits();

                ConvertSpeedToPedals();
                TranslateSteerToInput();

                UpdateStuckCheck();
                UpdateStuckRecovery();
                
                TractionControl();
                ApplyStuckRecoveryOverride();

                UpdateNitrous();
                UpdateYield();

            }
            else
            {
                IsStuckByThrottle = false;
                _lastStuckGameTime = 0;
                _isRecoveringFromStuck = false;
                _stuckRecoveryEndTime = 0;
            }
        }

        void UpdatePressure()
        {
            float closestDistance = float.MaxValue;
            if (BaseBehavior == RacerBaseBehavior.Race)
            {
                foreach (Racer racer in ARS._racers)
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
                // Map 100m→0, 20m→Aggression
                float t = ARS.Clamp((PressureProximityRange - closestDistance) / (PressureProximityRange - 20f), 0f, 1f);
                targetPressure = Aggression * t;
            }

            if (targetPressure > Pressure)
                Pressure = Math.Min(Pressure + PressureRisePerSecond * TickScale, targetPressure);
            else
                Pressure = Math.Max(Pressure - PressureFallPerSecond * TickScale, targetPressure);

            Pressure = ARS.Clamp(Pressure, 0f, PressureRange);

            // Pressure-driven lookahead: calm racers plan off the road at the car
            // (window start 0), pressured racers plan a full second ahead (start 1).
            // RouteWindowStart = Pressure / PressureRange;
            RouteWindowStart = 0.5f;
        }
 
 
        void UpdateStuckCheck()
        {
            if (ARS.MpsToMph(Car.Velocity.Length()) > 10f && Math.Abs(Brain.CurrentPerception.DeviationFromCenter) < CurrentTrackPoint.TrackHalfWidth)
            {
                _stuckRecoveryAttempts = 0;
            }

            if (_isRecoveringFromStuck)
            {
                IsStuckByThrottle = false;
                _lastStuckGameTime = 0;
                return;
            }

            if (BaseBehavior != RacerBaseBehavior.Race || !Driver.IsSittingInVehicle(Car))
            {
                IsStuckByThrottle = false;
                _lastStuckGameTime = 0;
                return;
            }

            bool lowLongitudinalGs = Math.Abs(VehicleData.LongitudinalGs) < 0.25f;
            bool stuckCondition = lowLongitudinalGs && ARS.MpsToMph(Car.Velocity.Length()) < 5f;

            if (!stuckCondition)
            {
                IsStuckByThrottle = false;
                _lastStuckGameTime = 0;
                return;
            }

            if (_lastStuckGameTime == 0)
            {
                _lastStuckGameTime = Game.GameTime;
            }

            bool stuckForLongEnough = (Game.GameTime - _lastStuckGameTime) >= StuckCheckTimeMs;
            IsStuckByThrottle = stuckForLongEnough;

            if (stuckForLongEnough && !_isRecoveringFromStuck)
            {
                _isRecoveringFromStuck = true;
                _stuckRecoveryAttempts++;
                HandleRecoveryAttemptEscalation();
                _stuckRecoveryEndTime = Game.GameTime + StuckRecoveryTimeMs;
                IsStuckByThrottle = false;
                _lastStuckGameTime = 0;
            }
        }

        void UpdateStuckRecovery()
        {
            if (BaseBehavior != RacerBaseBehavior.Race || !Driver.IsSittingInVehicle(Car))
            {
                _isRecoveringFromStuck = false;
                _stuckRecoveryEndTime = 0;
                return;
            }

            if (!_isRecoveringFromStuck && IsStuckByThrottle)
            {
                _isRecoveringFromStuck = true;
                _stuckRecoveryAttempts++;
                HandleRecoveryAttemptEscalation();
                _stuckRecoveryEndTime = Game.GameTime + StuckRecoveryTimeMs;
                IsStuckByThrottle = false;
            }

            if (!_isRecoveringFromStuck) return;

            if (Game.GameTime >= _stuckRecoveryEndTime)
            {
                _isRecoveringFromStuck = false;
                _stuckRecoveryEndTime = 0;
                _lastStuckGameTime = 0;
                return;
            }
        }

        void ApplyStuckRecoveryOverride()
        {
            if (!_isRecoveringFromStuck) return;

            if (Game.GameTime >= _stuckRecoveryEndTime)
            {
                _isRecoveringFromStuck = false;
                _stuckRecoveryEndTime = 0;
                _lastStuckGameTime = 0;
                return;
            }


            Control.SteerInput = 0f;
            Control.Throttle = -1f;
            Control.Brake = 0f;
        }

        void HandleRecoveryAttemptEscalation()
        {
            if (_stuckRecoveryAttempts < 2) return;

            Vector3 toTrack = CurrentTrackPoint.Position - Car.Position;
            if (toTrack.Length() < 0.01f) toTrack = Car.ForwardVector;

            Vector3 nudgedVelocity = toTrack.Normalized * ARS.MphToMps(30f);
            nudgedVelocity.Z += 30f;
            Car.Velocity = nudgedVelocity;

        }

        void UpdatePerceivedGrip()
        {


            float handlingGrip = Function.Call<float>((Hash)0xA132FB5370554DB0, Car) * (Handling.Gravity / 9.8f);
            handlingGrip = ARS.Clamp(handlingGrip, 0.1f, 5f);

            GroundGripMultiplier = ARS.WheelGripMultipliers(Car).Average();

            VehicleData.BaseMechanicalGrip = handlingGrip;
            VehicleData.CurrentMechanicalGrip = ((VehicleData.BaseMechanicalGrip) * GroundGripMultiplier);

        
            if (Math.Abs(Brain.CurrentPerception.DeviationFromCenter) < CurrentTrackPoint.TrackHalfWidth && RacePosition <= 2 && !ARS._terrainGripMultipliers.ContainsKey(CurrentTrackPoint.Node))
            {
                ARS._terrainGripMultipliers.Add(CurrentTrackPoint.Node, GroundGripMultiplier);
            }

            float zSpeedDegreesFromHoriz = (Math.Abs(VehicleData.SpeedVectorLocal.Normalized.Z) * 90);


            if (zSpeedDegreesFromHoriz > 5f)
            {
                if (VehicleData.AvgGroundStability >= 0.1f) VehicleData.AvgGroundStability -= zSpeedDegreesFromHoriz * TickScale * 0.1f;
            }
            else if (VehicleData.AvgGroundStability < 1f) VehicleData.AvgGroundStability += 1f * TickScale;

            if (VehicleData.AvgGroundStability > 1.0f) VehicleData.AvgGroundStability = 1f;

            VehicleData.YawRotationPerSecondDegrees = ARS.RadToDeg(Function.Call<Vector3>(Hash.GET_ENTITY_ROTATION_VELOCITY, Car).Z);            
        }
        public void UpdateRivals()
        {
            List<Racer> candidates = new List<Racer>();
            foreach (Racer r in ARS._racers)
            {
                if (r.Car.Handle != Car.Handle && r.Car.Position.DistanceTo(Car.Position) < 200f)
                {
                    candidates.Add(r);
                }
            }

            foreach (Rival r in Brain.Rivals) r.RivalRacer = null;
            if (candidates.Count > 0)
            {
                Vector3 hoodPos = Car.Position + Car.ForwardVector;
                candidates.Sort((a, b) => Vector3.Distance(a.Car.Position, hoodPos).CompareTo(Vector3.Distance(b.Car.Position, hoodPos)));
                for (int i = 0; i < Brain.Rivals.Count - 1; i++)
                {
                    if (i == candidates.Count) break;
                    Brain.Rivals[i].RivalRacer = candidates[i];
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
            }
            else
            {
                if (Car.CurrentBlip != null && Car.CurrentBlip.Exists()) Car.CurrentBlip.Remove();
                Car.Delete();
            }
        }
    }
}


