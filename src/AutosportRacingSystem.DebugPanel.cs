// Developer panel: the screen-space bars for pedal, speed, margins and the Gs floor.
using GTA;
using GTA.Math;
using GTA.Native;
using System;
using System.Drawing;
using System.Globalization;

namespace ARS
{
    public partial class ARS
    {
        const float DebugPanelLeft = 0.35f;
        const float DebugPanelRight = 0.65f;
        const float DebugPanelTop = 0f;
        const float DebugPanelBottom = 0.3f;
        const int BarsPerPedal = 3;
        const float PanelBarGap = 0.004f;
        const float SpeedDeltaFullSpanMps = 30f;
        static readonly Color ThrottleAppliedColour = Color.LimeGreen;
        static readonly Color ThrottleBoundColour = Color.FromArgb(235, 210, 60);
        static readonly Color BrakeBarColour = Color.Red;
        static readonly Color SpeedMarginColour = Color.Orange;
        static readonly Color FollowedSpeedColour = Color.DodgerBlue;
        static readonly Color PanelLabelColour = Color.White;
        const float PanelLabelY = 0.272f;
        const float PanelLabelScale = 0.35f;
        const float PanelIntendedY = 0.008f;
        const float PanelIntendedScale = 0.30f;
        const float PanelRadiusY = 0.032f;
        const float PanelRadiusScale = 0.26f;
        const float SpeedSentinelMps = 998f;
        static readonly string[] PanelLabels = { "THR", "STB", "FLR", "APEX", "ROUT", "BRAKE" };

        public static void DrawScreenRect(float left, float top, float right, float bottom, Color colour)
        {

            // DRAW_RECT takes the centre and the full size, and arg 9 is bStereo: the engine takes 9, the docs list 8.
            Function.Call(Hash.DRAW_RECT, (left + right) * 0.5f, (top + bottom) * 0.5f, right - left, bottom - top, colour.R, colour.G, colour.B, colour.A, false);
        }

        public static void DrawDebugPanel(Racer racer)
        {
            DrawScreenRect(DebugPanelLeft, DebugPanelTop, DebugPanelRight, DebugPanelBottom, Color.Black);
            float barWidth = (DebugPanelRight - DebugPanelLeft - (2 * BarsPerPedal - 1) * PanelBarGap) / (2 * BarsPerPedal);
            float speed = racer != null && CanWeUse(racer.Car) ? ARS.GetForwardSpeed(racer.Car) : 0f;
            DrawPanelBars(racer, barWidth, speed);
            DrawPanelLabels(racer, barWidth);
        }

        static void DrawPanelBars(Racer racer, float barWidth, float speed)
        {
            if (racer == null) return;
            DrawBar(racer.ThrottleSmooth, ThrottleAppliedColour, barWidth, 0);
            DrawHangingBar(racer.TcsSmooth, ThrottleBoundColour, barWidth, 0);
            DrawBar(racer.StabSmooth, ThrottleBoundColour, barWidth, 1);
            DrawBar(GsLaneThrottleFloor ? racer.ThrottleFloorLevel : 0f, ThrottleBoundColour, barWidth, 2);
            SpeedTerm followed = racer.ThrottleTerm;
            DrawDeltaBar(racer.CornerArrivalMps - speed, followed == SpeedTerm.Corner ? FollowedSpeedColour : SpeedMarginColour, barWidth, BarsPerPedal);
            DrawDeltaBar(racer.RouteTermMps - speed, followed == SpeedTerm.Route ? FollowedSpeedColour : SpeedMarginColour, barWidth, BarsPerPedal + 1);
            DrawBar(racer.BrakeSmooth, BrakeBarColour, barWidth, BarsPerPedal + 2);
        }

        static void DrawBar(float input, Color colour, float barWidth, int bar)
        {
            float top = DebugPanelBottom - Math.Min(Math.Max(input, 0f), 1f) * (DebugPanelBottom - DebugPanelTop);
            float left = DebugPanelLeft + bar * (barWidth + PanelBarGap);
            DrawScreenRect(left, top, left + barWidth, DebugPanelBottom, colour);
        }

        static void DrawHangingBar(float level, Color colour, float barWidth, int bar)
        {
            float bottom = DebugPanelTop + (1f - Math.Min(Math.Max(level, 0f), 1f)) * (DebugPanelBottom - DebugPanelTop);
            float left = DebugPanelLeft + bar * (barWidth + PanelBarGap);
            DrawScreenRect(left, DebugPanelTop, left + barWidth, bottom, colour);
        }

        static void DrawDeltaBar(float marginMps, Color colour, float barWidth, int bar)
        {
            float clamped = Math.Min(Math.Max(marginMps, -SpeedDeltaFullSpanMps), SpeedDeltaFullSpanMps);
            float halfBand = (DebugPanelBottom - DebugPanelTop) * 0.5f;
            float mid = (DebugPanelTop + DebugPanelBottom) * 0.5f;
            float top = mid - (clamped >= 0f ? clamped / SpeedDeltaFullSpanMps : 0f) * halfBand;
            float bottom = mid + (clamped < 0f ? -clamped / SpeedDeltaFullSpanMps : 0f) * halfBand;
            float left = DebugPanelLeft + bar * (barWidth + PanelBarGap);
            DrawScreenRect(left, top, left + barWidth, bottom, colour);
        }

        static void DrawPanelLabels(Racer racer, float barWidth)
        {
            SpeedTerm followed = racer != null ? racer.ThrottleTerm : SpeedTerm.None;
            for (int i = 0; i < PanelLabels.Length; i++)
            {
                bool followedHere = i == BarsPerPedal && followed == SpeedTerm.Corner
                    || i == BarsPerPedal + 1 && followed == SpeedTerm.Route;
                Color colour = followedHere ? FollowedSpeedColour : PanelLabelColour;
                float centreX = DebugPanelLeft + i * (barWidth + PanelBarGap) + barWidth * 0.5f;
                DrawText(new Vector2(centreX, PanelLabelY), PanelLabels[i], colour, DrawTextFont.Standard, DrawTextAlign.Center, PanelLabelScale);
                if (racer == null || i != BarsPerPedal && i != BarsPerPedal + 1) continue;
                float intended = i == BarsPerPedal ? racer.CornerArrivalMps : racer.RouteTermMps;
                string intendedMph = intended >= SpeedSentinelMps ? "-" : ARS.MpsToMph(intended).ToString("0", CultureInfo.InvariantCulture);
                DrawText(new Vector2(centreX, PanelIntendedY), intendedMph, colour, DrawTextFont.Standard, DrawTextAlign.Center, PanelIntendedScale);
                if (i != BarsPerPedal + 1) continue;
                string radiusText = Math.Round(racer.LiveRouteRadius).ToString("0", CultureInfo.InvariantCulture) + "m";
                DrawText(new Vector2(centreX, PanelRadiusY), radiusText, colour, DrawTextFont.Standard, DrawTextAlign.Center, PanelRadiusScale);
            }
        }

        void ProbeTopEnd()
        {
            int now = Game.GameTime;
            if (now - _probeLastMs < 1000) return;
            Vehicle car = Game.Player.Character.CurrentVehicle;
            if (!CanWeUse(car)) return;

            float mps = car.Speed;
            string dMps = "--";
            if (_probeLastMs >= 0)
            {
                float dt = (now - _probeLastMs) * 0.001f;
                dMps = ((mps - _probeLastMps) / dt).ToString("+0.00;-0.00", CultureInfo.InvariantCulture);
            }
            _probeLastMs = now;
            _probeLastMps = mps;

            string perWheel;
            float thrustG = VehicleMemory.DriveForceG(car, out perWheel);
            Log(LogImportance.Error, "[TOPEND] " + MpsToMph(mps).ToString("0.0", CultureInfo.InvariantCulture) + "mph"
                + " d" + dMps + "mps"
                + " thr " + VehicleMemory.GetThrottle(car).ToString("0.00", CultureInfo.InvariantCulture)
                + " thrustG " + thrustG.ToString("0.000", CultureInfo.InvariantCulture)
                + " wheelsG[" + perWheel + "]"
                + " drag " + VehicleMemory.GetDragCoeff(car).ToString("0.000000", CultureInfo.InvariantCulture)
                + " flatVel " + VehicleMemory.GetFlatVel(car).ToString("0.00", CultureInfo.InvariantCulture)
                + " estMph " + MpsToMph(EngineTopSpeed(car)).ToString("0.0", CultureInfo.InvariantCulture));
        }

        int _probeLastMs = -1;
        float _probeLastMps;

    }
}
