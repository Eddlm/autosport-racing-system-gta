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
        const float PanelBarGap = 0.004f; // gap between bars, screen width fraction
        const float SpeedDeltaFullSpanMps = 30f; // margin-bar scale: 30 m/s of margin reaches a band edge
        static readonly Color ThrottleAppliedColour = Color.LimeGreen;
        static readonly Color ThrottleBoundColour = Color.FromArgb(235, 210, 60); // the hanging TCS pull, the stability cap and the Gs floor
        static readonly Color BrakeBarColour = Color.Red;
        static readonly Color SpeedMarginColour = Color.Orange; // the APEX and ROUT bars
        static readonly Color FollowedSpeedColour = Color.DodgerBlue; // the column whose term is driving the speed
        static readonly Color PanelLabelColour = Color.White;
        const float PanelLabelY = 0.272f; // the label row, inside the band above its bottom edge
        const float PanelLabelScale = 0.35f;
        const float PanelIntendedY = 0.008f; // the intended speed, at the top of the APEX and ROUT columns
        const float PanelIntendedScale = 0.30f;
        const float SpeedSentinelMps = 998f; // the term's own open sentinel: no corner ahead, or no finite radius
        static readonly string[] PanelLabels = { "THR", "STB", "FLR", "APEX", "ROUT", "BRAKE" };

        public static void DrawScreenRect(float left, float top, float right, float bottom, Color colour)
        {
            // DRAW_RECT takes the centre and the full size, not a corner (commands_graphics.sch:688). The
            // 9th arg is bStereo (engine takes 9, the docs list 8): false = SCRIPT_GFX_SOLID_COLOUR.
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

        // Six bars in the panel band, bottom edge pinned to DebugPanelBottom, an input of 1 filling the
        // band: applied throttle, the stability cap and the brake bar ride SmoothFloat followers; the Gs
        // floor bar shows the imposed floor (hidden when the Gs floor setting is off). Columns 4-5 are
        // margins against the corner's arrival speed and the route ask, centred on the middle: margin
        // above, deficit below, 30 m/s reaches a band edge. They never grey out, and the one whose term is
        // driving the speed turns blue; the intended speed rides at the top of the band in mph, so a
        // learned change reads as a number even when the bar has re-settled. Column 0 also carries a bar
        // hanging from the band top down to the TCS cap line.
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

        // A bar pinned to the band top, hanging down until its lower edge reaches the given throttle level.
        static void DrawHangingBar(float level, Color colour, float barWidth, int bar)
        {
            float bottom = DebugPanelTop + (1f - Math.Min(Math.Max(level, 0f), 1f)) * (DebugPanelBottom - DebugPanelTop);
            float left = DebugPanelLeft + bar * (barWidth + PanelBarGap);
            DrawScreenRect(left, DebugPanelTop, left + barWidth, bottom, colour);
        }

        // A bar centred on the band's middle: a positive margin rises above the middle, a negative one
        // sinks below it, and |margin| = SpeedDeltaFullSpanMps reaches a band edge. Past that span the bar
        // pins to the edge and keeps its colour: an out-of-range margin is told apart by the bar being full.
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
            }
        }
    }
}
