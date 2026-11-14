using GTA;
using GTA.Native;
using System;
using System.Drawing;

namespace ARS
{
    public partial class ARS
    {
        const float DebugPanelLeft = 0.35f;
        const float DebugPanelRight = 0.65f;
        const float DebugPanelTop = 0f;
        const float DebugPanelBottom = 0.3f;
        const int BarsPerPedal = 3;
        const float ThrottleBarGap = 0.004f; // gap between bars, screen width fraction
        static readonly Color ThrottleAppliedColour = Color.LimeGreen;
        static readonly Color ThrottleBoundColour = Color.FromArgb(235, 210, 60); // TCS cap and the Gs floor
        static readonly Color BrakeBarColour = Color.Red;

        public static void DrawScreenRect(float left, float top, float right, float bottom, Color colour)
        {
            // DRAW_RECT takes the centre and the full size, not a corner (commands_graphics.sch:688). The
            // 9th arg is bStereo (engine takes 9, the docs list 8): false = SCRIPT_GFX_SOLID_COLOUR.
            Function.Call(Hash.DRAW_RECT, (left + right) * 0.5f, (top + bottom) * 0.5f, right - left, bottom - top, colour.R, colour.G, colour.B, colour.A, false);
        }

        public static void DrawDebugPanel(Racer racer)
        {
            DrawScreenRect(DebugPanelLeft, DebugPanelTop, DebugPanelRight, DebugPanelBottom, Color.Black);
            DrawPanelBars(racer);
        }

        // Six bars in the panel band, bottom edge pinned to DebugPanelBottom, an input of 1 filling the
        // band: the racer's applied throttle, the TCS max-throttle cap, the Gs floor as throttle (bar
        // hidden when the Gs floor setting is off), then three brake bars. Column 0 also carries a bar
        // hanging from the band top down to the TCS cap line, so the cap reads as a pull on the pedal.
        static void DrawPanelBars(Racer racer)
        {
            if (racer == null) return;
            float barWidth = (DebugPanelRight - DebugPanelLeft - (2 * BarsPerPedal - 1) * ThrottleBarGap) / (2 * BarsPerPedal);
            DrawBar(racer.ThrottleFinal, ThrottleAppliedColour, barWidth, 0);
            DrawHangingBar(racer.Control.MaxThrottleFromTCS, ThrottleBoundColour, barWidth, 0);
            DrawBar(racer.Control.MaxThrottleFromTCS, ThrottleBoundColour, barWidth, 1);
            DrawBar(GsLaneThrottleFloor ? racer.ThrottleFloorLevel : 0f, ThrottleBoundColour, barWidth, 2);
            for (int i = 0; i < BarsPerPedal; i++)
                DrawBar(racer.BrakeFinal, BrakeBarColour, barWidth, BarsPerPedal + i);
        }

        static void DrawBar(float input, Color colour, float barWidth, int bar)
        {
            float top = DebugPanelBottom - Math.Min(Math.Max(input, 0f), 1f) * (DebugPanelBottom - DebugPanelTop);
            float left = DebugPanelLeft + bar * (barWidth + ThrottleBarGap);
            DrawScreenRect(left, top, left + barWidth, DebugPanelBottom, colour);
        }

        // A bar pinned to the band top, hanging down until its lower edge reaches the given throttle level.
        static void DrawHangingBar(float level, Color colour, float barWidth, int bar)
        {
            float bottom = DebugPanelTop + (1f - Math.Min(Math.Max(level, 0f), 1f)) * (DebugPanelBottom - DebugPanelTop);
            float left = DebugPanelLeft + bar * (barWidth + ThrottleBarGap);
            DrawScreenRect(left, DebugPanelTop, left + barWidth, bottom, colour);
        }
    }
}
