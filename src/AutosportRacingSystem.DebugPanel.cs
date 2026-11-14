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
        const int ThrottleBarCount = 6;
        const float ThrottleBarGap = 0.004f; // gap between bars, screen width fraction
        static readonly Color ThrottleBarColour = Color.FromArgb(235, 210, 60);

        public static void DrawScreenRect(float left, float top, float right, float bottom, Color colour)
        {
            // DRAW_RECT takes the centre and the full size, not a corner (commands_graphics.sch:688). The
            // 9th arg is bStereo (engine takes 9, the docs list 8): false = SCRIPT_GFX_SOLID_COLOUR.
            Function.Call(Hash.DRAW_RECT, (left + right) * 0.5f, (top + bottom) * 0.5f, right - left, bottom - top, colour.R, colour.G, colour.B, colour.A, false);
        }

        public static void DrawDebugPanel(Racer racer)
        {
            DrawScreenRect(DebugPanelLeft, DebugPanelTop, DebugPanelRight, DebugPanelBottom, Color.Black);
            DrawThrottleBars(racer);
        }

        // Six identical bars in the panel band: bottom edge pinned to DebugPanelBottom, top edge rises
        // with the racer's throttle input (0 = no bar, full throttle reaches DebugPanelTop).
        static void DrawThrottleBars(Racer racer)
        {
            if (racer == null) return;
            float throttle = Math.Min(Math.Max(racer.ThrottleFinal, 0f), 1f);
            float top = DebugPanelBottom - throttle * (DebugPanelBottom - DebugPanelTop);
            float barWidth = (DebugPanelRight - DebugPanelLeft - (ThrottleBarCount - 1) * ThrottleBarGap) / ThrottleBarCount;
            for (int i = 0; i < ThrottleBarCount; i++)
            {
                float left = DebugPanelLeft + i * (barWidth + ThrottleBarGap);
                DrawScreenRect(left, top, left + barWidth, DebugPanelBottom, ThrottleBarColour);
            }
        }
    }
}
