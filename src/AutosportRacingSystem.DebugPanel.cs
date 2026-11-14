using GTA;
using GTA.Native;
using System.Drawing;

namespace ARS
{
    public partial class ARS
    {
        const float DebugPanelLeft = 0.35f;
        const float DebugPanelRight = 0.65f;
        const float DebugPanelTop = 0f;
        const float DebugPanelBottom = 0.3f;

        public static void DrawScreenRect(float left, float top, float right, float bottom, Color colour)
        {
            // DRAW_RECT takes the centre and the full size, not a corner (commands_graphics.sch:688).
            Function.Call(Hash.DRAW_RECT, (left + right) * 0.5f, (top + bottom) * 0.5f, right - left, bottom - top, colour.R, colour.G, colour.B, colour.A, false);
        }

        public static void DrawDebugPanel()
        {
            DrawScreenRect(DebugPanelLeft, DebugPanelTop, DebugPanelRight, DebugPanelBottom, Color.Black);
        }
    }
}
