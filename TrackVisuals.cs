using GTA;
using GTA.Math;
using System.Collections.Generic;
using System.Drawing;

namespace ARS
{
    public static class TrackVisuals
    {
        public static void DrawRoute(List<Vector3> nodes, Dictionary<int, float> widths, bool editorActive)
        {
            if (nodes.Count == 0) return;
            int closest = ARS.ClosestNodeToPlace(Game.Player.Character.Position, nodes);
            int start = System.Math.Max(0, closest - 50);
            int end = System.Math.Min(nodes.Count - 1, closest + 50);
            if (editorActive) World.DrawMarker(MarkerType.CheckeredFlagRect, nodes[0] + new Vector3(0, 0, 3f), (nodes[1] - nodes[0]).Normalized, Vector3.Zero, new Vector3(5f, 5f, 5f), Color.White);
            for (int node = start; node < end; node += 5)
            {
                float width;
                if (!widths.TryGetValue(node, out width) || width == 0f) continue;
                Vector3 previous = nodes[node == 0 ? nodes.Count - 1 : node - 1];
                Vector3 direction = (nodes[node] - previous).Normalized;
                Vector3 side = Vector3.Cross(direction, Vector3.WorldUp) * width;
                World.DrawMarker(MarkerType.DebugSphere, nodes[node] + side, Vector3.Zero, Vector3.Zero, new Vector3(.2f, .2f, .2f), Color.Blue);
                World.DrawMarker(MarkerType.DebugSphere, nodes[node] - side, Vector3.Zero, Vector3.Zero, new Vector3(.2f, .2f, .2f), Color.Blue);
            }
        }
    }
}
