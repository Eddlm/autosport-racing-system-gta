using GTA.Math;
using System;

namespace ARS
{
    public static class RouteAnalysis
    {
        public static void Generate()
        {
            ARS.Log(ARS.LogImportance.Info, "Generating route info...");
            ARS.TrackPoints.Clear();
            while (ARS.TrackPoints.Count < ARS.RouteNodes.Count)
            {
                int node = ARS.TrackPoints.Count;
                ARS.TrackPoints.Add(new TrackPoint { Node = node, Position = ARS.RouteNodes[node] });
            }

            int nodeCount = ARS.RouteNodes.Count;
            foreach (TrackPoint point in ARS.TrackPoints)
            {
                int average = (int)(point.TrackHalfWidth * 2);
                bool canCompute = ARS.IsPointToPoint
                    ? point.Node > 50 && point.Node < nodeCount - 50
                    : point.Node >= 0 && point.Node < nodeCount;

                if (canCompute)
                {
                    Vector3 left = (ARS.RouteNodes[point.Node] - ARS.RouteNodes[Wrap(point.Node - average, nodeCount)]).Normalized;
                    Vector3 forward = (ARS.RouteNodes[Wrap(point.Node + average, nodeCount)] - ARS.RouteNodes[point.Node]).Normalized;
                    point.Direction = (ARS.RouteNodes[Wrap(point.Node + 2, nodeCount)] - ARS.RouteNodes[Wrap(point.Node - 2, nodeCount)]).Normalized;
                    point.Elevation = (ARS.RouteNodes[Wrap(point.Node + 3, nodeCount)] - ARS.RouteNodes[Wrap(point.Node - 3, nodeCount)]).Normalized.Z * 90f;
                    left.Z = 0f;
                    forward.Z = 0f;
                    point.Angle = Vector3.SignedAngle(left, forward, Vector3.WorldUp);
                    if (float.IsNaN(point.Angle) || float.IsInfinity(point.Angle)) point.Angle = 0f;
                    point.GeneralCurveRadius = Circumradius3D(ARS.RouteNodes[Wrap(point.Node - 10, nodeCount)], ARS.RouteNodes[Wrap(point.Node + 10, nodeCount)], ARS.RouteNodes[point.Node]);
                    point.PreciseCurveRadius = Circumradius3D(ARS.RouteNodes[Wrap(point.Node - 4, nodeCount)], ARS.RouteNodes[Wrap(point.Node + 4, nodeCount)], ARS.RouteNodes[point.Node]);
                }
                else
                {
                    point.Angle = 0f;
                    point.Direction = (ARS.RouteNodes[10] - ARS.RouteNodes[5]).Normalized;
                    point.Elevation = 0f;
                }

                if (ARS.NodeHalfWidths.ContainsKey(point.Node)) point.TrackHalfWidth = ARS.NodeHalfWidths[point.Node];
            }

            BuildApexTable();
            ARS.Log(ARS.LogImportance.Info, "Route generated");
        }

        public static void BuildApexTable()
        {
            ARS.Corners.Clear();
            int count = ARS.TrackPoints.Count;
            if (count < 1) return;

            const int chunkSize = 30;
            const float radiusLimit = 100f;
            for (int start = 0; start < count; start += chunkSize)
            {
                int bestNode = -1;
                float bestRadius = float.MaxValue;
                for (int node = start; node < start + chunkSize; node++)
                {
                    int index = ARS.IsPointToPoint ? node : Wrap(node, count);
                    if (index >= count) index = count - 1;
                    float radius = ARS.TrackPoints[index].PreciseCurveRadius;
                    if (radius < bestRadius) { bestRadius = radius; bestNode = index; }
                }
                if (bestNode < 0 || bestRadius >= radiusLimit) continue;
                ARS.Corners.Add(new CornerPoint
                {
                    Node = bestNode,
                    SupposedRadius = bestRadius,
                    Speed = (float)Math.Sqrt(9.81f * bestRadius)
                });
            }
            ARS.Log(ARS.LogImportance.Info, "Apex table: " + ARS.Corners.Count + " corners");
        }

        public static float Circumradius3D(Vector3 a, Vector3 b, Vector3 midpoint)
        {
            return Circumradius2D(new Vector2(a.X, a.Y), new Vector2(b.X, b.Y), new Vector2(midpoint.X, midpoint.Y));
        }

        public static float Circumradius2D(Vector2 a, Vector2 b, Vector2 midpoint)
        {
            Vector2 midpoint1 = (a + b) / 2;
            float dx1 = b.X - a.X;
            float slope1 = dx1 != 0f ? (b.Y - a.Y) / dx1 : float.PositiveInfinity;
            float perpendicular1 = slope1 != 0f ? -1 / slope1 : float.PositiveInfinity;
            Vector2 midpoint2 = (b + midpoint) / 2;
            float dx2 = midpoint.X - b.X;
            float slope2 = dx2 != 0f ? (midpoint.Y - b.Y) / dx2 : float.PositiveInfinity;
            float perpendicular2 = slope2 != 0f ? -1 / slope2 : float.PositiveInfinity;
            float denominator = perpendicular2 - perpendicular1;
            if (denominator == 0f || float.IsNaN(denominator) || float.IsInfinity(denominator)) return 999f;
            float centerX = (midpoint1.Y - midpoint2.Y + perpendicular2 * midpoint2.X - perpendicular1 * midpoint1.X) / denominator;
            float centerY = midpoint1.Y + perpendicular1 * (centerX - midpoint1.X);
            float radius = Vector2.Distance(new Vector2(centerX, centerY), a);
            return float.IsNaN(radius) || float.IsInfinity(radius) ? 999f : radius;
        }

        static int Wrap(int index, int count)
        {
            int wrapped = index % count;
            return wrapped < 0 ? wrapped + count : wrapped;
        }
    }
}
