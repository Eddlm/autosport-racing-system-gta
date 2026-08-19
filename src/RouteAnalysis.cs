using GTA.Math;
using System;
using System.Collections.Generic;

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

            const float entryRadius = 100f;
            const float exitRadius = 130f;
            const int smoothingNodes = 2;
            const int exitRelaxationNodes = 4;
            const int minimumCornerNodes = 5;

            // Start a circuit scan on a straight so a corner crossing the node-zero boundary
            // is detected as one region rather than split between the end and beginning.
            int scanStart = 0;
            if (!ARS.IsPointToPoint)
            {
                for (int node = 0; node < count; node++)
                {
                    if (SmoothedRadius(node, count, smoothingNodes) >= exitRadius)
                    {
                        scanStart = node;
                        break;
                    }
                }
            }

            List<int> scanNodes = new List<int>(count);
            for (int offset = 0; offset < count; offset++)
            {
                int node = ARS.IsPointToPoint
                    ? scanStart + offset
                    : Wrap(scanStart + offset, count);
                if (node >= 0 && node < count) scanNodes.Add(node);
            }

            bool inCorner = false;
            int cornerStartPosition = -1;
            int cornerDirection = 0;
            int relaxedNodes = 0;

            for (int position = 0; position < scanNodes.Count; position++)
            {
                int node = scanNodes[position];
                float radius = SmoothedRadius(node, count, smoothingNodes);
                float generalRadius = ARS.TrackPoints[node].GeneralCurveRadius;
                int direction = Math.Sign(ARS.TrackPoints[node].Angle);
                bool cornerSignal = radius < entryRadius && generalRadius < exitRadius;

                if (!inCorner)
                {
                    if (cornerSignal && direction != 0)
                    {
                        inCorner = true;
                        cornerStartPosition = position;
                        cornerDirection = direction;
                        relaxedNodes = 0;
                    }
                    continue;
                }

                if (direction != 0 && direction != cornerDirection && cornerSignal)
                {
                    AddCornerRegion(scanNodes, cornerStartPosition, position - 1, count, smoothingNodes, minimumCornerNodes);
                    cornerStartPosition = position;
                    cornerDirection = direction;
                    relaxedNodes = 0;
                    continue;
                }

                if (!cornerSignal || radius >= exitRadius || direction == 0)
                    relaxedNodes++;
                else
                    relaxedNodes = 0;

                if (relaxedNodes >= exitRelaxationNodes)
                {
                    AddCornerRegion(scanNodes, cornerStartPosition, position - relaxedNodes, count, smoothingNodes, minimumCornerNodes);
                    inCorner = false;
                    cornerStartPosition = -1;
                    cornerDirection = 0;
                    relaxedNodes = 0;
                }
            }

            if (inCorner)
                AddCornerRegion(scanNodes, cornerStartPosition, scanNodes.Count - 1, count, smoothingNodes, minimumCornerNodes);

            ARS.Log(ARS.LogImportance.Info, "Apex table: " + ARS.Corners.Count + " corners");
        }

        static void AddCornerRegion(List<int> scanNodes, int startPosition, int endPosition, int count, int smoothingNodes, int minimumCornerNodes)
        {
            if (startPosition < 0 || endPosition < startPosition || endPosition - startPosition + 1 < minimumCornerNodes)
                return;

            int apexPosition = startPosition;
            float apexRadius = float.MaxValue;
            for (int position = startPosition; position <= endPosition; position++)
            {
                // The apex radius remains the precise node radius: circumradius through
                // the node and its +/-4-node samples. Smoothing is only for region detection.
                float radius = ARS.TrackPoints[scanNodes[position]].PreciseCurveRadius;
                if (radius < apexRadius)
                {
                    apexRadius = radius;
                    apexPosition = position;
                }
            }

            if (apexRadius >= 100f || float.IsNaN(apexRadius) || float.IsInfinity(apexRadius)) return;

            float entranceRadiusTarget = apexRadius * 2f;
            for (int position = startPosition; position < apexPosition; position++)
            {
                float radius = ARS.TrackPoints[scanNodes[position]].PreciseCurveRadius;
                if (radius < entranceRadiusTarget)
                {
                    startPosition = position;
                    break;
                }
            }

            int apexNode = scanNodes[apexPosition];
            ARS.Corners.Add(new CornerPoint
            {
                Node = apexNode,
                Angle = ARS.TrackPoints[apexNode].Angle,
                StartNode = scanNodes[startPosition],
                EndNode = scanNodes[endPosition],
                LengthStart = apexPosition - startPosition,
                LengthEnd = endPosition - apexPosition,
                SupposedRadius = apexRadius,
                Speed = (float)Math.Sqrt(9.81f * apexRadius)
            });
        }

        static float SmoothedRadius(int node, int count, int halfWindow)
        {
            float curvature = 0f;
            int samples = 0;
            for (int offset = -halfWindow; offset <= halfWindow; offset++)
            {
                int sample = ARS.IsPointToPoint
                    ? node + offset
                    : Wrap(node + offset, count);
                if (sample < 0 || sample >= count) continue;

                float radius = ARS.TrackPoints[sample].PreciseCurveRadius;
                if (radius <= 0f || radius >= 999f || float.IsNaN(radius) || float.IsInfinity(radius)) continue;
                curvature += 1f / radius;
                samples++;
            }

            return samples > 0 ? 1f / (curvature / samples) : 999f;
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
