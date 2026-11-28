// Debug drawing of the track: nodes, widths, corners and the route editor.
using GTA;
using GTA.Math;
using System;
using System.Collections.Generic;
using System.Drawing;

namespace ARS
{
    public static class TrackVisuals
    {
        static readonly Color CrestLineColor = Color.FromArgb(255, 255, 140, 0);
        static readonly Color DipLineColor = Color.FromArgb(255, 0, 200, 255);
        static readonly Color CrestIgnoredColor = Color.FromArgb(200, 150, 150, 150);
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

        public static void DrawCornerCheckpoints(Racer player, List<CornerPoint> corners, List<TrackPoint> trackPoints)
        {
            if (corners.Count == 0 || trackPoints.Count == 0) return;
            int centreNode = player.CurrentTrackPoint.Node;

            for (int i = 0; i < corners.Count; i++)
            {
                CornerPoint corner = corners[i];
                int node = corner.Node;
                if (node < 0 || node >= trackPoints.Count) continue;
                Vector3 apexGround = trackPoints[node].Position;

                int delta = ARS.IsPointToPoint ? node - centreNode : ((node - centreNode) % trackPoints.Count + trackPoints.Count) % trackPoints.Count;
                if (!ARS.IsBetween(delta, -10, 400)) continue;

                DrawCornerApexMarker(i, apexGround, corners, trackPoints);
            }
        }

        public static void DrawCornerCheckpoints(Vector3 center, float radiusMeters, List<CornerPoint> corners, List<TrackPoint> trackPoints)
        {
            if (corners.Count == 0 || trackPoints.Count == 0) return;

            for (int i = 0; i < corners.Count; i++)
            {
                CornerPoint corner = corners[i];
                int node = corner.Node;
                if (node < 0 || node >= trackPoints.Count) continue;
                Vector3 apexGround = trackPoints[node].Position;
                if (apexGround.DistanceTo2D(center) > radiusMeters) continue;

                DrawCornerApexMarker(i, apexGround, corners, trackPoints);
            }
        }

        static void DrawCornerApexMarker(int cornerIndex, Vector3 apexGround, List<CornerPoint> corners, List<TrackPoint> trackPoints)
        {
            CornerPoint corner = corners[cornerIndex];
            int node = corner.Node;
            Vector3 apex = apexGround + new Vector3(0f, 0f, -1.5f);
            Color ringColor = Color.FromArgb(220, 255, 225, 80);
            Color chevronColor = Color.FromArgb(242, 255, 140, 0);
            MarkerType chevronMarker = corner.IsChicane ? MarkerType.ChevronUpx2 : (MarkerType)20;
            if (corner.IsChicane) chevronColor = Color.FromArgb(242, 0, 200, 255);

            int nextCornerIndex = cornerIndex == corners.Count - 1 ? (ARS.IsPointToPoint ? -1 : 0) : cornerIndex + 1;
            Vector3 nextApex = nextCornerIndex >= 0 ? trackPoints[corners[nextCornerIndex].Node].Position : apexGround;

            float trackWidth = trackPoints[node].TrackHalfWidth * 2f;
            World.DrawMarker((MarkerType)1, apex, Vector3.Zero, Vector3.Zero, new Vector3(trackWidth, trackWidth, 2.97f), ringColor, false, true, 2, false, "", "", false);

            if (nextCornerIndex < 0) return;

            Vector3 toNext = nextApex - apexGround;
            toNext.Z = 0f;
            if (toNext.LengthSquared() <= 0.0001f) return;
            toNext = toNext.Normalized;

            Vector3 chevronPos = apexGround + new Vector3(0f, 0f, 2.35f);
            float chevronSize = ARS.Clamp(trackWidth * 0.18f, 1.2f, 2.2f);
            World.DrawMarker(chevronMarker, chevronPos, toNext, new Vector3(89f, 0f, -90f), new Vector3(chevronSize, chevronSize, chevronSize), chevronColor, false, false, 2, false, "", "", false);
        }

        public static void DrawCornerRegions(Racer player, List<CornerPoint> corners, List<TrackPoint> trackPoints)
        {
            if (corners.Count == 0 || trackPoints.Count == 0) return;
            int centreNode = player.CurrentTrackPoint.Node;

            for (int i = 0; i < corners.Count; i++)
            {
                CornerPoint corner = corners[i];
                int apexNode = corner.Node;
                if (apexNode < 0 || apexNode >= trackPoints.Count) continue;

                int delta = ARS.IsPointToPoint ? apexNode - centreNode : ((apexNode - centreNode) % trackPoints.Count + trackPoints.Count) % trackPoints.Count;
                if (!ARS.IsBetween(delta, -10, 400)) continue;

                Color startColor = Color.FromArgb(200, 0, 255, 0);
                Color apexColor = Color.FromArgb(200, 255, 255, 0);
                Color endColor = Color.FromArgb(200, 255, 0, 0);

                if (corner.StartNode >= 0 && corner.StartNode < trackPoints.Count)
                    DrawTrackCrossingLine(trackPoints, corner.StartNode, startColor);
                DrawTrackCrossingLine(trackPoints, apexNode, apexColor);
                if (corner.EndNode >= 0 && corner.EndNode < trackPoints.Count)
                    DrawTrackCrossingLine(trackPoints, corner.EndNode, endColor);
            }
        }

        public static void DrawCornerRegions(Vector3 center, float radiusMeters, List<CornerPoint> corners, List<TrackPoint> trackPoints)
        {
            if (corners.Count == 0 || trackPoints.Count == 0) return;

            for (int i = 0; i < corners.Count; i++)
            {
                CornerPoint corner = corners[i];
                int apexNode = corner.Node;
                if (apexNode < 0 || apexNode >= trackPoints.Count) continue;
                if (trackPoints[apexNode].Position.DistanceTo2D(center) > radiusMeters) continue;

                Color startColor = Color.FromArgb(200, 0, 255, 0);
                Color apexColor = Color.FromArgb(200, 255, 255, 0);
                Color endColor = Color.FromArgb(200, 255, 0, 0);

                if (corner.StartNode >= 0 && corner.StartNode < trackPoints.Count)
                    DrawTrackCrossingLine(trackPoints, corner.StartNode, startColor);
                DrawTrackCrossingLine(trackPoints, apexNode, apexColor);
                if (corner.EndNode >= 0 && corner.EndNode < trackPoints.Count)
                    DrawTrackCrossingLine(trackPoints, corner.EndNode, endColor);
            }
        }

        public static void DrawCrests(Racer player, List<CrestRegion> crestRegions, List<int> dipPeaks, List<TrackPoint> trackPoints)
        {
            if (trackPoints.Count == 0) return;
            int centreNode = player.CurrentTrackPoint.Node;
            int count = trackPoints.Count;
            DrawCrestDipMarkers(point =>
            {
                int delta = ARS.IsPointToPoint ? point.Node - centreNode : ((point.Node - centreNode) % count + count) % count;
                return ARS.IsBetween(delta, -10, 400);
            }, crestRegions, dipPeaks, trackPoints);
        }

        public static void DrawCrests(Vector3 center, float radiusMeters, List<CrestRegion> crestRegions, List<int> dipPeaks, List<TrackPoint> trackPoints)
        {
            DrawCrestDipMarkers(point => point.Position.DistanceTo2D(center) <= radiusMeters, crestRegions, dipPeaks, trackPoints);
        }

        static void DrawCrestDipMarkers(Func<TrackPoint, bool> include, List<CrestRegion> crestRegions, List<int> dipPeaks, List<TrackPoint> trackPoints)
        {
            if (trackPoints.Count == 0) return;

            foreach (CrestRegion region in crestRegions)
            {
                int node = region.PeakNode;
                if (node < 0 || node >= trackPoints.Count) continue;
                if (!include(trackPoints[node])) continue;
                DrawTrackCrossingLine(trackPoints, node, CrestLineColor);
            }

            foreach (int node in dipPeaks)
            {
                if (node < 0 || node >= trackPoints.Count) continue;
                if (!include(trackPoints[node])) continue;
                DrawTrackCrossingLine(trackPoints, node, DipLineColor);
            }
        }

        public static void DrawLiveCrestChecks(Racer racer, List<TrackPoint> trackPoints)
        {
            if (trackPoints.Count == 0) return;
            if (racer.LiveRouteCrestNode >= 0 && racer.LiveRouteCrestNode < trackPoints.Count)
                DrawTrackCrossingLine(trackPoints, racer.LiveRouteCrestNode, Color.FromArgb(200, 80, 160, 255));
        }

        public static void DrawCrestLookahead(Racer racer, List<TrackPoint> trackPoints)
        {
            int count = trackPoints.Count;
            if (count == 0 || racer == null || !ARS.CanWeUse(racer.Car)) return;
            float speed = racer.Car.Velocity.Length();
            if (speed < 1f) return;

            int region = ARS.CrestRegionAhead(racer.CurrentTrackPoint.Node, speed * ARS.CrestLookaheadSeconds);
            if (region < 0) return;

            CrestRegion crest = ARS.CrestRegions[region];
            int node = crest.PeakNode;
            if (node < 0 || node >= count) return;

            Color markColor = racer.CrestIsBinding ? CrestLineColor : CrestIgnoredColor;
            DrawTrackCrossingLine(trackPoints, node, markColor);
            ARS.DrawText(trackPoints[node].Position + Vector3.WorldUp * 2.2f,
                crest.Radius.ToString("0") + " m", markColor, 0.5f);
        }

        public static void DrawRouteRadiusChecks(Racer racer, List<TrackPoint> trackPoints)
        {
            if (trackPoints.Count == 0) return;
            if (racer.LiveRouteRadiusNearNode >= 0 && racer.LiveRouteRadiusNearNode < trackPoints.Count)
                DrawTrackCrossingLine(trackPoints, racer.LiveRouteRadiusNearNode, Color.FromArgb(220, 255, 0, 255));
            if (racer.LiveRouteRadiusMidNode >= 0 && racer.LiveRouteRadiusMidNode < trackPoints.Count)
                DrawTrackCrossingLine(trackPoints, racer.LiveRouteRadiusMidNode, Color.FromArgb(220, 255, 110, 255));
            if (racer.LiveRouteRadiusFarNode >= 0 && racer.LiveRouteRadiusFarNode < trackPoints.Count)
                DrawTrackCrossingLine(trackPoints, racer.LiveRouteRadiusFarNode, Color.FromArgb(220, 255, 180, 255));
            if (racer.LiveSlidRadiusNearNode >= 0 && racer.LiveSlidRadiusNearNode < trackPoints.Count)
                DrawTrackCrossingLine(trackPoints, racer.LiveSlidRadiusNearNode, Color.FromArgb(220, 0, 220, 220));
            if (racer.LiveSlidRadiusMidNode >= 0 && racer.LiveSlidRadiusMidNode < trackPoints.Count)
                DrawTrackCrossingLine(trackPoints, racer.LiveSlidRadiusMidNode, Color.FromArgb(220, 0, 220, 220));
            if (racer.LiveSlidRadiusFarNode >= 0 && racer.LiveSlidRadiusFarNode < trackPoints.Count)
                DrawTrackCrossingLine(trackPoints, racer.LiveSlidRadiusFarNode, Color.FromArgb(220, 0, 220, 220));
        }

        static void DrawTrackCrossingLine(List<TrackPoint> trackPoints, int node, Color color)
        {
            TrackPoint tp = trackPoints[node];
            Vector3 dir = tp.Direction;
            dir.Z = 0f;
            if (dir.LengthSquared() < 0.0001f) return;
            dir.Normalize();
            Vector3 right = Vector3.Cross(dir, Vector3.WorldUp);
            float halfWide = tp.TrackHalfWidth;
            Vector3 leftEdge = tp.Position - right * halfWide + new Vector3(0, 0, 0.5f);
            Vector3 rightEdge = tp.Position + right * halfWide + new Vector3(0, 0, 0.5f);
            ARS.DrawLine(leftEdge, rightEdge, color);
        }

        public static void DrawEdgeChevrons(Racer player, List<TrackPoint> trackPoints)
        {
            if (trackPoints.Count == 0) return;
            DrawEdgeChevronsAround(player.CurrentTrackPoint.Node, trackPoints);
        }

        public static void DrawEdgeChevrons(Vector3 center, List<TrackPoint> trackPoints)
        {
            if (trackPoints.Count == 0 || ARS.RouteNodes.Count == 0) return;
            int node = ARS.ClosestNodeToPlace(center, ARS.RouteNodes);
            DrawEdgeChevronsAround(node, trackPoints);
        }

        static void DrawEdgeChevronsAround(int centreNode, List<TrackPoint> trackPoints)
        {
            if (trackPoints.Count == 0) return;
            Color color = Color.FromArgb(128, 0, 120, 255);

            for (int node = centreNode - 30; node <= centreNode + 30; node++)
            {
                if (ARS.IsPointToPoint && (node < 0 || node >= trackPoints.Count)) continue;
                int index = ARS.IsPointToPoint ? node : ((node % trackPoints.Count) + trackPoints.Count) % trackPoints.Count;
                if (index % 2 == 0) continue;
                TrackPoint tp = trackPoints[index];

                Vector3 direction = tp.Direction;
                direction.Z = 0f;
                if (direction.LengthSquared() <= 0.0001f) continue;
                direction = direction.Normalized;

                Vector3 edgeRight = Vector3.Cross(direction, Vector3.WorldUp).Normalized;
                for (int side = -1; side <= 1; side += 2)
                {
                    Vector3 pos = tp.Position + edgeRight * (tp.TrackHalfWidth * side) + new Vector3(0f, 0f, 0.6f);
                    World.DrawMarker((MarkerType)20, pos, direction, new Vector3(89f, 0f, -90f), new Vector3(1f, 1f, 1f), color, false, false, 2, false, "", "", false);
                }
            }
        }
    }
}
