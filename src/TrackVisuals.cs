using GTA;
using GTA.Math;
using System;
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

        // Racing path (player on the grid): draw the checkpoint at every corner apex that is both
        // ahead of the player AND within the ahead node window, using the player racer's live
        // CurrentTrackPoint.Node as the LOD center. Unchanged racing visuals.
        public static void DrawCornerCheckpoints(Racer player, List<CornerPoint> corners, List<TrackPoint> trackPoints)
        {
            if (corners.Count == 0 || trackPoints.Count == 0) return;
            int playerNode = player.CurrentTrackPoint.Node;

            for (int i = 0; i < corners.Count; i++)
            {
                CornerPoint corner = corners[i];
                int node = corner.Node;
                if (node < 0 || node >= trackPoints.Count) continue;
                Vector3 apexGround = trackPoints[node].Position;

                int delta = ARS.IsPointToPoint ? node - playerNode : ((node - playerNode) % trackPoints.Count + trackPoints.Count) % trackPoints.Count;
                if (!ARS.IsBetween(delta, -10, 400)) continue;

                DrawCornerApexMarker(i, apexGround, corners, trackPoints);
            }
        }

        // Draw a FiveM-style checkpoint (torus ring + orange chevron) at every corner apex that is
        // within the given radius of the center position. Chevrons are centered above the apex and
        // point toward the next corner.
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

        // Shared ring + chevron rendering for a single corner apex (used by the racing and spectator paths).
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

        // Crossing lines at entrance, apex, and exit of each corner within range.
        public static void DrawCornerRegions(Racer player, List<CornerPoint> corners, List<TrackPoint> trackPoints)
        {
            if (corners.Count == 0 || trackPoints.Count == 0) return;
            int playerNode = player.CurrentTrackPoint.Node;

            for (int i = 0; i < corners.Count; i++)
            {
                CornerPoint corner = corners[i];
                int apexNode = corner.Node;
                if (apexNode < 0 || apexNode >= trackPoints.Count) continue;

                int delta = ARS.IsPointToPoint ? apexNode - playerNode : ((apexNode - playerNode) % trackPoints.Count + trackPoints.Count) % trackPoints.Count;
                if (!ARS.IsBetween(delta, -10, 400)) continue;

                Color startColor = Color.FromArgb(200, 0, 255, 0);   // green = entrance
                Color apexColor = Color.FromArgb(200, 255, 255, 0);  // yellow = apex
                Color endColor = Color.FromArgb(200, 255, 0, 0);     // red = exit

                if (corner.StartNode >= 0 && corner.StartNode < trackPoints.Count)
                    DrawTrackCrossingLine(trackPoints, corner.StartNode, startColor);
                DrawTrackCrossingLine(trackPoints, apexNode, apexColor);
                if (corner.EndNode >= 0 && corner.EndNode < trackPoints.Count)
                    DrawTrackCrossingLine(trackPoints, corner.EndNode, endColor);
            }
        }

        // Spectator overload: draws corner regions within a radius of the center position.
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

        // Outside approach line: blue line from the racer's current node to the corner entrance.
        public static void DrawOutsideApproachLine(Racer racer, List<TrackPoint> trackPoints)
        {
            if (racer.Brain.Corner == null || trackPoints.Count == 0) return;
            CornerPoint cp = racer.Brain.Corner.Point;
            int entranceNode = cp.StartNode >= 0 ? cp.StartNode : cp.Node;
            if (entranceNode < 0 || entranceNode >= trackPoints.Count) return;

            int fromNode = racer.CurrentTrackPoint.Node;
            int steps = entranceNode - fromNode;
            if (!ARS.IsPointToPoint && steps < 0) steps += trackPoints.Count;
            steps = Math.Min(steps, 200);

            // Same sign the AI uses: cornerDir * halfWidth (right for positive angle, left for negative).
            float cornerDir = Math.Sign(cp.Angle);
            if (cornerDir == 0f) return;

            Color blue = Color.FromArgb(200, 0, 120, 255);
            for (int n = 0; n < steps; n++)
            {
                int nodeA = fromNode + n;
                int nodeB = fromNode + n + 1;
                if (!ARS.IsPointToPoint)
                {
                    nodeA = nodeA % trackPoints.Count;
                    nodeB = nodeB % trackPoints.Count;
                }
                if (nodeA >= trackPoints.Count || nodeB >= trackPoints.Count) break;
                TrackPoint tpA = trackPoints[nodeA];
                TrackPoint tpB = trackPoints[nodeB];
                Vector3 dirA = tpA.Direction; dirA.Z = 0f;
                Vector3 dirB = tpB.Direction; dirB.Z = 0f;
                if (dirA.LengthSquared() < 0.0001f || dirB.LengthSquared() < 0.0001f) continue;
                Vector3 rightA = Vector3.Cross(dirA.Normalized, Vector3.WorldUp);
                Vector3 rightB = Vector3.Cross(dirB.Normalized, Vector3.WorldUp);
                Vector3 posA = tpA.Position + rightA * (tpA.TrackHalfWidth * cornerDir) + new Vector3(0, 0, 0.5f);
                Vector3 posB = tpB.Position + rightB * (tpB.TrackHalfWidth * cornerDir) + new Vector3(0, 0, 0.5f);
                ARS.DrawLine(posA, posB, blue);
            }
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

        // Small blue chevrons on both track edges, pointing ahead, on odd absolute nodes within ±30 of the given node.
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

        static void DrawEdgeChevronsAround(int playerNode, List<TrackPoint> trackPoints)
        {
            if (trackPoints.Count == 0) return;
            Color color = Color.FromArgb(128, 0, 120, 255);

            for (int node = playerNode - 30; node <= playerNode + 30; node++)
            {
                if (ARS.IsPointToPoint && (node < 0 || node >= trackPoints.Count)) continue;
                int index = ARS.IsPointToPoint ? node : ((node % trackPoints.Count) + trackPoints.Count) % trackPoints.Count;
                if (index % 2 == 0) continue; // odd nodes only so chevrons sit on a fixed track lattice
                TrackPoint tp = trackPoints[index];

                Vector3 direction = tp.Direction;
                direction.Z = 0f;
                if (direction.LengthSquared() <= 0.0001f) continue;
                direction = direction.Normalized;

                Vector3 edgeRight = Vector3.Cross(direction, Vector3.WorldUp).Normalized;
                for (int side = -1; side <= 1; side += 2)
                {
                    Vector3 pos = tp.Position + edgeRight * (tp.TrackHalfWidth * side) + new Vector3(0f, 0f, 0.2f);
                    World.DrawMarker((MarkerType)20, pos, direction, new Vector3(89f, 0f, -90f), new Vector3(1f, 1f, 1f), color, false, false, 2, false, "", "", false);
                }
            }
        }
    }
}
