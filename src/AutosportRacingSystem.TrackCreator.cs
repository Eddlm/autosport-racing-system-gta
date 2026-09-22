using GTA;
using GTA.Math;
using System;
using System.Collections.Generic;
using System.Drawing;

namespace ARS
{
    // The in-game track creator (route editor). Split out of AutosportRacingSystem.cs; the recording code
    // below is unchanged by that move. StartTrackCreator is the mode's entry point, reached from the Track
    // Creator submenu, and it takes over the free camera because that camera is the editing surface:
    // HandleTrackCreator records only while it is active. Saving the recorded route is still unwired.
    public partial class ARS
    {
        public static Dictionary<int, float> EditNodeHalfWidths = new Dictionary<int, float>();
        static bool _routeEditorActive = false;
        List<Vector3> _routeSection = new List<Vector3>();
        Vector3 _bezierStartAnchor = Vector3.Zero;
        int _pathWidth = 5;

        // CleanEverything tears down any loaded track and clears the route statics, so it must run first.
        void StartTrackCreator()
        {
            if (!CanWeUse(FreeCamRide))
            {
                Log(LogImportance.Error, "Track creator: the freecam ride is unavailable, so creator mode was not entered.");
                return;
            }

            CleanEverything();
            _routeSection.Clear();
            _routeEditorActive = true;
            if (!_freeCam.IsActive) _freeCam.Toggle();
        }

        // SaveRoute always builds a fresh document, so this is the new-track path only. Rewriting a loaded
        // track's file is UpdateRoute, which stays unwired.
        void SaveTrackFromCreator()
        {
            if (!_routeEditorActive)
            {
                UI.Notify("~r~Start the track creator before saving a route.");
                return;
            }
            if (RouteNodes.Count < 2)
            {
                UI.Notify("~r~Record a route before saving: place the start line, then Apply Section.");
                return;
            }

            UI.ShowSubtitle("Write a name for the track.", 5000);
            SaveRoute(Game.GetUserInput(30));
        }

        void HandleTrackCreator()
        {

            
            int cool = -1; 

            if (RouteNodes.Count > 50)
            {
                if (_routeSection.Count > 0)
                {
                    Vector3 last = _routeSection[_routeSection.Count - 1];
                    if (RouteNodes[RouteNodes.Count - 1].DistanceTo(RouteNodes[0]) < 20f)
                    {
                        if (RouteNodes[0].DistanceTo(RouteNodes[RouteNodes.Count - 1]) < 2f)
                        {
                            cool = 1;
                            DrawLine(RouteNodes[RouteNodes.Count - 1], RouteNodes[0], Color.Green);
                        }
                        else
                        {
                            cool = 0;
                            DrawLine(RouteNodes[RouteNodes.Count - 1], RouteNodes[0], Color.Red);
                        }
                    }
                    if (cool < 1 && last.DistanceTo(RouteNodes[0]) < 20f)
                    {
                        World.DrawMarker(MarkerType.DebugSphere, last + new Vector3(0, 0, 0.2f), Vector3.Zero, Vector3.Zero, new Vector3(0.5f, 0.5f, 0.5f), Color.Blue);
                        World.DrawMarker(MarkerType.DebugSphere, RouteNodes[0] + new Vector3(0, 0, 0.2f), Vector3.Zero, Vector3.Zero, new Vector3(0.5f, 0.5f, 0.5f), Color.Black);
                        if (last.DistanceTo(RouteNodes[0]) > 2f)
                        {
                            DrawLine(last + new Vector3(0, 0, 0.2f), RouteNodes[0] + new Vector3(0, 0, 0.2f), Color.Red);
                        }
                        else
                        {
                            DrawLine(last + new Vector3(0, 0, 0.2f), RouteNodes[0] + new Vector3(0, 0, 0.2f), Color.Green);
                        }
                    }
                }
            }
            // Draw the pending section until the circuit closes.
            if (cool < 1) DrawSection(_routeSection, EditNodeHalfWidths);

            
            // Route editing is available only from freecam.
            if (_routeEditorActive && _freeCam.IsActive)
            {
                RaycastResult ray = World.Raycast(GameplayCamera.Position, GameplayCamera.Position + ((GameplayCamera.Direction.Normalized) * 100), IntersectOptions.Everything);

                // Width is the only knob the geometry leaves open: the arc always runs to the aim point, so
                // the old reach multiplier has nothing left to scale. The floor is applied after adjusting,
                // or a single tap to the minimum commits a zero-width track.
                if (Game.IsControlJustPressed(2, GTA.Control.NextWeapon)) _pathWidth--;
                if (Game.IsControlJustPressed(2, GTA.Control.PrevWeapon)) _pathWidth++;
                if (_pathWidth < 1) _pathWidth = 1;
                
                if (RouteNodes.Count > 0)
                {
                    if (cool == -1) DisplayHelpTextThisFrame("Create the rest of the route. ~n~- Looped: ~b~Circuit~n~~w~- Open: ~b~Point to Point");
                    if (cool == 0) DisplayHelpTextThisFrame("Close the circuit near the ~b~Start Line.");
                    if (cool == 1) DisplayHelpTextThisFrame("~g~The circuit is closed.");


                    
                    // Aim removes one node; Sprint + Aim removes up to ten.
                    if (Game.IsControlJustPressed(2, GTA.Control.Aim))
                    {
                        if (RouteNodes.Count > 2)
                        {
                            if (Game.IsControlPressed(2, GTA.Control.Sprint))
                            {
                                int i = 0;
                                while (i < 10)
                                {
                                    if (RouteNodes.Count == 0) break;
                                    RouteNodes.RemoveAt(RouteNodes.Count - 1);
                                    i++;
                                }
                            }
                            else
                            {
                                RouteNodes.RemoveAt(RouteNodes.Count - 1);
                            }
                        }
                        else
                        {
                            RouteNodes.Clear();
                            return;
                        }
                    }

                    
                    if (Game.IsControlJustPressed(2, GTA.Control.Attack) && cool < 1)
                    {
                        for (int i = 1; i < _routeSection.Count; i++)
                        {
                            RouteNodes.Add(_routeSection[i]);
                        }
                    }
                }
                else 
                {
                    DisplayHelpTextThisFrame("Place the ~b~Start Line.");
                }



                
                if (RouteNodes.Count > 1)
                {
                    if (ray.DitHitAnything && cool < 1)
                    {
                        
                        World.DrawMarker(MarkerType.DebugSphere, ray.HitCoords, Vector3.Zero, -Vector3.WorldDown, new Vector3(0.25f, 0.25f, 0.25f), Color.Blue);

                        Vector3 sStart = RouteNodes[RouteNodes.Count - 1];
                        Vector3 sDirection = (RouteNodes[RouteNodes.Count - 1] - RouteNodes[RouteNodes.Count - 2]).Normalized;
                        Vector3 sEnd = ray.HitCoords;

                        List<Vector3> temporaryRouteNodes = GenerateArc(sStart, sDirection, sEnd);

                        foreach (Vector3 p in temporaryRouteNodes)
                        {
                            World.DrawMarker(MarkerType.DebugSphere, p, Vector3.Zero, -Vector3.WorldDown, new Vector3(0.25f, 0.25f, 0.25f), Color.Blue);
                        }
                        _routeSection.Clear();
                        _routeSection.AddRange(temporaryRouteNodes);



                        EditNodeHalfWidths.Clear();
                        for (int d = 0; d < _routeSection.Count - 1; d++)
                        {
                            EditNodeHalfWidths.Add(d, _pathWidth);
                        }
                        if (_routeSection.Count > 4)
                        {
                            Vector3 aim = Vector3.Lerp(_routeSection[_routeSection.Count - 2] + new Vector3(0, 0, 0.5f), _routeSection[_routeSection.Count - 1] + new Vector3(0, 0, 0.5f), 10f);

                            DrawLine(_routeSection[_routeSection.Count - 2] + new Vector3(0, 0, 0.5f), aim, Color.Blue);
                        }
                    }
                }
                else
                {

                    World.DrawMarker(MarkerType.ChevronUpx3, ray.HitCoords, FreeCamRide.ForwardVector, new Vector3(-90, 0, 0), new Vector3(1, 1, 1), Color.Blue);
                    World.DrawMarker(MarkerType.ChevronUpx3, ray.HitCoords, FreeCamRide.ForwardVector, new Vector3(-90, 0, 0), new Vector3(1, 1, 1), Color.Blue);
                    DrawLine(ray.HitCoords, ray.HitCoords - (FreeCamRide.ForwardVector * 10), Color.Blue);

                    Vector3 right = ray.HitCoords + (FreeCamRide.RightVector * _pathWidth);
                    Vector3 left = ray.HitCoords - (FreeCamRide.RightVector * _pathWidth);

                    World.DrawMarker(MarkerType.UpsideDownCone, right + new Vector3(0, 0, 1), FreeCamRide.ForwardVector, new Vector3(0, 0, 0), new Vector3(1, 1, 1), Color.Blue);
                    World.DrawMarker(MarkerType.UpsideDownCone, left + new Vector3(0, 0, 1), FreeCamRide.ForwardVector, new Vector3(0, 0, 0), new Vector3(1, 1, 1), Color.Blue);
                    DrawLine(left + new Vector3(0, 0, 0.05f), right + new Vector3(0, 0, 0.05f), Color.Blue);

                    if (Game.IsControlJustPressed(2, GTA.Control.Attack) && ray.DitHitAnything)
                    {
                        Vector3 p = ray.HitCoords;
                        RouteNodes.Add(p);
                        p = ray.HitCoords - (FreeCamRide.ForwardVector * 1);
                        RouteNodes.Add(p);
                        _bezierStartAnchor = (ray.HitCoords - (FreeCamRide.ForwardVector * 6));

                    }
                }

                // Keep one half-width entry for every route segment.
                for (int i = 0; i < RouteNodes.Count - 1; i++)
                {
                    if (NodeHalfWidths.Count - 1 < RouteNodes.Count - 1)
                    {
                        if (!NodeHalfWidths.ContainsKey(i))
                        {
                            NodeHalfWidths.Add(i, _pathWidth);
                        }
                    }
                }

                while (NodeHalfWidths.Count - 1 > RouteNodes.Count - 1)
                {
                    NodeHalfWidths.Remove(NodeHalfWidths.Count - 1);
                }

            }
        }

        // One constant-radius section: a circular arc running from the route end to the aim point, leaving
        // that end tangent to the incoming heading. Plan-view constant radius specifically, because that is
        // the plane the racing system measures - Circumradius3D drops Z - so this reads back as one radius
        // per section instead of the graded one a quadratic Bezier produced.
        public static List<Vector3> GenerateArc(Vector3 sStart, Vector3 sDirection, Vector3 sEnd)
        {
            const float separationDist = 1f;
            // Past this the centre is so distant that building the points loses more precision than the
            // curvature is worth, so the section is simply straight.
            const float straightRadiusFactor = 100f;
            const float halfPi = 1.5707964f;
            // The aim is a raycast hit, so the arc is short; this only bounds a pathological caller.
            const int maxSteps = 4000;

            List<Vector3> points = new List<Vector3>();
            Vector3 flat = new Vector3(sEnd.X - sStart.X, sEnd.Y - sStart.Y, 0f);
            Vector3 heading = new Vector3(sDirection.X, sDirection.Y, 0f);

            float chord = flat.Length();
            if (chord < 0.01f || heading.LengthSquared() < 0.0001f)
            {
                points.Add(sEnd);
                return points;
            }
            heading.Normalize();

            // The tangent-chord theorem puts the chord at half the arc's central angle off the entry
            // tangent, so the sweep is twice the aim's bearing. Past 90 degrees the aim is behind the
            // heading, which no forward-tangent arc can reach: hold a semicircle, so the preview shows the
            // aim is out of range rather than silently doubling the route back on itself.
            float bearing = (float)Math.Atan2((heading.X * flat.Y) - (heading.Y * flat.X), (heading.X * flat.X) + (heading.Y * flat.Y));
            if (bearing > halfPi) bearing = halfPi;
            if (bearing < -halfPi) bearing = -halfPi;
            float sweep = bearing * 2f;

            float radius = chord / (2f * (float)Math.Sin(Math.Abs(sweep) / 2f));

            if (!float.IsInfinity(radius) && radius <= chord * straightRadiusFactor)
            {
                float sign = Math.Sign(sweep);
                Vector3 centre = sStart + new Vector3(-heading.Y * sign, heading.X * sign, 0f) * radius;
                float startAngle = (float)Math.Atan2(sStart.Y - centre.Y, sStart.X - centre.X);
                int steps = (int)Math.Min(maxSteps, Math.Max(1.0, Math.Ceiling(radius * Math.Abs(sweep) / separationDist)));

                // Index 0 is the route end itself: the apply loop starts at 1, so the node already in
                // RouteNodes is not written twice.
                points.Add(sStart);
                for (int i = 1; i <= steps; i++)
                {
                    float angle = startAngle + (sweep * i / steps);
                    points.Add(new Vector3(centre.X + (radius * (float)Math.Cos(angle)), centre.Y + (radius * (float)Math.Sin(angle)), sStart.Z));
                }
            }
            else
            {
                int straightSteps = (int)Math.Min(maxSteps, Math.Max(1.0, Math.Ceiling(chord / separationDist)));
                points.Add(sStart);
                for (int i = 1; i <= straightSteps; i++) points.Add(sStart + (flat * (i / (float)straightSteps)));
            }

            // Follow the terrain: the arc is computed in plan, so every point below the route end takes its
            // height from the map. Sprint holds the section level, as it always did.
            if (!Game.IsControlPressed(2, GTA.Control.Sprint))
            {
                for (int i = 1; i < points.Count; i++)
                {
                    float ground;
                    if (TryResolveGround(points[i], out ground)) points[i] = new Vector3(points[i].X, points[i].Y, ground);
                }
            }
            return points;
        }

        // Ground height directly under a plan-view point. The probe starts well above the point rather than
        // just over it, because a probe anchored below the surface finds nothing at all and the point then
        // keeps a stale height - so a camera under the map, or a road rising toward the point, could never
        // pull it up. The hit is absolute, so the probe's offset needs no correction back out.
        // A single probe from that high would return the topmost surface, which over a road is a bridge
        // deck; so when the first hit is nowhere near the expected height, keep walking down through the
        // stacked surfaces and take the one nearest it. The common case accepts the first hit and stops.
        static bool TryResolveGround(Vector3 point, out float ground)
        {
            const float probeUpMeters = 50f;
            const float probeDownMeters = 30f;
            const float acceptWithinMeters = 3f;
            const int maxSurfaces = 4;

            ground = point.Z;
            float ceiling = point.Z + probeUpMeters;
            float floorHeight = point.Z - probeDownMeters;
            bool found = false;

            for (int pass = 0; pass < maxSurfaces; pass++)
            {
                RaycastResult hit = World.Raycast(new Vector3(point.X, point.Y, ceiling), new Vector3(point.X, point.Y, floorHeight), IntersectOptions.Map);
                if (!hit.DitHitAnything) break;
                if (Math.Abs(hit.HitCoords.Z - point.Z) < acceptWithinMeters)
                {
                    ground = hit.HitCoords.Z;
                    return true;
                }
                if (!found || Math.Abs(hit.HitCoords.Z - point.Z) < Math.Abs(ground - point.Z)) ground = hit.HitCoords.Z;
                found = true;

                ceiling = hit.HitCoords.Z - 0.05f;
                if (ceiling <= floorHeight) break;
            }
            return found;
        }

        public bool PlayerOrCameraNearPos(Vector3 pos, float dist)
        {
            if (_freeCam.IsActive) return Game.Player.Character.Position.DistanceTo(pos) < dist;
            else return World.RenderingCamera.Position.DistanceTo(pos) < dist;


        }

        public void DrawRouteNodes(List<Vector3> nodes, Dictionary<int, float> widedict, int fidelity)
        {
            if (nodes.Count == 0) return;
            int closestnode = ClosestNodeToPlace(Game.Player.Character.Position, nodes);
            Vector3 oldpos = Vector3.Zero;

            int start = closestnode - 50;
            int end = closestnode + 50;
            int countmax = 1;
            int count = 0;
            int dd = 0;
            Vector3 pos = Vector3.Zero;
            Vector3 lastline = Vector3.Zero;
            if (start < 0) start = 0;
            if (end > nodes.Count - 1) end = nodes.Count - 1;
            dd = start;


            if (_routeEditorActive) World.DrawMarker(MarkerType.CheckeredFlagRect, nodes[0] + new Vector3(0, 0, 3f), (nodes[1] - nodes[0]).Normalized, new Vector3(0, 0, 0), new Vector3(5f, 5f, 5f), Color.White);

            for (int ph = start; ph < end; ph += 1)
            {

                    pos = nodes[ph];
                    float w = 0f;
                    float oldw = 0f;
                    if (widedict != null)
                    {
                        if (widedict.ContainsKey(dd)) w = widedict[dd];
                        if (widedict.ContainsKey(dd - 1)) oldw = widedict[dd - 1]; else oldw = w;
                    }

                    if (oldpos == Vector3.Zero) oldpos = nodes[nodes.Count - 1];


                    if (oldpos != Vector3.Zero && PlayerOrCameraNearPos(nodes[ph], 120))
                    {

                            Vector3 rWidepos = GetPerpendicular(pos, oldpos, w, true);
                            Vector3 lWidepos = GetPerpendicular(pos, oldpos, w, false);

                            Vector3 oldrWidepos = GetPerpendicular(pos, oldpos, oldw, true) - (pos - oldpos);
                            Vector3 oldlWidepos = GetPerpendicular(pos, oldpos, oldw, false) - (pos - oldpos);




                            Color col = Color.Green;



                            

                            if (w != 0f)
                            {

                                if (_routeEditorActive)
                                {

                                    if (ph == end - 1)
                                    {

                                        
                                        
                                        

                                    }
                                    else
                                        if (pos != nodes[0] && ph % 5 == 0)
                                        {


                                            World.DrawMarker(MarkerType.DebugSphere, lWidepos, new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), Color.Blue);
                                            World.DrawMarker(MarkerType.DebugSphere, rWidepos, new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), Color.Blue);
                                                                                                                                                                                      



                                        }
                                }

                                col.ToArgb();

                                Color chevcolor = Color.FromArgb(50, col);

                                


                            }
                        }

                oldpos = pos;

                dd++;
            }
        }


        public void DrawSection(List<Vector3> nodes, Dictionary<int, float> widedict)
        {
            if (nodes.Count == 0) return;
            int closestnode = ClosestNodeToPlace(Game.Player.Character.Position, nodes);
            Vector3 oldpos = Vector3.Zero;

            int start = closestnode - 100;
            int end = closestnode + 100;
            int countmax = 1;
            int count = 0;
            int dd = 0;
            Vector3 pos = Vector3.Zero;
            Vector3 lastline = Vector3.Zero;
            if (start < 0) start = 0;
            if (end > nodes.Count - 1) end = nodes.Count - 1;
            dd = start;
            for (int ph = start; ph < end; ph += 1)
            {

                    pos = nodes[ph];
                    float w = 0f;
                    float oldw = 0f;
                    if (widedict != null)
                    {
                        if (widedict.ContainsKey(dd)) w = widedict[dd];
                        if (widedict.ContainsKey(dd - 1)) oldw = widedict[dd - 1]; else oldw = w;
                    }

                    if (oldpos == Vector3.Zero) oldpos = nodes[nodes.Count - 1];


                    if (oldpos != Vector3.Zero && PlayerOrCameraNearPos(nodes[ph], 120))
                    {

                            Vector3 rWidepos = GetPerpendicular(pos, oldpos, w, true);
                            Vector3 lWidepos = GetPerpendicular(pos, oldpos, w, false);

                            Vector3 oldrWidepos = GetPerpendicular(pos, oldpos, oldw, true) - (pos - oldpos);
                            Vector3 oldlWidepos = GetPerpendicular(pos, oldpos, oldw, false) - (pos - oldpos);


                            Color col = Color.Green;


                            


                            if (w != 0f)
                            {

                                if (_routeEditorActive)
                                {
                                    

                                    if (ph == end - 1)
                                    {

                                        DrawLine(lWidepos + new Vector3(0, 0, 0.5f), rWidepos + new Vector3(0, 0, 0.5f), Color.Blue);
                                        World.DrawMarker(MarkerType.DebugSphere, lWidepos + new Vector3(0, 0, 0.5f), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0.3f, 0.3f, 0.3f), Color.Blue);
                                        World.DrawMarker(MarkerType.DebugSphere, rWidepos + new Vector3(0, 0, 0.5f), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0.3f, 0.3f, 0.3f), Color.Blue);

                                    }
                                    else
                                        if (pos != nodes[0] && ph % 6 == 0)
                                        {


                                            World.DrawMarker(MarkerType.DebugSphere, lWidepos + new Vector3(0f, 0f, 0.5f), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), Color.Green);
                                            World.DrawMarker(MarkerType.DebugSphere, rWidepos + new Vector3(0f, 0f, 0.5f), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), Color.Green);
                                            DrawLine(lWidepos + new Vector3(0f, 0f, 0.5f), rWidepos + new Vector3(0f, 0f, 0.5f), Color.Green);



                                        }
                                }
                            }
                        }

                oldpos = pos;

                dd++;
            }
        }
        Vector3 GetPerpendicular(Vector3 a, Vector3 b, float length, bool clockwise)
        {
            Vector3 ab = (b - a).Normalized;
            Vector3 abCw = Vector3.Zero;
            if (clockwise)
            {
                abCw.X = -ab.Y;
                abCw.Y = ab.X;
            }
            else
            {
                abCw.X = ab.Y;
                abCw.Y = -ab.X;
            }
            return a + abCw * length;
        }
    }
}
