using GTA;
using GTA.Math;
using System.Collections.Generic;
using System.Drawing;

namespace ARS
{
    // The in-game track creator (route editor). Split out of AutosportRacingSystem.cs 2026-10; the code is
    // unchanged. DORMANT: _routeEditorActive has no writer that sets it true, so the editing branch below
    // never runs and no menu item, cheat or hotkey enters creator mode - the route-recording loop, the
    // section preview and the route-node visuals are all unreachable today. Kept for a later revival pass.
    public partial class ARS
    {
        public static Dictionary<int, float> EditNodeHalfWidths = new Dictionary<int, float>();
        static bool _routeEditorActive = false;
        List<Vector3> _routeSection = new List<Vector3>();
        Vector3 _bezierStartAnchor = Vector3.Zero;
        float _bezierScale = 1.5f;
        int _pathWidth = 5;

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
                if (_pathWidth < 1) _pathWidth = 1;
                if (_bezierScale < 5f) _bezierScale = 5f;
                RaycastResult ray = World.Raycast(GameplayCamera.Position, GameplayCamera.Position + ((GameplayCamera.Direction.Normalized) * 100), IntersectOptions.Everything);

                
                if (RouteNodes.Count > 0)
                {
                    if (cool == -1) DisplayHelpTextThisFrame("Create the rest of the route. ~n~- Looped: ~b~Circuit~n~~w~- Open: ~b~Point to Point");
                    if (cool == 0) DisplayHelpTextThisFrame("Close the circuit near the ~b~Start Line.");
                    if (cool == 1) DisplayHelpTextThisFrame("~g~The circuit is closed.");


                    
                    if (Game.IsControlJustPressed(2, GTA.Control.NextWeapon))
                    {
                        if (!Game.IsControlPressed(2, GTA.Control.Sprint)) _bezierScale -= 5f; else _pathWidth--;
                    }
                    if (Game.IsControlJustPressed(2, GTA.Control.PrevWeapon))
                    {
                        if (!Game.IsControlPressed(2, GTA.Control.Sprint)) _bezierScale += 5f; else _pathWidth++;
                    }
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

                    if (Game.IsControlJustPressed(2, GTA.Control.NextWeapon)) _pathWidth--;
                    if (Game.IsControlJustPressed(2, GTA.Control.PrevWeapon)) _pathWidth++;

                }



                
                if (RouteNodes.Count > 1)
                {
                    if (ray.DitHitAnything && cool < 1)
                    {
                        
                        World.DrawMarker(MarkerType.DebugSphere, ray.HitCoords, Vector3.Zero, -Vector3.WorldDown, new Vector3(0.25f, 0.25f, 0.25f), Color.Blue);

                        Vector3 sStart = RouteNodes[RouteNodes.Count - 1];
                        Vector3 sDirection = (RouteNodes[RouteNodes.Count - 1] - RouteNodes[RouteNodes.Count - 2]).Normalized;
                        Vector3 sEnd = ray.HitCoords;
                        float sScale = sStart.DistanceTo(sEnd) * 0.5f;

                        List<Vector3> temporaryRouteNodes = GenerateBezier(sStart, sDirection, sEnd, sScale);

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

        // Generate evenly spaced curve points from the route end to the raycast target.
        public static List<Vector3> GenerateBezier(Vector3 sStart, Vector3 sDirection, Vector3 sEnd, float sScale)
        {
            List<Vector3> points = new List<Vector3>();
            sScale = sStart.DistanceTo2D(sEnd) * Remap(Vector3.Angle((sEnd - sStart).Normalized, sDirection), 0f, 90f, 1f, 1.5f, true);


            Vector3 middlePoint = sStart + (sDirection * (sScale / 2));
            Vector3 directionStart = sStart - middlePoint;
            Vector3 directionEnd = ((sEnd) - middlePoint).Normalized * (sScale / 2f);

            
            

            float separationDist = 1f;

            float addition = (1 / directionEnd.DistanceTo(directionStart));

            float stepLerp = 0;
            int step = 0;
            float scaleAdjust = 0;
            while (stepLerp < 1.0f && step < 400)
            {
                step++;

                scaleAdjust = 0f;

                Vector3 currentPos = middlePoint + Bezier2(directionStart, directionEnd, stepLerp);
                Vector3 addPos = middlePoint + Bezier2(directionStart, directionEnd, stepLerp + addition);

                int tries = 0;
                while (currentPos.DistanceTo2D(addPos) < separationDist - 0.001f && tries < 200)
                {
                    tries++;
                    scaleAdjust += 0.001f;
                    addPos = middlePoint + Bezier2(directionStart, directionEnd, stepLerp + addition + scaleAdjust);
                }
                tries = 0;
                while (currentPos.DistanceTo2D(addPos) > separationDist + 0.001f && tries < 200)
                {
                    tries++;
                    scaleAdjust -= 0.001f;
                    addPos = middlePoint + Bezier2(directionStart, directionEnd, stepLerp + addition + scaleAdjust);
                }
                stepLerp += addition + scaleAdjust;

                
                
                if (!Game.IsControlPressed(2, GTA.Control.Sprint))
                {
                    RaycastResult toGround = World.Raycast(addPos + new Vector3(0, 0, 2f), addPos + (Vector3.WorldDown * 30f), IntersectOptions.Map);
                    if (toGround.DitHitAnything) addPos.Z = toGround.HitCoords.Z;
                }
                points.Add(addPos);

            }
            return points;
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

        public static Vector3 Bezier2(Vector3 Start, Vector3 End, float t)
        {
            return (((1 - t) * (1 - t)) * Start) + (2 * t * (1 - t) * Vector3.Zero) + ((t * t) * End);
        }
    }
}
