using GTA;
using GTA.Math;
using GTA.Native;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Xml;

namespace ARS
{
    public static class TrackBuilder
    {
        public static bool Build(List<Vector3> nodes, Dictionary<int, float> widths, XmlDocument trackFile, bool isPointToPoint, int opponents, List<Prop> trackLimits, List<int> flareEffects, List<Vector3> gridPositions)
        {
            gridPositions.Clear();
            if (nodes.Count == 0) return isPointToPoint;
            isPointToPoint |= nodes[0].DistanceTo(nodes[nodes.Count - 1]) > 20f;
            Color flareColor = ReadFlareColor(trackFile);

            if (trackLimits.Count == 0) SpawnStartGates(nodes, widths, isPointToPoint, opponents, trackLimits, flareEffects, flareColor);
            if (isPointToPoint) BuildPointToPointGrid(nodes, widths, gridPositions);
            else BuildCircuitGrid(nodes, widths, gridPositions);
            Function.Call(Hash.CLEAR_FOCUS);
            return isPointToPoint;
        }

        static void SpawnStartGates(List<Vector3> nodes, Dictionary<int, float> widths, bool isPointToPoint, int opponents, List<Prop> limits, List<int> effects, Color color)
        {
            Vector3 start = isPointToPoint ? nodes[8 + opponents * 3] : nodes[0];
            SpawnGatePair(start, nodes[1] - nodes[0], widths[0], limits, effects, color);
            if (isPointToPoint) SpawnGatePair(nodes.Last(), nodes[nodes.Count - 2] - nodes.Last(), widths[0], limits, effects, color);
        }

        static void SpawnGatePair(Vector3 position, Vector3 direction, float width, List<Prop> limits, List<int> effects, Color color)
        {
            for (int side = -1; side <= 1; side += 2)
            {
                Prop gate = World.CreateProp("prop_flare_01b", position + new Vector3(0f, 0f, 0.05f), new Vector3(0, 0, direction.Normalized.ToHeading() + 90f), false, false);
                gate.Position += gate.RightVector * width * side;
                AttachFlare(gate, color, effects);
                gate.FreezePosition = true;
                gate.HasCollision = false;
                limits.Add(gate);
            }
        }

        static void BuildPointToPointGrid(List<Vector3> nodes, Dictionary<int, float> widths, List<Vector3> grid)
        {
            for (int i = 50; i > 1; i--)
            {
                Vector3 point = nodes[i * 3];
                Vector3 direction = (point - nodes[i * 3 - 1]).Normalized;
                grid.Add(point - Quaternion.RotationAxis(Vector3.WorldUp, (float)Math.PI / 180f * -90f) * (direction * ((i % 2 == 1) ? widths[0] * 0.45f : -widths[0] * 0.45f)));
            }
        }

        static void BuildCircuitGrid(List<Vector3> nodes, Dictionary<int, float> widths, List<Vector3> grid)
        {
            int reference = nodes.Count - 1;
            for (int i = nodes.Count - 1; i > 50 && grid.Count <= 40; i--)
            {
                if (i >= reference - 5) continue;
                Vector3 direction = (nodes[reference] - nodes[i]).Normalized;
                grid.Add(nodes[i] - Quaternion.RotationAxis(Vector3.WorldUp, (float)Math.PI / 180f * -90f) * (direction * ((grid.Count % 2 == 1) ? -widths[0] * 0.45f : widths[0] * 0.45f)));
                reference = i;
            }
        }

        static Color ReadFlareColor(XmlDocument trackFile)
        {
            string value = trackFile?.SelectSingleNode("//Trackside") is XmlElement element ? element.GetAttribute("Flares") : "";
            int rgb;
            return value.Length == 9 && int.TryParse(value, out rgb) ? Color.FromArgb(int.Parse(value.Substring(0, 3)), int.Parse(value.Substring(3, 3)), int.Parse(value.Substring(6, 3))) : Color.GreenYellow;
        }

        static void AttachFlare(Prop prop, Color color, List<int> effects)
        {
            int tries = 0;
            while (!Function.Call<bool>(Hash.HAS_NAMED_PTFX_ASSET_LOADED, "scr_apartment_mp") && tries++ < 2000) Function.Call(Hash.REQUEST_NAMED_PTFX_ASSET, "scr_apartment_mp");
            if (!Function.Call<bool>(Hash.HAS_NAMED_PTFX_ASSET_LOADED, "scr_apartment_mp")) return;
            Function.Call(Hash._SET_PTFX_ASSET_NEXT_CALL, "scr_apartment_mp");
            int effect = Function.Call<int>(Hash.START_PARTICLE_FX_LOOPED_ON_ENTITY, "scr_finders_package_flare", prop, 0f, 0f, 0.1f, 0f, 0f, 0f, 1f, true, true, true);
            Function.Call(Hash.SET_PARTICLE_FX_LOOPED_ALPHA, effect, 1f);
            Function.Call(Hash.SET_PARTICLE_FX_LOOPED_COLOUR, effect, color.R / 255f, color.G / 255f, color.B / 255f, 1f, true);
            effects.Add(effect);
        }
    }
}
