using GTA.Math;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Xml;

namespace ARS
{
    public static class TrackRepository
    {
        public static XmlDocument Load(string path)
        {
            XmlDocument document = new XmlDocument();
            document.Load(path);
            return document;
        }

        // Pure helper: compute the world position left of a track's start line by half its width.
        // Does not mutate global ARS state. For circuits the +/-5 indices wrap; for point-to-point they clamp.
        public static TrackStartInfo ComputeTrackStartInfo(string path)
        {
            XmlDocument document = Load(path);
            XmlNodeList points = document.SelectNodes("//Route/Point");
            int count = points.Count;
            if (count == 0) return null;

            List<Vector3> positions = new List<Vector3>(count);
            List<float> widths = new List<float>(count);
            foreach (XmlElement e in points)
            {
                float x = 0f, y = 0f, z = 0f, w = 10f;
                float.TryParse(e.SelectSingleNode("X")?.InnerText, NumberStyles.Float, CultureInfo.InvariantCulture, out x);
                float.TryParse(e.SelectSingleNode("Y")?.InnerText, NumberStyles.Float, CultureInfo.InvariantCulture, out y);
                float.TryParse(e.SelectSingleNode("Z")?.InnerText, NumberStyles.Float, CultureInfo.InvariantCulture, out z);
                float.TryParse(e.SelectSingleNode("Wide")?.InnerText, NumberStyles.Float, CultureInfo.InvariantCulture, out w);
                positions.Add(new Vector3(x, y, z));
                widths.Add(w);
            }

            bool isCircuit = positions[0].DistanceTo(positions[count - 1]) <= 20f;
            Func<int, int> clampOrWrap = node => isCircuit ? ((node % count) + count) % count : (int)ARS.Clamp(node, 0, count - 1);

            Vector3 start = positions[0];
            Vector3 ahead = positions[clampOrWrap(5)];
            Vector3 behind = positions[clampOrWrap(-5)];
            Vector3 forward = (ahead - behind).Normalized;
            forward.Z = 0f;
            Vector3 left = Vector3.Cross(Vector3.WorldUp, forward).Normalized;
            float fullWidth = Math.Max(widths[0], 2f);

            return new TrackStartInfo
            {
                TrackPath = path,
                StartPosition = start,
                JoinPosition = start + left * fullWidth
            };
        }

        public static List<string> ReadTrackTags(string path)
        {
            XmlDocument document = Load(path);
            List<string> tags = new List<string> { Path.GetFileName(path).ToLowerInvariant() };
            foreach (XmlNode node in document.SelectNodes("//Tags/*")) tags.Add(node.InnerText.ToLowerInvariant());
            return tags;
        }

        public static Vector3 ReadTrackStartPosition(string path)
        {
            XmlNode point = Load(path).SelectNodes("//Route/Point")[0];
            Vector3 position = Vector3.Zero;
            foreach (XmlNode node in point.ChildNodes)
            {
                float value = 0f;
                float.TryParse(node.InnerText.Replace('.', ','), out value);
                if (node.Name == "X") position.X = value;
                if (node.Name == "Y") position.Y = value;
                if (node.Name == "Z") position.Z = value;
            }
            return position;
        }

        public static List<string> ReadVehicleTags(string path)
        {
            XmlDocument document = Load(path);
            string folder = Path.GetDirectoryName(path).Split(Path.DirectorySeparatorChar).Last().ToLowerInvariant();
            List<string> tags = new List<string>();
            if (folder != "vehicles") tags.Add(folder);
            foreach (XmlNode node in document.SelectNodes("//Disciplines/*")) tags.Add(node.InnerText.ToLowerInvariant());
            return tags;
        }

        public static string ReadVehicleModel(string path)
        {
            try { return Load(path).SelectSingleNode("//Model")?.InnerText?.Trim(); }
            catch { return null; }
        }
    }
}
