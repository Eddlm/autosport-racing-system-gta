using GTA.Math;
using System.Collections.Generic;
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
