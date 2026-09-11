using System;
using System.Collections.Generic;
using System.Linq;
using System.Xml;

namespace ARS
{
    public static class VehicleSelector
    {
        public static List<XmlDocument> Select(Dictionary<string, string> files, Dictionary<string, float> paceIndex, int maxCars, bool allowDuplicates, bool allowYield, Action yield, Func<int, int, int> random, Action<string> log, float powerTarget, float powerBracket)
        {
            List<XmlDocument> candidates = new List<XmlDocument>();
            int cooldown = 0;
            foreach (string path in files.Keys)
            {
                string model = TrackRepository.ReadVehicleModel(path);
                float pace;
                if (!string.IsNullOrWhiteSpace(model) && paceIndex.TryGetValue(model, out pace) && Math.Abs(pace - powerTarget) <= powerBracket)
                {
                    try { XmlDocument document = new XmlDocument(); document.Load(path); candidates.Add(document); }
                    catch (Exception) { }
                }
                if (allowYield && ++cooldown > 20) { cooldown = 0; yield(); }
            }

            log("Pace-matched candidates: " + candidates.Count);
            Shuffle(candidates, random);
            if (candidates.Count > maxCars) candidates.RemoveRange(maxCars, candidates.Count - maxCars);
            return candidates;
        }

        // One pool pass: rank every paced vehicle by pace distance to the target, take the closest maxCars.
        public static List<XmlDocument> SelectClosestByPace(Dictionary<string, string> files, Dictionary<string, float> paceIndex, int maxCars, bool allowYield, Action yield, Func<int, int, int> random, Action<string> log, float powerTarget)
        {
            List<KeyValuePair<float, string>> ranked = new List<KeyValuePair<float, string>>();
            int cooldown = 0;
            foreach (string path in files.Keys)
            {
                string model = TrackRepository.ReadVehicleModel(path);
                float pace;
                if (!string.IsNullOrWhiteSpace(model) && paceIndex.TryGetValue(model, out pace))
                    ranked.Add(new KeyValuePair<float, string>(Math.Abs(pace - powerTarget), path));
                if (allowYield && ++cooldown > 20) { cooldown = 0; yield(); }
            }

            Shuffle(ranked, random);
            List<KeyValuePair<float, string>> ordered = ranked.OrderBy(pair => pair.Key).ToList();

            List<XmlDocument> candidates = new List<XmlDocument>();
            foreach (KeyValuePair<float, string> pair in ordered)
            {
                if (candidates.Count >= maxCars) break;
                try { XmlDocument document = new XmlDocument(); document.Load(pair.Value); candidates.Add(document); }
                catch (Exception) { }
            }

            log("Closest-pace candidates: " + candidates.Count);
            Shuffle(candidates, random);
            return candidates;
        }

        // Temp: bypass pace matching and XML lookup — create minimal XML docs from model names directly.
        // Used for hardcoded roster testing — bypasses pace matching for the full grid.
        public static List<XmlDocument> SelectHardcoded(List<string> roster, Dictionary<string, string> files, int maxCars, bool allowDuplicates, Action yield, Func<int, int, int> random, Action<string> log)
        {
            List<XmlDocument> candidates = new List<XmlDocument>();
            HashSet<string> rosterSet = new HashSet<string>(roster, StringComparer.OrdinalIgnoreCase);
            foreach (string modelName in rosterSet)
            {
                try
                {
                    XmlDocument doc = new XmlDocument();
                    XmlElement root = doc.CreateElement("Vehicle");
                    doc.AppendChild(root);
                    XmlElement modelNode = doc.CreateElement("Model");
                    modelNode.InnerText = modelName;
                    root.AppendChild(modelNode);
                    candidates.Add(doc);
                }
                catch (Exception) { }
            }
            log("Hardcoded roster candidates: " + candidates.Count);
            Shuffle(candidates, random);
            if (candidates.Count > maxCars) candidates.RemoveRange(maxCars, candidates.Count - maxCars);
            return candidates;
        }

        static void Shuffle<T>(List<T> list, Func<int, int, int> random)
        {
            for (int i = list.Count - 1; i > 0; i--)
            {
                int index = random(0, i);
                T item = list[i];
                list[i] = list[index];
                list[index] = item;
            }
        }

        static XmlDocument CreateDuplicate(XmlDocument candidate)
        {
            XmlDocument copy = (XmlDocument)candidate.Clone();
            XmlNode colors = copy.SelectSingleNode("//Colors");
            if (colors != null) colors.RemoveAll();
            XmlNode mods = copy.SelectSingleNode("//Mods");
            XmlNode vehicle = copy.SelectSingleNode("//Vehicle");
            if (mods != null && vehicle != null) vehicle.RemoveChild(mods);
            return copy;
        }
    }
}
