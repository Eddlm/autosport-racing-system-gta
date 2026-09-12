using System;
using System.Collections.Generic;
using System.Linq;
using System.Xml;

namespace ARS
{
    public static class VehicleSelector
    {
        // One pool pass: rank every paced vehicle by pace distance to the target, take the closest maxCars.
        public static List<XmlDocument> SelectClosestByPace(List<string> pool, Dictionary<string, float> paceIndex, int maxCars, bool allowYield, Action yield, Func<int, int, int> random, Action<string> log, float powerTarget)
        {
            List<KeyValuePair<float, string>> ranked = new List<KeyValuePair<float, string>>();
            int cooldown = 0;
            foreach (string path in pool)
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
        public static List<XmlDocument> SelectHardcoded(List<string> roster, int maxCars, Action yield, Func<int, int, int> random, Action<string> log)
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
    }
}
