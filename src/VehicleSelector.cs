using System;
using System.Collections.Generic;
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

        static void Shuffle(List<XmlDocument> candidates, Func<int, int, int> random)
        {
            for (int i = candidates.Count - 1; i > 0; i--)
            {
                int index = random(0, i);
                XmlDocument item = candidates[i];
                candidates[i] = candidates[index];
                candidates[index] = item;
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
