using System;
using System.Collections.Generic;
using System.Xml;

namespace ARS
{
    public static class VehicleSelector
    {
        public static List<XmlDocument> Select(Dictionary<string, string> files, Dictionary<string, float> paceIndex, int maxCars, bool allowDuplicates, bool allowYield, Action yield, Func<int, int, int> random, Action<string> log, float powerTarget, float powerBracket, string alwaysIncludeModelName)
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
            // Pin AFTER shuffle so the pinned model is at the top before the trim — shuffle
            // can otherwise drift it toward the tail and have the maxCars trim drop it.
            PinModel(candidates, alwaysIncludeModelName, log, files);
            if (candidates.Count > maxCars) candidates.RemoveRange(maxCars, candidates.Count - maxCars);
            return candidates;
        }

        // Test-only hook: lift the named model out of the qualified pool to the front so the trim
        // can't drop it. If the pinned model isn't pace-qualified (e.g. mod cars with a non-numeric
        // <Model> string that never enters the pace cache), fall back to loading it directly from
        // the source files so the pin still applies.
        static void PinModel(List<XmlDocument> candidates, string pinName, Action<string> log, Dictionary<string, string> files)
        {
            if (string.IsNullOrWhiteSpace(pinName)) return;
            for (int i = 0; i < candidates.Count; i++)
            {
                string model = candidates[i].SelectSingleNode("//Model")?.InnerText?.Trim();
                if (!string.IsNullOrWhiteSpace(model) && string.Equals(model, pinName, StringComparison.OrdinalIgnoreCase))
                {
                    if (i != 0)
                    {
                        XmlDocument pinned = candidates[i];
                        candidates.RemoveAt(i);
                        candidates.Insert(0, pinned);
                    }
                    log("Pinned " + pinName + " into the grid (test override).");
                    return;
                }
            }

            // Not in the qualified pool — try the source files directly.
            foreach (string path in files.Keys)
            {
                string model = TrackRepository.ReadVehicleModel(path);
                if (string.IsNullOrWhiteSpace(model) || !string.Equals(model, pinName, StringComparison.OrdinalIgnoreCase)) continue;
                try
                {
                    XmlDocument document = new XmlDocument();
                    document.Load(path);
                    candidates.Insert(0, document);
                    log("Pinned " + pinName + " into the grid (test override, bypassed pace filter).");
                    return;
                }
                catch (Exception) { }
            }
            log("Always-include model '" + pinName + "' not found in vehicle pool; skipping.");
        }

        // Temp: bypass pace matching and load only the named models from the vehicle pool.
        // Used for hardcoded roster testing — mirrors the AlwaysIncludeModelName pin but for the full grid.
        public static List<XmlDocument> SelectHardcoded(List<string> roster, Dictionary<string, string> files, int maxCars, bool allowDuplicates, Action yield, Func<int, int, int> random, Action<string> log)
        {
            List<XmlDocument> candidates = new List<XmlDocument>();
            HashSet<string> rosterSet = new HashSet<string>(roster, StringComparer.OrdinalIgnoreCase);
            foreach (string path in files.Keys)
            {
                string model = TrackRepository.ReadVehicleModel(path);
                if (!string.IsNullOrWhiteSpace(model) && rosterSet.Contains(model))
                {
                    try { XmlDocument document = new XmlDocument(); document.Load(path); candidates.Add(document); }
                    catch (Exception) { }
                }
            }
            log("Hardcoded roster candidates: " + candidates.Count);
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
