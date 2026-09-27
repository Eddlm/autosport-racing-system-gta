using System;
using System.Collections.Generic;
using System.Linq;
using System.Xml;

namespace ARS
{
    public static class VehicleSelector
    {
        // **No script yield in this loop, deliberately.** A yield parks the whole race-start stack (track load ->
        // grid build -> ranking) inside SHVDN's per-script handshake across frames, the one window in ARS that can
        // strand a tick; the pass itself is pure C# over a few hundred entries and never needs one.
        public static List<string> SelectClosestByPace(List<string> pool, Dictionary<string, float> paceIndex, int maxCars, Func<int, int, int> random, Action<string> log, float powerTarget)
        {
            List<KeyValuePair<float, string>> ranked = new List<KeyValuePair<float, string>>();
            int inspected = 0;
            foreach (string key in pool)
            {
                float pace;
                if (paceIndex.TryGetValue(key, out pace))
                    ranked.Add(new KeyValuePair<float, string>(Math.Abs(pace - powerTarget), key));
                if (++inspected % 10 == 0) log("Ranking progress: " + inspected + "/" + pool.Count);
            }

            Shuffle(ranked, random);
            List<KeyValuePair<float, string>> ordered = ranked.OrderBy(pair => pair.Key).ToList();

            List<string> candidates = new List<string>();
            foreach (KeyValuePair<float, string> pair in ordered)
            {
                if (candidates.Count >= maxCars) break;
                candidates.Add(pair.Value);
            }

            log("Closest-pace candidates: " + candidates.Count);
            Shuffle(candidates, random);
            return candidates;
        }

        // Full-roster test bypass: no pace matching, the roster entries themselves become the grid.
        public static List<string> SelectHardcoded(List<string> roster, int maxCars, Func<int, int, int> random, Action<string> log)
        {
            List<string> candidates = new List<string>(new HashSet<string>(roster, StringComparer.OrdinalIgnoreCase));
            log("Hardcoded roster candidates: " + candidates.Count);
            Shuffle(candidates, random);
            if (candidates.Count > maxCars) candidates.RemoveRange(maxCars, candidates.Count - maxCars);
            return candidates;
        }

        static void Shuffle<T>(List<T> list, Func<int, int, int> random)
        {
            for (int i = list.Count - 1; i > 0; i--)
            {
                // Fisher-Yates needs the whole 0..i range, and random's max is exclusive, hence i + 1.
                int index = random(0, i + 1);
                T item = list[i];
                list[i] = list[index];
                list[index] = item;
            }
        }
    }
}
