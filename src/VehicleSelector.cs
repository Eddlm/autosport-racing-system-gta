using System;
using System.Collections.Generic;
using System.Linq;
using System.Xml;

namespace ARS
{
    public static class VehicleSelector
    {
        // One pool pass: rank every paced model by pace distance to the target, take the closest maxCars.
        // The pool holds canonical hash-string keys, so no file is touched here.
        public static List<string> SelectClosestByPace(List<string> pool, Dictionary<string, float> paceIndex, int maxCars, bool allowYield, Action yield, Func<int, int, int> random, Action<string> log, float powerTarget)
        {
            List<KeyValuePair<float, string>> ranked = new List<KeyValuePair<float, string>>();
            int cooldown = 0;
            int inspected = 0;
            foreach (string key in pool)
            {
                float pace;
                if (paceIndex.TryGetValue(key, out pace))
                    ranked.Add(new KeyValuePair<float, string>(Math.Abs(pace - powerTarget), key));
                if (++inspected % 10 == 0) log("Ranking progress: " + inspected + "/" + pool.Count);
                if (allowYield && ++cooldown > 20) { cooldown = 0; yield(); }
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
        public static List<string> SelectHardcoded(List<string> roster, int maxCars, Action yield, Func<int, int, int> random, Action<string> log)
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
                int index = random(0, i);
                T item = list[i];
                list[i] = list[index];
                list[index] = item;
            }
        }
    }
}
