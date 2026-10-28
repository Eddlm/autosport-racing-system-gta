// Picks the car for a grid slot: class, PI band and the driver's own choice first.
using System;
using System.Collections.Generic;
using System.Linq;
using System.Xml;

namespace ARS
{
    public static class VehicleSelector
    {

        public static List<string> SelectClosestByPace(List<string> pool, Dictionary<string, float> paceIndex, int maxCars, Func<int, int, int> random, Action<string> log, float powerTarget, int maxDeviation)
        {
            List<KeyValuePair<float, string>> ranked = new List<KeyValuePair<float, string>>();
            int inspected = 0;
            int outsideLimit = 0;
            foreach (string key in pool)
            {

                if (key == null) continue;
                float pace;
                if (paceIndex.TryGetValue(key, out pace))
                {

                    if (maxDeviation > 0 && Math.Abs(Math.Round(pace) - Math.Round(powerTarget)) > maxDeviation) outsideLimit++;
                    else ranked.Add(new KeyValuePair<float, string>(Math.Abs(pace - powerTarget), key));
                }
                if (++inspected % 10 == 0) log("Ranking progress: " + inspected + "/" + pool.Count);
            }
            if (maxDeviation > 0) log("PI limit " + maxDeviation + " around " + Math.Round(powerTarget) + ": " + ranked.Count + " of " + (ranked.Count + outsideLimit) + " paced models are inside it");

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

                int index = random(0, i + 1);
                T item = list[i];
                list[i] = list[index];
                list[index] = item;
            }
        }
    }
}
