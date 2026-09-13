using GTA;
using GTA.Native;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace ARS
{
    public static class VehicleCatalog
    {
        // The roster is one flat file: one model key per line, blank lines and # comments ignored.
        // A key is a model name ("sabregt") or a hash; BuildPowerCache canonicalises both to hash keys.
        public static string RosterPath { get { return ARS.ScriptsFolder + @"\Vehicles\cars.txt"; } }

        // Discovery only — no natives, so it is safe on the background load thread.
        public static void FillPool(List<string> pool)
        {
            pool.Clear();
            if (!File.Exists(RosterPath)) return;
            HashSet<string> seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (string line in File.ReadAllLines(RosterPath))
            {
                string entry = line.Trim();
                if (entry.Length == 0 || entry[0] == '#') continue;
                if (seen.Add(entry)) pool.Add(entry);
            }
        }

        // Called on the main script thread (NOT from the background load task).        // GTA natives are not safe to call off the main thread, so the stat reads live here. The pool
        // arrives holding the raw roster lines and is rewritten in place to canonical hash-string keys,
        // which is what the pace cache and the grid selection are keyed by; unresolvable lines are dropped.
        public static void BuildPowerCache(List<string> pool, Dictionary<string, float> gripByModel, Dictionary<string, float> topSpeedMphByModel, Dictionary<string, float> accelByModel, Dictionary<string, bool> electricByModel, Dictionary<string, string> nameByModel, HashSet<VehicleClass> blacklistedClasses, Action<string> log)
        {
            gripByModel.Clear();
            topSpeedMphByModel.Clear();
            accelByModel.Clear();
            electricByModel.Clear();
            nameByModel.Clear();
            int cached = 0;
            HashSet<string> seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            for (int i = 0; i < pool.Count; i++)
            {
                string entry = pool[i];
                int hash;
                if (!int.TryParse(entry, out hash)) hash = Game.GenerateHash(entry);
                if (hash == 0) { pool[i] = null; continue; }
                string key = hash.ToString();
                pool[i] = key;
                if (!seen.Add(key)) continue;
                Model model = new Model(hash);
                if (!model.IsCar || !model.IsValid) continue;
                try
                {
                    VehicleClass vehicleClass = (VehicleClass)Function.Call<int>(Hash.GET_VEHICLE_CLASS_FROM_NAME, model.Hash);
                    if (blacklistedClasses.Contains(vehicleClass)) continue;
                    float grip = Function.Call<float>((Hash)0x539DE94D44FDFD0D, model.Hash);
                    float topSpeedMph = ARS.MpsToMph(Function.Call<float>((Hash)0xF417C2502FFFED43, model.Hash));
                    float accel = Function.Call<float>(Hash.GET_VEHICLE_MODEL_ACCELERATION, model.Hash);
                    bool isElectric = Function.Call<int>((Hash)0xD839450756ED5A80, model.Hash) != 0;
                    gripByModel[key] = grip;
                    topSpeedMphByModel[key] = topSpeedMph;
                    accelByModel[key] = accel;
                    electricByModel[key] = isElectric;
                    // The model native hands back a GXT label ("SABREGT"); the readable name is the label
                    // resolved. An add-on the game knows nothing about falls back to its roster line.
                    string label = Function.Call<string>(Hash.GET_DISPLAY_NAME_FROM_VEHICLE_MODEL, model.Hash);
                    string friendly = string.IsNullOrWhiteSpace(label) ? null : Game.GetGXTEntry(label);
                    if (string.IsNullOrWhiteSpace(friendly) || string.Equals(friendly, label, StringComparison.OrdinalIgnoreCase)) friendly = null;
                    nameByModel[key] = friendly ?? entry;
                    cached++;
                }
                catch (Exception) { }
            }
            pool.RemoveAll(string.IsNullOrEmpty);
            log("BuildPowerCache: cached grip+topspeed+power for " + cached + " road cars.");
        }

        // A model key is a name ("sabregt") or a hash; both canonicalise to the same hash string.
        static string CanonicalKey(string entry)
        {
            int hash;
            if (!int.TryParse(entry, out hash)) hash = Game.GenerateHash(entry);
            return hash == 0 ? null : hash.ToString();
        }

        // The build cheats only ever add lines that are missing, so a hand-edited roster survives a rebuild.
        // Returns how many lines were added. Requires natives, so it belongs on the main thread.
        public static int AddToRoster(IEnumerable<string> keys)
        {
            HashSet<string> known = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            if (File.Exists(RosterPath))
            {
                foreach (string line in File.ReadAllLines(RosterPath))
                {
                    string entry = line.Trim();
                    if (entry.Length == 0 || entry[0] == '#') continue;
                    string canonical = CanonicalKey(entry);
                    if (canonical != null) known.Add(canonical);
                }
            }
            List<string> added = new List<string>();
            foreach (string key in keys)
            {
                string entry = key.Trim().ToLowerInvariant();
                string canonical = entry.Length == 0 ? null : CanonicalKey(entry);
                if (canonical == null || !known.Add(canonical)) continue;
                added.Add(entry);
            }
            if (added.Count == 0) return 0;
            Directory.CreateDirectory(Path.GetDirectoryName(RosterPath));
            File.AppendAllLines(RosterPath, added);
            return added.Count;
        }
    }
}
