// The pool of cars the mod can spawn, indexed once from the roster files.
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

        public static string RosterFolder { get { return ARS.ScriptsFolder + @"\Vehicles"; } }
        public static string SelectedRosterFile { get; private set; }

        static VehicleCatalog() { SelectRosterFile("cars.txt"); }

        public static void SelectRosterFile(string fileName)
        {
            string name = (fileName ?? "").Trim();
            if (name.Length == 0 || !name.EndsWith(".txt", StringComparison.OrdinalIgnoreCase)) name = "cars.txt";
            SelectedRosterFile = name;
        }

        public static string PoolLabel(string fileName)
        {
            string name = (fileName ?? "").Trim();
            return name.EndsWith(".txt", StringComparison.OrdinalIgnoreCase) ? name.Substring(0, name.Length - 4) : name;
        }

        public static string PoolFileName(string label)
        {
            string name = (label ?? "").Trim();
            return name.EndsWith(".txt", StringComparison.OrdinalIgnoreCase) ? name : name + ".txt";
        }

        public static List<string> ListRosterFiles()
        {
            try
            {
                if (!Directory.Exists(RosterFolder)) return new List<string> { SelectedRosterFile };
                List<string> files = Directory.GetFiles(RosterFolder, "*.txt").Select(Path.GetFileName).OrderBy(f => f, StringComparer.OrdinalIgnoreCase).ToList();
                return files.Count > 0 ? files : new List<string> { SelectedRosterFile };
            }
            catch (Exception) { return new List<string> { SelectedRosterFile }; }
        }

        public static string RosterPath { get { return RosterFolder + @"\" + SelectedRosterFile; } }

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

        public static void BuildPowerCache(List<string> pool, Dictionary<string, float> gripByModel, Dictionary<string, float> topSpeedMphByModel, Dictionary<string, float> accelByModel, Dictionary<string, bool> electricByModel, Dictionary<string, string> nameByModel, HashSet<VehicleClass> blacklistedClasses, Action<string> log)
        {
            gripByModel.Clear();
            topSpeedMphByModel.Clear();
            accelByModel.Clear();
            electricByModel.Clear();
            nameByModel.Clear();

            bool canAskElectric = true;
            try { Function.Call<int>((Hash)0x1FCB07FE230B6639, new Model(VehicleHash.Adder).Hash); }
            catch (Exception)
            {
                canAskElectric = false;
                log("GET_IS_VEHICLE_ELECTRIC is unavailable on this game build - no car will be treated as electric.");
            }
            int cached = 0;
            HashSet<string> seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            for (int i = 0; i < pool.Count; i++)
            {
                string entry = pool[i];
                int hash;
                if (!int.TryParse(entry, out hash)) hash = Game.GenerateHash(entry);

                if (hash == 0) continue;
                string key = hash.ToString();
                pool[i] = key;
                if (!seen.Add(key)) continue;
                Model model = new Model(hash);
                if (!model.IsCar || !model.IsValid)
                {

                    log("  dropped " + entry + (model.IsInCdImage ? ": not a car" : ": not in the cdimage"));
                    continue;
                }
                try
                {
                    VehicleClass vehicleClass = (VehicleClass)Function.Call<int>(Hash.GET_VEHICLE_CLASS_FROM_NAME, model.Hash);
                    if (blacklistedClasses.Contains(vehicleClass))
                    {
                        log("  dropped " + entry + ": class " + vehicleClass + " is blacklisted");
                        continue;
                    }
                    float grip = Function.Call<float>((Hash)0x539DE94D44FDFD0D, model.Hash);
                    float topSpeedMph = ARS.MpsToMph(Function.Call<float>((Hash)0xF417C2502FFFED43, model.Hash));
                    float accel = Function.Call<float>(Hash.GET_VEHICLE_MODEL_ACCELERATION, model.Hash);
                    bool isElectric = canAskElectric && Function.Call<int>((Hash)0x1FCB07FE230B6639, model.Hash) != 0;
                    gripByModel[key] = grip;
                    topSpeedMphByModel[key] = topSpeedMph;
                    accelByModel[key] = accel;
                    electricByModel[key] = isElectric;

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

        static string CanonicalKey(string entry)
        {
            int hash;
            if (!int.TryParse(entry, out hash)) hash = Game.GenerateHash(entry);
            return hash == 0 ? null : hash.ToString();
        }

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
