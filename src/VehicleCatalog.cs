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
        // Every supplier XML in the pool: Vehicles\ and its first-level subfolders.
        static IEnumerable<string> PoolFiles()
        {
            List<string> folders = Directory.GetDirectories(ARS.ScriptsFolder + @"\Vehicles").ToList();
            folders.Add(ARS.ScriptsFolder + @"\Vehicles");
            foreach (string folder in folders)
                foreach (string path in Directory.EnumerateFiles(folder))
                    yield return path;
        }

        // Discovery only — no natives, so it is safe on the background load thread.
        public static void FillPool(List<string> pool)
        {
            pool.Clear();
            foreach (string path in PoolFiles()) pool.Add(path);
        }

        // Called on the main script thread (NOT from the background load task).
        // GTA natives are not safe to call off the main thread, so we fill the
        // modelName -> power cache here rather than during the pool scan.
        public static void BuildPowerCache(List<string> pool, Dictionary<string, float> gripByModel, Dictionary<string, float> topSpeedMphByModel, Dictionary<string, float> accelByModel, Dictionary<string, bool> electricByModel, HashSet<VehicleClass> blacklistedClasses, Action<string> log)
        {
            gripByModel.Clear();
            topSpeedMphByModel.Clear();
            accelByModel.Clear();
            electricByModel.Clear();
            int cached = 0;
            HashSet<string> seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (string path in pool)
            {
                string modelText = TrackRepository.ReadVehicleModel(path);
                if (string.IsNullOrWhiteSpace(modelText) || !seen.Add(modelText)) continue;
                int hash;
                if (!int.TryParse(modelText, out hash)) continue;
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
                    gripByModel[modelText] = grip;
                    topSpeedMphByModel[modelText] = topSpeedMph;
                    accelByModel[modelText] = accel;
                    electricByModel[modelText] = isElectric;
                    cached++;
                }
                catch (Exception) { }
            }
            log("BuildPowerCache: cached grip+topspeed+power for " + cached + " road cars.");
        }
    }
}
