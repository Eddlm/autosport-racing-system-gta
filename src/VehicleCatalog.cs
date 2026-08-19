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
        public static void Scan(Dictionary<string, string> tagsByFile, List<Model> knownModels, Action<string> log, Action<string> showLoading, Action yield, bool allowYield)
        {
            tagsByFile.Clear();
            knownModels.Clear();
            HashSet<string> seenModels = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            List<string> folders = Directory.GetDirectories(@"scripts\ARS\Vehicles").ToList();
            folders.Add(@"scripts\ARS\Vehicles");

            foreach (string folder in folders)
            {
                int count = 0;
                foreach (string path in Directory.EnumerateFiles(folder))
                {
                    List<string> tags = TrackRepository.ReadVehicleTags(path);
                    string model = TrackRepository.ReadVehicleModel(path);
                    log(path + " - [" + string.Join(", ", tags) + "]");
                    tagsByFile.Add(path, string.Join(" ", tags));
                    if (!string.IsNullOrWhiteSpace(model) && seenModels.Add(model)) knownModels.Add(new Model(model));
                    if (allowYield && ++count > 20) { showLoading("Loading " + Path.GetFileName(path)); count = 0; yield(); }
                }
            }
        }

        public static void BuildPowerCache(Dictionary<string, string> tagsByFile, Dictionary<string, float> powerByModel, Dictionary<string, float> topSpeedByModel, HashSet<VehicleClass> blacklistedClasses, Action<string> log)
        {
            powerByModel.Clear();
            topSpeedByModel.Clear();
            int cached = 0;
            HashSet<string> seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (string path in tagsByFile.Keys)
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
                    float power = Function.Call<float>(Hash.GET_VEHICLE_MODEL_ACCELERATION, model.Hash);
                    float topSpeed = ARS.MpsToMph(Function.Call<float>((Hash)0xF417C2502FFFED43, model.Hash)) / ARS.TopSpeedScaleDivisor;
                    powerByModel[modelText] = power;
                    topSpeedByModel[modelText] = topSpeed;
                    cached++;
                }
                catch (Exception) { }
            }
            log("BuildPowerCache: cached power for " + cached + " road cars.");
        }
    }
}
