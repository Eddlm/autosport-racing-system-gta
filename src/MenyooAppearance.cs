using GTA;
using GTA.Native;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Xml;

namespace ARS
{
    // Loads Menyoo-style vehicle appearance XMLs (the "menyooStuff\Vehicle" save format)
    // and applies one matching file's tuning to a spawned vehicle. Purely cosmetic — no
    // handling/stat tuning — and entirely separate from ARS's own Vehicles\*.xml supplier pool.
    // Match is by the file's <ModelHash> against the spawned vehicle's model hash. When several
    // files match (multiple liveries for one car) one is picked at random. Folder is the game
    // install's menyooStuff\Vehicle, searched recursively (including the "SAM - Track Cars"
    // subfolder where livery skins live).
    public static class MenyooAppearance
    {
        private const string MenyooSubPath = @"menyooStuff\Vehicle";

        private static List<string> _files;
        private static List<string> Files
        {
            get
            {
                if (_files != null) return _files;
                _files = GetFiles();
                return _files;
            }
        }

        public static void InvalidateCache() { _files = null; }

        private static List<string> GetFiles()
        {
            try
            {
                string root = GameRoot();
                if (root == null) return new List<string>();
                string folder = Path.Combine(root, MenyooSubPath);
                if (!Directory.Exists(folder)) return new List<string>();
                return Directory.GetFiles(folder, "*.xml", SearchOption.AllDirectories).ToList();
            }
            catch (Exception) { return new List<string>(); }
        }

        private static string GameRoot()
        {
            // The script's working directory (and ScriptsFolder's parent) is the game install root.
            try
            {
                string scripts = Path.GetFullPath(ARS.ScriptsFolder);
                // ScriptsFolder is "scripts\AutosportRacingSystem" — the game root is two levels up.
                return Directory.GetParent(Directory.GetParent(scripts)?.FullName)?.FullName;
            }
            catch (Exception) { return null; }
        }

        static bool TryGetModelHash(Vehicle car, out int hash)
        {
            hash = 0;
            if (car == null || !car.Exists()) return false;
            try { hash = car.Model.Hash; return true; }
            catch (Exception) { return false; }
        }

        static int ReadHash(XmlDocument doc)
        {
            XmlNode node = doc.SelectSingleNode("/Vehicle/ModelHash");
            if (node == null) return 0;
            string text = node.InnerText.Trim();
            if (text.StartsWith("0x", StringComparison.OrdinalIgnoreCase))
                return Convert.ToInt32(text, 16);
            return int.TryParse(text, out int v) ? v : 0;
        }

        static List<string> FilesMatching(Vehicle car)
        {
            if (!TryGetModelHash(car, out int hash) || hash == 0) return new List<string>();
            List<string> matches = new List<string>();
            foreach (string file in Files)
            {
                try
                {
                    XmlDocument doc = new XmlDocument();
                    doc.Load(file);
                    if (ReadHash(doc) == hash) matches.Add(file);
                }
                catch (Exception) { }
            }
            return matches;
        }

        // Apply the tuning of one randomly-picked matching file to the given vehicle.
        // No-op if the vehicle model has no matching Menyoo file.
        public static void Apply(Vehicle car)
        {
            if (car == null || !car.Exists()) return;
            if (Files.Count == 0) return;

            List<string> matches = FilesMatching(car);
            if (matches.Count == 0) return;

            string chosen = matches[ARS.GetRandomInt(0, matches.Count - 1)];
            try
            {
                XmlDocument doc = new XmlDocument();
                doc.Load(chosen);
                ApplyAppearance(doc, car);
            }
            catch (Exception) { }
        }

        static void ApplyAppearance(XmlDocument doc, Vehicle car)
        {
            car.InstallModKit();

            XmlNode colours = doc.SelectSingleNode("/Vehicle/VehicleProperties/Colours");
            if (colours != null)
            {
                SetColor(car, colours, "Primary", (c, v) => c.PrimaryColor = v);
                SetColor(car, colours, "Secondary", (c, v) => c.SecondaryColor = v);
                SetColor(car, colours, "Pearl", (c, v) => c.PearlescentColor = v);
                SetColor(car, colours, "Rim", (c, v) => c.RimColor = v);
                SetColor(car, colours, "LrDashboard", (c, v) => c.DashboardColor = v);
                SetColor(car, colours, "Interior", (c, v) => c.TrimColor = v);
            }

            SetValue(doc, "/Vehicle/VehicleProperties/Livery", v => { if (v >= 0 && car.LiveryCount > v) car.Livery = v; });
            SetValue(doc, "/Vehicle/VehicleProperties/WheelType", v => car.WheelType = (VehicleWheelType)v);
            SetValue(doc, "/Vehicle/VehicleProperties/WindowTint", v => car.WindowTint = (VehicleWindowTint)v);

            ApplyMods(doc, car);
            ApplyNeons(colours, car);
        }

        static void SetColor(Vehicle car, XmlNode parent, string name, Action<Vehicle, VehicleColor> set)
        {
            XmlNode n = parent.SelectSingleNode(name);
            if (n == null) return;
            if (!int.TryParse(n.InnerText.Trim(), out int v)) return;
            try { set(car, (VehicleColor)v); } catch (Exception) { }
        }

        static void SetValue(XmlDocument doc, string path, Action<int> set)
        {
            XmlNode n = doc.SelectSingleNode(path);
            if (n == null) return;
            if (!int.TryParse(n.InnerText.Trim(), out int v)) return;
            set(v);
        }

        static void ApplyMods(XmlDocument doc, Vehicle car)
        {
            XmlNodeList mods = doc.SelectNodes("/Vehicle/VehicleProperties/Mods/*");
            if (mods == null) return;
            foreach (XmlNode m in mods)
            {
                string indexText = m.Name.TrimStart('_');
                if (!int.TryParse(indexText, out int slot)) continue;
                if (slot < 0) continue;

                string valueText = m.InnerText.Trim();
                if (valueText == "true" || valueText == "false")
                {
                    if (valueText == "true") car.ToggleMod((VehicleToggleMod)slot, true);
                    continue;
                }

                // Menyoo stores "value,variant"; the trailing variant is the custom-flag.
                int variant = 0;
                if (valueText.Contains(","))
                {
                    string[] parts = valueText.Split(',');
                    variant = int.TryParse(parts[1].Trim(), out int vt) ? vt : 0;
                    valueText = parts[0].Trim();
                }
                if (!int.TryParse(valueText, out int value)) continue;

                try { car.SetMod((VehicleMod)slot, value, variant != 0); } catch (Exception) { }
            }
        }

        static void ApplyNeons(XmlNode colours, Vehicle car)
        {
            if (colours == null) return;
            XmlNode neon = colours.SelectSingleNode("Neons");
            if (neon == null) return;

            bool left = BoolVal(neon, "Left");
            bool right = BoolVal(neon, "Right");
            bool front = BoolVal(neon, "Front");
            bool back = BoolVal(neon, "Back");
            if (!left && !right && !front && !back) return;

            int r = IntVal(neon, "R", 255), g = IntVal(neon, "G", 0), b = IntVal(neon, "B", 255);
            try
            {
                FunctionWrapper.SetNeonEnabled(car, left, right, front, back);
                FunctionWrapper.SetNeonColor(car, r, g, b);
            }
            catch (Exception) { }
        }

        static bool BoolVal(XmlNode parent, string name)
        {
            XmlNode n = parent.SelectSingleNode(name);
            return n != null && string.Equals(n.InnerText.Trim(), "true", StringComparison.OrdinalIgnoreCase);
        }

        static int IntVal(XmlNode parent, string name, int fallback)
        {
            XmlNode n = parent.SelectSingleNode(name);
            return (n != null && int.TryParse(n.InnerText.Trim(), out int v)) ? v : fallback;
        }
    }

    static class FunctionWrapper
    {
        // SET_VEHICLE_NEON_ENABLED (vehicle, frontLeft, frontRight, rearLeft, rearRight)
        const int NeonEnabledHash = unchecked((int)0x2AA720E4287BF269);
        // SET_VEHICLE_NEON_COLOUR (vehicle, r, g, b)
        const int NeonColorHash = unchecked((int)0x8E0A58289A6BAAAB);

        public static void SetNeonEnabled(Vehicle car, bool left, bool right, bool front, bool back)
        {
            Function.Call(unchecked((Hash)NeonEnabledHash), car, left, right, front, back);
        }
        public static void SetNeonColor(Vehicle car, int r, int g, int b)
        {
            Function.Call(unchecked((Hash)NeonColorHash), car, r, g, b);
        }
    }
}
