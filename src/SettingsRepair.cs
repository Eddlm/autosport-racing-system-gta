// The schema of every owned ini, and the two passes that keep a file inside it: prune undeclared keys, repair
// values the menu could not have produced. A file whose keys cannot be parsed is not rewritten as a whole.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Windows.Forms;

namespace ARS
{

    static class SettingsRepair
    {
        enum Kind { Text, Bool, Number }

        class KeySpec
        {
            public readonly string Key;
            public readonly string Default;
            public string[] Domain { get { return _domain; } }
            readonly Kind _kind;
            readonly string[] _domain;
            readonly float _min;
            readonly float _max;

            public KeySpec(string key, string byDefault, Kind kind = Kind.Text, string[] domain = null, float min = 0f, float max = 0f)
            {
                Key = key;
                Default = byDefault;
                _kind = kind;
                _domain = domain;
                _min = min;
                _max = max;
            }

            public string Repair(string current)
            {
                if (current == null) return Default;
                if (_kind == Kind.Bool) return bool.TryParse(current, out _) ? null : Default;
                if (_kind == Kind.Number)
                {
                    float value;
                    if (!float.TryParse(current, NumberStyles.Float, CultureInfo.InvariantCulture, out value)) return Default;
                    if (float.IsNaN(value) || float.IsInfinity(value)) return Default;
                    string snapped = SnappedToList(value);
                    if (snapped != null) return snapped;
                    if (_max > _min && (value < _min || value > _max)) return Clamp(value).ToString("0.###", CultureInfo.InvariantCulture);
                    return null;
                }
                if (_domain == null) return null;
                if (_domain.Contains(current, StringComparer.Ordinal)) return null;
                string canonical = _domain.FirstOrDefault(d => string.Equals(d, current, StringComparison.OrdinalIgnoreCase));
                return canonical ?? Default;
            }

            public bool HasUnusableValue(string text)
            {
                if (Default != null || _kind != Kind.Number) return false;
                float value;
                return !float.TryParse(text.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out value) || float.IsNaN(value) || float.IsInfinity(value);
            }

            string SnappedToList(float value)
            {
                if (_domain == null) return null;
                string best = null;
                float bestDistance = float.MaxValue;
                foreach (string entry in _domain)
                {
                    float candidate;
                    if (!float.TryParse(entry, NumberStyles.Float, CultureInfo.InvariantCulture, out candidate)) continue;
                    if (candidate == value) return null;
                    float distance = Math.Abs(candidate - value);
                    if (distance >= bestDistance) continue;
                    bestDistance = distance;
                    best = entry;
                }
                return best;
            }

            float Clamp(float value)
            {
                if (value < _min) return _min;
                return value > _max ? _max : value;
            }
        }

        class FileSpec
        {
            public readonly string Name;
            public readonly bool Owned;
            public readonly List<KeySpec> Specs = new List<KeySpec>();

            public FileSpec(string name, bool owned)
            {
                Name = name;
                Owned = owned;
            }

            public string Path { get { return ARS.SettingsFolder + @"\" + Name; } }
        }

        static List<FileSpec> _schema;

        static List<FileSpec> Schema { get { return _schema ?? (_schema = BuildSchema()); } }

        static FileSpec Owned(string name)
        {
            return new FileSpec(name, true);
        }

        static List<FileSpec> BuildSchema()
        {
            List<FileSpec> files = new List<FileSpec>();

            FileSpec race = Owned("Menu-Race.ini");
            race.Specs.Add(new KeySpec("Track", null));
            race.Specs.Add(new KeySpec("Laps", "6", Kind.Number, new[] { "2", "4", "6", "8", "10" }));
            race.Specs.Add(new KeySpec("GridSize", "8", Kind.Number, Array.ConvertAll(ARS.GridSizeChoices, v => v.ToString()), 0f, ARS.GridSizeChoices[ARS.GridSizeChoices.Length - 1]));
            race.Specs.Add(new KeySpec("CapturedGridMode", "Hybrid", Kind.Text, new[] { "Hybrid", "CustomClones", "CustomOnly", "Standard" }));
            race.Specs.Add(new KeySpec("PaceMode", PaceMode.RelativeToMine.ToString(), Kind.Text, Enum.GetNames(typeof(PaceMode))));
            race.Specs.Add(new KeySpec("PaceOffset", "0", Kind.Number, new[] { "-10", "-9", "-8", "-7", "-6", "-5", "-4", "-3", "-2", "-1", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10" }));
            race.Specs.Add(new KeySpec("PaceTarget", null, Kind.Number));
            race.Specs.Add(new KeySpec("PaceDeviation", "Off", Kind.Text, new[] { "Off", "2", "4", "6", "8" }));
            race.Specs.Add(new KeySpec("ReverseRoute", "False", Kind.Bool));
            files.Add(race);

            FileSpec settings = Owned("Menu-Settings.ini");
            settings.Specs.Add(new KeySpec("GridSorting", GridSort.Random.ToString(), Kind.Text, Enum.GetNames(typeof(GridSort))));
            settings.Specs.Add(new KeySpec("VehiclePool", "cars.txt"));
            settings.Specs.Add(new KeySpec("TimeoutSeconds", "30", Kind.Number, new[] { "15", "30", "45", "60" }));
            settings.Specs.Add(new KeySpec("AIRacerAutofix", AutofixMode.AutoRepair.ToString(), Kind.Text, Enum.GetNames(typeof(AutofixMode))));
            settings.Specs.Add(new KeySpec("CornerOffset", "4", Kind.Number, new[] { "-10", "-8", "-6", "-4", "-2", "0", "2", "4", "6", "8", "10" }));
            settings.Specs.Add(new KeySpec("RouteOffset", "2", Kind.Number, new[] { "-10", "-8", "-6", "-4", "-2", "0", "2", "4", "6", "8", "10" }));
            settings.Specs.Add(new KeySpec("SmartTuning", "True", Kind.Bool));
            settings.Specs.Add(new KeySpec("NitrousEnabled", "True", Kind.Bool));
            settings.Specs.Add(new KeySpec("TipRate", TipFrequency.High.ToString(), Kind.Text, Enum.GetNames(typeof(TipFrequency))));
            settings.Specs.Add(new KeySpec("UseMenyooSkins", "True", Kind.Bool));
            settings.Specs.Add(new KeySpec("ForceFillGrid", "True", Kind.Bool));
            settings.Specs.Add(new KeySpec("OverspeedCut", "0.2", Kind.Text, new[] { "Off", "0.1", "0.2", "0.3", "0.4", "0.5", "0.6", "0.7", "0.8", "0.9", "1.0" }));
            settings.Specs.Add(new KeySpec("BrakeLearning", "True", Kind.Bool));
            settings.Specs.Add(new KeySpec("TcsEnabled", "True", Kind.Bool));
            settings.Specs.Add(new KeySpec("AbsEnabled", "False", Kind.Bool));

            settings.Specs.Add(new KeySpec("TcsSlipLimitPct", "100", Kind.Number, new[] { "10", "20", "30", "40", "50", "60", "70", "80", "90", "100", "110", "120", "130", "140", "150", "160", "170", "180", "190", "200" }));
            settings.Specs.Add(new KeySpec("TcsTaperSpanPct", "50", Kind.Number, new[] { "10", "20", "30", "40", "50", "60", "70", "80", "90", "100", "110", "120", "130", "140", "150", "160", "170", "180", "190", "200" }));
            settings.Specs.Add(new KeySpec("DNFUnderTrack", "True", Kind.Bool));
            settings.Specs.Add(new KeySpec("NoCollision", "False", Kind.Bool));
            settings.Specs.Add(new KeySpec("SteerDampingGain", "0.50", Kind.Number, new[] { "0.00", "0.10", "0.20", "0.30", "0.40", "0.50", "0.60", "0.70", "0.80", "0.90", "1.00", "1.10", "1.20", "1.30", "1.40", "1.50", "1.60", "1.70", "1.80", "1.90", "2.00" }));
            settings.Specs.Add(new KeySpec("DamperCountersteerMaxDeg", "2.00", Kind.Number, new[] { "0.00", "2.00", "4.00", "6.00", "8.00", "10.00" }));
            settings.Specs.Add(new KeySpec("SteerCeilingFactor", "0.90", Kind.Number, new[] { "0.05", "0.10", "0.15", "0.20", "0.25", "0.30", "0.35", "0.40", "0.45", "0.50", "0.55", "0.60", "0.65", "0.70", "0.75", "0.80", "0.85", "0.90", "0.95", "1.00", "1.05", "1.10", "1.15", "1.20", "1.25", "1.30", "1.35", "1.40", "1.45", "1.50", "1.55", "1.60", "1.65", "1.70", "1.75", "1.80", "1.85", "1.90", "1.95", "2.00" }));
            settings.Specs.Add(new KeySpec("SlipCutFloor", "0.40", Kind.Number, new[] { "0.00", "0.05", "0.10", "0.15", "0.20", "0.25", "0.30", "0.35", "0.40", "0.45", "0.50", "0.55", "0.60", "0.65", "0.70", "0.75", "0.80", "0.85", "0.90" }));
            settings.Specs.Add(new KeySpec("MaxCrestLift", "0.4", Kind.Number, new[] { "0.1", "0.2", "0.3", "0.4", "0.5", "0.6", "0.7", "0.8", "0.9", "1.0", "1.1", "1.2", "1.3", "1.4", "1.5", "1.6", "1.7", "1.8", "1.9", "2.0" }));
            settings.Specs.Add(new KeySpec("HillGripEffect", "100", Kind.Number, new[] { "0", "25", "50", "75", "100", "150", "200" }));
            settings.Specs.Add(new KeySpec("GsLaneThrottleFloor", "True", Kind.Bool));
            settings.Specs.Add(new KeySpec("OpenUpOnThrottleFloorPct", "60", Kind.Number, ARS.OpenUpFloorChoices));
            settings.Specs.Add(new KeySpec("Rubberbanding", "100", Kind.Number, new[] { "0", "10", "20", "30", "40", "50", "60", "70", "80", "90", "100" }));
            settings.Specs.Add(new KeySpec("RubberbandMode", RubberbandMode.Natural.ToString(), Kind.Text, Enum.GetNames(typeof(RubberbandMode))));
            settings.Specs.Add(new KeySpec("StagedSpawns", "False", Kind.Bool));
            settings.Specs.Add(new KeySpec("Hotkeys", "True", Kind.Bool));
            settings.Specs.Add(new KeySpec("MenuModifierKey", ((int)Keys.LShiftKey).ToString(), Kind.Number, ARS.MenuHotkeyValues));
            settings.Specs.Add(new KeySpec("MenuKey", ((int)Keys.E).ToString(), Kind.Number, ARS.MenuHotkeyValues));
            settings.Specs.Add(new KeySpec("LogLevel", ARS.LogImportance.Error.ToString(), Kind.Text, Enum.GetNames(typeof(ARS.LogImportance))));
            settings.Specs.Add(new KeySpec("CornerCeiling", "200", Kind.Number, ARS.CornerCeilingValues));
            settings.Specs.Add(new KeySpec("AiNitroPct", "100", Kind.Number, ARS.AiNitroValues));
            files.Add(settings);

            FileSpec development = Owned("Menu-Development.ini");
            development.Specs.Add(new KeySpec("BrakeLearningNotify", "False", Kind.Bool));
development.Specs.Add(new KeySpec("SteerTrace", "False", Kind.Bool));
            development.Specs.Add(new KeySpec("SlipFromMemory", "True", Kind.Bool));
            files.Add(development);

            FileSpec debug = Owned("Menu-Debug.ini");
            foreach (Options option in ARS.DebugToggles.Keys)
                debug.Specs.Add(new KeySpec(option.ToString(), ARS.DebugToggles[option].ToString(), Kind.Bool));
            files.Add(debug);
            return files;
        }

        public static void CreateMissingFiles()
        {
            try
            {
                Directory.CreateDirectory(ARS.SettingsFolder);

                string legacyDevConfig = ARS.SettingsFolder + @"\DevSettings.ini";
                string devConfig = ARS.SettingsFolder + @"\DevConfig.ini";
                if (File.Exists(legacyDevConfig) && !File.Exists(devConfig))
                {
                    File.Move(legacyDevConfig, devConfig);
                    ARS.Log(ARS.LogImportance.Info, "Settings repair: renamed DevSettings.ini to DevConfig.ini.");
                }
            }
            catch (Exception ex)
            {
                ARS.Log(ARS.LogImportance.Error, "Settings repair: could not create missing files - " + ex.Message, true);
            }
        }

        public static void PruneOwnedFiles()
        {
            foreach (FileSpec file in Schema)
            {
                if (!file.Owned || !File.Exists(file.Path)) continue;
                try
                {
                    Dictionary<string, KeySpec> declared = new Dictionary<string, KeySpec>(StringComparer.OrdinalIgnoreCase);
                    foreach (KeySpec spec in file.Specs) declared[spec.Key] = spec;
                    List<string> kept = new List<string>();
                    List<string> stale = new List<string>();
                    List<string> unusable = new List<string>();
                    string section = null;
                    foreach (string line in File.ReadAllLines(file.Path))
                    {
                        string trimmed = line.Trim();
                        string opened = SectionNameOf(trimmed);
                        if (opened != null) section = opened;
                        int separator = trimmed.IndexOf('=');
                        bool isComment = trimmed.Length > 0 && (trimmed[0] == ';' || trimmed[0] == '#');
                        bool isKeyLine = opened == null && !isComment && separator > 0 && string.Equals(section, "MENU", StringComparison.OrdinalIgnoreCase);
                        if (!isKeyLine) { kept.Add(line); continue; }
                        string key = trimmed.Substring(0, separator).Trim();
                        KeySpec spec;
                        if (!declared.TryGetValue(key, out spec)) { stale.Add(key); continue; }
                        if (spec.HasUnusableValue(trimmed.Substring(separator + 1))) { unusable.Add(key); continue; }
                        kept.Add(line);
                    }
                    if (stale.Count == 0 && unusable.Count == 0) continue;
                    WriteAtomic(file.Path, kept);
                    if (stale.Count > 0) ARS.Log(ARS.LogImportance.Info, "Settings repair: " + file.Name + " dropped stale " + string.Join(", ", stale.ToArray()), true);
                    if (unusable.Count > 0) ARS.Log(ARS.LogImportance.Info, "Settings repair: " + file.Name + " dropped unusable " + string.Join(", ", unusable.ToArray()), true);
                }
                catch (Exception ex)
                {
                    ARS.Log(ARS.LogImportance.Error, "Settings repair: could not prune " + file.Name + " - " + ex.Message, true);
                }
            }
        }

        public static void CompleteOwnedKeys(MenuSettings race, MenuSettings settings, MenuSettings debug, MenuSettings development)
        {
            Dictionary<string, MenuSettings> stores = new Dictionary<string, MenuSettings>(StringComparer.OrdinalIgnoreCase)
            {
                { "Menu-Race.ini", race },
                { "Menu-Settings.ini", settings },
                { "Menu-Debug.ini", debug },
                { "Menu-Development.ini", development },
            };
            foreach (FileSpec file in Schema)
            {
                if (!file.Owned) continue;
                MenuSettings store;
                if (!stores.TryGetValue(file.Name, out store) || store == null) continue;
                foreach (KeySpec spec in file.Specs)
                {
                    try
                    {
                        string current = store.Get(spec.Key, null);
                        string repaired = spec.Repair(current);
                        if (repaired == null) continue;
                        store.Set(spec.Key, repaired);
                        ARS.Log(ARS.LogImportance.Info, "Settings repair: " + file.Name + " " + spec.Key + " = " + repaired + (current == null ? " (was missing)" : " (was '" + current + "')"));
                    }
                    catch (Exception ex)
                    {
                        ARS.Log(ARS.LogImportance.Error, "Settings repair: could not fix " + file.Name + " " + spec.Key + " - " + ex.Message, true);
                    }
                }
            }
        }

        public static List<string> DeclaredKeys(string fileName)
        {
            foreach (FileSpec file in Schema)
            {
                if (!string.Equals(file.Name, fileName, StringComparison.OrdinalIgnoreCase)) continue;
                List<string> keys = new List<string>();
                foreach (KeySpec spec in file.Specs) keys.Add(spec.Key);
                return keys;
            }
            return new List<string>();
        }

        static Dictionary<string, HashSet<string>> _declaredKeys;

        static Dictionary<string, HashSet<string>> DeclaredKeySets
        {
            get
            {
                if (_declaredKeys != null) return _declaredKeys;
                _declaredKeys = new Dictionary<string, HashSet<string>>(StringComparer.OrdinalIgnoreCase);
                foreach (FileSpec file in Schema)
                {

                    HashSet<string> keys = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
                    foreach (KeySpec spec in file.Specs) keys.Add(spec.Key);
                    _declaredKeys[file.Name] = keys;
                }
                return _declaredKeys;
            }
        }

        public static bool IsDeclared(string fileName, string key)
        {
            HashSet<string> keys;
            return DeclaredKeySets.TryGetValue(fileName, out keys) && keys.Contains(key);
        }

        public static string[] OptionsFor(string key)
        {
            foreach (FileSpec file in Schema)
                foreach (KeySpec spec in file.Specs)
                    if (string.Equals(spec.Key, key, StringComparison.Ordinal) && spec.Domain != null) return spec.Domain;
            return new string[0];
        }

        static readonly HashSet<string> ReportedUndeclared = new HashSet<string>(StringComparer.Ordinal);

        public static void ReportUndeclared(string fileName, string key, string operation)
        {
            if (!ReportedUndeclared.Add(fileName + "|" + key)) return;
            ARS.Log(ARS.LogImportance.Error, "Settings: " + fileName + " declares no key '" + key + "', so the menu's " + operation + " was refused - an undeclared key is dropped on the next load, which would revert the setting silently.", true);
        }

        public static void DeleteLegacyFiles()
        {
            foreach (string name in new[] { "Settings.ini", "Options.ini", "DevSettings.ini", "DevConfig.ini", "MemoryOffsets.ini", "Menu-Racers.ini", "Menu-DevSettings.ini" })
            {
                string path = ARS.SettingsFolder + @"\" + name;
                try
                {
                    if (!File.Exists(path)) continue;
                    File.Delete(path);
                    ARS.Log(ARS.LogImportance.Info, "Settings repair: removed retired " + name + " after migration.");
                }
                catch (Exception ex)
                {
                    ARS.Log(ARS.LogImportance.Error, "Settings repair: could not remove " + name + " - " + ex.Message, true);
                }
            }
        }

        static string SectionNameOf(string trimmed)
        {
            if (trimmed.Length < 3 || trimmed[0] != '[' || trimmed[trimmed.Length - 1] != ']') return null;
            return trimmed.Substring(1, trimmed.Length - 2).Trim();
        }

        public static void WriteAtomic(string path, List<string> lines)
        {
            string temporary = path + ".repair";
            File.WriteAllLines(temporary, lines);
            try
            {
                if (File.Exists(path)) File.Replace(temporary, path, null);
                else File.Move(temporary, path);
            }
            catch (Exception)
            {
                TryDelete(temporary);
                throw;
            }
        }

        static void TryDelete(string path)
        {
            try { if (File.Exists(path)) File.Delete(path); }
            catch (Exception) { }
        }
    }
}
