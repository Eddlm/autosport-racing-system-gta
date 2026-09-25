using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Windows.Forms;

namespace ARS
{
    // The declared shape of the menu-owned Settings folder. LoadSettings runs three passes in this order:
    //   CreateMissingFiles  Settings\ exists.
    //   PruneOwnedFiles     Menu-*.ini lose keys the schema no longer declares.
    //   CompleteOwnedKeys   Menu-*.ini gain missing keys and lose values the schema rejects.
    // Retired standalone files are read only for migration, then deleted after menu settings are complete.
    // Known limit: an optional key holding a value of the wrong type (PaceTarget = abc) is left alone,
    // since there is no default to repair it to and absence is expressed by removing the line by hand.
    static class SettingsRepair
    {
        enum Kind { Text, Bool, Number }

        // One declared key: its default (null = optional, absence is a valid state), its type, and —
        // for list-backed keys — the spellings the menu (and so the file) is allowed to hold.
        class KeySpec
        {
            public readonly string Key;
            public readonly string Default;
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

            // The value this key must hold instead of current, or null when current is already fine.
            public string Repair(string current)
            {
                if (current == null) return Default;
                if (_kind == Kind.Bool) return bool.TryParse(current, out _) ? null : Default;
                if (_kind == Kind.Number)
                {
                    float value;
                    if (!float.TryParse(current, NumberStyles.Float, CultureInfo.InvariantCulture, out value)) return Default;
                    if (float.IsNaN(value) || float.IsInfinity(value)) return Default;   // a non-finite number is not a value
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

            // A key the schema cannot repair (no default to fall back on) has to hold a finite number, or its line goes.
            public bool HasUnusableValue(string text)
            {
                if (Default != null || _kind != Kind.Number) return false;
                float value;
                return !float.TryParse(text.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out value) || float.IsNaN(value) || float.IsInfinity(value);
            }

            // Numeric keys the menu offers as a fixed list snap to the nearest offered value, so the
            // menu keeps showing what the file holds instead of silently falling back to its first entry.
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
            race.Specs.Add(new KeySpec("Track", null));                     // optional: absent = no track chosen yet
            race.Specs.Add(new KeySpec("Laps", "6", Kind.Number, new[] { "2", "4", "6", "8", "10" }));   // numeric: a retired lap count snaps to the nearest offer instead of resetting
            race.Specs.Add(new KeySpec("GridSize", "8", Kind.Number, Array.ConvertAll(ARS.GridSizeChoices, v => v.ToString()), 0f, ARS.GridSizeChoices[ARS.GridSizeChoices.Length - 1]));
            race.Specs.Add(new KeySpec("PaceMode", PaceMode.RelativeToMine.ToString(), Kind.Text, Enum.GetNames(typeof(PaceMode))));
            race.Specs.Add(new KeySpec("PaceOffset", "0", Kind.Number, new[] { "-10", "-9", "-8", "-7", "-6", "-5", "-4", "-3", "-2", "-1", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10" }));
            race.Specs.Add(new KeySpec("PaceTarget", null, Kind.Number));   // optional: absent triggers the fleet-span middle autoselect
            race.Specs.Add(new KeySpec("ReverseRoute", "False", Kind.Bool));
            files.Add(race);

            FileSpec settings = Owned("Menu-Settings.ini");
            settings.Specs.Add(new KeySpec("GridSorting", GridSort.Random.ToString(), Kind.Text, Enum.GetNames(typeof(GridSort))));
            settings.Specs.Add(new KeySpec("VehiclePool", "cars.txt"));   // any Vehicles\*.txt; free text by design (the file list is dynamic)
            settings.Specs.Add(new KeySpec("TimeoutSeconds", "60", Kind.Number, new[] { "15", "30", "45", "60" }));
            settings.Specs.Add(new KeySpec("AIRacerAutofix", "2", Kind.Number, new[] { "0", "1", "2" }));
            settings.Specs.Add(new KeySpec("CornerOffset", "6", Kind.Number, new[] { "-10", "-8", "-6", "-4", "-2", "0", "2", "4", "6", "8", "10" }));
            settings.Specs.Add(new KeySpec("RouteOffset", "6", Kind.Number, new[] { "-10", "-8", "-6", "-4", "-2", "0", "2", "4", "6", "8", "10" }));
            settings.Specs.Add(new KeySpec("SmartTuning", "True", Kind.Bool));
            settings.Specs.Add(new KeySpec("AiNitro", TriState.IfPlayerHas.ToString(), Kind.Text, Enum.GetNames(typeof(TriState))));
            settings.Specs.Add(new KeySpec("TipRate", TipFrequency.Medium.ToString(), Kind.Text, Enum.GetNames(typeof(TipFrequency))));
            settings.Specs.Add(new KeySpec("UseMenyooSkins", "True", Kind.Bool));
            settings.Specs.Add(new KeySpec("OverspeedEnabled", "True", Kind.Bool));
            settings.Specs.Add(new KeySpec("BrakeLearning", "True", Kind.Bool));
            settings.Specs.Add(new KeySpec("TcsEnabled", "True", Kind.Bool));
            settings.Specs.Add(new KeySpec("SteerDampingScale", "1.00", Kind.Number, new[] { "0.50", "0.75", "1.00", "1.25", "1.50", "1.75", "2.00" }));
            settings.Specs.Add(new KeySpec("SteerCeilingBias", "2.0", Kind.Number, new[] { "0.0", "0.5", "1.0", "1.5", "2.0", "2.5", "3.0", "3.5", "4.0" }));
            settings.Specs.Add(new KeySpec("SteerSlipCeiling", "0.0", Kind.Number, new[] { "-1.0", "-0.9", "-0.8", "-0.7", "-0.6", "-0.5", "-0.4", "-0.3", "-0.2", "-0.1", "0.0", "0.1", "0.2", "0.3", "0.4", "0.5", "0.6", "0.7", "0.8", "0.9", "1.0" }));
            settings.Specs.Add(new KeySpec("CrestEffect", "100", Kind.Number, new[] { "0", "25", "50", "75", "100", "150", "200" }));
            settings.Specs.Add(new KeySpec("HillGripEffect", "100", Kind.Number, new[] { "0", "25", "50", "75", "100", "150", "200" }));
            settings.Specs.Add(new KeySpec("Rubberbanding", "0", Kind.Number, new[] { "0", "10", "20", "30", "40", "50", "60", "70", "80", "90", "100" }));
            settings.Specs.Add(new KeySpec("RubberbandMode", RubberbandMode.Natural.ToString(), Kind.Text, Enum.GetNames(typeof(RubberbandMode))));
            settings.Specs.Add(new KeySpec("StagedSpawns", "False", Kind.Bool));
            settings.Specs.Add(new KeySpec("Hotkeys", "True", Kind.Bool));
            settings.Specs.Add(new KeySpec("MenuModifierKey", ((int)Keys.LShiftKey).ToString(), Kind.Number, ARS.MenuHotkeyValues));
            settings.Specs.Add(new KeySpec("MenuKey", ((int)Keys.E).ToString(), Kind.Number, ARS.MenuHotkeyValues));
            settings.Specs.Add(new KeySpec("LogLevel", ARS.LogImportance.None.ToString(), Kind.Text, Enum.GetNames(typeof(ARS.LogImportance))));
            files.Add(settings);

            // The debug key set is the toggle table itself: retiring a toggle retires its key.
            FileSpec debug = Owned("Menu-Debug.ini");
            foreach (Options option in ARS.DebugToggles.Keys)
                debug.Specs.Add(new KeySpec(option.ToString(), ARS.DebugToggles[option].ToString(), Kind.Bool));
            files.Add(debug);
            return files;
        }

        // Pass 1: Settings\ exists and the old dev file receives its migration name.
        public static void CreateMissingFiles()
        {
            try
            {
                Directory.CreateDirectory(ARS.SettingsFolder);
                // The dev config was DevSettings.ini before the menu-scoped naming: a move keeps its comments.
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

        // Pass 2: an owned file may only hold declared keys. Foreign sections and comments are preserved.
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
                        bool isKeyLine = opened == null && separator > 0 && string.Equals(section, "MENU", StringComparison.OrdinalIgnoreCase);
                        if (!isKeyLine) { kept.Add(line); continue; }
                        string key = trimmed.Substring(0, separator).Trim();
                        KeySpec spec;
                        if (!declared.TryGetValue(key, out spec)) { stale.Add(key); continue; }
                        if (spec.HasUnusableValue(trimmed.Substring(separator + 1))) { unusable.Add(key); continue; }
                        kept.Add(line);
                    }
                    if (stale.Count == 0 && unusable.Count == 0) continue;
                    WriteAtomic(file.Path, kept);
                    if (stale.Count > 0) ARS.Log(ARS.LogImportance.Info, "Settings repair: " + file.Name + " dropped stale " + string.Join(", ", stale.ToArray()));
                    if (unusable.Count > 0) ARS.Log(ARS.LogImportance.Info, "Settings repair: " + file.Name + " dropped unusable " + string.Join(", ", unusable.ToArray()));
                }
                catch (Exception ex)
                {
                    ARS.Log(ARS.LogImportance.Error, "Settings repair: could not prune " + file.Name + " - " + ex.Message, true);
                }
            }
        }

        // Pass 3: every declared key exists and holds a value the schema accepts. Runs after the legacy
        // migrations, so a repaired default can never shadow a value carried over from an older install.
        public static void CompleteOwnedKeys(MenuSettings race, MenuSettings settings, MenuSettings debug)
        {
            Dictionary<string, MenuSettings> stores = new Dictionary<string, MenuSettings>(StringComparer.OrdinalIgnoreCase)
            {
                { "Menu-Race.ini", race },
                { "Menu-Settings.ini", settings },
                { "Menu-Debug.ini", debug },
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

        // The declared key names of one owned file, so a rename migration walks the schema instead of a copy of it.
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

        // Retired files are removed only after any values they still carry have been migrated to menu settings.
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

        // Never leave a half-written settings file behind: the rewrite lands in one move.
        static void WriteAtomic(string path, List<string> lines)
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
