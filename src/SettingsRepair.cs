using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;

namespace ARS
{
    // The declared shape of the Settings folder: which ini files ARS expects and which keys each must
    // carry. LoadSettings runs three passes in this order:
    //   CreateMissingFiles  Settings\ exists, and every foreign file is present.
    //   PruneOwnedFiles     ARS-owned Menu-*.ini lose keys the schema no longer declares.
    //   CompleteOwnedKeys   ARS-owned files gain missing keys and lose values the schema rejects.
    // Foreign files (Options.ini, DevSettings.ini, MemoryOffsets.ini) are hand-editable and may carry
    // comments, which ScriptSettings.Save drops — so they are created when missing but never rewritten.
    // A created file holds exactly the values the code uses when a key is absent, so creating one can
    // never change behaviour. Settings.ini is a legacy migration input and is deliberately not expected.
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
            public readonly string DefaultText;
            public readonly List<KeySpec> Specs = new List<KeySpec>();

            public FileSpec(string name, bool owned, string defaultText = null)
            {
                Name = name;
                Owned = owned;
                DefaultText = defaultText;
            }

            public string Path { get { return ARS.SettingsFolder + @"\" + Name; } }
        }

        const string MEMORY_OFFSETS_DEFAULTS =
            ";GTA Online updates occasionally move the memory offsets ARS needs for direct handling,\r\n" +
            ";steering and throttle input. Update these when that happens, or leave at 0x0 to let ARS\r\n" +
            ";find them itself.\r\n" +
            "[MEMORY_OFFSETS]\r\n" +
            "Steer=0x0\r\n" +
            "Throttle=0x0\r\n" +
            "Brake=0x0\r\n";

        static readonly string OPTIONS_DEFAULTS = Lines(
            "[GENERAL_SETTINGS]",
            "Laps = 5",
            "ReverseRoutes = false",
            "",
            "[CATCHUP]",
            "OnlyLastHalf = true",
            "OnlyBehindPlayer = true");

        static readonly string DEVSETTINGS_DEFAULTS = Lines(
            "[GENERAL]",
            "Hotkeys = true",
            "LoadAtStart = true",
            "LogLevel = Info",
            "",
            "[CREATOR_DEFAULTS]",
            "TracksideModel = prop_wheel_tyre",
            "TracksideModelFrecuency = 10");

        static List<FileSpec> _schema;

        static List<FileSpec> Schema { get { return _schema ?? (_schema = BuildSchema()); } }

        static string Lines(params string[] lines)
        {
            return string.Join("\r\n", lines) + "\r\n";
        }

        static FileSpec Owned(string name)
        {
            return new FileSpec(name, true);
        }

        static List<FileSpec> BuildSchema()
        {
            List<FileSpec> files = new List<FileSpec>();

            FileSpec race = Owned("Menu-Race.ini");
            race.Specs.Add(new KeySpec("Track", null));                     // optional: absent = no track chosen yet
            race.Specs.Add(new KeySpec("Laps", "5", Kind.Text, new[] { "3", "5", "7", "9", "11", "13", "15", "17", "19" }));
            race.Specs.Add(new KeySpec("GridSize", "4", Kind.Number, null, 0f, 12f));
            race.Specs.Add(new KeySpec("PaceOffset", "0", Kind.Number));
            race.Specs.Add(new KeySpec("PaceTarget", null, Kind.Number));   // optional: absent triggers the fleet-span middle autoselect
            race.Specs.Add(new KeySpec("ReverseRoute", "False", Kind.Bool));
            files.Add(race);

            FileSpec racers = Owned("Menu-Racers.ini");
            racers.Specs.Add(new KeySpec("GridSorting", "Power", Kind.Text, new[] { "Power", "PowerDescendent", "TopSpeed", "TopSpeedDescendent", "Random" }));
            racers.Specs.Add(new KeySpec("TimeoutSeconds", "30", Kind.Number, new[] { "15", "30", "45", "60" }));
            racers.Specs.Add(new KeySpec("AIRacerAutofix", "1", Kind.Number, new[] { "0", "1", "2" }));
            racers.Specs.Add(new KeySpec("AITuningLevel", "1", Kind.Number, new[] { "0", "1", "2", "3" }));
            racers.Specs.Add(new KeySpec("AiNitro", "IfPlayerHas", Kind.Text, new[] { "Never", "IfPlayerHas", "Always" }));
            racers.Specs.Add(new KeySpec("UseMenyooSkins", "True", Kind.Bool));
            racers.Specs.Add(new KeySpec("OverspeedEnabled", "True", Kind.Bool));
            files.Add(racers);

            FileSpec settings = Owned("Menu-Settings.ini");
            settings.Specs.Add(new KeySpec("PaceMode", "Relative", Kind.Text, new[] { "Absolute", "Relative" }));
            files.Add(settings);

            // The dev file's key set is the debug toggle table itself: retiring a toggle retires its key.
            FileSpec dev = Owned("Menu-DevSettings.ini");
            foreach (Options option in ARS.DebugToggles.Keys)
                dev.Specs.Add(new KeySpec(option.ToString(), ARS.DebugToggles[option].ToString(), Kind.Bool));
            files.Add(dev);

            files.Add(new FileSpec("Options.ini", false, OPTIONS_DEFAULTS));
            files.Add(new FileSpec("DevSettings.ini", false, DEVSETTINGS_DEFAULTS));
            files.Add(new FileSpec("MemoryOffsets.ini", false, MEMORY_OFFSETS_DEFAULTS));
            return files;
        }

        // Pass 1: Settings\ and every foreign file exist. Existing files are left untouched byte for byte.
        public static void CreateMissingFiles()
        {
            try
            {
                Directory.CreateDirectory(ARS.SettingsFolder);
                foreach (FileSpec file in Schema)
                {
                    if (file.Owned || file.DefaultText == null || File.Exists(file.Path)) continue;
                    File.WriteAllText(file.Path, file.DefaultText);
                    ARS.Log(ARS.LogImportance.Error, "Settings repair: created missing " + file.Name + " with defaults.");
                    GTA.UI.Notify("~o~ARS restored a missing settings file:~w~ " + file.Name);
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
                    HashSet<string> declared = new HashSet<string>(file.Specs.Select(s => s.Key), StringComparer.OrdinalIgnoreCase);
                    List<string> kept = new List<string>();
                    List<string> dropped = new List<string>();
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
                        if (declared.Contains(key)) kept.Add(line);
                        else dropped.Add(key);
                    }
                    if (dropped.Count == 0) continue;
                    WriteAtomic(file.Path, kept);
                    ARS.Log(ARS.LogImportance.Info, "Settings repair: " + file.Name + " dropped stale " + string.Join(", ", dropped.ToArray()));
                }
                catch (Exception ex)
                {
                    ARS.Log(ARS.LogImportance.Error, "Settings repair: could not prune " + file.Name + " - " + ex.Message, true);
                }
            }
        }

        // Pass 3: every declared key exists and holds a value the schema accepts. Runs after the legacy
        // migrations, so a repaired default can never shadow a value carried over from an older install.
        public static void CompleteOwnedKeys(MenuSettings race, MenuSettings racers, MenuSettings settings, MenuSettings dev)
        {
            Dictionary<string, MenuSettings> stores = new Dictionary<string, MenuSettings>(StringComparer.OrdinalIgnoreCase)
            {
                { "Menu-Race.ini", race },
                { "Menu-Racers.ini", racers },
                { "Menu-Settings.ini", settings },
                { "Menu-DevSettings.ini", dev },
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
