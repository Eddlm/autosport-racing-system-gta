using GTA;
using System.Globalization;
using System.IO;

namespace ARS
{
    // One ini per menu: a single ScriptSettings object for Settings\Menu-<Name>.ini,
    // loaded lazily and written on every change.
    public class MenuSettings
    {
        readonly string _path;
        readonly string _fileName;
        ScriptSettings _file;

        public MenuSettings(string path)
        {
            _path = path;
            _fileName = Path.GetFileName(path);
        }

        ScriptSettings File
        {
            get { return _file ?? (_file = ScriptSettings.Load(_path)); }
        }

        // The schema owns which keys exist. An undeclared one is dropped by the next prune, so a refused
        // touch keeps the session behaving like the next load instead of pretending until then.
        bool Declared(string key, string operation)
        {
            if (SettingsRepair.IsDeclared(_fileName, key)) return true;
            SettingsRepair.ReportUndeclared(_fileName, key, operation);
            return false;
        }

        public string Get(string key, string fallback)
        {
            if (!Declared(key, "read")) return fallback;
            SeedIfMissing(key, fallback);
            return File.GetValue<string>("MENU", key, fallback);
        }

        public bool GetBool(string key, bool fallback)
        {
            if (!Declared(key, "read")) return fallback;
            SeedIfMissing(key, fallback.ToString());
            return File.GetValue<bool>("MENU", key, fallback);
        }

        public int GetInt(string key, int fallback)
        {
            if (!Declared(key, "read")) return fallback;
            SeedIfMissing(key, fallback.ToString(CultureInfo.InvariantCulture));
            return File.GetValue<int>("MENU", key, fallback);
        }

        public float GetFloat(string key, float fallback)
        {
            if (!Declared(key, "read")) return fallback;
            SeedIfMissing(key, fallback.ToString(CultureInfo.InvariantCulture));
            return File.GetValue<float>("MENU", key, fallback);
        }

        // Read without seeding, for keys whose absence is a meaningful state (see PaceTarget: absent
        // means "auto-pick", and seeding a sentinel like NaN would write that sentinel into the file).
        public bool TryGetFloat(string key, out float value)
        {
            value = 0f;
            string text = Get(key, null);
            return text != null && float.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value);
        }

        // Seed-on-read: the ini mirrors the live key set after the first load.
        void SeedIfMissing(string key, string value)
        {
            if (value == null || File.GetValue<string>("MENU", key, null) != null) return;
            File.SetValue("MENU", key, value);
            File.Save();
        }

        public void Set(string key, string value)
        {
            if (!Declared(key, "write")) return;
            if (File.GetValue<string>("MENU", key, null) == value) return;
            File.SetValue("MENU", key, value);
            File.Save();
        }

        public void Set(string key, int value)
        {
            Set(key, value.ToString());
        }

        public void Set(string key, float value)
        {
            Set(key, value.ToString(CultureInfo.InvariantCulture));
        }

        // One-time seed: if the per-menu file lacks the key, carry the legacy value over.
        public void Migrate(string key, string legacyValue)
        {
            if (legacyValue == null) return;
            if (Get(key, null) == null) Set(key, legacyValue);
        }
    }
}