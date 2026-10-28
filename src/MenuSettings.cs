// One settings row: type, limits, default and the ini key it lives under.
using GTA;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;

namespace ARS
{

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

        public bool TryGetFloat(string key, out float value)
        {
            value = 0f;
            string text = Get(key, null);
            return text != null && float.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value);
        }

        void Save()
        {
            List<string> lines = new List<string>();
            lines.Add("[MENU]");
            foreach (string key in SettingsRepair.DeclaredKeys(_fileName))
            {
                string value = File.GetValue<string>("MENU", key, null);
                if (value != null) lines.Add(key + "=" + value);
            }
            try
            {
                SettingsRepair.WriteAtomic(_path, lines);
            }
            catch (Exception ex)
            {
                ARS.Log(ARS.LogImportance.Error, "Settings: could not write " + _fileName + " - " + ex.Message, true);
            }
        }

        void SeedIfMissing(string key, string value)
        {
            if (value == null || File.GetValue<string>("MENU", key, null) != null) return;
            File.SetValue("MENU", key, value);
            Save();
        }

        public void Set(string key, string value)
        {
            if (!Declared(key, "write")) return;
            if (File.GetValue<string>("MENU", key, null) == value) return;
            File.SetValue("MENU", key, value);
            Save();
        }

        public void Set(string key, int value)
        {
            Set(key, value.ToString());
        }

        public void Set(string key, float value)
        {
            Set(key, value.ToString(CultureInfo.InvariantCulture));
        }

        public void Migrate(string key, string legacyValue)
        {
            if (legacyValue == null) return;
            if (Get(key, null) == null) Set(key, legacyValue);
        }
    }
}