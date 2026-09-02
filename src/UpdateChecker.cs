using System;
using System.IO;
using System.Net;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using GTA;

namespace ARS
{
    public static class UpdateChecker
    {
        const string ReleasesApiUrl = "https://api.github.com/repos/Eddlm/autosport-racing-system-gta/releases/latest";
        const string UserAgent = "ARS-UpdateChecker";
        const int RequestTimeoutMs = 5000;

        static string _latestTag;
        static bool _notified;
        static bool _checked;

        public static void CheckLatestRelease()
        {
            if (_checked) return;
            _checked = true;

            Task.Factory.StartNew(() =>
            {
                try
                {
                    ServicePointManager.SecurityProtocol = SecurityProtocolType.Tls12;

                    HttpWebRequest request = (HttpWebRequest)WebRequest.Create(ReleasesApiUrl);
                    request.UserAgent = UserAgent;
                    request.Timeout = RequestTimeoutMs;
                    request.ReadWriteTimeout = RequestTimeoutMs;

                    using (HttpWebResponse response = (HttpWebResponse)request.GetResponse())
                    using (Stream stream = response.GetResponseStream())
                    using (StreamReader reader = new StreamReader(stream))
                    {
                        string json = reader.ReadToEnd();
                        Match match = Regex.Match(json, "\"tag_name\"\\s*:\\s*\"([^\"]*)\"");
                        if (match.Success) _latestTag = match.Groups[1].Value;
                        else ARS.Log(ARS.LogImportance.Info, "UpdateChecker: tag_name not found in GitHub response.");
                    }
                }
                catch (Exception ex)
                {
                    ARS.Log(ARS.LogImportance.Info, "UpdateChecker failed: " + ex.Message);
                }
            });
        }

        public static void TryNotify()
        {
            if (_notified || string.IsNullOrEmpty(_latestTag)) return;
            UI.Notify($"~b~[ARS]~w~ Latest release: ~y~{_latestTag}");
            _notified = true;
        }
    }
}
