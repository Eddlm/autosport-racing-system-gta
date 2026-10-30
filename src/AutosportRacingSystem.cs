// The outside world: menu and ini, the race cycle (grid, laps, results), the HUD and the debug toggles. Settings are read
// once into Options and written back atomically; a value the ini does not carry keeps its default. Every per-tick path
// fails soft: a bad frame logs once and the race survives instead of dying with it.
using GTA;
using GTA.Math;
using GTA.Native;
using LemonUI;
using LemonUI.Menus;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Threading;
using System.Windows.Forms;
using System.Xml;
using System.Text.RegularExpressions;
using System.Threading.Tasks;

namespace ARS
{
    public enum RelativePos { Unreachable, Ahead, Left, Right, Behind }
    public enum Team
    {
        None, Red, Blue, Cop, Crook
    }
    public enum RaceState
    {
        None, NotInitiated, Countdown, InProgress, Finished,
    }

    public enum PaceMode
    {
        Absolute, RelativeToMine,
    }

    public enum RubberbandMode
    {
        Natural, Artificial,
    }

    public enum AutofixMode
    {
        Off, AutoRepair, Invincible,
    }

    public enum Options
    {
        ShowInputs, ReverseRoute, HighDownforceOnline, ShowCheckpoints, ShowEdgeChevrons, ShowLeaderboard, ShowDriverNames, ShowTrackAnalysis, ShowAiLapTimes, LockLaneCentre, ShowBumps, ShowWheelSlip, ShowGsProjection, ShowGLine, ShowRouteRadius, ShowRivalry, ShowFrontSlip
    }
    public partial class ARS : Script
    {

        public static List<TrackStartInfo> TrackStartInfos = new List<TrackStartInfo>();

        public static List<TrackPoint> TrackPoints = new List<TrackPoint>();
        public static float RouteLengthMeters = 0f;

        public static List<CornerPoint> Corners = new List<CornerPoint>();

        public static int CornersRevision = 0;

        public static int CornerCeilingMeters = 200;

        public static readonly string[] CornerCeilingValues = { "20", "40", "60", "80", "100", "120", "140", "160", "180", "200" };

        public static int AiNitroPct = 100;
        public static readonly string[] AiNitroValues = { "50", "100", "150", "200" };

        public static List<Bump> Bumps = new List<Bump>();

        public static List<Crest> Crests = new List<Crest>();

        public static List<string> KnownTracks = new List<string>();

        public static Dictionary<string, float> ModelGripCache = new Dictionary<string, float>(StringComparer.OrdinalIgnoreCase);

        public static Dictionary<string, float> ModelTopSpeedMphCache = new Dictionary<string, float>(StringComparer.OrdinalIgnoreCase);

        public static Dictionary<string, float> ModelAccelCache = new Dictionary<string, float>(StringComparer.OrdinalIgnoreCase);

        public static Dictionary<string, bool> ModelElectricCache = new Dictionary<string, bool>(StringComparer.OrdinalIgnoreCase);

        public static Dictionary<string, string> ModelNameCache = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);

        public static Dictionary<string, float> ModelPaceIndexCache = new Dictionary<string, float>(StringComparer.OrdinalIgnoreCase);

        const float ElectricDrivePeak = 5f;

        public static float ComputePaceIndex(float topSpeedMph, float grip, float accelRaw, bool isElectric)
        {
            if (isElectric) accelRaw *= ElectricDrivePeak;
            float cornerSpeedMph = (float)(Math.Sqrt(grip * 9.8 * 50.0) * 2.23694);
            return topSpeedMph + cornerSpeedMph + accelRaw * 123.1f;
        }

        static bool? _electricNativeAvailable;

        public static bool IsElectricModel(int modelHash)
        {
            if (_electricNativeAvailable == null)
            {
                try { Function.Call<int>((Hash)0x1FCB07FE230B6639, modelHash); _electricNativeAvailable = true; }
                catch (Exception) { _electricNativeAvailable = false; Log(LogImportance.Info, "GET_IS_VEHICLE_ELECTRIC is unavailable on this game build - no car will be treated as electric."); }
            }
            return _electricNativeAvailable.Value && Function.Call<int>((Hash)0x1FCB07FE230B6639, modelHash) != 0;
        }

        static readonly HashSet<VehicleClass> BlacklistedVehicleClasses = new HashSet<VehicleClass>
        {
            VehicleClass.Motorcycles, VehicleClass.Cycles, VehicleClass.Boats, VehicleClass.Helicopters,
            VehicleClass.Planes, VehicleClass.Trains
        };

        public static int RaceReward = 0;

        public static MenuSettings RaceMenuStore;
        public static MenuSettings SettingsMenuStore;
        public static MenuSettings DebugMenuStore;
        public static MenuSettings DevelopmentMenuStore;

        public static bool HideHudMode = false;

        public static bool PlayerParticipating = false;

        public static bool NitrousEnabled = true;

        public static TipFrequency TipRate = TipFrequency.High;

        public static bool UseMenyooSkins = true;

        public static bool ForceFillGrid = true;
        public static float OverspeedCutPerTenthG = 0.2f;
        public static bool SmartTuning = true;

        public static bool BrakeLearning = true;

        public static bool BrakeLearningNotify = false;

        public static bool SteerTrace = false;

        public static bool GsLaneThrottleFloor = true;

        public static int OpenUpOnThrottleFloorPct = 60;
        public static readonly string[] OpenUpFloorChoices = { "10", "20", "30", "40", "50", "60", "70", "80", "90", "100" };

        public static bool SlipFromMemory = true;

        public static bool TcsEnabled = true;
        public static bool AbsEnabled = false;

        public static int CornerOffsetMph = 4;
        public static int RouteOffsetMph = 2;
        public static float SteerDampingGain = 0.5f;

        public static float DamperCountersteerMaxDeg = 2f;
        public static float SteerCeilingFactor = 0.9f;

        public static float SlipCutFloor = 0.4f;

        public const float SlipPeakRatio = 0.4f;

        public static int TcsSlipLimitPct = 100;
        public static int TcsTaperSpanPct = 50;

        public static float MaxCrestLift = 0.4f;
        public static float HillGripEffect = 1f;
        public static int RubberbandingPct = 0;
        public static RubberbandMode CurrentRubberbandMode = RubberbandMode.Natural;

        public static bool StagedSpawns = false;
        public static bool HotkeysEnabled = true;
        public static Keys MenuModifierKey = Keys.LShiftKey;
        public static Keys MenuKey = Keys.E;
        static readonly Keys[] MenuHotkeyKeys = Enum.GetValues(typeof(Keys)).Cast<Keys>().Where(key => (int)key >= 8 && (int)key <= 255).Distinct().OrderBy(key => (int)key).ToArray();
        public static readonly string[] MenuHotkeyValues = MenuHotkeyKeys.Select(key => ((int)key).ToString(CultureInfo.InvariantCulture)).ToArray();
        public static LogImportance LogLevel = LogImportance.Error;

        public static Racer DebugFocusRacer;

        public static Racer PlayerRacer;

        public static Dictionary<Options, bool> DebugToggles = new Dictionary<Options, bool>()
    {
        { Options.ShowInputs, false },
        { Options.ShowTrackAnalysis, false },
        { Options.ShowRivalry, false },
        { Options.ReverseRoute, false },
        { Options.HighDownforceOnline, false },
        { Options.ShowCheckpoints, true },
        { Options.ShowEdgeChevrons, false },
        { Options.ShowLeaderboard, true },
        { Options.ShowDriverNames, false },
        { Options.ShowAiLapTimes, false },
        { Options.LockLaneCentre, false },
        { Options.ShowBumps, false },
        { Options.ShowWheelSlip, false },
        { Options.ShowFrontSlip, false },
        { Options.ShowGsProjection, false },
        { Options.ShowGLine, false },
        { Options.ShowRouteRadius, false }
    };

        public const float SpectateCheckpointRadiusMeters = 400f;

        public static float LaneGainExponent = 1f;

        public static List<Prop> CustomProps = new List<Prop>();
        public static List<Prop> AutoGeneratedProps = new List<Prop>();

        public static List<Racer> LeaderboardFinish = new List<Racer>();

        public static Prop FreeCamRide = null;

        public static PedHash[] StreetRacerModels = { PedHash.Car3Guy2, PedHash.Vinewood02AFY, PedHash.Stwhi02AMY, PedHash.StrPunk02GMY, PedHash.Stbla02AMY };

        public static RaceState RaceStatus = RaceState.None;

        public static bool RaceIsRunning { get { return RaceStatus == RaceState.Countdown || RaceStatus == RaceState.InProgress; } }
        public static int RaceStartTime = 0;

        public static int ParkedDNFs = 0;

        public static bool DNFUnderTrack = true;

        public static bool NoCollision = false;

        bool _trackInstanced;
        bool _gridInstanced;
        int _intendedOpponents = 4;

        public static readonly int[] GridSizeChoices = { 0, 1, 2, 3, 4, 5, 6, 8, 10, 12, 14, 16, 18, 22, 26, 30, 40, 50, 60, 70, 80, 90, 100 };

        int GridSlotsAvailable => Math.Max(0, GridPositions.Count - 2);

        static string[] GridSizeLabels()
        {
            string[] labels = new string[GridSizeChoices.Length];
            for (int i = 0; i < GridSizeChoices.Length; i++) labels[i] = GridSizeChoices[i].ToString();
            return labels;
        }

        static int NearestGridSize(int stored)
        {
            int best = GridSizeChoices[0];
            foreach (int value in GridSizeChoices) if (Math.Abs(value - stored) < Math.Abs(best - stored)) best = value;
            return best;
        }
        List<int> _flareFx = new List<int>();
        FreeCamController _freeCam;

        public static ulong WheelsPtr = 0x0;
        public static ulong NumWheelsOffset = 0x0;
        static bool _nullCarReported;
        public static ulong WheelPowerOffset = 0x0;

        public static List<int> Other = new List<int> { 555004797, -399872228, -1447280105, 722686013 };
        public static List<int> Road = new List<int> { 1187676648, 282940568, -108464011, 1187676648, -1084640111, };
        public static List<int> Dirt = new List<int> { 1144315879, 510490462, -1907520769, -1885547121, -700658213, 2128369009,
            -1595148316,-765206029,509508168,1333033863,951832588,-840216541,-1907520769,510490462,-1942898710};
        public static List<int> Sand = new List<int> { 1288448767, };

        public static List<string> HelpMessages = new List<string>();

        public static List<Racer> Racers = new List<Racer>();

        public static List<Vector3> RouteNodes = new List<Vector3>();
        public static Dictionary<int, float> NodeHalfWidths = new Dictionary<int, float>();

        public static List<Prop> StartLineFlares = new List<Prop>();
        public static XmlDocument CurrentFile = null;

        int _raceTimedFinishMs = 0;
        float _timeScale = 1f;
        internal float TimeScale { get => _timeScale; set => _timeScale = value; }
        int _posUpdateTickMs = 0;
        int _longTickMs = 0;

        public enum TerrainTypes
        {
            Sand = 1288448767,
            RockySand = -1595148316,
            Gravel = -1885547121,
            GravelGrass = 2128369009,
            LooseGravel = 510490462,
            Rock = -840216541,
            MoreTarmac = 1187676648,
            Tarmac = 282940568,
            WetTarmac = 999829011,
            Grass = 1286696947,
            FullGrass = -461750719,
            ShortGrass = 1333033863,
        }

        bool _loaded = false;
        bool _isLoadingScript = false;
        Task _loadScriptTask = null;

        public void StartLoadScript()
        {
            if (_isLoadingScript) return;

            DisplayHelpTextTimed("Loading ~b~ARS.~w~ May take a while.", 20000);
            _loaded = false;
            _isLoadingScript = true;
            _loadScriptTask = Task.Run(() =>
            {
                RefreshCatalogs();
                FillKnownTracks(false);
            });
        }

        void HandleLoadScriptTask()
        {
            if (!_isLoadingScript || _loadScriptTask == null) return;
            if (!_loadScriptTask.IsCompleted) return;

            if (_loadScriptTask.IsFaulted)
            {
                _loaded = false;
                Exception ex = _loadScriptTask.Exception?.GetBaseException();
                Log(LogImportance.Error, "Initialization failed: " + (ex != null ? ex.Message : "Unknown error"), true);
                DisplayHelpTextTimed("~r~ARS failed to load. Check log.", 4000);
            }
            else
            {
                try
                {
                    BuildPowerCache();
                    RefreshPowerControls();
                    FillCachedCandidates(_intendedOpponents);
                    RefreshTrackList();
                    Log(LogImportance.Info, "Initialization complete.", true);
                    DisplayHelpTextTimed("~g~ARS has loaded.", 2000);
                    HelpMessages.Add("Press ~INPUT_SPRINT~ + ~INPUT_CONTEXT~ to open the ~b~ARSe~w~ menu.");
                    _loaded = true;
                }
                catch (Exception loadError)
                {
                    _loaded = false;
                    Log(LogImportance.Error, "Initialization failed after the background load: " + loadError.Message, true);
                    DisplayHelpTextTimed("~r~ARS failed to load. Check log.", 4000);
                }
            }

            _isLoadingScript = false;
            _loadScriptTask = null;
        }
        public ARS()
        {
            _freeCam = new FreeCamController(this);

            Tick += OnTick;
            KeyDown += OnKeyDown;
            Aborted += OnAbort;

            string legacyLogPath = ScriptsFolder + @"\Log.log";
            try { if (File.Exists(legacyLogPath) && !File.Exists(LogPath)) File.Move(legacyLogPath, LogPath); }
            catch (Exception) { }

            try { string staleBuildNumber = ScriptsFolder + @"\build-number.txt"; if (File.Exists(staleBuildNumber)) File.Delete(staleBuildNumber); }
            catch (Exception) { }
            try { if (File.Exists(LogPath)) File.Copy(LogPath, PreviousLogPath, true); }
            catch (Exception) { }
            File.WriteAllText(LogPath, "----------------------------");

            Log(LogImportance.Info, "Script initialized - " + DateTime.Now + " - build " + ReadBuildNumber(), true);
            File.AppendAllText(LogPath, "\n----------------------------");
            VerifyScriptBridge();
            LoadSettings();

            try
            {
                InitializeMenu();
            }
            catch (Exception menuError)
            {
                Log(LogImportance.Error, "Menu build failed: " + menuError);
            }

            string scriptName = "ARS";
            string scriptVer = "0.0.0.0";
            string scriptDate = "30/09/2018";

            Assembly assem = Assembly.GetExecutingAssembly();
            AssemblyName assemName = assem.GetName();
            Version ver = assemName.Version;
            scriptVer = ver.ToString() + " ~o~[PUBLIC BETA]~w~";
            scriptDate = File.GetLastWriteTimeUtc(ScriptsFolder + @"\ARS.dll").ToString();

            UI.Notify("~b~" + scriptName + "~g~e~w~" + "~y~ " + scriptVer + "~n~Build: ~g~" + ReadBuildNumber() + " ~w~(" + scriptDate + ")");

            UpdateChecker.CheckLatestRelease();
        }

        static string ReadBuildNumber()
        {
            try
            {
                AssemblyFileVersionAttribute attribute = (AssemblyFileVersionAttribute)Attribute.GetCustomAttribute(
                    Assembly.GetExecutingAssembly(), typeof(AssemblyFileVersionAttribute));
                if (attribute == null || string.IsNullOrEmpty(attribute.Version)) return "?";
                string[] parts = attribute.Version.Split('.');
                return parts.Length == 4 ? parts[3] : attribute.Version;
            }
            catch (Exception) { return "?"; }
        }

        static void VerifyScriptBridge()
        {
            try
            {
                Game.GenerateHash("ars");
            }
            catch (Exception ex)
            {
                Log(LogImportance.Error, "ScriptHookVDotNet2.dll is not usable with this ScriptHookVDotNet.asi build (" + ex.GetType().Name + ") - install ScriptHookVDotNet as one complete, matching set.", true);
                throw;
            }
        }

        Dictionary<string, string> _trackTags = new Dictionary<string, string>();

        public void FillKnownTracks(bool allowScriptYield = true)
        {
            _trackTags.Clear();
            TrackStartInfos.Clear();
            Log(LogImportance.Info, "Learning available tracks...");
            List<string> folders = Directory.GetDirectories(ScriptsFolder + @"\Tracks").ToList();
            folders.Add(ScriptsFolder + @"\Tracks");
            foreach (string dir in folders)
            {
                int count = 0;

                foreach (string st in Directory.EnumerateFiles(dir, "*.xml"))
                {
                    string n = System.IO.Path.GetFileName(st);
                    try
                    {
                        string tags = string.Join(", ", TrackRepository.ReadTrackTags(st));
                        Log(LogImportance.Info, st + " - [" + tags + "]");
                        _trackTags.Add(st, tags);
                        TrackStartInfo info = TrackRepository.ComputeTrackStartInfo(st);
                        if (info != null && !TrackStartInfos.Any(existing => existing.StartPosition.DistanceTo(info.StartPosition) < 1f)) TrackStartInfos.Add(info);
                    }
                    catch (Exception trackError)
                    {
                        Log(LogImportance.Error, "Track skipped (" + st + "): " + trackError.Message);
                    }

                    count++;
                    if (allowScriptYield && count > 5)
                    {
                        DisplayHelpTextTimed("Loading " + n, 5000);
                        count = 0;
                        Yield();
                    }
                }
            }

            KnownTracks = Directory.GetFiles(ScriptsFolder + @"\Tracks").ToList();
            Log(LogImportance.Info, "Done.");
            Log(LogImportance.Info, "-------------");
        }

        public void RefreshCatalogs()
        {
            ModelGripCache.Clear();
            ModelTopSpeedMphCache.Clear();
            ModelAccelCache.Clear();
            ModelElectricCache.Clear();
            ModelPaceIndexCache.Clear();
            ModelNameCache.Clear();
            Log(LogImportance.Info, "-------------");
            Log(LogImportance.Info, "Discovering the vehicle pool and tracks...");
            VehicleCatalog.FillPool(_vehiclePool);
            KnownTracks = Directory.GetFiles(ScriptsFolder + @"\Tracks").ToList();
            Log(LogImportance.Info, "Found " + _vehiclePool.Count + " vehicle files and " + KnownTracks.Count + " tracks.");
            Log(LogImportance.Info, "-------------");
        }

        public void BuildPowerCache()
        {
            VehicleCatalog.BuildPowerCache(_vehiclePool, ModelGripCache, ModelTopSpeedMphCache, ModelAccelCache, ModelElectricCache, ModelNameCache, BlacklistedVehicleClasses, text => Log(LogImportance.Info, text));
            BuildPaceIndex();
        }

        void BuildPaceIndex()
        {
            ModelPaceIndexCache.Clear();
            int indexed = 0;
            foreach (var kv in ModelGripCache)
            {
                float grip = kv.Value;
                if (float.IsNaN(grip) || float.IsInfinity(grip) || grip <= 0f) continue;
                float mph;
                if (!ModelTopSpeedMphCache.TryGetValue(kv.Key, out mph)) continue;
                if (float.IsNaN(mph) || float.IsInfinity(mph) || mph <= 0f) continue;
                float accel = 0f;
                ModelAccelCache.TryGetValue(kv.Key, out accel);
                bool isElectric = false;
                ModelElectricCache.TryGetValue(kv.Key, out isElectric);
                float pace = ComputePaceIndex(mph, grip, accel, isElectric);
                if (float.IsNaN(pace) || float.IsInfinity(pace)) continue;
                ModelPaceIndexCache[kv.Key] = pace;
                indexed++;
            }
            Log(LogImportance.Info, "BuildPaceIndex: " + indexed + " models indexed.");
        }

        static string RosterKeyFor(VehicleHash hash)
        {
            string name = hash.ToString().ToLowerInvariant();
            return Game.GenerateHash(name) == (int)hash ? name : ((int)hash).ToString();
        }

        void RefreshRoster()
        {
            RefreshCatalogs();
            BuildPowerCache();
            RefreshPowerControls();
        }

        void RefreshCarPoolItem(NativeListItem<string> item)
        {
            List<string> files = VehicleCatalog.ListRosterFiles().Select(VehicleCatalog.PoolLabel).ToList();
            if (files.SequenceEqual(_carPoolFiles, StringComparer.OrdinalIgnoreCase)) return;
            List<string> added = files.Where(f => !_carPoolFiles.Contains(f, StringComparer.OrdinalIgnoreCase)).ToList();
            List<string> removed = _carPoolFiles.Where(f => !files.Contains(f, StringComparer.OrdinalIgnoreCase)).ToList();
            string current = VehicleCatalog.PoolLabel(VehicleCatalog.SelectedRosterFile);
            string target = files.Find(f => string.Equals(f, current, StringComparison.OrdinalIgnoreCase));
            _carPoolFiles = files;
            bool lost = target == null;
            if (lost) { target = files.Find(f => string.Equals(f, VehicleCatalog.PoolLabel("cars.txt"), StringComparison.OrdinalIgnoreCase)); if (target == null) target = files[0]; }

            _carPoolRefreshing = true;
            try
            {
                item.SelectedIndex = 0;
                item.Items = new List<string>(files);
                item.SelectedIndex = Math.Max(0, item.Items.FindIndex(i => string.Equals(i, target, StringComparison.OrdinalIgnoreCase)));
            }
            finally { _carPoolRefreshing = false; }
            if (lost) { string fallback = VehicleCatalog.PoolFileName(target); VehicleCatalog.SelectRosterFile(fallback); SaveRacerSetting("VehiclePool", fallback); RefreshRoster(); }
            string changes = (added.Count > 0 ? " +" + string.Join(", ", added) : "") + (removed.Count > 0 ? " -" + string.Join(", ", removed) : "");
            UI.Notify("~b~[ARS]:~w~ Car pools changed:" + changes + " (" + files.Count + " available)." + (lost ? " " + current + " is gone - using " + target + "." : ""));
        }

        public static float SignedLaneOffset(Vector3 pos, Vector3 refPoint, Vector3 refDir)
        {
            Vector3 right = Vector3.Cross(refDir, Vector3.WorldUp);
            Vector3 rPos = pos - refPoint;

            return Vector3.Dot(right, rPos);

        }

        public static Vector3 EntityRelativeOffset(Entity reference, Entity ent)
        {
            Vector3 pos = ent.Position;
            return Function.Call<Vector3>(Hash.GET_OFFSET_FROM_ENTITY_GIVEN_WORLD_COORDS, reference, pos.X, pos.Y, pos.Z);
        }
        public static Vector3 EntityRelativeOffset(Entity reference, Vector3 pos)
        {
            return Function.Call<Vector3>(Hash.GET_OFFSET_FROM_ENTITY_GIVEN_WORLD_COORDS, reference, pos.X, pos.Y, pos.Z);
        }

        public static float EngineTopSpeed(Vehicle v)
        {

            return Function.Call<float>((Hash)0xF417C2502FFFED43, Function.Call<int>(Hash.GET_ENTITY_MODEL, v));
        }

        public static float Remap(float x, float in_min, float in_max, float out_min, float out_max, bool clamp = false)
        {
            float r = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
            if (clamp) r = Clamp(r, out_min, out_max);
            return r;
        }

        public static float MapGamma(float value, float in_min, float in_max, float min, float max, float gamma, bool clamp = false)
        {
            if (in_max == in_min) return min;
            float t = (value - in_min) / (in_max - in_min);
            if (clamp) t = Clamp(t, 0f, 1f);
            t = (float)Math.Pow(t, gamma);
            return min + t * (max - min);
        }

        public static float Clamp(float val, float min, float max)
        {
            if (val.CompareTo(min) < 0) return min;
            else if (val.CompareTo(max) > 0) return max;
            else return val;
        }

        public static bool IsBetween(float value, float min, float max)
        {
            return value >= min && value <= max;
        }

        public static float RadToDeg(float rad)
        {
            return (rad * (180.0f / (float)Math.PI));
        }

        public static float DegToRad(float angle)
        {
            return (float)(Math.PI * angle / 180.0f);
        }

        int _countdownTickMs = 0;
        int _maxCountdown = 7;
        int _countdown = 7;

        public static Scaleform InstructionalScaleform = new Scaleform("INSTRUCTIONAL_BUTTONS");
        public static Scaleform Racertext = null;
        public static Scaleform DebugFrontend = new Scaleform("instructional_buttons");
        public static Scaleform Floatingtext = new Scaleform("HUD_FLOATING_HELP_TEXT");
        public static Scaleform CountdownScaleform = new Scaleform("MP_BIG_MESSAGE_FREEMODE");

        public static float PowerTargetScale = 0.52f;

        public static PaceMode CurrentPaceMode = PaceMode.RelativeToMine;
        public static float PaceOffsetScale = 0f;

        public static int PaceDeviationLimit = 0;

        public static string ScriptsFolder = @"scripts\AutosportRacingSystem";

        public static string SettingsFolder => ScriptsFolder + @"\Settings";

        public static string LogPath => Path.GetFullPath(Path.Combine(ScriptsFolder, "..", "..", "ARS.log"));
        public static string PreviousLogPath => Path.GetFullPath(Path.Combine(ScriptsFolder, "..", "..", "ARS.prev.log"));

        public static List<string> HardcodedRoster = null;

        List<string> _vehiclePool = new List<string>();

        List<string> _carPoolFiles = new List<string>();
        bool _carPoolRefreshing;
        public static int TrackListPos = 0;
        readonly ObjectPool _menuPool = new ObjectPool();
        NativeMenu _arsMenu;
        NativeMenu _raceMenu;

        NativeListItem<string> _trackListItem;
        readonly List<string> _trackListPaths = new List<string>();
        string _selectedTrackPath = null;
        NativeListItem<string> _gridSizeItem;
        NativeListItem<string> _powerTargetItem;
        NativeListItem<string> _paceModeItem;
        NativeListItem<string> _paceOffsetItem;
        readonly List<float> _powerTargetValues = new List<float>();
        readonly List<float> _paceOffsetValues = new List<float>();

        float _resolvedPaceTarget = 0.52f;

        float _lastKnownPlayerPace = float.NaN;

        readonly Dictionary<NativeMenu, NativeMenu> _menuParents = new Dictionary<NativeMenu, NativeMenu>();
        NativeSubmenuItem AddSub(NativeMenu parent, NativeMenu child)
        {
            NativeSubmenuItem item = parent.AddSubMenu(child);
            _menuParents[child] = parent;
            return item;
        }
        int ApplyMenuTrails(NativeMenu parent, string parentTrail)
        {
            int written = 0;
            foreach (KeyValuePair<NativeMenu, NativeMenu> edge in _menuParents)
            {
                if (edge.Value != parent) continue;
                string trail = parentTrail + " > " + edge.Key.Title.Text;
                edge.Key.Subtitle = trail;
                written += 1 + ApplyMenuTrails(edge.Key, trail);
            }
            return written;
        }

        void InitializeMenu()
        {
            _selectedTrackPath = RaceMenuStore.Get("Track", null);

            _arsMenu = new NativeMenu("ARS", "MAIN MENU")
            {
                UseMouse = false,
                DisableControls = true,
                Alignment = Alignment.Right
            };

            _raceMenu = new NativeMenu("Race", "Race")
            {
                UseMouse = false,
                DisableControls = true,
                Alignment = Alignment.Right
            };

            _trackListItem = new NativeListItem<string>("Select Track", "Pick the race to instance.", Array.Empty<string>());
            _trackListItem.ItemChanged += (sender, args) =>
            {
                if (args.Index >= 0 && args.Index < _trackListPaths.Count)
                {
                    _selectedTrackPath = _trackListPaths[args.Index];
                    RaceMenuStore.Set("Track", _selectedTrackPath);
                }
            };
            _raceMenu.Add(_trackListItem);
            HookListTextPicker(_trackListItem);

            NativeListItem<string> lapsItem = new NativeListItem<string>("Laps", "Number of laps before the race is considered finished.", SettingsRepair.OptionsFor("Laps"));
            lapsItem.ItemChanged += (sender, args) => RaceMenuStore.Set("Laps", lapsItem.Items[args.Index]);
            int laps = RaceMenuStore.GetInt("Laps", 6);
            lapsItem.SelectedIndex = Math.Max(0, lapsItem.Items.IndexOf(laps.ToString()));
            _raceMenu.Add(lapsItem);
            HookListTextPicker(lapsItem);

            AddDebugCheckbox(_raceMenu, Options.ReverseRoute, "Reverse Route", "Race the loaded route in reverse.", null, RaceMenuStore);

            NativeItem instanceTrackItem = new NativeItem("Load Track", "Load the selected track and teleport to it.");
            instanceTrackItem.Activated += (sender, args) =>
            {
                _raceMenu.Visible = false;
                InstanceTrack();
                _raceMenu.Visible = true;
            };
            _raceMenu.Add(instanceTrackItem);

            _gridSizeItem = new NativeListItem<string>("Target Grid Size", "Target number of AI opponents for the grid. 0 races the track alone.", GridSizeLabels());
            _gridSizeItem.ItemChanged += (sender, args) =>
            {
                if (args.Index < 0 || args.Index >= GridSizeChoices.Length) return;
                _intendedOpponents = GridSizeChoices[args.Index];
                RaceMenuStore.Set("GridSize", _intendedOpponents);
            };
            _intendedOpponents = NearestGridSize(RaceMenuStore.GetInt("GridSize", 8));
            _gridSizeItem.SelectedIndex = Array.IndexOf(GridSizeChoices, _intendedOpponents);
            _raceMenu.Add(_gridSizeItem);
            HookListTextPicker(_gridSizeItem);

            NativeMenu piSettingsMenu = new NativeMenu("PI Settings", "PI Settings", "The performance index the grid is drawn from, and how close to it a rival may sit.")
            {
                UseMouse = false,
                DisableControls = true,
                Alignment = Alignment.Right
            };
            _paceModeItem = new NativeListItem<string>("Grid PI Mode", "How the grid's performance index is chosen: Absolute uses PI Target; Relative To Mine uses your car's PI plus PI Offset.", EnumLabels<PaceMode>());
            _paceModeItem.ItemChanged += (sender, args) =>
            {
                CurrentPaceMode = (PaceMode)args.Index;
                RaceMenuStore.Set("PaceMode", CurrentPaceMode.ToString());
                ApplyPaceModeUI();
            };
            piSettingsMenu.Add(_paceModeItem);
            HookListTextPicker(_paceModeItem);

            _paceOffsetItem = new NativeListItem<string>("PI Offset", "Field PI = your car's PI plus this offset, resolved when the grid spawns.", Array.Empty<string>());
            for (int offset = -10; offset <= 10; offset++)
            {
                _paceOffsetValues.Add(offset);
                _paceOffsetItem.Items.Add(offset.ToString("+0;-0;0", CultureInfo.InvariantCulture));
            }
            _paceOffsetItem.ItemChanged += (sender, args) =>
            {
                if (args.Index >= 0 && args.Index < _paceOffsetValues.Count)
                {
                    PaceOffsetScale = _paceOffsetValues[args.Index];
                    RaceMenuStore.Set("PaceOffset", PaceOffsetScale.ToString(CultureInfo.InvariantCulture));
                    ApplyPaceModeUI();
                }
            };
            piSettingsMenu.Add(_paceOffsetItem);
            HookListTextPicker(_paceOffsetItem);

            _powerTargetItem = new NativeListItem<string>("PI Target", "Fixed performance index the grid is selected around.", Array.Empty<string>());
            _powerTargetItem.ItemChanged += (sender, args) =>
            {
                if (args.Index >= 0 && args.Index < _powerTargetValues.Count)
                {
                    PowerTargetScale = _powerTargetValues[args.Index];
                    RaceMenuStore.Set("PaceTarget", PowerTargetScale);
                    ApplyPaceModeUI();
                }
            };
            piSettingsMenu.Add(_powerTargetItem);
            HookListTextPicker(_powerTargetItem);
            NativeListItem<string> paceDeviationItem = new NativeListItem<string>("Max PI Deviation", "How far a rival's PI may sit from the grid's target. Off takes the closest cars however far.", SettingsRepair.OptionsFor("PaceDeviation"));
            paceDeviationItem.ItemChanged += (sender, args) =>
            {
                PaceDeviationLimit = ParseDeviationLimit(paceDeviationItem.Items[args.Index]);
                RaceMenuStore.Set("PaceDeviation", paceDeviationItem.Items[args.Index]);
            };
            paceDeviationItem.SelectedIndex = Math.Max(0, paceDeviationItem.Items.IndexOf(PaceDeviationLimit > 0 ? PaceDeviationLimit.ToString(CultureInfo.InvariantCulture) : "Off"));
            piSettingsMenu.Add(paceDeviationItem);
            HookListTextPicker(paceDeviationItem);
            AddSub(_raceMenu, piSettingsMenu);

            _paceModeItem.SelectedIndex = Math.Max(0, _paceModeItem.Items.IndexOf(EnumLabel(CurrentPaceMode)));

            NativeItem instanceGridItem = new NativeItem("Build Grid", "Spawn the AI grid with current settings. Hit again to re-instance after changing PI/size.");
            instanceGridItem.Activated += (sender, args) =>
            {
                _raceMenu.Visible = false;
                InstanceGrid();
                _raceMenu.Visible = true;
            };
            _raceMenu.Add(instanceGridItem);

            int spawnTrackSlot = _raceMenu.Items.IndexOf(instanceTrackItem);
            int spawnGridSlot = _raceMenu.Items.IndexOf(instanceGridItem);

            NativeItem startItem = new NativeItem("Start Race", "Starts the race, loading the track and building the grid first if needed. You race from the grid or watch from the sideline.");
            startItem.Activated += (sender, args) =>
            {
                _raceMenu.Visible = false;
                StartRace();
            };
            _raceMenu.Add(startItem);

            NativeItem restartItem = new NativeItem("Restart Race", "Re-place the same cars on the grid and restart the race.");
            restartItem.Activated += (sender, args) =>
            {
                _raceMenu.Visible = false;
                if (!PlayerParticipating) { StartRaceFromMenu(); return; }
                _raceTimedFinishMs = 0;
                SetupRace(true, true);
            };
            _raceMenu.Add(restartItem);

            NativeItem resetItem = new NativeItem("Reset To Track", "Teleport your car beside the closest track node at 10 mph, facing the route. Races only.");
            resetItem.Activated += (sender, args) =>
            {
                _arsMenu.Visible = false;
                ResetToTrack();
            };
            _arsMenu.Shown += (sender, args) => resetItem.Enabled = RaceIsRunning;

            NativeItem endItem = new NativeItem("End Race", "Tear down the current race and clean everything.");
            endItem.Activated += (sender, args) =>
            {
                _arsMenu.Visible = false;
                CleanEverything();
            };

            NativeMenu creatorMenu = new NativeMenu("Track Creator", "Track Creator", "Record a new race route with the free camera.")
            {
                UseMouse = false,
                DisableControls = true,
                Alignment = Alignment.Right
            };
            NativeItem startCreatorItem = new NativeItem("Start Creator", "Record a new route from the free camera: place the start line, then widen or tighten with the Weapon Wheel and apply each section.");
            startCreatorItem.Activated += (sender, args) =>
            {
                creatorMenu.Visible = false;
                StartTrackCreator();
            };
            creatorMenu.Add(startCreatorItem);

            NativeItem saveCreatorItem = new NativeItem("Save Track", "Write the recorded route to a new file in Tracks.");
            saveCreatorItem.Activated += (sender, args) =>
            {
                creatorMenu.Visible = false;
                SaveTrackFromCreator();
            };
            creatorMenu.Add(saveCreatorItem);

            NativeItem exitCreatorItem = new NativeItem("Exit Creator", "Leave route-creation mode and discard the recorded route.");
            exitCreatorItem.Activated += (sender, args) =>
            {
                creatorMenu.Visible = false;
                CleanEverything();
            };
            creatorMenu.Add(exitCreatorItem);

            NativeMenu rivalsMenu = new NativeMenu("Rivals", "Rivals", "The rivals' cars, how they look, and how they drive.")
            {
                UseMouse = false,
                DisableControls = true,
                Alignment = Alignment.Right
            };

            NativeListItem<string> gridSortItem = new NativeListItem<string>("Grid Sorting", "Sorts the rivals only; your car is added last - the back on a circuit, the front on point-to-point.", EnumLabels<GridSort>());
            gridSortItem.ItemChanged += (sender, args) => SaveRacerSetting("GridSorting", ((GridSort)args.Index).ToString());
            gridSortItem.SelectedIndex = Math.Max(0, gridSortItem.Items.IndexOf(EnumLabel(ParseEnum(SettingsMenuStore.Get("GridSorting", "Random"), GridSort.Random))));
            rivalsMenu.Add(gridSortItem);
            HookListTextPicker(gridSortItem);

            List<string> poolFiles = VehicleCatalog.ListRosterFiles();
            string storedPool = SettingsMenuStore.Get("VehiclePool", "cars.txt");
            if (!poolFiles.Contains(storedPool, StringComparer.OrdinalIgnoreCase)) { storedPool = "cars.txt"; SaveRacerSetting("VehiclePool", storedPool); }
            VehicleCatalog.SelectRosterFile(storedPool);
            _carPoolFiles = poolFiles.Select(VehicleCatalog.PoolLabel).ToList();
            NativeListItem<string> carPoolItem = new NativeListItem<string>("Car Roster", "Vehicles is re-read each time this menu opens, so a roster added there appears without a reload.", _carPoolFiles.ToArray());
            carPoolItem.ItemChanged += (sender, args) =>
            {
                if (_carPoolRefreshing) return;
                string pool = VehicleCatalog.PoolFileName(carPoolItem.Items[args.Index]);
                VehicleCatalog.SelectRosterFile(pool);
                SaveRacerSetting("VehiclePool", pool);
                RefreshRoster();
                UI.Notify("~b~[ARS]:~w~ Car pool: " + carPoolItem.Items[args.Index] + " (" + _vehiclePool.Count + " cars).");
            };

            _carPoolRefreshing = true;
            string currentPool = VehicleCatalog.PoolLabel(VehicleCatalog.SelectedRosterFile);
            carPoolItem.SelectedIndex = Math.Max(0, carPoolItem.Items.FindIndex(i => string.Equals(i, currentPool, StringComparison.OrdinalIgnoreCase)));
            _carPoolRefreshing = false;
            rivalsMenu.Add(carPoolItem);
            HookListTextPicker(carPoolItem);
            rivalsMenu.Shown += (sender, args) => RefreshCarPoolItem(carPoolItem);

            NativeCheckboxItem forceFillItem = new NativeCheckboxItem("Repeat Cars To Fill Grid", "Off, the grid is only as long as the pool allows.", ForceFillGrid);
            forceFillItem.CheckboxChanged += (sender, args) =>
            {
                ForceFillGrid = forceFillItem.Checked;
                SaveRacerSetting("ForceFillGrid", ForceFillGrid.ToString());
            };
            rivalsMenu.Add(forceFillItem);

            NativeCheckboxItem tuningItem = new NativeCheckboxItem("Auto Styling", "Restyle the spawned cars only, skipping your own and any car that already has a mod kit. Runs during the countdown.", SmartTuning);
            tuningItem.CheckboxChanged += (sender, args) =>
            {
                SmartTuning = tuningItem.Checked;
                SaveRacerSetting("SmartTuning", SmartTuning.ToString());
            };
            rivalsMenu.Add(tuningItem);

            NativeCheckboxItem menyooItem = new NativeCheckboxItem("Menyoo Skins", "Cosmetic only, and only on the cars ARS spawns: your own car is never touched.", UseMenyooSkins);
            menyooItem.CheckboxChanged += (sender, args) =>
            {
                UseMenyooSkins = menyooItem.Checked;
                SaveRacerSetting("UseMenyooSkins", UseMenyooSkins.ToString());
            };
            rivalsMenu.Add(menyooItem);

            NativeMenu rivalPaceMenu = new NativeMenu("Rival Pace", "Rival Pace", "For rivals that brake too early or arrive too hot.")
            {
                UseMouse = false,
                DisableControls = true,
                Alignment = Alignment.Right
            };
            NativeMenu rivalTractionMenu = new NativeMenu("Rival Traction", "Rival Traction", "For rivals that spin their wheels or lock up under braking.")
            {
                UseMouse = false,
                DisableControls = true,
                Alignment = Alignment.Right
            };
            NativeMenu rivalSteeringMenu = new NativeMenu("Rival Steering", "Rival Steering", "For rivals that wobble on the way in or wash wide.")
            {
                UseMouse = false,
                DisableControls = true,
                Alignment = Alignment.Right
            };
            AddSub(rivalsMenu, rivalPaceMenu);
            AddSub(rivalsMenu, rivalTractionMenu);
            AddSub(rivalsMenu, rivalSteeringMenu);

            NativeListItem<string> cornerOffsetItem = new NativeListItem<string>("Corner Speed Offset (mph)", "Raises the speed rivals brake for at the next corner. Only the slower of this and Road Curvature Offset applies.", SettingsRepair.OptionsFor("CornerOffset"));
            cornerOffsetItem.ItemChanged += (sender, args) =>
            {
                CornerOffsetMph = int.Parse(cornerOffsetItem.Items[args.Index], CultureInfo.InvariantCulture);
                SaveRacerSetting("CornerOffset", cornerOffsetItem.Items[args.Index]);
            };
            cornerOffsetItem.SelectedIndex = Math.Max(0, cornerOffsetItem.Items.IndexOf(SettingsMenuStore.GetInt("CornerOffset", CornerOffsetMph).ToString(CultureInfo.InvariantCulture)));
            rivalPaceMenu.Add(cornerOffsetItem);
            HookListTextPicker(cornerOffsetItem);

            NativeListItem<string> routeOffsetItem = new NativeListItem<string>("Road Curvature Offset (mph)", "Raises the speed the road's curvature allows ahead. Only the slower of this and Corner Speed Offset applies.", SettingsRepair.OptionsFor("RouteOffset"));
            routeOffsetItem.ItemChanged += (sender, args) =>
            {
                RouteOffsetMph = int.Parse(routeOffsetItem.Items[args.Index], CultureInfo.InvariantCulture);
                SaveRacerSetting("RouteOffset", routeOffsetItem.Items[args.Index]);
            };
            routeOffsetItem.SelectedIndex = Math.Max(0, routeOffsetItem.Items.IndexOf(SettingsMenuStore.GetInt("RouteOffset", RouteOffsetMph).ToString(CultureInfo.InvariantCulture)));
            rivalPaceMenu.Add(routeOffsetItem);
            HookListTextPicker(routeOffsetItem);

            NativeCheckboxItem brakeLearningItem = new NativeCheckboxItem("Brake Learning", "Each corner learns how hard rivals dare to brake: brake late and it brakes harder next lap, slide through and it loses 0.1.", BrakeLearning);
            brakeLearningItem.CheckboxChanged += (sender, args) =>
            {
                BrakeLearning = brakeLearningItem.Checked;
                SaveRacerSetting("BrakeLearning", BrakeLearning.ToString());
            };
            rivalPaceMenu.Add(brakeLearningItem);

            NativeListItem<string> maxCrestLiftItem = new NativeListItem<string>("Max Crest Lift (g)", "The crest lift, in g, the car may feel before the no-fly gate slows it: 0.1 is very conservative, 2.0 only the sharpest features.", SettingsRepair.OptionsFor("MaxCrestLift"));
            maxCrestLiftItem.ItemChanged += (sender, args) =>
            {
                MaxCrestLift = float.Parse(maxCrestLiftItem.Items[args.Index], CultureInfo.InvariantCulture);
                SaveRacerSetting("MaxCrestLift", maxCrestLiftItem.Items[args.Index]);
            };
            maxCrestLiftItem.SelectedIndex = Math.Max(0, maxCrestLiftItem.Items.IndexOf(SettingsMenuStore.GetFloat("MaxCrestLift", 0.4f).ToString("0.0", CultureInfo.InvariantCulture)));
            rivalPaceMenu.Add(maxCrestLiftItem);
            HookListTextPicker(maxCrestLiftItem);

            NativeCheckboxItem gsLaneFloorItem = new NativeCheckboxItem("Gs Lane Throttle Floor", "Corner exit aid: while the line ahead points at the inside of the road, the throttle cannot fall below a floor that grows toward the inside edge and never exceeds what TCS and stability allow.", GsLaneThrottleFloor);
            gsLaneFloorItem.CheckboxChanged += (sender, args) =>
            {
                GsLaneThrottleFloor = gsLaneFloorItem.Checked;
                SaveRacerSetting("GsLaneThrottleFloor", GsLaneThrottleFloor.ToString());
            };
            rivalPaceMenu.Add(gsLaneFloorItem);

            NativeListItem<string> openUpItem = new NativeListItem<string>("Open Up on Throttle", "Share of the normal steering clamp left at full pedal: 100 % is the rule off, 40 % opens the wheel to two fifths of its allowance under power. Asking for throttle with the wheel wound on spends the front tyre on lateral force, and that force is not pushing the car. The reduction is spent by half pedal.", SettingsRepair.OptionsFor("OpenUpOnThrottleFloorPct"));
            openUpItem.ItemChanged += (sender, args) =>
            {
                OpenUpOnThrottleFloorPct = int.Parse(openUpItem.Items[args.Index], CultureInfo.InvariantCulture);
                SaveRacerSetting("OpenUpOnThrottleFloorPct", OpenUpOnThrottleFloorPct.ToString(CultureInfo.InvariantCulture));
            };
            openUpItem.SelectedIndex = Math.Max(0, openUpItem.Items.IndexOf(SettingsMenuStore.GetInt("OpenUpOnThrottleFloorPct", OpenUpOnThrottleFloorPct).ToString(CultureInfo.InvariantCulture)));
            rivalPaceMenu.Add(openUpItem);
            HookListTextPicker(openUpItem);

            NativeListItem<string> hillEffectItem = new NativeListItem<string>("Slope Loss (%)", "Share of the tuned slope speed cut: 0% ignores slopes, 100% is the tuned cut, 200% doubles it. Pitch counts both ways.", SettingsRepair.OptionsFor("HillGripEffect"));
            hillEffectItem.ItemChanged += (sender, args) =>
            {
                HillGripEffect = int.Parse(hillEffectItem.Items[args.Index], CultureInfo.InvariantCulture) * 0.01f;
                SaveRacerSetting("HillGripEffect", hillEffectItem.Items[args.Index]);
            };
            hillEffectItem.SelectedIndex = Math.Max(0, hillEffectItem.Items.IndexOf(SettingsMenuStore.GetInt("HillGripEffect", 100).ToString(CultureInfo.InvariantCulture)));
            rivalPaceMenu.Add(hillEffectItem);
            HookListTextPicker(hillEffectItem);

            NativeListItem<string> overspeedItem = new NativeListItem<string>("Overspeed Correction", "Throttle lifted per 0.1 G of surplus when the car pulls harder than its wheels explain. Arms on a settled car above 30 mph on a straight.", SettingsRepair.OptionsFor("OverspeedCut"));
            overspeedItem.ItemChanged += (sender, args) =>
            {
                OverspeedCutPerTenthG = ParseOverspeedCut(overspeedItem.Items[args.Index]);
                SaveRacerSetting("OverspeedCut", overspeedItem.Items[args.Index]);
            };
            overspeedItem.SelectedIndex = Math.Max(0, overspeedItem.Items.IndexOf(OverspeedCutPerTenthG > 0f ? OverspeedCutPerTenthG.ToString("0.0", CultureInfo.InvariantCulture) : "Off"));
            rivalPaceMenu.Add(overspeedItem);
            HookListTextPicker(overspeedItem);

            NativeCheckboxItem tcsItem = new NativeCheckboxItem("TCS", "Cap AI throttle against a wheelspin target. Off lets AI throttle go unlimited - no traction control at all.", TcsEnabled);
            tcsItem.CheckboxChanged += (sender, args) =>
            {
                TcsEnabled = tcsItem.Checked;
                SaveRacerSetting("TcsEnabled", TcsEnabled.ToString());
            };
            rivalTractionMenu.Add(tcsItem);

            NativeListItem<string> tcsLimitItem = new NativeListItem<string>("TCS Wheelspin Limit (%)", "The wheelspin the AI throttles to, as a share of the peak-traction slip: 100% is the peak, 80% just under it.", SettingsRepair.OptionsFor("TcsSlipLimitPct"));
            tcsLimitItem.ItemChanged += (sender, args) =>
            {
                TcsSlipLimitPct = int.Parse(tcsLimitItem.Items[args.Index], CultureInfo.InvariantCulture);
                SaveRacerSetting("TcsSlipLimitPct", tcsLimitItem.Items[args.Index]);
            };
            tcsLimitItem.SelectedIndex = Math.Max(0, tcsLimitItem.Items.IndexOf(SettingsMenuStore.GetInt("TcsSlipLimitPct", TcsSlipLimitPct).ToString(CultureInfo.InvariantCulture)));
            rivalTractionMenu.Add(tcsLimitItem);
            HookListTextPicker(tcsLimitItem);

            NativeListItem<string> tcsSpanItem = new NativeListItem<string>("TCS Cut Softness (%)", "How far past the limit the cut spans before the throttle floor: the ceiling is the limit plus this share of the peak slip.", SettingsRepair.OptionsFor("TcsTaperSpanPct"));
            tcsSpanItem.ItemChanged += (sender, args) =>
            {
                TcsTaperSpanPct = int.Parse(tcsSpanItem.Items[args.Index], CultureInfo.InvariantCulture);
                SaveRacerSetting("TcsTaperSpanPct", tcsSpanItem.Items[args.Index]);
            };
            tcsSpanItem.SelectedIndex = Math.Max(0, tcsSpanItem.Items.IndexOf(SettingsMenuStore.GetInt("TcsTaperSpanPct", TcsTaperSpanPct).ToString(CultureInfo.InvariantCulture)));
            rivalTractionMenu.Add(tcsSpanItem);
            HookListTextPicker(tcsSpanItem);

            NativeCheckboxItem absItem = new NativeCheckboxItem("ABS", "GTA forces native ABS on non-player driven cars. I'm working on undoing that.", AbsEnabled);
            absItem.CheckboxChanged += (sender, args) =>
            {
                AbsEnabled = absItem.Checked;
                SaveRacerSetting("AbsEnabled", AbsEnabled.ToString());
            };
            rivalTractionMenu.Add(absItem);

            NativeListItem<string> steerKDItem = new NativeListItem<string>("Steer Damping", "Yaw-rate damping in seconds. Lower is crisper; higher opposes rotation more.", SettingsRepair.OptionsFor("SteerDampingGain"));
            steerKDItem.ItemChanged += (sender, args) =>
            {
                SteerDampingGain = float.Parse(steerKDItem.Items[args.Index], CultureInfo.InvariantCulture);
                SaveRacerSetting("SteerDampingGain", steerKDItem.Items[args.Index]);
            };
            steerKDItem.SelectedIndex = Math.Max(0, steerKDItem.Items.IndexOf(SettingsMenuStore.GetFloat("SteerDampingGain", SteerDampingGain).ToString("0.00", CultureInfo.InvariantCulture)));
            rivalSteeringMenu.Add(steerKDItem);
            HookListTextPicker(steerKDItem);

            NativeListItem<string> damperCounterItem = new NativeListItem<string>("Damper Countersteer", "How far past neutral the damper may turn against the car's rotation, in degrees. 0 cancels the command and no more.", SettingsRepair.OptionsFor("DamperCountersteerMaxDeg"));
            damperCounterItem.ItemChanged += (sender, args) =>
            {
                DamperCountersteerMaxDeg = float.Parse(damperCounterItem.Items[args.Index], CultureInfo.InvariantCulture);
                SaveRacerSetting("DamperCountersteerMaxDeg", damperCounterItem.Items[args.Index]);
            };
            damperCounterItem.SelectedIndex = Math.Max(0, damperCounterItem.Items.IndexOf(SettingsMenuStore.GetFloat("DamperCountersteerMaxDeg", DamperCountersteerMaxDeg).ToString("0.00", CultureInfo.InvariantCulture)));
            rivalSteeringMenu.Add(damperCounterItem);
            HookListTextPicker(damperCounterItem);

            NativeListItem<string> steerCeilingFactorItem = new NativeListItem<string>("Peak Slip Share", "How much of the tyre's peak slip the limiter allows before it cuts: 1.00 cuts exactly at the peak, below 1.00 cuts earlier and the car never uses its grip, above 1.00 lets it slide, 2.00 barely cuts at all.", SettingsRepair.OptionsFor("SteerCeilingFactor"));
            steerCeilingFactorItem.ItemChanged += (sender, args) =>
            {
                SteerCeilingFactor = float.Parse(steerCeilingFactorItem.Items[args.Index], CultureInfo.InvariantCulture);
                SaveRacerSetting("SteerCeilingFactor", steerCeilingFactorItem.Items[args.Index]);
            };
            steerCeilingFactorItem.SelectedIndex = Math.Max(0, steerCeilingFactorItem.Items.IndexOf(SettingsMenuStore.GetFloat("SteerCeilingFactor", SteerCeilingFactor).ToString("0.00", CultureInfo.InvariantCulture)));
            rivalSteeringMenu.Add(steerCeilingFactorItem);
            HookListTextPicker(steerCeilingFactorItem);

            NativeListItem<string> slipCutFloorItem = new NativeListItem<string>("Slip Cut Floor", "The deepest the limiter may cut the wheel: 0.40 leaves 40% of what was asked, 0.00 removes the request entirely and the car goes straight on."
                , SettingsRepair.OptionsFor("SlipCutFloor"));
            slipCutFloorItem.ItemChanged += (sender, args) =>
            {
                SlipCutFloor = float.Parse(slipCutFloorItem.Items[args.Index], CultureInfo.InvariantCulture);
                SaveRacerSetting("SlipCutFloor", slipCutFloorItem.Items[args.Index]);
            };
            slipCutFloorItem.SelectedIndex = Math.Max(0, slipCutFloorItem.Items.IndexOf(SettingsMenuStore.GetFloat("SlipCutFloor", SlipCutFloor).ToString("0.00", CultureInfo.InvariantCulture)));
            rivalSteeringMenu.Add(slipCutFloorItem);
            HookListTextPicker(slipCutFloorItem);

            NativeMenu rulesMenu = new NativeMenu("Race Rules", "Race Rules", "Rules that apply to every race: timing, damaged and dead cars, field pace, collision, nitrous and downforce.")
            {
                UseMouse = false,
                DisableControls = true,
                Alignment = Alignment.Right
            };

            NativeListItem<string> timeoutItem = new NativeListItem<string>("Timeout (s)", "Grace period after the first racer crosses the line.", SettingsRepair.OptionsFor("TimeoutSeconds"));
            timeoutItem.ItemChanged += (sender, args) => SaveRacerSetting("TimeoutSeconds", timeoutItem.Items[args.Index]);
            timeoutItem.SelectedIndex = Math.Max(0, timeoutItem.Items.IndexOf(SettingsMenuStore.GetInt("TimeoutSeconds", 30).ToString()));
            rulesMenu.Add(timeoutItem);
            HookListTextPicker(timeoutItem);

            NativeListItem<string> autofixItem = new NativeListItem<string>("Racer Autofix", "What the rivals' cars survive: Off takes damage like yours, Auto Repair rebuilds a body or engine below half, Invincible is proofs plus an instant repair.", EnumLabels<AutofixMode>());
            autofixItem.ItemChanged += (sender, args) => { Autofix = (AutofixMode)args.Index; SaveRacerSetting("AIRacerAutofix", Autofix.ToString()); };
            autofixItem.SelectedIndex = Math.Max(0, autofixItem.Items.IndexOf(EnumLabel(Autofix)));
            rulesMenu.Add(autofixItem);
            HookListTextPicker(autofixItem);

            NativeCheckboxItem dnfItem = new NativeCheckboxItem("Hide DNFs", "A dead-engine car is dropped under the track, frozen and invisible, and ignored by the AI. Off parks it on the shoulder.", DNFUnderTrack);
            dnfItem.CheckboxChanged += (sender, args) =>
            {
                DNFUnderTrack = dnfItem.Checked;
                SaveRacerSetting("DNFUnderTrack", DNFUnderTrack.ToString());
            };
            rulesMenu.Add(dnfItem);

            NativeListItem<string> rubberbandItem = new NativeListItem<string>("Rubberbanding (%)", "Packs the field: leaders slowed and laggards boosted by distance from the middle. 0 off, 100 max (0.66x leader, 1.33x last).", SettingsRepair.OptionsFor("Rubberbanding"));
            rubberbandItem.ItemChanged += (sender, args) =>
            {
                RubberbandingPct = int.Parse(rubberbandItem.Items[args.Index], CultureInfo.InvariantCulture);
                SaveRacerSetting("Rubberbanding", rubberbandItem.Items[args.Index]);
            };
            rubberbandItem.SelectedIndex = Math.Max(0, rubberbandItem.Items.IndexOf(SettingsMenuStore.GetInt("Rubberbanding", 100).ToString(CultureInfo.InvariantCulture)));
            rulesMenu.Add(rubberbandItem);
            HookListTextPicker(rubberbandItem);

            NativeListItem<string> rubberModeItem = new NativeListItem<string>("Rubberband Mode", "Natural = leaders are slowed only. Artificial = leaders are slowed and laggards get a torque boost.", Enum.GetNames(typeof(RubberbandMode)));
            rubberModeItem.ItemChanged += (sender, args) =>
            {
                CurrentRubberbandMode = (RubberbandMode)args.Index;
                SaveRacerSetting("RubberbandMode", CurrentRubberbandMode.ToString());
            };
            rubberModeItem.SelectedIndex = Math.Max(0, rubberModeItem.Items.IndexOf(SettingsMenuStore.Get("RubberbandMode", CurrentRubberbandMode.ToString())));
            rulesMenu.Add(rubberModeItem);
            HookListTextPicker(rubberModeItem);

            NativeCheckboxItem noCollisionItem = new NativeCheckboxItem("No Collision", "The racers do not detect each other and pass through each other. World collision is untouched.", NoCollision);
            noCollisionItem.CheckboxChanged += (sender, args) =>
            {
                NoCollision = noCollisionItem.Checked;
                SaveRacerSetting("NoCollision", NoCollision.ToString());
                ApplyNoCollision(NoCollision);
            };
            rulesMenu.Add(noCollisionItem);

            NativeCheckboxItem nitroItem = new NativeCheckboxItem("Nitrous", "A fresh nitrous bottle each lap. Rivals fire on straights; you fire manually.", NitrousEnabled);
            nitroItem.CheckboxChanged += (sender, args) =>
            {
                NitrousEnabled = nitroItem.Checked;
                SaveRacerSetting("NitrousEnabled", NitrousEnabled.ToString());
            };
            rulesMenu.Add(nitroItem);

            NativeListItem<string> aiNitroItem = new NativeListItem<string>("AI Nitrous Power (%)", "The AI's bottle as a percentage of yours: 100 the same, 200 twice. Your own burn is unchanged.", AiNitroValues);

            aiNitroItem.SelectedIndex = Math.Max(0, aiNitroItem.Items.IndexOf(SettingsMenuStore.GetInt("AiNitroPct", AiNitroPct).ToString(CultureInfo.InvariantCulture)));
            AiNitroPct = int.Parse(aiNitroItem.Items[aiNitroItem.SelectedIndex], CultureInfo.InvariantCulture);
            aiNitroItem.ItemChanged += (sender, args) =>
            {
                if (args.Index < 0 || args.Index >= aiNitroItem.Items.Count) return;
                AiNitroPct = int.Parse(aiNitroItem.Items[args.Index], CultureInfo.InvariantCulture);
                SaveRacerSetting("AiNitroPct", AiNitroPct.ToString(CultureInfo.InvariantCulture));
            };
            rulesMenu.Add(aiNitroItem);
            HookListTextPicker(aiNitroItem);

            AddDebugCheckbox(rulesMenu, Options.HighDownforceOnline, "Online Downforce Curve", "For downforce >100, use the full online scaling; off = fall back to the 0.3 singleplayer default.");

            NativeMenu interfaceMenu = new NativeMenu("Interface", "Interface", "What the mod shows you, and the free camera.")
            {
                UseMouse = false,
                DisableControls = true,
                Alignment = Alignment.Right
            };

            NativeListItem<string> tipRateItem = new NativeListItem<string>("Racing Tips", "How often a tip is shown when you pass an apex. The tips themselves are in Settings\\tips.txt.", EnumLabels<TipFrequency>());
            tipRateItem.ItemChanged += (sender, args) =>
            {
                TipRate = (TipFrequency)args.Index;
                SaveRacerSetting("TipRate", TipRate.ToString());
            };
            tipRateItem.SelectedIndex = Math.Max(0, tipRateItem.Items.IndexOf(EnumLabel(TipRate)));
            interfaceMenu.Add(tipRateItem);
            HookListTextPicker(tipRateItem);

            AddDebugCheckbox(interfaceMenu, Options.ShowCheckpoints, "Show Corner Checkpoints", "Draw a marker at every corner apex so you can see where the track goes.");
            AddDebugCheckbox(interfaceMenu, Options.ShowEdgeChevrons, "Show Edge Chevrons", "Draw small blue chevrons along both track edges so you can read the track limits.");
            AddDebugCheckbox(interfaceMenu, Options.ShowLeaderboard, "Show Leaderboard", "Show the race leaderboard on screen, even when you are not on the grid.");
            AddDebugCheckbox(interfaceMenu, Options.ShowDriverNames, "Show Driver Names", "The leaderboard names the driver instead of the car: the rival's name, the player's own for their row. Off shows the car model only.");
            AddDebugCheckbox(interfaceMenu, Options.ShowAiLapTimes, "Show Rival Lap Times", "Announce each AI lap time as it crosses the line. One per racer per lap, so a full grid is a lot of them.");

            NativeCheckboxItem stagedSpawnsItem = new NativeCheckboxItem("Show Track & Grid Rows", "Load Track and Build Grid in the Race menu. Off and not changeable.", false);
            stagedSpawnsItem.Enabled = false;
            stagedSpawnsItem.CheckboxChanged += (sender, args) =>
            {
                StagedSpawns = stagedSpawnsItem.Checked;
                SaveRacerSetting("StagedSpawns", StagedSpawns.ToString());
                if (StagedSpawns)
                {
                    if (!_raceMenu.Items.Contains(instanceTrackItem)) _raceMenu.Add(Math.Min(spawnTrackSlot, _raceMenu.Items.Count), instanceTrackItem);
                    if (!_raceMenu.Items.Contains(instanceGridItem)) _raceMenu.Add(Math.Min(spawnGridSlot, _raceMenu.Items.Count), instanceGridItem);
                }
                else
                {
                    if (_raceMenu.Items.Contains(instanceTrackItem)) _raceMenu.Remove(instanceTrackItem);
                    if (_raceMenu.Items.Contains(instanceGridItem)) _raceMenu.Remove(instanceGridItem);
                }
            };
            interfaceMenu.Add(stagedSpawnsItem);

            NativeItem freecamItem = new NativeItem("Freecam", "Toggle the ARS free camera.");
            freecamItem.Activated += (sender, args) =>
            {
                interfaceMenu.Visible = false;
                _freeCam.Toggle();
            };
            interfaceMenu.Add(freecamItem);

            NativeMenu controlsMenu = new NativeMenu("Controls", "Controls", "The keys that open the ARS menu.")
            {
                UseMouse = false,
                DisableControls = true,
                Alignment = Alignment.Right
            };

            NativeCheckboxItem hotkeysItem = new NativeCheckboxItem("Hotkeys", "Allow the configured virtual-key chord to open the ARS menu. The arsmenu cheat remains available.", HotkeysEnabled);
            hotkeysItem.CheckboxChanged += (sender, args) =>
            {
                HotkeysEnabled = hotkeysItem.Checked;
                SaveRacerSetting("Hotkeys", HotkeysEnabled.ToString());
            };
            controlsMenu.Add(hotkeysItem);

            NativeListItem<string> menuModifierItem = new NativeListItem<string>("Menu Modifier", "First key of the ARS menu chord (Windows virtual-key code).", MenuHotkeyKeys.Select(MenuHotkeyLabel).ToArray());
            menuModifierItem.ItemChanged += (sender, args) =>
            {
                if (args.Index < 0 || args.Index >= MenuHotkeyKeys.Length) return;
                MenuModifierKey = MenuHotkeyKeys[args.Index];
                SettingsMenuStore.Set("MenuModifierKey", (int)MenuModifierKey);
            };
            menuModifierItem.SelectedIndex = Math.Max(0, Array.IndexOf(MenuHotkeyKeys, MenuModifierKey));
            controlsMenu.Add(menuModifierItem);
            HookListTextPicker(menuModifierItem);

            NativeListItem<string> menuKeyItem = new NativeListItem<string>("Menu Key", "Second key of the ARS menu chord (Windows virtual-key code).", MenuHotkeyKeys.Select(MenuHotkeyLabel).ToArray());
            menuKeyItem.ItemChanged += (sender, args) =>
            {
                if (args.Index < 0 || args.Index >= MenuHotkeyKeys.Length) return;
                MenuKey = MenuHotkeyKeys[args.Index];
                SettingsMenuStore.Set("MenuKey", (int)MenuKey);
            };
            menuKeyItem.SelectedIndex = Math.Max(0, Array.IndexOf(MenuHotkeyKeys, MenuKey));
            controlsMenu.Add(menuKeyItem);
            HookListTextPicker(menuKeyItem);

            NativeMenu debugMenu = new NativeMenu("Debug", "Debug", "Developer visuals, lane experiments and logging.")
            {
                UseMouse = false,
                DisableControls = true,
                Alignment = Alignment.Right
            };
            AddDebugCheckbox(debugMenu, Options.ShowInputs, "Show Pedal Trail", "Draw the focus car's input trail: one chevron per metre, coloured green to red by pedal input, holding the last 40 m.");
            AddDebugCheckbox(debugMenu, Options.ShowTrackAnalysis, "Show Track Analysis", "Draw the focus car's steer-angle fan, the corner entrance/apex/exit lines and the lane aim lines 40 nodes ahead.");
            AddDebugCheckbox(debugMenu, Options.ShowRivalry, "Show Rivalry", "Draw the two rival walls from the car's node to one second ahead: red while a rival holds one in, blue once released, plus a line from the car to each far end.");
            AddDebugCheckbox(debugMenu, Options.ShowBumps, "Show Bumps", "Mark every lip the route scan found with a cyan tick and a line across the road.");
            AddDebugCheckbox(debugMenu, Options.ShowWheelSlip, "Show Wheel Slip", "Draw a sphere over each wheel of the focus car, coloured by side-slip: white at rest, green at the peak, red where grip goes flat.");
            AddDebugCheckbox(debugMenu, Options.ShowFrontSlip, "Show Front Slip", "Read the focus car's steer clamp against what the law allows, in the subtitle: a heading, then a factor times the allowance in degrees. x1.0 of 22.5° means the clamp sits on its allowance; x2.1 of 22.5° means it is still twice that and closing.");
            AddDebugCheckbox(debugMenu, Options.LockLaneCentre, "Lock Lane Centre (test)", "Force every racer's lane target to a fixed near-centre offset, overriding the corner, avoidance and wall systems.");
            AddDebugCheckbox(debugMenu, Options.ShowGsProjection, "Show Gs Projection", "Draw the focus car's constant-acceleration path from its roof: a line to 0.5 s and another from there to 1 s; the bend is the acceleration. One colour for both, decided where the path is measured: green fading to white as the Gs lane throttle floor lets go (green at the inside edge, white at the centre), red when the path leaves the road and runs further out than the car (or drifts out with the car still on it), yellow when it leaves the road but cuts inside the car, blue otherwise.");
            AddDebugCheckbox(debugMenu, Options.ShowGLine, "Show G Line", "Draw a line from your own car's roof along the horizontal acceleration it pulls, one metre per G, with a cross at the tail. Your car only.");
            AddDebugCheckbox(debugMenu, Options.ShowRouteRadius, "Show Route Radius", "Draw the three nodes the route radius is measured from: the car's own, the near sample and the far one at speed/grip and twice that ahead.");

            NativeListItem<string> logLevelItem = new NativeListItem<string>("Log Level", "Write this importance and higher to Log.log. Fatal keeps only forced diagnostics.", EnumLabels<LogImportance>());
            logLevelItem.ItemChanged += (sender, args) =>
            {
                LogLevel = ParseEnum(logLevelItem.Items[args.Index], LogLevel);
                SaveRacerSetting("LogLevel", LogLevel.ToString());
            };
            logLevelItem.SelectedIndex = Math.Max(0, logLevelItem.Items.IndexOf(EnumLabel(LogLevel)));
            debugMenu.Add(logLevelItem);
            HookListTextPicker(logLevelItem);

            NativeListItem<string> cornerCeilingItem = new NativeListItem<string>("Corner Ceiling", "The radius treated as a straight: regions are cut where the road opens past it. A change rebuilds the table at once.", CornerCeilingValues);
            cornerCeilingItem.SelectedIndex = Math.Max(0, cornerCeilingItem.Items.IndexOf(SettingsMenuStore.GetInt("CornerCeiling", CornerCeilingMeters).ToString(CultureInfo.InvariantCulture)));
            CornerCeilingMeters = int.Parse(cornerCeilingItem.Items[cornerCeilingItem.SelectedIndex], CultureInfo.InvariantCulture);
            cornerCeilingItem.ItemChanged += (sender, args) =>
            {
                if (args.Index < 0 || args.Index >= cornerCeilingItem.Items.Count) return;
                CornerCeilingMeters = int.Parse(cornerCeilingItem.Items[args.Index], CultureInfo.InvariantCulture);
                SaveRacerSetting("CornerCeiling", CornerCeilingMeters.ToString(CultureInfo.InvariantCulture));
                TrackLoader.BuildApexTable();
                Log(LogImportance.Info, "Corner ceiling " + CornerCeilingMeters + " m: " + Corners.Count + " corner(s) in the table.");
            };
            debugMenu.Add(cornerCeilingItem);
            HookListTextPicker(cornerCeilingItem);

            if (!StagedSpawns)
            {
                _raceMenu.Remove(instanceTrackItem);
                _raceMenu.Remove(instanceGridItem);
            }

            _raceMenu.Add(endItem);

            NativeMenu developmentMenu = new NativeMenu("Development", "Development", "Test-only levers, for probing one car on track.")
            {
                UseMouse = false,
                DisableControls = true,
                Alignment = Alignment.Right
            };

            NativeCheckboxItem steerTraceItem = new NativeCheckboxItem("Steer Trace", "Log the steering ask, the axis, the yaw and the front slip every tick, for your own car only. For measuring the loop delay: leave it off unless you are chasing one, because it writes about thirty lines a second.", SteerTrace);
            steerTraceItem.CheckboxChanged += (sender, args) =>
            {
                SteerTrace = steerTraceItem.Checked;
                DevelopmentMenuStore.Set("SteerTrace", SteerTrace.ToString());
            };
            developmentMenu.Add(steerTraceItem);

            NativeCheckboxItem brakeLearnNotifyItem = new NativeCheckboxItem("Brake Learn Notify", "Announce every brake-learner verdict: corner, before, change and after. Off by default, it prints once per corner per lap.", BrakeLearningNotify);
            brakeLearnNotifyItem.CheckboxChanged += (sender, args) =>
            {
                BrakeLearningNotify = brakeLearnNotifyItem.Checked;
                DevelopmentMenuStore.Set("BrakeLearningNotify", BrakeLearningNotify.ToString());
            };
            developmentMenu.Add(brakeLearnNotifyItem);

            NativeCheckboxItem slipSourceItem = new NativeCheckboxItem("Slip From Memory", "On reads the front tyres' side slip from the wheel memory and reports how much of its peak the inner tyre is using; off derives the demand from the inner wheel's radius against the grip the car has. Two different laws on the same scale, not two sensors for one: switching changes the behaviour, not just the source.", SlipFromMemory);
            slipSourceItem.CheckboxChanged += (sender, args) =>
            {
                SlipFromMemory = slipSourceItem.Checked;
                DevelopmentMenuStore.Set("SlipFromMemory", SlipFromMemory.ToString());
            };
            developmentMenu.Add(slipSourceItem);

            AddSub(_arsMenu, _raceMenu);
            AddSub(_arsMenu, rivalsMenu);
            AddSub(_arsMenu, rulesMenu);
            AddSub(_arsMenu, interfaceMenu);
            AddSub(_arsMenu, controlsMenu);
            AddSub(_arsMenu, debugMenu);
            NativeSubmenuItem developmentEntry = AddSub(_arsMenu, developmentMenu);

            developmentEntry.Enabled = false;
            _arsMenu.Shown += (sender, args) => developmentEntry.Enabled = RaceIsRunning && Racers.Count == 1;
            AddSub(_arsMenu, creatorMenu);

            int trails = ApplyMenuTrails(_arsMenu, _arsMenu.Subtitle);
            Log(LogImportance.Info, "Menu built: " + _menuParents.Count + " submenus, " + trails + " trails");
            _arsMenu.Add(resetItem);

            _menuPool.Add(_arsMenu);
            _menuPool.Add(_raceMenu);
            _menuPool.Add(piSettingsMenu);
            _menuPool.Add(rivalsMenu);
            _menuPool.Add(rivalPaceMenu);
            _menuPool.Add(rivalTractionMenu);
            _menuPool.Add(rivalSteeringMenu);
            _menuPool.Add(rulesMenu);
            _menuPool.Add(interfaceMenu);
            _menuPool.Add(controlsMenu);
            _menuPool.Add(debugMenu);
            _menuPool.Add(developmentMenu);
            _menuPool.Add(creatorMenu);
        }
        void SaveRacerSetting(string key, string value)
        {
            SettingsMenuStore.Set(key, value);
        }
        void AddDebugCheckbox(NativeMenu menu, Options option, string title, string description, Action<bool> onChanged = null, MenuSettings store = null)
        {
            NativeCheckboxItem checkbox = new NativeCheckboxItem(title, description, DebugToggles[option]);
            checkbox.CheckboxChanged += (sender, args) =>
            {
                DebugToggles[option] = checkbox.Checked;
                if (store != null) store.Set(option.ToString(), checkbox.Checked.ToString());
                else SaveDevToggle(option, checkbox.Checked);
                onChanged?.Invoke(checkbox.Checked);
            };
            menu.Add(checkbox);
        }
        void SaveDevToggle(Options option, bool value)
        {
            DebugMenuStore.Set(option.ToString(), value.ToString());
        }

        void HookListTextPicker(NativeListItem<string> item)
        {
            item.Activated += (sender, args) =>
            {
                string typed = Game.GetUserInput(64);
                if (string.IsNullOrEmpty(typed)) return;
                int match = item.Items.FindIndex(s => s.IndexOf(typed, StringComparison.OrdinalIgnoreCase) >= 0);
                if (match < 0)
                {
                    UI.Notify("~r~[ARS]:~w~ No list item contains '" + typed + "'.");
                    return;
                }
                if (match == item.SelectedIndex) return;
                item.SelectedIndex = match;
            };
        }

        public void InstanceTrack()
        {
            CleanRacers();
            _gridInstanced = false;

            string trackPath = _selectedTrackPath;
            if (trackPath == null)
            {
                if (_trackListPaths.Count == 0)
                {
                    UI.Notify("~r~No tracks found.~w~ Put track files in the Tracks folder.");
                    return;
                }
                trackPath = _trackListPaths[0];
            }

            bool loaded = TrackLoader.LoadTrack(this, TrackLoader.LoadTrackFile(trackPath));

            if (!_freeCam.IsActive) Function.Call(Hash.DO_SCREEN_FADE_IN, 500);

            if (!loaded)
            {
                _trackInstanced = false;
                return;
            }

            _trackInstanced = true;
            Log(LogImportance.Info, "Track instanced");
        }

        public void InstanceGrid()
        {
            if (!_trackInstanced)
            {
                UI.Notify("~r~Instance a track first.");
                return;
            }

            if (!_freeCam.IsActive) Function.Call(Hash.DO_SCREEN_FADE_OUT, 200);

            CleanRacers();

            ResolvePaceTargetForGrid();

            Log(LogImportance.Info, "Grid: ranking " + _vehiclePool.Count + " pool files, target " + _resolvedPaceTarget.ToString(CultureInfo.InvariantCulture));

            TrackLoader.BuildGridSlots(RouteNodes, NodeHalfWidths, IsPointToPoint, _intendedOpponents + 1, GridPositions);
            int gridSize = Math.Min(_intendedOpponents, Math.Max(0, GridSlotsAvailable - 1));
            if (gridSize != _intendedOpponents) Log(LogImportance.Info, "Grid: the track only yielded " + GridSlotsAvailable + " slots, so " + _intendedOpponents + " was capped to " + gridSize);
            GridFill fill = FillCachedCandidates(gridSize);
            LoadGrid(gridSize);

            if (Racers.Count == 0 && _intendedOpponents > 0)
            {

                if (fill == GridFill.NoneInBand)
                    UI.Notify("~o~No car within " + PaceDeviationLimit + " PI of " + Math.Round(_resolvedPaceTarget) + ": nothing to build a grid from. Widen Max PI Deviation, or pick a PI target the pool can reach.");
                else if (fill == GridFill.NoPaceIndex)
                    UI.Notify("~o~No pace index for any car in the pool: check the Vehicles folder.");
                else
                    UI.Notify("~o~The grid came back empty.");

                foreach (Prop p in StartLineFlares) if (CanWeUse(p)) p.Delete();
                StartLineFlares.Clear();
                foreach (Prop p in AutoGeneratedProps) if (CanWeUse(p)) p.Delete();
                AutoGeneratedProps.Clear();
                foreach (Prop p in CustomProps) if (CanWeUse(p)) p.Delete();
                CustomProps.Clear();
                _gridInstanced = false;
                _trackInstanced = false;
                if (!_freeCam.IsActive) Function.Call(Hash.DO_SCREEN_FADE_IN, 500);
                return;
            }

            PlaceCars();

            _gridInstanced = true;
            Log(LogImportance.Info, "Grid instanced (" + Racers.Count + " AI cars)");
            if (!_freeCam.IsActive) Function.Call(Hash.DO_SCREEN_FADE_IN, 500);
        }

        void ResolvePaceTargetForGrid()
        {
            _resolvedPaceTarget = PowerTargetScale;
            if (CurrentPaceMode == PaceMode.RelativeToMine)
            {
                if (TryComputePlayerCarPaceIndex(out float playerPace)) { _lastKnownPlayerPace = playerPace; _resolvedPaceTarget = playerPace + PaceOffsetScale; }
                else if (!float.IsNaN(_lastKnownPlayerPace)) { _resolvedPaceTarget = _lastKnownPlayerPace + PaceOffsetScale; UI.Notify("~o~No vehicle to read a performance index from - using your last car's PI."); }
                else UI.Notify("~o~No vehicle to read a performance index from - using the fixed PI Target.");
            }
        }

        bool PreflightStartRace(out string reason)
        {
            reason = null;

            Log(LogImportance.Info, "Preflight: paceMode=" + CurrentPaceMode + " participating=" + PlayerParticipating
                + " inCar=" + (Game.Player.Character.CurrentVehicle != null)
                + " canReadPace=" + TryComputePlayerCarPaceIndex(out float _)
                + " lastKnown=" + _lastKnownPlayerPace);

            string unsupported = VehicleMemory.UnsupportedBuildReason();
            if (unsupported != null) { reason = unsupported; return false; }

            string trackPath = _selectedTrackPath;
            if (trackPath == null)
            {
                if (_trackListPaths.Count == 0) { reason = "No tracks found. Put track files in the Tracks folder."; return false; }
                trackPath = _trackListPaths[0];
            }

            XmlDocument file = TrackLoader.LoadTrackFile(trackPath);
            if (file == null) { reason = "That track file could not be read."; return false; }

            XmlNode data = file.SelectSingleNode("Data");
            int routePoints = data == null ? 0 : data.SelectNodes("Route/Point").Count;
            if (routePoints < 6)
            {
                reason = "That track has " + routePoints + " route nodes; six are needed before a car can be placed on it.";
                return false;
            }

            if (CurrentPaceMode == PaceMode.RelativeToMine && !TryComputePlayerCarPaceIndex(out float _))
            {
                reason = "Pace Mode is Relative, and you are not in a car: get in one, or pick a fixed PI target.";
                return false;
            }

            if (_intendedOpponents > 0)
            {
                ResolvePaceTargetForGrid();
                FillCachedCandidates(_intendedOpponents + 1);
                if (_cachedCandidates.Count == 0)
                {
                    reason = PaceDeviationLimit > 0
                        ? "No car within " + PaceDeviationLimit + " PI of " + Math.Round(_resolvedPaceTarget) + ". Widen Max PI Deviation, or pick a PI target the pool can reach."
                        : "No car in the pool carries a pace index.";
                    return false;
                }
            }
            return true;
        }

        public void StartRaceFromMenu()
        {

            string refusal;
            if (!PreflightStartRace(out refusal))
            {
                UI.Notify("~o~" + refusal);
                Log(LogImportance.Error, "Start Race refused: " + refusal);
                return;
            }

            InstanceTrack();
            if (!_trackInstanced) return;
            InstanceGrid();
            if (!_gridInstanced) return;
            StartRace();
        }

        public void ResetToTrack()
        {
            if (!RaceIsRunning) { UI.Notify("~r~Reset to track:~w~ only while a race is running."); return; }
            Vehicle car = Game.Player.Character.CurrentVehicle;
            if (car == null || !car.Exists()) { UI.Notify("~r~Reset to track:~w~ you must be in a vehicle."); return; }
            if (TrackPoints.Count == 0) { UI.Notify("~r~Reset to track:~w~ no track loaded."); return; }
            TrackPoint nearest = TrackPoints[0];
            float best = float.MaxValue;
            foreach (TrackPoint point in TrackPoints)
            {
                float distance = point.Position.DistanceTo(car.Position);
                if (distance >= best) continue;
                best = distance;
                nearest = point;
            }
            Vector3 direction = new Vector3(nearest.Direction.X, nearest.Direction.Y, 0f);
            if (direction == Vector3.Zero) direction = Vector3.WorldNorth;
            direction.Normalize();
            Vector3 right = Vector3.Cross(direction, Vector3.WorldUp);
            float side = ARS.SignedLaneOffset(car.Position, nearest.Position, nearest.Direction) >= 0f ? 1f : -1f;
            car.Position = nearest.Position + right * (nearest.TrackHalfWidth * side) + new Vector3(0f, 0f, 0.5f);
            car.Heading = direction.ToHeading();
            car.Velocity = direction * MphToMps(10f);
        }

        void SelectTrackInMenu(string path)
        {
            if (_trackListItem == null || _trackListPaths.Count == 0 || string.IsNullOrEmpty(path)) return;
            int idx = _trackListPaths.IndexOf(path);
            if (idx < 0) return;
            _trackListItem.SelectedIndex = idx;
            _selectedTrackPath = path;
        }

        void RefreshTrackList()
        {
            if (_trackListItem == null) return;

            string chosen = _selectedTrackPath;
            string keepVisible = chosen != null ? System.IO.Path.GetFileNameWithoutExtension(chosen) : null;

            List<string> sorted = new List<string>(_trackTags.Keys);
            sorted.Sort(StringComparer.OrdinalIgnoreCase);

            _trackListPaths.Clear();
            for (int i = 0; i < sorted.Count; i++) _trackListPaths.Add(sorted[i]);

            List<string> names = new List<string>();
            foreach (string path in sorted)
                names.Add(System.IO.Path.GetFileNameWithoutExtension(path));

            if (_trackListItem.Items.Count > 0) _trackListItem.SelectedIndex = 0;
            _trackListItem.Items = names;

            int idx = -1;
            if (keepVisible != null) idx = _trackListItem.Items.IndexOf(keepVisible);
            if (idx < 0 && keepVisible == null)
            {
                for (int i = 0; i < _trackListItem.Items.Count; i++)
                {
                    if (string.Equals(_trackListItem.Items[i], "Figureight", StringComparison.OrdinalIgnoreCase))
                    {
                        idx = i;
                        break;
                    }
                }
            }
            if (_trackListItem.Items.Count > 0) _trackListItem.SelectedIndex = idx >= 0 ? idx : 0;

            if (_trackListPaths.Count > 0)
                _selectedTrackPath = _trackListPaths[_trackListItem.SelectedIndex];
            else
                _selectedTrackPath = null;
        }

        void RefreshPowerControls()
        {
            if (_powerTargetItem == null) return;

            float min = float.MaxValue;
            float max = float.MinValue;
            foreach (float pace in ModelPaceIndexCache.Values)
            {
                if (float.IsNaN(pace) || float.IsInfinity(pace)) continue;
                if (pace < min) min = pace;
                if (pace > max) max = pace;
            }
            if (min == float.MaxValue) { min = 0f; max = 1f; }

            bool targetStored = RaceMenuStore.TryGetFloat("PaceTarget", out float storedTarget) && !float.IsNaN(storedTarget);
            if (targetStored) PowerTargetScale = RoundToNearestEven(Clamp(storedTarget, min, max));
            PowerTargetScale = RoundToNearestEven(Clamp(PowerTargetScale, min, max));

            _powerTargetValues.Clear();
            AddPowerValues(_powerTargetValues, min, max, 2.0f);

            if (_powerTargetValues.Count == 0) _powerTargetValues.Add(RoundToNearestEven(Clamp(PowerTargetScale, min, max)));
            Log(LogImportance.Info, "Pace span " + min.ToString("0.0") + ".." + max.ToString("0.0") + " over " + ModelPaceIndexCache.Count + " models, " + _powerTargetValues.Count + " PI Target value(s).", true);

            _powerTargetItem.Items.Clear();
            foreach (float value in _powerTargetValues) _powerTargetItem.Items.Add(value.ToString("0"));
            _powerTargetItem.SelectedIndex = !targetStored ? _powerTargetValues.Count / 2 : FindNearestPowerValue(_powerTargetValues, PowerTargetScale);
            if (_powerTargetValues.Count > 0) PowerTargetScale = _powerTargetValues[_powerTargetItem.SelectedIndex];
            _resolvedPaceTarget = PowerTargetScale;

            if (_paceOffsetValues.Count > 0) _paceOffsetItem.SelectedIndex = FindNearestPowerValue(_paceOffsetValues, PaceOffsetScale);
            _paceModeItem.SelectedIndex = Math.Max(0, _paceModeItem.Items.IndexOf(EnumLabel(CurrentPaceMode)));
            RaceMenuStore.Migrate("PaceTarget", PowerTargetScale.ToString(CultureInfo.InvariantCulture));
            RaceMenuStore.Migrate("PaceOffset", PaceOffsetScale.ToString(CultureInfo.InvariantCulture));
            ApplyPaceModeUI();
        }

        void ApplyPaceModeUI()
        {

            if (_powerTargetItem == null || _paceOffsetItem == null) return;
            float playerPace;
            bool hasPlayerPace = TryComputePlayerCarPaceIndex(out playerPace);
            _powerTargetItem.Enabled = CurrentPaceMode == PaceMode.Absolute;
            _powerTargetItem.Description = WithBallpark(CurrentPaceMode == PaceMode.RelativeToMine ? "Disabled in Relative To Mine - resolves at Spawn Grid: your car's PI + offset." : "Fixed performance index for the grid (spectating).", BallparkText(PowerTargetScale));
            _paceOffsetItem.Enabled = CurrentPaceMode == PaceMode.RelativeToMine;
            _paceOffsetItem.Description = WithBallpark(CurrentPaceMode == PaceMode.RelativeToMine ? "Field PI = your car's performance index + this offset, resolved at Spawn Grid." : "Disabled in Absolute mode - the fixed PI Target governs.", hasPlayerPace ? BallparkText(playerPace + PaceOffsetScale) : null);
        }

        static string WithBallpark(string description, string ballpark)
        {
            return ballpark == null ? description : description + " " + ballpark;
        }

        string BallparkText(float pi)
        {
            if (ModelPaceIndexCache.Count == 0) return null;
            List<KeyValuePair<string, float>> ranked = ModelPaceIndexCache.OrderBy(entry => entry.Value).ThenBy(entry => entry.Key, StringComparer.OrdinalIgnoreCase).ToList();
            KeyValuePair<string, float>? below = null;
            KeyValuePair<string, float>? above = null;
            foreach (KeyValuePair<string, float> entry in ranked)
            {
                if (entry.Value <= pi) below = entry;
                else { above = entry; break; }
            }
            KeyValuePair<string, float>? nearest = below;
            if (above != null && (nearest == null || above.Value.Value - pi < pi - nearest.Value.Value)) nearest = above;
            List<string> names = new List<string>();
            AddBallparkName(names, below);
            AddBallparkName(names, nearest);
            AddBallparkName(names, above);
            return names.Count == 0 ? null : "Ballpark: " + string.Join(", ", names.ToArray());
        }

        void AddBallparkName(List<string> names, KeyValuePair<string, float>? entry)
        {
            if (entry == null) return;
            string name;
            if (!ModelNameCache.TryGetValue(entry.Value.Key, out name) || string.IsNullOrWhiteSpace(name)) name = entry.Value.Key;
            if (!names.Contains(name)) names.Add(name);
        }

        static void AddPowerValues(List<float> values, float min, float max, float step)
        {

            int startIndex = (int)Math.Ceiling(min / step);
            int endIndex = (int)Math.Floor(max / step);
            for (int i = startIndex; i <= endIndex; i++)
                values.Add((float)Math.Round(i * step, 0));
        }

        static float RoundToNearestEven(float value) => (float)(2 * Math.Round(value / 2f, MidpointRounding.AwayFromZero));

        static int FindNearestPowerValue(List<float> values, float target)
        {
            int nearest = 0;
            float distance = float.MaxValue;
            for (int i = 0; i < values.Count; i++)
            {
                float candidateDistance = Math.Abs(values[i] - target);
                if (candidateDistance < distance) { distance = candidateDistance; nearest = i; }
            }
            return nearest;
        }
        public static float MpsToMph(float ms)
        {
            return (float)Math.Round(ms * 2.236936f, 3);
        }
        public static float MphToMps(float mph)
        {
            return (float)Math.Round(mph * 0.44704f, 3);
        }

        void ApplyNoCollision(bool enable)
        {
            for (int i = 0; i < Racers.Count; i++)
            {
                Racer a = Racers[i];
                if (!a.Car.Exists()) continue;

                Racer nearest = null;
                float nearestDistance = float.MaxValue;
                for (int j = 0; j < Racers.Count; j++)
                {
                    if (j == i) continue;
                    Racer b = Racers[j];
                    if (!b.Car.Exists()) continue;
                    float distance = b.Car.Position.DistanceTo(a.Car.Position);
                    if (distance < nearestDistance)
                    {
                        nearestDistance = distance;
                        nearest = b;
                    }
                }

                if (nearest != null) Function.Call(Hash.SET_ENTITY_NO_COLLISION_ENTITY, a.Car, nearest.Car, !enable);
            }
        }

        void CleanEverything()
        {
            CleanRacers();
            foreach (Prop p in StartLineFlares) if (CanWeUse(p)) p.Delete();
            foreach (Prop p in CustomProps) if (CanWeUse(p)) p.Delete();
            foreach (Prop p in AutoGeneratedProps) if (CanWeUse(p)) p.Delete();

            foreach (int fx in _flareFx) Function.Call(Hash.STOP_PARTICLE_FX_LOOPED, fx);
            _flareFx.Clear();
            ApplyNoCollision(false);
            LeaderboardFinish.Clear();
            ParkedDNFs = 0;
            _routeEditorActive = false;
            _raceTimedFinishMs = 0;

            Bumps.Clear();
            RouteNodes.Clear();
            NodeHalfWidths.Clear();
            EditNodeHalfWidths.Clear();

            RaceStatus = RaceState.None;
            PlayerParticipating = false;
            _countdown = _maxCountdown;
            _trackInstanced = false;
            _gridInstanced = false;
        }

        int _nextInLine = 0;
        int _gameTimeNextInLine = 0;

        void OnKeyDown(object sender, KeyEventArgs e)
        {
            if (!HotkeysEnabled || _arsMenu == null || _menuPool.AreAnyVisible) return;
            if (e.KeyCode != MenuKey || !IsMenuModifierPressed(e)) return;
            _arsMenu.Visible = true;
        }

        static bool IsMenuModifierPressed(KeyEventArgs e)
        {
            if (MenuModifierKey == Keys.ShiftKey || MenuModifierKey == Keys.LShiftKey || MenuModifierKey == Keys.RShiftKey) return e.Shift;
            if (MenuModifierKey == Keys.ControlKey || MenuModifierKey == Keys.LControlKey || MenuModifierKey == Keys.RControlKey) return e.Control;
            if (MenuModifierKey == Keys.Menu || MenuModifierKey == Keys.LMenu || MenuModifierKey == Keys.RMenu) return e.Alt;
            return Game.IsKeyPressed(MenuModifierKey);
        }

        bool IsMenuHotkeyPressed()
        {
            return HotkeysEnabled && Game.IsKeyPressed(MenuModifierKey) && Game.IsKeyPressed(MenuKey);
        }

        void OnTick(object sender, EventArgs e)
        {
            try
            {
                UpdateChecker.TryNotify();

                HandleLoadScriptTask();

                Player player = Game.Player;
                if (player == null || player.Character == null || !player.Character.Exists()) return;

                Vehicle v = player.LastVehicle;

                if (!_loaded)
                {
                    StartLoadScript();
                    return;
                }

                SmartTuner.Tick(GetRandomInt);

                TrackLoader.TickQueuedFlares(FlareEffects);

                _menuPool.Process();
                HandleTrackCreator();

                if (!CanWeUse(FreeCamRide))
                {
                    FreeCamRide = World.CreateProp("ba_prop_battle_drone_quad_static", Game.Player.Character.Position + new Vector3(0, 0, 10), true, false);

                    if (CanWeUse(FreeCamRide)) FreeCamRide.HasGravity = false;
                    Log(LogImportance.Info, "Creating needed freecam object.");
                }

                _freeCam.Update(_routeEditorActive, RouteNodes.Count);

                if (_routeEditorActive) TrackVisuals.DrawRoute(RouteNodes, NodeHalfWidths, _routeEditorActive);
                if (PlayerRacer == null || !Racers.Contains(PlayerRacer)) PlayerRacer = Racers.FirstOrDefault(r => r.Driver != null && r.Driver.IsPlayer);
                bool raceLive = RaceIsRunning;

                if (DebugToggles[Options.ShowCheckpoints] && ARS.Corners.Count > 0)
                {
                    if (PlayerRacer != null)
                        TrackVisuals.DrawCornerCheckpoints(PlayerRacer, ARS.Corners, ARS.TrackPoints);
                    else if (raceLive)
                        TrackVisuals.DrawCornerCheckpoints(Game.Player.Character.Position, SpectateCheckpointRadiusMeters, ARS.Corners, ARS.TrackPoints);
                }
                if (DebugToggles[Options.ShowEdgeChevrons])
                {
                    if (PlayerRacer != null)
                        TrackVisuals.DrawEdgeChevrons(PlayerRacer, ARS.TrackPoints);
                    else if (raceLive)
                        TrackVisuals.DrawEdgeChevrons(Game.Player.Character.Position, ARS.TrackPoints);
                }
                if (DebugToggles[Options.ShowBumps] && ARS.Bumps.Count > 0 && Game.Player.Character != null)
                {
                    Vector3 focus = Game.Player.Character.Position;
                    foreach (Bump bump in ARS.Bumps)
                    {
                        Vector3 lip = ARS.TrackPoints[bump.LipNode].Position;
                        if ((lip - focus).Length() > SpectateCheckpointRadiusMeters) continue;

                        Vector3 dir = ARS.TrackPoints[bump.LipNode].Direction;
                        Vector3 right = Vector3.Cross(dir, Vector3.WorldUp).Normalized;
                        DrawLine(lip, lip + Vector3.WorldUp * 3f, Color.Cyan);
                        DrawLine(lip - right * 4f, lip + right * 4f, Color.Cyan);
                    }
                }
                if (DebugToggles[Options.ShowGLine] && player.Character.IsInVehicle())
                {
                    Vehicle gLineCar = player.Character.CurrentVehicle;
                    if (CanWeUse(gLineCar)) UpdatePlayerGLine(gLineCar);
                }

                if (DebugToggles[Options.ShowRivalry])
                {
                    foreach (Racer cardRacer in Racers)
                    {
                        if (cardRacer == null || cardRacer.ControlledByPlayer || !cardRacer.Car.Exists()) continue;
                        if (cardRacer.ActiveManeuver.Type == ManeuverType.None) continue;
                        if (cardRacer.Car.Position.DistanceTo(player.Character.Position) > 100f) continue;
                        DrawText(cardRacer.Car.Position + Vector3.WorldUp * 2.2f, CardLabel(cardRacer), Color.White, 0.5f);
                    }
                }

                if (DebugToggles[Options.ShowRouteRadius])
                {
                    Racer radiusRacer = DebugFocusRacer ?? PlayerRacer ?? (Racers.Count > 0 ? Racers[0] : null);
                    if (radiusRacer != null)
                    {
                        TrackVisuals.DrawRouteRadiusChecks(radiusRacer, ARS.TrackPoints);
                        if (radiusRacer.LiveRouteRadius > 0f)
                            DrawText(radiusRacer.Car.Position + Vector3.WorldUp * 2.2f,
                                     Math.Round(radiusRacer.LiveRouteRadius).ToString(CultureInfo.InvariantCulture) + " m",
                                     Color.Magenta, 0.55f);
                    }
                }
                if (DebugToggles[Options.ShowTrackAnalysis] && ARS.Corners.Count > 0)
                {
                    if (DrawTrackAnalysisExtras)
                    {
                        if (PlayerRacer != null)
                            TrackVisuals.DrawCornerRegions(PlayerRacer, ARS.Corners, ARS.TrackPoints);
                        else if (raceLive)
                            TrackVisuals.DrawCornerRegions(Game.Player.Character.Position, SpectateCheckpointRadiusMeters, ARS.Corners, ARS.TrackPoints);

                        if (PlayerRacer != null)
                            TrackVisuals.DrawCrests(PlayerRacer, ARS.Crests, ARS.TrackPoints);
                        else if (raceLive)
                            TrackVisuals.DrawCrests(Game.Player.Character.Position, SpectateCheckpointRadiusMeters, ARS.Crests, ARS.TrackPoints);
                    }

                    if (DebugFocusRacer != null)
                    {
                        Racer racer = DebugFocusRacer;
                        int currentNode = racer.CurrentTrackPoint.Node;
                        int count = ARS.TrackPoints.Count;
                        float nodeOffset = racer.Brain.CurrentPerception.CurrentNodeOffset;
                        float carriedOffset = racer.CarriedLane;
                        float requestOffset = racer.LaneRequest;
                        Color laneColor = Racer.LaneMotionColor(racer.LaneRequestReason, racer.LaneRate, racer.LaneCornerDir);
                        // A lead past the leash's near end blinks white: 200 ms on, 200 off, so the stretch is unmistakable.
                        if (Math.Abs(carriedOffset - nodeOffset) > Racer.LaneTaperNearMeters && Game.GameTime % LaneBlinkPeriodMs < LaneBlinkOnMs) laneColor = Color.White;
                        Color requestColor = Color.FromArgb(120, laneColor.R, laneColor.G, laneColor.B);
                        // The car's own lane wears the pedal trail's colours: green on throttle, yellow neutral, red on the brake.
                        Color ownLaneColor = Racer.InputColour(racer.Control.Throttle - racer.Control.Brake);
                        // Visual only: the lines follow the road's own profile and keep the car's height above it, so a crest or a
                        // bridge bends them with the surface instead of leaving them flat.
                        float carLift = racer.Car.Position.Z - racer.CurrentTrackPoint.Position.Z;
                        // The ribbon ends on the same node the aim uses, one short of it. The lookahead is a node offset, so
                        // counting nodes here is what keeps both ends together; walking metres would drift with node spacing.
                        int laneSpan = (int)Racer.SteerLookaheadNodes(racer.Car.Velocity.Length());
                        Vector3 laneEnd = Vector3.Zero;

                        for (int i = 0; i < laneSpan - 1; i++)
                        {
                            int node = ARS.IsPointToPoint ? currentNode + i : (currentNode + i) % count;
                            int nextNode = ARS.IsPointToPoint ? currentNode + i + 1 : (currentNode + i + 1) % count;
                            if (node >= count || nextNode >= count) continue;

                            Vector3 pos = ARS.TrackPoints[node].Position;
                            Vector3 nextPos = ARS.TrackPoints[nextNode].Position;
                            Vector3 dir = ARS.TrackPoints[node].Direction;
                            Vector3 right = Vector3.Cross(dir, Vector3.WorldUp).Normalized;

                            Vector3 ownFrom = pos + right * nodeOffset; ownFrom.Z = pos.Z + carLift + LaneHeightCurrent;
                            Vector3 ownTo = nextPos + right * nodeOffset; ownTo.Z = nextPos.Z + carLift + LaneHeightCurrent;
                            DrawLine(ownFrom, ownTo, ownLaneColor);

                            float t0 = i / (float)laneSpan;
                            float t1 = (i + 1) / (float)laneSpan;
                            Vector3 intendedFrom = pos + right * (nodeOffset + (carriedOffset - nodeOffset) * t0);
                            intendedFrom.Z = pos.Z + carLift + LaneHeightCarried;
                            Vector3 intendedTo = nextPos + right * (nodeOffset + (carriedOffset - nodeOffset) * t1);
                            intendedTo.Z = nextPos.Z + carLift + LaneHeightCarried;
                            DrawLine(intendedFrom, intendedTo, laneColor);
                            laneEnd = intendedTo;

                            if (DrawLaneRequest && i % 2 == 0)
                            {
                                Vector3 requestFrom = pos + right * requestOffset; requestFrom.Z = pos.Z + carLift + LaneHeightRequest;
                                Vector3 requestTo = nextPos + right * requestOffset; requestTo.Z = nextPos.Z + carLift + LaneHeightRequest;
                                DrawLine(requestFrom, requestTo, requestColor);
                            }

                        }

                        // A black ball marks the far end when the lookahead is at its cap, so a capped line reads as capped.
                        if (laneSpan >= (int)Racer.SteerLookaheadMaxNodes)
                            World.DrawMarker(MarkerType.DebugSphere, laneEnd, Vector3.Zero, Vector3.Zero, new Vector3(0.35f, 0.35f, 0.35f), Color.Black);

                        int adoptedNode = racer.LineCornerNode;
                        if (adoptedNode >= 0 && adoptedNode < ARS.TrackPoints.Count)
                        {
                            Vector3 apexPos = ARS.TrackPoints[adoptedNode].Position + Vector3.WorldUp * ApexMarkHeight;
                            int cornerIndex = ARS.Corners.FindIndex(c => c.Node == adoptedNode);
                            int apexNumber = cornerIndex >= 0 ? cornerIndex + 1 : 0;
                            float apexRadius = cornerIndex >= 0 ? ARS.Corners[cornerIndex].DetectedRadius : ARS.TrackPoints[adoptedNode].PreciseCurveRadius;
                            DrawText(apexPos + Vector3.WorldUp * 3f, (apexNumber > 0 ? "Apex #" + apexNumber : "Apex") + "~n~" + Math.Round(apexRadius) + "r", Color.White, 0.5f);
                        }

                    }

                    Racer liveCrestRacer = PlayerRacer ?? DebugFocusRacer;
                    if (DrawTrackAnalysisExtras && liveCrestRacer != null)
                        TrackVisuals.DrawLiveCrestChecks(liveCrestRacer, ARS.TrackPoints);
                }

                if (DebugToggles[Options.ShowRivalry] && DebugFocusRacer != null && DebugFocusRacer.WallRoadWide > 0.01f)
                {
                    Racer racer = DebugFocusRacer;
                    int wallCount = ARS.TrackPoints.Count;
                    int wallNode = racer.CurrentTrackPoint.Node % wallCount;
                    Color leftColor = racer.LeftWallHeld ? Color.Red : Color.DodgerBlue;
                    Color rightColor = racer.RightWallHeld ? Color.Red : Color.DodgerBlue;

                    float lookAheadMeters = racer.Car.Velocity.Length();
                    int endNode = wallNode;
                    float walked = 0f;
                    int steps = 0;
                    while (walked < lookAheadMeters && steps < 400)
                    {
                        int next = ARS.IsPointToPoint ? endNode + 1 : (endNode + 1) % wallCount;
                        if (next >= wallCount) break;
                        float step = ARS.TrackPoints[next].CumulativeDistance - ARS.TrackPoints[endNode].CumulativeDistance;
                        if (step <= 0f) break;
                        walked += step;
                        endNode = next;
                        steps++;
                    }

                    Vector3 startPos = ARS.TrackPoints[wallNode].Position;
                    Vector3 startRight = Vector3.Cross(ARS.TrackPoints[wallNode].Direction, Vector3.WorldUp).Normalized;
                    Vector3 endPos = ARS.TrackPoints[endNode].Position;
                    Vector3 endRight = Vector3.Cross(ARS.TrackPoints[endNode].Direction, Vector3.WorldUp).Normalized;
                    Vector3 lift = new Vector3(0, 0, LaneHeightWalls);

                    DrawLine(startPos + startRight * racer.AvoidLeftWall + lift, endPos + endRight * racer.AvoidLeftWall + lift, leftColor);
                    DrawLine(startPos + startRight * racer.AvoidRightWall + lift, endPos + endRight * racer.AvoidRightWall + lift, rightColor);

                    Vector3 carPos = racer.Car.Position + lift;
                    DrawLine(carPos, endPos + endRight * racer.AvoidLeftWall + lift, leftColor);
                    DrawLine(carPos, endPos + endRight * racer.AvoidRightWall + lift, rightColor);
                }

                if (NitrousEnabled && PlayerRacer != null
                    && (Game.IsKeyPressed(Keys.X) || Game.IsControlJustPressed(2, GTA.Control.VehicleDuck)))
                    PlayerRacer.TryFireNitrous();
                if (_raceTimedFinishMs != 0 && _raceTimedFinishMs > Game.GameTime) DisplayHelpText("~y~" + (_raceTimedFinishMs - Game.GameTime) / 1000 + "s~w~ to end the race.");
                if (RaceIsRunning) DrawRaceHud();

                if (RaceStatus == RaceState.None || RaceStatus == RaceState.NotInitiated)
                {
                    TrackStartInfo nearest = FindNearestTrackStartInfo(Game.Player.Character.Position);
                    if (nearest != null)
                    {
                        World.DrawMarker(MarkerType.ChevronUpx1, nearest.JoinPosition + new Vector3(0, 0, 2f), Vector3.Zero, new Vector3(180f, 0f, 0f), new Vector3(1f, 1f, 1f), Color.Yellow, false, false, 0, false, "", "", false);
                        float dist = nearest.JoinPosition.DistanceTo2D(Game.Player.Character.Position);
                        if (dist <= 5f)
                        {
                            DisplayHelpTextThisFrame("Press ~INPUT_CONTEXT~ to race at ~b~" + Path.GetFileNameWithoutExtension(nearest.TrackPath) + "~w~.");
                            if ((_arsMenu == null || !_arsMenu.Visible) && CanWeUse(Game.Player.Character.CurrentVehicle) && !IsMenuHotkeyPressed() && Game.IsControlJustPressed(2, GTA.Control.Context))
                            {

                                SelectTrackInMenu(nearest.TrackPath);
                                StartRaceFromMenu();
                            }
                        }
                    }
                }

                if (CountdownScaleform.IsLoaded && _countdown != _maxCountdown) CountdownScaleform.Render2D();

                if (_routeEditorActive || RouteNodes.Count > 0)
                {
                    Function.Call(Hash.SET_VEHICLE_DENSITY_MULTIPLIER_THIS_FRAME, 0f);
                    Function.Call(Hash.SET_RANDOM_VEHICLE_DENSITY_MULTIPLIER_THIS_FRAME, 0f);
                    Function.Call(Hash.SET_PARKED_VEHICLE_DENSITY_MULTIPLIER_THIS_FRAME, 0f);
                    if (Racers.Count() > 0) Function.Call(Hash._0x90B6DA738A9A25DA, 0f);
                    Function.Call(Hash.SET_PED_DENSITY_MULTIPLIER_THIS_FRAME, 0f);
                    Function.Call(Hash.SET_SCENARIO_PED_DENSITY_MULTIPLIER_THIS_FRAME, 0f);
                }

                if (!_menuPool.AreAnyVisible && WasCheatStringJustEntered("arsmenu"))
                {
                    _arsMenu.Visible = true;
                }

                if (RaceStatus != RaceState.Countdown && _countdownTickMs - Game.GameTime > 5000) _countdown = _maxCountdown - 1;
                if (RaceStatus == RaceState.NotInitiated)
                {
                    if (CanWeUse(Game.Player.Character.CurrentVehicle))
                    {
                        if ((Game.IsControlJustPressed(2, GTA.Control.VehicleAccelerate))) StartCountdown();
                    }
                    else StartCountdown();
                }

                if (_countdownTickMs < Game.GameTime)
                {
                    _countdownTickMs = Game.GameTime + 1000;

                    Vehicle jack = Game.Player.Character.GetVehicleIsTryingToEnter();
                    if (_countdown > 0 && Racers.Count > 0 && CanWeUse(jack) && jack != Game.Player.Character.LastVehicle && !jack.IsSeatFree(VehicleSeat.Driver) && jack.Handle == Racers.OrderBy(c => c.Car.Position.DistanceTo(Game.Player.Character.Position)).ToList()[0].Car.Handle)
                    {
                        Game.Player.Character.SetIntoVehicle(jack, VehicleSeat.Passenger);
                    }

                    if (RaceStatus == RaceState.Countdown)
                    {
                        _countdown--;

                        foreach (Racer r in Racers) r.BaseBehavior = RacerBaseBehavior.GridWait;

                        if (_countdown <= 3)
                        {

                            Game.PlaySound("3_2_1", "HUD_MINI_GAME_SOUNDSET");

                            if (_countdown == 0) CountdownScaleform.CallFunction("SHOW_SHARD_CENTERED_TOP_MP_MESSAGE", "GO", "", (int)12, (int)2);
                            else CountdownScaleform.CallFunction("SHOW_SHARD_CENTERED_TOP_MP_MESSAGE", _countdown, "", (int)12, (int)2);

                            if (_countdown == 0)
                            {
                                CountdownScaleform.CallFunction("TRANSITION_OUT", 0.6f);
                            }
                        }

                        if (_countdown == 0)
                        {
                            foreach (Racer r in Racers)
                            {
                                r.Launch();
                            }
                            _countdown = _maxCountdown;
                            RaceStatus = RaceState.InProgress;
                            RaceStartTime = Game.GameTime;
                            if (NoCollision) ApplyNoCollision(true);
                        }
                    }
                }

                if (Racers.Count > 0)
                {
                    if (_gameTimeNextInLine <= Game.GameTime)
                    {

                        int count = (int)Clamp(Racers.Count, 1, 6);
                        for (int i = 0; i < count; i++)
                        {

                            if (_nextInLine >= Racers.Count) _nextInLine = 0;
                            Racers[_nextInLine]?.RunTimedCore();
                            _nextInLine++;
                            _gameTimeNextInLine = Game.GameTime + Math.Max(1, 10 / (Racers.Count / count));
                        }
                    }

                    if (NoCollision) ApplyNoCollision(true);
                }

                if (DebugToggles[Options.ShowInputs] || DebugToggles[Options.ShowTrackAnalysis] || DebugToggles[Options.ShowWheelSlip] || DebugToggles[Options.ShowFrontSlip] || DebugToggles[Options.ShowGsProjection] || DebugToggles[Options.ShowRivalry])
                {
                    Vector3 focusPos = Game.Player.Character.Position;
                    DebugFocusRacer = Racers.Where(r => r.Driver != null && !r.Driver.IsPlayer && CanWeUse(r.Car)).OrderBy(r => r.Car.Position.DistanceTo(focusPos)).FirstOrDefault();
                }
                else DebugFocusRacer = null;
                int raceLaps = RaceMenuStore.GetInt("Laps", 6);
                foreach (Racer racer in Racers)
                {
                    racer.ProcessTick();

                    if (((!IsPointToPoint && racer.Lap > raceLaps) || (IsPointToPoint && racer.Lap > 1)) && !LeaderboardFinish.Contains(racer))
                    {
                        if (racer.Car.CurrentBlip != null) racer.Car.CurrentBlip.Color = BlipColor.Green;

                        racer.FinalPosition = LeaderboardFinish.Count + 1;
                        racer.RacePosition = racer.FinalPosition;
                        LeaderboardFinish.Add(racer);
                        racer.BaseBehavior = RacerBaseBehavior.FinishedRace;
                        if (IsPointToPoint) racer.BaseBehavior = RacerBaseBehavior.FinishedStandStill;
                        if (_raceTimedFinishMs == 0) _raceTimedFinishMs = Game.GameTime + (SettingsMenuStore.GetInt("TimeoutSeconds", 30) * 1000);
                    }
                }

                if (_posUpdateTickMs < Game.GameTime)
                {
                    _posUpdateTickMs = Game.GameTime + 200;

                    var unfinished = Racers.Where(r => r.FinalPosition == 0).ToList();
                    if (unfinished.Any())
                    {
                        var sorted = unfinished
                            .OrderByDescending(r => r.RaceProgress)
                            .ToList();
                        for (int i = 0; i < sorted.Count; i++)
                            sorted[i].RacePosition = i + 1;
                    }
                }

                bool raceCompleted = LeaderboardFinish.Count > 0 || (_raceTimedFinishMs != 0 && Game.GameTime > _raceTimedFinishMs);
                if (raceCompleted && (LeaderboardFinish.Count == Racers.Count(r => !r.IsDNF) || (_raceTimedFinishMs != 0 && Game.GameTime > _raceTimedFinishMs)))
                {
                    foreach (Racer r in Racers.Where(r => r.FinalPosition == 0).OrderByDescending(r => r.RaceProgress))
                        r.FinalPosition = LeaderboardFinish.Count + 1;
                    if (LeaderboardFinish.Count > 0 && LeaderboardFinish[0].Driver.IsPlayer) Game.Player.Money += RaceReward;
                    RaceStatus = RaceState.Finished;
                    CleanEverything();
                }

                if (_longTickMs < Game.GameTime)
                {
                    _longTickMs = Game.GameTime + 3000;

                    if (HelpMessages.Count > 0 && !HideHudMode)
                    {
                        if (!Function.Call<bool>(Hash.IS_HELP_MESSAGE_BEING_DISPLAYED))
                        {
                            DisplayHelpTextTimed(HelpMessages[0], (int)(HelpMessages[0].Length * 120));
                            HelpMessages.RemoveAt(0);
                        }
                    }
                }
                HandleCheats();
            }
            catch (Exception ex)
            {
                Log(LogImportance.Error, "OnTick error: " + ex, true);
            }
        }
        public static TimeSpan ParseToTimeSpan(int gameTime)
        {
            TimeSpan t = new TimeSpan();
            t = TimeSpan.FromMilliseconds(gameTime);
            return t;
        }

        static string Ordinal(int n)
        {
            int mod100 = n % 100;
            if (mod100 >= 11 && mod100 <= 13) return n + "th";
            switch (n % 10)
            {
                case 1: return n + "st";
                case 2: return n + "nd";
                case 3: return n + "rd";
                default: return n + "th";
            }
        }

        static void DrawRaceHud()
        {
            Racer player = PlayerRacer;

            if (DebugToggles[Options.ShowLeaderboard] && !HideHudMode) DrawLeaderboard();

            if (player == null) return;

            int raceElapsed = RaceStatus == RaceState.InProgress ? Game.GameTime - RaceStartTime : 0;
            int lapElapsed = player.LapStartTime > 0 ? Game.GameTime - player.LapStartTime : raceElapsed;
            if (lapElapsed < 0) lapElapsed = 0;

            string position = Ordinal(player.RacePosition).ToUpper();
            string lapTime = TimeSpan.FromMilliseconds(lapElapsed).ToString(@"mm\:ss\.ff");
            string totalTime = TimeSpan.FromMilliseconds(raceElapsed).ToString(@"mm\:ss\.ff");

            Color hudColor = Color.White;
            float leftX = 0.85f;
            DrawText(new Vector2(leftX, 0.72f), position, hudColor, DrawTextFont.Pricedown, DrawTextAlign.Left, 2.0f);
            DrawText(new Vector2(leftX, 0.83f), "CURRENT LAP    " + lapTime, hudColor, DrawTextFont.Condensed, DrawTextAlign.Left, 0.45f);
            DrawText(new Vector2(leftX, 0.87f), "TIME    " + totalTime, hudColor, DrawTextFont.Condensed, DrawTextAlign.Left, 0.45f);
        }

        const bool ShowPeakStats = false;

        static string BoardRowName(Racer r)
        {
            if (DebugToggles[Options.ShowDriverNames] && !string.IsNullOrEmpty(r.DriverName)) return r.DriverName;
            return r.CarModelName;
        }

        static void DrawLeaderboard()
        {

            const float headerY = 0.18f;
            const float step = 0.031f;
            const float colPos = 0.02f;
            const float colPI = 0.05f;
            const float colCar = 0.10f;
            const float colBest = 0.22f;
            const float colAcc = 0.32f;
            const float colDec = 0.39f;
            const float colLat = 0.46f;
            const float colTop = 0.53f;
            const float scale = 0.336f;
            const DrawTextFont font = DrawTextFont.Standard;
            Color hCol = Color.FromArgb(180, 180, 180);

            DrawText(new Vector2(colPos, headerY), "POS", hCol, font, DrawTextAlign.Left, scale);
            DrawText(new Vector2(colPI, headerY), "PI", hCol, font, DrawTextAlign.Left, scale);
            DrawText(new Vector2(colCar, headerY), DebugToggles[Options.ShowDriverNames] ? "DRIVER" : "CAR", hCol, font, DrawTextAlign.Left, scale);
            DrawText(new Vector2(colBest, headerY), "BEST LAP", hCol, font, DrawTextAlign.Left, scale);
            if (ShowPeakStats)
            {
                DrawText(new Vector2(colAcc, headerY), "ACCGS", hCol, font, DrawTextAlign.Left, scale);
                DrawText(new Vector2(colDec, headerY), "DECGS", hCol, font, DrawTextAlign.Left, scale);
                DrawText(new Vector2(colLat, headerY), "LATGS", hCol, font, DrawTextAlign.Left, scale);
                DrawText(new Vector2(colTop, headerY), "TOP MPH", hCol, font, DrawTextAlign.Left, scale);
            }

            float y = headerY + step;
            int pos = 1;

            foreach (Racer r in LeaderboardFinish)
            {
                Color c = r.Driver != null && r.Driver.IsPlayer ? Color.Yellow : Color.White;
                DrawText(new Vector2(colPos, y), pos + "º", c, font, DrawTextAlign.Left, scale);
                DrawText(new Vector2(colPI, y), r.VehicleData.TextPerformanceIndex, c, font, DrawTextAlign.Left, scale);
                DrawText(new Vector2(colCar, y), BoardRowName(r), c, font, DrawTextAlign.Left, scale);
                DrawLapStats(r, y, c, font, scale, colBest, colAcc, colDec, colLat, colTop);
                y += step;
                pos++;
            }

            List<Racer> racing = Racers.Where(v => v.FinalPosition == 0).OrderByDescending(v => v.RaceProgress).ToList();
            foreach (Racer r in racing)
            {
                Color c = r.Driver != null && r.Driver.IsPlayer ? Color.Yellow : Color.White;
                DrawText(new Vector2(colPos, y), pos + "º", c, font, DrawTextAlign.Left, scale);
                DrawText(new Vector2(colPI, y), r.VehicleData.TextPerformanceIndex, c, font, DrawTextAlign.Left, scale);
                DrawText(new Vector2(colCar, y), BoardRowName(r), c, font, DrawTextAlign.Left, scale);
                DrawLapStats(r, y, c, font, scale, colBest, colAcc, colDec, colLat, colTop);
                y += step;
                pos++;
            }
        }

        static void DrawLapStats(Racer r, float y, Color c, DrawTextFont font, float scale, float colBest, float colAcc, float colDec, float colLat, float colTop)
        {
            TimeSpan? best = r.BestLap();
            DrawText(new Vector2(colBest, y), best.HasValue ? best.Value.ToString("m':'ss'.'f") : "-", c, font, DrawTextAlign.Left, scale);
            if (!ShowPeakStats) return;
            DrawText(new Vector2(colAcc, y), r.VehicleData.PeakAccelG.ToString("0.00"), c, font, DrawTextAlign.Left, scale);
            DrawText(new Vector2(colDec, y), Math.Abs(r.VehicleData.PeakDecelG).ToString("0.00"), c, font, DrawTextAlign.Left, scale);
            DrawText(new Vector2(colLat, y), r.VehicleData.PeakLateralG.ToString("0.00"), c, font, DrawTextAlign.Left, scale);
            DrawText(new Vector2(colTop, y), ARS.MpsToMph(r.VehicleData.PeakTopSpeedMps).ToString("0"), c, font, DrawTextAlign.Left, scale);
        }
        public static float GetPercent(float current, float max)
        {
            return (current / max) * 100;
        }
        internal void SetSPLVisibility(bool state)
        {
            try { foreach (Prop p in World.GetAllProps()) if (p.Model == "prop_mp_max_out_lrg") if (state) p.Alpha = 255; else p.Alpha = 0; } catch (Exception) { }
        }

        public static Color GradientAtoBtoC(Color A, Color B, Color C, float percentage)
        {
            percentage = ARS.Clamp((float)percentage, 0, 100);

            if (percentage <= 50)
            {
                var red = ARS.Remap(percentage, 0, 100, A.R, B.R);
                var green = ARS.Remap(percentage, 0, 100, A.G, B.G);
                var blue = ARS.Remap(percentage, 0, 100, A.B, B.B);

                Color result = Color.FromArgb((int)red, (int)green, (int)blue);
                return result;

            }
            else
            {
                var red = ARS.Remap(percentage, 50, 100, B.R, C.R);
                var green = ARS.Remap(percentage, 50, 100, B.G, C.G);
                var blue = ARS.Remap(percentage, 50, 100, B.B, C.B);

                Color result = Color.FromArgb((int)red, (int)green, (int)blue);
                return result;

            }

        }
        internal bool IsFreeCamActive => _freeCam.IsActive;
        internal List<int> FlareEffects => _flareFx;

        internal void SetLoadingPromptText(string t)
        {
            Function.Call(Hash._0xABA17D7CE615ADBF, "STRING");
            Function.Call(Hash._ADD_TEXT_COMPONENT_STRING, t);
            Function.Call(Hash._0xBD12F8228410D9B4, 5);
        }

        public void StartRace()
        {
            Log(LogImportance.Info, "Starting race");

            string refusal;
            if (!PreflightStartRace(out refusal))
            {
                UI.Notify("~o~" + refusal);
                Log(LogImportance.Error, "Start Race refused: " + refusal);
                return;
            }

            if (!_trackInstanced)
            {
                InstanceTrack();
                if (!_trackInstanced) return;
            }
            if (!_gridInstanced)
            {
                InstanceGrid();
                if (!_gridInstanced) return;
            }

            Log(LogImportance.Info, "Adding player to grid");
            AddPlayerToGrid();
            if (Racers.Count == 0)
            {
                UI.Notify("~r~[ARS]:~w~ no racers to start - check the car roster and Max PI Deviation.");
                if (!_freeCam.IsActive) Function.Call(Hash.DO_SCREEN_FADE_IN, 500);
                return;
            }

            Game.MissionFlag = true;
            Log(LogImportance.Info, "Setting up");
            SetupRace(true, true);
            Log(LogImportance.Info, "Set up");

            if (!_freeCam.IsActive || Game.IsScreenFadedIn) Function.Call(Hash.DO_SCREEN_FADE_IN, 500);
            Game.SetControlNormal(2, GTA.Control.VehicleLookBehind, 1f);

            _trackInstanced = false;
            _gridInstanced = false;
            Log(LogImportance.Info, "Started race");

        }

        public void CleanRacers()
        {
            Game.MissionFlag = false;
            foreach (Racer r in Racers)
            {
                r.Delete();
            }
            Racers.Clear();

            _nextInLine = 0;
            _raceTimedFinishMs = 0;
            _gameTimeNextInLine = 0;
            LeaderboardFinish.Clear();
            RaceStatus = RaceState.None;
            PlayerParticipating = false;
            _gridInstanced = false;
        }

        bool AddPlayerToGrid()
        {
            Vehicle cv = Game.Player.Character.CurrentVehicle;
            if (CanWeUse(cv))
            {
                Racers.Add(new Racer(cv, Game.Player.Character));
                PlayerParticipating = true;
                return true;
            }
            return false;
        }

        void PlaceCars()
        {
            GridSort sort = GridSort.Power;
            string setting = SettingsMenuStore.Get("GridSorting", "Random");
            if (!Enum.TryParse(setting, true, out sort)) sort = GridSort.Power;

            TrackLoader.BuildGridSlots(RouteNodes, NodeHalfWidths, IsPointToPoint, Racers.Count, GridPositions);
            GridBuilder.Place(Racers, GridPositions, RouteNodes, IsPointToPoint, sort);
        }

        public void SetupRace(bool placecars, bool tunecars)
        {

            _raceTimedFinishMs = 0;
            LeaderboardFinish.Clear();
            Log(LogImportance.Info, "Initializing racers");
            foreach (Racer r in Racers)
            {
                r.Initialize();
            }

            if (placecars)
            {
                Log(LogImportance.Info, "Placing cars");
                PlaceCars();
                Racer mostPower = Racers.OrderBy(v => Function.Call<float>(Hash.GET_VEHICLE_ACCELERATION, v.Car)).ToList()[0];
                int r = ((RouteNodes.Count / 3) * RaceMenuStore.GetInt("Laps", 6)) + (Racers.Count * 100) + (int)Math.Round(Function.Call<float>(Hash.GET_VEHICLE_ACCELERATION, mostPower.Car) * 400, 0);
                RaceReward = (int)(Math.Round((float)r / 100)) * 100;
            }

            if (tunecars && SmartTuning)
            {
                Log(LogImportance.Info, "Smart tuning queued for " + Racers.Count + " cars");
                SmartTuner.Enqueue(Racers);
            }

            foreach (Racer r in Racers) if (r.Car.CurrentBlip != null) r.Car.CurrentBlip.Color = BlipColor.Blue;
            RaceStatus = RaceState.NotInitiated;
        }
        public void StartCountdown()
        {
            HelpMessages.Add("The prize is: ~g~" + RaceReward.ToString() + "~w~$.");

            int aiCount = Racers.Count - 1;
            int aiIndex = 0;
            foreach (Racer r in Racers)
            {
                if (r.Driver.IsPlayer) continue;
                float t = aiCount <= 0 ? 0.5f : (float)aiIndex / aiCount;
                r.Aggression = (float)Math.Round(ARS.Remap(t, 0f, 1f, 0f, 100f, true) / 10f) * 10f;
                aiIndex++;
            }

            RaceStatus = RaceState.Countdown;
            _countdownTickMs = Game.GameTime;
            _countdown = _maxCountdown;

        }

        public static bool TryGetSurfaceHash(Vector3 start, Vector3 end, Entity ignore, out int materialId, out Vector3 normal)
        {
            materialId = 0;
            normal = Vector3.Zero;
            if (ignore == null) return false;

            int shape = Function.Call<int>(Hash._0x28579D1B8F8AAC80, start.X, start.Y, start.Z, end.X, end.Y, end.Z, 0.3f, (int)IntersectOptions.Map, ignore.Handle, 7);

            OutputArgument didhit = new OutputArgument();
            OutputArgument hitpos = new OutputArgument();
            OutputArgument snormal = new OutputArgument();
            OutputArgument materialhash = new OutputArgument();
            OutputArgument entity = new OutputArgument();

            int status = Function.Call<int>(Hash._0x65287525D951F6BE, shape, didhit, hitpos, snormal, materialhash, entity);
            if (status == 0 || !didhit.GetResult<bool>()) return false;

            materialId = materialhash.GetResult<int>();
            normal = snormal.GetResult<Vector3>();
            return true;
        }

        static public List<Vector3> GridPositions = new List<Vector3>();
        static public bool IsPointToPoint = false;

        void DumpPaceIndex()
        {
            List<string[]> rows = new List<string[]>();
            rows.Add(new[] { "model", "electricNative", "cacheFlag", "class", "accelRaw", "topMph", "grip", "piCurrent", "piNoCorrection", "name" });
            int electricCount = 0;
            int probeErrors = 0;
            foreach (KeyValuePair<string, float> entry in ModelPaceIndexCache)
            {
                float topMph, grip, accel;
                if (!ModelTopSpeedMphCache.TryGetValue(entry.Key, out topMph)) continue;
                if (!ModelGripCache.TryGetValue(entry.Key, out grip)) continue;
                if (!ModelAccelCache.TryGetValue(entry.Key, out accel)) continue;
                bool cacheFlag = false;
                ModelElectricCache.TryGetValue(entry.Key, out cacheFlag);
                string name;
                if (!ModelNameCache.TryGetValue(entry.Key, out name)) name = entry.Key;
                int hash;
                bool numeric = int.TryParse(entry.Key, out hash);
                string vehicleClass = numeric ? Function.Call<int>(Hash.GET_VEHICLE_CLASS_FROM_NAME, hash).ToString() : "?";
                string electricNative = "n/a";
                if (numeric)
                {
                    try
                    {
                        electricNative = Function.Call<int>((Hash)0x1FCB07FE230B6639, hash) == 0 ? "0" : "1";
                        if (electricNative == "1") electricCount++;
                    }
                    catch (Exception ex)
                    {
                        probeErrors++;
                        if (probeErrors == 1) Log(LogImportance.Error, "GET_IS_VEHICLE_ELECTRIC probe failed: " + ex.Message, true);
                    }
                }
                rows.Add(new[]
                {
                    entry.Key,
                    electricNative,
                    cacheFlag ? "electric" : "ice",
                    vehicleClass,
                    accel.ToString("0.####", CultureInfo.InvariantCulture),
                    topMph.ToString("0.##", CultureInfo.InvariantCulture),
                    grip.ToString("0.###", CultureInfo.InvariantCulture),
                    entry.Value.ToString("0.##", CultureInfo.InvariantCulture),
                    ComputePaceIndex(topMph, grip, accel, false).ToString("0.##", CultureInfo.InvariantCulture),
                    name
                });
            }
            int[] widths = new int[rows[0].Length];
            for (int r = 0; r < rows.Count; r++)
            {
                for (int i = 0; i < widths.Length; i++)
                {
                    if (rows[r][i].Length > widths[i]) widths[i] = rows[r][i].Length;
                }
            }
            bool[] rightAligned = { false, true, false, true, true, true, true, true, true, false };

            List<string> lines = new List<string>();
            lines.Add(string.Join("\t", rows[0]));
            for (int r = 1; r < rows.Count; r++)
            {
                string[] cells = new string[rows[r].Length];
                for (int i = 0; i < cells.Length; i++)
                {
                    if (i == cells.Length - 1) cells[i] = rows[r][i];
                    else cells[i] = rightAligned[i] ? rows[r][i].PadLeft(widths[i]) : rows[r][i].PadRight(widths[i]);
                }
                lines.Add(string.Join("\t", cells));
            }
            File.WriteAllLines(ScriptsFolder + @"\pidump.txt", lines.ToArray());
            UI.Notify("~b~[ARS]:~w~ pidump.txt - " + (rows.Count - 1) + " models, " + electricCount + " electric by GET_IS_VEHICLE_ELECTRIC" + (probeErrors > 0 ? ", " + probeErrors + " probe errors" : ""));
        }

        void BuildRosterFromDumpFile()
        {
            UI.Notify("~b~[ARS]:~w~ Generating the car list from modeldump.txt.");

            string dumpFilePath = ScriptsFolder + @"\modeldump.txt";
            if (File.Exists(dumpFilePath))
            {
                string content = File.ReadAllText(dumpFilePath);
                string[] modelNames = content.Split(new[] { ',', '\n', '\r', ' ' }, StringSplitOptions.RemoveEmptyEntries);

                List<string> keys = new List<string>();
                foreach (string modelName in modelNames)
                {
                    string trimmedName = modelName.Trim().ToLowerInvariant();
                    if (!string.IsNullOrEmpty(trimmedName)) keys.Add(trimmedName);
                }

                int added = VehicleCatalog.AddToRoster(keys);
                RefreshRoster();
                UI.Notify("~b~[ARS]:~w~ " + VehicleCatalog.SelectedRosterFile + " - " + added + " new model(s) added from " + modelNames.Length + " entries.");
            }
            else
            {
                UI.Notify("~r~[ARS]:~w~ modeldump.txt not found in " + ScriptsFolder + "\\");
            }
        }

        void BuildRosterFromEnum()
        {
            UI.Notify("~b~[ARS]:~w~ Generating the car list for all SHVDN known vehicles in the game.");

            List<string> keys = new List<string>();
            int inspected = 0;
            foreach (VehicleHash hash in Enum.GetValues(typeof(VehicleHash)).Cast<VehicleHash>())
            {
                Model m = new Model(hash);
                if (m.IsBike || m.IsQuadbike || m.IsBicycle || m.IsCar) keys.Add(RosterKeyFor(hash));
                if (++inspected % 25 == 0) Script.Yield();
            }
            int added = VehicleCatalog.AddToRoster(keys);
            RefreshRoster();
            UI.Notify("~b~[ARS]:~w~ " + VehicleCatalog.SelectedRosterFile + " - " + added + " new model(s) added.");
        }

        public void HandleCheats()
        {
            if (WasCheatStringJustEntered("arspidump")) DumpPaceIndex();
            if (WasCheatStringJustEntered("arsbuildcarlist")) BuildRosterFromEnum();

            if (WasCheatStringJustEntered("arsbuilddumpcarlist")) BuildRosterFromDumpFile();
        }

        public static bool NodeExists(XmlNode node, string name)
        {
            return node.SelectSingleNode(name) != null;
        }
        public static XmlElement GetChild(XmlNode node, string name)
        {
            XmlNode n = node.SelectSingleNode(name);
            if (n == null) return null;
            else return n as XmlElement;
        }
        public static string GetAttribute(XmlElement node, string name)
        {
            if (node == null) return "";
            if (node.HasAttribute(name)) return node.GetAttribute(name);
            return "";
        }

        static TrackStartInfo FindNearestTrackStartInfo(Vector3 position)
        {
            TrackStartInfo nearest = null;
            float best = 150f * 150f;
            foreach (TrackStartInfo info in TrackStartInfos)
            {
                float dist = info.JoinPosition.DistanceTo2D(position);
                if (dist < best)
                {
                    best = dist;
                    nearest = info;
                }
            }
            return nearest;
        }

        public static TrackPoint FindNearestTrackPoint(Vector3 position, int referenceNode, int behind, int ahead)
        {
            int count = ARS.TrackPoints.Count;
            if (count == 0) return null;
            int center = referenceNode >= 0 && referenceNode < count ? referenceNode : 0;
            int first = ARS.IsPointToPoint ? Math.Max(center - behind, 0) : center - behind;
            int last = ARS.IsPointToPoint ? Math.Min(center + ahead, count - 1) : center + ahead;
            TrackPoint nearest = null;
            float best = float.MaxValue;
            for (int i = first; i <= last; i++)
            {
                int node = ARS.IsPointToPoint ? i : ((i % count) + count) % count;
                float dist = ARS.TrackPoints[node].Position.DistanceTo2D(position);
                if (dist < best)
                {
                    best = dist;
                    nearest = ARS.TrackPoints[node];
                }
            }
            return nearest;
        }

        public static float Circumradius3D(Vector3 a, Vector3 b, Vector3 midpoint)
        {
            return TrackLoader.Circumradius3D(a, b, midpoint);
        }

        public static float HillGripDeltaGs(Vector3 start, Vector3 midpoint, Vector3 end, float velocity)
        {
            if (velocity <= 0f) return 0f;

            float s0 = 0f;
            float s1 = Vector2.Distance(new Vector2(start.X, start.Y), new Vector2(midpoint.X, midpoint.Y));
            float s2 = s1 + Vector2.Distance(new Vector2(midpoint.X, midpoint.Y), new Vector2(end.X, end.Y));

            if (s1 <= 0.001f || s2 <= s1 + 0.001f) return 0f;

            Vector2 p0 = new Vector2(s0, start.Z);
            Vector2 p1 = new Vector2(s1, midpoint.Z);
            Vector2 p2 = new Vector2(s2, end.Z);

            Vector2 ab = p1 - p0;
            Vector2 bc = p2 - p1;
            Vector2 ac = p2 - p0;

            float abLen = ab.Length();
            float bcLen = bc.Length();
            float acLen = ac.Length();
            if (abLen < 0.001f || bcLen < 0.001f || acLen < 0.001f) return 0f;

            float cross = (ab.X * ac.Y) - (ab.Y * ac.X);
            float curvature = (2f * cross) / (abLen * bcLen * acLen);
            if (Math.Abs(curvature) < 0.00001f) return 0f;

            const float Gravity = 9.8f;
            float deltaGs = ((velocity * velocity) * curvature) / Gravity;

            if (float.IsNaN(deltaGs) || float.IsInfinity(deltaGs)) return 0f;
            return deltaGs;
        }

        public static float LocalSlope(Vector3 a, Vector3 b)
        {
            float dx = b.X - a.X;
            float dy = b.Y - a.Y;
            float ground = (float)Math.Sqrt(dx * dx + dy * dy);
            if (ground < 0.001f) return 0f;
            return (b.Z - a.Z) / ground;
        }

        public static float GetHillGsLossAtCurrentTrackPoint(Racer r)
        {
            if (r == null || !CanWeUse(r.Car) || TrackPoints.Count < 2) return 0f;

            int node = (int)Clamp(r.CurrentTrackPoint.Node, 0, TrackPoints.Count - 1);
            int forwardOffset = (int)Clamp((int)(Math.Max(r.Car.Velocity.Length(), 5f) * 0.5f), 1, 50);
            int aheadNode = node + forwardOffset;
            if (aheadNode >= TrackPoints.Count) aheadNode -= TrackPoints.Count;

            Vector3 from = TrackPoints[node].Position;
            Vector3 to = TrackPoints[aheadNode].Position;

            float horizontalDistance = Vector2.Distance(new Vector2(from.X, from.Y), new Vector2(to.X, to.Y));
            if (horizontalDistance <= 0.001f) return 0f;

            float climbAngleRad = (float)Math.Atan2(to.Z - from.Z, horizontalDistance);
            if (Math.Abs(climbAngleRad) <= 0.0001f) return 0f;

            float gravityScale = r.Handling.Gravity / 9.8f;
            float lossGs = (float)Math.Abs(Math.Sin(climbAngleRad)) * gravityScale;

            if (float.IsNaN(lossGs) || float.IsInfinity(lossGs)) return 0f;
            return Clamp(lossGs, 0f, 2f);
        }

        public static float GetHillGsLossAtCurrentVelocityVector(Racer r)
        {
            if (r == null || !CanWeUse(r.Car)) return 0f;

            Vector3 v = r.Car.Velocity;
            float horizontalSpeed = new Vector2(v.X, v.Y).Length();
            if (horizontalSpeed <= 0.001f) return 0f;

            float climbAngleRad = (float)Math.Atan2(v.Z, horizontalSpeed);
            if (Math.Abs(climbAngleRad) <= 0.0001f) return 0f;

            float gravityScale = r.Handling.Gravity / 9.8f;
            float lossGs = (float)Math.Abs(Math.Sin(climbAngleRad)) * gravityScale;

            if (float.IsNaN(lossGs) || float.IsInfinity(lossGs)) return 0f;
            return Clamp(lossGs, 0f, 2f);
        }

        public static float GetHillGripMultiplierAtCurrentTrackPoint(Racer r, float factor = 1f)
        {
            if (r == null || !CanWeUse(r.Car) || TrackPoints.Count < 2) return 1f;
            if (factor <= 0f) factor = 1f;

            int node = (int)Clamp(r.CurrentTrackPoint.Node, 0, TrackPoints.Count - 1);
            int forwardOffset = (int)Clamp((int)(Math.Max(r.Car.Velocity.Length(), 5f) * 0.5f), 1, 50);
            int aheadNode = node + forwardOffset;
            if (aheadNode >= TrackPoints.Count) aheadNode -= TrackPoints.Count;

            Vector3 from = TrackPoints[node].Position;
            Vector3 to = TrackPoints[aheadNode].Position;

            float horizontalDistance = Vector2.Distance(new Vector2(from.X, from.Y), new Vector2(to.X, to.Y));
            if (horizontalDistance <= 0.001f) return 1f;

            float climbAngleRad = (float)Math.Atan2(to.Z - from.Z, horizontalDistance);
            float slopeAngleRad = Math.Abs(climbAngleRad);
            float normalizedSlope = slopeAngleRad / ((float)Math.PI * 0.5f);
            float loss = normalizedSlope * factor;
            float multiplier = 1f - loss;
            if (float.IsNaN(multiplier) || float.IsInfinity(multiplier)) return 1f;
            return Clamp(multiplier, 0f, 1f);
        }

        public static float HillGripMultiplierFromVelocity(Racer r, float factor = 1f)
        {
            if (r == null || !CanWeUse(r.Car)) return 1f;
            if (factor <= 0f) factor = 1f;

            Vector3 v = r.Car.Velocity;
            float horizontalSpeed = new Vector2(v.X, v.Y).Length();
            if (horizontalSpeed <= 0.001f) return 1f;

            float climbAngleRad = (float)Math.Atan2(v.Z, horizontalSpeed);
            float slopeAngleRad = Math.Abs(climbAngleRad);
            float normalizedSlope = slopeAngleRad / ((float)Math.PI * 0.5f);
            float loss = normalizedSlope * factor;
            float multiplier = 1f - loss;
            if (float.IsNaN(multiplier) || float.IsInfinity(multiplier)) return 1f;
            return Clamp(multiplier, 0f, 1f);
        }

        const float HillGripKPerDegree = 0.693147f / 15f;
        const float HillGripMin = 0.5f;
        public static float HillGripFactorFromPitchAngle(float pitchDegrees, Racer r)
        {
            float gravityRatio = (r != null && r.Handling != null) ? (r.Handling.Gravity / 9.8f) : 1f;
            float effectiveDeg = Math.Abs(pitchDegrees) * Math.Max(gravityRatio, 0.1f);
            float factor = (float)Math.Exp(-HillGripKPerDegree * effectiveDeg);
            if (float.IsNaN(factor) || float.IsInfinity(factor)) return 1f;
            factor = Clamp(factor, HillGripMin, 1f);
            return 1f - Math.Min((1f - factor) * HillGripEffect, 0.9f);
        }

        public static float CornerApexSpeed(CornerPoint c, Racer r)
        {

            float radius = c.DetectedRadius;
            if (radius <= 0f) radius = c.GetPreciseRadius();

            if (float.IsInfinity(radius) || float.IsNaN(radius) || radius == 0f) return AiConstants.MaxSpeed;

            float vehicleGripGs = r.VehicleData.CurrentMechanicalGrip;

            float baseSpd = (float)Math.Sqrt((vehicleGripGs * r.Handling.Gravity) * radius);
            if (float.IsNaN(baseSpd) || float.IsInfinity(baseSpd)) return AiConstants.MaxSpeed;

            return ARS.Clamp(baseSpd, AiConstants.MinSpeed, AiConstants.MaxSpeed);
        }

        static public float SlidingBoundingBoxWidth(Entity e)
        {
            if (!CanWeUse(e)) return 0f;

            return ARS.Remap(ARS.Clamp(Vector3.Angle(e.ForwardVector, e.Velocity.Normalized), 0f, 90f), 0f, 90f, e.Model.GetDimensions().X, e.Model.GetDimensions().Y);
        }

        const float DefaultWheelBase = 2.7f;
        const float MinWheelBase = 1f;
        const float MaxWheelBase = 6f;

        static public float GetWheelBase(Vehicle vehicle)
        {
            if (!CanWeUse(vehicle)) return DefaultWheelBase;
            Vector3 front, rear;
            if (!WheelAxleCentre(vehicle, "wheel_lf", "wheel_rf", out front)) return DefaultWheelBase;
            if (!WheelAxleCentre(vehicle, "wheel_lr", "wheel_rr", out rear)) return DefaultWheelBase;
            float length = Math.Abs(Vector3.Dot(front - rear, vehicle.ForwardVector));
            if (float.IsNaN(length) || length < MinWheelBase || length > MaxWheelBase) return DefaultWheelBase;
            return length;
        }

        static bool WheelAxleCentre(Vehicle vehicle, string leftBone, string rightBone, out Vector3 centre)
        {
            bool hasLeft = vehicle.HasBone(leftBone);
            bool hasRight = vehicle.HasBone(rightBone);
            centre = Vector3.Zero;
            if (hasLeft && hasRight) centre = (vehicle.GetBoneCoord(leftBone) + vehicle.GetBoneCoord(rightBone)) * 0.5f;
            else if (hasLeft) centre = vehicle.GetBoneCoord(leftBone);
            else if (hasRight) centre = vehicle.GetBoneCoord(rightBone);
            return hasLeft || hasRight;
        }

        static void ReportNullCar()
        {
            if (_nullCarReported) return;
            _nullCarReported = true;
            Log(LogImportance.Error, "[MEMORY] A racer reached the wheel reads with no vehicle handle; grip reads return the neutral value until it has one.");
        }

        static public unsafe ulong GetWheelsPtr(Vehicle handle)
        {
            if (handle == null) { ReportNullCar(); return 0; }
            var address = (ulong)handle.MemoryAddress;
            if (WheelsPtr == 0x0)
            {
                IntPtr addr = (IntPtr)VehicleMemory.FindPattern("\x3B\xB7\x48\x0B\x00\x00\x7D\x0D", "xx????xx");

                if (addr != null)
                {
                    WheelsPtr = *(uint*)(addr + 2) - 8;
                    Log(LogImportance.Info, "[MEMORY] Learned the handling offset: " + WheelsPtr);

                }
                else
                {

                    Log(LogImportance.Error, "[MEMORY] Wheels offset not found on this game build; wheel reads disabled.");
                    return 0;
                }
            }
            return *((ulong*)(address + WheelsPtr));
        }

        static public unsafe int GetNumWheels(Vehicle handle)
        {
            if (handle == null) { ReportNullCar(); return 0; }

            if (NumWheelsOffset == 0x0)
            {
                IntPtr addr = (IntPtr)VehicleMemory.FindPattern("\x3B\xB7\x48\x0B\x00\x00\x7D\x0D", "xx????xx");

                if (addr != null)
                {
                    NumWheelsOffset = *(uint*)(addr + 2);
                }
                else
                {
                    return 0;
                }
            }
            GameVersion gameVersion = Game.Version;
            var address = (ulong)handle.MemoryAddress;
            return *((int*)(address + NumWheelsOffset));
        }

        static public unsafe List<ulong> GetWheelPtrs(Vehicle handle)
        {
            var wheelPtr = GetWheelsPtr(handle);
            var numWheels = GetNumWheels(handle);
            List<ulong> wheelPtrs = new List<ulong>();

            if (wheelPtr == 0 || numWheels <= 0 || numWheels > 8) return wheelPtrs;
            for (int i = 0; i < numWheels; i++)
            {
                var wheelAddr = *((ulong*)(wheelPtr + 0x008 * (ulong)i));
                wheelPtrs.Add(wheelAddr);
            }
            return wheelPtrs;
        }

        static public unsafe float MeanWheelGripMultiplier(Vehicle handle)
        {
            List<ulong> wheelPtrs = GetWheelPtrs(handle);
            if (wheelPtrs.Count == 0) return 1f;
            ulong offset = 0x198;
            double total = 0d;
            foreach (var wheel in wheelPtrs)
            {
                total += Math.Round(*((float*)(wheel + offset)), 2);
            }
            return (float)(total / wheelPtrs.Count);
        }

        static public unsafe float BrakeLockSlip(Vehicle handle)
        {
            List<ulong> wheelPtrs = GetWheelPtrs(handle);
            float w = 0f;
            foreach (var wheel in wheelPtrs)
            {
                float slip = *((float*)(wheel + 0x174));
                if (slip > w) w = slip;
            }
            return w;
        }

        static public unsafe float MaxWheelSlip(Vehicle handle)
        {
            List<ulong> wheelPtrs = GetWheelPtrs(handle);
            ulong offset = 0x174;
            float w = 0f;
            foreach (var wheel in wheelPtrs)
            {
                float pos = (float)Math.Round(*((float*)(wheel + offset)), 2);
                if (Math.Abs(pos) > Math.Abs(w)) w = pos;
            }
            return w;
        }

        static public unsafe List<float> WheelPowers(Vehicle handle)
        {
            if (WheelPowerOffset == 0x0)
            {

                IntPtr addr = (IntPtr)VehicleMemory.FindPattern(
                    "\x0F\x2F\x00\x00\x00\x00\x00\x0F\x97\xC0\xEB\x00\xD1",
                    "xx???xx???x?x");
                if (addr != null)
                {
                    uint steerOffset = *(uint*)(addr + 3);
                    WheelPowerOffset = steerOffset + 8;
                    Log(LogImportance.Info, "[MEMORY] Learned wheel power offset: 0x" + WheelPowerOffset.ToString("X") + " (steer 0x" + steerOffset.ToString("X") + ")");
                }
                else
                {
                    WheelPowerOffset = 0x1D8;
                    Log(LogImportance.Info, "[MEMORY] Wheel power pattern not found, using fallback 0x1D8");
                }
            }

            List<ulong> wheelPtrs = GetWheelPtrs(handle);
            List<float> powers = new List<float>();
            foreach (var wheel in wheelPtrs)
            {
                float p = *((float*)(wheel + WheelPowerOffset));
                powers.Add(p);
            }
            return powers;
        }

        static public unsafe bool AnyWheelRotationBelow(Vehicle handle, float minRadPerSecond)
        {
            List<ulong> wheelPtrs = GetWheelPtrs(handle);
            if (wheelPtrs.Count == 0) return false;
            foreach (var wheel in wheelPtrs)
            {
                if (Math.Abs(*((float*)(wheel + 0x170))) < minRadPerSecond) return true;
            }
            return false;
        }

        static public unsafe float MinWheelRotation(Vehicle handle)
        {
            List<ulong> wheelPtrs = GetWheelPtrs(handle);
            float min = float.MaxValue;
            foreach (var wheel in wheelPtrs)
            {
                float spin = Math.Abs(*((float*)(wheel + 0x170)));
                if (spin < min) min = spin;
            }
            return min;
        }

        static public unsafe float MaxWheelSideSlip(Vehicle handle)
        {
            List<ulong> wheelPtrs = GetWheelPtrs(handle);
            float w = 0f;
            foreach (var wheel in wheelPtrs)
            {
                float slip = Math.Abs(*((float*)(wheel + 0x1C4)));
                if (slip > w) w = slip;
            }
            return w;
        }

        static public unsafe int WheelsOffGround(Vehicle handle)
        {
            List<ulong> wheelPtrs = GetWheelPtrs(handle);
            int off = 0;
            foreach (var wheel in wheelPtrs)
            {
                if (*((float*)(wheel + 0x1C4)) == 0f) off++;
            }
            return off;
        }

        static public unsafe List<float> WheelSideSlipAngles(Vehicle handle)
        {
            List<ulong> wheelPtrs = GetWheelPtrs(handle);
            List<float> slips = new List<float>();
            foreach (var wheel in wheelPtrs)
            {
                slips.Add(*((float*)(wheel + 0x1C4)));
            }
            return slips;
        }

        static public unsafe List<float> WheelSlips(Vehicle handle)
        {
            List<ulong> wheelPtrs = GetWheelPtrs(handle);
            ulong offset = 0x174;
            List<float> slips = new List<float>();
            foreach (var wheel in wheelPtrs)
            {
                float slip = (float)Math.Round(*((float*)(wheel + offset)), 3);
                slips.Add(slip);
            }
            return slips;
        }

        public static int ClosestNodeToPlace(Vector3 v, List<Vector3> PathRoute)
        {
            int closest = 0;
            float best = float.MaxValue;
            for (int i = 0; i < PathRoute.Count; i++)
            {
                float distance = PathRoute[i].DistanceTo(v);
                if (distance >= best) continue;
                best = distance;
                closest = i;
            }
            return closest;
        }

        static Random _random = new Random();

        public static int GetRandomInt(int min, int max)
        {
            return _random.Next(min, max);
        }

        const float ApexMarkHeight = 1.6f;
        const int LaneBlinkPeriodMs = 400;
        const int LaneBlinkOnMs = 200;
        const float LaneHeightCurrent = 0.100f;
        const float LaneHeightCarried = 0.125f;
        const float LaneHeightRequest = 0.150f;
        // The demand is not drawn for now: the aim alone says where the lane is, and two lines of one colour read as one.
        const bool DrawLaneRequest = false;
        // Track analysis keeps the lane and drops the rest: corner regions, crests and the live crest checks are off.
        const bool DrawTrackAnalysisExtras = false;
        const float LaneHeightWalls = 0.175f;

        public static void DrawLine(Vector3 from, Vector3 to, Color color)
        {
            Function.Call(Hash.DRAW_LINE, from.X, from.Y, from.Z, to.X, to.Y, to.Z, color.R, color.G, color.B, color.A);
        }

        const float GLineMetresPerG = 1f;
        const float GLineMaxGs = 4f;
        const int GLineRestartMs = 200;
        readonly Vector3[] _playerAccelSamples = new Vector3[VehicleState.AccelWindow];
        int _playerAccelHead;
        int _playerAccelCount;
        int _playerAccelSampleTime;
        Vector3 _playerLastSpeed;

        void UpdatePlayerGLine(Vehicle car)
        {
            Vector3 speed = Function.Call<Vector3>(Hash.GET_ENTITY_SPEED_VECTOR, car, false);
            int now = Game.GameTime;
            int elapsed = now - _playerAccelSampleTime;
            if (elapsed > GLineRestartMs)
            {
                _playerAccelCount = 0;
                _playerAccelSampleTime = now;
                _playerLastSpeed = speed;
                return;
            }
            if (elapsed >= VehicleState.AccelIntervalMs)
            {
                float dt = Math.Max(elapsed * 0.001f, 0.001f);
                _playerAccelSamples[_playerAccelHead] = (speed - _playerLastSpeed) / dt;
                _playerAccelHead = (_playerAccelHead + 1) % VehicleState.AccelWindow;
                if (_playerAccelCount < VehicleState.AccelWindow) _playerAccelCount++;
                _playerLastSpeed = speed;
                _playerAccelSampleTime = now;
            }
            if (_playerAccelCount == 0) return;

            Vector3 sum = Vector3.Zero;
            for (int i = 0; i < _playerAccelCount; i++) sum += _playerAccelSamples[i];
            Vector3 g = new Vector3(sum.X, sum.Y, 0f) / (_playerAccelCount * 9.8f);
            float magnitude = g.Length();
            if (magnitude < 0.02f) return;
            if (magnitude > GLineMaxGs) g *= GLineMaxGs / magnitude;

            Vector3 origin = car.Position + Vector3.WorldUp * (car.Model.GetDimensions().Z + 0.2f);
            Vector3 side = Vector3.Cross(Vector3.WorldUp, g).Normalized;
            Vector3 tip = origin + g * GLineMetresPerG;
            DrawLine(origin - side * 0.3f, origin + side * 0.3f, Color.White);
            DrawLine(origin, tip, Color.Cyan);
            DrawText(tip + Vector3.WorldUp * 0.15f, magnitude.ToString("0.0", CultureInfo.InvariantCulture) + "Gs", Color.Cyan, 0.45f);
        }

        void OnAbort(object sourc, EventArgs e)
        {

            try
            {
            World.RenderingCamera = null;
            Function.Call(Hash._STOP_ALL_SCREEN_EFFECTS);
            if (_freeCam.IsActive)
            {
                _freeCam.Toggle();

            }
            Function.Call(Hash.DO_SCREEN_FADE_IN, 500);

            foreach (int fx in _flareFx) Function.Call(Hash.STOP_PARTICLE_FX_LOOPED, fx);
            if (CanWeUse(FreeCamRide)) FreeCamRide.Delete();

            foreach (Prop p in CustomProps) if (CanWeUse(p)) p.Delete();
            foreach (Prop p in AutoGeneratedProps) if (CanWeUse(p)) p.Delete();
            foreach (Prop p in StartLineFlares) if (CanWeUse(p)) p.Delete();
            foreach (Racer racer in Racers)
            {
                racer.Delete();
            }

            Racers.Clear();
            if (Racertext != null && Racertext.IsLoaded) Racertext.Unload();
            if (CountdownScaleform != null)
            {
                CountdownScaleform.Dispose();

            }
            InstructionalScaleform.Unload();
            InstructionalScaleform.Dispose();
            InstructionalScaleform = null;
            Function.Call(Hash._0x10D373323E5B9C0D);

            }
            catch (Exception abortError)
            {

                try { Log(LogImportance.Error, "Abort teardown failed: " + abortError.Message); } catch { }
            }
            finally
            {
                RaceStatus = RaceState.None;
                PlayerParticipating = false;
                LeaderboardFinish.Clear();
                _raceTimedFinishMs = 0;
                _routeEditorActive = false;

                try
                {
                    Ped player = Game.Player.Character;
                    if (player != null)
                    {
                        player.HasGravity = true;
                        Vehicle vehicle = player.CurrentVehicle;
                        if (CanWeUse(vehicle))
                        {
                            VehicleMemory.SetThrottle(vehicle, 0f);
                            VehicleMemory.SetBrakes(vehicle, 0f);
                            VehicleMemory.SetSteerInput(vehicle, 0f);
                            VehicleMemory.SetSteerAngle(vehicle, 0f);
                        }
                    }
                }
                catch (Exception)
                {
                }
            }
        }

        public static bool WasCheatStringJustEntered(string cheat)
        {
            return Function.Call<bool>(Hash._0x557E43C447E700A8, Game.GenerateHash(cheat));
        }

        static string TcsShareFromSlip(string[] offers, string rawSlip)
        {
            float raw;
            if (rawSlip == null || !float.TryParse(rawSlip, NumberStyles.Float, CultureInfo.InvariantCulture, out raw)) return null;
            float share = raw * (100f / SlipPeakRatio);
            string best = offers[0];
            foreach (string offer in offers)
            {
                if (Math.Abs(int.Parse(offer, CultureInfo.InvariantCulture) - share) < Math.Abs(int.Parse(best, CultureInfo.InvariantCulture) - share)) best = offer;
            }
            return best;
        }

        void LoadSettings()
        {
            Thread.CurrentThread.CurrentCulture = CultureInfo.GetCultureInfo("en-US");

            Log(LogImportance.Info, "Checking the Settings folder ...");

            ScriptSettings retiredDevToggles = ScriptSettings.Load(SettingsFolder + @"\Menu-DevSettings.ini");
            string legacyBrakeLearning = retiredDevToggles.GetValue<string>("MENU", "BrakeLearning", null);
            string legacyStagedSpawns = retiredDevToggles.GetValue<string>("MENU", "StagedSpawns", null);

            ScriptSettings retiredPaceSettings = ScriptSettings.Load(SettingsFolder + @"\Menu-Settings.ini");
            string legacyPaceMode = retiredPaceSettings.GetValue<string>("MENU", "PaceMode", null);
            string legacyTcsLimit = retiredPaceSettings.GetValue<string>("MENU", "TcsSlipThreshold", null);
            string legacyTcsSpan = retiredPaceSettings.GetValue<string>("MENU", "TcsTaperSpan", null);
            string legacyOverspeed = retiredPaceSettings.GetValue<string>("MENU", "OverspeedEnabled", null);
            SettingsRepair.CreateMissingFiles();
            SettingsRepair.PruneOwnedFiles();

            RaceMenuStore = new MenuSettings(SettingsFolder + @"\Menu-Race.ini");
            SettingsMenuStore = new MenuSettings(SettingsFolder + @"\Menu-Settings.ini");
            DebugMenuStore = new MenuSettings(SettingsFolder + @"\Menu-Debug.ini");
            DevelopmentMenuStore = new MenuSettings(SettingsFolder + @"\Menu-Development.ini");

            SettingsMenuStore.Migrate("TcsSlipLimitPct", TcsShareFromSlip(SettingsRepair.OptionsFor("TcsSlipLimitPct"), legacyTcsLimit));
            SettingsMenuStore.Migrate("TcsTaperSpanPct", TcsShareFromSlip(SettingsRepair.OptionsFor("TcsTaperSpanPct"), legacyTcsSpan));

            SettingsMenuStore.Migrate("OverspeedCut", legacyOverspeed == null ? null : (legacyOverspeed.Equals("True", StringComparison.OrdinalIgnoreCase) ? "0.2" : "Off"));

            Log(LogImportance.Info, "Loading per-menu settings (Menu-*.ini) ...");

            ScriptSettings legacySettings = ScriptSettings.Load(SettingsFolder + @"\Menu-Racers.ini");
            foreach (string key in SettingsRepair.DeclaredKeys("Menu-Settings.ini"))
                SettingsMenuStore.Migrate(key, legacySettings.GetValue<string>("MENU", key, null));
            ScriptSettings legacyDebug = ScriptSettings.Load(SettingsFolder + @"\Menu-DevSettings.ini");
            foreach (Options option in DebugToggles.Keys.ToArray())
                DebugMenuStore.Migrate(option.ToString(), legacyDebug.GetValue<string>("MENU", option.ToString(), null));
            ScriptSettings legacyRacers = ScriptSettings.Load(SettingsFolder + @"\Settings.ini");

            SettingsMenuStore.Migrate("GridSorting", legacyRacers.GetValue<string>("RACERS", "GridSorting", null));
            SettingsMenuStore.Migrate("TimeoutSeconds", legacyRacers.GetValue<string>("RACERS", "TimeoutSeconds", null));
            SettingsMenuStore.Migrate("AIRacerAutofix", legacyRacers.GetValue<string>("RACERS", "AIRacerAutofix", null));

            string storedAutofix = SettingsMenuStore.Get("AIRacerAutofix", null);
            if (storedAutofix == "0") SettingsMenuStore.Set("AIRacerAutofix", AutofixMode.Off.ToString());
            else if (storedAutofix == "1") SettingsMenuStore.Set("AIRacerAutofix", AutofixMode.AutoRepair.ToString());
            else if (storedAutofix == "2") SettingsMenuStore.Set("AIRacerAutofix", AutofixMode.Invincible.ToString());

            else if (storedAutofix == "DamageResistant") SettingsMenuStore.Set("AIRacerAutofix", AutofixMode.AutoRepair.ToString());
            Autofix = ParseEnum(SettingsMenuStore.Get("AIRacerAutofix", AutofixMode.AutoRepair.ToString()), AutofixMode.AutoRepair);
            SettingsMenuStore.Migrate("UseMenyooSkins", legacyRacers.GetValue<string>("RACERS", "UseMenyooSkins", null));
            UseMenyooSkins = SettingsMenuStore.GetBool("UseMenyooSkins", UseMenyooSkins);
            string legacyAiNitro = legacyRacers.GetValue<string>("RACERS", "AiNitro", null);
            if (legacyAiNitro != null) NitrousEnabled = !legacyAiNitro.Equals("Never", StringComparison.OrdinalIgnoreCase);
            NitrousEnabled = SettingsMenuStore.GetBool("NitrousEnabled", NitrousEnabled);
            ForceFillGrid = SettingsMenuStore.GetBool("ForceFillGrid", ForceFillGrid);
            OverspeedCutPerTenthG = ParseOverspeedCut(SettingsMenuStore.Get("OverspeedCut", "0.2"));
            SmartTuning = SettingsMenuStore.GetBool("SmartTuning", SmartTuning);
            CornerOffsetMph = SettingsMenuStore.GetInt("CornerOffset", CornerOffsetMph);
            RouteOffsetMph = SettingsMenuStore.GetInt("RouteOffset", RouteOffsetMph);
            SteerDampingGain = SettingsMenuStore.GetFloat("SteerDampingGain", SteerDampingGain);
            DamperCountersteerMaxDeg = SettingsMenuStore.GetFloat("DamperCountersteerMaxDeg", DamperCountersteerMaxDeg);
            SteerCeilingFactor = SettingsMenuStore.GetFloat("SteerCeilingFactor", SteerCeilingFactor);
            SlipCutFloor = SettingsMenuStore.GetFloat("SlipCutFloor", SlipCutFloor);
            SettingsMenuStore.Migrate("BrakeLearning", legacyBrakeLearning);
            SettingsMenuStore.Migrate("StagedSpawns", legacyStagedSpawns);
            BrakeLearning = SettingsMenuStore.GetBool("BrakeLearning", BrakeLearning);
            BrakeLearningNotify = DevelopmentMenuStore.GetBool("BrakeLearningNotify", BrakeLearningNotify);
            SteerTrace = DevelopmentMenuStore.GetBool("SteerTrace", SteerTrace);
            SlipFromMemory = DevelopmentMenuStore.GetBool("SlipFromMemory", SlipFromMemory);
            GsLaneThrottleFloor = SettingsMenuStore.GetBool("GsLaneThrottleFloor", GsLaneThrottleFloor);
            OpenUpOnThrottleFloorPct = SettingsMenuStore.GetInt("OpenUpOnThrottleFloorPct", OpenUpOnThrottleFloorPct);
            TcsEnabled = SettingsMenuStore.GetBool("TcsEnabled", TcsEnabled);
            AbsEnabled = SettingsMenuStore.GetBool("AbsEnabled", AbsEnabled);
            TcsSlipLimitPct = SettingsMenuStore.GetInt("TcsSlipLimitPct", TcsSlipLimitPct);
            TcsTaperSpanPct = SettingsMenuStore.GetInt("TcsTaperSpanPct", TcsTaperSpanPct);
            DNFUnderTrack = SettingsMenuStore.GetBool("DNFUnderTrack", DNFUnderTrack);
            NoCollision = SettingsMenuStore.GetBool("NoCollision", NoCollision);
            MaxCrestLift = SettingsMenuStore.GetFloat("MaxCrestLift", MaxCrestLift);

            int legacyCrestEffect = retiredPaceSettings.GetValue<int>("MENU", "CrestEffect", -1);
            if (legacyCrestEffect > 0)
            {
                float legacyCap = legacyCrestEffect * 0.01f * 0.2f;
                string rounded = Math.Round(legacyCap, 1).ToString("0.0", CultureInfo.InvariantCulture);
                SettingsMenuStore.Migrate("MaxCrestLift", rounded);
                MaxCrestLift = SettingsMenuStore.GetFloat("MaxCrestLift", MaxCrestLift);
            }
            HillGripEffect = SettingsMenuStore.GetInt("HillGripEffect", 100) * 0.01f;
            RubberbandingPct = SettingsMenuStore.GetInt("Rubberbanding", 100);
            CurrentRubberbandMode = ParseEnum(SettingsMenuStore.Get("RubberbandMode", CurrentRubberbandMode.ToString()), CurrentRubberbandMode);

            StagedSpawns = false;
            TipRate = ParseEnum(SettingsMenuStore.Get("TipRate", TipRate.ToString()), TipRate);
            RaceMenuStore.Migrate("PaceMode", legacyPaceMode);

            string storedMode = RaceMenuStore.Get("PaceMode", null);
            if (string.Equals(storedMode, "Relative", StringComparison.OrdinalIgnoreCase) || string.Equals(storedMode, "Relative To Mine", StringComparison.OrdinalIgnoreCase)) RaceMenuStore.Set("PaceMode", PaceMode.RelativeToMine.ToString());
            CurrentPaceMode = ParseEnum(RaceMenuStore.Get("PaceMode", CurrentPaceMode.ToString()), CurrentPaceMode);
            PaceOffsetScale = RaceMenuStore.GetFloat("PaceOffset", PaceOffsetScale);
            PaceDeviationLimit = ParseDeviationLimit(RaceMenuStore.Get("PaceDeviation", "Off"));
            Log(LogImportance.Info, "Loaded per-menu settings.");

            ScriptSettings legacyDevConfig = ScriptSettings.Load(SettingsFolder + @"\DevConfig.ini");
            SettingsMenuStore.Migrate("Hotkeys", legacyDevConfig.GetValue<string>("GENERAL", "Hotkeys", null));
            SettingsMenuStore.Migrate("LogLevel", legacyDevConfig.GetValue<string>("GENERAL", "LogLevel", null));
            foreach (Options option in DebugToggles.Keys.ToArray())
                DebugMenuStore.Migrate(option.ToString(), legacyDevConfig.GetValue<string>("DEBUG", option.ToString(), null));

            HotkeysEnabled = SettingsMenuStore.GetBool("Hotkeys", HotkeysEnabled);
            MenuModifierKey = ParseMenuHotkey(SettingsMenuStore.GetInt("MenuModifierKey", (int)MenuModifierKey), MenuModifierKey);
            MenuKey = ParseMenuHotkey(SettingsMenuStore.GetInt("MenuKey", (int)MenuKey), MenuKey);
            LogLevel = ParseEnum(SettingsMenuStore.Get("LogLevel", LogLevel.ToString()), LogLevel);
            CornerCeilingMeters = SettingsMenuStore.GetInt("CornerCeiling", CornerCeilingMeters);
            AiNitroPct = SettingsMenuStore.GetInt("AiNitroPct", AiNitroPct);
            foreach (Options option in DebugToggles.Keys.ToArray())
                DebugToggles[option] = DebugMenuStore.GetBool(option.ToString(), DebugToggles[option]);
            DebugToggles[Options.ReverseRoute] = RaceMenuStore.GetBool("ReverseRoute", DebugToggles[Options.ReverseRoute]);

            SettingsRepair.CompleteOwnedKeys(RaceMenuStore, SettingsMenuStore, DebugMenuStore, DevelopmentMenuStore);
            SettingsRepair.DeleteLegacyFiles();
            Log(LogImportance.Info, "Checked the Settings folder.");
        }

        public enum LogImportance { Info, Error, Fatal, None }

        public static AutofixMode Autofix = AutofixMode.AutoRepair;

        static int ParseDeviationLimit(string value)
        {
            int limit;
            return int.TryParse(value, NumberStyles.Integer, CultureInfo.InvariantCulture, out limit) && limit > 0 ? limit : 0;
        }

        static float ParseOverspeedCut(string value)
        {
            float rate;
            return float.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out rate) && rate > 0f ? rate : 0f;
        }

        static T ParseEnum<T>(string value, T fallback) where T : struct
        {
            T parsed;
            return Enum.TryParse(value, true, out parsed) && Enum.IsDefined(typeof(T), parsed) ? parsed : fallback;
        }

        static Keys ParseMenuHotkey(int keyCode, Keys fallback)
        {
            Keys key = (Keys)keyCode;
            return MenuHotkeyKeys.Contains(key) ? key : fallback;
        }

        static string MenuHotkeyLabel(Keys key)
        {
            int keyCode = (int)key;
            if (keyCode >= 48 && keyCode <= 57) return ((char)keyCode).ToString() + " (" + keyCode + ")";
            if (keyCode >= 65 && keyCode <= 90) return ((char)keyCode).ToString() + " (" + keyCode + ")";
            if (key == Keys.LShiftKey) return "Left Shift (160)";
            if (key == Keys.RShiftKey) return "Right Shift (161)";
            if (key == Keys.LControlKey) return "Left Control (162)";
            if (key == Keys.RControlKey) return "Right Control (163)";
            if (key == Keys.LMenu) return "Left Alt (164)";
            if (key == Keys.RMenu) return "Right Alt (165)";
            return EnumLabel(key) + " (" + keyCode + ")";
        }

        static string EnumLabel(Enum value)
        {
            string name = value.ToString();
            string label = string.Empty;
            for (int i = 0; i < name.Length; i++)
            {
                if (i > 0 && char.IsUpper(name[i]) && !char.IsUpper(name[i - 1])) label += " ";
                label += name[i];
            }
            return label.Replace("Descendent", "Descending");
        }

        static string[] EnumLabels<T>() where T : struct
        {
            Array values = Enum.GetValues(typeof(T));
            string[] labels = new string[values.Length];
            for (int i = 0; i < values.Length; i++) labels[i] = EnumLabel((Enum)values.GetValue(i));
            return labels;
        }

        public static void Log(LogImportance i, string text, bool forced = false)
        {

            if (LogLevel > i && !forced) return;
            string log = "\n[" + DateTime.Now + "](" + i.ToString() + "): " + text;
            File.AppendAllText(LogPath, log);
        }

        public static bool CanWeUse(Entity entity)
        {
            return entity != null && entity.Exists();
        }

        void GetPaceSpan(out float min, out float max)
        {
            min = float.MaxValue; max = float.MinValue;
            foreach (float value in ModelPaceIndexCache.Values)
            {
                if (value < min) min = value;
                if (value > max) max = value;
            }
            if (min > max) { min = 0f; max = 200f; }
        }

        bool TryComputePlayerCarPaceIndex(out float pace)
        {
            pace = 0f;
            Vehicle v = Game.Player.Character.CurrentVehicle;
            if (!CanWeUse(v)) return false;
            if (ModelPaceIndexCache.TryGetValue(v.Model.Hash.ToString(), out pace)) return true;
            Model model = v.Model;
            float grip = Function.Call<float>((Hash)0x539DE94D44FDFD0D, model.Hash);
            float topSpeedMph = MpsToMph(Function.Call<float>((Hash)0xF417C2502FFFED43, model.Hash));
            if (grip <= 0f || topSpeedMph <= 0f) return false;
            float accel = Function.Call<float>(Hash.GET_VEHICLE_MODEL_ACCELERATION, model.Hash);
            bool isElectric = IsElectricModel(model.Hash);
            pace = ComputePaceIndex(topSpeedMph, grip, accel, isElectric);
            return !float.IsNaN(pace) && !float.IsInfinity(pace);
        }

        void DisplayHelpTextThisFrame(string text)
        {
            if (HelpMessages.Count > 0) return;
            Function.Call(Hash._SET_TEXT_COMPONENT_FORMAT, "STRING");
            Function.Call(Hash._ADD_TEXT_COMPONENT_STRING, text);
            Function.Call(Hash._DISPLAY_HELP_TEXT_FROM_STRING_LABEL, 0, false, false, -1);
        }

        static bool World3DToScreen2d(Vector3 pos, out Vector2 screen)
        {
            var x2dp = new OutputArgument();
            var y2dp = new OutputArgument();

            bool onScreen = Function.Call<bool>(Hash._WORLD3D_TO_SCREEN2D, pos.X, pos.Y, pos.Z, x2dp, y2dp);
            screen = new Vector2(x2dp.GetResult<float>(), y2dp.GetResult<float>());
            return onScreen;
        }

        public enum DrawTextAlign { Center, Left, Right }

        public enum DrawTextFont { Standard = 0, Cursive = 1, Leaderboard = 3, Condensed = 4, FixedWidthNumbers = 5, Pricedown = 7 }

        static string CardLabel(Racer racer)
        {
            ManeuverType type = racer.ActiveManeuver.Type;
            if (type == ManeuverType.DiveBomb)
                return "DIVEBOMB~n~brake +" + Math.Round(racer.DivebombBrakeBonus * 100f).ToString(CultureInfo.InvariantCulture) + "%";
            if (type == ManeuverType.DefendLane) return "DEFEND";
            if (type == ManeuverType.Yield)
                return "YIELD~n~gas " + Math.Round(Racer.YieldThrottleLevel * 100f).ToString(CultureInfo.InvariantCulture) + "%";
            if (type == ManeuverType.ChillOut)
                return "CHILL~n~" + Math.Round(Racer.ChillStandoffDistance).ToString(CultureInfo.InvariantCulture) + " m";
            return string.Empty;
        }

        public static void DrawText(Vector3 pos, string t, Color c, float scale)
        {
            if (!World3DToScreen2d(pos, out Vector2 screenInfo)) return;
            Function.Call(Hash._SET_TEXT_ENTRY, "STRING");
            Function.Call(Hash.SET_TEXT_CENTRE, true);
            Function.Call(Hash.SET_TEXT_COLOUR, c.R, c.G, c.B, c.A);
            Function.Call(Hash.SET_TEXT_SCALE, 1f, scale);
            Function.Call(Hash.SET_TEXT_DROP_SHADOW, true);
            Function.Call(Hash.SET_TEXT_FONT, (int)DrawTextFont.Standard);
            Function.Call(Hash._ADD_TEXT_COMPONENT_STRING, t);
            Function.Call(Hash._DRAW_TEXT, screenInfo.X, screenInfo.Y);
        }

        public static void DrawText(Vector2 pos, string t, Color c, DrawTextFont font, DrawTextAlign align, float scale)
        {
            Function.Call(Hash._SET_TEXT_ENTRY, "STRING");
            Function.Call(Hash.SET_TEXT_COLOUR, c.R, c.G, c.B, c.A);
            Function.Call(Hash.SET_TEXT_SCALE, 1f, scale);
            Function.Call(Hash.SET_TEXT_DROP_SHADOW, true);
            Function.Call(Hash.SET_TEXT_JUSTIFICATION, (int)align);
            Function.Call(Hash.SET_TEXT_FONT, (int)font);
            Function.Call(Hash._ADD_TEXT_COMPONENT_STRING, t);
            Function.Call(Hash._DRAW_TEXT, pos.X, pos.Y);
        }

        public static float GetDownforceGsAtSpeed(Racer r, float forwardMs, float lateralMs)
        {
            if (lateralMs <= 0f || float.IsNaN(lateralMs)) return 0f;
            float df = r.Handling.Downforce;

            if (df <= 1.0f)
            {
                float basePerWheel = r.Car.HasBone("spoiler") ? 0.070f : 0.035f;
                return GsOrZero(basePerWheel * lateralMs / 9.8f);
            }

            float topSpeed = Function.Call<float>((Hash)0xF417C2502FFFED43, r.Car.Model.Hash);
            float maxVel = Math.Max(topSpeed, 1f);
            float vFactor = Math.Min(Math.Max(forwardMs, 0f) / (maxVel * 0.9f), 1f);
            vFactor *= vFactor;

            float downForceScale;
            if (df > 100f)
            {
                if (DebugToggles[Options.HighDownforceOnline])
                {
                    float vRatio = Math.Max(vFactor - 0.2f, 0f);
                    downForceScale = (0.3f + 0.7f * vRatio) * (df / 100f);
                }
                else
                {
                    downForceScale = 0.3f;
                }
            }
            else
            {
                downForceScale = vFactor * df;
            }

            bool hasSpoiler = r.Car.HasBone("spoiler") && !r.Car.HasBone("spflap_l") && !r.Car.HasBone("spflap_r");
            float perWheel = hasSpoiler ? 0.035f + 0.070f * downForceScale : 0.035f * downForceScale;

            return GsOrZero(perWheel * lateralMs / 9.8f);
        }

        private static float GsOrZero(float gs) => (float.IsNaN(gs) || gs > 5f) ? 0f : gs;

        public static float GetLateralSpeed(Vehicle v)
        {
            if (!CanWeUse(v)) return 0f;
            Vector3 right = v.RightVector;
            right.Z = 0f;
            float rightLen = right.Length();
            if (rightLen < 0.0001f) return 0f;
            right /= rightLen;
            return Math.Abs(Vector3.Dot(v.Velocity, right));
        }

        public static float GetForwardSpeed(Vehicle v)
        {
            if (!CanWeUse(v)) return 0f;
            Vector3 forward = v.ForwardVector;
            forward.Z = 0f;
            float forwardLen = forward.Length();
            if (forwardLen < 0.0001f) return 0f;
            forward /= forwardLen;
            return Vector3.Dot(v.Velocity, forward);
        }

        static public void DisplayHelpTextTimed(string text, int time)
        {

            Function.Call(Hash._SET_TEXT_COMPONENT_FORMAT, "STRING");
            Function.Call(Hash._ADD_TEXT_COMPONENT_STRING, text);
            Function.Call(Hash._DISPLAY_HELP_TEXT_FROM_STRING_LABEL, 0, false, false, time);
        }

        static public void DisplayHelpText(string text)
        {
            if (HelpMessages.Count > 0) return;
            Function.Call(Hash._SET_TEXT_COMPONENT_FORMAT, "STRING");
            Function.Call(Hash._ADD_TEXT_COMPONENT_STRING, text);
            Function.Call(Hash._DISPLAY_HELP_TEXT_FROM_STRING_LABEL, 0, false, false, -1f);
        }

        VehicleColor[] _randomColors = { VehicleColor.MetallicRed, VehicleColor.MetallicRaceYellow, VehicleColor.MetallicBlue, VehicleColor.MetallicOrange, VehicleColor.MetallicSteelGray };

        List<string> _cachedCandidates = new List<string>();

        enum GridFill { Filled, NoPaceIndex, NoneInBand }

        GridFill FillCachedCandidates(int maxcars)
        {
            if (HardcodedRoster != null && HardcodedRoster.Count > 0)
            {
                _cachedCandidates = VehicleSelector.SelectHardcoded(HardcodedRoster, maxcars, GetRandomInt, text => Log(LogImportance.Info, text));
            }
            else
            {

                GetPaceSpan(out float paceMin, out float paceMax);
                if (ModelPaceIndexCache.Count > 0 && !ARS.IsBetween(_resolvedPaceTarget, paceMin, paceMax))
                    Log(LogImportance.Info, "Pace target " + _resolvedPaceTarget + " outside fleet span " + paceMin + ".." + paceMax + " - ranking from the closest end");
                DateTime rankingStarted = DateTime.UtcNow;
                _cachedCandidates = VehicleSelector.SelectClosestByPace(_vehiclePool, ModelPaceIndexCache, maxcars, GetRandomInt, text => Log(LogImportance.Info, text), _resolvedPaceTarget, PaceDeviationLimit);
                Log(LogImportance.Info, "Ranking took " + (int)(DateTime.UtcNow - rankingStarted).TotalMilliseconds + " ms for " + _vehiclePool.Count + " pool files");
                if (_cachedCandidates.Count == 0 && maxcars > 0)
                {

                    if (PaceDeviationLimit > 0 && ModelPaceIndexCache.Count > 0)
                    {
                        Log(LogImportance.Error, "No vehicles within " + PaceDeviationLimit + " PI of the target " + Math.Round(_resolvedPaceTarget) + " - widen Max PI Deviation or the pool");
                        return GridFill.NoneInBand;
                    }
                    Log(LogImportance.Error, "No pace index loaded for the pool - " + _vehiclePool.Count + " pool files, " + ModelPaceIndexCache.Count + " indexed");
                    return GridFill.NoPaceIndex;
                }
                return GridFill.Filled;
            }
            return GridFill.Filled;
        }

        static readonly List<string> _sillyNames = new List<string>();
        static readonly Random _sillyNameRandom = new Random();

        static void ResetSillyNames()
        {
            _sillyNames.Clear();
            try
            {

                string names = Path.Combine(ScriptsFolder, "Drivers", "names.txt");
                if (!File.Exists(names)) names = Path.Combine(ScriptsFolder, "sillynames.txt");
                _sillyNames.AddRange(File.ReadAllLines(names).Select(l => l.Trim()).Where(l => l.Length > 0));
            }
            catch (Exception) { }
            for (int i = _sillyNames.Count - 1; i > 0; i--) { int j = _sillyNameRandom.Next(i + 1); string t = _sillyNames[i]; _sillyNames[i] = _sillyNames[j]; _sillyNames[j] = t; }
        }

        static string NextSillyName()
        {
            if (_sillyNames.Count == 0) return null;
            string name = _sillyNames[_sillyNames.Count - 1];
            _sillyNames.RemoveAt(_sillyNames.Count - 1);
            return name;
        }

        void LoadGrid(int maxcars)
        {
            SetLoadingPromptText("Loading vehicles...");
            ResetSillyNames();
            Tips.Reset();

            Log(LogImportance.Info, "Loading vehicle models");
            Model LoadVehicleModel(string modelName)
            {
                int modelHash = 0;
                int.TryParse(modelName, out modelHash);
                if (modelHash == 0) Log(LogImportance.Info, "Loading Racer: " + modelName); else Log(LogImportance.Info, "Loading Racer: " + Function.Call<string>(Hash.GET_DISPLAY_NAME_FROM_VEHICLE_MODEL, modelHash));

                Model vehicleModel = new Model(modelName);
                Script.Wait(10);
                if (vehicleModel.IsValid)
                {
                    vehicleModel = new Model(modelName);
                }
                else
                {
                    int hashVehicleModel = 0;
                    int.TryParse(modelName, out hashVehicleModel);
                    vehicleModel = hashVehicleModel;
                }

                int loadTries = 0;
                while (!vehicleModel.IsLoaded && loadTries < 500)
                {
                    vehicleModel.Request();
                    Script.Wait(10);
                    loadTries++;
                }

                if (!vehicleModel.IsLoaded)
                {
                    Log(LogImportance.Error, "Model " + modelName + " failed to load after 5s, skipping.", true);
                    return vehicleModel;
                }

                return vehicleModel;
            }

            void RandomizeCarColour(Vehicle car)
            {
                if (car.ColorCombinationCount > 2) car.ColorCombination = GetRandomInt(0, car.ColorCombinationCount);
                else
                {
                    VehicleColor c = _randomColors[GetRandomInt(0, _randomColors.Length)];
                    car.PrimaryColor = c;
                    car.SecondaryColor = c;
                    car.PearlescentColor = c;
                }
            }

            Ped CreateDriverPed(Vehicle car)
            {
                Model driverModel = StreetRacerModels[GetRandomInt(0, StreetRacerModels.Length)];
                Ped driverPed = null;
                try
                {
                    Vector3 spawnPos = car.Position + car.RightVector * 5f + car.ForwardVector * 3f;
                    driverModel.Request(500);
                    int pedHandle = Function.Call<int>(Hash.CREATE_PED, 26, (int)driverModel.Hash, spawnPos.X, spawnPos.Y, spawnPos.Z, car.Heading, false, false);
                    driverPed = pedHandle > 0 ? new Ped(pedHandle) : null;
                }
                catch (Exception ex)
                {
                    Log(LogImportance.Error, "Failed to create ped: " + ex.Message, true);
                    return null;
                }

                int p = 0;
                while (!CanWeUse(driverPed) && p < 500)
                {
                    Script.Wait(10);
                    try
                    {
                        Vector3 retryPos = car.Position + car.RightVector * (5f + p * 0.1f) + car.ForwardVector * 3f;
                        int retryHandle = Function.Call<int>(Hash.CREATE_PED, 26, (int)driverModel.Hash, retryPos.X, retryPos.Y, retryPos.Z, car.Heading, false, false);
                        driverPed = retryHandle > 0 ? new Ped(retryHandle) : null;
                    }
                    catch (Exception)
                    {

                    }
                    p++;
                }

                return driverPed;
            }

            void AddRacer(Vehicle car, Ped driverPed)
            {
                if (CanWeUse(car))
                {
                    Racer r = new Racer(car, driverPed);

                    if (car == Game.Player.Character.CurrentVehicle) r.Name = Game.Player.Name;
                    else r.Name = NextSillyName() ?? r.Name;

                    r._baseName = r.Name;
                    r.DriverName = r.Name;

                    Racers.Add(r);
                }
            }

            bool SpawnRacer(string modelKey)
            {
                int before = Racers.Count;
                Vehicle car = null;
                Ped driverPed = null;
                try
                {
                    Model vehicleModel = LoadVehicleModel(modelKey);
                    if (!vehicleModel.IsLoaded)
                    {
                        Log(LogImportance.Error, "Skipping " + modelKey + " - model not loaded", true);
                        return false;
                    }

                    car = World.CreateVehicle(vehicleModel, RouteNodes[(Racers.Count + 1) * 10]);
                    car.Heading = (RouteNodes[2] - RouteNodes[0]).ToHeading();
                    car.InstallModKit();

                    RandomizeCarColour(car);
                    if (UseMenyooSkins)
                    {

                        MenyooAppearance.Apply(car);
                    }

                    driverPed = CreateDriverPed(car);
                    if (driverPed == null)
                    {
                        Log(LogImportance.Error, "Skipping " + modelKey + " - no driver ped.", true);
                        car.Delete();
                        return false;
                    }
                    AddRacer(car, driverPed);
                }
                catch (Exception ex)
                {
                    Log(LogImportance.Error, "Failed to load racer: " + ex.Message, true);
                    try { if (driverPed != null) driverPed.Delete(); } catch (Exception) { }
                    try { if (car != null) car.Delete(); } catch (Exception) { }
                }
                return Racers.Count > before;
            }

            List<string> spawnedKeys = new List<string>();
            foreach (string modelKey in _cachedCandidates)
            {
                if (SpawnRacer(modelKey)) spawnedKeys.Add(modelKey);
            }

            int cycle = 0;
            int failures = 0;
            while (ForceFillGrid && Racers.Count < maxcars && failures < spawnedKeys.Count)
            {
                bool spawned = SpawnRacer(spawnedKeys[cycle % spawnedKeys.Count]);
                cycle++;
                failures = spawned ? 0 : failures + 1;
            }
            if (Racers.Count < maxcars) Log(LogImportance.Info, "Grid: " + Racers.Count + " of " + maxcars + " cars spawned from " + _cachedCandidates.Count + " candidate(s), Force-Fill " + (ForceFillGrid ? "on" : "off") + ".");
            else if (cycle > 0) Log(LogImportance.Info, "Force-Fill: repeated the pool's models " + cycle + " time(s) to reach the grid target of " + maxcars + ".");

            RaceStatus = RaceState.None;
            PlayerParticipating = Racers.Any(r => r.Driver != null && r.Driver.IsPlayer);
            Function.Call(Hash._0x10D373323E5B9C0D);
        }

    }

}
