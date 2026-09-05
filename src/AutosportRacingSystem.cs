using GTA;
using GTA.Math;
using GTA.Native;
using LemonUI;
using LemonUI.Menus;
using System;
using System.Collections.Generic;
using System.Diagnostics;
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

    public enum Options
    {
        Race, RaceOptions, Brakepower, RestartRace, StartRace, Start, GridSize, Laps, LeaveRace, StopRace, Freecam, LoadTrack, DebugLevel, SaveTrack, UpdateTrackFile, CreateTrack, ExitCreator, TrackNameFilter, TrackList,
        SaveThisCar, SaveDriverModel, Disciplines, FindCustomProps, ShowAggro, ShowInputs, ShowTrackAnalysis, ShowPhysics, UseNearbyCars, ReloadSettings, ReverseRoute, GsAwarePreview, BrakeLearning, HighDownforceOnline
    }

    public enum DebugDisplay
    {
        None, Inputs, Speed, Positioning, PropEdit
    }
    public class ARS : Script
    {

        public static List<TrackStartInfo> TrackStartInfos = new List<TrackStartInfo>();

        public static List<TrackPoint> TrackPoints = new List<TrackPoint>();
        // Pre-computed apex table. Built in BuildApexCorners after track generation.
        public static List<CornerPoint> Corners = new List<CornerPoint>();
        public static List<Vehicle> GlobalTraffic = new List<Vehicle>();

        public static List<string> KnownTracks = new List<string>();
        public static List<Model> KnownVehicleModels = new List<Model>();
        // Raw grip (max traction, in G) per model, keyed by the XML <Model> hash text.
        public static Dictionary<string, float> ModelGripCache = new Dictionary<string, float>(StringComparer.OrdinalIgnoreCase);

        // Raw estimated top speed (mph) per model, keyed same as ModelGripCache.
        public static Dictionary<string, float> ModelTopSpeedMphCache = new Dictionary<string, float>(StringComparer.OrdinalIgnoreCase);

        // Raw acceleration (G, ~0-0.5) per model from GET_VEHICLE_MODEL_ACCELERATION. Coded but
        // not yet weighted into the pace index (PaceWeightPower = 0) — activate by raising the
        // weight and rebalancing the others so the weights still sum to 1.0.
        public static Dictionary<string, float> ModelAccelCache = new Dictionary<string, float>(StringComparer.OrdinalIgnoreCase);

        // Per-model electric flag from GET_IS_VEHICLE_ELECTRIC (0xD839450756ED5A80). Electric
        // cars use a different first-gear multiplier (×5.0 vs ×3.33 for ICE), so the same raw
        // acceleration stat yields different real thrust. Used when computing the effective
        // acceleration component (currently weight 0).
        public static Dictionary<string, bool> ModelElectricCache = new Dictionary<string, bool>(StringComparer.OrdinalIgnoreCase);

        // Pace index per model (0-1), keyed same as the stat caches. Built in BuildPowerCache
        // from a normalized, weighted blend of top speed (0-200 mph) and grip (0-3 G).
        public static Dictionary<string, float> ModelPaceIndexCache = new Dictionary<string, float>(StringComparer.OrdinalIgnoreCase);

        // Pace-index weights. Top speed carries 0.75, power (acceleration) 0.25. Grip is
        // ignored (weight 0) for now. Sum is 1.0; rebalance when activating grip.
        public static readonly float PaceWeightTopSpeed = 0.75f;
        public static readonly float PaceWeightGrip = 0f;
        public static readonly float PaceWeightPower = 0.25f;

        // Normalization ceilings — raw native values divided by these to produce a 0-1 component.
        public const float TopSpeedMphCeiling = 200f;
        public const float GripCeiling = 3f;

        // Acceleration normalization. The raw stat (GET_VEHICLE_MODEL_ACCELERATION) reads in G
        // over a ~0-0.5 range. First-gear multiplier differs by drivetrain: ICE ×3.33, electric
        // ×5.0. For the pace index, ICE is the baseline (no multiplier); electric gets the
        // difference (5.0 - 3.33 = 1.67) as its effective multiplier. The effective ceiling is
        // the raw ceiling × the electric multiplier (the highest any car can reach).
        public const float AccelRawCeiling = 0.5f;
        public const float IceFirstGear = 3.33f;
        public const float ElectricFirstGear = 5.0f;
        public static readonly float ElectricAccelMultiplier = ElectricFirstGear - IceFirstGear; // 1.67
        public static readonly float AccelEffectiveCeiling = AccelRawCeiling * ElectricAccelMultiplier;

        // Normalized, weighted pace index from raw native values. Top speed against 0-200 mph,
        // power (effective accel, electric-aware) against the effective ceiling — all clamped
        // 0-1 then weighted. Grip is read but weighted 0 (ignored) for now.
        public static float ComputePaceIndex(float topSpeedMph, float grip, float accelRaw, bool isElectric)
        {
            float accelEffective = accelRaw * (isElectric ? 10f : 30f);
            return topSpeedMph + grip * 4f + accelEffective;
        }

        public const float TopSpeedScaleDivisor = 466.67f;

        // Vehicle classes we never race.
        static readonly HashSet<VehicleClass> BlacklistedVehicleClasses = new HashSet<VehicleClass>
        {
            VehicleClass.Motorcycles, VehicleClass.OffRoad, VehicleClass.Industrial, VehicleClass.Utility,
            VehicleClass.Cycles, VehicleClass.Boats, VehicleClass.Helicopters, VehicleClass.Planes,
            VehicleClass.Service, VehicleClass.Emergency, VehicleClass.Military, VehicleClass.Commercial,
            VehicleClass.Trains
        };

        public static int RaceReward = 0;

        public static ScriptSettings SettingsFile;
        public static ScriptSettings DevSettingsFile;
        public static ScriptSettings DevMenuFile;
        public static ScriptSettings RaceSettingsFile;

        public static bool HideHudMode = false;

        public static Dictionary<Options, bool> DebugToggles = new Dictionary<Options, bool>()
    {
        { Options.ShowAggro, false },
        { Options.ShowInputs, false },
        { Options.ShowTrackAnalysis, false },
        { Options.ShowPhysics, false },
        { Options.UseNearbyCars, true },
        { Options.ReverseRoute, false },
        { Options.GsAwarePreview, true },
        { Options.BrakeLearning, true },
        { Options.HighDownforceOnline, true }
    };

        // Gs-aware preview steering: how much of the lane error is measured at the 1s
        // Gs-aware projection instead of at the car. 0 = legacy behavior, 1 = full preview.
        public static float GsAwarePreviewBlend = 0.5f;

        // Exponent of the lane-pursuit gain curve (x^exp). Below 1 boosts small errors
        // (punchier), 1 is linear, above 1 damps them (softer).
        public static float LaneGainExponent = 1f;

        public static List<Prop> CustomProps = new List<Prop>();
        public static List<Prop> AutoGeneratedProps = new List<Prop>();

        public static List<Racer> LeaderboardFinish = new List<Racer>();

        
        public static Prop FreeCamRide = null;

        public static PedHash[] StreetRacerModels = { PedHash.Car3Guy2, PedHash.Vinewood02AFY, PedHash.Stwhi02AMY, PedHash.StrPunk02GMY, PedHash.Stbla02AMY };
        public static List<Model> RacerModels = new List<Model> { "a_m_y_motox_01", "a_m_y_motox_02" };


        public static RaceState RaceStatus = RaceState.None;

        // Phased race instancing: track which setup phases have been completed.
        // Reset by CleanRacers (_gridInstanced) and CleanEverything (both).
        bool _trackInstanced;
        bool _gridInstanced;
        int _intendedOpponents = 4;
        List<int> _flareFx = new List<int>();
        FreeCamController _freeCam;



        static public ulong SteerOffset = 0x0;
        static public ulong ThrottleOffset = 0x0;
        static public ulong BrakeOffset = 0x0;
        public static ulong HandlingPtr = 0x0;
        public static ulong WheelsPtr = 0x0;
        public static ulong NumWheelsOffset = 0x0;

        
        public static List<int> Other = new List<int> { 555004797, -399872228, -1447280105, 722686013 };
        public static List<int> Road = new List<int> { 1187676648, 282940568, -108464011, 1187676648, -1084640111, };       
        public static List<int> Dirt = new List<int> { 1144315879, 510490462, -1907520769, -1885547121, -700658213, 2128369009,
            -1595148316,-765206029,509508168,1333033863,951832588,-840216541,-1907520769,510490462,-1942898710}; 
        public static List<int> Sand = new List<int> { 1288448767, };



        public static List<string> HelpMessages = new List<string>();

        
        public static List<Racer> Racers = new List<Racer>();
        public static int CatchupPosition = 0; 


        
        public static List<Vector3> RouteNodes = new List<Vector3>(); 
        public static Dictionary<int, float> Angles = new Dictionary<int, float>(); 
        public static Dictionary<int, float> TerrainGripMultipliers = new Dictionary<int, float>();  
        public static Dictionary<int, float> NodeHalfWidths = new Dictionary<int, float>(); 


        public static Dictionary<int, float> EditNodeHalfWidths = new Dictionary<int, float>();
        public static List<Prop> TrackLimits = new List<Prop>(); 
        public static XmlDocument CurrentFile = null;
        static bool _routeEditorActive = false;
        public static int DebugVisual = 0;


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
                FillKnownDisciplines(false);
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
                
                BuildPowerCache();
                RefreshPowerControls();
                FillCachedCandidates(DisciplineFilter, _intendedOpponents, true);
                RefreshTrackList(); // tracks are now discovered in _trackTags. Populate the menu list.
                Log(LogImportance.Info, "Initialization complete.", true);
                DisplayHelpTextTimed("~g~ARS has loaded.", 2000);
                HelpMessages.Add("Press ~INPUT_SPRINT~ + ~INPUT_CONTEXT~ to open the ~b~ARSe~w~ menu.");
                _loaded = true;
            }

            _isLoadingScript = false;
            _loadScriptTask = null;
        }
        public ARS()
        {
            _freeCam = new FreeCamController(this);

            Tick += OnTick;
            Aborted += OnAbort;

            File.WriteAllText(ScriptsFolder + @"\Log.log", "----------------------------");
            Log(LogImportance.Info, "Script initialized - " + DateTime.Now);
            File.AppendAllText(ScriptsFolder + @"\Log.log", "\n----------------------------");
            LoadSettings();
            InitializeMenu();


            string scriptName = "ARS";
            string scriptVer = "0.0.0.0";
            string scriptDate = "30/09/2018";

            
            Assembly assem = Assembly.GetExecutingAssembly();
            AssemblyName assemName = assem.GetName();
            Version ver = assemName.Version;
            scriptVer = ver.ToString() + " ~o~[PUBLIC BETA]~w~";
            scriptDate = File.GetLastWriteTimeUtc(ScriptsFolder + @"\ARS.dll").ToString();


            UI.Notify("~b~" + scriptName + "~g~e~w~" + "~y~ " + scriptVer + "~n~Build: ~g~" + scriptDate);

            UpdateChecker.CheckLatestRelease();
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
                foreach (string st in Directory.EnumerateFiles(dir))
                {
                    string n = System.IO.Path.GetFileName(st);
                    Log(LogImportance.Info, st + " - [" + string.Join(", ", TrackRepository.ReadTrackTags(st)) + "]");
                    _trackTags.Add(st, string.Join(", ", TrackRepository.ReadTrackTags(st)));
                    TrackStartInfo info = TrackRepository.ComputeTrackStartInfo(st);
                    if (info != null && !TrackStartInfos.Any(existing => existing.StartPosition.DistanceTo(info.StartPosition) < 1f)) TrackStartInfos.Add(info);

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

        static XmlDocument LoadXmlOrThrow(string path)
        {
            XmlDocument document = new XmlDocument();
            document.Load(path);
            return document;
        }

        public static List<string> GetTrackTags(string _routeNodes)
        {
            return TrackRepository.ReadTrackTags(_routeNodes);
            XmlDocument document = LoadXmlOrThrow(_routeNodes);

            List<string> tags = new List<string>();
            tags.Add(System.IO.Path.GetFileName(_routeNodes).ToLowerInvariant());
            XmlNodeList nl = document.SelectNodes("//Tags/*");
            foreach (XmlNode node in nl)
            {
                tags.Add(node.InnerText.ToLowerInvariant());
            }

            return tags;
        }

        public static Vector3 GetTrackStartPos(string _routeNodes)
        {
            return TrackRepository.ReadTrackStartPosition(_routeNodes);
            XmlDocument document = LoadXmlOrThrow(_routeNodes);

            List<string> tags = new List<string>();
            tags.Add(System.IO.Path.GetFileName(_routeNodes).ToLowerInvariant());
            XmlNode point = document.SelectNodes("//Route/Point")[0];
            XmlNodeList nl = point.ChildNodes;

            if (nl != null)
            {
                Vector3 p = Vector3.Zero;

                foreach (XmlNode node in nl)
                {
                    string t = node.InnerText;

                    float i = 0;
                    float.TryParse(t.Replace('.', ','), out i);
                    if (node.Name == "X") p.X = i;
                    if (node.Name == "Y") p.Y = i;
                    if (node.Name == "Z") p.Z = i;
                }
                return p;
            }
            return Vector3.Zero;
        }

        public static List<string> GetRacerTags(string _routeNodes)
        {
            return TrackRepository.ReadVehicleTags(_routeNodes);
            XmlDocument document = LoadXmlOrThrow(_routeNodes);

            List<string> tags = new List<string>();
            string dir = System.IO.Path.GetDirectoryName(_routeNodes);

            dir = dir.Split(System.IO.Path.DirectorySeparatorChar).Last().ToLowerInvariant();
            if (dir != "vehicles") tags.Add(dir);
            XmlNodeList nl = document.SelectNodes("//Disciplines/*");
            foreach (XmlNode node in nl)
            {
                tags.Add(node.InnerText.ToLowerInvariant());
            }


            return tags;

        }

        public static string GetRacerModel(string _routeNodes)
        {
            return TrackRepository.ReadVehicleModel(_routeNodes);
            try
            {
                XmlDocument document = new XmlDocument();
                document.Load(_routeNodes);
                XmlNode modelNode = document.SelectSingleNode("//Model");
                return modelNode?.InnerText?.Trim();
            }
            catch
            {
                return null;
            }
        }



        public void FillKnownDisciplines(bool allowScriptYield = true)
        {
            ModelGripCache.Clear();
            ModelTopSpeedMphCache.Clear();
            ModelAccelCache.Clear();
            ModelElectricCache.Clear();
            ModelPaceIndexCache.Clear();
            Log(LogImportance.Info, "-------------");
            Log(LogImportance.Info, "Learning available disciplines...");
            VehicleCatalog.Scan(_racerTagLookup, KnownVehicleModels, text => Log(LogImportance.Info, text), text => DisplayHelpTextTimed(text, 5000), Yield, allowScriptYield);
            KnownTracks = Directory.GetFiles(ScriptsFolder + @"\Tracks").ToList();
            Log(LogImportance.Info, "Done.");
            Log(LogImportance.Info, "-------------");
        }

        // Called on the main script thread (NOT from the background load task).
        // GTA natives are not safe to call off the main thread, so we fill the
        // modelName -> power cache here rather than during the XML scan.
        public void BuildPowerCache()
        {
            VehicleCatalog.BuildPowerCache(_racerTagLookup, ModelGripCache, ModelTopSpeedMphCache, ModelAccelCache, ModelElectricCache, BlacklistedVehicleClasses, text => Log(LogImportance.Info, text));
            BuildPaceIndex();
        }

        // Second pass over the raw stat caches: compute the 0-1 pace index per model from top
        // speed (0-200 mph) and grip (0-3 G), weighted 2:1. Power is read and cached but weighted 0.
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
                ModelPaceIndexCache[kv.Key] = ComputePaceIndex(mph, grip, accel, isElectric);
                indexed++;
            }
            Log(LogImportance.Info, "BuildPaceIndex: " + indexed + " models indexed.");
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
            return Function.Call<float>(Hash._0x53AF99BAA671CA47, v) / 0.75f;
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

        public unsafe static byte* FindPattern(string pattern, string mask)
        {
            ProcessModule module = Process.GetCurrentProcess().MainModule;

            ulong address = (ulong)module.BaseAddress.ToInt64();
            ulong endAddress = address + (ulong)module.ModuleMemorySize;

            for (; address < endAddress; address++)
            {
                for (int i = 0; i < pattern.Length; i++)
                {
                    if (mask[i] != '?' && ((byte*)address)[i] != pattern[i])
                    {
                        break;
                    }
                    else if (i + 1 == pattern.Length)
                    {
                        return (byte*)address;
                    }
                }
            }

            return null;
        }


        static public unsafe void SetSteerInput(Vehicle handle, float value)
        {

            if (!CanWeUse(handle)) return;

            if (SteerOffset == 0x0)
            {
                IntPtr addr = (IntPtr)FindPattern("\x74\x0A\xF3\x0F\x11\xB3\x1C\x09\x00\x00\xEB\x25", "xxxxx?????xx");
                if (addr != null)
                {
                    SteerOffset = *(uint*)(addr + 6);
                    Log(LogImportance.Info, "[MEMORY] Learned the steer offset: " + SteerOffset);
                }
            }
            else
            {
                var address = (ulong)handle.MemoryAddress;
                *((float*)(address + SteerOffset)) = value;
            }

        }
        static ulong SteerAngleOffset = 0x0;
        static public unsafe void SetSteerAngle(Vehicle handle, float value)
        {


            if (!CanWeUse(handle)) return;

            if (SteerAngleOffset == 0x0)
            {
                IntPtr addr = (IntPtr)FindPattern("\x74\x0A\xF3\x0F\x11\xB3\x1C\x09\x00\x00\xEB\x25", "xxxxx?????xx");
                if (addr != null)
                {

                    SteerAngleOffset = *(uint*)(addr + 6) + 8;
                    Log(LogImportance.Info, "[MEMORY] Learned the steer offset: " + SteerAngleOffset);
                }
            }
            else
            {
                var address = (ulong)handle.MemoryAddress;
                *((float*)(address + SteerAngleOffset)) = value;
            }

        }


        static public unsafe void SetThrottle(Vehicle handle, float value)
        {


            if (ThrottleOffset == 0x0)
            {
                IntPtr addr = (IntPtr)FindPattern("\x74\x0A\xF3\x0F\x11\xB3\x1C\x09\x00\x00\xEB\x25", "xxxxx?????xx");

                if (addr != null)
                {
                    ThrottleOffset = *(uint*)(addr + 6) + 0x10;
                    Log(LogImportance.Info, "[MEMORY] Learned the throttle offset: " + ThrottleOffset);

                }
            }
            else
            {
                var address = (ulong)handle.MemoryAddress;

                *((float*)(address + ThrottleOffset)) = value;
            }


        }

        static public unsafe void SetBrakes(Vehicle handle, float value)
        {
            if (!CanWeUse(handle)) return;



            if (BrakeOffset == 0x0)
            {
                IntPtr addr = (IntPtr)FindPattern("\x74\x0A\xF3\x0F\x11\xB3\x1C\x09\x00\x00\xEB\x25", "xxxxx?????xx");

                if (addr != null)
                {

                    BrakeOffset = *(uint*)(addr + 6) + 0x14;
                    Log(LogImportance.Info, "[MEMORY] Learned the BrakeOffset offset:" + BrakeOffset);

                }
            }
            else
            {
                var address = (ulong)handle.MemoryAddress;

                *((float*)(address + BrakeOffset)) = value;
            }

        }
        public static float RadToDeg(float rad)
        {
            return (rad * (180.0f / (float)Math.PI));
        }

        public static float DegToRad(float angle)
        {
            return (float)(Math.PI * angle / 180.0f);
        }

        

        
        List<Vector3> _routeSection = new List<Vector3>();
        Vector3 _bezierStartAnchor = Vector3.Zero;
        float _bezierScale = 1.5f;
        int _pathWidth = 5;
        int _pathDisplayFidelity = 1;



        int _countdownTickMs = 0;
        int _maxCountdown = 7;
        int _countdown = 7;

        Vector3 _freeCamMovement = Vector3.Zero;
        Vector3 _freeCamRotation = Vector3.Zero;


        public static Scaleform InstructionalScaleform = new Scaleform("INSTRUCTIONAL_BUTTONS");
        public static Scaleform Racertext = null;
        public static Scaleform DebugFrontend = new Scaleform("instructional_buttons");
        public static Scaleform Floatingtext = new Scaleform("HUD_FLOATING_HELP_TEXT");
        public static Scaleform CountdownScaleform = new Scaleform("MP_BIG_MESSAGE_FREEMODE");



        public static string DisciplineFilter = "sports";
        public static float PowerTargetScale = 0.52f;
        public static float PowerBracketScale = 5f;
        // Default script folder under GTA's `scripts\` (Drivers/, Tracks/, Vehicles/, Options.ini,
        // Log.log, etc.). All path constants below derive from this so the folder name lives in one place.
        public static string ScriptsFolder = @"scripts\AutosportRacingSystem";
        // Config .ini files (Options/Developer Settings/DevSettings/MemoryOffsets) live in a
        // dedicated Settings subfolder, derived from ScriptsFolder so the base stays in one place.
        public static string SettingsFolder => ScriptsFolder + @"\Settings";
        // Test-only: force a specific car into the pace-matched grid for the next few tests.
        // Match is by the XML <Model> InnerText (case-insensitive, e.g. "tiberius"). Flip to null to disable.
        public static List<string> AlwaysIncludeModelNames = new List<string> { "tiberius", "2100457220" };
        // Temp: bypass pace matching entirely and load only these models into the grid.
        // Set to null (or empty list) to re-enable pace-matched selection.
        public static List<string> HardcodedRoster = null;

        Dictionary<string, string> _racerTagLookup = new Dictionary<string, string>();
        public static int TrackListPos = 0;
        readonly ObjectPool _menuPool = new ObjectPool();
        NativeMenu _arsMenu;
        NativeMenu _raceMenu;

        // Menu track selector: the user picks a race from the list and Start Race uses it.
        NativeListItem<string> _trackListItem;
        readonly List<string> _trackListPaths = new List<string>(); // parallel to _trackListItem.Items
        string _selectedTrackPath = null;
        NativeListItem<string> _gridSizeItem;
        NativeListItem<string> _powerTargetItem;
        NativeListItem<string> _powerBracketItem;
        readonly List<float> _powerTargetValues = new List<float>();
        readonly List<float> _powerBracketValues = new List<float>();

        void InitializeMenu()
        {
            _arsMenu = new NativeMenu("ARS", "RACING")
            {
                UseMouse = false,
                DisableControls = true,
                Alignment = Alignment.Right
            };

            // ── Race submenu ──
            _raceMenu = new NativeMenu("Race", "Race")
            {
                UseMouse = false,
                DisableControls = true,
                Alignment = Alignment.Right
            };

            // ── Track items (flattened into Race) ──
            _trackListItem = new NativeListItem<string>("Select Track", "Pick the race to instance.", Array.Empty<string>());
            _trackListItem.ItemChanged += (sender, args) =>
            {
                if (args.Index >= 0 && args.Index < _trackListPaths.Count)
                    _selectedTrackPath = _trackListPaths[args.Index];
            };
            _raceMenu.Add(_trackListItem);

            NativeListItem<string> lapsItem = new NativeListItem<string>("Laps", "Number of laps before the race is considered finished.", new[] { "3", "5", "7", "9", "11", "13", "15", "17", "19" });
            lapsItem.ItemChanged += (sender, args) => SaveOptionSetting("Laps", lapsItem.Items[args.Index]);
            int laps = SettingsFile?.GetValue<int>("GENERAL_SETTINGS", "Laps", 5) ?? 5;
            lapsItem.SelectedIndex = Math.Max(0, lapsItem.Items.IndexOf(laps.ToString()));
            _raceMenu.Add(lapsItem);

            NativeItem instanceTrackItem = new NativeItem("Spawn Track", "Load the selected track and teleport to it.");
            instanceTrackItem.Activated += (sender, args) =>
            {
                _raceMenu.Visible = false;
                InstanceTrack();
                _raceMenu.Visible = true;
            };
            _raceMenu.Add(instanceTrackItem);

            // ── Grid items (flattened into Race) ──
            _gridSizeItem = new NativeListItem<string>("Target Grid Size", "Target number of vehicles for the grid.", new[] { "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12" });
            _gridSizeItem.ItemChanged += (sender, args) =>
            {
                if (args.Index >= 0 && args.Index <= 12)
                    _intendedOpponents = args.Index;
            };
            _intendedOpponents = (int)Clamp(_intendedOpponents, 0, 12);
            _gridSizeItem.SelectedIndex = _intendedOpponents;
            _raceMenu.Add(_gridSizeItem);

            _powerTargetItem = new NativeListItem<string>("Pace Target", "Target pace score for grid selection.", Array.Empty<string>());
            _powerTargetItem.ItemChanged += (sender, args) =>
            {
                if (args.Index >= 0 && args.Index < _powerTargetValues.Count)
                    PowerTargetScale = _powerTargetValues[args.Index];
            };
            _raceMenu.Add(_powerTargetItem);

            _powerBracketItem = new NativeListItem<string>("Pace Bracket", "Allowed pace index above or below the target.", Array.Empty<string>());
            _powerBracketItem.ItemChanged += (sender, args) =>
            {
                if (args.Index >= 0 && args.Index < _powerBracketValues.Count)
                    PowerBracketScale = _powerBracketValues[args.Index];
            };
            _raceMenu.Add(_powerBracketItem);

            NativeItem instanceGridItem = new NativeItem("Spawn Grid", "Spawn the AI grid with current settings. Hit again to re-instance after changing pace/size.");
            instanceGridItem.Activated += (sender, args) =>
            {
                _raceMenu.Visible = false;
                InstanceGrid();
                _raceMenu.Visible = true;
            };
            _raceMenu.Add(instanceGridItem);

            // Start: add player to grid, place cars, tune, countdown. Lives at the end of Race.
            NativeItem startItem = new NativeItem("Start Race", "Add yourself to the grid and start the race.");
            startItem.Activated += (sender, args) =>
            {
                _raceMenu.Visible = false;
                StartRace();
            };
            _raceMenu.Add(startItem);

            // ── Root-level actions ──

            // End Race: tear down race, clean everything. Listed last in the root.
            NativeItem endItem = new NativeItem("End Race", "Tear down the current race and clean everything.");
            endItem.Activated += (sender, args) =>
            {
                _arsMenu.Visible = false;
                CleanEverything();
                UI.Notify("~r~Race ended.~w~ Everything cleaned.");
            };

            // ── Other submenu (root) ──
            NativeMenu cameraMenu = new NativeMenu("Other", "Other", "Miscellaneous controls.")
            {
                UseMouse = false,
                DisableControls = true,
                Alignment = Alignment.Right
            };
            NativeItem freecamItem = new NativeItem("Freecam", "Toggle the ARS free camera.");
            freecamItem.Activated += (sender, args) =>
            {
                cameraMenu.Visible = false;
                _freeCam.Toggle();
            };
            cameraMenu.Add(freecamItem);

            // ── Settings submenu (root) ──
            NativeMenu settingsMenu = new NativeMenu("Dev Settings", "Dev Settings", "Configure ARS debug and race setup options.")
            {
                UseMouse = false,
                DisableControls = true,
                Alignment = Alignment.Right
            };
            AddDebugCheckbox(settingsMenu, Options.ShowAggro, "Show Pressure", "Show each racer's pressure on the leaderboard and above their car.");
            AddDebugCheckbox(settingsMenu, Options.ShowInputs, "Show Inputs", "Show the AI throttle and brake trail.");
            AddDebugCheckbox(settingsMenu, Options.ShowTrackAnalysis, "Show Track Analysis", "Show corner start, apex, and exit markers.");
            AddDebugCheckbox(settingsMenu, Options.ShowPhysics, "Show Physics", "Show physics debug information.");
            AddDebugCheckbox(settingsMenu, Options.UseNearbyCars, "Use Nearby Cars", "Use nearby vehicles when creating a race grid.");
            AddDebugCheckbox(settingsMenu, Options.ReverseRoute, "Reverse Route", "Race the loaded route in reverse.");
            AddDebugCheckbox(settingsMenu, Options.GsAwarePreview, "Gs-Aware Preview", "Measure lane steering error at the 1s Gs-aware projection instead of the car's current position.");
            AddDebugCheckbox(settingsMenu, Options.BrakeLearning, "Brake Learning", "Learn the effective braking decel that keeps the car at full brake ~75% of each braking phase.");
            AddDebugCheckbox(settingsMenu, Options.HighDownforceOnline, "High Downforce: Online", "For downforce >100, use the full online scaling; off = fall back to the 0.3 singleplayer default.");

            // ── Racers submenu (root) — reads/writes Settings\Settings.ini ([RACERS]) ──
            NativeMenu racersMenu = new NativeMenu("Racers", "Racers", "Grid sorting, race timeout, AI behaviour and tuning.")
            {
                UseMouse = false,
                DisableControls = true,
                Alignment = Alignment.Right
            };
            NativeListItem<string> gridSortItem = new NativeListItem<string>("Grid Sorting", "How the grid is ordered.", new[] { "Power", "PowerDescendent", "TopSpeed", "TopSpeedDescendent", "Random" });
            gridSortItem.ItemChanged += (sender, args) => SaveRacerSetting("GridSorting", gridSortItem.Items[args.Index]);
            gridSortItem.SelectedIndex = Math.Max(0, gridSortItem.Items.IndexOf(RaceSettingsFile.GetValue<string>("RACERS", "GridSorting", "Random")));
            racersMenu.Add(gridSortItem);

            NativeListItem<string> timeoutItem = new NativeListItem<string>("Timeout (s)", "Grace period after the first racer crosses the line.", new[] { "15", "30", "45", "60" });
            timeoutItem.ItemChanged += (sender, args) => SaveRacerSetting("TimeoutSeconds", timeoutItem.Items[args.Index]);
            timeoutItem.SelectedIndex = Math.Max(0, timeoutItem.Items.IndexOf(RaceSettingsFile.GetValue<int>("RACERS", "TimeoutSeconds", 30).ToString()));
            racersMenu.Add(timeoutItem);

            NativeListItem<string> autofixItem = new NativeListItem<string>("AI Racer Autofix", "0 = disabled, 1 = fixed when damaged, 2 = invincible.", new[] { "0", "1", "2" });
            autofixItem.ItemChanged += (sender, args) => SaveRacerSetting("AIRacerAutofix", autofixItem.Items[args.Index]);
            autofixItem.SelectedIndex = Math.Max(0, autofixItem.Items.IndexOf(RaceSettingsFile.GetValue<int>("RACERS", "AIRacerAutofix", 1).ToString()));
            racersMenu.Add(autofixItem);

            NativeListItem<string> tuningItem = new NativeListItem<string>("AI Tuning Level", "0 = none, 1 = visual, 2 = +performance, 3 = +engine boost.", new[] { "0", "1", "2", "3" });
            tuningItem.ItemChanged += (sender, args) => SaveRacerSetting("AITuningLevel", tuningItem.Items[args.Index]);
            tuningItem.SelectedIndex = Math.Max(0, tuningItem.Items.IndexOf(RaceSettingsFile.GetValue<int>("RACERS", "AITuningLevel", 1).ToString()));
            racersMenu.Add(tuningItem);

            // ── Options submenu (root) — hosts Dev Settings and Racers ──
            NativeMenu optionsMenu = new NativeMenu("Options", "Options", "Race setup and AI behaviour.")
            {
                UseMouse = false,
                DisableControls = true,
                Alignment = Alignment.Right
            };
            optionsMenu.AddSubMenu(racersMenu);
            optionsMenu.AddSubMenu(settingsMenu);

            // Link Race, Options and Other as submenus of root
            _arsMenu.AddSubMenu(_raceMenu);
            _arsMenu.AddSubMenu(optionsMenu);
            _arsMenu.AddSubMenu(cameraMenu);
            _arsMenu.Add(endItem);

            // Register all menus in the pool
            _menuPool.Add(_arsMenu);
            _menuPool.Add(_raceMenu);
            _menuPool.Add(optionsMenu);
            _menuPool.Add(settingsMenu);
            _menuPool.Add(racersMenu);
            _menuPool.Add(cameraMenu);
        }
        void SaveRacerSetting(string key, string value)
        {
            if (RaceSettingsFile == null)
                RaceSettingsFile = ScriptSettings.Load(SettingsFolder + @"\Settings.ini");
            RaceSettingsFile.SetValue("RACERS", key, value);
            RaceSettingsFile.Save();
        }
        void AddDebugCheckbox(NativeMenu menu, Options option, string title, string description)
        {
            NativeCheckboxItem checkbox = new NativeCheckboxItem(title, description, DebugToggles[option]);
            checkbox.CheckboxChanged += (sender, args) =>
            {
                DebugToggles[option] = checkbox.Checked;
                SaveDevToggle(option, checkbox.Checked);
            };
            menu.Add(checkbox);
        }
        void SaveDevToggle(Options option, bool value)
        {
            if (DevMenuFile == null)
                DevMenuFile = ScriptSettings.Load(SettingsFolder + @"\DevSettings.ini");
            if (DevMenuFile == null) return;
            DevMenuFile.SetValue("DEBUG", option.ToString(), value);
            DevMenuFile.Save();
        }
        void SaveOptionSetting(string key, string value)
        {
            if (SettingsFile == null)
                SettingsFile = ScriptSettings.Load(SettingsFolder + @"\Options.ini");
            SettingsFile.SetValue("GENERAL_SETTINGS", key, value);
            SettingsFile.Save();
        }
        // ── Phased race instancing ──
        // Each phase is a self-contained method callable individually or chained
        // back-to-back (for future presets that instance everything in one go).

        // Phase 1: load the selected track, compute route/corners, teleport the player.
        public void InstanceTrack()
        {
            CleanRacers();
            _gridInstanced = false;

            string trackPath = _selectedTrackPath;
            if (trackPath == null)
            {
                if (_trackListPaths.Count == 0)
                {
                    UI.Notify("~r~No tracks found. Create one with 'arscreatetrack'.");
                    return;
                }
                trackPath = _trackListPaths[0];
            }

            TrackLoader.LoadTrack(this, TrackLoader.LoadTrackFile(trackPath));

            if (!_freeCam.IsActive) Function.Call(Hash.DO_SCREEN_FADE_IN, 500);

            _trackInstanced = true;
            Log(LogImportance.Info, "Track instanced");
        }

        // Phase 2: spawn the AI grid with current pace/grid settings. Repeatable —
        // calling again tears down the old grid and respawns with updated settings.
        public void InstanceGrid()
        {
            if (!_trackInstanced)
            {
                UI.Notify("~r~Instance a track first.");
                return;
            }

            // Tear down any existing AI grid (player is not a racer yet at this point).
            CleanRacers();

            FillCachedCandidates(DisciplineFilter, _intendedOpponents, true);
            LoadGrid(DisciplineFilter, _intendedOpponents);

            if (Racers.Count == 0)
            {
                UI.Notify("~o~No vehicles found with current pace settings. Try widening the bracket.");
                _gridInstanced = false;
                return;
            }

            // Place AI cars on grid positions so the player can see the field.
            // PlaceCars also calls Initialize() which puts them in GridWait (handbrake on).
            PlaceCars();

            _gridInstanced = true;
            Log(LogImportance.Info, "Grid instanced (" + Racers.Count + " AI cars)");
        }

        // All-in-one chain: runs all three phases back-to-back with no UI stops.
        // Used by future presets; also the backward-compatible single-call path.
        public void StartRaceFromMenu()
        {
            InstanceTrack();
            if (!_trackInstanced) return;
            InstanceGrid();
            if (!_gridInstanced) return;
            StartRace();
        }

        // (Re)build the "Select Track" list from every discovered race, keeping the current
        // choice when the file still exists. Must run after FillKnownTracks has populated
        // _trackTags (that is, after the load task completes).
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

            // Preserve the chosen path if its file is still among the discovered tracks.
            string chosen = _selectedTrackPath;
            string keepVisible = chosen != null ? System.IO.Path.GetFileNameWithoutExtension(chosen) : null;

            List<string> sorted = new List<string>(_trackTags.Keys);
            sorted.Sort(StringComparer.OrdinalIgnoreCase);

            _trackListPaths.Clear();
            for (int i = 0; i < sorted.Count; i++) _trackListPaths.Add(sorted[i]);

            // The list item shows the file name (without extension). _trackListPaths holds the
            // full paths in the same order so selection maps back cleanly.
            _trackListItem.Items.Clear();
            foreach (string path in sorted)
                _trackListItem.Items.Add(System.IO.Path.GetFileNameWithoutExtension(path));

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
            _trackListItem.SelectedIndex = idx >= 0 ? idx : 0;

            if (_trackListPaths.Count > 0)
                _selectedTrackPath = _trackListPaths[_trackListItem.SelectedIndex];
        }

        void RefreshPowerControls()
        {
            if (_powerTargetItem == null || _powerBracketItem == null) return;

            // The pace index is 0-1 by construction; take the actual cache span so the sliders
            // don't offer values no car can match.
            float min = float.MaxValue;
            float max = float.MinValue;
            foreach (float pace in ModelPaceIndexCache.Values)
            {
                if (float.IsNaN(pace) || float.IsInfinity(pace)) continue;
                if (pace < min) min = pace;
                if (pace > max) max = pace;
            }
            if (min == float.MaxValue) { min = 0f; max = 1f; }

            PowerTargetScale = RoundToNearestEven(Clamp(PowerTargetScale, min, max));
            PowerBracketScale = RoundToNearestEven(Clamp(PowerBracketScale, 0f, max - min));

            _powerTargetValues.Clear();
            AddPowerValues(_powerTargetValues, min, max, 2.0f);
            _powerBracketValues.Clear();
            AddPowerValues(_powerBracketValues, 0f, max - min, 2.0f);

            _powerTargetItem.Items.Clear();
            foreach (float value in _powerTargetValues) _powerTargetItem.Items.Add(value.ToString("0"));
            _powerTargetItem.SelectedIndex = _powerTargetValues.Count / 2;
            if (_powerTargetValues.Count > 0) PowerTargetScale = _powerTargetValues[_powerTargetItem.SelectedIndex];

            _powerBracketItem.Items.Clear();
            foreach (float value in _powerBracketValues) _powerBracketItem.Items.Add(value.ToString("0"));
            _powerBracketItem.SelectedIndex = FindNearestPowerValue(_powerBracketValues, PowerBracketScale);
        }

        static void AddPowerValues(List<float> values, float min, float max, float step)
        {
            // Anchor the grid at clean multiples of the step (e.g. 2, 4, 6...) rather than
            // at the raw cache min, so the offered values are always rounded and the default lands
            // exactly on a grid point.
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
        void CleanEverything()
        {
            CleanRacers();
            foreach (Prop p in TrackLimits) if (CanWeUse(p)) p.Delete();
            foreach (Prop p in CustomProps) if (CanWeUse(p)) p.Delete();
            foreach (Prop p in AutoGeneratedProps) if (CanWeUse(p)) p.Delete();

            foreach (int fx in _flareFx) Function.Call(Hash.STOP_PARTICLE_FX_LOOPED, fx);
            _flareFx.Clear();
            LeaderboardFinish.Clear();
            _routeEditorActive = false;
            _raceTimedFinishMs = 0;

            Angles.Clear();
            RouteNodes.Clear();
            NodeHalfWidths.Clear();
            EditNodeHalfWidths.Clear();

            RaceStatus = RaceState.None;
            _countdown = _maxCountdown;
            _trackInstanced = false;
            _gridInstanced = false;
        }

        int _shortTickMs = Game.GameTime;


        
        int _nextInLine = 0;
        int _gameTimeNextInLine = 0;

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
                    if (DevSettingsFile.GetValue<bool>("GENERAL", "LoadAtStart", true) || WasCheatStringJustEntered("arson"))
                    {
                        StartLoadScript();
                    }
                    return;
                }

                













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
                if (_raceTimedFinishMs != 0 && _raceTimedFinishMs > Game.GameTime) DisplayHelpText("~y~" + (_raceTimedFinishMs - Game.GameTime) / 1000 + "s~w~ to end the race.");
                if (DebugVisual == (int)DebugDisplay.PropEdit) foreach (Prop p in CustomProps) if (CanWeUse(p) && p.IsInRangeOf(Game.Player.Character.Position, 100f)) World.DrawMarker(MarkerType.ReplayIcon, p.Position + new Vector3(0, 0, p.Model.GetDimensions().Z + 2f), Vector3.Zero, p.Rotation, new Vector3(2, 2, 2), Color.Green);

                if (RaceStatus == RaceState.None || RaceStatus == RaceState.NotInitiated)
                {
                    TrackStartInfo nearest = FindNearestTrackStartInfo(Game.Player.Character.Position);
                    if (nearest != null)
                    {
                        World.DrawMarker(MarkerType.ChevronUpx1, nearest.JoinPosition + new Vector3(0, 0, 2f), new Vector3(0, 0, -1f), Vector3.Zero, new Vector3(1f, 1f, 1f), Color.Yellow, false, true, 0, false, "", "", false);
                        float dist = nearest.JoinPosition.DistanceTo2D(Game.Player.Character.Position);
                        if (dist <= 5f)
                        {
                            DisplayHelpTextThisFrame("Press ~INPUT_CONTEXT~ to race at ~b~" + Path.GetFileNameWithoutExtension(nearest.TrackPath) + "~w~.");
                            if (!_arsMenu.Visible && CanWeUse(Game.Player.Character.CurrentVehicle) && !Game.IsControlPressed(2, GTA.Control.Sprint) && Game.IsControlJustPressed(2, GTA.Control.Context))
                            {
                                PowerTargetScale = ComputePlayerCarPaceIndex();
                                PowerBracketScale = 5f;
                                SelectTrackInMenu(nearest.TrackPath);
                                StartRaceFromMenu();
                            }
                        }
                    }
                }


                
                if (CountdownScaleform.IsLoaded && _countdown != _maxCountdown) CountdownScaleform.Render2D();

                
                if (DebugVisual == (int)DebugDisplay.PropEdit) DisplayHelpTextThisFrame("Add or remove any ~g~prop~w~ with the tool of your prefence. They must be ~y~persistent~w~.");



                if (ListenMode)
                {
                    if (Game.IsControlJustPressed(2, GTA.Control.Jump))
                    {
                        Vehicle playerVeh = Game.Player.Character.CurrentVehicle;
                        if (CanWeUse(playerVeh)) CreateVehicle(playerVeh, true);
                    }
                }




                


                
                

                
                if ((_routeEditorActive || RouteNodes.Count > 0) && DevSettingsFile.GetValue("GENERAL_SETTINGS", "Traffic", false) == false)
                {
                    Function.Call(Hash.SET_VEHICLE_DENSITY_MULTIPLIER_THIS_FRAME, 0f);
                    Function.Call(Hash.SET_RANDOM_VEHICLE_DENSITY_MULTIPLIER_THIS_FRAME, 0f);
                    Function.Call(Hash.SET_PARKED_VEHICLE_DENSITY_MULTIPLIER_THIS_FRAME, 0f);
                    if (Racers.Count() > 0) Function.Call(Hash._0x90B6DA738A9A25DA, 0f); 
                    Function.Call(Hash.SET_PED_DENSITY_MULTIPLIER_THIS_FRAME, 0f);
                    Function.Call(Hash.SET_SCENARIO_PED_DENSITY_MULTIPLIER_THIS_FRAME, 0f);
                }

                
                if (!_menuPool.AreAnyVisible)
                {
                    if ((DevSettingsFile.GetValue<bool>("GENERAL", "Hotkeys", true) && Game.IsControlPressed(2, GTA.Control.Sprint) && Game.IsControlPressed(2, GTA.Control.Context)) || WasCheatStringJustEntered("arsmenu"))
                    {
                        _arsMenu.Visible = true;
                    }
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
                        }
                    }
                }


                if (Racers.Any())
                {
                    if (_gameTimeNextInLine <= Game.GameTime)
                    {
                        int count = (int)Clamp(Racers.Count, 1, 6);
                        for (int i = 0; i < count; i++)
                        {
                            Racers.GetRange(_nextInLine, 1).FirstOrDefault()?.RunTimedCore();
                            _nextInLine++;
                            if (_nextInLine > Racers.Count - 1) _nextInLine = 0;
                            _gameTimeNextInLine = Game.GameTime + (10 / (Racers.Count / count));
                        }
                    }
                }

                
                foreach (Racer racer in Racers)
                {
                    racer.ProcessTick();
                    if ((racer.Lap > SettingsFile.GetValue("GENERAL_SETTINGS", "Laps", 5) || (IsPointToPoint && racer.Lap > 1)) && !LeaderboardFinish.Contains(racer))
                    {
                        if (racer.Car.CurrentBlip != null) racer.Car.CurrentBlip.Color = BlipColor.Green;

                        LeaderboardFinish.Add(racer);
                        racer.BaseBehavior = RacerBaseBehavior.FinishedRace;
                        if (IsPointToPoint) racer.BaseBehavior = RacerBaseBehavior.FinishedStandStill;
                        if (_raceTimedFinishMs == 0) _raceTimedFinishMs = Game.GameTime + (RaceSettingsFile.GetValue<int>("RACERS", "TimeoutSeconds", 30) * 1000);
                    }
                }


                
                if (_posUpdateTickMs < Game.GameTime)
                {
                    _posUpdateTickMs = Game.GameTime + 200;
                    try { GlobalTraffic = World.GetAllVehicles().ToList(); } catch (Exception) { GlobalTraffic = new List<Vehicle>(); }
                    foreach (Racer r in ARS.Racers) if (GlobalTraffic.Contains(r.Car)) GlobalTraffic.Remove(r.Car);

                    List<Racer> LapPos = new List<Racer>();
                    List<Racer> RPositions = Racers;
                    if (Racers.Any())
                    {
                        RPositions = RPositions.OrderBy(d => d.Lap).Reverse().ToList();
                        int L = RPositions[0].Lap;

                        int pos = 1;

                        while (pos < Racers.Count)
                        {
                            LapPos = RPositions.Where(vl => vl.Lap == L).ToList();
                            LapPos = LapPos.OrderBy(vl => vl.CurrentTrackPoint.Node).Reverse().ToList();
                            foreach (Racer r in LapPos)
                            {
                                r.RacePosition = pos;
                                pos++;
                            }
                            L--;
                        }
                    }
                    Racers = Racers.OrderBy(vl => vl.RacePosition).ToList();
                }

                
                if (LeaderboardFinish.Count > 0 && (LeaderboardFinish.Count == Racers.Count || (_raceTimedFinishMs != 0 && Game.GameTime > _raceTimedFinishMs)))
                {
                    if (LeaderboardFinish[0].Driver.IsPlayer) Game.Player.Money += RaceReward;
                    RaceStatus = RaceState.Finished;
                    CleanEverything();
                }

                if (_longTickMs < Game.GameTime)
                {
                    _longTickMs = Game.GameTime + 3000;

                    Vehicle playerVeh = Game.Player.Character.CurrentVehicle;
                    if (1==2 && CanWeUse(playerVeh) && !KnownVehicleModels.Contains(playerVeh.Model))
                    {
                        KnownVehicleModels.Add(playerVeh.Model);

                        CreateVehicle(playerVeh, true);
                        UI.Notify("New vehicle model detected, added to the list of known models: " + playerVeh.Model.ToString());
                    }

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
        public static float GetPercent(float current, float max)
        {
            return (current / max) * 100;
        }
        internal void SetSPLVisibility(bool state)
        {
            try { foreach (Prop p in World.GetAllProps()) if (p.Model == "prop_mp_max_out_lrg") if (state) p.Alpha = 255; else p.Alpha = 0; } catch (Exception) { }
        }
        void HandleTrackCreator()
        {

            
            int cool = -1; 

            if (RouteNodes.Count > 50)
            {
                if (_routeSection.Count > 0)
                {
                    Vector3 last = _routeSection[_routeSection.Count - 1];
                    if (RouteNodes[RouteNodes.Count - 1].DistanceTo(RouteNodes[0]) < 20f)
                    {
                        if (RouteNodes[0].DistanceTo(RouteNodes[RouteNodes.Count - 1]) < 2f)
                        {
                            cool = 1;
                            DrawLine(RouteNodes[RouteNodes.Count - 1], RouteNodes[0], Color.Green);
                        }
                        else
                        {
                            cool = 0;
                            DrawLine(RouteNodes[RouteNodes.Count - 1], RouteNodes[0], Color.Red);
                        }
                    }
                    if (cool < 1 && last.DistanceTo(RouteNodes[0]) < 20f)
                    {
                        World.DrawMarker(MarkerType.DebugSphere, last + new Vector3(0, 0, 0.2f), Vector3.Zero, Vector3.Zero, new Vector3(0.5f, 0.5f, 0.5f), Color.Blue);
                        World.DrawMarker(MarkerType.DebugSphere, RouteNodes[0] + new Vector3(0, 0, 0.2f), Vector3.Zero, Vector3.Zero, new Vector3(0.5f, 0.5f, 0.5f), Color.Black);
                        if (last.DistanceTo(RouteNodes[0]) > 2f)
                        {
                            DrawLine(last + new Vector3(0, 0, 0.2f), RouteNodes[0] + new Vector3(0, 0, 0.2f), Color.Red);
                        }
                        else
                        {
                            DrawLine(last + new Vector3(0, 0, 0.2f), RouteNodes[0] + new Vector3(0, 0, 0.2f), Color.Green);
                        }
                    }
                }
            }
            // Draw the pending section until the circuit closes.
            if (cool < 1) DrawSection(_routeSection, EditNodeHalfWidths);

            
            // Route editing is available only from freecam.
            if (_routeEditorActive && _freeCam.IsActive)
            {
                if (_pathWidth < 1) _pathWidth = 1;
                if (_bezierScale < 5f) _bezierScale = 5f;
                RaycastResult ray = World.Raycast(GameplayCamera.Position, GameplayCamera.Position + ((GameplayCamera.Direction.Normalized) * 100), IntersectOptions.Everything);

                
                if (RouteNodes.Count > 0)
                {
                    if (cool == -1) DisplayHelpTextThisFrame("Create the rest of the route. ~n~- Looped: ~b~Circuit~n~~w~- Open: ~b~Point to Point");
                    if (cool == 0) DisplayHelpTextThisFrame("Close the circuit near the ~b~Start Line.");
                    if (cool == 1) DisplayHelpTextThisFrame("~g~The circuit is closed.");


                    
                    if (Game.IsControlJustPressed(2, GTA.Control.NextWeapon))
                    {
                        if (!Game.IsControlPressed(2, GTA.Control.Sprint)) _bezierScale -= 5f; else _pathWidth--;
                    }
                    if (Game.IsControlJustPressed(2, GTA.Control.PrevWeapon))
                    {
                        if (!Game.IsControlPressed(2, GTA.Control.Sprint)) _bezierScale += 5f; else _pathWidth++;
                    }
                    // Aim removes one node; Sprint + Aim removes up to ten.
                    if (Game.IsControlJustPressed(2, GTA.Control.Aim))
                    {
                        if (RouteNodes.Count > 2)
                        {
                            if (Game.IsControlPressed(2, GTA.Control.Sprint))
                            {
                                int i = 0;
                                while (i < 10)
                                {
                                    if (RouteNodes.Count == 0) break;
                                    RouteNodes.RemoveAt(RouteNodes.Count - 1);
                                    i++;
                                }
                            }
                            else
                            {
                                RouteNodes.RemoveAt(RouteNodes.Count - 1);
                            }
                        }
                        else
                        {
                            RouteNodes.Clear();
                            return;
                        }
                    }

                    
                    if (Game.IsControlJustPressed(2, GTA.Control.Attack) && cool < 1)
                    {
                        for (int i = 1; i < _routeSection.Count; i++)
                        {
                            RouteNodes.Add(_routeSection[i]);
                        }
                    }
                }
                else 
                {
                    DisplayHelpTextThisFrame("Place the ~b~Start Line.");

                    if (Game.IsControlJustPressed(2, GTA.Control.NextWeapon)) _pathWidth--;
                    if (Game.IsControlJustPressed(2, GTA.Control.PrevWeapon)) _pathWidth++;

                }



                
                if (RouteNodes.Count > 1)
                {
                    if (ray.DitHitAnything && cool < 1)
                    {
                        
                        World.DrawMarker(MarkerType.DebugSphere, ray.HitCoords, Vector3.Zero, -Vector3.WorldDown, new Vector3(0.25f, 0.25f, 0.25f), Color.Blue);

                        Vector3 sStart = RouteNodes[RouteNodes.Count - 1];
                        Vector3 sDirection = (RouteNodes[RouteNodes.Count - 1] - RouteNodes[RouteNodes.Count - 2]).Normalized;
                        Vector3 sEnd = ray.HitCoords;
                        float sScale = sStart.DistanceTo(sEnd) * 0.5f;

                        List<Vector3> temporaryRouteNodes = GenerateBezier(sStart, sDirection, sEnd, sScale);

                        foreach (Vector3 p in temporaryRouteNodes)
                        {
                            World.DrawMarker(MarkerType.DebugSphere, p, Vector3.Zero, -Vector3.WorldDown, new Vector3(0.25f, 0.25f, 0.25f), Color.Blue);
                        }
                        _routeSection.Clear();
                        _routeSection.AddRange(temporaryRouteNodes);



                        EditNodeHalfWidths.Clear();
                        for (int d = 0; d < _routeSection.Count - 1; d++)
                        {
                            EditNodeHalfWidths.Add(d, _pathWidth);
                        }
                        if (_routeSection.Count > 4)
                        {
                            Vector3 aim = Vector3.Lerp(_routeSection[_routeSection.Count - 2] + new Vector3(0, 0, 0.5f), _routeSection[_routeSection.Count - 1] + new Vector3(0, 0, 0.5f), 10f);

                            DrawLine(_routeSection[_routeSection.Count - 2] + new Vector3(0, 0, 0.5f), aim, Color.Blue);
                        }
                    }
                }
                else
                {

                    World.DrawMarker(MarkerType.ChevronUpx3, ray.HitCoords, FreeCamRide.ForwardVector, new Vector3(-90, 0, 0), new Vector3(1, 1, 1), Color.Blue);
                    World.DrawMarker(MarkerType.ChevronUpx3, ray.HitCoords, FreeCamRide.ForwardVector, new Vector3(-90, 0, 0), new Vector3(1, 1, 1), Color.Blue);
                    DrawLine(ray.HitCoords, ray.HitCoords - (FreeCamRide.ForwardVector * 10), Color.Blue);

                    Vector3 right = ray.HitCoords + (FreeCamRide.RightVector * _pathWidth);
                    Vector3 left = ray.HitCoords - (FreeCamRide.RightVector * _pathWidth);

                    World.DrawMarker(MarkerType.UpsideDownCone, right + new Vector3(0, 0, 1), FreeCamRide.ForwardVector, new Vector3(0, 0, 0), new Vector3(1, 1, 1), Color.Blue);
                    World.DrawMarker(MarkerType.UpsideDownCone, left + new Vector3(0, 0, 1), FreeCamRide.ForwardVector, new Vector3(0, 0, 0), new Vector3(1, 1, 1), Color.Blue);
                    DrawLine(left + new Vector3(0, 0, 0.05f), right + new Vector3(0, 0, 0.05f), Color.Blue);

                    if (Game.IsControlJustPressed(2, GTA.Control.Attack) && ray.DitHitAnything)
                    {
                        Vector3 p = ray.HitCoords;
                        RouteNodes.Add(p);
                        p = ray.HitCoords - (FreeCamRide.ForwardVector * 1);
                        RouteNodes.Add(p);
                        _bezierStartAnchor = (ray.HitCoords - (FreeCamRide.ForwardVector * 6));

                    }
                }

                // Keep one half-width entry for every route segment.
                for (int i = 0; i < RouteNodes.Count - 1; i++)
                {
                    if (NodeHalfWidths.Count - 1 < RouteNodes.Count - 1)
                    {
                        if (!NodeHalfWidths.ContainsKey(i))
                        {
                            NodeHalfWidths.Add(i, _pathWidth);
                        }
                    }
                }

                while (NodeHalfWidths.Count - 1 > RouteNodes.Count - 1)
                {
                    NodeHalfWidths.Remove(NodeHalfWidths.Count - 1);
                }

            }
        }


        
        
        // Generate evenly spaced curve points from the route end to the raycast target.
        public static List<Vector3> GenerateBezier(Vector3 sStart, Vector3 sDirection, Vector3 sEnd, float sScale)
        {
            List<Vector3> points = new List<Vector3>();
            sScale = sStart.DistanceTo2D(sEnd) * Remap(Vector3.Angle((sEnd - sStart).Normalized, sDirection), 0f, 90f, 1f, 1.5f, true);


            Vector3 middlePoint = sStart + (sDirection * (sScale / 2));
            Vector3 directionStart = sStart - middlePoint;
            Vector3 directionEnd = ((sEnd) - middlePoint).Normalized * (sScale / 2f);

            
            

            float separationDist = 1f;

            float addition = (1 / directionEnd.DistanceTo(directionStart));

            float stepLerp = 0;
            int step = 0;
            float scaleAdjust = 0;
            while (stepLerp < 1.0f && step < 400)
            {
                step++;

                scaleAdjust = 0f;

                Vector3 currentPos = middlePoint + Bezier2(directionStart, directionEnd, stepLerp);
                Vector3 addPos = middlePoint + Bezier2(directionStart, directionEnd, stepLerp + addition);

                int tries = 0;
                while (currentPos.DistanceTo2D(addPos) < separationDist - 0.001f && tries < 200)
                {
                    tries++;
                    scaleAdjust += 0.001f;
                    addPos = middlePoint + Bezier2(directionStart, directionEnd, stepLerp + addition + scaleAdjust);
                }
                tries = 0;
                while (currentPos.DistanceTo2D(addPos) > separationDist + 0.001f && tries < 200)
                {
                    tries++;
                    scaleAdjust -= 0.001f;
                    addPos = middlePoint + Bezier2(directionStart, directionEnd, stepLerp + addition + scaleAdjust);
                }
                stepLerp += addition + scaleAdjust;

                
                
                if (!Game.IsControlPressed(2, GTA.Control.Sprint))
                {
                    RaycastResult toGround = World.Raycast(addPos + new Vector3(0, 0, 2f), addPos + (Vector3.WorldDown * 30f), IntersectOptions.Map);
                    if (toGround.DitHitAnything) addPos.Z = toGround.HitCoords.Z;
                }
                points.Add(addPos);

            }
            return points;
        }

        public static Vector3 QuadraticBezier(Vector3 a, Vector3 b, Vector3 c, float t)
        {
            Vector3 ab = Vector3.Lerp(a, b, t);
            Vector3 bc = Vector3.Lerp(b, c, t);
            Vector3 abc = Vector3.Lerp(ab, bc, t);
            return abc;
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
        internal int IntendedOpponents => _intendedOpponents;
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

            // Auto-call missing phases: if no track, instance one; if no grid, spawn one.
            // InstanceGrid already requires the track, so order is implicit.
            if (!_trackInstanced)
            {
                InstanceTrack();
                if (!_trackInstanced) return; // InstanceTrack failed (e.g. no tracks available).
            }
            if (!_gridInstanced)
            {
                InstanceGrid();
                if (!_gridInstanced) return; // InstanceGrid failed (e.g. no candidates).
            }

           

            Log(LogImportance.Info, "Adding player to grid");
            AddPlayerToGrid();
            if (Racers.Count == 0)
            {
                UI.Notify("~r~No vehicles found with those tags.");
                if (!_freeCam.IsActive) Function.Call(Hash.DO_SCREEN_FADE_IN, 500);
                return;
            }

            Game.MissionFlag = true;
            Log(LogImportance.Info, "Setting up");
            SetupRace(true, true);
            Log(LogImportance.Info, "Set up");

            if (!_freeCam.IsActive || Game.IsScreenFadedIn) Function.Call(Hash.DO_SCREEN_FADE_IN, 500);
            Game.SetControlNormal(2, GTA.Control.VehicleLookBehind, 1f);


            if (ARS.SettingsFile.GetValue("CATCHUP", "OnlyLastHalf", true)) ARS.CatchupPosition = (int)(ARS.Racers.Count / 2);

            // Race is now in the countdown/in-progress flow — clear setup flags.
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
            LeaderboardFinish.Clear();
            RaceStatus = RaceState.None;
            _gridInstanced = false;
        }


        bool AddPlayerToGrid()
        {
            Vehicle cv = Game.Player.Character.CurrentVehicle;
            if (CanWeUse(cv))
            {
                Racers.Add(new Racer(cv, Game.Player.Character));
                return true;
            }
            return false;
        }

        void LoadRace(string track, string disciplines, int gridSize)
        {
            Log(LogImportance.Info, "Loading race (" + track + ")");
            if (!KnownTracks.Contains(track))
            {
                UI.Notify("Track does not exist.");
                return;
            }

            
            TrackLoader.LoadTrack(this, TrackLoader.LoadTrackFile(track));

            
            Log(LogImportance.Info, "Loading grid of vehicles");

            if (disciplines == null)
            {
                List<VehicleHash> hashes = Enum.GetValues(typeof(VehicleHash)).Cast<VehicleHash>().ToList();

                Vehicle playerveh = Game.Player.Character.LastVehicle;
                float acc = Function.Call<float>(Hash.GET_VEHICLE_MODEL_ACCELERATION, playerveh.Model.Hash);
                float spd = Function.Call<float>(Hash._GET_VEHICLE_MAX_SPEED, playerveh.Model.Hash);

                hashes.RemoveAll(h => Function.Call<float>(Hash.GET_VEHICLE_MODEL_ACCELERATION, (int)h) < acc - 0.075f);
                hashes.RemoveAll(h => Function.Call<float>(Hash.GET_VEHICLE_MODEL_ACCELERATION, (int)h) > acc + 0.075f);

                hashes.RemoveAll(h => Function.Call<float>(Hash._GET_VEHICLE_MAX_SPEED, (int)h) > spd + ARS.MphToMps(20));
                hashes.RemoveAll(h => Function.Call<float>(Hash._GET_VEHICLE_MAX_SPEED, (int)h) < spd - ARS.MphToMps(20));

                hashes.OrderByDescending(h => Function.Call<float>(Hash.GET_VEHICLE_MODEL_ACCELERATION, (int)h));




                List<VehicleHash> final = new List<VehicleHash>();

                foreach (VehicleHash hash in hashes)
                {
                    Model m = new Model(hash);

                    
                    if (Math.Abs(m.GetDimensions().Length() - playerveh.Model.GetDimensions().Length()) < 5f &&
                        (m.IsCar && playerveh.Model.IsCar) ||
                        (m.IsQuadbike && playerveh.Model.IsQuadbike) ||
                        (m.IsBike && playerveh.Model.IsBike) ||
                        (m.IsBicycle && playerveh.Model.IsBicycle))
                    {
                        final.Add(hash);
                    }
                }


                while (final.Count > gridSize) final.RemoveAt(GetRandomInt(0, final.Count - 1));

                foreach (VehicleHash h in final)
                {
                    Model m = new Model(h);

                    Vehicle v = World.CreateVehicle(m, RouteNodes[Racers.Count * 5]);
                    Ped p = v.CreateRandomPedOnSeat(VehicleSeat.Driver);
                    Racer r = new Racer(v, p);

                    Racers.Add(r);
                    

                }

            }
            else LoadGrid(disciplines, gridSize);
            StartRace();
        }

        

















        void PlaceCars()
        {
            GridSort sort = GridSort.Power;
            string setting = RaceSettingsFile.GetValue<string>("RACERS", "GridSorting", "Power");
            if (!Enum.TryParse(setting, true, out sort)) sort = GridSort.Power;
            GridBuilder.Place(Racers, GridPositions, RouteNodes, IsPointToPoint, sort);
        }


        public void SetupRace(bool placecars, bool tunecars)
        {
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
                int r = ((RouteNodes.Count / 3) * SettingsFile.GetValue("GENERAL_SETTINGS", "Laps", 5)) + (Racers.Count * 100) + (int)Math.Round(Function.Call<float>(Hash.GET_VEHICLE_ACCELERATION, mostPower.Car) * 400, 0);
                RaceReward = (int)(Math.Round((float)r / 100)) * 100;
            }

            if (tunecars)
            {
                Log(LogImportance.Info, "Tuning cars");
                foreach (Racer r in Racers)
                {
                    if (!Game.Player.Character.IsInVehicle(r.Car) && r.Car.GetMod(VehicleMod.Engine) == -1)
                    {
                        switch (RaceSettingsFile.GetValue<int>("RACERS", "AITuningLevel", 1))
                        {
                            case 0: continue;
                            case 1: ARS.RandomTuning(r.Car, true, true, true, false, false); break;
                            case 2: ARS.RandomTuning(r.Car, true, true, true, true, false); break;
                            case 3: ARS.RandomTuning(r.Car, true, true, true, true, false); r.Car.EnginePowerMultiplier = GetRandomInt(1, 5) * 10; break;
                        }
                    }
                }
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
        public static int GetSurfaceHash(Vector3 start, Vector3 end)
        {
            Vector3 pos = start;
            Vector3 endpos = end;

            int shape = Function.Call<int>(Hash._0x28579D1B8F8AAC80, pos.X, pos.Y, pos.Z, endpos.X, endpos.Y, endpos.Z, 0.3f, (int)IntersectOptions.Map, Game.Player.Character, 7);

            OutputArgument didhit = new OutputArgument();
            OutputArgument hitpos = new OutputArgument();
            OutputArgument snormal = new OutputArgument();
            OutputArgument materialhash = new OutputArgument();

            OutputArgument entity = new OutputArgument();

            Function.Call(Hash._0x65287525D951F6BE, shape, didhit, hitpos, snormal, materialhash, entity);


            return materialhash.GetResult<int>();
        }
        



        static public List<Vector3> GridPositions = new List<Vector3>();
        static public bool IsPointToPoint = false;
        public bool PlayerOrCameraNearPos(Vector3 pos, float dist)
        {
            if (_freeCam.IsActive) return Game.Player.Character.Position.DistanceTo(pos) < dist;
            else return World.RenderingCamera.Position.DistanceTo(pos) < dist;


        }

        public void DrawRouteNodes(List<Vector3> nodes, Dictionary<int, float> widedict, int fidelity)
        {
            if (nodes.Count == 0) return;
            int closestnode = ClosestNodeToPlace(Game.Player.Character.Position, nodes);
            Vector3 oldpos = Vector3.Zero;

            int start = closestnode - 50;
            int end = closestnode + 50;
            int countmax = 1;
            int count = 0;
            int dd = 0;
            Vector3 pos = Vector3.Zero;
            Vector3 lastline = Vector3.Zero;
            if (start < 0) start = 0;
            if (end > nodes.Count - 1) end = nodes.Count - 1;
            dd = start;


            if (_routeEditorActive) World.DrawMarker(MarkerType.CheckeredFlagRect, nodes[0] + new Vector3(0, 0, 3f), (nodes[1] - nodes[0]).Normalized, new Vector3(0, 0, 0), new Vector3(5f, 5f, 5f), Color.White);

            for (int ph = start; ph < end; ph += 1)
            {

                    pos = nodes[ph];
                    float w = 0f;
                    float oldw = 0f;
                    if (widedict != null)
                    {
                        if (widedict.ContainsKey(dd)) w = widedict[dd];
                        if (widedict.ContainsKey(dd - 1)) oldw = widedict[dd - 1]; else oldw = w;
                    }

                    if (oldpos == Vector3.Zero) oldpos = nodes[nodes.Count - 1];


                    if (oldpos != Vector3.Zero && PlayerOrCameraNearPos(nodes[ph], 120))
                    {

                            Vector3 rWidepos = GetPerpendicular(pos, oldpos, w, true);
                            Vector3 lWidepos = GetPerpendicular(pos, oldpos, w, false);

                            Vector3 oldrWidepos = GetPerpendicular(pos, oldpos, oldw, true) - (pos - oldpos);
                            Vector3 oldlWidepos = GetPerpendicular(pos, oldpos, oldw, false) - (pos - oldpos);




                            Color col = Color.Green;



                            

                            if (w != 0f)
                            {

                                if (_routeEditorActive)
                                {

                                    if (ph == end - 1)
                                    {

                                        
                                        
                                        

                                    }
                                    else
                                        if (pos != nodes[0] && ph % 5 == 0)
                                        {


                                            World.DrawMarker(MarkerType.DebugSphere, lWidepos, new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), Color.Blue);
                                            World.DrawMarker(MarkerType.DebugSphere, rWidepos, new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), Color.Blue);
                                                                                                                                                                                      



                                        }
                                }

                                col.ToArgb();

                                Color chevcolor = Color.FromArgb(50, col);

                                


                            }
                        }

                oldpos = pos;

                dd++;
            }
        }


        public void DrawSection(List<Vector3> nodes, Dictionary<int, float> widedict)
        {
            if (nodes.Count == 0) return;
            int closestnode = ClosestNodeToPlace(Game.Player.Character.Position, nodes);
            Vector3 oldpos = Vector3.Zero;

            int start = closestnode - 100;
            int end = closestnode + 100;
            int countmax = 1;
            int count = 0;
            int dd = 0;
            Vector3 pos = Vector3.Zero;
            Vector3 lastline = Vector3.Zero;
            if (start < 0) start = 0;
            if (end > nodes.Count - 1) end = nodes.Count - 1;
            dd = start;
            for (int ph = start; ph < end; ph += 1)
            {

                    pos = nodes[ph];
                    float w = 0f;
                    float oldw = 0f;
                    if (widedict != null)
                    {
                        if (widedict.ContainsKey(dd)) w = widedict[dd];
                        if (widedict.ContainsKey(dd - 1)) oldw = widedict[dd - 1]; else oldw = w;
                    }

                    if (oldpos == Vector3.Zero) oldpos = nodes[nodes.Count - 1];


                    if (oldpos != Vector3.Zero && PlayerOrCameraNearPos(nodes[ph], 120))
                    {

                            Vector3 rWidepos = GetPerpendicular(pos, oldpos, w, true);
                            Vector3 lWidepos = GetPerpendicular(pos, oldpos, w, false);

                            Vector3 oldrWidepos = GetPerpendicular(pos, oldpos, oldw, true) - (pos - oldpos);
                            Vector3 oldlWidepos = GetPerpendicular(pos, oldpos, oldw, false) - (pos - oldpos);


                            Color col = Color.Green;


                            


                            if (w != 0f)
                            {

                                if (_routeEditorActive)
                                {
                                    

                                    if (ph == end - 1)
                                    {

                                        DrawLine(lWidepos + new Vector3(0, 0, 0.5f), rWidepos + new Vector3(0, 0, 0.5f), Color.Blue);
                                        World.DrawMarker(MarkerType.DebugSphere, lWidepos + new Vector3(0, 0, 0.5f), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0.3f, 0.3f, 0.3f), Color.Blue);
                                        World.DrawMarker(MarkerType.DebugSphere, rWidepos + new Vector3(0, 0, 0.5f), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0.3f, 0.3f, 0.3f), Color.Blue);

                                    }
                                    else
                                        if (pos != nodes[0] && ph % 6 == 0)
                                        {


                                            World.DrawMarker(MarkerType.DebugSphere, lWidepos + new Vector3(0f, 0f, 0.5f), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), Color.Green);
                                            World.DrawMarker(MarkerType.DebugSphere, rWidepos + new Vector3(0f, 0f, 0.5f), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0.2f, 0.2f, 0.2f), Color.Green);
                                            DrawLine(lWidepos + new Vector3(0f, 0f, 0.5f), rWidepos + new Vector3(0f, 0f, 0.5f), Color.Green);



                                        }
                                }
                            }
                        }

                oldpos = pos;

                dd++;
            }
        }
        Vector3 GetPerpendicular(Vector3 a, Vector3 b, float length, bool clockwise)
        {
            Vector3 ab = (b - a).Normalized;
            Vector3 abCw = Vector3.Zero;
            if (clockwise)
            {
                abCw.X = -ab.Y;
                abCw.Y = ab.X;
            }
            else
            {
                abCw.X = ab.Y;
                abCw.Y = -ab.X;
            }
            return a + abCw * length;
        }
        


        public bool ListenMode = false;
        public void HandleCheats()
        {



            if (WasCheatStringJustEntered("combo"))
            {
                Vehicle v = Game.Player.Character.CurrentVehicle;
                string t = "";
                t += (int)v.PrimaryColor + "~n~";
                t += (int)v.SecondaryColor + "~n~";
                t += (int)v.PearlescentColor + "~n~";
                t += (int)v.RimColor + "~n~";
                t += (int)v.TrimColor + "~n~";
                t += (int)v.DashboardColor + "~n~";

                UI.Notify(t);
            }

            if (WasCheatStringJustEntered("arson"))
            {
                StartLoadScript();
            }
            if (WasCheatStringJustEntered("arsoff"))
            {
                DisplayHelpTextTimed("ARS is now disabled. You can re-enable it with the 'arson' cheat.", 3000);
                _loaded = false;
            }

            if (WasCheatStringJustEntered("arsreload"))
            {
                
                
                FillKnownDisciplines();

                FillKnownTracks();
            }
            if (WasCheatStringJustEntered("arscarlisten"))
            {
                ListenMode = !ListenMode;
                if (ListenMode) UI.Notify("~g~Listen mode is on."); else UI.Notify("~y~Listen mode disabled.");
            }
            if (WasCheatStringJustEntered("arsupdroute"))
            {
                
                UpdateRoute(true, true, true);

            }
            if (WasCheatStringJustEntered("arsbuildcarlist"))
            {
                UI.Notify("~b~[ARS]:~w~ Generating vehicle files for all SHVDN known vehicles in the game.");

                foreach (VehicleHash hash in Enum.GetValues(typeof(VehicleHash)).Cast<VehicleHash>())
                {
                    Model m = new Model(hash);
                    if (m.IsBike || m.IsQuadbike || m.IsBicycle || m.IsCar) CreateVehicleFromHash(hash);
                    Script.Yield();
                }
            }

            if (WasCheatStringJustEntered("arsbuilddumpcarlist"))
            {
                UI.Notify("~b~[ARS]:~w~ Generating vehicle files from modeldump.txt.");

                string dumpFilePath = ScriptsFolder + @"\modeldump.txt";
                if (File.Exists(dumpFilePath))
                {
                    string content = File.ReadAllText(dumpFilePath);
                    string[] modelNames = content.Split(new[] { ',', '\n', '\r', ' ' }, StringSplitOptions.RemoveEmptyEntries);

                    foreach (string modelName in modelNames)
                    {
                        string trimmedName = modelName.Trim().ToLowerInvariant();
                        if (!string.IsNullOrEmpty(trimmedName))
                        {
                            CreateVehicleFromName(trimmedName);
                            Script.Yield();
                        }
                    }

                    UI.Notify("~b~[ARS]:~w~ Processed " + modelNames.Length + " models from modeldump.txt.");
                }
                else
                {
                    UI.Notify("~r~[ARS]:~w~ modeldump.txt not found in " + ScriptsFolder + "\\");
                }
            }


            if (WasCheatStringJustEntered("arssettings"))
            {
                UI.Notify("Re loading settings.");
                SettingsFile = null;
                DevSettingsFile = null;
                RaceSettingsFile = null;
                LoadSettings();
            }


            if (WasCheatStringJustEntered("arssavedriver")) CreateDriver(Game.Player.Character);
            if (WasCheatStringJustEntered("arssavecar"))
            {
                CreateVehicle(Game.Player.Character.CurrentVehicle);
            }

            if (WasCheatStringJustEntered("arsclean"))
            {
                CleanEverything();
            }
        }
        

        public static void FindCustomProps()
        {
            if (RouteNodes.Count == 0) return;
            CustomProps.Clear();
            foreach (Prop propchecked in World.GetAllProps().ToList())
            {
                if (propchecked.IsPersistent && !AutoGeneratedProps.Contains(propchecked) && !TrackLimits.Contains(propchecked) && FreeCamRide != propchecked)
                {
                    float maxDist = 0f;

                    Vector3 d = RouteNodes.OrderBy(v => propchecked.Position.DistanceTo(v)).ToList().First();
                    for (int i = 0; i < RouteNodes.Count; i++) if (d == RouteNodes[i]) if (NodeHalfWidths.ContainsKey(i)) maxDist = NodeHalfWidths[i] + 200f;
                    if (maxDist > propchecked.Position.DistanceTo(d))
                    {
                        CustomProps.Add(propchecked);
                    }
                }
            }
        }

        public void UpdateRoute(bool path, bool raceline, bool props)
        {
            Log(LogImportance.Info, "-- UPDATING CURRENT ROUTE --");
            if (path || raceline)
            {
                UI.Notify("Updating path and raceline.");
                XmlNode route = CurrentFile.SelectSingleNode("//Route");
                Log(LogImportance.Info, "Removing original _routeNodes.");
                route.RemoveAll();

                XmlElement p = null;
                XmlElement info = null;
                int i = 0;
                int W = 5;
                Log(LogImportance.Info, "Creating new path info from the current loaded _routeNodes.");

                foreach (Vector3 v in RouteNodes)
                {
                    p = CurrentFile.CreateElement("Point");
                    

                    info = CurrentFile.CreateElement("X");
                    info.InnerText = Math.Round(v.X, 2).ToString();
                    info.InnerText = info.InnerText.Replace(",", ".");
                    p.AppendChild(info);

                    info = CurrentFile.CreateElement("Y");
                    info.InnerText = Math.Round(v.Y, 2).ToString();
                    info.InnerText = info.InnerText.Replace(",", ".");
                    p.AppendChild(info);

                    info = CurrentFile.CreateElement("Z");
                    info.InnerText = Math.Round(v.Z, 2).ToString();
                    info.InnerText = info.InnerText.Replace(",", ".");
                    p.AppendChild(info);

                    info = CurrentFile.CreateElement("Wide");
                    int wide = W;
                    if (NodeHalfWidths.ContainsKey(i)) int.TryParse(NodeHalfWidths[i].ToString(), out _pathWidth);
                    if (wide == 0)
                    {
                        wide = 5;
                        Log(LogImportance.Error, "Node nº" + i + "wide setting WAS 0, setting to default wide (5)");

                    }
                    else
                    {
                        W = _pathWidth;
                    }

                    info.InnerText = wide.ToString();
                    p.AppendChild(info);

                    route.AppendChild(p);
                    i++;
                }
                CurrentFile.SelectSingleNode("Data").AppendChild(route);

            }

            if (props)
            {
                UI.Notify("Updating objects.");
                XmlElement info = null;
                Log(LogImportance.Info, "Removing all object references.");

                foreach (XmlNode o in CurrentFile.SelectNodes("//Objects")) o.RemoveAll();

                XmlNode objects = CurrentFile.SelectSingleNode("//Objects");
                if (objects == null) objects = CurrentFile.CreateElement("Objects");

                objects.RemoveAll();

                CustomProps.Clear();
                FindCustomProps();

                Log(LogImportance.Info, "Filling objects list from the persistent props in the world. (Near the track).");

                foreach (Prop prop in CustomProps)
                {
                    XmlElement p = CurrentFile.CreateElement("Prop");
                    info = CurrentFile.CreateElement("Model");
                    info.InnerText = prop.Model.Hash.ToString();
                    info.InnerText = info.InnerText.Replace(",", ".");
                    p.AppendChild(info);

                    

                    info = CurrentFile.CreateElement("TextureVariation");
                    info.InnerText = Function.Call<int>((Hash)0xE84EB93729C5F36A, prop).ToString();
                    p.AppendChild(info);


                    info = CurrentFile.CreateElement("X");
                    info.InnerText = Math.Round(prop.Position.X, 2).ToString();
                    info.InnerText = info.InnerText.Replace(",", ".");
                    p.AppendChild(info);

                    info = CurrentFile.CreateElement("Y");
                    info.InnerText = Math.Round(prop.Position.Y, 2).ToString();
                    info.InnerText = info.InnerText.Replace(",", ".");
                    p.AppendChild(info);

                    info = CurrentFile.CreateElement("Z");
                    info.InnerText = Math.Round(prop.Position.Z, 2).ToString();
                    info.InnerText = info.InnerText.Replace(",", ".");
                    p.AppendChild(info);

                    info = CurrentFile.CreateElement("RotX");
                    info.InnerText = Math.Round(prop.Rotation.X, 2).ToString();
                    info.InnerText = info.InnerText.Replace(",", ".");
                    p.AppendChild(info);

                    info = CurrentFile.CreateElement("RotY");
                    info.InnerText = Math.Round(prop.Rotation.Y, 2).ToString();
                    info.InnerText = info.InnerText.Replace(",", ".");
                    p.AppendChild(info);

                    info = CurrentFile.CreateElement("RotZ");
                    info.InnerText = Math.Round(prop.Rotation.Z, 2).ToString();
                    info.InnerText = info.InnerText.Replace(",", ".");
                    p.AppendChild(info);

                    info = CurrentFile.CreateElement("IsDynamic");
                    info.InnerText = (!prop.FreezePosition).ToString(); 


                    info.InnerText = info.InnerText.Replace(",", ".");
                    p.AppendChild(info);

                    objects.AppendChild(p);

                    
                }
                CurrentFile.SelectSingleNode("Data").AppendChild(objects);
            }

            XmlNode getname = CurrentFile.SelectSingleNode("//Name");
            if (getname != null)
            {
                CurrentFile.Save(ScriptsFolder + @"\Tracks\" + CurrentFile.SelectSingleNode("//Name").InnerText + ".xml");

            }
            else
            {
                CurrentFile.Save(ScriptsFolder + @"\Tracks\" + Game.GetUserInput(200) + ".xml");


            }
        }
        public void SaveRoute(string filename)
        {
            if (filename == null || filename == "")
            {
                filename = World.GetStreetName(RouteNodes[0]);
            }

            if (File.Exists(ScriptsFolder + @"\Tracks\" + filename + ".xml"))
            {
                DateTime today = DateTime.Now;
                filename += " (" + today.Year + today.Month + today.Day + today.Hour + today.Minute + today.Second + ")";
            }


            XmlDocument document = new XmlDocument();



            XmlElement element = document.CreateElement("Data");
            document.AppendChild(element);

            XmlComment c = document.CreateComment("comment");

            c.InnerText = " Flares='204255051' would put flares at the startline.\n The value is actually three RGB values from 000 to 255.\n 255255255 would be white. ";
            element.AppendChild(c);

            XmlElement trackside = document.CreateElement("Trackside");
            XmlElement t = document.CreateElement("Model");
            t.InnerText = ARS.DevSettingsFile.GetValue("CREATOR_DEFAULTS", "TracksideModel", "prop_wheel_tyre");
            trackside.AppendChild(t);

            t = document.CreateElement("Frecuency");
            t.InnerText = ARS.DevSettingsFile.GetValue("CREATOR_DEFAULTS", "TracksideModelFrecuency", "10");
            trackside.AppendChild(t);


            XmlAttribute isFrozen = document.CreateAttribute("Frozen");
            isFrozen.InnerText = "true";
            trackside.Attributes.Append(isFrozen);

            XmlAttribute flares = document.CreateAttribute("Flares");
            flares.InnerText = "false";
            trackside.Attributes.Append(flares);

            element.AppendChild(trackside);



            
            XmlElement route = document.CreateElement("Route");

            XmlElement objects = document.CreateElement("Objects");

            XmlElement name = document.CreateElement("Name");
            name.InnerText = filename;
            element.AppendChild(name);




            UI.ShowSubtitle("Write any tags you want for this track, separated by spaces. Example: rally long");
            XmlElement tags = document.CreateElement("Tags");
            XmlElement tag = document.CreateElement("Tag");
            tag.InnerText = World.GetStreetName(RouteNodes[0]);
            tags.AppendChild(tag);
            tag = document.CreateElement("Tag");
            tag.InnerText = World.GetZoneName(RouteNodes[0]).Replace(" ", "");
            tags.AppendChild(tag);

            string userTags = Game.GetUserInput(32);
            if (userTags != "")
            {
                foreach (string s in userTags.Split(' '))
                {
                    tag = document.CreateElement("Tag");
                    tag.InnerText = s;
                    tags.AppendChild(tag);
                }
            }


            element.AppendChild(tags);




            XmlElement p = null;
            XmlElement info = null;
            int i = 0;
            int W = 5;

            foreach (Vector3 v in RouteNodes)
            {
                p = document.CreateElement("Point");
                

                info = document.CreateElement("X");
                info.InnerText = Math.Round(v.X, 2).ToString();
                info.InnerText = info.InnerText.Replace(",", ".");
                p.AppendChild(info);

                info = document.CreateElement("Y");
                info.InnerText = Math.Round(v.Y, 2).ToString();
                info.InnerText = info.InnerText.Replace(",", ".");
                p.AppendChild(info);

                info = document.CreateElement("Z");
                info.InnerText = Math.Round(v.Z, 2).ToString();
                info.InnerText = info.InnerText.Replace(",", ".");
                p.AppendChild(info);

                info = document.CreateElement("Wide");
                if (NodeHalfWidths.ContainsKey(i)) int.TryParse(NodeHalfWidths[i].ToString(), out W);
                info.InnerText = W.ToString();
                p.AppendChild(info);

                route.AppendChild(p);
                i++;
            }


            CustomProps.Clear();


            foreach (Prop propchecked in World.GetAllProps().ToList())
            {
                if (propchecked.IsPersistent && !AutoGeneratedProps.Contains(propchecked) && !TrackLimits.Contains(propchecked))
                {

                    float d = RouteNodes.OrderBy(v => propchecked.Position.DistanceTo(v)).ToList().First().DistanceTo(propchecked.Position);
                    if (d < 10) CustomProps.Add(propchecked);
                }
            }

            foreach (Prop prop in CustomProps)
            {
                p = document.CreateElement("Prop");
                info = document.CreateElement("Model");
                info.InnerText = prop.Model.Hash.ToString();
                info.InnerText = info.InnerText.Replace(",", ".");
                p.AppendChild(info);

                

                info = document.CreateElement("TextureVariation");
                info.InnerText = Function.Call<int>((Hash)0xE84EB93729C5F36A, prop).ToString();
                ;
                p.AppendChild(info);


                info = document.CreateElement("X");
                info.InnerText = Math.Round(prop.Position.X, 2).ToString();
                info.InnerText = info.InnerText.Replace(",", ".");
                p.AppendChild(info);

                info = document.CreateElement("Y");
                info.InnerText = Math.Round(prop.Position.Y, 2).ToString();
                info.InnerText = info.InnerText.Replace(",", ".");
                p.AppendChild(info);

                info = document.CreateElement("Z");
                info.InnerText = Math.Round(prop.Position.Z, 2).ToString();
                info.InnerText = info.InnerText.Replace(",", ".");
                p.AppendChild(info);

                info = document.CreateElement("RotX");
                info.InnerText = Math.Round(prop.Rotation.X, 2).ToString();
                info.InnerText = info.InnerText.Replace(",", ".");
                p.AppendChild(info);

                info = document.CreateElement("RotY");
                info.InnerText = Math.Round(prop.Rotation.Y, 2).ToString();
                info.InnerText = info.InnerText.Replace(",", ".");
                p.AppendChild(info);

                info = document.CreateElement("RotZ");
                info.InnerText = Math.Round(prop.Rotation.Z, 2).ToString();
                info.InnerText = info.InnerText.Replace(",", ".");
                p.AppendChild(info);

                info = document.CreateElement("IsDynamic");
                info.InnerText = (!prop.FreezePosition).ToString(); 


                info.InnerText = info.InnerText.Replace(",", ".");
                p.AppendChild(info);

                objects.AppendChild(p);

            }
            document.SelectSingleNode("Data").AppendChild(route);
            document.SelectSingleNode("Data").AppendChild(objects);


            document.Save(ScriptsFolder + @"\Tracks\" + filename + ".xml");

            DisplayHelpTextTimed("Adding to track dictionary...", 1000);
            FillKnownTracks();
            DisplayHelpTextTimed("~g~Done.", 2000);
        }

        public void AttachFlare(Prop p, Color color)
        {
            int d = 0;
            while (!Function.Call<bool>(Hash.HAS_NAMED_PTFX_ASSET_LOADED, "scr_apartment_mp") && d < 2000)
            {
                Function.Call(Hash.REQUEST_NAMED_PTFX_ASSET, "scr_apartment_mp");
                d++;
                Script.Wait(0);
            }
            if (Function.Call<bool>(Hash.HAS_NAMED_PTFX_ASSET_LOADED, "scr_apartment_mp"))
            {
                Function.Call(Hash._SET_PTFX_ASSET_NEXT_CALL, "scr_apartment_mp");
                if (CanWeUse(p))
                {
                    int fx = Function.Call<int>(Hash.START_PARTICLE_FX_LOOPED_ON_ENTITY, "scr_finders_package_flare", p, 0f, 0f, 0.1f, 0f, 0f, 0f, 1f, true, true, true);
                    Function.Call(Hash.SET_PARTICLE_FX_LOOPED_ALPHA, fx, 1f);
                    Function.Call(Hash.SET_PARTICLE_FX_LOOPED_COLOUR, fx, color.R / 255f, color.G / 255f, color.B / 255f, 1f, true); 
                    _flareFx.Add(fx);
                }
            }
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
        // Walk the track in 30m chunks. Pick the tightest node in each chunk as the apex.
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

        public static TrackPoint FindNearestTrackPoint(Vector3 position, int referenceNode = -1)
        {
            if (ARS.TrackPoints.Count == 0) return null;
            int lastNode = ARS.TrackPoints.Count - 1;
            int center = referenceNode >= 0 && referenceNode <= lastNode ? referenceNode : 0;
            int first = Math.Max(center - 10, 0);
            int last = Math.Min(center + 10, lastNode);
            TrackPoint nearest = ARS.TrackPoints[first];
            float best = nearest.Position.DistanceTo2D(position);
            for (int i = first + 1; i <= last; i++)
            {
                float dist = ARS.TrackPoints[i].Position.DistanceTo2D(position);
                if (dist < best)
                {
                    best = dist;
                    nearest = ARS.TrackPoints[i];
                }
            }
            return nearest;
        }

        public static float Circumradius3D(Vector3 a, Vector3 b, Vector3 midpoint)
        {
            return TrackLoader.Circumradius3D(a, b, midpoint);
        }

        public static float Circumradius2D(Vector2 a, Vector2 b, Vector2 midPoint)
        {
            return TrackLoader.Circumradius2D(a, b, midPoint);
        }

        public static float GetPreciseRadius(TrackPoint trackPoint, int span)
        {
            if (trackPoint == null || TrackPoints == null || TrackPoints.Count < 3) return 999f;

            int safeSpan = Math.Max(1, span);
            int node = (int)Clamp(trackPoint.Node, 0, TrackPoints.Count - 1);
            int minSafeNode = safeSpan;
            int maxSafeNode = TrackPoints.Count - 1 - safeSpan;
            if (node < minSafeNode || node > maxSafeNode) return trackPoint.PreciseCurveRadius;

            Vector3 prev = TrackPoints[node - safeSpan].Position;
            Vector3 next = TrackPoints[node + safeSpan].Position;
            Vector3 mid = TrackPoints[node].Position;

            float r = Circumradius3D(prev, next, mid);
            if (float.IsNaN(r) || float.IsInfinity(r) || r <= 0f) return trackPoint.PreciseCurveRadius;
            return r;
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

        // Exponential hill-grip loss, scaled by vehicle gravity.
        const float HillGripKPerDegree = 0.693147f / 15f; // ln(0.5)/15  ->  15 degrees halves grip
        const float HillGripMin = 0.5f;                  // never remove more than 50% of grip on a hill
        public static float HillGripFactorFromPitchAngle(float pitchDegrees, Racer r)
        {
            float gravityRatio = (r != null && r.Handling != null) ? (r.Handling.Gravity / 9.8f) : 1f;
            float effectiveDeg = Math.Abs(pitchDegrees) * Math.Max(gravityRatio, 0.1f);
            float factor = (float)Math.Exp(-HillGripKPerDegree * effectiveDeg);
            if (float.IsNaN(factor) || float.IsInfinity(factor)) return 1f;
            return Clamp(factor, HillGripMin, 1f);
        }

        public static float CornerApexSpeed(CornerPoint c, Racer r)
        {
            
            float radius = c.SupposedRadius;
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
            return ARS.Remap(Vector3.Angle(e.ForwardVector, e.Velocity.Normalized), 0f, 90f, e.Model.GetDimensions().X, e.Model.GetDimensions().Y, true);
        }

        
        static public unsafe ulong GetWheelsPtr(Vehicle handle)
        {
            GameVersion gameVersion = Game.Version;
            var address = (ulong)handle.MemoryAddress;
            if (WheelsPtr == 0x0)
            {
                IntPtr addr = (IntPtr)FindPattern("\x3B\xB7\x48\x0B\x00\x00\x7D\x0D", "xx????xx");

                if (addr != null)
                {
                    WheelsPtr = *(uint*)(addr + 2) - 8;
                    Log(LogImportance.Info, "[MEMORY] Learned the handling offset: " + WheelsPtr);

                }
            }
            return *((ulong*)(address + WheelsPtr));
        }

        static public unsafe int GetNumWheels(Vehicle handle)
        {

            if (NumWheelsOffset == 0x0)
            {
                IntPtr addr = (IntPtr)FindPattern("\x3B\xB7\x48\x0B\x00\x00\x7D\x0D", "xx????xx");

                if (addr != null)
                {
                    NumWheelsOffset = *(uint*)(addr + 2);
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
            for (int i = 0; i < numWheels; i++)
            {
                var wheelAddr = *((ulong*)(wheelPtr + 0x008 * (ulong)i));
                wheelPtrs.Add(wheelAddr);
            }
            return wheelPtrs;
        }


        

        static public unsafe List<float> WheelGripMultipliers(Vehicle handle)
        {
            List<ulong> wheelPtrs = GetWheelPtrs(handle);
            ulong offset = 0x198;
            List<float> angle = new List<float>();
            foreach (var wheel in wheelPtrs)
            {
                float pos = (float)Math.Round(*((float*)(wheel + offset)), 2);
                angle.Add(pos);
            }
            return angle;
        }

        // A wheel is grounded when its grip multiplier is positive.
        static public unsafe List<bool> WheelsOnGround(Vehicle handle)
        {
            List<ulong> wheelPtrs = GetWheelPtrs(handle);
            ulong offset = 0x198;
            List<bool> onGround = new List<bool>();
            foreach (var wheel in wheelPtrs)
            {
                float grip = *((float*)(wheel + offset));
                onGround.Add(grip > 0.01f);
            }
            return onGround;
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

        // Per-wheel slip (offset 0x174, same data TCS uses). A lifted wheel reads 0.00 slip, so
        // counting wheels with ~0 slip detects ground contact for the stability factor.
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

        

        

        


        

        

        
        

        public static Vector3 Bezier2(Vector3 Start, Vector3 End, float t)
        {
            return (((1 - t) * (1 - t)) * Start) + (2 * t * (1 - t) * Vector3.Zero) + ((t * t) * End);
        }

        public static int ClosestNodeToPlace(Vector3 v, List<Vector3> PathRoute)
        {
            Vector3 closest = PathRoute.OrderBy(p => p.DistanceTo(v)).ToList()[0];
            for (int i = 0; i < PathRoute.Count - 1; i++) if (PathRoute[i] == closest) return i;
            return 0;
        }
        
        // Legacy live corner scan; superseded by the apex-table pipeline.
        const int ChunkScanNodesPerCore = 10;
        const float CornerLimitRadius = 300f;
        const int MaxCornerRegionWalk = 300;

        public static void FindNextCorner(Racer r)
        {
            // Keep the current corner active through its apex.
            if (r.Brain.Corner != null && r.CurrentTrackPoint.Node <= r.Brain.Corner.Point.Node)
                return;

            int count = TrackPoints.Count;
            if (count < 20)
            {
                r.Brain.Corner = null;
                return;
            }

            int cur = r.CurrentTrackPoint.Node;
            // After an apex, jump the scan head 5s past it to skip the exit zone.
            if (r.CornerScanNode < 0 || r.CornerScanNode <= cur)
            {
                if (r.Brain.Corner != null)
                    r.CornerScanNode = r.Brain.Corner.Point.Node + (int)(r.Brain.Corner.Speed * 5f) - 1;
                else
                    r.CornerScanNode = cur + 1;
            }

            // Check the next chunk of nodes forward from the scan head.
            int chunkStart = r.CornerScanNode;
            for (int i = 0; i < ChunkScanNodesPerCore; i++)
            {
                int n = chunkStart + 1 + i;
                if (!IsPointToPoint) n %= count;
                if (n >= count) n = count - 1;

                // Interior nodes only (seam blind spot over the start-line straight).
                if (n < 6 || n > count - 6) continue;

                if (TrackPoints[n].PreciseCurveRadius >= CornerLimitRadius) continue;

                // Inside a corner region: walk to its limits on each side.
                int start = n;
                while (start > 6 && TrackPoints[start - 1].PreciseCurveRadius < CornerLimitRadius && n - start < MaxCornerRegionWalk)
                    start--;
                int end = n;
                while (end < count - 7 && TrackPoints[end + 1].PreciseCurveRadius < CornerLimitRadius && end - n < MaxCornerRegionWalk)
                    end++;

                // Apex = region midpoint (symmetric for a clean circumradius read).
                int apex = start + (end - start) / 2;

                // Apex already behind us: skip past the region.
                if (apex <= cur)
                {
                    r.CornerScanNode = end;
                    r.Brain.Corner = null;
                    return;
                }

                // Corner found. Head past the region end and 5s exit window.
                CornerPoint nextCorner = FillCornerPoint(r, start, end, apex);
                float apexSpeed = CornerApexSpeed(nextCorner, r);
                r.CornerScanNode = Math.Max(end, apex + (int)(apexSpeed * 5f) - 1);
                if (r.Brain.Corner == null || r.Brain.Corner.Point.Node != nextCorner.Node)
                    r.Brain.Corner = new Corner(apexSpeed, nextCorner);
                return;
            }

            // No corner in this chunk. Advance and keep scanning next core.
            r.CornerScanNode = chunkStart + ChunkScanNodesPerCore;
            r.Brain.Corner = null;
        }

        static CornerPoint FillCornerPoint(Racer r, int start, int end, int apex)
        {
            CornerPoint c = r.LiveCorner;
            c.Node = apex;
            c.StartNode = start;
            c.EndNode = end;
            c.RequiresEarlyBrake = false;
            c.RampEndNode = -1;
            c.Elevation = TrackPoints[apex].Elevation;

            // Corner angle: signed heading change across 20 nodes (sign: + left, - right).
            if (apex >= 20 && apex < TrackPoints.Count - 20)
            {
                Vector3 pre = TrackPoints[apex - 20].Direction;
                Vector3 fut = TrackPoints[apex + 20].Direction;
                c.Angle = Vector3.SignedAngle(pre, fut, Vector3.WorldUp);
                c.ElevationChange = ((fut - pre).Z * 90);
            }
            else
            {
                c.Angle = 0f;
                c.ElevationChange = 0f;
            }

            // Spans from the region limits: the apex offset within [start, end].
            c.LengthStart = Math.Max(1, apex - start);
            c.LengthEnd = Math.Max(1, end - apex);

            // Estimate the corner radius from the region endpoints and apex.
            float supposed = Circumradius3D(TrackPoints[start].Position, TrackPoints[apex].Position, TrackPoints[end].Position);
            if (supposed <= 0f || float.IsNaN(supposed) || float.IsInfinity(supposed) || supposed >= 999f)
            {
                // Fall back to the tightest region radius for degenerate arcs.
                supposed = 999f;
                for (int m = start; m <= end; m++)
                {
                    float rm = TrackPoints[m].PreciseCurveRadius;
                    if (rm > 0f && rm < supposed) supposed = rm;
                }
            }
            c.SupposedRadius = Clamp(supposed, 5f, 9999f);

            // Detect a lifting lip before the corner and brake before it.
            const int LipLookbackNodes = 60;
            const float LipGsThreshold = -1.0f;
            const float LipReferenceSpeed = 30f;
            int cornerStart = apex - c.LengthStart;
            if (apex >= LipLookbackNodes + 2 && cornerStart >= 2 && apex + 1 <= RouteNodes.Count - 1)
            {
                for (int lip = apex - LipLookbackNodes; lip + 2 <= cornerStart; lip++)
                {
                    Vector3 a = RouteNodes[lip - 2];
                    Vector3 b = RouteNodes[lip];
                    Vector3 e = RouteNodes[lip + 2];
                    float deltaGs = HillGripDeltaGs(a, b, e, LipReferenceSpeed);
                    if (deltaGs < LipGsThreshold)
                    {
                        c.RequiresEarlyBrake = true;
                        c.RampEndNode = lip;
                        break;
                    }
                }
            }

            // Evaluate vertical curvature at the apex speed.
            c.CrestGs = 0f;
            if (apex >= 3 && apex < TrackPoints.Count - 3)
            {
                float apexSpeed = CornerApexSpeed(c, r);
                c.CrestGs = HillGripDeltaGs(
                    TrackPoints[apex - 3].Position,
                    TrackPoints[apex].Position,
                    TrackPoints[apex + 3].Position,
                    apexSpeed);
            }

            return c;
        }

        
        
        

        
        
        
        const float BrakingCoastSecondsBeforeApex = 1f; // reach corner speed this many seconds before the apex

        public static float MaxSpeedForBrakingDistance(CornerPoint c, Racer r)
        {
            
            int apexNode = c.Node;
            if (!IsPointToPoint && apexNode < 0) apexNode += TrackPoints.Count;
            float velTarget = r.Brain.Corner.Speed;
            // Reserve coasting distance before the apex.
            float coastReserve = velTarget * BrakingCoastSecondsBeforeApex;
            // Pressure scales the reserve: low pressure brakes earlier.
            coastReserve *= ARS.Remap(r.Pressure, 100f, 0f, 0.8f, 1.2f, true);
            // Brake before a lifting lip.
            int brakeTargetNode = (c.RequiresEarlyBrake && c.RampEndNode >= 0) ? c.RampEndNode : apexNode;
            float rawDistance = brakeTargetNode - r.CurrentTrackPoint.Node;
            if (!IsPointToPoint && rawDistance < 0f) rawDistance += TrackPoints.Count;
            if (rawDistance < 0f) rawDistance = 0f;
            float distance = rawDistance - coastReserve;
            if (distance < 0f) distance = 0f;

            float brakingAbility = Math.Min(r.Handling.BrakingAbility * 4, r.VehicleData.CurrentMechanicalGrip);
            
            float decel = brakingAbility * r.Handling.Gravity * r.BrakeDecelFactor;
            // Yielding cars perceive half the deceleration, so they brake earlier.
            if (r.ActiveManeuver.Type == ManeuverType.Yield)
                decel *= 0.5f;

            float spd = (float)Math.Sqrt(velTarget * velTarget + 2f * decel * distance);
            if (float.IsNaN(spd) || float.IsInfinity(spd)) spd = 999f;
            return spd;
        }

        

        static Random _random = new Random();
        public static int GetRandomInt(int min, int max)
        {
            return _random.Next(min, max);
        }

        public static void DrawLine(Vector3 from, Vector3 to, Color color)
        {
            Function.Call(Hash.DRAW_LINE, from.X, from.Y, from.Z, to.X, to.Y, to.Z, color.R, color.G, color.B, color.A);
        }

        
        void OnAbort(object sourc, EventArgs e)
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
            foreach (Prop p in TrackLimits) if (CanWeUse(p)) p.Delete();
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
            Function.Call(Hash._0x10D373323E5B9C0D);
            Game.Player.Character.HasGravity = true;
            
        }
        

        
        public static bool WasCheatStringJustEntered(string cheat)
        {
            return Function.Call<bool>(Hash._0x557E43C447E700A8, Game.GenerateHash(cheat));
        }

        
        void LoadSettings()
        {
            Thread.CurrentThread.CurrentCulture = CultureInfo.GetCultureInfo("en-US");

            Log(LogImportance.Info, "Loading Options.ini ...");
            if (File.Exists(SettingsFolder + @"\Options.ini"))
            {

                SettingsFile = ScriptSettings.Load(SettingsFolder + @"\Options.ini");


                _intendedOpponents = 4;
                DisciplineFilter = SettingsFile.GetValue<string>("GENERAL_SETTINGS", "Disciplines", "muscle");
                Log(LogImportance.Info, "Loaded Options.");
            }
            else
            {
                Log(LogImportance.Error, " '" + SettingsFolder + "/Options.ini' does not exist. All config values will be default.");
                UI.Notify("~o~Failed to load the Options file.~w~ Check you've installed ARS properly.");
            }

            Log(LogImportance.Info, "Loading Developer Settings.ini ...");
            if (File.Exists(SettingsFolder + @"\Developer Settings.ini"))
            {

                DevSettingsFile = ScriptSettings.Load(SettingsFolder + @"\Developer Settings.ini");


                Log(LogImportance.Info, "Loaded Developer settings.");
            }
            else
            {
                Log(LogImportance.Error, " '" + SettingsFolder + "/Developer Settings.ini' does not exist. All config values will be default.");
                UI.Notify("~o~Failed to load the Settings file.~w~ Check you've installed ARS properly.");
            }

            Log(LogImportance.Info, "Loading Settings.ini ...");
            RaceSettingsFile = ScriptSettings.Load(SettingsFolder + @"\Settings.ini");
            Log(LogImportance.Info, "Loaded Settings.");

            if (File.Exists(SettingsFolder + @"\MemoryOffsets.ini"))
            {
                ScriptSettings menOffexts = ScriptSettings.Load(SettingsFolder + @"\MemoryOffsets.ini");
                ThrottleOffset = menOffexts.GetValue<ulong>("MEMORY_OFFSETS", "Throttle", 0x0);
                SteerOffset = menOffexts.GetValue<ulong>("MEMORY_OFFSETS", "Steer", 0x0);
                BrakeOffset = menOffexts.GetValue<ulong>("MEMORY_OFFSETS", "Brake", 0x0);
                Log(LogImportance.Info, "Loaded Memory Offsets.");

                
                Log(LogImportance.Info, "[MEMORY] Learned the steer offset from file: " + SteerOffset);

            }
            else
            {
                Log(LogImportance.Error, " '" + SettingsFolder + "/MemoryOffsets.ini' does not exist. ARS will try to learn the memory offsets from the game.");
                UI.Notify("~o~Failed to load the MemoryOffsets file.~w~ Check you've installed ARS properly.");
            }

            Log(LogImportance.Info, "Loading DevSettings.ini ...");
            DevMenuFile = null;
            if (File.Exists(SettingsFolder + @"\DevSettings.ini"))
            {
                DevMenuFile = ScriptSettings.Load(SettingsFolder + @"\DevSettings.ini");
                foreach (Options option in DebugToggles.Keys.ToArray())
                    DebugToggles[option] = DevMenuFile.GetValue<bool>("DEBUG", option.ToString(), DebugToggles[option]);
                Log(LogImportance.Info, "Loaded DevSettings.");
            }
            else
            {
                Log(LogImportance.Error, " '" + SettingsFolder + "/DevSettings.ini' does not exist. Debug toggles will be default.");
            }

        }
        public enum LogImportance { Info, Error, Fatal }
        public static void Log(LogImportance i, string text, bool forced = false)
        {
            if (DevSettingsFile != null && DevSettingsFile.GetValue<LogImportance>("GENERAL", "LogLevel", LogImportance.Info) > i && !forced) return;
            string log = "\n[" + DateTime.Now + "](" + i.ToString() + "): " + text;
            File.AppendAllText(ScriptsFolder + @"\Log.log", log);
        }

        

        public static bool CanWeUse(Entity entity)
        {
            return entity != null && entity.Exists();
        }

        // Pace index for the player's current vehicle, using the same model-level natives as the grid caches.
        float ComputePlayerCarPaceIndex()
        {
            Vehicle v = Game.Player.Character.CurrentVehicle;
            if (!CanWeUse(v)) return PowerTargetScale;
            Model model = v.Model;
            float grip = Function.Call<float>((Hash)0x539DE94D44FDFD0D, model.Hash);
            float topSpeedMph = MpsToMph(Function.Call<float>((Hash)0xF417C2502FFFED43, model.Hash));
            float accel = Function.Call<float>(Hash.GET_VEHICLE_MODEL_ACCELERATION, model.Hash);
            bool isElectric = Function.Call<int>((Hash)0xD839450756ED5A80, model.Hash) != 0;
            return ComputePaceIndex(topSpeedMph, grip, accel, isElectric);
        }


        void DisplayHelpTextThisFrame(string text)
        {
            if (HelpMessages.Count > 0) return;
            Function.Call(Hash._SET_TEXT_COMPONENT_FORMAT, "STRING");
            Function.Call(Hash._ADD_TEXT_COMPONENT_STRING, text);
            Function.Call(Hash._DISPLAY_HELP_TEXT_FROM_STRING_LABEL, 0, false, false, -1);
        }


        static Vector2 World3DToScreen2d(Vector3 pos)
        {
            var x2dp = new OutputArgument();
            var y2dp = new OutputArgument();

            Function.Call<bool>(Hash._WORLD3D_TO_SCREEN2D, pos.X, pos.Y, pos.Z, x2dp, y2dp);
            return new Vector2(x2dp.GetResult<float>(), y2dp.GetResult<float>());
        }

        public enum DrawTextAlign { Center, Left, Right }
        public enum DrawTextFont { Default, Italics, Squared }
        public static void DrawText(Vector3 pos, string t, Color c, float scale)
        {
            Vector2 screeninfo = World3DToScreen2d(pos);
            Function.Call(Hash._SET_TEXT_ENTRY, "STRING");
            Function.Call(Hash.SET_TEXT_CENTRE, true);
            Function.Call(Hash.SET_TEXT_COLOUR, c.R, c.G, c.B, c.A);
            Function.Call(Hash.SET_TEXT_SCALE, 1f, scale);
            Function.Call(Hash.SET_TEXT_DROP_SHADOW, true);
            Function.Call(Hash._ADD_TEXT_COMPONENT_STRING, t);
            Function.Call(Hash._DRAW_TEXT, screeninfo.X, screeninfo.Y);
        }


        public static float DrawText(Vector2 pos, string t, Color c, DrawTextFont font, DrawTextAlign align, float scale)
        {
            Function.Call(Hash._SET_TEXT_ENTRY, "STRING");
            Function.Call(Hash.SET_TEXT_COLOUR, c.R, c.G, c.B, c.A);
            Function.Call(Hash.SET_TEXT_SCALE, 1f, scale);
            Function.Call(Hash.SET_TEXT_RIGHT_JUSTIFY, align == DrawTextAlign.Right);
            Function.Call(Hash.SET_TEXT_DROP_SHADOW, true);
            Function.Call(Hash.SET_TEXT_JUSTIFICATION, (int)align);
            Function.Call(Hash.SET_TEXT_FONT, (int)font);
            Function.Call(Hash._ADD_TEXT_COMPONENT_STRING, t);
            Function.Call(Hash._DRAW_TEXT, pos.X, pos.Y);
            Function.Call(Hash._0x54CE8AC98E120CAB, "STRING");
            Function.Call(Hash._ADD_TEXT_COMPONENT_STRING, t);

            float size = Function.Call<float>(Hash._0x85F061DA64ED2F67, 1);

            return size;
        }


        


        static public unsafe ulong GetHandlingPtr(Vehicle handle)
        {
            GameVersion gameVersion = Game.Version;
            var address = (ulong)handle.MemoryAddress;
            if (HandlingPtr == 0x0)
            {
                IntPtr addr = (IntPtr)FindPattern("\x3C\x03\x0F\x85\x00\x00\x00\x00\x48\x8B\x41\x20\x48\x8B\x88", "xxxx????xxxxxxx");

                if (addr != null)
                {
                    HandlingPtr = *(uint*)(addr + 0x16);
                }
            }
            return *((ulong*)(address + HandlingPtr));
        }

        public static unsafe float GetTRCurveLat(Vehicle v)
        {

            if (!CanWeUse(v)) return 0f;
            ulong handlingAddress = GetHandlingPtr(v);
            if (handlingAddress == 0) return 0f;
            ulong tractionCurveMaxOffset = 0x0098;
            if (handlingAddress < 1) return 0f;
            float result = *(float*)(handlingAddress + tractionCurveMaxOffset);
            return result;
        }
        
        public static unsafe float GetSteerLock(Vehicle v)
        {

            if (!CanWeUse(v)) return 0f;
            ulong handlingAddress = GetHandlingPtr(v);
            if (handlingAddress == 0) return 0f;
            ulong steerlock = 0x0080;
            if (handlingAddress < 1) return 0f;
            float result = *(float*)(handlingAddress + steerlock);
            return result;
        }
        public static unsafe float GetDownforce(Vehicle v)
        {

            if (!CanWeUse(v)) return 0f;
            ulong handlingAddress = GetHandlingPtr(v);
            if (handlingAddress == 0) return 0f;
            ulong downfOffset = 0x0014;
            if (handlingAddress < 1) return 0f;
            float result = *(float*)(handlingAddress + downfOffset);
            return result;
        }

        // Engine reference: wheel.cpp UpdateDownforce() lines 7625-7670 + the per-tick application at 8194-8199.
        // Two velocities drive downforce: forward speed scales downForceScale (how strong the downforce CAN be),
        // and lateral speed scales the actual vertical acceleration. We approximate the lateral input with
        // centripetal acceleration (v²/r) instead of world-frame velocity·right because ARS drives via
        // pure-pursuit and rarely slides — world-frame lateral velocity is near zero under good driving,
        // which would zero out the bonus. Centripetal acceleration is what determines tire grip load
        // in steady-state cornering; that's the load the downforce actually counteracts.
        // forwardMs: |velocity·forward| (m/s). lateralMs: centripetal accel v²/r (m/s²).
        // Returns total extra grip in Gs that should be added to mechanical grip under this load.
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

            // perWheel (unitless) × lateralMs (m/s²) → m/s² of downforce acceleration → /9.8 for Gs.
            return GsOrZero(perWheel * lateralMs / 9.8f);
        }

        private static float GsOrZero(float gs) => (float.IsNaN(gs) || gs > 5f) ? 0f : gs;

        // Engine uses |velocity·right| as the lateral component for downforce application (wheel.cpp:8194-8199).
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


        
        public static unsafe int GetHandlingFlags(Vehicle v)
        {

            if (!CanWeUse(v)) return 0;
            ulong handlingAddress = GetHandlingPtr(v);
            if (handlingAddress == 0) return 0;
            ulong modelflags = 0x128;
            if (handlingAddress < 1) return 0;
            int result = *(int*)(handlingAddress + modelflags);
            return result;
        }
        
        

        public static void RandomTuning(Vehicle veh, bool color, bool livery, bool parts, bool performance, bool horn)
        {

            veh.InstallModKit();

            Script.Wait(100);
            if (livery && veh.LiveryCount > 0) veh.Livery = GetRandomInt(0, veh.LiveryCount);
            if (veh.GetModCount(VehicleMod.Livery) > 0) veh.SetMod(VehicleMod.Livery, GetRandomInt(0, veh.GetModCount(VehicleMod.Livery)), false);

            if (performance) veh.ToggleMod(VehicleToggleMod.Turbo, true);
            if (color)
            {
                int c = GetRandomInt(1, Function.Call<int>(Hash.GET_NUMBER_OF_VEHICLE_COLOURS, veh));
                Function.Call(Hash.SET_VEHICLE_COLOUR_COMBINATION, veh, c);
            }


            
            foreach (int mod in Enum.GetValues(typeof(VehicleMod)).Cast<VehicleMod>())
            {
                if (mod == (int)VehicleMod.Horns) continue;
                if (veh.GetModCount((VehicleMod)mod) > 0)
                {

                    if (new List<VehicleMod> { VehicleMod.Engine, VehicleMod.Transmission, VehicleMod.Brakes, VehicleMod.Suspension }.Contains((VehicleMod)mod))
                    {
                        if (!performance) continue;
                    }
                    else if (!parts) continue;
                    if (mod == (int)VehicleMod.FrontWheels) continue;
                    if (mod == (int)VehicleMod.Suspension) continue;
                    if (mod == (int)VehicleMod.Livery && !livery) continue;
                    if (mod == (int)VehicleMod.Horns && !horn) continue;
                    int d = veh.GetModCount((VehicleMod)mod);

                    if (d > 0)
                    {
                        Script.Wait(30);
                        veh.SetMod((VehicleMod)mod, GetRandomInt(0, d), false);

                    }
                }
            }

            
            if (World.CurrentDayTime.Hours > 20 || World.CurrentDayTime.Hours < 7)
            {

                
                Script.Wait(30);
                Color neoncolor = Color.Red;
                veh.NeonLightsColor = neoncolor;
                





            }






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


        public XmlDocument LoadDriver(string DriverName)
        {
            List<dynamic> info = new List<dynamic>();

            string fileRouteNodes = ScriptsFolder + @"\Drivers\" + DriverName + ".xml";



            
            

            Script.Wait(200);

            XmlDocument xmlFile = new XmlDocument();
            xmlFile.Load(fileRouteNodes);

            if (xmlFile == null)
            {
                UI.Notify("~r~cannot find file");
                return xmlFile;
            }


            info.Add(xmlFile);
            return xmlFile;
        }
        public static string CreateDriver(Ped ped)
        {
            string name = Game.GetUserInput(32);

            string fileRouteNodes = ScriptsFolder + @"\Drivers\" + name + ".xml";

            File.AppendAllText(fileRouteNodes, "");

            XmlDocument xmlFile = new XmlDocument();
            if (xmlFile == null)
            {
                UI.Notify("~r~cannot find file");
            }

            XmlNode data = xmlFile.CreateNode(XmlNodeType.Element, "Data", null);
            xmlFile.AppendChild(data);

            XmlNode driver = xmlFile.CreateNode(XmlNodeType.Element, "Driver", null);

            
            XmlNode temp = xmlFile.CreateElement("Name");
            temp.InnerText = name;
            driver.AppendChild(temp);

            temp = xmlFile.CreateElement("Model");
            temp.InnerText = ped.Model.Hash.ToString();
            driver.AppendChild(temp);
            temp = xmlFile.CreateElement("Clothes");
            for (int i = -1; i < 20; i++)
            {

                int Component = Function.Call<int>(Hash.GET_PED_DRAWABLE_VARIATION, ped, i);
                int drawable = Function.Call<int>(Hash.GET_PED_TEXTURE_VARIATION, ped, i);

                if (Component > -1 && drawable > -1)
                {
                    XmlElement cloth = xmlFile.CreateElement("Cloth");
                    cloth.InnerText = i.ToString();
                    XmlAttribute id = xmlFile.CreateAttribute("DrawableID");
                    id.InnerText = drawable.ToString();
                    XmlAttribute component = xmlFile.CreateAttribute("ComponentID");
                    component.InnerText = component.ToString();

                    cloth.Attributes.Append(component);

                    cloth.Attributes.Append(id);
                    
                    temp.AppendChild(cloth);
                }

                
            }
            for (int i = -1; i < 20; i++)
            {

                int Component = Function.Call<int>(Hash.GET_PED_PROP_INDEX, ped, i);
                int drawable = Function.Call<int>(Hash.GET_PED_PROP_TEXTURE_INDEX, ped, i);

                if (Component > -1 && drawable > -1)
                {
                    XmlElement prop = xmlFile.CreateElement("Prop");
                    prop.InnerText = i.ToString();
                    XmlAttribute id = xmlFile.CreateAttribute("PropID");
                    id.InnerText = Component.ToString();
                    XmlAttribute component = xmlFile.CreateAttribute("TextureID");
                    component.InnerText = drawable.ToString();

                    prop.Attributes.Append(component);

                    prop.Attributes.Append(id);
                    
                    temp.AppendChild(prop);
                }

                
            }
            driver.AppendChild(temp);

            
            XmlNode driverSkills = xmlFile.CreateElement("Skills");


            XmlNode skill = xmlFile.CreateElement("Skill");
            skill.InnerText = "ExampleSkill";
            XmlAttribute att = xmlFile.CreateAttribute("value");
            att.InnerText = "25";
            skill.Attributes.Append(att);
            driverSkills.AppendChild(skill);

            skill = xmlFile.CreateElement("Skill");
            skill.InnerText = "ExampleSkill2";
            att = xmlFile.CreateAttribute("value2");
            att.InnerText = "50";
            skill.Attributes.Append(att);
            driverSkills.AppendChild(skill);



            driver.AppendChild(driverSkills);


            data.AppendChild(driver);

            xmlFile.AppendChild(data);

            xmlFile.Save(ScriptsFolder + @"\Drivers\" + name + ".xml");
            UI.Notify("Saved");
            return "Finished";

        }
        VehicleColor[] _randomColors = { VehicleColor.MetallicRed, VehicleColor.MetallicRaceYellow, VehicleColor.MetallicBlue, VehicleColor.MetallicOrange, VehicleColor.MetallicSteelGray };

        List<XmlDocument> _cachedCandidates = new List<XmlDocument>();

        void FillCachedCandidates(string dlist, int maxcars, bool allowScriptYield = true)
        {
            bool allowDuplicates = SettingsFile.GetValue<bool>("GENERAL_SETTINGS", "AllowDuplicates", true);
            if (HardcodedRoster != null && HardcodedRoster.Count > 0)
            {
                _cachedCandidates = VehicleSelector.SelectHardcoded(HardcodedRoster, _racerTagLookup, maxcars, allowDuplicates, Yield, GetRandomInt, text => Log(LogImportance.Info, text));
            }
            else
            {
                _cachedCandidates = VehicleSelector.Select(_racerTagLookup, ModelPaceIndexCache, maxcars, allowDuplicates, allowScriptYield, Yield, GetRandomInt, text => Log(LogImportance.Info, text), PowerTargetScale, PowerBracketScale, AlwaysIncludeModelNames);
            }
        }

        void LoadGrid(string dlist, int maxcars)
        {
            SetLoadingPromptText("Loading vehicles...");

            Log(LogImportance.Info, "Loading vehicle models");
            Vehicle lastCar = null;
            List<dynamic> result = new List<dynamic>();

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

            List<string> GetVehicleTags(XmlDocument file)
            {
                XmlNodeList disciplines = file.SelectNodes("//Disciplines/Discipline");
                List<string> tags = new List<string>();
                foreach (XmlElement t in disciplines)
                {
                    tags.Add(t.InnerText.ToLowerInvariant());
                }
                return tags;
            }

            void ApplyCarAppearance(XmlDocument file, Vehicle car, List<string> tags)
            {
                if (tags.Contains("tuner"))
                {
                    RandomTuning(car, true, true, true, true, false);
                }
                else
                {
                    if (file.SelectSingleNode("//WheelType") != null) car.WheelType = (VehicleWheelType)int.Parse(file.SelectSingleNode("//WheelType").InnerText);
                    if (file.SelectSingleNode("//Livery") != null) car.Livery = int.Parse(file.SelectSingleNode("//Livery").InnerText);
                    if (file.SelectSingleNode("//Primary") != null) car.PrimaryColor = (VehicleColor)int.Parse(file.SelectSingleNode("//Primary").InnerText);
                    else
                    {
                        if (car.ColorCombinationCount > 2) car.ColorCombination = GetRandomInt(0, car.ColorCombinationCount);
                        else
                        {
                            VehicleColor c = _randomColors[GetRandomInt(0, _randomColors.Length - 1)];
                            car.PrimaryColor = c;
                            car.SecondaryColor = c;
                            car.PearlescentColor = c;
                        }
                    }
                    if (file.SelectSingleNode("//Secondary") != null) car.SecondaryColor = (VehicleColor)int.Parse(file.SelectSingleNode("//Secondary").InnerText);
                    if (file.SelectSingleNode("//Pearl") != null) car.PearlescentColor = (VehicleColor)int.Parse(file.SelectSingleNode("//Pearl").InnerText);
                    if (file.SelectSingleNode("//Wheel") != null) car.RimColor = (VehicleColor)int.Parse(file.SelectSingleNode("//Wheel").InnerText);
                    if (file.SelectSingleNode("//Dash") != null) car.DashboardColor = (VehicleColor)int.Parse(file.SelectSingleNode("//Dash").InnerText);
                    if (file.SelectSingleNode("//Trim") != null) car.TrimColor = (VehicleColor)int.Parse(file.SelectSingleNode("//Trim").InnerText);

                    if (NodeExists(file, "//Mods"))
                    {
                        if (file.SelectNodes("//Mods/Mod").Count > 0)
                        {
                            foreach (XmlElement modElement in file.SelectNodes("//Mods/Mod"))
                            {
                                if (int.Parse(modElement.GetAttribute("ModIndex")) == 48)
                                {
                                    if (int.Parse(modElement.InnerText) == -1) car.SetMod(VehicleMod.Livery, GetRandomInt(0, car.GetModCount(VehicleMod.Livery)), false);
                                }
                                else
                                {
                                    car.SetMod((VehicleMod)int.Parse(modElement.GetAttribute("ModIndex")), int.Parse(modElement.InnerText), modElement.HasAttribute("IsCustom") && modElement.GetAttribute("IsCustom").ToLowerInvariant() == "true");
                                }
                            }
                        }
                    }
                    else
                    {
                        
                    }

                    foreach (XmlElement modElement in file.SelectNodes("//Mods/ToggleMod")) car.ToggleMod((VehicleToggleMod)int.Parse(modElement.GetAttribute("ModIndex")), int.Parse(modElement.InnerText) == 1 ? true : false);

                    if (file.SelectNodes("//Extras/Extra").Count > 0) for (int i = 0; i < 15; i++) if (car.ExtraExists(i)) car.ToggleExtra(i, false);
                    foreach (XmlElement modElement in file.SelectNodes("//Extras/Extra")) car.ToggleExtra(int.Parse(modElement.InnerText), true);
                }
            }

            void ApplyAccelerationOverride(XmlDocument file, Vehicle car)
            {
                if (file.SelectSingleNode("//Acceleration") != null)
                {
                    float acc = float.Parse(file.SelectSingleNode("//Acceleration").InnerText) - 0.05f;

                    if (Function.Call<float>(Hash.GET_VEHICLE_ACCELERATION, car) < acc && car.HighGear > 2)
                    {
                        UI.Notify(car.DisplayName + " powers up");
                        float mul = 10f;
                        while (Function.Call<float>(Hash.GET_VEHICLE_ACCELERATION, car) < acc && mul < 500)
                        {
                            mul += 10;
                            car.EnginePowerMultiplier = mul;
                            Script.Wait(0);
                        }
                    }
                }
            }

            Ped CreateDriverPed(XmlDocument file, Vehicle car, List<string> tags, out XmlDocument driverXml)
            {
                driverXml = new XmlDocument();
                Model driverModel;
                string driverChosen = null;
                try
                {
                    if (file.SelectSingleNode("//DriverName") != null) driverChosen = file.SelectSingleNode("//DriverName").InnerText;
                    if (!string.IsNullOrEmpty(driverChosen))
                    {
                        driverXml = LoadDriver(driverChosen);
                        driverModel = int.Parse(driverXml.SelectSingleNode("//Model").InnerText);
                    }
                    else
                    {
                        driverModel = PedHash.Car3Guy2;
                    }
                }
                catch (Exception)
                {
                    Log(LogImportance.Info, "Driver XML not found, using fallback ped.", true);
                    driverModel = PedHash.Car3Guy2;
                }

                if (tags.Contains("street"))
                {
                    driverModel = StreetRacerModels[GetRandomInt(0, StreetRacerModels.Length - 1)];
                }

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

            void ApplyDriverClothes(Ped driverPed, XmlDocument driverXml, List<string> tags)
            {
                if (!tags.Contains("street"))
                {
                    foreach (XmlElement e in driverXml.SelectNodes("//Cloth"))
                    {
                        int component = int.Parse(e.GetAttribute("ComponentID"));
                        int drawable = int.Parse(e.GetAttribute("DrawableID"));
                        Function.Call(Hash.SET_PED_COMPONENT_VARIATION, driverPed, int.Parse(e.InnerText), component, drawable, 2);
                    }

                    foreach (XmlElement e in driverXml.SelectNodes("//Clothes/Prop"))
                    {
                        int prop = int.Parse(e.GetAttribute("PropID"));
                        int texture = int.Parse(e.GetAttribute("TextureID"));
                        Function.Call(Hash.SET_PED_PROP_INDEX, driverPed, int.Parse(e.InnerText), prop, texture, true);
                    }
                }
            }

            void AddRacer(XmlDocument file, Vehicle car, Ped driverPed)
            {
                if (CanWeUse(car))
                {
                    Racer r = new Racer(car, driverPed);

                    if (file.SelectSingleNode("//Name") != null) r.Name = file.SelectSingleNode("//Name").InnerText;
                    if (file.SelectSingleNode("//Nickname") != null) r.Name = file.SelectSingleNode("//Nickname").InnerText;
                    if (r.Name == "NULL" || r.Name == null) r.Name = r.Car.DisplayName.ToString()[0].ToString().ToUpper() + r.Car.DisplayName.ToString().Substring(1).ToLowerInvariant();
                    if (car == Game.Player.Character.CurrentVehicle) r.Name = Game.Player.Name;

                    






                    Racers.Add(r);
                }
            }

            foreach (XmlDocument file in _cachedCandidates)
            {
                Vehicle car = null;
                Ped driverPed = null;
                try
                {
                    string modelName = file.SelectSingleNode("//Model").InnerText;
                    Model vehicleModel = LoadVehicleModel(modelName);
                    if (!vehicleModel.IsLoaded)
                    {
                        Log(LogImportance.Error, "Skipping " + modelName + " - model not loaded", true);
                        continue;
                    }

                    car = World.CreateVehicle(vehicleModel, RouteNodes[(Racers.Count + 1) * 10]);
                    car.Heading = (RouteNodes[2] - RouteNodes[0]).ToHeading();
                    car.InstallModKit();

                    List<string> tags = GetVehicleTags(file);
                    try { ApplyCarAppearance(file, car, tags); } catch (Exception ex) { Log(LogImportance.Info, "Appearance skipped: " + ex.Message); }
                    // Menyoo livery override: if a matching Menyoo tuning file exists for this model,
                    // apply one at random (cosmetic only, separate from the ARS supplier pool).
                    // MenyooAppearance.Apply(car); // TEMP: skipped entirely
                    ApplyAccelerationOverride(file, car);

                    XmlDocument driverXml;
                    driverPed = CreateDriverPed(file, car, tags, out driverXml);
                    if (driverPed == null)
                    {
                        Log(LogImportance.Error, "Skipping " + modelName + " - no driver ped.", true);
                        car.Delete();
                        continue;
                    }
                    ApplyDriverClothes(driverPed, driverXml, tags);
                    AddRacer(file, car, driverPed);

                    lastCar = car;
                }
                catch (Exception ex)
                {
                    Log(LogImportance.Error, "Failed to load racer: " + ex.Message, true);
                    try { if (driverPed != null) driverPed.Delete(); } catch (Exception) { }
                    try { if (car != null) car.Delete(); } catch (Exception) { }
                }
            }

            // Add nearby vehicles as additional racers after the roster grid is built.
            if (DebugToggles[Options.UseNearbyCars])
            {
                var nearby = GetNearbyCandidates();
                int added = 0;
                foreach (Vehicle veh in nearby)
                {
                    if (Racers.Count >= maxcars) break;
                    Ped driver = veh.CreateRandomPedOnSeat(VehicleSeat.Driver);
                    if (driver == null || !CanWeUse(driver)) continue;
                    Racers.Add(new Racer(veh, driver));
                    added++;
                }
                Log(LogImportance.Info, "Added " + added + " nearby cars to the grid.");
            }

            result.Add(lastCar);

            // Stay in None — the race isn't ready to start until SetupRace (phase 3)
            // explicitly moves to NotInitiated. Setting it here caused auto-start.
            RaceStatus = RaceState.None;
            Function.Call(Hash._0x10D373323E5B9C0D);
        }


        List<Vehicle> GetNearbyCandidates()
        {
            return GlobalTraffic.Where(s => s.Health > 0 && s.IsDriveable && s.IsInRangeOf(Game.Player.Character.Position, 30f) && !CanWeUse(s.GetPedOnSeat(VehicleSeat.Driver))).ToList();
        }
        void CreateVehicle(Vehicle car, bool auto = false)
        {

            if (!CanWeUse(car))
            {
                UI.Notify("~o~Weird error.~w~Car doesn't seem to exist, try reentering.");
                return;
            }
            string name = "";
            if (auto) name = car.FriendlyName;
            else
            {
                UI.ShowSubtitle("~b~Enter your car's name, or leave empty to auto-generate one. ~w~~n~This will be the filename name.");
                name = Game.GetUserInput(32);
            }
            if (name == null || name == "") name = car.FriendlyName;
            if (name == null || name == "") name = car.DisplayName.ToString()[0].ToString().ToUpper() + car.DisplayName.ToString().Substring(1).ToLowerInvariant();

            string fileRouteNodes = ScriptsFolder + @"\Vehicles\" + name + ".xml";


            if (File.Exists(fileRouteNodes))
            {
                DateTime today = DateTime.Now;
                name += " (" + today.Year + today.Month + today.Day + today.Hour + today.Minute + today.Second + ")";

                
            }





            File.AppendAllText(fileRouteNodes, "");

            Script.Wait(200);

            XmlDocument xmlFile = new XmlDocument();


            XmlNode data = xmlFile.CreateNode(XmlNodeType.Element, "Data", null);
            xmlFile.AppendChild(data);

            XmlNode vehicle = xmlFile.CreateNode(XmlNodeType.Element, "Vehicle", null);


            XmlNode temp = xmlFile.CreateElement("Name");
            temp.InnerText = car.FriendlyName;
            vehicle.AppendChild(temp);

            XmlNode vehicleClass = xmlFile.CreateElement("Class");
            vehicleClass.InnerText = car.ClassType.ToString();
            vehicle.AppendChild(vehicleClass);

            List<string> keywords = new List<string>();
            string nameAutotag = car.FriendlyName;

            nameAutotag = nameAutotag.Replace(@"-", "");
            nameAutotag = nameAutotag.Replace(@"/", "");
            nameAutotag = nameAutotag.Replace(@" ", "");


            keywords.Add(car.ClassType.ToString());
            keywords.Add(car.DisplayName);
            
            keywords.Add(nameAutotag);
            UI.ShowSubtitle("~b~Enter the vehicle's Class set.~w~~n~Write as much as you need, separate with spaces.");
            if (!auto)
            {

                string userTags = Game.GetUserInput(32);
                if (userTags != "") keywords.AddRange(userTags.Split(' '));

            }
            XmlNode keyw = xmlFile.CreateElement("Disciplines");
            foreach (string keyword in keywords)
            {
                XmlNode ktoadd = xmlFile.CreateElement("Discipline");
                ktoadd.InnerText = keyword.ToLowerInvariant();
                keyw.AppendChild(ktoadd);
            }

            vehicle.AppendChild(keyw);
            temp = xmlFile.CreateElement("Model");
            temp.InnerText = car.Model.Hash.ToString();
            vehicle.AppendChild(temp);

            temp = xmlFile.CreateElement("Livery");
            temp.InnerText = car.Livery.ToString();
            vehicle.AppendChild(temp);

            
            
            temp = xmlFile.CreateElement("Acceleration");
            temp.InnerText = Math.Round(Function.Call<float>(Hash.GET_VEHICLE_ACCELERATION, car), 3).ToString();
            temp.InnerText = temp.InnerText.Replace(",", ".");

            vehicle.AppendChild(temp);


            XmlAttribute vname = xmlFile.CreateAttribute("ModelName");
            vname.InnerText = car.FriendlyName;
            if (vname.InnerText == "NULL") vname.InnerText = car.DisplayName;
            temp.Attributes.Append(vname);

            XmlElement colors = xmlFile.CreateElement("Colors");

            XmlElement c = xmlFile.CreateElement("Primary");
            c.InnerText = ((int)car.PrimaryColor).ToString();
            colors.AppendChild(c);

            c = xmlFile.CreateElement("Secondary");
            c.InnerText = ((int)car.SecondaryColor).ToString();
            colors.AppendChild(c);

            c = xmlFile.CreateElement("Pearl");
            c.InnerText = ((int)car.PearlescentColor).ToString();
            colors.AppendChild(c);

            c = xmlFile.CreateElement("Wheel");
            c.InnerText = ((int)car.RimColor).ToString();
            colors.AppendChild(c);

            c = xmlFile.CreateElement("Dash");
            c.InnerText = ((int)car.DashboardColor).ToString();
            colors.AppendChild(c);

            c = xmlFile.CreateElement("Trim");
            c.InnerText = ((int)car.TrimColor).ToString();
            colors.AppendChild(c);

            vehicle.AppendChild(colors);
            XmlElement mods = xmlFile.CreateElement("Mods");
            for (int i = 0; i <= 100; i++)
            {
                XmlElement component = xmlFile.CreateElement("Mod");

                XmlAttribute attribute = xmlFile.CreateAttribute("ModIndex");
                attribute.InnerText = i.ToString();
                component.Attributes.Append(attribute);

                if (Function.Call<int>(Hash.GET_VEHICLE_MOD, car, i) != -1)
                {
                    component.InnerText = Function.Call<int>(Hash.GET_VEHICLE_MOD, car, i).ToString();
                    bool iscustom = Function.Call<bool>(Hash.GET_VEHICLE_MOD_VARIATION, car, i);

                    if (iscustom)
                    {
                        XmlAttribute customMod = xmlFile.CreateAttribute("IsCustom");
                        customMod.InnerText = iscustom.ToString();
                        component.Attributes.Append(customMod);

                    }
                    mods.AppendChild(component);

                }
            }
            XmlElement wheelkind = xmlFile.CreateElement("WheelType");
            wheelkind.InnerText = ((int)car.WheelType).ToString();
            vehicle.AppendChild(wheelkind);

            for (int i = 0; i <= 100; i++)
            {
                XmlElement mod = xmlFile.CreateElement("ToggleMod");

                XmlAttribute attribute = xmlFile.CreateAttribute("ModIndex");
                attribute.InnerText = i.ToString();
                mod.Attributes.Append(attribute);

                if (Function.Call<int>(Hash.IS_TOGGLE_MOD_ON, car, i) != 0)
                {
                    mod.InnerText = Function.Call<int>(Hash.IS_TOGGLE_MOD_ON, car, i).ToString();
                    mods.AppendChild(mod);
                }
            }
            vehicle.AppendChild(mods);



            XmlElement components = xmlFile.CreateElement("Extras");

            for (int i = 0; i <= 15; i++)
            {
                if (Function.Call<bool>(Hash.IS_VEHICLE_EXTRA_TURNED_ON, car, i))
                {
                    XmlElement component = xmlFile.CreateElement("Extra");
                    component.InnerText = i.ToString();
                    components.AppendChild(component);
                }
            }
            vehicle.AppendChild(components);


            data.AppendChild(vehicle);


            xmlFile.AppendChild(data);

            xmlFile.Save(ScriptsFolder + @"\Vehicles\" + name + ".xml");
            UI.ShowSubtitle("~b~Vehicle saved succesfully.~w~~n~Filename: ~g~" + name + ".xml");
        }

        void CreateVehicleFromName(string modelName)
        {
            Log(LogImportance.Info, "Creating item from model name: " + modelName);

            // Convert model name to hash using natives
            int hash = Function.Call<int>(Hash.GET_HASH_KEY, modelName);

            // Check if model is valid using natives
            if (!Function.Call<bool>(Hash.IS_MODEL_VALID, hash))
            {
                Log(LogImportance.Info, modelName + " is not a valid model hash. Skipping.");
                return;
            }

            // Check if it is a vehicle model.
            if (!Function.Call<bool>(Hash.IS_MODEL_A_VEHICLE, hash))
            {
                Log(LogImportance.Info, modelName + " is not a vehicle model. Skipping.");
                return;
            }

            // Check if it has seats (is a drivable vehicle)
            if (Function.Call<int>(Hash._0x2AD93716F184EDA4, hash) == 0)
            {
                Log(LogImportance.Info, modelName + " has no seats. Aborting this one.");
                return;
            }

            // Check if it has an engine
            if (Function.Call<float>(Hash.GET_VEHICLE_MODEL_ACCELERATION, hash) <= 0.01f)
            {
                Log(LogImportance.Info, modelName + " has no engine. Aborting this one.");
                return;
            }

            string name = modelName;
            string fileRouteNodes = ScriptsFolder + @"\Vehicles\" + name + ".xml";

            if (File.Exists(fileRouteNodes))
            {
                name += " (" + DateTime.Now.GetHashCode() + ")";
            }

            File.AppendAllText(fileRouteNodes, "");

            XmlDocument xmlFile = new XmlDocument();

            XmlNode data = xmlFile.CreateNode(XmlNodeType.Element, "Data", null);
            xmlFile.AppendChild(data);

            XmlNode vehicle = xmlFile.CreateNode(XmlNodeType.Element, "Vehicle", null);

            XmlNode temp = xmlFile.CreateElement("Name");
            temp.InnerText = name;
            vehicle.AppendChild(temp);

            XmlNode vehicleClass = xmlFile.CreateElement("Class");
            vehicleClass.InnerText = ((VehicleClass)Function.Call<int>(Hash.GET_VEHICLE_CLASS_FROM_NAME, hash)).ToString();
            vehicle.AppendChild(vehicleClass);

            List<string> keywords = new List<string>();
            string nameAutotag = name;

            nameAutotag = nameAutotag.Replace(@"-", "");
            nameAutotag = nameAutotag.Replace(@"/", "");
            nameAutotag = nameAutotag.Replace(@" ", "");
            nameAutotag = nameAutotag.Replace(@"+", "");

            keywords.Add(vehicleClass.InnerText);
            keywords.Add(name);
            keywords.Add(nameAutotag);

            XmlNode keyw = xmlFile.CreateElement("Disciplines");
            foreach (string keyword in keywords)
            {
                XmlNode ktoadd = xmlFile.CreateElement("Discipline");
                ktoadd.InnerText = keyword.ToLowerInvariant();
                keyw.AppendChild(ktoadd);
            }

            vehicle.AppendChild(keyw);
            temp = xmlFile.CreateElement("Model");
            temp.InnerText = hash.ToString();
            vehicle.AppendChild(temp);

            XmlAttribute vname = xmlFile.CreateAttribute("ModelName");
            vname.InnerText = modelName;
            temp.Attributes.Append(vname);

            data.AppendChild(vehicle);

            xmlFile.AppendChild(data);

            xmlFile.Save(ScriptsFolder + @"\Vehicles\" + name + ".xml");
            UI.ShowSubtitle("~b~Vehicle saved succesfully.~w~~n~Filename: ~g~" + name + ".xml");
        }

        void CreateVehicleFromHash(VehicleHash h)
        {

            Log(LogImportance.Info, "Creating item from hash: " + h.ToString());

            if (Function.Call<int>(Hash._0x2AD93716F184EDA4, (int)h) == 0)
            {
                Log(LogImportance.Info, h.ToString() + " has no seats. Aborting this one.");
                return;
            }


            if (Function.Call<int>(Hash.GET_VEHICLE_MODEL_ACCELERATION, (int)h) <= 0.01f)
            {
                Log(LogImportance.Info, h.ToString() + " has no engine. Aborting this one.");
                return;
            }
            string name = "text";
            name = h.ToString(); 

            string fileRouteNodes = ScriptsFolder + @"\Vehicles\" + name + ".xml";


            if (File.Exists(fileRouteNodes))
            {
                name += " (" + DateTime.Now.GetHashCode() + ")";
            }


            File.AppendAllText(fileRouteNodes, "");


            XmlDocument xmlFile = new XmlDocument();


            XmlNode data = xmlFile.CreateNode(XmlNodeType.Element, "Data", null);
            xmlFile.AppendChild(data);

            XmlNode vehicle = xmlFile.CreateNode(XmlNodeType.Element, "Vehicle", null);


            XmlNode temp = xmlFile.CreateElement("Name");
            temp.InnerText = name;
            vehicle.AppendChild(temp);

            XmlNode vehicleClass = xmlFile.CreateElement("Class");
            vehicleClass.InnerText = ((VehicleClass)Function.Call<int>(Hash.GET_VEHICLE_CLASS_FROM_NAME, (int)h)).ToString();
            vehicle.AppendChild(vehicleClass);

            List<string> keywords = new List<string>();
            string nameAutotag = name;

            nameAutotag = nameAutotag.Replace(@"-", "");
            nameAutotag = nameAutotag.Replace(@"/", "");
            nameAutotag = nameAutotag.Replace(@" ", "");
            nameAutotag = nameAutotag.Replace(@"+", "");


            keywords.Add(vehicleClass.InnerText);
            keywords.Add(name);
            keywords.Add(nameAutotag);

            XmlNode keyw = xmlFile.CreateElement("Disciplines");
            foreach (string keyword in keywords)
            {
                XmlNode ktoadd = xmlFile.CreateElement("Discipline");
                ktoadd.InnerText = keyword.ToLowerInvariant();
                keyw.AppendChild(ktoadd);
            }

            vehicle.AppendChild(keyw);
            temp = xmlFile.CreateElement("Model");
            temp.InnerText = ((int)h).ToString();
            vehicle.AppendChild(temp);



            XmlAttribute vname = xmlFile.CreateAttribute("ModelName");
            vname.InnerText = h.ToString();
            temp.Attributes.Append(vname);


            data.AppendChild(vehicle);




            xmlFile.AppendChild(data);

            xmlFile.Save(ScriptsFolder + @"\Vehicles\" + name + ".xml");
            UI.ShowSubtitle("~b~Vehicle saved succesfully.~w~~n~Filename: ~g~" + name + ".xml");
        }
        
        
    }

}

