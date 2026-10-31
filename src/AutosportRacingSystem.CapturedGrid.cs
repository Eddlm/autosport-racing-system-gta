using GTA;
using GTA.Math;
using GTA.Native;
using LemonUI.Menus;
using System;
using System.Linq;
using System.Collections.Generic;

namespace ARS
{
    public partial class ARS
    {
        enum CapturedGridMode { Hybrid, CustomClones, CustomOnly, Standard }
        const int MaxCapturedGridCars = 10;
        CapturedGridMode _capturedGridMode = CapturedGridMode.Hybrid;
        NativeListItem<string> _gridRosterModeItem;
        readonly List<Vehicle> _capturedWorldCars = new List<Vehicle>();
        readonly Dictionary<int, Ped> _capturedDrivers = new Dictionary<int, Ped>();
        NativeMenu _capturedCarsMenu;
        Camera _capturedPreviewCamera;
        int _previewCarHandle = -1;
        bool _worldRosterWasChosen;

        // Do not call GET_VEHICLE_CLASS_FROM_NAME here: that native was added
        // in the previous patch and is not needed to select ordinary cars.
        // The ARS vehicle catalog already excludes military class 19.
        // This additional model-hash guard covers authored hardcoded rosters,
        // captured cars, and direct spawns without invoking a game native.
        bool IsRoadRaceCarModel(Model model)
        {
            if (!model.IsCar) return false;
            string[] blocked = { "rhino", "khanjali", "apc", "minitank",
                "scarab", "scarab2", "scarab3", "chernobog" };
            foreach (string name in blocked)
                if (model.Hash == new Model(name).Hash) return false;
            return true;
        }

        bool IsEligibleARSRosterModel(string key)
        {
            if (string.IsNullOrWhiteSpace(key)) return false;
            int hash;
            Model model = int.TryParse(key, out hash) ? new Model(hash) : new Model(key);
            return IsRoadRaceCarModel(model);
        }

        bool IsValidWorldCar(Vehicle car)
        {
            return car != null && car.Exists() && IsRoadRaceCarModel(car.Model);
        }

        // Keep the user's existing INI settings working after the menu migration:
        // CustomOnly now uses the same cloning behavior as CustomClones.
        CapturedGridMode EffectiveCapturedGridMode
        {
            get { return _capturedGridMode == CapturedGridMode.CustomOnly ? CapturedGridMode.CustomClones : _capturedGridMode; }
        }

        // Same model-level PI inputs as TryComputePlayerCarPaceIndex; used only when
        // there are more captured candidates than available grid slots.
        bool TryGetCapturedCarPace(Vehicle car, out float pace)
        {
            pace = 0f;
            if (!IsValidWorldCar(car)) return false;
            if (ModelPaceIndexCache.TryGetValue(car.Model.Hash.ToString(), out pace)) return true;
            Model model = car.Model;
            float grip = Function.Call<float>((Hash)0x539DE94D44FDFD0D, model.Hash);
            float topSpeedMph = MpsToMph(Function.Call<float>((Hash)0xF417C2502FFFED43, model.Hash));
            if (grip <= 0f || topSpeedMph <= 0f) return false;
            float accel = Function.Call<float>(Hash.GET_VEHICLE_MODEL_ACCELERATION, model.Hash);
            pace = ComputePaceIndex(topSpeedMph, grip, accel, IsElectricModel(model.Hash));
            return !float.IsNaN(pace) && !float.IsInfinity(pace);
        }

        void CaptureNearbyWorldCars()
        {
            if (Racers.Count > 0 || RaceStatus != RaceState.None)
            {
                UI.Notify("~o~End the current race/grid before capturing cars.");
                return;
            }
            Ped player = Game.Player.Character;
            if (player == null || !player.Exists()) return;
            ReleaseUnusedCapturedCars();
            int limit = MaxCapturedGridCars;
            if (_intendedOpponents <= 0) { UI.Notify("~o~Set Target Grid Size above zero first."); return; }
            Vehicle playerCar = player.CurrentVehicle;
            HashSet<int> reservedPeds = new HashSet<int>();
            foreach (Vehicle car in World.GetNearbyVehicles(player.Position, 100f)
                .Where(v => IsValidWorldCar(v) && (playerCar == null || v.Handle != playerCar.Handle))
                .OrderBy(v => v.Position.DistanceTo(player.Position)))
            {
                if (_capturedWorldCars.Count >= limit) break;
                if (_capturedWorldCars.Any(v => v.Handle == car.Handle)) continue;
                Ped driver = car.GetPedOnSeat(VehicleSeat.Driver);
                if (driver != null && driver.Exists())
                {
                    if (driver.IsPlayer || driver.IsDead || reservedPeds.Contains(driver.Handle)) continue;
                }
                else
                {
                    // Prefer an authored ped standing by the car, but never claim another captured car's actor.
                    driver = World.GetNearbyPeds(car.Position, 7f)
                        .Where(ped => ped != null && ped.Exists() && !ped.IsPlayer && !ped.IsDead &&
                            !ped.IsInVehicle() && !reservedPeds.Contains(ped.Handle))
                        .OrderBy(ped => ped.Position.DistanceTo(car.Position))
                        .FirstOrDefault();
                }
                car.IsPersistent = true;
                _capturedWorldCars.Add(car);
                if (driver != null && driver.Exists())
                {
                    driver.IsPersistent = true;
                    _capturedDrivers[car.Handle] = driver;
                    reservedPeds.Add(driver.Handle);
                }
            }
            _worldRosterWasChosen = _capturedWorldCars.Count > 0;
            RefreshCapturedCarsMenu();
            UI.Notify("~b~[ARS]:~w~ Selected " + _capturedWorldCars.Count + " / " + limit +
                " cars. Review them in Race > Captured Cars.");
        }

        void RefreshCapturedCarsMenu()
        {
            if (_capturedCarsMenu == null) return;
            _capturedCarsMenu.Clear();
            for (int i = 0; i < _capturedWorldCars.Count; i++)
            {
                Vehicle car = _capturedWorldCars[i];
                string name = IsValidWorldCar(car) ? car.DisplayName : "[Missing vehicle]";
                bool hasActor = IsValidWorldCar(car) && _capturedDrivers.ContainsKey(car.Handle);
                NativeItem item = new NativeItem((i + 1) + ". " + name,
                    hasActor ? "Existing driver/ped selected. Delete removes car from race." : "ARS will create a random driver. Delete removes car from race.");
                _capturedCarsMenu.Add(item);
            }
            if (_capturedWorldCars.Count == 0)
                _capturedCarsMenu.Add(new NativeItem("No selected cars", "Use Capture Nearby Cars in the Race menu."));
            _previewCarHandle = -1;
        }

        void RemoveSelectedCapturedCar()
        {
            if (_capturedCarsMenu == null || _capturedWorldCars.Count == 0) return;
            int index = _capturedCarsMenu.SelectedIndex;
            if (index < 0 || index >= _capturedWorldCars.Count) return;
            Vehicle car = _capturedWorldCars[index];
            if (IsValidWorldCar(car))
            {
                Ped actor;
                if (_capturedDrivers.TryGetValue(car.Handle, out actor) && actor != null && actor.Exists()) actor.IsPersistent = false;
                _capturedDrivers.Remove(car.Handle);
                car.IsPersistent = false;
            }
            _capturedWorldCars.RemoveAt(index);
            RefreshCapturedCarsMenu();
            if (_capturedCarsMenu.Items.Count > 0)
                _capturedCarsMenu.SelectedIndex = Math.Min(index, _capturedCarsMenu.Items.Count - 1);
            UI.Notify("~b~[ARS]:~w~ Race grid now has " + _capturedWorldCars.Count + " selected cars.");
        }

        void StopCapturedCarPreview()
        {
            if (_capturedPreviewCamera != null)
            {
                if (World.RenderingCamera == _capturedPreviewCamera) World.RenderingCamera = null;
                try { _capturedPreviewCamera.Destroy(); } catch (Exception) { }
                _capturedPreviewCamera = null;
            }
            _previewCarHandle = -1;
        }

        void UpdateCapturedCarPreview()
        {
            if (_capturedCarsMenu == null || !_capturedCarsMenu.Visible || _freeCam.IsActive)
            {
                StopCapturedCarPreview();
                return;
            }
            int index = _capturedCarsMenu.SelectedIndex;
            if (index < 0 || index >= _capturedWorldCars.Count) { StopCapturedCarPreview(); return; }
            Vehicle car = _capturedWorldCars[index];
            if (!IsValidWorldCar(car)) { StopCapturedCarPreview(); return; }
            if (_capturedPreviewCamera == null || _previewCarHandle != car.Handle)
            {
                StopCapturedCarPreview();
                // Film-style three-quarter view from the car's front-left, no entity movement.
                Vector3 cameraPos = car.Position + car.ForwardVector * 7f - car.RightVector * 5f + new Vector3(0, 0, 3.2f);
                _capturedPreviewCamera = World.CreateCamera(cameraPos, Vector3.Zero, 48f);
                if (_capturedPreviewCamera != null)
                {
                    _capturedPreviewCamera.PointAt(car);
                    World.RenderingCamera = _capturedPreviewCamera;
                    _previewCarHandle = car.Handle;
                }
            }
            // Native GTA chevron-style marker, only while the selection menu is visible.
            Vector3 pos = car.Position + new Vector3(0f, 0f, 2.6f);
            Function.Call(Hash.DRAW_MARKER, 2, pos.X, pos.Y, pos.Z,
                0f, 0f, 0f, 0f, 180f, 0f, 0.8f, 0.8f, 0.8f,
                255, 190, 0, 210, false, true, 2, false, 0, 0, false);
        }

        void ReleaseUnusedCapturedCars()
        {
            StopCapturedCarPreview();
            foreach (Vehicle car in _capturedWorldCars)
            {
                if (!IsValidWorldCar(car)) continue;
                if (Racers.Any(r => r.Car != null && r.Car.Handle == car.Handle)) continue;
                car.IsPersistent = false;
            }
            foreach (Ped actor in _capturedDrivers.Values)
                if (actor != null && actor.Exists() && !Racers.Any(r => r.Driver != null && r.Driver.Handle == actor.Handle))
                    actor.IsPersistent = false;
            _capturedDrivers.Clear();
            _capturedWorldCars.Clear();
            _worldRosterWasChosen = false;
        }


void CopyCapturedCarAppearance(Vehicle source, Vehicle clone)
            {
                clone.PrimaryColor = source.PrimaryColor;
                clone.SecondaryColor = source.SecondaryColor;
                clone.PearlescentColor = source.PearlescentColor;
                clone.RimColor = source.RimColor;
                clone.WheelType = source.WheelType;
                for (int slot = 0; slot <= 49; slot++)
                {
                    try
                    {
                        VehicleMod part = (VehicleMod)slot;
                        int index = source.GetMod(part);
                        if (index >= 0 && clone.GetModCount(part) > index)
                            clone.SetMod(part, index, false);
                    }
                    catch (Exception) { /* Unavailable DLC/custom slot. */ }
                }
                // GTA toggles: turbo, tyre smoke, xenon. Preserve their on/off state.
                foreach (int toggle in new[] { 18, 20, 22 })
                {
                    try
                    {
                        bool enabled = Function.Call<bool>((Hash)0x84B233A8C8FC8AE7, source, toggle); // IS_TOGGLE_MOD_ON
                        Function.Call((Hash)0x2A1F4F37F95BAD08, clone, toggle, enabled); // TOGGLE_VEHICLE_MOD
                    }
                    catch (Exception) { }
                }
                // Randomization is automatic and does not alter performance parts.
                VehicleColor body = _randomColors[GetRandomInt(0, _randomColors.Length)];
                VehicleColor secondary = _randomColors[GetRandomInt(0, _randomColors.Length)];
                if (body == source.PrimaryColor && _randomColors.Length > 1)
                    body = _randomColors[(Array.IndexOf(_randomColors, body) + 1) % _randomColors.Length];
                clone.PrimaryColor = body;
                clone.SecondaryColor = secondary;
                clone.PearlescentColor = body;

                // Only recolor neon already installed on the template; do not install
                // neon on cars that did not have it. Native numbers support SHVDN v2.
                try
                {
                    bool hasNeon = false;
                    for (int side = 0; side < 4; side++)
                    {
                        bool enabled = Function.Call<bool>((Hash)0x8C4B92553E4766A5, source, side); // _IS_VEHICLE_NEON_LIGHT_ENABLED
                        if (!enabled) continue;
                        hasNeon = true;
                        Function.Call((Hash)0x2AA720E4287BF269, clone, side, true); // _SET_VEHICLE_NEON_LIGHT_ENABLED
                    }
                    if (hasNeon)
                    {
                        int[,] palette = { { 30, 165, 255 }, { 255, 55, 170 }, { 65, 255, 130 }, { 255, 185, 25 }, { 160, 80, 255 }, { 255, 45, 45 } };
                        int pick = GetRandomInt(0, palette.GetLength(0));
                        Function.Call((Hash)0x8E0A58289A6BAAAB, clone, palette[pick, 0], palette[pick, 1], palette[pick, 2]);
                    }
                }
                catch (Exception ex) { Log(LogImportance.Info, "Optional clone neon copy: " + ex.Message); }
            }

        List<Vehicle> AddWorldCarsToGrid(int maxcars, Action<Vehicle, Ped> addRacer, Func<Vehicle, Ped> createDriver)
        {
            List<Vehicle> chosen = _worldRosterWasChosen
                ? _capturedWorldCars.Where(IsValidWorldCar).Where(v => Game.Player.Character.CurrentVehicle == null || v.Handle != Game.Player.Character.CurrentVehicle.Handle).ToList()
                : new List<Vehicle>();
            if (chosen.Count > maxcars)
            {
                float playerPace;
                if (TryComputePlayerCarPaceIndex(out playerPace))
                    chosen = chosen.OrderBy(v =>
                    {
                        float pace;
                        return TryGetCapturedCarPace(v, out pace) ? Math.Abs(pace - playerPace) : float.MaxValue;
                    }).ThenBy(v => v.Handle).Take(maxcars).ToList();
                else chosen = chosen.Take(maxcars).ToList();
            }
            List<Vehicle> accepted = new List<Vehicle>();
            foreach (Vehicle car in chosen)
            {
                Ped driver;
                bool borrowed = _capturedDrivers.TryGetValue(car.Handle, out driver)
                    && driver != null && driver.Exists() && !driver.IsDead && !driver.IsPlayer;
                if (!borrowed)
                {
                    Ped occupant = car.GetPedOnSeat(VehicleSeat.Driver);
                    borrowed = occupant != null && occupant.Exists() && !occupant.IsDead && !occupant.IsPlayer;
                    driver = borrowed ? occupant : createDriver(car);
                }
                if (driver == null || !driver.Exists()) continue;
                int before = Racers.Count;
                addRacer(car, driver);
                if (Racers.Count <= before) continue;
                Racer r = Racers[Racers.Count - 1];
                r.WorldCarBorrowed = true;
                r.WorldDriverBorrowed = borrowed;
                accepted.Add(car);
            }
            return accepted;
        }

        void FillWorldCarClones(int maxcars, IList<Vehicle> originals, Action<Vehicle, Ped> addRacer, Func<Vehicle, Ped> createDriver)
        {
            if (originals.Count == 0) return;
            int failures = 0;
            int cycle = 0;
            List<Vehicle> templates = new List<Vehicle>(originals);
            while (Racers.Count < maxcars && failures < originals.Count * 2)
            {
                if (cycle % templates.Count == 0)
                {
                    for (int k = templates.Count - 1; k > 0; k--)
                    {
                        int j = GetRandomInt(0, k + 1);
                        Vehicle swap = templates[k]; templates[k] = templates[j]; templates[j] = swap;
                    }
                }
                Vehicle source = templates[cycle % templates.Count];
                cycle++;
                Vehicle clone = null;
                Ped driver = null;
                try
                {
                    int before = Racers.Count;
                    clone = World.CreateVehicle(source.Model, RouteNodes[(Racers.Count + 1) * 10]);
                    if (clone == null || !clone.Exists()) { failures++; continue; }
                    clone.Heading = (RouteNodes[2] - RouteNodes[0]).ToHeading();
                    clone.InstallModKit();
                    CopyCapturedCarAppearance(source, clone);
                    driver = createDriver(clone);
                    if (driver == null || !driver.Exists()) { clone.Delete(); failures++; continue; }
                    addRacer(clone, driver);
                    if (Racers.Count > before) { Racers[Racers.Count - 1].WorldClone = true; failures = 0; }
                    else { driver.Delete(); clone.Delete(); failures++; }
                }
                catch (Exception ex)
                {
                    Log(LogImportance.Error, "Custom clone failed: " + ex.Message);
                    if (driver != null && driver.Exists()) driver.Delete();
                    if (clone != null && clone.Exists()) clone.Delete();
                    failures++;
                }
            }
        }

    }
}
