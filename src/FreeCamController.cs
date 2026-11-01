using GTA;
using GTA.Math;
using GTA.Native;
using System;

namespace ARS
{
    public class FreeCamController
    {
        readonly ARS _ars;
        bool _isActive = false;
        int _previousViewMode = -1;

        public FreeCamController(ARS ars)
        {
            _ars = ars;
        }

        public bool IsActive => _isActive;

        public void Update(bool routeEditorActive, int routeNodeCount)
        {
            if (World.RenderingCamera == null && ARS.HideHudMode) ARS.HideHudMode = false;
            if (!ARS.CanWeUse(ARS.FreeCamRide)) return;

            if (!_isActive && ARS.FreeCamRide.Position.DistanceTo(Game.Player.Character.Position) > 100f)
                ARS.FreeCamRide.Position = Game.Player.Character.Position + new Vector3(0, 0, 20);

            if (!_isActive)
            {
                return;
            }

            // The mode writes the ped's position every active frame, so anything the game does to that ped is undone next tick: a
            // player who dies, respawns or gets into a car during freecam is dragged back under the drone, and a mounted ped is warped
            // out of its vehicle. Leaving is the only safe answer, and Toggle restores gravity, collision, visibility and the view mode.
            Ped ped = Game.Player.Character;
            if (ped == null || !ped.Exists() || ped.IsDead || ped.IsInVehicle())
            {
                Toggle();
                return;
            }

            Function.Call(Hash.HIDE_HUD_AND_RADAR_THIS_FRAME, true);
            if (Game.IsControlJustPressed(2, Control.Detonate)) ARS.HideHudMode = !ARS.HideHudMode;

            if (!DrawInstructions(routeEditorActive, routeNodeCount)) return;

            Game.DisableControlThisFrame(0, Control.Attack);
            Game.DisableControlThisFrame(0, Control.Aim);
            Game.DisableControlThisFrame(0, Control.NextCamera);
            Game.DisableControlThisFrame(0, Control.Sprint);
            Game.DisableControlThisFrame(0, Control.Jump);
            Game.DisableControlThisFrame(0, Control.Phone);
            if (routeEditorActive)
                Game.DisableControlThisFrame(0, Control.FrontendPause);

            if (Game.IsControlJustPressed(2, Control.Jump))
            {
                if (_ars.TimeScale < 1.0f)
                {
                    _ars.TimeScale = 1.0f;
                    Game.TimeScale = 1.0f;
                }
                else
                {
                    Function.Call(Hash._START_SCREEN_EFFECT, "FocusOut", 500, false);
                    _ars.TimeScale = 0.5f;
                    Game.TimeScale = 0.5f;
                }
            }

            ARS.FreeCamRide.Rotation = new Vector3(0f, 0f, ARS.FreeCamRide.Rotation.Z);
            ARS.FreeCamRide.Rotation += new Vector3(0, 0, GameplayCamera.RelativeHeading);

            Game.Player.Character.Position = ARS.FreeCamRide.Position + new Vector3(0, 0, -1.5f);
            Game.Player.Character.Rotation = ARS.FreeCamRide.Rotation + new Vector3(0, 0, 180f);

            if (GameplayCamera.IsRendering)
            {
                Vector3 forward = GameplayCamera.Direction;
                forward.Z = 0f;
                if (forward.Length() > 0.001f) forward.Normalize();
                Vector3 right = Vector3.Cross(forward, Vector3.WorldUp);
                Vector3 input = Vector3.Zero;
                if (Game.IsControlPressed(2, Control.MoveUpOnly)) input += forward;
                if (Game.IsControlPressed(2, Control.MoveDownOnly)) input -= forward;
                if (Game.IsControlPressed(2, Control.MoveRightOnly)) input += right;
                if (Game.IsControlPressed(2, Control.MoveLeftOnly)) input -= right;
                if (Game.IsControlPressed(2, Control.Context)) input += Vector3.WorldUp;
                if (Game.IsControlPressed(2, Control.Cover)) input -= Vector3.WorldUp;
                if (input.Length() > 0.001f)
                {
                    input.Normalize();
                    float seconds = 1f / Math.Max(1f, Game.FPS);
                    float metresPerSecond = Game.IsControlPressed(2, Control.Sprint) ? 45f : 12f;
                    ARS.FreeCamRide.Position += input * metresPerSecond * Math.Min(seconds, 0.1f);
                }
            }
        }

        public void Toggle()
        {
            if (_isActive)
            {
                _isActive = false;
                // The ride borrows the player's first-person camera, so the player's own mode has to come back.
                if (_previousViewMode >= 0)
                {
                    Function.Call(Hash.SET_FOLLOW_PED_CAM_VIEW_MODE, _previousViewMode);
                    _previousViewMode = -1;
                }
                Game.Player.Character.Heading = ARS.FreeCamRide.Heading;
                Game.Player.Character.IsVisible = true;
                Game.Player.Character.HasGravity = true;
                Game.Player.Character.HasCollision = true;
                Game.Player.Character.Position = ARS.FreeCamRide.Position - new Vector3(0, 0, ARS.FreeCamRide.HeightAboveGround);
                _ars.TimeScale = 1.0f;
                Game.TimeScale = 1.0f;
                return;
            }

            if (!ARS.CanWeUse(ARS.FreeCamRide)) return;
            _previousViewMode = Function.Call<int>(Hash.GET_FOLLOW_PED_CAM_VIEW_MODE);
            // Forced only when the reading is a real mode: a -1 cannot be restored on the way out, so forcing 4 over it would leave
            // the player's camera on the drone's mode for good.
            if (_previousViewMode >= 0 && _previousViewMode != 4)
                Function.Call(Hash.SET_FOLLOW_PED_CAM_VIEW_MODE, 4);

            _isActive = true;
            ARS.FreeCamRide.HasGravity = false;
            Game.Player.Character.HasGravity = false;
            Game.Player.Character.IsVisible = false;
            Game.Player.Character.HasCollision = false;
        }

        bool DrawInstructions(bool routeEditorActive, int routeNodeCount)
        {
            if (ARS.InstructionalScaleform == null) ARS.InstructionalScaleform = new Scaleform("INSTRUCTIONAL_BUTTONS");
            if (!ARS.InstructionalScaleform.IsLoaded) return false;

            ARS.InstructionalScaleform.CallFunction("CLEAR_ALL", true);
            ARS.InstructionalScaleform.CallFunction("CREATE_CONTAINER");
            if (routeEditorActive)
            {
                // Keep a persistent exit hint. Shift has no alternate mouse action:
                // it only speeds up camera movement.
                if (routeNodeCount > 0)
                {
                    SetInstruction(0, Control.Sprint, "Shift: Fly faster / More options");
                    SetInstruction(1, Control.Aim, "RMB: Undo / hold to rewind");
                    SetInstruction(2, Control.Attack, "LMB: Apply section");
                }
                else
                {
                    SetInstruction(0, Control.Sprint, "Shift: Fly faster / More options");
                    SetInstruction(1, Control.WeaponWheelPrev, "Widen / Narrow");
                    SetInstruction(2, Control.Attack, "LMB: Place start line");
                }
            }
            else
            {
                SetInstruction(0, Control.Context, "Up / Cover: Down");
                SetInstruction(1, Control.Sprint, "Fast movement");
                SetInstruction(2, Control.Detonate, "Hide Hud");
            }
            if (routeEditorActive)
                SetInstruction(3, Control.FrontendCancel, "ESC / BACKSPACE: EXIT (discard route)");
            ARS.InstructionalScaleform.CallFunction("DRAW_INSTRUCTIONAL_BUTTONS", 0);
            if (!ARS.HideHudMode) ARS.InstructionalScaleform.Render2D();
            return true;
        }

        void SetInstruction(int slot, Control control, string text)
        {
            ARS.InstructionalScaleform.CallFunction("SET_DATA_SLOT", slot,
                Function.Call<string>(Hash._0x0499D7B09FC9B407, 2, (int)control), text);
        }

    }
}
