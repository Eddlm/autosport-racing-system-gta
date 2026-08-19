using GTA;
using GTA.Math;
using GTA.Native;
using System;

namespace ARS
{
    public class FreeCamController
    {
        readonly ARS _ars;
        bool _droneMode = true;
        bool _isActive = false;
        Vector3 _movement = Vector3.Zero;
        float _intendedHeight = 4f;

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
                _movement = Vector3.Zero;
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

            if (Game.IsControlJustPressed(2, Control.Jump))
            {
                if (_ars.TimeScale > 0.1f)
                {
                    if (_ars.TimeScale > 0.5f) Function.Call(Hash._START_SCREEN_EFFECT, "FocusOut", 500, false);
                    _ars.TimeScale = 0.001f;
                    _ars.IdealTimeScale = 0.001f;
                    Game.TimeScale = 0.001f;
                }
                else
                {
                    _ars.IdealTimeScale = 1.0f;
                    if (_ars.TimeScale < 0.1f) _ars.TimeScale = 0.1f;
                    Game.TimeScale = _ars.TimeScale;
                }
            }

            ARS.FreeCamRide.Position += _movement * ARS.Clamp(60 / Game.FPS, 1, 10);
            ARS.FreeCamRide.Rotation = new Vector3(0f, 0f, ARS.FreeCamRide.Rotation.Z);
            ARS.FreeCamRide.Rotation += new Vector3(0, 0, GameplayCamera.RelativeHeading);

            float speed = 0.01f;
            float heightAboveGround = ARS.FreeCamRide.HeightAboveGround;
            Game.Player.Character.Position = ARS.FreeCamRide.Position + new Vector3(0, 0, -1.5f);
            Game.Player.Character.Rotation = ARS.FreeCamRide.Rotation + new Vector3(0, 0, 180f);

            bool reduce = true;
            if (_movement.Length() < 7f)
            {
                if (!_droneMode) speed = 0.02f;
                if (Game.IsControlPressed(2, Control.Sprint)) speed += 0.05f;

                if (GameplayCamera.IsRendering)
                {
                    if (Game.IsControlJustPressed(2, Control.NextCamera))
                    {
                        _droneMode = !_droneMode;
                        if (_droneMode) _intendedHeight = ARS.FreeCamRide.HeightAboveGround;
                    }

                    if (Game.IsControlPressed(2, Control.Context)) _intendedHeight += 0.2f;
                    if (Game.IsControlPressed(2, Control.Cover)) _intendedHeight -= 0.2f;
                    if (Game.IsControlPressed(2, Control.VehicleAccelerate))
                    {
                        reduce = false;
                        _movement += (_droneMode ? Game.Player.Character.ForwardVector : GameplayCamera.Direction) * speed;
                    }
                    if (Game.IsControlPressed(2, Control.VehicleBrake))
                    {
                        reduce = false;
                        _movement -= (_droneMode ? Game.Player.Character.ForwardVector : GameplayCamera.Direction) * speed;
                    }
                    if (Game.IsControlPressed(2, Control.MoveLeftOnly))
                    {
                        reduce = false;
                        _movement += Game.Player.Character.RightVector * -speed;
                    }
                    if (Game.IsControlPressed(2, Control.MoveRightOnly))
                    {
                        reduce = false;
                        _movement += Game.Player.Character.RightVector * speed;
                    }
                }

                if (_droneMode)
                {
                    if (_intendedHeight < 2f) _intendedHeight = 2f;
                    if (heightAboveGround > 0f)
                    {
                        float difference = _intendedHeight - heightAboveGround;
                        if (Math.Abs(difference) > 1f)
                        {
                            _movement += new Vector3(0, 0, ARS.Clamp(difference / 200, -0.1f, 0.1f));
                            _movement.Z = ARS.Clamp(_movement.Z, -0.5f, 0.5f);
                        }
                        _movement.Z *= 0.9f;
                    }
                }
            }

            if (reduce) ApplyMovementDecay(speed, heightAboveGround);
        }

        public void Toggle()
        {
            if (_isActive)
            {
                _isActive = false;
                Game.Player.Character.Heading = ARS.FreeCamRide.Heading;
                Game.Player.Character.IsVisible = true;
                Game.Player.Character.HasGravity = true;
                Game.Player.Character.HasCollision = true;
                Game.Player.Character.Position = ARS.FreeCamRide.Position - new Vector3(0, 0, ARS.FreeCamRide.HeightAboveGround);
                if (_ars.IdealTimeScale != 1.0f)
                {
                    _ars.IdealTimeScale = 1.0f;
                    if (_ars.TimeScale < 0.1f) _ars.TimeScale = 0.1f;
                    Game.TimeScale = _ars.TimeScale;
                }
                return;
            }

            if (!ARS.CanWeUse(ARS.FreeCamRide)) return;
            if (Function.Call<int>(Hash.GET_FOLLOW_PED_CAM_VIEW_MODE) != 4)
                Function.Call(Hash.SET_FOLLOW_PED_CAM_VIEW_MODE, 4);

            _isActive = true;
            ARS.FreeCamRide.HasGravity = false;
            Game.Player.Character.HasGravity = false;
            Game.Player.Character.IsVisible = false;
            Game.Player.Character.HasCollision = false;
        }

        bool DrawInstructions(bool routeEditorActive, int routeNodeCount)
        {
            if (ARS.InstructionalScaleform == null || !ARS.InstructionalScaleform.IsLoaded)
            {
                ARS.InstructionalScaleform = new Scaleform("INSTRUCTIONAL_BUTTONS");
                UI.ShowSubtitle("~o~Scaleform not loaded", 500);
                return false;
            }

            ARS.InstructionalScaleform.CallFunction("CLEAR_ALL", true);
            ARS.InstructionalScaleform.CallFunction("CREATE_CONTAINER");
            if (routeEditorActive)
            {
                if (routeNodeCount > 0)
                {
                    if (Game.IsControlPressed(2, Control.Sprint))
                        SetInstruction(0, Control.Aim, "Delete last ten nodes");
                    else
                    {
                        SetInstruction(2, Control.Attack, "Apply Section");
                        SetInstruction(1, Control.Aim, "Delete last node");
                        SetInstruction(0, Control.Sprint, "More options");
                    }
                }
                else
                {
                    SetInstruction(2, Control.Attack, "Place Start Line");
                    SetInstruction(1, Control.WeaponWheelPrev, "Widen");
                    SetInstruction(0, Control.WeaponWheelNext, "Tighten");
                }
            }
            else
            {
                SetInstruction(0, Control.NextCamera, _droneMode ? "Freecam Mode" : "Drone Mode");
                SetInstruction(1, Control.Detonate, "Hide Hud");
                SetInstruction(2, Control.Jump, "Slow Mo");
            }
            ARS.InstructionalScaleform.CallFunction("DRAW_INSTRUCTIONAL_BUTTONS", 0);
            if (!ARS.HideHudMode) ARS.InstructionalScaleform.Render2D();
            return true;
        }

        void SetInstruction(int slot, Control control, string text)
        {
            ARS.InstructionalScaleform.CallFunction("SET_DATA_SLOT", slot,
                Function.Call<string>(Hash._0x0499D7B09FC9B407, 2, (int)control), text);
        }

        void ApplyMovementDecay(float speed, float heightAboveGround)
        {
            float multiplier = _droneMode && _movement.Length() > 0.2f ? 1 - (speed * 0.5f) : 1 - (speed * 2f);
            _movement.X *= multiplier;
            _movement.Y *= multiplier;
            if (!_droneMode || Math.Abs(heightAboveGround) < 1f) _movement.Z *= 1 - (speed * 2f);
        }
    }
}
