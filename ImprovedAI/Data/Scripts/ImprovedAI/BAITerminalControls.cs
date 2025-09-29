using BetterAIConstructor.Config;
using Sandbox.ModAPI;
using Sandbox.ModAPI.Interfaces.Terminal;
using System;
using System.Collections.Generic;
using System.Text;
using VRage.Utils;
using VRageMath;

namespace BetterAIConstructor.UI
{
    public static class TerminalControls
    {
        private static bool _initialized = false;
        private static bool _controlsCreated = false;
        private static List<IMyTerminalControl> _controls = new List<IMyTerminalControl>();
        private static List<IMyTerminalAction> _actions = new List<IMyTerminalAction>();

        public static void Initialize()
        {
            if (_initialized) return;
            _initialized = true;

            MyAPIGateway.TerminalControls.CustomControlGetter += CustomControlGetter;
            MyAPIGateway.TerminalControls.CustomActionGetter += CustomActionGetter;
        }

        public static void CreateControls()
        {
            if (_controlsCreated) return;
            _controlsCreated = true;

            try
            {
                // Create all our custom controls
                CreateEnableControl();
                CreateSeparator("Navigation Settings");
                CreateWaypointToleranceControl();
                CreateMaxApproachSpeedControl();
                CreateMaxTravelSpeedControl();

                CreateSeparator("Display Settings");
                CreateTextPanelMatchNameControl();

                CreateSeparator("Construction Settings");
                CreateRepairDamagedBlocksControl();
                CreateWeldUnfinishedBlocksControl();
                CreateWeldProjectedBlocksControl();
                CreateGrindBlocksControl();

                CreateSeparator("Gravity Alignment");
                CreateAlignToPGravityControl();
                CreatePGravityAlignMaxPitchControl();
                CreatePGravityAlignMaxRollControl();

                CreateSeparator("Power Monitoring");
                CreateMonitorHydrogenLevelsControl();
                CreateHydrogenRefuelThresholdControl();
                CreateHydrogenOperationalThresholdControl();
                CreateMonitorBatteryLevelsControl();
                CreateBatteryRefuelThresholdControl();
                CreateBatteryOperationalThresholdControl();

                CreateSeparator("Communication");
                CreateBroadcastUpdatesControl();

                // Create actions
                CreateToggleEnableAction();
            }
            catch (Exception ex)
            {
                MyAPIGateway.Utilities.ShowMessage("BetterAI_Constructor", $"Error creating controls: {ex.Message}");
                MyLog.Default.WriteLine($"BetterAI_Constructor: CreateControls exception: {ex}");
            }
        }

        private static BAIConstructorBlockLogic GetAIConstructorBlock(IMyTerminalBlock block)
        {
            try
            {
                if (block?.GameLogic != null)
                {
                    return block.GameLogic.GetAs<BAIConstructorBlockLogic>();
                }
            }
            catch (Exception ex)
            {
                MyLog.Default.WriteLine($"BetterAI_Constructor: GetAIConstructorBlock exception: {ex}");
            }
            return null;
        }

        private static bool IsAIConstructorBlock(IMyTerminalBlock block)
        {
            return block != null &&
                   (block.BlockDefinition.SubtypeName == "BetterAIConstructorSmall" ||
                    block.BlockDefinition.SubtypeName == "BetterAIConstructorLarge");
        }

        #region Control Creation Methods

        private static void CreateSeparator(string title)
        {
            var separator = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSeparator, IMyRemoteControl>($"BetterAI_Sep_{title.Replace(" ", "")}");
            separator.Visible = IsAIConstructorBlock;
            separator.Enabled = IsAIConstructorBlock;
            _controls.Add(separator);
        }

        private static void CreateEnableControl()
        {
            var control = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlCheckbox, IMyRemoteControl>("BetterAI_Enable");
            control.Title = MyStringId.GetOrCompute("Enable Construction AI");
            control.Tooltip = MyStringId.GetOrCompute("Enable/disable the construction AI system");
            control.Getter = (block) => GetAIConstructorBlock(block)?.IsEnabled ?? false;
            control.Setter = (block, value) => GetAIConstructorBlock(block)?.SetEnabled(value);
            control.Visible = IsAIConstructorBlock;
            control.Enabled = IsAIConstructorBlock;
            _controls.Add(control);
        }

        private static void CreateWaypointToleranceControl()
        {
            var control = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSlider, IMyRemoteControl>("BetterAI_WaypointTolerance");
            control.Title = MyStringId.GetOrCompute("Waypoint Tolerance");
            control.Tooltip = MyStringId.GetOrCompute("Distance tolerance for reaching waypoints (meters)");
            control.Getter = (block) => GetAIConstructorBlock(block)?.GetConfig()?.WaypointTolerance ?? 2.5f;
            control.Setter = (block, value) =>
            {
                var aiBlock = GetAIConstructorBlock(block);
                if (aiBlock != null)
                {
                    var config = aiBlock.GetConfig();
                    config.WaypointTolerance = MathHelper.Clamp(value, BAIConstructorConfig.MIN_WAYPOINT_TOLERANCE, BAIConstructorConfig.MAX_WAYPOINT_TOLERANCE);
                    aiBlock.SaveConfig();
                }
            };
            control.Visible = IsAIConstructorBlock;
            control.Writer = (block, result) => result.Append($"{GetAIConstructorBlock(block)?.GetConfig()?.WaypointTolerance ?? 2.5f:F1}m");
            control.SetLimits(BAIConstructorConfig.MIN_WAYPOINT_TOLERANCE, BAIConstructorConfig.MAX_WAYPOINT_TOLERANCE);
            _controls.Add(control);
        }

        private static void CreateMaxApproachSpeedControl()
        {
            var control = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSlider, IMyRemoteControl>("BetterAI_MaxApproachSpeed");
            control.Title = MyStringId.GetOrCompute("Max Approach Speed");
            control.Tooltip = MyStringId.GetOrCompute("Maximum speed when approaching targets (m/s)");
            control.Getter = (block) => GetAIConstructorBlock(block)?.GetConfig()?.MaxApproachSpeed ?? 5.0f;
            control.Setter = (block, value) =>
            {
                var aiBlock = GetAIConstructorBlock(block);
                if (aiBlock != null)
                {
                    var config = aiBlock.GetConfig();
                    config.MaxApproachSpeed = MathHelper.Clamp(value, BAIConstructorConfig.MIN_SPEED, BAIConstructorConfig.MAX_APPROACH_SPEED);
                    aiBlock.SaveConfig();
                }
            };
            control.Visible = IsAIConstructorBlock;
            control.Writer = (block, result) => result.Append($"{GetAIConstructorBlock(block)?.GetConfig()?.MaxApproachSpeed ?? 5.0f:F1} m/s");
            control.SetLimits(BAIConstructorConfig.MIN_SPEED, BAIConstructorConfig.MAX_APPROACH_SPEED);
            _controls.Add(control);
        }

        private static void CreateMaxTravelSpeedControl()
        {
            var control = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSlider, IMyRemoteControl>("BetterAI_MaxTravelSpeed");
            control.Title = MyStringId.GetOrCompute("Max Travel Speed");
            control.Tooltip = MyStringId.GetOrCompute("Maximum speed when traveling between distant waypoints (m/s)");
            control.Getter = (block) => GetAIConstructorBlock(block)?.GetConfig()?.MaxTravelSpeed ?? 20.0f;
            control.Setter = (block, value) =>
            {
                var aiBlock = GetAIConstructorBlock(block);
                if (aiBlock != null)
                {
                    var config = aiBlock.GetConfig();
                    config.MaxTravelSpeed = MathHelper.Clamp(value, BAIConstructorConfig.MIN_SPEED, BAIConstructorConfig.MAX_TRAVEL_SPEED);
                    aiBlock.SaveConfig();
                }
            };
            control.Visible = IsAIConstructorBlock;
            control.Writer = (block, result) => result.Append($"{GetAIConstructorBlock(block)?.GetConfig()?.MaxTravelSpeed ?? 20.0f:F1} m/s");
            control.SetLimits(BAIConstructorConfig.MIN_SPEED, BAIConstructorConfig.MAX_TRAVEL_SPEED);
            _controls.Add(control);
        }

        private static void CreateTextPanelMatchNameControl()
        {
            var control = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlTextbox, IMyRemoteControl>("BetterAI_TextPanelMatchName");
            control.Title = MyStringId.GetOrCompute("Text Panel Match Name");
            control.Tooltip = MyStringId.GetOrCompute("Text to match in LCD panel names for status display");
            control.Getter = (block) => new StringBuilder(GetAIConstructorBlock(block)?.GetConfig()?.TextPanelMatchName ?? "[BetterAI_Constructor]");
            control.Setter = (block, value) =>
            {
                var aiBlock = GetAIConstructorBlock(block);
                if (aiBlock != null)
                {
                    var config = aiBlock.GetConfig();
                    config.TextPanelMatchName = value.ToString() ?? "[BetterAI_Constructor]";
                    aiBlock.SaveConfig();
                }
            };
            control.Visible = IsAIConstructorBlock;
            _controls.Add(control);
        }

        private static void CreateRepairDamagedBlocksControl()
        {
            var control = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlCheckbox, IMyRemoteControl>("BetterAI_RepairDamagedBlocks");
            control.Title = MyStringId.GetOrCompute("Repair Damaged Blocks");
            control.Tooltip = MyStringId.GetOrCompute("Enable repairing of damaged blocks");
            control.Getter = (block) => GetAIConstructorBlock(block)?.GetConfig()?.RepairDamagedBlocks ?? true;
            control.Setter = (block, value) =>
            {
                var aiBlock = GetAIConstructorBlock(block);
                if (aiBlock != null)
                {
                    var config = aiBlock.GetConfig();
                    config.RepairDamagedBlocks = value;
                    aiBlock.SaveConfig();
                }
            };
            control.Visible = IsAIConstructorBlock;
            _controls.Add(control);
        }

        private static void CreateWeldUnfinishedBlocksControl()
        {
            var control = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlCheckbox, IMyRemoteControl>("BetterAI_WeldUnfinishedBlocks");
            control.Title = MyStringId.GetOrCompute("Weld Unfinished Blocks");
            control.Tooltip = MyStringId.GetOrCompute("Enable welding of incomplete blocks");
            control.Getter = (block) => GetAIConstructorBlock(block)?.GetConfig()?.WeldUnfinishedBlocks ?? true;
            control.Setter = (block, value) =>
            {
                var aiBlock = GetAIConstructorBlock(block);
                if (aiBlock != null)
                {
                    var config = aiBlock.GetConfig();
                    config.WeldUnfinishedBlocks = value;
                    aiBlock.SaveConfig();
                }
            };
            control.Visible = IsAIConstructorBlock;
            _controls.Add(control);
        }

        private static void CreateWeldProjectedBlocksControl()
        {
            var control = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlCheckbox, IMyRemoteControl>("BetterAI_WeldProjectedBlocks");
            control.Title = MyStringId.GetOrCompute("Weld Projected Blocks");
            control.Tooltip = MyStringId.GetOrCompute("Enable welding of projected/blueprint blocks");
            control.Getter = (block) => GetAIConstructorBlock(block)?.GetConfig()?.WeldProjectedBlocks ?? false;
            control.Setter = (block, value) =>
            {
                var aiBlock = GetAIConstructorBlock(block);
                if (aiBlock != null)
                {
                    var config = aiBlock.GetConfig();
                    config.WeldProjectedBlocks = value;
                    aiBlock.SaveConfig();
                }
            };
            control.Visible = IsAIConstructorBlock;
            _controls.Add(control);
        }

        private static void CreateGrindBlocksControl()
        {
            var control = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlCheckbox, IMyRemoteControl>("BetterAI_GrindBlocks");
            control.Title = MyStringId.GetOrCompute("Grind Blocks");
            control.Tooltip = MyStringId.GetOrCompute("Enable grinding/deconstruction of blocks");
            control.Getter = (block) => GetAIConstructorBlock(block)?.GetConfig()?.GrindBlocks ?? false;
            control.Setter = (block, value) =>
            {
                var aiBlock = GetAIConstructorBlock(block);
                if (aiBlock != null)
                {
                    var config = aiBlock.GetConfig();
                    config.GrindBlocks = value;
                    aiBlock.SaveConfig();
                }
            };
            control.Visible = IsAIConstructorBlock;
            _controls.Add(control);
        }

        private static void CreateAlignToPGravityControl()
        {
            var control = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlCheckbox, IMyRemoteControl>("BetterAI_AlignToPGravity");
            control.Title = MyStringId.GetOrCompute("Align to Planetary Gravity");
            control.Tooltip = MyStringId.GetOrCompute("Enable automatic alignment to planetary gravity");
            control.Getter = (block) => GetAIConstructorBlock(block)?.GetConfig()?.AlignToPGravity ?? true;
            control.Setter = (block, value) =>
            {
                var aiBlock = GetAIConstructorBlock(block);
                if (aiBlock != null)
                {
                    var config = aiBlock.GetConfig();
                    config.AlignToPGravity = value;
                    aiBlock.SaveConfig();
                }
            };
            control.Visible = IsAIConstructorBlock;
            _controls.Add(control);
        }

        private static void CreatePGravityAlignMaxPitchControl()
        {
            var control = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSlider, IMyRemoteControl>("BetterAI_PGravityAlignMaxPitch");
            control.Title = MyStringId.GetOrCompute("Max Pitch Angle");
            control.Tooltip = MyStringId.GetOrCompute("Maximum pitch angle when aligning to gravity (degrees)");
            control.Getter = (block) => GetAIConstructorBlock(block)?.GetConfig()?.PGravityAlignMaxPitch ?? 10.0f;
            control.Setter = (block, value) =>
            {
                var aiBlock = GetAIConstructorBlock(block);
                if (aiBlock != null)
                {
                    var config = aiBlock.GetConfig();
                    config.PGravityAlignMaxPitch = MathHelper.Clamp(value, BAIConstructorConfig.MIN_PITCH_ROLL, BAIConstructorConfig.MAX_PITCH_ROLL);
                    aiBlock.SaveConfig();
                }
            };
            control.Visible = IsAIConstructorBlock;
            control.Writer = (block, result) => result.Append($"{GetAIConstructorBlock(block)?.GetConfig()?.PGravityAlignMaxPitch ?? 10.0f:F1}°");
            control.SetLimits(BAIConstructorConfig.MIN_PITCH_ROLL, BAIConstructorConfig.MAX_PITCH_ROLL);
            _controls.Add(control);
        }

        private static void CreatePGravityAlignMaxRollControl()
        {
            var control = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSlider, IMyRemoteControl>("BetterAI_PGravityAlignMaxRoll");
            control.Title = MyStringId.GetOrCompute("Max Roll Angle");
            control.Tooltip = MyStringId.GetOrCompute("Maximum roll angle when aligning to gravity (degrees)");
            control.Getter = (block) => GetAIConstructorBlock(block)?.GetConfig()?.PGravityAlignMaxRoll ?? 10.0f;
            control.Setter = (block, value) =>
            {
                var aiBlock = GetAIConstructorBlock(block);
                if (aiBlock != null)
                {
                    var config = aiBlock.GetConfig();
                    config.PGravityAlignMaxRoll = MathHelper.Clamp(value, BAIConstructorConfig.MIN_PITCH_ROLL, BAIConstructorConfig.MAX_PITCH_ROLL);
                    aiBlock.SaveConfig();
                }
            };
            control.Visible = IsAIConstructorBlock;
            control.Writer = (block, result) => result.Append($"{GetAIConstructorBlock(block)?.GetConfig()?.PGravityAlignMaxRoll ?? 10.0f:F1}°");
            control.SetLimits(BAIConstructorConfig.MIN_PITCH_ROLL, BAIConstructorConfig.MAX_PITCH_ROLL);
            _controls.Add(control);
        }

        private static void CreateMonitorHydrogenLevelsControl()
        {
            var control = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlCheckbox, IMyRemoteControl>("BetterAI_MonitorHydrogenLevels");
            control.Title = MyStringId.GetOrCompute("Monitor Hydrogen Levels");
            control.Tooltip = MyStringId.GetOrCompute("Enable hydrogen level monitoring for automatic refueling");
            control.Getter = (block) => GetAIConstructorBlock(block)?.GetConfig()?.MonitorHydrogenLevels ?? false;
            control.Setter = (block, value) =>
            {
                var aiBlock = GetAIConstructorBlock(block);
                if (aiBlock != null)
                {
                    var config = aiBlock.GetConfig();
                    config.MonitorHydrogenLevels = value;
                    aiBlock.SaveConfig();

                    if (value && aiBlock.GetHydrogenTanks().Count == 0)
                    {
                        MyAPIGateway.Utilities.ShowMessage("AI Constructor", "Warning: No hydrogen tanks found!");
                    }
                }
            };
            control.Visible = IsAIConstructorBlock;
            _controls.Add(control);
        }

        private static void CreateHydrogenRefuelThresholdControl()
        {
            var control = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSlider, IMyRemoteControl>("BetterAI_HydrogenRefuelThreshold");
            control.Title = MyStringId.GetOrCompute("H2 Refuel Threshold");
            control.Tooltip = MyStringId.GetOrCompute("Return to base when hydrogen drops below this percentage");
            control.Getter = (block) => GetAIConstructorBlock(block)?.GetConfig()?.HydrogenRefuelThreshold ?? 25.0f;
            control.Setter = (block, value) =>
            {
                var aiBlock = GetAIConstructorBlock(block);
                if (aiBlock != null)
                {
                    var config = aiBlock.GetConfig();
                    config.HydrogenRefuelThreshold = MathHelper.Clamp(value, BAIConstructorConfig.MIN_POWER_THRESHOLD, BAIConstructorConfig.MAX_POWER_THRESHOLD);
                    if (config.HydrogenOperationalThreshold <= config.HydrogenRefuelThreshold)
                        config.HydrogenOperationalThreshold = config.HydrogenRefuelThreshold + 10.0f;
                    aiBlock.SaveConfig();
                }
            };
            control.Visible = (block) => IsAIConstructorBlock(block) && (GetAIConstructorBlock(block)?.GetConfig()?.MonitorHydrogenLevels ?? false);
            control.Writer = (block, result) => result.Append($"{GetAIConstructorBlock(block)?.GetConfig()?.HydrogenRefuelThreshold ?? 25.0f:F1}%");
            control.SetLimits(BAIConstructorConfig.MIN_POWER_THRESHOLD, BAIConstructorConfig.MAX_POWER_THRESHOLD);
            _controls.Add(control);
        }

        private static void CreateHydrogenOperationalThresholdControl()
        {
            var control = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSlider, IMyRemoteControl>("BetterAI_HydrogenOperationalThreshold");
            control.Title = MyStringId.GetOrCompute("H2 Operational Threshold");
            control.Tooltip = MyStringId.GetOrCompute("Resume operations when hydrogen reaches this percentage");
            control.Getter = (block) => GetAIConstructorBlock(block)?.GetConfig()?.HydrogenOperationalThreshold ?? 50.0f;
            control.Setter = (block, value) =>
            {
                var aiBlock = GetAIConstructorBlock(block);
                if (aiBlock != null)
                {
                    var config = aiBlock.GetConfig();
                    config.HydrogenOperationalThreshold = MathHelper.Clamp(value, BAIConstructorConfig.MIN_POWER_THRESHOLD, BAIConstructorConfig.MAX_POWER_THRESHOLD);
                    if (config.HydrogenOperationalThreshold <= config.HydrogenRefuelThreshold)
                        config.HydrogenRefuelThreshold = config.HydrogenOperationalThreshold - 10.0f;
                    aiBlock.SaveConfig();
                }
            };
            control.Visible = (block) => IsAIConstructorBlock(block) && (GetAIConstructorBlock(block)?.GetConfig()?.MonitorHydrogenLevels ?? false);
            control.Writer = (block, result) => result.Append($"{GetAIConstructorBlock(block)?.GetConfig()?.HydrogenOperationalThreshold ?? 50.0f:F1}%");
            control.SetLimits(BAIConstructorConfig.MIN_POWER_THRESHOLD, BAIConstructorConfig.MAX_POWER_THRESHOLD);
            _controls.Add(control);
        }

        private static void CreateMonitorBatteryLevelsControl()
        {
            var control = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlCheckbox, IMyRemoteControl>("BetterAI_MonitorBatteryLevels");
            control.Title = MyStringId.GetOrCompute("Monitor Battery Levels");
            control.Tooltip = MyStringId.GetOrCompute("Enable battery level monitoring for automatic recharging");
            control.Getter = (block) => GetAIConstructorBlock(block)?.GetConfig()?.MonitorBatteryLevels ?? false;
            control.Setter = (block, value) =>
            {
                var aiBlock = GetAIConstructorBlock(block);
                if (aiBlock != null)
                {
                    var config = aiBlock.GetConfig();
                    config.MonitorBatteryLevels = value;
                    aiBlock.SaveConfig();

                    if (value && aiBlock.GetBatteries().Count == 0)
                    {
                        MyAPIGateway.Utilities.ShowMessage("AI Constructor", "Warning: No batteries found!");
                    }
                }
            };
            control.Visible = IsAIConstructorBlock;
            _controls.Add(control);
        }

        private static void CreateBatteryRefuelThresholdControl()
        {
            var control = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSlider, IMyRemoteControl>("BetterAI_BatteryRefuelThreshold");
            control.Title = MyStringId.GetOrCompute("Battery Refuel Threshold");
            control.Tooltip = MyStringId.GetOrCompute("Return to base when battery drops below this percentage");
            control.Getter = (block) => GetAIConstructorBlock(block)?.GetConfig()?.BatteryRefuelThreshold ?? 20.0f;
            control.Setter = (block, value) =>
            {
                var aiBlock = GetAIConstructorBlock(block);
                if (aiBlock != null)
                {
                    var config = aiBlock.GetConfig();
                    config.BatteryRefuelThreshold = MathHelper.Clamp(value, BAIConstructorConfig.MIN_POWER_THRESHOLD, BAIConstructorConfig.MAX_POWER_THRESHOLD);
                    if (config.BatteryOperationalThreshold <= config.BatteryRefuelThreshold)
                        config.BatteryOperationalThreshold = config.BatteryRefuelThreshold + 10.0f;
                    aiBlock.SaveConfig();
                }
            };
            control.Visible = (block) => IsAIConstructorBlock(block) && (GetAIConstructorBlock(block)?.GetConfig()?.MonitorBatteryLevels ?? false);
            control.Writer = (block, result) => result.Append($"{GetAIConstructorBlock(block)?.GetConfig()?.BatteryRefuelThreshold ?? 20.0f:F1}%");
            control.SetLimits(BAIConstructorConfig.MIN_POWER_THRESHOLD, BAIConstructorConfig.MAX_POWER_THRESHOLD);
            _controls.Add(control);
        }

        private static void CreateBatteryOperationalThresholdControl()
        {
            var control = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlSlider, IMyRemoteControl>("BetterAI_BatteryOperationalThreshold");
            control.Title = MyStringId.GetOrCompute("Battery Operational Threshold");
            control.Tooltip = MyStringId.GetOrCompute("Resume operations when battery reaches this percentage");
            control.Getter = (block) => GetAIConstructorBlock(block)?.GetConfig()?.BatteryOperationalThreshold ?? 40.0f;
            control.Setter = (block, value) =>
            {
                var aiBlock = GetAIConstructorBlock(block);
                if (aiBlock != null)
                {
                    var config = aiBlock.GetConfig();
                    config.BatteryOperationalThreshold = MathHelper.Clamp(value, BAIConstructorConfig.MIN_POWER_THRESHOLD, BAIConstructorConfig.MAX_POWER_THRESHOLD);
                    if (config.BatteryOperationalThreshold <= config.BatteryRefuelThreshold)
                        config.BatteryRefuelThreshold = config.BatteryOperationalThreshold - 10.0f;
                    aiBlock.SaveConfig();
                }
            };
            control.Visible = (block) => IsAIConstructorBlock(block) && (GetAIConstructorBlock(block)?.GetConfig()?.MonitorBatteryLevels ?? false);
            control.Writer = (block, result) => result.Append($"{GetAIConstructorBlock(block)?.GetConfig()?.BatteryOperationalThreshold ?? 40.0f:F1}%");
            control.SetLimits(BAIConstructorConfig.MIN_POWER_THRESHOLD, BAIConstructorConfig.MAX_POWER_THRESHOLD);
            _controls.Add(control);
        }

        private static void CreateBroadcastUpdatesControl()
        {
            var control = MyAPIGateway.TerminalControls.CreateControl<IMyTerminalControlCheckbox, IMyRemoteControl>("BetterAI_BroadcastUpdates");
            control.Title = MyStringId.GetOrCompute("Broadcast Updates");
            control.Tooltip = MyStringId.GetOrCompute("Enable broadcasting status updates via antenna");
            control.Getter = (block) => GetAIConstructorBlock(block)?.GetConfig()?.BroadcastUpdates ?? false;
            control.Setter = (block, value) =>
            {
                var aiBlock = GetAIConstructorBlock(block);
                if (aiBlock != null)
                {
                    var config = aiBlock.GetConfig();
                    config.BroadcastUpdates = value;
                    aiBlock.SaveConfig();
                }
            };
            control.Visible = IsAIConstructorBlock;
            _controls.Add(control);
        }

        #endregion

        #region Actions

        private static void CreateToggleEnableAction()
        {
            var action = MyAPIGateway.TerminalControls.CreateAction<IMyRemoteControl>("BetterAI_ToggleEnable");
            action.Icon = @"Textures\GUI\Icons\Actions\Toggle.dds";
            action.Name = new StringBuilder("Toggle AI Constructor");
            action.Action = (block) =>
            {
                var aiBlock = GetAIConstructorBlock(block);
                if (aiBlock != null)
                {
                    aiBlock.SetEnabled(!aiBlock.IsEnabled);
                }
            };
            action.Writer = (block, result) => result.Append(GetAIConstructorBlock(block)?.IsEnabled == true ? "On" : "Off");
            action.ValidForGroups = true;
            action.Enabled = IsAIConstructorBlock;
            _actions.Add(action);
        }

        #endregion

        #region Event Handlers

        private static void CustomControlGetter(IMyTerminalBlock block, List<IMyTerminalControl> controls)
        {
            try
            {
                if (!IsAIConstructorBlock(block))
                    return;

                // Create controls if not done yet
                if (!_controlsCreated)
                    CreateControls();

                // Hide default remote control controls that we don't want
                HideDefaultControls(controls);

                // Add our custom controls
                foreach (var control in _controls)
                {
                    controls.Add(control);
                }
            }
            catch (Exception ex)
            {
                MyLog.Default.WriteLine($"BetterAI_Constructor: CustomControlGetter exception: {ex}");
            }
        }

        private static void CustomActionGetter(IMyTerminalBlock block, List<IMyTerminalAction> actions)
        {
            try
            {
                if (!IsAIConstructorBlock(block))
                    return;

                // Create controls if not done yet
                if (!_controlsCreated)
                    CreateControls();

                // Add our custom actions
                foreach (var action in _actions)
                {
                    actions.Add(action);
                }
            }
            catch (Exception ex)
            {
                MyLog.Default.WriteLine($"BetterAI_Constructor: CustomActionGetter exception: {ex}");
            }
        }

        private static void HideDefaultControls(List<IMyTerminalControl> controls)
        {
            // Hide specific default remote control features we don't want to show
            var controlsToHide = new HashSet<string>
            {
                "AutoPilot",
                "DockingMode",
                "Direction",
                "ControlWheels",
                "ControlThrusters",
                "HandBrake",
                "DampenersOverride",
                "MainCockpit",
                "HorizonIndicator",
                "AutopilotSpeedLimit",
                "FlightMode",
                "WaypointList",
                "RemoveWaypoint",
                "MoveUp",
                "MoveDown",
                "AddWaypoint",
                "GPS",
                "SetGPSCoords",
                "SpeedLimit"
            };

            for (int i = controls.Count - 1; i >= 0; i--)
            {
                if (controlsToHide.Contains(controls[i].Id))
                {
                    controls.RemoveAt(i);
                }
            }
        }

        #endregion

        public static void Unload()
        {
            try
            {
                MyAPIGateway.TerminalControls.CustomControlGetter -= CustomControlGetter;
                MyAPIGateway.TerminalControls.CustomActionGetter -= CustomActionGetter;

                _controls?.Clear();
                _actions?.Clear();
                _initialized = false;
                _controlsCreated = false;
            }
            catch (Exception ex)
            {
                MyLog.Default.WriteLine($"BetterAI_Constructor: Unload exception: {ex}");
            }
        }
    }
}