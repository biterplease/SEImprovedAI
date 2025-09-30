using ImprovedAI.Config;
using Sandbox.ModAPI;
using Sandbox.ModAPI.Interfaces.Terminal;
using System.Collections.Generic;
using VRage.Game.ModAPI;

namespace ImprovedAI
{
    public class IAIDroneControllerTerminalControls
    {
        public static ServerConfig.DroneControllerBlockConfig BlockConfig = ServerConfig.Instance.Drone;

        const string IdPrefix = IAISession.ModName + "_";
        static bool Done = false;

        private static readonly HashSet<string> defaultControlIdsToHide = new HashSet<string>
        {
            "ControlThrusters",
            "ControlWheels",
            "ControlGyros",
            "HandBrake",
            "Park",
            "DampenersOverride",
            "HorizonIndicator",
            "MainCockpit",
            "TargetLocking",
            "OpenToolbar",
            "MainRemoteControl",
            "Control",
            "AutoPilot",
            "CollisionAvoidance",
            "DockingMode",
            "CameraList",
            "FlightMode",
            "Direction",
            "SpeedLimit",
            "WaypointList",
            "Open Toolb",
            "RemoveWaypoint",
            "MoveUp",
            "MoveDown",
            "AddWaypoint",
            "GpsList",
            "Reset",
            "Copy",
            "Paste",
        };
        private static readonly HashSet<string> defaultActionIdsToHide = new HashSet<string>
        {
            "ControlThrusters",
            "ControlWheels",
            "ControlGyros",
            "HandBrake",
            "Park",
            "DampenersOverride",
            "HorizonIndicator",
            "MainCockpit",
            "TargetLocking",
            "MainRemoteControl",
            "Control",
            "AutoPilot",
            "AutoPilot_On",
            "AutoPilot_Off",
            "CollisionAvoidance",
            "CollisionAvoidance_On",
            "CollisionAvoidance_Off",
            "DockingMode",
            "DockingMode_On",
            "DockingMode_Off",
            "SetFlightMode_BlockPropertyTitle_FlightMode_Patrol",
            "SetFlightMode_BlockPropertyTitle_FlightMode_Circle",
            "SetFlightMode_BlockPropertyTitle_FlightMode_OneWay",
            "IncreaseSpeedLimit",
            "DecreaseSpeedLimit",
            "Forward",
            "Backward",
            "Left",
            "Right",
            "Up",
            "Down",
            "Reset",
        };

        public static void DoOnce(IMyModContext context)
        {
            if (Done) return;
            Done = true;

            HideDefaultControls();
            HideDefaultActions();
            CreateControls();
        }

        /// <summary>
        /// Check an return the GameLogic object
        /// </summary>
        /// <param name="block"></param>
        /// <returns></returns>
        private static IAIDroneControllerBlock GetBlock(IMyTerminalBlock block)
        {
            if (block != null && block.GameLogic != null) return block.GameLogic.GetAs<IAIDroneControllerBlock>();
            return null;
        }

        static bool CustomVisibleCondition(IMyTerminalBlock b)
        {
            // only visible for the blocks having this gamelogic comp
            return b?.GameLogic?.GetAs<IAIDroneControllerBlock>() != null;
        }
        /// <summary>
        /// Hides default controls in the Programmable Block.
        /// </summary>
        public static void HideDefaultControls()
        {
            List<IMyTerminalControl> controls;
            MyAPIGateway.TerminalControls.GetControls<IMyRemoteControl>(out controls);

            foreach (IMyTerminalControl c in controls)
            {
                if (defaultControlIdsToHide.Contains(c.Id))
                {
                    c.Visible = TerminalChainedDelegate.Create(c.Enabled, CustomVisibleCondition);
                }
            }
        }

        /// <summary>
        /// Hides default actions in the Programmable Block.
        /// </summary>
        public static void HideDefaultActions()
        {
            List<IMyTerminalAction> actions;
            MyAPIGateway.TerminalControls.GetActions<IMyRemoteControl>(out actions);

            foreach (IMyTerminalAction a in actions)
            {
                if (defaultActionIdsToHide.Contains(a.Id))
                {
                    a.Enabled = TerminalChainedDelegate.Create(a.Enabled, CustomVisibleCondition);
                }
            }
        }

        public static void CreateControls() { }
    }
}
