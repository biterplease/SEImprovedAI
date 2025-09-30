using EmptyKeys.UserInterface.Generated.DataTemplatesEditFaction_Bindings;
using Sandbox.ModAPI;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRageMath;

using static ImprovedAI.Drone;

namespace ImprovedAI
{

    /// <summary>
    /// Drone controller configurable settings.
    /// These should always be validated, and bounded by, the mod server settings.
    /// </summary>
    public class IAIDroneControllerSettings
    {
        // Configuration values
        public bool Enabled { get; set; }
        public OperationMode OperationMode { get; set; } = Drone.OperationMode.StandAlone;
        /// <summary>
        /// Drone will always return to this connector when no more tasks are assigned.
        /// WARNING: if this connector is set as home to more than one drone, may have catastrophic consequences.
        /// </summary>
        public long HomeConnectorEntityId { get; set; }
        /// <summary>
        /// Normally drones handle all tasks that they are capable for.
        /// If enabled, will allow the user to filter specific task types that this drone will be allowed to handle.
        /// </summary>
        public bool UseTaskTypeFilters { get; set; }
        public Scheduler.TaskType TaskTypeFilters { get; set; }
        /// <summary>
        /// Normally, drones report all their capabilities to the Scheduler class
        /// If enabled, it will filter out certain capabilities when reporting to its scheduler.
        /// StandAlone
        /// </summary>
        public bool UseCapabilityFilters { get; set; }
        public Capabilities CapabilityFilters { get; set; }
        /// <summary>
        /// How close to the waypoint does the drone need to be, in meters.
        /// </summary>
        public float WaypointTolerance { get; set; } = 5.0f;
        /// <summary>
        /// Recommended speed when approaching targets, in m/s. Mostly for weld or grind tasks.
        /// Logistics task generally use docking speed for approach
        /// </summary>
        public float ApproachSpeed { get; set; } = 5.0f;
        /// <summary>
        /// Max speed in m/s.
        /// </summary>
        public float SpeedLimit { get; set; } = 50.0f;
        /// <summary>
        /// Docking speed in m/s.
        /// </summary>
        public float DockingSpeed { get; set; } = 2.5f;
        public bool AlignToPGravity { get; set; } = true;
        public float PGravityAlignMaxPitch { get; set; } = 10.0f;
        public float PGravityAlignMaxRoll { get; set; } = 10.0f;
         public string LCDScreenTag { get; set; } = "[IAIDrone]"; // Drone state will be sent to each LCD screen that contains this tag.

        // Power monitoring settings
        /// <summary>
        /// Monitory hydrogen levels. Drone will auto-dock to refuel and resume on its own..
        /// Disable if your drone is electric.
        /// </summary>
        public bool MonitorHydrogenLevels { get; set; } = false;
        /// <summary>
        /// When hydrogen tank levels are below this threshold, percentage-wise, drone will return home
        /// </summary>
        public float HydrogenRefuelThreshold { get; set; } = 25.0f;     // Return to base when H2 below this %
        public float HydrogenOperationalThreshold { get; set; } = 50.0f; // Resume ops when H2 above this %
        /// <summary>
        /// Always refuel when drone is docked.
        /// </summary>
        public bool AlwaysRefuelWhenDocked { get; set; } = false; // Refuel everytime drone is docked

        /// <summary>
        /// Monitor battery levels. Drone will auto-dock to refuel and resume on its own.
        /// Disable if your drone has a reactor and will never run out of power.
        /// </summary>
        public bool MonitorBatteryLevels { get; set; } = false;
        /// <summary>
        /// When battery storage levels are below this threshold, drone will return to home connector and recharge.
        /// </summary>
        public float BatteryRefuelThreshold { get; set; } = 20.0f;      // Return to base when battery below this %
        public float BatteryOperationalThreshold { get; set; } = 80.0f; // Resume ops when battery above this %

        // Server-defined limits (not user configurable)
        public static readonly float MIN_WAYPOINT_TOLERANCE = 0.5f;
        public static readonly float MAX_WAYPOINT_TOLERANCE = 10.0f;
        public static readonly float MIN_SPEED = 1.0f;
        public static readonly float MAX_APPROACH_SPEED = 15.0f;
        public static readonly float MAX_TRAVEL_SPEED = 50.0f;
        public static readonly float MIN_PITCH_ROLL = 0.0f;
        public static readonly float MAX_PITCH_ROLL = 45.0f;
        public static readonly float MIN_POWER_THRESHOLD = 5.0f;
        public static readonly float MAX_POWER_THRESHOLD = 95.0f;




        public void ResetToDefaults()
        {
            OperationMode = Drone.OperationMode.StandAlone;
            WaypointTolerance = 2.5f;
            ApproachSpeed = 5.0f;
            DockingSpeed = 5.0f;
            SpeedLimit = 50.0f;
            AlignToPGravity = true;
            PGravityAlignMaxPitch = 10.0f;
            PGravityAlignMaxRoll = 10.0f;
            LCDScreenTag = "[CD]";

            // Power monitoring defaults
            MonitorHydrogenLevels = false;
            HydrogenRefuelThreshold = 25.0f;
            HydrogenOperationalThreshold = 50.0f;
            MonitorBatteryLevels = false;
            BatteryRefuelThreshold = 20.0f;
            BatteryOperationalThreshold = 40.0f;
        }

        //public bool ValidateSettings()
        //{
        //    //bool isValid = true;

        //    //if (WaypointTolerance < MIN_WAYPOINT_TOLERANCE || WaypointTolerance > MAX_WAYPOINT_TOLERANCE)
        //    //{
        //    //    MyAPIGateway.Utilities.ShowMessage("AIConfig", $"WaypointTolerance out of range. Clamping to valid values.");
        //    //    WaypointTolerance = MathHelper.Clamp(WaypointTolerance, MIN_WAYPOINT_TOLERANCE, MAX_WAYPOINT_TOLERANCE);
        //    //    isValid = false;
        //    //}

        //    //if (MaxApproachSpeed < MIN_SPEED || MaxApproachSpeed > MAX_APPROACH_SPEED)
        //    //{
        //    //    MyAPIGateway.Utilities.ShowMessage("AIConfig", $"MaxApproachSpeed out of range. Clamping to valid values.");
        //    //    MaxApproachSpeed = MathHelper.Clamp(MaxApproachSpeed, MIN_SPEED, MAX_APPROACH_SPEED);
        //    //    isValid = false;
        //    //}

        //    //if (SpeedLimit < MIN_SPEED || SpeedLimit > MAX_TRAVEL_SPEED)
        //    //{
        //    //    MyAPIGateway.Utilities.ShowMessage("AIConfig", $"SpeedLimit out of range. Clamping to valid values.");
        //    //    SpeedLimit = MathHelper.Clamp(m, MIN_SPEED, MAX_TRAVEL_SPEED);
        //    //    isValid = false;
        //    //}

        //    //if (PGravityAlignMaxPitch < MIN_PITCH_ROLL || PGravityAlignMaxPitch > MAX_PITCH_ROLL)
        //    //{
        //    //    MyAPIGateway.Utilities.ShowMessage("AIConfig", $"PGravityAlignMaxPitch out of range. Clamping to valid values.");
        //    //    PGravityAlignMaxPitch = MathHelper.Clamp(PGravityAlignMaxPitch, MIN_PITCH_ROLL, MAX_PITCH_ROLL);
        //    //    isValid = false;
        //    //}

        //    //if (PGravityAlignMaxRoll < MIN_PITCH_ROLL || PGravityAlignMaxRoll > MAX_PITCH_ROLL)
        //    //{
        //    //    MyAPIGateway.Utilities.ShowMessage("AIConfig", $"PGravityAlignMaxRoll out of range. Clamping to valid values.");
        //    //    PGravityAlignMaxRoll = MathHelper.Clamp(PGravityAlignMaxRoll, MIN_PITCH_ROLL, MAX_PITCH_ROLL);
        //    //    isValid = false;
        //    //}

        //    //return isValid;
        //}
    }
}