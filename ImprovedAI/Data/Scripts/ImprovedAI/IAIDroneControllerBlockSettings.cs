using Sandbox.ModAPI;
using VRageMath;

namespace ImprovedAI
{


    public class IAIDroneControllerBlockSettings
    {
        // Configuration values
        public Drone.OperationMode OperationMode { get; set; } = Drone.OperationMode.StandAlone;
        public float WaypointTolerance { get; set; } = 5.0f;
        public float MaxApproachSpeed { get; set; } = 5.0f;
        public float MaxTravelSpeed { get; set; } = 50.0f;
        public float DockingSpeed { get; set; } = 50.0f;
        public bool AlignToPGravity { get; set; } = true;
        public float PGravityAlignMaxPitch { get; set; } = 10.0f;
        public float PGravityAlignMaxRoll { get; set; } = 10.0f;
        public bool BroadcastToChat { get; set; } = false;
        public string LCDScreenTag { get; set; } = "[BAIC]"; // Drone state will be sent to each LCD screen that contains this tag.

        // Power monitoring settings
        public bool MonitorHydrogenLevels { get; set; } = false;
        public float HydrogenRefuelThreshold { get; set; } = 25.0f;     // Return to base when H2 below this %
        public float HydrogenOperationalThreshold { get; set; } = 50.0f; // Resume ops when H2 above this %
        public bool RefuelWhenDocked { get; set; } = false; // Refuel everytime drone is docked

        public bool MonitorBatteryLevels { get; set; } = false;
        public float BatteryRefuelThreshold { get; set; } = 20.0f;      // Return to base when battery below this %
        public float BatteryOperationalThreshold { get; set; } = 40.0f; // Resume ops when battery above this %

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
            MaxApproachSpeed = 5.0f;
            DockingSpeed = 5.0f;
            MaxTravelSpeed = 50.0f;
            AlignToPGravity = true;
            PGravityAlignMaxPitch = 10.0f;
            PGravityAlignMaxRoll = 10.0f;
            BroadcastToChat = false;
            LCDScreenTag = "[CD]";

            // Power monitoring defaults
            MonitorHydrogenLevels = false;
            HydrogenRefuelThreshold = 25.0f;
            HydrogenOperationalThreshold = 50.0f;
            MonitorBatteryLevels = false;
            BatteryRefuelThreshold = 20.0f;
            BatteryOperationalThreshold = 40.0f;
        }

        public bool ValidateSettings()
        {
            bool isValid = true;

            if (WaypointTolerance < MIN_WAYPOINT_TOLERANCE || WaypointTolerance > MAX_WAYPOINT_TOLERANCE)
            {
                MyAPIGateway.Utilities.ShowMessage("AIConfig", $"WaypointTolerance out of range. Clamping to valid values.");
                WaypointTolerance = MathHelper.Clamp(WaypointTolerance, MIN_WAYPOINT_TOLERANCE, MAX_WAYPOINT_TOLERANCE);
                isValid = false;
            }

            if (MaxApproachSpeed < MIN_SPEED || MaxApproachSpeed > MAX_APPROACH_SPEED)
            {
                MyAPIGateway.Utilities.ShowMessage("AIConfig", $"MaxApproachSpeed out of range. Clamping to valid values.");
                MaxApproachSpeed = MathHelper.Clamp(MaxApproachSpeed, MIN_SPEED, MAX_APPROACH_SPEED);
                isValid = false;
            }

            if (MaxTravelSpeed < MIN_SPEED || MaxTravelSpeed > MAX_TRAVEL_SPEED)
            {
                MyAPIGateway.Utilities.ShowMessage("AIConfig", $"MaxTravelSpeed out of range. Clamping to valid values.");
                MaxTravelSpeed = MathHelper.Clamp(MaxTravelSpeed, MIN_SPEED, MAX_TRAVEL_SPEED);
                isValid = false;
            }

            if (PGravityAlignMaxPitch < MIN_PITCH_ROLL || PGravityAlignMaxPitch > MAX_PITCH_ROLL)
            {
                MyAPIGateway.Utilities.ShowMessage("AIConfig", $"PGravityAlignMaxPitch out of range. Clamping to valid values.");
                PGravityAlignMaxPitch = MathHelper.Clamp(PGravityAlignMaxPitch, MIN_PITCH_ROLL, MAX_PITCH_ROLL);
                isValid = false;
            }

            if (PGravityAlignMaxRoll < MIN_PITCH_ROLL || PGravityAlignMaxRoll > MAX_PITCH_ROLL)
            {
                MyAPIGateway.Utilities.ShowMessage("AIConfig", $"PGravityAlignMaxRoll out of range. Clamping to valid values.");
                PGravityAlignMaxRoll = MathHelper.Clamp(PGravityAlignMaxRoll, MIN_PITCH_ROLL, MAX_PITCH_ROLL);
                isValid = false;
            }

            return isValid;
        }
    }
}