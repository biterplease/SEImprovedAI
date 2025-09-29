using Sandbox.ModAPI;
using System;

namespace BetterAIConstructor.Config
{
    public enum PathfindingMethod
    {
        Direct = 0,           // Cheapest - straight line
        ObstacleAvoidance = 1,    // Sensor-based avoidance
        SimpleRepositioning = 2,  // Dumb repositioning
        AStar = 3,               // A* pathfinding
        DStarLite = 4           // Most expensive - D* Lite
    }

    public class PathfindingConfig
    {
        // Server-configurable limits
        public static bool AllowSensors { get; set; } = true;
        public static bool AllowSimpleRepositioning { get; set; } = true;
        public static bool AllowAStar { get; set; } = true;
        public static bool AllowDStarLite { get; set; } = false; // Expensive, disabled by default

        // Performance settings
        public static double MinWaypointDistance { get; set; } = 5.0;  // Server minimum
        public static double MaxWaypointDistance { get; set; } = 15.0; // Server maximum
        public static int MaxRepositionAttempts { get; set; } = 10;
        public static double RepositionStep { get; set; } = 10.0;

        // Gravity awareness
        public static bool UseGravityAwarePathing { get; set; } = true;
        public static double MinAltitudeBuffer { get; set; } = 20.0; // Meters above ground

        // Performance limits
        public static int MaxPathNodes { get; set; } = 1000;
        public static TimeSpan MaxPathfindingTime { get; set; } = TimeSpan.FromMilliseconds(50);

        public static void LoadFromConfig()
        {
            try
            {
                // Load from server config file or mod settings
                // This would integrate with SE's config system
                MyAPIGateway.Utilities.ShowMessage("PathfindingConfig", "Loading pathfinding configuration...");

                // Example of how config might be loaded:
                // var config = MyAPIGateway.Utilities.GetVariable("ConstructionAI_Config");
                // if (!string.IsNullOrEmpty(config))
                //     ParseConfig(config);
            }
            catch (Exception ex)
            {
                MyAPIGateway.Utilities.ShowMessage("PathfindingConfig", $"Error loading config: {ex.Message}");
            }
        }

        public static PathfindingMethod GetAvailableMethods()
        {
            var highest = PathfindingMethod.Direct;

            if (AllowSensors)
                highest = PathfindingMethod.ObstacleAvoidance;
            if (AllowSimpleRepositioning)
                highest = PathfindingMethod.SimpleRepositioning;
            if (AllowAStar)
                highest = PathfindingMethod.AStar;
            if (AllowDStarLite)
                highest = PathfindingMethod.DStarLite;

            return highest;
        }
    }
}