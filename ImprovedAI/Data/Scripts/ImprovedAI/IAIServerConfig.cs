using ImprovedAI.Utils.Logging;
using Sandbox.ModAPI;
using System;
using System.Linq;
using VRage.Game.ModAPI;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Utils;
using VRageMath;


namespace ImprovedAI.Config
{
    public enum LogLevel
    {
        Verbose,
        Info,
        Debug,
        Warning,
        Error
    }
    /// <summary>
    /// Serialization mode for the message queue
    /// </summary>
    public enum MessageSerializationMode
    {
        /// <summary>
        /// XML serialization is slow, but good for debugging.
        /// </summary>
        XML,

        /// <summary>
        /// ProtoBuf serialization, faster, no visibility in-flight.
        /// </summary>
        ProtoBuf
    }

    public static class ServerConfig
    {
        public const string MOD_NAME = "BetterAIConstructor";
        private static bool _configLoaded = false;
        private static readonly string CONFIG_FILENAME = "BetterAIConstructor.cfg";

        // Limits

        public static class BlockLimits
        {
            public static int MaxSchedulersPerPlayer { get; private set; } = 1;
            public static int MaxSchedulersPerFaction { get; private set; } = 5;
            public static int MaxDroneControllersPerPlayer { get; private set; } = 10;
            public static int MaxDroneControllersPerFaction { get; private set; } = 50;
        }

        // Pathfinding
        public static class Pathfinding
        {
            public static bool AllowDirectPathfinding { get; private set; } = true;
            public static bool AllowSensors { get; private set; } = true;
            public static bool AllowObstacleAvoidance { get; private set; } = true;
            public static bool AllowSimpleRepositioning { get; private set; } = true;
            public static bool AllowAStar { get; private set; } = false;
            public static bool AllowDStarLite { get; private set; } = false;
            public static bool UseGravityAwarePathing { get; private set; } = true;
            public static double MinWaypointDistance { get; private set; } = 5.0;
            public static double MaxWaypointDistance { get; private set; } = 15.0;
            public static int MaxRepositionAttempts { get; private set; } = 10;
            public static double RepositionStep { get; private set; } = 10.0;
            public static double MinAltitudeBuffer { get; private set; } = 20.0;
        }

        // Performance
        public static int MaxPathNodes { get; private set; } = 1000;
        public static TimeSpan MaxPathfindingTime { get; private set; } = TimeSpan.FromMilliseconds(50);
        public static int AIUpdateInterval { get; private set; } = 10;
        public static int PowerCheckInterval { get; private set; } = 300;
        public static int BroadcastInterval { get; private set; } = 600;

        // Construction
        public static int MaxBlocksPerScan { get; private set; } = 50;
        public static int MaxTargetBlocks { get; private set; } = 20;
        public static double DefaultConstructionRange { get; private set; } = 100.0;
        public static double MaxConstructionRange { get; private set; } = 500.0;

        // Safety
        public static bool EnableSafetyLimits { get; private set; } = true;
        public static float MaxThrustOverride { get; private set; } = 1.0f;
        public static float MaxGyroOverride { get; private set; } = 1.0f;
        public static bool RequireConnector { get; private set; } = true;
        public static bool RequireWelderForWelding { get; private set; } = true;
        public static bool RequireGrinderForGrinding { get; private set; } = true;
        public static bool EnableEmergencyStop { get; private set; } = true;
        public static int MaxConsecutiveErrors { get; private set; } = 3;

        // Power
        public static bool DefaultMonitorHydrogen { get; private set; } = false;
        public static bool DefaultMonitorBattery { get; private set; } = false;
        public static float DefaultHydrogenRefuelThreshold { get; private set; } = 25.0f;
        public static float DefaultHydrogenOperationalThreshold { get; private set; } = 50.0f;
        public static float DefaultBatteryRefuelThreshold { get; private set; } = 20.0f;
        public static float DefaultBatteryOperationalThreshold { get; private set; } = 40.0f;
        public static float MinPowerThreshold { get; private set; } = 5.0f;
        public static float MaxPowerThreshold { get; private set; } = 95.0f;

        public static class SchedulerBounds
        {
            public static int MaxTargetLimit { get; private set; } = 100;
            public static int PerScanLimit { get; private set; } = 100;
            /// <summary>
            /// Power usage for scheduler in Watts. Default 1MW (1,000,000 W).
            /// </summary>
            public static float SchedulerPowerUsage { get; private set; } = 1000000.0f;
        }
        /// <summary>
        /// This section is about the Drone network communication.
        /// System implements a pseudo-messaging system between scheduler and drones.
        /// </summary>
        public static class DroneNetwork
        {
            public static bool AllowNetworkBroadcasting { get; private set; } = true;
            public static float DefaultAntennaRange { get; private set; } = 1000.0f;
            public static float NetworkUpdateRate { get; private set; } = 10.0f;
            /// <summary>
            /// Schedulers keep a cache of antennas. Limit how often said cache is updated.
            /// </summary>
            public static int SchedulerAntennaCacheUpdateIntervalTicks { get; private set; } = 60;
            /// <summary>
            /// Throttling for scheduler reading subscribed messages.
            /// </summary>
            // 4 messages/second, scheduler expects a lot of messages from managed drones
            public static int SchedulerMessageThrottlingTicks { get; private set; } = 15;
            public static int SchedulerMessageReadLimit { get; private set; } = 50;
            /// <summary>
            /// Throttling for scheduler reading subscribed messages.
            /// </summary>
            // 1 message every 3 seconds, drone orders are not as frequent.
            public static int DroneMessageThrottlingTicks { get; private set; } = 180; // 3 seconds
            /// <summary>
            /// How long messages live in the message queue.
            /// </summary>
            public static int MessageRetentionTicks { get; private set; } = 1800; // 30 seconds
            public static int MessageCleanupIntervalTicks { get; private set; } = 600; // 10 seconds
            public static MessageSerializationMode MessageSerializationMode { get; private set; } = MessageSerializationMode.ProtoBuf;
        }

        // Logging
        public static class Logging
        {
            public static LogLevel LogLevel { get; private set; } = LogLevel.Info;
            public static bool LogPathfinding { get; private set; } = false;
            public static bool LogDroneNetwork { get; private set; } = true;
            public static bool LogDroneOrders { get; private set; } = true;
        }

        // Compatibility
        public static bool CompatibilityMode { get; private set; } = false;
        public static bool SkipVersionChecks { get; private set; } = false;
        public static bool AllowLegacyConfig { get; private set; } = true;

        public static void LoadConfig()
        {
            if (_configLoaded) return;
            _configLoaded = true;

            try
            {
                if (!MyAPIGateway.Session.IsServer)
                {
                    Log.Info("Config loading skipped - not server");
                    return;
                }

                Log.Info("Loading server configuration...");

                // Try to read the config file from mod storage
                var configText = LoadConfigFile();
                if (string.IsNullOrEmpty(configText))
                {
                    MyAPIGateway.Utilities.ShowMessage(MOD_NAME, "No config file found, using defaults");
                    return;
                }

                ParseConfig(configText);
                ApplyPathfindingConfig();

                MyAPIGateway.Utilities.ShowMessage(MOD_NAME, "Server configuration loaded successfully");
            }
            catch (Exception ex)
            {
                MyAPIGateway.Utilities.ShowMessage(MOD_NAME, $"Error loading config: {ex.Message}");
                MyLog.Default.WriteLine($"{MOD_NAME}: LoadConfig exception: {ex}");
            }
        }

        private static string LoadConfigFile()
        {
            try
            {
                // Get the current mod item
                var modItem = MyAPIGateway.Session.Mods.FirstOrDefault(m => m.GetPath().Contains("ImprovedAI"));
                if (modItem.Name == MOD_NAME)
                {
                    Log.Warning($"{MOD_NAME}: Could not find mod item for config loading");
                    return null;
                }

                if (MyAPIGateway.Utilities.FileExistsInModLocation(CONFIG_FILENAME, modItem))
                {
                    using (var reader = MyAPIGateway.Utilities.ReadFileInModLocation(CONFIG_FILENAME, modItem))
                    {
                        return reader.ReadToEnd();
                    }
                }

                // Fallback: try to read from world storage
                if (MyAPIGateway.Utilities.FileExistsInWorldStorage(CONFIG_FILENAME, typeof(IAISession)))
                {
                    using (var reader = MyAPIGateway.Utilities.ReadFileInWorldStorage(CONFIG_FILENAME, typeof(IAISession)))
                    {
                        return reader.ReadToEnd();
                    }
                }

                return null;
            }
            catch (Exception ex)
            {
                MyLog.Default.WriteLine($"{MOD_NAME}: LoadConfigFile exception: {ex}");
                return null;
            }
        }

        private static void ParseConfig(string configText)
        {
            var ini = new MyIni();
            if (!ini.TryParse(configText))
            {
                MyAPIGateway.Utilities.ShowMessage(MOD_NAME, "Failed to parse config file - using defaults");
                return;
            }

            // Parse Limits section
            MaxConstructorsPerPlayer = Math.Max(0, ini.Get("Limits", "MaxConstructorsPerPlayer").ToInt32(MaxConstructorsPerPlayer));
            MaxConstructorsPerFaction = Math.Max(0, ini.Get("Limits", "MaxConstructorsPerFaction").ToInt32(MaxConstructorsPerFaction));
            MaxConstructorsTotal = Math.Max(0, ini.Get("Limits", "MaxConstructorsTotal").ToInt32(MaxConstructorsTotal));

            // Parse Pathfinding section
            AllowDirectPathfinding = ini.Get("Pathfinding", "AllowDirectPathfinding").ToBoolean(AllowDirectPathfinding);
            AllowSensors = ini.Get("Pathfinding", "AllowSensors").ToBoolean(AllowSensors);
            AllowObstacleAvoidance = ini.Get("Pathfinding", "AllowObstacleAvoidance").ToBoolean(AllowObstacleAvoidance);
            AllowSimpleRepositioning = ini.Get("Pathfinding", "AllowSimpleRepositioning").ToBoolean(AllowSimpleRepositioning);
            AllowAStar = ini.Get("Pathfinding", "AllowAStar").ToBoolean(AllowAStar);
            AllowDStarLite = ini.Get("Pathfinding", "AllowDStarLite").ToBoolean(AllowDStarLite);
            UseGravityAwarePathing = ini.Get("Pathfinding", "UseGravityAwarePathing").ToBoolean(UseGravityAwarePathing);
            MinWaypointDistance = Math.Max(1.0, ini.Get("Pathfinding", "MinWaypointDistance").ToDouble(MinWaypointDistance));
            MaxWaypointDistance = Math.Max(MinWaypointDistance, ini.Get("Pathfinding", "MaxWaypointDistance").ToDouble(MaxWaypointDistance));
            MaxRepositionAttempts = Math.Max(1, ini.Get("Pathfinding", "MaxRepositionAttempts").ToInt32(MaxRepositionAttempts));
            RepositionStep = Math.Max(1.0, ini.Get("Pathfinding", "RepositionStep").ToDouble(RepositionStep));
            MinAltitudeBuffer = Math.Max(0.0, ini.Get("Pathfinding", "MinAltitudeBuffer").ToDouble(MinAltitudeBuffer));

            // Parse Performance section
            MaxPathNodes = Math.Max(100, ini.Get("Performance", "MaxPathNodes").ToInt32(MaxPathNodes));
            var maxPathfindingMs = Math.Max(10, ini.Get("Performance", "MaxPathfindingTimeMs").ToInt32(50));
            MaxPathfindingTime = TimeSpan.FromMilliseconds(maxPathfindingMs);
            AIUpdateInterval = Math.Max(1, ini.Get("Performance", "AIUpdateInterval").ToInt32(AIUpdateInterval));
            PowerCheckInterval = Math.Max(60, ini.Get("Performance", "PowerCheckInterval").ToInt32(PowerCheckInterval));
            BroadcastInterval = Math.Max(60, ini.Get("Performance", "BroadcastInterval").ToInt32(BroadcastInterval));
            SchedulerAntennaCacheUpdateInterval = Math.Max(600, ini.Get("Performance", "SchedulerAntennaCacheInterval").ToInt32(BroadcastInterval));

            // Parse Construction section
            MaxBlocksPerScan = Math.Max(10, ini.Get("Construction", "MaxBlocksPerScan").ToInt32(MaxBlocksPerScan));
            MaxTargetBlocks = Math.Max(1, ini.Get("Construction", "MaxTargetBlocks").ToInt32(MaxTargetBlocks));
            DefaultConstructionRange = Math.Max(10.0, ini.Get("Construction", "DefaultConstructionRange").ToDouble(DefaultConstructionRange));
            MaxConstructionRange = Math.Max(DefaultConstructionRange, ini.Get("Construction", "MaxConstructionRange").ToDouble(MaxConstructionRange));

            // Parse Safety section
            EnableSafetyLimits = ini.Get("Safety", "EnableSafetyLimits").ToBoolean(EnableSafetyLimits);
            MaxThrustOverride = MathHelper.Clamp(ini.Get("Safety", "MaxThrustOverride").ToSingle(MaxThrustOverride), 0.0f, 1.0f);
            MaxGyroOverride = MathHelper.Clamp(ini.Get("Safety", "MaxGyroOverride").ToSingle(MaxGyroOverride), 0.0f, 1.0f);
            RequireConnector = ini.Get("Safety", "RequireConnector").ToBoolean(RequireConnector);
            RequireWelderForWelding = ini.Get("Safety", "RequireWelderForWelding").ToBoolean(RequireWelderForWelding);
            RequireGrinderForGrinding = ini.Get("Safety", "RequireGrinderForGrinding").ToBoolean(RequireGrinderForGrinding);
            EnableEmergencyStop = ini.Get("Safety", "EnableEmergencyStop").ToBoolean(EnableEmergencyStop);
            MaxConsecutiveErrors = Math.Max(1, ini.Get("Safety", "MaxConsecutiveErrors").ToInt32(MaxConsecutiveErrors));

            // Parse Power section
            DefaultMonitorHydrogen = ini.Get("Power", "DefaultMonitorHydrogen").ToBoolean(DefaultMonitorHydrogen);
            DefaultMonitorBattery = ini.Get("Power", "DefaultMonitorBattery").ToBoolean(DefaultMonitorBattery);
            DefaultHydrogenRefuelThreshold = MathHelper.Clamp(ini.Get("Power", "DefaultHydrogenRefuelThreshold").ToSingle(DefaultHydrogenRefuelThreshold), 5.0f, 95.0f);
            DefaultHydrogenOperationalThreshold = MathHelper.Clamp(ini.Get("Power", "DefaultHydrogenOperationalThreshold").ToSingle(DefaultHydrogenOperationalThreshold), DefaultHydrogenRefuelThreshold + 5.0f, 95.0f);
            DefaultBatteryRefuelThreshold = MathHelper.Clamp(ini.Get("Power", "DefaultBatteryRefuelThreshold").ToSingle(DefaultBatteryRefuelThreshold), 5.0f, 95.0f);
            DefaultBatteryOperationalThreshold = MathHelper.Clamp(ini.Get("Power", "DefaultBatteryOperationalThreshold").ToSingle(DefaultBatteryOperationalThreshold), DefaultBatteryRefuelThreshold + 5.0f, 95.0f);
            MinPowerThreshold = MathHelper.Clamp(ini.Get("Power", "MinPowerThreshold").ToSingle(MinPowerThreshold), 1.0f, 50.0f);
            MaxPowerThreshold = MathHelper.Clamp(ini.Get("Power", "MaxPowerThreshold").ToSingle(MaxPowerThreshold), 50.0f, 99.0f);

            // Parse Network section
            AllowNetworkBroadcasting = ini.Get("Network", "AllowNetworkBroadcasting").ToBoolean(AllowNetworkBroadcasting);
            DefaultAntennaRange = Math.Max(100.0f, ini.Get("Network", "DefaultAntennaRange").ToSingle(DefaultAntennaRange));
            NetworkUpdateRate = Math.Max(1.0f, ini.Get("Network", "NetworkUpdateRate").ToSingle(NetworkUpdateRate));

            // Parse Logging section
            EnableDebugLogging = ini.Get("Logging", "EnableDebugLogging").ToBoolean(EnableDebugLogging);
            Logging.LogPathfinding = ini.Get("Logging", "LogPathfinding").ToBoolean(LogPathfinding);
            LogConstruction = ini.Get("Logging", "LogConstruction").ToBoolean(LogConstruction);
            LogPowerManagement = ini.Get("Logging", "LogPowerManagement").ToBoolean(LogPowerManagement);
            LogPerformance = ini.Get("Logging", "LogPerformance").ToBoolean(LogPerformance);

            // Parse Compatibility section
            CompatibilityMode = ini.Get("Compatibility", "CompatibilityMode").ToBoolean(CompatibilityMode);
            SkipVersionChecks = ini.Get("Compatibility", "SkipVersionChecks").ToBoolean(SkipVersionChecks);
            AllowLegacyConfig = ini.Get("Compatibility", "AllowLegacyConfig").ToBoolean(AllowLegacyConfig);
        }


        public static bool CanPlayerCreateDroneController(long playerId)
        {
            if (BlockLimits.MaxDroneControllersPerPlayer < 0) return true;

            int playerConstructorCount = 0;
            if (IAISession.Instance?.AIDroneControllers != null)
            {
                foreach (var constructor in IAISession.Instance.AIDroneControllers.Values)
                {
                    var slim = (IMySlimBlock)constructor;
                    if (slim.OwnerId == playerId)
                        playerConstructorCount++;
                }
            }

            return playerConstructorCount < BlockLimits.MaxDroneControllersPerPlayer;
        }
        public static bool CanPlayerCreateScheduler(long playerId)
        {
            if (BlockLimits.MaxSchedulersPerPlayer < 0) return true;

            int playerSchedulerCount = 0;
            if (IAISession.Instance?.AIDroneSchedulers != null)
            {
                foreach (var constructor in IAISession.Instance.AIDroneSchedulers.Values)
                {
                    var slim = (IMySlimBlock)constructor;
                    if (slim.OwnerId == playerId)
                        playerSchedulerCount++;
                }
            }

            return playerSchedulerCount < BlockLimits.MaxDroneControllersPerPlayer;
        }

        public static bool CanFactionCreateScheduler(long factionId)
        {
            if (BlockLimits.MaxSchedulersPerFaction<= 0) return true;

            int factionConstructorCount = 0;
            if (IAISession.Instance?.AIDroneSchedulers != null)
            {
                foreach (var constructor in IAISession.Instance.AIDroneSchedulers.Values)
                {
                    var slim = (IMySlimBlock)constructor;
                    var faction = MyAPIGateway.Session.Factions.TryGetPlayerFaction(slim.OwnerId);
                    
                    if (faction?.FactionId == factionId)
                        factionConstructorCount++;
                }
            }

            return factionConstructorCount < BlockLimits.MaxSchedulersPerFaction;
        }
        public static bool CanFactionCreateDroneController(long factionId)
        {
            if (BlockLimits.MaxDroneControllersPerFaction <= 0) return true;

            int factionConstructorCount = 0;
            if (IAISession.Instance?.AIDroneControllers != null)
            {
                foreach (var constructor in IAISession.Instance.AIDroneControllers.Values)
                {
                    var slim = (IMySlimBlock)constructor;
                    var faction = MyAPIGateway.Session.Factions.TryGetPlayerFaction(slim.OwnerId);

                    if (faction?.FactionId == factionId)
                        factionConstructorCount++;
                }
            }

            return factionConstructorCount < BlockLimits.MaxDroneControllersPerFaction;
        }


        //public static string GetConfigSummary()
        //{
        //    var summary = "=== BetterAI Constructor Server Config ===\n";
        //    summary += $"Max per Player: {(MaxConstructorsPerPlayer > 0 ? MaxConstructorsPerPlayer.ToString() : "Unlimited")}\n";
        //    summary += $"Max per Faction: {(MaxConstructorsPerFaction > 0 ? MaxConstructorsPerFaction.ToString() : "Unlimited")}\n";
        //    summary += $"Max Total: {(MaxConstructorsTotal > 0 ? MaxConstructorsTotal.ToString() : "Unlimited")}\n";
        //    summary += $"Current Total: {IAISession.Instance?.AIConstructors?.Count ?? 0}\n";
        //    summary += $"Pathfinding: {(AllowAStar ? "Advanced" : AllowObstacleAvoidance ? "Sensors" : "Basic")}\n";
        //    summary += $"Safety Limits: {(EnableSafetyLimits ? "Enabled" : "Disabled")}\n";
        //    return summary;
        //}
    }
}