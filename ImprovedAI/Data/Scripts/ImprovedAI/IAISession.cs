using BetterAIConstructor.Config;
using BetterAIConstructor.UI;
using ImprovedAI.Config;
using ImprovedAI.Utils.Logging;
using Sandbox.ModAPI;
using System;
using System.Collections.Generic;
using VRage.Game.Components;
using VRage.Utils;

namespace ImprovedAI
{
    [MySessionComponentDescriptor(MyUpdateOrder.BeforeSimulation)]
    public class IAISession: MySessionComponentBase
    {
        public static IAISession Instance;

        // Mod-wide settings and management
        private bool isInitialized = false;
        private int updateCounter = 0;
        private const int UPDATE_INTERVAL = 60;
        private static Guid ModGuid = new Guid("1CFDA990-FD26-4950-A127-7BBC99FF1397");



        // Collection of all AI Constructor blocks in the world
        public Dictionary<long, IAIDroneBlock> AIDroneControllers = new Dictionary<long, IAIDroneBlock>();
        public Dictionary<long, IAISchedulerBlock> AIDroneSchedulers = new Dictionary<long, IAISchedulerBlock>();

        public override void LoadData()
        {
            Instance = this;
            Log.Info("Mod loaded successfully");
        }

        public override void BeforeStart()
        {
            try
            {
                // Initialize terminal controls for both client and server
                TerminalControls.Initialize();
                Log.Initialize(ServerConfig.MOD_NAME, 0, "ImprovedAI.log", typeof(IAISession));

                if (MyAPIGateway.Session.IsServer)
                {
                    // Server-specific initialization
                    ServerConfig.LoadConfig();
                    Log.Info("Session initialized on server");
                }
                else
                {
                    // Client-specific initialization
                    Log.Info("Session initialized on client");
                }

                isInitialized = true;
            }
            catch (Exception ex)
            {
                MyAPIGateway.Utilities.ShowMessage("BetterAIConstructor", $"Initialization error: {ex.Message}");
                MyLog.Default.WriteLine($"BetterAIConstructor: BeforeStart exception: {ex}");

                var writer = MyAPIGateway.Utilities.WriteFileInLocalStorage("error.log", typeof(IAISession));
                if (writer != null)
                {
                    writer.Write(ex.ToString());
                    writer.Flush();
                    writer.Close();
                }
            }
        }

        public override void UpdateBeforeSimulation()
        {
            try
            {
                if (!isInitialized)
                    return;

                // Only run AI logic on server
                if (!MyAPIGateway.Session.IsServer)
                    return;

                updateCounter++;

                if (updateCounter % UPDATE_INTERVAL == 0)
                {
                    // Periodic cleanup and maintenance
                    CleanupInvalidBlocks();

                    // Optional: Log active AI Constructor count
                    if (updateCounter % (UPDATE_INTERVAL * 10) == 0) // Every 10 seconds
                    {
                        Log.Info("Active AI Drone Schedulers: {0}", AIDroneSchedulers.Count);
                        Log.Info("Active AI Drone Controllers: {0}", AIDroneSchedulers.Count);
                    }
                }
            }
            catch (Exception ex)
            {
                MyAPIGateway.Utilities.ShowMessage("BetterAIConstructor", $"Update error: {ex.Message}");
                MyLog.Default.WriteLine($"BetterAIConstructor: UpdateBeforeSimulation exception: {ex}");
            }
        }

        private void CleanupInvalidBlocks()
        {
            var toRemove = new List<long>();
            foreach (var kvp in AIDroneSchedulers)
            {
                if (kvp.Value?.Entity == null || kvp.Value.Entity.MarkedForClose)
                {
                    toRemove.Add(kvp.Key);
                }
            }

            foreach (var entityId in toRemove)
            {
                AIDroneSchedulers.Remove(entityId);
            }
            var controllersToRemove = new List<long>();
            foreach (var kvp in AIDroneControllers)
            {
                if (kvp.Value?.Entity == null || kvp.Value.Entity.MarkedForClose)
                {
                    toRemove.Add(kvp.Key);
                }
            }

            foreach (var entityId in controllersToRemove)
            {
                AIDroneControllers.Remove(entityId);
            }
        }

        public void RegisterScheduler(IAISchedulerBlock block)
        {
            if (block?.Entity != null)
            {
                AIDroneSchedulers[block.Entity.EntityId] = block;
                Log.Info("Registered AI Drone Scheduler: {0} {1}", block.Entity.EntityId, block.Entity.DisplayName);
            }
        }

        public void UnregisterScheduler(long entityId)
        {
            if (AIDroneSchedulers.Remove(entityId))
            {
                Log.Info($"Unregistered AI Scheduler: {entityId}");
            }
        }
        public void RegisterDroneController(IAIDroneBlock block)
        {
            if (block?.Entity != null)
            {
                AIDroneControllers[block.Entity.EntityId] = block;
                Log.Info("Registered AI Drone controller: {0} {1}", block.Entity.EntityId, block.Entity.DisplayName);
            }
        }

        public void UnregisterDroneController(long entityId)
        {
            if (AIDroneControllers.Remove(entityId))
            {
                Log.Info($"Unregistered AI Drone controller: {entityId}");
            }
        }

        protected override void UnloadData()
        {
            try
            {
                // Clean shutdown
                AIDroneControllers.Clear();
                AIDroneSchedulers.Clear();

                // Unload terminal controls
                TerminalControls.Unload();

                Instance = null;
                Log.Info("Mod unloaded");
                Log.Close();
            }
            catch (Exception ex)
            {
                MyLog.Default.WriteLine($"BetterAIConstructor: UnloadData exception: {ex}");

                var writer = MyAPIGateway.Utilities.WriteFileInLocalStorage("shutdown_error.log", typeof(IAISession));
                if (writer != null)
                {
                    writer.Write(ex.ToString());
                    writer.Flush();
                    writer.Close();
                }
            }
        }
    }
}