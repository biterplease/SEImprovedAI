using BetterAIConstructor.UI;
using ImprovedAI.Config;
using ImprovedAI.Logistics;
using ImprovedAI.Network;
using ImprovedAI.Utils.Logging;
using Sandbox.ModAPI;
using System;
using System.Collections.Generic;
using VRage.Game.Components;
using VRage.Utils;

namespace ImprovedAI
{
    [MySessionComponentDescriptor(MyUpdateOrder.BeforeSimulation)]
    public class IAISession : MySessionComponentBase
    {
        public static IAISession Instance;

        // Shared message queue for all AI components
        public MessageQueue MessageQueue { get; private set; }

        // Mod-wide settings and management
        private bool isInitialized = false;
        private long _lastUpdateFrame = 0;
        private int _updateInterval = ServerConfig.Session.UpdateInterval;
        private int updateCounter = 0;
        private const int UPDATE_INTERVAL = 60;
        private static Guid ModGuid = new Guid("1CFDA990-FD26-4950-A127-7BBC99FF1397");

        // Collection of all AI blocks in the world
        public Dictionary<long, IAIDroneBlock> AIDroneControllers = new Dictionary<long, IAIDroneBlock>();
        public Dictionary<long, IAISchedulerBlock> AIDroneSchedulers = new Dictionary<long, IAISchedulerBlock>();
        public Dictionary<long, IAILogisticsComputer> AILogisticsComputers = new Dictionary<long, IAILogisticsComputer>();

        public override void LoadData()
        {
            Instance = this;
        }

        public void Init()
        {
            Log.Initialize(ServerConfig.MOD_NAME, 0, "ImprovedAI.log", typeof(IAISession));
            MessageQueue = MessageQueue.Instance;
            ServerConfig.LoadConfig();
            Log.Info("Mod loaded successfully");

            isInitialized = true;
        }


        public override void UpdateBeforeSimulation()
        {
            try
            {
                if (!isInitialized)
                {
                    if (MyAPIGateway.Session == null) return;
                    Init();
                }
                else
                {

                    var currentFrame = MyAPIGateway.Session.GameplayFrameCounter;
                    if (currentFrame - _lastUpdateFrame < _updateInterval)
                        return;

                    _lastUpdateFrame = currentFrame;

                    // Periodic cleanup and maintenance
                    CleanupInvalidBlocks();

                    // Optional: Log active AI block counts
                    if (updateCounter % (UPDATE_INTERVAL * 10) == 0) // Every 10 seconds
                    {
                        Log.Verbose("Active AI Drone Schedulers: {0}", AIDroneSchedulers.Count);
                        Log.Verbose("Active AI Drone Controllers: {0}", AIDroneControllers.Count);
                        Log.Verbose("Active AI Logistics Computers: {0}", AILogisticsComputers.Count);
                    }
                }

            }
            catch (Exception ex)
            {
                MyAPIGateway.Utilities.ShowMessage("ImprovedAI", $"Update error: {ex.Message}");
                MyLog.Default.WriteLine($"ImprovedAI: UpdateBeforeSimulation exception: {ex}");
            }
        }

        private void CleanupInvalidBlocks()
        {
            var schedulersToRemove = new List<long>();
            foreach (var kvp in AIDroneSchedulers)
            {
                if (kvp.Value?.Entity == null || kvp.Value.Entity.MarkedForClose)
                {
                    schedulersToRemove.Add(kvp.Key);
                }
            }

            foreach (var entityId in schedulersToRemove)
            {
                AIDroneSchedulers.Remove(entityId);
            }

            var controllersToRemove = new List<long>();
            foreach (var kvp in AIDroneControllers)
            {
                if (kvp.Value?.Entity == null || kvp.Value.Entity.MarkedForClose)
                {
                    controllersToRemove.Add(kvp.Key);
                }
            }

            foreach (var entityId in controllersToRemove)
            {
                AIDroneControllers.Remove(entityId);
            }

            var logisticsToRemove = new List<long>();
            foreach (var kvp in AILogisticsComputers)
            {
                if (kvp.Value?.Entity == null || kvp.Value.Entity.MarkedForClose)
                {
                    logisticsToRemove.Add(kvp.Key);
                }
            }

            foreach (var entityId in logisticsToRemove)
            {
                AILogisticsComputers.Remove(entityId);
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
                Log.Info("Unregistered AI Scheduler: {0}", entityId);
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
                Log.Info("Unregistered AI Drone controller: {0}", entityId);
            }
        }

        public void RegisterLogisticsComputer(IAILogisticsComputer block)
        {
            if (block?.Entity != null)
            {
                AILogisticsComputers[block.Entity.EntityId] = block;
                Log.Info("Registered AI Logistics Computer: {0} {1}", block.Entity.EntityId, block.Entity.DisplayName);
            }
        }

        public void UnregisterLogisticsComputer(long entityId)
        {
            if (AILogisticsComputers.Remove(entityId))
            {
                Log.Info("Unregistered AI Logistics Computer: {0}", entityId);
            }
        }

        protected override void UnloadData()
        {
            try
            {
                // Clean shutdown
                AIDroneControllers.Clear();
                AIDroneSchedulers.Clear();
                AILogisticsComputers.Clear();

                // Reset message queue singleton
                Network.MessageQueue.Reset();

                // Unload terminal controls
                TerminalControls.Unload();

                Instance = null;
                Log.Info("Mod unloaded");
                Log.Close();
            }
            catch (Exception ex)
            {
                MyLog.Default.WriteLine($"ImprovedAI: UnloadData exception: {ex}");

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