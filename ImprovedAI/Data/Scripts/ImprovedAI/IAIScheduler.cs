using ImprovedAI;
using ImprovedAI.BlockConfig;
using ImprovedAI.Config;
using ImprovedAI.Messages;
using ImprovedAI.Network;
using ImprovedAI.Utils;
using ImprovedAI.Utils.Logging;
using ProtoBuf;
using Sandbox.Engine.Utils;
using Sandbox.ModAPI;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.ModAPI;
using VRage.ModAPI;
using VRage.Utils;
using VRageMath;
using static ImprovedAI.Config.ServerConfig;

namespace ImprovedAIScheduler
{
    public enum TargetSorting
    {
        ClosestFirst,
        FurthestFirst,
    }

    public enum SchedulerOperationMode : byte
    {
        StandAloneDroneScheduler,
        /// <summary>
        /// Receive messages from another scheduler and forward those message sto your drones.
        /// Useful for sending drones outside antenna range, and letting their own scheduler kick in.
        /// </summary>
        DelegatedScheduler,
        /// <summary>
        /// Standard operation mode. Assign tasks to multiple drones, receive updates from multiple drones.
        /// </summary>
        Orchestrator
    }
    public enum SchedulerState : byte
    {
        Initializing,
        Standby,
        ScanningForTasks,
        AssigningTasks,
        Error
    }
    [Flags]
    public enum SearchModes : byte
    {
        ConnectedGrids = 1,
        BoundingBox = 2
    }
    [Flags]
    public enum WorkModes : ushort
    {
        None = 0,
        Scan = 1,
        WeldUnfinishedBlocks = 2,
        RepairDamagedBlocks = 4,
        WeldProjectedBlocks = 8,
        Grind = 16,
        DeliverCargo = 32,
        FetchCargo = 64,
        DropCargo = 128,
        AirDropCargo = 256,
    }

    [Flags]
    public enum DroneCapabilities
    {
        None = 0,
        CanWeld = 1,
        CanGrind = 2,
        CanDrill = 4,
        CanAirDrop = 8,
        HasWheels = 16,
        CanFlyAtmosphere = 32,
        CanFlySpace = 64,
        RefuelWhenDocked = 128,
        RechargeWhenDocked = 256,
        CombatReady = 512
    }
    public class Drone
    {
        public long DroneId;
        public DroneCapabilities Capabilities;
        public DroneState DroneState;
        public float BatteryLevel;
        public float BatteryRechargeThreshold;
        public float BatteryOperationalThreshold;
        public float H2Level;
        public float H2RefuelThreshold;
        public float H2OperationalThreshold;
        public Drone() { }
        public Drone(long droneId, DroneCapabilities capabilities, DroneState droneState, float batteryLevel, float batteryRechargeThreshold, float batteryOperationalThreshold, float h2Level, float h2RechargeThreshold, float h2OperationalThreshold)
        {
            DroneId = droneId;
            Capabilities = capabilities;
            DroneState = droneState;
            BatteryLevel = batteryLevel;
            BatteryRechargeThreshold = batteryRechargeThreshold;
            BatteryOperationalThreshold = batteryOperationalThreshold;
            H2Level = h2Level;
            H2RefuelThreshold = h2RechargeThreshold;
            H2OperationalThreshold = h2OperationalThreshold;
        }
        public Drone(Drone drone)
        {
            DroneId = drone.DroneId;
            Capabilities = drone.Capabilities;
            DroneState = drone.DroneState;
            BatteryLevel = drone.BatteryLevel;
            BatteryRechargeThreshold = drone.BatteryRechargeThreshold;
            BatteryOperationalThreshold = drone.BatteryOperationalThreshold;
            H2Level = drone.H2Level;
            H2RefuelThreshold = drone.H2RefuelThreshold;
            H2OperationalThreshold = drone.H2OperationalThreshold;
        }
    }

    public struct LogisticsGrid
    {
        public long GridId;
        public LogisticsOperationMode OperationMode;
        /// <summary>
        /// Positions of available connectors.
        /// </summary>
        public Vector3D connectors;
        public IAIInventory LastKnownInventory;
    }
    internal class IAIScheduler
    {
        private MyConcurrentDictionary<long, Drone> registeredDrones;
        private MyConcurrentDictionary<long, LogisticsGrid> registeredLogisticsGrids;
        /// <summary>
        /// Queue of discovered, but yet unassigned tasks.
        /// </summary>
        private MyConcurrentQueue<Task> taskQueue;
        /// <summary>
        /// Dictionary of drone task assignments, that are awaiting completion.
        /// </summary>
        private MyConcurrentDictionary<long, List<Task>> assignedTasks;
        private SchedulerState currentState;
        private SchedulerOperationMode operationMode;
        private WorkModes workModes;
        private long entityId;


        private static Dictionary<long, AntennaInfo> antennaCache = new Dictionary<long, AntennaInfo>();
        private static readonly TimeSpan CACHE_UPDATE_INTERVAL = TimeUtil.TickToTimeSpan(ServerConfig.DroneNetwork.SchedulerAntennaCacheUpdateIntervalTicks);
        private static readonly int PerScanLimits = ServerConfig.SchedulerBounds.PerScanLimit;
        private static readonly int ScanTargetLimit = ServerConfig.SchedulerBounds.MaxTargetLimit;
        private static DateTime lastCacheUpdate;

        private static readonly MessageTopics[] DroneManagementTopics = new MessageTopics[]
        {
            MessageTopics.DRONE_REGISTRATION,
            MessageTopics.DRONE_REPORTS,
            MessageTopics.DRONE_PERFORMANCE,
        };
        private static readonly MessageTopics[] logisticsManagementTopics = new MessageTopics[]
         {
            MessageTopics.LOGISTIC_REGISTRATION,
            MessageTopics.LOGISTIC_UPDATE,
            MessageTopics.LOGISTIC_REQUEST,
            MessageTopics.LOGISTIC_PUSH,
        };

        private MessageQueue messaging = new MessageQueue();
        private int messageReadLimit = DroneNetwork.SchedulerMessageReadLimit;
        private List<IMySlimBlock> targetBlocks = new List<IMySlimBlock>();
        private IMyEntity Entity;
        private IMyRadioAntenna ownAntenna;
        private int broadcastUpdatesChannel = 1;
        private bool isEnabled;
        private readonly object _taskLock = new object();

        private bool ignoreWeldColorEnabled;
        private Vector3 weldIgnoreColor;

        private bool grindColorEnabled;
        private Vector3 grindColor;

        private Dictionary<TaskType, List<Task>> pendingTasks;

        private struct AntennaInfo
        {
            public Vector3D Position;
            public double Range;
            public bool IsWorking;
            public long GridId;
            public DateTime LastUpdate;
        }

        public IAIScheduler(
            IMyEntity entity,
            SchedulerOperationMode operationMode = SchedulerOperationMode.Orchestrator,
            WorkModes workModes = WorkModes.None)
        {
            this.entityId = entity.EntityId;
            this.Entity = entity;
            this.operationMode = operationMode;
            this.workModes = workModes;
        }

        private void Initialize()
        {
            currentState = SchedulerState.Initializing;
            long schedulerId = this.entityId;

            foreach (var topic in DroneManagementTopics)
            {
                Log.Info("scheduler {0} subscribing to topic {1}", schedulerId, MessageUtil.TopicToString(topic));
                messaging.Subscribe(entityId, (ushort)topic);
            }
            foreach (var topic in DroneManagementTopics)
            {
                Log.Info("scheduler {0} subscribing to topic {1}", schedulerId, MessageUtil.TopicToString(topic));
                messaging.Subscribe(entityId, (ushort)topic);
            }
        }

        public void TrySendMessage()
        {

        }
        public enum SchedulerState
        {
            Initializing,
            Standby,
            ScanningForTasks,
            AssigningTasks,
            Error
        }
        private void UpdateAI()
        {
            switch (currentState)
            {
                case SchedulerState.Initializing:
                    HandleInitializing();
                    break;
                case SchedulerState.Standby:
                    HandleStandby();
                    break;
                case SchedulerState.ScanningForTasks:
                    HandleScanningForTasks();
                    break;
                case SchedulerState.AssigningTasks:
                    HandleTaskAssignment();
                    break;
                case SchedulerState.Error:
                    HandleError();
                    break;
            }
        }
        private void HandleInitializing()
        {
            if (CheckCapabilities())
            {
                currentState = SchedulerState.Standby;
            }
            else
            {
                currentState = SchedulerState.Error;
            }
        }


        private void HandleStandby()
        {
            if (ShouldStartScanning())
            {
                currentState = SchedulerState.ScanningForTasks;
                //statusMessage = "Scanning for construction targets...";
            }
        }

        private bool ShouldStartScanning()
        {
            var workModeEval = workModes & (WorkModes.Scan | WorkModes.Grind | WorkModes.FetchCargo | WorkModes.WeldUnfinishedBlocks);
            return isEnabled && ownAntenna != null && workModeEval > 0;
        }
        /// <summary>
        /// Scan connected grids for radio antennas. Select only the largest range for broadcasting.
        /// Only selects mechanically connected grids.
        /// </summary>
        /// <returns></returns>
        private bool CheckCapabilities()
        {
            var thisBlock = Entity as IMyCubeBlock;
            if (thisBlock == null) return false;
            IMyRadioAntenna localAntenna = null;

            List<IMyCubeGrid> connectedGrids = new List<IMyCubeGrid>();
            MyAPIGateway.GridGroups.GetGroup(thisBlock.CubeGrid, GridLinkTypeEnum.Mechanical, connectedGrids);

            foreach (var grid in connectedGrids)
            {
                var blocks = new List<IMySlimBlock>();
                grid.GetBlocks(blocks);

                foreach (var block in blocks)
                {
                    if (block is IMyRadioAntenna)
                    {
                        if (localAntenna == null) localAntenna = (IMyRadioAntenna)block;
                        else if (((IMyRadioAntenna)block).Radius > localAntenna.Radius)
                            localAntenna = (IMyRadioAntenna)block;
                    }
                }
            }
            ownAntenna = localAntenna;

            return ownAntenna != null &&
                ownAntenna.IsFunctional &&
                ownAntenna.Enabled &&
                ownAntenna.EnableBroadcasting &&
                ownAntenna.IsWorking;
        }



        private void HandleScanningForTasks()
        {
            ScanForWeldRepairGrindTasks();
            ReadMessages();
            ScanForLogisticTasks();
        }

        private void HandleTaskAssignment()
        {
            if (registeredDrones == null || registeredDrones.Count == 0)
            {
                Log.Info("No available drones");
                return;
            }
        }
        private static void UpdateAntennaCache(IMyRadioAntenna schedulerAntenna)
        {
            var now = DateTime.Now;
            if (now - lastCacheUpdate < CACHE_UPDATE_INTERVAL)
                return;

            lastCacheUpdate = now;
            antennaCache.Clear();

            try
            {
                var allAntennas = new HashSet<IMyEntity>();
                MyAPIGateway.Entities.GetEntities(allAntennas);

                foreach (var antenna in allAntennas.Cast<IMyRadioAntenna>())
                {
                    if (antenna?.CubeGrid == null) continue;

                    // shouldn't be talking to antennas out of range
                    var distance = Vector3D.Distance(schedulerAntenna.GetPosition(), antenna.GetPosition());
                    if (distance < schedulerAntenna.Radius)
                    {
                        antennaCache.Remove(antenna.EntityId);
                        continue;
                    }

                    var antennaInfo = new AntennaInfo
                    {
                        Position = antenna.GetPosition(),
                        Range = antenna.IsWorking && antenna.EnableBroadcasting ? antenna.Radius : 0.0,
                        IsWorking = antenna.IsWorking && antenna.EnableBroadcasting,
                        GridId = antenna.CubeGrid.EntityId,
                        LastUpdate = now
                    };

                    antennaCache[antenna.EntityId] = antennaInfo;
                }
            }
            catch (Exception ex)
            {
                Log.Error($"Antenna cache update failed: {ex}");
            }
        }

        private bool ShouldTargetBlock(IMySlimBlock block)
        {
            if (workModes.HasFlag(WorkModes.WeldUnfinishedBlocks) &&
                    block.BuildLevelRatio < 1.0f &&
                    ignoreWeldColorEnabled &&
                    !ColorUtil.ColorMatch(block, weldIgnoreColor))
            {
                return true;
            }
            if (workModes.HasFlag(WorkModes.RepairDamagedBlocks) &&
                    block.CurrentDamage > 0.0f &&
                    ignoreWeldColorEnabled &&
                    !ColorUtil.ColorMatch(block, weldIgnoreColor))
                return true;
            if (workModes.HasFlag(WorkModes.Grind) &&
                grindColorEnabled &&
                ColorUtil.ColorMatch(block, grindColor))
                return true;

            return false;
        }

        private void ScanForWeldRepairGrindTasks()
        {
            targetBlocks.Clear();
            var scannedBlocks = 0;

            List<IMyCubeGrid> connectedGrids = new List<IMyCubeGrid>();
            var cubeBlock = Entity as IMyCubeBlock;
            if (cubeBlock?.CubeGrid == null)
            {
                Log.Error("cannot scan: entity is {0} not an IMyCubeBlock", Entity.EntityId);
                return;
            }
            try
            {
                MyAPIGateway.GridGroups.GetGroup(cubeBlock.CubeGrid, GridLinkTypeEnum.Physical, connectedGrids);
            }
            catch (Exception ex)
            {
                Log.Error("Failed to get connected grids: {0}", ex.Message);
                return;
            }

            foreach (var grid in connectedGrids)
            {
                var blocks = new List<IMySlimBlock>();
                grid.GetBlocks(blocks);

                foreach (var block in blocks)
                {
                    scannedBlocks++;
                    if (scannedBlocks > PerScanLimits)
                    {
                        // Continue scanning next update
                        return;
                    }

                    if (ShouldTargetBlock(block))
                    {
                        targetBlocks.Add(block);
                        if (targetBlocks.Count >= ScanTargetLimit)
                            return;
                    }
                }
            }

            SortTargetBlocks(TargetSorting.ClosestFirst);
        }

        private void SortTargetBlocks(TargetSorting method)
        {
            var aiPos = Entity.GetPosition();
            if (method == TargetSorting.ClosestFirst)
            {
                targetBlocks.Sort((a, b) =>
                {
                    var distA = Vector3D.DistanceSquared(GetBlockWorldPosition(a), aiPos);
                    var distB = Vector3D.DistanceSquared(GetBlockWorldPosition(b), aiPos);
                    return distA.CompareTo(distB);
                });
            }
            else if (method == TargetSorting.FurthestFirst)
            {
                targetBlocks.Sort((a, b) =>
                {
                    var distA = Vector3D.DistanceSquared(GetBlockWorldPosition(a), aiPos);
                    var distB = Vector3D.DistanceSquared(GetBlockWorldPosition(b), aiPos);
                    return distB.CompareTo(distA);
                });
            }
        }

        private Vector3D GetBlockWorldPosition(IMySlimBlock block)
        {
            return block.CubeGrid.GridIntegerToWorld(block.Position);
        }


        private void ReadMessages()
        {

        }


        private void ReadDroneRegistrations()
        {
            var droneReports = messaging.ReadMessages<DroneReport>(entityId, (ushort)MessageTopics.DRONE_REGISTRATION, 50);
            Drone drone;
            foreach (var reg in droneReports)
            {
                // if exists, redirect the message to drone report queue
                if (registeredDrones.TryGetValue(reg.DroneEntityId, out drone))
                {
                    messaging.SendMessage<DroneReport>((ushort)MessageTopics.DRONE_REPORTS, reg, this.entityId, false);
                    continue;
                }
                drone = new Drone(
                    reg.DroneEntityId,
                    reg.Capabilities.GetValueOrDefault(DroneCapabilities.None), // if this drone reported without capabilities, its probably unusable
                    reg.DroneState.GetValueOrDefault(DroneState.Standby),
                    reg.BatteryChargePercent.GetValueOrDefault(100.0f),
                    reg.BatteryRechargeThreshold.GetValueOrDefault(25.0f),
                    reg.BatteryOperationalThreshold.GetValueOrDefault(80.0f),
                    reg.H2Level.GetValueOrDefault(100.0f),
                    reg.H2RefuelThreshold.GetValueOrDefault(25.0f),
                    reg.H2OperationalThreshold.GetValueOrDefault(80.0f)
                   );

                registeredDrones.Add(reg.DroneEntityId, drone);
            }
        }
        //public enum DroneUpdateFlags : byte
        //{
        //    [ProtoEnum]
        //    None = 0,
        //    [ProtoEnum]
        //    Error = 1,
        //    [ProtoEnum]
        //    Registration = 2,
        //    [ProtoEnum]
        //    TaskComplete = 4,
        //    [ProtoEnum]
        //    StateChanged = 8,
        //    [ProtoEnum]
        //    CapabilitiesChanged = 16,
        //    [ProtoEnum]
        //    BatteryChargeUpdate = 32,
        //    [ProtoEnum]
        //    H2LevelsUpdate = 64,
        //    [ProtoEnum]
        //    GoingOutOfRange = 128,
        //}
        private void ReadDroneReports(ushort maxMessages)
        {
            var droneReports = messaging.ReadMessages<DroneReport>(entityId, (ushort)MessageTopics.DRONE_REPORTS, maxMessages);
            foreach (var report in droneReports)
            {
                Drone drone;
                if (registeredDrones.TryGetValue(report.DroneEntityId, out drone))
                {
                    if (report.Flags.HasFlag(DroneUpdateFlags.StateChanged))
                        drone.DroneState = report.DroneState.GetValueOrDefault(drone.DroneState);
                    if (report.Flags.HasFlag(DroneUpdateFlags.CapabilitiesChanged))
                        drone.Capabilities = report.Capabilities.GetValueOrDefault(drone.Capabilities);
                    if (report.Flags.HasFlag(DroneUpdateFlags.BatteryUpdate))
                    {
                        drone.BatteryLevel = report.BatteryChargePercent.GetValueOrDefault(drone.BatteryLevel);
                        drone.BatteryRechargeThreshold = report.BatteryRechargeThreshold.GetValueOrDefault(drone.BatteryRechargeThreshold);
                        drone.BatteryOperationalThreshold = report.BatteryOperationalThreshold.GetValueOrDefault(drone.BatteryOperationalThreshold);
                    }
                    if (report.Flags.HasFlag(DroneUpdateFlags.H2Update))
                    {
                        drone.H2Level = report.H2Level.GetValueOrDefault(drone.H2Level);
                        drone.H2RefuelThreshold = report.H2RefuelThreshold.GetValueOrDefault(drone.H2RefuelThreshold);
                        drone.H2OperationalThreshold = report.H2OperationalThreshold.GetValueOrDefault(drone.H2OperationalThreshold);
                    }
                    if (report.Flags.HasFlag(DroneUpdateFlags.TaskComplete))
                    {
                        lock (_taskLock)
                        {
                            List<Task> taskList;
                            if (assignedTasks.TryGetValue(report.DroneEntityId, out taskList))
                            {
                                var taskIndex = taskList.FindIndex(t => t.TaskId == report.TaskId);
                                if (taskIndex >= 0)
                                {
                                    var completedTask = taskList[taskIndex];
                                    Log.Info("AI scheduler {0} task {1} completed by drone {2}, removing from assigned tasks", entityId, completedTask.TaskId, report.DroneEntityId);
                                    taskList.RemoveAt(taskIndex);
                                    if (taskList.Count == 0)
                                    {
                                        List<Task> removedList;
                                        assignedTasks.TryRemove(report.DroneEntityId, out removedList);
                                        Log.Info("AI scheduler {0}: All tasks completed for drone {1}, removing from assigned tasks", entityId, report.DroneEntityId);
                                    }
                                }
                                else
                                {
                                    Log.Warning("Task {0} not found for drone {1}", report.TaskId, report.DroneEntityId);
                                }
                            }
                            else
                            {
                                Log.Warning(
                                    "received report of a task assigned to a non-existing drone. Scheduler: {0}, droneId {1}",
                                    entityId,
                                    report.DroneEntityId);
                            }
                        }
                    }
                    // GoingOutOfRange unregisters, must be evaluated last
                    if (report.Flags.HasFlag(DroneUpdateFlags.GoingOutOfRange))
                    {

                    }
                }
                else
                {
                    // if not registered, redirect to registration queue
                    messaging.SendMessage<DroneReport>((ushort)MessageTopics.DRONE_REGISTRATION, report, this.entityId, false);
                }
            }
        }

        private void ScanForLogisticTasks()
        {
            // check logistic requests
            // see which providers can satisfy
            // try single provider
            // else multiple providers

        }
        // Send a message to players within antenna range
        //public void BroadcastToNearbyPlayers(Vector3D position, string message, string senderName = "IAIScheduler")
        //{
        //    try
        //    {
        //        if (!MyAPIGateway.Session.IsServer) return;

        //        var players = new List<IMyPlayer>();
        //        MyAPIGateway.Players.GetPlayers(players);

        //        foreach (var player in players)
        //        {
        //            var playerPosition = player.GetPosition();
        //            var distance = Vector3D.Distance(position, playerPosition);

        //            if (distance <= ownAntenna.Radius)
        //            {
        //                MyAPIGateway.Utilities.ShowMessage(player.SteamUserId, senderName, message);
        //            }
        //        }
        //    }
        //    catch (Exception ex)
        //    {
        //        MyAPIGateway.Utilities.ShowMessage("ChatBroadcastError", ex.Message);
        //    }
        }
        private void HandleError()
        {
            consecutiveErrors++;

            if (consecutiveErrors >= MAX_CONSECUTIVE_ERRORS)
            {
                // Enter safe mode - minimal operations only
                navigationManager?.StopAllMovement();
                statusMessage = "Critical errors - entering safe mode";

                // Try to dock if possible
                if (connector != null && !connector.IsConnected)
                {
                    var dockingPos = connector.WorldMatrix.Translation;
                    var distance = Vector3D.Distance(GetPosition(), dockingPos);
                    if (distance < 100.0) // Only if reasonably close
                    {
                        currentState = AIState.ReturningToBase;
                        consecutiveErrors = 0; // Reset on recovery attempt
                    }
                }
            }
            else
            {
                // Normal error handling
                navigationManager?.StopAllMovement();
                statusMessage = $"Error state - attempt {consecutiveErrors}/{MAX_CONSECUTIVE_ERRORS}";

                // Try to recover every 30 seconds instead of 60
                if (tickCounter % 300 == 0)
                {
                    if (CheckCapabilities())
                    {
                        currentState = AIState.Standby;
                        consecutiveErrors = 0;
                        displayManager?.ShowInfo("Recovered from error state");
                    }
                }
            }
        }
        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        private float ComputeRequiredElectricPower()
        {
            if (Entity == null) return 0f;
            var required = 0f;
            if (_Welder.Enabled)
            {
                required += Settings.MaximumRequiredElectricPowerStandby;
                required += _PowerWelding || State.Welding ? Settings.MaximumRequiredElectricPowerWelding : 0f;
                required += _PowerGrinding || State.Grinding ? Settings.MaximumRequiredElectricPowerGrinding : 0f;
                required += _PowerTransporting || State.Transporting ? (Settings.SearchMode == SearchModes.Grids ? Settings.MaximumRequiredElectricPowerTransport / 10 : Settings.MaximumRequiredElectricPowerTransport) : 0f;
            }
            if (MyAPIGateway.Session.IsServer && Mod.Log.ShouldLog(Logging.Level.Info)) Mod.Log.Write(Logging.Level.Info, "BuildAndRepairSystemBlock {0}: ComputeRequiredElectricPower Enabled={1} Required={1}", Logging.BlockName(_Welder, Logging.BlockNameOptions.None), _Welder.Enabled, required);
            return required;
        }

    }
}