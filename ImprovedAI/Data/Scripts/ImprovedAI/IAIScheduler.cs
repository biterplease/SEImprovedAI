using ImprovedAI;
using ImprovedAI.BlockConfig;
using ImprovedAI.Config;
using ImprovedAI.Messages;
using ImprovedAI.Network;
using ImprovedAI.Utils;
using ImprovedAI.Utils.Logging;
using ProtoBuf;
using Sandbox.Engine.Utils;
using Sandbox.Game.Entities;
using Sandbox.ModAPI;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.ModAPI;
using VRage.ModAPI;
using VRage.ObjectBuilders;
using VRage.Utils;
using VRageMath;
using static ImprovedAI.Config.ServerConfig;
using static VRage.Game.MyObjectBuilder_BehaviorTreeDecoratorNode;

namespace ImprovedAIScheduler
{
    [Flags]
    public enum SchedulerOperationMode : byte
    {
        None = 0,
        StandAloneDroneScheduler = 1,
        /// <summary>
        /// Forward messages to other schedulers.
        /// </summary>
        Repeater = 2,
        /// <summary>
        /// Standard operation mode. Assign tasks to multiple drones, receive updates from multiple drones.
        /// </summary>
        Orchestrator = 4,
        /// <summary>
        /// If no drones are available to this scheduler, forward found tasks to other schedulers.
        /// </summary>
        DelegateIfNoDrones = 8,
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
        public long DroneId { get; set; }
        public DroneCapabilities Capabilities { get; set; }
        public DroneState DroneState { get; set; }
        public float BatteryLevel { get; set; }
        public float BatteryRechargeThreshold { get; set; }
        public float BatteryOperationalThreshold { get; set; }
        public float H2Level { get; set; }
        public float H2RefuelThreshold { get; set; }
        public float H2OperationalThreshold { get; set; }
        public bool IsOutOfRange { get; set; }
        public DateTime LastSeenTime { get; set; }
        public DateTime LastReportTime { get; set; }
        public Drone()
        {
            Capabilities = DroneCapabilities.None;
            DroneState = DroneState.Standby;
            BatteryLevel = 25.0f;
            BatteryRechargeThreshold = 20.0f;
            BatteryOperationalThreshold = 80.0f;
            H2Level = 25.0f;
            H2RefuelThreshold = 20.0f;
            H2OperationalThreshold = 80.0f;
            IsOutOfRange = false;
            LastSeenTime = DateTime.UtcNow;
            LastReportTime = DateTime.UtcNow;
        }
        public Drone(
            long droneId,
            DroneCapabilities capabilities = DroneCapabilities.None,
            DroneState droneState = DroneState.Standby,
            float batteryLevel = 25.0f,
            float batteryRechargeThreshold = 20.0f,
            float batteryOperationalThreshold = 80.0f,
            float h2Level = 25.0f,
            float h2RechargeThreshold = 20.0f,
            float h2OperationalThreshold = 80.0f,
            bool isOutOfRange = false
            )
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
            IsOutOfRange = isOutOfRange;
            LastSeenTime = DateTime.UtcNow;
            LastReportTime = DateTime.UtcNow;
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
            IsOutOfRange = drone.IsOutOfRange;
            LastReportTime = drone.LastReportTime;
            LastSeenTime = drone.LastSeenTime;
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
        private MyConcurrentDictionary<long, Drone> registeredDrones = new MyConcurrentDictionary<long, Drone>();
        private MyConcurrentDictionary<long, LogisticsGrid> registeredLogisticsGrids = new MyConcurrentDictionary<long, LogisticsGrid>();
        /// <summary>
        /// Queue of discovered, but yet unassigned tasks.
        /// </summary>
        private MyConcurrentQueue<Task> taskQueue = new MyConcurrentQueue<Task>();
        /// <summary>
        /// Dictionary of drone task assignments, that are awaiting completion.
        /// </summary>
        private MyConcurrentDictionary<long, List<Task>> assignedTasks = new MyConcurrentDictionary<long, List<Task>>();
        /// <summary>
        /// MessageId to TaskId mapping of delegated tasks. Once the message is confirmed as acked,
        /// this scheduler can safely remove the task, as it has been picked up by a different scheduler.
        /// </summary>
        private MyConcurrentDictionary<ushort, uint> delegatedTasksNeedingAck = new MyConcurrentDictionary<ushort, uint>();
        private SchedulerState currentState;
        private SchedulerOperationMode operationMode;
        private WorkModes workModes;
        private long entityId;
        private readonly IdGenerator _idGenerator;
        private int _lastUpdateFrame = 0;
        private int _lastScanFrameAttempt = 0;
        private int _consecutiveErrors = 0;
        private long _lastErrorRecoveryAttemptFrame = 0;
        private long _lastMaintenanceFrame = 0;


        private static Dictionary<long, AntennaInfo> antennaCache = new Dictionary<long, AntennaInfo>();
        private static readonly TimeSpan CACHE_UPDATE_INTERVAL = TimeUtil.TickToTimeSpan(ServerConfig.DroneNetwork.SchedulerAntennaCacheUpdateIntervalTicks);
        private static readonly int ScanTargetLimit = ServerConfig.SchedulerBounds.MaxTargetLimit;
        public int PerScanLimits { get; set; }
        private int _perScanLimits;
        public int MaxTasksAssignedPerBatch { get; set; }
        private int _maxTasksAssignedPerBatch;

        private int _initializingUpdateIntervalTicks = SchedulerBounds.StateUpdateIntervalTicks.Initializing;
        private int _errorUpdateIntervalTicks = SchedulerBounds.StateUpdateIntervalTicks.Error;
        private int _standbyUpdateIntervalTicks = SchedulerBounds.StateUpdateIntervalTicks.Standby;
        private int _scanningUpdateIntervalTicks = SchedulerBounds.StateUpdateIntervalTicks.Scanning;
        private int _assigningUpdateIntervalTicks = SchedulerBounds.StateUpdateIntervalTicks.Assigning;
        private int _scanRetryIntervalTicks = SchedulerBounds.ScanDelayTicks;
        private int _errorRecoveryIntervalTicks = SchedulerBounds.ErrorRecoveryIntervalTicks;
        private int _maxConsecutiveErrors = SchedulerBounds.MaxConsecutiveErrors;
        private int _maintenanceIntervalTicks = SchedulerBounds.ManintenanceIntervalTicks;
        /// <summary>
        /// Timeout after which drones are removed if we don't hear from them again.
        /// </summary>
        private static readonly int droneTimeoutSeconds = 1800;
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
        private int messageReadLimit = ServerConfig.DroneNetwork.SchedulerMessageReadLimit;
        private IMyEntity Entity;
        private IMyRadioAntenna ownAntenna;
        private int broadcastUpdatesChannel = 1;
        private bool isEnabled;
        private readonly object _taskLock = new object();
        /// <summary>
        /// Range in meters after which tasks are ignored.
        /// </summary>
        private bool ignoreTasksOutsideSpecifiedRange = false;
        private float ignoreTasksOutsideSpecifiedRangeMeters = 1000.0f;
        private bool ignoreTasksOutsideOfAntenaRange = true;


        // block settings
        private bool _initialized = false;


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
            _idGenerator = new IdGenerator(entityId);
            this.Entity = entity;
            this.operationMode = operationMode;
            this.workModes = workModes;

            _maxTasksAssignedPerBatch = MaxTasksAssignedPerBatch > SchedulerBounds.MaxTaskAssignmentPerBatch
                ? SchedulerBounds.MaxTaskAssignmentPerBatch
                : MaxTasksAssignedPerBatch;

            _perScanLimits = PerScanLimits > SchedulerBounds.PerScanLimit ? SchedulerBounds.PerScanLimit : PerScanLimits;
        }


        public void UpdateBeforeSimulation(MyGameLogicComponent _base)
        {
            try
            {
                _base.UpdateBeforeSimulation();
                if (!(AntennaOK() && _initialized)) return;
            }
            catch (Exception ex)
            {
                Log.Error(ex);
            }
        }

        public void Init(MyGameLogicComponent _base, MyObjectBuilder_EntityBase objectBuilder)
        {
            Log.Info("Initializing {0}", Log.BlockName(Entity));
            // This method is called async! Use UpdateOnceBeforeFrame for proper initialization
            _base.Init(objectBuilder);
            _base.NeedsUpdate = MyEntityUpdateEnum.EACH_FRAME | MyEntityUpdateEnum.EACH_100TH_FRAME;

        }


        public void UpdateBeforeSimulation10(MyGameLogicComponent _base)
        {
            _base.UpdateBeforeSimulation10();
            if (!AntennaOK() && currentState != SchedulerState.Error && _initialized)
            {
                Log.Warning("Scheduler {0} antenna check failed, transitioning to error state", entityId);
                currentState = SchedulerState.Error;
            }
            if (!_initialized && currentState != SchedulerState.Initializing)
            {
                Initialize();
                return;
            }
            var currentFrame = MyAPIGateway.Session.GameplayFrameCounter;
            var updateInterval = GetUpdateIntervalTicks();

            if (currentFrame - _lastUpdateFrame < updateInterval)
                return;

            _lastUpdateFrame = currentFrame;

            try
            {
                UpdateAI();

                if (currentState != SchedulerState.Error && currentState != SchedulerState.Initializing)
                {
                    ReadDroneRegistrations();
                    ReadDroneReports((ushort)messageReadLimit);
                }
            }
            catch (Exception ex)
            {
                Log.Error("Scheduler {0} update threw exception: {1}", entityId, ex.Message);
                currentState = SchedulerState.Error;
            }
        }

        private int GetUpdateIntervalTicks()
        {
            switch (currentState)
            {
                case SchedulerState.Initializing:
                    return _initializingUpdateIntervalTicks;
                case SchedulerState.Error:
                    return _errorUpdateIntervalTicks;
                case SchedulerState.Standby:
                    return _standbyUpdateIntervalTicks;
                case SchedulerState.ScanningForTasks:
                    return _scanningUpdateIntervalTicks;
                case SchedulerState.AssigningTasks:
                    return _assigningUpdateIntervalTicks;
                default:
                    return 600; // Default 10 second updates
            }
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
            messaging.Subscribe(entityId, (ushort)MessageTopics.SCHEDULER_FORWARD);
        }

        public enum SchedulerState
        {
            Initializing,
            Standby,
            /// <summary>
            /// Finds tasks in connected grids.
            /// </summary>
            ScanningForTasks,
            /// <summary>
            ///  Sends messages to drones and other schedulers.
            /// </summary>
            AssigningTasks,
            Error
        }
        private void UpdateAI()
        {
            var currentFrame = MyAPIGateway.Session.GameplayFrameCounter;

            // Perform maintenance periodically, regardless of state
            if (currentFrame - _lastMaintenanceFrame >= _maintenanceIntervalTicks)
            {
                _lastMaintenanceFrame = currentFrame;
                PerformDroneMaintenance();
            }
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
                if (AntennaOK())
                {
                    UpdateAntennaCache(ownAntenna);
                    currentState = SchedulerState.Standby;
                    _initialized = true;
                }
            }
            else
            {
                currentState = SchedulerState.Error;
            }
        }



        private void HandleStandby()
        {
            var currentFrame = MyAPIGateway.Session.GameplayFrameCounter;
            // Periodically check if we should scan
            if (currentFrame - _lastScanFrameAttempt >= _scanRetryIntervalTicks)
            {
                _lastScanFrameAttempt = currentFrame;
                if (ShouldStartScanning())
                {
                    Log.Verbose("Scheduler {0} waking from standby to scan for tasks", entityId);
                    currentState = SchedulerState.ScanningForTasks;
                }
            }
        }

        private bool ShouldStartScanning()
        {
            var workModeEval = workModes & (
                WorkModes.Scan |
                WorkModes.Grind |
                WorkModes.FetchCargo |
                WorkModes.DeliverCargo |
                WorkModes.WeldUnfinishedBlocks);
            return isEnabled && AntennaOK() && workModeEval > 0;
        }
        /// <summary>
        /// Scan connected grids for radio antennas. Select only the largest range for broadcasting.
        /// Only selects mechanically connected grids (rotor, piston, wheel suspension).
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

        private bool AntennaOK()
        {
            return ownAntenna != null &&
                ownAntenna.IsFunctional &&
                ownAntenna.Enabled &&
                ownAntenna.EnableBroadcasting &&
                ownAntenna.IsWorking;
        }



        private void HandleScanningForTasks()
        {
            var totalTasks = 0;
            try
            {
                totalTasks += ScanForWeldRepairGrindTasks();
                totalTasks += ScanForLogisticTasks();
            }
            catch (Exception ex)
            {
                Log.Error("Scheduler {0} Error during task scanning: {1}", entityId, ex.Message);
                currentState = SchedulerState.Error;
                return;
            }

            if (totalTasks == 0)
            {
                Log.Verbose("Scheduler {0] going on standby, no tasks found.", entityId);
                currentState = SchedulerState.Standby;
                return;
            }
            Log.Verbose("Scheduler {0} found {1} tasks, transitioning to AssigningTasks", entityId, totalTasks);
            currentState = SchedulerState.AssigningTasks;
        }

        private void HandleTaskAssignment()
        {
            int assigned = 0;
            // scheduler set to only repeater task messages
            if (operationMode.Equals(SchedulerOperationMode.Repeater))
            {
                Task task;
                while (taskQueue.TryDequeue(out task))
                {
                    var needsAck = false; // repeater does not care
                    messaging.SendMessage((ushort)MessageTopics.SCHEDULER_FORWARD, task, entityId, needsAck);
                }
            }
            if (registeredDrones.Count != 0 && operationMode.HasFlag(SchedulerOperationMode.Orchestrator))
            {
            }
            // no registered drones
            if (registeredDrones.Count == 0 && operationMode.HasFlag(SchedulerOperationMode.DelegateIfNoDrones))
            {
                for (int i = 0; i < _maxTasksAssignedPerBatch; i++)
                {
                    Task task;
                    if (taskQueue.TryDequeue(out task))
                    {
                        var needsAck = true; // delegated messages should be checked, perhaps there is no onet o pick them up
                        var messageId = messaging.SendMessage((ushort)MessageTopics.SCHEDULER_FORWARD, task, entityId, needsAck);
                        delegatedTasksNeedingAck.Add(messageId, task.TaskId);

                        if (assigned > _maxTasksAssignedPerBatch)
                        {

                        }
                        assigned++;
                    }
                    else
                    { // no more tasks to deque
                        break;
                    }
                }
            }
            if (registeredDrones.Count == 0 && !operationMode.HasFlag(SchedulerOperationMode.DelegateIfNoDrones))
                Log.Info("Scheduler {0}: No available drones", entityId);
            return;

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

        private bool ShouldWeldOrRepair(IMySlimBlock block)
        {
            if (workModes.HasFlag(WorkModes.WeldUnfinishedBlocks) &&
                    block.BuildLevelRatio < 1.0f &&
                    ignoreWeldColorEnabled &&
                    !ColorUtil.ColorMatch(block, weldIgnoreColor))
                return true;
            if (workModes.HasFlag(WorkModes.RepairDamagedBlocks) &&
                    block.CurrentDamage > 0.0f &&
                    ignoreWeldColorEnabled &&
                    !ColorUtil.ColorMatch(block, weldIgnoreColor))
                return true;
            return false;
        }
        private bool ShouldGrind(IMySlimBlock block)
        {
            if (workModes.HasFlag(WorkModes.Grind) &&
                grindColorEnabled &&
                ColorUtil.ColorMatch(block, grindColor))
                return true;

            return false;
        }

        private int ScanForWeldRepairGrindTasks()
        {
            var scannedBlocks = 0;
            var tasksCreated = 0;

            var cubeBlock = Entity as IMyCubeBlock;
            if (cubeBlock?.CubeGrid == null)
            {
                Log.Error("cannot scan: entity {0} is not an IMyCubeBlock", entityId);
                return tasksCreated;
            }

            var antennaPosition = ownAntenna.CubeGrid.GridIntegerToWorld(ownAntenna.Position);
            var antennaRadius = ownAntenna.Radius;

            List<IMyCubeGrid> connectedGrids = new List<IMyCubeGrid>();
            try
            {
                MyAPIGateway.GridGroups.GetGroup(cubeBlock.CubeGrid, GridLinkTypeEnum.Physical, connectedGrids);
            }
            catch (Exception ex)
            {
                Log.Error("Failed to get connected grids: {0}", ex.Message);
                return tasksCreated;
            }

            foreach (var grid in connectedGrids)
            {
                var blocks = new List<IMySlimBlock>();
                grid.GetBlocks(blocks);

                foreach (var block in blocks)
                {
                    if (++scannedBlocks > _perScanLimits)
                    {
                        Log.Verbose("Scan limit reached ({0} blocks), continuing next update", _perScanLimits);
                        // Continue scanning next update
                        return tasksCreated;
                    }
                    var shouldWeld = ShouldWeldOrRepair(block);
                    var shouldGrind = ShouldGrind(block);
                    if (!shouldWeld && !shouldGrind)
                        continue;

                    var blockPosition = grid.GridIntegerToWorld(block.Position);
                    var distance = Vector3D.Distance(antennaPosition, blockPosition);
                    var isOutOfAntennaRange = distance > antennaRadius;
                    var isOutOfSpecificRange = distance > ignoreTasksOutsideSpecifiedRangeMeters;
                    if (isOutOfAntennaRange && ignoreTasksOutsideOfAntenaRange) continue;
                    if (isOutOfSpecificRange && ignoreTasksOutsideSpecifiedRange) continue;

                    if (shouldWeld)
                    {
                        var componentsDict = new Dictionary<string, int>();
                        block.GetMissingComponents(componentsDict);
                        taskQueue.Enqueue(new Task
                        {
                            TaskId = _idGenerator.GenerateId(),
                            TaskType = TaskType.PreciseWelding,
                            Payload = new IAIInventory(componentsDict),
                            Position = blockPosition,
                            OutOfSchedulerRange = isOutOfAntennaRange
                        });
                        tasksCreated++;
                    }
                    if (shouldGrind)
                    {
                        taskQueue.Enqueue(new Task
                        {
                            TaskId = _idGenerator.GenerateId(),
                            TaskType = TaskType.PreciseGrinding,
                            Position = blockPosition,
                            OutOfSchedulerRange = isOutOfAntennaRange
                        });
                        tasksCreated++;
                    }
                }
            }
            if (tasksCreated > 0)
                Log.Info("Scan complete: {0} blocks scanned, {1} tasks created", scannedBlocks, tasksCreated);
            return tasksCreated;
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
                        HandleTaskCompletion(report.DroneEntityId, report.TaskId.Value);
                    // GoingOutOfRange unregisters, must be evaluated last
                    if (report.Flags.HasFlag(DroneUpdateFlags.GoingOutOfRange))
                    {
                        drone.IsOutOfRange = true;
                        drone.LastSeenTime = DateTime.UtcNow;
                        Log.LogDroneNetwork("AI scheduler {0}: Drone {1} going out of range",
                            entityId, report.DroneEntityId);
                    }
                    if (report.Flags.HasFlag(DroneUpdateFlags.ReturningIntoRange))
                    {
                        drone.IsOutOfRange = false;
                        drone.LastSeenTime = DateTime.UtcNow;
                        Log.LogDroneNetwork("AI scheduler {0}: Drone {1} returning into range", entityId, report.DroneEntityId);
                    }
                }
                else
                {
                    // if not registered, redirect to registration queue
                    messaging.SendMessage<DroneReport>((ushort)MessageTopics.DRONE_REGISTRATION, report, this.entityId, false);
                }
            }
        }

        private void HandleTaskCompletion(long droneId, ushort taskId)
        {
            lock (_taskLock)
            {
                List<Task> taskList;
                if (assignedTasks.TryGetValue(droneId, out taskList))
                {
                    var taskIndex = taskList.FindIndex(t => t.TaskId == taskId);
                    if (taskIndex >= 0)
                    {
                        var completedTask = taskList[taskIndex];
                        Log.Info("AI scheduler {0} task {1} completed by drone {2}, removing from assigned tasks",
                            entityId, completedTask.TaskId, droneId);
                        taskList.RemoveAt(taskIndex);

                        if (taskList.Count == 0)
                        {
                            List<Task> removedList;
                            assignedTasks.TryRemove(droneId, out removedList);
                            Log.Info("AI scheduler {0}: All tasks completed for drone {1}, removing from assigned tasks",
                                entityId, droneId);
                        }
                    }
                    else
                    {
                        Log.Warning("Task {0} not found for drone {1}", taskId, droneId);
                    }
                }
                else
                {
                    Log.Warning("received report of a task assigned to a non-existing drone. Scheduler: {0}, droneId {1}",
                        entityId, droneId);
                }
            }
        }
        private void PerformDroneMaintenance()
        {
            var now = DateTime.UtcNow;
            var timeout = TimeSpan.FromMinutes(30); // Configurable timeout
            var dronesTimedOut = new List<long>();

            foreach (var kvp in registeredDrones)
            {
                var drone = kvp.Value;

                // Check if drone hasn't been seen in too long
                if (now - drone.LastSeenTime > timeout)
                {
                    dronesTimedOut.Add(kvp.Key);
                }
            }

            // Clean up timed-out drones
            foreach (var droneId in dronesTimedOut)
            {
                Drone removedDrone;
                if (registeredDrones.TryRemove(droneId, out removedDrone))
                {
                    Log.Warning("AI scheduler {0}: Drone {1} timed out (last seen: {2}), removing from registry",
                        entityId, droneId, removedDrone.LastSeenTime);

                    lock (_taskLock)
                    {
                        List<Task> removedTasks;
                        if (assignedTasks.TryRemove(droneId, out removedTasks))
                        {
                            if (removedTasks.Count > 0)
                            {
                                Log.Warning("AI scheduler {0}: Returning {1} abandoned tasks to queue from timed-out drone {2}",
                                    entityId, removedTasks.Count, droneId);

                                // Return tasks to unassigned queue
                                foreach (var task in removedTasks)
                                {
                                    taskQueue.Enqueue(task);
                                }
                            }
                        }
                    }
                }
            }
        }

        private void ReadForwardedSchedulerMessages()
        {
            var droneReports = messaging.ReadMessages<DroneReport>(entityId, (ushort)MessageTopics.DRONE_REPORTS, (ushort)messageReadLimit);
            foreach (var report in droneReports) { }
        }

        private int ScanForLogisticTasks()
        {
            // check logistic requests
            // see which providers can satisfy
            // try single provider
            // else multiple providers
            return 0;

        }

        /// <summary>
        /// Reset scheduler to initial state - useful for manual recovery
        /// </summary>
        public void ResetScheduler()
        {
            Log.Info("Scheduler {0} manual reset requested", entityId);

            lock (_taskLock)
            {
                taskQueue.Clear();
                assignedTasks.Clear();
                delegatedTasksNeedingAck.Clear();
            }

            registeredDrones.Clear();
            registeredLogisticsGrids.Clear();

            _consecutiveErrors = 0;
            _initialized = false;
            isEnabled = false;

            currentState = SchedulerState.Initializing;

            Log.Info("Scheduler {0} reset complete", entityId);
        }
        private void HandleError()
        {
            var currentFrame = MyAPIGateway.Session.GameplayFrameCounter;

            _consecutiveErrors++;

            if (_consecutiveErrors >= _maxConsecutiveErrors)
            {
                // Critical error state - enter safe mode
                Log.Error("Scheduler {0} entered critical error state after {1} consecutive errors. Entering safe mode.",
                    entityId, _consecutiveErrors);

                // Disable the scheduler to prevent further issues
                isEnabled = false;

                // Clear any pending work to prevent cascading failures
                lock (_taskLock)
                {
                    var abandonedTaskCount = taskQueue.Count;
                    taskQueue.Clear();

                    if (abandonedTaskCount > 0)
                    {
                        Log.Warning("Scheduler {0} abandoned {1} queued tasks due to critical errors",
                            entityId, abandonedTaskCount);
                    }

                    // Log assigned tasks that are now orphaned
                    var totalAssignedTasks = 0;
                    foreach (var kvp in assignedTasks)
                    {
                        totalAssignedTasks += kvp.Value.Count;
                    }

                    if (totalAssignedTasks > 0)
                    {
                        Log.Warning("Scheduler {0} has {1} tasks still assigned to drones that may be orphaned",
                            entityId, totalAssignedTasks);
                    }
                }

                // Unsubscribe from all message topics to stop processing
                foreach (var topic in DroneManagementTopics)
                {
                    // Note: You'll need to implement Unsubscribe in MessageQueue if not already present
                    // messaging.Unsubscribe(entityId, (ushort)topic);
                }

                foreach (var topic in logisticsManagementTopics)
                {
                    // messaging.Unsubscribe(entityId, (ushort)topic);
                }

                // Stay in error state indefinitely - requires manual intervention
                return;
            }

            // Normal error handling - attempt periodic recovery
            if (currentFrame - _lastErrorRecoveryAttemptFrame < _errorRecoveryIntervalTicks)
            {
                // Not time to retry yet
                Log.Verbose("Scheduler {0} in error state (attempt {1}/{2}), waiting for recovery interval",
                    entityId, _consecutiveErrors, _maxConsecutiveErrors);
                return;
            }
            _lastErrorRecoveryAttemptFrame = currentFrame;
            Log.Info("Scheduler {0} attempting recovery from error state (attempt {1}/{2})",
                entityId, _consecutiveErrors, _maxConsecutiveErrors);

            try
            {
                // Attempt to recover by re-checking capabilities
                if (CheckCapabilities())
                {
                    // Successfully recovered
                    _consecutiveErrors = 0;
                    currentState = SchedulerState.Standby;

                    Log.Info("Scheduler {0} successfully recovered from error state", entityId);

                    // Re-initialize if necessary
                    if (!_initialized)
                    {
                        Initialize();
                    }
                }
                else
                {
                    Log.Warning("Scheduler {0} recovery attempt failed - capabilities check failed", entityId);

                    // Provide specific error information
                    if (ownAntenna == null)
                    {
                        Log.Error("Scheduler {0} error: No antenna found", entityId);
                    }
                    else if (!ownAntenna.IsFunctional)
                    {
                        Log.Error("Scheduler {0} error: Antenna is not functional", entityId);
                    }
                    else if (!ownAntenna.Enabled)
                    {
                        Log.Error("Scheduler {0} error: Antenna is not enabled", entityId);
                    }
                    else if (!ownAntenna.EnableBroadcasting)
                    {
                        Log.Error("Scheduler {0} error: Antenna broadcasting is disabled", entityId);
                    }
                    else if (!ownAntenna.IsWorking)
                    {
                        Log.Error("Scheduler {0} error: Antenna is not working (may lack power)", entityId);
                    }
                }
            }
            catch (Exception ex)
            {
                Log.Error("Scheduler {0} recovery attempt threw exception: {1}", entityId, ex.Message);
                // Exception during recovery counts as another error
            }
        }
        /// <summary>
        /// Get diagnostic information about current error state
        /// </summary>
        public string GetErrorDiagnostics()
        {
            var diagnostics = new System.Text.StringBuilder();

            diagnostics.AppendLine($"=== Scheduler {entityId} Error Diagnostics ===");
            diagnostics.AppendLine($"State: {currentState}");
            diagnostics.AppendLine($"Consecutive Errors: {_consecutiveErrors}/{_maxConsecutiveErrors}");
            diagnostics.AppendLine($"Enabled: {isEnabled}");
            diagnostics.AppendLine($"Initialized: {_initialized}");
            diagnostics.AppendLine();

            diagnostics.AppendLine("Antenna Status:");
            if (ownAntenna == null)
            {
                diagnostics.AppendLine("  - No antenna found");
            }
            else
            {
                diagnostics.AppendLine($"  - Functional: {ownAntenna.IsFunctional}");
                diagnostics.AppendLine($"  - Enabled: {ownAntenna.Enabled}");
                diagnostics.AppendLine($"  - Broadcasting: {ownAntenna.EnableBroadcasting}");
                diagnostics.AppendLine($"  - Working: {ownAntenna.IsWorking}");
                diagnostics.AppendLine($"  - Range: {ownAntenna.Radius:F1}m");
            }
            diagnostics.AppendLine();

            diagnostics.AppendLine("Task Status:");
            diagnostics.AppendLine($"  - Queued: {taskQueue.Count}");
            diagnostics.AppendLine($"  - Assigned Drones: {assignedTasks.Count}");

            var totalAssigned = 0;
            foreach (var kvp in assignedTasks)
            {
                totalAssigned += kvp.Value.Count;
            }
            diagnostics.AppendLine($"  - Total Assigned Tasks: {totalAssigned}");
            diagnostics.AppendLine($"  - Delegated Awaiting Ack: {delegatedTasksNeedingAck.Count}");
            diagnostics.AppendLine();

            diagnostics.AppendLine("Drone Status:");
            diagnostics.AppendLine($"  - Registered: {registeredDrones.Count}");

            var now = DateTime.UtcNow;
            var outOfRange = 0;
            var stale = 0;

            foreach (var drone in registeredDrones.Values)
            {
                if (drone.IsOutOfRange) outOfRange++;
                if ((now - drone.LastSeenTime).TotalSeconds > 60) stale++;
            }

            diagnostics.AppendLine($"  - Out of Range: {outOfRange}");
            diagnostics.AppendLine($"  - Stale (>60s): {stale}");

            return diagnostics.ToString();
        }
    }
}