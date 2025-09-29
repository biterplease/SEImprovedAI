using ImprovedAI.Config;
using ImprovedAI.Messages;
using ImprovedAI.Network;
using ImprovedAI.Utils;
using ImprovedAI.Utils.Logging;
using Sandbox.ModAPI;
using System;
using System.Collections.Generic;
using System.Linq;
using VRage.Game.Components;
using VRage.Game.ModAPI;
using VRage.ModAPI;
using VRage.ObjectBuilders;
using VRageMath;

namespace ImprovedAI
{
    public class IAIDroneController
    {
        private readonly long entityId;
        private readonly IMyEntity entity;
        private MessageQueue messaging;
        private readonly IdGenerator idGenerator;

        // Configuration
        private IAIDroneControllerBlockSettings config;
        public Drone.OperationMode operationMode;
        public Drone.Capabilities capabilities;

        // State
        private Drone.State currentState = Drone.State.Initializing;
        public bool isEnabled { get; set; }
        private bool _initialized = false;
        private int _consecutiveErrors = 0;

        // Components
        private IMyShipController shipController;
        private IMyShipConnector connector;
        private IMyShipWelder welder;
        private IMyShipGrinder grinder;
        private IMyRadioAntenna primaryAntenna;
        private List<IMyGyro> gyroscopes = new List<IMyGyro>();
        private List<IMySensorBlock> sensors = new List<IMySensorBlock>();
        private List<IMyThrust> thrusters = new List<IMyThrust>();
        private List<IMyGasTank> hydrogenTanks = new List<IMyGasTank>();
        private List<IMyBatteryBlock> batteries = new List<IMyBatteryBlock>();
        private List<IMyCargoContainer> cargoContainers = new List<IMyCargoContainer>();

        // Cached data
        private ThrustData thrustData = new ThrustData();
        private float currentMass = 0f;
        private float maxLoad = 0f;
        private Vector3D gravityVector = Vector3D.Zero;
        private Inventory cachedInventory;

        // Task management
        private Scheduler.Task currentTask;
        private Queue<Scheduler.Task> taskQueue = new Queue<Scheduler.Task>();
        private ushort currentTaskId = 0;

        // Update tracking
        private long _lastUpdateFrame = 0;
        private long _lastComponentCheckFrame = 0;
        private long _lastStatusReportFrame = 0;
        private long _lastPowerCheckFrame = 0;

        // Power monitoring
        private bool needsHydrogenRefuel = false;
        private bool needsBatteryRecharge = false;
        private float lastH2Level = 100f;
        private float currentH2Level = 100f;
        private float lastBatteryLevel = 100f;
        private float currentBatteryLevel = 100f;

        // Navigation
        private Vector3D currentTargetPosition;
        private List<Vector3D> currentPath = new List<Vector3D>();
        private int pathIndex = 0;

        // Update intervals (from server config)
        private readonly int COMPONENT_CHECK_INTERVAL_TICKS = 600;
        private readonly int STATUS_REPORT_INTERVAL_TICKS = ServerConfig.DroneNetwork.DroneMessageThrottlingTicks;
        private readonly int POWER_CHECK_INTERVAL_TICKS = ServerConfig.PowerCheckInterval;

        public IAIDroneController(IMyEntity entity, Drone.OperationMode operationMode = Drone.OperationMode.StandAlone)
        {
            this.entity = entity;
            this.entityId = entity.EntityId;
            this.operationMode = operationMode;
            this.idGenerator = new IdGenerator(entityId);
            this.config = new IAIDroneControllerBlockSettings();
            this.config.OperationMode = operationMode;
            this.cachedInventory = new Inventory();
        }

        public void Init(MyGameLogicComponent _base, MyObjectBuilder_EntityBase objectBuilder)
        {
            Log.Info("Initializing DroneController {0}", Log.BlockName(entity));
            _base.Init(objectBuilder);
            _base.NeedsUpdate = MyEntityUpdateEnum.EACH_FRAME | MyEntityUpdateEnum.EACH_10TH_FRAME;
        }

        public void UpdateBeforeSimulation(MyGameLogicComponent _base)
        {
            _base.UpdateBeforeSimulation();

            if (!_initialized)
            {
                Initialize();
                return;
            }

            if (!isEnabled || currentState == Drone.State.Error)
                return;

            // Main update logic handled in UpdateBeforeSimulation10
        }

        public void UpdateBeforeSimulation10(MyGameLogicComponent _base)
        {
            _base.UpdateBeforeSimulation10();

            if (!_initialized)
                return;

            var currentFrame = MyAPIGateway.Session.GameplayFrameCounter;

            // Component check
            if (currentFrame - _lastComponentCheckFrame >= COMPONENT_CHECK_INTERVAL_TICKS)
            {
                _lastComponentCheckFrame = currentFrame;
                if (!CheckCapabilities())
                {
                    currentState = Drone.State.Error;
                    return;
                }
            }

            // Power check
            if (config.MonitorHydrogenLevels || config.MonitorBatteryLevels)
            {
                if (currentFrame - _lastPowerCheckFrame >= POWER_CHECK_INTERVAL_TICKS)
                {
                    _lastPowerCheckFrame = currentFrame;
                    CheckPowerLevels();
                }
            }

            // Status reporting
            if (operationMode == Drone.OperationMode.ManagedByScheduler)
            {
                if (currentFrame - _lastStatusReportFrame >= STATUS_REPORT_INTERVAL_TICKS)
                {
                    _lastStatusReportFrame = currentFrame;
                    SendStatusReport();
                }
            }

            // Read task assignments
            if (operationMode == Drone.OperationMode.ManagedByScheduler)
            {
                ReadTaskAssignments();
            }

            // Update AI state machine
            if (isEnabled)
            {
                UpdateAI();
            }
        }

        private void Initialize()
        {
            try
            {
                currentState = Drone.State.Initializing;
                messaging = IAISession.Instance.MessageQueue;

                shipController = entity as IMyShipController;
                if (shipController == null)
                {
                    Log.Error("DroneController {0} entity is not a ship controller", entityId);
                    currentState = Drone.State.Error;
                    return;
                }

                if (!CheckCapabilities())
                {
                    Log.Error("DroneController {0} failed capability check", entityId);
                    currentState = Drone.State.Error;
                    return;
                }

                // Register antenna if available
                if (primaryAntenna != null)
                {
                    messaging.RegisterAntenna(entityId, primaryAntenna);
                }

                // Subscribe to messages if managed by scheduler
                if (operationMode == Drone.OperationMode.ManagedByScheduler)
                {
                    messaging.Subscribe(entityId, (ushort)MessageTopics.DRONE_TASK_ASSIGNMENT);
                    SendDroneRegistration();
                }

                currentState = Drone.State.Standby;
                _initialized = true;

                Log.Info("DroneController {0} initialized in mode: {1}", entityId, operationMode);
            }
            catch (Exception ex)
            {
                Log.Error("DroneController {0} initialization error: {1}", entityId, ex.Message);
                currentState = Drone.State.Error;
            }
        }

        private bool CheckCapabilities()
        {
            var cubeBlock = entity as IMyCubeBlock;
            if (cubeBlock?.CubeGrid == null) return false;

            // Clear collections
            gyroscopes.Clear();
            thrusters.Clear();
            sensors.Clear();
            hydrogenTanks.Clear();
            batteries.Clear();
            cargoContainers.Clear();

            connector = null;
            welder = null;
            grinder = null;
            primaryAntenna = null;

            // Scan connected grids
            var connectedGrids = new List<IMyCubeGrid>();
            MyAPIGateway.GridGroups.GetGroup(cubeBlock.CubeGrid, GridLinkTypeEnum.Mechanical, connectedGrids);

            foreach (var grid in connectedGrids)
            {
                var blocks = new List<IMySlimBlock>();
                grid.GetBlocks(blocks);

                foreach (var block in blocks)
                {
                    var fatBlock = block.FatBlock;
                    if (fatBlock == null || !fatBlock.IsFunctional) continue;

                    // Categorize blocks
                    if (fatBlock is IMyShipConnector && connector == null)
                        connector = (IMyShipConnector)fatBlock;
                    else if (fatBlock is IMyShipWelder && welder == null)
                        welder = (IMyShipWelder)fatBlock;
                    else if (fatBlock is IMyShipGrinder && grinder == null)
                        grinder = (IMyShipGrinder)fatBlock;
                    else if (fatBlock is IMyGyro)
                        gyroscopes.Add((IMyGyro)fatBlock);
                    else if (fatBlock is IMyThrust)
                        thrusters.Add((IMyThrust)fatBlock);
                    else if (fatBlock is IMySensorBlock)
                        sensors.Add((IMySensorBlock)fatBlock);
                    else if (fatBlock is IMyGasTank)
                    {
                        var gasTank = (IMyGasTank)fatBlock;
                        if (gasTank.BlockDefinition.SubtypeName.Contains("Hydrogen"))
                            hydrogenTanks.Add(gasTank);
                    }
                    else if (fatBlock is IMyBatteryBlock)
                        batteries.Add((IMyBatteryBlock)fatBlock);
                    else if (fatBlock is IMyCargoContainer)
                        cargoContainers.Add((IMyCargoContainer)fatBlock);
                    else if (fatBlock is IMyRadioAntenna)
                    {
                        var antenna = (IMyRadioAntenna)fatBlock;
                        if (antenna.EnableBroadcasting && antenna.IsWorking)
                        {
                            if (primaryAntenna == null || antenna.Radius > primaryAntenna.Radius)
                                primaryAntenna = antenna;
                        }
                    }
                }
            }

            // Determine capabilities
            capabilities = Drone.Capabilities.None;

            if (welder != null)
                capabilities |= Drone.Capabilities.CanWeld;
            if (grinder != null)
                capabilities |= Drone.Capabilities.CanGrind;
            if (sensors.Count > 0)
                capabilities |= Drone.Capabilities.HasSensors;
            if (connector != null)
            {
                if (config.RefuelWhenDocked)
                    capabilities |= Drone.Capabilities.RefuelWhenDocked | Drone.Capabilities.RechargeWhenDocked;
            }

            // Check flight capabilities
            if (thrusters.Count > 0)
            {
                var total = thrusters.Count;
                var atmos = 0;
                var ion = 0;
                var h2 = 0;
                foreach (var thruster in thrusters)
                {
                    if (thruster.BlockDefinition.SubtypeName.Contains("Hydrogen")) h2++;
                    if (thruster.BlockDefinition.SubtypeName.Contains("Ion")) ion++;
                    if (thruster.BlockDefinition.SubtypeName.Contains("Atmospheric")) atmos++;
                }
                float h2Ratio = h2 / total;
                float atmoRatio = h2 / total;
                float ionRatio = h2 / total;
                if (h2Ratio > 0.5f) capabilities |= Drone.Capabilities.CanFlyAtmosphere | Drone.Capabilities.CanFlySpace;
                else if (atmoRatio > 0.5f) capabilities |= Drone.Capabilities.CanFlyAtmosphere;
                else if (ionRatio > 0.5f) capabilities |= Drone.Capabilities.CanFlySpace;
                    capabilities |= Drone.Capabilities.CanFlySpace;
                // Could add atmosphere check here
            }

            // Validate minimum requirements
            bool hasMinimumComponents = gyroscopes.Count > 0 &&
                                       thrusters.Count > 0 &&
                                       connector != null;

            if (hasMinimumComponents)
            {
                CalculateThrustData();
                return true;
            }

            Log.Warning("DroneController {0} missing required components", entityId);
            return false;
        }

        private void CalculateThrustData()
        {
            thrustData = new ThrustData();

            foreach (var thruster in thrusters)
            {
                if (!thruster.IsWorking) continue;

                Vector3I thrustDirection = thruster.GridThrustDirection;

                if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Forward))
                    thrustData.Forward += thruster.MaxEffectiveThrust;
                else if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Backward))
                    thrustData.Backward += thruster.MaxEffectiveThrust;
                else if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Up))
                    thrustData.Up += thruster.MaxEffectiveThrust;
                else if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Down))
                    thrustData.Down += thruster.MaxEffectiveThrust;
                else if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Left))
                    thrustData.Left += thruster.MaxEffectiveThrust;
                else if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Right))
                    thrustData.Right += thruster.MaxEffectiveThrust;
            }

            // Calculate mass
            var mass = shipController.CalculateShipMass();
            currentMass = mass.TotalMass;

            // Calculate max load in gravity
            gravityVector = shipController.GetNaturalGravity();
            if (gravityVector.LengthSquared() > 0.1)
            {
                var gravityStrength = gravityVector.Length();
                var requiredThrustForHover = currentMass * gravityStrength;
                maxLoad = (float)((thrustData.Up - requiredThrustForHover) / gravityStrength);
            }
            else
            {
                maxLoad = thrustData.GetMinThrust() / 2.0f;
            }
        }

        private void CheckPowerLevels()
        {
            // Check hydrogen
            if (config.MonitorHydrogenLevels && hydrogenTanks.Count > 0)
            {
                float totalCapacity = 0f;
                float totalStored = 0f;

                foreach (var tank in hydrogenTanks.Where(t => t.IsWorking))
                {
                    totalCapacity += tank.Capacity;
                    totalStored += (float)(tank.Capacity * tank.FilledRatio);
                }

                currentH2Level = totalCapacity > 0 ? (totalStored / totalCapacity) * 100f : 0f;

                if (!needsHydrogenRefuel && currentH2Level <= config.HydrogenRefuelThreshold)
                {
                    needsHydrogenRefuel = true;
                    Log.Info("Drone {0} H2 low: {1:F1}%", entityId, currentH2Level);

                    if (currentState != Drone.State.RefuelingHydrogen && currentState != Drone.State.ReturningToBase)
                    {
                        currentState = Drone.State.ReturningToBase;
                    }
                }
                else if (needsHydrogenRefuel && currentH2Level >= config.HydrogenOperationalThreshold)
                {
                    needsHydrogenRefuel = false;
                    Log.Info("Drone {0} H2 refueled: {1:F1}%", entityId, currentH2Level);
                }
            }

            // Check battery
            if (config.MonitorBatteryLevels && batteries.Count > 0)
            {
                float totalCapacity = 0f;
                float totalStored = 0f;

                foreach (var battery in batteries.Where(b => b.IsWorking))
                {
                    totalCapacity += battery.MaxStoredPower;
                    totalStored += battery.CurrentStoredPower;
                }

                currentBatteryLevel = totalCapacity > 0 ? (totalStored / totalCapacity) * 100f : 0f;

                if (!needsBatteryRecharge && currentBatteryLevel <= config.BatteryRefuelThreshold)
                {
                    needsBatteryRecharge = true;
                    Log.Info("Drone {0} battery low: {1:F1}%", entityId, currentBatteryLevel);

                    if (currentState != Drone.State.RechargingBattery && currentState != Drone.State.ReturningToBase)
                    {
                        currentState = Drone.State.ReturningToBase;
                    }
                }
                else if (needsBatteryRecharge && currentBatteryLevel >= config.BatteryOperationalThreshold)
                {
                    needsBatteryRecharge = false;
                    Log.Info("Drone {0} battery recharged: {1:F1}%", entityId, currentBatteryLevel);
                }
            }
        }

        private void UpdateAI()
        {
            switch (currentState)
            {
                case Drone.State.Initializing:
                    // Handled by Initialize()
                    break;

                case Drone.State.Standby:
                    HandleStandby();
                    break;

                case Drone.State.NavigatingToTarget:
                    HandleNavigating();
                    break;

                case Drone.State.Welding:
                    HandleWelding();
                    break;

                case Drone.State.Grinding:
                    HandleGrinding();
                    break;

                case Drone.State.ReturningToBase:
                    HandleReturningToBase();
                    break;

                case Drone.State.Docking:
                    HandleDocking();
                    break;

                case Drone.State.RefuelingHydrogen:
                case Drone.State.RechargingBattery:
                    HandleRefueling();
                    break;

                case Drone.State.Error:
                    HandleError();
                    break;
            }
        }

        private void HandleStandby()
        {
            // Check if we have tasks
            if (currentTask != null)
            {
                StartTask(currentTask);
            }
            else if (taskQueue.Count > 0)
            {
                currentTask = taskQueue.Dequeue();
                StartTask(currentTask);
            }

            // Check if we need refueling
            if (needsHydrogenRefuel || needsBatteryRecharge)
            {
                currentState = Drone.State.ReturningToBase;
            }
        }

        private void StartTask(Scheduler.Task task)
        {
            currentTaskId = (ushort)task.TaskId;
            currentTargetPosition = task.Position;

            switch (task.TaskType)
            {
                case Scheduler.TaskType.PreciseWelding:
                case Scheduler.TaskType.ScanWeld:
                    currentState = Drone.State.NavigatingToTarget;
                    Log.LogDroneOrders("Drone {0} starting weld task at {1}", entityId, task.Position);
                    break;

                case Scheduler.TaskType.PreciseGrinding:
                case Scheduler.TaskType.ScanGrind:
                    currentState = Drone.State.NavigatingToTarget;
                    Log.LogDroneOrders("Drone {0} starting grind task at {1}", entityId, task.Position);
                    break;

                default:
                    Log.Warning("Drone {0} received unsupported task type: {1}", entityId, task.TaskType);
                    CompleteCurrentTask();
                    break;
            }
        }

        private void HandleNavigating()
        {
            // Simple navigation - move towards target
            var currentPos = shipController.GetPosition();
            var distance = Vector3D.Distance(currentPos, currentTargetPosition);

            if (distance < config.WaypointTolerance)
            {
                // Arrived at target
                if (currentTask != null)
                {
                    switch (currentTask.TaskType)
                    {
                        case Scheduler.TaskType.PreciseWelding:
                        case Scheduler.TaskType.ScanWeld:
                            currentState = Drone.State.Welding;
                            break;

                        case Scheduler.TaskType.PreciseGrinding:
                        case Scheduler.TaskType.ScanGrind:
                            currentState = Drone.State.Grinding;
                            break;
                    }
                }
            }
            else
            {
                // Continue moving towards target
                MoveToPosition(currentTargetPosition);
            }
        }

        private void HandleWelding()
        {
            if (welder == null)
            {
                Log.Error("Drone {0} cannot weld - no welder", entityId);
                CompleteCurrentTask();
                return;
            }

            // Enable welder
            if (!welder.Enabled)
                welder.Enabled = true;

            // Simple welding logic - would need enhancement
            // Check if target is complete
            CompleteCurrentTask();
        }

        private void HandleGrinding()
        {
            if (grinder == null)
            {
                Log.Error("Drone {0} cannot grind - no grinder", entityId);
                CompleteCurrentTask();
                return;
            }

            // Enable grinder
            if (!grinder.Enabled)
                grinder.Enabled = true;

            // Simple grinding logic - would need enhancement
            CompleteCurrentTask();
        }

        private void HandleReturningToBase()
        {
            if (connector == null)
            {
                currentState = Drone.State.Standby;
                return;
            }

            var connectorPos = connector.GetPosition();
            var currentPos = shipController.GetPosition();
            var distance = Vector3D.Distance(currentPos, connectorPos);

            if (distance < 10.0)
            {
                currentState = Drone.State.Docking;
            }
            else
            {
                MoveToPosition(connectorPos);
            }
        }

        private void HandleDocking()
        {
            if (connector == null)
            {
                currentState = Drone.State.Standby;
                return;
            }

            if (!connector.IsConnected)
            {
                // Attempt to connect
                connector.Connect();
            }

            if (connector.IsConnected)
            {
                if (needsHydrogenRefuel)
                    currentState = Drone.State.RefuelingHydrogen;
                else if (needsBatteryRecharge)
                    currentState = Drone.State.RechargingBattery;
                else
                    currentState = Drone.State.Standby;
            }
        }

        private void HandleRefueling()
        {
            // Check if refueling is complete
            if (!needsHydrogenRefuel && !needsBatteryRecharge)
            {
                currentState = Drone.State.Standby;
                Log.Info("Drone {0} refueling complete", entityId);
            }
        }

        private void HandleError()
        {
            _consecutiveErrors++;

            if (_consecutiveErrors >= ServerConfig.MaxConsecutiveErrors)
            {
                // Critical error - disable drone
                isEnabled = false;
                Log.Error("Drone {0} critical error - disabling", entityId);
            }
        }

        private void CompleteCurrentTask()
        {
            if (currentTask != null && operationMode == Drone.OperationMode.ManagedByScheduler)
            {
                SendTaskCompleteReport();
            }

            currentTask = null;
            currentTaskId = 0;
            currentState = Drone.State.Standby;
        }

        private void MoveToPosition(Vector3D targetPos)
        {
            // Simple movement logic - would be replaced with proper pathfinding
            var currentPos = shipController.GetPosition();
            var direction = Vector3D.Normalize(targetPos - currentPos);

            // Apply thrust in direction
            foreach (var thruster in thrusters)
            {
                // Simplified thrust control
                thruster.ThrustOverridePercentage = 0.5f;
            }
        }

        private void SendDroneRegistration()
        {
            var report = new DroneReport
            {
                DroneEntityId = entityId,
                Flags = Drone.UpdateFlags.Registration,
                DroneState = currentState,
                Capabilities = capabilities,
                BatteryChargePercent = currentBatteryLevel,
                BatteryRechargeThreshold = config.BatteryRefuelThreshold,
                BatteryOperationalThreshold = config.BatteryOperationalThreshold,
                H2Level = currentH2Level,
                H2RefuelThreshold = config.HydrogenRefuelThreshold,
                H2OperationalThreshold = config.HydrogenOperationalThreshold
            };

            messaging.SendMessage((ushort)MessageTopics.DRONE_REGISTRATION, report, entityId, requiresAck: false);
            Log.LogDroneNetwork("Drone {0} sent registration", entityId);
        }

        private void SendStatusReport()
        {
            var flags = Drone.UpdateFlags.None;

            // Determine what changed
            if (Math.Abs(currentH2Level - lastH2Level) > 1.0f)
            {
                flags |= Drone.UpdateFlags.H2Update;
                lastH2Level = currentH2Level;
            }

            if (Math.Abs(currentBatteryLevel - lastBatteryLevel) > 1.0f)
            {
                flags |= Drone.UpdateFlags.BatteryUpdate;
                lastBatteryLevel = currentBatteryLevel;
            }

            if (flags == Drone.UpdateFlags.None)
                return; // Nothing to report

            var report = new DroneReport
            {
                DroneEntityId = entityId,
                Flags = flags,
                DroneState = currentState,
                BatteryChargePercent = currentBatteryLevel,
                H2Level = currentH2Level
            };

            messaging.SendMessage((ushort)MessageTopics.DRONE_REPORTS, report, entityId, requiresAck: false);
        }

        private void SendTaskCompleteReport()
        {
            var report = new DroneReport
            {
                DroneEntityId = entityId,
                Flags = Drone.UpdateFlags.TaskComplete,
                TaskId = currentTaskId
            };

            messaging.SendMessage((ushort)MessageTopics.DRONE_REPORTS, report, entityId, requiresAck: false);
            Log.LogDroneOrders("Drone {0} completed task {1}", entityId, currentTaskId);
        }

        private void ReadTaskAssignments()
        {
            var assignments = messaging.ReadMessages<TaskAssignment>(entityId, (ushort)MessageTopics.DRONE_TASK_ASSIGNMENT, 10);

            foreach (var assignment in assignments)
            {
                if (assignment.EntityId != entityId)
                    continue;

                foreach (var task in assignment.Tasks)
                {
                    taskQueue.Enqueue(task);
                    Log.LogDroneOrders("Drone {0} received task {1} type {2}", entityId, task.TaskId, task.TaskType);
                }
            }
        }

        public void ResetDrone()
        {
            Log.Info("Drone {0} reset requested", entityId);

            currentTask = null;
            taskQueue.Clear();
            currentPath.Clear();
            pathIndex = 0;

            // Disable tools
            if (welder?.Enabled == true) welder.Enabled = false;
            if (grinder?.Enabled == true) grinder.Enabled = false;

            // Reset thrusters
            foreach (var thruster in thrusters)
            {
                thruster.ThrustOverridePercentage = 0f;
            }

            // Reset gyros
            foreach (var gyro in gyroscopes)
            {
                gyro.GyroOverride = false;
            }

            _consecutiveErrors = 0;
            currentState = Drone.State.Standby;

            Log.Info("Drone {0} reset complete", entityId);
        }
    }
}