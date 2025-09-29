using BetterAIConstructor.Pathfinding;
using BetterAIConstructor.Config;
using BetterAIConstructor.Display;
using BetterAIConstructor.Interfaces;
using BetterAIConstructor.Navigation;
using BetterAIConstructor.DroneConfig;
using Sandbox.Common.ObjectBuilders;
using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI;
using System;
using System.Collections.Generic;
using System.Linq;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.ModAPI;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.ModAPI;
using VRage.ObjectBuilders;
using VRageMath;

namespace ImprovedAI
{
    public enum AIState
    {
        Initializing,
        Standby,
        ScanningTargets,
        NavigatingToTarget,
        Welding,
        Grinding,
        ReturningToBase,
        Docking,
        RefuelingHydrogen,
        RefuelingBattery,
        Error
    }

    [MyEntityComponentDescriptor(typeof(MyObjectBuilder_RemoteControl), false, "ImprovedAISmallDroneController", "ImprovedAISmallDroneController")]
    public class IAIDroneControllerBlock: MyGameLogicComponent
    {
        // Core components
        private BAIConstructorDroneConfig config;
        private NavigationManager navigationManager;
        private DisplayManager displayManager;
        private PathfindingManager pathfindingManager;

        // Block references
        private IMyShipController shipController;

        // AI State
        private AIState currentState = AIState.Initializing;
        private string statusMessage = "Initializing...";
        private bool isEnabled = false;
        private bool capabilitiesValid = false;
        private int tickCounter = 0;

        // Component references
        private IMyShipConnector connector;
        private IMyShipWelder welder;
        private IMyShipGrinder grinder;
        private List<IMyGyro> gyroscopes = new List<IMyGyro>();
        private List<IMySensorBlock> sensors = new List<IMySensorBlock>();
        private List<IMyThrust> thrusters = new List<IMyThrust>();

        // Power monitoring components
        private List<IMyGasTank> hydrogenTanks = new List<IMyGasTank>();
        private List<IMyBatteryBlock> batteries = new List<IMyBatteryBlock>();

        // Power monitoring state
        private bool needsHydrogenRefuel = false;
        private bool needsBatteryRecharge = false;
        private DateTime lastPowerCheck = DateTime.MinValue;
        private readonly TimeSpan powerCheckInterval = TimeSpan.FromSeconds(5); // Check power every 5 seconds

        // Navigation and targeting
        private Vector3D currentTargetPosition;
        private Vector3D connectorOffset;
        private Vector3D welderOffset;
        private Vector3D grinderOffset;
        private List<IMySlimBlock> targetBlocks = new List<IMySlimBlock>();
        private int targetBlockLimit = 10;
        private int currentTargetIndex = 0;

        // Performance data
        private ThrustData thrustData = new ThrustData();
        private double maxThrust = 0;
        private float maxLoad = 0f;
        private float currentLoad = 0f;
        public double BlockWaypointDistance { get; private set; } = 8.0;

        // Configuration
        private MyIni configIni = new MyIni();
        private const int UPDATE_INTERVAL = 10;

        private int consecutiveErrors = 0;
        private const int MAX_CONSECUTIVE_ERRORS = 3;

        public override void Init(MyObjectBuilder_EntityBase objectBuilder)
        {
            // This method is called async! Use UpdateOnceBeforeFrame for proper initialization
            NeedsUpdate = MyEntityUpdateEnum.BEFORE_NEXT_FRAME;
        }

        public override void UpdateOnceBeforeFrame()
        {
            try
            {
                shipController = Entity as IMyShipController;

                if (shipController?.CubeGrid?.Physics == null)
                    return; // Ignore projected and non-physical grids

                if (!MyAPIGateway.Session.IsServer)
                    return; // Only run on server

                // Check if this constructor is allowed based on server limits
                if (!CheckConstructionLimits())
                {
                    // Mark for removal if limits exceeded
                    Entity.Close();
                    return;
                }

                // Initialize configuration
                config = new BAIConstructorDroneConfig();
                LoadConfiguration();

                // Apply server defaults if enabled
                ApplyServerDefaults();

                // Initialize managers
                pathfindingManager = new PathfindingManager();

                // Wait a bit for grid to be stable before initializing components
                NeedsUpdate = MyEntityUpdateEnum.EACH_10TH_FRAME;
                tickCounter = 0;

                MyAPIGateway.Utilities.ShowMessage("BetterAI_Constructor",
                    $"AI Constructor block initialized: {Entity.DisplayName}");
            }
            catch (Exception ex)
            {
                MyAPIGateway.Utilities.ShowMessage("BetterAI_Constructor",
                    $"Initialization error: {ex.Message}");
                LogError("Init", ex);
            }
        }


        private bool CheckConstructionLimits()
        {
            try
            {
                var ownerId = Entity.;
                var ownerFaction = MyAPIGateway.Session.Factions.TryGetPlayerFaction(ownerId);

                // Check total server limit
                if (!ServerConfigManager.CanCreateConstructorTotal())
                {
                    MyAPIGateway.Utilities.ShowMessage("BetterAI_Constructor",
                        $"Cannot create AI Constructor: Server limit of {ServerConfigManager.MaxConstructorsTotal} reached");
                    return false;
                }

                // Check player limit
                if (!ServerConfigManager.CanPlayerCreateConstructor(ownerId))
                {
                    MyAPIGateway.Utilities.ShowMessage("BetterAI_Constructor",
                        $"Cannot create AI Constructor: Player limit of {ServerConfigManager.MaxConstructorsPerPlayer} reached");
                    return false;
                }

                // Check faction limit
                if (ownerFaction != null && !ServerConfigManager.CanFactionCreateConstructor(ownerFaction.FactionId))
                {
                    MyAPIGateway.Utilities.ShowMessage("BetterAI_Constructor",
                        $"Cannot create AI Constructor: Faction limit of {ServerConfigManager.MaxConstructorsPerFaction} reached");
                    return false;
                }

                return true;
            }
            catch (Exception ex)
            {
                LogError("CheckConstructionLimits", ex);
                return true; // Allow creation if check fails to avoid blocking legitimate use
            }
        }

        private void ApplyServerDefaults()
        {
            try
            {
                // Apply server-configured defaults to new blocks
                if (ServerConfigManager.DefaultMonitorHydrogen)
                {
                    config.MonitorHydrogenLevels = true;
                    config.HydrogenRefuelThreshold = ServerConfigManager.DefaultHydrogenRefuelThreshold;
                    config.HydrogenOperationalThreshold = ServerConfigManager.DefaultHydrogenOperationalThreshold;
                }

                if (ServerConfigManager.DefaultMonitorBattery)
                {
                    config.MonitorBatteryLevels = true;
                    config.BatteryRefuelThreshold = ServerConfigManager.DefaultBatteryRefuelThreshold;
                    config.BatteryOperationalThreshold = ServerConfigManager.DefaultBatteryOperationalThreshold;
                }

                // Apply server safety limits
                if (ServerConfigManager.EnableSafetyLimits)
                {
                    config.MaxApproachSpeed = Math.Min(config.MaxApproachSpeed, ServerConfigManager.MaxThrustOverride * 15.0f);
                    config.MaxTravelSpeed = Math.Min(config.MaxTravelSpeed, ServerConfigManager.MaxThrustOverride * 50.0f);
                }

                // Ensure power thresholds are within server limits
                config.HydrogenRefuelThreshold = MathHelper.Clamp(config.HydrogenRefuelThreshold,
                    ServerConfigManager.MinPowerThreshold, ServerConfigManager.MaxPowerThreshold);
                config.HydrogenOperationalThreshold = MathHelper.Clamp(config.HydrogenOperationalThreshold,
                    ServerConfigManager.MinPowerThreshold, ServerConfigManager.MaxPowerThreshold);
                config.BatteryRefuelThreshold = MathHelper.Clamp(config.BatteryRefuelThreshold,
                    ServerConfigManager.MinPowerThreshold, ServerConfigManager.MaxPowerThreshold);
                config.BatteryOperationalThreshold = MathHelper.Clamp(config.BatteryOperationalThreshold,
                    ServerConfigManager.MinPowerThreshold, ServerConfigManager.MaxPowerThreshold);
            }
            catch (Exception ex)
            {
                LogError("ApplyServerDefaults", ex);
            }
        }


        private bool ShouldEnterSleepMode()
        {
            // Check if any players are nearby
            var players = new List<IMyPlayer>();
            MyAPIGateway.Players.GetPlayers(players);

            var myPosition = GetPosition();
            foreach (var player in players)
            {
                if (Vector3D.DistanceSquared(player.GetPosition(), myPosition) < 2500) // 50m range
                    return false;
            }

            return players.Count > 0; // Only sleep if players exist but are far away
        }
        private int GetUpdateInterval()
        {
            switch (currentState)
            {
                case AIState.Standby:
                case AIState.Error:
                    return 60; // Update every 6 seconds when idle
                case AIState.ScanningTargets:
                    return 30; // Update every 3 seconds when scanning
                case AIState.Welding:
                case AIState.Grinding:
                    return 6;  // Update every 0.6 seconds when working
                default:
                    return 10; // Default 1 second updates
            }
        }
        public override void UpdateAfterSimulation10()
        {
            if (tickCounter % GetUpdateInterval() != 0) return;
            try
            {
                if (!MyAPIGateway.Session.IsServer || shipController?.CubeGrid?.Physics == null)
                    return;

                tickCounter++;

                // Delayed initialization after grid stabilizes
                if (tickCounter == 10) // After ~2.5 seconds
                {
                    InitializeComponents();
                    navigationManager = new NavigationManager(this, pathfindingManager, config);
                    displayManager = new DisplayManager(this, config);

                    // Register with session
                    BAISession.Instance?.RegisterAIConstructor(this);

                    NeedsUpdate = MyEntityUpdateEnum.EACH_10TH_FRAME;
                }
                else if (tickCounter > 10)
                {
                    // Main update loop
                    if (isEnabled)
                    {
                        UpdateAI();
                    }
                    else
                    {
                        navigationManager?.StopAllMovement();
                    }

                    displayManager?.UpdateDisplay();

                    // Periodic status broadcast
                    if (tickCounter % 600 == 0) // Every 60 seconds
                    {
                        displayManager?.BroadcastStatusUpdate();
                    }
                }
            }
            catch (Exception ex)
            {
                displayManager?.ShowError($"Update error: {ex.Message}");
                LogError("Update", ex);
                currentState = AIState.Error;
            }
        }

        #region Initialization


        public List<IMyBatteryBlock> GetBatteries() => batteries;
        public List<IMyGasTank> GetHydrogenTanks() => hydrogenTanks;
        public bool NeedsBatteryRecharge() => needsBatteryRecharge;
        public bool NeedsHydrogenRefuel() => needsHydrogenRefuel;

        private void InitializeComponents()
        {
            List<IMySlimBlock> blocks = new List<IMySlimBlock>();
            shipController.CubeGrid.GetBlocks(blocks);

            gyroscopes.Clear();
            thrusters.Clear();
            sensors.Clear();
            hydrogenTanks.Clear();
            batteries.Clear();

            foreach (IMySlimBlock block in blocks)
            {
                var functionalBlock = block.FatBlock;

                if (functionalBlock is IMyShipConnector && connector == null)
                    connector = (IMyShipConnector)functionalBlock;
                else if (functionalBlock is IMyShipWelder && welder == null)
                    welder = (IMyShipWelder)functionalBlock;
                else if (functionalBlock is IMyShipGrinder && grinder == null)
                    grinder = (IMyShipGrinder)functionalBlock;
                else if (functionalBlock is IMyGyro)
                    gyroscopes.Add((IMyGyro)functionalBlock);
                else if (functionalBlock is IMyThrust)
                    thrusters.Add((IMyThrust)functionalBlock);
                else if (functionalBlock is IMySensorBlock)
                    sensors.Add((IMySensorBlock)functionalBlock);
                else if (functionalBlock is IMyGasTank)
                {
                    // Check if it's a hydrogen tank
                    var gasTank = (IMyGasTank)functionalBlock;
                    if (gasTank.BlockDefinition.SubtypeName.Contains("Hydrogen") ||
                        gasTank.DisplayNameText.Contains("Hydrogen") || gasTank.DisplayNameText.Contains("H2"))
                        hydrogenTanks.Add(gasTank);
                } else if (functionalBlock is IMyBatteryBlock)
                    batteries.Add((IMyBatteryBlock)functionalBlock);
            }

            CheckCapabilities();
        }

        private bool CheckCapabilities()
        {
            
            var componentErrors = new List<string>();
            if (connector == null) componentErrors.Add("No Connector found");
            if (config.GrindBlocks && grinder == null) componentErrors.Add("No Grinder found");
            if ((config.WeldUnfinishedBlocks || config.WeldProjectedBlocks || config.RepairDamagedBlocks) && welder == null)
                componentErrors.Add("No welder found");
            if (gyroscopes.Count == 0) componentErrors.Add("No gyroscopes found");
            if (thrusters.Count == 0) componentErrors.Add("No thrusters found");
            if (config.MonitorHydrogenLevels && hydrogenTanks.Count == 0)
            {
                componentErrors.Add("No Hydrogen tanks found");
            }
            if (config.MonitorBatteryLevels && batteries.Count == 0)
            {
                componentErrors.Add("No batteries found");
                capabilitiesValid = false;
                return false;
            }

            if (componentErrors.Count > 0)
            {
                statusMessage = $"{string.Join(", ", componentErrors)}";
                displayManager?.ShowError($"Component errors: {string.Join(", ", componentErrors)}");
                capabilitiesValid = false;
                return false;
            }

            CalculateOffsets();
            CalculateThrust();
            CalculateMaxLoad();
            capabilitiesValid = true;
            return true;
        }

        #endregion

        #region AI State Machine

        private void UpdateAI()
        {
            // Always check power levels first
            CheckPowerLevels();

            switch (currentState)
            {
                case AIState.Initializing:
                    HandleInitializing();
                    break;
                case AIState.Standby:
                    HandleStandby();
                    break;
                case AIState.ScanningTargets:
                    HandleScanningTargets();
                    break;
                case AIState.NavigatingToTarget:
                    HandleNavigatingToTarget();
                    break;
                case AIState.Welding:
                    HandleWelding();
                    break;
                case AIState.Grinding:
                    HandleGrinding();
                    break;
                case AIState.ReturningToBase:
                    HandleReturningToBase();
                    break;
                case AIState.Docking:
                    HandleDocking();
                    break;
                case AIState.RefuelingHydrogen:
                case AIState.RefuelingBattery:
                    HandleRefueling();
                    break;
                case AIState.Error:
                    HandleError();
                    break;
            }
        }

        private void CheckPowerLevels()
        {
            var now = DateTime.Now;
            if (now - lastPowerCheck < powerCheckInterval)
                return;
            lastPowerCheck = now;

            // Check hydrogen levels
            if (config.MonitorHydrogenLevels && hydrogenTanks.Count > 0)
            {
                var totalCapacity = 0.0;
                var totalStored = 0.0;

                foreach (var tank in hydrogenTanks.Where(t => t.IsWorking))
                {
                    totalCapacity += tank.Capacity;
                    totalStored += tank.Capacity * tank.FilledRatio;
                }

                var hydrogenPercentage = totalCapacity > 0 ? (totalStored / totalCapacity) * 100.0 : 0.0;

                if (!needsHydrogenRefuel && hydrogenPercentage <= config.HydrogenRefuelThreshold)
                {
                    needsHydrogenRefuel = true;
                    displayManager?.ShowWarning($"Hydrogen low ({hydrogenPercentage:F1}%) - returning to base for refuel");
                }
                else if (needsHydrogenRefuel && hydrogenPercentage >= config.HydrogenOperationalThreshold)
                {
                    needsHydrogenRefuel = false;
                    displayManager?.ShowSuccess($"Hydrogen refueled ({hydrogenPercentage:F1}%) - resuming operations");
                }
            }

            // Check battery levels
            if (config.MonitorBatteryLevels && batteries.Count > 0)
            {
                var totalCapacity = 0.0f;
                var totalStored = 0.0f;

                foreach (var battery in batteries.Where(b => b.IsWorking))
                {
                    totalCapacity += battery.MaxStoredPower;
                    totalStored += battery.CurrentStoredPower;
                }

                var batteryPercentage = totalCapacity > 0 ? (totalStored / totalCapacity) * 100.0f : 0.0f;

                if (!needsBatteryRecharge && batteryPercentage <= config.BatteryRefuelThreshold)
                {
                    needsBatteryRecharge = true;
                    displayManager?.ShowWarning($"Battery low ({batteryPercentage:F1}%) - returning to base for recharge");
                }
                else if (needsBatteryRecharge && batteryPercentage >= config.BatteryOperationalThreshold)
                {
                    needsBatteryRecharge = false;
                    displayManager?.ShowSuccess($"Battery recharged ({batteryPercentage:F1}%) - resuming operations");
                }
            }
        }

        private bool ShouldRefuel()
        {
            return needsHydrogenRefuel || needsBatteryRecharge;
        }

        private string GetRefuelReason()
        {
            var reasons = new List<string>();
            if (needsHydrogenRefuel) reasons.Add("Hydrogen");
            if (needsBatteryRecharge) reasons.Add("Battery");
            return string.Join(" & ", reasons);
        }

        public float GetHydrogenPercentage()
        {
            if (hydrogenTanks.Count == 0) return 0.0f;

            var totalCapacity = 0.0;
            var totalStored = 0.0;

            foreach (var tank in hydrogenTanks.Where(t => t.IsWorking))
            {
                totalCapacity += tank.Capacity;
                totalStored += tank.Capacity * tank.FilledRatio;
            }

            return totalCapacity > 0 ? (float)((totalStored / totalCapacity) * 100.0) : 0.0f;
        }

        public float GetBatteryPercentage()
        {
            if (batteries.Count == 0) return 0.0f;

            var totalCapacity = 0.0f;
            var totalStored = 0.0f;

            foreach (var battery in batteries.Where(b => b.IsWorking))
            {
                totalCapacity += battery.MaxStoredPower;
                totalStored += battery.CurrentStoredPower;
            }

            return totalCapacity > 0 ? (totalStored / totalCapacity) * 100.0f : 0.0f;
        }

        private void HandleInitializing()
        {
            if (CheckCapabilities())
            {
                currentState = AIState.Standby;
                statusMessage = "AI Ready - Awaiting orders";
                displayManager?.ShowSuccess("Construction AI initialized successfully");
            }
            else
            {
                currentState = AIState.Error;
            }
        }

        private void HandleStandby()
        {
            statusMessage = "Standby - Ready for orders";

            // Check if we need to refuel before starting work
            if (ShouldRefuel())
            {
                if (needsHydrogenRefuel && needsBatteryRecharge)
                    currentState = AIState.RefuelingHydrogen; // Handle both, but start with hydrogen
                else if (needsHydrogenRefuel)
                    currentState = AIState.RefuelingHydrogen;
                else if (needsBatteryRecharge)
                    currentState = AIState.RefuelingBattery;

                statusMessage = $"Low {GetRefuelReason()} - returning to base";
                return;
            }

            if (ShouldStartScanning())
            {
                currentState = AIState.ScanningTargets;
                statusMessage = "Scanning for construction targets...";
            }
        }

        private bool ShouldStartScanning()
        {
            return isEnabled &&
                   (config.WeldUnfinishedBlocks || config.RepairDamagedBlocks || config.WeldProjectedBlocks || config.GrindBlocks) &&
                   (welder != null || grinder != null);
        }

        private void HandleScanningTargets()
        {
            ScanForUnfinishedBlocks();

            if (targetBlocks.Count > 0)
            {
                currentTargetIndex = 0;
                currentTargetPosition = GetBlockWorldPosition(targetBlocks[0]);
                CalculatePathToTarget();

                currentState = AIState.NavigatingToTarget;
                statusMessage = $"Found {targetBlocks.Count} targets - navigating to first";
                displayManager?.ShowInfo($"Beginning construction of {targetBlocks.Count} unfinished blocks");

                if (connector?.IsConnected == true)
                    connector.Disconnect();
            }
            else
            {
                currentState = AIState.Standby;
                statusMessage = "No construction targets found";
                displayManager?.ShowSuccess("All blocks complete - standing by");
            }
        }

        private void HandleNavigatingToTarget()
        {
            // Check if we need to refuel while working
            if (ShouldRefuel())
            {
                currentState = AIState.ReturningToBase;
                statusMessage = $"Low {GetRefuelReason()} - aborting work to refuel";
                return;
            }

            var currentPos = GetPosition();
            var distanceToTarget = Vector3D.Distance(currentPos, currentTargetPosition);

            // Use navigation manager for pathfinding and movement
            var maxSpeed = distanceToTarget < 10.0 ? config.MaxApproachSpeed : config.MaxTravelSpeed;
            navigationManager.NavigateToPosition(currentTargetPosition, maxSpeed);

            // Check if we've reached the target
            if (distanceToTarget < config.WaypointTolerance)
            {
                displayManager?.ShowSuccess("Arrived at target block");

                var targetBlock = targetBlocks[currentTargetIndex];
                if (targetBlock.BuildLevelRatio < 1.0f || targetBlock.CurrentDamage > 0)
                {
                    currentState = AIState.Welding;
                }
                else if (config.GrindBlocks)
                {
                    currentState = AIState.Grinding;
                }
            }

            statusMessage = $"Navigating to target ({distanceToTarget:F1}m away)";
        }

        private void HandleWelding()
        {
            // Check if we need to refuel while working
            if (ShouldRefuel())
            {
                if (welder?.Enabled == true) welder.Enabled = false;
                currentState = AIState.ReturningToBase;
                statusMessage = $"Low {GetRefuelReason()} - aborting welding to refuel";
                return;
            }

            // Existing welding logic remains the same...
            if (welder == null)
            {
                displayManager?.ShowError("No welder available");
                currentState = AIState.Error;
                return;
            }

            var targetBlock = targetBlocks[currentTargetIndex];
            var currentPos = GetPosition();
            var targetPos = GetBlockWorldPosition(targetBlock);
            var distance = Vector3D.Distance(currentPos, targetPos);

            // Position for welding
            var optimalPos = targetPos - Vector3D.Transform(welderOffset, WorldMatrix);
            navigationManager.NavigateToPosition(optimalPos, 1.0);

            if (distance < 5.0)
            {
                navigationManager.OrientTowardsTarget(targetPos);

                if (!welder.Enabled)
                    welder.Enabled = true;

                if (targetBlock.BuildLevelRatio >= 1.0f && targetBlock.CurrentDamage <= 0)
                {
                    welder.Enabled = false;
                    displayManager?.ShowSuccess("Block welding completed");
                    MoveToNextTarget();
                }
                else
                {
                    statusMessage = $"Grinding block... {(1 - targetBlock.BuildLevelRatio) * 100:F1}% complete";
                }
            }
            else
            {
                statusMessage = $"Approaching grinding position ({distance:F1}m)";
            }
        }

        private void HandleGrinding()
        {
            if (ShouldRefuel())
            {
                if (welder?.Enabled == true) welder.Enabled = false;
                currentState = AIState.ReturningToBase;
                statusMessage = $"Low {GetRefuelReason()} - aborting welding to refuel";
                return;
            }
            if (grinder == null)
            {
                displayManager?.ShowError("No grinder available");
                currentState = AIState.Error;
                return;
            }

            var targetBlock = targetBlocks[currentTargetIndex];
            var currentPos = GetPosition();
            var targetPos = GetBlockWorldPosition(targetBlock);
            var distance = Vector3D.Distance(currentPos, targetPos);

            var optimalPos = targetPos - Vector3D.Transform(grinderOffset, WorldMatrix);
            navigationManager.NavigateToPosition(optimalPos, 1.0);

            if (distance < 5.0)
            {
                navigationManager.OrientTowardsTarget(targetPos);

                if (!grinder.Enabled)
                    grinder.Enabled = true;

                if (targetBlock.IsDestroyed || targetBlock.BuildLevelRatio <= 0)
                {
                    grinder.Enabled = false;
                    displayManager?.ShowSuccess("Block grinding completed");
                    MoveToNextTarget();
                }
                else
                {
                    statusMessage = $"Grinding block... {(1 - targetBlock.BuildLevelRatio) * 100:F1}% complete";
                }
            }
            else
            {
                statusMessage = $"Approaching grinding position ({distance:F1}m)";
            }
        }

        private void HandleRefueling()
        {
            if (connector == null)
            {
                displayManager?.ShowError("No connector available for refueling");
                currentState = AIState.Error;
                return;
            }

            var connectorPos = connector.WorldMatrix.Translation;
            var currentPos = GetPosition();
            var distance = Vector3D.Distance(currentPos, connectorPos);

            // If not connected, navigate to and dock
            if (!connector.IsConnected)
            {
                if (distance > 10.0)
                {
                    navigationManager.NavigateToPosition(connectorPos, config.MaxApproachSpeed);
                    statusMessage = $"Returning to base for {GetRefuelReason().ToLower()} refuel ({distance:F1}m away)";
                }
                else
                {
                    // Close enough to dock
                    var dockingPos = connectorPos + Vector3D.Transform(connectorOffset, WorldMatrix);
                    navigationManager.NavigateToPosition(dockingPos, config.MaxApproachSpeed * 0.2);
                    navigationManager.OrientTowardsTarget(connectorPos);

                    if (distance < 0.5)
                    {
                        connector.Connect();
                    }
                    statusMessage = $"Docking for {GetRefuelReason().ToLower()} refuel...";
                }
                return;
            }

            // Connected - check if refueling is complete
            var hydrogenReady = !needsHydrogenRefuel || GetHydrogenPercentage() >= config.HydrogenOperationalThreshold;
            var batteryReady = !needsBatteryRecharge || GetBatteryPercentage() >= config.BatteryOperationalThreshold;

            if (hydrogenReady && batteryReady)
            {
                // Refueling complete
                needsHydrogenRefuel = false;
                needsBatteryRecharge = false;
                currentState = AIState.Standby;
                statusMessage = "Refueling complete - standing by";
                displayManager?.ShowSuccess($"Refueling complete - H2: {GetHydrogenPercentage():F1}%, Battery: {GetBatteryPercentage():F1}%");
            }
            else
            {
                // Still refueling
                var refuelStatus = new List<string>();
                if (!hydrogenReady) refuelStatus.Add($"H2: {GetHydrogenPercentage():F1}%/{config.HydrogenOperationalThreshold:F1}%");
                if (!batteryReady) refuelStatus.Add($"Battery: {GetBatteryPercentage():F1}%/{config.BatteryOperationalThreshold:F1}%");
                statusMessage = $"Refueling... {string.Join(", ", refuelStatus)}";
            }
        }

        private void HandleReturningToBase()
        {
            if (connector == null)
            {
                currentState = AIState.Standby;
                statusMessage = "No connector - returning to standby";
                return;
            }

            var connectorPos = connector.WorldMatrix.Translation;
            var currentPos = GetPosition();
            var distance = Vector3D.Distance(currentPos, connectorPos);

            navigationManager.NavigateToPosition(connectorPos, config.MaxApproachSpeed);

            if (distance < 10.0)
            {
                currentState = AIState.Docking;
                statusMessage = "Preparing to dock";
            }
            else
            {
                statusMessage = $"Returning to base ({distance:F1}m away)";
            }
        }

        private void HandleDocking()
        {
            if (connector == null)
            {
                currentState = AIState.Standby;
                return;
            }

            var dockingPos = connector.WorldMatrix.Translation + Vector3D.Transform(connectorOffset, WorldMatrix);
            var currentPos = GetPosition();
            var distance = Vector3D.Distance(currentPos, dockingPos);

            navigationManager.NavigateToPosition(dockingPos, config.MaxApproachSpeed * 0.2); // Very slow docking
            navigationManager.OrientTowardsTarget(connector.WorldMatrix.Translation);

            if (distance < 0.5 && !connector.IsConnected)
            {
                connector.Connect();
            }

            if (connector.IsConnected)
            {
                navigationManager.StopAllMovement();

                // Check if we need to refuel after docking
                if (ShouldRefuel())
                {
                    if (needsHydrogenRefuel && needsBatteryRecharge)
                        currentState = AIState.RefuelingHydrogen;
                    else if (needsHydrogenRefuel)
                        currentState = AIState.RefuelingHydrogen;
                    else if (needsBatteryRecharge)
                        currentState = AIState.RefuelingBattery;

                    statusMessage = $"Docked - beginning {GetRefuelReason().ToLower()} refuel";
                }
                else
                {
                    currentState = AIState.Standby;
                    statusMessage = "Docked successfully - standing by";
                    displayManager?.ShowSuccess("Construction cycle completed - docked at base");
                }
            }
            else
            {
                statusMessage = $"Docking... ({distance:F1}m from connector)";
            }
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

        private void MoveToNextTarget()
        {
            currentTargetIndex++;
            if (currentTargetIndex < targetBlocks.Count)
            {
                currentTargetPosition = GetBlockWorldPosition(targetBlocks[currentTargetIndex]);
                CalculatePathToTarget();
                currentState = AIState.NavigatingToTarget;
                statusMessage = $"Moving to next target ({currentTargetIndex + 1}/{targetBlocks.Count})";
            }
            else
            {
                currentState = AIState.ReturningToBase;
                statusMessage = "All targets completed - returning to base";
            }
        }

        #endregion

        #region Calculations and Utilities

        private void CalculateOffsets()
        {
            var aiPos = shipController.CubeGrid.GridIntegerToWorld(shipController.Position);
            bool isLargeGrid = shipController.CubeGrid.GridSizeEnum == MyCubeSize.Large;
            float activeRadius = isLargeGrid ? 2.0f : 1.5f;
            float centerOffset = isLargeGrid ? 2.5f : 1.5f;

            if (welder != null)
            {
                var sphereCenter = welder.WorldMatrix.Translation + welder.WorldMatrix.Forward * centerOffset;
                var activeSphereEdge = sphereCenter + welder.WorldMatrix.Forward * activeRadius;
                welderOffset = Vector3D.Transform(activeSphereEdge - aiPos, MatrixD.Transpose(shipController.WorldMatrix));
            }

            if (grinder != null)
            {
                var sphereCenter = grinder.WorldMatrix.Translation + grinder.WorldMatrix.Forward * centerOffset;
                var activeSphereEdge = sphereCenter + grinder.WorldMatrix.Forward * activeRadius;
                grinderOffset = Vector3D.Transform(activeSphereEdge - aiPos, MatrixD.Transpose(shipController.WorldMatrix));
            }

            if (connector != null)
            {
                var connectorEdge = connector.WorldMatrix.Translation + connector.WorldMatrix.Forward * 0.25f;
                connectorOffset = Vector3D.Transform(connectorEdge - aiPos, MatrixD.Transpose(shipController.WorldMatrix));
            }
        }

        private void CalculateThrust()
        {
            thrustData = new ThrustData();
            foreach (var thruster in thrusters)
            {
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

            maxThrust = thrustData.GetMaxThrust();
        }

        private void CalculateMaxLoad()
        {
            var mass = shipController.CalculateShipMass();
            var gravity = shipController.GetNaturalGravity();
            var gravityStrength = gravity.Length();

            if (gravityStrength > 0)
            {
                var requiredThrustForHover = mass.PhysicalMass * gravityStrength;
                currentLoad = mass.TotalMass - mass.BaseMass;
                maxLoad = (float)((thrustData.Up - requiredThrustForHover) / gravityStrength);
            }
            else
            {
                maxLoad = thrustData.GetMinThrust() / 2.0f;
            }

            maxLoad = Math.Max(0, maxLoad);
        }

        private void CalculatePathToTarget()
        {
            var startPos = GetPosition();
            var path = CalculatePath(startPos, currentTargetPosition);
            navigationManager?.SetPath(path);
        }

        private List<Vector3D> CalculatePath(Vector3D start, Vector3D end)
        {
            var context = new PathfindingContext
            {
                Controller = this,
                Sensors = sensors,
                GravityVector = shipController.GetNaturalGravity(),
                ThrustData = thrustData,
                ShipMass = shipController.CalculateShipMass().TotalMass,
                MaxThrust = maxThrust,
                MaxLoad = maxLoad,
                WaypointDistance = BlockWaypointDistance,
                CubeGrid = shipController.CubeGrid
            };

            return pathfindingManager.CalculatePath(start, end, context);
        }

        private void ScanForUnfinishedBlocks()
        {
            targetBlocks.Clear();
            var scannedBlocks = 0;
            const int MAX_BLOCKS_PER_SCAN = 50; // Limit blocks scanned per update

            List<IMyCubeGrid> connectedGrids = new List<IMyCubeGrid>();
            MyAPIGateway.GridGroups.GetGroup(shipController.CubeGrid, GridLinkTypeEnum.Physical, connectedGrids);

            foreach (var grid in connectedGrids)
            {
                var blocks = new List<IMySlimBlock>();
                grid.GetBlocks(blocks);

                foreach (var block in blocks)
                {
                    scannedBlocks++;
                    if (scannedBlocks > MAX_BLOCKS_PER_SCAN)
                    {
                        // Continue scanning next update
                        return;
                    }

                    if (ShouldTargetBlock(block))
                    {
                        targetBlocks.Add(block);
                        if (targetBlocks.Count >= targetBlockLimit)
                            return;
                    }
                }
            }

            SortTargetBlocks(TargetSorting.ClosestFirst);
        }
        private bool ShouldTargetBlock(IMySlimBlock block)
        {
            if (config.WeldUnfinishedBlocks && block.BuildLevelRatio < 1.0f)
                return true;
            if (config.RepairDamagedBlocks && block.CurrentDamage > 0.0f)
                return true;
            // Add projected block logic here
            return false;
        }

        private void SortTargetBlocks(TargetSorting method)
        {
            var aiPos = GetPosition();
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

        #endregion

        #region Utility Methods

        private void LogError(string context, Exception ex)
        {
            var message = $"[{context}] {ex.Message}\n{ex.StackTrace}";
            var writer = MyAPIGateway.Utilities.WriteFileInLocalStorage("error.log", typeof(BAISession));
            if (writer != null)
            {
                writer.Write(message.ToString());
                writer.Flush();
            }
            writer.Close();
        }

        #endregion



        public override void MarkForClose()
        {
            try
            {
                // Stop all movement
                navigationManager?.StopAllMovement();

                // Clear collections to help GC
                targetBlocks?.Clear();
                thrusters?.Clear();
                gyroscopes?.Clear();
                sensors?.Clear();

                // Dispose of managers
                navigationManager = null;
                displayManager = null;
                pathfindingManager = null;

                // Save and cleanup
                SaveConfig();
                BAISession.Instance?.UnregisterAIConstructor(Entity.EntityId);
            }
            catch (Exception ex)
            {
                LogError("Close", ex);
            }
        }

        #region BAIController Implementation

        public AIState GetCurrentState() => currentState;
        public string GetStatusMessage() => statusMessage;
        public void SetStatusMessage(string message) => statusMessage = message;
        public bool IsEnabled => isEnabled;
        public void SetEnabled(bool enabled) => isEnabled = enabled;
        public bool AreCapabilitiesValid() => capabilitiesValid;

        public BAIConstructorDroneConfig GetConfig() => config;

        public void SaveConfig()
        {
            try
            {
                config.SaveToIni(configIni);
                if (Entity.Storage == null)
                    Entity.Storage = new MyModStorageComponent();
                Entity.Storage.SetValue(Guid.Parse("89afa424-d317-41de-992c-be2854c2f158"), configIni.ToString());
            }
            catch (Exception ex)
            {
                LogError("SaveConfig", ex);
            }
        }

        public IMyCubeGrid CubeGrid => shipController.CubeGrid;
        public MatrixD WorldMatrix => shipController.WorldMatrix;
        public Vector3D GetPosition() => shipController.CubeGrid.GridIntegerToWorld(shipController.Position);
        public float GetCurrentMass() => shipController.CalculateShipMass().TotalMass;
        public Vector3D GetNaturalGravity() => shipController.GetNaturalGravity();
        public List<IMyThrust> GetThrusters() => thrusters;
        public List<IMyGyro> GetGyroscopes() => gyroscopes;
        public List<IMySensorBlock> GetSensors() => sensors;
        public ThrustData GetThrustData() => thrustData;
        public double GetMaxThrust() => maxThrust;
        public float GetMaxLoad() => maxLoad;
        public float GetCurrentLoad() => currentLoad;
        public List<IMySlimBlock> GetTargetBlocks() => targetBlocks;
        public int GetCurrentTargetIndex() => currentTargetIndex;
        public NavigationManager GetNavigationManager() => navigationManager;
        public IMyShipWelder GetWelder() => welder;
        public IMyShipGrinder GetGrinder() => grinder;
        public IMyShipConnector GetConnector() => connector;

        #endregion
    }
}