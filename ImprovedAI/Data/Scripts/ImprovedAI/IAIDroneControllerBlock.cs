using ImprovedAI.Utils.Logging;
using Sandbox.Common.ObjectBuilders;
using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI;
using System;
using VRage.Game.Components;
using VRage.ModAPI;
using VRage.ObjectBuilders;

namespace ImprovedAI
{
    /// <summary>
    /// Minimal game logic component that bridges the Space Engineers block system with the IAIDroneController functionality
    /// </summary>
    [MyEntityComponentDescriptor(typeof(MyObjectBuilder_RemoteControl), false,
        "ImprovedAISmallDroneController",
        "ImprovedAILargeDroneController")]
    public class IAIDroneControllerBlock : MyGameLogicComponent
    {
        // The actual drone controller instance
        private IAIDroneController droneController;

        // Configuration
        private Drone.OperationMode operationMode = Drone.OperationMode.StandAlone;

        // State tracking
        private bool isInitialized = false;
        private int initializationTicks = 0;
        private const int INITIALIZATION_DELAY = 60; // Wait 1 second for grid to stabilize

        /// <summary>
        /// Provides access to the internal drone controller for terminal controls and debugging
        /// </summary>
        public IAIDroneController DroneController => droneController;

        /// <summary>
        /// Gets whether the drone is enabled
        /// </summary>
        public bool IsEnabled => droneController?.isEnabled ?? false;

        public override void Init(MyObjectBuilder_EntityBase objectBuilder)
        {
            try
            {
                // Create the drone controller instance with this entity
                droneController = new IAIDroneController(Entity, operationMode);

                // Set update flags
                NeedsUpdate = MyEntityUpdateEnum.EACH_FRAME | MyEntityUpdateEnum.EACH_10TH_FRAME;

                Log.Info("Drone controller block initializing: {0}", Entity.DisplayName);
            }
            catch (Exception ex)
            {
                Log.Error("Init", ex);
            }
        }

        public override void UpdateOnceBeforeFrame()
        {
            try
            {
                if (!MyAPIGateway.Session.IsServer)
                    return;

                // Initialize drone controller's base components
                if (droneController != null)
                {
                    droneController.Init(this, Entity.GetObjectBuilder());
                }
            }
            catch (Exception ex)
            {
                Log.Error("UpdateOnceBeforeFrame", ex);
            }
        }

        public override void UpdateBeforeSimulation()
        {
            try
            {
                if (!isInitialized)
                {
                    initializationTicks++;
                    if (initializationTicks >= INITIALIZATION_DELAY)
                    {
                        InitializeDroneController();
                    }
                    return;
                }

                if (droneController != null && MyAPIGateway.Session.IsServer)
                {
                    droneController.UpdateBeforeSimulation(this);
                }
            }
            catch (Exception ex)
            {
                Log.Error("UpdateBeforeSimulation", ex);
            }
        }

        public override void UpdateBeforeSimulation10()
        {
            try
            {
                if (!isInitialized)
                    return;

                droneController?.UpdateBeforeSimulation10(this);
            }
            catch (Exception ex)
            {
                Log.Error("UpdateBeforeSimulation10", ex);
            }
        }

        private void InitializeDroneController()
        {
            try
            {
                if (isInitialized)
                    return;

                // Load configuration from block storage if available
                LoadConfiguration();

                // Mark as initialized
                isInitialized = true;

                // Register with session
                IAISession.Instance?.RegisterDroneController(this);

                MyAPIGateway.Utilities.ShowMessage("ImprovedAI_Drone",
                    $"Drone controller initialized: {Entity.DisplayName}");
            }
            catch (Exception ex)
            {
                Log.Error("InitializeDroneController", ex);
            }
        }

        private void LoadConfiguration()
        {
            try
            {
                if (Entity.Storage != null)
                {
                    var storage = Entity.Storage.GetValue(IAISession.ModGuid);
                    if (!string.IsNullOrEmpty(storage))
                    {
                        ParseConfiguration(storage);
                    }
                }
            }
            catch (Exception ex)
            {
                Log.Error("LoadConfiguration", ex);
                // Use default configuration on error
            }
        }

        private void SaveConfiguration()
        {
            try
            {
                if (Entity.Storage == null)
                {
                    Entity.Storage = new MyModStorageComponent();
                }

                var configString = SerializeConfiguration();
                Entity.Storage.SetValue(Guid.Parse("89afa424-d317-41de-992c-be2854c2f158"), configString);
            }
            catch (Exception ex)
            {
                Log.Error("SaveConfiguration", ex);
            }
        }

        private void ParseConfiguration(string config)
        {
            var parts = config.Split(';');
            foreach (var part in parts)
            {
                var kvp = part.Split('=');
                if (kvp.Length != 2)
                    continue;

                var key = kvp[0].Trim();
                var value = kvp[1].Trim();

                switch (key)
                {
                    case "OperationMode":
                        Enum.TryParse<Drone.OperationMode>(value, out operationMode);
                        break;
                }
            }
        }

        private string SerializeConfiguration()
        {
            return $"OperationMode={operationMode}";
        }

        /// <summary>
        /// Sets the drone operation mode
        /// </summary>
        public void SetOperationMode(Drone.OperationMode mode)
        {
            operationMode = mode;
            if (droneController != null)
            {
                droneController.operationMode = mode;
            }
            SaveConfiguration();
        }

        /// <summary>
        /// Enables or disables the drone
        /// </summary>
        public void SetEnabled(bool enabled)
        {
            if (droneController != null)
            {
                droneController.isEnabled = enabled;
            }
        }

        /// <summary>
        /// Gets diagnostic information about the drone
        /// </summary>
        public string GetDiagnostics()
        {
            // Placeholder for diagnostics
            return droneController != null ? "Drone operational" : "Drone not initialized";
        }

        /// <summary>
        /// Resets the drone to initial state
        /// </summary>
        public void Reset()
        {
            droneController?.ResetDrone();
        }

        public override void MarkForClose()
        {
            try
            {
                // Clean shutdown
                if (droneController != null)
                {
                    droneController.ResetDrone();
                }

                // Save configuration before closing
                SaveConfiguration();

                // Unregister from session
                IAISession.Instance?.UnregisterDroneController(Entity.EntityId);

                // Clear references
                droneController = null;

                MyAPIGateway.Utilities.ShowMessage("ImprovedAI_Drone", $"Drone controller closed: {Entity?.DisplayName}");
            }
            catch (Exception ex)
            {
                Log.Error("MarkForClose", ex);
            }
        }
    }
}