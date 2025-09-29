using ImprovedAI.BlockConfig;
using ImprovedAIScheduler;
using ProtoBuf;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using VRageMath;

namespace ImprovedAI.Messages
{
    public enum MessageTopics : ushort
    {
        DRONE_REGISTRATION,
        DRONE_REPORTS,
        DRONE_PERFORMANCE,
        DRONE_TASK_ASSIGNMENT,
        LOGISTIC_REGISTRATION,
        LOGISTIC_UPDATE,
        LOGISTIC_REQUEST,
        LOGISTIC_PUSH,
    }

    [ProtoContract]
    public enum TaskType : byte
    {
        /// <summary>
        ///  Weld a block with needed components.
        /// </summary>
        [ProtoEnum]
        PreciseWelding,
        /// <summary>
        /// Grind a specific block down until capacity.
        /// </summary>
        [ProtoEnum]
        PreciseGrinding,
        /// <summary>
        /// Go mine some ore at specific location
        /// </summary>
        [ProtoEnum]
        PreciseDrilling,
        /// <summary>
        /// Go to specific location, connect to connector at exact location, and attempt to transfer items.
        /// </summary>
        [ProtoEnum]
        PreciseDelivery,
        /// <summary>
        /// Go to specific location, and drop payload at location.
        /// </summary>
        [ProtoEnum]
        PreciseDrop,
        /// <summary>
        /// Drop a cargo pod with a parachute at specific location.
        /// </summary>
        [ProtoEnum]
        PreciseCargoAirdrop,
        /// <summary>
        /// Go to specific location, connect to connector at exact location, and attempt to retrieve items.
        /// </summary>
        [ProtoEnum]
        PreciseFetch,
        /// <summary>
        ///  Go to specified location, find a matching block to weld, and weld it.
        /// </summary>
        [ProtoEnum]
        ScanWeld,
        /// <summary>
        /// Go to specified location, find a matching block to grind, and grind it.
        /// </summary>
        [ProtoEnum]
        ScanGrind,
        /// <summary>
        /// Go to location, and scan for specified ore, and mine it.
        /// </summary>
        [ProtoEnum]
        ScanDrill,
        /// <summary>
        /// Go to location, scan for a friendly connector, connect and attempt to unload items.
        /// </summary>
        [ProtoEnum]
        ScanDelivery,
        /// <summary>
        /// Go to location, scan for a friendly connector, connect and attempt to load items.
        /// </summary>
        [ProtoEnum]
        ScanFetch,
        /// <summary>
        /// Attempt to push items into the logistics network.
        /// </summary>
        [ProtoEnum]
        ActiveProvide,
        [ProtoEnum]
        BecomeStandAlone,
    }
    [Serializable,ProtoContract(UseProtoMembersOnly = true)]
    public struct Task
    {
        [ProtoMember(1)]
        public IAIInventory Payload;
        [ProtoMember(2)]
        public Vector3D Position;
        [ProtoMember(3)]
        public ushort TaskId;
        [ProtoMember(4)]
        public TaskType TaskType;
    }
    [ProtoContract,Flags]
    public enum DroneUpdateFlags : byte
    {
        [ProtoEnum]
        None = 0,
        [ProtoEnum]
        Error = 1,
        [ProtoEnum]
        Registration = 2,
        [ProtoEnum]
        TaskComplete = 4,
        [ProtoEnum]
        StateChanged = 8,
        [ProtoEnum]
        CapabilitiesChanged = 16,
        [ProtoEnum]
        BatteryUpdate = 32,
        [ProtoEnum]
        H2Update = 64,
        [ProtoEnum]
        GoingOutOfRange = 128,
    }

    /// <summary>
    /// Drone messages sent back to the scheduler.
    /// </summary>
    [Serializable,ProtoContract(UseProtoMembersOnly = true)]
    public class DroneReport
    {
        [ProtoMember(1)]
        public long DroneEntityId;
        [ProtoMember(2)]
        public DroneUpdateFlags Flags;

        [ProtoMember(3)]
        public DroneState? DroneState;
        [ProtoMember(4)]
        public DroneCapabilities? Capabilities;
        [ProtoMember(5)]
        public ushort? TaskId;
        [ProtoMember(6)]
        public float? BatteryChargePercent;
        [ProtoMember(7)]
        public float? BatteryRechargeThreshold;
        [ProtoMember(8)]
        public float? BatteryOperationalThreshold;
        [ProtoMember(9)]
        public float? H2Level;
        [ProtoMember(10)]
        public float? H2RefuelThreshold;
        [ProtoMember(11)]
        public float? H2OperationalThreshold;
        [ProtoMember(12)]
        public string ErrorMessage;
    }

    [ProtoContract]
    public class TaskAssignment
    {
        [ProtoMember(1)]
        public List<Task> Tasks;
        [ProtoMember(2)]
        public long EntityId;
    }

    [ProtoContract]
    public struct InventoryProvider
    {
        [ProtoMember(1)]
        public IAIInventory Inventory;
        [ProtoMember(2)]
        public ushort ProviderEntityId;
        [ProtoMember(3)]
        public long LogisticsGridEntityId;
    }


    [ProtoContract]
    public class InventoryRequest
    {
        [ProtoMember(1)]
        public IAIInventory Inventory;
        [ProtoMember(2)]
        public ushort RequestingEntityId;
        [ProtoMember(3)]
        public long LogisticsGridEntityId;
    }

    [ProtoContract]
    public enum EntityType : byte
    {
        [ProtoEnum]
        Drone = 1,
        [ProtoEnum]
        LogisticsGrid = 2,
    }
    [ProtoContract]
    public class RegisterLogistics
    {
        [ProtoMember(1)]
        public IAIInventory Inventory;
        [ProtoMember(2)]
        public List<Vector3D> Connectors;
        [ProtoMember(3)]
        public ushort EntityId;
    }
}
