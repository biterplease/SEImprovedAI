using ProtoBuf;
using System;
using System.Collections.Generic;
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
        SCHEDULER_FORWARD,
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
        public Drone.UpdateFlags Flags;
        [ProtoMember(3)]
        public Drone.State? DroneState;
        [ProtoMember(4)]
        public Drone.Capabilities? Capabilities;
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
        public List<Scheduler.Task> Tasks;
        [ProtoMember(2)]
        public long EntityId;
    }


    [ProtoContract]
    public class LogisticsUpdate
    {
        [ProtoMember(1)]
        public Inventory Inventory;
        [ProtoMember(2)]
        public List<Vector3D> Connectors;
        [ProtoMember(3)]
        public DateTime Timestamp;
        [ProtoMember(4)]
        public long EntityId;
        [ProtoMember(5)]
        public LogisticsComputer.OperationMode OperationMode;
    }
    [ProtoContract]
    public class InventoryRequisition
    {
        [ProtoMember(1)]
        public Inventory Inventory;
        [ProtoMember(2)]
        public Vector3D ConnectorLocation;
        [ProtoMember(3)]
        public DateTime Timestamp;
        [ProtoMember(4)]
        public long RequestingEntityId;
        [ProtoMember(5)]
        public Inventory.RequisitionType RequisitionType;
        /// <summary>
        /// Does the grid move.
        /// </summary>
        [ProtoMember(6)]
        public bool IsStatic;
    }
}
