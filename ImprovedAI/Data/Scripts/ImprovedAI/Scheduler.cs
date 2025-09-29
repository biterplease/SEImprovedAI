using ProtoBuf;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRageMath;

namespace ImprovedAI
{
    [ProtoContract(UseProtoMembersOnly = true)]
    public class Scheduler
    {
        [Flags,ProtoContract]
        public enum WorkModes : ushort
        {
            [ProtoEnum]
            None = 0,
            [ProtoEnum]
            Scan = 1,
            [ProtoEnum]
            WeldUnfinishedBlocks = 2,
            [ProtoEnum]
            RepairDamagedBlocks = 4,
            [ProtoEnum]
            WeldProjectedBlocks = 8,
            [ProtoEnum]
            Grind = 16,
            [ProtoEnum]
            DeliverCargo = 32,
            [ProtoEnum]
            FetchCargo = 64,
            [ProtoEnum]
            DropCargo = 128,
            [ProtoEnum]
            AirDropCargo = 256,
        }

        [ProtoContract]
        public enum State : byte
        {
            [ProtoEnum]
            Initializing,
            [ProtoEnum]
            Standby,
            [ProtoEnum]
            ScanningForTasks,
            [ProtoEnum]
            AssigningTasks,
            [ProtoEnum]
            Error
        }
        [Flags, ProtoContract]
        public enum SearchModes : byte
        {
            [ProtoEnum]
            ConnectedGrids = 1,
            [ProtoEnum]
            BoundingBox = 2
        }

        [Flags, ProtoContract]
        public enum OperationMode : byte
        {
            [ProtoEnum]
            None = 0,
            [ProtoEnum]
            StandAloneDroneScheduler = 1,
            /// <summary>
            /// Forward messages to other schedulers.
            /// </summary>
            [ProtoEnum]
            Repeater = 2,
            /// <summary>
            /// Standard operation mode. Assign tasks to multiple drones, receive updates from multiple drones.
            /// </summary>
            [ProtoEnum]
            Orchestrator = 4,
            /// <summary>
            /// If no drones are available to this scheduler, forward found tasks to other schedulers.
            /// </summary>
            [ProtoEnum]
            DelegateIfNoDrones = 8,
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
        [Serializable, ProtoContract(UseProtoMembersOnly = true)]
        public class Task
        {
            /// <summary>
            /// Payload required for the task, if any.
            /// </summary>
            [ProtoMember(1)]
            public Inventory Payload { get; set; }
            [ProtoMember(2)]
            public Vector3D Position { get; set; }
            /// <summary>
            /// Scheduler that assigned the task.
            /// </summary>
            [ProtoMember(3)]
            public long AssignedBy;
            [ProtoMember(4)]
            public DateTime AssignedTime { get; set; }
            [ProtoMember(5)]
            public DateTime CreatedTime { get; set; }
            [ProtoMember(6)]
            public uint TaskId { get; set; }
            [ProtoMember(7)]
            public TaskType TaskType { get; set; }
            /// <summary>
            /// Is this task out of scheduler antenna range.
            /// </summary>
            [ProtoMember(8)]
            public bool OutOfSchedulerRange { get; set; }
            public Task() { }

            public Task(Inventory payload, Vector3D position, long assignedBy, DateTime assignedTime, ushort taskId, TaskType taskType, bool outOfSchedulerRange)
            {
                TaskId = taskId;
                Payload = payload;
                Position = position;
                AssignedBy = assignedBy;
                AssignedTime = assignedTime;
                CreatedTime = DateTime.UtcNow;
                TaskType = taskType;
                OutOfSchedulerRange = outOfSchedulerRange;
            }
        }
    }
}
