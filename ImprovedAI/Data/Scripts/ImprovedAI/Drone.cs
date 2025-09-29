﻿using ProtoBuf;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ImprovedAI
{

    [ProtoContract(UseProtoMembersOnly = true)]
    public class Drone
    {
        [ProtoContract]
        public enum State
        {
            [ProtoEnum]
            Initializing,
            [ProtoEnum]
            Standby,
            [ProtoEnum]
            NavigatingToTarget,
            [ProtoEnum]
            LoadingInventory,
            [ProtoEnum]
            Welding,
            [ProtoEnum]
            Grinding,
            [ProtoEnum]
            ReturningToBase,
            [ProtoEnum]
            Docking,
            [ProtoEnum]
            RefuelingHydrogen,
            [ProtoEnum]
            RechargingBattery,
            [ProtoEnum]
            Error
        }
        [ProtoContract]
        public enum OperationMode
        {
            [ProtoEnum]
            StandAlone,
            [ProtoEnum]
            ManagedByScheduler,
            [ProtoEnum]
            ManagedByPlayer,
        }
        [Flags, ProtoContract]
        public enum Capabilities
        {
            [ProtoEnum]
            None = 0,
            [ProtoEnum]
            CanWeld = 1,
            [ProtoEnum]
            CanGrind = 2,
            [ProtoEnum]
            CanDrill = 4,
            [ProtoEnum]
            CanAirDrop = 8,
            [ProtoEnum]
            HasWheels = 16,
            [ProtoEnum]
            CanFlyAtmosphere = 32,
            [ProtoEnum]
            CanFlySpace = 64,
            [ProtoEnum]
            RefuelWhenDocked = 128,
            [ProtoEnum]
            RechargeWhenDocked = 256,
            [ProtoEnum]
            CombatReady = 512
        }
        [ProtoContract, Flags]
        public enum UpdateFlags : ushort
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
            [ProtoEnum]
            ReturningIntoRange = 256,
            [ProtoEnum]
            UnderAttack = 512,
        }


        [ProtoMember(1)]
        public long DroneId { get; set; }
        [ProtoMember(2)]
        public Capabilities _Capabilities { get; set; }
        [ProtoMember(3)]
        public State _State { get; set; }
        [ProtoMember(4)]
        public float BatteryLevel { get; set; }
        [ProtoMember(5)]
        public float BatteryRechargeThreshold { get; set; }
        [ProtoMember(6)]
        public float BatteryOperationalThreshold { get; set; }
        [ProtoMember(7)]
        public float H2Level { get; set; }
        [ProtoMember(8)]
        public float H2RefuelThreshold { get; set; }
        [ProtoMember(9)]
        public float H2OperationalThreshold { get; set; }
        [ProtoMember(10)]
        public bool IsOutOfRange { get; set; }
        [ProtoMember(11)]
        public DateTime LastSeenTime { get; set; }
        [ProtoMember(12)]
        public DateTime LastReportTime { get; set; }
            
        public Drone()
        {
            _Capabilities = Capabilities.None;
            _State = State.Standby;
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
            Capabilities capabilities = Capabilities.None,
            State droneState = State.Standby,
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
            _Capabilities = capabilities;
            _State = droneState;
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
            _Capabilities = drone._Capabilities;
            _State = drone._State;
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
}
