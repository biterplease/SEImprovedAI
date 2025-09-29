using BetterAIConstructor.Network;
using ImprovedAI.Network;
using System;
using System.Collections.Generic;
using VRageMath;

namespace ImprovedAI.Network.Examples
{
    /// <summary>
    /// Example usage patterns for the MessageQueue system.
    /// This demonstrates how to use the messaging system in a Space Engineers mod context.
    /// </summary>
    public static class MessageQueueExample
    {
        private static MessageQueue _messaging = new MessageQueue(TimeSpan.FromSeconds(30));

        // Topic constants - organize by functional area
        #region Topic Definitions

        // Drone operation topics
        private const ushort TOPIC_DRONE_STATUS = 1;
        private const ushort TOPIC_DRONE_ALERTS = 2;
        private const ushort TOPIC_DRONE_PERFORMANCE = 3;

        // Work coordination topics
        private const ushort TOPIC_WORK_ORDERS = 10;
        private const ushort TOPIC_WORK_ASSIGNMENTS = 11;
        private const ushort TOPIC_WORK_COMPLETION = 12;

        // Fleet coordination topics
        private const ushort TOPIC_FLEET_COORDINATION = 20;
        private const ushort TOPIC_COLLISION_AVOIDANCE = 21;
        private const ushort TOPIC_RESOURCE_SHARING = 22;

        // System topics
        private const ushort TOPIC_SYSTEM_HEALTH = 30;
        private const ushort TOPIC_ERROR_REPORTS = 31;

        #endregion

        #region Initialization

        /// <summary>
        /// Initialize the messaging system with typical subscriptions and throttling
        /// </summary>
        public static void InitializeMessaging()
        {
            // Central scheduler subscribes to all drone updates
            long schedulerId = 12345L;
            _messaging.Subscribe(schedulerId, TOPIC_DRONE_STATUS);
            _messaging.Subscribe(schedulerId, TOPIC_WORK_COMPLETION);
            _messaging.Subscribe(schedulerId, TOPIC_DRONE_ALERTS);

            // Scheduler needs frequent updates for coordination
            _messaging.SetReadThrottle(schedulerId, TimeSpan.FromMilliseconds(50));  // 20 Hz

            // Example drones subscribe to work orders and fleet coordination
            long droneId1 = 67890L;
            long droneId2 = 67891L;

            foreach (var droneId in new[] { droneId1, droneId2 })
            {
                _messaging.Subscribe(droneId, TOPIC_WORK_ORDERS);
                _messaging.Subscribe(droneId, TOPIC_WORK_ASSIGNMENTS);
                _messaging.Subscribe(droneId, TOPIC_FLEET_COORDINATION);
                _messaging.Subscribe(droneId, TOPIC_COLLISION_AVOIDANCE);

                // Drones read less frequently to reduce load
                _messaging.SetReadThrottle(droneId, TimeSpan.FromMilliseconds(500)); // 2 Hz
            }

            // System monitor for health and error tracking
            long monitorId = 99999L;
            _messaging.Subscribe(monitorId, TOPIC_SYSTEM_HEALTH);
            _messaging.Subscribe(monitorId, TOPIC_ERROR_REPORTS);
            _messaging.SetReadThrottle(monitorId, TimeSpan.FromSeconds(1)); // 1 Hz
        }

        #endregion

        #region Publishing Examples

        /// <summary>
        /// Example: Drone publishes status update
        /// </summary>
        public static void SendDroneStatus(long droneId, Vector3D position, string status, float batteryLevel)
        {
            // Create a simple status message (in real implementation, use proper serialization)
            var statusData = System.Text.Encoding.UTF8.GetBytes(
                $"{droneId}|{position.X:F2},{position.Y:F2},{position.Z:F2}|{status}|{batteryLevel:F1}");

            _messaging.SendMessage(TOPIC_DRONE_STATUS, statusData);
        }

        /// <summary>
        /// Example: Scheduler sends work order to specific drone
        /// </summary>
        public static void SendWorkOrder(string taskType, Vector3D location, Dictionary<string, string> parameters)
        {
            // Serialize work order (simplified example)
            var orderData = System.Text.Encoding.UTF8.GetBytes(
                $"WORK_ORDER|{taskType}|{location.X:F2},{location.Y:F2},{location.Z:F2}|{string.Join(";", parameters)}");

            _messaging.SendMessage(TOPIC_WORK_ORDERS, orderData);
        }

        /// <summary>
        /// Example: Emergency alert broadcast
        /// </summary>
        public static void SendEmergencyAlert(string alertType, Vector3D location, string details)
        {
            var alertData = System.Text.Encoding.UTF8.GetBytes(
                $"EMERGENCY|{alertType}|{location.X:F2},{location.Y:F2},{location.Z:F2}|{details}|{DateTime.UtcNow:O}");

            _messaging.SendMessage(TOPIC_DRONE_ALERTS, alertData);
        }

        /// <summary>
        /// Example: Fleet coordination message for collision avoidance
        /// </summary>
        public static void SendCollisionWarning(long droneId, Vector3D position, Vector3D velocity)
        {
            var warningData = System.Text.Encoding.UTF8.GetBytes(
                $"COLLISION_WARNING|{droneId}|{position.X:F2},{position.Y:F2},{position.Z:F2}|{velocity.X:F2},{velocity.Y:F2},{velocity.Z:F2}");

            _messaging.SendMessage(TOPIC_COLLISION_AVOIDANCE, warningData);
        }

        #endregion

        #region Reading Examples

        /// <summary>
        /// Example: Drone reads work orders
        /// </summary>
        public static List<string> ReadWorkOrders(long droneId)
        {
            var workOrders = new List<string>();
            var messages = _messaging.ReadMessages(droneId, TOPIC_WORK_ORDERS, maxMessages: 5);

            foreach (var messageData in messages)
            {
                var messageText = System.Text.Encoding.UTF8.GetString(messageData);
                workOrders.Add(messageText);
            }

            return workOrders;
        }

        /// <summary>
        /// Example: Scheduler reads drone status updates
        /// </summary>
        public static List<DroneStatus> ReadDroneStatuses(long schedulerId)
        {
            var statuses = new List<DroneStatus>();
            var messages = _messaging.ReadMessages(schedulerId, TOPIC_DRONE_STATUS, maxMessages: 20);

            foreach (var messageData in messages)
            {
                var messageText = System.Text.Encoding.UTF8.GetString(messageData);
                var status = ParseDroneStatus(messageText);
                if (status != null)
                    statuses.Add(status);
            }

            return statuses;
        }

        /// <summary>
        /// Example: Check for collision warnings
        /// </summary>
        public static List<CollisionWarning> CheckCollisionWarnings(long droneId)
        {
            var warnings = new List<CollisionWarning>();
            var messages = _messaging.ReadMessages(droneId, TOPIC_COLLISION_AVOIDANCE, maxMessages: 10);

            foreach (var messageData in messages)
            {
                var messageText = System.Text.Encoding.UTF8.GetString(messageData);
                var warning = ParseCollisionWarning(messageText);
                if (warning != null)
                    warnings.Add(warning);
            }

            return warnings;
        }

        #endregion

        #region Message Parsing (Simplified Examples)

        private static DroneStatus ParseDroneStatus(string message)
        {
            try
            {
                var parts = message.Split('|');
                if (parts.Length >= 4)
                {
                    var droneId = long.Parse(parts[0]);
                    var positionParts = parts[1].Split(',');
                    var position = new Vector3D(
                        double.Parse(positionParts[0]),
                        double.Parse(positionParts[1]),
                        double.Parse(positionParts[2]));
                    var status = parts[2];
                    var batteryLevel = float.Parse(parts[3]);

                    return new DroneStatus
                    {
                        DroneId = droneId,
                        Position = position,
                        Status = status,
                        BatteryLevel = batteryLevel
                    };
                }
            }
            catch (Exception)
            {
                // In production, log the error
            }

            return null;
        }

        private static CollisionWarning ParseCollisionWarning(string message)
        {
            try
            {
                var parts = message.Split('|');
                if (parts.Length >= 4 && parts[0] == "COLLISION_WARNING")
                {
                    var droneId = long.Parse(parts[1]);
                    var positionParts = parts[2].Split(',');
                    var velocityParts = parts[3].Split(',');

                    return new CollisionWarning
                    {
                        DroneId = droneId,
                        Position = new Vector3D(
                            double.Parse(positionParts[0]),
                            double.Parse(positionParts[1]),
                            double.Parse(positionParts[2])),
                        Velocity = new Vector3D(
                            double.Parse(velocityParts[0]),
                            double.Parse(velocityParts[1]),
                            double.Parse(velocityParts[2]))
                    };
                }
            }
            catch (Exception)
            {
                // In production, log the error
            }

            return null;
        }

        #endregion

        #region Data Structures

        public class DroneStatus
        {
            public long DroneId { get; set; }
            public Vector3D Position { get; set; }
            public string Status { get; set; }
            public float BatteryLevel { get; set; }
        }

        public class CollisionWarning
        {
            public long DroneId { get; set; }
            public Vector3D Position { get; set; }
            public Vector3D Velocity { get; set; }
        }

        #endregion

        #region Diagnostic and Monitoring

        /// <summary>
        /// Get system health overview
        /// </summary>
        public static void PrintSystemHealth()
        {
            var topicCounts = _messaging.GetTopicMessageCounts();
            var subscriberCounts = _messaging.GetTopicSubscriberCounts();
            var totalMessages = _messaging.GetTotalMessageCount();

            Console.WriteLine($"Total messages in system: {totalMessages}");

            foreach (var kvp in topicCounts)
            {
                var topic = kvp.Key;
                var messageCount = kvp.Value;
                var subCount = subscriberCounts.ContainsKey(topic) ? subscriberCounts[topic] : 0;

                Console.WriteLine($"Topic {topic}: {messageCount} messages, {subCount} subscribers");
            }
        }

        /// <summary>
        /// Cleanup old messages manually
        /// </summary>
        public static void PerformMaintenanceCleanup()
        {
            _messaging.ForceCleanup();
        }

        #endregion
    }
}