using ImprovedAI.Config;
using ImprovedAI.Utils.Logging;
using Sandbox.ModAPI;
using System;
using System.Collections.Generic;
using VRage.Collections;
using VRage.Utils;
using System.Text;
using ImprovedAI.Utils;

namespace ImprovedAI.Network
{
    public class MessageQueue
    {
        private struct TimestampedMessage
        {
            public byte[] Data;
            public DateTime Timestamp;
            public long SenderId;
            public ushort MessageId;
            public bool RequiresAck;
            public MessageSerializationMode SerializationMode;
        }

        // Configuration
        private readonly MessageSerializationMode serializationMode = ServerConfig.DroneNetwork.MessageSerializationMode;

        // Existing fields...
        private readonly MyConcurrentDictionary<ushort, MyConcurrentQueue<TimestampedMessage>> _topicQueues;
        private readonly MyConcurrentDictionary<ushort, MyConcurrentHashSet<long>> _topicSubscribers;
        private readonly MyConcurrentDictionary<long, DateTime> _lastReadTimes;
        private readonly MyConcurrentDictionary<long, TimeSpan> _readThrottleIntervals;
        private readonly MyConcurrentDictionary<ushort, MyConcurrentHashSet<long>> _messageAcknowledgments;
        private readonly MyConcurrentDictionary<long, MyConcurrentHashSet<ushort>> _pendingAcknowledgments;

        private readonly TimeSpan _messageExpiration;
        private readonly object _cleanupLock = new object();
        private DateTime _lastCleanup = DateTime.UtcNow;
        private readonly TimeSpan _cleanupInterval = TimeUtil.TickToTimeSpan(ServerConfig.DroneNetwork.MessageCleanupIntervalTicks);
        private int _messageCounter = 0;

        /// <summary>
        /// Create MessageQueue with optional ProtoBuf serialization
        /// </summary>
        /// <param name="messageExpiration">How long messages stay in queue</param>
        /// <param name="defaultSerializationMode">Default serialization mode</param>
        /// <param name="allowMixedModes">Allow different serialization modes in same queue</param>
        public MessageQueue(TimeSpan? messageExpiration = null, bool allowMixedModes = true)
        {
            _messageExpiration = TimeUtil.TickToTimeSpan(ServerConfig.DroneNetwork.MessageRetentionTicks);

            _topicQueues = new MyConcurrentDictionary<ushort, MyConcurrentQueue<TimestampedMessage>>();
            _topicSubscribers = new MyConcurrentDictionary<ushort, MyConcurrentHashSet<long>>();
            _lastReadTimes = new MyConcurrentDictionary<long, DateTime>();
            _readThrottleIntervals = new MyConcurrentDictionary<long, TimeSpan>();
            _messageAcknowledgments = new MyConcurrentDictionary<ushort, MyConcurrentHashSet<long>>();
            _pendingAcknowledgments = new MyConcurrentDictionary<long, MyConcurrentHashSet<ushort>>();
        }

        /// <summary>
        /// Send message with specific serialization mode
        /// </summary>
        public ushort SendMessage<T>(ushort topic, T message, long senderId = 0, bool requiresAck = false)
            where T : class
        {
            if (message == null) return 0;

            try
            {
                var serializedData = SerializeMessage(message);
                if (serializedData == null) return 0;

                return SendMessage(topic, serializedData, senderId, requiresAck, serializationMode);
            }
            catch (Exception ex)
            {
                Log.Error("SendMessage<T> error: {0}", ex);
                return 0;
            }
        }
        private ushort SendMessage(ushort topic, byte[] message, long senderId = 0, bool requiresAck = false, MessageSerializationMode usedMode = MessageSerializationMode.ProtoBuf)
        {
            if (message == null) return 0;

            try
            {
                var queue = _topicQueues.GetOrAdd(topic, _ => new MyConcurrentQueue<TimestampedMessage>());
                var messageId = unchecked((ushort)System.Threading.Interlocked.Increment(ref _messageCounter));

                var timestampedMessage = new TimestampedMessage
                {
                    Data = message,
                    Timestamp = DateTime.UtcNow,
                    SenderId = senderId,
                    MessageId = messageId,
                    RequiresAck = requiresAck,
                    SerializationMode = usedMode, // Always track the serialization mode used
                };

                queue.Enqueue(timestampedMessage);

                if (requiresAck && senderId != 0)
                {
                    var pendingAcks = _pendingAcknowledgments.GetOrAdd(senderId, _ => new MyConcurrentHashSet<ushort>());
                    pendingAcks.Add(messageId);
                }

                TryPerformCleanup();
                return messageId;
            }
            catch (Exception ex)
            {
                Log.Error("SendMessage error: {0}", ex);
                return 0;
            }
        }
        private byte[] SerializeMessage<T>(T message) where T : class
        {
            if (message == null) return null;

            try
            {
                if (serializationMode == MessageSerializationMode.ProtoBuf)
                {
                    return TryProtoBufSerialization(message);
                }
                else
                {
                    return TryXmlSerialization(message);
                }
            }
            catch (Exception ex)
            {
                Log.Warning("Primary serialization failed, trying fallback: {0}", ex.Message);

                // Fallback: try the opposite serialization mode
                try
                {
                    if (serializationMode == MessageSerializationMode.ProtoBuf)
                    {
                        return TryXmlSerialization(message);
                    }
                    else
                    {
                        return TryProtoBufSerialization(message);
                    }
                }
                catch (Exception fallbackEx)
                {
                    Log.Error("All serialization attempts failed: Primary: {0}, Fallback: {1}",
                        ex.Message, fallbackEx.Message);
                    return null;
                }
            }
        }

        private byte[] TryProtoBufSerialization<T>(T message) where T : class
        {
            return MyAPIGateway.Utilities.SerializeToBinary(message);
        }

        private byte[] TryXmlSerialization<T>(T message) where T : class
        {
            var xmlString = MyAPIGateway.Utilities.SerializeToXML(message);
            return Encoding.UTF8.GetBytes(xmlString);
        }

        /// <summary>
        /// Read raw messages (original method)
        /// </summary>
        public List<T> ReadMessages<T>(long subscriberId, ushort topic, ushort maxMessages = 10)
            where T : class, new()
        {
            var messages = new List<T>();

            try
            {
                MyConcurrentHashSet<long> subscribers;
                if (!_topicSubscribers.TryGetValue(topic, out subscribers) ||
                    !subscribers.Contains(subscriberId))
                {
                    return messages;
                }

                if (!CanReadNow(subscriberId))
                {
                    return messages;
                }

                DateTime oldTime;
                _lastReadTimes.TryRemove(subscriberId, out oldTime);
                _lastReadTimes.TryAdd(subscriberId, DateTime.UtcNow);

                MyConcurrentQueue<TimestampedMessage> queue;
                if (_topicQueues.TryGetValue(topic, out queue))
                {
                    var currentTime = DateTime.UtcNow;

                    TimestampedMessage msg;
                    while (messages.Count < maxMessages && queue.TryDequeue(out msg))
                    {
                        if (currentTime - msg.Timestamp <= _messageExpiration)
                        {
                            var deserializedMsg = DeserializeMessage<T>(msg.Data);
                            if (deserializedMsg != null)
                            {
                                messages.Add(deserializedMsg);
                            }

                            if (msg.RequiresAck)
                            {
                                AckMessage(subscriberId, msg.MessageId);
                            }
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Log.Error("ReadMessages<T> error: {0}", ex);
            }

            return messages;
        }

        /// <summary>
        /// Automatically detect and deserialize message data
        /// </summary>
        private T DeserializeMessage<T>(byte[] data) where T : class, new()
        {
            if (data == null || data.Length == 0) return null;

            try
            {
                // First, try the configured serialization mode
                if (serializationMode == MessageSerializationMode.ProtoBuf)
                {
                    return TryProtoBufDeserialization<T>(data);
                }
                else
                {
                    return TryXmlDeserialization<T>(data);
                }
            }
            catch (Exception ex)
            {
                Log.Warning("Primary deserialization failed, trying fallback: {0}", ex.Message);

                // Fallback: try the opposite serialization mode
                try
                {
                    if (serializationMode == MessageSerializationMode.ProtoBuf)
                    {
                        return TryXmlDeserialization<T>(data);
                    }
                    else
                    {
                        return TryProtoBufDeserialization<T>(data);
                    }
                }
                catch (Exception fallbackEx)
                {
                    Log.Error("All deserialization attempts failed: Primary: {0}, Fallback: {1}",
                        ex.Message, fallbackEx.Message);
                    return null;
                }
            }
        }

        private T TryProtoBufDeserialization<T>(byte[] data) where T : class, new()
        {
            return MyAPIGateway.Utilities.SerializeFromBinary<T>(data);
        }

        private T TryXmlDeserialization<T>(byte[] data) where T : class, new()
        {
            var xmlString = Encoding.UTF8.GetString(data);
            return MyAPIGateway.Utilities.SerializeFromXML<T>(xmlString);
        }

        /// <summary>
        /// Read ProtoBuf messages with automatic deserialization
        /// </summary>
        public List<T> ReadProtoBufMessages<T>(long subscriberId, ushort topic, ushort maxMessages = 10)
            where T : class
        {
            var messages = new List<T>();

            try
            {
                MyConcurrentHashSet<long> subscribers;
                if (!_topicSubscribers.TryGetValue(topic, out subscribers) ||
                    !subscribers.Contains(subscriberId))
                {
                    return messages;
                }

                if (!CanReadNow(subscriberId))
                {
                    return messages;
                }

                DateTime oldTime;
                _lastReadTimes.TryRemove(subscriberId, out oldTime);
                _lastReadTimes.TryAdd(subscriberId, DateTime.UtcNow);

                MyConcurrentQueue<TimestampedMessage> queue;
                if (_topicQueues.TryGetValue(topic, out queue))
                {
                    var currentTime = DateTime.UtcNow;

                    TimestampedMessage msg;
                    while (messages.Count < maxMessages && queue.TryDequeue(out msg))
                    {
                        if (currentTime - msg.Timestamp <= _messageExpiration)
                        {
                            try
                            {
                                // Deserialize based on how it was stored
                                T deserializedMsg = null;

                                if (serializationMode == MessageSerializationMode.ProtoBuf)
                                {
                                    deserializedMsg = MyAPIGateway.Utilities.SerializeFromBinary<T>(msg.Data);
                                }
                                else
                                {
                                    // Try to deserialize raw data as ProtoBuf (for mixed mode compatibility)
                                    deserializedMsg = MyAPIGateway.Utilities.SerializeFromXML<T>(Encoding.Default.GetString(msg.Data));
                                }

                                if (deserializedMsg != null)
                                {
                                    messages.Add(deserializedMsg);
                                }
                            }
                            catch (Exception ex)
                            {
                                Log.Warning("Failed to deserialize message ID {0}: {1}", msg.MessageId, ex.Message);
                                // Skip this message but continue processing others
                            }

                            if (msg.RequiresAck)
                            {
                                AckMessage(subscriberId, msg.MessageId);
                            }
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Log.Error("ReadProtoBufMessages error: {0}", ex);
            }

            return messages;
        }



        // Include all your existing methods (Subscribe, Unsubscribe, AckMessage, etc.)
        // ... (existing implementation stays the same)
        public void Subscribe(long subscriberId, ushort topic)
        {
            try
            {
                var subscribers = _topicSubscribers.GetOrAdd(topic, _ => new MyConcurrentHashSet<long>());
                subscribers.Add(subscriberId);
                _readThrottleIntervals.TryAdd(subscriberId, TimeSpan.FromMilliseconds(100));
            }
            catch (Exception ex)
            {
                Log.Error("Subscribe error: {0}", ex);
            }
        }

        public bool AckMessage(long subscriberId, ushort messageId)
        {
            try
            {
                var ackSet = _messageAcknowledgments.GetOrAdd(messageId, _ => new MyConcurrentHashSet<long>());
                return ackSet.Add(subscriberId);
            }
            catch (Exception ex)
            {
                Log.Error("AckMessage error: {0}", ex);
                return false;
            }
        }

        public bool IsMessageAcknowledged(long senderId, ushort messageId)
        {
            try
            {
                MyConcurrentHashSet<ushort> pendingAcks;
                if (_pendingAcknowledgments.TryGetValue(senderId, out pendingAcks))
                {
                    return !pendingAcks.Contains(messageId);
                }
                return true;
            }
            catch (Exception ex)
            {
                Log.Error("IsMessageAcknowledged error: {0}", ex);
                return false;
            }
        }

        private bool CanReadNow(long subscriberId)
        {
            DateTime lastRead;
            if (!_lastReadTimes.TryGetValue(subscriberId, out lastRead))
                return true;

            TimeSpan throttleInterval;
            if (!_readThrottleIntervals.TryGetValue(subscriberId, out throttleInterval))
                return true;

            return DateTime.UtcNow - lastRead >= throttleInterval;
        }
        private void TryPerformCleanup()
        {
            var now = DateTime.UtcNow;
            if (now - _lastCleanup < _cleanupInterval)
                return;

            lock (_cleanupLock)
            {
                if (now - _lastCleanup < _cleanupInterval)
                    return; // Another thread already did cleanup

                _lastCleanup = now;
                PerformCleanup();
            }
        }

        private void PerformCleanup()
        {
            try
            {
                var currentTime = DateTime.UtcNow;
                var topicsToRemove = new List<ushort>();

                foreach (var kvp in _topicQueues)
                {
                    var topic = kvp.Key;
                    var queue = kvp.Value;

                    // Create a new queue with only non-expired messages
                    var newQueue = new MyConcurrentQueue<TimestampedMessage>();
                    var hasValidMessages = false;

                    TimestampedMessage message;
                    while (queue.TryDequeue(out message))
                    {
                        if (currentTime - message.Timestamp <= _messageExpiration)
                        {
                            newQueue.Enqueue(message);
                            hasValidMessages = true;
                        }
                    }

                    if (hasValidMessages)
                    {
                        // Replace the old queue with the cleaned one by removing and re-adding
                        MyConcurrentQueue<TimestampedMessage> removedQueue;
                        _topicQueues.TryRemove(topic, out removedQueue);
                        _topicQueues.TryAdd(topic, newQueue);
                    }
                    else
                    {
                        // Mark topic for removal if no valid messages
                        topicsToRemove.Add(topic);
                    }
                }

                // Remove empty topics
                foreach (var topic in topicsToRemove)
                {
                    MyConcurrentQueue<TimestampedMessage> removedQueue;
                    _topicQueues.TryRemove(topic, out removedQueue);

                    // Also clean up subscriber lists for removed topics
                    MyConcurrentHashSet<long> subscribers;
                    if (_topicSubscribers.TryGetValue(topic, out subscribers) && subscribers.Count == 0)
                    {
                        MyConcurrentHashSet<long> removedSubscribers;
                        _topicSubscribers.TryRemove(topic, out removedSubscribers);
                    }
                }

                // Clean up old acknowledgments
                CleanupOldAcknowledgments(currentTime);
            }
            catch (Exception ex)
            {
                MyLog.Default.WriteLine($"MessageQueue: Cleanup error: {ex}");
            }
        }

        private void CleanupOldAcknowledgments(DateTime currentTime)
        {
            try
            {
                // Clean up acknowledgment tracking for messages older than expiration time
                // Since we don't store message timestamps in the ack system, we'll use a simpler approach:
                // Remove acknowledgments for message IDs that are significantly old based on the counter

                var currentMessageId = unchecked((ushort)_messageCounter);
                var oldThreshold = unchecked((ushort)(currentMessageId - 1000)); // Rough estimate for old messages

                var acksToRemove = new List<ushort>();
                foreach (var kvp in _messageAcknowledgments)
                {
                    var messageId = kvp.Key;

                    // Simple heuristic: if message ID is much lower than current, it's probably old
                    // Note: This handles counter wraparound reasonably well for ushort range
                    var ageDiff = unchecked((ushort)(currentMessageId - messageId));
                    if (ageDiff > 500 && ageDiff < 60000) // Old but not wrapped around
                    {
                        acksToRemove.Add(messageId);
                    }
                }

                // Remove old acknowledgments
                foreach (var messageId in acksToRemove)
                {
                    MyConcurrentHashSet<long> removedAcks;
                    _messageAcknowledgments.TryRemove(messageId, out removedAcks);

                    // Also remove from pending acknowledgments for all senders
                    foreach (var senderEntry in _pendingAcknowledgments)
                    {
                        senderEntry.Value.Remove(messageId);
                    }
                }
            }
            catch (Exception ex)
            {
                MyLog.Default.WriteLine($"MessageQueue: CleanupOldAcknowledgments error: {ex}");
            }
        }
    }
}