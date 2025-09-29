using ImprovedAI.Config;
using ImprovedAI.Utils;
using ImprovedAI.Utils.Logging;
using ProtoBuf;
using Sandbox.ModAPI;
using System;
using System.Collections.Generic;
using System.Text;
using VRage.Collections;
using VRage.Game;
using VRage.Game.ModAPI;
using VRage.ModAPI;
using VRage.Utils;
using VRageMath;

namespace ImprovedAI.Network
{
    /// <summary>
    /// Cached antenna info for an entity
    /// </summary>
    public class AntennaCache
    {
        public IMyRadioAntenna Antenna;
        public Vector3D Position;
        public double TransmitRange;
        public long OwnerId;
        public DateTime LastUpdate;

        public bool IsValid()
        {
            return Antenna != null &&
                   Antenna.IsFunctional &&
                   Antenna.Enabled &&
                   Antenna.EnableBroadcasting &&
                   Antenna.IsWorking;
        }
    }

    public class MessageQueue
    {
        private static MessageQueue _instance;
        private static readonly object _instanceLock = new object();

        public static MessageQueue Instance
        {
            get
            {
                if (_instance == null)
                {
                    lock (_instanceLock)
                    {
                        if (_instance == null)
                        {
                            _instance = new MessageQueue();
                        }
                    }
                }
                return _instance;
            }
        }

        [Serializable, ProtoContract]
        private class TimestampedMessage
        {
            [ProtoMember(1)]
            public byte[] Data;
            [ProtoMember(2)]
            public DateTime Timestamp;
            [ProtoMember(3)]
            public long SenderId;
            [ProtoMember(4)]
            public ushort MessageId;
            [ProtoMember(5)]
            public MessageSerializationMode SerializationMode;
            [ProtoMember(6)]
            public bool RequiresAck;
            [ProtoMember(7)]
            public Vector3D SenderPosition;
            [ProtoMember(8)]
            public double SenderTransmitRange;
            [ProtoMember(9)]
            public long SenderOwnerId;
        }

        private readonly MessageSerializationMode serializationMode = ServerConfig.DroneNetwork.MessageSerializationMode;

        // Antenna cache - updated when entities register/update
        private readonly MyConcurrentDictionary<long, AntennaCache> _antennaCache;
        private readonly TimeSpan _antennaCacheTimeout = TimeSpan.FromSeconds(5);

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

        public MessageQueue()
        {
            _messageExpiration = TimeUtil.TickToTimeSpan(ServerConfig.DroneNetwork.MessageRetentionTicks);

            _antennaCache = new MyConcurrentDictionary<long, AntennaCache>();
            _topicQueues = new MyConcurrentDictionary<ushort, MyConcurrentQueue<TimestampedMessage>>();
            _topicSubscribers = new MyConcurrentDictionary<ushort, MyConcurrentHashSet<long>>();
            _lastReadTimes = new MyConcurrentDictionary<long, DateTime>();
            _readThrottleIntervals = new MyConcurrentDictionary<long, TimeSpan>();
            _messageAcknowledgments = new MyConcurrentDictionary<ushort, MyConcurrentHashSet<long>>();
            _pendingAcknowledgments = new MyConcurrentDictionary<long, MyConcurrentHashSet<ushort>>();
        }

        /// <summary>
        /// Register or update antenna for an entity
        /// Call this when initializing or when antenna might have changed
        /// </summary>
        public bool RegisterAntenna(long entityId, IMyRadioAntenna antenna)
        {
            if (antenna == null || !antenna.IsFunctional)
            {
                Log.Warning("Cannot register entity {0} - invalid antenna", entityId);
                return false;
            }

            var antennaBlock = antenna as IMyTerminalBlock;
            if (antennaBlock == null)
            {
                Log.Warning("Cannot register entity {0} - antenna is not a terminal block", entityId);
                return false;
            }

            var cache = new AntennaCache
            {
                Antenna = antenna,
                Position = antenna.GetPosition(),
                TransmitRange = antenna.Radius,
                OwnerId = antennaBlock.OwnerId,
                LastUpdate = DateTime.UtcNow
            };

            _antennaCache[entityId] = cache;
            Log.Verbose("Entity {0} registered antenna with range {1}m", entityId, cache.TransmitRange);
            return true;
        }

        /// <summary>
        /// Update cached position and range for an entity
        /// Call periodically (every few seconds) to keep cache fresh
        /// </summary>
        public void UpdateAntennaCache(long entityId)
        {
            AntennaCache cache;
            if (!_antennaCache.TryGetValue(entityId, out cache))
                return;

            if (!cache.IsValid())
            {
                _antennaCache.TryRemove(entityId, out cache);
                Log.Warning("Entity {0} antenna no longer valid, removed from cache", entityId);
                return;
            }

            cache.Position = cache.Antenna.GetPosition();
            cache.TransmitRange = cache.Antenna.Radius;
            cache.LastUpdate = DateTime.UtcNow;
        }

        /// <summary>
        /// Check if two entities can share information based on ownership
        /// </summary>
        private bool CanShare(long senderOwnerId, long receiverOwnerId, IMyTerminalBlock receiverBlock)
        {
            if (senderOwnerId == receiverOwnerId) return true;

            if (receiverBlock != null)
            {
                var relation = receiverBlock.GetUserRelationToOwner(senderOwnerId);
                return relation == MyRelationsBetweenPlayerAndBlock.Owner ||
                       relation == MyRelationsBetweenPlayerAndBlock.FactionShare ||
                       relation == MyRelationsBetweenPlayerAndBlock.Friends;
            }

            return false;
        }

        /// <summary>
        /// Send message with antenna range validation
        /// </summary>
        public ushort SendMessage<T>(ushort topic, T message, long senderId, bool requiresAck = false)
            where T : class
        {
            if (message == null) return 0;

            // Get sender's cached antenna info
            AntennaCache senderCache;
            if (!_antennaCache.TryGetValue(senderId, out senderCache))
            {
                throw new InvalidOperationException($"Entity {senderId} not registered with message queue");
            }

            if (!senderCache.IsValid())
            {
                throw new InvalidOperationException($"Entity {senderId} antenna is not functional");
            }

            // Check if there are any subscribers
            MyConcurrentHashSet<long> subscribers;
            if (!_topicSubscribers.TryGetValue(topic, out subscribers) || subscribers.Count == 0)
            {
                throw new InvalidOperationException($"No subscribers listening to topic {topic}");
            }

            try
            {
                var serializedData = SerializeMessage(message);
                if (serializedData == null) return 0;

                var queue = _topicQueues.GetOrAdd(topic, _ => new MyConcurrentQueue<TimestampedMessage>());
                var messageId = unchecked((ushort)System.Threading.Interlocked.Increment(ref _messageCounter));

                var timestampedMessage = new TimestampedMessage
                {
                    Data = serializedData,
                    Timestamp = DateTime.UtcNow,
                    SenderId = senderId,
                    MessageId = messageId,
                    RequiresAck = requiresAck,
                    SerializationMode = serializationMode,
                    SenderPosition = senderCache.Position,
                    SenderTransmitRange = senderCache.TransmitRange,
                    SenderOwnerId = senderCache.OwnerId
                };

                queue.Enqueue(timestampedMessage);

                if (requiresAck)
                {
                    var pendingAcks = _pendingAcknowledgments.GetOrAdd(senderId, _ => new MyConcurrentHashSet<ushort>());
                    pendingAcks.Add(messageId);
                }

                TryPerformCleanup();
                return messageId;
            }
            catch (Exception ex)
            {
                Log.Error("SendMessage<T> error: {0}", ex);
                return 0;
            }
        }

        /// <summary>
        /// Read messages with antenna range validation (IGC-like behavior)
        /// </summary>
        public List<T> ReadMessages<T>(long subscriberId, ushort topic, ushort maxMessages = 10)
            where T : class, new()
        {
            var messages = new List<T>();

            try
            {
                MyConcurrentHashSet<long> subscribers;
                if (!_topicSubscribers.TryGetValue(topic, out subscribers) || !subscribers.Contains(subscriberId))
                {
                    return messages;
                }

                if (!CanReadNow(subscriberId))
                {
                    return messages;
                }

                // Get receiver's cached antenna info
                AntennaCache receiverCache;
                if (!_antennaCache.TryGetValue(subscriberId, out receiverCache))
                {
                    Log.Warning("Entity {0} not registered, cannot receive messages", subscriberId);
                    return messages;
                }

                if (!receiverCache.IsValid())
                {
                    Log.Warning("Entity {0} antenna not functional, cannot receive messages", subscriberId);
                    return messages;
                }

                var receiverBlock = receiverCache.Antenna as IMyTerminalBlock;

                DateTime oldTime;
                _lastReadTimes.TryRemove(subscriberId, out oldTime);
                _lastReadTimes.TryAdd(subscriberId, DateTime.UtcNow);

                MyConcurrentQueue<TimestampedMessage> queue;
                if (_topicQueues.TryGetValue(topic, out queue))
                {
                    var currentTime = DateTime.UtcNow;
                    var tempMessages = new List<TimestampedMessage>();

                    // Collect all messages
                    TimestampedMessage msg;
                    while (queue.TryDequeue(out msg))
                    {
                        tempMessages.Add(msg);
                    }

                    // Process messages with range and ownership checking
                    foreach (var message in tempMessages)
                    {
                        bool messageExpired = (currentTime - message.Timestamp) > _messageExpiration;

                        if (!messageExpired)
                        {
                            // IGC behavior: receiver can receive if sender's transmit range reaches them
                            double distance = Vector3D.Distance(message.SenderPosition, receiverCache.Position);
                            bool inRange = distance <= message.SenderTransmitRange;

                            // Check ownership/sharing
                            bool canShareInfo = CanShare(message.SenderOwnerId, receiverCache.OwnerId, receiverBlock);

                            bool canReceive = inRange && canShareInfo;

                            if (canReceive && messages.Count < maxMessages)
                            {
                                var deserializedMsg = DeserializeMessage<T>(message.Data);
                                if (deserializedMsg != null)
                                {
                                    messages.Add(deserializedMsg);

                                    if (message.RequiresAck)
                                    {
                                        AckMessage(subscriberId, message.MessageId);
                                    }
                                }
                            }
                            else if (!canReceive)
                            {
                                // Re-queue for other potential receivers
                                queue.Enqueue(message);
                            }
                        }
                        // Expired messages are dropped
                    }
                }
            }
            catch (Exception ex)
            {
                Log.Error("ReadMessages<T> error: {0}", ex);
            }

            return messages;
        }

        private byte[] SerializeMessage<T>(T message) where T : class
        {
            if (message == null) return null;

            try
            {
                if (serializationMode == MessageSerializationMode.ProtoBuf)
                    return MyAPIGateway.Utilities.SerializeToBinary(message);
                else
                    return Encoding.UTF8.GetBytes(MyAPIGateway.Utilities.SerializeToXML(message));
            }
            catch (Exception ex)
            {
                Log.Error("Serialization failed: {0}", ex);
                return null;
            }
        }

        private T DeserializeMessage<T>(byte[] data) where T : class, new()
        {
            if (data == null || data.Length == 0) return null;

            try
            {
                if (serializationMode == MessageSerializationMode.ProtoBuf)
                    return MyAPIGateway.Utilities.SerializeFromBinary<T>(data);
                else
                    return MyAPIGateway.Utilities.SerializeFromXML<T>(Encoding.UTF8.GetString(data));
            }
            catch (Exception ex)
            {
                Log.Error("Deserialization failed: {0}", ex);
                return null;
            }
        }

        public void Subscribe(long subscriberId, ushort topic)
        {
            var subscribers = _topicSubscribers.GetOrAdd(topic, _ => new MyConcurrentHashSet<long>());
            subscribers.Add(subscriberId);
            _readThrottleIntervals.TryAdd(subscriberId, TimeSpan.FromMilliseconds(100));
        }

        public bool AckMessage(long subscriberId, ushort messageId)
        {
            var ackSet = _messageAcknowledgments.GetOrAdd(messageId, _ => new MyConcurrentHashSet<long>());
            return ackSet.Add(subscriberId);
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
                    return;

                _lastCleanup = now;
                PerformCleanup();
                CleanupStaleAntennas();
            }
        }

        private void CleanupStaleAntennas()
        {
            var staleEntities = new List<long>();
            var now = DateTime.UtcNow;

            foreach (var kvp in _antennaCache)
            {
                if (now - kvp.Value.LastUpdate > _antennaCacheTimeout || !kvp.Value.IsValid())
                {
                    staleEntities.Add(kvp.Key);
                }
            }

            foreach (var entityId in staleEntities)
            {
                AntennaCache removed;
                _antennaCache.TryRemove(entityId, out removed);
                Log.Verbose("Removed stale antenna cache for entity {0}", entityId);
            }
        }

        private void PerformCleanup()
        {
            var currentTime = DateTime.UtcNow;

            foreach (var kvp in _topicQueues)
            {
                var queue = kvp.Value;
                var newQueue = new MyConcurrentQueue<TimestampedMessage>();

                TimestampedMessage message;
                while (queue.TryDequeue(out message))
                {
                    if (currentTime - message.Timestamp <= _messageExpiration)
                    {
                        newQueue.Enqueue(message);
                    }
                }

                MyConcurrentQueue<TimestampedMessage> oldQueue;
                _topicQueues.TryRemove(kvp.Key, out oldQueue);
                if (newQueue.Count > 0)
                {
                    _topicQueues.TryAdd(kvp.Key, newQueue);
                }
            }
        }

        public static void Reset()
        {
            _instance = null;
        }
    }
}