using Sandbox.Game.Entities;
using Sandbox.ModAPI;
using System.Collections.Generic;
using VRage.Game.Entity;
using VRageMath;

namespace ImprovedAI
{
    /// <summary>
    /// Delegate interface for MyAPIGateway.Session operations.
    /// Allows mocking of static game methods during testing
    /// </summary>
    public interface IMySessionDelegate
    {
        int GameplayFrameCounter { get; }
    }

    /// <summary>
    /// Production implementation using real SE API
    /// </summary>
    public class MySessionDelegate : IMySessionDelegate
    {
        public int GameplayFrameCounter
        {
            get
            {
                return MyAPIGateway.Session.GameplayFrameCounter;
            }
        }
    }
    /// <summary>
    /// Delegate interface for MyAPIGateway.Session operations.
    /// Allows mocking of static game methods during testing
    /// </summary>
    public interface IMyUtilitiesDelegate
    {
        byte[] SerializeToBinary<T>(T obj);
        T SerializeFromBinary<T>(byte[] bytes);
        string SerializeToXML<T>(T objectToSerialize);
        T SerializeFromXML<T>(string buffer);
    }

    /// <summary>
    /// Production implementation using real SE API
    /// </summary>
    public class MyUtilitiesDelegate : IMyUtilitiesDelegate
    {
        public byte[] SerializeToBinary<T>(T obj)
        {
            return MyAPIGateway.Utilities.SerializeToBinary(obj);
        }
        public T SerializeFromBinary<T>(byte[] bytes)
        {
            return MyAPIGateway.Utilities.SerializeFromBinary<T>(bytes);
        }
        public string SerializeToXML<T>(T objectToSerialize)
        {
            return MyAPIGateway.Utilities.SerializeToXML<T>(objectToSerialize);
        }
        public T SerializeFromXML<T>(string buffer)
        {
            return MyAPIGateway.Utilities.SerializeFromXML<T>(buffer);
        }
    }
}
