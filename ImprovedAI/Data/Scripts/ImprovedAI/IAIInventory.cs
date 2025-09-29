using ProtoBuf;
using System;
using System.Collections.Generic;
using VRage.Collections;
using VRage.Game.ModAPI;

namespace ImprovedAI
{
    [ProtoContract]
    public struct KVPair
    {
        [ProtoMember(1)]
        public string Key;
        [ProtoMember(2)]
        public int Value;
    }

    [Serializable]
    [ProtoContract]
    public class IAIInventory
    {
        [ProtoMember(1)]
        private MyConcurrentList<KVPair> inventory = new MyConcurrentList<KVPair>();

        private readonly object _lock = new object();

        // Parameterless constructor required for ProtoBuf
        public IAIInventory()
        {
            inventory = new MyConcurrentList<KVPair>();
        }

        public IAIInventory(Dictionary<string,int> inventory)
        {
            this.inventory = new MyConcurrentList<KVPair>();
            foreach (var pair in inventory)
            {
                this.inventory.Add(new KVPair { Key = pair.Key, Value = pair.Value });
            }
        }
        public IAIInventory(List<KVPair> inventory)
        {
            this.inventory = new MyConcurrentList<KVPair>();
            foreach (var pair in inventory)
            {
                this.inventory.Add(new KVPair { Key = pair.Key, Value = pair.Value});
            }
        }

        /// <summary>
        /// Get the count of a specific item (thread-safe)
        /// </summary>
        public int GetItemCount(string subtypeId)
        {
            lock (_lock)
            {
                foreach (var pair in inventory)
                {
                    if (pair.Key == subtypeId)
                        return pair.Value;
                }
                return 0;
            }
        }

        /// <summary>
        /// Add a single item - if it exists, add to the count; if not, create new entry
        /// </summary>
        public void AddItem(string subtypeId, int amount)
        {
            if (amount <= 0) return;

            lock (_lock)
            {
                // Find existing item
                for (int i = 0; i < inventory.Count; i++)
                {
                    var pair = inventory[i];
                    if (pair.Key == subtypeId)
                    {
                        // Update existing item count
                        inventory[i] = new KVPair { Key = subtypeId, Value = pair.Value + amount };
                        return;
                    }
                }

                // Item doesn't exist, add new entry
                inventory.Add(new KVPair { Key = subtypeId, Value = amount });
            }
        }

        /// <summary>
        /// Add multiple items from a list - if items exist, add to their counts
        /// </summary>
        public void AddItems(List<KVPair> items)
        {
            if (items == null || items.Count == 0) return;

            lock (_lock)
            {
                foreach (var item in items)
                {
                    if (item.Value <= 0) continue;

                    // Find existing item
                    bool found = false;
                    for (int i = 0; i < inventory.Count; i++)
                    {
                        var pair = inventory[i];
                        if (pair.Key == item.Key)
                        {
                            // Update existing item count
                            inventory[i] = new KVPair { Key = item.Key, Value = pair.Value + item.Value };
                            found = true;
                            break;
                        }
                    }

                    // Item doesn't exist, add new entry
                    if (!found)
                    {
                        inventory.Add(new KVPair { Key = item.Key, Value = item.Value });
                    }
                }
            }
        }

        /// <summary>
        /// Add items from an IMyInventory - extracts all items and their amounts
        /// </summary>
        public void AddItems(IMyInventory sourceInventory)
        {
            if (sourceInventory == null) return;

            lock (_lock)
            {
                var items = new List<VRage.Game.ModAPI.Ingame.MyInventoryItem>();
                sourceInventory.GetItems(items);

                foreach (var item in items)
                {
                    var subtypeId = item.Type.SubtypeId;
                    var amount = (int)item.Amount; // Convert MyFixedPoint to int

                    if (amount <= 0) continue;

                    // Find existing item
                    bool found = false;
                    for (int i = 0; i < inventory.Count; i++)
                    {
                        var pair = inventory[i];
                        if (pair.Key == subtypeId)
                        {
                            // Update existing item count
                            inventory[i] = new KVPair { Key = subtypeId, Value = pair.Value + amount };
                            found = true;
                            break;
                        }
                    }

                    // Item doesn't exist, add new entry
                    if (!found)
                    {
                        inventory.Add(new KVPair { Key = subtypeId, Value = amount });
                    }
                }
            }
        }

        /// <summary>
        /// Check if an item exists in the inventory (thread-safe)
        /// </summary>
        public bool Exists(string subtypeId)
        {
            lock (_lock)
            {
                foreach (var pair in inventory)
                {
                    if (pair.Key == subtypeId)
                        return true;
                }
                return false;
            }
        }

        /// <summary>
        /// Remove a specific amount of an item
        /// </summary>
        public bool RemoveItem(string subtypeId, int amount)
        {
            if (amount <= 0) return false;

            lock (_lock)
            {
                for (int i = 0; i < inventory.Count; i++)
                {
                    var pair = inventory[i];
                    if (pair.Key == subtypeId)
                    {
                        if (pair.Value <= amount)
                        {
                            // Remove entire entry
                            inventory.RemoveAt(i);
                        }
                        else
                        {
                            // Reduce count
                            inventory[i] = new KVPair { Key = subtypeId, Value = pair.Value - amount };
                        }
                        return true;
                    }
                }
                return false; // Item not found
            }
        }

        /// <summary>
        /// Get all items as a list (thread-safe copy)
        /// </summary>
        public List<KVPair> GetAllItems()
        {
            lock (_lock)
            {
                var result = new List<KVPair>();
                foreach (var pair in inventory)
                {
                    result.Add(pair);
                }
                return result;
            }
        }

        /// <summary>
        /// Get total number of different item types
        /// </summary>
        public int GetItemTypeCount()
        {
            lock (_lock)
            {
                return inventory.Count;
            }
        }

        /// <summary>
        /// Get total quantity of all items
        /// </summary>
        public int GetTotalItemCount()
        {
            lock (_lock)
            {
                int total = 0;
                foreach (var pair in inventory)
                {
                    total += pair.Value;
                }
                return total;
            }
        }

        /// <summary>
        /// Clear all items from inventory
        /// </summary>
        public void Clear()
        {
            lock (_lock)
            {
                inventory.Clear();
            }
        }

        /// <summary>
        /// Check if inventory is empty
        /// </summary>
        public bool IsEmpty()
        {
            lock (_lock)
            {
                return inventory.Count == 0;
            }
        }

        /// <summary>
        /// Get a summary string of the inventory contents
        /// </summary>
        public override string ToString()
        {
            lock (_lock)
            {
                if (inventory.Count == 0)
                    return "Empty inventory";

                var items = new List<string>();
                foreach (var pair in inventory)
                {
                    items.Add($"{pair.Key}: {pair.Value}");
                }
                return string.Join(", ", items);
            }
        }
    }
}