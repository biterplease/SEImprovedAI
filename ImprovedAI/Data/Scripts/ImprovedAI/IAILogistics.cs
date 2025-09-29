using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ImprovedAI
{
    public enum LogisticsOperationMode
    {
        None = 0,
        Provider = 1,
        Requester = 2
    }
    /// <summary>
    /// Request or provide items to a scheduler network.
    /// </summary>
    internal class IAILogistics
    {
        public bool IsEnabled = false;
        public Dictionary<string, int> AvailableItems;
        public Dictionary<string, int> RequestedItems;
    }
}
