using Sandbox.Common.ObjectBuilders;
using Sandbox.Common.ObjectBuilders.Definitions;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRage.Game.Components;

namespace ImprovedAI
{
    [MyEntityComponentDescriptor(typeof(MyObjectBuilder_ProgrammableBlockDefinition), false, "ImprovedAILargeLogisticsComputer")]
    public class IAILogisticsComputerBlock : MyGameLogicComponent
    {
        public IAILogisticsComputerSettings settings { get; set; }
    }
}
