using Sandbox.Common.ObjectBuilders;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRage.Game.Components;

namespace ImprovedAI.Data.Scripts.ImprovedAI
{

    [MyEntityComponentDescriptor(typeof(MyObjectBuilder_TimerBlock), false, "ImprovedAILargeLogisticsComputer")]
    public class IAILogisticsComputerBlock : MyGameLogicComponent
    {
    }
}
