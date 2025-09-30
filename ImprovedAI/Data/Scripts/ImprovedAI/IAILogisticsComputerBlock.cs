using ImprovedAI.Config;
using ImprovedAI.Utils;
using Sandbox.Common.ObjectBuilders;
using Sandbox.Common.ObjectBuilders.Definitions;
using Sandbox.ModAPI;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRage.Game.Components;

using static ImprovedAI.LogisticsComputer;

namespace ImprovedAI
{
    [MyEntityComponentDescriptor(typeof(MyObjectBuilder_ProgrammableBlockDefinition), false, "ImprovedAILargeLogisticsComputer")]
    public class IAILogisticsComputerBlock : MyGameLogicComponent
    {
        IMyProgrammableBlock programmableBlock;
        public IAILogisticsComputerSettings settings { get; set; }

        public static ServerConfig.LogisticsComputerConfig BlockConfig = ServerConfig.Instance.LogisticsComputer;
        public bool Terminal_IsEnabled
        {
            get
            {
                return settings?.Enabled ?? false;
            }
            set
            {
                if (settings != null) settings.Enabled = value;
            }
        }
        public bool Terminal_OperationModeProvideForLogistics
        {
            get
            {
                return settings?.OperationMode.HasFlag(OperationMode.ProvideForLogistics) ?? false;
            }
            set
            {
                if (settings != null)
                {
                    if (value)
                        settings.OperationMode |= OperationMode.ProvideForLogistics;
                    else
                        settings.OperationMode &= ~OperationMode.ProvideForLogistics;
                }
            }
        }
        public bool Terminal_OperationModeProvideForConstruction
        {
            get
            {
                return settings?.OperationMode.HasFlag(OperationMode.ProvideForConstruction) ?? false;
            }
            set
            {
                if (settings != null)
                {
                    if (value)
                        settings.OperationMode |= OperationMode.ProvideForConstruction;
                    else
                        settings.OperationMode &= ~OperationMode.ProvideForConstruction;
                }
            }
        }
        public bool Terminal_OperationModePush
        {
            get
            {
                return settings?.OperationMode.HasFlag(OperationMode.Push) ?? false;
            }
            set
            {
                if (settings != null)
                {
                    if (value)
                        settings.OperationMode |= OperationMode.Push;
                    else
                        settings.OperationMode &= ~OperationMode.Push;
                }
            }
        }
        public bool Terminal_OperationModeRequest
        {
            get
            {
                return settings?.OperationMode.HasFlag(OperationMode.Request) ?? false;
            }
            set
            {
                if (settings != null)
                {
                    if (value)
                        settings.OperationMode |= OperationMode.Request;
                    else
                        settings.OperationMode &= ~OperationMode.Request;
                }
            }
        }
        public bool Terminal_PushAll
        {
            get
            {
                return settings?.PushAllInventory ?? false;
            }
            set
            {
                if (settings != null) settings.PushAllInventory = value;
            }
        }
        public float Terminal_PushFrequencySeconds
        {
            get { return settings?.PushFrequencySeconds ?? TimeUtil.TickToSeconds(BlockConfig.MinPushFrequencyTicks); }
            set
            {
                if (settings != null) settings.PushFrequencySeconds = value;
            }
        }
        public bool Terminal_SetRequests
        {
            get
            {
                return settings?.SetRequests ?? false;
            }
            set
            {
                if (settings != null) settings.SetRequests = value;
            }
        }
        public bool Terminal_UseItemLimits
        {
            get
            {
                return settings?.UseItemLimits ?? false;
            }
            set
            {
                if (settings != null) settings.UseItemLimits = value;
            }
        }
    }
}
