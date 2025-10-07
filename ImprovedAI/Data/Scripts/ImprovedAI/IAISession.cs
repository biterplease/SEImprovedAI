﻿using ImprovedAI.Config;
using ImprovedAI.Network;
using ImprovedAI.Utils.Logging;
using Sandbox.Game.World;
using Sandbox.ModAPI;
using Sandbox.ModAPI.Interfaces;
using Sandbox.ModAPI.Interfaces.Terminal;
using SpaceEngineers.Game.ModAPI;
using System;
using System.Collections.Generic;
using System.IO;
using VRage;
using VRage.Game.Components;
using VRage.Utils;

namespace ImprovedAI
{
    [MySessionComponentDescriptor(MyUpdateOrder.BeforeSimulation)]
    public class IAISession : MySessionComponentBase
    {
        public static IAISession Instance;
        private static ServerConfig _serverConfig;

        // Shared message queue for all AI components
        public MessageQueue MessageQueue { get; private set; }

        // Mod-wide settings and management
        private bool isInitialized = false;
        private long _lastUpdateFrame = 0;
        private int _updateInterval; // DON'T initialize here - will be set in Init()
        private int updateCounter = 0;
        private const int UPDATE_INTERVAL = 60;
        public static Guid ModGuid = new Guid("1CFDA990-FD26-4950-A127-7BBC99FF1397");
        public const string ModName = "ImprovedAI";

        // Collection of all AI blocks in the world
        public Dictionary<long, IAIDroneControllerBlock> AIDroneControllers = new Dictionary<long, IAIDroneControllerBlock>();
        public Dictionary<long, IAISchedulerBlock> AIDroneSchedulers = new Dictionary<long, IAISchedulerBlock>();
        public Dictionary<long, IAILogisticsComputerBlock> AILogisticsComputers = new Dictionary<long, IAILogisticsComputerBlock>();

        /// <summary>
        /// All components: steel plate, metal grid, etc.
        /// For use in logistic computer list filters.
        /// </summary>
        public Dictionary<long, string> AllComponents;

        /// <summary>
        /// All components: steel plate, metal grid, etc.
        /// For use in scheduler grind and weld list filters.
        /// </summary>
        public Dictionary<long, string> AllBlocks;

        public ServerConfig GetConfig()
        {
            return _serverConfig;
        }

        public override void LoadData()
        {
            Instance = this;
            // Localization settings
            LoadLangOverrides();
            MyAPIGateway.Gui.GuiControlRemoved += GuiControlRemoved;
        }
        public IAISession()
        {
            try
            {
                MyLog.Default.WriteLineAndConsole("ImprovedAI: Instance constructor called");
            }
            catch (Exception ex)
            {
                MyLog.Default.WriteLineAndConsole($"ImprovedAI: Instance constructor failed: {ex}");
            }
        }

        private void Init()
        {
            try
            {
                Log.Initialize(ServerConfig.MOD_NAME, 0, "ImprovedAI.log", typeof(IAISession));
                Log.Info("=== ImprovedAI Initializing ===");


                MessageQueue = MessageQueue.Instance;
                _serverConfig = ServerConfig.Instance;
                _serverConfig.LoadConfig();
                Log.Info("Log level: {0}", _serverConfig.Logging.LogLevel.ToString());

                _updateInterval = _serverConfig.Session.UpdateInterval;

                Log.Info("Mod loaded successfully");
                Log.Info("Update interval: {0} ticks", _updateInterval);
                IAILogisticsComputerTerminalControls.DoOnce(ModContext);

                //LogAllTerminalControlClasses();


                isInitialized = true;
            }
            catch (Exception ex)
            {
                MyLog.Default.WriteLine($"ImprovedAI: Init exception: {ex}");
            }
        }

        //static void LogAllTerminalControlClasses()
        //{

        //    List<IMyTerminalControl> broadcastControllerControls;
        //    MyAPIGateway.TerminalControls.GetControls<IMyBroadcastController>(out broadcastControllerControls);
        //    MyLog.Default.WriteLine($"");
        //    MyLog.Default.WriteLine($"");

        //    MyLog.Default.WriteLine($"[DEV] BROADCAST CONTROLLER TERMINAL CONTROS:");
        //    MyLog.Default.WriteLine($"");
        //    MyLog.Default.WriteLine($"");
        //    foreach (IMyTerminalControl c in broadcastControllerControls)
        //    {
        //        // a quick way to dump all IDs to SE's log
        //        string name = MyTexts.GetString((c as IMyTerminalControlTitleTooltip)?.Title.String ?? "N/A");
        //        string valueType = (c as ITerminalProperty)?.TypeName ?? "N/A";
        //        MyLog.Default.WriteLine($"[DEV] terminal property: id='{c.Id}'; type='{c.GetType().Name}'; valueType='{valueType}'; displayName='{name}'");
        //    }

        //    List<IMyTerminalAction> bcActions;
        //    MyAPIGateway.TerminalControls.GetActions<IMyBroadcastController>(out bcActions);
        //    foreach (IMyTerminalAction a in bcActions)
        //    {
        //        MyLog.Default.WriteLine($"[DEV] toolbar action: id='{a.Id}'; displayName='{a.Name}'");
        //    }

        //        MyLog.Default.WriteLine($"");
        //    MyLog.Default.WriteLine($"");
        //    MyLog.Default.WriteLine($"");
        //    MyLog.Default.WriteLine($"[DEV] PROGRAMMABLE BLOCK CONTROLS:");
        //    MyLog.Default.WriteLine($"");
        //    MyLog.Default.WriteLine($"");
        //    List<IMyTerminalControl> pbControls;
        //    MyAPIGateway.TerminalControls.GetControls<IMyProgrammableBlock>(out pbControls);

        //    foreach (IMyTerminalControl c in pbControls)
        //    {
        //        // a quick way to dump all IDs to SE's log
        //        string name = MyTexts.GetString((c as IMyTerminalControlTitleTooltip)?.Title.String ?? "N/A");
        //        string valueType = (c as ITerminalProperty)?.TypeName ?? "N/A";
        //        MyLog.Default.WriteLine($"[DEV] terminal property: id='{c.Id}'; type='{c.GetType().Name}'; valueType='{valueType}'; displayName='{name}'");
        //    }
        //    List<IMyTerminalAction> pbActions;
        //    MyAPIGateway.TerminalControls.GetActions<IMyProgrammableBlock>(out pbActions);
        //    foreach (IMyTerminalAction a in pbActions)
        //    {
        //        MyLog.Default.WriteLine($"[DEV] toolbar action: id='{a.Id}'; displayName='{a.Name}'");
        //    }

        //    MyLog.Default.WriteLine($"");
        //    MyLog.Default.WriteLine($"");
        //    MyLog.Default.WriteLine($"");
        //    MyLog.Default.WriteLine($"[DEV] REMOTE CONTROL BLOCK CONTROLS:");
        //    MyLog.Default.WriteLine($"");
        //    MyLog.Default.WriteLine($"");
        //    List<IMyTerminalControl> rcControls;
        //    MyAPIGateway.TerminalControls.GetControls<IMyRemoteControl>(out rcControls);

        //    foreach (IMyTerminalControl c in rcControls)
        //    {
        //        // a quick way to dump all IDs to SE's log
        //        string name = MyTexts.GetString((c as IMyTerminalControlTitleTooltip)?.Title.String ?? "N/A");
        //        string valueType = (c as ITerminalProperty)?.TypeName ?? "N/A";
        //        MyLog.Default.WriteLine($"[DEV] terminal property: id='{c.Id}'; type='{c.GetType().Name}'; valueType='{valueType}'; displayName='{name}'");
        //    }
        //    List<IMyTerminalAction> rcActions;
        //    MyAPIGateway.TerminalControls.GetActions<IMyRemoteControl>(out rcActions);
        //    foreach (IMyTerminalAction a in rcActions)
        //    {
        //        MyLog.Default.WriteLine($"[DEV] toolbar action: id='{a.Id}'; displayName='{a.Name}'");
        //    }
        //    MyLog.Default.WriteLine($"");
        //    MyLog.Default.WriteLine($"");
        //    MyLog.Default.WriteLine($"");
        //    MyLog.Default.WriteLine($"");
        //}

        public override void UpdateBeforeSimulation()
        {
            try
            {
                if (!isInitialized)
                {
                    if (MyAPIGateway.Session == null)
                        return;
                    Log.Info("Initializing IAI Session");
                    Init();
                    return;
                }

                var currentFrame = MyAPIGateway.Session.GameplayFrameCounter;
                if (currentFrame - _lastUpdateFrame < _updateInterval)
                    return;

                _lastUpdateFrame = currentFrame;

                // Periodic cleanup and maintenance
                CleanupInvalidBlocks();

                // Optional: Log active AI block counts
                updateCounter++;
                if (updateCounter % (UPDATE_INTERVAL * 10) == 0) // Every 10 seconds
                {
                    Log.Verbose("Active AI Drone Schedulers: {0}", AIDroneSchedulers.Count);
                    Log.Verbose("Active AI Drone Controllers: {0}", AIDroneControllers.Count);
                    Log.Verbose("Active AI Logistics Computers: {0}", AILogisticsComputers.Count);
                }
            }
            catch (Exception ex)
            {
                MyAPIGateway.Utilities.ShowMessage("ImprovedAI", $"Update error: {ex.Message}");
                MyLog.Default.WriteLine($"ImprovedAI: UpdateBeforeSimulation exception: {ex}");
            }
        }

        private void CleanupInvalidBlocks()
        {
            var schedulersToRemove = new List<long>();
            foreach (var kvp in AIDroneSchedulers)
            {
                if (kvp.Value?.Entity == null || kvp.Value.Entity.MarkedForClose)
                {
                    schedulersToRemove.Add(kvp.Key);
                }
            }

            foreach (var entityId in schedulersToRemove)
            {
                AIDroneSchedulers.Remove(entityId);
            }

            var controllersToRemove = new List<long>();
            foreach (var kvp in AIDroneControllers)
            {
                if (kvp.Value?.Entity == null || kvp.Value.Entity.MarkedForClose)
                {
                    controllersToRemove.Add(kvp.Key);
                }
            }

            foreach (var entityId in controllersToRemove)
            {
                AIDroneControllers.Remove(entityId);
            }

            var logisticsToRemove = new List<long>();
            foreach (var kvp in AILogisticsComputers)
            {
                if (kvp.Value?.Entity == null || kvp.Value.Entity.MarkedForClose)
                {
                    logisticsToRemove.Add(kvp.Key);
                }
            }

            foreach (var entityId in logisticsToRemove)
            {
                AILogisticsComputers.Remove(entityId);
            }
        }

        public void RegisterScheduler(IAISchedulerBlock block)
        {
            if (block?.Entity != null)
            {
                AIDroneSchedulers[block.Entity.EntityId] = block;
                Log.Info("Registered AI Drone Scheduler: {0} {1}", block.Entity.EntityId, block.Entity.DisplayName);
            }
        }

        public void UnregisterScheduler(long entityId)
        {
            if (AIDroneSchedulers.Remove(entityId))
            {
                Log.Info("Unregistered AI Scheduler: {0}", entityId);
            }
        }

        public void RegisterDroneController(IAIDroneControllerBlock block)
        {

            if (block?.Entity != null)
            {
                AIDroneControllers[block.Entity.EntityId] = block;
                Log.Info("Registered AI Drone controller: {0} {1}", block.Entity.EntityId, block.Entity.DisplayName);
            }
        }

        public void UnregisterDroneController(long entityId)
        {
            if (AIDroneControllers.Remove(entityId))
            {
                Log.Info("Unregistered AI Drone controller: {0}", entityId);
            }
        }

        public void RegisterLogisticsComputer(IAILogisticsComputerBlock block)
        {
            if (block?.Entity != null)
            {
                AILogisticsComputers[block.Entity.EntityId] = block;
                Log.Info("Registered AI Logistics Computer: {0} {1}", block.Entity.EntityId, block.Entity.DisplayName);
            }
        }

        public void UnregisterLogisticsComputer(long entityId)
        {
            if (AILogisticsComputers.Remove(entityId))
            {
                Log.Info("Unregistered AI Logistics Computer: {0}", entityId);
            }
        }

        protected override void UnloadData()
        {
            try
            {
                // Clean shutdown
                AIDroneControllers.Clear();
                AIDroneSchedulers.Clear();
                AILogisticsComputers.Clear();

                // Reset message queue singleton
                if (MessageQueue != null)
                {
                    MessageQueue.Reset();
                }

                // Remove localization texts
                MyAPIGateway.Gui.GuiControlRemoved -= GuiControlRemoved;

                Instance = null;

                if (Log.IsInitialized)
                {
                    Log.Info("Mod unloaded");
                    Log.Close();
                }
            }
            catch (Exception ex)
            {
                MyLog.Default.WriteLine($"ImprovedAI: UnloadData exception: {ex}");

                var writer = MyAPIGateway.Utilities.WriteFileInLocalStorage("shutdown_error.log", typeof(IAISession));
                if (writer != null)
                {
                    writer.Write(ex.ToString());
                    writer.Flush();
                    writer.Close();
                }
            }
        }

        void LoadLangOverrides()
        {
            try
            {
                string folder = Path.Combine(ModContext.ModPathData, "Localization");
                MyTexts.LoadTexts(folder, cultureName: "override", subcultureName: null);
            }
            catch
            {
                // ModContext might not be ready yet, will try again in Init
            }
        }

        void GuiControlRemoved(object screen)
        {
            if (screen == null)
                return;

            // detect when options menu is closed in case player changes language
            if (screen.ToString().EndsWith("ScreenOptionsSpace"))
            {
                LoadLangOverrides();
            }
        }
    }
}