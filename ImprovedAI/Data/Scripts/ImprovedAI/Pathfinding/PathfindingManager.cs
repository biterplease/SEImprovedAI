using ImprovedAI.Config;
using ImprovedAI.Utils.Logging;
using Sandbox.Game.Entities;
using Sandbox.ModAPI;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using VRage.Game.Entity;
using VRage.Game.ModAPI;
using VRage.ModAPI;
using VRageMath;

namespace ImprovedAI.Pathfinding
{
    /// <summary>
    /// Manages pathfinding requests and coordinates between different pathfinding strategies.
    /// Uses dynamic waypoint generation to reduce performance cost.
    /// </summary>
    public class PathfindingManager
    {
        public enum Method
        {
            None = 0,
            DirectPathfinding = 1,
            AStar = 2,
            DStarLite = 4
        }

        private readonly IPathfindingConfig config;
        private readonly IPathfinder directPathfinder;
        private readonly IPathfinder astarPathfinder;
        private readonly IMyGamePruningStructureDelegate pruningStructure;
        private readonly IMyPlanetDelegate planetDelegate;
        private readonly AdjacencyList traveledGraph;
        private Base6Directions.Direction controllerForwardDirection;

        // Reusable buffers to avoid allocations
        private List<MyLineSegmentOverlapResult<MyEntity>> _raycastResults;
        private List<Vector3D> _nearbyNodesBuffer;
        private List<IMyCubeGrid> _connectedGridsBuffer;

        // Cache for temporary calculations
        private Vector3D _tempDirection;
        private Vector3D _tempOffset;

        public PathfindingManager(
            IPathfindingConfig pathfindingConfig,
            Base6Directions.Direction controllerFwdDirection = Base6Directions.Direction.Forward,
            IMyGamePruningStructureDelegate pruningStructure = null,
            IMyPlanetDelegate planetDelegate = null
        )
        {
            config = pathfindingConfig ?? IAISession.Instance?.GetConfig()?.Pathfinding;
            traveledGraph = new AdjacencyList();
            controllerForwardDirection = controllerFwdDirection;
            this.pruningStructure = pruningStructure ?? new MyGamePruningStructureDelegate();
            this.planetDelegate = planetDelegate ?? new MyPlanetDelegate();

            // Initialize reusable buffers
            _raycastResults = new List<MyLineSegmentOverlapResult<MyEntity>>(10);
            _nearbyNodesBuffer = new List<Vector3D>(20);
            _connectedGridsBuffer = new List<IMyCubeGrid>(10);

            // Initialize available pathfinders
            directPathfinder = config?.AllowDirectPathfinding() == true ? new DirectPathfinder() : null;
            astarPathfinder = config?.AllowAStar() == true ? new AStarPathfinder() : null;

            if (directPathfinder == null)
            {
                Log.Error("PathfindingManager: DirectPathfinding is disabled but required! Drones will not function.");
            }
        }

        /// <summary>
        /// Get the next waypoint dynamically without generating the full path.
        /// </summary>
        public bool GetNextWaypoint(ref Vector3D currentPosition, ref Vector3D targetPosition,
            PathfindingContext context, out Vector3D waypoint)
        {
            waypoint = default(Vector3D);

            if (config == null)
            {
                Log.Error("PathfindingManager: Configuration not loaded");
                return false;
            }
            if (!config.AllowDirectPathfinding() && !config.AllowAStar())
                return false;

            context.WaypointDistance = MathHelper.Clamp(
                context.WaypointDistance,
                config.MinWaypointDistance(),
                config.MaxWaypointDistance());

            double distance = Vector3D.Distance(currentPosition, targetPosition);

            // Check if we're close enough to target
            if (distance < context.WaypointDistance)
            {
                waypoint = targetPosition;
                return true;
            }

            // OPTIMIZATION: Check for direct line of sight before expensive pathfinding
            if (distance > 50.0 && HasDirectLineOfSight(ref currentPosition, ref targetPosition, context))
            {
                Log.Info("PathfindingManager: Using direct pathfinding (line of sight confirmed)");
                return GetDirectWaypoint(ref currentPosition, ref targetPosition, context, out waypoint);
            }

            // No direct line of sight, use normal pathfinder selection
            var pathfinder = SelectPathfinder(ref context, ref currentPosition, ref targetPosition);
            if (pathfinder == null)
            {
                Log.Error("PathfindingManager: No pathfinder available");
                return HandleNoPathfinderAvailable(ref currentPosition, ref targetPosition, out waypoint);
            }

            var stopwatch = Stopwatch.StartNew();

            try
            {
                Vector3D waypointResult;
                bool hasWaypoint = pathfinder.GetNextWaypoint(ref context, ref currentPosition, ref targetPosition, out waypointResult);

                if (hasWaypoint)
                {
                    CacheWaypoint(ref currentPosition, ref waypointResult, distance);
                    stopwatch.Stop();
                    LogPathfindingResult(pathfinder.Method, stopwatch.Elapsed, ref waypointResult);
                    waypoint = waypointResult;
                    return true;
                }

                if (config.AllowRepathing())
                {
                    return HandleRepathing(ref currentPosition, ref targetPosition, context,
                        pathfinder, out waypoint);
                }

                Log.Warning("PathfindingManager: Pathfinder failed and repathing is disabled");
                return false;
            }
            catch (Exception ex)
            {
                stopwatch.Stop();
                Log.Error("PathfindingManager: Exception in GetNextWaypoint: {0}", ex.Message);

                if (config.AllowRepathing())
                {
                    return HandleEmergencyPath(ref currentPosition, ref targetPosition, context, out waypoint);
                }

                return false;
            }
        }

        /// <summary>
        /// Generate the complete path for debugging purposes.
        /// WARNING: This is expensive and should not be used during normal operation!
        /// </summary>
        public bool GenerateCompletePath(ref Vector3D start, ref Vector3D end,
            PathfindingContext context, List<Vector3D> pathOutput)
        {
            if (pathOutput == null)
                throw new ArgumentNullException("pathOutput");

            Log.Warning("PathfindingManager: GenerateCompletePath called - this is for debugging only!");

            if (config == null)
            {
                Log.Error("PathfindingManager: Configuration not loaded");
                return CreateEmergencyPath(ref start, ref end, pathOutput);
            }

            context.WaypointDistance = MathHelper.Clamp(
                context.WaypointDistance,
                config.MinWaypointDistance(),
                config.MaxWaypointDistance());

            var pathfinder = SelectPathfinder(ref context, ref start, ref end);
            if (pathfinder == null)
            {
                return CreateEmergencyPath(ref start, ref end, pathOutput);
            }

            var stopwatch = Stopwatch.StartNew();

            try
            {
                pathfinder.CalculatePath(ref context, ref start, ref end, pathOutput);

                if (IsValidPath(pathOutput, ref start, ref end, context))
                {
                    stopwatch.Stop();
                    LogPathfindingResult(pathfinder.Method, stopwatch.Elapsed, pathOutput.Count);
                    return true;
                }

                // Path invalid, try fallback
                if (config.AllowRepathing() && pathfinder != directPathfinder && directPathfinder != null)
                {
                    Log.Warning("PathfindingManager: Primary pathfinder failed, falling back to direct");
                    pathOutput.Clear();
                    directPathfinder.CalculatePath(ref context, ref start, ref end, pathOutput);

                    if (IsValidPath(pathOutput, ref start, ref end, context))
                    {
                        return true;
                    }
                }
            }
            catch (Exception ex)
            {
                Log.Error("PathfindingManager: Exception in GenerateCompletePath: {0}", ex.Message);
            }

            // Check if we're allowed to use emergency path
            if (config.AllowRepathing())
            {
                Log.Warning("PathfindingManager: All pathfinders failed, using emergency path");
                return CreateEmergencyPath(ref start, ref end, pathOutput);
            }

            Log.Error("PathfindingManager: Pathfinding failed and repathing/emergency paths disabled");
            pathOutput.Clear();
            return false;
        }

        /// <summary>
        /// Validate that a path is safe and reaches the destination
        /// </summary>
        private bool IsValidPath(List<Vector3D> path, ref Vector3D start, ref Vector3D end,
            PathfindingContext context)
        {
            if (path == null || path.Count < 1)
            {
                Log.Warning("PathfindingManager: Path is null or empty");
                return false;
            }

            // Check that path starts reasonably close to start position
            Vector3D pathStart = path[0];
            if (Vector3D.DistanceSquared(pathStart, start) > 100) // Within 10m
            {
                Log.Warning("PathfindingManager: Path start is too far from actual start position");
                return false;
            }

            // Check that path ends reasonably close to target
            Vector3D pathEnd = path[path.Count - 1];
            if (Vector3D.DistanceSquared(pathEnd, end) > 100) // Within 10m
            {
                Log.Warning("PathfindingManager: Path end is too far from target position");
                return false;
            }

            // Check waypoint count limits
            if (path.Count > config.MaxPathNodes())
            {
                Log.Warning("PathfindingManager: Path exceeds maximum node count ({0} > {1})",
                    path.Count, config.MaxPathNodes());
                return false;
            }

            // Validate waypoint spacing
            for (int i = 0; i < path.Count - 1; i++)
            {
                double distance = Vector3D.Distance(path[i], path[i + 1]);

                if (distance < config.MinWaypointDistance() * 0.5)
                {
                    Log.Warning("PathfindingManager: Waypoints too close together at index {0}", i);
                    return false;
                }

                if (distance > config.MaxWaypointDistance() * 2.0)
                {
                    Log.Warning("PathfindingManager: Waypoints too far apart at index {0}", i);
                    return false;
                }
            }

            // Optional: Check if path stays at safe altitude in gravity
            if (config.UsePlanetAwarePathfinding() && context.IsInPlanetGravity())
            {
                var altitude = context.GetSurfaceAltitude();
                if (altitude.HasValue && altitude.Value < config.MinAltitudeBuffer())
                {
                    Log.Warning("PathfindingManager: Path goes below safe altitude");
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Select the best pathfinder based on context and configuration
        /// </summary>
        private IPathfinder SelectPathfinder(ref PathfindingContext context, ref Vector3D start, ref Vector3D end)
        {
            double distance = Vector3D.Distance(start, end);

            // For very short distances, always use direct pathfinding
            if (distance < config.MinWaypointDistance() * 2)
            {
                return directPathfinder;
            }

            // Check if A* is available and conditions warrant its use
            if (astarPathfinder != null && astarPathfinder.IsAvailable(ref context))
            {
                if (context.SensorInfos != null && context.SensorInfos.Count > 0 ||
                    context.CamerasByDirection != null && context.CamerasByDirection.Count > 0)
                {
                    int complexity = astarPathfinder.EstimatedComplexity(ref start, ref end);

                    if (complexity < config.MaxPathNodes())
                    {
                        Log.Verbose("PathfindingManager: Selected A* pathfinder (complexity: {0})", complexity);
                        return astarPathfinder;
                    }
                }
            }

            // Default to direct pathfinding
            if (directPathfinder != null && directPathfinder.IsAvailable(ref context))
            {
                Log.Verbose("PathfindingManager: Selected Direct pathfinder");
                return directPathfinder;
            }

            return null;
        }

        /// <summary>
        /// Handle repathing when primary pathfinder fails
        /// </summary>
        private bool HandleRepathing(ref Vector3D currentPosition, ref Vector3D targetPosition,
            PathfindingContext context, IPathfinder failedPathfinder, out Vector3D waypoint)
        {
            waypoint = default(Vector3D);
            Log.Info("PathfindingManager: Attempting repathing from {0}", failedPathfinder.Method);

            // Try to find a previously traveled node that could help
            FindNearbyTraveledNodes(ref currentPosition, config.MaxWaypointDistance() * 2, _nearbyNodesBuffer);

            double distanceDirect = Vector3D.Distance(currentPosition, targetPosition);

            for (int i = 0; i < _nearbyNodesBuffer.Count; i++)
            {
                Vector3D node = _nearbyNodesBuffer[i];
                double distanceFromNode = Vector3D.Distance(node, targetPosition);

                if (distanceFromNode < distanceDirect * 0.9) // 10% improvement threshold
                {
                    Log.Info("PathfindingManager: Found useful cached node for repathing");
                    waypoint = node;
                    return true;
                }
            }

            // No useful cached nodes, try fallback pathfinder
            if (failedPathfinder == astarPathfinder && directPathfinder != null)
            {
                Log.Info("PathfindingManager: Falling back to Direct pathfinder");
                if (GetDirectWaypoint(ref currentPosition, ref targetPosition, context, out waypoint))
                {
                    return true;
                }
            }

            // Last resort: emergency path
            return HandleEmergencyPath(ref currentPosition, ref targetPosition, context, out waypoint);
        }

        /// <summary>
        /// Get waypoint using direct pathfinder
        /// </summary>
        private bool GetDirectWaypoint(ref Vector3D currentPosition, ref Vector3D targetPosition,
            PathfindingContext context, out Vector3D waypoint)
        {
            if (directPathfinder != null)
            {
                return directPathfinder.GetNextWaypoint(
                    ref context,
                    ref currentPosition,
                    ref targetPosition,
                    out waypoint);
            }

            waypoint = default(Vector3D);
            return false;
        }

        /// <summary>
        /// Create an emergency direct path when all else fails
        /// </summary>
        private bool HandleEmergencyPath(ref Vector3D currentPosition, ref Vector3D targetPosition,
            PathfindingContext context, out Vector3D waypoint)
        {
            waypoint = default(Vector3D);

            if (!config.AllowRepathing())
            {
                Log.Error("PathfindingManager: Emergency path needed but repathing is disabled");
                return false;
            }

            Log.Warning("PathfindingManager: Using emergency waypoint generation");

            // Calculate a safe intermediate waypoint
            Vector3D.Subtract(ref targetPosition, ref currentPosition, out _tempDirection);
            Vector3D.Normalize(ref _tempDirection, out _tempDirection);

            double distance = Vector3D.Distance(currentPosition, targetPosition);
            double waypointDistance = Math.Min(distance * 0.5, config.MaxWaypointDistance());

            Vector3D.Multiply(ref _tempDirection, waypointDistance, out _tempOffset);
            Vector3D.Add(ref currentPosition, ref _tempOffset, out waypoint);

            // If in gravity, ensure we maintain safe altitude
            if (context.IsInPlanetGravity() && config.UsePlanetAwarePathfinding())
            {
                var altitude = context.GetSurfaceAltitude();
                if (altitude.HasValue && altitude.Value < config.MinAltitudeBuffer())
                {
                    Vector3D gravityUp;
                    Vector3D.Negate(ref context.GravityVector, out gravityUp);
                    Vector3D.Normalize(ref gravityUp, out gravityUp);

                    double liftAmount = config.MinAltitudeBuffer() - altitude.Value;
                    Vector3D.Multiply(ref gravityUp, liftAmount, out _tempOffset);
                    Vector3D.Add(ref waypoint, ref _tempOffset, out waypoint);
                }
            }

            return true;
        }

        /// <summary>
        /// Handle case where no pathfinder is available
        /// </summary>
        private bool HandleNoPathfinderAvailable(ref Vector3D currentPosition, ref Vector3D targetPosition,
            out Vector3D emergencyWaypoint)
        {
            if (!config.AllowRepathing())
            {
                Log.Error("PathfindingManager: No pathfinder available and repathing disabled");
                emergencyWaypoint = default(Vector3D);
                return false;
            }

            Log.Error("PathfindingManager: No pathfinder available, using basic emergency waypoint");

            Vector3D.Subtract(ref targetPosition, ref currentPosition, out _tempDirection);
            Vector3D.Normalize(ref _tempDirection, out _tempDirection);

            double distance = Vector3D.Distance(currentPosition, targetPosition);
            double step = Math.Min(distance, config.MaxWaypointDistance());

            Vector3D.Multiply(ref _tempDirection, step, out _tempOffset);
            Vector3D.Add(ref currentPosition, ref _tempOffset, out emergencyWaypoint);

            return true;
        }

        /// <summary>
        /// Create a simple two-point emergency path
        /// </summary>
        private bool CreateEmergencyPath(ref Vector3D start, ref Vector3D end, List<Vector3D> result)
        {
            if (result == null)
                throw new ArgumentNullException("result");

            if (!config.AllowRepathing())
            {
                Log.Error("PathfindingManager: Emergency path requested but repathing is disabled");
                result.Clear();
                return false;
            }

            Log.Warning("PathfindingManager: Creating emergency direct path");
            result.Clear();
            result.Add(start);
            result.Add(end);
            return true;
        }

        /// <summary>
        /// Cache a waypoint in the traveled graph for potential repathing
        /// </summary>
        private void CacheWaypoint(ref Vector3D from, ref Vector3D to, double costToTarget)
        {
            try
            {
                double waypointDist = config.MaxWaypointDistance();
                Vector3D gridFromD = from / waypointDist;
                Vector3D gridToD = to / waypointDist;

                Vector3I gridFrom = Vector3I.Round(gridFromD);
                Vector3I gridTo = Vector3I.Round(gridToD);

                double distance = Vector3D.Distance(from, to);
                traveledGraph.AddEdge(gridFrom, gridTo, (float)distance);

                Log.Verbose("PathfindingManager: Cached waypoint edge in traveled graph");
            }
            catch (Exception ex)
            {
                Log.Warning("PathfindingManager: Failed to cache waypoint: {0}", ex.Message);
            }
        }

        /// <summary>
        /// Find previously traveled nodes near a position
        /// </summary>
        private void FindNearbyTraveledNodes(ref Vector3D position, double searchRadius,
            List<Vector3D> outputBuffer)
        {
            outputBuffer.Clear();

            double waypointDist = config.MaxWaypointDistance();
            Vector3D gridPositionD = position / waypointDist;
            Vector3I gridPosition = Vector3I.Round(gridPositionD);
            int searchRadiusGrid = (int)Math.Ceiling(searchRadius / waypointDist);

            try
            {
                var allNodes = traveledGraph.GetAllNodes();
                foreach (var node in allNodes)
                {
                    float gridDistance = Vector3.Distance(node, gridPosition);
                    if (gridDistance <= searchRadiusGrid)
                    {
                        Vector3D worldPos = new Vector3D(node.X, node.Y, node.Z) * waypointDist;
                        outputBuffer.Add(worldPos);
                    }
                }
            }
            catch (Exception ex)
            {
                Log.Warning("PathfindingManager: Error searching traveled graph: {0}", ex.Message);
            }
        }

        private void LogPathfindingResult(PathfindingManager.Method method, TimeSpan elapsed,
            ref Vector3D waypoint)
        {
            Log.LogPathfinding("Used {0}: {1:F1}ms, waypoint at {2}",
                method, elapsed.TotalMilliseconds, waypoint);
        }

        private void LogPathfindingResult(PathfindingManager.Method method, TimeSpan elapsed,
            int waypointCount)
        {
            Log.LogPathfinding("Used {0}: {1:F1}ms, {2} waypoints",
                method, elapsed.TotalMilliseconds, waypointCount);
        }

        /// <summary>
        /// Clear the traveled graph cache
        /// </summary>
        public void ClearTraveledGraph()
        {
            traveledGraph.Clear();
            Log.Info("PathfindingManager: Cleared traveled graph cache");
        }

        /// <summary>
        /// Clear old nodes from the traveled graph
        /// </summary>
        public void PruneOldTraveledNodes(TimeSpan maxAge)
        {
            var allNodes = traveledGraph.GetAllNodes();
            int nodeCount = 0;
            foreach (var node in allNodes)
            {
                nodeCount++;
            }

            if (nodeCount > 1000)
            {
                Log.Warning("PathfindingManager: Traveled graph has {0} nodes, clearing to prevent bloat",
                    nodeCount);
                ClearTraveledGraph();
            }
        }

        /// <summary>
        /// Check if there's a direct, unobstructed line of sight to the target
        /// </summary>
        private bool HasDirectLineOfSight(ref Vector3D start, ref Vector3D end,
            PathfindingContext context)
        {
            double distance = Vector3D.Distance(start, end);

            if (distance < 50.0)
                return false;

            if (distance > config.MaxSimulatedCameraRaycastMeters())
                return false;

            if (config.RequireCamerasForPathfinding())
            {
                Vector3D.Subtract(ref end, ref start, out _tempDirection);

                // Need to get transposed world matrix for CanRaycastInDirection
                if (context.Controller == null)
                    return false;

                MatrixD worldMatrix = context.Controller.WorldMatrix;
                MatrixD worldMatrixTransposed;
                MatrixD.Transpose(ref worldMatrix, out worldMatrixTransposed);

                if (!context.CanRaycastInDirection(ref _tempDirection, ref worldMatrixTransposed))
                {
                    return false;
                }
            }

            LineD ray = new LineD(start, end);
            _raycastResults.Clear();

            try
            {
                pruningStructure.GetTopmostEntitiesOverlappingRay(ref ray, _raycastResults);

                for (int i = 0; i < _raycastResults.Count; i++)
                {
                    if (IsObstacleEntity(_raycastResults[i].Element, context))
                    {
                        Log.Verbose("PathfindingManager: Direct line of sight blocked by {0}",
                            _raycastResults[i].Element.DisplayName ?? "unknown");
                        return false;
                    }
                }

                Log.Verbose("PathfindingManager: Direct line of sight confirmed ({0:F1}m)", distance);
                return true;
            }
            catch (Exception ex)
            {
                Log.Warning("PathfindingManager: Line of sight check failed: {0}", ex.Message);
                return false;
            }
        }

        /// <summary>
        /// Determine if an entity is an obstacle
        /// </summary>
        private bool IsObstacleEntity(IMyEntity entity, PathfindingContext context)
        {
            if (entity == null) return false;
            if (context.CubeGrid != null && entity.EntityId == context.CubeGrid.EntityId)
                return false;

            IMyCubeGrid grid = entity as IMyCubeGrid;
            if (grid != null)
            {
                if (context.CubeGrid != null)
                {
                    _connectedGridsBuffer.Clear();
                    MyAPIGateway.GridGroups.GetGroup(context.CubeGrid, GridLinkTypeEnum.Mechanical,
                        _connectedGridsBuffer);

                    if (_connectedGridsBuffer.Contains(grid))
                        return false;
                }

                return true;
            }

            if (entity is MyPlanet) return true;
            if (entity.ToString().Contains("Asteroid")) return true;

            return false;
        }
    }
}