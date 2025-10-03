using ImprovedAI.Config;
using ImprovedAI.Data.Scripts.ImprovedAI.Pathfinding;
using ImprovedAI.Utils.Logging;
using Sandbox.ModAPI;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
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
            DirectPathfinding = 1,           // Cheapest - straight line
            AStar = 2,               // A* pathfinding
            DStarLite = 4 // added later
        }
        private readonly ServerConfig.PathfindingConfig config;
        private readonly IPathfinder directPathfinder;
        private readonly IPathfinder astarPathfinder;

        // Adjacency list for caching traveled paths and enabling efficient repathing
        private readonly AdjacencyList traveledGraph;

        // Current active pathfinding state
        private PathfindingState currentState;
        Base6Directions.Direction controllerForwardDirection;
        public PathfindingManager(Base6Directions.Direction controllerFwdDirection)
        {
            config = IAISession.Instance?.GetConfig()?.Pathfinding;
            traveledGraph = new AdjacencyList();
            controllerForwardDirection = controllerFwdDirection;

            // Initialize available pathfinders
            directPathfinder = config?.AllowDirectPathfinding == true ? new DirectPathfinder() : null;
            astarPathfinder = config?.AllowAStar == true ? new AStarPathfinder() : null;

            if (directPathfinder == null)
            {
                Log.Error("PathfindingManager: DirectPathfinding is disabled but required! Drones will not function.");
            }
        }

        /// <summary>
        /// Get the next waypoint dynamically without generating the full path.
        /// This is the primary method used during drone operation.
        /// </summary>
        public Vector3D? GetNextWaypoint(Vector3D currentPosition, Vector3D targetPosition, PathfindingContext context)
        {
            if (config == null)
            {
                Log.Error("PathfindingManager: Configuration not loaded");
                return null;
            }

            // Clamp waypoint distance to server limits
            context.WaypointDistance = MathHelper.Clamp(
                context.WaypointDistance,
                config.MinWaypointDistance,
                config.MaxWaypointDistance);

            // Check if we're close enough to target
            var distanceToTarget = Vector3D.Distance(currentPosition, targetPosition);
            if (distanceToTarget < context.WaypointDistance)
            {
                return targetPosition; // We're close, just go directly
            }

            // Try to get waypoint from preferred pathfinder
            var pathfinder = SelectPathfinder(currentPosition, targetPosition, context);
            if (pathfinder == null)
            {
                Log.Error("PathfindingManager: No pathfinder available");
                return HandleNoPathfinderAvailable(currentPosition, targetPosition, context);
            }

            var stopwatch = Stopwatch.StartNew();

            try
            {
                var waypoint = pathfinder.GetNextWaypoint(currentPosition, targetPosition, context);

                if (waypoint.HasValue)
                {
                    // Cache this waypoint in traveled graph for potential repathing
                    CacheWaypoint(currentPosition, waypoint.Value, distanceToTarget);

                    stopwatch.Stop();
                    LogPathfindingResult(pathfinder.Method, stopwatch.Elapsed, waypoint.Value);
                    return waypoint.Value;
                }

                // Pathfinder failed, try repathing if allowed
                if (config.AllowRepathing)
                {
                    return HandleRepathing(currentPosition, targetPosition, context, pathfinder);
                }

                Log.Warning("PathfindingManager: Pathfinder failed and repathing is disabled");
                return null;
            }
            catch (Exception ex)
            {
                stopwatch.Stop();
                Log.Error("PathfindingManager: Exception in GetNextWaypoint: {0}", ex.Message);

                if (config.AllowRepathing)
                {
                    return HandleEmergencyPath(currentPosition, targetPosition, context);
                }

                return null;
            }
        }

        /// <summary>
        /// Generate the complete path for debugging purposes.
        /// WARNING: This is expensive and should not be used during normal operation!
        /// </summary>
        public List<Vector3D> GenerateCompletePath(Vector3D start, Vector3D end, PathfindingContext context)
        {
            Log.Warning("PathfindingManager: GenerateCompletePath called - this is for debugging only!");

            if (config == null)
            {
                Log.Error("PathfindingManager: Configuration not loaded");
                return CreateEmergencyPath(start, end);
            }

            context.WaypointDistance = MathHelper.Clamp(
                context.WaypointDistance,
                config.MinWaypointDistance,
                config.MaxWaypointDistance);

            var pathfinder = SelectPathfinder(start, end, context);
            if (pathfinder == null)
            {
                return CreateEmergencyPath(start, end);
            }

            var stopwatch = Stopwatch.StartNew();

            try
            {
                var path = pathfinder.CalculatePath(start, end, context);

                if (IsValidPath(path, start, end, context))
                {
                    stopwatch.Stop();
                    LogPathfindingResult(pathfinder.Method, stopwatch.Elapsed, path.Count);
                    return path;
                }

                // Path invalid, try fallback
                if (config.AllowRepathing && pathfinder != directPathfinder)
                {
                    Log.Warning("PathfindingManager: Primary pathfinder failed, falling back to direct");
                    path = directPathfinder?.CalculatePath(start, end, context);

                    if (IsValidPath(path, start, end, context))
                    {
                        return path;
                    }
                }
            }
            catch (Exception ex)
            {
                Log.Error("PathfindingManager: Exception in GenerateCompletePath: {0}", ex.Message);
            }

            // Check if we're allowed to use emergency path
            if (config.AllowRepathing)
            {
                Log.Warning("PathfindingManager: All pathfinders failed, using emergency path");
                return CreateEmergencyPath(start, end);
            }

            Log.Error("PathfindingManager: Pathfinding failed and repathing/emergency paths disabled");
            return new List<Vector3D>(); // Empty path indicates failure
        }

        /// <summary>
        /// Validate that a path is safe and reaches the destination
        /// </summary>
        private bool IsValidPath(List<Vector3D> path, Vector3D start, Vector3D end, PathfindingContext context)
        {
            if (path == null || path.Count < 1)
            {
                Log.Warning("PathfindingManager: Path is null or empty");
                return false;
            }

            // Check that path starts reasonably close to start position
            var pathStart = path.First();
            if (Vector3D.DistanceSquared(pathStart, start) > 100) // Within 10m
            {
                Log.Warning("PathfindingManager: Path start is too far from actual start position");
                return false;
            }

            // Check that path ends reasonably close to target
            var pathEnd = path.Last();
            if (Vector3D.DistanceSquared(pathEnd, end) > 100) // Within 10m
            {
                Log.Warning("PathfindingManager: Path end is too far from target position");
                return false;
            }

            // Check waypoint count limits
            if (path.Count > config.MaxPathNodes)
            {
                Log.Warning("PathfindingManager: Path exceeds maximum node count ({0} > {1})",
                    path.Count, config.MaxPathNodes);
                return false;
            }

            // Validate waypoint spacing
            for (int i = 0; i < path.Count - 1; i++)
            {
                var distance = Vector3D.Distance(path[i], path[i + 1]);

                if (distance < config.MinWaypointDistance * 0.5) // Allow some tolerance
                {
                    Log.Warning("PathfindingManager: Waypoints too close together at index {0}", i);
                    return false;
                }

                if (distance > config.MaxWaypointDistance * 2.0) // Allow some tolerance
                {
                    Log.Warning("PathfindingManager: Waypoints too far apart at index {0}", i);
                    return false;
                }
            }

            // Optional: Check if path stays at safe altitude in gravity
            if (config.UsePlanetAwarePathfinding && context.IsInPlanetGravity())
            {
                foreach (var waypoint in path)
                {
                    // Create temporary context at waypoint position to check altitude
                    var tempContext = new PathfindingContext(
                        context.Controller,
                        context.Sensors,
                        context.Cameras,
                        new List<IMyThrust>(), // Don't need thrusters for altitude check
                        context.ShipMass,
                        context.MaxLoad,
                        context.WaypointDistance,
                        controllerForwardDirection
                    );

                    // Manually set position for check
                    var altitude = context.GetSurfaceAltitude();
                    if (altitude.HasValue && altitude.Value < config.MinAltitudeBuffer)
                    {
                        Log.Warning("PathfindingManager: Path goes below safe altitude");
                        return false;
                    }
                }
            }

            return true;
        }

        /// <summary>
        /// Select the best pathfinder based on context and configuration
        /// </summary>
        private IPathfinder SelectPathfinder(Vector3D start, Vector3D end, PathfindingContext context)
        {
            var distance = Vector3D.Distance(start, end);

            // For very short distances, always use direct pathfinding
            if (distance < config.MinWaypointDistance * 2)
            {
                return directPathfinder;
            }

            // Check if A* is available and conditions warrant its use
            if (astarPathfinder != null && astarPathfinder.IsAvailable(context))
            {
                // Use A* for complex environments with obstacles
                if (context.SensorInfos?.Count > 0 || context.CamerasByDirection?.Count > 0)
                {
                    var complexity = astarPathfinder.EstimatedComplexity(start, end);

                    // Only use A* if the complexity is reasonable
                    if (complexity < config.MaxPathNodes)
                    {
                        Log.Verbose("PathfindingManager: Selected A* pathfinder (complexity: {0})", complexity);
                        return astarPathfinder;
                    }
                }
            }

            // Default to direct pathfinding
            if (directPathfinder != null && directPathfinder.IsAvailable(context))
            {
                Log.Verbose("PathfindingManager: Selected Direct pathfinder");
                return directPathfinder;
            }

            return null;
        }

        /// <summary>
        /// Handle repathing when primary pathfinder fails
        /// </summary>
        private Vector3D? HandleRepathing(Vector3D currentPosition, Vector3D targetPosition,
            PathfindingContext context, IPathfinder failedPathfinder)
        {
            Log.Info("PathfindingManager: Attempting repathing from {0}", failedPathfinder.Method);

            // Try to find a previously traveled node that could help
            var nearbyNodes = FindNearbyTraveledNodes(currentPosition, config.MaxWaypointDistance * 2);

            foreach (var node in nearbyNodes)
            {
                // Check if this node provides a better path to target
                var distanceFromNode = Vector3D.Distance(node, targetPosition);
                var distanceDirect = Vector3D.Distance(currentPosition, targetPosition);

                if (distanceFromNode < distanceDirect * 0.9) // 10% improvement threshold
                {
                    Log.Info("PathfindingManager: Found useful cached node for repathing");
                    return node;
                }
            }

            // No useful cached nodes, try fallback pathfinder
            if (failedPathfinder == astarPathfinder && directPathfinder != null)
            {
                Log.Info("PathfindingManager: Falling back to Direct pathfinder");
                var waypoint = directPathfinder.GetNextWaypoint(currentPosition, targetPosition, context);

                if (waypoint.HasValue)
                {
                    return waypoint;
                }
            }

            // Last resort: emergency path
            return HandleEmergencyPath(currentPosition, targetPosition, context);
        }

        /// <summary>
        /// Create an emergency direct path when all else fails (only if repathing is enabled)
        /// </summary>
        private Vector3D? HandleEmergencyPath(Vector3D currentPosition, Vector3D targetPosition,
            PathfindingContext context)
        {
            if (!config.AllowRepathing)
            {
                Log.Error("PathfindingManager: Emergency path needed but repathing is disabled");
                return null;
            }

            Log.Warning("PathfindingManager: Using emergency waypoint generation");

            // Calculate a safe intermediate waypoint
            var direction = Vector3D.Normalize(targetPosition - currentPosition);
            var distance = Vector3D.Distance(currentPosition, targetPosition);
            var waypointDistance = Math.Min(distance * 0.5, config.MaxWaypointDistance);

            var waypoint = currentPosition + direction * waypointDistance;

            // If in gravity, ensure we maintain safe altitude
            if (context.IsInPlanetGravity() && config.UsePlanetAwarePathfinding)
            {
                var altitude = context.GetSurfaceAltitude();
                if (altitude.HasValue && altitude.Value < config.MinAltitudeBuffer)
                {
                    var gravityUp = Vector3D.Normalize(-context.GravityVector);
                    waypoint += gravityUp * (config.MinAltitudeBuffer - altitude.Value);
                }
            }

            return waypoint;
        }

        /// <summary>
        /// Handle case where no pathfinder is available
        /// </summary>
        private Vector3D? HandleNoPathfinderAvailable(Vector3D currentPosition, Vector3D targetPosition,
            PathfindingContext context)
        {
            if (!config.AllowRepathing)
            {
                Log.Error("PathfindingManager: No pathfinder available and repathing disabled");
                return null;
            }

            Log.Error("PathfindingManager: No pathfinder available, using basic emergency waypoint");

            var direction = Vector3D.Normalize(targetPosition - currentPosition);
            var distance = Vector3D.Distance(currentPosition, targetPosition);
            var step = Math.Min(distance, config.MaxWaypointDistance);

            return currentPosition + direction * step;
        }

        /// <summary>
        /// Create a simple two-point emergency path (only if repathing is enabled)
        /// </summary>
        private List<Vector3D> CreateEmergencyPath(Vector3D start, Vector3D end)
        {
            if (!config.AllowRepathing)
            {
                Log.Error("PathfindingManager: Emergency path requested but repathing is disabled");
                return new List<Vector3D>(); // Empty path
            }

            Log.Warning("PathfindingManager: Creating emergency direct path");
            return new List<Vector3D> { start, end };
        }

        /// <summary>
        /// Cache a waypoint in the traveled graph for potential repathing
        /// </summary>
        private void CacheWaypoint(Vector3D from, Vector3D to, double costToTarget)
        {
            try
            {
                // Convert to grid coordinates for caching (reduce precision to avoid bloat)
                var gridFrom = Vector3I.Round(from / config.MaxWaypointDistance);
                var gridTo = Vector3I.Round(to / config.MaxWaypointDistance);

                var distance = Vector3D.Distance(from, to);

                // Add edge with distance as cost
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
        private List<Vector3D> FindNearbyTraveledNodes(Vector3D position, double searchRadius)
        {
            var nearbyNodes = new List<Vector3D>();
            var gridPosition = Vector3I.Round(position / config.MaxWaypointDistance);
            var searchRadiusGrid = (int)Math.Ceiling(searchRadius / config.MaxWaypointDistance);

            try
            {
                foreach (var node in traveledGraph.GetAllNodes())
                {
                    var gridDistance = Vector3.Distance(node, gridPosition);
                    if (gridDistance <= searchRadiusGrid)
                    {
                        // Convert back to world space
                        var worldPos = new Vector3D(node.X, node.Y, node.Z) * config.MaxWaypointDistance;
                        nearbyNodes.Add(worldPos);
                    }
                }
            }
            catch (Exception ex)
            {
                Log.Warning("PathfindingManager: Error searching traveled graph: {0}", ex.Message);
            }

            return nearbyNodes;
        }

        private void LogPathfindingResult(PathfindingManager.Method method, TimeSpan elapsed, Vector3D waypoint)
        {
            Log.LogPathfinding("Used {0}: {1:F1}ms, waypoint at {2}",
                method, elapsed.TotalMilliseconds, waypoint);
        }

        private void LogPathfindingResult(PathfindingManager.Method method, TimeSpan elapsed, int waypointCount)
        {
            Log.LogPathfinding("Used {0}: {1:F1}ms, {2} waypoints",
                method, elapsed.TotalMilliseconds, waypointCount);
        }

        /// <summary>
        /// Internal state tracking for multi-step pathfinding operations
        /// </summary>
        private class PathfindingState
        {
            public Vector3D CurrentPosition { get; set; }
            public Vector3D TargetPosition { get; set; }
            public List<Vector3D> RemainingPath { get; set; }
            public int RepathAttempts { get; set; }
            public PathfindingManager.Method LastMethod { get; set; }
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
        /// Clear old nodes from the traveled graph (older than specified age)
        /// </summary>
        public void PruneOldTraveledNodes(TimeSpan maxAge)
        {
            // This would require adding timestamps to the AdjacencyList
            // For now, just clear everything if it gets too big
            var nodeCount = traveledGraph.GetAllNodes().Count();
            if (nodeCount > 1000) // Arbitrary threshold
            {
                Log.Warning("PathfindingManager: Traveled graph has {0} nodes, clearing to prevent bloat", nodeCount);
                ClearTraveledGraph();
            }
        }
    }
}