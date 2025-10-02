using ImprovedAI.Config;
using ImprovedAI.Utils.Logging;
using Sandbox.Game.Entities;
using Sandbox.ModAPI;
using System;
using System.Collections.Generic;
using VRage.Game.Entity;
using VRage.Game.ModAPI;
using VRage.ModAPI;
using VRageMath;

namespace ImprovedAI.Pathfinding
{
    public class AStarPathfinder : IPathfinder
    {
        /// <summary>
        /// Helper class to store direction and its priority
        /// </summary>
        private class DirectionInfo
        {
            public Vector3I Direction { get; set; }
            public int Priority { get; set; }
        }
        public PathfindingManager.Method Method => PathfindingManager.Method.AStar;

        private readonly ServerConfig.PathfindingConfig config;
        private readonly DirectPathfinder directFallback;

        // A* algorithm state
        private Dictionary<Vector3I, AStarNode> openSet;
        private Dictionary<Vector3I, AStarNode> closedSet;
        private PriorityQueue<AStarNode> openQueue;

        public AStarPathfinder()
        {
            config = IAISession.Instance?.GetConfig()?.Pathfinding;
            directFallback = new DirectPathfinder();
        }

        public bool IsAvailable(PathfindingContext context)
        {
            if (config?.AllowAStar != true)
                return false;

            // A* requires either sensors or cameras for obstacle detection
            bool hasSensors = config.RequireSensorsForPathinding &&
                             context.SensorInfos?.Count > 0;
            bool hasCameras = config.RequireCamerasForPathfinding &&
                             context.CamerasByDirection?.Count > 0;

            return hasSensors || hasCameras;
        }

        public int EstimatedComplexity(Vector3D start, Vector3D end)
        {
            var distance = Vector3D.Distance(start, end);
            // Estimate number of nodes based on distance and waypoint spacing
            var estimatedNodes = (int)(distance / config.MinWaypointDistance);
            return Math.Min(estimatedNodes, config.MaxPathNodes);
        }

        /// <summary>
        /// Get the next waypoint dynamically using A* search
        /// </summary>
        public Vector3D? GetNextWaypoint(Vector3D currentPosition, Vector3D targetPosition, PathfindingContext context)
        {
            if (config == null)
            {
                Log.Error("AStarPathfinder: Config not loaded");
                return null;
            }

            var distance = Vector3D.Distance(currentPosition, targetPosition);

            // For very short distances, use direct pathfinding
            if (distance < context.WaypointDistance * 2)
            {
                return directFallback.GetNextWaypoint(currentPosition, targetPosition, context);
            }

            // Try to find a path using A*
            var path = FindPath(currentPosition, targetPosition, context);

            if (path != null && path.Count > 1)
            {
                // Return the first waypoint in the path (after current position)
                return path[1];
            }

            // A* failed, fallback to direct pathfinding if repathing is allowed
            if (config.AllowRepathing)
            {
                Log.Warning("AStarPathfinder: Failed to find path, falling back to direct pathfinding");
                return directFallback.GetNextWaypoint(currentPosition, targetPosition, context);
            }

            Log.Warning("AStarPathfinder: Failed to find path and repathing disabled");
            return null;
        }

        /// <summary>
        /// Generate complete path using A* algorithm
        /// </summary>
        public List<Vector3D> CalculatePath(Vector3D start, Vector3D end, PathfindingContext context)
        {
            if (config == null)
            {
                Log.Error("AStarPathfinder: Config not loaded");
                return new List<Vector3D> { start, end };
            }

            var path = FindPath(start, end, context);

            if (path != null && path.Count > 0)
            {
                return path;
            }

            // Fallback to direct pathfinding
            if (config.AllowRepathing)
            {
                Log.Warning("AStarPathfinder: A* failed, using direct pathfinding fallback");
                return directFallback.CalculatePath(start, end, context);
            }

            Log.Error("AStarPathfinder: Failed to find path and repathing disabled");
            return new List<Vector3D>();
        }

        /// <summary>
        /// Core A* pathfinding algorithm
        /// </summary>
        /// <summary>
        /// Core A* pathfinding algorithm with deadlock detection and retracing
        /// </summary>
        private List<Vector3D> FindPath(Vector3D start, Vector3D end, PathfindingContext context)
        {
            var startTime = DateTime.UtcNow;

            // Convert to grid coordinates for discrete space
            var gridStart = WorldToGrid(start, context.WaypointDistance);
            var gridEnd = WorldToGrid(end, context.WaypointDistance);

            // Initialize A* data structures
            openSet = new Dictionary<Vector3I, AStarNode>();
            closedSet = new Dictionary<Vector3I, AStarNode>();
            openQueue = new PriorityQueue<AStarNode>();

            // Create start node
            var startNode = new AStarNode
            {
                Position = gridStart,
                GCost = 0,
                HCost = Heuristic(gridStart, gridEnd, context),
                Parent = null
            };

            openSet[gridStart] = startNode;
            openQueue.Enqueue(startNode);

            int nodesExplored = 0;
            int stuckCounter = 0;
            Vector3I? lastPosition = null;

            while (openQueue.Count > 0)
            {
                // Check timeout
                if ((DateTime.UtcNow - startTime) > config.MaxPathfindingTime)
                {
                    Log.Warning("AStarPathfinder: Timeout after exploring {0} nodes", nodesExplored);
                    return null;
                }

                // Check node limit
                if (nodesExplored >= config.MaxPathNodes)
                {
                    Log.Warning("AStarPathfinder: Max nodes reached ({0})", config.MaxPathNodes);
                    return null;
                }

                // Get node with lowest F cost
                var current = openQueue.Dequeue();
                openSet.Remove(current.Position);
                closedSet[current.Position] = current;
                nodesExplored++;

                // Deadlock detection: Check if we're stuck exploring the same area
                if (lastPosition.HasValue && Vector3.Distance(current.Position, lastPosition.Value) < 2)
                {
                    stuckCounter++;
                    if (stuckCounter > 10)
                    {
                        Log.Verbose("AStarPathfinder: Possible deadlock detected, allowing backward exploration");
                        // The priority system will now naturally allow backward movement
                        // since all forward and side options are likely in closedSet
                        stuckCounter = 0; // Reset counter
                    }
                }
                else
                {
                    stuckCounter = 0;
                }
                lastPosition = current.Position;

                // Check if we reached the goal
                if (Vector3.Distance(current.Position, gridEnd) <= 1)
                {
                    Log.Info("AStarPathfinder: Found path with {0} nodes in {1:F1}ms",
                        nodesExplored, (DateTime.UtcNow - startTime).TotalMilliseconds);
                    return ReconstructPath(current, context.WaypointDistance);
                }

                // Explore neighbors
                var neighbors = GetNeighbors(current.Position, context);

                foreach (var neighbor in neighbors)
                {
                    // Skip if already explored
                    if (closedSet.ContainsKey(neighbor))
                    {
                        // Only allow revisiting if we're stuck and repathing is enabled
                        if (!config.AllowRepathing || stuckCounter < 5)
                            continue;

                        // If stuck and repathing allowed, we can reconsider closed nodes
                        // Remove from closed set to allow re-exploration
                        closedSet.Remove(neighbor);
                        Log.Verbose("AStarPathfinder: Retracing to position {0} due to deadlock", neighbor);
                    }

                    var neighborWorld = GridToWorld(neighbor, context.WaypointDistance);

                    // Check if neighbor is traversable
                    if (!IsTraversable(neighborWorld, context))
                        continue;

                    var tentativeGCost = current.GCost + GetMovementCost(current.Position, neighbor, context);

                    AStarNode neighborNode;
                    if (!openSet.TryGetValue(neighbor, out neighborNode))
                    {
                        neighborNode = new AStarNode
                        {
                            Position = neighbor,
                            GCost = float.MaxValue,
                            HCost = Heuristic(neighbor, gridEnd, context),
                            Parent = null
                        };
                        openSet[neighbor] = neighborNode;
                    }

                    // If this path to neighbor is better
                    if (tentativeGCost < neighborNode.GCost)
                    {
                        neighborNode.GCost = tentativeGCost;
                        neighborNode.Parent = current;

                        // Add to queue if not already there
                        if (!openQueue.Contains(neighborNode))
                        {
                            openQueue.Enqueue(neighborNode);
                        }
                    }
                }
            }

            Log.Warning("AStarPathfinder: No path found after exploring {0} nodes", nodesExplored);
            return null;
        }

        /// <summary>
        /// Heuristic function for A* (estimated cost from current to goal)
        /// </summary>
        private float Heuristic(Vector3I from, Vector3I to, PathfindingContext context)
        {
            // Euclidean distance
            var distance = Vector3.Distance(from, to);

            // Add penalty for moving against gravity if in planet gravity
            if (context.IsInPlanetGravity() && context.PlanetCenter.HasValue)
            {
                var fromWorld = GridToWorld(from, config.MinWaypointDistance);
                var toWorld = GridToWorld(to, config.MinWaypointDistance);

                var fromRadius = Vector3D.Distance(fromWorld, context.PlanetCenter.Value);
                var toRadius = Vector3D.Distance(toWorld, context.PlanetCenter.Value);

                // Add cost for climbing (moving away from planet)
                if (toRadius > fromRadius)
                {
                    var climbDistance = toRadius - fromRadius;
                    distance += (float)(climbDistance * 0.5); // 50% penalty for climbing
                }
            }

            return distance;
        }

        /// <summary>
        /// Get actual movement cost between two adjacent grid positions
        /// </summary>
        /// <summary>
        /// Get actual movement cost between two adjacent grid positions
        /// </summary>
        private float GetMovementCost(Vector3I from, Vector3I to, PathfindingContext context)
        {
            var fromWorld = GridToWorld(from, config.MinWaypointDistance);
            var toWorld = GridToWorld(to, config.MinWaypointDistance);

            var distance = Vector3D.Distance(fromWorld, toWorld);
            var movementCost = (float)distance;

            // Calculate direction and priority
            var direction = to - from;
            var controllerForward = context.GetControllerForwardInWorld();
            var localForward = Vector3D.TransformNormal(controllerForward,
                MatrixD.Transpose(context.Controller.WorldMatrix));

            var absForward = Vector3D.Abs(localForward);
            Vector3I forwardAxis;

            if (absForward.Z > absForward.X && absForward.Z > absForward.Y)
                forwardAxis = new Vector3I(0, 0, localForward.Z > 0 ? 1 : -1);
            else if (absForward.Y > absForward.X)
                forwardAxis = new Vector3I(0, localForward.Y > 0 ? 1 : -1, 0);
            else
                forwardAxis = new Vector3I(localForward.X > 0 ? 1 : -1, 0, 0);

            int alignment = direction.Dot(ref forwardAxis);

            // Add cost penalty for backward movement
            if (alignment < 0)
            {
                movementCost *= 1.5f; // 50% penalty for backward movement
            }
            else if (alignment == 0)
            {
                movementCost *= 1.1f; // 10% penalty for sideways movement
            }
            // Forward movement has no penalty

            // Add cost for moving against gravity
            if (context.GravityVector.LengthSquared() > 0.1)
            {
                var worldDirection = toWorld - fromWorld;
                var gravityUp = Vector3D.Normalize(-context.GravityVector);
                var climbFactor = Vector3D.Dot(Vector3D.Normalize(worldDirection), gravityUp);

                if (climbFactor > 0) // Moving up
                {
                    // Check if we have enough thrust
                    if (!context.CanClimbInDirection(worldDirection))
                    {
                        movementCost += 1000.0f; // High penalty for impossible climbs
                    }
                    else
                    {
                        movementCost *= (1.0f + (float)climbFactor * 0.5f); // Increase cost proportional to climb angle
                    }
                }
            }

            // Add cost for diagonal movement (more than one axis)
            var axisCount = 0;
            if (direction.X != 0) axisCount++;
            if (direction.Y != 0) axisCount++;
            if (direction.Z != 0) axisCount++;

            if (axisCount == 2)
            {
                movementCost *= 1.414f; // sqrt(2) for 2D diagonal
            }
            else if (axisCount == 3)
            {
                movementCost *= 1.732f; // sqrt(3) for 3D diagonal
            }

            return movementCost;
        }

        /// <summary>
        /// Get neighboring grid positions to explore with intelligent prioritization
        /// Priority: Forward 9 directions > Side 8 directions > Backward 9 directions
        /// </summary>
        private List<Vector3I> GetNeighbors(Vector3I position, PathfindingContext context)
        {
            var neighbors = new List<Vector3I>();

            // Get the forward direction from the controller
            var controllerForward = context.GetControllerForwardInWorld();
            var localForward = Vector3D.TransformNormal(controllerForward,
                MatrixD.Transpose(context.Controller.WorldMatrix));

            // Determine primary forward axis in grid space
            var absForward = Vector3D.Abs(localForward);
            Vector3I forwardAxis;

            if (absForward.Z > absForward.X && absForward.Z > absForward.Y)
                forwardAxis = new Vector3I(0, 0, localForward.Z > 0 ? 1 : -1);
            else if (absForward.Y > absForward.X)
                forwardAxis = new Vector3I(0, localForward.Y > 0 ? 1 : -1, 0);
            else
                forwardAxis = new Vector3I(localForward.X > 0 ? 1 : -1, 0, 0);

            // Generate all 26 directions (3x3x3 cube minus center)
            var allDirections = new List<DirectionInfo>();

            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    for (int z = -1; z <= 1; z++)
                    {
                        // Skip center (current position)
                        if (x == 0 && y == 0 && z == 0)
                            continue;

                        var direction = new Vector3I(x, y, z);
                        var priority = CalculateDirectionPriority(direction, forwardAxis);

                        allDirections.Add(new DirectionInfo
                        {
                            Direction = direction,
                            Priority = priority
                        });
                    }
                }
            }

            // Sort by priority (lower number = higher priority)
            allDirections.Sort((a, b) => a.Priority.CompareTo(b.Priority));

            // Add to neighbors list in priority order
            foreach (var dirInfo in allDirections)
            {
                neighbors.Add(position + dirInfo.Direction);
            }

            return neighbors;
        }

        /// <summary>
        /// Calculate direction priority based on alignment with forward direction
        /// Priority 1: Forward 9 directions (front face + edges + corners)
        /// Priority 2: Side 8 directions (middle ring without forward/backward)
        /// Priority 3: Backward 9 directions (back face + edges + corners)
        /// </summary>
        private int CalculateDirectionPriority(Vector3I direction, Vector3I forwardAxis)
        {
            // Calculate dot product to determine forward/backward alignment
            int alignment = direction.Dot(ref forwardAxis);

            if (alignment > 0)
            {
                // Forward directions (priority 1)
                return 1;
            }
            else if (alignment == 0)
            {
                // Side directions (priority 2)
                return 2;
            }
            else
            {
                // Backward directions (priority 3)
                return 3;
            }
        }

        /// <summary>
        /// Check if a position is traversable (no obstacles)
        /// </summary>
        private bool IsTraversable(Vector3D position, PathfindingContext context)
        {
            // Check altitude if in gravity
            if (config.UsePlanetAwarePathfinding && context.IsInPlanetGravity() && context.PlanetCenter.HasValue)
            {
                var altitude = Vector3D.Distance(position, context.PlanetCenter.Value) - context.PlanetRadius;
                if (altitude < config.MinAltitudeBuffer)
                {
                    return false; // Too low
                }
            }

            // Check for obstacles using sensors
            if (config.RequireSensorsForPathinding && context.SensorInfos?.Count > 0)
            {
                if (HasObstacleAtPosition(position, context))
                {
                    return false;
                }
            }

            // Check thrust capability
            // (Note: This is a simplified check - in reality would need to check from previous position)
            if (!context.HasDetailedThrustData())
            {
                return true; // Can't verify, assume traversable
            }

            return true;
        }

        /// <summary>
        /// Check if there's an obstacle at a specific position using sensor detection
        /// </summary>
        private bool HasObstacleAtPosition(Vector3D position, PathfindingContext context)
        {
            foreach (var sensorInfo in context.SensorInfos)
            {
                var distanceToSensor = Vector3D.Distance(position, sensorInfo.Position);
                if (distanceToSensor > sensorInfo.MaxRange)
                    continue;

                // Create small bounding box around position
                var halfSize = new Vector3D(config.MinWaypointDistance * 0.5);
                var box = new BoundingBoxD(position - halfSize, position + halfSize);

                var entities = new List<MyEntity>();
                try
                {
                    MyGamePruningStructure.GetTopmostEntitiesInBox(ref box, entities);

                    foreach (var entity in entities)
                    {
                        if (IsObstacleEntity(entity, context))
                        {
                            return true;
                        }
                    }
                }
                catch (Exception ex)
                {
                    Log.Warning("AStarPathfinder: Obstacle detection exception: {0}", ex.Message);
                }
            }

            return false;
        }

        /// <summary>
        /// Determine if an entity is an obstacle
        /// </summary>
        private bool IsObstacleEntity(IMyEntity entity, PathfindingContext context)
        {
            if (entity == null) return false;

            // Ignore our own grid
            if (entity.EntityId == context.CubeGrid?.EntityId) return false;

            // Check if it's a grid that belongs to us
            var grid = entity as IMyCubeGrid;
            if (grid != null)
            {
                var connectedGrids = new List<IMyCubeGrid>();
                MyAPIGateway.GridGroups.GetGroup(context.CubeGrid, GridLinkTypeEnum.Mechanical, connectedGrids);

                if (connectedGrids.Contains(grid))
                {
                    return false; // Part of our ship
                }

                return true; // Other grid
            }

            // Planets and asteroids
            if (entity is MyPlanet) return true;
            if (entity.ToString().Contains("Asteroid")) return true;

            return false;
        }

        /// <summary>
        /// Reconstruct the path from the goal node back to start
        /// </summary>
        private List<Vector3D> ReconstructPath(AStarNode goalNode, double waypointDistance)
        {
            var path = new List<Vector3D>();
            var current = goalNode;

            while (current != null)
            {
                path.Add(GridToWorld(current.Position, waypointDistance));
                current = current.Parent;
            }

            path.Reverse();
            return path;
        }

        /// <summary>
        /// Convert world position to grid coordinates
        /// </summary>
        private Vector3I WorldToGrid(Vector3D world, double gridSize)
        {
            return new Vector3I(
                (int)Math.Round(world.X / gridSize),
                (int)Math.Round(world.Y / gridSize),
                (int)Math.Round(world.Z / gridSize)
            );
        }

        /// <summary>
        /// Convert grid coordinates to world position
        /// </summary>
        private Vector3D GridToWorld(Vector3I grid, double gridSize)
        {
            return new Vector3D(
                grid.X * gridSize,
                grid.Y * gridSize,
                grid.Z * gridSize
            );
        }

        /// <summary>
        /// A* node class
        /// </summary>
        private class AStarNode : IComparable<AStarNode>
        {
            public Vector3I Position { get; set; }
            public float GCost { get; set; } // Cost from start
            public float HCost { get; set; } // Heuristic cost to goal
            public float FCost => GCost + HCost; // Total cost
            public AStarNode Parent { get; set; }

            public int CompareTo(AStarNode other)
            {
                return FCost.CompareTo(other.FCost);
            }
        }

        /// <summary>
        /// Simple priority queue for A* open set
        /// </summary>
        private class PriorityQueue<T> where T : IComparable<T>
        {
            private List<T> data;

            public PriorityQueue()
            {
                data = new List<T>();
            }

            public void Enqueue(T item)
            {
                data.Add(item);
                int ci = data.Count - 1;
                while (ci > 0)
                {
                    int pi = (ci - 1) / 2;
                    if (data[ci].CompareTo(data[pi]) >= 0)
                        break;
                    T tmp = data[ci];
                    data[ci] = data[pi];
                    data[pi] = tmp;
                    ci = pi;
                }
            }

            public T Dequeue()
            {
                int li = data.Count - 1;
                T frontItem = data[0];
                data[0] = data[li];
                data.RemoveAt(li);

                --li;
                int pi = 0;
                while (true)
                {
                    int ci = pi * 2 + 1;
                    if (ci > li) break;
                    int rc = ci + 1;
                    if (rc <= li && data[rc].CompareTo(data[ci]) < 0)
                        ci = rc;
                    if (data[pi].CompareTo(data[ci]) <= 0)
                        break;
                    T tmp = data[pi];
                    data[pi] = data[ci];
                    data[ci] = tmp;
                    pi = ci;
                }
                return frontItem;
            }

            public int Count => data.Count;

            public bool Contains(T item)
            {
                return data.Contains(item);
            }
        }
    }
}