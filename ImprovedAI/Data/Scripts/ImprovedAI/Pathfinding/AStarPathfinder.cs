using ImprovedAI.Config;
using ImprovedAI.Utils.Logging;
using Sandbox.Game.Entities;
using Sandbox.ModAPI;
using System;
using System.Collections.Generic;
using System.Linq;
using VRage.Game.Entity;
using VRage.Game.ModAPI;
using VRage.ModAPI;
using VRageMath;

namespace ImprovedAI.Pathfinding
{
    public class AStarPathfinder : IPathfinder
    {
        public PathfindingManager.Method Method => PathfindingManager.Method.AStar;

        private readonly ServerConfig.PathfindingConfig config;
        private readonly DirectPathfinder directFallback;

        // Pre-allocated data structures (reused across pathfinding operations)
        private readonly Dictionary<Vector3I, AStarNode> openSet = new Dictionary<Vector3I, AStarNode>();
        private readonly Dictionary<Vector3I, AStarNode> closedSet = new Dictionary<Vector3I, AStarNode>();
        private readonly PriorityQueue<AStarNode> openQueue = new PriorityQueue<AStarNode>();

        // Pre-allocated node pool to avoid allocations
        private readonly AStarNode[] nodePool = new AStarNode[1000]; // Match MaxPathNodes
        private int nodePoolIndex = 0;

        // Reusable lists to avoid allocations
        private readonly List<Vector3I> neighborList = new List<Vector3I>(26);
        private readonly List<Vector3D> pathList = new List<Vector3D>();
        private readonly List<MyEntity> entityList = new List<MyEntity>();

        // Static pre-computed direction arrays for all 6 forward orientations
        // Each array is pre-sorted by priority: Forward 9, Side 8, Backward 9

        // Forward = +Z
        private static readonly Vector3I[] DIRECTIONS_FORWARD_Z = new Vector3I[26]
        {
            // Forward 9 (priority 1)
            new Vector3I(1,1,1), new Vector3I(0,1,1), new Vector3I(-1,1,1),
            new Vector3I(1,0,1), new Vector3I(0,0,1), new Vector3I(-1,0,1),
            new Vector3I(1,-1,1), new Vector3I(0,-1,1), new Vector3I(-1,-1,1),
            // Side 8 (priority 2)
            new Vector3I(1,1,0), new Vector3I(0,1,0), new Vector3I(-1,1,0),
            new Vector3I(1,0,0), new Vector3I(-1,0,0),
            new Vector3I(1,-1,0), new Vector3I(0,-1,0), new Vector3I(-1,-1,0),
            // Backward 9 (priority 3)
            new Vector3I(1,1,-1), new Vector3I(0,1,-1), new Vector3I(-1,1,-1),
            new Vector3I(1,0,-1), new Vector3I(0,0,-1), new Vector3I(-1,0,-1),
            new Vector3I(1,-1,-1), new Vector3I(0,-1,-1), new Vector3I(-1,-1,-1)
        };

        // Forward = -Z
        private static readonly Vector3I[] DIRECTIONS_BACKWARD_Z = new Vector3I[26]
        {
            // Forward 9 (priority 1)
            new Vector3I(1,1,-1), new Vector3I(0,1,-1), new Vector3I(-1,1,-1),
            new Vector3I(1,0,-1), new Vector3I(0,0,-1), new Vector3I(-1,0,-1),
            new Vector3I(1,-1,-1), new Vector3I(0,-1,-1), new Vector3I(-1,-1,-1),
            // Side 8 (priority 2)
            new Vector3I(1,1,0), new Vector3I(0,1,0), new Vector3I(-1,1,0),
            new Vector3I(1,0,0), new Vector3I(-1,0,0),
            new Vector3I(1,-1,0), new Vector3I(0,-1,0), new Vector3I(-1,-1,0),
            // Backward 9 (priority 3)
            new Vector3I(1,1,1), new Vector3I(0,1,1), new Vector3I(-1,1,1),
            new Vector3I(1,0,1), new Vector3I(0,0,1), new Vector3I(-1,0,1),
            new Vector3I(1,-1,1), new Vector3I(0,-1,1), new Vector3I(-1,-1,1)
        };

        // Forward = +Y
        private static readonly Vector3I[] DIRECTIONS_FORWARD_Y = new Vector3I[26]
        {
            // Forward 9 (priority 1)
            new Vector3I(1,1,1), new Vector3I(0,1,1), new Vector3I(-1,1,1),
            new Vector3I(1,1,0), new Vector3I(0,1,0), new Vector3I(-1,1,0),
            new Vector3I(1,1,-1), new Vector3I(0,1,-1), new Vector3I(-1,1,-1),
            // Side 8 (priority 2)
            new Vector3I(1,0,1), new Vector3I(0,0,1), new Vector3I(-1,0,1),
            new Vector3I(1,0,0), new Vector3I(-1,0,0),
            new Vector3I(1,0,-1), new Vector3I(0,0,-1), new Vector3I(-1,0,-1),
            // Backward 9 (priority 3)
            new Vector3I(1,-1,1), new Vector3I(0,-1,1), new Vector3I(-1,-1,1),
            new Vector3I(1,-1,0), new Vector3I(0,-1,0), new Vector3I(-1,-1,0),
            new Vector3I(1,-1,-1), new Vector3I(0,-1,-1), new Vector3I(-1,-1,-1)
        };

        // Forward = -Y
        private static readonly Vector3I[] DIRECTIONS_BACKWARD_Y = new Vector3I[26]
        {
            // Forward 9 (priority 1)
            new Vector3I(1,-1,1), new Vector3I(0,-1,1), new Vector3I(-1,-1,1),
            new Vector3I(1,-1,0), new Vector3I(0,-1,0), new Vector3I(-1,-1,0),
            new Vector3I(1,-1,-1), new Vector3I(0,-1,-1), new Vector3I(-1,-1,-1),
            // Side 8 (priority 2)
            new Vector3I(1,0,1), new Vector3I(0,0,1), new Vector3I(-1,0,1),
            new Vector3I(1,0,0), new Vector3I(-1,0,0),
            new Vector3I(1,0,-1), new Vector3I(0,0,-1), new Vector3I(-1,0,-1),
            // Backward 9 (priority 3)
            new Vector3I(1,1,1), new Vector3I(0,1,1), new Vector3I(-1,1,1),
            new Vector3I(1,1,0), new Vector3I(0,1,0), new Vector3I(-1,1,0),
            new Vector3I(1,1,-1), new Vector3I(0,1,-1), new Vector3I(-1,1,-1)
        };

        // Forward = +X
        private static readonly Vector3I[] DIRECTIONS_FORWARD_X = new Vector3I[26]
        {
            // Forward 9 (priority 1)
            new Vector3I(1,1,1), new Vector3I(1,0,1), new Vector3I(1,-1,1),
            new Vector3I(1,1,0), new Vector3I(1,0,0), new Vector3I(1,-1,0),
            new Vector3I(1,1,-1), new Vector3I(1,0,-1), new Vector3I(1,-1,-1),
            // Side 8 (priority 2)
            new Vector3I(0,1,1), new Vector3I(0,0,1), new Vector3I(0,-1,1),
            new Vector3I(0,1,0), new Vector3I(0,-1,0),
            new Vector3I(0,1,-1), new Vector3I(0,0,-1), new Vector3I(0,-1,-1),
            // Backward 9 (priority 3)
            new Vector3I(-1,1,1), new Vector3I(-1,0,1), new Vector3I(-1,-1,1),
            new Vector3I(-1,1,0), new Vector3I(-1,0,0), new Vector3I(-1,-1,0),
            new Vector3I(-1,1,-1), new Vector3I(-1,0,-1), new Vector3I(-1,-1,-1)
        };

        // Forward = -X
        private static readonly Vector3I[] DIRECTIONS_BACKWARD_X = new Vector3I[26]
        {
            // Forward 9 (priority 1)
            new Vector3I(-1,1,1), new Vector3I(-1,0,1), new Vector3I(-1,-1,1),
            new Vector3I(-1,1,0), new Vector3I(-1,0,0), new Vector3I(-1,-1,0),
            new Vector3I(-1,1,-1), new Vector3I(-1,0,-1), new Vector3I(-1,-1,-1),
            // Side 8 (priority 2)
            new Vector3I(0,1,1), new Vector3I(0,0,1), new Vector3I(0,-1,1),
            new Vector3I(0,1,0), new Vector3I(0,-1,0),
            new Vector3I(0,1,-1), new Vector3I(0,0,-1), new Vector3I(0,-1,-1),
            // Backward 9 (priority 3)
            new Vector3I(1,1,1), new Vector3I(1,0,1), new Vector3I(1,-1,1),
            new Vector3I(1,1,0), new Vector3I(1,0,0), new Vector3I(1,-1,0),
            new Vector3I(1,1,-1), new Vector3I(1,0,-1), new Vector3I(1,-1,-1)
        };

        public AStarPathfinder()
        {
            config = IAISession.Instance?.GetConfig()?.Pathfinding;
            directFallback = new DirectPathfinder();

            // Pre-allocate node pool
            for (int i = 0; i < nodePool.Length; i++)
            {
                nodePool[i] = new AStarNode();
            }
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

            // Adaptive grid spacing based on distance
            double gridSpacing;

            if (distance < 100.0) // Short distance
            {
                gridSpacing = config.MinWaypointDistance;
            }
            else if (distance < 500.0) // Medium distance
            {
                // Lerp between min and mid-range spacing
                gridSpacing = MathHelper.Lerp(
                    (float)config.MinWaypointDistance,
                    (float)((config.MinWaypointDistance + config.MaxWaypointDistance) * 0.5),
                    (float)((distance - 100.0) / 400.0)
                );
            }
            else if (distance < 2000.0) // Long distance
            {
                // Lerp between mid-range and max spacing
                gridSpacing = MathHelper.Lerp(
                    (float)((config.MinWaypointDistance + config.MaxWaypointDistance) * 0.5),
                    (float)config.MaxWaypointDistance,
                    (float)((distance - 500.0) / 1500.0)
                );
            }
            else // Very long distance
            {
                gridSpacing = config.MaxWaypointDistance;
            }

            // Estimate nodes (multiply by 1.5 for non-straight paths)
            var estimatedNodes = (int)((distance / gridSpacing) * 1.5);
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
        /// Core A* pathfinding algorithm with deadlock detection and retracing
        /// </summary>
        private List<Vector3D> FindPath(Vector3D start, Vector3D end, PathfindingContext context)
        {
            var startTime = DateTime.UtcNow;

            // Determine grid spacing based on distance
            var distance = Vector3D.Distance(start, end);
            var gridSpacing = CalculateOptimalGridSpacing(distance);

            // Convert to grid coordinates for discrete space
            var gridStart = WorldToGrid(start, gridSpacing);
            var gridEnd = WorldToGrid(end, gridSpacing);

            // Clear reusable data structures
            openSet.Clear();
            closedSet.Clear();
            openQueue.Clear();
            nodePoolIndex = 0;

            // Determine forward direction once for this pathfinding operation
            var forwardDirections = SelectDirectionArray(context);

            // Get start node from pool
            var startNode = GetNodeFromPool();
            if (startNode == null)
            {
                Log.Error("AStarPathfinder: Node pool exhausted at start");
                return null;
            }

            startNode.Position = gridStart;
            startNode.GCost = 0;
            startNode.HCost = Heuristic(gridStart, gridEnd, context);
            startNode.Parent = null;

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
                        stuckCounter = 0;
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
                    return ReconstructPath(current, gridSpacing);
                }

                // Explore neighbors
                GetNeighbors(current.Position, forwardDirections);

                for (int i = 0; i < neighborList.Count; i++)
                {
                    var neighbor = neighborList[i];

                    // Skip if already explored
                    if (closedSet.ContainsKey(neighbor))
                    {
                        // Only allow revisiting if we're stuck and repathing is enabled
                        if (!config.AllowRepathing || stuckCounter < 5)
                            continue;

                        closedSet.Remove(neighbor);
                        Log.Verbose("AStarPathfinder: Retracing to position {0} due to deadlock", neighbor);
                    }

                    var neighborWorld = GridToWorld(neighbor, gridSpacing);

                    // Check if neighbor is traversable
                    if (!IsTraversable(neighborWorld, context))
                        continue;

                    var tentativeGCost = current.GCost + GetMovementCost(current.Position, neighbor, context, forwardDirections);

                    AStarNode neighborNode;
                    if (!openSet.TryGetValue(neighbor, out neighborNode))
                    {
                        neighborNode = GetNodeFromPool();
                        if (neighborNode == null)
                        {
                            Log.Warning("AStarPathfinder: Node pool exhausted");
                            break;
                        }

                        neighborNode.Position = neighbor;
                        neighborNode.GCost = float.MaxValue;
                        neighborNode.HCost = Heuristic(neighbor, gridEnd, context);
                        neighborNode.Parent = null;
                        openSet[neighbor] = neighborNode;
                    }

                    // If this path to neighbor is better
                    if (tentativeGCost < neighborNode.GCost)
                    {
                        neighborNode.GCost = tentativeGCost;
                        neighborNode.Parent = current;

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
        /// Calculate optimal grid spacing based on distance
        /// </summary>
        private double CalculateOptimalGridSpacing(double distance)
        {
            if (distance < 100.0)
                return config.MinWaypointDistance;
            else if (distance < 500.0)
                return MathHelper.Lerp((float)config.MinWaypointDistance, (float)config.MaxWaypointDistance * 0.5f, (float)((distance - 100.0) / 400.0));
            else if (distance < 2000.0)
                return MathHelper.Lerp((float)config.MaxWaypointDistance * 0.5f, (float)config.MaxWaypointDistance, (float)((distance - 500.0) / 1500.0));
            else
                return config.MaxWaypointDistance;
        }

        /// <summary>
        /// Get node from pre-allocated pool
        /// </summary>
        private AStarNode GetNodeFromPool()
        {
            if (nodePoolIndex >= nodePool.Length)
                return null;

            return nodePool[nodePoolIndex++];
        }

        /// <summary>
        /// Select the appropriate pre-computed direction array based on forward direction
        /// </summary>
        private Vector3I[] SelectDirectionArray(PathfindingContext context)
        {
            var controllerForward = context.GetControllerForwardInWorld();
            var localForward = Vector3D.TransformNormal(controllerForward,
                MatrixD.Transpose(context.Controller.WorldMatrix));

            var absForward = Vector3D.Abs(localForward);

            if (absForward.Z > absForward.X && absForward.Z > absForward.Y)
                return localForward.Z > 0 ? DIRECTIONS_FORWARD_Z : DIRECTIONS_BACKWARD_Z;
            else if (absForward.Y > absForward.X)
                return localForward.Y > 0 ? DIRECTIONS_FORWARD_Y : DIRECTIONS_BACKWARD_Y;
            else
                return localForward.X > 0 ? DIRECTIONS_FORWARD_X : DIRECTIONS_BACKWARD_X;
        }

        /// <summary>
        /// Get neighboring grid positions using pre-computed direction array (zero allocations)
        /// </summary>
        private void GetNeighbors(Vector3I position, Vector3I[] directions)
        {
            neighborList.Clear();

            for (int i = 0; i < 26; i++)
            {
                neighborList.Add(position + directions[i]);
            }
        }

        /// <summary>
        /// Heuristic function for A* (estimated cost from current to goal)
        /// </summary>
        private float Heuristic(Vector3I from, Vector3I to, PathfindingContext context)
        {
            var distance = Vector3.Distance(from, to);

            // Add penalty for moving against gravity if in planet gravity
            if (context.IsInPlanetGravity() && context.PlanetCenter.HasValue)
            {
                var fromWorld = GridToWorld(from, config.MinWaypointDistance);
                var toWorld = GridToWorld(to, config.MinWaypointDistance);

                var fromRadius = Vector3D.Distance(fromWorld, context.PlanetCenter.Value);
                var toRadius = Vector3D.Distance(toWorld, context.PlanetCenter.Value);

                if (toRadius > fromRadius)
                {
                    var climbDistance = toRadius - fromRadius;
                    distance += (float)(climbDistance * 0.5);
                }
            }

            return distance;
        }

        /// <summary>
        /// Get actual movement cost between two adjacent grid positions
        /// </summary>
        private float GetMovementCost(Vector3I from, Vector3I to, PathfindingContext context, Vector3I[] directions)
        {
            var fromWorld = GridToWorld(from, config.MinWaypointDistance);
            var toWorld = GridToWorld(to, config.MinWaypointDistance);

            var distance = Vector3D.Distance(fromWorld, toWorld);
            var movementCost = (float)distance;

            // Calculate direction priority for cost adjustment
            var direction = to - from;

            // Find this direction in our array to determine priority
            int priorityZone = 3; // Default to backward
            for (int i = 0; i < 26; i++)
            {
                if (directions[i] == direction)
                {
                    if (i < 9) priorityZone = 1; // Forward
                    else if (i < 17) priorityZone = 2; // Side
                    else priorityZone = 3; // Backward
                    break;
                }
            }

            // Apply cost penalty based on priority zone
            if (priorityZone == 3)
                movementCost *= 1.5f; // 50% penalty for backward
            else if (priorityZone == 2)
                movementCost *= 1.1f; // 10% penalty for sideways

            // Add cost for moving against gravity
            if (context.GravityVector.LengthSquared() > 0.1)
            {
                var worldDirection = toWorld - fromWorld;
                var gravityUp = Vector3D.Normalize(-context.GravityVector);
                var climbFactor = Vector3D.Dot(Vector3D.Normalize(worldDirection), gravityUp);

                if (climbFactor > 0)
                {
                    if (!context.CanClimbInDirection(worldDirection))
                    {
                        movementCost += 1000.0f;
                    }
                    else
                    {
                        movementCost *= (1.0f + (float)climbFactor * 0.5f);
                    }
                }
            }

            // Add cost for diagonal movement
            var axisCount = 0;
            if (direction.X != 0) axisCount++;
            if (direction.Y != 0) axisCount++;
            if (direction.Z != 0) axisCount++;

            if (axisCount == 2)
                movementCost *= 1.414f;
            else if (axisCount == 3)
                movementCost *= 1.732f;

            return movementCost;
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
                    return false;
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

                var halfSize = new Vector3D(config.MinWaypointDistance * 0.5);
                var box = new BoundingBoxD(position - halfSize, position + halfSize);

                entityList.Clear();
                try
                {
                    MyGamePruningStructure.GetTopmostEntitiesInBox(ref box, entityList);

                    foreach (var entity in entityList)
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

            if (entity.EntityId == context.CubeGrid?.EntityId) return false;

            var grid = entity as IMyCubeGrid;
            if (grid != null)
            {
                var connectedGrids = new List<IMyCubeGrid>();
                MyAPIGateway.GridGroups.GetGroup(context.CubeGrid, GridLinkTypeEnum.Mechanical, connectedGrids);

                if (connectedGrids.Contains(grid))
                {
                    return false;
                }

                return true;
            }

            if (entity is MyPlanet) return true;
            if (entity.ToString().Contains("Asteroid")) return true;

            return false;
        }

        /// <summary>
        /// Reconstruct the path from the goal node back to start (reuses pathList)
        /// </summary>
        private List<Vector3D> ReconstructPath(AStarNode goalNode, double waypointDistance)
        {
            pathList.Clear();
            var current = goalNode;

            while (current != null)
            {
                pathList.Add(GridToWorld(current.Position, waypointDistance));
                current = current.Parent;
            }

            pathList.Reverse();
            return pathList;
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
            public float GCost { get; set; }
            public float HCost { get; set; }
            public float FCost => GCost + HCost;
            public AStarNode Parent { get; set; }
            public int CompareTo(AStarNode other)
            {
                return FCost.CompareTo(other.FCost);
            }
        }

        /// <summary>
        /// Simple priority queue for A* open set (optimized min-heap)
        /// </summary>
        private class PriorityQueue<T> where T : IComparable<T>
        {
            private List<T> data;

            public PriorityQueue()
            {
                data = new List<T>(200); // Pre-allocate for typical use
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

            public void Clear()
            {
                data.Clear();
            }
        }
    }
}