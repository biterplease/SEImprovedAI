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
        public PathfindingManager.Method Method
        {
            get { return PathfindingManager.Method.AStar; }
        }

        private readonly IPathfindingConfig config;
        private readonly DirectPathfinder directFallback;

        // Pre-allocated data structures (reused across pathfinding operations)
        private readonly Dictionary<Vector3I, AStarNode> openSet;
        private readonly Dictionary<Vector3I, AStarNode> closedSet;
        private readonly PriorityQueue<AStarNode> openQueue;

        // Pre-allocated node pool to avoid allocations
        private readonly AStarNode[] nodePool;
        private int nodePoolIndex;

        // Reusable lists to avoid allocations
        private readonly List<Vector3I> neighborList;
        private readonly List<Vector3D> pathList;
        private readonly List<MyEntity> entityList;
        private readonly List<IMyCubeGrid> connectedGridsBuffer;

        // Cached calculations
        private Vector3D _tempWorldPos;
        private Vector3D _tempDirection;
        private Vector3D _tempGravityUp;
        private Vector3D _tempNormalized;
        private Vector3D _tempHalfSize;
        private Vector3D _tempBoxMin;
        private Vector3D _tempBoxMax;
        private MatrixD _cachedWorldMatrix;
        private MatrixD _cachedWorldMatrixTransposed;
        private double _cachedDistance;

        // Static pre-computed direction arrays for all 6 forward orientations
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

        private static readonly Vector3I[] DIRECTIONS_BACKWARD_Z = new Vector3I[26]
        {
            new Vector3I(1,1,-1), new Vector3I(0,1,-1), new Vector3I(-1,1,-1),
            new Vector3I(1,0,-1), new Vector3I(0,0,-1), new Vector3I(-1,0,-1),
            new Vector3I(1,-1,-1), new Vector3I(0,-1,-1), new Vector3I(-1,-1,-1),
            new Vector3I(1,1,0), new Vector3I(0,1,0), new Vector3I(-1,1,0),
            new Vector3I(1,0,0), new Vector3I(-1,0,0),
            new Vector3I(1,-1,0), new Vector3I(0,-1,0), new Vector3I(-1,-1,0),
            new Vector3I(1,1,1), new Vector3I(0,1,1), new Vector3I(-1,1,1),
            new Vector3I(1,0,1), new Vector3I(0,0,1), new Vector3I(-1,0,1),
            new Vector3I(1,-1,1), new Vector3I(0,-1,1), new Vector3I(-1,-1,1)
        };

        private static readonly Vector3I[] DIRECTIONS_FORWARD_Y = new Vector3I[26]
        {
            new Vector3I(1,1,1), new Vector3I(0,1,1), new Vector3I(-1,1,1),
            new Vector3I(1,1,0), new Vector3I(0,1,0), new Vector3I(-1,1,0),
            new Vector3I(1,1,-1), new Vector3I(0,1,-1), new Vector3I(-1,1,-1),
            new Vector3I(1,0,1), new Vector3I(0,0,1), new Vector3I(-1,0,1),
            new Vector3I(1,0,0), new Vector3I(-1,0,0),
            new Vector3I(1,0,-1), new Vector3I(0,0,-1), new Vector3I(-1,0,-1),
            new Vector3I(1,-1,1), new Vector3I(0,-1,1), new Vector3I(-1,-1,1),
            new Vector3I(1,-1,0), new Vector3I(0,-1,0), new Vector3I(-1,-1,0),
            new Vector3I(1,-1,-1), new Vector3I(0,-1,-1), new Vector3I(-1,-1,-1)
        };

        private static readonly Vector3I[] DIRECTIONS_BACKWARD_Y = new Vector3I[26]
        {
            new Vector3I(1,-1,1), new Vector3I(0,-1,1), new Vector3I(-1,-1,1),
            new Vector3I(1,-1,0), new Vector3I(0,-1,0), new Vector3I(-1,-1,0),
            new Vector3I(1,-1,-1), new Vector3I(0,-1,-1), new Vector3I(-1,-1,-1),
            new Vector3I(1,0,1), new Vector3I(0,0,1), new Vector3I(-1,0,1),
            new Vector3I(1,0,0), new Vector3I(-1,0,0),
            new Vector3I(1,0,-1), new Vector3I(0,0,-1), new Vector3I(-1,0,-1),
            new Vector3I(1,1,1), new Vector3I(0,1,1), new Vector3I(-1,1,1),
            new Vector3I(1,1,0), new Vector3I(0,1,0), new Vector3I(-1,1,0),
            new Vector3I(1,1,-1), new Vector3I(0,1,-1), new Vector3I(-1,1,-1)
        };

        private static readonly Vector3I[] DIRECTIONS_FORWARD_X = new Vector3I[26]
        {
            new Vector3I(1,1,1), new Vector3I(1,0,1), new Vector3I(1,-1,1),
            new Vector3I(1,1,0), new Vector3I(1,0,0), new Vector3I(1,-1,0),
            new Vector3I(1,1,-1), new Vector3I(1,0,-1), new Vector3I(1,-1,-1),
            new Vector3I(0,1,1), new Vector3I(0,0,1), new Vector3I(0,-1,1),
            new Vector3I(0,1,0), new Vector3I(0,-1,0),
            new Vector3I(0,1,-1), new Vector3I(0,0,-1), new Vector3I(0,-1,-1),
            new Vector3I(-1,1,1), new Vector3I(-1,0,1), new Vector3I(-1,-1,1),
            new Vector3I(-1,1,0), new Vector3I(-1,0,0), new Vector3I(-1,-1,0),
            new Vector3I(-1,1,-1), new Vector3I(-1,0,-1), new Vector3I(-1,-1,-1)
        };

        private static readonly Vector3I[] DIRECTIONS_BACKWARD_X = new Vector3I[26]
        {
            new Vector3I(-1,1,1), new Vector3I(-1,0,1), new Vector3I(-1,-1,1),
            new Vector3I(-1,1,0), new Vector3I(-1,0,0), new Vector3I(-1,-1,0),
            new Vector3I(-1,1,-1), new Vector3I(-1,0,-1), new Vector3I(-1,-1,-1),
            new Vector3I(0,1,1), new Vector3I(0,0,1), new Vector3I(0,-1,1),
            new Vector3I(0,1,0), new Vector3I(0,-1,0),
            new Vector3I(0,1,-1), new Vector3I(0,0,-1), new Vector3I(0,-1,-1),
            new Vector3I(1,1,1), new Vector3I(1,0,1), new Vector3I(1,-1,1),
            new Vector3I(1,1,0), new Vector3I(1,0,0), new Vector3I(1,-1,0),
            new Vector3I(1,1,-1), new Vector3I(1,0,-1), new Vector3I(1,-1,-1)
        };

        public AStarPathfinder()
        {
            config = IAISession.Instance?.GetConfig()?.Pathfinding;
            directFallback = new DirectPathfinder();

            openSet = new Dictionary<Vector3I, AStarNode>();
            closedSet = new Dictionary<Vector3I, AStarNode>();
            openQueue = new PriorityQueue<AStarNode>();
            nodePool = new AStarNode[1000];
            nodePoolIndex = 0;
            neighborList = new List<Vector3I>(26);
            pathList = new List<Vector3D>();
            entityList = new List<MyEntity>(10);
            connectedGridsBuffer = new List<IMyCubeGrid>(10);

            for (int i = 0; i < nodePool.Length; i++)
            {
                nodePool[i] = new AStarNode();
            }
        }

        public AStarPathfinder(IPathfindingConfig pathfindingConfig)
        {
            config = pathfindingConfig;
            directFallback = new DirectPathfinder(pathfindingConfig);

            openSet = new Dictionary<Vector3I, AStarNode>();
            closedSet = new Dictionary<Vector3I, AStarNode>();
            openQueue = new PriorityQueue<AStarNode>();
            nodePool = new AStarNode[1000];
            nodePoolIndex = 0;
            neighborList = new List<Vector3I>(26);
            pathList = new List<Vector3D>();
            entityList = new List<MyEntity>(10);
            connectedGridsBuffer = new List<IMyCubeGrid>(10);

            for (int i = 0; i < nodePool.Length; i++)
            {
                nodePool[i] = new AStarNode();
            }
        }

        public bool IsAvailable(ref PathfindingContext context)
        {
            if (config == null || !config.AllowAStar())
                return false;

            bool hasSensors = config.RequireSensorsForPathfinding() &&
                             context.SensorInfos != null && context.SensorInfos.Count > 0;
            bool hasCameras = config.RequireCamerasForPathfinding() &&
                             context.CamerasByDirection != null && context.CamerasByDirection.Count > 0;

            return hasSensors || hasCameras;
        }

        public int EstimatedComplexity(ref Vector3D start, ref Vector3D end)
        {
            Vector3D.Distance(ref start, ref end, out _cachedDistance);
            double gridSpacing = CalculateOptimalGridSpacing(_cachedDistance);
            int estimatedNodes = (int)((_cachedDistance / gridSpacing) * 1.5);
            return Math.Min(estimatedNodes, config.MaxPathNodes());
        }

        public bool GetNextWaypoint(ref PathfindingContext context, ref Vector3D currentPosition,
            ref Vector3D targetPosition, out Vector3D result)
        {
            result = default(Vector3D);

            if (config == null)
            {
                Log.Error("AStarPathfinder: Config not loaded");
                return false;
            }

            Vector3D.Distance(ref currentPosition, ref targetPosition, out _cachedDistance);

            // For very short distances, use direct pathfinding
            if (_cachedDistance < context.WaypointDistance * 2)
            {
                return directFallback.GetNextWaypoint(ref context, ref currentPosition,
                    ref targetPosition, out result);
            }

            // Cache world matrix
            if (context.Controller != null)
            {
                _cachedWorldMatrix = context.Controller.WorldMatrix;
                MatrixD.Transpose(ref _cachedWorldMatrix, out _cachedWorldMatrixTransposed);
            }

            // Try to find a path using A*
            if (FindPath(ref currentPosition, ref targetPosition, ref context, pathList))
            {
                if (pathList.Count > 1)
                {
                    result = pathList[1];
                    return true;
                }
            }

            // A* failed, fallback to direct pathfinding if repathing is allowed
            if (config.AllowRepathing())
            {
                Log.Warning("AStarPathfinder: Failed to find path, falling back to direct pathfinding");
                return directFallback.GetNextWaypoint(ref context, ref currentPosition,
                    ref targetPosition, out result);
            }

            Log.Warning("AStarPathfinder: Failed to find path and repathing disabled");
            return false;
        }

        public bool CalculatePath(
            ref PathfindingContext context,
            ref Vector3D start,
            ref Vector3D end,
            List<Vector3D> pathWaypoints)
        {
            if (pathWaypoints == null)
                throw new ArgumentNullException("pathWaypoints");

            if (config == null)
            {
                Log.Error("AStarPathfinder: Config not loaded");
                pathWaypoints.Clear();
                pathWaypoints.Add(start);
                pathWaypoints.Add(end);
                return false;
            }

            // Cache world matrix
            if (context.Controller != null)
            {
                _cachedWorldMatrix = context.Controller.WorldMatrix;
                MatrixD.Transpose(ref _cachedWorldMatrix, out _cachedWorldMatrixTransposed);
            }

            if (FindPath(ref start, ref end, ref context, pathWaypoints))
            {
                return true;
            }

            // Fallback to direct pathfinding
            if (config.AllowRepathing())
            {
                Log.Warning("AStarPathfinder: A* failed, using direct pathfinding fallback");
                return directFallback.CalculatePath(ref context, ref start, ref end, pathWaypoints);
            }

            Log.Error("AStarPathfinder: Failed to find path and repathing disabled");
            pathWaypoints.Clear();
            return false;
        }

        private bool FindPath(ref Vector3D start, ref Vector3D end, ref PathfindingContext context,
            List<Vector3D> outputPath)
        {
            DateTime startTime = DateTime.UtcNow;

            Vector3D.Distance(ref start, ref end, out _cachedDistance);
            double gridSpacing = CalculateOptimalGridSpacing(_cachedDistance);

            Vector3I gridStart = WorldToGrid(ref start, gridSpacing);
            Vector3I gridEnd = WorldToGrid(ref end, gridSpacing);

            // Clear reusable data structures
            openSet.Clear();
            closedSet.Clear();
            openQueue.Clear();
            nodePoolIndex = 0;

            Vector3I[] forwardDirections = SelectDirectionArray(ref context);

            AStarNode startNode = GetNodeFromPool();
            if (startNode == null)
            {
                Log.Error("AStarPathfinder: Node pool exhausted at start");
                return false;
            }

            startNode.Position = gridStart;
            startNode.GCost = 0;
            startNode.HCost = Heuristic(ref gridStart, ref gridEnd, ref context, gridSpacing);
            startNode.Parent = null;

            openSet[gridStart] = startNode;
            openQueue.Enqueue(startNode);

            int nodesExplored = 0;
            int stuckCounter = 0;
            Vector3I lastPosition = default(Vector3I);
            bool hasLastPosition = false;

            while (openQueue.Count > 0)
            {
                if ((DateTime.UtcNow - startTime) > config.MaxPathfindingTime())
                {
                    Log.Warning("AStarPathfinder: Timeout after exploring {0} nodes", nodesExplored);
                    return false;
                }

                if (nodesExplored >= config.MaxPathNodes())
                {
                    Log.Warning("AStarPathfinder: Max nodes reached ({0})", config.MaxPathNodes());
                    return false;
                }

                AStarNode current = openQueue.Dequeue();
                openSet.Remove(current.Position);
                closedSet[current.Position] = current;
                nodesExplored++;

                // Deadlock detection
                if (hasLastPosition && Vector3.Distance(current.Position, lastPosition) < 2)
                {
                    stuckCounter++;
                    if (stuckCounter > 10)
                    {
                        Log.Verbose("AStarPathfinder: Possible deadlock detected");
                        stuckCounter = 0;
                    }
                }
                else
                {
                    stuckCounter = 0;
                }
                lastPosition = current.Position;
                hasLastPosition = true;

                if (Vector3.Distance(current.Position, gridEnd) <= 1)
                {
                    Log.Info("AStarPathfinder: Found path with {0} nodes in {1:F1}ms",
                        nodesExplored, (DateTime.UtcNow - startTime).TotalMilliseconds);
                    ReconstructPath(current, gridSpacing, outputPath);
                    return true;
                }

                GetNeighbors(ref current.Position, forwardDirections);

                for (int i = 0; i < neighborList.Count; i++)
                {
                    Vector3I neighbor = neighborList[i];

                    if (closedSet.ContainsKey(neighbor))
                    {
                        if (!config.AllowRepathing() || stuckCounter < 5)
                            continue;

                        closedSet.Remove(neighbor);
                        Log.Verbose("AStarPathfinder: Retracing to position {0}", neighbor);
                    }

                    GridToWorld(ref neighbor, gridSpacing, out _tempWorldPos);

                    if (!IsTraversable(ref _tempWorldPos, ref context))
                        continue;

                    float tentativeGCost = current.GCost +
                        GetMovementCost(ref current.Position, ref neighbor, ref context, forwardDirections, gridSpacing);

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
                        neighborNode.HCost = Heuristic(ref neighbor, ref gridEnd, ref context, gridSpacing);
                        neighborNode.Parent = null;
                        openSet[neighbor] = neighborNode;
                    }

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
            return false;
        }

        private double CalculateOptimalGridSpacing(double distance)
        {
            if (distance < 100.0)
                return config.MinWaypointDistance();
            else if (distance < 500.0)
                return MathHelper.Lerp((float)config.MinWaypointDistance(),
                    (float)config.MaxWaypointDistance() * 0.5f, (float)((distance - 100.0) / 400.0));
            else if (distance < 2000.0)
                return MathHelper.Lerp((float)config.MaxWaypointDistance() * 0.5f,
                    (float)config.MaxWaypointDistance(), (float)((distance - 500.0) / 1500.0));
            else
                return config.MaxWaypointDistance();
        }

        private AStarNode GetNodeFromPool()
        {
            if (nodePoolIndex >= nodePool.Length)
                return null;

            return nodePool[nodePoolIndex++];
        }

        private Vector3I[] SelectDirectionArray(ref PathfindingContext context)
        {
            context.GetControllerForwardInWorld(ref _cachedWorldMatrix, out _tempDirection);
            Vector3D.TransformNormal(ref _tempDirection, ref _cachedWorldMatrixTransposed, out _tempDirection);

            Vector3D absForward = Vector3D.Abs(_tempDirection);

            if (absForward.Z > absForward.X && absForward.Z > absForward.Y)
                return _tempDirection.Z > 0 ? DIRECTIONS_FORWARD_Z : DIRECTIONS_BACKWARD_Z;
            else if (absForward.Y > absForward.X)
                return _tempDirection.Y > 0 ? DIRECTIONS_FORWARD_Y : DIRECTIONS_BACKWARD_Y;
            else
                return _tempDirection.X > 0 ? DIRECTIONS_FORWARD_X : DIRECTIONS_BACKWARD_X;
        }

        private void GetNeighbors(ref Vector3I position, Vector3I[] directions)
        {
            neighborList.Clear();

            for (int i = 0; i < 26; i++)
            {
                neighborList.Add(position + directions[i]);
            }
        }

        private float Heuristic(ref Vector3I from, ref Vector3I to, ref PathfindingContext context,
            double gridSpacing)
        {
            float distance = Vector3.Distance(from, to);

            if (context.IsInPlanetGravity() && context.PlanetCenter.HasValue)
            {
                GridToWorld(ref from, gridSpacing, out _tempWorldPos);
                Vector3D toWorld;
                GridToWorld(ref to, gridSpacing, out toWorld);

                double fromRadius = Vector3D.Distance(_tempWorldPos, context.PlanetCenter.Value);
                double toRadius = Vector3D.Distance(toWorld, context.PlanetCenter.Value);

                if (toRadius > fromRadius)
                {
                    double climbDistance = toRadius - fromRadius;
                    distance += (float)(climbDistance * 0.5);
                }
            }

            return distance;
        }

        private float GetMovementCost(ref Vector3I from, ref Vector3I to, ref PathfindingContext context,
            Vector3I[] directions, double gridSpacing)
        {
            GridToWorld(ref from, gridSpacing, out _tempWorldPos);
            Vector3D toWorld;
            GridToWorld(ref to, gridSpacing, out toWorld);

            double distance = Vector3D.Distance(_tempWorldPos, toWorld);
            float movementCost = (float)distance;

            Vector3I direction = to - from;

            int priorityZone = 3;
            for (int i = 0; i < 26; i++)
            {
                if (directions[i] == direction)
                {
                    if (i < 9) priorityZone = 1;
                    else if (i < 17) priorityZone = 2;
                    else priorityZone = 3;
                    break;
                }
            }

            if (priorityZone == 3)
                movementCost *= 1.5f;
            else if (priorityZone == 2)
                movementCost *= 1.1f;

            double gravityLengthSq = context.GravityVector.LengthSquared();
            if (gravityLengthSq > 0.1)
            {
                Vector3D.Subtract(ref toWorld, ref _tempWorldPos, out _tempDirection);
                Vector3D.Negate(ref context.GravityVector, out _tempGravityUp);
                Vector3D.Normalize(ref _tempGravityUp, out _tempGravityUp);
                Vector3D.Normalize(ref _tempDirection, out _tempNormalized);

                double climbFactor = Vector3D.Dot(_tempNormalized, _tempGravityUp);

                if (climbFactor > 0)
                {
                    if (!context.CanClimbInDirection(ref _tempDirection, ref _cachedWorldMatrixTransposed))
                    {
                        movementCost += 1000.0f;
                    }
                    else
                    {
                        movementCost *= (1.0f + (float)climbFactor * 0.5f);
                    }
                }
            }

            int axisCount = 0;
            if (direction.X != 0) axisCount++;
            if (direction.Y != 0) axisCount++;
            if (direction.Z != 0) axisCount++;

            if (axisCount == 2)
                movementCost *= 1.414f;
            else if (axisCount == 3)
                movementCost *= 1.732f;

            return movementCost;
        }

        private bool IsTraversable(ref Vector3D position, ref PathfindingContext context)
        {
            if (config.UsePlanetAwarePathfinding() && context.IsInPlanetGravity() &&
                context.PlanetCenter.HasValue)
            {
                double altitude = Vector3D.Distance(position, context.PlanetCenter.Value) - context.PlanetRadius;
                if (altitude < config.MinAltitudeBuffer())
                {
                    return false;
                }
            }

            if (config.RequireSensorsForPathfinding() &&
                context.SensorInfos != null && context.SensorInfos.Count > 0)
            {
                if (HasObstacleAtPosition(ref position, ref context))
                {
                    return false;
                }
            }

            return true;
        }

        private bool HasObstacleAtPosition(ref Vector3D position, ref PathfindingContext context)
        {
            for (int s = 0; s < context.SensorInfos.Count; s++)
            {
                PathfindingContext.SensorInfo sensorInfo = context.SensorInfos[s];
                double distanceToSensor = Vector3D.Distance(position, sensorInfo.Position);
                if (distanceToSensor > sensorInfo.MaxRange)
                    continue;

                double halfSize = config.MinWaypointDistance() * 0.5;
                _tempHalfSize = new Vector3D(halfSize, halfSize, halfSize);

                Vector3D.Subtract(ref position, ref _tempHalfSize, out _tempBoxMin);
                Vector3D.Add(ref position, ref _tempHalfSize, out _tempBoxMax);

                BoundingBoxD box = new BoundingBoxD(_tempBoxMin, _tempBoxMax);

                entityList.Clear();
                try
                {
                    MyGamePruningStructure.GetTopmostEntitiesInBox(ref box, entityList);

                    for (int i = 0; i < entityList.Count; i++)
                    {
                        if (IsObstacleEntity(entityList[i], ref context))
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

        private bool IsObstacleEntity(IMyEntity entity, ref PathfindingContext context)
        {
            if (entity == null) return false;
            if (context.CubeGrid != null && entity.EntityId == context.CubeGrid.EntityId) return false;

            IMyCubeGrid grid = entity as IMyCubeGrid;
            if (grid != null)
            {
                if (context.CubeGrid != null)
                {
                    connectedGridsBuffer.Clear();
                    MyAPIGateway.GridGroups.GetGroup(context.CubeGrid, GridLinkTypeEnum.Mechanical,
                        connectedGridsBuffer);

                    if (connectedGridsBuffer.Contains(grid))
                    {
                        return false;
                    }
                }

                return true;
            }

            if (entity is MyPlanet) return true;
            if (entity.ToString().Contains("Asteroid")) return true;

            return false;
        }

        private void ReconstructPath(AStarNode goalNode, double waypointDistance, List<Vector3D> outputPath)
        {
            outputPath.Clear();
            AStarNode current = goalNode;

            while (current != null)
            {
                GridToWorld(ref current.Position, waypointDistance, out _tempWorldPos);
                outputPath.Add(_tempWorldPos);
                current = current.Parent;
            }

            outputPath.Reverse();
        }

        private Vector3I WorldToGrid(ref Vector3D world, double gridSize)
        {
            return new Vector3I(
                (int)Math.Round(world.X / gridSize),
                (int)Math.Round(world.Y / gridSize),
                (int)Math.Round(world.Z / gridSize)
            );
        }

        private void GridToWorld(ref Vector3I grid, double gridSize, out Vector3D result)
        {
            result = new Vector3D(
                grid.X * gridSize,
                grid.Y * gridSize,
                grid.Z * gridSize
            );
        }

        private class AStarNode : IComparable<AStarNode>
        {
            public Vector3I Position;
            public float GCost;
            public float HCost;
            public float FCost { get { return GCost + HCost; } }
            public AStarNode Parent;

            public int CompareTo(AStarNode other)
            {
                return FCost.CompareTo(other.FCost);
            }
        }

        private class PriorityQueue<T> where T : IComparable<T>
        {
            private List<T> data;

            public PriorityQueue()
            {
                data = new List<T>(200);
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

            public int Count
            {
                get { return data.Count; }
            }

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