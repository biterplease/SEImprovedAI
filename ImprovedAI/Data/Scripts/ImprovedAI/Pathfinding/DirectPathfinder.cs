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
    public class DirectPathfinder : IPathfinder
    {
        public PathfindingManager.Method Method => PathfindingManager.Method.DirectPathfinding;

        private readonly IPathfindingConfig config;

        public DirectPathfinder()
        {
            config = IAISession.Instance?.GetConfig()?.Pathfinding;
        }
        public DirectPathfinder(IPathfindingConfig pathfindingConfig)
        {
            config = pathfindingConfig;
        }

        public bool IsAvailable(PathfindingContext context)
        {
            return config?.AllowDirectPathfinding() == true;
        }

        public int EstimatedComplexity(Vector3D start, Vector3D end)
        {
            return 1; // Lowest complexity
        }

        /// <summary>
        /// Get the next waypoint dynamically (primary method for drone operation)
        /// </summary>
        public Vector3D? GetNextWaypoint(Vector3D currentPosition, Vector3D targetPosition, PathfindingContext context)
        {
            if (config == null)
            {
                Log.Error("DirectPathfinder: Config not loaded");
                return null;
            }

            var distance = Vector3D.Distance(currentPosition, targetPosition);

            // If we're very close, just go directly
            if (distance < context.WaypointDistance)
            {
                return targetPosition;
            }

            // Check if we can reach directly
            if (CanReachDirectly(currentPosition, targetPosition, context))
            {
                // Calculate next waypoint along direct path
                var direction = Vector3D.Normalize(targetPosition - currentPosition);
                var waypointDistance = Math.Min(distance, context.WaypointDistance);
                var waypoint = currentPosition + direction * waypointDistance;

                // Apply gravity correction if needed
                if (config.UsePlanetAwarePathfinding() && context.IsInPlanetGravity())
                {
                    waypoint = ApplyPlanetAwareCorrection(currentPosition, waypoint, targetPosition, context);
                }

                return waypoint;
            }

            // Path is blocked, try repositioning if allowed
            if (config.AllowRepathing())
            {
                return FindClearWaypoint(currentPosition, targetPosition, context);
            }

            Log.Warning("DirectPathfinder: Path blocked and repathing disabled");
            return null;
        }

        /// <summary>
        /// Generate complete path (for debugging only)
        /// </summary>
        public List<Vector3D> CalculatePath(Vector3D start, Vector3D end, PathfindingContext context)
        {
            if (config == null)
            {
                Log.Error("DirectPathfinder: Config not loaded");
                return new List<Vector3D> { start, end };
            }

            var path = new List<Vector3D>();
            var distance = Vector3D.Distance(start, end);
            var steps = Math.Max(1, (int)(distance / context.WaypointDistance));

            // Generate waypoints
            for (int i = 0; i <= steps; i++)
            {
                var t = (float)i / steps;
                var basePos = Vector3D.Lerp(start, end, t);

                // Apply planet-aware correction if needed
                if (config.UsePlanetAwarePathfinding() && context.IsInPlanetGravity())
                {
                    basePos = ApplyPlanetAwareCorrection(start, basePos, end, context);
                }

                path.Add(basePos);
            }

            return path;
        }

        /// <summary>
        /// Check if we can reach the target directly without obstacles
        /// </summary>
        private bool CanReachDirectly(Vector3D start, Vector3D end, PathfindingContext context)
        {
            var diff = end - start;
            var worldMatrix = context.Controller.WorldMatrix;
            MatrixD worldMatrixTransposed;
            MatrixD.Transpose(ref worldMatrix, out worldMatrixTransposed);
            // Check if we have thrust capability in this direction
            if (!context.CanClimbInDirection(ref diff, ref worldMatrixTransposed))
            {
                Log.Verbose("DirectPathfinder: Insufficient thrust to reach target directly");
                return false;
            }

            // Check for obstacles using raycasting (if cameras available)
            if (config.RequireCamerasForPathfinding() && context.CamerasByDirection?.Count > 0)
            {
                if (!IsPathClearViaRaycast(start, end, context))
                {
                    Log.Verbose("DirectPathfinder: Raycast detected obstacle in path");
                    return false;
                }
            }

            // Check for obstacles using sensors (if sensors available)
            if (config.RequireSensorsForPathfinding() && context.SensorInfos?.Count > 0)
            {
                if (HasObstacleInSensorRange(start, end, context))
                {
                    Log.Verbose("DirectPathfinder: Sensor detected obstacle in path");
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Use camera raycasting to detect obstacles
        /// </summary>
        private bool IsPathClearViaRaycast(Vector3D start, Vector3D end, PathfindingContext context)
        {
            var direction = end - start;
            var distance = direction.Length();

            // Check if we can raycast in this direction
            if (!context.CanRaycastInDirection(direction))
            {
                // No camera facing this direction, assume clear
                return true;
            }

            // Scale raycast distance based on trip length (shorter trips = shorter raycasts)
            var maxRaycastDistance = Math.Min(
                distance,
                Math.Min(config.MaxSimulatedCameraRaycastMeters(), distance * 0.5)
            );

            // Perform raycast
            var ray = new LineD(start, start + Vector3D.Normalize(direction) * maxRaycastDistance);
            var results = new List<MyLineSegmentOverlapResult<MyEntity>>();

            try
            {
                MyGamePruningStructure.GetTopmostEntitiesOverlappingRay(ref ray, results);

                // Filter out non-obstacle entities
                foreach (var result in results)
                {
                    if (IsObstacleEntity(result.Element, context))
                    {
                        Log.Verbose("DirectPathfinder: Raycast hit obstacle at distance {0}m",
                            Vector3D.Distance(start, result.Element.PositionComp.GetPosition()));
                        return false;
                    }
                }
            }
            catch (Exception ex)
            {
                Log.Warning("DirectPathfinder: Raycast exception: {0}", ex.Message);
                return true; // Assume clear on error
            }

            return true;
        }

        /// <summary>
        /// Use sensor box detection to find obstacles
        /// </summary>
        private bool HasObstacleInSensorRange(Vector3D start, Vector3D end, PathfindingContext context)
        {
            var direction = Vector3D.Normalize(end - start);
            var distance = Vector3D.Distance(start, end);

            foreach (var sensorInfo in context.SensorInfos)
            {
                // Check points along the path within sensor range
                var checkDistance = Math.Min(distance, sensorInfo.MaxRange);
                var checkPoint = start + direction * (checkDistance * 0.5);

                // Create bounding box around the check point
                var halfSize = new Vector3D(sensorInfo.MaxRange * 0.5);
                var box = new BoundingBoxD(checkPoint - halfSize, checkPoint + halfSize);

                var entities = new List<MyEntity>();
                try
                {
                    MyGamePruningStructure.GetTopmostEntitiesInBox(ref box, entities);

                    foreach (var entity in entities)
                    {
                        if (IsObstacleEntity(entity, context))
                        {
                            Log.Verbose("DirectPathfinder: Sensor detected obstacle");
                            return true;
                        }
                    }
                }
                catch (Exception ex)
                {
                    Log.Warning("DirectPathfinder: Sensor detection exception: {0}", ex.Message);
                }
            }

            return false;
        }

        /// <summary>
        /// Determine if an entity is an obstacle we need to avoid
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
                // Check if it's mechanically connected to us (pistons, rotors, etc)
                var connectedGrids = new List<IMyCubeGrid>();
                MyAPIGateway.GridGroups.GetGroup(context.CubeGrid, GridLinkTypeEnum.Mechanical, connectedGrids);

                if (connectedGrids.Contains(grid))
                {
                    return false; // It's part of our ship
                }

                // It's another grid - it's an obstacle
                return true;
            }

            // Check for planets and asteroids
            if (entity is MyPlanet) return true;
            if (entity.ToString().Contains("Asteroid")) return true;

            return false;
        }

        /// <summary>
        /// Apply planet-aware arc pathing when in gravity
        /// </summary>
        private Vector3D ApplyPlanetAwareCorrection(Vector3D start, Vector3D waypoint, Vector3D target, PathfindingContext context)
        {
            if (!context.PlanetCenter.HasValue)
                return waypoint;

            var planetCenter = context.PlanetCenter.Value;
            var distance = Vector3D.Distance(start, target);

            // For short distances, don't apply arc pathing (within 500m)
            if (distance < 500.0)
            {
                return EnsureSafeAltitude(waypoint, context);
            }

            // Calculate arc path around planet
            // Get vectors from planet center
            var startVector = start - planetCenter;
            var targetVector = target - planetCenter;
            var waypointVector = waypoint - planetCenter;

            // Calculate the angle between start and target
            var angle = Math.Acos(MathHelper.Clamp(
                Vector3D.Dot(Vector3D.Normalize(startVector), Vector3D.Normalize(targetVector)),
                -1.0, 1.0));

            // Determine what percentage along the path this waypoint is
            var progress = Vector3D.Distance(start, waypoint) / distance;

            // Calculate position along the arc
            var currentAngle = angle * progress;
            var rotation = QuaternionD.CreateFromAxisAngle(
                Vector3D.Cross(startVector, targetVector),
                currentAngle);

            var arcVector = Vector3D.Transform(startVector, rotation);

            // Maintain altitude (lerp between start and target altitudes)
            var startRadius = startVector.Length();
            var targetRadius = targetVector.Length();
            var desiredRadius = MathHelper.Lerp(startRadius, targetRadius, (float)progress);

            // Add altitude buffer
            var minSafeRadius = context.PlanetRadius + config.MinAltitudeBuffer();
            desiredRadius = Math.Max(desiredRadius, minSafeRadius);

            // Calculate final waypoint position
            var correctedWaypoint = planetCenter + Vector3D.Normalize(arcVector) * desiredRadius;

            return correctedWaypoint;
        }

        /// <summary>
        /// Ensure waypoint maintains safe altitude above planet surface
        /// </summary>
        private Vector3D EnsureSafeAltitude(Vector3D waypoint, PathfindingContext context)
        {
            if (!context.PlanetCenter.HasValue)
                return waypoint;

            var planetCenter = context.PlanetCenter.Value;
            var waypointVector = waypoint - planetCenter;
            var currentRadius = waypointVector.Length();
            var minSafeRadius = context.PlanetRadius + config.MinAltitudeBuffer();

            if (currentRadius < minSafeRadius)
            {
                // Lift waypoint to safe altitude
                var correctedWaypoint = planetCenter + Vector3D.Normalize(waypointVector) * minSafeRadius;
                Log.Verbose("DirectPathfinder: Lifted waypoint to safe altitude ({0}m above surface)",
                    config.MinAltitudeBuffer());
                return correctedWaypoint;
            }

            return waypoint;
        }

        /// <summary>
        /// Find a clear waypoint when direct path is blocked (repositioning)
        /// </summary>
        private Vector3D? FindClearWaypoint(Vector3D currentPosition, Vector3D targetPosition, PathfindingContext context)
        {
            if (!config.AllowRepathing())
            {
                Log.Warning("DirectPathfinder: Repathing disabled, cannot find alternate waypoint");
                return null;
            }

            var directionToTarget = Vector3D.Normalize(targetPosition - currentPosition);
            var perpendicular = CalculatePerpendicularDirections(directionToTarget, context);

            // Try repositioning in different directions
            for (int attempt = 1; attempt <= config.MaxRepositionAttempts(); attempt++)
            {
                var repositionDistance = attempt * context.WaypointDistance * 0.5; // Half waypoint distance per attempt

                // Try each perpendicular direction
                foreach (var perpDir in perpendicular)
                {
                    var testPosition = currentPosition + perpDir * repositionDistance;

                    // Ensure safe altitude
                    if (config.UsePlanetAwarePathfinding() && context.IsInPlanetGravity())
                    {
                        testPosition = EnsureSafeAltitude(testPosition, context);
                    }

                    // Check if this position has a clear path to target
                    if (CanReachDirectly(testPosition, targetPosition, context))
                    {
                        Log.Info("DirectPathfinder: Found clear waypoint after {0} reposition attempts", attempt);
                        return testPosition;
                    }
                }

                Log.Verbose("DirectPathfinder: Reposition attempt {0} failed, trying further", attempt);
            }

            Log.Warning("DirectPathfinder: Failed to find clear waypoint after {0} attempts",
                config.MaxRepositionAttempts());
            return null;
        }

        /// <summary>
        /// Calculate perpendicular directions for repositioning
        /// Prioritizes: Up, Down, Left, Right (relative to ship orientation)
        /// </summary>
        private List<Vector3D> CalculatePerpendicularDirections(Vector3D forwardDirection, PathfindingContext context)
        {
            var directions = new List<Vector3D>();

            // Get ship's up and right vectors
            var shipUp = Vector3D.Transform(Base6Directions.GetVector(Base6Directions.Direction.Up),
                context.Controller.WorldMatrix);
            var shipRight = Vector3D.Transform(Base6Directions.GetVector(Base6Directions.Direction.Right),
                context.Controller.WorldMatrix);

            // In gravity, prefer moving up first
            if (context.GravityVector.LengthSquared() > 0.1)
            {
                var gravityUp = Vector3D.Normalize(-context.GravityVector);
                directions.Add(gravityUp);           // Up (away from planet)
                directions.Add(-gravityUp);          // Down (toward planet)
            }
            else
            {
                directions.Add(shipUp);              // Ship up
                directions.Add(-shipUp);             // Ship down
            }

            // Add horizontal directions
            directions.Add(shipRight);               // Right
            directions.Add(-shipRight);              // Left

            // Remove any directions that are too parallel to forward
            var filtered = directions.Where(d =>
            {
                var dot = Math.Abs(Vector3D.Dot(Vector3D.Normalize(d), forwardDirection));
                return dot < 0.7; // Less than 45 degrees
            }).ToList();

            return filtered.Count > 0 ? filtered : directions;
        }
    }
}