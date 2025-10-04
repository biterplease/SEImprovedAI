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
        public PathfindingManager.Method Method
        {
            get { return PathfindingManager.Method.DirectPathfinding; }
        }

        private readonly IPathfindingConfig config;

        // Reusable buffers to avoid allocations
        private List<MyLineSegmentOverlapResult<MyEntity>> _raycastResults;
        private List<MyEntity> _sensorResults;
        private List<IMyCubeGrid> _connectedGridsBuffer;
        private List<Vector3D> _perpendicularDirections;

        // Cached temporary calculations
        private Vector3D _tempDirection;
        private Vector3D _tempOffset;
        private Vector3D _tempWaypoint;
        private Vector3D _tempVector;
        private MatrixD _cachedWorldMatrix;
        private MatrixD _cachedWorldMatrixTransposed;
        private MatrixD _cachedRotation;

        public DirectPathfinder()
        {
            config = IAISession.Instance?.GetConfig()?.Pathfinding;
            InitializeBuffers();
        }

        public DirectPathfinder(IPathfindingConfig pathfindingConfig)
        {
            config = pathfindingConfig;
            InitializeBuffers();
        }

        private void InitializeBuffers()
        {
            _raycastResults = new List<MyLineSegmentOverlapResult<MyEntity>>(10);
            _sensorResults = new List<MyEntity>(10);
            _connectedGridsBuffer = new List<IMyCubeGrid>(10);
            _perpendicularDirections = new List<Vector3D>(6);
        }

        public bool IsAvailable(ref PathfindingContext context)
        {
            return config != null && config.AllowDirectPathfinding();
        }

        public int EstimatedComplexity(ref Vector3D start, ref Vector3D end)
        {
            return 1; // Lowest complexity
        }

        /// <summary>
        /// Get the next waypoint dynamically (primary method for drone operation)
        /// </summary>
        public bool GetNextWaypoint(ref PathfindingContext context, ref Vector3D currentPosition,
            ref Vector3D targetPosition, out Vector3D result)
        {
            result = default(Vector3D);

            if (config == null)
            {
                Log.Error("DirectPathfinder: Config not loaded");
                return false;
            }

            double distance = Vector3D.Distance(currentPosition, targetPosition);

            // If we're very close, just go directly
            if (distance < context.WaypointDistance)
            {
                result = targetPosition;
                return true;
            }

            // Cache world matrix for this call
            if (context.Controller != null)
            {
                _cachedWorldMatrix = context.Controller.WorldMatrix;
                MatrixD.Transpose(ref _cachedWorldMatrix, out _cachedWorldMatrixTransposed);
            }

            // Check if we can reach directly
            if (CanReachDirectly(ref currentPosition, ref targetPosition, ref context))
            {
                // Calculate next waypoint along direct path
                Vector3D.Subtract(ref targetPosition, ref currentPosition, out _tempDirection);
                Vector3D.Normalize(ref _tempDirection, out _tempDirection);

                double waypointDistance = Math.Min(distance, context.WaypointDistance);

                Vector3D.Multiply(ref _tempDirection, waypointDistance, out _tempOffset);
                Vector3D.Add(ref currentPosition, ref _tempOffset, out _tempWaypoint);

                // Apply gravity correction if needed
                if (config.UsePlanetAwarePathfinding() && context.IsInPlanetGravity())
                {
                    ApplyPlanetAwareCorrection(ref currentPosition, ref _tempWaypoint, ref targetPosition,
                        ref context, out result);
                    return true;
                }

                result = _tempWaypoint;
                return true;
            }

            // Path is blocked, try repositioning if allowed
            if (config.AllowRepathing())
            {
                return FindClearWaypoint(ref currentPosition, ref targetPosition, ref context, out result);
            }

            Log.Warning("DirectPathfinder: Path blocked and repathing disabled");
            return false;
        }

        /// <summary>
        /// Generate complete path (for debugging only)
        /// </summary>
        public bool CalculatePath(ref PathfindingContext context, ref Vector3D start, ref Vector3D end,
            List<Vector3D> pathWaypoints)
        {
            if (pathWaypoints == null)
                throw new ArgumentNullException("pathWaypoints");

            if (config == null)
            {
                Log.Error("DirectPathfinder: Config not loaded");
                pathWaypoints.Clear();
                pathWaypoints.Add(start);
                pathWaypoints.Add(end);
                return false;
            }

            // Cache world matrix for this call
            if (context.Controller != null)
            {
                _cachedWorldMatrix = context.Controller.WorldMatrix;
                MatrixD.Transpose(ref _cachedWorldMatrix, out _cachedWorldMatrixTransposed);
            }

            pathWaypoints.Clear();
            double distance = Vector3D.Distance(start, end);
            int steps = Math.Max(1, (int)(distance / context.WaypointDistance));

            // Generate waypoints
            for (int i = 0; i <= steps; i++)
            {
                float t = (float)i / steps;
                Vector3D basePos;
                Vector3D.Lerp(ref start, ref end, t, out basePos);

                // Apply planet-aware correction if needed
                if (config.UsePlanetAwarePathfinding() && context.IsInPlanetGravity())
                {
                    Vector3D correctedPos;
                    ApplyPlanetAwareCorrection(ref start, ref basePos, ref end, ref context, out correctedPos);
                    pathWaypoints.Add(correctedPos);
                }
                else
                {
                    pathWaypoints.Add(basePos);
                }
            }

            return true;
        }

        /// <summary>
        /// Check if we can reach the target directly without obstacles
        /// </summary>
        private bool CanReachDirectly(ref Vector3D start, ref Vector3D end, ref PathfindingContext context)
        {
            Vector3D.Subtract(ref end, ref start, out _tempDirection);

            // Check if we have thrust capability in this direction
            if (!context.CanClimbInDirection(ref _tempDirection, ref _cachedWorldMatrixTransposed))
            {
                Log.Verbose("DirectPathfinder: Insufficient thrust to reach target directly");
                return false;
            }

            // Check for obstacles using raycasting (if cameras available)
            if (config.RequireCamerasForPathfinding() &&
                context.CamerasByDirection != null && context.CamerasByDirection.Count > 0)
            {
                if (!IsPathClearViaRaycast(ref start, ref end, ref context))
                {
                    Log.Verbose("DirectPathfinder: Raycast detected obstacle in path");
                    return false;
                }
            }

            // Check for obstacles using sensors (if sensors available)
            if (config.RequireSensorsForPathfinding() &&
                context.SensorInfos != null && context.SensorInfos.Count > 0)
            {
                if (HasObstacleInSensorRange(ref start, ref end, ref context))
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
        private bool IsPathClearViaRaycast(ref Vector3D start, ref Vector3D end, ref PathfindingContext context)
        {
            Vector3D.Subtract(ref end, ref start, out _tempDirection);
            double distance = _tempDirection.Length();

            // Check if we can raycast in this direction
            if (!context.CanRaycastInDirection(ref _tempDirection, ref _cachedWorldMatrixTransposed))
            {
                return true; // No camera facing this direction, assume clear
            }

            // Scale raycast distance based on trip length
            double maxRaycastDistance = Math.Min(
                distance,
                Math.Min(config.MaxSimulatedCameraRaycastMeters(), distance * 0.5)
            );

            // Normalize and scale direction
            Vector3D.Normalize(ref _tempDirection, out _tempDirection);
            Vector3D.Multiply(ref _tempDirection, maxRaycastDistance, out _tempOffset);
            Vector3D.Add(ref start, ref _tempOffset, out _tempVector);

            LineD ray = new LineD(start, _tempVector);
            _raycastResults.Clear();

            try
            {
                MyGamePruningStructure.GetTopmostEntitiesOverlappingRay(ref ray, _raycastResults);

                for (int i = 0; i < _raycastResults.Count; i++)
                {
                    if (IsObstacleEntity(_raycastResults[i].Element, ref context))
                    {
                        Log.Verbose("DirectPathfinder: Raycast hit obstacle at distance {0}m",
                            Vector3D.Distance(start, _raycastResults[i].Element.PositionComp.GetPosition()));
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
        private bool HasObstacleInSensorRange(ref Vector3D start, ref Vector3D end, ref PathfindingContext context)
        {
            Vector3D.Subtract(ref end, ref start, out _tempDirection);
            Vector3D.Normalize(ref _tempDirection, out _tempDirection);

            double distance = Vector3D.Distance(start, end);

            for (int s = 0; s < context.SensorInfos.Count; s++)
            {
                PathfindingContext.SensorInfo sensorInfo = context.SensorInfos[s];

                // Check points along the path within sensor range
                double checkDistance = Math.Min(distance, sensorInfo.MaxRange);

                Vector3D.Multiply(ref _tempDirection, checkDistance * 0.5, out _tempOffset);
                Vector3D.Add(ref start, ref _tempOffset, out _tempVector);

                // Create bounding box around the check point
                double halfSize = sensorInfo.MaxRange * 0.5;
                Vector3D halfSizeVec = new Vector3D(halfSize, halfSize, halfSize);

                Vector3D boxMin, boxMax;
                Vector3D.Subtract(ref _tempVector, ref halfSizeVec, out boxMin);
                Vector3D.Add(ref _tempVector, ref halfSizeVec, out boxMax);

                BoundingBoxD box = new BoundingBoxD(boxMin, boxMax);

                _sensorResults.Clear();
                try
                {
                    MyGamePruningStructure.GetTopmostEntitiesInBox(ref box, _sensorResults);

                    for (int i = 0; i < _sensorResults.Count; i++)
                    {
                        if (IsObstacleEntity(_sensorResults[i], ref context))
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
        private bool IsObstacleEntity(IMyEntity entity, ref PathfindingContext context)
        {
            if (entity == null) return false;

            // Ignore our own grid
            if (context.CubeGrid != null && entity.EntityId == context.CubeGrid.EntityId)
                return false;

            // Check if it's a grid that belongs to us
            IMyCubeGrid grid = entity as IMyCubeGrid;
            if (grid != null)
            {
                if (context.CubeGrid != null)
                {
                    // Check if it's mechanically connected to us
                    _connectedGridsBuffer.Clear();
                    MyAPIGateway.GridGroups.GetGroup(context.CubeGrid, GridLinkTypeEnum.Mechanical,
                        _connectedGridsBuffer);

                    if (_connectedGridsBuffer.Contains(grid))
                    {
                        return false; // It's part of our ship
                    }
                }

                return true; // It's another grid - it's an obstacle
            }

            // Check for planets and asteroids
            if (entity is MyPlanet) return true;
            if (entity.ToString().Contains("Asteroid")) return true;

            return false;
        }

        /// <summary>
        /// Apply planet-aware arc pathing when in gravity
        /// </summary>
        private void ApplyPlanetAwareCorrection(ref Vector3D start, ref Vector3D waypoint,
            ref Vector3D target, ref PathfindingContext context, out Vector3D result)
        {
            if (!context.PlanetCenter.HasValue)
            {
                result = waypoint;
                return;
            }

            Vector3D planetCenter = context.PlanetCenter.Value;
            double distance = Vector3D.Distance(start, target);

            // For short distances, don't apply arc pathing (within 500m)
            if (distance < 500.0)
            {
                EnsureSafeAltitude(ref waypoint, ref context, out result);
                return;
            }

            // Calculate arc path around planet
            Vector3D startVector, targetVector, waypointVector;
            Vector3D.Subtract(ref start, ref planetCenter, out startVector);
            Vector3D.Subtract(ref target, ref planetCenter, out targetVector);
            Vector3D.Subtract(ref waypoint, ref planetCenter, out waypointVector);

            // Calculate the angle between start and target
            Vector3D normalizedStart, normalizedTarget;
            Vector3D.Normalize(ref startVector, out normalizedStart);
            Vector3D.Normalize(ref targetVector, out normalizedTarget);

            double dot = Vector3D.Dot(normalizedStart, normalizedTarget);
            double angle = Math.Acos(MathHelper.Clamp(dot, -1.0, 1.0));

            // Determine what percentage along the path this waypoint is
            double progress = Vector3D.Distance(start, waypoint) / distance;

            // Calculate position along the arc
            double currentAngle = angle * progress;
            Vector3D crossProduct;
            Vector3D.Cross(ref startVector, ref targetVector, out crossProduct);

            MatrixD.CreateFromAxisAngle(ref crossProduct, currentAngle, out _cachedRotation);
            Vector3D arcVector;
            Vector3D.Transform(ref startVector, ref _cachedRotation, out arcVector);

            // Maintain altitude (lerp between start and target altitudes)
            double startRadius = startVector.Length();
            double targetRadius = targetVector.Length();
            double desiredRadius = MathHelper.Lerp(startRadius, targetRadius, (float)progress);

            // Add altitude buffer
            double minSafeRadius = context.PlanetRadius + config.MinAltitudeBuffer();
            desiredRadius = Math.Max(desiredRadius, minSafeRadius);

            // Calculate final waypoint position
            Vector3D.Normalize(ref arcVector, out arcVector);
            Vector3D.Multiply(ref arcVector, desiredRadius, out arcVector);
            Vector3D.Add(ref planetCenter, ref arcVector, out result);
        }

        /// <summary>
        /// Ensure waypoint maintains safe altitude above planet surface
        /// </summary>
        private void EnsureSafeAltitude(ref Vector3D waypoint, ref PathfindingContext context,
            out Vector3D result)
        {
            if (!context.PlanetCenter.HasValue)
            {
                result = waypoint;
                return;
            }

            Vector3D planetCenter = context.PlanetCenter.Value;
            Vector3D waypointVector;
            Vector3D.Subtract(ref waypoint, ref planetCenter, out waypointVector);

            double currentRadius = waypointVector.Length();
            double minSafeRadius = context.PlanetRadius + config.MinAltitudeBuffer();

            if (currentRadius < minSafeRadius)
            {
                // Lift waypoint to safe altitude
                Vector3D.Normalize(ref waypointVector, out waypointVector);
                Vector3D.Multiply(ref waypointVector, minSafeRadius, out waypointVector);
                Vector3D.Add(ref planetCenter, ref waypointVector, out result);

                Log.Verbose("DirectPathfinder: Lifted waypoint to safe altitude ({0}m above surface)",
                    config.MinAltitudeBuffer());
                return;
            }

            result = waypoint;
        }

        /// <summary>
        /// Find a clear waypoint when direct path is blocked (repositioning)
        /// </summary>
        private bool FindClearWaypoint(ref Vector3D currentPosition, ref Vector3D targetPosition,
            ref PathfindingContext context, out Vector3D result)
        {
            result = default(Vector3D);

            if (!config.AllowRepathing())
            {
                Log.Warning("DirectPathfinder: Repathing disabled, cannot find alternate waypoint");
                return false;
            }

            Vector3D.Subtract(ref targetPosition, ref currentPosition, out _tempDirection);
            Vector3D.Normalize(ref _tempDirection, out _tempDirection);

            CalculatePerpendicularDirections(ref _tempDirection, ref context);

            // Try repositioning in different directions
            for (int attempt = 1; attempt <= config.MaxRepositionAttempts(); attempt++)
            {
                double repositionDistance = attempt * context.WaypointDistance * 0.5;

                // Try each perpendicular direction
                for (int i = 0; i < _perpendicularDirections.Count; i++)
                {
                    Vector3D perpDir = _perpendicularDirections[i];

                    Vector3D.Multiply(ref perpDir, repositionDistance, out _tempOffset);
                    Vector3D.Add(ref currentPosition, ref _tempOffset, out _tempWaypoint);

                    // Ensure safe altitude
                    if (config.UsePlanetAwarePathfinding() && context.IsInPlanetGravity())
                    {
                        EnsureSafeAltitude(ref _tempWaypoint, ref context, out _tempWaypoint);
                    }

                    // Check if this position has a clear path to target
                    if (CanReachDirectly(ref _tempWaypoint, ref targetPosition, ref context))
                    {
                        Log.Info("DirectPathfinder: Found clear waypoint after {0} reposition attempts", attempt);
                        result = _tempWaypoint;
                        return true;
                    }
                }

                Log.Verbose("DirectPathfinder: Reposition attempt {0} failed, trying further", attempt);
            }

            Log.Warning("DirectPathfinder: Failed to find clear waypoint after {0} attempts",
                config.MaxRepositionAttempts());
            return false;
        }

        /// <summary>
        /// Calculate perpendicular directions for repositioning
        /// </summary>
        private void CalculatePerpendicularDirections(ref Vector3D forwardDirection,
            ref PathfindingContext context)
        {
            _perpendicularDirections.Clear();

            if (context.Controller == null)
                return;

            // Get ship's up and right vectors
            Vector3D upDir = Base6Directions.GetVector(Base6Directions.Direction.Up);
            Vector3D rightDir = Base6Directions.GetVector(Base6Directions.Direction.Right);

            Vector3D shipUp, shipRight;
            Vector3D.Transform(ref upDir, ref _cachedWorldMatrix, out shipUp);
            Vector3D.Transform(ref rightDir, ref _cachedWorldMatrix, out shipRight);

            // In gravity, prefer moving up first
            double gravityLengthSq = context.GravityVector.LengthSquared();
            if (gravityLengthSq > 0.1)
            {
                Vector3D gravityUp;
                Vector3D.Negate(ref context.GravityVector, out gravityUp);
                Vector3D.Normalize(ref gravityUp, out gravityUp);

                _perpendicularDirections.Add(gravityUp);    // Up (away from planet)

                Vector3D gravityDown;
                Vector3D.Negate(ref gravityUp, out gravityDown);
                _perpendicularDirections.Add(gravityDown);  // Down (toward planet)
            }
            else
            {
                _perpendicularDirections.Add(shipUp);       // Ship up

                Vector3D shipDown;
                Vector3D.Negate(ref shipUp, out shipDown);
                _perpendicularDirections.Add(shipDown);     // Ship down
            }

            // Add horizontal directions
            _perpendicularDirections.Add(shipRight);        // Right

            Vector3D shipLeft;
            Vector3D.Negate(ref shipRight, out shipLeft);
            _perpendicularDirections.Add(shipLeft);         // Left

            // Filter out directions too parallel to forward (>45 degrees)
            for (int i = _perpendicularDirections.Count - 1; i >= 0; i--)
            {
                Vector3D dir = _perpendicularDirections[i];
                Vector3D normalizedDir;
                Vector3D.Normalize(ref dir, out normalizedDir);

                double dot = Math.Abs(Vector3D.Dot(normalizedDir, forwardDirection));
                if (dot >= 0.7) // 45 degrees threshold
                {
                    _perpendicularDirections.RemoveAt(i);
                }
            }
        }
    }
}