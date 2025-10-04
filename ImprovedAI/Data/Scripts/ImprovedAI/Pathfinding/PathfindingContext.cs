using ImprovedAI.Config;
using Sandbox.Game.Entities;
using Sandbox.ModAPI;
using System;
using System.Collections.Generic;
using VRage.Game.ModAPI;
using VRageMath;

namespace ImprovedAI.Pathfinding
{
    public class PathfindingContext
    {
        public class SensorInfo
        {
            public IMySensorBlock Sensor { get; set; }
            public Vector3D Position { get; set; }
            public float MaxRange { get; set; }
        }

        // Public fields - direct access, no property overhead
        public IMyShipController Controller;
        public IMyGamePruningStructureDelegate pruningStructure;
        public Base6Directions.Direction ControllerForwardDirection;
        public IPathfindingConfig PathfindingConfig;
        public List<IMySensorBlock> Sensors;
        public List<SensorInfo> SensorInfos;
        public List<IMyCameraBlock> Cameras;
        public Dictionary<Base6Directions.Direction, List<IMyCameraBlock>> CamerasByDirection;
        public Vector3D GravityVector;
        public float MaxLoad;
        public float WaypointDistance;
        public IMyCubeGrid CubeGrid;
        public ThrustData ThrustData;
        public float ShipMass;
        public MyPlanet ClosestPlanet;
        public Vector3D? PlanetCenter;
        public double PlanetRadius;
        public bool isInPlanetGravity;

        // Private cache fields for frequently used calculations
        private Vector3D _cachedLocalDirection;
        private Vector3D _cachedLocalGravity;

        public PathfindingContext(
             IPathfindingConfig pathfindingConfig,
             IMyShipController controller,
             List<IMySensorBlock> sensors,
             List<IMyCameraBlock> cameras,
             List<IMyThrust> thrusters,
             float shipMass,
             float maxLoad,
             float waypointDistance,
             Base6Directions.Direction controllerForwardDirection,
             IMyGamePruningStructureDelegate pruningStructureDelegate = null,
             Vector3D? planetCenter = null,
             double? planetRadius = null
         )
        {
            PathfindingConfig = pathfindingConfig;
            Controller = controller;
            ShipMass = shipMass;
            MaxLoad = maxLoad;
            WaypointDistance = waypointDistance;
            CubeGrid = controller?.CubeGrid;

            this.pruningStructure = pruningStructureDelegate ?? new MyGamePruningStructureDelegate();
            ControllerForwardDirection = controllerForwardDirection;
            GravityVector = controller?.GetNaturalGravity() ?? Vector3D.Zero;

            // Detect planet if in gravity
            if (planetCenter != null && planetRadius != null)
            {
                PlanetCenter = planetCenter.Value;
                PlanetRadius = planetRadius.Value;
                isInPlanetGravity = true;
            }
            else if (GravityVector.LengthSquared() > 0.1)
            {
                isInPlanetGravity = true;
            }

            if (PathfindingConfig.RequireSensorsForPathfinding())
            {
                SensorInfos = new List<SensorInfo>();
                if (sensors != null)
                {
                    foreach (var sensor in sensors)
                    {
                        if (sensor?.IsFunctional == true)
                        {
                            SensorInfos.Add(new SensorInfo
                            {
                                Sensor = sensor,
                                Position = sensor.GetPosition(),
                                MaxRange = sensor.MaxRange
                            });
                        }
                    }
                }
                Sensors = sensors;
            }

            if (PathfindingConfig.RequireCamerasForPathfinding())
            {
                CamerasByDirection = new Dictionary<Base6Directions.Direction, List<IMyCameraBlock>>();
                if (cameras != null)
                {
                    foreach (var camera in cameras)
                    {
                        if (camera?.IsFunctional == true)
                        {
                            Base6Directions.Direction cameraDirection = camera.Orientation.Forward;
                            Base6Directions.Direction navDirection = PathfindingUtil.MapToNavigationDirection(
                                cameraDirection,
                                ControllerForwardDirection);

                            if (!CamerasByDirection.ContainsKey(navDirection))
                                CamerasByDirection[navDirection] = new List<IMyCameraBlock>();

                            CamerasByDirection[navDirection].Add(camera);
                        }
                    }
                }
                Cameras = cameras;
            }

            // Build thrust data
            if (thrusters != null && thrusters.Count > 0)
            {
                ThrustData = new ThrustData();
                ThrustData.CalculateThrust(thrusters);
            }
        }

        /// <summary>
        /// Get surface altitude above planet. Returns null if not near a planet.
        /// </summary>
        public double? GetSurfaceAltitude()
        {
            if (!PlanetCenter.HasValue || Controller == null)
                return null;

            var dronePosition = Controller.GetPosition();
            double distanceFromCenter = Vector3D.Distance(dronePosition, PlanetCenter.Value);
            return distanceFromCenter - PlanetRadius;
        }

        /// <summary>
        /// Check if at safe altitude above planet surface.
        /// </summary>
        public bool IsAtSafeAltitude()
        {
            if (PathfindingConfig == null)
                return true;

            var altitude = GetSurfaceAltitude();
            if (!altitude.HasValue)
                return true;

            return altitude.Value > PathfindingConfig.MinAltitudeBuffer();
        }

        /// <summary>
        /// Calculate effective thrust in world direction.
        /// Use version with cached matrix for better performance.
        /// </summary>
        public float GetEffectiveThrustInWorldDirection(ref Vector3D worldDirection, ref MatrixD worldMatrixTransposed)
        {
            Vector3D.Transform(ref worldDirection, ref worldMatrixTransposed, out _cachedLocalDirection);
            return PathfindingUtil.GetThrustInDirection(ref _cachedLocalDirection, ref ThrustData);
        }

        /// <summary>
        /// Get the actual world forward direction accounting for controller orientation.
        /// </summary>
        public void GetControllerForwardInWorld(ref MatrixD worldMatrix, out Vector3D result)
        {
            if (Controller == null)
            {
                result = Vector3D.Forward;
                return;
            }

            var forwardDir = Base6Directions.GetVector(ControllerForwardDirection);
            Vector3D.Transform(ref forwardDir, ref worldMatrix, out result);
        }

        /// <summary>
        /// Check if ship can climb against gravity in a specific direction.
        /// Pass pre-calculated transposed world matrix for best performance.
        /// </summary>
        public bool CanClimbInDirection(ref Vector3D worldDirection, ref MatrixD worldMatrixTransposed)
        {
            Vector3D.Transform(ref worldDirection, ref worldMatrixTransposed, out _cachedLocalDirection);
            Vector3D.Transform(ref GravityVector, ref worldMatrixTransposed, out _cachedLocalGravity);

            return PathfindingUtil.CanMoveInDirection(ref _cachedLocalDirection, ref _cachedLocalGravity, ref ThrustData, ShipMass);
        }

        /// <summary>
        /// Get maximum safe climb angle considering thrust limitations.
        /// </summary>
        public double GetMaxSafeClimbAngle()
        {
            double gravityLengthSquared = GravityVector.LengthSquared();
            if (gravityLengthSquared < 0.1)
                return 90.0;

            var upThrust = ThrustData.Up;
            var gravityForce = ShipMass * Math.Sqrt(gravityLengthSquared);

            if (upThrust <= gravityForce)
                return 0.0;

            var thrustToWeight = upThrust / gravityForce;
            return Math.Asin(Math.Min(1.0, 1.0 / thrustToWeight)) * 180.0 / Math.PI;
        }

        /// <summary>
        /// Check if we have detailed thrust data.
        /// </summary>
        public bool HasDetailedThrustData()
        {
            return PathfindingUtil.GetMaxThrust(ref ThrustData) > 0;
        }

        /// <summary>
        /// Check if currently in planet gravity.
        /// </summary>
        public bool IsInPlanetGravity() => isInPlanetGravity;

        /// <summary>
        /// Check if can raycast in direction with available cameras.
        /// Use version with cached matrix for better performance.
        /// </summary>
        public bool CanRaycastInDirection(ref Vector3D worldDirection, ref MatrixD worldMatrixTransposed)
        {
            if (!PathfindingConfig.RequireCamerasForPathfinding())
                return true;

            if (Controller == null || CamerasByDirection == null)
                return false;

            // Convert world direction to local ship coordinates
            Vector3D.Transform(ref worldDirection, ref worldMatrixTransposed, out _cachedLocalDirection);

            // Find the dominant direction
            var absDir = Vector3D.Abs(_cachedLocalDirection);
            Base6Directions.Direction dominantDir;

            if (absDir.Z > absDir.X && absDir.Z > absDir.Y)
                dominantDir = _cachedLocalDirection.Z > 0 ? Base6Directions.Direction.Backward : Base6Directions.Direction.Forward;
            else if (absDir.Y > absDir.X)
                dominantDir = _cachedLocalDirection.Y > 0 ? Base6Directions.Direction.Up : Base6Directions.Direction.Down;
            else
                dominantDir = _cachedLocalDirection.X > 0 ? Base6Directions.Direction.Right : Base6Directions.Direction.Left;

            // Check if we have cameras facing this direction
            return CamerasByDirection.ContainsKey(dominantDir) && CamerasByDirection[dominantDir].Count > 0;
        }
    }
}