using ImprovedAI.Config;
using Sandbox.Game.Entities;
using Sandbox.ModAPI;
using System;
using System.Collections.Generic;
using System.Runtime.Remoting.Messaging;
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
        public IMyShipController Controller;
        private Vector3D _cachedLocalDirection;
        private Vector3D _cachedLocalGravity;
        public IMyGamePruningStructureDelegate pruningStructure { get; set; }
        public Base6Directions.Direction ControllerForwardDirection { get; set; }
        public IPathfindingConfig PathfindingConfig;
        public List<IMySensorBlock> Sensors { get; set; }
        public List<SensorInfo> SensorInfos { get; set; }
        public List<IMyCameraBlock> Cameras { get; set; }
        public Dictionary<Base6Directions.Direction, List<IMyCameraBlock>> CamerasByDirection { get; set; }
        public Vector3D GravityVector;
        public float MaxLoad { get; set; }
        public float WaypointDistance { get; set; }
        public IMyCubeGrid CubeGrid { get; set; }

        // Enhanced thrust and mass properties
        public ThrustData ThrustData;
        public float ShipMass { get; set; }

        public MyPlanet ClosestPlanet { get; set; }

        public Vector3D? PlanetCenter { get; set; }
        public double PlanetRadius { get; set; }
        public bool isInPlanetGravity { get; set; }





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

            // Get controller's flight direction
            ControllerForwardDirection = controllerForwardDirection;

            // Cache gravity information
            GravityVector = controller?.GetNaturalGravity() ?? Vector3D.Zero;

            // Detect planet if in gravity
            if (planetCenter != null && planetRadius != null)
            {
                PlanetCenter = planetCenter.Value;
                PlanetRadius = planetRadius.Value;
                isInPlanetGravity = true;  // FIX: Set flag
            }
            else if (GravityVector.LengthSquared() > 0.1)
            {
                // In gravity but no explicit planet data provided
                isInPlanetGravity = true;  // FIX: Set flag based on gravity
            }

            // FIX: Check correct config for sensors
            if (PathfindingConfig.RequireSensorsForPathfinding())
            {
                // Build sensor cache
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
                            // Get camera's physical mounting direction on the ship
                            Base6Directions.Direction cameraDirection = camera.Orientation.Forward;

                            // Map ship direction to navigation direction based on controller forward
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
 
        public double? GetSurfaceAltitude()
        {
            if (!PlanetCenter.HasValue || Controller == null)
                return null;

            var dronePosition = Controller.GetPosition();
            double distanceFromCenter = Vector3D.Distance(dronePosition, PlanetCenter.Value);
            return distanceFromCenter - PlanetRadius;
        }
        public bool IsAtSafeAltitude()
        {
            if (PathfindingConfig == null)
                return true; // Assume safe if no config

            var altitude = GetSurfaceAltitude();
            if (!altitude.HasValue)
                return true; // Not near a planet, altitude doesn't matter

            return altitude.Value > PathfindingConfig.MinAltitudeBuffer();
        }
        // Calculate effective thrust in world direction
        public float GetEffectiveThrustInWorldDirection(Vector3D worldDirection)
        {
            if (Controller == null) return 0f;

            // Convert world direction to local ship coordinates
            var localDirection = Vector3D.Transform(worldDirection, MatrixD.Transpose(Controller.WorldMatrix));
            return PathfindingUtil.GetThrustInDirection(ref localDirection, ref ThrustData);
        }

        /// <summary>
        /// Get the actual world forward direction accounting for controller orientation
        /// </summary>
        public Vector3D GetControllerForwardInWorld()
        {
            if (Controller == null) return Vector3D.Forward;

            var forwardDir = Base6Directions.GetVector(ControllerForwardDirection);
            return Vector3D.Transform(forwardDir, Controller.WorldMatrix);
        }

        public bool CanClimbInDirection(ref Vector3D worldDirection, ref MatrixD worldMatrixTransposed)
        {
            Vector3D.Transform(ref worldDirection, ref worldMatrixTransposed, out _cachedLocalDirection);
            Vector3D.Transform(ref GravityVector, ref worldMatrixTransposed, out _cachedLocalGravity);

            return PathfindingUtil.CanMoveInDirection(ref _cachedLocalDirection, ref _cachedLocalGravity, ref ThrustData, ShipMass);
        }


        // Get maximum safe climb angle considering thrust limitations
        public double GetMaxSafeClimbAngle()
        {
            if (GravityVector.LengthSquared() < 0.1)
                return 90.0; // No gravity, can go straight up

            var upThrust = ThrustData.Up;
            var gravityForce = ShipMass * GravityVector.Length();

            if (upThrust <= gravityForce)
                return 0.0; // Can't climb at all

            // Calculate angle based on thrust-to-weight ratio
            var thrustToWeight = upThrust / gravityForce;
            return Math.Asin(Math.Min(1.0, 1.0 / thrustToWeight)) * 180.0 / Math.PI;
        }

        // Convenience method to check if we have detailed thrust data
        public bool HasDetailedThrustData()
        {
            return PathfindingUtil.GetMaxThrust(ref ThrustData) > 0;
        }
        public bool IsInPlanetGravity() => isInPlanetGravity;
        public bool CanRaycastInDirection(Vector3D worldDirection)
        {
            if (!PathfindingConfig.RequireCamerasForPathfinding()) return true;
            if (Controller == null || CamerasByDirection == null) return false;

            // Convert world direction to local ship coordinates
            var localDirection = Vector3D.Transform(worldDirection, MatrixD.Transpose(Controller.WorldMatrix));

            // Find the dominant direction
            var absDir = Vector3D.Abs(localDirection);
            Base6Directions.Direction dominantDir;

            if (absDir.Z > absDir.X && absDir.Z > absDir.Y)
                dominantDir = localDirection.Z > 0 ? Base6Directions.Direction.Backward : Base6Directions.Direction.Forward;
            else if (absDir.Y > absDir.X)
                dominantDir = localDirection.Y > 0 ? Base6Directions.Direction.Up : Base6Directions.Direction.Down;
            else
                dominantDir = localDirection.X > 0 ? Base6Directions.Direction.Right : Base6Directions.Direction.Left;

            // Check if we have cameras facing this direction
            return CamerasByDirection.ContainsKey(dominantDir) && CamerasByDirection[dominantDir].Count > 0;
        }
    }
}