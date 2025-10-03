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
        public IMyShipController Controller { get; set; }
        public Base6Directions.Direction ControllerForwardDirection { get; set; }
        public IPathfindingConfig PathfindingConfig;
        public List<IMySensorBlock> Sensors { get; set; }
        public List<SensorInfo> SensorInfos { get; set; }
        public List<IMyCameraBlock> Cameras { get; set; }
        public Dictionary<Base6Directions.Direction, List<IMyCameraBlock>> CamerasByDirection { get; set; }
        public Vector3D GravityVector { get; set; }
        public float MaxLoad { get; set; }
        public float WaypointDistance { get; set; }
        public IMyCubeGrid CubeGrid { get; set; }

        // Enhanced thrust and mass properties
        public ThrustData ThrustData { get; set; }
        public float ShipMass { get; set; }

        public Vector3D? PlanetCenter { get; set; }
        public double PlanetRadius { get; set; }
        public bool isInPlanetGravity { get; set; }


        public PathfindingContext(
            IMyShipController controller,
            List<IMySensorBlock> sensors,
            List<IMyCameraBlock> cameras,
            List<IMyThrust> thrusters,
            float shipMass,
            float maxLoad,
            float waypointDistance,
            Base6Directions.Direction controllerForwardDirection
        )
        {
            Controller = controller;
            ShipMass = shipMass;
            MaxLoad = maxLoad;
            WaypointDistance = waypointDistance;
            CubeGrid = controller?.CubeGrid;


            // Get controller's flight direction
            ControllerForwardDirection = controllerForwardDirection;

            // Cache gravity information
            GravityVector = controller?.GetNaturalGravity() ?? Vector3D.Zero;

            // Detect planet if in gravity
            if (GravityVector.LengthSquared() > 0.1)
            {
                var controllerPos = controller.GetPosition();
                var planet = MyGamePruningStructure.GetClosestPlanet(controllerPos);
                if (planet != null)
                {
                    PlanetCenter = planet.PositionComp.GetPosition();
                    PlanetRadius = planet.AverageRadius;
                }
            }

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
                Sensors = sensors; // Keep original list reference
            }

            if (PathfindingConfig.RequireSensorsForPathfinding())
            {
                // Build camera direction cache
                CamerasByDirection = new Dictionary<Base6Directions.Direction, List<IMyCameraBlock>>();
                if (cameras != null)
                {
                    foreach (var camera in cameras)
                    {
                        if (camera?.IsFunctional == true)
                        {
                            // Determine which direction this camera faces in ship's local space
                            var cameraForward = camera.WorldMatrix.Forward;
                            var localForward = Vector3D.TransformNormal(cameraForward, MatrixD.Transpose(controller.WorldMatrix));

                            // Find dominant direction
                            var absDir = Vector3D.Abs(localForward);
                            Base6Directions.Direction direction;

                            if (absDir.Z > absDir.X && absDir.Z > absDir.Y)
                                direction = localForward.Z > 0 ? Base6Directions.Direction.Forward : Base6Directions.Direction.Backward;
                            else if (absDir.Y > absDir.X)
                                direction = localForward.Y > 0 ? Base6Directions.Direction.Up : Base6Directions.Direction.Down;
                            else
                                direction = localForward.X > 0 ? Base6Directions.Direction.Right : Base6Directions.Direction.Left;

                            // Add to dictionary
                            if (!CamerasByDirection.ContainsKey(direction))
                                CamerasByDirection[direction] = new List<IMyCameraBlock>();

                            CamerasByDirection[direction].Add(camera);
                        }
                    }
                }
                Cameras = cameras; // Keep original list reference
            }

            // Build thrust data cache
            ThrustData = new ThrustData();
            if (thrusters != null)
            {
                foreach (var thruster in thrusters)
                {
                    if (thruster?.IsWorking == true)
                    {
                        Vector3I thrustDirection = thruster.GridThrustDirection;

                        if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Forward))
                            ThrustData.Forward += thruster.MaxEffectiveThrust;
                        else if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Backward))
                            ThrustData.Backward += thruster.MaxEffectiveThrust;
                        else if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Up))
                            ThrustData.Up += thruster.MaxEffectiveThrust;
                        else if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Down))
                            ThrustData.Down += thruster.MaxEffectiveThrust;
                        else if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Left))
                            ThrustData.Left += thruster.MaxEffectiveThrust;
                        else if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Right))
                            ThrustData.Right += thruster.MaxEffectiveThrust;
                    }
                }
            }

            // Load pathfinding configuration
            PathfindingConfig = IAISession.Instance?.GetConfig()?.Pathfinding;
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
            return ThrustData.GetThrustInDirection(localDirection);
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

        // Check if ship can climb against gravity in a specific direction
        public bool CanClimbInDirection(Vector3D worldDirection)
        {
            if (Controller == null) return false;

            var localDirection = Vector3D.Transform(worldDirection, MatrixD.Transpose(Controller.WorldMatrix));
            var localGravity = Vector3D.Transform(GravityVector, MatrixD.Transpose(Controller.WorldMatrix));

            return ThrustData.CanMoveInDirection(localDirection, localGravity, ShipMass);
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
            return ThrustData.GetMaxThrust() > 0;
        }
        public bool IsInPlanetGravity() { return isInPlanetGravity; }
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
                dominantDir = localDirection.Z > 0 ? Base6Directions.Direction.Forward : Base6Directions.Direction.Backward;
            else if (absDir.Y > absDir.X)
                dominantDir = localDirection.Y > 0 ? Base6Directions.Direction.Up : Base6Directions.Direction.Down;
            else
                dominantDir = localDirection.X > 0 ? Base6Directions.Direction.Right : Base6Directions.Direction.Left;

            // Check if we have cameras facing this direction
            return CamerasByDirection.ContainsKey(dominantDir) && CamerasByDirection[dominantDir].Count > 0;
        }
    }
}