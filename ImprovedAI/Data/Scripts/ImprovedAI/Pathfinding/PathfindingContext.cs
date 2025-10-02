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
            public IMySensorBlock Sensor {  get; set; }
            public Vector3D Position { get; set; }
            public float MaxRange { get; set; }
        }
        public IMyShipController Controller { get; set; }
        public Base6Directions.Direction ControllerFlightDirection { get; set; }
        public List<IMySensorBlock> Sensors { get; set; }
        public List<SensorInfo> SensorInfos { get; set; }
        public bool ConfigRequireCamerasForRaycasting = IAISession.Instance.GetConfig().Pathfinding.RequireCamerasForPathfinding;
        public bool ConfigRequireSensorsForRaycasting = IAISession.Instance.GetConfig().Pathfinding.RequireSensorsForPathinding;
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

            var forwardDir = Base6Directions.GetVector(ControllerFlightDirection);
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

        public bool CanRaycastInDirection(Vector3D worldDirection)
        {
            if (!ConfigRequireCamerasForRaycasting) return true;
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