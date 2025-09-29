using BetterAIConstructor.Interfaces;
using Sandbox.ModAPI;
using System;
using System.Collections.Generic;
using VRage.Game.ModAPI;
using VRageMath;


namespace ImprovedAI.Pathfinding
{ 
    public class PathfindingContext
    {
        public BAIController Controller { get; set; }
        public List<IMySensorBlock> Sensors { get; set; }
        public Vector3D GravityVector { get; set; }
        public double MaxThrust { get; set; }
        public double MaxLoad { get; set; }
        public double WaypointDistance { get; set; }
        public IMyCubeGrid CubeGrid { get; set; }

        // Enhanced thrust and mass properties
        public ThrustData ThrustData { get; set; }
        public float ShipMass { get; set; }

        // Calculate effective thrust in world direction
        public float GetEffectiveThrustInWorldDirection(Vector3D worldDirection)
        {
            if (Controller == null) return 0f;

            // Convert world direction to local ship coordinates
            var localDirection = Vector3D.Transform(worldDirection, MatrixD.Transpose(Controller.WorldMatrix));
            return ThrustData.GetThrustInDirection(localDirection);
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

        // Fallback thrust value when detailed data isn't available
        public double GetFallbackMaxThrust()
        {
            return HasDetailedThrustData() ? ThrustData.GetMaxThrust() : MaxThrust;
        }
    }
}