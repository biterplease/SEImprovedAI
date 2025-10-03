using Sandbox.ModAPI;
using System;
using System.Collections.Generic;
using VRageMath;

namespace ImprovedAI
{
    public class ThrustData
    {
        public float Forward;
        public float Backward;
        public float Up;
        public float Down;
        public float Left;
        public float Right;

        // Convenience properties
        public float MaxForwardBackward => Math.Max(Forward, Backward);
        public float MaxUpDown => Math.Max(Up, Down);
        public float MaxLeftRight => Math.Max(Left, Right);

        public ThrustData() { }

        public void CalculateThrust(List<IMyThrust> thrusters)
        {
            foreach (var thruster in thrusters)
            {
                if (!thruster.IsWorking) continue;

                Vector3I thrustDirection = thruster.GridThrustDirection;

                if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Forward))
                    Forward += thruster.MaxEffectiveThrust;
                else if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Backward))
                    Backward += thruster.MaxEffectiveThrust;
                else if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Up))
                    Up += thruster.MaxEffectiveThrust;
                else if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Down))
                    Down += thruster.MaxEffectiveThrust;
                else if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Left))
                    Left += thruster.MaxEffectiveThrust;
                else if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Right))
                    Right += thruster.MaxEffectiveThrust;
            }
        }

        // Get thrust capability in a specific direction
        public float GetThrustInDirection(Vector3D localDirection)
        {
            // Normalize the direction and find dominant axis
            var absDir = Vector3D.Abs(localDirection);

            if (absDir.Z > absDir.X && absDir.Z > absDir.Y) // Forward/Backward
            {
                return localDirection.Z > 0 ? Forward : Backward;
            }
            else if (absDir.Y > absDir.X) // Up/Down
            {
                return localDirection.Y > 0 ? Up : Down;
            }
            else // Left/Right
            {
                return localDirection.X > 0 ? Right : Left;
            }
        }

        // Get maximum thrust in any direction (for general capability)
        public float GetMaxThrust()
        {
            return Math.Max(Math.Max(Math.Max(Forward, Backward), Math.Max(Up, Down)), Math.Max(Left, Right));
        }

        public float GetMinThrust()
        {
            return Math.Min(Math.Min(Math.Min(Forward, Backward), Math.Min(Up, Down)), Math.Min(Left, Right));
        }

        // Check if we can effectively move in a direction considering gravity
        public bool CanMoveInDirection(Vector3D localDirection, Vector3D localGravity, float shipMass)
        {
            var availableThrust = GetThrustInDirection(localDirection);

            // If there's gravity, we need extra thrust to overcome it
            if (localGravity.LengthSquared() > 0.1)
            {
                float gravityForce = (float)(shipMass * localGravity.Length());
                var gravityDirection = Vector3D.Normalize(localGravity);

                // If moving against gravity, add gravity compensation
                var dot = Vector3D.Dot(Vector3D.Normalize(localDirection), -gravityDirection);
                if (dot > 0) // Moving against gravity
                {
                    var requiredThrust = gravityForce * (float)dot;
                    return availableThrust > requiredThrust; // Need enough thrust to overcome gravity
                }
            }

            return availableThrust > 0; // We have some thrust in this direction
        }
    }
}