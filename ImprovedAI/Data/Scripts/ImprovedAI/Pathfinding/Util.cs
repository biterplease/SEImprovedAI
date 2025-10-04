using System;
using VRageMath;

namespace ImprovedAI.Pathfinding
{
    public class PathfindingUtil
    {
        /// <summary>
        /// Directional mappings for controllers with a different forward direction.
        /// Indexed as [controllerForward, objectDirection]
        /// </summary>
        private static readonly Base6Directions.Direction[,] NavigationMappings = InitializeMappings();
        private static Base6Directions.Direction[,] InitializeMappings()
        {
            var mappings = new Base6Directions.Direction[6, 6];

            mappings[0, 0] = Base6Directions.Direction.Forward;  // Forward -> Backward
            mappings[0, 1] = Base6Directions.Direction.Backward;   // Backward -> Forward
            mappings[0, 2] = Base6Directions.Direction.Left;     // Left -> Right
            mappings[0, 3] = Base6Directions.Direction.Right;      // Right -> Left
            mappings[0, 4] = Base6Directions.Direction.Up;        // Up -> Up
            mappings[0, 5] = Base6Directions.Direction.Down;      // Down -> Down


            // Backward (1)
            mappings[1, 0] = Base6Directions.Direction.Backward;  // Forward -> Backward
            mappings[1, 1] = Base6Directions.Direction.Forward;   // Backward -> Forward
            mappings[1, 2] = Base6Directions.Direction.Right;     // Left -> Right
            mappings[1, 3] = Base6Directions.Direction.Left;      // Right -> Left
            mappings[1, 4] = Base6Directions.Direction.Up;        // Up -> Up
            mappings[1, 5] = Base6Directions.Direction.Down;      // Down -> Down

            // Left (2)
            mappings[2, 0] = Base6Directions.Direction.Right;     // Forward -> Right
            mappings[2, 1] = Base6Directions.Direction.Left;      // Backward -> Left
            mappings[2, 2] = Base6Directions.Direction.Forward;   // Left -> Forward
            mappings[2, 3] = Base6Directions.Direction.Backward;  // Right -> Backward
            mappings[2, 4] = Base6Directions.Direction.Up;        // Up -> Up
            mappings[2, 5] = Base6Directions.Direction.Down;      // Down -> Down

            // Right (3)
            mappings[3, 0] = Base6Directions.Direction.Left;      // Forward -> Left
            mappings[3, 1] = Base6Directions.Direction.Right;     // Backward -> Right
            mappings[3, 2] = Base6Directions.Direction.Backward;  // Left -> Backward
            mappings[3, 3] = Base6Directions.Direction.Forward;   // Right -> Forward
            mappings[3, 4] = Base6Directions.Direction.Up;        // Up -> Up
            mappings[3, 5] = Base6Directions.Direction.Down;      // Down -> Down

            // Up (4)
            mappings[4, 0] = Base6Directions.Direction.Down;      // Forward -> Down
            mappings[4, 1] = Base6Directions.Direction.Up;        // Backward -> Up
            mappings[4, 2] = Base6Directions.Direction.Left;      // Left -> Left
            mappings[4, 3] = Base6Directions.Direction.Right;     // Right -> Right
            mappings[4, 4] = Base6Directions.Direction.Forward;   // Up -> Forward
            mappings[4, 5] = Base6Directions.Direction.Backward;  // Down -> Backward

            // Down (5)
            mappings[5, 0] = Base6Directions.Direction.Up;        // Forward -> Up
            mappings[5, 1] = Base6Directions.Direction.Down;      // Backward -> Down
            mappings[5, 2] = Base6Directions.Direction.Left;      // Left -> Left
            mappings[5, 3] = Base6Directions.Direction.Right;     // Right -> Right
            mappings[5, 4] = Base6Directions.Direction.Backward;  // Up -> Backward
            mappings[5, 5] = Base6Directions.Direction.Forward;   // Down -> Forward

            return mappings;
        }
        public static Base6Directions.Direction MapToNavigationDirection(
            Base6Directions.Direction objectDirection,
            Base6Directions.Direction controllerForward)
        {
            return NavigationMappings[(int)controllerForward, (int)objectDirection];
        }


        /// <summary>
        /// Helper method to get rotation matrix, adjusted for controller forward direction.
        /// </summary>
        /// <param name="controllerForward"></param>
        /// <returns></returns>
        public static MatrixD GetNavigationRotationMatrix(Base6Directions.Direction controllerForward)
        {
            switch (controllerForward)
            {
                case Base6Directions.Direction.Forward:
                    return MatrixD.Identity; // No rotation needed
                case Base6Directions.Direction.Backward:
                    return MatrixD.CreateRotationY(Math.PI); // 180° around Y
                case Base6Directions.Direction.Up:
                    return MatrixD.CreateRotationX(Math.PI / 2); // +90° around X
                                                                 // This rotates -Z to +Y (forward becomes up)
                case Base6Directions.Direction.Down:
                    return MatrixD.CreateRotationX(-Math.PI / 2); // -90° around X
                                                                  // This rotates -Z to -Y (forward becomes down)
                case Base6Directions.Direction.Left:
                    return MatrixD.CreateRotationY(Math.PI / 2); // +90° around Y
                                                                 // This rotates -Z to -X (forward becomes left)
                case Base6Directions.Direction.Right:
                    return MatrixD.CreateRotationY(-Math.PI / 2); // -90° around Y
                                                                  // This rotates -Z to +X (forward becomes right)
                default:
                    return MatrixD.Identity;
            }
        }

        public static float GetMaxThrust(ref ThrustData thrustData)
        {
            return Math.Max(Math.Max(Math.Max(thrustData.Forward, thrustData.Backward), Math.Max(thrustData.Up, thrustData.Down)), Math.Max(thrustData.Left, thrustData.Right));
        }

        public static float GetMinThrust(ref ThrustData thrustData)
        {
            return Math.Min(Math.Min(Math.Min(thrustData.Forward, thrustData.Backward), Math.Min(thrustData.Up, thrustData.Down)), Math.Min(thrustData.Left, thrustData.Right));
        }


        // Get thrust capability in a specific direction
        public static float GetThrustInDirection(ref Vector3D localDirection, ref ThrustData thrustData)
        {
            // Normalize the direction and find dominant axis
            var absDir = Vector3D.Abs(localDirection);

            if (absDir.Z > absDir.X && absDir.Z > absDir.Y) // Forward/Backward
            {
                return localDirection.Z > 0 ? thrustData.Backward : thrustData.Forward;
            }
            else if (absDir.Y > absDir.X) // Up/Down
            {
                return localDirection.Y > 0 ? thrustData.Up : thrustData.Down;
            }
            else // Left/Right
            {
                return localDirection.X > 0 ? thrustData.Right : thrustData.Left;
            }
        }

        // Check if we can effectively move in a direction considering gravity
        public static bool CanMoveInDirection(ref Vector3D localDirection, ref Vector3D localGravity, ref ThrustData thrustData, float shipMass)
        {
            var availableThrust = GetThrustInDirection(ref localDirection, ref thrustData);

            double gravityLengthSquared = localGravity.LengthSquared();
            if (gravityLengthSquared > 0.1)
            {
                double gravityLength = Math.Sqrt(gravityLengthSquared); // Already computed the square
                float gravityForce = (float)(shipMass * gravityLength);

                // Normalize both directions using ref/out to avoid allocations
                Vector3D normalizedDirection;
                Vector3D normalizedGravity;
                Vector3D.Normalize(ref localDirection, out normalizedDirection);
                Vector3D.Normalize(ref localGravity, out normalizedGravity);

                // Check if moving against gravity
                double dot = Vector3D.Dot(normalizedDirection, -normalizedGravity);
                if (dot > 0) // Moving against gravity
                {
                    var requiredThrust = gravityForce * (float)dot;
                    return availableThrust > requiredThrust;
                }
            }

            return availableThrust > 0;
        }
    }
}