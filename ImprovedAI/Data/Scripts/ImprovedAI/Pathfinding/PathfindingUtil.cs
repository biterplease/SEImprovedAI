using System;
using VRageMath;

namespace ImprovedAI.Pathfinding
{
    public class PathfindingUtil
    {
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
    }
}