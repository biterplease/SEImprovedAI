using System;
using System.Collections.Generic;
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
                    return MatrixD.CreateRotationX(-Math.PI / 2); // -90° around X
                                                                  // This rotates +Y to -Z (up becomes forward)
                case Base6Directions.Direction.Down:
                    return MatrixD.CreateRotationX(Math.PI / 2); // 90° around X
                                                                 // This rotates -Y to -Z (down becomes forward)
                case Base6Directions.Direction.Left:
                    return MatrixD.CreateRotationY(Math.PI / 2); // 90° around Y
                                                                 // This rotates -X to -Z (left becomes forward)
                case Base6Directions.Direction.Right:
                    return MatrixD.CreateRotationY(-Math.PI / 2); // -90° around Y
                                                                  // This rotates +X to -Z (right becomes forward)
                default:
                    return MatrixD.Identity;
            }
        }
    }
}
