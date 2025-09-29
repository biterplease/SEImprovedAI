using ImprovedAI.Config;
using System.Collections.Generic;
using VRageMath;

namespace ImprovedAI.Pathfinding
{
    public interface IPathfinder
    {
        PathfindingMethod Method { get; }
        List<Vector3D> CalculatePath(Vector3D start, Vector3D end, PathfindingContext context);
        bool IsAvailable(PathfindingContext context);
        int EstimatedComplexity(Vector3D start, Vector3D end);
    }
}