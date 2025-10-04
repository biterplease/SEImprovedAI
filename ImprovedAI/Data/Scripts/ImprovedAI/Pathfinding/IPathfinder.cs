using ImprovedAI.Config;
using System.Collections.Generic;
using VRageMath;

namespace ImprovedAI.Pathfinding
{
    public interface IPathfinder
    {
        PathfindingManager.Method Method { get; }
        bool CalculatePath(ref PathfindingContext context, ref Vector3D start, ref Vector3D end, List<Vector3D> output);
        bool IsAvailable(ref PathfindingContext context);
        int EstimatedComplexity(ref Vector3D start, ref Vector3D end);
        bool GetNextWaypoint(ref PathfindingContext context,  ref Vector3D currentPosition, ref     Vector3D targetPosition, out Vector3D result);
    }
}