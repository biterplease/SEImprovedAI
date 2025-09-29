using BetterAIConstructor.Config;
using Sandbox.ModAPI;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using VRageMath;

namespace BetterAIConstructor.Pathfinding
{
    public class PathfindingManager
    {
        private readonly List<IPathfinder> pathfinders = new List<IPathfinder>
        {
            new DirectPathfinder(),
            new ObstacleAvoidancePathfinder(),
            new SimpleRepositioningPathfinder(),
            // A* and D* Lite would be added here when implemented
        };

        public List<Vector3D> CalculatePath(Vector3D start, Vector3D end, PathfindingContext context)
        {
            // Clamp waypoint distance to server limits
            context.WaypointDistance = MathHelper.Clamp(
                context.WaypointDistance,
                PathfindingConfig.MinWaypointDistance,
                PathfindingConfig.MaxWaypointDistance);

            var availablePathfinders = pathfinders
                .Where(p => p.IsAvailable(context))
                .OrderBy(p => p.EstimatedComplexity(start, end))
                .ToList();

            if (!availablePathfinders.Any())
            {
                MyAPIGateway.Utilities.ShowMessage("PathfindingManager", "No pathfinders available! Using emergency direct path.");
                return CreateEmergencyPath(start, end);
            }

            var stopwatch = Stopwatch.StartNew();

            foreach (var pathfinder in availablePathfinders)
            {
                try
                {
                    var path = pathfinder.CalculatePath(start, end, context);

                    if (IsValidPath(path, start, end))
                    {
                        stopwatch.Stop();
                        LogPathfindingResult(pathfinder.Method, stopwatch.Elapsed, path.Count);
                        return path;
                    }
                }
                catch (Exception ex)
                {
                    MyAPIGateway.Utilities.ShowMessage("PathfindingManager",
                        $"Pathfinder {pathfinder.Method} failed: {ex.Message}");
                }

                // Check if we've exceeded time limit
                if (stopwatch.Elapsed > PathfindingConfig.MaxPathfindingTime)
                {
                    MyAPIGateway.Utilities.ShowMessage("PathfindingManager",
                        "Pathfinding timeout - using emergency path");
                    break;
                }
            }

            // All pathfinders failed, use emergency direct path
            return CreateEmergencyPath(start, end);
        }

        private bool IsValidPath(List<Vector3D> path, Vector3D start, Vector3D end)
        {
            if (path == null || path.Count < 2)
                return false;

            // Check that path starts and ends reasonably close to expected positions
            var pathStart = path.First();
            var pathEnd = path.Last();

            return Vector3D.DistanceSquared(pathStart, start) < 100 && // Within 10m of start
                   Vector3D.DistanceSquared(pathEnd, end) < 100;     // Within 10m of end
        }

        private List<Vector3D> CreateEmergencyPath(Vector3D start, Vector3D end)
        {
            return new List<Vector3D> { start, end };
        }

        private void LogPathfindingResult(PathfindingMethod method, TimeSpan elapsed, int waypointCount)
        {
            MyAPIGateway.Utilities.ShowMessage("PathfindingManager",
                $"Used {method} pathfinding: {elapsed.TotalMilliseconds:F1}ms, {waypointCount} waypoints");
        }

        public void UpdateConfiguration()
        {
            PathfindingConfig.LoadFromConfig();
        }
    }
}