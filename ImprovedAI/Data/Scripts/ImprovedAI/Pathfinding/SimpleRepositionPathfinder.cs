using ImprovedAI.Config;
using System;
using System.Collections.Generic;
using VRageMath;

namespace ImprovedAI.Pathfinding
{
    public class SimpleRepositioningPathfinder : IPathfinder
    {
        public PathfindingMethod Method => PathfindingMethod.SimpleRepositioning;
        private readonly ObstacleAvoidancePathfinder fallback = new ObstacleAvoidancePathfinder();

        public List<Vector3D> CalculatePath(Vector3D start, Vector3D end, PathfindingContext context)
        {
            var distance = Vector3D.Distance(start, end);

            // Only use repositioning for medium distances
            if (distance > 50.0 || !IsAvailable(context))
                return fallback.CalculatePath(start, end, context);

            // Check if direct path is clear
            if (IsPathClear(start, end, context))
            {
                return CreateDirectPath(start, end, context);
            }

            // Try repositioning to find clear path
            var clearStart = FindClearPosition(start, end, context);
            if (clearStart.HasValue)
            {
                var path = new List<Vector3D>();
                path.Add(start);

                if (Vector3D.DistanceSquared(start, clearStart.Value) > 1.0) // If we moved significantly
                {
                    path.Add(clearStart.Value);
                }

                // Add waypoints from clear position to target
                var directPath = CreateDirectPath(clearStart.Value, end, context);
                path.AddRange(directPath.GetRange(1, directPath.Count - 1)); // Skip duplicate start

                return path;
            }

            // Fallback if repositioning fails
            return fallback.CalculatePath(start, end, context);
        }

        public bool IsAvailable(PathfindingContext context)
        {
            return PathfindingConfig.AllowSimpleRepositioning;
        }

        public int EstimatedComplexity(Vector3D start, Vector3D end)
        {
            var distance = Vector3D.Distance(start, end);
            return distance > 50.0 ? 2 : 5; // More complex for short distances where it's used
        }

        private bool IsPathClear(Vector3D start, Vector3D end, PathfindingContext context)
        {
            // Simple raycast check - in real implementation would use proper collision detection
            var direction = Vector3D.Normalize(end - start);
            var distance = Vector3D.Distance(start, end);
            var checkPoints = Math.Max(3, (int)(distance / 5.0));

            for (int i = 1; i < checkPoints; i++)
            {
                var checkPos = start + direction * (distance * i / checkPoints);
                if (HasObstacleAt(checkPos, context))
                {
                    return false;
                }
            }

            return true;
        }

        private Vector3D? FindClearPosition(Vector3D start, Vector3D target, PathfindingContext context)
        {
            var directionToTarget = Vector3D.Normalize(target - start);
            var perpendiculars = new[]
            {
                Vector3D.Cross(directionToTarget, Vector3D.Up),
                Vector3D.Cross(directionToTarget, Vector3D.Forward),
                Vector3D.Cross(directionToTarget, Vector3D.Right),
                -Vector3D.Cross(directionToTarget, Vector3D.Up),
                -Vector3D.Cross(directionToTarget, Vector3D.Forward),
                -Vector3D.Cross(directionToTarget, Vector3D.Right)
            };

            for (int attempt = 1; attempt <= PathfindingConfig.MaxRepositionAttempts; attempt++)
            {
                var repositionDistance = attempt * PathfindingConfig.RepositionStep;

                foreach (var perpendicular in perpendiculars)
                {
                    if (perpendicular.LengthSquared() < 0.1) continue;

                    var testPos = start + Vector3D.Normalize(perpendicular) * repositionDistance;

                    // Apply gravity correction
                    if (PathfindingConfig.UseGravityAwarePathing && context.GravityVector.LengthSquared() > 0.1)
                    {
                        testPos = ApplyGravityCorrection(testPos, context);
                    }

                    if (!HasObstacleAt(testPos, context) && IsPathClear(testPos, target, context))
                    {
                        return testPos;
                    }
                }
            }

            return null; // No clear position found
        }

        private bool HasObstacleAt(Vector3D position, PathfindingContext context)
        {
            // Simplified obstacle detection
            // In real implementation, would check against known obstacles or use raycasting

            // Check if position is inside the ship's own grid bounds (avoid self-collision)
            var gridBounds = context.CubeGrid.LocalAABB;
            var localPos = Vector3D.Transform(position, context.CubeGrid.PositionComp.WorldMatrixInvScaled);

            if (gridBounds.Contains(localPos) != ContainmentType.Disjoint)
            {
                return true; // Inside our own grid
            }

            // Additional obstacle checks could be added here
            return false;
        }

        private List<Vector3D> CreateDirectPath(Vector3D start, Vector3D end, PathfindingContext context)
        {
            var path = new List<Vector3D>();
            var distance = Vector3D.Distance(start, end);
            var steps = Math.Max(1, (int)(distance / context.WaypointDistance));

            for (int i = 0; i <= steps; i++)
            {
                var t = (float)i / steps;
                var pos = Vector3D.Lerp(start, end, t);

                if (PathfindingConfig.UseGravityAwarePathing && context.GravityVector.LengthSquared() > 0.1)
                {
                    pos = ApplyGravityCorrection(pos, context);
                }

                path.Add(pos);
            }

            return path;
        }

        private Vector3D ApplyGravityCorrection(Vector3D position, PathfindingContext context)
        {
            if (context.GravityVector.LengthSquared() < 0.1)
                return position;

            var gravityUp = Vector3D.Normalize(-context.GravityVector);
            var planetCenter = position - gravityUp * 1000000;

            var distanceFromCenter = Vector3D.Distance(position, planetCenter);
            var minSafeDistance = 1000000 - PathfindingConfig.MinAltitudeBuffer;

            if (distanceFromCenter < minSafeDistance)
            {
                var direction = Vector3D.Normalize(position - planetCenter);
                return planetCenter + direction * minSafeDistance;
            }

            return position;
        }
    }
}