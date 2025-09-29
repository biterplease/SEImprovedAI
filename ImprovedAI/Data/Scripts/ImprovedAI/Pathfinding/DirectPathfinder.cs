using ImprovedAI.Config;
using System;
using System.Collections.Generic;
using VRageMath;

namespace ImprovedAI.Pathfinding
{
    public class DirectPathfinder : IPathfinder
    {
        public PathfindingMethod Method => PathfindingMethod.Direct;

        public List<Vector3D> CalculatePath(Vector3D start, Vector3D end, PathfindingContext context)
        {
            var path = new List<Vector3D>();
            var totalDistance = Vector3D.Distance(start, end);

            // For very short distances, check if we can reach directly
            if (totalDistance < context.WaypointDistance)
            {
                if (!context.HasDetailedThrustData() || CanReachDirectly(start, end, context))
                {
                    path.Add(start);
                    path.Add(end);
                    return path;
                }
            }

            // Calculate path considering thrust limitations and gravity if available
            if (context.HasDetailedThrustData())
            {
                return CalculateThrustAwarePath(start, end, context);
            }
            else
            {
                return CalculateBasicPath(start, end, context);
            }
        }

        private List<Vector3D> CalculateThrustAwarePath(Vector3D start, Vector3D end, PathfindingContext context)
        {
            var path = new List<Vector3D>();
            var distance = Vector3D.Distance(start, end);
            var steps = Math.Max(1, (int)(distance / context.WaypointDistance));

            // Check if we need to climb against gravity
            var hasGravity = context.GravityVector.LengthSquared() > 0.1;
            var maxClimbAngle = hasGravity ? context.GetMaxSafeClimbAngle() : 90.0;

            for (int i = 0; i <= steps; i++)
            {
                var t = (float)i / steps;
                var basePos = Vector3D.Lerp(start, end, t);

                if (hasGravity)
                {
                    basePos = ApplyThrustAwareGravityCorrection(basePos, start, end, context, maxClimbAngle);
                }

                path.Add(basePos);
            }

            return path;
        }

        private List<Vector3D> CalculateBasicPath(Vector3D start, Vector3D end, PathfindingContext context)
        {
            var path = new List<Vector3D>();
            var distance = Vector3D.Distance(start, end);
            var steps = Math.Max(1, (int)(distance / context.WaypointDistance));

            for (int i = 0; i <= steps; i++)
            {
                var t = (float)i / steps;
                var pos = Vector3D.Lerp(start, end, t);

                // Apply basic gravity correction
                if (PathfindingConfig.UseGravityAwarePathing && context.GravityVector.LengthSquared() > 0.1)
                {
                    pos = ApplyBasicGravityCorrection(pos, context);
                }

                path.Add(pos);
            }

            return path;
        }

        private Vector3D ApplyThrustAwareGravityCorrection(Vector3D position, Vector3D start, Vector3D end,
            PathfindingContext context, double maxClimbAngle)
        {
            var gravityUp = Vector3D.Normalize(-context.GravityVector);
            var directionToEnd = Vector3D.Normalize(end - start);

            // Calculate the angle between our path and gravity
            var climbAngle = Math.Acos(Math.Abs(Vector3D.Dot(directionToEnd, gravityUp))) * 180.0 / Math.PI;

            // If the climb angle is too steep, adjust the path
            if (climbAngle > maxClimbAngle)
            {
                // Create a gentler climb path
                var horizontalDirection = directionToEnd - Vector3D.Dot(directionToEnd, gravityUp) * gravityUp;
                if (horizontalDirection.LengthSquared() > 0.01)
                {
                    horizontalDirection.Normalize();

                    // Calculate a safe climb rate
                    var safeClimbRatio = Math.Tan(maxClimbAngle * Math.PI / 180.0);
                    var horizontalDistance = Vector3D.Distance(
                        start - Vector3D.Dot(start, gravityUp) * gravityUp,
                        end - Vector3D.Dot(end, gravityUp) * gravityUp);

                    var verticalDistance = Vector3D.Dot(end - start, gravityUp);
                    var safeVerticalDistance = horizontalDistance * safeClimbRatio;

                    if (Math.Abs(verticalDistance) > safeVerticalDistance)
                    {
                        // Adjust the position to maintain safe climb angle
                        var adjustedVertical = Math.Sign(verticalDistance) * safeVerticalDistance;
                        var progressRatio = Math.Abs(adjustedVertical / verticalDistance);

                        position = start + (end - start) * progressRatio;
                    }
                }
            }

            return position;
        }

        private Vector3D ApplyBasicGravityCorrection(Vector3D position, PathfindingContext context)
        {
            // Ensure we maintain safe altitude in gravity
            if (context.GravityVector.LengthSquared() < 0.1)
                return position;

            var gravityUp = Vector3D.Normalize(-context.GravityVector);
            var planetCenter = position - gravityUp * 1000000; // Approximate planet center

            var distanceFromCenter = Vector3D.Distance(position, planetCenter);
            var minSafeDistance = 1000000 - PathfindingConfig.MinAltitudeBuffer;

            if (distanceFromCenter < minSafeDistance)
            {
                var direction = Vector3D.Normalize(position - planetCenter);
                return planetCenter + direction * minSafeDistance;
            }

            return position;
        }

        private bool CanReachDirectly(Vector3D start, Vector3D end, PathfindingContext context)
        {
            var direction = Vector3D.Normalize(end - start);
            return context.CanClimbInDirection(direction);
        }

        public bool IsAvailable(PathfindingContext context)
        {
            return true; // Always available as fallback
        }

        public int EstimatedComplexity(Vector3D start, Vector3D end)
        {
            return 1; // Minimal complexity, but slightly higher if using thrust-aware calculations
        }
    }
}