using ImprovedAI.Config;
using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;
using VRageMath;

namespace ImprovedAI.Pathfinding
{
    public class ObstacleAvoidancePathfinder : IPathfinder
    {
        public PathfindingMethod Method => PathfindingMethod.ObstacleAvoidance;
        private readonly DirectPathfinder directFallback = new DirectPathfinder();

        public List<Vector3D> CalculatePath(Vector3D start, Vector3D end, PathfindingContext context)
        {
            if (!IsAvailable(context))
                return directFallback.CalculatePath(start, end, context);

            var path = new List<Vector3D>();
            var distance = Vector3D.Distance(start, end);
            var steps = Math.Max(1, (int)(distance / context.WaypointDistance));

            for (int i = 0; i <= steps; i++)
            {
                var t = (float)i / steps;
                var basePos = Vector3D.Lerp(start, end, t);

                // Check for obstacles using sensors
                var adjustedPos = AvoidObstacles(basePos, end, context);

                // Apply gravity correction
                if (PathfindingConfig.UseGravityAwarePathing && context.GravityVector.LengthSquared() > 0.1)
                {
                    adjustedPos = ApplyGravityCorrection(adjustedPos, context);
                }

                path.Add(adjustedPos);
            }

            return path;
        }

        public bool IsAvailable(PathfindingContext context)
        {
            return PathfindingConfig.AllowSensors &&
                   context.Sensors != null &&
                   context.Sensors.Any(s => s.IsWorking);
        }

        public int EstimatedComplexity(Vector3D start, Vector3D end)
        {
            return 3; // Low-medium complexity
        }

        private Vector3D AvoidObstacles(Vector3D position, Vector3D target, PathfindingContext context)
        {
            var adjustedPos = position;
            var directionToTarget = Vector3D.Normalize(target - position);

            foreach (var sensor in context.Sensors.Where(s => s.IsWorking))
            {
                var detectedEntities = new List<MyDetectedEntityInfo>();
                sensor.DetectedEntities(detectedEntities);

                foreach (var entity in detectedEntities)
                {
                    if (IsObstacleRelevant(entity, position))
                    {
                        var obstaclePos = entity.Position;
                        var toObstacle = obstaclePos - position;
                        var obstacleDistance = toObstacle.Length();

                        // If obstacle is close and in our path
                        if (obstacleDistance < 50.0 && Vector3D.Dot(Vector3D.Normalize(toObstacle), directionToTarget) > 0.7)
                        {
                            // Calculate avoidance vector perpendicular to path
                            var avoidanceDirection = Vector3D.Cross(directionToTarget, Vector3D.Up);
                            if (avoidanceDirection.LengthSquared() < 0.1) // If up is parallel, use forward
                                avoidanceDirection = Vector3D.Cross(directionToTarget, Vector3D.Forward);

                            avoidanceDirection.Normalize();
                            var avoidanceDistance = Math.Max(10.0, 60.0 - obstacleDistance);
                            adjustedPos += avoidanceDirection * avoidanceDistance;
                        }
                    }
                }
            }

            return adjustedPos;
        }

        private bool IsObstacleRelevant(MyDetectedEntityInfo entity, Vector3D position)
        {
            return entity.Type == MyDetectedEntityType.LargeGrid ||
                   entity.Type == MyDetectedEntityType.SmallGrid ||
                   entity.Type == MyDetectedEntityType.Asteroid ||
                   entity.Type == MyDetectedEntityType.Planet;
        }

        private Vector3D ApplyGravityCorrection(Vector3D position, PathfindingContext context)
        {
            // Use thrust-aware correction if detailed data is available
            if (context.HasDetailedThrustData())
            {
                return ApplyThrustAwareGravityCorrection(position, context);
            }
            else
            {
                return ApplyBasicGravityCorrection(position, context);
            }
        }

        private Vector3D ApplyThrustAwareGravityCorrection(Vector3D position, PathfindingContext context)
        {
            if (context.GravityVector.LengthSquared() < 0.1)
                return position;

            var maxClimbAngle = context.GetMaxSafeClimbAngle();
            var gravityUp = Vector3D.Normalize(-context.GravityVector);

            // Ensure we don't create paths steeper than our thrust capability
            var altitudeAdjustment = PathfindingConfig.MinAltitudeBuffer * Math.Sin(maxClimbAngle * Math.PI / 180.0);
            var safePosition = position + gravityUp * altitudeAdjustment;

            return safePosition;
        }

        private Vector3D ApplyBasicGravityCorrection(Vector3D position, PathfindingContext context)
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