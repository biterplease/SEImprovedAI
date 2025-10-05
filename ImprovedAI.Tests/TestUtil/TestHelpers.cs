using ImprovedAI.Pathfinding;
using Moq;
using Sandbox.Game.Entities;
using Sandbox.ModAPI;
using System.Collections.Generic;
using VRage.Game.Components;
using VRage.Game.Entity;
using VRage.Game.ModAPI;
using VRageMath;

namespace ImprovedAI.Tests.TestUtil
{
    public static class TestHelpers
    {
        public static PathfindingContext CreateTestContext(Vector3 gravity, Vector3D position)
        {
            var config = new FakePathfindingConfig();

            var context = new PathfindingContext
            {
                // Configuration primitives
                MinWaypointDistance = config.MinWaypointDistance(),
                MaxWaypointDistance = config.MaxWaypointDistance(),
                MinAltitudeBuffer = config.MinAltitudeBuffer(),
                MaxPathNodes = config.MaxPathNodes(),
                MaxRepositionAttempts = config.MaxRepositionAttempts(),
                RequireSensorsForPathfinding = config.RequireSensorsForPathfinding(),
                RequireCamerasForPathfinding = config.RequireCamerasForPathfinding(),
                UsePlanetAwarePathfinding = config.UsePlanetAwarePathfinding(),
                AllowRepathing = config.AllowRepathing(),

                // Controller state
                ControllerPosition = position,
                ControllerWorldMatrix = MatrixD.CreateTranslation(position),
                ControllerForwardDirection = Base6Directions.Direction.Forward,

                // Environmental data
                GravityVector = gravity,
                IsInPlanetGravity = gravity.LengthSquared() > 0.1,
                PlanetCenter = null,
                PlanetRadius = 0,

                // Ship capabilities
                ShipMass = 1000f,
                MaxLoad = 1.0f,
                WaypointDistance = config.MaxWaypointDistance(),

                // Initialize lists
                Sensors = new List<PathfindingContext.SensorData>(),
                Cameras = new List<PathfindingContext.CameraData>(),
                TraveledNodes = new List<Vector3D>(),
                KnownObstacles = new List<PathfindingContext.ObstacleData>(),
                RaycastCache = new HashSet<Vector3D>(),
                PathBuffer = new List<Vector3D>(),
                NeighborBuffer = new List<Vector3I>()
            };

            // Calculate thrust data
            context.ThrustData.Forward = 100000f;
            context.ThrustData.Backward = 100000f;
            context.ThrustData.Up = 100000f;
            context.ThrustData.Down = 100000f;
            context.ThrustData.Left = 100000f;
            context.ThrustData.Right = 100000f;

            return context;
        }

        public static void AddPlanetToContext(ref PathfindingContext context, Vector3D planetCenter, double radius)
        {
            context.PlanetCenter = planetCenter;
            context.PlanetRadius = radius;
            context.IsInPlanetGravity = true;
        }

        public static MyEntity CreateMockObstacle(Vector3D position, double size = 10.0)
        {
            var entityMock = new Mock<MyEntity>();
            var positionCompMock = new Mock<MyPositionComponentBase>();

            positionCompMock.Setup(p => p.GetPosition()).Returns(position);

            var boundingBox = new BoundingBoxD(
                position - new Vector3D(size / 2),
                position + new Vector3D(size / 2)
            );
            positionCompMock.Setup(p => p.WorldAABB).Returns(boundingBox);

            entityMock.Setup(e => e.PositionComp).Returns(positionCompMock.Object);
            entityMock.Setup(e => e.EntityId).Returns((long)(position.X + position.Y + position.Z));

            return entityMock.Object;
        }
    }

    public class MockGamePruningStructureDelegate : IMyGamePruningStructureDelegate
    {
        private readonly List<MyEntity> lineIntersectionEntities;
        private readonly List<MyEntity> boxEntities;

        public MockGamePruningStructureDelegate()
        {
            lineIntersectionEntities = new List<MyEntity>();
            boxEntities = new List<MyEntity>();
        }

        public void AddLineIntersectionEntity(MyEntity entity)
        {
            lineIntersectionEntities.Add(entity);
        }

        public void AddBoxEntity(MyEntity entity)
        {
            boxEntities.Add(entity);
        }

        public void ClearLineIntersectionEntities()
        {
            lineIntersectionEntities.Clear();
        }

        public void ClearBoxEntities()
        {
            boxEntities.Clear();
        }

        public void ClearAll()
        {
            lineIntersectionEntities.Clear();
            boxEntities.Clear();
        }

        public void GetTopmostEntitiesOverlappingRay(
            ref LineD line,
            List<MyLineSegmentOverlapResult<MyEntity>> result,
            MyEntityQueryType queryType = MyEntityQueryType.Both)
        {
            result.Clear();

            foreach (var entity in lineIntersectionEntities)
            {
                result.Add(new MyLineSegmentOverlapResult<MyEntity>
                {
                    Element = entity,
                    Distance = Vector3D.Distance(line.From, entity.PositionComp.GetPosition())
                });
            }
        }

        public void GetTopMostEntitiesInBox(ref BoundingBoxD boundingBox, List<MyEntity> result, MyEntityQueryType queryType)
        {
            result.Clear();

            foreach (var entity in boxEntities)
            {
                var entityPos = entity.PositionComp.GetPosition();
                if (boundingBox.Contains(entityPos) != ContainmentType.Disjoint)
                {
                    result.Add(entity);
                }
            }
        }
    }

    public class MockPlanetDelegate : IMyPlanetDelegate
    {
        private MyPlanet mockPlanet;
        private Vector3D mockPlanetCenter;
        private double mockPlanetRadius;

        public MockPlanetDelegate()
        {
            mockPlanetCenter = Vector3D.Zero;
            mockPlanetRadius = 60000.0;
        }

        public void SetMockPlanet(Vector3D center, double radius)
        {
            mockPlanetCenter = center;
            mockPlanetRadius = radius;
            mockPlanet = null;
        }

        public void ClearMockPlanet()
        {
            mockPlanet = null;
        }

        public MyPlanet GetClosestPlanet(Vector3D position)
        {
            if (mockPlanetRadius > 0)
            {
                var distance = Vector3D.Distance(position, mockPlanetCenter);
                if (distance < mockPlanetRadius * 3)
                {
                    return mockPlanet;
                }
            }

            return null;
        }

        public double GetSurfaceAltitude(Vector3D position, MyPlanet planet)
        {
            if (mockPlanetRadius > 0)
            {
                var distanceFromCenter = Vector3D.Distance(position, mockPlanetCenter);
                return distanceFromCenter - mockPlanetRadius;
            }

            return double.MaxValue;
        }
    }
}