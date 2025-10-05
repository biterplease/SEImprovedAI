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
            // Mock grid
            var gridMock = new Mock<IMyCubeGrid>();
            gridMock.Setup(g => g.EntityId).Returns(1);
            gridMock.Setup(g => g.GridSize).Returns(2.5f);

            // Mock controller
            var controllerMock = new Mock<IMyShipController>();
            controllerMock.Setup(c => c.GetNaturalGravity()).Returns(gravity);
            controllerMock.Setup(c => c.CubeGrid).Returns(gridMock.Object);
            controllerMock.Setup(c => c.WorldMatrix).Returns(MatrixD.CreateTranslation(position));
            controllerMock.Setup(c => c.GetPosition()).Returns(position);
            controllerMock.Setup(c => c.IsWorking).Returns(true);
            controllerMock.Setup(c => c.IsFunctional).Returns(true);
            controllerMock.Setup(c => c.Orientation).Returns(new MyBlockOrientation(
                Base6Directions.Direction.Forward,
                Base6Directions.Direction.Up));

            // Mock thrusters
            var thrusters = new List<IMyThrust>();
            var directions = new[] {
                Base6Directions.Direction.Forward,
                Base6Directions.Direction.Backward,
                Base6Directions.Direction.Up,
                Base6Directions.Direction.Down,
                Base6Directions.Direction.Left,
                Base6Directions.Direction.Right
            };

            foreach (var dir in directions)
            {
                var thrustMock = new Mock<IMyThrust>();
                thrustMock.Setup(t => t.Orientation).Returns(new MyBlockOrientation(dir, Base6Directions.Direction.Up));
                thrustMock.Setup(t => t.MaxEffectiveThrust).Returns(100000f);
                thrustMock.Setup(t => t.CurrentThrust).Returns(0f);
                thrustMock.Setup(t => t.IsWorking).Returns(true);
                thrustMock.Setup(t => t.IsFunctional).Returns(true);
                thrustMock.Setup(t => t.CubeGrid).Returns(gridMock.Object);
                thrustMock.Setup(t => t.WorldMatrix).Returns(MatrixD.Identity);
                thrusters.Add(thrustMock.Object);
            }

            return new PathfindingContext(
                new FakePathfindingConfig(),
                controllerMock.Object,
                new List<IMySensorBlock>(),
                new List<IMyCameraBlock>(),
                thrusters,
                1000f,
                5000f,
                50f,
                Base6Directions.Direction.Forward
            );
        }

        public static void AddPlanetToContext(PathfindingContext context, Vector3D planetCenter, double radius)
        {
            context.PlanetCenter = planetCenter;
            context.PlanetRadius = radius;
            context.isInPlanetGravity = true;
        }
    }
    public class MockGamePruningStructureDelegate : IMyGamePruningStructureDelegate
    {
        private readonly List<MyEntity> entitiesToReturn;
        private IMyPlanetDelegate planet;

        public MockGamePruningStructureDelegate()
        {
            entitiesToReturn = new List<MyEntity>();
        }

        public void AddMockEntity(MyEntity entity)
        {
            entitiesToReturn.Add(entity);
        }
        public void AddPlanetDelegate(IMyPlanetDelegate planetDelegate)
        {
            planet = planetDelegate;
        }


        public void ClearMockEntities()
        {
            entitiesToReturn.Clear();
        }

        public void GetTopmostEntitiesOverlappingRay(
            ref LineD line,
            List<MyLineSegmentOverlapResult<MyEntity>> result,
            MyEntityQueryType queryType = MyEntityQueryType.Both)
        {
            result.Clear();

            // Return mock entities
            foreach (var entity in entitiesToReturn)
            {
                result.Add(new MyLineSegmentOverlapResult<MyEntity>
                {
                    Element = entity,
                    Distance = Vector3D.Distance(line.From, entity.PositionComp.GetPosition())
                });
            }
        }
        public IMyPlanetDelegate GetClosestPlanet(Vector3D position)
        {
            return planet;
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

            // Create a minimal mock planet if needed
            // In reality, you might use Moq here too
            mockPlanet = null; // Will be null in tests, handled by altitude calculation
        }

        public void ClearMockPlanet()
        {
            mockPlanet = null;
        }

        public MyPlanet GetClosestPlanet(Vector3D position)
        {
            // Return mock planet if position is within reasonable distance
            if (mockPlanetRadius > 0)
            {
                var distance = Vector3D.Distance(position, mockPlanetCenter);
                if (distance < mockPlanetRadius * 3) // Within 3x radius
                {
                    return mockPlanet; // Can be null, handled by callers
                }
            }

            return null;
        }

        public double GetSurfaceAltitude(Vector3D position, MyPlanet planet)
        {
            // Use mock planet data regardless of actual planet object
            if (mockPlanetRadius > 0)
            {
                var distanceFromCenter = Vector3D.Distance(position, mockPlanetCenter);
                return distanceFromCenter - mockPlanetRadius;
            }

            return double.MaxValue;
        }
        public static MyEntity CreateMockObstacle(Vector3D position)
        {
            // Use Moq to create a minimal MyEntity
            var entityMock = new Mock<MyEntity>();
            var positionCompMock = new Mock<MyPositionComponentBase>();

            positionCompMock.Setup(p => p.GetPosition()).Returns(position);
            entityMock.Setup(e => e.PositionComp).Returns(positionCompMock.Object);
            entityMock.Setup(e => e.EntityId).Returns(123);

            return entityMock.Object;
        }
    }
}