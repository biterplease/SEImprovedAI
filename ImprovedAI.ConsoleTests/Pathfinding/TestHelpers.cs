using ImprovedAI.Pathfinding;
using Moq;
using Sandbox.ModAPI;
using System.Collections.Generic;
using VRage.Game.ModAPI;
using VRageMath;

namespace ImprovedAI.Tests
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
}