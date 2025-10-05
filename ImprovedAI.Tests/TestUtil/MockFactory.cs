using Sandbox.ModAPI;
using System.Collections.Generic;
using VRage.Game.ModAPI;
using VRageMath;
using Moq;
using VRage.Game;
using ImprovedAI.Pathfinding;
using VRage.Game.Components;

namespace ImprovedAI.Tests.TestUtil
{
    public static class SEMockFactory
    {
        public static IMyShipController CreateMockController(
            Vector3D position,
            Vector3 gravity = default(Vector3),
            bool isWorking = true,
            bool isFunctional = true,
            long entityId = 1,
            long gridEntityId = 1)
        {
            var gridMock = CreateMockGrid(gridEntityId);

            var controllerMock = new Mock<IMyShipController>();
            controllerMock.Setup(c => c.GetNaturalGravity()).Returns(gravity);
            controllerMock.Setup(c => c.CubeGrid).Returns(gridMock);

            var worldMatrix = MatrixD.Identity;
            worldMatrix.Translation = position;
            controllerMock.Setup(c => c.WorldMatrix).Returns(worldMatrix);

            controllerMock.Setup(c => c.GetPosition()).Returns(position);
            controllerMock.Setup(c => c.EntityId).Returns(entityId);
            controllerMock.Setup(c => c.DisplayName).Returns("Test Controller");
            controllerMock.Setup(c => c.IsWorking).Returns(isWorking);
            controllerMock.Setup(c => c.IsFunctional).Returns(isFunctional);
            controllerMock.Setup(c => c.Orientation).Returns(
                new MyBlockOrientation(Base6Directions.Direction.Forward, Base6Directions.Direction.Up));

            return controllerMock.Object;
        }

        public static IMyThrust CreateMockThruster(
            Base6Directions.Direction direction,
            float maxThrust,
            bool isWorking = true,
            bool isFunctional = true,
            Vector3D? position = null,
            MatrixD? worldMatrix = null,
            long entityId = 1,
            long gridEntityId = 1)
        {
            var gridMock = CreateMockGrid(gridEntityId);

            var thrustMock = new Mock<IMyThrust>();
            thrustMock.Setup(t => t.Orientation).Returns(
                new MyBlockOrientation(direction, Base6Directions.Direction.Up));
            thrustMock.Setup(t => t.GridThrustDirection).Returns(Base6Directions.GetIntVector(direction));
            thrustMock.Setup(t => t.MaxEffectiveThrust).Returns(maxThrust);
            thrustMock.Setup(t => t.MaxThrust).Returns(maxThrust);
            thrustMock.Setup(t => t.CurrentThrust).Returns(0f);
            thrustMock.Setup(t => t.IsWorking).Returns(isWorking);
            thrustMock.Setup(t => t.IsFunctional).Returns(isFunctional);
            thrustMock.Setup(t => t.CubeGrid).Returns(gridMock);
            thrustMock.Setup(t => t.EntityId).Returns(entityId);
            thrustMock.Setup(t => t.WorldMatrix).Returns(worldMatrix ?? MatrixD.Identity);
            thrustMock.Setup(t => t.GetPosition()).Returns(position ?? Vector3D.Zero);

            return thrustMock.Object;
        }

        public static IMySensorBlock CreateMockSensor(
            Vector3D position,
            float range,
            bool isWorking = true,
            bool isFunctional = true,
            long entityId = 1,
            long gridEntityId = 1)
        {
            var gridMock = CreateMockGrid(gridEntityId);

            var sensorMock = new Mock<IMySensorBlock>();
            sensorMock.Setup(s => s.GetPosition()).Returns(position);
            sensorMock.Setup(s => s.MaxRange).Returns(range);
            sensorMock.Setup(s => s.IsWorking).Returns(isWorking);
            sensorMock.Setup(s => s.IsFunctional).Returns(isFunctional);
            sensorMock.Setup(s => s.CubeGrid).Returns(gridMock);
            sensorMock.Setup(s => s.EntityId).Returns(entityId);
            sensorMock.Setup(s => s.WorldMatrix).Returns(MatrixD.CreateTranslation(position));

            return sensorMock.Object;
        }

        public static IMyCameraBlock CreateMockCamera(
            Base6Directions.Direction direction,
            bool isWorking = true,
            bool isFunctional = true,
            long entityId = 1,
            long gridEntityId = 1)
        {
            var gridMock = CreateMockGrid(gridEntityId);

            var cameraMock = new Mock<IMyCameraBlock>();

            cameraMock.Setup(c => c.Orientation).Returns(
                new MyBlockOrientation(direction, Base6Directions.Direction.Up));

            cameraMock.Setup(c => c.IsWorking).Returns(isWorking);
            cameraMock.Setup(c => c.IsFunctional).Returns(isFunctional);
            cameraMock.Setup(c => c.CubeGrid).Returns(gridMock);
            cameraMock.Setup(c => c.EntityId).Returns(entityId);

            MatrixD worldMatrix = PathfindingUtil.GetNavigationRotationMatrix(direction);
            worldMatrix.Translation = Vector3D.Zero;

            cameraMock.Setup(c => c.WorldMatrix).Returns(worldMatrix);
            cameraMock.Setup(c => c.AvailableScanRange).Returns(2000.0);
            cameraMock.Setup(c => c.CanScan(It.IsAny<double>())).Returns<double>(d => d <= 2000.0);
            cameraMock.Setup(c => c.GetPosition()).Returns(Vector3D.Zero);

            return cameraMock.Object;
        }

        public static IMyCubeGrid CreateMockGrid(long entityId = 1, float mass = 1000f)
        {
            var gridMock = new Mock<IMyCubeGrid>();
            gridMock.Setup(g => g.EntityId).Returns(entityId);
            gridMock.Setup(g => g.DisplayName).Returns($"Test Grid {entityId}");
            gridMock.Setup(g => g.GetPosition()).Returns(Vector3D.Zero);
            gridMock.Setup(g => g.GridSize).Returns(2.5f);
            gridMock.Setup(g => g.GridSizeEnum).Returns(MyCubeSize.Large);
            gridMock.Setup(g => g.IsStatic).Returns(false);
            gridMock.Setup(g => g.WorldMatrix).Returns(MatrixD.Identity);

            var physicsMock = new Mock<MyPhysicsComponentBase>();
            physicsMock.Setup(p => p.Mass).Returns(mass);
            gridMock.Setup(g => g.Physics).Returns(physicsMock.Object);

            return gridMock.Object;
        }

        public static List<IMyThrust> CreateStandardThrusterSet(
            float thrustPerDirection = 100000f,
            long baseEntityId = 100,
            long gridEntityId = 1)
        {
            var thrusters = new List<IMyThrust>();
            var directions = new[] {
                Base6Directions.Direction.Forward,
                Base6Directions.Direction.Backward,
                Base6Directions.Direction.Up,
                Base6Directions.Direction.Down,
                Base6Directions.Direction.Left,
                Base6Directions.Direction.Right
            };

            for (int i = 0; i < directions.Length; i++)
            {
                thrusters.Add(CreateMockThruster(
                    directions[i],
                    thrustPerDirection,
                    entityId: baseEntityId + i,
                    gridEntityId: gridEntityId
                ));
            }

            return thrusters;
        }
    }
}