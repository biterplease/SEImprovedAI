using Sandbox.ModAPI;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRage.Game.ModAPI;
using VRageMath;
using Moq;
using VRage.Game;
using ImprovedAI.Pathfinding;
using ImprovedAI.Utils.Logging;

namespace ImprovedAI.Tests.TestUtil
{

    public static class SEMockFactory
    {
        public static IMyShipController CreateMockController(
            Vector3D position,
            Vector3 gravity = default(Vector3),
            bool isWorking = true,
            bool isFunctional = true
        )
        {
            var gridMock = new Mock<IMyCubeGrid>();
            gridMock.Setup(g => g.EntityId).Returns(1);
            gridMock.Setup(g => g.DisplayName).Returns("Test Grid");
            gridMock.Setup(g => g.GetPosition()).Returns(Vector3D.Zero);
            gridMock.Setup(g => g.GridSize).Returns(2.5f);
            gridMock.Setup(g => g.GridSizeEnum).Returns(MyCubeSize.Large);
            gridMock.Setup(g => g.IsStatic).Returns(false);
            gridMock.Setup(g => g.WorldMatrix).Returns(MatrixD.Identity);

            var controllerMock = new Mock<IMyShipController>();
            controllerMock.Setup(c => c.GetNaturalGravity()).Returns(gravity);
            controllerMock.Setup(c => c.CubeGrid).Returns(gridMock.Object);

            // FIX: Use identity matrix with translation, not just CreateTranslation
            var worldMatrix = MatrixD.Identity;
            worldMatrix.Translation = position;
            controllerMock.Setup(c => c.WorldMatrix).Returns(worldMatrix);

            controllerMock.Setup(c => c.GetPosition()).Returns(position);
            controllerMock.Setup(c => c.EntityId).Returns(1);
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
            bool isWorking=true,
            bool isFunctional=true,
            Vector3D? position = null,
            MatrixD? worldMatrix = null)
        {
            var gridMock = new Mock<IMyCubeGrid>();
            gridMock.Setup(g => g.EntityId).Returns(1);

            var thrustMock = new Mock<IMyThrust>();
            thrustMock.Setup(t => t.Orientation).Returns(
                new MyBlockOrientation(direction, Base6Directions.Direction.Up));
            thrustMock.Setup(t => t.MaxEffectiveThrust).Returns(maxThrust);
            thrustMock.Setup(t => t.MaxThrust).Returns(maxThrust);
            thrustMock.Setup(t => t.CurrentThrust).Returns(0f);
            thrustMock.Setup(t => t.IsWorking).Returns(isWorking);
            thrustMock.Setup(t => t.IsFunctional).Returns(isFunctional);
            thrustMock.Setup(t => t.CubeGrid).Returns(gridMock.Object);
            thrustMock.Setup(t => t.EntityId).Returns(1);
            thrustMock.Setup(t => t.WorldMatrix).Returns(worldMatrix != null ? worldMatrix.Value : MatrixD.Identity);
            thrustMock.Setup(t => t.GetPosition()).Returns(position != null ? position.Value : Vector3D.Zero);

            return thrustMock.Object;
        }

        public static IMySensorBlock CreateMockSensor(Vector3D position, float range)
        {
            var gridMock = new Mock<IMyCubeGrid>();
            gridMock.Setup(g => g.EntityId).Returns(1);

            var sensorMock = new Mock<IMySensorBlock>();
            sensorMock.Setup(s => s.GetPosition()).Returns(position);
            sensorMock.Setup(s => s.MaxRange).Returns(range);
            sensorMock.Setup(s => s.IsWorking).Returns(true);
            sensorMock.Setup(s => s.IsFunctional).Returns(true);
            sensorMock.Setup(s => s.CubeGrid).Returns(gridMock.Object);
            sensorMock.Setup(s => s.EntityId).Returns(1);
            sensorMock.Setup(s => s.WorldMatrix).Returns(MatrixD.CreateTranslation(position));

            return sensorMock.Object;
        }

        public static IMyCameraBlock CreateMockCamera(Base6Directions.Direction direction)
        {
            var gridMock = new Mock<IMyCubeGrid>();
            gridMock.Setup(g => g.EntityId).Returns(1);
            // Grid is at identity orientation
            gridMock.Setup(g => g.WorldMatrix).Returns(MatrixD.Identity);

            var cameraMock = new Mock<IMyCameraBlock>();

            // Camera's orientation in grid-local space
            // Cameras always have their "forward" as the direction they face
            // and we'll use Up as the up orientation (standard orientation)
            cameraMock.Setup(c => c.Orientation).Returns(
                new MyBlockOrientation(direction, Base6Directions.Direction.Up));

            cameraMock.Setup(c => c.IsWorking).Returns(true);
            cameraMock.Setup(c => c.IsFunctional).Returns(true);
            cameraMock.Setup(c => c.CubeGrid).Returns(gridMock.Object);
            cameraMock.Setup(c => c.EntityId).Returns(1);

            // Camera's world matrix = grid world matrix * local orientation matrix
            // Since grid is at identity, camera world matrix = local orientation matrix
            // Use GetNavigationRotationMatrix to get the correct rotation
            MatrixD worldMatrix = PathfindingUtil.GetNavigationRotationMatrix(direction);
            worldMatrix.Translation = Vector3D.Zero;

            cameraMock.Setup(c => c.WorldMatrix).Returns(worldMatrix);
            cameraMock.Setup(c => c.AvailableScanRange).Returns(2000.0);
            cameraMock.Setup(c => c.CanScan(It.IsAny<double>())).Returns<double>(d => d <= 2000.0);
            cameraMock.Setup(c => c.GetPosition()).Returns(Vector3D.Zero);

            return cameraMock.Object;
        }
    }
}