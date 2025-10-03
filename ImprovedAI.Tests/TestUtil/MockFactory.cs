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

namespace ImprovedAI.TestUtil
{

    public static class SEMockFactory
    {
        public static IMyShipController CreateMockController(Vector3D position, Vector3 gravity = default(Vector3))
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
            controllerMock.Setup(c => c.WorldMatrix).Returns(MatrixD.CreateTranslation(position));
            controllerMock.Setup(c => c.GetPosition()).Returns(position);
            controllerMock.Setup(c => c.EntityId).Returns(1);
            controllerMock.Setup(c => c.DisplayName).Returns("Test Controller");
            controllerMock.Setup(c => c.IsWorking).Returns(true);
            controllerMock.Setup(c => c.IsFunctional).Returns(true);
            controllerMock.Setup(c => c.Orientation).Returns(
                new MyBlockOrientation(Base6Directions.Direction.Forward, Base6Directions.Direction.Up));

            return controllerMock.Object;
        }

        public static IMyThrust CreateMockThruster(Base6Directions.Direction direction, float maxThrust)
        {
            var gridMock = new Mock<IMyCubeGrid>();
            gridMock.Setup(g => g.EntityId).Returns(1);

            var thrustMock = new Mock<IMyThrust>();
            thrustMock.Setup(t => t.Orientation).Returns(
                new MyBlockOrientation(direction, Base6Directions.Direction.Up));
            thrustMock.Setup(t => t.MaxEffectiveThrust).Returns(maxThrust);
            thrustMock.Setup(t => t.MaxThrust).Returns(maxThrust);
            thrustMock.Setup(t => t.CurrentThrust).Returns(0f);
            thrustMock.Setup(t => t.IsWorking).Returns(true);
            thrustMock.Setup(t => t.IsFunctional).Returns(true);
            thrustMock.Setup(t => t.CubeGrid).Returns(gridMock.Object);
            thrustMock.Setup(t => t.EntityId).Returns(1);
            thrustMock.Setup(t => t.WorldMatrix).Returns(MatrixD.Identity);
            thrustMock.Setup(t => t.GetPosition()).Returns(Vector3D.Zero);

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

            // Create a world matrix for the grid that rotates it
            MatrixD gridWorldMatrix = PathfindingUtil.GetNavigationRotationMatrix(direction);
            gridMock.Setup(g => g.WorldMatrix).Returns(gridWorldMatrix);

            var cameraMock = new Mock<IMyCameraBlock>();

            // Block orientation should be in LOCAL grid space - always Forward/Up for a "standard" camera
            cameraMock.Setup(c => c.Orientation).Returns(
                new MyBlockOrientation(Base6Directions.Direction.Forward, Base6Directions.Direction.Up));

            cameraMock.Setup(c => c.IsWorking).Returns(true);
            cameraMock.Setup(c => c.IsFunctional).Returns(true);
            cameraMock.Setup(c => c.CubeGrid).Returns(gridMock.Object);
            cameraMock.Setup(c => c.EntityId).Returns(1);

            // Camera's world matrix = grid's world matrix (since camera is at grid origin with standard orientation)
            // If you need the camera rotated relative to the grid, compose transformations here
            cameraMock.Setup(c => c.WorldMatrix).Returns(gridWorldMatrix);

            cameraMock.Setup(c => c.AvailableScanRange).Returns(2000.0);
            cameraMock.Setup(c => c.CanScan(It.IsAny<double>())).Returns<double>(d => d <= 2000.0);
            cameraMock.Setup(c => c.GetPosition()).Returns(Vector3D.Zero);

            return cameraMock.Object;
        }
    }
}