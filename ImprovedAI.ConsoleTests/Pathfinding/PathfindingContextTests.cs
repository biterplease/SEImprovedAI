using ImprovedAI.Config;
using ImprovedAI.Pathfinding;
using Moq;
using Sandbox.ModAPI;
using System;
using System.Collections.Generic;
using VRage.Game;
using VRage.Game.ModAPI;
using VRageMath;
using static VRage.Game.MyObjectBuilder_ControllerSchemaDefinition;

namespace ImprovedAI.Tests
{
    /// <summary>
    /// Unit tests for PathfindingContext
    /// These tests mock the SE API dependencies to verify context creation and properties
    /// </summary>
    public static class PathfindingContextTests
    {
        public static void RunAllTests()
        {
            Console.WriteLine("=== PathfindingContext Tests ===\n");

            int passed = 0;
            int failed = 0;

            // Basic tests
            TestBasicContextCreation(ref passed, ref failed);
            TestGravityDetection(ref passed, ref failed);
            TestThrustCapabilityCalculation(ref passed, ref failed);
            TestSensorConfiguration(ref passed, ref failed);
            TestCameraConfiguration(ref passed, ref failed);

            // Planet tests
            TestPlanetAltitudeCalculation(ref passed, ref failed);
            TestClimbCapability(ref passed, ref failed);

            // Edge cases
            TestNullInputHandling(ref passed, ref failed);
            TestZeroThrustScenario(ref passed, ref failed);

            Console.WriteLine($"\n=== Test Summary ===");
            Console.WriteLine($"Passed: {passed}");
            Console.WriteLine($"Failed: {failed}");
            Console.WriteLine($"Total:  {passed + failed}");
        }

        private static void TestBasicContextCreation(ref int passed, ref int failed)
        {
            Console.Write("Test: Basic Context Creation... ");
            try
            {
                var controller = MockFactory.CreateMockController(new Vector3D(0, 0, 0));
                var context = new PathfindingContext(
                    new FakeConfig(),
                    controller,
                    new List<IMySensorBlock>(),
                    new List<IMyCameraBlock>(),
                    new List<IMyThrust>(),
                    1000f,
                    5000f,
                    50f,
                    Base6Directions.Direction.Forward,
                    new MockGamePruningStructureDelegate()
                );

                Assert(context.Controller == controller, "Controller not set");
                Assert(context.ShipMass == 1000f, "Mass not set correctly");
                Assert(context.MaxLoad == 5000f, "MaxLoad not set correctly");
                Assert(context.WaypointDistance == 50f, "WaypointDistance not set correctly");

                Console.WriteLine("PASSED");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        private static void TestGravityDetection(ref int passed, ref int failed)
        {
            Console.Write("Test: Gravity Detection... ");
            try
            {
                var position = new Vector3D(0, 0, 0);
                var gravity = new Vector3D(0, -9.81f, 0);
                var controller = MockFactory.CreateMockController(position, gravity);

                var context = new PathfindingContext(
                    new FakeConfig(),
                    controller,
                    new List<IMySensorBlock>(),
                    new List<IMyCameraBlock>(),
                    new List<IMyThrust>(),
                    1000f,
                    5000f,
                    50f,
                    Base6Directions.Direction.Forward
                );

                Assert(context.GravityVector.LengthSquared() > 0, "Gravity not detected");
                Assert(context.IsInPlanetGravity(), "Should detect planet gravity");

                Console.WriteLine("PASSED");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        private static void TestThrustCapabilityCalculation(ref int passed, ref int failed)
        {
            Console.Write("Test: Thrust Capability Calculation... ");
            try
            {
                var position = new Vector3D(0, 0, 0);
                var gravity = new Vector3D(0, -9.81f, 0);
                var controller = MockFactory.CreateMockController(position, gravity);
                var thrusters = new List<IMyThrust>
                {
                    MockFactory.CreateMockThruster(Base6Directions.Direction.Forward, 100000f),
                    MockFactory.CreateMockThruster(Base6Directions.Direction.Backward, 50000f),
                    MockFactory.CreateMockThruster(Base6Directions.Direction.Up, 75000f)
                };

                var context = new PathfindingContext(
                    new FakeConfig(),
                    controller,
                    new List<IMySensorBlock>(),
                    new List<IMyCameraBlock>(),
                    thrusters,
                    1000f,
                    5000f,
                    50f,
                    Base6Directions.Direction.Forward
                );

                Assert(context.ThrustData != null, "ThrustData not created");
                //Assert(context.ThrustData.MaxEffectiveThrust > 0, "No effective thrust calculated");

                Console.WriteLine("PASSED");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        private static void TestSensorConfiguration(ref int passed, ref int failed)
        {
            Console.Write("Test: Sensor Configuration... ");
            try
            {
                var position = new Vector3D(0, 0, 0);
                var controller = MockFactory.CreateMockController(position);
                var sensors = new List<IMySensorBlock>
                {
                    MockFactory.CreateMockSensor(new Vector3D(0, 0, 0), 50f),
                    MockFactory.CreateMockSensor(new Vector3D(10, 0, 0), 100f)
                };

                var context = new PathfindingContext(
                    new FakeConfig(),
                    controller,
                    sensors,
                    new List<IMyCameraBlock>(),
                    new List<IMyThrust>(),
                    1000f,
                    5000f,
                    50f,
                    Base6Directions.Direction.Forward
                );

                Assert(context.SensorInfos != null, "SensorInfos not created");
                Assert(context.SensorInfos.Count == 2, "Sensor count mismatch");

                Console.WriteLine("PASSED");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        private static void TestCameraConfiguration(ref int passed, ref int failed)
        {
            Console.Write("Test: Camera Configuration... ");
            try
            {
                var position = new Vector3D(0, 0, 0);
                var controller = MockFactory.CreateMockController(position);
                var cameras = new List<IMyCameraBlock>
                {
                    MockFactory.CreateMockCamera(Base6Directions.Direction.Forward),
                    MockFactory.CreateMockCamera(Base6Directions.Direction.Up)
                };

                var context = new PathfindingContext(
                    new FakeConfig(),
                    controller,
                    new List<IMySensorBlock>(),
                    cameras,
                    new List<IMyThrust>(),
                    1000f,
                    5000f,
                    50f,
                    Base6Directions.Direction.Forward
                );

                Assert(context.CamerasByDirection != null, "CamerasByDirection not created");
                Assert(context.CamerasByDirection.ContainsKey(Base6Directions.Direction.Forward),
                    "Forward camera not organized");

                Console.WriteLine("PASSED");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        private static void TestPlanetAltitudeCalculation(ref int passed, ref int failed)
        {
            Console.Write("Test: Planet Altitude Calculation... ");
            try
            {
                var planetCenter = new Vector3D(0, 0, 0);
                var planetRadius = 60000.0;

                var position = new Vector3D(0, planetRadius + 1000, 0);
                var gravity = new Vector3D(0, -9.81f, 0);
                var controller = MockFactory.CreateMockController(position, gravity);

                var context = new PathfindingContext(
                    new FakeConfig(),
                    controller,
                    new List<IMySensorBlock>(),
                    new List<IMyCameraBlock>(),
                    new List<IMyThrust>(),
                    1000f,
                    5000f,
                    50f,
                    Base6Directions.Direction.Forward
                );

                var altitude = context.GetSurfaceAltitude();
                Assert(altitude.HasValue, "Altitude not calculated");
                Assert(Math.Abs(altitude.Value - 1000.0) < 1.0,
                    $"Altitude incorrect: expected ~1000, got {altitude.Value}");

                Console.WriteLine("PASSED");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        private static void TestClimbCapability(ref int passed, ref int failed)
        {
            Console.Write("Test: Climb Capability... ");
            try
            {
                var position = new Vector3D(0, 0, 0);
                var gravity = new Vector3D(0, -9.81f, 0);
                var controller = MockFactory.CreateMockController(position, gravity);

                // Create thrusters with upward thrust
                var thrusters = new List<IMyThrust>
                {
                    MockFactory.CreateMockThruster(Base6Directions.Direction.Up, 200000f), // Strong upward
                    MockFactory.CreateMockThruster(Base6Directions.Direction.Down, 50000f)
                };

                var context = new PathfindingContext(
                    new FakeConfig(),
                    controller,
                    new List<IMySensorBlock>(),
                    new List<IMyCameraBlock>(),
                    thrusters,
                    1000f,
                    5000f,
                    50f,
                    Base6Directions.Direction.Forward
                );

                // Test climbing upward
                var upDirection = new Vector3D(0, 100, 0);
                Assert(context.CanClimbInDirection(upDirection), "Should be able to climb with strong thrust");

                // Test descending (should always work with gravity)
                var downDirection = new Vector3D(0, -100, 0);
                Assert(context.CanClimbInDirection(downDirection), "Should be able to descend");

                Console.WriteLine("PASSED");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        private static void TestNullInputHandling(ref int passed, ref int failed)
        {
            Console.Write("Test: Null Input Handling... ");
            try
            {
                var position = new Vector3D(0, 0, 0);
                var controller = MockFactory.CreateMockController(position);

                // Should handle null lists gracefully
                var context = new PathfindingContext(
                    new FakeConfig(),
                    controller,
                    null,
                    null,
                    null,
                    1000f,
                    5000f,
                    50f,
                    Base6Directions.Direction.Forward
                );

                Assert(context.SensorInfos == null || context.SensorInfos.Count == 0,
                    "Should handle null sensors");
                Assert(context.CamerasByDirection == null || context.CamerasByDirection.Count == 0,
                    "Should handle null cameras");

                Console.WriteLine("PASSED");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        private static void TestZeroThrustScenario(ref int passed, ref int failed)
        {
            Console.Write("Test: Zero Thrust Scenario... ");
            try
            {
                var position = new Vector3D(0, 0, 0);
                var controller = MockFactory.CreateMockController(position);

                var context = new PathfindingContext(
                    new FakeConfig(),
                    controller,
                    new List<IMySensorBlock>(),
                    new List<IMyCameraBlock>(),
                    new List<IMyThrust>(), // No thrusters
                    1000f,
                    5000f,
                    50f,
                    Base6Directions.Direction.Forward
                );

                // Should not crash, but climbing should fail
                var upDirection = new Vector3D(0, 100, 0);
                Assert(!context.CanClimbInDirection(upDirection),
                    "Should not be able to climb with no thrust");

                Console.WriteLine("PASSED");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        private static void Assert(bool condition, string message)
        {
            if (!condition)
                throw new Exception(message);
        }
    }

    #region Mock Helper Factory

    public static class MockFactory
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

            var cameraMock = new Mock<IMyCameraBlock>();
            cameraMock.Setup(c => c.Orientation).Returns(
                new MyBlockOrientation(direction, Base6Directions.Direction.Up));
            cameraMock.Setup(c => c.IsWorking).Returns(true);
            cameraMock.Setup(c => c.IsFunctional).Returns(true);
            cameraMock.Setup(c => c.CubeGrid).Returns(gridMock.Object);
            cameraMock.Setup(c => c.EntityId).Returns(1);
            cameraMock.Setup(c => c.WorldMatrix).Returns(MatrixD.Identity);
            cameraMock.Setup(c => c.AvailableScanRange).Returns(2000.0);
            cameraMock.Setup(c => c.CanScan(It.IsAny<double>())).Returns<double>(d => d <= 2000.0);
            cameraMock.Setup(c => c.GetPosition()).Returns(Vector3D.Zero);

            return cameraMock.Object;
        }
    }

    #endregion
}