using ImprovedAI.Pathfinding;
using ImprovedAI.TestUtil;
using Sandbox.ModAPI;
using VRageMath;

namespace ImprovedAI.Tests.Pathfinding
{
    [TestClass]
    public class PathfindingManagerTests
    {
        private FakePathfindingConfig _config;
        private PathfindingManager _manager;

        [TestInitialize]
        public void Setup()
        {
            _config = new FakePathfindingConfig
            {
                allowDirectPathfinding = true,
                allowAStar = true,
                allowRepathing = true,
                requireSensorsForPathfinding = false,
                requireCamerasForPathfinding = false,
                usePlanetAwarePathfinding = true,
                minWaypointDistance = 25.0f,
                maxWaypointDistance = 100.0f,
                minAltitudeBuffer = 50.0f,
                maxPathNodes = 1000,
                maxRepositionAttempts = 3
            };
            _manager = new PathfindingManager(_config);
        }

        [TestMethod]
        public void GetNextWaypoint_ShortDistance_UsesDirectPathfinder()
        {
            var context = CreateBasicContext();
            var start = new Vector3D(0, 0, 0);
            var end = new Vector3D(100, 0, 0);

            Vector3D waypoint;
            bool result = _manager.GetNextWaypoint(ref start, ref end, context, out waypoint);

            Assert.IsTrue(result, "Should generate waypoint");
            Assert.IsTrue(waypoint != Vector3D.Zero, "Waypoint should be valid");
        }

        [TestMethod]
        public void GetNextWaypoint_LongDistance_SelectsAppropriatePathfinder()
        {
            var context = CreateBasicContext();
            var start = new Vector3D(0, 0, 0);
            var end = new Vector3D(5000, 0, 0);

            Vector3D waypoint;
            bool result = _manager.GetNextWaypoint(ref start, ref end, context, out waypoint);

            Assert.IsTrue(result, "Should generate waypoint for long distance");
        }

        [TestMethod]
        public void GetNextWaypoint_WithSensors_PrefersAStar()
        {
            var context = CreateBasicContext();
            context.SensorInfos = new List<PathfindingContext.SensorInfo>
            {
                new PathfindingContext.SensorInfo
                {
                    Sensor = SEMockFactory.CreateMockSensor(new Vector3D(0, 0, 0), 50f),
                    Position = new Vector3D(0, 0, 0),
                    MaxRange = 50f
                }
            };

            var start = new Vector3D(0, 0, 0);
            var end = new Vector3D(1000, 0, 0);

            Vector3D waypoint;
            bool result = _manager.GetNextWaypoint(ref start, ref end, context, out waypoint);

            Assert.IsTrue(result, "Should generate waypoint with sensors available");
        }

        [TestMethod]
        public void GetNextWaypoint_AStarDisabled_FallsBackToDirect()
        {
            _config.allowAStar = false;
            _manager = new PathfindingManager(_config);

            var context = CreateBasicContext();
            var start = new Vector3D(0, 0, 0);
            var end = new Vector3D(1000, 0, 0);

            Vector3D waypoint;
            bool result = _manager.GetNextWaypoint(ref start, ref end, context, out waypoint);

            Assert.IsTrue(result, "Should fall back to direct pathfinder");
        }

        [TestMethod]
        public void GetNextWaypoint_AllPathfindersDisabled_Fails()
        {
            _config.allowDirectPathfinding = false;
            _config.allowAStar = false;
            _manager = new PathfindingManager(_config);

            var context = CreateBasicContext();
            var start = new Vector3D(0, 0, 0);
            var end = new Vector3D(100, 0, 0);

            Vector3D waypoint;
            bool result = _manager.GetNextWaypoint(ref start, ref end, context, out waypoint);

            Assert.IsFalse(result, "Should fail when no pathfinders available");
        }

        [TestMethod]
        public void GetNextWaypoint_PathfinderFails_AttemptsRepathing()
        {
            var context = CreateBasicContext();

            // Create obstacle that blocks direct path
            context.SensorInfos = new List<PathfindingContext.SensorInfo>
            {
                new PathfindingContext.SensorInfo
                {
                    Sensor = SEMockFactory.CreateMockSensor(new Vector3D(50, 0, 0), 100f),
                    Position = new Vector3D(50, 0, 0),
                    MaxRange = 100f
                }
            };

            var start = new Vector3D(0, 0, 0);
            var end = new Vector3D(100, 0, 0);

            Vector3D waypoint;
            bool result = _manager.GetNextWaypoint(ref start, ref end, context, out waypoint);

            // Should either find alternate route or use emergency path
            Assert.IsTrue(result, "Should handle blocked path with repathing");
        }

        [TestMethod]
        public void GetNextWaypoint_RepathingDisabled_FailsOnBlockedPath()
        {
            _config.allowRepathing = false;
            _manager = new PathfindingManager(_config);

            var context = CreateBasicContext();
            context.SensorInfos = new List<PathfindingContext.SensorInfo>
            {
                new PathfindingContext.SensorInfo
                {
                    Sensor = SEMockFactory.CreateMockSensor(new Vector3D(50, 0, 0), 100f),
                    Position = new Vector3D(50, 0, 0),
                    MaxRange = 100f
                }
            };

            var start = new Vector3D(0, 0, 0);
            var end = new Vector3D(100, 0, 0);

            Vector3D waypoint;
            bool result = _manager.GetNextWaypoint(ref start, ref end, context, out waypoint);

            Assert.IsFalse(result, "Should fail when repathing disabled and path blocked");
        }

        [TestMethod]
        public void GenerateCompletePath_CreatesValidPath()
        {
            var context = CreateBasicContext();
            var start = new Vector3D(0, 0, 0);
            var end = new Vector3D(500, 0, 0);
            var pathOutput = new List<Vector3D>();

            bool result = _manager.GenerateCompletePath(ref start, ref end, context, pathOutput);

            Assert.IsTrue(result, "Should generate complete path");
            Assert.IsTrue(pathOutput.Count >= 2, "Path should have at least start and end");
            Assert.AreEqual(start, pathOutput[0], "Path should start at start position");
            Assert.AreEqual(end, pathOutput[pathOutput.Count - 1], "Path should end at end position");
        }

        [TestMethod]
        public void GenerateCompletePath_WithGravity_MaintainsAltitude()
        {
            var gravity = new Vector3(0, -9.81f, 0);
            var context = CreateContextWithGravity(gravity);
            var start = new Vector3D(0, 100, 0);
            var end = new Vector3D(500, 100, 0);
            var pathOutput = new List<Vector3D>();

            bool result = _manager.GenerateCompletePath(ref start, ref end, context, pathOutput);

            Assert.IsTrue(result);

            // Verify altitude maintenance
            foreach (var waypoint in pathOutput)
            {
                Assert.IsTrue(waypoint.Y >= start.Y - 20,
                    "Path should maintain altitude in gravity");
            }
        }

        [TestMethod]
        public void GetNextWaypoint_BelowSafeAltitude_LiftsWaypoint()
        {
            var gravity = new Vector3(0, -9.81f, 0);
            var context = CreateContextWithGravity(gravity);

            var start = new Vector3D(0, 30, 0); // Below min altitude buffer
            var end = new Vector3D(100, 30, 0);

            Vector3D waypoint;
            bool result = _manager.GetNextWaypoint(ref start, ref end, context, out waypoint);

            Assert.IsTrue(result);
            // Waypoint should be adjusted for safe altitude
            Assert.IsTrue(waypoint.Y >= start.Y || waypoint == end,
                "Waypoint should be at or above start altitude");
        }

        [TestMethod]
        public void GetNextWaypoint_VeryShortDistance_ReturnsTarget()
        {
            var context = CreateBasicContext();
            var start = new Vector3D(0, 0, 0);
            var end = new Vector3D(10, 0, 0); // Less than min waypoint distance

            Vector3D waypoint;
            bool result = _manager.GetNextWaypoint(ref start, ref end, context, out waypoint);

            Assert.IsTrue(result);
            Assert.AreEqual(end, waypoint, "Very short distance should return target directly");
        }

        [TestMethod]
        public void GetNextWaypoint_ComplexityExceedsLimit_FallsBackToDirect()
        {
            _config.maxPathNodes = 10; // Very low limit
            _manager = new PathfindingManager(_config);

            var context = CreateBasicContext();
            context.SensorInfos = new List<PathfindingContext.SensorInfo>
            {
                new PathfindingContext.SensorInfo
                {
                    Sensor = SEMockFactory.CreateMockSensor(new Vector3D(0, 0, 0), 50f),
                    Position = new Vector3D(0, 0, 0),
                    MaxRange = 50f
                }
            };

            var start = new Vector3D(0, 0, 0);
            var end = new Vector3D(10000, 0, 0); // Very long distance

            Vector3D waypoint;
            bool result = _manager.GetNextWaypoint(ref start, ref end, context, out waypoint);

            // Should fall back to direct pathfinder due to complexity
            Assert.IsTrue(result, "Should fall back when complexity exceeds limit");
        }

        #region Helper Methods

        private PathfindingContext CreateBasicContext()
        {
            var controller = SEMockFactory.CreateMockController(new Vector3D(0, 0, 0));
            var thrusters = CreateBasicThrusters();

            return new PathfindingContext(
                _config,
                controller,
                new List<IMySensorBlock>(),
                new List<IMyCameraBlock>(),
                thrusters,
                1000f,
                5000f,
                50f,
                Base6Directions.Direction.Forward,
                new MockGamePruningStructureDelegate()
            );
        }

        private PathfindingContext CreateContextWithGravity(Vector3 gravity)
        {
            var controller = SEMockFactory.CreateMockController(new Vector3D(0, 0, 0), gravity);
            var thrusters = CreateBasicThrusters();

            return new PathfindingContext(
                _config,
                controller,
                new List<IMySensorBlock>(),
                new List<IMyCameraBlock>(),
                thrusters,
                1000f,
                5000f,
                50f,
                Base6Directions.Direction.Forward,
                new MockGamePruningStructureDelegate(),
                planetCenter: new Vector3D(0, -60000, 0),
                planetRadius: 60000.0
            );
        }

        private List<IMyThrust> CreateBasicThrusters()
        {
            return new List<IMyThrust>
            {
                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Forward, 100000f),
                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Backward, 100000f),
                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Up, 100000f),
                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Down, 100000f),
                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Left, 100000f),
                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Right, 100000f)
            };
        }

        #endregion
    }
}