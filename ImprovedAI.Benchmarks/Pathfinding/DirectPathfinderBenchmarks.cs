//using BenchmarkDotNet.Attributes;
//using ImprovedAI.Config;
//using ImprovedAI.Pathfinding;
//using ImprovedAI.Tests.TestUtil;
//using Sandbox.ModAPI;
//using System.Collections.Generic;
//using VRage.Game;
//using VRageMath;

//namespace ImprovedAI.Benchmarks.Pathfinding
//{
//    /// <summary>
//    /// Benchmarks for DirectPathfinder performance characteristics.
//    /// Tests waypoint generation under various conditions.
//    /// </summary>
//    [MemoryDiagnoser]
//    [SimpleJob(warmupCount: 3, iterationCount: 10)]
//    public class DirectPathfinderBenchmarks
//    {
//        private DirectPathfinder _pathfinder;
//        private PathfindingContext _contextNoGravity;
//        private PathfindingContext _contextWithGravity;
//        private PathfindingContext _contextWithObstacle;
//        private FakePathfindingConfig _config;

//        private Vector3D _start;
//        private Vector3D _end100;
//        private Vector3D _end500;
//        private Vector3D _end1000;
//        private Vector3D _end5000;

//        [GlobalSetup]
//        public void Setup()
//        {
//            _config = new FakePathfindingConfig
//            {
//                allowDirectPathfinding = true,
//                allowRepathing = true,
//                requireSensorsForPathfinding = false,
//                requireCamerasForPathfinding = false,
//                usePlanetAwarePathfinding = true,
//                minWaypointDistance = 25.0f,
//                maxWaypointDistance = 100.0f,
//                minAltitudeBuffer = 50.0f,
//                maxRepositionAttempts = 3
//            };
//            _pathfinder = new DirectPathfinder(_config);

//            // Setup contexts
//            _contextNoGravity = CreateBasicContext();
//            _contextWithGravity = CreateContextWithGravity(new Vector3(0, -9.81f, 0));
//            _contextWithObstacle = CreateContextWithObstacle();

//            // Setup test positions
//            _start = new Vector3D(0, 0, 0);
//            _end100 = new Vector3D(100, 0, 0);
//            _end500 = new Vector3D(500, 0, 0);
//            _end1000 = new Vector3D(1000, 0, 0);
//            _end5000 = new Vector3D(5000, 0, 0);
//        }

//        [Benchmark]
//        public void GetNextWaypoint_NoGravity_100m()
//        {
//            Vector3D vec;
//            _pathfinder.GetNextWaypoint(ref _contextNoGravity, ref _start, ref _end100, out vec);
//        }

//        [Benchmark]
//        public void GetNextWaypoint_NoGravity_500m()
//        {
//            Vector3D vec;
//            _pathfinder.GetNextWaypoint(ref _contextNoGravity, ref _start, ref _end500, out vec);
//        }

//        [Benchmark]
//        public void GetNextWaypoint_NoGravity_1000m()
//        {
//            Vector3D vec;
//            _pathfinder.GetNextWaypoint(ref _contextNoGravity, ref _start, ref _end1000, out vec);
//        }

//        [Benchmark]
//        public void GetNextWaypoint_NoGravity_5000m()
//        {
//            Vector3D vec;
//            _pathfinder.GetNextWaypoint(ref _contextNoGravity, ref _start, ref _end5000, out vec);
//        }

//        [Benchmark]
//        public void GetNextWaypoint_WithGravity_100m()
//        {
//            Vector3D vec;
//            _pathfinder.GetNextWaypoint(ref _contextWithGravity, ref _start, ref _end100, out vec);
//        }

//        [Benchmark]
//        public void GetNextWaypoint_WithGravity_500m()
//        {
//            Vector3D vec;
//            _pathfinder.GetNextWaypoint(ref _contextWithGravity, ref _start, ref _end500, out vec);
//        }

//        [Benchmark]
//        public void GetNextWaypoint_WithGravity_1000m()
//        {
//            Vector3D vec;
//            _pathfinder.GetNextWaypoint(ref _contextWithGravity, ref _start, ref _end1000, out vec);
//        }

//        [Benchmark]
//        public void GetNextWaypoint_WithObstacle_Repositioning()
//        {
//            Vector3D vec;
//            _pathfinder.GetNextWaypoint(ref _contextWithObstacle, ref _start, ref _end500, out vec);
//        }

//        [Benchmark]
//        public void CalculatePath_NoGravity_500m()
//        {
//            var pathOutput = new List<Vector3D>();
//            _pathfinder.CalculatePath(ref _contextNoGravity, ref _start, ref _end500, pathOutput);
//        }

//        [Benchmark]
//        public void CalculatePath_NoGravity_1000m()
//        {
//            var pathOutput = new List<Vector3D>();
//            _pathfinder.CalculatePath(ref _contextNoGravity, ref _start, ref _end1000, pathOutput);
//        }

//        [Benchmark]
//        public void CalculatePath_WithGravity_1000m()
//        {
//            var pathOutput = new List<Vector3D>();
//            _pathfinder.CalculatePath(ref _contextWithGravity, ref _start, ref _end1000, pathOutput);
//        }

//        #region Helper Methods

//        private PathfindingContext CreateBasicContext()
//        {
//            var controller = SEMockFactory.CreateMockController(new Vector3D(0, 0, 0));
//            var thrusters = CreateBasicThrusters();

//            return new PathfindingContext(
//                _config,
//                controller,
//                new List<IMySensorBlock>(),
//                new List<IMyCameraBlock>(),
//                thrusters,
//                1000f,
//                5000f,
//                50f,
//                Base6Directions.Direction.Forward,
//                new MockGamePruningStructureDelegate()
//            );
//        }

//        private PathfindingContext CreateContextWithGravity(Vector3 gravity)
//        {
//            var controller = SEMockFactory.CreateMockController(new Vector3D(0, 0, 0), gravity);
//            var thrusters = CreateBasicThrusters();

//            return new PathfindingContext(
//                _config,
//                controller,
//                new List<IMySensorBlock>(),
//                new List<IMyCameraBlock>(),
//                thrusters,
//                1000f,
//                5000f,
//                50f,
//                Base6Directions.Direction.Forward,
//                new MockGamePruningStructureDelegate(),
//                planetCenter: new Vector3D(0, -60000, 0),
//                planetRadius: 60000.0
//            );
//        }

//        private PathfindingContext CreateContextWithObstacle()
//        {
//            var controller = SEMockFactory.CreateMockController(new Vector3D(0, 0, 0));
//            var thrusters = CreateBasicThrusters();

//            var context = new PathfindingContext(
//                _config,
//                controller,
//                new List<IMySensorBlock>(),
//                new List<IMyCameraBlock>(),
//                thrusters,
//                1000f,
//                5000f,
//                50f,
//                Base6Directions.Direction.Forward,
//                new MockGamePruningStructureDelegate()
//            );

//            // Add obstacle in path
//            context.SensorInfos = new List<PathfindingContext.SensorInfo>
//            {
//                new PathfindingContext.SensorInfo
//                {
//                    Sensor = SEMockFactory.CreateMockSensor(new Vector3D(250, 0, 0), 50f),
//                    Position = new Vector3D(250, 0, 0),
//                    MaxRange = 50f
//                }
//            };

//            return context;
//        }

//        private List<IMyThrust> CreateBasicThrusters()
//        {
//            return new List<IMyThrust>
//            {
//                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Forward, 100000f),
//                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Backward, 100000f),
//                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Up, 100000f),
//                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Down, 100000f),
//                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Left, 100000f),
//                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Right, 100000f)
//            };
//        }

//        #endregion
//    }

//    /// <summary>
//    /// Benchmarks for DirectPathfinder parameter variations.
//    /// Tests impact of different configuration values.
//    /// </summary>
//    [MemoryDiagnoser]
//    [SimpleJob(warmupCount: 3, iterationCount: 10)]
//    public class DirectPathfinderParameterBenchmarks
//    {
//        private PathfindingContext _context;
//        private Vector3D _start;
//        private Vector3D _end;

//        [Params(1, 3, 5, 10)]
//        public int MaxRepositionAttempts;

//        [Params(25.0f, 50.0f, 100.0f, 200.0f)]
//        public float WaypointDistance;

//        [GlobalSetup]
//        public void Setup()
//        {
//            var config = new FakePathfindingConfig
//            {
//                allowDirectPathfinding = true,
//                allowRepathing = true,
//                requireSensorsForPathfinding = false,
//                requireCamerasForPathfinding = false,
//                usePlanetAwarePathfinding = false,
//                minWaypointDistance = 25.0f,
//                maxWaypointDistance = WaypointDistance,
//                maxRepositionAttempts = MaxRepositionAttempts
//            };

//            var controller = SEMockFactory.CreateMockController(new Vector3D(0, 0, 0));
//            var thrusters = new List<IMyThrust>
//            {
//                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Forward, 100000f),
//                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Backward, 100000f),
//                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Up, 100000f),
//                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Down, 100000f),
//                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Left, 100000f),
//                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Right, 100000f)
//            };

//            _context = new PathfindingContext(
//                config,
//                controller,
//                new List<IMySensorBlock>(),
//                new List<IMyCameraBlock>(),
//                thrusters,
//                1000f,
//                5000f,
//                WaypointDistance,
//                Base6Directions.Direction.Forward,
//                new MockGamePruningStructureDelegate()
//            );

//            // Add obstacle to trigger repositioning
//            _context.SensorInfos = new List<PathfindingContext.SensorInfo>
//            {
//                new PathfindingContext.SensorInfo
//                {
//                    Sensor = SEMockFactory.CreateMockSensor(new Vector3D(250, 0, 0), 50f),
//                    Position = new Vector3D(250, 0, 0),
//                    MaxRange = 50f
//                }
//            };

//            _start = new Vector3D(0, 0, 0);
//            _end = new Vector3D(500, 0, 0);
//        }

//        [Benchmark]
//        public void GetNextWaypoint_ParameterVariation()
//        {
//            var pathfinder = new DirectPathfinder((FakePathfindingConfig)_context.PathfindingConfig);
//            Vector3D vec;
//            pathfinder.GetNextWaypoint(ref _context, ref _start, ref _end, out vec);
//        }
//    }
//}