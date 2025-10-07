//using BenchmarkDotNet.Attributes;
//using ImprovedAI.Pathfinding;
//using ImprovedAI.Tests.TestUtil;
//using Sandbox.ModAPI;
//using System.Collections.Generic;
//using VRageMath;

//namespace ImprovedAI.Benchmarks.Pathfinding
//{
//    /// <summary>
//    /// Benchmarks for AStarPathfinder performance characteristics.
//    /// Tests path calculation under various conditions and complexities.
//    /// </summary>
//    [MemoryDiagnoser]
//    [SimpleJob(warmupCount: 3, iterationCount: 10)]
//    public class AStarPathfinderBenchmarks
//    {
//        private AStarPathfinder _pathfinder;
//        private PathfindingContext _contextNoObstacles;
//        private PathfindingContext _contextSingleObstacle;
//        private PathfindingContext _contextMultipleObstacles;
//        private PathfindingContext _contextWithGravity;
//        private FakePathfindingConfig _config;

//        private Vector3D _start;
//        private Vector3D _end500;
//        private Vector3D _end1000;
//        private Vector3D _end2000;

//        [GlobalSetup]
//        public void Setup()
//        {
//            _config = new FakePathfindingConfig
//            {
//                allowAStar = true,
//                allowRepathing = true,
//                requireSensorsForPathfinding = false,
//                requireCamerasForPathfinding = false,
//                usePlanetAwarePathfinding = true,
//                minWaypointDistance = 25.0f,
//                maxWaypointDistance = 100.0f,
//                minAltitudeBuffer = 50.0f,
//                maxPathNodes = 1000,
//                maxRepositionAttempts = 3
//            };
//            _pathfinder = new AStarPathfinder(_config);

//            // Setup contexts
//            _contextNoObstacles = CreateBasicContext();
//            _contextSingleObstacle = CreateContextWithSingleObstacle();
//            _contextMultipleObstacles = CreateContextWithMultipleObstacles();
//            _contextWithGravity = CreateContextWithGravity(new Vector3(0, -9.81f, 0));

//            // Setup test positions
//            _start = new Vector3D(0, 0, 0);
//            _end500 = new Vector3D(500, 0, 0);
//            _end1000 = new Vector3D(1000, 0, 0);
//            _end2000 = new Vector3D(2000, 0, 0);
//        }

//        [Benchmark]
//        public void EstimatedComplexity_500m()
//        {
//            _pathfinder.EstimatedComplexity(ref _start, ref _end500);
//        }

//        [Benchmark]
//        public void EstimatedComplexity_1000m()
//        {
//            _pathfinder.EstimatedComplexity(ref _start, ref _end1000);
//        }

//        [Benchmark]
//        public void EstimatedComplexity_2000m()
//        {
//            _pathfinder.EstimatedComplexity(ref _start, ref _end2000);
//        }

//        [Benchmark]
//        public void GetNextWaypoint_NoObstacles_500m()
//        {
//            Vector3D vec;
//            _pathfinder.GetNextWaypoint(ref _contextNoObstacles, ref _start, ref _end500, out vec);
//        }

//        [Benchmark]
//        public void GetNextWaypoint_NoObstacles_1000m()
//        {
//            Vector3D vec;
//            _pathfinder.GetNextWaypoint(ref _contextNoObstacles, ref _start, ref _end1000, out vec);
//        }

//        [Benchmark]
//        public void GetNextWaypoint_SingleObstacle_1000m()
//        {
//            Vector3D vec;
//            _pathfinder.GetNextWaypoint(ref _contextSingleObstacle, ref _start, ref _end1000, out vec);
//        }

//        [Benchmark]
//        public void GetNextWaypoint_MultipleObstacles_1000m()
//        {
//            Vector3D vec;
//            _pathfinder.GetNextWaypoint(ref _contextMultipleObstacles, ref _start, ref _end1000, out vec);
//        }

//        [Benchmark]
//        public void GetNextWaypoint_WithGravity_1000m()
//        {
//            Vector3D vec;
//            _pathfinder.GetNextWaypoint(ref _contextWithGravity, ref _start, ref _end1000, out vec);
//        }

//        [Benchmark]
//        public void CalculatePath_NoObstacles_500m()
//        {
//            var pathOutput = new List<Vector3D>();
//            _pathfinder.CalculatePath(ref _contextNoObstacles, ref _start, ref _end500, pathOutput);
//        }

//        [Benchmark]
//        public void CalculatePath_NoObstacles_1000m()
//        {
//            var pathOutput = new List<Vector3D>();
//            _pathfinder.CalculatePath(ref _contextNoObstacles, ref _start, ref _end1000, pathOutput);
//        }

//        [Benchmark]
//        public void CalculatePath_NoObstacles_2000m()
//        {
//            var pathOutput = new List<Vector3D>();
//            _pathfinder.CalculatePath(ref _contextNoObstacles, ref _start, ref _end2000, pathOutput);
//        }

//        [Benchmark]
//        public void CalculatePath_SingleObstacle_1000m()
//        {
//            var pathOutput = new List<Vector3D>();
//            _pathfinder.CalculatePath(ref _contextSingleObstacle, ref _start, ref _end1000, pathOutput);
//        }

//        [Benchmark]
//        public void CalculatePath_MultipleObstacles_1000m()
//        {
//            var pathOutput = new List<Vector3D>();
//            _pathfinder.CalculatePath(ref _contextMultipleObstacles, ref _start, ref _end1000, pathOutput);
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

//        private PathfindingContext CreateContextWithSingleObstacle()
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

//            context.SensorInfos = new List<PathfindingContext.SensorInfo>
//            {
//                new PathfindingContext.SensorInfo
//                {
//                    Sensor = SEMockFactory.CreateMockSensor(new Vector3D(500, 0, 0), 100f),
//                    Position = new Vector3D(500, 0, 0),
//                    MaxRange = 100f
//                }
//            };

//            return context;
//        }

//        private PathfindingContext CreateContextWithMultipleObstacles()
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

//            context.SensorInfos = new List<PathfindingContext.SensorInfo>
//            {
//                new PathfindingContext.SensorInfo
//                {
//                    Sensor = SEMockFactory.CreateMockSensor(new Vector3D(300, 0, 0), 80f),
//                    Position = new Vector3D(300, 0, 0),
//                    MaxRange = 80f
//                },
//                new PathfindingContext.SensorInfo
//                {
//                    Sensor = SEMockFactory.CreateMockSensor(new Vector3D(600, 0, 0), 80f),
//                    Position = new Vector3D(600, 0, 0),
//                    MaxRange = 80f
//                },
//                new PathfindingContext.SensorInfo
//                {
//                    Sensor = SEMockFactory.CreateMockSensor(new Vector3D(900, 0, 0), 80f),
//                    Position = new Vector3D(900, 0, 0),
//                    MaxRange = 80f
//                }
//            };

//            return context;
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
//    /// Benchmarks for A* pathfinder with varying configuration parameters.
//    /// Tests impact of node limits and waypoint distances.
//    /// </summary>
//    [MemoryDiagnoser]
//    [SimpleJob(warmupCount: 3, iterationCount: 10)]
//    public class AStarParameterBenchmarks
//    {
//        private PathfindingContext _context;
//        private Vector3D _start;
//        private Vector3D _end;

//        [Params(100, 500, 1000, 2000)]
//        public int MaxPathNodes;

//        [Params(50.0f, 100.0f, 200.0f)]
//        public float WaypointDistance;

//        [GlobalSetup]
//        public void Setup()
//        {
//            var config = new FakePathfindingConfig
//            {
//                allowAStar = true,
//                allowRepathing = true,
//                requireSensorsForPathfinding = false,
//                requireCamerasForPathfinding = false,
//                usePlanetAwarePathfinding = false,
//                minWaypointDistance = 25.0f,
//                maxWaypointDistance = WaypointDistance,
//                maxPathNodes = MaxPathNodes,
//                maxRepositionAttempts = 3
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

//            // Add obstacles to make pathfinding non-trivial
//            _context.SensorInfos = new List<PathfindingContext.SensorInfo>
//            {
//                new PathfindingContext.SensorInfo
//                {
//                    Sensor = SEMockFactory.CreateMockSensor(new Vector3D(500, 0, 0), 80f),
//                    Position = new Vector3D(500, 0, 0),
//                    MaxRange = 80f
//                }
//            };

//            _start = new Vector3D(0, 0, 0);
//            _end = new Vector3D(1000, 0, 0);
//        }

//        [Benchmark]
//        public void CalculatePath_ParameterVariation()
//        {
//            var pathfinder = new AStarPathfinder((FakePathfindingConfig)_context.PathfindingConfig);
//            var pathOutput = new List<Vector3D>();
//            pathfinder.CalculatePath(ref _context, ref _start, ref _end, pathOutput);
//        }
//    }
//}