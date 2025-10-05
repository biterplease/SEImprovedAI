using BenchmarkDotNet.Attributes;
using ImprovedAI.Config;
using ImprovedAI.Pathfinding;
using ImprovedAI.Tests.TestUtil;
using Sandbox.ModAPI;
using System.Collections.Generic;
using VRageMath;

namespace ImprovedAI.Benchmarks.Pathfinding
{
    /// <summary>
    /// Benchmarks for PathfindingManager pathfinder selection and coordination.
    /// Tests overhead of manager decision-making and fallback logic.
    /// </summary>
    [MemoryDiagnoser]
    [SimpleJob(warmupCount: 3, iterationCount: 10)]
    public class PathfindingManagerBenchmarks
    {
        private PathfindingManager _managerDirectOnly;
        private PathfindingManager _managerWithAStar;
        private PathfindingContext _contextNoSensors;
        private PathfindingContext _contextWithSensors;
        private PathfindingContext _contextWithGravity;

        private Vector3D _start;
        private Vector3D _endShort;
        private Vector3D _endMedium;
        private Vector3D _endLong;

        [GlobalSetup]
        public void Setup()
        {
            // Manager with only direct pathfinding
            var configDirectOnly = new FakePathfindingConfig
            {
                allowDirectPathfinding = true,
                allowAStar = false,
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
            _managerDirectOnly = new PathfindingManager(configDirectOnly);

            // Manager with A* enabled
            var configWithAStar = new FakePathfindingConfig
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
            _managerWithAStar = new PathfindingManager(configWithAStar);

            // Setup contexts
            _contextNoSensors = CreateBasicContext(configDirectOnly);
            _contextWithSensors = CreateContextWithSensors(configWithAStar);
            _contextWithGravity = CreateContextWithGravity(configWithAStar, new Vector3(0, -9.81f, 0));

            // Setup test positions
            _start = new Vector3D(0, 0, 0);
            _endShort = new Vector3D(100, 0, 0);
            _endMedium = new Vector3D(500, 0, 0);
            _endLong = new Vector3D(2000, 0, 0);
        }

        [Benchmark]
        public void GetNextWaypoint_DirectOnly_Short()
        {
            Vector3D vec;
            _managerDirectOnly.GetNextWaypoint(ref _start, ref _endShort, _contextNoSensors, out vec);
        }

        [Benchmark]
        public void GetNextWaypoint_DirectOnly_Medium()
        {
            Vector3D vec;
            _managerDirectOnly.GetNextWaypoint(ref _start, ref _endMedium, _contextNoSensors, out vec);
        }

        [Benchmark]
        public void GetNextWaypoint_DirectOnly_Long()
        {
            Vector3D vec;
            _managerDirectOnly.GetNextWaypoint(ref _start, ref _endLong, _contextNoSensors, out vec);
        }

        [Benchmark]
        public void GetNextWaypoint_WithAStar_NoSensors_Medium()
        {
            Vector3D vec;
            _managerWithAStar.GetNextWaypoint(ref _start, ref _endMedium, _contextNoSensors, out vec);
        }

        [Benchmark]
        public void GetNextWaypoint_WithAStar_WithSensors_Medium()
        {
            Vector3D vec;
            _managerWithAStar.GetNextWaypoint(ref _start, ref _endMedium, _contextWithSensors, out vec);
        }

        [Benchmark]
        public void GetNextWaypoint_WithAStar_WithSensors_Long()
        {
            Vector3D vec;
            _managerWithAStar.GetNextWaypoint(ref _start, ref _endLong, _contextWithSensors, out vec);
        }

        [Benchmark]
        public void GetNextWaypoint_WithGravity_Medium()
        {
            Vector3D vec;
            _managerWithAStar.GetNextWaypoint(ref _start, ref _endMedium, _contextWithGravity, out vec);
        }

        [Benchmark]
        public void GenerateCompletePath_DirectOnly_Medium()
        {
            var pathOutput = new List<Vector3D>();
            _managerDirectOnly.GenerateCompletePath(ref _start, ref _endMedium, _contextNoSensors, pathOutput);
        }

        [Benchmark]
        public void GenerateCompletePath_WithAStar_NoSensors_Medium()
        {
            var pathOutput = new List<Vector3D>();
            _managerWithAStar.GenerateCompletePath(ref _start, ref _endMedium, _contextNoSensors, pathOutput);
        }

        [Benchmark]
        public void GenerateCompletePath_WithAStar_WithSensors_Medium()
        {
            var pathOutput = new List<Vector3D>();
            _managerWithAStar.GenerateCompletePath(ref _start, ref _endMedium, _contextWithSensors, pathOutput);
        }

        #region Helper Methods

        private PathfindingContext CreateBasicContext(IPathfindingConfig config)
        {
            var controller = SEMockFactory.CreateMockController(new Vector3D(0, 0, 0));
            var thrusters = CreateBasicThrusters();

            return new PathfindingContext(
                config,
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

        private PathfindingContext CreateContextWithSensors(IPathfindingConfig config)
        {
            var controller = SEMockFactory.CreateMockController(new Vector3D(0, 0, 0));
            var thrusters = CreateBasicThrusters();

            var context = new PathfindingContext(
                config,
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

            context.SensorInfos = new List<PathfindingContext.SensorInfo>
            {
                new PathfindingContext.SensorInfo
                {
                    Sensor = SEMockFactory.CreateMockSensor(new Vector3D(0, 0, 0), 50f),
                    Position = new Vector3D(0, 0, 0),
                    MaxRange = 50f
                }
            };

            return context;
        }

        private PathfindingContext CreateContextWithGravity(IPathfindingConfig config, Vector3 gravity)
        {
            var controller = SEMockFactory.CreateMockController(new Vector3D(0, 0, 0), gravity);
            var thrusters = CreateBasicThrusters();

            return new PathfindingContext(
                config,
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

    /// <summary>
    /// Benchmarks comparing pathfinder selection overhead and fallback scenarios.
    /// </summary>
    [MemoryDiagnoser]
    [SimpleJob(warmupCount: 3, iterationCount: 10)]
    public class PathfindingManagerSelectionBenchmarks
    {
        private PathfindingManager _manager;
        private PathfindingContext _contextLowComplexity;
        private PathfindingContext _contextHighComplexity;

        private Vector3D _start;
        private Vector3D _end;

        [GlobalSetup]
        public void Setup()
        {
            var config = new FakePathfindingConfig
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
                maxPathNodes = 500, // Moderate limit to trigger fallbacks
                maxRepositionAttempts = 3
            };
            _manager = new PathfindingManager(config);

            var controller = SEMockFactory.CreateMockController(new Vector3D(0, 0, 0));
            var thrusters = CreateBasicThrusters();

            // Low complexity context - short distance, no obstacles
            _contextLowComplexity = new PathfindingContext(
                config,
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

            // High complexity context - sensors present
            _contextHighComplexity = new PathfindingContext(
                config,
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
            _contextHighComplexity.SensorInfos = new List<PathfindingContext.SensorInfo>
            {
                new PathfindingContext.SensorInfo
                {
                    Sensor = SEMockFactory.CreateMockSensor(new Vector3D(0, 0, 0), 100f),
                    Position = new Vector3D(0, 0, 0),
                    MaxRange = 100f
                }
            };

            _start = new Vector3D(0, 0, 0);
            _end = new Vector3D(1000, 0, 0);
        }

        [Benchmark(Baseline = true)]
        public void PathfinderSelection_LowComplexity_SelectsDirect()
        {
            Vector3D vec;
            _manager.GetNextWaypoint(ref _start, ref _end, _contextLowComplexity, out vec);
        }

        [Benchmark]
        public void PathfinderSelection_HighComplexity_SelectsAStar()
        {
            Vector3D vec;
            _manager.GetNextWaypoint(ref _start, ref _end, _contextHighComplexity, out vec);
        }

        [Benchmark]
        public void PathfinderSelection_VeryLongDistance()
        {
            var farEnd = new Vector3D(10000, 0, 0);
            Vector3D vec;
            _manager.GetNextWaypoint(ref _start, ref farEnd, _contextHighComplexity, out vec);
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
    }
}