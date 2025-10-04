using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Running;
using ImprovedAI.Config;
using ImprovedAI.Pathfinding;
using ImprovedAI.TestUtil;
using Sandbox.ModAPI;
using System.Collections.Generic;
using VRageMath;

namespace ImprovedAI.Tests.Benchmarks
{
    /// <summary>
    /// Benchmarks for PathfindingContext climb direction methods.
    /// Tests performance impact of using ref parameters and caching.
    /// Uses real PathfindingContext with mocked dependencies from SEMockFactory.
    /// </summary>
    [MemoryDiagnoser]
    [SimpleJob(warmupCount: 3, iterationCount: 5)]
    public class PathfindingContextBenchmarks
    {
        private PathfindingContext _context;
        private Vector3D _testDirection;
        private MatrixD _worldMatrixTransposed;

        [GlobalSetup]
        public void Setup()
        {
            // Create mock controller using SEMockFactory
            var mockController = SEMockFactory.CreateMockController(
                position: new Vector3D(1000, 2000, 3000),
                gravity: new Vector3(0, -9.81f, 0),
                isWorking: true,
                isFunctional: true
            );

            // Create mock thrusters with realistic thrust values
            var mockThrusters = new List<IMyThrust>
            {
                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Forward, 100000f, isWorking: true),
                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Backward, 80000f, isWorking: true),
                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Up, 60000f, isWorking: true),
                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Down, 60000f, isWorking: true),
                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Left, 40000f, isWorking: true),
                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Right, 40000f, isWorking: true)
            };

            // Create mock config
            var mockConfig = new FakePathfindingConfig();

            // Create real PathfindingContext matching the actual constructor signature
            _context = new PathfindingContext(
                pathfindingConfig: mockConfig,
                controller: mockController,
                sensors: null, // Not needed for these benchmarks
                cameras: null, // Not needed for these benchmarks
                thrusters: mockThrusters,
                shipMass: 50000f,
                maxLoad: 1.0f,
                waypointDistance: 100f,
                controllerForwardDirection: Base6Directions.Direction.Forward,
                pruningStructureDelegate: null,
                planetCenter: null,
                planetRadius: null
            );

            // Test direction (45 degree angle upward)
            _testDirection = new Vector3D(0.707, 0.707, 0);
            var worldMatrix = mockController.WorldMatrix;
            MatrixD.Transpose(ref worldMatrix, out _worldMatrixTransposed);
        }

        [Benchmark(Baseline = true)]
        public bool CanClimbInDirection_Original()
        {
            return _context.CanClimbInDirection(ref _testDirection, ref _worldMatrixTransposed);
        }


        [GlobalCleanup]
        public void Cleanup()
        {
            _context = null;
        }
    }

    /// <summary>
    /// Benchmarks testing multiple directions in sequence
    /// (simulates real pathfinding scenario where multiple directions are tested)
    /// </summary>
    [MemoryDiagnoser]
    [SimpleJob(warmupCount: 3, iterationCount: 5)]
    public class PathfindingContextMultiDirectionBenchmarks
    {
        private PathfindingContext _context;
        private Vector3D[] _testDirections;
        private MatrixD _worldMatrixTransposed;

        [Params(6, 26)] // 6 cardinal directions, or 26 directions (cardinal + diagonals)
        public int DirectionCount { get; set; }

        [GlobalSetup]
        public void Setup()
        {
            // Create mock controller
            var mockController = SEMockFactory.CreateMockController(
                position: new Vector3D(1000, 2000, 3000),
                gravity: new Vector3(0, -9.81f, 0),
                isWorking: true,
                isFunctional: true
            );

            // Create mock thrusters
            var mockThrusters = new List<IMyThrust>
            {
                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Forward, 100000f, isWorking: true),
                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Backward, 80000f, isWorking: true),
                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Up, 60000f, isWorking: true),
                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Down, 60000f, isWorking: true),
                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Left, 40000f, isWorking: true),
                SEMockFactory.CreateMockThruster(Base6Directions.Direction.Right, 40000f, isWorking: true)
            };

            var mockConfig = new FakePathfindingConfig();

            _context = new PathfindingContext(
                pathfindingConfig: mockConfig,
                controller: mockController,
                sensors: null,
                cameras: null,
                thrusters: mockThrusters,
                shipMass: 50000f,
                maxLoad: 1.0f,
                waypointDistance: 100f,
                controllerForwardDirection: Base6Directions.Direction.Forward,
                pruningStructureDelegate: null,
                planetCenter: null,
                planetRadius: null
            );

            // Generate test directions
            _testDirections = GenerateTestDirections(DirectionCount);
        }

        [Benchmark(Baseline = true)]
        public int TestAllDirections_Original()
        {
            int validDirections = 0;
            for (int i = 0; i < _testDirections.Length; i++)
            {
                if (_context.CanClimbInDirection(ref _testDirections[i], ref _worldMatrixTransposed))
                    validDirections++;
            }
            return validDirections;
        }

        private Vector3D[] GenerateTestDirections(int count)
        {
            if (count == 6)
            {
                // 6 cardinal directions
                return new Vector3D[]
                {
                    new Vector3D(1, 0, 0),   // Right
                    new Vector3D(-1, 0, 0),  // Left
                    new Vector3D(0, 1, 0),   // Up
                    new Vector3D(0, -1, 0),  // Down
                    new Vector3D(0, 0, 1),   // Forward
                    new Vector3D(0, 0, -1)   // Backward
                };
            }
            else // 26 directions
            {
                var directions = new List<Vector3D>();

                // All combinations of -1, 0, 1 for x, y, z (except 0,0,0)
                for (int x = -1; x <= 1; x++)
                {
                    for (int y = -1; y <= 1; y++)
                    {
                        for (int z = -1; z <= 1; z++)
                        {
                            if (x == 0 && y == 0 && z == 0)
                                continue;

                            var direction = new Vector3D(x, y, z);
                            Vector3D.Normalize(ref direction, out direction);
                            directions.Add(direction);
                        }
                    }
                }

                return directions.ToArray();
            }
        }

        [GlobalCleanup]
        public void Cleanup()
        {
            _context = null;
            _testDirections = null;
        }
    }


    /// <summary>
    /// Entry point for running benchmarks
    /// </summary>
    class Program
    {
        static void Main(string[] args)
        {
            // Run all benchmark classes
            var switcher = new BenchmarkSwitcher(new[] {
                typeof(PathfindingContextBenchmarks),
                typeof(PathfindingContextMultiDirectionBenchmarks)
            });

            switcher.Run(args);
        }
    }
}