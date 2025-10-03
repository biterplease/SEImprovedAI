using System;
using System.Diagnostics;
using ImprovedAI.Tests;
using ImprovedAI.Pathfinding;
using VRageMath;
using Sandbox.ModAPI;
using System.Collections.Generic;

namespace ImprovedAI.Tests
{
    /// <summary>
    /// Main test runner for pathfinding tests and benchmarks
    /// Usage: Run this in a test environment or console app
    /// </summary>
    public static class PathfindingTestRunner
    {
        public static void RunAll(string[] args)
        {
            Console.WriteLine("╔════════════════════════════════════════════════════════╗");
            Console.WriteLine("║     ImprovedAI Pathfinding Test & Benchmark Suite     ║");
            Console.WriteLine("╚════════════════════════════════════════════════════════╝");
            Console.WriteLine();

            if (args.Length == 0)
            {
                ShowUsage();
                return;
            }

            var command = args[0].ToLower();

            switch (command)
            {
                case "all":
                    RunAllTests();
                    RunAllBenchmarks();
                    break;

                case "tests":
                    RunAllTests();
                    break;

                case "benchmarks":
                    RunAllBenchmarks();
                    break;

                case "context":
                    PathfindingContextTests.RunAllTests();
                    break;

                case "astar":
                    AStarPathfinderTests.RunAllTests();
                    break;

                case "direct":
                    DirectPathfinderTests.RunAllTests();
                    break;

                case "bench-astar":
                    AStarPathfinderTests.RunBenchmarks();
                    break;

                case "bench-direct":
                    DirectPathfinderTests.RunBenchmarks();
                    break;

                case "quick":
                    RunQuickTest();
                    break;

                default:
                    Console.WriteLine($"Unknown command: {command}");
                    ShowUsage();
                    break;
            }

            Console.WriteLine("\n" + new string('═', 58));
            Console.WriteLine("Tests complete. Press any key to exit...");
            Console.ReadKey();
        }

        private static void ShowUsage()
        {
            Console.WriteLine("Usage: PathfindingTestRunner <command>");
            Console.WriteLine();
            Console.WriteLine("Commands:");
            Console.WriteLine("  all            - Run all tests and benchmarks");
            Console.WriteLine("  tests          - Run all unit tests");
            Console.WriteLine("  benchmarks     - Run all benchmarks");
            Console.WriteLine("  context        - Run PathfindingContext tests only");
            Console.WriteLine("  astar          - Run A* pathfinder tests only");
            Console.WriteLine("  direct         - Run Direct pathfinder tests only");
            Console.WriteLine("  bench-astar    - Run A* benchmarks only");
            Console.WriteLine("  bench-direct   - Run Direct pathfinder benchmarks only");
            Console.WriteLine("  quick          - Run quick smoke test");
            Console.WriteLine();
            Console.WriteLine("Examples:");
            Console.WriteLine("  PathfindingTestRunner all");
            Console.WriteLine("  PathfindingTestRunner tests");
            Console.WriteLine("  PathfindingTestRunner bench-astar");
        }

        private static void RunAllTests()
        {
            Console.WriteLine("╔════════════════════════════════════════════════════════╗");
            Console.WriteLine("║                  RUNNING ALL TESTS                     ║");
            Console.WriteLine("╚════════════════════════════════════════════════════════╝");
            Console.WriteLine();

            var sw = Stopwatch.StartNew();

            PathfindingContextTests.RunAllTests();
            DirectPathfinderTests.RunAllTests();
            AStarPathfinderTests.RunAllTests();

            sw.Stop();

            Console.WriteLine("\n" + new string('═', 58));
            Console.WriteLine($"All tests completed in {sw.ElapsedMilliseconds}ms");
        }

        private static void RunAllBenchmarks()
        {
            Console.WriteLine("\n╔════════════════════════════════════════════════════════╗");
            Console.WriteLine("║                RUNNING ALL BENCHMARKS                  ║");
            Console.WriteLine("╚════════════════════════════════════════════════════════╝");
            Console.WriteLine();

            var sw = Stopwatch.StartNew();

            DirectPathfinderTests.RunBenchmarks();
            AStarPathfinderTests.RunBenchmarks();

            sw.Stop();

            Console.WriteLine("\n" + new string('═', 58));
            Console.WriteLine($"All benchmarks completed in {sw.ElapsedMilliseconds}ms");
        }

        private static void RunQuickTest()
        {
            Console.WriteLine("╔════════════════════════════════════════════════════════╗");
            Console.WriteLine("║                   QUICK SMOKE TEST                     ║");
            Console.WriteLine("╚════════════════════════════════════════════════════════╝");
            Console.WriteLine();
            Console.WriteLine("Running quick validation tests...\n");

            try
            {
                // Quick context test
                Console.Write("✓ PathfindingContext creation... ");
                var controller = MockFactory.CreateMockController(Vector3D.Zero);
                var context = new PathfindingContext(
                    controller,
                    new List<IMySensorBlock>(),
                    new List<IMyCameraBlock>(),
                    new List<IMyThrust>(),
                    1000f, 5000f, 50f,
                    Base6Directions.Direction.Forward
                );
                Console.WriteLine("OK");

                // Quick Direct pathfinder test
                Console.Write("✓ DirectPathfinder basic path... ");
                var config = new FakeConfig
                {
                    allowRepathing = true,
                    minWaypointDistance = 25.0f,
                    maxWaypointDistance = 100.0f
                };
                var directPathfinder = new DirectPathfinder(config);
                var waypoint = directPathfinder.GetNextWaypoint(
                    new Vector3D(0, 0, 0),
                    new Vector3D(100, 0, 0),
                    context
                );
                if (!waypoint.HasValue)
                    throw new Exception("No waypoint generated");
                Console.WriteLine("OK");

                // Quick A* test
                Console.Write("✓ AStarPathfinder basic path... ");
                config.allowAStar = true;
                var astarPathfinder = new AStarPathfinder(config);
                var path = astarPathfinder.CalculatePath(
                    new Vector3D(0, 0, 0),
                    new Vector3D(200, 0, 0),
                    context
                );
                if (path == null || path.Count == 0)
                    throw new Exception("No path generated");
                Console.WriteLine("OK");

                Console.WriteLine("\n✅ All quick tests passed!");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"\n❌ Quick test failed: {ex.Message}");
                Console.WriteLine($"   {ex.StackTrace}");
            }
        }
    }
}