using System;
using System.Collections.Generic;
using System.Diagnostics;
using ImprovedAI.Pathfinding;
using ImprovedAI.Config;
using VRageMath;
using Sandbox.ModAPI;
using VRage.Game.ModAPI;

namespace ImprovedAI.Tests
{
    /// <summary>
    /// Unit tests and benchmarks for AStarPathfinder
    /// </summary>
    public static class AStarPathfinderTests
    {
        public static void RunAllTests()
        {
            Console.WriteLine("\n=== A* Pathfinder Tests ===\n");

            int passed = 0;
            int failed = 0;

            // Functional tests
            TestSimpleStraightPath(ref passed, ref failed);
            TestPathWithObstacle(ref passed, ref failed);
            TestComplexityEstimation(ref passed, ref failed);
            TestFallbackToDirectPathfinder(ref passed, ref failed);
            TestMaxNodeLimit(ref passed, ref failed);
            TestPlanetAwarePathfinding(ref passed, ref failed);

            // Edge cases
            TestVeryShortDistance(ref passed, ref failed);
            TestImpossiblePath(ref passed, ref failed);

            Console.WriteLine($"\n=== Test Summary ===");
            Console.WriteLine($"Passed: {passed}");
            Console.WriteLine($"Failed: {failed}");
            Console.WriteLine($"Total:  {passed + failed}");
        }

        public static void RunBenchmarks()
        {
            Console.WriteLine("\n=== A* Pathfinder Benchmarks ===\n");

            BenchmarkShortPath();
            BenchmarkMediumPath();
            BenchmarkLongPath();
            BenchmarkWithObstacles();
            BenchmarkComplexityScaling();
        }

        #region Functional Tests

        private static void TestSimpleStraightPath(ref int passed, ref int failed)
        {
            Console.Write("Test: Simple Straight Path... ");
            try
            {
                var config = CreateTestConfig();
                var pathfinder = new AStarPathfinder(config);
                var context = CreateTestContext(Vector3D.Zero, Vector3.Zero);

                var start = new Vector3D(0, 0, 0);
                var end = new Vector3D(100, 0, 0);

                var path = pathfinder.CalculatePath(start, end, context);

                Assert(path != null && path.Count > 0, "Path not generated");
                Assert(Vector3D.Distance(path[0], start) < 10, "Path doesn't start near start point");
                Assert(Vector3D.Distance(path[path.Count - 1], end) < 10, "Path doesn't end near end point");

                Console.WriteLine($"PASSED (waypoints: {path.Count})");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        private static void TestPathWithObstacle(ref int passed, ref int failed)
        {
            Console.Write("Test: Path With Obstacle... ");
            try
            {
                var config = CreateTestConfig();
                var pathfinder = new AStarPathfinder(config);

                // Create context with obstacle in the middle
                var context = CreateTestContext(Vector3D.Zero, Vector3.Zero);
                context.SensorInfos = new List<PathfindingContext.SensorInfo>
                {
                    new PathfindingContext.SensorInfo
                    {
                        Position = new Vector3D(50, 0, 0),
                        MaxRange = 20f
                    }
                };

                var start = new Vector3D(0, 0, 0);
                var end = new Vector3D(100, 0, 0);

                var path = pathfinder.CalculatePath(start, end, context);

                Assert(path != null && path.Count > 0, "Path not generated");

                // Path should route around the obstacle
                bool pathAvoidsObstacle = true;
                foreach (var waypoint in path)
                {
                    var distToObstacle = Vector3D.Distance(waypoint, new Vector3D(50, 0, 0));
                    if (distToObstacle < 10) // Too close to obstacle
                    {
                        pathAvoidsObstacle = false;
                        break;
                    }
                }

                Assert(pathAvoidsObstacle, "Path should avoid obstacle");

                Console.WriteLine($"PASSED (waypoints: {path.Count})");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        private static void TestComplexityEstimation(ref int passed, ref int failed)
        {
            Console.Write("Test: Complexity Estimation... ");
            try
            {
                var config = CreateTestConfig();
                var pathfinder = new AStarPathfinder(config);

                var start = new Vector3D(0, 0, 0);

                var shortEnd = new Vector3D(50, 0, 0);
                var mediumEnd = new Vector3D(300, 0, 0);
                var longEnd = new Vector3D(1500, 0, 0);

                var shortComplexity = pathfinder.EstimatedComplexity(start, shortEnd);
                var mediumComplexity = pathfinder.EstimatedComplexity(start, mediumEnd);
                var longComplexity = pathfinder.EstimatedComplexity(start, longEnd);

                Assert(shortComplexity < mediumComplexity, "Short path should be less complex than medium");
                Assert(mediumComplexity < longComplexity, "Medium path should be less complex than long");
                Assert(longComplexity <= config.maxPathNodes, "Complexity should respect max nodes");

                Console.WriteLine($"PASSED (short:{shortComplexity}, med:{mediumComplexity}, long:{longComplexity})");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        private static void TestFallbackToDirectPathfinder(ref int passed, ref int failed)
        {
            Console.Write("Test: Fallback to Direct Pathfinder... ");
            try
            {
                var config = CreateTestConfig();
                config.allowRepathing = true;
                var pathfinder = new AStarPathfinder(config);
                var context = CreateTestContext(Vector3D.Zero, Vector3.Zero);

                var start = new Vector3D(0, 0, 0);
                var end = new Vector3D(50, 0, 0); // Very short distance

                var waypoint = pathfinder.GetNextWaypoint(start, end, context);

                Assert(waypoint.HasValue, "Should get waypoint from fallback");

                Console.WriteLine("PASSED");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        private static void TestMaxNodeLimit(ref int passed, ref int failed)
        {
            Console.Write("Test: Max Node Limit... ");
            try
            {
                var config = CreateTestConfig();
                config.maxPathNodes = 20; // Very low limit
                var pathfinder = new AStarPathfinder(config);
                var context = CreateTestContext(Vector3D.Zero, Vector3.Zero);

                var start = new Vector3D(0, 0, 0);
                var end = new Vector3D(10000, 0, 0); // Very long distance

                var path = pathfinder.CalculatePath(start, end, context);

                // Should either succeed with limited nodes or fallback gracefully
                Assert(path != null, "Should handle node limit gracefully");
                if (path.Count > 0)
                {
                    Assert(path.Count <= config.MaxPathNodes() + 10,
                        $"Path should respect max nodes (got {path.Count})");
                }

                Console.WriteLine($"PASSED (nodes: {path.Count})");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        private static void TestPlanetAwarePathfinding(ref int passed, ref int failed)
        {
            Console.Write("Test: Planet Aware Pathfinding... ");
            try
            {
                var config = CreateTestConfig();
                config.usePlanetAwarePathfinding = true;
                config.minAltitudeBuffer = 50.0f;

                var pathfinder = new AStarPathfinder(config);

                var planetCenter = new Vector3D(0, -60000, 0);
                var context = CreateTestContext(new Vector3(0, -9.81f, 0), Vector3.Zero);
                context.PlanetCenter = planetCenter;
                context.PlanetRadius = 60000.0;

                var start = new Vector3D(0, 100, 0);
                var end = new Vector3D(500, 100, 0);

                var path = pathfinder.CalculatePath(start, end, context);

                Assert(path != null && path.Count > 0, "Path not generated");

                // All waypoints should maintain safe altitude
                foreach (var waypoint in path)
                {
                    var altitude = Vector3D.Distance(waypoint, planetCenter) - context.PlanetRadius;
                    Assert(altitude >= config.minAltitudeBuffer * 0.9,
                        $"Waypoint too low: {altitude:F1}m < {config.minAltitudeBuffer}m");
                }

                Console.WriteLine($"PASSED (waypoints: {path.Count})");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        private static void TestVeryShortDistance(ref int passed, ref int failed)
        {
            Console.Write("Test: Very Short Distance... ");
            try
            {
                var config = CreateTestConfig();
                var pathfinder = new AStarPathfinder(config);
                var context = CreateTestContext(Vector3D.Zero, Vector3.Zero);

                var start = new Vector3D(0, 0, 0);
                var end = new Vector3D(5, 0, 0); // Very short

                var waypoint = pathfinder.GetNextWaypoint(start, end, context);

                Assert(waypoint.HasValue, "Should handle very short distances");

                Console.WriteLine("PASSED");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        private static void TestImpossiblePath(ref int passed, ref int failed)
        {
            Console.Write("Test: Impossible Path... ");
            try
            {
                var config = CreateTestConfig();
                config.allowRepathing = false; // Disable fallback
                config.maxPathNodes = 5; // Make it very hard to find path

                var pathfinder = new AStarPathfinder(config);
                var context = CreateTestContext(Vector3D.Zero, Vector3.Zero);

                var start = new Vector3D(0, 0, 0);
                var end = new Vector3D(10000, 10000, 10000); // Very far

                var path = pathfinder.CalculatePath(start, end, context);

                // Should fail gracefully and return empty or minimal path
                Assert(path != null, "Should not crash on impossible path");

                Console.WriteLine("PASSED (handled gracefully)");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        #endregion

        #region Benchmarks

        private static void BenchmarkShortPath()
        {
            Console.WriteLine("\n--- Benchmark: Short Path (100m) ---");

            var config = CreateTestConfig();
            var pathfinder = new AStarPathfinder(config);
            var context = CreateTestContext(Vector3D.Zero, Vector3.Zero);

            var start = new Vector3D(0, 0, 0);
            var end = new Vector3D(100, 0, 0);

            var sw = Stopwatch.StartNew();
            var iterations = 100;

            for (int i = 0; i < iterations; i++)
            {
                pathfinder.CalculatePath(start, end, context);
            }

            sw.Stop();
            var avgMs = sw.ElapsedMilliseconds / (double)iterations;
            Console.WriteLine($"Average time: {avgMs:F3}ms per path");
            Console.WriteLine($"Total time: {sw.ElapsedMilliseconds}ms for {iterations} iterations");
        }

        private static void BenchmarkMediumPath()
        {
            Console.WriteLine("\n--- Benchmark: Medium Path (500m) ---");

            var config = CreateTestConfig();
            var pathfinder = new AStarPathfinder(config);
            var context = CreateTestContext(Vector3D.Zero, Vector3.Zero);

            var start = new Vector3D(0, 0, 0);
            var end = new Vector3D(500, 0, 0);

            var sw = Stopwatch.StartNew();
            var iterations = 50;

            for (int i = 0; i < iterations; i++)
            {
                pathfinder.CalculatePath(start, end, context);
            }

            sw.Stop();
            var avgMs = sw.ElapsedMilliseconds / (double)iterations;
            Console.WriteLine($"Average time: {avgMs:F3}ms per path");
            Console.WriteLine($"Total time: {sw.ElapsedMilliseconds}ms for {iterations} iterations");
        }

        private static void BenchmarkLongPath()
        {
            Console.WriteLine("\n--- Benchmark: Long Path (2000m) ---");

            var config = CreateTestConfig();
            var pathfinder = new AStarPathfinder(config);
            var context = CreateTestContext(Vector3D.Zero, Vector3.Zero);

            var start = new Vector3D(0, 0, 0);
            var end = new Vector3D(2000, 0, 0);

            var sw = Stopwatch.StartNew();
            var iterations = 20;

            for (int i = 0; i < iterations; i++)
            {
                pathfinder.CalculatePath(start, end, context);
            }

            sw.Stop();
            var avgMs = sw.ElapsedMilliseconds / (double)iterations;
            Console.WriteLine($"Average time: {avgMs:F3}ms per path");
            Console.WriteLine($"Total time: {sw.ElapsedMilliseconds}ms for {iterations} iterations");
        }

        private static void BenchmarkWithObstacles()
        {
            Console.WriteLine("\n--- Benchmark: Path with Multiple Obstacles (500m) ---");

            var config = CreateTestConfig();
            var pathfinder = new AStarPathfinder(config);
            var context = CreateTestContext(Vector3D.Zero, Vector3.Zero);

            // Add multiple obstacles
            context.SensorInfos = new List<PathfindingContext.SensorInfo>
            {
                new PathfindingContext.SensorInfo { Position = new Vector3D(100, 0, 0), MaxRange = 30f },
                new PathfindingContext.SensorInfo { Position = new Vector3D(200, 50, 0), MaxRange = 30f },
                new PathfindingContext.SensorInfo { Position = new Vector3D(300, -50, 0), MaxRange = 30f },
                new PathfindingContext.SensorInfo { Position = new Vector3D(400, 0, 0), MaxRange = 30f }
            };

            var start = new Vector3D(0, 0, 0);
            var end = new Vector3D(500, 0, 0);

            var sw = Stopwatch.StartNew();
            var iterations = 30;

            for (int i = 0; i < iterations; i++)
            {
                pathfinder.CalculatePath(start, end, context);
            }

            sw.Stop();
            var avgMs = sw.ElapsedMilliseconds / (double)iterations;
            Console.WriteLine($"Average time: {avgMs:F3}ms per path");
            Console.WriteLine($"Total time: {sw.ElapsedMilliseconds}ms for {iterations} iterations");
            Console.WriteLine($"Obstacles: {context.SensorInfos.Count}");
        }

        private static void BenchmarkComplexityScaling()
        {
            Console.WriteLine("\n--- Benchmark: Complexity Scaling ---");

            var config = CreateTestConfig();
            var pathfinder = new AStarPathfinder(config);
            var context = CreateTestContext(Vector3D.Zero, Vector3.Zero);

            var distances = new[] { 50, 100, 200, 500, 1000, 2000 };

            Console.WriteLine("Distance | Time (ms) | Waypoints");
            Console.WriteLine("---------|-----------|----------");

            foreach (var distance in distances)
            {
                var start = new Vector3D(0, 0, 0);
                var end = new Vector3D(distance, 0, 0);

                var sw = Stopwatch.StartNew();
                var iterations = 10;
                int totalWaypoints = 0;

                for (int i = 0; i < iterations; i++)
                {
                    var path = pathfinder.CalculatePath(start, end, context);
                    totalWaypoints += path.Count;
                }

                sw.Stop();
                var avgMs = sw.ElapsedMilliseconds / (double)iterations;
                var avgWaypoints = totalWaypoints / iterations;

                Console.WriteLine($"{distance,8} | {avgMs,9:F3} | {avgWaypoints,9}");
            }
        }

        #endregion

        #region Helper Methods

        private static FakeConfig CreateTestConfig()
        {
            return new FakeConfig
            {
                allowAStar = true,
                allowRepathing = true,
                requireSensorsForPathfinding = false,
                requireCamerasForPathfinding = false,
                usePlanetAwarePathfinding = true,
                minWaypointDistance = 25.0f,
                maxWaypointDistance = 100.0f,
                minAltitudeBuffer = 50.0f,
                maxPathNodes = 1000,
                maxRepositionAttempts = 3,
                maxPathfindingTime = new TimeSpan(0, 0, 5)
            };
        }

        private static PathfindingContext CreateTestContext(Vector3 gravity, Vector3 planetCenter)
        {
            var controller = MockFactory.CreateMockController(Vector3D.Zero, gravity);

            //if (gravity.LengthSquared() > 0)
            //{
            //    controller.SetPlanet(new Vector3D(planetCenter), 60000.0);
            //}

            var thrusters = new List<IMyThrust>
            {
                MockFactory.CreateMockThruster(Base6Directions.Direction.Forward, 100000f),
                MockFactory.CreateMockThruster(Base6Directions.Direction.Backward, 100000f),
                MockFactory.CreateMockThruster(Base6Directions.Direction.Up, 100000f),
                MockFactory.CreateMockThruster(Base6Directions.Direction.Down, 100000f),
                MockFactory.CreateMockThruster(Base6Directions.Direction.Left, 100000f),
                MockFactory.CreateMockThruster(Base6Directions.Direction.Right, 100000f)
            };

            return new PathfindingContext(
                controller,
                new List<IMySensorBlock>(),
                new List<IMyCameraBlock>(),
                thrusters,
                1000f,
                5000f,
                50f,
                Base6Directions.Direction.Forward
            );
        }

        private static void Assert(bool condition, string message)
        {
            if (!condition)
                throw new Exception(message);
        }

        #endregion
    }
}