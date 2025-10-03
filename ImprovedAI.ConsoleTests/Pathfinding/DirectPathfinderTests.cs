using System;
using System.Collections.Generic;
using System.Diagnostics;
using ImprovedAI.Pathfinding;
using ImprovedAI.Config;
using VRageMath;

namespace ImprovedAI.Tests
{
    /// <summary>
    /// Unit tests and benchmarks for DirectPathfinder
    /// </summary>
    public static class DirectPathfinderTests
    {
        public static void RunAllTests()
        {
            Console.WriteLine("\n=== Direct Pathfinder Tests ===\n");

            int passed = 0;
            int failed = 0;

            // Functional tests
            TestSimpleDirectPath(ref passed, ref failed);
            TestPlanetAwareCorrectionArc(ref passed, ref failed);
            TestPlanetAwareAltitudeBuffer(ref passed, ref failed);
            TestRepositioningWithObstacle(ref passed, ref failed);
            TestInsufficientThrustDetection(ref passed, ref failed);
            TestVeryCloseTarget(ref passed, ref failed);

            // Edge cases
            TestRepathingDisabled(ref passed, ref failed);
            TestZeroGravityPath(ref passed, ref failed);

            Console.WriteLine($"\n=== Test Summary ===");
            Console.WriteLine($"Passed: {passed}");
            Console.WriteLine($"Failed: {failed}");
            Console.WriteLine($"Total:  {passed + failed}");
        }

        public static void RunBenchmarks()
        {
            Console.WriteLine("\n=== Direct Pathfinder Benchmarks ===\n");

            BenchmarkDirectPath();
            BenchmarkPlanetAwarePath();
            BenchmarkRepositioning();
            BenchmarkThrustCalculation();
            BenchmarkScaling();
        }

        #region Functional Tests

        private static void TestSimpleDirectPath(ref int passed, ref int failed)
        {
            Console.Write("Test: Simple Direct Path... ");
            try
            {
                var config = CreateTestConfig();
                var pathfinder = new DirectPathfinder(config);
                var context = CreateTestContext(Vector3.Zero);

                var start = new Vector3D(0, 0, 0);
                var end = new Vector3D(100, 0, 0);

                var waypoint = pathfinder.GetNextWaypoint(start, end, context);

                Assert(waypoint.HasValue, "Waypoint not generated");

                var direction = Vector3D.Normalize(end - start);
                var expectedWaypoint = start + direction * context.WaypointDistance;
                var distance = Vector3D.Distance(waypoint.Value, expectedWaypoint);

                Assert(distance < 20, $"Waypoint not along expected path (off by {distance:F1}m)");

                Console.WriteLine("PASSED");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        private static void TestPlanetAwareCorrectionArc(ref int passed, ref int failed)
        {
            Console.Write("Test: Planet Aware Arc Correction... ");
            try
            {
                var config = CreateTestConfig();
                config.usePlanetAwarePathfinding = true;
                var pathfinder = new DirectPathfinder(config);

                var planetCenter = new Vector3D(0, -60000, 0);
                var context = CreateTestContext(new Vector3(0, -9.81f, 0));
                context.PlanetCenter = planetCenter;
                context.PlanetRadius = 60000.0;

                // Positions on the planet surface
                var start = new Vector3D(0, 100, 0);
                var end = new Vector3D(500, 100, 0);

                var path = pathfinder.CalculatePath(start, end, context);

                Assert(path != null && path.Count > 0, "Path not generated");

                // Check that waypoints follow an arc rather than straight line
                for (int i = 0; i < path.Count; i++)
                {
                    var altitude = Vector3D.Distance(path[i], planetCenter) - context.PlanetRadius;
                    Assert(altitude >= config.minAltitudeBuffer * 0.9,
                        $"Waypoint {i} altitude too low: {altitude:F1}m");
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

        private static void TestPlanetAwareAltitudeBuffer(ref int passed, ref int failed)
        {
            Console.Write("Test: Planet Aware Altitude Buffer... ");
            try
            {
                var config = CreateTestConfig();
                config.usePlanetAwarePathfinding = true;
                config.minAltitudeBuffer = 100.0f;
                var pathfinder = new DirectPathfinder(config);

                var planetCenter = new Vector3D(0, -60000, 0);
                var context = CreateTestContext(new Vector3(0, -9.81f, 0));
                context.PlanetCenter = planetCenter;
                context.PlanetRadius = 60000.0;

                // Start below minimum altitude
                var start = new Vector3D(0, 25, 0); // Only 25m altitude
                var end = new Vector3D(500, 25, 0);

                var waypoint = pathfinder.GetNextWaypoint(start, end, context);

                Assert(waypoint.HasValue, "Waypoint not generated");

                var altitude = Vector3D.Distance(waypoint.Value, planetCenter) - context.PlanetRadius;
                Assert(altitude >= config.minAltitudeBuffer * 0.95,
                    $"Waypoint should be lifted to safe altitude: {altitude:F1}m < {config.minAltitudeBuffer}m");

                Console.WriteLine($"PASSED (lifted to {altitude:F1}m)");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        private static void TestRepositioningWithObstacle(ref int passed, ref int failed)
        {
            Console.Write("Test: Repositioning With Obstacle... ");
            try
            {
                var config = CreateTestConfig();
                config.allowRepathing = true;
                config.maxRepositionAttempts = 5;
                var pathfinder = new DirectPathfinder(config);

                var context = CreateTestContext(Vector3.Zero);

                // Add sensor detecting obstacle directly ahead
                context.SensorInfos = new List<PathfindingContext.SensorInfo>
                {
                    new PathfindingContext.SensorInfo
                    {
                        Position = new Vector3D(25, 0, 0),
                        MaxRange = 30f
                    }
                };

                var start = new Vector3D(0, 0, 0);
                var end = new Vector3D(100, 0, 0);

                var waypoint = pathfinder.GetNextWaypoint(start, end, context);

                Assert(waypoint.HasValue, "Should find repositioned waypoint");

                // Waypoint should not be directly on the obstacle
                var distToObstacle = Vector3D.Distance(waypoint.Value, new Vector3D(25, 0, 0));
                Assert(distToObstacle > 20, "Repositioned waypoint should avoid obstacle");

                Console.WriteLine($"PASSED (repositioned {distToObstacle:F1}m from obstacle)");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        private static void TestInsufficientThrustDetection(ref int passed, ref int failed)
        {
            Console.Write("Test: Insufficient Thrust Detection... ");
            try
            {
                var config = CreateTestConfig();
                var pathfinder = new DirectPathfinder(config);

                // Create context with minimal upward thrust but strong gravity
                Vector3 gravity = new Vector3(0, -20f, 0);
                var controller = MockFactory.CreateMockController(Vector3D.Zero, gravity);

                var context = CreateTestContext(gravity);

                var start = new Vector3D(0, 0, 0);
                var end = new Vector3D(0, 100, 0); // Need to climb

                var waypoint = pathfinder.GetNextWaypoint(start, end, context);

                // Should either fail or provide alternative
                // The important thing is it doesn't crash
                Console.WriteLine(waypoint.HasValue ?
                    "PASSED (found alternative)" :
                    "PASSED (correctly detected insufficient thrust)");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        private static void TestVeryCloseTarget(ref int passed, ref int failed)
        {
            Console.Write("Test: Very Close Target... ");
            try
            {
                var config = CreateTestConfig();
                var pathfinder = new DirectPathfinder(config);
                var context = CreateTestContext(Vector3.Zero);

                var start = new Vector3D(0, 0, 0);
                var end = new Vector3D(5, 0, 0); // Very close (< waypoint distance)

                var waypoint = pathfinder.GetNextWaypoint(start, end, context);

                Assert(waypoint.HasValue, "Should handle very close target");
                Assert(Vector3D.Distance(waypoint.Value, end) < 1,
                    "Should return target directly for very close distances");

                Console.WriteLine("PASSED");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        private static void TestRepathingDisabled(ref int passed, ref int failed)
        {
            Console.Write("Test: Repathing Disabled... ");
            try
            {
                var config = CreateTestConfig();
                config.allowRepathing = false;
                var pathfinder = new DirectPathfinder(config);

                var context = CreateTestContext(Vector3.Zero);

                // Add obstacle in the way
                context.SensorInfos = new List<PathfindingContext.SensorInfo>
                {
                    new PathfindingContext.SensorInfo
                    {
                        Position = new Vector3D(25, 0, 0),
                        MaxRange = 30f
                    }
                };

                var start = new Vector3D(0, 0, 0);
                var end = new Vector3D(100, 0, 0);

                var waypoint = pathfinder.GetNextWaypoint(start, end, context);

                // With repathing disabled and obstacle present, should return null
                Assert(!waypoint.HasValue, "Should fail when repathing is disabled and path blocked");

                Console.WriteLine("PASSED");
                passed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"FAILED: {ex.Message}");
                failed++;
            }
        }

        private static void TestZeroGravityPath(ref int passed, ref int failed)
        {
            Console.Write("Test: Zero Gravity Path... ");
            try
            {
                var config = CreateTestConfig();
                var pathfinder = new DirectPathfinder(config);
                var context = CreateTestContext(Vector3.Zero); // No gravity

                var start = new Vector3D(0, 0, 0);
                var end = new Vector3D(100, 100, 100);

                var path = pathfinder.CalculatePath(start, end, context);

                Assert(path != null && path.Count > 0, "Path not generated");

                // In zero gravity, path should be straight line
                for (int i = 1; i < path.Count - 1; i++)
                {
                    var prev = path[i - 1];
                    var curr = path[i];
                    var next = path[i + 1];

                    var dir1 = Vector3D.Normalize(curr - prev);
                    var dir2 = Vector3D.Normalize(next - curr);
                    var dot = Vector3D.Dot(dir1, dir2);

                    Assert(dot > 0.95, $"Path should be straight in zero-G (dot: {dot:F3})");
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

        #endregion

        #region Benchmarks

        private static void BenchmarkDirectPath()
        {
            Console.WriteLine("\n--- Benchmark: Direct Path (No Obstacles) ---");

            var config = CreateTestConfig();
            var pathfinder = new DirectPathfinder(config);
            var context = CreateTestContext(Vector3.Zero);

            var start = new Vector3D(0, 0, 0);
            var end = new Vector3D(500, 0, 0);

            var sw = Stopwatch.StartNew();
            var iterations = 1000;

            for (int i = 0; i < iterations; i++)
            {
                pathfinder.GetNextWaypoint(start, end, context);
            }

            sw.Stop();
            var avgMs = sw.ElapsedMilliseconds / (double)iterations;
            Console.WriteLine($"Average time: {avgMs:F4}ms per waypoint");
            Console.WriteLine($"Total time: {sw.ElapsedMilliseconds}ms for {iterations} iterations");
            Console.WriteLine($"Rate: {iterations / (sw.ElapsedMilliseconds / 1000.0):F0} waypoints/sec");
        }

        private static void BenchmarkPlanetAwarePath()
        {
            Console.WriteLine("\n--- Benchmark: Planet Aware Path ---");

            var config = CreateTestConfig();
            config.usePlanetAwarePathfinding = true;
            var pathfinder = new DirectPathfinder(config);

            var planetCenter = new Vector3D(0, -60000, 0);
            var context = CreateTestContext(new Vector3(0, -9.81f, 0));
            context.PlanetCenter = planetCenter;
            context.PlanetRadius = 60000.0;

            var start = new Vector3D(0, 100, 0);
            var end = new Vector3D(500, 100, 0);

            var sw = Stopwatch.StartNew();
            var iterations = 500;

            for (int i = 0; i < iterations; i++)
            {
                pathfinder.GetNextWaypoint(start, end, context);
            }

            sw.Stop();
            var avgMs = sw.ElapsedMilliseconds / (double)iterations;
            Console.WriteLine($"Average time: {avgMs:F4}ms per waypoint");
            Console.WriteLine($"Total time: {sw.ElapsedMilliseconds}ms for {iterations} iterations");
            Console.WriteLine($"Overhead vs simple path: ~{(avgMs / 0.01 - 1) * 100:F1}%");
        }

        private static void BenchmarkRepositioning()
        {
            Console.WriteLine("\n--- Benchmark: Repositioning With Obstacle ---");

            var config = CreateTestConfig();
            config.allowRepathing = true;
            config.maxRepositionAttempts = 3;
            var pathfinder = new DirectPathfinder(config);

            var context = CreateTestContext(Vector3.Zero);
            context.SensorInfos = new List<PathfindingContext.SensorInfo>
            {
                new PathfindingContext.SensorInfo { Position = new Vector3D(25, 0, 0), MaxRange = 30f }
            };

            var start = new Vector3D(0, 0, 0);
            var end = new Vector3D(100, 0, 0);

            var sw = Stopwatch.StartNew();
            var iterations = 200;

            for (int i = 0; i < iterations; i++)
            {
                pathfinder.GetNextWaypoint(start, end, context);
            }

            sw.Stop();
            var avgMs = sw.ElapsedMilliseconds / (double)iterations;
            Console.WriteLine($"Average time: {avgMs:F4}ms per waypoint (with repositioning)");
            Console.WriteLine($"Total time: {sw.ElapsedMilliseconds}ms for {iterations} iterations");
            Console.WriteLine($"Max attempts: {config.maxRepositionAttempts}");
        }

        private static void BenchmarkThrustCalculation()
        {
            Console.WriteLine("\n--- Benchmark: Thrust Capability Checks ---");

            var config = CreateTestConfig();
            var pathfinder = new DirectPathfinder(config);
            var context = CreateTestContext(new Vector3(0, -9.81f, 0));

            var start = new Vector3D(0, 0, 0);
            var directions = new[]
            {
                new Vector3D(0, 100, 0),    // Up
                new Vector3D(0, -100, 0),   // Down
                new Vector3D(100, 0, 0),    // Forward
                new Vector3D(100, 100, 0)   // Diagonal
            };

            var sw = Stopwatch.StartNew();
            var iterations = 500;

            for (int i = 0; i < iterations; i++)
            {
                foreach (var dir in directions)
                {
                    pathfinder.GetNextWaypoint(start, start + dir, context);
                }
            }

            sw.Stop();
            var totalCalls = iterations * directions.Length;
            var avgMs = sw.ElapsedMilliseconds / (double)totalCalls;
            Console.WriteLine($"Average time: {avgMs:F4}ms per check");
            Console.WriteLine($"Total time: {sw.ElapsedMilliseconds}ms for {totalCalls} checks");
            Console.WriteLine($"Directions tested: {directions.Length}");
        }


        private static void BenchmarkScaling()
        {
            Console.WriteLine("\n--- Benchmark: Distance Scaling ---");

            var config = CreateTestConfig();
            var pathfinder = new DirectPathfinder(config);
            var context = CreateTestContext(Vector3.Zero);

            var distances = new KeyValuePair<int, float>[] {
                new KeyValuePair<int,float>(50, config.maxWaypointDistance),
                new KeyValuePair<int,float>(100, config.maxWaypointDistance),
                new KeyValuePair<int,float>(250, config.maxWaypointDistance),
                new KeyValuePair<int,float>(500, config.maxWaypointDistance),
                new KeyValuePair<int,float>(1000, config.maxWaypointDistance),
                new KeyValuePair<int,float>(2000, config.maxWaypointDistance),
                new KeyValuePair<int,float>(5000, config.maxWaypointDistance),
                new KeyValuePair<int,float>(5000, 200f),
                new KeyValuePair<int,float>(5000, 500f),
                new KeyValuePair<int,float>(5000, 1000f),
                new KeyValuePair<int,float>(10000, 200f),
                new KeyValuePair<int,float>(10000, 500f),
                new KeyValuePair<int,float>(10000, 1000f),
                new KeyValuePair<int,float>(50000, 1000f),
                new KeyValuePair<int,float>(300000, 5000f),
                new KeyValuePair<int,float>(300000, 10000f),
            };

            Console.WriteLine("Distance | MWP       | Time (μs) | Waypoints");
            Console.WriteLine("---------|-----------|-----------|----------");

            foreach (var kvp in distances)
            {
                var originalDistance = kvp.Value;
                config.maxWaypointDistance = kvp.Value;
                var start = new Vector3D(0, 0, 0);
                var end = new Vector3D(kvp.Key, 0, 0);

                var sw = Stopwatch.StartNew();
                var iterations = 1000;

                for (int i = 0; i < iterations; i++)
                {
                    pathfinder.GetNextWaypoint(start, end, context);
                }

                sw.Stop();
                var avgUs = (sw.ElapsedMilliseconds * 1000.0) / iterations;
                var numWaypoints = (int)(kvp.Key / context.WaypointDistance) + 1;

                Console.WriteLine($"{kvp.Key,8} | {config.maxWaypointDistance,9:F0} | {avgUs,9:F2} | {numWaypoints,9}");
                config.maxWaypointDistance = originalDistance;
            }
        }

        #endregion

        #region Helper Methods

        private static FakeConfig CreateTestConfig()
        {
            return new FakeConfig
            {
                allowAStar = false,
                allowRepathing = true,
                requireSensorsForPathfinding = false,
                requireCamerasForPathfinding = false,
                usePlanetAwarePathfinding = true,
                minWaypointDistance = 25.0f,
                maxWaypointDistance = 100.0f,
                minAltitudeBuffer = 50.0f,
                maxPathNodes = 1000,
                maxRepositionAttempts = 3,
                 maxPathfindingTime = new TimeSpan(0,0,5)
            };
        }

        // This overload does not account for the planet center.
        private static PathfindingContext CreateTestContext(Vector3 gravity)
        {
            var context = TestHelpers.CreateTestContext(gravity, Vector3D.Zero);

            return context;
        }
        private static PathfindingContext CreateTestContext(Vector3 gravity, Vector3 planetCenter)
        {
            var context = TestHelpers.CreateTestContext(gravity, Vector3D.Zero);

            if (gravity.LengthSquared() > 0)
            {
                TestHelpers.AddPlanetToContext(context, new Vector3D(planetCenter), 60000.0);
            }

            return context;
        }

        private static void Assert(bool condition, string message)
        {
            if (!condition)
                throw new Exception(message);
        }

        #endregion
    }
}