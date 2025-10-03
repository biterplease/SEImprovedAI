using System;
using System.Collections.Generic;
using System.Diagnostics;
using ImprovedAI.Pathfinding;
using ImprovedAI.Config;
using VRageMath;

namespace ImprovedAI.Tests
{
    /// <summary>
    /// Expanded benchmarks with aggressive iteration counts and config variations
    /// </summary>
    public static class ExpandedBenchmarks
    {
        public static void RunAll()
        {
            Console.WriteLine("\n╔════════════════════════════════════════════════════════╗");
            Console.WriteLine("║          EXPANDED PERFORMANCE BENCHMARKS               ║");
            Console.WriteLine("╚════════════════════════════════════════════════════════╝\n");

            // Direct Pathfinder Benchmarks
            BenchmarkDirectPathfinder_WaypointDistance();
            BenchmarkDirectPathfinder_RepositionAttempts();
            BenchmarkDirectPathfinder_AltitudeBuffer();
            BenchmarkDirectPathfinder_Distance();

            // A* Pathfinder Benchmarks
            BenchmarkAStar_MaxPathNodes();
            BenchmarkAStar_GridSpacing();
            BenchmarkAStar_ObstacleCount();
            BenchmarkAStar_Distance();

            // Combined scenario benchmarks
            BenchmarkRealisticScenarios();
        }

        #region Direct Pathfinder Benchmarks

        private static void BenchmarkDirectPathfinder_WaypointDistance()
        {
            Console.WriteLine("═══ DirectPathfinder: Waypoint Distance Scaling ═══");
            Console.WriteLine("Testing performance with varying waypoint distances");
            Console.WriteLine();

            var waypointDistances = new float[] { 25f, 50f, 100f, 200f, 500f, 1000f, 2000f, 5000f };
            var distance = 10000.0; // Fixed 10km distance
            var iterations = 1000;

            Console.WriteLine("WP Dist | Iterations | Avg (µs) | Total (ms) | Waypoints");
            Console.WriteLine("--------|------------|----------|------------|----------");

            foreach (var wpDist in waypointDistances)
            {
                var config = new FakeConfig
                {
                    allowRepathing = false,
                    requireSensorsForPathfinding = false,
                    requireCamerasForPathfinding = false,
                    usePlanetAwarePathfinding = false,
                    minWaypointDistance = wpDist * 0.5f,
                    maxWaypointDistance = wpDist
                };

                var pathfinder = new DirectPathfinder(config);
                var context = TestHelpers.CreateTestContext(Vector3.Zero, Vector3D.Zero);
                context.WaypointDistance = wpDist;

                var start = new Vector3D(0, 0, 0);
                var end = new Vector3D(distance, 0, 0);

                var sw = Stopwatch.StartNew();
                for (int i = 0; i < iterations; i++)
                {
                    pathfinder.GetNextWaypoint(start, end, context);
                }
                sw.Stop();

                var avgUs = (sw.ElapsedMilliseconds * 1000.0) / iterations;
                var numWaypoints = (int)(distance / wpDist) + 1;

                Console.WriteLine($"{wpDist,7:F0} | {iterations,10} | {avgUs,8:F2} | {sw.ElapsedMilliseconds,10} | {numWaypoints,9}");
            }
            Console.WriteLine();
        }

        private static void BenchmarkDirectPathfinder_RepositionAttempts()
        {
            Console.WriteLine("═══ DirectPathfinder: Reposition Attempts Impact ═══");
            Console.WriteLine("Testing performance with varying max reposition attempts");
            Console.WriteLine();

            var maxAttempts = new int[] { 1, 3, 5, 10, 20, 50 };
            var iterations = 1000;

            Console.WriteLine("Max Attempts | Iterations | Avg (µs) | Total (ms)");
            Console.WriteLine("-------------|------------|----------|----------");

            foreach (var attempts in maxAttempts)
            {
                var config = new FakeConfig
                {
                    allowRepathing = true,
                    requireSensorsForPathfinding = true, // Enable sensor checks
                    requireCamerasForPathfinding = false,
                    usePlanetAwarePathfinding = false,
                    maxRepositionAttempts = attempts,
                    minWaypointDistance = 25f,
                    maxWaypointDistance = 100f
                };

                var pathfinder = new DirectPathfinder(config);
                var context = TestHelpers.CreateTestContext(Vector3.Zero, Vector3D.Zero);

                // Add obstacle directly in path
                context.SensorInfos = new List<PathfindingContext.SensorInfo>
                {
                    new PathfindingContext.SensorInfo
                    {
                        Position = new Vector3D(50, 0, 0),
                        MaxRange = 30f
                    }
                };

                var start = new Vector3D(0, 0, 0);
                var end = new Vector3D(100, 0, 0);

                var sw = Stopwatch.StartNew();
                for (int i = 0; i < iterations; i++)
                {
                    pathfinder.GetNextWaypoint(start, end, context);
                }
                sw.Stop();

                var avgUs = (sw.ElapsedMilliseconds * 1000.0) / iterations;

                Console.WriteLine($"{attempts,12} | {iterations,10} | {avgUs,8:F2} | {sw.ElapsedMilliseconds,10}");
            }
            Console.WriteLine();
        }

        private static void BenchmarkDirectPathfinder_AltitudeBuffer()
        {
            Console.WriteLine("═══ DirectPathfinder: Altitude Buffer Impact ═══");
            Console.WriteLine("Testing planet-aware pathfinding with varying altitude buffers");
            Console.WriteLine();

            var altitudeBuffers = new float[] { 10f, 20f, 50f, 100f, 200f, 500f, 1000f };
            var iterations = 1000;

            Console.WriteLine("Alt Buffer | Iterations | Avg (µs) | Total (ms)");
            Console.WriteLine("-----------|------------|----------|----------");

            foreach (var buffer in altitudeBuffers)
            {
                var config = new FakeConfig
                {
                    allowRepathing = false,
                    requireSensorsForPathfinding = false,
                    requireCamerasForPathfinding = false,
                    usePlanetAwarePathfinding = true,
                    minAltitudeBuffer = buffer,
                    minWaypointDistance = 25f,
                    maxWaypointDistance = 100f
                };

                var pathfinder = new DirectPathfinder(config);
                var context = TestHelpers.CreateTestContext(new Vector3(0, -9.81f, 0), Vector3D.Zero);
                TestHelpers.AddPlanetToContext(context, new Vector3D(0, -60000, 0), 60000.0);

                var start = new Vector3D(0, 100, 0);
                var end = new Vector3D(500, 100, 0);

                var sw = Stopwatch.StartNew();
                for (int i = 0; i < iterations; i++)
                {
                    pathfinder.GetNextWaypoint(start, end, context);
                }
                sw.Stop();

                var avgUs = (sw.ElapsedMilliseconds * 1000.0) / iterations;

                Console.WriteLine($"{buffer,10:F0} | {iterations,10} | {avgUs,8:F2} | {sw.ElapsedMilliseconds,10}");
            }
            Console.WriteLine();
        }

        private static void BenchmarkDirectPathfinder_Distance()
        {
            Console.WriteLine("═══ DirectPathfinder: Distance Scaling (Detailed) ═══");
            Console.WriteLine("Testing performance across distance ranges");
            Console.WriteLine();

            var distances = new double[] {
                10, 25, 50, 100, 250, 500, 1000,
                2500, 5000, 10000, 25000, 50000,
                100000, 250000, 500000, 1000000
            };
            var iterations = 1000;

            Console.WriteLine("Distance | Iterations | Avg (µs) | Total (ms) | WP Count");
            Console.WriteLine("---------|------------|----------|------------|----------");

            var config = new FakeConfig
            {
                allowRepathing = false,
                requireSensorsForPathfinding = false,
                requireCamerasForPathfinding = false,
                usePlanetAwarePathfinding = false,
                minWaypointDistance = 25f,
                maxWaypointDistance = 100f
            };

            var pathfinder = new DirectPathfinder(config);

            foreach (var distance in distances)
            {
                var context = TestHelpers.CreateTestContext(Vector3.Zero, Vector3D.Zero);
                var start = new Vector3D(0, 0, 0);
                var end = new Vector3D(distance, 0, 0);

                var sw = Stopwatch.StartNew();
                for (int i = 0; i < iterations; i++)
                {
                    pathfinder.GetNextWaypoint(start, end, context);
                }
                sw.Stop();

                var avgUs = (sw.ElapsedMilliseconds * 1000.0) / iterations;
                var wpCount = (int)(distance / context.WaypointDistance) + 1;

                Console.WriteLine($"{distance,8:F0} | {iterations,10} | {avgUs,8:F2} | {sw.ElapsedMilliseconds,10} | {wpCount,8}");
            }
            Console.WriteLine();
        }

        #endregion

        #region A* Pathfinder Benchmarks

        private static void BenchmarkAStar_MaxPathNodes()
        {
            Console.WriteLine("═══ A* Pathfinder: Max Path Nodes Limit ═══");
            Console.WriteLine("Testing performance with varying node limits");
            Console.WriteLine();

            var maxNodes = new int[] { 50, 100, 250, 500, 1000, 2000, 5000 };
            var iterations = 100;
            var distance = 1000.0;

            Console.WriteLine("Max Nodes | Iterations | Avg (ms) | Total (ms) | Waypoints");
            Console.WriteLine("----------|------------|----------|------------|----------");

            foreach (var nodes in maxNodes)
            {
                var config = new FakeConfig
                {
                    allowAStar = true,
                    allowRepathing = false,
                    requireSensorsForPathfinding = false,
                    requireCamerasForPathfinding = false,
                    usePlanetAwarePathfinding = false,
                    maxPathNodes = nodes,
                    minWaypointDistance = 25f,
                    maxWaypointDistance = 100f
                };

                var pathfinder = new AStarPathfinder(config);
                var context = TestHelpers.CreateTestContext(Vector3.Zero, Vector3D.Zero);

                var start = new Vector3D(0, 0, 0);
                var end = new Vector3D(distance, 0, 0);

                var sw = Stopwatch.StartNew();
                int totalWaypoints = 0;
                for (int i = 0; i < iterations; i++)
                {
                    var path = pathfinder.CalculatePath(start, end, context);
                    totalWaypoints += path.Count;
                }
                sw.Stop();

                var avgMs = sw.ElapsedMilliseconds / (double)iterations;
                var avgWaypoints = totalWaypoints / iterations;

                Console.WriteLine($"{nodes,9} | {iterations,10} | {avgMs,8:F2} | {sw.ElapsedMilliseconds,10} | {avgWaypoints,9}");
            }
            Console.WriteLine();
        }

        private static void BenchmarkAStar_GridSpacing()
        {
            Console.WriteLine("═══ A* Pathfinder: Grid Spacing (Waypoint Distance) ═══");
            Console.WriteLine("Testing A* with varying grid spacing");
            Console.WriteLine();

            var gridSpacings = new float[] { 10f, 25f, 50f, 100f, 200f, 500f };
            var iterations = 100;
            var distance = 1000.0;

            Console.WriteLine("Grid Space | Iterations | Avg (ms) | Total (ms) | Waypoints");
            Console.WriteLine("-----------|------------|----------|------------|----------");

            foreach (var spacing in gridSpacings)
            {
                var config = new FakeConfig
                {
                    allowAStar = true,
                    allowRepathing = false,
                    requireSensorsForPathfinding = false,
                    requireCamerasForPathfinding = false,
                    usePlanetAwarePathfinding = false,
                    maxPathNodes = 1000,
                    minWaypointDistance = spacing * 0.5f,
                    maxWaypointDistance = spacing
                };

                var pathfinder = new AStarPathfinder(config);
                var context = TestHelpers.CreateTestContext(Vector3.Zero, Vector3D.Zero);
                context.WaypointDistance = spacing;

                var start = new Vector3D(0, 0, 0);
                var end = new Vector3D(distance, 0, 0);

                var sw = Stopwatch.StartNew();
                int totalWaypoints = 0;
                for (int i = 0; i < iterations; i++)
                {
                    var path = pathfinder.CalculatePath(start, end, context);
                    totalWaypoints += path.Count;
                }
                sw.Stop();

                var avgMs = sw.ElapsedMilliseconds / (double)iterations;
                var avgWaypoints = totalWaypoints / iterations;

                Console.WriteLine($"{spacing,10:F0} | {iterations,10} | {avgMs,8:F2} | {sw.ElapsedMilliseconds,10} | {avgWaypoints,9}");
            }
            Console.WriteLine();
        }

        private static void BenchmarkAStar_ObstacleCount()
        {
            Console.WriteLine("═══ A* Pathfinder: Obstacle Count Impact ═══");
            Console.WriteLine("Testing A* with varying numbers of obstacles");
            Console.WriteLine();

            var obstacleCounts = new int[] { 0, 1, 2, 5, 10, 20, 50 };
            var iterations = 100;

            Console.WriteLine("Obstacles | Iterations | Avg (ms) | Total (ms)");
            Console.WriteLine("----------|------------|----------|----------");

            foreach (var count in obstacleCounts)
            {
                var config = new FakeConfig
                {
                    allowAStar = true,
                    allowRepathing = false,
                    requireSensorsForPathfinding = true,
                    requireCamerasForPathfinding = false,
                    usePlanetAwarePathfinding = false,
                    maxPathNodes = 1000,
                    minWaypointDistance = 25f,
                    maxWaypointDistance = 100f
                };

                var pathfinder = new AStarPathfinder(config);
                var context = TestHelpers.CreateTestContext(Vector3.Zero, Vector3D.Zero);

                // Add obstacles
                context.SensorInfos = new List<PathfindingContext.SensorInfo>();
                for (int i = 0; i < count; i++)
                {
                    context.SensorInfos.Add(new PathfindingContext.SensorInfo
                    {
                        Position = new Vector3D(100 + i * 50, (i % 2) * 50 - 25, 0),
                        MaxRange = 30f
                    });
                }

                var start = new Vector3D(0, 0, 0);
                var end = new Vector3D(500, 0, 0);

                var sw = Stopwatch.StartNew();
                for (int i = 0; i < iterations; i++)
                {
                    pathfinder.CalculatePath(start, end, context);
                }
                sw.Stop();

                var avgMs = sw.ElapsedMilliseconds / (double)iterations;

                Console.WriteLine($"{count,9} | {iterations,10} | {avgMs,8:F2} | {sw.ElapsedMilliseconds,10}");
            }
            Console.WriteLine();
        }

        private static void BenchmarkAStar_Distance()
        {
            Console.WriteLine("═══ A* Pathfinder: Distance Scaling (Detailed) ═══");
            Console.WriteLine("Testing A* performance across distance ranges");
            Console.WriteLine();

            var distances = new double[] {
                50, 100, 200, 500, 1000,
                2000, 5000, 10000, 20000
            };
            var iterations = 100;

            Console.WriteLine("Distance | Iterations | Avg (ms) | Total (ms) | Waypoints");
            Console.WriteLine("---------|------------|----------|------------|----------");

            var config = new FakeConfig
            {
                allowAStar = true,
                allowRepathing = false,
                requireSensorsForPathfinding = false,
                requireCamerasForPathfinding = false,
                usePlanetAwarePathfinding = false,
                maxPathNodes = 1000,
                minWaypointDistance = 25f,
                maxWaypointDistance = 100f
            };

            var pathfinder = new AStarPathfinder(config);

            foreach (var distance in distances)
            {
                var context = TestHelpers.CreateTestContext(Vector3.Zero, Vector3D.Zero);
                var start = new Vector3D(0, 0, 0);
                var end = new Vector3D(distance, 0, 0);

                var sw = Stopwatch.StartNew();
                int totalWaypoints = 0;
                for (int i = 0; i < iterations; i++)
                {
                    var path = pathfinder.CalculatePath(start, end, context);
                    totalWaypoints += path.Count;
                }
                sw.Stop();

                var avgMs = sw.ElapsedMilliseconds / (double)iterations;
                var avgWaypoints = totalWaypoints / iterations;

                Console.WriteLine($"{distance,8:F0} | {iterations,10} | {avgMs,8:F2} | {sw.ElapsedMilliseconds,10} | {avgWaypoints,9}");
            }
            Console.WriteLine();
        }

        #endregion

        #region Realistic Scenario Benchmarks

        private static void BenchmarkRealisticScenarios()
        {
            Console.WriteLine("═══ Realistic Scenario Benchmarks ═══");
            Console.WriteLine("Testing common real-world pathfinding scenarios");
            Console.WriteLine();

            BenchmarkScenario_ShortRangeNavigation();
            BenchmarkScenario_LongRangeTravel();
            BenchmarkScenario_PlanetaryApproach();
            BenchmarkScenario_ObstacleField();
        }

        private static void BenchmarkScenario_ShortRangeNavigation()
        {
            Console.WriteLine("--- Scenario: Short Range Navigation (< 500m) ---");

            var config = new FakeConfig
            {
                allowRepathing = true,
                requireSensorsForPathfinding = true,
                requireCamerasForPathfinding = true,
                usePlanetAwarePathfinding = false,
                maxRepositionAttempts = 5,
                minWaypointDistance = 25f,
                maxWaypointDistance = 50f
            };

            var pathfinder = new DirectPathfinder(config);
            var context = TestHelpers.CreateTestContext(Vector3.Zero, Vector3D.Zero);

            var scenarios = new[]
            {
                (start: new Vector3D(0, 0, 0), end: new Vector3D(50, 0, 0)),
                (start: new Vector3D(0, 0, 0), end: new Vector3D(100, 50, 0)),
                (start: new Vector3D(0, 0, 0), end: new Vector3D(250, 100, 50)),
                (start: new Vector3D(0, 0, 0), end: new Vector3D(500, 0, 0))
            };

            var iterations = 1000;

            Console.WriteLine("Distance | Iterations | Avg (µs) | Total (ms)");
            Console.WriteLine("---------|------------|----------|----------");

            foreach (var scenario in scenarios)
            {
                var distance = Vector3D.Distance(scenario.start, scenario.end);

                var sw = Stopwatch.StartNew();
                for (int i = 0; i < iterations; i++)
                {
                    pathfinder.GetNextWaypoint(scenario.start, scenario.end, context);
                }
                sw.Stop();

                var avgUs = (sw.ElapsedMilliseconds * 1000.0) / iterations;
                Console.WriteLine($"{distance,8:F1} | {iterations,10} | {avgUs,8:F2} | {sw.ElapsedMilliseconds,10}");
            }
            Console.WriteLine();
        }

        private static void BenchmarkScenario_LongRangeTravel()
        {
            Console.WriteLine("--- Scenario: Long Range Travel (> 10km) ---");

            var config = new FakeConfig
            {
                allowAStar = false,
                allowRepathing = false,
                requireSensorsForPathfinding = false,
                requireCamerasForPathfinding = false,
                usePlanetAwarePathfinding = false,
                minWaypointDistance = 100f,
                maxWaypointDistance = 1000f
            };

            var pathfinder = new DirectPathfinder(config);
            var context = TestHelpers.CreateTestContext(Vector3.Zero, Vector3D.Zero);
            context.WaypointDistance = 1000f;

            var scenarios = new[]
            {
                (start: new Vector3D(0, 0, 0), end: new Vector3D(10000, 0, 0)),
                (start: new Vector3D(0, 0, 0), end: new Vector3D(50000, 0, 0)),
                (start: new Vector3D(0, 0, 0), end: new Vector3D(100000, 0, 0)),
                (start: new Vector3D(0, 0, 0), end: new Vector3D(500000, 0, 0))
            };

            var iterations = 1000;

            Console.WriteLine("Distance | Iterations | Avg (µs) | Total (ms)");
            Console.WriteLine("---------|------------|----------|----------");

            foreach (var scenario in scenarios)
            {
                var distance = Vector3D.Distance(scenario.start, scenario.end);

                var sw = Stopwatch.StartNew();
                for (int i = 0; i < iterations; i++)
                {
                    pathfinder.GetNextWaypoint(scenario.start, scenario.end, context);
                }
                sw.Stop();

                var avgUs = (sw.ElapsedMilliseconds * 1000.0) / iterations;
                Console.WriteLine($"{distance,8:F0} | {iterations,10} | {avgUs,8:F2} | {sw.ElapsedMilliseconds,10}");
            }
            Console.WriteLine();
        }

        private static void BenchmarkScenario_PlanetaryApproach()
        {
            Console.WriteLine("--- Scenario: Planetary Surface Navigation ---");

            var config = new FakeConfig
            {
                allowRepathing = true,
                requireSensorsForPathfinding = false,
                requireCamerasForPathfinding = false,
                usePlanetAwarePathfinding = true,
                minAltitudeBuffer = 50f,
                minWaypointDistance = 50f,
                maxWaypointDistance = 200f
            };

            var pathfinder = new DirectPathfinder(config);
            var context = TestHelpers.CreateTestContext(new Vector3(0, -9.81f, 0), Vector3D.Zero);
            TestHelpers.AddPlanetToContext(context, new Vector3D(0, -60000, 0), 60000.0);

            var scenarios = new[]
            {
                (start: new Vector3D(0, 100, 0), end: new Vector3D(500, 100, 0)),
                (start: new Vector3D(0, 100, 0), end: new Vector3D(1000, 100, 0)),
                (start: new Vector3D(0, 100, 0), end: new Vector3D(5000, 100, 0)),
                (start: new Vector3D(0, 100, 0), end: new Vector3D(10000, 100, 0))
            };

            var iterations = 1000;

            Console.WriteLine("Distance | Iterations | Avg (µs) | Total (ms)");
            Console.WriteLine("---------|------------|----------|----------");

            foreach (var scenario in scenarios)
            {
                var distance = Vector3D.Distance(scenario.start, scenario.end);

                var sw = Stopwatch.StartNew();
                for (int i = 0; i < iterations; i++)
                {
                    pathfinder.GetNextWaypoint(scenario.start, scenario.end, context);
                }
                sw.Stop();

                var avgUs = (sw.ElapsedMilliseconds * 1000.0) / iterations;
                Console.WriteLine($"{distance,8:F0} | {iterations,10} | {avgUs,8:F2} | {sw.ElapsedMilliseconds,10}");
            }
            Console.WriteLine();
        }

        private static void BenchmarkScenario_ObstacleField()
        {
            Console.WriteLine("--- Scenario: Dense Obstacle Field Navigation ---");

            var config = new FakeConfig
            {
                allowAStar = true,
                allowRepathing = true,
                requireSensorsForPathfinding = true,
                requireCamerasForPathfinding = false,
                usePlanetAwarePathfinding = false,
                maxRepositionAttempts = 10,
                maxPathNodes = 1000,
                minWaypointDistance = 25f,
                maxWaypointDistance = 100f
            };

            var obstacleCounts = new int[] { 5, 10, 20, 50 };
            var iterations = 100; // Fewer iterations for A*

            Console.WriteLine("Obstacles | Iterations | Avg (ms) | Total (ms)");
            Console.WriteLine("----------|------------|----------|----------");

            foreach (var obstacleCount in obstacleCounts)
            {
                var pathfinder = new AStarPathfinder(config);
                var context = TestHelpers.CreateTestContext(Vector3.Zero, Vector3D.Zero);

                // Create dense obstacle field
                context.SensorInfos = new List<PathfindingContext.SensorInfo>();
                for (int i = 0; i < obstacleCount; i++)
                {
                    var x = 50 + (i * 100) % 1000;
                    var y = ((i * 37) % 3 - 1) * 50; // Vary Y position
                    var z = ((i * 23) % 3 - 1) * 50; // Vary Z position

                    context.SensorInfos.Add(new PathfindingContext.SensorInfo
                    {
                        Position = new Vector3D(x, y, z),
                        MaxRange = 40f
                    });
                }

                var start = new Vector3D(0, 0, 0);
                var end = new Vector3D(1000, 0, 0);

                var sw = Stopwatch.StartNew();
                for (int i = 0; i < iterations; i++)
                {
                    pathfinder.CalculatePath(start, end, context);
                }
                sw.Stop();

                var avgMs = sw.ElapsedMilliseconds / (double)iterations;
                Console.WriteLine($"{obstacleCount,9} | {iterations,10} | {avgMs,8:F2} | {sw.ElapsedMilliseconds,10}");
            }
            Console.WriteLine();
        }

        #endregion
    }
}