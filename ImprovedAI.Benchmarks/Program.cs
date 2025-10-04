using BenchmarkDotNet.Configs;
using BenchmarkDotNet.Running;
using System;

namespace ImprovedAI.Benchmarks
{
    /// <summary>
    /// Entry point for ImprovedAI benchmark suite.
    /// Run with: dotnet run -c Release
    /// </summary>
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("=== ImprovedAI Pathfinding Benchmarks ===");
            Console.WriteLine();

            if (args.Length == 0)
            {
                ShowMenu();
                return;
            }

            // Allow filtering from command line
            var config = DefaultConfig.Instance;
            BenchmarkSwitcher.FromAssembly(typeof(Program).Assembly).Run(args, config);
        }

        static void ShowMenu()
        {
            Console.WriteLine("Select benchmark suite to run:");
            Console.WriteLine();
            Console.WriteLine("1. DirectPathfinder - Basic performance tests");
            Console.WriteLine("2. DirectPathfinder - Parameter variations");
            Console.WriteLine("3. AStarPathfinder - Basic performance tests");
            Console.WriteLine("4. AStarPathfinder - Parameter variations");
            Console.WriteLine("5. PathfindingManager - Coordination overhead");
            Console.WriteLine("6. PathfindingManager - Selection benchmarks");
            Console.WriteLine("7. PathfindingContext - Context operations");
            Console.WriteLine("8. All Pathfinding Benchmarks");
            Console.WriteLine("9. Quick Comparison (selected benchmarks only)");
            Console.WriteLine("0. Exit");
            Console.WriteLine();
            Console.Write("Enter selection (0-9): ");

            var input = Console.ReadLine();
            Console.WriteLine();

            switch (input)
            {
                case "1":
                    RunBenchmark<Pathfinding.DirectPathfinderBenchmarks>();
                    break;
                case "2":
                    RunBenchmark<Pathfinding.DirectPathfinderParameterBenchmarks>();
                    break;
                case "3":
                    RunBenchmark<Pathfinding.AStarPathfinderBenchmarks>();
                    break;
                case "4":
                    RunBenchmark<Pathfinding.AStarParameterBenchmarks>();
                    break;
                case "5":
                    RunBenchmark<Pathfinding.PathfindingManagerBenchmarks>();
                    break;
                case "6":
                    RunBenchmark<Pathfinding.PathfindingManagerSelectionBenchmarks>();
                    break;
                case "7":
                    RunBenchmark<Pathfinding.PathfindingContextBenchmarks>();
                    RunBenchmark<Pathfinding.PathfindingContextMultiDirectionBenchmarks>();
                    Console.WriteLine("PathfindingContext benchmarks - check PathfindingContextBenchmarks.cs");
                    Console.WriteLine("(These may already exist in the project)");
                    break;
                case "8":
                    RunAllPathfindingBenchmarks();
                    break;
                case "9":
                    RunQuickComparison();
                    break;
                case "0":
                    return;
                default:
                    Console.WriteLine("Invalid selection");
                    break;
            }

            Console.WriteLine();
            Console.WriteLine("Benchmark complete! Results saved to BenchmarkDotNet.Artifacts/results/");
            Console.WriteLine("Press any key to continue...");
            Console.ReadKey();
        }

        static void RunBenchmark<T>()
        {
            Console.WriteLine($"Running {typeof(T).Name}...");
            Console.WriteLine();
            BenchmarkRunner.Run<T>();
        }

        static void RunAllPathfindingBenchmarks()
        {
            Console.WriteLine("Running all pathfinding benchmarks...");
            Console.WriteLine("This may take 30-60 minutes depending on your system.");
            Console.WriteLine();

            var benchmarks = new[]
            {
                typeof(Pathfinding.DirectPathfinderBenchmarks),
                typeof(Pathfinding.DirectPathfinderParameterBenchmarks),
                typeof(Pathfinding.AStarPathfinderBenchmarks),
                typeof(Pathfinding.AStarParameterBenchmarks),
                typeof(Pathfinding.PathfindingManagerBenchmarks),
                typeof(Pathfinding.PathfindingManagerSelectionBenchmarks)
            };

            foreach (var benchmark in benchmarks)
            {
                Console.WriteLine($"Running {benchmark.Name}...");
                BenchmarkRunner.Run(benchmark);
                Console.WriteLine();
            }
        }

        static void RunQuickComparison()
        {
            Console.WriteLine("Running quick comparison benchmarks...");
            Console.WriteLine("This runs a subset of benchmarks for quick performance comparison.");
            Console.WriteLine();

            // Run only the baseline comparison benchmarks
            var config = DefaultConfig.Instance
                .WithOptions(ConfigOptions.DisableOptimizationsValidator);

            BenchmarkRunner.Run<Pathfinding.DirectPathfinderBenchmarks>(config);
            BenchmarkRunner.Run<Pathfinding.AStarPathfinderBenchmarks>(config);
            BenchmarkRunner.Run<Pathfinding.PathfindingManagerSelectionBenchmarks>(config);
        }
    }
}