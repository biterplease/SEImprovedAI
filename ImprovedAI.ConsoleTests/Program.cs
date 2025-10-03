using System;
using ImprovedAI.Tests;

namespace ImprovedAI.TestRunner
{
    class Program
    {
        static void Main(string[] args)
        {
            if (args.Length == 0)
                args = new[] { "all" };

            PathfindingTestRunner.RunAll(args);
        }
    }
}