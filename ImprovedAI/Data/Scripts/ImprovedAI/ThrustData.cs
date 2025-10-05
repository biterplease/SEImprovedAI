using Sandbox.ModAPI;
using System;
using System.Collections.Generic;
using VRageMath;

namespace ImprovedAI
{
    public struct ThrustData
    {
        public float Forward;
        public float Backward;
        public float Up;
        public float Down;
        public float Left;
        public float Right;

        // Convenience properties
        public float MaxForwardBackward => Math.Max(Forward, Backward);
        public float MaxUpDown => Math.Max(Up, Down);
        public float MaxLeftRight => Math.Max(Left, Right);
    }
}