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

        public void CalculateThrust(List<IMyThrust> thrusters)
        {
            foreach (var thruster in thrusters)
            {
                if (!thruster.IsWorking) continue;

                Vector3I thrustDirection = thruster.GridThrustDirection;

                if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Forward))
                    Forward += thruster.MaxEffectiveThrust;
                else if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Backward))
                    Backward += thruster.MaxEffectiveThrust;
                else if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Up))
                    Up += thruster.MaxEffectiveThrust;
                else if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Down))
                    Down += thruster.MaxEffectiveThrust;
                else if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Left))
                    Left += thruster.MaxEffectiveThrust;
                else if (thrustDirection == Base6Directions.GetIntVector(Base6Directions.Direction.Right))
                    Right += thruster.MaxEffectiveThrust;
            }
        }
    }
}