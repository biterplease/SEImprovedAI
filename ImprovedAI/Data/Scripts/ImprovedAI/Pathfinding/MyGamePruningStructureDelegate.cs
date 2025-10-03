using Sandbox.Game.Entities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRage.Game.Entity;
using VRageMath;

namespace ImprovedAI.Pathfinding
{
    /// <summary>
    /// Delegate interface for MyGamePruningStructure operations
    /// Allows mocking of static game methods during testing
    /// </summary>
    public interface IMyGamePruningStructureDelegate
    {
        void GetTopmostEntitiesOverlappingRay(
            ref LineD line,
            List<MyLineSegmentOverlapResult<MyEntity>> result,
            MyEntityQueryType queryType = MyEntityQueryType.Both);

        IMyPlanetDelegate GetClosestPlanet(Vector3D position);
    }

    /// <summary>
    /// Production implementation using real SE API
    /// </summary>
    public class MyGamePruningStructureDelegate : IMyGamePruningStructureDelegate
    {
        public void GetTopmostEntitiesOverlappingRay(
            ref LineD line,
            List<MyLineSegmentOverlapResult<MyEntity>> result,
            MyEntityQueryType queryType = MyEntityQueryType.Both)
        {
            MyGamePruningStructure.GetTopmostEntitiesOverlappingRay(ref line, result, queryType);
        }
        public IMyPlanetDelegate GetClosestPlanet(Vector3D position)
        {
            return (IMyPlanetDelegate)MyGamePruningStructure.GetClosestPlanet(position);
        }
    }
}
