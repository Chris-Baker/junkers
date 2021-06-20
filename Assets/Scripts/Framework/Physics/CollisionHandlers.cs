using System;
using System.Collections.Generic;
using Framework.Physics.Components;
using Game.Components;
using Unity.Entities;

namespace Framework.Physics
{
    public static class CollisionHandlers
    {
        // TODO dynamic type here should be T where T : struct, ICollision but /shrug 
        public static readonly Dictionary<int, Action<Collision, dynamic, ComponentDataFromEntity<PhysicsLayer>>> Handlers 
            = new Dictionary<int, Action<Collision, dynamic, ComponentDataFromEntity<PhysicsLayer>>>();
    }
}