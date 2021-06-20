using System.Linq;
using Framework.Physics.Components;
using Game.Components;
using Unity.Collections;
using Unity.Entities;

namespace Framework.Physics
{
    public static class Utils
    {
        /// <summary>
        /// Merge any number of native collision maps into a single map. 
        /// </summary>
        /// <param name="target">The target map to merge the others into</param>
        /// <param name="others">The other maps to merge into target</param>
        /// <returns>All maps merged into target</returns>
        public static NativeHashMap<int, Collision> Merge(NativeHashMap<int, Collision> target, params NativeHashMap<int, Collision>[] others)
        {
            return others.Aggregate(target, Merge);
        }
        
        /// <summary>
        /// Merge a native collision map into the target map. 
        /// </summary>
        /// <param name="target">The target map to merge the other into</param>
        /// <param name="other">The other map to merge into target</param>
        /// <returns>All maps merged into target</returns>
        private static NativeHashMap<int, Collision> Merge(NativeHashMap<int, Collision> target, NativeHashMap<int, Collision> other)
        {
            NativeArray<int> keys = other.GetKeyArray(Allocator.Persistent);
            foreach (int key in keys)
            {
                target.Add(key, other[key]);
            }
            keys.Dispose();
            return target;
        }
        
        /// <summary>
        /// Map two integers into a single integer creating a unique pair.
        /// The lowest number is mapped to the first 16 bits and the highest number the second 16 bits.
        /// </summary>
        /// <param name="a">The first number</param>
        /// <param name="b">The second number</param>
        /// <returns>A single unique pair</returns>
        public static int CollisionPair(int a, int b)
        {
            // swap a and b if a is greater than b
            if (a > b)
            {
                a += b;
                b = a - b;
                a -= b;
            }
            
            // map a to the first half and b to the second
            return a | (b << 16);
        }
        
        public static Entity GetEntityInGroup(Entity entityA, Entity entityB, CollisionGroup @group,
            ComponentDataFromEntity<PhysicsLayer> physicsLayerEntities)
        {
            CollisionGroup groupA = physicsLayerEntities[entityA].Value;
            return (groupA == group) ? entityA : entityB;
        }
    }
}