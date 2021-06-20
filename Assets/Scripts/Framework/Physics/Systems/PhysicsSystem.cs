using Framework.Physics.Components;
using Unity.Collections;
using Unity.Entities;

namespace Framework.Physics.Systems
{
    [UpdateBefore(typeof(TriggerSystem))]
    public class PhysicsSystem : ComponentSystem
    {
        private const int Capacity = 1024;
        public NativeHashMap<int, Collision> Enter;
        public NativeHashMap<int, Collision> Stay;
        public NativeHashMap<int, Collision> Exit;
        
        protected override void OnCreate()
        {
            base.OnCreate();
            Enter = new NativeHashMap<int, Collision>(Capacity, Allocator.Persistent);
            Stay = new NativeHashMap<int, Collision>(Capacity, Allocator.Persistent);
            Exit = new NativeHashMap<int, Collision>(Capacity, Allocator.Persistent);
        }

        protected override void OnUpdate()
        {
            Exit.Clear();
            Exit = Utils.Merge(Exit, Enter, Stay);
            Stay.Clear();
            Enter.Clear();
        }

        protected override void OnDestroy()
        {
            Enter.Dispose();
            Stay.Dispose();
            Exit.Dispose();
        }
    }
}