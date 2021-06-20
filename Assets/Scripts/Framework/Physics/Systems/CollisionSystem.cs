using Game.Components;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Physics;
using Unity.Physics.Systems;
using Collision = Framework.Physics.Components.Collision;

namespace Framework.Physics.Systems
{
    [UpdateAfter(typeof(FixedStepSimulationSystemGroup))]
    public class CollisionSystem : JobComponentSystem
    {
        private BuildPhysicsWorld _buildPhysicsWorld;
        private StepPhysicsWorld _stepPhysicsWorld;
        private PhysicsSystem _physicsSystem;

        protected override void OnCreate()
        {
            _buildPhysicsWorld = World.GetOrCreateSystem<BuildPhysicsWorld>();
            _stepPhysicsWorld = World.GetOrCreateSystem<StepPhysicsWorld>();
            _physicsSystem = World.GetOrCreateSystem<PhysicsSystem>();
        }
        
        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            CollisionJob collisionJob = new CollisionJob
            {
                PhysicsLayerEntities = GetComponentDataFromEntity<PhysicsLayer>(),
                Enter = _physicsSystem.Enter,
                Stay = _physicsSystem.Stay,
                Exit = _physicsSystem.Exit
            };
            JobHandle jobHandle = collisionJob.Schedule(_stepPhysicsWorld.Simulation, ref _buildPhysicsWorld.PhysicsWorld, inputDeps);
            return jobHandle;
        }

        private struct CollisionJob : ICollisionEventsJob
        {
            public ComponentDataFromEntity<PhysicsLayer> PhysicsLayerEntities;
            public NativeHashMap<int, Collision> Enter;
            public NativeHashMap<int, Collision> Stay;
            public NativeHashMap<int, Collision> Exit;
            

            public void Execute(CollisionEvent triggerEvent)
            {
                if (!PhysicsLayerEntities.HasComponent(triggerEvent.EntityA) ||
                    !PhysicsLayerEntities.HasComponent(triggerEvent.EntityB))
                {
                    return;
                }
                
                int key = Utils.CollisionPair(triggerEvent.EntityA.Index, triggerEvent.EntityB.Index);
                Collision collision = new Collision
                {
                    EntityA = triggerEvent.EntityA,
                    EntityB = triggerEvent.EntityB,
                };
                    
                // if the collision is in the exit set then we want to remove it from the exit set
                // and add it to the stay set.
                // else we have a new collision and we add it to the enter set
                if (Exit.ContainsKey(key))
                {
                    Exit.Remove(key);
                    Stay.Add(key, collision);
                }
                else
                {
                    // we want to add a new collision
                    Enter.Add(key, collision);                        
                }
            }
        }
    }
}