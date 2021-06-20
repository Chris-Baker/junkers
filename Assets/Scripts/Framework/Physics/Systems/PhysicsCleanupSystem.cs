using Framework.Events;
using Framework.Physics.Tags;
using Game.Components;
using Game.Events.Data;
using Unity.Collections;
using Unity.Entities;
using UnityEngine;
using Collision = Framework.Physics.Components.Collision;

namespace Framework.Physics.Systems
{
    [UpdateAfter(typeof(TriggerSystem))]
    public class PhysicsCleanupSystem : ComponentSystem
    {
        private PhysicsSystem _physicsSystem;
        
        protected override void OnCreate()
        {
            base.OnCreate();
            _physicsSystem = World.GetOrCreateSystem<PhysicsSystem>();
        }

        protected override void OnUpdate()
        {
            ComponentDataFromEntity<PhysicsLayer> physicsLayerEntities = GetComponentDataFromEntity<PhysicsLayer>();
            ProcessCollisions(_physicsSystem.Enter, new EnterCollision(), physicsLayerEntities);
            ProcessCollisions(_physicsSystem.Stay, new StayCollision(), physicsLayerEntities);
            ProcessCollisions(_physicsSystem.Exit, new ExitCollision(), physicsLayerEntities);
        }

        private void ProcessCollisions<T>(NativeHashMap<int, Collision> collisions, T type,
            ComponentDataFromEntity<PhysicsLayer> physicsLayerEntities) where T : struct, ICollision
        {
            NativeArray<Collision> values = collisions.GetValueArray(Allocator.Temp);
            foreach (Collision collision in values)
            {
                Handle(collision, type, physicsLayerEntities);
            }

            values.Dispose();
        }
        
        private void Handle<T>(Collision collision, T type, ComponentDataFromEntity<PhysicsLayer> physicsLayerEntities) where T : struct, ICollision
        {
            // If either Entity has been destroyed already we don't need to handle the collision
            if (!physicsLayerEntities.HasComponent(collision.EntityA) || !physicsLayerEntities.HasComponent(collision.EntityB))
            {
                return;
            }
            
            // get our entity collision groups
            CollisionGroup groupA = physicsLayerEntities[collision.EntityA].Value;
            CollisionGroup groupB = physicsLayerEntities[collision.EntityB].Value;

            // get the pair number for these two groups
            int pair = Utils.CollisionPair((int) groupA, (int) groupB);

            // handle the event
            if (CollisionHandlers.Handlers.ContainsKey(pair))
            {
                CollisionHandlers.Handlers[pair].Invoke(collision, type, physicsLayerEntities);
            }
        }
    }
}