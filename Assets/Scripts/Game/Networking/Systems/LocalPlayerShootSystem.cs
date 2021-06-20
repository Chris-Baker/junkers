using Framework.Events;
using Game.Components;
using Game.Events.Data;
using Game.Tags;
using Unity.Entities;
using Unity.Jobs;
using Unity.Transforms;
using UnityEngine.InputSystem;

namespace Game.Systems
{
    public class LocalPlayerShootSystem : JobComponentSystem
    {
        private BeginInitializationEntityCommandBufferSystem _entityCommandBufferSystem;

        protected override void OnCreate()
        {
            base.OnCreate();
            RequireSingletonForUpdate<DisabledSystem>();
            // Cache the BeginInitializationEntityCommandBufferSystem in a field, so we don't have to create it every frame
            _entityCommandBufferSystem = World.GetOrCreateSystem<BeginInitializationEntityCommandBufferSystem>();
        }
        
        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            // get input
            bool isShooting = Mouse.current.leftButton.isPressed;

            // early exit if we're not shooting
            if (!isShooting)
            {
                return default;
            }
            
            // we need a command buffer so we can create events at the next sync point on the main thread
            // we are using the begin initialisation command buffer so this runs at the start of the next frame
            EntityCommandBuffer.ParallelWriter commandBuffer = _entityCommandBufferSystem.CreateCommandBuffer().AsParallelWriter();

            // get the time so we can set when we can next shoot
            double time = Time.ElapsedTime;

            // ref to read write, in to read
            JobHandle jobHandle = Entities.ForEach((int entityInQueryIndex, ref Weapon weapon, in Translation translation) =>
            {
                // early exit if weapon is on cooldown
                if (time < weapon.NextShotTime)
                {
                    return;
                }
                
                // trigger a shoot event
                EventManager.TriggerEvent(commandBuffer, entityInQueryIndex, new ShootEvent
                {
                    FromPosition = translation,
                    Prefab = weapon.BulletType
                });

                // put our weapon on cooldown
                weapon.NextShotTime = time + weapon.Cooldown;
            }).Schedule(inputDeps);

            _entityCommandBufferSystem.AddJobHandleForProducer(jobHandle);
            
            return jobHandle;
        }
    }
}
