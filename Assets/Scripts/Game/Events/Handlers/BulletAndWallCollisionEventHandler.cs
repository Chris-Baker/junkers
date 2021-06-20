using Framework.Events;
using Framework.Physics.Tags;
using Game.Events.Data;
using Unity.Entities;
using Unity.Jobs;

namespace Game.Events.Handlers
{
    [UpdateInGroup(typeof(EventGroup))]
    public class BulletAndWallCollisionEventHandler: JobComponentSystem
    {
        private BeginInitializationEntityCommandBufferSystem _entityCommandBufferSystem;

        protected override void OnCreate()
        {
            base.OnCreate();
            // Cache the BeginInitializationEntityCommandBufferSystem in a field, so we don't have to create it every frame
            _entityCommandBufferSystem = World.GetOrCreateSystem<BeginInitializationEntityCommandBufferSystem>();
        }
        
        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            // we need a command buffer so we can create events at the next sync point on the main thread
            // we are using the begin initialisation command buffer so this runs at the start of the next frame
            EntityCommandBuffer.ParallelWriter commandBuffer = _entityCommandBufferSystem.CreateCommandBuffer().AsParallelWriter();

            // ref to read write, in to read
            JobHandle jobHandle = Entities.ForEach((int entityInQueryIndex, in BulletAndWallCollisionEvent gameEvent, in EnterCollision type) =>
            {
                // destroy the bullet entity
                commandBuffer.DestroyEntity(entityInQueryIndex, gameEvent.Bullet);
                
                // spawn some effect
                
                // make a sound
                
            }).Schedule(inputDeps);
            
            _entityCommandBufferSystem.AddJobHandleForProducer(jobHandle);
            return jobHandle;
        }
    }
}