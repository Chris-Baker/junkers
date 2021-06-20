using Framework.Events;
using Game.Events.Data;
using Unity.Entities;
using Unity.Jobs;

namespace Game.Events.Handlers
{
    [UpdateInGroup(typeof(EventGroup))]
    public class ShootEventHandler: JobComponentSystem
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
            JobHandle jobHandle = Entities.ForEach((int entityInQueryIndex, in ShootEvent shootEvent) =>
            {
                // create a bullet entity
                Entity bullet = commandBuffer.Instantiate(entityInQueryIndex, shootEvent.Prefab);
                
                // set our position
                commandBuffer.SetComponent(entityInQueryIndex, bullet, shootEvent.FromPosition);
                
                // spawn a bullet
            }).Schedule(inputDeps);

            _entityCommandBufferSystem.AddJobHandleForProducer(jobHandle);
            
            return jobHandle;
        }
    }
}