using Framework.Events.Tags;
using Unity.Entities;
using Unity.Jobs;

namespace Framework.Events
{
    [UpdateAfter(typeof(EventGroup))]
    public class EventCleanupSystem : JobComponentSystem
    {
        private EndSimulationEntityCommandBufferSystem _entityCommandBufferSystem;

        protected override void OnCreate()
        {
            base.OnCreate();
            _entityCommandBufferSystem = World.GetOrCreateSystem<EndSimulationEntityCommandBufferSystem>();
        }
        
        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            EntityCommandBuffer.ParallelWriter commandBuffer = _entityCommandBufferSystem.CreateCommandBuffer().AsParallelWriter();
            
            JobHandle jobHandle = Entities.ForEach((Entity entity, int entityInQueryIndex, in GameEvent gameEvent) => 
            {
                commandBuffer.DestroyEntity(entityInQueryIndex, entity);
            }).Schedule(inputDeps);
            
            _entityCommandBufferSystem.AddJobHandleForProducer(jobHandle);
            
            return jobHandle;
        }
    }
}
