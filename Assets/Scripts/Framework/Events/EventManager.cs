using Framework.Events.Tags;
using Framework.Physics.Tags;
using Unity.Entities;

namespace Framework.Events
{
    public static class EventManager
    {
        public static void TriggerEvent<T>(EntityManager entityManager, T eventData) where T : struct, IComponentData
        {
            // create an event entity
            Entity gameEvent = entityManager.CreateEntity(typeof(T));
        
            // add our base event data
            entityManager.AddComponentData(gameEvent, new GameEvent());

            // add our custom event data
            entityManager.AddComponentData(gameEvent, eventData);
        }
        
        public static void TriggerEvent<T>(EntityCommandBuffer.ParallelWriter commandBuffer, int index, T eventData) where T : struct, IComponentData
        {
            // create an event entity
            Entity gameEvent = commandBuffer.CreateEntity(index);
        
            // add our base event data
            commandBuffer.AddComponent(index, gameEvent, new GameEvent {});

            // add our custom event data
            commandBuffer.AddComponent(index, gameEvent, eventData);
        }
        
        public static void TriggerEvent<T>(EntityCommandBuffer commandBuffer, T eventData) where T : struct, IComponentData
        {
            // create an event entity
            Entity gameEvent = commandBuffer.CreateEntity();
        
            // add our base event data
            commandBuffer.AddComponent(gameEvent, new GameEvent {});

            // add our custom event data
            commandBuffer.AddComponent(gameEvent, eventData);
        }
        
        public static void TriggerEvent<TEventData, TCollisionType>(EntityManager entityManager, TEventData eventData, TCollisionType collision) 
            where TEventData : struct, IComponentData 
            where TCollisionType: struct, ICollision
        {
            // create an event entity
            Entity gameEvent = entityManager.CreateEntity(typeof(TEventData));
        
            // add our base event data
            entityManager.AddComponentData(gameEvent, new GameEvent());

            // add our custom event data
            entityManager.AddComponentData(gameEvent, eventData);
            entityManager.AddComponentData(gameEvent, collision);
        }
        
        public static void TriggerEvent<TEventData, TCollisionType>(EntityCommandBuffer.ParallelWriter commandBuffer, int index, TEventData eventData, TCollisionType collision) 
            where TEventData : struct, IComponentData 
            where TCollisionType: struct, ICollision
        {
            // create an event entity
            Entity gameEvent = commandBuffer.CreateEntity(index);
        
            // add our base event data
            commandBuffer.AddComponent(index, gameEvent, new GameEvent {});

            // add our custom event data
            commandBuffer.AddComponent(index, gameEvent, eventData);
            commandBuffer.AddComponent(index, gameEvent, collision);
        }
        
        public static void TriggerEvent<TEventData, TCollisionType>(EntityCommandBuffer commandBuffer, TEventData eventData, TCollisionType collision)
            where TEventData : struct, IComponentData 
            where TCollisionType: struct, ICollision
        {
            // create an event entity
            Entity gameEvent = commandBuffer.CreateEntity();
        
            // add our base event data
            commandBuffer.AddComponent(gameEvent, new GameEvent {});

            // add our custom event data
            commandBuffer.AddComponent(gameEvent, eventData);
            commandBuffer.AddComponent(gameEvent, collision);
        }
    }
}
