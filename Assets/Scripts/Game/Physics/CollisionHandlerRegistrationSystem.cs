using Framework.Events;
using Framework.Physics;
using Game.Components;
using Game.Events.Data;
using Unity.Entities;
using UnityEngine;

namespace Game.Physics
{
    public class CollisionHandlerRegistrationSystem : ComponentSystem
    {
        protected override void OnStartRunning()
        {
            CollisionHandlers.Handlers.Add(
                Utils.CollisionPair((int) CollisionGroup.PlayerBullet, (int) CollisionGroup.Wall), (collision, type, entities) =>
                {
                    // raise a game event
                    EventManager.TriggerEvent(EntityManager, new BulletAndWallCollisionEvent
                    {
                        Bullet = Utils.GetEntityInGroup(collision.EntityA, collision.EntityB, CollisionGroup.PlayerBullet, entities),
                        Wall = Utils.GetEntityInGroup(collision.EntityA, collision.EntityB, CollisionGroup.Wall, entities),
                    }, type);
                });
            
            CollisionHandlers.Handlers.Add(
                Utils.CollisionPair((int) CollisionGroup.Player, (int) CollisionGroup.Wall), (collision, type, entities) =>
                {
                    Debug.Log("Boing!");
                });
        }

        protected override void OnUpdate() {}

        protected override void OnDestroy()
        {
            CollisionHandlers.Handlers.Clear();
        }
    }
}