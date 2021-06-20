using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Systems;

namespace Framework.Physics.Systems
{
    public class RaycastSystem : ComponentSystem
    {
        // TODO finish this https://www.youtube.com/watch?v=B3SFWm9gkL8 12:40
        public Entity Raycast(float3 fromPosition, float3 toPosition)
        {
            BuildPhysicsWorld buildPhysicsWorld = World.DefaultGameObjectInjectionWorld.GetExistingSystem<BuildPhysicsWorld>();
            CollisionWorld collisionWorld = buildPhysicsWorld.PhysicsWorld.CollisionWorld;
            RaycastInput raycastInput = new RaycastInput
            {
                Start = fromPosition,
                End = toPosition,
                Filter = new CollisionFilter
                {
                    BelongsTo = ~0u, // all bits to 111... this is unsigned so we can't do -1
                    CollidesWith = ~0u,
                    GroupIndex = 0
                }
            };
            RaycastHit raycastHit = new RaycastHit();

            if (collisionWorld.CastRay(raycastInput, out raycastHit))
            {
                // hit something
                Entity hitEntity = buildPhysicsWorld.PhysicsWorld.Bodies[raycastHit.RigidBodyIndex].Entity;
                return hitEntity;
            }

            // hit nothing
            return Entity.Null;
        }

        protected override void OnUpdate()
        {
            // do nothing?
            // maybe here we want to go through some queue of raycasts and handle them in a job
        }
    }
}