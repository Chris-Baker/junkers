using Game.Tags;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;

namespace Game.Systems
{
    /// <summary>
    /// For all characters we lock the velocity and inertia so they can be controlled easily
    /// and don't topple over or get spun around by obstacles
    /// </summary>
    public class PhysicsCharacterSystem : JobComponentSystem
    {
        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            JobHandle jobHandle = Entities.ForEach((ref PhysicsVelocity physicsVelocity, ref PhysicsMass physicsMass, in Character character) => 
            {
                // zero out velocity
                physicsVelocity.Angular = float3.zero;
                physicsVelocity.Linear = float3.zero;
                
                // lock our inertia
                physicsMass.InverseInertia = float3.zero;

            }).Schedule(inputDeps);

            return jobHandle;
        }
    }
}
