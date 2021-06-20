using Game.Components;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Rendering;
using Unity.Transforms;

namespace Game.Systems
{
    public class MovementSystem : JobComponentSystem
    {
        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            float deltaTime = Time.DeltaTime;
            
            JobHandle jobHandle = Entities.ForEach((ref Translation translation, ref Rotation rotation, ref MoveSpeed moveSpeed, in MoveDirection moveDirection) =>
            {
                float3 forward = math.forward(rotation.Value);
                float3 right = math.cross(forward, math.up()) * -1;
                translation.Value += right * (moveDirection.Direction.x * (moveSpeed.Max + moveSpeed.Over) * deltaTime);
                translation.Value += forward * (moveDirection.Direction.z * (moveSpeed.Max + moveSpeed.Over) * deltaTime);
            }).Schedule(inputDeps);

            return jobHandle;
        }
    }
}
