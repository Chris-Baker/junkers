using Game.Components;
using Unity.Entities;
using Unity.Transforms;
using UnityEngine;

namespace Game.Systems
{
    // https://forum.unity.com/threads/ecs-hybrid-architectural-approaches.746864/
    class AnimatorSystem : ComponentSystem {
        
        private EntityQuery _animatorQuery;
        private static readonly int VelocityX = Animator.StringToHash("VelocityX");
        private static readonly int VelocityY = Animator.StringToHash("VelocityY");

        protected override void OnCreate() {
            _animatorQuery = GetEntityQuery(typeof(Animator), typeof(MoveDirection));
        }
        
        protected override void OnUpdate() {
            Entities.With(_animatorQuery).ForEach((Entity entity, Animator animator, ref MoveDirection moveDirection, ref Rotation rotation) =>
            {
                animator.SetFloat(VelocityX, moveDirection.Direction.x);
                animator.SetFloat(VelocityY, moveDirection.Direction.z);
            });
        }
    }
}