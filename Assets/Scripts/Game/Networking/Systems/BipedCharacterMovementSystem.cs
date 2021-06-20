using Game.CharacterController;
using Game.Components;
using Game.Tags;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine.InputSystem;

namespace Game.Systems
{
    public class BipedCharacterMovementSystem : JobComponentSystem
    {
        protected override void OnCreate() {
            
        }
        
        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            JobHandle jobHandle = Entities.ForEach((ref CharacterControllerInputs characterControllerInputs, in BipedCharacterInput inputs, in Rotation rotation) =>
            {
                characterControllerInputs.WorldMoveVector.x = 0;
                characterControllerInputs.WorldMoveVector.z = 0;
                
                if (inputs.Right)
                {
                    characterControllerInputs.WorldMoveVector.x += 1;
                }
                if (inputs.Left)
                {
                    characterControllerInputs.WorldMoveVector.x -= 1;
                }
                if (inputs.Up)
                {
                    characterControllerInputs.WorldMoveVector.z += 1;
                }
                if (inputs.Down)
                {
                    characterControllerInputs.WorldMoveVector.z -= 1;
                }

                characterControllerInputs.JumpRequested = inputs.Jump;
                
                characterControllerInputs.WorldMoveVector = math.normalizesafe(characterControllerInputs.WorldMoveVector);
                
            }).Schedule(inputDeps);

            return jobHandle;
        }
    }
}
