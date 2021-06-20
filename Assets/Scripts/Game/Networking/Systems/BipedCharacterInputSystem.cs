using System;
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
    [UpdateBefore(typeof(BipedCharacterMovementSystem))]
    public class BipedCharacterInputSystem : ComponentSystem
    {
        protected override void OnCreate() {
            RequireSingletonForUpdate<BipedCharacterInput>();
        }
        
        protected override void OnUpdate()
        {
            // Get our input component
            BipedCharacterInput inputs = GetSingleton<BipedCharacterInput>();
            
            // get the keyboard
            Keyboard keyboard = Keyboard.current;
            inputs.Up = keyboard[Key.W].isPressed;
            inputs.Down = keyboard[Key.S].isPressed;
            inputs.Left = keyboard[Key.A].isPressed;
            inputs.Right = keyboard[Key.D].isPressed;
            inputs.Jump = keyboard[Key.Space].isPressed;
            
            // get the mouse
            Mouse mouse = Mouse.current;
            float3 mousePosition = new float3()
            {
                x = mouse.position.x.ReadValue(),
                z = mouse.position.y.ReadValue()
            };

            // do we need to move this into a job to get the player position?
            // do we want to just send the look at (mouse position) instead?
            // we need a camera to do world to screenspace conversion
            float3 targetForward = new float3
            {
                x = mousePosition.x - playerPosition.x, z = mousePosition.z - playerPosition.y
            };
            
            inputs.Pitch = x;
            inputs.Yaw = y;

            SetSingleton(inputs);
        }
    }
}
