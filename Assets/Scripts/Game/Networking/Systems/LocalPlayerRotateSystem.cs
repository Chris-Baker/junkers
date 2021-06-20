using Game.Tags;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.InputSystem;

namespace Game.Systems
{
    public class LocalPlayerRotateSystem : ComponentSystem
    {
        private Camera _camera;
        private float3 _mousePosition;

        protected override void OnCreate()
        {
            base.OnCreate();
            RequireSingletonForUpdate<DisabledSystem>();
            _camera = Camera.main;
            _mousePosition = new float3();
        }

        protected override void OnUpdate()
        {
            return;
            if (_camera == null)
            {
                Debug.Log("Camera is null");
                _camera = Camera.main;
                return;
            }
            
            // get the mouse position
            Mouse mouse = Mouse.current;
            _mousePosition.x = mouse.position.x.ReadValue();
            _mousePosition.y = mouse.position.y.ReadValue();
            Debug.Log("Mouse position: " + _mousePosition);
            
            Entities.ForEach((ref Rotation rotation, ref Translation translation) =>
            {
                float3 playerPosition = _camera.WorldToScreenPoint(translation.Value);
                float3 targetForward = new float3
                {
                    x = _mousePosition.x - playerPosition.x, z = _mousePosition.y - playerPosition.y
                };
                quaternion target = quaternion.LookRotationSafe(targetForward, math.up());
                rotation.Value = math.slerp(rotation.Value, target, 0.2f);
            });
        }
    }
}
