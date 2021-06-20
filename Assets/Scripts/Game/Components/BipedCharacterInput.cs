using Unity.Entities;
using UnityEngine;

namespace Game.Components
{
    [GenerateAuthoringComponent]
    public struct BipedCharacterInput : IComponentData
    {
        [HideInInspector] public bool Up, Down, Left, Right, Shoot, Use, Sprint, Jump;
        [HideInInspector] public float Yaw, Pitch;
    }
}