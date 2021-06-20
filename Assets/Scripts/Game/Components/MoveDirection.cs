using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Game.Components
{
    [GenerateAuthoringComponent]
    public struct MoveDirection : IComponentData
    {
        [HideInInspector] public float3 Direction;
    }
}
