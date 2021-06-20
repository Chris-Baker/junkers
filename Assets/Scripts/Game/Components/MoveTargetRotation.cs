using Unity.Entities;
using Unity.Mathematics;

namespace Game.Components
{
    [GenerateAuthoringComponent]
    public struct MoveTargetRotation : IComponentData
    {
        public quaternion Forward;
    }
}