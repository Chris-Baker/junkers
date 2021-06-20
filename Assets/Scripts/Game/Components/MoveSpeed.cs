using Unity.Entities;

namespace Game.Components
{
    [GenerateAuthoringComponent]
    public struct MoveSpeed : IComponentData
    {
        public float Acceleration;
        public float Current;
        public float Max;
        public float Over;
    }
}
