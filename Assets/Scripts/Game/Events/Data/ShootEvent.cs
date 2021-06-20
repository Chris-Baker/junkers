using Unity.Entities;
using Unity.Transforms;

namespace Game.Events.Data
{
    [GenerateAuthoringComponent]
    public struct ShootEvent : IComponentData
    {
        public Translation FromPosition;
        public Translation ToPosition;
        public Entity Prefab;
    }
}
