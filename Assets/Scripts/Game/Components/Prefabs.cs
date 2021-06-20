using Unity.Entities;

namespace Game.Components
{
    [GenerateAuthoringComponent]
    public struct Prefabs : IComponentData
    {
        public Entity PlayerPrefab;
    }
}
