using Unity.Entities;
using Unity.Transforms;

namespace Game.Events.Data
{
    [GenerateAuthoringComponent]
    public struct SpawnEvent : IComponentData
    {
        public Translation Translation;
    }
}