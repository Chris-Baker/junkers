using Unity.Entities;

namespace Game.Events.Data
{
    [GenerateAuthoringComponent]
    public struct PlayerAndWallCollisionEvent : IComponentData
    {
        public Entity Player;
        public Entity Wall;
    }
}
