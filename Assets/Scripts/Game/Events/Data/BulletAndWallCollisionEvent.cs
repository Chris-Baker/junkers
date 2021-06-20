using Unity.Entities;

namespace Game.Events.Data
{
    [GenerateAuthoringComponent]
    public struct BulletAndWallCollisionEvent : IComponentData
    {
        public Entity Bullet;
        public Entity Wall;
    }
}
