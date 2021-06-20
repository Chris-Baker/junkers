using Unity.Entities;

namespace Game.Components
{
    public enum CollisionGroup : uint
    {
        Player,
        PlayerBullet,
        Enemy,
        EnemyBullet,
        Wall,
        DeckLift
    }
    
    [GenerateAuthoringComponent]
    public struct PhysicsLayer : IComponentData
    {
        public CollisionGroup Value;
    }
}