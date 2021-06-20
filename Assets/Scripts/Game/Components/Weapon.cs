using Unity.Entities;

namespace Game.Components
{
    [GenerateAuthoringComponent]
    public struct Weapon : IComponentData
    {
        public Entity BulletType;
        public float Damage;
        public float Speed;
        public float Cooldown;
        public double NextShotTime;
    }
}
