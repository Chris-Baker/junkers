using Rival;
using Unity.Entities;
using Unity.Physics.Authoring;
using UnityEngine;

namespace Game.CharacterController
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(PhysicsShapeAuthoring))]
    [UpdateAfter(typeof(EndColliderConversionSystem))]
    public class CharacterControllerAuthoring : MonoBehaviour, IConvertGameObjectToEntity
    {
        public AuthoringKinematicCharacterBody CharacterBody = AuthoringKinematicCharacterBody.GetDefault();
        public CharacterControllerComponent CharacterController = CharacterControllerComponent.GetDefault();

        public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
        {
            KinematicCharacterUtilities.HandleConversionForCharacter(dstManager, entity, gameObject, CharacterBody);

            dstManager.AddComponentData(entity, CharacterController);
            dstManager.AddComponentData(entity, new CharacterControllerInputs());
        }
    }
}
