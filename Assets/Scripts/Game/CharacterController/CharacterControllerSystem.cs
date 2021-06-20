using Rival;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Physics;
using Unity.Physics.Systems;
using Unity.Transforms;

namespace Game.CharacterController
{
    [UpdateInGroup(typeof(KinematicCharacterUpdateGroup))]
    public class CharacterControllerSystem : SystemBase
    {
        public BuildPhysicsWorld BuildPhysicsWorldSystem;
        public EndFramePhysicsSystem EndFramePhysicsSystem;
        public EntityQuery CharacterQuery;

        [BurstCompile]
        public struct CharacterControllerJob : IJobEntityBatchWithIndex
        {
            public float DeltaTime;
            [ReadOnly]
            public CollisionWorld CollisionWorld;

            [ReadOnly]
            public ComponentDataFromEntity<PhysicsVelocity> PhysicsVelocityFromEntity;
            [ReadOnly]
            public ComponentDataFromEntity<PhysicsMass> PhysicsMassFromEntity;
            [NativeDisableParallelForRestriction] // we need to write to our own characterBody, but we might also need to read from other characterBodies during the update (for dynamics handling)
            public ComponentDataFromEntity<KinematicCharacterBody> CharacterBodyFromEntity;
            [ReadOnly]
            public ComponentDataFromEntity<TrackedTransform> TrackedTransformFromEntity;

            [ReadOnly]
            public EntityTypeHandle EntityType;
            public ComponentTypeHandle<Translation> TranslationType;
            public ComponentTypeHandle<Rotation> RotationType;
            public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;
            public BufferTypeHandle<KinematicCharacterHit> CharacterHitsBufferType;
            public BufferTypeHandle<KinematicVelocityProjectionHit> VelocityProjectionHitsBufferType;
            public BufferTypeHandle<KinematicCharacterDeferredImpulse> CharacterDeferredImpulsesBufferType;
            public BufferTypeHandle<StatefulKinematicCharacterHit> StatefulCharacterHitsBufferType;

            public ComponentTypeHandle<CharacterControllerComponent> CharacterControllerType;
            public ComponentTypeHandle<CharacterControllerInputs> CharacterControllerInputsType;

            [NativeDisableContainerSafetyRestriction]
            public NativeList<int> TmpRigidbodyIndexesProcessed;
            [NativeDisableContainerSafetyRestriction]
            public NativeList<Unity.Physics.RaycastHit> TmpRaycastHits;
            [NativeDisableContainerSafetyRestriction]
            public NativeList<ColliderCastHit> TmpColliderCastHits;
            [NativeDisableContainerSafetyRestriction]
            public NativeList<DistanceHit> TmpDistanceHits;

            public void Execute(ArchetypeChunk chunk, int batchIndex, int indexOfFirstEntityInQuery)
            {
                NativeArray<Entity> chunkEntities = chunk.GetNativeArray(EntityType);
                NativeArray<Translation> chunkTranslations = chunk.GetNativeArray(TranslationType);
                NativeArray<Rotation> chunkRotations = chunk.GetNativeArray(RotationType);
                NativeArray<PhysicsCollider> chunkPhysicsColliders = chunk.GetNativeArray(PhysicsColliderType);
                BufferAccessor<KinematicCharacterHit> chunkCharacterHitBuffers = chunk.GetBufferAccessor(CharacterHitsBufferType);
                BufferAccessor<KinematicVelocityProjectionHit> chunkVelocityProjectionHitBuffers = chunk.GetBufferAccessor(VelocityProjectionHitsBufferType);
                BufferAccessor<KinematicCharacterDeferredImpulse> chunkCharacterDeferredImpulsesBuffers = chunk.GetBufferAccessor(CharacterDeferredImpulsesBufferType);
                BufferAccessor<StatefulKinematicCharacterHit> chunkStatefulCharacterHitsBuffers = chunk.GetBufferAccessor(StatefulCharacterHitsBufferType);
                NativeArray<CharacterControllerComponent> chunkCharacterControllers = chunk.GetNativeArray(CharacterControllerType);
                NativeArray<CharacterControllerInputs> chunkCharacterControllerInputs = chunk.GetNativeArray(CharacterControllerInputsType);

                // Initialize the Temp collections
                if (!TmpRigidbodyIndexesProcessed.IsCreated)
                {
                    TmpRigidbodyIndexesProcessed = new NativeList<int>(24, Allocator.Temp);
                }
                if (!TmpRaycastHits.IsCreated)
                {
                    TmpRaycastHits = new NativeList<Unity.Physics.RaycastHit>(24, Allocator.Temp);
                }
                if (!TmpColliderCastHits.IsCreated)
                {
                    TmpColliderCastHits = new NativeList<ColliderCastHit>(24, Allocator.Temp);
                }
                if (!TmpDistanceHits.IsCreated)
                {
                    TmpDistanceHits = new NativeList<DistanceHit>(24, Allocator.Temp);
                }

                // Assign the global data of the processor
                CharacterControllerProcessor processor = default;
                processor.DeltaTime = DeltaTime;
                processor.CollisionWorld = CollisionWorld;
                processor.CharacterBodyFromEntity = CharacterBodyFromEntity;
                processor.PhysicsMassFromEntity = PhysicsMassFromEntity;
                processor.PhysicsVelocityFromEntity = PhysicsVelocityFromEntity;
                processor.TrackedTransformFromEntity = TrackedTransformFromEntity;
                processor.TmpRigidbodyIndexesProcessed = TmpRigidbodyIndexesProcessed;
                processor.TmpRaycastHits = TmpRaycastHits;
                processor.TmpColliderCastHits = TmpColliderCastHits;
                processor.TmpDistanceHits = TmpDistanceHits;

                // Iterate on individual characters
                for (int i = 0; i < chunk.Count; i++)
                {
                    Entity entity = chunkEntities[i];

                    // Assign the per-character data of the processor
                    processor.Entity = entity;
                    processor.Translation = chunkTranslations[i].Value;
                    processor.Rotation = chunkRotations[i].Value;
                    processor.PhysicsCollider = chunkPhysicsColliders[i];
                    processor.CharacterBody = CharacterBodyFromEntity[entity];
                    processor.CharacterHitsBuffer = chunkCharacterHitBuffers[i];
                    processor.CharacterDeferredImpulsesBuffer = chunkCharacterDeferredImpulsesBuffers[i];
                    processor.VelocityProjectionHitsBuffer = chunkVelocityProjectionHitBuffers[i];
                    processor.StatefulCharacterHitsBuffer = chunkStatefulCharacterHitsBuffers[i];
                    processor.CharacterController = chunkCharacterControllers[i];
                    processor.CharacterControllerInputs = chunkCharacterControllerInputs[i];

                    // Update character
                    processor.OnUpdate();

                    // Write back updated data
                    // The core character update loop only writes to Translation, Rotation, KinematicCharacterBody, and the various character DynamicBuffers. 
                    // You must remember to write back any extra data you modify in your own code
                    chunkTranslations[i] = new Translation { Value = processor.Translation };
                    chunkRotations[i] = new Rotation { Value = processor.Rotation };
                    CharacterBodyFromEntity[entity] = processor.CharacterBody;
                    chunkPhysicsColliders[i] = processor.PhysicsCollider; // safe to remove if not needed. This would be needed if you resize the character collider, for example
                    chunkCharacterControllers[i] = processor.CharacterController; // safe to remove if not needed. This would be needed if you changed data in your own character component
                    chunkCharacterControllerInputs[i] = processor.CharacterControllerInputs; // safe to remove if not needed. This would be needed if you changed data in your own character component
                }
            }
        }

        protected override void OnCreate()
        {
            BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();
            EndFramePhysicsSystem = World.GetOrCreateSystem<EndFramePhysicsSystem>();

            CharacterQuery = GetEntityQuery(new EntityQueryDesc
            {
                All = MiscUtilities.CombineArrays(
                    KinematicCharacterUtilities.GetCoreCharacterComponentTypes(),
                    new ComponentType[]
                    {
                        typeof(CharacterControllerComponent),
                        typeof(CharacterControllerInputs),
                    }),
            });

            RequireForUpdate(CharacterQuery);
        }

        protected override void OnUpdate()
        {
            Dependency = JobHandle.CombineDependencies(EndFramePhysicsSystem.GetOutputDependency(), Dependency);

            Dependency = new CharacterControllerJob
            {
                DeltaTime = Time.DeltaTime,
                CollisionWorld = BuildPhysicsWorldSystem.PhysicsWorld.CollisionWorld,

                PhysicsVelocityFromEntity = GetComponentDataFromEntity<PhysicsVelocity>(true),
                PhysicsMassFromEntity = GetComponentDataFromEntity<PhysicsMass>(true),
                CharacterBodyFromEntity = GetComponentDataFromEntity<KinematicCharacterBody>(false),
                TrackedTransformFromEntity = GetComponentDataFromEntity<TrackedTransform>(true),

                EntityType = GetEntityTypeHandle(),
                TranslationType = GetComponentTypeHandle<Translation>(false),
                RotationType = GetComponentTypeHandle<Rotation>(false),
                PhysicsColliderType = GetComponentTypeHandle<PhysicsCollider>(false),
                CharacterHitsBufferType = GetBufferTypeHandle<KinematicCharacterHit>(false),
                VelocityProjectionHitsBufferType = GetBufferTypeHandle<KinematicVelocityProjectionHit>(false),
                CharacterDeferredImpulsesBufferType = GetBufferTypeHandle<KinematicCharacterDeferredImpulse>(false),
                StatefulCharacterHitsBufferType = GetBufferTypeHandle<StatefulKinematicCharacterHit>(false),

                CharacterControllerType = GetComponentTypeHandle<CharacterControllerComponent>(false),
                CharacterControllerInputsType = GetComponentTypeHandle<CharacterControllerInputs>(false),
            }.ScheduleParallel(CharacterQuery, 1, Dependency);

            Dependency = KinematicCharacterUtilities.ScheduleDeferredImpulsesJob(this, CharacterQuery, Dependency);

            BuildPhysicsWorldSystem.AddInputDependencyToComplete(Dependency);
        }
    }
}
