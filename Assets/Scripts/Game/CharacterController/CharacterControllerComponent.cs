using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Game.CharacterController
{
    [Serializable]
    public struct CharacterControllerComponent : IComponentData
    {
        [Header("Movement")]
        public float RotationSharpness;
        public float GroundMaxSpeed;
        public float GroundedMovementSharpness;
        public float AirAcceleration;
        public float AirMaxSpeed;
        public float AirDrag;
        public float JumpSpeed;
        public float3 Gravity;

        [Header("Step Handling")]
        public bool StepHandling;
        public float MaxStepHeight;
        public float ExtraStepChecksDistance;

        [Header("Slope Changes")]
        public bool PreventGroundingWhenMovingTowardsNoGrounding;
        public bool HasMaxDownwardSlopeChangeAngle;
        [Range(0f, 180f)]
        public float MaxDownwardSlopeChangeAngle;

        [Header("Misc")]
        public bool ConstrainVelocityToGroundPlane;

        public static CharacterControllerComponent GetDefault()
        {
            return new CharacterControllerComponent
            {
                RotationSharpness = 25f,
                GroundMaxSpeed = 10f,
                GroundedMovementSharpness = 15f,
                AirAcceleration = 50f,
                AirMaxSpeed = 10f,
                AirDrag = 0f,
                JumpSpeed = 10f,
                Gravity = math.up() * -30f,

                StepHandling = false,
                MaxStepHeight = 0.5f,
                ExtraStepChecksDistance = 0.1f,

                PreventGroundingWhenMovingTowardsNoGrounding = true,
                HasMaxDownwardSlopeChangeAngle = false,
                MaxDownwardSlopeChangeAngle = 90f,

                ConstrainVelocityToGroundPlane = true,
            };
        }
    }

    [Serializable]
    public struct CharacterControllerInputs : IComponentData
    {
        public float3 WorldMoveVector;
        public float3 WorldRotationVector;
        public bool JumpRequested;
    }
}