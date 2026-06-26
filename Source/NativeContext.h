#pragma once
#include <cstdint>

namespace MMDBulletPhysics
{
    /// <summary>
    /// Opaque native physics context owning the Bullet world and all rigid bodies,
    /// constraints, shapes, and motion states. Created by <c>MMDBulletPhysicsCreate</c>
    /// and destroyed by <c>MMDBulletPhysicsDestroy</c>; mirrors the managed
    /// <c>UMT.MMDBulletPhysics</c> wrapper's native handle.
    /// </summary>
    struct NativeContext;

    /// <summary>Collider shape kind. Layout-compatible with managed <c>UMT.PMXRigidBody.Shape</c>.</summary>
    enum class RigidBodyShape : std::uint8_t
    {
        /// <summary>Sphere collider (uses <c>size[0]</c> as radius).</summary>
        Sphere = 0,
        /// <summary>Box collider (uses <c>size</c> as half-extents).</summary>
        Box = 1,
        /// <summary>Capsule collider (uses <c>size[0]</c> as radius, <c>size[1]</c> as height).</summary>
        Capsule = 2
    };

    /// <summary>Physics mode. Layout-compatible with managed <c>UMT.PMXRigidBody.Mode</c>.</summary>
    enum class RigidBodyMode : std::uint8_t
    {
        /// <summary>Bone-following kinematic body, not simulated.</summary>
        Static = 0,
        /// <summary>Fully dynamic, simulated body.</summary>
        Dynamic = 1,
        /// <summary>Dynamic body whose position is aligned to its related bone.</summary>
        DynamicBoneAligned = 2
    };

    /// <summary>Joint constraint kind. Layout-compatible with managed <c>UMT.PMXJoint.Type</c>.</summary>
    enum class JointType : std::uint8_t
    {
        /// <summary>6-DOF constraint with per-axis springs.</summary>
        Spring6DOF = 0,
        /// <summary>6-DOF constraint without springs.</summary>
        Generic6DOF = 1
    };

    /// <summary>
    /// Blittable rigid-body simulation data passed from managed code.
    /// Field order is layout-critical and must stay byte-for-byte in sync with the managed
    /// <c>UMT.MMDRigidBody.RigidBodySimulationData</c> struct; do not reorder.
    /// </summary>
    struct RigidBodySimulationData
    {
        /// <summary>Index of this rigid body within the PMX rigid-body list.</summary>
        int rigidBodyIndex;
        /// <summary>Index of the related bone, or negative when there is none.</summary>
        int relatedBoneIndex;
        /// <summary>Collision group index this rigid body belongs to.</summary>
        uint8_t groupIndex;
        /// <summary>Bitmask of collision groups this rigid body does not collide with.</summary>
        int16_t collisionGroupMask;
        /// <summary>Collider shape (sphere, box, or capsule).</summary>
        RigidBodyShape shape;
        /// <summary>Collider dimensions in Unity units.</summary>
        float size[3];
        /// <summary>Rigid-body position in Unity space.</summary>
        float position[3];
        /// <summary>Rigid-body rotation (Euler) in Unity space.</summary>
        float rotation[3];
        /// <summary>Rigid-body mass.</summary>
        float mass;
        /// <summary>Linear velocity damping factor.</summary>
        float linearDamping;
        /// <summary>Angular velocity damping factor.</summary>
        float angularDamping;
        /// <summary>Bounciness (restitution) coefficient.</summary>
        float restitution;
        /// <summary>Surface friction coefficient.</summary>
        float friction;
        /// <summary>Physics mode: bone-following (kinematic), dynamic, or dynamic bone-aligned.</summary>
        RigidBodyMode mode;
        /// <summary>Rigid-body transform relative to its related bone's model-space origin (column-major 4x4).</summary>
        float initialTransform[16];
        /// <summary>Whether this rigid body has no related bone (managed counterpart is <c>hasRelatedBone</c>, inverted).</summary>
        bool noRelatedBone;
        /// <summary>Inverse of <c>initialTransform</c>, mapping the rigid body back to bone-local space (column-major 4x4).</summary>
        float boneLocalTransform[16];
        /// <summary>Related bone's initial model-space position.</summary>
        float boneModelPosition[3];
        /// <summary>Rigid-body rest world transform used as the joint reference frame (column-major 4x4).</summary>
        float restWorldTransform[16];
        /// <summary>MMD transform level of the related bone, or negative when there is none.</summary>
        int boneTransformLevel;
    };

    /// <summary>
    /// Blittable joint data passed from managed code.
    /// Field order is layout-critical and must stay byte-for-byte in sync with the managed
    /// <c>UMT.MMDBulletPhysics.NativeJointData</c> struct; do not reorder.
    /// </summary>
    struct NativeJointData
    {
        /// <summary>Joint type (Spring6DOF or Generic6DOF).</summary>
        JointType type;
        /// <summary>Index of the first connected rigid body.</summary>
        int rigidBodyAIndex;
        /// <summary>Index of the second connected rigid body.</summary>
        int rigidBodyBIndex;
        /// <summary>Joint frame relative to the first rigid body (column-major 4x4).</summary>
        float frameInA[16];
        /// <summary>Joint frame relative to the second rigid body (column-major 4x4).</summary>
        float frameInB[16];
        /// <summary>Minimum translation limit per axis.</summary>
        float translationLimitMin[3];
        /// <summary>Maximum translation limit per axis.</summary>
        float translationLimitMax[3];
        /// <summary>Minimum rotation limit per axis.</summary>
        float rotationLimitMin[3];
        /// <summary>Maximum rotation limit per axis.</summary>
        float rotationLimitMax[3];
        /// <summary>Translation spring stiffness per axis.</summary>
        float springTranslation[3];
        /// <summary>Rotation spring stiffness per axis.</summary>
        float springRotation[3];
    };
}
