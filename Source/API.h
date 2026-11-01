#pragma once
#include <cstdint>

#if defined(UMT_NATIVE_STATIC)
// Built as a static library linked directly by the host (e.g. Unreal Engine via UBT). No dllexport/keepalive: the host's linker resolves the plain extern "C" symbols out of the archive.
#define MMDPHYSICSAPI extern "C"
#elif defined(_WIN32)
#define MMDPHYSICSAPI extern "C" __declspec(dllexport)
#elif defined(__EMSCRIPTEN__)
// Unity's Web platform links this plugin as a static archive into the IL2CPP/Emscripten player; EMSCRIPTEN_KEEPALIVE keeps the exported symbols from being dead-stripped so the managed [DllImport("__Internal")] entry points resolve.
#include <emscripten.h>
#define MMDPHYSICSAPI extern "C" EMSCRIPTEN_KEEPALIVE
#else
#define MMDPHYSICSAPI extern "C"
#endif

/// <summary>
/// Creates a new native physics context with the given simulation parameters.
/// Counterpart to the managed <c>MMDBulletPhysics(float3, int, int, float)</c> constructor.
/// </summary>
/// <param name="gravity">Gravity vector in Unity space (pointer to 3 floats).</param>
/// <param name="solverIterations">Number of constraint solver iterations per step.</param>
/// <param name="maxSubSteps">Maximum number of fixed sub-steps per simulation step.</param>
/// <param name="fixedTimeStep">Fixed time step used for sub-stepping.</param>
/// <returns>Pointer to the newly created native context.</returns>
MMDPHYSICSAPI MMDBulletPhysics::NativeContext* MMDBulletPhysicsCreate(
    const float* gravity,
    int solverIterations,
    int maxSubSteps,
    float fixedTimeStep);

/// <summary>
/// Destroys the native physics context and frees all owned Bullet objects.
/// Counterpart to the managed <c>Dispose()</c>.
/// </summary>
/// <param name="context">Native context to destroy.</param>
MMDPHYSICSAPI void MMDBulletPhysicsDestroy(MMDBulletPhysics::NativeContext* context);

/// <summary>
/// Overrides the Spring2 context parameters. Structure layout is retained, but the old
/// Bullet 2.75 motor damping values are not behavior-compatible; see NativeConfig.
/// Ground/margin settings take effect when ground/bodies are built, and spring settings
/// take effect when joints are built, so configure the context before world construction.
/// </summary>
/// <param name="context">Native context.</param>
/// <param name="config">Configuration to copy into the context (pointer to a <c>NativeConfig</c>).</param>
MMDPHYSICSAPI void MMDBulletPhysicsSetConfig(
    MMDBulletPhysics::NativeContext* context,
    const MMDBulletPhysics::NativeConfig* config);

/// <summary>
/// Resets the simulation to its initial state using the given random seed.
/// Counterpart to the managed <c>Reset(uint)</c>.
/// </summary>
/// <param name="context">Native context.</param>
/// <param name="seed">Random seed used to reset the deterministic simulation.</param>
MMDPHYSICSAPI void MMDBulletPhysicsReset(MMDBulletPhysics::NativeContext* context, unsigned int seed);

/// <summary>
/// Builds native rigid bodies from the supplied simulation data, clearing any previously
/// built world objects first. Counterpart to the managed <c>BuildRigidBodies(NativeArray&lt;RigidBodySimulationData&gt;)</c>.
/// </summary>
/// <param name="context">Native context.</param>
/// <param name="rigidBodies">Array of rigid-body simulation data to register with the native context.</param>
/// <param name="rigidBodyCount">Number of rigid bodies in the array.</param>
MMDPHYSICSAPI void MMDBulletPhysicsBuildRigidBodies(
    MMDBulletPhysics::NativeContext* context,
    const MMDBulletPhysics::RigidBodySimulationData* rigidBodies,
    int rigidBodyCount);

/// <summary>
/// Builds native joints linking previously built rigid bodies.
/// Counterpart to the managed <c>BuildJoints(NativeJointData[])</c>.
/// </summary>
/// <param name="context">Native context.</param>
/// <param name="joints">Array of joint data to register with the native context.</param>
/// <param name="jointCount">Number of joints in the array.</param>
MMDPHYSICSAPI void MMDBulletPhysicsBuildJoints(
    MMDBulletPhysics::NativeContext* context,
    const MMDBulletPhysics::NativeJointData* joints,
    int jointCount);

/// <summary>
/// Builds an optional static ground plane collider in the native context.
/// Counterpart to the managed <c>BuildGround(bool, byte, short)</c>.
/// </summary>
/// <param name="context">Native context.</param>
/// <param name="enabled">Whether the ground collider is active.</param>
/// <param name="groupIndex">Collision group of the ground collider.</param>
/// <param name="collisionMask">Collision mask of the ground collider.</param>
MMDPHYSICSAPI void MMDBulletPhysicsBuildGround(
    MMDBulletPhysics::NativeContext* context,
    bool enabled,
    uint8_t groupIndex,
    int16_t collisionMask);

/// <summary>
/// Toggles ground collision on the existing native ground collider.
/// Counterpart to the managed <c>SetGroundCollisionEnabled(bool)</c>.
/// </summary>
/// <param name="context">Native context.</param>
/// <param name="enabled">Whether ground collision is active.</param>
MMDPHYSICSAPI void MMDBulletPhysicsSetGroundCollisionEnabled(MMDBulletPhysics::NativeContext* context, bool enabled);

/// <summary>
/// Pushes world transforms onto the given rigid bodies, optionally clearing their velocity.
/// Counterpart to the managed <c>SetRigidBodyTransforms(int, NativeArray&lt;float4x4&gt;, NativeArray&lt;int&gt;, bool)</c>.
/// </summary>
/// <param name="context">Native context.</param>
/// <param name="rigidBodyCount">Number of rigid bodies to update.</param>
/// <param name="transform">World transforms to apply, one column-major 4x4 (16 floats) per rigid body.</param>
/// <param name="rigidBodyIndices">Indices of the rigid bodies being updated.</param>
/// <param name="clearVelocity">True: teleport - also zeroes the linear/angular velocity and resets the interpolation transform. False: leaves the interpolation transform at its previous value so Bullet's saveKinematicState derives kinematic-body velocity from the pushed motion over the next fixed substep (push once per substep for correct velocities).</param>
MMDPHYSICSAPI void MMDBulletPhysicsSetRigidBodyTransforms(
    MMDBulletPhysics::NativeContext* context,
    size_t rigidBodyCount,
    const float* transform,
    const int* rigidBodyIndices,
    bool clearVelocity);

/// <summary>
/// Reads back the current motion-state world transforms of the given rigid bodies.
/// Counterpart to the managed <c>GetRigidBodyMotionTransforms(int, NativeArray&lt;int&gt;, ref NativeArray&lt;float4x4&gt;)</c>.
/// </summary>
/// <param name="context">Native context.</param>
/// <param name="rigidBodyCount">Number of rigid bodies to read.</param>
/// <param name="rigidBodyIndices">Indices of the rigid bodies to read.</param>
/// <param name="transforms">Output buffer receiving one column-major 4x4 (16 floats) per rigid body.</param>
MMDPHYSICSAPI void MMDBulletPhysicsGetRigidBodyMotionTransforms(
    MMDBulletPhysics::NativeContext* context,
    size_t rigidBodyCount,
    const int* rigidBodyIndices,
    float* transforms);

/// <summary>
/// Reads the current simulation world transforms, without motion-state interpolation.
/// Use this snapshot for simulation feedback such as dynamic bone-aligned position correction.
/// GetRigidBodyMotionTransforms retains its interpolated display-pose behavior for existing callers.
/// </summary>
/// <param name="context">Native context.</param>
/// <param name="rigidBodyCount">Number of rigid bodies to read.</param>
/// <param name="rigidBodyIndices">Indices of the rigid bodies to read.</param>
/// <param name="transforms">Output buffer receiving one column-major 4x4 (16 floats) per rigid body.</param>
MMDPHYSICSAPI void MMDBulletPhysicsGetRigidBodyWorldTransforms(
    MMDBulletPhysics::NativeContext* context,
    size_t rigidBodyCount,
    const int* rigidBodyIndices,
    float* transforms);

/// <summary>
/// Shifts a rigid body's position by a world-space delta without resetting velocity.
/// Counterpart to the managed <c>ShiftRigidBodyPosition(int, float3)</c>.
/// </summary>
/// <param name="context">Native context.</param>
/// <param name="rigidBodyIndex">Index of the rigid body to shift.</param>
/// <param name="delta">World-space translation delta (pointer to 3 floats).</param>
MMDPHYSICSAPI void MMDBulletPhysicsShiftRigidBodyPosition(
    MMDBulletPhysics::NativeContext* context,
    int rigidBodyIndex,
    const float* delta);

/// <summary>
/// Advances the simulation by the given elapsed time using fixed sub-steps.
/// Counterpart to the managed <c>StepSimulation(float)</c>.
/// </summary>
/// <param name="context">Native context.</param>
/// <param name="elapsedTime">Elapsed time to simulate, in seconds.</param>
MMDPHYSICSAPI void MMDBulletPhysicsStepSimulation(MMDBulletPhysics::NativeContext* context, float elapsedTime);
