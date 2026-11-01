#include "NativeContext.h"
#include "API.h"

#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h>
#include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btMinMax.h>
#include <LinearMath/btTransform.h>

#include <cstdlib>

namespace MMDBulletPhysics
{
    constexpr btScalar k_MMDUnitToUnityUnit = btScalar(0.08);
    constexpr btScalar k_CCDSweptSphereScale = btScalar(0.8);
    constexpr btScalar k_MinCcdSweptSphereRadius = btScalar(0.0001);
    // Bullet's default convex collision margin (0.04) is set for MMD-native-scale colliders. This pipeline scales everything to Unity units (x0.08), so the unscaled 0.04 margin hugely inflates the small colliders (e.g. a 0.1-radius capsule gains a 0.04 skin = +40%), causing spurious deep penetration at rest. Scale the margin into Unity space so it stays proportionally tiny.
    constexpr btScalar k_BulletDefaultConvexMargin = btScalar(0.04);
    constexpr btScalar k_ScaledBulletDefaultConvexMargin = k_BulletDefaultConvexMargin * k_MMDUnitToUnityUnit;

    // Config seeded into every context at creation so an untouched context reproduces the historical hardcoded behavior exactly.
    inline NativeConfig DefaultConfig()
    {
        NativeConfig config;
        config.groundNormal[0] = 0.0f;
        config.groundNormal[1] = 1.0f;
        config.groundNormal[2] = 0.0f;
        config.groundConstant = 0.0f;
        config.mmdUnitToUnityUnit = float(k_MMDUnitToUnityUnit);
        config.convexMargin = float(k_ScaledBulletDefaultConvexMargin);
        config.springTranslationDamping = 1.0f;
        config.springRotationDamping = 1.0f;
        config.lockedTranslationReinforce = 0;
        return config;
    }

    struct NativeContext
    {
        btDefaultCollisionConfiguration* collisionConfiguration;
        btCollisionDispatcher* dispatcher;
        btDbvtBroadphase* broadphase;
        btSequentialImpulseConstraintSolver* solver;
        btDiscreteDynamicsWorld* world;
        btRigidBody* groundRigidBody;
        int maxSubSteps;
        float fixedTimeStep;
        NativeConfig config;
        btAlignedObjectArray<btCollisionShape*> shapes;
        btAlignedObjectArray<btDefaultMotionState*> motionStates;
        btAlignedObjectArray<btRigidBody*> rigidBodies;
        btAlignedObjectArray<btTypedConstraint*> constraints;

        NativeContext(const float* gravity, int solverIterations, int maxSubStepsValue, float fixedTimeStepValue);

        ~NativeContext();

        void ClearWorldObjects();
    };
}

btVector3 ToBulletVector(const float value[3])
{
    return btVector3(value[0], value[1], value[2]);
}

btTransform ToBulletTransform(const float value[16])
{
    btTransform result;
    result.getBasis().setValue(
        value[0], value[4], value[8],
        value[1], value[5], value[9],
        value[2], value[6], value[10]);
    result.getOrigin().setValue(value[12], value[13], value[14]);
    return result;
}

void WriteManagedTransform(const btTransform& value, float* transform)
{
    transform[0] = (float)value.getBasis().getRow(0).getX();
    transform[4] = (float)value.getBasis().getRow(0).getY();
    transform[8] = (float)value.getBasis().getRow(0).getZ();
    transform[12] = (float)value.getOrigin().getX();
    transform[1] = (float)value.getBasis().getRow(1).getX();
    transform[5] = (float)value.getBasis().getRow(1).getY();
    transform[9] = (float)value.getBasis().getRow(1).getZ();
    transform[13] = (float)value.getOrigin().getY();
    transform[2] = (float)value.getBasis().getRow(2).getX();
    transform[6] = (float)value.getBasis().getRow(2).getY();
    transform[10] = (float)value.getBasis().getRow(2).getZ();
    transform[14] = (float)value.getOrigin().getZ();
    transform[3] = 0.0f;
    transform[7] = 0.0f;
    transform[11] = 0.0f;
    transform[15] = 1.0f;
}

bool IsSimulated(MMDBulletPhysics::RigidBodyMode mode)
{
    return mode == MMDBulletPhysics::RigidBodyMode::Dynamic ||
        mode == MMDBulletPhysics::RigidBodyMode::DynamicBoneAligned;
}

btScalar ComputeCCDSweptSphereRadius(const MMDBulletPhysics::RigidBodySimulationData& rigidBody)
{
    switch (rigidBody.shape)
    {
    case MMDBulletPhysics::RigidBodyShape::Sphere:
    case MMDBulletPhysics::RigidBodyShape::Capsule:
        return rigidBody.size[0] * MMDBulletPhysics::k_CCDSweptSphereScale;
    case MMDBulletPhysics::RigidBodyShape::Box:
        return btMin(rigidBody.size[0], btMin(rigidBody.size[1], rigidBody.size[2])) * MMDBulletPhysics::k_CCDSweptSphereScale;
    default:
        std::abort();
    }
}

void ConfigureCCD(btRigidBody* body, const MMDBulletPhysics::RigidBodySimulationData& rigidBody)
{
    if (!IsSimulated(rigidBody.mode))
    {
        return;
    }

    btScalar sweptSphereRadius = ComputeCCDSweptSphereRadius(rigidBody);
    if (sweptSphereRadius <= MMDBulletPhysics::k_MinCcdSweptSphereRadius)
    {
        return;
    }

    body->setCcdSweptSphereRadius(sweptSphereRadius);
    body->setCcdMotionThreshold(sweptSphereRadius);
}

// Shapes are built in the natural MMD/Unity body-local frame (Y-up): the capsule long axis runs along local Y and `size` is used directly as box half-extents (radius, MMD-Y extent, MMD-Z extent order). This keeps the native library coordinate-system-agnostic and byte-for-byte compatible with the Unity build; the host is responsible for orienting the rigid-body frame so this natural shape lands correctly. The Unreal port bakes a 90-degree shape-alignment rotation into the rigid-body transforms rather than rotating the shape here (see Plugins/UnrealMMDTools MMDPhysicsManager ShapeAlignmentRotation).
btCollisionShape* CreateShape(const MMDBulletPhysics::RigidBodySimulationData& rigidBody, btScalar convexMargin)
{
    switch (rigidBody.shape)
    {
    case MMDBulletPhysics::RigidBodyShape::Sphere:
        // A sphere's collision margin is its radius, so it needs no separate margin override.
        return new btSphereShape(rigidBody.size[0]);
    case MMDBulletPhysics::RigidBodyShape::Box:
    {
        btBoxShape* shape = new btBoxShape(ToBulletVector(rigidBody.size));
        shape->setMargin(convexMargin);
        return shape;
    }
    case MMDBulletPhysics::RigidBodyShape::Capsule:
    {
        btCapsuleShape* shape = new btCapsuleShape(rigidBody.size[0], rigidBody.size[1]);
        shape->setMargin(convexMargin);
        return shape;
    }
    default:
        std::abort();
    }
}

MMDBulletPhysics::NativeContext::NativeContext(const float* gravity, int solverIterations, int maxSubStepsValue, float fixedTimeStepValue)
    : collisionConfiguration(new btDefaultCollisionConfiguration()),
    dispatcher(new btCollisionDispatcher(collisionConfiguration)),
    broadphase(new btDbvtBroadphase()),
    solver(new btSequentialImpulseConstraintSolver()),
    world(new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration)),
    groundRigidBody(nullptr),
    maxSubSteps(maxSubStepsValue),
    fixedTimeStep(fixedTimeStepValue),
    config(DefaultConfig())
{
    world->setGravity(ToBulletVector(gravity));
    world->getSolverInfo().m_numIterations = solverIterations;
}

MMDBulletPhysics::NativeContext::~NativeContext()
{
    ClearWorldObjects();
    delete world;
    delete solver;
    delete broadphase;
    delete dispatcher;
    delete collisionConfiguration;
}

void MMDBulletPhysics::NativeContext::ClearWorldObjects()
{
    for (int i = constraints.size() - 1; i >= 0; --i)
    {
        world->removeConstraint(constraints[i]);
        delete constraints[i];
    }

    constraints.clear();

    for (int i = rigidBodies.size() - 1; i >= 0; --i)
    {
        world->removeRigidBody(rigidBodies[i]);
        delete rigidBodies[i];
    }

    rigidBodies.clear();
    groundRigidBody = nullptr;

    for (int i = motionStates.size() - 1; i >= 0; --i)
    {
        delete motionStates[i];
    }

    motionStates.clear();

    for (int i = shapes.size() - 1; i >= 0; --i)
    {
        delete shapes[i];
    }

    shapes.clear();
}

void ConfigureGenericConstraint(btGeneric6DofConstraint* constraint, const MMDBulletPhysics::NativeJointData& joint)
{
    constraint->setLinearLowerLimit(ToBulletVector(joint.translationLimitMin));
    constraint->setLinearUpperLimit(ToBulletVector(joint.translationLimitMax));
    constraint->setAngularLowerLimit(ToBulletVector(joint.rotationLimitMin));
    constraint->setAngularUpperLimit(ToBulletVector(joint.rotationLimitMax));
}

void SetSpring(btGeneric6DofSpringConstraint* constraint, int index, float stiffness, float damping)
{
    constraint->setStiffness(index, stiffness);
    constraint->setDamping(index, damping);
    constraint->enableSpring(index, stiffness != 0.0f);
}

MMDPHYSICSAPI MMDBulletPhysics::NativeContext* MMDBulletPhysicsCreate(
    const float* gravity,
    int solverIterations,
    int maxSubSteps,
    float fixedTimeStep)
{
    return new MMDBulletPhysics::NativeContext(gravity, solverIterations, maxSubSteps, fixedTimeStep);
}

MMDPHYSICSAPI void MMDBulletPhysicsDestroy(MMDBulletPhysics::NativeContext* context)
{
    delete context;
}

MMDPHYSICSAPI void MMDBulletPhysicsSetConfig(
    MMDBulletPhysics::NativeContext* context,
    const MMDBulletPhysics::NativeConfig* config)
{
    context->config = *config;
}

MMDPHYSICSAPI void MMDBulletPhysicsReset(MMDBulletPhysics::NativeContext* context, unsigned int seed)
{
    context->world->clearForces();
    context->broadphase->resetPool(context->dispatcher);
    context->solver->reset();
    context->solver->setRandSeed(seed);
}

MMDPHYSICSAPI void MMDBulletPhysicsBuildRigidBodies(
    MMDBulletPhysics::NativeContext* context,
    const MMDBulletPhysics::RigidBodySimulationData* rigidBodies,
    int rigidBodyCount)
{
    context->ClearWorldObjects();
    context->rigidBodies.reserve(rigidBodyCount);

    for (int i = 0; i < rigidBodyCount; ++i)
    {
        const MMDBulletPhysics::RigidBodySimulationData& rigidBody = rigidBodies[i];
        btCollisionShape* shape = CreateShape(rigidBody, btScalar(context->config.convexMargin));
        context->shapes.push_back(shape);

        float mass = IsSimulated(rigidBody.mode) ? rigidBody.mass : 0.0f;
        btVector3 inertia(0.0f, 0.0f, 0.0f);
        if (mass > 0.0f)
        {
            shape->calculateLocalInertia(mass, inertia);
        }

        btTransform restWorldTransform = ToBulletTransform(rigidBody.restWorldTransform);
        btDefaultMotionState* motionState = new btDefaultMotionState(restWorldTransform);
        context->motionStates.push_back(motionState);

        btRigidBody::btRigidBodyConstructionInfo info(mass, motionState, shape, inertia);
        info.m_additionalDamping = true;
        info.m_friction = rigidBody.friction;
        info.m_linearDamping = rigidBody.linearDamping;
        info.m_angularDamping = rigidBody.angularDamping;
        info.m_restitution = rigidBody.restitution;

        btRigidBody* body = new btRigidBody(info);
        body->setSleepingThresholds(0.0f, 0.0f);
        body->setActivationState(DISABLE_DEACTIVATION);
        if (!IsSimulated(rigidBody.mode))
        {
            body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
        }
        ConfigureCCD(body, rigidBody);

        context->rigidBodies.push_back(body);
        context->world->addRigidBody(body, static_cast<short>(1 << rigidBody.groupIndex), rigidBody.collisionGroupMask);
    }
}

MMDPHYSICSAPI void MMDBulletPhysicsBuildJoints(
    MMDBulletPhysics::NativeContext* context,
    const MMDBulletPhysics::NativeJointData* joints,
    int jointCount)
{
    for (int i = 0; i < jointCount; ++i)
    {
        const MMDBulletPhysics::NativeJointData& joint = joints[i];
        btRigidBody& bodyA = *context->rigidBodies[joint.rigidBodyAIndex];
        btRigidBody& bodyB = *context->rigidBodies[joint.rigidBodyBIndex];
        btTransform frameInA = ToBulletTransform(joint.frameInA);
        btTransform frameInB = ToBulletTransform(joint.frameInB);

        btTypedConstraint* constraint = nullptr;
        if (joint.type == MMDBulletPhysics::JointType::Spring6DOF)
        {
            btGeneric6DofSpringConstraint* springConstraint = new btGeneric6DofSpringConstraint(bodyA, bodyB, frameInA, frameInB, true);
            ConfigureGenericConstraint(springConstraint, joint);
            SetSpring(springConstraint, 0, joint.springTranslation[0], context->config.springTranslationDamping);
            SetSpring(springConstraint, 1, joint.springTranslation[1], context->config.springTranslationDamping);
            SetSpring(springConstraint, 2, joint.springTranslation[2], context->config.springTranslationDamping);
            SetSpring(springConstraint, 3, joint.springRotation[0], context->config.springRotationDamping);
            SetSpring(springConstraint, 4, joint.springRotation[1], context->config.springRotationDamping);
            SetSpring(springConstraint, 5, joint.springRotation[2], context->config.springRotationDamping);
            constraint = springConstraint;
        }
        else if (joint.type == MMDBulletPhysics::JointType::Generic6DOF)
        {
            btGeneric6DofConstraint* genericConstraint = new btGeneric6DofConstraint(bodyA, bodyB, frameInA, frameInB, true);
            ConfigureGenericConstraint(genericConstraint, joint);
            constraint = genericConstraint;
        }
        else
        {
            std::abort();
        }

        context->world->addConstraint(constraint, false);
        context->constraints.push_back(constraint);

        // Reinforce fully-locked translation (min == max == 0) with redundant point-to-point copies of the same equality: each copy adds one extra Gauss-Seidel solve of those rows per solver iteration, so spring-less locked chains survive fast motion at a low iteration count. The converged solution is identical to the locked 6DOF rows; only convergence improves. See NativeConfig::lockedTranslationReinforce.
        const bool translationLocked =
            joint.translationLimitMin[0] == 0.0f && joint.translationLimitMax[0] == 0.0f &&
            joint.translationLimitMin[1] == 0.0f && joint.translationLimitMax[1] == 0.0f &&
            joint.translationLimitMin[2] == 0.0f && joint.translationLimitMax[2] == 0.0f;
        if (translationLocked)
        {
            for (int copy = 0; copy < context->config.lockedTranslationReinforce; ++copy)
            {
                btPoint2PointConstraint* reinforcement = new btPoint2PointConstraint(bodyA, bodyB, frameInA.getOrigin(), frameInB.getOrigin());
                context->world->addConstraint(reinforcement, false);
                context->constraints.push_back(reinforcement);
            }
        }
    }
}

MMDPHYSICSAPI void MMDBulletPhysicsBuildGround(
    MMDBulletPhysics::NativeContext* context,
    bool enabled,
    uint8_t groupIndex,
    int16_t collisionMask)
{
    btCollisionShape* shape = new btStaticPlaneShape(ToBulletVector(context->config.groundNormal), btScalar(context->config.groundConstant));
    context->shapes.push_back(shape);
    btDefaultMotionState* motionState = new btDefaultMotionState(btTransform::getIdentity());
    context->motionStates.push_back(motionState);
    btRigidBody::btRigidBodyConstructionInfo info(0.0f, motionState, shape, btVector3(0.0f, 0.0f, 0.0f));
    context->groundRigidBody = new btRigidBody(info);
    context->rigidBodies.push_back(context->groundRigidBody);
    MMDBulletPhysicsSetGroundCollisionEnabled(context, enabled);
    context->world->addRigidBody(context->groundRigidBody, static_cast<short>(1 << groupIndex), collisionMask);
}

MMDPHYSICSAPI void MMDBulletPhysicsSetGroundCollisionEnabled(MMDBulletPhysics::NativeContext* context, bool enabled)
{
    int flags = context->groundRigidBody->getCollisionFlags();
    context->groundRigidBody->setCollisionFlags(enabled
        ? flags & ~btCollisionObject::CF_NO_CONTACT_RESPONSE
        : flags | btCollisionObject::CF_NO_CONTACT_RESPONSE);
}

MMDPHYSICSAPI void MMDBulletPhysicsSetRigidBodyTransforms(
    MMDBulletPhysics::NativeContext* context,
    size_t rigidBodyCount,
    const float* transforms,
    const int* rigidBodyIndices,
    bool clearVelocity)
{
    for (size_t i = 0; i < rigidBodyCount; ++i)
    {
        int rigidBodyIndex = rigidBodyIndices[i];
        btRigidBody* body = context->rigidBodies[rigidBodyIndex];
        btTransform bodyTransform = ToBulletTransform(transforms + i * 16);
        body->setWorldTransform(bodyTransform);
        body->getMotionState()->setWorldTransform(bodyTransform);
        if (clearVelocity)
        {
            body->setInterpolationWorldTransform(bodyTransform);
            body->setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
            body->setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
            body->setInterpolationLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
            body->setInterpolationAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
            body->clearForces();
        }
        // With clearVelocity false the interpolation transform is intentionally left at its previous value: btDiscreteDynamicsWorld::saveKinematicState derives kinematic body velocity from (interpolationWorldTransform -> worldTransform) over the fixed substep, so a host pushing kinematic targets once per substep gives the constraint solver correct kinematic velocities instead of pure position error - this is how continuously-animated kinematic bodies behave in a live Bullet scene.
    }
}

MMDPHYSICSAPI void MMDBulletPhysicsGetRigidBodyMotionTransform(
    MMDBulletPhysics::NativeContext* context,
    int rigidBodyIndex,
    float* transform)
{
    btTransform bodyTransform;
    context->rigidBodies[rigidBodyIndex]->getMotionState()->getWorldTransform(bodyTransform);
    WriteManagedTransform(bodyTransform, transform);
}

MMDPHYSICSAPI void MMDBulletPhysicsGetRigidBodyMotionTransforms(
    MMDBulletPhysics::NativeContext *context,
    size_t rigidBodyCount,
    const int *rigidBodyIndices,
    float *transforms)
{
    for (size_t i = 0; i < rigidBodyCount; ++i)
    {
        int rigidBodyIndex = rigidBodyIndices[i];
        btTransform bodyTransform;
        context->rigidBodies[rigidBodyIndex]->getMotionState()->getWorldTransform(bodyTransform);
        WriteManagedTransform(bodyTransform, transforms + i * 16);
    }
}

MMDPHYSICSAPI void MMDBulletPhysicsGetRigidBodyWorldTransforms(
    MMDBulletPhysics::NativeContext* context,
    size_t rigidBodyCount,
    const int* rigidBodyIndices,
    float* transforms)
{
    for (size_t i = 0; i < rigidBodyCount; ++i)
    {
        const btTransform& bodyTransform = context->rigidBodies[rigidBodyIndices[i]]->getWorldTransform();
        WriteManagedTransform(bodyTransform, transforms + i * 16);
    }
}

MMDPHYSICSAPI void MMDBulletPhysicsShiftRigidBodyPosition(
    MMDBulletPhysics::NativeContext* context,
    int rigidBodyIndex,
    const float* delta)
{
    btRigidBody* body = context->rigidBodies[rigidBodyIndex];
    btTransform bodyTransform = body->getWorldTransform();
    bodyTransform.getOrigin() -= ToBulletVector(delta);

    body->setWorldTransform(bodyTransform);
    body->getMotionState()->setWorldTransform(bodyTransform);
    body->setInterpolationWorldTransform(bodyTransform);
}

MMDPHYSICSAPI void MMDBulletPhysicsStepSimulation(MMDBulletPhysics::NativeContext* context, float elapsedTime)
{
    context->world->stepSimulation(elapsedTime, context->maxSubSteps, context->fixedTimeStep);
}
