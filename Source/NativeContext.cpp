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
#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h>
#include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btMinMax.h>
#include <LinearMath/btTransform.h>

#include <cstdlib>
#include <cmath>
#include <limits>

namespace MMDBulletPhysics
{
    constexpr btScalar k_MMDUnitToUnityUnit = btScalar(0.08);
    constexpr btScalar k_CCDSweptSphereScale = btScalar(0.8);
    constexpr btScalar k_MinCcdSweptSphereRadius = btScalar(0.0001);
    // Match the mature MMD Tools setup in world metres. Bullet keeps sphere/capsule
    // radius semantics; this is not a uniform outward expansion of every shape.
    constexpr btScalar k_ConvexMargin = btScalar(1.e-6);

    // Spring2 uses physical viscous damping, unlike Bullet 2.75's spring motor coefficient.
    inline NativeConfig DefaultConfig()
    {
        NativeConfig config;
        config.groundNormal[0] = 0.0f;
        config.groundNormal[1] = 1.0f;
        config.groundNormal[2] = 0.0f;
        config.groundConstant = 0.0f;
        config.mmdUnitToUnityUnit = float(k_MMDUnitToUnityUnit);
        config.convexMargin = float(k_ConvexMargin);
        config.springTranslationDamping = 0.5f;
        config.springRotationDamping = 0.5f;
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
        struct AnchoredWeld { int parent, child; btTransform local; };
        btAlignedObjectArray<AnchoredWeld> anchoredWelds;
        btAlignedObjectArray<bool> canAnchorWeld;

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
    // Regularize redundant small-body constraint loops without changing their authored limits.
    world->getSolverInfo().m_globalCfm = .001;
    // Use Bullet's standard two-direction, velocity-independent friction basis.
    world->getSolverInfo().m_solverMode |= SOLVER_USE_2_FRICTION_DIRECTIONS
        | SOLVER_ENABLE_FRICTION_DIRECTION_CACHING
        | SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION;
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
    anchoredWelds.clear();
    canAnchorWeld.clear();

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

template<class Constraint>
void ConfigureGenericConstraint(Constraint* constraint, const MMDBulletPhysics::NativeJointData& joint)
{
    constraint->setLinearLowerLimit(ToBulletVector(joint.translationLimitMin));
    constraint->setLinearUpperLimit(ToBulletVector(joint.translationLimitMax));
    constraint->setAngularLowerLimit(ToBulletVector(joint.rotationLimitMin));
    constraint->setAngularUpperLimit(ToBulletVector(joint.rotationLimitMax));
}

void SetSpring(btGeneric6DofSpring2Constraint* constraint, int index, float stiffness, float damping)
{
    constraint->setStiffness(index, stiffness);
    constraint->setDamping(index, damping);
    // A zero-stiffness axis still applies relative-velocity damping, as in MMD Tools.
    constraint->enableSpring(index, true);
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
        // Keep Bullet's normal sleeping thresholds; static contacts can then settle as whole islands.
        if (!IsSimulated(rigidBody.mode))
        {
            body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
            body->forceActivationState(ISLAND_SLEEPING);
        }
        ConfigureCCD(body, rigidBody);

        context->rigidBodies.push_back(body);
        // Bound Mode2 bodies retain their host-side animated-position policy.
        context->canAnchorWeld.push_back(rigidBody.mode == MMDBulletPhysics::RigidBodyMode::Dynamic
            || (rigidBody.mode == MMDBulletPhysics::RigidBodyMode::DynamicBoneAligned && rigidBody.noRelatedBone));
        context->world->addRigidBody(body, static_cast<short>(1 << rigidBody.groupIndex), rigidBody.collisionGroupMask);
    }
}

MMDPHYSICSAPI void MMDBulletPhysicsBuildJoints(
    MMDBulletPhysics::NativeContext* context,
    const MMDBulletPhysics::NativeJointData* joints,
    int jointCount)
{
    // A zero-DOF weld rooted at a kinematic body has an exact prescribed transform.
    // Reduce these anchored chains before solving the remaining dynamic system.
    for (int pass = 0; pass < context->rigidBodies.size(); ++pass)
    {
        bool changed = false;
        for (int i = 0; i < jointCount; ++i)
        {
            const auto& joint = joints[i];
            bool locked = true;
            for (int axis = 0; axis < 3; ++axis)
                locked &= joint.translationLimitMin[axis] == 0 && joint.translationLimitMax[axis] == 0
                    && joint.rotationLimitMin[axis] == 0 && joint.rotationLimitMax[axis] == 0;
            if (!locked) continue;
            const int a = joint.rigidBodyAIndex, b = joint.rigidBodyBIndex;
            int parent = -1, child = -1;
            btTransform local;
            if (context->rigidBodies[a]->isKinematicObject() && !context->rigidBodies[b]->isStaticOrKinematicObject())
            { parent = a; child = b; local = ToBulletTransform(joint.frameInA) * ToBulletTransform(joint.frameInB).inverse(); }
            else if (context->rigidBodies[b]->isKinematicObject() && !context->rigidBodies[a]->isStaticOrKinematicObject())
            { parent = b; child = a; local = ToBulletTransform(joint.frameInB) * ToBulletTransform(joint.frameInA).inverse(); }
            if (parent < 0 || !context->canAnchorWeld[child]) continue;
            auto* body = context->rigidBodies[child];
            body->setMassProps(0, btVector3(0, 0, 0));
            body->updateInertiaTensor();
            body->setCollisionFlags((body->getCollisionFlags() & ~btCollisionObject::CF_STATIC_OBJECT) | btCollisionObject::CF_KINEMATIC_OBJECT);
            body->forceActivationState(ISLAND_SLEEPING);
            context->anchoredWelds.push_back({parent, child, local});
            changed = true;
        }
        if (!changed) break;
    }
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
            btGeneric6DofSpring2Constraint* springConstraint = new btGeneric6DofSpring2Constraint(bodyA, bodyB, frameInA, frameInB, RO_XYZ);
            ConfigureGenericConstraint(springConstraint, joint);
            SetSpring(springConstraint, 0, joint.springTranslation[0], context->config.springTranslationDamping);
            SetSpring(springConstraint, 1, joint.springTranslation[1], context->config.springTranslationDamping);
            SetSpring(springConstraint, 2, joint.springTranslation[2], context->config.springTranslationDamping);
            SetSpring(springConstraint, 3, joint.springRotation[0], context->config.springRotationDamping);
            SetSpring(springConstraint, 4, joint.springRotation[1], context->config.springRotationDamping);
            SetSpring(springConstraint, 5, joint.springRotation[2], context->config.springRotationDamping);
            springConstraint->setEquilibriumPoint();
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

        // Suppress only this linked pair; all other PMX group/mask filtering remains active.
        context->world->addConstraint(constraint, true);
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
                context->world->addConstraint(reinforcement, true);
                context->constraints.push_back(reinforcement);
            }
        }
    }

    // Let authored high angular damping on constrained secondary motion suppress residual micro-motion by sleeping.
    // This sleep approximation retains source contacts and joint limits, including their remaining error.
    for (int index = 0; index < context->rigidBodies.size(); ++index)
    {
        btRigidBody* body = context->rigidBodies[index];
        if (!body->isStaticOrKinematicObject() && body->getNumConstraintRefs() > 0 &&
            body->getAngularDamping() >= btScalar(.9f))
            body->setSleepingThresholds(body->getLinearSleepingThreshold(), btScalar(3));
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

namespace
{
bool SetBodyTransforms(
    MMDBulletPhysics::NativeContext* context,
    size_t rigidBodyCount,
    const float* transforms,
    const int* rigidBodyIndices,
    bool clearVelocity)
{
    bool movedKinematic = false;
    for (size_t i = 0; i < rigidBodyCount; ++i)
    {
        int rigidBodyIndex = rigidBodyIndices[i];
        btRigidBody* body = context->rigidBodies[rigidBodyIndex];
        btTransform bodyTransform = ToBulletTransform(transforms + i * 16);
        if (body->isKinematicObject())
        {
            const btTransform& previous = body->getWorldTransform();
            // Float ABI roundoff must not wake every dynamic island after animation stops.
            const btScalar precision = std::numeric_limits<float>::epsilon();
            bool moved = (previous.getOrigin() - bodyTransform.getOrigin()).length2() > precision * precision;
            for (int axis = 0; axis < 3; ++axis)
                moved |= (previous.getBasis().getRow(axis) - bodyTransform.getBasis().getRow(axis)).length2() > precision * precision;
            body->forceActivationState(moved ? DISABLE_DEACTIVATION : ISLAND_SLEEPING);
            movedKinematic |= moved;
            if (!moved)
            {
                // A stopped kinematic collider must neither retain its last velocity nor wake resting contacts.
                body->setLinearVelocity(btVector3(0, 0, 0));
                body->setAngularVelocity(btVector3(0, 0, 0));
                body->setInterpolationWorldTransform(bodyTransform);
                body->setInterpolationLinearVelocity(btVector3(0, 0, 0));
                body->setInterpolationAngularVelocity(btVector3(0, 0, 0));
            }
        }
        else body->activate(true);
        body->setWorldTransform(bodyTransform);
        body->updateInertiaTensor();
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
        // Moving kinematic targets retain the previous interpolation transform for Bullet's velocity calculation.
    }
    return movedKinematic;
}

void WakeDynamicBodies(MMDBulletPhysics::NativeContext* context)
{
#ifdef UMT_TEST_KINEMATIC_WAKE_PASS
    UMT_TEST_KINEMATIC_WAKE_PASS();
#endif
    // Constraint links do not themselves wake sleeping bodies when a kinematic endpoint moves.
    for (int i = 0; i < context->rigidBodies.size(); ++i)
        if (!context->rigidBodies[i]->isStaticOrKinematicObject()) context->rigidBodies[i]->activate(true);
}
}

MMDPHYSICSAPI void MMDBulletPhysicsSetRigidBodyTransforms(
    MMDBulletPhysics::NativeContext* context,
    size_t rigidBodyCount,
    const float* transforms,
    const int* rigidBodyIndices,
    bool clearVelocity)
{
    if (SetBodyTransforms(context, rigidBodyCount, transforms, rigidBodyIndices, clearVelocity))
        WakeDynamicBodies(context);
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
    bool movedKinematic = false;
    for (int i = 0; i < context->anchoredWelds.size(); ++i)
    {
        const auto& weld = context->anchoredWelds[i];
        float target[16];
        WriteManagedTransform(context->rigidBodies[weld.parent]->getWorldTransform() * weld.local, target);
        movedKinematic |= SetBodyTransforms(context, 1, target, &weld.child, false);
    }
    // Preserve chained transforms in order; wake once before solving instead of once per follower.
    if (movedKinematic) WakeDynamicBodies(context);
    context->world->stepSimulation(elapsedTime, context->maxSubSteps, context->fixedTimeStep);
}
