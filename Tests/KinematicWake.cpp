// SPDX-License-Identifier: MIT
// Exercise public setters and observe Bullet activation without expanding the public ABI.
static int GlobalKinematicWakePasses = 0;
#define UMT_TEST_KINEMATIC_WAKE_PASS() (++GlobalKinematicWakePasses)
#include "../Source/NativeContext.cpp"
#include <algorithm>
#include <array>
#include <cstdio>
#include <memory>
#include <stdexcept>

namespace
{
using namespace MMDBulletPhysics;
using World = std::unique_ptr<NativeContext, decltype(&MMDBulletPhysicsDestroy)>;
constexpr float Step = 1.f / 60.f;

void Require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

void Identity(float* transform, float x = 0, float y = 0)
{
    std::fill(transform, transform + 16, 0.f);
    transform[0] = transform[5] = transform[10] = transform[15] = 1.f;
    transform[12] = x; transform[13] = y;
}

World WeldWorld()
{
    const float gravity[3] = {0, -9.8f, 0};
    World context(MMDBulletPhysicsCreate(gravity, 80, 1, Step), &MMDBulletPhysicsDestroy);
    Require(bool(context), "Cannot create the welded follower fixture");
    std::array<RigidBodySimulationData, 6> bodies{};
    for (int index = 0; index < int(bodies.size()); ++index)
    {
        auto& body = bodies[index];
        body.rigidBodyIndex = index; body.relatedBoneIndex = -1;
        body.noRelatedBone = true; body.shape = RigidBodyShape::Sphere;
        body.size[0] = .1f; body.mass = 1.f;
        body.mode = index ? RigidBodyMode::Dynamic : RigidBodyMode::Static;
        body.linearDamping = .1f; body.angularDamping = .3f;
        Identity(body.initialTransform); Identity(body.boneLocalTransform);
        Identity(body.restWorldTransform, float(index < 4 ? index : index == 4 ? 3 : 10), index == 4 ? 2.f : 3.f);
    }
    bodies[4].mode = RigidBodyMode::DynamicBoneAligned;
    bodies[4].noRelatedBone = false; bodies[4].relatedBoneIndex = 0;
    MMDBulletPhysicsBuildRigidBodies(context.get(), bodies.data(), int(bodies.size()));
    std::array<NativeJointData, 4> joints{};
    for (int index = 0; index < int(joints.size()); ++index)
    {
        auto& joint = joints[index];
        joint.type = JointType::Spring6DOF;
        joint.rigidBodyAIndex = index; joint.rigidBodyBIndex = index + 1;
        Identity(joint.frameInA, index < 3 ? 1.f : 0.f, index < 3 ? 0.f : -1.f);
        Identity(joint.frameInB);
    }
    joints[3].rotationLimitMin[2] = -.6f;
    joints[3].rotationLimitMax[2] = .6f;
    joints[3].springRotation[2] = .5f;
    MMDBulletPhysicsBuildJoints(context.get(), joints.data(), int(joints.size()));
    Require(context->anchoredWelds.size() == 3, "The test must reach three actual anchored followers");
    Require(!context->rigidBodies[4]->isKinematicObject(), "A bound Mode2 pendant must remain dynamic");
    return context;
}

void LegacyFollowerStep(NativeContext* context)
{
    for (int index = 0; index < context->anchoredWelds.size(); ++index)
    {
        const auto& weld = context->anchoredWelds[index];
        float target[16];
        WriteManagedTransform(context->rigidBodies[weld.parent]->getWorldTransform() * weld.local, target);
        MMDBulletPhysicsSetRigidBodyTransforms(context, 1, target, &weld.child, false);
    }
    context->world->stepSimulation(Step, context->maxSubSteps, context->fixedTimeStep);
}

void RequireEqualState(const NativeContext* current, const NativeContext* legacy)
{
    for (int index = 0; index < current->rigidBodies.size(); ++index)
    {
        const auto* a = current->rigidBodies[index]; const auto* b = legacy->rigidBodies[index];
        btTransform motionA, motionB;
        a->getMotionState()->getWorldTransform(motionA); b->getMotionState()->getWorldTransform(motionB);
        Require(a->getWorldTransform() == b->getWorldTransform() && motionA == motionB
            && a->getInterpolationWorldTransform() == b->getInterpolationWorldTransform(), "Follower batching changed a native transform");
        Require(a->getLinearVelocity() == b->getLinearVelocity() && a->getAngularVelocity() == b->getAngularVelocity()
            && a->getInterpolationLinearVelocity() == b->getInterpolationLinearVelocity()
            && a->getInterpolationAngularVelocity() == b->getInterpolationAngularVelocity(), "Follower batching changed a native velocity");
        Require(a->getActivationState() == b->getActivationState() && a->getDeactivationTime() == b->getDeactivationTime(),
            "Follower batching changed activation or its timer");
    }
}

void WeldWakeBatch()
{
    auto current = WeldWorld(); auto legacy = WeldWorld();
    for (int frame = 0; frame < 360; ++frame)
    {
        const bool moving = frame < 60 || frame >= 300;
        btTransform target = btTransform::getIdentity();
        target.setOrigin(btVector3(moving ? btScalar(.2) * btSin(btScalar(frame + 1) * btScalar(.04)) : btScalar(0), 3, 0));
        target.setRotation(btQuaternion(btVector3(0, 0, 1), moving ? btScalar(.1) * btSin(btScalar(frame + 1) * btScalar(.03)) : btScalar(0)));
        // Directly move the root: only the internal follower path may wake the separate sleeping body.
        for (auto* context : {current.get(), legacy.get()})
        {
            context->rigidBodies[0]->setWorldTransform(target);
            context->rigidBodies[0]->getMotionState()->setWorldTransform(target);
            context->rigidBodies[0]->forceActivationState(DISABLE_DEACTIVATION);
            if (frame == 0) context->rigidBodies[5]->forceActivationState(ISLAND_SLEEPING);
        }
        GlobalKinematicWakePasses = 0;
        MMDBulletPhysicsStepSimulation(current.get(), Step);
        const int currentPasses = GlobalKinematicWakePasses;
        GlobalKinematicWakePasses = 0;
        LegacyFollowerStep(legacy.get());
        const int legacyPasses = GlobalKinematicWakePasses;
        RequireEqualState(current.get(), legacy.get());
        if (currentPasses != (legacyPasses ? 1 : 0))
            std::fprintf(stderr, "frame=%d batched_passes=%d legacy_passes=%d\n", frame, currentPasses, legacyPasses);
        Require(currentPasses == (legacyPasses ? 1 : 0), "Internal welded followers must share one global wake pass");
        if (frame == 0) Require(current->rigidBodies[5]->isActive(), "Moving internal followers did not wake the separate dynamic body");
    }
    std::puts("PASS three welded followers share one wake pass; double-precision state equals the legacy loop");
}
}

int main()
{
    using namespace MMDBulletPhysics;
    const float gravity[3] = {0, 0, 0};
    auto* context = MMDBulletPhysicsCreate(gravity, 20, 1, 1.f / 60.f);
    try
    {
        std::array<RigidBodySimulationData, 2> bodies{};
        for (int index = 0; index < 2; ++index)
        {
            auto& body = bodies[index];
            body.rigidBodyIndex = index;
            body.noRelatedBone = true;
            body.shape = RigidBodyShape::Sphere;
            body.size[0] = .1f;
            body.mass = 1;
            body.mode = index ? RigidBodyMode::Dynamic : RigidBodyMode::Static;
            for (int element = 0; element < 16; ++element)
                body.restWorldTransform[element] = element % 5 == 0 ? 1.f : 0.f;
            body.restWorldTransform[12] = float(index);
        }
        MMDBulletPhysicsBuildRigidBodies(context, bodies.data(), int(bodies.size()));
        for (int frame = 0; frame < 240; ++frame)
            MMDBulletPhysicsStepSimulation(context, 1.f / 60.f);
        if (context->rigidBodies[1]->isActive()) throw std::runtime_error("Resting body did not sleep");

        const int anchor = 0;
        float target[16];
        std::copy(bodies[0].restWorldTransform, bodies[0].restWorldTransform + 16, target);
        for (int frame = 0; frame < 8; ++frame)
        {
            const float angle = frame % 2 ? 0 : std::numeric_limits<float>::epsilon() * .25f;
            target[1] = angle; target[4] = -angle;
            MMDBulletPhysicsSetRigidBodyTransforms(context, 1, target, &anchor, false);
            MMDBulletPhysicsStepSimulation(context, 1.f / 60.f);
        }
        if (context->rigidBodies[1]->isActive()) throw std::runtime_error("Float ABI roundoff falsely woke the resting model");

        target[12] = .00001f;
        GlobalKinematicWakePasses = 0;
        MMDBulletPhysicsSetRigidBodyTransforms(context, 1, target, &anchor, false);
        if (!context->rigidBodies[1]->isActive()) throw std::runtime_error("Real small movement failed to wake the model");
        if (GlobalKinematicWakePasses != 1) throw std::runtime_error("The public setter must wake immediately exactly once");
        MMDBulletPhysicsDestroy(context);
        context = nullptr;
        WeldWakeBatch();
        std::puts("PASS float roundoff sleeps; real 10-micrometre movement wakes");
        return 0;
    }
    catch (const std::exception& error)
    {
        MMDBulletPhysicsDestroy(context);
        std::fprintf(stderr, "FAIL %s\n", error.what());
        return 1;
    }
}
