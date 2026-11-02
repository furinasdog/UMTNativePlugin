// SPDX-License-Identifier: MIT
// Exercise public setters and observe Bullet activation without expanding the public ABI.
#include "../Source/NativeContext.cpp"
#include <algorithm>
#include <array>
#include <cstdio>
#include <stdexcept>

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
        MMDBulletPhysicsSetRigidBodyTransforms(context, 1, target, &anchor, false);
        if (!context->rigidBodies[1]->isActive()) throw std::runtime_error("Real small movement failed to wake the model");
        MMDBulletPhysicsDestroy(context);
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
