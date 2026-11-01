// SPDX-License-Identifier: MIT
#include "NativeContext.h"
#include "API.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

static void Identity(float* m, float x, float y, float z)
{
    for (int i = 0; i < 16; ++i) m[i] = 0;
    m[0] = m[5] = m[10] = m[15] = 1;
    m[12] = x; m[13] = y; m[14] = z;
}

static MMDBulletPhysics::NativeContext* Create()
{
    const float gravity[3] = {0, -9.8f, 0};
    auto* world = MMDBulletPhysicsCreate(gravity, 20, 1, 1.f / 60.f);
    if (!world) throw std::runtime_error("Cannot create physics context");
    MMDBulletPhysics::RigidBodySimulationData body{};
    body.shape = MMDBulletPhysics::RigidBodyShape::Sphere;
    body.mode = MMDBulletPhysics::RigidBodyMode::DynamicBoneAligned;
    body.size[0] = .1f; body.mass = 1; body.collisionGroupMask = 0;
    Identity(body.restWorldTransform, 0, 1, 0);
    MMDBulletPhysicsBuildRigidBodies(world, &body, 1);
    MMDBulletPhysicsReset(world, 0);
    return world;
}

int main()
{
    auto* currentWorld = Create();
    auto* legacyWorld = Create();
    const int index = 0;
    double maxCurrentError = 0, maxLegacyError = 0, snapshotDifference = 0;
    for (int frame = 0; frame < 120; ++frame)
    {
        const float desired[3] = {float(.15 * std::sin(frame * .1)), 1, 0};
        for (int legacy = 0; legacy < 2; ++legacy)
        {
            auto* world = legacy ? legacyWorld : currentWorld;
            MMDBulletPhysicsStepSimulation(world, 1.f / 60.f);
            float snapshot[16], current[16];
            MMDBulletPhysicsGetRigidBodyWorldTransforms(world, 1, &index, current);
            if (legacy) MMDBulletPhysicsGetRigidBodyMotionTransforms(world, 1, &index, snapshot);
            else for (int i = 0; i < 16; ++i) snapshot[i] = current[i];
            for (int axis = 0; axis < 3; ++axis)
                snapshotDifference = std::max(snapshotDifference, std::abs(double(snapshot[12 + axis] - current[12 + axis])));
            const float delta[3] = {snapshot[12] - desired[0], snapshot[13] - desired[1], snapshot[14] - desired[2]};
            MMDBulletPhysicsShiftRigidBodyPosition(world, index, delta);
            MMDBulletPhysicsGetRigidBodyWorldTransforms(world, 1, &index, current);
            for (int axis = 0; axis < 3; ++axis)
            {
                const double error = std::abs(double(current[12 + axis] - desired[axis]));
                if (legacy) maxLegacyError = std::max(maxLegacyError, error);
                else maxCurrentError = std::max(maxCurrentError, error);
            }
        }
    }
    MMDBulletPhysicsDestroy(currentWorld); MMDBulletPhysicsDestroy(legacyWorld);
    if (!(maxCurrentError < 1.e-6 && maxLegacyError > .001 && snapshotDifference > .001))
    {
        std::fprintf(stderr, "FAIL mode2 current_error=%.9f legacy_error=%.9f snapshot_difference=%.9f\n", maxCurrentError, maxLegacyError, snapshotDifference);
        return 1;
    }
    std::printf("PASS mode2 current_world_error=%.9f legacy_motion_feedback_error=%.9f snapshot_difference=%.9f\n", maxCurrentError, maxLegacyError, snapshotDifference);
    return 0;
}
