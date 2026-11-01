// SPDX-License-Identifier: MIT
// Behavioral regressions through the public C ABI; NativeContext remains opaque.
#include <cstddef>
#include "NativeContext.h"
#include "API.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace
{
using namespace MMDBulletPhysics;
constexpr float Step = 1.f / 60.f;
constexpr double Pi = 3.14159265358979323846;
using Matrix = std::array<float, 16>;
using World = std::unique_ptr<NativeContext, decltype(&MMDBulletPhysicsDestroy)>;

void Require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

void Identity(float* matrix, float x = 0, float y = 0, float z = 0)
{
    std::fill(matrix, matrix + 16, 0.f);
    matrix[0] = matrix[5] = matrix[10] = matrix[15] = 1;
    matrix[12] = x; matrix[13] = y; matrix[14] = z;
}

World Create(const std::array<float, 3>& gravity)
{
    World world(MMDBulletPhysicsCreate(gravity.data(), 20, 1, Step), &MMDBulletPhysicsDestroy);
    Require(bool(world), "Cannot create native world");
    NativeConfig config{};
    config.groundNormal[1] = 1;
    config.mmdUnitToUnityUnit = .1f;
    config.convexMargin = 1.e-6f;
    config.springTranslationDamping = .5f;
    config.springRotationDamping = .5f;
    config.lockedTranslationReinforce = 0;
    MMDBulletPhysicsSetConfig(world.get(), &config);
    return world;
}

RigidBodySimulationData Body(int index, float x, float y, float z, bool kinematic = false)
{
    RigidBodySimulationData body{};
    body.rigidBodyIndex = index;
    body.relatedBoneIndex = -1;
    body.boneTransformLevel = -1;
    body.noRelatedBone = true;
    body.shape = RigidBodyShape::Sphere;
    body.mode = kinematic ? RigidBodyMode::Static : RigidBodyMode::Dynamic;
    body.size[0] = .1f;
    body.mass = 1;
    // No body damping, restitution, friction, ground, or collisions unless a test opts in.
    Identity(body.initialTransform);
    Identity(body.boneLocalTransform);
    Identity(body.restWorldTransform, x, y, z);
    return body;
}

NativeJointData FreeJoint(JointType type, int a, int b)
{
    NativeJointData joint{};
    joint.type = type;
    joint.rigidBodyAIndex = a;
    joint.rigidBodyBIndex = b;
    Identity(joint.frameInA);
    Identity(joint.frameInB);
    for (int axis = 0; axis < 3; ++axis)
    {
        // Bullet's documented free-axis representation: lower > upper.
        joint.translationLimitMin[axis] = joint.rotationLimitMin[axis] = 1;
        joint.translationLimitMax[axis] = joint.rotationLimitMax[axis] = -1;
    }
    return joint;
}

Matrix Read(NativeContext* world, int index)
{
    Matrix result{};
    MMDBulletPhysicsGetRigidBodyWorldTransforms(world, 1, &index, result.data());
    for (float value : result) Require(std::isfinite(value), "Non-finite public world snapshot");
    return result;
}

double Distance(const Matrix& a, const Matrix& b)
{
    double squared = 0;
    for (int axis = 12; axis < 15; ++axis)
    {
        const double delta = double(a[axis]) - b[axis];
        squared += delta * delta;
    }
    return std::sqrt(squared);
}

std::array<double, 4> Quaternion(const Matrix& matrix)
{
    // Extract and normalize in double precision; snapshots are float ABI matrices.
    std::array<double, 4> q{};
    const double trace = double(matrix[0]) + matrix[5] + matrix[10];
    if (trace > 0)
    {
        const double s = std::sqrt(trace + 1) * 2;
        q = { (matrix[6] - double(matrix[9])) / s,
              (matrix[8] - double(matrix[2])) / s,
              (matrix[1] - double(matrix[4])) / s, s / 4 };
    }
    else
    {
        const int i = matrix[0] > matrix[5] ? (matrix[0] > matrix[10] ? 0 : 2)
                                           : (matrix[5] > matrix[10] ? 1 : 2);
        const int j = (i + 1) % 3, k = (i + 2) % 3;
        const double s = std::sqrt(std::max(0., 1. + matrix[i * 5] - matrix[j * 5] - matrix[k * 5])) * 2;
        Require(s > 1.e-12, "Invalid snapshot rotation");
        q[i] = s / 4;
        q[j] = (double(matrix[j * 4 + i]) + matrix[i * 4 + j]) / s;
        q[k] = (double(matrix[k * 4 + i]) + matrix[i * 4 + k]) / s;
        q[3] = (double(matrix[j * 4 + k]) - matrix[k * 4 + j]) / s;
    }
    double length = 0;
    for (double value : q) length += value * value;
    Require(length > 1.e-12 && std::isfinite(length), "Invalid quaternion length");
    for (double& value : q) value /= std::sqrt(length);
    return q;
}

double AngularSpeed(const Matrix& a, const Matrix& b)
{
    const auto qa = Quaternion(a), qb = Quaternion(b);
    double dot = 0;
    for (int i = 0; i < 4; ++i) dot += qa[i] * qb[i];
    return 2 * std::acos(std::min(1., std::abs(dot))) * 180 / Pi / double(Step);
}

double P95(std::vector<double> values)
{
    Require(!values.empty(), "No tail samples");
    std::sort(values.begin(), values.end());
    return values[size_t(double(values.size() - 1) * .95)];
}

void ZeroStiffnessDamping()
{
    // A normal-gravity magnitude escapes Bullet's shared low-speed additional damping.
    // Both fresh worlds otherwise use identical bodies and integration settings.
    auto free = Create({ 9.8f, 0, 0 });
    auto damped = Create({ 9.8f, 0, 0 });
    const auto freeBody = Body(0, 0, 0, 0);
    MMDBulletPhysicsBuildRigidBodies(free.get(), &freeBody, 1);
    const std::array<RigidBodySimulationData, 2> bodies = { Body(0, 0, 0, 0, true), Body(1, 0, 0, 0) };
    MMDBulletPhysicsBuildRigidBodies(damped.get(), bodies.data(), int(bodies.size()));
    const auto joint = FreeJoint(JointType::Spring6DOF, 0, 1);
    MMDBulletPhysicsBuildJoints(damped.get(), &joint, 1);
    Matrix previousFree{}, previousDamped{};
    for (int step = 0; step < 240; ++step)
    {
        previousFree = Read(free.get(), 0);
        previousDamped = Read(damped.get(), 1);
        MMDBulletPhysicsStepSimulation(free.get(), Step);
        MMDBulletPhysicsStepSimulation(damped.get(), Step);
    }
    const auto finalFree = Read(free.get(), 0), finalDamped = Read(damped.get(), 1);
    const double freeSpeed = (finalFree[12] - double(previousFree[12])) / Step;
    const double dampedSpeed = (finalDamped[12] - double(previousDamped[12])) / Step;
    std::printf("zero_k free_distance_m=%.9g damped_distance_m=%.9g free_speed_m_s=%.9g damped_speed_m_s=%.9g\n",
                double(finalFree[12]), double(finalDamped[12]), freeSpeed, dampedSpeed);
    Require(finalFree[12] > 4 && freeSpeed > 2, "Gravity control did not accelerate");
    Require(finalDamped[12] > .5f && dampedSpeed > .1, "Damped body was frozen instead of simulated");
    Require(finalDamped[12] < finalFree[12] * .8 && dampedSpeed < freeSpeed * .75,
            "Zero-stiffness spring axis lost its velocity damping");
}

void AngularLimitPendulum()
{
    auto world = Create({ 0, -9.8f, 0 });
    std::array<RigidBodySimulationData, 2> bodies = { Body(0, 0, 0, 0, true), Body(1, 0, .4f, .2f) };
    // Offset center of mass gives a persistent positive X torque at the permitted upper limit.
    // A solid box has inertia consistent with this lever arm, avoiding a near point-mass
    // pendulum whose pivot/angle rows are poorly conditioned at only 20 iterations.
    bodies[1].shape = RigidBodyShape::Box;
    bodies[1].size[0] = .1f;
    bodies[1].size[1] = .4f;
    bodies[1].size[2] = .2f;
    MMDBulletPhysicsBuildRigidBodies(world.get(), bodies.data(), int(bodies.size()));
    auto joint = FreeJoint(JointType::Spring6DOF, 0, 1);
    Identity(joint.frameInB, 0, -.4f, -.2f);
    for (int axis = 0; axis < 3; ++axis)
    {
        joint.translationLimitMin[axis] = joint.translationLimitMax[axis] = 0;
        joint.rotationLimitMin[axis] = joint.rotationLimitMax[axis] = 0;
    }
    const float limit = float(20 * Pi / 180);
    joint.rotationLimitMin[0] = -limit;
    joint.rotationLimitMax[0] = limit;
    MMDBulletPhysicsBuildJoints(world.get(), &joint, 1);
    const auto initial = Read(world.get(), 1);
    std::vector<Matrix> tail;
    double maxTravel = 0;
    for (int step = 1; step <= 3600; ++step)
    {
        MMDBulletPhysicsStepSimulation(world.get(), Step);
        const auto current = Read(world.get(), 1);
        maxTravel = std::max(maxTravel, Distance(initial, current));
        if (step > 3480) tail.push_back(current);
    }
    const auto& final = tail.back();
    const double angleX = std::atan2(-double(final[9]), double(final[10])) * 180 / Pi;
    double positionRange = 0;
    double minAngle = 180, maxAngle = -180;
    std::vector<double> angularSpeeds;
    for (size_t i = 0; i < tail.size(); ++i)
    {
        for (size_t j = 0; j < i; ++j) positionRange = std::max(positionRange, Distance(tail[i], tail[j]));
        const double angle = std::atan2(-double(tail[i][9]), double(tail[i][10])) * 180 / Pi;
        minAngle = std::min(minAngle, angle);
        maxAngle = std::max(maxAngle, angle);
        if (i) angularSpeeds.push_back(AngularSpeed(tail[i - 1], tail[i]));
    }
    const double angularP95 = P95(angularSpeeds);
    std::printf("pendulum final_x_deg=%.9g max_travel_m=%.9g tail_position_range_m=%.9g tail_angle_range_deg=%.9g tail_angular_p95_deg_s=%.9g\n",
                angleX, maxTravel, positionRange, maxAngle - minAngle, angularP95);
    Require(maxTravel > .05 && Distance(initial, final) > .05, "Pendulum did not move under gravity");
    Require(std::abs(angleX - 20) < .1, "Pendulum did not reach its permitted 20-degree upper limit");
    Require(positionRange < 1.e-4 && maxAngle - minAngle < .01 && angularP95 < .1,
            "Angular limit retained a visible steady-state limit cycle");
}


void AlignedRestWake()
{
    auto world = Create({0, -9.8f, 0});
    std::array<RigidBodySimulationData, 2> bodies = {Body(0, 0, 0, 0, true), Body(1, 0, .1f, .05f)};
    bodies[1].mode = RigidBodyMode::DynamicBoneAligned;
    bodies[1].shape = RigidBodyShape::Capsule;
    bodies[1].size[0] = .008f; bodies[1].size[1] = .12f;
    MMDBulletPhysicsBuildRigidBodies(world.get(), bodies.data(), int(bodies.size()));
    auto joint = FreeJoint(JointType::Spring6DOF, 0, 1);
    Identity(joint.frameInB, 0, -.1f, -.05f);
    for (int axis = 0; axis < 3; ++axis)
    {
        joint.translationLimitMin[axis] = joint.translationLimitMax[axis] = 0;
        joint.rotationLimitMin[axis] = joint.rotationLimitMax[axis] = 0;
    }
    joint.rotationLimitMin[0] = float(-20 * Pi / 180);
    joint.rotationLimitMax[0] = float(20 * Pi / 180);
    MMDBulletPhysicsBuildJoints(world.get(), &joint, 1);
    const int anchor = 0, child = 1;
    Matrix target{}; Identity(target.data());
    auto frame = [&]() {
        MMDBulletPhysicsSetRigidBodyTransforms(world.get(), 1, target.data(), &anchor, false);
        MMDBulletPhysicsStepSimulation(world.get(), Step);
        auto pose = Read(world.get(), child);
        // PMX Mode2 retains the animated pivot; only its rotation is simulated.
        const float shift[3] = {pose[12] - target[12] - pose[4] * .1f - pose[8] * .05f,
                               pose[13] - target[13] - pose[5] * .1f - pose[9] * .05f,
                               pose[14] - target[14] - pose[6] * .1f - pose[10] * .05f};
        MMDBulletPhysicsShiftRigidBodyPosition(world.get(), child, shift);
        return Read(world.get(), child);
    };
    std::vector<double> speeds;
    Matrix previous = Read(world.get(), child);
    for (int step = 0; step < 1200; ++step)
    {
        const auto current = frame();
        if (step >= 1080) speeds.push_back(AngularSpeed(previous, current));
        previous = current;
    }
    const double restP95 = P95(speeds);
    std::printf("aligned_rest angular_p95_deg_s=%.9g\n", restP95);
    Require(restP95 < .01, "An unchanged kinematic anchor kept a Mode2 pendant visibly moving");
    const auto resting = previous;
    const float angle = float(40 * Pi / 180);
    target[5] = target[10] = std::cos(angle); target[6] = std::sin(angle); target[9] = -std::sin(angle);
    for (int step = 0; step < 180; ++step) previous = frame();
    const auto q0 = Quaternion(resting), q1 = Quaternion(previous);
    double dot = 0; for (int i = 0; i < 4; ++i) dot += q0[i] * q1[i];
    const double response = 2 * std::acos(std::min(1., std::abs(dot))) * 180 / Pi;
    std::printf("aligned_wake response_deg=%.9g\n", response);
    Require(response > 10, "Moving a kinematic endpoint did not wake its resting pendant");
    // Reset to the original bind pose after rest; the next gravity steps must run again.
    const int indices[2] = {0, 1};
    float reset[32]; Identity(reset); Identity(reset + 16, 0, .1f, .05f); Identity(target.data());
    MMDBulletPhysicsSetRigidBodyTransforms(world.get(), 2, reset, indices, true);
    MMDBulletPhysicsReset(world.get(), 0);
    for (int step = 0; step < 60; ++step) previous = frame();
    Require(std::abs(std::atan2(-double(previous[9]), double(previous[10]))) > .1,
            "Reset left dynamic bodies asleep instead of restoring gravity simulation");
}

void LinkedOnlyCollision()
{
    auto world = Create({ 0, 0, 0 });
    std::array<RigidBodySimulationData, 4> bodies = {
        Body(0, 0, 0, 0), Body(1, .25f, 0, 0), Body(2, 10, 0, 0), Body(3, 10.25f, 0, 0) };
    for (int i = 0; i < int(bodies.size()); ++i)
    {
        bodies[i].size[0] = .5f;
        bodies[i].groupIndex = uint8_t(i % 2 == 0 ? 3 : 9);
        // Deliberately different raw allowed-group masks; each permits the opposite group.
        // This also rejects accidentally complementing the PMX mask or disabling all contacts.
        bodies[i].collisionGroupMask = int16_t(i % 2 == 0 ? (1 << 9) | (1 << 6) : (1 << 3) | (1 << 12));
    }
    MMDBulletPhysicsBuildRigidBodies(world.get(), bodies.data(), int(bodies.size()));
    const auto joint = FreeJoint(JointType::Generic6DOF, 0, 1);
    MMDBulletPhysicsBuildJoints(world.get(), &joint, 1);
    for (int step = 0; step < 120; ++step) MMDBulletPhysicsStepSimulation(world.get(), Step);
    const double linkedDistance = Distance(Read(world.get(), 0), Read(world.get(), 1));
    const double unlinkedDistance = Distance(Read(world.get(), 2), Read(world.get(), 3));
    std::printf("collision linked_distance_m=%.9g unlinked_distance_m=%.9g sphere_diameter_m=1\n",
                linkedDistance, unlinkedDistance);
    Require(std::abs(linkedDistance - .25) < 1.e-4,
            "A free joint's linked bodies incorrectly collided and separated");
    Require(unlinkedDistance > .95, "Unlinked allowed-mask bodies did not resolve their overlap");
}
}

int main()
{
    const std::array<std::pair<const char*, void (*)()>, 4> tests = {{
        { "zero_stiffness_damping", &ZeroStiffnessDamping },
        { "angular_limit_pendulum", &AngularLimitPendulum },
        { "linked_only_collision", &LinkedOnlyCollision },
        { "aligned_rest_wake", &AlignedRestWake }
    }};
    int failures = 0;
    for (const auto& test : tests)
    {
        try { test.second(); std::printf("PASS %s\n", test.first); }
        catch (const std::exception& error)
        {
            ++failures;
            std::printf("FAIL %s: %s\n", test.first, error.what());
        }
    }
    return failures ? 1 : 0;
}
