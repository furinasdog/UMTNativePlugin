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

World Create(const std::array<float, 3>& gravity, float fixedStep = Step)
{
    World world(MMDBulletPhysicsCreate(gravity.data(), 20, 1, fixedStep), &MMDBulletPhysicsDestroy);
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
    // Mode2 remains permanent here; free-body release is not its correctness criterion.
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


void ExactAnchoredWeld()
{
    auto world = Create({0, -9.8f, 0});
    std::array<RigidBodySimulationData, 3> bodies = {
        Body(0, 0, 0, 0, true), Body(1, .1f, .4f, .3f), Body(2, .3f, .7f, .1f)};
    bodies[1].size[0] = .005f;
    MMDBulletPhysicsBuildRigidBodies(world.get(), bodies.data(), int(bodies.size()));
    std::array<NativeJointData, 2> joints = {FreeJoint(JointType::Spring6DOF, 0, 1), FreeJoint(JointType::Spring6DOF, 1, 2)};
    Identity(joints[0].frameInB, -.1f, -.4f, -.3f);
    Identity(joints[1].frameInB, -.2f, -.3f, .2f);
    for (auto& joint : joints)
        for (int axis = 0; axis < 3; ++axis)
            joint.translationLimitMin[axis] = joint.translationLimitMax[axis]
                = joint.rotationLimitMin[axis] = joint.rotationLimitMax[axis] = 0;
    MMDBulletPhysicsBuildJoints(world.get(), joints.data(), int(joints.size()));
    const int anchor = 0;
    Matrix target{}; Identity(target.data());
    double error = 0;
    for (int frame = 0; frame < 180; ++frame)
    {
        const float angle = frame < 60 ? 0.f : frame < 120 ? .7f : -.4f;
        target[0] = target[10] = std::cos(angle); target[2] = -std::sin(angle); target[8] = std::sin(angle);
        target[12] = frame < 60 ? 0.f : .8f;
        MMDBulletPhysicsSetRigidBodyTransforms(world.get(), 1, target.data(), &anchor, false);
        MMDBulletPhysicsStepSimulation(world.get(), Step);
        for (int body = 1; body < 3; ++body)
        {
            const auto actual = Read(world.get(), body);
            const float* rest = bodies[body].restWorldTransform;
            for (int axis = 0; axis < 3; ++axis)
            {
                const double expected = target[12 + axis] + target[axis] * rest[12]
                    + target[4 + axis] * rest[13] + target[8 + axis] * rest[14];
                error = std::max(error, std::abs(actual[12 + axis] - expected));
            }
            for (int column = 0; column < 3; ++column)
                for (int row = 0; row < 3; ++row)
                    error = std::max(error, std::abs(double(actual[4 * column + row]) - target[4 * column + row]));
        }
    }
    std::printf("anchored_weld rest_and_moving_frame_error=%.9g\n", error);
    Require(error < 1.e-6, "An anchored zero-DOF weld drifted from its prescribed rigid transform");
}


void ObstructedAttachment()
{
    constexpr float ContactStep = 1.f / 30.f;
    auto world = Create({0, -9.8f, 0}, ContactStep);
    std::array<RigidBodySimulationData, 3> bodies = {
        Body(0, 0, 2, 0, true), Body(1, 0, 1.8f, 0), Body(2, .4f, 1.65f, 0, true)};
    bodies[1].shape = RigidBodyShape::Capsule;
    bodies[1].size[0] = .02f; bodies[1].size[1] = .4f; bodies[1].mass = .25f;
    bodies[1].linearDamping = bodies[1].angularDamping = .95f;
    bodies[1].groupIndex = 1; bodies[1].collisionGroupMask = 1 << 2;
    bodies[2].size[0] = .15f; bodies[2].groupIndex = 2; bodies[2].collisionGroupMask = 1 << 1;
    MMDBulletPhysicsBuildRigidBodies(world.get(), bodies.data(), int(bodies.size()));
    auto joint = FreeJoint(JointType::Spring6DOF, 0, 1);
    Identity(joint.frameInB, 0, .2f, 0);
    for (int axis = 0; axis < 3; ++axis) {
        joint.translationLimitMin[axis] = joint.translationLimitMax[axis] = 0;
        joint.rotationLimitMin[axis] = -.08726646f; joint.rotationLimitMax[axis] = .08726646f;
        joint.springRotation[axis] = .5f;
    }
    MMDBulletPhysicsBuildJoints(world.get(), &joint, 1);
    Matrix obstacle{}; const int obstacleIndex = 2;
    double maximumDepth = 0, maximumPivotError = 0;
    for (int frame = 0; frame < 360; ++frame) {
        const float x = .4f - .32f * std::min(1.f, float(frame) / 60.f);
        Identity(obstacle.data(), x, 1.65f, 0);
        MMDBulletPhysicsSetRigidBodyTransforms(world.get(), 1, obstacle.data(), &obstacleIndex, false);
        MMDBulletPhysicsStepSimulation(world.get(), ContactStep);
        const auto pose = Read(world.get(), 1);
        if (frame < 240) continue;
        double along = 0;
        for (int axis = 0; axis < 3; ++axis) along += (obstacle[12 + axis] - pose[12 + axis]) * pose[4 + axis];
        along = std::clamp(along, -.2, .2);
        double distance2 = 0, pivot2 = 0;
        for (int axis = 0; axis < 3; ++axis) {
            const double distance = pose[12 + axis] + along * pose[4 + axis] - obstacle[12 + axis];
            const double pivot = pose[12 + axis] + .2 * pose[4 + axis] - (axis == 1 ? 2. : 0.);
            distance2 += distance * distance; pivot2 += pivot * pivot;
        }
        maximumDepth = std::max(maximumDepth, .17 - std::sqrt(distance2));
        maximumPivotError = std::max(maximumPivotError, std::sqrt(pivot2));
    }
    std::printf("obstructed_attachment penetration_m=%.9g pivot_error_m=%.9g\n", maximumDepth, maximumPivotError);
    Require(maximumDepth < .001, "Obstructed attachment remains inside a kinematic collider");
    Require(maximumPivotError < .000001, "Contact correction detached the attachment pivot");
    Identity(obstacle.data(), .4f, 1.65f, 0);
    MMDBulletPhysicsSetRigidBodyTransforms(world.get(), 1, obstacle.data(), &obstacleIndex, false);
    for (int frame = 0; frame < 360; ++frame) MMDBulletPhysicsStepSimulation(world.get(), ContactStep);
    const auto returned = Read(world.get(), 1);
    const double lean = std::acos(std::clamp(double(returned[5]), -1., 1.));
    Require(lean < .1, "Unobstructed attachment did not return within its original angular range");
}

void WeldedContactOwnership()
{
    auto simulate = [](bool welded) {
        constexpr float dt = 1.f / 30.f;
        auto world = Create({0, -9.8f, 0}, dt);
        std::array<RigidBodySimulationData, 5> bodies = {
            Body(0, 0, 2, 0, true), Body(1, 0, 1.8f, 0), Body(2, .08f, 1.65f, 0, true),
            Body(3, 0, 2, 0), Body(4, 0, 2, 0)};
        for (int i = 0; i < 5; ++i) { bodies[i].relatedBoneIndex = i; bodies[i].noRelatedBone = false; }
        bodies[2].relatedBoneIndex = 0;
        bodies[1].shape = RigidBodyShape::Capsule;
        bodies[1].size[0] = .02f; bodies[1].size[1] = .4f; bodies[1].mass = .25f;
        bodies[1].linearDamping = bodies[1].angularDamping = .95f;
        bodies[1].groupIndex = 1; bodies[1].collisionGroupMask = 1 << 2;
        bodies[2].size[0] = .15f; bodies[2].groupIndex = 2; bodies[2].collisionGroupMask = 1 << 1;
        MMDBulletPhysicsBuildRigidBodies(world.get(), bodies.data(), int(bodies.size()));
        std::array<NativeJointData, 3> joints = {
            FreeJoint(JointType::Spring6DOF, 3, 4), FreeJoint(JointType::Spring6DOF, 0, 3),
            FreeJoint(JointType::Spring6DOF, welded ? 4 : 0, 1)};
        for (auto& joint : joints) for (int axis = 0; axis < 3; ++axis) {
            joint.translationLimitMin[axis] = joint.translationLimitMax[axis] = 0;
            joint.rotationLimitMin[axis] = joint.rotationLimitMax[axis] = 0;
        }
        Identity(joints[2].frameInB, 0, .2f, 0);
        for (int axis = 0; axis < 3; ++axis) {
            joints[2].rotationLimitMin[axis] = -.034906585f;
            joints[2].rotationLimitMax[axis] = .034906585f;
        }
        MMDBulletPhysicsBuildJoints(world.get(), joints.data(), int(joints.size()));
        for (int frame = 0; frame < 360; ++frame) MMDBulletPhysicsStepSimulation(world.get(), dt);
        return Read(world.get(), 1);
    };
    const auto direct = simulate(false), welded = simulate(true);
    double error = 0;
    for (int i = 0; i < 16; ++i) error = std::max(error, std::abs(double(direct[i]) - welded[i]));
    std::printf("welded_own_head direct_vs_welded_pose_error=%.9g\n", error);
    Require(error < 1.e-5, "An exact welded root incorrectly changes own-head contact relief");
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
    const std::array<std::pair<const char*, void (*)()>, 7> tests = {{
        { "zero_stiffness_damping", &ZeroStiffnessDamping },
        { "angular_limit_pendulum", &AngularLimitPendulum },
        { "linked_only_collision", &LinkedOnlyCollision },
        { "aligned_rest_wake", &AlignedRestWake },
        { "exact_anchored_weld", &ExactAnchoredWeld },
        { "obstructed_attachment", &ObstructedAttachment },
        { "welded_contact_ownership", &WeldedContactOwnership }
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
