// SPDX-License-Identifier: MIT
#pragma once
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <vector>
#include <queue>

class PmxContactWorld : public btDiscreteDynamicsWorld
{
    struct Anchor {
        btGeneric6DofSpring2Constraint* joint;
        btRigidBody* kinematic;
        btVector3 low, high;
        btScalar relief = 0;
    };
    std::vector<Anchor> anchors;
    std::vector<int> owners;
    std::vector<bool> touching;
public:
    PmxContactWorld(btDispatcher* dispatcher, btBroadphaseInterface* broadphase,
        btConstraintSolver* solver, btCollisionConfiguration* config)
        : btDiscreteDynamicsWorld(dispatcher, broadphase, solver, config) {}
    void Clear() { anchors.clear(); owners.clear(); touching.clear(); }
    void Configure(const btAlignedObjectArray<btRigidBody*>& bodies, const btAlignedObjectArray<btTypedConstraint*>& joints)
    {
        anchors.clear(); owners.assign(size_t(bodies.size()), -1);
        std::vector<std::vector<int>> graph(size_t(bodies.size()));
        for (int i = 0; i < bodies.size(); ++i) bodies[i]->setUserIndex(i);
        std::queue<int> queue;
        for (int i = 0; i < joints.size(); ++i) {
            auto* base = joints[i];
            auto& a = base->getRigidBodyA(); auto& b = base->getRigidBodyB();
            const int ai = a.getUserIndex(), bi = b.getUserIndex();
            if (ai < 0 || bi < 0) continue;
            graph[size_t(ai)].push_back(bi); graph[size_t(bi)].push_back(ai);
            if (base->getConstraintType() != D6_SPRING_2_CONSTRAINT_TYPE || a.isKinematicObject() == b.isKinematicObject()) continue;
            auto* joint = static_cast<btGeneric6DofSpring2Constraint*>(base);
            btVector3 low, high, tlo, thi;
            joint->getAngularLowerLimit(low); joint->getAngularUpperLimit(high);
            joint->getLinearLowerLimit(tlo); joint->getLinearUpperLimit(thi);
            const int child = a.isKinematicObject() ? bi : ai;
            if (bodies[child]->isStaticOrKinematicObject() || tlo.length2() != 0 || thi.length2() != 0 || low == high) continue;
            if (owners[size_t(child)] >= 0) continue;
            owners[size_t(child)] = int(anchors.size());
            anchors.push_back({joint, a.isKinematicObject() ? &a : &b, low, high, 0});
            queue.push(child);
        }
        while (!queue.empty()) {
            const int parent = queue.front(); queue.pop();
            for (int child : graph[size_t(parent)]) {
                if (owners[size_t(child)] >= 0 || bodies[child]->isStaticOrKinematicObject()) continue;
                owners[size_t(child)] = owners[size_t(parent)]; queue.push(child);
            }
        }
    }
    void solveConstraints(btContactSolverInfo& info) override
    {
        touching.assign(anchors.size(), false);
        for (int i = 0; i < getDispatcher()->getNumManifolds(); ++i) {
            auto* manifold = getDispatcher()->getManifoldByIndexInternal(i);
            const auto* a = manifold->getBody0(); const auto* b = manifold->getBody1();
            if (a->isStaticOrKinematicObject() == b->isStaticOrKinematicObject()) continue;
            const auto* collider = a->isStaticOrKinematicObject() ? a : b;
            const auto* dynamic = a->isStaticOrKinematicObject() ? b : a;
            const int index = dynamic->getUserIndex();
            if (index < 0 || size_t(index) >= owners.size() || owners[size_t(index)] < 0) continue;
            const size_t owner = size_t(owners[size_t(index)]);
            const auto* anchor = anchors[owner].kinematic;
            // Existing overlapping proxies on the attachment's own bone retain their authored response.
            if (collider == anchor || (collider->getUserIndex2() >= 0
                && collider->getUserIndex2() == anchor->getUserIndex2())) continue;
            bool contact = false;
            for (int p = 0; p < manifold->getNumContacts(); ++p) {
                auto& point = manifold->getContactPoint(p);
                point.m_contactPointFlags |= BT_CONTACT_FLAG_HAS_CONTACT_CFM | BT_CONTACT_FLAG_HAS_CONTACT_ERP;
                // Joint-loop regularization must not also soften body contacts.
                point.m_contactCFM = 0; point.m_contactERP = .8;
                contact |= point.getDistance() < .002;
            }
            if (contact) touching[owner] = true;
        }
        for (size_t i = 0; i < anchors.size(); ++i) {
            auto& root = anchors[i];
            // Recover authored limits even when the source spring has zero stiffness.
            // Move each limit by at most 30 degrees/sec; persistent/repeated contact retains relief.
            if (touching[i]) {
                for (int axis = 0; axis < 3; ++axis) if (root.low[axis] < root.high[axis])
                    root.relief = btMax(root.relief, btMax(root.low[axis] + SIMD_PI / 3,
                        SIMD_PI / 3 - root.high[axis]));
            } else if (root.relief > 0) {
                root.relief = btMax(btScalar(0), root.relief - info.m_timeStep * SIMD_PI / 6);
                auto& body = &root.joint->getRigidBodyA() == root.kinematic
                    ? root.joint->getRigidBodyB() : root.joint->getRigidBodyA();
                body.activate(true);
            }
            auto low = root.low; auto high = root.high;
            for (int axis = 0; axis < 3; ++axis) if (low[axis] < high[axis]) {
                low[axis] = btMax(btMin(root.low[axis], btScalar(-SIMD_PI / 3)), root.low[axis] - root.relief);
                high[axis] = btMin(btMax(root.high[axis], btScalar(SIMD_PI / 3)), root.high[axis] + root.relief);
            }
            root.joint->setAngularLowerLimit(low); root.joint->setAngularUpperLimit(high);
        }
        btDiscreteDynamicsWorld::solveConstraints(info);
    }
    void integrateTransforms(btScalar timeStep) override
    {
        btDiscreteDynamicsWorld::integrateTransforms(timeStep);
        for (const auto& root : anchors) {
            const bool anchorIsA = &root.joint->getRigidBodyA() == root.kinematic;
            auto& body = anchorIsA ? root.joint->getRigidBodyB() : root.joint->getRigidBodyA();
            const auto& anchorFrame = anchorIsA ? root.joint->getFrameOffsetA() : root.joint->getFrameOffsetB();
            const auto& bodyFrame = anchorIsA ? root.joint->getFrameOffsetB() : root.joint->getFrameOffsetA();
            const auto& anchorPose = root.kinematic->getWorldTransform();
            auto pose = body.getWorldTransform();
            const btVector3 arm = pose.getBasis() * bodyFrame.getOrigin();
            // A translation-locked kinematic pivot has an exact position even at a limited solver budget.
            pose.setOrigin(anchorPose * anchorFrame.getOrigin() - arm);
            body.setCenterOfMassTransform(pose);
            body.setLinearVelocity(root.kinematic->getVelocityInLocalPoint(anchorPose.getBasis() * anchorFrame.getOrigin())
                - body.getAngularVelocity().cross(arm));
        }
    }
};
