#pragma once

#include "Collider.hpp"
#include "Joint.hpp"
#include "PhysicsMaterial.hpp"
#include "RigidBody.hpp"

namespace evo_engine {
void SerializeCollider(YAML::Emitter& out, const Collider& target);
void DeserializeCollider(const YAML::Node& in, Collider& target);
void SerializePhysicsMaterial(YAML::Emitter& out, const PhysicsMaterial& target);
void DeserializePhysicsMaterial(const YAML::Node& in, PhysicsMaterial& target);
void SerializeRigidBody(YAML::Emitter& out, const RigidBody& target);
void DeserializeRigidBody(const YAML::Node& in, RigidBody& target);
void SerializeJoint(YAML::Emitter& out, const Joint& target);
void DeserializeJoint(const YAML::Node& in, Joint& target);
}  // namespace evo_engine
