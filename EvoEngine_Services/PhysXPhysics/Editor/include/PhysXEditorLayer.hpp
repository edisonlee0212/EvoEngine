#pragma once

#include "ILayer.hpp"
#include "InspectorRegistry.hpp"

namespace evo_engine {
class Collider;
class RigidBody;
class Joint;
class PhysicsMaterial;
class PhysXEditorLayer final : public ILayer {
  static bool Inspect(InspectorContext& context, Collider& target);
  static bool Inspect(InspectorContext& context, RigidBody& target);
  static bool Inspect(InspectorContext& context, Joint& target);
  static bool Inspect(InspectorContext& context, PhysicsMaterial& target);

 public:
  void RegisterTypes(Application& application) override;
};
}  // namespace evo_engine
