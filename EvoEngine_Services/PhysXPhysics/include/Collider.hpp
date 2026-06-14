#pragma once
#include <PhysicsMaterial.hpp>
#include "Entities.hpp"
#include "IAsset.hpp"

namespace evo_engine {
enum class ShapeType { Sphere, Box, Capsule };
class Collider : public IAsset {
  friend class PhysicsLayer;
  friend class RigidBody;
  friend void SerializeCollider(YAML::Emitter& out, const Collider& target);
  friend void DeserializeCollider(const YAML::Node& in, Collider& target);
  PxShape* shape_ = nullptr;
  glm::vec3 shape_param_ = glm::vec3(1.0f);
  ShapeType shape_type_ = ShapeType::Box;
  AssetRef physics_material_;

  size_t attach_count_ = 0;

 public:
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
  void OnCreate() override;
  ~Collider() override;
  void SetShapeType(const ShapeType& type);
  void SetShapeParam(const glm::vec3& param);
  void SetMaterial(const std::shared_ptr<PhysicsMaterial>& material);
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};
}  // namespace evo_engine
