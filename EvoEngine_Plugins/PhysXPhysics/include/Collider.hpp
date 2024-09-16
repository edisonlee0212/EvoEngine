#pragma once
#include "Entities.hpp"
#include "IAsset.hpp"
#include <PhysicsMaterial.hpp>

namespace evo_engine
{
enum class ShapeType
{
    Sphere,
    Box,
    Capsule
};
class Collider : public IAsset
{
    friend class PhysicsLayer;
    friend class RigidBody;
    PxShape *shape_ = nullptr;
    glm::vec3 shape_param_ = glm::vec3(1.0f);
    ShapeType shape_type_ = ShapeType::Box;
    AssetRef physics_material_;

    size_t attach_count_ = 0;

  public:
    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
    void OnCreate() override;
    ~Collider() override;
    void SetShapeType(const ShapeType& type);
    void SetShapeParam(const glm::vec3& param);
    void SetMaterial(const std::shared_ptr<PhysicsMaterial>& material);
    void CollectAssetRef(std::vector<AssetRef> &list) override;
    void Serialize(YAML::Emitter &out) const override;
    void Deserialize(const YAML::Node &in) override;
};
} // namespace evo_engine