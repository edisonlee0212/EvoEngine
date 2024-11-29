#pragma once
#include "DynamicStrands.hpp"
using namespace evo_engine;

namespace eco_sys_lab_plugin {
class IDsCollider : public IPrivateComponent {
 public:
  virtual void RenderBound(const std::shared_ptr<EditorLayer>& editor_layer, const std::shared_ptr<Camera>& editor_camera,
                   const glm::vec4& color) = 0;
  virtual void ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                         const DynamicStrands& target_dynamic_strands) {
  }
  virtual void ProjectVelocityConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                         const DynamicStrands& target_dynamic_strands) {
  }
};

class DsBoxCollider : public IDsCollider {
 public:
  struct PushConstant {
    glm::quat obb_rotation;
    glm::vec3 obb_center;
    float padding0;
    glm::vec3 obb_scale;
    float padding1;
    float softness = 1.0f;
    uint32_t segment_size;
  };
  float softness = 1.f;
  PrivateComponentRef mesh_renderer_ref;
  void RenderBound(const std::shared_ptr<EditorLayer>& editor_layer, const std::shared_ptr<Camera>& editor_camera,
                   const glm::vec4& color) override;
  inline static std::shared_ptr<ComputePipeline> pipeline;
  DsBoxCollider();
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                 const DynamicStrands& target_dynamic_strands) override;
  void OnDestroy() override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene) override;
};
}  // namespace eco_sys_lab_plugin