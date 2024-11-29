#pragma once
#include "DynamicStrands.hpp"
using namespace evo_engine;

namespace eco_sys_lab_plugin {
class IDsCollider : public IPrivateComponent {
 public:
  glm::vec4 bound_color = glm::vec4(1, 0, 1, 0.1f);
  virtual void RenderBound(const std::shared_ptr<EditorLayer>& editor_layer,
                           const std::shared_ptr<Camera>& editor_camera, const glm::vec4& color) = 0;
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

  glm::vec3 scale = glm::vec3(0.5f);

  float softness = 1.f;

  void RenderBound(const std::shared_ptr<EditorLayer>& editor_layer, const std::shared_ptr<Camera>& editor_camera,
                   const glm::vec4& color) override;
  inline static std::shared_ptr<ComputePipeline> pipeline;
  DsBoxCollider();
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                 const DynamicStrands& target_dynamic_strands) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};

class DsCylinderCollider : public IDsCollider {
 public:
  struct PushConstant {
    glm::quat obb_rotation;
    glm::vec3 obb_center;
    float padding;
    float radius;
    float height;
    float softness = 1.0f;
    uint32_t segment_size;
  };

  float radius = .5f;
  float height = .5f;

  float softness = 1.f;

  void RenderBound(const std::shared_ptr<EditorLayer>& editor_layer, const std::shared_ptr<Camera>& editor_camera,
                   const glm::vec4& color) override;
  inline static std::shared_ptr<ComputePipeline> pipeline;
  DsCylinderCollider();
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                 const DynamicStrands& target_dynamic_strands) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};

class DsSphereCollider : public IDsCollider {
 public:
  struct PushConstant {
    glm::vec3 obb_center;
    float padding;
    float radius;
    float softness = 1.0f;
    uint32_t segment_size;
  };
  float radius = .5f;
  float softness = 1.f;

  void RenderBound(const std::shared_ptr<EditorLayer>& editor_layer, const std::shared_ptr<Camera>& editor_camera,
                   const glm::vec4& color) override;
  inline static std::shared_ptr<ComputePipeline> pipeline;
  DsSphereCollider();
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                 const DynamicStrands& target_dynamic_strands) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};

}  // namespace eco_sys_lab_plugin