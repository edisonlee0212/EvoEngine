#pragma once
#include "DynamicStrands.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

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
  struct SegmentPositionPushConstant {
    glm::quat obb_rotation;
    glm::vec3 obb_center;
    float padding0;
    glm::vec3 obb_scale;
    float padding1;
    float softness = 1.0f;
    uint32_t segment_size;
    float friction = 1.0f;
    float rotational_friction;
  };

  struct LeafPositionPushConstant {
    glm::quat obb_rotation;
    glm::vec3 obb_center;
    float padding0;
    glm::vec3 obb_scale;
    float padding1;
    float softness = 1.0f;
    uint32_t leaf_size;
    float friction = 1.0f;
    float rotational_friction;
  };

  struct SegmentVelocityPushConstant {
    glm::quat obb_rotation;
    glm::vec3 obb_center;
    float padding0;
    glm::vec3 obb_scale;
    float padding1;
    float velocity_friction = 1.0f;
    float angular_velocity_friction;
    uint32_t segment_size;
  };

  struct LeafVelocityPushConstant {
    glm::quat obb_rotation;
    glm::vec3 obb_center;
    float padding0;
    glm::vec3 obb_scale;
    float padding1;
    float velocity_friction = 1.0f;
    float angular_velocity_friction;
    uint32_t leaf_size;
  };

  glm::vec3 scale = glm::vec3(0.51f);

  float softness = 1.f;
  float friction = 1.0f;
  float rotational_friction = 1.0f;

  float velocity_friction = 0.0f;
  float angular_velocity_friction = 0.0f;
  void RenderBound(const std::shared_ptr<EditorLayer>& editor_layer, const std::shared_ptr<Camera>& editor_camera,
                   const glm::vec4& color) override;
  inline static std::shared_ptr<ComputePipeline> segment_position_pipeline;
  inline static std::shared_ptr<ComputePipeline> leaf_position_pipeline;
  inline static std::shared_ptr<ComputePipeline> segment_velocity_pipeline;
  inline static std::shared_ptr<ComputePipeline> leaf_velocity_pipeline;

  DsBoxCollider();
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                 const DynamicStrands& target_dynamic_strands) override;

  void ProjectVelocityConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                 const DynamicStrands& target_dynamic_strands) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};

class DsCylinderCollider : public IDsCollider {
 public:
  struct SegmentPushConstant {
    glm::quat obb_rotation;
    glm::vec3 obb_center;
    float padding;
    float radius;
    float height;
    float softness = 1.0f;
    uint32_t segment_size;
  };

  struct LeafPushConstant {
    glm::quat obb_rotation;
    glm::vec3 obb_center;
    float padding;
    float radius;
    float height;
    float softness = 1.0f;
    uint32_t leaf_size;
  };

  float radius = .51f;
  float height = .51f;

  float softness = 1.f;

  void RenderBound(const std::shared_ptr<EditorLayer>& editor_layer, const std::shared_ptr<Camera>& editor_camera,
                   const glm::vec4& color) override;
  inline static std::shared_ptr<ComputePipeline> segment_position_pipeline;
  inline static std::shared_ptr<ComputePipeline> leaf_position_pipeline;
  DsCylinderCollider();
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                 const DynamicStrands& target_dynamic_strands) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};

class DsSphereCollider : public IDsCollider {
 public:
  struct SegmentPushConstant {
    glm::vec3 obb_center;
    float padding;
    float radius;
    float softness = 1.0f;
    uint32_t segment_size;
  };

  struct LeafPushConstant {
    glm::vec3 obb_center;
    float padding;
    float radius;
    float softness = 1.0f;
    uint32_t leaf_size;
  };

  float radius = .51f;
  float softness = 1.f;

  void RenderBound(const std::shared_ptr<EditorLayer>& editor_layer, const std::shared_ptr<Camera>& editor_camera,
                   const glm::vec4& color) override;
  inline static std::shared_ptr<ComputePipeline> segment_position_pipeline;
  inline static std::shared_ptr<ComputePipeline> leaf_position_pipeline;
  DsSphereCollider();
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                 const DynamicStrands& target_dynamic_strands) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};
}  // namespace eco_sys_lab_plugin