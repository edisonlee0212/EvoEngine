#pragma once
#include "DynamicStrands.hpp"
using namespace evo_engine;

namespace eco_sys_lab_plugin {
class IDsConstraint {
 public:
  virtual void InitializeData(const DynamicStrands::InitializeParameters& initialize_parameters,
                              const StrandModelSkeleton& strand_model_skeleton,
                              const DtsStrandGroup& subdivided_strand_group,
                              const DynamicStrands& target_dynamic_strands) {
  }
  virtual void ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                         const DynamicStrands& target_dynamic_strands) {
  }
  virtual void ProjectVelocityConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                         const DynamicStrands& target_dynamic_strands) {
  }
  virtual void DownloadData() {
  }
  virtual void UploadData() {
  }
  virtual void UpdateBindings() {
  }
  virtual bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
    return false;
  }
  bool enabled = true;
};

#pragma region Constraints
class DsPivotPoint final : public IDsConstraint {
 public:
  GlobalTransform inverse_base_global_transform{};
  GlobalTransform base_global_transform{};

  struct SegmentUpdate {
    float point_distance;
    uint32_t segment_index;
    uint32_t padding0;
    uint32_t padding1;
  };
  inline static std::shared_ptr<DescriptorSetLayout> layout{};
  std::vector<SegmentUpdate> commands;
  std::shared_ptr<Buffer> segment_update_commands_buffer;

  struct SegmentUpdatePushConstant {
    glm::vec3 pivot_position;
    uint32_t commands_size = 0;
  };

  SegmentUpdatePushConstant push_constant;
  inline static std::shared_ptr<ComputePipeline> segment_update_pipeline;
  std::vector<std::shared_ptr<DescriptorSet>> segment_commands_descriptor_sets;

  DsPivotPoint();
  void Initialize(const GlobalTransform& target_base_global_transform,
                  const std::shared_ptr<DynamicStrands>& target_dynamic_strands,
                  const std::vector<std::pair<uint32_t, bool>>& segment_list);
  void Update(const GlobalTransform& new_global_transform);

  void ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                 const DynamicStrands& target_dynamic_strands) override;
};

class DsPivotAxis final : public IDsConstraint {
 public:
  GlobalTransform inverse_base_global_transform{};
  GlobalTransform base_global_transform{};

  struct SegmentUpdate {
    float axis_offset;
    float axis_distance;
    uint32_t segment_index;
    uint32_t particle0_closer;
  };
  inline static std::shared_ptr<DescriptorSetLayout> layout{};
  std::vector<SegmentUpdate> commands;
  std::shared_ptr<Buffer> segment_update_commands_buffer;

  struct SegmentUpdatePushConstant {
    glm::vec3 pivot_position;
    uint32_t commands_size = 0;
    glm::vec3 axis;
  };
  SegmentUpdatePushConstant push_constant;
  inline static std::shared_ptr<ComputePipeline> segment_update_pipeline;
  std::vector<std::shared_ptr<DescriptorSet>> segment_commands_descriptor_sets;

  DsPivotAxis();
  void Initialize(const GlobalTransform& target_base_global_transform,
                  const std::shared_ptr<DynamicStrands>& target_dynamic_strands,
                  const std::vector<std::pair<uint32_t, bool>>& segment_list);
  void Update(const GlobalTransform& new_global_transform);

  void ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                 const DynamicStrands& target_dynamic_strands) override;
};

class DsPivotTransform final : public IDsConstraint {
 public:
  GlobalTransform inverse_base_global_transform{};
  GlobalTransform base_global_transform{};

  struct SegmentUpdate {
    glm::quat new_rotation;
    glm::vec3 new_particle0_position;
    uint32_t fix_particle0;
    glm::vec3 new_particle1_position;
    uint32_t fix_particle1;
    uint32_t segment_index;

    uint32_t padding0;
    uint32_t padding1;
    uint32_t padding2;
  };
  inline static std::shared_ptr<DescriptorSetLayout> layout{};
  std::vector<SegmentUpdate> commands;
  std::vector<std::shared_ptr<Buffer>> segment_update_commands_buffer;

  struct SegmentUpdatePushConstant {
    uint32_t commands_size = 0;
  };

  inline static std::shared_ptr<ComputePipeline> segment_update_pipeline;
  std::vector<std::shared_ptr<DescriptorSet>> segment_commands_descriptor_sets;

  DsPivotTransform();
  void Initialize(const GlobalTransform& target_base_global_transform,
                  const std::shared_ptr<DynamicStrands>& target_dynamic_strands,
                  const std::vector<std::pair<uint32_t, std::pair<bool, bool>>>& segment_list);
  void Update(const GlobalTransform& new_global_transform,
              const std::shared_ptr<DynamicStrands>& target_dynamic_strands);

  void ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                 const DynamicStrands& target_dynamic_strands) override;
};

class DsGroundPlane final : public IDsConstraint {
 public:
  struct SegmentGroundPlanePushConstant {
    uint32_t segment_size;
    float ground_height;
    float ground_softness;
    float ground_friction;
  };
  struct LeafGroundPlanePushConstant {
    uint32_t leaf_size;
    float ground_height;
    float ground_softness;
    float ground_friction;
  };
  float ground_height = -0.075f;
  float ground_softness = 0.95f;
  float ground_friction = 0.5f;
  inline static std::shared_ptr<ComputePipeline> segment_pipeline{};
  inline static std::shared_ptr<ComputePipeline> leaf_pipeline{};
  void ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                 const DynamicStrands& target_dynamic_strands) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  DsGroundPlane();
};

class DsStiffRod final : public IDsConstraint {
 public:
  DsStiffRod();
  inline static std::shared_ptr<DescriptorSetLayout> layout{};
  struct ShearStretchConstraintConstant {
    uint32_t strand_size = 0;
    float inv_time_step;
    uint32_t frame_index;
  };

  int sub_iteration = 1;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  inline static std::shared_ptr<ComputePipeline> pipeline{};

  void ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                 const DynamicStrands& target_dynamic_strands) override;
  static glm::vec3 ComputeDarbouxVector(const glm::quat& q0, const glm::quat& q1, float average_segment_length);
};

class DsBundle : public IDsConstraint {
 public:
  struct RandomBundleShearStretchConstant {
    uint32_t skip_index = 0;
    uint32_t skip_size = 1;
    uint32_t segment_size = 0;
    float inv_time_step = 0.0f;
  };

  struct RandomBundleConstant {
    uint32_t skip_index = 0;
    uint32_t skip_size = 1;
    uint32_t segment_size = 0;
    float inv_time_step = 0.0f;
    float over_relaxation;
  };

  struct RandomBundleBendTwistConstant {
    uint32_t skip_index = 0;
    uint32_t skip_size = 1;
    uint32_t segment_size = 0;
    float inv_time_step = 0.0f;
    float over_relaxation;
  };

  struct RandomBundleApplySegmentsConstant {
    uint32_t skip_index = 0;
    uint32_t skip_size = 1;
    uint32_t segment_size = 0;
    float inv_time_step = 0.0f;
  };

  struct RandomBundleApplyConnectionsConstant {
    uint32_t segment_pair_size = 0;
    float inv_time_step = 0.0f;
  };
  int skip_size = 1;
  float over_relaxation = 1.f;
  float bend_twist_over_relaxation = 1.f;

  inline static std::shared_ptr<ComputePipeline> stretch_shear_pipeline{};
  inline static std::shared_ptr<ComputePipeline> bend_twist_pipeline{};
  inline static std::shared_ptr<ComputePipeline> bundle_position_pipeline{};
  inline static std::shared_ptr<ComputePipeline> bundle_rotation_pipeline{};
  inline static std::shared_ptr<ComputePipeline> apply_rotation_pipeline{};
  inline static std::shared_ptr<ComputePipeline> apply_position_pipeline{};
  inline static std::shared_ptr<ComputePipeline> apply_position_rotation_pipeline{};
  inline static std::shared_ptr<ComputePipeline> connections_pipeline{};
  DsBundle();
  int sub_iteration = 1;
  bool enable_bundle_position = true;
  // Not stable
  bool enable_bundle_rotation = false;
  bool enable_bend_twist = true;
  bool enable_stretch_shear = true;
  bool enable_connections = true;
  void ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                 const DynamicStrands& target_dynamic_strands) override;

  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};

class DsLeafAttachment : public IDsConstraint {
 public:
  struct LeafPredictionPushConstant {
    uint32_t leaf_size = 0;
  };
  inline static std::shared_ptr<ComputePipeline> pipeline{};
  DsLeafAttachment();
  void ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                 const DynamicStrands& target_dynamic_strands) override;
};
#pragma endregion
}  // namespace eco_sys_lab_plugin