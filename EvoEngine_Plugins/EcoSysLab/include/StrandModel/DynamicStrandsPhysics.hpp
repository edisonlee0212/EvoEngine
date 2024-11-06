#pragma once
#include "DynamicStrands.hpp"
using namespace evo_engine;

namespace eco_sys_lab_plugin {
class DynamicStrandsPreStep {
 public:
  DynamicStrandsPreStep();

  struct ParticlePreStepPushConstant {
    uint32_t particle_size = 0;
    float time_step = 0.01f;
    float inv_time_step = 100.f;
  };

  struct SegmentPreStepPushConstant {
    uint32_t segment_size = 0;
    float time_step = 0.01f;
    float inv_time_step = 100.f;
  };

  struct ConnectionPreStepPushConstant {
    uint32_t connection_size = 0;
    float time_step = 0.01f;
    float inv_time_step = 100.f;
  };

  inline static std::shared_ptr<ComputePipeline> particle_pre_step_pipeline;
  inline static std::shared_ptr<ComputePipeline> segment_pre_step_pipeline;
  inline static std::shared_ptr<ComputePipeline> connection_pre_step_pipeline;
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands);
};
class DynamicStrandsPrediction {
 public:
  DynamicStrandsPrediction();

  struct ParticlePredictionPushConstant {
    uint32_t particle_size = 0;
    float time_step = 0.01f;
    float inv_time_step = 100.f;
  };

  struct SegmentPredictionPushConstant {
    uint32_t segment_size = 0;
    float time_step = 0.01f;
    float inv_time_step = 100.f;
  };

  struct ConnectionPredictionPushConstant {
    uint32_t connection_size = 0;
    float time_step = 0.01f;
    float inv_time_step = 100.f;
  };

  inline static std::shared_ptr<ComputePipeline> particle_prediction_pipeline;
  inline static std::shared_ptr<ComputePipeline> segment_prediction_pipeline;
  inline static std::shared_ptr<ComputePipeline> connection_prediction_pipeline;
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands);
};
class IDynamicStrandsOperator {
 public:
  virtual ~IDynamicStrandsOperator() = default;

  virtual void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                       const std::shared_ptr<DynamicStrands>& target_dynamic_strands) = 0;
  virtual bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
    return false;
  }
  bool enabled = true;
};
class IDynamicStrandsConstraint {
 public:
  virtual void InitializeData(const DynamicStrands::InitializeParameters& initialize_parameters,
                              const StrandModelSkeleton& strand_model_skeleton,
                              const DynamicStrands& target_dynamic_strands) {
  }
  virtual void Project(const DynamicStrands::PhysicsParameters& physics_parameters,
                       const DynamicStrands& target_dynamic_strands) = 0;

  virtual void DownloadData() {
  }
  virtual void UploadData() {
  }
  virtual bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
    return false;
  }
  bool enabled = true;
};

#pragma region Operators
class DsTransform final : public IDynamicStrandsOperator {
 public:
  GlobalTransform inverse_base_global_transform{};
  GlobalTransform base_global_transform{};
  // Position
  struct PositionUpdate {
    glm::vec3 new_position = glm::vec3(0.f);
    uint32_t particle_index = 0;
  };
  inline static std::shared_ptr<DescriptorSetLayout> position_layout{};
  std::vector<PositionUpdate> position_commands;
  std::vector<std::shared_ptr<Buffer>> position_commands_buffer;
  struct PositionUpdatePushConstant {
    uint32_t commands_size = 0;
  };
  inline static std::shared_ptr<ComputePipeline> position_update_pipeline;
  std::vector<std::shared_ptr<DescriptorSet>> position_commands_descriptor_sets;
  // Rotation
  struct RotationUpdate {
    glm::quat new_rotation = glm::vec3(0.f);
    uint32_t segment_index = 0;
    uint32_t padding0 = 0;
    uint32_t padding1 = 0;
    uint32_t padding2 = 0;
  };
  inline static std::shared_ptr<DescriptorSetLayout> rotation_layout{};
  std::vector<RotationUpdate> rotation_commands;
  std::vector<std::shared_ptr<Buffer>> rotation_commands_buffer;

  struct RotationUpdatePushConstant {
    uint32_t commands_size = 0;
  };

  inline static std::shared_ptr<ComputePipeline> rotation_update_pipeline;
  std::vector<std::shared_ptr<DescriptorSet>> rotation_commands_descriptor_sets;

  DsTransform();
  void Initialize(const GlobalTransform& target_base_global_transform,
                  const std::shared_ptr<DynamicStrands>& target_dynamic_strands,
                  const std::vector<uint32_t>& segment_handles);
  void Update(const GlobalTransform& new_global_transform,
              const std::shared_ptr<DynamicStrands>& target_dynamic_strands);
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const std::shared_ptr<DynamicStrands>& target_dynamic_strands) override;
};

class DsGravity final : public IDynamicStrandsOperator {
 public:
  struct GravityPushConstant {
    glm::vec3 acceleration;
    uint32_t particle_size = 0;
  };

  glm::vec3 gravity = glm::vec3(0, -9.81, 0);
  inline static std::shared_ptr<ComputePipeline> gravity_force_pipeline{};
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const std::shared_ptr<DynamicStrands>& target_dynamic_strands) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  DsGravity();
};
class DsDragForce final : public IDynamicStrandsOperator {
 public:
  glm::vec3 target_position;
  float distance_multiplier = 0.5f;
  DsDragForce();

  inline static std::shared_ptr<DescriptorSetLayout> layout{};

  std::vector<int> commands;
  std::vector<std::shared_ptr<Buffer>> commands_buffer;

  struct DragForcePushConstant {
    glm::vec3 target_position;
    float distance_multiplier;
    float time_step = 0.0f;
    uint32_t commands_size = 0;
  };

  inline static std::shared_ptr<ComputePipeline> drag_force_pipeline{};
  std::vector<std::shared_ptr<DescriptorSet>> commands_descriptor_sets{};
  void Initialize(const std::vector<int>& particle_handles);
  void Update(const glm::vec3& new_position);
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const std::shared_ptr<DynamicStrands>& target_dynamic_strands) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};

#pragma endregion

#pragma region Constraints
class DsStiffRod final : public IDynamicStrandsConstraint {
 public:
  DsStiffRod();
  inline static std::shared_ptr<DescriptorSetLayout> layout{};

  struct PerStrandData {
    int front_propagate_begin_connection_handle = -1;
    int back_propagate_begin_connection_handle = -1;
    int front_propagate_begin_segment_handle = -1;
    int back_propagate_begin_segment_handle = -1;
  };

  std::vector<PerStrandData> per_strand_data_list;

  std::shared_ptr<Buffer> per_strand_data_list_buffer;

  struct StretchShearConstraintConstant {
    uint32_t strand_size = 0;
  };

  struct BendTwistConstraintConstant {
    uint32_t strand_size = 0;
  };

  enum class ProjectMode { Bilateral, Forward, Backward, Balanced };

  uint32_t project_mode = static_cast<uint32_t>(ProjectMode::Bilateral);

  int sub_iteration = 1;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  inline static std::shared_ptr<ComputePipeline> bilateral_stretch_shear_constraint_pipeline{};
  inline static std::shared_ptr<ComputePipeline> bilateral_bend_twist_constraint_pipeline{};

  inline static std::shared_ptr<ComputePipeline> forward_stretch_shear_constraint_pipeline{};
  inline static std::shared_ptr<ComputePipeline> forward_bend_twist_constraint_pipeline{};

  inline static std::shared_ptr<ComputePipeline> backward_stretch_shear_constraint_pipeline{};
  inline static std::shared_ptr<ComputePipeline> backward_bend_twist_constraint_pipeline{};

  std::vector<std::shared_ptr<DescriptorSet>> strands_physics_descriptor_sets{};

  void InitializeData(const DynamicStrands::InitializeParameters& initialize_parameters,
                      const StrandModelSkeleton& strand_model_skeleton,
                      const DynamicStrands& target_dynamic_strands) override;

  void Project(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands) override;
  void DownloadData() override;
  void UploadData() override;
  static glm::vec3 ComputeDarbouxVector(const glm::quat& q0, const glm::quat& q1, float average_segment_length);
};

class DsParticleNeighbor : public IDynamicStrandsConstraint {
 public:
  struct ParticleNeighbor {
    glm::vec3 new_position;
    float padding;
    int neighbors[8];
    float distances[8];
  };
  DsParticleNeighbor();

  inline static std::shared_ptr<DescriptorSetLayout> layout{};
  struct ParticleNeighborConstraintConstant {
    uint32_t particle_size = 0;
  };
  std::vector<ParticleNeighbor> particle_neighbors;
  std::shared_ptr<Buffer> particle_neighbors_buffer;
  inline static std::shared_ptr<ComputePipeline> particle_neighbor_offset_pipeline{};
  inline static std::shared_ptr<ComputePipeline> particle_neighbor_apply_pipeline{};
  std::vector<std::shared_ptr<DescriptorSet>> particle_neighbors_descriptor_sets{};
  void InitializeData(const DynamicStrands::InitializeParameters& initialize_parameters,
                      const StrandModelSkeleton& strand_model_skeleton,
                      const DynamicStrands& target_dynamic_strands) override;
  int sub_iteration = 1;
  void Project(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void UploadData() override;
};
#pragma endregion
}  // namespace eco_sys_lab_plugin