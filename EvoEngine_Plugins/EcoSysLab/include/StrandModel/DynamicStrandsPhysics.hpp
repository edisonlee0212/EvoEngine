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

  inline static std::shared_ptr<ComputePipeline> particle_pre_step_pipeline;
  inline static std::shared_ptr<ComputePipeline> segment_pre_step_pipeline;

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

  inline static std::shared_ptr<ComputePipeline> particle_prediction_pipeline;
  inline static std::shared_ptr<ComputePipeline> segment_prediction_pipeline;

  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands);
};
class IDynamicStrandsOperator {
 public:
  virtual ~IDynamicStrandsOperator() = default;
  virtual void InitializeData(const DynamicStrands::InitializeParameters& initialize_parameters,
                              const StrandModelSkeleton& strand_model_skeleton,
                              const DynamicStrands& target_dynamic_strands) {
  }
  virtual void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
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
class DsPositionUpdate final : public IDynamicStrandsOperator {
 public:
  DsPositionUpdate();

  struct PositionUpdate {
    glm::vec3 new_position = glm::vec3(0.f);
    uint32_t particle_index = 0;
  };

  inline static std::shared_ptr<DescriptorSetLayout> layout{};

  std::vector<PositionUpdate> commands;
  std::vector<std::shared_ptr<Buffer>> commands_buffer;

  struct PositionUpdatePushConstant {
    uint32_t commands_size = 0;
  };

  inline static std::shared_ptr<ComputePipeline> position_update_pipeline;
  std::vector<std::shared_ptr<DescriptorSet>> commands_descriptor_sets;

  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands) override;
};
class DsRotationUpdate final : public IDynamicStrandsOperator {
 public:
  DsRotationUpdate();

  struct RotationUpdate {
    glm::quat new_rotation = glm::vec3(0.f);
    uint32_t segment_index = 0;
    uint32_t padding0 = 0;
    uint32_t padding1 = 0;
    uint32_t padding2 = 0;
  };

  inline static std::shared_ptr<DescriptorSetLayout> layout{};

  std::vector<RotationUpdate> commands;
  std::vector<std::shared_ptr<Buffer>> commands_buffer;

  struct RotationUpdatePushConstant {
    uint32_t commands_size = 0;
  };

  inline static std::shared_ptr<ComputePipeline> rotation_update_pipeline;
  std::vector<std::shared_ptr<DescriptorSet>> commands_descriptor_sets;

  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands) override;
};
class DsGravityForce final : public IDynamicStrandsOperator {
 public:
  struct GravityForcePushConstant {
    glm::vec3 acceleration;
    uint32_t particle_size = 0;
  };

  glm::vec3 gravity = glm::vec3(0, -9.81, 0);
  inline static std::shared_ptr<ComputePipeline> gravity_force_pipeline{};
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  DsGravityForce();
};
class DsExternalForce final : public IDynamicStrandsOperator {
 public:
  DsExternalForce();

  struct ExternalForce {
    glm::vec3 force = glm::vec3(0.f);
    uint32_t particle_index = 0;
  };

  inline static std::shared_ptr<DescriptorSetLayout> layout{};

  std::vector<ExternalForce> commands;
  std::vector<std::shared_ptr<Buffer>> commands_buffer;

  struct ExternalForcePushConstant {
    uint32_t commands_size = 0;
  };

  inline static std::shared_ptr<ComputePipeline> external_force_pipeline{};
  std::vector<std::shared_ptr<DescriptorSet>> commands_descriptor_sets{};

  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands) override;
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

  enum class ProjectMode {
    Bilateral,
    Forward,
    Backward,
    Balanced
  };

  uint32_t project_mode = static_cast<uint32_t>(ProjectMode::Forward);

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