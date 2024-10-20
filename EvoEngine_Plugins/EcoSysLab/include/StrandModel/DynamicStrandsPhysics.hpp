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

class IDynamicStrandsOperator {
 public:
  virtual ~IDynamicStrandsOperator() = default;
  virtual void InitializeData(const DynamicStrands::InitializeParameters& initialize_parameters,
                              const DynamicStrands& target_dynamic_strands) {
  }
  virtual void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                       const DynamicStrands& target_dynamic_strands) = 0;
  virtual void DownloadData() {
  }
  virtual void UploadData() {
  }

  bool enabled = false;
};
class IDynamicStrandsConstraint {
 public:
  virtual void InitializeData(const DynamicStrands::InitializeParameters& initialize_parameters,
                              const DynamicStrands& target_dynamic_strands){};
  virtual void Project(const DynamicStrands::PhysicsParameters& physics_parameters,
                       const DynamicStrands& target_dynamic_strands) = 0;

  virtual void DownloadData() {
  }
  virtual void UploadData() {
  }

  bool enabled = false;
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
    int front_propagate_begin_constraint_handle = -1;
    int back_propagate_begin_constraint_handle = -1;
    int front_propagate_begin_segment_handle = -1;
    int back_propagate_begin_segment_handle = -1;
  };

  struct GpuRodConstraint {
    glm::quat rest_darboux_vector;

    int segment0_index = -1;
    int segment1_index = -1;
    int padding = -1;
    float average_segment_length;

    float bending_stiffness;
    float twisting_stiffness;
    int next_constraint_handle = -1;
    int prev_constraint_handle = -1;
  };

  std::vector<PerStrandData> per_strand_data_list;
  std::vector<GpuRodConstraint> rod_constraints;

  std::shared_ptr<Buffer> per_strand_data_list_buffer;
  std::shared_ptr<Buffer> rod_constraints_buffer;

  struct StretchShearConstraintConstant {
    uint32_t strand_size = 0;
  };

  struct BendTwistConstraintConstant {
    uint32_t strand_size = 0;
  };
  uint32_t sub_iteration = 50;
  inline static std::shared_ptr<ComputePipeline> stretch_shear_constraint_pipeline{};
  inline static std::shared_ptr<ComputePipeline> bend_twist_constraint_pipeline{};

  std::vector<std::shared_ptr<DescriptorSet>> strands_physics_descriptor_sets{};

  void InitializeData(const DynamicStrands::InitializeParameters& initialize_parameters,
                      const DynamicStrands& target_dynamic_strands) override;

  void Project(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands) override;
  void DownloadData() override;
  void UploadData() override;
  static glm::vec3 ComputeDarbouxVector(const glm::quat& q0, const glm::quat& q1, float average_segment_length);
};

class DsConnectivity : public IDynamicStrandsConstraint {
 public:
  struct Connectivity {
    glm::vec3 new_position;
    float padding;
    int neighbors[8];
    float distances[8];
  };
  DsConnectivity();

  inline static std::shared_ptr<DescriptorSetLayout> layout{};
  struct ConnectivityConstraintConstant {
    uint32_t particle_size = 0;
  };
  std::vector<Connectivity> connectivity_list;
  std::shared_ptr<Buffer> connectivity_list_buffer;
  inline static std::shared_ptr<ComputePipeline> connectivity_offset_pipeline{};
  inline static std::shared_ptr<ComputePipeline> connectivity_apply_pipeline{};
  std::vector<std::shared_ptr<DescriptorSet>> connectivity_list_descriptor_sets{};
  uint32_t sub_iteration = 50;
  void InitializeData(const DynamicStrands::InitializeParameters& initialize_parameters,
                      const DynamicStrands& target_dynamic_strands) override;

  void Project(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands) override;

  void UploadData() override;
};
#pragma endregion
}  // namespace eco_sys_lab_plugin