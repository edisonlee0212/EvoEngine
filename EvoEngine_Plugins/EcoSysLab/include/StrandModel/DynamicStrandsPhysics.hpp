#pragma once
#include "DynamicStrands.hpp"
using namespace evo_engine;

namespace eco_sys_lab_plugin {
class DynamicStrandsPreStep {
 public:
  DynamicStrandsPreStep();

  struct PreStepPushConstant {
    uint32_t segment_size = 0;
    float time_step = 0.01f;
    float inv_time_step = 100.f;
  };

  inline static std::shared_ptr<ComputePipeline> pre_step_pipeline;
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands);
};

class DynamicStrandsPostStep {
 public:
  DynamicStrandsPostStep();

  struct PostStepPushConstant {
    uint32_t segment_size = 0;
  };

  inline static std::shared_ptr<ComputePipeline> post_step_pipeline;
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
};
class IDynamicStrandsConstraint {
 public:
  virtual void InitializeData(const DynamicStrands::InitializeParameters& initialize_parameters,
                              const DynamicStrands& target_dynamic_strands){};
  virtual void Project(const DynamicStrands::PhysicsParameters& physics_parameters,
                       const DynamicStrands& target_dynamic_strands) = 0;

  virtual void DownloadData() {
  }
};

#pragma region Operators
class DsPositionUpdate final : public IDynamicStrandsOperator {
 public:
  DsPositionUpdate();

  struct PositionUpdate {
    glm::vec3 new_position = glm::vec3(0.f);
    uint32_t strand_segment_index = 0;
  };

  inline static std::shared_ptr<DescriptorSetLayout> layout{};

  std::vector<PositionUpdate> commands;
  std::vector<std::shared_ptr<Buffer>> commands_buffer;

  struct PositionUpdatePushConstant {
    uint32_t commands_size = 0;
  };

  inline static std::shared_ptr<ComputePipeline> external_force_pipeline;
  std::vector<std::shared_ptr<DescriptorSet>> commands_descriptor_sets;

  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands) override;
};
class DsExternalForce final : public IDynamicStrandsOperator {
 public:
  DsExternalForce();

  struct ExternalForce {
    glm::vec3 force = glm::vec3(0.f);
    uint32_t strand_segment_index = 0;
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

  struct RodProperties {
    float density;
    float i1;
    float j;
    float e;

    float g;
    float padding0;
    float padding1;
    float padding2;
  };

  struct GpuRodConstraint {
    glm::vec3 stiffness_coefficient_k;
    int segment0_index;

    glm::vec3 rest_darboux_vector;
    int segment1_index;

    glm::vec3 stretch_compliance;
    float average_segment_length;

    glm::vec3 bending_and_torsion_compliance;
    int next_handle = -1;

    /**
     * \brief
     * constraintInfo contains
     * 0: connector in segment 0 (local)
     * 1: connector in segment 1 (local)
     * 2: connector in segment 0 (global)
     * 3: connector in segment 1 (global)
     */
    glm::mat4 constraint_info;

    float lambda_sum[8];
  };

  struct PerStrandData {
    int begin_rod_constraint_handle = -1;
    int end_rod_constraint_handle = -1;
    int padding0 = -1;
    int padding1 = -1;
  };

  RodProperties rod_properties;

  std::vector<PerStrandData> per_strand_data_list;
  std::vector<GpuRodConstraint> rod_constraints;

  std::vector<std::shared_ptr<Buffer>> rod_properties_buffer;

  std::shared_ptr<Buffer> per_strand_data_list_buffer;
  std::shared_ptr<Buffer> rod_constraints_buffer;

  struct StiffRodInitConstraintConstant {
    uint32_t constraint_size = 0;
    float time_step;
    float inv_time_step;
  };

  struct StiffRodUpdateConstraintConstant {
    uint32_t constraint_size = 0;
  };

  struct StiffRodProjectConstraintConstant {
    uint32_t strand_size = 0;
  };

  inline static std::shared_ptr<ComputePipeline> init_constraint_pipeline{};
  inline static std::shared_ptr<ComputePipeline> update_constraint_pipeline{};
  inline static std::shared_ptr<ComputePipeline> project_constraint_pipeline{};

  std::vector<std::shared_ptr<DescriptorSet>> strands_physics_descriptor_sets{};

  void InitializeData(const DynamicStrands::InitializeParameters& initialize_parameters,
                      const DynamicStrands& target_dynamic_strands) override;

  void Project(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands) override;
  void DownloadData() override;
  static glm::vec3 ComputeDarbouxVector(const glm::quat& q0, const glm::quat& q1, float average_segment_length);
};
#pragma endregion
}  // namespace eco_sys_lab_plugin