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
  };

  inline static std::shared_ptr<ComputePipeline> particle_prediction_pipeline;
  inline static std::shared_ptr<ComputePipeline> segment_prediction_pipeline;
  inline static std::shared_ptr<ComputePipeline> connection_prediction_pipeline;
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands);
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
  virtual void UpdateBindings() {
  }
  virtual bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
    return false;
  }
  bool enabled = true;
};

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
    float inv_time_step;
  };

  struct BendTwistConstraintConstant {
    uint32_t strand_size = 0;
    float inv_time_step;
  };

  enum class ProjectMode { Forward, Backward, Bilateral };

  uint32_t project_mode = static_cast<uint32_t>(ProjectMode::Backward);

  int sub_iteration = 1;
  bool bend_twist = true;
  bool stretch_shear = true;
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
  void UpdateBindings() override;
  static glm::vec3 ComputeDarbouxVector(const glm::quat& q0, const glm::quat& q1, float average_segment_length);
};

#define BUNDLE_MAX_CONNECTION 8
class DsBundle : public IDynamicStrandsConstraint {
 public:
  struct BundleUpdateConstant {
    uint32_t pair_size = 0;
  };

  struct BundleConstant {
    uint32_t segment_size = 0;
    float inv_time_step = 0.0f;
  };

  struct Pair {
    int handle0;
    int handle1;
    int valid;
    int padding;
    glm::vec4 stiffness;
  };
  struct SegmentData {
    glm::vec3 particle0_position_correction;
    float particle0_max_strain;
    glm::vec3 particle1_position_correction;
    float particle1_max_strain;

    glm::quat q_correction;

    int pair_handles[BUNDLE_MAX_CONNECTION];

    glm::vec4 particle0_offset[BUNDLE_MAX_CONNECTION];
    glm::vec4 particle1_offset[BUNDLE_MAX_CONNECTION];

    glm::quat rest_darboux_vectors[BUNDLE_MAX_CONNECTION];
  };

  std::vector<Pair> pairs;
  std::vector<SegmentData> segment_data_list;

  inline static std::shared_ptr<DescriptorSetLayout> layout{};
  std::shared_ptr<Buffer> pairs_buffer;
  std::shared_ptr<Buffer> segment_data_list_buffer;

  std::vector<std::shared_ptr<DescriptorSet>> bundle_descriptor_sets{};

  inline static std::shared_ptr<ComputePipeline> bundle_update_pipeline{};

  inline static std::shared_ptr<ComputePipeline> bundle_offset_pipeline{};
  inline static std::shared_ptr<ComputePipeline> bundle_apply_pipeline{};

  DsBundle();
  int sub_iteration = 5;
  void Project(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands) override;
  void InitializeData(const DynamicStrands::InitializeParameters& initialize_parameters,
                      const StrandModelSkeleton& strand_model_skeleton,
                      const DynamicStrands& target_dynamic_strands) override;

  void UploadData() override;
  void UpdateBindings() override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};
#pragma endregion
}  // namespace eco_sys_lab_plugin