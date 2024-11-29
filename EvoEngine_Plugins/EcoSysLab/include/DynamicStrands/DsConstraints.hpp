#pragma once
#include "DynamicStrands.hpp"
using namespace evo_engine;

namespace eco_sys_lab_plugin {
class DsPreStep {
 public:
  DsPreStep();

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
class DsPrediction {
 public:
  DsPrediction();

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
    uint32_t allow_breaking;
  };

  struct UniformParticlePredictionPushConstant {
    uint32_t uniform_particle_size = 0;
  };

  inline static std::shared_ptr<ComputePipeline> particle_prediction_pipeline;
  inline static std::shared_ptr<ComputePipeline> uniform_particle_prediction_pipeline;
  inline static std::shared_ptr<ComputePipeline> segment_prediction_pipeline;
  inline static std::shared_ptr<ComputePipeline> connection_prediction_pipeline;
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands);
};

class DsVelocityUpdate {
 public:
  struct ParticlePushConstant {
    uint32_t particle_size = 0;
    float time_step = 0.01f;
    float inv_time_step = 100.f;
  };

  struct SegmentPushConstant {
    uint32_t segment_size = 0;
    float time_step = 0.01f;
    float inv_time_step = 100.f;
  };

  DsVelocityUpdate();

  inline static std::shared_ptr<ComputePipeline> particle_pipeline;
  inline static std::shared_ptr<ComputePipeline> segment_pipeline;

  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands);
};

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

class DsGroundPlane final : public IDsConstraint {
 public:
  struct GroundPlanePushConstant {
    uint32_t particle_size;
    float ground_height;
    float ground_softness;
    float ground_friction;
  };

  float ground_height = -0.5f;
  float ground_softness = 0.95f;
  float ground_friction = 0.5f;
  inline static std::shared_ptr<ComputePipeline> pipeline{};
  void ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                 const DynamicStrands& target_dynamic_strands) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  DsGroundPlane();
};

class DsStiffRod final : public IDsConstraint {
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
                      const StrandModelSkeleton& strand_model_skeleton, const DtsStrandGroup& subdivided_strand_group,
                      const DynamicStrands& target_dynamic_strands) override;

  void ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                 const DynamicStrands& target_dynamic_strands) override;
  void DownloadData() override;
  void UploadData() override;
  void UpdateBindings() override;
  static glm::vec3 ComputeDarbouxVector(const glm::quat& q0, const glm::quat& q1, float average_segment_length);
};

#define BUNDLE_MAX_CONNECTION 16
class DsRandomBundle : public IDsConstraint {
 public:
  struct RandomBundleUpdateConstant {
    uint32_t pair_size = 0;
    uint32_t allow_breaking;
  };

  struct RandomBundleStretchShearConstant {
    uint32_t segment_size = 0;
    float inv_time_step = 0.0f;
  };

  struct RandomBundleConstant {
    uint32_t segment_size = 0;
    float inv_time_step = 0.0f;
    float over_relaxation;
  };

  struct RandomBundleBendTwistConstant {
    uint32_t segment_size = 0;
    float inv_time_step = 0.0f;
    float over_relaxation;
  };

  struct RandomBundleApplySegmentsConstant {
    uint32_t segment_size = 0;
    float inv_time_step = 0.0f;
  };

  struct RandomBundleApplyConnectionsConstant {
    uint32_t connection_size = 0;
    float inv_time_step = 0.0f;
  };
  struct SegmentPair {
    int segment0_handle;
    int segment1_handle;
    int valid;
    float max_strain;
    float bending_alpha;
    float twisting_alpha;
    float bundle_weight;
    float bend_twist_weight;

    glm::vec4 segment0_particle0_offset;
    glm::vec4 segment0_particle1_offset;

    glm::vec4 segment1_particle0_offset;
    glm::vec4 segment1_particle1_offset;

    glm::quat rest_darboux_vector;
  };

  struct SegmentData {
    glm::vec3 particle0_position_correction;
    float padding0;
    glm::vec3 particle1_position_correction;
    float padding1;

    glm::quat q_correction;

    int pair_handles[BUNDLE_MAX_CONNECTION];
  };

  float over_relaxation = 1.f;
  float bend_twist_over_relaxation = 1.f;

  std::vector<SegmentPair> segment_pairs;
  std::vector<SegmentData> segment_data_list;

  inline static std::shared_ptr<DescriptorSetLayout> layout{};
  std::shared_ptr<Buffer> pairs_buffer;
  std::shared_ptr<Buffer> segment_data_list_buffer;

  std::vector<std::shared_ptr<DescriptorSet>> bundle_descriptor_sets{};

  inline static std::shared_ptr<ComputePipeline> bundle_update_pipeline{};

  inline static std::shared_ptr<ComputePipeline> bundle_stretch_shear_offset_pipeline{};
  inline static std::shared_ptr<ComputePipeline> bundle_bend_twist_offset_pipeline{};
  inline static std::shared_ptr<ComputePipeline> bundle_offset_pipeline{};
  inline static std::shared_ptr<ComputePipeline> bundle_apply_segments_pipeline{};
  inline static std::shared_ptr<ComputePipeline> connections_correction_pipeline{};
  DsRandomBundle();
  int sub_iteration = 1;
  void ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                 const DynamicStrands& target_dynamic_strands) override;
  void InitializeData(const DynamicStrands::InitializeParameters& initialize_parameters,
                      const StrandModelSkeleton& strand_model_skeleton, const DtsStrandGroup& subdivided_strand_group,
                      const DynamicStrands& target_dynamic_strands) override;

  void UploadData() override;
  void UpdateBindings() override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};
#pragma endregion
}  // namespace eco_sys_lab_plugin