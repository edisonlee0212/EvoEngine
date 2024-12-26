#pragma once
#include "DynamicStrands.hpp"
using namespace evo_engine;

namespace eco_sys_lab_plugin {
class DsPreStep {
 public:
  DsPreStep();

  struct SegmentPreStepPushConstant {
    uint32_t segment_size = 0;
    float time_step = 0.01f;
    float inv_time_step = 100.f;
  };

  inline static std::shared_ptr<ComputePipeline> segment_pre_step_pipeline;
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands);
};
class DsPrediction {
 public:
  DsPrediction();

  struct SegmentPredictionPushConstant {
    uint32_t segment_size = 0;
    float time_step = 0.01f;
    float inv_time_step = 100.f;
    float angular_velocity_damping;
    float velocity_damping;
  };

  struct SegmentPairPredictionPushConstant {
    uint32_t segment_pair_size = 0;
    uint32_t allow_disconnection;
    uint32_t allow_breaking;
  };
  struct UniformParticlePredictionPushConstant {
    uint32_t uniform_particle_size = 0;
  };
  struct LeafPredictionPushConstant {
    uint32_t leaf_size = 0;
    float time_step = 0.01f;
    float inv_time_step = 100.f;
    float angular_velocity_damping;
    float velocity_damping;
  };
  inline static std::shared_ptr<ComputePipeline> uniform_particle_prediction_pipeline;
  inline static std::shared_ptr<ComputePipeline> segment_prediction_pipeline;
  inline static std::shared_ptr<ComputePipeline> segment_pair_prediction_pipeline;

  inline static std::shared_ptr<ComputePipeline> leaf_prediction_pipeline;
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands);
};

class DsVelocityUpdate {
 public:
  struct SegmentPushConstant {
    uint32_t segment_size = 0;
    float time_step = 0.01f;
    float inv_time_step = 100.f;
  };

  DsVelocityUpdate();

  inline static std::shared_ptr<ComputePipeline> segment_pipeline;

  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands);
};
class DsDynamicHashedGrid {
 public:
  struct PartitionPushConstant {
    uint32_t segment_size = 0;
    float grid_cell_size;
  };

  struct SortPushConstant {
    uint32_t segment_size = 0;
    uint32_t segment_group_size = 0;
  };

  struct OffsetPushConstant {
    uint32_t segment_size = 0;
  };

  inline static std::shared_ptr<ComputePipeline> partition_pipeline;
  inline static std::shared_ptr<ComputePipeline> offset_pipeline;

  inline static std::shared_ptr<Shader> local_merge_sort_shader;
  inline static std::shared_ptr<Shader> big_flip_shader;
  inline static std::shared_ptr<Shader> local_disperse_shader;
  inline static std::shared_ptr<Shader> global_disperse_shader;
  float grid_cell_size = 0.1f;

  std::unique_ptr<ComputePipeline> local_merge_sort_pipeline;
  std::unique_ptr<ComputePipeline> big_flip_pipeline;
  std::unique_ptr<ComputePipeline> local_disperse_pipeline;
  std::unique_ptr<ComputePipeline> global_disperse_pipeline;
  DsDynamicHashedGrid();
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  void BuildGrid(const DynamicStrands::PhysicsParameters& physics_parameters,
                 const DynamicStrands& target_dynamic_strands);
};

class DsSegmentCollision {
 public:
  enum class CollisionMode { Spherical };
  struct SphericalPushConstant {
    uint32_t segment_size = 0;
    float grid_cell_size;
  };
  uint32_t collision_mode = static_cast<uint32_t>(CollisionMode::Spherical);
  inline static std::shared_ptr<ComputePipeline> spherical_pipeline;
  DsSegmentCollision();
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands);
};

}  // namespace eco_sys_lab_plugin