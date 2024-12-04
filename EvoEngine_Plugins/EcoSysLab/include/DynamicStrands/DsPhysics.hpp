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
    float velocity_damping;
  };

  struct SegmentPredictionPushConstant {
    uint32_t segment_size = 0;
    float time_step = 0.01f;
    float inv_time_step = 100.f;
    float angular_velocity_damping;
  };

  struct ConnectionPredictionPushConstant {
    uint32_t connection_size = 0;
    uint32_t allow_breaking;
  };
  struct SegmentPairPredictionPushConstant {
    uint32_t segment_pair_size = 0;
    uint32_t allow_breaking;
  };
  struct UniformParticlePredictionPushConstant {
    uint32_t uniform_particle_size = 0;
  };

  inline static std::shared_ptr<ComputePipeline> particle_prediction_pipeline;
  inline static std::shared_ptr<ComputePipeline> uniform_particle_prediction_pipeline;
  inline static std::shared_ptr<ComputePipeline> segment_prediction_pipeline;
  inline static std::shared_ptr<ComputePipeline> connection_prediction_pipeline;
  inline static std::shared_ptr<ComputePipeline> segment_pair_prediction_pipeline;
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
class DsHashedGrid {
public:
  struct PartitionPushConstant {
    uint32_t segment_size = 0;
    float grid_size;
  };

  inline static std::shared_ptr<ComputePipeline> partition_pipeline;

  float grid_size = 0.05f;

  DsHashedGrid();

  void Initialize(const DynamicStrands::PhysicsParameters& physics_parameters,
                 const DynamicStrands& target_dynamic_strands);
};

}  // namespace eco_sys_lab_plugin