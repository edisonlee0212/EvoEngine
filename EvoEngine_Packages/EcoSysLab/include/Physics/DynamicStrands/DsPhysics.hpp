#pragma once
#include "DynamicStrands.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

/**
 * @class DsFungus
 * @brief Handles the fungal diffusion simulation on dynamic strands.
 */
class DsFungus {
 public:
  /**
   * @brief Constructor for DsFungus.
   */
  DsFungus();

  /**
   * @struct FungusDiffusionEdgePushConstant
   * @brief Stores push constants for fungal diffusion edge calculations.
   */
  struct FungusDiffusionEdgePushConstant {
    uint32_t pair_size = 0;  ///< Number of segment pairs.
    float be = 2.0f;
    float lignin_threshold = -1.0f;
    uint32_t global_parameter = 1;
    float HC_threshold = 0.4f;
    float HL_threshold = 0.4f;
    uint32_t treespace = 1;
    float padding1 = 0.0f;
    glm::mat4 matrixAw4;
    glm::mat4 matrixAb4;
    glm::mat4 matrixAc4;
    glm::mat4 matrixAm4;
  };

  /**
   * @struct FungusDiffusionNodePushConstant
   * @brief Stores push constants for fungal diffusion node calculations.
   */
  struct FungusDiffusionNodePushConstant {
    uint32_t segment_size = 0;  ///< Number of segments.
    float dt = 0.0005f;
    float aw = 5.0f;
    float ab = 5.0f;
    float bw = 2.0f;
    float bb = 2.0f;
    float ycw = 1.0f;
    float ycb = 1.0f;
    float ylw = 2.0f;
    float pc = 0.2f;
    float pl = 0.1f;
    float k = 5.0f;
    float delta = 0.05f;
    float ll = 0.5f;
    float lc = 0.5f;
    float bo = 1.0f;
    float kc = 0.2f;
    float brw = 0.5f;
    float brb = 0.5f;
    float msr = 0.15f;
    float bd_offset = 0.02f;
    float cpb = 1.0f;
    float cpw = 1.0f;
  };

  inline static std::shared_ptr<ComputePipeline>
      fungus_diffusion_edge_pipeline;  ///< Compute pipeline for fungal diffusion through edges.
  inline static std::shared_ptr<ComputePipeline>
      fungus_diffusion_node_pipeline;  ///< Compute pipeline for fungal diffusion at nodes.

  /**
   * @brief Inspects the object's properties in the editor.
   * @param editor_layer The editor layer used to inspect the object.
   * @return True if the asset content remains unmodified.
   */
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);

  /**
   * @brief Executes the fungal diffusion simulation on the strands.
   * @param physics_parameters The physics simulation parameters.
   * @param target_dynamic_strands The target strand system.
   */
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands);
};

/**
 * @class DsPreStep
 * @brief Handles the pre-step calculations for dynamic strands in GPU simulation.
 */
class DsPreStep {
 public:
  /**
   * @brief Constructor for DsPreStep.
   */
  DsPreStep();

  /**
   * @struct SegmentPreStepPushConstant
   * @brief Stores push constants for segment pre-step calculations.
   */
  struct SegmentPreStepPushConstant {
    glm::vec3 acceleration;       ///< Acceleration applied to the segments.
    uint32_t segment_size = 0;    ///< Number of segments.
    float time_step = 0.01f;      ///< Time step for the simulation.
    float inv_time_step = 100.f;  ///< Inverse of the time step.
  };

  /**
   * @struct LeafPreStepPushConstant
   * @brief Stores push constants for leaf pre-step calculations.
   */
  struct LeafPreStepPushConstant {
    glm::vec3 acceleration;       ///< Acceleration applied to the leaves.
    uint32_t leaf_size = 0;       ///< Number of leaves.
    float time_step = 0.01f;      ///< Time step for the simulation.
    float inv_time_step = 100.f;  ///< Inverse of the time step.
  };

  inline static std::shared_ptr<ComputePipeline> segment_pre_step_pipeline;  ///< Compute pipeline for segment pre-step.
  inline static std::shared_ptr<ComputePipeline> leaf_pre_step_pipeline;     ///< Compute pipeline for leaf pre-step.

  /**
   * @brief Executes the pre-step calculations on the strands.
   * @param physics_parameters The physics simulation parameters.
   * @param target_dynamic_strands The target strand system.
   */
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands);
};

/**
 * @class DsPrediction
 * @brief Handles the prediction step for dynamic strand simulation.
 */
class DsPrediction {
 public:
  /**
   * @brief Constructor for DsPrediction.
   */
  DsPrediction();

  /**
   * @struct SegmentPredictionPushConstant
   * @brief Stores push constants for segment prediction calculations.
   */
  struct SegmentPredictionPushConstant {
    uint32_t segment_size = 0;    ///< Number of segments.
    float time_step = 0.01f;      ///< Time step for simulation.
    float inv_time_step = 100.f;  ///< Inverse of the time step.
  };

  /**
   * @struct SegmentPairPredictionPushConstant
   * @brief Stores push constants for segment pair prediction calculations.
   */
  struct SegmentPairPredictionPushConstant {
    uint32_t pair_size = 0;           ///< Number of segment pairs.
    float time_step = 0.01f;          ///< Time step for simulation.
    float inv_time_step = 100.f;      ///< Inverse of the time step.
    float fungus_growth_rate = 0.1f;  ///< Ratio of fungus growth.
  };

  /**
   * @struct LeafPredictionPushConstant
   * @brief Stores push constants for leaf prediction calculations.
   */
  struct LeafPredictionPushConstant {
    uint32_t leaf_size = 0;       ///< Number of leaves.
    float time_step = 0.01f;      ///< Time step for simulation.
    float inv_time_step = 100.f;  ///< Inverse of the time step.
  };

  inline static std::shared_ptr<ComputePipeline>
      segment_prediction_pipeline;  ///< Compute pipeline for segment prediction.
  inline static std::shared_ptr<ComputePipeline>
      segment_pair_prediction_pipeline;  ///< Compute pipeline for segment pair prediction.
  inline static std::shared_ptr<ComputePipeline> leaf_prediction_pipeline;  ///< Compute pipeline for leaf prediction.

  /**
   * @brief Inspects the object's properties in the editor.
   * @param editor_layer The editor layer used to inspect the object.
   * @return True if the asset content remains unmodified.
   */
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);

  /**
   * @brief Executes the prediction step on the strands.
   * @param physics_parameters The physics simulation parameters.
   * @param target_dynamic_strands The target strand system.
   */
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands);
};

class DsStructuralDamage {
 public:
  DsStructuralDamage();

  struct SegmentPairBreakingPushConstant {
    uint32_t segment_pair_size = 0;
    uint32_t allow_disconnection;
    uint32_t allow_breaking;
    float compression_strength_factor;

    uint32_t tensile_disconnection;
    uint32_t compression_disconnection;
    uint32_t positional_breaking;
    uint32_t rotational_breaking;

    float rod_strength_factor;
    float bundle_strength_factor;
    float boundary_strength_decay_factor;
    uint32_t moisture_breaking_rod;

    uint32_t pull_cubical = 0;
  };

  struct LeafBreakingPushConstant {
    uint32_t leaf_size = 0;
    uint32_t leaf_break_from_moisture = 0;
    float leaf_break_threshold;
  };

  inline static std::shared_ptr<ComputePipeline> segment_pair_breaking_pipeline;
  inline static std::shared_ptr<ComputePipeline> leaf_breaking_pipeline;
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands);
};

class DsVelocityUpdate {
 public:
  struct SegmentPushConstant {
    glm::vec3 max_angular_velocity;
    float time_step = 0.01f;
    glm::vec3 max_velocity;
    float inv_time_step = 100.f;

    uint32_t segment_size = 0;
    float angular_velocity_damping;
    float velocity_damping;
  };

  struct LeafPushConstant {
    glm::vec3 max_angular_velocity;
    float time_step = 0.01f;
    glm::vec3 max_velocity;
    float inv_time_step = 100.f;

    uint32_t leaf_size = 0;
    float angular_velocity_damping;
    float velocity_damping;
  };

  DsVelocityUpdate();

  glm::vec3 max_angular_velocity = glm::vec3(1e6f);
  glm::vec3 max_velocity = glm::vec3(1e6f);

  inline static std::shared_ptr<ComputePipeline> segment_pipeline;
  inline static std::shared_ptr<ComputePipeline> leaf_pipeline;

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
  float grid_cell_size = 0.05f;

  std::unique_ptr<ComputePipeline> local_merge_sort_pipeline;
  std::unique_ptr<ComputePipeline> big_flip_pipeline;
  std::unique_ptr<ComputePipeline> local_disperse_pipeline;
  std::unique_ptr<ComputePipeline> global_disperse_pipeline;
  DsDynamicHashedGrid();
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
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

  struct CapsulePushConstant {
    uint32_t segment_size = 0;
    float grid_cell_size;
    float dt;
    float a_geom;
    float b_vel;
    float c_bias;
    float s_min;
    float s_max_ratio;
    float eta;
    float bmax_far;
  };

  uint32_t collision_mode = static_cast<uint32_t>(CollisionMode::Spherical);
  inline static std::shared_ptr<ComputePipeline> spherical_pipeline;
  DsSegmentCollision();
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands);
};

class DsSegmentCollisionPostStep {
 public:
  enum class CollisionMode { Spherical };

  struct SphericalPushConstant {
    uint32_t segment_size = 0;
    float dt;
  };

  uint32_t collision_mode = static_cast<uint32_t>(CollisionMode::Spherical);
  inline static std::shared_ptr<ComputePipeline> spherical_pipeline;
  DsSegmentCollisionPostStep();
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const DynamicStrands& target_dynamic_strands);
};
}  // namespace eco_sys_lab_package