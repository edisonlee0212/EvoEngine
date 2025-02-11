
#pragma once
#include "DynamicStrands.hpp"
using namespace evo_engine;

namespace eco_sys_lab_plugin {

/**
 * @class IDsPhysicsOperator
 * @brief Abstract base class for physics operators applied to DynamicStrands.
 */
class IDsPhysicsOperator {
 public:
  /**
   * @brief Virtual destructor.
   */
  virtual ~IDsPhysicsOperator() = default;

  /**
   * @brief Executes a physics operation on the provided DynamicStrands.
   * @param physics_parameters The physics parameters controlling the operation.
   * @param target_dynamic_strands The target DynamicStrands to apply the operation.
   */
  virtual void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                       const std::shared_ptr<DynamicStrands>& target_dynamic_strands) = 0;

  /**
   * @brief Inspects and potentially modifies the operator in the editor.
   * @param editor_layer The current editor layer.
   * @return True if the asset content is unmodified, false otherwise.
   */
  virtual bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
    return false;
  }

  /**
   * @brief Indicates whether the operator is enabled.
   */
  bool enabled = true;
};

/**
 * @class DsLeafDrop
 * @brief Simulates leaf detachment and falling effects.
 */
class DsLeafDrop final : public IDsPhysicsOperator {
 public:
  /**
   * @struct LeafDropPushConstant
   * @brief Push constant structure for leaf drop simulation.
   */
  struct LeafDropPushConstant {
    uint32_t leaf_size = 0;              ///< Number of leaves.
    float ground_height = 0.0f;          ///< Ground collision height.
    float rotation_correction_strength;  ///< Rotation correction force.
    float air_resistance_strength;       ///< Air resistance force.

    glm::vec3 disturbance_frequency;  ///< Frequency of disturbances.
    float disturbance_strength;       ///< Strength of disturbances.
  };

  float ground_height = 0.03f;                        ///< Default ground height where leaves settle.
  float rotation_correction_strength = 1.f;           ///< Default rotation correction strength.
  float air_resistance_strength = 0.8f;               ///< Default air resistance strength.
  glm::vec3 disturbance_frequency = glm::vec3(0.1f);  ///< Default disturbance frequency.
  float disturbance_strength = 0.2f;                  ///< Default disturbance strength.

  inline static std::shared_ptr<ComputePipeline> pipeline{};  ///< Compute pipeline for leaf drop.

  /**
   * @brief Executes the leaf drop physics simulation.
   * @param physics_parameters The physics parameters controlling the operation.
   * @param target_dynamic_strands The target DynamicStrands.
   */
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const std::shared_ptr<DynamicStrands>& target_dynamic_strands) override;

  /**
   * @brief Inspects parameters in the editor.
   * @param editor_layer The editor layer.
   * @return True if content is unchanged.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Constructor for setting default values.
   */
  DsLeafDrop();
};

/**
 * @class DsAttraction
 * @brief Simulates attraction forces towards a target position.
 */
class DsAttraction final : public IDsPhysicsOperator {
 public:
  glm::vec3 target_position;         ///< Target position for attraction.
  float distance_multiplier = 0.5f;  ///< Strength of attraction force.

  /**
   * @brief Constructor initializing the attraction operator.
   */
  DsAttraction();

  inline static std::shared_ptr<DescriptorSetLayout> layout{};  ///< Descriptor set layout for GPU computations.

  std::vector<int> commands;                             ///< Attraction commands for applying forces.
  std::vector<std::shared_ptr<Buffer>> commands_buffer;  ///< GPU buffers storing attraction commands.

  /**
   * @struct AttractionPushConstant
   * @brief Push constant for attraction forces.
   */
  struct AttractionPushConstant {
    glm::vec3 target_position;   ///< Target position affecting attraction.
    float distance_multiplier;   ///< Multiplier for controlling attraction strength.
    uint32_t commands_size = 0;  ///< Size of the attraction command list.
  };

  inline static std::shared_ptr<ComputePipeline>
      drag_force_pipeline{};  ///< Compute pipeline for drag force calculation.
  std::vector<std::shared_ptr<DescriptorSet>> commands_descriptor_sets{};  ///< Descriptor sets for commands.

  /**
   * @brief Initializes attraction forces for dynamic strands.
   * @param particle_handles The indices of particles being affected.
   */
  void Initialize(const std::vector<int>& particle_handles);

  /**
   * @brief Updates target position dynamically.
   * @param new_position The new target position.
   */
  void Update(const glm::vec3& new_position);

  /**
   * @brief Executes the attraction force simulation.
   * @param physics_parameters The physics parameters controlling the operation.
   * @param target_dynamic_strands The target DynamicStrands to affect.
   */
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const std::shared_ptr<DynamicStrands>& target_dynamic_strands) override;

  /**
   * @brief Inspects parameters in the editor.
   * @param editor_layer The editor layer.
   * @return True if content is unchanged.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};

/**
 * @class IDsOperator
 * @brief Abstract base class for non-physics-based operators applied to DynamicStrands.
 */
class IDsOperator {
 public:
  /**
   * @brief Virtual destructor.
   */
  virtual ~IDsOperator() = default;

  /**
   * @brief Executes an operation on the provided DynamicStrands.
   * @param target_dynamic_strands The target DynamicStrands to apply the operation.
   */
  virtual void Execute(const std::shared_ptr<DynamicStrands>& target_dynamic_strands) = 0;

  /**
   * @brief Inspects and potentially modifies the operator in the editor.
   * @param editor_layer The current editor layer.
   * @return True if the asset content is unmodified, false otherwise.
   */
  virtual bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
    return false;
  }

  /**
   * @brief Indicates whether the operator is enabled.
   */
  bool enabled = true;
};

/**
 * @class DsBoxSelection
 * @brief Implements box-based selection for segments and leaves.
 */
class DsBoxSelection : public IDsOperator {
 public:
  /**
   * @struct SegmentBoxSelectionPushConstant
   * @brief Push constant structure for segment selection.
   */
  struct SegmentBoxSelectionPushConstant {
    glm::vec2 box_min;          ///< Minimum bounds of the selection box.
    glm::vec2 box_max;          ///< Maximum bounds of the selection box.
    glm::mat4 projection_view;  ///< Projection-view matrix for screen-space calculations.
    uint32_t selection_mode;    ///< Selection mode for identifying segments.
    uint32_t segment_size;      ///< Number of segments to consider.
  };

  /**
   * @struct LeafBoxSelectionPushConstant
   * @brief Push constant structure for leaf selection.
   */
  struct LeafBoxSelectionPushConstant {
    glm::vec2 box_min;          ///< Minimum bounds of the selection box.
    glm::vec2 box_max;          ///< Maximum bounds of the selection box.
    glm::mat4 projection_view;  ///< Projection-view matrix for screen-space calculations.
    uint32_t selection_mode;    ///< Selection mode for identifying leaves.
    uint32_t leaf_size;         ///< Number of leaves to consider.
  };

  inline static std::shared_ptr<ComputePipeline> segment_pipeline{};  ///< GPU pipeline for segment selection.
  SegmentBoxSelectionPushConstant segment_push_constant;              ///< Push constant for segment selection.

  inline static std::shared_ptr<ComputePipeline> leaf_pipeline{};  ///< GPU pipeline for leaf selection.
  LeafBoxSelectionPushConstant leaf_push_constant;                 ///< Push constant for leaf selection.

  /**
   * @brief Constructor initializing default selection values.
   */
  DsBoxSelection();

  /**
   * @brief Updates selection box parameters.
   * @param box_start The starting position of the selection box.
   * @param box_end The ending position of the selection box.
   * @param projection_view The projection-view matrix for rendering.
   * @param selection_mode The selection mode being applied.
   */
  void Update(const glm::vec2& box_start, const glm::vec2& box_end, const glm::mat4& projection_view,
              uint32_t selection_mode);

  /**
   * @brief Executes the selection operation on the strands.
   * @param target_dynamic_strands The target DynamicStrands.
   */
  void Execute(const std::shared_ptr<DynamicStrands>& target_dynamic_strands) override;
};

/**
 * @class DsDrag
 * @brief Applies drag forces to DynamicStrands.
 */
class DsDrag : public IDsPhysicsOperator {
 public:
  glm::vec3 target_acceleration;  ///< Acceleration applied to the strands.

  /**
   * @struct DragPushConstant
   * @brief Push constant structure for drag force.
   */
  struct DragPushConstant {
    glm::vec3 acceleration;  ///< Drag acceleration applied.
    float padding;           ///< Padding for memory alignment.
    uint32_t segment_size;   ///< Number of segments affected.
  };

  inline static std::shared_ptr<ComputePipeline> pipeline{};  ///< Compute pipeline for drag force calculation.

  /**
   * @brief Constructor initializing default drag values.
   */
  DsDrag();

  /**
   * @brief Updates the drag acceleration.
   * @param acceleration The new acceleration value.
   */
  void Update(const glm::vec3& acceleration);

  /**
   * @brief Executes the drag force simulation.
   * @param physics_parameters The physics parameters controlling the operation.
   * @param target_dynamic_strands The target DynamicStrands.
   */
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const std::shared_ptr<DynamicStrands>& target_dynamic_strands) override;
};

/**
 * @class DsLineCut
 * @brief Implements line-based cutting of DynamicStrands.
 */
class DsLineCut : public IDsOperator {
 public:
  /**
   * @struct LineCutPushConstant
   * @brief Push constant structure for line cutting.
   */
  struct LineCutPushConstant {
    glm::vec2 line_start;        ///< Start position of the cutting line.
    glm::vec2 line_end;          ///< End position of the cutting line.
    glm::mat4 projection_view;   ///< Projection-view matrix for rendering.
    uint32_t segment_pair_size;  ///< Number of segment pairs being cut.
    uint32_t cut_mode = 0;       ///< Mode defining how the cut is applied.
  };

  inline static std::shared_ptr<ComputePipeline> pipeline{};  ///< Compute pipeline for line cut operation.
  LineCutPushConstant push_constant;                          ///< Push constant controlling the line cut operation.

  /**
   * @brief Constructor initializing default line cut values.
   */
  DsLineCut();

  /**
   * @brief Updates the line cut parameters.
   * @param line_start The starting position of the cutting line.
   * @param line_end The ending position of the cutting line.
   * @param projection_view The projection-view matrix.
   * @param cut_mode The mode defining how the cut is applied.
   */
  void Update(const glm::vec2& line_start, const glm::vec2& line_end, const glm::mat4& projection_view,
              unsigned cut_mode);

  /**
   * @brief Executes the line cutting operation on the strands.
   * @param target_dynamic_strands The target DynamicStrands.
   */
  void Execute(const std::shared_ptr<DynamicStrands>& target_dynamic_strands) override;
};

/**
 * @class DsPointCut
 * @brief Implements point-based cutting of DynamicStrands.
 */
class DsPointCut : public IDsOperator {
 public:
  /**
   * @struct PointCutPushConstant
   * @brief Push constant structure for point cutting.
   */
  struct PointCutPushConstant {
    glm::mat4 projection_view;  ///< Projection-view matrix for rendering.
    glm::vec2 point;            ///< Screen-space position for cutting.
    glm::vec2 screen_size;      ///< Screen resolution.

    uint32_t segment_pair_size;  ///< Number of segment pairs affected by the cut.
    uint32_t cut_mode = 0;       ///< Mode defining how the cut is applied.
    float point_size;            ///< Size of the point affecting the cut.
  };

  inline static std::shared_ptr<ComputePipeline> pipeline{};  ///< Compute pipeline for point cutting.
  PointCutPushConstant push_constant;                         ///< Push constant controlling the point cut operation.

  /**
   * @brief Constructor initializing default point cut values.
   */
  DsPointCut();

  /**
   * @brief Updates the point cut parameters.
   * @param point The screen-space position of the cutting point.
   * @param screen_size The dimensions of the screen.
   * @param point_size The size of the point affecting the cut.
   * @param projection_view The projection-view matrix.
   * @param cut_mode The mode that determines how the cut is applied.
   */
  void Update(const glm::vec2& point, const glm::vec2& screen_size, float point_size, const glm::mat4& projection_view,
              unsigned cut_mode);

  /**
   * @brief Executes the point cutting operation on the strands.
   * @param target_dynamic_strands The target DynamicStrands.
   */
  void Execute(const std::shared_ptr<DynamicStrands>& target_dynamic_strands) override;
};

/**
 * @class DsSaw
 * @brief Implements a saw-like cutting operation on DynamicStrands.
 */
class DsSaw : public IDsOperator {
 public:
  /**
   * @struct SawPushConstant
   * @brief Push constant structure for saw-based cutting.
   */
  struct SawPushConstant {
    glm::mat4 projection_view;      ///< Projection-view matrix for rendering.
    uint32_t segment_pair_size;     ///< Number of segment pairs considered for cutting.
    uint32_t line_point_pair_size;  ///< Number of line-point pairs for determining the cut.
    uint32_t cut_mode = 0;          ///< Mode defining how the cut is applied.
  };

  std::vector<glm::vec4> line_point_pairs;           ///< Collection of line-point pairs used in cutting.
  std::vector<std::shared_ptr<Buffer>> line_buffer;  ///< GPU buffer for storing line data.

  inline static std::shared_ptr<DescriptorSetLayout> layout{};         ///< Descriptor set layout for GPU processing.
  std::vector<std::shared_ptr<DescriptorSet>> line_descriptor_sets{};  ///< Descriptor sets for line processing.

  inline static std::shared_ptr<ComputePipeline> pipeline{};  ///< Compute pipeline for saw-based cutting.
  SawPushConstant push_constant;                              ///< Push constant controlling the saw operation.

  /**
   * @brief Constructor initializing default saw cutting values.
   */
  DsSaw();

  /**
   * @brief Updates the saw cut parameters.
   * @param line The collection of screen-space line positions.
   * @param projection_view The projection-view matrix.
   * @param cut_mode The mode defining how the cut is applied.
   */
  void Update(const std::vector<glm::vec2>& line, const glm::mat4& projection_view, unsigned cut_mode);

  /**
   * @brief Executes the saw cutting operation on the strands.
   * @param target_dynamic_strands The target DynamicStrands.
   */
  void Execute(const std::shared_ptr<DynamicStrands>& target_dynamic_strands) override;
};

/**
 * @class DsSnow
 * @brief Simulates the accumulation and retention of snow on DynamicStrands.
 */
class DsSnow : public IDsPhysicsOperator {
 public:
  /**
   * @struct SegmentPushConstant
   * @brief Push constant structure for snow accumulation on segments.
   */
  struct SegmentPushConstant {
    uint32_t segment_size;    ///< Number of segments affected.
    float snow_intensity;     ///< Intensity of the snow accumulation.
    float snow_retain_ratio;  ///< Ratio of retained snow over time.
  };

  inline static std::shared_ptr<ComputePipeline>
      segment_pipeline{};  ///< Compute pipeline for segment-based snow accumulation.

  /**
   * @struct LeafPushConstant
   * @brief Push constant structure for snow accumulation on leaves.
   */
  struct LeafPushConstant {
    uint32_t leaf_size;       ///< Number of leaves affected.
    float snow_intensity;     ///< Intensity of the snow accumulation.
    float snow_retain_ratio;  ///< Ratio of retained snow over time.
  };

  inline static std::shared_ptr<ComputePipeline>
      leaf_pipeline{};  ///< Compute pipeline for leaf-based snow accumulation.

  float snow_intensity = 0.000f;   ///< Default intensity of snowfall.
  float snow_retain_ratio = 0.2f;  ///< Default retention ratio of accumulated snow.

  /**
   * @brief Constructor initializing default snow effect values.
   */
  DsSnow();

  /**
   * @brief Executes the snow accumulation simulation on strands.
   * @param physics_parameters The physics parameters controlling the operation.
   * @param target_dynamic_strands The target DynamicStrands.
   */
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const std::shared_ptr<DynamicStrands>& target_dynamic_strands) override;

  /**
   * @brief Inspects parameters in the editor.
   * @param editor_layer The editor layer.
   * @return True if content is unchanged.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};

/**
 * @class DsWind
 * @brief Simulates wind forces acting on DynamicStrands.
 */
class DsWind : public IDsPhysicsOperator {
 public:
  /**
   * @struct SegmentPushConstant
   * @brief Push constant structure for wind simulation on segments.
   */
  struct SegmentPushConstant {
    glm::vec3 main_force;   ///< Main directional force of the wind.
    uint32_t segment_size;  ///< Number of segments affected.

    float turbulence_direction_frequency;  ///< Frequency of directional turbulence.
    float turbulence_speed_frequency;      ///< Frequency of speed turbulence.
    float turbulence_strength;             ///< Strength of turbulence.
    float simulated_time;                  ///< Time factor for animated turbulence.
  };

  /**
   * @struct LeafPushConstant
   * @brief Push constant structure for wind simulation on leaves.
   */
  struct LeafPushConstant {
    glm::vec3 main_force;  ///< Main directional force of the wind.
    uint32_t leaf_size;    ///< Number of leaves affected.

    float turbulence_direction_frequency;  ///< Frequency of directional turbulence.
    float turbulence_speed_frequency;      ///< Frequency of speed turbulence.
    float turbulence_strength;             ///< Strength of turbulence.
    float simulated_time;                  ///< Time factor for animated turbulence.
  };

  glm::vec3 main_force = glm::vec3(0.02f, 0.f, 0.f);  ///< Default main wind force.
  float turbulence_strength = 0.5f;                   ///< Default turbulence strength.
  float turbulence_direction_frequency = 100.f;       ///< Default frequency for directional turbulence.
  float turbulence_speed_frequency = 100.f;           ///< Default frequency for speed turbulence.

  inline static std::shared_ptr<ComputePipeline> segment_pipeline{};  ///< Compute pipeline for segment wind simulation.
  inline static std::shared_ptr<ComputePipeline> leaf_pipeline{};     ///< Compute pipeline for leaf wind simulation.

  /**
   * @brief Constructor initializing default wind effect values.
   */
  DsWind();

  /**
   * @brief Executes the wind simulation on strands.
   * @param physics_parameters The physics parameters controlling the operation.
   * @param target_dynamic_strands The target DynamicStrands.
   */
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const std::shared_ptr<DynamicStrands>& target_dynamic_strands) override;

  /**
   * @brief Inspects parameters in the editor.
   * @param editor_layer The editor layer.
   * @return True if content is unchanged.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};

/**
 * @class DsStopAll
 * @brief Stops all dynamic movements in the DynamicStrands simulation.
 */
class DsStopAll : public IDsPhysicsOperator {
  /**
   * @struct SegmentPushConstant
   * @brief Push constant structure for stopping motion in segments.
   */
  struct SegmentPushConstant {
    uint32_t segment_size;  ///< Number of segments affected.
  };

  /**
   * @struct LeafPushConstant
   * @brief Push constant structure for stopping motion in leaves.
   */
  struct LeafPushConstant {
    uint32_t leaf_size;  ///< Number of leaves affected.
  };

  inline static std::shared_ptr<ComputePipeline> segment_pipeline{};  ///< Compute pipeline for stopping segment motion.
  inline static std::shared_ptr<ComputePipeline> leaf_pipeline{};     ///< Compute pipeline for stopping leaf motion.

 public:
  /**
   * @brief Constructor initializing default stop values.
   */
  DsStopAll();

  /**
   * @brief Executes the stop operation on strands, freezing motion instantly.
   * @param physics_parameters The physics parameters controlling the operation.
   * @param target_dynamic_strands The target DynamicStrands.
   */
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const std::shared_ptr<DynamicStrands>& target_dynamic_strands) override;
};

}  // namespace eco_sys_lab_plugin
