#pragma once
#include "DynamicStrands.hpp"
using namespace evo_engine;

namespace eco_sys_lab_plugin {
class IDsPhysicsOperator {
 public:
  virtual ~IDsPhysicsOperator() = default;
  virtual void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                       const std::shared_ptr<DynamicStrands>& target_dynamic_strands) = 0;
  virtual bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
    return false;
  }
  bool enabled = true;
};

class DsLeafDrop final : public IDsPhysicsOperator {
 public:
  struct LeafDropPushConstant {
    uint32_t leaf_size = 0;
    float ground_height = 0.0f;
    float rotation_correction_strength;
    float air_resistance_strength;

    glm::vec3 disturbance_frequency;
    float disturbance_strength;
  };
  float ground_height = 0.03f;
  float rotation_correction_strength = 1.f;
  float air_resistance_strength = 0.8f;
  glm::vec3 disturbance_frequency = glm::vec3(0.1f);
  float disturbance_strength = 0.2f;
  inline static std::shared_ptr<ComputePipeline> pipeline{};
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const std::shared_ptr<DynamicStrands>& target_dynamic_strands) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  DsLeafDrop();
};

class DsAttraction final : public IDsPhysicsOperator {
 public:
  glm::vec3 target_position;
  float distance_multiplier = 0.5f;
  DsAttraction();

  inline static std::shared_ptr<DescriptorSetLayout> layout{};

  std::vector<int> commands;
  std::vector<std::shared_ptr<Buffer>> commands_buffer;

  struct AttractionPushConstant {
    glm::vec3 target_position;
    float distance_multiplier;
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

class IDsOperator {
 public:
  virtual ~IDsOperator() = default;
  virtual void Execute(const std::shared_ptr<DynamicStrands>& target_dynamic_strands) = 0;
  virtual bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
    return false;
  }
  bool enabled = true;
};

class DsBoxSelection : public IDsOperator {
 public:
  struct BoxSelectionPushConstant {
    glm::vec2 box_min;
    glm::vec2 box_max;
    glm::mat4 projection_view;
    uint32_t selection_mode;
    uint32_t segment_size;
  };

  inline static std::shared_ptr<ComputePipeline> pipeline{};
  BoxSelectionPushConstant push_constant;
  DsBoxSelection();
  void Update(const glm::vec2& box_start, const glm::vec2& box_end, const glm::mat4& projection_view,
              uint32_t selection_mode);
  void Execute(const std::shared_ptr<DynamicStrands>& target_dynamic_strands) override;
};

class DsDrag : public IDsPhysicsOperator {
 public:
  glm::vec3 target_acceleration;
  struct DragPushConstant {
    glm::vec3 acceleration;
    float padding;
    uint32_t segment_size;
  };
  inline static std::shared_ptr<ComputePipeline> pipeline{};

  DsDrag();
  void Update(const glm::vec3& acceleration);
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const std::shared_ptr<DynamicStrands>& target_dynamic_strands) override;
};

class DsLineCut : public IDsOperator {
 public:
  struct LineCutPushConstant {
    glm::vec2 line_start;
    glm::vec2 line_end;
    glm::mat4 projection_view;
    uint32_t segment_pair_size;
    uint32_t cut_mode = 0;
  };
  inline static std::shared_ptr<ComputePipeline> pipeline{};
  LineCutPushConstant push_constant;
  DsLineCut();
  void Update(const glm::vec2& line_start, const glm::vec2& line_end, const glm::mat4& projection_view,
              unsigned cut_mode);
  void Execute(const std::shared_ptr<DynamicStrands>& target_dynamic_strands) override;
};

class DsSaw : public IDsOperator {
 public:
  struct SawPushConstant {
    glm::mat4 projection_view;
    uint32_t segment_pair_size;
    uint32_t line_point_pair_size;
    uint32_t cut_mode = 0;
  };
  std::vector<glm::vec4> line_point_pairs;
  std::vector<std::shared_ptr<Buffer>> line_buffer;
  inline static std::shared_ptr<DescriptorSetLayout> layout{};
  std::vector<std::shared_ptr<DescriptorSet>> line_descriptor_sets{};

  inline static std::shared_ptr<ComputePipeline> pipeline{};
  SawPushConstant push_constant;
  DsSaw();
  void Update(const std::vector<glm::vec2>& line, const glm::mat4& projection_view, unsigned cut_mode);
  void Execute(const std::shared_ptr<DynamicStrands>& target_dynamic_strands) override;
};

}  // namespace eco_sys_lab_plugin