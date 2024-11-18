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

class DsTransform final : public IDsPhysicsOperator {
 public:
  GlobalTransform inverse_base_global_transform{};
  GlobalTransform base_global_transform{};
  // Position
  struct PositionUpdate {
    glm::vec3 new_position = glm::vec3(0.f);
    uint32_t particle_index = 0;
  };
  inline static std::shared_ptr<DescriptorSetLayout> position_layout{};
  std::vector<PositionUpdate> position_commands;
  std::vector<std::shared_ptr<Buffer>> position_commands_buffer;
  struct PositionUpdatePushConstant {
    uint32_t commands_size = 0;
  };
  inline static std::shared_ptr<ComputePipeline> position_update_pipeline;
  std::vector<std::shared_ptr<DescriptorSet>> position_commands_descriptor_sets;
  // Rotation
  struct RotationUpdate {
    glm::quat new_rotation = glm::vec3(0.f);
    uint32_t segment_index = 0;
    uint32_t padding0 = 0;
    uint32_t padding1 = 0;
    uint32_t padding2 = 0;
  };
  inline static std::shared_ptr<DescriptorSetLayout> rotation_layout{};
  std::vector<RotationUpdate> rotation_commands;
  std::vector<std::shared_ptr<Buffer>> rotation_commands_buffer;

  struct RotationUpdatePushConstant {
    uint32_t commands_size = 0;
  };

  inline static std::shared_ptr<ComputePipeline> rotation_update_pipeline;
  std::vector<std::shared_ptr<DescriptorSet>> rotation_commands_descriptor_sets;

  DsTransform();
  void Initialize(const GlobalTransform& target_base_global_transform,
                  const std::shared_ptr<DynamicStrands>& target_dynamic_strands,
                  const std::vector<uint32_t>& segment_handles);
  void Update(const GlobalTransform& new_global_transform,
              const std::shared_ptr<DynamicStrands>& target_dynamic_strands);
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const std::shared_ptr<DynamicStrands>& target_dynamic_strands) override;
};

class DsGravity final : public IDsPhysicsOperator {
 public:
  struct GravityPushConstant {
    glm::vec3 acceleration;
    uint32_t particle_size = 0;
    float ground_height = 0.0f;
  };

  float ground_height = 0.0f;
  glm::vec3 gravity = glm::vec3(0, -9.81, 0);
  inline static std::shared_ptr<ComputePipeline> gravity_force_pipeline{};
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const std::shared_ptr<DynamicStrands>& target_dynamic_strands) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  DsGravity();
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
    uint32_t particle_size;
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
    uint32_t particle_size;
  };
  inline static std::shared_ptr<ComputePipeline> pipeline{};

  DsDrag();
  void Update(const glm::vec3& acceleration);
  void Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
               const std::shared_ptr<DynamicStrands>& target_dynamic_strands) override;
};
}  // namespace eco_sys_lab_plugin