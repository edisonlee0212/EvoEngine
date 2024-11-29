#pragma once
#include "DynamicStrands.hpp"
#include "DsOperators.hpp"
#include "StrandModelData.hpp"
#include "TreeGrowthData.hpp"

using namespace evo_engine;
namespace eco_sys_lab_plugin {
class DynamicTreeStrands : public IPrivateComponent {
 public:
  bool enable_simulation = true;

  PrivateComponentRef tree_ref{};
  StrandModelSkeleton strand_model_skeleton{};
  DtsStrandGroup subdivided_strand_group{};

  DynamicStrands::InitializeParameters initialize_parameters{};
  bool enable_physics = true;
  DynamicStrands::PhysicsParameters physics_parameters{};
  DynamicStrands::VisualizationParameters visualization_parameters{};
  DynamicStrands::RenderParameters render_parameters{};
  std::shared_ptr<DynamicStrands> dynamic_strands{};

  struct EntityTransform {
    Entity target_entity;
    std::shared_ptr<DsTransform> ds_transform;
  };
  struct EntityAttraction {
    Entity target_entity;
    std::shared_ptr<DsAttraction> ds_attraction;
  };
  
  bool limit_strand_length = true;
  float max_strand_length = 1.f;

  std::vector<EntityTransform> transform_operators;
  std::vector<EntityAttraction> attraction_operators;

  std::shared_ptr<DsBoxSelection> box_selection_operator;
  std::shared_ptr<DsDrag> drag_operator;
  std::shared_ptr<DsGravity> gravity;
  void UpdateDynamicStrands();
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void OnCreate() override;
  void OnDestroy() override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;

  struct MultipleRodExperimentSetupSettings {
    float segment_length = 0.05f;
    float radius = 0.002f;
    glm::ivec3 rod_dimension = {10, 10, 10};
    bool add_operator = true;
  };

  void MultipleRodExperimentSetup(const MultipleRodExperimentSetupSettings& settings);
  void InitializeStrandParticles(const DtsStrandGroup& target_strand_group) const;
  void ClearStrandParticles() const;
  void PhysicsStep() const;

  void Visualization(const std::shared_ptr<Camera>& target_camera) const;

  void Render(const std::shared_ptr<Camera>& target_camera) const;
};
}  // namespace eco_sys_lab_plugin