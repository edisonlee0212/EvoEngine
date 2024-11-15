#pragma once
#include "DynamicStrands.hpp"
#include "DynamicStrandsOperators.hpp"
#include "StrandModelData.hpp"
#include "TreeGrowthData.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {
class DsParticleNeighbor;
class DsGravity;
class DsTransform;
class DsAttraction;
class DsBoxSelection;

class DynamicTreeStrands : public IPrivateComponent {
 public:
  bool enable_simulation = true;

  PrivateComponentRef tree_ref{};
  StrandModelSkeleton strand_model_skeleton{};
  StrandModelStrandGroup subdivided_strand_group{};

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
  std::vector<EntityTransform> transform_operators;
  std::vector<EntityAttraction> attraction_operators;

  std::shared_ptr<DsBoxSelection> box_selection_operator;
  std::shared_ptr<DsDrag> drag_operator;
  std::shared_ptr<DsGravity> gravity;
  std::shared_ptr<DsParticleNeighbor> connectivity;
  void UpdateDynamicStrands();
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void OnCreate() override;
  void OnDestroy() override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
  void SingleRodExperimentSetup(float total_length, float min_segment_length, float max_segment_length);
  void MultipleRodExperimentSetup(float total_length, float min_segment_length, float max_segment_length, float radius,
                                  const glm::ivec2& rod_dimension, bool add_operator);
  void UniformMultipleRodExperimentSetup(float segment_length, float radius,
                                  const glm::ivec3& rod_dimension,
                                  bool add_operator);

  void Subdivide(float min_segment_length, float max_segment_length, const StrandModelStrandGroup& src);
  void InitializeStrandParticles(const StrandModelStrandGroup& target_strand_group) const;
  void ClearStrandParticles() const;
  void PhysicsStep() const;

  void Visualization(const std::shared_ptr<Camera>& target_camera) const;

  void Render(const std::shared_ptr<Camera>& target_camera) const;
};
}  // namespace eco_sys_lab_plugin