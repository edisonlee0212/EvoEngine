#pragma once
#include "Animator.hpp"
#include "Material.hpp"
#include "PrivateComponentRef.hpp"
#include "SkinnedMesh.hpp"
namespace evo_engine {
class SkinnedMeshRenderer : public IPrivateComponent {
  friend class Animator;
  friend class AnimationLayer;
  friend class Prefab;
  friend class RenderLayer;
  void RenderBound(const std::shared_ptr<EditorLayer>& editor_layer, glm::vec4& color);
  friend class Platform;
  bool rag_doll_ = false;
  std::vector<glm::mat4> rag_doll_transform_chain_;
  std::vector<EntityRef> bound_entities_;

 public:
  void UpdateBoneMatrices();
  bool rag_doll_freeze = false;
  [[nodiscard]] bool RagDoll() const;
  void SetRagDoll(bool value);
  PrivateComponentRef animator;
  std::shared_ptr<BoneMatrices> bone_matrices;
  bool cast_shadow = true;
  AssetRef skinned_mesh;
  AssetRef material;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void OnCreate() override;
  void OnDestroy() override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
  void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) override;

  [[nodiscard]] size_t GetRagDollBoneSize() const;
  void SetRagDollBoundEntity(int index, const Entity& entity, bool reset_transform = true);
  void SetRagDollBoundEntities(const std::vector<Entity>& entities, bool reset_transform = true);
};

}  // namespace evo_engine
