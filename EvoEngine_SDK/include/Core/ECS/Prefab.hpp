#pragma once
#include <Animator.hpp>
#include <IAsset.hpp>
#include <IPrivateComponent.hpp>
#include <ISystem.hpp>
#include <Material.hpp>
#include <Mesh.hpp>
#include <SkinnedMesh.hpp>
#include <Transform.hpp>

#include "Texture2D.hpp"

namespace evo_engine {

struct DataComponentHolder {
  DataComponentType data_component_type;
  std::shared_ptr<IDataComponent> data_component;

  void Serialize(YAML::Emitter& out) const;
  bool Deserialize(const YAML::Node& in);
};

struct PrivateComponentHolder {
  bool enabled;
  std::shared_ptr<IPrivateComponent> private_component;

  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
};

class Prefab : public IAsset {
  bool enabled_ = true;
#pragma region Model Loading
  static void AttachAnimator(Prefab* parent, const Handle& animator_entity_handle);
  static void ApplyBoneIndices(const std::unordered_map<Handle, std::vector<std::shared_ptr<Bone>>>& bones_lists,
                               Prefab* node);
  static void AttachChildren(const std::shared_ptr<Scene>& scene, const std::shared_ptr<Prefab>& model_node,
                             Entity parent_entity, std::unordered_map<Handle, Handle>& map);

  void AttachChildrenPrivateComponent(const std::shared_ptr<Scene>& scene, const std::shared_ptr<Prefab>& model_node,
                                      const Entity& parent_entity, const std::unordered_map<Handle, Handle>& map) const;
  static void RelinkChildren(const std::shared_ptr<Scene>& scene, const Entity& parent_entity,
                             const std::unordered_map<Handle, Handle>& map);
#pragma endregion

  static bool OnInspectComponents(const std::shared_ptr<Prefab>& walker);
  static bool OnInspectWalker(const std::shared_ptr<Prefab>& walker);
  static void GatherAssetsWalker(const std::shared_ptr<Prefab>& walker, std::unordered_map<Handle, AssetRef>& assets);

 protected:
  [[nodiscard]] bool LoadInternal(const std::filesystem::path& path) override;
  [[nodiscard]] bool SaveInternal(const std::filesystem::path& path) const override;
  [[nodiscard]] bool LoadModelInternal(const std::filesystem::path& path, bool optimize = false,
                         unsigned flags = aiProcess_Triangulate | aiProcess_CalcTangentSpace |
                                          aiProcess_GenSmoothNormals);
  [[nodiscard]] bool SaveModelInternal(const std::filesystem::path& path) const;

 public:
  std::string instance_name;
  void GatherAssets();

  std::unordered_map<Handle, AssetRef> collected_assets;

  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  Handle entity_handle = Handle();
  std::vector<DataComponentHolder> data_components;
  std::vector<PrivateComponentHolder> private_components;
  std::vector<std::shared_ptr<Prefab>> child_prefabs;
  template <typename T = IPrivateComponent>
  std::shared_ptr<T> GetPrivateComponent();
  void OnCreate() override;
  [[nodiscard]] Bound GetBoundingBox() const;
  [[nodiscard]] Transform CalculateAdjustedTransform(bool rescale = true, bool recenter = true) const;
  [[maybe_unused]] Entity ToEntity(const std::shared_ptr<Scene>& scene, bool rescale = false, bool recenter = false) const;

  void LoadModel(const std::filesystem::path& path, bool optimize = false,
                 unsigned flags = aiProcess_Triangulate | aiProcess_CalcTangentSpace | aiProcess_GenSmoothNormals);

  void FromEntity(const Entity& entity);
  void CollectAssets(std::unordered_map<Handle, std::shared_ptr<IAsset>>& map) const;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};

template <typename T>
std::shared_ptr<T> Prefab::GetPrivateComponent() {
  auto type_name = Serialization::GetSerializableTypeName<T>();
  for (auto& i : private_components) {
    if (i.private_component->GetTypeName() == type_name) {
      return std::static_pointer_cast<T>(i.private_component);
    }
  }
  return nullptr;
}

}  // namespace evo_engine