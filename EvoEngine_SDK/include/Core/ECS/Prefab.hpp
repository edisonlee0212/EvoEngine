
#pragma once
#include "Animator.hpp"
#include "IAsset.hpp"
#include "IPrivateComponent.hpp"
#include "Transform.hpp"

namespace evo_engine {

/**
 * @brief Represents a holder for data components, which includes
 * a specific DataComponentType and its associated data component.
 */
struct DataComponentHolder {
  /// The type of the data component.
  DataComponentType data_component_type;
  /// Shared pointer to the data component.
  std::shared_ptr<IDataComponent> data_component;

  /**
   * @brief Serializes the data component to YAML format.
   * @param[out] out The YAML emitter for serialization.
   */
  void Serialize(YAML::Emitter& out) const;

  /**
   * @brief Deserializes the data component from YAML format.
   * @param[in] in The YAML node containing the serialized data component.
   * @return True if deserialization is successful, otherwise false.
   */
  bool Deserialize(const YAML::Node& in);
};

/**
 * @brief Represents a holder for private components, which includes
 * an enable flag and the private component itself.
 */
struct PrivateComponentHolder {
  /// Indicates whether the private component is enabled.
  bool enabled;
  /// Shared pointer to the private component.
  std::shared_ptr<IPrivateComponent> private_component;

  /**
   * @brief Serializes the private component to YAML format.
   * @param[out] out The YAML emitter for serialization.
   */
  void Serialize(YAML::Emitter& out) const;

  /**
   * @brief Deserializes the private component from YAML format.
   * @param[in] in The YAML node containing the serialized private component.
   */
  void Deserialize(const YAML::Node& in);
};

/**
 * @brief Represents a Prefab asset, including capabilities to manage
 * components, children, and other asset operations like loading, saving,
 * and serialization.
 */
class Prefab : public IAsset {
  /// The enabled state of the prefab.
  bool enabled_ = true;

#pragma region Model Loading
  /**
   * @brief Attaches an animator to the prefab.
   * @param[in] parent The parent prefab to attach the animator to.
   * @param[in] animator_entity_handle The handle to the animator entity.
   */
  static void AttachAnimator(Prefab* parent, const Handle& animator_entity_handle);

  /**
   * @brief Applies bone indices to the prefab.
   * @param[in] bones_lists The unordered map containing bones and their indices.
   * @param[in] node The prefab to apply the bone indices to.
   */
  static void ApplyBoneIndices(const std::unordered_map<Handle, std::vector<std::shared_ptr<Bone>>>& bones_lists,
                               Prefab* node);

  /**
   * @brief Attaches child entities to the parent entity in the given prefab.
   * @param[in] scene The scene containing the entities.
   * @param[in] model_node The node of the prefab model.
   * @param[in] parent_entity The parent entity to attach children to.
   * @param[out] map A map of handles for relinking entities.
   */
  static void AttachChildren(const std::shared_ptr<Scene>& scene, const std::shared_ptr<Prefab>& model_node,
                             Entity parent_entity, std::unordered_map<Handle, Handle>& map);

  /**
   * @brief Attaches private components of child entities to the parent entity in the given prefab.
   * @param[in] scene The scene containing the entities.
   * @param[in] model_node The node of the prefab model.
   * @param[in] parent_entity The parent entity to attach children to.
   * @param[in] map A map of handles for relinking entities.
   */
  void AttachChildrenPrivateComponent(const std::shared_ptr<Scene>& scene, const std::shared_ptr<Prefab>& model_node,
                                      const Entity& parent_entity, const std::unordered_map<Handle, Handle>& map) const;

  /**
   * @brief Relinks child entities to their parent entity.
   * @param[in] scene The scene containing the entities.
   * @param[in] parent_entity The parent entity.
   * @param[in] map A map of handles for relinking entities.
   */
  static void RelinkChildren(const std::shared_ptr<Scene>& scene, const Entity& parent_entity,
                             const std::unordered_map<Handle, Handle>& map);
#pragma endregion

  /**
   * @brief Inspects the components of the given prefab.
   * @param[in] walker The prefab to inspect.
   * @return True if the inspection modifies any data; otherwise false.
   */
  static bool OnInspectComponents(const std::shared_ptr<Prefab>& walker);

  /**
   * @brief Inspects the given prefab walker.
   * @param[in] walker The prefab to inspect.
   * @return True if the inspection modifies any data; otherwise false.
   */
  static bool OnInspectWalker(const std::shared_ptr<Prefab>& walker);

  /**
   * @brief Gathers the assets associated with the given prefab walker.
   * @param[in] walker The prefab from which to gather assets.
   * @param[out] assets A map of asset handles and their references.
   */
  static void GatherAssetsWalker(const std::shared_ptr<Prefab>& walker, std::unordered_map<Handle, AssetRef>& assets);

 protected:
  /**
   * @brief Loads the prefab asset from a specified file path.
   * @param[in] path The path to the file to load from.
   * @return True if loading is successful; otherwise false.
   */
  [[nodiscard]] bool LoadInternal(const std::filesystem::path& path) override;

  /**
   * @brief Saves the prefab asset to a specified file path.
   * @param[in] path The path to the file to save to.
   * @return True if saving is successful; otherwise false.
   */
  [[nodiscard]] bool SaveInternal(const std::filesystem::path& path) const override;

  /**
   * @brief Loads a model from a specified file path with optional optimization and flags.
   * @param[in] path The path to the model file.
   * @param[in] optimize Whether to optimize the model during loading.
   * @param[in] flags Flags controlling the import process.
   * @return True if loading is successful; otherwise false.
   */
  bool LoadModelInternal(const std::filesystem::path& path, bool optimize = false,
                         unsigned flags = aiProcess_Triangulate | aiProcess_CalcTangentSpace |
                                          aiProcess_GenSmoothNormals);

  /**
   * @brief Saves the model to a specified file path.
   * @param[in] path The path to the file to save the model to.
   * @return True if saving is successful; otherwise false.
   */
  [[nodiscard]] bool SaveModelInternal(const std::filesystem::path& path) const;

 public:
  /**
   * @brief Generates a thumbnail texture for the prefab.
   * @return The generated thumbnail texture.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture() override;

  /// Name of the prefab instance.
  std::string instance_name;

  /**
   * @brief Gathers all assets referenced by the prefab.
   */
  void GatherAssets();

  /// A map of collected asset handles and their references.
  std::unordered_map<Handle, AssetRef> collected_assets;

  /**
   * @brief Inspects the prefab using the editor layer.
   * @param[in] editor_layer The editor layer used for inspection.
   * @return True if the inspection modifies any data; otherwise false.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /// Handle to the prefab's entity.
  Handle entity_handle = Handle();

  /// A list of data component holders contained in the prefab.
  std::vector<DataComponentHolder> data_components;

  /// A list of private component holders contained in the prefab.
  std::vector<PrivateComponentHolder> private_components;

  /// A list of child prefabs belonging to this prefab.
  std::vector<std::shared_ptr<Prefab>> child_prefabs;

  /**
   * @brief Retrieves a private component of type T.
   * @tparam T The type of private component to retrieve.
   * @return A shared pointer to the private component, or nullptr if not found.
   */
  template <typename T = IPrivateComponent>
  std::shared_ptr<T> GetPrivateComponent();

  /**
   * @brief Called when the prefab is created.
   */
  void OnCreate() override;

  /**
   * @brief Computes the bounding box of the prefab.
   * @return The bounding box of the prefab.
   */
  [[nodiscard]] Bound GetBoundingBox() const;

  /**
   * @brief Calculates the adjusted transform of the prefab with rescale and recenter options.
   * @param[in] rescale Whether to rescale the prefab.
   * @param[in] recenter Whether to recenter the prefab.
   * @return The adjusted transform of the prefab.
   */
  [[nodiscard]] Transform CalculateAdjustedTransform(bool rescale = true, bool recenter = true) const;

  /**
   * @brief Converts the prefab into an entity in the given scene.
   * @param[in] scene The scene where the entity is created.
   * @param[in] rescale Whether to rescale the entity.
   * @param[in] recenter Whether to recenter the entity.
   * @return The created entity.
   */
  [[maybe_unused]] Entity ToEntity(const std::shared_ptr<Scene>& scene, bool rescale = false,
                                   bool recenter = false) const;

  /**
   * @brief Loads a model into the prefab from the specified file path.
   * @param[in] path The path to the model file.
   * @param[in] optimize Whether to optimize the model during loading.
   * @param[in] flags Optional flags for controlling the import process.
   */
  void LoadModel(const std::filesystem::path& path, bool optimize = false,
                 unsigned flags = aiProcess_Triangulate | aiProcess_CalcTangentSpace | aiProcess_GenSmoothNormals);

  /**
   * @brief Populates the prefab with data from an entity.
   * @param[in] entity The entity to use as the source.
   */
  void FromEntity(const Entity& entity);

  /**
   * @brief Collects all assets referred by the prefab and adds them to a provided map.
   * @param[out] map A map of asset handles and their corresponding IAsset instances.
   */
  void CollectAssets(std::unordered_map<Handle, std::shared_ptr<IAsset>>& map) const;

  /**
   * @brief Serializes the prefab to YAML format.
   * @param[out] out The YAML emitter for serialization.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the prefab from YAML format.
   * @param[in] in The YAML node containing the serialized prefab data.
   */
  void Deserialize(const YAML::Node& in) override;
};

template <typename T>
/**
 * @brief Retrieves a private component of type T from the prefab.
 * @tparam T The type of private component to retrieve.
 * @return A shared pointer to the private component, or nullptr if not found.
 */
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
