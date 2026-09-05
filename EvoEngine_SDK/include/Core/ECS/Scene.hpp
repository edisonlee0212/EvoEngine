
#pragma once
#include "Bound.hpp"
#include "Entities.hpp"
#include "Entity.hpp"
#include "EntityMetadata.hpp"
#include "GlobalReflectionProbe.hpp"
#include "IAsset.hpp"
#include "IPrivateComponent.hpp"
#include "ISystem.hpp"
#include "Input.hpp"
#include "Jobs.hpp"
#include "PrivateComponentRef.hpp"
#include "PrivateComponentStorage.hpp"
#include "Utilities.hpp"

namespace evo_engine {

struct SdfgiRuntime;

/**
 * @brief Enum for categorizing system groups in the engine.
 */
enum SystemGroup {
  PreparationSystemGroup = 0,  ///< Systems for preparation tasks.
  SimulationSystemGroup = 1,   ///< Systems for simulation tasks.
  PresentationSystemGroup = 2  ///< Systems for presentation/rendering tasks.
};

/**
 * @brief Structure for storing scene data, including entities, metadata, and components.
 */
struct EVOENGINE_API SceneDataStorage {
  /// List of entities in the scene.
  std::vector<Entity> entities;

  /// Metadata associated with the entities.
  std::vector<EntityMetadata> entity_metadata_list;

  /// List of data component storages.
  std::vector<DataComponentStorage> data_component_storage_list;

  /// Map of handles to entities.
  std::unordered_map<Handle, Entity> entity_map;

  /// Storage for private components associated with entities.
  PrivateComponentStorage entity_private_component_storage;

  /**
   * @brief Clones data from a source SceneDataStorage to create a new scene.
   * @param entity_links Map to store links between old and new entity handles.
   * @param source The source SceneDataStorage to clone from.
   * @param new_scene Shared pointer to the new scene created.
   */
  void Clone(std::unordered_map<Handle, Handle>& entity_links, const SceneDataStorage& source,
             const std::shared_ptr<Scene>& new_scene);
};

void SerializeScene(YAML::Emitter& out, const Scene& scene);
void DeserializeScene(const YAML::Node& in, Scene& scene);
void WriteSceneDataComponentStorage(const Scene& scene, const DataComponentStorage& storage, YAML::Emitter& out);
void ReadSceneDataComponentStorage(Scene& scene, size_t storage_index, DataComponentStorage& data_component_storage,
                                   const YAML::Node& in);

/**
 * @brief Represents a scene in the engine, including entities, systems, and environmental properties.
 */
class EVOENGINE_API Scene final : public IAsset {
  friend class RenderLayer;
  std::shared_ptr<SdfgiRuntime> sdfgi_runtime_;

 public:
  [[nodiscard]] std::shared_ptr<const SdfgiRuntime> GetSdfgiRuntime() const {
    return sdfgi_runtime_;
  }
  static bool RegisterAssetIoHandlers(const std::string& owner_name = {}, const std::string& type_name = "Scene");

  /**
   * @brief Generates a thumbnail texture for the scene.
   * @return A shared pointer to the generated 2D texture.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture();

  /**
   * @brief Retrieves a list of entities with a specific private component.
   * @tparam T The type of the private component.
   * @return A vector of entities that own the private component of type T.
   */
  template <typename T>
  std::vector<Entity> GetPrivateComponentOwnersList();

  /**
   * @brief Retrieves the action type of a specific key.
   * @param key The key to query.
   * @return The action type for the specified key.
   */
  Input::KeyActionType GetKey(int key);

  /**
   * @brief Retrieves component data of entities matching a query and stores it in a container.
   * @tparam T The type of data components to retrieve.
   * @param entity_query The query to filter entities.
   * @param container The vector to store the retrieved data components.
   * @param check_enable Whether to check if the entities are enabled.
   */
  template <typename T = IDataComponent>
  void GetComponentDataArray(const EntityQuery& entity_query, std::vector<T>& container, bool check_enable = true);

  /**
   * @brief Retrieves data components of entities matching a query and filters them with a function.
   * @tparam T1 The type of data components to retrieve and store.
   * @tparam T2 The type of data to filter entities.
   * @param entity_query The query to filter entities.
   * @param container The vector to store the retrieved data components.
   * @param filter_func The function to filter the data components.
   * @param check_enable Whether to check if the entities are enabled.
   */
  template <typename T1 = IDataComponent, typename T2 = IDataComponent>
  void GetComponentDataArray(const EntityQuery& entity_query, std::vector<T1>& container,
                             std::function<bool(const T2&)>&& filter_func, bool check_enable = true);

  /**
   * @brief Retrieves data components of entities matching a query and filters them with a two-input function.
   * @tparam T1 The type of data components to retrieve and store.
   * @tparam T2 The first type of data to filter entities.
   * @tparam T3 The second type of data to filter entities.
   * @param entity_query The query to filter entities.
   * @param container The vector to store the retrieved data components.
   * @param filter_func The function to filter the data components.
   * @param check_enable Whether to check if the entities are enabled.
   */
  template <typename T1 = IDataComponent, typename T2 = IDataComponent, typename T3 = IDataComponent>
  void GetComponentDataArray(const EntityQuery& entity_query, std::vector<T1>& container,
                             std::function<bool(const T2&, const T3&)>&& filter_func, bool check_enable = true);
  /**
   * @brief Retrieves data components of entities matching a query and filters them with a two-input function.
   * @tparam T1 The type of data components to retrieve and store.
   * @tparam T2 The first type of data to filter entities.
   * @param entity_query The query to filter entities.
   * @param filter Required identical data component.
   * @param container The vector to store the retrieved data components.
   * @param check_enable Whether to check if the entities are enabled.
   */
  template <typename T1 = IDataComponent, typename T2 = IDataComponent>
  void GetComponentDataArray(const EntityQuery& entity_query, const T1& filter, std::vector<T2>& container,
                             bool check_enable = true);

  /**
   * @brief Retrieves entities matching a query and applies a filter function.
   * @tparam T1 The type of data component for filtering.
   * @param entity_query The query to filter entities.
   * @param container A vector to store the matching entities.
   * @param filter_func The function used to filter the entities.
   * @param check_enable Whether to check if the entities are enabled.
   */
  template <typename T1 = IDataComponent>
  void GetEntityArray(const EntityQuery& entity_query, std::vector<Entity>& container,
                      std::function<bool(const Entity&, const T1&)>&& filter_func, bool check_enable = true);

  /**
   * @brief Filters entities using a query and stores them in a container.
   * @param entity_query The query to filter entities.
   * @param container A vector to store the matching entities.
   * @param check_enable Whether to check if the entities are enabled.
   */
  void GetEntityArray(const EntityQuery& entity_query, std::vector<Entity>& container, bool check_enable = true);
  /**
   * @brief Filters entities using a query and stores them in a container.
   * @param entity_query The query to filter entities.
   * @param container A vector to store the matching entities.
   * @param filter_func The function used to filter the entities.
   * @param check_enable Whether to check if the entities are enabled.
   */
  template <typename T1 = IDataComponent, typename T2 = IDataComponent>
  void GetEntityArray(const EntityQuery& entity_query, std::vector<Entity>& container,
                      std::function<bool(const Entity&, const T1&, const T2&)>&& filter_func, bool check_enable = true);

  /**
   * @brief Filters entities using a query and stores them in a container.
   * @param entity_query The query to filter entities.
   * @param filter Required identical data component.
   * @param container A vector to store the matching entities.
   * @param check_enable Whether to check if the entities are enabled.
   */
  template <typename T1 = IDataComponent>
  void GetEntityArray(const EntityQuery& entity_query, const T1& filter, std::vector<Entity>& container,
                      bool check_enable = true);
  /**
   * @brief Retrieves the total count of entities that match a query.
   * @param entity_query The query to filter entities.
   * @param check_enable Whether to count only enabled entities.
   * @return The total number of entities matching the query.
   */
  size_t GetEntityAmount(EntityQuery entity_query, bool check_enable = true);

  /**
   * @brief Gets the handle to a specified entity.
   * @param entity The entity to retrieve the handle for.
   * @return The handle of the entity.
   */
  [[nodiscard]] Handle GetEntityHandle(const Entity& entity);

  /**
   * @brief Retrieves or creates a system of a specific type with a given execution rank.
   * @tparam T The type of the system.
   * @param rank The execution rank of the system.
   * @return A shared pointer to the system.
   */
  template <typename T = ISystem>
  std::shared_ptr<T> GetOrCreateSystem(float rank);

  /**
   * @brief Retrieves a system of a specific type.
   * @tparam T The type of the system.
   * @return A shared pointer to the system, or nullptr if it doesn't exist.
   */
  template <typename T = ISystem>
  std::shared_ptr<T> GetSystem();

  /**
   * @brief Checks if a system of a specific type exists in the scene.
   * @tparam T The type of the system.
   * @return True if the system exists, false otherwise.
   */
  template <typename T = ISystem>
  bool HasSystem();

  /**
   * @brief Retrieves or creates a system with a specified name and execution order.
   * @param system_name The name of the system.
   * @param order The execution order of the system.
   * @return A shared pointer to the system.
   */
  std::shared_ptr<ISystem> GetOrCreateSystem(const std::string& system_name, float order);
  /// Explicit scene-global prefiltered specular fallback used after local reflection probes.
  AssetRef global_reflection_probe_fallback;

  /// Optional authoring asset for local reflection probes, DDGI volumes, and shared indirect environment lighting.
  AssetRef environmental_lighting;

  /**
   * @brief Retrieves the explicit scene-global reflection fallback.
   * @param require_runtime_ready If true, only returns a probe whose runtime cubemap payload is ready.
   * @return Shared pointer to the assigned global reflection fallback, or nullptr.
   */
  [[nodiscard]] std::shared_ptr<GlobalReflectionProbe> GetGlobalReflectionProbeFallback(
      bool require_runtime_ready = true);

  /// Reference to the main camera used in the scene.
  PrivateComponentRef main_camera;

  /**
   * @brief Purges all entities and systems from the scene.
   */
  void Purge();

  /**
   * @brief Called when the scene is created.
   */
  void OnCreate() override;

  /**
   * @brief Clones a source scene into a new scene.
   * @param source The source scene to clone.
   * @param new_scene The new scene to create.
   */
  static void Clone(const std::shared_ptr<Scene>& source, const std::shared_ptr<Scene>& new_scene);

  /**
   * @brief Retrieves the world boundary of the scene.
   * @return The boundary of the world.
   */
  [[nodiscard]] Bound GetBound() const;

  [[nodiscard]] uint64_t GetHierarchyRevision() const;
  [[nodiscard]] uint64_t GetRenderStructureRevision() const;

  /**
   * @brief Sets the boundary of the world in the scene.
   * @param value The new boundary to set.
   */
  void SetBound(const Bound& value);

  /**
   * @brief Destroys a system of a specific type.
   * @tparam T The type of the system.
   */
  template <typename T = ISystem>
  void DestroySystem();

  /**
   * @brief Destructs the scene and cleans up resources.
   */
  ~Scene() override;

  /**
   * @brief Executes fixed update operations for the scene.
   */
  void FixedUpdate() const;

  /**
   * @brief Starts the scene and its components.
   */
  void Start() const;

  /**
   * @brief Handles update operations for the scene.
   */
  void Update() const;

  /**
   * @brief Handles late update operations for the scene.
   */
  void LateUpdate() const;

  [[nodiscard]] const std::multimap<float, std::shared_ptr<ISystem>>& PeekSystems() const;

  [[nodiscard]] bool HasSystemType(const size_t& type_id) const;

  std::shared_ptr<ISystem> CreateSystemByTypeId(const size_t& type_id, float order);

  [[nodiscard]] std::shared_ptr<Scene> GetSelfScene();

  /**
   * @brief Removes a private component from an entity.
   * @param entity The entity to remove the private component from.
   * @param type_id The type ID of the private component to remove.
   */
  void RemovePrivateComponent(const Entity& entity, size_t type_id);

  /**
   * @brief Enables or disables an entity. Disabling an entity will recursively disable its children.
   * @param entity The entity to enable or disable.
   * @param value Boolean value indicating whether to enable or disable the entity.
   */
  void SetEnable(const Entity& entity, const bool& value);

  /**
   * @brief Checks if an entity is valid.
   * @param entity The entity to validate.
   * @return True if the entity is valid, false otherwise.
   */
  [[nodiscard]] bool IsEntityValid(const Entity& entity) const;

  /**
   * @brief Checks if an entity is enabled.
   * @param entity The entity to check.
   * @return True if the entity is enabled, false otherwise.
   */
  [[nodiscard]] bool IsEntityEnabled(const Entity& entity) const;

  /**
   * @brief Checks if the given entity is a root entity.
   * @param entity The entity to check.
   * @return True if the entity is a root entity, false otherwise.
   */
  [[nodiscard]] bool IsEntityRoot(const Entity& entity) const;

  /**
   * @brief Checks if the given entity is marked as static.
   * @param entity The entity to check.
   * @return True if the entity is static, false otherwise.
   */
  [[nodiscard]] bool IsEntityStatic(const Entity& entity) const;

  /**
   * @brief Checks if the given entity has an ancestor selected in the hierarchy.
   * @param entity The entity to check.
   * @return True if an ancestor is selected, false otherwise.
   */
  /**
   * @brief Retrieves the root entity of a given entity.
   * @param entity The entity whose root is needed.
   * @return The root entity.
   */
  Entity GetRoot(const Entity& entity) const;

  /**
   * @brief Retrieves the name of a given entity.
   * @param entity The entity to query.
   * @return The name of the entity.
   */
  std::string GetEntityName(const Entity& entity);

  /**
   * @brief Sets the name of a given entity.
   * @param entity The entity to update.
   * @param name The new name for the entity.
   */
  void SetEntityName(const Entity& entity, const std::string& name);

  /**
   * @brief Marks an entity as static or dynamic.
   * @param entity The entity to update.
   * @param value True to mark the entity as static, false for dynamic.
   */
  void SetEntityStatic(const Entity& entity, bool value);

  /**
   * @brief Sets the parent of a child entity.
   * @param child The child entity to update.
   * @param parent The parent entity to assign.
   * @param recalculate_transform Whether to recalculate the child's transform after setting the parent.
   */
  void SetParent(const Entity& child, const Entity& parent, const bool& recalculate_transform = false);

  /**
   * @brief Retrieves the parent of a given entity.
   * @param entity The entity to query.
   * @return The parent entity of the given entity.
   */
  [[nodiscard]] Entity GetParent(const Entity& entity) const;

  /**
   * @brief Retrieves all child entities of a given entity.
   * @param entity The entity to query.
   * @return A vector of child entities.
   */
  [[nodiscard]] std::vector<Entity> GetChildren(const Entity& entity);

  /**
   * @brief Retrieves the child entity at a specific index.
   * @param entity The parent entity.
   * @param index The index of the child.
   * @return The child entity at the given index.
   */
  [[nodiscard]] Entity GetChild(const Entity& entity, size_t index) const;

  /**
   * @brief Retrieves the number of children a given entity has.
   * @param entity The entity to query.
   * @return The number of child entities.
   */
  [[nodiscard]] size_t GetChildrenAmount(const Entity& entity) const;

  /**
   * @brief Iterates through all children of an entity and applies a function to each.
   * @param entity The parent entity.
   * @param func The function to apply to each child.
   */
  void ForEachChild(const Entity& entity, const std::function<void(Entity child)>& func) const;

  /**
   * @brief Removes a child entity from a parent entity.
   * @param child The child entity to remove.
   * @param parent The parent entity to update.
   */
  void RemoveChild(const Entity& child, const Entity& parent);

  /**
   * @brief Retrieves all descendants of a given entity.
   * @param entity The entity to query.
   * @return A vector of descendant entities.
   */
  std::vector<Entity> GetDescendants(const Entity& entity);

  /**
   * @brief Iterates through all descendants of an entity and applies a function to each.
   * @param target The target entity whose descendants will be iterated through.
   * @param func The function to apply to each descendant.
   * @param from_root Whether to iterate from the root entity downwards.
   */
  void ForEachDescendant(const Entity& target, const std::function<void(const Entity& entity)>& func,
                         const bool& from_root = true);

  template <typename T = IDataComponent>
  void AddDataComponent(const Entity& entity, const T& value);
  template <typename T = IDataComponent>
  void RemoveDataComponent(const Entity& entity);
  template <typename T = IDataComponent>
  void SetDataComponent(const Entity& entity, const T& value);
  template <typename T = IDataComponent>
  T GetDataComponent(const Entity& entity);
  template <typename T = IDataComponent>
  [[nodiscard]] bool HasDataComponent(const Entity& entity) const;

  template <typename T = IPrivateComponent>
  [[maybe_unused]] std::weak_ptr<T> GetOrSetPrivateComponent(const Entity& entity);
  [[nodiscard]] std::weak_ptr<IPrivateComponent> GetPrivateComponent(const Entity& entity,
                                                                     const std::string& type_name);

  template <typename T = IPrivateComponent>
  void RemovePrivateComponent(const Entity& entity);
  template <typename T = IPrivateComponent>
  [[nodiscard]] bool HasPrivateComponent(const Entity& entity) const;
  [[nodiscard]] bool HasPrivateComponent(const Entity& entity, const std::string& type_name) const;
  [[nodiscard]] bool HasPrivateComponent(const Entity& entity, const size_t& type_id) const;
  [[nodiscard]] bool HasPrivateComponentOwners(const size_t& type_id) const;
  size_t ClearPrivateComponentPool(const size_t& type_id);
  size_t RestoreUnknownRuntimeTypes();

  [[maybe_unused]] Entity CreateEntity(const std::string& name = "New Entity");
  [[maybe_unused]] Entity CreateEntity(const EntityArchetype& archetype, const std::string& name = "New Entity",
                                       const Handle& handle = Handle());
  [[maybe_unused]] std::vector<Entity> CreateEntities(const EntityArchetype& archetype, const size_t& amount,
                                                      const std::string& name = "New Entity");
  [[maybe_unused]] std::vector<Entity> CreateEntities(const size_t& amount, const std::string& name = "New Entity");
  void DeleteEntity(const Entity& entity);
  Entity GetEntity(const Handle& handle);
  Entity GetEntity(const size_t& index) const;
  void ForEachPrivateComponent(const Entity& entity,
                               const std::function<void(PrivateComponentElement& data)>& func) const;
  void GetAllEntities(std::vector<Entity>& target);
  void ForAllEntities(const std::function<void(size_t i, Entity entity)>& func) const;

  Bound GetEntityBoundingBox(const Entity& entity);

  std::vector<std::reference_wrapper<DataComponentStorage>> QueryDataComponentStorageList(
      const EntityQuery& entity_query);
  std::optional<std::pair<std::reference_wrapper<DataComponentStorage>, unsigned>> GetDataComponentStorage(
      const EntityArchetype& entity_archetype);

  /**
   * @brief Retrieves the internal storage of all entities in the scene (unsafe).
   * @return A constant reference to the vector of all entities.
   */
  const std::vector<Entity>& UnsafeGetAllEntities();

  /**
   * @brief Iterates over the data components of an entity and applies a function (unsafe).
   * @param entity The entity to iterate over.
   * @param func The function to apply to each data component.
   */
  void UnsafeForEachDataComponent(const Entity& entity,
                                  const std::function<void(const DataComponentType& type, void* data)>& func);

  /**
   * @brief Iterates over all entity storages in the scene and applies a function (unsafe).
   * @param func The function to apply to each entity storage.
   */
  void UnsafeForEachEntityStorage(
      const std::function<void(size_t i, const std::string& name, const DataComponentStorage& storage)>& func);

  /**
   * @brief Unsafe method to retrieve pointers and sizes of component data arrays.
   * @tparam T The type of data components to retrieve.
   * @param entity_query The query to filter the data for targeted entity types.
   * @return A vector of pairs containing pointers to the first data instance and the size of data instances.
   */
  template <typename T>
  std::vector<std::pair<T*, size_t>> UnsafeGetDataComponentArray(const EntityQuery& entity_query);

  /**
   * @brief Unsafe method to retrieve the owners list of private components.
   * @tparam T The type of the private component.
   * @return A constant pointer to the vector of entity owners.
   */
  template <typename T>
  const std::vector<Entity>* UnsafeGetPrivateComponentOwnersList();

  /**
   * @brief Iterates over entities matching a query and applies a function in parallel.
   * @tparam T1 The type of data components to process.
   * @param dependencies A list of job dependencies to ensure proper execution order.
   * @param entity_query The query defining the entities to process.
   * @param func The function to apply to the entities.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle representing the scheduled parallel operation.
   */
  template <typename T1>
  JobHandle ForEach(const std::vector<JobHandle>& dependencies, const EntityQuery& entity_query,
                    std::function<void(int i, Entity entity, T1&)>&& func, bool check_enable = true);

  /**
   * @brief Iterates over entities with two data components matching a query and applies a function in parallel.
   * @tparam T1 The 1st type of data components to process.
   * @tparam T2 The 2nd type of data components to process.
   * @param dependencies A list of job dependencies to ensure proper execution order.
   * @param entity_query The query defining the entities to process.
   * @param func The function to apply to the entities.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle representing the scheduled parallel operation.
   */
  template <typename T1, typename T2>
  JobHandle ForEach(const std::vector<JobHandle>& dependencies, const EntityQuery& entity_query,
                    std::function<void(int i, Entity entity, T1&, T2&)>&& func, bool check_enable = true);

  /**
   * @brief Iterates over entities with two data components matching a query and applies a function in parallel.
   * @tparam T1 The 1st type of data components to process.
   * @tparam T2 The 2nd type of data components to process.
   * @tparam T3 The 3rd type of data components to process.
   * @param dependencies A list of job dependencies to ensure proper execution order.
   * @param entity_query The query defining the entities to process.
   * @param func The function to apply to the entities.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle representing the scheduled parallel operation.
   */
  template <typename T1, typename T2, typename T3>
  JobHandle ForEach(const std::vector<JobHandle>& dependencies, const EntityQuery& entity_query,
                    std::function<void(int i, Entity entity, T1&, T2&, T3&)>&& func, bool check_enable = true);
  /**
   * @brief Iterates over entities with two data components matching a query and applies a function in parallel.
   * @tparam T1 The 1st type of data components to process.
   * @tparam T2 The 2nd type of data components to process.
   * @tparam T3 The 3rd type of data components to process.
   * @tparam T4 The 4th type of data components to process.
   * @param dependencies A list of job dependencies to ensure proper execution order.
   * @param entity_query The query defining the entities to process.
   * @param func The function to apply to the entities.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle representing the scheduled parallel operation.
   */
  template <typename T1 = IDataComponent, typename T2 = IDataComponent, typename T3 = IDataComponent,
            typename T4 = IDataComponent>
  JobHandle ForEach(const std::vector<JobHandle>& dependencies, const EntityQuery& entity_query,
                    std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&)>&& func, bool check_enable = true);
  /**
   * @brief Iterates over entities with two data components matching a query and applies a function in parallel.
   * @tparam T1 The 1st type of data components to process.
   * @tparam T2 The 2nd type of data components to process.
   * @tparam T3 The 3rd type of data components to process.
   * @tparam T4 The 4th type of data components to process.
   * @tparam T5 The 5th type of data components to process.
   * @param dependencies A list of job dependencies to ensure proper execution order.
   * @param entity_query The query defining the entities to process.
   * @param func The function to apply to the entities.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle representing the scheduled parallel operation.
   */
  template <typename T1 = IDataComponent, typename T2 = IDataComponent, typename T3 = IDataComponent,
            typename T4 = IDataComponent, typename T5 = IDataComponent>
  JobHandle ForEach(const std::vector<JobHandle>& dependencies, const EntityQuery& entity_query,
                    std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&)>&& func,
                    bool check_enable = true);
  /**
   * @brief Iterates over entities with two data components matching a query and applies a function in parallel.
   * @tparam T1 The 1st type of data components to process.
   * @tparam T2 The 2nd type of data components to process.
   * @tparam T3 The 3rd type of data components to process.
   * @tparam T4 The 4th type of data components to process.
   * @tparam T5 The 5th type of data components to process.
   * @tparam T6 The 6th type of data components to process.
   * @param dependencies A list of job dependencies to ensure proper execution order.
   * @param entity_query The query defining the entities to process.
   * @param func The function to apply to the entities.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle representing the scheduled parallel operation.
   */
  template <typename T1 = IDataComponent, typename T2 = IDataComponent, typename T3 = IDataComponent,
            typename T4 = IDataComponent, typename T5 = IDataComponent, typename T6 = IDataComponent>
  JobHandle ForEach(const std::vector<JobHandle>& dependencies, const EntityQuery& entity_query,
                    std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&, T6&)>&& func,
                    bool check_enable = true);
  /**
   * @brief Iterates over entities with two data components matching a query and applies a function in parallel.
   * @tparam T1 The 1st type of data components to process.
   * @tparam T2 The 2nd type of data components to process.
   * @tparam T3 The 3rd type of data components to process.
   * @tparam T4 The 4th type of data components to process.
   * @tparam T5 The 5th type of data components to process.
   * @tparam T6 The 6th type of data components to process.
   * @tparam T7 The 7th type of data components to process.
   * @param dependencies A list of job dependencies to ensure proper execution order.
   * @param entity_query The query defining the entities to process.
   * @param func The function to apply to the entities.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle representing the scheduled parallel operation.
   */
  template <typename T1 = IDataComponent, typename T2 = IDataComponent, typename T3 = IDataComponent,
            typename T4 = IDataComponent, typename T5 = IDataComponent, typename T6 = IDataComponent,
            typename T7 = IDataComponent>
  JobHandle ForEach(const std::vector<JobHandle>& dependencies, const EntityQuery& entity_query,
                    std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&, T6&, T7&)>&& func,
                    bool check_enable = true);

  /**
   * @brief Iterates over entities with two data components matching a query and applies a function in parallel.
   * @tparam T1 The 1st type of data components to process.
   * @tparam T2 The 2nd type of data components to process.
   * @tparam T3 The 3rd type of data components to process.
   * @tparam T4 The 4th type of data components to process.
   * @tparam T5 The 5th type of data components to process.
   * @tparam T6 The 6th type of data components to process.
   * @tparam T7 The 7th type of data components to process.
   * @tparam T8 The 8th type of data components to process.
   * @param dependencies A list of job dependencies to ensure proper execution order.
   * @param entity_query The query defining the entities to process.
   * @param func The function to apply to the entities.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle representing the scheduled parallel operation.
   */
  template <typename T1 = IDataComponent, typename T2 = IDataComponent, typename T3 = IDataComponent,
            typename T4 = IDataComponent, typename T5 = IDataComponent, typename T6 = IDataComponent,
            typename T7 = IDataComponent, typename T8 = IDataComponent>
  JobHandle ForEach(const std::vector<JobHandle>& dependencies, const EntityQuery& entity_query,
                    std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&, T6&, T7&, T8&)>&& func,
                    bool check_enable = true);

  /**
   * @brief Iterates over entities matching a general query and applies a function in parallel.
   * @tparam T1 The type of data components to process.
   * @param dependencies A list of job dependencies to ensure proper execution order.
   * @param func The function to apply to each entity.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle representing the scheduled parallel operation.
   */
  template <typename T1>
  JobHandle ForEach(const std::vector<JobHandle>& dependencies, std::function<void(int i, Entity entity, T1&)>&& func,
                    bool check_enable = true);

  /**
   * @brief Iterates over entities matching a general query and applies a function in parallel.
   * @tparam T1 The 1st type of data components to process.
   * @tparam T2 The 2nd type of data components to process.
   * @param dependencies A list of job dependencies to ensure proper execution order.
   * @param func The function to apply to each entity.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle representing the scheduled parallel operation.
   */
  template <typename T1, typename T2>
  JobHandle ForEach(const std::vector<JobHandle>& dependencies,
                    std::function<void(int i, Entity entity, T1&, T2&)>&& func, bool check_enable = true);

  /**
   * @brief Iterates over entities matching a general query and applies a function in parallel.
   * @tparam T1 The 1st type of data components to process.
   * @tparam T2 The 2nd type of data components to process.
   * @tparam T3 The 3rd type of data components to process.
   * @param dependencies A list of job dependencies to ensure proper execution order.
   * @param func The function to apply to each entity.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle representing the scheduled parallel operation.
   */
  template <typename T1 = IDataComponent, typename T2 = IDataComponent, typename T3 = IDataComponent>
  JobHandle ForEach(const std::vector<JobHandle>& dependencies,
                    std::function<void(int i, Entity entity, T1&, T2&, T3&)>&& func, bool check_enable = true);

  /**
   * @brief Iterates over entities matching a general query and applies a function in parallel.
   * @tparam T1 The 1st type of data components to process.
   * @tparam T2 The 2nd type of data components to process.
   * @tparam T3 The 3rd type of data components to process.
   * @tparam T4 The 4th type of data components to process.
   * @param dependencies A list of job dependencies to ensure proper execution order.
   * @param func The function to apply to each entity.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle representing the scheduled parallel operation.
   */
  template <typename T1 = IDataComponent, typename T2 = IDataComponent, typename T3 = IDataComponent,
            typename T4 = IDataComponent>
  JobHandle ForEach(const std::vector<JobHandle>& dependencies,
                    std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&)>&& func, bool check_enable = true);

  /**
   * @brief Iterates over entities matching a general query and applies a function in parallel.
   * @tparam T1 The 1st type of data components to process.
   * @tparam T2 The 2nd type of data components to process.
   * @tparam T3 The 3rd type of data components to process.
   * @tparam T4 The 4th type of data components to process.
   * @tparam T5 The 5th type of data components to process.
   * @param dependencies A list of job dependencies to ensure proper execution order.
   * @param func The function to apply to each entity.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle representing the scheduled parallel operation.
   */
  template <typename T1 = IDataComponent, typename T2 = IDataComponent, typename T3 = IDataComponent,
            typename T4 = IDataComponent, typename T5 = IDataComponent>
  JobHandle ForEach(const std::vector<JobHandle>& dependencies,
                    std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&)>&& func,
                    bool check_enable = true);

  /**
   * @brief Iterates over entities matching a general query and applies a function in parallel.
   * @tparam T1 The 1st type of data components to process.
   * @tparam T2 The 2nd type of data components to process.
   * @tparam T3 The 3rd type of data components to process.
   * @tparam T4 The 4th type of data components to process.
   * @tparam T5 The 5th type of data components to process.
   * @tparam T6 The 6th type of data components to process.
   * @param dependencies A list of job dependencies to ensure proper execution order.
   * @param func The function to apply to each entity.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle representing the scheduled parallel operation.
   */
  template <typename T1 = IDataComponent, typename T2 = IDataComponent, typename T3 = IDataComponent,
            typename T4 = IDataComponent, typename T5 = IDataComponent, typename T6 = IDataComponent>
  JobHandle ForEach(const std::vector<JobHandle>& dependencies,
                    std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&, T6&)>&& func,
                    bool check_enable = true);

  /**
   * @brief Iterates over entities matching a general query and applies a function in parallel.
   * @tparam T1 The 1st type of data components to process.
   * @tparam T2 The 2nd type of data components to process.
   * @tparam T3 The 3rd type of data components to process.
   * @tparam T4 The 4th type of data components to process.
   * @tparam T5 The 5th type of data components to process.
   * @tparam T7 The 7th type of data components to process.
   * @param dependencies A list of job dependencies to ensure proper execution order.
   * @param func The function to apply to each entity.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle representing the scheduled parallel operation.
   */
  template <typename T1 = IDataComponent, typename T2 = IDataComponent, typename T3 = IDataComponent,
            typename T4 = IDataComponent, typename T5 = IDataComponent, typename T6 = IDataComponent,
            typename T7 = IDataComponent>
  JobHandle ForEach(const std::vector<JobHandle>& dependencies,
                    std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&, T6&, T7&)>&& func,
                    bool check_enable = true);

  /**
   * @brief Iterates over entities matching a general query and applies a function in parallel.
   * @tparam T1 The 1st type of data components to process.
   * @tparam T2 The 2nd type of data components to process.
   * @tparam T3 The 3rd type of data components to process.
   * @tparam T4 The 4th type of data components to process.
   * @tparam T5 The 5th type of data components to process.
   * @tparam T7 The 7th type of data components to process.
   * @tparam T8 The 8th type of data components to process.
   * @param dependencies A list of job dependencies to ensure proper execution order.
   * @param func The function to apply to each entity.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle representing the scheduled parallel operation.
   */
  template <typename T1 = IDataComponent, typename T2 = IDataComponent, typename T3 = IDataComponent,
            typename T4 = IDataComponent, typename T5 = IDataComponent, typename T6 = IDataComponent,
            typename T7 = IDataComponent, typename T8 = IDataComponent>
  JobHandle ForEach(const std::vector<JobHandle>& dependencies,
                    std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&, T6&, T7&, T8&)>&& func,
                    bool check_enable = true);

 private:
  friend void SerializeScene(YAML::Emitter& out, const Scene& scene);
  friend void DeserializeScene(const YAML::Node& in, Scene& scene);
  friend void WriteSceneDataComponentStorage(const Scene& scene, const DataComponentStorage& storage,
                                             YAML::Emitter& out);
  friend void ReadSceneDataComponentStorage(Scene& scene, size_t storage_index,
                                            DataComponentStorage& data_component_storage, const YAML::Node& in);
  friend class Application;
  friend class Entities;
  friend class EditorLayer;
  friend class Serialization;
  friend class SystemRef;
  friend struct Entity;
  friend class Prefab;
  friend class TransformGraph;
  friend class PrivateComponentStorage;
  friend class Input;
  friend class EditorLayer;
  friend class PackageManager;
  friend class IPrivateComponent;

  /// Stores the states of pressed keys in the scene.
  std::unordered_map<int, Input::KeyActionType> pressed_keys_ = {};

  /// Storage structure for scene data.
  SceneDataStorage scene_data_storage_;

  uint64_t hierarchy_revision_ = 0;
  uint64_t render_structure_revision_ = 0;

  void MarkRenderStructureChanged();

  /// Multimap of systems ordered by their execution order.
  std::multimap<float, std::shared_ptr<ISystem>> systems_;

  /// Map of indexed systems by hash codes.
  std::map<size_t, std::shared_ptr<ISystem>> indexed_systems_;

  /// Map of systems by their handle identifiers.
  std::map<Handle, std::shared_ptr<ISystem>> mapped_systems_;

  /// The boundary of the world contained within the scene.
  Bound world_bound_;

  /**
   * @brief Deletes an entity from the scene internally.
   * @param entity_index Index of the entity to delete.
   */
  void DeleteEntityInternal(unsigned entity_index);

  /**
   * @brief Queries a list of data component storages based on an entity query index.
   * @param entity_query_index The index of the query for filtering data component storages.
   * @return A vector of references to the matched data component storages.
   */
  std::vector<std::reference_wrapper<DataComponentStorage>> QueryDataComponentStorageList(size_t entity_query_index);

  /**
   * @brief Retrieves a data component storage based on the archetype index of an entity.
   * @param entity_archetype_index The archetype index to query.
   * @return An optional pair containing a reference to the data component storage and an unsigned index.
   */
  std::optional<std::pair<std::reference_wrapper<DataComponentStorage>, unsigned>> GetDataComponentStorage(
      size_t entity_archetype_index);

  /**
   * @brief Retrieves a data component array and stores it in a container.
   * @tparam T The type of data component.
   * @param storage The storage structure containing the data components.
   * @param container A reference to the container where the components will be stored.
   * @param check_enable Whether to check if the entities are enabled.
   */
  template <typename T = IDataComponent>
  void GetDataComponentArrayStorage(const DataComponentStorage& storage, std::vector<T>& container, bool check_enable);

  /**
   * @brief Retrieves a list of entities stored in a data component storage.
   * @param storage A reference to the data component storage to retrieve from.
   * @param container A reference to the vector where the entities will be stored.
   * @param check_enable Whether to check if the entities are enabled.
   */
  void GetEntityStorage(const DataComponentStorage& storage, std::vector<Entity>& container, bool check_enable) const;

  /**
   * @brief Swaps two entities within a data component storage.
   * @param storage Reference to the data component storage where the swap occurs.
   * @param index1 Index of the first entity in the data component storage.
   * @param index2 Index of the second entity in the data component storage.
   * @return The size_t index of the swapped entity.
   */
  static size_t SwapEntity(DataComponentStorage& storage, size_t index1, size_t index2);

  /**
   * @brief Enables or disables a single entity.
   * @param entity The entity to set the enabled state for.
   * @param value Boolean value indicating whether to enable or disable the entity.
   */
  void SetEnableSingle(const Entity& entity, const bool& value);

  /**
   * @brief Sets data for a specific data component within an entity.
   * @param entity_index The index of the entity.
   * @param id The type ID of the data component.
   * @param size The size of the data to set.
   * @param data The pointer to the data to set.
   */
  void SetDataComponent(const unsigned& entity_index, size_t id, size_t size, const void* data);

  /**
   * @brief Retrieves a pointer to a data component for the specified entity.
   * @param entity The entity containing the data component.
   * @param id The type ID of the data component.
   * @return A void pointer to the data component.
   */
  [[nodiscard]] void* GetDataComponentPointer(const Entity& entity, const size_t& id);

  /**
   * @brief Retrieves a pointer to a data component using the entity index and its type ID.
   * @param entity_index Index of the entity.
   * @param id The type ID of the data component.
   * @return A void pointer to the data component.
   */
  [[nodiscard]] void* GetDataComponentPointer(unsigned entity_index, const size_t& id);

  /**
   * @brief Sets a private component for the specified entity.
   * @param entity The entity to add a private component to.
   * @param ptr The shared pointer to the private component.
   */
  void SetPrivateComponent(const Entity& entity, const std::shared_ptr<IPrivateComponent>& ptr);

  /**
   * @brief Helper function to execute a function for each descendant of a target entity.
   * @param target The target entity whose descendants will be iterated over.
   * @param func The function to execute for each descendant.
   */
  void ForEachDescendantHelper(const Entity& target, const std::function<void(const Entity& entity)>& func);

  /**
   * @brief Helper function to retrieve all descendants of a target entity.
   * @param target The target entity whose descendants will be retrieved.
   * @param results A vector to store the resulting descendants.
   */
  void GetDescendantsHelper(const Entity& target, std::vector<Entity>& results);

  /**
   * @brief Removes a data component from an entity.
   * @param entity The entity from which the data component will be removed.
   * @param type_index The type index of the data component.
   */
  void RemoveDataComponent(const Entity& entity, const size_t& type_index);

  /**
   * @brief Retrieves a data component for an entity.
   * @tparam T The type of the data component.
   * @param index The index of the entity.
   * @return The data component of type T.
   */
  template <typename T = IDataComponent>
  T GetDataComponent(const size_t& index);

  /**
   * @brief Checks if an entity contains a data component of a specific type.
   * @tparam T The type of the data component.
   * @param index The index of the entity.
   * @return True if the entity has the data component, false otherwise.
   */
  template <typename T = IDataComponent>
  [[nodiscard]] bool HasDataComponent(const size_t& index) const;

  /**
   * @brief Checks if an entity contains a data component of a specific type.
   * @param entity The entity to check.
   * @param type_id The type ID of the data component.
   * @return True if the entity has the data component, false otherwise.
   */
  [[nodiscard]] bool HasDataComponent(const Entity& entity, size_t type_id) const;

  /**
   * @brief Checks if an entity in the data storage contains a data component of a specific type.
   * @param index The index of the entity in the storage.
   * @param type_id The type ID of the data component.
   * @return True if the entity has the data component, false otherwise.
   */
  [[nodiscard]] bool HasDataComponent(const size_t& index, size_t type_id) const;

  /**
   * @brief Adds a data component of a specific type to an entity.
   * @param entity The entity to add the data component to.
   * @param type_id The type ID of the data component.
   */
  void AddDataComponent(const Entity& entity, size_t type_id);

  /**
   * @brief Sets a data component for an entity.
   * @tparam T The type of the data component.
   * @param index The index of the entity in the storage.
   * @param value The value to set for the data component.
   */
  template <typename T = IDataComponent>
  void SetDataComponent(const size_t& index, const T& value);

  /**
   * @brief Iterates over all entities in storage and applies the given function to each.
   * @tparam T1 The type of the data component.
   * @param dependencies A vector of job handles for dependencies.
   * @param storage The data component storage.
   * @param func The function to apply.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle for the operation.
   */
  template <typename T1 = IDataComponent>
  JobHandle ForEachStorage(const std::vector<JobHandle>& dependencies, DataComponentStorage& storage,
                           std::function<void(int i, Entity entity, T1&)>&& func, bool check_enable = true);

  /**
   * @brief Iterates over all entities in storage and applies the given function to each.
   * @tparam T1 The first type of the data component.
   * @tparam T2 The second type of the data component.
   * @param dependencies A vector of job handles for dependencies.
   * @param storage The data component storage.
   * @param func The function to apply.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle for the operation.
   */
  template <typename T1 = IDataComponent, typename T2 = IDataComponent>
  JobHandle ForEachStorage(const std::vector<JobHandle>& dependencies, DataComponentStorage& storage,
                           std::function<void(int i, Entity entity, T1&, T2&)>&& func, bool check_enable = true);

  /**
   * @brief Iterates over all entities in storage and applies the given function to each.
   * @tparam T1 The first type of the data component.
   * @tparam T2 The second type of the data component.
   * @tparam T3 The third type of the data component.
   * @param dependencies A vector of job handles for dependencies.
   * @param storage The data component storage.
   * @param func The function to apply.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle for the operation.
   */
  template <typename T1 = IDataComponent, typename T2 = IDataComponent, typename T3 = IDataComponent>
  JobHandle ForEachStorage(const std::vector<JobHandle>& dependencies, DataComponentStorage& storage,
                           std::function<void(int i, Entity entity, T1&, T2&, T3&)>&& func, bool check_enable = true);

  /**
   * @brief Iterates over all entities in storage and applies the given function to each.
   * @tparam T1 The first type of the data component.
   * @tparam T2 The second type of the data component.
   * @tparam T3 The third type of the data component.
   * @tparam T4 The fourth type of the data component.
   * @param dependencies A vector of job handles for dependencies.
   * @param storage The data component storage.
   * @param func The function to apply.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle for the operation.
   */
  template <typename T1 = IDataComponent, typename T2 = IDataComponent, typename T3 = IDataComponent,
            typename T4 = IDataComponent>
  JobHandle ForEachStorage(const std::vector<JobHandle>& dependencies, DataComponentStorage& storage,
                           std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&)>&& func,
                           bool check_enable = true);

  /**
   * @brief Iterates over all entities in storage and applies the given function to each.
   * @tparam T1 The first type of the data component.
   * @tparam T2 The second type of the data component.
   * @tparam T3 The third type of the data component.
   * @tparam T4 The fourth type of the data component.
   * @tparam T5 The fifth type of the data component.
   * @param dependencies A vector of job handles for dependencies.
   * @param storage The data component storage.
   * @param func The function to apply.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle for the operation.
   */
  template <typename T1 = IDataComponent, typename T2 = IDataComponent, typename T3 = IDataComponent,
            typename T4 = IDataComponent, typename T5 = IDataComponent>
  JobHandle ForEachStorage(const std::vector<JobHandle>& dependencies, DataComponentStorage& storage,
                           std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&)>&& func,
                           bool check_enable = true);

  /**
   * @brief Iterates over all entities in storage and applies the given function to each.
   * @tparam T1 The first type of the data component.
   * @tparam T2 The second type of the data component.
   * @tparam T3 The third type of the data component.
   * @tparam T4 The fourth type of the data component.
   * @tparam T5 The fifth type of the data component.
   * @tparam T6 The sixth type of the data component.
   * @param dependencies A vector of job handles for dependencies.
   * @param storage The data component storage.
   * @param func The function to apply.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle for the operation.
   */
  template <typename T1 = IDataComponent, typename T2 = IDataComponent, typename T3 = IDataComponent,
            typename T4 = IDataComponent, typename T5 = IDataComponent, typename T6 = IDataComponent>
  JobHandle ForEachStorage(const std::vector<JobHandle>& dependencies, DataComponentStorage& storage,
                           std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&, T6&)>&& func,
                           bool check_enable = true);

  /**
   * @brief Iterates over all entities in storage and applies the given function to each.
   * @tparam T1 The first type of the data component.
   * @tparam T2 The second type of the data component.
   * @tparam T3 The third type of the data component.
   * @tparam T4 The fourth type of the data component.
   * @tparam T5 The fifth type of the data component.
   * @tparam T6 The sixth type of the data component.
   * @tparam T7 The seventh type of the data component.
   * @param dependencies A vector of job handles for dependencies.
   * @param storage The data component storage.
   * @param func The function to apply.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle for the operation.
   */
  template <typename T1 = IDataComponent, typename T2 = IDataComponent, typename T3 = IDataComponent,
            typename T4 = IDataComponent, typename T5 = IDataComponent, typename T6 = IDataComponent,
            typename T7 = IDataComponent>
  JobHandle ForEachStorage(const std::vector<JobHandle>& dependencies, DataComponentStorage& storage,
                           std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&, T6&, T7&)>&& func,
                           bool check_enable = true);

  /**
   * @brief Iterates over all entities in storage and applies the given function to each.
   * @tparam T1 The first type of the data component.
   * @tparam T2 The second type of the data component.
   * @tparam T3 The third type of the data component.
   * @tparam T4 The fourth type of the data component.
   * @tparam T5 The fifth type of the data component.
   * @tparam T6 The sixth type of the data component.
   * @tparam T7 The seventh type of the data component.
   * @tparam T8 The eighth type of the data component.
   * @param dependencies A vector of job handles for dependencies.
   * @param storage The data component storage.
   * @param func The function to apply.
   * @param check_enable Whether to check if the entities are enabled.
   * @return A JobHandle for the operation.
   */
  template <typename T1 = IDataComponent, typename T2 = IDataComponent, typename T3 = IDataComponent,
            typename T4 = IDataComponent, typename T5 = IDataComponent, typename T6 = IDataComponent,
            typename T7 = IDataComponent, typename T8 = IDataComponent>
  JobHandle ForEachStorage(const std::vector<JobHandle>& dependencies, DataComponentStorage& storage,
                           std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&, T6&, T7&, T8&)>&& func,
                           bool check_enable = true);

  /**
   * @brief Retrieves the metadata for a specified entity.
   * @param entity The entity whose metadata is being requested.
   * @return Reference to the metadata of the specified entity.
   */
  [[nodiscard]] EntityMetadata& GetEntityMetadata(const Entity& entity);

  /**
   * @brief Checks if a system of a specific type exists in the scene.
   * @param type_id The type ID of the system to check.
   * @return True if the system exists, false otherwise.
   */
  bool HasSystem(const size_t& type_id);

  /**
   * @brief Creates a system of a specific type and adds it to the scene.
   * @param type_id The type ID of the system to create.
   * @param order The execution order of the system.
   * @return Shared pointer to the created system.
   */
  std::shared_ptr<ISystem> CreateSystem(const size_t& type_id, float order);

  /**
   * @brief Adds a private component to the entity.
   * @param entity The entity to add the private component to.
   * @param type_id The type ID of the private component.
   */
  void AddPrivateComponent(const Entity& entity, const size_t& type_id);

 protected:
  /**
   * @brief Loads the scene from the specified file path.
   * @param path The file path to load the scene from.
   * @return True if the scene was loaded successfully, false otherwise.
   */
  bool LoadInternal(const std::filesystem::path& path);

  /**
   * @brief Scene YAML can be parsed off-thread, but scene reconstruction must finalize on the main thread.
   */
  [[nodiscard]] bool SupportsStagedLoading(const std::filesystem::path& path) const;

  /**
   * @brief Parses scene YAML into a staged payload.
   */
  [[nodiscard]] std::shared_ptr<StagedAssetLoadPayload> LoadStagedPayloadInternal(
      const std::filesystem::path& path) const;

  /**
   * @brief Reconstructs the scene from a staged YAML payload.
   */
  bool ApplyStagedPayloadInternal(const std::filesystem::path& path,
                                  const std::shared_ptr<StagedAssetLoadPayload>& payload);
};
template <typename T>
std::vector<Entity> Scene::GetPrivateComponentOwnersList() {
  return scene_data_storage_.entity_private_component_storage.GetOwnersList<T>();
}
template <typename T>
std::shared_ptr<T> Scene::GetSystem() {
  if (const auto search = indexed_systems_.find(typeid(T).hash_code()); search != indexed_systems_.end())
    return std::dynamic_pointer_cast<T>(search->second);
  return nullptr;
}
template <typename T>
bool Scene::HasSystem() {
  if (const auto search = indexed_systems_.find(typeid(T).hash_code()); search != indexed_systems_.end())
    return true;
  return false;
}

template <typename T>
void Scene::DestroySystem() {
  auto system = GetSystem<T>();
  if (system != nullptr)
    return;
  indexed_systems_.erase(typeid(T).hash_code());
  for (auto& i : systems_) {
    if (i.second.get() == system.get()) {
      systems_.erase(i.first);
      return;
    }
  }
}
template <typename T>
std::shared_ptr<T> Scene::GetOrCreateSystem(const float rank) {
  if (const auto search = indexed_systems_.find(typeid(T).hash_code()); search != indexed_systems_.end())
    return std::dynamic_pointer_cast<T>(search->second);
  return std::dynamic_pointer_cast<T>(CreateSystem(typeid(T).hash_code(), rank));
}

#pragma region GetSetHas
template <typename T>
void Scene::AddDataComponent(const Entity& entity, const T& value) {
  assert(IsEntityValid(entity));
  const auto id = typeid(T).hash_code();
  auto& entity_info = scene_data_storage_.entity_metadata_list.at(entity.index_);

#pragma region Check if componentdata already exists.If yes, go to SetComponentData
  const auto& data_component_storage =
      scene_data_storage_.data_component_storage_list.at(entity_info.data_component_storage_index);
  const auto original_component_types = data_component_storage.data_component_types;
  for (const auto& type : data_component_storage.data_component_types) {
    if (type.type_index == id) {
      EVOENGINE_ERROR("Data Component already exists!");
      return;
    }
  }
#pragma endregion
#pragma region If not exist, we first need to create a new archetype
  EntityArchetypeInfo new_archetype_info;
  new_archetype_info.archetype_name = "New archetype";
  new_archetype_info.data_component_types = original_component_types;
  new_archetype_info.data_component_types.push_back(Typeof<T>());
  std::sort(new_archetype_info.data_component_types.begin() + 3, new_archetype_info.data_component_types.end(),
            ComponentTypeComparator);
  size_t offset = 0;
  DataComponentType prev = new_archetype_info.data_component_types[0];
  // Erase duplicates

  std::vector<DataComponentType> copy;
  copy.insert(copy.begin(), new_archetype_info.data_component_types.begin(),
              new_archetype_info.data_component_types.end());
  new_archetype_info.data_component_types.clear();
  for (const auto& i : copy) {
    bool found = false;
    for (const auto& j : new_archetype_info.data_component_types) {
      if (i == j) {
        found = true;
        break;
      }
    }
    if (found)
      continue;
    new_archetype_info.data_component_types.push_back(i);
  }

  for (auto& i : new_archetype_info.data_component_types) {
    i.type_offset = offset;
    offset += i.type_size;
  }
  new_archetype_info.entity_size = new_archetype_info.data_component_types.back().type_offset +
                                   new_archetype_info.data_component_types.back().type_size;
  new_archetype_info.chunk_capacity = Entities::GetArchetypeChunkSize() / new_archetype_info.entity_size;
  const auto archetype = Entities::CreateEntityArchetypeHelper(new_archetype_info);
#pragma endregion
#pragma region Create new Entity with new archetype.
  Entity new_entity = CreateEntity(archetype);
  auto& original_entity_info = scene_data_storage_.entity_metadata_list.at(entity.index_);
  // Transfer component data
  for (const auto& type : original_component_types) {
    SetDataComponent(new_entity.index_, type.type_index, type.type_size,
                     GetDataComponentPointer(entity, type.type_index));
  }
  SetDataComponent(new_entity, value);
  // 5. Swap entity.
  auto& new_entity_info = scene_data_storage_.entity_metadata_list.at(new_entity.index_);
  const auto temp_archetype_info_index = new_entity_info.data_component_storage_index;
  const auto temp_chunk_array_index = new_entity_info.chunk_array_index;
  new_entity_info.data_component_storage_index = original_entity_info.data_component_storage_index;
  new_entity_info.chunk_array_index = original_entity_info.chunk_array_index;
  original_entity_info.data_component_storage_index = temp_archetype_info_index;
  original_entity_info.chunk_array_index = temp_chunk_array_index;
  // Apply to chunk.
  scene_data_storage_.data_component_storage_list.at(original_entity_info.data_component_storage_index)
      .chunk_array.entity_array[original_entity_info.chunk_array_index] = entity;
  scene_data_storage_.data_component_storage_list.at(new_entity_info.data_component_storage_index)
      .chunk_array.entity_array[new_entity_info.chunk_array_index] = new_entity;
  DeleteEntity(new_entity);
#pragma endregion
  SetUnsaved();
}

template <typename T>
void Scene::RemoveDataComponent(const Entity& entity) {
  assert(IsEntityValid(entity));
  const auto id = typeid(T).hash_code();
  if (id == typeid(Transform).hash_code() || id == typeid(GlobalTransform).hash_code() ||
      id == typeid(TransformUpdateFlag).hash_code()) {
    return;
  }
  const auto& entity_info = scene_data_storage_.entity_metadata_list.at(entity.index_);
#pragma region Check if componentdata already exists.If yes, go to SetComponentData
  const auto& data_component_storage =
      scene_data_storage_.data_component_storage_list[entity_info.data_component_storage_index];
  if (data_component_storage.data_component_types.size() <= 3) {
    EVOENGINE_ERROR(
        "Remove Component Data failed: Entity must have at least 1 data component besides 3 basic data "
        "components!");
    return;
  }
#pragma region Create new archetype
  EntityArchetypeInfo new_archetype_info;
  new_archetype_info.archetype_name = "New archetype";
  new_archetype_info.data_component_types = data_component_storage.data_component_types;
  bool found = false;
  for (int i = 0; i < new_archetype_info.data_component_types.size(); i++) {
    if (new_archetype_info.data_component_types[i].type_index == id) {
      new_archetype_info.data_component_types.erase(new_archetype_info.data_component_types.begin() + i);
      found = true;
      break;
    }
  }
  if (!found) {
    EVOENGINE_ERROR("Failed to remove component data: Component not found");
    return;
  }
  size_t offset = 0;
  for (auto& i : new_archetype_info.data_component_types) {
    i.type_offset = offset;
    offset += i.type_size;
  }
  new_archetype_info.entity_size = new_archetype_info.data_component_types.back().type_offset +
                                   new_archetype_info.data_component_types.back().type_size;
  new_archetype_info.chunk_capacity = Entities::GetArchetypeChunkSize() / new_archetype_info.entity_size;
  const auto archetype = Entities::CreateEntityArchetypeHelper(new_archetype_info);
#pragma endregion
#pragma region Create new Entity with new archetype
  const Entity new_entity = CreateEntity(archetype);
  auto& original_entity_info = scene_data_storage_.entity_metadata_list.at(entity.index_);
  // Transfer component data
  for (const auto& type : new_archetype_info.data_component_types) {
    SetDataComponent(new_entity.index_, type.type_index, type.type_size,
                     GetDataComponentPointer(entity, type.type_index));
  }
  T return_value = GetDataComponent<T>(entity);
  // 5. Swap entity.
  EntityMetadata& new_entity_info = scene_data_storage_.entity_metadata_list.at(new_entity.index_);
  const auto temp_archetype_info_index = new_entity_info.data_component_storage_index;
  const auto temp_chunk_array_index = new_entity_info.chunk_array_index;
  new_entity_info.data_component_storage_index = original_entity_info.data_component_storage_index;
  new_entity_info.chunk_array_index = original_entity_info.chunk_array_index;
  original_entity_info.data_component_storage_index = temp_archetype_info_index;
  original_entity_info.chunk_array_index = temp_chunk_array_index;
  // Apply to chunk.
  scene_data_storage_.data_component_storage_list.at(original_entity_info.data_component_storage_index)
      .chunk_array.entity_array[original_entity_info.chunk_array_index] = entity;
  scene_data_storage_.data_component_storage_list.at(new_entity_info.data_component_storage_index)
      .chunk_array.entity_array[new_entity_info.chunk_array_index] = new_entity;
  DeleteEntity(new_entity);
#pragma endregion
  SetUnsaved();
}

template <typename T>
void Scene::SetDataComponent(const Entity& entity, const T& value) {
  assert(IsEntityValid(entity));
  SetDataComponent(entity.index_, typeid(T).hash_code(), sizeof(T), static_cast<const void*>(&value));
}
template <typename T>
void Scene::SetDataComponent(const size_t& index, const T& value) {
  const size_t id = typeid(T).hash_code();
  assert(index < scene_data_storage_.entity_metadata_list.size());
  SetDataComponent(index, id, sizeof(T), static_cast<void*>(&value));
}
template <typename T>
T Scene::GetDataComponent(const Entity& entity) {
  assert(IsEntityValid(entity));
  const EntityMetadata& entity_info = scene_data_storage_.entity_metadata_list.at(entity.index_);
  const auto& data_component_storage =
      scene_data_storage_.data_component_storage_list[entity_info.data_component_storage_index];
  const size_t chunk_index = entity_info.chunk_array_index / data_component_storage.chunk_capacity;
  const size_t chunk_pointer = entity_info.chunk_array_index % data_component_storage.chunk_capacity;
  const ComponentDataChunk& chunk = data_component_storage.chunk_array.chunks[chunk_index];
  const size_t id = typeid(T).hash_code();
  if (id == typeid(Transform).hash_code()) {
    return chunk.GetData<T>(chunk_pointer * sizeof(Transform));
  }
  if (id == typeid(GlobalTransform).hash_code()) {
    return chunk.GetData<T>(sizeof(Transform) * data_component_storage.chunk_capacity +
                            chunk_pointer * sizeof(GlobalTransform));
  }
  if (id == typeid(TransformUpdateFlag).hash_code()) {
    return chunk.GetData<T>((sizeof(Transform) + sizeof(GlobalTransform)) * data_component_storage.chunk_capacity +
                            chunk_pointer * sizeof(TransformUpdateFlag));
  }
  for (const auto& type : data_component_storage.data_component_types) {
    if (type.type_index == id) {
      return chunk.GetData<T>(type.type_offset * data_component_storage.chunk_capacity + chunk_pointer * sizeof(T));
    }
  }
  EVOENGINE_WARNING("ComponentData doesn't exist")
  return T();
}
template <typename T>
bool Scene::HasDataComponent(const Entity& entity) const {
  assert(IsEntityValid(entity));

  const EntityMetadata& entity_info = scene_data_storage_.entity_metadata_list.at(entity.index_);
  const auto& data_component_storage =
      scene_data_storage_.data_component_storage_list[entity_info.data_component_storage_index];
  const size_t id = typeid(T).hash_code();
  if (id == typeid(Transform).hash_code()) {
    return true;
  }
  if (id == typeid(GlobalTransform).hash_code()) {
    return true;
  }
  if (id == typeid(TransformUpdateFlag).hash_code()) {
    return true;
  }
  for (const auto& type : data_component_storage.data_component_types) {
    if (type.type_index == id) {
      return true;
    }
  }
  return false;
}
template <typename T>
T Scene::GetDataComponent(const size_t& index) {
  if (index > scene_data_storage_.entity_metadata_list.size())
    return T();
  const EntityMetadata& entity_info = scene_data_storage_.entity_metadata_list.at(index);
  auto& data_component_storage =
      scene_data_storage_.data_component_storage_list[entity_info.data_component_storage_index];
  const size_t chunk_index = entity_info.chunk_array_index / data_component_storage.chunk_capacity;
  const size_t chunk_pointer = entity_info.chunk_array_index % data_component_storage.chunk_capacity;
  const ComponentDataChunk& chunk = data_component_storage.chunk_array.chunks[chunk_index];
  const size_t id = typeid(T).hash_code();
  if (id == typeid(Transform).hash_code()) {
    return chunk.GetData<T>(chunk_pointer * sizeof(Transform));
  }
  if (id == typeid(GlobalTransform).hash_code()) {
    return chunk.GetData<T>(sizeof(Transform) * data_component_storage.chunk_capacity +
                            chunk_pointer * sizeof(GlobalTransform));
  }
  if (id == typeid(TransformUpdateFlag).hash_code()) {
    return chunk.GetData<T>((sizeof(Transform) + sizeof(GlobalTransform)) * data_component_storage.chunk_capacity +
                            chunk_pointer * sizeof(TransformUpdateFlag));
  }
  for (const auto& type : data_component_storage.data_component_types) {
    if (type.type_index == id) {
      return chunk.GetData<T>(type.type_offset * data_component_storage.chunk_capacity + chunk_pointer * sizeof(T));
    }
  }
  EVOENGINE_WARNING("ComponentData doesn't exist")
  return T();
}
template <typename T>
bool Scene::HasDataComponent(const size_t& index) const {
  return HasDataComponent(index, typeid(T).hash_code());
}

template <typename T>
std::weak_ptr<T> Scene::GetOrSetPrivateComponent(const Entity& entity) {
  assert(IsEntityValid(entity));

  auto type_name = Serialization::GetSerializableTypeName<T>();
  size_t i = 0;
  auto& elements = scene_data_storage_.entity_metadata_list.at(entity.index_).private_component_elements;
  for (const auto& element : elements) {
    if (type_name == element.private_component_data->GetTypeName()) {
      return std::static_pointer_cast<T>(element.private_component_data);
    }
    i++;
  }
  auto ptr = scene_data_storage_.entity_private_component_storage.GetOrSetPrivateComponent<T>(entity);
  elements.emplace_back(typeid(T).hash_code(), ptr, entity, std::dynamic_pointer_cast<Scene>(GetSelf()));
  MarkRenderStructureChanged();
  SetUnsaved();
  return std::move(ptr);
}
template <typename T>
void Scene::RemovePrivateComponent(const Entity& entity) {
  assert(IsEntityValid(entity));

  auto& elements = scene_data_storage_.entity_metadata_list.at(entity.index_).private_component_elements;
  for (auto i = 0; i < elements.size(); i++) {
    if (std::dynamic_pointer_cast<T>(elements[i].private_component_data)) {
      scene_data_storage_.entity_private_component_storage.RemovePrivateComponent<T>(
          entity, elements[i].private_component_data);
      elements.erase(elements.begin() + i);
      MarkRenderStructureChanged();
      SetUnsaved();
      return;
    }
  }
}

template <typename T>
bool Scene::HasPrivateComponent(const Entity& entity) const {
  assert(IsEntityValid(entity));
  auto& entity_metadata = scene_data_storage_.entity_metadata_list.at(entity.index_);
  for (auto& element : entity_metadata.private_component_elements) {
    if (std::dynamic_pointer_cast<T>(element.private_component_data)) {
      return true;
    }
  }
  return false;
}

#pragma endregion
#pragma region For Each
template <typename T1>
JobHandle Scene::ForEach(const std::vector<JobHandle>& dependencies, const EntityQuery& entity_query,
                         std::function<void(int i, Entity entity, T1&)>&& func, bool check_enable) {
  assert(entity_query.IsValid());
  std::vector<JobHandle> jobs;
  const auto queried_storage_list = QueryDataComponentStorageList(entity_query);
  for (const auto i : queried_storage_list) {
    if (const auto job = ForEachStorage(dependencies, i.get(), std::move(func), check_enable); job.Valid())
      jobs.emplace_back(job);
  }
  return Jobs::Combine(jobs);
}
template <typename T1, typename T2>
JobHandle Scene::ForEach(const std::vector<JobHandle>& dependencies, const EntityQuery& entity_query,
                         std::function<void(int i, Entity entity, T1&, T2&)>&& func, bool check_enable) {
  assert(entity_query.IsValid());
  const auto queried_storage_list = QueryDataComponentStorageList(entity_query);
  std::vector<JobHandle> jobs;
  for (const auto i : queried_storage_list) {
    if (const auto job = ForEachStorage(dependencies, i.get(), std::move(func), check_enable); job.Valid())
      jobs.emplace_back(job);
  }
  return Jobs::Combine(jobs);
}
template <typename T1, typename T2, typename T3>
JobHandle Scene::ForEach(const std::vector<JobHandle>& dependencies, const EntityQuery& entity_query,
                         std::function<void(int i, Entity entity, T1&, T2&, T3&)>&& func, bool check_enable) {
  assert(entity_query.IsValid());
  const auto queried_storage_list = QueryDataComponentStorageList(entity_query);
  std::vector<JobHandle> jobs;
  for (const auto i : queried_storage_list) {
    if (const auto job = ForEachStorage(dependencies, i.get(), std::move(func), check_enable); job.Valid())
      jobs.emplace_back(job);
  }
  return Jobs::Combine(jobs);
}
template <typename T1, typename T2, typename T3, typename T4>
JobHandle Scene::ForEach(const std::vector<JobHandle>& dependencies, const EntityQuery& entity_query,
                         std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&)>&& func, bool check_enable) {
  assert(entity_query.IsValid());
  const auto queried_storage_list = QueryDataComponentStorageList(entity_query);
  std::vector<JobHandle> jobs;
  for (const auto i : queried_storage_list) {
    if (const auto job = ForEachStorage(dependencies, i.get(), std::move(func), check_enable); job.Valid())
      jobs.emplace_back(job);
  }
  return Jobs::Combine(jobs);
}
template <typename T1, typename T2, typename T3, typename T4, typename T5>
JobHandle Scene::ForEach(const std::vector<JobHandle>& dependencies, const EntityQuery& entity_query,
                         std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&)>&& func, bool check_enable) {
  assert(entity_query.IsValid());
  const auto queried_storage_list = QueryDataComponentStorageList(entity_query);
  std::vector<JobHandle> jobs;
  for (const auto i : queried_storage_list) {
    if (const auto job = ForEachStorage(dependencies, i.get(), std::move(func), check_enable); job.Valid())
      jobs.emplace_back(job);
  }
  return Jobs::Combine(jobs);
}
template <typename T1, typename T2, typename T3, typename T4, typename T5, typename T6>
JobHandle Scene::ForEach(const std::vector<JobHandle>& dependencies, const EntityQuery& entity_query,
                         std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&, T6&)>&& func,
                         const bool check_enable) {
  assert(entity_query.IsValid());
  const auto queried_storage_list = QueryDataComponentStorageList(entity_query);
  std::vector<JobHandle> jobs;
  for (const auto i : queried_storage_list) {
    if (const auto job = ForEachStorage(dependencies, i.get(), std::move(func), check_enable); job.Valid())
      jobs.emplace_back(job);
  }
  return Jobs::Combine(jobs);
}
template <typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7>
JobHandle Scene::ForEach(const std::vector<JobHandle>& dependencies, const EntityQuery& entity_query,
                         std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&, T6&, T7&)>&& func,
                         const bool check_enable) {
  assert(entity_query.IsValid());
  const auto queried_storage_list = QueryDataComponentStorageList(entity_query);
  std::vector<JobHandle> jobs;
  for (const auto i : queried_storage_list) {
    if (const auto job = ForEachStorage(dependencies, i.get(), std::move(func), check_enable); job.Valid())
      jobs.emplace_back(job);
  }
  return Jobs::Combine(jobs);
}
template <typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7, typename T8>
JobHandle Scene::ForEach(const std::vector<JobHandle>& dependencies, const EntityQuery& entity_query,
                         std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&, T6&, T7&, T8&)>&& func,
                         const bool check_enable) {
  assert(entity_query.IsValid());
  const auto queried_storage_list = QueryDataComponentStorageList(entity_query);
  std::vector<JobHandle> jobs;
  for (const auto i : queried_storage_list) {
    if (const auto job = ForEachStorage(dependencies, i.get(), std::move(func), check_enable); job.Valid())
      jobs.emplace_back(job);
  }
  return Jobs::Combine(jobs);
}

template <typename T1>
JobHandle Scene::ForEach(const std::vector<JobHandle>& dependencies,
                         std::function<void(int i, Entity entity, T1&)>&& func, bool check_enable) {
  auto& storage_list = scene_data_storage_.data_component_storage_list;
  std::vector<JobHandle> jobs;
  for (auto i = storage_list.begin() + 1; i < storage_list.end(); ++i) {
    if (const auto job = ForEachStorage(dependencies, *i, std::move(func), check_enable); job.Valid())
      jobs.emplace_back(job);
  }
  return Jobs::Combine(jobs);
}

template <typename T1, typename T2>
JobHandle Scene::ForEach(const std::vector<JobHandle>& dependencies,
                         std::function<void(int i, Entity entity, T1&, T2&)>&& func, bool check_enable) {
  auto& storage_list = scene_data_storage_.data_component_storage_list;
  std::vector<JobHandle> jobs;
  for (auto i = storage_list.begin() + 1; i < storage_list.end(); ++i) {
    if (const auto job = ForEachStorage(dependencies, *i, std::move(func), check_enable); job.Valid())
      jobs.emplace_back(job);
  }
  return Jobs::Combine(jobs);
}

template <typename T1, typename T2, typename T3>
JobHandle Scene::ForEach(const std::vector<JobHandle>& dependencies,
                         std::function<void(int i, Entity entity, T1&, T2&, T3&)>&& func, bool check_enable) {
  auto& storage_list = scene_data_storage_.data_component_storage_list;
  std::vector<JobHandle> jobs;
  for (auto i = storage_list.begin() + 1; i < storage_list.end(); ++i) {
    if (const auto job = ForEachStorage(dependencies, *i, std::move(func), check_enable); job.Valid())
      jobs.emplace_back(job);
  }
  return Jobs::Combine(jobs);
}

template <typename T1, typename T2, typename T3, typename T4>
JobHandle Scene::ForEach(const std::vector<JobHandle>& dependencies,
                         std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&)>&& func, bool check_enable) {
  auto& storage_list = scene_data_storage_.data_component_storage_list;
  std::vector<JobHandle> jobs;
  for (auto i = storage_list.begin() + 1; i < storage_list.end(); ++i) {
    if (const auto job = ForEachStorage(dependencies, *i, std::move(func), check_enable); job.Valid())
      jobs.emplace_back(job);
  }
  return Jobs::Combine(jobs);
}

template <typename T1, typename T2, typename T3, typename T4, typename T5>
JobHandle Scene::ForEach(const std::vector<JobHandle>& dependencies,
                         std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&)>&& func, bool check_enable) {
  auto& storage_list = scene_data_storage_.data_component_storage_list;
  std::vector<JobHandle> jobs;
  for (auto i = storage_list.begin() + 1; i < storage_list.end(); ++i) {
    if (const auto job = ForEachStorage(dependencies, *i, std::move(func), check_enable); job.Valid())
      jobs.emplace_back(job);
  }
  return Jobs::Combine(jobs);
}

template <typename T1, typename T2, typename T3, typename T4, typename T5, typename T6>
JobHandle Scene::ForEach(const std::vector<JobHandle>& dependencies,
                         std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&, T6&)>&& func,
                         const bool check_enable) {
  auto& storage_list = scene_data_storage_.data_component_storage_list;
  std::vector<JobHandle> jobs;
  for (auto i = storage_list.begin() + 1; i < storage_list.end(); ++i) {
    if (const auto job = ForEachStorage(dependencies, *i, std::move(func), check_enable); job.Valid())
      jobs.emplace_back(job);
  }
  return Jobs::Combine(jobs);
}

template <typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7>
JobHandle Scene::ForEach(const std::vector<JobHandle>& dependencies,
                         std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&, T6&, T7&)>&& func,
                         const bool check_enable) {
  auto& storage_list = scene_data_storage_.data_component_storage_list;
  std::vector<JobHandle> jobs;
  for (auto i = storage_list.begin() + 1; i < storage_list.end(); ++i) {
    if (const auto job = ForEachStorage(dependencies, *i, std::move(func), check_enable); job.Valid())
      jobs.emplace_back(job);
  }
  return Jobs::Combine(jobs);
}

template <typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7, typename T8>
JobHandle Scene::ForEach(const std::vector<JobHandle>& dependencies,
                         std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&, T6&, T7&, T8&)>&& func,
                         const bool check_enable) {
  auto& storage_list = scene_data_storage_.data_component_storage_list;
  std::vector<JobHandle> jobs;
  for (auto i = storage_list.begin() + 1; i < storage_list.end(); ++i) {
    if (const auto job = ForEachStorage(dependencies, *i, std::move(func), check_enable); job.Valid())
      jobs.emplace_back(job);
  }
  return Jobs::Combine(jobs);
}

#pragma endregion
template <typename T>
void Scene::GetComponentDataArray(const EntityQuery& entity_query, std::vector<T>& container, bool check_enable) {
  assert(entity_query.IsValid());
  const auto queried_storage_list = QueryDataComponentStorageList(entity_query);
  for (const auto i : queried_storage_list) {
    GetDataComponentArrayStorage(i.get(), container, check_enable);
  }
}

template <typename T1, typename T2>
void Scene::GetComponentDataArray(const EntityQuery& entity_query, std::vector<T1>& container,
                                  std::function<bool(const T2&)>&& filter_func, bool check_enable) {
  assert(entity_query.IsValid());
  std::vector<T2> component_data_list;
  std::vector<T1> target_data_list;
  GetComponentDataArray(entity_query, component_data_list, check_enable);
  GetComponentDataArray(entity_query, target_data_list, check_enable);
  if (target_data_list.size() != component_data_list.size())
    return;
  size_t size = component_data_list.size();
  std::vector<std::vector<T1>> collected_data_lists;
  const auto thread_size = Jobs::GetWorkerSize();
  for (int i = 0; i < thread_size; i++) {
    collected_data_lists.push_back(std::vector<T1>());
  }
  Jobs::RunParallelFor(
      size,
      [&target_data_list, &component_data_list, &collected_data_lists, filter_func](size_t i, size_t thread_index) {
        if (filter_func(component_data_list[i])) {
          collected_data_lists[thread_index].push_back(target_data_list[i]);
        }
      },
      thread_size);

  for (int i = 0; i < collected_data_lists.size(); i++) {
    auto list_size = collected_data_lists[i].size();
    if (list_size == 0)
      continue;
    container.resize(container.size() + list_size);
    memcpy(&container.at(container.size() - list_size), collected_data_lists[i].data(), list_size * sizeof(T1));
  }

  const size_t remainder = size % thread_size;
  for (int i = 0; i < remainder; i++) {
    if (filter_func(component_data_list[size - remainder + i])) {
      container.push_back(target_data_list[size - remainder + i]);
    }
  }
}

template <typename T1, typename T2, typename T3>
void Scene::GetComponentDataArray(const EntityQuery& entity_query, std::vector<T1>& container,
                                  std::function<bool(const T2&, const T3&)>&& filter_func, bool check_enable) {
  assert(entity_query.IsValid());
  std::vector<T3> component_data_list2;
  std::vector<T2> component_data_list1;
  std::vector<T1> target_data_list;
  GetComponentDataArray(entity_query, component_data_list2, check_enable);
  GetComponentDataArray(entity_query, component_data_list1, check_enable);
  GetComponentDataArray(entity_query, target_data_list, check_enable);
  if (target_data_list.size() != component_data_list1.size() ||
      component_data_list1.size() != component_data_list2.size())
    return;
  size_t size = component_data_list1.size();
  std::vector<std::vector<T1>> collected_data_lists;
  const auto thread_size = Jobs::GetWorkerSize();
  for (int i = 0; i < thread_size; i++) {
    collected_data_lists.push_back(std::vector<T1>());
  }
  Jobs::RunParallelFor(
      size,
      [&target_data_list, &component_data_list1, &component_data_list2, &collected_data_lists, filter_func](
          size_t i, size_t thread_index) {
        if (filter_func(component_data_list1[i], component_data_list2[i])) {
          collected_data_lists.at(thread_index).push_back(target_data_list[i]);
        }
      },
      thread_size);
  for (int i = 0; i < collected_data_lists.size(); i++) {
    auto list_size = collected_data_lists[i].size();
    if (list_size == 0)
      continue;
    container.resize(container.size() + list_size);
    memcpy(&container.at(container.size() - list_size), collected_data_lists[i].data(), list_size * sizeof(T1));
  }

  const size_t remainder = size % thread_size;
  for (int i = 0; i < remainder; i++) {
    if (filter_func(component_data_list1[size - remainder + i], component_data_list2[size - remainder + i])) {
      container.push_back(target_data_list[size - remainder + i]);
    }
  }
}

template <typename T1, typename T2>
void Scene::GetComponentDataArray(const EntityQuery& entity_query, const T1& filter, std::vector<T2>& container,
                                  const bool check_enable) {
  assert(entity_query.IsValid());
  std::vector<T1> component_data_list;
  std::vector<T2> target_data_list;
  GetComponentDataArray(entity_query, component_data_list, check_enable);
  GetComponentDataArray(entity_query, target_data_list, check_enable);
  if (target_data_list.size() != component_data_list.size())
    return;
  std::vector<std::shared_future<void>> futures;
  size_t size = component_data_list.size();
  std::vector<std::vector<T2>> collected_data_lists;
  const auto thread_size = Jobs::GetWorkerSize();
  for (int i = 0; i < thread_size; i++) {
    collected_data_lists.push_back(std::vector<T2>());
  }
  Jobs::RunParallelFor(
      size,
      [&target_data_list, &component_data_list, filter, &collected_data_lists](size_t i, size_t thread_index) {
        if (filter == component_data_list[i]) {
          collected_data_lists.at(thread_index)->push_back(target_data_list[i]);
        }
      },
      thread_size);

  for (int i = 0; i < collected_data_lists.size(); i++) {
    auto list_size = collected_data_lists[i].size();
    if (list_size == 0)
      continue;
    container.resize(container.size() + list_size);
    memcpy(&container.at(container.size() - list_size), collected_data_lists[i].data(), list_size * sizeof(T2));
  }

  const size_t remainder = size % thread_size;
  for (int i = 0; i < remainder; i++) {
    if (filter == component_data_list[size - remainder + i]) {
      container.push_back(target_data_list[size - remainder + i]);
    }
  }
}

template <typename T1>
void Scene::GetEntityArray(const EntityQuery& entity_query, std::vector<Entity>& container,
                           std::function<bool(const Entity&, const T1&)>&& filter_func, bool check_enable) {
  assert(entity_query.IsValid());
  std::vector<Entity> all_entities;
  std::vector<T1> component_data_list;
  GetEntityArray(entity_query, all_entities, check_enable);
  GetComponentDataArray(entity_query, component_data_list, check_enable);
  if (all_entities.size() != component_data_list.size())
    return;
  std::vector<std::shared_future<void>> futures;
  size_t size = all_entities.size();
  std::vector<std::vector<Entity>> collected_entity_lists;
  const auto thread_size = Jobs::GetWorkerSize();
  for (int i = 0; i < thread_size; i++) {
    collected_entity_lists.push_back(std::vector<Entity>());
  }
  Jobs::RunParallelFor(
      size,
      [&all_entities, &component_data_list, &collected_entity_lists, filter_func](size_t i, size_t thread_index) {
        if (filter_func(all_entities[i], component_data_list[i])) {
          collected_entity_lists.at(thread_index).push_back(all_entities[i]);
        }
      },
      thread_size);
  for (int i = 0; i < collected_entity_lists.size(); i++) {
    const auto list_size = collected_entity_lists[i].size();
    if (list_size == 0)
      continue;
    container.resize(container.size() + list_size);
    memcpy(&container.at(container.size() - list_size), collected_entity_lists[i].data(), list_size * sizeof(Entity));
  }

  const size_t remainder = size % thread_size;
  for (int i = 0; i < remainder; i++) {
    if (filter_func(all_entities[size - remainder + i], component_data_list[size - remainder + i])) {
      container.push_back(all_entities[size - remainder + i]);
    }
  }
}

template <typename T1, typename T2>
void Scene::GetEntityArray(const EntityQuery& entity_query, std::vector<Entity>& container,
                           std::function<bool(const Entity&, const T1&, const T2&)>&& filter_func, bool check_enable) {
  assert(entity_query.IsValid());
  std::vector<Entity> all_entities;
  std::vector<T1> component_data_list1;
  std::vector<T2> component_data_list2;
  GetEntityArray(entity_query, all_entities, check_enable);
  GetComponentDataArray(entity_query, component_data_list1, check_enable);
  GetComponentDataArray(entity_query, component_data_list2, check_enable);
  if (all_entities.size() != component_data_list1.size() || component_data_list1.size() != component_data_list2.size())
    return;
  std::vector<std::shared_future<void>> futures;
  size_t size = all_entities.size();
  std::vector<std::vector<Entity>> collected_entity_lists;
  const auto thread_size = Jobs::GetWorkerSize();
  for (int i = 0; i < thread_size; i++) {
    collected_entity_lists.push_back(std::vector<Entity>());
  }
  Jobs::RunParallelFor(
      size,
      [=, &all_entities, &component_data_list1, &component_data_list2, &collected_entity_lists](
          size_t i, const size_t thread_index) {
        if (filter_func(all_entities[i], component_data_list1[i], component_data_list2[i])) {
          collected_entity_lists.at(thread_index).push_back(all_entities[i]);
        }
      },
      thread_size);
  for (int i = 0; i < collected_entity_lists.size(); i++) {
    const auto list_size = collected_entity_lists[i].size();
    if (list_size == 0)
      continue;
    container.resize(container.size() + list_size);
    memcpy(&container.at(container.size() - list_size), collected_entity_lists[i].data(), list_size * sizeof(Entity));
  }

  const size_t remainder = size % thread_size;
  for (int i = 0; i < remainder; i++) {
    if (filter_func(all_entities[size - remainder + i], component_data_list1[size - remainder + i],
                    component_data_list2[size - remainder + i])) {
      container.push_back(all_entities[size - remainder + i]);
    }
  }
}

template <typename T1>
void Scene::GetEntityArray(const EntityQuery& entity_query, const T1& filter, std::vector<Entity>& container,
                           bool check_enable) {
  assert(entity_query.IsValid());
  std::vector<Entity> all_entities;
  std::vector<T1> component_data_list;
  GetEntityArray(entity_query, all_entities, check_enable);
  GetComponentDataArray(entity_query, component_data_list, check_enable);
  std::vector<std::shared_future<void>> futures;
  size_t size = all_entities.size();
  std::vector<std::vector<Entity>> collected_entity_lists;
  const auto thread_size = Jobs::GetWorkerSize();
  for (int i = 0; i < thread_size; i++) {
    collected_entity_lists.push_back(std::vector<Entity>());
  }
  Jobs::RunParallelFor(
      size,
      [&all_entities, &component_data_list, filter, &collected_entity_lists](size_t i, const size_t thread_index) {
        if (filter == component_data_list[i]) {
          collected_entity_lists.at(thread_index).push_back(all_entities[i]);
        }
      },
      thread_size);

  for (int i = 0; i < collected_entity_lists.size(); i++) {
    const auto list_size = collected_entity_lists[i].size();
    if (list_size == 0)
      continue;
    container.resize(container.size() + list_size);
    memcpy(&container.at(container.size() - list_size), collected_entity_lists[i].data(), list_size * sizeof(Entity));
  }

  const size_t remainder = size % thread_size;
  for (int i = 0; i < remainder; i++) {
    if (filter == component_data_list[size - remainder + i]) {
      container.push_back(all_entities[size - remainder + i]);
    }
  }
}

template <typename T>
std::vector<std::pair<T*, size_t>> Scene::UnsafeGetDataComponentArray(const EntityQuery& entity_query) {
  std::vector<std::pair<T*, size_t>> return_value;
  assert(entity_query.IsValid());
  const auto queried_storage_list = QueryDataComponentStorageList(entity_query);
  for (const auto storage : queried_storage_list) {
    auto& i = storage.get();
    auto target_type = Typeof<T>();
    const auto entity_count = i.entity_alive_count;
    auto found = false;
    for (const auto& type : i.data_component_types) {
      if (type.type_index == target_type.type_index) {
        target_type = type;
        found = true;
      }
    }
    if (!found)
      continue;
    const auto capacity = i.chunk_capacity;
    auto& chunk_array = i.chunk_array;
    const auto chunk_size = entity_count / capacity;
    const auto chunk_reminder = entity_count % capacity;
    for (int chunk_index = 0; chunk_index < chunk_size; chunk_index++) {
      T* ptr = static_cast<T*>(chunk_array.chunks[chunk_index].RefData(target_type.type_offset * capacity));
      return_value.emplace_back(ptr, capacity);
    }
    if (chunk_reminder > 0) {
      T* ptr = static_cast<T*>(chunk_array.chunks[chunk_size].RefData(target_type.type_offset * capacity));
      return_value.emplace_back(ptr, chunk_reminder);
    }
  }
  return return_value;
}

template <typename T>
const std::vector<Entity>* Scene::UnsafeGetPrivateComponentOwnersList() {
  return scene_data_storage_.entity_private_component_storage.UnsafeGetOwnersList<T>();
}

template <typename T>
void Scene::GetDataComponentArrayStorage(const DataComponentStorage& storage, std::vector<T>& container,
                                         const bool check_enable) {
  auto target_type = Typeof<T>();
  for (const auto& type : storage.data_component_types) {
    if (type.type_index == target_type.type_index) {
      target_type = type;
      size_t amount = storage.entity_alive_count;
      if (amount == 0)
        return;
      if (check_enable) {
        const auto thread_size = Jobs::GetWorkerSize();
        std::vector<std::vector<T>> temp_storage;
        temp_storage.resize(thread_size);
        const auto capacity = storage.chunk_capacity;
        const auto& chunk_array = storage.chunk_array;
        const auto& entities = chunk_array.entity_array;
        Jobs::RunParallelFor(amount, [&](size_t i, size_t thread_index) {
          const auto chunk_index = i / capacity;
          const auto remainder = i % capacity;
          const T* address1 =
              static_cast<const T*>(chunk_array.chunks[chunk_index].PeekData(type.type_offset * capacity));
          if (const auto entity = entities.at(i);
              !scene_data_storage_.entity_metadata_list.at(entity.index_).entity_enabled)
            return;
          temp_storage[thread_index].push_back(address1[remainder]);
        });
        for (auto& i : temp_storage) {
          container.insert(container.end(), i.begin(), i.end());
        }
      } else {
        container.resize(container.size() + amount);
        const auto capacity = storage.chunk_capacity;
        const auto chunk_amount = amount / capacity;
        const auto remain_amount = amount % capacity;
        for (size_t i = 0; i < chunk_amount; i++) {
          memcpy(&container.at(container.size() - remain_amount - capacity * (chunk_amount - i)),
                 storage.chunk_array.chunks[i].PeekData(capacity * target_type.type_offset),
                 capacity * target_type.type_size);
        }
        if (remain_amount > 0)
          memcpy(&container.at(container.size() - remain_amount),
                 storage.chunk_array.chunks[chunk_amount].PeekData(capacity * target_type.type_offset),
                 remain_amount * target_type.type_size);
      }
    }
  }
}
#pragma region ForEachStorage
template <typename T1>
JobHandle Scene::ForEachStorage(const std::vector<JobHandle>& dependencies, DataComponentStorage& storage,
                                std::function<void(int i, Entity entity, T1&)>&& func, const bool check_enable) {
  auto target_type1 = Typeof<T1>();
  const auto entity_count = storage.entity_alive_count;
  auto found1 = false;
  for (const auto& type : storage.data_component_types) {
    if (type.type_index == target_type1.type_index) {
      target_type1 = type;
      found1 = true;
    }
  }
  if (!found1)
    return JobHandle();
  const auto capacity = storage.chunk_capacity;
  auto& chunk_array = storage.chunk_array;
  const auto& entities = chunk_array.entity_array;
  return Jobs::ScheduleParallelFor(dependencies, entity_count, [=, &chunk_array](const size_t i) {
    const auto chunk_index = i / capacity;
    const auto remainder = i % capacity;
    auto& chunk = chunk_array.chunks[chunk_index];
    T1* address1 = static_cast<T1*>(chunk.RefData(target_type1.type_offset * capacity));
    const auto entity = entities.at(i);
    if (check_enable && !scene_data_storage_.entity_metadata_list.at(entity.index_).entity_enabled)
      return;
    func(static_cast<int>(i), entity, address1[remainder]);
  });
}
template <typename T1, typename T2>
JobHandle Scene::ForEachStorage(const std::vector<JobHandle>& dependencies, DataComponentStorage& storage,
                                std::function<void(int i, Entity entity, T1&, T2&)>&& func, const bool check_enable) {
  auto target_type1 = Typeof<T1>();
  auto target_type2 = Typeof<T2>();
  const auto entity_count = storage.entity_alive_count;
  bool found1 = false;
  bool found2 = false;
  for (const auto& type : storage.data_component_types) {
    if (type.type_index == target_type1.type_index) {
      target_type1 = type;
      found1 = true;
    } else if (type.type_index == target_type2.type_index) {
      target_type2 = type;
      found2 = true;
    }
  }

  if (!found1 || !found2)
    return JobHandle();
  const auto capacity = storage.chunk_capacity;
  auto& chunk_array = storage.chunk_array;
  const auto& entities = chunk_array.entity_array;
  return Jobs::ScheduleParallelFor(dependencies, entity_count, [=, &chunk_array](const size_t i) {
    const auto chunk_index = i / capacity;
    const auto remainder = i % capacity;
    auto& chunk = chunk_array.chunks[chunk_index];
    T1* address1 = static_cast<T1*>(chunk.RefData(target_type1.type_offset * capacity));
    T2* address2 = static_cast<T2*>(chunk.RefData(target_type2.type_offset * capacity));
    const auto entity = entities.at(i);
    if (check_enable && !scene_data_storage_.entity_metadata_list.at(entity.index_).entity_enabled)
      return;
    func(static_cast<int>(i), entity, address1[remainder], address2[remainder]);
  });
}
template <typename T1, typename T2, typename T3>
JobHandle Scene::ForEachStorage(const std::vector<JobHandle>& dependencies, DataComponentStorage& storage,
                                std::function<void(int i, Entity entity, T1&, T2&, T3&)>&& func,
                                const bool check_enable) {
  auto target_type1 = Typeof<T1>();
  auto target_type2 = Typeof<T2>();
  auto target_type3 = Typeof<T3>();
  const auto entity_count = storage.entity_alive_count;
  bool found1 = false;
  bool found2 = false;
  bool found3 = false;
  for (const auto& type : storage.data_component_types) {
    if (type.type_index == target_type1.type_index) {
      target_type1 = type;
      found1 = true;
    } else if (type.type_index == target_type2.type_index) {
      target_type2 = type;
      found2 = true;
    } else if (type.type_index == target_type3.type_index) {
      target_type3 = type;
      found3 = true;
    }
  }
  if (!found1 || !found2 || !found3)
    return JobHandle();
  const auto capacity = storage.chunk_capacity;
  auto& chunk_array = storage.chunk_array;
  const auto& entities = chunk_array.entity_array;
  return Jobs::ScheduleParallelFor(dependencies, entity_count, [=, &chunk_array](const size_t i) {
    const auto chunk_index = i / capacity;
    const auto remainder = i % capacity;
    auto& chunk = chunk_array.chunks[chunk_index];
    T1* address1 = static_cast<T1*>(chunk.RefData(target_type1.type_offset * capacity));
    T2* address2 = static_cast<T2*>(chunk.RefData(target_type2.type_offset * capacity));
    T3* address3 = static_cast<T3*>(chunk.RefData(target_type3.type_offset * capacity));
    const auto entity = entities.at(i);
    if (check_enable && !scene_data_storage_.entity_metadata_list.at(entity.index_).entity_enabled)
      return;
    func(static_cast<int>(i), entity, address1[remainder], address2[remainder], address3[remainder]);
  });
}
template <typename T1, typename T2, typename T3, typename T4>
JobHandle Scene::ForEachStorage(const std::vector<JobHandle>& dependencies, DataComponentStorage& storage,
                                std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&)>&& func,
                                const bool check_enable) {
  auto target_type1 = Typeof<T1>();
  auto target_type2 = Typeof<T2>();
  auto target_type3 = Typeof<T3>();
  auto target_type4 = Typeof<T4>();
  const auto entity_count = storage.entity_alive_count;
  bool found1 = false;
  bool found2 = false;
  bool found3 = false;
  bool found4 = false;
  for (const auto& type : storage.data_component_types) {
    if (type.type_index == target_type1.type_index) {
      target_type1 = type;
      found1 = true;
    } else if (type.type_index == target_type2.type_index) {
      target_type2 = type;
      found2 = true;
    } else if (type.type_index == target_type3.type_index) {
      target_type3 = type;
      found3 = true;
    } else if (type.type_index == target_type4.type_index) {
      target_type4 = type;
      found4 = true;
    }
  }
  if (!found1 || !found2 || !found3 || !found4)
    return JobHandle();
  const auto capacity = storage.chunk_capacity;
  auto& chunk_array = storage.chunk_array;
  const auto& entities = chunk_array.entity_array;
  return Jobs::ScheduleParallelFor(dependencies, entity_count, [=, &chunk_array](const size_t i) {
    const auto chunk_index = i / capacity;
    const auto remainder = i % capacity;
    auto& chunk = chunk_array.chunks[chunk_index];
    T1* address1 = static_cast<T1*>(chunk.RefData(target_type1.type_offset * capacity));
    T2* address2 = static_cast<T2*>(chunk.RefData(target_type2.type_offset * capacity));
    T3* address3 = static_cast<T3*>(chunk.RefData(target_type3.type_offset * capacity));
    T4* address4 = static_cast<T4*>(chunk.RefData(target_type4.type_offset * capacity));
    const auto entity = entities.at(i);
    if (check_enable && !scene_data_storage_.entity_metadata_list.at(entity.index_).entity_enabled)
      return;
    func(static_cast<int>(i), entity, address1[remainder], address2[remainder], address3[remainder],
         address4[remainder]);
  });
}
template <typename T1, typename T2, typename T3, typename T4, typename T5>
JobHandle Scene::ForEachStorage(const std::vector<JobHandle>& dependencies, DataComponentStorage& storage,
                                std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&)>&& func,
                                const bool check_enable) {
  auto target_type1 = Typeof<T1>();
  auto target_type2 = Typeof<T2>();
  auto target_type3 = Typeof<T3>();
  auto target_type4 = Typeof<T4>();
  auto target_type5 = Typeof<T5>();
  const auto entity_count = storage.entity_alive_count;
  bool found1 = false;
  bool found2 = false;
  bool found3 = false;
  bool found4 = false;
  bool found5 = false;
  for (const auto& type : storage.data_component_types) {
    if (type.type_index == target_type1.type_index) {
      target_type1 = type;
      found1 = true;
    } else if (type.type_index == target_type2.type_index) {
      target_type2 = type;
      found2 = true;
    } else if (type.type_index == target_type3.type_index) {
      target_type3 = type;
      found3 = true;
    } else if (type.type_index == target_type4.type_index) {
      target_type4 = type;
      found4 = true;
    } else if (type.type_index == target_type5.type_index) {
      target_type5 = type;
      found5 = true;
    }
  }
  if (!found1 || !found2 || !found3 || !found4 || !found5)
    return JobHandle();
  const auto capacity = storage.chunk_capacity;
  auto& chunk_array = storage.chunk_array;
  const auto& entities = chunk_array.entity_array;
  return Jobs::ScheduleParallelFor(dependencies, entity_count, [=, &chunk_array](const size_t i) {
    const auto chunk_index = i / capacity;
    const auto remainder = i % capacity;
    auto& chunk = chunk_array.chunks[chunk_index];
    T1* address1 = static_cast<T1*>(chunk.RefData(target_type1.type_offset * capacity));
    T2* address2 = static_cast<T2*>(chunk.RefData(target_type2.type_offset * capacity));
    T3* address3 = static_cast<T3*>(chunk.RefData(target_type3.type_offset * capacity));
    T4* address4 = static_cast<T4*>(chunk.RefData(target_type4.type_offset * capacity));
    T5* address5 = static_cast<T5*>(chunk.RefData(target_type5.type_offset * capacity));
    const auto entity = entities.at(i);
    if (check_enable && !scene_data_storage_.entity_metadata_list.at(entity.index_).entity_enabled)
      return;
    func(static_cast<int>(i), entity, address1[remainder], address2[remainder], address3[remainder],
         address4[remainder], address5[remainder]);
  });
}
template <typename T1, typename T2, typename T3, typename T4, typename T5, typename T6>
JobHandle Scene::ForEachStorage(const std::vector<JobHandle>& dependencies, DataComponentStorage& storage,
                                std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&, T6&)>&& func,
                                const bool check_enable) {
  auto target_type1 = Typeof<T1>();
  auto target_type2 = Typeof<T2>();
  auto target_type3 = Typeof<T3>();
  auto target_type4 = Typeof<T4>();
  auto target_type5 = Typeof<T5>();
  auto target_type6 = Typeof<T6>();
  const auto entity_count = storage.entity_alive_count;
  bool found1 = false;
  bool found2 = false;
  bool found3 = false;
  bool found4 = false;
  bool found5 = false;
  bool found6 = false;
  for (const auto& type : storage.data_component_types) {
    if (type.type_index == target_type1.type_index) {
      target_type1 = type;
      found1 = true;
    } else if (type.type_index == target_type2.type_index) {
      target_type2 = type;
      found2 = true;
    } else if (type.type_index == target_type3.type_index) {
      target_type3 = type;
      found3 = true;
    } else if (type.type_index == target_type4.type_index) {
      target_type4 = type;
      found4 = true;
    } else if (type.type_index == target_type5.type_index) {
      target_type5 = type;
      found5 = true;
    } else if (type.type_index == target_type6.type_index) {
      target_type6 = type;
      found6 = true;
    }
  }
  if (!found1 || !found2 || !found3 || !found4 || !found5 || !found6)
    return JobHandle();
  const auto capacity = storage.chunk_capacity;
  auto& chunk_array = storage.chunk_array;
  const auto& entities = chunk_array.entity_array;
  return Jobs::ScheduleParallelFor(dependencies, entity_count, [=, &chunk_array](const size_t i) {
    const auto chunk_index = i / capacity;
    const auto remainder = i % capacity;
    auto& chunk = chunk_array.chunks[chunk_index];
    T1* address1 = static_cast<T1*>(chunk.RefData(target_type1.type_offset * capacity));
    T2* address2 = static_cast<T2*>(chunk.RefData(target_type2.type_offset * capacity));
    T3* address3 = static_cast<T3*>(chunk.RefData(target_type3.type_offset * capacity));
    T4* address4 = static_cast<T4*>(chunk.RefData(target_type4.type_offset * capacity));
    T5* address5 = static_cast<T5*>(chunk.RefData(target_type5.type_offset * capacity));
    T6* address6 = static_cast<T6*>(chunk.RefData(target_type6.type_offset * capacity));
    const auto entity = entities.at(i);
    if (check_enable && !scene_data_storage_.entity_metadata_list.at(entity.index_).entity_enabled)
      return;
    func(static_cast<int>(i), entity, address1[remainder], address2[remainder], address3[remainder],
         address4[remainder], address5[remainder], address6[remainder]);
  });
}
template <typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7>
JobHandle Scene::ForEachStorage(const std::vector<JobHandle>& dependencies, DataComponentStorage& storage,
                                std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&, T6&, T7&)>&& func,
                                const bool check_enable) {
  auto target_type1 = Typeof<T1>();
  auto target_type2 = Typeof<T2>();
  auto target_type3 = Typeof<T3>();
  auto target_type4 = Typeof<T4>();
  auto target_type5 = Typeof<T5>();
  auto target_type6 = Typeof<T6>();
  auto target_type7 = Typeof<T7>();
  const auto entity_count = storage.entity_alive_count;
  bool found1 = false;
  bool found2 = false;
  bool found3 = false;
  bool found4 = false;
  bool found5 = false;
  bool found6 = false;
  bool found7 = false;
  for (const auto& type : storage.data_component_types) {
    if (type.type_index == target_type1.type_index) {
      target_type1 = type;
      found1 = true;
    } else if (type.type_index == target_type2.type_index) {
      target_type2 = type;
      found2 = true;
    } else if (type.type_index == target_type3.type_index) {
      target_type3 = type;
      found3 = true;
    } else if (type.type_index == target_type4.type_index) {
      target_type4 = type;
      found4 = true;
    } else if (type.type_index == target_type5.type_index) {
      target_type5 = type;
      found5 = true;
    } else if (type.type_index == target_type6.type_index) {
      target_type6 = type;
      found6 = true;
    } else if (type.type_index == target_type7.type_index) {
      target_type7 = type;
      found7 = true;
    }
  }
  if (!found1 || !found2 || !found3 || !found4 || !found5 || !found6 || !found7)
    return JobHandle();
  const auto capacity = storage.chunk_capacity;
  auto& chunk_array = storage.chunk_array;
  const auto& entities = chunk_array.entity_array;
  return Jobs::ScheduleParallelFor(dependencies, entity_count, [=, &chunk_array](const size_t i) {
    const auto chunk_index = i / capacity;
    const auto remainder = i % capacity;
    auto& chunk = chunk_array.chunks[chunk_index];
    T1* address1 = static_cast<T1*>(chunk.RefData(target_type1.type_offset * capacity));
    T2* address2 = static_cast<T2*>(chunk.RefData(target_type2.type_offset * capacity));
    T3* address3 = static_cast<T3*>(chunk.RefData(target_type3.type_offset * capacity));
    T4* address4 = static_cast<T4*>(chunk.RefData(target_type4.type_offset * capacity));
    T5* address5 = static_cast<T5*>(chunk.RefData(target_type5.type_offset * capacity));
    T6* address6 = static_cast<T6*>(chunk.RefData(target_type6.type_offset * capacity));
    T7* address7 = static_cast<T7*>(chunk.RefData(target_type7.type_offset * capacity));
    const auto entity = entities.at(i);
    if (check_enable && !scene_data_storage_.entity_metadata_list.at(entity.index_).entity_enabled)
      return;
    func(static_cast<int>(i), entity, address1[remainder], address2[remainder], address3[remainder],
         address4[remainder], address5[remainder], address6[remainder], address7[remainder]);
  });
}
template <typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7, typename T8>
JobHandle Scene::ForEachStorage(
    const std::vector<JobHandle>& dependencies, DataComponentStorage& storage,
    std::function<void(int i, Entity entity, T1&, T2&, T3&, T4&, T5&, T6&, T7&, T8&)>&& func, const bool check_enable) {
  auto target_type1 = Typeof<T1>();
  auto target_type2 = Typeof<T2>();
  auto target_type3 = Typeof<T3>();
  auto target_type4 = Typeof<T4>();
  auto target_type5 = Typeof<T5>();
  auto target_type6 = Typeof<T6>();
  auto target_type7 = Typeof<T7>();
  auto target_type8 = Typeof<T8>();
  const auto entity_count = storage.entity_alive_count;
  bool found1 = false;
  bool found2 = false;
  bool found3 = false;
  bool found4 = false;
  bool found5 = false;
  bool found6 = false;
  bool found7 = false;
  bool found8 = false;
  for (const auto& type : storage.data_component_types) {
    if (type.type_index == target_type1.type_index) {
      target_type1 = type;
      found1 = true;
    } else if (type.type_index == target_type2.type_index) {
      target_type2 = type;
      found2 = true;
    } else if (type.type_index == target_type3.type_index) {
      target_type3 = type;
      found3 = true;
    } else if (type.type_index == target_type4.type_index) {
      target_type4 = type;
      found4 = true;
    } else if (type.type_index == target_type5.type_index) {
      target_type5 = type;
      found5 = true;
    } else if (type.type_index == target_type6.type_index) {
      target_type6 = type;
      found6 = true;
    } else if (type.type_index == target_type7.type_index) {
      target_type7 = type;
      found7 = true;
    } else if (type.type_index == target_type8.type_index) {
      target_type8 = type;
      found8 = true;
    }
  }
  if (!found1 || !found2 || !found3 || !found4 || !found5 || !found6 || !found7 || !found8)
    return JobHandle();
  const auto capacity = storage.chunk_capacity;
  auto& chunk_array = storage.chunk_array;
  const auto& entities = chunk_array.entity_array;
  return Jobs::ScheduleParallelFor(dependencies, entity_count, [=, &chunk_array](const size_t i) {
    const auto chunk_index = i / capacity;
    const auto remainder = i % capacity;
    auto& chunk = chunk_array.chunks[chunk_index];
    T1* address1 = static_cast<T1*>(chunk.RefData(target_type1.type_offset * capacity));
    T2* address2 = static_cast<T2*>(chunk.RefData(target_type2.type_offset * capacity));
    T3* address3 = static_cast<T3*>(chunk.RefData(target_type3.type_offset * capacity));
    T4* address4 = static_cast<T4*>(chunk.RefData(target_type4.type_offset * capacity));
    T5* address5 = static_cast<T5*>(chunk.RefData(target_type5.type_offset * capacity));
    T6* address6 = static_cast<T6*>(chunk.RefData(target_type6.type_offset * capacity));
    T7* address7 = static_cast<T7*>(chunk.RefData(target_type7.type_offset * capacity));
    T8* address8 = static_cast<T8*>(chunk.RefData(target_type8.type_offset * capacity));
    const auto entity = entities.at(i);
    if (check_enable && !scene_data_storage_.entity_metadata_list.at(entity.index_).entity_enabled)
      return;
    func(static_cast<int>(i), entity, address1[remainder], address2[remainder], address3[remainder],
         address4[remainder], address5[remainder], address6[remainder], address7[remainder], address8[remainder]);
  });
}
#pragma endregion
#pragma endregion
}  // namespace evo_engine
