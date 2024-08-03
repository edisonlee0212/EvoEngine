#include "Scene.hpp"
#include "Application.hpp"

#include "ClassRegistry.hpp"
#include "EditorLayer.hpp"
#include "Entities.hpp"
#include "EntityMetadata.hpp"
#include "Jobs.hpp"
#include "Lights.hpp"
#include "MeshRenderer.hpp"
#include "Resources.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "UnknownPrivateComponent.hpp"
using namespace evo_engine;

void Scene::Purge() {
  pressed_keys_.clear();
  main_camera.Clear();

  scene_data_storage_.entity_private_component_storage = PrivateComponentStorage();
  scene_data_storage_.entity_private_component_storage.owner_scene = std::dynamic_pointer_cast<Scene>(GetSelf());
  scene_data_storage_.entities.clear();
  scene_data_storage_.entity_metadata_list.clear();
  scene_data_storage_.data_component_storage_list.clear();

  scene_data_storage_.data_component_storage_list.emplace_back();
  scene_data_storage_.entities.emplace_back();
  scene_data_storage_.entity_metadata_list.emplace_back();
}

Bound Scene::GetBound() const {
  return world_bound_;
}

void Scene::SetBound(const Bound& value) {
  world_bound_ = value;
}

Scene::~Scene() {
  Purge();
  for (const auto& i : systems_) {
    i.second->OnDestroy();
  }
}

void Scene::Start() const {
  const auto entities = scene_data_storage_.entities;
  for (const auto& entity : entities) {
    if (entity.version_ == 0)
      continue;
    const auto entity_info = scene_data_storage_.entity_metadata_list[entity.index_];
    if (!entity_info.entity_enabled)
      continue;
    for (const auto& private_component_element : entity_info.private_component_elements) {
      if (!private_component_element.private_component_data->enabled_)
        continue;
      if (!private_component_element.private_component_data->started_) {
        private_component_element.private_component_data->Start();
        if (entity.version_ != entity_info.entity_version)
          break;
        private_component_element.private_component_data->started_ = true;
      }
      if (entity.version_ != entity_info.entity_version)
        break;
    }
  }
  for (auto& i : systems_) {
    if (i.second->Enabled()) {
      if (!i.second->started_) {
        i.second->Start();
        i.second->started_ = true;
      }
    }
  }
}

void Scene::Update() const {
  const auto entities = scene_data_storage_.entities;
  for (const auto& entity : entities) {
    if (entity.version_ == 0)
      continue;
    const auto entity_info = scene_data_storage_.entity_metadata_list[entity.index_];
    if (!entity_info.entity_enabled)
      continue;
    for (const auto& private_component_element : entity_info.private_component_elements) {
      if (!private_component_element.private_component_data->enabled_ ||
          !private_component_element.private_component_data->started_)
        continue;
      private_component_element.private_component_data->Update();
      if (entity.version_ != entity_info.entity_version)
        break;
    }
  }

  for (auto& i : systems_) {
    if (i.second->Enabled() && i.second->started_) {
      i.second->Update();
    }
  }
}

void Scene::LateUpdate() const {
  const auto entities = scene_data_storage_.entities;
  for (const auto& entity : entities) {
    if (entity.version_ == 0)
      continue;
    const auto entity_info = scene_data_storage_.entity_metadata_list[entity.index_];
    if (!entity_info.entity_enabled)
      continue;
    for (const auto& private_component_element : entity_info.private_component_elements) {
      if (!private_component_element.private_component_data->enabled_ ||
          !private_component_element.private_component_data->started_)
        continue;
      private_component_element.private_component_data->LateUpdate();
      if (entity.version_ != entity_info.entity_version)
        break;
    }
  }

  for (auto& i : systems_) {
    if (i.second->Enabled() && i.second->started_) {
      i.second->LateUpdate();
    }
  }
}
void Scene::FixedUpdate() const {
  const auto entities = scene_data_storage_.entities;
  for (const auto& entity : entities) {
    if (entity.version_ == 0)
      continue;
    const auto entity_info = scene_data_storage_.entity_metadata_list[entity.index_];
    if (!entity_info.entity_enabled)
      continue;
    for (const auto& private_component_element : entity_info.private_component_elements) {
      if (!private_component_element.private_component_data->enabled_ ||
          !private_component_element.private_component_data->started_)
        continue;
      private_component_element.private_component_data->FixedUpdate();
      if (entity.version_ != entity_info.entity_version)
        break;
    }
  }

  for (const auto& i : systems_) {
    if (i.second->Enabled() && i.second->started_) {
      i.second->FixedUpdate();
    }
  }
}
const char* environment_types[]{"Environmental Map", "Color"};
bool Scene::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool modified = false;
  if (this == Application::GetActiveScene().get())
    if (EditorLayer::DragAndDropButton<Camera>(main_camera, "Main Camera", true))
      modified = true;
  if (ImGui::TreeNodeEx("Environment Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    static int type = static_cast<int>(environment.environment_type);
    if (ImGui::Combo("Environment type", &type, environment_types, IM_ARRAYSIZE(environment_types))) {
      environment.environment_type = static_cast<EnvironmentType>(type);
      modified = true;
    }
    switch (environment.environment_type) {
      case EnvironmentType::EnvironmentalMap: {
        if (EditorLayer::DragAndDropButton<EnvironmentalMap>(environment.environmental_map, "Environmental Map"))
          modified = true;
      } break;
      case EnvironmentType::Color: {
        if (ImGui::ColorEdit3("Background Color", &environment.background_color.x))
          modified = true;
      } break;
    }
    if (ImGui::DragFloat("Environmental light intensity", &environment.ambient_light_intensity, 0.01f, 0.0f, 10.0f))
      modified = true;
    if (ImGui::DragFloat("Environmental light gamma", &environment.environment_gamma, 0.01f, 0.0f, 10.0f)) {
      modified = true;
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Systems")) {
    if (ImGui::BeginPopupContextWindow("SystemInspectorPopup")) {
      ImGui::Text("Add system: ");
      ImGui::Separator();
      static float rank = 0.0f;
      ImGui::DragFloat("Rank", &rank, 1.0f, 0.0f, 999.0f);
      for (const auto& i : Serialization::GetInstance().system_ids_) {
        const auto id = i.second;
        const auto name = i.first;
        if (!HasSystem(id) && ImGui::Button(name.c_str())) {
          CreateSystem(id, rank);
        }
      }
      ImGui::Separator();
      ImGui::EndPopup();
    }
    for (auto& i : Application::GetActiveScene()->systems_) {
      if (ImGui::CollapsingHeader(i.second->GetTypeName().c_str())) {
        bool enabled = i.second->Enabled();
        if (ImGui::Checkbox("Enabled", &enabled)) {
          if (i.second->Enabled() != enabled) {
            if (enabled) {
              i.second->Enable();
              modified = true;
            } else {
              i.second->Disable();
              modified = true;
            }
          }
        }
        i.second->OnInspect(editor_layer);
      }
    }
    ImGui::TreePop();
  }
  return modified;
}

std::shared_ptr<ISystem> Scene::GetOrCreateSystem(const std::string& system_name, float order) {
  size_t type_index;
  auto ptr = Serialization::ProduceSerializable(system_name, type_index);
  auto system = std::dynamic_pointer_cast<ISystem>(ptr);
  system->scene_ = std::dynamic_pointer_cast<Scene>(GetSelf());
  system->handle_ = Handle();
  system->rank_ = order;
  systems_.insert({order, system});
  indexed_systems_[type_index] = system;
  mapped_systems_[system->handle_] = system;
  system->started_ = false;
  system->OnCreate();
  SetUnsaved();
  return std::dynamic_pointer_cast<ISystem>(ptr);
}

void Scene::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "environment" << YAML::Value << YAML::BeginMap;
  environment.Serialize(out);
  out << YAML::EndMap;
  main_camera.Save("main_camera", out);
  std::unordered_map<Handle, std::shared_ptr<IAsset>> asset_map;
  std::vector<AssetRef> list;
  list.push_back(environment.environmental_map);
  auto& scene_data_storage = scene_data_storage_;
#pragma region EntityInfo
  out << YAML::Key << "entity_metadata_list" << YAML::Value << YAML::BeginSeq;
  for (size_t i = 1; i < scene_data_storage.entity_metadata_list.size(); i++) {
    auto& entity_metadata = scene_data_storage.entity_metadata_list[i];
    if (entity_metadata.entity_handle == 0)
      continue;
    for (const auto& element : entity_metadata.private_component_elements) {
      element.private_component_data->CollectAssetRef(list);
    }
    entity_metadata.Serialize(out, std::dynamic_pointer_cast<Scene>(GetSelf()));
  }
  out << YAML::EndSeq;
#pragma endregion

#pragma region Systems
  out << YAML::Key << "systems_" << YAML::Value << YAML::BeginSeq;
  for (const auto& i : systems_) {
    SerializeSystem(i.second, out);
    i.second->CollectAssetRef(list);
  }
  out << YAML::EndSeq;
#pragma endregion

#pragma region Assets
  for (auto& i : list) {
    const auto asset = i.Get<IAsset>();

    if (asset && !Resources::IsResource(asset->GetHandle())) {
      if (asset->IsTemporary()) {
        asset_map[asset->GetHandle()] = asset;
      } else if (!asset->Saved()) {
        asset->Save();
      }
    }
  }
  bool list_check = true;
  while (list_check) {
    const size_t current_size = asset_map.size();
    list.clear();
    for (const auto& i : asset_map) {
      i.second->CollectAssetRef(list);
    }
    for (auto& i : list) {
      if (const auto asset = i.Get<IAsset>(); asset && !Resources::IsResource(asset->GetHandle())) {
        if (asset->IsTemporary()) {
          asset_map[asset->GetHandle()] = asset;
        } else if (!asset->Saved()) {
          asset->Save();
        }
      }
    }
    if (asset_map.size() == current_size)
      list_check = false;
  }
  if (!asset_map.empty()) {
    out << YAML::Key << "LocalAssets" << YAML::Value << YAML::BeginSeq;
    for (auto& i : asset_map) {
      out << YAML::BeginMap;
      out << YAML::Key << "type_name" << YAML::Value << i.second->GetTypeName();
      out << YAML::Key << "handle" << YAML::Value << i.second->GetHandle();
      i.second->Serialize(out);
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
#pragma endregion

#pragma region DataComponentStorage
  out << YAML::Key << "data_component_storage_list" << YAML::Value << YAML::BeginSeq;
  for (size_t i = 1; i < scene_data_storage.data_component_storage_list.size(); i++) {
    SerializeDataComponentStorage(scene_data_storage.data_component_storage_list[i], out);
  }
  out << YAML::EndSeq;
#pragma endregion
  out << YAML::EndMap;
}
void Scene::Deserialize(const YAML::Node& in) {
  Purge();
  auto scene = std::dynamic_pointer_cast<Scene>(GetSelf());
  scene_data_storage_.entities.clear();
  scene_data_storage_.entity_metadata_list.clear();
  scene_data_storage_.data_component_storage_list.clear();
  scene_data_storage_.entities.emplace_back();
  scene_data_storage_.entity_metadata_list.emplace_back();
  scene_data_storage_.data_component_storage_list.emplace_back();

#pragma region EntityMetadata
  auto in_entity_metadata_list = in["entity_metadata_list"];
  int current_index = 1;
  for (const auto& in_entity_metadata : in_entity_metadata_list) {
    scene_data_storage_.entity_metadata_list.emplace_back();
    auto& new_info = scene_data_storage_.entity_metadata_list.back();
    new_info.Deserialize(in_entity_metadata, scene);
    Entity entity;
    entity.version_ = 1;
    entity.index_ = current_index;
    scene_data_storage_.entity_map[new_info.entity_handle] = entity;
    scene_data_storage_.entities.push_back(entity);
    current_index++;
  }
  current_index = 1;
  for (const auto& in_entity_metadata : in_entity_metadata_list) {
    auto& metadata = scene_data_storage_.entity_metadata_list[current_index];
    if (in_entity_metadata["p"]) {
      metadata.parent = scene_data_storage_.entity_map[Handle(in_entity_metadata["p"].as<uint64_t>())];
      auto& parent_metadata = scene_data_storage_.entity_metadata_list[metadata.parent.index_];
      Entity entity;
      entity.version_ = 1;
      entity.index_ = current_index;
      parent_metadata.children.push_back(entity);
    }
    if (in_entity_metadata["r"])
      metadata.root = scene_data_storage_.entity_map[Handle(in_entity_metadata["r"].as<uint64_t>())];
    current_index++;
  }
#pragma endregion

#pragma region DataComponentStorage
  const auto& in_data_component_storage_list = in["data_component_storage_list"];
  int storage_index = 1;
  for (const auto& in_data_component_storage : in_data_component_storage_list) {
    scene_data_storage_.data_component_storage_list.emplace_back();
    auto& data_component_storage = scene_data_storage_.data_component_storage_list.back();
    DeserializeDataComponentStorage(storage_index, data_component_storage, in_data_component_storage);
    storage_index++;
  }
  auto self = std::dynamic_pointer_cast<Scene>(GetSelf());
#pragma endregion
  main_camera.Load("main_camera", in, self);
#pragma region Assets
  if (const auto in_local_assets = in["LocalAssets"]) {
    std::vector<std::pair<int, std::shared_ptr<IAsset>>> local_assets;
    int index = 0;
    for (const auto& i : in_local_assets) {
      // First, find the asset in asset registry
      if (const auto type_name = i["type_name"].as<std::string>(); Serialization::HasSerializableType(type_name)) {
        auto asset = ProjectManager::CreateTemporaryAsset(type_name, i["handle"].as<uint64_t>());
        local_assets.emplace_back(index, asset);
      }
      index++;
    }

    for (const auto& i : local_assets) {
      i.second->Deserialize(in_local_assets[i.first]);
    }
#ifdef _DEBUG
    EVOENGINE_LOG(std::string("Scene Deserialization: Loaded " + std::to_string(local_assets.size()) + " assets."))
#endif
  }

#pragma endregion
  if (in["environment"])
    environment.Deserialize(in["environment"]);
  int entity_index = 1;
  for (const auto& in_entity_info : in_entity_metadata_list) {
    auto& entity_metadata = scene_data_storage_.entity_metadata_list.at(entity_index);
    auto entity = scene_data_storage_.entities[entity_index];
    if (auto in_private_components = in_entity_info["pc"]) {
      for (const auto& in_private_component : in_private_components) {
        const auto name = in_private_component["tn"].as<std::string>();
        size_t hash_code;
        if (Serialization::HasSerializableType(name)) {
          auto ptr = std::static_pointer_cast<IPrivateComponent>(Serialization::ProduceSerializable(name, hash_code));
          ptr->enabled_ = in_private_component["e"].as<bool>();
          ptr->started_ = false;
          scene_data_storage_.entity_private_component_storage.SetPrivateComponent(entity, hash_code);
          entity_metadata.private_component_elements.emplace_back(hash_code, ptr, entity, self);
        } else {
          auto ptr = std::static_pointer_cast<IPrivateComponent>(
              Serialization::ProduceSerializable("UnknownPrivateComponent", hash_code));
          ptr->enabled_ = false;
          ptr->started_ = false;
          std::dynamic_pointer_cast<UnknownPrivateComponent>(ptr)->original_type_name_ = name;
          scene_data_storage_.entity_private_component_storage.SetPrivateComponent(entity, hash_code);
          entity_metadata.private_component_elements.emplace_back(hash_code, ptr, entity, self);
        }
      }
    }
    entity_index++;
  }

#pragma region Systems
  if (auto in_systems = in["systems_"]) {
    std::vector<std::pair<int, std::shared_ptr<ISystem>>> systems;
    int index = 0;
    for (const auto& in_system : in_systems) {
      if (const auto type_name = in_system["type_name"].as<std::string>();
          Serialization::HasSerializableType(type_name)) {
        size_t hash_code;
        if (const auto ptr =
                std::static_pointer_cast<ISystem>(Serialization::ProduceSerializable(type_name, hash_code))) {
          ptr->handle_ = Handle(in_system["handle_"].as<uint64_t>());
          ptr->enabled_ = in_system["enabled_"].as<bool>();
          ptr->rank_ = in_system["rank_"].as<float>();
          ptr->started_ = false;
          systems_.insert({ptr->rank_, ptr});
          indexed_systems_.insert({hash_code, ptr});
          mapped_systems_[ptr->handle_] = ptr;
          systems.emplace_back(index, ptr);
          ptr->scene_ = self;
          ptr->OnCreate();
        }
      }
      index++;
    }
#pragma endregion

    entity_index = 1;
    for (const auto& in_entity_metadata : in_entity_metadata_list) {
      auto& entity_info = scene_data_storage_.entity_metadata_list.at(entity_index);
      if (auto in_private_components = in_entity_metadata["pc"]) {
        int component_index = 0;
        for (const auto& in_private_component : in_private_components) {
          auto name = in_private_component["tn"].as<std::string>();
          auto ptr = entity_info.private_component_elements[component_index].private_component_data;
          ptr->Deserialize(in_private_component);
          ptr->enabled_ = in_private_component["e"].as<bool>();
          component_index++;
        }
      }
      entity_index++;
    }

    for (const auto& i : systems) {
      i.second->Deserialize(in_systems[i.first]);
    }
  }
}
void Scene::SerializeDataComponentStorage(const DataComponentStorage& storage, YAML::Emitter& out) const {
  out << YAML::BeginMap;
  {
    out << YAML::Key << "entity_size" << YAML::Value << storage.entity_size;
    out << YAML::Key << "chunk_capacity" << YAML::Value << storage.chunk_capacity;
    out << YAML::Key << "entity_alive_count" << YAML::Value << storage.entity_alive_count;
    out << YAML::Key << "data_component_types" << YAML::Value << YAML::BeginSeq;
    for (const auto& i : storage.data_component_types) {
      out << YAML::BeginMap;
      out << YAML::Key << "type_name" << YAML::Value << i.type_name;
      out << YAML::Key << "type_size" << YAML::Value << i.type_size;
      out << YAML::Key << "type_offset" << YAML::Value << i.type_offset;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    out << YAML::Key << "chunk_array" << YAML::Value << YAML::BeginSeq;
    for (size_t i = 0; i < storage.entity_alive_count; i++) {
      const auto entity = storage.chunk_array.entity_array[i];
      if (entity.version_ == 0)
        continue;

      out << YAML::BeginMap;
      auto& entity_info = scene_data_storage_.entity_metadata_list.at(entity.index_);
      out << YAML::Key << "h" << YAML::Value << entity_info.entity_handle;

      auto& data_component_storage =
          scene_data_storage_.data_component_storage_list[entity_info.data_component_storage_index];
      const auto chunk_index = entity_info.chunk_array_index / data_component_storage.chunk_capacity;
      const auto chunk_pointer = entity_info.chunk_array_index % data_component_storage.chunk_capacity;
      auto& chunk = data_component_storage.chunk_array.chunks[chunk_index];

      out << YAML::Key << "dc" << YAML::Value << YAML::BeginSeq;
      for (const auto& type : data_component_storage.data_component_types) {
        out << YAML::BeginMap;
        out << YAML::Key << "d" << YAML::Value
            << YAML::Binary(
                   static_cast<const unsigned char*>(chunk.PeekData(
                       type.type_offset * data_component_storage.chunk_capacity + chunk_pointer * type.type_size)),
                   type.type_size);
        out << YAML::EndMap;
      }
      out << YAML::EndSeq;

      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
  out << YAML::EndMap;
}

void Scene::DeserializeDataComponentStorage(const size_t storage_index, DataComponentStorage& data_component_storage,
                                            const YAML::Node& in) {
  if (in["entity_size"])
    data_component_storage.entity_size = in["entity_size"].as<size_t>();
  if (in["chunk_capacity"])
    data_component_storage.chunk_capacity = in["chunk_capacity"].as<size_t>();
  if (in["entity_alive_count"])
    data_component_storage.entity_alive_count = data_component_storage.entity_count =
        in["entity_alive_count"].as<size_t>();
  data_component_storage.chunk_array.entity_array.resize(data_component_storage.entity_alive_count);
  const size_t chunk_size = data_component_storage.entity_count / data_component_storage.chunk_capacity + 1;
  while (data_component_storage.chunk_array.chunks.size() <= chunk_size) {
    // Allocate new chunk;
    data_component_storage.chunk_array.chunks.emplace_back();
  }
  auto in_data_component_types = in["data_component_types"];
  for (const auto& in_data_component_type : in_data_component_types) {
    DataComponentType data_component_type;
    if (in_data_component_type["type_name"])
      data_component_type.type_name = in_data_component_type["type_name"].as<std::string>();
    if (in_data_component_type["type_size"])
      data_component_type.type_size = in_data_component_type["type_size"].as<size_t>();
    if (in_data_component_type["type_offset"])
      data_component_type.type_offset = in_data_component_type["type_offset"].as<size_t>();
    data_component_type.type_index = Serialization::GetDataComponentTypeId(data_component_type.type_name);
    data_component_storage.data_component_types.push_back(data_component_type);
  }
  auto in_data_chunk_array = in["chunk_array"];
  int chunk_array_index = 0;
  for (const auto& entity_data_component : in_data_chunk_array) {
    Handle handle = entity_data_component["h"].as<uint64_t>();
    const Entity entity = scene_data_storage_.entity_map[handle];
    data_component_storage.chunk_array.entity_array[chunk_array_index] = entity;
    auto& metadata = scene_data_storage_.entity_metadata_list[entity.index_];
    metadata.data_component_storage_index = storage_index;
    metadata.chunk_array_index = chunk_array_index;
    const auto chunk_index = metadata.chunk_array_index / data_component_storage.chunk_capacity;
    const auto chunk_pointer = metadata.chunk_array_index % data_component_storage.chunk_capacity;
    auto& chunk = data_component_storage.chunk_array.chunks[chunk_index];

    int type_index = 0;
    for (const auto& in_data_component : entity_data_component["dc"]) {
      auto& type = data_component_storage.data_component_types[type_index];
      auto data = in_data_component["d"].as<YAML::Binary>();
      std::memcpy(
          chunk.RefData(type.type_offset * data_component_storage.chunk_capacity + chunk_pointer * type.type_size),
          data.data(), data.size());
      type_index++;
    }
    chunk_array_index++;
  }
}

void Scene::SerializeSystem(const std::shared_ptr<ISystem>& system, YAML::Emitter& out) {
  out << YAML::BeginMap;
  {
    out << YAML::Key << "type_name" << YAML::Value << system->GetTypeName();
    out << YAML::Key << "enabled_" << YAML::Value << system->enabled_;
    out << YAML::Key << "rank_" << YAML::Value << system->rank_;
    out << YAML::Key << "handle_" << YAML::Value << system->handle_;
    system->Serialize(out);
  }
  out << YAML::EndMap;
}

void Scene::OnCreate() {
  scene_data_storage_.entities.emplace_back();
  scene_data_storage_.entity_metadata_list.emplace_back();
  scene_data_storage_.data_component_storage_list.emplace_back();
  scene_data_storage_.entity_private_component_storage.owner_scene = std::dynamic_pointer_cast<Scene>(GetSelf());

#pragma region Main Camera
  const auto main_camera_entity = CreateEntity("Main Camera");
  Transform ltw;
  ltw.SetPosition(glm::vec3(0.0f, 5.0f, 10.0f));
  ltw.SetScale(glm::vec3(1, 1, 1));
  ltw.SetEulerRotation(glm::radians(glm::vec3(0, 0, 0)));
  SetDataComponent(main_camera_entity, ltw);
  const auto main_camera_component = GetOrSetPrivateComponent<Camera>(main_camera_entity).lock();
  main_camera = main_camera_component;
  main_camera_component->skybox = Resources::GetResource<Cubemap>("DEFAULT_SKYBOX");
#pragma endregion

#pragma region Directional Light
  const auto directional_light_entity = CreateEntity("Directional Light");
  ltw.SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
  ltw.SetEulerRotation(glm::radians(glm::vec3(90, 0, 0)));
  SetDataComponent(directional_light_entity, ltw);
  auto direction_light = GetOrSetPrivateComponent<DirectionalLight>(directional_light_entity).lock();
#pragma endregion
  /*
#pragma region Ground
  const auto ground_entity = CreateEntity("Ground");
  ltw.SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
  ltw.SetScale(glm::vec3(10, 1, 10));
  ltw.SetEulerRotation(glm::radians(glm::vec3(0, 0, 0)));
  SetDataComponent(ground_entity, ltw);
  const auto ground_mesh_renderer_component = GetOrSetPrivateComponent<MeshRenderer>(ground_entity).lock();
  ground_mesh_renderer_component->material = ProjectManager::CreateTemporaryAsset<Material>();
  ground_mesh_renderer_component->mesh = Resources::GetResource<Mesh>("PRIMITIVE_QUAD");
#pragma endregion
  */
}

bool Scene::LoadInternal(const std::filesystem::path& path) {
  const auto previous_scene = Application::GetActiveScene();
  Application::Attach(std::shared_ptr<Scene>(this, [](Scene*) {
  }));
  std::ifstream stream(path.string());
  std::stringstream string_stream;
  string_stream << stream.rdbuf();
  YAML::Node in = YAML::Load(string_stream.str());
  Deserialize(in);
  Application::Attach(previous_scene);
  return true;
}
void Scene::Clone(const std::shared_ptr<Scene>& source, const std::shared_ptr<Scene>& new_scene) {
  new_scene->environment = source->environment;
  new_scene->saved_ = source->saved_;
  new_scene->world_bound_ = source->world_bound_;
  std::unordered_map<Handle, Handle> entity_map;

  new_scene->scene_data_storage_.Clone(entity_map, source->scene_data_storage_, new_scene);
  for (const auto& i : source->systems_) {
    auto system_name = i.second->GetTypeName();
    size_t hash_code;
    auto system = std::dynamic_pointer_cast<ISystem>(
        Serialization::ProduceSerializable(system_name, hash_code, i.second->GetHandle()));
    new_scene->systems_.insert({i.first, system});
    new_scene->indexed_systems_[hash_code] = system;
    new_scene->mapped_systems_[i.second->GetHandle()] = system;
    system->scene_ = new_scene;
    system->OnCreate();
    Serialization::CloneSystem(system, i.second);
    system->scene_ = new_scene;
  }
  new_scene->main_camera.entity_handle_ = source->main_camera.entity_handle_;
  new_scene->main_camera.private_component_type_name_ = source->main_camera.private_component_type_name_;
  new_scene->main_camera.Relink(entity_map, new_scene);
}

std::shared_ptr<LightProbe> Environment::GetLightProbe(const glm::vec3& position) {
  if (const auto em = environmental_map.Get<EnvironmentalMap>()) {
    if (auto light_probe = em->light_probe.Get<LightProbe>())
      return light_probe;
  }
  return nullptr;
}

std::shared_ptr<ReflectionProbe> Environment::GetReflectionProbe(const glm::vec3& position) {
  if (const auto em = environmental_map.Get<EnvironmentalMap>()) {
    if (auto reflection_probe = em->light_probe.Get<ReflectionProbe>())
      return reflection_probe;
  }
  return nullptr;
}

void Environment::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "background_color" << YAML::Value << background_color;
  out << YAML::Key << "environment_gamma" << YAML::Value << environment_gamma;
  out << YAML::Key << "ambient_light_intensity" << YAML::Value << ambient_light_intensity;
  out << YAML::Key << "environment_type" << YAML::Value << static_cast<unsigned>(environment_type);
  environmental_map.Save("environmental_map", out);
}
void Environment::Deserialize(const YAML::Node& in) {
  if (in["background_color"])
    background_color = in["background_color"].as<glm::vec3>();
  if (in["environment_gamma"])
    environment_gamma = in["environment_gamma"].as<float>();
  if (in["ambient_light_intensity"])
    ambient_light_intensity = in["ambient_light_intensity"].as<float>();
  if (in["environment_type"])
    environment_type = static_cast<EnvironmentType>(in["environment_type"].as<unsigned>());
  environmental_map.Load("environmental_map", in);
}
void SceneDataStorage::Clone(std::unordered_map<Handle, Handle>& entity_links, const SceneDataStorage& source,
                             const std::shared_ptr<Scene>& new_scene) {
  entities = source.entities;
  entity_metadata_list.resize(source.entity_metadata_list.size());

  for (const auto& i : source.entity_metadata_list) {
    entity_links.insert({i.entity_handle, i.entity_handle});
  }
  data_component_storage_list.resize(source.data_component_storage_list.size());
  for (size_t i = 0; i < data_component_storage_list.size(); i++)
    data_component_storage_list[i] = source.data_component_storage_list[i];
  for (size_t i = 0; i < entity_metadata_list.size(); i++)
    entity_metadata_list[i].Clone(entity_links, source.entity_metadata_list[i], new_scene);

  entity_map = source.entity_map;
  entity_private_component_storage = source.entity_private_component_storage;
  entity_private_component_storage.owner_scene = new_scene;
}

Input::KeyActionType Scene::GetKey(int key) {
  const auto search = pressed_keys_.find(key);
  if (search != pressed_keys_.end())
    return search->second;
  return Input::KeyActionType::Release;
}

#pragma region Entity Management
void Scene::UnsafeForEachDataComponent(const Entity& entity,
                                       const std::function<void(const DataComponentType& type, void* data)>& func) {
  assert(IsEntityValid(entity));
  const EntityMetadata& entity_info = scene_data_storage_.entity_metadata_list.at(entity.index_);
  auto& data_component_storage =
      scene_data_storage_.data_component_storage_list[entity_info.data_component_storage_index];
  const size_t chunk_index = entity_info.chunk_array_index / data_component_storage.chunk_capacity;
  const size_t chunk_pointer = entity_info.chunk_array_index % data_component_storage.chunk_capacity;
  auto& chunk = data_component_storage.chunk_array.chunks[chunk_index];
  for (const auto& i : data_component_storage.data_component_types) {
    func(i, chunk.RefData(i.type_offset * data_component_storage.chunk_capacity + chunk_pointer * i.type_size));
  }
}

void Scene::ForEachPrivateComponent(const Entity& entity,
                                    const std::function<void(PrivateComponentElement& data)>& func) const {
  assert(IsEntityValid(entity));
  auto elements = scene_data_storage_.entity_metadata_list.at(entity.index_).private_component_elements;
  for (auto& component : elements) {
    func(component);
  }
}

void Scene::UnsafeForEachEntityStorage(
    const std::function<void(size_t i, const std::string& name, const DataComponentStorage& storage)>& func) {
  const auto& archetype_infos = Entities::GetInstance().entity_archetype_infos_;
  for (size_t i = 0; i < archetype_infos.size(); i++) {
    auto dcs = GetDataComponentStorage(i);
    if (!dcs.has_value())
      continue;
    func(i, archetype_infos[i].archetype_name, dcs->first.get());
  }
}

void Scene::DeleteEntityInternal(const unsigned entity_index) {
  EntityMetadata& entity_info = scene_data_storage_.entity_metadata_list.at(entity_index);
  auto& data_component_storage =
      scene_data_storage_.data_component_storage_list[entity_info.data_component_storage_index];
  Entity actual_entity = scene_data_storage_.entities.at(entity_index);

  scene_data_storage_.entity_private_component_storage.DeleteEntity(actual_entity);
  entity_info.entity_version = actual_entity.version_ + 1;
  entity_info.entity_enabled = true;
  entity_info.entity_static = false;
  entity_info.ancestor_selected = false;
  scene_data_storage_.entity_map.erase(entity_info.entity_handle);
  entity_info.entity_handle = Handle(0);

  entity_info.private_component_elements.clear();
  // Set to version 0, marks it as deleted.
  actual_entity.version_ = 0;
  data_component_storage.chunk_array.entity_array[entity_info.chunk_array_index] = actual_entity;
  const auto original_index = entity_info.chunk_array_index;
  if (entity_info.chunk_array_index != data_component_storage.entity_alive_count - 1) {
    const auto swapped_index = SwapEntity(data_component_storage, entity_info.chunk_array_index,
                                          data_component_storage.entity_alive_count - 1);
    entity_info.chunk_array_index = data_component_storage.entity_alive_count - 1;
    scene_data_storage_.entity_metadata_list.at(swapped_index).chunk_array_index = original_index;
  }
  data_component_storage.entity_alive_count--;

  scene_data_storage_.entities.at(entity_index) = actual_entity;
}

std::optional<std::pair<std::reference_wrapper<DataComponentStorage>, unsigned>> Scene::GetDataComponentStorage(
    const size_t entity_archetype_index) {
  auto& archetype_info = Entities::GetInstance().entity_archetype_infos_.at(entity_archetype_index);
  unsigned target_index = 0;
  for (auto& i : scene_data_storage_.data_component_storage_list) {
    if (i.data_component_types.size() != archetype_info.data_component_types.size()) {
      target_index++;
      continue;
    }
    bool check = true;
    for (size_t j = 0; j < i.data_component_types.size(); j++) {
      if (i.data_component_types[j].type_name != archetype_info.data_component_types[j].type_name) {
        check = false;
        break;
      }
    }
    if (check) {
      return {{std::ref(i), target_index}};
    }
    target_index++;
  }
  // If we didn't find the target storage, then we need to create a new one.
  scene_data_storage_.data_component_storage_list.emplace_back(archetype_info);
  return {{std::ref(scene_data_storage_.data_component_storage_list.back()),
           scene_data_storage_.data_component_storage_list.size() - 1}};
}

std::optional<std::pair<std::reference_wrapper<DataComponentStorage>, unsigned>> Scene::GetDataComponentStorage(
    const EntityArchetype& entity_archetype) {
  return GetDataComponentStorage(entity_archetype.index_);
}

std::vector<std::reference_wrapper<DataComponentStorage>> Scene::QueryDataComponentStorageList(
    const EntityQuery& entity_query) {
  return QueryDataComponentStorageList(entity_query.index_);
}

void Scene::GetEntityStorage(const DataComponentStorage& storage, std::vector<Entity>& container,
                             const bool check_enable) const {
  const size_t amount = storage.entity_alive_count;
  if (amount == 0)
    return;
  if (check_enable) {
    const auto worker_size = Jobs::GetWorkerSize();
    std::vector<std::vector<Entity>> temp_storage;
    temp_storage.resize(worker_size);
    const auto& chunk_array = storage.chunk_array;
    const auto& entities = &chunk_array.entity_array;
    Jobs::RunParallelFor(
        amount,
        [this, &entities, &temp_storage](const size_t i, const size_t worker_index) {
          const auto entity = entities->at(i);
          if (!scene_data_storage_.entity_metadata_list.at(entity.index_).entity_enabled)
            return;
          temp_storage[worker_index].push_back(entity);
        },
        worker_size);
    for (auto& i : temp_storage) {
      container.insert(container.end(), i.begin(), i.end());
    }
  } else {
    container.resize(container.size() + amount);
    memcpy(&container.at(container.size() - amount), storage.chunk_array.entity_array.data(), amount * sizeof(Entity));
  }
}

auto Scene::SwapEntity(DataComponentStorage& storage, const size_t index1, const size_t index2) -> size_t {
  if (index1 == index2)
    return -1;
  const size_t ret_val = storage.chunk_array.entity_array[index2].index_;
  const auto other = storage.chunk_array.entity_array[index2];
  storage.chunk_array.entity_array[index2] = storage.chunk_array.entity_array[index1];
  storage.chunk_array.entity_array[index1] = other;
  const auto capacity = storage.chunk_capacity;
  const auto chunk_index1 = index1 / capacity;
  const auto chunk_index2 = index2 / capacity;
  const auto chunk_pointer1 = index1 % capacity;
  const auto chunk_pointer2 = index2 % capacity;
  for (const auto& i : storage.data_component_types) {
    void* temp = malloc(i.type_size);
    void* d1 =
        storage.chunk_array.chunks[chunk_index1].RefData(i.type_offset * capacity + i.type_size * chunk_pointer1);

    void* d2 =
        storage.chunk_array.chunks[chunk_index2].RefData(i.type_offset * capacity + i.type_size * chunk_pointer2);

    memcpy(temp, d1, i.type_size);
    memcpy(d1, d2, i.type_size);
    memcpy(d2, temp, i.type_size);
    free(temp);
  }
  return ret_val;
}

void Scene::GetAllEntities(std::vector<Entity>& target) {
  target.insert(target.end(), scene_data_storage_.entities.begin() + 1, scene_data_storage_.entities.end());
}

void Scene::ForEachDescendant(const Entity& target, const std::function<void(const Entity& entity)>& func,
                              const bool& from_root) {
  Entity real_target = target;
  if (!IsEntityValid(real_target))
    return;
  if (from_root)
    real_target = GetRoot(real_target);
  ForEachDescendantHelper(real_target, func);
}

const std::vector<Entity>& Scene::UnsafeGetAllEntities() {
  return scene_data_storage_.entities;
}

Entity Scene::CreateEntity(const std::string& name) {
  return CreateEntity(Entities::GetInstance().basic_archetype_, name);
}

Entity Scene::CreateEntity(const EntityArchetype& archetype, const std::string& name, const Handle& handle) {
  assert(archetype.IsValid());

  Entity ret_val;
  const auto search = GetDataComponentStorage(archetype);
  if (!search.has_value())
    throw std::runtime_error("Archetype not registered!");
  if (DataComponentStorage& storage = search->first; storage.entity_count == storage.entity_alive_count) {
    if (const size_t chunk_index = storage.entity_count / storage.chunk_capacity + 1;
        storage.chunk_array.chunks.size() <= chunk_index) {
      // Allocate new chunk;
      storage.chunk_array.chunks.emplace_back();
    }
    ret_val.index_ = static_cast<unsigned>(scene_data_storage_.entities.size());
    // If the version is 0 in chunk means it's deleted.
    ret_val.version_ = 1;
    EntityMetadata entity_info;
    entity_info.root = ret_val;
    entity_info.entity_static = false;
    entity_info.entity_name = name;
    entity_info.entity_handle = handle;
    entity_info.data_component_storage_index = search->second;
    entity_info.chunk_array_index = storage.entity_count;
    storage.chunk_array.entity_array.push_back(ret_val);

    scene_data_storage_.entity_map[entity_info.entity_handle] = ret_val;
    scene_data_storage_.entity_metadata_list.push_back(std::move(entity_info));
    scene_data_storage_.entities.push_back(ret_val);
    storage.entity_count++;
    storage.entity_alive_count++;
  } else {
    ret_val = storage.chunk_array.entity_array.at(storage.entity_alive_count);
    EntityMetadata& entity_info = scene_data_storage_.entity_metadata_list.at(ret_val.index_);
    entity_info.root = ret_val;
    entity_info.entity_static = false;
    entity_info.entity_handle = handle;
    entity_info.entity_enabled = true;
    entity_info.entity_name = name;
    ret_val.version_ = entity_info.entity_version;

    scene_data_storage_.entity_map[entity_info.entity_handle] = ret_val;
    storage.chunk_array.entity_array[entity_info.chunk_array_index] = ret_val;
    scene_data_storage_.entities.at(ret_val.index_) = ret_val;
    storage.entity_alive_count++;
    // Reset all component data
    const auto chunk_index = entity_info.chunk_array_index / storage.chunk_capacity;
    const auto chunk_pointer = entity_info.chunk_array_index % storage.chunk_capacity;
    auto& chunk = storage.chunk_array.chunks[chunk_index];
    for (const auto& i : storage.data_component_types) {
      const auto offset = i.type_offset * storage.chunk_capacity + chunk_pointer * i.type_size;
      chunk.ClearData(offset, i.type_size);
    }
  }
  SetDataComponent(ret_val, Transform());
  SetDataComponent(ret_val, GlobalTransform());
  SetDataComponent(ret_val, TransformUpdateFlag());
  SetUnsaved();
  return ret_val;
}

std::vector<Entity> Scene::CreateEntities(const EntityArchetype& archetype, const size_t& amount,
                                          const std::string& name) {
  assert(archetype.IsValid());
  std::vector<Entity> ret_val;
  const auto search = GetDataComponentStorage(archetype);
  if (!search.has_value())
    throw std::runtime_error("Archetype not registered!");
  DataComponentStorage& storage = search->first;
  auto remain_amount = amount;
  const Transform transform;
  const GlobalTransform global_transform;
  while (remain_amount > 0 && storage.entity_alive_count != storage.entity_count) {
    remain_amount--;
    Entity entity = storage.chunk_array.entity_array.at(storage.entity_alive_count);
    EntityMetadata& entity_info = scene_data_storage_.entity_metadata_list.at(entity.index_);
    entity_info.root = entity;
    entity_info.entity_static = false;
    entity_info.entity_enabled = true;
    entity_info.entity_name = name;
    entity.version_ = entity_info.entity_version;
    entity_info.entity_handle = Handle();
    scene_data_storage_.entity_map[entity_info.entity_handle] = entity;
    storage.chunk_array.entity_array[entity_info.chunk_array_index] = entity;
    scene_data_storage_.entities.at(entity.index_) = entity;
    storage.entity_alive_count++;
    // Reset all component data
    const size_t chunk_index = entity_info.chunk_array_index / storage.chunk_capacity;
    const size_t chunk_pointer = entity_info.chunk_array_index % storage.chunk_capacity;
    ComponentDataChunk& chunk = storage.chunk_array.chunks[chunk_index];
    for (const auto& i : storage.data_component_types) {
      const size_t offset = i.type_offset * storage.chunk_capacity + chunk_pointer * i.type_size;
      chunk.ClearData(offset, i.type_size);
    }
    ret_val.push_back(entity);
    SetDataComponent(entity, transform);
    SetDataComponent(entity, global_transform);
    SetDataComponent(entity, TransformUpdateFlag());
  }
  if (remain_amount == 0)
    return ret_val;
  storage.entity_count += remain_amount;
  storage.entity_alive_count += remain_amount;
  const size_t chunk_index = storage.entity_count / storage.chunk_capacity + 1;
  while (storage.chunk_array.chunks.size() <= chunk_index) {
    // Allocate new chunk;
    storage.chunk_array.chunks.emplace_back();
  }
  const size_t original_size = scene_data_storage_.entities.size();
  scene_data_storage_.entities.resize(original_size + remain_amount);
  scene_data_storage_.entity_metadata_list.resize(original_size + remain_amount);

  for (size_t i = 0; i < remain_amount; i++) {
    auto& entity = scene_data_storage_.entities.at(original_size + i);
    entity.index_ = static_cast<unsigned>(original_size + i);
    entity.version_ = 1;

    auto& entity_info = scene_data_storage_.entity_metadata_list.at(original_size + i);
    entity_info = EntityMetadata();
    entity_info.root = entity;
    entity_info.entity_static = false;
    entity_info.entity_name = name;
    entity_info.data_component_storage_index = search->second;
    entity_info.chunk_array_index = storage.entity_alive_count - remain_amount + i;

    entity_info.entity_handle = Handle();

    scene_data_storage_.entity_map[entity_info.entity_handle] = entity;
  }

  storage.chunk_array.entity_array.insert(storage.chunk_array.entity_array.end(),
                                          scene_data_storage_.entities.begin() + original_size,
                                          scene_data_storage_.entities.end());
  Jobs::RunParallelFor(remain_amount, [&, original_size](const size_t i) {
    const auto& entity = scene_data_storage_.entities.at(original_size + i);
    SetDataComponent(entity, transform);
    SetDataComponent(entity, global_transform);
    SetDataComponent(entity, TransformUpdateFlag());
  });

  ret_val.insert(ret_val.end(), scene_data_storage_.entities.begin() + original_size,
                 scene_data_storage_.entities.end());
  SetUnsaved();
  return ret_val;
}

std::vector<Entity> Scene::CreateEntities(const size_t& amount, const std::string& name) {
  return CreateEntities(Entities::GetInstance().basic_archetype_, amount, name);
}

void Scene::DeleteEntity(const Entity& entity) {
  if (!IsEntityValid(entity)) {
    return;
  }
  const size_t entity_index = entity.index_;
  const auto children = scene_data_storage_.entity_metadata_list.at(entity_index).children;
  for (const auto& child : children) {
    DeleteEntity(child);
  }
  if (scene_data_storage_.entity_metadata_list.at(entity_index).parent.index_ != 0)
    RemoveChild(entity, scene_data_storage_.entity_metadata_list.at(entity_index).parent);
  DeleteEntityInternal(entity.index_);
  SetUnsaved();
}

std::string Scene::GetEntityName(const Entity& entity) {
  assert(IsEntityValid(entity));
  const size_t index = entity.index_;
  if (entity != scene_data_storage_.entities.at(index)) {
    EVOENGINE_ERROR("Child already deleted!")
    return "";
  }
  return scene_data_storage_.entity_metadata_list.at(index).entity_name;
}

void Scene::SetEntityName(const Entity& entity, const std::string& name) {
  assert(IsEntityValid(entity));
  const size_t index = entity.index_;
  if (entity != scene_data_storage_.entities.at(index)) {
    EVOENGINE_ERROR("Child already deleted!")
    return;
  }
  if (name.length() != 0) {
    scene_data_storage_.entity_metadata_list.at(index).entity_name = name;
    return;
  }
  scene_data_storage_.entity_metadata_list.at(index).entity_name = "Unnamed";
  SetUnsaved();
}
void Scene::SetEntityStatic(const Entity& entity, bool value) {
  assert(IsEntityValid(entity));
  auto& entity_info = scene_data_storage_.entity_metadata_list.at(GetRoot(entity).index_);
  entity_info.entity_static = value;
  SetUnsaved();
}
void Scene::SetParent(const Entity& child, const Entity& parent, const bool& recalculate_transform) {
  assert(IsEntityValid(child) && IsEntityValid(parent));
  const size_t child_index = child.index_;
  const size_t parent_index = parent.index_;
  auto& parent_entity_info = scene_data_storage_.entity_metadata_list.at(parent_index);
  for (const auto& i : parent_entity_info.children) {
    if (i == child)
      return;
  }
  auto& child_entity_info = scene_data_storage_.entity_metadata_list.at(child_index);
  if (child_entity_info.parent.GetIndex() != 0) {
    RemoveChild(child, child_entity_info.parent);
  }

  if (recalculate_transform) {
    const auto child_global_transform = GetDataComponent<GlobalTransform>(child);
    const auto parent_global_transform = GetDataComponent<GlobalTransform>(parent);
    Transform child_transform;
    child_transform.value = glm::inverse(parent_global_transform.value) * child_global_transform.value;
    SetDataComponent(child, child_transform);
  }
  child_entity_info.parent = parent;
  if (parent_entity_info.parent.GetIndex() == child_index) {
    parent_entity_info.parent = Entity();
    parent_entity_info.root = parent;
    const size_t children_count = child_entity_info.children.size();

    for (size_t i = 0; i < children_count; i++) {
      if (child_entity_info.children[i].index_ == parent.GetIndex()) {
        child_entity_info.children[i] = child_entity_info.children.back();
        child_entity_info.children.pop_back();
        break;
      }
    }
  }
  child_entity_info.root = parent_entity_info.root;
  child_entity_info.entity_static = false;
  parent_entity_info.children.push_back(child);
  if (parent_entity_info.ancestor_selected) {
    const auto descendants = GetDescendants(child);
    for (const auto& i : descendants) {
      GetEntityMetadata(i).ancestor_selected = true;
    }
    child_entity_info.ancestor_selected = true;
  }
  SetUnsaved();
}

Entity Scene::GetParent(const Entity& entity) const {
  assert(IsEntityValid(entity));
  const size_t entity_index = entity.index_;
  return scene_data_storage_.entity_metadata_list.at(entity_index).parent;
}

std::vector<Entity> Scene::GetChildren(const Entity& entity) {
  assert(IsEntityValid(entity));
  const size_t entity_index = entity.index_;
  return scene_data_storage_.entity_metadata_list.at(entity_index).children;
}

Entity Scene::GetChild(const Entity& entity, const size_t index) const {
  assert(IsEntityValid(entity));
  const size_t entity_index = entity.index_;
  if (auto& children = scene_data_storage_.entity_metadata_list.at(entity_index).children; children.size() > index)
    return children[index];
  return {};
}

size_t Scene::GetChildrenAmount(const Entity& entity) const {
  assert(IsEntityValid(entity));
  const size_t entity_index = entity.index_;
  return scene_data_storage_.entity_metadata_list.at(entity_index).children.size();
}

void Scene::ForEachChild(const Entity& entity, const std::function<void(Entity child)>& func) const {
  assert(IsEntityValid(entity));
  const auto children = scene_data_storage_.entity_metadata_list.at(entity.index_).children;
  for (auto i : children) {
    if (IsEntityValid(i))
      func(i);
  }
}

void Scene::RemoveChild(const Entity& child, const Entity& parent) {
  assert(IsEntityValid(child) && IsEntityValid(parent));
  const size_t child_index = child.index_;
  const size_t parent_index = parent.index_;
  auto& child_entity_metadata = scene_data_storage_.entity_metadata_list.at(child_index);
  auto& parent_entity_metadata = scene_data_storage_.entity_metadata_list.at(parent_index);
  if (child_entity_metadata.parent.index_ == 0) {
    EVOENGINE_ERROR("No child by the parent!")
  }
  child_entity_metadata.parent = Entity();
  child_entity_metadata.root = child;
  if (parent_entity_metadata.ancestor_selected) {
    const auto descendants = GetDescendants(child);
    for (const auto& i : descendants) {
      GetEntityMetadata(i).ancestor_selected = false;
    }
    child_entity_metadata.ancestor_selected = false;
  }
  const size_t children_count = parent_entity_metadata.children.size();

  for (size_t i = 0; i < children_count; i++) {
    if (parent_entity_metadata.children[i].index_ == child_index) {
      parent_entity_metadata.children[i] = parent_entity_metadata.children.back();
      parent_entity_metadata.children.pop_back();
      break;
    }
  }
  const auto child_global_transform = GetDataComponent<GlobalTransform>(child);
  Transform child_transform;
  child_transform.value = child_global_transform.value;
  SetDataComponent(child, child_transform);
  SetUnsaved();
}

void Scene::RemoveDataComponent(const Entity& entity, const size_t& type_index) {
  assert(IsEntityValid(entity));
  if (type_index == typeid(Transform).hash_code() || type_index == typeid(GlobalTransform).hash_code() ||
      type_index == typeid(TransformUpdateFlag).hash_code()) {
    return;
  }
  EntityMetadata& entity_info = scene_data_storage_.entity_metadata_list.at(entity.index_);
  const auto& data_component_storage =
      scene_data_storage_.data_component_storage_list[entity_info.data_component_storage_index];
  if (data_component_storage.data_component_types.size() <= 3) {
    EVOENGINE_ERROR(
        "Remove Component Data failed: Entity must have at least 1 data component besides 3 basic data "
        "components!")
    return;
  }
#pragma region Create new archetype
  EntityArchetypeInfo new_archetype_info;
  new_archetype_info.archetype_name = "New archetype";
  new_archetype_info.data_component_types = data_component_storage.data_component_types;
  bool found = false;
  for (size_t i = 0; i < new_archetype_info.data_component_types.size(); i++) {
    if (new_archetype_info.data_component_types[i].type_index == type_index) {
      new_archetype_info.data_component_types.erase(new_archetype_info.data_component_types.begin() + i);
      found = true;
      break;
    }
  }
  if (!found) {
    EVOENGINE_ERROR("Failed to remove component data: Component not found")
    return;
  }
  size_t offset = 0;
  DataComponentType prev = new_archetype_info.data_component_types[0];
  for (auto& i : new_archetype_info.data_component_types) {
    i.type_offset = offset;
    offset += i.type_size;
  }
  new_archetype_info.entity_size = new_archetype_info.data_component_types.back().type_offset +
                                   new_archetype_info.data_component_types.back().type_size;
  new_archetype_info.chunk_capacity = Entities::GetArchetypeChunkSize() / new_archetype_info.entity_size;
  auto archetype = Entities::CreateEntityArchetypeHelper(new_archetype_info);
#pragma endregion
#pragma region Create new Entity with new archetype
  const Entity new_entity = CreateEntity(archetype);
  // Transfer component data
  for (const auto& type : new_archetype_info.data_component_types) {
    SetDataComponent(new_entity.index_, type.type_index, type.type_size,
                     GetDataComponentPointer(entity, type.type_index));
  }
  // 5. Swap entity.
  EntityMetadata& new_entity_info = scene_data_storage_.entity_metadata_list.at(new_entity.index_);
  const auto temp_archetype_info_index = new_entity_info.data_component_storage_index;
  const auto temp_chunk_array_index = new_entity_info.chunk_array_index;
  new_entity_info.data_component_storage_index = entity_info.data_component_storage_index;
  new_entity_info.chunk_array_index = entity_info.chunk_array_index;
  entity_info.data_component_storage_index = temp_archetype_info_index;
  entity_info.chunk_array_index = temp_chunk_array_index;
  // Apply to chunk.
  scene_data_storage_.data_component_storage_list.at(entity_info.data_component_storage_index)
      .chunk_array.entity_array[entity_info.chunk_array_index] = entity;
  scene_data_storage_.data_component_storage_list.at(new_entity_info.data_component_storage_index)
      .chunk_array.entity_array[new_entity_info.chunk_array_index] = new_entity;
  DeleteEntity(new_entity);
#pragma endregion
  SetUnsaved();
}

bool Scene::HasDataComponent(const Entity& entity, const size_t type_id) const {
  return HasDataComponent(entity.GetIndex(), type_id);
}

bool Scene::HasDataComponent(const size_t& index, const size_t type_id) const {
  if (index > scene_data_storage_.entity_metadata_list.size())
    return false;
  const EntityMetadata& entity_info = scene_data_storage_.entity_metadata_list.at(index);
  auto& data_component_storage =
      scene_data_storage_.data_component_storage_list[entity_info.data_component_storage_index];

  if (type_id == typeid(Transform).hash_code()) {
    return true;
  }
  if (type_id == typeid(GlobalTransform).hash_code()) {
    return true;
  }
  if (type_id == typeid(TransformUpdateFlag).hash_code()) {
    return true;
  }
  for (const auto& type : data_component_storage.data_component_types) {
    if (type.type_index == type_id) {
      return true;
    }
  }
  return false;
}

void Scene::AddDataComponent(const Entity& entity, const size_t type_id) {
  assert(IsEntityValid(entity));
  auto& entity_info = scene_data_storage_.entity_metadata_list.at(entity.index_);

#pragma region Check if componentdata already exists.If yes, go to SetComponentData
  const auto& data_component_storage =
      scene_data_storage_.data_component_storage_list.at(entity_info.data_component_storage_index);
  const auto original_component_types = data_component_storage.data_component_types;
  for (const auto& type : data_component_storage.data_component_types) {
    if (type.type_index == type_id) {
      EVOENGINE_ERROR("Data Component already exists!")
      return;
    }
  }
#pragma endregion
#pragma region If not exist, we first need to create a new archetype
  EntityArchetypeInfo new_archetype_info;
  new_archetype_info.archetype_name = "New archetype";
  new_archetype_info.data_component_types = original_component_types;
  DataComponentType data_component_type{};
  data_component_type.type_index = type_id;
  data_component_type.type_name = Serialization::GetSerializableTypeName(type_id);
  data_component_type.type_size = Serialization::GetDataComponentTypeSize(type_id);
  new_archetype_info.data_component_types.emplace_back(data_component_type);
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
                     GetDataComponentPointer(entity.index_, type.type_index));
  }
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

void Scene::SetDataComponent(const unsigned& entity_index, const size_t id, const size_t size, const void* data) {
  const auto& entity_info = scene_data_storage_.entity_metadata_list.at(entity_index);
  auto& data_component_storage =
      scene_data_storage_.data_component_storage_list[entity_info.data_component_storage_index];
  const auto chunk_index = entity_info.chunk_array_index / data_component_storage.chunk_capacity;
  const auto chunk_pointer = entity_info.chunk_array_index % data_component_storage.chunk_capacity;
  auto& chunk = data_component_storage.chunk_array.chunks[chunk_index];
  if (id == typeid(Transform).hash_code()) {
    chunk.SetData(chunk_pointer * sizeof(Transform), sizeof(Transform), data);
    static_cast<TransformUpdateFlag*>(
        chunk.RefData((sizeof(Transform) + sizeof(GlobalTransform)) * data_component_storage.chunk_capacity +
                      chunk_pointer * sizeof(TransformUpdateFlag)))
        ->transform_modified = true;
  } else if (id == typeid(GlobalTransform).hash_code()) {
    chunk.SetData(sizeof(Transform) * data_component_storage.chunk_capacity + chunk_pointer * sizeof(GlobalTransform),
                  sizeof(GlobalTransform), data);
    static_cast<TransformUpdateFlag*>(
        chunk.RefData((sizeof(Transform) + sizeof(GlobalTransform)) * data_component_storage.chunk_capacity +
                      chunk_pointer * sizeof(TransformUpdateFlag)))
        ->global_transform_modified = true;
  } else if (id == typeid(TransformUpdateFlag).hash_code()) {
    chunk.SetData((sizeof(Transform) + sizeof(GlobalTransform)) * data_component_storage.chunk_capacity +
                      chunk_pointer * sizeof(TransformUpdateFlag),
                  sizeof(TransformUpdateFlag), data);
  } else {
    for (const auto& type : data_component_storage.data_component_types) {
      if (type.type_index == id) {
        chunk.SetData(type.type_offset * data_component_storage.chunk_capacity + chunk_pointer * type.type_size, size,
                      data);
        return;
      }
    }
    EVOENGINE_LOG("ComponentData doesn't exist")
  }
  SetUnsaved();
}
void* Scene::GetDataComponentPointer(unsigned entity_index, const size_t& id) {
  const EntityMetadata& entity_info = scene_data_storage_.entity_metadata_list.at(entity_index);
  auto& data_component_storage =
      scene_data_storage_.data_component_storage_list[entity_info.data_component_storage_index];
  const auto chunk_index = entity_info.chunk_array_index / data_component_storage.chunk_capacity;
  const auto chunk_pointer = entity_info.chunk_array_index % data_component_storage.chunk_capacity;
  auto& chunk = data_component_storage.chunk_array.chunks[chunk_index];
  if (id == typeid(Transform).hash_code()) {
    return chunk.RefData(chunk_pointer * sizeof(Transform));
  }
  if (id == typeid(GlobalTransform).hash_code()) {
    return chunk.RefData(sizeof(Transform) * data_component_storage.chunk_capacity +
                         chunk_pointer * sizeof(GlobalTransform));
  }
  if (id == typeid(TransformUpdateFlag).hash_code()) {
    return chunk.RefData((sizeof(Transform) + sizeof(GlobalTransform)) * data_component_storage.chunk_capacity +
                         chunk_pointer * sizeof(TransformUpdateFlag));
  }
  for (const auto& type : data_component_storage.data_component_types) {
    if (type.type_index == id) {
      return chunk.RefData(type.type_offset * data_component_storage.chunk_capacity + chunk_pointer * type.type_size);
    }
  }
  EVOENGINE_LOG("ComponentData doesn't exist")
  return nullptr;
}
void* Scene::GetDataComponentPointer(const Entity& entity, const size_t& id) {
  assert(IsEntityValid(entity));
  EntityMetadata& entity_info = scene_data_storage_.entity_metadata_list.at(entity.index_);
  auto& data_component_storage =
      scene_data_storage_.data_component_storage_list[entity_info.data_component_storage_index];
  const auto chunk_index = entity_info.chunk_array_index / data_component_storage.chunk_capacity;
  auto chunk_pointer = entity_info.chunk_array_index % data_component_storage.chunk_capacity;
  auto& chunk = data_component_storage.chunk_array.chunks[chunk_index];
  if (id == typeid(Transform).hash_code()) {
    return chunk.RefData(chunk_pointer * sizeof(Transform));
  }
  if (id == typeid(GlobalTransform).hash_code()) {
    return chunk.RefData(sizeof(Transform) * data_component_storage.chunk_capacity +
                         chunk_pointer * sizeof(GlobalTransform));
  }
  if (id == typeid(TransformUpdateFlag).hash_code()) {
    return chunk.RefData((sizeof(Transform) + sizeof(GlobalTransform)) * data_component_storage.chunk_capacity +
                         chunk_pointer * sizeof(TransformUpdateFlag));
  }
  for (const auto& type : data_component_storage.data_component_types) {
    if (type.type_index == id) {
      return chunk.RefData(type.type_offset * data_component_storage.chunk_capacity + chunk_pointer * type.type_size);
    }
  }
  EVOENGINE_LOG("ComponentData doesn't exist")
  return nullptr;
}
Handle Scene::GetEntityHandle(const Entity& entity) {
  return scene_data_storage_.entity_metadata_list.at(entity.index_).entity_handle;
}
void Scene::SetPrivateComponent(const Entity& entity, const std::shared_ptr<IPrivateComponent>& ptr) {
  assert(ptr && IsEntityValid(entity));
  const auto type_name = ptr->GetTypeName();
  auto& elements = scene_data_storage_.entity_metadata_list.at(entity.index_).private_component_elements;
  for (auto& element : elements) {
    if (type_name == element.private_component_data->GetTypeName()) {
      return;
    }
  }

  auto id = Serialization::GetSerializableTypeId(type_name);
  scene_data_storage_.entity_private_component_storage.SetPrivateComponent(entity, id);
  elements.emplace_back(id, ptr, entity, std::dynamic_pointer_cast<Scene>(GetSelf()));
  SetUnsaved();
}

void Scene::ForEachDescendantHelper(const Entity& target, const std::function<void(const Entity& entity)>& func) {
  func(target);
  ForEachChild(target, [&](Entity child) {
    ForEachDescendantHelper(child, func);
  });
}

Entity Scene::GetRoot(const Entity& entity) const {
  Entity ret_val = entity;
  auto parent = GetParent(ret_val);
  while (parent.GetIndex() != 0) {
    ret_val = parent;
    parent = GetParent(ret_val);
  }
  return ret_val;
}

Entity Scene::GetEntity(const size_t& index) const {
  if (index > 0 && index < scene_data_storage_.entities.size())
    return scene_data_storage_.entities.at(index);
  return {};
}

void Scene::RemovePrivateComponent(const Entity& entity, size_t type_id) {
  assert(IsEntityValid(entity));
  auto& private_component_elements =
      scene_data_storage_.entity_metadata_list.at(entity.index_).private_component_elements;
  for (size_t i = 0; i < private_component_elements.size(); i++) {
    if (private_component_elements[i].type_index == type_id) {
      scene_data_storage_.entity_private_component_storage.RemovePrivateComponent(
          entity, type_id, private_component_elements[i].private_component_data);
      private_component_elements.erase(private_component_elements.begin() + i);
      SetUnsaved();
      break;
    }
  }
}

void Scene::SetEnable(const Entity& entity, const bool& value) {
  assert(IsEntityValid(entity));
  if (scene_data_storage_.entity_metadata_list.at(entity.index_).entity_enabled != value) {
    for (const auto& i : scene_data_storage_.entity_metadata_list.at(entity.index_).private_component_elements) {
      if (value) {
        i.private_component_data->OnEntityEnable();
      } else {
        i.private_component_data->OnEntityDisable();
      }
    }
  }
  scene_data_storage_.entity_metadata_list.at(entity.index_).entity_enabled = value;

  for (const auto& i : scene_data_storage_.entity_metadata_list.at(entity.index_).children) {
    SetEnable(i, value);
  }
  SetUnsaved();
}

void Scene::SetEnableSingle(const Entity& entity, const bool& value) {
  assert(IsEntityValid(entity));
  if (auto& entity_metadata = scene_data_storage_.entity_metadata_list.at(entity.index_);
      entity_metadata.entity_enabled != value) {
    for (const auto& i : entity_metadata.private_component_elements) {
      if (value) {
        i.private_component_data->OnEntityEnable();
      } else {
        i.private_component_data->OnEntityDisable();
      }
    }
    entity_metadata.entity_enabled = value;
  }
}
EntityMetadata& Scene::GetEntityMetadata(const Entity& entity) {
  assert(IsEntityValid(entity));
  return scene_data_storage_.entity_metadata_list.at(entity.index_);
}

bool Scene::HasSystem(const size_t& type_id) {
  if (const auto search = indexed_systems_.find(type_id); search != indexed_systems_.end())
    return true;
  return false;
}

std::shared_ptr<ISystem> Scene::CreateSystem(const size_t& type_id, float order) {
  const auto ptr = Serialization::ProduceSerializable(Serialization::GetSerializableTypeName(type_id));
  auto system = std::dynamic_pointer_cast<ISystem>(ptr);
  system->scene_ = std::dynamic_pointer_cast<Scene>(GetSelf());
  system->handle_ = Handle();
  system->rank_ = order;
  systems_.insert({order, system});
  indexed_systems_[type_id] = system;
  mapped_systems_[system->handle_] = system;
  system->started_ = false;
  system->OnCreate();
  SetUnsaved();
  return system;
}

void Scene::AddPrivateComponent(const Entity& entity, const size_t& type_id) {
  assert(IsEntityValid(entity));
  auto& elements = scene_data_storage_.entity_metadata_list.at(entity.index_).private_component_elements;
  auto ptr = scene_data_storage_.entity_private_component_storage.GetOrSetPrivateComponent(entity, type_id);
  elements.emplace_back(type_id, ptr, entity, std::dynamic_pointer_cast<Scene>(GetSelf()));
  SetUnsaved();
}

void Scene::ForAllEntities(const std::function<void(size_t i, Entity entity)>& func) const {
  for (size_t index = 0; index < scene_data_storage_.entities.size(); index++) {
    if (scene_data_storage_.entities.at(index).version_ != 0) {
      func(index, scene_data_storage_.entities.at(index));
    }
  }
}

Bound Scene::GetEntityBoundingBox(const Entity& entity) {
  auto descendants = GetDescendants(entity);
  descendants.emplace_back(entity);
  Bound ret_val{};
  for (const auto& walker : descendants) {
    auto gt = GetDataComponent<GlobalTransform>(walker);
    if (HasPrivateComponent<MeshRenderer>(walker)) {
      auto mesh_renderer = GetOrSetPrivateComponent<MeshRenderer>(walker).lock();
      if (const auto mesh = mesh_renderer->mesh.Get<Mesh>()) {
        auto mesh_bound = mesh->GetBound();
        mesh_bound.ApplyTransform(gt.value);
        glm::vec3 center = mesh_bound.Center();

        glm::vec3 size = mesh_bound.Size();
        ret_val.min =
            glm::vec3((glm::min)(ret_val.min.x, center.x - size.x), (glm::min)(ret_val.min.y, center.y - size.y),
                      (glm::min)(ret_val.min.z, center.z - size.z));
        ret_val.max =
            glm::vec3((glm::max)(ret_val.max.x, center.x + size.x), (glm::max)(ret_val.max.y, center.y + size.y),
                      (glm::max)(ret_val.max.z, center.z + size.z));
      }
    } else if (HasPrivateComponent<SkinnedMeshRenderer>(walker)) {
      auto mesh_renderer = GetOrSetPrivateComponent<SkinnedMeshRenderer>(walker).lock();
      if (const auto mesh = mesh_renderer->skinned_mesh.Get<SkinnedMesh>()) {
        auto mesh_bound = mesh->GetBound();
        mesh_bound.ApplyTransform(gt.value);
        glm::vec3 center = mesh_bound.Center();

        glm::vec3 size = mesh_bound.Size();
        ret_val.min =
            glm::vec3((glm::min)(ret_val.min.x, center.x - size.x), (glm::min)(ret_val.min.y, center.y - size.y),
                      (glm::min)(ret_val.min.z, center.z - size.z));
        ret_val.max =
            glm::vec3((glm::max)(ret_val.max.x, center.x + size.x), (glm::max)(ret_val.max.y, center.y + size.y),
                      (glm::max)(ret_val.max.z, center.z + size.z));
      }
    }
  }

  return ret_val;
}

void Scene::GetEntityArray(const EntityQuery& entity_query, std::vector<Entity>& container, const bool check_enable) {
  assert(entity_query.IsValid());
  const auto queried_storage_list = QueryDataComponentStorageList(entity_query);
  for (const auto i : queried_storage_list) {
    GetEntityStorage(i.get(), container, check_enable);
  }
}

size_t Scene::GetEntityAmount(const EntityQuery entity_query, const bool check_enable) {
  assert(entity_query.IsValid());
  size_t ret_val = 0;
  if (check_enable) {
    const auto queried_storage_list = QueryDataComponentStorageList(entity_query);
    for (const auto i : queried_storage_list) {
      for (size_t index = 0; index < i.get().entity_alive_count; index++) {
        if (IsEntityEnabled(i.get().chunk_array.entity_array[index]))
          ret_val++;
      }
    }
  } else {
    const auto queried_storage_list = QueryDataComponentStorageList(entity_query);
    for (const auto i : queried_storage_list) {
      ret_val += i.get().entity_alive_count;
    }
  }
  return ret_val;
}

std::vector<Entity> Scene::GetDescendants(const Entity& entity) {
  std::vector<Entity> ret_val;
  if (!IsEntityValid(entity))
    return ret_val;
  GetDescendantsHelper(entity, ret_val);
  return ret_val;
}
void Scene::GetDescendantsHelper(const Entity& target, std::vector<Entity>& results) {
  auto& children = scene_data_storage_.entity_metadata_list.at(target.index_).children;
  if (!children.empty())
    results.insert(results.end(), children.begin(), children.end());
  for (const auto& i : children)
    GetDescendantsHelper(i, results);
}

std::weak_ptr<IPrivateComponent> Scene::GetPrivateComponent(const Entity& entity, const std::string& type_name) {
  size_t i = 0;
  auto& elements = scene_data_storage_.entity_metadata_list.at(entity.index_).private_component_elements;
  for (auto& element : elements) {
    if (type_name == element.private_component_data->type_name_) {
      return element.private_component_data;
    }
    i++;
  }
  throw std::runtime_error("Private component doesn't exist!");
}

Entity Scene::GetEntity(const Handle& handle) {
  if (const auto search = scene_data_storage_.entity_map.find(handle); search != scene_data_storage_.entity_map.end()) {
    return search->second;
  }
  return {};
}
bool Scene::HasPrivateComponent(const Entity& entity, const std::string& type_name) const {
  assert(IsEntityValid(entity));
  for (auto& element : scene_data_storage_.entity_metadata_list.at(entity.index_).private_component_elements) {
    if (element.private_component_data->type_name_ == type_name) {
      return true;
    }
  }
  return false;
}

bool Scene::HasPrivateComponent(const Entity& entity, const size_t& type_id) const {
  assert(IsEntityValid(entity));
  for (auto& element : scene_data_storage_.entity_metadata_list.at(entity.index_).private_component_elements) {
    if (Serialization::GetSerializableTypeId(element.private_component_data->type_name_) == type_id) {
      return true;
    }
  }
  return false;
}

std::vector<std::reference_wrapper<DataComponentStorage>> Scene::QueryDataComponentStorageList(
    const size_t entity_query_index) {
  const auto& query_infos = Entities::GetInstance().entity_query_infos_.at(entity_query_index);
  auto& entity_component_storage = scene_data_storage_.data_component_storage_list;
  std::vector<std::reference_wrapper<DataComponentStorage>> queried_storage;
  // Select storage with every contained.
  if (!query_infos.all_data_component_types.empty()) {
    for (auto& data_storage : entity_component_storage) {
      bool check = true;
      for (const auto& type : query_infos.all_data_component_types) {
        if (!data_storage.HasType(type.type_index))
          check = false;
      }
      if (check)
        queried_storage.push_back(std::ref(data_storage));
    }
  } else {
    for (auto& data_storage : entity_component_storage) {
      queried_storage.push_back(std::ref(data_storage));
    }
  }
  // Erase with any
  if (!query_infos.any_data_component_types.empty()) {
    for (size_t i = 0; i < queried_storage.size(); i++) {
      bool contain = false;
      for (const auto& type : query_infos.any_data_component_types) {
        if (queried_storage.at(i).get().HasType(type.type_index))
          contain = true;
        if (contain)
          break;
      }
      if (!contain) {
        queried_storage.erase(queried_storage.begin() + i);
        i--;
      }
    }
  }
  // Erase with none
  if (!query_infos.none_data_component_types.empty()) {
    for (size_t i = 0; i < queried_storage.size(); i++) {
      bool contain = false;
      for (const auto& type : query_infos.none_data_component_types) {
        if (queried_storage.at(i).get().HasType(type.type_index))
          contain = true;
        if (contain)
          break;
      }
      if (contain) {
        queried_storage.erase(queried_storage.begin() + i);
        i--;
      }
    }
  }
  return queried_storage;
}
bool Scene::IsEntityValid(const Entity& entity) const {
  auto& storage = scene_data_storage_.entities;
  return entity.index_ != 0 && entity.version_ != 0 && entity.index_ < storage.size() &&
         storage.at(entity.index_).version_ == entity.version_;
}
bool Scene::IsEntityEnabled(const Entity& entity) const {
  assert(IsEntityValid(entity));
  return scene_data_storage_.entity_metadata_list.at(entity.index_).entity_enabled;
}
bool Scene::IsEntityRoot(const Entity& entity) const {
  assert(IsEntityValid(entity));
  return scene_data_storage_.entity_metadata_list.at(entity.index_).root == entity;
}
bool Scene::IsEntityStatic(const Entity& entity) const {
  assert(IsEntityValid(entity));
  return scene_data_storage_.entity_metadata_list.at(GetRoot(entity).index_).entity_static;
}

bool Scene::IsEntityAncestorSelected(const Entity& entity) const {
  assert(IsEntityValid(entity));
  return scene_data_storage_.entity_metadata_list.at(entity.index_).ancestor_selected;
}

#pragma endregion
