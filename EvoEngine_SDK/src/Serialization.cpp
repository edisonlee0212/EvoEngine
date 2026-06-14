#include "Serialization.hpp"

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "Console.hpp"
#include "EditorLayer.hpp"
using namespace evo_engine;

namespace {
class YamlStagedLoadPayload final : public StagedAssetLoadPayload {
 public:
  YAML::Node node;
};
}  // namespace

Serialization &Serialization::GetInstance() {
  return ApplicationContext::Get().GetSerialization();
}

const Serialization::SerializationHandlerRecord *Serialization::FindSerializationRecord(
    const Serialization &serialization, const size_t &type_id) {
  const auto search = serialization.serialization_handlers_.find(type_id);
  return search == serialization.serialization_handlers_.end() ? nullptr : &search->second;
}

const Serialization::SerializationSupportHandlerRecord *Serialization::FindSerializationSupportRecord(
    const Serialization &serialization, const size_t &type_id) {
  const auto search = serialization.serialization_support_handlers_.find(type_id);
  return search == serialization.serialization_support_handlers_.end() ? nullptr : &search->second;
}

const Serialization::AssetIoHandlerRecord *Serialization::FindAssetIoRecord(const Serialization &serialization,
                                                                            const size_t &type_id) {
  const auto search = serialization.asset_io_handlers_.find(type_id);
  return search == serialization.asset_io_handlers_.end() ? nullptr : &search->second;
}

const Serialization::AssetPreviewHandlerRecord *Serialization::FindAssetPreviewRecord(
    const Serialization &serialization, const size_t &type_id) {
  const auto search = serialization.asset_preview_handlers_.find(type_id);
  return search == serialization.asset_preview_handlers_.end() ? nullptr : &search->second;
}

void Serialization::SerializeObject(YAML::Emitter &out, const ISerializable &serializable) {
  const auto &serialization = Serialization::GetInstance();
  if (const auto *record = Serialization::FindSerializationRecord(serialization, typeid(serializable).hash_code());
      record && record->serialize_handler) {
    record->serialize_handler(out, dynamic_cast<const void *>(&serializable));
    return;
  }
  EVOENGINE_ERROR("Serialization handler is not registered for type " + std::string(typeid(serializable).name()))
}

void Serialization::DeserializeObject(const YAML::Node &in, ISerializable &serializable) {
  const auto &serialization = Serialization::GetInstance();
  if (const auto *record = Serialization::FindSerializationRecord(serialization, typeid(serializable).hash_code());
      record && record->deserialize_handler) {
    record->deserialize_handler(in, dynamic_cast<void *>(&serializable));
    return;
  }
  EVOENGINE_ERROR("Deserialization handler is not registered for type " + std::string(typeid(serializable).name()))
}

void Serialization::CollectAssetRefs(ISerializable &serializable, std::vector<AssetRef> &list) {
  const auto &serialization = Serialization::GetInstance();
  if (const auto *record =
          Serialization::FindSerializationSupportRecord(serialization, typeid(serializable).hash_code());
      record && record->collect_asset_ref_handler) {
    record->collect_asset_ref_handler(dynamic_cast<void *>(&serializable), list);
    return;
  }
}

void Serialization::RelinkObject(ISerializable &serializable, const std::unordered_map<Handle, Handle> &map,
                                 const std::shared_ptr<Scene> &scene) {
  const auto &serialization = Serialization::GetInstance();
  if (const auto *record =
          Serialization::FindSerializationSupportRecord(serialization, typeid(serializable).hash_code());
      record && record->relink_handler) {
    record->relink_handler(dynamic_cast<void *>(&serializable), map, scene);
    return;
  }
}

bool Serialization::RegisterSerializationHandler(const size_t &type_id, SerializeHandler serialize_handler,
                                                 DeserializeHandler deserialize_handler, const std::string &owner_name,
                                                 const std::string &type_name, const uint32_t version) {
  if (!serialize_handler && !deserialize_handler) {
    return false;
  }

  auto &serialization = GetInstance();
  serialization.serialization_handlers_.insert_or_assign(
      type_id, SerializationHandlerRecord{std::move(serialize_handler), std::move(deserialize_handler),
                                          SerializationHandlerInfo{type_id, type_name, owner_name, version}});
  return true;
}

bool Serialization::UnregisterSerializationHandler(const size_t &type_id) {
  return GetInstance().serialization_handlers_.erase(type_id) != 0;
}

size_t Serialization::UnregisterSerializationHandlersByOwner(const std::string &owner_name) {
  if (owner_name.empty()) {
    return 0;
  }

  auto &serialization = GetInstance();
  size_t removed = 0;
  for (auto it = serialization.serialization_handlers_.begin(); it != serialization.serialization_handlers_.end();) {
    if (it->second.info.owner_name == owner_name) {
      it = serialization.serialization_handlers_.erase(it);
      ++removed;
    } else {
      ++it;
    }
  }
  return removed;
}

bool Serialization::HasSerializationHandler(const size_t &type_id) {
  return FindSerializationRecord(GetInstance(), type_id) != nullptr;
}

const Serialization::SerializationHandlerInfo *Serialization::FindSerializationHandler(const size_t &type_id) {
  if (const auto *record = FindSerializationRecord(GetInstance(), type_id)) {
    return &record->info;
  }
  return nullptr;
}

bool Serialization::RegisterSerializationSupportHandler(const size_t &type_id,
                                                        CollectAssetRefHandler collect_asset_ref_handler,
                                                        RelinkHandler relink_handler, CloneHandler clone_handler,
                                                        const std::string &owner_name, const std::string &type_name,
                                                        const uint32_t version) {
  if (!collect_asset_ref_handler && !relink_handler && !clone_handler) {
    return false;
  }

  auto &record = GetInstance().serialization_support_handlers_[type_id];
  if (collect_asset_ref_handler) {
    record.collect_asset_ref_handler = std::move(collect_asset_ref_handler);
  }
  if (relink_handler) {
    record.relink_handler = std::move(relink_handler);
  }
  if (clone_handler) {
    record.clone_handler = std::move(clone_handler);
  }
  record.info.type_id = type_id;
  if (!type_name.empty()) {
    record.info.type_name = type_name;
  }
  if (!owner_name.empty()) {
    record.info.owner_name = owner_name;
  }
  record.info.version = version;
  return true;
}

bool Serialization::UnregisterSerializationSupportHandler(const size_t &type_id) {
  return GetInstance().serialization_support_handlers_.erase(type_id) != 0;
}

size_t Serialization::UnregisterSerializationSupportHandlersByOwner(const std::string &owner_name) {
  if (owner_name.empty()) {
    return 0;
  }

  auto &serialization = GetInstance();
  size_t removed = 0;
  for (auto it = serialization.serialization_support_handlers_.begin();
       it != serialization.serialization_support_handlers_.end();) {
    if (it->second.info.owner_name == owner_name) {
      it = serialization.serialization_support_handlers_.erase(it);
      ++removed;
    } else {
      ++it;
    }
  }
  return removed;
}

bool Serialization::HasSerializationSupportHandler(const size_t &type_id) {
  return FindSerializationSupportRecord(GetInstance(), type_id) != nullptr;
}

const Serialization::SerializationSupportHandlerInfo *Serialization::FindSerializationSupportHandler(
    const size_t &type_id) {
  if (const auto *record = FindSerializationSupportRecord(GetInstance(), type_id)) {
    return &record->info;
  }
  return nullptr;
}

bool Serialization::RegisterAssetIoHandler(const size_t &type_id, AssetSaveHandler save_handler,
                                           AssetLoadHandler load_handler,
                                           AssetSupportsStagedLoadingHandler supports_staged_loading_handler,
                                           AssetLoadStagedPayloadHandler load_staged_payload_handler,
                                           AssetApplyStagedPayloadHandler apply_staged_payload_handler,
                                           const std::string &owner_name, const std::string &type_name,
                                           const uint32_t version) {
  if (!save_handler && !load_handler && !supports_staged_loading_handler && !load_staged_payload_handler &&
      !apply_staged_payload_handler) {
    return false;
  }

  auto &record = GetInstance().asset_io_handlers_[type_id];
  if (save_handler) {
    record.save_handler = std::move(save_handler);
  }
  if (load_handler) {
    record.load_handler = std::move(load_handler);
  }
  if (supports_staged_loading_handler) {
    record.supports_staged_loading_handler = std::move(supports_staged_loading_handler);
  }
  if (load_staged_payload_handler) {
    record.load_staged_payload_handler = std::move(load_staged_payload_handler);
  }
  if (apply_staged_payload_handler) {
    record.apply_staged_payload_handler = std::move(apply_staged_payload_handler);
  }
  record.info.type_id = type_id;
  if (!type_name.empty()) {
    record.info.type_name = type_name;
  }
  if (!owner_name.empty()) {
    record.info.owner_name = owner_name;
  }
  record.info.version = version;
  return true;
}

bool Serialization::UnregisterAssetIoHandler(const size_t &type_id) {
  return GetInstance().asset_io_handlers_.erase(type_id) != 0;
}

size_t Serialization::UnregisterAssetIoHandlersByOwner(const std::string &owner_name) {
  if (owner_name.empty()) {
    return 0;
  }

  auto &serialization = GetInstance();
  size_t removed = 0;
  for (auto it = serialization.asset_io_handlers_.begin(); it != serialization.asset_io_handlers_.end();) {
    if (it->second.info.owner_name == owner_name) {
      it = serialization.asset_io_handlers_.erase(it);
      ++removed;
    } else {
      ++it;
    }
  }
  return removed;
}

bool Serialization::HasAssetIoHandler(const size_t &type_id) {
  return FindAssetIoRecord(GetInstance(), type_id) != nullptr;
}

const Serialization::AssetIoHandlerInfo *Serialization::FindAssetIoHandler(const size_t &type_id) {
  if (const auto *record = FindAssetIoRecord(GetInstance(), type_id)) {
    return &record->info;
  }
  return nullptr;
}

bool Serialization::SaveAsset(const IAsset &asset, const std::filesystem::path &path) {
  const auto &serialization = GetInstance();
  if (const auto *record = FindAssetIoRecord(serialization, typeid(asset).hash_code());
      record && record->save_handler) {
    return record->save_handler(dynamic_cast<const void *>(&asset), path);
  }
  EVOENGINE_ERROR("Asset save handler is not registered for type " + std::string(typeid(asset).name()))
  return false;
}

bool Serialization::LoadAsset(IAsset &asset, const std::filesystem::path &path) {
  const auto &serialization = GetInstance();
  if (const auto *record = FindAssetIoRecord(serialization, typeid(asset).hash_code());
      record && record->load_handler) {
    return record->load_handler(dynamic_cast<void *>(&asset), path);
  }
  EVOENGINE_ERROR("Asset load handler is not registered for type " + std::string(typeid(asset).name()))
  return false;
}

bool Serialization::SaveAssetAsYaml(const ISerializable &serializable, const std::filesystem::path &path) {
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    SerializeObject(out, serializable);
    out << YAML::EndMap;
    std::ofstream file_output(path.string());
    file_output << out.c_str();
    file_output.close();
  } catch (const std::exception &e) {
    EVOENGINE_ERROR("Failed to save: " + std::string(e.what()))
    return false;
  }
  return true;
}

bool Serialization::LoadAssetFromYaml(ISerializable &serializable, const std::filesystem::path &path) {
  if (!std::filesystem::exists(path)) {
    EVOENGINE_ERROR("Not exist!")
    return false;
  }
  try {
    const std::ifstream stream(path.string());
    std::stringstream string_stream;
    string_stream << stream.rdbuf();
    const YAML::Node in = YAML::Load(string_stream.str());
    DeserializeObject(in, serializable);
  } catch (const std::exception &e) {
    EVOENGINE_ERROR("Failed to load: " + std::string(e.what()))
    return false;
  }
  return true;
}

std::shared_ptr<StagedAssetLoadPayload> Serialization::LoadAssetYamlPayload(const std::filesystem::path &path) {
  if (!std::filesystem::exists(path)) {
    EVOENGINE_ERROR("Not exist!")
    return {};
  }
  try {
    const std::ifstream stream(path.string());
    std::stringstream string_stream;
    string_stream << stream.rdbuf();
    auto payload = std::make_shared<YamlStagedLoadPayload>();
    payload->node = YAML::Load(string_stream.str());
    return payload;
  } catch (const std::exception &e) {
    EVOENGINE_ERROR("Failed to load staged payload: " + std::string(e.what()))
    return {};
  }
}

bool Serialization::ApplyAssetYamlPayload(ISerializable &serializable,
                                          const std::shared_ptr<StagedAssetLoadPayload> &payload) {
  const auto yaml_payload = std::dynamic_pointer_cast<YamlStagedLoadPayload>(payload);
  if (!yaml_payload) {
    return false;
  }
  try {
    DeserializeObject(yaml_payload->node, serializable);
  } catch (const std::exception &e) {
    EVOENGINE_ERROR("Failed to apply staged payload: " + std::string(e.what()))
    return false;
  }
  return true;
}

bool Serialization::SupportsStagedAssetLoading(const IAsset &asset, const std::filesystem::path &path) {
  const auto &serialization = GetInstance();
  if (const auto *record = FindAssetIoRecord(serialization, typeid(asset).hash_code());
      record && record->supports_staged_loading_handler) {
    return record->supports_staged_loading_handler(dynamic_cast<const void *>(&asset), path);
  }
  return false;
}

std::shared_ptr<StagedAssetLoadPayload> Serialization::LoadStagedAssetPayload(const IAsset &asset,
                                                                              const std::filesystem::path &path) {
  const auto &serialization = GetInstance();
  if (const auto *record = FindAssetIoRecord(serialization, typeid(asset).hash_code());
      record && record->load_staged_payload_handler) {
    return record->load_staged_payload_handler(dynamic_cast<const void *>(&asset), path);
  }
  return {};
}

bool Serialization::ApplyStagedAssetPayload(IAsset &asset, const std::filesystem::path &path,
                                            const std::shared_ptr<StagedAssetLoadPayload> &payload) {
  const auto &serialization = GetInstance();
  if (const auto *record = FindAssetIoRecord(serialization, typeid(asset).hash_code());
      record && record->apply_staged_payload_handler) {
    return record->apply_staged_payload_handler(dynamic_cast<void *>(&asset), path, payload);
  }
  return false;
}

bool Serialization::RegisterAssetPreviewHandler(const size_t &type_id, AssetPreviewHandler generate_thumbnail_handler,
                                                const std::string &owner_name, const std::string &type_name,
                                                const uint32_t version) {
  if (!generate_thumbnail_handler) {
    return false;
  }

  GetInstance().asset_preview_handlers_.insert_or_assign(
      type_id, AssetPreviewHandlerRecord{std::move(generate_thumbnail_handler),
                                         AssetPreviewHandlerInfo{type_id, type_name, owner_name, version}});
  return true;
}

bool Serialization::UnregisterAssetPreviewHandler(const size_t &type_id) {
  return GetInstance().asset_preview_handlers_.erase(type_id) != 0;
}

size_t Serialization::UnregisterAssetPreviewHandlersByOwner(const std::string &owner_name) {
  if (owner_name.empty()) {
    return 0;
  }

  auto &serialization = GetInstance();
  size_t removed = 0;
  for (auto it = serialization.asset_preview_handlers_.begin(); it != serialization.asset_preview_handlers_.end();) {
    if (it->second.info.owner_name == owner_name) {
      it = serialization.asset_preview_handlers_.erase(it);
      ++removed;
    } else {
      ++it;
    }
  }
  return removed;
}

bool Serialization::HasAssetPreviewHandler(const size_t &type_id) {
  return FindAssetPreviewRecord(GetInstance(), type_id) != nullptr;
}

bool Serialization::HasAssetPreviewHandler(const std::string &type_name) {
  const auto &serialization = GetInstance();
  if (const auto id_search = serialization.serializable_ids_.find(type_name);
      id_search != serialization.serializable_ids_.end() &&
      serialization.asset_preview_handlers_.find(id_search->second) != serialization.asset_preview_handlers_.end()) {
    return true;
  }
  for (const auto &[type_id, record] : serialization.asset_preview_handlers_) {
    if (record.info.type_name == type_name) {
      return true;
    }
  }
  return false;
}

const Serialization::AssetPreviewHandlerInfo *Serialization::FindAssetPreviewHandler(const size_t &type_id) {
  if (const auto *record = FindAssetPreviewRecord(GetInstance(), type_id)) {
    return &record->info;
  }
  return nullptr;
}

std::shared_ptr<Texture2D> Serialization::GenerateAssetThumbnail(const std::shared_ptr<IAsset> &asset,
                                                                 const OffscreenPreviewSettings &settings) {
  if (!asset) {
    return {};
  }

  const auto &serialization = GetInstance();
  if (const auto *record = FindAssetPreviewRecord(serialization, typeid(*asset).hash_code());
      record && record->generate_thumbnail_handler) {
    return record->generate_thumbnail_handler(asset, settings);
  }
  return {};
}

std::shared_ptr<Texture2D> Serialization::GenerateDefaultAssetThumbnail(const OffscreenPreviewSettings &settings) {
  static_cast<void>(settings);
  return EditorLayer::FindIcon("Binary");
}

void Serialization::SerializeObject(YAML::Emitter &out, const IAsset &asset) {
  SerializeObject(out, static_cast<const ISerializable &>(asset));
}

void Serialization::SerializeObject(YAML::Emitter &out, const IPrivateComponent &component) {
  SerializeObject(out, static_cast<const ISerializable &>(component));
}

void Serialization::SerializeObject(YAML::Emitter &out, const ISystem &system) {
  SerializeObject(out, static_cast<const ISerializable &>(system));
}

void Serialization::DeserializeObject(const YAML::Node &in, IAsset &asset) {
  DeserializeObject(in, static_cast<ISerializable &>(asset));
}

void Serialization::DeserializeObject(const YAML::Node &in, IPrivateComponent &component) {
  DeserializeObject(in, static_cast<ISerializable &>(component));
}

void Serialization::DeserializeObject(const YAML::Node &in, ISystem &system) {
  DeserializeObject(in, static_cast<ISerializable &>(system));
}

void Serialization::CollectAssetRefs(IAsset &asset, std::vector<AssetRef> &list) {
  CollectAssetRefs(static_cast<ISerializable &>(asset), list);
}

void Serialization::CollectAssetRefs(IPrivateComponent &component, std::vector<AssetRef> &list) {
  CollectAssetRefs(static_cast<ISerializable &>(component), list);
}

void Serialization::CollectAssetRefs(ISystem &system, std::vector<AssetRef> &list) {
  CollectAssetRefs(static_cast<ISerializable &>(system), list);
}

void Serialization::RelinkObject(IPrivateComponent &component, const std::unordered_map<Handle, Handle> &map,
                                 const std::shared_ptr<Scene> &scene) {
  RelinkObject(static_cast<ISerializable &>(component), map, scene);
}

std::string Serialization::GetDataComponentTypeName(const size_t &type_id) {
  const auto &serialization = GetInstance();
  if (const auto search = serialization.data_component_names_.find(type_id);
      search != serialization.data_component_names_.end()) {
    return search->second;
  }
  throw std::invalid_argument("Data component type is unregistered!");
}

std::string Serialization::GetSerializableTypeName(const size_t &type_id) {
  const auto &serialization = GetInstance();
  if (const auto search = serialization.serializable_names_.find(type_id);
      search != serialization.serializable_names_.end()) {
    return search->second;
  }
  throw std::invalid_argument("Serializable type is unregistered!");
}

bool Serialization::RegisterDataComponentType(
    const std::string &type_name, const size_t &type_index, const size_t &type_size,
    const std::function<std::shared_ptr<IDataComponent>(size_t &, size_t &)> &func) {
  auto &serialization = GetInstance();
  if (serialization.data_component_names_.find(type_index) != GetInstance().data_component_names_.end()) {
    EVOENGINE_ERROR("DataComponent already registered!")
    return false;
  }
  serialization.data_component_names_[type_index] = type_name;
  serialization.data_component_sizes_[type_index] = type_size;
  serialization.data_component_ids_[type_name] = type_index;
  return serialization.data_component_generators_.insert({type_name, func}).second;
}

std::shared_ptr<IDataComponent> Serialization::ProduceDataComponent(const std::string &type_name, size_t &hash_code,
                                                                    size_t &size) {
  auto &factory = GetInstance();
  const auto it = factory.data_component_generators_.find(type_name);
  if (it != factory.data_component_generators_.end()) {
    return it->second(hash_code, size);
  }
  throw std::runtime_error("DataComponent " + type_name + "is not registered!");
}

bool Serialization::RegisterSerializableType(const std::string &type_name, const size_t &type_index,
                                             const std::function<std::shared_ptr<ISerializable>(size_t &)> &func) {
  auto &serialization_manger = GetInstance();
  if (serialization_manger.serializable_names_.find(type_index) != serialization_manger.serializable_names_.end() ||
      serialization_manger.serializable_ids_.find(type_name) != serialization_manger.serializable_ids_.end()) {
    EVOENGINE_ERROR(type_name + " already registered!")
    return false;
  }
  serialization_manger.serializable_names_[type_index] = type_name;
  serialization_manger.serializable_ids_[type_name] = type_index;
  return serialization_manger.serializable_generators_.insert({type_name, func}).second;
}
bool Serialization::RegisterPrivateComponentType(
    const std::string &type_name, const size_t &type_index,
    const std::function<void(std::shared_ptr<IPrivateComponent>, const std::shared_ptr<IPrivateComponent> &)>
        &clone_func) {
  auto &serialization = GetInstance();
  if (serialization.private_component_names_.find(type_index) != serialization.private_component_names_.end() ||
      serialization.private_component_ids_.find(type_name) != serialization.private_component_ids_.end()) {
    EVOENGINE_ERROR(type_name + " already registered!")
    return false;
  }
  serialization.private_component_names_[type_index] = type_name;
  serialization.private_component_ids_[type_name] = type_index;
  return RegisterSerializationSupportHandler(
      type_index, {}, {},
      [clone_func](const std::shared_ptr<ISerializable> &target, const std::shared_ptr<ISerializable> &source) {
        clone_func(std::dynamic_pointer_cast<IPrivateComponent>(target),
                   std::dynamic_pointer_cast<IPrivateComponent>(source));
      },
      {}, type_name);
}
bool Serialization::RegisterSystemType(
    const std::string &type_name, const size_t &type_index,
    const std::function<void(std::shared_ptr<ISystem>, const std::shared_ptr<ISystem> &)> &clone_func) {
  auto &serialization = GetInstance();
  if (serialization.system_names_.find(type_index) != serialization.system_names_.end() ||
      serialization.system_ids_.find(type_name) != serialization.system_ids_.end()) {
    EVOENGINE_ERROR(type_name + " already registered!")
    return false;
  }
  serialization.system_names_[type_index] = type_name;
  serialization.system_ids_[type_name] = type_index;
  return RegisterSerializationSupportHandler(
      type_index, {}, {},
      [clone_func](const std::shared_ptr<ISerializable> &target, const std::shared_ptr<ISerializable> &source) {
        clone_func(std::dynamic_pointer_cast<ISystem>(target), std::dynamic_pointer_cast<ISystem>(source));
      },
      {}, type_name);
}

bool Serialization::RegisterAssetType(const std::string &type_name, const size_t &type_index,
                                      const std::vector<std::string> &extensions,
                                      const std::function<std::shared_ptr<ISerializable>(size_t &)> &func) {
  auto &serialization = GetInstance();
  serialization.asset_extensions_[type_name] = extensions;
  for (const auto &extension : extensions) {
    serialization.type_names_[extension] = type_name;
  }
  return RegisterSerializableType(type_name, type_index, func);
}

bool Serialization::UnregisterSerializableType(const std::string &type_name) {
  auto &serialization = GetInstance();
  const auto id_search = serialization.serializable_ids_.find(type_name);
  if (id_search == serialization.serializable_ids_.end()) {
    return false;
  }
  const auto type_id = id_search->second;
  serialization.serializable_generators_.erase(type_name);
  serialization.serializable_ids_.erase(id_search);
  serialization.serializable_names_.erase(type_id);
  serialization.serializable_type_owners_.erase(type_name);
  serialization.serializable_type_id_owners_.erase(type_id);
  return true;
}

bool Serialization::UnregisterPrivateComponentType(const std::string &type_name) {
  auto &serialization = GetInstance();
  const auto id_search = serialization.private_component_ids_.find(type_name);
  if (id_search == serialization.private_component_ids_.end()) {
    return false;
  }
  const auto type_id = id_search->second;
  serialization.serialization_support_handlers_.erase(type_id);
  serialization.private_component_ids_.erase(id_search);
  serialization.private_component_names_.erase(type_id);
  serialization.private_component_type_owners_.erase(type_name);
  serialization.private_component_type_id_owners_.erase(type_id);
  return true;
}

bool Serialization::UnregisterAssetType(const std::string &type_name) {
  auto &serialization = GetInstance();
  const auto id_search = serialization.serializable_ids_.find(type_name);
  if (id_search != serialization.serializable_ids_.end()) {
    serialization.asset_io_handlers_.erase(id_search->second);
    serialization.asset_preview_handlers_.erase(id_search->second);
  }
  if (const auto extension_search = serialization.asset_extensions_.find(type_name);
      extension_search != serialization.asset_extensions_.end()) {
    for (const auto &extension : extension_search->second) {
      serialization.type_names_.erase(extension);
    }
    serialization.asset_extensions_.erase(extension_search);
  }
  return UnregisterSerializableType(type_name);
}

bool Serialization::UnregisterDataComponentType(const std::string &type_name) {
  auto &serialization = GetInstance();
  const auto id_search = serialization.data_component_ids_.find(type_name);
  if (id_search == serialization.data_component_ids_.end()) {
    return false;
  }
  const auto type_id = id_search->second;
  serialization.data_component_generators_.erase(type_name);
  serialization.data_component_ids_.erase(id_search);
  serialization.data_component_sizes_.erase(type_id);
  serialization.data_component_names_.erase(type_id);
  serialization.data_component_type_owners_.erase(type_name);
  serialization.data_component_type_id_owners_.erase(type_id);
  return true;
}

bool Serialization::UnregisterSystemType(const std::string &type_name) {
  auto &serialization = GetInstance();
  const auto id_search = serialization.system_ids_.find(type_name);
  if (id_search == serialization.system_ids_.end()) {
    return false;
  }
  const auto type_id = id_search->second;
  serialization.serialization_support_handlers_.erase(type_id);
  serialization.system_ids_.erase(id_search);
  serialization.system_names_.erase(type_id);
  serialization.system_type_owners_.erase(type_name);
  serialization.system_type_id_owners_.erase(type_id);
  return true;
}

const std::map<std::string, size_t> &Serialization::GetRegisteredSystemTypes() {
  return GetInstance().system_ids_;
}

void Serialization::SetSerializableTypeOwner(const std::string &type_name, const std::string &owner_name) {
  auto &serialization = GetInstance();
  if (const auto id_search = serialization.serializable_ids_.find(type_name);
      id_search != serialization.serializable_ids_.end()) {
    serialization.serializable_type_owners_[type_name] = owner_name;
    serialization.serializable_type_id_owners_[id_search->second] = owner_name;
    if (const auto support_search = serialization.serialization_support_handlers_.find(id_search->second);
        support_search != serialization.serialization_support_handlers_.end()) {
      support_search->second.info.owner_name = owner_name;
    }
    if (const auto asset_io_search = serialization.asset_io_handlers_.find(id_search->second);
        asset_io_search != serialization.asset_io_handlers_.end()) {
      asset_io_search->second.info.owner_name = owner_name;
    }
    if (const auto asset_preview_search = serialization.asset_preview_handlers_.find(id_search->second);
        asset_preview_search != serialization.asset_preview_handlers_.end()) {
      asset_preview_search->second.info.owner_name = owner_name;
    }
  }
}

void Serialization::SetPrivateComponentTypeOwner(const std::string &type_name, const std::string &owner_name) {
  auto &serialization = GetInstance();
  if (const auto id_search = serialization.private_component_ids_.find(type_name);
      id_search != serialization.private_component_ids_.end()) {
    serialization.private_component_type_owners_[type_name] = owner_name;
    serialization.private_component_type_id_owners_[id_search->second] = owner_name;
    if (const auto support_search = serialization.serialization_support_handlers_.find(id_search->second);
        support_search != serialization.serialization_support_handlers_.end()) {
      support_search->second.info.owner_name = owner_name;
    }
  }
}

void Serialization::SetDataComponentTypeOwner(const std::string &type_name, const std::string &owner_name) {
  auto &serialization = GetInstance();
  if (const auto id_search = serialization.data_component_ids_.find(type_name);
      id_search != serialization.data_component_ids_.end()) {
    serialization.data_component_type_owners_[type_name] = owner_name;
    serialization.data_component_type_id_owners_[id_search->second] = owner_name;
  }
}

void Serialization::SetSystemTypeOwner(const std::string &type_name, const std::string &owner_name) {
  auto &serialization = GetInstance();
  if (const auto id_search = serialization.system_ids_.find(type_name); id_search != serialization.system_ids_.end()) {
    serialization.system_type_owners_[type_name] = owner_name;
    serialization.system_type_id_owners_[id_search->second] = owner_name;
    if (const auto support_search = serialization.serialization_support_handlers_.find(id_search->second);
        support_search != serialization.serialization_support_handlers_.end()) {
      support_search->second.info.owner_name = owner_name;
    }
  }
}

std::vector<size_t> Serialization::GetPackageOwnedPrivateComponentTypeIds(const std::string &owner_name) {
  const auto &serialization = GetInstance();
  std::vector<size_t> ret_val;
  for (const auto &[type_id, owner] : serialization.private_component_type_id_owners_) {
    if (owner == owner_name) {
      ret_val.emplace_back(type_id);
    }
  }
  return ret_val;
}

std::vector<size_t> Serialization::GetPackageOwnedDataComponentTypeIds(const std::string &owner_name) {
  const auto &serialization = GetInstance();
  std::vector<size_t> ret_val;
  for (const auto &[type_id, owner] : serialization.data_component_type_id_owners_) {
    if (owner == owner_name) {
      ret_val.emplace_back(type_id);
    }
  }
  return ret_val;
}

std::vector<size_t> Serialization::GetPackageOwnedSystemTypeIds(const std::string &owner_name) {
  const auto &serialization = GetInstance();
  std::vector<size_t> ret_val;
  for (const auto &[type_id, owner] : serialization.system_type_id_owners_) {
    if (owner == owner_name) {
      ret_val.emplace_back(type_id);
    }
  }
  return ret_val;
}

void Serialization::UnregisterPackageOwnedTypes(const std::string &owner_name) {
  auto &serialization = GetInstance();
  UnregisterSerializationHandlersByOwner(owner_name);
  UnregisterSerializationSupportHandlersByOwner(owner_name);
  UnregisterAssetIoHandlersByOwner(owner_name);
  UnregisterAssetPreviewHandlersByOwner(owner_name);

  std::vector<std::string> data_component_names;
  for (const auto &[type_name, owner] : serialization.data_component_type_owners_) {
    if (owner == owner_name) {
      data_component_names.emplace_back(type_name);
    }
  }
  for (const auto &type_name : data_component_names) {
    UnregisterDataComponentType(type_name);
  }

  std::vector<std::string> private_component_names;
  for (const auto &[type_name, owner] : serialization.private_component_type_owners_) {
    if (owner == owner_name) {
      private_component_names.emplace_back(type_name);
    }
  }
  for (const auto &type_name : private_component_names) {
    UnregisterPrivateComponentType(type_name);
  }

  std::vector<std::string> system_names;
  for (const auto &[type_name, owner] : serialization.system_type_owners_) {
    if (owner == owner_name) {
      system_names.emplace_back(type_name);
    }
  }
  for (const auto &type_name : system_names) {
    UnregisterSystemType(type_name);
  }

  std::vector<std::string> serializable_names;
  for (const auto &[type_name, owner] : serialization.serializable_type_owners_) {
    if (owner == owner_name) {
      serializable_names.emplace_back(type_name);
    }
  }
  for (const auto &type_name : serializable_names) {
    if (HasAssetType(type_name)) {
      UnregisterAssetType(type_name);
    } else {
      UnregisterSerializableType(type_name);
    }
  }
}

std::shared_ptr<ISerializable> Serialization::ProduceSerializable(const std::string &type_name, size_t &hash_code) {
  auto &serialization = GetInstance();
  if (const auto it = serialization.serializable_generators_.find(type_name);
      it != serialization.serializable_generators_.end()) {
    auto ret_val = it->second(hash_code);
    ret_val->type_name_ = type_name;
    ret_val->handle_ = Handle();
    ret_val->application_ = &ApplicationContext::Get();
    return ret_val;
  }
  EVOENGINE_ERROR("Serializable " + type_name + " is not registered!")
  return nullptr;
}

std::shared_ptr<ISerializable> Serialization::ProduceSerializable(const std::string &type_name) {
  auto &serialization = GetInstance();
  if (const auto it = serialization.serializable_generators_.find(type_name);
      it != serialization.serializable_generators_.end()) {
    size_t temp;
    auto ret_val = it->second(temp);
    ret_val->type_name_ = type_name;
    ret_val->handle_ = Handle();
    ret_val->application_ = &ApplicationContext::Get();
    return ret_val;
  }
  EVOENGINE_ERROR("Serializable " + type_name + " is not registered!")
  return nullptr;
}

std::shared_ptr<ISerializable> Serialization::ProduceSerializable(const std::string &type_name, size_t &hash_code,
                                                                  const Handle &handle) {
  auto &serialization = GetInstance();
  const auto it = serialization.serializable_generators_.find(type_name);
  if (it != serialization.serializable_generators_.end()) {
    auto ret_val = it->second(hash_code);
    ret_val->type_name_ = type_name;
    ret_val->handle_ = handle;
    ret_val->application_ = &ApplicationContext::Get();
    return ret_val;
  }
  EVOENGINE_ERROR("PrivateComponent " + type_name + " is not registered!")
  return nullptr;
}
YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::vec2 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::vec3 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::vec4 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << v.w << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::quat &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << v.w << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::mat4 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v[0] << v[1] << v[2] << v[3] << YAML::EndSeq;
  return out;
}
YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::dvec2 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::dvec3 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::dvec4 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << v.w << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::ivec2 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::ivec3 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::ivec4 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << v.w << YAML::EndSeq;
  return out;
}
YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::uvec2 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::uvec3 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::uvec4 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << v.w << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::u16vec4 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << v.w << YAML::EndSeq;
  return out;
}

size_t Serialization::GetDataComponentTypeId(const std::string &type_name) {
  const auto &serialization = GetInstance();
  return serialization.data_component_ids_.at(type_name);
}

size_t Serialization::GetDataComponentTypeSize(const size_t &type_id) {
  const auto &serialization = GetInstance();
  return serialization.data_component_sizes_.at(type_id);
}

void Serialization::SaveAssetList(const std::string &name, const std::vector<AssetRef> &target, YAML::Emitter &out) {
  if (target.empty())
    return;
  out << YAML::Key << name << YAML::Value << YAML::BeginSeq;
  for (auto &i : target) {
    out << YAML::BeginMap;
    i.Serialize(out);
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
}

void Serialization::LoadAssetList(const std::string &name, std::vector<AssetRef> &target, const YAML::Node &in) {
  if (in[name]) {
    target.clear();
    for (const auto &i : in[name]) {
      AssetRef instance;
      instance.Deserialize(i);
      target.push_back(instance);
    }
  }
}
void Serialization::OnDestroy() {
  auto &serialization = GetInstance();
  serialization = {};
}

size_t Serialization::GetSerializableTypeId(const std::string &type_name) {
  const auto &serialization = GetInstance();
  return serialization.serializable_ids_.at(type_name);
}
bool Serialization::HasSerializableType(const std::string &type_name) {
  const auto &serialization = GetInstance();
  return serialization.serializable_ids_.find(type_name) != serialization.serializable_ids_.end();
}

bool Serialization::HasSerializableType(const size_t &type_id) {
  const auto &serialization = GetInstance();
  return serialization.serializable_names_.find(type_id) != serialization.serializable_names_.end();
}

bool Serialization::HasComponentDataType(const std::string &type_name) {
  const auto &serialization = GetInstance();
  return serialization.data_component_ids_.find(type_name) != serialization.data_component_ids_.end();
}

bool Serialization::HasComponentDataType(const size_t &type_id) {
  const auto &serialization = GetInstance();
  return serialization.data_component_names_.find(type_id) != serialization.data_component_names_.end();
}

bool Serialization::HasAssetType(const std::string &type_name) {
  const auto &serialization = GetInstance();
  return serialization.asset_extensions_.find(type_name) != serialization.asset_extensions_.end();
}

const std::vector<std::string> &Serialization::PeekAssetExtensions(const std::string &type_name) {
  const auto &serialization = GetInstance();
  if (const auto search = serialization.asset_extensions_.find(type_name);
      search != serialization.asset_extensions_.end()) {
    return search->second;
  }
  throw std::runtime_error("Asset type not registered!");
}

std::string Serialization::GetAssetTypeName(const std::string &extension) {
  const auto &serialization = GetInstance();
  if (const auto search = serialization.type_names_.find(extension); search != serialization.type_names_.end()) {
    return search->second;
  }
  return "Binary";
}

void Serialization::ClonePrivateComponent(const std::shared_ptr<IPrivateComponent> &target,
                                          const std::shared_ptr<IPrivateComponent> &source) {
  assert(typeid(*target).hash_code() == typeid(*source).hash_code());
  const auto &serialization = GetInstance();
  if (const auto *record = FindSerializationSupportRecord(serialization, typeid(*target).hash_code());
      record && record->clone_handler) {
    record->clone_handler(std::static_pointer_cast<ISerializable>(target),
                          std::static_pointer_cast<ISerializable>(source));
    return;
  }
  EVOENGINE_ERROR("PrivateComponent " + target->GetTypeName() + " clone handler is not registered!")
}
void Serialization::CloneSystem(const std::shared_ptr<ISystem> &target, const std::shared_ptr<ISystem> &source) {
  const auto &serialization = GetInstance();
  assert(typeid(*target).hash_code() == typeid(*source).hash_code());
  if (const auto *record = FindSerializationSupportRecord(serialization, typeid(*target).hash_code());
      record && record->clone_handler) {
    record->clone_handler(std::static_pointer_cast<ISerializable>(target),
                          std::static_pointer_cast<ISerializable>(source));
    return;
  }
  EVOENGINE_ERROR("System " + target->GetTypeName() + " clone handler is not registered!")
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::u8vec4 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << v.w << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::i8vec4 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << v.w << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::i16vec4 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << v.w << YAML::EndSeq;
  return out;
}
