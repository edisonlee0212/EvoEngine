#pragma once
#include "AssetRef.hpp"
#include "ISingleton.hpp"
#include "Serialization.hpp"
namespace evo_engine {
class Resources : ISingleton<Resources> {
  Handle current_max_handle_ = Handle(1);
  std::unordered_map<std::string, std::unordered_map<Handle, std::shared_ptr<IAsset>>> typed_resources_;
  std::unordered_map<std::string, std::shared_ptr<IAsset>> named_resources_;

  std::unordered_map<std::string, std::vector<AssetRef>> shared_assets_;

  std::unordered_map<Handle, std::string> resource_names_;
  std::unordered_map<Handle, std::shared_ptr<IAsset>> resources_;
  static void LoadShaders();
  static void LoadPrimitives();
  bool show_assets_ = true;
  static void Initialize();
  static void InitializeEnvironmentalMap();
  [[nodiscard]] Handle GenerateNewHandle();
  friend class ProjectManager;
  friend class Application;

 public:
  static void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  template <class T>
  static std::shared_ptr<T> CreateResource(const std::string& name);

  [[nodiscard]] static bool IsResource(const Handle& handle);
  [[nodiscard]] static bool IsResource(const std::shared_ptr<IAsset>& target);
  [[nodiscard]] static bool IsResource(const AssetRef& target);

  template <class T>
  [[nodiscard]] static std::shared_ptr<T> GetResource(const std::string& name);
  template <class T>
  [[nodiscard]] static std::shared_ptr<T> GetResource(const Handle& handle);
};

template <class T>
std::shared_ptr<T> Resources::CreateResource(const std::string& name) {
  auto& resources = GetInstance();
  assert(resources.named_resources_.find(name) == resources.named_resources_.end());
  auto type_name = Serialization::GetSerializableTypeName<T>();
  const auto handle = resources.GenerateNewHandle();
  auto ret_val = std::dynamic_pointer_cast<IAsset>(Serialization::ProduceSerializable<T>());
  ret_val->self_ = ret_val;
  ret_val->handle_ = handle;

  resources.resource_names_[handle] = name;
  resources.typed_resources_[type_name][handle] = ret_val;
  resources.named_resources_[name] = ret_val;
  resources.resources_[handle] = ret_val;
  ret_val->OnCreate();
  return std::dynamic_pointer_cast<T>(ret_val);
}

template <class T>
std::shared_ptr<T> Resources::GetResource(const std::string& name) {
  const auto& resources = GetInstance();
  return std::dynamic_pointer_cast<T>(resources.named_resources_.at(name));
}

template <class T>
std::shared_ptr<T> Resources::GetResource(const Handle& handle) {
  const auto& resources = GetInstance();
  return std::dynamic_pointer_cast<T>(resources.resources_.at(handle));
}
}  // namespace evo_engine
