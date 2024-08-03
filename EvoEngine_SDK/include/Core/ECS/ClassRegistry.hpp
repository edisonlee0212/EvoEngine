#pragma once
#include "Serialization.hpp"
namespace evo_engine {

template <typename T>
class DataComponentRegistration {
 public:
  DataComponentRegistration(const std::string &name);
};

template <typename T>
class AssetRegistration {
 public:
  AssetRegistration(const std::string &name, const std::vector<std::string> &external_extensions);
};
template <typename T>
class PrivateComponentRegistration {
 public:
  PrivateComponentRegistration(const std::string &name);
};
template <typename T>
class SystemRegistration {
 public:
  SystemRegistration(const std::string &name);
};

class ClassRegistry {
  template <typename T = IAsset>
  static void RegisterAsset(const std::string &name, const std::vector<std::string> &external_extensions);
  template <typename T = IDataComponent>
  static void RegisterDataComponent(const std::string &name);
  template <typename T = IPrivateComponent>
  static void RegisterPrivateComponent(const std::string &name);
  template <typename T = ISystem>
  static void RegisterSystem(const std::string &name);

  template <typename DataComponentType>
  friend class DataComponentRegistration;

  template <typename AssetType>
  friend class AssetRegistration;

  template <typename PrivateComponentType>
  friend class PrivateComponentRegistration;

  template <typename SystemType>
  friend class SystemRegistration;
};
template <typename T>
void ClassRegistry::RegisterPrivateComponent(const std::string &name) {
  Serialization::RegisterSerializableType<T>(name);
  Serialization::RegisterPrivateComponentType<T>(name);
}
template <typename T>
void ClassRegistry::RegisterDataComponent(const std::string &name) {
  Serialization::RegisterDataComponentType<T>(name);
}
template <typename T>
void ClassRegistry::RegisterAsset(const std::string &name, const std::vector<std::string> &external_extensions) {
  Serialization::RegisterAssetType<T>(name, external_extensions);
}

template <typename T>
void ClassRegistry::RegisterSystem(const std::string &name) {
  Serialization::RegisterSerializableType<T>(name);
  Serialization::RegisterSystemType<T>(name);
}

template <typename T>
DataComponentRegistration<T>::DataComponentRegistration(const std::string &name) {
  ClassRegistry::RegisterDataComponent<T>(name);
}

template <typename T>
AssetRegistration<T>::AssetRegistration(const std::string &name, const std::vector<std::string> &external_extensions) {
  ClassRegistry::RegisterAsset<T>(name, external_extensions);
}

template <typename T>
PrivateComponentRegistration<T>::PrivateComponentRegistration(const std::string &name) {
  ClassRegistry::RegisterPrivateComponent<T>(name);
}

template <typename T>
SystemRegistration<T>::SystemRegistration(const std::string &name) {
  ClassRegistry::RegisterSystem<T>(name);
}
}  // namespace evo_engine
