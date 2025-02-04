
#pragma once
#include "Serialization.hpp"

namespace evo_engine {

/**
 * @brief Template class for registering data components.
 *
 * @tparam T The type of the data component to register.
 */
template <typename T>
class DataComponentRegistration {
 public:
  /**
   * @brief Constructs a DataComponentRegistration object and
   *        registers a data component with the given name.
   *
   * @param name The name of the data component to register.
   */
  DataComponentRegistration(const std::string &name);
};

/**
 * @brief Template class for registering assets.
 *
 * @tparam T The type of the asset to register.
 */
template <typename T>
class AssetRegistration {
 public:
  /**
   * @brief Constructs an AssetRegistration object and
   *        registers an asset with the given name and external extensions.
   *
   * @param name The name of the asset to register.
   * @param external_extensions A list of external extensions associated with the asset.
   */
  AssetRegistration(const std::string &name, const std::vector<std::string> &external_extensions);
};

/**
 * @brief Template class for registering private components.
 *
 * @tparam T The type of the private component to register.
 */
template <typename T>
class PrivateComponentRegistration {
 public:
  /**
   * @brief Constructs a PrivateComponentRegistration object and
   *        registers a private component with the given name.
   *
   * @param name The name of the private component to register.
   */
  PrivateComponentRegistration(const std::string &name);
};

/**
 * @brief Template class for registering systems.
 *
 * @tparam T The type of the system to register.
 */
template <typename T>
class SystemRegistration {
 public:
  /**
   * @brief Constructs a SystemRegistration object and
   *        registers a system with the given name.
   *
   * @param name The name of the system to register.
   */
  SystemRegistration(const std::string &name);
};

/**
 * @brief A class that serves as the central registry for registering
 *        data components, private components, assets, and systems.
 */
class ClassRegistry {
  /**
   * @brief Registers an asset with the given name and associated external extensions.
   *
   * @tparam T The type of the asset to register.
   * @param name The name of the asset.
   * @param external_extensions A list of external extensions associated with the asset.
   */
  template <typename T = IAsset>
  static void RegisterAsset(const std::string &name, const std::vector<std::string> &external_extensions);

  /**
   * @brief Registers a data component with the given name.
   *
   * @tparam T The type of the data component to register.
   * @param name The name of the data component.
   */
  template <typename T = IDataComponent>
  static void RegisterDataComponent(const std::string &name);

  /**
   * @brief Registers a private component with the given name.
   *
   * @tparam T The type of the private component to register.
   * @param name The name of the private component.
   */
  template <typename T = IPrivateComponent>
  static void RegisterPrivateComponent(const std::string &name);

  /**
   * @brief Registers a system with the given name.
   *
   * @tparam T The type of the system to register.
   * @param name The name of the system.
   */
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

/**
 * @brief Registers a private component with the given name.
 *
 * @tparam T The type of the private component to register.
 * @param name The name of the private component.
 */
template <typename T>
void ClassRegistry::RegisterPrivateComponent(const std::string &name) {
  Serialization::RegisterSerializableType<T>(name);
  Serialization::RegisterPrivateComponentType<T>(name);
}

/**
 * @brief Registers a data component with the given name.
 *
 * @tparam T The type of the data component to register.
 * @param name The name of the data component.
 */
template <typename T>
void ClassRegistry::RegisterDataComponent(const std::string &name) {
  Serialization::RegisterDataComponentType<T>(name);
}

/**
 * @brief Registers an asset with the given name and external extensions.
 *
 * @tparam T The type of the asset to register.
 * @param name The name of the asset.
 * @param external_extensions A list of external extensions associated with the asset.
 */
template <typename T>
void ClassRegistry::RegisterAsset(const std::string &name, const std::vector<std::string> &external_extensions) {
  Serialization::RegisterAssetType<T>(name, external_extensions);
}

/**
 * @brief Registers a system with the given name.
 *
 * @tparam T The type of the system to register.
 * @param name The name of the system.
 */
template <typename T>
void ClassRegistry::RegisterSystem(const std::string &name) {
  Serialization::RegisterSerializableType<T>(name);
  Serialization::RegisterSystemType<T>(name);
}

/**
 * @brief Constructs a DataComponentRegistration object and
 *        calls the ClassRegistry to register the data component.
 *
 * @tparam T The type of the data component to register.
 * @param name The name of the data component to register.
 */
template <typename T>
DataComponentRegistration<T>::DataComponentRegistration(const std::string &name) {
  ClassRegistry::RegisterDataComponent<T>(name);
}

/**
 * @brief Constructs an AssetRegistration object and
 *        calls the ClassRegistry to register the asset.
 *
 * @tparam T The type of the asset to register.
 * @param name The name of the asset to register.
 * @param external_extensions A list of external extensions associated with the asset.
 */
template <typename T>
AssetRegistration<T>::AssetRegistration(const std::string &name, const std::vector<std::string> &external_extensions) {
  ClassRegistry::RegisterAsset<T>(name, external_extensions);
}

/**
 * @brief Constructs a PrivateComponentRegistration object and
 *        calls the ClassRegistry to register the private component.
 *
 * @tparam T The type of the private component to register.
 * @param name The name of the private component to register.
 */
template <typename T>
PrivateComponentRegistration<T>::PrivateComponentRegistration(const std::string &name) {
  ClassRegistry::RegisterPrivateComponent<T>(name);
}

/**
 * @brief Constructs a SystemRegistration object and
 *        calls the ClassRegistry to register the system.
 *
 * @tparam T The type of the system to register.
 * @param name The name of the system to register.
 */
template <typename T>
SystemRegistration<T>::SystemRegistration(const std::string &name) {
  ClassRegistry::RegisterSystem<T>(name);
}

}  // namespace evo_engine
