#pragma once
#include "EditorLayer.hpp"
#include "Application.hpp"
#include "Entity.hpp"
#include "Serialization.hpp"
#include "ProjectManager.hpp"
namespace EvoEngine
{
class ClassRegistry
{
  public:
    template <typename T = IAsset> static void RegisterAsset(const std::string &name, const std::vector<std::string> &externalExtensions);
    template <typename T = IDataComponent> static void RegisterDataComponent(const std::string &name);
    template <typename T = IPrivateComponent> static void RegisterPrivateComponent(const std::string &name);
    template <typename T = ISystem> static void RegisterSystem(const std::string &name);
    template <typename T = ISerializable> static void RegisterSerializable(const std::string &name);
};
template <typename T> void ClassRegistry::RegisterPrivateComponent(const std::string &name)
{
    Serialization::RegisterSerializableType<T>(name);
    Serialization::RegisterPrivateComponentType<T>(name);
    if (const auto editorLayer = Application::GetLayer<EditorLayer>())
    {
        editorLayer->RegisterPrivateComponent<T>();
    }
}
template <typename T> void ClassRegistry::RegisterDataComponent(const std::string &name)
{
    Serialization::RegisterDataComponentType<T>(name);
    if (const auto editorLayer = Application::GetLayer<EditorLayer>())
    {
        editorLayer->RegisterDataComponent<T>();
    }
}
template <typename T>
void ClassRegistry::RegisterAsset(const std::string &name, const std::vector<std::string> &externalExtensions)
{
    ProjectManager::RegisterAssetType<T>(name, externalExtensions);
}
template <typename T> void ClassRegistry::RegisterSerializable(const std::string &name)
{
    Serialization::RegisterSerializableType<T>(name);
}
template <typename T> void ClassRegistry::RegisterSystem(const std::string &name)
{
    Serialization::RegisterSerializableType<T>(name);
    Serialization::RegisterSystemType<T>(name);
    if(const auto editorLayer = Application::GetLayer<EditorLayer>())
    {
        editorLayer->RegisterSystem<T>();
    }
}

template <typename T> class DataComponentRegistration
{
  public:
    DataComponentRegistration(const std::string &name)
    {
        ClassRegistry::RegisterDataComponent<T>(name);
    }
};

template <typename T> class AssetRegistration
{
  public:
    AssetRegistration(const std::string &name, const std::vector<std::string> &externalExtensions)
    {
        ClassRegistry::RegisterAsset<T>(name, externalExtensions);
    }
};

template <typename T> class PrivateComponentRegistration
{
  public:
    PrivateComponentRegistration(const std::string &name)
    {
        ClassRegistry::RegisterPrivateComponent<T>(name);
    }
};

template <typename T> class SystemRegistration
{
  public:
    SystemRegistration(const std::string &name)
    {
        ClassRegistry::RegisterSystem<T>(name);
    }
};

template <typename T> class SerializableRegistration
{
  public:
    SerializableRegistration(const std::string &name)
    {
        ClassRegistry::RegisterSerializable<T>(name);
    }
};

} // namespace EvoEngine
