#include "Application.hpp"

#include "ApplicationContext.hpp"
#include "AssetManager.hpp"
#include "Console.hpp"
#include "Entities.hpp"
#include "FileManager.hpp"
#include "GeometryStorage.hpp"
#include "Input.hpp"
#include "Jobs.hpp"
#include "PackageManager.hpp"
#include "Platform.hpp"
#include "ProjectManager.hpp"
#include "Resources.hpp"
#include "TextureStorage.hpp"
#include "Times.hpp"
#include "TransformGraph.hpp"

using namespace evo_engine;

AssetManager& AssetManager::GetInstance() {
  return ApplicationContext::Get().GetAssetManager();
}

Console& Console::GetInstance() {
  return ApplicationContext::Get().GetConsole();
}

Entities& Entities::GetInstance() {
  return ApplicationContext::Get().GetEntities();
}

FileManager& FileManager::GetInstance() {
  return ApplicationContext::Get().GetFileManager();
}

GeometryStorage& GeometryStorage::GetInstance() {
  return ApplicationContext::Get().GetGeometryStorage();
}

Input& Input::GetInstance() {
  return ApplicationContext::Get().GetInput();
}

Jobs& Jobs::GetInstance() {
  return ApplicationContext::Get().GetJobs();
}

PackageManager& PackageManager::GetInstance() {
  return ApplicationContext::Get().GetPackageManager();
}

Platform& Platform::GetInstance() {
  return ApplicationContext::Get().GetPlatform();
}

ProjectManager& ProjectManager::GetInstance() {
  return ApplicationContext::Get().GetProjectManager();
}

Resources& Resources::GetInstance() {
  return ApplicationContext::Get().GetResources();
}

TextureStorage& TextureStorage::GetInstance() {
  return ApplicationContext::Get().GetTextureStorage();
}

TransformGraph& TransformGraph::GetInstance() {
  return ApplicationContext::Get().GetTransformGraph();
}
