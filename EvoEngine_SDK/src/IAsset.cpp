#include <IAsset.hpp>
#include "Console.hpp"
#include "ProjectManager.hpp"
using namespace evo_engine;
bool IAsset::Save() {
  if (IsTemporary())
    return false;
  if (const auto path = GetAbsolutePath(); SaveInternal(path)) {
    saved_ = true;
    return true;
  }
  return false;
}
bool IAsset::Load() {
  if (IsTemporary())
    return false;
  if (const auto path = GetAbsolutePath(); LoadInternal(path)) {
    saved_ = true;
    return true;
  }
  return false;
}

std::shared_ptr<IAsset> IAsset::GetSelf() const {
  return self_.lock();
}

bool IAsset::SaveInternal(const std::filesystem::path &path) const {
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    Serialize(out);
    out << YAML::EndMap;
    std::ofstream file_output(path.string());
    file_output << out.c_str();
    file_output.close();
  } catch (const std::exception &e) {
    EVOENGINE_ERROR("Failed to save!")
    return false;
  }
  return true;
}
bool IAsset::LoadInternal(const std::filesystem::path &path) {
  if (!std::filesystem::exists(path)) {
    EVOENGINE_ERROR("Not exist!")
    return false;
  }
  try {
    const std::ifstream stream(path.string());
    std::stringstream string_stream;
    string_stream << stream.rdbuf();
    const YAML::Node in = YAML::Load(string_stream.str());
    Deserialize(in);
  } catch (const std::exception &e) {
    EVOENGINE_ERROR("Failed to load!")
    return false;
  }
  return true;
}

void IAsset::OnCreate() {
}

bool IAsset::Export(const std::filesystem::path &path) const {
  if (ProjectManager::IsInProjectFolder(path)) {
    EVOENGINE_ERROR("Path is in project folder!")
    return false;
  }
  return SaveInternal(path);
}
bool IAsset::Import(const std::filesystem::path &path) {
  if (!ProjectManager::GetProjectPath().empty() && ProjectManager::IsInProjectFolder(path)) {
    EVOENGINE_ERROR("Path is in project folder!")
    return false;
  }
  return LoadInternal(path);
}

void IAsset::SetUnsaved() {
  saved_ = false;
  version_++;
}
bool IAsset::Saved() const {
  return saved_;
}
bool IAsset::IsTemporary() const {
  return asset_record_.expired();
}
std::weak_ptr<AssetRecord> IAsset::GetAssetRecord() const {
  return asset_record_;
}
std::filesystem::path IAsset::GetProjectRelativePath() const {
  if (asset_record_.expired())
    return {};
  return asset_record_.lock()->GetProjectRelativePath();
}
std::filesystem::path IAsset::GetAbsolutePath() const {
  if (asset_record_.expired())
    return {};
  return asset_record_.lock()->GetAbsolutePath();
}

uint32_t IAsset::GetVersion() const {
  return version_;
}

bool IAsset::SetPathAndSave(const std::filesystem::path &project_relative_path) {
  if (!project_relative_path.is_relative()) {
    EVOENGINE_ERROR("Not relative path!")
    return false;
  }

  if (std::filesystem::exists(ProjectManager::GetProjectPath().parent_path() / project_relative_path)) {
    return false;
  }
  if (ProjectManager::IsValidAssetFileName(project_relative_path)) {
    EVOENGINE_ERROR("Asset path invalid!")
    return false;
  }
  const auto new_folder = ProjectManager::GetOrCreateFolder(project_relative_path.parent_path()).lock();
  if (!IsTemporary()) {
    const auto asset_record = asset_record_.lock();
    if (const auto folder = asset_record->GetFolder().lock(); new_folder == folder) {
      asset_record->SetAssetFileName(project_relative_path.stem().string());
    } else {
      folder->MoveAsset(handle_, new_folder);
    }
  } else {
    auto stem = project_relative_path.stem().string();
    const auto file_name = project_relative_path.filename().string();
    auto extension = project_relative_path.extension().string();
    if (file_name == stem) {
      stem = "";
      extension = file_name;
    }
    new_folder->RegisterAsset(self_.lock(), stem, extension);
  }

  Save();
  return true;
}
std::string IAsset::GetTitle() const {
  return IsTemporary() ? "Temporary " + type_name_ : GetProjectRelativePath().stem().string() + (saved_ ? "" : " *");
}
IAsset::~IAsset() {
  auto &project_manager = ProjectManager::GetInstance();
  project_manager.asset_registry_.erase(handle_);
}
