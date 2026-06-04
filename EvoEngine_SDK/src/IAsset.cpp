#include <IAsset.hpp>
#include "Console.hpp"
#include "EditorLayer.hpp"
#include "ProjectManager.hpp"
using namespace evo_engine;

namespace {
class YamlStagedLoadPayload final : public StagedAssetLoadPayload {
 public:
  YAML::Node node;
};
}  // namespace

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
void IAsset::Save(const std::string &name, YAML::Emitter &out) const {
  ISerializable::Save(name, out);
}
void IAsset::Load(const std::string &name, const YAML::Node &in) {
  ISerializable::Load(name, in);
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
    EVOENGINE_ERROR("Failed to save: " + std::string(e.what()))
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
    EVOENGINE_ERROR("Failed to load: " + std::string(e.what()))
    return false;
  }
  return true;
}

bool IAsset::SupportsStagedLoading() const {
  return false;
}

bool IAsset::SupportsStagedLoading(const std::filesystem::path &) const {
  return SupportsStagedLoading();
}

std::shared_ptr<StagedAssetLoadPayload> IAsset::LoadStagedPayloadInternal(const std::filesystem::path &path) const {
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

bool IAsset::ApplyStagedPayloadInternal(const std::filesystem::path &,
                                        const std::shared_ptr<StagedAssetLoadPayload> &payload) {
  const auto yaml_payload = std::dynamic_pointer_cast<YamlStagedLoadPayload>(payload);
  if (!yaml_payload) {
    return false;
  }
  try {
    Deserialize(yaml_payload->node);
  } catch (const std::exception &e) {
    EVOENGINE_ERROR("Failed to apply staged payload: " + std::string(e.what()))
    return false;
  }
  return true;
}

void IAsset::TrackPendingGpuWork(const JobHandle &handle) {
  if (!handle.Valid()) {
    return;
  }
  if (!pending_gpu_work_state_) {
    pending_gpu_work_state_ = std::make_shared<PendingGpuWorkState>();
  }
  std::lock_guard lock(pending_gpu_work_state_->mutex);
  pending_gpu_work_state_->handles.emplace_back(handle);
}

bool IAsset::HasPendingGpuWork() const {
  if (!pending_gpu_work_state_) {
    return false;
  }
  std::lock_guard lock(pending_gpu_work_state_->mutex);
  for (const auto &handle : pending_gpu_work_state_->handles) {
    if (handle.Valid()) {
      return true;
    }
  }
  return false;
}

std::vector<JobHandle> IAsset::GetPendingGpuWorkHandles() const {
  if (!pending_gpu_work_state_) {
    return {};
  }
  std::lock_guard lock(pending_gpu_work_state_->mutex);
  std::vector<JobHandle> handles;
  handles.reserve(pending_gpu_work_state_->handles.size());
  for (const auto &handle : pending_gpu_work_state_->handles) {
    if (handle.Valid()) {
      handles.emplace_back(handle);
    }
  }
  return handles;
}

void IAsset::WaitForPendingGpuWork() const {
  const auto handles = GetPendingGpuWorkHandles();
  for (const auto &handle : handles) {
    Jobs::Wait(handle);
  }
  if (!pending_gpu_work_state_) {
    return;
  }
  std::lock_guard lock(pending_gpu_work_state_->mutex);
  pending_gpu_work_state_->handles.clear();
}

void IAsset::OnCreate() {
}

bool IAsset::Export(const std::filesystem::path &path) const {
  if (ProjectManager::IsInAssetsFolder(path)) {
    EVOENGINE_ERROR("Path is in project folder!")
    return false;
  }
  return SaveInternal(path);
}
bool IAsset::Import(const std::filesystem::path &path) {
  if (!ProjectManager::GetAssetsFolderPath().empty() && ProjectManager::IsInAssetsFolder(path)) {
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
  return file_record_.expired();
}
std::weak_ptr<File> IAsset::GetFileRecord() const {
  return file_record_;
}
std::filesystem::path IAsset::GetAssetsFolderRelativePath() const {
  if (file_record_.expired())
    return {};
  return file_record_.lock()->GetAssetsFolderRelativePath();
}
std::filesystem::path IAsset::GetAbsolutePath() const {
  if (file_record_.expired())
    return {};
  return file_record_.lock()->GetAbsolutePath();
}

std::shared_ptr<Texture2D> IAsset::GenerateThumbnailTexture() {
  return EditorLayer::FindIcon("Binary");
}

uint32_t IAsset::GetVersion() const {
  return version_;
}

bool IAsset::SetPathAndSave(const std::filesystem::path &asset_folder_relative_path) {
  if (!asset_folder_relative_path.is_relative()) {
    EVOENGINE_ERROR("Not relative path!")
    return false;
  }

  if (std::filesystem::exists(ProjectManager::GetAssetsFolderPath() / asset_folder_relative_path)) {
    return false;
  }
  if (ProjectManager::IsValidAssetFileName(asset_folder_relative_path)) {
    EVOENGINE_ERROR("Asset path invalid!")
    return false;
  }
  const auto new_folder = ProjectManager::GetOrCreateFolder(asset_folder_relative_path.parent_path()).lock();
  if (!IsTemporary()) {
    const auto asset_record = file_record_.lock();
    if (const auto folder = asset_record->GetFolder().lock(); new_folder == folder) {
      asset_record->SetAssetFileName(asset_folder_relative_path.stem().string());
      asset_record->asset_ = GetSelf();
    } else {
      folder->MoveAsset(handle_, new_folder)->asset_ = GetSelf();
    }
  } else {
    auto stem = asset_folder_relative_path.stem().string();
    const auto file_name = asset_folder_relative_path.filename().string();
    auto extension = asset_folder_relative_path.extension().string();
    if (file_name == stem) {
      stem = "";
      extension = file_name;
    }
    file_record_ = new_folder->RegisterAsset(handle_, type_name_, stem, extension);
    file_record_.lock()->asset_ = GetSelf();
  }

  Save();
  return true;
}
std::string IAsset::GetTitle() const {
  return IsTemporary() ? "Temporary " + type_name_
                       : GetAssetsFolderRelativePath().stem().string() + (saved_ ? "" : " *");
}
IAsset::~IAsset() {
  AssetManager::RemoveAssetImpl(handle_);
}
