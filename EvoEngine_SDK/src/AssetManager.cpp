#include "AssetManager.hpp"
#include "EditorLayer.hpp"
#include "FileManager.hpp"
#include "Jobs.hpp"
#include "ProjectManager.hpp"
#include "Resources.hpp"
#include "UnknownPrivateComponent.hpp"

#include <chrono>

using namespace evo_engine;

namespace {
std::shared_future<std::shared_ptr<IAsset>> MakeReadyAssetFuture(std::shared_ptr<IAsset> asset) {
  std::promise<std::shared_ptr<IAsset>> promise;
  promise.set_value(std::move(asset));
  return promise.get_future().share();
}

std::string AssetLoadStateName(const AssetManager::AssetLoadState state) {
  switch (state) {
    case AssetManager::AssetLoadState::Discovered:
      return "Discovered";
    case AssetManager::AssetLoadState::Queued:
      return "Queued";
    case AssetManager::AssetLoadState::LoadingCpu:
      return "Loading CPU payload";
    case AssetManager::AssetLoadState::WaitingForFinalize:
      return "Waiting for finalization";
    case AssetManager::AssetLoadState::GpuPending:
      return "Waiting for GPU finalization";
    case AssetManager::AssetLoadState::Loaded:
      return "Loaded";
    case AssetManager::AssetLoadState::Failed:
      return "Failed";
    case AssetManager::AssetLoadState::Cancelled:
      return "Cancelled";
    default:
      return "Unknown";
  }
}
}  // namespace

bool AssetManager::AssetLoadSnapshot::Active() const {
  return queued != 0 || loading_cpu != 0 || waiting_for_finalize != 0 || gpu_pending != 0;
}

void AssetManager::Initialize() {
  auto& asset_manager = GetInstance();
  asset_manager.initialized = true;
}
void AssetManager::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  auto& asset_manager = GetInstance();
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("View")) {
      ImGui::Checkbox("Assets", &asset_manager.show_asset_inspector_);
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }
  if (asset_manager.show_asset_inspector_) {
    if (ImGui::Begin("Asset Inspector")) {
      if (editor_layer->inspecting_asset) {
        const auto& asset = editor_layer->inspecting_asset;
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0.5f, 0, 1));
        ImGui::Button(asset->GetTitle().c_str());
        ImGui::PopStyleColor(1);
        editor_layer->DraggableAsset(asset);
        ImGui::SameLine();
        ImGui::Text("Type:");
        ImGui::SameLine();
        ImGui::Text(asset->GetTypeName().c_str());
        if (!asset->IsTemporary()) {
          if (ImGui::Button("Save")) {
            asset->Save();
          }
          ImGui::SameLine();
          if (ImGui::Button("Reload")) {
            asset->Load();
          }
        }
        ImGui::SameLine();
        FileUtils::SaveFile(
            "Export...", asset->GetTypeName(), Serialization::PeekAssetExtensions(asset->GetTypeName()),
            [&](const std::filesystem::path& path) {
              asset->Export(path);
            },
            false);
        ImGui::SameLine();
        FileUtils::OpenFile(
            "Import...", asset->GetTypeName(), Serialization::PeekAssetExtensions(asset->GetTypeName()),
            [&](const std::filesystem::path& path) {
              asset->Import(path);
            },
            false);

        ImGui::Separator();
        if (asset->OnInspect(editor_layer))
          asset->SetUnsaved();
      } else {
        ImGui::Text("None");
      }
    }
    ImGui::End();
  }
}

void AssetManager::OnDestroy() {
  auto& asset_manager = GetInstance();
  Clear();
  asset_manager.initialized = false;
}

size_t AssetManager::RestoreUnknownAssets() {
  auto& asset_manager = GetInstance();
  std::vector<std::pair<Handle, std::shared_ptr<UnknownAsset>>> unknown_assets;
  {
    std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
    for (auto& [handle, weak_asset] : asset_manager.asset_registry_.assets_) {
      const auto asset = weak_asset.lock();
      if (const auto unknown_asset = std::dynamic_pointer_cast<UnknownAsset>(asset)) {
        unknown_assets.emplace_back(handle, unknown_asset);
      }
    }
  }

  size_t restored_count = 0;
  for (const auto& [handle, unknown_asset] : unknown_assets) {
    const auto& original_type_name = unknown_asset->GetOriginalTypeName();
    if (original_type_name.empty() || !Serialization::HasSerializableType(original_type_name)) {
      continue;
    }

    size_t hash_code = 0;
    auto restored_asset =
        std::dynamic_pointer_cast<IAsset>(Serialization::ProduceSerializable(original_type_name, hash_code, handle));
    if (!restored_asset) {
      continue;
    }
    restored_asset->self_ = restored_asset;
    restored_asset->OnCreate();
    restored_asset->Deserialize(unknown_asset->GetSerializedNode());
    {
      std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
      asset_manager.asset_registry_.assets_[handle] = restored_asset;
    }
    ++restored_count;
  }
  return restored_count;
}

void AssetManager::Clear() {
  auto& asset_manager = GetInstance();
  struct LoadingFutureSnapshot {
    std::thread::id owner_thread_id;
    std::shared_future<std::shared_ptr<IAsset>> future;
    bool allow_same_thread_partial_access = false;
  };
  std::vector<LoadingFutureSnapshot> loading_futures;
  {
    std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
    for (const auto& [handle, record] : asset_manager.asset_registry_.loading_assets_) {
      loading_futures.push_back({record.owner_thread_id, record.future, record.allow_same_thread_partial_access});
    }
  }

  const auto current_thread_id = std::this_thread::get_id();
  for (const auto& loading_future : loading_futures) {
    const bool is_same_thread_recursive_load =
        loading_future.allow_same_thread_partial_access && loading_future.owner_thread_id == current_thread_id;
    if (loading_future.future.valid() && !is_same_thread_recursive_load) {
      try {
        WaitForAssetLoadFutureImpl(loading_future.future);
      } catch (...) {
      }
    }
  }

  {
    std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
    asset_manager.asset_registry_.assets_.clear();
    asset_manager.asset_registry_.loading_assets_.clear();
    asset_manager.asset_registry_.main_thread_asset_tasks_.clear();
    asset_manager.asset_registry_.load_snapshot_ = {};
  }
}

std::shared_ptr<IAsset> AssetManager::GetAsset(const std::string& type_name, const Handle& asset_handle) {
  if (type_name.empty()) {
    throw std::invalid_argument("Empty type name!");
  }
  if (type_name == "Binary") {
    throw std::invalid_argument("Type name is Binary!");
  }
  const auto ret_val = GetAssetImpl(asset_handle);
  if (ret_val->GetTypeName() != type_name) {
    throw std::invalid_argument("Type name mismatch!");
  }
  return ret_val;
}

void AssetManager::RemoveAssetImpl(const Handle& asset_handle) {
  auto& asset_manager = GetInstance();
  std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
  if (asset_manager.initialized &&
      asset_manager.asset_registry_.assets_.find(asset_handle) != asset_manager.asset_registry_.assets_.end())
    asset_manager.asset_registry_.assets_.erase(asset_handle);
}

std::shared_ptr<IAsset> AssetManager::GetAssetImpl(const Handle& asset_handle) {
  if (asset_handle == 0) {
    throw std::invalid_argument("Asset handle is 0!");
  }
  return WaitForAssetLoadFutureImpl(GetOrCreateAssetLoadFutureImpl(asset_handle, false));
}

std::shared_future<std::shared_ptr<IAsset>> AssetManager::RequestAssetLoad(const Handle& asset_handle) {
  return GetOrCreateAssetLoadFutureImpl(asset_handle, true);
}

std::vector<std::shared_future<std::shared_ptr<IAsset>>> AssetManager::RequestAssetLoads(
    const std::set<Handle>& asset_handles) {
  ResetAssetLoadSnapshotImpl(asset_handles.size());
  std::vector<std::shared_future<std::shared_ptr<IAsset>>> futures;
  futures.reserve(asset_handles.size());
  for (const auto& asset_handle : asset_handles) {
    futures.emplace_back(RequestAssetLoad(asset_handle));
  }
  return futures;
}

AssetManager::AssetLoadSnapshot AssetManager::GetAssetLoadSnapshot() {
  auto& asset_manager = GetInstance();
  std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
  auto snapshot = asset_manager.asset_registry_.load_snapshot_;
  snapshot.queued = 0;
  snapshot.loading_cpu = 0;
  snapshot.waiting_for_finalize = 0;
  snapshot.gpu_pending = 0;
  for (const auto& [handle, record] : asset_manager.asset_registry_.loading_assets_) {
    switch (record.state) {
      case AssetLoadState::Queued:
        ++snapshot.queued;
        break;
      case AssetLoadState::LoadingCpu:
        ++snapshot.loading_cpu;
        break;
      case AssetLoadState::WaitingForFinalize:
        ++snapshot.waiting_for_finalize;
        break;
      case AssetLoadState::GpuPending:
        ++snapshot.gpu_pending;
        break;
      default:
        break;
    }
  }
  return snapshot;
}

size_t AssetManager::ExecuteMainThreadAssetTasks(const size_t max_task_size) {
  size_t executed_task_size = 0;
  while (max_task_size == 0 || executed_task_size < max_task_size) {
    std::function<void()> task;
    {
      auto& asset_manager = GetInstance();
      std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
      if (asset_manager.asset_registry_.main_thread_asset_tasks_.empty()) {
        break;
      }
      task = std::move(asset_manager.asset_registry_.main_thread_asset_tasks_.front());
      asset_manager.asset_registry_.main_thread_asset_tasks_.pop_front();
    }
    if (task) {
      task();
      ++executed_task_size;
    }
  }
  return executed_task_size;
}

std::shared_ptr<IAsset> AssetManager::LoadAssetImpl(const Handle& asset_handle) {
  auto& asset_manager = GetInstance();
  {
    std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
    if (const auto search = asset_manager.asset_registry_.assets_.find(asset_handle);
        search != asset_manager.asset_registry_.assets_.end() && !search->second.expired()) {
      return search->second.lock();
    }
    if (const auto loading_search = asset_manager.asset_registry_.loading_assets_.find(asset_handle);
        loading_search != asset_manager.asset_registry_.loading_assets_.end()) {
      loading_search->second.owner_thread_id = std::this_thread::get_id();
    }
  }
  if (const std::shared_ptr<File> file = FileManager::GetFile(asset_handle)) {
    size_t hash_code;
    auto ret_val = std::dynamic_pointer_cast<IAsset>(Serialization::ProduceSerializable(
        Serialization::HasSerializableType(file->asset_type_name_) ? file->asset_type_name_ : "UnknownAsset", hash_code,
        asset_handle));
    if (const auto unknown_asset = std::dynamic_pointer_cast<UnknownAsset>(ret_val)) {
      unknown_asset->SetOriginalTypeName(file->asset_type_name_);
    }
    ret_val->file_record_ = file;
    ret_val->self_ = ret_val;
    SetLoadingAssetImpl(asset_handle, ret_val, true);
    ret_val->OnCreate();
    if (const auto absolute_path = file->GetAbsolutePath(); std::filesystem::exists(absolute_path)) {
      ret_val->Load();
    } else {
      ret_val->Save();
    }
    file->asset_ = ret_val;
    // file->GetThumbnail();
    {
      std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
      asset_manager.asset_registry_.assets_[asset_handle] = ret_val;
    }
    return ret_val;
  }
  return Resources::TryGetResource<IAsset>(asset_handle);
}

void AssetManager::StartAssetServiceLoadImpl(const Handle& asset_handle,
                                             const std::shared_ptr<std::promise<std::shared_ptr<IAsset>>>& promise) {
  struct AssetServiceLoadContext {
    std::shared_ptr<IAsset> asset;
    std::shared_ptr<File> file;
    std::filesystem::path absolute_path;
    bool path_exists = false;
    bool staged = false;
  };

  auto finish_with_exception = [&]() {
    promise->set_exception(std::current_exception());
    UpdateAssetLoadStateImpl(asset_handle, AssetLoadState::Failed, "Asset service load failed.");
    FinishAssetLoadingImpl(asset_handle);
  };

  try {
    UpdateAssetLoadStateImpl(asset_handle, AssetLoadState::WaitingForFinalize, "Creating asset object.");
    auto context_promise = std::make_shared<std::promise<AssetServiceLoadContext>>();
    const auto context_future = context_promise->get_future().share();
    ScheduleMainThreadAssetTaskImpl([asset_handle, context_promise]() {
      try {
        auto& asset_manager = GetInstance();
        {
          std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
          if (const auto search = asset_manager.asset_registry_.assets_.find(asset_handle);
              search != asset_manager.asset_registry_.assets_.end() && !search->second.expired()) {
            AssetServiceLoadContext context;
            context.asset = search->second.lock();
            context_promise->set_value(std::move(context));
            return;
          }
        }

        AssetServiceLoadContext context;
        context.file = FileManager::GetFile(asset_handle);
        if (context.file) {
          size_t hash_code;
          context.asset = std::dynamic_pointer_cast<IAsset>(Serialization::ProduceSerializable(
              Serialization::HasSerializableType(context.file->asset_type_name_) ? context.file->asset_type_name_
                                                                                 : "UnknownAsset",
              hash_code, asset_handle));
          if (!context.asset) {
            throw std::runtime_error("Failed to create asset instance.");
          }
          if (const auto unknown_asset = std::dynamic_pointer_cast<UnknownAsset>(context.asset)) {
            unknown_asset->SetOriginalTypeName(context.file->asset_type_name_);
          }
          context.asset->file_record_ = context.file;
          context.asset->self_ = context.asset;
          context.asset->OnCreate();
          context.absolute_path = context.file->GetAbsolutePath();
          context.path_exists = std::filesystem::exists(context.absolute_path);
          context.staged = context.path_exists && context.asset->SupportsStagedLoading(context.absolute_path);
          SetLoadingAssetImpl(asset_handle, context.asset, !context.staged);
        }
        context_promise->set_value(std::move(context));
      } catch (...) {
        context_promise->set_exception(std::current_exception());
      }
    });

    auto context = context_future.get();
    if (context.asset && !context.file) {
      promise->set_value(context.asset);
      UpdateAssetLoadStateImpl(asset_handle, AssetLoadState::Loaded, "Asset was already loaded.");
      FinishAssetLoadingImpl(asset_handle);
      return;
    }

    if (!context.file) {
      promise->set_value(Resources::TryGetResource<IAsset>(asset_handle));
      UpdateAssetLoadStateImpl(asset_handle, AssetLoadState::Loaded, "Loaded runtime resource.");
      FinishAssetLoadingImpl(asset_handle);
      return;
    }

    if (context.staged) {
      UpdateAssetLoadStateImpl(asset_handle, AssetLoadState::LoadingCpu, "Loading CPU payload.");
      auto payload = context.asset->LoadStagedPayloadInternal(context.absolute_path);
      if (!payload) {
        throw std::runtime_error("Failed to build staged asset payload.");
      }
      UpdateAssetLoadStateImpl(asset_handle, AssetLoadState::WaitingForFinalize, "Waiting for asset finalization.");
      ScheduleMainThreadAssetTaskImpl(
          [asset_handle, context = std::move(context), promise, payload = std::move(payload)]() {
            try {
              if (!context.asset->ApplyStagedPayloadInternal(context.absolute_path, payload)) {
                throw std::runtime_error("Failed to apply staged asset load payload.");
              }
              context.asset->saved_ = true;
              auto publish_loaded_asset = [asset_handle, context, promise]() {
                context.file->asset_ = context.asset;
                {
                  auto& asset_manager = GetInstance();
                  std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
                  asset_manager.asset_registry_.assets_[asset_handle] = context.asset;
                }
                promise->set_value(context.asset);
                UpdateAssetLoadStateImpl(asset_handle, AssetLoadState::Loaded, "Asset loaded.");
                FinishAssetLoadingImpl(asset_handle);
              };
              if (context.asset->HasPendingGpuWork()) {
                UpdateAssetLoadStateImpl(asset_handle, AssetLoadState::GpuPending, "Waiting for GPU finalization.");
                JobOptions gpu_ready_options;
                gpu_ready_options.executor = JobExecutorType::Background;
                gpu_ready_options.affinity = JobThreadAffinity::Background;
                gpu_ready_options.debug_name = "AssetManager::WaitForAssetGpuReady";
                const auto gpu_ready_handle = Jobs::Run(
                    context.asset->GetPendingGpuWorkHandles(), gpu_ready_options, [asset_handle, context, promise]() {
                      try {
                        context.asset->WaitForPendingGpuWork();
                        ScheduleMainThreadAssetTaskImpl([asset_handle, context, promise]() {
                          context.file->asset_ = context.asset;
                          {
                            auto& asset_manager = GetInstance();
                            std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
                            asset_manager.asset_registry_.assets_[asset_handle] = context.asset;
                          }
                          promise->set_value(context.asset);
                          UpdateAssetLoadStateImpl(asset_handle, AssetLoadState::Loaded, "Asset loaded.");
                          FinishAssetLoadingImpl(asset_handle);
                        });
                      } catch (const std::exception& e) {
                        promise->set_exception(std::current_exception());
                        UpdateAssetLoadStateImpl(asset_handle, AssetLoadState::Failed, e.what());
                        FinishAssetLoadingImpl(asset_handle);
                      } catch (...) {
                        promise->set_exception(std::current_exception());
                        UpdateAssetLoadStateImpl(asset_handle, AssetLoadState::Failed,
                                                 "Unknown asset GPU finalization failure.");
                        FinishAssetLoadingImpl(asset_handle);
                      }
                    });
                if (!gpu_ready_handle.Valid()) {
                  throw std::runtime_error("Failed to schedule asset GPU readiness wait.");
                }
                Jobs::Execute(gpu_ready_handle);
                return;
              }
              publish_loaded_asset();
            } catch (const std::exception& e) {
              promise->set_exception(std::current_exception());
              UpdateAssetLoadStateImpl(asset_handle, AssetLoadState::Failed, e.what());
              FinishAssetLoadingImpl(asset_handle);
            } catch (...) {
              promise->set_exception(std::current_exception());
              UpdateAssetLoadStateImpl(asset_handle, AssetLoadState::Failed, "Unknown asset finalization failure.");
              FinishAssetLoadingImpl(asset_handle);
            }
          });
      return;
    }

    UpdateAssetLoadStateImpl(asset_handle, AssetLoadState::WaitingForFinalize, "Waiting for legacy asset load.");
    ScheduleMainThreadAssetTaskImpl([asset_handle, context = std::move(context), promise]() {
      try {
        if (context.path_exists) {
          context.asset->Load();
        } else {
          context.asset->Save();
        }
        context.file->asset_ = context.asset;
        {
          auto& asset_manager = GetInstance();
          std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
          asset_manager.asset_registry_.assets_[asset_handle] = context.asset;
        }
        promise->set_value(context.asset);
        UpdateAssetLoadStateImpl(asset_handle, AssetLoadState::Loaded, "Asset loaded.");
      } catch (const std::exception& e) {
        promise->set_exception(std::current_exception());
        UpdateAssetLoadStateImpl(asset_handle, AssetLoadState::Failed, e.what());
      } catch (...) {
        promise->set_exception(std::current_exception());
        UpdateAssetLoadStateImpl(asset_handle, AssetLoadState::Failed, "Unknown asset load failure.");
      }
      FinishAssetLoadingImpl(asset_handle);
    });
  } catch (...) {
    finish_with_exception();
  }
}

std::shared_ptr<IAsset> AssetManager::WaitForAssetLoadFutureImpl(
    const std::shared_future<std::shared_ptr<IAsset>>& asset_future) {
  if (!asset_future.valid()) {
    return {};
  }
  while (asset_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
    if (Jobs::IsMainThread()) {
      const auto executed_asset_task_size = ExecuteMainThreadAssetTasks(1);
      const auto executed_job_size = Jobs::ExecuteMainThreadJobs(1);
      if (executed_asset_task_size == 0 && executed_job_size == 0) {
        asset_future.wait_for(std::chrono::milliseconds(1));
      }
    } else {
      asset_future.wait_for(std::chrono::milliseconds(1));
    }
  }
  return asset_future.get();
}

std::shared_future<std::shared_ptr<IAsset>> AssetManager::GetOrCreateAssetLoadFutureImpl(const Handle& asset_handle,
                                                                                         const bool async) {
  if (asset_handle == 0) {
    throw std::invalid_argument("Asset handle is 0!");
  }

  auto& asset_manager = GetInstance();
  std::string asset_name;
  if (const auto file = FileManager::GetFile(asset_handle)) {
    asset_name = file->GetAssetsFolderRelativePath().string();
  }
  auto promise = std::make_shared<std::promise<std::shared_ptr<IAsset>>>();
  auto future = promise->get_future().share();
  {
    std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
    if (const auto search = asset_manager.asset_registry_.assets_.find(asset_handle);
        search != asset_manager.asset_registry_.assets_.end() && !search->second.expired()) {
      return MakeReadyAssetFuture(search->second.lock());
    }
    if (const auto loading_search = asset_manager.asset_registry_.loading_assets_.find(asset_handle);
        loading_search != asset_manager.asset_registry_.loading_assets_.end()) {
      if (loading_search->second.allow_same_thread_partial_access &&
          loading_search->second.owner_thread_id == std::this_thread::get_id()) {
        if (const auto loading_asset = loading_search->second.loading_asset.lock()) {
          return MakeReadyAssetFuture(loading_asset);
        }
        throw std::runtime_error("Recursive asset load requested before asset object was created.");
      }
      return loading_search->second.future;
    }

    AssetRegistry::AssetLoadingRecord record;
    record.future = future;
    record.owner_thread_id = async ? std::thread::id() : std::this_thread::get_id();
    record.async = async;
    record.allow_same_thread_partial_access = !async;
    record.state = AssetLoadState::Queued;
    record.asset_name = asset_name;
    asset_manager.asset_registry_.loading_assets_[asset_handle] = record;

    auto& snapshot = asset_manager.asset_registry_.load_snapshot_;
    const auto finished_size = snapshot.completed + snapshot.failed + snapshot.cancelled;
    if (snapshot.total == 0 || (!snapshot.Active() && finished_size >= snapshot.total)) {
      snapshot = {};
      snapshot.total = 1;
    }
    snapshot.active_asset_handle = asset_handle;
    snapshot.active_state = AssetLoadState::Queued;
    snapshot.active_asset_name = asset_name;
    snapshot.message = "Queued";
  }

  try {
    const auto job_handle = Jobs::RunOnAssetIoThread([asset_handle, promise]() {
      StartAssetServiceLoadImpl(asset_handle, promise);
    });
    if (!job_handle.Valid()) {
      throw std::runtime_error("Failed to schedule asset service task.");
    }
    Jobs::Execute(job_handle);
  } catch (...) {
    promise->set_exception(std::current_exception());
    UpdateAssetLoadStateImpl(asset_handle, AssetLoadState::Failed, "Failed to schedule asset service task.");
    FinishAssetLoadingImpl(asset_handle);
  }
  return future;
}

void AssetManager::ScheduleMainThreadAssetTaskImpl(const std::function<void()>& action) {
  auto& asset_manager = GetInstance();
  std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
  asset_manager.asset_registry_.main_thread_asset_tasks_.emplace_back(action);
}

void AssetManager::ResetAssetLoadSnapshotImpl(const size_t total) {
  auto& asset_manager = GetInstance();
  std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
  asset_manager.asset_registry_.load_snapshot_ = {};
  asset_manager.asset_registry_.load_snapshot_.total = total;
}

void AssetManager::UpdateAssetLoadStateImpl(const Handle& asset_handle, const AssetLoadState state,
                                            const std::string& message) {
  auto& asset_manager = GetInstance();
  std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
  auto& snapshot = asset_manager.asset_registry_.load_snapshot_;
  snapshot.active_asset_handle = asset_handle;
  snapshot.active_state = state;
  snapshot.message = message.empty() ? AssetLoadStateName(state) : message;
  if (const auto search = asset_manager.asset_registry_.loading_assets_.find(asset_handle);
      search != asset_manager.asset_registry_.loading_assets_.end()) {
    search->second.state = state;
    search->second.message = snapshot.message;
    snapshot.active_asset_name = search->second.asset_name;
  } else {
    snapshot.active_asset_name.clear();
  }
}

void AssetManager::SetLoadingAssetImpl(const Handle& asset_handle, const std::shared_ptr<IAsset>& asset,
                                       const bool allow_same_thread_partial_access) {
  auto& asset_manager = GetInstance();
  std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
  if (const auto search = asset_manager.asset_registry_.loading_assets_.find(asset_handle);
      search != asset_manager.asset_registry_.loading_assets_.end()) {
    search->second.loading_asset = asset;
    search->second.owner_thread_id = std::this_thread::get_id();
    search->second.allow_same_thread_partial_access = allow_same_thread_partial_access;
  }
}

void AssetManager::FinishAssetLoadingImpl(const Handle& asset_handle) {
  auto& asset_manager = GetInstance();
  std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
  if (const auto search = asset_manager.asset_registry_.loading_assets_.find(asset_handle);
      search != asset_manager.asset_registry_.loading_assets_.end()) {
    auto& snapshot = asset_manager.asset_registry_.load_snapshot_;
    switch (search->second.state) {
      case AssetLoadState::Failed:
        ++snapshot.failed;
        break;
      case AssetLoadState::Cancelled:
        ++snapshot.cancelled;
        break;
      default:
        ++snapshot.completed;
        break;
    }
    if (snapshot.total == 0) {
      snapshot.total = snapshot.completed + snapshot.failed + snapshot.cancelled;
    }
    asset_manager.asset_registry_.loading_assets_.erase(search);
  }
}

std::shared_future<std::shared_ptr<IAsset>> AssetManager::GetAssetFutureImpl(const Handle& asset_handle) {
  return GetOrCreateAssetLoadFutureImpl(asset_handle, true);
}

std::shared_ptr<IAsset> AssetManager::CreateTemporaryAsset(const std::string& type_name) {
  return CreateTemporaryAssetImpl(type_name, Handle());
}

std::shared_ptr<IAsset> AssetManager::GetAsset(const Handle& asset_handle) {
  return GetAssetImpl(asset_handle);
}

std::shared_ptr<IAsset> AssetManager::CreateTemporaryAssetImpl(const std::string& type_name,
                                                               const Handle& asset_handle) {
  size_t hash_code;
  auto ret_val =
      std::dynamic_pointer_cast<IAsset>(Serialization::ProduceSerializable(type_name, hash_code, asset_handle));
  if (!ret_val) {
    return nullptr;
  }
  {
    auto& asset_manager = GetInstance();
    std::lock_guard lock(asset_manager.asset_registry_.asset_registry_mutex);
    asset_manager.asset_registry_.assets_[ret_val->GetHandle()] = ret_val;
  }
  ret_val->self_ = ret_val;
  ret_val->OnCreate();
  return ret_val;
}
