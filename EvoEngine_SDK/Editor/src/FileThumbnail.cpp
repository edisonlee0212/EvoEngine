#include "Application.hpp"
#include "AssetManager.hpp"
#include "AssetThumbnailProvider.hpp"
#include "EditorLayer.hpp"
#include "FileManager.hpp"
#include "Platform.hpp"

using namespace evo_engine;

namespace {
constexpr uint32_t kMaxThumbnailGenerationsPerFrame = 2;
constexpr auto kThumbnailGenerationBudget = std::chrono::milliseconds(8);
constexpr glm::uvec2 kProjectThumbnailResolution = {256, 256};

uint32_t thumbnail_generation_frame = 0;
uint32_t thumbnail_generation_count = 0;
std::chrono::steady_clock::duration thumbnail_generation_duration{};

bool CanGenerateThumbnailThisFrame() {
  if (!Platform::Initialized()) {
    return true;
  }

  const auto current_frame_index = Platform::GetFrameCount();
  if (thumbnail_generation_frame != current_frame_index) {
    thumbnail_generation_frame = current_frame_index;
    thumbnail_generation_count = 0;
    thumbnail_generation_duration = {};
  }
  if (thumbnail_generation_count >= kMaxThumbnailGenerationsPerFrame ||
      thumbnail_generation_duration >= kThumbnailGenerationBudget) {
    return false;
  }
  ++thumbnail_generation_count;
  return true;
}

void RecordThumbnailGenerationDuration(const std::chrono::steady_clock::duration duration) {
  thumbnail_generation_duration += duration;
}

struct FileThumbnail {
  uint64_t content_version_ = 0;
  std::shared_ptr<Texture2D> thumbnail_;
  std::shared_future<std::shared_ptr<IAsset>> thumbnail_future_;
  std::filesystem::file_time_type thumbnail_source_write_time_;
  bool thumbnail_source_write_time_initialized_ = false;
  bool thumbnail_asset_reload_required_ = false;
  std::shared_ptr<Texture2D> GetThumbnail(const File& file, bool allow_asset_load);
  void InvalidateThumbnail();
  void SyncThumbnailSourceWriteTime(const File& file);
  std::shared_ptr<Texture2D> GetFallbackThumbnail(const File& file) const;
};
using ThumbnailCache = std::map<std::weak_ptr<File>, FileThumbnail, std::owner_less<std::weak_ptr<File>>>;
auto& FileThumbnailCaches() {
  static std::unordered_map<Application*, ThumbnailCache> caches;
  return caches;
}
ThumbnailCache& FileThumbnails() {
  auto* application = &ApplicationContext::Get();
  auto [it, inserted] = FileThumbnailCaches().try_emplace(application);
  if (inserted) {
    static_cast<void>(application->RegisterCleanupFunction([application] {
      FileThumbnailCaches().erase(application);
    }));
  }
  return it->second;
}
std::shared_ptr<Texture2D> FileThumbnail::GetThumbnail(const File& file, const bool allow_asset_load) {
  const auto fallback_thumbnail = GetFallbackThumbnail(file);
  if (!AssetThumbnailProvider::SupportsGeneratedThumbnail(file.GetAssetTypeName())) {
    return fallback_thumbnail;
  }

  if (content_version_ != file.GetContentVersion()) {
    InvalidateThumbnail();
    content_version_ = file.GetContentVersion();
  }
  SyncThumbnailSourceWriteTime(file);
  if (thumbnail_) {
    return thumbnail_;
  }
  if (!allow_asset_load) {
    return fallback_thumbnail;
  }

  if (!thumbnail_future_.valid()) {
    try {
      thumbnail_future_ = AssetManager::RequestAssetLoad(file.GetAssetHandle());
    } catch (const std::exception& e) {
      EVOENGINE_ERROR("Failed to request thumbnail asset load: " + std::string(e.what()))
      thumbnail_ = fallback_thumbnail;
    } catch (...) {
      EVOENGINE_ERROR("Failed to request thumbnail asset load.")
      thumbnail_ = fallback_thumbnail;
    }
    return fallback_thumbnail;
  }
  if (thumbnail_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready ||
      !CanGenerateThumbnailThisFrame()) {
    return fallback_thumbnail;
  }

  try {
    auto asset = thumbnail_future_.get();
    thumbnail_future_ = {};
    if (!asset) {
      return fallback_thumbnail;
    }

    if (thumbnail_asset_reload_required_) {
      thumbnail_asset_reload_required_ = false;
      if (!asset->Load()) {
        thumbnail_ = fallback_thumbnail;
        return fallback_thumbnail;
      }
    }

    const auto generation_start = std::chrono::steady_clock::now();
    OffscreenPreviewSettings settings;
    settings.resolution = kProjectThumbnailResolution;
    thumbnail_ = AssetThumbnailProvider::GenerateThumbnail(asset, settings);
    RecordThumbnailGenerationDuration(std::chrono::steady_clock::now() - generation_start);
  } catch (const std::exception& e) {
    EVOENGINE_ERROR("Failed to generate thumbnail: " + std::string(e.what()))
    thumbnail_future_ = {};
    thumbnail_ = fallback_thumbnail;
  } catch (...) {
    EVOENGINE_ERROR("Failed to generate thumbnail.")
    thumbnail_future_ = {};
    thumbnail_ = fallback_thumbnail;
  }

  if (!thumbnail_) {
    return fallback_thumbnail;
  }
  return thumbnail_;
}

void FileThumbnail::InvalidateThumbnail() {
  thumbnail_.reset();
  thumbnail_future_ = {};
  thumbnail_source_write_time_initialized_ = false;
  thumbnail_asset_reload_required_ = false;
}

void FileThumbnail::SyncThumbnailSourceWriteTime(const File& file) {
  std::error_code error_code;
  const auto source_write_time = std::filesystem::last_write_time(file.GetAbsolutePath(), error_code);
  if (error_code) {
    return;
  }

  if (!thumbnail_source_write_time_initialized_) {
    thumbnail_source_write_time_ = source_write_time;
    thumbnail_source_write_time_initialized_ = true;
    return;
  }
  if (thumbnail_source_write_time_ == source_write_time) {
    return;
  }

  thumbnail_source_write_time_ = source_write_time;
  thumbnail_.reset();
  thumbnail_future_ = {};
  thumbnail_asset_reload_required_ = true;
}

std::shared_ptr<Texture2D> FileThumbnail::GetFallbackThumbnail(const File& file) const {
  if (const auto icon = EditorLayer::FindIcon(file.GetAssetTypeName())) {
    return icon;
  }
  return EditorLayer::FindIcon("Binary");
}

}  // namespace

std::shared_ptr<Texture2D> AssetThumbnailProvider::GetFileThumbnail(const std::shared_ptr<File>& file,
                                                                    const bool allow_asset_load) {
  if (!file)
    return {};
  auto& cache = FileThumbnails();
  for (auto it = cache.begin(); it != cache.end();) {
    if (it->first.expired())
      it = cache.erase(it);
    else
      ++it;
  }
  return cache[file].GetThumbnail(*file, allow_asset_load);
}
void AssetThumbnailProvider::ClearFileThumbnails() {
  const auto it = FileThumbnailCaches().find(ApplicationContext::TryGet());
  if (it != FileThumbnailCaches().end())
    it->second.clear();
}
