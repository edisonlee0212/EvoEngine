#include "AssetThumbnailProvider.hpp"

#include "FileManager.hpp"
#include "IAsset.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "OffscreenPreviewRenderer.hpp"
#include "Prefab.hpp"
#include "ProjectManager.hpp"
#include "Scene.hpp"
#include "Serialization.hpp"
#include "Texture2D.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>

using namespace evo_engine;

namespace {
constexpr uint32_t kThumbnailCacheVersion = 3;
constexpr uint64_t kThumbnailCacheLimit = 512ull * 1024ull * 1024ull;
constexpr glm::uvec2 kProjectThumbnailResolution = {256, 256};

uint64_t HashBytes(uint64_t hash, const void* data, const size_t size) {
  constexpr uint64_t prime = 1099511628211ull;
  const auto* bytes = static_cast<const unsigned char*>(data);
  for (size_t i = 0; i < size; ++i) {
    hash = (hash ^ bytes[i]) * prime;
  }
  return hash;
}

template <typename T>
uint64_t HashValue(const uint64_t hash, const T& value) {
  return HashBytes(hash, &value, sizeof(value));
}

uint64_t HashString(const uint64_t hash, const std::string& value) {
  return HashBytes(hash, value.data(), value.size());
}

uint64_t HashFile(uint64_t hash, const Handle handle) {
  hash = HashValue(hash, handle.GetValue());
  const auto file = FileManager::GetFile(handle);
  if (!file) {
    return hash;
  }
  const auto path = file->GetAbsolutePath();
  hash = HashString(hash, path.generic_string());
  std::error_code error_code;
  const auto write_time = std::filesystem::last_write_time(path, error_code);
  if (!error_code) {
    hash = HashValue(hash, write_time.time_since_epoch().count());
  }
  const auto file_size = std::filesystem::file_size(path, error_code);
  if (!error_code) {
    hash = HashValue(hash, file_size);
  }
  return hash;
}

bool UsesPersistentCache(const std::shared_ptr<IAsset>& asset, const OffscreenPreviewSettings& settings) {
  if (!asset || asset->IsTemporary() || settings.resolution != kProjectThumbnailResolution ||
      settings.subject_rotation != glm::vec2(0.0f) || settings.camera_zoom != 1.0f) {
    return false;
  }
  const auto& type_name = asset->GetTypeName();
  return type_name == "Material" || type_name == "Mesh";
}

std::filesystem::path ThumbnailCacheDirectory() {
  const auto project_folder = ProjectManager::GetProjectFolderPath();
  return project_folder.empty() ? std::filesystem::path{} : project_folder / "Cache/Thumbnail";
}

uint64_t ThumbnailFingerprint(const std::shared_ptr<IAsset>& asset, const OffscreenPreviewSettings& settings) {
  uint64_t hash = 14695981039346656037ull;
  hash = HashValue(hash, kThumbnailCacheVersion);
  hash = HashString(hash, asset->GetTypeName());
  hash = HashValue(hash, settings.resolution.x);
  hash = HashValue(hash, settings.resolution.y);
  hash = HashValue(hash, settings.camera_distance_multiplier);
  hash = HashValue(hash, settings.clear_color.r);
  hash = HashValue(hash, settings.clear_color.g);
  hash = HashValue(hash, settings.clear_color.b);
  hash = HashFile(hash, asset->GetHandle());
  if (const auto material = std::dynamic_pointer_cast<Material>(asset)) {
    for (const auto& texture_ref : material->PeekTextureRefs()) {
      hash = HashFile(hash, texture_ref.GetAssetHandle());
    }
  }
  return hash;
}

std::filesystem::path ThumbnailCachePath(const std::shared_ptr<IAsset>& asset,
                                         const OffscreenPreviewSettings& settings) {
  std::ostringstream file_name;
  file_name << 'v' << kThumbnailCacheVersion << '-' << asset->GetHandle().GetValue() << '-' << std::hex
            << ThumbnailFingerprint(asset, settings) << ".png";
  return ThumbnailCacheDirectory() / file_name.str();
}

void PruneThumbnailCache(const std::filesystem::path& cache_directory) {
  struct CacheEntry {
    std::filesystem::path path;
    std::filesystem::file_time_type access_time;
    uint64_t size = 0;
  };
  std::vector<CacheEntry> entries;
  uint64_t total_size = 0;
  std::error_code error_code;
  for (const auto& entry : std::filesystem::directory_iterator(cache_directory, error_code)) {
    if (error_code || !entry.is_regular_file()) {
      continue;
    }
    const auto file_name = entry.path().filename().string();
    const auto first_dash = file_name.find('-');
    const auto second_dash = first_dash == std::string::npos ? std::string::npos : file_name.find('-', first_dash + 1);
    const auto version_prefix = "v" + std::to_string(kThumbnailCacheVersion) + "-";
    const bool current_version = file_name.compare(0, version_prefix.size(), version_prefix) == 0;
    uint64_t asset_handle = 0;
    if (second_dash != std::string::npos) {
      const auto handle_text = file_name.substr(first_dash + 1, second_dash - first_dash - 1);
      std::istringstream(handle_text) >> asset_handle;
    }
    if (!current_version || asset_handle == 0 || !FileManager::GetFile(Handle(asset_handle))) {
      std::filesystem::remove(entry.path(), error_code);
      error_code.clear();
      continue;
    }
    const auto size = entry.file_size(error_code);
    if (error_code) {
      error_code.clear();
      continue;
    }
    entries.push_back({entry.path(), entry.last_write_time(error_code), size});
    error_code.clear();
    total_size += size;
  }
  if (total_size <= kThumbnailCacheLimit) {
    return;
  }
  std::sort(entries.begin(), entries.end(), [](const CacheEntry& left, const CacheEntry& right) {
    return left.access_time < right.access_time;
  });
  for (const auto& entry : entries) {
    std::filesystem::remove(entry.path, error_code);
    error_code.clear();
    total_size -= std::min(total_size, entry.size);
    if (total_size <= kThumbnailCacheLimit) {
      break;
    }
  }
}

void EnsureThumbnailCache() {
  static std::filesystem::path cleaned_project;
  const auto cache_directory = ThumbnailCacheDirectory();
  if (cache_directory.empty() || cleaned_project == cache_directory) {
    return;
  }
  std::error_code error_code;
  std::filesystem::create_directories(cache_directory, error_code);
  if (!error_code) {
    PruneThumbnailCache(cache_directory);
    cleaned_project = cache_directory;
  }
}

std::shared_ptr<Texture2D> LoadCachedThumbnail(const std::shared_ptr<IAsset>& asset,
                                               const OffscreenPreviewSettings& settings) {
  if (!UsesPersistentCache(asset, settings)) {
    return {};
  }
  EnsureThumbnailCache();
  const auto cache_path = ThumbnailCachePath(asset, settings);
  if (!std::filesystem::is_regular_file(cache_path)) {
    return {};
  }
  const auto texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  if (!texture || !texture->Import(cache_path)) {
    std::error_code error_code;
    std::filesystem::remove(cache_path, error_code);
    return {};
  }
  std::error_code error_code;
  std::filesystem::last_write_time(cache_path, std::filesystem::file_time_type::clock::now(), error_code);
  return texture;
}

void StoreCachedThumbnail(const std::shared_ptr<IAsset>& asset, const OffscreenPreviewSettings& settings,
                          const std::shared_ptr<Texture2D>& thumbnail) {
  if (!thumbnail || !UsesPersistentCache(asset, settings)) {
    return;
  }
  EnsureThumbnailCache();
  const auto cache_path = ThumbnailCachePath(asset, settings);
  std::error_code error_code;
  std::filesystem::create_directories(cache_path.parent_path(), error_code);
  if (error_code) {
    return;
  }
  auto temporary_path = cache_path;
  temporary_path += ".tmp";
  thumbnail->StoreToPng(temporary_path, -1, -1, 4);
  if (!std::filesystem::is_regular_file(temporary_path)) {
    return;
  }
  std::filesystem::remove(cache_path, error_code);
  error_code.clear();
  std::filesystem::rename(temporary_path, cache_path, error_code);
  if (error_code) {
    std::filesystem::remove(temporary_path, error_code);
    return;
  }
  const auto prefix =
      "v" + std::to_string(kThumbnailCacheVersion) + "-" + std::to_string(asset->GetHandle().GetValue()) + "-";
  for (const auto& entry : std::filesystem::directory_iterator(cache_path.parent_path(), error_code)) {
    const auto file_name = entry.path().filename().string();
    if (!error_code && entry.path() != cache_path && file_name.compare(0, prefix.size(), prefix) == 0) {
      std::filesystem::remove(entry.path(), error_code);
      error_code.clear();
    }
  }
  PruneThumbnailCache(cache_path.parent_path());
}

void EnsureDefaultPreviewHandlers() {
  Serialization::RegisterAssetPreviewHandler<Texture2D>(
      [](const std::shared_ptr<Texture2D>& texture, const OffscreenPreviewSettings&) {
        return texture ? texture->GenerateThumbnailTexture() : nullptr;
      },
      {}, "Texture2D");
  Serialization::RegisterAssetPreviewHandler<Material>(
      [](const std::shared_ptr<Material>& material, const OffscreenPreviewSettings& settings) {
        return OffscreenPreviewRenderer::RenderMaterial(material, settings);
      },
      {}, "Material");
  Serialization::RegisterAssetPreviewHandler<Mesh>(
      [](const std::shared_ptr<Mesh>& mesh, const OffscreenPreviewSettings& settings) {
        return OffscreenPreviewRenderer::RenderMesh(mesh, {}, settings);
      },
      {}, "Mesh");
  Serialization::RegisterAssetPreviewHandler<Prefab>(
      [](const std::shared_ptr<Prefab>& prefab, const OffscreenPreviewSettings&) {
        return prefab ? prefab->GenerateThumbnailTexture() : nullptr;
      },
      {}, "Prefab");
  Serialization::RegisterAssetPreviewHandler<Scene>(
      [](const std::shared_ptr<Scene>& scene, const OffscreenPreviewSettings&) {
        return scene ? scene->GenerateThumbnailTexture() : nullptr;
      },
      {}, "Scene");
}
}  // namespace

bool AssetThumbnailProvider::SupportsGeneratedThumbnail(const std::string& asset_type_name) {
  EnsureDefaultPreviewHandlers();
  return Serialization::HasAssetPreviewHandler(asset_type_name) ||
         (Serialization::HasAssetType(asset_type_name) && Serialization::HasAssetPreviewHandler<IAsset>());
}

std::shared_ptr<Texture2D> AssetThumbnailProvider::GenerateThumbnail(const std::shared_ptr<IAsset>& asset,
                                                                     const OffscreenPreviewSettings& settings) {
  EnsureDefaultPreviewHandlers();
  if (!asset || !SupportsGeneratedThumbnail(asset->GetTypeName())) {
    return {};
  }
  if (const auto cached = LoadCachedThumbnail(asset, settings)) {
    return cached;
  }
  const auto thumbnail = Serialization::GenerateAssetThumbnail(asset, settings);
  StoreCachedThumbnail(asset, settings, thumbnail);
  return thumbnail;
}
