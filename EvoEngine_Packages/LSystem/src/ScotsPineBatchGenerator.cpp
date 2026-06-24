#include "PackageManager.hpp"

#include "Application.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "LSystemLayer.hpp"
#include "LSystemRuleHelpers.hpp"
#include "Lights.hpp"
#include "Platform.hpp"
#include "PostProcessingStack.hpp"
#include "ProjectManager.hpp"
#include "Scene.hpp"
#include "ScotsPine.hpp"
#include "ScotsPineBatchContract.hpp"
#include "ScotsPineDescriptor.hpp"
#include "ScotsPineTemporalGrowth.hpp"
#include "Transform.hpp"

#include <array>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <future>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

using namespace evo_engine;
using namespace l_system_package;

namespace {

std::string Trim(std::string value) {
  auto is_space = [](const unsigned char c) {
    return std::isspace(c) != 0;
  };
  while (!value.empty() && is_space(static_cast<unsigned char>(value.front()))) {
    value.erase(value.begin());
  }
  while (!value.empty() && is_space(static_cast<unsigned char>(value.back()))) {
    value.pop_back();
  }
  return value;
}

bool TryParseInt(const std::string& value, int& out_value) {
  try {
    size_t consumed = 0;
    const int parsed = std::stoi(value, &consumed);
    if (consumed != value.size()) {
      return false;
    }
    out_value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

bool TryParseBool(const std::string& value, bool& out_value) {
  std::string normalized = value;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  if (normalized == "1" || normalized == "true" || normalized == "yes" || normalized == "on") {
    out_value = true;
    return true;
  }
  if (normalized == "0" || normalized == "false" || normalized == "no" || normalized == "off") {
    out_value = false;
    return true;
  }
  return false;
}

std::filesystem::path FindResourceFolder() {
  std::filesystem::path resource_folder_path("../../../../../Resources");
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../Resources";
  }
  return resource_folder_path;
}

std::filesystem::path ResolveDefaultScotsPineProjectPath(const std::filesystem::path& resource_folder_path) {
  return std::filesystem::absolute(resource_folder_path / "DigitalAgricultureProject" / "test.eveproj");
}

std::shared_ptr<Scene> ResolveSceneAsset(const std::filesystem::path& scene_path, std::string& error) {
  std::shared_ptr<IAsset> scene_asset;
  if (scene_path.is_absolute()) {
    const auto absolute_scene_path = std::filesystem::absolute(scene_path);
    if (!ProjectManager::IsInAssetsFolder(absolute_scene_path)) {
      error = "Scene absolute path is outside the project's Assets folder: " + absolute_scene_path.string();
      return nullptr;
    }
    scene_asset = ProjectManager::GetOrCreateAsset(ProjectManager::GetAssetsRelativePath(absolute_scene_path));
  } else {
    scene_asset = ProjectManager::GetOrCreateAsset(scene_path);
  }

  if (!scene_asset) {
    error = "Scene asset could not be loaded: " + scene_path.string();
    return nullptr;
  }

  const auto scene = std::dynamic_pointer_cast<Scene>(scene_asset);
  if (!scene) {
    error = "Asset is not a Scene: " + scene_path.string();
    return nullptr;
  }

  return scene;
}

std::shared_ptr<ScotsPineDescriptor> ResolveDescriptorAsset(const std::filesystem::path& descriptor_path,
                                                            std::string& error) {
  std::shared_ptr<IAsset> descriptor_asset;
  if (descriptor_path.is_absolute()) {
    const auto absolute_descriptor_path = std::filesystem::absolute(descriptor_path);
    if (!ProjectManager::IsInAssetsFolder(absolute_descriptor_path)) {
      error = "Descriptor absolute path is outside the project's Assets folder: " + absolute_descriptor_path.string();
      return nullptr;
    }
    descriptor_asset =
        ProjectManager::GetOrCreateAsset(ProjectManager::GetAssetsRelativePath(absolute_descriptor_path));
  } else {
    descriptor_asset = ProjectManager::GetOrCreateAsset(descriptor_path);
  }

  if (!descriptor_asset) {
    error = "Descriptor asset could not be loaded: " + descriptor_path.string();
    return nullptr;
  }

  const auto descriptor = std::dynamic_pointer_cast<ScotsPineDescriptor>(descriptor_asset);
  if (!descriptor) {
    error = "Descriptor is not a ScotsPineDescriptor: " + descriptor_path.string();
    return nullptr;
  }

  return descriptor;
}

using SyntheticOptions = ScotsPineSyntheticRenderOptions;

std::string JsonEscapeSynthetic(const std::string& value);

std::filesystem::path ResolveProjectAssetFilesystemPath(const std::filesystem::path& asset_path) {
  if (asset_path.empty()) {
    return {};
  }
  if (asset_path.is_absolute()) {
    return std::filesystem::absolute(asset_path);
  }
  return std::filesystem::absolute(ProjectManager::GetAssetsFolderPath() / asset_path);
}

std::string Fnv1aFileHashHex(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    return {};
  }

  std::uint64_t hash = 1469598103934665603ull;
  char buffer[4096];
  while (in.good()) {
    in.read(buffer, sizeof(buffer));
    const std::streamsize count = in.gcount();
    for (std::streamsize i = 0; i < count; ++i) {
      hash ^= static_cast<unsigned char>(buffer[i]);
      hash *= 1099511628211ull;
    }
  }

  std::ostringstream ss;
  ss << std::hex << std::setw(16) << std::setfill('0') << hash;
  return ss.str();
}

std::vector<std::string> BuildSyntheticOverrideList(const SyntheticOptions& options) {
  std::vector<std::string> overrides;
  if (options.override_max_target_gdd) {
    overrides.emplace_back("max_target_gdd");
  }
  if (options.override_scene_pine_target_gdd_multiplier) {
    overrides.emplace_back("scene_pine_target_gdd_multiplier");
  }
  if (options.override_annual_whorl_probability) {
    overrides.emplace_back("annual_whorl_probability");
  }
  if (options.override_whorl_position_norm) {
    overrides.emplace_back("whorl_position_norm");
  }
  if (options.override_branches_per_whorl) {
    overrides.emplace_back("branches_per_whorl");
  }
  if (options.override_whorl_dormancy_years) {
    overrides.emplace_back("whorl_dormancy_years");
  }
  if (options.override_max_branching_order) {
    overrides.emplace_back("max_branching_order");
  }
  if (options.override_young_needle_palette_rgba) {
    overrides.emplace_back("young_needle_palette_rgba");
  }
  if (options.override_older_needle_palette_rgba) {
    overrides.emplace_back("older_needle_palette_rgba");
  }
  if (options.override_dry_brown_needle_palette_rgba) {
    overrides.emplace_back("dry_brown_needle_palette_rgba");
  }
  if (options.override_main_stem_palette_rgba) {
    overrides.emplace_back("main_stem_palette_rgba");
  }
  if (options.override_mature_bark_stem_palette_rgba) {
    overrides.emplace_back("mature_bark_stem_palette_rgba");
  }
  if (options.override_node_sheath_brown_palette_rgba) {
    overrides.emplace_back("node_sheath_brown_palette_rgba");
  }
  if (options.override_fascicle_sheath_palette_rgba) {
    overrides.emplace_back("fascicle_sheath_palette_rgba");
  }
  if (options.override_needle_tip_color_mix_start) {
    overrides.emplace_back("needle_tip_color_mix_start");
  }
  if (options.override_needle_tip_color_exponent) {
    overrides.emplace_back("needle_tip_color_exponent");
  }
  if (options.override_needle_old_thinning_fraction) {
    overrides.emplace_back("needle_old_thinning_fraction");
  }
  if (options.override_needle_min_strand_thickness_m) {
    overrides.emplace_back("needle_min_strand_thickness_m");
  }
  if (options.override_needle_micro_variation) {
    overrides.emplace_back("needle_micro_variation");
  }
  if (options.override_stem_micro_variation) {
    overrides.emplace_back("stem_micro_variation");
  }
  if (options.override_young_needle_roughness) {
    overrides.emplace_back("young_needle_roughness");
  }
  if (options.override_old_needle_roughness) {
    overrides.emplace_back("old_needle_roughness");
  }
  if (options.override_young_needle_specular) {
    overrides.emplace_back("young_needle_specular");
  }
  if (options.override_old_needle_specular) {
    overrides.emplace_back("old_needle_specular");
  }
  if (options.override_stem_roughness) {
    overrides.emplace_back("stem_roughness");
  }
  if (options.override_stem_specular) {
    overrides.emplace_back("stem_specular");
  }
  if (options.override_node_browning_strength) {
    overrides.emplace_back("node_browning_strength");
  }
  if (options.override_sheath_browning_strength) {
    overrides.emplace_back("sheath_browning_strength");
  }
  if (options.override_node_browning_radius_norm) {
    overrides.emplace_back("node_browning_radius_norm");
  }
  if (options.override_needle_twist_turns) {
    overrides.emplace_back("needle_twist_turns");
  }
  if (options.override_needle_edge_darkening) {
    overrides.emplace_back("needle_edge_darkening");
  }
  if (options.override_needle_segment_count) {
    overrides.emplace_back("needle_segment_count");
  }
  if (options.override_fascicle_sheath_length_m) {
    overrides.emplace_back("fascicle_sheath_length_m");
  }
  if (options.override_fascicle_sheath_width_m) {
    overrides.emplace_back("fascicle_sheath_width_m");
  }
  if (options.override_needle_year0_length_multiplier) {
    overrides.emplace_back("needle_year0_length_multiplier");
  }
  if (options.override_needle_axial_age_span) {
    overrides.emplace_back("needle_axial_age_span");
  }
  if (options.override_needle_axial_age_exponent) {
    overrides.emplace_back("needle_axial_age_exponent");
  }
  return overrides;
}

bool HasSyntheticDescriptorOverrides(const SyntheticOptions& options) {
  const auto overrides = BuildSyntheticOverrideList(options);
  return std::any_of(overrides.begin(), overrides.end(), [](const std::string& key) {
    return key != "max_target_gdd" && key != "scene_pine_target_gdd_multiplier";
  });
}

struct SyntheticCameraRigView {
  std::string label{"default"};
  glm::vec3 position{0.0f, 0.8f, 7.5f};
  glm::vec3 forward{0.0f, 0.0f, -1.0f};
  glm::vec3 up{0.0f, 1.0f, 0.0f};
  float fx = 0.0f;
  float fy = 0.0f;
  float cx = 0.0f;
  float cy = 0.0f;
  int width = 1372;
  int height = 1040;
  float near_distance = 0.1f;
  float far_distance = 200.0f;
  float fov_deg = 60.0f;
};

struct SyntheticProfilePhaseDurations {
  double setup_ms = 0.0;
  double scene_load_or_clone_ms = 0.0;
  double scene_reset_ms = 0.0;
  double descriptor_load_ms = 0.0;
  double growth_ms = 0.0;
  double graph_export_ms = 0.0;
  double view_setup_ms = 0.0;
  double rgb_render_ms = 0.0;
  double gpu_readback_ms = 0.0;
  double composite_ms = 0.0;
  double encode_ms = 0.0;
  double write_ms = 0.0;
  double rgb_write_ms = 0.0;
  double depth_render_write_ms = 0.0;
  double instance_mask_render_write_ms = 0.0;
  double label_render_write_ms = 0.0;
  double total_engine_ms = 0.0;
};

struct SyntheticProfileDatapoint {
  int datapoint_index = 0;
  int global_sample_index = 0;
  int local_sample_index = 0;
  int frame_index = 0;
  int view_index = 0;
  std::string view_label{};
  std::array<std::uint32_t, 3> seeds{};
  int width = 0;
  int height = 0;
  bool warmup = false;
  bool success = true;
  std::string error_message{};
  std::filesystem::path rgb_path{};
  std::filesystem::path foreground_mask_path{};
  std::filesystem::path raw_rgba_path{};
  std::filesystem::path annotation_overlay_path{};
  std::filesystem::path background_path{};
  std::uintmax_t bytes_written = 0;
  SyntheticProfilePhaseDurations phases{};
};

struct SyntheticEncodedImage {
  std::vector<unsigned char> bytes{};
};

struct SyntheticImageWriteTask {
  int width = 0;
  int height = 0;
  std::vector<uint8_t> rgba{};
  std::filesystem::path rgb_path{};
  std::filesystem::path foreground_mask_path{};
  std::filesystem::path raw_rgba_path{};
  std::filesystem::path annotation_overlay_path{};
  std::filesystem::path background_path{};
  std::vector<glm::ivec2> annotation_overlay_pixels{};
  std::string rgb_output_format{"jpg"};
  int jpg_quality = 95;
  int png_compression_level = 1;
  float plant_blur_radius_px = 0.0f;
  bool composite_background = true;
  bool write_foreground_mask = true;
  bool write_raw_rgba = false;
  bool write_annotation_overlay = true;
};

struct SyntheticImageWriteResult {
  bool success = true;
  std::string error_message{};
  double composite_ms = 0.0;
  double encode_ms = 0.0;
  double write_ms = 0.0;
  std::uintmax_t bytes_written = 0;
};

std::mutex& SyntheticStbiMutex() {
  static std::mutex mutex;
  return mutex;
}

double SyntheticElapsedMs(const std::chrono::steady_clock::time_point start_time) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time).count();
}

constexpr int kAnnotationNeedleSampleCount = 64;
constexpr int kAnnotationSheathSampleCount = 3;
constexpr int kAnnotationStemSampleCount = 64;

std::array<std::uint32_t, 3> SyntheticSeedsForSample(const SyntheticOptions& options, const int global_sample_index) {
  if (options.sample_count == 1 && options.start_index == 0) {
    return {options.seed_a, options.seed_b, options.seed_c};
  }
  const std::uint64_t seed_base = static_cast<std::uint64_t>(options.base_seed) +
                                  static_cast<std::uint64_t>(std::max(0, global_sample_index)) * 3ull;
  return {static_cast<std::uint32_t>((seed_base + 0ull) & 0xffffffffull),
          static_cast<std::uint32_t>((seed_base + 1ull) & 0xffffffffull),
          static_cast<std::uint32_t>((seed_base + 2ull) & 0xffffffffull)};
}

void StbiVectorWriteCallback(void* context, void* data, int size) {
  if (!context || !data || size <= 0) {
    return;
  }
  auto& out = *static_cast<SyntheticEncodedImage*>(context);
  const auto* bytes = static_cast<const unsigned char*>(data);
  out.bytes.insert(out.bytes.end(), bytes, bytes + size);
}

SyntheticEncodedImage EncodeSyntheticJpg(const std::vector<uint8_t>& rgb, const int width, const int height,
                                         const int quality) {
  SyntheticEncodedImage encoded;
  std::lock_guard<std::mutex> lock(SyntheticStbiMutex());
  stbi_flip_vertically_on_write(false);
  stbi_write_jpg_to_func(StbiVectorWriteCallback, &encoded, width, height, 3, rgb.data(),
                         std::clamp(quality, 1, 100));
  return encoded;
}

SyntheticEncodedImage EncodeSyntheticPng(const std::vector<uint8_t>& pixels, const int width, const int height,
                                         const int channels, const int compression_level) {
  SyntheticEncodedImage encoded;
  (void)compression_level;
  std::lock_guard<std::mutex> lock(SyntheticStbiMutex());
  stbi_flip_vertically_on_write(false);
  stbi_write_png_to_func(StbiVectorWriteCallback, &encoded, width, height, channels, pixels.data(), width * channels);
  return encoded;
}

void DownloadSyntheticColorRgba8(const std::shared_ptr<RenderTexture>& render_texture, std::vector<uint8_t>& pixels,
                                 int& width, int& height, const bool flip_vertically) {
  if (!render_texture || !render_texture->GetColorImage()) {
    throw std::runtime_error("Render texture color image is unavailable.");
  }
  const auto& color_image = render_texture->GetColorImage();
  width = static_cast<int>(color_image->GetExtent().width);
  height = static_cast<int>(color_image->GetExtent().height);
  const size_t pixel_count = static_cast<size_t>(width) * static_cast<size_t>(height);

  std::vector<float> rgba(pixel_count * 4);
  Buffer image_buffer(sizeof(glm::vec4) * pixel_count);
  image_buffer.CopyFromImage(*color_image);
  image_buffer.DownloadVector(rgba, pixel_count * 4);

  pixels.resize(pixel_count * 4);
  for (int y = 0; y < height; ++y) {
    const int src_y = flip_vertically ? height - 1 - y : y;
    for (int x = 0; x < width; ++x) {
      const size_t dst = static_cast<size_t>(y * width + x) * 4;
      const size_t src = static_cast<size_t>(src_y * width + x) * 4;
      pixels[dst] = glm::clamp<int>(static_cast<int>(255.9f * rgba[src]), 0, 255);
      pixels[dst + 1] = glm::clamp<int>(static_cast<int>(255.9f * rgba[src + 1]), 0, 255);
      pixels[dst + 2] = glm::clamp<int>(static_cast<int>(255.9f * rgba[src + 2]), 0, 255);
      pixels[dst + 3] = glm::clamp<int>(static_cast<int>(255.9f * rgba[src + 3]), 0, 255);
    }
  }
}

std::vector<uint8_t> QuantizeSyntheticLabelRgb(const std::vector<uint8_t>& rgba, const int width, const int height) {
  constexpr std::array<std::array<uint8_t, 3>, 6> kPalette{{
      {{255, 0, 0}},
      {{0, 255, 0}},
      {{0, 0, 255}},
      {{255, 255, 0}},
      {{255, 0, 255}},
      {{0, 255, 255}},
  }};

  std::vector<uint8_t> rgb(static_cast<size_t>(width) * static_cast<size_t>(height) * 3);
  for (int i = 0; i < width * height; ++i) {
    const size_t rgba_index = static_cast<size_t>(i) * 4;
    const size_t rgb_index = static_cast<size_t>(i) * 3;
    const float r = static_cast<float>(rgba[rgba_index]);
    const float g = static_cast<float>(rgba[rgba_index + 1]);
    const float b = static_cast<float>(rgba[rgba_index + 2]);
    const float max_channel = std::max(r, std::max(g, b));
    if (rgba[rgba_index + 3] == 0 || max_channel <= 2.0f) {
      continue;
    }

    const std::array<float, 3> normalized{r / max_channel, g / max_channel, b / max_channel};
    const auto* best = &kPalette.front();
    float best_distance = std::numeric_limits<float>::max();
    for (const auto& color : kPalette) {
      const float pr = static_cast<float>(color[0]) / 255.0f;
      const float pg = static_cast<float>(color[1]) / 255.0f;
      const float pb = static_cast<float>(color[2]) / 255.0f;
      const float distance = (normalized[0] - pr) * (normalized[0] - pr) +
                             (normalized[1] - pg) * (normalized[1] - pg) +
                             (normalized[2] - pb) * (normalized[2] - pb);
      if (distance < best_distance) {
        best_distance = distance;
        best = &color;
      }
    }

    rgb[rgb_index] = (*best)[0];
    rgb[rgb_index + 1] = (*best)[1];
    rgb[rgb_index + 2] = (*best)[2];
  }
  return rgb;
}

std::vector<uint8_t> BuildSyntheticForegroundMaskFromLabelRgb(const std::vector<uint8_t>& label_rgb, const int width,
                                                              const int height) {
  std::vector<uint8_t> mask(static_cast<size_t>(width) * static_cast<size_t>(height));
  for (int i = 0; i < width * height; ++i) {
    const size_t rgb_index = static_cast<size_t>(i) * 3;
    if (label_rgb[rgb_index] != 0 || label_rgb[rgb_index + 1] != 0 || label_rgb[rgb_index + 2] != 0) {
      mask[static_cast<size_t>(i)] = 255;
    }
  }
  return mask;
}

void WriteSyntheticBinaryFile(const std::filesystem::path& path, const SyntheticEncodedImage& image) {
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream out(path.string(), std::ios::binary | std::ios::trunc);
  if (!out.is_open()) {
    throw std::runtime_error("Failed to open output image: " + path.string());
  }
  out.write(reinterpret_cast<const char*>(image.bytes.data()), static_cast<std::streamsize>(image.bytes.size()));
  if (!out.good()) {
    throw std::runtime_error("Failed to write output image: " + path.string());
  }
}

std::vector<uint8_t> LoadSyntheticBackgroundRgb(const std::filesystem::path& path, const int width, const int height) {
  int source_width = 0;
  int source_height = 0;
  int source_channels = 0;
  uint8_t* loaded = nullptr;
  {
    std::lock_guard<std::mutex> lock(SyntheticStbiMutex());
    stbi_set_flip_vertically_on_load(false);
    loaded = stbi_load(path.string().c_str(), &source_width, &source_height, &source_channels, 3);
  }
  if (!loaded) {
    throw std::runtime_error("Failed to load compositing background: " + path.string());
  }

  std::vector<uint8_t> background(static_cast<size_t>(width) * height * 3);
  if (source_width == width && source_height == height) {
    std::memcpy(background.data(), loaded, background.size());
  } else {
    for (int y = 0; y < height; ++y) {
      const int source_y = std::clamp(static_cast<int>((static_cast<int64_t>(y) * source_height) / height), 0,
                                      std::max(0, source_height - 1));
      for (int x = 0; x < width; ++x) {
        const int source_x = std::clamp(static_cast<int>((static_cast<int64_t>(x) * source_width) / width), 0,
                                        std::max(0, source_width - 1));
        const size_t source_index = (static_cast<size_t>(source_y) * source_width + source_x) * 3;
        const size_t target_index = (static_cast<size_t>(y) * width + x) * 3;
        background[target_index] = loaded[source_index];
        background[target_index + 1] = loaded[source_index + 1];
        background[target_index + 2] = loaded[source_index + 2];
      }
    }
  }
  stbi_image_free(loaded);
  return background;
}

bool SyntheticRgbaHasUsableAlpha(const std::vector<uint8_t>& rgba) {
  if (rgba.size() < 4) {
    return false;
  }
  uint8_t min_alpha = 255;
  uint8_t max_alpha = 0;
  for (size_t i = 3; i < rgba.size(); i += 4) {
    min_alpha = std::min(min_alpha, rgba[i]);
    max_alpha = std::max(max_alpha, rgba[i]);
  }
  return !(min_alpha == max_alpha && (max_alpha == 0 || max_alpha == 255));
}

uint8_t SyntheticPlantAlpha(const std::vector<uint8_t>& rgba, const size_t rgba_index, const bool use_alpha) {
  if (use_alpha) {
    return rgba[rgba_index + 3];
  }
  return rgba[rgba_index] > 2 || rgba[rgba_index + 1] > 2 || rgba[rgba_index + 2] > 2 ? 255 : 0;
}

std::vector<float> BuildSyntheticGaussianKernel(const float radius_px) {
  const float sigma = std::max(0.1f, radius_px);
  const int radius = std::max(1, static_cast<int>(std::ceil(sigma * 3.0f)));
  std::vector<float> kernel(static_cast<size_t>(radius * 2 + 1));
  float sum = 0.0f;
  for (int offset = -radius; offset <= radius; ++offset) {
    const float x = static_cast<float>(offset);
    const float weight = std::exp(-(x * x) / (2.0f * sigma * sigma));
    kernel[static_cast<size_t>(offset + radius)] = weight;
    sum += weight;
  }
  for (float& weight : kernel) {
    weight /= sum;
  }
  return kernel;
}

std::vector<float> BlurSyntheticPlantLayer(const std::vector<uint8_t>& rgba, const int width, const int height,
                                           const bool use_alpha, const float radius_px) {
  const size_t pixel_count = static_cast<size_t>(width) * static_cast<size_t>(height);
  std::vector<float> source(pixel_count * 4);
  for (size_t i = 0; i < pixel_count; ++i) {
    const size_t rgba_index = i * 4;
    const size_t layer_index = i * 4;
    const float alpha = static_cast<float>(SyntheticPlantAlpha(rgba, rgba_index, use_alpha)) / 255.0f;
    source[layer_index] = static_cast<float>(rgba[rgba_index]) * alpha;
    source[layer_index + 1] = static_cast<float>(rgba[rgba_index + 1]) * alpha;
    source[layer_index + 2] = static_cast<float>(rgba[rgba_index + 2]) * alpha;
    source[layer_index + 3] = alpha;
  }

  const std::vector<float> kernel = BuildSyntheticGaussianKernel(radius_px);
  const int radius = static_cast<int>(kernel.size() / 2);
  std::vector<float> temp(source.size());
  std::vector<float> blurred(source.size());
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const size_t dst = (static_cast<size_t>(y) * width + x) * 4;
      for (int offset = -radius; offset <= radius; ++offset) {
        const int sample_x = std::clamp(x + offset, 0, width - 1);
        const size_t src = (static_cast<size_t>(y) * width + sample_x) * 4;
        const float weight = kernel[static_cast<size_t>(offset + radius)];
        for (int channel = 0; channel < 4; ++channel) {
          temp[dst + channel] += source[src + channel] * weight;
        }
      }
    }
  }
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const size_t dst = (static_cast<size_t>(y) * width + x) * 4;
      for (int offset = -radius; offset <= radius; ++offset) {
        const int sample_y = std::clamp(y + offset, 0, height - 1);
        const size_t src = (static_cast<size_t>(sample_y) * width + x) * 4;
        const float weight = kernel[static_cast<size_t>(offset + radius)];
        for (int channel = 0; channel < 4; ++channel) {
          blurred[dst + channel] += temp[src + channel] * weight;
        }
      }
    }
  }
  return blurred;
}

void DrawSyntheticAnnotationOverlayDots(std::vector<uint8_t>& rgb, const int width, const int height,
                                        const std::vector<glm::ivec2>& pixels) {
  constexpr int kDotRadius = 1;
  if (width <= 0 || height <= 0 || rgb.size() != static_cast<size_t>(width) * height * 3) {
    return;
  }
  for (const auto& pixel : pixels) {
    for (int dy = -kDotRadius; dy <= kDotRadius; ++dy) {
      for (int dx = -kDotRadius; dx <= kDotRadius; ++dx) {
        if (dx * dx + dy * dy > kDotRadius * kDotRadius) {
          continue;
        }
        const int x = pixel.x + dx;
        const int y = pixel.y + dy;
        if (x < 0 || y < 0 || x >= width || y >= height) {
          continue;
        }
        const size_t index = (static_cast<size_t>(y) * width + x) * 3;
        rgb[index] = 255;
        rgb[index + 1] = 0;
        rgb[index + 2] = 0;
      }
    }
  }
}

SyntheticImageWriteResult WriteSyntheticImageOutputs(SyntheticImageWriteTask task) {
  SyntheticImageWriteResult result;
  try {
    const auto composite_start_time = std::chrono::steady_clock::now();
    if (task.width <= 0 || task.height <= 0 ||
        task.rgba.size() != static_cast<size_t>(task.width) * task.height * 4) {
      throw std::runtime_error("Invalid RGBA buffer for synthetic image write.");
    }

    const bool use_alpha = SyntheticRgbaHasUsableAlpha(task.rgba);
    std::vector<uint8_t> mask(static_cast<size_t>(task.width) * task.height);
    std::vector<uint8_t> rgb(static_cast<size_t>(task.width) * task.height * 3);
    std::vector<uint8_t> background;
    if (task.composite_background && !task.background_path.empty()) {
      background = LoadSyntheticBackgroundRgb(task.background_path, task.width, task.height);
    }

    size_t foreground_pixels = 0;
    for (int i = 0; i < task.width * task.height; ++i) {
      const size_t rgba_index = static_cast<size_t>(i) * 4;
      const uint8_t alpha = SyntheticPlantAlpha(task.rgba, rgba_index, use_alpha);
      mask[static_cast<size_t>(i)] = alpha > 0 ? 255 : 0;
      if (alpha > 0) {
        foreground_pixels++;
      }
    }

    const std::vector<float> blurred =
        task.plant_blur_radius_px > 0.0f
            ? BlurSyntheticPlantLayer(task.rgba, task.width, task.height, use_alpha, task.plant_blur_radius_px)
            : std::vector<float>{};
    for (int i = 0; i < task.width * task.height; ++i) {
      const size_t rgba_index = static_cast<size_t>(i) * 4;
      const size_t rgb_index = static_cast<size_t>(i) * 3;
      const size_t layer_index = static_cast<size_t>(i) * 4;
      const float alpha_f =
          blurred.empty() ? static_cast<float>(SyntheticPlantAlpha(task.rgba, rgba_index, use_alpha)) / 255.0f
                          : std::clamp(blurred[layer_index + 3], 0.0f, 1.0f);
      for (int channel = 0; channel < 3; ++channel) {
        const float foreground = blurred.empty() ? static_cast<float>(task.rgba[rgba_index + channel]) * alpha_f
                                                 : blurred[layer_index + channel];
        const float backdrop = background.empty() ? 0.0f : static_cast<float>(background[rgb_index + channel]);
        rgb[rgb_index + channel] =
            glm::clamp<int>(static_cast<int>(foreground + backdrop * (1.0f - alpha_f) + 0.5f), 0, 255);
      }
    }
    result.composite_ms = SyntheticElapsedMs(composite_start_time);

    const auto encode_start_time = std::chrono::steady_clock::now();
    std::optional<SyntheticEncodedImage> mask_image;
    if (task.write_foreground_mask && !task.foreground_mask_path.empty()) {
      mask_image = EncodeSyntheticPng(mask, task.width, task.height, 1, task.png_compression_level);
    }
    std::optional<SyntheticEncodedImage> raw_image;
    if (task.write_raw_rgba && !task.raw_rgba_path.empty()) {
      raw_image = EncodeSyntheticPng(task.rgba, task.width, task.height, 4, task.png_compression_level);
    }
    std::optional<SyntheticEncodedImage> annotation_overlay_image;
    if (task.write_annotation_overlay && !task.annotation_overlay_path.empty()) {
      std::vector<uint8_t> overlay_rgb = rgb;
      DrawSyntheticAnnotationOverlayDots(overlay_rgb, task.width, task.height, task.annotation_overlay_pixels);
      annotation_overlay_image = EncodeSyntheticPng(overlay_rgb, task.width, task.height, 3, task.png_compression_level);
    }
    if (foreground_pixels == 0) {
      result.encode_ms = SyntheticElapsedMs(encode_start_time);
      const auto write_start_time = std::chrono::steady_clock::now();
      if (mask_image) {
        WriteSyntheticBinaryFile(task.foreground_mask_path, *mask_image);
        result.bytes_written += mask_image->bytes.size();
      }
      if (raw_image) {
        WriteSyntheticBinaryFile(task.raw_rgba_path, *raw_image);
        result.bytes_written += raw_image->bytes.size();
      }
      if (annotation_overlay_image) {
        WriteSyntheticBinaryFile(task.annotation_overlay_path, *annotation_overlay_image);
        result.bytes_written += annotation_overlay_image->bytes.size();
      }
      result.write_ms = SyntheticElapsedMs(write_start_time);
      throw std::runtime_error(
          "Synthetic foreground mask is empty; render produced no non-background pixels before compositing.");
    }
    const std::string format = task.rgb_output_format == "png" ? "png" : "jpg";
    SyntheticEncodedImage rgb_image =
        format == "png" ? EncodeSyntheticPng(rgb, task.width, task.height, 3, task.png_compression_level)
                        : EncodeSyntheticJpg(rgb, task.width, task.height, task.jpg_quality);
    result.encode_ms = SyntheticElapsedMs(encode_start_time);

    const auto write_start_time = std::chrono::steady_clock::now();
    WriteSyntheticBinaryFile(task.rgb_path, rgb_image);
    result.bytes_written += rgb_image.bytes.size();
    if (mask_image) {
      WriteSyntheticBinaryFile(task.foreground_mask_path, *mask_image);
      result.bytes_written += mask_image->bytes.size();
    }
    if (raw_image) {
      WriteSyntheticBinaryFile(task.raw_rgba_path, *raw_image);
      result.bytes_written += raw_image->bytes.size();
    }
    if (annotation_overlay_image) {
      WriteSyntheticBinaryFile(task.annotation_overlay_path, *annotation_overlay_image);
      result.bytes_written += annotation_overlay_image->bytes.size();
    }
    result.write_ms = SyntheticElapsedMs(write_start_time);
  } catch (const std::exception& e) {
    result.success = false;
    result.error_message = e.what();
  }
  return result;
}

double SyntheticProfileMean(const std::vector<double>& values) {
  if (values.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (const double value : values) {
    sum += value;
  }
  return sum / static_cast<double>(values.size());
}

double SyntheticProfileStdDev(const std::vector<double>& values, const double mean) {
  if (values.size() < 2) {
    return 0.0;
  }
  double sum_sq = 0.0;
  for (const double value : values) {
    const double delta = value - mean;
    sum_sq += delta * delta;
  }
  return std::sqrt(sum_sq / static_cast<double>(values.size()));
}

std::string SyntheticRenderModeLabel(const SyntheticOptions& options) {
  return options.render_mode == "ray_tracing" ? "ray_tracing" : "rasterization";
}

Camera::CameraRenderMode SyntheticCameraMode(const SyntheticOptions& options) {
  return options.render_mode == "ray_tracing" ? Camera::CameraRenderMode::RayTracing
                                               : Camera::CameraRenderMode::Rasterization;
}

constexpr int kSyntheticRenderLoopFlushCount = 4;

void ApplySyntheticCameraRenderMode(const std::shared_ptr<Camera>& camera, const SyntheticOptions& options) {
  if (!camera) {
    return;
  }
  camera->camera_render_mode = SyntheticCameraMode(options);
  camera->camera_settings.sample_size = static_cast<int>(std::max<std::uint32_t>(1u, options.ray_trace_samples));
  camera->camera_settings.bounce = static_cast<int>(std::max<std::uint32_t>(1u, options.ray_trace_bounces));
  camera->ResetFrameCount();
}

void RenderSyntheticCamera(Application& app, const std::shared_ptr<Camera>& camera, const SyntheticOptions& options) {
  ApplySyntheticCameraRenderMode(camera, options);
  camera->SetRequireRendering(true);
  for (int i = 0; i < kSyntheticRenderLoopFlushCount; ++i) {
    app.Loop();
  }
}

void RenderSyntheticCameraRaster(Application& app, const std::shared_ptr<Camera>& camera) {
  camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
  camera->ResetFrameCount();
  camera->SetRequireRendering(true);
  for (int i = 0; i < kSyntheticRenderLoopFlushCount; ++i) {
    app.Loop();
  }
}

void WriteSyntheticProfileCsv(const std::filesystem::path& path,
                              const std::vector<SyntheticProfileDatapoint>& datapoints) {
  std::ofstream out(path.string(), std::ios::trunc);
  if (!out.is_open()) {
    EVOENGINE_WARNING("Failed to write synthetic profile CSV: " + path.string())
    return;
  }

  out << "datapoint_index,global_sample_index,local_sample_index,frame_index,view_index,view_label,warmup,success,"
         "seed_a,seed_b,seed_c,width,height,total_engine_ms,setup_ms,scene_load_or_clone_ms,scene_reset_ms,"
         "descriptor_load_ms,growth_ms,graph_export_ms,view_setup_ms,rgb_render_ms,gpu_readback_ms,composite_ms,"
         "encode_ms,write_ms,rgb_write_ms,depth_render_write_ms,instance_mask_render_write_ms,label_render_write_ms,"
         "bytes_written,rgb_path,foreground_mask_path,raw_rgba_path,annotation_overlay_path,background_path,error\n";
  out << std::fixed << std::setprecision(4);
  for (const auto& point : datapoints) {
    const auto& p = point.phases;
    out << point.datapoint_index << ',' << point.global_sample_index << ',' << point.local_sample_index << ','
        << point.frame_index << ',' << point.view_index << ",\"" << JsonEscapeSynthetic(point.view_label) << "\","
        << static_cast<int>(point.warmup) << ',' << static_cast<int>(point.success) << ',' << point.seeds[0] << ','
        << point.seeds[1] << ',' << point.seeds[2] << ',' << point.width << ',' << point.height << ','
        << p.total_engine_ms << ',' << p.setup_ms << ',' << p.scene_load_or_clone_ms << ',' << p.scene_reset_ms
        << ',' << p.descriptor_load_ms << ',' << p.growth_ms << ',' << p.graph_export_ms << ','
        << p.view_setup_ms << ',' << p.rgb_render_ms << ',' << p.gpu_readback_ms << ',' << p.composite_ms << ','
        << p.encode_ms << ',' << p.write_ms << ',' << p.rgb_write_ms << ',' << p.depth_render_write_ms << ','
        << p.instance_mask_render_write_ms << ',' << p.label_render_write_ms << ',' << point.bytes_written << ",\""
        << JsonEscapeSynthetic(point.rgb_path.string()) << "\",\""
        << JsonEscapeSynthetic(point.foreground_mask_path.string()) << "\",\""
        << JsonEscapeSynthetic(point.raw_rgba_path.string()) << "\",\""
        << JsonEscapeSynthetic(point.annotation_overlay_path.string()) << "\",\""
        << JsonEscapeSynthetic(point.background_path.string()) << "\",\"" << JsonEscapeSynthetic(point.error_message)
        << "\"\n";
  }
}

void WriteSyntheticProfileJson(const std::filesystem::path& path, const SyntheticOptions& options,
                               const std::vector<SyntheticProfileDatapoint>& datapoints,
                               const double total_engine_ms) {
  std::vector<double> valid_totals;
  valid_totals.reserve(datapoints.size());
  int warmup_count = 0;
  int success_count = 0;
  std::uintmax_t bytes_written = 0;
  for (const auto& point : datapoints) {
    if (point.warmup) {
      warmup_count++;
    }
    if (point.success) {
      success_count++;
    }
    if (point.success && !point.warmup) {
      valid_totals.emplace_back(point.phases.total_engine_ms);
    }
    bytes_written += point.bytes_written;
  }
  const double mean = SyntheticProfileMean(valid_totals);
  const double stddev = SyntheticProfileStdDev(valid_totals, mean);
  const auto minmax = std::minmax_element(valid_totals.begin(), valid_totals.end());
  const double min_value = valid_totals.empty() ? 0.0 : *minmax.first;
  const double max_value = valid_totals.empty() ? 0.0 : *minmax.second;
  const double samples_per_hour = mean > 0.0 ? 3600000.0 / mean : 0.0;

  std::ofstream out(path.string(), std::ios::trunc);
  if (!out.is_open()) {
    EVOENGINE_WARNING("Failed to write synthetic profile JSON: " + path.string())
    return;
  }

  out << std::fixed << std::setprecision(4);
  out << "{\n";
  out << "  \"run\": {\n";
  out << "    \"output_name\": \"" << JsonEscapeSynthetic(options.output_name) << "\",\n";
  out << "    \"output_root\": \"" << JsonEscapeSynthetic(options.output_root.string()) << "\",\n";
  out << "    \"render_mode\": \"" << SyntheticRenderModeLabel(options) << "\",\n";
  out << "    \"ray_trace_samples\": " << options.ray_trace_samples << ",\n";
  out << "    \"ray_trace_bounces\": " << options.ray_trace_bounces << ",\n";
  out << "    \"profile_warmup_datapoints\": " << options.profile_warmup_datapoints << ",\n";
  out << "    \"sample_count\": " << options.sample_count << ",\n";
  out << "    \"start_index\": " << options.start_index << ",\n";
  out << "    \"worker_id\": " << options.worker_id << ",\n";
  out << "    \"worker_count\": " << options.worker_count << ",\n";
  out << "    \"rgb_output_format\": \"" << JsonEscapeSynthetic(options.rgb_output_format) << "\",\n";
  out << "    \"jpg_quality\": " << options.jpg_quality << ",\n";
  out << "    \"png_compression_level\": " << options.png_compression_level << ",\n";
  out << "    \"plant_blur_radius_px\": " << options.plant_blur_radius_px << ",\n";
  out << "    \"export_annotation_skeleton\": " << (options.export_annotation_skeleton ? "true" : "false") << ",\n";
  out << "    \"export_annotation_overlay\": " << (options.export_annotation_overlay ? "true" : "false") << ",\n";
  out << "    \"frame_count\": " << options.frame_count << "\n";
  out << "  },\n";
  const auto overrides = BuildSyntheticOverrideList(options);
  out << "  \"parity\": {\n";
  out << "    \"strict_parity\": " << (options.strict_parity ? "true" : "false") << ",\n";
  out << "    \"project_path\": \"" << JsonEscapeSynthetic(options.project_path.string()) << "\",\n";
  out << "    \"scene_path\": \"" << JsonEscapeSynthetic(options.scene_path.string()) << "\",\n";
  out << "    \"descriptor_path\": \"" << JsonEscapeSynthetic(options.descriptor_path.string()) << "\",\n";
  out << "    \"load_scene\": " << (options.load_scene ? "true" : "false") << ",\n";
  out << "    \"use_scene_pine_transforms\": " << (options.use_scene_pine_transforms ? "true" : "false") << ",\n";
  out << "    \"use_scene_pine_growth\": " << (options.use_scene_pine_growth ? "true" : "false") << ",\n";
  out << "    \"preserve_scene_pine_seed\": " << (options.preserve_scene_pine_seed ? "true" : "false") << ",\n";
  out << "    \"scene_pine_growth_mode\": \"" << JsonEscapeSynthetic(options.scene_pine_growth_mode) << "\",\n";
  out << "    \"applied_overrides\": [";
  for (size_t i = 0; i < overrides.size(); ++i) {
    out << (i == 0 ? "" : ", ") << "\"" << JsonEscapeSynthetic(overrides[i]) << "\"";
  }
  out << "]\n";
  out << "  },\n";
  out << "  \"summary\": {\n";
  out << "    \"datapoints\": " << datapoints.size() << ",\n";
  out << "    \"successful_datapoints\": " << success_count << ",\n";
  out << "    \"warmup_datapoints\": " << warmup_count << ",\n";
  out << "    \"estimated_datapoints\": " << valid_totals.size() << ",\n";
  out << "    \"total_engine_ms\": " << total_engine_ms << ",\n";
  out << "    \"mean_ms\": " << mean << ",\n";
  out << "    \"min_ms\": " << min_value << ",\n";
  out << "    \"max_ms\": " << max_value << ",\n";
  out << "    \"stddev_ms\": " << stddev << ",\n";
  out << "    \"samples_per_hour\": " << samples_per_hour << ",\n";
  out << "    \"bytes_written\": " << bytes_written << ",\n";
  out << "    \"bytes_per_successful_datapoint\": "
      << (success_count > 0 ? static_cast<double>(bytes_written) / static_cast<double>(success_count) : 0.0) << "\n";
  out << "  },\n";
  out << "  \"datapoints\": [\n";
  for (size_t i = 0; i < datapoints.size(); ++i) {
    const auto& point = datapoints[i];
    const auto& p = point.phases;
    out << "    {\n";
    out << "      \"datapoint_index\": " << point.datapoint_index << ",\n";
    out << "      \"global_sample_index\": " << point.global_sample_index << ",\n";
    out << "      \"local_sample_index\": " << point.local_sample_index << ",\n";
    out << "      \"frame_index\": " << point.frame_index << ",\n";
    out << "      \"view_index\": " << point.view_index << ",\n";
    out << "      \"view_label\": \"" << JsonEscapeSynthetic(point.view_label) << "\",\n";
    out << "      \"seeds\": [" << point.seeds[0] << ", " << point.seeds[1] << ", " << point.seeds[2] << "],\n";
    out << "      \"warmup\": " << (point.warmup ? "true" : "false") << ",\n";
    out << "      \"success\": " << (point.success ? "true" : "false") << ",\n";
    out << "      \"resolution\": [" << point.width << ", " << point.height << "],\n";
    out << "      \"rgb_path\": \"" << JsonEscapeSynthetic(point.rgb_path.string()) << "\",\n";
    out << "      \"foreground_mask_path\": \"" << JsonEscapeSynthetic(point.foreground_mask_path.string()) << "\",\n";
    out << "      \"raw_rgba_path\": \"" << JsonEscapeSynthetic(point.raw_rgba_path.string()) << "\",\n";
    out << "      \"annotation_overlay_path\": \"" << JsonEscapeSynthetic(point.annotation_overlay_path.string()) << "\",\n";
    out << "      \"background_path\": \"" << JsonEscapeSynthetic(point.background_path.string()) << "\",\n";
    out << "      \"bytes_written\": " << point.bytes_written << ",\n";
    out << "      \"phases_ms\": {";
    out << "\"setup\": " << p.setup_ms << ", \"scene_load_or_clone\": " << p.scene_load_or_clone_ms
        << ", \"scene_reset\": " << p.scene_reset_ms << ", \"descriptor_load\": " << p.descriptor_load_ms
        << ", \"growth\": " << p.growth_ms
        << ", \"graph_export\": " << p.graph_export_ms << ", \"view_setup\": " << p.view_setup_ms
        << ", \"rgb_render\": " << p.rgb_render_ms << ", \"gpu_readback\": " << p.gpu_readback_ms
        << ", \"composite\": " << p.composite_ms << ", \"encode\": " << p.encode_ms
        << ", \"write\": " << p.write_ms << ", \"rgb_write\": " << p.rgb_write_ms
        << ", \"depth_render_write\": " << p.depth_render_write_ms
        << ", \"instance_mask_render_write\": " << p.instance_mask_render_write_ms
        << ", \"label_render_write\": " << p.label_render_write_ms << ", \"total_engine\": " << p.total_engine_ms
        << "},\n";
    out << "      \"error\": \"" << JsonEscapeSynthetic(point.error_message) << "\"\n";
    out << "    }" << (i + 1 < datapoints.size() ? "," : "") << "\n";
  }
  out << "  ]\n";
  out << "}\n";
}

void WriteSyntheticParityManifestJson(const std::filesystem::path& path, const SyntheticOptions& options) {
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }

  std::ofstream out(path.string(), std::ios::trunc);
  if (!out.is_open()) {
    EVOENGINE_WARNING("Failed to write synthetic parity manifest: " + path.string())
    return;
  }

  const auto descriptor_file = ResolveProjectAssetFilesystemPath(options.descriptor_path);
  const auto scene_file = ResolveProjectAssetFilesystemPath(options.scene_path);
  const auto overrides = BuildSyntheticOverrideList(options);

  out << std::fixed << std::setprecision(6);
  out << "{\n";
  out << "  \"strict_parity\": " << (options.strict_parity ? "true" : "false") << ",\n";
  out << "  \"project_path\": \"" << JsonEscapeSynthetic(options.project_path.string()) << "\",\n";
  out << "  \"scene\": {\n";
  out << "    \"configured_path\": \"" << JsonEscapeSynthetic(options.scene_path.string()) << "\",\n";
  out << "    \"resolved_file\": \"" << JsonEscapeSynthetic(scene_file.string()) << "\",\n";
  out << "    \"fnv1a64\": \"" << JsonEscapeSynthetic(Fnv1aFileHashHex(scene_file)) << "\"\n";
  out << "  },\n";
  out << "  \"descriptor\": {\n";
  out << "    \"configured_path\": \"" << JsonEscapeSynthetic(options.descriptor_path.string()) << "\",\n";
  out << "    \"resolved_file\": \"" << JsonEscapeSynthetic(descriptor_file.string()) << "\",\n";
  out << "    \"fnv1a64\": \"" << JsonEscapeSynthetic(Fnv1aFileHashHex(descriptor_file)) << "\"\n";
  out << "  },\n";
  out << "  \"growth_contract\": {\n";
  out << "    \"load_scene\": " << (options.load_scene ? "true" : "false") << ",\n";
  out << "    \"use_scene_pine_transforms\": " << (options.use_scene_pine_transforms ? "true" : "false") << ",\n";
  out << "    \"use_scene_pine_growth\": " << (options.use_scene_pine_growth ? "true" : "false") << ",\n";
  out << "    \"preserve_scene_pine_seed\": " << (options.preserve_scene_pine_seed ? "true" : "false") << ",\n";
  out << "    \"scene_pine_growth_mode\": \"" << JsonEscapeSynthetic(options.scene_pine_growth_mode) << "\",\n";
  out << "    \"calendar_start_from_reset\": " << (options.calendar_start_from_reset ? "true" : "false") << ",\n";
  out << "    \"calendar_step_days\": " << options.calendar_step_days << "\n";
  out << "  },\n";
  out << "  \"annotation_contract\": {\n";
  out << "    \"export_annotation_skeleton\": " << (options.export_annotation_skeleton ? "true" : "false") << ",\n";
  out << "    \"export_annotation_overlay\": " << (options.export_annotation_overlay ? "true" : "false") << ",\n";
  out << "    \"needle_sample_count\": " << kAnnotationNeedleSampleCount << ",\n";
  out << "    \"sheath_sample_count\": " << kAnnotationSheathSampleCount << ",\n";
  out << "    \"stem_sample_count\": " << kAnnotationStemSampleCount << "\n";
  out << "  },\n";
  out << "  \"applied_overrides\": [";
  for (size_t i = 0; i < overrides.size(); ++i) {
    out << (i == 0 ? "" : ", ") << "\"" << JsonEscapeSynthetic(overrides[i]) << "\"";
  }
  out << "]\n";
  out << "}\n";
}

bool TryParseUInt32(const std::string& value, std::uint32_t& out_value) {
  try {
    size_t consumed = 0;
    const unsigned long parsed = std::stoul(value, &consumed);
    if (consumed != value.size() || parsed > std::numeric_limits<std::uint32_t>::max()) {
      return false;
    }
    out_value = static_cast<std::uint32_t>(parsed);
    return true;
  } catch (...) {
    return false;
  }
}

bool TryParseFloat(const std::string& value, float& out_value) {
  try {
    size_t consumed = 0;
    const float parsed = std::stof(value, &consumed);
    if (consumed != value.size() || !std::isfinite(parsed)) {
      return false;
    }
    out_value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

bool TryParseVec4(const std::string& value, std::array<float, 4>& out_value) {
  std::string normalized = value;
  for (char& ch : normalized) {
    if (ch == ',' || ch == '[' || ch == ']') {
      ch = ' ';
    }
  }

  std::istringstream iss(normalized);
  std::array<float, 4> parsed{};
  if (!(iss >> parsed[0] >> parsed[1] >> parsed[2] >> parsed[3])) {
    return false;
  }
  for (const float component : parsed) {
    if (!std::isfinite(component)) {
      return false;
    }
  }
  out_value = parsed;
  return true;
}

glm::vec4 ArrayToVec4(const std::array<float, 4>& value) {
  return glm::vec4(value[0], value[1], value[2], value[3]);
}

bool LoadSyntheticOptionsFromFile(const std::filesystem::path& options_file_path, SyntheticOptions& options,
                                  std::string& error) {
  std::ifstream in(options_file_path, std::ios::in);
  if (!in.is_open()) {
    error = "Failed to open synthetic options file: " + options_file_path.string();
    return false;
  }

  std::string line;
  while (std::getline(in, line)) {
    line = Trim(line);
    if (line.empty() || line[0] == '#') {
      continue;
    }

    const auto separator_index = line.find('=');
    if (separator_index == std::string::npos) {
      continue;
    }

    const auto key = Trim(line.substr(0, separator_index));
    const auto value = Trim(line.substr(separator_index + 1));

    if (key == "project_path") {
      if (!value.empty()) {
        options.project_path = value;
      }
    } else if (key == "scene_path") {
      options.scene_path = value;
    } else if (key == "descriptor_path") {
      options.descriptor_path = value;
    } else if (key == "output_root") {
      options.output_root = value;
    } else if (key == "output_name") {
      options.output_name = value;
    } else if (key == "camera_rig_file") {
      options.camera_rig_file = value;
    } else if (key == "background_image") {
      options.background_image = value;
    } else if (key == "background_dir") {
      options.background_dir = value;
    } else if (key == "scene_pine_growth_mode") {
      options.scene_pine_growth_mode = value;
    } else if (key == "render_mode") {
      options.render_mode = value;
    } else if (key == "rgb_output_format") {
      options.rgb_output_format = value;
    } else if (key == "base_seed") {
      if (!TryParseUInt32(value, options.base_seed)) {
        error = "Invalid uint32 for base_seed: " + value;
        return false;
      }
    } else if (key == "seed_a") {
      if (!TryParseUInt32(value, options.seed_a)) {
        error = "Invalid uint32 for seed_a: " + value;
        return false;
      }
    } else if (key == "seed_b") {
      if (!TryParseUInt32(value, options.seed_b)) {
        error = "Invalid uint32 for seed_b: " + value;
        return false;
      }
    } else if (key == "seed_c") {
      if (!TryParseUInt32(value, options.seed_c)) {
        error = "Invalid uint32 for seed_c: " + value;
        return false;
      }
    } else if (key == "sample_count") {
      if (!TryParseInt(value, options.sample_count)) {
        error = "Invalid integer for sample_count: " + value;
        return false;
      }
    } else if (key == "start_index") {
      if (!TryParseInt(value, options.start_index)) {
        error = "Invalid integer for start_index: " + value;
        return false;
      }
    } else if (key == "worker_id") {
      if (!TryParseInt(value, options.worker_id)) {
        error = "Invalid integer for worker_id: " + value;
        return false;
      }
    } else if (key == "worker_count") {
      if (!TryParseInt(value, options.worker_count)) {
        error = "Invalid integer for worker_count: " + value;
        return false;
      }
    } else if (key == "frame_count") {
      if (!TryParseInt(value, options.frame_count)) {
        error = "Invalid integer for frame_count: " + value;
        return false;
      }
    } else if (key == "width") {
      if (!TryParseInt(value, options.width)) {
        error = "Invalid integer for width: " + value;
        return false;
      }
    } else if (key == "height") {
      if (!TryParseInt(value, options.height)) {
        error = "Invalid integer for height: " + value;
        return false;
      }
    } else if (key == "profile_warmup_datapoints") {
      if (!TryParseInt(value, options.profile_warmup_datapoints)) {
        error = "Invalid integer for profile_warmup_datapoints: " + value;
        return false;
      }
    } else if (key == "writer_threads") {
      if (!TryParseInt(value, options.writer_threads)) {
        error = "Invalid integer for writer_threads: " + value;
        return false;
      }
    } else if (key == "jpg_quality") {
      if (!TryParseInt(value, options.jpg_quality)) {
        error = "Invalid integer for jpg_quality: " + value;
        return false;
      }
    } else if (key == "png_compression_level") {
      if (!TryParseInt(value, options.png_compression_level)) {
        error = "Invalid integer for png_compression_level: " + value;
        return false;
      }
    } else if (key == "plant_blur_radius_px") {
      if (!TryParseFloat(value, options.plant_blur_radius_px)) {
        error = "Invalid float for plant_blur_radius_px: " + value;
        return false;
      }
    } else if (key == "max_target_gdd") {
      if (!TryParseFloat(value, options.max_target_gdd)) {
        error = "Invalid float for max_target_gdd: " + value;
        return false;
      }
      options.override_max_target_gdd = true;
    } else if (key == "triangle_side_length") {
      if (!TryParseFloat(value, options.triangle_side_length)) {
        error = "Invalid float for triangle_side_length: " + value;
        return false;
      }
    } else if (key == "triangle_offset_x") {
      if (!TryParseFloat(value, options.triangle_offset_x)) {
        error = "Invalid float for triangle_offset_x: " + value;
        return false;
      }
    } else if (key == "triangle_offset_z") {
      if (!TryParseFloat(value, options.triangle_offset_z)) {
        error = "Invalid float for triangle_offset_z: " + value;
        return false;
      }
    } else if (key == "triangle_yaw_deg") {
      if (!TryParseFloat(value, options.triangle_yaw_deg)) {
        error = "Invalid float for triangle_yaw_deg: " + value;
        return false;
      }
    } else if (key == "camera_position_offset_y") {
      if (!TryParseFloat(value, options.camera_position_offset_y)) {
        error = "Invalid float for camera_position_offset_y: " + value;
        return false;
      }
    } else if (key == "camera_screen_offset_x_percent") {
      if (!TryParseFloat(value, options.camera_screen_offset_x_percent)) {
        error = "Invalid float for camera_screen_offset_x_percent: " + value;
        return false;
      }
    } else if (key == "camera_screen_offset_y_percent") {
      if (!TryParseFloat(value, options.camera_screen_offset_y_percent)) {
        error = "Invalid float for camera_screen_offset_y_percent: " + value;
        return false;
      }
    } else if (key == "visual_scale") {
      if (!TryParseFloat(value, options.visual_scale)) {
        error = "Invalid float for visual_scale: " + value;
        return false;
      }
    } else if (key == "ambient_light") {
      if (!TryParseFloat(value, options.ambient_light)) {
        error = "Invalid float for ambient_light: " + value;
        return false;
      }
    } else if (key == "directional_light") {
      if (!TryParseFloat(value, options.directional_light)) {
        error = "Invalid float for directional_light: " + value;
        return false;
      }
    } else if (key == "scene_pine_target_gdd_multiplier") {
      if (!TryParseFloat(value, options.scene_pine_target_gdd_multiplier)) {
        error = "Invalid float for scene_pine_target_gdd_multiplier: " + value;
        return false;
      }
      options.override_scene_pine_target_gdd_multiplier = true;
    } else if (key == "annual_whorl_probability") {
      if (!TryParseFloat(value, options.annual_whorl_probability)) {
        error = "Invalid float for annual_whorl_probability: " + value;
        return false;
      }
      options.override_annual_whorl_probability = true;
    } else if (key == "whorl_position_norm") {
      if (!TryParseFloat(value, options.whorl_position_norm)) {
        error = "Invalid float for whorl_position_norm: " + value;
        return false;
      }
      options.override_whorl_position_norm = true;
    } else if (key == "branches_per_whorl") {
      if (!TryParseFloat(value, options.branches_per_whorl)) {
        error = "Invalid float for branches_per_whorl: " + value;
        return false;
      }
      options.override_branches_per_whorl = true;
    } else if (key == "whorl_dormancy_years") {
      if (!TryParseFloat(value, options.whorl_dormancy_years)) {
        error = "Invalid float for whorl_dormancy_years: " + value;
        return false;
      }
      options.override_whorl_dormancy_years = true;
    } else if (key == "max_branching_order") {
      if (!TryParseInt(value, options.max_branching_order)) {
        error = "Invalid integer for max_branching_order: " + value;
        return false;
      }
      options.override_max_branching_order = true;
    } else if (key == "calendar_step_days") {
      if (!TryParseFloat(value, options.calendar_step_days)) {
        error = "Invalid float for calendar_step_days: " + value;
        return false;
      }
    } else if (key == "young_needle_palette_rgba") {
      if (!TryParseVec4(value, options.young_needle_palette_rgba)) {
        error = "Invalid vec4 for young_needle_palette_rgba: " + value;
        return false;
      }
      options.override_young_needle_palette_rgba = true;
    } else if (key == "older_needle_palette_rgba") {
      if (!TryParseVec4(value, options.older_needle_palette_rgba)) {
        error = "Invalid vec4 for older_needle_palette_rgba: " + value;
        return false;
      }
      options.override_older_needle_palette_rgba = true;
    } else if (key == "dry_brown_needle_palette_rgba") {
      if (!TryParseVec4(value, options.dry_brown_needle_palette_rgba)) {
        error = "Invalid vec4 for dry_brown_needle_palette_rgba: " + value;
        return false;
      }
      options.override_dry_brown_needle_palette_rgba = true;
    } else if (key == "main_stem_palette_rgba") {
      if (!TryParseVec4(value, options.main_stem_palette_rgba)) {
        error = "Invalid vec4 for main_stem_palette_rgba: " + value;
        return false;
      }
      options.override_main_stem_palette_rgba = true;
    } else if (key == "mature_bark_stem_palette_rgba") {
      if (!TryParseVec4(value, options.mature_bark_stem_palette_rgba)) {
        error = "Invalid vec4 for mature_bark_stem_palette_rgba: " + value;
        return false;
      }
      options.override_mature_bark_stem_palette_rgba = true;
    } else if (key == "node_sheath_brown_palette_rgba") {
      if (!TryParseVec4(value, options.node_sheath_brown_palette_rgba)) {
        error = "Invalid vec4 for node_sheath_brown_palette_rgba: " + value;
        return false;
      }
      options.override_node_sheath_brown_palette_rgba = true;
    } else if (key == "fascicle_sheath_palette_rgba") {
      if (!TryParseVec4(value, options.fascicle_sheath_palette_rgba)) {
        error = "Invalid vec4 for fascicle_sheath_palette_rgba: " + value;
        return false;
      }
      options.override_fascicle_sheath_palette_rgba = true;
    } else if (key == "needle_tip_color_mix_start") {
      if (!TryParseFloat(value, options.needle_tip_color_mix_start)) {
        error = "Invalid float for needle_tip_color_mix_start: " + value;
        return false;
      }
      options.override_needle_tip_color_mix_start = true;
    } else if (key == "needle_tip_color_exponent") {
      if (!TryParseFloat(value, options.needle_tip_color_exponent)) {
        error = "Invalid float for needle_tip_color_exponent: " + value;
        return false;
      }
      options.override_needle_tip_color_exponent = true;
    } else if (key == "needle_old_thinning_fraction") {
      if (!TryParseFloat(value, options.needle_old_thinning_fraction)) {
        error = "Invalid float for needle_old_thinning_fraction: " + value;
        return false;
      }
      options.override_needle_old_thinning_fraction = true;
    } else if (key == "needle_min_strand_thickness_m") {
      if (!TryParseFloat(value, options.needle_min_strand_thickness_m)) {
        error = "Invalid float for needle_min_strand_thickness_m: " + value;
        return false;
      }
      options.override_needle_min_strand_thickness_m = true;
    } else if (key == "needle_micro_variation") {
      if (!TryParseFloat(value, options.needle_micro_variation)) {
        error = "Invalid float for needle_micro_variation: " + value;
        return false;
      }
      options.override_needle_micro_variation = true;
    } else if (key == "stem_micro_variation") {
      if (!TryParseFloat(value, options.stem_micro_variation)) {
        error = "Invalid float for stem_micro_variation: " + value;
        return false;
      }
      options.override_stem_micro_variation = true;
    } else if (key == "young_needle_roughness") {
      if (!TryParseFloat(value, options.young_needle_roughness)) {
        error = "Invalid float for young_needle_roughness: " + value;
        return false;
      }
      options.override_young_needle_roughness = true;
    } else if (key == "old_needle_roughness") {
      if (!TryParseFloat(value, options.old_needle_roughness)) {
        error = "Invalid float for old_needle_roughness: " + value;
        return false;
      }
      options.override_old_needle_roughness = true;
    } else if (key == "young_needle_specular") {
      if (!TryParseFloat(value, options.young_needle_specular)) {
        error = "Invalid float for young_needle_specular: " + value;
        return false;
      }
      options.override_young_needle_specular = true;
    } else if (key == "old_needle_specular") {
      if (!TryParseFloat(value, options.old_needle_specular)) {
        error = "Invalid float for old_needle_specular: " + value;
        return false;
      }
      options.override_old_needle_specular = true;
    } else if (key == "stem_roughness") {
      if (!TryParseFloat(value, options.stem_roughness)) {
        error = "Invalid float for stem_roughness: " + value;
        return false;
      }
      options.override_stem_roughness = true;
    } else if (key == "stem_specular") {
      if (!TryParseFloat(value, options.stem_specular)) {
        error = "Invalid float for stem_specular: " + value;
        return false;
      }
      options.override_stem_specular = true;
    } else if (key == "node_browning_strength") {
      if (!TryParseFloat(value, options.node_browning_strength)) {
        error = "Invalid float for node_browning_strength: " + value;
        return false;
      }
      options.override_node_browning_strength = true;
    } else if (key == "sheath_browning_strength") {
      if (!TryParseFloat(value, options.sheath_browning_strength)) {
        error = "Invalid float for sheath_browning_strength: " + value;
        return false;
      }
      options.override_sheath_browning_strength = true;
    } else if (key == "node_browning_radius_norm") {
      if (!TryParseFloat(value, options.node_browning_radius_norm)) {
        error = "Invalid float for node_browning_radius_norm: " + value;
        return false;
      }
      options.override_node_browning_radius_norm = true;
    } else if (key == "needle_twist_turns") {
      if (!TryParseFloat(value, options.needle_twist_turns)) {
        error = "Invalid float for needle_twist_turns: " + value;
        return false;
      }
      options.override_needle_twist_turns = true;
    } else if (key == "needle_edge_darkening") {
      if (!TryParseFloat(value, options.needle_edge_darkening)) {
        error = "Invalid float for needle_edge_darkening: " + value;
        return false;
      }
      options.override_needle_edge_darkening = true;
    } else if (key == "needle_segment_count") {
      if (!TryParseInt(value, options.needle_segment_count)) {
        error = "Invalid integer for needle_segment_count: " + value;
        return false;
      }
      options.override_needle_segment_count = true;
    } else if (key == "fascicle_sheath_length_m") {
      if (!TryParseFloat(value, options.fascicle_sheath_length_m)) {
        error = "Invalid float for fascicle_sheath_length_m: " + value;
        return false;
      }
      options.override_fascicle_sheath_length_m = true;
    } else if (key == "fascicle_sheath_width_m") {
      if (!TryParseFloat(value, options.fascicle_sheath_width_m)) {
        error = "Invalid float for fascicle_sheath_width_m: " + value;
        return false;
      }
      options.override_fascicle_sheath_width_m = true;
    } else if (key == "needle_year0_length_multiplier") {
      if (!TryParseFloat(value, options.needle_year0_length_multiplier)) {
        error = "Invalid float for needle_year0_length_multiplier: " + value;
        return false;
      }
      options.override_needle_year0_length_multiplier = true;
    } else if (key == "needle_axial_age_span") {
      if (!TryParseFloat(value, options.needle_axial_age_span)) {
        error = "Invalid float for needle_axial_age_span: " + value;
        return false;
      }
      options.override_needle_axial_age_span = true;
    } else if (key == "needle_axial_age_exponent") {
      if (!TryParseFloat(value, options.needle_axial_age_exponent)) {
        error = "Invalid float for needle_axial_age_exponent: " + value;
        return false;
      }
      options.override_needle_axial_age_exponent = true;
    } else if (key == "ray_trace_samples") {
      if (!TryParseUInt32(value, options.ray_trace_samples)) {
        error = "Invalid uint32 for ray_trace_samples: " + value;
        return false;
      }
    } else if (key == "ray_trace_bounces") {
      if (!TryParseUInt32(value, options.ray_trace_bounces)) {
        error = "Invalid uint32 for ray_trace_bounces: " + value;
        return false;
      }
    } else if (key == "transparent_bg") {
      if (!TryParseBool(value, options.transparent_bg)) {
        error = "Invalid bool for transparent_bg: " + value;
        return false;
      }
    } else if (key == "load_scene") {
      if (!TryParseBool(value, options.load_scene)) {
        error = "Invalid bool for load_scene: " + value;
        return false;
      }
    } else if (key == "use_scene_main_camera") {
      if (!TryParseBool(value, options.use_scene_main_camera)) {
        error = "Invalid bool for use_scene_main_camera: " + value;
        return false;
      }
    } else if (key == "use_scene_pine_transforms") {
      if (!TryParseBool(value, options.use_scene_pine_transforms)) {
        error = "Invalid bool for use_scene_pine_transforms: " + value;
        return false;
      }
    } else if (key == "use_scene_pine_growth") {
      if (!TryParseBool(value, options.use_scene_pine_growth)) {
        error = "Invalid bool for use_scene_pine_growth: " + value;
        return false;
      }
    } else if (key == "render_needles") {
      if (!TryParseBool(value, options.render_needles)) {
        error = "Invalid bool for render_needles: " + value;
        return false;
      }
    } else if (key == "preserve_scene_pine_seed") {
      if (!TryParseBool(value, options.preserve_scene_pine_seed)) {
        error = "Invalid bool for preserve_scene_pine_seed: " + value;
        return false;
      }
    } else if (key == "calendar_start_from_reset") {
      if (!TryParseBool(value, options.calendar_start_from_reset)) {
        error = "Invalid bool for calendar_start_from_reset: " + value;
        return false;
      }
    } else if (key == "strict_parity") {
      if (!TryParseBool(value, options.strict_parity)) {
        error = "Invalid bool for strict_parity: " + value;
        return false;
      }
    } else if (key == "batch_output_subdirs") {
      if (!TryParseBool(value, options.batch_output_subdirs)) {
        error = "Invalid bool for batch_output_subdirs: " + value;
        return false;
      }
    } else if (key == "keep_going") {
      if (!TryParseBool(value, options.keep_going)) {
        error = "Invalid bool for keep_going: " + value;
        return false;
      }
    } else if (key == "composite_background") {
      if (!TryParseBool(value, options.composite_background)) {
        error = "Invalid bool for composite_background: " + value;
        return false;
      }
    } else if (key == "write_raw_rgba") {
      if (!TryParseBool(value, options.write_raw_rgba)) {
        error = "Invalid bool for write_raw_rgba: " + value;
        return false;
      }
    } else if (key == "write_foreground_mask") {
      if (!TryParseBool(value, options.write_foreground_mask)) {
        error = "Invalid bool for write_foreground_mask: " + value;
        return false;
      }
    } else if (key == "export_depth") {
      if (!TryParseBool(value, options.export_depth)) {
        error = "Invalid bool for export_depth: " + value;
        return false;
      }
    } else if (key == "export_instance_mask") {
      if (!TryParseBool(value, options.export_instance_mask)) {
        error = "Invalid bool for export_instance_mask: " + value;
        return false;
      }
    } else if (key == "export_synthetic_labels") {
      if (!TryParseBool(value, options.export_synthetic_labels)) {
        error = "Invalid bool for export_synthetic_labels: " + value;
        return false;
      }
    } else if (key == "export_flow_graph") {
      if (!TryParseBool(value, options.export_flow_graph)) {
        error = "Invalid bool for export_flow_graph: " + value;
        return false;
      }
    } else if (key == "export_node_graph") {
      if (!TryParseBool(value, options.export_node_graph)) {
        error = "Invalid bool for export_node_graph: " + value;
        return false;
      }
    } else if (key == "export_needle_skeleton") {
      if (!TryParseBool(value, options.export_needle_skeleton)) {
        error = "Invalid bool for export_needle_skeleton: " + value;
        return false;
      }
    } else if (key == "export_annotation_skeleton") {
      if (!TryParseBool(value, options.export_annotation_skeleton)) {
        error = "Invalid bool for export_annotation_skeleton: " + value;
        return false;
      }
    } else if (key == "export_annotation_overlay") {
      if (!TryParseBool(value, options.export_annotation_overlay)) {
        error = "Invalid bool for export_annotation_overlay: " + value;
        return false;
      }
    } else if (key == "uncapped_growth") {
      if (!TryParseBool(value, options.uncapped_growth)) {
        error = "Invalid bool for uncapped_growth: " + value;
        return false;
      }
    } else {
      error = "Unknown synthetic config key: " + key;
      return false;
    }
  }

  if (options.output_name.empty()) {
    error = "output_name is required.";
    return false;
  }
  if (options.descriptor_path.empty()) {
    error = "descriptor_path is required.";
    return false;
  }
  if (!options.use_scene_main_camera && options.camera_rig_file.empty()) {
    error = "camera_rig_file is required.";
    return false;
  }
  if (options.frame_count <= 0) {
    error = "frame_count must be > 0.";
    return false;
  }
  if (options.sample_count <= 0) {
    error = "sample_count must be > 0.";
    return false;
  }
  if (options.start_index < 0) {
    error = "start_index must be >= 0.";
    return false;
  }
  if (options.worker_id < 0) {
    error = "worker_id must be >= 0.";
    return false;
  }
  if (options.worker_count <= 0) {
    error = "worker_count must be > 0.";
    return false;
  }
  if (options.worker_id >= options.worker_count) {
    error = "worker_id must be less than worker_count.";
    return false;
  }
  if (options.width <= 0 || options.height <= 0) {
    error = "width and height must be > 0.";
    return false;
  }
  if (options.profile_warmup_datapoints < 0) {
    error = "profile_warmup_datapoints must be >= 0.";
    return false;
  }
  if (options.triangle_side_length <= 0.0f) {
    error = "triangle_side_length must be > 0.";
    return false;
  }
  if (options.visual_scale <= 0.0f) {
    error = "visual_scale must be > 0.";
    return false;
  }
  if (options.scene_pine_growth_mode != "direct" && options.scene_pine_growth_mode != "calendar_scene_multiplier") {
    error = "scene_pine_growth_mode must be 'direct' or 'calendar_scene_multiplier'.";
    return false;
  }
  if (options.render_mode != "rasterization" && options.render_mode != "ray_tracing") {
    error = "render_mode must be 'rasterization' or 'ray_tracing'.";
    return false;
  }
  if (options.ray_trace_samples == 0) {
    error = "ray_trace_samples must be > 0.";
    return false;
  }
  if (options.ray_trace_bounces == 0) {
    error = "ray_trace_bounces must be > 0.";
    return false;
  }
  if (options.writer_threads <= 0) {
    error = "writer_threads must be > 0.";
    return false;
  }
  if (options.jpg_quality <= 0 || options.jpg_quality > 100) {
    error = "jpg_quality must be in [1, 100].";
    return false;
  }
  if (options.png_compression_level < 0 || options.png_compression_level > 9) {
    error = "png_compression_level must be in [0, 9].";
    return false;
  }
  if (options.rgb_output_format != "jpg" && options.rgb_output_format != "png") {
    error = "rgb_output_format must be 'jpg' or 'png'.";
    return false;
  }
  if (options.plant_blur_radius_px < 0.0f || options.plant_blur_radius_px > 20.0f) {
    error = "plant_blur_radius_px must be in [0, 20].";
    return false;
  }
  if (options.calendar_step_days <= 0.0f) {
    error = "calendar_step_days must be > 0.";
    return false;
  }
  if (options.strict_parity) {
    if (!options.load_scene) {
      error = "strict_parity requires load_scene=true.";
      return false;
    }
    if (!options.use_scene_pine_transforms) {
      error = "strict_parity requires use_scene_pine_transforms=true.";
      return false;
    }
    if (!options.use_scene_pine_growth) {
      error = "strict_parity requires use_scene_pine_growth=true.";
      return false;
    }
    if (!options.preserve_scene_pine_seed) {
      error = "strict_parity requires preserve_scene_pine_seed=true.";
      return false;
    }
    if (options.scene_pine_growth_mode != "calendar_scene_multiplier") {
      error = "strict_parity requires scene_pine_growth_mode=calendar_scene_multiplier.";
      return false;
    }
    if (options.override_scene_pine_target_gdd_multiplier) {
      error = "strict_parity rejects scene_pine_target_gdd_multiplier; edit the scene/descriptor instead.";
      return false;
    }
    if (options.override_max_target_gdd) {
      error = "strict_parity rejects max_target_gdd; edit the descriptor or scene target GDD instead.";
      return false;
    }
    if (HasSyntheticDescriptorOverrides(options)) {
      error = "strict_parity rejects descriptor parameter overrides; edit the .spine asset instead.";
      return false;
    }
    if (options.uncapped_growth) {
      error = "strict_parity rejects uncapped_growth.";
      return false;
    }
    if (!options.calendar_start_from_reset) {
      error = "strict_parity requires calendar_start_from_reset=true.";
      return false;
    }
  }
  return true;
}

std::string JsonEscapeSynthetic(const std::string& value) {
  std::string escaped;
  escaped.reserve(value.size());
  for (const char ch : value) {
    switch (ch) {
      case '\\':
        escaped += "\\\\";
        break;
      case '"':
        escaped += "\\\"";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        escaped += ch;
        break;
    }
  }
  return escaped;
}

bool IsSyntheticBackgroundImagePath(const std::filesystem::path& path) {
  const std::string extension = path.extension().string();
  std::string lower;
  lower.reserve(extension.size());
  for (const char ch : extension) {
    lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
  }
  return lower == ".png" || lower == ".jpg" || lower == ".jpeg" || lower == ".bmp" || lower == ".tga";
}

std::vector<std::filesystem::path> ResolveSyntheticBackgroundPaths(const SyntheticOptions& options) {
  std::vector<std::filesystem::path> paths;
  if (!options.background_image.empty()) {
    paths.emplace_back(std::filesystem::absolute(options.background_image));
  }
  if (!options.background_dir.empty() && std::filesystem::exists(options.background_dir)) {
    for (const auto& entry : std::filesystem::directory_iterator(options.background_dir)) {
      if (entry.is_regular_file() && IsSyntheticBackgroundImagePath(entry.path())) {
        paths.emplace_back(std::filesystem::absolute(entry.path()));
      }
    }
  }
  std::sort(paths.begin(), paths.end());
  paths.erase(std::unique(paths.begin(), paths.end()), paths.end());
  return paths;
}

std::filesystem::path SelectSyntheticBackgroundPath(const std::vector<std::filesystem::path>& backgrounds,
                                                    const int global_sample_index, const int frame_index,
                                                    const int view_index) {
  if (backgrounds.empty()) {
    return {};
  }
  const std::uint64_t mixed = static_cast<std::uint64_t>(std::max(0, global_sample_index)) * 0x9e3779b97f4a7c15ull ^
                              static_cast<std::uint64_t>(std::max(0, frame_index)) * 0xbf58476d1ce4e5b9ull ^
                              static_cast<std::uint64_t>(std::max(0, view_index)) * 0x94d049bb133111ebull;
  return backgrounds[static_cast<size_t>(mixed % backgrounds.size())];
}

glm::vec3 SafeNormalizeSynthetic(const glm::vec3& value, const glm::vec3& fallback) {
  const float len = glm::length(value);
  if (!std::isfinite(len) || len <= 1.0e-6f) {
    return fallback;
  }
  return value / len;
}

glm::quat BuildSyntheticCameraRotation(const glm::vec3& forward, const glm::vec3& up) {
  const glm::vec3 front = SafeNormalizeSynthetic(forward, glm::vec3(0.0f, 0.0f, -1.0f));
  glm::vec3 up_vec = SafeNormalizeSynthetic(up, glm::vec3(0.0f, 1.0f, 0.0f));
  glm::vec3 right = SafeNormalizeSynthetic(glm::cross(front, up_vec), glm::vec3(1.0f, 0.0f, 0.0f));
  up_vec = SafeNormalizeSynthetic(glm::cross(right, front), glm::vec3(0.0f, 1.0f, 0.0f));

  glm::mat3 world_from_local(1.0f);
  world_from_local[0] = right;
  world_from_local[1] = up_vec;
  world_from_local[2] = -front;
  return glm::normalize(glm::quat_cast(world_from_local));
}

bool LoadSyntheticCameraRigFromFile(const std::filesystem::path& path, const float y_offset,
                                    std::vector<SyntheticCameraRigView>& out_views, std::string& error) {
  std::ifstream in(path.string(), std::ios::in);
  if (!in.is_open()) {
    error = "Failed to open camera rig file: " + path.string();
    return false;
  }

  out_views.clear();
  std::string line;
  int line_number = 0;
  while (std::getline(in, line)) {
    line_number++;
    line = Trim(line);
    if (line.empty() || line[0] == '#') {
      continue;
    }

    SyntheticCameraRigView view;
    std::istringstream iss(line);
    if (!(iss >> view.label >> view.position.x >> view.position.y >> view.position.z >> view.fx >> view.fy >>
          view.cx >> view.cy >> view.width >> view.height >> view.near_distance >> view.far_distance >>
          view.fov_deg >> view.forward.x >> view.forward.y >> view.forward.z >> view.up.x >> view.up.y >>
          view.up.z)) {
      error = "Malformed camera rig line " + std::to_string(line_number) + " in " + path.string();
      return false;
    }

    if (view.width <= 0 || view.height <= 0) {
      error = "Invalid camera resolution on line " + std::to_string(line_number) + " in " + path.string();
      return false;
    }
    view.position.y += y_offset;
    view.near_distance = std::max(1.0e-4f, view.near_distance);
    view.far_distance = std::max(view.near_distance + 1.0e-4f, view.far_distance);
    view.fov_deg = std::clamp(view.fov_deg, 5.0f, 175.0f);
    view.forward = SafeNormalizeSynthetic(view.forward, glm::vec3(0.0f, 0.0f, -1.0f));
    view.up = SafeNormalizeSynthetic(view.up, glm::vec3(0.0f, 1.0f, 0.0f));
    out_views.emplace_back(view);
  }

  if (out_views.empty()) {
    error = "Camera rig file has no usable camera lines: " + path.string();
    return false;
  }
  return true;
}

void WriteSyntheticCameraMetadataJson(const std::filesystem::path& path,
                                      const std::vector<SyntheticCameraRigView>& views) {
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }

  std::ofstream out(path.string(), std::ios::trunc);
  if (!out.is_open()) {
    EVOENGINE_WARNING("Failed to write camera metadata JSON: " + path.string())
    return;
  }

  out << std::fixed << std::setprecision(9);
  out << "{\n  \"views\": [\n";
  for (size_t i = 0; i < views.size(); ++i) {
    const auto& view = views[i];
    out << "    {\n";
    out << "      \"index\": " << i << ",\n";
    out << "      \"label\": \"" << JsonEscapeSynthetic(view.label) << "\",\n";
    out << "      \"position\": [" << view.position.x << ", " << view.position.y << ", " << view.position.z << "],\n";
    out << "      \"forward\": [" << view.forward.x << ", " << view.forward.y << ", " << view.forward.z << "],\n";
    out << "      \"up\": [" << view.up.x << ", " << view.up.y << ", " << view.up.z << "],\n";
    out << "      \"intrinsic\": {\"fx\": " << view.fx << ", \"fy\": " << view.fy << ", \"cx\": " << view.cx
        << ", \"cy\": " << view.cy << "},\n";
    out << "      \"resolution\": [" << view.width << ", " << view.height << "],\n";
    out << "      \"near\": " << view.near_distance << ",\n";
    out << "      \"far\": " << view.far_distance << ",\n";
    out << "      \"fov_deg\": " << view.fov_deg << "\n";
    out << "    }" << (i + 1 < views.size() ? "," : "") << "\n";
  }
  out << "  ]\n}\n";
}

bool IsFiniteSyntheticVec3(const glm::vec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

void WriteSyntheticAnnotationSkeletonJson(const std::filesystem::path& path, const std::string& sample_name,
                                          const int global_sample_index, const int local_sample_index,
                                          const int frame_index, const std::array<std::uint32_t, 3>& seeds,
                                          const std::vector<std::shared_ptr<ScotsPine>>& pines) {
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }

  std::ofstream out(path.string(), std::ios::trunc);
  if (!out.is_open()) {
    throw std::runtime_error("Failed to write annotation skeleton JSON: " + path.string());
  }

  const size_t tree_count = std::min<size_t>(3, pines.size());
  out << std::fixed << std::setprecision(9);
  out << "{\n";
  out << "  \"schema\": \"scots_pine_annotation_skeleton\",\n";
  out << "  \"version\": 1,\n";
  out << "  \"sample_name\": \"" << JsonEscapeSynthetic(sample_name) << "\",\n";
  out << "  \"global_sample_index\": " << global_sample_index << ",\n";
  out << "  \"local_sample_index\": " << local_sample_index << ",\n";
  out << "  \"frame_index\": " << frame_index << ",\n";
  out << "  \"seeds\": [" << seeds[0] << ", " << seeds[1] << ", " << seeds[2] << "],\n";
  out << "  \"sample_counts\": {\"needle\": " << kAnnotationNeedleSampleCount << ", \"sheath\": "
      << kAnnotationSheathSampleCount << ", \"stem\": " << kAnnotationStemSampleCount << "},\n";
  out << "  \"trees\": [\n";
  for (size_t tree_index = 0; tree_index < tree_count; ++tree_index) {
    out << "    ";
    if (pines[tree_index]) {
      pines[tree_index]->WriteAnnotationSkeletonTreeJson(out, static_cast<int>(tree_index));
    } else {
      out << "{\"tree_index\":" << tree_index << ",\"organs\":{\"needles\":[],\"sheaths\":[],\"stems\":[]}}";
    }
    out << (tree_index + 1 < tree_count ? "," : "") << "\n";
  }
  out << "  ]\n";
  out << "}\n";
}

std::array<glm::vec3, 3> BuildSyntheticTrianglePositions(const SyntheticOptions& options) {
  const float side = options.triangle_side_length;
  const float half_side = side * 0.5f;
  const float tri_h = side * std::sqrt(3.0f) * 0.5f;
  const float centroid_to_apex = (2.0f / 3.0f) * tri_h;
  const float centroid_to_base = (1.0f / 3.0f) * tri_h;

  std::array<glm::vec3, 3> points = {
      glm::vec3(0.0f, 0.0f, centroid_to_apex),
      glm::vec3(-half_side, 0.0f, -centroid_to_base),
      glm::vec3(half_side, 0.0f, -centroid_to_base),
  };

  const float yaw = glm::radians(options.triangle_yaw_deg);
  const float c = std::cos(yaw);
  const float s = std::sin(yaw);
  for (auto& p : points) {
    const float x = c * p.x + s * p.z;
    const float z = -s * p.x + c * p.z;
    p.x = x + options.triangle_offset_x;
    p.z = z + options.triangle_offset_z;
  }
  return points;
}

void DisableSyntheticPostProcessing(const std::shared_ptr<Camera>& camera) {
  if (const auto post_processing_stack = camera->post_processing_stack_ref.Get<PostProcessingStack>()) {
    post_processing_stack->enable_screen_space_ambient_occlusion = false;
    post_processing_stack->enable_bloom = false;
    post_processing_stack->enable_screen_space_reflection = false;
    post_processing_stack->enable_tone_mapping = false;
  }
}

std::shared_ptr<Scene> ResolveSyntheticWorkingScene(const SyntheticOptions& options, std::string& error) {
  if (!options.load_scene) {
    return AssetManager::CreateTemporaryAsset<Scene>();
  }

  auto source_scene = ResolveSceneAsset(options.scene_path, error);
  if (!source_scene) {
    return nullptr;
  }

  auto working_scene = AssetManager::CreateTemporaryAsset<Scene>();
  Scene::Clone(source_scene, working_scene);
  return working_scene;
}

std::vector<std::pair<Entity, std::shared_ptr<ScotsPine>>> FindSyntheticScenePines(
    const std::shared_ptr<Scene>& scene) {
  std::vector<std::pair<Entity, std::shared_ptr<ScotsPine>>> pines;
  if (!scene) {
    return pines;
  }

  const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>();
  if (!owners) {
    return pines;
  }

  for (const auto& entity : *owners) {
    if (!scene->IsEntityValid(entity)) {
      continue;
    }
    const auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
    if (pine) {
      pines.emplace_back(entity, pine);
    }
  }

  std::sort(pines.begin(), pines.end(), [&](const auto& lhs, const auto& rhs) {
    return scene->GetEntityName(lhs.first) < scene->GetEntityName(rhs.first);
  });
  return pines;
}

SyntheticCameraRigView BuildSyntheticViewFromSceneMainCamera(const std::shared_ptr<Scene>& scene,
                                                            const std::shared_ptr<Camera>& camera,
                                                            const SyntheticOptions& options) {
  if (!scene || !camera || !scene->IsEntityValid(camera->GetOwner())) {
    throw std::runtime_error("Scene main camera is missing or invalid.");
  }

  const auto camera_transform = scene->GetDataComponent<GlobalTransform>(camera->GetOwner());
  auto size = camera->GetSize();
  if (size.x == 0 || size.y == 0) {
    size = glm::uvec2(static_cast<unsigned>(options.width), static_cast<unsigned>(options.height));
  }

  SyntheticCameraRigView view;
  view.label = ScotsPineSyntheticSceneMainCameraLabel();
  view.position = camera_transform.GetPosition();
  view.forward = SafeNormalizeSynthetic(camera_transform.GetRotation() * glm::vec3(0.0f, 0.0f, -1.0f),
                                        glm::vec3(0.0f, 0.0f, -1.0f));
  view.up = SafeNormalizeSynthetic(camera_transform.GetRotation() * glm::vec3(0.0f, 1.0f, 0.0f),
                                   glm::vec3(0.0f, 1.0f, 0.0f));
  view.width = static_cast<int>(size.x);
  view.height = static_cast<int>(size.y);
  view.near_distance = std::max(1.0e-4f, camera->camera_settings.near_distance);
  view.far_distance = std::max(view.near_distance + 1.0e-4f, camera->camera_settings.far_distance);
  view.fov_deg = std::clamp(camera->camera_settings.fov, 5.0f, 175.0f);

  const float projection_half_angle = glm::radians(view.fov_deg * 0.25f);
  const float fy = projection_half_angle > 1.0e-6f ? (0.5f * static_cast<float>(view.height)) /
                                                         std::tan(projection_half_angle)
                                                   : 0.0f;
  view.fx = fy;
  view.fy = fy;
  view.cx = 0.5f * static_cast<float>(view.width);
  view.cy = 0.5f * static_cast<float>(view.height);
  return view;
}

std::vector<glm::ivec2> ProjectSyntheticAnnotationOverlayPixels(const std::vector<glm::vec3>& points,
                                                               const GlobalTransform& camera_transform,
                                                               const Camera& camera, const int width,
                                                               const int height) {
  std::vector<glm::ivec2> pixels;
  if (width <= 0 || height <= 0 || points.empty()) {
    return pixels;
  }

  const auto rotation = camera_transform.GetRotation();
  const auto position = camera_transform.GetPosition();
  const glm::vec3 front = rotation * glm::vec3(0.0f, 0.0f, -1.0f);
  const glm::vec3 up = rotation * glm::vec3(0.0f, 1.0f, 0.0f);
  const float aspect = static_cast<float>(width) / static_cast<float>(height);
  const glm::mat4 projection =
      glm::perspective(glm::radians(camera.camera_settings.fov * 0.5f), aspect,
                       camera.camera_settings.near_distance, camera.camera_settings.far_distance);
  const glm::mat4 view = glm::lookAt(position, position + front, up);
  const glm::mat4 projection_view = projection * view;

  pixels.reserve(points.size());
  for (const auto& point : points) {
    if (!IsFiniteSyntheticVec3(point)) {
      continue;
    }
    const glm::vec4 clip = projection_view * glm::vec4(point, 1.0f);
    if (!std::isfinite(clip.x) || !std::isfinite(clip.y) || !std::isfinite(clip.z) || !std::isfinite(clip.w) ||
        clip.w <= 1.0e-6f) {
      continue;
    }
    const glm::vec3 ndc = glm::vec3(clip) / clip.w;
    if (!IsFiniteSyntheticVec3(ndc) || ndc.x < -1.0f || ndc.x > 1.0f || ndc.y < -1.0f || ndc.y > 1.0f ||
        ndc.z < -1.0f || ndc.z > 1.0f) {
      continue;
    }
    const int x = glm::clamp(static_cast<int>(std::lround((ndc.x * 0.5f + 0.5f) * static_cast<float>(width - 1))), 0,
                             width - 1);
    const int y = glm::clamp(static_cast<int>(std::lround((1.0f - (ndc.y * 0.5f + 0.5f)) *
                                                          static_cast<float>(height - 1))),
                             0, height - 1);
    pixels.emplace_back(x, y);
  }
  return pixels;
}

glm::vec3 SyntheticPineCentroid(const std::shared_ptr<Scene>& scene,
                                const std::vector<std::shared_ptr<ScotsPine>>& pines) {
  glm::vec3 centroid(0.0f);
  int count = 0;
  for (const auto& pine : pines) {
    if (!pine || !scene->IsEntityValid(pine->GetOwner())) {
      continue;
    }
    centroid += scene->GetDataComponent<GlobalTransform>(pine->GetOwner()).GetPosition();
    count++;
  }
  return count > 0 ? centroid / static_cast<float>(count) : glm::vec3(0.0f);
}

void ApplySyntheticCameraScreenOffset(const std::shared_ptr<Scene>& scene, const Entity& camera_entity,
                                      const std::shared_ptr<Camera>& camera,
                                      const std::vector<std::shared_ptr<ScotsPine>>& pines,
                                      const SyntheticOptions& options) {
  if (!scene || !camera || !scene->IsEntityValid(camera_entity)) {
    return;
  }
  if (std::abs(options.camera_screen_offset_x_percent) <= 1.0e-6f &&
      std::abs(options.camera_screen_offset_y_percent) <= 1.0e-6f) {
    return;
  }

  auto transform = scene->GetDataComponent<GlobalTransform>(camera_entity);
  const glm::quat rotation = transform.GetRotation();
  const glm::vec3 forward = SafeNormalizeSynthetic(rotation * glm::vec3(0.0f, 0.0f, -1.0f),
                                                   glm::vec3(0.0f, 0.0f, -1.0f));
  const glm::vec3 up = SafeNormalizeSynthetic(rotation * glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
  const glm::vec3 right = SafeNormalizeSynthetic(rotation * glm::vec3(1.0f, 0.0f, 0.0f),
                                                 glm::vec3(1.0f, 0.0f, 0.0f));
  const glm::vec3 centroid = SyntheticPineCentroid(scene, pines);
  const float depth = std::max(0.01f, glm::dot(centroid - transform.GetPosition(), forward));
  const float vertical_span_at_depth = 2.0f * depth * std::tan(glm::radians(camera->camera_settings.fov * 0.25f));
  const glm::uvec2 size = camera->GetSize();
  const float aspect = size.y > 0 ? static_cast<float>(size.x) / static_cast<float>(size.y) : 1.0f;
  const float horizontal_span_at_depth = vertical_span_at_depth * aspect;

  const glm::vec3 delta = up * (vertical_span_at_depth * options.camera_screen_offset_y_percent * 0.01f) -
                          right * (horizontal_span_at_depth * options.camera_screen_offset_x_percent * 0.01f);
  transform.SetPosition(transform.GetPosition() + delta);
  scene->SetDataComponent(camera_entity, transform);
}

void ApplySyntheticDescriptorOverrides(const std::shared_ptr<ScotsPineDescriptor>& descriptor,
                                       const SyntheticOptions& options) {
  if (!descriptor) {
    return;
  }
  if (options.override_annual_whorl_probability) {
    descriptor->annual_whorl_probability.mean = std::clamp(options.annual_whorl_probability, 0.0f, 1.0f);
    descriptor->annual_whorl_probability.deviation = 0.0f;
  }
  if (options.override_whorl_position_norm) {
    descriptor->whorl_position_norm.mean = std::clamp(options.whorl_position_norm, 0.75f, 1.0f);
    descriptor->whorl_position_norm.deviation = 0.0f;
  }
  if (options.override_branches_per_whorl) {
    descriptor->branches_per_whorl.mean = std::clamp(options.branches_per_whorl, 1.0f, 5.0f);
    descriptor->branches_per_whorl.deviation = 0.0f;
  }
  if (options.override_whorl_dormancy_years) {
    descriptor->whorl_dormancy_years.mean = std::max(0.0f, options.whorl_dormancy_years);
    descriptor->whorl_dormancy_years.deviation = 0.0f;
  }
  if (options.override_max_branching_order) {
    descriptor->max_branching_order.mean = static_cast<float>(std::clamp(options.max_branching_order, 0, 4));
    descriptor->max_branching_order.deviation = 0.0f;
  }
  if (options.override_young_needle_palette_rgba) {
    descriptor->young_needle_palette_rgba = ArrayToVec4(options.young_needle_palette_rgba);
  }
  if (options.override_older_needle_palette_rgba) {
    descriptor->older_needle_palette_rgba = ArrayToVec4(options.older_needle_palette_rgba);
  }
  if (options.override_dry_brown_needle_palette_rgba) {
    descriptor->dry_brown_needle_palette_rgba = ArrayToVec4(options.dry_brown_needle_palette_rgba);
  }
  if (options.override_main_stem_palette_rgba) {
    descriptor->main_stem_palette_rgba = ArrayToVec4(options.main_stem_palette_rgba);
  }
  if (options.override_mature_bark_stem_palette_rgba) {
    descriptor->mature_bark_stem_palette_rgba = ArrayToVec4(options.mature_bark_stem_palette_rgba);
  }
  if (options.override_node_sheath_brown_palette_rgba) {
    descriptor->node_sheath_brown_palette_rgba = ArrayToVec4(options.node_sheath_brown_palette_rgba);
  }
  if (options.override_fascicle_sheath_palette_rgba) {
    descriptor->fascicle_sheath_palette_rgba = ArrayToVec4(options.fascicle_sheath_palette_rgba);
  }
  if (options.override_needle_tip_color_mix_start) {
    descriptor->needle_tip_color_mix_start = std::clamp(options.needle_tip_color_mix_start, 0.0f, 1.0f);
  }
  if (options.override_needle_tip_color_exponent) {
    descriptor->needle_tip_color_exponent = std::clamp(options.needle_tip_color_exponent, 0.1f, 6.0f);
  }
  if (options.override_needle_old_thinning_fraction) {
    descriptor->needle_old_thinning_fraction = std::clamp(options.needle_old_thinning_fraction, 0.0f, 0.95f);
  }
  if (options.override_needle_min_strand_thickness_m) {
    descriptor->needle_min_strand_thickness_m =
        std::clamp(options.needle_min_strand_thickness_m, 0.000001f, 0.002f);
  }
  if (options.override_needle_micro_variation) {
    descriptor->needle_micro_variation = std::clamp(options.needle_micro_variation, 0.0f, 0.25f);
  }
  if (options.override_stem_micro_variation) {
    descriptor->stem_micro_variation = std::clamp(options.stem_micro_variation, 0.0f, 0.25f);
  }
  if (options.override_young_needle_roughness) {
    descriptor->young_needle_roughness = std::clamp(options.young_needle_roughness, 0.02f, 1.0f);
  }
  if (options.override_old_needle_roughness) {
    descriptor->old_needle_roughness = std::clamp(options.old_needle_roughness, 0.02f, 1.0f);
  }
  if (options.override_young_needle_specular) {
    descriptor->young_needle_specular = std::clamp(options.young_needle_specular, 0.0f, 1.0f);
  }
  if (options.override_old_needle_specular) {
    descriptor->old_needle_specular = std::clamp(options.old_needle_specular, 0.0f, 1.0f);
  }
  if (options.override_stem_roughness) {
    descriptor->stem_roughness = std::clamp(options.stem_roughness, 0.02f, 1.0f);
  }
  if (options.override_stem_specular) {
    descriptor->stem_specular = std::clamp(options.stem_specular, 0.0f, 1.0f);
  }
  if (options.override_node_browning_strength) {
    descriptor->node_browning_strength = std::clamp(options.node_browning_strength, 0.0f, 1.0f);
  }
  if (options.override_sheath_browning_strength) {
    descriptor->sheath_browning_strength = std::clamp(options.sheath_browning_strength, 0.0f, 1.0f);
  }
  if (options.override_node_browning_radius_norm) {
    descriptor->node_browning_radius_norm = std::clamp(options.node_browning_radius_norm, 0.0f, 1.0f);
  }
  if (options.override_needle_twist_turns) {
    descriptor->needle_twist_turns = std::clamp(options.needle_twist_turns, -8.0f, 8.0f);
  }
  if (options.override_needle_edge_darkening) {
    descriptor->needle_edge_darkening = std::clamp(options.needle_edge_darkening, 0.0f, 0.75f);
  }
  if (options.override_needle_segment_count) {
    descriptor->needle_segment_count = std::clamp(options.needle_segment_count, 1, 128);
  }
  if (options.override_fascicle_sheath_length_m) {
    descriptor->fascicle_sheath_length_m.mean = std::clamp(options.fascicle_sheath_length_m, 0.0005f, 0.030f);
    descriptor->fascicle_sheath_length_m.deviation = 0.0f;
  }
  if (options.override_fascicle_sheath_width_m) {
    descriptor->fascicle_sheath_width_m.mean = std::clamp(options.fascicle_sheath_width_m, 0.0002f, 0.010f);
    descriptor->fascicle_sheath_width_m.deviation = 0.0f;
  }
  if (options.override_needle_year0_length_multiplier) {
    descriptor->needle_year0_length_multiplier = std::max(0.0f, options.needle_year0_length_multiplier);
  }
  if (options.override_needle_axial_age_span) {
    descriptor->needle_axial_age_span = std::clamp(options.needle_axial_age_span, -1.0f, 1.0f);
  }
  if (options.override_needle_axial_age_exponent) {
    descriptor->needle_axial_age_exponent = std::max(0.1f, options.needle_axial_age_exponent);
  }
}

void SetSyntheticColorModeAndRebuild(const ScotsPine::ColorMode mode,
                                     const std::vector<std::shared_ptr<ScotsPine>>& pines) {
  ScotsPine::SetGlobalColorMode(mode);
  for (const auto& pine : pines) {
    if (pine) {
      pine->RebuildGeometry();
    }
  }
}

ScotsPineCalendarSettings ResolveSyntheticCalendarSettings() {
  ScotsPineCalendarSettings settings;
  if (const auto lsystem_layer = ApplicationContext::Get().GetLayer<LSystemLayer>()) {
    settings.simulation_year = lsystem_layer->simulation_year;
    settings.simulation_day_of_year = lsystem_layer->simulation_day_of_year;
    settings.calendar_end_enabled = lsystem_layer->calendar_end_enabled;
    settings.calendar_end_year = lsystem_layer->calendar_end_year;
    settings.calendar_end_day_of_year = lsystem_layer->calendar_end_day_of_year;
  }
  settings.simulation_year = std::max(0, settings.simulation_year);
  settings.simulation_day_of_year = NormalizeScotsPineDayOfYear(settings.simulation_day_of_year);
  settings.calendar_end_year = std::max(0, settings.calendar_end_year);
  settings.calendar_end_day_of_year = std::clamp(settings.calendar_end_day_of_year, 0.0f, 364.999f);
  return settings;
}

double SyntheticCalendarAbsoluteDay(const int year, const float day_of_year) {
  return static_cast<double>(std::max(0, year)) * 365.0 + static_cast<double>(std::clamp(day_of_year, 0.0f, 364.999f));
}

void SetSyntheticCalendarFromAbsoluteDay(ScotsPineCalendarSettings& settings, double absolute_day) {
  absolute_day = std::max(0.0, absolute_day);
  settings.simulation_year = static_cast<int>(std::floor(absolute_day / 365.0));
  const double day = absolute_day - static_cast<double>(settings.simulation_year) * 365.0;
  settings.simulation_day_of_year = std::clamp(static_cast<float>(day), 0.0f, 364.999f);
}

float AdvanceSyntheticCalendarSettings(ScotsPineCalendarSettings& settings,
                                       const float requested_delta_days,
                                       bool& reached_cap) {
  reached_cap = false;
  const float safe_delta = std::max(0.0f, requested_delta_days);
  const double current = SyntheticCalendarAbsoluteDay(settings.simulation_year, settings.simulation_day_of_year);
  double target = current + static_cast<double>(safe_delta);
  const double cap = SyntheticCalendarAbsoluteDay(settings.calendar_end_year, settings.calendar_end_day_of_year);
  if (current >= cap) {
    target = cap;
    reached_cap = true;
  } else if (target >= cap) {
    target = cap;
    reached_cap = true;
  }
  SetSyntheticCalendarFromAbsoluteDay(settings, target);
  return static_cast<float>(std::max(0.0, target - current));
}

void GrowSyntheticPinesWithCalendar(const SyntheticOptions& options,
                                    const std::vector<std::shared_ptr<ScotsPine>>& pines,
                                    const std::vector<float>& baseline_target_gdds) {
  if (pines.empty()) {
    throw std::runtime_error("calendar_scene_multiplier requested, but no ScotsPine components are available.");
  }
  if (baseline_target_gdds.size() < pines.size()) {
    throw std::runtime_error("calendar_scene_multiplier requested, but baseline target GDD data is incomplete.");
  }

  ScotsPineCalendarSettings settings = ResolveSyntheticCalendarSettings();
  if (options.calendar_start_from_reset) {
    if (const auto lsystem_layer = ApplicationContext::Get().GetLayer<LSystemLayer>()) {
      settings.simulation_year = std::max(0, lsystem_layer->calendar_start_year);
      settings.simulation_day_of_year = std::clamp(lsystem_layer->calendar_start_day_of_year, 0.0f, 364.999f);
    } else {
      settings.simulation_year = 0;
      settings.simulation_day_of_year = 0.0f;
    }
    for (const auto& pine : pines) {
      if (pine) {
        ResetScotsPineCalendarState(*pine);
      }
    }
  }

  const float multiplier = std::max(0.0f, options.scene_pine_target_gdd_multiplier);
  const float step_days = std::max(1.0e-4f, options.calendar_step_days);
  constexpr int kMaxCalendarSteps = 200000;

  int steps = 0;
  bool calendar_end_reached = false;
  for (; steps < kMaxCalendarSteps; ++steps) {
    const float actual_step_days = AdvanceSyntheticCalendarSettings(settings, step_days, calendar_end_reached);
    if (actual_step_days <= 0.0f && calendar_end_reached) {
      break;
    }

    bool all_reached = true;
    for (size_t pine_index = 0; pine_index < pines.size(); ++pine_index) {
      const auto& pine = pines[pine_index];
      if (!pine) {
        continue;
      }
      const float requested_target_gdd = std::max(0.0f, baseline_target_gdds[pine_index] * multiplier);
      const auto step_result = AdvanceScotsPineCalendarStep(*pine, settings, actual_step_days, requested_target_gdd);
      all_reached = all_reached && step_result.reached_target;
    }
    if (all_reached || calendar_end_reached) {
      ++steps;
      break;
    }
  }

  if (steps >= kMaxCalendarSteps) {
    throw std::runtime_error("calendar_scene_multiplier growth did not converge within the step limit.");
  }

  std::cout << "Calendar growth finished: multiplier=" << multiplier << " step_days=" << step_days
            << " year=" << settings.simulation_year << " day=" << settings.simulation_day_of_year
            << " end_reached=" << (calendar_end_reached ? "true" : "false") << "\n";
}

using SyntheticWriteFuture = std::pair<size_t, std::future<SyntheticImageWriteResult>>;

void ApplySyntheticWriteResult(SyntheticProfileDatapoint& point, const SyntheticImageWriteResult& result) {
  point.phases.composite_ms = result.composite_ms;
  point.phases.encode_ms = result.encode_ms;
  point.phases.write_ms = result.write_ms;
  point.phases.rgb_write_ms = result.composite_ms + result.encode_ms + result.write_ms;
  point.bytes_written = result.bytes_written;
  if (!result.success) {
    point.success = false;
    point.error_message = result.error_message;
  }
}

void ResolveCompletedSyntheticWrites(std::vector<SyntheticWriteFuture>& futures,
                                     std::vector<SyntheticProfileDatapoint>& profile_datapoints,
                                     const size_t max_in_flight, const bool wait_all) {
  while (!futures.empty() && (wait_all || futures.size() >= max_in_flight)) {
    auto item = std::move(futures.front());
    futures.erase(futures.begin());
    const SyntheticImageWriteResult result = item.second.get();
    if (item.first < profile_datapoints.size()) {
      ApplySyntheticWriteResult(profile_datapoints[item.first], result);
    }
  }
}

int RunSyntheticRender(SyntheticOptions options) {
  const auto total_start_time = std::chrono::steady_clock::now();
  const auto setup_start_time = total_start_time;
  if (options.project_path.empty()) {
    options.project_path = ResolveDefaultScotsPineProjectPath(FindResourceFolder());
  } else {
    options.project_path = std::filesystem::absolute(options.project_path);
  }
  options.output_root = std::filesystem::absolute(options.output_root);
  if (!options.camera_rig_file.empty()) {
    options.camera_rig_file = std::filesystem::absolute(options.camera_rig_file);
  }
  if (!options.background_image.empty()) {
    options.background_image = std::filesystem::absolute(options.background_image);
  }
  if (!options.background_dir.empty()) {
    options.background_dir = std::filesystem::absolute(options.background_dir);
  }

  std::cout << "ScotsPine synthetic render configuration:\n"
            << "  project: " << options.project_path << "\n"
            << "  scene: " << options.scene_path << "\n"
            << "  descriptor: " << options.descriptor_path << "\n"
            << "  output_root: " << options.output_root << "\n"
            << "  output_name: " << options.output_name << "\n"
            << "  camera_rig_file: " << options.camera_rig_file << "\n"
            << "  use_scene_main_camera: " << (options.use_scene_main_camera ? "true" : "false") << "\n"
            << "  use_scene_pine_transforms: " << (options.use_scene_pine_transforms ? "true" : "false") << "\n"
            << "  scene_pine_growth_mode: " << options.scene_pine_growth_mode << "\n"
            << "  strict_parity: " << (options.strict_parity ? "true" : "false") << "\n"
            << "  render_mode: " << SyntheticRenderModeLabel(options) << "\n"
            << "  base_seed: " << options.base_seed << "\n"
            << "  sample_count: " << options.sample_count << "\n"
            << "  start_index: " << options.start_index << "\n"
            << "  worker: " << options.worker_id << "/" << options.worker_count << "\n"
            << "  frame_count: " << options.frame_count << "\n"
            << "  rgb_output_format: " << options.rgb_output_format << "\n"
            << "  plant_blur_radius_px: " << options.plant_blur_radius_px << "\n"
            << "  export_annotation_skeleton: " << (options.export_annotation_skeleton ? "true" : "false") << "\n"
            << "  export_annotation_overlay: " << (options.export_annotation_overlay ? "true" : "false") << "\n";
  const double setup_ms = SyntheticElapsedMs(setup_start_time);

  auto& app = ApplicationContext::Get();
  int load_frame_count = 0;
  while (!ProjectManager::IsProjectIdle()) {
    if (!app.Loop()) {
      throw std::runtime_error("Application terminated while waiting for project load.");
    }
    load_frame_count++;
    if (load_frame_count > 20000) {
      throw std::runtime_error("Project load timed out.");
    }
  }

  if (options.render_mode == "ray_tracing" && !Platform::RayTracingEnabled()) {
    throw std::runtime_error("render_mode=ray_tracing was requested, but ray tracing is not enabled on this device.");
  }

  const auto descriptor_start_time = std::chrono::steady_clock::now();
  std::string descriptor_error;
  auto descriptor = ResolveDescriptorAsset(options.descriptor_path, descriptor_error);
  if (!descriptor) {
    throw std::runtime_error(descriptor_error);
  }
  ApplySyntheticDescriptorOverrides(descriptor, options);

  const double descriptor_load_ms = SyntheticElapsedMs(descriptor_start_time);

  std::vector<SyntheticCameraRigView> camera_views;
  std::string camera_error;

  std::filesystem::create_directories(options.output_root);
  WriteSyntheticParityManifestJson(options.output_root / "parity_manifest.json", options);
  const auto background_paths = ResolveSyntheticBackgroundPaths(options);
  if (options.composite_background && background_paths.empty()) {
    std::cout << "Synthetic compositing requested without backgrounds; final RGB will use a black backdrop.\n";
  } else if (!background_paths.empty()) {
    std::cout << "Synthetic compositing backgrounds: " << background_paths.size() << "\n";
  }

  const auto scene_start_time = std::chrono::steady_clock::now();
  std::string scene_error;
  auto scene = ResolveSyntheticWorkingScene(options, scene_error);
  if (!scene) {
    throw std::runtime_error(scene_error.empty() ? "Failed to create synthetic scene." : scene_error);
  }
  app.Attach(scene);
  ScotsPine::SetRenderNeedlesEnabled(options.render_needles);
  const double scene_load_or_clone_ms = SyntheticElapsedMs(scene_start_time);

  std::shared_ptr<Camera> camera = nullptr;
  Entity camera_entity{};
  if (options.use_scene_main_camera) {
    camera = scene->main_camera.Get<Camera>();
    if (!camera || !scene->IsEntityValid(camera->GetOwner())) {
      throw std::runtime_error("use_scene_main_camera is enabled, but the loaded scene has no valid main camera.");
    }
    camera_entity = camera->GetOwner();
    ApplySyntheticCameraRenderMode(camera, options);
    DisableSyntheticPostProcessing(camera);
    camera->Resize(glm::uvec2(static_cast<unsigned>(options.width), static_cast<unsigned>(options.height)));
    camera_views.emplace_back(BuildSyntheticViewFromSceneMainCamera(scene, camera, options));
  } else {
    if (!LoadSyntheticCameraRigFromFile(options.camera_rig_file, options.camera_position_offset_y, camera_views,
                                        camera_error)) {
      throw std::runtime_error(camera_error);
    }

    if (!options.load_scene) {
      scene->environment.ambient_light_intensity = std::max(0.0f, options.ambient_light);
      scene->environment.background_intensity = 1.0f;
    }

    const auto light_entity = scene->CreateEntity("Synthetic Directional Light");
    if (const auto light = scene->GetOrSetPrivateComponent<DirectionalLight>(light_entity).lock()) {
      light->diffuse_brightness = std::max(0.0f, options.directional_light);
    }
    auto light_transform = scene->GetDataComponent<GlobalTransform>(light_entity);
    light_transform.SetEulerRotation(glm::radians(glm::vec3(-45.0f, 35.0f, 0.0f)));
    scene->SetDataComponent(light_entity, light_transform);

    camera_entity = scene->CreateEntity("ScotsPine Synthetic Camera");
    camera = scene->GetOrSetPrivateComponent<Camera>(camera_entity).lock();
    if (!camera) {
      throw std::runtime_error("Failed to create synthetic camera.");
    }
    scene->main_camera = camera;
    ApplySyntheticCameraRenderMode(camera, options);
    camera->camera_settings.use_clear_color = true;
    DisableSyntheticPostProcessing(camera);
  }

  std::vector<std::shared_ptr<ScotsPine>> pines;
  bool using_loaded_scene_pines = false;
  if (options.load_scene && options.use_scene_pine_transforms) {
    for (const auto& [entity, pine] : FindSyntheticScenePines(scene)) {
      (void)entity;
      pines.emplace_back(pine);
    }
    using_loaded_scene_pines = !pines.empty();
  } else if (options.load_scene) {
    for (const auto& [entity, pine] : FindSyntheticScenePines(scene)) {
      (void)entity;
      if (pine) {
        pine->ClearGeometryEntities();
      }
    }
  }
  if (pines.empty()) {
    const auto positions = BuildSyntheticTrianglePositions(options);
    for (size_t i = 0; i < positions.size(); ++i) {
      const auto entity = scene->CreateEntity("Scots Pine " + std::to_string(i));
      const auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
      if (!pine) {
        throw std::runtime_error("Failed to create ScotsPine component.");
      }
      auto transform = scene->GetDataComponent<GlobalTransform>(entity);
      transform.SetScale(glm::vec3(options.visual_scale));
      transform.SetPosition(positions[i]);
      scene->SetDataComponent(entity, transform);
      pines.emplace_back(pine);
    }
  }

  if (pines.empty()) {
    throw std::runtime_error("Synthetic renderer requires at least one ScotsPine component.");
  }

  if (options.use_scene_main_camera) {
    ApplySyntheticCameraScreenOffset(scene, camera_entity, camera, pines, options);
    camera_views.clear();
    camera_views.emplace_back(BuildSyntheticViewFromSceneMainCamera(scene, camera, options));
  }
  WriteSyntheticCameraMetadataJson(options.output_root / (options.output_name + "_camera_views.json"), camera_views);

  for (auto& pine : pines) {
    pine->descriptor_ref = descriptor;
  }
  std::vector<float> baseline_target_gdds;
  baseline_target_gdds.reserve(pines.size());
  for (const auto& pine : pines) {
    baseline_target_gdds.emplace_back(pine ? pine->target_gdd : 0.0f);
  }

  std::vector<SyntheticProfileDatapoint> profile_datapoints;
  profile_datapoints.reserve(static_cast<size_t>(options.sample_count) * static_cast<size_t>(options.frame_count) *
                             camera_views.size());
  std::vector<SyntheticWriteFuture> write_futures;
  const size_t max_write_futures = static_cast<size_t>(std::max(1, options.writer_threads));
  int datapoint_index = 0;
  int attempted_sample_count = 0;
  int successful_sample_count = 0;
  int failed_sample_count = 0;
  bool first_datapoint = true;

  for (int local_sample_index = 0; local_sample_index < options.sample_count; ++local_sample_index) {
    const auto sample_start_time = std::chrono::steady_clock::now();
    const int global_sample_index = options.start_index + local_sample_index;
    const auto sample_seeds = SyntheticSeedsForSample(options, global_sample_index);

    std::ostringstream sample_suffix;
    sample_suffix << std::setw(6) << std::setfill('0') << global_sample_index;
    const std::string sample_name =
        (options.batch_output_subdirs || options.sample_count > 1) ? options.output_name + "_" + sample_suffix.str()
                                                                   : options.output_name;
    const std::filesystem::path sample_output_root =
        (options.batch_output_subdirs || options.sample_count > 1) ? options.output_root / ("scene_" + sample_suffix.str())
                                                                   : options.output_root;
    std::filesystem::create_directories(sample_output_root);
    WriteSyntheticCameraMetadataJson(sample_output_root / (sample_name + "_camera_views.json"), camera_views);

    try {
      const auto sample_reset_start_time = std::chrono::steady_clock::now();
      for (size_t tree_index = 0; tree_index < pines.size(); ++tree_index) {
        auto& pine = pines[tree_index];
        if (!pine) {
          continue;
        }
        pine->ClearGeometryEntities();
        pine->growth_model.Reset();
        if (tree_index < baseline_target_gdds.size()) {
          pine->target_gdd = baseline_target_gdds[tree_index];
        }
        const bool allow_seed_override = !using_loaded_scene_pines || !options.preserve_scene_pine_seed;
        if (tree_index < sample_seeds.size() && !options.strict_parity && allow_seed_override) {
          pine->seed = sample_seeds[tree_index];
        }
      }
      const double sample_reset_ms = SyntheticElapsedMs(sample_reset_start_time);

      for (int frame_index = 0; frame_index < options.frame_count; ++frame_index) {
        const float t = options.frame_count <= 1 ? 1.0f : static_cast<float>(frame_index) / (options.frame_count - 1);
        const bool calendar_scene_multiplier_mode = options.scene_pine_growth_mode == "calendar_scene_multiplier";

        const auto growth_start_time = std::chrono::steady_clock::now();
        if (calendar_scene_multiplier_mode) {
          if (!using_loaded_scene_pines) {
            throw std::runtime_error("calendar_scene_multiplier requires loaded scene ScotsPine components.");
          }
          GrowSyntheticPinesWithCalendar(options, pines, baseline_target_gdds);
        }

        for (size_t tree_index = 0; tree_index < std::min<size_t>(3, pines.size()); ++tree_index) {
          auto& pine = pines[tree_index];
          if (!calendar_scene_multiplier_mode) {
            if (using_loaded_scene_pines && options.override_scene_pine_target_gdd_multiplier) {
              const float multiplier = std::max(0.0f, options.scene_pine_target_gdd_multiplier);
              pine->target_gdd = std::max(0.0f, baseline_target_gdds[tree_index] * multiplier);
            } else if (!options.use_scene_pine_growth) {
              pine->target_gdd = std::max(0.0f, options.max_target_gdd * t);
            }
            pine->ClearGeometryEntities();
            pine->growth_model.Reset();
            pine->GrowToTargetGDD(options.uncapped_growth);
          }
        }
        const double frame_growth_ms = SyntheticElapsedMs(growth_start_time);

        const auto graph_export_start_time = std::chrono::steady_clock::now();
        for (size_t tree_index = 0; tree_index < std::min<size_t>(3, pines.size()); ++tree_index) {
          auto& pine = pines[tree_index];
          std::ostringstream tree_stem;
          tree_stem << sample_name << "_f" << std::setw(4) << std::setfill('0') << frame_index << "_tree"
                    << tree_index;
          const auto prefix = sample_output_root / tree_stem.str();
          if (options.export_node_graph) {
            pine->ExportNodeGraph(prefix.string() + "_node.yaml");
          }
          if (options.export_flow_graph) {
            pine->ExportFlowGraph(prefix.string() + "_flow.yaml");
          }
          if (options.export_needle_skeleton) {
            pine->ExportNeedleSkeleton(prefix.string() + "_needle_skeleton.yaml");
          }
        }
        std::vector<glm::vec3> annotation_points;
        if (options.export_annotation_skeleton || options.export_annotation_overlay) {
          std::ostringstream frame_stem;
          frame_stem << sample_name << "_f" << std::setw(4) << std::setfill('0') << frame_index;
          if (options.export_annotation_skeleton) {
            WriteSyntheticAnnotationSkeletonJson(sample_output_root / (frame_stem.str() + "_annotation_skeleton.json"),
                                                 sample_name, global_sample_index, local_sample_index, frame_index,
                                                 sample_seeds, pines);
          }
          if (options.export_annotation_overlay) {
            for (size_t tree_index = 0; tree_index < std::min<size_t>(3, pines.size()); ++tree_index) {
              if (pines[tree_index]) {
                pines[tree_index]->CollectAnnotationSkeletonPoints(annotation_points);
              }
            }
          }
        }
        const double frame_graph_export_ms = SyntheticElapsedMs(graph_export_start_time);
        const double view_count = static_cast<double>(std::max<size_t>(1, camera_views.size()));

        for (size_t view_index = 0; view_index < camera_views.size(); ++view_index) {
          const auto datapoint_start_time = std::chrono::steady_clock::now();
          SyntheticProfileDatapoint profile_point;
          profile_point.datapoint_index = datapoint_index++;
          profile_point.global_sample_index = global_sample_index;
          profile_point.local_sample_index = local_sample_index;
          profile_point.frame_index = frame_index;
          profile_point.view_index = static_cast<int>(view_index);
          profile_point.seeds = sample_seeds;
          profile_point.warmup = profile_point.datapoint_index < options.profile_warmup_datapoints;

          const auto& view = camera_views[view_index];
          profile_point.view_label = view.label;
          profile_point.width = view.width;
          profile_point.height = view.height;
          profile_point.phases.scene_reset_ms = sample_reset_ms / (static_cast<double>(options.frame_count) * view_count);
          profile_point.phases.growth_ms = frame_growth_ms / view_count;
          profile_point.phases.graph_export_ms = frame_graph_export_ms / view_count;
          if (first_datapoint) {
            profile_point.phases.setup_ms = setup_ms;
            profile_point.phases.scene_load_or_clone_ms = scene_load_or_clone_ms;
            profile_point.phases.descriptor_load_ms = descriptor_load_ms;
            first_datapoint = false;
          }

          const auto view_setup_start_time = std::chrono::steady_clock::now();
          if (!options.use_scene_main_camera) {
            auto camera_transform = scene->GetDataComponent<GlobalTransform>(camera_entity);
            camera_transform.SetPosition(view.position);
            camera_transform.SetRotation(BuildSyntheticCameraRotation(view.forward, view.up));
            scene->SetDataComponent(camera_entity, camera_transform);
          }

          camera->Resize(glm::uvec2(static_cast<unsigned>(view.width), static_cast<unsigned>(view.height)));
          camera->camera_settings.near_distance = view.near_distance;
          camera->camera_settings.far_distance = view.far_distance;
          camera->camera_settings.fov = view.fov_deg;
          camera->camera_settings.use_clear_color = true;
          camera->camera_settings.clear_color =
              options.transparent_bg ? glm::vec4(0.0f, 0.0f, 0.0f, 0.0f) : glm::vec4(0.97f, 0.97f, 0.97f, 1.0f);

          SetSyntheticColorModeAndRebuild(ScotsPine::ColorMode::Shaded, pines);
          profile_point.phases.view_setup_ms = SyntheticElapsedMs(view_setup_start_time);

          const auto rgb_render_start_time = std::chrono::steady_clock::now();
          RenderSyntheticCamera(app, camera, options);
          profile_point.phases.rgb_render_ms = SyntheticElapsedMs(rgb_render_start_time);

          std::ostringstream view_stem;
          view_stem << sample_name << "_f" << std::setw(4) << std::setfill('0') << frame_index << "_v"
                    << std::setw(2) << std::setfill('0') << view_index << "_"
                    << ScotsPineSyntheticSanitizeViewLabel(view.label);
          const auto prefix = sample_output_root / view_stem.str();
          const std::string rgb_extension = options.rgb_output_format == "png" ? ".png" : ".jpg";
          profile_point.rgb_path = prefix.string() + "_composited" + rgb_extension;
          profile_point.foreground_mask_path = prefix.string() + "_foreground_mask.png";
          profile_point.raw_rgba_path = prefix.string() + "_rgba.png";
          if (options.export_annotation_overlay) {
            profile_point.annotation_overlay_path = prefix.string() + "_annotation_overlay.png";
          }
          profile_point.background_path =
              SelectSyntheticBackgroundPath(background_paths, global_sample_index, frame_index, static_cast<int>(view_index));

          std::vector<uint8_t> rgba_pixels;
          int readback_width = 0;
          int readback_height = 0;
          const auto readback_start_time = std::chrono::steady_clock::now();
          DownloadSyntheticColorRgba8(camera->GetRenderTexture(), rgba_pixels, readback_width, readback_height, true);
          profile_point.phases.gpu_readback_ms = SyntheticElapsedMs(readback_start_time);
          profile_point.width = readback_width;
          profile_point.height = readback_height;

          SyntheticImageWriteTask write_task;
          write_task.width = readback_width;
          write_task.height = readback_height;
          write_task.rgba = std::move(rgba_pixels);
          write_task.rgb_path = profile_point.rgb_path;
          write_task.foreground_mask_path = profile_point.foreground_mask_path;
          write_task.raw_rgba_path = profile_point.raw_rgba_path;
          write_task.annotation_overlay_path = profile_point.annotation_overlay_path;
          write_task.background_path = profile_point.background_path;
          if (options.export_annotation_overlay) {
            const auto camera_transform = scene->GetDataComponent<GlobalTransform>(camera_entity);
            write_task.annotation_overlay_pixels =
                ProjectSyntheticAnnotationOverlayPixels(annotation_points, camera_transform, *camera, readback_width,
                                                        readback_height);
          }
          write_task.rgb_output_format = options.rgb_output_format;
          write_task.jpg_quality = options.jpg_quality;
          write_task.png_compression_level = options.png_compression_level;
          write_task.plant_blur_radius_px = options.plant_blur_radius_px;
          write_task.composite_background = options.composite_background;
          write_task.write_foreground_mask = options.write_foreground_mask;
          write_task.write_raw_rgba = options.write_raw_rgba;
          write_task.write_annotation_overlay = options.export_annotation_overlay;

          const size_t profile_index = profile_datapoints.size();
          profile_datapoints.emplace_back(profile_point);
          write_futures.emplace_back(profile_index,
                                     std::async(std::launch::async, WriteSyntheticImageOutputs, std::move(write_task)));
          ResolveCompletedSyntheticWrites(write_futures, profile_datapoints, max_write_futures, false);

          if (options.export_depth) {
            const auto depth_start_time = std::chrono::steady_clock::now();
            if (options.render_mode == "ray_tracing") {
              SetSyntheticColorModeAndRebuild(ScotsPine::ColorMode::Shaded, pines);
              RenderSyntheticCameraRaster(app, camera);
            }
            camera->GetRenderTexture()->StoreLinearDepthToPng(prefix.string() + "_depth.png",
                                                              camera->camera_settings.near_distance,
                                                              camera->camera_settings.far_distance,
                                                              camera->camera_settings.far_distance, view.width,
                                                              view.height, options.png_compression_level);
            profile_datapoints[profile_index].phases.depth_render_write_ms = SyntheticElapsedMs(depth_start_time);
          }

          if (options.export_instance_mask) {
            const auto instance_start_time = std::chrono::steady_clock::now();
            camera->camera_settings.clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);
            SetSyntheticColorModeAndRebuild(ScotsPine::ColorMode::ByInstance, pines);
            app.Loop();
            RenderSyntheticCameraRaster(app, camera);
            camera->GetRenderTexture()->StoreToPng(prefix.string() + "_instance_mask.png", view.width, view.height,
                                                   options.png_compression_level);
            SetSyntheticColorModeAndRebuild(ScotsPine::ColorMode::Shaded, pines);
            profile_datapoints[profile_index].phases.instance_mask_render_write_ms =
                SyntheticElapsedMs(instance_start_time);
          }

          if (options.export_synthetic_labels) {
            const auto label_start_time = std::chrono::steady_clock::now();
            camera->camera_settings.clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);
            SetSyntheticColorModeAndRebuild(ScotsPine::ColorMode::SyntheticOrganLabels, pines);
            app.Loop();
            RenderSyntheticCameraRaster(app, camera);
            std::vector<uint8_t> label_rgba;
            int label_width = 0;
            int label_height = 0;
            DownloadSyntheticColorRgba8(camera->GetRenderTexture(), label_rgba, label_width, label_height, true);
            const auto label_rgb = QuantizeSyntheticLabelRgb(label_rgba, label_width, label_height);
            const auto label_mask = BuildSyntheticForegroundMaskFromLabelRgb(label_rgb, label_width, label_height);
            const auto label_image = EncodeSyntheticPng(label_rgb, label_width, label_height, 3,
                                                        options.png_compression_level);
            const auto label_mask_image = EncodeSyntheticPng(label_mask, label_width, label_height, 1,
                                                             options.png_compression_level);
            ResolveCompletedSyntheticWrites(write_futures, profile_datapoints, max_write_futures, true);
            WriteSyntheticBinaryFile(prefix.string() + "_synthetic_labels.png", label_image);
            WriteSyntheticBinaryFile(profile_datapoints[profile_index].foreground_mask_path, label_mask_image);
            profile_datapoints[profile_index].bytes_written += label_image.bytes.size() + label_mask_image.bytes.size();
            SetSyntheticColorModeAndRebuild(ScotsPine::ColorMode::Shaded, pines);
            profile_datapoints[profile_index].phases.label_render_write_ms = SyntheticElapsedMs(label_start_time);
          }

          ApplySyntheticCameraRenderMode(camera, options);
          profile_datapoints[profile_index].phases.total_engine_ms =
              std::max(profile_datapoints[profile_index].phases.total_engine_ms, SyntheticElapsedMs(datapoint_start_time));
        }
      }
      ResolveCompletedSyntheticWrites(write_futures, profile_datapoints, max_write_futures, true);
      attempted_sample_count++;
      const bool sample_success = std::any_of(profile_datapoints.begin(), profile_datapoints.end(),
                                             [local_sample_index](const SyntheticProfileDatapoint& point) {
                                               return point.local_sample_index == local_sample_index && point.success;
                                             });
      if (sample_success) {
        successful_sample_count++;
      } else {
        failed_sample_count++;
      }
      const double sample_ms = SyntheticElapsedMs(sample_start_time);
      const double elapsed_ms = SyntheticElapsedMs(total_start_time);
      const double samples_per_hour =
          elapsed_ms > 0.0 ? 3600000.0 * static_cast<double>(attempted_sample_count) / elapsed_ms : 0.0;
      std::cout << "[scots-pine-synthetic] sample " << (local_sample_index + 1) << "/" << options.sample_count
                << " global=" << global_sample_index << " status=" << (sample_success ? "ok" : "failed")
                << " ok=" << successful_sample_count << " failed=" << failed_sample_count
                << " sample_ms=" << std::fixed << std::setprecision(1) << sample_ms
                << " throughput=" << std::setprecision(1) << samples_per_hour << "/h" << std::endl;
    } catch (const std::exception& e) {
      SyntheticProfileDatapoint failed_point;
      failed_point.datapoint_index = datapoint_index++;
      failed_point.global_sample_index = global_sample_index;
      failed_point.local_sample_index = local_sample_index;
      failed_point.seeds = sample_seeds;
      failed_point.success = false;
      failed_point.error_message = e.what();
      failed_point.warmup = failed_point.datapoint_index < options.profile_warmup_datapoints;
      profile_datapoints.emplace_back(failed_point);
      attempted_sample_count++;
      failed_sample_count++;
      const double sample_ms = SyntheticElapsedMs(sample_start_time);
      const double elapsed_ms = SyntheticElapsedMs(total_start_time);
      const double samples_per_hour =
          elapsed_ms > 0.0 ? 3600000.0 * static_cast<double>(attempted_sample_count) / elapsed_ms : 0.0;
      std::cout << "[scots-pine-synthetic] sample " << (local_sample_index + 1) << "/" << options.sample_count
                << " global=" << global_sample_index << " status=failed"
                << " ok=" << successful_sample_count << " failed=" << failed_sample_count
                << " sample_ms=" << std::fixed << std::setprecision(1) << sample_ms
                << " throughput=" << std::setprecision(1) << samples_per_hour << "/h" << std::endl;
      if (!options.keep_going) {
        throw;
      }
      std::cout << "Synthetic sample failed and will be skipped: sample=" << global_sample_index
                << " error=" << e.what() << "\n";
    }
  }

  ResolveCompletedSyntheticWrites(write_futures, profile_datapoints, max_write_futures, true);
  for (auto& point : profile_datapoints) {
    const auto& p = point.phases;
    point.phases.total_engine_ms =
        p.setup_ms + p.scene_load_or_clone_ms + p.scene_reset_ms + p.descriptor_load_ms + p.growth_ms +
        p.graph_export_ms + p.view_setup_ms + p.rgb_render_ms + p.gpu_readback_ms + p.rgb_write_ms +
        p.depth_render_write_ms + p.instance_mask_render_write_ms + p.label_render_write_ms;
  }

  ScotsPine::SetGlobalColorMode(ScotsPine::ColorMode::Shaded);
  app.Loop();
  const double total_engine_ms = SyntheticElapsedMs(total_start_time);
  WriteSyntheticProfileJson(options.output_root / "render_profile.json", options, profile_datapoints, total_engine_ms);
  WriteSyntheticProfileCsv(options.output_root / "render_profile.csv", profile_datapoints);
  const int success_count = static_cast<int>(std::count_if(profile_datapoints.begin(), profile_datapoints.end(),
                                                           [](const SyntheticProfileDatapoint& point) {
                                                             return point.success;
                                                           }));
  const int failure_count = static_cast<int>(std::count_if(profile_datapoints.begin(), profile_datapoints.end(),
                                                           [](const SyntheticProfileDatapoint& point) {
                                                             return !point.success;
                                                           }));
  if (!options.keep_going && failure_count > 0) {
    return 1;
  }
  return success_count > 0 ? 0 : 1;
}

}  // namespace

EVOENGINE_PACKAGE_EXPORT int EvoEngineLSystemRunScotsPineSyntheticRender(const char* options_file_path) {
  if (!options_file_path || std::strlen(options_file_path) == 0) {
    EVOENGINE_ERROR("Scots pine synthetic renderer requires a non-empty options file path.")
    return 1;
  }

  SyntheticOptions options;
  std::string load_error;
  if (!LoadSyntheticOptionsFromFile(std::filesystem::path(options_file_path), options, load_error)) {
    EVOENGINE_ERROR("Failed to load Scots pine synthetic options: " + load_error)
    return 1;
  }

  try {
    return RunSyntheticRender(options);
  } catch (const std::exception& e) {
    EVOENGINE_ERROR(std::string("Scots pine synthetic render failed: ") + e.what())
    return 1;
  }
}
