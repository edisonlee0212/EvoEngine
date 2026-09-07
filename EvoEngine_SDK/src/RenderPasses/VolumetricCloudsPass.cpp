#include "RenderPasses/VolumetricCloudsPass.hpp"

#include "Camera.hpp"
#include "ComputePipeline.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "RenderTexture.hpp"

#include <array>
#include <unordered_map>

using namespace evo_engine;

namespace {
constexpr uint32_t kCloudBaseShapeResolution = 64;
constexpr uint32_t kCloudDetailErosionResolution = 32;
constexpr uint32_t kCloudWeatherResolution = 128;
constexpr uint32_t kCloudWeatherTexelCount = kCloudWeatherResolution * kCloudWeatherResolution;
constexpr uint32_t kCloudCurlResolution = 128;
constexpr uint32_t kCloudCurlTexelCount = kCloudCurlResolution * kCloudCurlResolution;
constexpr int kInputIsRayHitDistanceFlag = 1 << 0;
constexpr int kCloudShadowsEnabledFlag = 1 << 1;
constexpr int kCloudShadowStepShift = 2;

struct CloudNoiseResources {
  std::shared_ptr<Image> base_shape_image;
  std::shared_ptr<ImageView> base_shape_view;
  std::shared_ptr<Image> detail_erosion_image;
  std::shared_ptr<ImageView> detail_erosion_view;
  std::shared_ptr<Image> weather_coverage_image;
  std::shared_ptr<ImageView> weather_coverage_view;
  std::shared_ptr<Image> curl_noise_image;
  std::shared_ptr<ImageView> curl_noise_view;
  std::shared_ptr<Sampler> sampler;
};

struct VolumetricCloudHistoryResources {
  VkExtent3D extent{};
  std::array<std::shared_ptr<Image>, 2> accumulation_images{};
  std::array<std::shared_ptr<ImageView>, 2> accumulation_views{};
  std::array<std::shared_ptr<Image>, 2> transmittance_images{};
  std::array<std::shared_ptr<ImageView>, 2> transmittance_views{};
  uint32_t frame_index = 0;
  bool valid = false;
};

uint32_t HashNoiseCoordinate(uint32_t x, uint32_t y, uint32_t z, uint32_t seed) {
  uint32_t value = x * 73856093u ^ y * 19349663u ^ z * 83492791u ^ seed * 2654435761u;
  value ^= value >> 16u;
  value *= 2246822519u;
  value ^= value >> 13u;
  value *= 3266489917u;
  value ^= value >> 16u;
  return value;
}

float HashNoise01(const uint32_t x, const uint32_t y, const uint32_t z, const uint32_t seed) {
  return static_cast<float>(HashNoiseCoordinate(x, y, z, seed) & 0x00ffffffu) / static_cast<float>(0x00ffffffu);
}

float SmoothStep(const float value) {
  return value * value * (3.0f - 2.0f * value);
}

float WrappedValueNoise(const glm::vec3& position, const uint32_t period, const uint32_t seed) {
  const auto period_vector = glm::vec3(static_cast<float>(period));
  const auto wrapped_position = glm::mod(glm::mod(position, period_vector) + period_vector, period_vector);
  const auto cell = glm::floor(wrapped_position);
  const auto local = wrapped_position - cell;
  const glm::vec3 fade = {SmoothStep(local.x), SmoothStep(local.y), SmoothStep(local.z)};
  const auto x0 = static_cast<uint32_t>(cell.x) % period;
  const auto y0 = static_cast<uint32_t>(cell.y) % period;
  const auto z0 = static_cast<uint32_t>(cell.z) % period;
  const auto x1 = (x0 + 1u) % period;
  const auto y1 = (y0 + 1u) % period;
  const auto z1 = (z0 + 1u) % period;

  const float n000 = HashNoise01(x0, y0, z0, seed);
  const float n100 = HashNoise01(x1, y0, z0, seed);
  const float n010 = HashNoise01(x0, y1, z0, seed);
  const float n110 = HashNoise01(x1, y1, z0, seed);
  const float n001 = HashNoise01(x0, y0, z1, seed);
  const float n101 = HashNoise01(x1, y0, z1, seed);
  const float n011 = HashNoise01(x0, y1, z1, seed);
  const float n111 = HashNoise01(x1, y1, z1, seed);

  const float nx00 = glm::mix(n000, n100, fade.x);
  const float nx10 = glm::mix(n010, n110, fade.x);
  const float nx01 = glm::mix(n001, n101, fade.x);
  const float nx11 = glm::mix(n011, n111, fade.x);
  const float nxy0 = glm::mix(nx00, nx10, fade.y);
  const float nxy1 = glm::mix(nx01, nx11, fade.y);
  return glm::mix(nxy0, nxy1, fade.z);
}

float WrappedValueFbm(glm::vec3 position, const uint32_t seed, const uint32_t base_period) {
  float value = 0.0f;
  float amplitude = 0.5f;
  float amplitude_sum = 0.0f;
  uint32_t period = base_period;
  for (int octave_index = 0; octave_index < 5; ++octave_index) {
    value += WrappedValueNoise(position, period, seed + static_cast<uint32_t>(octave_index) * 31u) * amplitude;
    amplitude_sum += amplitude;
    position *= 2.0f;
    period = glm::max(2u, period / 2u);
    amplitude *= 0.5f;
  }
  return amplitude_sum > 0.0f ? value / amplitude_sum : 0.0f;
}

uint32_t WrapNoiseIndex(const int value, const uint32_t period) {
  const int wrapped = value % static_cast<int>(period);
  return static_cast<uint32_t>(wrapped < 0 ? wrapped + static_cast<int>(period) : wrapped);
}

glm::vec3 WrappedCellFeaturePoint(const glm::ivec3& cell, const uint32_t period, const uint32_t seed) {
  const glm::uvec3 wrapped(WrapNoiseIndex(cell.x, period), WrapNoiseIndex(cell.y, period),
                           WrapNoiseIndex(cell.z, period));
  return glm::vec3(wrapped) + glm::vec3(HashNoise01(wrapped.x, wrapped.y, wrapped.z, seed),
                                        HashNoise01(wrapped.x, wrapped.y, wrapped.z, seed + 17u),
                                        HashNoise01(wrapped.x, wrapped.y, wrapped.z, seed + 43u));
}

float WrappedWorleyNoise(const glm::vec3& position, const uint32_t period, const uint32_t seed) {
  const auto period_vector = glm::vec3(static_cast<float>(period));
  const auto wrapped_position = glm::mod(glm::mod(position, period_vector) + period_vector, period_vector);
  const auto base_cell = glm::ivec3(glm::floor(wrapped_position));
  float min_distance = 1000000.0f;
  for (int z = -1; z <= 1; ++z) {
    for (int y = -1; y <= 1; ++y) {
      for (int x = -1; x <= 1; ++x) {
        const auto feature = WrappedCellFeaturePoint(base_cell + glm::ivec3(x, y, z), period, seed);
        glm::vec3 delta = glm::abs(feature - wrapped_position);
        delta = glm::min(delta, period_vector - delta);
        min_distance = glm::min(min_distance, glm::length(delta));
      }
    }
  }
  return glm::clamp(min_distance / 1.7320508f, 0.0f, 1.0f);
}

std::vector<uint8_t> BuildBaseShapeNoiseBytes() {
  std::vector<uint8_t> bytes(kCloudBaseShapeResolution * kCloudBaseShapeResolution * kCloudBaseShapeResolution * 4u);
  size_t output_index = 0;
  for (uint32_t z = 0; z < kCloudBaseShapeResolution; ++z) {
    for (uint32_t y = 0; y < kCloudBaseShapeResolution; ++y) {
      for (uint32_t x = 0; x < kCloudBaseShapeResolution; ++x) {
        const glm::vec3 position(x, y, z);
        const float billow = WrappedValueFbm(position, 11u, kCloudBaseShapeResolution);
        const float cellular = 1.0f - WrappedWorleyNoise(position * 0.25f, 8u, 29u);
        const float low_frequency = WrappedValueNoise(position * 0.125f, 4u, 53u);
        const float shape = glm::clamp(billow * 0.55f + cellular * 0.35f + low_frequency * 0.10f, 0.0f, 1.0f);
        bytes[output_index++] = static_cast<uint8_t>(glm::round(shape * 255.0f));
        bytes[output_index++] = static_cast<uint8_t>(glm::round(billow * 255.0f));
        bytes[output_index++] = static_cast<uint8_t>(glm::round(cellular * 255.0f));
        bytes[output_index++] = static_cast<uint8_t>(glm::round(low_frequency * 255.0f));
      }
    }
  }
  return bytes;
}

std::vector<uint8_t> BuildDetailErosionNoiseBytes() {
  std::vector<uint8_t> bytes(kCloudDetailErosionResolution * kCloudDetailErosionResolution *
                             kCloudDetailErosionResolution * 4u);
  size_t output_index = 0;
  for (uint32_t z = 0; z < kCloudDetailErosionResolution; ++z) {
    for (uint32_t y = 0; y < kCloudDetailErosionResolution; ++y) {
      for (uint32_t x = 0; x < kCloudDetailErosionResolution; ++x) {
        const glm::vec3 position(x, y, z);
        const float fine_worley = WrappedWorleyNoise(position * 0.75f, 24u, 71u);
        const float medium_worley = WrappedWorleyNoise(position * 0.5f, 16u, 97u);
        const float coarse_worley = WrappedWorleyNoise(position * 0.25f, 8u, 131u);
        const float detail = WrappedValueFbm(position * 2.0f, 163u, kCloudDetailErosionResolution);
        bytes[output_index++] = static_cast<uint8_t>(glm::round((1.0f - fine_worley) * 255.0f));
        bytes[output_index++] = static_cast<uint8_t>(glm::round((1.0f - medium_worley) * 255.0f));
        bytes[output_index++] = static_cast<uint8_t>(glm::round((1.0f - coarse_worley) * 255.0f));
        bytes[output_index++] = static_cast<uint8_t>(glm::round(detail * 255.0f));
      }
    }
  }
  return bytes;
}

std::vector<uint8_t> BuildWeatherCoverageNoiseBytes() {
  std::vector<uint8_t> bytes(kCloudWeatherTexelCount * 4u);
  size_t output_index = 0;
  for (uint32_t y = 0; y < kCloudWeatherResolution; ++y) {
    for (uint32_t x = 0; x < kCloudWeatherResolution; ++x) {
      const glm::vec3 position(static_cast<float>(x) * 0.25f, static_cast<float>(y) * 0.25f, 19.0f);
      const float coverage = WrappedValueFbm(position, 211u, kCloudWeatherResolution);
      const float cloud_type = WrappedValueNoise(position * 0.125f + glm::vec3(7.0f, 0.0f, 13.0f), 16u, 241u);
      const float density_bias = WrappedValueNoise(position * 0.0625f + glm::vec3(0.0f, 5.0f, 23.0f), 8u, 277u);
      bytes[output_index++] = static_cast<uint8_t>(glm::round(coverage * 255.0f));
      bytes[output_index++] = static_cast<uint8_t>(glm::round(cloud_type * 255.0f));
      bytes[output_index++] = static_cast<uint8_t>(glm::round(density_bias * 255.0f));
      bytes[output_index++] = 255u;
    }
  }
  return bytes;
}

std::vector<uint8_t> BuildCurlNoiseBytes() {
  std::vector<uint8_t> bytes(kCloudCurlTexelCount * 4u);
  size_t output_index = 0;
  for (uint32_t y = 0; y < kCloudCurlResolution; ++y) {
    for (uint32_t x = 0; x < kCloudCurlResolution; ++x) {
      const glm::vec3 position(static_cast<float>(x) * 0.25f, static_cast<float>(y) * 0.25f, 37.0f);
      const float dx = WrappedValueFbm(position + glm::vec3(1.0f, 0.0f, 0.0f), 311u, kCloudCurlResolution) -
                       WrappedValueFbm(position - glm::vec3(1.0f, 0.0f, 0.0f), 311u, kCloudCurlResolution);
      const float dy = WrappedValueFbm(position + glm::vec3(0.0f, 1.0f, 0.0f), 311u, kCloudCurlResolution) -
                       WrappedValueFbm(position - glm::vec3(0.0f, 1.0f, 0.0f), 311u, kCloudCurlResolution);
      const glm::vec2 curl =
          glm::length(glm::vec2(dy, -dx)) > 0.0001f ? glm::normalize(glm::vec2(dy, -dx)) : glm::vec2(0.0f);
      bytes[output_index++] = static_cast<uint8_t>(glm::round((curl.x * 0.5f + 0.5f) * 255.0f));
      bytes[output_index++] = 128u;
      bytes[output_index++] = static_cast<uint8_t>(glm::round((curl.y * 0.5f + 0.5f) * 255.0f));
      bytes[output_index++] = 255u;
    }
  }
  return bytes;
}

std::shared_ptr<Image> CreateCloudNoiseImage(const VkImageType image_type, const VkExtent3D extent) {
  VkImageCreateInfo image_info{};
  image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  image_info.imageType = image_type;
  image_info.extent = extent;
  image_info.mipLevels = 1;
  image_info.arrayLayers = 1;
  image_info.format = VK_FORMAT_R8G8B8A8_UNORM;
  image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
  image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  image_info.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
  image_info.samples = VK_SAMPLE_COUNT_1_BIT;
  image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  return std::make_shared<Image>(image_info);
}

std::shared_ptr<Image> CreateCloudHistoryImage(const VkFormat format, const VkExtent3D extent) {
  VkImageCreateInfo image_info{};
  image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  image_info.imageType = VK_IMAGE_TYPE_2D;
  image_info.extent = extent;
  image_info.mipLevels = 1;
  image_info.arrayLayers = 1;
  image_info.format = format;
  image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
  image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  image_info.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  image_info.samples = VK_SAMPLE_COUNT_1_BIT;
  image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  return std::make_shared<Image>(image_info);
}

std::shared_ptr<Image> CreateCloudNoiseVolumeImage(const uint32_t resolution) {
  return CreateCloudNoiseImage(VK_IMAGE_TYPE_3D, {resolution, resolution, resolution});
}

std::shared_ptr<Image> CreateCloudWeatherImage() {
  return CreateCloudNoiseImage(VK_IMAGE_TYPE_2D, {kCloudWeatherResolution, kCloudWeatherResolution, 1u});
}

std::shared_ptr<Image> CreateCloudCurlNoiseImage() {
  return CreateCloudNoiseImage(VK_IMAGE_TYPE_2D, {kCloudCurlResolution, kCloudCurlResolution, 1u});
}

std::shared_ptr<ImageView> CreateCloudNoiseImageView(const std::shared_ptr<Image>& image,
                                                     const VkImageViewType view_type) {
  VkImageViewCreateInfo view_info{};
  view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  view_info.image = image->GetVkImage();
  view_info.viewType = view_type;
  view_info.format = image->GetFormat();
  view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  view_info.subresourceRange.baseMipLevel = 0;
  view_info.subresourceRange.levelCount = 1;
  view_info.subresourceRange.baseArrayLayer = 0;
  view_info.subresourceRange.layerCount = 1;
  return std::make_shared<ImageView>(view_info, image);
}

std::shared_ptr<ImageView> CreateCloudNoiseVolumeImageView(const std::shared_ptr<Image>& image) {
  return CreateCloudNoiseImageView(image, VK_IMAGE_VIEW_TYPE_3D);
}

std::shared_ptr<ImageView> CreateCloudWeatherImageView(const std::shared_ptr<Image>& image) {
  return CreateCloudNoiseImageView(image, VK_IMAGE_VIEW_TYPE_2D);
}

std::shared_ptr<ImageView> CreateCloudCurlNoiseImageView(const std::shared_ptr<Image>& image) {
  return CreateCloudNoiseImageView(image, VK_IMAGE_VIEW_TYPE_2D);
}

std::shared_ptr<Sampler> CreateCloudNoiseSampler() {
  VkSamplerCreateInfo sampler_info{};
  sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  sampler_info.magFilter = VK_FILTER_LINEAR;
  sampler_info.minFilter = VK_FILTER_LINEAR;
  sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  sampler_info.anisotropyEnable = VK_FALSE;
  sampler_info.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
  sampler_info.unnormalizedCoordinates = VK_FALSE;
  sampler_info.compareEnable = VK_FALSE;
  sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
  sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
  sampler_info.minLod = 0.0f;
  sampler_info.maxLod = 1.0f;
  sampler_info.mipLodBias = 0.0f;
  return std::make_shared<Sampler>(sampler_info);
}

void UploadCloudNoiseImage(const std::shared_ptr<Image>& image, const std::vector<uint8_t>& bytes) {
  Buffer staging_buffer(bytes.size(), false);
  staging_buffer.UploadData(bytes.size(), bytes.data());
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    image->CopyFromBuffer(vk_command_buffer, staging_buffer.GetVkBuffer());
    image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
  });
}

CloudNoiseResources& GetCloudNoiseResources() {
  static CloudNoiseResources resources;
  if (!resources.base_shape_image && Platform::Initialized()) {
    resources.base_shape_image = CreateCloudNoiseVolumeImage(kCloudBaseShapeResolution);
    resources.detail_erosion_image = CreateCloudNoiseVolumeImage(kCloudDetailErosionResolution);
    resources.weather_coverage_image = CreateCloudWeatherImage();
    resources.curl_noise_image = CreateCloudCurlNoiseImage();
    UploadCloudNoiseImage(resources.base_shape_image, BuildBaseShapeNoiseBytes());
    UploadCloudNoiseImage(resources.detail_erosion_image, BuildDetailErosionNoiseBytes());
    UploadCloudNoiseImage(resources.weather_coverage_image, BuildWeatherCoverageNoiseBytes());
    UploadCloudNoiseImage(resources.curl_noise_image, BuildCurlNoiseBytes());
    resources.base_shape_view = CreateCloudNoiseVolumeImageView(resources.base_shape_image);
    resources.detail_erosion_view = CreateCloudNoiseVolumeImageView(resources.detail_erosion_image);
    resources.weather_coverage_view = CreateCloudWeatherImageView(resources.weather_coverage_image);
    resources.curl_noise_view = CreateCloudCurlNoiseImageView(resources.curl_noise_image);
    resources.sampler = CreateCloudNoiseSampler();
  }
  return resources;
}

VolumetricCloudHistoryResources& GetCloudHistoryResources(const uint64_t camera_handle, const VkExtent3D extent) {
  static std::unordered_map<uint64_t, VolumetricCloudHistoryResources> history_resources;
  auto& resources = history_resources[camera_handle];
  if (resources.extent.width == extent.width && resources.extent.height == extent.height &&
      resources.extent.depth == extent.depth && resources.accumulation_views[0] && resources.accumulation_views[1] &&
      resources.transmittance_views[0] && resources.transmittance_views[1]) {
    return resources;
  }

  resources = {};
  resources.extent = extent;
  for (size_t image_index = 0; image_index < resources.accumulation_images.size(); ++image_index) {
    resources.accumulation_images[image_index] = CreateCloudHistoryImage(VK_FORMAT_R16G16B16A16_SFLOAT, extent);
    resources.accumulation_views[image_index] =
        CreateCloudNoiseImageView(resources.accumulation_images[image_index], VK_IMAGE_VIEW_TYPE_2D);
    resources.transmittance_images[image_index] = CreateCloudHistoryImage(VK_FORMAT_R16_SFLOAT, extent);
    resources.transmittance_views[image_index] =
        CreateCloudNoiseImageView(resources.transmittance_images[image_index], VK_IMAGE_VIEW_TYPE_2D);
  }
  return resources;
}

VolumetricCloudsPushConstant CreatePushConstant(const VolumetricCloudsPass::Parameters& parameters) {
  VolumetricCloudSettings settings = parameters.settings;
  settings.ClampSettings();

  VolumetricCloudsPushConstant push_constant{};
  push_constant.altitude_coverage_density = {settings.bottom_altitude, settings.top_altitude, settings.coverage,
                                             settings.density};
  push_constant.wind_time = {settings.wind_direction.x, settings.wind_direction.y, settings.wind_speed,
                             parameters.time_seconds};
  const float max_distance =
      settings.max_march_distance > 0.0f
          ? settings.max_march_distance
          : (parameters.max_distance > 0.0f
                 ? parameters.max_distance
                 : (parameters.camera ? glm::max(parameters.camera->camera_settings.far_distance, 0.0f) : 0.0f));
  push_constant.lighting_phase_max_distance = {settings.lighting_intensity, settings.ambient_lighting_strength,
                                               settings.phase_anisotropy, settings.cloud_shadow_strength};
  push_constant.noise_extinction_march_distance = {settings.base_noise_scale, settings.detail_noise_scale,
                                                   settings.extinction_scale, max_distance};
  push_constant.atmosphere_cloud_type_curl = {settings.use_spherical_atmosphere ? 1.0f : 0.0f,
                                              settings.atmosphere_radius, settings.cloud_type, settings.curl_strength};
  push_constant.march_control = {settings.coarse_step_fraction, settings.fine_step_scale,
                                 static_cast<float>(settings.empty_step_fallback_count),
                                 settings.enable_temporal_reprojection ? settings.temporal_blend_factor : 0.0f};
  push_constant.camera_frame_steps = {parameters.camera_index, static_cast<int>(parameters.frame_index),
                                      settings.primary_step_count, settings.light_step_count};
  int packed_flags = parameters.input_is_ray_hit_distance ? kInputIsRayHitDistanceFlag : 0;
  packed_flags |= settings.enable_cloud_shadows ? kCloudShadowsEnabledFlag : 0;
  packed_flags |= glm::clamp(settings.cloud_shadow_step_count, 1, 64) << kCloudShadowStepShift;
  push_constant.flags = {settings.enabled ? 1 : 0, settings.debug_visualization ? 1 : 0, settings.debug_mode,
                         packed_flags};
  return push_constant;
}

RenderPassDescriptor CreateDescriptor(const char* dependency, const char* depth_or_hit_distance_resource) {
  return {RenderPassNames::volumetric_clouds,
          RenderPassQueue::Graphics,
          RenderPassScope::Camera,
          {{depth_or_hit_distance_resource, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
           {RenderResourceNames::camera_color, RenderResourceUsage::ReadWrite, RenderResourceState::StorageReadWrite},
           {RenderResourceNames::camera_volumetric_cloud_accumulation, RenderResourceUsage::Write,
            RenderResourceState::StorageReadWrite},
           {RenderResourceNames::camera_volumetric_cloud_transmittance, RenderResourceUsage::Write,
            RenderResourceState::StorageReadWrite}},
          {dependency},
          RenderPassProfilerGroup::Lighting,
          "Volumetric Clouds"};
}
}  // namespace

RenderPassDescriptor VolumetricCloudsPass::CreateRasterDescriptor(const char* dependency) {
  return CreateDescriptor(dependency ? dependency : RenderPassNames::deferred_camera,
                          RenderResourceNames::camera_depth);
}

RenderPassDescriptor VolumetricCloudsPass::CreateRayTracingDescriptor(const char* dependency) {
  return CreateDescriptor(dependency ? dependency : RenderPassNames::ray_tracing_camera,
                          RenderResourceNames::camera_ray_hit_distance);
}

void VolumetricCloudsPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (!parameters.record_commands) {
    return;
  }
  const auto& noise_resources = GetCloudNoiseResources();
  if (!noise_resources.base_shape_view || !noise_resources.detail_erosion_view ||
      !noise_resources.weather_coverage_view || !noise_resources.curl_noise_view || !noise_resources.sampler) {
    return;
  }
  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    ApplyGraphResourceBarriers(vk_command_buffer, context);
    const RenderPassGpuTimestampScope gpu_timestamp(vk_command_buffer, context,
                                                    parameters.camera ? parameters.camera->GetHandle().GetValue() : 0);
    if (parameters.pipeline && parameters.pipeline->Initialized() && parameters.composite_pipeline &&
        parameters.composite_pipeline->Initialized() && parameters.per_frame_descriptor_set &&
        parameters.descriptor_set_layout && parameters.transient_resources && parameters.camera &&
        parameters.camera->GetRenderTexture()) {
      const auto render_texture = parameters.camera->GetRenderTexture();
      const auto* depth_binding = context.GetResourceBinding(parameters.input_resource_name);
      const auto* color_binding = context.GetResourceBinding(RenderResourceNames::camera_color);
      const auto* accumulation_binding =
          context.GetResourceBinding(RenderResourceNames::camera_volumetric_cloud_accumulation);
      const auto* transmittance_binding =
          context.GetResourceBinding(RenderResourceNames::camera_volumetric_cloud_transmittance);
      if (depth_binding && depth_binding->image && color_binding && color_binding->image && accumulation_binding &&
          accumulation_binding->image && transmittance_binding && transmittance_binding->image) {
        std::shared_ptr<ImageView> ray_hit_distance_view;
        if (parameters.input_is_ray_hit_distance) {
          ray_hit_distance_view = CreateGraphImageMipView(depth_binding->image, 0);
          if (!ray_hit_distance_view) {
            ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
            return;
          }
          parameters.transient_resources->RetainImageView(ray_hit_distance_view);
        }
        const auto accumulation_view = CreateGraphImageMipView(accumulation_binding->image, 0);
        const auto transmittance_view = CreateGraphImageMipView(transmittance_binding->image, 0);
        if (accumulation_view && transmittance_view) {
          parameters.transient_resources->RetainImageView(accumulation_view);
          parameters.transient_resources->RetainImageView(transmittance_view);
          auto& history_resources =
              GetCloudHistoryResources(parameters.camera->GetHandle().GetValue(), color_binding->image->GetExtent());
          const uint32_t previous_history_index = history_resources.frame_index % 2u;
          const uint32_t output_history_index = 1u - previous_history_index;
          const bool history_valid = history_resources.valid;
          if (!history_resources.accumulation_views[previous_history_index] ||
              !history_resources.transmittance_views[previous_history_index] ||
              !history_resources.accumulation_views[output_history_index] ||
              !history_resources.transmittance_views[output_history_index]) {
            ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
            return;
          }
          for (const auto& image : history_resources.accumulation_images) {
            if (image) {
              image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
            }
          }
          for (const auto& image : history_resources.transmittance_images) {
            if (image) {
              image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
            }
          }

          const auto descriptor_set = std::make_shared<DescriptorSet>(parameters.descriptor_set_layout);
          VkDescriptorImageInfo image_info{};
          image_info.imageLayout = depth_binding->image->GetLayout();
          image_info.imageView = parameters.input_is_ray_hit_distance
                                     ? ray_hit_distance_view->GetVkImageView()
                                     : render_texture->GetDepthImageView()->GetVkImageView();
          image_info.sampler = render_texture->GetDepthSampler()->GetVkSampler();
          descriptor_set->UpdateImageDescriptorBinding(0, image_info);

          image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
          image_info.imageView = render_texture->GetColorImageView()->GetVkImageView();
          image_info.sampler = VK_NULL_HANDLE;
          descriptor_set->UpdateImageDescriptorBinding(1, image_info);
          image_info.imageView = accumulation_view->GetVkImageView();
          descriptor_set->UpdateImageDescriptorBinding(2, image_info);
          image_info.imageView = transmittance_view->GetVkImageView();
          descriptor_set->UpdateImageDescriptorBinding(3, image_info);

          image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
          image_info.imageView = accumulation_view->GetVkImageView();
          image_info.sampler = render_texture->GetColorSampler()->GetVkSampler();
          descriptor_set->UpdateImageDescriptorBinding(4, image_info);
          image_info.imageView = transmittance_view->GetVkImageView();
          descriptor_set->UpdateImageDescriptorBinding(5, image_info);

          image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
          image_info.imageView = noise_resources.base_shape_view->GetVkImageView();
          image_info.sampler = noise_resources.sampler->GetVkSampler();
          descriptor_set->UpdateImageDescriptorBinding(6, image_info);
          image_info.imageView = noise_resources.detail_erosion_view->GetVkImageView();
          descriptor_set->UpdateImageDescriptorBinding(7, image_info);
          image_info.imageView = noise_resources.weather_coverage_view->GetVkImageView();
          descriptor_set->UpdateImageDescriptorBinding(8, image_info);
          image_info.imageView = noise_resources.curl_noise_view->GetVkImageView();
          descriptor_set->UpdateImageDescriptorBinding(9, image_info);

          auto push_constant = CreatePushConstant(parameters);
          if (!history_valid) {
            push_constant.march_control.w = 0.0f;
          }
          image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
          image_info.imageView = history_resources.accumulation_views[previous_history_index]->GetVkImageView();
          image_info.sampler = render_texture->GetColorSampler()->GetVkSampler();
          descriptor_set->UpdateImageDescriptorBinding(10, image_info);
          image_info.imageView = history_resources.transmittance_views[previous_history_index]->GetVkImageView();
          descriptor_set->UpdateImageDescriptorBinding(11, image_info);
          image_info.sampler = VK_NULL_HANDLE;
          image_info.imageView = history_resources.accumulation_views[output_history_index]->GetVkImageView();
          descriptor_set->UpdateImageDescriptorBinding(12, image_info);
          image_info.imageView = history_resources.transmittance_views[output_history_index]->GetVkImageView();
          descriptor_set->UpdateImageDescriptorBinding(13, image_info);

          const auto cloud_extent = accumulation_binding->image->GetExtent();
          parameters.pipeline->Bind(vk_command_buffer);
          parameters.pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                 parameters.per_frame_descriptor_set->GetVkDescriptorSet());
          parameters.pipeline->BindDescriptorSet(vk_command_buffer, 1, descriptor_set->GetVkDescriptorSet());
          parameters.pipeline->PushConstant(vk_command_buffer, 0, push_constant);
          parameters.pipeline->Dispatch(vk_command_buffer, Platform::DivUp(cloud_extent.width, 16),
                                        Platform::DivUp(cloud_extent.height, 16));
          Platform::EverythingBarrier(vk_command_buffer);

          const auto extent = color_binding->image->GetExtent();
          parameters.composite_pipeline->Bind(vk_command_buffer);
          parameters.composite_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                           parameters.per_frame_descriptor_set->GetVkDescriptorSet());
          parameters.composite_pipeline->BindDescriptorSet(vk_command_buffer, 1, descriptor_set->GetVkDescriptorSet());
          parameters.composite_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
          parameters.composite_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(extent.width, 16),
                                                  Platform::DivUp(extent.height, 16));
          parameters.transient_resources->RetainDescriptorSet(descriptor_set);
          history_resources.valid = true;
          history_resources.frame_index++;
          Platform::EverythingBarrier(vk_command_buffer);
        }
      }
    }
    ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
  });
}
