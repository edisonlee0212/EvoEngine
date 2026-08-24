#include "GlobalReflectionProbe.hpp"

#include "Application.hpp"
#include "AssetManager.hpp"
#include "Console.hpp"
#include "GeometryStorage.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Serialization.hpp"
#include "Shader.hpp"
#include "TextureStorage.hpp"

#include <glm/gtc/packing.hpp>

#include <atomic>
#include <chrono>

#ifdef _WIN32
#  include <Windows.h>
#endif

using namespace evo_engine;

std::shared_ptr<GraphicsPipeline> GlobalReflectionProbe::shared_prefilter_construct_pipeline_;

namespace {
constexpr const char* kPayloadLayout = "rgba16f-linear-vulkan-face-major-mip-major-le-v1";
std::atomic<uint64_t> temporary_file_counter = 0;
constexpr std::array<uint32_t, GlobalReflectionProbe::kMipLevels> kPrefilterSampleCounts = {1u,  256u, 128u, 64u, 32u,
                                                                                            16u, 16u,  16u,  16u};

struct PrefilterConstant {
  glm::mat4 projection_view{};
  float roughness = 0.0f;
  uint32_t sample_count = 1u;
};

class GlobalReflectionProbeStagedLoadPayload final : public StagedAssetLoadPayload {
 public:
  std::vector<uint16_t> pixels;
  GlobalReflectionProbe::SourceKind source_kind = GlobalReflectionProbe::SourceKind::Empty;
  uint64_t payload_hash = 0;
};

std::shared_ptr<GlobalReflectionProbeStagedLoadPayload> DecodePayloadNode(const YAML::Node& node) {
  auto payload = std::make_shared<GlobalReflectionProbeStagedLoadPayload>();
  if (!node || node.IsNull()) {
    throw std::invalid_argument("Global reflection probe document is empty or truncated.");
  }
  if (!node["schema_version"] || !node["format"] || !node["payload_layout"] || !node["resolution"] ||
      !node["mip_levels"] || !node["source_kind"] || !node["payload_hash"] || !node["pixels"] ||
      node["schema_version"].as<uint32_t>() != GlobalReflectionProbe::kSchemaVersion ||
      node["format"].as<uint32_t>() != static_cast<uint32_t>(GlobalReflectionProbe::kCanonicalFormat) ||
      node["payload_layout"].as<std::string>() != kPayloadLayout ||
      node["resolution"].as<uint32_t>() != GlobalReflectionProbe::kResolution ||
      node["mip_levels"].as<uint32_t>() != GlobalReflectionProbe::kMipLevels) {
    throw std::invalid_argument("Global reflection probe header is not canonical.");
  }
  const auto binary = node["pixels"].as<YAML::Binary>();
  if (binary.size() != GlobalReflectionProbe::kCanonicalPayloadByteSize) {
    throw std::invalid_argument("Global reflection probe payload size is not canonical.");
  }
  payload->pixels.resize(GlobalReflectionProbe::kCanonicalTexelCount * 4);
  std::memcpy(payload->pixels.data(), binary.data(), binary.size());
  const auto source_kind = node["source_kind"].as<uint32_t>();
  if (source_kind > static_cast<uint32_t>(GlobalReflectionProbe::SourceKind::Baked)) {
    throw std::invalid_argument("Global reflection probe source kind is invalid.");
  }
  payload->source_kind = static_cast<GlobalReflectionProbe::SourceKind>(source_kind);
  payload->payload_hash = node["payload_hash"].as<uint64_t>();
  std::string error;
  if (!GlobalReflectionProbe::ValidateCanonicalPayload(payload->pixels, error)) {
    throw std::invalid_argument(error);
  }
  if (payload->payload_hash != GlobalReflectionProbe::CalculatePayloadHash(payload->pixels)) {
    throw std::invalid_argument("Global reflection probe payload hash does not match its contents.");
  }
  return payload;
}

std::shared_ptr<GlobalReflectionProbeStagedLoadPayload> DecodePayload(const std::filesystem::path& path) {
  try {
    std::ifstream stream(path);
    return stream.is_open() ? DecodePayloadNode(YAML::Load(stream)) : nullptr;
  } catch (const std::exception& e) {
    EVOENGINE_ERROR("Failed to load global reflection probe: " + std::string(e.what()))
    return {};
  }
}

bool ReplaceFile(const std::filesystem::path& temporary_path, const std::filesystem::path& path) {
#ifdef _WIN32
  return MoveFileExW(temporary_path.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != FALSE;
#else
  std::error_code error;
  std::filesystem::rename(temporary_path, path, error);
  return !error;
#endif
}
}  // namespace

GlobalReflectionProbe::GlobalReflectionProbe() {
  Initialize();
}

void GlobalReflectionProbe::Initialize() {
  mip_map_views_.clear();
  cubemap_ = AssetManager::CreateTemporaryAsset<Cubemap>();
  const std::vector<uint16_t> empty_payload(kCanonicalTexelCount * 4);
  cubemap_->SetRgba16fData(empty_payload, kResolution, kMipLevels);
  source_kind_ = SourceKind::Empty;
  payload_hash_ = CalculatePayloadHash(empty_payload);
  RebuildMipMapViews();
}

void GlobalReflectionProbe::RebuildMipMapViews() {
  mip_map_views_.clear();
  mip_map_views_.resize(6);
  if (!Platform::Initialized() || !cubemap_->RefStorage().image) {
    return;
  }
  for (uint32_t face = 0; face < 6; ++face) {
    auto& face_views = mip_map_views_[face];
    face_views.reserve(kMipLevels);
    for (uint32_t mip = 0; mip < kMipLevels; ++mip) {
      VkImageViewCreateInfo view_info{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
      view_info.image = cubemap_->RefStorage().image->GetVkImage();
      view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
      view_info.format = cubemap_->GetFormat();
      view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      view_info.subresourceRange.baseMipLevel = mip;
      view_info.subresourceRange.levelCount = 1;
      view_info.subresourceRange.baseArrayLayer = face;
      view_info.subresourceRange.layerCount = 1;
      face_views.emplace_back(std::make_shared<ImageView>(view_info));
    }
  }
}

std::shared_ptr<Cubemap> GlobalReflectionProbe::GetCubemap() const {
  return cubemap_;
}

const std::vector<uint16_t>& GlobalReflectionProbe::GetCanonicalPayload() const {
  static const std::vector<uint16_t> empty;
  return cubemap_ ? cubemap_->PeekRgba16fData() : empty;
}

bool GlobalReflectionProbe::ReadCanonicalPayload(std::vector<uint16_t>& payload) const {
  if (!cubemap_)
    return false;
  cubemap_->GetRgba16fData(payload, true);
  return payload.size() * sizeof(uint16_t) == kCanonicalPayloadByteSize;
}

size_t GlobalReflectionProbe::GetCanonicalPayloadByteSize() const {
  return GetCanonicalPayload().size() * sizeof(uint16_t);
}

VkFormat GlobalReflectionProbe::GetRuntimeFormat() const {
  return kCanonicalFormat;
}

GlobalReflectionProbe::SourceKind GlobalReflectionProbe::GetSourceKind() const {
  return source_kind_;
}

uint64_t GlobalReflectionProbe::GetPayloadHash() const {
  return payload_hash_;
}

bool GlobalReflectionProbe::IsRuntimeReady() const {
  if (source_kind_ == SourceKind::Empty || !cubemap_ || cubemap_->GetResolution() != kResolution ||
      cubemap_->GetMipLevels() != kMipLevels || cubemap_->GetFormat() != kCanonicalFormat) {
    return false;
  }
  if (Platform::Initialized()) {
    return cubemap_->gpu_content_valid_ && cubemap_->GetImage() && cubemap_->GetImageView() && cubemap_->GetSampler();
  }
  return payload_hash_ != 0u && GetCanonicalPayloadByteSize() == kCanonicalPayloadByteSize;
}

bool GlobalReflectionProbe::PackedRuntimeFormatSupported() const {
  return Platform::Initialized() &&
         Platform::SupportsCubemapFormat(VK_FORMAT_B10G11R11_UFLOAT_PACK32, kResolution, kMipLevels);
}

std::shared_ptr<GraphicsPipeline> GlobalReflectionProbe::AcquirePrefilterPipeline(
    const std::shared_ptr<DescriptorSetLayout>& descriptor_set_layout) {
  if (shared_prefilter_construct_pipeline_) {
    return shared_prefilter_construct_pipeline_;
  }
  auto pipeline = std::make_shared<GraphicsPipeline>();
  pipeline->vertex_shader =
      Shader::CreateTemporary(ShaderType::Vertex, Resources::GetDefaultResourcesPath() /
                                                      "Shaders/Graphics/Vertex/Lighting/CubemapProcess.slang");
  pipeline->fragment_shader = Shader::CreateTemporary(
      ShaderType::Fragment,
      Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Lighting/EnvironmentalMapPrefilter.slang");
  pipeline->geometry_type = GeometryType::Mesh;
  pipeline->vertex_input_attribute_set = VertexInputAttributeSet::Position;
  pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  pipeline->color_attachment_formats = {1, kCanonicalFormat};
  pipeline->descriptor_set_layouts.emplace_back(descriptor_set_layout);
  auto& push_constant_range = pipeline->push_constant_ranges.emplace_back();
  push_constant_range.size = sizeof(PrefilterConstant);
  push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  pipeline->Initialize();
  shared_prefilter_construct_pipeline_ = std::move(pipeline);
  return shared_prefilter_construct_pipeline_;
}

void GlobalReflectionProbe::RecordPrefilter(
    const VkCommandBuffer command_buffer, const std::shared_ptr<Cubemap>& filtered,
    const std::vector<std::vector<std::shared_ptr<ImageView>>>& filtered_mip_views,
    const std::shared_ptr<Image>& depth_image, const std::shared_ptr<ImageView>& depth_view,
    const std::shared_ptr<DescriptorSet>& descriptor_set, const std::shared_ptr<GraphicsPipeline>& pipeline) {
  RecordPrefilterFaces(command_buffer, filtered, filtered_mip_views, depth_image, depth_view, descriptor_set, pipeline,
                       0u, 6u);
}

void GlobalReflectionProbe::RecordPrefilterFaces(
    const VkCommandBuffer command_buffer, const std::shared_ptr<Cubemap>& filtered,
    const std::vector<std::vector<std::shared_ptr<ImageView>>>& filtered_mip_views,
    const std::shared_ptr<Image>& depth_image, const std::shared_ptr<ImageView>& depth_view,
    const std::shared_ptr<DescriptorSet>& descriptor_set, const std::shared_ptr<GraphicsPipeline>& pipeline,
    const uint32_t first_face, const uint32_t face_count) {
  if (!filtered || !filtered->GetImage() || !depth_image || !depth_view || !descriptor_set || !pipeline ||
      !pipeline->Initialized() || filtered_mip_views.size() != 6 || first_face >= 6u || face_count == 0u ||
      first_face + face_count > 6u) {
    throw std::runtime_error("Global reflection probe prefilter resources are unavailable.");
  }
  filtered->RefStorage().image->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
  depth_image->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
  GeometryStorage::BindVertices(command_buffer);
  const glm::mat4 projection = glm::perspective(glm::radians(90.0f), 1.0f, 0.1f, 10.0f);
  const glm::mat4 views[] = {glm::lookAt(glm::vec3(0), glm::vec3(1, 0, 0), glm::vec3(0, -1, 0)),
                             glm::lookAt(glm::vec3(0), glm::vec3(-1, 0, 0), glm::vec3(0, -1, 0)),
                             glm::lookAt(glm::vec3(0), glm::vec3(0, 1, 0), glm::vec3(0, 0, 1)),
                             glm::lookAt(glm::vec3(0), glm::vec3(0, -1, 0), glm::vec3(0, 0, -1)),
                             glm::lookAt(glm::vec3(0), glm::vec3(0, 0, 1), glm::vec3(0, -1, 0)),
                             glm::lookAt(glm::vec3(0), glm::vec3(0, 0, -1), glm::vec3(0, -1, 0))};
  for (uint32_t mip = 0; mip < kMipLevels; ++mip) {
    const uint32_t mip_width = glm::max(kResolution >> mip, 1u);
    VkRect2D render_area{{0, 0}, {mip_width, mip_width}};
    pipeline->states.view_port = {0.0f, 0.0f, static_cast<float>(mip_width), static_cast<float>(mip_width), 0.0f, 1.0f};
    pipeline->states.scissor = render_area;
    pipeline->states.cull_mode = VK_CULL_MODE_NONE;
    pipeline->states.color_blend_attachment_states.resize(1);
    auto& blend = pipeline->states.color_blend_attachment_states[0];
    blend.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    blend.blendEnable = VK_FALSE;
    for (uint32_t face = first_face; face < first_face + face_count; ++face) {
      if (filtered_mip_views[face].size() != kMipLevels) {
        throw std::runtime_error("Global reflection probe prefilter views are incomplete.");
      }
      VkRenderingAttachmentInfo color_attachment{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
      color_attachment.imageView = filtered_mip_views[face][mip]->GetVkImageView();
      color_attachment.imageLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL;
      color_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
      color_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
      color_attachment.clearValue = {{0, 0, 0, 1}};
      VkRenderingAttachmentInfo depth_attachment{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
      depth_attachment.imageView = depth_view->GetVkImageView();
      depth_attachment.imageLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL;
      depth_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
      depth_attachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
      depth_attachment.clearValue.depthStencil = {1.0f, 0};
      VkRenderingInfo render_info{VK_STRUCTURE_TYPE_RENDERING_INFO};
      render_info.renderArea = render_area;
      render_info.layerCount = 1;
      render_info.colorAttachmentCount = 1;
      render_info.pColorAttachments = &color_attachment;
      render_info.pDepthAttachment = &depth_attachment;
      Platform::BeginRendering(command_buffer, render_info);
      pipeline->Bind(command_buffer);
      pipeline->BindDescriptorSet(command_buffer, 0, descriptor_set->GetVkDescriptorSet());
      const PrefilterConstant constant{projection * views[face],
                                       static_cast<float>(mip) / static_cast<float>(kMipLevels - 1),
                                       kPrefilterSampleCounts[mip]};
      pipeline->PushConstant(command_buffer, 0, constant);
      Resources::GetInstance().GetRenderingCube()->DrawIndexed(command_buffer, pipeline->states, 1);
      Platform::EndRendering(command_buffer);
      VkImageMemoryBarrier2 depth_reuse{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2};
      depth_reuse.srcStageMask =
          VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
      depth_reuse.srcAccessMask = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
      depth_reuse.dstStageMask = depth_reuse.srcStageMask;
      depth_reuse.dstAccessMask =
          VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
      depth_reuse.oldLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL;
      depth_reuse.newLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL;
      depth_reuse.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
      depth_reuse.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
      depth_reuse.image = depth_image->GetVkImage();
      depth_reuse.subresourceRange = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1};
      VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
      dependency.imageMemoryBarrierCount = 1;
      dependency.pImageMemoryBarriers = &depth_reuse;
      vkCmdPipelineBarrier2(command_buffer, &dependency);
    }
  }
  filtered->RefStorage().image->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
}

bool GlobalReflectionProbe::ConstructFromCubemap(const std::shared_ptr<Cubemap>& target_cubemap) {
  return ConstructFilteredFromCubemap(target_cubemap, SourceKind::Imported, true);
}

bool GlobalReflectionProbe::ConstructFilteredFromCubemap(const std::shared_ptr<Cubemap>& target_cubemap,
                                                         const SourceKind source_kind, const bool retain_cpu_payload) {
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!render_layer || !target_cubemap || !target_cubemap->GetImage() || !target_cubemap->GetImageView() ||
      !target_cubemap->GetSampler()) {
    EVOENGINE_ERROR("Global reflection probe source cubemap is unavailable.")
    return false;
  }

  const auto filtered_cubemap = AssetManager::CreateTemporaryAsset<Cubemap>();
  filtered_cubemap->Initialize(kResolution, kMipLevels, kCanonicalFormat);
  if (!filtered_cubemap->GetImage()) {
    EVOENGINE_ERROR("Global reflection probe import target could not be allocated.")
    return false;
  }
  std::vector<std::vector<std::shared_ptr<ImageView>>> filtered_mip_views(6);
  for (uint32_t face = 0; face < 6; ++face) {
    auto& face_views = filtered_mip_views[face];
    face_views.reserve(kMipLevels);
    for (uint32_t mip = 0; mip < kMipLevels; ++mip) {
      VkImageViewCreateInfo view_info{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
      view_info.image = filtered_cubemap->RefStorage().image->GetVkImage();
      view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
      view_info.format = kCanonicalFormat;
      view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      view_info.subresourceRange.baseMipLevel = mip;
      view_info.subresourceRange.levelCount = 1;
      view_info.subresourceRange.baseArrayLayer = face;
      view_info.subresourceRange.layerCount = 1;
      face_views.emplace_back(std::make_shared<ImageView>(view_info));
    }
  }

  VkImageCreateInfo depth_image_info{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
  depth_image_info.imageType = VK_IMAGE_TYPE_2D;
  depth_image_info.extent = {kResolution, kResolution, 1};
  depth_image_info.mipLevels = 1;
  depth_image_info.arrayLayers = 1;
  depth_image_info.format = Platform::Constants::shadow_map;
  depth_image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
  depth_image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  depth_image_info.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
  depth_image_info.samples = VK_SAMPLE_COUNT_1_BIT;
  depth_image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  const auto depth_image = std::make_shared<Image>(depth_image_info);

  VkImageViewCreateInfo depth_view_info{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
  depth_view_info.image = depth_image->GetVkImage();
  depth_view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
  depth_view_info.format = Platform::Constants::shadow_map;
  depth_view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
  depth_view_info.subresourceRange.levelCount = 1;
  depth_view_info.subresourceRange.layerCount = 1;
  const auto depth_view = std::make_shared<ImageView>(depth_view_info);

  const auto descriptor_set =
      std::make_shared<DescriptorSet>(render_layer->GetRenderTexturePresentDescriptorSetLayout());
  VkDescriptorImageInfo descriptor_image_info{};
  descriptor_image_info.imageView = target_cubemap->GetImageView()->GetVkImageView();
  descriptor_image_info.imageLayout = target_cubemap->GetImage()->GetLayout();
  descriptor_image_info.sampler = target_cubemap->GetSampler()->GetVkSampler();
  descriptor_set->UpdateImageDescriptorBinding(0, descriptor_image_info);

  const auto pipeline = AcquirePrefilterPipeline(render_layer->GetRenderTexturePresentDescriptorSetLayout());
  filtered_cubemap->BeginGpuWrite();
  Platform::ImmediateSubmit([&](const VkCommandBuffer command_buffer) {
    RecordPrefilter(command_buffer, filtered_cubemap, filtered_mip_views, depth_image, depth_view, descriptor_set,
                    pipeline);
  });
  filtered_cubemap->MarkGpuContentValid();
  uint64_t payload_hash = 0u;
  if (retain_cpu_payload) {
    std::vector<uint16_t> canonical;
    filtered_cubemap->GetRgba16fData(canonical, true);
    std::string error;
    if (!ValidateCanonicalPayload(canonical, error)) {
      EVOENGINE_ERROR("Global reflection probe import rejected: " + error)
      return false;
    }
    payload_hash = CalculatePayloadHash(canonical);
  }
  cubemap_ = filtered_cubemap;
  RebuildMipMapViews();
  payload_hash_ = payload_hash;
  source_kind_ = source_kind;
  SetUnsaved();
  return true;
}

void GlobalReflectionProbe::MarkBaked() {
  source_kind_ = SourceKind::Baked;
  payload_hash_ = CalculatePayloadHash(GetCanonicalPayload());
  SetUnsaved();
}

bool GlobalReflectionProbe::SetCanonicalPayload(const std::vector<uint16_t>& payload) {
  std::string error;
  if (!ValidateCanonicalPayload(payload, error)) {
    EVOENGINE_ERROR(error)
    return false;
  }
  if (!cubemap_) {
    cubemap_ = AssetManager::CreateTemporaryAsset<Cubemap>();
  }
  mip_map_views_.clear();
  if (!cubemap_->SetRgba16fData(payload, kResolution, kMipLevels)) {
    return false;
  }
  RebuildMipMapViews();
  payload_hash_ = CalculatePayloadHash(payload);
  source_kind_ = SourceKind::Imported;
  SetUnsaved();
  return true;
}

uint64_t GlobalReflectionProbe::CalculatePayloadHash(const std::vector<uint16_t>& payload) {
  uint64_t hash = 1469598103934665603ull;
  for (const auto value : payload) {
    hash ^= static_cast<uint8_t>(value & 0xffu);
    hash *= 1099511628211ull;
    hash ^= static_cast<uint8_t>(value >> 8u);
    hash *= 1099511628211ull;
  }
  return hash;
}

bool GlobalReflectionProbe::ValidateCanonicalPayload(const std::vector<uint16_t>& payload, std::string& error) {
  if (payload.size() != kCanonicalTexelCount * 4) {
    error = "Global reflection probe payload must contain exactly 524286 RGBA16F texels.";
    return false;
  }
  for (size_t i = 0; i < payload.size(); ++i) {
    const float value = glm::unpackHalf1x16(payload[i]);
    if (!std::isfinite(value) || (i % 4 != 3 && value < 0.0f)) {
      error = "Global reflection probe payload contains a non-finite or negative color channel.";
      return false;
    }
  }
  error.clear();
  return true;
}

bool GlobalReflectionProbe::EvaluatePackedRuntimeQuality(const std::vector<uint16_t>& payload,
                                                         float& normalized_rms_error, float& relative_peak_error) {
  std::string error;
  if (!ValidateCanonicalPayload(payload, error)) {
    return false;
  }
  double squared_error = 0.0;
  double squared_signal = 0.0;
  float peak_error = 0.0f;
  float peak_signal = 0.0f;
  for (size_t i = 0; i < payload.size(); i += 4) {
    const glm::vec3 source{glm::unpackHalf1x16(payload[i]), glm::unpackHalf1x16(payload[i + 1]),
                           glm::unpackHalf1x16(payload[i + 2])};
    const glm::vec3 restored = glm::unpackF2x11_1x10(glm::packF2x11_1x10(source));
    const glm::vec3 difference = glm::abs(restored - source);
    squared_error += glm::dot(difference, difference);
    squared_signal += glm::dot(source, source);
    peak_error = glm::max(peak_error, glm::max(difference.x, glm::max(difference.y, difference.z)));
    peak_signal = glm::max(peak_signal, glm::max(source.x, glm::max(source.y, source.z)));
  }
  normalized_rms_error = squared_signal > 0.0 ? static_cast<float>(std::sqrt(squared_error / squared_signal)) : 0.0f;
  relative_peak_error = peak_signal > 0.0f ? peak_error / peak_signal : 0.0f;
  return true;
}

void GlobalReflectionProbe::Serialize(YAML::Emitter& out) const {
  if (!cubemap_) {
    throw std::runtime_error("Global reflection probe has no canonical cubemap.");
  }
  std::vector<uint16_t> payload;
  cubemap_->GetRgba16fData(payload, true);
  std::string error;
  if (!ValidateCanonicalPayload(payload, error)) {
    throw std::runtime_error(error);
  }
  payload_hash_ = CalculatePayloadHash(payload);
  out << YAML::Key << "schema_version" << YAML::Value << kSchemaVersion;
  out << YAML::Key << "format" << YAML::Value << static_cast<uint32_t>(kCanonicalFormat);
  out << YAML::Key << "payload_layout" << YAML::Value << kPayloadLayout;
  out << YAML::Key << "resolution" << YAML::Value << kResolution;
  out << YAML::Key << "mip_levels" << YAML::Value << kMipLevels;
  out << YAML::Key << "source_kind" << YAML::Value << static_cast<uint32_t>(source_kind_);
  out << YAML::Key << "payload_hash" << YAML::Value << payload_hash_;
  out << YAML::Key << "pixels" << YAML::Value
      << YAML::Binary(reinterpret_cast<const unsigned char*>(payload.data()), payload.size() * sizeof(uint16_t));
}

void GlobalReflectionProbe::Deserialize(const YAML::Node& in) {
  const auto payload = DecodePayloadNode(in);
  if (!ApplyStagedPayloadInternal({}, payload)) {
    throw std::invalid_argument("Global reflection probe payload could not be applied.");
  }
}

bool GlobalReflectionProbe::SaveInternal(const std::filesystem::path& path) const {
  YAML::Emitter out;
  out << YAML::BeginMap;
  try {
    Serialize(out);
  } catch (const std::exception& exception) {
    EVOENGINE_ERROR(exception.what())
    return false;
  }
  out << YAML::EndMap;
  std::filesystem::path temporary_path;
  try {
    temporary_path = path;
    temporary_path += ".tmp." + std::to_string(temporary_file_counter.fetch_add(1)) + "." +
                      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    std::ofstream stream(temporary_path, std::ios::binary | std::ios::trunc);
    stream << out.c_str();
    stream.flush();
    if (!stream) {
      stream.close();
      std::filesystem::remove(temporary_path);
      return false;
    }
    stream.close();
    if (ReplaceFile(temporary_path, path)) {
      return true;
    }
  } catch (const std::exception& exception) {
    EVOENGINE_ERROR("Failed to save global reflection probe: " + std::string(exception.what()))
  }
  std::error_code error;
  if (!temporary_path.empty()) {
    std::filesystem::remove(temporary_path, error);
  }
  return false;
}

bool GlobalReflectionProbe::LoadInternal(const std::filesystem::path& path) {
  return ApplyStagedPayloadInternal(path, LoadStagedPayloadInternal(path));
}

std::shared_ptr<StagedAssetLoadPayload> GlobalReflectionProbe::LoadStagedPayloadInternal(
    const std::filesystem::path& path) const {
  return DecodePayload(path);
}

bool GlobalReflectionProbe::ApplyStagedPayloadInternal(const std::filesystem::path&,
                                                       const std::shared_ptr<StagedAssetLoadPayload>& payload) {
  const auto probe_payload = std::dynamic_pointer_cast<GlobalReflectionProbeStagedLoadPayload>(payload);
  if (!probe_payload || !SetCanonicalPayload(probe_payload->pixels)) {
    return false;
  }
  source_kind_ = probe_payload->source_kind;
  payload_hash_ = probe_payload->payload_hash;
  return true;
}

bool GlobalReflectionProbe::RegisterAssetIoHandlers(const std::string& owner_name, const std::string& type_name) {
  return Serialization::RegisterAssetIoHandler<GlobalReflectionProbe>(
      [](const GlobalReflectionProbe& asset, const std::filesystem::path& path) {
        return asset.SaveInternal(path);
      },
      [](GlobalReflectionProbe& asset, const std::filesystem::path& path) {
        return asset.LoadInternal(path);
      },
      [](const GlobalReflectionProbe&, const std::filesystem::path&) {
        return true;
      },
      [](const GlobalReflectionProbe& asset, const std::filesystem::path& path) {
        return asset.LoadStagedPayloadInternal(path);
      },
      [](GlobalReflectionProbe& asset, const std::filesystem::path& path,
         const std::shared_ptr<StagedAssetLoadPayload>& payload) {
        return asset.ApplyStagedPayloadInternal(path, payload);
      },
      owner_name, type_name);
}
