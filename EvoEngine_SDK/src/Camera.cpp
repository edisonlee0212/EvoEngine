#include "Camera.hpp"
#include "Application.hpp"
#include "Cubemap.hpp"
#include "EditorLayer.hpp"
#include "PostProcessingStack.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Scene.hpp"
#include "Serialization.hpp"
#include "Utilities.hpp"
using namespace evo_engine;

glm::vec3 CameraInfoBlock::Project(const glm::vec3& position) const {
  return projection * view * glm::vec4(position, 1.0f);
}

glm::vec3 CameraInfoBlock::UnProject(const glm::vec3& position) const {
  const glm::mat4 inverse = glm::inverse(projection * view);
  auto start = glm::vec4(position, 1.0f);
  start = inverse * start;
  return start / start.w;
}

void Camera::UpdateGBuffer() {
  if (!Platform::Initialized())
    return;
  g_buffer_normal_view_.reset();
  g_buffer_material_view_.reset();

  g_buffer_normal_.reset();
  g_buffer_material_.reset();

  {
    VkImageCreateInfo image_info{};
    image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    image_info.imageType = VK_IMAGE_TYPE_2D;
    image_info.extent = render_texture_->GetExtent();
    image_info.mipLevels = 1;
    image_info.arrayLayers = 1;
    image_info.format = Platform::Constants::g_buffer_color;
    image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
    image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    image_info.usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                       VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    image_info.samples = VK_SAMPLE_COUNT_1_BIT;
    image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    g_buffer_normal_ = std::make_unique<Image>(image_info);

    VkImageViewCreateInfo view_info{};
    view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    view_info.image = g_buffer_normal_->GetVkImage();
    view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view_info.format = Platform::Constants::g_buffer_color;
    view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    view_info.subresourceRange.baseMipLevel = 0;
    view_info.subresourceRange.levelCount = 1;
    view_info.subresourceRange.baseArrayLayer = 0;
    view_info.subresourceRange.layerCount = 1;

    g_buffer_normal_view_ = std::make_unique<ImageView>(view_info);

    VkSamplerCreateInfo sampler_info{};
    sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    sampler_info.magFilter = VK_FILTER_NEAREST;
    sampler_info.minFilter = VK_FILTER_NEAREST;
    sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    sampler_info.anisotropyEnable = VK_TRUE;
    sampler_info.maxAnisotropy = Platform::GetSelectedPhysicalDevice()->properties.limits.maxSamplerAnisotropy;
    sampler_info.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
    sampler_info.unnormalizedCoordinates = VK_FALSE;
    sampler_info.compareEnable = VK_FALSE;
    sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
    sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;

    g_buffer_normal_sampler_ = std::make_unique<Sampler>(sampler_info);
  }
  {
    VkImageCreateInfo image_info{};
    image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    image_info.imageType = VK_IMAGE_TYPE_2D;
    image_info.extent = render_texture_->GetExtent();
    image_info.mipLevels = 1;
    image_info.arrayLayers = 1;
    image_info.format = Platform::Constants::g_buffer_color;
    image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
    image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    image_info.usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                       VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    image_info.samples = VK_SAMPLE_COUNT_1_BIT;
    image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    g_buffer_material_ = std::make_unique<Image>(image_info);

    VkImageViewCreateInfo view_info{};
    view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    view_info.image = g_buffer_material_->GetVkImage();
    view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view_info.format = Platform::Constants::g_buffer_material;
    view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    view_info.subresourceRange.baseMipLevel = 0;
    view_info.subresourceRange.levelCount = 1;
    view_info.subresourceRange.baseArrayLayer = 0;
    view_info.subresourceRange.layerCount = 1;

    g_buffer_material_view_ = std::make_unique<ImageView>(view_info);

    view_info.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
    view_info.components.g = VK_COMPONENT_SWIZZLE_ZERO;
    view_info.components.b = VK_COMPONENT_SWIZZLE_G;
    view_info.components.a = VK_COMPONENT_SWIZZLE_ONE;
    g_buffer_material_tex_coord_view_ = std::make_unique<ImageView>(view_info);

    view_info.components.r = VK_COMPONENT_SWIZZLE_B;
    view_info.components.g = VK_COMPONENT_SWIZZLE_ZERO;
    view_info.components.b = VK_COMPONENT_SWIZZLE_A;
    view_info.components.a = VK_COMPONENT_SWIZZLE_ONE;
    g_buffer_material_indices_view_ = std::make_unique<ImageView>(view_info);

    VkSamplerCreateInfo sampler_info{};
    sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    sampler_info.magFilter = VK_FILTER_NEAREST;
    sampler_info.minFilter = VK_FILTER_NEAREST;
    sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    sampler_info.anisotropyEnable = VK_TRUE;
    sampler_info.maxAnisotropy = Platform::GetSelectedPhysicalDevice()->properties.limits.maxSamplerAnisotropy;
    sampler_info.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
    sampler_info.unnormalizedCoordinates = VK_FALSE;
    sampler_info.compareEnable = VK_FALSE;
    sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
    sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;

    g_buffer_material_sampler_ = std::make_unique<Sampler>(sampler_info);
  }
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    TransitGBufferImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });

  EditorLayer::UpdateTextureId(g_buffer_normal_im_texture_id_, g_buffer_normal_sampler_->GetVkSampler(),
                               g_buffer_normal_view_->GetVkImageView(), g_buffer_normal_->GetLayout());
  EditorLayer::UpdateTextureId(g_buffer_material_tex_coord_im_texture_id_, g_buffer_material_sampler_->GetVkSampler(),
                               g_buffer_material_tex_coord_view_->GetVkImageView(), g_buffer_material_->GetLayout());
  EditorLayer::UpdateTextureId(g_buffer_material_indices_im_texture_id_, g_buffer_material_sampler_->GetVkSampler(),
                               g_buffer_material_indices_view_->GetVkImageView(), g_buffer_material_->GetLayout());
  {
    VkDescriptorImageInfo image_info{};
    image_info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    image_info.imageView = render_texture_->GetDepthImageView()->GetVkImageView();
    image_info.sampler = render_texture_->GetDepthSampler()->GetVkSampler();
    g_buffer_descriptor_set_->UpdateImageDescriptorBinding(17, image_info);
    image_info.imageView = g_buffer_normal_view_->GetVkImageView();
    image_info.sampler = g_buffer_normal_sampler_->GetVkSampler();
    g_buffer_descriptor_set_->UpdateImageDescriptorBinding(18, image_info);
    image_info.imageView = g_buffer_material_view_->GetVkImageView();
    image_info.sampler = g_buffer_material_sampler_->GetVkSampler();
    g_buffer_descriptor_set_->UpdateImageDescriptorBinding(19, image_info);
  }
}

void Camera::TransitGBufferImageLayout(const VkCommandBuffer vk_command_buffer, VkImageLayout target_layout) const {
  g_buffer_normal_->TransitImageLayout(vk_command_buffer, target_layout);
  g_buffer_material_->TransitImageLayout(vk_command_buffer, target_layout);
}
void Camera::UpdateCameraInfoBlock(CameraInfoBlock& camera_info_block, const GlobalTransform& global_transform) {
  const auto rotation = global_transform.GetRotation();
  const auto position = global_transform.GetPosition();
  const glm::vec3 front = rotation * glm::vec3(0, 0, -1);
  const glm::vec3 up = rotation * glm::vec3(0, 1, 0);
  const auto ratio = GetSizeRatio();

  camera_info_block.projection = glm::perspective(glm::radians(camera_settings.fov * 0.5f), ratio,
                                                  camera_settings.near_distance, camera_settings.far_distance);
  camera_info_block.view = glm::lookAt(position, position + front, up);
  camera_info_block.projection_view = camera_info_block.projection * camera_info_block.view;
  camera_info_block.inverse_projection = glm::inverse(camera_info_block.projection);
  camera_info_block.inverse_view = glm::inverse(camera_info_block.view);
  camera_info_block.inverse_projection_view = glm::inverse(camera_info_block.projection * camera_info_block.view);
  camera_info_block.clear_color = glm::vec4(camera_settings.clear_color, camera_settings.background_intensity);
  camera_info_block.resolution = size_;
  camera_info_block.fade_factor = camera_settings.fade_factor;
  camera_info_block.fade_ratio = camera_settings.fade_ratio;
  if (camera_settings.use_clear_color) {
    camera_info_block.camera_use_clear_color = 1;
  } else {
    camera_info_block.camera_use_clear_color = 0;
  }
  if (const auto camera_skybox = skybox.Get<Cubemap>()) {
    camera_info_block.skybox_texture_index = camera_skybox->GetTextureStorageIndex();
  } else {
    const auto default_cubemap = Resources::TryGetResource<Cubemap>("DEFAULT_SKYBOX");
    camera_info_block.skybox_texture_index = default_cubemap->GetTextureStorageIndex();
  }
  if (const auto render_layer = Application::GetLayer<RenderLayer>()) {
    const auto camera_position = global_transform.GetPosition();
    const auto scene = Application::GetActiveScene();
    auto light_probe = scene->environment.GetLightProbe(camera_position);
    auto reflection_probe = scene->environment.GetReflectionProbe(camera_position);
    if (!light_probe) {
      light_probe =
          Resources::TryGetResource<EnvironmentalMap>("DEFAULT_ENVIRONMENTAL_MAP")->light_probe.Get<LightProbe>();
    }
    camera_info_block.environmental_irradiance_texture_index = light_probe->cubemap_->GetTextureStorageIndex();
    if (!reflection_probe) {
      reflection_probe = Resources::TryGetResource<EnvironmentalMap>("DEFAULT_ENVIRONMENTAL_MAP")
                             ->reflection_probe.Get<ReflectionProbe>();
    }
    camera_info_block.environmental_prefiltered_index = reflection_probe->cubemap_->GetTextureStorageIndex();
  }
}

void Camera::AppendGBufferColorAttachmentInfos(std::vector<VkRenderingAttachmentInfo>& attachment_infos,
                                               const VkAttachmentLoadOp load_op,
                                               const VkAttachmentStoreOp store_op) const {
  VkRenderingAttachmentInfo attachment{};
  attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;

  attachment.imageLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL;
  attachment.loadOp = load_op;
  attachment.storeOp = store_op;

  attachment.clearValue = {0, 0, 0, 0};
  attachment.imageView = g_buffer_normal_view_->GetVkImageView();
  attachment_infos.push_back(attachment);

  attachment.clearValue = {0, 0, 0, 0};
  attachment.imageView = g_buffer_material_view_->GetVkImageView();
  attachment_infos.push_back(attachment);
}

float Camera::GetSizeRatio() const {
  if (size_.x == 0 || size_.y == 0)
    return 0;
  return static_cast<float>(size_.x) / static_cast<float>(size_.y);
}

const std::shared_ptr<RenderTexture>& Camera::GetRenderTexture() const {
  return render_texture_;
}

glm::uvec2 Camera::GetSize() const {
  return size_;
}

void Camera::Resize(const glm::uvec2& size) {
  if (size.x == 0 || size.y == 0)
    return;
  if (size.x > 16384 || size.y >= 16384)
    return;
  if (size_ == size)
    return;
  size_ = size;
  if (render_texture_) {
    render_texture_->Resize({size_.x, size_.y, 1});
    UpdateGBuffer();
  }
}

void Camera::OnCreate() {
  size_ = glm::uvec2(1, 1);
  RenderTextureCreateInfo render_texture_create_info{};
  render_texture_create_info.extent.width = size_.x;
  render_texture_create_info.extent.height = size_.y;
  render_texture_create_info.extent.depth = 1;
  render_texture_ = std::make_unique<RenderTexture>(render_texture_create_info);

  g_buffer_descriptor_set_ = std::make_shared<DescriptorSet>(g_buffer_layout);

  post_processing_stack_ref = AssetManager::CreateTemporaryAsset<PostProcessingStack>();
  UpdateGBuffer();
}

bool Camera::Rendered() const {
  return rendered_;
}
void Camera::SetRequireRendering(const bool value) {
  require_rendering_ = require_rendering_ || value;
}

void Camera::CalculatePlanes(std::vector<Plane>& planes, const glm::mat4& projection, const glm::mat4& view) {
  glm::mat4 combo_matrix = projection * glm::transpose(view);
  planes[0].a = combo_matrix[3][0] + combo_matrix[0][0];
  planes[0].b = combo_matrix[3][1] + combo_matrix[0][1];
  planes[0].c = combo_matrix[3][2] + combo_matrix[0][2];
  planes[0].d = combo_matrix[3][3] + combo_matrix[0][3];

  planes[1].a = combo_matrix[3][0] - combo_matrix[0][0];
  planes[1].b = combo_matrix[3][1] - combo_matrix[0][1];
  planes[1].c = combo_matrix[3][2] - combo_matrix[0][2];
  planes[1].d = combo_matrix[3][3] - combo_matrix[0][3];

  planes[2].a = combo_matrix[3][0] - combo_matrix[1][0];
  planes[2].b = combo_matrix[3][1] - combo_matrix[1][1];
  planes[2].c = combo_matrix[3][2] - combo_matrix[1][2];
  planes[2].d = combo_matrix[3][3] - combo_matrix[1][3];

  planes[3].a = combo_matrix[3][0] + combo_matrix[1][0];
  planes[3].b = combo_matrix[3][1] + combo_matrix[1][1];
  planes[3].c = combo_matrix[3][2] + combo_matrix[1][2];
  planes[3].d = combo_matrix[3][3] + combo_matrix[1][3];

  planes[4].a = combo_matrix[3][0] + combo_matrix[2][0];
  planes[4].b = combo_matrix[3][1] + combo_matrix[2][1];
  planes[4].c = combo_matrix[3][2] + combo_matrix[2][2];
  planes[4].d = combo_matrix[3][3] + combo_matrix[2][3];

  planes[5].a = combo_matrix[3][0] - combo_matrix[2][0];
  planes[5].b = combo_matrix[3][1] - combo_matrix[2][1];
  planes[5].c = combo_matrix[3][2] - combo_matrix[2][2];
  planes[5].d = combo_matrix[3][3] - combo_matrix[2][3];

  planes[0].Normalize();
  planes[1].Normalize();
  planes[2].Normalize();
  planes[3].Normalize();
  planes[4].Normalize();
  planes[5].Normalize();
}

void Camera::CalculateFrustumPoints(const std::shared_ptr<Camera>& camera_component, float near_plane, float far_plane,
                                    glm::vec3 camera_pos, glm::quat camera_rot, glm::vec3* points) {
  const glm::vec3 front = camera_rot * glm::vec3(0, 0, -1);
  const glm::vec3 right = camera_rot * glm::vec3(1, 0, 0);
  const glm::vec3 up = camera_rot * glm::vec3(0, 1, 0);
  const glm::vec3 near_center = front * near_plane;
  const glm::vec3 far_center = front * far_plane;

  const float e = tanf(glm::radians(camera_component->camera_settings.fov * 0.5f));
  const float near_ext_y = e * near_plane;
  const float near_ext_x = near_ext_y * camera_component->GetSizeRatio();
  const float far_ext_y = e * far_plane;
  const float far_ext_x = far_ext_y * camera_component->GetSizeRatio();

  points[0] = camera_pos + near_center - right * near_ext_x - up * near_ext_y;
  points[1] = camera_pos + near_center - right * near_ext_x + up * near_ext_y;
  points[2] = camera_pos + near_center + right * near_ext_x + up * near_ext_y;
  points[3] = camera_pos + near_center + right * near_ext_x - up * near_ext_y;
  points[4] = camera_pos + far_center - right * far_ext_x - up * far_ext_y;
  points[5] = camera_pos + far_center - right * far_ext_x + up * far_ext_y;
  points[6] = camera_pos + far_center + right * far_ext_x + up * far_ext_y;
  points[7] = camera_pos + far_center + right * far_ext_x - up * far_ext_y;
}

glm::quat Camera::ProcessMouseMovement(float yaw_angle, float pitch_angle, bool constrain_pitch) {
  // Make sure that when pitch is out of bounds, screen doesn't get flipped
  if (constrain_pitch) {
    if (pitch_angle > 89.0f)
      pitch_angle = 89.0f;
    if (pitch_angle < -89.0f)
      pitch_angle = -89.0f;
  }

  glm::vec3 front;
  front.x = cos(glm::radians(yaw_angle)) * cos(glm::radians(pitch_angle));
  front.y = sin(glm::radians(pitch_angle));
  front.z = sin(glm::radians(yaw_angle)) * cos(glm::radians(pitch_angle));
  front = glm::normalize(front);
  const glm::vec3 right = glm::normalize(glm::cross(
      front, glm::vec3(0.0f, 1.0f, 0.0f)));  // Normalize the vectors, because their length gets closer to 0 the more
  // you look up or down which results in slower movement.
  const glm::vec3 up = glm::normalize(glm::cross(right, front));
  return glm::quatLookAt(front, up);
}

void Camera::ReverseAngle(const glm::quat& rotation, float& pitch_angle, float& yaw_angle,
                          const bool& constrain_pitch) {
  const auto angle = glm::degrees(glm::eulerAngles(rotation));
  pitch_angle = angle.x;
  // yawAngle = glm::abs(angle.z) > 90.0f ? 90.0f - angle.y : -90.0f - angle.y;
  glm::vec3 front = rotation * glm::vec3(0, 0, -1);
  front.y = 0;
  yaw_angle = glm::degrees(glm::acos(glm::dot(glm::vec3(0, 0, 1), glm::normalize(front))));
  if (constrain_pitch) {
    if (pitch_angle > 89.0f)
      pitch_angle = 89.0f;
    if (pitch_angle < -89.0f)
      pitch_angle = -89.0f;
  }
}
glm::mat4 Camera::GetProjection() const {
  return glm::perspective(glm::radians(camera_settings.fov * 0.5f), GetSizeRatio(), camera_settings.near_distance,
                          camera_settings.far_distance);
}

glm::vec3 Camera::GetMouseWorldPoint(GlobalTransform& ltw, glm::vec2 mouse_position) const {
  const float half_x = static_cast<float>(size_.x) / 2.0f;
  const float half_y = static_cast<float>(size_.y) / 2.0f;
  const auto start =
      glm::vec4(-1.0f * (mouse_position.x - half_x) / half_x, -1.0f * (mouse_position.y - half_y) / half_y, 0.0f, 1.0f);
  return start / start.w;
}

Ray Camera::ScreenPointToRay(GlobalTransform& ltw, glm::vec2 mouse_position) const {
  const auto position = ltw.GetPosition();
  const auto rotation = ltw.GetRotation();
  const glm::vec3 front = rotation * glm::vec3(0, 0, -1);
  const glm::vec3 up = rotation * glm::vec3(0, 1, 0);
  const auto view = glm::lookAt(position, position + front, up);
  const glm::mat4 inv = glm::inverse(GetProjection() * view);
  const float half_x = static_cast<float>(size_.x) / 2.0f;
  const float half_y = static_cast<float>(size_.y) / 2.0f;
  const auto real_x = (mouse_position.x - half_x) / half_x;
  const auto real_y = (mouse_position.y - half_y) / half_y;
  if (glm::abs(real_x) > 1.0f || glm::abs(real_y) > 1.0f)
    return {glm::vec3(FLT_MAX), glm::vec3(FLT_MAX)};
  auto start = glm::vec4(real_x, -1 * real_y, -1, 1.0);
  auto end = glm::vec4(real_x, -1.0f * real_y, 1.0f, 1.0f);
  start = inv * start;
  end = inv * end;
  start /= start.w;
  end /= end.w;
  const glm::vec3 dir = glm::normalize(glm::vec3(end - start));
  return {glm::vec3(ltw.value[3]) + camera_settings.near_distance * dir,
          glm::vec3(ltw.value[3]) + camera_settings.far_distance * dir};
}

void Camera::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "x" << YAML::Value << size_.x;

  out << YAML::Key << "y" << YAML::Value << size_.y;
  out << YAML::Key << "use_clear_color" << YAML::Value << camera_settings.use_clear_color;
  out << YAML::Key << "clear_color" << YAML::Value << camera_settings.clear_color;
  out << YAML::Key << "near_distance" << YAML::Value << camera_settings.near_distance;
  out << YAML::Key << "far_distance" << YAML::Value << camera_settings.far_distance;
  out << YAML::Key << "fov" << YAML::Value << camera_settings.fov;
  out << YAML::Key << "background_intensity" << YAML::Value << camera_settings.background_intensity;
  out << YAML::Key << "fade_ratio" << YAML::Value << camera_settings.fade_ratio;
  out << YAML::Key << "fade_factor" << YAML::Value << camera_settings.fade_factor;
  skybox.Save("skybox", out);
  post_processing_stack_ref.Save("post_processing_stack_ref", out);
}

void Camera::Deserialize(const YAML::Node& in) {
  if (in["use_clear_color"])
    camera_settings.use_clear_color = in["use_clear_color"].as<bool>();
  if (in["clear_color"])
    camera_settings.clear_color = in["clear_color"].as<glm::vec3>();
  if (in["near_distance"])
    camera_settings.near_distance = in["near_distance"].as<float>();
  if (in["far_distance"])
    camera_settings.far_distance = in["far_distance"].as<float>();
  if (in["fade_ratio"])
    camera_settings.fade_ratio = in["fade_ratio"].as<float>();
  if (in["fade_factor"])
    camera_settings.fade_factor = in["fade_factor"].as<float>();

  if (in["fov"])
    camera_settings.fov = in["fov"].as<float>();

  if (in["x"] && in["y"]) {
    int resolution_x = in["x"].as<int>();
    int resolution_y = in["y"].as<int>();
    Resize({resolution_x, resolution_y});
  }
  skybox.Load("skybox", in);
  post_processing_stack_ref.Load("post_processing_stack_ref", in);
  rendered_ = false;
  require_rendering_ = false;

  if (in["background_intensity"])
    camera_settings.background_intensity = in["background_intensity"].as<float>();
}

void Camera::OnDestroy() {
  post_processing_stack_ref.Clear();
  skybox.Clear();
}

bool Camera::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  uint32_t mode = static_cast<uint32_t>(camera_render_mode);
  if (ImGui::Combo("Render Mode", {"Rasterization", "Ray Tracing"}, mode)) {
    camera_render_mode = static_cast<CameraRenderMode>(mode);
    changed = true;
  }
  if (ImGui::DragFloat("Fade ratio", &camera_settings.fade_ratio, 0.01f, 0.01f, 1.0f)) {
    changed = true;
  }
  if (camera_settings.fade_ratio != 0.f) {
    if (ImGui::DragFloat("Fade factor", &camera_settings.fade_factor, 0.01f, 0.01f, 1.0f)) {
      changed = true;
    }
  }
  if (ImGui::TreeNode("Debug")) {
    require_rendering_ = true;
    static bool external_window = false;

    ImGui::Checkbox("Display in external window", &external_window);

    static float debug_scale = 0.25f;
    if (rendered_) {
      if (external_window) {
        if (ImGui::Begin("Camera Debug")) {
          ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 1.0f);
          debug_scale = glm::clamp(debug_scale, 0.1f, 1.0f);
          DebugViews(debug_scale);
        }
        ImGui::End();
      } else {
        ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 1.0f);
        debug_scale = glm::clamp(debug_scale, 0.1f, 1.0f);
        DebugViews(debug_scale);
      }
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Background", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::DragFloat("Intensity", &camera_settings.background_intensity, 0.01f, 0.0f, 10.f)) {
      changed = true;
    }
    if (ImGui::Checkbox("Use clear color", &camera_settings.use_clear_color)) {
      changed = true;
    }
    if (camera_settings.use_clear_color) {
      if (ImGui::ColorEdit3("Clear Color", (float*)(void*)&camera_settings.clear_color)) {
        changed = true;
      }
    } else if (editor_layer->DragAndDropButton<Cubemap>(skybox, "Skybox")) {
      changed = true;
    }
    ImGui::TreePop();
  }

  if (const auto scene = GetScene()) {
    const bool saved_state = (this == scene->main_camera.Get<Camera>().get());
    bool is_main_camera = saved_state;
    ImGui::Checkbox("Main Camera", &is_main_camera);
    if (saved_state != is_main_camera) {
      changed = true;
      if (is_main_camera) {
        scene->main_camera = scene->GetOrSetPrivateComponent<Camera>(GetOwner()).lock();
      } else {
        Application::GetActiveScene()->main_camera.Clear();
      }
    }
    if (!is_main_camera || !Application::GetLayer<EditorLayer>()->main_camera_allow_auto_resize) {
      glm::ivec2 resolution = {size_.x, size_.y};
      if (ImGui::DragInt2("Resolution", &resolution.x, 1, 1, 4096)) {
        Resize({resolution.x, resolution.y});
      }
    }
  }
  if (editor_layer->DragAndDropButton<PostProcessingStack>(post_processing_stack_ref, "PostProcessingStack")) {
    changed = true;
  }
  const auto pps = post_processing_stack_ref.Get<PostProcessingStack>();
  if (ImGui::TreeNode("Intrinsic Settings")) {
    if (ImGui::DragFloat("Near", &camera_settings.near_distance, camera_settings.near_distance / 10.0f, 0,
                         camera_settings.far_distance)) {
      changed = true;
    }
    if (ImGui::DragFloat("Far", &camera_settings.far_distance, camera_settings.far_distance / 10.0f,
                         camera_settings.near_distance)) {
      changed = true;
    }
    if (ImGui::DragFloat("FOV", &camera_settings.fov, 1.0f, 1, 359)) {
      changed = true;
    }
    ImGui::TreePop();
  }
  FileUtils::SaveFile(
      "ScreenShot", "Image", {".png", ".jpg", ".hdr"},
      [this](const std::filesystem::path& file_path) {
        render_texture_->Save(file_path);
      },
      false);

  return changed;
}

void Camera::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(skybox);
  list.push_back(post_processing_stack_ref);
}

const std::shared_ptr<DescriptorSet>& Camera::GetGBufferDescriptorSet() const {
  return g_buffer_descriptor_set_;
}

void Camera::DebugViews(const float debug_scale) const {
  if (ImGui::TreeNodeEx("Normal", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Image(g_buffer_normal_im_texture_id_, ImVec2(size_.x * debug_scale, size_.y * debug_scale), ImVec2(0, 1),
                 ImVec2(1, 0));
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("UV", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Image(g_buffer_material_tex_coord_im_texture_id_, ImVec2(size_.x * debug_scale, size_.y * debug_scale),
                 ImVec2(0, 1), ImVec2(1, 0));
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Instance/Material Index")) {
    ImGui::Image(g_buffer_material_indices_im_texture_id_, ImVec2(size_.x * debug_scale, size_.y * debug_scale),
                 ImVec2(0, 1), ImVec2(1, 0));
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Depth")) {
    ImGui::Image(render_texture_->GetDepthImTextureId(), ImVec2(size_.x * debug_scale, size_.y * debug_scale),
                 ImVec2(0, 1), ImVec2(1, 0));
    ImGui::TreePop();
  }
}
