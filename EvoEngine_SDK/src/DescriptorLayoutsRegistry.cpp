#include "Platform.hpp"
#include "Utilities.hpp"
using namespace evo_engine;

void Platform::PrepareDescriptorSetLayouts() {
  const auto render_texture_present = std::make_shared<DescriptorSetLayout>();
  render_texture_present->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                              VK_SHADER_STAGE_FRAGMENT_BIT, 0);
  render_texture_present->Initialize();
  RegisterDescriptorSetLayout("RENDER_TEXTURE_PRESENT_LAYOUT", render_texture_present);

  const auto per_frame_layout = std::make_shared<DescriptorSetLayout>();
  per_frame_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_ALL, 0);
  per_frame_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_ALL, 0);
  per_frame_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
  per_frame_layout->PushDescriptorBinding(3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
  per_frame_layout->PushDescriptorBinding(4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
  per_frame_layout->PushDescriptorBinding(5, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_ALL, 0);
  per_frame_layout->PushDescriptorBinding(6, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
  per_frame_layout->PushDescriptorBinding(7, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
  per_frame_layout->PushDescriptorBinding(8, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
  per_frame_layout->PushDescriptorBinding(9, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_MESH_BIT_EXT, 0);
  per_frame_layout->PushDescriptorBinding(10, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_MESH_BIT_EXT, 0);

  per_frame_layout->PushDescriptorBinding(12, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_MESH_BIT_EXT, 0);


  per_frame_layout->PushDescriptorBinding(13, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                        VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT,
                                        Settings::max_texture_2d_resource_size);
  per_frame_layout->PushDescriptorBinding(14, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                        VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT, Settings::max_cubemap_resource_size);

  per_frame_layout->Initialize();
  RegisterDescriptorSetLayout("PER_FRAME_LAYOUT", per_frame_layout);

  const auto light_layout = std::make_shared<DescriptorSetLayout>();
  light_layout->PushDescriptorBinding(15, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 0);
  light_layout->PushDescriptorBinding(16, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 0);
  light_layout->PushDescriptorBinding(17, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 0);
  light_layout->Initialize();
  RegisterDescriptorSetLayout("LIGHTING_LAYOUT", light_layout);

  const auto bone_matrices_layout = std::make_shared<DescriptorSetLayout>();
  bone_matrices_layout->PushDescriptorBinding(18, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_VERTEX_BIT, 0);
  bone_matrices_layout->Initialize();
  RegisterDescriptorSetLayout("BONE_MATRICES_LAYOUT", bone_matrices_layout);

  const auto instanced_data_layout = std::make_shared<DescriptorSetLayout>();
  instanced_data_layout->PushDescriptorBinding(18, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
  instanced_data_layout->Initialize();
  RegisterDescriptorSetLayout("INSTANCED_DATA_LAYOUT", instanced_data_layout);

  const auto camera_g_buffer_layout = std::make_shared<DescriptorSetLayout>();
  camera_g_buffer_layout->PushDescriptorBinding(18, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                             VK_SHADER_STAGE_FRAGMENT_BIT, 0);
  camera_g_buffer_layout->PushDescriptorBinding(19, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                             VK_SHADER_STAGE_FRAGMENT_BIT, 0);
  camera_g_buffer_layout->PushDescriptorBinding(20, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                             VK_SHADER_STAGE_FRAGMENT_BIT, 0);
  camera_g_buffer_layout->PushDescriptorBinding(21, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                             VK_SHADER_STAGE_FRAGMENT_BIT, 0);
  camera_g_buffer_layout->Initialize();
  RegisterDescriptorSetLayout("CAMERA_GBUFFER_LAYOUT", camera_g_buffer_layout);

  const auto ssr_reflect_layout = std::make_shared<DescriptorSetLayout>();
  ssr_reflect_layout->PushDescriptorBinding(18, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                          0);
  ssr_reflect_layout->PushDescriptorBinding(19, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                          0);
  ssr_reflect_layout->PushDescriptorBinding(20, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                          0);
  ssr_reflect_layout->PushDescriptorBinding(21, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                          0);
  ssr_reflect_layout->Initialize();
  RegisterDescriptorSetLayout("SSR_REFLECT_LAYOUT", ssr_reflect_layout);

  const auto ssr_blur = std::make_shared<DescriptorSetLayout>();
  ssr_blur->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 0);
  ssr_blur->Initialize();
  RegisterDescriptorSetLayout("SSR_BLUR_LAYOUT", ssr_blur);

  const auto ssr_combine_layout = std::make_shared<DescriptorSetLayout>();
  ssr_combine_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                          0);
  ssr_combine_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                          0);
  ssr_combine_layout->Initialize();
  RegisterDescriptorSetLayout("SSR_COMBINE_LAYOUT", ssr_combine_layout);
}
