#include "GpuRayTracerCamera.hpp"

#include "ClassRegistry.hpp"
#include "CpuRayTracer.hpp"
#include "EditorLayer.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Shader.hpp"
using namespace evo_engine;

void GpuRayTracerCamera::OnCreate() {
  /**
   * When this PrivateComponent is created, we also create a new texture2d asset to accept the rendered results from GPU
   * Ray Tracer.
   */
  texture_ref = ProjectManager::CreateTemporaryAsset<Texture2D>();
  texture_ref.Get<Texture2D>()->SetResolution(resolution, false);
}

void GpuRayTracerCamera::LateUpdate() {
  /**
   * If per_frame_capture is set to TRUE, we will trace scene for every frame.
   * Reason we are doing it in LateUpdate, is because the render_instances_list will only be updated right before
   * LateUpdate during a frame. RenderLayer will collect all MeshRenderers in the scene and build a collections that
   * represents what needs to be rendered for current frame.
   *
   * If you know about GPU driven rendering or indirect rendering, you will understand that it's a general practice to
   * summarize everything (geometry, textures, materials, transforms, etc.) before dispatching the first draw call of a
   * frame instead of draw things once you have it.
   */
  if (const auto texture = texture_ref.Get<Texture2D>(); texture && per_frame_capture) {
    Capture();
  }
}

/**
 * \brief Descriptor set layout tells compute pipeline the collection of resources (textures, buffers, etc) that we'll
 * use during compute task. Here we have 2 set layouts, having multiple different layouts allows you sharing layout
 * between pipelines. One may use aggregated_scene for a ray tracing camera, other may use it for simulating point cloud
 * scanner, where you don't need to have a texture to take output but a buffer instead.
 */
std::shared_ptr<DescriptorSetLayout> aggregated_scene_descriptor_set_layout;
std::shared_ptr<DescriptorSetLayout> output_texture_descriptor_set_layout;
/**
 * \brief The compute shader. Code is located at .../EvoEngine_SDK/Internals/Shaders/Compute/RayTracerCamera.comp
 */
std::shared_ptr<Shader> ray_tracer_camera_shader{};
/**
 * \brief The compute shader pipeline. In Vulkan we have compute pipeline, graphics pipeline, and ray tracer pipeline. A
 * pipeline represents a combination of shaders for a dedicated task on dedicated inputs(resources) and outputs. Since
 * in compute shader you have just one shader, so for each different shader you setup individual compute pipeline.
 */
std::shared_ptr<ComputePipeline> ray_tracer_camera_pipeline{};

struct CameraConfig {
  glm::vec4 resolution_xy_sample_bounce;
  glm::mat4 inverse_projection_view;
};
void GpuRayTracerCamera::Capture() {
  /**
   * You may take a look at render instances, to see what it contains. RenderLayer will prepare a RenderInstance every
   * frame that contains all needed information for rendering everything for current scene. It's used in rasterization
   * rendering, and here we also use it for ray tracing. It also detects updates of the scene, like transformation,
   * mesh, material changes.
   */
  std::shared_ptr<RenderInstances> render_instances;
  if (const auto render_layer = Application::GetLayer<RenderLayer>()) {
    render_instances = render_layer->render_instances_list[Platform::GetCurrentFrameIndex()];
  } else
    return;
  CameraInfoBlock camera_info_block;
  const auto scene = GetScene();
  const auto owner = GetOwner();
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(owner);
  UpdateCameraInfoBlock(camera_info_block, global_transform);
  CpuRayTracer cpu_ray_tracer;
  /**
   * During this step, the cpu_ray_tracer will scan all MeshRendereres in the scene, and establish TLAS and BLAS based
   * on them.
   */
  cpu_ray_tracer.Initialize(
      render_instances,
      [&](uint32_t, const std::shared_ptr<Mesh>&) {

      },
      [&](const uint32_t node_index, const Entity& entity) {

      });
  /**
   * The cpu_ray_tracer will aggregate and flatten TLAS and BLAS so from its hierarcal structure to vectors so we can
   * use it on GPU.
   */
  auto aggregate_scene = cpu_ray_tracer.Aggregate();
  /**
   * Upload prepared data to GPU, these data will be linked to the compute pipeline via Descriptors (collectively
   * DescriptorSet) so we can read them in shader. You may take a look at its implementation to see how easy to send
   * data to GPU.
   */
  aggregate_scene.InitializeBuffers();
  /**
   * We need to set up descriptor set layout(s) for compute pipeline.
   * Descriptor set layout tells compute pipeline the collection of resources (textures, buffers, etc) that we'll use
   * during compute task.
   * In Vulkan you need to set up everything beforhand so it can help you validate if the layout matches the layout you
   * use in shader code. Later on, you also need to bind corresponding DescriptorSet to the compute pipeline when you
   * actually dispatch compute task, and Vulkan will then check if it matches the DescriptorSetLayout(s).
   */
  if (!aggregated_scene_descriptor_set_layout) {
    aggregated_scene_descriptor_set_layout = std::make_shared<DescriptorSetLayout>();

    /**
     * This corresponds to SCENE_GRAPH_DATA_BINDING in RayTracerCamera.comp, you can find the resource in
     * .../Includes/Trace.glsl This contains TLAS for ray tracing
     */
    aggregated_scene_descriptor_set_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                                  VK_SHADER_STAGE_COMPUTE_BIT, 0);
    /**
     * This corresponds to SCENE_GEOMETRY_DATA_BINDING in RayTracerCamera.comp, you can find the resource in
     * .../Includes/Trace.glsl This contains BLAS for ray tracing
     */
    aggregated_scene_descriptor_set_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                                  VK_SHADER_STAGE_COMPUTE_BIT, 0);
    /**
     * This corresponds to SCENE_INFO_BLOCK_BINDING in RayTracerCamera.comp, you can find the resource in
     * .../Includes/Trace.glsl This contains extra metadata for ray tracing
     */
    aggregated_scene_descriptor_set_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                                  VK_SHADER_STAGE_COMPUTE_BIT, 0);
    /**
     * This corresponds to EE_MATERIAL_BLOCK_BINDING in RayTracerCamera.comp, you can find the resource in
     * .../Includes/Materials.glsl It contains all material properties, like PBR texture indices, albedo, roughness,
     * etc.
     */
    aggregated_scene_descriptor_set_layout->PushDescriptorBinding(3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                                  VK_SHADER_STAGE_COMPUTE_BIT, 0);
    /**
     * This corresponds to EE_INSTANCE_BLOCK_BINDING in RayTracerCamera.comp, you can find the resource in
     * .../Includes/Instances.glsl It contains all render instances information, like the material index, etc.
     */
    aggregated_scene_descriptor_set_layout->PushDescriptorBinding(4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                                  VK_SHADER_STAGE_COMPUTE_BIT, 0);

    /**
     * This corresponds to EE_TEXTURE_2DS_BINDING in RayTracerCamera.comp, you can find the resource in
     * .../Includes/Textures.glsl It contains all 2d textures within the framework. Material contains index for choosing
     * correct texture for rendering
     */
    aggregated_scene_descriptor_set_layout->PushDescriptorBinding(
        6, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT,
        VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT, Platform::Settings::max_texture_2d_resource_size);
    /**
     * This corresponds to EE_CUBEMAPS_BINDING in RayTracerCamera.comp, you can find the resource in
     * .../Includes/Textures.glsl It contains all cubemaps within the framework.
     */
    aggregated_scene_descriptor_set_layout->PushDescriptorBinding(
        7, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT,
        VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT, Platform::Settings::max_cubemap_resource_size);
    /**
     * Once we nominate all descriptors(resources) we are going to use, we initialize the descriptor set layout.
     */
    aggregated_scene_descriptor_set_layout->Initialize();

    output_texture_descriptor_set_layout = std::make_shared<DescriptorSetLayout>();
    /**
     * This corresponds to "result_image" in RayTracerCamera.comp.
     * This is the output texture.
     */
    output_texture_descriptor_set_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
                                                                VK_SHADER_STAGE_COMPUTE_BIT, 0);
    output_texture_descriptor_set_layout->Initialize();
  }
  if (!ray_tracer_camera_shader) {
    /**
     * Here we load shader.
     */
    ray_tracer_camera_shader = ProjectManager::CreateTemporaryAsset<Shader>();
    /**
     * Load shader from path. Note the the CMake will copy the everything under .../EvoEngine_SDK/Internals/ to the
     * executable folder after compilation. If you are going to write your own shader, put it under
     * .../EvoEngine_Plugins/EcoSysLab/Internals/EcoSysLabResources/Shaders/ and CMake will also copy it to the
     * excutable folder. In that case, you will write something like: xxx_shader->Set(ShaderType::Compute,
     * std::filesystem::path("./EcoSysLabResources/Shaders/Compute/AlphaShape.comp"));
     */
    ray_tracer_camera_shader->Set(ShaderType::Compute,
                                  std::filesystem::path("./DefaultResources") / "Shaders/Compute/RayTracerCamera.comp");
  }
  /**
   * Next, with description of resources and shader in place, we set up compute pipeline.
   *
   */
  if (!ray_tracer_camera_pipeline) {
    ray_tracer_camera_pipeline = std::make_shared<ComputePipeline>();
    /**
     * We feed 2 descriptor layouts to our pipeline.
     */
    ray_tracer_camera_pipeline->descriptor_set_layouts.emplace_back(aggregated_scene_descriptor_set_layout);
    ray_tracer_camera_pipeline->descriptor_set_layouts.emplace_back(output_texture_descriptor_set_layout);

    /**
     * In shader we also have trival data that changes every time so here we use push constant instead of considering
     * them as resources. Push constant should generally be less than 128 bytes. Here we feed camera matrices since it's
     * constantly changing.
     */
    auto& push_constant_range = ray_tracer_camera_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(CameraConfig);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    /**
     * Also link the shader we are going to use to this pipeline.
     */
    ray_tracer_camera_pipeline->compute_shader = ray_tracer_camera_shader;
    /**
     * Finally, with input (descriptor set layout), task (shader), and output (descriptor set layout) in place, we
     * initialize compute pipeline.
     */
    ray_tracer_camera_pipeline->Initialize();
  }
  /**
   * Now, we are going to prepare/upload the actual data for our task. (Setting up descriptor sets)
   */
  const auto ray_tracer_descriptor_set = std::make_shared<DescriptorSet>(aggregated_scene_descriptor_set_layout);
  /**
   * We need to upload material/instance data to GPU with buffer. First we create buffer and then we upload data.
   */
  VkBufferCreateInfo buffer_create_info{};
  /**
   * You must set the type to VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO.
   */
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  /**
   * You have to nominate all potential usages for this buffer as they can't change in future.
   * VK_BUFFER_USAGE_TRANSFER_DST_BIT: This means we are going to upload data to this buffer from CPU.
   * VK_BUFFER_USAGE_STORAGE_BUFFER_BIT: This means we are going to set this buffer as a storage buffer so we can
   * read/modify it in shader as buffer.
   */
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  /**
   * When you create a buffer, you don't need to calculate the size, since the UploadBuffer() will auto resize the
   * buffer for you. But you can't create zero size buffer. So we set it as 1.
   */
  buffer_create_info.size = 1;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  /**
   * This means we want the buffer to sit on GPU memory.
   */
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto material_info_descriptor_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  const auto instance_info_descriptor_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  /**
   * Upload data from render instances. Material and instance information is already prepared in RenderLayer befor our
   * LateUpdate(). UploadVector() will auto resize buffer to fit what you need to upload. We also have Upload() if you
   * just want to upload single instance, or a chunk of memory.
   */
  material_info_descriptor_buffer->UploadVector(render_instances->GetMaterialInfoBlocks());
  instance_info_descriptor_buffer->UploadVector(render_instances->GetInstanceInfoBlocks());
  /**
   * We need to bind the buffers/textures to the decriptor set so when we send the descriptor set to the pipeline, it
   * knows where to find these resources. This function takes the binding point and the corresponding resource as
   * parameters.
   */
  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(0, aggregate_scene.aggregate_scene_graph_buffer);
  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(1, aggregate_scene.aggregate_scene_geometry_buffer);
  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(2, aggregate_scene.aggregate_scene_info_buffer);
  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(3, material_info_descriptor_buffer);
  ray_tracer_descriptor_set->UpdateBufferDescriptorBinding(4, instance_info_descriptor_buffer);
  /**
   * This is doing the same thing, but TextureStorage hides it's resources for protection so you can only give it the
   * descriptor set and binding point so it can bind by itself.
   */
  TextureStorage::BindTexture2DToDescriptorSet(ray_tracer_descriptor_set, 6);
  TextureStorage::BindCubemapToDescriptorSet(ray_tracer_descriptor_set, 7);

  /**
   * The second descriptor set, for taking output texture.
   */
  const auto output_texture_descriptor_set = std::make_shared<DescriptorSet>(output_texture_descriptor_set_layout);
  /**
   * Retrieve actual asset ptr from AssetRef.
   */
  const auto texture2d = texture_ref.Get<Texture2D>();
  /**
   * As you may know, contrary to OpenGL, where every glXXX command is sync and it will block GPU untill it finishes,
   * most of operations in Vulkan is async.
   * This means you record commands (APIs start with vkCmdXXX) to a command buffer, add it to a command queue, submit it
   * to GPU, and continue on something else and comeback later for results. However, sometimes we want commands that
   * finishes now because we have things on CPU that depends on it. So Platform::ImmediateSubmit is the way to go.
   *
   * In Vulkan, textures have different layouts. Layout helps GPU understand what's the task for this texture, like
   * reading in a shader, use it as output in framebuffer, send it to swapchain for display,etc. You need to transit the
   * texture to correct layout before performing certain task.
   *
   * Layout transition is not blocking, because you may transit image layout multiple times before getting back to GPU.
   * For example, one may render something on a texture, and then use this texture as sampler on rendering something
   * else. Having these taken care of without getting back to CPU everytime it needs transition save lots of time.
   *
   * But here, we need to transit this output image immediately because we are going to link it to a descriptor set
   * which will check if it's in correct layout. So we use Platform::ImmediateSubmit, which basically is to setup a
   * temporary command buffer, and you record commands(transit image layout), and it will submit it and block CPU until
   * it's done on GPU.
   */
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    texture2d->GetImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
  });
  /**
   * Now we can safely finishing set up the decriptor set for output texture.
   */
  VkDescriptorImageInfo image_info;
  image_info.imageLayout = texture2d->GetImage()->GetLayout();
  image_info.imageView = texture2d->GetVkImageView();
  image_info.sampler = texture2d->GetVkSampler();
  output_texture_descriptor_set->UpdateImageDescriptorBinding(0, image_info);

  /**
   * Finally, let's dispatch compute task!
   */
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    /**
     * First thing first, you need to tell GPU that we are going to use our compute pipeline.
     */
    ray_tracer_camera_pipeline->Bind(vk_command_buffer);
    /**
     * Now, we bind 2 descriptor sets.
     */
    ray_tracer_camera_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                  ray_tracer_descriptor_set->GetVkDescriptorSet());
    ray_tracer_camera_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                  output_texture_descriptor_set->GetVkDescriptorSet());
    /**
     * Then, we set up and upload Push Constant, as you can see, it's very straightforward, no need to setup
     * buffer->descriptorset, but it's also limited in capabilities.
     */
    CameraConfig camera_config;
    camera_config.resolution_xy_sample_bounce = glm::vec4(
        glm::uintBitsToFloat(resolution.x), glm::uintBitsToFloat(resolution.y),
        glm::uintBitsToFloat(capture_parameters.sampled_count), glm::uintBitsToFloat(capture_parameters.bounce));
    camera_config.inverse_projection_view = camera_info_block.inverse_projection_view;
    ray_tracer_camera_pipeline->PushConstant(vk_command_buffer, 0, camera_config);

    /**
     * Dispatch!
     */
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(resolution.x * resolution.y, 256), 1, 1);
    /**
     * Remember, many of vulkan commands are executed without ordering. So we have this Platform::EverythingBarrier() to
     * make sure that the above commands finishes before moving on. This is syncronization on GPU, not between GPU and
     * CPU.
     */
    Platform::EverythingBarrier(vk_command_buffer);
    /**
     * We need to transit the image layout to VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL for other codes that use it for
     * rendering (Typically, for ImGui).
     */
    texture2d->GetImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });
  capture_parameters.sampled_count++;
}

void GpuRayTracerCamera::OnDestroy() {
  texture_ref.Clear();
}

float GpuRayTracerCamera::GetSizeRatio() const {
  if (resolution.x == 0 || resolution.y == 0)
    return 0;
  return static_cast<float>(resolution.x) / static_cast<float>(resolution.y);
}

void GpuRayTracerCamera::UpdateCameraInfoBlock(CameraInfoBlock& camera_info_block,
                                               const GlobalTransform& global_transform) const {
  const auto rotation = global_transform.GetRotation();
  const auto position = global_transform.GetPosition();
  const glm::vec3 front = rotation * glm::vec3(0, 0, -1);
  const glm::vec3 up = rotation * glm::vec3(0, 1, 0);
  const auto ratio = GetSizeRatio();
  camera_info_block.projection = glm::perspective(glm::radians(fov * 0.5f), ratio, near_distance, far_distance);
  camera_info_block.view = glm::lookAt(position, position + front, up);
  camera_info_block.projection_view = camera_info_block.projection * camera_info_block.view;
  camera_info_block.inverse_projection = glm::inverse(camera_info_block.projection);
  camera_info_block.inverse_view = glm::inverse(camera_info_block.view);
  camera_info_block.inverse_projection_view = glm::inverse(camera_info_block.projection * camera_info_block.view);
  camera_info_block.reserved_parameters1 =
      glm::vec4(near_distance, far_distance, glm::tan(glm::radians(fov * 0.5f)), glm::tan(glm::radians(fov * 0.25f)));
}

bool GpuRayTracerCamera::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (EditorLayer::DragAndDropButton<Texture2D>(texture_ref, "Target texture"))
    changed = true;

  static glm::ivec2 new_resolution = {128, 128};
  if (ImGui::DragInt2("Resolution", &new_resolution.x, 1, 1, 4096)) {
    capture_parameters.sampled_count = 0;
    resolution = new_resolution;
    texture_ref.Get<Texture2D>()->SetResolution(resolution, false);
  }
  ImGui::Checkbox("Per frame capture", &per_frame_capture);
  if (!per_frame_capture || !Application::IsPlaying()) {
    if (ImGui::Button("Capture")) {
      Capture();
    }
  }
  ImGui::Text((std::string("Accumulation count: ") + std::to_string(capture_parameters.sampled_count)).c_str());
  if (ImGui::Button("Restart accumulation")) {
    capture_parameters.sampled_count = 0;
  }
  if (const auto texture2d = texture_ref.Get<Texture2D>(); texture2d) {
    const auto texture_storage = texture2d->PeekTexture2DStorage();
    if (texture_storage.im_texture_id) {
      static float debug_scale = 0.25f;
      ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 10.0f);
      debug_scale = glm::clamp(debug_scale, 0.1f, 10.0f);
      ImGui::Image(texture_storage.im_texture_id,
                   ImVec2(texture_storage.image->GetExtent().width * debug_scale,
                          texture_storage.image->GetExtent().height * debug_scale),
                   ImVec2(0, 1), ImVec2(1, 0));
    }
  }
  return changed;
}