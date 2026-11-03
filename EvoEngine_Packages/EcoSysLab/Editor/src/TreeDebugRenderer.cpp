#include "TreeDebugRenderer.hpp"

#include <cstdlib>
#include "BranchRenderer.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "GpuTreePhysics.hpp"
#include "Profiler.hpp"
#include "Shader.hpp"
#include "TreeDescriptor.hpp"
#include "TreeVisualizer.hpp"

using namespace eco_sys_lab_package;

bool TreeDebugRenderer::Prepare() {
  if (!Platform::MeshShaderEnabled())
    return false;
  if (!pipeline_) {
    layout_ = std::make_shared<DescriptorSetLayout>();
    for (uint32_t i = 0; i < 5; ++i)
      layout_->PushDescriptorBinding(
          i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
          VK_SHADER_STAGE_TASK_BIT_EXT | VK_SHADER_STAGE_MESH_BIT_EXT | VK_SHADER_STAGE_FRAGMENT_BIT, 0);
    layout_->Initialize();
    deform_layout_ = std::make_shared<DescriptorSetLayout>();
    for (uint32_t i = 0; i < 5; ++i)
      deform_layout_->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    deform_layout_->Initialize();
    deform_pipeline_ = std::make_shared<ComputePipeline>();
    deform_pipeline_->compute_shader =
        Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                "./EcoSysLabResources/Shaders/Compute/DeformTreeNodes.slang");
    deform_pipeline_->descriptor_set_layouts = {deform_layout_};
    deform_pipeline_->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(GpuTreeDeformParameters)});
    deform_pipeline_->Initialize();
    pipeline_ = std::make_shared<GraphicsPipeline>();
    const auto defines = Platform::GetShaderGlobalDefines() + "\n#define EE_TREE_DEBUG 1\n";
    pipeline_->task_shader =
        Shader::CreateTemporary(ShaderType::Task, defines, "./EcoSysLabResources/Shaders/Graphics/Task/TreeMesh.slang");
    pipeline_->mesh_shader =
        Shader::CreateTemporary(ShaderType::Mesh, defines, "./EcoSysLabResources/Shaders/Graphics/Mesh/TreeMesh.slang");
    pipeline_->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, defines, "./EcoSysLabResources/Shaders/Graphics/Fragment/TreeDebug.slang");
    const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
    pipeline_->descriptor_set_layouts = {render->GetEmptyDescriptorSetLayout(), render->GetEmptyDescriptorSetLayout(),
                                         render->GetEmptyDescriptorSetLayout(), layout_};
    pipeline_->vertex_input_enabled = false;
    pipeline_->geometry_type = GeometryType::Mesh;
    pipeline_->depth_attachment_format = Platform::Constants::render_texture_depth;
    pipeline_->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    pipeline_->color_attachment_formats = {Platform::Constants::render_texture_color, VK_FORMAT_R32G32B32A32_UINT};
    pipeline_->push_constant_ranges.push_back({VK_SHADER_STAGE_ALL, 0, sizeof(GpuTreeDrawParameters)});
    pipeline_->Initialize();
    impostor_pipeline_ = std::make_shared<GraphicsPipeline>();
    impostor_pipeline_->task_shader = Shader::CreateTemporary(
        ShaderType::Task, defines, "./EcoSysLabResources/Shaders/Graphics/Task/TreeMeshBatched.slang");
    impostor_pipeline_->mesh_shader = Shader::CreateTemporary(
        ShaderType::Mesh, defines, "./EcoSysLabResources/Shaders/Graphics/Mesh/TreeImpostor.slang");
    impostor_pipeline_->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, defines, "./EcoSysLabResources/Shaders/Graphics/Fragment/TreeDebugImpostor.slang");
    impostor_pipeline_->descriptor_set_layouts = pipeline_->descriptor_set_layouts;
    impostor_pipeline_->vertex_input_enabled = false;
    impostor_pipeline_->geometry_type = GeometryType::Mesh;
    impostor_pipeline_->depth_attachment_format = pipeline_->depth_attachment_format;
    impostor_pipeline_->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    impostor_pipeline_->color_attachment_formats = pipeline_->color_attachment_formats;
    impostor_pipeline_->push_constant_ranges = pipeline_->push_constant_ranges;
    impostor_pipeline_->Initialize();
  }
  if (frames_.empty()) {
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = sizeof(GpuTreeNode);
    info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    VmaAllocationCreateInfo device{};
    device.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
    VmaAllocationCreateInfo host{};
    host.usage = VMA_MEMORY_USAGE_AUTO_PREFER_HOST;
    host.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;
    host.requiredFlags = VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
    frames_.resize(Platform::GetMaxFramesInFlight());
    for (auto& frame : frames_) {
      frame.nodes = std::make_shared<Buffer>(info, device);
      frame.rest = std::make_shared<Buffer>(info, device);
      frame.colors = std::make_shared<Buffer>(info, device);
      info.size = sizeof(GpuTreeDebugState);
      frame.state = std::make_shared<Buffer>(info, host);
      info.size = sizeof(glm::uvec4);
      info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
      frame.readback = std::make_shared<Buffer>(info, host);
      info.size = sizeof(GpuTreeNode);
      info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
      frame.draw_set = std::make_shared<DescriptorSet>(layout_);
      frame.deform_set = std::make_shared<DescriptorSet>(deform_layout_);
    }
  }
  return pipeline_->Initialized() && impostor_pipeline_->Initialized() && deform_pipeline_->Initialized();
}

void TreeDebugRenderer::PollPick(const std::shared_ptr<Tree>& tree, ShootVisualizer& shoot, RootVisualizer& root) {
  // Frame tickets resolve after their fence completes; polling does not wait for the GPU.
  for (auto& frame : frames_) {
    if (!frame.submission || frame.submission->status == FrameSubmissionState::Status::Pending)
      continue;
    const bool valid = frame.submission->status == FrameSubmissionState::Status::Submitted &&
                       frame.pick_serial == request_serial_ && frame.pick_generation == generation_ &&
                       cached_tree_.lock() == tree && shoot_revision_ == tree->GetShootModelRevision().content &&
                       root_revision_ == tree->GetRootModelRevision().content &&
                       shoot_checkpoint_ == shoot.checkpoint_iteration && root_checkpoint_ == root.checkpoint_iteration;
    frame.submission.reset();
    if (!valid)
      continue;
    const auto& shoots = tree->shoot_model.PeekShootSkeleton(shoot.checkpoint_iteration);
    const auto& roots = tree->root_model.PeekRootSkeleton(root.checkpoint_iteration);
    const auto pick =
        DecodeGpuTreeDebugPick(*static_cast<const glm::uvec4*>(frame.readback->GetVmaAllocationInfo().pMappedData),
                               shoots.PeekRawNodes().size(), roots.PeekRawNodes().size());
    if (!pick)
      continue;
    auto apply = [&](auto& visualizer, const auto& skeleton) {
      const auto& active = skeleton.PeekSortedNodeList();
      if (std::find(active.begin(), active.end(), static_cast<int>(pick->node)) == active.end())
        return;
      visualizer.SetSelectedNode(skeleton, static_cast<int>(pick->node));
      visualizer.selected_node_length_factor = pick->length_factor;
      visualizer.need_update = true;
    };
    if (pick->shoot)
      apply(shoot, shoots);
    else
      apply(root, roots);
  }
}

void TreeDebugRenderer::Render(const std::shared_ptr<Tree>& tree, ShootVisualizer& shoot, RootVisualizer& root,
                               const std::shared_ptr<Camera>& camera, const glm::mat4& projection_view,
                               const bool context) {
  const ProfilerScope cpu_scope("EcoSysLab / Plant Visual Tree", "EcoSysLab");
  const int shoot_checkpoint = context ? tree->shoot_model.CurrentIteration() : shoot.checkpoint_iteration;
  const int root_checkpoint = context ? tree->root_model.CurrentIteration() : root.checkpoint_iteration;
  if (!Prepare()) {
    click_.reset();
    return;
  }
  if (requested_tree_.lock() != tree)
    click_.reset();
  const auto scene = tree->GetScene();
  const auto physics = scene->HasPrivateComponent<DynamicTreeSkeleton>(tree->GetOwner())
                           ? scene->GetOrSetPrivateComponent<DynamicTreeSkeleton>(tree->GetOwner()).lock()
                           : nullptr;
  const auto gpu = physics && physics->IsEnabled() && physics->HasPhysics(*tree) &&
                           shoot_checkpoint == tree->shoot_model.CurrentIteration()
                       ? physics->GetGpuPhysics()
                       : nullptr;
  std::shared_ptr<BasicBarkDescriptor> bark;
  if (const auto descriptor = tree->tree_descriptor_ref.Get<TreeDescriptor>())
    bark = descriptor->bark_descriptor.Get<BasicBarkDescriptor>();
  const auto parameters = GpuTreeMeshParameters(tree->tree_mesh_generator_settings, bark.get());
  const glm::vec4 settings(context ? 0.0f : shoot.line_thickness, context ? 0.0f : root.line_thickness,
                           context || shoot.visualization ? 1.0f : 0.0f, context || root.visualization ? 1.0f : 0.0f);
  if (cached_tree_.lock() != tree || shoot_revision_ != tree->GetShootModelRevision().content ||
      root_revision_ != tree->GetRootModelRevision().content || shoot_checkpoint_ != shoot_checkpoint ||
      root_checkpoint_ != root_checkpoint || parameters_ != parameters || visualizer_settings_ != settings ||
      cached_physics_.lock() != gpu || cached_state_.lock() != (gpu ? physics->GetGpuPhysicsState() : nullptr) ||
      shoot.need_update || root.need_update) {
    nodes_.clear();
    colors_.clear();
    const auto append = [&](const auto& skeleton, auto& visualizer, bool is_shoot) {
      if (!context && !visualizer.visualization) {
        visualizer.need_update = false;
        return;
      }
      if (!context)
        visualizer.SyncColors(skeleton);
      const size_t base = nodes_.size();
      AppendGpuTreeNodes(skeleton, tree->tree_mesh_generator_settings, bark.get(), 0, is_shoot, nodes_);
      if (context)
        colors_.insert(colors_.end(), nodes_.size() - base, glm::vec4(1.0f, 1.0f, 1.0f, 0.18f));
      else
        colors_.insert(colors_.end(), visualizer.GetDebugColors().begin(), visualizer.GetDebugColors().end());
      for (size_t i = base; i < nodes_.size(); ++i) {
        auto& node = nodes_[i];
        node.links.z = 0;
        if (!context && visualizer.line_thickness > 0.0f) {
          node.position_thickness.w = visualizer.line_thickness;
          for (auto walker = static_cast<int>(node.foliage_meta.x); walker != -1;
               walker = skeleton.PeekNode(walker).GetParentHandle())
            if (walker == visualizer.selected_node_handle) {
              node.position_thickness.w *= 1.25f;
              break;
            }
        }
        if (is_shoot && gpu) {
          if (!physics->HasPhysicsRodForNode(static_cast<int>(node.foliage_meta.x)))
            node.links.z = UINT32_MAX;
          else
            while (node.links.x != UINT32_MAX &&
                   !physics->HasPhysicsRodForNode(static_cast<int>(nodes_[node.links.x].foliage_meta.x)))
              node.links.x = nodes_[node.links.x].links.x;
        }
      }
      visualizer.need_update = false;
    };
    append(tree->shoot_model.PeekShootSkeleton(shoot_checkpoint), shoot, true);
    shoot_count_ = static_cast<uint32_t>(nodes_.size());
    append(tree->root_model.PeekRootSkeleton(root_checkpoint), root, false);
    cached_tree_ = tree;
    cached_physics_ = gpu;
    cached_state_ = gpu ? physics->GetGpuPhysicsState() : nullptr;
    shoot_revision_ = tree->GetShootModelRevision().content;
    root_revision_ = tree->GetRootModelRevision().content;
    shoot_checkpoint_ = shoot_checkpoint;
    root_checkpoint_ = root_checkpoint;
    parameters_ = parameters;
    visualizer_settings_ = settings;
    ++generation_;
  }
  if (nodes_.empty()) {
    click_.reset();
    return;
  }
  auto& frame = frames_[Platform::GetCurrentFrameIndex()];
  if (frame.uploaded_generation != generation_) {
    BufferUploadBatch upload;
    upload.AddVector(frame.nodes, nodes_,
                     {BufferUploadUsage::StorageReadWrite, BufferUploadCapacityPolicy::GrowIfNeeded});
    upload.AddVector(frame.rest, nodes_, {BufferUploadUsage::StorageRead, BufferUploadCapacityPolicy::GrowIfNeeded});
    upload.AddVector(frame.colors, colors_, {BufferUploadUsage::StorageRead, BufferUploadCapacityPolicy::GrowIfNeeded});
    upload.Record(upload_arena_);
    frame.uploaded_generation = generation_;
  }
  const auto transform = scene->GetDataComponent<GlobalTransform>(tree->GetOwner());
  const GpuTreeDebugState state{transform.value,
                                projection_view,
                                {context ? UINT32_MAX : static_cast<uint32_t>(shoot.selected_node_handle),
                                 context ? UINT32_MAX : static_cast<uint32_t>(root.selected_node_handle), 0, 0},
                                {shoot.selected_node_length_factor, root.selected_node_length_factor, 0, 0},
                                glm::inverse(projection_view)};
  std::memcpy(frame.state->GetVmaAllocationInfo().pMappedData, &state, sizeof(state));
  frame.draw_set->UpdateBufferDescriptorBinding(0, frame.nodes);
  frame.draw_set->UpdateBufferDescriptorBinding(1, frame.nodes);
  frame.draw_set->UpdateBufferDescriptorBinding(2, frame.nodes);
  frame.draw_set->UpdateBufferDescriptorBinding(3, frame.state);
  frame.draw_set->UpdateBufferDescriptorBinding(4, frame.colors);
  if (gpu && shoot_count_) {
    frame.deform_set->UpdateBufferDescriptorBinding(0, frame.nodes);
    frame.deform_set->UpdateBufferDescriptorBinding(1, gpu->GetVertexBuffer(physics->GetGpuTreeIndex()));
    frame.deform_set->UpdateBufferDescriptorBinding(2, gpu->GetRodBuffer(physics->GetGpuTreeIndex()));
    frame.deform_set->UpdateBufferDescriptorBinding(3, gpu->GetNodeRodBuffer(physics->GetGpuTreeIndex()));
    frame.deform_set->UpdateBufferDescriptorBinding(4, frame.rest);
    const auto rotation = glm::inverse(transform.GetRotation());
    const GpuTreeDeformParameters deform{0,
                                         shoot_count_,
                                         0,
                                         gpu->GetTreeState(physics->GetGpuTreeIndex())->attribute_arrays ? 1u : 0u,
                                         glm::vec4(transform.GetPosition(), 0),
                                         glm::vec4(transform.GetScale(), 0),
                                         {rotation.x, rotation.y, rotation.z, rotation.w}};
    Platform::RecordCommandsMainQueue([&](VkCommandBuffer command) {
      const auto timing = Platform::BeginGpuTimestampScope(
          command, {"EcoSysLabDebugDeform", "EcoSysLab Debug Deform", "EcoSysLab", GpuTimestampQueue::Compute});
      deform_pipeline_->Bind(command);
      deform_pipeline_->BindDescriptorSet(command, 0, frame.deform_set->GetVkDescriptorSet());
      deform_pipeline_->PushConstant(command, 0, deform);
      deform_pipeline_->Dispatch(command, (shoot_count_ + 63) / 64, 1, 1);
      Platform::EverythingBarrier(command);
      Platform::EndGpuTimestampScope(command, timing);
    });
  }
  auto& pick_frame = context ? shared_renderer_->frames_[Platform::GetCurrentFrameIndex()] : frame;
  const auto size = camera->GetSize();
  const uint32_t width = static_cast<uint32_t>(size.x), height = static_cast<uint32_t>(size.y);
  if (!pick_frame.pick_image || pick_frame.pick_image->GetExtent().width != width ||
      pick_frame.pick_image->GetExtent().height != height) {
    VkImageCreateInfo image{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    image.imageType = VK_IMAGE_TYPE_2D;
    image.format = VK_FORMAT_R32G32B32A32_UINT;
    image.extent = {width, height, 1};
    image.mipLevels = image.arrayLayers = 1;
    image.samples = VK_SAMPLE_COUNT_1_BIT;
    image.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    pick_frame.pick_image = std::make_shared<Image>(image);
    VkImageViewCreateInfo view{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    view.image = pick_frame.pick_image->GetVkImage();
    view.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view.format = image.format;
    view.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    pick_frame.pick_view = std::make_shared<ImageView>(view, pick_frame.pick_image);
  }
  Platform::RecordCommandsMainQueue([&](VkCommandBuffer command) {
    const auto timing = Platform::BeginGpuTimestampScope(
        command, {context ? "EcoSysLabDebugContext" : "EcoSysLabDebugSelected",
                  context ? "EcoSysLab Debug Other Trees" : "EcoSysLab Debug Selected Tree", "EcoSysLab",
                  GpuTimestampQueue::Graphics});
    pick_frame.pick_image->TransitImageLayout(command, VK_IMAGE_LAYOUT_GENERAL);
    camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(command, VK_IMAGE_LAYOUT_GENERAL);
    camera->GetRenderTexture()->GetDepthImage()->TransitImageLayout(command, VK_IMAGE_LAYOUT_GENERAL);
    std::vector<VkRenderingAttachmentInfo> attachments;
    camera->GetRenderTexture()->AppendColorAttachmentInfos(attachments, VK_ATTACHMENT_LOAD_OP_LOAD,
                                                           VK_ATTACHMENT_STORE_OP_STORE);
    VkRenderingAttachmentInfo pick{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
    pick.imageView = pick_frame.pick_view->GetVkImageView();
    pick.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    pick.loadOp = context ? VK_ATTACHMENT_LOAD_OP_LOAD : VK_ATTACHMENT_LOAD_OP_CLEAR;
    pick.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    attachments.push_back(pick);
    const auto depth =
        camera->GetRenderTexture()->GetDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE);
    VkRenderingInfo rendering{VK_STRUCTURE_TYPE_RENDERING_INFO};
    rendering.renderArea.extent = {width, height};
    rendering.layerCount = 1;
    rendering.colorAttachmentCount = static_cast<uint32_t>(attachments.size());
    rendering.pColorAttachments = attachments.data();
    rendering.pDepthAttachment = &depth;
    Platform::BeginRendering(command, rendering);
    float diameter = 3.0f;
    if (scene->HasPrivateComponent<BranchRenderer>(tree->GetOwner())) {
      const auto branch = scene->GetOrSetPrivateComponent<BranchRenderer>(tree->GetOwner()).lock();
      diameter = branch->camera_impostors ? glm::clamp(branch->impostor_diameter_pixels, 1.0f, 8.0f) : 0;
    }
    if (const auto value = std::getenv("EVOENGINE_TREE_IMPOSTOR_PIXELS"))
      diameter = glm::clamp(static_cast<float>(std::atof(value)), 0.0f, 8.0f);
    if (const auto value = std::getenv("EVOENGINE_TREE_DEBUG_IMPOSTOR_PIXELS"))
      diameter = glm::clamp(static_cast<float>(std::atof(value)), 0.0f, 8.0f);
    for (uint32_t proxy = 0; proxy < (diameter > 0 ? 2u : 1u); ++proxy) {
      const auto& draw_pipeline = proxy ? impostor_pipeline_ : pipeline_;
      draw_pipeline->Bind(command);
      draw_pipeline->BindDescriptorSet(command, 3, frame.draw_set->GetVkDescriptorSet());
      draw_pipeline->states.ResetAllStates(2);
      draw_pipeline->states.SetViewportScissor({0, 0, width, height});
      draw_pipeline->states.cull_mode = VK_CULL_MODE_NONE;
      draw_pipeline->states.depth_test = true;
      draw_pipeline->states.depth_write = !context;
      if (context) {
        auto& blend = draw_pipeline->states.color_blend_attachment_states[0];
        blend.blendEnable = VK_TRUE;
        blend.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        blend.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        blend.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        blend.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        draw_pipeline->states.color_blend_attachment_states[1].colorWriteMask = 0;
      }
      draw_pipeline->states.depth_compare = VK_COMPARE_OP_LESS_OR_EQUAL;
      draw_pipeline->states.ApplyAllStates(command);
      const uint32_t limit = Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxTaskWorkGroupCount[0];
      const uint32_t batch = proxy ? 32u : 1u;
      for (uint32_t base = 0; base < nodes_.size(); base += limit * batch) {
        GpuTreeDrawParameters push;
        push.node_base = base;
        push.padding0 = batch;
        push.impostor = {diameter, proxy ? 1.0f : 0.0f, 0, 0};
        push.tessellation = {static_cast<float>(width), static_cast<float>(height), 0, 0};
        push.node_count = static_cast<uint32_t>(nodes_.size());
        if (context && !std::getenv("EVOENGINE_TREE_FULL_TESSELLATION")) {
          float error = 0.5f, edge = 12.0f;
          if (tree->GetScene()->HasPrivateComponent<BranchRenderer>(tree->GetOwner())) {
            const auto branch = tree->GetScene()->GetOrSetPrivateComponent<BranchRenderer>(tree->GetOwner()).lock();
            error = branch->screen_space_tessellation ? glm::clamp(branch->pixel_error, 0.1f, 2.0f) : 0;
            edge = glm::clamp(branch->segment_length_pixels, 2.0f, 64.0f);
          }
          push.tessellation = {static_cast<float>(width), static_cast<float>(height), error, edge};
        }
        draw_pipeline->PushConstant(command, 0, push);
        draw_pipeline->DrawMeshTasks(
            command, std::min(limit, (static_cast<uint32_t>(nodes_.size()) - base + batch - 1) / batch), 1, 1);
      }
    }
    Platform::EndRendering(command);
    if (!context && click_ && click_->x >= 0 && click_->y >= 0 && click_->x < width && click_->y < height) {
      pick_frame.pick_image->TransitImageLayout(command, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
      VkBufferImageCopy copy{};
      copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
      copy.imageExtent = {1, 1, 1};
      copy.imageOffset = {static_cast<int32_t>(click_->x),
                          static_cast<int32_t>(height - 1) - static_cast<int32_t>(click_->y), 0};
      Platform::CopyImageToBuffer(command, *pick_frame.pick_image, *frame.readback, copy);
      Platform::EverythingBarrier(command);
      frame.submission = Platform::TrackCurrentFrameSubmission();
      frame.pick_generation = generation_;
      frame.pick_serial = request_serial_;
    }
    Platform::EndGpuTimestampScope(command, timing);
  });
  click_.reset();
}
