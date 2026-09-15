#include "EditorLayer.hpp"
#include "GraphicsPipeline.hpp"
#include "RenderLayer.hpp"
#include "RenderPasses/EntitySelectionHighlightPass.hpp"
#include "Resources.hpp"
#include "Shader.hpp"

using namespace evo_engine;
namespace {
constexpr const char* kSelectionPassOwner = "EvoEngine.Editor.Selection";
}

void EditorLayer::InitializeSelectionRendering() {
  const auto render = GetApplication().GetLayer<RenderLayer>();
  if (!render)
    return;
  if (!entity_selection_highlight_pipeline_) {
    entity_selection_highlight_pipeline_ = std::make_shared<GraphicsPipeline>();
    entity_selection_highlight_pipeline_->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/TexturePassThrough.slang");
    entity_selection_highlight_pipeline_->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Resources::GetDefaultResourcesPath() /
                                  "Shaders/Graphics/Fragment/PostProcessing/EntitySelectionHighlight.slang");
    entity_selection_highlight_pipeline_->geometry_type = GeometryType::Mesh;
    entity_selection_highlight_pipeline_->vertex_input_attribute_set = VertexInputAttributeSet::PositionTexCoord;
    entity_selection_highlight_pipeline_->descriptor_set_layouts.emplace_back(render->GetEmptyDescriptorSetLayout());
    entity_selection_highlight_pipeline_->descriptor_set_layouts.emplace_back(
        render->GetCameraGBufferDescriptorSetLayout());
    entity_selection_highlight_pipeline_->depth_attachment_format = VK_FORMAT_UNDEFINED;
    entity_selection_highlight_pipeline_->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    entity_selection_highlight_pipeline_->color_attachment_formats = {Platform::Constants::render_texture_color};
    auto& push_constant_range = entity_selection_highlight_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(EntitySelectionHighlightPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    entity_selection_highlight_pipeline_->Initialize();
  }

  RenderInstanceCallbacks callbacks;
  callbacks.highlight = [this](const std::shared_ptr<Scene>& scene) {
    const auto started = std::chrono::steady_clock::now();
    const auto selection = GetEntitySelectionSnapshot();
    const auto hierarchy_revision = scene->GetHierarchyRevision();
    if (selection_highlight_coverage_scene_.lock() != scene ||
        selection_highlight_coverage_selection_revision_ != selection.revision ||
        selection_highlight_coverage_hierarchy_revision_ != hierarchy_revision) {
      const ProfilerScope selection_scope("EditorLayer::BuildSelectionHighlightCoverage", "Render");
      auto coverage = std::make_shared<EntitySelectionHighlightCoverage>();
      for (const auto& selected : selection.entities) {
        if (!scene->IsEntityValid(selected))
          continue;
        coverage->emplace(selected);
        for (const auto& descendant : scene->GetDescendants(selected))
          coverage->emplace(descendant);
      }
      selection_highlight_coverage_scene_ = scene;
      selection_highlight_coverage_selection_revision_ = selection.revision;
      selection_highlight_coverage_hierarchy_revision_ = hierarchy_revision;
      selection_highlight_coverage_ = std::move(coverage);
      if (Platform::Initialized())
        Platform::RecordCpuTimingSample(
            "Editor Selection / Expand Coverage",
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count());
    }
    return RenderInstanceHighlightInput{selection_highlight_coverage_, selection.revision};
  };
  callbacks.prepared = [this] {
    ProcessPendingViewportSelection();
  };
  render->SetRenderInstanceCallbacks(std::move(callbacks));
}

void EditorLayer::RegisterSelectionPass() {
  const auto render = GetApplication().GetLayer<RenderLayer>();
  if (!render)
    return;
  render->RemoveCameraRenderPasses(kSelectionPassOwner);
  const auto camera = GetSceneCamera();
  if (!GetScene() || !camera)
    return;
  render->RegisterCameraRenderPass(
      EntitySelectionHighlightPass::CreateDescriptor(),
      [this](VkCommandBuffer command, const std::shared_ptr<Camera>& target, const RenderLayer::ForwardRenderingView&,
             const RenderGraphExecutionContext& context) -> uint32_t {
        auto presentation = GetEntitySelectionHighlightSnapshot();
        const auto instances = GetApplication().GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage();
        presentation.active = presentation.active && instances && instances->HasSelectionHighlightRenderInstances();
        EntitySelectionHighlightPass::Record(command, context,
                                             {target, entity_selection_highlight_pipeline_, presentation});
        return 0;
      },
      {CameraRenderPassStage::AfterPostProcessing, camera, kSelectionPassOwner});
}

void EditorLayer::ClearSelectionRendering() {
  if (const auto render = GetApplication().GetLayer<RenderLayer>()) {
    render->SetRenderInstanceCallbacks({});
    render->RemoveCameraRenderPasses(kSelectionPassOwner);
  }
  entity_selection_highlight_pipeline_.reset();
  selection_highlight_coverage_scene_.reset();
  selection_highlight_coverage_.reset();
}
