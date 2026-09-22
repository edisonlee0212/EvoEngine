#include "DsMeshing.hpp"
#include "Application.hpp"
#include "GraphicsPipeline.hpp"
#include "Platform/Platform.hpp"
#include "RenderLayer.hpp"

using namespace eco_sys_lab_package;

DsMeshing::DsMeshing() {
}

DsMeshing::~DsMeshing() {
}

void DsMeshing::InitGeometryDescriptors() {
  if (!geometry_descriptor_set_layout) {
    geometry_descriptor_set_layout = std::make_shared<DescriptorSetLayout>();
    geometry_descriptor_set_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    geometry_descriptor_set_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    geometry_descriptor_set_layout->Initialize();
  }
  geometry_descriptor_sets.resize(Platform::GetMaxFramesInFlight());
  for (auto& descriptor_set : geometry_descriptor_sets) {
    descriptor_set = std::make_shared<DescriptorSet>(geometry_descriptor_set_layout);
  }
}

void DsMeshing::CompleteGraphicsDescriptorLayouts(const std::shared_ptr<GraphicsPipeline>& pipeline) {
  // Keep set 2 for lighting in deferred pipelines; other pipelines leave it empty.
  if (pipeline->descriptor_set_layouts.size() == 2) {
    pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetEmptyDescriptorSetLayout());
  }
  pipeline->descriptor_set_layouts.emplace_back(geometry_descriptor_set_layout);
}
