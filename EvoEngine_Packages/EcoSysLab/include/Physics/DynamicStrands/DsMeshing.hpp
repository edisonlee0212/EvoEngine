#pragma once
#include "DsMaterials.hpp"
#include "DtsStrandGroup.hpp"
#include "DynamicStrandsVisualizationParameters.hpp"
#include "ShootGrowthData.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;
class DynamicStrands;
struct DynamicStrandsInitializeParameters;
/**
 * Base class for meshing of dynamic strands. Any type of meshing should implement the interface specified here.
 */
class DsMeshing {
 public:
  DsMeshing();
  ~DsMeshing();

  virtual void InitBuffer(VkBufferCreateInfo& buffer_create_info,
                          VmaAllocationCreateInfo& buffer_vma_allocation_create_info) = 0;
  virtual void InitData(const DynamicStrandsInitializeParameters& initialize_parameters,
                        const StrandModelSkeleton& strand_model_skeleton,
                        const StrandModelStrandGroup& strand_model_strand_group,
                        DtsStrandGroup& randomly_subdivided_strand_group,
                        DtsStrandGroup& uniformly_subdivided_strand_group) = 0;
  virtual void InitializationGraphicsPipeline(const DynamicStrandsInitializeParameters& initialize_parameters) = 0;

  virtual void BuildRenderComputePipelines() = 0;
  virtual void RenderCompute() const = 0;
  virtual void BuildRenderingPipelines() = 0;

  virtual void Download() = 0;
  virtual void Upload() = 0;
  virtual void Clear() = 0;

  virtual void UpdateBindings() const = 0;

  virtual bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) = 0;

  virtual void Stats(const std::shared_ptr<EditorLayer>& editor_layer);

  virtual void RegisterRenderInstances(Handle& rendering_instance_handle, std::shared_ptr<Scene> scene,
                                       Entity& owner) = 0;

  virtual void Visualize(const std::shared_ptr<Camera>& target_camera,
                         const DynamicStrandsInitializeParameters& initialize_parameters,
                         const DynamicStrandsVisualizationParameters& visualization_parameters) = 0;

  DynamicStrands* dynamic_strands;  // Raw pointer is fine here since DynamicStrands owns DsMeshing
};
}  // namespace eco_sys_lab_package