#pragma once
#include "CubicHermiteSpline.hpp"
#include "DsMeshing.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;
class DsKineticVoronoiMeshing : public DsMeshing {
 public:
  DsKineticVoronoiMeshing();
  ~DsKineticVoronoiMeshing();

  void InitBuffer(VkBufferCreateInfo& buffer_create_info,
                  VmaAllocationCreateInfo& buffer_vma_allocation_create_info) override;
  void InitData(const DynamicStrandsInitializeParameters& initialize_parameters,
                const StrandModelSkeleton& strand_model_skeleton,
                const StrandModelStrandGroup& strand_model_strand_group,
                DtsStrandGroup& randomly_subdivided_strand_group,
                DtsStrandGroup& uniformly_subdivided_strand_group) override;
  void InitializationGraphicsPipeline(const DynamicStrandsInitializeParameters& initialize_parameters) override;

  void BuildRenderComputePipelines() override;
  void RenderCompute() const override;
  void BuildRenderingPipelines() override;

  void Download() override;
  void Upload() override;
  void Clear() override;

  void UpdateBindings() const override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  static void OnInspectRenderSettings(const std::shared_ptr<EditorLayer>& editor_layer);
  void RegisterRenderInstances(Handle& rendering_instance_handle, std::shared_ptr<Scene> scene, Entity& owner) override;
  void Visualize(const std::shared_ptr<Camera>& target_camera,
                 const DynamicStrandsInitializeParameters& initialize_parameters,
                 const DynamicStrandsVisualizationParameters& visualization_parameters);

  // everything specific to kinetic voronoi meshing
  struct SegmentMeshletsRenderParameters {
    // TODO
    bool enabled = true;
  };

  struct RenderSettings {
    // TODO: Add render settings specific to kinetic voronoi meshing
    bool render_segment_meshlets = true;
    SegmentMeshletsRenderParameters segment_meshlet_render_parameters;
  };

  static RenderSettings render_settings;

  struct GpuSegmentMeshletVertex {
    float relative_position_x;
    float relative_position_y;
    float relative_position_z;
    unsigned int segment_index;
  };

  struct GpuSegmentMeshletTriangle {
    unsigned int vertex_index0;
    unsigned int vertex_index1;
    unsigned int vertex_index2;
    int twin_triangle_index;
  };

  struct SegmentMeshletPushConstant {
    union Index1 {
      int instance_index;
      int sub_light_index;
    } index1;
    union Index2 {
      int camera_index;
      int light_index;
    } index2;

    unsigned int vertex_count;
    unsigned int triangle_count;
  };

  // public:
  inline static std::shared_ptr<GraphicsPipeline> segment_meshlet_point_light_render_pipeline{};
  inline static std::shared_ptr<GraphicsPipeline> segment_meshlet_directional_light_render_pipeline{};
  inline static std::shared_ptr<GraphicsPipeline> segment_meshlet_spot_light_render_pipeline{};
  inline static std::shared_ptr<GraphicsPipeline> segment_meshlet_render_pipeline{};
  inline static std::shared_ptr<GraphicsPipeline> segment_meshlet_visualization_render_pipeline{};

  std::vector<GpuSegmentMeshletVertex> segment_meshlet_vertices;
  std::vector<GpuSegmentMeshletTriangle> segment_meshlet_triangles;

  std::shared_ptr<Buffer> device_segment_meshlet_vertices_buffer;
  std::shared_ptr<Buffer> device_segment_meshlet_triangles_buffer;

  // registration
  void RegisterSegmentMeshletsRenderInstance(Handle& rendering_instance_handle, std::shared_ptr<Scene> scene,
                                             Entity& owner);

  // build render pipelines
  static void BuildSegmentMeshletsRenderingPipelines();

  // rendering
  uint32_t RenderSegmentMeshletsToPointLightShadowMap(const SegmentMeshletsRenderParameters& render_parameters,
                                                      const VkCommandBuffer vk_command_buffer,
                                                      const RenderLayer::PointLightShadowMapView& view) const;
  uint32_t RenderSegmentMeshletsToSpotLightShadowMap(const SegmentMeshletsRenderParameters& render_parameters,
                                                     VkCommandBuffer vk_command_buffer,
                                                     const RenderLayer::SpotLightShadowMapView& view) const;
  uint32_t RenderSegmentMeshletsToDirectionalLightShadowMap(
      const SegmentMeshletsRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
      const RenderLayer::DirectionalLightShadowMapView& view) const;
  uint32_t RenderSegmentMeshletsToCameraDeferred(
      const Handle& renderer_handle, int inner_wood_material_index, int snow_material_index,
      const SegmentMeshletsRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
      const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
      const RenderLayer::DeferredRenderingView& view, VkPolygonMode polygon_mode) const;
  /* uint32_t RenderSegmentMeshletVisualizationToCameraDeferred(
      const Handle& renderer_handle, const DynamicStrandsInitializeParameters& initialize_parameters,
      const SmallSegmentsVisualizationRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
      const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
      const RenderLayer::DeferredRenderingView& view) const;*/

  void RunMeshingAlgorithm(std::vector<kinDS::CubicHermiteSpline<2>> strand_splines);
};
}  // namespace eco_sys_lab_plugin
