#pragma once
#include "DsMeshing.hpp"
#include "kinDS/kinDS/TreeMesher.hpp"
#include "kinDS/kinDS/VoronoiMesh.hpp"
#include <filesystem>

namespace kinDS {
class StrandTree;
}
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
  void Stats(const std::shared_ptr<EditorLayer>& editor_layer) override;
  static void OnInspectRenderSettings(const std::shared_ptr<EditorLayer>& editor_layer);
  void RegisterRenderInstances(Handle& rendering_instance_handle, std::shared_ptr<Scene> scene, Entity& owner) override;
  void Visualize(const std::shared_ptr<Camera>& target_camera,
                 const DynamicStrandsInitializeParameters& initialize_parameters,
                 const DynamicStrandsVisualizationParameters& visualization_parameters) override;

  // everything specific to kinetic voronoi meshing
  struct SegmentMeshletsRenderParameters {
    // TODO
    bool enabled = true;
    enum ColorMode { Standard, Normals, UVs };
    int color_mode = 0;
    float uv_height_factor = 0.02f;
    float uv_circum_factor = 2.0f;
    float fracture_distance = 0.0004f;
    double alpha_cutoff = 10.0;
  };

  struct RenderSettings {
    // TODO: Add render settings specific to kinetic voronoi meshing
    SegmentMeshletsRenderParameters segment_meshlet_render_parameters;
  };

  struct MeshingSettings {
    bool dry_run_strand_tree_only = false;
    bool debug_svg = false;
    /// When true, store JSON vertex/face metadata on meshlets (@ref TreeMesher::Settings::store_mesh_metadata).
    bool store_mesh_metadata = false;
    /// Blend for meshing-only plane-spline sampling. 0 = Strands cubic (away from knots), 1 = Catmull-Rom (through knots).
    float spline_tension = 0.5f;
    /// When true, apply inverse root transform to a loaded intersection boundary OBJ before meshlet clipping.
    bool intersection_boundary_apply_inverse_root_transform = true;
    /// When true, attempt to repair empty meshlets after boundary intersection (@ref TreeMesher::fixFailedSegments).
    bool intersection_boundary_fix_missing_meshes = false;
  };

  static RenderSettings render_settings;
  static MeshingSettings meshing_settings;

  struct GpuSegmentMeshletVertex {
    glm::vec3 x0;
    unsigned int segment_index;
    glm::vec3 x = glm::vec3(1.0f, 2.0f, 3.0f);
    int padding0;
    glm::vec3 shift = glm::vec3(0.0f, 0.0f, 0.0f);
    int padding1;
  };

  struct GpuSegmentMeshletTriangle {
    unsigned int vertex_index0;
    unsigned int vertex_index1;
    unsigned int vertex_index2;
    int neighbor_segment_index;
    // TODO: perhaps split these off into separate buffers with indices
    glm::vec4 normal[3];   // 4th dimension is padding
    glm::vec4 normal0[3];  // 4th dimension is padding
    glm::vec4 uv[3];       // 4th dimension is padding
    int segment_pair_index;
    int padding0;
    int padding1;
    int padding2;
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
    int color_mode;

    int bark_material_index;
    int inner_wood_material_index;
    float uv_height_factor;
    float uv_circum_factor;
    float fracture_distance;
  };

  // public:
  inline static std::shared_ptr<ComputePipeline> branches_vertex_update_pipeline{};
  inline static std::shared_ptr<ComputePipeline> branches_triangle_update_pipeline{};
  inline static std::shared_ptr<GraphicsPipeline> segment_meshlet_point_light_render_pipeline{};
  inline static std::shared_ptr<GraphicsPipeline> segment_meshlet_directional_light_render_pipeline{};
  inline static std::shared_ptr<GraphicsPipeline> segment_meshlet_spot_light_render_pipeline{};
  inline static std::shared_ptr<GraphicsPipeline> segment_meshlet_render_pipeline{};
  inline static std::shared_ptr<GraphicsPipeline> segment_meshlet_visualization_render_pipeline{};

  std::vector<GpuSegmentMeshletVertex> segment_meshlet_vertices;
  std::vector<GpuSegmentMeshletTriangle> segment_meshlet_triangles;

  std::shared_ptr<Buffer> device_segment_meshlet_vertices_buffer;
  std::shared_ptr<Buffer> device_segment_meshlet_triangles_buffer;

  // kinDS::VoronoiMesh transformed_boundary_mesh;
  kinDS::VoronoiMesh intersection_boundary_mesh_;
  std::filesystem::path intersection_boundary_mesh_path_;
  std::vector<float> boundary_distances_by_vertex;
  std::shared_ptr<kinDS::StrandTree> strand_tree;

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
      const Handle& renderer_handle, int bark_material_index, int inner_wood_material_index, int snow_material_index,
      const SegmentMeshletsRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
      const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
      const RenderLayer::DeferredRenderingView& view, VkPolygonMode polygon_mode) const;
  /* uint32_t RenderSegmentMeshletVisualizationToCameraDeferred(
      const Handle& renderer_handle, const DynamicStrandsInitializeParameters& initialize_parameters,
      const SmallSegmentsVisualizationRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
      const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
      const RenderLayer::DeferredRenderingView& view) const;*/

  // kinDS::VoronoiMesh TransformBoundaryMesh(
  //     const kinDS::VoronoiMesh& boundary_mesh,
  //     const std::vector<std::vector<glm::dmat4>>& transforms_by_height_and_branch,
  //     const std::vector<std::vector<glm::dmat4>>& normal_transforms_by_height_and_branch,
  //     const GlobalTransform& root_transform, const std::vector<std::vector<size_t>>& branch_indices,
  //     const std::vector<size_t>& boundary_vertex_to_strand_id);

  void RecomputeSegmentPairs(const kinDS::TreeMesher& tree_mesher);
  void PopulateGpuMeshletBuffers(const std::vector<kinDS::VoronoiMesh>& meshes,
                                 const std::vector<std::vector<int>>& physics_strand_to_segment_indices,
                                 const std::vector<std::vector<size_t>>& meshing_strand_to_segment_indices,
                                 const std::vector<std::vector<int>>& meshing_neighbor_indices,
                                 const std::vector<size_t>& meshing_to_physics_segment_indices,
                                 const GlobalTransform& root_transform);

  void RunMeshingAlgorithm(const std::vector<std::vector<glm::dvec2>>& support_points,
                           std::vector<std::vector<double>>& subdivisions_by_strand,
                           std::vector<std::vector<int>>& physics_strand_to_segment_indices,
                           const std::vector<std::vector<glm::dmat4>>& transforms_by_height_and_branch,
                           const GlobalTransform& root_transform,
                           const std::vector<std::vector<size_t>>& branch_indices,
                           std::vector<std::vector<std::vector<size_t>>>& strands_by_branch_id);
};
}  // namespace eco_sys_lab_plugin