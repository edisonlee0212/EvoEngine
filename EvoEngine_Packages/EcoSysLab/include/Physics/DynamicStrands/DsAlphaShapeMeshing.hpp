#pragma once

#include "DsMeshing.hpp"
#include "RenderLayer.hpp"
#include "RenderParameters.hpp"

#ifdef USE_CGAL
#  include <CGAL/Delaunay_triangulation_3.h>
#  include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#  include <CGAL/Triangulation_vertex_base_with_info_3.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_with_info_3<unsigned int, K> Vb;
typedef CGAL::Triangulation_data_structure_3<Vb> Tds;
typedef CGAL::Delaunay_triangulation_3<K, Tds> Delaunay_CGAL;
typedef K::Point_3 Point_CGAL;
#endif

namespace eco_sys_lab_package {
using namespace evo_engine;

struct DsAlphaShapeVisualizationParameters {
  enum class UniformParticleRenderMode { Default, SegmentColor, SingleParticles };

  bool render_uniform_particles = false;
  uint32_t uniform_particle_render_mode = 2;
  glm::vec4 uniform_particle_main = glm::vec4(1, 1, 1, 0.8f);
  float uniform_particle_radius_multiplier = 2.f;

  // TODO: This isn't called anywhere
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
};

class DsAlphaShapeMeshing : public DsMeshing {
 public:
  DsAlphaShapeMeshing();
  ~DsAlphaShapeMeshing();

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

  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Stats(const std::shared_ptr<EditorLayer>& editor_layer) override;
  static void DrawRenderSettingsGui(const std::shared_ptr<EditorLayer>& editor_layer);

  void RegisterRenderInstances(Handle& rendering_instance_handle, std::shared_ptr<Scene> scene, Entity& owner) override;

  void Visualize(const std::shared_ptr<Camera>& target_camera,
                 const DynamicStrandsInitializeParameters& initialize_parameters,
                 const DynamicStrandsVisualizationParameters& visualization_parameters) override;

  // everything specific to alpha shape meshing
  struct RenderSettings {
    BranchesRenderParameters branches_render_parameters{};             ///< Rendering parameters for branches.
    SmallSegmentsRenderParameters small_segments_render_parameters{};  ///< Rendering parameters for small segments.
    SmallSegmentsVisualizationRenderParameters
        small_segments_visualization_render_parameters{};  ///< Visualization settings for small segments.
    bool visualization_rendering = false;
    DsAlphaShapeVisualizationParameters meshing_visualization_parameters;
  };

  static RenderSettings render_settings;

  struct GpuUniformParticle {
    glm::vec3 position;
    float t;
    glm::vec3 normal;
    float deg;
    glm::vec3 tangent;
    int padding0;

    glm::vec2 profile_position;
    glm::vec2 profile_polar_coordinate;

    glm::vec4 override_color = glm::vec4(0.f);

    int segment_handle;
    int node_index;
    int segment_index;
    float distance_to_boundary;
    int next_particle_handle;
    int prev_particle_handle;
    int next_node_index;
    int strand_index;

    int is_single_strand_particle;
    float local_extrusion_distance;
    int is_on_surface;
    int is_bark;
    glm::vec3 initial_position;
    int padding4;
    glm::vec4 normal_q;
  };

  struct GpuDelaunayTetrahedron {
    int indices[4];
    int neighbor_tet_ids[4];
    int render_neighbor[4];
    int is_bark[4];
    glm::vec4 color;  // for debugging
    unsigned int task_looked_at = 0;
    unsigned int mesh_looked_at = 0;
    int inside = -1;
    int triangles_accepted = 0;
    float sidelengths[6];
    int padding0;
    int padding1;
    int segment_pair_index[6];
    int inside_at_init;
    int padding2;
  };

  // specific functions and members for alpha shape meshing
 public:
  inline static std::shared_ptr<ComputePipeline> branches_uniform_particle_update_pipeline;
  inline static std::shared_ptr<ComputePipeline> branches_tetrahedron_filtering_pipeline{};
  inline static std::shared_ptr<ComputePipeline> branches_triangle_filtering_pipeline{};

  inline static std::shared_ptr<GraphicsPipeline> branches_point_light_render_pipeline{};
  inline static std::shared_ptr<GraphicsPipeline> branches_spot_light_render_pipeline{};
  inline static std::shared_ptr<GraphicsPipeline> branches_directional_light_render_pipeline{};
  inline static std::shared_ptr<GraphicsPipeline> branches_render_pipeline{};
  inline static std::shared_ptr<GraphicsPipeline> branches_masked_render_pipeline{};

  inline static std::shared_ptr<GraphicsPipeline> small_segments_point_light_render_pipeline{};
  inline static std::shared_ptr<GraphicsPipeline> small_segments_spot_light_render_pipeline{};
  inline static std::shared_ptr<GraphicsPipeline> small_segments_directional_light_render_pipeline{};
  inline static std::shared_ptr<GraphicsPipeline> small_segments_render_pipeline{};
  inline static std::shared_ptr<GraphicsPipeline> small_segments_masked_render_pipeline{};
  inline static std::shared_ptr<GraphicsPipeline> small_segments_visualization_render_pipeline{};
  inline static std::shared_ptr<GraphicsPipeline> small_segments_visualization_masked_render_pipeline{};

  std::vector<GpuUniformParticle> uniform_particles;
  std::vector<GpuDelaunayTetrahedron> delaunay_tetrahedrons;

  std::shared_ptr<Buffer> device_uniform_particles_buffer;
  std::shared_ptr<Buffer> device_delaunay_tetrahedrons_buffer;

  Handle mesh_wireframe_rendering_instance_handle;  ///< Handle for mesh wireframe rendering instance.
  Handle small_segments_rendering_instance_handle;  ///< Handle for small segment rendering instance.

 private:
#ifdef USE_CGAL
  void CGALDelaunay(const std::vector<std::pair<Point_CGAL, unsigned>>& points,
                    std::vector<GpuDelaunayTetrahedron>& tetrahedrons);
#endif
  void TetDelaunay(const std::vector<glm::vec3>& points, const std::vector<size_t>& particle_indices,
                   std::vector<GpuDelaunayTetrahedron>& tetrahedrons);
  void ComputeDelaunayPerBundle(std::vector<GpuDelaunayTetrahedron>& tetrahedrons, bool use_cgal = false);
  void ComputeDelaunay(std::vector<GpuDelaunayTetrahedron>& tetrahedrons, bool use_cgal = false,
                       size_t min_bundle_size = 3);

  // registration

  /**
   * @brief Registers rendering instance for branch visualization.
   * @param render_parameters Parameters for branch rendering.
   */
  void RegisterBranchesRenderInstance(Handle& rendering_instance_handle, std::shared_ptr<Scene> scene, Entity& owner);

  /**
   * @brief Registers rendering instance for branch wireframe visualization.
   * @param render_parameters Parameters for branch wireframe rendering.
   */
  void RegisterBranchesWireframeRenderInstance(Handle& rendering_instance_handle, std::shared_ptr<Scene> scene,
                                               Entity& owner);

  /**
   * @brief Registers rendering instance for small segment visualization.
   * @param render_parameters Parameters for small segment rendering.
   */
  void RegisterSmallSegmentsRenderInstance(Handle& rendering_instance_handle, std::shared_ptr<Scene> scene,
                                           Entity& owner);

  /**
   * @brief Registers visualization render instance for small segments.
   * @param render_parameters Parameters for small segment rendering.
   * @param visualization_render_parameters Parameters for small segment visualization.
   */
  void RegisterSmallSegmentsVisualizationRenderInstance(Handle& rendering_instance_handle, std::shared_ptr<Scene> scene,
                                                        Entity& owner);

  // build render pipelines
  static void BuildBranchesRenderingPipelines();
  static void BuildSmallSegmentsRenderingPipelines();

  // rendering
  uint32_t RenderBranchesToPointLightShadowMap(const BranchesRenderParameters& render_parameters,
                                               const VkCommandBuffer vk_command_buffer,
                                               const RenderLayer::PointLightShadowMapView& view) const;
  uint32_t RenderBranchesToSpotLightShadowMap(const BranchesRenderParameters& render_parameters,
                                              VkCommandBuffer vk_command_buffer,
                                              const RenderLayer::SpotLightShadowMapView& view) const;
  uint32_t RenderBranchesToDirectionalLightShadowMap(const BranchesRenderParameters& render_parameters,
                                                     VkCommandBuffer vk_command_buffer,
                                                     const RenderLayer::DirectionalLightShadowMapView& view) const;
  uint32_t RenderBranchesToCameraDeferred(
      const Handle& renderer_handle, int bark_material_index, int inner_wood_material_index, int snow_material_index,
      int render_material_index, const std::shared_ptr<GraphicsPipeline>& pipeline, VkCullModeFlags cull_mode,
      const BranchesRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
      const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
      const RenderLayer::DeferredRenderingView& view, VkPolygonMode polygon_mode) const;

  uint32_t RenderSmallSegmentsToPointLightShadowMap(const SmallSegmentsRenderParameters& render_parameters,
                                                    const VkCommandBuffer vk_command_buffer,
                                                    const RenderLayer::PointLightShadowMapView& view) const;
  uint32_t RenderSmallSegmentsToSpotLightShadowMap(const SmallSegmentsRenderParameters& render_parameters,
                                                   VkCommandBuffer vk_command_buffer,
                                                   const RenderLayer::SpotLightShadowMapView& view) const;
  uint32_t RenderSmallSegmentsToDirectionalLightShadowMap(const SmallSegmentsRenderParameters& render_parameters,
                                                          VkCommandBuffer vk_command_buffer,
                                                          const RenderLayer::DirectionalLightShadowMapView& view) const;
  uint32_t RenderSmallSegmentsToCameraDeferred(
      const Handle& renderer_handle, int splinter_material_index, int render_material_index,
      const std::shared_ptr<GraphicsPipeline>& pipeline, VkCullModeFlags cull_mode,
      const SmallSegmentsRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
      const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
      const RenderLayer::DeferredRenderingView& view) const;

  uint32_t RenderSmallSegmentsVisualizationToCameraDeferred(
      const Handle& renderer_handle, const DynamicStrandsInitializeParameters& initialize_parameters,
      const std::shared_ptr<GraphicsPipeline>& pipeline, VkCullModeFlags cull_mode,
      const SmallSegmentsVisualizationRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
      const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
      const RenderLayer::DeferredRenderingView& view) const;
};
}  // namespace eco_sys_lab_package
