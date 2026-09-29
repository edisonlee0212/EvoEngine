#pragma once
#include <filesystem>
#include <unordered_map>
#include "DsMeshing.hpp"
#include "DynamicStrandsInitializationParameters.hpp"
#include "Entity.hpp"
#include "Transform.hpp"
#include "kinDS/kinDS/TreeMesher.hpp"
#include "kinDS/kinDS/VoronoiMesh.hpp"
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
  /// Shared Kinetic Voronoi meshing/render scalars (usable before a meshing instance exists).
  static void InspectSharedMeshingSettings(const std::shared_ptr<EditorLayer>& editor_layer);
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
    enum ColorMode { Standard = 0, Normals = 1, UVs = 2, Pair = 3, NeighborConnectivity = 4, NeighborTags = 5 };
    int color_mode = 0;
    float uv_height_factor = 0.1f;
    float uv_circum_factor = 2.0f;
    float fracture_distance = 0.0004f;
    /// When true, override color mode with neighbor-connectivity debug colors (grey / brown / red / green).
    bool debug_neighbor_connectivity = false;
  };

  struct RenderSettings {
    // TODO: Add render settings specific to kinetic voronoi meshing
    SegmentMeshletsRenderParameters segment_meshlet_render_parameters;
  };

  struct MeshingSettings {
    /// When true, rebuild physics segment pairs from meshlet adjacency after meshing.
    /// @c pair_handles[0]/[1] are reserved for same-strand below/above neighbors.
    bool recompute_segment_pairs = false;
    /// When true, skip loading a cached meshing buffer and overwrite it after computing.
    bool override_meshing_buffer = false;
    bool dry_run_strand_tree_only = false;
    bool debug_svg = false;
    /// When true, kinDS collects runtime/event statistics and writes CSV after meshing
    /// (plus a companion event-list CSV with per-event rows, including radius shift).
    bool collect_meshing_statistics = true;
    /// When true, export meshlets/combined OBJ after meshing for debugging.
    bool debug_export_meshes = false;
    /// When true, debug/failed OBJ dumps emit one object per interior/boundary contributor.
    bool export_separate_contributor_objects = true;
    /// When true, store JSON vertex/face metadata on meshlets (@ref TreeMesher::Settings::store_mesh_metadata).
    bool store_mesh_metadata = false;
    /// Blend for meshing-only plane-spline sampling. 0 = Strands cubic (away from knots), 1 = Catmull-Rom (through
    /// knots).
    float spline_tension = 1.0f;
    /// Laplacian smoothing iterations on welded bark triangles before GPU upload (0 = disabled).
    /// Reuses @ref StrandModelMeshGenerator::MeshSmoothing (no ground-plane lock).
    int bark_smooth_iterations = 1;
    /// Per-iteration blend toward the neighbor average: 0 = no move, 1 = full Laplacian step.
    float bark_smooth_strength = 0.5f;
    /// When true, vertices on the bark manifold boundary (edges with only one bark triangle) keep
    /// their position during Laplacian smooth.
    bool bark_smooth_lock_boundary = true;
    /// When true, co-smooth bark corner UVs with the same iterations/strength as positions (wrap-aware
    /// circumferential lift so seam discontinuities do not poison the average).
    bool bark_smooth_uvs = true;
    /// When true, 1→4-subdivide every bark triangle (edge midpoints) during prepare.
    /// Runs even when @ref bark_smooth_iterations is 0. Each bark edge split is also applied to
    /// adjacent non-bark faces (same meshlet interior, and the neighbor meshlet at segment interfaces).
    bool bark_subdivide = false;
    /// Alpha / radius cutoff for kinDS inside-outside classification (@ref TreeMesher::Settings::alpha_cutoff).
    double alpha_cutoff = 10.0;
    /// Cross-branch alpha cutoff (@ref TreeMesher::Settings::branch_alpha_cutoff). Disabled when equal to alpha_cutoff.
    double branch_alpha_cutoff = 10.0;
    /// Extra sections above floor(t)+1 when classifying same-branch membership for branch_alpha_cutoff (0 = default).
    size_t look_ahead = 0;
    /// Debug: for non-parallel profile-plane mixes, apply only the hinge about the intersection line
    /// (skip in-plane origin shift and rotation about the plane normal).
    bool hinge_only_profile_plane_mix = false;
    /// When true, apply inverse root transform to a loaded intersection boundary OBJ before meshlet clipping.
    bool intersection_boundary_apply_inverse_root_transform = true;
    /// Free-form note stored in mesh buffer YML metadata (not used for cache hashing).
    std::string meshing_buffer_description = "created manually through DynamicTreeStrands";
    /// When non-empty, used as the meshing-statistics CSV experiment tag (spaces → underscores at write).
    /// When empty, the active DynamicStrandsDemo type name is used instead.
    std::string meshing_statistics_experiment_name;
    /// When true, attempt to repair empty meshlets after boundary intersection (@ref TreeMesher::fixFailedSegments).
    bool intersection_boundary_fix_missing_meshes = false;
    /// When true, failed intersections keep the uncut meshlet; when false, replace with an empty mesh.
    bool intersection_keep_original_on_failure = true;
    /// Seam vertices on clip-boundary faces receive segment-meshlet UVs (editor intersection).
    bool intersection_prefer_meshlet_uv_on_seam = true;
    /// Clip-boundary-origin faces use interior (a,b,h) UVs; bark polar distance is treated as r=1.
    bool intersection_boundary_faces_interior_uv = true;
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
    // Per triangle corner (not per shared vertex). Keep layout in sync with KineticVoronoiMeshing.glsl.
    glm::vec4 normal[3];   // current / skinned
    glm::vec4 normal0[3];  // rest pose (prediction source)
    glm::vec4 uv[3];
    int segment_pair_index;
    int padding0;
    int padding1;
    int padding2;
  };
  static_assert(sizeof(GpuSegmentMeshletTriangle) == 176,
                "GpuSegmentMeshletTriangle must match GLSL SegmentMeshletTriangle std430 size");

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
  /// True after the last @ref RunMeshingAlgorithm completed without catching a failure.
  bool last_meshing_succeeded_ = true;
  std::vector<GpuSegmentMeshletTriangle> segment_meshlet_triangles;
  /// CPU-only JSON metadata parallel to @ref segment_meshlet_vertices / @ref segment_meshlet_triangles.
  /// Filled during @ref PopulateGpuMeshletBuffers when @ref MeshingSettings::store_mesh_metadata is true;
  /// kept in lockstep with @ref segment_meshlet_vertices / @ref segment_meshlet_triangles for OBJ export.
  /// never uploaded to the GPU. Reattached when rebuilding/exporting meshes after download.
  std::vector<std::string> segment_meshlet_vertex_metadata;
  std::vector<std::string> segment_meshlet_face_metadata;

  std::shared_ptr<Buffer> device_segment_meshlet_vertices_buffer;
  std::shared_ptr<Buffer> device_segment_meshlet_triangles_buffer;

  std::vector<float> boundary_distances_by_vertex;
  std::shared_ptr<kinDS::StrandTree> strand_tree;
  std::shared_ptr<kinDS::TreeMesher> tree_mesher_;
  /// Pristine meshlets from the last meshing run (after T-junction/bark prepare, before intersection).
  std::vector<kinDS::VoronoiMesh> segment_meshlets_;
  /// Neighbor indices matching @ref segment_meshlets_ before intersection.
  std::vector<std::vector<int>> meshing_neighbor_indices_;
  /// Welded bark-only debug mesh after @ref PrepareSegmentMeshletsForGpu. Empty until first prepare.
  kinDS::VoronoiMesh bark_debug_mesh_{};
  bool has_bark_debug_mesh_ = false;
  /// Root transform used when uploading meshlets to GPU (tree frame → GPU/world frame).
  GlobalTransform meshlets_root_transform_{};
  /// Cached from @ref InitData; used when rebuilding pair rest state after intersection compact.
  DynamicStrandsInitializeParameters initialize_parameters_{};

  /// Rest-pose meshlet volumes captured at the end of a successful meshing run (keyed by physics segment).
  std::unordered_map<unsigned int, double> initial_meshlet_volumes_by_segment_;
  double initial_meshlet_cumulative_volume_ = 0.0;
  bool has_initial_meshlet_volumes_ = false;

  /// Snapshot current CPU meshlet volumes (rest `x0`) as the baseline for volume-change heatmaps.
  void CaptureInitialMeshletVolumes();

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
  bool HasMeshedSegmentMeshlets() const;
  /// If CPU Voronoi meshlets are missing, rebuild them from the GPU vertex/triangle buffers
  /// (same geometry, different layout). Needed so intersection works after a mesh-buffer cache hit.
  bool EnsureCpuMeshletsFromGpu();
  /// Locate Kinetic Voronoi meshing for @p entity: nearest meshed DynamicTreeStrands on the entity or
  /// its ancestors; if ancestors are empty (e.g. PhysicsDemo host), any meshed DTS in the scene
  /// (Tree child from trunk experiments); otherwise the nearest empty KVM host.
  struct OwnerMeshing {
    DsKineticVoronoiMeshing* meshing = nullptr;
    Entity dts_owner{};
  };
  static OwnerMeshing FindForEntity(const std::shared_ptr<Scene>& scene, const Entity& entity);
  struct IntersectionRunStats {
    size_t inside_meshlets = 0;
    size_t intersecting_meshlets = 0;
    size_t outside_meshlets = 0;
    size_t input_poly_count = 0;
    double runtime_seconds = 0.0;
  };
  /// Clip meshlets against @p raw_mesh placed at @p boundary_world_transform and rebuild GPU buffers.
  /// @p tree_world_transform is the current world transform of the DynamicTreeStrands entity; used to
  /// convert the boundary from world space into tree-local space (where the raw meshlets live).
  /// When @p stats is non-null, fills classification counts, input poly count, and clip runtime.
  /// When @p apply_to_simulation is false, only rebuilds CPU meshlet export buffers without GPU upload
  /// or physics compaction (for "Intersect and export all"; each boundary is evaluated from pristine meshlets).
  bool IntersectMeshletsWithBoundary(const kinDS::VoronoiMesh& raw_mesh,
                                     const GlobalTransform& boundary_world_transform,
                                     const GlobalTransform& tree_world_transform,
                                     IntersectionRunStats* stats = nullptr, bool apply_to_simulation = true);
  /// Load an intersection-setup YAML under @p owner into a newly created Intersection Meshes group
  /// (existing groups are left untouched). Relative @c obj_path entries are resolved against the
  /// project assets folder, then the YAML directory.
  static bool LoadIntersectionSetup(const std::shared_ptr<Scene>& scene, const Entity& owner,
                                    const std::filesystem::path& yaml_path);
  /// Write a timestamped intersection statistics CSV. @p base_csv_path is the desired filename
  /// (e.g. @c foo_intersection_stats.csv); a timestamp is inserted before the extension.
  /// A @c total row summing poly count and runtime is included when @p rows has more than one entry.
  static void WriteIntersectionStatisticsCsv(const std::filesystem::path& base_csv_path,
                                             const std::vector<std::pair<std::string, IntersectionRunStats>>& rows);
  /// Reload pristine (pre-intersection) meshlets into GPU buffers.
  bool ResetMeshletsToGpu();
  void DownloadPhysicsSegmentsAndPairs();
  void UploadPhysicsSegmentsAndPairs();
  /// Remove OUTSIDE physics segments and densely remap survivors after boundary intersection.
  /// When @p rebuild_pairs is false, segment/map compaction still runs but pair topology is left
  /// empty for a subsequent @ref RecomputeSegmentPairs.
  void CompactSurvivingPhysicsSegments(const std::vector<size_t>& outside_meshing_indices, bool rebuild_pairs = true);
  /// Split INTERSECT meshlets with multiple triangle connected components into separate meshlets
  /// and physics segments (length-fitted on the parent axis). Requires CPU physics already downloaded.
  /// @return number of extra physics segments created.
  size_t SplitIntersectingMeshletsByConnectedComponents(const std::vector<size_t>& intersecting_meshing_indices);
  /// After @ref RecomputeSegmentPairs: refresh strand prev/next/begin/end and initialize pair materials.
  void FinalizeStrandConnectivityAndPairMaterials();

  /// Close T-junctions (cross- and intra-meshlet), then bark smooth / seam normals / cap UVs.
  /// Mutates @p meshes and @p meshing_neighbor_indices in place and rebuilds @ref bark_debug_mesh_.
  void PrepareSegmentMeshletsForGpu(std::vector<kinDS::VoronoiMesh>& meshes,
                                    std::vector<std::vector<int>>& meshing_neighbor_indices,
                                    const std::vector<size_t>& meshing_to_physics_segment_indices,
                                    const GlobalTransform& root_transform);

  /// Upload already-prepared CPU segment meshlets into GPU vertex/triangle buffers (no geometry edits).
  void PopulateGpuMeshletBuffers(const std::vector<kinDS::VoronoiMesh>& meshes,
                                 const std::vector<std::vector<int>>& physics_strand_to_segment_indices,
                                 const std::vector<std::vector<size_t>>& meshing_strand_to_segment_indices,
                                 const std::vector<std::vector<int>>& meshing_neighbor_indices,
                                 const std::vector<size_t>& meshing_to_physics_segment_indices,
                                 const GlobalTransform& root_transform);

  /// @ref PrepareSegmentMeshletsForGpu then sync members and @ref PopulateGpuMeshletBuffers.
  void PrepareAndPopulateGpuMeshletBuffers(std::vector<kinDS::VoronoiMesh>& meshes,
                                           const std::vector<std::vector<int>>& physics_strand_to_segment_indices,
                                           const std::vector<std::vector<size_t>>& meshing_strand_to_segment_indices,
                                           std::vector<std::vector<int>>& meshing_neighbor_indices,
                                           const std::vector<size_t>& meshing_to_physics_segment_indices,
                                           const GlobalTransform& root_transform);

  /// Returns false if meshing threw; previous mesh buffers are restored and a failure statistics
  /// CSV is written when collection is enabled.
  bool RunMeshingAlgorithm(const std::vector<std::vector<glm::dvec2>>& support_points,
                           std::vector<std::vector<double>>& subdivisions_by_strand,
                           std::vector<std::vector<int>>& physics_strand_to_segment_indices,
                           const std::vector<std::vector<glm::dmat4>>& transforms_by_height_and_branch,
                           const GlobalTransform& root_transform,
                           const std::vector<std::vector<size_t>>& branch_indices,
                           std::vector<std::vector<std::vector<size_t>>>& strands_by_branch_id,
                           float min_segment_length, float max_segment_length);

  /// Write a statistics CSV with @c failure filled (and alpha from current cutoff) before restoring state.
  void WriteMeshingFailureStatistics(const std::string& failure_message);
};
}  // namespace eco_sys_lab_plugin