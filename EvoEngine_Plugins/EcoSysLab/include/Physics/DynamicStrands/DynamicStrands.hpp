
#pragma once
#include "Delaunay.hpp"
#include "DsAlphaShapeMeshing.hpp"
#include "DsKineticVoronoiMeshing.hpp"
#include "DsMaterials.hpp"
#include "DtsStrandGroup.hpp"
#include "DynamicStrandsInitializationParameters.hpp"
#include "RenderLayer.hpp"
#include "RenderParameters.hpp"
#include "ShootGrowthData.hpp"
#include "StrandGroup.hpp"
#include "StrandModelData.hpp"
namespace eco_sys_lab_plugin {
class DsSegmentCollisionPostStep;
}
namespace eco_sys_lab_plugin {
class DsSegmentCollision;
class DsDynamicHashedGrid;
}  // namespace eco_sys_lab_plugin

#ifdef USE_RENDERDOC
#  include "C:\Program Files\RenderDoc\renderdoc_app.h"
static RENDERDOC_API_1_1_2* rdoc_api = NULL;
#endif

namespace eco_sys_lab_plugin {
using namespace evo_engine;

class DsSegmentCollision;
class DsDynamicHashedGrid;
class DsPreStep;
class IDsPhysicsOperator;
class IDsConstraint;
class DsPrediction;
class DsFungus;
class DsStructuralDamage;
class DsVelocityUpdate;

#define BUNDLE_MAX_CONNECTION 64
#define HASH_GRID_CELL_SIZE 2 << 15

/**
 * \brief Class responsible for handling dynamic strands physics simulation.
 */
class DynamicStrands {
 private:
  void OnCreate();

 public:
  /**
   * \brief Default constructor for DynamicStrands.
   */
  DynamicStrands(std::shared_ptr<DsAlphaShapeMeshing> meshing, DsMaterials& materials)
      : materials(materials), meshing(meshing) {
    OnCreate();
  }

  /**
   * \brief Default constructor for DynamicStrands.
   */
  DynamicStrands(std::shared_ptr<DsKineticVoronoiMeshing> meshing, DsMaterials& materials)
      : materials(materials), meshing(meshing) {
    OnCreate();
  }

  /**
   * \brief Gets the current frame index of the simulation.
   * \return The current frame index.
   */
  uint32_t GetFrameIndex() const;

  /**
   * \brief Gets the accumulated simulated time.
   * \return The total simulated time.
   */
  float GetSimulatedTime() const;

  /**
   * \brief Computes the inertia tensor for a box.
   * \param mass The total mass of the box.
   * \param width The width of the box.
   * \param height The height of the box.
   * \param depth The depth of the box.
   * \return Computed inertia tensor as a 3D vector.
   */
  static glm::vec3 ComputeInertiaTensorBox(float mass, float width, float height, float depth);

  /**
   * \brief Computes the inertia tensor for a rod.
   * \param mass The mass of the rod.
   * \param radius The radius of the rod.
   * \param length The length of the rod.
   * \return Computed inertia tensor as a 3D vector.
   */
  static glm::vec3 ComputeInertiaTensorRod(float mass, float radius, float length);

#pragma region Initialization

  /**
   * \brief Initializes strand data with specified parameters and structures.
   * \param random_engine random engine used for randomization.
   * \param initialize_parameters The initialization parameters.
   * \param strand_model_skeleton The skeleton structure for the strand model.
   * \param strand_model_strand_group The strand group in the model.
   * \param randomly_subdivided_strand_group Randomly subdivided strand group, for physics simulation.
   * \param uniformly_subdivided_strand_group Uniformly subdivided strand group, for meshing and rendering.
   */
  void InitializeData(std::mt19937& random_engine, const DynamicStrandsInitializeParameters& initialize_parameters,
                      const StrandModelSkeleton& strand_model_skeleton,
                      const StrandModelStrandGroup& strand_model_strand_group,
                      DtsStrandGroup& randomly_subdivided_strand_group,
                      DtsStrandGroup& uniformly_subdivided_strand_group);

  /**
   * \brief Initializes the strand mesh with given parameters.
   * \param initialize_parameters The initialization parameters used for mesh construction.
   */
  void InitializeMesh(const DynamicStrandsInitializeParameters& initialize_parameters);
#pragma endregion
#pragma region Step
  struct PhysicsParameters {
    float time_step = 0.01f;
    int sub_step = 25;

    int position_constraint_iteration = 1;
    int velocity_constraint_iteration = 1;

    bool enable_structural_damage = true;

    bool enable_segment_breaking = true;
    bool enable_segment_disconnection = true;
    bool enable_foliage_detachment = true;

    bool enable_segment_tensile_disconnection = true;
    bool enable_segment_compression_disconnection = true;
    float compression_strength_factor = 5.f;

    bool enable_positional_breaking = true;
    bool enable_rotational_breaking = true;

    float segment_velocity_damping = 1.f;
    float segment_angular_velocity_damping = 1.f;

    float leaf_velocity_damping = 10.f;
    float leaf_angular_velocity_damping = 10.f;

    bool enable_segment_collision = false;
    bool dynamic_grouping = false;
    int grouping_iteration = 128;
    glm::vec3 gravity = glm::vec3(0, -9.81f, 0);
    float fungus_growth_rate = 0.1f;  ///< The growth rate of the fungus.

    bool enable_fungus = false;

    float a_geom = 0.03f;
    float b_vel = 0.5f;
    float c_bias = 0.f;
    float s_min = 1e-5f;
    float s_max_ratio = 0.2f;
    float eta = 0.75f;
    float bmax_far = 10.0f * s_min;

    float dt = 0.0005f;
    float aw = 5.0f;  // growth rate
    float ab = 5.0f;
    float bw = 2.0f;  // chemical defense
    float bb = 2.0f;
    float ycw = 1.0f;     // carbon tissue damage (w)
    float ycb = 1.0f;     // carbon tissue damage (b)
    float ylw = 2.0f;     // lignin tissue damage (w)
    float pc = 0.2f;      // carbon regeneration
    float pl = 0.1f;      // lignin regeneration
    float k = 0.2f;       // defense rate
    float delta = 0.05f;  // defense decay rate
    float ll = 0.5f;      // lignin weight for w
    float lc = 0.5f;      // carbon weight for w
    float bo = 1.0f;
    float be = 2.0f;
    float lignin_threshold = -1.0f;
    glm::mat3 matrixAw = glm::mat3(0.1f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 2.0f);
    glm::mat3 matrixAb = glm::mat3(0.1f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 2.0f);
    glm::mat3 matrixAc = glm::mat3(0.01f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.01f);

    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };

  std::shared_ptr<DsFungus> fungus;
  std::shared_ptr<DsPreStep> pre_step;
  std::shared_ptr<DsPrediction> prediction;
  std::shared_ptr<DsStructuralDamage> structural_damage;
  std::shared_ptr<DsVelocityUpdate> velocity_update;
  std::shared_ptr<DsDynamicHashedGrid> dynamic_hashed_grid;
  std::shared_ptr<DsSegmentCollision> segment_collision;
  std::shared_ptr<DsSegmentCollisionPostStep> collision_post_step;
  std::vector<std::shared_ptr<IDsConstraint>> constraints;
  std::shared_ptr<DsMeshing> meshing;

  void UpdateBindings() const;
#pragma endregion
#pragma region Shared Data
  struct GpuStrand {
    int begin_segment_handle = -1;
    int end_segment_handle = -1;

    int begin_segment_pair_handle = -1;
    int end_segment_pair_handle = -1;

    int front_propagate_begin_segment_pair_handle = -1;
    int back_propagate_begin_segment_pair_handle = -1;
    int front_propagate_begin_segment_handle = -1;
    int back_propagate_begin_segment_handle = -1;

    int alternative_front_propagate_begin_segment_pair_handle = -1;
    int alternative_back_propagate_begin_segment_pair_handle = -1;
    int alternative_front_propagate_begin_segment_handle = -1;
    int alternative_back_propagate_begin_segment_handle = -1;
  };

  struct GpuNode {
    int prev_handle = -1;
    int padding0;
    int padding1;
    int padding2;
  };

  struct GpuParticle {
    // Initial position
    glm::vec3 x0;
    float padding0 = 0.f;
    // Current position
    glm::vec3 x;
    int selected = 0;
    // Last frame position
    glm::vec3 last_x;
    int highlighted = 0;

    // Velocity
    glm::vec3 v;
    int hop_distance_to_root = -1;

    glm::vec3 acceleration = glm::vec3(0.f);
    int node_handle = -1;

    float dmin_external = 1e30;
    float user_bound = 1e30;
    float root_distance = 0.f;
    int padding2 = 0;
  };

  struct GpuSegment {
    int prev_handle = -1;
    int next_handle = -1;
    int strand_handle = -1;
    float inv_mass = 0.0f;
    glm::vec4 color;

    // Initial rotation
    glm::quat q0;
    // Current rotation
    glm::quat q;
    // Last frame rotation
    glm::quat last_q;
    // Angular velocity
    glm::vec3 angular_v;
    float radius = 0.0f;

    glm::vec3 torque = glm::vec3(0.f);
    float rest_length = 0.0f;

    float max_young_modulus = 0.0f;
    float shear_stretch_alpha = 0.0f;
    float strength = 0.0f;
    float boundary_distance = 0.0f;

    glm::vec2 profile_position;
    glm::vec2 profile_polar_coordinate;

    glm::vec3 inertia_tensor;
    float max_shear_stretch_strain = 0.0f;

    glm::vec3 inv_inertia_tensor;
    float shear_stretch_strain_limit = 0.0f;

    glm::mat4 inertia_w;
    glm::mat4 inv_inertia_w;

    float shear_stretch_strain = 0.0f;
    int32_t node_handle = 0.0f;
    float original_mass = 0.0f;
    int32_t group_index = 0;

    float extra_mass = 0.f;
    float snow_amount = 0.f;
    float screen_depth = 0.0f;
    int reach_ground;

    float C = 0.2f;
    float HC = 1.0f;
    float HL = 1.0f;
    float RW = 0.0f;

    float RB = 0.0f;
    float C_pre = 0.2f;
    float HC_pre = 1.0f;
    float HL_pre = 1.0f;

    float RW_pre = 0.0f;
    float RB_pre = 0.0f;
    float K = 0.2f;
    float diffusion_c = 0.f;

    float diffusion_w = 0.f;
    float diffusion_b = 0.f;

    int32_t pairs_count = 0;
    float moisture = 1.0f;

    float moisture_pre = 1.0f;
    float diffusion_m = 0.f;
    int32_t internal_pattern = 0;
    int32_t cube_pattern = 0;

    int32_t prev_inside = 1;
    float ground_damping = 1.0f;
    int32_t quasi_stable = 0;
    float quasi_damping = 1.0f;

    GpuParticle particle0{};
    GpuParticle particle1{};

    glm::vec3 GetCenterX0() const;
  };

  struct GpuSegmentPair {
    int segment0_handle;
    int segment1_handle;
    float bend_twist_bundle_integrity = 1.f;
    float connectivity_integrity = 1.f;
    float bending_alpha = 0.0f;
    float torsion_alpha = 0.0f;
    float max_bending_modulus;
    float max_torsion_modulus;

    glm::vec4 segment0_offset;
    glm::vec4 segment1_offset;

    glm::quat rest_darboux_vector;

    glm::vec3 bending_twist_bundle_strain;
    float connectivity_strain = 0.0f;
    glm::vec3 max_bending_twist_bundle_strain;
    float max_connectivity_strain = 0.f;
    glm::vec3 bending_twist_bundle_strain_limit;
    float connectivity_strain_limit = 0.0f;

    int tensile_lock = 0;
    int compression_lock = 0;
    int positional_lock = 0;
    int rotational_lock = 0;
  };

  struct GpuSegmentData {
    glm::vec3 particle0_position_correction;
    float padding0 = 0.0f;
    glm::vec3 particle1_position_correction;
    float padding1 = 0.0f;

    glm::quat q_correction;

    int pair_handles[BUNDLE_MAX_CONNECTION];
  };

  struct GpuLeaf {
    glm::vec3 x0;
    int segment_handle;
    glm::vec3 x;
    float attachment_integrity;
    glm::vec3 last_x;
    float rotation_integrity;
    glm::quat q0;
    glm::quat q;
    glm::quat last_q;

    glm::vec3 scale;
    float inv_mass;

    glm::vec3 position_offset;
    float original_mass;

    glm::vec3 v;
    float position_strain;

    glm::vec3 acceleration;
    float rotation_strain;

    glm::vec3 angular_v;
    float position_alpha;

    glm::vec3 torque;
    float rotation_alpha;

    glm::vec3 inertia_tensor;
    float rotation_strain_limit;
    glm::vec3 inv_inertia_tensor;
    float position_strain_limit;

    glm::mat4 inertia_w;
    glm::mat4 inv_inertia_w;

    float extra_mass = 0.f;
    float property1 = 0.f;
    float property2 = 0.f;
    float property3 = 0.f;

    int selected = 0;
    int highlighted = 0;
    int detachment_lock = 0;
    int padding1 = 0;
  };

  struct GpuHashedGridElement {
    uint32_t cell_id;
    uint32_t segment_handle;
    uint32_t padding0;
    uint32_t padding1;
  };

  struct GpuHashedGridCellStart {
    uint32_t start_index = -1;
    uint32_t end_index;
    uint32_t padding1;
    uint32_t padding2;
  };

  inline static std::shared_ptr<DescriptorSetLayout> strands_layout{};
  uint32_t connection_segment_pair_size = 0;
  std::shared_ptr<Buffer> device_strands_buffer;
  std::shared_ptr<Buffer> device_segments_buffer;
  std::shared_ptr<Buffer> device_segment_pairs_buffer;
  std::shared_ptr<Buffer> device_segment_data_list_buffer;
  std::shared_ptr<Buffer> device_hashed_grid_elements_buffer;
  std::shared_ptr<Buffer> device_hashed_grid_cell_starts_buffer;
  std::shared_ptr<Buffer> device_foliage_buffer;

  std::vector<GpuStrand> strands;
  std::vector<GpuSegment> segments;
  std::vector<GpuSegmentPair> segment_pairs;
  std::vector<GpuSegmentData> segment_data_list;
  std::vector<GpuHashedGridElement> hashed_grid_elements;
  std::vector<GpuHashedGridCellStart> hashed_grid_cell_starts;
  std::shared_ptr<Buffer> device_nodes_buffer;
  std::vector<GpuNode> nodes;
  std::vector<GpuLeaf> foliage;
#pragma endregion

  void Upload();
  void Download();

  void CalculateGroups(const PhysicsParameters& physics_parameters) const;
  void Clear();

  std::vector<std::shared_ptr<DescriptorSet>> strands_descriptor_sets;

  uint32_t RenderFoliageToPointLightShadowMap(const FoliageRenderParameters& render_parameters,
                                              const VkCommandBuffer vk_command_buffer,
                                              const RenderLayer::PointLightShadowMapView& view) const;
  uint32_t RenderFoliageToSpotLightShadowMap(const FoliageRenderParameters& render_parameters,
                                             VkCommandBuffer vk_command_buffer,
                                             const RenderLayer::SpotLightShadowMapView& view) const;
  uint32_t RenderFoliageToDirectionalLightShadowMap(const FoliageRenderParameters& render_parameters,
                                                    VkCommandBuffer vk_command_buffer,
                                                    const RenderLayer::DirectionalLightShadowMapView& view) const;
  uint32_t RenderFoliageToCameraDeferred(
      const Handle& renderer_handle, const FoliageRenderParameters& render_parameters,
      VkCommandBuffer vk_command_buffer,
      const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
      const RenderLayer::DeferredRenderingView& view) const;

  uint32_t RenderSegmentPairsToCameraForward(int material_index,
                                             const DynamicStrandsInitializeParameters& initialize_parameters,
                                             const SegmentPairsRenderParameters& render_parameters,

                                             VkCommandBuffer vk_command_buffer,
                                             const std::shared_ptr<Camera>& target_camera,
                                             const RenderLayer::ForwardRenderingView& view) const;

  void Visualize(const std::shared_ptr<Camera>& target_camera,
                 const DynamicStrandsInitializeParameters& initialize_parameters,
                 const DynamicStrandsVisualizationParameters& visualization_parameters) const;
  void Physics(const PhysicsParameters& physics_parameters, const std::function<void()>& pre_step_action);
  void RenderCompute() const;
  static void BuildFoliageRenderingPipelines();
  static void BuildSegmentPairsRenderingPipeline();

  inline static std::shared_ptr<GraphicsPipeline> foliage_point_light_render_pipeline{};
  inline static std::shared_ptr<GraphicsPipeline> foliage_spot_light_render_pipeline{};
  inline static std::shared_ptr<GraphicsPipeline> foliage_directional_light_render_pipeline{};
  inline static std::shared_ptr<GraphicsPipeline> foliage_render_pipeline{};

  inline static std::shared_ptr<GraphicsPipeline> segment_pairs_visualization_render_pipeline{};

  DsMaterials& materials;

 private:
  uint32_t frame_index = 0;
  float simulated_time = 0.f;
};
}  // namespace eco_sys_lab_plugin
