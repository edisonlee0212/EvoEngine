#pragma once
#include "StrandGroup.hpp"
#include "StrandModelData.hpp"
#include "TreeGrowthData.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {
class DynamicStrandsPreStep;
class IDsPhysicsOperator;
class IDynamicStrandsConstraint;
class DynamicStrandsPrediction;

struct DtsStrandGroupData {};

struct DtsStrandData {};

struct DtsStrandSegmentData {
  /**
   * \brief The handle of the internode this pipe segment belongs to. Pipe -> PipeSegment <-> Cell <- Profile <-
   * Internode
   */
  SkeletonNodeHandle node_handle = -1;
  StrandSegmentHandle original_segment_handle;
  float segment_t;
  uint32_t segment_index;

  float initial_distance_to_boundary;
};

typedef StrandGroup<DtsStrandGroupData, DtsStrandData, DtsStrandSegmentData> DtsStrandGroup;

class DynamicStrands {
  bool wait_for_upload = true;

 public:
  DynamicStrands();
  [[nodiscard]] bool WaitForUpload() const;
#pragma region Initialization
  struct InitializeParameters {
    int sub_segment = 1;
    float wood_density = 400.f;  // kg/m^3
    SingleDistribution<float> youngs_modulus = {12.f, 1.2f};  // GPa
    SingleDistribution<float> shear_modulus = {12.f, 0.15f};    // GPa
    SingleDistribution<float> bending_modulus = {1.5f, 0.15f};  // GPa
    SingleDistribution<float> torsion_modulus = {1.5f, 0.15f};  // GPa

    float velocity_damping = 0.005f;
    float angular_velocity_damping = 0.005f;

    float neighbor_range = 6.0f;
    SingleDistribution<float> max_neighbor_strain = {0.1f, 0.1f};

    float min_neighbor_strain = 0.001f;

    SingleDistribution<glm::vec3> max_stretch_shear_strain = {glm::vec3(0.01f), 0.01f};
    SingleDistribution<glm::vec3> max_bend_twist_strain = {glm::vec3(0.05f), 0.05f};

    glm::vec3 min_stretch_shear_strain = glm::vec3(0.001f);
    glm::vec3 min_bend_twist_strain = glm::vec3(0.001f);

    GlobalTransform root_transform{};

    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };
  void Initialize(const InitializeParameters& initialize_parameters, const StrandModelSkeleton& strand_model_skeleton,
                  const DtsStrandGroup& strand_group);

#pragma endregion
#pragma region Step
  struct PhysicsParameters {
    float time_step = 0.01f;
    int sub_step = 5;
    int constraint_iteration = 1;
    bool allow_breaking = true;
    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };

  struct VisualizationParameters {
    enum class ParticleRenderMode { Default, SegmentColor, ConnectivityStrain };
    enum class SegmentRenderMode { Default, SegmentColor, StretchShearStrain };
    enum class ConnectionRenderMode { Default, BendTwistStrain };
    bool render_particles = true;
    bool render_segments = true;
    bool render_connections = true;
    uint32_t particle_render_mode = 0;
    uint32_t segment_render_mode = 0;
    uint32_t connection_render_mode = 0;

    glm::vec4 particle_color_min = glm::vec4(0, 0, 1, 1);
    glm::vec4 particle_color_max = glm::vec4(1, 0, 0, 1);
    glm::vec4 particle_color_main = glm::vec4(0.2, 1, 1, 0.5);
    float particle_multiplier = 1.0f;

    glm::vec4 segment_color_min = glm::vec4(0, 0, 1, 1);
    glm::vec4 segment_color_max = glm::vec4(1, 0, 0, 1);
    glm::vec4 segment_color_main = glm::vec4(0.6, 0.3, 0.0, 0.4);
    glm::vec4 segment_color_sub = glm::vec4(0.6, 0.3, 0.0, 0.1);
    float segment_multiplier = 1.0f;

    glm::vec4 connection_color_min = glm::vec4(0, 0, 1, 1);
    glm::vec4 connection_color_max = glm::vec4(1, 0, 0, 1);
    glm::vec4 connection_color_main = glm::vec4(1, 1, 1, 0.8);
    glm::vec4 connection_color_sub = glm::vec4(1, 1, 1, 0.2);
    float connection_multiplier = 1.0f;

    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };

  struct RenderParameters {
    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };

  std::shared_ptr<DynamicStrandsPreStep> pre_step;
  std::shared_ptr<DynamicStrandsPrediction> prediction;
  std::vector<std::shared_ptr<IDynamicStrandsConstraint>> constraints;

  void UpdateBindings() const;
#pragma endregion
#pragma region Shared Data
  struct GpuStrand {
    int begin_segment_handle = -1;
    int end_segment_handle = -1;

    int begin_connection_handle = -1;
    int end_connection_handle = -1;

    int begin_jump_segment_handle = -1;
    int end_jump_segment_handle = -1;

    int begin_jump_connection_handle = -1;
    int end_jump_connection_handle = -1;
  };

  struct GpuSegment {
    int prev_handle = -1;
    int next_handle = -1;
    int strand_handle = -1;
    float inv_mass;

    glm::vec4 color;

    // Initial rotation
    glm::quat q0;
    // Current rotation
    glm::quat q;
    // Last frame rotation
    glm::quat last_q;
    glm::quat old_q;
    glm::vec3 torque = glm::vec3(0.f);
    float rest_length;

    float radius;
    float shearing_alpha;
    float stretching_alpha;
    float damping;

    glm::vec3 inertia_tensor;
    int particle0_handle = -1;

    glm::vec3 inv_inertia_tensor;
    int particle1_handle = -1;

    glm::mat4 inertia_w;
    glm::mat4 inv_inertia_w;

    glm::vec3 stretch_shear_strain = glm::vec3(0.f);
    float original_inv_mass = 0.0f;

    glm::vec4 max_stretch_shear_strain;

    int prev_jump_handle;
    int next_jump_handle;
    int padding0;
    int padding1;
  };
  struct GpuParticle {
    // Initial position
    glm::vec3 x0;
    float damping;
    // Current position
    glm::vec3 x;
    int node_handle = -1;
    // Last frame position
    glm::vec3 last_x;
    int strand_handle = -1;
    glm::vec3 old_x;
    int segment_handle = -1;

    glm::vec3 acceleration = glm::vec3(0.f);
    float connectivity_strain;

    int selected = 0;
    int highlighted = 0;
    int connection_handle = 0;
    int padding2 = 0;
  };

  struct GpuConnection {
    int segment0_handle;
    int segment1_handle;
    int segment0_particle_handle;
    int segment1_particle_handle;

    glm::quat rest_darboux_vector;
    float bending_alpha;
    float torsion_alpha;

    int prev_handle = -1;
    int next_handle = -1;

    glm::vec4 bend_twist_strain_valid = glm::vec4(0.f);

    glm::vec4 max_bend_twist_strain;

    int prev_jump_handle;
    int next_jump_handle;
    int padding0;
    int padding1;
  };

  struct GpuDelaunayTetrahedron {
    int indices[4];
    int neighbors[4];
    int render_neighbor[4];
    float neighbor_circumference[4];
    glm::vec4 color;  // for debugging
    unsigned int task_looked_at = 0;
    unsigned int mesh_looked_at = 0;
    int inside = -1;
    int triangles_accepted = 0;
  };

  inline static std::shared_ptr<DescriptorSetLayout> strands_layout{};

  std::shared_ptr<Buffer> device_strands_buffer;
  std::shared_ptr<Buffer> device_segments_buffer;
  std::shared_ptr<Buffer> device_particles_buffer;
  std::shared_ptr<Buffer> device_connections_buffer;
  std::vector<GpuStrand> strands;
  std::vector<GpuSegment> segments;
  std::vector<GpuParticle> particles;
  std::vector<GpuConnection> connections;

  std::shared_ptr<Buffer> device_delaunay_tetrahedrons_buffer;
  std::vector<GpuDelaunayTetrahedron> delaunay_tetrahedrons;
#pragma endregion

  void Upload();
  void Download();

  void Clear();

  std::vector<std::shared_ptr<DescriptorSet>> strands_descriptor_sets;

  void Render(const std::shared_ptr<Camera>& target_camera, const RenderParameters& render_parameters) const;
  void Visualize(const std::shared_ptr<Camera>& target_camera,
                 const VisualizationParameters& visualization_parameters) const;
  void Physics(const PhysicsParameters& physics_parameters, 
      const std::function<void()>& operators_action,
               const std::function<void()>& pre_constraint_action) const;

 private:
  void ComputeDelaunay(std::vector<GpuDelaunayTetrahedron>& tetrahedrons);
  static glm::vec3 ComputeInertiaTensorBox(float mass, float width, float height, float depth);
  static glm::vec3 ComputeInertiaTensorRod(float mass, float radius, float length);
};
}  // namespace eco_sys_lab_plugin