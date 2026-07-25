#pragma once

#include "BtfBase.cuh"

#include "CUDABuffer.hpp"

#include "Optix7.hpp"

#include "Vertex.hpp"

#include "cuda.h"
#include "unordered_map"

#include "string"

#include "vector"

#include "map"

#include "Enums.hpp"
#include "MaterialProperties.hpp"
#include "PointCloudSample.hpp"
#include "VulkanInterlop.hpp"
#include "filesystem"

namespace evo_engine {
enum class OutputType { Color, Normal, Albedo, Depth };
enum class BackgroundType { Environment, Skybox, Color };
struct CameraProperties {
#pragma region FrameBuffer
  /*! the color buffer we use during _rendering_, which is a bit
  larger than the actual displayed frame buffer (to account for
  the border), and in float4 format (the denoiser requires
  floats) */
  CudaBuffer frame_buffer_color;
  CudaBuffer frame_buffer_normal;
  CudaBuffer frame_buffer_albedo;
#pragma endregion
#pragma region Denoiser
#if ENABLE_OPTIX_DENOISER
  /*! output of the denoiser pass, in float4 */
  CudaBuffer denoised_buffer;
  OptixDenoiser denoiser = nullptr;
  CudaBuffer denoiser_scratch;
  CudaBuffer denoiser_state;
  CudaBuffer denoiser_intensity;

#endif
#pragma endregion
  bool accumulate = true;
  float denoiser_strength = 0.0f;
  float fov = 120;
  /*! camera position - *from* where we are looking */
  glm::vec3 camera_position = glm::vec3(0.0f);
  /*! general up-vector */
  glm::vec3 horizontal_direction;
  glm::vec3 vertical_direction;
  glm::mat4 inverse_projection_view;

  float max_distance = 50.0f;
  std::shared_ptr<CudaImage> target_image;
  OutputType output_type = OutputType::Color;
  glm::vec4 background_color = glm::vec4(1.0f);
  BackgroundType background_type = BackgroundType::Environment;
  float gamma = 2.2f;
  struct {
    glm::vec4* color_buffer = nullptr;
    glm::vec4* normal_buffer = nullptr;
    glm::vec4* albedo_buffer = nullptr;
    /*! the size of the frame buffer to render */
    glm::uvec2 size = {};
    size_t frame_id = 0;
  } target_frame{};

  bool modified = false;

  float aperture = 0.0f;
  float focal_length = 1.0f;

  cudaTextureObject_t skybox = 0;
  void SetBackgroundType(BackgroundType background_type);
  void SetBackgroundColor(const glm::vec4& background_color);
  void SetAperture(float value);

  void SetFocalLength(float value);

  void SetFov(float value);

  void SetGamma(float value);

  void SetMaxDistance(float value);

  void SetOutputType(OutputType value);

  void Resize(const glm::uvec2& new_size);
  void SetDenoiserStrength(float value);
  void Set(const glm::vec3& position, const glm::quat& rotation);
  void SetSkybox(const std::shared_ptr<CudaImage>& cubemap);
  void DrawGui();
};

#pragma region MyRegion
enum class EnvironmentalLightingType { Scene, Skydome, SingleLightSource };

struct EnvironmentProperties {
  EnvironmentalLightingType environmental_lighting_type = EnvironmentalLightingType::Scene;
  float sky_light_intensity_scale = 1.0f;
  float indirect_lighting_intensity = 1.0f;
  float environment_rotation = 0.0f;
  float light_size = 0.0f;
  float gamma = 1.0f;
  glm::vec3 sun_direction = glm::vec3(0, 1, 0);
  glm::vec3 color = glm::vec3(1, 1, 1);
  bool use_environmental_map = false;
  cudaTextureObject_t environmental_map = 0;
  struct {
    float earth_radius = 6360;       // In the paper this is usually Rg or Re (radius ground, eart)
    float atmosphere_radius = 6420;  // In the paper this is usually R or Ra (radius atmosphere)
    float hr = 7994;                 // Thickness of the atmosphere if density was uniform (Hr)
    float hm = 1200;                 // Same as above but for Mie scattering (Hm)
    float g = 0.76f;                 // Mean cosine for Mie scattering
    int num_samples = 16;
    int num_samples_light = 8;
  } atmosphere;

  [[nodiscard]] bool Changed(const EnvironmentProperties& properties) const {
    return properties.environmental_lighting_type != environmental_lighting_type ||
           properties.use_environmental_map != use_environmental_map || properties.light_size != light_size ||
           properties.sky_light_intensity_scale != sky_light_intensity_scale ||
           properties.indirect_lighting_intensity != indirect_lighting_intensity ||
           properties.environment_rotation != environment_rotation || properties.gamma != gamma ||
           properties.sun_direction != sun_direction || properties.color != color ||
           properties.environmental_map != environmental_map ||
           properties.atmosphere.earth_radius != atmosphere.earth_radius ||
           properties.atmosphere.atmosphere_radius != atmosphere.atmosphere_radius ||
           properties.atmosphere.hr != atmosphere.hr || properties.atmosphere.hm != atmosphere.hm ||
           properties.atmosphere.g != atmosphere.g || properties.atmosphere.num_samples != atmosphere.num_samples ||
           properties.atmosphere.num_samples_light != atmosphere.num_samples_light;
  }

  void DrawGui();
};

struct RayProperties {
  int bounces = 4;
  int samples = 1;

  [[nodiscard]] bool Changed(const RayProperties& properties) const {
    return properties.bounces != bounces || properties.samples != samples;
  }

  void DrawGui();
};

struct RayTracerProperties {
  EnvironmentProperties environment;
  RayProperties ray_properties;

  [[nodiscard]] bool Changed(const RayTracerProperties& properties) const {
    return environment.Changed(properties.environment) || ray_properties.Changed(properties.ray_properties);
  }

  void DrawGui();
};

enum class RayType { Radiance, SpacialSampling, RayTypeCount };

struct CameraRenderingLaunchParams {
  CameraProperties camera_properties;
  RayTracerProperties ray_tracer_properties;
  OptixTraversableHandle traversable;
};

template <typename T>
struct IlluminationSampler {
  evo_engine::Vertex v_0;
  evo_engine::Vertex v_1;
  evo_engine::Vertex v_2;
  /**
   * \brief The calculated overall direction where the triangle received most
   * light.
   */
  glm::vec3 direction;
  /**
   * \brief The total energy received at this triangle.
   */
  T energy;
  bool front_face = true;
  bool back_face = true;

  [[nodiscard]] float GetArea() const {
    const float a = glm::distance(v_0.position, v_1.position);
    const float b = glm::distance(v_1.position, v_2.position);
    const float c = glm::distance(v_2.position, v_0.position);
    const float p = (a + b + c) * 0.5f;
    return glm::sqrt(p * (p - a) * (p - b) * (p - c));
  }

  [[nodiscard]] glm::vec3 GetCenter() const {
    return (v_0.position + v_1.position + v_2.position) / 3.f;
  }
};

struct IlluminationEstimationLaunchParams {
  unsigned seed = 0;
  float push_normal_distance = 0.001f;
  size_t size;
  RayTracerProperties ray_tracer_properties;
  IlluminationSampler<glm::vec3>* light_probes;
  OptixTraversableHandle traversable;
};

struct PointCloudScanningLaunchParams {
  size_t size;
  PointCloudSample* samples;
  OptixTraversableHandle traversable;
};

#pragma endregion

struct SurfaceMaterial;

struct RayTracedMaterial {
  MaterialType material_type = MaterialType::Default;

  BtfBase* btf_base;
  evo_engine::MaterialProperties material_properties;

  std::shared_ptr<CudaImage> albedo_texture;
  std::shared_ptr<CudaImage> normal_texture;
  std::shared_ptr<CudaImage> metallic_texture;
  std::shared_ptr<CudaImage> roughness_texture;

  size_t version = -1;
  uint64_t handle = 0;

  CudaBuffer material_buffer;

  bool remove_flag = true;

  void UploadForSbt();

  static void BindTexture(unsigned int id, cudaGraphicsResource_t& graphics_resource,
                          cudaTextureObject_t& texture_object);
};

enum class CurveMode { Linear, Quadratic, Cubic };

struct InstanceMatrix {
  glm::mat4 instance_matrix = {};
  glm::vec4 instance_color = glm::vec4(1.0f);
};

struct RayTracedGeometry {
  RendererType renderer_type = RendererType::Default;
  PrimitiveType geometry_type = PrimitiveType::Triangle;
  union {
    std::vector<evo_engine::Vertex>* vertices = nullptr;
    std::vector<evo_engine::SkinnedVertex>* skinned_vertices;
    std::vector<evo_engine::StrandPoint>* curve_points;
  };
  std::vector<glm::mat4>* bone_matrices = nullptr;
  std::vector<InstanceMatrix>* instance_matrices = nullptr;
  union {
    std::vector<glm::uvec3>* triangles = nullptr;
    std::vector<glm::uint>* curve_segments;
  };

  OptixTraversableHandle traversable_handle = 0;

  CudaBuffer vertex_data_buffer;
  CudaBuffer curve_strand_u_buffer;
  CudaBuffer curve_strand_i_buffer;
  CudaBuffer curve_strand_info_buffer;

  CudaBuffer triangle_buffer;
  CudaBuffer accelerated_structure_buffer;
  size_t version = -1;
  uint64_t handle = 0;
  bool update_flag = false;
  bool remove_flag = true;

  void BuildGas(const OptixDeviceContext& context);

  void UploadForSbt();

  CudaBuffer geometry_buffer;
};

struct RayTracedInstance {
  uint64_t entity_handle = 0;
  size_t version = -1;
  size_t data_version = -1;
  uint64_t private_component_handle = 0;

  uint64_t geometry_map_key = 0;
  uint64_t material_map_key = 0;
  glm::mat4 global_transform;
  bool remove_flag = true;
};

struct RayTracerPipeline {
  std::string launch_params_name;
  OptixModule module;
  OptixModule quadratic_curve_module;
  OptixModule cubic_curve_module;
  OptixModule linear_curve_module;
  OptixModuleCompileOptions module_compile_options = {};

  OptixPipeline pipeline;
  OptixPipelineCompileOptions pipeline_compile_options = {};
  OptixPipelineLinkOptions pipeline_link_options = {};

  OptixProgramGroup ray_gen_program_groups;
  CudaBuffer ray_gen_records_buffer;
  std::map<RayType, std::map<PrimitiveType, OptixProgramGroup>> hit_group_program_groups;
  CudaBuffer miss_records_buffer;
  std::map<RayType, OptixProgramGroup> miss_program_groups;
  CudaBuffer hit_group_records_buffer;
  OptixShaderBindingTable sbt = {};
  CudaBuffer launch_params_buffer;
};
struct SurfaceCompressedBtf;
struct MlvqMaterialStorage {
  std::shared_ptr<SurfaceCompressedBtf> material;
  CudaBuffer buffer;
};

class OptiXRayTracer {
 public:
  bool scene_modified = false;
  std::unordered_map<uint64_t, RayTracedMaterial> materials;
  std::unordered_map<uint64_t, RayTracedGeometry> geometries;
  std::unordered_map<uint64_t, RayTracedInstance> instances;

  // ------------------------------------------------------------------
  // internal helper functions
  // ------------------------------------------------------------------
  [[nodiscard]] bool RenderToCamera(const EnvironmentProperties& environment_properties,
                                    CameraProperties& camera_properties, const RayProperties& ray_properties);

  void EstimateIllumination(const size_t& size, const EnvironmentProperties& environment_properties,
                            const RayProperties& ray_properties, const CudaBuffer& light_probes, unsigned seed,
                            float push_normal_distance);

  void ScanPointCloud(const size_t& size, const EnvironmentProperties& environment_properties,
                      const CudaBuffer& samples);

  OptiXRayTracer();
  ~OptiXRayTracer();
  /*! build an acceleration structure for the given triangle mesh */
  void BuildIas();

  /*! constructs the shader binding table */
  void BuildSbt();

 protected:
#pragma region Device and context
  /*! @{ CUDA device context and stream that optix pipeline will run
                  on, as well as device properties for this device */
  CUcontext cuda_context_;
  CUstream stream_;
  cudaDeviceProp device_props_;
  /*! @} */
  //! the optix context that our pipeline will run in.
  OptixDeviceContext optix_device_context_;

  friend struct CameraProperties;

  /*! creates and configures a optix device context (in this simple
    example, only for the primary GPU device) */
  void CreateContext();

#pragma endregion
#pragma region Pipeline setup

  CameraRenderingLaunchParams camera_rendering_launch_params_;
  IlluminationEstimationLaunchParams illumination_estimation_launch_params_;
  PointCloudScanningLaunchParams point_cloud_scanning_launch_params_;

  RayTracerPipeline camera_rendering_pipeline_;
  RayTracerPipeline illumination_estimation_pipeline_;
  RayTracerPipeline point_cloud_scanning_pipeline_;

  /*! creates the module that contains all the programs we are going
    to use. in this simple example, we use a single module from a
    single .cu file, using a single embedded ptx string */
  void CreateModules();

  /*! does all setup for the rayGen program(s) we are going to use */
  void CreateRayGenPrograms();

  /*! does all setup for the miss program(s) we are going to use */
  void CreateMissPrograms();

  /*! does all setup for the hitGroup program(s) we are going to use */
  void CreateHitGroupPrograms();

  /*! assembles the full pipeline of all programs */
  void AssemblePipelines();

  void CreateRayGenProgram(RayTracerPipeline& target_pipeline, char entry_function_name[]) const;

  void CreateModule(RayTracerPipeline& target_pipeline, char ptx_code[], char launch_params_name[]) const;

  void AssemblePipeline(RayTracerPipeline& target_pipeline) const;

#pragma endregion

#pragma region Accleration structure
  /*! check if we have build the acceleration structure. */
  bool has_acceleration_structure_ = false;
  //! buffer that keeps the (final, compacted) acceleration structure
  CudaBuffer ias_buffer_;
#pragma endregion

  friend class RayTracerCamera;
};

}  // namespace evo_engine
