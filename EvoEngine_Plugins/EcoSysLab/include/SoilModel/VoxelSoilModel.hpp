#pragma once

#include <functional>
#include <glm/glm.hpp>
#include <map>
#include <random>
#include <valarray>
#include <vector>

using namespace evo_engine;

namespace eco_sys_lab {

using Field = std::valarray<float>;

class SoilParameters;

struct SoilSurface {
  std::function<float(const glm::vec2& position)> m_height;
};

struct SoilMaterialTexture {
  std::vector<float> m_metallic_map;
  std::vector<float> m_roughness_map;
  std::vector<glm::vec3> m_normal_map;
  std::vector<glm::vec4> m_color_map;
  std::vector<float> m_height_map;
};

struct SoilPhysicalMaterial {
  int m_id = -1;
  std::function<float(const glm::vec3& position)> m_c;  // capacity
  std::function<float(const glm::vec3& position)> m_p;  // permeability
  std::function<float(const glm::vec3& position)> m_d;  // density

  std::function<float(const glm::vec3& position)> m_n;  // initial amount of nutrients
  std::function<float(const glm::vec3& position)> m_w;  // initial amount of water
  std::shared_ptr<SoilMaterialTexture> m_soilMaterialTexture;
};

struct SoilLayer {
  SoilPhysicalMaterial m_mat;
  std::function<float(const glm::vec2& position)> m_thickness;
};

class VoxelSoilModel {
  friend class Soil;
  friend class EcoSysLabLayer;

 public:
  enum class Boundary : int { sink, block, wrap, absorb };

  class Source {
   public:
    std::vector<int> idx;
    std::vector<float> amounts;
    void Apply(Field& target);
  };

  void Initialize(const SoilParameters& p, const SoilSurface& soilSurface, const std::vector<SoilLayer>& soilLayers);

  void Reset();
  void Run(float t_in_hrs);  // simulates a given amount of hours
  void Step();               // performs a single forward step (same as calling Run(t = m_dt);
  void Irrigation();         // can be called for each step to add some water to the volume

  [[nodiscard]] float IntegrateWater(const glm::vec3& position,
                                     float width) const;  // returns the amount of water in grams within a certain area.
  [[nodiscard]] float GetWaterDensity(const glm::vec3& position)
      const;  // returns the water density at one position (position rounded to nearest voxel). Unit is g / cm^3.

  [[nodiscard]] float IntegrateNutrient(const glm::vec3& position, float width) const;
  [[nodiscard]] float GetNutrientDensity(const glm::vec3& position) const;

  [[nodiscard]] float GetDensity(const glm::vec3& position) const;
  [[nodiscard]] float GetCapacity(const glm::vec3& position) const;

  void ChangeWater(const glm::vec3& center, float amount_in_g,
                   float width);  // distributes the given amount of grams of water in the area around the center
  void ChangeNutrient(const glm::vec3& center, float amount_in_AU, float width);
  void ChangeDensity(const glm::vec3& center, float amount, float width);
  void ChangeCapacity(const glm::vec3& center, float amount, float width);

  // negative indices are useful as relative offsets
  [[nodiscard]] static int Index(const glm::ivec3& resolution, int x, int y, int z);
  [[nodiscard]] int Index(int x, int y, int z) const;
  [[nodiscard]] static int Index(const glm::ivec3& resolution, const glm::ivec3& coordinate);
  [[nodiscard]] int Index(const glm::ivec3& coordinate) const;

  [[nodiscard]] glm::ivec3 GetCoordinateFromIndex(int index) const;
  [[nodiscard]] glm::ivec3 GetCoordinateFromPosition(const glm::vec3& position) const;
  [[nodiscard]] glm::vec3 GetPositionFromCoordinate(const glm::ivec3& coordinate, float dx, float dy, float dz) const;
  [[nodiscard]] glm::vec3 GetPositionFromCoordinate(const glm::ivec3& coordinate) const;  // uses m_dx as dx

  [[nodiscard]] glm::ivec3 GetVoxelResolution() const;
  [[nodiscard]] float GetVoxelSize() const;
  [[nodiscard]] float GetTime() const;
  [[nodiscard]] glm::vec3 GetBoundingBoxCenter() const;
  [[nodiscard]] glm::vec3 GetBoundingBoxMin() const;
  [[nodiscard]] glm::vec3 GetBoundingBoxMax() const;
  [[nodiscard]] bool PositionInsideVolume(const glm::vec3& position) const;
  [[nodiscard]] bool CoordinateInsideVolume(const glm::ivec3& coordinate) const;
  [[nodiscard]] bool Initialized() const;

  void GetSoilTextureSlideZ(
      bool backFacing, float z, const glm::vec2& xyMin, const glm::vec2& xyMax, std::vector<glm::vec4>& albedoData,
      std::vector<glm::vec3>& normalData, std::vector<float>& roughnessData, std::vector<float>& metallicData,
      glm::ivec2& outputResolution, float waterFactor, float nutrientFactor,
      float blur_width = 1);  // the output as well as all input textures must have the same resolution!
  void GetSoilTextureSlideX(
      bool backFacing, float x, const glm::vec2& yzMin, const glm::vec2& yzMax, std::vector<glm::vec4>& albedoData,
      std::vector<glm::vec3>& normalData, std::vector<float>& roughnessData, std::vector<float>& metallicData,
      glm::ivec2& outputResolution, float waterFactor, float nutrientFactor,
      float blur_width = 1);  // the output as well as all input textures must have the same resolution!

  void GetSoilTextureColorForPosition(const glm::vec3& position, int texture_idx, float blur_width, glm::vec4& albedo,
                                      glm::vec3& normal, float& roughness, float& metallic, float waterFactor,
                                      float nutrientFactor);

  int m_version = 0;  // TODO: what does this do?
 protected:
  void BuildFromLayers();  // helper function called inside initialize to set up soil layers
  void SetVoxel(const glm::ivec3& coordinate, const SoilPhysicalMaterial& material);

  [[nodiscard]] float GetField(const Field& field, const glm::vec3& position, float default_value) const;
  void ChangeField(Field& field, const glm::vec3& center, float amount_in_cm3, float width_in_m);
  [[nodiscard]] float IntegrateFieldValue(const Field& field, const glm::vec3& center, float width)
      const;  // returns the cm3 of a certain quantity within the width in the field
  void SetField(Field& field, const glm::vec3& bb_min, const glm::vec3& bb_max, float value);
  void BlurField(Field& field);  // for now there is just one standard kernel

  void AddWaterSource(Source&& source);
  void AddNutrientSource(Source&& source);

  void Convolution3(const Field& input, Field& output, const std::vector<int>& indices,
                    const std::vector<float>& weights) const;

  // Boundary stuff

  void Boundary_Wrap_Axis(const Field& input, Field& output, const std::vector<int>& indices_1D,
                          const std::vector<float>& weights, int lim_a, int lim_b, int lim_f,
                          std::function<int(int, int, int)> WrapIndex) const;
  void Boundary_Wrap_X(const Field& input, Field& output, const std::vector<int>& indices_1D,
                       const std::vector<float>& weights) const;
  void Boundary_Wrap_Y(const Field& input, Field& output, const std::vector<int>& indices_1D,
                       const std::vector<float>& weights) const;
  void Boundary_Wrap_Z(const Field& input, Field& output, const std::vector<int>& indices_1D,
                       const std::vector<float>& weights) const;

  void Boundary_Barrier_Axis(const Field& input, Field& output, const std::vector<int>& indices_1D,
                             const std::vector<float>& weights, int lim_a, int lim_b, int lim_f,
                             std::function<int(int, int, int)> WrapIndex) const;
  void Boundary_Barrier_X(const Field& input, Field& output, const std::vector<int>& indices_1D,
                          const std::vector<float>& weights) const;
  void Boundary_Barrier_Y(const Field& input, Field& output, const std::vector<int>& indices_1D,
                          const std::vector<float>& weights) const;
  void Boundary_Barrier_Z(const Field& input, Field& output, const std::vector<int>& indices_1D,
                          const std::vector<float>& weights) const;

  //////

  // Debug and test Functions:
  void UpdateStats();  // updates sum of water and max speeds
  void Test_InitializeEmpty(glm::uvec3 resolution);
  void Test_WaterDensity();
  void Test_PermeabilitySpeed();

  void Test_NutrientTransport(float p, float c, const std::shared_ptr<SoilMaterialTexture>& texture);
  void Test_NutrientTransport_Sand(const std::shared_ptr<SoilMaterialTexture>& texture);
  void Test_NutrientTransport_Loam(const std::shared_ptr<SoilMaterialTexture>& texture);
  void Test_NutrientTransport_Silt(const std::shared_ptr<SoilMaterialTexture>& texture);

  bool m_initialized = false;

  glm::ivec3 m_resolution;
  float m_dx;  // delta x, distance between two voxels
  float m_voxel_volume_in_cm3;
  float m_water_g_per_cm3;                   // how much water in g a volume of 1 cm^3 with density 1 contains
  float m_nutrient_unit_per_cm3;             // how much nutrient units a volume of 1 cm^3 with density 1 contains
  float m_dt;                                // delta t, time between steps, also measured in hrs
  float m_time_since_start_in_hrs = 0.0f;    // time since start, always a multiple of m_dt
  float m_time_since_start_requested = 0.f;  // up until which time we should simulate

  // scaling factors for different forces
  float m_diffusionForce;
  glm::vec3 m_gravityForce;
  float m_nutrientForce;

  // Fields:
  std::valarray<int> m_material_id;  // material id for each foxel in the soil volume

  Field m_w;  // water density of each cell. Unit is g / cm^3
  Field m_c;  // the capacity of each cell
  Field m_l;  // filling level of each cell, w/c
  Field m_p;  // permeability of each cell

  // these field are temporary variables but kept so we don't have to reallocate them each step
  Field m_w_grad_x;
  Field m_w_grad_y;
  Field m_w_grad_z;

  Field m_div_diff_x;  // divergence components for diffusion process
  Field m_div_diff_y;
  Field m_div_diff_z;

  Field m_div_diff_n_x;  // divergence components for diffusion process of nutrients
  Field m_div_diff_n_y;
  Field m_div_diff_n_z;

  Field m_div_grav_x;  // divergence components for gravity
  Field m_div_grav_y;
  Field m_div_grav_z;

  Field m_div_grav_n_x;  // divergence components for gravity of nutrients
  Field m_div_grav_n_y;
  Field m_div_grav_n_z;

  // nutrients
  Field m_n;
  // soil density
  Field m_d;
  Boundary m_boundary_x, m_boundary_y, m_boundary_z;
  int m_absorption_width = 5;

  /////////////////////////////////

  glm::vec3 m_boundingBoxMin;

  float m_w_sum_in_g = 0;  // in g
  float m_n_sum = 0;       // in AU
  float m_max_speed_diff = 0.f;
  float m_max_speed_grav = 0.f;
  std::mt19937 m_rnd;
  float m_irrigationAmount = 1;

  std::vector<Source> m_water_sources;
  std::vector<Source> m_nutrient_sources;

  // helper variables:
  std::vector<glm::ivec3> m_blur_3x3_idx;
  std::vector<float> m_blur_3x3_weights;

  glm::ivec2 m_materialTextureResolution = {128, 128};
  std::vector<SoilLayer> m_soilLayers;
  SoilSurface m_soilSurface;
};

class SoilParameters {
 public:
  glm::ivec3 m_voxelResolution = glm::ivec3(64, 64, 64);
  float m_deltaX = 0.1f;
  float m_deltaTime = 0.001f;  // delta t, time between steps
  glm::vec3 m_boundingBoxMin = glm::vec3(-3.2, -4.8, -3.2);

  VoxelSoilModel::Boundary m_boundary_x = VoxelSoilModel::Boundary::absorb;
  VoxelSoilModel::Boundary m_boundary_y = VoxelSoilModel::Boundary::absorb;
  VoxelSoilModel::Boundary m_boundary_z = VoxelSoilModel::Boundary::absorb;

  float m_diffusionForce = 1;
  glm::vec3 m_gravityForce = glm::vec3(0, -1.0, 0);
  float m_nutrientForce = 0.5;
};
}  // namespace eco_sys_lab
