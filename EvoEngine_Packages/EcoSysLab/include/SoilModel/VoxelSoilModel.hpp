#pragma once

#include <functional>
#include <glm/glm.hpp>
#include <map>
#include <random>
#include <valarray>
#include <vector>

namespace eco_sys_lab_package {
using namespace evo_engine;

/**
 * @brief Represents a field of floating-point values using std::valarray.
 */
using Field = std::valarray<float>;

class SoilParameters;

/**
 * @brief Represents the surface of the soil by defining a height function.
 */
struct SoilSurface {
  /**
   * @brief A function that returns the height of the soil surface at a given position.
   */
  std::function<float(const glm::vec2& position)> m_height;
};

/**
 * @brief Holds various texture maps that define the soil material visual properties.
 */
struct SoilMaterialTexture {
  std::vector<float> m_metallic_map;    ///< Metallic texture map.
  std::vector<float> m_roughness_map;   ///< Roughness texture map.
  std::vector<glm::vec3> m_normal_map;  ///< Normal texture map.
  std::vector<glm::vec4> m_color_map;   ///< Color texture map.
  std::vector<float> m_height_map;      ///< Height texture map.
};

/**
 * @brief Represents the physical properties of a specific soil material.
 */
struct SoilPhysicalMaterial {
  int m_id = -1;  ///< Unique identifier for the soil material.

  std::function<float(const glm::vec3& position)> m_c;  ///< Capacity function.
  std::function<float(const glm::vec3& position)> m_p;  ///< Permeability function.
  std::function<float(const glm::vec3& position)> m_d;  ///< Density function.

  std::function<float(const glm::vec3& position)> m_n;  ///< Initial nutrient amount function.
  std::function<float(const glm::vec3& position)> m_w;  ///< Initial water amount function.

  std::shared_ptr<SoilMaterialTexture> m_soilMaterialTexture;  ///< Pointer to soil material texture.
};

/**
 * @brief Represents a single soil layer with its material and thickness.
 */
struct SoilLayer {
  SoilPhysicalMaterial m_mat;  ///< The physical material of the layer.

  /**
   * @brief A function that defines the thickness of the soil layer at a given position.
   */
  std::function<float(const glm::vec2& position)> m_thickness;
};

/**
 * @brief Represents a voxel-based soil model with physical properties and simulation capabilities.
 */
class VoxelSoilModel {
  friend class Soil;
  friend class EcoSysLabLayer;

 public:
  /**
   * @brief Defines the boundary conditions of the soil model.
   */
  enum class Boundary : int { sink, block, wrap, absorb };

  /**
   * @brief Represents an external source injecting water or nutrients into the soil.
   */
  class Source {
   public:
    std::vector<int> idx;        ///< Indices of affected voxels.
    std::vector<float> amounts;  ///< Amounts to be applied to each voxel.

    /**
     * @brief Applies the source's effect to the given field.
     * @param target The field affected by the source.
     */
    void Apply(Field& target);
  };

  /**
   * @brief Initializes the soil model with parameters, a surface definition, and soil layers.
   * @param p Parameters defining the soil model.
   * @param soilSurface The surface structure of the soil.
   * @param soilLayers The layered physical structure of the soil.
   */
  void Initialize(const SoilParameters& p, const SoilSurface& soilSurface, const std::vector<SoilLayer>& soilLayers);

  /**
   * @brief Resets the soil model to its initial state.
   */
  void Reset();

  /**
   * @brief Simulates the soil for a given amount of time in hours.
   * @param t_in_hrs The amount of simulation time in hours.
   */
  void Run(float t_in_hrs);

  /**
   * @brief Performs a single simulation step.
   */
  void Step();

  /**
   * @brief Adds water to the soil volume, simulating irrigation.
   */
  void Irrigation();

  /**
   * @brief Integrates the water amount in a specified area.
   * @param position The center of the area.
   * @param width The width of the area.
   * @return The total water amount in grams.
   */
  [[nodiscard]] float IntegrateWater(const glm::vec3& position, float width) const;

  /**
   * @brief Retrieves the water density at a given position.
   * @param position The position to check.
   * @return The water density in g/cm .
   */
  [[nodiscard]] float GetWaterDensity(const glm::vec3& position) const;

  /**
   * @brief Integrates the nutrient amount in a specified area.
   * @param position The center of the area.
   * @param width The width of the area.
   * @return The total nutrient amount.
   */
  [[nodiscard]] float IntegrateNutrient(const glm::vec3& position, float width) const;

  /**
   * @brief Retrieves the nutrient density at a given position.
   * @param position The position to check.
   * @return The nutrient density.
   */
  [[nodiscard]] float GetNutrientDensity(const glm::vec3& position) const;

  /**
   * @brief Retrieves the soil density at a given position.
   * @param position The position to check.
   * @return The soil density.
   */
  [[nodiscard]] float GetDensity(const glm::vec3& position) const;

  /**
   * @brief Retrieves the soil capacity at a given position.
   * @param position The position to check.
   * @return The soil capacity.
   */
  [[nodiscard]] float GetCapacity(const glm::vec3& position) const;

  /**
   * @brief Adjusts the water content in the soil around a center point.
   * @param center The center of the affected area.
   * @param amount_in_g The amount of water in grams.
   * @param width The width of the affected area.
   */
  void ChangeWater(const glm::vec3& center, float amount_in_g, float width);

  /**
   * @brief Adjusts the nutrient content in the soil around a center point.
   * @param center The center of the affected area.
   * @param amount_in_AU The amount of nutrients.
   * @param width The width of the affected area.
   */
  void ChangeNutrient(const glm::vec3& center, float amount_in_AU, float width);

  /**
   * @brief Adjusts the density of the soil around a center point.
   * @param center The center of the affected area.
   * @param amount The amount of density change.
   * @param width The width of the affected area.
   */
  void ChangeDensity(const glm::vec3& center, float amount, float width);

  /**
   * @brief Adjusts the capacity of the soil around a center point.
   * @param center The center of the affected area.
   * @param amount The amount of capacity change.
   * @param width The width of the affected area.
   */
  void ChangeCapacity(const glm::vec3& center, float amount, float width);

  /**
   * @brief Computes the 1D index from a 3D coordinate and resolution.
   * @param resolution The voxel resolution in each dimension.
   * @param x The x-coordinate.
   * @param y The y-coordinate.
   * @param z The z-coordinate.
   * @return The computed 1D index.
   */
  [[nodiscard]] static int Index(const glm::ivec3& resolution, int x, int y, int z);

  /**
   * @brief Computes the 1D index from a 3D coordinate.
   * @param x The x-coordinate.
   * @param y The y-coordinate.
   * @param z The z-coordinate.
   * @return The computed 1D index.
   */
  [[nodiscard]] int Index(int x, int y, int z) const;

  /**
   * @brief Computes the 1D index from a 3D coordinate given a resolution.
   * @param resolution The voxel resolution in each dimension.
   * @param coordinate The 3D coordinate.
   * @return The computed 1D index.
   */
  [[nodiscard]] static int Index(const glm::ivec3& resolution, const glm::ivec3& coordinate);

  /**
   * @brief Computes the 1D index from a 3D coordinate.
   * @param coordinate The 3D coordinate.
   * @return The computed 1D index.
   */
  [[nodiscard]] int Index(const glm::ivec3& coordinate) const;

  /**
   * @brief Retrieves the 3D coordinate from a 1D index.
   * @param index The 1D index.
   * @return The computed 3D coordinate.
   */
  [[nodiscard]] glm::ivec3 GetCoordinateFromIndex(int index) const;

  /**
   * @brief Retrieves the 3D coordinate corresponding to a given world position.
   * @param position The world position.
   * @return The computed 3D coordinate.
   */
  [[nodiscard]] glm::ivec3 GetCoordinateFromPosition(const glm::vec3& position) const;

  /**
   * @brief Computes the world position from a 3D coordinate with specific offsets.
   * @param coordinate The 3D coordinate.
   * @param dx Offset in the x-direction.
   * @param dy Offset in the y-direction.
   * @param dz Offset in the z-direction.
   * @return The computed world position.
   */
  [[nodiscard]] glm::vec3 GetPositionFromCoordinate(const glm::ivec3& coordinate, float dx, float dy, float dz) const;

  /**
   * @brief Computes the world position from a 3D coordinate using default voxel size.
   * @param coordinate The 3D coordinate.
   * @return The computed world position.
   */
  [[nodiscard]] glm::vec3 GetPositionFromCoordinate(const glm::ivec3& coordinate) const;

  /**
   * @brief Retrieves the resolution of the voxel grid.
   * @return The voxel grid resolution.
   */
  [[nodiscard]] glm::ivec3 GetVoxelResolution() const;

  /**
   * @brief Retrieves the size of a single voxel.
   * @return The voxel size.
   */
  [[nodiscard]] float GetVoxelSize() const;

  /**
   * @brief Retrieves the current simulation time.
   * @return The current simulation time in hours.
   */
  [[nodiscard]] float GetTime() const;

  /**
   * @brief Retrieves the center of the bounding box.
   * @return The bounding box center.
   */
  [[nodiscard]] glm::vec3 GetBoundingBoxCenter() const;

  /**
   * @brief Retrieves the minimum coordinate of the bounding box.
   * @return The bounding box minimum coordinate.
   */
  [[nodiscard]] glm::vec3 GetBoundingBoxMin() const;

  /**
   * @brief Retrieves the maximum coordinate of the bounding box.
   * @return The bounding box maximum coordinate.
   */
  [[nodiscard]] glm::vec3 GetBoundingBoxMax() const;

  /**
   * @brief Checks if a given position is inside the soil volume.
   * @param position The position to check.
   * @return True if inside, otherwise false.
   */
  [[nodiscard]] bool PositionInsideVolume(const glm::vec3& position) const;

  /**
   * @brief Checks if a given coordinate is inside the soil volume.
   * @param coordinate The coordinate to check.
   * @return True if inside, otherwise false.
   */
  [[nodiscard]] bool CoordinateInsideVolume(const glm::ivec3& coordinate) const;

  /**
   * @brief Checks whether the soil model has been initialized.
   * @return True if initialized, otherwise false.
   */
  [[nodiscard]] bool Initialized() const;

  /**
   * @brief Generates a texture slice of the soil model along the Z-axis.
   * @param backFacing Whether the slice is viewed from the back.
   * @param z The position along the Z-axis.
   * @param xyMin Minimum XY bounds of the slice.
   * @param xyMax Maximum XY bounds of the slice.
   * @param albedoData Output vector for albedo (color) data.
   * @param normalData Output vector for normal data.
   * @param roughnessData Output vector for roughness data.
   * @param metallicData Output vector for metallic data.
   * @param outputResolution Output resolution of the generated texture.
   * @param waterFactor Scaling factor for water influence.
   * @param nutrientFactor Scaling factor for nutrient influence.
   * @param blur_width The width of the blur applied to the texture.
   */
  void GetSoilTextureSlideZ(bool backFacing, float z, const glm::vec2& xyMin, const glm::vec2& xyMax,
                            std::vector<glm::vec4>& albedoData, std::vector<glm::vec3>& normalData,
                            std::vector<float>& roughnessData, std::vector<float>& metallicData,
                            glm::ivec2& outputResolution, float waterFactor, float nutrientFactor,
                            float blur_width = 1);

  /**
   * @brief Generates a texture slice of the soil model along the X-axis.
   * @param backFacing Whether the slice is viewed from the back.
   * @param x The position along the X-axis.
   * @param yzMin Minimum YZ bounds of the slice.
   * @param yzMax Maximum YZ bounds of the slice.
   * @param albedoData Output vector for albedo (color) data.
   * @param normalData Output vector for normal data.
   * @param roughnessData Output vector for roughness data.
   * @param metallicData Output vector for metallic data.
   * @param outputResolution Output resolution of the generated texture.
   * @param waterFactor Scaling factor for water influence.
   * @param nutrientFactor Scaling factor for nutrient influence.
   * @param blur_width The width of the blur applied to the texture.
   */
  void GetSoilTextureSlideX(bool backFacing, float x, const glm::vec2& yzMin, const glm::vec2& yzMax,
                            std::vector<glm::vec4>& albedoData, std::vector<glm::vec3>& normalData,
                            std::vector<float>& roughnessData, std::vector<float>& metallicData,
                            glm::ivec2& outputResolution, float waterFactor, float nutrientFactor,
                            float blur_width = 1);

  /**
   * @brief Retrieves the soil texture parameters for a specific position.
   * @param position The position to retrieve data for.
   * @param texture_idx The texture index to sample.
   * @param blur_width The width of the blur applied to the texture.
   * @param albedo Output variable for the albedo (color) value.
   * @param normal Output variable for the normal vector.
   * @param roughness Output variable for the roughness value.
   * @param metallic Output variable for the metallic value.
   * @param waterFactor Scaling factor for water influence.
   * @param nutrientFactor Scaling factor for nutrient influence.
   */
  void GetSoilTextureColorForPosition(const glm::vec3& position, int texture_idx, float blur_width, glm::vec4& albedo,
                                      glm::vec3& normal, float& roughness, float& metallic, float waterFactor,
                                      float nutrientFactor);

  int m_version = 0;  ///< @brief Versioning for potential future changes.

 protected:
  /**
   * @brief Builds the internal voxel representation from the provided soil layers.
   */
  void BuildFromLayers();

  /**
   * @brief Sets the voxel data at a specific coordinate with the given soil material.
   * @param coordinate The 3D coordinate of the voxel.
   * @param material The soil material to apply.
   */
  void SetVoxel(const glm::ivec3& coordinate, const SoilPhysicalMaterial& material);

  /**
   * @brief Retrieves a field value at a given position with a default fallback.
   * @param field The field to sample from.
   * @param position The position to sample.
   * @param default_value The default value if the position is outside the field.
   * @return The sampled field value.
   */
  [[nodiscard]] float GetField(const Field& field, const glm::vec3& position, float default_value) const;

  /**
   * @brief Modifies a field's values around a central point with a given distribution width.
   * @param field The target field.
   * @param center The center of the change.
   * @param amount_in_cm3 The amount by which the field is altered, measured in cm .
   * @param width_in_m The width of the change distribution in meters.
   */
  void ChangeField(Field& field, const glm::vec3& center, float amount_in_cm3, float width_in_m);

  /**
   * @brief Computes the total amount of a field's values in a given area.
   * @param field The field to integrate.
   * @param center The center of the integration area.
   * @param width The width of the integration area.
   * @return The computed integral value.
   */
  [[nodiscard]] float IntegrateFieldValue(const Field& field, const glm::vec3& center, float width) const;

  /**
   * @brief Sets a specific value within a bounding box in the given field.
   * @param field The field to modify.
   * @param bb_min The minimum bounding box coordinate.
   * @param bb_max The maximum bounding box coordinate.
   * @param value The value to be set.
   */
  void SetField(Field& field, const glm::vec3& bb_min, const glm::vec3& bb_max, float value);

  /**
   * @brief Applies a blurring function to a field.
   * @param field The field to blur.
   */
  void BlurField(Field& field);

  /**
   * @brief Adds a new water source to the model.
   * @param source The water source to add.
   */
  void AddWaterSource(Source&& source);

  /**
   * @brief Adds a new nutrient source to the model.
   * @param source The nutrient source to add.
   */
  void AddNutrientSource(Source&& source);

  /**
   * @brief Performs a 3D convolution operation on a field using specific indices and weights.
   * @param input The input field.
   * @param output The output field.
   * @param indices The indices defining the convolution area.
   * @param weights The convolution kernel weights.
   */
  void Convolution3(const Field& input, Field& output, const std::vector<int>& indices,
                    const std::vector<float>& weights) const;

  /**
   * @brief Handles boundary conditions for an axis by wrapping values.
   * @param input The input field.
   * @param output The output field.
   * @param indices_1D The 1D indices for processing.
   * @param weights The convolution weights.
   * @param lim_a Lower boundary limit.
   * @param lim_b Upper boundary limit.
   * @param lim_f Final boundary.
   * @param WrapIndex Function for handling index wrapping.
   */
  void Boundary_Wrap_Axis(const Field& input, Field& output, const std::vector<int>& indices_1D,
                          const std::vector<float>& weights, int lim_a, int lim_b, int lim_f,
                          std::function<int(int, int, int)> WrapIndex) const;

  /**
   * @brief Applies wrap boundary conditions along the X-axis.
   */
  void Boundary_Wrap_X(const Field& input, Field& output, const std::vector<int>& indices_1D,
                       const std::vector<float>& weights) const;

  /**
   * @brief Applies wrap boundary conditions along the Y-axis.
   */
  void Boundary_Wrap_Y(const Field& input, Field& output, const std::vector<int>& indices_1D,
                       const std::vector<float>& weights) const;

  /**
   * @brief Applies wrap boundary conditions along the Z-axis.
   */
  void Boundary_Wrap_Z(const Field& input, Field& output, const std::vector<int>& indices_1D,
                       const std::vector<float>& weights) const;

  /**
   * @brief Handles barrier boundary conditions for an axis.
   * @param input The input field.
   * @param output The output field.
   * @param indices_1D The 1D indices for processing.
   * @param weights The convolution weights.
   * @param lim_a Lower boundary limit.
   * @param lim_b Upper boundary limit.
   * @param lim_f Final boundary.
   * @param WrapIndex Function for handling index wrapping.
   */
  void Boundary_Barrier_Axis(const Field& input, Field& output, const std::vector<int>& indices_1D,
                             const std::vector<float>& weights, int lim_a, int lim_b, int lim_f,
                             std::function<int(int, int, int)> WrapIndex) const;

  /**
   * @brief Applies barrier boundary conditions along the X-axis.
   */
  void Boundary_Barrier_X(const Field& input, Field& output, const std::vector<int>& indices_1D,
                          const std::vector<float>& weights) const;

  /**
   * @brief Applies barrier boundary conditions along the Y-axis.
   */
  void Boundary_Barrier_Y(const Field& input, Field& output, const std::vector<int>& indices_1D,
                          const std::vector<float>& weights) const;

  /**
   * @brief Applies barrier boundary conditions along the Z-axis.
   */
  void Boundary_Barrier_Z(const Field& input, Field& output, const std::vector<int>& indices_1D,
                          const std::vector<float>& weights) const;

  /**
   * @brief Updates internal statistics related to soil properties, such as water and speed.
   */
  void UpdateStats();

  /**
   * @brief Initializes an empty test soil model with the given resolution.
   * @param resolution The voxel grid resolution.
   */
  void Test_InitializeEmpty(glm::uvec3 resolution);

  /**
   * @brief Tests water density calculations within the soil model.
   */
  void Test_WaterDensity();

  /**
   * @brief Tests the effect of permeability on simulation speed.
   */
  void Test_PermeabilitySpeed();

  /**
   * @brief Tests nutrient transport behavior in the soil model.
   * @param p Permeability factor.
   * @param c Capacity factor.
   * @param texture The soil material texture used for testing.
   */
  void Test_NutrientTransport(float p, float c, const std::shared_ptr<SoilMaterialTexture>& texture);

  /**
   * @brief Simulates nutrient transport behavior in sandy soil conditions.
   * @param texture The soil material texture used for testing sandy soil.
   */
  void Test_NutrientTransport_Sand(const std::shared_ptr<SoilMaterialTexture>& texture);

  /**
   * @brief Simulates nutrient transport behavior in loamy soil conditions.
   * @param texture The soil material texture used for testing loamy soil.
   */
  void Test_NutrientTransport_Loam(const std::shared_ptr<SoilMaterialTexture>& texture);

  /**
   * @brief Simulates nutrient transport behavior in silty soil conditions.
   * @param texture The soil material texture used for testing silty soil.
   */
  void Test_NutrientTransport_Silt(const std::shared_ptr<SoilMaterialTexture>& texture);

  bool m_initialized = false;  ///< Indicates whether the soil model has been initialized.

  glm::ivec3 m_resolution;                   ///< The resolution of the voxel grid.
  float m_dx;                                ///< Distance between two voxels.
  float m_voxel_volume_in_cm3;               ///< Volume of a single voxel in cm .
  float m_water_g_per_cm3;                   ///< Water content per cm  at density 1.
  float m_nutrient_unit_per_cm3;             ///< Nutrient content per cm  at density 1.
  float m_dt;                                ///< Time step in hours.
  float m_time_since_start_in_hrs = 0.0f;    ///< Simulation time since start.
  float m_time_since_start_requested = 0.f;  ///< Target time to simulate up to.

  float m_diffusionForce;    ///< Scaling factor for diffusion forces.
  glm::vec3 m_gravityForce;  ///< Direction and magnitude of gravity force.
  float m_nutrientForce;     ///< Scaling factor for nutrient movement.

  std::valarray<int> m_material_id;  ///< Material identifiers for each voxel.

  Field m_w;  ///< Water density field (g/cm ).
  Field m_c;  ///< Capacity field.
  Field m_l;  ///< Filling level field (water/capacity).
  Field m_p;  ///< Permeability field.

  Field m_w_grad_x;  ///< Water gradient x-component.
  Field m_w_grad_y;  ///< Water gradient y-component.
  Field m_w_grad_z;  ///< Water gradient z-component.

  Field m_div_diff_x;  ///< Divergence component for diffusion in x.
  Field m_div_diff_y;  ///< Divergence component for diffusion in y.
  Field m_div_diff_z;  ///< Divergence component for diffusion in z.

  Field m_div_diff_n_x;  ///< Divergence component for nutrient diffusion in x.
  Field m_div_diff_n_y;  ///< Divergence component for nutrient diffusion in y.
  Field m_div_diff_n_z;  ///< Divergence component for nutrient diffusion in z.

  Field m_div_grav_x;  ///< Divergence component for gravity in x.
  Field m_div_grav_y;  ///< Divergence component for gravity in y.
  Field m_div_grav_z;  ///< Divergence component for gravity in z.

  Field m_div_grav_n_x;  ///< Divergence component for nutrient gravity in x.
  Field m_div_grav_n_y;  ///< Divergence component for nutrient gravity in y.
  Field m_div_grav_n_z;  ///< Divergence component for nutrient gravity in z.

  Field m_n;                                          ///< Nutrient field.
  Field m_d;                                          ///< Soil density field.
  Boundary m_boundary_x, m_boundary_y, m_boundary_z;  ///< Boundary conditions for each axis.
  int m_absorption_width = 5;                         ///< Width of absorption boundary.

  glm::vec3 m_boundingBoxMin;  ///< Minimum bounding box coordinate.

  float m_w_sum_in_g = 0;        ///< Total water sum in grams.
  float m_n_sum = 0;             ///< Total nutrient sum in arbitrary units.
  float m_max_speed_diff = 0.f;  ///< Maximum speed from diffusion forces.
  float m_max_speed_grav = 0.f;  ///< Maximum speed from gravity forces.
  std::mt19937 m_rnd;            ///< Random number generator.
  float m_irrigationAmount = 1;  ///< Default irrigation water amount.

  std::vector<Source> m_water_sources;     ///< List of water sources.
  std::vector<Source> m_nutrient_sources;  ///< List of nutrient sources.

  std::vector<glm::ivec3> m_blur_3x3_idx;  ///< Indices for 3x3 blurring.
  std::vector<float> m_blur_3x3_weights;   ///< Weights for 3x3 blurring.

  glm::ivec2 m_materialTextureResolution = {128, 128};  ///< Resolution of material textures.
  std::vector<SoilLayer> m_soilLayers;                  ///< List of soil layers.
  SoilSurface m_soilSurface;                            ///< Surface properties of the soil.
};

/**
 * @brief Represents the parameters used to initialize the VoxelSoilModel.
 */
class SoilParameters {
 public:
  glm::ivec3 m_voxelResolution = glm::ivec3(64, 64, 64);     ///< The resolution of the voxel grid.
  float m_deltaX = 0.1f;                                     ///< The voxel size in meters.
  float m_deltaTime = 0.001f;                                ///< Time step duration in hours.
  glm::vec3 m_boundingBoxMin = glm::vec3(-3.2, -4.8, -3.2);  ///< Minimum coordinates of the bounding box.

  VoxelSoilModel::Boundary m_boundary_x = VoxelSoilModel::Boundary::absorb;  ///< Boundary condition along the X-axis.
  VoxelSoilModel::Boundary m_boundary_y = VoxelSoilModel::Boundary::absorb;  ///< Boundary condition along the Y-axis.
  VoxelSoilModel::Boundary m_boundary_z = VoxelSoilModel::Boundary::absorb;  ///< Boundary condition along the Z-axis.

  float m_diffusionForce = 1;                        ///< Scaling factor for diffusion.
  glm::vec3 m_gravityForce = glm::vec3(0, -1.0, 0);  ///< Gravity force acting on the soil.
  float m_nutrientForce = 0.5;                       ///< Scaling factor for nutrient transport.
};
}  // namespace eco_sys_lab_package