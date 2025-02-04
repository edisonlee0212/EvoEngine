
#pragma once
#include <Utilities.hpp>
#include "GeometryStorage.hpp"
#include "IAsset.hpp"
#include "Particles.hpp"
#include "Vertex.hpp"

namespace evo_engine {

/**
 * @brief Struct representing the attributes of a strand point.
 */
struct StrandPointAttributes {
  bool normal = false;     ///< Indicates if normal is enabled.
  bool tex_coord = false;  ///< Indicates if texture coordinates are enabled.
  bool color = false;      ///< Indicates if color is enabled.

  /**
   * @brief Serializes the StrandPointAttributes to the given YAML emitter.
   * @param out The YAML emitter to serialize to.
   */
  void Serialize(YAML::Emitter& out) const;

  /**
   * @brief Deserializes the StrandPointAttributes from the given YAML node.
   * @param in The YAML node to deserialize from.
   */
  void Deserialize(const YAML::Node& in);
};

/**
 * @brief Represents a collection of hair strands.
 */
class Strands final : public IAsset, public IGeometry {
 public:
  /**
   * @brief Provides unsafe access to the segments vector.
   * @return Reference to the vector of segments.
   * @note This function is not thread-safe.
   */
  [[nodiscard]] std::vector<glm::uint>& UnsafeGetSegments();

  /**
   * @brief Provides unsafe access to the strand points vector.
   * @return Reference to the vector of strand points.
   * @note This function is not thread-safe.
   */
  [[nodiscard]] std::vector<StrandPoint>& UnsafeGetStrandPoints();

  /**
   * @brief Inspects the object in the editor.
   * @param editor_layer Shared pointer to the editor layer.
   * @return True if the inspection was successful, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Serializes the Strands object to the given YAML emitter.
   * @param out The YAML emitter to serialize to.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the Strands object from the given YAML node.
   * @param in The YAML node to deserialize from.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Sets the segments with the given attributes, segment indices, and strand points.
   * @param strand_point_attributes Attributes of the strand points.
   * @param segments Indices of the segments.
   * @param points Strand points.
   */
  void SetSegments(const StrandPointAttributes& strand_point_attributes, const std::vector<glm::uint>& segments,
                   const std::vector<StrandPoint>& points);

  /**
   * @brief Sets the strands with the given attributes, strand indices, and strand points.
   * @param strand_point_attributes Attributes of the strand points.
   * @param strands Indices of the strands.
   * @param points Strand points.
   */
  void SetStrands(const StrandPointAttributes& strand_point_attributes, const std::vector<glm::uint>& strands,
                  const std::vector<StrandPoint>& points);

  /**
   * @brief Recalculates the normals for the strands.
   */
  void RecalculateNormal();

  /**
   * @brief Draws the indexed strands on the given Vulkan command buffer.
   * @param vk_command_buffer The Vulkan command buffer.
   * @param global_pipeline_state Global graphics pipeline states.
   * @param instances_count Number of instances to draw.
   */
  void DrawIndexed(VkCommandBuffer vk_command_buffer, GraphicsPipelineStates& global_pipeline_state,
                   int instances_count) const override;

  /**
   * @brief Invoked upon creation of the strands object.
   */
  void OnCreate() override;

  /**
   * @brief Gets the bounding box of the strands.
   * @return Bounding box of the strands.
   */
  [[nodiscard]] Bound GetBound() const;

  /**
   * @brief Gets the number of segments in the strands.
   * @return Number of segments.
   */
  [[nodiscard]] size_t GetSegmentAmount() const;

  /**
   * @brief Destructor for the Strands object.
   */
  ~Strands() override;

  /**
   * @brief Gets the number of strand points in the strands.
   * @return Number of strand points.
   */
  [[nodiscard]] size_t GetStrandPointAmount() const;

  /**
   * @brief Performs cubic interpolation with tangent calculation.
   * @tparam T Type of the points.
   * @param v0 First control point.
   * @param v1 Second control point.
   * @param v2 Third control point.
   * @param v3 Fourth control point.
   * @param result Interpolated result.
   * @param tangent Tangent at the interpolated point.
   * @param u Interpolation parameter.
   */
  template <class T>
  static void CubicInterpolation(const T& v0, const T& v1, const T& v2, const T& v3, T& result, T& tangent, float u);

  /**
   * @brief Performs cubic interpolation and returns the interpolated value.
   * @tparam T Type of the points.
   * @param v0 First control point.
   * @param v1 Second control point.
   * @param v2 Third control point.
   * @param v3 Fourth control point.
   * @param u Interpolation parameter.
   * @return Interpolated value.
   */
  template <class T>
  static T CubicInterpolation(const T& v0, const T& v1, const T& v2, const T& v3, float u);

  /**
   * @brief Calculate the segment length of the curve.
   * @tparam T Point type
   * @param v0 1st control point
   * @param v1 2nd control point
   * @param v2 3rd control point
   * @param v3 4th control point
   * @param t_start Start parameter t
   * @param t_end End parameter t
   * @param tolerance Precision
   * @return The length of the segment.
   */
  template <class T>
  static float CalculateLengthAdaptive(const T& v0, const T& v1, const T& v2, const T& v3, float t_start = 0.f,
                                       float t_end = 1.f, float tolerance = 0.001f);

  /**
   * @brief Find parameter t with given t_start and segment length.
   * @tparam T Point type
   * @param v0 1st control point
   * @param v1 2nd control point
   * @param v2 3rd control point
   * @param v3 4th control point
   * @param t_start Start parameter t
   * @param target_length Target length of the segment.
   * @param tolerance Precision
   * @return End parameter t.
   */
  template <class T>
  static float FindTAdaptive(const T& v0, const T& v1, const T& v2, const T& v3, float t_start, float target_length,
                             float tolerance = 0.001f);

 protected:
  /**
   * @brief Loads the strands object from a specified path.
   * @param path File path to load from.
   * @return True if loading was successful, false otherwise.
   */
  bool LoadInternal(const std::filesystem::path& path) override;

 private:
  std::shared_ptr<RangeDescriptor> segment_range_;         ///< Descriptor for segment range.
  std::shared_ptr<RangeDescriptor> strand_meshlet_range_;  ///< Descriptor for strand meshlet range.

  StrandPointAttributes strand_point_attributes_ = {};  ///< Attributes of the strand points.

  friend class StrandsRenderer;
  friend class RenderLayer;
  friend class RenderInstanceStorage;
  Bound bound_;  ///< Bounding information of the strands.

  /**
   * @brief Prepares the strands data based on the given attributes.
   * @param strand_point_attributes The strand point attributes.
   */
  void PrepareStrands(const StrandPointAttributes& strand_point_attributes);

  std::vector<glm::uint> segment_raw_indices_;  ///< Starting indices of the points where each segment starts.

  std::vector<glm::uvec4> segments_;        ///< Vector storing segment data.
  std::vector<StrandPoint> strand_points_;  ///< Vector storing strand points.
};

template <class T>
void Strands::CubicInterpolation(const T& v0, const T& v1, const T& v2, const T& v3, T& result, T& tangent, float u) {
  const T p0 = (v2 + v0) / 6.0f + v1 * (4.0f / 6.0f);
  const T p1 = v2 - v0;
  const T p2 = v2 - v1;
  const T p3 = v3 - v1;
  const float uu = u * u;
  const float u3 = 1.0f / 6.0f * uu * u;
  const auto q = glm::vec3(u3 + 0.5 * (u - uu), uu - 4.0 * u3, u3);
  result = p0 + q.x * p1 + q.y * p2 + q.z * p3;
  if (u == 0.0)
    u = 0.000001f;
  if (u == 1.0)
    u = 0.999999f;
  const float v = 1.0f - u;
  tangent = 0.5f * v * v * p1 + 2.0f * v * u * p2 + 0.5f * u * u * p3;
}

template <class T>
T Strands::CubicInterpolation(const T& v0, const T& v1, const T& v2, const T& v3, const float u) {
  const T p0 = (v2 + v0) / 6.0f + v1 * (4.0f / 6.0f);
  const T p1 = v2 - v0;
  const T p2 = v2 - v1;
  const T p3 = v3 - v1;
  const float uu = u * u;
  const float u3 = 1.0f / 6.0f * uu * u;
  const auto q = glm::vec3(u3 + 0.5 * (u - uu), uu - 4.0 * u3, u3);
  return p0 + q.x * p1 + q.y * p2 + q.z * p3;
}

template <class T>
float Strands::CalculateLengthAdaptive(const T& v0, const T& v1, const T& v2, const T& v3, float t_start, float t_end,
                                       const float tolerance) {
  const glm::vec3 mid_point = CubicInterpolation(v0, v1, v2, v3, (t_start + t_end) * 0.5f);
  const glm::vec3 start_point = CubicInterpolation(v0, v1, v2, v3, t_start);
  const glm::vec3 end_point = CubicInterpolation(v0, v1, v2, v3, t_end);

  const float linear_distance = glm::distance(start_point, end_point);
  const float curve_distance = glm::distance(start_point, mid_point) + glm::distance(mid_point, end_point);
  if (fabs(linear_distance - curve_distance) < tolerance) {
    return curve_distance;  // Close enough, return this estimate
  }
  // Subdivide further
  return CalculateLengthAdaptive(v0, v1, v2, v3, t_start, (t_start + t_end) * 0.5f, tolerance) +
         CalculateLengthAdaptive(v0, v1, v2, v3, (t_start + t_end) * 0.5f, t_end, tolerance);
}

template <class T>
float Strands::FindTAdaptive(const T& v0, const T& v1, const T& v2, const T& v3, const float t_start,
                             const float target_length, const float tolerance) {
  if (CalculateLengthAdaptive(v0, v1, v2, v3, t_start, 1.f, tolerance) <= target_length)
    return 1.f;
  float t_low = t_start, t_high = 1.0f;
  while (t_high - t_low > tolerance) {
    if (float t_mid = (t_low + t_high) * 0.5f;
        CalculateLengthAdaptive(v0, v1, v2, v3, t_start, t_mid) < target_length) {
      t_low = t_mid;
    } else {
      t_high = t_mid;
    }
  }
  return (t_low + t_high) * 0.5f;
}

}  // namespace evo_engine
