#pragma once
#include <Utilities.hpp>
#include "GeometryStorage.hpp"
#include "IAsset.hpp"
#include "Particles.hpp"
#include "Vertex.hpp"
namespace evo_engine {
struct StrandPointAttributes {
  bool normal = false;
  bool tex_coord = false;
  bool color = false;

  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
};
class Strands final : public IAsset, public IGeometry {
 public:
  [[nodiscard]] std::vector<glm::uint>& UnsafeGetSegments();
  [[nodiscard]] std::vector<StrandPoint>& UnsafeGetStrandPoints();
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  void Serialize(YAML::Emitter& out) const override;

  void Deserialize(const YAML::Node& in) override;

  void SetSegments(const StrandPointAttributes& strand_point_attributes, const std::vector<glm::uint>& segments,
                   const std::vector<StrandPoint>& points);

  void SetStrands(const StrandPointAttributes& strand_point_attributes, const std::vector<glm::uint>& strands,
                  const std::vector<StrandPoint>& points);
  void RecalculateNormal();
  void DrawIndexed(VkCommandBuffer vk_command_buffer, GraphicsPipelineStates& global_pipeline_state,
                   int instances_count) const override;
  void OnCreate() override;

  [[nodiscard]] Bound GetBound() const;

  [[nodiscard]] size_t GetSegmentAmount() const;
  ~Strands() override;
  [[nodiscard]] size_t GetStrandPointAmount() const;

  template <class T>
  static void CubicInterpolation(const T& v0, const T& v1, const T& v2, const T& v3, T& result, T& tangent, float u);
  template <class T>
  static T CubicInterpolation(const T& v0, const T& v1, const T& v2, const T& v3, float u);
  /**
   * \brief Calculate the segment length of the curve.
   * \tparam T Point type
   * \param v0 1st control point
   * \param v1 2nd control point
   * \param v2 3rd control point
   * \param v3 4th control point
   * \param t_start Start parameter t
   * \param t_end End parameter t
   * \param tolerance Precision
   * \return The length of the segment.
   */
  template <class T>
  static float CalculateLengthAdaptive(const T& v0, const T& v1, const T& v2, const T& v3, float t_start = 0.f,
                                       float t_end = 1.f, float tolerance = 0.001f);
  /**
   * \brief Find parameter t with given t_start and segment length.
   * \tparam T Point type
   * \param v0 1st control point
   * \param v1 2nd control point
   * \param v2 3rd control point
   * \param v3 4th control point
   * \param t_start Start parameter t
   * \param target_length Target length of the segment.
   * \param tolerance Precision
   * \return End parameter t.
   */
  template <class T>
  static float FindTAdaptive(const T& v0, const T& v1, const T& v2, const T& v3, float t_start, float target_length, float tolerance = 0.001f);
 protected:
  bool LoadInternal(const std::filesystem::path& path) override;

 private:
  std::shared_ptr<RangeDescriptor> segment_range_;
  std::shared_ptr<RangeDescriptor> strand_meshlet_range_;

  StrandPointAttributes strand_point_attributes_ = {};

  friend class StrandsRenderer;
  friend class RenderLayer;
  friend class RenderInstances;
  Bound bound_;

  void PrepareStrands(const StrandPointAttributes& strand_point_attributes);
  // The starting index of the point where this segment starts;
  std::vector<glm::uint> segment_raw_indices_;

  std::vector<glm::uvec4> segments_;
  std::vector<StrandPoint> strand_points_;
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

  if (const float curve_distance = glm::distance(start_point, mid_point) + glm::distance(mid_point, end_point);
      fabs(linear_distance - curve_distance) < tolerance) {
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
    if (float t_mid = (t_low + t_high) * 0.5f; CalculateLengthAdaptive(v0, v1, v2, v3, t_start, t_mid) < target_length) {
      t_low = t_mid;
    } else {
      t_high = t_mid;
    }
  }
  return (t_low + t_high) * 0.5f;
}
}  // namespace evo_engine
