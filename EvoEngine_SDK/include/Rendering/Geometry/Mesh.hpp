#pragma once
#include "Bound.hpp"
#include "GeometryStorage.hpp"
#include "GraphicsResources.hpp"
#include "IAsset.hpp"
#include "IGeometry.hpp"
#include "Platform.hpp"
#include "Vertex.hpp"
namespace evo_engine {
struct VertexAttributes {
  bool normal = false;
  bool tangent = false;
  bool tex_coord = false;
  bool color = false;

  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
};

class ParticleInfoList final : public IAsset {
  std::shared_ptr<RangeDescriptor> range_descriptor_;

 public:
  void OnCreate() override;
  ~ParticleInfoList() override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void ApplyRays(const std::vector<Ray>& rays, const glm::vec4& color, float ray_width) const;
  void ApplyRays(const std::vector<Ray>& rays, const std::vector<glm::vec4>& colors, float ray_width) const;
  void ApplyConnections(const std::vector<glm::vec3>& starts, const std::vector<glm::vec3>& ends,
                        const glm::vec4& color, float ray_width) const;
  void ApplyConnections(const std::vector<glm::vec3>& starts, const std::vector<glm::vec3>& ends,
                        const std::vector<glm::vec4>& colors, float ray_width) const;
  void ApplyConnections(const std::vector<glm::vec3>& starts, const std::vector<glm::vec3>& ends,
                        const std::vector<glm::vec4>& colors, const std::vector<float>& ray_widths) const;
  void SetParticleInfos(const std::vector<ParticleInfo>& particle_infos) const;
  const std::vector<ParticleInfo>& PeekParticleInfoList() const;
  [[nodiscard]] const std::shared_ptr<DescriptorSet>& GetDescriptorSet() const;
};

class Mesh final : public IAsset, public IGeometry {
  Bound bound_ = {};

  std::vector<Vertex> vertices_;
  std::vector<glm::uvec3> triangles_;

  VertexAttributes vertex_attributes_ = {};
  friend class RenderLayer;
  friend class RenderInstances;
  friend class TopLevelAccelerationStructure;
  std::shared_ptr<RangeDescriptor> triangle_range_;
  std::shared_ptr<RangeDescriptor> meshlet_range_;

  std::shared_ptr<BottomLevelAccelerationStructure> blas_;

 protected:
  bool SaveInternal(const std::filesystem::path& path) const override;

 public:
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void OnCreate() override;
  ~Mesh() override;
  void DrawIndexed(VkCommandBuffer vk_command_buffer, GraphicsPipelineStates& global_pipeline_state,
                   int instances_count) const override;

  void SetVertices(const VertexAttributes& vertex_attributes, const std::vector<Vertex>& vertices,
                   const std::vector<unsigned>& indices);
  void SetVertices(const VertexAttributes& vertex_attributes, const std::vector<Vertex>& vertices,
                   const std::vector<glm::uvec3>& triangles);

  void MergeVertices();
  [[nodiscard]] size_t GetVerticesAmount() const;
  [[nodiscard]] size_t GetTriangleAmount() const;

  void RecalculateNormal();
  void RecalculateTangent();
  [[nodiscard]] const std::shared_ptr<RangeDescriptor>& GetTriangleRange() const;
  [[nodiscard]] float CalculateTriangleArea(const glm::uvec3& triangle) const;
  [[nodiscard]] glm::vec3 CalculateCentroid(const glm::uvec3& triangle) const;
  [[nodiscard]] std::vector<Vertex>& UnsafeGetVertices();
  [[nodiscard]] std::vector<glm::uvec3>& UnsafeGetTriangles();
  [[nodiscard]] Bound GetBound() const;
  [[nodiscard]] std::shared_ptr<BottomLevelAccelerationStructure> GetBlas() const;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};
}  // namespace evo_engine