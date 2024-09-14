#pragma once
#include "Animator.hpp"
#include "GeometryStorage.hpp"
#include "GraphicsResources.hpp"
#include "Scene.hpp"
#include "Vertex.hpp"
namespace evo_engine {
struct SkinnedVertexAttributes {
  bool normal = false;
  bool tangent = false;
  bool tex_coord = false;
  bool color = false;

  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
};

class BoneMatrices {
  size_t version_ = 0;
  std::vector<std::unique_ptr<Buffer>> bone_matrices_buffer_ = {};
  std::vector<std::shared_ptr<DescriptorSet>> descriptor_set_;
  friend class RenderLayer;
  void UploadData();

 public:
  [[nodiscard]] const std::shared_ptr<DescriptorSet>& GetDescriptorSet() const;
  BoneMatrices();
  [[nodiscard]] size_t& GetVersion();

  std::vector<glm::mat4> value;
};
class SkinnedMesh : public IAsset, public IGeometry {
  Bound bound_;
  friend class SkinnedMeshRenderer;
  friend class Particles;
  friend class Platform;
  friend class RenderLayer;
  friend class RenderInstances;

  SkinnedVertexAttributes skinned_vertex_attributes_ = {};
  std::vector<SkinnedVertex> skinned_vertices_;
  std::vector<glm::uvec3> skinned_triangles_;
  std::shared_ptr<RangeDescriptor> skinned_triangle_range_;
  std::shared_ptr<RangeDescriptor> skinned_meshlet_range_;

  friend struct SkinnedMeshBonesBlock;
  // Don't serialize.

  friend class Prefab;

 protected:
  bool SaveInternal(const std::filesystem::path& path) const override;

 public:
  ~SkinnedMesh() override;
  void DrawIndexed(VkCommandBuffer vk_command_buffer, GraphicsPipelineStates& global_pipeline_state,
                   int instances_count) const override;
  void OnCreate() override;
  void FetchIndices(const std::vector<std::shared_ptr<Bone>>& bones);
  std::vector<unsigned> bone_animator_indices;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  [[nodiscard]] glm::vec3 GetCenter() const;
  [[nodiscard]] Bound GetBound() const;
  void SetVertices(const SkinnedVertexAttributes& skinned_vertex_attributes,
                   const std::vector<SkinnedVertex>& skinned_vertices, const std::vector<unsigned>& indices);
  void SetVertices(const SkinnedVertexAttributes& skinned_vertex_attributes,
                   const std::vector<SkinnedVertex>& skinned_vertices, const std::vector<glm::uvec3>& triangles);
  [[nodiscard]] size_t GetSkinnedVerticesAmount() const;
  [[nodiscard]] size_t GetTriangleAmount() const;
  void RecalculateNormal();
  void RecalculateTangent();
  [[nodiscard]] std::vector<SkinnedVertex>& UnsafeGetSkinnedVertices();
  [[nodiscard]] std::vector<glm::uvec3>& UnsafeGetTriangles();

  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};
}  // namespace evo_engine