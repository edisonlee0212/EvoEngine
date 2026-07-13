
/**
 * @file SkinnedMesh.hpp
 * @brief Declaration of classes and structures related to skinned meshes and bone matrices for rendering.
 */

#pragma once
#include "Animator.hpp"
#include "GeometryStorage.hpp"
#include "GraphicsResources.hpp"
#include "MorphTarget.hpp"
#include "Scene.hpp"
#include "Vertex.hpp"

namespace evo_engine {

/**
 * @struct SkinnedVertexAttributes
 * @brief Represents the attributes of a skinned vertex.
 */
struct SkinnedVertexAttributes {
  bool normal = false;      /**< Whether the normal attribute is enabled. */
  bool tangent = false;     /**< Whether the tangent attribute is enabled. */
  bool tex_coord = false;   /**< Whether the texture coordinate attribute is enabled. */
  bool tex_coord_1 = false; /**< Whether the secondary texture coordinate attribute is enabled. */
  bool tex_coord_2 = false; /**< Whether the third texture coordinate attribute is enabled. */
  bool tex_coord_3 = false; /**< Whether the fourth texture coordinate attribute is enabled. */
  bool color = false;       /**< Whether the color attribute is enabled. */

  /**
   * @brief Serializes the attributes to a YAML emitter.
   * @param out The YAML emitter to serialize to.
   */
  void Serialize(YAML::Emitter& out) const;

  /**
   * @brief Deserializes the attributes from a YAML node.
   * @param in The YAML node to deserialize from.
   */
  void Deserialize(const YAML::Node& in);
};

/**
 * @class BoneMatrices
 * @brief Manages bone matrices used for skeletal animations.
 */
class BoneMatrices {
  uint32_t version_ = 0;                                      /**< Version of the bone matrices. */
  std::vector<std::unique_ptr<Buffer>> bone_matrices_buffer_; /**< Buffer for storing bone matrices. */
  std::vector<std::unique_ptr<Buffer>> previous_bone_matrices_buffer_;
  std::vector<std::shared_ptr<DescriptorSet>> descriptor_set_; /**< Descriptor sets for the bone matrices. */
  friend class RenderLayer;
  friend class RenderInstanceStorage;

  /**
   * @brief Uploads bone matrix data to the GPU or relevant buffers.
   */
  void UploadData();
  void UploadPreviousData(const std::vector<glm::mat4>& matrices);

 public:
  std::vector<glm::mat4> value; /**< Bone matrix values. */

  /**
   * @brief Retrieves the descriptor set.
   * @return A shared pointer to the descriptor set.
   */
  [[nodiscard]] const std::shared_ptr<DescriptorSet>& GetDescriptorSet() const;

  [[nodiscard]] VkDescriptorBufferInfo GetPreviousBufferInfo() const;

  /**
   * @brief Constructor for BoneMatrices.
   */
  BoneMatrices();

  /**
   * @brief Gets the current version of the bone matrices.
   * @return The version number.
   */
  [[nodiscard]] uint32_t GetVersion() const;
};

[[nodiscard]] Vertex BuildSkinnedRayTracingVertex(const SkinnedVertex& skinned_vertex,
                                                  const std::vector<glm::mat4>& bone_matrices);

[[nodiscard]] std::vector<Vertex> BuildSkinnedRayTracingVertices(const std::vector<SkinnedVertex>& skinned_vertices,
                                                                 const std::vector<glm::mat4>& bone_matrices);

[[nodiscard]] std::vector<Vertex> BuildSkinnedRayTracingVertices(const std::vector<SkinnedVertex>& skinned_vertices,
                                                                 const std::vector<glm::mat4>& bone_matrices,
                                                                 const std::vector<uint32_t>& source_vertex_indices);

/**
 * @class SkinnedMesh
 * @brief Represents a skinned mesh used for skeletal animation and rendering.
 * Inherits from IAsset and IGeometry.
 */
class SkinnedMesh : public IAsset, public IGeometry {
  Bound bound_; /**< The bounding box of the skinned mesh. */
  friend class SkinnedMeshRenderer;
  friend class Particles;
  friend class Platform;
  friend class RenderLayer;
  friend class RenderInstanceStorage;
  friend class TopLevelAccelerationStructure;

  SkinnedVertexAttributes skinned_vertex_attributes_; /**< Attributes of the skinned vertices. */
  std::vector<SkinnedVertex> skinned_vertices_;       /**< List of skinned vertices. */
  std::vector<glm::uvec3> skinned_triangles_;         /**< List of triangles (indices) for the skinned mesh. */
  std::vector<MorphTarget> morph_targets_;
  std::vector<float> default_morph_weights_;
  std::vector<SkinnedVertex> morph_base_vertices_;
  std::shared_ptr<RangeDescriptor> skinned_triangle_range_; /**< Range descriptor for skinned triangles. */
  std::shared_ptr<RangeDescriptor> skinned_meshlet_range_;  /**< Range descriptor for skinned meshlets. */
  std::shared_ptr<RangeDescriptor>
      ray_tracing_triangle_range_; /**< Static triangle payload range used by ray tracing hit shaders. */
  std::shared_ptr<RangeDescriptor>
      ray_tracing_meshlet_range_; /**< Static meshlet payload range used by the ray tracing mirror geometry. */
  std::shared_ptr<BottomLevelAccelerationStructure>
      blas_; /**< Bind-pose bottom-level acceleration structure for ray tracing. */

  void ClearMorphTargets();

  friend struct SkinnedMeshBonesBlock;

  friend class Prefab;

 protected:
  /**
   * @brief Saves the skinned mesh to the specified path.
   * @param path The filesystem path to save the mesh to.
   * @return True if saving was successful, otherwise false.
   */
  bool SaveInternal(const std::filesystem::path& path) const;

 public:
  static bool RegisterAssetIoHandlers(const std::string& owner_name = {}, const std::string& type_name = "SkinnedMesh");

  [[nodiscard]] bool SupportsStagedLoading() const {
    return true;
  }

  /**
   * @brief Destructor for SkinnedMesh.
   */
  ~SkinnedMesh() override;

  /**
   * @brief Draws indexed elements of the skinned mesh.
   * @param vk_command_buffer The Vulkan command buffer for the drawing operation.
   * @param global_pipeline_state The global pipeline state for rendering.
   * @param instances_count The number of instances to draw.
   */
  void DrawIndexed(VkCommandBuffer vk_command_buffer, GraphicsPipelineStates& global_pipeline_state,
                   int instances_count) const override;

  /**
   * @brief Initializes resources for the skinned mesh.
   */
  void OnCreate() override;

  /**
   * @brief Fetches indices for the specified set of bones.
   * @param bones A vector of shared pointers to Bone objects.
   */
  void FetchIndices(const std::vector<std::shared_ptr<Bone>>& bones);

  std::vector<unsigned> bone_animator_indices; /**< Stores indices of the bone animator. */

  /**
   * @brief Gets the center of the skinned mesh.
   * @return The center position in 3D space.
   */
  [[nodiscard]] glm::vec3 GetCenter() const;

  /**
   * @brief Gets the bounding box of the skinned mesh.
   * @return The bounding box of the mesh.
   */
  [[nodiscard]] Bound GetBound() const;

  /**
   * @brief Sets the vertices for the skinned mesh using attribute information, vertices, and indices.
   * @param skinned_vertex_attributes The vertex attributes for the skinned mesh.
   * @param skinned_vertices A vector of SkinnedVertex objects.
   * @param indices A vector of indices defining triangles.
   */
  void SetVertices(const SkinnedVertexAttributes& skinned_vertex_attributes,
                   const std::vector<SkinnedVertex>& skinned_vertices, const std::vector<unsigned>& indices,
                   int tangent_tex_coord = 0, std::vector<uint32_t>* source_vertex_indices = nullptr);

  /**
   * @brief Sets the vertices for the skinned mesh using attribute information, vertices, and triangle data.
   * @param skinned_vertex_attributes The vertex attributes for the skinned mesh.
   * @param skinned_vertices A vector of SkinnedVertex objects.
   * @param triangles A vector of glm::uvec3 objects defining triangles.
   */
  void SetVertices(const SkinnedVertexAttributes& skinned_vertex_attributes,
                   const std::vector<SkinnedVertex>& skinned_vertices, const std::vector<glm::uvec3>& triangles,
                   int tangent_tex_coord = 0, std::vector<uint32_t>* source_vertex_indices = nullptr);

  void SetMorphTargets(std::vector<MorphTarget> morph_targets, std::vector<float> default_weights,
                       std::vector<SkinnedVertex> morph_base_vertices);

  [[nodiscard]] const std::vector<MorphTarget>& PeekMorphTargets() const;

  [[nodiscard]] const std::vector<float>& GetDefaultMorphWeights() const;

  [[nodiscard]] const std::vector<SkinnedVertex>& PeekMorphBaseVertices() const;

  [[nodiscard]] std::vector<SkinnedVertex> BuildMorphedVertices(const std::vector<float>& weights) const;

  /**
   * @brief Gets the amount of skinned vertices.
   * @return The number of skinned vertices.
   */
  [[nodiscard]] size_t GetSkinnedVerticesAmount() const;

  /**
   * @brief Gets the amount of triangles in the skinned mesh.
   * @return The number of triangles.
   */
  [[nodiscard]] size_t GetTriangleAmount() const;

  [[nodiscard]] const SkinnedVertexAttributes& GetSkinnedVertexAttributes() const;

  [[nodiscard]] const std::vector<SkinnedVertex>& PeekSkinnedVertices() const;

  [[nodiscard]] const std::vector<glm::uvec3>& PeekTriangles() const;

  /**
   * @brief Recalculates the normals of the skinned mesh.
   */
  void RecalculateNormal();

  /**
   * @brief Recalculates the tangents of the skinned mesh.
   */
  void RecalculateTangent(int tex_coord = 0);

  /**
   * @brief Provides unsafe access to the skinned vertices.
   * @return A reference to the vector of skinned vertices.
   */
  [[nodiscard]] std::vector<SkinnedVertex>& UnsafeGetSkinnedVertices();

  /**
   * @brief Provides unsafe access to the triangles of the skinned mesh.
   * @return A reference to the vector of triangles.
   */
  [[nodiscard]] std::vector<glm::uvec3>& UnsafeGetTriangles();
};

}  // namespace evo_engine
