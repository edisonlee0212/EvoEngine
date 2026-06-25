
#pragma once
#include "Bound.hpp"
#include "GeometryStorage.hpp"
#include "GraphicsResources.hpp"
#include "IAsset.hpp"
#include "IGeometry.hpp"
#include "Platform.hpp"
#include "Vertex.hpp"

namespace evo_engine {

/**
 * @brief Structure representing vertex attributes.
 */
struct VertexAttributes {
  bool normal = false;    /**< Indicates if normals are enabled. */
  bool tangent = false;   /**< Indicates if tangents are enabled. */
  bool tex_coord = false; /**< Indicates if texture coordinates are enabled. */
  bool color = false;     /**< Indicates if vertex color is enabled. */

  /**
   * @brief Serializes the vertex attributes to a YAML emitter.
   *
   * @param out The YAML emitter to serialize the attributes to.
   */
  void Serialize(YAML::Emitter& out) const;

  /**
   * @brief Deserializes the vertex attributes from a YAML node.
   *
   * @param in The YAML node to deserialize the attributes from.
   */
  void Deserialize(const YAML::Node& in);
};

/**
 * @brief A class for managing particle information lists.
 */
class ParticleInfoList final : public IAsset {
  std::shared_ptr<RangeDescriptor> range_descriptor_; /**< Shared pointer to the range descriptor. */

 public:
  [[nodiscard]] bool SupportsStagedLoading() const {
    return true;
  }

  /**
   * @brief Creates the particle information list asset.
   */
  void OnCreate() override;

  /**
   * @brief Destructor for the ParticleInfoList class.
   */
  ~ParticleInfoList() override;

  /**
   * @brief Applies rays to the particle information list.
   *
   * @param rays A vector of rays to apply.
   * @param color The color of the rays.
   * @param ray_width The width of the rays.
   */
  void ApplyRays(const std::vector<Ray>& rays, const glm::vec4& color, float ray_width) const;

  /**
   * @brief Applies rays to the particle information list with individual colors.
   *
   * @param rays A vector of rays to apply.
   * @param colors A vector of colors for the rays.
   * @param ray_width The width of the rays.
   */
  void ApplyRays(const std::vector<Ray>& rays, const std::vector<glm::vec4>& colors, float ray_width) const;

  /**
   * @brief Applies connections between points with a single color.
   *
   * @param starts A vector of starting points.
   * @param ends A vector of ending points.
   * @param color The color of the connections.
   * @param ray_width The width of the connections.
   */
  void ApplyConnections(const std::vector<glm::vec3>& starts, const std::vector<glm::vec3>& ends,
                        const glm::vec4& color, float ray_width) const;

  /**
   * @brief Applies connections between points with individual colors.
   *
   * @param starts A vector of starting points.
   * @param ends A vector of ending points.
   * @param colors A vector of colors for the connections.
   * @param ray_width The width of the connections.
   */
  void ApplyConnections(const std::vector<glm::vec3>& starts, const std::vector<glm::vec3>& ends,
                        const std::vector<glm::vec4>& colors, float ray_width) const;

  /**
   * @brief Applies connections with individual colors and ray widths.
   *
   * @param starts A vector of starting points.
   * @param ends A vector of ending points.
   * @param colors A vector of colors for the connections.
   * @param ray_widths A vector of widths for the connections.
   */
  void ApplyConnections(const std::vector<glm::vec3>& starts, const std::vector<glm::vec3>& ends,
                        const std::vector<glm::vec4>& colors, const std::vector<float>& ray_widths) const;

  /**
   * @brief Sets particle information for the particle information list.
   *
   * @param particle_infos A vector of particle information to set.
   */
  void SetParticleInfos(const std::vector<ParticleInfo>& particle_infos) const;

  /**
   * @brief Retrieves a constant reference to the list of particle information.
   *
   * @return A constant reference to the list of particle information.
   */
  const std::vector<ParticleInfo>& PeekParticleInfoList() const;

  /**
   * @brief Retrieves the descriptor set for the particle information.
   *
   * @return A shared pointer to the descriptor set.
   */
  [[nodiscard]] const std::shared_ptr<DescriptorSet>& GetDescriptorSet() const;
};

/**
 * @brief A class representing a 3D mesh.
 */
class Mesh final : public IAsset, public IGeometry {
  Bound bound_ = {}; /**< The bounding box of the mesh. */

  std::vector<Vertex> vertices_;      /**< The vertices of the mesh. */
  std::vector<glm::uvec3> triangles_; /**< The triangles of the mesh. */

  VertexAttributes vertex_attributes_ = {}; /**< The vertex attributes of the mesh. */
  friend class RenderLayer;
  friend class RenderInstanceStorage;
  friend class TopLevelAccelerationStructure;
  std::shared_ptr<RangeDescriptor> triangle_range_; /**< Shared pointer to the triangle range descriptor. */
  std::shared_ptr<RangeDescriptor> meshlet_range_;  /**< Shared pointer to the meshlet range descriptor. */

  std::shared_ptr<BottomLevelAccelerationStructure>
      blas_; /**< Shared pointer to the bottom-level acceleration structure. */

 protected:
  /**
   * @brief Saves the mesh to a specified file path.
   *
   * @param path The file path to save the mesh to.
   * @return Returns true if the mesh was successfully saved.
   */
  bool SaveInternal(const std::filesystem::path& path) const;

 public:
  static bool RegisterAssetIoHandlers(const std::string& owner_name = {}, const std::string& type_name = "Mesh");

  [[nodiscard]] bool SupportsStagedLoading() const {
    return true;
  }

  /**
   * @brief Generates a thumbnail texture for the mesh.
   *
   * @return A shared pointer to the generated thumbnail texture.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture();

  /**
   * @brief Initializes the mesh asset.
   */
  void OnCreate() override;

  /**
   * @brief Destructor for the Mesh class.
   */
  ~Mesh() override;

  /**
   * @brief Renders the mesh using indexed drawing.
   *
   * @param vk_command_buffer The Vulkan command buffer to record the draw commands to.
   * @param global_pipeline_state The global graphics pipeline state.
   * @param instances_count The number of instances to render.
   */
  void DrawIndexed(VkCommandBuffer vk_command_buffer, GraphicsPipelineStates& global_pipeline_state,
                   int instances_count) const override;

  /**
   * @brief Sets the vertices and indices for the mesh.
   *
   * @param vertex_attributes The attributes for the vertices.
   * @param vertices A vector containing the vertices.
   * @param indices A vector containing the indices.
   */
  void SetVertices(const VertexAttributes& vertex_attributes, const std::vector<Vertex>& vertices,
                   const std::vector<unsigned>& indices);

  /**
   * @brief Sets the vertices and triangles for the mesh.
   *
   * @param vertex_attributes The attributes for the vertices.
   * @param vertices A vector containing the vertices.
   * @param triangles A vector containing the triangle indices.
   */
  void SetVertices(const VertexAttributes& vertex_attributes, const std::vector<Vertex>& vertices,
                   const std::vector<glm::uvec3>& triangles);

  /**
   * @brief Merges duplicate vertices in the mesh.
   */
  void MergeVertices();

  /**
   * @brief Retrieves the number of vertices in the mesh.
   *
   * @return The number of vertices.
   */
  [[nodiscard]] uint32_t GetVerticesAmount() const;

  /**
   * @brief Retrieves the number of triangles in the mesh.
   *
   * @return The number of triangles.
   */
  [[nodiscard]] uint32_t GetTriangleAmount() const;

  /**
   * @brief Recalculates the normals for the mesh.
   */
  void RecalculateNormal();

  /**
   * @brief Recalculates the tangents for the mesh.
   */
  void RecalculateTangent();

  /**
   * @brief Retrieves the range descriptor for the triangles in the mesh.
   *
   * @return A shared pointer to the range descriptor for the triangles.
   */
  [[nodiscard]] const std::shared_ptr<RangeDescriptor>& GetTriangleRange() const;

  /**
   * @brief Calculates the area of a triangle within the mesh.
   *
   * @param triangle The triangle to calculate the area for.
   * @return The area of the triangle.
   */
  [[nodiscard]] float CalculateTriangleArea(const glm::uvec3& triangle) const;

  /**
   * @brief Calculates the centroid of a triangle within the mesh.
   *
   * @param triangle The triangle to calculate the centroid for.
   * @return The centroid of the triangle as a 3D vector.
   */
  [[nodiscard]] glm::vec3 CalculateCentroid(const glm::uvec3& triangle) const;

  [[nodiscard]] const VertexAttributes& GetVertexAttributes() const;

  [[nodiscard]] const std::vector<Vertex>& PeekVertices() const;

  [[nodiscard]] const std::vector<glm::uvec3>& PeekTriangles() const;

  /**
   * @brief Provides unsafe access to the vertices of the mesh.
   *
   * @return A reference to the vertices vector.
   */
  [[nodiscard]] std::vector<Vertex>& UnsafeGetVertices();

  /**
   * @brief Provides unsafe access to the triangles of the mesh.
   *
   * @return A reference to the triangles vector.
   */
  [[nodiscard]] std::vector<glm::uvec3>& UnsafeGetTriangles();

  /**
   * @brief Retrieves the bounding box of the mesh.
   *
   * @return A Bound object representing the bounding box of the mesh.
   */
  [[nodiscard]] Bound GetBound() const;

  /**
   * @brief Retrieves the bottom-level acceleration structure (BLAS) of the mesh.
   *
   * @return A shared pointer to the BLAS.
   */
  [[nodiscard]] std::shared_ptr<BottomLevelAccelerationStructure> GetBlas() const;
};

}  // namespace evo_engine
