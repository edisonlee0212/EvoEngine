#pragma once
#include "KineticDelaunay.hpp"
#include "MeshStructure.hpp"
#include "VoronoiMesh.hpp"

namespace kinDS {

class SegmentBuilder : public KineticDelaunay::EventHandler {
 private:
  // Maps strand IDs to their corresponding segment indices in correct order
  std::vector<std::vector<size_t>> strand_to_segment_indices;
  std::vector<MeshStructure::SegmentProperties> segment_properties;  // Properties for each segment mesh
  // Pairs of segments and their corresponding mesh data
  std::vector<MeshStructure::SegmentMeshPair> segment_mesh_pairs;
  std::vector<size_t>
      half_edge_index_to_segment_mesh_pair_index;  // Maps edge indices to their corresponding segment mesh pair indices
  std::vector<VoronoiMesh> meshes;                 // List of all generated meshes
  std::vector<std::pair<size_t, size_t>> segment_mesh_pair_last_left_and_right_vertex;
  // Maps corner indices (correspoding to outgoing half-edge inside the cell) to the index of the cutoff mesh, -1 if no
  // cutoff mesh exists
  std::vector<int> corner_to_cutoff_mesh_indices;
  bool create_transformed_mesh;

  // We no longer use these two factors, they are instead adjusted dynamically in the shader at runtime
  double uv_height_factor = 1.0;
  // 0.1;
  double uv_circum_factor = 1.0;
  // 2.0;
  double texture_diameter = 0.9;

  // for the boundary
  VoronoiMesh boundary_mesh;  // Mesh for the boundary cuts
  std::vector<std::pair<size_t, size_t>> boundary_mesh_last_left_and_right_vertex;
  std::vector<size_t> boundary_vertex_to_strand_id;

  // UVs must be adjusted to avoid seams, these are the raw UVs before adjustment
  std::vector<glm::dvec2> boundary_mesh_raw_uvs;

  // Map half-edges to a vertex index in the boundary mesh if a flip created a new boundary edge
  std::vector<int> half_edge_to_boundary_vertex_index;

  KineticDelaunay& kin_del;
  bool finalized = false;  // Flag to indicate if the mesh has been finalized
  std::vector<std::pair<size_t, double>> subdivisions;
  size_t subdivision_index = 0;

  glm::dvec3 computeVoronoiVertex(size_t half_edge_id, double t, size_t segment_mesh_pair_index) const;

  void finishMesh(size_t half_edge_id, double t, const std::vector<BoundaryPoint>& boundary_points);

  void startNewMesh(size_t half_edge_id, double t);

  void completeBoundaryMeshSection(size_t he_id, size_t new_left, size_t new_right);

  /**
   * Add a triangle to the boundary mesh. Automatically takes the raw UVs and adjusts them to avoid seams.
   * @param u first vertex index
   * @param v second vertex index
   * @param w third vertex index
   */
  size_t addBoundaryTriangle(size_t u, size_t v, size_t w);

  size_t addBoundaryVertex(glm::dvec3 vertex, glm::dvec2 centroid, size_t strand_id, double t);

  size_t addMeshletTriangle(VoronoiMesh& mesh, size_t u, size_t v, size_t w);

  size_t addMeshletVertex(VoronoiMesh& mesh, const std::vector<BoundaryPoint>& boundary_polygon,
                          const glm::dvec2& centroid, glm::dvec3 vertex, size_t strand_id, double t);

  void addVoronoiTriangulationToBoundaryMesh(double t, bool invert_orientation, double offset);

  std::vector<BoundaryPoint> traceConvexHull(double t) const;

  void advanceBoundaryMesh(double t, const std::vector<BoundaryPoint>& boundary_points, const glm::dvec2& centroid);

  void updateBoundary(double t, std::vector<bool>& visited, size_t component_index);

  void updateBoundaries(double t);

  void advanceBoundaryMeshes(double t);

  size_t createClosingMesh(size_t strand_id, double t, const std::vector<BoundaryPoint>& boundary_polygon,
                           const glm::dvec2& centroid);

  void accumulateSegmentProperties();

 public:
  SegmentBuilder(KineticDelaunay& kin_del, std::vector<std::pair<size_t, double>> subdivisions,
                 bool create_transformed_mesh);
  SegmentBuilder(KineticDelaunay& kin_del, bool create_transformed_mesh);

  void init() override;

  void betweenSections(size_t index) override;

  void beforeEvent(KineticDelaunay::Event& e) override;

  void afterEvent(KineticDelaunay::Event& e) override;

  void beforeBoundaryEvent(KineticDelaunay::Event& e) override;

  void afterBoundaryEvent(KineticDelaunay::Event& e) override;

  void insertSubdivision(size_t strand_id, double t);

  void finalize(double t) override;

  std::vector<VoronoiMesh> extractMeshes() const;

  std::pair<std::vector<VoronoiMesh>, std::vector<std::vector<int>>> extractSegmentMeshlets() const;

  const VoronoiMesh& getBoundaryMesh() const;

  const std::vector<size_t>& getBoundaryVertexToStrandId() const;

  const std::vector<std::vector<size_t>>& getStrandToSegmentIndices() const;

  std::vector<glm::dvec3> computeClampedVoronoiVertices(size_t strand_id, double t,
                                                        const std::vector<BoundaryPoint>& boundary_polygon,
                                                        const glm::dvec2& centroid);

  void splitComponent(size_t component_id, const std::vector<std::vector<size_t>>& new_components, double t);
};
}  // namespace kinDS