#pragma once
#include "KineticDelaunay.hpp"
#include "MeshStructure.hpp"
#include "VoronoiMesh.hpp"
namespace kinDS {

class SegmentBuilder : public KineticDelaunay::EventHandler {
 private:
  std::vector<std::vector<size_t>>
      strand_to_segment_indices;  // Maps strand IDs to their corresponding segment indices in correct order
  std::vector<MeshStructure::SegmentProperties> segment_properties;  // Properties for each segment mesh
  std::vector<MeshStructure::SegmentMeshPair>
      segment_mesh_pairs;  // Pairs of segments and their corresponding mesh data
  std::vector<size_t>
      half_edge_index_to_segment_mesh_pair_index;  // Maps edge indices to their corresponding segment mesh pair indices
  std::vector<Mesh> meshes;                        // List of all generated meshes
  std::vector<std::pair<size_t, size_t>> segment_mesh_pair_last_left_and_right_vertex;
  // Maps corner indices (correspoding to outgoing half-edge inside the cell) to the index of the cutoff mesh, -1 if no
  // cutoff mesh exists
  std::vector<int> corner_to_cutoff_mesh_indices;

  // for the boundary
  Mesh boundary_mesh;  // Mesh for the boundary cuts
  std::vector<std::pair<size_t, size_t>> boundary_mesh_last_left_and_right_vertex;

  // Map half-edges to a vertex index in the boundary mesh if a flip created a new boundary edge
  std::vector<int> half_edge_to_boundary_vertex_index;

  const KineticDelaunay& kin_del;
  const std::vector<CubicHermiteSpline<2>>& splines;  // Reference to the splines used for the triangulation
  bool finalized = false;                             // Flag to indicate if the mesh has been finalized
  std::vector<std::pair<size_t, double>> subdivisions;
  size_t subdivision_index = 0;

  Point<3> computeVoronoiVertex(size_t half_edge_id, double t, size_t segment_mesh_pair_index) const;

  void finishMesh(size_t half_edge_id, double t);

  void startNewMesh(size_t half_edge_id, double t);

  void completeBoundaryMeshSection(size_t he_id, size_t new_left, size_t new_right);

  void addVoronoiTriangulationToBoundaryMesh(double t, bool invert_orientation, double offset);

  void traceBoundary(double t);

  size_t createClosingMesh(size_t strand_id, double t);

  void accumulateSegmentProperties();

 public:
  SegmentBuilder(const KineticDelaunay& kin_del, std::vector<CubicHermiteSpline<2>>& splines,
                 std::vector<std::pair<size_t, double>> subdivisions);
  SegmentBuilder(const KineticDelaunay& kin_del, std::vector<CubicHermiteSpline<2>>& splines);

  void init() override;

  void betweenSections(size_t index) override;

  void beforeEvent(KineticDelaunay::Event& e) override;

  void afterEvent(KineticDelaunay::Event& e) override;

  void insertSubdivision(size_t strand_id, double t);

  void finalize(double t) override;

  std::vector<Mesh> extractMeshes() const;

  std::vector<Mesh> extractSegmentMeshlets() const;

  const Mesh& getBoundaryMesh() const;
};
}  // namespace kinDS
