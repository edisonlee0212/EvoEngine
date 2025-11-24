#include "VoronoiMesh.hpp"

namespace kinDS {
// Provides a structure to hold the mesh data for the segments as it will later be on the GPU
struct MeshStructure {
  struct SegmentMeshPair {
    size_t segment_index0;
    size_t segment_index1;
    size_t mesh_start_index;  // start index into the index buffer
    size_t mesh_length;       // length of the mesh in the index buffer
    size_t connected = 1;
  };

  // TODO: will a fixed size array be sufficient here? Alternatively, we need a separate buffer with start index and
  // length
  struct SegmentProperties {
    static const size_t MAX_NEIGHBORS = 63;

    size_t mesh_pair_indices[MAX_NEIGHBORS];
    size_t neighbor_count = 0;
    size_t neighbor_indices[MAX_NEIGHBORS];  // TODO: do we need this? We can access it through the segment mesh pairs
    size_t padding;
  };

 private:
  VoronoiMesh mesh;

  std::vector<SegmentMeshPair> segment_pairs;  // pairs of (start, end) indices into the index buffer for each meshlet
};
}  // namespace kinDS