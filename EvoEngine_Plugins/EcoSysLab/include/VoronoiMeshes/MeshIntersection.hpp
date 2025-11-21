#include "VoronoiMesh.hpp"

namespace kinDS {

class MeshIntersection {
 public:
  static Mesh intersect(const Mesh& meshA, const Mesh& meshB);
};
}  // namespace kinDS
