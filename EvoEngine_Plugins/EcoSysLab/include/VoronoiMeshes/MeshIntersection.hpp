#include "MeshCGAL.hpp"
#include "VoronoiMesh.hpp"

#ifdef USE_CGAL
#  include <CGAL/AABB_face_graph_triangle_primitive.h>
#  include <CGAL/AABB_traits.h>
#  include <CGAL/AABB_tree.h>
#  include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#  include <CGAL/Polygon_mesh_processing/connected_components.h>
#  include <CGAL/Polygon_mesh_processing/corefinement.h>  // for corefine_and_compute_intersection()
#  include <CGAL/Polygon_mesh_processing/measure.h>       // optional area/volume utils
#  include <CGAL/Polygon_mesh_processing/orientation.h>   // for orient_to_bound_a_volume()
#  include <CGAL/Polygon_mesh_processing/repair.h>        // for merge_duplicate_vertices(), remove_isolated_vertices()
#  include <CGAL/Polygon_mesh_processing/repair_degeneracies.h>  // for remove_degenerate_faces()
#  include <CGAL/Polygon_mesh_processing/stitch_borders.h>       // for stitch_borders()
#  include <CGAL/Polygon_mesh_processing/triangulate_faces.h>    // (optional) triangulate_face if needed
#  include <CGAL/Surface_mesh.h>
#endif
namespace kinDS {
#ifdef USE_CGAL
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_3 = Kernel::Point_3;
using Primitive = CGAL::AABB_face_graph_triangle_primitive<MeshCGAL_internal>;
using Traits = CGAL::AABB_traits<Kernel, Primitive>;
using TreeCGAL = CGAL::AABB_tree<Traits>;
namespace PMP = CGAL::Polygon_mesh_processing;

typedef Kernel::Triangle_3 Triangle_3;
typedef std::vector<Triangle_3> TriangleList;

struct FaceProperties {
  int face_id;
  int test0;
  size_t test1;
};

// Small POD to record origin
struct Origin {
  int mesh_index;       // 0 -> m1, 1 -> m2
  std::size_t face_id;  // id assigned on the source mesh via face_index_map
};

//
// Visitor that records mapping from output face -> Origin
//
struct RecordingVisitor : public PMP::Corefinement::Default_visitor<MeshCGAL_internal> {
  using FaceDescriptor = MeshCGAL_internal::Face_index;

  Origin current_origin{-1, 0};

  // input face index maps
  const MeshCGAL<Origin>& m0;
  const MeshCGAL<Origin>& m1;
  MeshCGAL<Origin>& result;

  RecordingVisitor(const MeshCGAL<Origin>& m0, const MeshCGAL<Origin>& m1, MeshCGAL<Origin>& result)
      : m0(m0), m1(m1), result(result) {
  }

  void after_face_copy(FaceDescriptor f_src, const MeshCGAL_internal& tm_src, FaceDescriptor f_tgt,
                       MeshCGAL_internal& tm_tgt) {
    auto [fmap, ok] = tm_tgt.property_map<FaceDescriptor, Origin>(result.property_name);

    if (!ok) {
      EVOENGINE_ERROR("after_face_copy: Could not retrieve property map!");
      return;
    }

    if (&tm_src == &m0.mesh) {
      fmap[f_tgt] = m0.fidx[f_src];
    } else if (&tm_src == &m1.mesh) {
      fmap[f_tgt] = m1.fidx[f_src];
    }
  }

  void before_subface_creations(FaceDescriptor f_old, MeshCGAL_internal& tm_src) {
    if (&tm_src == &m0.mesh) {
      current_origin = m0.fidx[f_old];
    } else {
      current_origin = m1.fidx[f_old];
    }
  }

  void after_subface_created(FaceDescriptor f_new, MeshCGAL_internal& tm_tgt) {
    auto property_map_pair = tm_tgt.property_map<FaceDescriptor, Origin>(m0.property_name);

    if (!property_map_pair.second) {
      EVOENGINE_ERROR("after_subface_created: Could not retrieve property map!");
      return;
    }

    property_map_pair.first[f_new] = current_origin;
  }
};

#endif

struct MatchResult {
  bool hit = false;
  size_t triangle_index;
  double u, v, w;  // barycentric
};

class MeshIntersection {
 public:
  MeshIntersection(const VoronoiMesh& static_mesh);

  std::pair<VoronoiMesh, std::vector<int>> Intersect(const VoronoiMesh& mesh,
                                                     const std::vector<int>& neighbor_segments = {});

  MatchResult MatchPointOnSurface(const VoronoiPoint<3>& p, double epsilon = 1e-6);

  enum class MeshRelation { INSIDE, OUTSIDE, INTERSECTING, UNDEFINED };

  /**
   * Classify the relation of the input with respect to the internal mesh, can be INSIDE, OUTSIDE or INTERSECTING.
   * If CGAL is not available, UNDEFINED will always be returned.
   *
   * \param mesh The mesh to classify in relation to the internal mesh.
   * \param assume_inside If set to true, it will be assumed that the mesh is entirely inside if no intersections are
   * found. This is useful for the Voronoi meshing algortihm because we know that all cells are at least partially
   * inside.
   * \return the relation of the mesh to the internal mesh.
   */
  MeshRelation ClassifyMeshRelation(const VoronoiMesh& mesh, bool assume_inside = false);

 private:
#ifdef USE_CGAL
  VoronoiMesh boundary_mesh_voronoi;  // We need to store this too for interpolating properties
  MeshCGAL<Origin> boundary_mesh;
  TreeCGAL tree;
#endif
};
}  // namespace kinDS