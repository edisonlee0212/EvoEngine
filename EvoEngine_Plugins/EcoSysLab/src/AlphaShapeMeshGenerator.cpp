#include "AlphaShapeMeshGenerator.hpp"

#ifdef USE_CGAL
#include <CGAL/Alpha_shape_3.h>
#include <CGAL/Alpha_shape_cell_base_3.h>
#include <CGAL/Alpha_shape_vertex_base_3.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#endif

#include <cassert>
#include <fstream>
#include <list>
#include "MeshGenUtils.hpp"

#ifdef USE_CGAL
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Alpha_shape_vertex_base_3<K> Vb;
typedef CGAL::Alpha_shape_cell_base_3<K> Fb;
typedef CGAL::Triangulation_data_structure_3<Vb, Fb> Tds;
typedef CGAL::Delaunay_triangulation_3<K, Tds, CGAL::Fast_location> Delaunay_CGAL;
typedef CGAL::Alpha_shape_3<Delaunay_CGAL> Alpha_shape_3;
typedef K::Point_3 Point;
typedef Alpha_shape_3::Alpha_iterator Alpha_iterator;
typedef Alpha_shape_3::NT NT;
#endif

using namespace evo_engine;
namespace eco_sys_lab_plugin {

void AlphaShapeMeshGenerator::Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                       std::vector<unsigned int>& indices,
                                       const StrandModelMeshGeneratorSettings& settings) {
  // extract all particles
  std::vector<glm::vec3> points;

  const StrandModelSkeleton& skeleton = strand_model.strand_model_skeleton;
  const StrandModelStrandGroup& strand_group = skeleton.data.strand_group;

  const auto& nodes = skeleton.PeekRawNodes();
  SkeletonNodeHandle root_handle = 0;  // root is always 0
  const auto& profile = skeleton.PeekNode(root_handle).data.profile;

  for (auto& node_handle : skeleton.PeekSortedNodeList()) {
    auto& node = skeleton.PeekNode(node_handle);
    float t =
        float(skeleton.GetChainToRoot(node_handle).size() - 1);  // This is somewhat inefficient, but will do for now
    auto& particles = skeleton.PeekNode(node_handle).data.particle_map;

    for (const auto& k_v_pair : particles) {
      auto& particle = profile.PeekParticle(k_v_pair.second);

      for (size_t i = 0; i < settings.steps_per_segment; i++) {
        points.emplace_back(
            GetPipePos(strand_model, k_v_pair.first, t + (float(i) / float(settings.steps_per_segment))));
      }
    }
  }

  float max_thickness = 0.0f;
  std::vector<StrandSegmentHandle> seg_group0 = GetSegGroup(strand_group, 0);
  ForEachSegment(strand_group, seg_group0, [&](const StrandSegment& seg) {
    if (seg.end_thickness > max_thickness) {
      max_thickness = seg.end_thickness;
    }
  });

  float max_dist = 2 * max_thickness * sqrt(2) * 2.5f * settings.cluster_distance;

  ComputeAlphaShape(points, vertices, indices, 1.0f / 50000.0f);
}

void AlphaShapeMeshGenerator::Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                       std::vector<glm::vec2>& tex_coords,
                                       std::vector<std::pair<unsigned int, unsigned int>>& index_pairs,
                                       const StrandModelMeshGeneratorSettings& settings) {
  // so far, we don't support texture coordinates
  std::vector<unsigned> indices;

  Generate(strand_model, vertices, indices, settings);

  for (Vertex& v : vertices) {
    tex_coords.emplace_back(v.tex_coord);
  }

  for (size_t index : indices) {
    index_pairs.emplace_back(index, index);
  }
}

void AlphaShapeMeshGenerator::ComputeAlphaShape(std::vector<glm::vec3> points, std::vector<Vertex>& vertices,
                                                std::vector<unsigned int>& indices, double alpha) {
#ifdef USE_CGAL
  Delaunay_CGAL dt;

  for (glm::vec3& point : points) {
    Point p_cgal(point[0], point[1], point[2]);
    dt.insert(p_cgal);
  }

  EVOENGINE_LOG("Delaunay computed.");
  // compute alpha shape
  Alpha_shape_3 as(dt);
  EVOENGINE_LOG("Alpha shape computed.");

  as.set_alpha(alpha);
  std::vector<Alpha_shape_3::Facet> facets;
  as.get_alpha_shape_facets(std::back_inserter(facets), Alpha_shape_3::REGULAR);

  // TODO: extract vertices separately
  std::size_t nbf = facets.size();
  for (std::size_t i = 0; i < nbf; ++i) {
    // To have a consistent orientation of the facet, always consider an exterior cell
    if (as.classify(facets[i].first) != Alpha_shape_3::EXTERIOR)
      facets[i] = as.mirror_facet(facets[i]);
    CGAL_assertion(as.classify(facets[i].first) == Alpha_shape_3::EXTERIOR);

    int triangle[3] = {
        (facets[i].second + 1) % 4,
        (facets[i].second + 2) % 4,
        (facets[i].second + 3) % 4,
    };

    /// according to the encoding of vertex indices, this is needed to get
    /// a consistent orienation
    if (facets[i].second % 2 == 0)
      std::swap(triangle[0], triangle[1]);

    for (size_t j = 0; j < 3; j++) {
      const Point& p = facets[i].first->vertex(triangle[j])->point();
      Vertex v;
      v.position = glm::vec3(p.x(), p.y(), p.z());
      vertices.emplace_back(v);
    }

    indices.emplace_back(3 * i);
    indices.emplace_back(3 * i + 1);
    indices.emplace_back(3 * i + 2);
  }

  #else
  EVOENGINE_ERROR("CGAL is required for alpha-shape computation!");
  #endif
}
}  // namespace eco_sys_lab_plugin
