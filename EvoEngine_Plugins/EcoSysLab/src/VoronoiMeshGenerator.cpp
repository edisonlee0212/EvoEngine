#include "VoronoiMeshGenerator.hpp"
#include "CubicHermiteSpline.hpp"
#include "KineticDelaunay.hpp"
#include "ObjExporter.hpp"
#include "SegmentBuilder.hpp"
#include "VoronoiMesh.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {
void printStrandGuidePoints(const std::vector<std::vector<kinDS::Point<2>>>& strand_guide_points) {
  // Print the points such that they can be copy-pasted into C++
  std::cout << "std::vector<std::vector<kinDS::Point<2>>> strand_guide_points = {\n";
  for (size_t strand_id = 0; strand_id < strand_guide_points.size(); ++strand_id) {
    std::cout << "  {";
    for (size_t point_id = 0; point_id < strand_guide_points[strand_id].size(); ++point_id) {
      const auto& point = strand_guide_points[strand_id][point_id];
      std::cout << "kinDS::Point<2>{" << point[0] << ", " << point[1] << "}";
      if (point_id < strand_guide_points[strand_id].size() - 1) {
        std::cout << ", ";
      }
    }
    std::cout << "},\n";
  }
}

static void RunMeshingAlgorithm(std::vector<Vertex>& vertices, std::vector<glm::vec2>& tex_coords,
                                std::vector<std::pair<unsigned int, unsigned int>>& index_pairs,
                                std::vector<kinDS::CubicHermiteSpline<2>> strand_splines) {
  // TODO: Here we could only extract the boundary mesh and use it
  kinDS::KineticDelaunay kinetic_delaunay(strand_splines, 10.0);

  kinetic_delaunay.init();
  kinDS::SegmentBuilder mesh_builder(kinetic_delaunay, strand_splines);
  mesh_builder.init();
  auto points = kinetic_delaunay.getPointsAt(0.0);

  size_t section_count = kinetic_delaunay.getSectionCount();

  evo_engine::ProgressBar progress_bar(0, section_count, "Computing Kinetic Voronoi Sections",
                                       evo_engine::ProgressBar::Display::Absolute);
  for (size_t i = 0; i < section_count; ++i) {
    progress_bar.Update(i);
    if (i != 0)
      mesh_builder.betweenSections(i);
    kinetic_delaunay.advanceOneSection(mesh_builder);

    points = kinetic_delaunay.getPointsAt(static_cast<double>(i + 1));
  }
  progress_bar.Finish();

  mesh_builder.finalize(section_count);

  auto [meshes, neighbor_segments] = mesh_builder.extractSegmentMeshlets();

  auto& boundary_mesh = mesh_builder.getBoundaryMesh();

  // intersect all meshes with the boundary mesh and save the result

  kinDS::MeshIntersection boundary_intersector(boundary_mesh);

  // TODO: We should probably remove this entire file
  // We no longer need this
  /* Jobs::RunParallelFor(meshes.size(), [&](const size_t mesh_index) {
    auto intersect_relation = boundary_intersector.ClassifyMeshRelation(meshes[mesh_index]);

    if (intersect_relation == kinDS::MeshIntersection::MeshRelation::INSIDE) {
      // fully inside, no need to compute intersection
      return;
    } else if (intersect_relation == kinDS::MeshIntersection::MeshRelation::OUTSIDE) {
      // fully outside, result is empty mesh
      meshes[mesh_index] = kinDS::VoronoiMesh();
      return;
    } else {
      // partially intersecting, compute intersection}
      meshes[mesh_index] = boundary_intersector.Intersect(meshes[mesh_index]);
    }
  });*/

  // for now, just combine all meshes into one
  kinDS::VoronoiMesh combined_mesh;
  for (const auto& mesh : meshes) {
    combined_mesh += mesh;
  }

  // export combined mesh
  combined_mesh.mergeDuplicateVertices(0.0001);
  kinDS::ObjExporter::writeMesh(combined_mesh, "meshtest.obj");

  // convert to output format

  for (const auto& v : combined_mesh.getVertices()) {
    // v is a relative position in 2D, we need to convert it to 3D

    Vertex vertex;
    vertex.position = glm::vec3(v[0], v[2], v[1]);
    vertices.push_back(vertex);
  }
  for (const auto& idx : combined_mesh.getTriangles()) {
    index_pairs.push_back(std::make_pair(static_cast<unsigned int>(idx), 0));
  }
}

void VoronoiMeshGenerator::Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                    std::vector<unsigned int>& indices,
                                    const StrandModelMeshGeneratorSettings& settings) {
  // Implementation of the Voronoi mesh generation algorithm
  // just use the index pair version and discard the tex coords
  std::vector<glm::vec2> tex_coords;
  std::vector<std::pair<unsigned int, unsigned int>> index_pairs;
  Generate(strand_model, vertices, tex_coords, index_pairs, settings);

  for (const auto& pair : index_pairs) {
    indices.emplace_back(pair.first);
  }
}

void VoronoiMeshGenerator::Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                    std::vector<glm::vec2>& tex_coords,
                                    std::vector<std::pair<unsigned int, unsigned int>>& index_pairs,
                                    const StrandModelMeshGeneratorSettings& settings) {
  // Implementation of the Voronoi mesh generation algorithm with texture coordinates
  // We need to generate this per slice and then connect the slices

  // Proof of concept, just assume we have one trunk with no branches and all strands have the same length

  const auto& strands = strand_model.strand_model_skeleton.data.strand_group.PeekStrands();
  // strand_model.strand_model_skeleton.data.strand_group.Subdivide()

  const StrandModelSkeleton& skeleton = strand_model.strand_model_skeleton;
  const StrandModelStrandGroup& strand_group = skeleton.data.strand_group;

  if (strands.size() < 3) {
    EVOENGINE_WARNING("VoronoiMeshGenerator requires at least 3 strands to generate a mesh.");
    return;
  }

  std::vector<std::vector<kinDS::Point<2>>> strand_guide_points(strands.size());

  const auto& nodes = skeleton.PeekRawNodes();
  SkeletonNodeHandle root_handle = 0;  // root is always 0
  const auto& profile = skeleton.PeekNode(root_handle).data.profile;

  for (auto& node_handle : skeleton.PeekSortedNodeList()) {
    auto& node = skeleton.PeekNode(node_handle);
    float t =
        float(skeleton.GetChainToRoot(node_handle).size() - 1);  // This is somewhat inefficient, but will do for now
    auto& particles = node.data.particle_map;

    for (const auto& k_v_pair : particles) {
      auto& particle = profile.PeekParticle(k_v_pair.second);
      glm::vec2 pos = particle.GetPosition();
      size_t strand_id = particle.strand_handle;

      // compute the 3D position of the particle
      glm::vec3 pos3D = node.info.global_position + node.info.global_rotation * glm::vec3(pos.x, pos.y, 0.0f);
      strand_guide_points[strand_id].emplace_back(kinDS::Point<2>{pos3D.x, pos3D.z});
    }
  }

  // EVOENGINE_LOG("Strand guide points:");
  // printStrandGuidePoints(strand_guide_points);

  // construct cubic hermite spline for each strand
  std::vector<kinDS::CubicHermiteSpline<2>> strand_splines;
  for (const auto& guide_points : strand_guide_points) {
    strand_splines.push_back(kinDS::CubicHermiteSpline<2>(guide_points));
  }

  RunMeshingAlgorithm(vertices, tex_coords, index_pairs, strand_splines);
}

void VoronoiMeshGenerator::Generate(const DtsStrandGroup& randomly_subdivided_strands, const StrandModel& strand_model,
                                    std::vector<Vertex>& vertices, std::vector<glm::vec2>& tex_coords,
                                    std::vector<std::pair<unsigned int, unsigned int>>& index_pairs,
                                    const StrandModelMeshGeneratorSettings& settings) {
  // Obtain strand guide points from the randomly subdivided strands
  // For now we use uniformly subdivided strands only for ease of implementation

  std::vector<std::vector<kinDS::Point<2>>> strand_guide_points;
  const auto& strands = randomly_subdivided_strands.PeekStrands();
  strand_guide_points.resize(strands.size());

  for (size_t strand_id = 0; strand_id < strands.size(); ++strand_id) {
    const auto& strand = strands[strand_id];
    const auto& segment_handles = strand.PeekStrandSegmentHandles();

    for (const auto& segment_handle : segment_handles) {
      const auto& segment = randomly_subdivided_strands.PeekStrandSegment(segment_handle);

      glm::vec3 end_pos = segment.end_position;

      // Add end position
      strand_guide_points[strand_id].emplace_back(kinDS::Point<2>{end_pos.y, end_pos.z});
    }
  }

  // construct cubic hermite spline for each strand
  std::vector<kinDS::CubicHermiteSpline<2>> strand_splines;
  for (const auto& guide_points : strand_guide_points) {
    strand_splines.push_back(kinDS::CubicHermiteSpline<2>(guide_points));
  }

  RunMeshingAlgorithm(vertices, tex_coords, index_pairs, strand_splines);
}
}  // namespace eco_sys_lab_plugin