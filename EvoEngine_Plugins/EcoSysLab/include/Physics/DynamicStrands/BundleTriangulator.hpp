#pragma once
#include "DynamicStrands.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {
class BundleTriangulator {
 public:
  BundleTriangulator(const std::vector<DynamicStrands::GpuUniformParticle>& uniform_particles);
  ~BundleTriangulator();
  void Triangulate(std::vector<DynamicStrands::GpuDelaunayTetrahedron>& tetrahedrons);

 private:
  const std::vector<DynamicStrands::GpuUniformParticle>& m_uniform_particles;
  // inherit from tuple so we can use std::set and a lexicographical comparison
  struct TriangleKey : public std::tuple<size_t, size_t, size_t> {
    TriangleKey(size_t a, size_t b, size_t c);
  };

  struct Triangle {
    Triangle(size_t a, size_t b, size_t c);

    size_t v0;
    size_t v1;
    size_t v2;
    int tet_id_below = -1;
    int face_index_below = -1;
    int tet_id_above = -1;
    int face_index_above = -1;
  };

  struct Bundle {
    std::vector<size_t> particles;
    std::map<TriangleKey, Triangle> triangulation;
  };

  std::map<eco_sys_lab_plugin::BundleTriangulator::TriangleKey,
           eco_sys_lab_plugin::BundleTriangulator::Triangle>::const_iterator
  FindTriangle(const TriangleKey& key, const Bundle& bundle, std::set<int>& node_indices_above,
               std::vector<glm::vec3>& points, size_t d, std::vector<std::map<int, Bundle>>& bundle_maps);
  unsigned int CheckTrianglesForMatchingTetrahedon(std::map<TriangleKey, Triangle>& triangulation,
                                                   std::vector<Delaunay3D::Tetrahedron>& local_tets,
                                                   std::vector<size_t>& indices);
};
}  // namespace eco_sys_lab_plugin
