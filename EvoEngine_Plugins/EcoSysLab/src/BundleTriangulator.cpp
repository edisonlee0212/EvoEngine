#include "BundleTriangulator.hpp"
#include "DynamicStrandUtils.hpp"

using namespace eco_sys_lab_plugin;

BundleTriangulator::TriangleKey::TriangleKey(size_t a, size_t b, size_t c) {
  std::get<0>(*this) = std::min(a, std::min(b, c));
  std::get<1>(*this) = std::max(std::min(a, b), std::min(std::max(a, b), c));
  std::get<2>(*this) = std::max(a, std::max(b, c));
}

BundleTriangulator::Triangle::Triangle(size_t a, size_t b, size_t c) : v0(a), v1(b), v2(c) {
}

BundleTriangulator::BundleTriangulator(const std::vector<DynamicStrands::GpuUniformParticle>& uniform_particles)
    : m_uniform_particles(uniform_particles) {
}

BundleTriangulator::~BundleTriangulator() {
}

void BundleTriangulator::Triangulate(std::vector<DynamicStrands::GpuDelaunayTetrahedron>& tetrahedrons) {
  int max_dist_from_root = 0;

  for (int i = 0; i < m_uniform_particles.size(); i++) {
    max_dist_from_root = std::max(max_dist_from_root, m_uniform_particles[i].segment_index);
  }

  std::vector<std::map<int, Bundle>> bundle_maps(max_dist_from_root + 1);

  // Sort particles into a map according to distance from root and node handle
  for (int i = 0; i < m_uniform_particles.size(); i++) {
    auto& particle = m_uniform_particles[i];
    auto& node_handle = particle.node_index;

    auto it = bundle_maps[particle.segment_index].lower_bound(node_handle);
    if (it == bundle_maps[particle.segment_index].end() ||
        bundle_maps[particle.segment_index].key_comp()(node_handle, it->first)) {
      it = bundle_maps[particle.segment_index].insert(it, std::make_pair(node_handle, Bundle()));
    }

    it->second.particles.push_back(i);
  }

  // Triangulate each plane between bundles
  for (size_t d = 0; d < bundle_maps.size(); d++) {
    auto& map = bundle_maps[d];
    Jobs::RunParallelFor(map.size(), [&](const size_t i) {
      auto& kv_pair = *std::next(map.begin(), i);
      auto& bundle = kv_pair.second;

      std::vector<float> points;

      for (auto& particle_index : bundle.particles) {
        auto& particle = m_uniform_particles[particle_index];

        points.push_back(particle.profile_position.x);
        points.push_back(particle.profile_position.y);
      }

      Delaunator::Delaunator2D delaunator(points);
      for (std::size_t i = 0; i < delaunator.triangles.size(); i += 3) {
        const auto& v0 = delaunator.triangles[i];
        const auto& v1 = delaunator.triangles[i + 1];
        const auto& v2 = delaunator.triangles[i + 2];

        bundle.triangulation.insert(std::make_pair(
            TriangleKey(bundle.particles[v0], bundle.particles[v1], bundle.particles[v2]), Triangle(v0, v1, v2)));
      }
    });
  }

  // 3D triangulate each bundle:
  for (int d = 0; d < bundle_maps.size() - 1; d++) {  // skip the last one as there is no match above
    auto& map = bundle_maps[d];
    for (auto& kv_pair : map) {
      const int& node_handle = kv_pair.first;
      auto& bundle = kv_pair.second;
      std::vector<size_t> indices;
      std::vector<glm::vec3> points;
      std::vector<unsigned int> triangles;

      if (bundle.particles.size() < 3) {
        continue;
      }

      // collect points and nodes above and compute average direction
      std::set<int> node_indices_above;
      glm::vec3 avg_dir = glm::vec3(0.f);
      for (auto particle_index : bundle.particles) {
        auto& particle = m_uniform_particles[particle_index];
        if (particle.next_particle_handle == -1) {
          continue;
        }

        indices.emplace_back(particle_index);
        points.emplace_back(particle.position);
        auto& next_particle = m_uniform_particles[particle.next_particle_handle];
        avg_dir += next_particle.position - particle.position;
        node_indices_above.insert(next_particle.node_index);
      }

      // compute a stretch matrix to "squish" the bundle
      avg_dir = glm::normalize(avg_dir);
      float stretch_factor = 0.1f;

      glm::mat3 projection_matrix = glm::mat3(avg_dir.x * avg_dir.x, avg_dir.x * avg_dir.y, avg_dir.x * avg_dir.z,
                                              avg_dir.y * avg_dir.x, avg_dir.y * avg_dir.y, avg_dir.y * avg_dir.z,
                                              avg_dir.z * avg_dir.x, avg_dir.z * avg_dir.y, avg_dir.z * avg_dir.z);

      glm::mat3 stretch_matrix = glm::mat3(1.f) + (stretch_factor - 1.0f) * projection_matrix;

      // update positions of already added points
      for (auto& point : points) {
        point = stretch_matrix * point;
      }

      // collect triangles from this plane
      for (const auto& tri_kv : bundle.triangulation) {
        triangles.push_back(tri_kv.second.v0);
        triangles.push_back(tri_kv.second.v1);
        triangles.push_back(tri_kv.second.v2);
      }

      // for each node above, collect the points and triangles
      for (auto node_index : node_indices_above) {
        size_t offset = points.size();
        auto& map_above = bundle_maps[d + 1];
        if (map_above.find(node_index) == map_above.end()) {
          continue;
        }

        // collect points
        auto& bundle_above = map_above[node_index];
        for (size_t i : bundle_above.particles) {
          auto& particle = m_uniform_particles[i];
          indices.emplace_back(i);
          points.emplace_back(stretch_matrix * particle.position);
        }

        // collect triangles
        for (const auto& tri_kv : bundle_above.triangulation) {
          triangles.push_back(tri_kv.second.v0 + offset);
          triangles.push_back(tri_kv.second.v1 + offset);
          triangles.push_back(tri_kv.second.v2 + offset);
        }
      }

      // run constrained Delaunay triangulation
      auto local_tets = Delaunay3D::GenerateTetrahedronsConstrained(points, triangles);
      std::vector<int> valid_index_map(local_tets.size(), -1);

      // Run a fill-algorithm that collects all tetrahedrons between the two planes
      std::vector<bool> visited(local_tets.size(), false);  // WARNING: this is not thread safe

      for (size_t tet_index = 0; tet_index < local_tets.size(); tet_index++) {
        auto& tet = local_tets[tet_index];
        int tet_vertices[4];
        for (size_t i = 0; i < 4; i++) {
          tet_vertices[i] = indices[tet.v[i]];
        }

        int max_segment_index_diff = DynamicStrandUtils::MaxSegmentIndexDifference(tet_vertices, m_uniform_particles);

        // now traverse neighboring tetrahedrons until we hit the plane triangles using a bfs
        if (max_segment_index_diff == 1) {
          std::queue<size_t> queue;
          queue.push(tet_index);
          visited[tet_index] = true;

          while (!queue.empty()) {
            size_t current_tet_index = queue.front();
            queue.pop();
            auto& current_tet = local_tets[current_tet_index];
            int current_tet_vertices[4];
            for (size_t i = 0; i < 4; i++) {
              current_tet_vertices[i] = indices[current_tet.v[i]];
            }

            for (size_t i = 0; i < 4; i++) {
              if (current_tet.neighbor_tet_indices[i] == -1) {
                continue;
              }
              size_t neighbor_tet_index = current_tet.neighbor_tet_indices[i];
              if (visited[neighbor_tet_index]) {
                continue;
              }

              visited[neighbor_tet_index] = true;
              auto& neighbor_tet = local_tets[neighbor_tet_index];
              int neighbor_vertices[4];
              for (size_t j = 0; j < 4; j++) {
                neighbor_vertices[j] = indices[neighbor_tet.v[j]];
              }
              int max_segment_index_diff =
                  DynamicStrandUtils::MaxSegmentIndexDifference(neighbor_vertices, m_uniform_particles);
              if (max_segment_index_diff == 0) {
                // check if we reached a triangle from the triangulated plane
                auto face_vertices = DynamicStrandUtils::GetFaceVertices(current_tet_vertices, i);
                TriangleKey key(face_vertices[0], face_vertices[1], face_vertices[2]);
                // need to check bundle and all bundles above
                std::map<eco_sys_lab_plugin::BundleTriangulator::TriangleKey,
                         eco_sys_lab_plugin::BundleTriangulator::Triangle>::const_iterator tri_it =
                    FindTriangle(key, bundle, node_indices_above, points, d, bundle_maps);

                if (tri_it == bundle.triangulation.end()) {
                  // Must be a (nearly) degenerate triangle that we need to traverse to reach the boundary
                  queue.push(neighbor_tet_index);
                }

              } else if (max_segment_index_diff == 1) {
                queue.push(neighbor_tet_index);
              }
            }
          }

          break;
        }
      }
      // add the tets to the global list
      for (size_t tet_index = 0; tet_index < local_tets.size(); tet_index++) {
        auto& tet = local_tets[tet_index];
        DynamicStrands::GpuDelaunayTetrahedron gpu_tet;
        for (size_t i = 0; i < 4; i++) {
          gpu_tet.indices[i] = indices[tet.v[i]];
          gpu_tet.neighbor_tet_ids[i] = -1;
          // tet.neighbor_tet_indices[i];
          gpu_tet.is_bark[i] = -1;
        }

        if (!visited[tet_index]) {
          // check if there are any triangles from the plane that we skip here
          for (size_t face_index = 0; face_index < 4; face_index++) {
            auto face_vertices = DynamicStrandUtils::GetFaceVertices(gpu_tet.indices, face_index);
            TriangleKey key(face_vertices[0], face_vertices[1], face_vertices[2]);
            // need to check bundle and all bundles above
            std::map<eco_sys_lab_plugin::BundleTriangulator::TriangleKey,
                     eco_sys_lab_plugin::BundleTriangulator::Triangle>::const_iterator tri_it =
                FindTriangle(key, bundle, node_indices_above, points, d, bundle_maps);
            if (tri_it != bundle.triangulation.end()) {
              EVOENGINE_WARNING("Found an omitted tetrahedron that borders the plane!");
              continue;
            }
          }
        } else {
          // TODO: set up other members
          valid_index_map[tet_index] = tetrahedrons.size();
          tetrahedrons.push_back(gpu_tet);
        }
      }

      // update neighbor indices
      for (size_t tet_index = 0; tet_index < local_tets.size(); tet_index++) {
        auto& tet = local_tets[tet_index];
        if (valid_index_map[tet_index] == -1) {
          continue;
        }
        auto& gpu_tet = tetrahedrons[valid_index_map[tet_index]];
        for (size_t i = 0; i < 4; i++) {
          if (tet.neighbor_tet_indices[i] == -1) {
            continue;
          }
          gpu_tet.neighbor_tet_ids[i] = valid_index_map[tet.neighbor_tet_indices[i]];
        }
      }
    }
  }

  // TODO: we should do this using the constraint faces, that would be much more efficient

  for (size_t tet_id = 0; tet_id < tetrahedrons.size(); tet_id++) {
    auto& gpu_tet = tetrahedrons[tet_id];
    // iterate through all faces of the tetrahedron
    for (size_t face_index = 0; face_index < 4; face_index++) {
      size_t u = (face_index == 0) ? 1 : 0;
      std::vector<size_t> face_vertices;
      for (size_t v = 0; v < 4; v++) {
        if (face_index == v) {
          continue;
        }

        // check if all vertices have the same distance from root
        if (m_uniform_particles[gpu_tet.indices[v]].segment_index !=
            m_uniform_particles[gpu_tet.indices[u]].segment_index) {
          break;
        }
        face_vertices.emplace_back(gpu_tet.indices[v]);
      }

      if (face_vertices.size() != 3) {
        continue;
      }

      size_t d = m_uniform_particles[gpu_tet.indices[u]].segment_index;

      auto& map = bundle_maps[d];
      auto& bundle = map[m_uniform_particles[gpu_tet.indices[u]].node_index];
      auto tri_it = bundle.triangulation.find(TriangleKey(face_vertices[0], face_vertices[1], face_vertices[2]));

      if (tri_it == bundle.triangulation.end()) {
        // EVOENGINE_LOG("Triangle not found in triangulation, skipping!");
        continue;
      }
      // EVOENGINE_LOG("Triangle found in triangulation!");

      if (m_uniform_particles[gpu_tet.indices[face_index]].segment_index < d) {
        tri_it->second.tet_id_below = tet_id;
        tri_it->second.face_index_below = face_index;
      } else {
        tri_it->second.tet_id_above = tet_id;
        tri_it->second.face_index_above = face_index;
      }
    }
  }

  // now iterate through all triangles and glue the tets together
  for (size_t d = 0; d < bundle_maps.size(); d++) {
    size_t missing_below = 0;
    size_t missing_above = 0;
    size_t total = 0;
    auto& map = bundle_maps[d];
    for (auto& kv_pair : map) {
      auto& bundle = kv_pair.second;
      for (auto& tri_kv : bundle.triangulation) {
        auto& tri = tri_kv.second;
        total++;
        if (tri.tet_id_below != -1 && tri.tet_id_above != -1) {
          auto& tet_below = tetrahedrons[tri.tet_id_below];
          auto& tet_above = tetrahedrons[tri.tet_id_above];

          tet_below.neighbor_tet_ids[tri.face_index_below] = tri.tet_id_above;
          tet_above.neighbor_tet_ids[tri.face_index_above] = tri.tet_id_below;
        } else {
          if (tri.tet_id_below == -1) {
            missing_below++;
          }
          if (tri.tet_id_above == -1) {
            missing_above++;
          }
        }
      }
    }

    EVOENGINE_LOG("Level " << d << ": Missing below: " << missing_below << "/" << total
                           << ", missing above: " << missing_above << "/" << total);
  }
}

std::map<eco_sys_lab_plugin::BundleTriangulator::TriangleKey,
         eco_sys_lab_plugin::BundleTriangulator::Triangle>::const_iterator
BundleTriangulator::FindTriangle(const TriangleKey& key, const Bundle& bundle, std::set<int>& node_indices_above,
                                 std::vector<glm::vec3>& points, size_t d,
                                 std::vector<std::map<int, Bundle>>& bundle_maps) {
  bool found_triangle = false;
  auto tri_it = bundle.triangulation.find(key);
  if (tri_it != bundle.triangulation.end()) {
    // tri_it->second.tet_id_above = tet_index;
    // tri_it->second.face_index_above = i;
    found_triangle = true;
  } else {
    for (auto node_index : node_indices_above) {
      size_t offset = points.size();
      auto& map_above = bundle_maps[d + 1];
      if (map_above.find(node_index) == map_above.end()) {
        continue;
      }

      // collect points
      auto& bundle_above = map_above[node_index];

      tri_it = bundle_above.triangulation.find(key);
      if (tri_it != bundle_above.triangulation.end()) {
        // tri_it->second.tet_id_below = tet_index;
        // tri_it->second.face_index_below = i;
        found_triangle = true;
        break;
      }
    }
  }

  if (!found_triangle) {
    tri_it = bundle.triangulation.end();
  }

  return tri_it;
}
