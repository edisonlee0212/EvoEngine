#include "BillboardCloud.hpp"
using namespace evo_engine;

typedef int VIndex;
typedef int TIndex;
typedef int CIndex;
struct ConnectivityGraph {
  struct V {
    enum class VType { Default, LocalExtremum, SaddlePoint };
    VType vertex_type = VType::Default;
    std::vector<std::pair<VIndex, float>> connected_vertices;
    VIndex vertex_index = -1;
    float ind = -1;
    std::vector<TIndex> connected_triangles;
    int contour_index = -1;
    float distance_to_source_point = FLT_MAX;
    VIndex source_index = -1;
    VIndex prev_index = -1;
    int connected_component_id = -1;
  };

  std::vector<V> vertices;

  void EstablishConnectivityGraph(const BillboardCloud::Element& element);

  struct ConnectedComponent {
    std::vector<VIndex> component_vertices;
    std::unordered_set<VIndex> vertices_set;
    VIndex source_index = -1;
    CIndex index = -1;

    std::unordered_map<VIndex, int> groups;
  };
  std::vector<ConnectedComponent> connected_components;
  std::vector<std::vector<VIndex>> contour_table;
};

void ConnectivityGraph::EstablishConnectivityGraph(const BillboardCloud::Element& element) {
  vertices.clear();
  vertices.resize(element.vertices.size());
  for (int vi = 0; vi < vertices.size(); vi++) {
    vertices[vi] = {};
    vertices[vi].vertex_index = vi;
    vertices[vi].distance_to_source_point = FLT_MAX;
  }
  for (TIndex triangle_index = 0; triangle_index < element.triangles.size(); triangle_index++) {
    const auto& triangle = element.triangles.at(triangle_index);

    const auto& vertex0 = element.vertices[triangle.x];
    const auto& vertex1 = element.vertices[triangle.y];
    const auto& vertex2 = element.vertices[triangle.z];

    auto& v0 = vertices[triangle.x];
    auto& v1 = vertices[triangle.y];
    auto& v2 = vertices[triangle.z];

    v0.connected_triangles.emplace_back(triangle_index);
    v1.connected_triangles.emplace_back(triangle_index);
    v2.connected_triangles.emplace_back(triangle_index);

    {
      bool find_y = false;
      bool find_z = false;
      for (const auto& connected_index : v0.connected_vertices) {
        if (connected_index.first == triangle.y)
          find_y = true;
        if (connected_index.first == triangle.z)
          find_z = true;
      }
      if (!find_y)
        v0.connected_vertices.emplace_back(triangle.y, glm::distance(vertex0.position, vertex1.position));
      if (!find_z)
        v0.connected_vertices.emplace_back(triangle.z, glm::distance(vertex0.position, vertex2.position));
    }
    {
      bool find_x = false;
      bool find_z = false;
      for (const auto& connected_index : v1.connected_vertices) {
        if (connected_index.first == triangle.x)
          find_x = true;
        if (connected_index.first == triangle.z)
          find_z = true;
      }
      if (!find_x)
        v1.connected_vertices.emplace_back(triangle.x, glm::distance(vertex1.position, vertex0.position));
      if (!find_z)
        v1.connected_vertices.emplace_back(triangle.z, glm::distance(vertex1.position, vertex2.position));
    }
    {
      bool find_x = false;
      bool find_y = false;
      for (const auto& connected_index : v2.connected_vertices) {
        if (connected_index.first == triangle.x)
          find_x = true;
        if (connected_index.first == triangle.y)
          find_y = true;
      }
      if (!find_x)
        v2.connected_vertices.emplace_back(triangle.x, glm::distance(vertex2.position, vertex0.position));
      if (!find_y)
        v2.connected_vertices.emplace_back(triangle.y, glm::distance(vertex2.position, vertex1.position));
    }
  }
}

std::vector<std::vector<unsigned>> BillboardCloud::Element::CalculateLevelSets(const glm::vec3& direction) {
  ConnectivityGraph connectivity_graph{};
  connectivity_graph.EstablishConnectivityGraph(*this);
  std::vector<std::vector<unsigned>> ret_val;
  struct VNode {
    VIndex vertex_index = -1;
    float m_distance = 0.f;
  };
  struct VNodeComparator {
    bool operator()(const VNode& left, const VNode& right) const {
      return left.m_distance > right.m_distance;
    }
  };
  std::vector<bool> visited;
  visited.resize(vertices.size());
  std::fill(visited.begin(), visited.end(), false);

  bool dist_updated = true;
  std::vector<bool> updated;
  updated.resize(vertices.size());
  while (dist_updated) {
    dist_updated = false;
    std::fill(updated.begin(), updated.end(), false);

    VIndex seed_vertex_index = -1;
    float min_height = FLT_MAX;
    for (const auto& triangle : triangles) {
      const auto& vertex0 = vertices[triangle.x];
      const auto& vertex1 = vertices[triangle.y];
      const auto& vertex2 = vertices[triangle.z];

      if (!connectivity_graph.vertices[triangle.x].connected_vertices.empty() && !visited[triangle.x] &&
          glm::dot(direction, vertex0.position) < min_height) {
        seed_vertex_index = static_cast<VIndex>(triangle.x);
        min_height = vertex0.position.y;
      }
      if (!connectivity_graph.vertices[triangle.y].connected_vertices.empty() && !visited[triangle.y] &&
          glm::dot(direction, vertex1.position) < min_height) {
        seed_vertex_index = static_cast<VIndex>(triangle.y);
        min_height = vertex1.position.y;
      }
      if (!connectivity_graph.vertices[triangle.z].connected_vertices.empty() && !visited[triangle.z] &&
          glm::dot(direction, vertex2.position) < min_height) {
        seed_vertex_index = static_cast<VIndex>(triangle.z);
        min_height = vertex2.position.y;
      }
    }
    if (seed_vertex_index == -1)
      break;
    connectivity_graph.vertices[seed_vertex_index].distance_to_source_point = 0;
    int number_of_contours = 1;
    connectivity_graph.vertices[seed_vertex_index].contour_index = number_of_contours - 1;

    updated[seed_vertex_index] = true;

    std::priority_queue<VNode, std::vector<VNode>, VNodeComparator> q;
    q.push({seed_vertex_index, 0});

    while (!q.empty()) {
      const auto node = q.top();
      if (node.m_distance == FLT_MAX)
        break;
      const VIndex u = node.vertex_index;
      q.pop();
      if (visited[node.vertex_index])
        continue;
      auto& vu = connectivity_graph.vertices[u];
      for (const auto& neighbor : vu.connected_vertices) {
        const float new_dist = vu.distance_to_source_point + neighbor.second;
        if (auto& neighbor_v = connectivity_graph.vertices[neighbor.first];
            neighbor_v.distance_to_source_point > new_dist) {
          neighbor_v.prev_index = u;
          neighbor_v.distance_to_source_point = new_dist;
          neighbor_v.contour_index = vu.contour_index;
          q.push({neighbor.first, new_dist});
          dist_updated = true;
          updated[neighbor.first] = true;
        }
      }
      visited[u] = true;

      int sign_change_count = 0;
      for (const auto& triangle_index : vu.connected_triangles) {
        const auto& triangle = triangles.at(triangle_index);
        const auto dist_u = vu.distance_to_source_point;
        if (static_cast<int>(triangle.x) == u) {
          const auto& dist_y = connectivity_graph.vertices[triangle.y].distance_to_source_point;
          const auto& dist_z = connectivity_graph.vertices[triangle.z].distance_to_source_point;
          if ((dist_y > dist_u && dist_z < dist_u) || (dist_y < dist_u && dist_z > dist_u)) {
            sign_change_count++;
          }
        } else if (static_cast<int>(triangle.y) == u) {
          const auto& dist_x = connectivity_graph.vertices[triangle.x].distance_to_source_point;
          const auto& dist_z = connectivity_graph.vertices[triangle.z].distance_to_source_point;
          if ((dist_x > dist_u && dist_z < dist_u) || (dist_x < dist_u && dist_z > dist_u)) {
            sign_change_count++;
          }
        } else if (static_cast<int>(triangle.z) == u) {
          const auto& dist_x = connectivity_graph.vertices[triangle.x].distance_to_source_point;
          const auto& dist_y = connectivity_graph.vertices[triangle.y].distance_to_source_point;
          if ((dist_x > dist_u && dist_y < dist_u) || (dist_x < dist_u && dist_y > dist_u)) {
            sign_change_count++;
          }
        }
      }
      vu.ind = 1.f - static_cast<float>(sign_change_count) / 2.f;
      if (vu.ind == 1.f) {
        vu.vertex_type = ConnectivityGraph::V::VType::LocalExtremum;

      } else if (vu.ind < 0) {
        vu.vertex_type = ConnectivityGraph::V::VType::SaddlePoint;

        // Split into sub-contours
        std::vector<VIndex> unprocessed_neighbors;
        unprocessed_neighbors.resize(vu.connected_vertices.size());
        for (int neighbor_index = 0; neighbor_index < vu.connected_vertices.size(); neighbor_index++) {
          unprocessed_neighbors.at(neighbor_index) = vu.connected_vertices[neighbor_index].first;
        }
        std::vector<std::pair<bool, std::vector<VIndex>>> groups;
        while (!unprocessed_neighbors.empty()) {
          // Create a new group
          groups.emplace_back();
          auto& group = groups.back();
          auto next_neighbor = unprocessed_neighbors.back();

          std::vector<VIndex> wait_list;
          wait_list.emplace_back(next_neighbor);
          // Assign sign to current group
          group.first =
              connectivity_graph.vertices[next_neighbor].distance_to_source_point > vu.distance_to_source_point;
          while (!wait_list.empty()) {
            auto walker = wait_list.back();
            wait_list.pop_back();
            // Add current walker into group and remove it from unprocessed list
            group.second.emplace_back(walker);
            for (int unprocessed_vertex_index = 0; unprocessed_vertex_index < unprocessed_neighbors.size();
                 unprocessed_vertex_index++) {
              if (unprocessed_neighbors[unprocessed_vertex_index] == walker) {
                unprocessed_neighbors[unprocessed_vertex_index] = unprocessed_neighbors.back();
                unprocessed_neighbors.pop_back();
                break;
              }
            }
            // Try to find another adjacent vertex
            for (const auto& triangle_index : vu.connected_triangles) {
              const auto& triangle = triangles.at(triangle_index);
              for (int v0 = 0; v0 < 3; v0++) {
                if (static_cast<int>(triangle[v0]) == u) {
                  if (static_cast<int>(triangle[(v0 + 1) % 3]) == walker) {
                    const int target = static_cast<int>(triangle[(v0 + 2) % 3]);
                    // If target is not processed
                    bool unprocessed = false;
                    for (const auto& unprocessed_index : unprocessed_neighbors) {
                      if (unprocessed_index == target) {
                        unprocessed = true;
                      }
                    }
                    if (unprocessed) {
                      // And it has the same sign as current group...
                      if (group.first &&
                          connectivity_graph.vertices[target].distance_to_source_point > vu.distance_to_source_point) {
                        wait_list.emplace_back(target);
                      } else if (!group.first && connectivity_graph.vertices[target].distance_to_source_point <
                                                     vu.distance_to_source_point) {
                        wait_list.emplace_back(target);
                      }
                    }
                  } else if (static_cast<int>(triangle[(v0 + 2) % 3]) == walker) {
                    const int target = static_cast<int>(triangle[(v0 + 1) % 3]);
                    // If target is not processed
                    bool unprocessed = false;
                    for (const auto& unprocessed_index : unprocessed_neighbors) {
                      if (unprocessed_index == target) {
                        unprocessed = true;
                      }
                    }
                    if (unprocessed) {
                      // And it has the same sign as current group...
                      if (group.first &&
                          connectivity_graph.vertices[target].distance_to_source_point > vu.distance_to_source_point) {
                        wait_list.emplace_back(target);
                      } else if (!group.first && connectivity_graph.vertices[target].distance_to_source_point <
                                                     vu.distance_to_source_point) {
                        wait_list.emplace_back(target);
                      }
                    }
                  }
                }
              }
            }
          }
        }

        for (const auto& group : groups) {
          if (group.first) {
            for (const auto& vertex_index : group.second) {
              connectivity_graph.vertices[vertex_index].contour_index = number_of_contours;
            }
            number_of_contours++;
          }
        }
      } else {
        vu.vertex_type = ConnectivityGraph::V::VType::Default;
      }
    }
    float max_distance = 0.f;
    // Establish connected component for current group.

    ConnectivityGraph::ConnectedComponent component{};
    component.index = connectivity_graph.connected_components.size();
    component.source_index = seed_vertex_index;
    for (VIndex vertex_index = 0; vertex_index < vertices.size(); vertex_index++) {
      if (updated[vertex_index]) {
        max_distance = glm::max(max_distance, connectivity_graph.vertices[vertex_index].distance_to_source_point);
        component.component_vertices.emplace_back(vertex_index);
        component.vertices_set.insert(vertex_index);
      }
    }

    std::vector<glm::vec3> contour_colors;
    contour_colors.resize(number_of_contours);
    for (auto& contour_color : contour_colors) {
      contour_color = glm::abs(glm::sphericalRand(1.f));
    }
    for (const auto& vertex_index : component.component_vertices) {
      auto& v = connectivity_graph.vertices[vertex_index];
      v.source_index = seed_vertex_index;
      v.connected_component_id = component.index;

      switch (connectivity_graph.vertices[vertex_index].vertex_type) {
        case ConnectivityGraph::V::VType::Default:
          vertices[vertex_index].color = glm::vec4(1.f);
          break;
        case ConnectivityGraph::V::VType::LocalExtremum:
          vertices[vertex_index].color = glm::vec4(1.f, 0.f, 0.f, 1.f);
          break;
        case ConnectivityGraph::V::VType::SaddlePoint:
          vertices[vertex_index].color = glm::vec4(0.f, 0.f, 1.f, 1.f);
          break;
      }
      // vertices[vertex_index].color =
      //     glm::vec4(glm::vec3(glm::mod(v.distance_to_source_point / max_distance * 20.f, 1.f)), 1.f);
      vertices[vertex_index].color = glm::vec4(contour_colors[v.contour_index], 1.f);
    }
    if (!component.component_vertices.empty())
      connectivity_graph.connected_components.emplace_back(std::move(component));
  }
  EVOENGINE_LOG("Distance calculation finished!")

  return ret_val;
}
