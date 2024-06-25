#include "SorghumState.hpp"

#include "IVolume.hpp"
#include "SorghumLayer.hpp"
using namespace eco_sys_lab;

bool SorghumMeshGeneratorSettings::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  if (ImGui::TreeNode("Sorghum mesh generator settings")) {
    ImGui::Checkbox("Panicle", &m_enablePanicle);
    ImGui::Checkbox("Stem", &m_enableStem);
    ImGui::Checkbox("Leaves", &m_enableLeaves);
    ImGui::Checkbox("Bottom Face", &m_bottomFace);
    ImGui::Checkbox("Leaf separated", &m_leafSeparated);
    ImGui::DragFloat("Leaf thickness", &m_leafThickness, 0.0001f);
    ImGui::TreePop();
  }
  return false;
}

bool SorghumPanicleState::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  return false;
}

void SorghumPanicleState::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "m_panicleSize" << YAML::Value << m_panicleSize;
  out << YAML::Key << "m_seedAmount" << YAML::Value << m_seedAmount;
  out << YAML::Key << "m_seedRadius" << YAML::Value << m_seedRadius;
}

void SorghumPanicleState::Deserialize(const YAML::Node& in) {
  if (in["m_panicleSize"])
    m_panicleSize = in["m_panicleSize"].as<glm::vec3>();
  if (in["m_seedAmount"])
    m_seedAmount = in["m_seedAmount"].as<int>();
  if (in["m_seedRadius"])
    m_seedRadius = in["m_seedRadius"].as<float>();
}

void SorghumPanicleState::GenerateGeometry(const glm::vec3& stem_tip, std::vector<Vertex>& vertices,
                                           std::vector<unsigned>& indices) const {
  std::vector<glm::vec3> icosahedron_vertices;
  std::vector<glm::uvec3> icosahedron_triangles;
  SphereMeshGenerator::Icosahedron(icosahedron_vertices, icosahedron_triangles);
  int offset = 0;
  Vertex archetype = {};
  SphericalVolume volume;
  volume.m_radius = m_panicleSize;
  for (int seed_index = 0; seed_index < m_seedAmount; seed_index++) {
    glm::vec3 position_offset = volume.GetRandomPoint();
    for (const auto position : icosahedron_vertices) {
      archetype.position = position * m_seedRadius + glm::vec3(0, m_panicleSize.y, 0) + position_offset + stem_tip;
      vertices.push_back(archetype);
    }
    for (const auto triangle : icosahedron_triangles) {
      glm::uvec3 actual_triangle = triangle + glm::uvec3(offset);
      indices.emplace_back(actual_triangle.x);
      indices.emplace_back(actual_triangle.y);
      indices.emplace_back(actual_triangle.z);
    }
    offset += icosahedron_vertices.size();
  }
}

void SorghumPanicleState::GenerateGeometry(const glm::vec3& stem_tip, std::vector<Vertex>& vertices,
                                           std::vector<unsigned>& indices,
                                           const std::shared_ptr<ParticleInfoList>& particle_info_list) const {
  std::vector<glm::vec3> icosahedron_vertices;
  std::vector<glm::uvec3> icosahedron_triangles;
  SphereMeshGenerator::Icosahedron(icosahedron_vertices, icosahedron_triangles);
  Vertex archetype = {};
  archetype.color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
  for (const auto position : icosahedron_vertices) {
    archetype.position = position;
    vertices.push_back(archetype);
  }
  for (const auto triangle : icosahedron_triangles) {
    glm::uvec3 actual_triangle = triangle;
    indices.emplace_back(actual_triangle.x);
    indices.emplace_back(actual_triangle.y);
    indices.emplace_back(actual_triangle.z);
  }
  std::vector<ParticleInfo> infos;
  infos.resize(m_seedAmount);
  SphericalVolume volume;
  volume.m_radius = m_panicleSize;

  for (int seed_index = 0; seed_index < m_seedAmount; seed_index++) {
    glm::vec3 position_offset = volume.GetRandomPoint();
    glm::vec3 position = glm::vec3(0, m_panicleSize.y, 0) + position_offset + stem_tip;
    infos.at(seed_index).instance_matrix.value = glm::translate(position) * glm::scale(glm::vec3(m_seedRadius));
  }

  particle_info_list->SetParticleInfos(infos);
}

bool SorghumStemState::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  return false;
}

void SorghumStemState::Serialize(YAML::Emitter& out) const {
}

void SorghumStemState::Deserialize(const YAML::Node& in) {
}

void SorghumStemState::GenerateGeometry(std::vector<Vertex>& vertices, std::vector<unsigned>& indices) const {
  if (m_spline.m_segments.empty())
    return;
  auto sorghum_layer = Application::GetLayer<SorghumLayer>();
  if (!sorghum_layer)
    return;
  std::vector<SorghumSplineSegment> segments;
  m_spline.Subdivide(sorghum_layer->vertical_subdivision_length, segments);

  const int vertex_index = vertices.size();
  Vertex archetype{};
  glm::vec4 m_vertex_color = glm::vec4(0, 0, 0, 1);
  archetype.color = m_vertex_color;

  const float x_step = 1.0f / sorghum_layer->horizontal_subdivision_step / 2.0f;
  auto segment_size = segments.size();
  const float y_stem_step = 0.5f / segment_size;
  for (int i = 0; i < segment_size; i++) {
    auto& segment = segments.at(i);
    if (i <= segment_size / 3) {
      archetype.color = glm::vec4(1, 0, 0, 1);
    } else if (i <= segment_size * 2 / 3) {
      archetype.color = glm::vec4(0, 1, 0, 1);
    } else {
      archetype.color = glm::vec4(0, 0, 1, 1);
    }
    const float angle_step = segment.m_theta / sorghum_layer->horizontal_subdivision_step;
    const int verts_count = sorghum_layer->horizontal_subdivision_step * 2 + 1;
    for (int j = 0; j < verts_count; j++) {
      const auto position = segment.GetStemPoint((j - sorghum_layer->horizontal_subdivision_step) * angle_step);
      archetype.position = glm::vec3(position.x, position.y, position.z);
      float y_pos = y_stem_step * i;
      archetype.tex_coord = glm::vec2(j * x_step, y_pos);
      vertices.push_back(archetype);
    }
    if (i != 0) {
      for (int j = 0; j < verts_count - 1; j++) {
        // Down triangle
        indices.emplace_back(vertex_index + ((i - 1) + 1) * verts_count + j);
        indices.emplace_back(vertex_index + (i - 1) * verts_count + j + 1);
        indices.emplace_back(vertex_index + (i - 1) * verts_count + j);
        // Up triangle
        indices.emplace_back(vertex_index + (i - 1) * verts_count + j + 1);
        indices.emplace_back(vertex_index + ((i - 1) + 1) * verts_count + j);
        indices.emplace_back(vertex_index + ((i - 1) + 1) * verts_count + j + 1);
      }
    }
  }
}

bool SorghumLeafState::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  return false;
}

void SorghumLeafState::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "m_index" << YAML::Value << m_index;
}

void SorghumLeafState::Deserialize(const YAML::Node& in) {
  if (in["m_index"])
    m_index = in["m_index"].as<int>();
}

void SorghumLeafState::GenerateGeometry(std::vector<Vertex>& vertices, std::vector<unsigned>& indices, bool bottom_face,
                                        float thickness) const {
  if (m_spline.m_segments.empty())
    return;
  auto sorghum_layer = Application::GetLayer<SorghumLayer>();
  if (!sorghum_layer)
    return;
  std::vector<SorghumSplineSegment> segments;  // = m_spline.m_segments;
  m_spline.Subdivide(sorghum_layer->vertical_subdivision_length, segments);

  const int vertex_index = vertices.size();
  Vertex archetype{};
#pragma region Semantic mask color
  auto index = m_index + 1;
  const auto vertex_color = glm::vec4(index % 3 * 0.5f, index / 3 % 3 * 0.5f, index / 9 % 3 * 0.5f, 1.0f);
#pragma endregion
  archetype.color = vertex_color;
  archetype.vertex_info1 = m_index + 1;
  const float x_step = 1.0f / static_cast<float>(sorghum_layer->horizontal_subdivision_step) / 2.0f;
  auto segment_size = segments.size();
  const float y_leaf_step = 0.5f / segment_size;

  for (int i = 0; i < segment_size; i++) {
    auto& segment = segments.at(i);
    const float angle_step = segment.m_theta / static_cast<float>(sorghum_layer->horizontal_subdivision_step);
    const int verts_count = sorghum_layer->horizontal_subdivision_step * 2 + 1;
    for (int j = 0; j < verts_count; j++) {
      auto position = segment.GetLeafPoint((j - sorghum_layer->horizontal_subdivision_step) * angle_step);
      auto normal = segment.GetNormal((j - sorghum_layer->horizontal_subdivision_step) * angle_step);
      if (i != 0 && j != 0 && j != verts_count - 1) {
        position -= normal * thickness;
      }
      archetype.position = glm::vec3(position.x, position.y, position.z);
      float y_pos = 0.5f + y_leaf_step * i;
      archetype.tex_coord = glm::vec2(j * x_step, y_pos);
      vertices.push_back(archetype);
    }
    if (i != 0) {
      for (int j = 0; j < verts_count - 1; j++) {
        if (bottom_face) {
          // Down triangle
          indices.emplace_back(vertex_index + i * verts_count + j);
          indices.emplace_back(vertex_index + (i - 1) * verts_count + j + 1);
          indices.emplace_back(vertex_index + (i - 1) * verts_count + j);
          // Up triangle
          indices.emplace_back(vertex_index + (i - 1) * verts_count + j + 1);
          indices.emplace_back(vertex_index + i * verts_count + j);
          indices.emplace_back(vertex_index + i * verts_count + j + 1);
        } else {
          // Down triangle
          indices.emplace_back(vertex_index + (i - 1) * verts_count + j);
          indices.emplace_back(vertex_index + (i - 1) * verts_count + j + 1);
          indices.emplace_back(vertex_index + i * verts_count + j);
          // Up triangle
          indices.emplace_back(vertex_index + i * verts_count + j + 1);
          indices.emplace_back(vertex_index + i * verts_count + j);
          indices.emplace_back(vertex_index + (i - 1) * verts_count + j + 1);
        }
      }
    }
  }
}

bool SorghumState::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNodeEx((std::string("Stem")).c_str())) {
    if (m_stem.OnInspect(editor_layer))
      changed = true;
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Leaves")) {
    int leaf_size = m_leaves.size();
    if (ImGui::InputInt("Number of leaves", &leaf_size)) {
      changed = true;
      leaf_size = glm::clamp(leaf_size, 0, 999);
      const auto previous_size = m_leaves.size();
      m_leaves.resize(leaf_size);
      for (int i = 0; i < leaf_size; i++) {
        if (i >= previous_size) {
          if (i - 1 >= 0) {
            m_leaves[i] = m_leaves[i - 1];
            /*
            m_leaves[i].m_rollAngle =
                    glm::mod(m_leaves[i - 1].m_rollAngle + 180.0f, 360.0f);
            m_leaves[i].m_startingPoint =
                    m_leaves[i - 1].m_startingPoint + 0.1f;*/
          } else {
            m_leaves[i] = {};
            /*
            m_leaves[i].m_rollAngle = 0;
            m_leaves[i].m_startingPoint = 0.1f;*/
          }
        }
        m_leaves[i].m_index = i;
      }
    }
    for (auto& leaf : m_leaves) {
      if (ImGui::TreeNode(
              ("Leaf No." + std::to_string(leaf.m_index + 1) + (leaf.m_spline.m_segments.empty() ? " (Dead)" : ""))
                  .c_str())) {
        if (leaf.OnInspect(editor_layer))
          changed = true;
        ImGui::TreePop();
      }
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx((std::string("Panicle")).c_str())) {
    if (m_panicle.OnInspect(editor_layer))
      changed = true;
    ImGui::TreePop();
  }

  return changed;
}

void SorghumState::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "m_panicle" << YAML::Value << YAML::BeginMap;
  m_panicle.Serialize(out);
  out << YAML::EndMap;
  out << YAML::Key << "m_stem" << YAML::Value << YAML::BeginMap;
  m_stem.Serialize(out);
  out << YAML::EndMap;

  if (!m_leaves.empty()) {
    out << YAML::Key << "m_leaves" << YAML::Value << YAML::BeginSeq;
    for (auto& i : m_leaves) {
      out << YAML::BeginMap;
      i.Serialize(out);
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
}

void SorghumState::Deserialize(const YAML::Node& in) {
  if (in["m_panicle"])
    m_panicle.Deserialize(in["m_panicle"]);

  if (in["m_stem"])
    m_stem.Deserialize(in["m_stem"]);

  if (in["m_leaves"]) {
    for (const auto& i : in["m_leaves"]) {
      SorghumLeafState leaf_state{};
      leaf_state.Deserialize(i);
      m_leaves.push_back(leaf_state);
    }
  }
}
