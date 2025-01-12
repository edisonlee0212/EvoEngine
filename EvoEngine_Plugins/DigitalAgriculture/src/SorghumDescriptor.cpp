#include "SorghumDescriptor.hpp"

#include "IVolume.hpp"
#include "Sorghum.hpp"
#include "SorghumLayer.hpp"
#include "assimp/code/AssetLib/3MF/3MFXmlTags.h"
using namespace digital_agriculture_plugin;

bool SorghumMeshGeneratorSettings::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  if (ImGui::TreeNode("Sorghum mesh generator settings")) {
    ImGui::Checkbox("Panicle", &enable_panicle);
    ImGui::Checkbox("Stem", &enable_stem);
    ImGui::Checkbox("Leaves", &enable_leaves);
    if (enable_leaves) {
      ImGui::Checkbox("Leaves sheath", &enable_leaf_sheath);
    }
    ImGui::Checkbox("Bottom Face", &bottom_face);
    ImGui::Checkbox("Leaf separated", &leaf_separated);
    ImGui::DragFloat("Leaf thickness", &leaf_thickness, 0.0001f);
    ImGui::TreePop();
  }
  return false;
}

bool SorghumPanicleDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  return false;
}

void SorghumPanicleDescriptor::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "panicle_size" << YAML::Value << panicle_size;
  out << YAML::Key << "seed_amount" << YAML::Value << seed_amount;
  out << YAML::Key << "seed_radius" << YAML::Value << seed_radius;
}

void SorghumPanicleDescriptor::Deserialize(const YAML::Node& in) {
  if (in["panicle_size"])
    panicle_size = in["panicle_size"].as<glm::vec3>();
  if (in["seed_amount"])
    seed_amount = in["seed_amount"].as<int>();
  if (in["seed_radius"])
    seed_radius = in["seed_radius"].as<float>();
}

void SorghumPanicleDescriptor::GenerateGeometry(const glm::vec3& stem_tip, std::vector<Vertex>& vertices,
                                                std::vector<unsigned>& indices) const {
  std::vector<glm::vec3> icosahedron_vertices;
  std::vector<glm::uvec3> icosahedron_triangles;
  SphereMeshGenerator::Icosahedron(icosahedron_vertices, icosahedron_triangles);
  int offset = 0;
  Vertex archetype = {};
  eco_sys_lab_plugin::SphericalVolume volume;
  volume.m_radius = panicle_size;
  for (int seed_index = 0; seed_index < seed_amount; seed_index++) {
    glm::vec3 position_offset = volume.GetRandomPoint();
    for (const auto position : icosahedron_vertices) {
      archetype.position = position * seed_radius + glm::vec3(0, panicle_size.y, 0) + position_offset + stem_tip;
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

void SorghumPanicleDescriptor::GenerateGeometry(const glm::vec3& stem_tip, std::vector<Vertex>& vertices,
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
  infos.resize(seed_amount);
  eco_sys_lab_plugin::SphericalVolume volume;
  volume.m_radius = panicle_size;

  for (int seed_index = 0; seed_index < seed_amount; seed_index++) {
    glm::vec3 position_offset = volume.GetRandomPoint();
    glm::vec3 position = glm::vec3(0, panicle_size.y, 0) + position_offset + stem_tip;
    infos.at(seed_index).instance_matrix.value = glm::translate(position) * glm::scale(glm::vec3(seed_radius));
  }

  particle_info_list->SetParticleInfos(infos);
}

bool SorghumStemDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  return false;
}

void SorghumStemDescriptor::Serialize(YAML::Emitter& out) const {
  spline.Serialize("spline", out);
}

void SorghumStemDescriptor::Deserialize(const YAML::Node& in) {
  spline.Deserialize("spline", in);
}

void SorghumStemDescriptor::GenerateGeometry(std::vector<Vertex>& vertices, std::vector<unsigned>& indices) const {
  if (spline.segments.empty())
    return;
  auto sorghum_layer = Application::GetLayer<SorghumLayer>();
  if (!sorghum_layer)
    return;
  std::vector<SorghumSplineSegment> segments;
  spline.SubdivideByDistance(sorghum_layer->vertical_subdivision_length, segments);

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
    const float angle_step = segment.theta / sorghum_layer->horizontal_subdivision_step;
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

bool SorghumLeafDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  return false;
}

void SorghumLeafDescriptor::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "index" << YAML::Value << index;
  spline.Serialize("spline", out);
}

void SorghumLeafDescriptor::Deserialize(const YAML::Node& in) {
  if (in["index"])
    index = in["index"].as<int>();
  spline.Deserialize("spline", in);
}

void SorghumLeafDescriptor::GenerateGeometry(std::vector<Vertex>& vertices, std::vector<unsigned>& indices,
                                             const SorghumMeshGeneratorSettings& mesh_generator_settings,
                                             bool current_bottom_face) const {
  if (spline.segments.empty())
    return;
  auto sorghum_layer = Application::GetLayer<SorghumLayer>();
  if (!sorghum_layer)
    return;
  std::vector<SorghumSplineSegment> segments;  // = spline.segments;
  SorghumSpline temp_spline;
  spline.SubdivideByDistance(sorghum_layer->vertical_subdivision_length, temp_spline.segments);
  if (mesh_generator_settings.enable_leaf_sheath) {
    segments = temp_spline.segments;
  } else {
    segments = temp_spline.GetLeafPart();
  }
  const int vertex_index = vertices.size();
  Vertex archetype{};
#pragma region Semantic mask color
  const uint32_t actual_index = this->index + 1;
  const auto vertex_color =
      glm::vec4(actual_index % 3 * 0.5f, actual_index / 3 % 3 * 0.5f, actual_index / 9 % 3 * 0.5f, 1.0f);
#pragma endregion
  archetype.color = vertex_color;
  archetype.vertex_info1 = glm::uintBitsToFloat(actual_index);
  const float x_step = 1.0f / static_cast<float>(sorghum_layer->horizontal_subdivision_step) / 2.0f;
  auto segment_size = segments.size();
  const float y_leaf_step = 0.5f / segment_size;

  for (int i = 0; i < segment_size; i++) {
    auto& segment = segments.at(i);
    const float angle_step = segment.theta / static_cast<float>(sorghum_layer->horizontal_subdivision_step);
    const int verts_count = sorghum_layer->horizontal_subdivision_step * 2 + 1;
    for (int j = 0; j < verts_count; j++) {
      auto position =
          segment.GetLeafPoint((j - static_cast<float>(sorghum_layer->horizontal_subdivision_step)) * angle_step);
      auto normal =
          segment.GetNormal((j - static_cast<float>(sorghum_layer->horizontal_subdivision_step)) * angle_step);
      if (i != 0 && j != 0 && j != verts_count - 1) {
        position -= normal * mesh_generator_settings.leaf_thickness;
      }
      archetype.position = glm::vec3(position.x, position.y, position.z);
      float y_pos = 0.5f + y_leaf_step * i;
      archetype.tex_coord = glm::vec2(j * x_step, y_pos);
      vertices.push_back(archetype);
    }
    if (i != 0) {
      for (int j = 0; j < verts_count - 1; j++) {
        if (current_bottom_face) {
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
struct CubicBezierPoint {
  glm::vec3 position;
  // the third control point for the preceding bezier
  glm::vec3 left_handle;

  // the second control point for the next bezier
  glm::vec3 right_handle;

  bool IsC1Continuity() const{
    return right_handle - position == position - left_handle;
  }
};

// position = (Strands::CubicInterpolation(joints[i].position, joints[i].right_handle, joints[i + 1].left_handle, joints[i + 1].position, t))
struct SplineSample {
  int segmentIndex;
  float t;
  glm::vec3 position;
};
struct CubicBezierSpline {
  // n+1 joints -> n segments -> 3n+1 control points
  std::vector<CubicBezierPoint> joints;

  // recording segmentLengths;
  std::vector<float> segmentLengths;

  // get length of the whole spline and update segmentLengths
  float getLength() {
    float result = 0;
    segmentLengths.clear();
    for (int i = 0; i < joints.size() - 1; i++) {
      float segmentLength = Strands::CalculateLengthAdaptive<glm::vec3>(
          joints[i].position, joints[i].right_handle, joints[i + 1].left_handle, joints[i + 1].position);
      result += segmentLength;
      segmentLengths.push_back(segmentLength);
    }
    return result;
  }

  // return uniform sample points on the spline
  std::vector<SplineSample> getUniformSamples(int num) {
    std::vector<SplineSample> samples;
    float length = getLength();
    float lengthPerSample = length / (num + 1);
    float sampleLength = lengthPerSample;
    float currentLength = 0;
    for (int i = 0; i < segmentLengths.size(); i++) {
      SplineSample s;
      while (currentLength < sampleLength) {
        currentLength += segmentLengths[i++];

      }
      i--;
      float t = Strands::FindTAdaptive(joints[i].position, joints[i].right_handle, joints[i + 1].left_handle,
                                       joints[i + 1].position, 0, sampleLength - currentLength + segmentLengths[i]);
      glm::vec3 pos = (Strands::CubicInterpolation(joints[i].position, joints[i].right_handle,
                                                    joints[i + 1].left_handle, joints[i + 1].position, t));
      s.position = pos;
      s.segmentIndex = i;
      s.t = t;
      samples.push_back(s);

      sampleLength += lengthPerSample;
    }

    assert(samples.size() == num);
    return samples;
  }

  // return intersection of the spline and a plane
  [[nodiscard]] std::vector<glm::vec3> getSurfaceIntersection(const glm::vec3& planePoint, const glm::vec3& normal) const {

    // todo: how to handle multiple intersections?
    std::vector<glm::vec3> results;
    for (int i = 0; i < joints.size() - 1; i++) {
      glm::vec3 p0 = joints[i].position;
      glm::vec3 p1 = joints[i].right_handle;
      glm::vec3 p2 = joints[i + 1].left_handle;
      glm::vec3 p3 = joints[i + 1].position;

      // assume each of the bezier curves should only intersect with the plane once
      float t = bisectionMethod(0.0f, 1.0f, p0, p1, p2, p3, planePoint, normal);

      t = newtonMethod(t, p0, p1, p2, p3, planePoint, normal);

      glm::vec3 intersectionPoint = Strands::CubicInterpolation(p0, p1, p2, p3, t);
      results.push_back(intersectionPoint);
    }
    return results;

  }

  static float planeEquation(const glm::vec3& point, const glm::vec3& normal, const glm::vec3& planePoint) {
    return glm::dot(normal, point - planePoint);
  }

  // bisection for t
  static float bisectionMethod(float t_min, float t_max, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
                        const glm::vec3& v3, const glm::vec3& planePoint, const glm::vec3& normal,
                        float epsilon = 0.0001f) {
    float t_mid;
    while (t_max - t_min > epsilon) {
      t_mid = (t_min + t_max) / 2.0f;
      glm::vec3 bezierPoint = Strands::CubicInterpolation(v0, v1, v2, v3, t_mid);
      float value = planeEquation(bezierPoint, normal, planePoint);
      if (value > 0) {
        t_max = t_mid; 
      } else {
        t_min = t_mid;  
      }
    }
    return (t_min + t_max) / 2.0f;
  }

  // newton method for intersection t
  static float newtonMethod(float t, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3,
                     const glm::vec3& planePoint, const glm::vec3& normal, float epsilon = 0.0001f, int maxIter = 100) {
    for (int i = 0; i < maxIter; ++i) {
      glm::vec3 bezierPoint = Strands::CubicInterpolation(v0, v1, v2, v3, t);
      float value = planeEquation(bezierPoint, normal, planePoint);
      if (std::abs(value) < epsilon) {
        return t;  // 找到交点
      }

      // 计算切线（导数）
      glm::vec3 bezierDerivative = getTangent(v0, v1, v2, v3, t);
      float derivative = glm::dot(normal, bezierDerivative);  // 计算平面方程的导数
      if (std::abs(derivative) < epsilon) {
        break;
      }
      t -= value / derivative;        // 使用牛顿法更新 t
      t = std::clamp(t, 0.0f, 1.0f);  // 保证 t 在 [0, 1] 范围内
    }
    return t;
  }


  // return the tangent of a point on the spline
  [[nodiscard]] glm::vec3 getTangent(const SplineSample& sample) const {
    glm::vec3 p0 = joints[sample.segmentIndex].position;
    glm::vec3 p1 = joints[sample.segmentIndex].right_handle;
    glm::vec3 p2 = joints[sample.segmentIndex + 1].left_handle;
    glm::vec3 p3 = joints[sample.segmentIndex + 1].position;
    float t = sample.t;
    return getTangent(p0, p1, p2, p3, t);
     
  }


  static glm::vec3 getTangent(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3, float t) {
    return normalize(3.0f * (1.0f - t) * (1.0f - t) * (v1 - v0) + 6.0f * (1.0f - t) * t * (v2 - v1) + 3.0f * t * t * (v3 - v2));
  }
  
};


// yaml content to splines
[[nodiscard]]std::vector<std::unordered_map<std::string, CubicBezierSpline>> ReconstructBezierSplineFromYAML(
    std::vector<std::unordered_map<std::string, std::vector<glm::vec3>>>& yaml_content) {
  if (yaml_content.empty()) {
    EVOENGINE_ERROR("Empty yaml loaded")
  }
  std::vector<std::unordered_map<std::string, CubicBezierSpline>> bezierSplines;

  std::vector<std::string> keys = {"leftPoints", "rightPoints", "centerPoints"};

  // iteration for leaves
  for (int i = 0; i < yaml_content.size(); i++) {
    std::unordered_map<std::string, CubicBezierSpline> leafRepresentation;

    auto& leaf = yaml_content[i];

    // iteration for lines
    for (int j = 0; j < keys.size(); j++) {
      CubicBezierSpline bezierSpline;

      // iteration for points
      for (int k = 0; k < leaf["centerPoints"].size(); k++) {
        std::vector<glm::vec3> points = leaf[keys[j]];

        // generate CubicBezierSpline from the points
        glm::vec3 front;
        if (k == leaf[keys[j]].size() - 1) {
          front = glm::normalize((points[k] - points[k - 1]));
        } else {
          front = glm::normalize((points[k + 1] - points[k]));
        }

        CubicBezierPoint p;
        p.position = points[k];
        // C1 continuity
        p.right_handle = p.position + front * 0.25f;
        p.left_handle = p.position - front * 0.25f;

        bezierSpline.joints.push_back(p);
      }

      leafRepresentation.insert(std::make_pair(keys[j], bezierSpline));
    }

    bezierSplines.emplace_back(leafRepresentation);
  }

  return bezierSplines;
}

// splines to sorghum
void ReconstructSorghumFromBezierSplines(const std::shared_ptr<SorghumDescriptor>& sorghum_descriptor,
                                    const std::vector<std::unordered_map<std::string, CubicBezierSpline>>& bezierSplines,
                                    float theta, float scale) {
  std::vector<std::string> keys = {"leftPoints", "rightPoints", "centerPoints"};
  for (int i = 0; i < bezierSplines.size(); i++) {
    auto leafSplines = bezierSplines[i];

    SorghumSpline sorghumSpline;
    auto leftLine = leafSplines[keys[0]];
    auto rightLine = leafSplines[keys[1]];
    auto centerLine = leafSplines[keys[2]];

    // get uniform samples from the centerLine
    auto samples = centerLine.getUniformSamples(32);

    // reconstruct the local coordinate at each of the samples
    for(SplineSample sample : samples) {
      SorghumSplineSegment segment;

      // position
      segment.position = sample.position;

      // front
      segment.front = centerLine.getTangent(sample);

      // get the intersections of the left and right lines on the profile
      glm::vec3 position = sample.position;
      glm::vec3 normal = segment.front;
      // todo: assume the closest intersection point should be the right one
      std::vector<glm::vec3> leftCandidates = leftLine.getSurfaceIntersection(position, normal);
      auto leftIntersection = std::min_element(leftCandidates.begin(), leftCandidates.end(),
                                   [&position](const glm::vec3& a, const glm::vec3& b) {
                                     return glm::distance(a, position) < glm::distance(b, position);
      });
      assert(leftIntersection != leftCandidates.end());

      std::vector<glm::vec3> rightCandidates = leftLine.getSurfaceIntersection(position, normal);
      auto rightIntersection = std::min_element(rightCandidates.begin(), rightCandidates.end(),
                                   [&position](const glm::vec3& a, const glm::vec3& b) {
                                     return glm::distance(a, position) < glm::distance(b, position);
                                   });
      assert(rightIntersection != rightCandidates.end());

      // from the profile, reconstruct the up vector
      glm::vec3 rightLocal = *rightIntersection - sample.position;
      glm::vec3 leftLocal = *leftIntersection - sample.position;

      glm::vec3 right = glm::cross(segment.front, glm::vec3(0, 1, 0));
      right = glm::dot(right, rightLocal) < 0 ? -right : right;

      segment.up = normalize(glm::cross(right, segment.front));

      // reconstruct theta, radius, height_offsets
      segment.theta = theta;
      segment.radius = std::max(std::max(glm::length(leftLocal), glm::length(rightLocal)), 0.01f);

      segment.left_height_offset = leftLocal.y;
      segment.right_height_offset = rightLocal.y;

      sorghumSpline.segments.push_back(segment);
    }

    SorghumLeafDescriptor leaf_descriptor;
    leaf_descriptor.spline = sorghumSpline;
    leaf_descriptor.index = i;

    sorghum_descriptor->leaves.push_back(leaf_descriptor);
  }
    
}

bool SorghumDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  if (ImGui::Button("Instantiate")) {
    CreateEntity("New Sorghum");
  }
  // todo: after load from spline, replace data in sorghumdescriptor
  FileUtils::OpenFile(
      "Load splines", "YAML", {".yml"},
      [&](const std::filesystem::path& path) {
        ImportPrediction(path);
      },
      false);
  bool changed = false;
  if (ImGui::TreeNodeEx((std::string("Stem")).c_str())) {
    if (stem.OnInspect(editor_layer))
      changed = true;
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Leaves")) {
    int leaf_size = leaves.size();
    if (ImGui::InputInt("Number of leaves", &leaf_size)) {
      changed = true;
      leaf_size = glm::clamp(leaf_size, 0, 999);
      const auto previous_size = leaves.size();
      leaves.resize(leaf_size);
      for (int i = 0; i < leaf_size; i++) {
        if (i >= previous_size) {
          if (i - 1 >= 0) {
            leaves[i] = leaves[i - 1];
            /*
            leaves[i].m_rollAngle =
                    glm::mod(leaves[i - 1].m_rollAngle + 180.0f, 360.0f);
            leaves[i].m_startingPoint =
                    leaves[i - 1].m_startingPoint + 0.1f;*/
          } else {
            leaves[i] = {};
            /*
            leaves[i].m_rollAngle = 0;
            leaves[i].m_startingPoint = 0.1f;*/
          }
        }
        leaves[i].index = i;
      }
    }
    for (auto& leaf : leaves) {
      if (ImGui::TreeNode(
              ("Leaf No." + std::to_string(leaf.index + 1) + (leaf.spline.segments.empty() ? " (Dead)" : ""))
                  .c_str())) {
        if (leaf.OnInspect(editor_layer))
          changed = true;
        ImGui::TreePop();
      }
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx((std::string("Panicle")).c_str())) {
    if (panicle.OnInspect(editor_layer))
      changed = true;
    ImGui::TreePop();
  }

  //------------------------------------
  // load and reconstruct from yaml
  //------------------------------------

  // loaded yamlContent
  static std::vector<std::unordered_map<std::string, std::vector<glm::vec3>>> yamlContent;

  // reconstructed sorghum
  static std::shared_ptr<SorghumDescriptor> sorghum_descriptor =
      ProjectManager::CreateTemporaryAsset<SorghumDescriptor>();

  // use gizmo to visualize all the points
  static std::shared_ptr<ParticleInfoList> pi;
  static bool drawPoints;
  static float theta = 30.0f;
  static Entity reconstructed_entity;
  // enlarge the sorghum
  static float scale = 1.0f;
  Transform transform;
  transform.SetPosition({0, 0, 0});
  transform.SetEulerRotation(glm::radians(glm::vec3(0, 0, 0)));
  GizmoSettings gizmo_settings;
  gizmo_settings.draw_settings.blending = true;

  ImGui::Checkbox("Enable yaml Gizmo", &drawPoints);

  if (drawPoints) {
    editor_layer->DrawGizmoCubes(pi, 1.0f, 1, gizmo_settings);
  }
  if (ImGui::SliderFloat("theta", &theta, 0.0f, 180.0f)) {
    ReconstructFromYAML(sorghum_descriptor, yamlContent, theta, scale);
    // const auto scene = Application::GetActiveScene();
    // const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(reconstructed_entity).lock();
    // sorghum->ClearGeometryEntities();
    // sorghum->sorghum_descriptor = sorghum_descriptor->GetSelf();
    //   if (const auto sorghum_layer = Application::GetLayer<SorghumLayer>()) {

    //  sorghum_layer->sorghum_mesh_generator_settings.enable_stem = false;
    //  sorghum->GenerateGeometryEntities(sorghum_layer->sorghum_mesh_generator_settings);
    //} else {
    //  sorghum->GenerateGeometryEntities({});
    //}
    const auto scene = Application::GetActiveScene();
    scene->DeleteEntity(reconstructed_entity);
    reconstructed_entity = sorghum_descriptor->CreateEntity("Temp Sorghum");
  }

  if (ImGui::SliderFloat("scale", &scale, 1.0f, 10.0f)) {
    ReconstructFromYAML(sorghum_descriptor, yamlContent, theta, scale);
    const auto scene = Application::GetActiveScene();
    scene->DeleteEntity(reconstructed_entity);
    reconstructed_entity = sorghum_descriptor->CreateEntity("Temp Sorghum");

    // update the gizmo together
    int leafCount = yamlContent.size();
    int PointsCount = yamlContent[0]["centerPoints"].size();
    std::vector<ParticleInfo> particle_infos(leafCount * PointsCount * 3);
    Jobs::RunParallelFor(leafCount * PointsCount, [&](const auto i) {
      auto& center_info = particle_infos[i];
      center_info.instance_color = glm::vec4(256, 0, 0, 256) / 256.f;
      center_info.instance_matrix.SetPosition(glm::vec3(yamlContent[i / PointsCount]["centerPoints"][i % PointsCount]) *
                                              scale);

      center_info.instance_matrix.SetScale(glm::vec3(0.01f));

      auto& left_info = particle_infos[i + leafCount * PointsCount];
      left_info.instance_color = glm::vec4(0, 256, 0, 256) / 256.f;
      left_info.instance_matrix.SetPosition(glm::vec3(yamlContent[i / PointsCount]["leftPoints"][i % PointsCount]) *
                                            scale);

      left_info.instance_matrix.SetScale(glm::vec3(0.01f));

      auto& right_info = particle_infos[i + 2 * leafCount * PointsCount];
      right_info.instance_color = glm::vec4(0, 0, 256, 256) / 256.f;
      right_info.instance_matrix.SetPosition(glm::vec3(yamlContent[i / PointsCount]["rightPoints"][i % PointsCount]) *
                                             scale);

      right_info.instance_matrix.SetScale(glm::vec3(0.01f));
    });

    pi->SetParticleInfos(particle_infos);
  }
  // load points from yaml
  if (yamlContent.empty()) {
    std::filesystem::path yamlPath(
        "E:/Computer Graphics/spline.yml");
    if (std::filesystem::exists(yamlPath)) {
      auto tempResult = ImportPrediction(yamlPath);

      if (tempResult) {
        // todo: generate stem descriptor
        yamlContent = *tempResult;
        std::cout << "imported from yaml" << std::endl;
        auto splines = ReconstructBezierSplineFromYAML(yamlContent);
        ReconstructSorghumFromBezierSplines(sorghum_descriptor, splines, theta, scale);
        //ReconstructFromYAML(sorghum_descriptor, yamlContent, theta, scale);

        // draw sorghum
        reconstructed_entity = sorghum_descriptor->CreateEntity("Temp Sorghum");
      }
    }
  }

  // gizmo setup
  if (!pi && !yamlContent.empty()) {
    pi = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
    int leafCount = yamlContent.size();
    int PointsCount = yamlContent[0]["centerPoints"].size();
    std::vector<ParticleInfo> particle_infos(leafCount * PointsCount * 3);

    Jobs::RunParallelFor(leafCount * PointsCount, [&](const auto i) {
      auto& center_info = particle_infos[i];
      center_info.instance_color = glm::vec4(256, 0, 0, 256) / 256.f;
      center_info.instance_matrix.SetPosition(glm::vec3(yamlContent[i / PointsCount]["centerPoints"][i % PointsCount]) *
                                              scale);

      center_info.instance_matrix.SetScale(glm::vec3(0.01f));

      auto& left_info = particle_infos[i + leafCount * PointsCount];
      left_info.instance_color = glm::vec4(0, 256, 0, 256) / 256.f;
      left_info.instance_matrix.SetPosition(glm::vec3(yamlContent[i / PointsCount]["leftPoints"][i % PointsCount]) *
                                            scale);

      left_info.instance_matrix.SetScale(glm::vec3(0.01f));

      auto& right_info = particle_infos[i + 2 * leafCount * PointsCount];
      right_info.instance_color = glm::vec4(0, 0, 256, 256) / 256.f;
      right_info.instance_matrix.SetPosition(glm::vec3(yamlContent[i / PointsCount]["rightPoints"][i % PointsCount]) *
                                             scale);

      right_info.instance_matrix.SetScale(glm::vec3(0.01f));
    });

    pi->SetParticleInfos(particle_infos);
  }

  return changed;
}



void SorghumDescriptor::ReconstructFromYAML(
    const std::shared_ptr<SorghumDescriptor>& sorghum_descriptor,
    std::vector<std::unordered_map<std::string, std::vector<glm::vec3>>>& yaml_content, float theta, float scale) {
  if (yaml_content.empty()) {
    EVOENGINE_ERROR("Empty yaml loaded")
  }
  // clear previous data
  sorghum_descriptor->leaves.clear();

  // generate SorghumDescriptor from loaded content
  // SorghumSplineSegment -> SorghumSpline -> SorghumLeafDescriptor

  std::vector<std::string> keys = {"leftPoints", "rightPoints", "centerPoints"};

  // todo: switch to parallel mode
  for (int i = 0; i < yaml_content.size(); i++) {
    auto& leaf = yaml_content[i];
    // a leaf has a spline
    SorghumSpline spline;

    for (int j = 0; j < leaf["centerPoints"].size(); j++) {
      SorghumSplineSegment segment;

      glm::vec3 left_point = leaf["leftPoints"][j];
      glm::vec3 right_point = leaf["rightPoints"][j];
      glm::vec3 center_point = leaf["centerPoints"][j];

      left_point = scale * glm::vec4(left_point, 1.0f);
      right_point = scale * glm::vec4(right_point, 1.0f);
      center_point = scale * glm::vec4(center_point, 1.0f);

      glm::vec3 left_point_local = left_point - center_point;
      glm::vec3 right_point_local = right_point - center_point;

      segment.position = center_point;

      // todo: front vector might come from an interpolated curve

      // front: (0,0,-1)
      if (j == leaf["centerPoints"].size() - 1) {
        segment.front = glm::normalize((leaf["centerPoints"][j] - leaf["centerPoints"][j - 1]));
      } else {
        segment.front = glm::normalize((leaf["centerPoints"][j + 1] - leaf["centerPoints"][j]));
      }

      // right: (1,0,0)
      glm::vec3 right = glm::cross(segment.front, glm::vec3(0, 1, 0));

      // up: (0,1,0)
      segment.up = normalize(glm::cross(right, segment.front));

      // todo: may need revise
      // reconstruct theta;
      segment.theta = theta;

      // reconstruct radius, height_offsets
      //segment.radius = std::max(std::abs(left_point_local.x), std::abs(right_point_local.x));
      segment.radius = std::max(std::max(glm::length(left_point_local), glm::length(right_point_local)),0.01f);
      

      segment.left_height_offset = left_point_local.y;
      segment.right_height_offset = right_point_local.y;

      // append segment to spline
      spline.segments.push_back(segment);
    }

    // SorghumSpline -> SorghumLeafDescriptor
    SorghumLeafDescriptor leaf_descriptor;
    leaf_descriptor.spline = spline;
    leaf_descriptor.index = i;

    // SorghumLeafDescriptor -> SorghumDescriptor
    sorghum_descriptor->leaves.push_back(leaf_descriptor);
  }
}

void SorghumDescriptor::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "panicle" << YAML::Value << YAML::BeginMap;
  panicle.Serialize(out);
  out << YAML::EndMap;
  out << YAML::Key << "stem" << YAML::Value << YAML::BeginMap;
  stem.Serialize(out);
  out << YAML::EndMap;

  if (!leaves.empty()) {
    out << YAML::Key << "leaves" << YAML::Value << YAML::BeginSeq;
    for (auto& i : leaves) {
      out << YAML::BeginMap;
      i.Serialize(out);
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
}

void SorghumDescriptor::Deserialize(const YAML::Node& in) {
  if (in["panicle"])
    panicle.Deserialize(in["panicle"]);

  if (in["stem"])
    stem.Deserialize(in["stem"]);

  if (in["leaves"]) {
    for (const auto& i : in["leaves"]) {
      SorghumLeafDescriptor leaf_state{};
      leaf_state.Deserialize(i);
      leaves.push_back(leaf_state);
    }
  }
}

Entity SorghumDescriptor::CreateEntity(const std::string& name) const {
  const auto scene = Application::GetActiveScene();
  const auto sorghum_entity = scene->CreateEntity(name);
  const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock();
  sorghum->sorghum_descriptor = GetSelf();

  if (const auto sorghum_layer = Application::GetLayer<SorghumLayer>()) {
    // todo: delete the test setting
    sorghum_layer->sorghum_mesh_generator_settings.enable_stem = false;
    sorghum->GenerateGeometryEntities(sorghum_layer->sorghum_mesh_generator_settings);
  } else {
    sorghum->GenerateGeometryEntities({});
  }

  return sorghum_entity;
}

std::shared_ptr<Texture2D> SorghumDescriptor::GenerateThumbnailTexture() {
  static std::shared_ptr<Texture2D> thumbnail;
  if (!thumbnail) {
    thumbnail = AssetManager::CreateTemporaryAsset<Texture2D>();
    thumbnail->Import(std::filesystem::absolute(std::filesystem::path("./DigitalAgricultureResources") /
                                                "Icons/SorghumDescriptor.png"));
  }
  return thumbnail;
}

void SorghumDescriptor::ImportPrediction(const std::filesystem::path& yaml_path) {
//  TODO: just load YAML, need to reconstruct Sorghum from loaded data
std::optional<std::vector<std::unordered_map<std::string, std::vector<glm::vec3>>>> SorghumDescriptor::ImportPrediction(
    const std::filesystem::path& yaml_path) {
  if (!std::filesystem::exists(yaml_path)) {
    EVOENGINE_ERROR("File not exist!")
    return std::nullopt;
  }
  try {
    const std::ifstream stream(yaml_path.string());
    std::stringstream string_stream;
    string_stream << stream.rdbuf();
    const YAML::Node in = YAML::Load(string_stream.str());

    std::vector<std::unordered_map<std::string, std::vector<glm::vec3>>> results;

    if (in["Sorghum"]) {
      const auto& sorghum_in = in["Sorghum"];
      for (const auto sorghum_real_in : sorghum_in) {
        if (sorghum_real_in["Leaves"]) {
          const auto& leaves_in = sorghum_real_in["Leaves"];
          for (const auto& leaf_in : leaves_in) {
            std::vector<glm::vec3> center_points;
            std::vector<glm::vec3> left_points;
            std::vector<glm::vec3> right_points;
            int leaf_index;
            std::unordered_map<std::string, std::vector<glm::vec3>> leafInfo;

            if (leaf_in["Center Points"]) {
              const auto& center_points_in = leaf_in["Center Points"];
              for (const auto& point_in : center_points_in) {
                glm::vec3 point;
                int i = 0;
                for (const auto& number : point_in) {
                  point[i] = number.as<float>();
                  i++;
                }
                center_points.emplace_back(point);
              }

              leafInfo.insert(std::make_pair("centerPoints", center_points));
            }
            if (leaf_in["Left Points"]) {
              const auto& left_points_in = leaf_in["Left Points"];
              for (const auto& point_in : left_points_in) {
                glm::vec3 point;
                int i = 0;
                for (const auto& number : point_in) {
                  point[i] = number.as<float>();
                  i++;
                }
                left_points.emplace_back(point);
              }

              leafInfo.insert(std::make_pair("leftPoints", left_points));
            }
            if (leaf_in["Right Points"]) {
              const auto& right_points_in = leaf_in["Right Points"];
              for (const auto& point_in : right_points_in) {
                glm::vec3 point;
                int i = 0;
                for (const auto& number : point_in) {
                  point[i] = number.as<float>();
                  i++;
                }
                right_points.emplace_back(point);
              }

              leafInfo.insert(std::make_pair("rightPoints", right_points));
            }
            if (leaf_in["Leaf Index"]) {
              leaf_index = leaf_in["Leaf Index"].as<int>();

              leafInfo.insert(std::make_pair("leafIndex", std::vector<glm::vec3>(leaf_index)));
            }

            results.emplace_back(leafInfo);
          }
        }
      }
    }
    return std::make_optional(results);

  } catch (const std::exception& e) {
    EVOENGINE_ERROR("Failed to load!")
    return std::nullopt;
  }
  return std::nullopt;
}
