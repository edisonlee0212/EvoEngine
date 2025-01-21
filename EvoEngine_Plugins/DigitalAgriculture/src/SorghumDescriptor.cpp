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
  bool changed = false;
  for (int i = 0; i < spline.segments.size(); i++) {
    auto segment = spline.segments[i];
    std::string label = "segment No." + std::to_string(i);
    if (ImGui::TreeNode(label.c_str())) {
      ImGui::Text("position: (%.2f, %.2f, %.2f)", segment.position.x, segment.position.y, segment.position.z);
      ImGui::Text("up: (%.2f, %.2f, %.2f)", segment.up.x, segment.up.y, segment.up.z);
      ImGui::Text("front: (%.2f, %.2f, %.2f)", segment.front.x, segment.front.y, segment.front.z);
      ImGui::Text("radius: %.2f", segment.radius);
      ImGui::Text("theta: (%.2f)", segment.theta);
      ImGui::Text("left height offset: (%.2f)", segment.left_height_offset);
      ImGui::Text("right height offset: (%.2f)", segment.right_height_offset);

      changed = true;
      ImGui::TreePop();
    }
  }
  
  return changed;
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
  bool changed = false;
  for (int i = 0; i < spline.segments.size(); i++){
    auto segment = spline.segments[i];
    std::string label = "segment No." + std::to_string(i);
    if (ImGui::TreeNode(label.c_str() )){
      ImGui::Text("position: (%.2f, %.2f, %.2f)", segment.position.x, segment.position.y, segment.position.z);
      ImGui::Text("up: (%.2f, %.2f, %.2f)", segment.up.x, segment.up.y, segment.up.z);
      ImGui::Text("front: (%.2f, %.2f, %.2f)", segment.front.x, segment.front.y, segment.front.z);
      ImGui::Text("radius: %.2f", segment.radius);
      ImGui::Text("theta: (%.2f)", segment.theta);
      ImGui::Text("left height offset: (%.2f)", segment.left_height_offset);
      ImGui::Text("right height offset: (%.2f)", segment.right_height_offset);

      changed = true;
      ImGui::TreePop();
      
    }
  }
  

  return changed;
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


  static glm::vec3 interpolation(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3,
                                 const float t) {
    glm::vec3 b = (v1 - v0) * 3.0f;
    glm::vec3 c = (v2 - v1) * 3.0f - b;
    glm::vec3 d = (v3 - v0) - b - c;
    return v0 + b * t + c * t * t + d * t * t * t;
  }

  static glm::vec3 getTangent(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3,
                              const float t) {
    glm::vec3 b = (v1 - v0) * 3.0f;
    glm::vec3 c = (v2 - v1) * 3.0f - b;
    glm::vec3 d = (v3 - v0) - b - c;

    glm::vec3 tangent = b + 2.0f * c * t + 3.0f * d * t * t;

    return normalize(tangent);
  }

  static float calculateLengthAdaptive(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
                                       const glm::vec3& v3, float t_start=0, float t_end=1, const float tolerance=0.001f) {
    const glm::vec3 mid_point = interpolation(v0, v1, v2, v3, (t_start + t_end) * 0.5f);
    const glm::vec3 start_point = interpolation(v0, v1, v2, v3, t_start);
    const glm::vec3 end_point = interpolation(v0, v1, v2, v3, t_end);

    const float linear_distance = glm::distance(start_point, end_point);
    const float curve_distance = glm::distance(start_point, mid_point) + glm::distance(mid_point, end_point);
    if (fabs(linear_distance - curve_distance) < tolerance) {
      return curve_distance;  // Close enough, return this estimate
    }
    // Subdivide further
    return calculateLengthAdaptive(v0, v1, v2, v3, t_start, (t_start + t_end) * 0.5f, tolerance) +
           calculateLengthAdaptive(v0, v1, v2, v3, (t_start + t_end) * 0.5f, t_end, tolerance);
  }

  static float findTAdaptive(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3,
                             const float t_start, const float target_length, const float tolerance) {
    if (calculateLengthAdaptive(v0, v1, v2, v3, t_start, 1.f, tolerance) <= target_length)
      return 1.f;
    float t_low = t_start, t_high = 1.0f;
    while (t_high - t_low > tolerance) {
      if (float t_mid = (t_low + t_high) * 0.5f;
          calculateLengthAdaptive(v0, v1, v2, v3, t_start, t_mid) < target_length) {
        t_low = t_mid;
      } else {
        t_high = t_mid;
      }
    }
    return (t_low + t_high) * 0.5f;
  }


  // get length of the whole spline and update segmentLengths
  float getLength() {
    float result = 0;
    segmentLengths.clear();
    for (int i = 0; i < joints.size() - 1; i++) {
      float segmentLength = calculateLengthAdaptive(
          joints[i].position, joints[i].right_handle, joints[i + 1].left_handle, joints[i + 1].position);
      result += segmentLength;
      segmentLengths.push_back(segmentLength);
    }
    return result;
  }

  std::vector<SplineSample> getSamplesByLength(float distance) {
    float length = getLength();
    int sampleNum = length / distance;
    return getUniformSamples(sampleNum);

  }

  // return uniform sample points on the spline
  std::vector<SplineSample> getUniformSamples(int num) {
    std::vector<SplineSample> samples;
    float length = getLength();
    float lengthPerSample = length / (num + 1);
    float sampleLength = lengthPerSample;
    float currentLength = 0;
    std::cout << "segment count in getUniform: " << segmentLengths.size() << "\n";
    for (int i = 0; i < segmentLengths.size(); i++) {


      while (currentLength < sampleLength) {
        currentLength += segmentLengths[i++];
      }
      i--;
      glm::vec3 p0 = joints[i].position;
      glm::vec3 p1 = joints[i].right_handle;
      glm::vec3 p2 = joints[i + 1].left_handle;
      glm::vec3 p3 = joints[i + 1].position;
      glm::vec3 pp0 = p0 + (p0 - p1);
      glm::vec3 pp3 = p3 + (p3 - p2);

      float t = findTAdaptive(joints[i].position, joints[i].right_handle, joints[i + 1].left_handle,
                                       joints[i + 1].position, 0, sampleLength - currentLength + segmentLengths[i], 0.00001f);
      glm::vec3 pos = (interpolation(joints[i].position, joints[i].right_handle,
                                                    joints[i + 1].left_handle, joints[i + 1].position, t));
      

      SplineSample s;
      s.position = pos;
      s.segmentIndex = i;
      s.t = t;
      samples.emplace_back(s);

      if (samples.size() == num) {
        break;
      }
      sampleLength += lengthPerSample;
    }
    if (samples.size() != num) {
      EVOENGINE_ERROR("invalid samples count in getUniformSamples")
    }
    return samples;
  }

  // return intersection of the spline and a plane
  [[nodiscard]] std::vector<glm::vec3> getSurfaceIntersection(const glm::vec3& planePoint, const glm::vec3& normal) const {

    
    std::vector<glm::vec3> results;
    for (int i = 0; i < joints.size() - 1; i++) {
      glm::vec3 p0 = joints[i].position;
      glm::vec3 p1 = joints[i].right_handle;
      glm::vec3 p2 = joints[i + 1].left_handle;
      glm::vec3 p3 = joints[i + 1].position;


      // assume each of the bezier curves should only intersect with the plane once 

      // check whether the start point and the end point are on different sides of the plane
      if (glm::dot(p0 - planePoint, normal) * glm::dot(p3 - planePoint, normal) > 0) {
        //std::cout << "skip, i = " << i << "\n";
        continue;
      }

      // if there is intersection, two-step search for t
      float t = bisectionMethod(0.0f, 1.0f, p0, p1, p2, p3, planePoint, normal);
      //std::cout << "after bisection, t = :" << t << "i = " <<i<< "\n";
      t = newtonMethod(t, p0, p1, p2, p3, planePoint, normal);
      //std::cout << "after newton, t = :" << t << "i = " << i << "\n";
      if (t > 1 || t < 0) {
        continue;
      }
      glm::vec3 intersectionPoint = interpolation(p0, p1, p2, p3, t);

      results.push_back(intersectionPoint);
    }
    //std::cout << "get intersection size:" << results.size() << "\n";
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
      glm::vec3 bezierPoint = interpolation(v0, v1, v2, v3, t_mid);
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
      glm::vec3 bezierPoint = interpolation(v0, v1, v2, v3, t);
      float value = planeEquation(bezierPoint, normal, planePoint);
      if (std::abs(value) < epsilon) {
        //std::cout << "newton find intersection: " << t
        //          << "\n";
        return t;  // find intersection
      }

      // calculate the tangent
      glm::vec3 bezierDerivative = getTangent(v0, v1, v2, v3, t);
      float derivative = glm::dot(normal, bezierDerivative);  // calculate derivative
      if (std::abs(derivative) < epsilon) {
        //std::cout << "newton derivative too small, return t" << t
        //          << "\n";
        break;
      }
      t -= value / derivative;        // update t
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

  glm::vec3 segmentInterpolation(int segmentIndex, float t) const {
    glm::vec3 p0 = joints[segmentIndex].position;
    glm::vec3 p1 = joints[segmentIndex].right_handle;
    glm::vec3 p2 = joints[segmentIndex + 1].left_handle;
    glm::vec3 p3 = joints[segmentIndex + 1].position;
    return interpolation(p0, p1, p2, p3, t);
  }

  

  std::vector<glm::vec3> getLineSamples(int numPerCurve = 4) const{
    std::vector<glm::vec3> results;

    for (int i = 0; i < joints.size() - 1; i++) {
      glm::vec3 p0 = joints[i].position;
      glm::vec3 p1 = joints[i].right_handle;
      glm::vec3 p2 = joints[i + 1].left_handle;
      glm::vec3 p3 = joints[i + 1].position;

      int num = 0;
      float val = 1 / (float)numPerCurve;
      while (num < numPerCurve) {
        float t = num * val;
        results.push_back(interpolation(p0, p1, p2, p3, t));
        num++;
      }

    }
    return results;
  }
  
};

//-------------------------
// Debug static params
//-------------------------
// todo: delete this global variable, this is only for debugging
static float radius = 0.003;
static float theta = 50.0f;
// enlarge the sorghum
static float scale = 1.0f;
static float left_height_offset = 0.0f;
static float right_height_offset = 0.0f;




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
      std::vector<glm::vec3> points =  leaf[keys[j]];
      
      // iteration for points
      for (int k = 0; k < points.size(); k++) {
        

        // generate CubicBezierSpline from the points
        glm::vec3 front;
        if (k == points.size() - 1) {
          front = (points[k] - points[k - 1]);
        } else {
          front = (points[k + 1] - points[k]);
        }
        
        CubicBezierPoint p;
        p.position = points[k];
        // C1 continuity
        p.right_handle = p.position + front * 0.25f;
        p.left_handle = p.position - front * 0.25f;

        bezierSpline.joints.emplace_back(p);
      }

      leafRepresentation.insert(std::make_pair(keys[j], bezierSpline));
    }

    bezierSplines.emplace_back(leafRepresentation);
  }

  return bezierSplines;
}


// sample points from the splines to visualize the splines
[[nodiscard]] std::vector<std::unordered_map<std::string, std::vector<glm::vec3>>> GetLineSamplesFromBezierSplines(
    std::vector<std::unordered_map<std::string, CubicBezierSpline>> bezierSplines, int numPerCurve=4) {

  std::vector < std::unordered_map < std::string, std::vector < glm::vec3 >>> results;

  std::vector<std::string> keys = {"leftPoints", "rightPoints", "centerPoints"};

  for (int i = 0; i < bezierSplines.size(); i++) {
    std::unordered_map < std::string, std::vector < glm::vec3 >> pointSet;

    auto leafSplines = bezierSplines[i];

    for (auto key : keys) {

      CubicBezierSpline bezierSpline = leafSplines[key];
      auto points = bezierSpline.getLineSamples(numPerCurve);

      pointSet.insert(std::make_pair(key, points));
    }

    results.emplace_back(pointSet);
  }

  return results;
}
// todo: fix problem of un-smooth transition
void ExtendLeafToStem(SorghumLeafDescriptor& leaf) {

  auto firstSegment = leaf.spline.segments[0];

  auto right = normalize(glm::cross(firstSegment.front, glm::vec3(0, 1, 0)));

  // construct the local frame
  SorghumSplineSegment segment;
  
  segment.front = normalize(glm::vec3(-0.04, 1, 0.01));
  
  segment.up = normalize(glm::cross(right, segment.front));

  segment.theta = 180;
  segment.radius = 0.01f;
  segment.left_height_offset = segment.right_height_offset = 0;


  // find the position on the circle;
  auto projection(segment.up);
  projection.y = 0;
  projection = segment.radius * normalize(-projection);

  segment.position = projection;

  // the segment for interpolation
  SorghumSplineSegment segment2(segment);
  float coefficient = 2.1f;
  segment2.position.y = firstSegment.position.y / coefficient;
  //segment2.radius = firstSegment.radius / coefficient;

  leaf.spline.segments.insert(leaf.spline.segments.begin(), segment);
  leaf.spline.segments.insert(leaf.spline.segments.begin()+1, segment2);

}



// reconstruct the stem
std::vector<glm::vec3> ReconstructSorghumStem(
    const std::shared_ptr<SorghumDescriptor>& sorghum_descriptor,
    int samples = 32) {
  std::vector<glm::vec3> results;
  SorghumStemDescriptor stem;
  glm::vec3 startPoint(0, 0, 0);
  glm::vec3 up(0, 0.01, -1);
  glm::vec3 front(-0.04, 1, 0.01);
  for (int i = 0; i < samples; i++) {
    SorghumSplineSegment segment;
    segment.position = startPoint + (float)i * glm::vec3(0, 0.01, 0);
    segment.up = up;
    segment.front = front;
    segment.radius = 0.01f;
    segment.theta = 180;
    segment.right_height_offset = left_height_offset = 0;
    stem.spline.segments.emplace_back(segment);

    results.emplace_back(segment.position);
    results.emplace_back(up);
    results.emplace_back(front);
  }

  sorghum_descriptor->stem = stem;
  return results;
}

// reconstruct splines to sorghum
std::vector < glm::vec3> ReconstructSorghumFromBezierSplines(
      const std::shared_ptr<SorghumDescriptor>& sorghum_descriptor,
                                    const std::vector<std::unordered_map<std::string, CubicBezierSpline>>& bezierSplines,
                                    float theta, float scale) {

  // clear previous data
  sorghum_descriptor->leaves.clear();

  std::vector<std::string> keys = {"leftPoints", "rightPoints", "centerPoints"};
  int leafCount = bezierSplines.size();

  std::vector<glm::vec3> results;
  for (int i = 0; i < bezierSplines.size(); i++) {
    auto leafSplines = bezierSplines[i];

    SorghumSpline sorghumSpline;
    auto leftLine = leafSplines[keys[0]];
    auto rightLine = leafSplines[keys[1]];
    auto centerLine = leafSplines[keys[2]];


    
    // get uniform samples from the centerLine
    // update: change to getting samples based on distance
    //auto samples = centerLine.getUniformSamples(32);
    auto samples = centerLine.getSamplesByLength(0.015f);

    // reconstruct the local coordinate at each of the samples
    for (int j = 0; j < samples.size(); j++) {
      SplineSample sample = samples[j];
      SorghumSplineSegment segment;

      // get the intersections of the left and right lines on the profile
      glm::vec3 position = sample.position * scale;
      glm::vec3 normal = centerLine.getTangent(sample);
      // assume the closest intersection point should be the correct one
      // todo: how to deal with no intersection?
      std::vector<glm::vec3> leftCandidates = leftLine.getSurfaceIntersection(sample.position, normal);
      auto leftIntersection = std::min_element(leftCandidates.begin(), leftCandidates.end(),
                                               [&position](const glm::vec3& a, const glm::vec3& b) {
                                                 return glm::distance(a, position) < glm::distance(b, position);
                                               });

      std::vector<glm::vec3> rightCandidates = rightLine.getSurfaceIntersection(sample.position, normal);
      auto rightIntersection = std::min_element(rightCandidates.begin(), rightCandidates.end(),
                                                [&position](const glm::vec3& a, const glm::vec3& b) {
                                                  return glm::distance(a, position) < glm::distance(b, position);
                                                });

      glm::vec3 leftPoint;
      glm::vec3 rightPoint;

      if (leftIntersection != leftCandidates.end()) {
        leftPoint = *leftIntersection * scale;
      }
      if (rightIntersection != rightCandidates.end()) {
        rightPoint = *rightIntersection * scale;
      }

      if (leftIntersection == leftCandidates.end() || rightIntersection == rightCandidates.end()) {
        std::cout << "error in leaf: " << i << "\n";
      }

      //-----------------------
      // debug info
      //-----------------------
      results.emplace_back(position);
      results.emplace_back(leftPoint);
      results.emplace_back(rightPoint);

      //----------------------------
      // from the profile, reconstruct the up vector
      //----------------------------
      glm::vec3 right = normalize(glm::cross(normal, glm::vec3(0, 1, 0)));
      right = glm::dot(right, leftPoint - position) < 0 ? -right : right;

      // position
      segment.position = position;

      // front
      segment.front = normal;                                                                                                                                                                                                          

      segment.up = normalize(glm::cross(right, segment.front));

      // w->l
      glm::mat4 worldToLocal = inverse(glm::mat4(right.x, right.y, right.z, 0,         // 1st column
                                         segment.up.x, segment.up.y, segment.up.z, 0,  // 2nd column
                                         -normal.x, -normal.y, -normal.z, 0,  // 3rd column
                                         position.x, position.y, position.z , 1.0f      // 4th column
                                                   ));
      glm::vec3 rightLocal = worldToLocal * glm::vec4(rightPoint, 1.0f);
      glm::vec3 leftLocal = worldToLocal * glm::vec4(leftPoint, 1.0f);

      // reconstruct theta, radius, height_offsets
      segment.theta = theta;
      //segment.radius = std::max(std::max(glm::length(leftLocal), glm::length(rightLocal)), 0.01f);
      segment.radius = std::max(std::abs(leftLocal.x), std::abs(rightLocal.x));
      //segment.radius = radius;

      segment.left_height_offset = -leftLocal.y * 1.5;
      segment.right_height_offset = -rightLocal.y * 1.5;
      //segment.left_height_offset = left_height_offset;
      //segment.right_height_offset = right_height_offset;


      sorghumSpline.segments.emplace_back(segment);
    }
    
    SorghumLeafDescriptor leaf_descriptor;
    leaf_descriptor.spline = sorghumSpline;
    leaf_descriptor.index = i;

    ExtendLeafToStem(leaf_descriptor);

    sorghum_descriptor->leaves.emplace_back(leaf_descriptor);
  }
  std::cout << "total samples count: " << results.size() << "\n";
  return results;
}

void FillYAMLPointsParticle(
    int leafIndex, float scale,
    std::vector<std::unordered_map<std::string, std::vector<glm::vec3>>> & yamlContent, std::vector<ParticleInfo>& particleInfos,
    int leafCount = 1, int lineCount=3, int PointsCount = 32) {
  particleInfos.assign(leafCount * PointsCount * lineCount, ParticleInfo{});
  Jobs::RunParallelFor(leafCount * PointsCount, [&](const auto i) {
    int yamlContentIndex = leafIndex > -1 ? leafIndex : i / PointsCount;
    auto& center_info = particleInfos[i];
    int index = i % PointsCount;
    center_info.instance_color = glm::vec4((256) , 0, 0, 256) / 256.f;
    center_info.instance_matrix.SetPosition(glm::vec3(yamlContent[yamlContentIndex]["centerPoints"][index]) *
                                            scale);
    center_info.instance_matrix.SetScale(glm::vec3(0.005f));

    auto& left_info = particleInfos[i + leafCount * PointsCount];
    left_info.instance_color = glm::vec4(0, 256, 0, 256) / 256.f;
    left_info.instance_matrix.SetPosition(glm::vec3(yamlContent[yamlContentIndex]["leftPoints"][ index]) *
                                          scale);
    left_info.instance_matrix.SetScale(glm::vec3(0.005f));

    auto& right_info = particleInfos[i + 2 * leafCount * PointsCount];
    right_info.instance_color = glm::vec4(0, 0, 256, 256) / 256.f;
    right_info.instance_matrix.SetPosition(glm::vec3(yamlContent[yamlContentIndex]["rightPoints"][index]) *
                                           scale);
    right_info.instance_matrix.SetScale(glm::vec3(0.005f));
  });
}


// todo: handle nonuniform samples per leaf
void FillBezierSplinePointsParticle(int leafIndex, float scale,
                            std::vector<glm::vec3> & bezierSplinePoints,
                                    std::vector<ParticleInfo>& particleInfos, int leafCount = 1, int lineCount = 3,
                                    int PointsCount = 32) {
  std::vector<glm::vec4> colors = {glm::vec4(0, 256, 256, 128), glm::vec4(256, 0, 256, 128),
                                   glm::vec4(256, 256, 0, 128)};
  particleInfos.assign(leafCount * PointsCount * lineCount, ParticleInfo{});
  Jobs::RunParallelFor(leafCount * PointsCount, [&](const auto i) {
    int startIndex = 3 * i;
    int leafStartIndex = leafIndex > -1 ? leafIndex * PointsCount * 3 : 0;
    auto& centerInfo = particleInfos[startIndex];

    centerInfo.instance_matrix.SetPosition(bezierSplinePoints[leafStartIndex + startIndex] * scale);
    centerInfo.instance_matrix.SetScale(glm::vec3(0.005f));
    centerInfo.instance_color = colors[i%3] / 256.f;

    auto& leftInfo = particleInfos[startIndex + 1];

    leftInfo.instance_matrix.SetPosition(bezierSplinePoints[leafStartIndex + startIndex + 1 ] * scale);
    leftInfo.instance_matrix.SetScale(glm::vec3(0.005f));
    leftInfo.instance_color = colors[i % 3] / 256.f;

    auto& rightInfo = particleInfos[startIndex + 2];

    rightInfo.instance_matrix.SetPosition(bezierSplinePoints[leafStartIndex + startIndex + 2] * scale);
    rightInfo.instance_matrix.SetScale(glm::vec3(0.005f));
    rightInfo.instance_color = colors[i % 3] / 256.f;
  });
}

void FillLeafSegmentFrameParticle(int leafIndex, float scale, SorghumDescriptor& sorghum_descriptor,
                             std::vector<ParticleInfo>& particleInfos, int leafCount, bool uniformSegmentCount=true) {
  std::vector<glm::vec4> colors = {glm::vec4(256, 256, 256, 256) , glm::vec4(0, 256, 256, 256), glm::vec4(256, 0, 256, 256),
                                   glm::vec4(256, 256, 0, 256)};
  int samplePoints = 10;
  // assume all leaves have the same segment length
  int segmentCount = sorghum_descriptor.leaves[0].spline.segments.size();
  float dis =
      glm::distance(sorghum_descriptor.leaves[0].spline.segments[0].position, sorghum_descriptor.leaves[0].spline.segments[1].position);
  

  dis = 0.003f;
  if (uniformSegmentCount) {
    particleInfos.assign(leafCount * segmentCount * samplePoints, ParticleInfo{});
    Jobs::RunParallelFor(leafCount * segmentCount, [&](const auto i) {
      int leafStartIndex = leafIndex > -1 ? leafIndex : i / segmentCount;
      int segmentIndex = i % segmentCount;

      auto segment = sorghum_descriptor.leaves[leafStartIndex].spline.segments[segmentIndex];
      auto position = segment.position;
      auto front = segment.front;
      auto up = segment.up;
      auto right = normalize(glm::cross(front, up));

      for (int j = 0; j < samplePoints; j++) {
        auto& info = particleInfos[i + j * leafCount * segmentCount];
        if (j == 0) {
          info.instance_color = colors[0] / 256.f;
          info.instance_matrix.SetPosition(position * scale);
        } else if (j < 3) {
          info.instance_color = colors[1] / 256.f;
          info.instance_matrix.SetPosition(((3 - j) * dis * (front) + position) * scale);
        } else if (j < 6) {
          info.instance_color = colors[2] / 256.f;
          info.instance_matrix.SetPosition(((6 - j) * dis * (right) + position) * scale);
        } else {
          info.instance_color = colors[3] / 256.f;
          info.instance_matrix.SetPosition(((9 - j) * dis * (up) + position) * scale);
        }

        info.instance_matrix.SetScale(glm::vec3(0.002f));
      }
    });
  } else {
    particleInfos.clear();
    int leafStartIndex = leafIndex > -1 ? leafIndex : 0;
    // for loop
    for (int i = leafStartIndex; i < sorghum_descriptor.leaves.size()&& i <  leafStartIndex + leafCount; i++) {
      auto leaf = sorghum_descriptor.leaves[i];

      for (auto segment: leaf.spline.segments) {
        auto position = segment.position;
        auto front = normalize(segment.front);
        auto up = normalize(segment.up);
        auto right = normalize(glm::cross(front, up));

        for (int j = 0; j < samplePoints; j++) {
          ParticleInfo info;
          if (j == 0) {
            info.instance_color = colors[0] / 256.f;
            info.instance_matrix.SetPosition(position * scale);
          } else if (j < 3) {
            info.instance_color = colors[1] / 256.f;
            info.instance_matrix.SetPosition(((3 - j) * dis * (front) + position) * scale);
          } else if (j < 6) {
            info.instance_color = colors[2] / 256.f;
            info.instance_matrix.SetPosition(((6 - j) * dis * (right) + position) * scale);
          } else {
            info.instance_color = colors[3] / 256.f;
            info.instance_matrix.SetPosition(((9 - j) * dis * (up) + position) * scale);
          }
          info.instance_matrix.SetScale(glm::vec3(0.002f));


          particleInfos.emplace_back(info);
        }
      }
    }

    
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
  static std::shared_ptr<ParticleInfoList> yamlPoints;
  static std::shared_ptr<ParticleInfoList> bezierSamples;
  static std::shared_ptr<ParticleInfoList> bezierVisualization;
  static std::shared_ptr<ParticleInfoList> bezierFrames;
  static std::shared_ptr<ParticleInfoList> exampleVisualization;
  static std::vector<glm::vec3> bezierSampleResults;
  static std::vector<std::unordered_map<std::string, CubicBezierSpline>> splines;
  static int gizmoType = 2;
  static bool splitLeaf;
  static bool prevSplitLeaf;
  static int pointsCount = 32;
  static int leafIndex;

  static Entity reconstructed_entity;

  bool updateMesh = false;
  bool updateGizmo = false;

  Transform transform;
  transform.SetPosition({0, 0, 0});
  transform.SetEulerRotation(glm::radians(glm::vec3(0, 0, 0)));
  GizmoSettings gizmo_settings;
  gizmo_settings.draw_settings.blending = true;


  if (ImGui::SliderFloat("theta", &theta, 0.0f, 180.0f)) {
    updateMesh = true;
  }

  if (ImGui::SliderFloat("scale", &scale, 1.0f, 10.0f)) {
    updateMesh = true;
    updateGizmo = true;
  }

  if (ImGui::SliderFloat("radius", &radius, 0.001f, 10.0f)) {
    updateMesh = true;
  }
  if (ImGui::SliderFloat("left height offset", &left_height_offset, -1, 1)) {
    updateMesh = true;
  }
  if (ImGui::SliderFloat("right height offset", &right_height_offset, -1, 1)) {
    updateMesh = true;
  }

  ImGui::Text("Gizmo");
  ImGui::RadioButton("yamlContent", &gizmoType, 0);
  ImGui::RadioButton("bezierSamples", &gizmoType, 1);
  ImGui::RadioButton("bezierVisualization", &gizmoType, 3);
  ImGui::RadioButton("bezierFrames", &gizmoType, 4);
  ImGui::RadioButton("Disable", &gizmoType, 2);
  ImGui::Checkbox("SplitLeaf", &splitLeaf);
  
  switch (gizmoType) {
    case 0:
      editor_layer->DrawGizmoCubes(yamlPoints, 1.0f, 1, gizmo_settings);
      break;
    case 1:
      editor_layer->DrawGizmoCubes(bezierSamples, 1.0f, 1, gizmo_settings);
      break;
    case 3:
      editor_layer->DrawGizmoCubes(bezierVisualization, 1.0f, 1, gizmo_settings);
      break;
    case 4:
      editor_layer->DrawGizmoCubes(bezierFrames, 1.0f, 1, gizmo_settings);
      break;
    default:
      break;

  }
 
  if (splitLeaf != prevSplitLeaf ) {
    updateGizmo = true;
    prevSplitLeaf = splitLeaf;
  }
  if (ImGui::SliderInt("LeafIndex", &leafIndex, 0, 20) && splitLeaf) {
    updateGizmo = true;
  }


  if (updateMesh) {
    // ReconstructFromYAML(sorghum_descriptor, yamlContent, theta, scale);
    splines = ReconstructBezierSplineFromYAML(yamlContent);
    bezierSampleResults = ReconstructSorghumFromBezierSplines(sorghum_descriptor, splines, theta, scale);
    ReconstructSorghumStem(sorghum_descriptor);

    const auto scene = Application::GetActiveScene();
    scene->DeleteEntity(reconstructed_entity);
    reconstructed_entity = sorghum_descriptor->CreateEntity("Temp Sorghum");
  }
  if (updateGizmo) {
    // update the gizmo together
    if (splitLeaf) {
      // update particleInfo
      std::vector<ParticleInfo> particle_infos;
      FillYAMLPointsParticle(leafIndex, scale, yamlContent, particle_infos);
      yamlPoints->SetParticleInfos(particle_infos);

      FillBezierSplinePointsParticle(leafIndex, 1, bezierSampleResults, particle_infos);
      bezierSamples->SetParticleInfos(particle_infos);

      auto lineData = GetLineSamplesFromBezierSplines(splines, 4);
      int PointsCount = lineData[0]["centerPoints"].size();
      FillYAMLPointsParticle(leafIndex, scale, lineData, particle_infos, 1, 3, PointsCount);
      bezierVisualization->SetParticleInfos(particle_infos);

      FillLeafSegmentFrameParticle(leafIndex, 1, *sorghum_descriptor, particle_infos, 1, false);
      bezierFrames->SetParticleInfos(particle_infos);

      FillLeafSegmentFrameParticle(leafIndex, 1, *this, particle_infos, 1, false);
      exampleVisualization->SetParticleInfos(particle_infos);

    } else {
      int leafCount = yamlContent.size();
      int PointsCount = yamlContent[0]["centerPoints"].size();
      std::vector<ParticleInfo> particle_infos;

      FillYAMLPointsParticle(-1, scale, yamlContent, particle_infos, leafCount);
      yamlPoints->SetParticleInfos(particle_infos);

      FillBezierSplinePointsParticle(-1, 1, bezierSampleResults, particle_infos, leafCount);
      bezierSamples->SetParticleInfos(particle_infos);

      auto lineData = GetLineSamplesFromBezierSplines(splines, 4);
      leafCount = splines.size();
      int lineCount = 3;
      PointsCount = lineData[0]["centerPoints"].size();
      FillYAMLPointsParticle(-1, scale, lineData, particle_infos, leafCount, lineCount, PointsCount);
      bezierVisualization->SetParticleInfos(particle_infos);

      FillLeafSegmentFrameParticle(-1, 1, *sorghum_descriptor, particle_infos, sorghum_descriptor->leaves.size(), false);
      bezierFrames->SetParticleInfos(particle_infos);

      FillLeafSegmentFrameParticle(-1, 1, *this, particle_infos, this->leaves.size(), false);
      exampleVisualization->SetParticleInfos(particle_infos);
    }
  }

  if (ImGui::Button("dump leaf segment info")) {
    if (splitLeaf) {
      int i = 0;
      for (auto segment : leaves[leafIndex].spline.segments) {
        std::cout << "radius in segment" << i << ": "<< segment.radius << "\n";
        i++;
      }

      i = 0;
      for (auto segment : leaves[leafIndex].spline.segments) {
        std::cout << "theta in segment" << i << ": " << segment.theta << "\n";
        i++;
      }

      i = 0;
      for (auto segment : leaves[leafIndex].spline.segments) {
        std::cout << "left height offset in segment" << i << ": " << segment.left_height_offset << "\n";
        i++;
      }

      i = 0;
      for (auto segment : leaves[leafIndex].spline.segments) {
        std::cout << "right height offset in segment" << i << ": " << segment.right_height_offset << "\n";
        i++;
      }

    }
  }

  if (ImGui::Button("dump reconstruction leaf segment info")) {
    if (splitLeaf) {
      int i = 0;
      for (auto segment : sorghum_descriptor->leaves[leafIndex].spline.segments) {
        std::cout << "radius in segment" << i << ": " << segment.radius << "\n";
        i++;
      }

      i = 0;
      for (auto segment : sorghum_descriptor->leaves[leafIndex].spline.segments) {
        std::cout << "theta in segment" << i << ": " << segment.theta << "\n";
        i++;
      }

      i = 0;
      for (auto segment : sorghum_descriptor->leaves[leafIndex].spline.segments) {
        std::cout << "left height offset in segment" << i << ": " << segment.left_height_offset << "\n";
        i++;
      }

      i = 0;
      for (auto segment : sorghum_descriptor->leaves[leafIndex].spline.segments) {
        std::cout << "right height offset in segment" << i << ": " << segment.right_height_offset << "\n";
        i++;
      }
    }
  }

  //--------------------------
  // load points from yaml and reconstruct the mesh
  //--------------------------
  if (yamlContent.empty()) {
    std::filesystem::path yamlPath(
        "E:/Computer Graphics/spline.yml");
    if (std::filesystem::exists(yamlPath)) {
      auto tempResult = ImportPrediction(yamlPath);

      if (tempResult) {
        // todo: generate stem descriptor
        yamlContent = *tempResult;
        std::cout << "imported from yaml"
                  << "\n"
                  << "leaf count: " << yamlContent.size() << "\n"
                  << "total points: " << yamlContent[0]["centerPoints"].size() * yamlContent.size() * 3 << "\n";

        splines = ReconstructBezierSplineFromYAML(yamlContent);
        bezierSampleResults = ReconstructSorghumFromBezierSplines(sorghum_descriptor, splines, theta, scale);
        std::cout << "fit bezier splines"
                  << "\n"
                  << "leaf count: " << bezierSampleResults.size() / (3 * 32) << "\n"
                  << "total points: " << bezierSampleResults.size() << "\n";
        ReconstructSorghumStem(sorghum_descriptor);
        
        //ReconstructFromYAML(sorghum_descriptor, yamlContent, theta, scale);

        // draw sorghum
        reconstructed_entity = sorghum_descriptor->CreateEntity("Temp Sorghum");
      }
    }
  }

  //--------------------------
  // gizmo data setup
  //--------------------------
  if (!yamlPoints && !yamlContent.empty()) {
    yamlPoints = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
    int leafCount = yamlContent.size();
    int PointsCount = yamlContent[0]["centerPoints"].size();
    std::vector<ParticleInfo> particle_infos(leafCount * PointsCount * 3);

    FillYAMLPointsParticle(-1, scale, yamlContent, particle_infos, leafCount, 3, PointsCount);

    yamlPoints->SetParticleInfos(particle_infos);
  }

  if (!bezierSamples && ! bezierSampleResults.empty()) {
    bezierSamples = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
    std::vector<ParticleInfo> particle_infos;
    int leafCount = bezierSampleResults.size() / (3 * 32);
    FillBezierSplinePointsParticle(-1, 1, bezierSampleResults, particle_infos, leafCount);
    bezierSamples->SetParticleInfos(particle_infos);
  }
  if (!bezierVisualization && !splines.empty()) {
    bezierVisualization = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
    
    auto lineData = GetLineSamplesFromBezierSplines(splines, 4);

    int leafCount = splines.size();
    int lineCount = 3;
    int PointsCount = lineData[0]["centerPoints"].size();
    std::vector<ParticleInfo> particle_infos;
    FillYAMLPointsParticle(-1, scale, lineData, particle_infos, leafCount,
                           lineCount, PointsCount);
    bezierVisualization->SetParticleInfos(particle_infos);
  }
  if (!bezierFrames && !bezierSampleResults.empty()) {
    bezierFrames = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
    std::vector<ParticleInfo> particle_infos;
    FillLeafSegmentFrameParticle(-1, 1, *sorghum_descriptor, particle_infos, sorghum_descriptor->leaves.size(), false);
    bezierFrames->SetParticleInfos(particle_infos);

  }

  if (!exampleVisualization) {
    exampleVisualization = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
    std::vector<ParticleInfo> particle_infos;
    FillLeafSegmentFrameParticle(-1, 1, *this, particle_infos, this->leaves.size());
    exampleVisualization->SetParticleInfos(particle_infos);
    
  }
  //editor_layer->DrawGizmoCubes(exampleVisualization, 1.0f, 1, gizmo_settings);

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
    sorghum_layer->sorghum_mesh_generator_settings.enable_stem = true;
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
