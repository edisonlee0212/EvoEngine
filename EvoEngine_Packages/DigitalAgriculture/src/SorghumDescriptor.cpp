#include "SorghumDescriptor.hpp"

#include "IVolume.hpp"
#include "Sorghum.hpp"
#include "SorghumDescriptorReconstruction.hpp"
#include "SorghumLayer.hpp"
#include "SorghumTraitDescriptor.hpp"
#include "assimp/code/AssetLib/3MF/3MFXmlTags.h"
using namespace digital_agriculture_package;

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
  eco_sys_lab_package::SphericalVolume volume;
  volume.radius = panicle_size;
  for (int seed_index = 0; seed_index < seed_amount; seed_index++) {
    glm::vec3 position_offset = volume.GetRandomPoint();
    for (const auto position : icosahedron_vertices) {
      archetype.position = position * seed_radius + glm::vec3(0, panicle_size.y * .5f, 0) + position_offset + stem_tip;
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
  eco_sys_lab_package::SphericalVolume volume;
  volume.radius = panicle_size;

  for (int seed_index = 0; seed_index < seed_amount; seed_index++) {
    glm::vec3 position_offset = volume.GetRandomPoint();
    glm::vec3 position = glm::vec3(0, panicle_size.y * .5f, 0) + position_offset + stem_tip;
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
  auto sorghum_layer = ApplicationContext::Get().GetLayer<SorghumLayer>();
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
  for (int i = 0; i < spline.segments.size(); i++) {
    const auto segment = spline.segments[i];
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
  auto sorghum_layer = ApplicationContext::Get().GetLayer<SorghumLayer>();
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

bool SorghumDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  if (ImGui::Button("Instantiate")) {
    CreateEntity("New Sorghum");
  }
  // after load from spline, replace data in sorghumdescriptor
  FileUtils::OpenFile(
      "Load splines", "YAML", {".yml"},
      [&](const std::filesystem::path& path) {
        // @edisonlee0212: here I reconstruct the sorghum descriptor from yaml and create the mesh.
        if (auto temp_result = ImportPrediction(path)) {
          SorghumDescriptorReconstruction reconstruction;
          auto yaml_content = *temp_result;
          std::cout << "imported from yaml"
                    << "\n"
                    << "leaf count: " << yaml_content.size() << "\n"
                    << "total points: " << yaml_content[0]["centerPoints"].size() * yaml_content.size() * 3 << "\n";

          auto splines = SorghumDescriptorReconstruction::ReconstructBezierSplineFromYaml(yaml_content);
          SorghumDescriptor temp;
          auto bezier_sample_results = reconstruction.ReconstructSorghumFromBezierSplines(temp, splines);
          reconstruction.ReconstructSorghumStem(temp);

          // todo: may need have a copy constructor
          this->leaves = temp.leaves;
          this->stem = temp.stem;
          this->panicle = temp.panicle;
          CreateEntity("New Sorghum");
        }
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

  return changed;
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
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto sorghum_entity = scene->CreateEntity(name);
  const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock();
  sorghum->sorghum_descriptor = GetSelf();

  if (const auto sorghum_layer = ApplicationContext::Get().GetLayer<SorghumLayer>()) {
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
            std::unordered_map<std::string, std::vector<glm::vec3>> leaf_info;

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

              leaf_info.insert(std::make_pair("centerPoints", center_points));
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

              leaf_info.insert(std::make_pair("leftPoints", left_points));
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

              leaf_info.insert(std::make_pair("rightPoints", right_points));
            }
            if (leaf_in["Leaf Index"]) {
              leaf_index = leaf_in["Leaf Index"].as<int>();

              leaf_info.insert(std::make_pair("leafIndex", std::vector<glm::vec3>(leaf_index)));
            }

            results.emplace_back(leaf_info);
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

void SorghumDescriptor::ExtractTraits() const {
  auto sorghum_trait = AssetManager::CreateTemporaryAsset<SorghumTraitDescriptor>();
  auto internode_lengths = CalculateInterNodeLengths();
  for (auto leaf : leaves) {
    SorghumLeafTrait leaf_trait;
    leaf_trait.leaf_index = leaf.index;

    // internode below the leaf
    leaf_trait.internode_length = internode_lengths[leaf.index];

    // the widest width along the leaf
    leaf_trait.leaf_width = CalculateLeafWidth(leaf.index);  

    // the length along the leaf
    leaf_trait.leaf_length = CalculateLeafLength(leaf.index); 

    // total area on the leaf
    leaf_trait.leaf_area = CalculateLeafArea(leaf.index);

    sorghum_trait->leaf_traits.emplace_back(leaf_trait);
  }

  // stem length
  sorghum_trait->stem_length = 0;


  std::filesystem::path resource_folder_path("../../../../../Resources");
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../Resources";
  }
  resource_folder_path = std::filesystem::absolute(resource_folder_path);
  auto output_path = resource_folder_path / "output.yml";
  std::cout << "extract traits to " << output_path << std::endl;
  YAML::Emitter out;
  sorghum_trait->Serialize(out);
  std::ofstream output_file(output_path.string());
  output_file << out.c_str();
  output_file.flush();
}

float SorghumDescriptor::CalculateLeafArea(int leaf_index) const {

  
  float total_area = 0.0f;
  const auto scene = Application::GetActiveScene();
  const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();

  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;
  const auto leaf_state = leaves[leaf_index];
  constexpr auto sorghum_mesh_generator_settings = SorghumMeshGeneratorSettings{};
  leaf_state.GenerateGeometry(vertices, indices, sorghum_mesh_generator_settings, false);
   if (sorghum_mesh_generator_settings.bottom_face) {
     leaf_state.GenerateGeometry(vertices, indices, sorghum_mesh_generator_settings, true);
   }
  VertexAttributes attributes{};
  attributes.tex_coord = true;
  mesh->SetVertices(attributes, vertices, indices);


  for (const auto& triangle : mesh->UnsafeGetTriangles()) {
    auto& v = mesh->UnsafeGetVertices();

    IlluminationSampler<glm::vec3> light_probe;
    light_probe.v_0 = v[triangle.x];
    light_probe.v_1 = v[triangle.y];
    light_probe.v_2 = v[triangle.z];


    const float area = light_probe.GetArea();

    //todo: this may need argue
    total_area += area;

  }
  
  return total_area;
}

float SorghumDescriptor::CalculateLeafLength(int leaf_index) const {
  const auto& spline = leaves[leaf_index].spline;
  return spline.GetArcLength();
}

float SorghumDescriptor::CalculateLeafWidth(int leaf_index) const {
  const auto& spline = leaves[leaf_index].spline;
  if (spline.segments.empty())
    return 0.0f;
  auto sorghum_layer = Application::GetLayer<SorghumLayer>();


  SorghumSpline temp_spline;
  spline.SubdivideByDistance(sorghum_layer->vertical_subdivision_length, temp_spline.segments);

  const auto& segments = temp_spline.GetLeafPart();
  const int steps = sorghum_layer->horizontal_subdivision_step;
  float max_width = 0.0f;
  for (const auto& segment : segments) {
    const float angle_step = segment.theta / static_cast<float>(steps);
    float arc_width = 0.0f;
    glm::vec3 prev = glm::vec3(segment.GetLeafPoint(-steps * angle_step));
    for (int j = -steps + 1; j <= steps; j++) {
      glm::vec3 curr = glm::vec3(segment.GetLeafPoint(j * angle_step));
      arc_width += glm::distance(curr, prev);
      prev = curr;
    }
    max_width = std::max(max_width, arc_width);
  }
  return max_width;

}

std::vector<float> SorghumDescriptor::CalculateInterNodeLengths() const {
  glm::vec3 center_point(0, 0, 0);
  for (const auto& leaf : leaves) {
    SorghumSpline leaf_spline;
    leaf_spline.segments = leaf.spline.segments;
    auto stem_part = leaf_spline.GetStemPart();
    auto point = stem_part[0].position;
    point.y = 0;
    center_point += point;
  }

  center_point /= leaves.size();

  auto stem_direction = normalize(stem.spline.segments[0].front);

  std::vector<glm::vec3> nodes;

  for (const auto& leaf : leaves) {
    SorghumSpline leaf_spline;
    leaf_spline.segments = leaf.spline.segments;

    auto segments = leaf_spline.RebuildFixedSizeSegments(64);

    auto stem_part = leaf_spline.GetStemPart();

    auto stem_part_size = stem_part.size();

    auto leaf_part = leaf_spline.GetLeafPart();

    /////////////////////////////////////////
    // extend leaf to the center of the coordinate, which is the node position
    auto p1 = leaf_part[0].position;
    auto p2 = leaf_part[1].position;
    auto c = center_point;
    auto d = stem_direction;

    auto u = p1 - p2;
    auto w = p2 - c;

    float a = glm::dot(u, u);
    float b = glm::dot(u, d);
    float c2 = glm::dot(d, d);
    float e = glm::dot(u, w);
    float f = glm::dot(d, w);

    float denom = a * c2 - b * b;
    float s = 0.0f, t = 0.0f;

    if (denom > 1e-6f) {
      s = (b * f - c2 * e) / denom;
      t = (a * f - b * e) / denom;
    }

    glm::vec3 intersection = p2 + s * u;

    nodes.emplace_back(intersection);
  }

  std::vector<float> internode_lengths;
  for (int i = 0; i < nodes.size(); i++) {
    auto node_position = nodes[i];
    float internode_length = 0;
    // internode length = node point - ground point
    if (i == 0) {
      internode_length = glm::distance(node_position, center_point);
    } else {
      internode_length = glm::distance(node_position, nodes[i - 1]);
    }
    internode_lengths.emplace_back(internode_length);
    
    
  }
  return internode_lengths;
}


