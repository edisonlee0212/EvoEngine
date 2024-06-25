#include "RadialBoundingVolume.hpp"

#include <Material.hpp>

#include "Graphics.hpp"
#include "PointCloud.hpp"
#include "Tree.hpp"

using namespace eco_sys_lab;
using namespace evo_engine;

glm::vec3 RadialBoundingVolume::GetRandomPoint() {
  if (!m_meshGenerated)
    return glm::vec3(0);
  float sizePoint = glm::linearRand(0.0f, m_totalSize);
  int layerIndex = 0;
  int sectorIndex = 0;
  bool found = false;
  for (int i = 0; i < m_layerAmount; i++) {
    if (sizePoint > m_sizes[i].first) {
      sizePoint -= m_sizes[i].first;
      continue;
    }
    for (int j = 0; j < m_sectorAmount; j++) {
      auto& sector = m_sizes[i].second;
      if (sizePoint > sector[j]) {
        sizePoint -= sector[j];
      } else {
        layerIndex = i;
        sectorIndex = j;
        found = true;
        break;
      }
    }
    break;
  }
  if (!found) {
    layerIndex = m_layerAmount - 1;
    sectorIndex = m_sectorAmount - 1;
  }
  const float heightLevel = m_maxHeight / m_layerAmount;
  const float sliceAngle = 360.0f / m_sectorAmount;
  const float height = heightLevel * layerIndex + glm::linearRand(0.0f, heightLevel);
  const float angle = sliceAngle * sectorIndex + glm::linearRand(0.0f, sliceAngle);
  const float distance = m_layers[layerIndex][sectorIndex].m_maxDistance * glm::length(glm::diskRand(1.0f));
  return glm::vec3(distance * glm::sin(glm::radians(angle)), height, distance * glm::cos(glm::radians(angle)));
}

glm::ivec2 RadialBoundingVolume::SelectSlice(const glm::vec3& position) const {
  glm::ivec2 retVal;
  const float heightLevel = m_maxHeight / m_layerAmount;
  const float sliceAngle = 360.0f / m_sectorAmount;
  auto x = static_cast<int>(position.y / heightLevel);
  if (x < 0)
    x = 0;
  retVal.x = x;
  if (retVal.x >= m_layerAmount)
    retVal.x = m_layerAmount - 1;
  if (position.x == 0 && position.z == 0)
    retVal.y = 0;
  else
    retVal.y = static_cast<int>((glm::degrees(glm::atan(position.x, position.z)) + 180.0f) / sliceAngle);
  if (retVal.y >= m_sectorAmount)
    retVal.y = m_sectorAmount - 1;
  return retVal;
}

glm::vec3 RadialBoundingVolume::TipPosition(int layer, int slice) const {
  const float heightLevel = m_maxHeight / m_layerAmount;
  glm::vec3 retVal;
  retVal.y = heightLevel * (0.5f + layer);
  const float sliceAngle = 360.0f / m_sectorAmount;

  retVal.x = m_layers[layer][slice].m_maxDistance * glm::sin(glm::radians(sliceAngle * slice + 180.0f));
  retVal.z = m_layers[layer][slice].m_maxDistance * glm::cos(glm::radians(sliceAngle * slice + 180.0f));
  return retVal;
}

void RadialBoundingVolume::GenerateMesh() {
  m_boundMeshes.clear();
  if (m_layers.empty())
    return;
  for (int tierIndex = 0; tierIndex < m_layerAmount; tierIndex++) {
    auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
    std::vector<Vertex> vertices;
    std::vector<unsigned> indices;

    const float sliceAngle = 360.0f / m_sectorAmount;
    const int totalAngleStep = 360.0f / m_sectorAmount;
    const int totalLevelStep = 2;
    const float stepAngle = sliceAngle / (totalAngleStep - 1);
    const float heightLevel = m_maxHeight / m_layerAmount;
    const float stepLevel = heightLevel / (totalLevelStep - 1);
    vertices.resize(totalLevelStep * m_sectorAmount * totalAngleStep * 2 + totalLevelStep);
    indices.resize((12 * (totalLevelStep - 1) * totalAngleStep) * m_sectorAmount);
    for (int levelStep = 0; levelStep < totalLevelStep; levelStep++) {
      const float currentHeight = heightLevel * tierIndex + stepLevel * levelStep;
      for (int sliceIndex = 0; sliceIndex < m_sectorAmount; sliceIndex++) {
        for (int angleStep = 0; angleStep < totalAngleStep; angleStep++) {
          const int actualAngleStep = sliceIndex * totalAngleStep + angleStep;

          float currentAngle = sliceAngle * sliceIndex + stepAngle * angleStep;
          if (currentAngle >= 360)
            currentAngle = 0;
          float x = glm::abs(glm::tan(glm::radians(currentAngle)));
          float z = 1.0f;
          if (currentAngle >= 0 && currentAngle <= 90) {
            z *= -1;
            x *= -1;
          } else if (currentAngle > 90 && currentAngle <= 180) {
            x *= -1;
          } else if (currentAngle > 270 && currentAngle <= 360) {
            z *= -1;
          }
          glm::vec3 position = glm::normalize(glm::vec3(x, 0.0f, z)) * m_layers[tierIndex][sliceIndex].m_maxDistance;
          position.y = currentHeight;
          vertices[levelStep * totalAngleStep * m_sectorAmount + actualAngleStep].position = position;
          vertices[levelStep * totalAngleStep * m_sectorAmount + actualAngleStep].tex_coord =
              glm::vec2((float)levelStep / (totalLevelStep - 1), (float)angleStep / (totalAngleStep - 1));
          vertices[levelStep * totalAngleStep * m_sectorAmount + actualAngleStep].normal = glm::normalize(position);
          vertices[totalLevelStep * m_sectorAmount * totalAngleStep + levelStep * totalAngleStep * m_sectorAmount +
                   actualAngleStep]
              .position = position;
          vertices[totalLevelStep * m_sectorAmount * totalAngleStep + levelStep * totalAngleStep * m_sectorAmount +
                   actualAngleStep]
              .tex_coord = glm::vec2((float)levelStep / (totalLevelStep - 1), (float)angleStep / (totalAngleStep - 1));
          vertices[totalLevelStep * m_sectorAmount * totalAngleStep + levelStep * totalAngleStep * m_sectorAmount +
                   actualAngleStep]
              .normal = glm::vec3(0, levelStep == 0 ? -1 : 1, 0);
        }
      }
      vertices[vertices.size() - totalLevelStep + levelStep].position = glm::vec3(0, currentHeight, 0);
      vertices[vertices.size() - totalLevelStep + levelStep].normal = glm::vec3(0, levelStep == 0 ? -1 : 1, 0);
      vertices[vertices.size() - totalLevelStep + levelStep].tex_coord = glm::vec2(0.0f);
    }
    for (int levelStep = 0; levelStep < totalLevelStep - 1; levelStep++) {
      for (int sliceIndex = 0; sliceIndex < m_sectorAmount; sliceIndex++) {
        for (int angleStep = 0; angleStep < totalAngleStep; angleStep++) {
          const int actualAngleStep = sliceIndex * totalAngleStep + angleStep;  // 0-5
          // Fill a quad here.
          if (actualAngleStep < m_sectorAmount * totalAngleStep - 1) {
            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep)] =
                levelStep * totalAngleStep * m_sectorAmount + actualAngleStep;
            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep) + 1] =
                levelStep * totalAngleStep * m_sectorAmount + actualAngleStep + 1;
            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep) + 2] =
                (levelStep + 1) * totalAngleStep * m_sectorAmount + actualAngleStep;

            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep) + 3] =
                (levelStep + 1) * totalAngleStep * m_sectorAmount + actualAngleStep + 1;
            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep) + 4] =
                (levelStep + 1) * totalAngleStep * m_sectorAmount + actualAngleStep;
            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep) + 5] =
                levelStep * totalAngleStep * m_sectorAmount + actualAngleStep + 1;
            // Connect with center here.
            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep) + 6] =
                totalLevelStep * m_sectorAmount * totalAngleStep + levelStep * totalAngleStep * m_sectorAmount +
                actualAngleStep;
            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep) + 7] =
                vertices.size() - totalLevelStep + levelStep;
            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep) + 8] =
                totalLevelStep * m_sectorAmount * totalAngleStep + levelStep * totalAngleStep * m_sectorAmount +
                actualAngleStep + 1;

            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep) + 9] =
                totalLevelStep * m_sectorAmount * totalAngleStep + (levelStep + 1) * totalAngleStep * m_sectorAmount +
                actualAngleStep + 1;
            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep) + 10] =
                vertices.size() - totalLevelStep + (levelStep + 1);
            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep) + 11] =
                totalLevelStep * m_sectorAmount * totalAngleStep + (levelStep + 1) * totalAngleStep * m_sectorAmount +
                actualAngleStep;
          } else {
            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep)] =
                levelStep * totalAngleStep * m_sectorAmount + actualAngleStep;
            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep) + 1] =
                levelStep * totalAngleStep * m_sectorAmount;
            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep) + 2] =
                (levelStep + 1) * totalAngleStep * m_sectorAmount + actualAngleStep;

            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep) + 3] =
                (levelStep + 1) * totalAngleStep * m_sectorAmount;
            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep) + 4] =
                (levelStep + 1) * totalAngleStep * m_sectorAmount + actualAngleStep;
            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep) + 5] =
                levelStep * totalAngleStep * m_sectorAmount;
            // Connect with center here.
            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep) + 6] =
                totalLevelStep * m_sectorAmount * totalAngleStep + levelStep * totalAngleStep * m_sectorAmount +
                actualAngleStep;
            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep) + 7] =
                vertices.size() - totalLevelStep + levelStep;
            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep) + 8] =
                totalLevelStep * m_sectorAmount * totalAngleStep + levelStep * totalAngleStep * m_sectorAmount;

            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep) + 9] =
                totalLevelStep * m_sectorAmount * totalAngleStep + (levelStep + 1) * totalAngleStep * m_sectorAmount;
            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep) + 10] =
                vertices.size() - totalLevelStep + (levelStep + 1);
            indices[12 * (levelStep * totalAngleStep * m_sectorAmount + actualAngleStep) + 11] =
                totalLevelStep * m_sectorAmount * totalAngleStep + (levelStep + 1) * totalAngleStep * m_sectorAmount +
                actualAngleStep;
          }
        }
      }
    }
    VertexAttributes vertexAttributes;
    vertexAttributes.normal = true;
    vertexAttributes.tex_coord = true;
    mesh->SetVertices(vertexAttributes, vertices, indices);
    m_boundMeshes.push_back(std::move(mesh));
  }
  m_meshGenerated = true;
}

void RadialBoundingVolume::FormEntity() {
  if (!m_meshGenerated)
    return;
  const auto scene = Application::GetActiveScene();
  const auto entity = scene->CreateEntity(GetTitle());
  auto children = scene->GetChildren(entity);
  for (auto& child : children) {
    if (scene->GetEntityName(child) == "RBV Geometry")
      scene->DeleteEntity(child);
  }
  children.clear();
  const auto rBVEntity = scene->CreateEntity("RBV Geometry");
  scene->SetParent(rBVEntity, entity, false);
  for (auto i = 0; i < m_boundMeshes.size(); i++) {
    auto slice = scene->CreateEntity("RBV_" + std::to_string(i));
    auto mmc = scene->GetOrSetPrivateComponent<MeshRenderer>(slice).lock();
    auto mat = ProjectManager::CreateTemporaryAsset<Material>();
    mmc->material = mat;
    mmc->mesh = m_boundMeshes[i];
    scene->SetParent(slice, rBVEntity, false);
  }
}

std::string RadialBoundingVolume::AsString() {
  std::string output;
  output += std::to_string(m_layerAmount) + "\n";
  output += std::to_string(m_sectorAmount) + "\n";
  output += std::to_string(m_maxHeight) + "\n";
  int tierIndex = 0;
  for (const auto& tier : m_layers) {
    int sliceIndex = 0;
    for (const auto& slice : tier) {
      output += std::to_string(slice.m_maxDistance);
      output += "\n";
      sliceIndex++;
    }
    tierIndex++;
  }
  output += "\n";
  for (const auto& tier : m_layers) {
    int sliceIndex = 0;
    for (const auto& slice : tier) {
      output += std::to_string(slice.m_maxDistance);
      output += ",";
      sliceIndex++;
    }
    tierIndex++;
  }
  output += "\n";
  return output;
}

void RadialBoundingVolume::FromString(const std::string& string) {
  std::stringstream ifs(string);

  ifs >> m_layerAmount;
  ifs >> m_sectorAmount;
  ifs >> m_maxHeight;
  m_layers.resize(m_layerAmount);
  for (auto& tier : m_layers) {
    tier.resize(m_sectorAmount);
    for (auto& slice : tier) {
      ifs >> slice.m_maxDistance;
    }
  }
  GenerateMesh();
}

void RadialBoundingVolume::ExportAsObj(const std::string& filename) {
  GenerateMesh();
  const auto& meshes = m_boundMeshes;

  std::ofstream of;
  of.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc);
  if (of.is_open()) {
    std::string o = "o ";
    o += "RBV\n";
    of.write(o.c_str(), o.size());
    of.flush();
    std::string data;
    int offset = 1;
#pragma region Data collection
    for (auto& mesh : meshes) {
      for (const auto& vertices : mesh->UnsafeGetVertices()) {
        data += "v " + std::to_string(vertices.position.x) + " " + std::to_string(vertices.position.y) + " " +
                std::to_string(vertices.position.z) + "\n";
      }
    }
    for (auto& mesh : meshes) {
      data += "# List of indices for faces vertices, with (x, y, z).\n";
      auto& triangles = mesh->UnsafeGetTriangles();
      for (auto i = 0; i < triangles.size(); i++) {
        auto f1 = triangles.at(i).x + offset;
        auto f2 = triangles.at(i).y + offset;
        auto f3 = triangles.at(i).z + offset;
        data += "f " + std::to_string(f1) + " " + std::to_string(f2) + " " + std::to_string(f3) + "\n";
      }
      offset += mesh->GetVerticesAmount();
    }
#pragma endregion
    of.write(data.c_str(), data.size());
    of.flush();
  }
}

void RadialBoundingVolume::CalculateVolume(const std::vector<glm::vec3>& points) {
  ResizeVolumes();
  m_maxHeight = 0;
  m_maxRadius = 0;
  for (const auto& point : points) {
    if (point.y > m_maxHeight)
      m_maxHeight = point.y;
    const float radius = glm::length(glm::vec2(point.x, point.z));
    if (radius > m_maxRadius)
      m_maxRadius = radius;
  }
  for (const auto& point : points) {
    const auto sliceIndex = SelectSlice(point);
    const float currentDistance = glm::length(glm::vec2(point.x, point.z));
    if (currentDistance <= m_offset) {
      for (auto& slice : m_layers[sliceIndex.x]) {
        if (slice.m_maxDistance < currentDistance + m_offset)
          slice.m_maxDistance = currentDistance + m_offset;
      }
    } else if (m_layers[sliceIndex.x][sliceIndex.y].m_maxDistance < currentDistance)
      m_layers[sliceIndex.x][sliceIndex.y].m_maxDistance = currentDistance + m_offset;
  }
  GenerateMesh();
  CalculateSizes();
}

bool RadialBoundingVolume::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
  bool changed = false;

  if (IVolume::OnInspect(editorLayer))
    changed = true;

  PrivateComponentRef treeRef;
  if (editorLayer->DragAndDropButton<Tree>(treeRef, "Apply tree volume")) {
    auto tree = treeRef.Get<Tree>();
    if (tree) {
      // CopyVolume(tree->tree_model);
      treeRef.Clear();
      changed = true;
    }
  }

  ImGui::ColorEdit4("Display Color", &m_displayColor.x);
  ImGui::DragFloat("Display Scale", &m_displayScale, 0.01f, 0.01f, 1.0f);
  if (ImGui::DragInt("Layer Amount", &m_layerAmount, 1, 1, 100))
    changed = true;
  if (ImGui::DragInt("Slice Amount", &m_sectorAmount, 1, 1, 100))
    changed = true;
  if (ImGui::Button("Form Entity")) {
    FormEntity();
  }

  bool displayLayer = false;
  if (m_meshGenerated) {
    if (ImGui::TreeNodeEx("Transformations")) {
      ImGui::DragFloat("Max height", &m_maxHeight, 0.01f);
      static float augmentation = 1.0f;
      ImGui::DragFloat("Augmentation radius", &augmentation, 0.01f);
      if (ImGui::Button("Process")) {
        Augmentation(augmentation);
      }
      if (ImGui::Button("Generate mesh")) {
        GenerateMesh();
      }
      ImGui::TreePop();
    }

    if (ImGui::TreeNodeEx("Layers", ImGuiTreeNodeFlags_DefaultOpen)) {
      for (int i = 0; i < m_layerAmount; i++) {
        if (ImGui::TreeNodeEx(("Layer " + std::to_string(i)).c_str())) {
          for (int j = 0; j < m_sectorAmount; j++) {
            if (ImGui::DragFloat(("Sector " + std::to_string(j) + "##" + std::to_string(i)).c_str(),
                                 &m_layers[i][j].m_maxDistance, 0.1f, 0.0f, 100.0f))
              GenerateMesh();
          }

          editorLayer->DrawGizmoMesh(m_boundMeshes[i], m_displayColor);

          displayLayer = true;
          ImGui::TreePop();
        }
      }
      ImGui::TreePop();
    }
  }
  FileUtils::SaveFile("Save RBV", "RBV", {".rbv"}, [this](const std::filesystem::path& path) {
    const std::string data = AsString();
    std::ofstream ofs;
    ofs.open(path.string().c_str(), std::ofstream::out | std::ofstream::trunc);
    ofs.write(data.c_str(), data.length());
    ofs.flush();
    ofs.close();
  });
  FileUtils::OpenFile("Load RBV", "RBV", {".rbv"}, [this](const std::filesystem::path& path) {
    FromString(FileUtils::LoadFileAsString(path));
  });
  FileUtils::SaveFile("Export RBV as OBJ", "3D Model", {".obj"}, [this](const std::filesystem::path& path) {
    ExportAsObj(path.string());
  });

  static AssetRef pointCloud;
  if (editorLayer->DragAndDropButton(pointCloud, "Import from Point Cloud", {"PointCloud"}, true)) {
    if (auto pc = pointCloud.Get<PointCloud>()) {
      std::vector<glm::vec3> points;
      points.resize(pc->positions.size());
      for (int i = 0; i < pc->positions.size(); i++) {
        auto point = pc->positions[i] + pc->offset;
        points[i] = glm::vec3(point.z, point.x, point.y);
      }
      CalculateVolume(points);
      pointCloud.Clear();
    }
  }
  static bool displayBound = false;
  ImGui::Checkbox("Display Bound", &displayBound);
  if (!displayLayer && displayBound && m_meshGenerated) {
    for (auto& i : m_boundMeshes) {
      editorLayer->DrawGizmoMesh(i, m_displayColor);
    }
    for (int i = 0; i < m_layerAmount; i++) {
      for (int j = 0; j < m_sectorAmount; j++) {
        editorLayer->DrawGizmoMesh(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), glm::vec4(0, 0, 0, 1),
                                   glm::translate(TipPosition(i, j)) * glm::scale(glm::vec3(0.1f)));
      }
    }
  }
  return changed;
}

bool RadialBoundingVolume::InVolume(const glm::vec3& position) {
  if (glm::any(glm::isnan(position)))
    return true;
  if (m_meshGenerated) {
    const auto sliceIndex = SelectSlice(position);
    const float currentDistance = glm::length(glm::vec2(position.x, position.z));
    return glm::max(1.0f, m_layers[sliceIndex.x][sliceIndex.y].m_maxDistance) >= currentDistance &&
           position.y <= m_maxHeight;
  }
  return true;
}

bool RadialBoundingVolume::InVolume(const GlobalTransform& globalTransform, const glm::vec3& position) {
  const auto& finalPos = glm::vec3((glm::inverse(globalTransform.value) * glm::translate(position))[3]);

  if (m_meshGenerated) {
    const auto sliceIndex = SelectSlice(finalPos);
    const float currentDistance = glm::length(glm::vec2(finalPos.x, finalPos.z));
    return glm::max(1.0f, m_layers[sliceIndex.x][sliceIndex.y].m_maxDistance) >= currentDistance &&
           finalPos.y <= m_maxHeight;
  }
  return true;
}

void RadialBoundingVolume::Deserialize(const YAML::Node& in) {
  IVolume::Deserialize(in);
  m_meshGenerated = false;
  m_offset = in["m_offset"].as<float>();
  m_displayColor = in["m_displayColor"].as<glm::vec4>();
  m_maxHeight = in["m_maxHeight"].as<float>();
  m_maxRadius = in["m_maxRadius"].as<float>();
  m_displayScale = in["m_displayScale"].as<float>();
  m_layerAmount = in["m_layerAmount"].as<int>();
  m_sectorAmount = in["m_sectorAmount"].as<int>();

  if (in["m_layers"]) {
    m_layers.resize(m_layerAmount);
    for (auto& i : m_layers) {
      i.resize(m_sectorAmount);
    }
    int index = 0;
    for (const auto& i : in["m_layers"]) {
      m_layers[index / m_sectorAmount][index % m_sectorAmount].m_maxDistance = i["m_maxDistance"].as<float>();
      index++;
    }
  }
  GenerateMesh();
}

void RadialBoundingVolume::Serialize(YAML::Emitter& out) const {
  IVolume::Serialize(out);
  out << YAML::Key << "offset" << YAML::Value << m_offset;
  out << YAML::Key << "m_displayColor" << YAML::Value << m_displayColor;
  out << YAML::Key << "m_maxHeight" << YAML::Value << m_maxHeight;
  out << YAML::Key << "m_maxRadius" << YAML::Value << m_maxRadius;
  out << YAML::Key << "m_displayScale" << YAML::Value << m_displayScale;
  out << YAML::Key << "m_layerAmount" << YAML::Value << m_layerAmount;
  out << YAML::Key << "m_sectorAmount" << YAML::Value << m_sectorAmount;

  if (!m_layers.empty()) {
    out << YAML::Key << "m_layers" << YAML::BeginSeq;
    for (const auto& i : m_layers) {
      for (const auto& j : i) {
        out << YAML::BeginMap;
        out << YAML::Key << "m_maxDistance" << YAML::Value << j.m_maxDistance;
        out << YAML::EndMap;
      }
    }
    out << YAML::EndSeq;
  }
}

void RadialBoundingVolume::Augmentation(float value) {
  m_maxHeight += value * 2.0f;
  for (auto& i : m_layers) {
    for (auto& j : i) {
      j.m_maxDistance += value;
    }
  }
}

void RadialBoundingVolume::CalculateSizes() {
  m_sizes.resize(m_layerAmount);
  m_totalSize = 0.0f;
  for (int i = 0; i < m_layerAmount; i++) {
    auto& layer = m_layers[i];
    m_sizes[i].second.resize(layer.size());
    m_sizes[i].first = 0;
    // 1. Calculate each each sector's volume
    for (int j = 0; j < layer.size(); j++) {
      auto& sector = layer[j];
      m_sizes[i].second[j] = sector.m_maxDistance * sector.m_maxDistance;
      m_sizes[i].first += sector.m_maxDistance * sector.m_maxDistance;
    }
    m_totalSize += m_sizes[i].first;
  }
}

void RadialBoundingVolume::ResizeVolumes() {
  m_layers.resize(m_layerAmount);
  for (auto& tier : m_layers) {
    tier.resize(m_sectorAmount);
    for (auto& slice : tier) {
      slice.m_maxDistance = 0.0f;
    }
  }
}