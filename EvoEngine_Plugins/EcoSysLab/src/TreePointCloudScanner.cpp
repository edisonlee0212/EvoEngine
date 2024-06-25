#include "TreePointCloudScanner.hpp"
#ifdef BUILD_WITH_RAYTRACER
#  include <CUDAModule.hpp>
#  include <RayTracer.hpp>
#  include <RayTracerLayer.hpp>
#endif
#include "Tinyply.hpp"
using namespace tinyply;
#include "EcoSysLabLayer.hpp"
#include "Soil.hpp"
using namespace eco_sys_lab;
#pragma region Settings
void TreePointCloudPointSettings::OnInspect() {
  ImGui::DragFloat("Point variance", &m_variance, 0.01f);
  ImGui::DragFloat("Point uniform random radius", &m_ballRandRadius, 0.01f);
  ImGui::DragFloat("Bounding box offset", &m_boundingBoxLimit, 0.01f);
  ImGui::Checkbox("Type Index", &m_typeIndex);
  ImGui::Checkbox("Instance Index", &m_instanceIndex);
  ImGui::Checkbox("Branch Index", &m_branchIndex);
  ImGui::Checkbox("Tree Part Index", &m_treePartIndex);
  ImGui::Checkbox("Line Index", &m_lineIndex);
  ImGui::Checkbox("Internode Index", &m_internodeIndex);
}

void TreePointCloudPointSettings::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "m_variance" << YAML::Value << m_variance;
  out << YAML::Key << "m_ballRandRadius" << YAML::Value << m_ballRandRadius;
  out << YAML::Key << "m_typeIndex" << YAML::Value << m_typeIndex;
  out << YAML::Key << "m_instanceIndex" << YAML::Value << m_instanceIndex;
  out << YAML::Key << "m_branchIndex" << YAML::Value << m_branchIndex;
  out << YAML::Key << "tree_part_index" << YAML::Value << m_treePartIndex;
  out << YAML::Key << "line_index" << YAML::Value << m_lineIndex;
  out << YAML::Key << "m_internodeIndex" << YAML::Value << m_internodeIndex;
  out << YAML::Key << "m_boundingBoxLimit" << YAML::Value << m_boundingBoxLimit;
  out << YAML::EndMap;
}

void TreePointCloudPointSettings::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    auto& cd = in[name];
    if (cd["m_variance"])
      m_variance = cd["m_variance"].as<float>();
    if (cd["m_ballRandRadius"])
      m_ballRandRadius = cd["m_ballRandRadius"].as<float>();
    if (cd["m_typeIndex"])
      m_typeIndex = cd["m_typeIndex"].as<bool>();
    if (cd["m_instanceIndex"])
      m_instanceIndex = cd["m_instanceIndex"].as<bool>();
    if (cd["m_branchIndex"])
      m_branchIndex = cd["m_branchIndex"].as<bool>();
    if (cd["tree_part_index"])
      m_treePartIndex = cd["tree_part_index"].as<bool>();
    if (cd["line_index"])
      m_lineIndex = cd["line_index"].as<bool>();
    if (cd["m_internodeIndex"])
      m_internodeIndex = cd["m_internodeIndex"].as<bool>();
    if (cd["m_boundingBoxLimit"])
      m_boundingBoxLimit = cd["m_boundingBoxLimit"].as<float>();
  }
}

bool TreePointCloudCircularCaptureSettings::OnInspect() {
  bool changed = false;
  if (ImGui::DragFloat("Distance to focus point", &m_distance, 0.01f))
    changed = true;
  if (ImGui::DragFloat("Height to ground", &m_height, 0.01f))
    changed = true;
  ImGui::Separator();
  ImGui::Text("Rotation:");
  if (ImGui::DragInt3("Pitch Angle Start/Step/End", &m_pitchAngleStart, 1))
    changed = true;
  if (ImGui::DragInt3("Turn Angle Start/Step/End", &m_turnAngleStart, 1))
    changed = true;
  ImGui::Separator();
  ImGui::Text("Camera Settings:");
  if (ImGui::DragFloat("FOV", &m_fov))
    changed = true;
  if (ImGui::DragInt("Resolution", &m_resolution))
    changed = true;
  if (ImGui::DragFloat("Max Depth", &m_cameraDepthMax))
    changed = true;
  return changed;
}

void TreePointCloudCircularCaptureSettings::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;

  out << YAML::Key << "m_pitchAngleStart" << YAML::Value << m_pitchAngleStart;
  out << YAML::Key << "m_pitchAngleStep" << YAML::Value << m_pitchAngleStep;
  out << YAML::Key << "m_pitchAngleEnd" << YAML::Value << m_pitchAngleEnd;
  out << YAML::Key << "m_turnAngleStart" << YAML::Value << m_turnAngleStart;
  out << YAML::Key << "m_turnAngleStep" << YAML::Value << m_turnAngleStep;
  out << YAML::Key << "m_turnAngleEnd" << YAML::Value << m_turnAngleEnd;
  out << YAML::Key << "m_distance" << YAML::Value << m_distance;
  out << YAML::Key << "m_height" << YAML::Value << m_height;
  out << YAML::Key << "m_fov" << YAML::Value << m_fov;
  out << YAML::Key << "resolution_" << YAML::Value << m_resolution;
  out << YAML::Key << "m_cameraDepthMax" << YAML::Value << m_cameraDepthMax;
  out << YAML::EndMap;
}

void TreePointCloudCircularCaptureSettings::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    auto& cd = in[name];

    if (cd["m_pitchAngleStart"])
      m_pitchAngleStart = cd["m_pitchAngleStart"].as<int>();
    if (cd["m_pitchAngleStep"])
      m_pitchAngleStep = cd["m_pitchAngleStep"].as<int>();
    if (cd["m_pitchAngleEnd"])
      m_pitchAngleEnd = cd["m_pitchAngleEnd"].as<int>();
    if (cd["m_turnAngleStart"])
      m_turnAngleStart = cd["m_turnAngleStart"].as<int>();
    if (cd["m_turnAngleStep"])
      m_turnAngleStep = cd["m_turnAngleStep"].as<int>();
    if (cd["m_turnAngleEnd"])
      m_turnAngleEnd = cd["m_turnAngleEnd"].as<int>();
    if (cd["m_distance"])
      m_distance = cd["m_distance"].as<float>();
    if (cd["m_height"])
      m_height = cd["m_height"].as<float>();
    if (cd["m_fov"])
      m_fov = cd["m_fov"].as<float>();
    if (cd["resolution_"])
      m_resolution = cd["resolution_"].as<int>();
    if (cd["m_cameraDepthMax"])
      m_cameraDepthMax = cd["m_cameraDepthMax"].as<float>();
  }
}

GlobalTransform TreePointCloudCircularCaptureSettings::GetTransform(const glm::vec2& focusPoint, const float turnAngle,
                                                                    const float pitchAngle) const {
  GlobalTransform cameraGlobalTransform;
  const glm::vec3 cameraPosition = glm::vec3(glm::sin(glm::radians(turnAngle)) * m_distance, m_height,
                                             glm::cos(glm::radians(turnAngle)) * m_distance);
  const glm::vec3 cameraDirection =
      glm::vec3(glm::sin(glm::radians(turnAngle)) * m_distance, m_distance * glm::sin(glm::radians(pitchAngle)),
                glm::cos(glm::radians(turnAngle)) * m_distance);
  cameraGlobalTransform.SetPosition(cameraPosition + glm::vec3(focusPoint.x, 0, focusPoint.y));
  cameraGlobalTransform.SetRotation(glm::quatLookAt(glm::normalize(-cameraDirection), glm::vec3(0, 1, 0)));
  return cameraGlobalTransform;
}

void TreePointCloudCircularCaptureSettings::GenerateSamples(std::vector<PointCloudSample>& pointCloudSamples) {
  int counter = 0;
  for (int turnAngle = m_turnAngleStart; turnAngle < m_turnAngleEnd; turnAngle += m_turnAngleStep) {
    for (int pitchAngle = m_pitchAngleStart; pitchAngle < m_pitchAngleEnd; pitchAngle += m_pitchAngleStep) {
      pointCloudSamples.resize((counter + 1) * m_resolution * m_resolution);
      auto scannerGlobalTransform = GetTransform(glm::vec2(m_focusPoint.x, m_focusPoint.y), turnAngle, pitchAngle);
      auto front = scannerGlobalTransform.GetRotation() * glm::vec3(0, 0, -1);
      auto up = scannerGlobalTransform.GetRotation() * glm::vec3(0, 1, 0);
      auto left = scannerGlobalTransform.GetRotation() * glm::vec3(1, 0, 0);
      auto position = scannerGlobalTransform.GetPosition();
      std::vector<std::shared_future<void>> results;
      Jobs::RunParallelFor(m_resolution * m_resolution, [&](const unsigned i) {
        const float x = i % m_resolution;
        const float y = i / m_resolution;
        const float xAngle =
            (x - m_resolution / 2.0f + glm::linearRand(-0.5f, 0.5f)) / static_cast<float>(m_resolution) * m_fov / 2.0f;
        const float yAngle =
            (y - m_resolution / 2.0f + glm::linearRand(-0.5f, 0.5f)) / static_cast<float>(m_resolution) * m_fov / 2.0f;
        auto& sample = pointCloudSamples[counter * m_resolution * m_resolution + i];
        sample.direction =
            glm::normalize(glm::rotate(glm::rotate(front, glm::radians(xAngle), left), glm::radians(yAngle), up));
        sample.start = position;
      });
      counter++;
    }
  }
}

bool TreePointCloudGridCaptureSettings::OnInspect() {
  bool changed = false;
  if (ImGui::DragFloat("Max size", &m_boundingBoxSize, 0.1f, 0.f, 999.f))
    changed = true;
  if (ImGui::DragInt2("Grid size", &m_gridSize.x, 1, 0, 100))
    changed = true;
  if (ImGui::DragFloat2("Grid distance", &m_gridDistance.x, 0.1f, 0.0f, 100.0f))
    changed = true;
  if (ImGui::DragFloat("Step", &m_step, 0.01f, 0.0f, 0.5f))
    changed = true;
  if (ImGui::DragInt("Sample", &m_backpackSample, 1, 1, INT_MAX))
    changed = true;
  return changed;
}

void TreePointCloudGridCaptureSettings::GenerateSamples(std::vector<PointCloudSample>& pointCloudSamples) {
  const glm::vec2 startPoint = glm::vec2((static_cast<float>(m_gridSize.x) * 0.5f - 0.5f) * m_gridDistance.x,
                                         (static_cast<float>(m_gridSize.y) * 0.5f - 0.5f) * m_gridDistance.y);

  const int yStepSize = m_gridSize.y * m_gridDistance.y / m_step;
  const int xStepSize = m_gridSize.x * m_gridDistance.x / m_step;

  pointCloudSamples.resize((m_gridSize.x * yStepSize + m_gridSize.y * xStepSize) * (m_backpackSample + m_droneSample));
  unsigned startIndex = 0;
  for (int i = 0; i < m_gridSize.x; i++) {
    float x = i * m_gridDistance.x;
    for (int step = 0; step < yStepSize; step++) {
      float z = step * m_step;
      const glm::vec3 center = glm::vec3{x, m_backpackHeight, z} - glm::vec3(startPoint.x, 0, startPoint.y);
      Jobs::RunParallelFor(m_backpackSample, [&](unsigned sampleIndex) {
        auto& sample = pointCloudSamples[m_backpackSample * (i * yStepSize + step) + sampleIndex];
        sample.direction = glm::sphericalRand(1.0f);
        if (glm::linearRand(0.0f, 1.0f) > 0.3f) {
          sample.direction.y = glm::abs(sample.direction.y);
        } else {
          sample.direction.y = -glm::abs(sample.direction.y);
        }
        sample.start = center;
      });
    }
  }

  startIndex += m_gridSize.x * yStepSize * m_backpackSample;
  for (int i = 0; i < m_gridSize.y; i++) {
    float z = i * m_gridDistance.y;
    for (int step = 0; step < xStepSize; step++) {
      float x = step * m_step;
      const glm::vec3 center = glm::vec3{x, m_backpackHeight, z} - glm::vec3(startPoint.x, 0, startPoint.y);
      Jobs::RunParallelFor(m_backpackSample, [&](unsigned sampleIndex) {
        auto& sample = pointCloudSamples[startIndex + m_backpackSample * (i * xStepSize + step) + sampleIndex];
        sample.direction = glm::sphericalRand(1.0f);
        if (glm::linearRand(0.0f, 1.0f) > 0.3f) {
          sample.direction.y = glm::abs(sample.direction.y);
        } else {
          sample.direction.y = -glm::abs(sample.direction.y);
        }
        sample.start = center;
      });
    }
  }

  startIndex += m_gridSize.y * xStepSize * m_backpackSample;
  for (int i = 0; i < m_gridSize.x; i++) {
    float x = i * m_gridDistance.x;
    for (int step = 0; step < yStepSize; step++) {
      float z = step * m_step;
      const glm::vec3 center = glm::vec3{x, m_droneHeight, z} - glm::vec3(startPoint.x, 0, startPoint.y);
      Jobs::RunParallelFor(m_droneSample, [&](unsigned sampleIndex) {
        auto& sample = pointCloudSamples[m_droneSample * (i * yStepSize + step) + sampleIndex];
        sample.direction = glm::sphericalRand(1.0f);
        sample.direction.y = -glm::abs(sample.direction.y);
        sample.start = center;
      });
    }
  }

  startIndex += m_gridSize.x * yStepSize * m_droneSample;
  for (int i = 0; i < m_gridSize.y; i++) {
    float z = i * m_gridDistance.y;
    for (int step = 0; step < xStepSize; step++) {
      float x = step * m_step;
      const glm::vec3 center = glm::vec3{x, m_droneHeight, z} - glm::vec3(startPoint.x, 0, startPoint.y);
      Jobs::RunParallelFor(m_droneSample, [&](unsigned sampleIndex) {
        auto& sample = pointCloudSamples[startIndex + m_droneSample * (i * xStepSize + step) + sampleIndex];
        sample.direction = glm::sphericalRand(1.0f);
        sample.direction.y = -glm::abs(sample.direction.y);
        sample.start = center;
      });
    }
  }
}

bool TreePointCloudGridCaptureSettings::SampleFilter(const PointCloudSample& sample) {
  if (m_boundingBoxSize == 0.f)
    return true;
  return glm::abs(sample.m_hitInfo.position.x) < m_boundingBoxSize &&
         glm::abs(sample.m_hitInfo.position.z) < m_boundingBoxSize;
}
#pragma endregion

void TreePointCloudScanner::Capture(const TreeMeshGeneratorSettings& meshGeneratorSettings,
                                    const std::filesystem::path& savePath,
                                    const std::shared_ptr<PointCloudCaptureSettings>& captureSettings) const {
#ifdef BUILD_WITH_RAYTRACER

  const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
  std::shared_ptr<Soil> soil;
  const auto soilCandidate = EcoSysLabLayer::FindSoil();
  if (!soilCandidate.expired())
    soil = soilCandidate.lock();
  if (!soil) {
    EVOENGINE_ERROR("No soil!");
    return;
  }
  std::unordered_map<Handle, Handle> branchMeshRendererHandles, foliageMeshRendererHandles;
  Bound plantBound{};
  auto scene = GetScene();
  const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  if (treeEntities == nullptr) {
    EVOENGINE_ERROR("No trees!");
    return;
  }
  for (const auto& treeEntity : *treeEntities) {
    if (scene->IsEntityValid(treeEntity)) {
      // auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
      // auto copyPath = savePath;
      // tree->ExportTreeParts(ecoSysLabLayer->meshGeneratorSettings, copyPath.replace_extension(".yml"));

      scene->ForEachChild(treeEntity, [&](Entity child) {
        if (scene->GetEntityName(child) == "Branch Mesh" && scene->HasPrivateComponent<MeshRenderer>(child)) {
          const auto branchMeshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
          branchMeshRendererHandles.insert({branchMeshRenderer->GetHandle(), treeEntity.GetIndex()});

          const auto globalTransform = scene->GetDataComponent<GlobalTransform>(child);
          const auto mesh = branchMeshRenderer->mesh.Get<Mesh>();
          plantBound.min =
              glm::min(plantBound.min, glm::vec3(globalTransform.value * glm::vec4(mesh->GetBound().min, 1.0f)));
          plantBound.max =
              glm::max(plantBound.max, glm::vec3(globalTransform.value * glm::vec4(mesh->GetBound().max, 1.0f)));
        } else if (scene->GetEntityName(child) == "Foliage Mesh" && scene->HasPrivateComponent<Particles>(child)) {
          const auto foliageMeshRenderer = scene->GetOrSetPrivateComponent<Particles>(child).lock();
          foliageMeshRendererHandles.insert({foliageMeshRenderer->GetHandle(), treeEntity.GetIndex()});

          const auto globalTransform = scene->GetDataComponent<GlobalTransform>(child);
          const auto mesh = foliageMeshRenderer->mesh.Get<Mesh>();
          plantBound.min =
              glm::min(plantBound.min, glm::vec3(globalTransform.value * glm::vec4(mesh->GetBound().min, 1.0f)));
          plantBound.max =
              glm::max(plantBound.max, glm::vec3(globalTransform.value * glm::vec4(mesh->GetBound().max, 1.0f)));
        } else if (scene->GetEntityName(child) == "Twig Strands" &&
                   scene->HasPrivateComponent<StrandsRenderer>(child)) {
          const auto twigStrandsRenderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(child).lock();
          branchMeshRendererHandles.insert({twigStrandsRenderer->GetHandle(), treeEntity.GetIndex()});

          const auto globalTransform = scene->GetDataComponent<GlobalTransform>(child);
          const auto strands = twigStrandsRenderer->strands.Get<Strands>();
          plantBound.min = glm::min(plantBound.min,
                                      glm::vec3(globalTransform.value * glm::vec4(strands->GetBound().min, 1.0f)));
          plantBound.max = glm::max(plantBound.max,
                                      glm::vec3(globalTransform.value * glm::vec4(strands->GetBound().max, 1.0f)));
        }
      });
    }
  }
  Handle groundMeshRendererHandle = 0;
  auto soilEntity = soil->GetOwner();
  if (scene->IsEntityValid(soilEntity)) {
    scene->ForEachChild(soilEntity, [&](Entity child) {
      if (scene->GetEntityName(child) == "Ground Mesh" && scene->HasPrivateComponent<MeshRenderer>(child)) {
        groundMeshRendererHandle = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock()->GetHandle();
      }
    });
  }

  std::vector<PointCloudSample> pcSamples;
  captureSettings->GenerateSamples(pcSamples);
  CudaModule::SamplePointCloud(Application::GetLayer<RayTracerLayer>()->environment_properties, pcSamples);

  std::vector<glm::vec3> points;

  std::vector<int> internodeIndex;
  std::vector<int> branchIndex;
  std::vector<int> treePartIndex;
  std::vector<int> treePartTypeIndex;
  std::vector<int> lineIndex;
  std::vector<int> instanceIndex;
  std::vector<int> typeIndex;

  for (const auto& sample : pcSamples) {
    if (!sample.m_hit)
      continue;
    if (!captureSettings->SampleFilter(sample))
      continue;
    auto& position = sample.m_hitInfo.position;
    if (position.x < (plantBound.min.x - m_pointSettings.m_boundingBoxLimit) ||
        position.y < (plantBound.min.y - m_pointSettings.m_boundingBoxLimit) ||
        position.z < (plantBound.min.z - m_pointSettings.m_boundingBoxLimit) ||
        position.x > (plantBound.max.x + m_pointSettings.m_boundingBoxLimit) ||
        position.y > (plantBound.max.y + m_pointSettings.m_boundingBoxLimit) ||
        position.z > (plantBound.max.z + m_pointSettings.m_boundingBoxLimit))
      continue;
    auto ballRand = glm::vec3(0.0f);
    if (m_pointSettings.m_ballRandRadius > 0.0f) {
      ballRand = glm::ballRand(m_pointSettings.m_ballRandRadius);
    }
    const auto distance = glm::distance(sample.m_hitInfo.position, sample.m_start);
    points.emplace_back(sample.m_hitInfo.position +
                        distance * glm::vec3(glm::gaussRand(0.0f, m_pointSettings.m_variance),
                                             glm::gaussRand(0.0f, m_pointSettings.m_variance),
                                             glm::gaussRand(0.0f, m_pointSettings.m_variance)) +
                        ballRand);

    if (m_pointSettings.m_internodeIndex) {
      internodeIndex.emplace_back(static_cast<int>(sample.m_hitInfo.data.x + 0.1f));
    }
    if (m_pointSettings.m_branchIndex) {
      branchIndex.emplace_back(static_cast<int>(sample.m_hitInfo.data.y + 0.1f));
    }
    if (m_pointSettings.m_lineIndex) {
      lineIndex.emplace_back(static_cast<int>(sample.m_hitInfo.data.z + 0.1f));
    }
    if (m_pointSettings.m_treePartIndex) {
      treePartIndex.emplace_back(static_cast<int>(sample.m_hitInfo.data2.x + 0.1f));
    }
    if (m_pointSettings.m_treePartTypeIndex) {
      treePartTypeIndex.emplace_back(static_cast<int>(sample.m_hitInfo.data2.y + 0.1f));
    }
    auto branchSearch = branchMeshRendererHandles.find(sample.m_handle);
    auto foliageSearch = foliageMeshRendererHandles.find(sample.m_handle);
    if (m_pointSettings.m_instanceIndex) {
      if (branchSearch != branchMeshRendererHandles.end()) {
        instanceIndex.emplace_back(branchSearch->second);
      } else if (foliageSearch != foliageMeshRendererHandles.end()) {
        instanceIndex.emplace_back(foliageSearch->second);
      } else {
        instanceIndex.emplace_back(0);
      }
    }

    if (m_pointSettings.m_typeIndex) {
      if (branchSearch != branchMeshRendererHandles.end()) {
        typeIndex.emplace_back(0);
      } else if (foliageSearch != foliageMeshRendererHandles.end()) {
        typeIndex.emplace_back(1);
      } else if (sample.m_handle == groundMeshRendererHandle) {
        typeIndex.emplace_back(2);
      } else {
        typeIndex.emplace_back(-1);
      }
    }
  }
  std::filebuf fb_binary;
  fb_binary.open(savePath.string(), std::ios::out | std::ios::binary);
  std::ostream outstream_binary(&fb_binary);
  if (outstream_binary.fail())
    throw std::runtime_error("failed to open " + savePath.string());
  /*
  std::filebuf fb_ascii;
  fb_ascii.open(filename + "-ascii.ply", std::ios::out);
  std::ostream outstream_ascii(&fb_ascii);
  if (outstream_ascii.fail()) throw std::runtime_error("failed to open " +
  filename);
  */
  PlyFile cube_file;
  cube_file.add_properties_to_element("vertex", {"x", "y", "z"}, Type::FLOAT32, points.size(),
                                      reinterpret_cast<uint8_t*>(points.data()), Type::INVALID, 0);

  if (m_pointSettings.m_typeIndex)
    cube_file.add_properties_to_element("type_index", {"type_index"}, Type::INT32, typeIndex.size(),
                                        reinterpret_cast<uint8_t*>(typeIndex.data()), Type::INVALID, 0);

  if (m_pointSettings.m_instanceIndex) {
    cube_file.add_properties_to_element("instance_index", {"instance_index"}, Type::INT32, instanceIndex.size(),
                                        reinterpret_cast<uint8_t*>(instanceIndex.data()), Type::INVALID, 0);
  }
  if (m_pointSettings.m_branchIndex) {
    cube_file.add_properties_to_element("branch_index", {"branch_index"}, Type::INT32, branchIndex.size(),
                                        reinterpret_cast<uint8_t*>(branchIndex.data()), Type::INVALID, 0);
  }
  if (m_pointSettings.m_treePartIndex) {
    cube_file.add_properties_to_element("tree_part_index", {"tree_part_index"}, Type::INT32, treePartIndex.size(),
                                        reinterpret_cast<uint8_t*>(treePartIndex.data()), Type::INVALID, 0);
  }
  if (m_pointSettings.m_treePartTypeIndex) {
    cube_file.add_properties_to_element("tree_part_type_index", {"tree_part_type_index"}, Type::INT32,
                                        treePartTypeIndex.size(), reinterpret_cast<uint8_t*>(treePartTypeIndex.data()),
                                        Type::INVALID, 0);
  }
  if (m_pointSettings.m_lineIndex) {
    cube_file.add_properties_to_element("line_index", {"line_index"}, Type::INT32, lineIndex.size(),
                                        reinterpret_cast<uint8_t*>(lineIndex.data()), Type::INVALID, 0);
  }
  if (m_pointSettings.m_internodeIndex) {
    cube_file.add_properties_to_element("internode_index", {"internode_index"}, Type::INT32, internodeIndex.size(),
                                        reinterpret_cast<uint8_t*>(internodeIndex.data()), Type::INVALID, 0);
  }
  // Write a binary file
  cube_file.write(outstream_binary, true);

  if (m_pointSettings.m_treePartIndex) {
    try {
      std::filesystem::path ymlPath = savePath;
      ymlPath.replace_extension(".yml");
      YAML::Emitter out;
      out << YAML::BeginMap;
      out << YAML::Key << "Forest" << YAML::BeginSeq;
      for (const auto& treeEntity : *treeEntities) {
        if (!scene->IsEntityValid(treeEntity))
          continue;
        const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
        if (!tree->generate_mesh)
          continue;
        out << YAML::BeginMap;
        out << YAML::Key << "II" << YAML::Value << treeEntity.GetIndex();
        const auto gt = scene->GetDataComponent<GlobalTransform>(treeEntity);
        out << YAML::Key << "P" << YAML::Value << gt.GetPosition();
        tree->ExportTreeParts(meshGeneratorSettings, out);
        out << YAML::EndMap;
      }
      out << YAML::EndSeq;
      out << YAML::EndMap;
      std::ofstream outputFile(ymlPath.string());
      outputFile << out.c_str();
      outputFile.flush();
    } catch (const std::exception& e) {
      EVOENGINE_ERROR("Failed to save!");
    }
  }
#endif
}

bool TreePointCloudScanner::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
  bool changed = false;
  const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
  if (ImGui::TreeNodeEx("Circular Capture")) {
    static std::shared_ptr<TreePointCloudCircularCaptureSettings> captureSettings =
        std::make_shared<TreePointCloudCircularCaptureSettings>();
    captureSettings->OnInspect();
    FileUtils::SaveFile(
        "Capture", "Point Cloud", {".ply"},
        [&](const std::filesystem::path& path) {
          Capture(ecoSysLabLayer->m_meshGeneratorSettings, path, captureSettings);
        },
        false);
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Grid Capture")) {
    static std::shared_ptr<TreePointCloudGridCaptureSettings> captureSettings =
        std::make_shared<TreePointCloudGridCaptureSettings>();
    captureSettings->OnInspect();
    FileUtils::SaveFile(
        "Capture", "Point Cloud", {".ply"},
        [&](const std::filesystem::path& path) {
          Capture(ecoSysLabLayer->m_meshGeneratorSettings, path, captureSettings);
        },
        false);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Point settings")) {
    m_pointSettings.OnInspect();
    ImGui::TreePop();
  }

  return changed;
}

void TreePointCloudScanner::OnDestroy() {
  m_pointSettings = {};
}

void TreePointCloudScanner::Serialize(YAML::Emitter& out) const {
  m_pointSettings.Save("m_pointSettings", out);
}

void TreePointCloudScanner::Deserialize(const YAML::Node& in) {
  m_pointSettings.Load("m_pointSettings", in);
}
