#include "DatasetGenerator.hpp"
#include "Climate.hpp"
#include "EcoSysLabLayer.hpp"
#include "ForestDescriptor.hpp"
#include "Soil.hpp"
#include "Sorghum.hpp"
#include "SorghumLayer.hpp"
using namespace eco_sys_lab;

void DatasetGenerator::GenerateTreeTrunkMesh(const std::string& treeParametersPath, float deltaTime, int maxIterations,
                                             int maxTreeNodeCount,
                                             const TreeMeshGeneratorSettings& meshGeneratorSettings,
                                             const std::string& treeMeshOutputPath,
                                             const std::string& treeTrunkOutputPath, const std::string& treeInfoPath) {
  const auto applicationStatus = Application::GetApplicationStatus();
  if (applicationStatus == ApplicationStatus::NoProject) {
    EVOENGINE_ERROR("No project!");
    return;
  }
  if (applicationStatus == ApplicationStatus::OnDestroy) {
    EVOENGINE_ERROR("Application is destroyed!");
    return;
  }
  if (applicationStatus == ApplicationStatus::Uninitialized) {
    EVOENGINE_ERROR("Application not uninitialized!");
    return;
  }
  const auto scene = Application::GetActiveScene();
  const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
  if (!ecoSysLabLayer) {
    EVOENGINE_ERROR("Application doesn't contain EcoSysLab layer!");
    return;
  }
  std::shared_ptr<TreeDescriptor> treeDescriptor;
  if (ProjectManager::IsInProjectFolder(treeParametersPath)) {
    treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(
        ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(treeParametersPath)));
  } else {
    EVOENGINE_ERROR("Tree Descriptor doesn't exist!");
    return;
  }
  if (const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      treeEntities && !treeEntities->empty()) {
    for (const auto& treeEntity : *treeEntities) {
      scene->DeleteEntity(treeEntity);
    }
  }

  const auto treeEntity = scene->CreateEntity("Tree");
  const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();

  tree->tree_descriptor = treeDescriptor;
  tree->tree_model.tree_growth_settings.use_space_colonization = false;
  Application::Loop();
  ecoSysLabLayer->m_simulationSettings.delta_time = deltaTime;
  for (int i = 0; i < maxIterations; i++) {
    ecoSysLabLayer->Simulate();
    if (tree->tree_model.RefShootSkeleton().PeekSortedNodeList().size() >= maxTreeNodeCount) {
      break;
    }
  }
  tree->GenerateGeometryEntities(meshGeneratorSettings);
  Application::Loop();
  tree->ExportTrunkObj(treeTrunkOutputPath, meshGeneratorSettings);
  tree->ExportObj(treeMeshOutputPath, meshGeneratorSettings);
  Application::Loop();
  float trunkHeight = 0.0f;
  float baseDiameter = 0.0f;
  float topDiameter = 0.0f;
  const auto& skeleton = tree->tree_model.RefShootSkeleton();
  const auto& sortedInternodeList = skeleton.PeekSortedNodeList();
  if (sortedInternodeList.size() > 1) {
    std::unordered_set<SkeletonNodeHandle> trunkHandles{};
    for (const auto& nodeHandle : sortedInternodeList) {
      const auto& node = skeleton.PeekNode(nodeHandle);
      trunkHandles.insert(nodeHandle);
      if (node.PeekChildHandles().size() > 1) {
        trunkHeight = node.info.GetGlobalEndPosition().y;
        topDiameter = node.info.thickness;
        break;
      }
    }
    baseDiameter = skeleton.PeekNode(0).info.thickness;
    std::ofstream of;
    of.open(treeInfoPath, std::ofstream::out | std::ofstream::trunc);
    std::stringstream data;
    data << "TrunkHeight " << std::to_string(trunkHeight) << "\n";
    data << "TrunkBaseDiameter " << std::to_string(baseDiameter) << "\n";
    data << "TrunkTopDiameter " << std::to_string(topDiameter) << "\n";

    data << "TreeBoundingBoxMinX " << std::to_string(skeleton.min.x) << "\n";
    data << "TreeBoundingBoxMinY " << std::to_string(skeleton.min.y) << "\n";
    data << "TreeBoundingBoxMinZ " << std::to_string(skeleton.min.z) << "\n";
    data << "TreeBoundingBoxMaxX " << std::to_string(skeleton.max.x) << "\n";
    data << "TreeBoundingBoxMaxY " << std::to_string(skeleton.max.y) << "\n";
    data << "TreeBoundingBoxMaxZ " << std::to_string(skeleton.max.z) << "\n";
    const auto result = data.str();
    of.write(result.c_str(), result.size());
    of.flush();
  }
  scene->DeleteEntity(treeEntity);
  Application::Loop();
}

void DatasetGenerator::GenerateTreeMesh(const std::string& treeParametersPath, float deltaTime, int maxIterations,
                                        int maxTreeNodeCount, const TreeMeshGeneratorSettings& meshGeneratorSettings,
                                        const std::string& treeMeshOutputPath) {
  const auto applicationStatus = Application::GetApplicationStatus();
  if (applicationStatus == ApplicationStatus::NoProject) {
    EVOENGINE_ERROR("No project!");
    return;
  }
  if (applicationStatus == ApplicationStatus::OnDestroy) {
    EVOENGINE_ERROR("Application is destroyed!");
    return;
  }
  if (applicationStatus == ApplicationStatus::Uninitialized) {
    EVOENGINE_ERROR("Application not uninitialized!");
    return;
  }
  const auto scene = Application::GetActiveScene();
  const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
  if (!ecoSysLabLayer) {
    EVOENGINE_ERROR("Application doesn't contain EcoSysLab layer!");
    return;
  }
  std::shared_ptr<TreeDescriptor> treeDescriptor;
  if (ProjectManager::IsInProjectFolder(treeParametersPath)) {
    treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(
        ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(treeParametersPath)));
  } else {
    EVOENGINE_ERROR("Tree Descriptor doesn't exist!");
    return;
  }
  if (const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      treeEntities && !treeEntities->empty()) {
    for (const auto& treeEntity : *treeEntities) {
      scene->DeleteEntity(treeEntity);
    }
  }

  const auto treeEntity = scene->CreateEntity("Tree");
  const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();

  tree->tree_descriptor = treeDescriptor;
  tree->tree_model.tree_growth_settings.use_space_colonization = false;
  Application::Loop();
  ecoSysLabLayer->m_simulationSettings.delta_time = deltaTime;

  for (int i = 0; i < maxIterations; i++) {
    ecoSysLabLayer->Simulate();
    if (tree->tree_model.RefShootSkeleton().PeekSortedNodeList().size() >= maxTreeNodeCount) {
      break;
    }
  }
  tree->GenerateGeometryEntities(meshGeneratorSettings);
  Application::Loop();
  tree->ExportObj(treeMeshOutputPath, meshGeneratorSettings);
  Application::Loop();
  scene->DeleteEntity(treeEntity);
  Application::Loop();
}

void DatasetGenerator::GenerateTreeMesh(const std::string& treeParametersPath, float deltaTime, int maxIterations,
                                        std::vector<int> targetTreeNodeCount,
                                        const TreeMeshGeneratorSettings& meshGeneratorSettings,
                                        const std::string& treeMeshOutputPath) {
  const auto applicationStatus = Application::GetApplicationStatus();
  if (applicationStatus == ApplicationStatus::NoProject) {
    EVOENGINE_ERROR("No project!");
    return;
  }
  if (applicationStatus == ApplicationStatus::OnDestroy) {
    EVOENGINE_ERROR("Application is destroyed!");
    return;
  }
  if (applicationStatus == ApplicationStatus::Uninitialized) {
    EVOENGINE_ERROR("Application not uninitialized!");
    return;
  }
  const auto scene = Application::GetActiveScene();
  const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
  if (!ecoSysLabLayer) {
    EVOENGINE_ERROR("Application doesn't contain EcoSysLab layer!");
    return;
  }
  std::shared_ptr<TreeDescriptor> treeDescriptor;
  if (ProjectManager::IsInProjectFolder(treeParametersPath)) {
    treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(
        ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(treeParametersPath)));
  } else {
    EVOENGINE_ERROR("Tree Descriptor doesn't exist!");
    return;
  }
  if (const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      treeEntities && !treeEntities->empty()) {
    for (const auto& treeEntity : *treeEntities) {
      scene->DeleteEntity(treeEntity);
    }
  }

  const auto treeEntity = scene->CreateEntity("Tree");
  const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();

  tree->tree_descriptor = treeDescriptor;
  tree->tree_model.tree_growth_settings.use_space_colonization = false;
  Application::Loop();
  int testIndex = 0;
  std::filesystem::path basePath = treeMeshOutputPath;
  ecoSysLabLayer->m_simulationSettings.delta_time = deltaTime;

  for (int i = 0; i < maxIterations; i++) {
    ecoSysLabLayer->Simulate();
    if (tree->tree_model.RefShootSkeleton().PeekSortedNodeList().size() >= targetTreeNodeCount[testIndex]) {
      auto copyPath = basePath;
      tree->GenerateGeometryEntities(meshGeneratorSettings);
      Application::Loop();
      copyPath.replace_filename(basePath.replace_extension("").string() + "_" + std::to_string(testIndex) + ".obj");
      tree->ExportObj(copyPath, meshGeneratorSettings);
      testIndex++;
    }
    if (testIndex == targetTreeNodeCount.size())
      break;
  }

  Application::Loop();
  scene->DeleteEntity(treeEntity);
  Application::Loop();
}

void DatasetGenerator::GeneratePointCloudForTree(const TreePointCloudPointSettings& pointSettings,
                                                 const std::shared_ptr<PointCloudCaptureSettings>& captureSettings,
                                                 const std::string& treeParametersPath, const float deltaTime,
                                                 const int maxIterations, const int maxTreeNodeCount,
                                                 const TreeMeshGeneratorSettings& meshGeneratorSettings,
                                                 const std::string& pointCloudOutputPath, bool exportTreeMesh,
                                                 const std::string& treeMeshOutputPath) {
  const auto applicationStatus = Application::GetApplicationStatus();
  if (applicationStatus == ApplicationStatus::NoProject) {
    EVOENGINE_ERROR("No project!");
    return;
  }
  if (applicationStatus == ApplicationStatus::OnDestroy) {
    EVOENGINE_ERROR("Application is destroyed!");
    return;
  }
  if (applicationStatus == ApplicationStatus::Uninitialized) {
    EVOENGINE_ERROR("Application not uninitialized!");
    return;
  }
  const auto scene = Application::GetActiveScene();
  const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
  if (!ecoSysLabLayer) {
    EVOENGINE_ERROR("Application doesn't contain EcoSysLab layer!");
    return;
  }
  std::shared_ptr<TreeDescriptor> treeDescriptor;
  if (ProjectManager::IsInProjectFolder(treeParametersPath)) {
    treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(
        ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(treeParametersPath)));
  } else {
    EVOENGINE_ERROR("Tree Descriptor doesn't exist!");
    return;
  }
  if (const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      treeEntities && !treeEntities->empty()) {
    for (const auto& treeEntity : *treeEntities) {
      scene->DeleteEntity(treeEntity);
    }
  }

  const auto treeEntity = scene->CreateEntity("Tree");
  const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();

  tree->tree_descriptor = treeDescriptor;
  tree->tree_model.tree_growth_settings.use_space_colonization = false;
  Application::Loop();
  ecoSysLabLayer->m_simulationSettings.delta_time = deltaTime;

  for (int i = 0; i < maxIterations; i++) {
    ecoSysLabLayer->Simulate();
    if (tree->tree_model.RefShootSkeleton().PeekSortedNodeList().size() >= maxTreeNodeCount) {
      break;
    }
  }
  tree->GenerateGeometryEntities(meshGeneratorSettings);
  Application::Loop();
  if (exportTreeMesh) {
    tree->ExportObj(treeMeshOutputPath, meshGeneratorSettings);
  }
  Application::Loop();
  const auto scannerEntity = scene->CreateEntity("Scanner");
  const auto scanner = scene->GetOrSetPrivateComponent<TreePointCloudScanner>(scannerEntity).lock();
  scanner->m_pointSettings = pointSettings;
  Application::Loop();
  scanner->Capture(meshGeneratorSettings, pointCloudOutputPath, captureSettings);
  Application::Loop();
  scene->DeleteEntity(treeEntity);
  scene->DeleteEntity(scannerEntity);
  Application::Loop();
}

void DatasetGenerator::GeneratePointCloudForForest(
    const int gridSize, const float gridDistance, const float randomShift,
    const TreePointCloudPointSettings& pointSettings, const std::shared_ptr<PointCloudCaptureSettings>& captureSettings,
    const std::string& treeParametersFolderPath, float deltaTime, int maxIterations, int maxTreeNodeCount,
    const TreeMeshGeneratorSettings& meshGeneratorSettings, const std::string& pointCloudOutputPath) {
  const auto applicationStatus = Application::GetApplicationStatus();
  if (applicationStatus == ApplicationStatus::NoProject) {
    EVOENGINE_ERROR("No project!");
    return;
  }
  if (applicationStatus == ApplicationStatus::OnDestroy) {
    EVOENGINE_ERROR("Application is destroyed!");
    return;
  }
  if (applicationStatus == ApplicationStatus::Uninitialized) {
    EVOENGINE_ERROR("Application not uninitialized!");
    return;
  }
  const auto scene = Application::GetActiveScene();
  const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
  if (!ecoSysLabLayer) {
    EVOENGINE_ERROR("Application doesn't contain EcoSysLab layer!");
    return;
  }
  std::shared_ptr<ForestDescriptor> forestPatch = ProjectManager::CreateTemporaryAsset<ForestDescriptor>();
  std::shared_ptr<Soil> soil;
  const std::vector<Entity>* soilEntities = scene->UnsafeGetPrivateComponentOwnersList<Soil>();
  if (soilEntities && !soilEntities->empty()) {
    soil = scene->GetOrSetPrivateComponent<Soil>(soilEntities->at(0)).lock();
  }
  if (!soil) {
    EVOENGINE_ERROR("No soil in scene!");
    return;
  }

  soil->RandomOffset(0, 99999);
  soil->GenerateMesh(0.0f, 0.0f);
  if (const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      treeEntities && !treeEntities->empty()) {
    for (const auto& treeEntity : *treeEntities) {
      scene->DeleteEntity(treeEntity);
    }
  }
  forestPatch->m_treeGrowthSettings.use_space_colonization = false;

  Application::Loop();

  forestPatch->SetupGrid({gridSize, gridSize}, gridDistance, randomShift);
  forestPatch->ApplyTreeDescriptors(treeParametersFolderPath, {1.f});
  forestPatch->InstantiatePatch(false);

  ecoSysLabLayer->m_simulationSettings.max_node_count = maxTreeNodeCount;
  ecoSysLabLayer->m_simulationSettings.delta_time = deltaTime;

  for (int i = 0; i < maxIterations; i++) {
    ecoSysLabLayer->Simulate();
  }
  ecoSysLabLayer->GenerateMeshes(meshGeneratorSettings);
  Application::Loop();
  const auto scannerEntity = scene->CreateEntity("Scanner");
  const auto scanner = scene->GetOrSetPrivateComponent<TreePointCloudScanner>(scannerEntity).lock();
  scanner->m_pointSettings = pointSettings;
  Application::Loop();
  scanner->Capture(meshGeneratorSettings, pointCloudOutputPath, captureSettings);

  if (const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      treeEntities && !treeEntities->empty()) {
    for (const auto& treeEntity : *treeEntities) {
      scene->DeleteEntity(treeEntity);
    }
  }
  scene->DeleteEntity(scannerEntity);
  Application::Loop();
}

void DatasetGenerator::GeneratePointCloudForForestPatch(
    const glm::ivec2& gridSize, const TreePointCloudPointSettings& pointSettings,
    const std::shared_ptr<PointCloudCaptureSettings>& captureSettings, const std::shared_ptr<ForestPatch>& forestPatch,
    const TreeMeshGeneratorSettings& meshGeneratorSettings, const std::string& pointCloudOutputPath) {
  const auto applicationStatus = Application::GetApplicationStatus();
  if (applicationStatus == ApplicationStatus::NoProject) {
    EVOENGINE_ERROR("No project!");
    return;
  }
  if (applicationStatus == ApplicationStatus::OnDestroy) {
    EVOENGINE_ERROR("Application is destroyed!");
    return;
  }
  if (applicationStatus == ApplicationStatus::Uninitialized) {
    EVOENGINE_ERROR("Application not uninitialized!");
    return;
  }
  const auto scene = Application::GetActiveScene();
  const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
  if (!ecoSysLabLayer) {
    EVOENGINE_ERROR("Application doesn't contain EcoSysLab layer!");
    return;
  }
  std::shared_ptr<Soil> soil;
  const std::vector<Entity>* soilEntities = scene->UnsafeGetPrivateComponentOwnersList<Soil>();
  if (soilEntities && !soilEntities->empty()) {
    soil = scene->GetOrSetPrivateComponent<Soil>(soilEntities->at(0)).lock();
  }
  if (!soil) {
    EVOENGINE_ERROR("No soil in scene!");
    return;
  }

  soil->RandomOffset(0, 99999);
  soil->GenerateMesh(0.0f, 0.0f);
  if (const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      treeEntities && !treeEntities->empty()) {
    for (const auto& treeEntity : *treeEntities) {
      scene->DeleteEntity(treeEntity);
    }
    ecoSysLabLayer->ResetAllTrees(treeEntities);
  } else {
    ecoSysLabLayer->ResetAllTrees(treeEntities);
  }

  forestPatch->tree_growth_settings.use_space_colonization = false;

  Application::Loop();

  const auto forest = forestPatch->InstantiatePatch(gridSize, true);

  while (ecoSysLabLayer->GetSimulatedTime() < forestPatch->simulation_time) {
    ecoSysLabLayer->Simulate();
  }
  ecoSysLabLayer->GenerateMeshes(meshGeneratorSettings);
  Application::Loop();
  const auto scannerEntity = scene->CreateEntity("Scanner");
  const auto scanner = scene->GetOrSetPrivateComponent<TreePointCloudScanner>(scannerEntity).lock();
  scanner->m_pointSettings = pointSettings;
  Application::Loop();
  scanner->Capture(meshGeneratorSettings, pointCloudOutputPath, captureSettings);
  Application::Loop();
  scene->DeleteEntity(forest);
  scene->DeleteEntity(scannerEntity);
  Application::Loop();
}

void DatasetGenerator::GeneratePointCloudForForestPatchJoinedSpecies(
    const glm::ivec2& gridSize, const TreePointCloudPointSettings& pointSettings,
    const std::shared_ptr<PointCloudCaptureSettings>& captureSettings, const std::shared_ptr<ForestPatch>& forestPatch,
    const std::string& speciesFolderPath, const TreeMeshGeneratorSettings& meshGeneratorSettings,
    const std::string& pointCloudOutputPath) {
  const auto applicationStatus = Application::GetApplicationStatus();
  if (applicationStatus == ApplicationStatus::NoProject) {
    EVOENGINE_ERROR("No project!");
    return;
  }
  if (applicationStatus == ApplicationStatus::OnDestroy) {
    EVOENGINE_ERROR("Application is destroyed!");
    return;
  }
  if (applicationStatus == ApplicationStatus::Uninitialized) {
    EVOENGINE_ERROR("Application not uninitialized!");
    return;
  }
  const auto scene = Application::GetActiveScene();
  const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
  if (!ecoSysLabLayer) {
    EVOENGINE_ERROR("Application doesn't contain EcoSysLab layer!");
    return;
  }
  std::shared_ptr<Soil> soil;
  const std::vector<Entity>* soilEntities = scene->UnsafeGetPrivateComponentOwnersList<Soil>();
  if (soilEntities && !soilEntities->empty()) {
    soil = scene->GetOrSetPrivateComponent<Soil>(soilEntities->at(0)).lock();
  }
  if (!soil) {
    EVOENGINE_ERROR("No soil in scene!");
    return;
  }

  soil->RandomOffset(0, 99999);
  soil->GenerateMesh(0.0f, 0.0f);
  if (const std::vector<Entity>* treeEntities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
      treeEntities && !treeEntities->empty()) {
    for (const auto& treeEntity : *treeEntities) {
      scene->DeleteEntity(treeEntity);
    }
    ecoSysLabLayer->ResetAllTrees(treeEntities);
  } else {
    ecoSysLabLayer->ResetAllTrees(treeEntities);
  }

  forestPatch->tree_growth_settings.use_space_colonization = false;

  Application::Loop();
  Entity forest;

  std::vector<std::pair<TreeGrowthSettings, std::shared_ptr<TreeDescriptor>>> treeDescriptors;
  for (const auto& i : std::filesystem::recursive_directory_iterator(speciesFolderPath)) {
    if (i.is_regular_file() && i.path().extension().string() == ".tree") {
      if (const auto treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(
              ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(i.path())))) {
        treeDescriptors.emplace_back(std::make_pair(forestPatch->tree_growth_settings, treeDescriptor));
      }
    }
  }
  if (!treeDescriptors.empty()) {
    forest = forestPatch->InstantiatePatch(treeDescriptors, gridSize, true);
  }

  while (ecoSysLabLayer->GetSimulatedTime() < forestPatch->simulation_time) {
    ecoSysLabLayer->Simulate();
  }
  ecoSysLabLayer->GenerateMeshes(meshGeneratorSettings);
  Application::Loop();
  const auto scannerEntity = scene->CreateEntity("Scanner");
  const auto scanner = scene->GetOrSetPrivateComponent<TreePointCloudScanner>(scannerEntity).lock();
  scanner->m_pointSettings = pointSettings;
  Application::Loop();
  scanner->Capture(meshGeneratorSettings, pointCloudOutputPath, captureSettings);
  Application::Loop();
  if (scene->IsEntityValid(forest))
    scene->DeleteEntity(forest);
  scene->DeleteEntity(scannerEntity);
  Application::Loop();
}

void DatasetGenerator::GeneratePointCloudForSorghumPatch(
    const SorghumFieldPatch& pattern, const std::shared_ptr<SorghumStateGenerator>& sorghumDescriptor,
    const SorghumPointCloudPointSettings& pointSettings,
    const std::shared_ptr<PointCloudCaptureSettings>& captureSettings,
    const SorghumMeshGeneratorSettings& sorghumMeshGeneratorSettings, const std::string& pointCloudOutputPath) {
  const auto applicationStatus = Application::GetApplicationStatus();
  if (applicationStatus == ApplicationStatus::NoProject) {
    EVOENGINE_ERROR("No project!");
    return;
  }
  if (applicationStatus == ApplicationStatus::OnDestroy) {
    EVOENGINE_ERROR("Application is destroyed!");
    return;
  }
  if (applicationStatus == ApplicationStatus::Uninitialized) {
    EVOENGINE_ERROR("Application not uninitialized!");
    return;
  }
  const auto scene = Application::GetActiveScene();
  const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
  if (!ecoSysLabLayer) {
    EVOENGINE_ERROR("Application doesn't contain EcoSysLab layer!");
    return;
  }
  std::shared_ptr<Soil> soil;
  const std::vector<Entity>* soilEntities = scene->UnsafeGetPrivateComponentOwnersList<Soil>();
  if (soilEntities && !soilEntities->empty()) {
    soil = scene->GetOrSetPrivateComponent<Soil>(soilEntities->at(0)).lock();
  }
  if (!soil) {
    EVOENGINE_ERROR("No soil in scene!");
    return;
  }
  soil->RandomOffset(0, 99999);
  soil->GenerateMesh(0.0f, 0.0f);
  Application::Loop();
  const auto sorghumField = ProjectManager::CreateTemporaryAsset<SorghumField>();
  std::vector<glm::mat4> matricesList;
  pattern.GenerateField(matricesList);
  sorghumField->m_matrices.resize(matricesList.size());
  for (int i = 0; i < matricesList.size(); i++) {
    sorghumField->m_matrices[i] = {sorghumDescriptor, matricesList[i]};
  }

  const auto field = sorghumField->InstantiateField();
  Application::GetLayer<SorghumLayer>()->GenerateMeshForAllSorghums(sorghumMeshGeneratorSettings);
  Application::Loop();
  const auto scannerEntity = scene->CreateEntity("Scanner");
  const auto scanner = scene->GetOrSetPrivateComponent<SorghumPointCloudScanner>(scannerEntity).lock();
  scanner->m_sorghumPointCloudPointSettings = pointSettings;
  Application::Loop();
  scanner->Capture(pointCloudOutputPath, captureSettings);
  scene->DeleteEntity(field);
  scene->DeleteEntity(scannerEntity);
  Application::Loop();
}
