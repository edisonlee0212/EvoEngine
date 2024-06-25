#include "Application.hpp"
#include "ClassRegistry.hpp"
#include "DatasetGenerator.hpp"
#include "EcoSysLabLayer.hpp"
#include "EditorLayer.hpp"
#include "ObjectRotator.hpp"
#include "ParticlePhysics2DDemo.hpp"
#include "RenderLayer.hpp"
#include "Soil.hpp"
#include "SorghumLayer.hpp"
#include "Tree.hpp"
#include "TreeStructor.hpp"
#include "WindowLayer.hpp"
#ifdef BUILD_WITH_RAYTRACER
#  include <CUDAModule.hpp>
#  include <RayTracerLayer.hpp>
#endif
#include <TreePointCloudScanner.hpp>

#include "DatasetGenerator.hpp"
#include "ParticlePhysics2DDemo.hpp"
#include "Physics2DDemo.hpp"
using namespace eco_sys_lab;
void register_classes() {
  ClassRegistry::RegisterPrivateComponent<ObjectRotator>("ObjectRotator");
  ClassRegistry::RegisterPrivateComponent<Physics2DDemo>("Physics2DDemo");
  ClassRegistry::RegisterPrivateComponent<ParticlePhysics2DDemo>("ParticlePhysics2DDemo");
}

void register_layers(bool enableWindowLayer, bool enableEditorLayer) {
  if (enableWindowLayer)
    Application::PushLayer<WindowLayer>();
  if (enableWindowLayer && enableEditorLayer)
    Application::PushLayer<EditorLayer>();
  Application::PushLayer<RenderLayer>();
  Application::PushLayer<EcoSysLabLayer>();
  Application::PushLayer<SorghumLayer>();
#ifdef BUILD_WITH_RAYTRACER
  Application::PushLayer<RayTracerLayer>();
#endif
}

void start_project_windowless(const std::filesystem::path& projectPath) {
  if (std::filesystem::path(projectPath).extension().string() != ".eveproj") {
    EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!");
    return;
  }
  register_classes();
  register_layers(false, false);
  ApplicationInfo applicationInfo{};
  applicationInfo.project_path = projectPath;
  Application::Initialize(applicationInfo);
  Application::Start();
}

void forest_patch_point_cloud() {
  std::filesystem::path resourceFolderPath("../../../Resources");
  if (!std::filesystem::exists(resourceFolderPath)) {
    resourceFolderPath = "../../Resources";
  }
  if (!std::filesystem::exists(resourceFolderPath)) {
    resourceFolderPath = "../Resources";
  }
  resourceFolderPath = std::filesystem::absolute(resourceFolderPath);

  std::filesystem::path project_path = resourceFolderPath / "EcoSysLabProject" / "test.eveproj";
  start_project_windowless(project_path);

  TreeMeshGeneratorSettings tmgs{};
  tmgs.branch_y_subdivision = 0.05f;
  tmgs.trunk_y_subdivision = 0.05f;
  tmgs.enable_foliage = true;
  tmgs.vertex_color_mode = static_cast<unsigned>(TreeMeshGeneratorSettings::VertexColorMode::Junction);
  std::filesystem::path output_root = "D:\\ForestPointCloudData\\";

  std::filesystem::create_directories(output_root);

  TreePointCloudPointSettings treePointCloudPointSettings{};
  std::shared_ptr<TreePointCloudCircularCaptureSettings> treePointCloudCircularCaptureSettings =
      std::make_shared<TreePointCloudCircularCaptureSettings>();
  std::shared_ptr<TreePointCloudGridCaptureSettings> treePointCloudGridCaptureSettings =
      std::make_shared<TreePointCloudGridCaptureSettings>();
  treePointCloudPointSettings.m_ballRandRadius = 0.0f;
  treePointCloudPointSettings.m_treePartIndex = false;
  treePointCloudPointSettings.m_instanceIndex = true;
  treePointCloudPointSettings.m_typeIndex = true;
  treePointCloudPointSettings.m_treePartTypeIndex = false;
  treePointCloudPointSettings.m_branchIndex = false;
  treePointCloudPointSettings.m_lineIndex = false;
  treePointCloudCircularCaptureSettings->m_distance = 4.0f;
  treePointCloudCircularCaptureSettings->m_height = 3.0f;

  glm::ivec2 gridSize = {17, 17};
  treePointCloudGridCaptureSettings->m_gridSize = {gridSize.x + 1, gridSize.y + 1};
  treePointCloudGridCaptureSettings->m_backpackSample = 1024;
  treePointCloudGridCaptureSettings->m_droneSample = 256;

  std::filesystem::path target_descriptor_folder_path =
      std::filesystem::absolute(resourceFolderPath / "EcoSysLabProject" / "Digital Forestry");
  for (int index = 0; index < 256; index++) {
    for (const auto& i : std::filesystem::recursive_directory_iterator(target_descriptor_folder_path)) {
      if (i.is_regular_file() && i.path().extension().string() == ".forestpatch") {
        const auto forestPatch = std::dynamic_pointer_cast<ForestPatch>(
            ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(i.path())));
        std::filesystem::create_directories(output_root / forestPatch->GetTitle());

        std::string name = forestPatch->GetTitle() + "_" + std::to_string(index);
        std::filesystem::path target_tree_point_cloud_path = output_root / forestPatch->GetTitle() / (name + ".ply");

        treePointCloudGridCaptureSettings->m_gridDistance = forestPatch->grid_distance;

        DatasetGenerator::GeneratePointCloudForForestPatch(gridSize, treePointCloudPointSettings,
                                                           treePointCloudGridCaptureSettings, forestPatch, tmgs,
                                                           target_tree_point_cloud_path.string());
      }
    }
  }
}

void forest_patch_point_cloud_joined(const std::string& folderName, const bool exportJunction, const int count,
                                     const int gridSideCount) {
  std::filesystem::path resourceFolderPath("../../../Resources");
  if (!std::filesystem::exists(resourceFolderPath)) {
    resourceFolderPath = "../../Resources";
  }
  if (!std::filesystem::exists(resourceFolderPath)) {
    resourceFolderPath = "../Resources";
  }
  resourceFolderPath = std::filesystem::absolute(resourceFolderPath);

  TreeMeshGeneratorSettings tmgs{};
  tmgs.branch_y_subdivision = 0.05f;
  tmgs.trunk_y_subdivision = 0.05f;
  tmgs.enable_foliage = true;
  tmgs.vertex_color_mode = exportJunction
                               ? static_cast<unsigned>(TreeMeshGeneratorSettings::VertexColorMode::Junction)
                               : static_cast<unsigned>(TreeMeshGeneratorSettings::VertexColorMode::InternodeColor);
  std::filesystem::path output_root = "D:\\ForestPointCloudData\\";

  std::filesystem::create_directories(output_root);

  TreePointCloudPointSettings treePointCloudPointSettings{};
  std::shared_ptr<TreePointCloudCircularCaptureSettings> treePointCloudCircularCaptureSettings =
      std::make_shared<TreePointCloudCircularCaptureSettings>();
  std::shared_ptr<TreePointCloudGridCaptureSettings> treePointCloudGridCaptureSettings =
      std::make_shared<TreePointCloudGridCaptureSettings>();
  treePointCloudPointSettings.m_ballRandRadius = 0.0f;
  treePointCloudPointSettings.m_treePartIndex = exportJunction;
  treePointCloudPointSettings.m_instanceIndex = true;
  treePointCloudPointSettings.m_typeIndex = true;
  treePointCloudPointSettings.m_treePartTypeIndex = exportJunction;
  treePointCloudPointSettings.m_branchIndex = false;
  treePointCloudPointSettings.m_lineIndex = exportJunction;
  treePointCloudCircularCaptureSettings->m_distance = 4.0f;
  treePointCloudCircularCaptureSettings->m_height = 3.0f;

  glm::ivec2 gridSize = {gridSideCount + 2, gridSideCount + 2};
  treePointCloudGridCaptureSettings->m_gridSize = {gridSize.x + 1, gridSize.y + 1};
  treePointCloudGridCaptureSettings->m_backpackSample = 1024;
  treePointCloudGridCaptureSettings->m_droneSample = 256;

  std::filesystem::path target_descriptor_folder_path =
      std::filesystem::absolute(resourceFolderPath / "EcoSysLabProject" / folderName);

  for (int index = 0; index < count; index++) {
    for (const auto& i : std::filesystem::recursive_directory_iterator(target_descriptor_folder_path)) {
      if (i.is_regular_file() && i.path().extension().string() == ".forestpatch") {
        const auto forestPatch = std::dynamic_pointer_cast<ForestPatch>(
            ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(i.path())));
        std::filesystem::create_directories(output_root / forestPatch->GetTitle());

        std::string name = folderName + "_" + std::to_string(index);
        std::filesystem::path target_tree_point_cloud_path = output_root / folderName / (name + ".ply");

        treePointCloudGridCaptureSettings->m_gridDistance = forestPatch->grid_distance;

        DatasetGenerator::GeneratePointCloudForForestPatchJoinedSpecies(
            gridSize, treePointCloudPointSettings, treePointCloudGridCaptureSettings, forestPatch,
            target_descriptor_folder_path.string(), tmgs, target_tree_point_cloud_path.string());
      }
    }
  }
}

void sorghum_field_point_cloud() {
  std::filesystem::path resourceFolderPath("../../../Resources");
  if (!std::filesystem::exists(resourceFolderPath)) {
    resourceFolderPath = "../../Resources";
  }
  if (!std::filesystem::exists(resourceFolderPath)) {
    resourceFolderPath = "../Resources";
  }
  resourceFolderPath = std::filesystem::absolute(resourceFolderPath);

  std::filesystem::path project_path = resourceFolderPath / "SorghumProject" / "test.eveproj";
  start_project_windowless(project_path);

  SorghumMeshGeneratorSettings sorghumMeshGeneratorSettings{};

  std::filesystem::path output_root = "D:\\SorghumPointCloudData\\";
  std::filesystem::create_directories(output_root);

  SorghumPointCloudPointSettings sorghumPointCloudPointSettings{};
  std::shared_ptr<SorghumGantryCaptureSettings> sorghumGantryCaptureSettings =
      std::make_shared<SorghumGantryCaptureSettings>();
  sorghumPointCloudPointSettings.m_ballRandRadius = 0.005f;
  sorghumPointCloudPointSettings.m_variance = 0.0f;
  sorghumPointCloudPointSettings.m_instanceIndex = true;
  sorghumPointCloudPointSettings.m_typeIndex = true;
  sorghumPointCloudPointSettings.m_leafIndex = true;

  int gridSize = 6;
  float gridDistance = 0.75f;
  float randomShift = 0.5f;
  sorghumGantryCaptureSettings->m_gridSize = {gridSize, gridSize};
  sorghumGantryCaptureSettings->m_gridDistance = {gridDistance, gridDistance};
  SorghumFieldPatch pattern{};
  pattern.m_gridSize = {gridSize, gridSize};
  pattern.m_gridDistance = {gridDistance, gridDistance};
  pattern.m_positionOffsetMean = {randomShift, randomShift};
  pattern.m_positionOffsetVariance = {0.3f, 0.3f};
  int index = 0;
  for (int i = 0; i < 4096; i++) {
    std::filesystem::path target_descriptor_folder_path =
        resourceFolderPath / "SorghumProject" / "SorghumStateGenerator";
    const auto sorghumDescriptor = std::dynamic_pointer_cast<SorghumStateGenerator>(
        ProjectManager::GetOrCreateAsset(std::filesystem::path("SorghumStateGenerator") / "Random.ssg"));
    std::string name = "Sorghum_" + std::to_string(i);
    std::filesystem::path target_tree_pointcloud_path = output_root / (name + ".ply");
    DatasetGenerator::GeneratePointCloudForSorghumPatch(pattern, sorghumDescriptor, sorghumPointCloudPointSettings,
                                                        sorghumGantryCaptureSettings, sorghumMeshGeneratorSettings,
                                                        target_tree_pointcloud_path.string());
    index++;
  }
}
void tree_trunk_mesh() {
  std::filesystem::path resourceFolderPath("../../../Resources");
  if (!std::filesystem::exists(resourceFolderPath)) {
    resourceFolderPath = "../../Resources";
  }
  if (!std::filesystem::exists(resourceFolderPath)) {
    resourceFolderPath = "../Resources";
  }
  resourceFolderPath = std::filesystem::absolute(resourceFolderPath);

  std::filesystem::path project_path = resourceFolderPath / "EcoSysLabProject" / "test.eveproj";

  start_project_windowless(project_path);

  TreeMeshGeneratorSettings tmgs{};
  tmgs.x_subdivision = 0.01f;
  tmgs.branch_y_subdivision = 0.03f;
  tmgs.trunk_y_subdivision = 0.01f;
  tmgs.enable_foliage = true;

  std::filesystem::path output_root = "D:\\TreeTrunkData\\";
  std::filesystem::create_directories(output_root);

  tmgs.enable_foliage = false;
  std::filesystem::path target_descriptor_folder_path = resourceFolderPath / "EcoSysLabProject" / "Trunk";
  std::vector<std::shared_ptr<TreeDescriptor>> collectedTreeDescriptors{};
  for (const auto& i : std::filesystem::recursive_directory_iterator(target_descriptor_folder_path)) {
    if (i.is_regular_file() && i.path().extension().string() == ".tree") {
      for (int treeIndex = 0; treeIndex < 500; treeIndex++) {
        std::string name = "Tree_" + std::to_string(treeIndex);
        std::string infoName = "Info_" + std::to_string(treeIndex);
        std::string trunkName = "Trunk_" + std::to_string(treeIndex);
        std::filesystem::path target_info_path = output_root / (infoName + ".txt");
        std::filesystem::path target_tree_mesh_path = output_root / (name + ".obj");
        std::filesystem::path target_trunk_mesh_path = output_root / (trunkName + ".obj");
        DatasetGenerator::GenerateTreeTrunkMesh(i.path().string(), 0.08220f, 999, 50000, tmgs,
                                                target_tree_mesh_path.string(), target_trunk_mesh_path.string(),
                                                target_info_path.string());
      }
    }
  }
}

void tree_growth_mesh() {
  std::filesystem::path resourceFolderPath("../../../Resources");
  if (!std::filesystem::exists(resourceFolderPath)) {
    resourceFolderPath = "../../Resources";
  }
  if (!std::filesystem::exists(resourceFolderPath)) {
    resourceFolderPath = "../Resources";
  }
  resourceFolderPath = std::filesystem::absolute(resourceFolderPath);

  std::filesystem::path project_path = resourceFolderPath / "EcoSysLabProject" / "test.eveproj";

  start_project_windowless(project_path);

  TreeMeshGeneratorSettings tmgs{};
  tmgs.x_subdivision = 0.01f;
  tmgs.branch_y_subdivision = 0.03f;
  tmgs.trunk_y_subdivision = 0.01f;
  tmgs.enable_foliage = true;

  std::filesystem::path output_root = "D:\\TreeGrowth\\";
  std::filesystem::create_directories(output_root);
  tmgs.enable_foliage = true;
  std::filesystem::path target_descriptor_folder_path = resourceFolderPath / "EcoSysLabProject" / "TreeDescriptors";
  std::vector<std::shared_ptr<TreeDescriptor>> collectedTreeDescriptors{};
  for (const auto& i : std::filesystem::recursive_directory_iterator(target_descriptor_folder_path)) {
    if (i.is_regular_file() && i.path().extension().string() == ".tree") {
      for (int treeIndex = 0; treeIndex < 500; treeIndex++) {
        std::string name = "Tree_" + std::to_string(treeIndex);
        std::string infoName = "Info_" + std::to_string(treeIndex);
        std::string trunkName = "Trunk_" + std::to_string(treeIndex);
        std::filesystem::path target_info_path = output_root / (infoName + ".txt");
        std::filesystem::path target_tree_mesh_path = output_root / (name + ".obj");
        std::filesystem::path target_trunk_mesh_path = output_root / (trunkName + ".obj");
        std::vector<int> nodes = {3000, 6000, 9000, 12000, 15000};
        DatasetGenerator::GenerateTreeMesh(i.path().string(), 0.08220f, 999, nodes, tmgs,
                                           target_tree_mesh_path.string());
      }
    }
  }
}

void apple_tree_growth() {
  // std::filesystem::path project_folder_path =
  // "C:\\Users\\62469\\Work\\TreeEngine\\EcoSysLab\\Resources\\EcoSysLabProject";
  std::filesystem::path resourceFolderPath("../../../Resources");
  if (!std::filesystem::exists(resourceFolderPath)) {
    resourceFolderPath = "../../Resources";
  }
  if (!std::filesystem::exists(resourceFolderPath)) {
    resourceFolderPath = "../Resources";
  }
  resourceFolderPath = std::filesystem::absolute(resourceFolderPath);
  std::filesystem::path project_path = resourceFolderPath / "EcoSysLabProject" / "test.eveproj";

  start_project_windowless(project_path);

  TreeMeshGeneratorSettings tmgs{};
  tmgs.x_subdivision = 0.01f;
  tmgs.branch_y_subdivision = 0.03f;
  tmgs.trunk_y_subdivision = 0.01f;
  tmgs.enable_foliage = true;

  std::filesystem::path output_root = "D:\\TreeGrowth\\";
  std::filesystem::create_directories(output_root);
  tmgs.enable_foliage = true;
  std::filesystem::path target_descriptor_path =
      resourceFolderPath / "EcoSysLabProject" / "TreeDescriptors" / "Apple.tree";
  std::vector<std::shared_ptr<TreeDescriptor>> collectedTreeDescriptors{};

  for (int treeIndex = 0; treeIndex < 500; treeIndex++) {
    std::string name = "Tree_" + std::to_string(treeIndex);
    std::string infoName = "Info_" + std::to_string(treeIndex);
    std::string trunkName = "Trunk_" + std::to_string(treeIndex);
    std::filesystem::path target_info_path = output_root / (infoName + ".txt");
    std::filesystem::path target_tree_mesh_path = output_root / (name + ".obj");
    std::filesystem::path target_trunk_mesh_path = output_root / (trunkName + ".obj");
    std::vector<int> nodes = {5000, 10000, 15000, 20000, 25000};
    DatasetGenerator::GenerateTreeMesh(target_descriptor_path.string(), 0.08220f, 999, nodes, tmgs,
                                       target_tree_mesh_path.string());
  }
}

int main() {
  // apple_tree_growth();
  // sorghum_field_point_cloud();
  std::filesystem::path resourceFolderPath("../../../Resources");
  if (!std::filesystem::exists(resourceFolderPath)) {
    resourceFolderPath = "../../Resources";
  }
  if (!std::filesystem::exists(resourceFolderPath)) {
    resourceFolderPath = "../Resources";
  }
  resourceFolderPath = std::filesystem::absolute(resourceFolderPath);

  std::filesystem::path project_path = resourceFolderPath / "EcoSysLabProject" / "test.eveproj";
  start_project_windowless(project_path);

  bool exportJunction = true;
  forest_patch_point_cloud_joined("TreeStructor", exportJunction, 5, 8);
  // forest_patch_point_cloud_joined("Coniferous", exportJunction, 1, 8);
  // forest_patch_point_cloud_joined("Broadleaf", exportJunction, 1, 8);
}
