#include "Application.hpp"
#include "ClassRegistry.hpp"

#include "EditorLayer.hpp"

#include "RenderLayer.hpp"

#ifdef DIGITAL_AGRICULTURE_PLUGIN

#  include "SorghumLayer.hpp"
using namespace digital_agriculture_plugin;
#endif
#include "WindowLayer.hpp"
#ifdef OPTIX_RAY_TRACER_PLUGIN
#  include <CUDAModule.hpp>
#  include <RayTracerLayer.hpp>
#endif

#ifdef DATASET_GENERATION_PLUGIN
#  include <SorghumPointCloudScanner.hpp>
#  include <TreePointCloudScanner.hpp>
#  include "DatasetGenerator.hpp"
using namespace dataset_generation_plugin;
#endif
#ifdef ECO_SYS_LAB_PLUGIN
#  include "EcoSysLabLayer.hpp"
#  include "ObjectRotator.hpp"
#  include "ParticlePhysics2DDemo.hpp"
#  include "Physics2DDemo.hpp"
#  include "Soil.hpp"
#  include "Tree.hpp"
#  include "TreeStructor.hpp"
using namespace eco_sys_lab_plugin;
#endif

using namespace evo_engine;

void register_classes() {
#ifdef ECO_SYS_LAB_PLUGIN
  PrivateComponentRegistration<ObjectRotator>("ObjectRotator");
  PrivateComponentRegistration<Physics2DDemo>("Physics2DDemo");
  PrivateComponentRegistration<ParticlePhysics2DDemo>("ParticlePhysics2DDemo");
  PrivateComponentRegistration<TreePointCloudScanner>("TreePointCloudScanner");
#endif
}

void register_layers(bool enable_window_layer, bool enable_editor_layer) {
  if (enable_window_layer)
    Application::PushLayer<WindowLayer>();
  if (enable_window_layer && enable_editor_layer)
    Application::PushLayer<EditorLayer>();
  Application::PushLayer<RenderLayer>();
#ifdef ECO_SYS_LAB_PLUGIN
  Application::PushLayer<EcoSysLabLayer>();
#endif
#ifdef OPTIX_RAY_TRACER_PLUGIN
  Application::PushLayer<RayTracerLayer>();
#endif
}

void start_project_windowless(const std::filesystem::path& project_path) {
  if (std::filesystem::path(project_path).extension().string() != ".eveproj") {
    EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!");
    return;
  }
  register_classes();
  register_layers(false, false);
  ApplicationInfo application_info{};
  application_info.project_path = project_path;
  Application::Initialize(application_info);
  const auto new_scene = std::dynamic_pointer_cast<Scene>(ProjectManager::GetOrCreateAsset("DigitalForestry.evescene"));
  Application::Attach(new_scene);
  Application::Start();
}

void forest_patch_point_cloud() {
#ifndef ECO_SYS_LAB_PLUGIN
  throw std::runtime_error("EcoSysLab plugin missing!");
#else
#  ifndef OPTIX_RAY_TRACER_PLUGIN
  throw std::runtime_error("OptixRayTracer plugin missing!");
#  else
#    ifndef DATASET_GENERATION_PLUGIN
  throw std::runtime_error("DatasetGeneration plugin missing!");
#    endif
#  endif
  std::filesystem::path resource_folder_path("../../../Resources");
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../Resources";
  }
  resource_folder_path = std::filesystem::absolute(resource_folder_path);

  std::filesystem::path project_path = resource_folder_path / "EcoSysLabProject" / "test.eveproj";
  start_project_windowless(project_path);

  TreeMeshGeneratorSettings tmgs{};
  tmgs.branch_y_subdivision = 0.05f;
  tmgs.trunk_y_subdivision = 0.05f;
  tmgs.enable_foliage = true;
  tmgs.vertex_color_mode = static_cast<unsigned>(TreeMeshGeneratorSettings::VertexColorMode::Junction);
  std::filesystem::path output_root = "D:\\ForestPointCloudData\\";

  std::filesystem::create_directories(output_root);

  TreePointCloudPointSettings tree_point_cloud_point_settings{};
  std::shared_ptr<TreePointCloudCircularCaptureSettings> tree_point_cloud_circular_capture_settings =
      std::make_shared<TreePointCloudCircularCaptureSettings>();
  std::shared_ptr<TreePointCloudGridCaptureSettings> tree_point_cloud_grid_capture_settings =
      std::make_shared<TreePointCloudGridCaptureSettings>();
  tree_point_cloud_point_settings.m_ballRandRadius = 0.0f;
  tree_point_cloud_point_settings.m_treePartIndex = false;
  tree_point_cloud_point_settings.m_instanceIndex = true;
  tree_point_cloud_point_settings.m_typeIndex = true;
  tree_point_cloud_point_settings.m_treePartTypeIndex = false;
  tree_point_cloud_point_settings.m_branchIndex = false;
  tree_point_cloud_point_settings.m_lineIndex = false;
  tree_point_cloud_circular_capture_settings->m_distance = 4.0f;
  tree_point_cloud_circular_capture_settings->m_height = 3.0f;

  glm::ivec2 grid_size = {17, 17};
  tree_point_cloud_grid_capture_settings->m_gridSize = {grid_size.x + 1, grid_size.y + 1};
  tree_point_cloud_grid_capture_settings->m_backpackSample = 1024;
  tree_point_cloud_grid_capture_settings->m_droneSample = 256;

  std::filesystem::path target_descriptor_folder_path =
      std::filesystem::absolute(resource_folder_path / "EcoSysLabProject" / "Digital Forestry");
  for (int index = 0; index < 256; index++) {
    for (const auto& i : std::filesystem::recursive_directory_iterator(target_descriptor_folder_path)) {
      if (i.is_regular_file() && i.path().extension().string() == ".forestpatch") {
        const auto forest_patch = std::dynamic_pointer_cast<ForestPatch>(
            ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(i.path())));
        std::filesystem::create_directories(output_root / forest_patch->GetTitle());

        std::string name = forest_patch->GetTitle() + "_" + std::to_string(index);
        std::filesystem::path target_tree_point_cloud_path = output_root / forest_patch->GetTitle() / (name + ".ply");

        tree_point_cloud_grid_capture_settings->m_gridDistance = forest_patch->grid_distance;

        DatasetGenerator::GeneratePointCloudForForestPatch(grid_size, tree_point_cloud_point_settings,
                                                           tree_point_cloud_grid_capture_settings, forest_patch, tmgs,
                                                           target_tree_point_cloud_path.string());
      }
    }
  }
#endif
}

void forest_patch_point_cloud_joined(const std::string& folder_name, const bool export_junction, const int count,
                                     const int grid_side_count) {
#ifndef ECO_SYS_LAB_PLUGIN
  throw std::runtime_error("EcoSysLab plugin missing!");
#else
#  ifndef OPTIX_RAY_TRACER_PLUGIN
  throw std::runtime_error("OptixRayTracer plugin missing!");
#  else
#    ifndef DATASET_GENERATION_PLUGIN
  throw std::runtime_error("DatasetGeneration plugin missing!");
#    endif
#  endif
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
  TreeMeshGeneratorSettings tmgs{};
  tmgs.branch_y_subdivision = 0.05f;
  tmgs.trunk_y_subdivision = 0.05f;
  tmgs.enable_foliage = true;
  tmgs.vertex_color_mode = export_junction
                               ? static_cast<unsigned>(TreeMeshGeneratorSettings::VertexColorMode::Junction)
                               : static_cast<unsigned>(TreeMeshGeneratorSettings::VertexColorMode::InternodeColor);
  std::filesystem::path output_root = "D:\\ForestPointCloudData\\";

  std::filesystem::create_directories(output_root);

  TreePointCloudPointSettings tree_point_cloud_point_settings{};
  std::shared_ptr<TreePointCloudCircularCaptureSettings> tree_point_cloud_circular_capture_settings =
      std::make_shared<TreePointCloudCircularCaptureSettings>();
  std::shared_ptr<TreePointCloudGridCaptureSettings> tree_point_cloud_grid_capture_settings =
      std::make_shared<TreePointCloudGridCaptureSettings>();
  tree_point_cloud_point_settings.m_ballRandRadius = 0.0f;
  tree_point_cloud_point_settings.m_treePartIndex = export_junction;
  tree_point_cloud_point_settings.m_instanceIndex = true;
  tree_point_cloud_point_settings.m_typeIndex = true;
  tree_point_cloud_point_settings.m_treePartTypeIndex = export_junction;
  tree_point_cloud_point_settings.m_branchIndex = false;
  tree_point_cloud_point_settings.m_lineIndex = export_junction;
  tree_point_cloud_circular_capture_settings->m_distance = 4.0f;
  tree_point_cloud_circular_capture_settings->m_height = 3.0f;

  glm::ivec2 grid_size = {grid_side_count + 2, grid_side_count + 2};
  tree_point_cloud_grid_capture_settings->m_gridSize = {grid_size.x + 1, grid_size.y + 1};
  tree_point_cloud_grid_capture_settings->m_backpackSample = 1024;
  tree_point_cloud_grid_capture_settings->m_droneSample = 256;

  std::filesystem::path target_descriptor_folder_path =
      std::filesystem::absolute(resource_folder_path / "EcoSysLabProject" / folder_name);

  for (int index = 0; index < count; index++) {
    for (const auto& i : std::filesystem::recursive_directory_iterator(target_descriptor_folder_path)) {
      if (i.is_regular_file() && i.path().extension().string() == ".forestpatch") {
        const auto forest_patch = std::dynamic_pointer_cast<ForestPatch>(
            ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(i.path())));
        std::filesystem::create_directories(output_root / forest_patch->GetTitle());

        std::string name = folder_name + "_" + std::to_string(index);
        std::filesystem::path target_tree_point_cloud_path = output_root / folder_name / (name + ".ply");

        tree_point_cloud_grid_capture_settings->m_gridDistance = forest_patch->grid_distance;

        DatasetGenerator::GeneratePointCloudForForestPatchJoinedSpecies(
            grid_size, tree_point_cloud_point_settings, tree_point_cloud_grid_capture_settings, forest_patch,
            target_descriptor_folder_path.string(), tmgs, target_tree_point_cloud_path.string());
      }
    }
  }
#endif
}

void tree_trunk_mesh() {
#ifndef ECO_SYS_LAB_PLUGIN
  throw std::runtime_error("EcoSysLab plugin missing!");
#else
#  ifndef DATASET_GENERATION_PLUGIN
  throw std::runtime_error("DatasetGeneration plugin missing!");
#  endif
  std::filesystem::path resource_folder_path("../../../Resources");
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../Resources";
  }
  resource_folder_path = std::filesystem::absolute(resource_folder_path);

  std::filesystem::path project_path = resource_folder_path / "EcoSysLabProject" / "test.eveproj";

  start_project_windowless(project_path);

  TreeMeshGeneratorSettings tmgs{};
  tmgs.x_subdivision = 0.01f;
  tmgs.branch_y_subdivision = 0.03f;
  tmgs.trunk_y_subdivision = 0.01f;
  tmgs.enable_foliage = true;

  std::filesystem::path output_root = "D:\\TreeTrunkData\\";
  std::filesystem::create_directories(output_root);

  tmgs.enable_foliage = false;
  std::filesystem::path target_descriptor_folder_path = resource_folder_path / "EcoSysLabProject" / "Trunk";
  std::vector<std::shared_ptr<TreeDescriptor> > collected_tree_descriptors{};
  for (const auto& i : std::filesystem::recursive_directory_iterator(target_descriptor_folder_path)) {
    if (i.is_regular_file() && i.path().extension().string() == ".tree") {
      for (int tree_index = 0; tree_index < 500; tree_index++) {
        std::string name = "Tree_" + std::to_string(tree_index);
        std::string info_name = "Info_" + std::to_string(tree_index);
        std::string trunk_name = "Trunk_" + std::to_string(tree_index);
        std::filesystem::path target_info_path = output_root / (info_name + ".txt");
        std::filesystem::path target_tree_mesh_path = output_root / (name + ".obj");
        std::filesystem::path target_trunk_mesh_path = output_root / (trunk_name + ".obj");
        DatasetGenerator::GenerateTreeTrunkMesh(i.path().string(), 0.08220f, 999, 50000, tmgs,
                                                target_tree_mesh_path.string(), target_trunk_mesh_path.string(),
                                                target_info_path.string());
      }
    }
  }
#endif
}

void tree_growth_mesh() {
#ifndef ECO_SYS_LAB_PLUGIN
  throw std::runtime_error("EcoSysLab plugin missing!");
#else
#  ifndef DATASET_GENERATION_PLUGIN
  throw std::runtime_error("DatasetGeneration plugin missing!");
#  endif
  std::filesystem::path resource_folder_path("../../../Resources");
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../Resources";
  }
  resource_folder_path = std::filesystem::absolute(resource_folder_path);

  std::filesystem::path project_path = resource_folder_path / "EcoSysLabProject" / "test.eveproj";

  start_project_windowless(project_path);

  TreeMeshGeneratorSettings tmgs{};
  tmgs.x_subdivision = 0.01f;
  tmgs.branch_y_subdivision = 0.03f;
  tmgs.trunk_y_subdivision = 0.01f;
  tmgs.enable_foliage = true;

  std::filesystem::path output_root = "D:\\TreeGrowth\\";
  std::filesystem::create_directories(output_root);
  tmgs.enable_foliage = true;
  std::filesystem::path target_descriptor_folder_path = resource_folder_path / "EcoSysLabProject" / "TreeDescriptors";
  std::vector<std::shared_ptr<TreeDescriptor> > collected_tree_descriptors{};
  for (const auto& i : std::filesystem::recursive_directory_iterator(target_descriptor_folder_path)) {
    if (i.is_regular_file() && i.path().extension().string() == ".tree") {
      for (int tree_index = 0; tree_index < 500; tree_index++) {
        std::string name = "Tree_" + std::to_string(tree_index);
        std::string info_name = "Info_" + std::to_string(tree_index);
        std::string trunk_name = "Trunk_" + std::to_string(tree_index);
        std::filesystem::path target_info_path = output_root / (info_name + ".txt");
        std::filesystem::path target_tree_mesh_path = output_root / (name + ".obj");
        std::filesystem::path target_trunk_mesh_path = output_root / (trunk_name + ".obj");
        std::vector<int> nodes = {3000, 6000, 9000, 12000, 15000};
        DatasetGenerator::GenerateTreeMesh(i.path().string(), 0.08220f, 999, nodes, tmgs,
                                           target_tree_mesh_path.string());
      }
    }
  }
#endif
}

int main() {
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

  const std::filesystem::path project_path = resource_folder_path / "EcoSysLabProject" / "test.eveproj";
  start_project_windowless(project_path);

  constexpr bool export_junction = true;
  forest_patch_point_cloud_joined("TreeStructor", export_junction, 5, 8);
  forest_patch_point_cloud_joined("Coniferous", export_junction, 5, 8);
  forest_patch_point_cloud_joined("Broadleaf", export_junction, 5, 8);
}
