#include "pybind11/pybind11.h"
#include "pybind11/stl/filesystem.h"
#include "AnimationPlayer.hpp"
#include "Application.hpp"
#include "ClassRegistry.hpp"
#include "WindowLayer.hpp"
#include "RenderLayer.hpp"
#include "EditorLayer.hpp"
#include "MeshRenderer.hpp"
#include "PlayerController.hpp"
#include "Prefab.hpp"
#include "Times.hpp"
#include "PostProcessingStack.hpp"
#include "ProjectManager.hpp"
#include "ClassRegistry.hpp"
#include "TreeModel.hpp"
#include "Tree.hpp"
#include "Soil.hpp"
#include "Climate.hpp"
#include "EcoSysLabLayer.hpp"
#include "RadialBoundingVolume.hpp"
#include "HeightField.hpp"
#include "ObjectRotator.hpp"
#include "SorghumLayer.hpp"
#include "TreeStructor.hpp"
#include "Scene.hpp"
#ifdef BUILD_WITH_RAYTRACER
#include <CUDAModule.hpp>
#include <RayTracerLayer.hpp>
#endif
#include <TreePointCloudScanner.hpp>

#include "DatasetGenerator.hpp"
#include "FoliageDescriptor.hpp"
#include "ParticlePhysics2DDemo.hpp"
#include "Physics2DDemo.hpp"

using namespace evo_engine;
using namespace eco_sys_lab;

namespace py = pybind11;

void register_classes() {
	ClassRegistry::RegisterPrivateComponent<ObjectRotator>("ObjectRotator");
	ClassRegistry::RegisterPrivateComponent<Physics2DDemo>("Physics2DDemo");
	ClassRegistry::RegisterPrivateComponent<ParticlePhysics2DDemo>("ParticlePhysics2DDemo");
}

void push_window_layer() {
	Application::PushLayer<WindowLayer>();
}
void push_editor_layer() {
	Application::PushLayer<EditorLayer>();
}
void push_render_layer() {
	Application::PushLayer<RenderLayer>();
}
void push_ecosyslab_layer() {
	Application::PushLayer<EcoSysLabLayer>();
}
void push_sorghum_layer() {
	Application::PushLayer<SorghumLayer>();
}

void register_layers(bool enableWindowLayer, bool enableEditorLayer)
{
	if (enableWindowLayer) Application::PushLayer<WindowLayer>();
	if (enableWindowLayer && enableEditorLayer) Application::PushLayer<EditorLayer>();
	Application::PushLayer<RenderLayer>();
	Application::PushLayer<EcoSysLabLayer>();
	Application::PushLayer<SorghumLayer>();
#ifdef BUILD_WITH_RAYTRACER
	Application::PushLayer<RayTracerLayer>();
#endif
}

void start_project_windowless(const std::filesystem::path& projectPath)
{
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

void start_project(const std::filesystem::path& projectPath)
{
	if (!projectPath.empty()) {
		if (std::filesystem::path(projectPath).extension().string() != ".eveproj") {
			EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!");
			return;
		}
	}
	register_classes();
	register_layers(true, true);
	ApplicationInfo applicationInfo{};
	applicationInfo.project_path = projectPath;
	Application::Initialize(applicationInfo);
	Application::Start();
}

void scene_capture(
	const float posX, const float posY, const float posZ,
	const float angleX, const float angleY, const float angleZ,
	const int resolutionX, const int resolutionY, bool whiteBackground, const std::string& outputPath)
{
	if (resolutionX <= 0 || resolutionY <= 0)
	{
		EVOENGINE_ERROR("Resolution error!");
		return;
	}

	const auto scene = Application::GetActiveScene();
	if (!scene)
	{
		EVOENGINE_ERROR("No active scene!");
		return;
	}
	auto mainCamera = scene->main_camera.Get<Camera>();
	Entity mainCameraEntity;
	bool tempCamera = false;
	if (!mainCamera)
	{
		mainCameraEntity = scene->CreateEntity("Main Camera");
		mainCamera = scene->GetOrSetPrivateComponent<Camera>(mainCameraEntity).lock();
		scene->main_camera = mainCamera;
		tempCamera = true;
	}
	else
	{
		mainCameraEntity = mainCamera->GetOwner();
	}
	auto globalTransform = scene->GetDataComponent<GlobalTransform>(mainCameraEntity);
	const auto originalTransform = globalTransform;
	globalTransform.SetPosition({ posX, posY, posZ });
	globalTransform.SetEulerRotation(glm::radians(glm::vec3(angleX, angleY, angleZ)));
	scene->SetDataComponent(mainCameraEntity, globalTransform);
	mainCamera->Resize({ resolutionX, resolutionY });
	const auto useClearColor = mainCamera->use_clear_color;
	const auto clearColor = mainCamera->clear_color;
	if (whiteBackground)
	{
		mainCamera->use_clear_color = true;
		mainCamera->clear_color = glm::vec3(1, 1, 1);
	}
	Application::Loop();
	mainCamera->GetRenderTexture()->StoreToPng(outputPath);
	if (tempCamera)
	{
		scene->DeleteEntity(mainCameraEntity);
	}
	else
	{
		scene->SetDataComponent(mainCameraEntity, originalTransform);
		if (whiteBackground)
		{
			mainCamera->use_clear_color = useClearColor;
			mainCamera->clear_color = clearColor;
		}
	}

	EVOENGINE_LOG("Exported image to " + outputPath);
}

Entity import_tree_pointcloud(const std::string& yamlPath)
{
	const auto scene = Application::GetActiveScene();
	const auto retVal = scene->CreateEntity("TreeStructor");
	const auto treePointCloud = scene->GetOrSetPrivateComponent<TreeStructor>(retVal).lock();
	treePointCloud->ImportGraph(yamlPath);
	return retVal;
}

void tree_structor(const std::string& yamlPath,
	const ConnectivityGraphSettings& connectivityGraphSettings,
	const ReconstructionSettings& reconstructionSettings,
	const TreeMeshGeneratorSettings& meshGeneratorSettings,
	const std::string& meshPath) {
	const auto scene = Application::GetActiveScene();
	const auto tempEntity = scene->CreateEntity("Temp");
	const auto treePointCloud = scene->GetOrSetPrivateComponent<TreeStructor>(tempEntity).lock();
	
	treePointCloud->connectivity_graph_settings = connectivityGraphSettings;
	treePointCloud->reconstruction_settings = reconstructionSettings;
	treePointCloud->ImportGraph(yamlPath);
	treePointCloud->EstablishConnectivityGraph();
	treePointCloud->BuildSkeletons();
	treePointCloud->ExportForestOBJ(meshGeneratorSettings, meshPath);
	EVOENGINE_LOG("Exported forest as OBJ");
	scene->DeleteEntity(tempEntity);
}
void yaml_visualization(const std::string& yamlPath,
	const ConnectivityGraphSettings& connectivityGraphSettings,
	const ReconstructionSettings& reconstructionSettings,
	const TreeMeshGeneratorSettings& meshGeneratorSettings,
	const float posX, const float posY, const float posZ,
	const float angleX, const float angleY, const float angleZ,
	const int resolutionX, const int resolutionY, const std::string& outputPath)
{
	const auto scene = Application::GetActiveScene();
	const auto tempEntity = scene->CreateEntity("Temp");
	const auto treePointCloud = scene->GetOrSetPrivateComponent<TreeStructor>(tempEntity).lock();
	treePointCloud->connectivity_graph_settings = connectivityGraphSettings;
	treePointCloud->reconstruction_settings = reconstructionSettings;
	treePointCloud->ImportGraph(yamlPath);
	treePointCloud->EstablishConnectivityGraph();
	treePointCloud->BuildSkeletons();
	treePointCloud->GenerateForest();
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	ecoSysLabLayer->GenerateMeshes(meshGeneratorSettings);
	scene_capture(posX, posY, posZ, angleX, angleY, angleZ, resolutionX, resolutionY, true, outputPath);
	scene->DeleteEntity(tempEntity);
}

void voxel_space_colonization_tree_data(
	const float radius,
	const std::string& binvoxPath,
	const std::string& treeParametersPath,
	const float deltaTime,
	const int iterations,
	const TreeMeshGeneratorSettings& meshGeneratorSettings,
	bool exportTreeMesh,
	const std::string& treeMeshOutputPath,
	bool exportTreeIO,
	const std::string& treeIOOutputPath,
	bool exportRadialBoundingVolume,
	const std::string& radialBoundingVolumeOutputPath,
	bool exportRadialBoundingVolumeMesh,
	const std::string& radialBoundingVolumeMeshOutputPath
)
{
	const auto applicationStatus = Application::GetApplicationStatus();
	if (applicationStatus == ApplicationStatus::NoProject)
	{
		EVOENGINE_ERROR("No project!");
		return;
	}
	if (applicationStatus == ApplicationStatus::OnDestroy)
	{
		EVOENGINE_ERROR("Application is destroyed!");
		return;
	}
	if (applicationStatus == ApplicationStatus::Uninitialized)
	{
		EVOENGINE_ERROR("Application not uninitialized!");
		return;
	}
	const auto scene = Application::GetActiveScene();
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	if (!ecoSysLabLayer)
	{
		EVOENGINE_ERROR("Application doesn't contain EcoSysLab layer!");
		return;
	}
	std::shared_ptr<Soil> soil;
	std::shared_ptr<Climate> climate;

	const std::vector<Entity>* soilEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Soil>();
	if (soilEntities && !soilEntities->empty()) {
		soil = scene->GetOrSetPrivateComponent<Soil>(soilEntities->at(0)).lock();
	}
	if (!soil)
	{
		EVOENGINE_ERROR("No soil in scene!");
		return;
	}
	const std::vector<Entity>* climateEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Climate>();
	if (climateEntities && !climateEntities->empty()) {
		climate = scene->GetOrSetPrivateComponent<Climate>(climateEntities->at(0)).lock();
	}
	if (!climate)
	{
		EVOENGINE_ERROR("No climate in scene!");
		return;
	}

	const auto tempEntity = scene->CreateEntity("Temp");
	const auto tree = scene->GetOrSetPrivateComponent<Tree>(tempEntity).lock();
	tree->soil = soil;
	tree->climate = climate;
	std::shared_ptr<TreeDescriptor> treeDescriptor;
	if (ProjectManager::IsInProjectFolder(treeParametersPath))
	{
		treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(treeParametersPath)));
	}
	else {
		treeDescriptor = ProjectManager::CreateTemporaryAsset<TreeDescriptor>();
	}
	tree->tree_descriptor = treeDescriptor;
	auto& occupancyGrid = tree->tree_model.tree_occupancy_grid;
	VoxelGrid<TreeOccupancyGridBasicData> inputGrid{};
	if (tree->ParseBinvox(binvoxPath, inputGrid, 1.f))
	{
		occupancyGrid.Initialize(inputGrid,
			glm::vec3(-radius, 0, -radius),
			glm::vec3(radius, 2.0f * radius, radius),
			treeDescriptor->shoot_descriptor.Get<ShootDescriptor>()->internode_length,
			tree->tree_model.tree_growth_settings.space_colonization_removal_distance_factor,
			tree->tree_model.tree_growth_settings.space_colonization_theta,
			tree->tree_model.tree_growth_settings.space_colonization_detection_distance_factor);
	}
	tree->tree_model.tree_growth_settings.use_space_colonization = true;
	tree->tree_model.tree_growth_settings.space_colonization_auto_resize = false;

	ecoSysLabLayer->m_simulationSettings.delta_time = deltaTime;

	Application::Loop();
	for (int i = 0; i < iterations; i++)
	{
		ecoSysLabLayer->Simulate();
	}

	if (exportTreeMesh) {
		tree->ExportObj(treeMeshOutputPath, meshGeneratorSettings);
	}
	if (exportTreeIO)
	{
		bool succeed = tree->ExportIoTree(treeIOOutputPath);
	}
	if (exportRadialBoundingVolume || exportRadialBoundingVolumeMesh)
	{
		const auto rbv = ProjectManager::CreateTemporaryAsset<RadialBoundingVolume>();
		tree->ExportRadialBoundingVolume(rbv);
		if (exportRadialBoundingVolume)
		{
			rbv->Export(radialBoundingVolumeOutputPath);
		}
		if (exportRadialBoundingVolumeMesh)
		{
			rbv->ExportAsObj(radialBoundingVolumeMeshOutputPath);
		}
	}
	scene->DeleteEntity(tempEntity);
}

void rbv_to_obj(
	const std::string& rbvPath,
	const std::string& radialBoundingVolumeMeshOutputPath
)
{
	const auto rbv = ProjectManager::CreateTemporaryAsset<RadialBoundingVolume>();
	rbv->Import(rbvPath);
	rbv->ExportAsObj(radialBoundingVolumeMeshOutputPath);
}

void rbv_space_colonization_tree_data(
	const std::string& rbvPath,
	const std::string& treeParametersPath,
	const float deltaTime,
	const int iterations,
	const TreeMeshGeneratorSettings& meshGeneratorSettings,
	bool exportTreeMesh,
	const std::string& treeMeshOutputPath,
	bool exportTreeIO,
	const std::string& treeIOOutputPath,
	bool exportRadialBoundingVolumeMesh,
	const std::string& radialBoundingVolumeMeshOutputPath
)
{
	const auto applicationStatus = Application::GetApplicationStatus();
	if (applicationStatus == ApplicationStatus::NoProject)
	{
		EVOENGINE_ERROR("No project!");
		return;
	}
	if (applicationStatus == ApplicationStatus::OnDestroy)
	{
		EVOENGINE_ERROR("Application is destroyed!");
		return;
	}
	if (applicationStatus == ApplicationStatus::Uninitialized)
	{
		EVOENGINE_ERROR("Application not uninitialized!");
		return;
	}
	const auto scene = Application::GetActiveScene();
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	if (!ecoSysLabLayer)
	{
		EVOENGINE_ERROR("Application doesn't contain EcoSysLab layer!");
		return;
	}
	std::shared_ptr<Soil> soil;
	std::shared_ptr<Climate> climate;

	const std::vector<Entity>* soilEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Soil>();
	if (soilEntities && !soilEntities->empty()) {
		soil = scene->GetOrSetPrivateComponent<Soil>(soilEntities->at(0)).lock();
	}
	if (!soil)
	{
		EVOENGINE_ERROR("No soil in scene!");
		return;
	}
	const std::vector<Entity>* climateEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Climate>();
	if (climateEntities && !climateEntities->empty()) {
		climate = scene->GetOrSetPrivateComponent<Climate>(climateEntities->at(0)).lock();
	}
	if (!climate)
	{
		EVOENGINE_ERROR("No climate in scene!");
		return;
	}

	const auto tempEntity = scene->CreateEntity("Temp");
	const auto tree = scene->GetOrSetPrivateComponent<Tree>(tempEntity).lock();
	tree->soil = soil;
	tree->climate = climate;
	std::shared_ptr<TreeDescriptor> treeDescriptor;
	if (ProjectManager::IsInProjectFolder(treeParametersPath))
	{
		treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(treeParametersPath)));
	}
	else {
		treeDescriptor = ProjectManager::CreateTemporaryAsset<TreeDescriptor>();
	}
	tree->tree_descriptor = treeDescriptor;
	auto& occupancyGrid = tree->tree_model.tree_occupancy_grid;
	const auto rbv = ProjectManager::CreateTemporaryAsset<RadialBoundingVolume>();
	rbv->Import(rbvPath);

	occupancyGrid.Initialize(rbv,
		glm::vec3(-rbv->m_maxRadius, 0, -rbv->m_maxRadius),
		glm::vec3(rbv->m_maxRadius, 2.0f * rbv->m_maxRadius, rbv->m_maxRadius),
		treeDescriptor->shoot_descriptor.Get<ShootDescriptor>()->internode_length,
		tree->tree_model.tree_growth_settings.space_colonization_removal_distance_factor,
		tree->tree_model.tree_growth_settings.space_colonization_theta,
		tree->tree_model.tree_growth_settings.space_colonization_detection_distance_factor);

	tree->tree_model.tree_growth_settings.use_space_colonization = true;
	tree->tree_model.tree_growth_settings.space_colonization_auto_resize = false;
	Application::Loop();
	ecoSysLabLayer->m_simulationSettings.delta_time = deltaTime;
	for (int i = 0; i < iterations; i++)
	{
		ecoSysLabLayer->Simulate();
	}

	if (exportTreeMesh) {
		tree->ExportObj(treeMeshOutputPath, meshGeneratorSettings);
	}
	if (exportTreeIO)
	{
		bool succeed = tree->ExportIoTree(treeIOOutputPath);
	}
	if (exportRadialBoundingVolumeMesh)
	{
		rbv->ExportAsObj(radialBoundingVolumeMeshOutputPath);
	}
	scene->DeleteEntity(tempEntity);
}

PYBIND11_MODULE(pyecosyslab, m) {
	py::class_<Entity>(m, "Entity")
		.def("GetIndex", &Entity::GetIndex)
		.def("GetVersion", &Entity::GetVersion);

	py::class_<ConnectivityGraphSettings>(m, "ConnectivityGraphSettings")
		.def(py::init<>())
		.def_readwrite("m_pointExistenceCheckRadius", &ConnectivityGraphSettings::point_existence_check_radius)
		.def_readwrite("m_pointPointConnectionDetectionRadius", &ConnectivityGraphSettings::point_point_connection_detection_radius)
		.def_readwrite("m_pointBranchConnectionDetectionRadius", &ConnectivityGraphSettings::point_branch_connection_detection_radius)
		.def_readwrite("m_branchBranchConnectionMaxLengthRange", &ConnectivityGraphSettings::branch_branch_connection_max_length_range)
		.def_readwrite("m_directionConnectionAngleLimit", &ConnectivityGraphSettings::direction_connection_angle_limit)
		.def_readwrite("m_indirectConnectionAngleLimit", &ConnectivityGraphSettings::indirect_connection_angle_limit)
		;

	py::class_<TreePointCloudPointSettings>(m, "TreePointCloudPointSettings")
		.def(py::init<>())
		.def_readwrite("m_variance", &TreePointCloudPointSettings::m_variance)
		.def_readwrite("m_ballRandRadius", &TreePointCloudPointSettings::m_ballRandRadius)
		.def_readwrite("m_typeIndex", &TreePointCloudPointSettings::m_typeIndex)
		.def_readwrite("m_instanceIndex", &TreePointCloudPointSettings::m_instanceIndex)
		.def_readwrite("tree_part_index", &TreePointCloudPointSettings::m_treePartIndex)
		.def_readwrite("line_index", &TreePointCloudPointSettings::m_lineIndex)
		.def_readwrite("m_branchIndex", &TreePointCloudPointSettings::m_branchIndex)
		.def_readwrite("m_internodeIndex", &TreePointCloudPointSettings::m_internodeIndex)
		.def_readwrite("m_boundingBoxLimit", &TreePointCloudPointSettings::m_boundingBoxLimit);


	py::class_<TreePointCloudCircularCaptureSettings>(m, "PointCloudCircularCaptureSettings")
		.def(py::init<>())
		.def_readwrite("m_pitchAngleStart", &TreePointCloudCircularCaptureSettings::m_pitchAngleStart)
		.def_readwrite("m_pitchAngleStep", &TreePointCloudCircularCaptureSettings::m_pitchAngleStep)
		.def_readwrite("m_pitchAngleEnd", &TreePointCloudCircularCaptureSettings::m_pitchAngleEnd)
		.def_readwrite("m_turnAngleStart", &TreePointCloudCircularCaptureSettings::m_turnAngleStart)
		.def_readwrite("m_turnAngleStep", &TreePointCloudCircularCaptureSettings::m_turnAngleStep)
		.def_readwrite("m_turnAngleEnd", &TreePointCloudCircularCaptureSettings::m_turnAngleEnd)
		.def_readwrite("m_gridDistance", &TreePointCloudCircularCaptureSettings::m_distance)
		.def_readwrite("m_height", &TreePointCloudCircularCaptureSettings::m_height)
		.def_readwrite("m_fov", &TreePointCloudCircularCaptureSettings::m_fov)
		.def_readwrite("resolution_", &TreePointCloudCircularCaptureSettings::m_resolution)
		.def_readwrite("m_cameraDepthMax", &TreePointCloudCircularCaptureSettings::m_cameraDepthMax);

	py::class_<ReconstructionSettings>(m, "ReconstructionSettings")
		.def(py::init<>())
		.def_readwrite("m_internodeLength", &ReconstructionSettings::internode_length)
		.def_readwrite("m_minHeight", &ReconstructionSettings::min_height)
		.def_readwrite("m_minimumTreeDistance", &ReconstructionSettings::minimum_tree_distance)
		.def_readwrite("m_branchShortening", &ReconstructionSettings::branch_shortening)
		.def_readwrite("m_endNodeThickness", &ReconstructionSettings::end_node_thickness)
		.def_readwrite("m_minimumNodeCount", &ReconstructionSettings::minimum_node_count);

	py::class_<PresentationOverrideSettings>(m, "PresentationOverrideSettings")
		.def(py::init<>())
		.def_readwrite("m_maxThickness", &PresentationOverrideSettings::max_thickness);

	py::class_<ShootDescriptor>(m, "ShootDescriptor")
		.def(py::init<>());
		

	py::class_<FoliageDescriptor>(m, "FoliageDescriptor")
		.def(py::init<>());

	py::class_<TreeMeshGeneratorSettings>(m, "TreeMeshGeneratorSettings")
		.def(py::init<>())
		.def_readwrite("enable_foliage", &TreeMeshGeneratorSettings::enable_foliage)
		.def_readwrite("enable_fruit", &TreeMeshGeneratorSettings::enable_fruit)
		.def_readwrite("enable_branch", &TreeMeshGeneratorSettings::enable_branch)
		.def_readwrite("presentation_override_settings", &TreeMeshGeneratorSettings::presentation_override_settings)
		.def_readwrite("x_subdivision", &TreeMeshGeneratorSettings::x_subdivision)
		.def_readwrite("trunk_y_subdivision", &TreeMeshGeneratorSettings::trunk_y_subdivision)
		.def_readwrite("trunk_thickness", &TreeMeshGeneratorSettings::trunk_thickness)
		.def_readwrite("branch_y_subdivision", &TreeMeshGeneratorSettings::branch_y_subdivision)


		.def_readwrite("override_radius", &TreeMeshGeneratorSettings::override_radius)
		.def_readwrite("m_thickness", &TreeMeshGeneratorSettings::radius)
		.def_readwrite("tree_part_base_distance", &TreeMeshGeneratorSettings::tree_part_base_distance)
		.def_readwrite("tree_part_end_distance", &TreeMeshGeneratorSettings::tree_part_end_distance)
		.def_readwrite("base_control_point_ratio", &TreeMeshGeneratorSettings::base_control_point_ratio)
		.def_readwrite("branch_control_point_ratio", &TreeMeshGeneratorSettings::branch_control_point_ratio)
		.def_readwrite("smoothness", &TreeMeshGeneratorSettings::smoothness)
		.def_readwrite("auto_level", &TreeMeshGeneratorSettings::auto_level)
		.def_readwrite("voxel_subdivision_level", &TreeMeshGeneratorSettings::voxel_subdivision_level)
		.def_readwrite("voxel_smooth_iteration", &TreeMeshGeneratorSettings::voxel_smooth_iteration)
		.def_readwrite("remove_duplicate", &TreeMeshGeneratorSettings::remove_duplicate)
		.def_readwrite("branch_mesh_type", &TreeMeshGeneratorSettings::branch_mesh_type);

	py::class_<Scene>(m, "Scene")
		.def("CreateEntity", static_cast<Entity(Scene::*)(const std::string&)>(&Scene::CreateEntity))
		.def("DeleteEntity", static_cast<void(Scene::*)(const Entity&)>(&Scene::DeleteEntity));

	py::class_<ApplicationInfo>(m, "ApplicationInfo")
		.def(py::init<>())
		.def_readwrite("project_path", &ApplicationInfo::project_path)
		.def_readwrite("m_applicationName", &ApplicationInfo::application_name)
		.def_readwrite("m_enableDocking", &ApplicationInfo::enable_docking)
		.def_readwrite("m_enableViewport", &ApplicationInfo::enable_viewport)
		.def_readwrite("m_fullScreen", &ApplicationInfo::full_screen);

	py::class_<Application>(m, "Application")
		.def_static("Initialize", &Application::Initialize)
		.def_static("Start", &Application::Start)
		.def_static("Run", &Application::Run)
		.def_static("Loop", &Application::Loop)
		.def_static("Terminate", &Application::Terminate)
		.def_static("GetActiveScene", &Application::GetActiveScene);

	py::class_<ProjectManager>(m, "ProjectManager")
		.def_static("GetOrCreateProject", &ProjectManager::GetOrCreateProject);

	m.doc() = "EcoSysLab"; // optional module docstring
	m.def("register_classes", &register_classes, "register_classes");
	m.def("register_layers", &register_layers, "register_layers");
	m.def("start_project_windowless", &start_project_windowless, "StartProjectWindowless");
	m.def("start_project_with_editor", &start_project, "start_project");

	m.def("tree_structor", &tree_structor, "TreeStructor");
	m.def("scene_capture", &scene_capture, "CaptureScene");
	m.def("yaml_visualization", &yaml_visualization, "yaml_visualization");
	m.def("voxel_space_colonization_tree_data", &voxel_space_colonization_tree_data, "voxel_space_colonization_tree_data");
	m.def("rbv_space_colonization_tree_data", &rbv_space_colonization_tree_data, "rbv_space_colonization_tree_data");
	m.def("rbv_to_obj", &rbv_to_obj, "rbv_to_obj");
	

	py::class_<DatasetGenerator>(m, "DatasetGenerator")
		.def(py::init<>())
		.def_static("GeneratePointCloudForTree", &DatasetGenerator::GeneratePointCloudForTree);

}