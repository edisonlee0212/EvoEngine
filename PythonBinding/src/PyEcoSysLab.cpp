#include "PyEcoSysLab.hpp"
#include "PyEvoEngine.hpp"

#ifdef ECOSYSLAB_PACKAGE
namespace py = pybind11;
using namespace py_eco_sys_lab_package;

#endif
void PyEcoSysLab::PushEcoSysLabLayer() {
  ApplicationContext::Get().PushLayer<EcoSysLabLayer>("EcoSysLab Layer");
}
void PyEcoSysLab::RegisterClasses() {
#ifdef ECOSYSLAB_PACKAGE
  auto& application = PyEvoEngine::GetRuntime().GetApplication();
  application.RegisterPrivateComponent<ObjectRotator>("ObjectRotator");
  application.RegisterPrivateComponent<Physics2DDemo>("Physics2DDemo");
  application.RegisterPrivateComponent<ParticlePhysics2DDemo>("ParticlePhysics2DDemo");
  application.RegisterPrivateComponent<TreePointCloudScanner>("TreePointCloudScanner");
#endif
}
void PyEcoSysLab::Initialize(pybind11::module& m) {
  PyEvoEngine::Initialize(m);
  m.def("RegisterClasses", &RegisterClasses);
  m.def("PushEcoSysLabLayer", &PushEcoSysLabLayer);
  py::class_<ConnectivityGraphSettings>(m, "ConnectivityGraphSettings")
      .def(py::init<>())
      .def_readwrite("point_existence_check", &ConnectivityGraphSettings::point_existence_check)
      .def_readwrite("point_existence_check_radius", &ConnectivityGraphSettings::point_existence_check_radius)
      .def_readwrite("zigzag_check", &ConnectivityGraphSettings::zigzag_check)
      .def_readwrite("zigzag_branch_shortening", &ConnectivityGraphSettings::zigzag_branch_shortening)
      .def_readwrite("parallel_shift_check_height_limit", &ConnectivityGraphSettings::parallel_shift_check_height_limit)
      .def_readwrite("parallel_shift_check", &ConnectivityGraphSettings::parallel_shift_check)
      .def_readwrite("parallel_shift_limit_range", &ConnectivityGraphSettings::parallel_shift_limit_range)
      .def_readwrite("point_point_connection_detection_radius",
                     &ConnectivityGraphSettings::point_point_connection_detection_radius)
      .def_readwrite("point_branch_connection_detection_radius",
                     &ConnectivityGraphSettings::point_branch_connection_detection_radius)
      .def_readwrite("branch_branch_connection_max_length_range",
                     &ConnectivityGraphSettings::branch_branch_connection_max_length_range)
      .def_readwrite("direction_connection_angle_limit", &ConnectivityGraphSettings::direction_connection_angle_limit)
      .def_readwrite("indirect_connection_angle_limit", &ConnectivityGraphSettings::indirect_connection_angle_limit)
      .def_readwrite("connection_range_limit", &ConnectivityGraphSettings::connection_range_limit)
      .def_readwrite("max_scatter_point_connection_height",
                     &ConnectivityGraphSettings::max_scatter_point_connection_height);

#ifdef DATASET_GENERATION_PACKAGE
  py::class_<TreePointCloudPointSettings>(m, "TreePointCloudPointSettings")
      .def(py::init<>())
      .def_readwrite("variance", &TreePointCloudPointSettings::variance)
      .def_readwrite("ball_rand_radius", &TreePointCloudPointSettings::ball_rand_radius)
      .def_readwrite("type_index", &TreePointCloudPointSettings::type_index)
      .def_readwrite("instance_index", &TreePointCloudPointSettings::instance_index)
      .def_readwrite("tree_part_index", &TreePointCloudPointSettings::tree_part_index)
      .def_readwrite("tree_part_type_index", &TreePointCloudPointSettings::tree_part_type_index)
      .def_readwrite("line_index", &TreePointCloudPointSettings::line_index)
      .def_readwrite("branch_index", &TreePointCloudPointSettings::branch_index)
      .def_readwrite("internode_index", &TreePointCloudPointSettings::internode_index)
      .def_readwrite("bounding_box_limit", &TreePointCloudPointSettings::bounding_box_limit);

  py::class_<TreePointCloudCircularCaptureSettings>(m, "TreePointCloudCircularCaptureSettings")
      .def(py::init<>())
      .def_readwrite("pitch_angle_start", &TreePointCloudCircularCaptureSettings::pitch_angle_start)
      .def_readwrite("pitch_angle_step", &TreePointCloudCircularCaptureSettings::pitch_angle_step)
      .def_readwrite("pitch_angle_end", &TreePointCloudCircularCaptureSettings::pitch_angle_end)
      .def_readwrite("turn_angle_start", &TreePointCloudCircularCaptureSettings::turn_angle_start)
      .def_readwrite("turn_angle_step", &TreePointCloudCircularCaptureSettings::turn_angle_step)
      .def_readwrite("turn_angle_end", &TreePointCloudCircularCaptureSettings::turn_angle_end)
      .def_readwrite("distance_from_trees", &TreePointCloudCircularCaptureSettings::distance_from_trees)
      .def_readwrite("capture_height", &TreePointCloudCircularCaptureSettings::capture_height)
      .def_readwrite("camera_fov", &TreePointCloudCircularCaptureSettings::camera_fov)
      .def_readwrite("scan_resolution", &TreePointCloudCircularCaptureSettings::scan_resolution)
      .def_readwrite("max_capture_depth", &TreePointCloudCircularCaptureSettings::max_capture_depth);
#endif

  py::class_<ReconstructionSettings>(m, "ReconstructionSettings")
      .def(py::init<>())
      .def_readwrite("internode_length", &ReconstructionSettings::internode_length)
      .def_readwrite("min_height", &ReconstructionSettings::min_height)
      .def_readwrite("minimum_tree_distance", &ReconstructionSettings::minimum_tree_distance)
      .def_readwrite("branch_shortening", &ReconstructionSettings::branch_shortening)
      .def_readwrite("max_parent_candidate_size", &ReconstructionSettings::max_parent_candidate_size)
      .def_readwrite("max_child_size", &ReconstructionSettings::max_child_size)
      .def_readwrite("end_node_thickness", &ReconstructionSettings::end_node_thickness)
      .def_readwrite("thickness_sum_factor", &ReconstructionSettings::thickness_sum_factor)
      .def_readwrite("apply_root_thickness", &ReconstructionSettings::apply_root_thickness)
      .def_readwrite("thickness_accumulation_factor", &ReconstructionSettings::thickness_accumulation_factor)
      .def_readwrite("override_thickness_root_distance", &ReconstructionSettings::override_thickness_root_distance)
      .def_readwrite("override_thickness_root_distance", &ReconstructionSettings::override_thickness_root_distance)
      .def_readwrite("space_colonization_timeout", &ReconstructionSettings::space_colonization_timeout)
      .def_readwrite("space_colonization_factor", &ReconstructionSettings::space_colonization_factor)
      .def_readwrite("space_colonization_removal_distance_factor",
                     &ReconstructionSettings::space_colonization_removal_distance_factor)
      .def_readwrite("space_colonization_detection_distance_factor",
                     &ReconstructionSettings::space_colonization_detection_distance_factor)
      .def_readwrite("space_colonization_theta", &ReconstructionSettings::space_colonization_theta)
      .def_readwrite("minimum_node_count", &ReconstructionSettings::minimum_node_count)
      .def_readwrite("limit_parent_thickness", &ReconstructionSettings::limit_parent_thickness)
      .def_readwrite("minimum_root_thickness", &ReconstructionSettings::minimum_root_thickness)
      .def_readwrite("node_back_track_limit", &ReconstructionSettings::node_back_track_limit)
      .def_readwrite("branch_back_track_limit", &ReconstructionSettings::branch_back_track_limit)
      .def_readwrite("use_root_distance", &ReconstructionSettings::use_root_distance)
      .def_readwrite("optimization_timeout", &ReconstructionSettings::optimization_timeout)
      .def_readwrite("direction_smoothing", &ReconstructionSettings::direction_smoothing)
      .def_readwrite("position_smoothing", &ReconstructionSettings::position_smoothing)
      .def_readwrite("smooth_iteration", &ReconstructionSettings::smooth_iteration)
      .def_readwrite("use_foliage", &ReconstructionSettings::use_foliage);

  py::class_<PresentationOverrideSettings>(m, "PresentationOverrideSettings")
      .def(py::init<>())
      .def_readwrite("max_thickness", &PresentationOverrideSettings::max_thickness);

  py::class_<TreeMeshGeneratorSettings>(m, "TreeMeshGeneratorSettings")
      .def(py::init<>())
      .def_readwrite("enable_foliage", &TreeMeshGeneratorSettings::enable_foliage)
      .def_readwrite("enable_fruit", &TreeMeshGeneratorSettings::enable_fruit)
      .def_readwrite("enable_shoot_branch", &TreeMeshGeneratorSettings::enable_shoot_branch)
      .def_readwrite("enable_root_branch", &TreeMeshGeneratorSettings::enable_root_branch)
      .def_readwrite("presentation_override_settings", &TreeMeshGeneratorSettings::presentation_override_settings)
      .def_readwrite("x_subdivision", &TreeMeshGeneratorSettings::x_subdivision)
      .def_readwrite("trunk_y_subdivision", &TreeMeshGeneratorSettings::trunk_y_subdivision)
      .def_readwrite("trunk_thickness", &TreeMeshGeneratorSettings::trunk_thickness)
      .def_readwrite("branch_y_subdivision", &TreeMeshGeneratorSettings::branch_y_subdivision)
      .def_readwrite("vertex_color_mode", &TreeMeshGeneratorSettings::vertex_color_mode)
      .def_readwrite("override_radius", &TreeMeshGeneratorSettings::override_radius)
      .def_readwrite("radius", &TreeMeshGeneratorSettings::radius)
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

  py::class_<SimulationSettings>(m, "SimulationSettings")
      .def(py::init<>())
      .def_readwrite("delta_time", &SimulationSettings::delta_time)
      .def_readwrite("soil_simulation", &SimulationSettings::soil_simulation)
      .def_readwrite("auto_clear_fruit_and_leaves", &SimulationSettings::auto_clear_fruit_and_leaves)
      .def_readwrite("crown_shyness_distance", &SimulationSettings::crown_shyness_distance)
      .def_readwrite("max_node_count", &SimulationSettings::max_node_count)
      .def_readwrite("max_flow_count", &SimulationSettings::max_flow_count)
      .def_readwrite("skylight_intensity", &SimulationSettings::skylight_intensity)
      .def_readwrite("shadow_distance_loss", &SimulationSettings::shadow_distance_loss)
      .def_readwrite("detection_radius", &SimulationSettings::detection_radius)
      .def_readwrite("environment_light_intensity", &SimulationSettings::environment_light_intensity)
      .def_readwrite("blur_iteration", &SimulationSettings::blur_iteration)
      .def_readwrite("auto_generate_skeletal_graph", &SimulationSettings::auto_generate_skeletal_graph);

  py::class_<TreeGrowthSettings>(m, "TreeGrowthSettings")
      .def(py::init<>())
      .def_readwrite("node_developmental_vigor_filling_rate",
                     &TreeGrowthSettings::node_developmental_vigor_filling_rate)
      .def_readwrite("use_space_colonization", &TreeGrowthSettings::use_space_colonization)
      .def_readwrite("space_colonization_auto_resize", &TreeGrowthSettings::space_colonization_auto_resize)
      .def_readwrite("space_colonization_removal_distance_factor",
                     &TreeGrowthSettings::space_colonization_removal_distance_factor)
      .def_readwrite("space_colonization_detection_distance_factor",
                     &TreeGrowthSettings::space_colonization_detection_distance_factor)
      .def_readwrite("space_colonization_theta", &TreeGrowthSettings::space_colonization_theta);

  py::class_<CameraSettings>(m, "CameraSettings")
      .def(py::init<>())
      .def_readwrite("near_distance", &CameraSettings::near_distance)
      .def_readwrite("far_distance", &CameraSettings::far_distance)
      .def_readwrite("fade_ratio", &CameraSettings::fade_ratio)
      .def_readwrite("fade_factor", &CameraSettings::fade_factor)
      .def_readwrite("fov", &CameraSettings::fov)
      .def_readwrite("use_clear_color", &CameraSettings::use_clear_color)
      .def_readwrite("clear_color", &CameraSettings::clear_color)
      .def_readwrite("background_intensity", &CameraSettings::background_intensity)
      .def_readwrite("sample_size", &CameraSettings::sample_size)
      .def_readwrite("bounce", &CameraSettings::bounce)
      .def_readwrite("gamma", &CameraSettings::gamma);

  py::class_<DatasetGenerator::CameraCaptureSettings>(m, "CameraCaptureSettings")
      .def(py::init<>())
      .def_readwrite("pivot_position", &DatasetGenerator::CameraCaptureSettings::pivot_position)
      .def_readwrite("pivot_euler_rotation", &DatasetGenerator::CameraCaptureSettings::pivot_euler_rotation)
      .def_readwrite("pivot_position_delta", &DatasetGenerator::CameraCaptureSettings::pivot_position_delta)
      .def_readwrite("pivot_euler_rotation_delta", &DatasetGenerator::CameraCaptureSettings::pivot_euler_rotation_delta)
      .def_readwrite("anchor_position", &DatasetGenerator::CameraCaptureSettings::anchor_position)
      .def_readwrite("anchor_rotation", &DatasetGenerator::CameraCaptureSettings::anchor_rotation)
      .def_readwrite("anchor_position_delta", &DatasetGenerator::CameraCaptureSettings::anchor_position_delta)
      .def_readwrite("anchor_rotation_delta", &DatasetGenerator::CameraCaptureSettings::anchor_rotation_delta)
      .def_readwrite("camera_settings", &DatasetGenerator::CameraCaptureSettings::camera_settings)
      .def_readwrite("render_resolution", &DatasetGenerator::CameraCaptureSettings::render_resolution)
      .def_readwrite("output_resolution", &DatasetGenerator::CameraCaptureSettings::output_resolution);

  py::class_<DatasetGenerator::TreeDataGenerationParameters>(m, "TreeDataGenerationParameters")
      .def(py::init<>())
      .def_readwrite("tree_descriptor_path", &DatasetGenerator::TreeDataGenerationParameters::tree_descriptor_path)
      .def_readwrite("overriding_shoot_descriptor_path",
                     &DatasetGenerator::TreeDataGenerationParameters::overriding_shoot_descriptor_path)
      .def_readwrite("overriding_root_descriptor_path",
                     &DatasetGenerator::TreeDataGenerationParameters::overriding_root_descriptor_path)
      .def_readwrite("overriding_fine_root_descriptor_path",
                     &DatasetGenerator::TreeDataGenerationParameters::overriding_fine_root_descriptor_path)
      .def_readwrite("overriding_pruning_descriptor_path",
                     &DatasetGenerator::TreeDataGenerationParameters::overriding_pruning_descriptor_path)
      .def_readwrite("overriding_foliage_descriptor_path",
                     &DatasetGenerator::TreeDataGenerationParameters::overriding_foliage_descriptor_path)
      .def_readwrite("overriding_reproduction_module_descriptor_path",
                     &DatasetGenerator::TreeDataGenerationParameters::overriding_reproduction_module_descriptor_path)
      .def_readwrite("overriding_bark_descriptor_path",
                     &DatasetGenerator::TreeDataGenerationParameters::overriding_bark_descriptor_path)

      .def_readwrite("simulation_settings", &DatasetGenerator::TreeDataGenerationParameters::simulation_settings)
      .def_readwrite("tree_growth_settings", &DatasetGenerator::TreeDataGenerationParameters::tree_growth_settings)

      .def_readwrite("max_iteration", &DatasetGenerator::TreeDataGenerationParameters::max_iteration)
      .def_readwrite("use_node_growth_capture",
                     &DatasetGenerator::TreeDataGenerationParameters::use_node_growth_capture)
      .def_readwrite("growth_capture", &DatasetGenerator::TreeDataGenerationParameters::growth_capture)

      .def_readwrite("export_point_cloud", &DatasetGenerator::TreeDataGenerationParameters::export_point_cloud)
      .def_readwrite("export_mesh", &DatasetGenerator::TreeDataGenerationParameters::export_mesh)
      .def_readwrite("export_rendering", &DatasetGenerator::TreeDataGenerationParameters::export_rendering)
      .def_readwrite("export_ray_traced_rendering",
                     &DatasetGenerator::TreeDataGenerationParameters::export_ray_traced_rendering)
      .def_readwrite("export_depth", &DatasetGenerator::TreeDataGenerationParameters::export_depth)
      .def_readwrite("export_statistics", &DatasetGenerator::TreeDataGenerationParameters::export_statistics)
      .def_readwrite("export_flow_graph", &DatasetGenerator::TreeDataGenerationParameters::export_flow_graph)
      .def_readwrite("export_node_graph", &DatasetGenerator::TreeDataGenerationParameters::export_node_graph)

      .def_readwrite("generate_ground_mesh", &DatasetGenerator::TreeDataGenerationParameters::generate_ground_mesh)
      .def_readwrite("tree_point_cloud_point_settings",
                     &DatasetGenerator::TreeDataGenerationParameters::tree_point_cloud_point_settings)
      .def_readwrite("tree_mesh_generator_settings",
                     &DatasetGenerator::TreeDataGenerationParameters::tree_mesh_generator_settings)

      .def_readwrite("camera_capture_settings",
                     &DatasetGenerator::TreeDataGenerationParameters::camera_capture_settings)

      .def_readwrite("seed", &DatasetGenerator::TreeDataGenerationParameters::seed)
      .def_readwrite("max_depth", &DatasetGenerator::TreeDataGenerationParameters::max_depth)
      .def_readwrite("output_folder", &DatasetGenerator::TreeDataGenerationParameters::output_folder)
      .def_readwrite("output_file_name", &DatasetGenerator::TreeDataGenerationParameters::output_file_name);
}
