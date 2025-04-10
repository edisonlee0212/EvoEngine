import os

#If you moved this python script, you should change following lines to make sure they points to the correct directory
file_path = os.path.abspath(__file__)
file_folder = os.path.dirname(file_path)
evoengine_directory = os.path.dirname(file_folder) + "/"
root_dir = os.path.dirname(evoengine_directory)
#You may modify output folder path here.
output_root = os.path.dirname(root_dir) + "/TreeData"

#Capture current working directory to restore later
current_directory = os.getcwd()

#Redirect working directory to the binaries directory of the framework. This has to be done because we also have resource files copied to that folder after compilation of the framework
library_directory = os.path.expanduser(evoengine_directory + "out/build/x64-Release/PythonBinding")
os.chdir(library_directory)

#==================================#
#         Framework Init           #
#==================================#
#Add directory that contains the python library to system path so we can import the library
import sys
sys.path.append(library_directory)

#Import framework and start data generation``
import PyEcoSysLab as tree_framework

#Project folder path
project_folder_path = os.path.expanduser(evoengine_directory + "Resources/EcoSysLabProject/")

#Point the framework to load the default project
project_path = os.path.expanduser(evoengine_directory + "Resources/EcoSysLabProject/test.eveproj")

#Create new folder for output path if necessary
if not os.path.isdir(output_root):
	os.mkdir(output_root)

#Enable GPU
use_gpu = True

#Start the framework without editor and window.
tree_framework.PushRenderLayer()
if use_gpu:
	tree_framework.PushRayTracerLayer()
tree_framework.RegisterClasses()
tree_framework.PushEcoSysLabLayer()
tree_framework.Run(project_path)

#==================================#
#         Configurations           #
#==================================#
#Following configurations are defined in PythonBinding/src/PyEcoSysLab.cpp. You may check all available settings there.
#Create settings for data generation
data_generation_parameters = tree_framework.TreeDataGenerationParameters()

export_junction = False

data_generation_parameters.tree_point_cloud_point_settings.ball_rand_radius = 0.0
data_generation_parameters.tree_point_cloud_point_settings.tree_part_index = export_junction
data_generation_parameters.tree_point_cloud_point_settings.instance_index = True
data_generation_parameters.tree_point_cloud_point_settings.type_index = True
data_generation_parameters.tree_point_cloud_point_settings.tree_part_type_index = export_junction
data_generation_parameters.tree_point_cloud_point_settings.branch_index = False
data_generation_parameters.tree_point_cloud_point_settings.line_index = export_junction
data_generation_parameters.tree_mesh_generator_settings.enable_foliage = True
data_generation_parameters.tree_mesh_generator_settings.vertex_color_mode = 0
data_generation_parameters.simulation_settings.max_flow_count = 1024
#Max amount of nodes
data_generation_parameters.simulation_settings.max_node_count = 65536
data_generation_parameters.output_folder = output_root
data_generation_parameters.export_point_cloud = True
data_generation_parameters.export_mesh = True

data_generation_parameters.export_rendering = True
data_generation_parameters.export_depth = True
data_generation_parameters.export_statistics = True

data_generation_parameters.export_node_graph = True
data_generation_parameters.export_flow_graph = True

#Depth value is linearized and clamp with max value. Smaller value means closer to camera. 1.0 means max depth/inf depth.
data_generation_parameters.max_depth = 8
data_generation_parameters.generate_ground_mesh = False

camera_capture_settings = tree_framework.CameraCaptureSettings()
camera_capture_settings.camera_settings.fov = 60
camera_capture_settings.camera_settings.use_clear_color = True
camera_capture_settings.camera_settings.clear_color.x = 1
camera_capture_settings.camera_settings.clear_color.y = 1
camera_capture_settings.camera_settings.clear_color.z = 1
camera_capture_settings.camera_settings.background_intensity = 10
camera_capture_settings.position.x = 0
camera_capture_settings.position.y = 6
camera_capture_settings.position.z = 0
camera_capture_settings.euler_rotation.x = -90
camera_capture_settings.euler_rotation.y = 0
camera_capture_settings.euler_rotation.z = 0

camera_capture_settings.render_resolution.x = 2048
camera_capture_settings.render_resolution.y = 2048

camera_capture_settings.output_resolution.x = 1024
camera_capture_settings.output_resolution.y = 1024

#This can be a relative path to the asset folder of the project, or an relative/absolute path outside the project
data_generation_parameters.tree_descriptor_path = "./TreeStructor/TreeStructor.tree"
data_generation_parameters.foliage_descriptor_path = "./TreeStructor/TreeStructor.foliage"


point_cloud_capture_settings = tree_framework.TreePointCloudCircularCaptureSettings()
point_cloud_capture_settings.distance_from_trees = 4.0
point_cloud_capture_settings.capture_height = 3.0

#Ambient light & directional light intensity
tree_framework.scene_light_settings(0.2, 4)

for x in range(3):
	#The seed for random tree generator. Same seed will result in same tree geometry.
	data_generation_parameters.seed = x
	#The prefix of the output file name.
	data_generation_parameters.output_file_name = "Tree_Sample" + str(x)
	tree_framework.generate_tree_data(
		point_cloud_capture_settings,
		camera_capture_settings,
		data_generation_parameters
	)

#Terminate engine
tree_framework.Terminate()

#Change back to original working directory
os.chdir(current_directory)