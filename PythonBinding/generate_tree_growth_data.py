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
tree_framework.RunWithScene(project_path, "./PlayGround.evescene")

#==================================#
#         Configurations           #
#==================================#
#Following configurations are defined in PythonBinding/src/PyEcoSysLab.cpp. You may check all available settings there.
#Create settings for data generation
data_generation_parameters = tree_framework.TreeDataGenerationParameters()

data_generation_parameters.tree_mesh_generator_settings.enable_foliage = True
data_generation_parameters.tree_mesh_generator_settings.enable_root_branch = True
data_generation_parameters.tree_mesh_generator_settings.vertex_color_mode = 0

data_generation_parameters.simulation_settings.delta_time = 30

data_generation_parameters.output_folder = output_root
data_generation_parameters.export_mesh = False
data_generation_parameters.export_rendering = True
#data_generation_parameters.export_ray_traced_rendering = True
#How many growth cycles...
data_generation_parameters.max_iteration = 250
camera_capture_settings = tree_framework.CameraCaptureSettings()
camera_capture_settings.camera_settings.fov = 90
camera_capture_settings.camera_settings.use_clear_color = True
camera_capture_settings.camera_settings.clear_color.x = 0
camera_capture_settings.camera_settings.clear_color.y = 0
camera_capture_settings.camera_settings.clear_color.z = 0
camera_capture_settings.camera_settings.clear_color.w = 1
camera_capture_settings.camera_settings.sample_size = 128
camera_capture_settings.camera_settings.background_intensity = 10
camera_capture_settings.anchor_position.x = 0
camera_capture_settings.anchor_position.y = 0
camera_capture_settings.anchor_position.z = 0.2
camera_capture_settings.anchor_position_delta.z = 0.10
camera_capture_settings.pivot_euler_rotation_delta.x = 0
camera_capture_settings.pivot_euler_rotation_delta.y = 3
camera_capture_settings.pivot_euler_rotation_delta.z = 0

camera_capture_settings.render_resolution.x = 4096
camera_capture_settings.render_resolution.y = 4096

camera_capture_settings.output_resolution.x = 2048
camera_capture_settings.output_resolution.y = 2048

#This can be a relative path to the asset folder of the project, or an relative/absolute path outside the project
data_generation_parameters.tree_descriptor_path = "./TreeDescriptors/Basic/RootSystemHeart.tree"
#Note that you can overwrite foliage descriptor and bark descriptor and etc.
#data_generation_parameters.overriding_shoot_descriptor_path = ""
#data_generation_parameters.overriding_root_descriptor_path = ""
#data_generation_parameters.overriding_fine_root_descriptor_path = ""
#data_generation_parameters.overriding_pruning_descriptor_path = ""
#data_generation_parameters.overriding_foliage_descriptor_path = ""
#data_generation_parameters.overriding_reproduction_module_descriptor_path = ""
#data_generation_parameters.overriding_bark_descriptor_path = ""

#Ambient light & directional light intensity
tree_framework.scene_light_settings(0.3, 7)

#The seed for random tree generator. Same seed will result in same tree geometry.
data_generation_parameters.seed = 0
#The prefix of the output file name.
data_generation_parameters.output_file_name = "Tree_Sample"
tree_framework.generate_tree_growth_data(
	camera_capture_settings,
	data_generation_parameters
)
print("Finished!")
#Terminate engine
tree_framework.Terminate()

#Change back to original working directory
os.chdir(current_directory)