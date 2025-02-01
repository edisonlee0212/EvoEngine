import platform
import os

def is_windows():
    return platform.system() == "Windows"

#You should change following lines to make sure they points to the correct directory
evoengine_directory = "~/EvoEngine/"
if is_windows():
	evoengine_directory = "C:/Users/lllll/Documents/GitHub/EvoEngine/"
output_root = os.path.expanduser("~/SorghumPointCloudData")

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
import PyDigitalAgriculture as sorghum_framework

#Point the framework to load the default project folder that contains 2 sample sorghum descriptors.
project_path = os.path.expanduser(evoengine_directory + "Resources/DigitalAgricultureProject/test.eveproj")

#Create new folder for output path if necessary
if not os.path.isdir(output_root):
	os.mkdir(output_root)

#Enable GPU
use_gpu = True

#Start the framework without editor and window.
sorghum_framework.engine_run_windowless(use_gpu, project_path)

#==================================#
#         Configurations           #
#==================================#

#Create settings for data generation
data_generation_parameters = sorghum_framework.SorghumDataGenerationParameters()
#Whether generate ground surface mesh
data_generation_parameters.generate_ground_mesh = False
#Whether we want to simulate no occlusion (we will sample point cloud for each sorghum leaf individually and combine them together)
data_generation_parameters.avoid_occlusion = False

#Should panicle geometry be generated
data_generation_parameters.sorghum_mesh_generator_settings.enable_panicle = True
#Should stem geometry be generated
data_generation_parameters.sorghum_mesh_generator_settings.enable_stem = True
#Should leaf geometry be generated
data_generation_parameters.sorghum_mesh_generator_settings.enable_leaves = True
#Should geometry of stem part of each leaf be generated
data_generation_parameters.sorghum_mesh_generator_settings.enable_leaf_sheath = False
#If -1, generate all leaves, otherwise generate specific leaf only
data_generation_parameters.sorghum_mesh_generator_settings.single_leaf_index = -1
#Should leaf mesh contain 2 faces
data_generation_parameters.sorghum_mesh_generator_settings.bottom_face = False
#Should all leaves be generated in separated meshes or combined as one single mesh, if set to false, you will not get leaf index in point cloud.
data_generation_parameters.sorghum_mesh_generator_settings.leaf_separated = True
#If bottom_face is ON, this sets the distance between top and bottom face of each leaf
data_generation_parameters.sorghum_mesh_generator_settings.leaf_thickness = 0.001

#The variance of uncertainty on point positions for simulating inaccuracy
data_generation_parameters.sorghum_point_cloud_point_settings.variance = 0.015
#The uniform random range of uncertainty on point positions for simulating inaccuracy
data_generation_parameters.sorghum_point_cloud_point_settings.ball_rand_radius = 0.01

#Should mesh type be included for each point (0 = leaf, 1 = stem, 2 = panicle, 3 = ground surface, -1 = everything else)
data_generation_parameters.sorghum_point_cloud_point_settings.type_index = True
#If multiple sorghums are in the scene, should their instance indices be included for each sorghum (All will be 0 if only one sorghum present)
data_generation_parameters.sorghum_point_cloud_point_settings.instance_index = False
#Should indices of leaves be included for each point (Start from bottom leaf and first index is 0)
data_generation_parameters.sorghum_point_cloud_point_settings.leaf_index = True
#The bounding box size for scanning. Set to 2.0 as default to include full sorghum geometry (Scanner simulates Gantry, so it needs a bigger bounding box to include full geometry)
data_generation_parameters.sorghum_point_cloud_point_settings.bounding_box_limit = 2.0

#The path of the output folder.
data_generation_parameters.output_folder = output_root

#Whether output point clouds
data_generation_parameters.export_point_cloud = True
#Whether output point meshes
data_generation_parameters.export_mesh = True

#Create settings for point cloud capture
gantry_capture_settings = sorghum_framework.SorghumGantryCaptureSettings()
#Expected distance (m) between points, smaller distance means more points.
gantry_capture_settings.step = 0.005
#Expected distance (m) between capture point and ground level
gantry_capture_settings.sample_height = 2.5
#Whether output spline info (yaml)
gantry_capture_settings.output_spline_info = False

#==================================#
#         Data generation          #
#==================================#

#Now we generate and save mesh and point cloud for single sorghum with sorghum descriptor.
#Path to sorghum parameters, can be sorghum generator, sorghum state, or sorghum descriptor; can either be a relative path, or a absolute path; can be inside or outside project folder.
data_generation_parameters.sorghum_path = "./SorghumGenerator/Sample0.sorghum"
#The prefix of the output file name.
data_generation_parameters.output_file_name = "SD_Sample"
sorghum_framework.generate_sorghum_data(
	use_gpu,
	gantry_capture_settings,
	data_generation_parameters
)

#Note: You don't need to restart the framework to generate another sorghum data.

#Now we generate and save mesh and point cloud for single sorghums with sorghum state.
data_generation_parameters.sorghum_path = "./SorghumGenerator/Sample0.ss"
#The prefix of the output file name.
data_generation_parameters.output_file_name = "SS_Sample"
sorghum_framework.generate_sorghum_data(
	use_gpu,
	gantry_capture_settings,
	data_generation_parameters
)

#Now we generate and save mesh and point cloud for 3 randomly generated sorghums with sorghum generator.
for x in range(3):
	#The seed for random sorghum generator. Same seed will result in same sorghum geometry.
	data_generation_parameters.seed = x
	data_generation_parameters.sorghum_path = "./SorghumGenerator/Season12.sg"
	#The prefix of the output file name.
	data_generation_parameters.output_file_name = "SG_Sample" + str(x)
	sorghum_framework.generate_sorghum_data(
		use_gpu,
		gantry_capture_settings,
		data_generation_parameters
	)

#Now we generate and save mesh and point cloud for 3 randomly generated sorghums grids with sorghum generator.
#First, we need to setup a sorghum grid.
sorghum_grid = sorghum_framework.SorghumGrid()
#The dimension of the grid
sorghum_grid.grid_size_x = sorghum_grid.grid_size_y = 8
#Distance between sorghums
sorghum_grid.grid_distance_x = sorghum_grid.grid_distance_y = .75
#Average of random shift distance of sorghum position
sorghum_grid.position_offset_mean = 0.2
#Variance of random shift distance of sorghum position
sorghum_grid.position_offset_variance = 0.1
for x in range(3):
	#The seed for random sorghum generator. Same seed will result in same sorghum geometry.
	data_generation_parameters.seed = x
	data_generation_parameters.sorghum_path = "./SorghumGenerator/Season12.sg"
	#The prefix of the output file name.
	data_generation_parameters.output_file_name = "Grid_Sample" + str(x)
	sorghum_framework.generate_sorghum_grid_data(
		use_gpu,
		gantry_capture_settings,
		sorghum_grid,
		data_generation_parameters
	)

#==================================#
#            Clean up              #
#==================================#

#Close the framework after we finished data generation
sorghum_framework.engine_terminate()

#Change back to original working directory
os.chdir(current_directory)