import os
import traceback, importlib

#If you moved this python script, you should change following lines to make sure they points to the correct directory
file_path = os.path.abspath(__file__)
file_folder = os.path.dirname(file_path)
evoengine_directory = os.path.dirname(file_folder) + "/"
root_dir = os.path.dirname(evoengine_directory)
#You may modify output folder path here.
output_root = os.path.dirname(root_dir) + "/SorghumData"


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
if use_gpu:
	sorghum_framework.PushRayTracerLayer()

sorghum_framework.RegisterClasses()
sorghum_framework.PushSorghumLayer()
sorghum_framework.Run(project_path)


#==================================#
#         Configurations           #
#==================================#
#Following configurations are defined in PythonBinding/src/PyDigitalAgriculture.cpp. You may check all available settings there.

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

#Create settings for point cloud capture single sorghum
single_sorghum_gantry_capture_settings = sorghum_framework.SorghumGantryCaptureSettings()
#Expected distance (m) between points, smaller distance means more points.
single_sorghum_gantry_capture_settings.step = 0.005
#Expected distance (m) between capture point and ground level
single_sorghum_gantry_capture_settings.sample_height = 2.5
#Whether output spline info (yaml)
single_sorghum_gantry_capture_settings.output_spline_info = False

#==================================#
#  Single Sorghum Data generation  #
#==================================#

#Now we generate and save mesh and point cloud for single sorghum with sorghum descriptor.

#Load an SorghumDescriptor asset from project's asset folder, and create an entity with this asset.
#If the sorghum_path points to an asset within project's asset folder, it must be a relative path. If it's an external asset, it must be an absolute path.
sorghum_descriptor_handle = sorghum_framework.GetAssetHandle("./SorghumGenerator/Sample0.sorghum")
sorghum_entity = sorghum_framework.CreateEntityFromSorghumDescriptor(sorghum_descriptor_handle)
#The prefix of the output file name.
data_generation_parameters.output_file_name = "SD_Sample"
#Generate data for current sorghum entity.
sorghum_framework.GenerateDataForSorghum(
	use_gpu,
	sorghum_entity,
	single_sorghum_gantry_capture_settings,
	data_generation_parameters
)
#Make sure you delete this entity afterwards.
sorghum_framework.DeleteEntity(sorghum_entity)



#Note: You don't need to restart the framework to generate another sorghum data.

#Now we generate and save mesh and point cloud for single sorghums with sorghum state.
sorghum_state_handle = sorghum_framework.GetAssetHandle("./SorghumGenerator/Sample0.ss")
sorghum_entity = sorghum_framework.CreateEntityFromSorghumState(sorghum_state_handle)
#The prefix of the output file name.
data_generation_parameters.output_file_name = "SS_Sample"
#Generate data for current sorghum entity.
sorghum_framework.GenerateDataForSorghum(
	use_gpu,
	sorghum_entity,
	single_sorghum_gantry_capture_settings,
	data_generation_parameters
)
#Make sure you delete this entity afterwards.
sorghum_framework.DeleteEntity(sorghum_entity)


#Now we generate and save mesh and point cloud for 3 randomly generated sorghums with sorghum generator.
for x in range(2):
	#The seed for random sorghum generator. Same seed will result in same sorghum geometry.
	seed = x
	sorghum_generator_handle = sorghum_framework.GetAssetHandle("./SorghumGenerator/Season11.sg")
	sorghum_entity = sorghum_framework.CreateEntityFromSorghumGenerator(sorghum_generator_handle, seed)
	#The prefix of the output file name.
	data_generation_parameters.output_file_name = "SG_Sample" + str(x)
	#Generate data for current sorghum entity.
	sorghum_framework.GenerateDataForSorghum(
		use_gpu,
		sorghum_entity,
		single_sorghum_gantry_capture_settings,
		data_generation_parameters
	)
	#Make sure you delete this entity afterwards.
	sorghum_framework.DeleteEntity(sorghum_entity)


#==================================#
#  Sorghum Field Data generation   #
#==================================#

#Create settings for point cloud capture sorghum field
grid_dimension = 8
distance_between_sorghum = 0.75

sorghum_field_gantry_capture_settings = sorghum_framework.SorghumGantryCaptureSettings()
#Expected distance (m) between points, smaller distance means more points.
sorghum_field_gantry_capture_settings.step = 0.005
#Expected distance (m) between capture point and ground level
sorghum_field_gantry_capture_settings.sample_height = 2.5
#Whether output spline info (yaml)
sorghum_field_gantry_capture_settings.output_spline_info = False
#The dimension of the grid
sorghum_field_gantry_capture_settings.grid_size.x = sorghum_field_gantry_capture_settings.grid_size.y = grid_dimension
#Distance between sorghums
sorghum_field_gantry_capture_settings.grid_distance.x = sorghum_field_gantry_capture_settings.grid_distance.y = distance_between_sorghum

#Now we generate and save mesh and point cloud for 3 randomly generated sorghums grids with sorghum generator.
#First, we need to setup a sorghum grid.
sorghum_grid = sorghum_framework.SorghumGrid()
#The dimension of the grid
sorghum_grid.grid_size.x = sorghum_grid.grid_size.y = grid_dimension
#Distance between sorghums
sorghum_grid.grid_distance.x = sorghum_grid.grid_distance.y = distance_between_sorghum
#Average of random shift distance of sorghum position
sorghum_grid.position_offset_mean = 0.2
#Variance of random shift distance of sorghum position
sorghum_grid.position_offset_variance = 0.1

#Here we create a runtime asset, it's a temporary asset not exist on disk.
#============================================================#
#          YOU have the ownership of runtime asset!          #
#     YOU are responsible for cleaning it after using it!    #
#============================================================#
sorghum_field_handle = sorghum_framework.CreateRuntimeAsset("SorghumField")
#Prepare sorghum generator asset.
sorghum_generator_handle = sorghum_framework.GetAssetHandle("./SorghumGenerator/Season12.sg")
#Apply grid settings and sorghum generator to the sorghum field asset.
sorghum_framework.ApplySorghumGrid(sorghum_field_handle, sorghum_generator_handle, sorghum_grid)
for x in range(2):
	#The seed for random sorghum generator. Same seed will result in same sorghum geometry.
	seed = x
	sorghum_field_entity = sorghum_framework.CreateEntityFromSorghumField(sorghum_field_handle, seed)
	#The prefix of the output file name.
	data_generation_parameters.output_file_name = "Grid_Sample" + str(x)
	sorghum_framework.GenerateDataForAllSorghums(
		use_gpu,
		sorghum_field_gantry_capture_settings,
		data_generation_parameters
	)
	#Make sure you delete this entity afterwards.
	sorghum_framework.DeleteEntity(sorghum_field_entity)
#Delete runtime asset as we don't need it anymore.
sorghum_framework.DeleteRuntimeAsset(sorghum_field_handle)

#==================================#
#            Clean up              #
#==================================#

#Close the framework after we finished data generation
sorghum_framework.Terminate()

#Change back to original working directory
os.chdir(current_directory)