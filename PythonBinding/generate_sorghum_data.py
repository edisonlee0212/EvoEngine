#You should change following lines to make sure they points to the correct directory.
evoengine_directory = "C:\\Users\\lllll\\Documents\\GitHub\\EvoEngine\\"
output_root = "D:\\SorghumPointCloudData"

import os
current_directory = os.getcwd()

#Redirect working directory to the binaries directory of the framework. This has to be done because we also have resource files copied to that folder after compilation of the framework
library_directory = evoengine_directory + "out\\build\\x64-Release\\PythonBinding"
os.chdir(library_directory)

#Add directory that contains the python library to system path so we can import the library
import sys
sys.path.append(library_directory)
#Import framework and start data generation``
import PyDigitalAgriculture as sorghum_framework

#Point the framework to load the default project folder that contains 2 sample sorghum descriptors.
project_path = evoengine_directory + "Resources\\DigitalAgricultureProject\\test.eveproj"

#Create new folder for output path if necessary
if not os.path.isdir(output_root):
	os.mkdir(output_root)


#Enable GPU
use_gpu = False

#Start the framework without editor and window.
sorghum_framework.engine_run_windowless(use_gpu, project_path)

#Create settings for data generation
mesh_settings = sorghum_framework.SorghumMeshGeneratorSettings()
point_cloud_settings = sorghum_framework.SorghumPointCloudPointSettings()

#Should panicle geometry be generated
mesh_settings.enable_panicle = True
#Should stem geometry be generated
mesh_settings.enable_stem = True
#Should leaf geometry be generated
mesh_settings.enable_leaves = True
#Should geometry of stem part of each leaf be generated
mesh_settings.enable_leaf_sheath = False
#If -1, generate all leaves, otherwise generate specific leaf only
mesh_settings.single_leaf_index = -1
#Should leaf mesh contain 2 faces
mesh_settings.bottom_face = False
#Should all leaves be generated in separated meshes or combined as one single mesh, if set to false, you will not get leaf index in point cloud.
mesh_settings.leaf_separated = True
#If bottom_face is ON, this sets the distance between top and bottom face of each leaf
mesh_settings.leaf_thickness = 0.001

#The variance of uncertainty on point positions for simulating inaccuracy
point_cloud_settings.variance = 0.015
#The uniform random range of uncertainty on point positions for simulating inaccuracy
point_cloud_settings.ball_rand_radius = 0.01

#Should mesh type be included for each point (0 = leaf, 1 = stem, 2 = panicle, 3 = ground surface, -1 = everything else)
point_cloud_settings.type_index = True
#If multiple sorghums are in the scene, should their instance indices be included for each sorghum (All will be 0 if only one sorghum present)
point_cloud_settings.instance_index = False
#Should indices of leaves be included for each point (Start from bottom leaf and first index is 0)
point_cloud_settings.leaf_index = True
#The bounding box size for scanning. Set to 2.0 as default to include full sorghum geometry (Scanner simulates Gantry, so it needs a bigger bounding box to include full geometry)
point_cloud_settings.bounding_box_limit = 2.0

#Now we generate and save mesh and point cloud for 2 sorghums with sorghum descriptor.
sorghum_framework.sorghum_descriptor_to_mesh_and_point_cloud(
	use_gpu,
	#Path to sorghum descriptor [[!!!IF THE SORGHUM DESCRIPTOR IS IN PROJECT FOLDER, YOU SHOULD USE RELATIVE PATH, IF IT'S SAVED OUTSIDE THE FOLDER, USE ABSOLUTE PATH!!!]]
	".\\SorghumGenerator\\Sample0.sorghum",
	#Sorghum Point Cloud Point Settings
	point_cloud_settings,
	#Sorghum Mesh Generator Settings
	mesh_settings,
	#Whether generate 2 point clouds (with/without occclusion) for the sorghum
	False,
	#Whether generate ground surface mesh
	False,
	#Where to store the 3d model
	output_root + "\\SD_Sample0.obj",
	#Where to save the point cloud
	output_root + "\\SD_Sample0.ply"
)
#Note that you don't need to restart the framework to generate another sorghum data.
sorghum_framework.sorghum_descriptor_to_mesh_and_point_cloud(
	use_gpu,
	".\\SorghumGenerator\\Sample1.sorghum",
	point_cloud_settings,
	mesh_settings,
	#Whether generate 2 point clouds (with/without occclusion) for the sorghum
	False,
	#Whether generate ground surface mesh
	False,
	output_root + "\\SD_Sample1.obj",
	output_root + "\\SD_Sample1.ply"
)

#Now we generate and save mesh and point cloud for 2 sorghums with sorghum state.
sorghum_framework.sorghum_state_to_mesh_and_point_cloud(
	use_gpu,
	#Path to sorghum descriptor [[!!!IF THE SORGHUM DESCRIPTOR IS IN PROJECT FOLDER, YOU SHOULD USE RELATIVE PATH, IF IT'S SAVED OUTSIDE THE FOLDER, USE ABSOLUTE PATH!!!]]
	".\\SorghumGenerator\\Sample0.ss",
	#Sorghum Point Cloud Point Settings
	point_cloud_settings,
	#Sorghum Mesh Generator Settings
	mesh_settings,
	#Whether generate 2 point clouds (with/without occclusion) for the sorghum
	False,
	#Whether generate ground surface mesh
	False,
	#Where to store the 3d model
	output_root + "\\SS_Sample0.obj",
	#Where to save the point cloud
	output_root + "\\SS_Sample0.ply"
)
#Note that you don't need to restart the framework to generate another sorghum data.
sorghum_framework.sorghum_state_to_mesh_and_point_cloud(
	use_gpu,
	".\\SorghumGenerator\\Sample1.ss",
	point_cloud_settings,
	mesh_settings,
	#Whether generate 2 point clouds (with/without occclusion) for the sorghum
	False,
	#Whether generate ground surface mesh
	False,
	output_root + "\\SS_Sample1.obj",
	output_root + "\\SS_Sample1.ply"
)

#Close the framework after we finished data generation
sorghum_framework.engine_terminate()

#Change back to original working directory
os.chdir(current_directory)