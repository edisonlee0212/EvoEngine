import os
from pathlib import Path
import sys
import argparse


# parser = argparse.ArgumentParser(description="Operation Classification")

# todo: add arguments


# If you moved this python script, you should change following lines to make sure they points to the correct directory
current_directory = Path.cwd()
evoengine_directory = current_directory.parent
root_dir = evoengine_directory.parent

output_root = root_dir.joinpath( "IlluminationEstimationResults")

library_directory = evoengine_directory.expanduser().joinpath("out").joinpath("build").joinpath("x64-Release").joinpath("PythonBinding")

sys.path.append(str(library_directory))
os.chdir(library_directory)

import PyDigitalAgriculture as sorghum_framework


# Point the framework to load the default project folder that contains 2 sample sorghum descriptors.
project_path = evoengine_directory.expanduser().joinpath("Resources").joinpath("DigitalAgricultureProject").joinpath("test.eveproj")

# Create new folder for output path if necessary
if not os.path.isdir(output_root):
	os.mkdir(output_root)

print(dir(sorghum_framework))
# Enable GPU
use_gpu = True

# Start the framework without editor and window.
if use_gpu:
	sorghum_framework.PushRayTracerLayer()

sorghum_framework.RegisterClasses()
sorghum_framework.PushSorghumLayer()
sorghum_framework.PushRayTracerLayer()
sorghum_framework.Run(project_path)

# sorghum generation parameters
data_generation_parameters = sorghum_framework.SorghumDataGenerationParameters()
#Whether generate ground surface mesh
data_generation_parameters.generate_ground_mesh = False
#Whether we want to simulate no occlusion (we will sample point cloud for each sorghum leaf individually and combine them together)
data_generation_parameters.avoid_occlusion = False

#Should panicle geometry be generated
data_generation_parameters.sorghum_mesh_generator_settings.enable_panicle = True
#Should stem geometry be generated
data_generation_parameters.sorghum_mesh_generator_settings.enable_stem = False
#Should leaf geometry be generated
data_generation_parameters.sorghum_mesh_generator_settings.enable_leaves = True
#Should geometry of stem part of each leaf be generated
data_generation_parameters.sorghum_mesh_generator_settings.enable_leaf_sheath = True
#If -1, generate all leaves, otherwise generate specific leaf only
data_generation_parameters.sorghum_mesh_generator_settings.single_leaf_index = -1
#Should leaf mesh contain 2 faces
data_generation_parameters.sorghum_mesh_generator_settings.bottom_face = True
#Should all leaves be generated in separated meshes or combined as one single mesh, if set to false, you will not get leaf index in point cloud.
data_generation_parameters.sorghum_mesh_generator_settings.leaf_separated = True
#If bottom_face is ON, this sets the distance between top and bottom face of each leaf
data_generation_parameters.sorghum_mesh_generator_settings.leaf_thickness = 0.001



# Sorghum Layer: Enable BTF
sorghum_framework.EnableBTF() 

# Sorghum layer: Set CBTFGroup
cbtf_group_handle = sorghum_framework.GetAssetHandle("./BTFGroup.cbtfgroup")
print("load cbtgroup with handle: ", cbtf_group_handle.GetValue())
sorghum_framework.SetCBTFGroup(cbtf_group_handle)

# Add sorghum to the scene
seed = 0
sorghum_state_handle = sorghum_framework.GetAssetHandle("./SorghumGenerator/Sample1.sorghum")
sorghum_entity = sorghum_framework.CreateEntityFromSorghumDescriptor(sorghum_state_handle)

data_generation_parameters.output_file_name = "SD_Sample"



sorghum_framework.InitiateSorghumEntity(
	sorghum_entity,
	data_generation_parameters
)

print("BTFMeshRenderer exists? : " )
btf_exist = sorghum_framework.CheckBTFComponentsExist()
print("BTFMeshRenderer exists? : " , btf_exist)

# set skydome
sorghum_framework.SetSkyDome()

# todo: set samples and bonces of rays
# run illumination estimation
sorghum_framework.IlluminationEstimation(sorghum_entity)



#==================================#
#            Clean up              #
#==================================#

#Close the framework after we finished data generation
sorghum_framework.Terminate()

#Change back to original working directory
os.chdir(current_directory)
# def main(args):
	


# if __name__ == "__main__":

#     parsed_args = parser.parse_args()
#     main(parsed_args)