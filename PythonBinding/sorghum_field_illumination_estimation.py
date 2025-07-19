import os
from pathlib import Path
import sys
import argparse
import numpy as np

# todo: parameters
# parser = argparse.ArgumentParser(description="Operation Classification")



# If you moved this python script, you should change following lines to make sure they points to the correct directory
current_directory = Path.cwd()
evoengine_directory = current_directory.parent
root_dir = evoengine_directory.parent

output_root = root_dir.joinpath( "IlluminationEstimationResults")

library_directory = evoengine_directory.expanduser().joinpath("out").joinpath("build").joinpath("x64-Release").joinpath("PythonBinding")

sys.path.append(str(library_directory))
os.add_dll_directory(library_directory)
os.chdir(library_directory)

import PyDigitalAgriculture as sorghum_framework

import utils


# Create new folder for output path if necessary
if not os.path.isdir(output_root):
	os.mkdir(output_root)

utils.initialize_sorghum_app(evoengine_directory)

mesh_generation_settings = utils.initialize_illumination_estimation_mesh_parameters()


sorghum_framework.EnableBTF() 

# Sorghum layer: Set CBTFGroup
cbtf_group_handle = sorghum_framework.GetAssetHandle("./BTFGroup.cbtfgroup")

sorghum_framework.SetCBTFGroup(cbtf_group_handle)

# set skydome
sorghum_framework.SetSkyDome()

# todo: sun direction

# load the sorghumcoordinates
sorghum_coordinates_handle = sorghum_framework.GetAssetHandle("./SorghumField/Season12.sorghumcoords")

# load the sorghum field
sorghum_field_handle = sorghum_framework.GetAssetHandle("./SorghumField/Season12.sorghumfield")

# apply the sorghumcoordinates to the field
sorghum_field_entity = sorghum_framework.InstantiateSorghumField(sorghum_field_handle, sorghum_coordinates_handle, 1)

# todo: api for change settings of the field
# todo: replace sorghum generator
# todo: give a customized list of positions of the sorghums
# todo: give a customized list of rotations of the sorghums

# generate mesh for the sorghum_field_entity
sorghum_framework.InitiateSorghumEntity(sorghum_field_entity, mesh_generation_settings)

# illumination estimation
sorghum_framework.IlluminationEstimation()

# get the result
results = sorghum_framework.GetAllIlluminationEstimationResults()


results_np = np.array([[(v.x, v.y, v.z) for v in row] for row in results], dtype=float)
# todo: handle the results

sorghum_framework.Terminate()

#Change back to original working directory
os.chdir(current_directory)