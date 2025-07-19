#note: import after the path has been set the library direcctory

import PyDigitalAgriculture as sorghum_framework
from pathlib import Path
import os

def initialize_sorghum_app(evoengine_directory: str):
    # Point the framework to load the default project folder that contains 2 sample sorghum descriptors.
    project_path = evoengine_directory.expanduser().joinpath("Resources").joinpath("DigitalAgricultureProject").joinpath("test.eveproj")


    # Enable GPU
    use_gpu = True

    # Start the framework without editor and window.
    if use_gpu:
        sorghum_framework.PushRayTracerLayer()

    sorghum_framework.RegisterClasses()
    sorghum_framework.PushSorghumLayer()
    sorghum_framework.PushRayTracerLayer()
    sorghum_framework.Run(project_path)
    

def initialize_illumination_estimation_mesh_parameters()->sorghum_framework.SorghumDataGenerationParameters:
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
    return data_generation_parameters

