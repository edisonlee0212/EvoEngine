import os
from pathlib import Path
import sys
import argparse
import numpy as np
import pandas as pd




parser = argparse.ArgumentParser(description="Test Ray Properties of Illumination Estimation")

parser.add_argument(
    "--output-path",
    default=r"C:\Users\sdjkl\CG\VRBioTalk\results.csv",
    type=str,
    help="path to store the output",
)

parser.add_argument(
    "--max-samples", default=64, type=int, help="max samples for ray tracing"
)

parser.add_argument(
    "--max-bounces", default=10, type=int, help="max bounces for ray tracing"
)

parser.add_argument("--enable-btf", action="store_true", help="whether to enable BTF in this test")



# If you moved this python script, you should change following lines to make sure they points to the correct directory
current_directory = Path.cwd()
evoengine_directory = current_directory.parent
root_dir = evoengine_directory.parent

output_root = root_dir.joinpath( "IlluminationEstimationResults")

library_directory = evoengine_directory.expanduser().joinpath("out").joinpath("build").joinpath("x64-Release").joinpath("PythonBinding")

sys.path.append(str(library_directory))
os.chdir(library_directory)


import PyDigitalAgriculture as sorghum_framework

import utils

def test_one_sorghum(mesh_generation_settings)->pd.DataFrame:
    # Add sorghum to the scene
    seed = 0
    sorghum_handle = sorghum_framework.GetAssetHandle("./SorghumGenerator/Sample1.sorghum")
    sorghum_entity = sorghum_framework.CreateEntityFromSorghumDescriptor(sorghum_handle)

    mesh_generation_settings.output_file_name = "SD_Sample"



    sorghum_framework.InitiateSorghumEntity(
        sorghum_entity,
        mesh_generation_settings
    )

    print("BTFMeshRenderer exists? : " )
    btf_exist = sorghum_framework.CheckBTFComponentsExist()
    print("BTFMeshRenderer exists? : " , btf_exist)

    # set skydome
    sorghum_framework.SetSkyDome()

    rows = []

    for samples in range(1, args.max_samples + 1):
        for bounces in range(1, args.max_bounces + 1):
            print(f"Start ray tracing with samples: {samples}, bounces: {bounces}")
            sorghum_framework.SetIlluminationSamples(samples, bounces)
            sorghum_framework.IlluminationEstimationOnSorghum()
            result = sorghum_framework.GetAllIlluminationEstimationResultsOnSorghum()

            for sorghum_idx, sorghum_result in enumerate(result):

                position, rotation, total_area, total_flux, average_flux = sorghum_result

                rows.append({
                    "samples": samples,
                    "bounces": bounces,
                    "sorghum_idx": sorghum_idx,

                    "position_x": position.x,
                    "position_y": position.y,
                    "position_z": position.z,

                    "rotation_x": rotation.x,
                    "rotation_y": rotation.y,
                    "rotation_z": rotation.z,

                    "total_area_x": total_area.x,
                    "total_area_y": total_area.y,
                    "total_area_z": total_area.z,

                    "total_flux_x": total_flux.x,
                    "total_flux_y": total_flux.y,
                    "total_flux_z": total_flux.z,

                    "average_flux_x": average_flux.x,
                    "average_flux_y": average_flux.y,
                    "average_flux_z": average_flux.z,
                })

    df = pd.DataFrame(rows)
    df.to_csv(f"{args.output_path}", index=False)
    return df


def test_sorghum_field(mesh_generation_settings):

    seed = 0
    # Build field grid
    sorghum_grid = sorghum_framework.SorghumGrid()
    sorghum_grid.grid_size.x = 10
    sorghum_grid.grid_size.y = 10
    sorghum_grid.grid_distance.x = 0.3
    sorghum_grid.grid_distance.y = 0.3
    sorghum_grid.position_offset_mean = 0.0
    sorghum_grid.position_offset_variance = 0.0
    sorghum_grid.rotation_variance_y = 180.0  # uniform azimuth randomization

    field_handle = sorghum_framework.CreateRuntimeAsset("SorghumField")
    
    sg_handle = sorghum_framework.GetAssetHandle("./SorghumGenerator/Random.sg")
    sorghum_framework.ApplySorghumGrid(field_handle, sg_handle, sorghum_grid)
    field_entity = sorghum_framework.CreateEntityFromSorghumField(field_handle, seed) 

    mesh_generation_settings.output_file_name = "SD_Sample"

    print("BTFMeshRenderer exists? : " )
    btf_exist = sorghum_framework.CheckBTFComponentsExist()
    print("BTFMeshRenderer exists? : " , btf_exist)

    # set skydome
    sorghum_framework.SetSkyDome()

    rows = []

    for samples in range(1, args.max_samples + 1):
        for bounces in range(1, args.max_bounces + 1):
            print(f"Start ray tracing with samples: {samples}, bounces: {bounces}")
            sorghum_framework.SetIlluminationSamples(samples, bounces)
            sorghum_framework.IlluminationEstimationOnSorghum()
            result = sorghum_framework.GetAllIlluminationEstimationResultsOnSorghum()

            for sorghum_idx, sorghum_result in enumerate(result):

                position, rotation, total_area, total_flux, average_flux = sorghum_result

                rows.append({
                    "samples": samples,
                    "bounces": bounces,
                    "sorghum_idx": sorghum_idx,

                    "position_x": position.x,
                    "position_y": position.y,
                    "position_z": position.z,

                    "rotation_x": rotation.x,
                    "rotation_y": rotation.y,
                    "rotation_z": rotation.z,

                    "total_area_x": total_area.x,
                    "total_area_y": total_area.y,
                    "total_area_z": total_area.z,

                    "total_flux_x": total_flux.x,
                    "total_flux_y": total_flux.y,
                    "total_flux_z": total_flux.z,

                    "average_flux_x": average_flux.x,
                    "average_flux_y": average_flux.y,
                    "average_flux_z": average_flux.z,
                })

    df = pd.DataFrame(rows)
    df.to_csv(f"{args.output_path}", index=False)
    return df

def main(args):


        
    utils.initialize_sorghum_app(evoengine_directory)
    mesh_generation_settings = (
        utils.initialize_illumination_estimation_mesh_parameters()
    )

    if(args.enable_btf):
        # Sorghum Layer: Enable BTF
        sorghum_framework.EnableBTF() 

        # Sorghum layer: Set CBTFGroup
        cbtf_group_handle = sorghum_framework.GetAssetHandle("./BTFGroup.cbtfgroup")
        print("load cbtgroup with handle: ", cbtf_group_handle.GetValue())
        sorghum_framework.SetCBTFGroup(cbtf_group_handle)

    test_one_sorghum(mesh_generation_settings)
    # test_sorghum_field(mesh_generation_settings)
    

    #==================================#
    #            Clean up              #
    #==================================#

    #Close the framework after we finished data generation
    sorghum_framework.Terminate()

    #Change back to original working directory

	


if __name__ == "__main__":
    args = parser.parse_args()
    main(args)
    
os.chdir(current_directory)