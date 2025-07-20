import os
from pathlib import Path
import sys
import argparse
import numpy as np
import pandas as pd
from datetime import datetime, timezone, timedelta
from tqdm import tqdm

# todo: parameters for specific sorghum generator, etc...
parser = argparse.ArgumentParser(description="Sorghum Field Illumination Estimation")

parser.add_argument(
    "--output-path",
    default=r"D:\zhan5455\SorghumIlluminationEstimationResults",
    type=str,
    help="path to store the output",
)

parser.add_argument(
    "--no-sun-file", action="store_true", help="ignore sun_light_dir_source"
)

parser.add_argument(
    "--use-sensors", action="store_true", help="whether to estimate illumination on sensors instead of on the plants"
)

parser.add_argument(
    "--sun-light-dir-source",
    default=r"D:\zhan5455\SorghumData\524042_33.08_-111.97_2021.csv",
    type=str,
)

parser.add_argument("--sun-light-dir-source-skip-row", default=2, type=int)

parser.add_argument("--start-date", default="2021-07-07", type=str)

parser.add_argument("--end-date", default="2021-07-10", type=str)


# If you moved this python script, you should change following lines to make sure they points to the correct directory
current_directory = Path.cwd()
evoengine_directory = current_directory.parent
root_dir = evoengine_directory.parent


library_directory = (
    evoengine_directory.expanduser()
    .joinpath("out")
    .joinpath("build")
    .joinpath("x64-Release")
    .joinpath("PythonBinding")
)

sys.path.append(str(library_directory))
os.add_dll_directory(library_directory)
os.chdir(library_directory)

import PyDigitalAgriculture as sorghum_framework

import utils


def main(args):
    if not os.path.isdir(args.output_path):
        os.mkdir(args.output_path)
    
    utils.initialize_sorghum_app(evoengine_directory)
    mesh_generation_settings = (
        utils.initialize_illumination_estimation_mesh_parameters()
    )

    sorghum_framework.EnableBTF()

    # Sorghum layer: Set CBTFGroup
    cbtf_group_handle = sorghum_framework.GetAssetHandle("./BTFGroup.cbtfgroup")

    sorghum_framework.SetCBTFGroup(cbtf_group_handle)

    # set skydome
    sorghum_framework.SetSkyDome()

    # load the sorghumcoordinates
    sorghum_coordinates_handle = sorghum_framework.GetAssetHandle(
        "./SorghumField/Season12.sorghumcoords"
    )

    # load the sorghum field
    sorghum_field_handle = sorghum_framework.GetAssetHandle(
        "./SorghumField/Season12.sorghumfield"
    )

    # apply the sorghumcoordinates to the field
    sorghum_field_entity = sorghum_framework.InstantiateSorghumField(
        sorghum_field_handle, sorghum_coordinates_handle, 1
    )

    # todo: api for changing settings of the field
    # todo: replace sorghum generator
    # todo: give a customized list of positions of the sorghums
    # todo: give a customized list of rotations of the sorghums

    # generate mesh for the sorghum_field_entity
    sorghum_framework.InitiateSorghumEntity(
        sorghum_field_entity, mesh_generation_settings
    )
    
    # just use the current time and default sun angle if no data source
    sun_light_directions = pd.DataFrame(
        {
            "datetime": [datetime.now().strftime("%Y-%m-%d %H:%M:%S")],
            "sun_direction": [90],
        }
    )

    if not args.no_sun_file:
        sun_light_directions = utils.get_sun_light_direction_sequence(
            args.sun_light_dir_source,
            args.sun_light_dir_source_skip_row,
            args.start_date,
            args.end_date,
        )
    
    # todo: api for setting up PAR sensors
    
    if args.use_sensors:
        print("[Illumination Estimation]: use sensors")
        sensor_asset_handle = sorghum_framework.SetPARSensors(sorghum_field_entity)
    
    # loop through the sun directions and collect the results in a np array
    time = sun_light_directions['datetime'].to_numpy()
    sun_direction = sun_light_directions['sun_direction'].to_numpy()
    results = []
    
    pbar = tqdm(sun_direction, desc="Sun moving")
    for i, x in enumerate(pbar):
        dir = sorghum_framework.Vec3()
        dir.x = 90 - x  
        sorghum_framework.SetSunDirection(dir)
        pbar.set_postfix(time=pd.to_datetime(time[i]).strftime('%Y-%m-%d %H:%M'), zenith=f"{x:.2f}")
        # illumination estimation
        if args.use_sensors:
            sorghum_framework.IlluminationEstimationOnSensors(sensor_asset_handle)

            result = sorghum_framework.GetAllIlluminationEstimationResultsFromSensors(sensor_asset_handle)
        
        else:
            sorghum_framework.IlluminationEstimationOnSorghum()

            result = sorghum_framework.GetAllIlluminationEstimationResultsOnSorghum()

        # N * C * 3 (glm::vec3)
        result_np = np.array(
            [[(v.x, v.y, v.z) for v in row] for row in result], dtype=float
        )
        results.append(result_np)
    
    results = np.array(results)
    # todo: make all results into one numpy array
    
    print(os.path.join(args.output_path, "results.npy"))
    np.save(os.path.join(args.output_path, "results.npy"), results)
    np.save(os.path.join(args.output_path, "timestamps.npy"), time)
    np.save(os.path.join(args.output_path, "sun_directions.npy"), sun_direction)

    if args.use_sensors:
        sorghum_framework.DeleteRuntimeAsset(sensor_asset_handle)
    sorghum_framework.Terminate()


if __name__ == "__main__":
    parsed_args = parser.parse_args()
    main(parsed_args)

# Change back to original working directory
os.chdir(current_directory)
