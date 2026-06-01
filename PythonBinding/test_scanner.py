import os
from pathlib import Path
import sys
import argparse
import numpy as np
import pandas as pd




parser = argparse.ArgumentParser(description="Test scanners")

parser.add_argument(
    "--output-folder",
    default=r"..\..\VRBioTalk",
    type=str,
    help="path to store the output",
)

parser.add_argument(
    "--input-path",
    default=r"..\..\VRBioTalk\test_plants",
    type=str,
    help="path to the input folder containing obj/npz pairs, or a single obj",
)

parser.add_argument(
    "--grid-size",
    default=8,
    type=int,
    help="Size of the grid",
)

parser.add_argument(
    "--grid-distance",
    default=0.75,
    type=float,
    help="Distance between the grids",
)

parser.add_argument(
    "--scan-step",
    default=0.005,
    type=float,
    help="Expected distance (m) between points, smaller distance means more points.",
)

parser.add_argument(
    "--scanner-height",
    default=2.5,
    type=float,
    help="height of the scanner (m) from the ground level",
)





# If you moved this python script, you should change following lines to make sure they points to the correct directory
current_directory = Path.cwd()
evoengine_directory = current_directory.parent
root_dir = evoengine_directory.parent


library_directory = evoengine_directory.expanduser().joinpath("out").joinpath("build").joinpath("x64-Release").joinpath("PythonBinding")

sys.path.append(str(library_directory))
os.chdir(library_directory)


import PyDigitalAgriculture as sorghum_framework
import utils




def collect_mesh_label_pairs(input_path: Path):
    if not input_path.exists():
        raise FileNotFoundError(f"Input path does not exist: {input_path}")

    if input_path.is_file():
        obj_paths = [input_path]
    else:
        obj_paths = sorted(input_path.glob("*.obj"))

    pairs = []
    for obj_path in obj_paths:
        npz_path = obj_path.with_suffix(".npz")
        if not npz_path.exists():
            print(f"Skipping {obj_path.name}: missing {npz_path.name}")
            continue
        pairs.append((obj_path, npz_path))

    if not pairs:
        raise FileNotFoundError(f"No obj/npz pairs found in {input_path}")

    return pairs


def resolve_user_path(path_str: str) -> Path:
    path = Path(path_str).expanduser()
    if path.is_absolute():
        return path.resolve()
    return (current_directory / path).resolve()



def main(args):


    utils.initialize_sorghum_app(evoengine_directory)
    data_generation_parameters = utils.initialize_illumination_estimation_mesh_parameters()


    input_path = resolve_user_path(args.input_path)
    output_folder = resolve_user_path(args.output_folder)
    mesh_label_pairs = collect_mesh_label_pairs(input_path)
    
    # gantry capture settings
    grid_dimension = args.grid_size
    grid_distance = args.grid_distance

    gantry_capture_settings = sorghum_framework.SorghumGantryCaptureSettings()
    #Expected distance (m) between points, smaller distance means more points.
    gantry_capture_settings.step = args.scan_step
    #Expected distance (m) between capture point and ground level
    gantry_capture_settings.sample_height = args.scanner_height
    #Whether output spline info (yaml)
    gantry_capture_settings.output_spline_info = False
    #The dimension of the grid
    gantry_capture_settings.grid_size.x = gantry_capture_settings.grid_size.y = grid_dimension
    #Distance between sorghums
    gantry_capture_settings.grid_distance.x = gantry_capture_settings.grid_distance.y = grid_distance
    
    point_cloud_point_settings = sorghum_framework.SorghumPointCloudPointSettings()
    #The variance of uncertainty on point positions for simulating inaccuracy
    point_cloud_point_settings.variance = 0.015
    #The uniform random range of uncertainty on point positions for simulating inaccuracy
    point_cloud_point_settings.ball_rand_radius = 0.01

    #Should mesh type be included for each point (0 = leaf, 1 = stem, 2 = panicle, 3 = ground surface, -1 = everything else)
    point_cloud_point_settings.type_index = True
    #If multiple sorghums are in the scene, should their instance indices be included for each sorghum (All will be 0 if only one sorghum present)
    point_cloud_point_settings.instance_index = True
    #Should indices of leaves be included for each point (Start from bottom leaf and first index is 0)
    point_cloud_point_settings.leaf_index = True
    #The bounding box size for scanning. Set to 2.0 as default to include full sorghum geometry (Scanner simulates Gantry, so it needs a bigger bounding box to include full geometry)
    point_cloud_point_settings.bounding_box_limit = 2.0
    
    data_generation_parameters.sorghum_point_cloud_point_settings = point_cloud_point_settings
    
    data_generation_parameters.output_folder = output_folder
    data_generation_parameters.output_file_name = "test_scan"

    targets = []
    label_lists = []
    instance_spacing = args.grid_distance
    row_size = max(1, int(np.ceil(np.sqrt(len(mesh_label_pairs)))))

    for index, (obj_path, npz_path) in enumerate(mesh_label_pairs):
        prefab_handle = sorghum_framework.ImportRuntimeAsset("Prefab", str(obj_path.resolve()))

        grid_x = index % row_size
        grid_z = index // row_size
        position = utils.make_vec3(grid_x * instance_spacing, 0.0, grid_z * instance_spacing)
        # input to rotation should be in the unit of radians
        euler_rotation = utils.make_vec3(np.radians(0), 0.0, 0.0)
        scale = utils.make_vec3(1.0, 1.0, 1.0)
        entity = sorghum_framework.CreateEntityFromPrefab(prefab_handle, position, euler_rotation, scale)

        npz_data = np.load(npz_path, allow_pickle=True)
        face_labels = np.asarray(npz_data["instance_labels"], dtype=np.int32)
        unique_labels = np.unique(face_labels)

        print(f"[{index}] Loaded mesh: {obj_path}")
        print(f"[{index}] Loaded labels: {npz_path}")
        print(f"[{index}] instance_labels shape: {face_labels.shape}")
        print(f"[{index}] unique labels ({len(unique_labels)}): {unique_labels.tolist()}")
        print(f"[{index}] position: ({position.x}, {position.y}, {position.z})")



        targets.append(entity)
        label_lists.append(face_labels.tolist())
        
    sorghum_framework.ScanLabeledMeshes(
        True,
        targets,
        label_lists,
        gantry_capture_settings,
        data_generation_parameters,
    )



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
