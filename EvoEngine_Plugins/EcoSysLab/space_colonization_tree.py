import pyecosyslab as pesl
import os

current_directory = os.getcwd()

if not os.path.isdir(current_directory + "\\scd"):
	os.mkdir(current_directory + "\\scd")

project_path = "C:\\Users\\lllll\\Desktop\\EcoSysLabProject\\test.eveproj"
binvox_path = "C:\\Users\\lllll\\Downloads\\binvox_files\\san_jose_acer_san_jose_9050_0_mesh.binvox"
target_descriptor_path = "C:\\Users\\lllll\\Desktop\\EcoSysLabProject\\TreeDescriptors\\Acacia.td"
target_voxel_tree_mesh_path = current_directory + "\\scd\\voxel_tree.obj"
target_voxel_tree_tree_io_path = current_directory + "\\scd\\voxel_tree.treeio"
target_rbv_path = current_directory + "\\scd\\rbv.txt"
target_voxel_tree_rbv_mesh_path = current_directory + "\\scd\\voxel_rbv.obj"

target_rbv_tree_mesh_path = current_directory + "\\scd\\rbv_tree.obj"
target_rbv_tree_tree_io_path = current_directory + "\\scd\\rbv_tree.treeio"
target_rbv_tree_rbv_mesh_path = current_directory + "\\scd\\rbv_tree_rbv.obj"

target_rbv_rbv_mesh_path = current_directory + "\\scd\\rbv_rbv.obj"

pesl.start_project_windowless(project_path)
tmgs = pesl.TreeMeshGeneratorSettings()

##NOTE: The above code should only be run once for entire application lifespan. Do not start framework multiple times within single execution.

##NOTE: You may run below line multiple times for exporting multiple OBJs from multiple binvox inputs.
#Parameters: 
#1.		voxel grid radius
#2.		binvox absolute path
#3.		tree descriptor absolute path
#4.		growth delta time
#5.		growth iteration count

#6.		Tree mesh generator settings
#7.		enable tree mesh export
#8.		tree mesh output path

#9.		enable tree io
#10.	tree io output path
#11.	enable RBV
#12.	RBV output path
#13.	enable RBV mesh
#14.	RBV mesh output path
pesl.voxel_space_colonization_tree_data(2.0, binvox_path, target_descriptor_path, 0.08220, 250, tmgs, True, target_voxel_tree_mesh_path, True, target_voxel_tree_tree_io_path, True, target_rbv_path, True, target_voxel_tree_rbv_mesh_path)

##NOTE: You may run below line multiple times for exporting multiple OBJs from multiple rbv inputs.
#Parameters: 
#1.		rbv absolute path
#2.		tree descriptor absolute path
#3.		growth delta time
#4.		growth iteration count

#5.		Tree mesh generator settings
#6.		enable tree mesh export
#7.		tree mesh output path

#8.		enable tree io
#9.	tree io output path
#10.	enable RBV mesh
#11.	RBV mesh output path
pesl.rbv_space_colonization_tree_data(target_rbv_path, target_descriptor_path, 0.08220, 250, tmgs, True, target_rbv_tree_mesh_path, True, target_rbv_tree_tree_io_path, True, target_rbv_tree_rbv_mesh_path)


##NOTE: You may run below line multiple times for exporting OBJ for given RBV.
#Parameters: 
#1.		rbv absolute path
#2.		RBV mesh output path
pesl.rbv_to_obj(target_rbv_path, target_rbv_rbv_mesh_path);