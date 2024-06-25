import pyecosyslab as pesl
import os

current_directory = os.getcwd()
project_path = "C:\\Users\\lllll\\Documents\\GitHub\\EcoSysLab\\Resources\\EcoSysLabProject\\test.eveproj"
output_root = "D:\\TreePointCloudData\\"

if not os.path.isdir("D:\\TreePointCloudData"):
	os.mkdir("D:\\TreePointCloudData")

pesl.start_project_windowless(project_path)
tmgs = pesl.TreeMeshGeneratorSettings()
pcps = pesl.PointCloudPointSettings()
pcps.m_ballRandRadius = 0.01
pcps.m_junctionIndex = False
pccs = pesl.PointCloudCaptureSettings()
pccs.m_distance = 4.0
pccs.m_height = 3.0
##NOTE: The above code should only be run once for entire application lifespan. Do not start framework multiple times within single execution.

##NOTE: You may run below line multiple times for exporting multiple OBJs from multiple binvox inputs.
#Parameters: 
#1.		point cloud point settings
#2.		point cloud capture settings
#3.		tree descriptor absolute path
#4.		growth delta time (0.0822 years equal to 1 month)
#5.		growth iteration count (96 iterations of 1 month is 8 years, which gives you a old tree by default)
#6.		max tree node count
#7.		tree mesh generator settings
#8.		tree point cloud output path
#9.		enable tree mesh export
#10.		tree mesh output path
#11		enable junction export
#12		junction path

dsg = pesl.DatasetGenerator()

index = 0
numberPerSpecie = 50
for x in range(0, numberPerSpecie):
	target_descriptor_path = "C:\\Users\\lllll\\Documents\\GitHub\\EcoSysLab\\Resources\\EcoSysLabProject\\TreeDescriptors\\Elm.td"
	name = "Elm_" + str(index)
	target_tree_mesh_path = output_root + name + ".obj"
	target_tree_pointcloud_path = output_root + name + ".ply"
	target_tree_junction_path = output_root + name + ".yml"
	dsg.GeneratePointCloudForTree(pcps, pccs, target_descriptor_path, 0.08220, 999, 20000, tmgs, target_tree_pointcloud_path, False, target_tree_mesh_path, False, target_tree_junction_path)
	index += 1

for x in range(0, numberPerSpecie):
	target_descriptor_path = "C:\\Users\\lllll\\Documents\\GitHub\\EcoSysLab\\Resources\\EcoSysLabProject\\TreeDescriptors\\Maple.td"
	name = "Maple_" + str(index)
	target_tree_mesh_path = output_root + name + ".obj"
	target_tree_pointcloud_path = output_root + name + ".ply"
	target_tree_junction_path = output_root + name + ".yml"
	dsg.GeneratePointCloudForTree(pcps, pccs, target_descriptor_path, 0.08220, 999, 20000, tmgs, target_tree_pointcloud_path, False, target_tree_mesh_path, False, target_tree_junction_path)
	index += 1