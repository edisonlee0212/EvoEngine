import os

#If you moved this python script, you should change following lines to make sure they points to the correct directory
file_path = os.path.abspath(__file__)
file_folder = os.path.dirname(file_path)
evoengine_directory = os.path.dirname(file_folder) + "/"
root_dir = os.path.dirname(evoengine_directory)
#You may modify output folder path here.
output_root = os.path.dirname(root_dir) + "/TreeData"

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
import PyEcoSysLab as tree_framework

#Project folder path
project_folder_path = os.path.expanduser(evoengine_directory + "Resources/EcoSysLabProject/")

#Point the framework to load the default project
project_path = os.path.expanduser(evoengine_directory + "Resources/EcoSysLabProject/test.eveproj")

#Create new folder for output path if necessary
if not os.path.isdir(output_root):
	os.mkdir(output_root)

#Start the framework without editor and window.
tree_framework.RegisterClasses()
tree_framework.PushEcoSysLabLayer()
tree_framework.Run(project_path)

#==================================#
#         Configurations           #
#==================================#
#Following configurations are defined in PythonBinding/src/PyEcoSysLab.cpp. You may check all available settings there.
#You may modify connectivity graph settings here.
connectivity_graph_settings = tree_framework.ConnectivityGraphSettings()
connectivity_graph_settings.zigzag_check = True
#You may modify reconstruction settings here.
reconstruction_settings = tree_framework.ReconstructionSettings()
reconstruction_settings.end_node_thickness = 0.004
reconstruction_settings.apply_root_thickness = False
#You may modify mesh generation settings here.
mesh_generator_settings = tree_framework.TreeMeshGeneratorSettings()
#You may modify import scale here.
import_scale = 0.1

#You may modify path to the input yaml file here.
yaml_input_path = project_folder_path + "/tree_structor_example.yml"
#You may modify output file name here.
output_file_name = "reconstructed_result"

tree_framework.tree_structor(yaml_input_path, import_scale, connectivity_graph_settings, reconstruction_settings, mesh_generator_settings, output_root, output_file_name)

#Terminate engine
tree_framework.Terminate()

#Change back to original working directory
os.chdir(current_directory)