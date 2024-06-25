import pyecosyslab as pesl
import os
current_directory = os.getcwd()

project_path = "C:\\Users\\lllll\\Documents\\GitHub\\EvoEngine\\Resources\\Example Projects\\Rendering\\Rendering.eveproj"
output_path = current_directory + "\\out.png"

pesl.start_project_windowless(project_path)
pesl.capture_scene(0, 0, 10, 0, 0, 0, 1920, 1080, output_path)
