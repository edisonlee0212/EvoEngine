with open('src/TreeVisualizer.cpp', 'r') as f:
    d = f.read()

# Replace the specific case in the true branch for line_thickness
d = d.replace('line_thickness * (sub_tree ? 1.25f : 1.0f), node.info.length,\n',
              'line_thickness * (sub_tree ? 1.25f : 1.0f), node.info.length + line_thickness,\n')
d = d.replace('line_thickness * (sub_tree ? 1.25f : 1.0f), node.info.length,\r\n',
              'line_thickness * (sub_tree ? 1.25f : 1.0f), node.info.length + line_thickness,\r\n')

# Then replace the default thickness ones
d = d.replace('node.info.length, node.info.thickness', 'node.info.length + node.info.thickness, node.info.thickness')

with open('src/TreeVisualizer.cpp', 'w') as f:
    f.write(d)
