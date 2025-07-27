import os
import numpy as np
import yaml
import pickle
from plyfile import PlyData, PlyElement
from tqdm import tqdm
import trimesh
from IPython import embed
import multiprocessing



def write_obj(filepath, vertex, faces):
    output = []
    for v in vertex:
        output.append("v %f, %f, %f\n"%(v[0], v[1], v[2]))
    for f in faces:
        output.append("f %d %d %d\n"%(f[0]+1, f[1]+1, f[2]+1))
    
    with open(filepath, 'w') as f:
        f.writelines(output)
        
    
def interpolate_spline(sphere_center, num=6, spline_num=-1):
    
    assert spline_num != -1
    sphere_center = sphere_center.reshape(spline_num, -1, 3)
    
    res = []
    for spline in sphere_center:
        for i in range(1, spline.shape[0]):
            prev = spline[i-1]
            curr = spline[i]
            
            dist = np.sqrt((curr - prev) ** 2)
            interval = dist / num
            direction = (curr - prev) / dist
            
            for idx in range(num+1):
                res.append(prev + direction * interval * idx)
        
    return np.array(res)


def sphere_visualize(sphere_center, sphere_radius, pc=None, interpolate=False, spline_num=-1, color=None):
    # visualize
    # generate sphere
    u = np.random.rand(10,1) * 2 - 1
    v = np.random.rand(10,1) * 2 - 1
    w = np.random.rand(10,1) * 2 - 1

    norm = (u*u + v*v + w*w)**(0.5)

    xi,yi,zi = u/norm,v/norm,w/norm
    
    if interpolate:
        sphere_center = interpolate_spline(sphere_center, num=6, spline_num=spline_num)
    
    sphere = np.concatenate((xi, yi, zi), axis=1)[None, :] * sphere_radius + sphere_center[:, None]
    sphere = sphere.reshape(-1, 3)
    
    if not color:
        sphere_color = np.ones_like(sphere) * 255
        sphere_color[:, 1:] = 0
    else:
        sphere_color = np.ones_like(sphere) * 255
        for i in range(3):
            sphere_color[:, i] = color[i]
        
    sphere = np.concatenate((sphere, sphere_color), axis=-1)

    if pc:
        pc = np.concatenate((pc, sphere), axis=0)

    return pc, sphere


def save_ply(fn, xyz, color=None):

    with open(fn, 'w') as f:
        pn = xyz.shape[0]
        f.write('ply\n')
        f.write('format ascii 1.0\n')
        f.write('element vertex %d\n' % (pn))
        f.write('property float x\n')
        f.write('property float y\n')
        f.write('property float z\n')
        f.write('end_header\n')
        for i in range(pn):
            if color is None:
                f.write('%.6f %.6f %.6f\n' % (xyz[i][0], xyz[i][1], xyz[i][2]))


def update_treepart(treepart, node, node_density, connectivity):
    
    if 'BSP' in treepart:
        start_pos = treepart['BSP']
        end_pos = treepart['BEP']
    else:
        start_pos = treepart['SP']
        end_pos = treepart['EP']

    beginning_node_density = np.sum(np.array(start_pos)**2)
    end_node_density = np.sum(np.array(end_pos)**2)
    parent = np.argwhere(beginning_node_density == node_density)[0].tolist()[0]
    
    node = np.concatenate([node, np.array(end_pos)[None, :]], axis=0)
    node_density = np.append(node_density, end_node_density)
    connectivity.append([parent, node_density.shape[0]-1])
    
    return node, node_density, connectivity


def parse_yaml(filenames):

    min_edge_num = np.inf
    max_edge_num = -1
    
    offset_factor = 0
    scale = 1.0
    
    for filename in tqdm(filenames):
        
        with open(os.path.join(data_root, 'graph', filename), 'r') as f:
            data = yaml.safe_load(f)['Skeleton']['Flows']
        
        remapper = {-1: 0}

        connectivity = []
        vertex = [[0,0,0]]
        dir = [[0,1,0]]
        radius_list = [data[0]['SR']]
        
        curr_edge_num = len(data)
        min_edge_num = min_edge_num if min_edge_num < curr_edge_num else curr_edge_num
        max_edge_num = max_edge_num if max_edge_num > curr_edge_num else curr_edge_num
        
        # generate skeleton info
        for tree in data:
            tree_idx = tree['I']
            position = np.array(tree['EP'])
            direction = np.array(tree['ED'])
            radius = tree['ER']
            parent_idx = tree['PI']
            
            remapper[tree_idx] = len(vertex)
            vertex.append(position.tolist())
            dir.append(direction.tolist())
            radius_list.append(radius)
            
            if parent_idx in remapper:
                connectivity.append([remapper[tree_idx], remapper[parent_idx]])
        
        data_dict = {
            'nodes': vertex,
            'edges': connectivity,
            'direction': dir,
            'radius': radius_list
        }
        
        with open(os.path.join(root, 'skeleton', filename.split('.')[0][:-9]+'.pkl'), 'wb') as f:
            pickle.dump(data_dict, f)
        
        # generate paper cut mesh
        connectivity_list = np.array(data_dict['edges'])
        node_list = np.array(data_dict['nodes'])
        radius_list = np.array(data_dict['radius'])

        offset = np.ones_like(node_list) * radius_list[:, None]
        offset[:, :2] *= 0
        offset[:, -1] = offset[:, -1] + offset_factor
        
        duplicated_node_list = np.concatenate([node_list, node_list+offset], axis=-1).reshape(-1, 3)
        duplicated_node_list[:, 2] *= scale
        
        faces = []
        for edge in connectivity_list:
            s, e = edge.tolist()
            faces.append([2*s+1, 2*s, e*2])
            faces.append([2*e, 2*e+1, 2*s+1])
        
        mesh = trimesh.Trimesh(vertices=duplicated_node_list.tolist(),
                        faces=faces)
        mesh.export(os.path.join(root, "paper_cut_meshes", filename.split('.')[0][:-9]+'.obj'))
        
        
        #generate low resolution mesh
        connectivity_list = np.array(data_dict['edges'])
        node_list = np.array(data_dict['nodes'])
        radius_list = np.array(data_dict['radius'])

        offset_z = np.ones_like(node_list) * radius_list[:, None]
        offset_z[:, :2] *= 0
        offset_z[:, 2] = offset_z[:, 2] + offset_factor
        
        offset_x = np.ones_like(node_list) * radius_list[:, None]
        offset_x[:, 1:] *= 0
        offset_x[:, 0] = offset_x[:, 0] + offset_factor
        
        duplicated_node_list = np.concatenate([node_list, node_list+offset_z, node_list+offset_x+offset_z, node_list+offset_x], axis=-1).reshape(-1, 3)
        duplicated_node_list[:, 2] *= scale
        
        faces = []
        for edge in connectivity_list: 
            #s=0, e=1
            # 0,1,5; 5,4,0
            # 1,2,6; 6,5,1
            # 2,3,7; 7,6,2
            # 3,0,4; 4,7,3
            
            s, e = edge.tolist()
            faces.append([s*4, s*4+1, e*4+1])
            faces.append([e*4+1, e*4, s*4])
            faces.append([s*4+1, s*4+2, e*4+2])
            faces.append([e*4+2, e*4+1, s*4+1])
            faces.append([s*4+2, s*4+3, e*4+3])
            faces.append([e*4+3, e*4+2, s*4+2])
            faces.append([s*4+3, s*4, e*4])
            faces.append([e*4, e*4+3, s*4+3])
        
        mesh = trimesh.Trimesh(vertices=duplicated_node_list.tolist(),
                        faces=faces)
        mesh.export(os.path.join(root, "low_res_meshes", filename.split('.')[0][:-9]+'.obj'))
        
    
    print("min_edge_num: ", min_edge_num)
    print("max_edge_num: ", max_edge_num)
    


if __name__ == "__main__":
    
    data_root = "/data/zhou1178/MeshGPT_TreeStructor_v2/"
    root = "/data/zhou1178/MeshGPT_TreeStructor_v2/"
    dst = "paper_cut_meshes"
    
    # root = "/media/dummy1/zhou1178/PointCloudTreePart"
    os.makedirs(os.path.join(root, "skeleton"), exist_ok=True)
    # os.makedirs(os.path.join(root, "pointcloud"), exist_ok=True)
    # os.makedirs(os.path.join(root, "paper_cut"), exist_ok=True)
    os.makedirs(os.path.join(root, dst), exist_ok=True)
    
    data_list = os.listdir(os.path.join(data_root, 'graph'))
    data_list.sort()
    
    thread_num = 20
    file_num_per_thread = len(data_list) // thread_num
    
    input_list = []
    for i in range(thread_num):
        input_list.append(data_list[i*file_num_per_thread:(i+1)*file_num_per_thread])
    
    ##Debug
    # parse_yaml(['Spruce_1.yml'])
    # embed()
    with multiprocessing.Pool(len(input_list)) as pool:
        for i in pool.imap_unordered(parse_yaml, input_list):
            pass

    # parse_yaml(data_list)