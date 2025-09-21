
#include "Tree.hpp"

using namespace eco_sys_lab_plugin;

bool Tree::ParseBinvox(const std::filesystem::path& file_path, VoxelGrid<TreeOccupancyGridBasicData>& voxel_grid,
                       float voxel_size) {
  std::ifstream input(file_path, std::ios::in | std::ios::binary);
  if (!input.is_open()) {
    std::cout << "Error: could not open file " << file_path << std::endl;
    return false;
  }

  // Read header
  std::string line;
  input >> line;  // #binvox
  if (line.compare("#binvox") != 0) {
    std::cout << "Error: first line reads [" << line << "] instead of [#binvox]" << std::endl;
    return false;
  }
  int version;
  input >> version;
#ifndef NDEBUG
  std::cout << "reading binvox version " << version << std::endl;
#endif
  int depth, height, width;
  depth = -1;
  bool done = false;
  while (input.good() && !done) {
    input >> line;
    if (line.compare("data") == 0)
      done = true;
    else if (line.compare("dim") == 0) {
      input >> depth >> height >> width;
    } else {
#ifndef NDEBUG
      std::cout << "  unrecognized keyword [" << line << "], skipping" << std::endl;
#endif
      char c;
      do {  // skip until end of line
        c = input.get();
      } while (input.good() && (c != '\n'));
    }
  }

  if (!done) {
    std::cout << "  error reading header" << std::endl;
    return false;
  }
  if (depth == -1) {
    std::cout << "  missing dimensions in header" << std::endl;
    return false;
  }

  // Initialize the voxel grid based on the dimensions read
  glm::vec3 min_bound(0, 0, 0);  // Assuming starting from origin
  glm::ivec3 resolution(width, height, depth);
  voxel_grid.Initialize(voxel_size, resolution, min_bound,
                        {});  // Assuming voxelSize is globally defined or passed as an argument

  // Read voxel data
  unsigned char value;
  unsigned char count;
  int index = 0;
  int end_index = 0;
  int nr_voxels = 0;

  input.unsetf(std::ios::skipws);  // need to read every byte now (!)
  input >> value;                  // read the linefeed char
  glm::vec3 low_sum = glm::ivec3(0.0f);
  size_t low_sum_count = 0;
  while (end_index < width * height * depth && input.good()) {
    input >> value >> count;

    if (input.good()) {
      end_index = index + count;
      if (end_index > (width * height * depth))
        return false;

      for (int i = index; i < end_index; i++) {
        // Convert 1D index to 3D coordinates
        const int x = (i / width) % height;
        const int y = i % width;
        const int z = i / (width * height);

        if (value) {
          voxel_grid.Ref(glm::ivec3(x, y, z)).occupied = true;
          nr_voxels++;

          if (y < (height * 0.2f)) {
            low_sum += voxel_grid.GetPosition(glm::ivec3(x, y, z));
            low_sum_count++;
          }
        }
      }

      index = end_index;
    }
  }
  low_sum /= low_sum_count;
  voxel_grid.ShiftMinBound(-glm::vec3(low_sum.x, 0, low_sum.z));

  input.close();
#ifndef NDEBUG
  std::cout << "  read " << nr_voxels << " voxels" << std::endl;
#endif
  return true;
}

void Tree::ExportObj(const std::filesystem::path& path, const TreeMeshGeneratorSettings& mesh_generator_settings) {
  if (path.extension() == ".obj") {
    try {
      std::ofstream of;
      of.open(path.string(), std::ofstream::out | std::ofstream::trunc);
      if (of.is_open()) {
        std::string start = "#Forest OBJ exporter, by Bosheng Li";
        start += "\n";
        of.write(start.c_str(), start.size());
        of.flush();
        unsigned start_index = 1;
        if (mesh_generator_settings.enable_branch) {
          if (const auto branch_mesh = GenerateBranchMesh(mesh_generator_settings)) {
            auto& vertices = branch_mesh->UnsafeGetVertices();
            auto& triangles = branch_mesh->UnsafeGetTriangles();
            if (!vertices.empty() && !triangles.empty()) {
              std::string header =
                  "#Vertices: " + std::to_string(vertices.size()) + ", tris: " + std::to_string(triangles.size());
              header += "\n";
              of.write(header.c_str(), header.size());
              of.flush();
              std::stringstream data;
              data << "o branch " + std::to_string(0) + "\n";
#pragma region Data collection
              for (auto i = 0; i < vertices.size(); i++) {
                auto& vertex_position = vertices.at(i).position;
                auto& color = vertices.at(i).color;
                data << "v " + std::to_string(vertex_position.x) + " " + std::to_string(vertex_position.y) + " " +
                            std::to_string(vertex_position.z) + " " + std::to_string(color.x) + " " +
                            std::to_string(color.y) + " " + std::to_string(color.z) + "\n";
              }
              for (const auto& vertex : vertices) {
                data << "vt " + std::to_string(vertex.tex_coord.x) + " " + std::to_string(vertex.tex_coord.y) + "\n";
              }
              // data += "s off\n";
              data << "# List of indices for faces vertices, with (x, y, z).\n";
              for (auto i = 0; i < triangles.size(); i++) {
                const auto triangle = triangles[i];
                const auto f1 = triangle.x + start_index;
                const auto f2 = triangle.y + start_index;
                const auto f3 = triangle.z + start_index;
                data << "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" + std::to_string(f1) + " " +
                            std::to_string(f2) + "/" + std::to_string(f2) + "/" + std::to_string(f2) + " " +
                            std::to_string(f3) + "/" + std::to_string(f3) + "/" + std::to_string(f3) + "\n";
              }
#pragma endregion
              const auto result = data.str();
              of.write(result.c_str(), result.size());
              of.flush();
              start_index += vertices.size();
            }
          }
        }
        if (mesh_generator_settings.enable_foliage) {
          if (const auto foliage_mesh = GenerateFoliageMesh(mesh_generator_settings)) {
            auto& vertices = foliage_mesh->UnsafeGetVertices();
            auto& triangles = foliage_mesh->UnsafeGetTriangles();
            if (!vertices.empty() && !triangles.empty()) {
              std::string header =
                  "#Vertices: " + std::to_string(vertices.size()) + ", tris: " + std::to_string(triangles.size());
              header += "\n";
              of.write(header.c_str(), header.size());
              of.flush();
              std::stringstream data;
              data << "o foliage " + std::to_string(0) + "\n";
#pragma region Data collection
              for (auto i = 0; i < vertices.size(); i++) {
                auto& vertex_position = vertices.at(i).position;
                auto& color = vertices.at(i).color;
                data << "v " + std::to_string(vertex_position.x) + " " + std::to_string(vertex_position.y) + " " +
                            std::to_string(vertex_position.z) + " " + std::to_string(color.x) + " " +
                            std::to_string(color.y) + " " + std::to_string(color.z) + "\n";
              }
              for (const auto& vertex : vertices) {
                data << "vt " + std::to_string(vertex.tex_coord.x) + " " + std::to_string(vertex.tex_coord.y) + "\n";
              }
              // data += "s off\n";
              data << "# List of indices for faces vertices, with (x, y, z).\n";
              for (auto i = 0; i < triangles.size(); i++) {
                const auto triangle = triangles[i];
                const auto f1 = triangle.x + start_index;
                const auto f2 = triangle.y + start_index;
                const auto f3 = triangle.z + start_index;
                data << "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" + std::to_string(f1) + " " +
                            std::to_string(f2) + "/" + std::to_string(f2) + "/" + std::to_string(f2) + " " +
                            std::to_string(f3) + "/" + std::to_string(f3) + "/" + std::to_string(f3) + "\n";
              }
#pragma endregion
              const auto result = data.str();
              of.write(result.c_str(), result.size());
              of.flush();
              start_index += vertices.size();
            }
          }
        }
        of.close();
      }
    } catch (std::exception e) {
      EVOENGINE_ERROR("Export failed: " + std::string(e.what()));
    }
  }
}

void Tree::ExportStrandModelObj(const std::filesystem::path& path,
                                const StrandModelMeshGeneratorSettings& mesh_generator_settings) {
  if (path.extension() == ".obj") {
    if (strand_model.strand_model_skeleton.RefRawNodes().size() !=
        shoot_model.PeekShootSkeleton().PeekRawNodes().size()) {
      BuildStrandModel();
    }
    try {
      std::ofstream of;
      of.open(path.string(), std::ofstream::out | std::ofstream::trunc);
      if (of.is_open()) {
        std::string start = "#Forest OBJ exporter, by Bosheng Li";
        start += "\n";
        of.write(start.c_str(), start.size());
        of.flush();
        unsigned vertex_start_index = 1;
        unsigned tex_coords_start_index = 1;
        if (mesh_generator_settings.enable_branch) {
          std::vector<Vertex> vertices;
          std::vector<glm::vec2> tex_coords;
          std::vector<std::pair<unsigned int, unsigned int>> indices;
          StrandModelMeshGenerator::Generate(strand_model, vertices, tex_coords, indices, mesh_generator_settings);
          if (!vertices.empty() && !indices.empty()) {
            std::string header =
                "#Vertices: " + std::to_string(vertices.size()) + ", tris: " + std::to_string(indices.size());
            header += "\n";
            of.write(header.c_str(), header.size());
            of.flush();
            std::stringstream data;
            data << "o tree " + std::to_string(0) + "\n";
#pragma region Data collection
            for (auto& vertex : vertices) {
              auto& vertex_position = vertex.position;
              auto& color = vertex.color;
              data << "v " + std::to_string(vertex_position.x) + " " + std::to_string(vertex_position.y) + " " +
                          std::to_string(vertex_position.z) + " " + std::to_string(color.x) + " " +
                          std::to_string(color.y) + " " + std::to_string(color.z) + "\n";
            }
            for (const auto& tex_coord : tex_coords) {
              data << "vt " + std::to_string(tex_coord.x) + " " + std::to_string(tex_coord.y) + "\n";
            }
            data << "# List of indices for faces vertices, with (x, y, z).\n";
            for (auto i = 0; i < indices.size() / 3; i++) {
              const auto f1 = indices.at(i * 3).first + vertex_start_index;
              const auto f2 = indices.at(i * 3 + 1).first + vertex_start_index;
              const auto f3 = indices.at(i * 3 + 2).first + vertex_start_index;
              const auto t1 = indices.at(i * 3).second + tex_coords_start_index;
              const auto t2 = indices.at(i * 3 + 1).second + tex_coords_start_index;
              const auto t3 = indices.at(i * 3 + 2).second + tex_coords_start_index;
              data << "f " + std::to_string(f1) + "/" + std::to_string(t1) + "/" + std::to_string(f1) + " " +
                          std::to_string(f2) + "/" + std::to_string(t2) + "/" + std::to_string(f2) + " " +
                          std::to_string(f3) + "/" + std::to_string(t3) + "/" + std::to_string(f3) + "\n";
            }
#pragma endregion
            const auto result = data.str();
            of.write(result.c_str(), result.size());
            of.flush();
            vertex_start_index += vertices.size();
            tex_coords_start_index += tex_coords.size();
          }
        }
        if (mesh_generator_settings.enable_foliage) {
          if (const auto foliage_mesh = GenerateStrandModelFoliageMesh(mesh_generator_settings)) {
            const auto& vertices = foliage_mesh->UnsafeGetVertices();
            const auto& triangles = foliage_mesh->UnsafeGetTriangles();
            if (!vertices.empty() && !triangles.empty()) {
              std::string header =
                  "#Vertices: " + std::to_string(vertices.size()) + ", tris: " + std::to_string(triangles.size());
              header += "\n";
              of.write(header.c_str(), header.size());
              of.flush();
              std::stringstream data;
              data << "o tree " + std::to_string(0) + "\n";
#pragma region Data collection
              for (auto& vertex : vertices) {
                auto& vertex_position = vertex.position;
                auto& color = vertex.color;
                data << "v " + std::to_string(vertex_position.x) + " " + std::to_string(vertex_position.y) + " " +
                            std::to_string(vertex_position.z) + " " + std::to_string(color.x) + " " +
                            std::to_string(color.y) + " " + std::to_string(color.z) + "\n";
              }
              for (const auto& vertex : vertices) {
                data << "vt " + std::to_string(vertex.tex_coord.x) + " " + std::to_string(vertex.tex_coord.y) + "\n";
              }
              // data += "s off\n";
              data << "# List of indices for faces vertices, with (x, y, z).\n";
              for (auto triangle : triangles) {
                const auto f1 = triangle.x + vertex_start_index;
                const auto f2 = triangle.y + vertex_start_index;
                const auto f3 = triangle.z + vertex_start_index;
                const auto t1 = triangle.x + tex_coords_start_index;
                const auto t2 = triangle.y + tex_coords_start_index;
                const auto t3 = triangle.z + tex_coords_start_index;
                data << "f " + std::to_string(f1) + "/" + std::to_string(t1) + "/" + std::to_string(f1) + " " +
                            std::to_string(f2) + "/" + std::to_string(t2) + "/" + std::to_string(f2) + " " +
                            std::to_string(f3) + "/" + std::to_string(t3) + "/" + std::to_string(f3) + "\n";
              }
#pragma endregion
              const auto result = data.str();
              of.write(result.c_str(), result.size());
              of.flush();
              vertex_start_index += vertices.size();
              tex_coords_start_index += vertices.size();
            }
          }
        }
        of.close();
      }
    } catch (std::exception e) {
      EVOENGINE_ERROR("Export failed: " + std::string(e.what()));
    }
  }
}

void Tree::ExportTrunkObj(const std::filesystem::path& path, const TreeMeshGeneratorSettings& mesh_generator_settings) {
  if (path.extension() == ".obj") {
    std::ofstream of;
    of.open(path.string(), std::ofstream::out | std::ofstream::trunc);
    if (of.is_open()) {
      std::string start = "#Forest OBJ exporter, by Bosheng Li";
      start += "\n";
      of.write(start.c_str(), start.size());
      of.flush();
      unsigned start_index = 1;
      if (mesh_generator_settings.enable_branch) {
        std::shared_ptr<Mesh> trunk_mesh = AssetManager::CreateTemporaryAsset<Mesh>();
        GenerateTrunkMeshes(trunk_mesh, mesh_generator_settings);
        if (trunk_mesh) {
          auto& vertices = trunk_mesh->UnsafeGetVertices();
          auto& triangles = trunk_mesh->UnsafeGetTriangles();
          if (!vertices.empty() && !triangles.empty()) {
            std::string header =
                "#Vertices: " + std::to_string(vertices.size()) + ", tris: " + std::to_string(triangles.size());
            header += "\n";
            of.write(header.c_str(), header.size());
            of.flush();
            std::stringstream data;
            data << std::string("o trunk") + "\n";
#pragma region Data collection
            for (auto i = 0; i < vertices.size(); i++) {
              auto& vertex_position = vertices.at(i).position;
              auto& color = vertices.at(i).color;
              data << "v " + std::to_string(vertex_position.x) + " " + std::to_string(vertex_position.y) + " " +
                          std::to_string(vertex_position.z) + " " + std::to_string(color.x) + " " +
                          std::to_string(color.y) + " " + std::to_string(color.z) + "\n";
            }
            for (const auto& vertex : vertices) {
              data << "vt " + std::to_string(vertex.tex_coord.x) + " " + std::to_string(vertex.tex_coord.y) + "\n";
            }
            // data += "s off\n";
            data << "# List of indices for faces vertices, with (x, y, z).\n";
            for (auto i = 0; i < triangles.size(); i++) {
              const auto triangle = triangles[i];
              const auto f1 = triangle.x + start_index;
              const auto f2 = triangle.y + start_index;
              const auto f3 = triangle.z + start_index;
              data << "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" + std::to_string(f1) + " " +
                          std::to_string(f2) + "/" + std::to_string(f2) + "/" + std::to_string(f2) + " " +
                          std::to_string(f3) + "/" + std::to_string(f3) + "/" + std::to_string(f3) + "\n";
            }
#pragma endregion
            const auto result = data.str();
            of.write(result.c_str(), result.size());
            of.flush();
            start_index += vertices.size();
          }
        }
      }
      of.close();
    }
  }
}

void Tree::GenerateTreeParts(const TreeMeshGeneratorSettings& mesh_generator_settings,
                             std::vector<TreePartData>& tree_parts) {
  auto td = tree_descriptor_ref.Get<TreeDescriptor>();
  if (!td) {
    EVOENGINE_WARNING("TreeDescriptor missing!");
    td = AssetManager::CreateTemporaryAsset<TreeDescriptor>();
    td->foliage_descriptor = AssetManager::CreateTemporaryAsset<BasicFoliageDescriptor>();
  }
  auto fd = td->foliage_descriptor.Get<BasicFoliageDescriptor>();
  if (!fd)
    fd = AssetManager::CreateTemporaryAsset<BasicFoliageDescriptor>();

  const auto& skeleton = shoot_model.RefShootSkeleton();
  const auto& sorted_internode_list = skeleton.PeekSortedNodeList();

  std::unordered_map<SkeletonNodeHandle, TreePartInfo> tree_part_infos{};
  int next_line_index = 0;
  for (int internode_handle : sorted_internode_list) {
    const auto& internode = skeleton.PeekNode(internode_handle);
    const auto& internode_info = internode.info;

    auto parent_internode_handle = internode.GetParentHandle();
    const auto flow_handle = internode.GetFlowHandle();
    const auto& flow = skeleton.PeekFlow(flow_handle);
    const auto& chain_handles = flow.PeekNodeHandles();
    const bool has_multiple_children = flow.PeekChildHandles().size() > 1;
    bool only_child = true;
    const auto parent_flow_handle = flow.GetParentHandle();
    float distance_to_chain_start = 0;
    float distance_to_chain_end = 0;
    const auto chain_size = chain_handles.size();
    for (int i = 0; i < chain_size; i++) {
      if (chain_handles[i] == internode_handle)
        break;
      distance_to_chain_start += skeleton.PeekNode(chain_handles[i]).info.length;
    }
    distance_to_chain_end = flow.info.flow_length - distance_to_chain_start - internode.info.length;
    float compare_radius = internode.info.thickness;
    if (parent_flow_handle != -1) {
      const auto& parent_flow = skeleton.PeekFlow(parent_flow_handle);
      only_child = parent_flow.PeekChildHandles().size() <= 1;
      compare_radius = parent_flow.info.end_thickness;
    }
    int tree_part_type = 0;
    if (has_multiple_children &&
        distance_to_chain_end <= mesh_generator_settings.tree_part_base_distance * compare_radius) {
      tree_part_type = 1;
    } else if (!only_child &&
               distance_to_chain_start <= mesh_generator_settings.tree_part_end_distance * compare_radius) {
      tree_part_type = 2;
    }
    int current_tree_part_index = -1;
    int current_line_index = -1;
    if (tree_part_type == 0) {
      // IShape
      // If root or parent is Y Shape or length exceeds limit, create a new IShape from this node.
      bool restart_i_shape =
          parent_internode_handle == -1 || tree_part_infos[parent_internode_handle].tree_part_type != 0;
      if (!restart_i_shape) {
        if (const auto& parent_junction_info = tree_part_infos[parent_internode_handle];
            parent_junction_info.distance_to_start / internode_info.thickness >
            mesh_generator_settings.tree_part_break_ratio)
          restart_i_shape = true;
      }
      if (restart_i_shape) {
        TreePartInfo tree_part_info;
        tree_part_info.tree_part_type = 0;
        tree_part_info.tree_part_index = tree_parts.size();
        tree_part_info.line_index = next_line_index;
        tree_part_info.distance_to_start = 0.0f;
        tree_part_infos[internode_handle] = tree_part_info;
        tree_parts.emplace_back();
        auto& tree_part = tree_parts.back();
        tree_part.is_junction = false;
        tree_part.num_of_leaves = 0;
        current_tree_part_index = tree_part.tree_part_index = tree_part_info.tree_part_index;

        current_line_index = next_line_index;
        next_line_index++;
      } else {
        auto& current_tree_part_info = tree_part_infos[internode_handle];
        current_tree_part_info = tree_part_infos[parent_internode_handle];
        current_tree_part_info.distance_to_start += internode_info.length;
        current_tree_part_info.tree_part_type = 0;
        current_tree_part_index = current_tree_part_info.tree_part_index;

        current_line_index = current_tree_part_info.line_index;
      }
    } else if (tree_part_type == 1) {
      // Base of Y Shape
      if (parent_internode_handle == -1 || tree_part_infos[parent_internode_handle].tree_part_type != 1 ||
          tree_part_infos[parent_internode_handle].base_flow_handle != flow_handle) {
        TreePartInfo tree_part_info;
        tree_part_info.tree_part_type = 1;
        tree_part_info.tree_part_index = tree_parts.size();
        tree_part_info.line_index = next_line_index;
        tree_part_info.distance_to_start = 0.0f;
        tree_part_info.base_flow_handle = flow_handle;
        tree_part_infos[internode_handle] = tree_part_info;
        tree_parts.emplace_back();
        auto& tree_part = tree_parts.back();
        tree_part.is_junction = true;
        tree_part.num_of_leaves = 0;
        current_tree_part_index = tree_part.tree_part_index = tree_part_info.tree_part_index;

        current_line_index = next_line_index;
        next_line_index++;
      } else {
        auto& current_tree_part_info = tree_part_infos[internode_handle];
        current_tree_part_info = tree_part_infos[parent_internode_handle];
        current_tree_part_info.tree_part_type = 1;
        current_tree_part_index = current_tree_part_info.tree_part_index;

        current_line_index = current_tree_part_info.line_index;
      }
    } else if (tree_part_type == 2) {
      // Branch of Y Shape
      if (parent_internode_handle == -1 || tree_part_infos[parent_internode_handle].tree_part_type == 0 ||
          tree_part_infos[parent_internode_handle].base_flow_handle != parent_flow_handle) {
        EVOENGINE_ERROR("Error!");
      }

      auto& current_tree_part_info = tree_part_infos[internode_handle];
      current_tree_part_info = tree_part_infos[parent_internode_handle];
      if (current_tree_part_info.tree_part_type != 2) {
        current_tree_part_info.line_index = next_line_index;
        next_line_index++;
      }
      current_tree_part_info.tree_part_type = 2;
      current_tree_part_index = current_tree_part_info.tree_part_index;

      current_line_index = current_tree_part_info.line_index;
    }
    auto& tree_part = tree_parts[current_tree_part_index];
    tree_part.node_handles.emplace_back(internode_handle);
    tree_part.is_end.emplace_back(true);
    tree_part.line_index.emplace_back(current_line_index);
    for (int i = 0; i < tree_part.node_handles.size(); i++) {
      if (tree_part.node_handles[i] == parent_internode_handle) {
        tree_part.is_end[i] = false;
        break;
      }
    }
  }
  for (int internode_handle : sorted_internode_list) {
    const auto& internode = skeleton.PeekNode(internode_handle);
    const auto& internode_info = internode.info;
    std::vector<glm::mat4> leaf_matrices;
    const auto tree_dim = skeleton.max - skeleton.min;
    fd->GenerateFoliageMatrices(leaf_matrices, internode_info, glm::length(tree_dim));

    auto& current_tree_part_info = tree_part_infos[internode_handle];
    auto& tree_part = tree_parts[current_tree_part_info.tree_part_index];
    tree_part.num_of_leaves += leaf_matrices.size();
  }
  for (auto& tree_part : tree_parts) {
    const auto& start_internode = skeleton.PeekNode(tree_part.node_handles.front());
    if (tree_part.is_junction) {
      const auto& base_node = skeleton.PeekNode(tree_part.node_handles.front());
      const auto& flow = skeleton.PeekFlow(base_node.GetFlowHandle());
      const auto& chain_handles = flow.PeekNodeHandles();
      const auto center_internode_handle = chain_handles.back();
      const auto& center_internode = skeleton.PeekNode(center_internode_handle);
      tree_part.base_line.start_position = start_internode.info.global_position;
      tree_part.base_line.start_radius = start_internode.info.thickness;
      tree_part.base_line.end_position = center_internode.info.GetGlobalEndPosition();
      tree_part.base_line.end_radius = center_internode.info.thickness;

      tree_part.base_line.start_direction = start_internode.info.GetGlobalDirection();
      tree_part.base_line.end_direction = center_internode.info.GetGlobalDirection();

      tree_part.base_line.line_index = tree_part.line_index.front();
      for (int i = 1; i < tree_part.node_handles.size(); i++) {
        if (tree_part.is_end[i]) {
          const auto& end_internode = skeleton.PeekNode(tree_part.node_handles[i]);
          tree_part.children_lines.emplace_back();
          auto& new_line = tree_part.children_lines.back();
          new_line.start_position = center_internode.info.GetGlobalEndPosition();
          new_line.start_radius = center_internode.info.thickness;
          new_line.end_position = end_internode.info.GetGlobalEndPosition();
          new_line.end_radius = end_internode.info.thickness;

          new_line.start_direction = center_internode.info.GetGlobalDirection();
          new_line.end_direction = end_internode.info.GetGlobalDirection();

          new_line.line_index = tree_part.line_index[i];
        }
      }
    } else {
      const auto& end_internode = skeleton.PeekNode(tree_part.node_handles.back());
      tree_part.base_line.start_position = start_internode.info.global_position;
      tree_part.base_line.start_radius = start_internode.info.thickness;
      tree_part.base_line.end_position = end_internode.info.GetGlobalEndPosition();
      tree_part.base_line.end_radius = end_internode.info.thickness;

      tree_part.base_line.start_direction = start_internode.info.GetGlobalDirection();
      tree_part.base_line.end_direction = end_internode.info.GetGlobalDirection();

      tree_part.base_line.line_index = tree_part.line_index.front();
    }
  }
}

void Tree::ExportFlowGraph(YAML::Emitter& out) const {
  out << YAML::Key << "Flows" << YAML::Value << YAML::BeginSeq;
  const auto& skeleton = shoot_model.PeekShootSkeleton();
  for (const auto& flow_handle : skeleton.PeekSortedFlowList()) {
    const auto& flow = skeleton.PeekFlow(flow_handle);
    out << YAML::BeginMap;
    out << YAML::Key << "I" << YAML::Value << flow_handle;
    out << YAML::Key << "PI" << YAML::Value << flow.GetParentHandle();
    out << YAML::Key << "SP" << YAML::Value << flow.info.global_start_position;
    out << YAML::Key << "SD" << YAML::Value << flow.info.global_start_rotation * glm::vec3(0, 0, -1);
    out << YAML::Key << "ST" << YAML::Value << flow.info.start_thickness;

    out << YAML::Key << "EP" << YAML::Value << flow.info.global_end_position;
    out << YAML::Key << "ED" << YAML::Value << flow.info.global_end_rotation * glm::vec3(0, 0, -1);
    out << YAML::Key << "ET" << YAML::Value << flow.info.end_thickness;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
}

void Tree::ExportFlowGraph(const std::filesystem::path& path) const {
  try {
    std::filesystem::path yaml_path = path;
    YAML::Emitter out;
    out << YAML::BeginMap;
    ExportFlowGraph(out);
    out << YAML::EndMap;
    std::ofstream output_file(yaml_path.string());
    output_file << out.c_str();
    output_file.flush();
  } catch (const std::exception& e) {
    EVOENGINE_ERROR(std::string("Failed to save: ") + e.what());
  }
}
void Tree::ExportNodeGraph(YAML::Emitter& out) const {
  out << YAML::Key << "Nodes" << YAML::Value << YAML::BeginSeq;
  const auto& skeleton = shoot_model.PeekShootSkeleton();
  for (const auto& node_handle : skeleton.PeekSortedNodeList()) {
    const auto& node = skeleton.PeekNode(node_handle);
    out << YAML::BeginMap;
    out << YAML::Key << "I" << YAML::Value << node_handle;
    out << YAML::Key << "PI" << YAML::Value << node.GetParentHandle();
    out << YAML::Key << "FI" << YAML::Value << node.GetFlowHandle();
    out << YAML::Key << "SP" << YAML::Value << node.info.global_position;
    out << YAML::Key << "EP" << YAML::Value << node.info.GetGlobalEndPosition();
    out << YAML::Key << "D" << YAML::Value << node.info.GetGlobalDirection();
    out << YAML::Key << "T" << YAML::Value << node.info.thickness;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
}
void Tree::ExportNodeGraph(const std::filesystem::path& path) const {
  try {
    std::filesystem::path yaml_path = path;
    YAML::Emitter out;
    out << YAML::BeginMap;
    ExportNodeGraph(out);
    out << YAML::EndMap;
    std::ofstream output_file(yaml_path.string());
    output_file << out.c_str();
    output_file.flush();
  } catch (const std::exception& e) {
    EVOENGINE_ERROR(std::string("Failed to save: ") + e.what());
  }
}

void Tree::ExportTreeParts(const TreeMeshGeneratorSettings& mesh_generator_settings, YAML::Emitter& out) {
  out << YAML::Key << "Tree" << YAML::Value << YAML::BeginMap;
  {
    std::vector<TreePartData> tree_parts{};
    GenerateTreeParts(mesh_generator_settings, tree_parts);
    std::unordered_set<int> line_index_check{};
    out << YAML::Key << "TreeParts" << YAML::Value << YAML::BeginSeq;
    for (const auto& tree_part : tree_parts) {
      out << YAML::BeginMap;
      out << YAML::Key << "J" << YAML::Value << (tree_part.is_junction ? 1 : 0);
      out << YAML::Key << "I" << YAML::Value << tree_part.tree_part_index + 1;
      out << YAML::Key << "LI" << YAML::Value << tree_part.base_line.line_index + 1;

      out << YAML::Key << "F" << YAML::Value << tree_part.num_of_leaves;
      /*
      if (lineIndexCheck.find(treePart.base_line.line_index) != lineIndexCheck.end())
      {
              EVOENGINE_ERROR("Duplicate!");
      }
      lineIndexCheck.emplace(treePart.base_line.line_index);*/
      out << YAML::Key << "BSP" << YAML::Value << tree_part.base_line.start_position;
      out << YAML::Key << "BEP" << YAML::Value << tree_part.base_line.end_position;
      out << YAML::Key << "BSR" << YAML::Value << tree_part.base_line.start_radius;
      out << YAML::Key << "BER" << YAML::Value << tree_part.base_line.end_radius;
      out << YAML::Key << "BSD" << YAML::Value << tree_part.base_line.start_direction;
      out << YAML::Key << "BED" << YAML::Value << tree_part.base_line.end_direction;

      out << YAML::Key << "C" << YAML::Value << YAML::BeginSeq;
      if (tree_part.children_lines.size() > 3) {
        EVOENGINE_ERROR("Too many child!");
      }
      for (const auto& child_line : tree_part.children_lines) {
        out << YAML::BeginMap;
        out << YAML::Key << "LI" << YAML::Value << child_line.line_index + 1;
        /*
        if (lineIndexCheck.find(childLine.line_index) != lineIndexCheck.end())
        {
                EVOENGINE_ERROR("Duplicate!");
        }
        lineIndexCheck.emplace(childLine.line_index);*/
        out << YAML::Key << "SP" << YAML::Value << child_line.start_position;
        out << YAML::Key << "EP" << YAML::Value << child_line.end_position;
        out << YAML::Key << "SR" << YAML::Value << child_line.start_radius;
        out << YAML::Key << "ER" << YAML::Value << child_line.end_radius;
        out << YAML::Key << "SD" << YAML::Value << child_line.start_direction;
        out << YAML::Key << "ED" << YAML::Value << child_line.end_direction;
        out << YAML::EndMap;
      }
      out << YAML::EndSeq;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
  out << YAML::EndMap;
}

void Tree::ExportTreeParts(const TreeMeshGeneratorSettings& mesh_generator_settings,
                           const std::filesystem::path& path) {
  try {
    auto directory = path;
    directory.remove_filename();
    std::filesystem::create_directories(directory);
    YAML::Emitter out;
    ExportTreeParts(mesh_generator_settings, out);
    std::ofstream output_file(path.string());
    output_file << out.c_str();
    output_file.flush();
  } catch (const std::exception& e) {
    EVOENGINE_ERROR("Failed to save!");
  }
}

bool Tree::ExportIoTree(const std::filesystem::path& path) const {
  treeio::ArrayTree tree{};
  using namespace treeio;
  const auto& shoot_skeleton = shoot_model.PeekShootSkeleton();
  const auto& sorted_internode_list = shoot_skeleton.PeekSortedNodeList();
  if (sorted_internode_list.empty())
    return false;
  const auto& root_node = shoot_skeleton.PeekNode(0);
  TreeNodeData root_node_data;
  // rootNodeData.direction = rootNode.info.regulated_global_rotation * glm::vec3(0, 0, -1);
  root_node_data.thickness = root_node.info.thickness;
  root_node_data.pos = root_node.info.global_position;

  auto root_id = tree.addRoot(root_node_data);
  std::unordered_map<SkeletonNodeHandle, size_t> node_map;
  node_map[0] = root_id;
  for (const auto& node_handle : sorted_internode_list) {
    if (node_handle == 0)
      continue;
    const auto& node = shoot_skeleton.PeekNode(node_handle);
    TreeNodeData node_data;
    // nodeData.direction = node.info.regulated_global_rotation * glm::vec3(0, 0, -1);
    node_data.thickness = node.info.thickness;
    node_data.pos = node.info.global_position;

    auto current_id = tree.addNodeChild(node_map[node.GetParentHandle()], node_data);
    node_map[node_handle] = current_id;
  }
  return tree.saveTree(path.string());
}
