#include "BasicBarkDescriptor.hpp"
#include "TransformGraph.hpp"
#include "Tree.hpp"
#include "TreeSkinnedMeshGenerator.hpp"

using namespace eco_sys_lab_plugin;

void Tree::GenerateSkeletalGraph(const SkeletalGraphSettings& skeletal_graph_settings,
                                 SkeletonNodeHandle base_node_handle, const std::shared_ptr<Mesh>& point_mesh_sample,
                                 const std::shared_ptr<Mesh>& line_mesh_sample) const {
  const auto scene = GetScene();
  const auto self = GetOwner();
  ClearSkeletalGraph();

  const auto line_entity = scene->CreateEntity("Skeletal Graph Lines");
  scene->SetParent(line_entity, self);

  const auto point_entity = scene->CreateEntity("Skeletal Graph Points");
  scene->SetParent(point_entity, self);

  bool strand_ready = false;
  if (strand_model.strand_model_skeleton.PeekSortedNodeList().size() > 1) {
    strand_ready = true;
  }

  const auto line_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  const auto line_material = AssetManager::CreateTemporaryAsset<Material>();
  const std::shared_ptr<Particles> line_particles = scene->GetOrSetPrivateComponent<Particles>(line_entity).lock();
  line_particles->mesh = line_mesh_sample;
  line_particles->material = line_material;
  line_particles->particle_info_list = line_list;
  line_material->vertex_color_only = true;
  const auto point_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  const auto point_material = AssetManager::CreateTemporaryAsset<Material>();
  const std::shared_ptr<Particles> point_particles = scene->GetOrSetPrivateComponent<Particles>(point_entity).lock();
  point_particles->mesh = point_mesh_sample;
  point_particles->material = point_material;
  point_particles->particle_info_list = point_list;
  point_material->vertex_color_only = true;

  std::vector<ParticleInfo> line_particle_infos;
  std::vector<ParticleInfo> point_particle_infos;
  const int node_size = strand_ready ? strand_model.strand_model_skeleton.PeekSortedNodeList().size()
                                     : shoot_model.PeekShootSkeleton().PeekSortedNodeList().size();
  if (strand_ready) {
    line_particle_infos.resize(node_size);
    point_particle_infos.resize(node_size);
  } else {
    line_particle_infos.resize(node_size);
    point_particle_infos.resize(node_size);
  }
  Jobs::RunParallelFor(node_size, [&](unsigned internode_index) {
    if (strand_ready) {
      const auto& sorted_internode_list = strand_model.strand_model_skeleton.PeekSortedNodeList();
      const auto internode_handle = sorted_internode_list[internode_index];
      SkeletonNodeHandle walker = internode_handle;
      bool sub_tree = false;
      const auto& skeleton = strand_model.strand_model_skeleton;
      const auto& node = skeleton.PeekNode(internode_handle);

      while (walker != -1) {
        if (walker == base_node_handle) {
          sub_tree = true;
          break;
        }
        walker = skeleton.PeekNode(walker).GetParentHandle();
      }
      const glm::vec3 position = node.info.global_position;
      auto rotation = node.info.global_rotation;
      {
        rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
        const glm::mat4 rotation_transform = glm::mat4_cast(rotation);
        line_particle_infos[internode_index].instance_matrix.value =
            glm::translate(position + (node.info.length / 2.0f) * node.info.GetGlobalDirection()) * rotation_transform *
            glm::scale(glm::vec3(skeletal_graph_settings.fixed_line_thickness * (sub_tree ? 1.25f : 1.0f),
                                 node.info.length,
                                 skeletal_graph_settings.fixed_line_thickness * (sub_tree ? 1.25f : 1.0f)));

        if (sub_tree) {
          line_particle_infos[internode_index].instance_color = skeletal_graph_settings.line_focus_color;
        } else {
          line_particle_infos[internode_index].instance_color = skeletal_graph_settings.line_color;
        }
      }
      {
        rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
        const glm::mat4 rotation_transform = glm::mat4_cast(rotation);
        float thickness_factor = node.info.thickness;
        if (skeletal_graph_settings.fixed_point_size)
          thickness_factor = skeletal_graph_settings.fixed_point_size_factor;
        auto scale = glm::vec3(skeletal_graph_settings.branch_point_size * thickness_factor);
        point_particle_infos[internode_index].instance_color = skeletal_graph_settings.branch_point_color;
        if (internode_index == 0 || node.PeekChildHandles().size() > 1) {
          scale = glm::vec3(skeletal_graph_settings.junction_point_size * thickness_factor);
          point_particle_infos[internode_index].instance_color = skeletal_graph_settings.junction_point_color;
        }
        point_particle_infos[internode_index].instance_matrix.value =
            glm::translate(position) * rotation_transform * glm::scale(scale * (sub_tree ? 1.25f : 1.0f));
        if (sub_tree) {
          point_particle_infos[internode_index].instance_color = skeletal_graph_settings.branch_focus_color;
        }
      }
    } else {
      const auto& sorted_internode_list = shoot_model.PeekShootSkeleton().PeekSortedNodeList();
      const auto internode_handle = sorted_internode_list[internode_index];
      SkeletonNodeHandle walker = internode_handle;
      bool sub_tree = false;
      const auto& skeleton = shoot_model.PeekShootSkeleton();
      const auto& node = skeleton.PeekNode(internode_handle);

      while (walker != -1) {
        if (walker == base_node_handle) {
          sub_tree = true;
          break;
        }
        walker = skeleton.PeekNode(walker).GetParentHandle();
      }
      const glm::vec3 position = node.info.global_position;
      auto rotation = node.info.global_rotation;
      {
        rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
        const glm::mat4 rotation_transform = glm::mat4_cast(rotation);
        line_particle_infos[internode_index].instance_matrix.value =
            glm::translate(position + (node.info.length / 2.0f) * node.info.GetGlobalDirection()) * rotation_transform *
            glm::scale(glm::vec3(skeletal_graph_settings.fixed_line_thickness * (sub_tree ? 1.25f : 1.0f),
                                 node.info.length,
                                 skeletal_graph_settings.fixed_line_thickness * (sub_tree ? 1.25f : 1.0f)));

        if (sub_tree) {
          line_particle_infos[internode_index].instance_color = skeletal_graph_settings.line_focus_color;
        } else {
          line_particle_infos[internode_index].instance_color = skeletal_graph_settings.line_color;
        }
      }
      {
        rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
        const glm::mat4 rotation_transform = glm::mat4_cast(rotation);
        float thickness_factor = node.info.thickness;
        if (skeletal_graph_settings.fixed_point_size)
          thickness_factor = skeletal_graph_settings.fixed_point_size_factor;
        auto scale = glm::vec3(skeletal_graph_settings.branch_point_size * thickness_factor);
        point_particle_infos[internode_index].instance_color = skeletal_graph_settings.branch_point_color;
        if (internode_index == 0 || node.PeekChildHandles().size() > 1) {
          scale = glm::vec3(skeletal_graph_settings.junction_point_size * thickness_factor);
          point_particle_infos[internode_index].instance_color = skeletal_graph_settings.junction_point_color;
        }
        point_particle_infos[internode_index].instance_matrix.value =
            glm::translate(position) * rotation_transform * glm::scale(scale * (sub_tree ? 1.25f : 1.0f));
        if (sub_tree) {
          point_particle_infos[internode_index].instance_color = skeletal_graph_settings.branch_focus_color;
        }
      }
    }
  });
  line_list->SetParticleInfos(line_particle_infos);
  point_list->SetParticleInfos(point_particle_infos);
}
void Tree::ClearSkeletalGraph() const {
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto children = scene->GetChildren(self);
  for (const auto& child : children) {
    auto name = scene->GetEntityName(child);
    if (name == "Skeletal Graph Lines") {
      scene->DeleteEntity(child);
    } else if (name == "Skeletal Graph Points") {
      scene->DeleteEntity(child);
    }
  }
}
std::shared_ptr<Strands> Tree::GenerateStrands() const {
  const auto strands_asset = AssetManager::CreateTemporaryAsset<Strands>();
  const auto& parameters = strand_model_parameters;
  std::vector<glm::uint> strands_list;
  std::vector<StrandPoint> points;
  strand_model.strand_model_skeleton.data.strand_group.BuildStrands(strands_list, points, parameters.node_max_count);
  if (!points.empty())
    strands_list.emplace_back(points.size());
  StrandPointAttributes strand_point_attributes{};
  strand_point_attributes.color = true;
  strands_asset->SetStrands(strand_point_attributes, strands_list, points);
  return strands_asset;
}

std::shared_ptr<ParticleInfoList> Tree::GenerateStrandParticles() const {
  const auto particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  std::vector<ParticleInfo> particle_infos;
  strand_model.strand_model_skeleton.data.strand_group.BuildParticles(particle_infos);
  particle_info_list->SetParticleInfos(particle_infos);
  return particle_info_list;
}

void Tree::GenerateTrunkMeshes(const std::shared_ptr<Mesh>& trunk_mesh,
                               const TreeMeshGeneratorSettings& mesh_generator_settings) {
  const auto& sorted_internode_list = shoot_model.RefShootSkeleton().PeekSortedNodeList();
  std::unordered_set<SkeletonNodeHandle> trunk_handles{};
  for (const auto& node_handle : sorted_internode_list) {
    const auto& node = shoot_model.RefShootSkeleton().PeekNode(node_handle);
    trunk_handles.insert(node_handle);
    if (node.PeekChildHandles().size() > 1)
      break;
  }
  {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    const auto td = tree_descriptor_ref.Get<TreeDescriptor>();
    std::shared_ptr<BasicBarkDescriptor> bd{};
    if (td) {
      bd = td->bark_descriptor.Get<BasicBarkDescriptor>();
    }
    CylindricalMeshGenerator<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData>::GeneratePartially(
        trunk_handles, shoot_model.PeekShootSkeleton(), vertices, indices, mesh_generator_settings,
        [&](glm::vec3& vertex_position, const glm::vec3& direction, const float x_factor, const float y_factor) {
          if (bd) {
            const float push_value = bd->GetValue(x_factor, y_factor);
            vertex_position += push_value * direction;
          }
        },
        [&](glm::vec2&, float, float) {
        });
    VertexAttributes attributes{};
    attributes.tex_coord = true;
    trunk_mesh->SetVertices(attributes, vertices, indices);
  }
}

std::shared_ptr<Mesh> Tree::GenerateBranchMesh(const TreeMeshGeneratorSettings& mesh_generator_settings) {
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;
  if (mesh_generator_settings.branch_mesh_type == 0) {
    auto td = tree_descriptor_ref.Get<TreeDescriptor>();
    if (!td) {
      EVOENGINE_WARNING("TreeDescriptor missing!");
      td = AssetManager::CreateTemporaryAsset<TreeDescriptor>();
      td->foliage_descriptor = AssetManager::CreateTemporaryAsset<BasicFoliageDescriptor>();
    }
    std::shared_ptr<BasicBarkDescriptor> bd{};
    bd = td->bark_descriptor.Get<BasicBarkDescriptor>();
    if (strand_model.strand_model_skeleton.RefRawNodes().size() == shoot_model.shoot_skeleton_.RefRawNodes().size()) {
      CylindricalMeshGenerator<StrandModelSkeletonData, StrandModelFlowData, StrandModelNodeData>::Generate(
          strand_model.strand_model_skeleton, vertices, indices, mesh_generator_settings,
          [&](glm::vec3& vertex_position, const glm::vec3& direction, const float x_factor, const float y_factor) {
            if (bd) {
              const float push_value = bd->GetValue(x_factor, y_factor);
              vertex_position += push_value * direction;
            }
          },
          [&](glm::vec2&, float, float) {
          });
    } else {
      CylindricalMeshGenerator<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData>::Generate(
          shoot_model.PeekShootSkeleton(), vertices, indices, mesh_generator_settings,
          [&](glm::vec3& vertex_position, const glm::vec3& direction, const float x_factor, const float y_factor) {
            if (bd) {
              const float push_value = bd->GetValue(x_factor, y_factor);
              vertex_position += push_value * direction;
            }
          },
          [&](glm::vec2&, float, float) {
          });
    }
  } else {
    auto td = tree_descriptor_ref.Get<TreeDescriptor>();
    if (!td) {
      EVOENGINE_WARNING("TreeDescriptor missing!");
      td = AssetManager::CreateTemporaryAsset<TreeDescriptor>();
      td->foliage_descriptor = AssetManager::CreateTemporaryAsset<BasicFoliageDescriptor>();
    }
    VoxelMeshGenerator<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData>::Generate(
        shoot_model.PeekShootSkeleton(), vertices, indices, mesh_generator_settings);
  }
  auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
  VertexAttributes attributes{};
  attributes.tex_coord = true;
  mesh->SetVertices(attributes, vertices, indices);
  return mesh;
}

std::shared_ptr<Mesh> Tree::GenerateFoliageMesh(const TreeMeshGeneratorSettings& mesh_generator_settings) {
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;

  auto quad_mesh = Resources::Primitives::quad;
  auto& quad_triangles = quad_mesh->UnsafeGetTriangles();
  size_t quad_vertices_size;
  quad_vertices_size = quad_mesh->GetVerticesAmount();
  size_t offset = 0;
  auto td = tree_descriptor_ref.Get<TreeDescriptor>();
  if (!td) {
    EVOENGINE_WARNING("TreeDescriptor missing!");
    td = AssetManager::CreateTemporaryAsset<TreeDescriptor>();
    td->foliage_descriptor = AssetManager::CreateTemporaryAsset<BasicFoliageDescriptor>();
  }
  auto fd = td->foliage_descriptor.Get<BasicFoliageDescriptor>();
  if (!fd)
    fd = AssetManager::CreateTemporaryAsset<BasicFoliageDescriptor>();
  const auto tree_dim = shoot_model.PeekShootSkeleton().max - shoot_model.PeekShootSkeleton().min;

  const auto& node_list = shoot_model.PeekShootSkeleton().PeekSortedNodeList();
  for (const auto& internode_handle : node_list) {
    const auto& internode_info = shoot_model.PeekShootSkeleton().PeekNode(internode_handle).info;
    std::vector<glm::mat4> leaf_matrices;
    fd->GenerateFoliageMatrices(leaf_matrices, internode_info, glm::length(tree_dim));
    Vertex archetype;
    for (const auto& matrix : leaf_matrices) {
      for (auto i = 0; i < quad_mesh->GetVerticesAmount(); i++) {
        archetype.position = matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].position, 1.0f);
        archetype.normal =
            glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].normal, 0.0f)));
        archetype.tangent =
            glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].tangent, 0.0f)));
        archetype.tex_coord = quad_mesh->UnsafeGetVertices()[i].tex_coord;
        archetype.color = internode_info.color;
        vertices.push_back(archetype);
      }
      for (auto triangle : quad_triangles) {
        triangle.x += offset;
        triangle.y += offset;
        triangle.z += offset;
        indices.push_back(triangle.x);
        indices.push_back(triangle.y);
        indices.push_back(triangle.z);
      }

      offset += quad_vertices_size;

      for (auto i = 0; i < quad_mesh->GetVerticesAmount(); i++) {
        archetype.position = matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].position, 1.0f);
        archetype.normal =
            glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].normal, 0.0f)));
        archetype.tangent =
            glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].tangent, 0.0f)));
        archetype.tex_coord = quad_mesh->UnsafeGetVertices()[i].tex_coord;
        vertices.push_back(archetype);
      }
      for (auto triangle : quad_triangles) {
        triangle.x += offset;
        triangle.y += offset;
        triangle.z += offset;
        indices.push_back(triangle.z);
        indices.push_back(triangle.y);
        indices.push_back(triangle.x);
      }
      offset += quad_vertices_size;
    }
  }

  auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
  VertexAttributes attributes{};
  attributes.tex_coord = true;
  mesh->SetVertices(attributes, vertices, indices);
  return mesh;
}

std::shared_ptr<ParticleInfoList> Tree::GenerateFoliageParticleInfoList(
    const TreeMeshGeneratorSettings& mesh_generator_settings) {
  auto td = tree_descriptor_ref.Get<TreeDescriptor>();
  if (!td) {
    EVOENGINE_WARNING("TreeDescriptor missing!");
    td = AssetManager::CreateTemporaryAsset<TreeDescriptor>();
    td->foliage_descriptor = AssetManager::CreateTemporaryAsset<BasicFoliageDescriptor>();
  }
  const auto ret_val = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  auto fd = td->foliage_descriptor.Get<BasicFoliageDescriptor>();
  if (!fd)
    fd = AssetManager::CreateTemporaryAsset<BasicFoliageDescriptor>();
  std::vector<ParticleInfo> particle_infos;
  const auto& node_list = shoot_model.PeekShootSkeleton().PeekSortedNodeList();
  const bool sm =
      strand_model.strand_model_skeleton.RefRawNodes().size() == shoot_model.shoot_skeleton_.RefRawNodes().size();
  const auto tree_dim = sm ? strand_model.strand_model_skeleton.max - strand_model.strand_model_skeleton.min
                           : shoot_model.PeekShootSkeleton().max - shoot_model.PeekShootSkeleton().min;

  for (const auto& internode_handle : node_list) {
    const auto& internode_info = sm ? strand_model.strand_model_skeleton.PeekNode(internode_handle).info
                                    : shoot_model.PeekShootSkeleton().PeekNode(internode_handle).info;

    std::vector<glm::mat4> leaf_matrices{};
    fd->GenerateFoliageMatrices(leaf_matrices, internode_info, glm::length(tree_dim));
    const auto start_index = particle_infos.size();
    particle_infos.resize(start_index + leaf_matrices.size());
    for (int i = 0; i < leaf_matrices.size(); i++) {
      auto& particle_info = particle_infos.at(start_index + i);
      particle_info.instance_matrix.value = leaf_matrices.at(i);
      particle_info.instance_color = internode_info.color;
    }
  }
  ret_val->SetParticleInfos(particle_infos);
  return ret_val;
}

std::shared_ptr<Mesh> Tree::GenerateStrandModelFoliageMesh(
    const StrandModelMeshGeneratorSettings& strand_model_mesh_generator_settings) {
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;

  auto quad_mesh = Resources::Primitives::quad;
  auto& quad_triangles = quad_mesh->UnsafeGetTriangles();
  auto quad_vertices_size = quad_mesh->GetVerticesAmount();
  size_t offset = 0;
  auto td = tree_descriptor_ref.Get<TreeDescriptor>();
  if (!td)
    return nullptr;
  auto fd = td->foliage_descriptor.Get<BasicFoliageDescriptor>();
  if (!fd)
    fd = AssetManager::CreateTemporaryAsset<BasicFoliageDescriptor>();
  const auto& node_list = strand_model.strand_model_skeleton.PeekSortedNodeList();
  const auto tree_dim = strand_model.strand_model_skeleton.max - strand_model.strand_model_skeleton.min;
  for (const auto& internode_handle : node_list) {
    const auto& strand_model_node = strand_model.strand_model_skeleton.PeekNode(internode_handle);
    std::vector<glm::mat4> leaf_matrices;
    fd->GenerateFoliageMatrices(leaf_matrices, strand_model_node.info, glm::length(tree_dim));
    Vertex archetype;
    for (const auto& matrix : leaf_matrices) {
      for (auto i = 0; i < quad_mesh->GetVerticesAmount(); i++) {
        archetype.position = matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].position, 1.0f);
        archetype.normal =
            glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].normal, 0.0f)));
        archetype.tangent =
            glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].tangent, 0.0f)));
        archetype.tex_coord = quad_mesh->UnsafeGetVertices()[i].tex_coord;
        archetype.color = strand_model_node.info.color;
        vertices.push_back(archetype);
      }
      for (auto triangle : quad_triangles) {
        triangle.x += offset;
        triangle.y += offset;
        triangle.z += offset;
        indices.push_back(triangle.x);
        indices.push_back(triangle.y);
        indices.push_back(triangle.z);
      }

      offset += quad_vertices_size;

      for (auto i = 0; i < quad_mesh->GetVerticesAmount(); i++) {
        archetype.position = matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].position, 1.0f);
        archetype.normal =
            glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].normal, 0.0f)));
        archetype.tangent =
            glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].tangent, 0.0f)));
        archetype.tex_coord = quad_mesh->UnsafeGetVertices()[i].tex_coord;
        vertices.push_back(archetype);
      }
      for (auto triangle : quad_triangles) {
        triangle.x += offset;
        triangle.y += offset;
        triangle.z += offset;
        indices.push_back(triangle.z);
        indices.push_back(triangle.y);
        indices.push_back(triangle.x);
      }
      offset += quad_vertices_size;
    }
  }

  auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
  VertexAttributes attributes{};
  attributes.tex_coord = true;
  mesh->SetVertices(attributes, vertices, indices);
  return mesh;
}

std::shared_ptr<Mesh> Tree::GenerateStrandModelBranchMesh(
    const StrandModelMeshGeneratorSettings& strand_model_mesh_generator_settings) const {
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;
  StrandModelMeshGenerator::Generate(strand_model, vertices, indices, strand_model_mesh_generator_settings);

  auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
  VertexAttributes attributes{};
  attributes.tex_coord = true;
  mesh->SetVertices(attributes, vertices, indices);
  return mesh;
}

void Tree::GenerateAnimatedGeometryEntities(const TreeMeshGeneratorSettings& mesh_generator_settings,
                                            const int iteration, const bool enable_physics) {
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto children = scene->GetChildren(self);
  auto td = tree_descriptor_ref.Get<TreeDescriptor>();
  const auto tree_global_transform = scene->GetDataComponent<GlobalTransform>(self);
  ClearAnimatedGeometryEntities();
  Entity rag_doll;
  rag_doll = scene->CreateEntity("Rag Doll");
  scene->SetParent(rag_doll, self);
  auto actual_iteration = iteration;
  if (actual_iteration < 0 || actual_iteration > shoot_model.CurrentIteration()) {
    actual_iteration = shoot_model.CurrentIteration();
  }
  const auto& skeleton = shoot_model.PeekShootSkeleton(actual_iteration);
  const auto& sorted_flow_list = skeleton.PeekSortedFlowList();
  std::vector<glm::mat4> offset_matrices;
  std::unordered_map<SkeletonFlowHandle, int> flow_bone_id_map;

  CylindricalSkinnedMeshGenerator<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData>::GenerateBones(
      skeleton, sorted_flow_list, offset_matrices, flow_bone_id_map);

  std::vector<std::string> names;
  std::vector<Entity> bound_entities;
  std::vector<unsigned> bone_indices_lists;
  bone_indices_lists.resize(offset_matrices.size());
  bound_entities.resize(offset_matrices.size());
  names.resize(offset_matrices.size());
  std::unordered_map<SkeletonFlowHandle, Entity> corresponding_flow_handles;
  std::unordered_map<unsigned, SkeletonFlowHandle> corresponding_entities;
  for (const auto& [flowHandle, matrixIndex] : flow_bone_id_map) {
    names[matrixIndex] = std::to_string(flowHandle);
    bound_entities[matrixIndex] = scene->CreateEntity(names[matrixIndex]);
    const auto& flow = skeleton.PeekFlow(flowHandle);

    corresponding_flow_handles[flowHandle] = bound_entities[matrixIndex];
    corresponding_entities[bound_entities[matrixIndex].GetIndex()] = flowHandle;
    GlobalTransform global_transform;

    global_transform.value = tree_global_transform.value * (glm::translate(flow.info.global_start_position) *
                                                            glm::mat4_cast(flow.info.global_start_rotation));
    scene->SetDataComponent(bound_entities[matrixIndex], global_transform);

    bone_indices_lists[matrixIndex] = matrixIndex;
  }
  for (const auto& flow_handle : sorted_flow_list) {
    const auto& flow = skeleton.PeekFlow(flow_handle);
    if (const auto& parent_flow_handle = flow.GetParentHandle(); parent_flow_handle != -1)
      scene->SetParent(corresponding_flow_handles[flow_handle], corresponding_flow_handles[parent_flow_handle]);
    else {
      scene->SetParent(corresponding_flow_handles[flow_handle], rag_doll);
    }
  }
  if (mesh_generator_settings.enable_branch) {
    Entity branch_entity;
    branch_entity = scene->CreateEntity("Animated Branch Mesh");
    scene->SetParent(branch_entity, self);
    auto animator = scene->GetOrSetPrivateComponent<Animator>(branch_entity).lock();
    auto skinned_mesh = AssetManager::CreateTemporaryAsset<SkinnedMesh>();
    auto material = AssetManager::CreateTemporaryAsset<Material>();
    auto skinned_mesh_renderer = scene->GetOrSetPrivateComponent<SkinnedMeshRenderer>(branch_entity).lock();
    bool copied_material = false;
    if (td) {
      if (const auto bark_descriptor = td->bark_descriptor.Get<BasicBarkDescriptor>()) {
        if (const auto bark_material = bark_descriptor->bark_material_ref.Get<Material>()) {
          material->SetAlbedoTexture(bark_material->GetAlbedoTexture());
          material->SetNormalTexture(bark_material->GetNormalTexture());
          material->SetRoughnessTexture(bark_material->GetRoughnessTexture());
          material->SetMetallicTexture(bark_material->GetMetallicTexture());
          material->material_properties = bark_material->material_properties;
          copied_material = true;
        }
      }
    }
    if (!copied_material) {
      material->material_properties.albedo_color = glm::vec3(109, 79, 75) / 255.0f;
      material->material_properties.roughness = 1.0f;
      material->material_properties.metallic = 0.0f;
    }

    std::vector<SkinnedVertex> skinned_vertices;
    std::vector<unsigned int> indices;

    if (!td) {
      EVOENGINE_WARNING("TreeDescriptor missing!");
      td = AssetManager::CreateTemporaryAsset<TreeDescriptor>();
      td->foliage_descriptor = AssetManager::CreateTemporaryAsset<BasicFoliageDescriptor>();
    }
    std::shared_ptr<BasicBarkDescriptor> bark_descriptor{};
    bark_descriptor = td->bark_descriptor.Get<BasicBarkDescriptor>();
    if (strand_model.strand_model_skeleton.RefRawNodes().size() == shoot_model.shoot_skeleton_.RefRawNodes().size()) {
      CylindricalSkinnedMeshGenerator<StrandModelSkeletonData, StrandModelFlowData, StrandModelNodeData>::Generate(
          strand_model.strand_model_skeleton, skinned_vertices, indices, offset_matrices, mesh_generator_settings,
          [&](glm::vec3& vertex_position, const glm::vec3& direction, const float x_factor, const float y_factor) {
            if (bark_descriptor) {
              const float push_value = bark_descriptor->GetValue(x_factor, y_factor);
              vertex_position += push_value * direction;
            }
          },
          [&](glm::vec2&, float, float) {
          });
    } else {
      CylindricalSkinnedMeshGenerator<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData>::Generate(
          skeleton, skinned_vertices, indices, offset_matrices, mesh_generator_settings,
          [&](glm::vec3& vertex_position, const glm::vec3& direction, const float x_factor, const float y_factor) {
            if (bark_descriptor) {
              const float push_value = bark_descriptor->GetValue(x_factor, y_factor);
              vertex_position += push_value * direction;
            }
          },
          [&](glm::vec2&, float, float) {
          });
    }

    skinned_mesh->bone_animator_indices = bone_indices_lists;
    SkinnedVertexAttributes attributes{};
    attributes.tex_coord = true;
    skinned_mesh->SetVertices(attributes, skinned_vertices, indices);
    skinned_mesh_renderer->animator = animator;
    skinned_mesh_renderer->skinned_mesh = skinned_mesh;
    skinned_mesh_renderer->material = material;

    animator->Setup(names, offset_matrices);
    skinned_mesh_renderer->SetRagDoll(true);
    skinned_mesh_renderer->SetRagDollBoundEntities(bound_entities, false);
  }

  if (mesh_generator_settings.enable_foliage) {
    const auto foliage_entity = scene->CreateEntity("Animated Foliage Mesh");
    scene->SetParent(foliage_entity, self);
    auto animator = scene->GetOrSetPrivateComponent<Animator>(foliage_entity).lock();

    auto skinned_mesh = AssetManager::CreateTemporaryAsset<SkinnedMesh>();
    auto skinned_mesh_renderer = scene->GetOrSetPrivateComponent<SkinnedMeshRenderer>(foliage_entity).lock();
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    bool copied_material = false;
    if (td) {
      if (const auto foliage_descriptor = td->foliage_descriptor.Get<BasicFoliageDescriptor>()) {
        if (const auto leaf_material = foliage_descriptor->leaf_material_ref.Get<Material>()) {
          material->SetAlbedoTexture(leaf_material->GetAlbedoTexture());
          material->SetNormalTexture(leaf_material->GetNormalTexture());
          material->SetRoughnessTexture(leaf_material->GetRoughnessTexture());
          material->SetMetallicTexture(leaf_material->GetMetallicTexture());
          material->material_properties = leaf_material->material_properties;
          copied_material = true;
        }
      }
    }
    if (!copied_material) {
      material->material_properties.albedo_color = glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f);
      material->material_properties.roughness = 1.0f;
      material->material_properties.metallic = 0.0f;
    }
    std::vector<SkinnedVertex> skinned_vertices;
    std::vector<unsigned> indices;
    {
      auto quad_mesh = Resources::Primitives::quad;
      auto& quad_triangles = quad_mesh->UnsafeGetTriangles();
      auto quad_vertices_size = quad_mesh->GetVerticesAmount();
      size_t offset = 0;
      if (!td) {
        EVOENGINE_WARNING("TreeDescriptor missing!");
        td = AssetManager::CreateTemporaryAsset<TreeDescriptor>();
        td->foliage_descriptor = AssetManager::CreateTemporaryAsset<BasicFoliageDescriptor>();
      }
      auto fd = td->foliage_descriptor.Get<BasicFoliageDescriptor>();
      if (!fd)
        fd = AssetManager::CreateTemporaryAsset<BasicFoliageDescriptor>();
      const auto tree_dim = skeleton.max - skeleton.min;

      const auto& node_list = skeleton.PeekSortedNodeList();
      for (const auto& internode_handle : node_list) {
        const auto& node = skeleton.PeekNode(internode_handle);
        const auto flow_handle = node.GetFlowHandle();
        const auto& flow = skeleton.PeekFlow(flow_handle);
        const auto& internode_info = node.info;
        std::vector<glm::mat4> leaf_matrices;
        fd->GenerateFoliageMatrices(leaf_matrices, internode_info, glm::length(tree_dim));
        SkinnedVertex archetype;
        archetype.bond_id = glm::ivec4(flow_bone_id_map[flow_handle], flow_bone_id_map[flow_handle], -1, -1);
        archetype.bond_id2 = glm::ivec4(-1);
        archetype.weight = glm::vec4(.5f, .5f, .0f, .0f);
        archetype.weight2 = glm::vec4(0.f);

        for (const auto& matrix : leaf_matrices) {
          for (auto i = 0; i < quad_mesh->GetVerticesAmount(); i++) {
            archetype.position = matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].position, 1.0f);
            archetype.normal =
                glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].normal, 0.0f)));
            archetype.tangent =
                glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].tangent, 0.0f)));
            archetype.tex_coord = quad_mesh->UnsafeGetVertices()[i].tex_coord;
            archetype.color = internode_info.color;
            skinned_vertices.push_back(archetype);
          }
          for (auto triangle : quad_triangles) {
            triangle.x += offset;
            triangle.y += offset;
            triangle.z += offset;
            indices.push_back(triangle.x);
            indices.push_back(triangle.y);
            indices.push_back(triangle.z);
          }

          offset += quad_vertices_size;

          for (auto i = 0; i < quad_mesh->GetVerticesAmount(); i++) {
            archetype.position = matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].position, 1.0f);
            archetype.normal =
                glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].normal, 0.0f)));
            archetype.tangent =
                glm::normalize(glm::vec3(matrix * glm::vec4(quad_mesh->UnsafeGetVertices()[i].tangent, 0.0f)));
            archetype.tex_coord = quad_mesh->UnsafeGetVertices()[i].tex_coord;
            skinned_vertices.push_back(archetype);
          }
          for (auto triangle : quad_triangles) {
            triangle.x += offset;
            triangle.y += offset;
            triangle.z += offset;
            indices.push_back(triangle.z);
            indices.push_back(triangle.y);
            indices.push_back(triangle.x);
          }
          offset += quad_vertices_size;
        }
      }
    }

    skinned_mesh->bone_animator_indices = bone_indices_lists;
    SkinnedVertexAttributes attributes{};
    attributes.tex_coord = true;
    skinned_mesh->SetVertices(attributes, skinned_vertices, indices);
    skinned_mesh_renderer->animator = animator;
    skinned_mesh_renderer->skinned_mesh = skinned_mesh;
    skinned_mesh_renderer->material = material;

    animator->Setup(names, offset_matrices);
    skinned_mesh_renderer->SetRagDoll(true);
    skinned_mesh_renderer->SetRagDollBoundEntities(bound_entities, false);
  }

  if (mesh_generator_settings.enable_fruit) {
    const auto fruit_entity = scene->CreateEntity("Animated Fruit Mesh");
    scene->SetParent(fruit_entity, self);
  }
}

void Tree::ClearAnimatedGeometryEntities() const {
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto children = scene->GetChildren(self);
  for (const auto& child : children) {
    auto name = scene->GetEntityName(child);
    if (name == "Rag Doll") {
      scene->DeleteEntity(child);
    } else if (name == "Animated Branch Mesh") {
      scene->DeleteEntity(child);
    } else if (name == "Animated Root Mesh") {
      scene->DeleteEntity(child);
    } else if (name == "Animated Foliage Mesh") {
      scene->DeleteEntity(child);
    } else if (name == "Animated Fruit Mesh") {
      scene->DeleteEntity(child);
    }
  }
}

void Tree::ClearGeometryEntities() const {
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto children = scene->GetChildren(self);
  for (const auto& child : children) {
    auto name = scene->GetEntityName(child);
    if (name == "Branch Mesh") {
      scene->DeleteEntity(child);
    } else if (name == "Root Mesh") {
      scene->DeleteEntity(child);
    } else if (name == "Foliage Mesh") {
      scene->DeleteEntity(child);
    } else if (name == "Fruit Mesh") {
      scene->DeleteEntity(child);
    }
  }
}

void Tree::GenerateGeometryEntities(const TreeMeshGeneratorSettings& mesh_generator_settings, int iteration) {
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto children = scene->GetChildren(self);
  const auto tree_descriptor = tree_descriptor_ref.Get<TreeDescriptor>();
  ClearGeometryEntities();
  if (auto actual_iteration = iteration; actual_iteration < 0 || actual_iteration > shoot_model.CurrentIteration()) {
    actual_iteration = shoot_model.CurrentIteration();
  }
  if (mesh_generator_settings.enable_branch) {
    const Entity branch_entity = scene->CreateEntity("Branch Mesh");
    scene->SetParent(branch_entity, self);

    const auto mesh = GenerateBranchMesh(mesh_generator_settings);
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(branch_entity).lock();
    bool copied_material = false;
    if (tree_descriptor) {
      if (const auto bark_descriptor = tree_descriptor->bark_descriptor.Get<BasicBarkDescriptor>()) {
        if (const auto bark_material = bark_descriptor->bark_material_ref.Get<Material>()) {
          material->SetAlbedoTexture(bark_material->GetAlbedoTexture());
          material->SetNormalTexture(bark_material->GetNormalTexture());
          material->SetRoughnessTexture(bark_material->GetRoughnessTexture());
          material->SetMetallicTexture(bark_material->GetMetallicTexture());
          material->material_properties = bark_material->material_properties;
          copied_material = true;
        }
      }
    }
    if (!copied_material) {
      material->material_properties.albedo_color = glm::vec3(109, 79, 75) / 255.0f;
      material->material_properties.roughness = 1.0f;
      material->material_properties.metallic = 0.0f;
    }
    mesh_renderer->mesh = mesh;
    mesh_renderer->material = material;
  }

  if (mesh_generator_settings.enable_foliage) {
    const auto foliage_entity = scene->CreateEntity("Foliage Mesh");
    scene->SetParent(foliage_entity, self);
    if (mesh_generator_settings.foliage_instancing) {
      const auto mesh = Resources::Primitives::quad;
      const auto particle_info_list = GenerateFoliageParticleInfoList(mesh_generator_settings);
      const auto material = AssetManager::CreateTemporaryAsset<Material>();
      bool copied_material = false;
      if (tree_descriptor) {
        if (const auto foliage_descriptor = tree_descriptor->foliage_descriptor.Get<BasicFoliageDescriptor>()) {
          if (const auto leaf_material = foliage_descriptor->leaf_material_ref.Get<Material>()) {
            material->SetAlbedoTexture(leaf_material->GetAlbedoTexture());
            material->SetNormalTexture(leaf_material->GetNormalTexture());
            material->SetRoughnessTexture(leaf_material->GetRoughnessTexture());
            material->SetMetallicTexture(leaf_material->GetMetallicTexture());
            material->material_properties = leaf_material->material_properties;
            copied_material = true;
          }
        }
      }
      if (!copied_material) {
        material->material_properties.albedo_color = glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f);
        material->material_properties.roughness = 1.0f;
        material->material_properties.metallic = 0.0f;
      }
      const auto particles = scene->GetOrSetPrivateComponent<Particles>(foliage_entity).lock();
      particles->mesh = mesh;
      particles->material = material;
      particles->particle_info_list = particle_info_list;
    } else {
      const auto mesh = GenerateFoliageMesh(mesh_generator_settings);
      const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(foliage_entity).lock();
      const auto material = AssetManager::CreateTemporaryAsset<Material>();
      bool copied_material = false;
      if (tree_descriptor) {
        if (const auto foliage_descriptor = tree_descriptor->foliage_descriptor.Get<BasicFoliageDescriptor>()) {
          if (const auto leaf_material = foliage_descriptor->leaf_material_ref.Get<Material>()) {
            material->SetAlbedoTexture(leaf_material->GetAlbedoTexture());
            material->SetNormalTexture(leaf_material->GetNormalTexture());
            material->SetRoughnessTexture(leaf_material->GetRoughnessTexture());
            material->SetMetallicTexture(leaf_material->GetMetallicTexture());
            material->material_properties = leaf_material->material_properties;
            copied_material = true;
          }
        }
      }
      if (!copied_material) {
        material->material_properties.albedo_color = glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f);
        material->material_properties.roughness = 1.0f;
        material->material_properties.metallic = 0.0f;
      }
      mesh_renderer->mesh = mesh;
      mesh_renderer->material = material;
    }
  }
  if (mesh_generator_settings.enable_fruit) {
    const auto fruit_entity = scene->CreateEntity("Fruit Mesh");
    scene->SetParent(fruit_entity, self);
  }
}
#ifdef BILLBOARD_CLOUDS_PLUGIN
inline void TransformVertex(Vertex& v, const glm::mat4& transform) {
  v.normal = glm::normalize(transform * glm::vec4(v.normal, 0.f));
  v.tangent = glm::normalize(transform * glm::vec4(v.tangent, 0.f));
  v.position = transform * glm::vec4(v.position, 1.f);
}
void Tree::GenerateBillboardClouds(const BillboardCloud::GenerateSettings& foliage_generate_settings) {
  auto mesh_generator_settings = tree_mesh_generator_settings;
  mesh_generator_settings.foliage_instancing = false;
  GenerateGeometryEntities(mesh_generator_settings);

  const auto scene = GetScene();
  const auto owner = GetOwner();
  TransformGraph::CalculateTransformGraphForDescendants(scene, owner);
  const auto children = scene->GetChildren(owner);
  const auto owner_global_transform = scene->GetDataComponent<GlobalTransform>(owner);
  for (const auto& child : children) {
    auto name = scene->GetEntityName(child);
    if (name == "Projected Tree") {
      scene->DeleteEntity(child);
    }
  }
  const auto projected_tree = scene->CreateEntity("Projected Tree");
  scene->SetParent(projected_tree, owner);
  std::vector<BillboardCloud> billboard_clouds;

  for (const auto& child : children) {
    if (!scene->IsEntityValid(child))
      continue;
    auto name = scene->GetEntityName(child);
    const auto model_space_transform =
        glm::inverse(owner_global_transform.value) * scene->GetDataComponent<GlobalTransform>(child).value;
    if (name == "Foliage Mesh") {
      if (scene->HasPrivateComponent<MeshRenderer>(child)) {
        const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
        const auto mesh = mesh_renderer->mesh.Get<Mesh>();
        const auto material = mesh_renderer->material.Get<Material>();
        if (mesh && material) {
          billboard_clouds.emplace_back();
          auto& billboard_cloud = billboard_clouds.back();
          billboard_cloud.elements.emplace_back();
          auto& element = billboard_cloud.elements.back();
          element.vertices = mesh->UnsafeGetVertices();
          element.material = material;
          element.triangles = mesh->UnsafeGetTriangles();
          Jobs::RunParallelFor(element.vertices.size(), [&](const unsigned vertex_index) {
            TransformVertex(element.vertices.at(vertex_index), model_space_transform);
          });
          billboard_cloud.Generate(foliage_generate_settings);
        }
      }
      scene->SetEnable(child, false);
    } else if (name == "Fruit Mesh") {
    }
  }
  int cloud_index = 0;
  for (const auto& billboard_cloud : billboard_clouds) {
    const auto billboard_cloud_entity = billboard_cloud.BuildEntity(scene);
    scene->SetEntityName(billboard_cloud_entity, "Projected Cluster [" + std::to_string(cloud_index) + "]");
    scene->SetParent(billboard_cloud_entity, projected_tree);
    cloud_index++;
  }
}
#endif
void Tree::ClearStrandRenderer() const {
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto children = scene->GetChildren(self);
  for (const auto& child : children) {
    auto name = scene->GetEntityName(child);
    if (name == "Branch Strands") {
      scene->DeleteEntity(child);
    }
  }
}

void Tree::InitializeStrandParticles() {
  const auto scene = GetScene();
  const auto owner = GetOwner();

  ClearStrandParticles();
  if (strand_model.strand_model_skeleton.RefRawNodes().size() !=
      shoot_model.PeekShootSkeleton().PeekRawNodes().size()) {
    BuildStrandModel();
  }
  const auto strands_entity = scene->CreateEntity("Branch Strand Particles");
  scene->SetParent(strands_entity, owner);

  const auto renderer = scene->GetOrSetPrivateComponent<Particles>(strands_entity).lock();
  renderer->particle_info_list = GenerateStrandParticles();
  renderer->mesh = Resources::Primitives::cube;
  const auto material = AssetManager::CreateTemporaryAsset<Material>();

  renderer->material = material;
  material->vertex_color_only = true;
  material->material_properties.albedo_color = glm::vec3(0.6f, 0.3f, 0.0f);
}

void Tree::InitializeStrandParticles(const std::shared_ptr<ParticleInfoList>& particle_info_list) const {
  const auto scene = GetScene();
  const auto owner = GetOwner();

  ClearStrandParticles();

  const auto strands_entity = scene->CreateEntity("Branch Strand Particles");
  scene->SetParent(strands_entity, owner);

  const auto renderer = scene->GetOrSetPrivateComponent<Particles>(strands_entity).lock();
  renderer->particle_info_list = particle_info_list;
  renderer->mesh = Resources::Primitives::cube;
  const auto material = AssetManager::CreateTemporaryAsset<Material>();

  renderer->material = material;
  material->vertex_color_only = true;
  material->material_properties.albedo_color = glm::vec3(0.6f, 0.3f, 0.0f);
}

void Tree::ClearStrandParticles() const {
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto children = scene->GetChildren(self);
  for (const auto& child : children) {
    auto name = scene->GetEntityName(child);
    if (name == "Branch Strand Particles") {
      scene->DeleteEntity(child);
    }
  }
}

void Tree::InitializeStrandRenderer() {
  const auto scene = GetScene();
  const auto owner = GetOwner();

  ClearStrandRenderer();
  if (strand_model.strand_model_skeleton.RefRawNodes().size() !=
      shoot_model.PeekShootSkeleton().PeekRawNodes().size()) {
    BuildStrandModel();
  }
  const auto strands_entity = scene->CreateEntity("Branch Strands");
  scene->SetParent(strands_entity, owner);

  const auto renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(strands_entity).lock();
  renderer->strands = GenerateStrands();

  const auto material = AssetManager::CreateTemporaryAsset<Material>();

  renderer->material = material;
  material->vertex_color_only = true;
  material->material_properties.albedo_color = glm::vec3(0.6f, 0.3f, 0.0f);
}

void Tree::InitializeStrandRenderer(const std::shared_ptr<Strands>& strands) const {
  const auto scene = GetScene();
  const auto owner = GetOwner();

  ClearStrandRenderer();

  const auto strands_entity = scene->CreateEntity("Branch Strands");
  scene->SetParent(strands_entity, owner);

  const auto renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(strands_entity).lock();

  renderer->strands = strands;

  const auto material = AssetManager::CreateTemporaryAsset<Material>();

  renderer->material = material;
  material->vertex_color_only = true;
  material->material_properties.albedo_color = glm::vec3(0.6f, 0.3f, 0.0f);
}

void Tree::InitializeStrandModelMeshRenderer(
    const StrandModelMeshGeneratorSettings& strand_model_mesh_generator_settings) {
  ClearStrandModelMeshRenderer();
  if (strand_model.strand_model_skeleton.RefRawNodes().size() !=
      shoot_model.PeekShootSkeleton().PeekRawNodes().size()) {
    BuildStrandModel();
  }
  const float time = Times::Now();
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto td = tree_descriptor_ref.Get<TreeDescriptor>();
  if (strand_model_mesh_generator_settings.enable_branch) {
    const auto foliage_entity = scene->CreateEntity("Strand Model Branch Mesh");
    scene->SetParent(foliage_entity, self);

    const auto mesh = GenerateStrandModelBranchMesh(strand_model_mesh_generator_settings);
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(foliage_entity).lock();
    bool copied_material = false;
    if (td) {
      if (const auto bd = td->bark_descriptor.Get<BasicBarkDescriptor>()) {
        if (const auto bark_material = bd->bark_material_ref.Get<Material>()) {
          material->SetAlbedoTexture(bark_material->GetAlbedoTexture());
          material->SetNormalTexture(bark_material->GetNormalTexture());
          material->SetRoughnessTexture(bark_material->GetRoughnessTexture());
          material->SetMetallicTexture(bark_material->GetMetallicTexture());
          material->material_properties = bark_material->material_properties;
          copied_material = true;
        }
      }
    }
    if (!copied_material) {
      material->material_properties.albedo_color = glm::vec3(109, 79, 75) / 255.0f;
      material->material_properties.roughness = 1.0f;
      material->material_properties.metallic = 0.0f;
    }

    mesh_renderer->mesh = mesh;
    mesh_renderer->material = material;
  }
  if (strand_model_mesh_generator_settings.enable_foliage) {
    const Entity foliage_entity = scene->CreateEntity("Strand Model Foliage Mesh");
    scene->SetParent(foliage_entity, self);

    const auto mesh = GenerateStrandModelFoliageMesh(strand_model_mesh_generator_settings);
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    bool copied_material = false;
    if (td) {
      if (const auto fd = td->foliage_descriptor.Get<BasicFoliageDescriptor>()) {
        if (const auto leaf_material = fd->leaf_material_ref.Get<Material>()) {
          material->SetAlbedoTexture(leaf_material->GetAlbedoTexture());
          material->SetNormalTexture(leaf_material->GetNormalTexture());
          material->SetRoughnessTexture(leaf_material->GetRoughnessTexture());
          material->SetMetallicTexture(leaf_material->GetMetallicTexture());
          material->material_properties = leaf_material->material_properties;
          copied_material = true;
        }
      }
    }
    if (!copied_material) {
      material->material_properties.albedo_color = glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f);
      material->material_properties.roughness = 1.0f;
      material->material_properties.metallic = 0.0f;
    }
    const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(foliage_entity).lock();
    mesh_renderer->mesh = mesh;
    mesh_renderer->material = material;
  }
  std::string output;
  const float mesh_generation_time = Times::Now() - time;
  output += "\nMesh generation Used time: " + std::to_string(mesh_generation_time) + "\n";
  EVOENGINE_LOG(output);
}

void Tree::ClearStrandModelMeshRenderer() const {
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto children = scene->GetChildren(self);
  for (const auto& child : children) {
    auto name = scene->GetEntityName(child);
    if (name == "Strand Model Branch Mesh") {
      scene->DeleteEntity(child);
    } else if (name == "Strand Model Foliage Mesh") {
      scene->DeleteEntity(child);
    }
  }
}
