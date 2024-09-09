
//=========================================================================================
//| Scene data definition                                                                 |
//=========================================================================================
layout(std140, binding = SCENE_LEVEL_BVH_BLOCK_BINDING) readonly buffer SCENE_LEVEL_BVH_BLOCK {
  vec4 scene_level_bvh_nodes[];
};

layout(std140, binding = NODE_INDICES_BLOCK_BINDING) readonly buffer NODE_INDICES_BLOCK {
  vec4 node_indices[];
};

layout(std140, binding = NODE_INFO_LIST_BLOCK_BINDING) readonly buffer NODE_INFO_LIST_BLOCK {
  vec4 node_info_list[];
};

layout(std140, binding = NODE_LEVEL_BVH_NODES_BLOCK_BINDING) readonly buffer NODE_LEVEL_BVH_NODES_BLOCK {
  vec4 node_level_bvh_nodes[];
};

layout(std140, binding = MESH_INDICES_BLOCK_BINDING) readonly buffer MESH_INDICES_BLOCK {
  vec4 mesh_indices[];
};

layout(std140, binding = MESH_INFO_LIST_BLOCK_BINDING) readonly buffer MESH_INFO_LIST_BLOCK {
  vec4 mesh_info_list[];
};

layout(std140, binding = MESH_LEVEL_BVH_NODES_BLOCK_BINDING) readonly buffer MESH_LEVEL_BVH_NODES_BLOCK {
  vec4 mesh_level_bvh_nodes[];
};

layout(std140, binding = TRIANGLE_INDICES_BLOCK_BINDING) readonly buffer TRIANGLE_INDICES_BLOCK {
  vec4 triangle_indices[];
};

layout(std140, binding = LOCAL_TRIANGLE_INDICES_BLOCK_BINDING) readonly buffer LOCAL_TRIANGLE_INDICES_BLOCK {
  vec4 local_triangle_indices[];
};

layout(std140, binding = SCENE_TRIANGLES_BLOCK_BINDING) readonly buffer SCENE_TRIANGLES_BLOCK {
  vec4 scene_triangles[];
};

layout(std140, binding = SCENE_VERTEX_POSITIONS_BLOCK_BINDING) readonly buffer SCENE_VERTEX_POSITIONS_BLOCK {
  vec4 scene_vertex_positions[];
};

layout(std140, binding = SCENE_INFO_BLOCK_BINDING) readonly buffer SCENE_INFO_BLOCK {
  vec4 scene_info;
};

//=========================================================================================
//| Trace data definition                                                                 |
//=========================================================================================
struct RayDescriptor {
  vec3 origin;
  vec3 direction;
  float t_min;
  float t_max;
};

struct HitInfo {
  vec3 hit;
  vec3 barycentric;
  bool back_face;
  vec3 normal;
  float hit_dist;
  uint triangle_index;
  uint mesh_index;
  uint node_index;
};

//=========================================================================================
//| Main API                                                                              |
//=========================================================================================
HitInfo Trace(in RayDescriptor ray_descriptor, in bool cull_back_face, in bool cull_front_face, out bool has_hit);

//=========================================================================================
//| Implementations                                                                       |
//=========================================================================================
bool RayAabb(vec3 r_o, vec3 r_inv_d, vec3 aabb_min, vec3 aabb_max) {
  float tx1 = (aabb_min.x - r_o.x) * r_inv_d.x;
  float tx2 = (aabb_max.x - r_o.x) * r_inv_d.x;

  float t_min = min(tx1, tx2);
  float t_max = max(tx1, tx2);

  float ty1 = (aabb_min.y - r_o.y) * r_inv_d.y;
  float ty2 = (aabb_max.y - r_o.y) * r_inv_d.y;

  t_min = max(t_min, min(ty1, ty2));
  t_max = min(t_max, max(ty1, ty2));

  float tz1 = (aabb_min.z - r_o.z) * r_inv_d.z;
  float tz2 = (aabb_max.z - r_o.z) * r_inv_d.z;

  t_min = max(t_min, min(tz1, tz2));
  t_max = min(t_max, max(tz1, tz2));

  return t_max >= t_min;
}

struct BvhNode {
  vec3 aabb_min;
  vec3 aabb_max;
  uint alternate_node_index;
  uint begin_next_level_element_index;
  uint end_next_level_element_index;
};

void GetBvhNode(in vec4 buffer_0, in vec4 buffer_1, in vec4 buffer_2, out BvhNode bvh_node) {
  bvh_node.aabb_min = buffer_0.xyz;
  bvh_node.aabb_max = buffer_1.xyz;
  bvh_node.alternate_node_index = floatBitsToUint(buffer_2.x);
  bvh_node.begin_next_level_element_index = floatBitsToUint(buffer_2.y);
  bvh_node.end_next_level_element_index = floatBitsToUint(buffer_2.z);
}

vec3 TransformPosition(mat4 transform, vec3 position) {
  vec4 p = vec4(position, 1.0);
  return (transform * p).xyz;
}

vec3 TransformDirection(mat4 transform, vec3 direction) {
  vec4 d = vec4(direction, 0.0);
  return (transform * d).xyz;
}

vec3 Barycentric(vec3 p, vec3 a, vec3 b, vec3 c) {
  vec3 v0 = b - a;
  vec3 v1 = c - a;
  vec3 v2 = p - a;
  float d00 = dot(v0, v0);
  float d01 = dot(v0, v1);
  float d11 = dot(v1, v1);
  float d20 = dot(v2, v0);
  float d21 = dot(v2, v1);
  float denominator = d00 * d11 - d01 * d01;
  float y = (d11 * d20 - d01 * d21) / denominator;
  float z = (d00 * d21 - d01 * d20) / denominator;
  return vec3(1.0f - y - z, y, z);
}

HitInfo Trace(in RayDescriptor ray_descriptor, in bool cull_back_face, in bool cull_front_face, out bool has_hit) {
  HitInfo hit_info;
  hit_info.hit_dist = ray_descriptor.t_max;
  has_hit = false;
  vec3 scene_space_ray_direction = normalize(ray_descriptor.direction);
  vec3 scene_space_ray_origin = ray_descriptor.origin;
  vec3 scene_space_inv_ray_direction =
      vec3(1.0 / scene_space_ray_direction.x, 1.0 / scene_space_ray_direction.y, 1.0 / scene_space_ray_direction.z);
  uint node_group_index = 0;
  while (node_group_index < floatBitsToUint(scene_info.x)) {
    BvhNode node_group;
    GetBvhNode(scene_level_bvh_nodes[node_group_index * 3], scene_level_bvh_nodes[node_group_index * 3 + 1],
               scene_level_bvh_nodes[node_group_index * 3 + 2], node_group);
    if (!RayAabb(scene_space_ray_origin, scene_space_inv_ray_direction, node_group.aabb_min, node_group.aabb_max)) {
      node_group_index = node_group.alternate_node_index;
      continue;
    }
    for (uint test_node_index = node_group.begin_next_level_element_index;
         test_node_index < node_group.end_next_level_element_index; test_node_index += 1) {
      //=============DO NOT TOUCH UNLESS YOU UNDERSTAND WHY==========================
      uint node_index =
          floatBitsToUint(node_indices[test_node_index / 4][test_node_index % 4]);  // >>>--> This doesn't work on dx.
      // vec4 node_index_v = node_indices[test_node_index / 4];
      // float node_index_l[4] = {node_index_v.x, node_index_v.y, node_index_v.z, node_index_v.w};
      // uint node_index = floatBitsToUint(node_index_l[test_node_index % 4]);
      //=============================================================================
      mat4 node_global_transform;
      node_global_transform[0] = node_info_list[node_index * 9];
      node_global_transform[1] = node_info_list[node_index * 9 + 1];
      node_global_transform[2] = node_info_list[node_index * 9 + 2];
      node_global_transform[3] = node_info_list[node_index * 9 + 3];
      mat4 node_inverse_global_transform;
      node_inverse_global_transform[0] = node_info_list[node_index * 9 + 4];
      node_inverse_global_transform[1] = node_info_list[node_index * 9 + 5];
      node_inverse_global_transform[2] = node_info_list[node_index * 9 + 6];
      node_inverse_global_transform[3] = node_info_list[node_index * 9 + 7];
      uint node_level_bvh_node_offset = floatBitsToUint(node_info_list[node_index * 9 + 8].x);
      uint node_level_bvh_node_size = floatBitsToUint(node_info_list[node_index * 9 + 8].y);
      uint mesh_indices_offset = floatBitsToUint(node_info_list[node_index * 9 + 8].z);
      uint mesh_indices_size = floatBitsToUint(node_info_list[node_index * 9 + 8].w);
      vec3 node_space_ray_origin = TransformPosition(node_inverse_global_transform, scene_space_ray_origin);
      vec3 node_space_ray_direction = TransformDirection(node_inverse_global_transform, scene_space_ray_direction);
      vec3 node_space_inv_ray_direction =
          vec3(1.0 / node_space_ray_direction.x, 1.0 / node_space_ray_direction.y, 1.0 / node_space_ray_direction.z);
      uint mesh_group_index = 0;
      while (mesh_group_index < node_level_bvh_node_size) {
        BvhNode mesh_group;
        GetBvhNode(node_level_bvh_nodes[(mesh_group_index + node_level_bvh_node_offset) * 3],
                   node_level_bvh_nodes[(mesh_group_index + node_level_bvh_node_offset) * 3 + 1],
                   node_level_bvh_nodes[(mesh_group_index + node_level_bvh_node_offset) * 3 + 2], mesh_group);

        if (!RayAabb(node_space_ray_origin, node_space_inv_ray_direction, mesh_group.aabb_min, mesh_group.aabb_max)) {
          mesh_group_index = mesh_group.alternate_node_index;
          continue;
        }
        for (uint test_mesh_index = mesh_group.begin_next_level_element_index + mesh_indices_offset;
             test_mesh_index < mesh_group.end_next_level_element_index + mesh_indices_offset; test_mesh_index += 1) {
          //=============DO NOT TOUCH UNLESS YOU UNDERSTAND WHY==========================
          uint mesh_index = floatBitsToUint(
              mesh_indices[test_mesh_index / 4][test_mesh_index % 4]);  //>>>--> This doesn't work on dx.
          // vec4 mesh_index_v = mesh_indices[test_mesh_index / 4];
          // float mesh_index_l[4] = {mesh_index_v.x, mesh_index_v.y, mesh_index_v.z, mesh_index_v.w};
          // uint mesh_index = floatBitsToUint(mesh_index_l[test_mesh_index % 4]);
          //=============================================================================
          uint triangle_group_index = 0;
          uint mesh_level_bvh_node_offset = floatBitsToUint(mesh_info_list[mesh_index].x);
          uint mesh_level_bvh_node_size = floatBitsToUint(mesh_info_list[mesh_index].y);
          uint triangle_indices_offset = floatBitsToUint(mesh_info_list[mesh_index].z);
          while (triangle_group_index < mesh_level_bvh_node_size) {
            BvhNode triangle_group;
            GetBvhNode(mesh_level_bvh_nodes[(triangle_group_index + mesh_level_bvh_node_offset) * 3],
                       mesh_level_bvh_nodes[(triangle_group_index + mesh_level_bvh_node_offset) * 3 + 1],
                       mesh_level_bvh_nodes[(triangle_group_index + mesh_level_bvh_node_offset) * 3 + 2],
                       triangle_group);
            if (!RayAabb(node_space_ray_origin, node_space_inv_ray_direction, triangle_group.aabb_min,
                         triangle_group.aabb_max)) {
              triangle_group_index = triangle_group.alternate_node_index;
              continue;
            }
            for (uint test_triangle_index = triangle_group.begin_next_level_element_index + triangle_indices_offset;
                 test_triangle_index < triangle_group.end_next_level_element_index + triangle_indices_offset;
                 test_triangle_index += 1) {
              //=============DO NOT TOUCH UNLESS YOU UNDERSTAND WHY==========================
              uint triangle_index =
                  floatBitsToUint(triangle_indices[test_triangle_index / 4]
                                                  [test_triangle_index % 4]);  //>>>--> This doesn't work on dx.
              // vec4 triangle_index_v = triangle_indices[test_triangle_index / 4];
              // float triangle_index_l[4] = {triangle_index_v.x, triangle_index_v.y, triangle_index_v.z,
              // triangle_index_v.w}; uint triangle_index = floatBitsToUint(triangle_index_l[test_triangle_index % 4]);
              //=============================================================================

              uint t_x = floatBitsToUint(scene_triangles[triangle_index].x);
              uint t_y = floatBitsToUint(scene_triangles[triangle_index].y);
              uint t_z = floatBitsToUint(scene_triangles[triangle_index].z);
              vec3 p0 = scene_vertex_positions[t_x].xyz;
              vec3 p1 = scene_vertex_positions[t_y].xyz;
              vec3 p2 = scene_vertex_positions[t_z].xyz;
              if (p0.x == p1.x && p0.y == p1.y && p0.z == p1.z && p1.x == p2.x && p1.y == p2.y && p1.z == p2.z)
                continue;
              vec3 node_space_triangle_normal = normalize(cross(p1 - p0, p2 - p0));
              float normal_test = dot(node_space_ray_direction, node_space_triangle_normal);
              if (cull_back_face && normal_test > 0.0)
                continue;
              if (cull_front_face && normal_test < 0.0)
                continue;
              if (normal_test == 0.0)
                continue;
              float node_space_hit_distance =
                  (dot(p0, node_space_triangle_normal) - dot(node_space_ray_origin, node_space_triangle_normal)) /
                  normal_test;
              if (node_space_hit_distance > 0) {
                vec3 node_space_hit = node_space_ray_origin + node_space_ray_direction * node_space_hit_distance;
                vec3 scene_space_hit = TransformPosition(node_global_transform, node_space_hit);
                float scene_hit_distance = distance(scene_space_ray_origin, scene_space_hit);
                if (scene_hit_distance >= ray_descriptor.t_min && scene_hit_distance <= hit_info.hit_dist) {
                  vec3 barycentric = Barycentric(node_space_hit, p0, p1, p2);
                  if (barycentric.x >= 0.0 && barycentric.x <= 1.0 && barycentric.y >= 0.0 && barycentric.y <= 1.0 &&
                      barycentric.z >= 0.0 && barycentric.z <= 1.0) {
                    hit_info.hit = scene_space_hit;
                    hit_info.normal = TransformDirection(node_global_transform, node_space_triangle_normal);
                    hit_info.hit_dist = scene_hit_distance;
                    hit_info.barycentric = barycentric;
                    hit_info.back_face = normal_test > 0.0 ? true : false;
                    //=============DO NOT TOUCH UNLESS YOU UNDERSTAND WHY==========================
                    hit_info.triangle_index = floatBitsToUint(
                        local_triangle_indices[test_triangle_index / 4]
                                              [test_triangle_index % 4]);  //>>>--> This doesn't work on dx.
                    // vec4 local_triangle_index_v = local_triangle_indices[test_triangle_index / 4];
                    // float local_triangle_index_l[4] = {local_triangle_index_v.x, local_triangle_index_v.y,
                    // local_triangle_index_v.z, local_triangle_index_v.w}; hit_info.triangle_index =
                    // floatBitsToUint(local_triangle_index_l[test_triangle_index % 4]);
                    //=============================================================================
                    hit_info.mesh_index = mesh_index;
                    hit_info.node_index = node_index;
                    has_hit = true;
                  }
                }
              }
            }
            triangle_group_index += 1;
          }
        }
        mesh_group_index += 1;
      }
    }
    node_group_index += 1;
  }
  return hit_info;
}
