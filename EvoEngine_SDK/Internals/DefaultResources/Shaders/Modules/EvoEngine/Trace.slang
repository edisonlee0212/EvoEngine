

//=========================================================================================
//| Scene data definition                                                                 |
//=========================================================================================
[[vk::binding(0, 0)]]
StructuredBuffer<float4> scene_graph_data;

[[vk::binding(1, 0)]]
StructuredBuffer<float4> scene_geometry_data;

[[vk::binding(2, 0)]]
StructuredBuffer<float4> scene_info;

//=========================================================================================
//| Trace data definition                                                                 |
//=========================================================================================
public struct RayDescriptor {
  public float3 origin;
  public float3 direction;
  public float t_min;
  public float t_max;
};

public struct HitInfo {
  public float3 hit;
  public float3 barycentric;
  public bool back_face;
  public float3 normal;
  public float hit_dist;
  public uint triangle_index;

  public uint instance_index;
  public uint node_index;
  public uint mesh_index;
  public uint local_triangle_index;
};

//=========================================================================================
//| Main API                                                                              |
//=========================================================================================
public HitInfo Trace(in RayDescriptor ray_descriptor, in bool cull_back_face, in bool cull_front_face,
                     out bool has_hit);

//=========================================================================================
//| Implementations                                                                       |
//=========================================================================================
bool RayAabb(float3 r_o, float3 r_inv_d, float3 aabb_min, float3 aabb_max) {
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
  float3 aabb_min;
  float3 aabb_max;
  uint alternate_node_index;
  uint begin_next_level_element_index;
  uint end_next_level_element_index;
};

void GetBvhNode(in float4 buffer_0, in float4 buffer_1, in float4 buffer_2, out BvhNode bvh_node) {
  bvh_node.aabb_min = buffer_0.xyz;
  bvh_node.aabb_max = buffer_1.xyz;
  bvh_node.alternate_node_index = asuint(buffer_2.x);
  bvh_node.begin_next_level_element_index = asuint(buffer_2.y);
  bvh_node.end_next_level_element_index = asuint(buffer_2.z);
}

float3 TransformPosition(float4x4 transform, float3 position) {
  float4 p = float4(position, 1.0);
  return mul(p, transform).xyz;
}

float3 TransformDirection(float4x4 transform, float3 direction) {
  float4 d = float4(direction, 0.0);
  return mul(d, transform).xyz;
}

float3 Barycentric(float3 p, float3 a, float3 b, float3 c) {
  float3 v0 = b - a;
  float3 v1 = c - a;
  float3 v2 = p - a;
  float d00 = dot(v0, v0);
  float d01 = dot(v0, v1);
  float d11 = dot(v1, v1);
  float d20 = dot(v2, v0);
  float d21 = dot(v2, v1);
  float denominator = d00 * d11 - d01 * d01;
  float y = (d11 * d20 - d01 * d21) / denominator;
  float z = (d00 * d21 - d01 * d20) / denominator;
  return float3(1.0f - y - z, y, z);
}

public HitInfo Trace(in RayDescriptor ray_descriptor, in bool cull_back_face, in bool cull_front_face,
                     out bool has_hit) {
  HitInfo hit_info;
  hit_info.hit_dist = ray_descriptor.t_max;
  has_hit = false;
  float3 scene_space_ray_direction = normalize(ray_descriptor.direction);
  float3 scene_space_ray_origin = ray_descriptor.origin;
  float3 scene_space_inv_ray_direction =
      float3(1.0 / scene_space_ray_direction.x, 1.0 / scene_space_ray_direction.y, 1.0 / scene_space_ray_direction.z);

  uint scene_level_bvh_nodes_size = asuint(scene_info[0].x);
  uint scene_level_bvh_nodes_offset = asuint(scene_info[0].y);
  uint node_indices_offset = asuint(scene_info[0].z);
  uint node_infos_offset = asuint(scene_info[0].w);
  uint node_level_bvh_nodes_offset = asuint(scene_info[1].x);
  uint mesh_indices_offset = asuint(scene_info[1].y);

  uint mesh_mappings_offset = asuint(scene_info[1].z);
  uint mesh_level_bvh_nodes_offset = asuint(scene_info[1].w);

  uint triangle_indices_offset = asuint(scene_info[2].x);
  uint triangles_offset = asuint(scene_info[2].y);
  uint vertices_offset = asuint(scene_info[2].z);
  uint local_triangle_indices_offset = asuint(scene_info[2].w);


  uint node_group_index = 0;
  while (node_group_index < scene_level_bvh_nodes_size) {
    BvhNode node_group;
    GetBvhNode(scene_graph_data[scene_level_bvh_nodes_offset + node_group_index * 3],
               scene_graph_data[scene_level_bvh_nodes_offset + node_group_index * 3 + 1],
               scene_graph_data[scene_level_bvh_nodes_offset + node_group_index * 3 + 2], node_group);
    if (!RayAabb(scene_space_ray_origin, scene_space_inv_ray_direction, node_group.aabb_min, node_group.aabb_max)) {
      node_group_index = node_group.alternate_node_index;
      continue;
    }
    for (uint test_node_index = node_group.begin_next_level_element_index;
         test_node_index < node_group.end_next_level_element_index; test_node_index += 1) {
      //=============DO NOT TOUCH UNLESS YOU UNDERSTAND WHY==========================
      uint node_index = asuint(scene_graph_data[node_indices_offset + uint(test_node_index / 4)]
                                                        [test_node_index % 4]);  // >>>--> This doesn't work on dx.
      // float4 node_index_v = scene_graph_data[asuint(scene_info_offsets_0.z) +test_node_index / 4];
      // float node_index_l[4] = {node_index_v.x, node_index_v.y, node_index_v.z, node_index_v.w};
      // uint node_index = asuint(node_index_l[test_node_index % 4]);
      //=============================================================================
      float4x4 node_global_transform;
      node_global_transform[0] = scene_graph_data[node_infos_offset + node_index * 9];
      node_global_transform[1] = scene_graph_data[node_infos_offset + node_index * 9 + 1];
      node_global_transform[2] = scene_graph_data[node_infos_offset + node_index * 9 + 2];
      node_global_transform[3] = scene_graph_data[node_infos_offset + node_index * 9 + 3];
      float4x4 node_inverse_global_transform;
      node_inverse_global_transform[0] = scene_graph_data[node_infos_offset + node_index * 9 + 4];
      node_inverse_global_transform[1] = scene_graph_data[node_infos_offset + node_index * 9 + 5];
      node_inverse_global_transform[2] = scene_graph_data[node_infos_offset + node_index * 9 + 6];
      node_inverse_global_transform[3] = scene_graph_data[node_infos_offset + node_index * 9 + 7];
      uint node_level_bvh_node_offset = asuint(scene_graph_data[node_infos_offset + node_index * 9 + 8].x);
      uint node_level_bvh_node_size = asuint(scene_graph_data[node_infos_offset + node_index * 9 + 8].y);
      uint mesh_index_offset = asuint(scene_graph_data[node_infos_offset + node_index * 9 + 8].z);
      uint instance_index = asuint(scene_graph_data[node_infos_offset + node_index * 9 + 8].w);
      float3 node_space_ray_origin = TransformPosition(node_inverse_global_transform, scene_space_ray_origin);
      float3 node_space_ray_direction = TransformDirection(node_inverse_global_transform, scene_space_ray_direction);
      float3 node_space_inv_ray_direction =
          float3(1.0 / node_space_ray_direction.x, 1.0 / node_space_ray_direction.y, 1.0 / node_space_ray_direction.z);
      uint mesh_group_index = 0;
      while (mesh_group_index < node_level_bvh_node_size) {
        BvhNode mesh_group;
        GetBvhNode(scene_graph_data[node_level_bvh_nodes_offset + (mesh_group_index + node_level_bvh_node_offset) * 3],
            scene_graph_data[node_level_bvh_nodes_offset + (mesh_group_index + node_level_bvh_node_offset) * 3 + 1],
            scene_graph_data[node_level_bvh_nodes_offset + (mesh_group_index + node_level_bvh_node_offset) * 3 + 2],
            mesh_group);

        if (!RayAabb(node_space_ray_origin, node_space_inv_ray_direction, mesh_group.aabb_min, mesh_group.aabb_max)) {
          mesh_group_index = mesh_group.alternate_node_index;
          continue;
        }
        for (uint test_mesh_index = mesh_group.begin_next_level_element_index + mesh_index_offset;
             test_mesh_index < mesh_group.end_next_level_element_index + mesh_index_offset; test_mesh_index += 1) {
          //=============DO NOT TOUCH UNLESS YOU UNDERSTAND WHY==========================
          uint mesh_index = asuint(scene_graph_data[mesh_indices_offset + test_mesh_index / 4]
                                                            [test_mesh_index % 4]);  //>>>--> This doesn't work on dx.
          // float4 mesh_index_v = scene_graph_data[asuint(scene_info_offsets_1.y) + test_mesh_index / 4];
          // float mesh_index_l[4] = {mesh_index_v.x, mesh_index_v.y, mesh_index_v.z, mesh_index_v.w};
          // uint mesh_index = asuint(mesh_index_l[test_mesh_index % 4]);
          //=============================================================================
          uint triangle_group_index = 0;
          uint mesh_level_bvh_node_offset = asuint(scene_geometry_data[mesh_mappings_offset + mesh_index].x);
          uint mesh_level_bvh_node_size = asuint(scene_geometry_data[mesh_mappings_offset + mesh_index].y);
          uint triangle_index_offset = asuint(scene_geometry_data[mesh_mappings_offset + mesh_index].z);
          while (triangle_group_index < mesh_level_bvh_node_size) {
            BvhNode triangle_group;
            GetBvhNode(scene_geometry_data[mesh_level_bvh_nodes_offset +
                                           (triangle_group_index + mesh_level_bvh_node_offset) * 3],
                       scene_geometry_data[mesh_level_bvh_nodes_offset +
                                           (triangle_group_index + mesh_level_bvh_node_offset) * 3 + 1],
                       scene_geometry_data[mesh_level_bvh_nodes_offset +
                                           (triangle_group_index + mesh_level_bvh_node_offset) * 3 + 2],
                       triangle_group);
            if (!RayAabb(node_space_ray_origin, node_space_inv_ray_direction, triangle_group.aabb_min,
                         triangle_group.aabb_max)) {
              triangle_group_index = triangle_group.alternate_node_index;
              continue;
            }
            for (uint test_triangle_index = triangle_group.begin_next_level_element_index + triangle_index_offset;
                 test_triangle_index < triangle_group.end_next_level_element_index + triangle_index_offset;
                 test_triangle_index += 1) {
              //=============DO NOT TOUCH UNLESS YOU UNDERSTAND WHY==========================
              uint triangle_index =
                  asuint(scene_geometry_data[triangle_indices_offset + test_triangle_index / 4]
                                                     [test_triangle_index % 4]);  //>>>--> This doesn't work on dx.
              // float4 triangle_index_v = scene_geometry_data[triangle_indices_offset + test_triangle_index /
              // 4]; float triangle_index_l[4] = {triangle_index_v.x, triangle_index_v.y, triangle_index_v.z,
              // triangle_index_v.w}; uint triangle_index = asuint(triangle_index_l[test_triangle_index % 4]);
              //=============================================================================
              uint t_x = asuint(scene_geometry_data[triangles_offset + triangle_index].x);
              uint t_y = asuint(scene_geometry_data[triangles_offset + triangle_index].y);
              uint t_z = asuint(scene_geometry_data[triangles_offset + triangle_index].z);
              float3 p0 = scene_geometry_data[vertices_offset + t_x * 7].xyz;
              float3 p1 = scene_geometry_data[vertices_offset + t_y * 7].xyz;
              float3 p2 = scene_geometry_data[vertices_offset + t_z * 7].xyz;
              if (p0.x == p1.x && p0.y == p1.y && p0.z == p1.z && p1.x == p2.x && p1.y == p2.y && p1.z == p2.z)
                continue;
              float3 node_space_triangle_normal = normalize(cross(p1 - p0, p2 - p0));
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
                float3 node_space_hit = node_space_ray_origin + node_space_ray_direction * node_space_hit_distance;
                float3 scene_space_hit = TransformPosition(node_global_transform, node_space_hit);
                float scene_hit_distance = distance(scene_space_ray_origin, scene_space_hit);
                if (scene_hit_distance >= ray_descriptor.t_min && scene_hit_distance <= hit_info.hit_dist) {
                  float3 barycentric = Barycentric(node_space_hit, p0, p1, p2);
                  if (barycentric.x >= 0.0 && barycentric.x <= 1.0 && barycentric.y >= 0.0 && barycentric.y <= 1.0 &&
                      barycentric.z >= 0.0 && barycentric.z <= 1.0) {
                    hit_info.hit = scene_space_hit;
                    hit_info.normal = TransformDirection(node_global_transform, node_space_triangle_normal);
                    hit_info.hit_dist = scene_hit_distance;
                    hit_info.barycentric = barycentric;
                    hit_info.back_face = normal_test > 0.0 ? true : false;
                    hit_info.triangle_index = triangle_index;
                    //=============DO NOT TOUCH UNLESS YOU UNDERSTAND WHY==========================
                    hit_info.local_triangle_index = asuint(
                        scene_geometry_data[local_triangle_indices_offset + (test_triangle_index / 4)]
                                           [test_triangle_index % 4]);  //>>>--> This doesn't work on dx.
                    // float4 local_triangle_index_v = scene_geometry_data[local_triangle_indices_offset +
                    // test_triangle_index / 4]; float local_triangle_index_l[4] = {local_triangle_index_v.x,
                    // local_triangle_index_v.y, local_triangle_index_v.z, local_triangle_index_v.w};
                    // hit_info.triangle_index = asuint(local_triangle_index_l[test_triangle_index % 4]);
                    //=============================================================================
                    hit_info.mesh_index = mesh_index;
                    hit_info.node_index = node_index;
                    hit_info.instance_index = instance_index;
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
