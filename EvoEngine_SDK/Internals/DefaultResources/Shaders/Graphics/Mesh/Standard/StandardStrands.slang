import EvoEngine.BasicConstants;
import EvoEngine.PerFrame;
import EvoEngine.RasterVertex;
import EvoEngine.StrandMeshlet;
import EvoEngine.Strands;

static const uint WORKGROUP_SIZE = EXT_MESH_SUBGROUP_COUNT * SUBGROUP_SIZE;
static const uint STRAND_RING_MAX = 15u;
static const float PI2 = 6.28318530718f;

[outputtopology("triangle")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
[shader("mesh")]
void main(in payload StrandTaskPayload strand_task, uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex,
          OutputVertices<StandardRasterOutput, 32> vertices, OutputIndices<uint3, 30> triangles) {
  uint interval_index = group_id.x;
  uint segment_index = 0u;
  for (; segment_index + 1u < strand_task.segment_count &&
         interval_index >= strand_task.interval_offsets[segment_index + 1u];
       ++segment_index) {
  }

  uint interval_begin = strand_task.interval_offsets[segment_index];
  uint subdivision = strand_task.interval_offsets[segment_index + 1u] - interval_begin;
  float u0 = float(interval_index - interval_begin) / float(subdivision);
  float u1 = float(interval_index - interval_begin + 1u) / float(subdivision);

  StrandMeshlet meshlet = EE_STRAND_MESHLETS[strand_task.meshlet_index];
  uint8_t4 segment = meshlet.segments[segment_index];
  StrandPoint p0 = EE_STRAND_POINT_DATA_CHUNKS[meshlet.strand_point_chunk_index].strand_points[segment.x];
  StrandPoint p1 = EE_STRAND_POINT_DATA_CHUNKS[meshlet.strand_point_chunk_index].strand_points[segment.y];
  StrandPoint p2 = EE_STRAND_POINT_DATA_CHUNKS[meshlet.strand_point_chunk_index].strand_points[segment.z];
  StrandPoint p3 = EE_STRAND_POINT_DATA_CHUNKS[meshlet.strand_point_chunk_index].strand_points[segment.w];

  float3 position0, position1, tangent0, tangent1;
  float3 normal0, normal1, unused_float3;
  float4 color0, color1, unused_float4;
  float thickness0, thickness1, tex_coord0, tex_coord1, unused_float;
  EE_SPLINE_INTERPOLATION(p0.position, p1.position, p2.position, p3.position, position0, tangent0, u0);
  EE_SPLINE_INTERPOLATION(p0.position, p1.position, p2.position, p3.position, position1, tangent1, u1);
  EE_SPLINE_INTERPOLATION(p0.normal, p1.normal, p2.normal, p3.normal, normal0, unused_float3, u0);
  EE_SPLINE_INTERPOLATION(p0.normal, p1.normal, p2.normal, p3.normal, normal1, unused_float3, u1);
  EE_SPLINE_INTERPOLATION(p0.thickness, p1.thickness, p2.thickness, p3.thickness, thickness0, unused_float, u0);
  EE_SPLINE_INTERPOLATION(p0.thickness, p1.thickness, p2.thickness, p3.thickness, thickness1, unused_float, u1);
  EE_SPLINE_INTERPOLATION(p0.tex_coord, p1.tex_coord, p2.tex_coord, p3.tex_coord, tex_coord0, unused_float, u0);
  EE_SPLINE_INTERPOLATION(p0.tex_coord, p1.tex_coord, p2.tex_coord, p3.tex_coord, tex_coord1, unused_float, u1);
  EE_SPLINE_INTERPOLATION(p0.color, p1.color, p2.color, p3.color, color0, unused_float4, u0);
  EE_SPLINE_INTERPOLATION(p0.color, p1.color, p2.color, p3.color, color1, unused_float4, u1);

  float3 frame_normal0, frame_normal1, frame_bitangent0, frame_bitangent1;
  EE_STRAND_FRAME(tangent0, normal0, frame_normal0, frame_bitangent0);
  EE_STRAND_FRAME(tangent1, normal1, frame_normal1, frame_bitangent1);

  uint instance_index = strand_task.instance_index;
  int camera_index = EE_BASIC_CONSTANTS.camera_index;
  float4x4 model = EE_INSTANCES[instance_index].model;
  float3 world_position0 = mul(float4(position0, 1.0f), model).xyz;
  float3 world_position1 = mul(float4(position1, 1.0f), model).xyz;
  uint ring0 = uint(EE_STRANDS_RING_SUBDIVISION(camera_index, model, world_position0, position0, thickness0));
  uint ring1 = uint(EE_STRANDS_RING_SUBDIVISION(camera_index, model, world_position1, position1, thickness1));
  uint ring_size = min(max(ring0, ring1), STRAND_RING_MAX);
  uint vertex_count = (ring_size + 1u) * 2u;
  uint primitive_count = ring_size * 2u;
  SetMeshOutputCounts(vertex_count, primitive_count);

  float4x4 projection_view = EE_CAMERAS[camera_index].projection_view;
  float handedness = EE_TRANSFORM_HANDEDNESS(model);
  for (uint vertex = lane_id; vertex < vertex_count; vertex += WORKGROUP_SIZE) {
    bool endpoint = (vertex & 1u) != 0u;
    uint ring_vertex = vertex >> 1u;
    uint endpoint_ring = endpoint ? ring1 : ring0;
    uint ring_step = ring_vertex * endpoint_ring / ring_size;
    float angle = -PI2 * float(ring_step) / float(endpoint_ring);
    float3 position = endpoint ? position1 : position0;
    float3 tangent = endpoint ? tangent1 : tangent0;
    float3 frame_normal = endpoint ? frame_normal1 : frame_normal0;
    float3 frame_bitangent = endpoint ? frame_bitangent1 : frame_bitangent0;
    float thickness = endpoint ? thickness1 : thickness0;
    float3 radial = frame_normal * sin(angle) + frame_bitangent * cos(angle);
    float2 tex_coord = float2(float(ring_step) / float(endpoint_ring), endpoint ? tex_coord1 : tex_coord0);

    vertices[vertex] = EE_BUILD_STANDARD_RASTER_OUTPUT(
        position + radial * thickness, radial, tangent, 1.0f, tex_coord, tex_coord, float2(0.0f), float2(0.0f),
        endpoint ? color1 : color0, instance_index, model, projection_view);
    vertices[vertex].tangent_handedness = handedness;
  }

  for (uint primitive = lane_id; primitive < primitive_count; primitive += WORKGROUP_SIZE) {
    uint ring_index = primitive >> 1u;
    uint source = ring_index * 2u;
    uint target = source + 1u;
    uint next_source = (ring_index + 1u) * 2u;
    uint next_target = next_source + 1u;
    triangles[primitive] = (primitive & 1u) == 0u ? uint3(source, target, next_source)
                                                  : uint3(next_source, target, next_target);
  }
}
