// @evoengine-dialect native
import EvoEngine.BasicConstants;
import EvoEngine.PerFrame;
import EvoEngine.StrandMeshlet;
import EvoEngine.Strands;
static const uint WORKGROUP_SIZE = EXT_MESH_SUBGROUP_COUNT * SUBGROUP_SIZE;
static const uint STRAND_SHADOW_RING_SIZE = 4u;
static const float PI2 = 6.28318530718f;
struct EEStrandShadowOutput {
  float4 position : SV_Position;
};
[outputtopology("triangle")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
[shader("mesh")]
void main(in payload StrandShadowTaskPayload strand_shadow_task, uint3 group_id : SV_GroupID,
          uint lane_id : SV_GroupIndex, OutputVertices<EEStrandShadowOutput, 10> vertices,
          OutputIndices<uint3, 8> triangles) {
  StrandMeshlet meshlet = EE_STRAND_MESHLETS[strand_shadow_task.meshlet_index];
  uint8_t4 segment = meshlet.segments[group_id.x];
  StrandPoint p0 = EE_STRAND_POINT_DATA_CHUNKS[meshlet.strand_point_chunk_index].strand_points[segment.x];
  StrandPoint p1 = EE_STRAND_POINT_DATA_CHUNKS[meshlet.strand_point_chunk_index].strand_points[segment.y];
  StrandPoint p2 = EE_STRAND_POINT_DATA_CHUNKS[meshlet.strand_point_chunk_index].strand_points[segment.z];
  StrandPoint p3 = EE_STRAND_POINT_DATA_CHUNKS[meshlet.strand_point_chunk_index].strand_points[segment.w];
  float3 position0, position1, tangent0, tangent1;
  float3 normal0, normal1, unused_float3;
  float thickness0, thickness1, unused_float;
  EE_SPLINE_INTERPOLATION(p0.position, p1.position, p2.position, p3.position, position0, tangent0, 0.0f);
  EE_SPLINE_INTERPOLATION(p0.position, p1.position, p2.position, p3.position, position1, tangent1, 1.0f);
  EE_SPLINE_INTERPOLATION(p0.normal, p1.normal, p2.normal, p3.normal, normal0, unused_float3, 0.0f);
  EE_SPLINE_INTERPOLATION(p0.normal, p1.normal, p2.normal, p3.normal, normal1, unused_float3, 1.0f);
  EE_SPLINE_INTERPOLATION(p0.thickness, p1.thickness, p2.thickness, p3.thickness, thickness0, unused_float, 0.0f);
  EE_SPLINE_INTERPOLATION(p0.thickness, p1.thickness, p2.thickness, p3.thickness, thickness1, unused_float, 1.0f);
  float3 frame_normal0, frame_normal1, frame_bitangent0, frame_bitangent1;
  EE_STRAND_FRAME(tangent0, normal0, frame_normal0, frame_bitangent0);
  EE_STRAND_FRAME(tangent1, normal1, frame_normal1, frame_bitangent1);
  SetMeshOutputCounts(10u, 8u);
  uint instance_index = strand_shadow_task.instance_index;
  float4x4 model = EE_INSTANCES[instance_index].model;
  float4x4 light_projection = EE_SPOT_LIGHTS[EE_BASIC_CONSTANTS.camera_index].light_space_matrix;
  for (uint vertex = lane_id; vertex < 10u; vertex += WORKGROUP_SIZE) {
    bool endpoint = (vertex & 1u) != 0u;
    uint ring_vertex = vertex >> 1u;
    float angle = -PI2 * float(ring_vertex) / float(STRAND_SHADOW_RING_SIZE);
    float3 position = endpoint ? position1 : position0;
    float3 radial = (endpoint ? frame_normal1 : frame_normal0) * sin(angle) +
                    (endpoint ? frame_bitangent1 : frame_bitangent0) * cos(angle);
    float thickness = endpoint ? thickness1 : thickness0;
    float4 world_position = mul(float4(position + radial * thickness, 1.0f), model);
    vertices[vertex].position = mul(world_position, light_projection);
  }
  for (uint primitive = lane_id; primitive < 8u; primitive += WORKGROUP_SIZE) {
    uint ring_index = primitive >> 1u;
    uint source = ring_index * 2u;
    uint target = source + 1u;
    uint next_source = source + 2u;
    uint next_target = next_source + 1u;
    triangles[primitive] =
        (primitive & 1u) == 0u ? uint3(source, target, next_source) : uint3(next_source, target, next_target);
  }
}
