[[vk::push_constant]] public cbuffer STRANDS_RENDER_CONSTANTS {
  public int EE_INSTANCE_INDEX;
  public int EE_CAMERA_INDEX;
  public uint uniform_particles_size;
  public float thickness_multiplier;
  public float4 position_scale;
  public int splinter_material_index;
  public int render_material_index;
};

public static const uint SMALL_SEGMENT_SHADOW_MAP_VERTICES_SIZE = 8u;
public static const uint SMALL_SEGMENT_SHADOW_MAP_TRIANGLE_SIZE = 8u;



public static const float3 shadow_map_positions[] = {
    float3(-0.5f, -0.5f, -0.5f), float3(0.5f, 0.5f, -0.5f), float3(0.5f, -0.5f, -0.5f), float3(-0.5f, 0.5f, -0.5f),

    float3(-0.5f, -0.5f, 0.5f),  float3(0.5f, -0.5f, 0.5f), float3(0.5f, 0.5f, 0.5f),   float3(-0.5f, 0.5f, 0.5f)};

public static const uint3 shadow_map_triangles[] = {/* uint3(0, 1, 2),
                        uint3(1, 0, 3),
                        uint3(4, 5, 6),
                        uint3(6, 7, 4), */
                                uint3(7, 3, 0), uint3(0, 4, 7),

                                uint3(6, 2, 1), uint3(2, 6, 5), uint3(0, 2, 5),

                                uint3(5, 4, 0), uint3(3, 6, 1), uint3(6, 3, 7)};


public static const uint SMALL_SEGMENT_VERTICES_SIZE = 12u;
public static const uint SMALL_SEGMENT_TRIANGLE_SIZE = 16u;


public static const float3 positions0[] = {
    float3(-0.5f, -0.5f, -0.5f), float3(0.5f, 0.5f, -0.5f), float3(0.5f, -0.5f, -0.5f), float3(-0.5f, 0.5f, -0.5f),
    float3(-0.5f, -0.5f, 0.0f), float3(0.5f, -0.5f, 0.0f), float3(0.5f, 0.5f, 0.0f), float3(-0.5f, 0.5f, 0.0f),
    float3(-0.5f, -0.5f, 0.5f),  float3(0.5f, -0.5f, 0.5f), float3(0.5f, 0.5f, 0.5f),   float3(-0.5f, 0.5f, 0.5f)};

public static const float3 positions1[] = {
    float3(-0.0f, -0.0f, -0.5f), float3(0.0f, 0.0f, -0.5f), float3(0.0f, -0.0f, -0.5f), float3(-0.0f, 0.0f, -0.5f),
    float3(-0.25f, -0.25f, 0.0f), float3(0.25f, -0.25f, 0.0f), float3(0.25f, 0.25f, 0.0f), float3(-0.25f, 0.25f, 0.0f),
    float3(-0.5f, -0.5f, 0.5f),  float3(0.5f, -0.5f, 0.5f), float3(0.5f, 0.5f, 0.5f),   float3(-0.5f, 0.5f, 0.5f)};

public static const float3 positions2[] = {
    float3(-0.5f, -0.5f, -0.5f), float3(0.5f, 0.5f, -0.5f), float3(0.5f, -0.5f, -0.5f), float3(-0.5f, 0.5f, -0.5f),
    float3(-0.25f, -0.25f, 0.0f), float3(0.25f, -0.25f, 0.0f), float3(0.25f, 0.25f, 0.0f), float3(-0.25f, 0.25f, 0.0f),
    float3(-0.0f, -0.0f, 0.5f),  float3(0.0f, -0.0f, 0.5f), float3(0.0f, 0.0f, 0.5f),   float3(-0.0f, 0.0f, 0.5f)};

public static const float3 positions3[] = {
    float3(-0.0f, -0.0f, -0.5f), float3(0.0f, 0.0f, -0.5f), float3(0.0f, -0.0f, -0.5f), float3(-0.0f, 0.0f, -0.5f),
    float3(-0.25f, -0.25f, 0.0f), float3(0.25f, -0.25f, 0.0f), float3(0.25f, 0.25f, 0.0f), float3(-0.25f, 0.25f, 0.0f),
    float3(-0.0f, -0.0f, 0.5f),  float3(0.0f, -0.0f, 0.5f), float3(0.0f, 0.0f, 0.5f),   float3(-0.0f, 0.0f, 0.5f)};

public static const float3 normals[] = {float3(-1.f, -1.f, 0.f), float3(1.f, 1.f, 0.f), float3(1.f, -1.f, 0.f), float3(-1.f, 1.f, 0.f),
                  float3(-1.f, -1.f, 0.f), float3(1.f, -1.f, 0.f), float3(1.f, 1.f, 0.f), float3(-1.f, 1.f, 0.f),
                  float3(-1.f, -1.f, 0.f), float3(1.f, -1.f, 0.f), float3(1.f, 1.f, 0.f), float3(-1.f, 1.f, 0.f)};

public static const float3 tangents[] = {
    float3(0, 0, -1), float3(0, 0, -1), float3(0, 0, -1), float3(0, 0, -1),
    float3(0, 0, -1), float3(0, 0, -1), float3(0, 0, -1), float3(0, 0, -1),
    float3(0, 0, -1), float3(0, 0, -1), float3(0, 0, -1), float3(0, 0, -1),
};

public static const uint3 triangles[] = {/* uint3(0, 1, 2),
                        uint3(1, 0, 3),
                        uint3(4, 5, 6),
                        uint3(6, 7, 4), */
                                uint3(7, 3, 0), uint3(0, 4, 7),
                                uint3(6, 2, 1), uint3(2, 6, 5),
                                uint3(0, 2, 5), uint3(5, 4, 0),
                                uint3(3, 6, 1), uint3(6, 3, 7),

                                uint3(11, 7, 4), uint3(4, 8, 11),
                                uint3(10, 5, 6), uint3(5, 10, 9),
                                uint3(4, 5, 9), uint3(9, 8, 4),
                                uint3(7, 10, 6), uint3(10, 7, 11)
};

public static const float2 tex_coords[] = {
    float2(0, 0), float2(0, 0), float2(1, 0), float2(1, 0),
    float2(0, .5f), float2(1, .5f), float2(0,.5f), float2(1, .5f),
    float2(0, 1), float2(1, 1), float2(0, 1), float2(1, 1)
};
