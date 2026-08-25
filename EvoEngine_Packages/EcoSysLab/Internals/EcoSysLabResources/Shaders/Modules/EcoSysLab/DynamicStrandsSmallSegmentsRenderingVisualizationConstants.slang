[[vk::push_constant]] public cbuffer STRANDS_RENDER_CONSTANTS {
  public int EE_INSTANCE_INDEX;
  public int EE_CAMERA_INDEX;
  public uint uniform_particles_size;
  public float thickness_multiplier;

  public float4 color0;
  public float4 color1;

  public float4 position_scale;

  public uint color_mode;
  public float factor;
  public float boundary_layer_radius;
};


public static const uint SMALL_SEGMENT_VERTICES_SIZE = 20u;
public static const uint SMALL_SEGMENT_TRIANGLE_SIZE = 20u;


public static const float3 positions[] = {
    float3(-0.5f, -0.5f, -0.5f), float3(0.5f, 0.5f, -0.5f), float3(0.5f, -0.5f, -0.5f), float3(-0.5f, 0.5f, -0.5f),
    float3(-0.5f, -0.5f, 0.0f), float3(0.5f, -0.5f, 0.0f), float3(0.5f, 0.5f, 0.0f), float3(-0.5f, 0.5f, 0.0f),
    float3(-0.5f, -0.5f, 0.5f),  float3(0.5f, -0.5f, 0.5f), float3(0.5f, 0.5f, 0.5f),   float3(-0.5f, 0.5f, 0.5f),

    float3(-0.5f, -0.5f, -0.5f), float3(0.5f, 0.5f, -0.5f), float3(0.5f, -0.5f, -0.5f), float3(-0.5f, 0.5f, -0.5f),
    float3(-0.5f, -0.5f, 0.5f),  float3(0.5f, -0.5f, 0.5f), float3(0.5f, 0.5f, 0.5f),   float3(-0.5f, 0.5f, 0.5f)
};

public static const float3 normals[] = {
    float3(-1.f, -1.f, 0.f), float3(1.f, 1.f, 0.f), float3(1.f, -1.f, 0.f), float3(-1.f, 1.f, 0.f),
    float3(-1.f, -1.f, 0.f), float3(1.f, -1.f, 0.f), float3(1.f, 1.f, 0.f), float3(-1.f, 1.f, 0.f),
    float3(-1.f, -1.f, 0.f), float3(1.f, -1.f, 0.f), float3(1.f, 1.f, 0.f), float3(-1.f, 1.f, 0.f),

    float3(0, 0, -1), float3(0, 0, -1), float3(0, 0, -1),  float3(0, 0, -1),
    float3(0, 0, 1), float3(0, 0, 1), float3(0, 0, 1),  float3(0, 0, 1)
};

public static const float3 tangents[] = {
    float3(0, 0, -1), float3(0, 0, -1), float3(0, 0, -1), float3(0, 0, -1),
    float3(0, 0, -1), float3(0, 0, -1), float3(0, 0, -1), float3(0, 0, -1),
    float3(0, 0, -1), float3(0, 0, -1), float3(0, 0, -1), float3(0, 0, -1),

    float3(0, 1, 0),  float3(0, 1, 0),  float3(0, 1, 0),  float3(0, 1, 0),
    float3(0, 1, 0),  float3(0, 1, 0),  float3(0, 1, 0),  float3(0, 1, 0),
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
                                uint3(7, 10, 6), uint3(10, 7, 11),

                                uint3(15, 13, 14), uint3(14, 12, 15), // + 12
                                uint3(17, 19, 16), uint3(19, 17, 18) // + 8
};

public static const float2 tex_coords[] = {
    float2(0, 0), float2(0, 0), float2(1, 0), float2(1, 0),
    float2(0, .5f), float2(1, .5f), float2(0,.5f), float2(1, .5f),
    float2(0, 1), float2(1, 1), float2(0, 1), float2(1, 1),


    float2(0, 0),   float2(0, 0),   float2(1, 0), float2(1, 0),
    float2(0, 1), float2(1, 1), float2(0, 1), float2(1, 1)
};
