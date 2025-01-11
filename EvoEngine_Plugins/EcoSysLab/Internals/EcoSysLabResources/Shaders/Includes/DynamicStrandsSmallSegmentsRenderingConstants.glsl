
layout(push_constant) uniform STRANDS_RENDER_CONSTANTS {
  int EE_INSTANCE_INDEX;
  int EE_CAMERA_INDEX;
  uint uniform_particles_size;
  float thickness_multiplier;
};

#define SMALL_SEGMENT_SHADOW_MAP_VERTICES_SIZE 8
#define SMALL_SEGMENT_SHADOW_MAP_TRIANGLE_SIZE 8



vec3 shadow_map_positions[] = {
    vec3(-0.5f, -0.5f, -0.5f), vec3(0.5f, 0.5f, -0.5f), vec3(0.5f, -0.5f, -0.5f), vec3(-0.5f, 0.5f, -0.5f),

    vec3(-0.5f, -0.5f, 0.5f),  vec3(0.5f, -0.5f, 0.5f), vec3(0.5f, 0.5f, 0.5f),   vec3(-0.5f, 0.5f, 0.5f)};

uvec3 shadow_map_triangles[] = {/* uvec3(0, 1, 2),
                        uvec3(1, 0, 3),
                        uvec3(4, 5, 6),
                        uvec3(6, 7, 4), */
                                uvec3(7, 3, 0), uvec3(0, 4, 7),

                                uvec3(6, 2, 1), uvec3(2, 6, 5), uvec3(0, 2, 5),

                                uvec3(5, 4, 0), uvec3(3, 6, 1), uvec3(6, 3, 7)};


#define SMALL_SEGMENT_VERTICES_SIZE 12
#define SMALL_SEGMENT_TRIANGLE_SIZE 16


vec3 positions0[] = {
    vec3(-0.5f, -0.5f, -0.5f), vec3(0.5f, 0.5f, -0.5f), vec3(0.5f, -0.5f, -0.5f), vec3(-0.5f, 0.5f, -0.5f),
    vec3(-0.5f, -0.5f, 0.0f), vec3(0.5f, -0.5f, 0.0f), vec3(0.5f, 0.5f, 0.0f), vec3(-0.5f, 0.5f, 0.0f),
    vec3(-0.5f, -0.5f, 0.5f),  vec3(0.5f, -0.5f, 0.5f), vec3(0.5f, 0.5f, 0.5f),   vec3(-0.5f, 0.5f, 0.5f)};

vec3 positions1[] = {
    vec3(-0.0f, -0.0f, -0.5f), vec3(0.0f, 0.0f, -0.5f), vec3(0.0f, -0.0f, -0.5f), vec3(-0.0f, 0.0f, -0.5f),
    vec3(-0.25f, -0.25f, 0.0f), vec3(0.25f, -0.25f, 0.0f), vec3(0.25f, 0.25f, 0.0f), vec3(-0.25f, 0.25f, 0.0f),
    vec3(-0.5f, -0.5f, 0.5f),  vec3(0.5f, -0.5f, 0.5f), vec3(0.5f, 0.5f, 0.5f),   vec3(-0.5f, 0.5f, 0.5f)};

vec3 positions2[] = {
    vec3(-0.5f, -0.5f, -0.5f), vec3(0.5f, 0.5f, -0.5f), vec3(0.5f, -0.5f, -0.5f), vec3(-0.5f, 0.5f, -0.5f),
    vec3(-0.25f, -0.25f, 0.0f), vec3(0.25f, -0.25f, 0.0f), vec3(0.25f, 0.25f, 0.0f), vec3(-0.25f, 0.25f, 0.0f),
    vec3(-0.0f, -0.0f, 0.5f),  vec3(0.0f, -0.0f, 0.5f), vec3(0.0f, 0.0f, 0.5f),   vec3(-0.0f, 0.0f, 0.5f)};

vec3 positions3[] = {
    vec3(-0.0f, -0.0f, -0.5f), vec3(0.0f, 0.0f, -0.5f), vec3(0.0f, -0.0f, -0.5f), vec3(-0.0f, 0.0f, -0.5f),
    vec3(-0.25f, -0.25f, 0.0f), vec3(0.25f, -0.25f, 0.0f), vec3(0.25f, 0.25f, 0.0f), vec3(-0.25f, 0.25f, 0.0f),
    vec3(-0.0f, -0.0f, 0.5f),  vec3(0.0f, -0.0f, 0.5f), vec3(0.0f, 0.0f, 0.5f),   vec3(-0.0f, 0.0f, 0.5f)};

vec3 normals[] = {vec3(-1.f, -1.f, 0.f), vec3(1.f, 1.f, 0.f), vec3(1.f, -1.f, 0.f), vec3(-1.f, 1.f, 0.f),
                  vec3(-1.f, -1.f, 0.f), vec3(1.f, -1.f, 0.f), vec3(1.f, 1.f, 0.f), vec3(-1.f, 1.f, 0.f),
                  vec3(-1.f, -1.f, 0.f), vec3(1.f, -1.f, 0.f), vec3(1.f, 1.f, 0.f), vec3(-1.f, 1.f, 0.f)};

vec3 tangents[] = {
    vec3(0, 0, -1), vec3(0, 0, -1), vec3(0, 0, -1), vec3(0, 0, -1),
    vec3(0, 0, -1), vec3(0, 0, -1), vec3(0, 0, -1), vec3(0, 0, -1),
    vec3(0, 0, -1), vec3(0, 0, -1), vec3(0, 0, -1), vec3(0, 0, -1),
};

uvec3 triangles[] = {/* uvec3(0, 1, 2),
                        uvec3(1, 0, 3),
                        uvec3(4, 5, 6),
                        uvec3(6, 7, 4), */
                                uvec3(7, 3, 0), uvec3(0, 4, 7),
                                uvec3(6, 2, 1), uvec3(2, 6, 5), 
                                uvec3(0, 2, 5), uvec3(5, 4, 0), 
                                uvec3(3, 6, 1), uvec3(6, 3, 7),

                                uvec3(11, 7, 4), uvec3(4, 8, 11),
                                uvec3(10, 5, 6), uvec3(5, 10, 9), 
                                uvec3(4, 5, 9), uvec3(9, 8, 4), 
                                uvec3(7, 10, 6), uvec3(10, 7, 11)
};

vec2 tex_coords[] = {
    vec2(0, 0), vec2(0, 0), vec2(1, 0), vec2(1, 0), 
    vec2(0, .5f), vec2(1, .5f), vec2(0,.5f), vec2(1, .5f), 
    vec2(0, 1), vec2(1, 1), vec2(0, 1), vec2(1, 1)
};


