
layout(push_constant) uniform STRANDS_RENDER_CONSTANTS {
  int EE_INSTANCE_INDEX;
  int EE_CAMERA_INDEX;
  uint uniform_particles_size;
};

#define SMALL_SEGMENT_VERTICES_SIZE 8
#define SMALL_SEGMENT_TRIANGLE_SIZE 12

vec3 positions[] = {vec3(-0.5, -0.5, -0.5), vec3(0.5, 0.5, -0.5), vec3(0.5, -0.5, -0.5), vec3(-0.5, 0.5, -0.5),

                    vec3(-0.5, -0.5, 0.5),  vec3(0.5, -0.5, 0.5), vec3(0.5, 0.5, 0.5),   vec3(-0.5, 0.5, 0.5)};


vec3 normals[] = {vec3(-0.5, -0.5, -0.5), vec3(0.5, 0.5, -0.5), vec3(0.5, -0.5, -0.5), vec3(-0.5, 0.5, -0.5),

                    vec3(-0.5, -0.5, 0.5),  vec3(0.5, -0.5, 0.5), vec3(0.5, 0.5, 0.5),   vec3(-0.5, 0.5, 0.5)};

vec2 tex_coords[] = {vec2(0, 0), vec2(0, 1), vec2(1, 1), vec2(1, 0), vec2(0, 0), vec2(0, 1), vec2(1, 1), vec2(1, 0)};


uvec3 triangles[] = {uvec3(0, 1, 2), uvec3(1, 0, 3), uvec3(4, 5, 6),

                     uvec3(6, 7, 4), uvec3(7, 3, 0), uvec3(0, 4, 7),

                     uvec3(6, 2, 1), uvec3(2, 6, 5), uvec3(0, 2, 5),

                     uvec3(5, 4, 0), uvec3(3, 6, 1), uvec3(6, 3, 7)};
