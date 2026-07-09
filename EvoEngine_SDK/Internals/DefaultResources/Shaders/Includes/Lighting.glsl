
#extension GL_EXT_control_flow_attributes : require

#include "DDGI.glsl"
#include "VogelDisk.glsl"

layout(set = EE_PER_GROUP_SET, binding = 14) uniform sampler2DArray EE_DIRECTIONAL_LIGHT_SM;
layout(set = EE_PER_GROUP_SET, binding = 15) uniform sampler2DArray EE_POINT_LIGHT_SM;
layout(set = EE_PER_GROUP_SET, binding = 16) uniform sampler2D EE_SPOT_LIGHT_SM;
layout(set = EE_PER_GROUP_SET, binding = 17) uniform sampler2D EE_DDGI_IRRADIANCE_ATLAS;
layout(set = EE_PER_GROUP_SET, binding = 18) uniform sampler2D EE_DDGI_VISIBILITY_ATLAS;
layout(set = EE_PER_GROUP_SET, binding = 19) readonly buffer EE_DDGI_PROBE_STATE_BLOCK {
  vec4 EE_DDGI_PROBE_STATE[];
};

#ifdef EE_RASTER_FIXED_LIGHTING_TEXTURES
layout(set = EE_RASTER_FIXED_LIGHTING_TEXTURE_SET, binding = 0) uniform sampler2D EE_RASTER_BRDF_LUT;
layout(set = EE_RASTER_FIXED_LIGHTING_TEXTURE_SET, binding = 1) uniform samplerCube EE_RASTER_SKYBOX;
layout(set = EE_RASTER_FIXED_LIGHTING_TEXTURE_SET, binding = 2) uniform samplerCube EE_RASTER_IRRADIANCE_MAP;
layout(set = EE_RASTER_FIXED_LIGHTING_TEXTURE_SET, binding = 3) uniform samplerCube EE_RASTER_PREFILTERED_MAP;
#endif

vec3 EE_SKY_COLOR(vec3 direction) {
	Camera camera = EE_CAMERAS[EE_CAMERA_INDEX];
	if (camera.use_clear_color == 1) {
		return camera.clear_color.xyz * camera.clear_color.w;
	}
#ifdef EE_RASTER_FIXED_LIGHTING_TEXTURES
	return pow(texture(EE_RASTER_SKYBOX, normalize(direction)).rgb, vec3(1.0f / EE_ENVIRONMENT.gamma)) * camera.clear_color.w;
#else
	return pow(texture(EE_CUBEMAPS[camera.skybox_tex_index], normalize(direction)).rgb, vec3(1.0f / EE_ENVIRONMENT.gamma)) * camera.clear_color.w;
#endif
}

const float PI = 3.14159265359;
// ----------------------------------------------------------------------------
float EE_FUNC_DISTRIBUTION_GGX(vec3 N, vec3 H, float roughness)
{
	float a = roughness * roughness;
	float a2 = a * a;
	float NdotH = max(dot(N, H), 0.0f);
	float NdotH2 = NdotH * NdotH;

	float nom = a2;
	float denom = (NdotH2 * (a2 - 1.0f) + 1.0f);
	denom = PI * denom * denom;

	return nom / max(denom, 0.001f); // prevent divide by zero for roughness=0.0f and NdotH=1.0
}
// ----------------------------------------------------------------------------
float EE_FUNC_GEOMETRY_SCHLICK_GGX(float NdotV, float roughness)
{
	float r = (roughness + 1.0f);
	float k = (r * r) / 8.0f;

	float nom = NdotV;
	float denom = NdotV * (1.0f - k) + k;

	return nom / denom;
}
// ----------------------------------------------------------------------------
float EE_FUNC_GEOMETRY_SMITH(vec3 N, vec3 V, vec3 L, float roughness)
{
	float NdotV = max(dot(N, V), 0.0f);
	float NdotL = max(dot(N, L), 0.0f);
	float ggx2 = EE_FUNC_GEOMETRY_SCHLICK_GGX(NdotV, roughness);
	float ggx1 = EE_FUNC_GEOMETRY_SCHLICK_GGX(NdotL, roughness);

	return ggx1 * ggx2;
}
// ----------------------------------------------------------------------------
vec3 EE_FUNC_FRESNEL_SCHLICK(float cosTheta, vec3 F0)
{
	return F0 + (1.0f - F0) * pow(max(1.0f - cosTheta, 0.0f), 5.0f);
}

// ----------------------------------------------------------------------------
vec3 EE_FUNC_FRESNEL_SCHLICK_ROUGHNESS(float cosTheta, vec3 F0, float roughness)
{
	return F0 + (max(vec3(1.0f - roughness), F0) - F0) * pow(max(1.0f - cosTheta, 0.0f), 5.0f);
}

vec3 EE_FUNC_CALCULATE_LIGHTS(in bool calculateShadow, vec3 albedo, float specular, float dist, vec3 normal, vec3 viewDir, vec3 fragPos, float metallic, float roughness, vec3 F0);
vec3 EE_FUNC_DIRECTIONAL_LIGHT(vec3 albedo, float specular, int i, vec3 normal, vec3 viewDir, float metallic, float roughness, vec3 F0);
vec3 EE_FUNC_POINT_LIGHT(vec3 albedo, float specular, int i, vec3 normal, vec3 fragPos, vec3 viewDir, float metallic, float roughness, vec3 F0);
vec3 EE_FUNC_SPOT_LIGHT(vec3 albedo, float specular, int i, vec3 normal, vec3 fragPos, vec3 viewDir, float metallic, float roughness, vec3 F0);
float EE_FUNC_DIRECTIONAL_LIGHT_SHADOW(int i, int splitIndex, vec3 fragPos, vec3 normal, float cameraFragDistance);
float EE_FUNC_POINT_LIGHT_SHADOW(int i, vec3 fragPos, float cameraFragDistance);
float EE_FUNC_SPOT_LIGHT_SHADOW(int i, vec3 fragPos, float cameraFragDistance);
int EE_FUNC_DIRECTIONAL_SHADOW_CASCADE_INDEX(float dist);
float EE_FUNC_DIRECTIONAL_LIGHT_CASCADE_SHADOW(int lightIndex, float dist, vec3 fragPos, vec3 normal,
                                              float cameraFragDistance);
vec4 EE_FUNC_DIRECTIONAL_SHADOW_DEBUG(float dist, vec3 fragPos);

vec3 EE_FUNC_CALCULATE_DDGI_DIFFUSE(vec3 albedo, vec3 normal, vec3 viewDir, vec3 fragPos) {
  const float intensity = EE_RENDER_INFO.ddgi_indirect_intensity;
  if (intensity <= 0.0f) {
    return vec3(0.0f);
  }

  const uvec3 probe_counts = max(uvec3(EE_RENDER_INFO.ddgi_probe_counts.xyz), uvec3(1u));
  const vec3 biased_frag_pos = EE_DDGI_SURFACE_BIASED_POSITION(
      fragPos, normal, viewDir, EE_RENDER_INFO.ddgi_volume_parameters.z, EE_RENDER_INFO.ddgi_volume_parameters.w);
  const vec3 volume_probe_coordinate =
      EE_DDGI_PROBE_COORDINATE(fragPos, EE_RENDER_INFO.ddgi_first_probe.xyz,
                               EE_RENDER_INFO.ddgi_probe_step_x.xyz, EE_RENDER_INFO.ddgi_probe_step_y.xyz,
                               EE_RENDER_INFO.ddgi_probe_step_z.xyz);
  const float volume_blend_weight =
      EE_DDGI_VOLUME_BLEND_WEIGHT(volume_probe_coordinate, probe_counts, EE_RENDER_INFO.ddgi_probe_step_x.xyz,
                                  EE_RENDER_INFO.ddgi_probe_step_y.xyz, EE_RENDER_INFO.ddgi_probe_step_z.xyz);
  if (volume_blend_weight <= 0.0f) {
    return vec3(0.0f);
  }

  const vec3 biased_probe_coordinate =
      EE_DDGI_PROBE_COORDINATE(biased_frag_pos, EE_RENDER_INFO.ddgi_first_probe.xyz,
                               EE_RENDER_INFO.ddgi_probe_step_x.xyz, EE_RENDER_INFO.ddgi_probe_step_y.xyz,
                               EE_RENDER_INFO.ddgi_probe_step_z.xyz);

  const uint irradiance_tile_size = max(uint(EE_RENDER_INFO.ddgi_atlas_parameters.x), 1u);
  const uint irradiance_atlas_columns = max(uint(EE_RENDER_INFO.ddgi_atlas_parameters.y), 1u);
  const uint visibility_tile_size = max(uint(EE_RENDER_INFO.ddgi_atlas_parameters.z), 1u);
  const uint visibility_atlas_columns = max(uint(EE_RENDER_INFO.ddgi_atlas_parameters.w), 1u);
  const vec2 irradiance_atlas_size = vec2(textureSize(EE_DDGI_IRRADIANCE_ATLAS, 0));
  const vec2 visibility_atlas_size = vec2(textureSize(EE_DDGI_VISIBILITY_ATLAS, 0));
  const ivec3 probe_scroll_offset = ivec3(round(EE_RENDER_INFO.ddgi_probe_scroll_offset.xyz));
  const float irradiance_gamma = max(EE_RENDER_INFO.ddgi_probe_counts.w, 1.0f);
  const float visibility_bias = max(EE_RENDER_INFO.ddgi_volume_parameters.y, 0.0f);
  const vec3 max_probe_grid = vec3(probe_counts - uvec3(1u));
  const vec3 base_probe_grid = clamp(floor(biased_probe_coordinate), vec3(0.0f), max_probe_grid);
  const vec3 base_probe_world_position =
      EE_DDGI_PROBE_WORLD_POSITION(base_probe_grid, EE_RENDER_INFO.ddgi_first_probe.xyz,
                                   EE_RENDER_INFO.ddgi_probe_step_x.xyz, EE_RENDER_INFO.ddgi_probe_step_y.xyz,
                                   EE_RENDER_INFO.ddgi_probe_step_z.xyz);
  const vec3 base_probe_to_biased_position = biased_frag_pos - base_probe_world_position;
  const vec3 probe_fraction =
      clamp(vec3(EE_DDGI_AXIS_COORDINATE(base_probe_to_biased_position, EE_RENDER_INFO.ddgi_probe_step_x.xyz),
                 EE_DDGI_AXIS_COORDINATE(base_probe_to_biased_position, EE_RENDER_INFO.ddgi_probe_step_y.xyz),
                 EE_DDGI_AXIS_COORDINATE(base_probe_to_biased_position, EE_RENDER_INFO.ddgi_probe_step_z.xyz)),
            vec3(0.0f), vec3(1.0f));

  vec3 diffuse = vec3(0.0f);
  float weight_sum = 0.0f;
  for (uint z = 0u; z < 2u; ++z) {
    for (uint y = 0u; y < 2u; ++y) {
      for (uint x = 0u; x < 2u; ++x) {
        const uvec3 corner = uvec3(x, y, z);
        const vec3 corner_weight = max(vec3(0.001f), mix(vec3(1.0f) - probe_fraction, probe_fraction, vec3(corner)));
        const float trilinear_weight = corner_weight.x * corner_weight.y * corner_weight.z;

        const vec3 probe_grid = clamp(base_probe_grid + vec3(corner), vec3(0.0f), max_probe_grid);
        const uvec3 probe_index_3d = uvec3(probe_grid);
        const uint probe_index = EE_DDGI_SCROLL_PROBE_INDEX(probe_index_3d, probe_scroll_offset, probe_counts);
        const vec4 probe_state = EE_DDGI_PROBE_STATE[probe_index];
        const float probe_active = 1.0f - clamp(probe_state.w, 0.0f, 1.0f);
        if (probe_active <= 0.0f) {
          continue;
        }
        const vec3 probe_position =
            EE_DDGI_PROBE_WORLD_POSITION(probe_grid, EE_RENDER_INFO.ddgi_first_probe.xyz,
                                         EE_RENDER_INFO.ddgi_probe_step_x.xyz, EE_RENDER_INFO.ddgi_probe_step_y.xyz,
                                         EE_RENDER_INFO.ddgi_probe_step_z.xyz) +
            probe_state.xyz;

        const vec3 surface_to_probe = probe_position - fragPos;
        const float probe_distance = length(surface_to_probe);
        const vec3 biased_surface_to_probe = probe_position - biased_frag_pos;
        const float biased_probe_distance = length(biased_surface_to_probe);
        const vec3 surface_to_probe_direction = probe_distance > 0.001f ? surface_to_probe / probe_distance : normal;
        const vec3 biased_surface_to_probe_direction =
            biased_probe_distance > 0.001f ? biased_surface_to_probe / biased_probe_distance : normal;
        const vec3 probe_to_surface = -biased_surface_to_probe_direction;
        const vec2 irradiance_atlas_uv =
            EE_DDGI_ATLAS_UV(probe_index, irradiance_atlas_columns, irradiance_tile_size, normal, irradiance_atlas_size);
        const vec2 visibility_atlas_uv =
            EE_DDGI_ATLAS_UV(probe_index, visibility_atlas_columns, visibility_tile_size, probe_to_surface,
                             visibility_atlas_size);
        const vec4 irradiance = texture(EE_DDGI_IRRADIANCE_ATLAS, irradiance_atlas_uv);
        const vec4 visibility_sample = texture(EE_DDGI_VISIBILITY_ATLAS, visibility_atlas_uv);
        const float wrap_shading = (dot(surface_to_probe_direction, normal) + 1.0f) * 0.5f;
        float visibility_weight = wrap_shading * wrap_shading + 0.2f;
        const float distance_visibility =
            EE_DDGI_CHEBYSHEV_VISIBILITY(visibility_sample.rg, biased_probe_distance, visibility_bias);
        visibility_weight *= max(0.05f, distance_visibility);
        visibility_weight = max(0.000001f, visibility_weight);
        visibility_weight = EE_DDGI_CRUSH_LOW_WEIGHT(visibility_weight);
        const float sample_weight = trilinear_weight * visibility_weight;
        const vec3 decoded_irradiance = pow(max(irradiance.rgb, vec3(0.0f)), vec3(irradiance_gamma * 0.5f));
        diffuse += decoded_irradiance * sample_weight;
        weight_sum += sample_weight;
      }
    }
  }
  if (weight_sum <= 0.0f) {
    return vec3(0.0f);
  }
  diffuse /= weight_sum;
  diffuse *= diffuse * (2.0f * EE_DDGI_PI);
  return albedo / EE_DDGI_PI * diffuse * intensity * volume_blend_weight;
}

vec3 EE_FUNC_CALCULATE_ENVIRONMENTAL_LIGHT(vec3 albedo, vec3 normal, vec3 viewDir, float metallic, float roughness, vec3 F0)
{
	// ambient lighting (we now use IBL as the ambient term)
	vec3 F = EE_FUNC_FRESNEL_SCHLICK_ROUGHNESS(max(dot(normal, viewDir), 0.0f), F0, roughness);
	vec3 R = reflect(-viewDir, normal);
	vec3 kS = F;
	vec3 kD = 1.0f - kS;
	kD *= 1.0f - metallic;

#ifdef EE_RASTER_FIXED_LIGHTING_TEXTURES
	vec3 irradiance = EE_ENVIRONMENT.background_color.w == 1.0f ? EE_ENVIRONMENT.background_color.xyz : pow(texture(EE_RASTER_IRRADIANCE_MAP, normal).rgb, vec3(1.0f / EE_ENVIRONMENT.gamma));
#else
	vec3 irradiance = EE_ENVIRONMENT.background_color.w == 1.0f ? EE_ENVIRONMENT.background_color.xyz : pow(texture(EE_CUBEMAPS[EE_CAMERAS[EE_CAMERA_INDEX].irradiance_map_index], normal).rgb, vec3(1.0f / EE_ENVIRONMENT.gamma));
#endif
	vec3 diffuse = irradiance * albedo;

	// sample both the pre-filter map and the BRDF lut and combine them together as per the Split-Sum approximation to get the IBL specular part.
#ifdef EE_RASTER_FIXED_LIGHTING_TEXTURES
	float reflectionLodScale = float(textureQueryLevels(EE_RASTER_PREFILTERED_MAP));
	vec3 prefilteredColor = EE_ENVIRONMENT.background_color.w == 1.0f ? EE_ENVIRONMENT.background_color.xyz : pow(textureLod(EE_RASTER_PREFILTERED_MAP, R, roughness * reflectionLodScale).rgb, vec3(1.0f / EE_ENVIRONMENT.gamma));
	vec2 brdf = texture(EE_RASTER_BRDF_LUT, vec2(max(dot(normal, viewDir), 0.0f), roughness)).rg;
#else
	int prefilteredMapIndex = EE_CAMERAS[EE_CAMERA_INDEX].prefiltered_map_index;
	float reflectionLodScale = float(textureQueryLevels(EE_CUBEMAPS[prefilteredMapIndex]));
	vec3 prefilteredColor = EE_ENVIRONMENT.background_color.w == 1.0f ? EE_ENVIRONMENT.background_color.xyz : pow(textureLod(EE_CUBEMAPS[prefilteredMapIndex], R, roughness * reflectionLodScale).rgb, vec3(1.0f / EE_ENVIRONMENT.gamma));
	vec2 brdf = texture(EE_TEXTURE_2DS[EE_RENDER_INFO.brdf_lut_map_index], vec2(max(dot(normal, viewDir), 0.0f), roughness)).rg;
#endif
	vec3 specular = prefilteredColor * (F * brdf.x + brdf.y);
	vec3 ambient = (kD * diffuse + specular) * EE_ENVIRONMENT.light_intensity;
	return ambient;
}

vec3 EE_FUNC_CALCULATE_LIGHTS(bool calculateShadow, vec3 albedo, float specular, float dist, vec3 normal, vec3 viewDir, vec3 fragPos, float metallic, float roughness, vec3 F0) {
	vec3 result = vec3(0.0, 0.0, 0.0f);
	vec3 fragToCamera = fragPos - EE_CAMERA_POSITION(EE_CAMERA_INDEX);
	float cameraFragDistance = length(fragToCamera);
	// phase 1: directional lighting
	for (int i = 0; i < EE_RENDER_INFO.directional_light_size; i++) {
		float shadow = 1.0f;
		int lightIndex = EE_CAMERA_INDEX * MAX_DIRECTIONAL_LIGHT_SIZE + i;
		if (calculateShadow && EE_DIRECTIONAL_LIGHTS[lightIndex].diffuse.w == 1.0f) {
			shadow = EE_FUNC_DIRECTIONAL_LIGHT_CASCADE_SHADOW(lightIndex, dist, fragPos, normal, cameraFragDistance);
		}
		result += EE_FUNC_DIRECTIONAL_LIGHT(albedo, specular, lightIndex, normal, viewDir, metallic, roughness, F0) * shadow;
	}
	// phase 2: point lights
	for (int i = 0; i < EE_RENDER_INFO.point_light_size; i++) {
		float shadow = 1.0f;
		if (calculateShadow && EE_POINT_LIGHTS[i].diffuse.w == 1.0f) {
			shadow = EE_FUNC_POINT_LIGHT_SHADOW(i, fragPos, cameraFragDistance);
		}
		result += EE_FUNC_POINT_LIGHT(albedo, specular, i, normal, fragPos, viewDir, metallic, roughness, F0) * shadow;
	}
	// phase 3: spot light
	for (int i = 0; i < EE_RENDER_INFO.spot_light_size; i++) {
		float shadow = 1.0f;
		if (calculateShadow && EE_SPOT_LIGHTS[i].diffuse.w == 1.0f) {
			shadow = EE_FUNC_SPOT_LIGHT_SHADOW(i, fragPos, cameraFragDistance);
		}
		result += EE_FUNC_SPOT_LIGHT(albedo, specular, i, normal, fragPos, viewDir, metallic, roughness, F0) * shadow;
	}
	return result;
}

// calculates the color when using a directional light.
vec3 EE_FUNC_DIRECTIONAL_LIGHT(vec3 albedo, float specular, int i, vec3 normal, vec3 viewDir, float metallic, float roughness, vec3 F0)
{
	DirectionalLight light = EE_DIRECTIONAL_LIGHTS[i];
	vec3 lightDir = normalize(-light.direction);
	vec3 H = normalize(viewDir + lightDir);
	vec3 radiance = light.diffuse.xyz;
	float normalDF = EE_FUNC_DISTRIBUTION_GGX(normal, H, roughness);
	float G = EE_FUNC_GEOMETRY_SMITH(normal, viewDir, lightDir, roughness);
	vec3 F = EE_FUNC_FRESNEL_SCHLICK(clamp(dot(H, viewDir), 0.0, 1.0f), F0);
	vec3 nominator = normalDF * G * F;
	float denominator = 4 * max(dot(normal, viewDir), 0.0f) * max(dot(normal, lightDir), 0.0f);
	vec3 spec = nominator / max(denominator, 0.001f) * specular;
	vec3 kS = F;
	vec3 kD = vec3(1.0f) - kS;
	kD *= 1.0f - metallic;
	float NdotL = max(dot(normal, lightDir), 0.0f);
	return (kD * albedo / PI + spec) * radiance * NdotL;
}

// calculates the color when using a point light.
vec3 EE_FUNC_POINT_LIGHT(vec3 albedo, float specular, int i, vec3 normal, vec3 fragPos, vec3 viewDir, float metallic, float roughness, vec3 F0)
{
	PointLight light = EE_POINT_LIGHTS[i];
	vec3 lightDir = normalize(light.position - fragPos);
	vec3 H = normalize(viewDir + lightDir);
	float distance = length(light.position - fragPos);
	float attenuation = 1.0f / (light.constant_linear_quadratic_far.x + light.constant_linear_quadratic_far.y * distance + light.constant_linear_quadratic_far.z * (distance * distance));
	vec3 radiance = light.diffuse.xyz * attenuation;
	float normalDF = EE_FUNC_DISTRIBUTION_GGX(normal, H, roughness);
	float G = EE_FUNC_GEOMETRY_SMITH(normal, viewDir, lightDir, roughness);
	vec3 F = EE_FUNC_FRESNEL_SCHLICK(clamp(dot(H, viewDir), 0.0f, 1.0f), F0);
	vec3 nominator = normalDF * G * F;
	float denominator = 4 * max(dot(normal, viewDir), 0.0f) * max(dot(normal, lightDir), 0.0f);
	vec3 spec = nominator / max(denominator, 0.001) * specular;
	vec3 kS = F;
	vec3 kD = vec3(1.0f) - kS;
	kD *= 1.0f - metallic;
	float NdotL = max(dot(normal, lightDir), 0.0f);
	return (kD * albedo / PI + spec) * radiance * NdotL;

}

// calculates the color when using a spot light.
vec3 EE_FUNC_SPOT_LIGHT(vec3 albedo, float specular, int i, vec3 normal, vec3 fragPos, vec3 viewDir, float metallic, float roughness, vec3 F0)
{
	SpotLight light = EE_SPOT_LIGHTS[i];
	vec3 lightDir = normalize(light.position - fragPos);
	vec3 H = normalize(viewDir + lightDir);
	float distance = length(light.position - fragPos);
	float attenuation = 1.0f / (light.constant_linear_quadratic_far.x + light.constant_linear_quadratic_far.y * distance + light.constant_linear_quadratic_far.z * (distance * distance));
	// spotlight intensity
	float theta = dot(lightDir, normalize(-light.direction));
	float epsilon = light.cutoff_outer_inner_size_bias.x - light.cutoff_outer_inner_size_bias.y;
	float intensity = clamp((theta - light.cutoff_outer_inner_size_bias.y) / epsilon, 0.0f, 1.0f);

	vec3 radiance = light.diffuse.xyz * attenuation * intensity;
	float normalDF = EE_FUNC_DISTRIBUTION_GGX(normal, H, roughness);
	float G = EE_FUNC_GEOMETRY_SMITH(normal, viewDir, lightDir, roughness);
	vec3 F = EE_FUNC_FRESNEL_SCHLICK(clamp(dot(H, viewDir), 0.0f, 1.0f), F0);
	vec3 nominator = normalDF * G * F;
	float denominator = 4 * max(dot(normal, viewDir), 0.0f) * max(dot(normal, lightDir), 0.0f);
	vec3 spec = nominator / max(denominator, 0.001f) * specular;
	vec3 kS = F;
	vec3 kD = vec3(1.0f) - kS;
	kD *= 1.0f - metallic;
	float NdotL = max(dot(normal, lightDir), 0.0f);
	return (kD * albedo / PI + spec) * radiance * NdotL;
}

int EE_FUNC_DIRECTIONAL_SHADOW_CASCADE_INDEX(float dist) {
	if (dist < EE_RENDER_INFO.shadow_split_0) return 0;
	if (dist < EE_RENDER_INFO.shadow_split_1) return 1;
	if (dist < EE_RENDER_INFO.shadow_split_2) return 2;
	if (dist < EE_RENDER_INFO.shadow_split_3) return 3;
	return -1;
}

float EE_FUNC_DIRECTIONAL_SHADOW_SPLIT_DISTANCE(int splitIndex) {
	if (splitIndex <= 0) return EE_RENDER_INFO.shadow_split_0;
	if (splitIndex == 1) return EE_RENDER_INFO.shadow_split_1;
	if (splitIndex == 2) return EE_RENDER_INFO.shadow_split_2;
	return EE_RENDER_INFO.shadow_split_3;
}

float EE_FUNC_DIRECTIONAL_SHADOW_TRANSITION_HALF_WIDTH(int boundaryIndex) {
	float requestedWidth = max(EE_RENDER_INFO.shadow_cascade_transition_width, 0.0f);
	if (requestedWidth <= 0.0f) return 0.0f;

	float splitDistance = EE_FUNC_DIRECTIONAL_SHADOW_SPLIT_DISTANCE(boundaryIndex);
	float previousSplit = boundaryIndex == 0 ? 0.0f : EE_FUNC_DIRECTIONAL_SHADOW_SPLIT_DISTANCE(boundaryIndex - 1);
	float nextSplit = EE_FUNC_DIRECTIONAL_SHADOW_SPLIT_DISTANCE(boundaryIndex + 1);
	float availableWidth = max(min(splitDistance - previousSplit, nextSplit - splitDistance), 0.0f);
	return min(requestedWidth, availableWidth) * 0.5f;
}

float EE_FUNC_DIRECTIONAL_SHADOW_DISTANCE_FADE(float dist) {
	float maxShadowDistance = EE_RENDER_INFO.shadow_split_3;
	if (dist >= maxShadowDistance) return 0.0f;

	float fadeWidth = max(EE_RENDER_INFO.shadow_fade_parameters.x, 0.0f);
	if (fadeWidth <= 0.0f) return 1.0f;
	return clamp((maxShadowDistance - dist) / fadeWidth, 0.0f, 1.0f);
}

float EE_FUNC_DIRECTIONAL_LIGHT_CASCADE_SHADOW(int lightIndex, float dist, vec3 fragPos, vec3 normal,
                                              float cameraFragDistance) {
	int splitIndex = EE_FUNC_DIRECTIONAL_SHADOW_CASCADE_INDEX(dist);
	if (splitIndex < 0) return 1.0f;

	float shadow = EE_FUNC_DIRECTIONAL_LIGHT_SHADOW(lightIndex, splitIndex, fragPos, normal, cameraFragDistance);
	for (int boundaryIndex = 0; boundaryIndex < 3; boundaryIndex++) {
		float halfWidth = EE_FUNC_DIRECTIONAL_SHADOW_TRANSITION_HALF_WIDTH(boundaryIndex);
		if (halfWidth <= 0.0f) continue;

		float splitDistance = EE_FUNC_DIRECTIONAL_SHADOW_SPLIT_DISTANCE(boundaryIndex);
		if (dist < splitDistance - halfWidth || dist >= splitDistance + halfWidth) continue;

		float previousShadow =
		    EE_FUNC_DIRECTIONAL_LIGHT_SHADOW(lightIndex, boundaryIndex, fragPos, normal, cameraFragDistance);
		float nextShadow =
		    EE_FUNC_DIRECTIONAL_LIGHT_SHADOW(lightIndex, boundaryIndex + 1, fragPos, normal, cameraFragDistance);
		float blend = smoothstep(splitDistance - halfWidth, splitDistance + halfWidth, dist);
		shadow = mix(previousShadow, nextShadow, blend);
		break;
	}

	float distanceFade = EE_FUNC_DIRECTIONAL_SHADOW_DISTANCE_FADE(dist);
	return mix(1.0f, shadow, distanceFade);
}

vec3 EE_FUNC_DIRECTIONAL_SHADOW_CASCADE_COLOR(int cascadeIndex) {
	if (cascadeIndex == 0) return vec3(0.08f, 0.55f, 1.0f);
	if (cascadeIndex == 1) return vec3(0.0f, 0.95f, 0.35f);
	if (cascadeIndex == 2) return vec3(1.0f, 0.85f, 0.05f);
	if (cascadeIndex == 3) return vec3(1.0f, 0.18f, 0.16f);
	return vec3(0.02f);
}

vec4 EE_FUNC_DIRECTIONAL_SHADOW_DEBUG(float dist, vec3 fragPos) {
	int mode = EE_RENDER_INFO.shadow_debug_parameters.x;
	if (mode <= 0) {
		return vec4(0.0f);
	}

	int cascadeIndex = EE_FUNC_DIRECTIONAL_SHADOW_CASCADE_INDEX(dist);
	if (mode == 1) {
		vec3 cascadeColor = EE_FUNC_DIRECTIONAL_SHADOW_CASCADE_COLOR(cascadeIndex);
		for (int boundaryIndex = 0; boundaryIndex < 3; boundaryIndex++) {
			float halfWidth = EE_FUNC_DIRECTIONAL_SHADOW_TRANSITION_HALF_WIDTH(boundaryIndex);
			if (halfWidth <= 0.0f) continue;

			float splitDistance = EE_FUNC_DIRECTIONAL_SHADOW_SPLIT_DISTANCE(boundaryIndex);
			if (dist < splitDistance - halfWidth || dist >= splitDistance + halfWidth) continue;

			float blend = smoothstep(splitDistance - halfWidth, splitDistance + halfWidth, dist);
			cascadeColor = mix(EE_FUNC_DIRECTIONAL_SHADOW_CASCADE_COLOR(boundaryIndex),
			                   EE_FUNC_DIRECTIONAL_SHADOW_CASCADE_COLOR(boundaryIndex + 1), blend);
			break;
		}
		cascadeColor = mix(vec3(0.02f), cascadeColor, EE_FUNC_DIRECTIONAL_SHADOW_DISTANCE_FADE(dist));
		return vec4(cascadeColor, 1.0f);
	}

	if (EE_RENDER_INFO.directional_light_size <= 0) {
		return vec4(1.0f, 0.0f, 1.0f, 1.0f);
	}

	int localLightIndex = clamp(EE_RENDER_INFO.shadow_debug_parameters.z, 0,
	                            max(EE_RENDER_INFO.directional_light_size - 1, 0));
	int lightIndex = EE_CAMERA_INDEX * MAX_DIRECTIONAL_LIGHT_SIZE + localLightIndex;
	DirectionalLight light = EE_DIRECTIONAL_LIGHTS[lightIndex];
	if (light.diffuse.w < 0.5f || light.viewport_x_size <= 0 || light.viewport_y_size <= 0) {
		return vec4(0.25f, 0.0f, 0.25f, 1.0f);
	}

	int splitIndex = clamp(EE_RENDER_INFO.shadow_debug_parameters.y, 0, 3);
	if (cascadeIndex != splitIndex) {
		return vec4(0.02f, 0.02f, 0.02f, 1.0f);
	}

	vec4 fragPosLightSpace = light.light_space_matrix[splitIndex] * vec4(fragPos, 1.0f);
	if (abs(fragPosLightSpace.w) < 0.000001f) {
		return vec4(1.0f, 0.0f, 0.0f, 1.0f);
	}

	vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
	vec2 lightUv = projCoords.xy * 0.5f + 0.5f;
	bool insideLightRect = all(greaterThanEqual(lightUv, vec2(0.0f))) &&
	                       all(lessThanEqual(lightUv, vec2(1.0f))) &&
	                       projCoords.z >= 0.0f && projCoords.z <= 1.0f;
	float insideScale = insideLightRect ? 1.0f : 0.25f;

	if (mode == 2) {
		return vec4(mix(vec3(0.75f, 0.0f, 0.0f), vec3(lightUv, 1.0f), insideScale), 1.0f);
	}
	if (mode == 3) {
		float lightDepth = clamp(projCoords.z, 0.0f, 1.0f);
		vec3 depthColor = vec3(lightDepth);
		if (!insideLightRect) {
			depthColor = mix(vec3(0.75f, 0.0f, 0.0f), depthColor, 0.25f);
		}
		return vec4(depthColor, 1.0f);
	}

	vec2 atlasSize = vec2(textureSize(EE_DIRECTIONAL_LIGHT_SM, 0).xy);
	vec2 viewportOffset = vec2(float(light.viewport_x_offset), float(light.viewport_y_offset));
	vec2 viewportSize = vec2(float(light.viewport_x_size), float(light.viewport_y_size));
	vec2 atlasUv = (lightUv * viewportSize + viewportOffset) / max(atlasSize, vec2(1.0f));
	if (mode == 4) {
		return vec4(mix(vec3(0.75f, 0.0f, 0.0f), vec3(atlasUv, 1.0f), insideScale), 1.0f);
	}

	float worldUnitsPerTexel = (2.0f * light.light_frustum_width[splitIndex]) / max(float(light.viewport_x_size), 1.0f);
	float density = clamp(log2(max(worldUnitsPerTexel, 0.001f)) * 0.125f + 0.5f, 0.0f, 1.0f);
	vec3 densityColor = mix(vec3(0.0f, 0.35f, 1.0f), vec3(1.0f, 0.1f, 0.0f), density);
	vec2 texelUv = fract(lightUv * viewportSize);
	float grid = texelUv.x < 0.04f || texelUv.y < 0.04f ? 1.0f : 0.0f;
	return vec4(mix(densityColor, vec3(1.0f), grid * 0.6f) * insideScale, 1.0f);
}

vec2 EE_FUNC_DIRECTIONAL_SHADOW_ATLAS_UV(DirectionalLight light, vec2 lightUv) {
	vec2 atlasSize = max(vec2(textureSize(EE_DIRECTIONAL_LIGHT_SM, 0).xy), vec2(1.0f));
	vec2 viewportSize = max(vec2(float(light.viewport_x_size), float(light.viewport_y_size)), vec2(1.0f));
	vec2 halfTexel = vec2(0.5f) / viewportSize;
	vec2 clampedLightUv = clamp(lightUv, halfTexel, vec2(1.0f) - halfTexel);
	return (vec2(float(light.viewport_x_offset), float(light.viewport_y_offset)) + clampedLightUv * viewportSize) /
	       atlasSize;
}

float EE_FUNC_DIRECTIONAL_SHADOW_DEPTH(DirectionalLight light, int splitIndex, vec2 lightUv) {
	return texture(EE_DIRECTIONAL_LIGHT_SM, vec3(EE_FUNC_DIRECTIONAL_SHADOW_ATLAS_UV(light, lightUv), splitIndex)).r;
}

float EE_FUNC_DIRECTIONAL_SHADOW_COMPARE(float receiverDepth, float closestDepth) {
	if (closestDepth <= 0.0f) return 1.0f;
	return receiverDepth < closestDepth ? 1.0f : 0.0f;
}

float EE_FUNC_DIRECTIONAL_SHADOW_HARD(DirectionalLight light, int splitIndex, vec3 projCoords) {
	float closestDepth = EE_FUNC_DIRECTIONAL_SHADOW_DEPTH(light, splitIndex, projCoords.xy);
	return EE_FUNC_DIRECTIONAL_SHADOW_COMPARE(projCoords.z, closestDepth);
}

float EE_FUNC_DIRECTIONAL_SHADOW_PCF(DirectionalLight light, int splitIndex, vec3 projCoords, float radiusUv,
                                     int sampleAmount, vec3 randomSeed) {
	if (radiusUv <= 0.0f || sampleAmount <= 1) {
		return EE_FUNC_DIRECTIONAL_SHADOW_HARD(light, splitIndex, projCoords);
	}

	float shadow = 0.0f;
	for (int sampleIndex = 0; sampleIndex < sampleAmount; sampleIndex++)
	{
		vec2 texCoord = projCoords.xy + EE_VOGEL_DISK_SAMPLE(sampleIndex, sampleAmount, randomSeed) * radiusUv;
		float closestDepth = EE_FUNC_DIRECTIONAL_SHADOW_DEPTH(light, splitIndex, texCoord);
		shadow += EE_FUNC_DIRECTIONAL_SHADOW_COMPARE(projCoords.z, closestDepth);
	}
	return shadow / float(sampleAmount);
}

float EE_FUNC_DIRECTIONAL_LIGHT_SHADOW(int i, int splitIndex, vec3 fragPos, vec3 normal, float cameraFragDistance)
{
	DirectionalLight light = EE_DIRECTIONAL_LIGHTS[i];
	vec3 lightDir = light.direction;
	float nDotL = max(dot(normal, -lightDir), 0.0f);
	float shadowTexelSize = light.light_frustum_width[splitIndex] / max(float(light.viewport_x_size), 1.0f);
	float constantBias = light.reserved_parameters.z;
	float slopeBias = light.reserved_parameters.y * (1.0f - nDotL);
	float bias = (constantBias + slopeBias) * shadowTexelSize;
	float normalOffset = light.reserved_parameters.w * shadowTexelSize;

	fragPos = fragPos + normal * normalOffset;
	vec4 fragPosLightSpace = light.light_space_matrix[splitIndex] * vec4(fragPos, 1.0f);
	vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
	if (projCoords.z < 0.0f || projCoords.z > 1.0f) {
		return 1.0f;
	}
	projCoords.x = projCoords.x * 0.5f + 0.5f;
	projCoords.y = projCoords.y * 0.5f + 0.5f;
	if (projCoords.x < 0.0f || projCoords.x > 1.0f || projCoords.y < 0.0f || projCoords.y > 1.0f) {
		return 1.0f;
	}
	projCoords = vec3(projCoords.xy, projCoords.z - bias);

	float radiusUv = max(light.reserved_parameters.x * 100.0f, 0.0f) / max(float(light.viewport_x_size), 1.0f);
	int sampleAmount = clamp(EE_RENDER_INFO.shadow_sample_size, 1, 64);
	return EE_FUNC_DIRECTIONAL_SHADOW_PCF(light, splitIndex, projCoords, radiusUv, sampleAmount, fragPos * 3141);
}

vec2 EE_FUNC_SHADOW_ATLAS_UV(vec2 lightUv, int viewportXOffset, int viewportYOffset, int viewportXSize,
                             int viewportYSize, vec2 atlasSize) {
	vec2 viewportSize = max(vec2(float(viewportXSize), float(viewportYSize)), vec2(1.0f));
	vec2 halfTexel = vec2(0.5f) / viewportSize;
	vec2 clampedLightUv = clamp(lightUv, halfTexel, vec2(1.0f) - halfTexel);
	return (vec2(float(viewportXOffset), float(viewportYOffset)) + clampedLightUv * viewportSize) /
	       max(atlasSize, vec2(1.0f));
}

float EE_FUNC_SPOT_SHADOW_DEPTH(SpotLight light, vec2 lightUv) {
	return texture(EE_SPOT_LIGHT_SM,
	               EE_FUNC_SHADOW_ATLAS_UV(lightUv, light.viewport_x_offset, light.viewport_y_offset,
	                                       light.viewport_x_size, light.viewport_y_size,
	                                       vec2(textureSize(EE_SPOT_LIGHT_SM, 0)))).r;
}

float EE_FUNC_POINT_SHADOW_DEPTH(PointLight light, int slice, vec2 lightUv) {
	return texture(EE_POINT_LIGHT_SM,
	               vec3(EE_FUNC_SHADOW_ATLAS_UV(lightUv, light.viewport_x_offset, light.viewport_y_offset,
	                                            light.viewport_x_size, light.viewport_y_size,
	                                            vec2(textureSize(EE_POINT_LIGHT_SM, 0).xy)), slice)).r;
}

float EE_FUNC_SPOT_LIGHT_SHADOW(int i, vec3 fragPos, float cameraFragDistance) {
	SpotLight light = EE_SPOT_LIGHTS[i];
	vec4 fragPosLightSpace = light.light_space_matrix * vec4(fragPos, 1.0f);
	if (abs(fragPosLightSpace.w) < 0.000001f) {
		return 1.0f;
	}
	vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
	projCoords.z -= light.cutoff_outer_inner_size_bias.w;
	projCoords.x = projCoords.x * 0.5f + 0.5f;
	projCoords.y = projCoords.y * 0.5f + 0.5f;

	if (projCoords.z < 0.0f || projCoords.z > 1.0f || projCoords.x < 0.0f || projCoords.x > 1.0f ||
	    projCoords.y < 0.0f || projCoords.y > 1.0f) {
		return 1.0f;
	}

	float radiusUv = max(light.cutoff_outer_inner_size_bias.z * 100.0f, 0.0f) /
	                 max(float(light.viewport_x_size), 1.0f);
	int sampleAmount = clamp(EE_RENDER_INFO.shadow_sample_size, 1, 64);
	if (radiusUv <= 0.0f || sampleAmount <= 1) {
		return EE_FUNC_DIRECTIONAL_SHADOW_COMPARE(projCoords.z, EE_FUNC_SPOT_SHADOW_DEPTH(light, projCoords.xy));
	}

	float shadow = 0.0f;
	for (int sampleIndex = 0; sampleIndex < sampleAmount; sampleIndex++) {
		vec2 texCoord = projCoords.xy + EE_VOGEL_DISK_SAMPLE(sampleIndex, sampleAmount, fragPos * 3141) * radiusUv;
		float closestDepth = EE_FUNC_SPOT_SHADOW_DEPTH(light, texCoord);
		shadow += EE_FUNC_DIRECTIONAL_SHADOW_COMPARE(projCoords.z, closestDepth);
	}
	return shadow / float(sampleAmount);
}

float EE_FUNC_POINT_LIGHT_SHADOW(int i, vec3 fragPos, float cameraFragDistance)
{
	PointLight light = EE_POINT_LIGHTS[i];
	// get vector between fragment position and light position
	vec3 fragToLight = fragPos - light.position;
	float shadow = 0.0f;
	int slice = 0;
	if (abs(fragToLight.x) >= abs(fragToLight.y) && abs(fragToLight.x) >= abs(fragToLight.z))
	{
		if (fragToLight.x > 0) {
			slice = 0;
		}
		else {
			slice = 1;
		}
	}
	else if (abs(fragToLight.y) >= abs(fragToLight.z)) {
		if (fragToLight.y > 0) {
			slice = 2;
		}
		else {
			slice = 3;
		}
	}
	else {
		if (fragToLight.z > 0) {
			slice = 4;
		}
		else {
			slice = 5;
		}
	}
	vec4 fragPosLightSpace = light.light_space_matrix[slice] * vec4(fragPos, 1.0f);
	if (abs(fragPosLightSpace.w) < 0.000001f) {
		return 1.0f;
	}
	vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
	projCoords.z -= light.reserved_parameters.x;
	projCoords.x = projCoords.x * 0.5f + 0.5f;
	projCoords.y = projCoords.y * 0.5f + 0.5f;

	if (projCoords.z < 0.0f || projCoords.z > 1.0f || projCoords.x < 0.0f || projCoords.x > 1.0f ||
	    projCoords.y < 0.0f || projCoords.y > 1.0f) {
		return 1.0f;
	}

	float radiusUv = max(light.reserved_parameters.y * 100.0f, 0.0f) / max(float(light.viewport_x_size), 1.0f);
	int sampleAmount = clamp(EE_RENDER_INFO.shadow_sample_size, 1, 64);
	if (radiusUv <= 0.0f || sampleAmount <= 1) {
		return EE_FUNC_DIRECTIONAL_SHADOW_COMPARE(projCoords.z, EE_FUNC_POINT_SHADOW_DEPTH(light, slice, projCoords.xy));
	}

	for (int sampleIndex = 0; sampleIndex < sampleAmount; sampleIndex++) {
		vec2 texCoord = projCoords.xy + EE_VOGEL_DISK_SAMPLE(sampleIndex, sampleAmount, fragPos * 3141) * radiusUv;
		float closestDepth = EE_FUNC_POINT_SHADOW_DEPTH(light, slice, texCoord);
		shadow += EE_FUNC_DIRECTIONAL_SHADOW_COMPARE(projCoords.z, closestDepth);
	}
	return shadow / float(sampleAmount);
}
