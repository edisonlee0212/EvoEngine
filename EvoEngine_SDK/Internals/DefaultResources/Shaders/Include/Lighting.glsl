layout(set = EE_PER_GROUP_SET, binding = 15) uniform sampler2DArray EE_DIRECTIONAL_LIGHT_SM;
layout(set = EE_PER_GROUP_SET, binding = 16) uniform sampler2DArray EE_POINT_LIGHT_SM;
layout(set = EE_PER_GROUP_SET, binding = 17) uniform sampler2D EE_SPOT_LIGHT_SM;

vec3 EE_SKY_COLOR(vec3 direction) {
	Camera camera = EE_CAMERAS[EE_CAMERA_INDEX];
	return pow(camera.use_clear_color == 1 ?
		pow(camera.clear_color.xyz * camera.clear_color.w, vec3(EE_RENDER_INFO.gamma))
		: pow(texture(EE_CUBEMAPS[camera.skybox_tex_index], normalize(direction)).rgb, vec3(1.0 / EE_ENVIRONMENT.gamma)), vec3(1.0 / EE_RENDER_INFO.gamma)) * pow(camera.clear_color.w, EE_RENDER_INFO.gamma);
}

const float PI = 3.14159265359;
// ----------------------------------------------------------------------------
float EE_FUNC_DISTRIBUTION_GGX(vec3 N, vec3 H, float roughness)
{
	float a = roughness * roughness;
	float a2 = a * a;
	float NdotH = max(dot(N, H), 0.0);
	float NdotH2 = NdotH * NdotH;

	float nom = a2;
	float denom = (NdotH2 * (a2 - 1.0) + 1.0);
	denom = PI * denom * denom;

	return nom / max(denom, 0.001); // prevent divide by zero for roughness=0.0 and NdotH=1.0
}
// ----------------------------------------------------------------------------
float EE_FUNC_GEOMETRY_SCHLICK_GGX(float NdotV, float roughness)
{
	float r = (roughness + 1.0);
	float k = (r * r) / 8.0;

	float nom = NdotV;
	float denom = NdotV * (1.0 - k) + k;

	return nom / denom;
}
// ----------------------------------------------------------------------------
float EE_FUNC_GEOMETRY_SMITH(vec3 N, vec3 V, vec3 L, float roughness)
{
	float NdotV = max(dot(N, V), 0.0);
	float NdotL = max(dot(N, L), 0.0);
	float ggx2 = EE_FUNC_GEOMETRY_SCHLICK_GGX(NdotV, roughness);
	float ggx1 = EE_FUNC_GEOMETRY_SCHLICK_GGX(NdotL, roughness);

	return ggx1 * ggx2;
}
// ----------------------------------------------------------------------------
vec3 EE_FUNC_FRESNEL_SCHLICK(float cosTheta, vec3 F0)
{
	return F0 + (1.0 - F0) * pow(max(1.0 - cosTheta, 0.0), 5.0);
}

// ----------------------------------------------------------------------------
vec3 EE_FUNC_FRESNEL_SCHLICK_ROUGHNESS(float cosTheta, vec3 F0, float roughness)
{
	return F0 + (max(vec3(1.0 - roughness), F0) - F0) * pow(max(1.0 - cosTheta, 0.0), 5.0);
}

vec3 EE_FUNC_CALCULATE_LIGHTS(in bool calculateShadow, vec3 albedo, float specular, float dist, vec3 normal, vec3 viewDir, vec3 fragPos, float metallic, float roughness, vec3 F0);
vec3 EE_FUNC_DIRECTIONAL_LIGHT(vec3 albedo, float specular, int i, vec3 normal, vec3 viewDir, float metallic, float roughness, vec3 F0);
vec3 EE_FUNC_POINT_LIGHT(vec3 albedo, float specular, int i, vec3 normal, vec3 fragPos, vec3 viewDir, float metallic, float roughness, vec3 F0);
vec3 EE_FUNC_SPOT_LIGHT(vec3 albedo, float specular, int i, vec3 normal, vec3 fragPos, vec3 viewDir, float metallic, float roughness, vec3 F0);
float EE_FUNC_DIRECTIONAL_LIGHT_SHADOW(int i, int splitIndex, vec3 fragPos, vec3 normal, float cameraFragDistance);
float EE_FUNC_POINT_LIGHT_SHADOW(int i, vec3 fragPos, float cameraFragDistance);
float EE_FUNC_SPOT_LIGHT_SHADOW(int i, vec3 fragPos, float cameraFragDistance);
float EE_LINEARIZE_DEPTH(float ndcDepth);
vec3 EE_DEPTH_TO_CLIP_POS(vec2 tex_coords, float ndcDepth);
vec3 EE_DEPTH_TO_WORLD_POS(vec2 tex_coords, float ndcDepth);
vec3 EE_DEPTH_TO_VIEW_POS(vec2 tex_coords, float ndcDepth);

vec3 EE_FUNC_CALCULATE_ENVIRONMENTAL_LIGHT(vec3 albedo, vec3 normal, vec3 viewDir, float metallic, float roughness, vec3 F0)
{
	// ambient lighting (we now use IBL as the ambient term)
	vec3 F = EE_FUNC_FRESNEL_SCHLICK_ROUGHNESS(max(dot(normal, viewDir), 0.0), F0, roughness);
	vec3 R = reflect(-viewDir, normal);
	vec3 kS = F;
	vec3 kD = 1.0 - kS;
	kD *= 1.0 - metallic;

	vec3 irradiance = EE_ENVIRONMENT.background_color.w == 1.0 ? pow(EE_ENVIRONMENT.background_color.xyz, vec3(EE_RENDER_INFO.gamma)) : pow(texture(EE_CUBEMAPS[EE_CAMERAS[EE_CAMERA_INDEX].irradiance_map_index], normal).rgb, vec3(1.0 / EE_ENVIRONMENT.gamma));
	vec3 diffuse = irradiance * albedo;

	// sample both the pre-filter map and the BRDF lut and combine them together as per the Split-Sum approximation to get the IBL specular part.
	const float MAX_REFLECTION_LOD = 4.0;
	vec3 prefilteredColor = EE_ENVIRONMENT.background_color.w == 1.0 ? pow(EE_ENVIRONMENT.background_color.xyz, vec3(EE_RENDER_INFO.gamma)) : pow(textureLod(EE_CUBEMAPS[EE_CAMERAS[EE_CAMERA_INDEX].prefiltered_map_index], R, roughness * MAX_REFLECTION_LOD).rgb, vec3(1.0 / EE_ENVIRONMENT.gamma));
	vec2 brdf = texture(EE_TEXTURE_2DS[EE_RENDER_INFO.brdf_lut_map_index], vec2(max(dot(normal, viewDir), 0.0), roughness)).rg;
	vec3 specular = prefilteredColor * (F * brdf.x + brdf.y);
	vec3 ambient = (kD * diffuse + specular) * pow(EE_ENVIRONMENT.light_intensity, EE_RENDER_INFO.gamma);
	return ambient;
}

vec3 EE_FUNC_CALCULATE_LIGHTS(bool calculateShadow, vec3 albedo, float specular, float dist, vec3 normal, vec3 viewDir, vec3 fragPos, float metallic, float roughness, vec3 F0) {
	vec3 result = vec3(0.0, 0.0, 0.0);
	vec3 fragToCamera = fragPos - EE_CAMERA_POSITION();
	float cameraFragDistance = length(fragToCamera);
	// phase 1: directional lighting
	for (int i = 0; i < EE_RENDER_INFO.directional_light_size; i++) {
		float shadow = 1.0;
		int lightIndex = EE_CAMERA_INDEX * MAX_DIRECTIONAL_LIGHT_SIZE + i;
		if (calculateShadow && EE_DIRECTIONAL_LIGHTS[lightIndex].diffuse.w == 1.0) {
			int split = 0;
			if (dist < EE_RENDER_INFO.shadow_split_0 - EE_RENDER_INFO.shadow_split_0 * EE_RENDER_INFO.shadow_seam_fix) {
				shadow = EE_FUNC_DIRECTIONAL_LIGHT_SHADOW(lightIndex, 0, fragPos, normal, cameraFragDistance);
			}
			else if (dist < EE_RENDER_INFO.shadow_split_0) {
				//Blend between split 1 & 2
				shadow = EE_FUNC_DIRECTIONAL_LIGHT_SHADOW(lightIndex, 0, fragPos, normal, cameraFragDistance);
				float nextLevel = EE_FUNC_DIRECTIONAL_LIGHT_SHADOW(lightIndex, 1, fragPos, normal, cameraFragDistance);
				shadow = (nextLevel * (dist - (EE_RENDER_INFO.shadow_split_0 - EE_RENDER_INFO.shadow_split_0 * EE_RENDER_INFO.shadow_seam_fix)) + shadow * (EE_RENDER_INFO.shadow_split_0 - dist)) / (EE_RENDER_INFO.shadow_split_0 * EE_RENDER_INFO.shadow_seam_fix);
			}
			else if (dist < EE_RENDER_INFO.shadow_split_1 - EE_RENDER_INFO.shadow_split_1 * EE_RENDER_INFO.shadow_seam_fix) {
				shadow = EE_FUNC_DIRECTIONAL_LIGHT_SHADOW(lightIndex, 1, fragPos, normal, cameraFragDistance);
			}
			else if (dist < EE_RENDER_INFO.shadow_split_1) {
				//Blend between split 2 & 3
				shadow = EE_FUNC_DIRECTIONAL_LIGHT_SHADOW(lightIndex, 1, fragPos, normal, cameraFragDistance);
				float nextLevel = EE_FUNC_DIRECTIONAL_LIGHT_SHADOW(lightIndex, 2, fragPos, normal, cameraFragDistance);
				shadow = (nextLevel * (dist - (EE_RENDER_INFO.shadow_split_1 - EE_RENDER_INFO.shadow_split_1 * EE_RENDER_INFO.shadow_seam_fix)) + shadow * (EE_RENDER_INFO.shadow_split_1 - dist)) / (EE_RENDER_INFO.shadow_split_1 * EE_RENDER_INFO.shadow_seam_fix);
			}
			else if (dist < EE_RENDER_INFO.shadow_split_2 - EE_RENDER_INFO.shadow_split_2 * EE_RENDER_INFO.shadow_seam_fix) {
				shadow = EE_FUNC_DIRECTIONAL_LIGHT_SHADOW(lightIndex, 2, fragPos, normal, cameraFragDistance);
			}
			else if (dist < EE_RENDER_INFO.shadow_split_2) {
				//Blend between split 3 & 4
				shadow = EE_FUNC_DIRECTIONAL_LIGHT_SHADOW(lightIndex, 2, fragPos, normal, cameraFragDistance);
				float nextLevel = EE_FUNC_DIRECTIONAL_LIGHT_SHADOW(lightIndex, 3, fragPos, normal, cameraFragDistance);
				shadow = (nextLevel * (dist - (EE_RENDER_INFO.shadow_split_2 - EE_RENDER_INFO.shadow_split_2 * EE_RENDER_INFO.shadow_seam_fix)) + shadow * (EE_RENDER_INFO.shadow_split_2 - dist)) / (EE_RENDER_INFO.shadow_split_2 * EE_RENDER_INFO.shadow_seam_fix);
			}
			else if (dist < EE_RENDER_INFO.shadow_split_3) {
				shadow = EE_FUNC_DIRECTIONAL_LIGHT_SHADOW(lightIndex, 3, fragPos, normal, cameraFragDistance);
			}
			else {
				shadow = 1.0;
			}
		}
		result += EE_FUNC_DIRECTIONAL_LIGHT(albedo, specular, lightIndex, normal, viewDir, metallic, roughness, F0) * shadow;
	}
	// phase 2: point lights
	for (int i = 0; i < EE_RENDER_INFO.point_light_size; i++) {
		float shadow = 1.0;
		if (calculateShadow && EE_POINT_LIGHTS[i].diffuse.w == 1.0) {
			shadow = EE_FUNC_POINT_LIGHT_SHADOW(i, fragPos, cameraFragDistance);
		}
		result += EE_FUNC_POINT_LIGHT(albedo, specular, i, normal, fragPos, viewDir, metallic, roughness, F0) * shadow;
	}
	// phase 3: spot light
	for (int i = 0; i < EE_RENDER_INFO.spot_light_size; i++) {
		float shadow = 1.0;
		if (calculateShadow && EE_SPOT_LIGHTS[i].diffuse.w == 1.0) {
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
	vec3 radiance = pow(light.diffuse.xyz, vec3(EE_RENDER_INFO.gamma));
	float normalDF = EE_FUNC_DISTRIBUTION_GGX(normal, H, roughness);
	float G = EE_FUNC_GEOMETRY_SMITH(normal, viewDir, lightDir, roughness);
	vec3 F = EE_FUNC_FRESNEL_SCHLICK(clamp(dot(H, viewDir), 0.0, 1.0), F0);
	vec3 nominator = normalDF * G * F;
	float denominator = 4 * max(dot(normal, viewDir), 0.0) * max(dot(normal, lightDir), 0.0);
	vec3 spec = nominator / max(denominator, 0.001) * specular;
	vec3 kS = F;
	vec3 kD = vec3(1.0) - kS;
	kD *= 1.0 - metallic;
	float NdotL = max(dot(normal, lightDir), 0.0);
	return (kD * albedo / PI + spec) * radiance * NdotL;
}

// calculates the color when using a point light.
vec3 EE_FUNC_POINT_LIGHT(vec3 albedo, float specular, int i, vec3 normal, vec3 fragPos, vec3 viewDir, float metallic, float roughness, vec3 F0)
{
	PointLight light = EE_POINT_LIGHTS[i];
	vec3 lightDir = normalize(light.position - fragPos);
	vec3 H = normalize(viewDir + lightDir);
	float distance = length(light.position - fragPos);
	float attenuation = 1.0 / (light.constant_linear_quadratic_far.x + light.constant_linear_quadratic_far.y * distance + light.constant_linear_quadratic_far.z * (distance * distance));
	vec3 radiance = pow(light.diffuse.xyz, vec3(EE_RENDER_INFO.gamma)) * attenuation;
	float normalDF = EE_FUNC_DISTRIBUTION_GGX(normal, H, roughness);
	float G = EE_FUNC_GEOMETRY_SMITH(normal, viewDir, lightDir, roughness);
	vec3 F = EE_FUNC_FRESNEL_SCHLICK(clamp(dot(H, viewDir), 0.0, 1.0), F0);
	vec3 nominator = normalDF * G * F;
	float denominator = 4 * max(dot(normal, viewDir), 0.0) * max(dot(normal, lightDir), 0.0);
	vec3 spec = nominator / max(denominator, 0.001) * specular;
	vec3 kS = F;
	vec3 kD = vec3(1.0) - kS;
	kD *= 1.0 - metallic;
	float NdotL = max(dot(normal, lightDir), 0.0);
	return (kD * albedo / PI + spec) * radiance * NdotL;

}

// calculates the color when using a spot light.
vec3 EE_FUNC_SPOT_LIGHT(vec3 albedo, float specular, int i, vec3 normal, vec3 fragPos, vec3 viewDir, float metallic, float roughness, vec3 F0)
{
	SpotLight light = EE_SPOT_LIGHTS[i];
	vec3 lightDir = normalize(light.position - fragPos);
	vec3 H = normalize(viewDir + lightDir);
	float distance = length(light.position - fragPos);
	float attenuation = 1.0 / (light.constant_linear_quadratic_far.x + light.constant_linear_quadratic_far.y * distance + light.constant_linear_quadratic_far.z * (distance * distance));
	// spotlight intensity
	float theta = dot(lightDir, normalize(-light.direction));
	float epsilon = light.cutoff_outer_inner_size_bias.x - light.cutoff_outer_inner_size_bias.y;
	float intensity = clamp((theta - light.cutoff_outer_inner_size_bias.y) / epsilon, 0.0, 1.0);

	vec3 radiance = pow(light.diffuse.xyz, vec3(EE_RENDER_INFO.gamma)) * attenuation * intensity;
	float normalDF = EE_FUNC_DISTRIBUTION_GGX(normal, H, roughness);
	float G = EE_FUNC_GEOMETRY_SMITH(normal, viewDir, lightDir, roughness);
	vec3 F = EE_FUNC_FRESNEL_SCHLICK(clamp(dot(H, viewDir), 0.0, 1.0), F0);
	vec3 nominator = normalDF * G * F;
	float denominator = 4 * max(dot(normal, viewDir), 0.0) * max(dot(normal, lightDir), 0.0);
	vec3 spec = nominator / max(denominator, 0.001) * specular;
	vec3 kS = F;
	vec3 kD = vec3(1.0) - kS;
	kD *= 1.0 - metallic;
	float NdotL = max(dot(normal, lightDir), 0.0);
	return (kD * albedo / PI + spec) * radiance * NdotL;
}

vec2 VogelDiskSample(int sampleIndex, int sampleCount, float phi)
{
	float goldenAngle = 2.4;
	float r = sqrt(float(sampleIndex + 0.5)) / sqrt(float(sampleCount));
	float theta = goldenAngle * sampleIndex + phi;
	return r * vec2(cos(theta), sin(theta));
}

float InterleavedGradientNoise(vec3 fragCoords) {
	vec3 magic = vec3(0.06711056, 0.00583715, 52.9829189);
	return fract(dot(fragCoords, magic));
}

float EE_FUNC_DIRECTIONAL_LIGHT_SHADOW(int i, int splitIndex, vec3 fragPos, vec3 normal, float cameraFragDistance)
{
	DirectionalLight light = EE_DIRECTIONAL_LIGHTS[i];
	vec3 lightDir = light.direction;
	if (dot(lightDir, normal) > -0.02) return 1.0;
	float bias = light.reserved_parameters.z * light.light_frustum_width[splitIndex] / light.viewport_x_size;
	float normalOffset = light.reserved_parameters.w * light.light_frustum_width[splitIndex] / light.viewport_x_size;

	fragPos = fragPos + normal * normalOffset;
	vec4 fragPosLightSpace = light.light_space_matrix[splitIndex] * vec4(fragPos, 1.0);
	// perform perspective divide
	vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
	//
	if (projCoords.z > 1.0) {
		return 0.0;
	}
	// transform to [0,1] range
	projCoords.x = projCoords.x * 0.5 + 0.5;
	projCoords.y = projCoords.y * 0.5 + 0.5;

	// get depth of current fragment from light's perspective
	projCoords = vec3(projCoords.xy, projCoords.z - bias);
	float shadow = 0.0;
	float lightSize = light.reserved_parameters.x;

	int blockers = 0;
	float avgDistance = 0;

	int sampleAmount = EE_RENDER_INFO.pcss_blocker_search;
	float sampleWidth = lightSize / light.light_frustum_width[splitIndex] / sampleAmount;

	float texScale = float(light.viewport_x_size) / float(textureSize(EE_DIRECTIONAL_LIGHT_SM, 0).x);
	vec2 texBase = vec2(float(light.viewport_x_offset) / float(textureSize(EE_DIRECTIONAL_LIGHT_SM, 0).y), float(light.viewport_y_offset) / float(textureSize(EE_DIRECTIONAL_LIGHT_SM, 0).y));

	for (int i = -sampleAmount; i <= sampleAmount; i++)
	{
		for (int j = -sampleAmount; j <= sampleAmount; j++) {
			vec2 tex_coord = projCoords.xy + vec2(i, j) * sampleWidth;
			float closestDepth = texture(EE_DIRECTIONAL_LIGHT_SM, vec3(tex_coord * texScale + texBase, splitIndex)).r;
			int tf = int(closestDepth != 0.0 && projCoords.z > closestDepth);
			avgDistance += closestDepth * tf;
			blockers += tf;
		}
	}
	if (blockers == 0) return 1.0;
	float blockerDistance = avgDistance / blockers;
	float penumbraWidth = (projCoords.z - blockerDistance) / blockerDistance * lightSize;
	float texelSize = penumbraWidth * light.reserved_parameters.x / light.light_frustum_width[splitIndex] * light.light_frustum_distance[splitIndex] / 100.0;

	int shadowCount = 0;
	float distanceFactor = (EE_RENDER_INFO.shadow_split_3 - cameraFragDistance) / EE_RENDER_INFO.shadow_split_3;
	sampleAmount = int(EE_RENDER_INFO.shadow_sample_size * distanceFactor * distanceFactor);
	for (int i = 0; i < sampleAmount; i++)
	{
		vec2 tex_coord = projCoords.xy + VogelDiskSample(i, sampleAmount, InterleavedGradientNoise(fragPos * 3141)) * (texelSize + 0.001);
		float closestDepth = texture(EE_DIRECTIONAL_LIGHT_SM, vec3(tex_coord * texScale + texBase, splitIndex)).r;
		if (closestDepth == 0.0) continue;
		shadow += projCoords.z < closestDepth ? 1.0 : 0.0;
	}
	shadow /= sampleAmount;
	return shadow;
}

float EE_FUNC_SPOT_LIGHT_SHADOW(int i, vec3 fragPos, float cameraFragDistance) {
	SpotLight light = EE_SPOT_LIGHTS[i];
	vec4 fragPosLightSpace = light.light_space_matrix * vec4(fragPos, 1.0);
	fragPosLightSpace.z -= light.cutoff_outer_inner_size_bias.w;
	vec3 projCoords = (fragPosLightSpace.xyz) / fragPosLightSpace.w;
	
	projCoords.x = projCoords.x * 0.5 + 0.5;
	projCoords.y = projCoords.y * 0.5 + 0.5;

	float texScale = float(light.viewport_x_size) / float(textureSize(EE_SPOT_LIGHT_SM, 0).x);
	vec2 texBase = vec2(float(light.viewport_x_offset) / float(textureSize(EE_SPOT_LIGHT_SM, 0).y), float(light.viewport_y_offset) / float(textureSize(EE_SPOT_LIGHT_SM, 0).y));

	//Blocker Search
	int sampleAmount = EE_RENDER_INFO.pcss_blocker_search;
	float lightSize = light.cutoff_outer_inner_size_bias.z * projCoords.z / light.cutoff_outer_inner_size_bias.y;
	float blockers = 0;
	float avgDistance = 0;
	float sampleWidth = lightSize / sampleAmount;
	for (int i = -sampleAmount; i <= sampleAmount; i++)
	{
		for (int j = -sampleAmount; j <= sampleAmount; j++) {
			vec2 tex_coord = projCoords.xy + vec2(i, j) * sampleWidth;
			float closestDepth = texture(EE_SPOT_LIGHT_SM, vec2(tex_coord * texScale + texBase)).r;
			int tf = int(closestDepth != 0.0 && projCoords.z > closestDepth);
			avgDistance += closestDepth * tf;
			blockers += tf;
		}
	}
	if (blockers == 0) return 1.0;
	float blockerDistance = avgDistance / blockers;
	float penumbraWidth = (projCoords.z - blockerDistance) / blockerDistance * lightSize;
	//End search
	float distanceFactor = (EE_RENDER_INFO.shadow_split_3 - cameraFragDistance) / EE_RENDER_INFO.shadow_split_3;
	sampleAmount = int(EE_RENDER_INFO.shadow_sample_size * distanceFactor * distanceFactor);
	float shadow = 0.0;
	for (int i = 0; i < sampleAmount; i++)
	{
		vec2 tex_coord = projCoords.xy + VogelDiskSample(i, sampleAmount, InterleavedGradientNoise(fragPos * 3141)) * (penumbraWidth + 0.001);
		float closestDepth = texture(EE_SPOT_LIGHT_SM, vec2(tex_coord * texScale + texBase)).r;
		if (closestDepth == 0.0) continue;
		shadow += projCoords.z < closestDepth ? 1.0 : 0.0;
	}
	shadow /= sampleAmount;
	return shadow;
}

float EE_FUNC_POINT_LIGHT_SHADOW(int i, vec3 fragPos, float cameraFragDistance)
{
	PointLight light = EE_POINT_LIGHTS[i];
	// get vector between fragment position and light position
	vec3 fragToLight = fragPos - light.position;
	float shadow = 0.0;
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
	vec4 fragPosLightSpace = light.light_space_matrix[slice] * vec4(fragPos, 1.0);
	fragPosLightSpace.z -= light.reserved_parameters.x;
	vec3 projCoords = (fragPosLightSpace.xyz) / fragPosLightSpace.w;
	
	projCoords.x = projCoords.x * 0.5 + 0.5;
	projCoords.y = projCoords.y * 0.5 + 0.5;

	float texScale = float(light.viewport_x_size) / float(textureSize(EE_POINT_LIGHT_SM, 0).x);
	vec2 texBase = vec2(float(light.viewport_x_offset) / float(textureSize(EE_POINT_LIGHT_SM, 0).y), float(light.viewport_y_offset) / float(textureSize(EE_POINT_LIGHT_SM, 0).y));

	//Blocker Search
	int sampleAmount = EE_RENDER_INFO.pcss_blocker_search;
	float lightSize = light.reserved_parameters.y * projCoords.z;
	float blockers = 0;
	float avgDistance = 0;
	float sampleWidth = lightSize / sampleAmount;
	for (int i = -sampleAmount; i <= sampleAmount; i++)
	{
		for (int j = -sampleAmount; j <= sampleAmount; j++) {
			vec2 tex_coord = projCoords.xy + vec2(i, j) * sampleWidth;
			tex_coord.x = clamp(tex_coord.x, 1.0 / float(light.viewport_x_size), 1.0 - 1.0 / float(light.viewport_x_size));
			tex_coord.y = clamp(tex_coord.y, 1.0 / float(light.viewport_x_size), 1.0 - 1.0 / float(light.viewport_x_size));
			float closestDepth = texture(EE_POINT_LIGHT_SM, vec3(tex_coord * texScale + texBase, slice)).r;
			int tf = int(closestDepth != 0.0 && projCoords.z > closestDepth);
			avgDistance += closestDepth * tf;
			blockers += tf;
		}
	}

	if (blockers == 0) return 1.0;
	float blockerDistance = avgDistance / blockers;
	float penumbraWidth = (projCoords.z - blockerDistance) / blockerDistance * lightSize;
	//End search
	float distanceFactor = (EE_RENDER_INFO.shadow_split_3 - cameraFragDistance) / EE_RENDER_INFO.shadow_split_3;
	sampleAmount = int(EE_RENDER_INFO.shadow_sample_size * distanceFactor * distanceFactor);
	for (int i = 0; i < sampleAmount; i++)
	{
		vec2 tex_coord = projCoords.xy + VogelDiskSample(i, sampleAmount, InterleavedGradientNoise(fragPos * 3141)) * (penumbraWidth + 0.001);
		tex_coord.x = clamp(tex_coord.x, 1.0 / float(light.viewport_x_size), 1.0 - 1.0 / float(light.viewport_x_size));
		tex_coord.y = clamp(tex_coord.y, 1.0 / float(light.viewport_x_size), 1.0 - 1.0 / float(light.viewport_x_size));
		float closestDepth = texture(EE_POINT_LIGHT_SM, vec3(tex_coord * texScale + texBase, slice)).r;
		if (closestDepth == 0.0) continue;
		shadow += projCoords.z < closestDepth ? 1.0 : 0.0;
	}
	shadow /= sampleAmount;
	return shadow;
}

