#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.glsl"
#include "Basic.glsl"
#include "Lighting.glsl"

precision highp float;

layout (location = 0) in VS_OUT {
    vec2 TexCoord;
} fs_in;

layout(set = EE_PER_PASS_SET, binding = 17) uniform sampler2D inDepth;
layout(set = EE_PER_PASS_SET, binding = 18) uniform sampler2D inNormal;
layout(set = EE_PER_PASS_SET, binding = 19) uniform sampler2D inMaterial;

layout (location = 0) out vec4 FragColor;

void main()
{
    // --- Common data fetches (reuse components instead of resampling) ---
    float ndcDepth = texture(inDepth, fs_in.TexCoord).x;

    vec4 matSample   = texture(inMaterial, fs_in.TexCoord);
    vec2 materialTexCoord = matSample.xy;
    int  material_index   = int(round(matSample.z));
    int  info_index       = int(round(matSample.w));

    vec4 normalSample = texture(inNormal, fs_in.TexCoord);
    vec3 normal       = normalSample.xyz;
    int  instance_index = int(round(normalSample.a));

    bool instance_selected = (info_index & 1) == 1; // faster than % 2

    vec3 fragPos        = EE_DEPTH_TO_WORLD_POS(EE_CAMERA_INDEX, fs_in.TexCoord, ndcDepth);
    vec3 cameraPosition = EE_CAMERA_POSITION(EE_CAMERA_INDEX);
    vec3 skyColor       = EE_SKY_COLOR(fragPos - cameraPosition);

    // Precompute texel offset once; used in both “sky” and “solid” paths
    vec2 texelSize  = vec2(textureSize(inMaterial, 0));
    vec2 texOffset  = 1.0 / texelSize;

    // --------------------------------------------------------------------
    // Background (depth == 1.0) path
    // --------------------------------------------------------------------
    if (ndcDepth == 1.0) {
        if (!instance_selected && EE_INSTANCE_INDEX == 1) {
            bool foundNeighbor = false;

            // 7x7 neighborhood search for selected instances
            for (int i = -3; i <= 3 && !foundNeighbor; ++i) {
                for (int j = -3; j <= 3; ++j) {
                    int info_index_local = int(round(
                        texture(inMaterial,
                                fs_in.TexCoord + vec2(texOffset.x * float(i),
                                                      texOffset.y * float(j))).w));
                    if ((info_index_local & 1) == 1) {
                        FragColor = mix(vec4(1.0, 0.75, 0.0, 1.0),
                                        vec4(skyColor, 1.0),
                                        0.1);
                        foundNeighbor = true;
                        break;
                    }
                }
            }

            if (!foundNeighbor) {
                FragColor = mix(vec4(0.5, 0.5, 0.5, 1.0),
                                vec4(skyColor, 1.0),
                                float(EE_LIGHT_SPLIT_INDEX) / 256.0);
            }
        } else {
            FragColor = vec4(skyColor, 1.0);
        }
        return;
    }

    // --------------------------------------------------------------------
    // Opaque / solid fragment path
    // --------------------------------------------------------------------
    float depth = EE_LINEARIZE_DEPTH(EE_CAMERA_INDEX, ndcDepth);

    MaterialProperties materialProperties = EE_MATERIAL_PROPERTIES[material_index];

    float roughness = EE_SAMPLE_TEXTURE_2D(materialProperties.roughness_map_index, materialTexCoord, vec4(materialProperties.roughness, 0, 0, 0)).r;
	float metallic = EE_SAMPLE_TEXTURE_2D(materialProperties.metallic_map_index, materialTexCoord, vec4(materialProperties.metallic, 0, 0, 0)).r;
	float emission = materialProperties.emission;
	float ao = EE_SAMPLE_TEXTURE_2D(materialProperties.ao_texture_index, materialTexCoord, vec4(materialProperties.ambient_occulusion, 0, 0, 0)).r;
	vec4 albedo = EE_SAMPLE_TEXTURE_2D(materialProperties.albedo_map_index, materialTexCoord, materialProperties.albedo);

    // --------------------------------------------------------------------
    // Debug visualization (branchless override, but keeps default behavior)
    // --------------------------------------------------------------------
    vec3 base  = matSample.rgb;
    vec3 kMat  = abs(EE_UNIFORM_KERNEL[material_index  % MAX_KERNEL_AMOUNT].xyz);
    vec3 kInst = abs(EE_UNIFORM_KERNEL[instance_index  % MAX_KERNEL_AMOUNT].xyz);
    vec3 kInfo = abs(EE_UNIFORM_KERNEL[info_index      % MAX_KERNEL_AMOUNT].xyz);

    float dv   = float(EE_RENDER_INFO.debug_visualization);

    float is0 = float(dv == 0.0 && info_index > 1);
    float is1 = float(dv == 1.0);
    float is2 = float(dv == 2.0);
    float is3 = float(dv == 3.0);

    float anyDebug = clamp(is0 + is1 + is2 + is3, 0.0, 1.0);

    vec3 debugColor =
          base  * is0 +
          kMat  * is1 +
          kInst * is2 +
          kInfo * is3;

    // Only override albedo if a debug mode is active; otherwise keep PBR albedo
    vec3 finalAlbedoRGB = mix(albedo.rgb, debugColor, anyDebug);
    albedo = vec4(finalAlbedoRGB, albedo.a);

    // --------------------------------------------------------------------
    // Lighting
    // --------------------------------------------------------------------
    vec3 viewDir = normalize(cameraPosition - fragPos);
    bool receiveShadow = true;

    vec3 F0 = vec3(0.04);
    F0 = mix(F0, albedo.rgb, metallic);

    vec3 direct  = EE_FUNC_CALCULATE_LIGHTS(receiveShadow,
                                            albedo.rgb, 1.0, depth,
                                            normal, viewDir, fragPos,
                                            metallic, roughness, F0);
    vec3 ambient = EE_FUNC_CALCULATE_ENVIRONMENTAL_LIGHT(albedo.rgb,
                                                         normal, viewDir,
                                                         metallic, roughness, F0) +
                   EE_FUNC_CALCULATE_DDGI_DIFFUSE(albedo.rgb, normal, viewDir, fragPos);
    vec3 color = direct + emission * normalize(albedo.rgb) + ambient * ao;

    // --------------------------------------------------------------------
    // Selection / neighborhood highlight
    // --------------------------------------------------------------------
    vec4 outputColor;

    if (!instance_selected && EE_INSTANCE_INDEX == 1) {
        bool foundNeighbor = false;

        for (int i = -3; i <= 3 && !foundNeighbor; ++i) {
            for (int j = -3; j <= 3; ++j) {
                int info_index_local = int(round(
                    texture(inMaterial,
                            fs_in.TexCoord + vec2(texOffset.x * float(i),
                                                  texOffset.y * float(j))).w));
                if ((info_index_local & 1) == 1) {
                    outputColor = mix(vec4(1.0, 0.75, 0.0, 1.0),
                                      vec4(color, 1.0),
                                      0.1);
                    foundNeighbor = true;
                    break;
                }
            }
        }

        if (!foundNeighbor) {
            outputColor = mix(vec4(0.5, 0.5, 0.5, 1.0),
                              vec4(color, 1.0),
                              float(EE_LIGHT_SPLIT_INDEX) / 256.0);
        }
    } else {
        outputColor = vec4(color, 1.0);
    }

    // --------------------------------------------------------------------
    // Distance-based sky fade
    // --------------------------------------------------------------------
    float fade_ratio = EE_CAMERA_FADE_RATIO(EE_CAMERA_INDEX);
    float camFar     = EE_CAMERA_FAR(EE_CAMERA_INDEX);
    float fadeStart  = camFar * fade_ratio;

    if (depth > fadeStart) {
        float t = (depth - fadeStart) / (camFar * (1.0 - fade_ratio));
        outputColor.rgb = mix(outputColor.rgb, skyColor, clamp(t, 0.0, 1.0));
    }

    FragColor = outputColor;
}
