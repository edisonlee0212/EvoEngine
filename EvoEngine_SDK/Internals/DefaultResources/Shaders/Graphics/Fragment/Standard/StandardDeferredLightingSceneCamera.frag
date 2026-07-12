#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.glsl"
#include "Basic.glsl"
#include "Lighting.glsl"

precision highp float;

layout (location = 0) in VS_OUT {
    vec2 TexCoord;
} fs_in;

layout(set = EE_PER_PASS_SET, binding = 17) uniform sampler2D inDepth;
layout(set = EE_PER_PASS_SET, binding = 20) uniform sampler2D inBaseColorAO;
layout(set = EE_PER_PASS_SET, binding = 21) uniform sampler2D inNormalRoughness;
layout(set = EE_PER_PASS_SET, binding = 22) uniform sampler2D inPbrFlags;
layout(set = EE_PER_PASS_SET, binding = 23) uniform sampler2D inEmissive;
layout(set = EE_PER_PASS_SET, binding = 24) uniform sampler2D inUtility;

layout (location = 0) out vec4 FragColor;

void main()
{
    float ndcDepth = texture(inDepth, fs_in.TexCoord).x;

    vec4 utilitySample = texture(inUtility, fs_in.TexCoord);
    int  instance_index = int(round(utilitySample.x));
    int  info_index     = int(round(utilitySample.y));
    int  material_index = int(round(utilitySample.z));

    vec4 baseColorAO = texture(inBaseColorAO, fs_in.TexCoord);
    vec4 normalRoughness = texture(inNormalRoughness, fs_in.TexCoord);
    vec4 pbrFlags = texture(inPbrFlags, fs_in.TexCoord);
    vec4 emissiveSample = texture(inEmissive, fs_in.TexCoord);
    vec3 emissive = emissiveSample.rgb;
    vec3 normal = normalize(normalRoughness.xyz);

    bool instance_selected = (info_index & 1) == 1; // faster than % 2

    vec3 fragPos        = EE_DEPTH_TO_WORLD_POS(EE_CAMERA_INDEX, fs_in.TexCoord, ndcDepth);
    vec3 cameraPosition = EE_CAMERA_POSITION(EE_CAMERA_INDEX);
    vec3 skyColor       = EE_SKY_COLOR(fragPos - cameraPosition);

    vec2 texelSize  = vec2(textureSize(inUtility, 0));
    vec2 texOffset  = 1.0 / texelSize;

    if (ndcDepth == 1.0) {
        if (!instance_selected && EE_INSTANCE_INDEX == 1) {
            bool foundNeighbor = false;

            // 7x7 neighborhood search for selected instances
            for (int i = -3; i <= 3 && !foundNeighbor; ++i) {
                for (int j = -3; j <= 3; ++j) {
                    int info_index_local = int(round(
                        texture(inUtility,
                                fs_in.TexCoord + vec2(texOffset.x * float(i),
                                                      texOffset.y * float(j))).y));
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

    bool unlit = emissiveSample.a < 0.0;

    float depth = EE_LINEARIZE_DEPTH(EE_CAMERA_INDEX, ndcDepth);
    vec4 shadowDebugColor = EE_FUNC_DIRECTIONAL_SHADOW_DEBUG(depth, fragPos);
    if (shadowDebugColor.a > 0.0f) {
        FragColor = shadowDebugColor;
        return;
    }

    float roughness = normalRoughness.a;
	float metallic = pbrFlags.x;
	float ao = baseColorAO.a;
	vec4 albedo = vec4(baseColorAO.rgb, 1.0);

    vec3 base  = baseColorAO.rgb;
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

    vec3 finalAlbedoRGB = mix(albedo.rgb, debugColor, anyDebug);
    albedo = vec4(finalAlbedoRGB, albedo.a);

    vec3 color = albedo.rgb;
    if (!unlit) {
        vec3 viewDir = normalize(cameraPosition - fragPos);
        bool receiveShadow = true;
        vec3 F0 = pbrFlags.yzw;
        float F90 = emissiveSample.a;
        vec3 direct = EE_FUNC_CALCULATE_LIGHTS(receiveShadow, albedo.rgb, 1.0, depth, normal, viewDir, fragPos,
                                               metallic, roughness, F0, F90);
        vec3 ambient = EE_FUNC_CALCULATE_ENVIRONMENTAL_LIGHT(albedo.rgb, normal, viewDir, metallic, roughness, F0,
                                                             F90) +
                       EE_FUNC_CALCULATE_DDGI_DIFFUSE(albedo.rgb, normal, viewDir, fragPos);
        color = direct + emissive + ambient * ao;
    }

    vec4 outputColor;

    if (!instance_selected && EE_INSTANCE_INDEX == 1) {
        bool foundNeighbor = false;

        for (int i = -3; i <= 3 && !foundNeighbor; ++i) {
            for (int j = -3; j <= 3; ++j) {
                int info_index_local = int(round(
                    texture(inUtility,
                            fs_in.TexCoord + vec2(texOffset.x * float(i),
                                                  texOffset.y * float(j))).y));
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

    float fade_ratio = EE_CAMERA_FADE_RATIO(EE_CAMERA_INDEX);
    float camFar     = EE_CAMERA_FAR(EE_CAMERA_INDEX);
    float fadeStart  = camFar * fade_ratio;

    if (depth > fadeStart) {
        float t = (depth - fadeStart) / (camFar * (1.0 - fade_ratio));
        outputColor.rgb = mix(outputColor.rgb, skyColor, clamp(t, 0.0, 1.0));
    }

    FragColor = outputColor;
}
