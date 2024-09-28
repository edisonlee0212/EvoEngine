layout (location = 0) out vec3 originalColor;
layout (location = 1) out float proximity;

in VS_OUT {
	vec2 TexCoord;
} fs_in;

uniform sampler2D color;
uniform sampler2D gNormal;
uniform sampler2D gDepth;

// parameters (you'd probably want to use them as uniforms to more easily tweak the effect)
uniform int kernelSize;
uniform float radius;
uniform float bias;
uniform float factor;
// tile noise texture over screen based on screen dimensions divided by noise size
uniform vec2 noiseScale; 

void main()
{
    // get input for SSAO algorithm
    float ndcDepth = texture(gDepth, fs_in.TexCoord).r;
    vec3 viewPos = EE_DEPTH_TO_VIEW_POS(EE_CAMERA_INDEX, fs_in.TexCoord, ndcDepth);
    vec3 normal = texture(gNormal, fs_in.TexCoord).rgb;
    originalColor = texture(color, fs_in.TexCoord).rgb;
    if(normal == vec3(0.0)) return;
    normal = normalize(mat3(view) * normal);
    vec3 randomVec = EE_UNIFORM_KERNEL[int(InterleavedGradientNoise(viewPos) * MAX_KERNEL_AMOUNT) % MAX_KERNEL_AMOUNT].xyz;
    // create TBN change-of-basis matrix: from tangent-space to view-space
    vec3 tangent = normalize(randomVec - normal * dot(randomVec, normal));
    vec3 bitangent = cross(normal, tangent);
    mat3 TBN = mat3(tangent, bitangent, normal);
    // iterate over the sample kernel and calculate occlusion factor
    float occlusion = 0.0;
    int validAmount = 0;
    for(int i = 0; i < kernelSize; ++i)
    {
        vec3 point = EE_UNIFORM_KERNEL[i].xyz;
        point.z = abs(point.z);
        // get sample position
        vec3 samplePos = TBN * point; // from tangent to view-space
        samplePos = viewPos + samplePos * radius;
        // project sample position (to sample texture) (to get position on screen/texture)
        vec4 offset = vec4(samplePos, 1.0);
        offset = projection * offset; // from view to clip-space
        offset.xyz /= offset.w; // perspective divide
        offset.xyz = offset.xyz * 0.5 + 0.5; // transform to range 0.0 - 1.0
        validAmount = validAmount + 1;
        // get sample depth
        float sampleDepth = EE_DEPTH_TO_VIEW_POS(EE_CAMERA_INDEX, offset.xy, texture(gDepth, offset.xy).r).z;
        // range check & accumulate
        float rangeCheck = smoothstep(0.0, 1.0, radius / abs(viewPos.z - sampleDepth));
        occlusion += (sampleDepth >= samplePos.z + bias ? 1.0 : 0.0) * rangeCheck;           
    }
    if(validAmount == 0){
        proximity = 0.0;
    }else{
        proximity = occlusion / validAmount;
    }

}