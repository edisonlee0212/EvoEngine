layout (location = 0) out vec3 fragColor;

in VS_OUT {
    vec2 TexCoords;
} fs_in;

uniform sampler2D depthStencil;
uniform float near;
uniform float far;
float CalculateLinearDepth(float ndcDepth, float near, float far)
{
    float z = ndcDepth * 2.0 - 1.0;
    return (2.0 * near * far) / (far + near - z * (far - near));
}

void main()
{
    float ndcDepth = texture(depthStencil, fs_in.TexCoords).r;
    float normalizedDepth = 1.0 - CalculateLinearDepth(ndcDepth, near, far) / far;
    int depth = int(normalizedDepth * 32768.0);
    fragColor = vec3((depth / 1024) % 32 / 32.0f, (depth / 32) % 32 / 32.0f, depth % 32 / 32.0f);
}