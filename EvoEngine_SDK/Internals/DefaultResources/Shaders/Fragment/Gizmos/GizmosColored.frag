layout(location = 0) out vec4 FragColor;

layout(location = 0) in VS_OUT {
	vec4 Color;
} fs_in;

void main()
{	
	FragColor = fs_in.Color;
}