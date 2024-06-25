out vec4 FragColor;

in VS_OUT {
	vec4 aColor;
} fs_in;

void main()
{	
	FragColor = fs_in.aColor;
}