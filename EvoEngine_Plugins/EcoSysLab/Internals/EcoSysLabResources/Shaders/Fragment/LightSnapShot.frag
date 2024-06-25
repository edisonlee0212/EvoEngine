layout (location = 0) out float vFragColor;

in VS_OUT {
	flat uint LeafIndex;
	float distance;
} fs_in;

void main()
{
	vFragColor = fs_in.LeafIndex;
}