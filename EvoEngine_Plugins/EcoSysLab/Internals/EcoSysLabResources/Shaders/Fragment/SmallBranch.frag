out vec3 FragColor;

in VS_OUT {
	vec2 TexCoords;
} vs_in;

uniform sampler2D InputTex;

void main()
{
	float delta = 1.0 / textureSize(InputTex, 0).y;
	vec3 originalColor = texture(InputTex, vs_in.TexCoords).rgb;
	if(originalColor != vec3(0, 1, 0)){
		int greenCount = 0;
		int whiteCount = 0;
		int brownCount = 0;
		for(int x = -1; x <= 1; x++){
			for(int y = -1; y <= 1; y++){
				if(texture(InputTex, vs_in.TexCoords + vec2(delta * x, delta * y)).rgb == vec3(0, 1, 0)) greenCount++;
				else if(texture(InputTex, vs_in.TexCoords + vec2(delta * x, delta * y)).rgb == vec3(1, 1, 1)) whiteCount++;
				else if(texture(InputTex, vs_in.TexCoords + vec2(delta * x, delta * y)).rgb == vec3(0, 0, 1)) brownCount++;
			}
		}
		if(greenCount > 3) originalColor = vec3(0, 1, 0);
		if(whiteCount > 5) originalColor = vec3(1, 1, 1);
	}
	else{
		int greenCount = 0;
		int whiteCount = 0;
		int brownCount = 0;
		for(int x = -1; x <= 1; x++){
			for(int y = -1; y <= 1; y++){
				if(texture(InputTex, vs_in.TexCoords + vec2(delta * x, delta * y)).rgb == vec3(0, 1, 0)) greenCount++;
				else if(texture(InputTex, vs_in.TexCoords + vec2(delta * x, delta * y)).rgb == vec3(1, 1, 1)) whiteCount++;
				else if(texture(InputTex, vs_in.TexCoords + vec2(delta * x, delta * y)).rgb == vec3(0, 0, 1)) brownCount++;
			}
		}
		if(brownCount > 4) originalColor = vec3(0, 0, 1);
		if(whiteCount > 5) originalColor = vec3(1, 1, 1);
	}
	FragColor = originalColor;
}