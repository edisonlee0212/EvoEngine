out vec3 FragColor;

in VS_OUT {
	vec2 TexCoords;
} vs_in;

uniform sampler2D InputTex;

void main()
{
	float delta = 1.0 / textureSize(InputTex, 0).y;
	vec2 texCoords = vs_in.TexCoords;
	if(texture(InputTex, texCoords).rgb != vec3(1.0, 1.0, 1.0)) {
		FragColor = vec3(0.0);
		return;
	}
	if(texCoords.x > 0.5){
		for(float testX = texCoords.x; testX <= 1.0; testX += delta){
			if(texture(InputTex, vec2(testX, texCoords.y)).rgb != vec3(1.0, 1.0, 1.0)) {
				FragColor = vec3(0.0);
				return;
			}
		}
	}else{
		for(float testX = texCoords.x; testX >= 0.0; testX -= delta){
			if(texture(InputTex, vec2(testX, texCoords.y)).rgb != vec3(1.0, 1.0, 1.0)) {
				FragColor = vec3(0.0);
				return;
			}
		}
	}
	if(texCoords.y > 0.5){
		for(float testY = texCoords.y; testY <= 1.0; testY += delta){
			if(texture(InputTex, vec2(texCoords.x, testY)).rgb != vec3(1.0, 1.0, 1.0)) {
				FragColor = vec3(0.0);
				return;
			}
		}
	}else{
		for(float testY = texCoords.y; testY >= 0.0; testY -= delta){
			if(texture(InputTex, vec2(texCoords.x, testY)).rgb != vec3(1.0, 1.0, 1.0)) {
				FragColor = vec3(0.0);
				return;
			}
		}
	}
	FragColor = vec3(1.0);
}