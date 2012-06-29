#version 130 

uniform sampler2D textureMap;

varying vec2 VertTexCoord;

smooth out vec4 Color;

void main(){
	Color = texture(textureMap, VertTexCoord);
	//Color = vec4(0.5, 0.0, 0.0, 0);
}
