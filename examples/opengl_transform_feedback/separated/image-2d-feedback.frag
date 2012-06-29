#version 330 core

#define FRAG_COLOR	0

varying vec2 VertTexCoord;
uniform sampler2D textureMap;

layout(location = FRAG_COLOR, index = 0) out vec4 FragColor;

void main(){
	FragColor = texture(textureMap, VertTexCoord);
}
