#version 330 core 

#define ATTR_POSITION 0
#define ATTR_TEXCOORD 4

uniform mat4 MVP;

layout(location = ATTR_POSITION) in vec2 Position;
layout(location = ATTR_TEXCOORD) in vec2 TexCoord;

varying vec2 VertTexCoord;

void main(){
	VertTexCoord = TexCoord;
	gl_Position = MVP * vec4(Position.xy, 0.0, 1.0);
}
