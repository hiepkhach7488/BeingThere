#version 330 core 

#define ATTR_POSITION 0
#define ATTR_TEXCOORD 4

layout(location = ATTR_POSITION) in vec4 Position;
layout(location = ATTR_TEXCOORD) in vec2 TexCoord;

varying vec2 VertTexCoord;

void main(){	
	gl_Position = Position;
	VertTexCoord = TexCoord;
}
