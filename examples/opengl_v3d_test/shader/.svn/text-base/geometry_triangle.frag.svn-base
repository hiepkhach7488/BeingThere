#version 130 
#extension GL_EXT_gpu_shader4 : enable

precision highp float;

//in vec4 geom_color;

uniform sampler2D texColor;
in vec2 TexCoord;
out vec4 output_color;

void main() {
	//output_color = geom_color;
	output_color = texture2D(texColor, TexCoord.st);
}

