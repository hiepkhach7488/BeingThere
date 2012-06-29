#version 410
#extension GL_EXT_gpu_shader4 : enable

layout(location = 0) in vec3 Position;

void main() {
	gl_Position = vec4(Position.xyz, 1.0);
}


