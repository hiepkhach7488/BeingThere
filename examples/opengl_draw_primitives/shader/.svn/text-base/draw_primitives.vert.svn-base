#version 410
#extension GL_EXT_gpu_shader4 : enable

layout(location = 0) in vec3 Position;
out vec4 color;

uniform mat4 MVP;

void main() {
	gl_Position = MVP * vec4(Position.xyz, 1.0);
	int index = int(Position.x + Position.y);

	if(index % 2 == 0)
		color = vec4(1.0, 0.0, 0.0, 0.0);  
	else
		color = vec4(0.0, 1.0, 0.0, 0.0);  
}


