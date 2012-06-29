#version 120

uniform mat4 MVP;
varying vec2 VertTexCoord;

void main(){
	VertTexCoord = gl_MultiTexCoord0.xy;
	gl_Position = MVP * vec4(gl_Vertex.xy, 0.0, 1.0);
}
