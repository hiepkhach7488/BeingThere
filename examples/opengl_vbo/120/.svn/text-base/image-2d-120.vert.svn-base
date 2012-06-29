#version 120

varying vec2 VertTexCoord;

void main(){
	VertTexCoord = gl_MultiTexCoord0.xy;
	gl_Position = gl_ModelViewProjectionMatrix * vec4(gl_Vertex.xy, 0.0, 1.0);
}
