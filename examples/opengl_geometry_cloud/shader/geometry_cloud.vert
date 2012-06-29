#version 130 
#extension GL_EXT_gpu_shader4 : enable

uniform mat4 MVP;

uniform float fx;
uniform float fy;
uniform float cx;
uniform float cy;

uniform sampler2D texColor; 
uniform sampler2D texDepth;

out vec4 v_color;
out vec2 tex_coord;

void main() {
	vec2 texSize = vec2(textureSize2D(texDepth, 0));        
	vec2 texInvSize = 1.0/vec2(textureSize2D(texDepth, 0));
	float depth = texture2D(texDepth, gl_Vertex.xy * texInvSize).g * 65535.0 * 0.1;

	vec4 pt = vec4((gl_Vertex.x-cx)*depth/fx, (gl_Vertex.y-cy)*depth/fy, depth,1);
	gl_Position = MVP * pt;
	gl_PointSize = 1;
	v_color = texture2D(texColor, gl_Vertex.xy * texInvSize);
	tex_coord = vec2(gl_Vertex.x, gl_Vertex.y);
}

