/*
Point cloud transformation using depth map texture
*/
#extension GL_EXT_gpu_shader4 : enable

// Input texture
uniform mat4 MVP;

uniform float fx;
uniform float fy;
uniform float cx;
uniform float cy;

uniform sampler2D texColor; 
uniform sampler2D texDepth;

varying vec4 v_color;

void main() {
	vec2 texSize = vec2(textureSize2D(texDepth, 0));        //size of depth map
	vec2 texInvSize = 1.0/vec2(textureSize2D(texDepth, 0)); //size of pixel in normalized texture coords
	float depth = texture2D(texDepth, gl_Vertex.xy * texInvSize).g * 65535.0 * 0.1;

	vec4 pt = vec4((gl_Vertex.x-cx)*depth/fx, (gl_Vertex.y-cy)*depth/fy, depth,1);
	//vec4 pt = vec4(gl_Vertex.x, gl_Vertex.y, 0.0, 1.0);

	gl_Position = MVP * pt;
	gl_PointSize = 1;
	//gl_FrontColor = texture2D(texColor, gl_Vertex.xy * texInvSize);
	v_color = texture2D(texColor, gl_Vertex.xy * texInvSize);
	//gl_FrontColor = texture2D(texColor, gl_Vertex.xy * texInvSize);
}

