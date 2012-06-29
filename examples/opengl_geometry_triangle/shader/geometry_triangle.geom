#version 410
#extension GL_EXT_gpu_shader4 : enable

precision highp float;

#define DEPTH_TO_CM 6553.5
#define NORMAL_SMOOTH_PER_M 5.0
#define WIDTH 640.0
#define HEIGHT 480.0

layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

uniform mat4 MVP;
uniform sampler2D texDepth;
uniform sampler2D texColor;

uniform float fx;
uniform float fy;
uniform float cx;
uniform float cy;
uniform float max_triangle_z_offset; 

out vec2 TexCoord;
//out vec4 geom_color;

vec3 calcNormal(vec2 coord, vec3 p) {
	float normal_smooth = NORMAL_SMOOTH_PER_M*p.z*.01;
	
	//texture coords of adjacent points
	vec2 cL = coord+vec2(-normal_smooth, 0.0);
	vec2 cR = coord+vec2(+normal_smooth, 0.0);
	vec2 cT = coord+vec2( 0.0,           +normal_smooth);
	vec2 cB = coord+vec2( 0.0,           -normal_smooth);

	//depth values of adjacent points
	float dL = texelFetch2D(texDepth, ivec2(cL), 0).g*DEPTH_TO_CM;
	float dR = texelFetch2D(texDepth, ivec2(cR), 0).g*DEPTH_TO_CM;
	float dT = texelFetch2D(texDepth, ivec2(cT), 0).g*DEPTH_TO_CM;
	float dB = texelFetch2D(texDepth, ivec2(cB), 0).g*DEPTH_TO_CM;

	//projected coordinates of adjacent points
	vec3 pL = vec3((cL.x-cx)*dL/fx, (cL.y-cy)*dL/fy, dL);
	vec3 pR = vec3((cR.x-cx)*dR/fx, (cR.y-cy)*dR/fy, dR);
	vec3 pT = vec3((cT.x-cx)*dT/fx, (cT.y-cy)*dT/fy, dT);
	vec3 pB = vec3((cB.x-cx)*dB/fx, (cB.y-cy)*dB/fy, dB);
	
	//find average normal of 4 triangles meeting at center point
	vec3 temp;
	vec3 normal = vec3(0.0, 0.0, 0.0);
	
	//uncomment sections below for more accuracy
	//if(dL != 0.0) {
		/*if(dT != 0.0) {*/temp=/*normalize(*/cross(pT-p,pL-p)/*)*/; if(temp.z>0.0) temp=-temp; normal += temp;//};
		/*if(dB != 0.0) {*/temp=/*normalize(*/cross(pB-p,pL-p)/*)*/; if(temp.z>0.0) temp=-temp; normal += temp;//};
	//}
	//if(dR != 0.0) {
		/*if(dT != 0.0) {*/temp=/*normalize(*/cross(pT-p,pR-p)/*)*/; if(temp.z>0.0) temp=-temp; normal += temp;//};	
		/*if(dB != 0.0) {*/temp=/*normalize(*/cross(pB-p,pR-p)/*)*/; if(temp.z>0.0) temp=-temp; normal += temp;//};
	//}	
	
	return normalize(normal);
}

void main(void){	
	//vertex and six neighboring vertex depth values that can be used to form triangles
	float d0 = texelFetch2D(texDepth, ivec2(gl_in[0].gl_Position.xy), 0).g;
	float d1 = texelFetch2D(texDepth, ivec2(gl_in[1].gl_Position.xy), 0).g;
	float d2 = texelFetch2D(texDepth, ivec2(gl_in[2].gl_Position.xy), 0).g;

	float maxOffset = 0.01*min(min(d0,d1),d2)*max_triangle_z_offset;

	if(d0 != 0.0 && d1 !=0.0 && d2 !=0.0 && abs(d0-d1) <= maxOffset && abs(d0-d2) <= maxOffset && 
		abs(d1-d2) <= maxOffset && d0*DEPTH_TO_CM<500) {
			//First Vertex
			float depth = d0*DEPTH_TO_CM;
			vec4 pos = vec4((gl_in[0].gl_Position.x-cx)*depth/fx, (gl_in[0].gl_Position.y-cy)*depth/fy, depth, 1);
			TexCoord  = vec2(gl_in[0].gl_Position.x/WIDTH, gl_in[0].gl_Position.y/HEIGHT);
			//geom_color = texture2D(texColor, TexCoord.st);
			gl_Position = MVP * pos;
			EmitVertex();

			//Second Vertex
			depth = d1*DEPTH_TO_CM;
			pos = vec4((gl_in[1].gl_Position.x-cx)*depth/fx, (gl_in[1].gl_Position.y-cy)*depth/fy, depth, 1);
			TexCoord  = vec2(gl_in[1].gl_Position.x/WIDTH, gl_in[1].gl_Position.y/HEIGHT);
			//geom_color = texture2D(texColor, TexCoord.st);
			gl_Position = MVP * pos;
			EmitVertex();

			//Third Vertex
			depth = d2*DEPTH_TO_CM;
			pos = vec4((gl_in[2].gl_Position.x-cx)*depth/fx, (gl_in[2].gl_Position.y-cy)*depth/fy, depth, 1);
			TexCoord  = vec2(gl_in[2].gl_Position.x/WIDTH, gl_in[2].gl_Position.y/HEIGHT);
			//geom_color = texture2D(texColor, TexCoord.st);
			gl_Position = MVP * pos;
			EmitVertex();

			EndPrimitive();	
		}
}

