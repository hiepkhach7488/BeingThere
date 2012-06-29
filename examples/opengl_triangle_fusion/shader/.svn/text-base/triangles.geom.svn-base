/*
Triangle transformation using depth map texture
*/

#extension GL_ARB_geometry_shader4 : enable


// Input texture
uniform float fx;
uniform float fy;
uniform float cx;
uniform float cy;
uniform float max_triangle_z_offset; //cm offset allowed per meter
uniform sampler2D texDepth;
uniform sampler2D texColor;
uniform int calcQuality;

#define CORRECT_START 100.0	 //minimum depth (in CM) to start depth correction
uniform float zCorrectQuad;  
uniform float zCorrectLinear;  
uniform float zCorrectConstant;  //in cm

varying out float nDotL;
varying out float distanceToCam;

uniform mat4 MVP;

#define DEPTH_TO_CM 6553.5
#define NORMAL_SMOOTH_PER_M 5.0
#define WIDTH 640.0
#define HEIGHT 480.0

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


void main() {

	//vertex and six neighboring vertex depth values that can be used to form triangles
	float d0 = texelFetch2D(texDepth, ivec2(gl_PositionIn[0].xy), 0).g;
	float d1 = texelFetch2D(texDepth, ivec2(gl_PositionIn[1].xy), 0).g;
	float d2 = texelFetch2D(texDepth, ivec2(gl_PositionIn[2].xy), 0).g;
	float maxOffset = 0.01*min(min(d0,d1),d2)*max_triangle_z_offset;

	nDotL = 0.0;
	distanceToCam = 0.0;
	if(d0 != 0.0 && d1 !=0.0 && d2 !=0.0 && abs(d0-d1) <= maxOffset && abs(d0-d2) <= maxOffset && abs(d1-d2) <= maxOffset && d0*DEPTH_TO_CM<500) {
		gl_FrontColor = vec4(1,1,1,1);

		//float factor = 0.0;
		//if(zCorrectQuad == -2.3888772255786783E-05) {
		//	factor = -2;
		//};

		float depth = d0*DEPTH_TO_CM;
		depth = depth<CORRECT_START?depth:depth*depth*zCorrectQuad + depth*zCorrectLinear + zCorrectConstant;		
		//depth += 0.0001*factor*depth*depth;
		vec4 pos = vec4((gl_PositionIn[0].x-cx)*depth/fx, (gl_PositionIn[0].y-cy)*depth/fy, depth, 1);
		gl_TexCoord[0]  = vec4(gl_PositionIn[0].x/WIDTH, gl_PositionIn[0].y/HEIGHT, 0, 1);
		//gl_Position = gl_ModelViewProjectionMatrix * pos;
		gl_Position = MVP * pos;

		if(calcQuality==1) {
			distanceToCam = distance(vec3(pos), vec3(0,0,0));
			nDotL = dot(calcNormal(gl_PositionIn[0].xy, vec3(pos)), normalize(vec3(-pos)));
		}
		EmitVertex();

		depth = d1*DEPTH_TO_CM;
		depth = depth<CORRECT_START?depth:depth*depth*zCorrectQuad + depth*zCorrectLinear + zCorrectConstant;	
		//depth += 0.0001*factor*depth*depth;
		pos = vec4((gl_PositionIn[1].x-cx)*depth/fx, (gl_PositionIn[1].y-cy)*depth/fy, depth, 1);
		gl_TexCoord[0]  = vec4(gl_PositionIn[1].x/WIDTH, gl_PositionIn[1].y/HEIGHT, 0, 1);
		//gl_Position = gl_ModelViewProjectionMatrix * pos;
		gl_Position = MVP * pos;

		if(calcQuality==1) {
			distanceToCam = distance(vec3(pos), vec3(0,0,0));
			nDotL = dot(calcNormal(gl_PositionIn[1].xy, vec3(pos)), normalize(vec3(-pos)));
		}
		EmitVertex();

		depth = d2*DEPTH_TO_CM;
		depth = depth<CORRECT_START?depth:depth*depth*zCorrectQuad + depth*zCorrectLinear + zCorrectConstant;	
		//depth += 0.0001*factor*depth*depth;
		pos = vec4((gl_PositionIn[2].x-cx)*depth/fx, (gl_PositionIn[2].y-cy)*depth/fy, depth, 1);
		gl_TexCoord[0]  = vec4(gl_PositionIn[2].x/WIDTH, gl_PositionIn[2].y/HEIGHT, 0, 1);
		//gl_Position = gl_ModelViewProjectionMatrix * pos;
		gl_Position = MVP * pos;

		if(calcQuality==1) {
			distanceToCam = distance(vec3(pos), vec3(0,0,0));
			nDotL = dot(calcNormal(gl_PositionIn[2].xy, vec3(pos)), normalize(vec3(-pos)));
		}
		EmitVertex();

		EndPrimitive();	
	}
}




