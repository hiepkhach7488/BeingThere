#version 400 compatibility
#extension GL_EXT_gpu_shader4: enable

uniform sampler2D warpedDepthTexture;
uniform sampler2D colorTexture;
uniform sampler2D segmentationTexture;

uniform int targetResX;
uniform int targetResY;
uniform mat4 virtualCamMv; 
uniform mat4 virtualCamProj;
uniform mat4 colorCamMv; 
uniform mat4 colorCamProj;

uniform int renderMode;
uniform int cameraMode;

const int renderModeTexture = 1;
const int renderModeDepthMap = 2;
const int renderModeSegmentation = 3;

const int cameraModeFixed = 1;
const int cameraModeInteractive = 2;


vec4 world2screen(in mat4 mv, in mat4 proj, in vec4 worldPoint){
//project world point to image plane of color camera, keep z coordinate in meters (for visibility test)
	vec4 eyePoint = mv*worldPoint;
	vec4 screenPoint = proj*eyePoint;
	screenPoint /= screenPoint.w;
	screenPoint = vec4((screenPoint.x+1.0)/2.0, (screenPoint.y+1.0)/2.0, eyePoint.z/eyePoint.w, 1.0);
	return screenPoint;
}

vec4 screen2world(in mat4 mv, in mat4 proj, in vec3 screenPoint){
	vec4 virtualScreenPos = proj * vec4(0.0, 0.0, -screenPoint.z, 1.0); //convert z to [0..1]
	virtualScreenPos /= virtualScreenPos.w;
	virtualScreenPos.xy = vec2(screenPoint.x/float(targetResX)*2.0-1.0, screenPoint.y/float(targetResY)*2.0-1.0); //xy in [-1..1]
	virtualScreenPos.xy = -virtualScreenPos.xy;
	vec4 worldPos = inverse(mv)*inverse(proj)*virtualScreenPos; //project from virtual camera's screen to world
	return worldPos;
}

void main(){
	
	//get depth of current pixel
	float z = texture2D(warpedDepthTexture, vec2(1.0-gl_FragCoord.x/targetResX, 1.0-gl_FragCoord.y/targetResY)).x; 

	//depth is negative: no geometry
	if (z <= 0){
		vec4 backgroundColor = vec4(0.0, 0.0, 0.0, 1.0);
		gl_FragColor = backgroundColor;
		return;
	}

	//render depth map (interactive and fixed camera)
	if (renderMode == renderModeDepthMap){
		gl_FragColor = vec4(vec3(z/4.0), 1.0);
		return;
	}
	
	//interactive camera
	if (cameraMode == cameraModeInteractive){
		vec4 worldPos = screen2world(virtualCamMv, virtualCamProj, vec3(gl_FragCoord.xy, z));
		vec4 colorScreenPos = world2screen(colorCamMv, colorCamProj, worldPos);
		
		//render segmentation
		if (renderMode == renderModeSegmentation){
			gl_FragColor = texture2D(segmentationTexture, colorScreenPos.xy);
			gl_FragColor = vec4(1.0) - clamp(gl_FragColor, 0.0, 1.0);
			return;
		}
		
		//render texture
		gl_FragColor = texture2D(colorTexture, colorScreenPos.xy);
		return;
		
	}

	//fixed camera
	//render segmentation
	if (renderMode == renderModeSegmentation){
		gl_FragColor = vec4(vec3(texture2D(segmentationTexture, vec2(1.0-gl_FragCoord.x/targetResX, 1.0-gl_FragCoord.y/targetResY)).x), 1.0);
		gl_FragColor = vec4(1.0) - clamp(gl_FragColor, 0.0, 1.0);
		return;
	}
	//render texture
	gl_FragColor = texture2D(colorTexture, vec2(1.0-gl_FragCoord.x/targetResX, 1.0-gl_FragCoord.y/targetResY));
	return;
}

