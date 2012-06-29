/*
Bilateral Filter
Andrew Maimone
*/

#extension GL_EXT_gpu_shader4 : enable

// Input texture
uniform sampler2D depth; 
uniform sampler2D lastDepth; 
uniform sampler2D lastDepthActual; 
uniform sampler2D color; 
uniform int init; //if init = 1, always filter so we get a smooth maps before the temporal change thesholds are enabled

// Gaussian sigmas
#define RADIUS 20
#define SKIP 4
#define SIGMA_DIST   RADIUS
#define SIGMA_DEPTH_CM_SQ_PER_M  0.02
#define SIGMA_DEPTH_CM_SQ_PER_M_THRESH  0.004
#define SIGMA_COLOR  0.1
#define DEPTH_TO_CM 6553.5

float gaussian(float t, float sigma) {
	return exp(-(t*t)/(sigma*sigma));
}

void main() {
	//init = 1;

	vec2 pos = vec2(gl_TexCoord[0].xy);
	//vec3 centralColor = texture2D(color, pos).rgb;
	float centralDepth = texture2D(depth, pos).g;
	float lastcentralDepth = texture2D(lastDepth, pos).g;
	float lastcentralDepthActual = texture2D(lastDepthActual, pos).g;
	vec2 Tinvsize = 1.0/vec2(textureSize2D(depth, 0));


	//if this raw frame or last raw frame missing, leave as missing
	/*if(centralDepth == 0.0 || lastcentralDepthActual == 0.0) { 
		gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
	//current raw frame not missing, if change is small from last filtered version, use last filtered version 
	} else */if(init==0 && abs(centralDepth-lastcentralDepth)*DEPTH_TO_CM < centralDepth*DEPTH_TO_CM*centralDepth*DEPTH_TO_CM*SIGMA_DEPTH_CM_SQ_PER_M_THRESH/100.0) {
			gl_FragColor = vec4(0.0, lastcentralDepth, 0.0, 1.0);
	//current raw frame not missing, and change is not small, do normal filtering
	} else { 
		float sum = 0.0, totalWeight = 0.0, minv = 1.0, maxv = 0.0;
		for(int u = -RADIUS; u <= RADIUS; u+=SKIP) {
			for(int v = -RADIUS; v <= RADIUS; v+=SKIP) {
				vec2 offset = vec2(u,v);
				//vec3 c = texture2D(color, pos+offset*Tinvsize).rgb;
				float d = texture2D(depth, pos+offset*Tinvsize).g;

				if(d != 0.0) {
					if(d > maxv) maxv = d;
					if(d < minv) minv = d;
			
					float weightDist = gaussian(length(vec2(offset)), float(SIGMA_DIST));
					float weightDepth = gaussian(abs(d-centralDepth)*DEPTH_TO_CM, centralDepth*DEPTH_TO_CM*centralDepth*DEPTH_TO_CM*SIGMA_DEPTH_CM_SQ_PER_M/100.0);
					//float weightColor = gaussian(distance(c, centralColor), SIGMA_COLOR);
					float weight = weightDist*weightDepth;//*weightColor;

					sum += d * weight;
					totalWeight += weight;
				}
			}
	
			float value = sum/totalWeight;
			gl_FragColor = vec4(0.0, value, 0.0, 1.0);
		}	
	}
	

	//gl_FragColor = vec4(0.0, centralDepth, 0.0, 1.0);

}

