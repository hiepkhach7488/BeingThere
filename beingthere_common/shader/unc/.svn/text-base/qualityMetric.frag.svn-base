/*
Select best quality points using depth and quality information
*/

#extension GL_EXT_gpu_shader4 : enable

#define NUM_CAM 12
uniform float closestDepthTolerancePerMeter;
uniform vec3 colorMatchLinear[NUM_CAM];
uniform vec3 colorMatchConstant[NUM_CAM];
uniform int camEnabled[NUM_CAM];
uniform sampler2D texColorQuality[NUM_CAM];
uniform sampler2D texDepth[NUM_CAM];
uniform float zNear;
uniform float zFar; 
uniform int photometric_merge;

#define WIDTH 640.0
#define HEIGHT 480.0

#define PHOTOMETRIC_SEARCH_RADIUS 0
#define PHOTOMETRIC_THRESHOLD 0.25

void main()
{

	//load depth values and find smallest (closest) depth value
	float closestDepth = 999999.0;
	float depth[NUM_CAM];
	for(int i = 0; i < NUM_CAM; i++) {
		if(camEnabled[i]==1) {
			depth[i] = (zNear*zFar)/(zFar-texture2D(texDepth[i], gl_TexCoord[0].xy).g*(zFar-zNear)); //Z = Zn*Zf / (Zf - z*(Zf-Zn))
			if(depth[i] < closestDepth) closestDepth = depth[i];
		} else {
			depth[i] = 0.0;
		}
	}

	float quality, bestQuality = 0.0;
	vec4 color;
	vec3 bestColor;
	if(photometric_merge) {
		//find surface with best quality
		for(int i = 0; i < NUM_CAM; i++) {
			if(camEnabled[i]==1&&depth[i] <= closestDepth+(closestDepthTolerancePerMeter*depth[i]*0.01)) {
				color = texture2D(texColorQuality[i], gl_TexCoord[0].xy);
				quality = pow(color.a, 2.0);
				if(quality > bestQuality) {
					bestColor = clamp(color.rgb*colorMatchLinear[i]+colorMatchConstant[i],0.0,1.0);
					bestQuality = quality;
				}
			}
		}
	}

	//weight within threshold of largest depth value by quality
	vec4 candColor;
	vec3 colorSum = vec3(0,0,0);
	float totalQuality = 0.0, match, bestMatch;
	for(int i = 0; i < NUM_CAM; i++) {
		if(camEnabled[i]==1&&depth[i] <= closestDepth+(closestDepthTolerancePerMeter*depth[i]*0.01)) {
			if(photometric_merge) {
				//only best match and contribuution if within photometric threshold
				bestMatch = 999.0;
				for(int u = -PHOTOMETRIC_SEARCH_RADIUS; u <= PHOTOMETRIC_SEARCH_RADIUS; u++) {
					for(int v = -PHOTOMETRIC_SEARCH_RADIUS; v <= PHOTOMETRIC_SEARCH_RADIUS; v++) {
						candColor = texture2D(texColorQuality[i], gl_TexCoord[0].xy+vec2(u/WIDTH,v/HEIGHT));
						candColor.rgb = clamp(candColor.rgb*colorMatchLinear[i]+colorMatchConstant[i],0.0,1.0);
						match = distance(candColor.rgb,bestColor);
						if(match < bestMatch) {
							color.rgb = candColor.rgb;
							quality = pow(candColor.a, 2.0);
							bestMatch = match;
						}
					}
				}
				if(bestMatch <  PHOTOMETRIC_THRESHOLD) {
					colorSum += quality*color.rgb;
					totalQuality += quality;
				}
			} else {
				color = texture2D(texColorQuality[i], gl_TexCoord[0].xy);
				color.rgb = clamp(color.rgb*colorMatchLinear[i]+colorMatchConstant[i],0.0,1.0);
				quality = pow(color.a, 2.0);
				colorSum += quality*color.rgb;
				totalQuality += quality;
			}
		}
	}
	
	gl_FragColor = vec4(colorSum/totalQuality, 1);
}
