#version 400 compatibility

//convert float depths to 0..255 so they fit in the R channel of an image
uniform sampler2D warpedDepthTexture;
uniform float minDepth;
uniform float maxDepth;

uniform int w;
uniform int h;

void main(){

	vec2 p = vec2(gl_FragCoord.x/w, gl_FragCoord.y/h);
	float d = texture2D(warpedDepthTexture, p).x;
	
	//scale to 0..1
	d = ((d-minDepth)/(maxDepth-minDepth)); 
	
	//flip values > 0
	d = (d > 0.0)? 1.0-d:d; 

	//clip
	d = (d > 1.0)?1.0:d; 
	d = (d < 0.0)?0.0:d;

	gl_FragColor = vec4(d, 0.0, 0.0, 1.0);

}