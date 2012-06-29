#version 400 compatibility

//mask unused parts of color image with black

uniform sampler2D colorTexture;
uniform sampler2D depthTexture;
uniform int w;
uniform int h;

void main(){

	vec2 p = vec2(gl_FragCoord.x/w, gl_FragCoord.y/h);
	float m = texture2D(depthTexture, p).x;
	vec4 c = texture2D(colorTexture, p);

	vec4 black = vec4(0, 0, 0, 1);
	
	bool inside = true;
	if(gl_FragCoord.x < 40 || gl_FragCoord.x > 440) inside = false;
	if(gl_FragCoord.y < 40 || gl_FragCoord.y > 600) inside = false;

	if(!inside && (m<0.5))
		gl_FragColor = black;
	else 
		gl_FragColor = c;
}