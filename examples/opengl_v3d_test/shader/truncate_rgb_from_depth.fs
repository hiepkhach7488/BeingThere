#version 400 compatibility

uniform sampler2D colorTex;
uniform sampler2D depthTex;

uniform int w;
uniform int h;

uniform float depthThreshold;

void main(){
	vec2 p = vec2(gl_FragCoord.x/w, gl_FragCoord.y/h);

	float d = texture2D(depthTex, p).x;
	vec4 c = texture2D(colorTex, p);
	
	if(d > depthThreshold || d < 0) 
		gl_FragColor = vec4(0,0,0,0);
	else 
		gl_FragColor = c;

	bool inside = true;
	if(gl_FragCoord.x < 120 || gl_FragCoord.x > 480) inside = false;
	if(gl_FragCoord.y < 120 || gl_FragCoord.y > 480) inside = false;

	if(!inside)
		gl_FragColor = vec4(0,0,0,0);
}