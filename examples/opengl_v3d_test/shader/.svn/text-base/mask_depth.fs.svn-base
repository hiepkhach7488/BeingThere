#version 400 compatibility

uniform sampler2D maskTexture;
uniform sampler2D depthTexture;
uniform int w;
uniform int h;

void main(){

	vec2 p = vec2(gl_FragCoord.x/w, gl_FragCoord.y/h);
	float m = texture2D(maskTexture, p).x;
	float d = texture2D(depthTexture, p).x;

	d = max(0.0f, d);

	gl_FragColor.x = (m > 0.5) ? -1.0 : d;

}