#version 400 compatibility

//mask unused parts of color image with black

uniform sampler2D maskTexture;
uniform sampler2D colorTexture;
uniform int w;
uniform int h;

void main(){

	vec2 p = vec2(gl_FragCoord.x/w, gl_FragCoord.y/h);
	float m = texture2D(maskTexture, p).x;
	vec4 c = texture2D(colorTexture, p);

	vec4 black = vec4(0, 0, 0, 1);
	gl_FragColor = (m > 0.5) ? black : c;

}