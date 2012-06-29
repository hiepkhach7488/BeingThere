#version 400 compatibility

uniform float depthTransparency;
uniform float depthContrast;
uniform sampler2D texDepth;
uniform sampler2D texColor; 

void main() {
	
	vec4 colorImage = texture2D(texColor, gl_TexCoord[0].xy);
	vec4 colorDepth = texture2D(texDepth, gl_TexCoord[0].xy)*depthContrast;
 
	gl_FragColor = (1.0-depthTransparency)*colorImage+colorDepth*depthTransparency;
}


