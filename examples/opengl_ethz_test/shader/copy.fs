#version 400 compatibility

uniform sampler2D inputTexture;

void main(){
	gl_FragColor = texelFetch(inputTexture, ivec2(gl_FragCoord.xy), 0);
}