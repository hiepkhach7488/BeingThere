#version 400 compatibility

uniform sampler2D inputTexture;
uniform float depthThreshold;

void main(){
   float d = texelFetch(inputTexture, ivec2(gl_FragCoord.xy), 0).x;
   d = (d > depthThreshold)?0:d; 
   gl_FragColor = vec4(d, 0, 0, 0);
}