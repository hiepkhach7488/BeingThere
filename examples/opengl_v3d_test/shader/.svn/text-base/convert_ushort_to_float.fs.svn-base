#version 400 compatibility

#define NORMALIZED_CONST 65.535

uniform sampler2D inputTexture;

void main(){
   vec4 c = texelFetch(inputTexture, ivec2(gl_FragCoord.xy), 0);
   float d = c.y * NORMALIZED_CONST;
   gl_FragColor = vec4(d, 0, 0, 0);
}