#version 400 compatibility

//convert float depths to 0..255 so they fit in the R channel of an image
uniform sampler2D inputTex;

#define NORMALIZED_CONST 32
 
void main(){
   vec4 c = texelFetch(inputTex, ivec2(gl_FragCoord.xy), 0);
   float d = c.y * NORMALIZED_CONST;

   gl_FragColor = vec4(0.0, d, 0.0, 1.0);
}