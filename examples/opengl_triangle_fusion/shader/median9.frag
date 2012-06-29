/*
3x3 Median
Morgan McGuire and Kyle Whitson
http://graphics.cs.williams.edu

Modified by Andrew Maimone
*/
#extension GL_EXT_gpu_shader4 : enable

// Input texture
uniform sampler2D src; 

//iteration
uniform int pass;

// Change these 2 defines to change precision
#define vec float
#define toVec(x) x.g

#define s2(a, b)				temp = a; a = min(a, b); b = max(temp, b);
#define t2(a, b)				s2(v[a], v[b]);
#define t24(a, b, c, d, e, f, g, h)		t2(a, b); t2(c, d); t2(e, f); t2(g, h); 
#define t25(a, b, c, d, e, f, g, h, i, j)	t24(a, b, c, d, e, f, g, h); t2(i, j);

#define MEDIAN_DEPTH_TOL_MM_PER_M 0.25
#define MAX_ZEROES 21
#define LARGE_VALUE 9999999.0
#define MIN_ENCLOSED 10

#define SCALE 2.0

void main() {

  //read in values in window
  vec v[25];
  vec2 Tinvsize = 1.0/vec2(textureSize2D(src, 0));
  if(gl_TexCoord[0].x>4.0*Tinvsize[0] && gl_TexCoord[0].x<1.0-4.0*Tinvsize[0] && gl_TexCoord[0].y>4.0*Tinvsize[1] && gl_TexCoord[0].y<1.0-4.0*Tinvsize[1]) {
	  v[0] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2(-2.0*SCALE, -2.0*SCALE) * Tinvsize));
	  v[1] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2(-SCALE, -2.0*SCALE) * Tinvsize));
	  v[2] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2( 0.0, -2.0*SCALE) * Tinvsize));
	  v[3] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2( SCALE, -2.0*SCALE) * Tinvsize));
	  v[4] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2( 2.0*SCALE, -2.0*SCALE) * Tinvsize));
	  v[5] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2(-2.0*SCALE, -SCALE) * Tinvsize));
	  v[6] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2(-SCALE, -SCALE) * Tinvsize));
	  v[7] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2( 0.0, -SCALE) * Tinvsize));
	  v[8] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2( SCALE, -SCALE) * Tinvsize));
	  v[9] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2( 2.0*SCALE, -SCALE) * Tinvsize));
	  v[10] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2(-2.0*SCALE, 0.0) * Tinvsize));
	  v[11] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2(-SCALE, 0.0) * Tinvsize));
	  v[12] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2( 0.0, 0.0) * Tinvsize));
	  v[13] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2( SCALE, 0.0) * Tinvsize));
	  v[14] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2( 2.0*SCALE, 0.0) * Tinvsize));
	  v[15] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2(-2.0*SCALE, SCALE) * Tinvsize));
	  v[16] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2(-SCALE, SCALE) * Tinvsize));
	  v[17] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2( 0.0, SCALE) * Tinvsize));
	  v[18] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2( SCALE, SCALE) * Tinvsize));
	  v[19] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2( 2.0*SCALE, SCALE) * Tinvsize));
	  v[20] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2(-2.0*SCALE, 2.0*SCALE) * Tinvsize));
	  v[21] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2(-SCALE, 2.0*SCALE) * Tinvsize));
	  v[22] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2( 0.0, 2.0*SCALE) * Tinvsize));
	  v[23] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2( SCALE, 2.0*SCALE) * Tinvsize));
	  v[24] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2( 2.0*SCALE, 2.0*SCALE) * Tinvsize));


	  //determine number of defined values on edge  
	  int enclosed = 0;
	  enclosed += int(v[0] != 0.0); enclosed += int(v[1] != 0.0); enclosed += int(v[2] != 0.0);
	  enclosed += int(v[3] != 0.0); enclosed += int(v[4] != 0.0); enclosed += int(v[5] != 0.0);
	  enclosed += int(v[9] != 0.0); enclosed += int(v[10] != 0.0); enclosed += int(v[14] != 0.0);
	  enclosed += int(v[15] != 0.0); enclosed += int(v[19] != 0.0); enclosed += int(v[20] != 0.0);
	  enclosed += int(v[21] != 0.0); enclosed += int(v[22] != 0.0); enclosed += int(v[23] != 0.0);
	  enclosed += int(v[24] != 0.0);
	 
	  //determine number of undefined (0) values, and min/max value
	  int numZeros = 0;
	  float minv =  LARGE_VALUE;
	  float maxv = 0.0;
	  for(int i = 0; i < 25; i++) {
		if(v[i]==0.0) {
			numZeros++;
			//if value is undefined (0), replace every other occurence with large 
		        //value to minimize impact to median
			if(numZeros %2 == 0) { 
				v[i] = LARGE_VALUE;
			}		
		} else {
			if(v[i]<minv) minv=v[i];
			if(v[i]>maxv) maxv=v[i];
		}
	  }


	  //add border condition  
	  if(numZeros <= MAX_ZEROES && maxv-minv <= minv*MEDIAN_DEPTH_TOL_MM_PER_M && enclosed >= MIN_ENCLOSED ) {
		  vec temp;
		  t25(0, 1,		3, 4,		2, 4,		2, 3,		6, 7);
		  t25(5, 7,		5, 6,		9, 7,		1, 7,		1, 4);
		  t25(12, 13,		11, 13,		11, 12,		15, 16,		14, 16);
		  t25(14, 15,		18, 19,		17, 19,		17, 18,		21, 22);
		  t25(20, 22,		20, 21,		23, 24,		2, 5,		3, 6);
		  t25(0, 6,		0, 3,		4, 7,		1, 7,		1, 4);
		  t25(11, 14,		8, 14,		8, 11,		12, 15,		9, 15);
		  t25(9, 12,		13, 16,		10, 16,		10, 13,		20, 23);
		  t25(17, 23,		17, 20,		21, 24,		18, 24,		18, 21);
		  t25(19, 22,		8, 17,		9, 18,		0, 18,		0, 9);
		  t25(10, 19,		1, 19,		1, 10,		11, 20,		2, 20);
		  t25(2, 11,		12, 21,		3, 21,		3, 12,		13, 22);
		  t25(4, 22,		4, 13,		14, 23,		5, 23,		5, 14);
		  t25(15, 24,		6, 24,		6, 15,		7, 16,		7, 19);
		  t25(3, 11,		5, 17,		11, 17,		9, 17,		4, 10);
		  t25(6, 12,		7, 14,		4, 6,		4, 7,		12, 14);
		  t25(10, 14,		6, 7,		10, 12,		6, 10,		6, 17);
		  t25(12, 17,		7, 17,		7, 10,		12, 18,		7, 12);
		  t24(10, 18,		12, 20,		10, 20,		10, 12);
		  
	   } else {
		v[12] = (pass<=1)?0.0:toVec(texture2D(src, gl_TexCoord[0].xy + vec2( 0.0, 0.0) * Tinvsize));
	   }
   } else {
	v[12] = toVec(texture2D(src, gl_TexCoord[0].xy + vec2(0.0, 0.0) * Tinvsize));
   }

   gl_FragColor = vec4(0.0, v[12], 0.0, 1.0); 

 
}


