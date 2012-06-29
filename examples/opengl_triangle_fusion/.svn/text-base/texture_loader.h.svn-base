#ifndef TEXTURE_LOADER_
#define TEXTURE_LOADER_

#include <cv.h>
#include <highgui.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <GL/glew.h>
#include <GL/GL.h>
#include <GL/freeglut.h>

using namespace std;

#include "ImageSequence.h"
#include "glslprogram.h"

#include "beingthere_config.h"

#ifndef COMMON_PARAMS_
	const int NUM_CAM = 10;
	#define COMMON_PARAMS_
#endif

class TextureLoader{
public:
	TextureLoader();
	~TextureLoader();

public:
 	GLuint colorTex[NUM_CAM];
	GLuint depthTex[NUM_CAM];
	
	//Processed Depth Texture
	GLuint processedDepthTex[NUM_CAM];
	GLuint processedColorTex[NUM_CAM];

	cv::Mat depthMat[NUM_CAM];
	cv::Mat colorMat[NUM_CAM];
	
	bool loadImageData();
	bool initTexture();
	bool uploadToTexture();
	bool processTextures();

	bool createShader();

private:
	ImageSequence* colorLoader;
	ImageSequence* depthLoader;

	GLSLProgram* progMedianSm;
	GLSLProgram* progMedianLg;
	GLSLProgram* progBilateral;

	//Temp Depth Texture
	GLuint tempDepthTex;
	GLuint tempColorTex;

	GLuint lastDepthTex[NUM_CAM];
	GLuint lastColorTex[NUM_CAM];

	bool initColorTex(GLuint& const colorTex);
	bool initDepthTex(GLuint& const depthTex);

	bool processDepthTexture(GLuint& const, GLuint& const, GLuint& );
	void renderNormalizedQuad();
	void copyTexture(GLuint& const inTex, GLuint& const outTex);

	//To get temp result
	GLuint fbo;
};

#endif