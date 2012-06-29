#ifndef V3D_SHADER_H
#define V3D_SHADER_H

#include <cv.h>
#include <highgui.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <GL/glew.h>
#include <GL/GL.h>
#include <GL/freeglut.h>

#include "GL/v3d_gpubase.h"
#include "Base/v3d_image.h"

using namespace std;

#include "v3d_common.h"
#include "glslprogram.h"
#include "beingthere_config.h"

class V3dShader{
public:
	V3dShader();
	~V3dShader();

	void createShader();
	void initTextureFBO();

	void maskColor(GLuint srcTex, GLuint dstTex, GLuint maskTex);
	void maskDepth(GLuint srcTex, GLuint dstTex, GLuint maskTex);
	void maskColorFromDepth(GLuint srcTex, GLuint dstTex, GLuint depthTex);

	void convertFloatToUshort(GLuint srcTex, GLuint dstTex);
	void convertUshortToFloat(GLuint srcTex, GLuint dstTex);

	void truncateDepth(GLuint srcTex, GLuint dstTex, float depth_threshold);
	void truncateRGBFromDepth(GLuint rgbTex, GLuint depthTex, GLuint dstTex, float depth_threshold);
	void convertDepthToRGB(GLuint srcTex, GLuint dstTex, int min_depth, int max_depth);
	void convertUshortDepthToRGB(GLuint srcTex, GLuint dstTex);

	void drawTriangle();

private:
	GLSLProgram* colorMaskProg;
	GLSLProgram* depthMaskProg;
	GLSLProgram* borderMaskProg;

	GLSLProgram* floatToUshortProg;
	GLSLProgram* ushortToFloatProg;

	GLSLProgram* depthTruncationProg;
	GLSLProgram* rgbTruncationProg;
	GLSLProgram* depthToRGBProg;
	GLSLProgram* ushortDepthToRGBProg;

	GLuint colorFBO;
	GLuint depthFBO;
	GLuint tempColorTex;
	GLuint tempDepthTex;
	GLuint tempDepthUshortTex;
};

#endif