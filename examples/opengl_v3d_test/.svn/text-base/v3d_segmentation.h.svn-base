#ifndef V3D_SEGMENTATION_H_
#define V3D_SEGMENTATION_H_

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
#include "GL/v3d_gpubinarysegmentation.h"

using namespace std;

#include "v3d_common.h"
#include "glslprogram.h"
#include "beingthere_config.h"
#include "v3d_shaders.h"

using namespace V3D;

class V3dSegmentation{
public:
	V3dSegmentation();
	~V3dSegmentation();

	void createShader();
	void initTextureFBO();

	V3D_GPU::RTT_Buffer _edgeWeightBuf;

	void performSegmentation(GLuint bgColor, GLuint bgDepth, GLuint curColor, GLuint curDepth, GLuint segTex);

private:
	V3D_GPU::BinarySegmentationUzawa _segmentation;
	V3D_GPU::RTT_Buffer _costBuf, _depthBufV, _depthBufH;

	GLuint erodedTex;
	V3D_GPU::ImageTexture2D bgColorTexNoBorder, curColorTexNoBorder;

	V3dShader* v3dShader;

private:
	void renderSegmentationCost(float const lambdaColor, float const colorBias, float const lambdaDepth, float const depthBias,
		unsigned int const bgColorTexId, unsigned int const bgDepthTexId,
		unsigned int const curColorTexId, unsigned int const curDepthTexId,
		V3D_GPU::RTT_Buffer& dst);

	void erodeTextureCG(GLuint inTex, GLuint outTex);
};

#endif