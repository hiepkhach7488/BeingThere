#ifndef TRIANGLE_FUSION_
#define TRIANGLE_FUSION_

#include <cv.h>
#include <highgui.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <GL/glew.h>
#include <GL/GL.h>
#include <GL/freeglut.h>

#include "vector_types.h"
#include "common.h"

#include "glm/glm.hpp"
#include <glm/gtc/matrix_transform.hpp>

#include "calibration_loader.h"
#include "glslprogram.h"

using namespace std;

#ifndef COMMON_PARAMS_
	const int NUM_CAM = 10;
	#define COMMON_PARAMS_
#endif

class TriangleFusion{
public:
	TriangleFusion();
	~TriangleFusion();

	bool loadCalibrationData();
	void createTriangleTemplate();

	bool createShaderProgram();
	void renderFromSingle(int idx, GLuint colorTexId, GLuint depthTexId, glm::mat4& const MVP);
	void renderFromAll(GLuint* colorTex, GLuint* depthTex, glm::mat4& const MVP);
	void renderCloudFromSingle(int idx, GLuint colorTexId, GLuint depthTexId, glm::mat4& const MVP);
	void renderCloudFromAll(GLuint* colorTex, GLuint* depthTex, glm::mat4& const MVP);

	void renderQualityTexture();
	void renderFinalResult();

public:
	//Calibration Mat
	beingthere::gpu::Intr intrinsic[NUM_CAM];
	beingthere::gpu::Mat33 g_extrinsicR[NUM_CAM];
	float3 g_extrinsicT[NUM_CAM];

	GLuint tempColorTexture[NUM_CAM];
	GLuint tempDepthTexture[NUM_CAM];

	GLuint outColorTexture;
	GLuint outDepthTexture;

	bool initTextures();

	void renderToTempTexture(GLuint* colorTex, GLuint* depthTex, glm::mat4& const MVP);
	void renderNormalizedQuad();
	
private:
	CalibrationLoader* calibLoader;
	
	//OpenGL Params
	GLSLProgram* shaderProgram;
	GLSLProgram* cloudProgram;
	GLSLProgram* progQuality;

	int triangleTemplateVertices;
	GLuint triangleTemplateBuffer;

	bool initDepthTex(GLuint& const colorTex);
	bool initColorTex(GLuint& const depthTex);

	GLuint fbo;
};

#endif