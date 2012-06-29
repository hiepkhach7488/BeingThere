// -*- C++ -*-
#ifndef FGBG_SEGMENTATION
#define FGBG_SEGMENTATION

#include "Base/v3d_image.h"
#include "GL/v3d_gpubinarysegmentation.h"
#include <GL/glew.h>
#include <GL/glut.h>

using namespace V3D_GPU;

class FGBG_Segmentation
{
public:
	FGBG_Segmentation();
	~FGBG_Segmentation();
	void renderEdgeWeights(float const alpha, unsigned int texId, RTT_Buffer& dst);
	void renderSegmentationCost(float const lambdaColor, float const colorBias, float const lambdaDepth, float const depthThreshold,
								unsigned int const bgColorTexId, unsigned int const bgDepthTexId,
								unsigned int const curColorTexId, unsigned int const curDepthTexId,
								RTT_Buffer& dst);
	void init(int w, int h);
	void clear();
	void run(unsigned int const bgColorTexId, unsigned int const bgDepthTexId, unsigned int const curColorTexId, unsigned int const curDepthTexId);
	void getResult(GLuint texId);
	GLuint getResult();
	//void erodeDepth(int const radius, V3D::Image<float>& depth);

	void setAlpha(float value){_alpha = value;};
	float getAlpha() { return _alpha;};
	void setLambdaColor(float value) {_lambda_color = value;};
	float getLambdaColor() { return _lambda_color;};
	void setColorBias(float value){ _color_bias = value;};
	float getColorBias() { return _color_bias;};
	void setLambdaDepth(float value) {_lambda_depth = value;};
	float getLambdaDepth() { return _lambda_depth;};
	void setDepthBias(float value) { _depth_bias = value;};
	float getDepthBias() { return _depth_bias;};
	void setNrIterations(int value) { _nIterations = value;};
	int getNrIterations() { return _nIterations;};

	RTT_Buffer _edgeWeightBuf;

private:
	BinarySegmentationUzawa _segmentation;
	RTT_Buffer _costBuf, _depthBufV, _depthBufH;
	int _width;
	int _height;

	float _alpha;
	float _lambda_color;
	float _color_bias;
	float _lambda_depth;
	float _depth_bias ;
	int _nIterations;
	
}; // end struct FGBG_Segmentation

#endif
