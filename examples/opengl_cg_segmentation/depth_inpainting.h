// -*- C++ -*-
#ifndef DEPTH_INPAINTING_H
#define DEPTH_INPAINTING_H

#include <iostream>

#include "GL/v3d_gpubase.h"

using namespace std;
using namespace V3D;
//using namespace V3D_GPU;

struct GPU_DepthInpainting
{
	GPU_DepthInpainting();
	~GPU_DepthInpainting();

	void allocate(int const w, int const h);

	void setDepth(float const * u0);
	void initialize(float const initDepth);
	void iterate(unsigned int uOrigTex, unsigned int edgeTexId);
	void setTheta(float value) {_theta = value;};
	float getTheta() { return _theta;};
	void setNrIterations(int value) { _nIterations = value;};
	int getNrIterations() { return _nIterations;};
	void setTau(float value) { _tau = value;};
	float getTau() { return _tau;};

	V3D_GPU::RTT_Buffer& getResult() { return *_uBufs[0]; }
	V3D_GPU::RTT_Buffer * _uBufs[2];
	V3D_GPU::RTT_Buffer * _pBufs[2];

	V3D_GPU::ImageTexture2D _uOrigTex;
	int _width, _height, _nIterations;
	float _theta, _tau;
}; // end struct GPU_DepthInpainting

#endif
