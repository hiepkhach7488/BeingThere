#ifndef V3D_COMMON_H_
#define V3D_COMMON_H_

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

#include "beingthere_config.h"

namespace beingthere{

	template<typename T, typename K>
	void readImage(string img_path, V3D::Image<T>& img, short nchannels);

	template<typename T, typename K>
	void convertImage(V3D::Image<T>& src, cv::Mat& dst, short nchannels, bool to_ucharDepth);

	template<typename T, typename K>
	void convertImage(cv::Mat& src, V3D::Image<K>& dst, short nchannels, bool to_ucharDepth);

	template<typename T, typename K>
	void convertImage(V3D::Image<T>& src, V3D::Image<K>& dst, short nchannels, bool to_ucharDepth);

	template<typename T, typename K>
	void convertImage(cv::Mat& src, cv::Mat& dst, short nchannels, bool to_ucharDepth);

	template<typename T>
	void minConvolveImageHorizontal(V3D::Image<T> const& src, int const kernelSize, int const center, V3D::Image<T>& dst);
	
	template<typename T>
	void minConvolveImageVertical(V3D::Image<T> const& src, int const kernelSize, int const center, V3D::Image<T>& dst);

	void erodeDepth(int const radius, V3D::Image<float>& depth);

	void renderEdgeWeights(float const alpha, unsigned int texId, V3D_GPU::RTT_Buffer& dst);

	void copyTexture(unsigned int src, unsigned int dst);

	void erodeDepthTexture(unsigned int src, unsigned int dst);
}

#include "v3d_common.hpp"

#endif