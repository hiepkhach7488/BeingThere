#ifndef CALIBRATION_LOADER_
#define CALIBRATION_LOADER_

#include <cv.h>
#include <highgui.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "vector_types.h"
#include "common.h"
#include "beingthere_config.h"

using namespace std;

#ifndef COMMON_PARAMS_
	const int MAX_CAM = 15;
	const int NUM_CAM = 10;
	#define COMMON_PARAMS_
#endif

class CalibrationLoader{
public:
	CalibrationLoader(){}
	~CalibrationLoader(){}

	bool loadAllCalibrationFiles();
	void printMat(cv::Mat& mat) const;
	void printMat(float3& mat) const;
	void copyMatToFloat3(cv::Mat& mat, float3& mat3);
	
	void loadIdentity(beingthere::gpu::Mat33& mat33) const;
	void printMat(beingthere::gpu::Mat33& ) const;
	void printMat(beingthere::gpu::Intr& mat) const;
	void copyMatToMat33(cv::Mat& mat, beingthere::gpu::Mat33& mat33);

	void loadZeros(float3& mat3) const;

	void calculateGlobalCalib();
public:
	beingthere::gpu::Intr intrisic[MAX_CAM];
	cv::Mat distortMat[MAX_CAM];

	beingthere::gpu::Mat33 extrinsicR[MAX_CAM];
	float3 extrinsicT[MAX_CAM];

	//Global Extrinsic Matrix
	beingthere::gpu::Mat33 g_extrinsicR[MAX_CAM];
	float3 g_extrinsicT[MAX_CAM];
	cv::Mat g_extrinsicMatR[MAX_CAM];
	cv::Mat g_extrinsicMatT[MAX_CAM];

private:
	cv::Mat intrinMat[MAX_CAM];
	cv::Mat extrinMatR[MAX_CAM];
	cv::Mat extrinMatT[MAX_CAM];
};

#endif