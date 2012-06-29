#ifndef DATA_FUSION_
#define DATA_FUSION_

#include "cv.h"
#include "highgui.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/cuda/point_cloud.h>
#include <pcl/cuda/point_types.h>
#include <pcl/cuda/time_cpu.h>
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>

#include "FileSequence.h"
#include "ImageSequence.h"
#include "calibration_loader.h"
#include "common.h"

using namespace std;
using namespace pcl;

namespace {
	const int NUM_CAM = 10;
}

class DataFusion{
public:
	DataFusion();
	~DataFusion();

	bool loadImageData();
	bool loadCalibrationData();
	bool uploadImageData();
	void createPointCloud();
	void fuseAllCloud();

public:
	//Input
	cv::Mat depthMat[NUM_CAM];
	cv::Mat colorMat[NUM_CAM];
	DeviceArray2D<unsigned short> device_depth[NUM_CAM];

	//Vertex And Normal Map
	DeviceArray2D<float> vMapSOA[NUM_CAM]; 
	DeviceArray2D<float> vMapSOA_dst[NUM_CAM];
	DeviceArray2D<float4> vMapAOS[NUM_CAM];
	
	DeviceArray2D<float> nMapSOA[NUM_CAM];
	DeviceArray2D<float> nMapSOA_dst[NUM_CAM];
	

	//Calibration Mat
	beingthere::gpu::Intr intrinsic[NUM_CAM];
	beingthere::gpu::Mat33 g_extrinsicR[NUM_CAM];
	float3 g_extrinsicT[NUM_CAM];

	//Cloud Output
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud[NUM_CAM];
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud;
private:
	ImageSequence* colorLoader;
	ImageSequence* depthLoader;
	CalibrationLoader* calibLoader;
};

#endif