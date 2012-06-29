#include <cv.h>
#include <highgui.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/cuda/point_cloud.h>
#include <pcl/cuda/point_types.h>
#include <pcl/cuda/time_cpu.h>
#include <boost/thread/thread.hpp>

#include "pcl/cuda/io/cloud_to_pcl.h"
#include "pcl/cuda/io/disparity_to_cloud.h"
#include <pcl/visualization/cloud_viewer.h>
#include <boost/shared_ptr.hpp>
//
//#include "openni_capture.h"

#include "common.h"
#include "data_fusion.h"

using namespace std;
using namespace pcl;

#include "calibration_loader.h"

int main(int argc, char* argv[]){

	CalibrationLoader* calibLoader = new CalibrationLoader();
	calibLoader->loadAllCalibrationFiles();
	calibLoader->calculateGlobalCalib();

	DataFusion* dataFusion = new DataFusion();
	dataFusion->loadImageData();
	dataFusion->loadCalibrationData();
	dataFusion->uploadImageData();
	dataFusion->createPointCloud();

	visualization::CloudViewer cloudViewer1("Cloud Viewer");
	while(!cloudViewer1.wasStopped()){
		cloudViewer1.showCloud(dataFusion->merged_cloud);
	}

	getchar();

	return 0;
}

