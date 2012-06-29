#ifndef KINECT_VISUALIZATION
#define KINECT_VISUALIZATION

//Common Header
#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/image_viewer.h>
#include "pcl/visualization/pcl_visualizer.h"
using namespace pcl;
using namespace Eigen;

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
using namespace std;

#include "cv.h"
#include "highgui.h"

//Other Header
#include "pcl/gpu/kinfu/kinfu.h"
#include "pcl/gpu/kinfu/raycaster.h"
#include "pcl/gpu/kinfu/marching_cubes.h"
#include "pcl/gpu/containers/initialization.h"
using namespace pcl::gpu;

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>

class KinectVisualization{
public:
	KinectVisualization();
	~KinectVisualization();

public:
	  void showDepth (const PtrStepSz<const unsigned short>& depth);
	  void showRGB(const PtrStepSz<const KinfuTracker::PixelRGB>& rgb24);
	  void KinectVisualization::showCloudXYZ(const KinfuTracker& kinfu);
	  void KinectVisualization::showCloudXYZRGB(const KinfuTracker& kinfu);

	  template<typename CloudPtr> void
	  writeCloudFile (int format, const CloudPtr& cloud_prt);
	  Eigen::Affine3f getViewerPose (visualization::PCLVisualizer& viewer);
	  void setViewerPose(visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose);
	  //void KinectVisualization::setViewerPose(const Eigen::Affine3f& viewer_pose);

public:
	visualization::ImageViewer viewerScene_;
	visualization::ImageViewer viewerDepth_;
	visualization::ImageViewer viewerColor_;

	//Point-Cloud Visualization
	PointCloud<PointXYZ>::Ptr cloud_ptr_;
	DeviceArray2D<PointXYZ> cloud_device_xyz;
	DeviceArray2D<PointXYZRGB> cloud_device_xyzrgb;
	visualization::PCLVisualizer cloud_viewer_xyz_;
	visualization::PCLVisualizer cloud_viewer_xyzrgb_;

	enum { PCD_BIN = 1, PCD_ASCII = 2, PLY = 3, MESH_PLY = 7, MESH_VTK = 8 };
};

#endif