#ifndef VOLUMETRIC_RENDERING_
#define VOLUMETRIC_RENDERING_

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

#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/containers/kernel_containers.h>

#include "openni_capture.h"
#include "color_handler.h"

#include "pcl/common/angles.h"

#include "tsdf_volume.h"
#include "tsdf_volume.hpp"

class VolumetricRendering{
public:
	//VolumetricRendering();
	VolumetricRendering(CaptureOpenNI*, KinfuTracker*);
	~VolumetricRendering();

	void initVolumeVisualizer();
	void initTSDFVisualizer();
	
	void showAll();
	void renderTSDF(Eigen::Affine3f* pose_ptr = 0);
	void renderTSDFCloud(KinfuTracker& kinfu, bool integrate_colors);
	void toggleCube(const Eigen::Vector3f&);

private:
	//Rendering Parameters
	CaptureOpenNI* capture_;
	KinfuTracker* kinfu_;

	RayCaster::Ptr raycaster_ptr_;
	KinfuTracker::DepthMap generated_depth_;

	//Display Rendering Ray-Tracking Result
	visualization::ImageViewer viewerScene_;
	KinfuTracker::View view_device_;
	KinfuTracker::View colors_device_;
	vector<KinfuTracker::PixelRGB> view_host_;

private:
	//TSDF Cloud Visualizer
	visualization::PCLVisualizer cloud_viewer_;
	Eigen::Affine3f viewer_pose_;
	enum { GPU_Connected6 = 0, CPU_Connected6 = 1, CPU_Connected26 = 2 };
	bool exit_;
	Eigen::Vector3f volume_size;

private:
	//For TSDF Rendering
	int extraction_mode_;
	bool compute_normals_;
	bool valid_combined_;
	bool cube_added_;
	
	//Cloud Params
	PointCloud<PointXYZ>::Ptr cloud_ptr_;
	PointCloud<Normal>::Ptr normals_ptr_;
	PointCloud<PointNormal>::Ptr combined_ptr_;
	PointCloud<RGB>::Ptr point_colors_ptr_;

	DeviceArray<PointXYZ> cloud_buffer_device_;
	DeviceArray<Normal> normals_device_;
	DeviceArray<PointNormal> combined_device_;  
	DeviceArray<RGB> point_colors_device_; 

	//Create Mesh
	MarchingCubes::Ptr marching_cubes_;
	DeviceArray<PointXYZ> triangles_buffer_device_;
	boost::shared_ptr<pcl::PolygonMesh> mesh_ptr_;

	//Volumetric Rendering Params
	KinfuTracker::DepthMap depth_device_;
	pcl::TSDFVolume<float, short> tsdf_volume_;
	pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_cloud_ptr_;
};

#endif