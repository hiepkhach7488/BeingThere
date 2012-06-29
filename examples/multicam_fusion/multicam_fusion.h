#ifndef MULTICAM_FUSION
#define MULTICAM_FUSION

#include <iostream>

#include <pcl/console/parse.h>

#include "pcl/gpu/kinfu/kinfu.h"
#include "pcl/gpu/kinfu/raycaster.h"
#include "pcl/gpu/kinfu/marching_cubes.h"
#include "pcl/gpu/containers/initialization.h"
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/kinfu/pixel_rgb.h>
#include <pcl/gpu/kinfu/tsdf_volume.h>
#include <pcl/gpu/kinfu/color_volume.h>
#include <pcl/gpu/kinfu/raycaster.h>

#include <pcl/pcl_macros.h>
#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/image_viewer.h>
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>

#include <Eigen/Core>
#include <vector>
#include <string>
#include <fstream>


#define HAVE_OPENCV

#ifdef HAVE_OPENCV  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "pcl/gpu/utils/timers_opencv.hpp"
typedef pcl::gpu::ScopeTimerCV ScopeTimeT;
#else
typedef pcl::ScopeTime ScopeTimeT;
#endif

#include "../src/internal.h"

using namespace std;
using namespace pcl;
using namespace pcl::gpu;
using namespace Eigen;
namespace pc = pcl::console;


class MulticamFusion{
public:
	typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;
	typedef Eigen::Vector3f Vector3f;
	typedef DeviceArray2D<PixelRGB> View;
	typedef DeviceArray2D<unsigned short> DepthMap;

	MulticamFusion();
	~MulticamFusion();
	void show(Eigen::Affine3f* pose_ptr = 0);

	Eigen::Affine3f getViewerPose();
	void setViewerPose(Eigen::Affine3f& view_pose_);
	void getLastFrameCloud (DeviceArray2D<PointXYZ>& cloud) const;

	void savePointCloud(string file_name_ = "test_pcd.pcd") const;

public:
	TsdfVolume::Ptr tsdf_volume_;
	Eigen::Affine3f viewer_pose_;
	visualization::PCLVisualizer cloud_viewer_;
	void integrateTSDF(const DepthMap& depth_raw, Matrix3frm* rot = 0, Vector3f* trans = 0);
	DeviceArray2D<float> depthRawScaled_;

	Vector3f volume_size;
	Vector3i volume_resolution;
	View view_device_;
	int rows, cols;

	PointCloud<PointXYZ>::Ptr cloud_ptr_;
	DeviceArray2D<PointXYZ> *cloud_device_;

	pcl::visualization::ImageViewer image_viewer_;
	vector<PixelRGB> viewer_host_;
	RayCaster::Ptr raycaster_ptr_;
	KinfuTracker::DepthMap generated_depth_;

	pcl::device::MapArr vmaps_g_prev_, nmaps_g_prev_;

	Eigen::Affine3f getViewerPose (visualization::PCLVisualizer& viewer)
	{
		Eigen::Affine3f pose = viewer.getViewerPose();
		Eigen::Matrix3f rotation = pose.linear();

		Matrix3f axis_reorder;  
		axis_reorder << 0,  0,  1,
			-1,  0,  0,
			0, -1,  0;

		rotation = rotation * axis_reorder;
		pose.linear() = rotation;
		return pose;
	}

	void setViewerPose (visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
	{
		Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f (0, 0, 0);
		Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
		Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f (0, -1, 0);
		viewer.camera_.pos[0] = pos_vector[0];
		viewer.camera_.pos[1] = pos_vector[1];
		viewer.camera_.pos[2] = pos_vector[2];
		viewer.camera_.focal[0] = look_at_vector[0];
		viewer.camera_.focal[1] = look_at_vector[1];
		viewer.camera_.focal[2] = look_at_vector[2];
		viewer.camera_.view[0] = up_vector[0];
		viewer.camera_.view[1] = up_vector[1];
		viewer.camera_.view[2] = up_vector[2];
		viewer.updateCamera ();
	}
};

#endif