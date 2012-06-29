#include "multicam_fusion.h"

#include <iostream>
#include <algorithm>

#include <pcl/io/pcd_io.h>
#include "pcl/common/time.h"
#include "pcl/gpu/kinfu/kinfu.h"
#include "internal.h"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>

using namespace pcl::device;
using namespace pcl::gpu;
using namespace pcl;

using Eigen::AngleAxisf;
using Eigen::Array3f;
using Eigen::Vector3i;
using Eigen::Vector3f;

namespace{
	Eigen::Affine3f default_pose;
	//Call-back Function
	static void keyboard_callback (const visualization::KeyboardEvent &e, void *cookie)
	{
		int key = e.getKeyCode ();
		MulticamFusion* multiFusion = (MulticamFusion*)cookie;

		default_pose.linear() = Eigen::Matrix3f::Identity ();
		default_pose.translation() = Vector3f(0,0,0);

		if (e.keyUp ())    
			switch (key)
		{
			case 27: 
				exit(0); 
				break;
			case 'p':
				multiFusion->savePointCloud();
				break;

			case 's':
				multiFusion->viewer_pose_ = multiFusion->getViewerPose();
				cout << multiFusion->viewer_pose_.linear() << endl;
				cout << multiFusion->viewer_pose_.translation() << endl;
				break;
			case 'd':
				multiFusion->setViewerPose(default_pose);
				break;
			case 'l':
				default_pose.translation() -= Vector3f(1.f,0,0); 
				multiFusion->setViewerPose(default_pose);
				break;
			case 'r':
				default_pose.translation() += Vector3f(1.f,0,0); 
				multiFusion->setViewerPose(default_pose);
				break;
			case 'f':
				default_pose.translation() += Vector3f(0,0,1.f); 
				multiFusion->setViewerPose(default_pose);
				break;
			case 'u':
				default_pose.translation() -= Vector3f(0,0,1.f); 
				multiFusion->setViewerPose(default_pose);
				break;
			case 't':
				default_pose.translation() -= Vector3f(0,1.f,0.f); 
				multiFusion->setViewerPose(default_pose);
				break;
			case 'b':
				default_pose.translation() += Vector3f(0,1.f,0.f); 
				multiFusion->setViewerPose(default_pose);
				break;
			default: 
				break;
		}
	}
}

MulticamFusion::MulticamFusion(){
	cloud_viewer_.setBackgroundColor (0, 0, 0);
	cloud_viewer_.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1);    
	cloud_viewer_.addCoordinateSystem (1.0);
	cloud_viewer_.initCameraParameters ();
	cloud_viewer_.camera_.clip[0] = 0.01;
	cloud_viewer_.camera_.clip[1] = 10.01;

	cloud_viewer_.addText ("H: print help", 2, 15, 20, 34, 135, 246);      
	
	//Initialize TSDF Volume
	volume_size = Vector3f::Constant (4.4f);
	volume_resolution = Vector3i(512, 512, 512);

	tsdf_volume_ = TsdfVolume::Ptr( new TsdfVolume(volume_resolution) );
	tsdf_volume_->setSize(volume_size);

	image_viewer_.registerKeyboardCallback(keyboard_callback, (void*)this);

	raycaster_ptr_ = RayCaster::Ptr( new RayCaster(480, 640) );

	cloud_viewer_.addCube(volume_size*0.5, Eigen::Quaternionf::Identity(), volume_size(0), volume_size(1), volume_size(2));
	cloud_viewer_.registerKeyboardCallback(keyboard_callback, (void*)this);

	//init_tcam = volume_size * 0.5f - Vector3f (0, 0, volume_size (2) / 2 * 1.2f);
	
	Eigen::Affine3f default_pose;
	default_pose.linear() = Eigen::Matrix3f::Identity ();
	default_pose.translation() = Vector3f(1.5f, 1.5f, -0.3f);
	cout << default_pose.translation() << endl;

	setViewerPose(cloud_viewer_, default_pose);
	rows = 480; cols = 640;
	cloud_device_ = new DeviceArray2D<PointXYZ>(rows, cols);
	cloud_ptr_ = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);
}

MulticamFusion::~MulticamFusion(){

}

void MulticamFusion::getLastFrameCloud (DeviceArray2D<PointXYZ>& cloud) const{
	cloud.create (rows, cols);
	DeviceArray2D<float4>& c = (DeviceArray2D<float4>&)cloud;
	device::convert (vmaps_g_prev_, c);
}

void MulticamFusion::show(Eigen::Affine3f* pose_ptr){
	device::Intr intr (525.f, 525.f, 0, 0);

	//Ray-Casting
	int pyr_rows = 480; int pyr_cols = 640;
	depthRawScaled_.create (480, 640);
	vmaps_g_prev_.create (pyr_rows*3, pyr_cols);
	nmaps_g_prev_.create (pyr_rows*3, pyr_cols);
	view_device_.create (480, 640);

	if(pose_ptr != 0){
		raycaster_ptr_->run(*tsdf_volume_, *pose_ptr); 
		raycaster_ptr_->generateSceneView(view_device_);
	}
	else{
		//Generate Images
		Eigen::Vector3f light_source_pose = tsdf_volume_->getSize() * (-3.f);

		device::LightSource light;
		light.number = 1;
		light.pos[0] = device_cast<const float3>(light_source_pose);
		float3 device_volume_size = device_cast<const float3>(tsdf_volume_->getSize());

		//Viewer's Pose
		Matrix3frm tempR = Eigen::Matrix3f::Identity ();
		//Vector3f tempT = Vector3f(1.5f, 1.5f, -0.3f);
		Vector3f tempT = Vector3f(0.09f, 0.29f, 0.14f);
		Mat33&  device_Rcam = device_cast<Mat33> (tempR);
		float3& device_tcam = device_cast<float3>(tempT);

		raycast (intr, device_Rcam, device_tcam, tsdf_volume_->getTsdfTruncDist(), device_volume_size, 
		tsdf_volume_->data(), vmaps_g_prev_, nmaps_g_prev_);
		
		generateImage (vmaps_g_prev_, nmaps_g_prev_, light, view_device_);
	}

	getLastFrameCloud(*cloud_device_);
	int c;
    cloud_device_->download (cloud_ptr_->points, c);
    cloud_ptr_->width = cloud_device_->cols ();
    cloud_ptr_->height = cloud_device_->rows ();
    cloud_ptr_->is_dense = false;

	cloud_viewer_.removeAllPointClouds ();
    cloud_viewer_.addPointCloud<PointXYZ>(cloud_ptr_);

	//For Getting Pose
	cloud_viewer_.spinOnce();		

	//Display
	int cols;
	view_device_.download(viewer_host_, cols);
	image_viewer_.showRGBImage ((unsigned char*)&viewer_host_[0], view_device_.cols (), view_device_.rows ());
	image_viewer_.spinOnce();
}

void MulticamFusion::savePointCloud(string file_name_) const{
	pcl::io::savePCDFileASCII (file_name_, *cloud_ptr_);
}

Eigen::Affine3f MulticamFusion::getViewerPose(){
	return getViewerPose(cloud_viewer_);
}

void MulticamFusion::setViewerPose(Eigen::Affine3f& viewer_pose_){
	setViewerPose(cloud_viewer_, viewer_pose_);
}

void MulticamFusion::integrateTSDF(const DepthMap& depth_raw, Matrix3frm* rotPtr, Vector3f* transPtr){
	Matrix3frm init_Rcam;
	Vector3f   init_tcam;

	if(rotPtr == 0 && transPtr == 0){
		//Default Pose
		init_Rcam = Eigen::Matrix3f::Identity ();// * AngleAxisf(-30.f/180*3.1415926, Vector3f::UnitX());
		//init_tcam = volume_size * 0.5f - Vector3f (0, 0, volume_size (2) / 2 * 1.2f);
		//init_tcam = Vector3f(1.5f, 1.5f, -0.3f);
		init_tcam = Vector3f(0, 0, -0.0f);
	}
	else{
		init_Rcam = *rotPtr;    //  [Ri|ti] - pos of camera, i.e.
		init_tcam = *transPtr; //  transform from camera to global coo space for (i-1)th camera pose
	}

	Mat33&  device_Rcam = device_cast<Mat33> (init_Rcam);
	float3& device_tcam = device_cast<float3>(init_tcam);

	Matrix3frm init_Rcam_inv = init_Rcam.inverse ();
	Mat33&   device_Rcam_inv = device_cast<Mat33> (init_Rcam_inv);
	float3 device_volume_size = device_cast<const float3>(tsdf_volume_->getSize());

	device::Intr intr (525.f, 525.f, 0, 0);

	device::integrateTsdfVolume(depth_raw, intr, device_volume_size, device_Rcam_inv, device_tcam, 
		tsdf_volume_->getTsdfTruncDist(), tsdf_volume_->data(), depthRawScaled_);

}
