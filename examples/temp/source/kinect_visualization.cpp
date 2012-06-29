#include "kinect_visualization.h"

KinectVisualization::KinectVisualization() : cloud_viewer_xyz_("Cloud Visualization"){
	viewerDepth_.setWindowTitle("Kinect Depth Window");
	viewerColor_.setWindowTitle("Kinect Color Window");

	//cloud_viewer_xyz_ = visualization::PCLVisualizer("Cloud Visualization");
	cloud_device_xyz = DeviceArray2D<PointXYZ>(640, 480);

	//For PointCloud Visualization
	cloud_ptr_ = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);

    cloud_viewer_xyz_.setBackgroundColor (0, 0, 0.15);
    cloud_viewer_xyz_.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1);
    cloud_viewer_xyz_.addCoordinateSystem (1.0);
    cloud_viewer_xyz_.initCameraParameters ();
    cloud_viewer_xyz_.camera_.clip[0] = 0.01;
    cloud_viewer_xyz_.camera_.clip[1] = 10.01;
}

KinectVisualization::~KinectVisualization(){

}

void KinectVisualization::showDepth (const PtrStepSz<const unsigned short>& depth) { 
	viewerDepth_.showShortImage (depth.data, depth.cols, depth.rows, 0, 5000, true); 
}

void  KinectVisualization::showRGB(const PtrStepSz<const KinfuTracker::PixelRGB>& rgb24){
	viewerColor_.showRGBImage(&rgb24.data->r, rgb24.cols, rgb24.rows);
}

void KinectVisualization::showCloudXYZ(const KinfuTracker& kinfu)
{
	//cout << cloud_device_xyz.cols() << endl;
	kinfu.getLastFrameCloud (cloud_device_xyz);

	int c;
	cloud_device_xyz.download (cloud_ptr_->points, c);
	cloud_ptr_->width = cloud_device_xyz.cols ();
	cloud_ptr_->height = cloud_device_xyz.rows ();
	cloud_ptr_->is_dense = false;

	//cout << cloud_ptr_->width << " " << cloud_ptr_->height << endl;
	//writeCloudFile(2, cloud_ptr_);
	cloud_viewer_xyz_.removeAllPointClouds ();
	cloud_viewer_xyz_.addPointCloud<PointXYZ>(cloud_ptr_);
}

template<typename CloudPtr> void
KinectVisualization::writeCloudFile (int format, const CloudPtr& cloud_prt)
{
  if (format == PCD_BIN)
  {
    cout << "Saving point cloud to 'cloud_bin.pcd' (binary)... " << flush;
    pcl::io::savePCDFile ("cloud_bin.pcd", *cloud_prt, true);
  }
  else
  if (format == PCD_ASCII)
  {
    cout << "Saving point cloud to 'cloud.pcd' (ASCII)... " << flush;
    pcl::io::savePCDFile ("cloud.pcd", *cloud_prt, false);
  }
  else   /* if (format == KinFuApp::PLY) */
  {
    cout << "Saving point cloud to 'cloud.ply' (ASCII)... " << flush;
    pcl::io::savePLYFileASCII ("cloud.ply", *cloud_prt);
  
  }
  cout << "Done" << endl;
}

void
KinectVisualization::setViewerPose (visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
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

Eigen::Affine3f 
KinectVisualization::getViewerPose (visualization::PCLVisualizer& viewer)
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
