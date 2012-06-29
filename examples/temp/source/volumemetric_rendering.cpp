#include "volumemetric_rendering.h"

namespace pcl
{
  namespace gpu
  {
    void paint3DView (const KinfuTracker::View& rgb24, KinfuTracker::View& view, float colors_weight = 0.5f);
    void mergePointNormal (const DeviceArray<PointXYZ>& cloud, const DeviceArray<Normal>& normals, DeviceArray<PointNormal>& output);
  }
}

VolumetricRendering::VolumetricRendering(CaptureOpenNI* niCap, KinfuTracker* kinfu):extraction_mode_ (GPU_Connected6), compute_normals_ (false), 
	valid_combined_ (false), cube_added_(false), cloud_viewer_ ("Scene Cloud Viewer"), capture_(niCap), kinfu_(kinfu){
	initVolumeVisualizer();
	initTSDFVisualizer();
	exit_ = false;
	Eigen::Vector3f volume_size = Vector3f::Constant (3.f/*meters*/);
	toggleCube(volume_size);  
}

VolumetricRendering::~VolumetricRendering(){

}

void VolumetricRendering::initVolumeVisualizer(){
	viewerScene_.setWindowTitle ("View3D from ray tracing");
}

//For Visualize Point Cloud
void VolumetricRendering::initTSDFVisualizer(){
	cloud_ptr_ = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);
    normals_ptr_ = PointCloud<Normal>::Ptr (new PointCloud<Normal>);
    combined_ptr_ = PointCloud<PointNormal>::Ptr (new PointCloud<PointNormal>);
    point_colors_ptr_ = PointCloud<RGB>::Ptr (new PointCloud<RGB>);

    cloud_viewer_.setBackgroundColor (0, 0, 0);
    cloud_viewer_.addCoordinateSystem (1.0);
    cloud_viewer_.initCameraParameters ();
    cloud_viewer_.camera_.clip[0] = 0.01;
    cloud_viewer_.camera_.clip[1] = 10.01;

    cloud_viewer_.addText ("H: print help", 2, 15, 20, 34, 135, 246);      

	float f = capture_->depth_focal_length_VGA;
	float diag = sqrt ((float)kinfu_->cols () * kinfu_->cols () + kinfu_->rows () * kinfu_->rows ());
	cloud_viewer_.camera_.fovy = 2 * atan (diag / (2 * f)) * 1.5;
	
	tsdf_cloud_ptr_ = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
	raycaster_ptr_ = RayCaster::Ptr( new RayCaster(kinfu_->rows (), kinfu_->cols (), f, f) );
}

void VolumetricRendering::showAll(){
	PtrStepSz<const unsigned short> depth;
	PtrStepSz<const KinfuTracker::PixelRGB> rgb24;
	int time_ms = 0;
	bool has_image = false;

	for (int i = 0; !exit_; ++i)
	{
		bool has_frame = capture_->grab (depth, rgb24);      
		if (!has_frame)
		{
			cout << "Can't grab" << endl;
			break;
		}

		depth_device_.upload (depth.data, depth.step, depth.rows, depth.cols);
		has_image = (*kinfu_ )(depth_device_);   

		if(has_image){
			renderTSDF(0);
			renderTSDFCloud(*kinfu_, false);
		}
	}
}

void VolumetricRendering::renderTSDF (Eigen::Affine3f* pose_ptr)
{
	if (pose_ptr)
	{
		raycaster_ptr_->run(kinfu_->volume(), *pose_ptr);
		raycaster_ptr_->generateSceneView(view_device_);
	}
	else
		kinfu_->getImage (view_device_);

	//if (paint_image_ && registration && !pose_ptr)
	//{
	//	colors_device_.upload (rgb24.data, rgb24.step, rgb24.rows, rgb24.cols);
	//	//paint3DView (colors_device_, view_device_);
	//}

	int cols;
	view_device_.download (view_host_, cols);
	viewerScene_.showRGBImage ((unsigned char*)&view_host_[0], view_device_.cols (), view_device_.rows ());
	viewerScene_.spinOnce();

	//viewerColor_.showRGBImage ((unsigned char*)&rgb24.data, rgb24.cols, rgb24.rows);
	//viewerColor_.spinOnce();
}

void VolumetricRendering::renderTSDFCloud(KinfuTracker& kinfu, bool integrate_colors)
{
	viewer_pose_ = kinfu.getCameraPose();

	//ScopeTimeT time ("PointCloud Extraction");
	//cout << "\nGetting cloud... " << flush;

	valid_combined_ = false;

	if (extraction_mode_ != GPU_Connected6)     // So use CPU
	{
		kinfu.volume().fetchCloudHost (*cloud_ptr_, extraction_mode_ == CPU_Connected26);
	}
	else
	{
		DeviceArray<PointXYZ> extracted = kinfu.volume().fetchCloud (cloud_buffer_device_);             

		if (compute_normals_)
		{
			kinfu.volume().fetchNormals (extracted, normals_device_);
			pcl::gpu::mergePointNormal (extracted, normals_device_, combined_device_);
			combined_device_.download (combined_ptr_->points);
			combined_ptr_->width = (int)combined_ptr_->points.size ();
			combined_ptr_->height = 1;

			valid_combined_ = true;
		}
		else
		{
			extracted.download (cloud_ptr_->points);
			cloud_ptr_->width = (int)cloud_ptr_->points.size ();
			cloud_ptr_->height = 1;
		}

		if (integrate_colors)
		{
			kinfu.colorVolume().fetchColors(extracted, point_colors_device_);
			point_colors_device_.download(point_colors_ptr_->points);
			point_colors_ptr_->width = (int)point_colors_ptr_->points.size ();
			point_colors_ptr_->height = 1;
		}
		else
			point_colors_ptr_->points.clear();
	}

	size_t points_size = valid_combined_ ? combined_ptr_->points.size () : cloud_ptr_->points.size ();
	//cout << "Done.  Cloud size: " << points_size / 1000 << "K" << endl;

	cloud_viewer_.removeAllPointClouds ();    
	if (valid_combined_)
	{
		visualization::PointCloudColorHandlerRGBHack<PointNormal> rgb(combined_ptr_, point_colors_ptr_);
		cloud_viewer_.addPointCloud<PointNormal> (combined_ptr_, rgb, "Cloud");
		cloud_viewer_.addPointCloudNormals<PointNormal>(combined_ptr_, 50);
	}
	else
	{
		visualization::PointCloudColorHandlerRGBHack<PointXYZ> rgb(cloud_ptr_, point_colors_ptr_);
		cloud_viewer_.addPointCloud<PointXYZ> (cloud_ptr_, rgb);
	}    
}

void VolumetricRendering::toggleCube(const Eigen::Vector3f& size)
{
	if (cube_added_)
		cloud_viewer_.removeShape("cube");
	else
		cloud_viewer_.addCube(size*0.5, Eigen::Quaternionf::Identity(), size(0), size(1), size(2));

	cube_added_ = !cube_added_;
}