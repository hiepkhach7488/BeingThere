#ifndef PCL_FUSION_RENDERER_HPP_
#define PCL_FUSION_RENDERER_HPP_

#include <pcl/pcl_macros.h>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/kinfu/pixel_rgb.h>
#include <pcl/gpu/kinfu/tsdf_volume.h>
#include <pcl/gpu/kinfu/color_volume.h>
#include <pcl/gpu/kinfu/raycaster.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <vector>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl::gpu;
using namespace std;

namespace pcl
{
	namespace gpu
	{
	/*	PCL_EXPORTS void 
			paint3DView(const KinfuTracker::View& rgb24, KinfuTracker::View& view, float colors_weight = 0.5f);*/
		PCL_EXPORTS void
			mergePointNormal(const DeviceArray<PointXYZ>& cloud, const DeviceArray<Normal>& normals, DeviceArray<PointNormal>& output);
		Eigen::Vector3f rodrigues2(const Eigen::Matrix3f& matrix);
	}
}

class FusionRenderer{
public:
	/** \brief Pixel type for rendered image. */
	typedef pcl::gpu::PixelRGB PixelRGB;

	typedef DeviceArray2D<PixelRGB> View;
	typedef DeviceArray2D<unsigned short> DepthMap;

	typedef pcl::PointXYZ PointType;
	typedef pcl::Normal NormalType;

	FusionRenderer(){};
	FusionRenderer(int rows, int cols);
	~FusionRenderer();

	void selfInit();
	bool testFusionAlgorithm(const DepthMap& depth);

	View view_device_;
	vector<PixelRGB> view_host_;
	pcl::visualization::ImageViewer viewerScene_;

public:
	/** \brief Sets Depth camera intrinsics
	* \param[in] fx focal length x 
	* \param[in] fy focal length y
	* \param[in] cx principal point x
	* \param[in] cy principal point y
	*/
	void
		setDepthIntrinsics (float fx, float fy, float cx = -1, float cy = -1);

	/** \brief Sets initial camera pose relative to volume coordiante space
	* \param[in] pose Initial camera pose
	*/
	void
		setInitalCameraPose (const Eigen::Affine3f& pose);

	/** \brief Sets truncation threshold for depth image for ICP step only! This helps 
	*  to filter measurements that are outside tsdf volume. Pass zero to disable the truncation.
	* \param[in] max_icp_distance_ Maximal distance, higher values are reset to zero (means no measurement). 
	*/
	void
		setDepthTruncationForICP (float max_icp_distance = 0.f);

	/** \brief Sets ICP filtering parameters.
	* \param[in] distThreshold distance.
	* \param[in] sineOfAngle sine of angle between normals.
	*/
	void
		setIcpCorespFilteringParams (float distThreshold, float sineOfAngle);

	/** \brief Sets integration threshold. TSDF volume is integrated iff a camera movement metric exceedes the threshold value. 
	* The metric represents the following: M = (rodrigues(Rotation).norm() + alpha*translation.norm())/2, where alpha = 1.f (hardcoded constant)
	* \param[in] threshold a value to compare with the metric. Suitable values are ~0.001          
	*/
	void
		setCameraMovementThreshold(float threshold = 0.001f);

	/** \brief Performs initialization for color integration. Must be called before calling color integration. 
	* \param[in] max_weight max weighe for color integration. -1 means default weight.
	*/
	void
		initColorIntegration(int max_weight = -1);        

	/** \brief Returns cols passed to ctor */
	int
		cols ();

	/** \brief Returns rows passed to ctor */
	int
		rows ();
	
	/** \brief Returns TSDF volume storage */
	const TsdfVolume& volume() const;

	/** \brief Returns TSDF volume storage */
	TsdfVolume& volume();

	/** \brief Returns color volume storage */
	const ColorVolume& colorVolume() const;

	/** \brief Returns color volume storage */
	ColorVolume& colorVolume();

	/** \brief Returns camera pose at given time, default the last pose
	* \param[in] time Index of frame for which camera pose is returned.
	* \return camera pose
	*/
	Eigen::Affine3f
		getCameraPose (int time = -1) const;

	/** \brief Returns number of poses including initial */
	size_t
		getNumberOfPoses () const;


	/** \brief Renders 3D scene to display to human
	* \param[out] view output array with image
	*/
	void
		getImage (View& view) const;

	/** \brief Returns point cloud abserved from last camera pose
	* \param[out] cloud output array for points
	*/
	void
		getLastFrameCloud (DeviceArray2D<PointType>& cloud) const;

	/** \brief Returns point cloud abserved from last camera pose
	* \param[out] normals output array for normals
	*/
	void
		getLastFrameNormals (DeviceArray2D<NormalType>& normals) const;

public:
	/** \brief Number of pyramid levels */
	enum { LEVELS = 3 };

	/** \brief ICP Correspondences  map type */
	typedef DeviceArray2D<int> CorespMap;

	/** \brief Vertex or Normal Map type */
	typedef DeviceArray2D<float> MapArr;

	typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;
	typedef Eigen::Vector3f Vector3f;

	/** \brief Height of input depth image. */
	int rows_;
	/** \brief Width of input depth image. */
	int cols_;
	/** \brief Frame counter */
	int global_time_;

	/** \brief Truncation threshold for depth image for ICP step */
	float max_icp_distance_;
	
	/** \brief Intrinsic parameters of depth camera. */
	float fx_, fy_, cx_, cy_;

	/** \brief Tsdf volume container. */
	TsdfVolume::Ptr tsdf_volume_;
	ColorVolume::Ptr color_volume_;
	/** \brief Initial camera rotation in volume coo space. */
	Matrix3frm init_Rcam_;
	/** \brief Initial camera position in volume coo space. */
	Vector3f   init_tcam_;

	/** \brief array with IPC iteration numbers for each pyramid level */
	int icp_iterations_[LEVELS];
	/** \brief distance threshold in correspondences filtering */
	float  distThres_;
	/** \brief angle threshold in correspondences filtering. Represents max sine of angle between normals. */
	float angleThres_;

	/** \brief Depth pyramid. */
	std::vector<DepthMap> depths_curr_;
	/** \brief Vertex maps pyramid for current frame in global coordinate space. */
	std::vector<MapArr> vmaps_g_curr_;
	/** \brief Normal maps pyramid for current frame in global coordinate space. */
	std::vector<MapArr> nmaps_g_curr_;

	/** \brief Vertex maps pyramid for previous frame in global coordinate space. */
	std::vector<MapArr> vmaps_g_prev_;
	/** \brief Normal maps pyramid for previous frame in global coordinate space. */
	std::vector<MapArr> nmaps_g_prev_;

	/** \brief Vertex maps pyramid for current frame in current coordinate space. */
	std::vector<MapArr> vmaps_curr_;
	/** \brief Normal maps pyramid for current frame in current coordinate space. */
	std::vector<MapArr> nmaps_curr_;

	/** \brief Array of buffers with ICP correspondences for each pyramid level. */
	std::vector<CorespMap> coresps_;

	/** \brief Buffer for storing scaled depth image */
	DeviceArray2D<float> depthRawScaled_;

	/** \brief Temporary buffer for ICP */
	DeviceArray2D<float> gbuf_;
	/** \brief Buffer to store MLS matrix. */
	DeviceArray<float> sumbuf_;

	/** \brief Array of camera rotation matrices for each moment of time. */
	std::vector<Matrix3frm> rmats_;

	/** \brief Array of camera translations for each moment of time. */
	std::vector<Vector3f>   tvecs_;

	/** \brief Camera movement threshold. TSDF is integrated iff a camera movement metric exceedes some value. */
	float integration_metric_threshold_;

	/** \brief Allocates all GPU internal buffers.
	* \param[in] rows_arg
	* \param[in] cols_arg          
	*/
	void allocateBufffers (int rows_arg, int cols_arg);

	/** \brief Performs the tracker reset to initial  state. It's used if case of camera tracking fail.
	*/
	void reset ();  

};

#endif