#ifndef POSE_TRACKER
#define POSE_TRACKER

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
#include <string>
#include <fstream>

using namespace pcl::gpu;
using namespace std;

class PoseTracker{
public:
	PoseTracker(int row, int col);

	/** \brief Pixel type for rendered image. */
	typedef pcl::gpu::PixelRGB PixelRGB;

	typedef DeviceArray2D<PixelRGB> View;
	typedef DeviceArray2D<unsigned short> DepthMap;

	typedef pcl::PointXYZ PointType;
	typedef pcl::Normal NormalType;

	typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;
	typedef Eigen::Vector3f Vector3f;

public:
	void setDepthTruncationForICP (float max_icp_distance);
	void setDepthIntrinsics (float fx, float fy, float cx = -1, float cy = -1);
	void setInitalCameraPose (const Eigen::Affine3f& pose);
	void reset();

	Eigen::Affine3f getCameraPose (int time = -1) const;
	void setIcpCorespFilteringParams (float distThreshold, float sineOfAngle);

	bool operator()(const DepthMap& depth_raw);
	bool operator()(const DepthMap& depth_raw, Matrix3frm& rot, Vector3f& trans);

	void allocateBufffers (int rows, int cols);
	void getImage (View& view) const;

	bool writePoseToFile(std::string file_name_) const;
	bool readPoseToMemory(string file_name, vector<Matrix3frm>& rot_vec, vector<Vector3f>& trans_vec);

public:
	/** \brief Frame counter */
	int global_time_;
	/** \brief Intrinsic parameters of depth camera. */
	float fx_, fy_, cx_, cy_;
	/** \brief Height of input depth image. */
	int rows_;
	/** \brief Width of input depth image. */
	int cols_;
	/** \brief Number of pyramid levels */
	enum { LEVELS = 3 };

public:
	/** \brief Initial camera rotation in volume coo space. */
	Matrix3frm init_Rcam_;

	/** \brief Initial camera position in volume coo space. */
	Vector3f   init_tcam_;

	/** \brief Array of camera rotation matrices for each moment of time. */
	std::vector<Matrix3frm> rmats_;

	/** \brief Array of camera translations for each moment of time. */
	std::vector<Vector3f>   tvecs_;

	/** \brief Tsdf volume container. */
	TsdfVolume::Ptr tsdf_volume_;
	ColorVolume::Ptr color_volume_;
	/** \brief Camera movement threshold. TSDF is integrated iff a camera movement metric exceedes some value. */
	float integration_metric_threshold_;

public:
	/** \brief ICP Correspondences  map type */
	typedef DeviceArray2D<int> CorespMap;

	/** \brief Vertex or Normal Map type */
	typedef DeviceArray2D<float> MapArr;

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

public:
	/** \brief Truncation threshold for depth image for ICP step */
	float max_icp_distance_;
	/** \brief array with IPC iteration numbers for each pyramid level */
	int icp_iterations_[LEVELS];
	/** \brief distance threshold in correspondences filtering */
	float  distThres_;
	/** \brief angle threshold in correspondences filtering. Represents max sine of angle between normals. */
	float angleThres_;

	/** \brief Temporary buffer for ICP */
	DeviceArray2D<float> gbuf_;
	/** \brief Buffer to store MLS matrix. */
	DeviceArray<float> sumbuf_;
};

#endif