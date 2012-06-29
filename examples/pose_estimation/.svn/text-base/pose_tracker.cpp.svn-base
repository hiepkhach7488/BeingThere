#include "pose_tracker.h"

#include <iostream>
#include <algorithm>

#include "pcl/common/time.h"
#include "pcl/gpu/kinfu/kinfu.h"
#include "internal.h"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>

#ifdef HAVE_OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>
#include "pcl/gpu/utils/timers_opencv.hpp"
#endif

using namespace std;
using namespace pcl::device;
using namespace pcl::gpu;
using namespace pcl;

using Eigen::AngleAxisf;
using Eigen::Array3f;
using Eigen::Vector3i;
using Eigen::Vector3f;

namespace pcl
{
	namespace gpu
	{
		Eigen::Vector3f rodrigues2(const Eigen::Matrix3f& matrix);
	}
}

PoseTracker::PoseTracker(int rows, int cols) : rows_(rows), cols_(cols), global_time_(0), max_icp_distance_(0), integration_metric_threshold_(0.f){
	//const Vector3f volume_size = Vector3f::Constant (VOLUME_SIZE);
	const Vector3f volume_size = Vector3f::Constant (4.0);
	const Vector3i volume_resolution(VOLUME_X, VOLUME_Y, VOLUME_Z);

	tsdf_volume_ = TsdfVolume::Ptr( new TsdfVolume(volume_resolution) );
	tsdf_volume_->setSize(volume_size);

	setDepthIntrinsics (525.f, 525.f);

	init_Rcam_ = Eigen::Matrix3f::Identity ();// * AngleAxisf(-30.f/180*3.1415926, Vector3f::UnitX());
	init_tcam_ = volume_size * 0.5f - Vector3f (0, 0, volume_size (2) / 2 * 1.2f);

	const int iters[] = {10, 5, 4};
	std::copy (iters, iters + LEVELS, icp_iterations_);

	const float default_distThres = 0.10f; //meters
	const float default_angleThres = sin (20.f * 3.14159254f / 180.f);
	const float default_tranc_dist = 0.03f; //meters

	setIcpCorespFilteringParams (default_distThres, default_angleThres);
	tsdf_volume_->setTsdfTruncDist (default_tranc_dist);

	allocateBufffers (rows, cols);

	rmats_.reserve (30000);
	tvecs_.reserve (30000);

	reset ();
}

void PoseTracker::setDepthIntrinsics (float fx, float fy, float cx, float cy)
{
	fx_ = fx;
	fy_ = fy;
	cx_ = (cx == -1) ? cols_/2 : cx;
	cy_ = (cy == -1) ? rows_/2 : cy; 
}

void PoseTracker::setInitalCameraPose (const Eigen::Affine3f& pose)
{
	init_Rcam_ = pose.rotation ();
	init_tcam_ = pose.translation ();
	reset ();
}


void PoseTracker::setDepthTruncationForICP (float max_icp_distance)
{
	max_icp_distance_ = max_icp_distance;
}


void PoseTracker::allocateBufffers (int rows, int cols)
{    
	depths_curr_.resize (LEVELS);
	vmaps_g_curr_.resize (LEVELS);
	nmaps_g_curr_.resize (LEVELS);

	vmaps_g_prev_.resize (LEVELS);
	nmaps_g_prev_.resize (LEVELS);

	vmaps_curr_.resize (LEVELS);
	nmaps_curr_.resize (LEVELS);

	coresps_.resize (LEVELS);

	for (int i = 0; i < LEVELS; ++i)
	{
		int pyr_rows = rows >> i;
		int pyr_cols = cols >> i;

		depths_curr_[i].create (pyr_rows, pyr_cols);

		vmaps_g_curr_[i].create (pyr_rows*3, pyr_cols);
		nmaps_g_curr_[i].create (pyr_rows*3, pyr_cols);

		vmaps_g_prev_[i].create (pyr_rows*3, pyr_cols);
		nmaps_g_prev_[i].create (pyr_rows*3, pyr_cols);

		vmaps_curr_[i].create (pyr_rows*3, pyr_cols);
		nmaps_curr_[i].create (pyr_rows*3, pyr_cols);

		coresps_[i].create (pyr_rows, pyr_cols);
	}  

	depthRawScaled_.create (rows, cols);
	// see estimate tranform for the magic numbers
	gbuf_.create (27, 20*60);
	sumbuf_.create (27);
}


Eigen::Affine3f PoseTracker::getCameraPose (int time) const
{
	if (time > (int)rmats_.size () || time < 0)
		time = rmats_.size () - 1;

	Eigen::Affine3f aff;
	aff.linear () = rmats_[time];
	aff.translation () = tvecs_[time];
	return (aff);
}

void PoseTracker::reset()
{
	if (global_time_)
		cout << "Reset" << endl;

	global_time_ = 0;
	rmats_.clear ();
	tvecs_.clear ();

	rmats_.push_back (init_Rcam_);
	tvecs_.push_back (init_tcam_);

	tsdf_volume_->reset();

	if (color_volume_) // color integration mode is enabled
		color_volume_->reset(); 
}

void PoseTracker::setIcpCorespFilteringParams (float distThreshold, float sineOfAngle)
{
	distThres_  = distThreshold; //mm
	angleThres_ = sineOfAngle;
}

void PoseTracker::getImage (View& view) const
{
  Eigen::Vector3f light_source_pose = tsdf_volume_->getSize() * (-3.f);

  device::LightSource light;
  light.number = 1;
  light.pos[0] = device_cast<const float3>(light_source_pose);

  view.create (rows_, cols_);
  generateImage (vmaps_g_prev_[0], nmaps_g_prev_[0], light, view);
}

bool PoseTracker::operator()(const DepthMap& depth_raw){
	device::Intr intr (fx_, fy_, cx_, cy_);
	{
		//ScopeTime time(">>> Bilateral, pyr-down-all, create-maps-all");
		//depth_raw.copyTo(depths_curr[0]);
		device::bilateralFilter (depth_raw, depths_curr_[0]);

		if (max_icp_distance_ > 0)
			device::truncateDepth(depths_curr_[0], max_icp_distance_);

		for (int i = 1; i < LEVELS; ++i)
			device::pyrDown (depths_curr_[i-1], depths_curr_[i]);

		for (int i = 0; i < LEVELS; ++i)
		{
			device::createVMap (intr(i), depths_curr_[i], vmaps_curr_[i]);
			//device::createNMap(vmaps_curr_[i], nmaps_curr_[i]);
			computeNormalsEigen (vmaps_curr_[i], nmaps_curr_[i]);
		}
		pcl::device::sync ();
	}

	//can't perform more on first frame
	if (global_time_ == 0)
	{
		Matrix3frm init_Rcam = rmats_[0]; //  [Ri|ti] - pos of camera, i.e.
		Vector3f   init_tcam = tvecs_[0]; //  transform from camera to global coo space for (i-1)th camera pose

		Mat33&  device_Rcam = device_cast<Mat33> (init_Rcam);
		float3& device_tcam = device_cast<float3>(init_tcam);

		Matrix3frm init_Rcam_inv = init_Rcam.inverse ();
		Mat33&   device_Rcam_inv = device_cast<Mat33> (init_Rcam_inv);
		float3 device_volume_size = device_cast<const float3>(tsdf_volume_->getSize());

		//integrateTsdfVolume(depth_raw, intr, device_volume_size, device_Rcam_inv, device_tcam, tranc_dist, volume_);    
		device::integrateTsdfVolume(depth_raw, intr, device_volume_size, device_Rcam_inv, device_tcam, tsdf_volume_->getTsdfTruncDist(), tsdf_volume_->data(), depthRawScaled_);

		for (int i = 0; i < LEVELS; ++i)
			device::tranformMaps (vmaps_curr_[i], nmaps_curr_[i], device_Rcam, device_tcam, vmaps_g_prev_[i], nmaps_g_prev_[i]);

		++global_time_;
		return (false);
	}

	///////////////////////////////////////////////////////////////////////////////////////////
	// Iterative Closest Point
	Matrix3frm Rprev = rmats_[global_time_ - 1]; //  [Ri|ti] - pos of camera, i.e.
	Vector3f   tprev = tvecs_[global_time_ - 1]; //  tranfrom from camera to global coo space for (i-1)th camera pose
	Matrix3frm Rprev_inv = Rprev.inverse (); //Rprev.t();

	//Mat33&  device_Rprev     = device_cast<Mat33> (Rprev);
	Mat33&  device_Rprev_inv = device_cast<Mat33> (Rprev_inv);
	float3& device_tprev     = device_cast<float3> (tprev);

	Matrix3frm Rcurr = Rprev; // tranform to global coo for ith camera pose
	Vector3f   tcurr = tprev;
	{
		//ScopeTime time("icp-all");
		for (int level_index = LEVELS-1; level_index>=0; --level_index)
		{
			int iter_num = icp_iterations_[level_index];

			MapArr& vmap_curr = vmaps_curr_[level_index];
			MapArr& nmap_curr = nmaps_curr_[level_index];

			//MapArr& vmap_g_curr = vmaps_g_curr_[level_index];
			//MapArr& nmap_g_curr = nmaps_g_curr_[level_index];

			MapArr& vmap_g_prev = vmaps_g_prev_[level_index];
			MapArr& nmap_g_prev = nmaps_g_prev_[level_index];

			//CorespMap& coresp = coresps_[level_index];

			for (int iter = 0; iter < iter_num; ++iter)
			{
				Mat33&  device_Rcurr = device_cast<Mat33> (Rcurr);
				float3& device_tcurr = device_cast<float3>(tcurr);

				Eigen::Matrix<float, 6, 6, Eigen::RowMajor> A;
				Eigen::Matrix<float, 6, 1> b;
#if 0
				device::tranformMaps(vmap_curr, nmap_curr, device_Rcurr, device_tcurr, vmap_g_curr, nmap_g_curr);
				findCoresp(vmap_g_curr, nmap_g_curr, device_Rprev_inv, device_tprev, intr(level_index), vmap_g_prev, nmap_g_prev, distThres_, angleThres_, coresp);
				device::estimateTransform(vmap_g_prev, nmap_g_prev, vmap_g_curr, coresp, gbuf_, sumbuf_, A.data(), b.data());

				//cv::gpu::GpuMat ma(coresp.rows(), coresp.cols(), CV_32S, coresp.ptr(), coresp.step());
				//cv::Mat cpu;
				//ma.download(cpu);
				//cv::imshow(names[level_index] + string(" --- coresp white == -1"), cpu == -1);
#else
				estimateCombined (device_Rcurr, device_tcurr, vmap_curr, nmap_curr, device_Rprev_inv, device_tprev, intr (level_index),
					vmap_g_prev, nmap_g_prev, distThres_, angleThres_, gbuf_, sumbuf_, A.data (), b.data ());
#endif
				//checking nullspace
				float det = A.determinant ();

				if (fabs (det) < 1e-15 || !pcl::device::valid_host (det))
				{
					if (!valid_host (det)) cout << "qnan" << endl;

					reset ();
					return (false);
				}
				//float maxc = A.maxCoeff();

				Eigen::Matrix<float, 6, 1> result = A.llt ().solve (b);
				//Eigen::Matrix<float, 6, 1> result = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);

				float alpha = result (0);
				float beta  = result (1);
				float gamma = result (2);

				Eigen::Matrix3f Rinc = (Eigen::Matrix3f)AngleAxisf (gamma, Vector3f::UnitZ ()) * AngleAxisf (beta, Vector3f::UnitY ()) * AngleAxisf (alpha, Vector3f::UnitX ());
				Vector3f tinc = result.tail<3> ();

				//compose
				tcurr = Rinc * tcurr + tinc;
				Rcurr = Rinc * Rcurr;
			}
		}
	}
	//save tranform
	rmats_.push_back (Rcurr);
	tvecs_.push_back (tcurr);

	///////////////////////////////////////////////////////////////////////////////////////////
	// Integration check - We do not integrate volume if camera does not move.  
	float rnorm = rodrigues2(Rcurr.inverse() * Rprev).norm();
	float tnorm = (tcurr - tprev).norm();
	const float alpha = 1.f;
	bool integrate = (rnorm + alpha * tnorm)/2 >= integration_metric_threshold_;  

	///////////////////////////////////////////////////////////////////////////////////////////
	// Volume integration
	float3 device_volume_size = device_cast<const float3> (tsdf_volume_->getSize());

	Matrix3frm Rcurr_inv = Rcurr.inverse ();
	Mat33&  device_Rcurr_inv = device_cast<Mat33> (Rcurr_inv);
	float3& device_tcurr = device_cast<float3> (tcurr);
	if (integrate)
	{
		//ScopeTime time("tsdf");
		//integrateTsdfVolume(depth_raw, intr, device_volume_size, device_Rcurr_inv, device_tcurr, tranc_dist, volume_);
		integrateTsdfVolume (depth_raw, intr, device_volume_size, device_Rcurr_inv, device_tcurr, tsdf_volume_->getTsdfTruncDist(), tsdf_volume_->data(), depthRawScaled_);
	}

	///////////////////////////////////////////////////////////////////////////////////////////
	// Ray casting
	Mat33& device_Rcurr = device_cast<Mat33> (Rcurr);
	{
		//ScopeTime time("ray-cast-all");                
		raycast (intr, device_Rcurr, device_tcurr, tsdf_volume_->getTsdfTruncDist(), device_volume_size, tsdf_volume_->data(), vmaps_g_prev_[0], nmaps_g_prev_[0]);
		for (int i = 1; i < LEVELS; ++i)
		{
			resizeVMap (vmaps_g_prev_[i-1], vmaps_g_prev_[i]);
			resizeNMap (nmaps_g_prev_[i-1], nmaps_g_prev_[i]);
		}
		pcl::device::sync ();
	}

	++global_time_;
	return (true);
}

bool PoseTracker::operator()(const DepthMap& depth_raw, Matrix3frm& rot, Vector3f& trans){
	device::Intr intr (fx_, fy_, cx_, cy_);

	//First Frame
	if(global_time_ == 0){
		Matrix3frm init_Rcam = rot;    //  [Ri|ti] - pos of camera, i.e.
		Vector3f   init_tcam = trans; //  transform from camera to global coo space for (i-1)th camera pose

		Mat33&  device_Rcam = device_cast<Mat33> (init_Rcam);
		float3& device_tcam = device_cast<float3>(init_tcam);

		Matrix3frm init_Rcam_inv = init_Rcam.inverse ();
		Mat33&   device_Rcam_inv = device_cast<Mat33> (init_Rcam_inv);
		float3 device_volume_size = device_cast<const float3>(tsdf_volume_->getSize());

		device::integrateTsdfVolume(depth_raw, intr, device_volume_size, device_Rcam_inv, device_tcam, tsdf_volume_->getTsdfTruncDist(), tsdf_volume_->data(), depthRawScaled_);
		global_time_++;
		return false;
	}

	rmats_.push_back(rot);
	tvecs_.push_back(trans);

	// Integration check - We do not integrate volume if camera does not move.  
	Matrix3frm Rcurr = rot;
	Vector3f tcurr = trans;
	Matrix3frm Rprev = rmats_[rmats_.size()-2]; 
	Vector3f tprev = tvecs_[tvecs_.size()-2];
	
	float rnorm = rodrigues2(Rcurr.inverse() * Rprev).norm();
	float tnorm = (tcurr - tprev).norm();
	const float alpha = 1.f;
	bool integrate = (rnorm + alpha * tnorm)/2 >= integration_metric_threshold_;  

	// Volume integration
	float3 device_volume_size = device_cast<const float3> (tsdf_volume_->getSize());

	Matrix3frm Rcurr_inv = Rcurr.inverse ();
	Mat33&  device_Rcurr_inv = device_cast<Mat33> (Rcurr_inv);
	float3& device_tcurr = device_cast<float3> (tcurr);
	if (integrate)
	{
		//ScopeTime time("tsdf");
		//integrateTsdfVolume(depth_raw, intr, device_volume_size, device_Rcurr_inv, device_tcurr, tranc_dist, volume_);
		integrateTsdfVolume (depth_raw, intr, device_volume_size, device_Rcurr_inv, device_tcurr, tsdf_volume_->getTsdfTruncDist(), tsdf_volume_->data(), depthRawScaled_);
	}

	Mat33& device_Rcurr = device_cast<Mat33> (Rcurr);
	{
		//ScopeTime time("ray-cast-all");                
		raycast (intr, device_Rcurr, device_tcurr, tsdf_volume_->getTsdfTruncDist(), device_volume_size, tsdf_volume_->data(), vmaps_g_prev_[0], nmaps_g_prev_[0]);
		for (int i = 1; i < LEVELS; ++i)
		{
			resizeVMap (vmaps_g_prev_[i-1], vmaps_g_prev_[i]);
			resizeNMap (nmaps_g_prev_[i-1], nmaps_g_prev_[i]);
		}
		pcl::device::sync ();
	}

	++global_time_;
	return (true);

}

bool PoseTracker::writePoseToFile(string file_name_) const{
	fstream FILE_OUT;
	FILE_OUT.open(file_name_, ios::out);

	if(FILE_OUT.is_open()){
		FILE_OUT << rmats_.size() << endl;

		for(int i=0; i<rmats_.size(); ++i){
			Matrix3frm rots = rmats_.at(i);
			Vector3f trans = tvecs_.at(i);

			FILE_OUT<< rots << endl;
			FILE_OUT<< trans << endl;
		}
	}

	FILE_OUT.close();
	return true;
}

bool PoseTracker::readPoseToMemory(string file_name_, vector<Matrix3frm>& rot_vec, vector<Vector3f>& trans_vec){
	ifstream FILE_IN(file_name_);

	int size;

	if(FILE_IN.is_open()){
		FILE_IN >> size;

		for(int idx=0; idx<size; ++idx){
			Matrix3frm rots;
			Vector3f trans;

			for(int i=0; i<3; ++i)
				for(int j=0; j<3; ++j){
					FILE_IN >> rots(i,j);
				}

			for(int i=0; i<3; ++i)
				FILE_IN >> trans(i);
			
			rot_vec.push_back(rots);
			trans_vec.push_back(trans);
		}
	}

	return true;
}

