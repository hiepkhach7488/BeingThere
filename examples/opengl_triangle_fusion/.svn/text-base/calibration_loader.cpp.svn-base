#include "calibration_loader.h"

namespace {
	string kinect[] = {"01", "02", "03", "04", "05", "07", "08", "09", "11", "12"};
	string kinect_parent[] = {"12", "12", "12", "01", "12", "01", "12", "01", "12", "-1"};
	int MAS_CAM = 12;
}

bool CalibrationLoader::loadAllCalibrationFiles(){
	char temp[1024];
	CvMat* tempMat;
	string calibPath = BEINGTHERE_ROOT_DIR + "resources/multi-cam/calib";
	//string calibPath = "C:/Logs/PCL/data/calib";

	for(int i = 0; i < NUM_CAM; i++) {
		sprintf(temp, "%s/cam_rgb_k%s.xml",calibPath.c_str(), kinect[i].c_str());

		tempMat = (CvMat*)cvLoad(temp, NULL, NULL, NULL);
		intrinMat[i] = cv::Mat(tempMat);

		intrisic[i].fx = intrinMat[i].at<float>(0, 0);
		intrisic[i].fy = intrinMat[i].at<float>(1, 1);
		intrisic[i].cx = intrinMat[i].at<float>(0, 2);
		intrisic[i].cy = intrinMat[i].at<float>(1, 2);

		//Load Distortion
		sprintf(temp, "%s/distort_rgb_k%s.xml",calibPath.c_str(), kinect[i].c_str());

		tempMat = (CvMat*)cvLoad(temp, NULL, NULL, NULL);
		distortMat[i] = cv::Mat(tempMat);
	} 

	//Load Extrinsic Params
	for(int i=0; i<NUM_CAM; ++i){
		if(kinect_parent[i].compare("-1") != 0) {

			sprintf(temp, "%s/rot_k%s_k%s.xml", calibPath.c_str(), kinect_parent[i].c_str(), kinect[i].c_str());
			tempMat = (CvMat*)cvLoad(temp, NULL, NULL, NULL);
			extrinMatR[i] = cv::Mat(tempMat);

			//Convert To Mat33 Types
			for(int x = 0; x < 3; x++){
				extrinsicR[i].data[x].x = extrinMatR[i].at<float>(x, 0);
				extrinsicR[i].data[x].y = extrinMatR[i].at<float>(x, 1);
				extrinsicR[i].data[x].z = extrinMatR[i].at<float>(x, 2);
			}

			//Load Extrinsic T To cv::Mat
			sprintf(temp, "%s/trans_k%s_k%s.xml",calibPath.c_str(), kinect_parent[i].c_str(), kinect[i].c_str());
			tempMat = (CvMat*)cvLoad(temp, NULL, NULL, NULL);
			extrinMatT[i] = cv::Mat(tempMat);
			extrinMatT[i] /= 100.f;

			//Convert cv::Mat to float3
			extrinsicT[i].x = extrinMatT[i].at<float>(0);
			extrinsicT[i].y = extrinMatT[i].at<float>(1);
			extrinsicT[i].z = extrinMatT[i].at<float>(2);
		}
	}

	return true;
}

void CalibrationLoader::calculateGlobalCalib(){
	for(int i=0; i<NUM_CAM - 1 ; ++i){
		cv::Mat extrinsicMatR_inv;
		cv::invert(extrinMatR[i], extrinsicMatR_inv);
		cv::Mat extrinMatT_inv = extrinMatT[i] * -1.f; //to m
		extrinMatT_inv = extrinsicMatR_inv * extrinMatT_inv;

		g_extrinsicMatR[i] = extrinsicMatR_inv;
		g_extrinsicMatT[i] = extrinMatT_inv;
	}

	g_extrinsicMatR[NUM_CAM-1] = cv::Mat::eye(3, 3, CV_32F);
	g_extrinsicMatT[NUM_CAM-1] = cv::Mat::zeros(3, 1, CV_32F);

	for(int i=0; i<NUM_CAM - 1; ++i){
		if(i==3 || i==5 || i==7){
			g_extrinsicMatR[i] = g_extrinsicMatR[0] * g_extrinsicMatR[i];
			g_extrinsicMatT[i] = g_extrinsicMatR[0] * g_extrinsicMatT[i] + g_extrinsicMatT[0];
		}		
	}

	for(int i=0; i<NUM_CAM; ++i){
		copyMatToMat33(g_extrinsicMatR[i], g_extrinsicR[i]);
		copyMatToFloat3(g_extrinsicMatT[i], g_extrinsicT[i]);
	}
}

void CalibrationLoader::printMat(cv::Mat& mat) const{
	for(int i=0; i<mat.rows; ++i){
		for(int j=0; j<mat.cols; ++j)
			cout << mat.at<float>(i,j) << " ";
		cout << endl;
	}
}
void CalibrationLoader::printMat(beingthere::gpu::Mat33& mat) const{
	for(int i=0; i<3; ++i)
		cout << mat.data[i].x << " " << mat.data[i].y << " " << mat.data[i].z << endl;
}

void CalibrationLoader::printMat(float3& mat) const{
		cout << mat.x << " " << mat.y << " " << mat.z << endl;
}

void CalibrationLoader::printMat(beingthere::gpu::Intr& mat) const{
	cout << mat.fx << " " << mat.fy << " " << mat.cx << " " << mat.cy << endl;
}

void CalibrationLoader::copyMatToMat33(cv::Mat& mat, beingthere::gpu::Mat33& mat33){
	for(int i=0; i<3; ++i){
		mat33.data[i].x = mat.at<float>(i,0);
		mat33.data[i].y = mat.at<float>(i,1);
		mat33.data[i].z = mat.at<float>(i,2);
	}
}

void CalibrationLoader::copyMatToFloat3(cv::Mat& mat, float3& mat3){
	assert((mat.rows == 3 || mat.cols == 3) && (mat.rows == 1 || mat.cols == 1));
	mat3.x = mat.at<float>(0);
	mat3.y = mat.at<float>(1);
	mat3.z = mat.at<float>(2);
}

void CalibrationLoader::loadIdentity(beingthere::gpu::Mat33& mat33) const{
	for(int i=0; i<3; ++i)
		mat33.data[i].x = mat33.data[i].y = mat33.data[i].z = 0;
	mat33.data[0].x = mat33.data[1].y = mat33.data[2].z = 1;
}

void CalibrationLoader::loadZeros(float3& mat3) const{
	mat3.x = mat3.y = mat3.z = 0;
}