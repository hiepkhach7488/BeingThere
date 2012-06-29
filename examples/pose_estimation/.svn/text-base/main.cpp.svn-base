#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "ImageSequence.h"
#include "pose_tracker.h"

#include <pcl/visualization/image_viewer.h>

#include "cv.h"
#include "highgui.h"

using namespace std;
using namespace pcl;

typedef DeviceArray2D<float> MapArr;
typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;
typedef Eigen::Vector3f Vector3f;

static void keyboard_callback (const visualization::KeyboardEvent &e, void *cookie)
{
	PoseTracker* app = reinterpret_cast<PoseTracker*> (cookie);

	int key = e.getKeyCode ();

	if (e.keyUp ())    
		switch (key)
	{
		case 27: 
			exit(0); 
			break;
		case 's':

			break;
		default: 
			break;
	}
}

void fuseSingleImg(){
	//ImageSequence* imgCapture = new ImageSequence();
	//imgCapture->init("C:/Logs/PCL/data/image/", ".png", "snapshot_depth_");
	//imgCapture->open();
	//imgCapture->captureStart();

	IplImage *depthImg0 = cvLoadImage("C:/Logs/PCL/data/image/snapshot_depth_0.png", CV_LOAD_IMAGE_UNCHANGED);
	IplImage *depthImg1 = cvLoadImage("C:/Logs/PCL/data/image/snapshot_depth_3.png", CV_LOAD_IMAGE_UNCHANGED);

	CvMat* rots_0_1 = (CvMat*)cvLoad("C:/Logs/PCL/data/calib/rot_k01_k04.xml", NULL, NULL, NULL);
	CvMat* trans_0_1 = (CvMat*)cvLoad("C:/Logs/PCL/data/calib/trans_k01_k04.xml", NULL, NULL, NULL);

	//Poses Of Cam 0
	Vector3f tran_vec_0_(1.5,1.5,-0.3);
	Matrix3frm rot_vec_0_; 
	rot_vec_0_.fill(0); 
	rot_vec_0_(0,0) = rot_vec_0_(1,1) = rot_vec_0_(2,2) = 1.f;

	//Poses of Cam1
	Vector3f tran_vec_1_;
	Matrix3frm rot_vec_1_;
	
	for(int i=0; i<3; ++i){
		for(int j=0; j<3; ++j)
			rot_vec_1_(i,j) = CV_MAT_ELEM(*rots_0_1, float, i, j);
		
		tran_vec_1_(i) = CV_MAT_ELEM(*trans_0_1, float, i, 0) / 100;
	}	
			
	tran_vec_1_ = tran_vec_1_ - tran_vec_0_;
	rot_vec_1_.transpose();
	//cout << tran_vec_0_ << endl;
	//cout << rot_vec_0_ << endl;
	//cout << tran_vec_1_ << endl;
	//cout << rot_vec_1_ << endl;
	//
	//getchar();

	PoseTracker* pose_tracker = new PoseTracker(480,640);
	DeviceArray2D<unsigned short> depth_raw;	
	
	DeviceArray2D<PixelRGB> view_device_;
	vector<PixelRGB> viewer_host_;
	pcl::visualization::ImageViewer image_viewer_;
	image_viewer_.registerKeyboardCallback(keyboard_callback, (void*)pose_tracker);

	while(true){
		//IplImage* depthImg = imgCapture->image();
		//imgCapture->captureNext();

		depth_raw.upload(depthImg0->imageData, 640*2, 480, 640);
		(*pose_tracker)(depth_raw, rot_vec_0_, tran_vec_0_);
		//depth_raw.upload(depthImg1->imageData, 640*2, 480, 640);
		//(*pose_tracker)(depth_raw, rot_vec_1_, tran_vec_1_);
		//(*pose_tracker)(depth_raw);

		pose_tracker->getImage(view_device_);
		int cols;
		view_device_.download(viewer_host_, cols);
		image_viewer_.showRGBImage ((unsigned char*)&viewer_host_[0], view_device_.cols (), view_device_.rows ());
		image_viewer_.spinOnce();

		//getchar();
	}

	getchar();
}

int main(int argc, char* argv[]){
	fuseSingleImg();
	return 0;
	
	ImageSequence* imgCapture = new ImageSequence();
	imgCapture->init("C:/Logs/PCL/data/depth", ".png", "img_depth_");
	imgCapture->open();
	imgCapture->captureStart();
		
	PoseTracker* pose_tracker = new PoseTracker(480,640);
	DeviceArray2D<unsigned short> depth_raw;
	IplImage* depthImg = 0;
	
	DeviceArray2D<PixelRGB> view_device_;
	vector<PixelRGB> viewer_host_;
	pcl::visualization::ImageViewer image_viewer_;

	//READ FILE
	vector<Matrix3frm> rot_vec;
	vector<Vector3f> tran_vec;
	//pose_tracker->readPoseToMemory("Pose.txt", rot_vec, tran_vec);
	pose_tracker->readPoseToMemory("C:/Logs/PCL/beingthere/resources/ExtrinsicPose.txt", rot_vec, tran_vec);
	//for(int i=0; i<10; ++i){
	//	cout << rot_vec.at(i) << endl;
	//	cout << tran_vec.at(i) << endl;
	//}

	//getchar();

	int count = 0;
	while(true){
		++count;
		imgCapture->captureNext();
		depthImg = imgCapture->image();
		//if(count % 5 != 0) continue; cout << count << endl;
		//cout << imgCapture->currentFile() << endl;

		depth_raw.upload(depthImg->imageData, 640*2, 480, 640);
		(*pose_tracker)(depth_raw, rot_vec[count], tran_vec[count]);

		//Eigen::Affine3f pose = pose_tracker->getCameraPose();
		//cout << pose.linear() << endl;
		//cout << pose.translation() << endl;
		
		pose_tracker->getImage(view_device_);
		int cols;
		view_device_.download(viewer_host_, cols);
		image_viewer_.showRGBImage ((unsigned char*)&viewer_host_[0], view_device_.cols (), view_device_.rows ());
		image_viewer_.spinOnce();


		if(count >= 198) getchar();
		getchar();
	}

	//pose_tracker->writePoseToFile("Pose.txt");

	getchar();
	return 0;
}