#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>

#include "openni_capture.h"
#include "kinect_visualization.h"

#include "pcl/common/angles.h"
#include "ImageSequence.h"
#include "kinect_capture.h"
#include "volumemetric_rendering.h"

#include <boost/algorithm/string.hpp>

using namespace std;

using namespace pcl::gpu;

static void keyboard_callback (const visualization::KeyboardEvent &e, void *cookie){
	KinectCapture* app = reinterpret_cast<KinectCapture*> (cookie);
	int key = e.getKeyCode ();

	if (e.keyUp ()){
		switch (key){
			case 27: 
				exit(0); 
				break;
			case 'c': 
				app->pushViews(500);
				break;
			case 's':
				app->saveViews();
				break;
			case 't':
				app->testQuality();
				break;
			case 'x':
				app->clearViews();
				break;
		}
	}
}

void saveData(){
	KinectCapture capture_;
	capture_.open(0);

	PtrStepSz<const unsigned short> depth;
	PtrStepSz<const KinfuTracker::PixelRGB> rgb24;

	KinectVisualization kvis;
	//kvis.setViewerPose(kvis.cloud_viewer_xyz_, kinfu_.getCameraPose());
	kvis.viewerColor_.registerKeyboardCallback(keyboard_callback, (void*)(&capture_));

	while(capture_.grab (depth, rgb24)){
		kvis.showDepth(depth);
		kvis.viewerDepth_.spinOnce();
		kvis.showRGB(rgb24);
		kvis.viewerColor_.spinOnce();
	}
}

#include "fusion_renderer.h"
void test(){
	ImageSequence* imgCapture = new ImageSequence();
	imgCapture->init("C:/Logs/PCL/KinectFusion/build/tools/data/depth", ".png", "img_depth_");
	imgCapture->open();
	imgCapture->captureStart();
	imgCapture->captureNext();

	IplImage* depthImg = imgCapture->image();
	KinfuTracker::DepthMap depth_device_;
	depth_device_.upload(depthImg->imageData, 640*2, 480, 640);

	FusionRenderer fusionRenderer(480, 640);
	//fusionRenderer.selfInit();
	fusionRenderer.testFusionAlgorithm(depth_device_);

	//Next Image
	while(true){
		imgCapture->captureNext();
		depthImg = imgCapture->image();
		depth_device_.upload(depthImg->imageData, 640*2, 480, 640);
		fusionRenderer.testFusionAlgorithm(depth_device_);
	}

	getchar();
}

int main(int argc, char* argv[]){	
	test();
	return 0;

	KinectCapture capture_;
	//capture_.open(0);
	//cout << capture_.depth_focal_length_VGA << endl;
	//getchar();

	PtrStepSz<const unsigned short> depth;
    PtrStepSz<const KinfuTracker::PixelRGB> rgb24;

	KinectVisualization kvis;
	
	KinfuTracker kinfu_;
	KinfuTracker::DepthMap depth_device_;

	Eigen::Vector3f volume_size = Vector3f::Constant (3.f/*meters*/);
	//float f = capture_.depth_focal_length_VGA;
	float f = 575.816;
    kinfu_.setDepthIntrinsics (f, f);
    kinfu_.volume().setSize (volume_size);

    Eigen::Matrix3f R = Eigen::Matrix3f::Identity ();   // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
    Eigen::Vector3f t = volume_size * 0.5f - Vector3f (0, 0, volume_size (2) / 2 * 1.2f);

    Eigen::Affine3f pose = Eigen::Translation3f (t) * Eigen::AngleAxisf (R);

    kinfu_.setInitalCameraPose (pose);
    kinfu_.volume().setTsdfTruncDist (0.030f/*meters*/);    
    kinfu_.setIcpCorespFilteringParams (0.1f/*meters*/, sin ( pcl::deg2rad(20.f) ));
    //kinfu_.setDepthTruncationForICP(5.f/*meters*/);
    kinfu_.setCameraMovementThreshold(0.001f);

	kvis.setViewerPose(kvis.cloud_viewer_xyz_, kinfu_.getCameraPose());
	kvis.viewerColor_.registerKeyboardCallback(keyboard_callback, (void*)(&capture_));

	VolumetricRendering* volRenderer = new VolumetricRendering(&capture_, &kinfu_);
	//volRenderer->showAll();

	ImageSequence* imgCapture = new ImageSequence();
	//imgCapture->init("C:/Logs/PCL/PCLTestProject/build/data/depth", ".png", "Depth");
	imgCapture->init("C:/Logs/PCL/KinectFusion/build/tools/data/depth", ".png", "img_depth_");
	imgCapture->open();
	imgCapture->captureStart();
	imgCapture->captureNext();

	IplImage* depthImg = 0;

	//while(capture_.grab (depth, rgb24)){
	while(true){
		depthImg = imgCapture->image();
		imgCapture->captureNext();

		//depth.cols = 640;
		//depth.rows = 480;
		////depth.data = (unsigned short*)depthImg->imageData;
		//depth.step = 640 * depth.elemSize ();
		//
		//kvis.showDepth(depth);
		//kvis.viewerDepth_.spinOnce();
		//getchar();
		//kvis.showRGB(rgb24);
		//kvis.viewerColor_.spinOnce();

		//depth_device_.upload (depth.data, depth.step, depth.rows, depth.cols);
		depth_device_.upload(depthImg->imageData, 640*2, 480, 640);
		bool has_image = kinfu_ (depth_device_);

		if(has_image){
			volRenderer->renderTSDF(0);
			volRenderer->renderTSDFCloud(kinfu_, false);
		}


		//kvis.showCloudXYZ(kinfu_);
		//kvis.cloud_viewer_xyz_.spinOnce();
	}

	getchar();
	return 0;
}