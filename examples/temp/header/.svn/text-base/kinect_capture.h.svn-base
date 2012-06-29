#ifndef KINECT_CAPTURE_H
#define KINECT_CAPTURE_H

#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/containers/kernel_containers.h>

#include "openni_capture.h"
#include "cv.h"
#include "highgui.h"

using namespace cv;

using namespace pcl::gpu;

class KinectCapture : public pcl::gpu::CaptureOpenNI{
public:
	KinectCapture();
	KinectCapture(int);
	KinectCapture(const std::string& oni_filename);

	~KinectCapture();

	bool pushViews();
	bool pushViews(int);
	bool clearViews();

	int saveViews();

	bool testQuality();

private:
	bool accumuateViews;
	int maxNumViews;
	int curNumViews;
	vector<IplImage> color_images_;
	vector<IplImage> depth_images_;

	vector<cv::Mat> color_;
	vector<cv::Mat> depth_;

	PtrStepSz<const unsigned short> depth;
    PtrStepSz<const KinfuTracker::PixelRGB> rgb24;
};

#endif
