#include "kinect_capture.h"

KinectCapture::KinectCapture() : CaptureOpenNI(){
	accumuateViews = false;
	maxNumViews = 1000;
	curNumViews = 0;
}

KinectCapture::KinectCapture(int devId) : CaptureOpenNI(devId){
	accumuateViews = false;
	maxNumViews = 1000;
	curNumViews = 0;
}

KinectCapture::KinectCapture(const std::string& oni_filename) : CaptureOpenNI(oni_filename){
	accumuateViews = false;
	maxNumViews = 1000;
	curNumViews = 0;
}

KinectCapture::~KinectCapture(){

}

bool KinectCapture::pushViews(){
	if(curNumViews >= maxNumViews) return false;

	if(!grab (depth, rgb24)) return false;
	/*cv::Mat depthMat(depth.rows, depth.cols, CV_16UC1);
	cv::Mat rgbMat(rgb24.rows, rgb24.cols, CV_8UC3);
	memcpy(depthMat.data, depth.data, depth.rows*depth.cols*2);
	memcpy(rgbMat.data, rgb24.data, rgb24.rows*rgb24.cols*3);

	color_.push_back(rgbMat);
	depth_.push_back(depthMat);*/

	IplImage* depthImg = cvCreateImage(cvSize(640,480), 16, 1);
	IplImage* colorImg = cvCreateImage(cvSize(640,480), 8, 3);
	memcpy(depthImg->imageData, depth.data, depth.rows*depth.cols*2);
	memcpy(colorImg->imageData, rgb24.data, rgb24.rows*rgb24.cols*3);
	color_images_.push_back(*colorImg);
	depth_images_.push_back(*depthImg);
	++curNumViews;
	//std::cout << "Cur Views" << curNumViews << std::endl;

	return true;
}

bool KinectCapture::testQuality(){
	if(!grab(depth, rgb24)) return false;
	IplImage* depthImg = cvCreateImage(cvSize(640,480), 16, 1);
	memcpy(depthImg->imageData, depth.data, depth.rows*depth.cols*2);
	int save_params[] = {CV_IMWRITE_JPEG_QUALITY, 100, 0};
	cvSaveImage("test1.png", depthImg, save_params);
	IplImage* loadedImg = cvLoadImage("test1.png", CV_LOAD_IMAGE_UNCHANGED);
	unsigned short* loadedImgPtr = (unsigned short*)loadedImg->imageData;
	//Test Quality
	for(int i=0; i<loadedImg->width; ++i)
		for(int j=0; j < loadedImg->height; ++j){
			int idx = i + j * loadedImg->width;
			//unsigned short* depthValPtr = (unsigned short*)depth.data + idx;
			//unsigned short depthVal = depthValPtr[0];
			unsigned short depthVal = depth.data[idx];
			unsigned short loadedVal = loadedImgPtr[idx];
			if(abs(loadedVal - depthVal) > 1) std::cout << "Different" << depthVal << " " << loadedVal << std::endl;
			else std::cout << "No Different" << depthVal << " " << loadedVal << std::endl;
		}
}

bool KinectCapture::pushViews(int numView){
	for(int i=0; i<numView; ++i)
		if(!pushViews()) return false;
}

bool KinectCapture::clearViews(){
	color_.clear();
	depth_.clear();

	color_images_.clear();
	depth_images_.clear(); 

	return true;
}

int KinectCapture::saveViews(){
	//std::vector<int> params;
	//params.push_back(CV_IMWRITE_JPEG_QUALITY);
	//params.push_back(100);
	//params.push_back(0);
	//int params[] = {CV_IMWRITE_JPEG_QUALITY, 100, 0};

	int save_params[] = {CV_IMWRITE_JPEG_QUALITY, 100, 0};

	char buf[4096];
 
	for(int i=0; i<color_images_.size(); ++i){
		sprintf (buf, "data/color/Color%03d.png", (int)i);
		cvSaveImage(buf, &color_images_[i], save_params);
		//cv::imwrite(buf, color_[i], params);
		sprintf (buf, "data/depth/Depth%03d.png", (int)i);
		cvSaveImage(buf, &depth_images_[i], save_params);
		//cv::imwrite(buf, depth_[i], params);
	}

	return true;
}