#ifndef KINECT_COLOR_CAMERA_H_
#define KINECT_COLOR_CAMERA_H_

#include <iostream>
#include <fstream>
#include "camera.h"
#include "kinectNui.h"


using namespace std;

class KinectColorCamera: public Camera
{
public:

	//KinectColorCamera(string calibFileName, Kinect::KinectFinder* kinectFinder);
	KinectColorCamera(string calibFileName);  //constructor for loading recorded sequences
	virtual ~KinectColorCamera();
	bool start();
	void acquire();
	bool stop();

	//unsigned char* getRawBuffer() { return _kinect->mColorBuffer;};

protected:

private:

	string _serialNumber;
	KinectNui* _kinect;

};
#endif /*KINECT_COLOR_CAMERA_H_*/
