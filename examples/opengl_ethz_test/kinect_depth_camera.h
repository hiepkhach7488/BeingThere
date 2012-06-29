#ifndef KINECT_DEPTH_CAMERA_H_
#define KINECT_DEPTH_CAMERA_H_

#include <iostream>
#include <fstream>
#include "camera.h"
#include "kinectNui.h"



using namespace std;


class KinectDepthCamera: public Camera
{
public:

	//KinectDepthCamera(string calibFileName, Kinect::KinectFinder* kinectFinder);
	KinectDepthCamera(string calibFileName); //constructor for loading recorded sequences
	
	virtual ~KinectDepthCamera();
	bool start();
	void acquire();
	bool stop();

	//string getSerialNumber() { return _serialNumber;};
	//unsigned short* getRawBuffer() { return _kinect->mDepthBuffer;};

protected:

private:

	string _serialNumber;
	KinectNui* _kinect;

};
#endif /*KINECT_DEPTH_CAMERA_H_*/
