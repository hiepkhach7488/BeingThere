#ifndef KINECTNUI_H_
#define KINECTNUI_H_

#include <windows.h>
#include <XnCppWrapper.h>

#define KINECT_DEPTH_WIDTH 640
#define KINECT_DEPTH_HEIGHT 480
#define KINECT_COLOR_WIDTH 640
#define KINECT_COLOR_HEIGHT 480

class KinectNui
{

public:
	static KinectNui* getInstance();
	virtual ~KinectNui();

	void Nui_Init();
	void Nui_UnInit();
	
	void copyColor(unsigned char* color);
	void copyDepth(float* depth);
	void copyDepth(unsigned short* udepth);
	void copyDepth(unsigned char* ucdepth);

    void Nui_ProcessDepth();
    void Nui_ProcessColor();

protected:
	
	KinectNui();
	static KinectNui* mInst;

	xn::Context mContext;
	xn::DepthGenerator mDepthGenerator;
	xn::ImageGenerator mColorGenerator;

	unsigned short mDepthBuffer[KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT];
	unsigned char mColorBuffer[KINECT_COLOR_WIDTH * KINECT_COLOR_HEIGHT * 3];
};

#endif
