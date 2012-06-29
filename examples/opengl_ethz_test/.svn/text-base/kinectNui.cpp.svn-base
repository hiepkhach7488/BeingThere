#include <iostream>
#include "kinectNui.h"

KinectNui* KinectNui::mInst = NULL;

KinectNui* KinectNui::getInstance()
{
	if(mInst == NULL)
		mInst = new KinectNui();
	return mInst;
}

KinectNui::KinectNui()
{
	Nui_Init();
}

KinectNui::~KinectNui()
{
	Nui_UnInit();
}


void KinectNui::Nui_Init()
{
    XnStatus nRetVal = XN_STATUS_OK;
	
	nRetVal = mContext.Init(); 
	// TODO: check error code 
	
	// Create a depth generator 
	nRetVal = mDepthGenerator.Create(mContext);
	// TODO: check error code 
	
	// Set it to VGA maps at 30 FPS 
	XnMapOutputMode mapMode; 
	mapMode.nXRes = XN_VGA_X_RES; //ok it is 640*480
	mapMode.nYRes = XN_VGA_Y_RES; 
	mapMode.nFPS = 30; 
	nRetVal = mDepthGenerator.SetMapOutputMode(mapMode); 
	// TODO: check error code 

	// Create a color generator 
	nRetVal = mColorGenerator.Create(mContext); 
	// TODO: check error code 
	
	// Set it to VGA maps at 30 FPS 
	nRetVal = mColorGenerator.SetMapOutputMode(mapMode); 
	nRetVal = mColorGenerator.SetPixelFormat(XN_PIXEL_FORMAT_RGB24); 
	// TODO: check error code 
	
	//Aligned Depth And Color
	XnBool isSupported = mDepthGenerator.IsCapabilitySupported("AlternativeViewPoint");
	if(TRUE == isSupported)
	{
		XnStatus res = mDepthGenerator.GetAlternativeViewPointCap().SetViewPoint(mColorGenerator);
		if(XN_STATUS_OK != res)
		{
			printf("Getting and setting AlternativeViewPoint failed: %s\n", xnGetStatusString(res));
		}
		else 
			printf("Getting and setting AlternativeViewPoint successed: %s\n", xnGetStatusString(res));
	} 

	// Start generating 
	nRetVal = mContext.StartGeneratingAll(); 
	// TODO: check error code 
}



void KinectNui::Nui_UnInit( )
{
	mContext.Shutdown();
 }

void KinectNui::Nui_ProcessDepth()
{
    XnStatus nRetVal = XN_STATUS_OK;

    // Update to next frame
	nRetVal = mContext.WaitOneUpdateAll(mDepthGenerator);
	// TODO: check error code 
	
	const XnDepthPixel* pDepthMap = mDepthGenerator.GetDepthMap();
	
	for(int i=0; i<KINECT_DEPTH_WIDTH*KINECT_DEPTH_HEIGHT;i++)
		mDepthBuffer[i] = unsigned short(pDepthMap[i]);
}

void KinectNui::Nui_ProcessColor()
{
    XnStatus nRetVal = XN_STATUS_OK;

    // Update to next frame
	nRetVal = mContext.WaitOneUpdateAll(mColorGenerator);
	// TODO: check error code 
	
	const XnRGB24Pixel* pImageMap = mColorGenerator.GetRGB24ImageMap();
	for(int i=0; i<KINECT_DEPTH_WIDTH*KINECT_DEPTH_HEIGHT;i++)
	{
		mColorBuffer[i*3] = pImageMap[i].nRed;
		mColorBuffer[i*3+1] = pImageMap[i].nGreen;
		mColorBuffer[i*3+2] = pImageMap[i].nBlue;
	}

}

void KinectNui::copyColor(unsigned char* color)
{
	memcpy(color,mColorBuffer,KINECT_DEPTH_WIDTH*KINECT_DEPTH_HEIGHT*3*sizeof(unsigned char));
}

void KinectNui::copyDepth(float* depth)
{
	bool flipped = false;//mContext.GetGlobalMirror();
	//std::cout << flipped << std::endl;
	/*for(int i=0; i<KINECT_DEPTH_WIDTH;i++)
		for(int j=0; j<=KINECT_DEPTH_HEIGHT;j++)
		{
			//if(i%1000 == 0)
			//	std::cout << mDepthBuffer[i] << " ";
			//in case with flip the depth map
			if(flipped)
				depth[i*KINECT_DEPTH_HEIGHT+j] = float(mDepthBuffer[(KINECT_DEPTH_WIDTH-i-1)*KINECT_DEPTH_HEIGHT+j]);
			else
				depth[i*KINECT_DEPTH_HEIGHT+j] = float(mDepthBuffer[i*KINECT_DEPTH_HEIGHT+j]);
		}*/
	for(int i=0; i<KINECT_DEPTH_HEIGHT;i++)
		for(int j=0; j<=KINECT_DEPTH_WIDTH;j++)
		{
			//if(i%1000 == 0)
			//	std::cout << mDepthBuffer[i] << " ";
			//in case with flip the depth map
			if(flipped)
				depth[i*KINECT_DEPTH_WIDTH+j] = float(mDepthBuffer[i*KINECT_DEPTH_WIDTH+(KINECT_DEPTH_WIDTH-j-1)]);
			else
				depth[i*KINECT_DEPTH_WIDTH+j] = float(mDepthBuffer[i*KINECT_DEPTH_WIDTH+j]);
		}
	//std::cout << std::endl;
}


void KinectNui::copyDepth(unsigned short* uDepth){
	bool flipped = false;

	for(int i=0; i<KINECT_DEPTH_HEIGHT;i++)
		for(int j=0; j<=KINECT_DEPTH_WIDTH;j++)
		{
			if(flipped)
				uDepth[i*KINECT_DEPTH_WIDTH+j] = mDepthBuffer[i*KINECT_DEPTH_WIDTH+(KINECT_DEPTH_WIDTH-j-1)];
			else
				uDepth[i*KINECT_DEPTH_WIDTH+j] = mDepthBuffer[i*KINECT_DEPTH_WIDTH+j];
		}
}

void KinectNui::copyDepth(unsigned char* ucDepth){
	bool flipped = false;

	for(int i=0; i<KINECT_DEPTH_HEIGHT;i++)
		for(int j=0; j<=KINECT_DEPTH_WIDTH;j++)
		{
			if(flipped)
				ucDepth[i*KINECT_DEPTH_WIDTH+j] = mDepthBuffer[i*KINECT_DEPTH_WIDTH+(KINECT_DEPTH_WIDTH-j-1)] >> 3;
			else
				ucDepth[i*KINECT_DEPTH_WIDTH+j] = mDepthBuffer[i*KINECT_DEPTH_WIDTH+j] >> 3;
		}
}
