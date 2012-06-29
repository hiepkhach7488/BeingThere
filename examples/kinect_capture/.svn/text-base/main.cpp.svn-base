//#include <stdio.h>
//#include <iostream>
//#include <stdlib.h>
//#include <time.h>
//
//#include <pcl/visualization/image_viewer.h>
//
//#include <GL/glew.h>
//#include <GL/GL.h>
//#include <GL/freeglut.h>
//
//#include <cv.h>
//#include <highgui.h>
//
//#include "openni_capture.h"
//
//using namespace std;
//
//using namespace pcl;
//using namespace pcl::gpu;
//
//#define COLOR_WIDTH 1280
//#define COLOR_HEIGHT 1024
//#define SAVE_HEIGHT 960
//
//vector<IplImage> colorVec;
//vector<IplImage> depthVec;
//
//CaptureOpenNI capture_;
//PtrStepSz<const unsigned short> depth;
//PtrStepSz<const KinfuTracker::PixelRGB> rgb24;
//IplImage* colorImg;
//IplImage* depthImg;
//
//IplImage* tempImg;
//IplImage* scaleImg;
//
//char buf[4096];
//int save_params[] = {CV_IMWRITE_JPEG_QUALITY, 100, 0};
//
//static void keyboard_callback (const visualization::KeyboardEvent &e, void *cookie){
//	//KinectCapture* app = reinterpret_cast<KinectCapture*> (cookie);
//	int key = e.getKeyCode ();
//
//	if (e.keyUp ()){
//		switch (key){
//			case 27: 
//				exit(0); 
//				break;
//			case 'p': 
//				tempImg = cvCreateImage(cvSize(COLOR_WIDTH, COLOR_HEIGHT), 8, 3);
//				colorImg = cvCreateImage(cvSize(COLOR_WIDTH, COLOR_HEIGHT), 8, 1);
//				memcpy(tempImg->imageData, rgb24.data, rgb24.rows*rgb24.cols*3);
//				cvCvtColor(tempImg, colorImg, CV_BGR2GRAY);
//				
//				#ifdef SAVE_HEIGHT
//				scaleImg = cvCreateImage(cvSize(COLOR_WIDTH, SAVE_HEIGHT), 8, 1);
//				cvResize(colorImg, scaleImg, CV_INTER_LINEAR);
//				colorVec.push_back(*scaleImg);
//				#endif
//				#ifndef SAVE_HEIGHT
//				colorVec.push_back(*colorImg);
//				#endif
//				break;
//
//			case 's':				
//				for(int i=0; i<colorVec.size(); ++i){
//					sprintf (buf, "data/img_%s_%d.png", "rgb", (int)i);
//					cvSaveImage(buf, &colorVec[i], save_params);
//				}
//				break;
//
//			case 't':
//			
//				break;
//
//			case 'x':
//				colorVec.clear();
//				break;
//		}
//	}
//}
//
//void pclDisplay(){
//	//cout << "base_line: " << capture_.baseline << " " << capture_.depth_focal_length_VGA << " " 
//	//	<< capture_.max_depth << " " << capture_.pixelSize << " " << capture_.no_sample_value << endl;
//
//	visualization::ImageViewer viewerDepth_;
//	visualization::ImageViewer viewerColor_;
//
//	viewerDepth_.setWindowTitle("Kinect Depth Window");
//	viewerColor_.setWindowTitle("Kinect Color Window");
//
//	viewerDepth_.registerKeyboardCallback(keyboard_callback);
//	viewerColor_.registerKeyboardCallback(keyboard_callback);
//	
//	clock_t startTime, stopTime;
//	int frameCount = 0;
//	startTime = clock();
//
//	while(capture_.grab(depth, rgb24)){
//		viewerDepth_.showShortImage (depth.data, depth.cols, depth.rows, 0, 5000, true); 
//		viewerColor_.showRGBImage(&rgb24.data->r, rgb24.cols, rgb24.rows);
//		//cout << depth.rows << " " << depth.cols << " " << rgb24.rows << " " << rgb24.cols << endl;
//		viewerColor_.spinOnce();
//		viewerDepth_.spinOnce();
//
//		frameCount++;
//
//		if(frameCount == 50){
//			stopTime = clock();
//			double period = ( (double)stopTime - startTime) / CLOCKS_PER_SEC;
//			cout << "FPS=" << 50.f/period << endl;
//
//			startTime = clock();
//			frameCount = 0;
//		}
//	}
//}
//
//
//////////////////////////////////////////////////////////////
////GL_DISPLAY HIGHER_FRAME_RATE
//#define MAX_CAM 1
//#define KINECTS_PER_ROW 1
//
////textures
//GLuint colorTex[MAX_CAM];
//GLuint depthTex[MAX_CAM];
//GLuint colorPBO[MAX_CAM];
//GLuint depthPBO[MAX_CAM];
//
//void glutIdle (void)
//{
//	// Display the frame
//	glutPostRedisplay();
//}
//
//void ReSizeGLScene (int Width, int Height)
//{
//  glViewport (0, 0, Width, Height);
//  glMatrixMode (GL_PROJECTION);
//  glLoadIdentity ();
//  glOrtho (0, 1, 0, 1, -1, 1);
//  glMatrixMode (GL_MODELVIEW);
//}
//
//void glutKeyboard (unsigned char key, int x, int y)
//{
//	if(key==27) exit(0);
//	//if(key=='s'&&!capture) pthread_create(&countThread, NULL, count, NULL);
//	if(key == 'p'){
//		for(int i=0; i<600; ++i){
//			if(i%2 !=0 ) continue;
//			capture_.grab(depth, rgb24);
//			depthImg = cvCreateImage(cvSize(640,480), 16, 1);
//			colorImg = cvCreateImage(cvSize(640,480), 8, 3);
//			memcpy(depthImg->imageData, depth.data, depth.rows*depth.cols*2);
//			memcpy(colorImg->imageData, rgb24.data, rgb24.rows*rgb24.cols*3);
//			colorVec.push_back(*colorImg);
//			depthVec.push_back(*depthImg);
//			//glutPostRedisplay();
//		}
//	}
//
//	if(key == 's'){
//		for(int i=0; i<colorVec.size(); ++i){
//			sprintf (buf, "data/color/img_%s_%d.png", "rgb", (int)i);
//			cvSaveImage(buf, &colorVec[i], save_params);
//			sprintf (buf, "data/depth/img_%s_%d.png", "depth", (int)i);
//			cvSaveImage(buf, &depthVec[i], save_params);
//		}
//	}
//
//}
//
//void glutDisplay (void)
//{
//	capture_.grab(depth, rgb24);
//
//	glEnable(GL_TEXTURE_2D);
//	glBindTexture(GL_TEXTURE_2D, colorTex[0]);
//	if(true) {
//		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb24.data);
//		/*glBindBuffer(GL_PIXEL_UNPACK_BUFFER, colorPBO[0]);
//		void * ptr = glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
//		if(ptr != NULL) {
//			memcpy(ptr, rgb24.data, 640*480*3);
//			glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
//			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_RGB, GL_UNSIGNED_BYTE, 0);
//
//		}
//		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);*/
//	}
//
//	glBegin(GL_QUADS);
//		glTexCoord2f(0,1);
//		glVertex2f(0, 0);
//		glTexCoord2f(1,1);
//		glVertex2f(0.5, 0);
//		glTexCoord2f(1,0);
//		glVertex2f(0.5, 1);
//		glTexCoord2f(0,0);
//		glVertex2f(0, 1);	
//	glEnd();
//
//
//	glBindTexture(GL_TEXTURE_2D, depthTex[0]);
//	if(true) {
//		//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, depthTemp);
//		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, depthPBO[0]);
//		void * ptr = glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
//		if(ptr != NULL) {
//			memcpy(ptr, depth.data, 640*480*2);
//			glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
//			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_GREEN, GL_UNSIGNED_SHORT, 0);	
//		}
//		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
//	}
//
//	glBegin(GL_QUADS);
//		glTexCoord2f(0,1);
//		glVertex2f(0.5, 0);
//		glTexCoord2f(1,1);
//		glVertex2f(1, 0);
//		glTexCoord2f(1,0);
//		glVertex2f(1, 1);
//		glTexCoord2f(0,0);
//		glVertex2f(0.5, 1);	
//	glEnd();
//	glDisable(GL_TEXTURE_2D);
//
//	glutSwapBuffers();
//}
//
//void glDisplay(int argc, char* argv[]){
//	// OpenGL init
//	glutInit(&argc, argv);
//	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
//	
//	int numDev = 1;
//	int initWidth = (640*KINECTS_PER_ROW*2);
//	int initHeight = (480*ceil((float)numDev/KINECTS_PER_ROW));	
//
//	glutInitWindowSize(initWidth, initHeight);
//	glutCreateWindow ("OpenNI Kinect Test");
//	//glutFullScreen();
//
//	glewInit();
//
//	//create textures
//	unsigned char * dummyTex = (unsigned char *)malloc(640*480*3);
//	memset(dummyTex, 0, 640*480*3);
//	unsigned short* dummyDepth = (unsigned short*) malloc(640*480*2);
//	memset(dummyDepth, 0, 640*480*2);
//
//	//create textures
//	glGenTextures(numDev, colorTex);
//	glGenTextures(numDev, depthTex);
//	for(int i = 0; i < numDev; i++) {
//		glBindTexture(GL_TEXTURE_2D, colorTex[i]);
//		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, dummyTex);
//
//		glBindTexture(GL_TEXTURE_2D, depthTex[i]);
//		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16, 640, 480, 0, GL_GREEN, GL_UNSIGNED_SHORT, dummyDepth);
//	}
//
//	//create PBOs
//	glGenBuffers(numDev, colorPBO);
//	glGenBuffers(numDev, depthPBO);
//	for(int i = 0; i < numDev; i++) {
//		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, colorPBO[i]);
//		glBufferData(GL_PIXEL_UNPACK_BUFFER, 640*480*3, dummyTex, GL_DYNAMIC_DRAW);
//
//		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, depthPBO[i]);
//		glBufferData(GL_PIXEL_UNPACK_BUFFER, 640*480*2, dummyDepth, GL_DYNAMIC_DRAW);
//	}
//
//	//create opencv images
//	//saveDepth = cvCreateImage(cvSize(640, 480), IPL_DEPTH_16U, 1);
//	//saveColor = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
//
//	glutKeyboardFunc(glutKeyboard);
//	glutDisplayFunc(glutDisplay);
//	glutIdleFunc(glutIdle);
//	glutReshapeFunc (&ReSizeGLScene);
//
//	// Per frame code is in glutDisplay
//	glutMainLoop();
//}
//
//int main(int argc, char* argv[]){
//	capture_.open(0);
//	capture_.setRegistration(true);
//
//	//PCL display
//	//pclDisplay();
//	//return 0;
//
//	//GL display
//	glDisplay(argc, argv);
//	return 0;
//}









/*****************************************************************************
*                                                                            *
*  OpenNI 1.0 Alpha                                                          *
*  Copyright (C) 2010 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  OpenNI is free software: you can redistribute it and/or modify            *
*  it under the terms of the GNU Lesser General Public License as published  *
*  by the Free Software Foundation, either version 3 of the License, or      *
*  (at your option) any later version.                                       *
*                                                                            *
*  OpenNI is distributed in the hope that it will be useful,                 *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of            *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              *
*  GNU Lesser General Public License for more details.                       *
*                                                                            *
*  You should have received a copy of the GNU Lesser General Public License  *
*  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.            *
*                                                                            *
*****************************************************************************/




//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#define GL_GLEXT_PROTOTYPES 1
#include <XnOS.h>
#include <XnCppWrapper.h>
#include <math.h>

#include <time.h>

#include <GL/glew.h>
#include <GL/GL.h>
#include <GL/freeglut.h>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

using namespace xn;

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define MAX_CAM 32
#define KINECTS_PER_ROW 2

Context g_context;
DepthGenerator g_depth[MAX_CAM];
ImageGenerator g_image[MAX_CAM];
DepthMetaData g_depthMD[MAX_CAM];
ImageMetaData g_imageMD[MAX_CAM];
int numDev;

#define MAX_DEPTH 10000
float g_pDepthHist[MAX_DEPTH];
unsigned char depthTemp[3*640*480];

//textures
GLuint colorTex[MAX_CAM];
GLuint depthTex[MAX_CAM];
GLuint colorPBO[MAX_CAM];
GLuint depthPBO[MAX_CAM];

//timing
long frame = 0;

int cap_frames = 150;
int session = 0;
int capFrame = 0;
int capture = 0;
IplImage *saveDepth, *saveColor;

int cap_delay_sec = 10;

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------


void glutIdle (void)
{
	// Display the frame
	glutPostRedisplay();
}


void glutDisplay (void)
{
  	// Read a new frame
	XnStatus rc = XN_STATUS_OK;
	rc = g_context.WaitAndUpdateAll();
	if (rc != XN_STATUS_OK)
	{
		printf("Read failed: %s\n", xnGetStatusString(rc));
		return;
	}

	glClear (GL_COLOR_BUFFER_BIT);
  	glLoadIdentity ();

	for(int i = 0; i < numDev; i++) {
		g_image[i].GetMetaData(g_imageMD[i]);
		g_depth[i].GetMetaData(g_depthMD[i]);

		const XnDepthPixel* pDepth = g_depthMD[i].Data();
		const XnUInt8* pImage = g_imageMD[i].Data();

		//
		//process depth map into scaled 8-bit RGB image
		//
		if(g_depth[i].IsDataNew()) {
			/*
			//create histogram
			const XnDepthPixel* dPtr = pDepth; 
			xnOSMemSet(g_pDepthHist, 0, MAX_DEPTH*sizeof(float));
			unsigned int nNumberOfPoints = 0;
			for (XnUInt y = 0; y < g_depthMD[i].YRes(); ++y) {
				for (XnUInt x = 0; x < g_depthMD[i].XRes(); ++x, ++dPtr) {
					if (*dPtr != 0) {
						g_pDepthHist[*dPtr]++;
						nNumberOfPoints++;
					}
				}
			}
			for (int nIndex=1; nIndex<MAX_DEPTH; nIndex++) {
				g_pDepthHist[nIndex] += g_pDepthHist[nIndex-1];
			}
			if (nNumberOfPoints) {
				for (int nIndex=1; nIndex<MAX_DEPTH; nIndex++) {
					g_pDepthHist[nIndex] = (unsigned int)(256 * (1.0f - (g_pDepthHist[nIndex] / nNumberOfPoints)));
				}
			}*/
		
			//process depth values
			int imgSize = 640*480;
			for(int j = 0; j < imgSize; j++) {
				//int nHistValue = g_pDepthHist[pDepth[j]];
				depthTemp[3*j+0] = pDepth[j]>>3;//0;
				depthTemp[3*j+1] = pDepth[j]>>3;//nHistValue;	
				depthTemp[3*j+2] = pDepth[j]>>3;//0;
			}
		}
		//
		//done processing depth image
		//

		int rows = ceil((float)numDev/KINECTS_PER_ROW);
		int gridX = i%KINECTS_PER_ROW;
		int gridY = i/KINECTS_PER_ROW;
		float imgWidth = 1.0/(KINECTS_PER_ROW*2.0);
		float imgHeight = 1.0/rows;

		glEnable(GL_TEXTURE_2D);

		//draw color image
		float coordXColor = (float)gridX/KINECTS_PER_ROW;
		float coordYColor = (rows-(float)gridY-1)/rows;

		glBindTexture(GL_TEXTURE_2D, colorTex[i]);
		if(g_image[i].IsDataNew()) {
			//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, pImage);
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, colorPBO[i]);
			void * ptr = glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
			if(ptr != NULL) {
				memcpy(ptr, pImage, 640*480*3);
				glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
				glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_RGB, GL_UNSIGNED_BYTE, 0);
				
			}
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
		}

		glBegin(GL_QUADS);
			glTexCoord2f(0,1);
			glVertex2f(coordXColor, coordYColor);
			glTexCoord2f(1,1);
			glVertex2f(coordXColor+imgWidth, coordYColor);
			glTexCoord2f(1,0);
			glVertex2f(coordXColor+imgWidth, coordYColor+imgHeight);
			glTexCoord2f(0,0);
			glVertex2f(coordXColor, coordYColor+imgHeight);	
		glEnd();

		//draw depth image
		float coordXDepth = (2.0*gridX+1)/(KINECTS_PER_ROW*2);
		float coordYDepth = (rows-(float)gridY-1)/rows;

		glBindTexture(GL_TEXTURE_2D, depthTex[i]);
		if(g_depth[i].IsDataNew()) {
			//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, depthTemp);
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, depthPBO[i]);
			void * ptr = glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
			if(ptr != NULL) {
				memcpy(ptr, depthTemp, 640*480*3);
				glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
				glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_RGB, GL_UNSIGNED_BYTE, 0);	
			}
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
		}
		glBegin(GL_QUADS);
			glTexCoord2f(0,1);
			glVertex2f(coordXDepth, coordYDepth);
			glTexCoord2f(1,1);
			glVertex2f(coordXDepth+imgWidth, coordYDepth);
			glTexCoord2f(1,0);
			glVertex2f(coordXDepth+imgWidth, coordYDepth+imgHeight);
			glTexCoord2f(0,0);
			glVertex2f(coordXDepth, coordYDepth+imgHeight);	
		glEnd();


		glDisable(GL_TEXTURE_2D);

	}

	if(capture) {
		capFrame++;
		if(capFrame == cap_frames) {
			capFrame = 0;
			capture = 0;
			session++;
		}
	}

   	glutSwapBuffers ();

	frame++;
}

void ReSizeGLScene (int Width, int Height)
{
  glViewport (0, 0, Width, Height);
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();
  glOrtho (0, 1, 0, 1, -1, 1);
  glMatrixMode (GL_MODELVIEW);
}


void glutKeyboard (unsigned char key, int x, int y)
{
	if(key==27) exit(0);
}

int initOpenNI(Context context) {


	XnMapOutputMode mapMode;
	mapMode.nXRes = XN_VGA_X_RES;
	mapMode.nYRes = XN_VGA_Y_RES;
	mapMode.nFPS = 30;

	XnNodeHandle depthDeviceHandle[MAX_CAM];

        int numDevDepth = 0;
        NodeInfoList list1;
        XnStatus rc = context.EnumerateProductionTrees(XN_NODE_TYPE_DEPTH, NULL, list1, NULL);

	if (rc == XN_STATUS_OK) {
		for (NodeInfoList::Iterator it = list1.Begin(); it != list1.End(); ++it) {
			NodeInfo n = *it;

			NodeInfoList listDep = n.GetNeededNodes();
			printf("Depth %d:\n",numDevDepth);
			for (NodeInfoList::Iterator it2 = listDep.Begin(); it2 != listDep.End(); ++it2) {
				ProductionNode node;
				NodeInfo nDep = *it2;
				context.CreateProductionTree(nDep);
				nDep.GetInstance(node);
				node.GetInfo();
				if(nDep.GetDescription().Type == XN_NODE_TYPE_DEVICE)
					depthDeviceHandle[numDevDepth] = XnNodeHandle(node);
				printf("\tDependency: type = %d, address: %p\n", nDep.GetDescription().Type, depthDeviceHandle[numDevDepth]);
	
			}

			context.CreateProductionTree(n);
			n.GetInstance(g_depth[numDevDepth]);
			g_depth[numDevDepth].SetMapOutputMode(mapMode);
			
                        numDevDepth++;
		}
	} else {
		printf ("Error querying depth devices\n");
		return 1;
	}

	int count = 0;
	int numDevImage = 0;
	NodeInfoList list2;
	rc = context.EnumerateProductionTrees(XN_NODE_TYPE_IMAGE, NULL, list2, NULL);
	if (rc == XN_STATUS_OK) {
		for(int i = 0; i < numDevDepth; i++) {
			int count = 0;
			bool found = 0;
			for (NodeInfoList::Iterator it = list2.Begin(); it != list2.End()&&!found; ++it) {
				NodeInfo n = *it;
				NodeInfoList listDep = n.GetNeededNodes();
				for (NodeInfoList::Iterator it2 = listDep.Begin(); it2 != listDep.End(); ++it2) {
					ProductionNode node;
					NodeInfo nDep = *it2;
					context.CreateProductionTree(nDep);
					nDep.GetInstance(node);
					node.GetInfo();
					XnNodeHandle handle = XnNodeHandle(node);
					if(handle == depthDeviceHandle[i]) {
						found = 1;
					}					
				}
				if(found) {
					context.CreateProductionTree(n);
					n.GetInstance(g_image[numDevImage]);			
					g_image[numDevImage].SetMapOutputMode(mapMode);

					numDevImage++;
				}
				count++;
			}
			//if(!found) {
			//	printf("Couldn't find image device for depth device %d\n", i);
			//	exit(0);
			//}
		}
	} else {
		printf ("Error querying rgb devices\n");
		exit(0);
	}
	return  numDevDepth;
}

int main(int argc, char* argv[])
{

	XnStatus rc;
	EnumerationErrors errors;
	rc = g_context.Init();
	if (rc == XN_STATUS_NO_NODE_PRESENT)
	{
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return (rc);
	}
	else if (rc != XN_STATUS_OK)
	{
		printf("Open failed: %s\n", xnGetStatusString(rc));
		return (rc);
	}

	numDev = initOpenNI(g_context);

	printf("Num devices: %d\n", numDev);


	for(int i = 0; i < numDev; i++) {
		g_depth[i].GetAlternativeViewPointCap().SetViewPoint(g_image[i]);
	}

	g_context.StartGeneratingAll();

	// RGB is the only image format supported.
	if (g_imageMD[0].PixelFormat() != XN_PIXEL_FORMAT_RGB24)
	{
		printf("The device image format must be RGB24\n");
		return 1;
	}

	// OpenGL init
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);

	int initWidth = (640*KINECTS_PER_ROW*2);
	int initHeight = (480*ceil((float)numDev/KINECTS_PER_ROW));	

	glutInitWindowSize(initWidth, initHeight);
	glutCreateWindow ("OpenNI Kinect Test");
	//glutFullScreen();
	glewInit();

	//create textures
	unsigned char * dummyTex = (unsigned char *)malloc(640*480*3);
	memset(dummyTex, 0, 640*480*3);

	//create textures
	glGenTextures(numDev, colorTex);
	glGenTextures(numDev, depthTex);
	for(int i = 0; i < numDev; i++) {
		glBindTexture(GL_TEXTURE_2D, colorTex[i]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, dummyTex);

		glBindTexture(GL_TEXTURE_2D, depthTex[i]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, dummyTex);
	}

	//create PBOs
	glGenBuffers(numDev, colorPBO);
	glGenBuffers(numDev, depthPBO);
	for(int i = 0; i < numDev; i++) {
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, colorPBO[i]);
		glBufferData(GL_PIXEL_UNPACK_BUFFER, 640*480*3, dummyTex, GL_DYNAMIC_DRAW);

		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, depthPBO[i]);
		glBufferData(GL_PIXEL_UNPACK_BUFFER, 640*480*3, dummyTex, GL_DYNAMIC_DRAW);
	}

	//create opencv images
	saveDepth = cvCreateImage(cvSize(640, 480), IPL_DEPTH_16U, 1);
	saveColor = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);

	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);
	glutReshapeFunc (&ReSizeGLScene);

	// Per frame code is in glutDisplay
	glutMainLoop();

	return 0;
}
