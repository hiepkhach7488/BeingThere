#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <time.h>

#include <pcl/visualization/image_viewer.h>

#include <GL/glew.h>
#include <GL/GL.h>
#include <GL/freeglut.h>

#include <cv.h>
#include <highgui.h>

#include "openni_capture.h"

using namespace std;

using namespace pcl;
using namespace pcl::gpu;

////////////////////////////////////////////////////////////
//GL_DISPLAY HIGHER_FRAME_RATE

#define MAX_CAM 6
#define KINECTS_PER_ROW 2

CaptureOpenNI capture[MAX_CAM];
PtrStepSz<const unsigned short> depth[MAX_CAM];
PtrStepSz<const KinfuTracker::PixelRGB> rgb24[MAX_CAM];

//textures
GLuint colorTex[MAX_CAM];
GLuint depthTex[MAX_CAM];
GLuint colorPBO[MAX_CAM];
GLuint depthPBO[MAX_CAM];

int numDev;

void glutIdle (void)
{
	// Display the frame
	glutPostRedisplay();
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
	//if(key=='s'&&!capture) pthread_create(&countThread, NULL, count, NULL);
}

void glutDisplay (void)
{
	for(int i=0; i<numDev; ++i){
		capture[i].grab(depth[i], rgb24[i]);

		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, colorTex[i]);
		if(true) {
			//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, pImage);
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, colorPBO[i]);
			void * ptr = glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
			if(ptr != NULL) {
				memcpy(ptr, rgb24[i].data, 640*480*3);
				glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
				glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_RGB, GL_UNSIGNED_BYTE, 0);

			}
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
		}

		int rows = ceil((float)numDev/KINECTS_PER_ROW);
		int gridX = i%KINECTS_PER_ROW;
		int gridY = i/KINECTS_PER_ROW;
		float imgWidth = 1.0/(KINECTS_PER_ROW*2.0);
		float imgHeight = 1.0/rows;

		float coordXColor = (float)gridX/KINECTS_PER_ROW;
		float coordYColor = (rows-(float)gridY-1)/rows;

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

		//glBegin(GL_QUADS);
		//	glTexCoord2f(0,1);
		//	glVertex2f(0, 0);
		//	glTexCoord2f(1,1);
		//	glVertex2f(0.5, 0);
		//	glTexCoord2f(1,0);
		//	glVertex2f(0.5, 1);
		//	glTexCoord2f(0,0);
		//	glVertex2f(0, 1);	
		//glEnd();


		glBindTexture(GL_TEXTURE_2D, depthTex[i]);
		if(true) {
			//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, depthTemp);
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, depthPBO[i]);
			void * ptr = glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
			if(ptr != NULL) {
				memcpy(ptr, depth[i].data, 640*480*2);
				glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
				glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_GREEN, GL_UNSIGNED_SHORT, 0);	
			}
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
		}

		float coordXDepth = (2.0*gridX+1)/(KINECTS_PER_ROW*2);
		float coordYDepth = (rows-(float)gridY-1)/rows;

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

		//glBegin(GL_QUADS);
		//	glTexCoord2f(0,1);
		//	glVertex2f(0.5, 0);
		//	glTexCoord2f(1,1);
		//	glVertex2f(1, 0);
		//	glTexCoord2f(1,0);
		//	glVertex2f(1, 1);
		//	glTexCoord2f(0,0);
		//	glVertex2f(0.5, 1);	
		//glEnd();

		glDisable(GL_TEXTURE_2D);
	}

	glutSwapBuffers();
}

void glDisplay(int argc, char* argv[]){
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
	unsigned short* dummyDepth = (unsigned short*) malloc(640*480*2);
	memset(dummyDepth, 0, 640*480*2);

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
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16, 640, 480, 0, GL_GREEN, GL_UNSIGNED_SHORT, dummyDepth);
	}

	//create PBOs
	glGenBuffers(numDev, colorPBO);
	glGenBuffers(numDev, depthPBO);
	for(int i = 0; i < numDev; i++) {
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, colorPBO[i]);
		glBufferData(GL_PIXEL_UNPACK_BUFFER, 640*480*3, dummyTex, GL_DYNAMIC_DRAW);

		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, depthPBO[i]);
		glBufferData(GL_PIXEL_UNPACK_BUFFER, 640*480*2, dummyDepth, GL_DYNAMIC_DRAW);
	}

	//create opencv images
	//saveDepth = cvCreateImage(cvSize(640, 480), IPL_DEPTH_16U, 1);
	//saveColor = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);

	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);
	glutReshapeFunc (&ReSizeGLScene);

	// Per frame code is in glutDisplay
	glutMainLoop();
}

int main(int argc, char* argv[]){
	if(argc < 2) {
		cout << "Usage: prog -numDev" << endl;
	}
	else {
		numDev = atoi(argv[1]);
		cout << "Numdev:= " << numDev << endl;
	}

	for(int i=0; i<numDev; ++i){
		capture[i].open(i);
		capture[i].setRegistration(true);

	}	
	//PCL display
	//pclDisplay();
	//return 0;

	//GL display
	glDisplay(argc, argv);
	return 0;
}