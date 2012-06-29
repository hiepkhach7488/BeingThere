#include <Windows.h>

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <time.h>

#include <GL/glew.h>
#include <GL/GL.h>
#include <GL/freeglut.h>

#include <cv.h>
#include <highgui.h>

using namespace std;

#include "NuiApi.h"
#include "nui_sensor.h"

#define MAX_CAM 6
#define KINECTS_PER_ROW 2

int numDev;

NuiSensor* nuiSensor[MAX_CAM];

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
	if(key==27) {
		for(int i=0; i<numDev; ++i)
			nuiSensor[i]->~NuiSensor();
		exit(0);
	}
}

void renderColorTex(int camIdx){
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, nuiSensor[camIdx]->colorTex);

	int rows = ceil((float)numDev/KINECTS_PER_ROW);
	int gridX = camIdx%KINECTS_PER_ROW;
	int gridY = camIdx/KINECTS_PER_ROW;
	float imgWidth = 1.0/(KINECTS_PER_ROW*2.0);
	float imgHeight = 1.0/rows;

	float coordXColor = (float)gridX/KINECTS_PER_ROW;
	float coordYColor = (rows-(float)gridY-1)/rows;

	glBegin(GL_QUADS);
		glTexCoord2f(0,1); glVertex2f(coordXColor, coordYColor);
		glTexCoord2f(1,1); glVertex2f(coordXColor+imgWidth, coordYColor);
		glTexCoord2f(1,0); glVertex2f(coordXColor+imgWidth, coordYColor+imgHeight);
		glTexCoord2f(0,0); glVertex2f(coordXColor, coordYColor+imgHeight);	
	glEnd();

	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);
}

void renderDepthTex(int camIdx){
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, nuiSensor[camIdx]->depthTex);

	int rows = ceil((float)numDev/KINECTS_PER_ROW);
	int gridX = camIdx%KINECTS_PER_ROW;
	int gridY = camIdx/KINECTS_PER_ROW;
	float imgWidth = 1.0/(KINECTS_PER_ROW*2.0);
	float imgHeight = 1.0/rows;

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

	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);
}

void glutDisplay (void)
{
	glClear(GL_COLOR_BUFFER_BIT);

	for(int i=0; i<numDev; ++i){
		if(nuiSensor[i]->waitNextFrame()){
			renderColorTex(i);
			renderDepthTex(i);
		}
	}

	glutSwapBuffers();
}

int main(int argc, char* argv[]){
	HRESULT hr = NuiGetSensorCount(&numDev);

	if ( FAILED(hr) )
	{
		return -1;
	}

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);

	int initWidth = (640*KINECTS_PER_ROW*2);
	int initHeight = (480*ceil((float)numDev/KINECTS_PER_ROW));	

	glutInitWindowSize(initWidth, initHeight);
	glutCreateWindow ("SDK Kinect Test");

	glewInit();

	for(int i=0; i<numDev; ++i){
		nuiSensor[i] = new NuiSensor(i);
		nuiSensor[i]->openStreams();
		nuiSensor[i]->initTexturePBO();
	}

	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);
	glutReshapeFunc (&ReSizeGLScene);

	// Per frame code is in glutDisplay
	glutMainLoop();

	return 0;
}