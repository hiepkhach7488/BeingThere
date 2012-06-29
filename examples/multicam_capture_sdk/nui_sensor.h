#ifndef NUI_SENSOR
#define NUI_SENSOR

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

class NuiSensor{
public:
	NuiSensor();
	NuiSensor(int camIdx);
	~NuiSensor();

	void Nui_GotColorAlert( );
	void Nui_GotDepthAlert( );

	GLuint colorTex;
	GLuint depthTex;
	GLuint colorPBO;
	GLuint depthPBO;

	bool openStreams();

	void initTexturePBO();
	void uploadColorToTexture(void* colorData);
	void uploadDepthToTexture(void* depthData);

	bool waitNextFrame();

public:
	INuiSensor *m_pNuiSensor;

	HANDLE        m_pDepthStreamHandle;
	HANDLE        m_pVideoStreamHandle;

	HANDLE        m_hEvNuiProcessStop;
	HANDLE        m_hNextDepthFrameEvent;
	HANDLE        m_hNextColorFrameEvent;
};

#endif