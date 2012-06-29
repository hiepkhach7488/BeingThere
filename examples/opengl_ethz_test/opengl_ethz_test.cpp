#include <stdio.h>   
#include <stdlib.h> 
#include <iostream>
#include <string>

#include <GL/glew.h>
#include <GL/GL.h>
#include <GL/glut.h>
#include <GL/v3d_gpuundistort.h>

#include <Cg/cg.h>   
#include <Cg/cgGL.h>

#include "glslprogram.h"
#include "scene.h"
#include "gl_tools.h"
#include "beingthere_config.h"

#include <cv.h>
#include <highgui.h>

using std::string;
using namespace std;
using namespace V3D_GPU;

ParametricUndistortionFilter _parametricUndistortionFilter;

GLuint _colorTexture, _copyFbo, _depthTexture, _depth8Texture;
Scene* kinectScene;
GLuint _copyShader;

GLSLProgram* depthOverColorProg;
void saveBackground(string, string);

bool isRenderingTexture = false;

void setupRC(){
	//string const CALIB_FILE_PATH = "C:/Logs/PCL/beingthere/examples/opengl_ethz_test/kinect.calib";
	string const CALIB_FILE_PATH = BEINGTHERE_EXAMPLE_ROOT_DIR + "opengl_ethz_test/kinect.calib";

	kinectScene = new Scene();
	kinectScene->setKinectColorCam(CALIB_FILE_PATH);
	kinectScene->setKinectDepthCam(CALIB_FILE_PATH);
	
	Resolution _colorRes = kinectScene->getColorResolution();

	glGenTextures(1, &_colorTexture);
	glBindTexture(GL_TEXTURE_2D, _colorTexture);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, _colorRes.x, _colorRes.y, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	glBindTexture(GL_TEXTURE_2D, 0);

	glGenTextures(1, &_depthTexture);
	glBindTexture(GL_TEXTURE_2D, _depthTexture);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16, _colorRes.x, _colorRes.y, 0, GL_RED, GL_UNSIGNED_SHORT, NULL);
	glBindTexture(GL_TEXTURE_2D, 0);

	glGenTextures(1, &_depth8Texture);
	glBindTexture(GL_TEXTURE_2D, _depth8Texture);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, _colorRes.x, _colorRes.y, 0, GL_RED, GL_UNSIGNED_BYTE, NULL);
	glBindTexture(GL_TEXTURE_2D, 0);

	//Create Shader Program
	string const SHADER_ROOT_PATH = BEINGTHERE_EXAMPLE_ROOT_DIR + "opengl_ethz_test/shader/";
		//"C:/Logs/PCL/beingthere/examples/opengl_ethz_test/shader/";

	_copyShader = GLTools::createShaderProgram((SHADER_ROOT_PATH + "vertex.vs").c_str(), (SHADER_ROOT_PATH+ "copy.fs").c_str());

	//Initialize CG Program
	Cg_ProgramBase::initializeCg();

	//Set Undistortion Filter
	_parametricUndistortionFilter.allocate(_colorRes.x, _colorRes.y, SHADER_ROOT_PATH + "undistort_parametric.cg");

	//Generate Few FBO
	glGenFramebuffers(1, &_copyFbo);

	//Depth Over Color Program
	depthOverColorProg = new GLSLProgram();
	//depthOverColorProg->compileShaderFromFile((SHADER_ROOT_PATH+"vertex.vs").c_str(), GLSLShader::VERTEX); cout << depthOverColorProg->log() << endl;
	depthOverColorProg->compileShaderFromFile((SHADER_ROOT_PATH+"depth_over_color.frag").c_str(), GLSLShader::FRAGMENT); cout << depthOverColorProg->log() << endl;
	depthOverColorProg->link(); cout << depthOverColorProg->log() << endl;
	depthOverColorProg->validate(); cout << depthOverColorProg->log() << endl;	
}

void glutIdle (void)
{
	glutPostRedisplay();
}

void glutKeyboard (unsigned char key, int x, int y)
{
	if(key==27) {
		exit(0);
	}

	if(key == 'b'){
		cout << "Saving Background" << endl;
		saveBackground("bg_depth.dat", "bg_color.dat");
	}

	if(key == 'c'){
		cout << "Saving Current Frame" << endl;
		saveBackground("cur_depth.dat", "cur_color.dat");
	}

	if(key == 'w'){
		cout << "Switching" << endl;
		isRenderingTexture = !isRenderingTexture;
	}
}

void ReSizeGLScene (int Width, int Height)
{
	glViewport (0, 0, Width, Height);
	glMatrixMode (GL_PROJECTION);										
	glLoadIdentity ();												
	glOrtho (0, 1, 0, 1, -1, 1);
	glMatrixMode (GL_MODELVIEW);										
	glLoadIdentity ();
}

void copyTexture(GLuint inputTexture, GLuint outputTexture, int w, int h){
	//copy inputTexture to outputTexture on the GPU
	glBindFramebuffer(GL_FRAMEBUFFER, _copyFbo);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, outputTexture, 0);

	glUseProgram(_copyShader);

	glViewport(0, 0, w, h);

	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);	
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, inputTexture);
	glUniform1i(glGetUniformLocation(_copyShader, "inputTexture"), 0);
	GLTools::drawTriangle();

	glUseProgram(0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void captureNext(){
	bool _undistort = false;

	kinectScene->getColorCam()->acquire();
	kinectScene->getDepthCam()->acquire();

	Resolution _colorRes = kinectScene->getColorResolution();

	unsigned char* colorImage = new unsigned char[_colorRes.x*_colorRes.y*3];

	Camera* colorCam = kinectScene->getColorCam();
	memcpy(colorImage, colorCam->getCurrentFrame(), _colorRes.x*_colorRes.y*3*sizeof(unsigned char));

	unsigned short* depthImage = new unsigned short[_colorRes.x*_colorRes.y * 2];
	KinectDepthCamera* depthCam = (KinectDepthCamera*)kinectScene->getDepthCam();
	memcpy(depthImage, depthCam->getCurrentUShortFrame(), _colorRes.x*_colorRes.y*2);

	if (_undistort){
		//undistort and set texture
		CalibData* c = colorCam->getCalibration();
		double radial[4] = {c->kc[0], c->kc[1], 0, 0};
		double center[2] = {c->kc[2], c->kc[3]};
		_parametricUndistortionFilter.setDistortionParameters(c->fx, c->fy, radial, center);
		_parametricUndistortionFilter.undistortColorImage(colorImage);
		GLuint undistortedTexId = _parametricUndistortionFilter.getResult().getTexture().textureID();
		copyTexture(undistortedTexId, _colorTexture, _colorRes.x, _colorRes.y);
	} else {
		glBindTexture(GL_TEXTURE_2D, _colorTexture);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, _colorRes.x, _colorRes.y, 0, GL_RGB, GL_UNSIGNED_BYTE, colorImage);
	}

	//Upload To 16Bit R-Channel Depth Texture
	glBindTexture(GL_TEXTURE_2D, _depthTexture);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_RED, GL_UNSIGNED_SHORT, depthImage);	

	//Upload To 8Bit Texture
	unsigned char* depth8Image = new unsigned char[_colorRes.x*_colorRes.y];
	memcpy(depth8Image, depthCam->getCurrentUCharFrame(), _colorRes.x*_colorRes.y);
	glBindTexture(GL_TEXTURE_2D, _depth8Texture);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_RED, GL_UNSIGNED_BYTE, depth8Image);	

	glBindTexture(GL_TEXTURE_2D, 0);

	free(colorImage);
}

void renderDepthOverColor(){
	glActiveTexture( GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, _colorTexture);

	glActiveTexture( GL_TEXTURE0);
	//glBindTexture(GL_TEXTURE_2D, _depthTexture);
	glBindTexture(GL_TEXTURE_2D, _depth8Texture);

	glDisable( GL_DEPTH_TEST);

	depthOverColorProg->use();
	depthOverColorProg->setUniform("depthTransparency", 0.5f);
	depthOverColorProg->setUniform("depthContrast", 20.0f);
	//depthOverColorProg->setUniform("depthContrast", 0.f);
	depthOverColorProg->setUniform("texColor", 1);
	depthOverColorProg->setUniform("texDepth", 0);

	//glBegin( GL_QUADS);
	//	glTexCoord2f(0, 0); glVertex2f(0, 0);
	//	glTexCoord2f(1, 0); glVertex2f(1, 0);
	//	glTexCoord2f(1, 1); glVertex2f(1, 1);
	//	glTexCoord2f(0, 1); glVertex2f(0, 1);
	//glEnd();
	glBegin(GL_QUADS);
		glTexCoord2f(0,1); glVertex2f(0, 0);
		glTexCoord2f(1,1); glVertex2f(1, 0);
		glTexCoord2f(1,0); glVertex2f(1, 1);
		glTexCoord2f(0,0); glVertex2f(0, 1);	
	glEnd();

	glUseProgram(0);

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, 0);

	glEnable(GL_DEPTH_TEST);
}

void renderColorTexture(){
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, _colorTexture);
	//glBindTexture(GL_TEXTURE_2D, _depthTexture);
	//glBindTexture(GL_TEXTURE_2D, _depth8Texture);

	glBegin(GL_QUADS);
		glTexCoord2f(0,1); glVertex2f(0, 0);
		glTexCoord2f(1,1); glVertex2f(1, 0);
		glTexCoord2f(1,0); glVertex2f(1, 1);
		glTexCoord2f(0,0); glVertex2f(0, 1);	
	glEnd();

	//Draw To Scene
	glBindTexture(GL_TEXTURE_2D, 0);
}

void glutDisplay (void)
{
	captureNext();

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);				

	glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);

	if(isRenderingTexture)
		renderColorTexture();
	else
		renderDepthOverColor();

	glDisable(GL_TEXTURE_2D);

	glutSwapBuffers();
}

void readBackground(){
	Resolution _rawDepthRes = kinectScene->getDepthResolution();
	Resolution _colorRes = kinectScene->getColorResolution();

	ifstream file;
	file.open("./background/depth.dat", ios::in | ios::binary);
	float* depthBuffer = new float[_rawDepthRes.x*_rawDepthRes.y];
	memset(depthBuffer, 0, _rawDepthRes.x*_rawDepthRes.y*sizeof(float));

	if (!file){
		cout << "No background depth file - please capture background" << endl;
	} else {
		file.read((char*) depthBuffer, _rawDepthRes.x*_rawDepthRes.y*sizeof(float));	
	}
	file.close();
}

void saveBackground(string depthFileName, string colorFileName){
	//save current frame as background
	ofstream file;

	Resolution _rawDepthRes = kinectScene->getDepthResolution();
	Resolution _colorRes = kinectScene->getColorResolution();

	kinectScene->getDepthCam()->acquire();
	float* depthData = (float*) kinectScene->getDepthCam()->getCurrentFrame();
	//file.open("depth.dat", ios::out | ios::binary);
	file.open(depthFileName.c_str(), ios::out | ios::binary);
	file.write((const char*) depthData, _rawDepthRes.x*_rawDepthRes.y*sizeof(float));
	file.close();

	kinectScene->getColorCam()->acquire();
	unsigned char* colorData = (unsigned char*) kinectScene->getColorCam()->getCurrentFrame();
	ofstream colorFile;
	file.open(colorFileName.c_str(), ios::out | ios::binary);
	file.write((const char*) colorData, _colorRes.x*_colorRes.y*sizeof(unsigned char)*3);
	file.close();
	//updateBackground();
}

void saveBackgroundOpenCV(){
	Resolution _rawDepthRes = kinectScene->getDepthResolution();
	Resolution _colorRes = kinectScene->getColorResolution();

	kinectScene->getDepthCam()->acquire();
	float* depthData = (float*) kinectScene->getDepthCam()->getCurrentFrame();

	kinectScene->getColorCam()->acquire();
	unsigned char* colorData = (unsigned char*) kinectScene->getColorCam()->getCurrentFrame();
}

int main(int argc, char* argv[]){
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
	glutInitWindowSize(640, 480);
	glutCreateWindow ("OpenGL Test");

	glewInit();

	setupRC();

	glutReshapeFunc (&ReSizeGLScene);
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glutMainLoop();
	return 0;
}

