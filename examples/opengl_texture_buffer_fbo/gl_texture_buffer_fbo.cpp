/*BRIEF:
Upload/Download Data To Texture, Buffer
Draw To Single FBO
*/

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

//Texture & Buffer
GLuint textureID;
GLuint bufferID;

//Frame Buffer Object
GLuint fboID;
GLuint fbo_colorID, fbo_depthID;
static const GLenum fboBuffs[] = { GL_COLOR_ATTACHMENT0};
static const GLenum windowBuff[] = { GL_BACK_LEFT};

//Constant
int wWidth;
int wHeight;

void glutIdle (void)
{
	glutPostRedisplay();
}

void glutKeyboard (unsigned char key, int x, int y)
{
	if(key==27) exit(0);
}

void ReSizeGLScene (int Width, int Height)
{
  glViewport (0, 0, Width, Height);
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();
  glOrtho (0, 1, 0, 1, -1, 1);
  glMatrixMode (GL_MODELVIEW);

  wWidth = Width;
  wHeight = Height;

  //Resize FBO Renderer Buffer
  glBindRenderbuffer(GL_RENDERBUFFER, fbo_colorID);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB, Width, Height);
}

void glutDisplay (void)
{
	bool renderToFBO = false;
	if(renderToFBO){
		//Draw To FBO
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fboID);
		glDrawBuffers(1, fboBuffs);
		glClear(GL_COLOR_BUFFER_BIT);
		//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, textureID);

	glBegin(GL_QUADS);
		glTexCoord2f(0,1); glVertex2f(0, 0);
		glTexCoord2f(1,1); glVertex2f(1, 0);
		glTexCoord2f(1,0); glVertex2f(1, 1);
		glTexCoord2f(0,0); glVertex2f(0, 1);	
	glEnd();

	//Draw To Scene
	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);

	if(renderToFBO){
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
		glDrawBuffers(1, windowBuff);
		//glViewport(0, 0, 640, 480);

		//Read Data From FBO
		glBindFramebuffer(GL_READ_FRAMEBUFFER, fboID);
		glReadBuffer(GL_COLOR_ATTACHMENT0);
		glBlitFramebuffer(0, 0, wWidth, wHeight,
			0, 0, wWidth, wHeight,
			GL_COLOR_BUFFER_BIT, GL_LINEAR );

		//Read From Color Attachment 0
		void* img_ptr = malloc(wWidth * wHeight * 3);
		glReadPixels(0, 0, 640, 480, GL_BGR, GL_UNSIGNED_BYTE, img_ptr);
		cv::Mat img(wHeight, wWidth, CV_8UC3);
		memcpy(img.data, img_ptr, wWidth*wHeight*3);
		cv::imshow("Image", img);
		cv::waitKey(3);
	}

	glutSwapBuffers();
}

bool createTexture(GLuint& textureID){
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	unsigned char * dummyTex = (unsigned char *)malloc(640*480*3);
	memset(dummyTex, 0, 640*480*3);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, dummyTex);

	glBindTexture(GL_TEXTURE_2D, 0);

	return true;
}

bool createBuffer(GLuint& bufferID){
	unsigned char * dummyTex = (unsigned char *)malloc(640*480*3);
	memset(dummyTex, 0, 640*480*3);

	glGenBuffers(1, &bufferID);
	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, bufferID);
	glBufferData(GL_PIXEL_UNPACK_BUFFER, 640*480*3, dummyTex, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);

	return true;
}

bool uploadToTexture(GLuint& textureID){
	cv::Mat img = cv::imread("C:/Logs/PCL/data/snapshot_rgb_3.png");
	glBindTexture(GL_TEXTURE_2D, textureID);

	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	/*glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 640, 480, 0,
		GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*) img.data);*/

	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_RGB, GL_UNSIGNED_BYTE, img.data);

	glBindTexture(GL_TEXTURE_2D, 0);

	return true;
}

bool readTexture(GLuint& textureID){
	unsigned char* img_data = (unsigned char*) malloc(640*480*3);
	glGetTextureImageEXT(textureID, GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);

	cv::Mat img(480,640, CV_8UC3);
	memcpy(img.data, img_data, 640*480*3);

	cv::imshow("Image", img);
	cvWaitKey(0);

	return true;
}

bool uploadToBuffer(GLuint& bufferID){
	cv::Mat img = cv::imread("C:/Logs/PCL/data/snapshot_rgb_3.png");

	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, bufferID);
	void * ptr = glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
	if(ptr != NULL) {
		memcpy(ptr, img.data, 640*480*3);
		glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
		glBindTexture(GL_TEXTURE_2D, textureID);
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_RGB, GL_UNSIGNED_BYTE, 0);
		//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);
		glBindTexture(GL_TEXTURE_2D, 0);
	}

	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);

	return true;
}

bool readBuffer(GLuint& bufferID){
	cv::Mat img(480, 640, CV_8UC3);

	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, bufferID);
	void* ptr = glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_READ_ONLY);
	if(ptr != NULL){
		memcpy(img.data, ptr, 640*480*3);
		cv::imshow("Image", img);
		cv::waitKey(0);
	}

	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);

	return true;
}

bool createFBO(GLuint& fboID){
	//Create Color RendererBuffer
	glGenRenderbuffers(1, &fbo_colorID);
	glBindRenderbuffer(GL_RENDERBUFFER, fbo_colorID);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB, 640, 480);

	//Create FBO 
	glGenFramebuffers(1, &fboID);
	glBindFramebuffer(GL_FRAMEBUFFER, fboID);

	//Attach Buffers/Textures To FBO
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, fbo_colorID);
	if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		return false;

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	return true;
}


int main(int argc, char* argv[]){
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
	glutInitWindowSize(640, 480);
	glutCreateWindow ("OpenGL Test");
	//glutHideWindow();

	glewInit();
	
	createTexture(textureID);
	//uploadToTexture(textureID);
	//readTexture(textureID);

	createBuffer(bufferID);
	uploadToBuffer(bufferID);
	//readBuffer(bufferID);	

	createFBO(fboID);

	glutReshapeFunc (&ReSizeGLScene);
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glutMainLoop();
	return 0;
}