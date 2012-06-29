//Set ModelViewProjection Matrix From Shader
//Update MVP Matrix Through Mouse Motion

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <time.h>

#include <GL/glew.h>
#include <GL/GL.h>
#include <GL/freeglut.h>

#include <cv.h>
#include <highgui.h>

#include "glm/glm.hpp"
#include <glm/gtc/matrix_transform.hpp>

#include "glslprogram.h"

using namespace std;

//Texture & Buffer
GLuint textureID;
GLuint bufferID;

GLSLProgram* textureShader = 0;

//Button, Keyboard Function
int ox, oy;
int buttonState = 0;
bool keyDown[256];

glm::vec3 cameraPos(0, -1, -4);
glm::vec3 cameraRot(0, 0, 0);
glm::vec3 cameraPosLag(cameraPos);
glm::vec3 cameraRotLag(cameraRot);

const float inertia = 0.1f;
const float translateSpeed = 0.002f;
const float cursorSpeed = 0.01f;
const float rotateSpeed = 0.2f;
const float walkSpeed = 0.05f;

glm::mat4 Projection;
glm::mat4 View;
glm::mat4 Model;
glm::mat4 MVP;

namespace{
	std::string const SHADER_ROOT_PATH = "C:/Logs/PCL/beingthere/examples/opengl_shader_mvp/";
	//std::string const SHADER_ROOT_PATH = "D:/Trunk/bibleProj/beingthere/examples/opengl_shader_mvp/";
	std::string const VERT_SHADER_SOURCE(SHADER_ROOT_PATH + "220/opengl_mvp.vert");
	std::string const FRAG_SHADER_SOURCE(SHADER_ROOT_PATH + "220/opengl_mvp.frag");
}

void printMat(const glm::mat4& mat){
	for(int i=0; i<4; ++i)
		cout << mat[i].x << " " << mat[i].y << " " << 
		mat[i].z << " " << mat[i].z << endl;
}

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
}

//Ortho Projection 
void updateMVPOrtho(glm::mat4& MVP){
	Projection = glm::ortho(0.f, 1.f, 0.f, 1.f, -1.f, 1.f);
	View = glm::mat4(1.f);
	Model = glm::mat4(1.0f);
	
	MVP = Projection * View * Model;
}

//For Perspective Projection, zNear must >=0 , zFar can be any number > znear
void updateMVPPersp(glm::mat4& MVP){
	Projection = glm::perspective(35.0f, 640.0f / 480.0f, 0.01f, 5.f);
	View = glm::translate(glm::mat4(1.f), glm::vec3(0.f, 0.f, -2.f));
	Model = glm::translate(glm::mat4(1.0f), glm::vec3(-0.5f, -0.5f, 0.f));
	
	MVP = Projection * View * Model;
}

void glutDisplay (void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, textureID);

	textureShader->setUniform("MVP", MVP);

	glBegin(GL_QUADS);
		glTexCoord2f(0,1); glVertex2f(0, 0);
		glTexCoord2f(1,1); glVertex2f(1, 0);
		glTexCoord2f(1,0); glVertex2f(1, 1);
		glTexCoord2f(0,0); glVertex2f(0, 1);	
	glEnd();
	
	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);

	glutSwapBuffers();
}

void createShader(){
	textureShader = new GLSLProgram();

	textureShader->compileShaderFromFile(VERT_SHADER_SOURCE.c_str(), GLSLShader::VERTEX);
	cout << textureShader->log() << endl;
	textureShader->compileShaderFromFile(FRAG_SHADER_SOURCE.c_str(), GLSLShader::FRAGMENT);
	cout << textureShader->log() << endl;

	textureShader->link(); 
	cout << textureShader->log() << endl;

	textureShader->validate();	
	cout << textureShader->log() << endl;

	textureShader->use();

	textureShader->setUniform("textureMap", 0);

	//textureShader->printActiveAttribs();
	//textureShader->printActiveUniforms();
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

bool uploadToTexture(GLuint& textureID){
	cv::Mat img = cv::imread("C:/Logs/PCL/data/snapshot_rgb_3.png");
	//cv::Mat img = cv::imread("D:/Trunk/bibleProj/beingthere-data/multicams/snapshot_rgb_3.png");
	glBindTexture(GL_TEXTURE_2D, textureID);

	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	/*glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 640, 480, 0,
		GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*) img.data);*/

	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_RGB, GL_UNSIGNED_BYTE, img.data);

	glBindTexture(GL_TEXTURE_2D, 0);

	return true;
}

//Mouse, Keyboard Function
void mouse(int button, int state, int x, int y)
{
    int mods;

    if (state == GLUT_DOWN)
        buttonState |= 1<<button;
    else if (state == GLUT_UP)
        buttonState = 0;

    mods = glutGetModifiers();
    if (mods & GLUT_ACTIVE_SHIFT) {
        buttonState = 2;
    } else if (mods & GLUT_ACTIVE_CTRL) {
        buttonState = 3;
    }

    ox = x; oy = y;

	//cout << buttonState << endl;
	//cout << button << endl;
    glutPostRedisplay();
}

void motion(int x, int y)
{
    float dx, dy;
    dx = (float)(x - ox);
    dy = (float)(y - oy);

	if (buttonState == 1) {
		cameraRot[0] += dy * rotateSpeed;
		cameraRot[1] += dx * rotateSpeed;

		cameraRotLag += (cameraRot - cameraRotLag) * inertia;
		View = glm::translate(glm::mat4(1.f), glm::vec3(0.f, 0.f, -2.f));
		View = glm::rotate(View, cameraRotLag[0], glm::vec3(1.0, 0.0, 0.0));
		View = glm::rotate(View, cameraRotLag[1], glm::vec3(0.0, 1.0, 0.0));
	}
	if (buttonState == 2) {
		// middle = translate
		View = glm::translate(View, glm::vec3(dx * translateSpeed, -dy*translateSpeed, 0.f));
	}
	if (buttonState == 3) {
		// left+middle = zoom
		View = glm::translate(View, glm::vec3(0.f, 0.f, dy*translateSpeed));
	} 

	MVP = Projection * View * Model;
	ox = x; oy = y;
	glutPostRedisplay();
}

int main(int argc, char* argv[]){
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
	glutInitWindowSize(640, 480);
	glutCreateWindow ("OpenGL Test");
	//glutHideWindow();

	glewInit();
	
	createTexture(textureID);
	uploadToTexture(textureID);
	//readTexture(textureID);

	createShader();
	//updateMVPOrtho(MVP);
	updateMVPPersp(MVP);

	glutReshapeFunc (&ReSizeGLScene);

	glutKeyboardFunc(glutKeyboard);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);

	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glutMainLoop();
	return 0;
}