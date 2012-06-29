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
#include "beingthere_config.h"

using namespace std;

//Texture & Buffer
GLuint textureColorID;
GLuint textureDepthID;

namespace SHADER_ATTR{
	enum type{
		POSITION = 0,
		NORMAL = 1,
		COLOR = 3,
		TEXCOORD = 4
	};
} //end namespace attr

GLuint vertexArrayID;
GLuint positionBufferID;
GLuint texCoordBufferID;
GLuint elementBufferID;

GLSLProgram* textureShader = 0;

glm::mat4 Projection;
glm::mat4 View;
glm::mat4 Model;
glm::mat4 MVP;

namespace{
	std::string const SHADER_ROOT_PATH = BEINGTHERE_EXAMPLE_ROOT_DIR + "opengl_pointcloud_renderer/";
	//std::string const SHADER_ROOT_PATH = "D:/Trunk/bibleProj/beingthere/examples/opengl_pointcloud_renderer/";
	std::string const VERT_SHADER_SOURCE(SHADER_ROOT_PATH + "shader/point_cloud.vert");
	std::string const FRAG_SHADER_SOURCE(SHADER_ROOT_PATH + "shader/point_cloud.frag");
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

void updateMVPOrtho(glm::mat4& MVP){
	Projection = glm::ortho(0.f, 640.f, 0.f, 480.f, -1.f, 1.f);
	View = glm::mat4(1.f);
	Model = glm::mat4(1.0f);
	
	MVP = Projection * View * Model;
}

void updateMVPPersp(glm::mat4& MVP){
	Projection = glm::perspective(45.0f, 640.0f / 480.0f, 1.f, 5000.f);
	View = glm::translate(glm::mat4(1.f), glm::vec3(0.f, 0.f, -500.f));
	Model = glm::mat4(1.0f);
	
	MVP = Projection * View * Model;
}

void glutDisplay (void)
{
	glEnable(GL_TEXTURE_2D);

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, textureColorID);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureDepthID);
	
	float fx, fy, cx, cy;
	fx = fy = 532.f;
	cx = cy = 323.f;

	textureShader->setUniform("fx", fx);
	textureShader->setUniform("fy", fy);
	textureShader->setUniform("cx", cx);
	textureShader->setUniform("cy", cy);

	glBegin(GL_POINTS);
	for(int i = 0; i < 640; i++) {
		for(int j = 0; j < 480; j++) {
			glVertex3f(i, j, 0);
		}
	}
	glEnd();
	
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, 0);	
	glActiveTexture(GL_TEXTURE1);
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

	textureShader->setUniform("texColor", 1);
	textureShader->setUniform("texDepth", 0);

	updateMVPPersp(MVP);
	//updateMVPOrtho(MVP);
	textureShader->setUniform("MVP", MVP);

	//textureShader->printActiveAttribs();
	//textureShader->printActiveUniforms();
}

bool createTexture(GLuint& textureID, bool isDepth){
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	if(!isDepth){
		unsigned char * dummyTex = (unsigned char *)malloc(640*480*3);
		memset(dummyTex, 0, 640*480*3);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, dummyTex);
	}
	else{
		unsigned char * dummyTex = (unsigned char *)malloc(640*480*2);
		memset(dummyTex, 0, 640*480*2);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16, 640, 480, 0, GL_GREEN, GL_UNSIGNED_SHORT, dummyTex);
	}

	glBindTexture(GL_TEXTURE_2D, 0);

	return true;
}

bool uploadToTexture(GLuint& textureID, string imgPath, bool isDepth){
	cv::Mat img = cv::imread(imgPath.c_str(), CV_LOAD_IMAGE_UNCHANGED);
	
	glBindTexture(GL_TEXTURE_2D, textureID);

	//glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	/*glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 640, 480, 0,
		GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*) img.data);*/

	if(!isDepth)
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_RGB, GL_UNSIGNED_BYTE, img.data);
	else 
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_GREEN, GL_UNSIGNED_SHORT, img.data);

	glBindTexture(GL_TEXTURE_2D, 0);

	return true;
}

int main(int argc, char* argv[]){
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
	glutInitWindowSize(640, 480);
	glutCreateWindow ("OpenGL Test");

	glewInit();
	
	//Load Texture
	std::string imgColorPath = BEINGTHERE_ROOT_DIR + "resources/multi-cam/data/snapshot_rgb_3.png";
	std::string imgDepthPath = BEINGTHERE_ROOT_DIR + "resources/multi-cam/data/snapshot_depth_3.png";
	//std::string imgColorPath = "D:/Trunk/bibleProj/beingthere-data/multicams/snapshot_rgb_3.png";
	//std::string imgDepthPath = "D:/Trunk/bibleProj/beingthere-data/multicams/snapshot_depth_3.png"; 

	createTexture(textureColorID, false);
	uploadToTexture(textureColorID, imgColorPath, false);

	createTexture(textureDepthID, true);
	uploadToTexture(textureDepthID, imgDepthPath, true);

	createShader();

	glutReshapeFunc (&ReSizeGLScene);
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glutMainLoop();
	return 0;
}