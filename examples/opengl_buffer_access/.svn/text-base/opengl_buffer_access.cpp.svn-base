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

namespace{
	std::string const SHADER_ROOT_PATH = "D:/Trunk/bibleProj/beingthere/examples/shader_test/";
	std::string const VERT_SHADER_SOURCE(SHADER_ROOT_PATH + "220/image-2d-220.vert");
	std::string const FRAG_SHADER_SOURCE(SHADER_ROOT_PATH + "220/image-2d-220.frag");
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
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();
  glOrtho (0, 1, 0, 1, -1, 1);
  glMatrixMode (GL_MODELVIEW);
}

void glutDisplay (void)
{
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, textureID);

	glm::mat4 Projection = glm::ortho(0.f, 1.f, 0.f, 1.f, -1.f, 1.f);
	glm::mat4 View = glm::mat4(1.f);
	glm::mat4 Model = glm::mat4(1.0f);
	glm::mat4 MVP = Projection * View * Model;

	textureShader->setUniform("MVP", MVP);

	glBindVertexArray(vertexArrayID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementBufferID);
	glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, NULL, 1);

	//glBegin(GL_QUADS);
	//	glTexCoord2f(0,1); glVertex2f(0, 0);
	//	glTexCoord2f(1,1); glVertex2f(1, 0);
	//	glTexCoord2f(1,0); glVertex2f(1, 1);
	//	glTexCoord2f(0,0); glVertex2f(0, 1);	
	//glEnd();
	
	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);

	glutSwapBuffers();
}

//Buffer For Drawing
void createArrayBuffer(){
	//Create Data For Drawing
	GLsizei const VertexCount(4);

	GLsizei const VertexPositionSize = VertexCount * sizeof(glm::vec2);
	glm::vec2 const PositionData[] = {
		glm::vec2( 0.0f, 0.0f),
		glm::vec2( 1.0f, 0.0f),
		glm::vec2( 1.0f, 1.0f), 
		glm::vec2( 0.0f, 1.0f), 
	};

	GLsizei const VertexTexCoordSize = VertexCount * sizeof(glm::vec2);
	glm::vec2 const TextCoordData[] = {
		glm::vec2( 0.0f, 1.0f),
		glm::vec2( 1.0f, 1.0f),
		glm::vec2( 1.0f, 0.0f), 
		glm::vec2( 0.0f, 0.0f), 
	};

	GLsizei const VertexElementSize = 6 * sizeof(glm::uint);
	glm::uint const ElementData[] = {
		0, 1, 3,
		1, 3, 2
	};

	//Upload Data To Buffer
	glGenBuffers(1, &positionBufferID);
    glBindBuffer(GL_ARRAY_BUFFER, positionBufferID);
    glBufferData(GL_ARRAY_BUFFER, VertexPositionSize, PositionData, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glGenBuffers(1, &texCoordBufferID);
    glBindBuffer(GL_ARRAY_BUFFER, texCoordBufferID);
    glBufferData(GL_ARRAY_BUFFER, VertexTexCoordSize, TextCoordData, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glGenBuffers(1, &elementBufferID);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementBufferID);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, VertexElementSize, ElementData, GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void createVertexArray(){
	glGenVertexArrays(1, &vertexArrayID);
	glBindVertexArray(vertexArrayID);

	glBindBuffer(GL_ARRAY_BUFFER, positionBufferID);
	glVertexAttribPointer(SHADER_ATTR::POSITION, 2, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(SHADER_ATTR::POSITION);

	glBindBuffer(GL_ARRAY_BUFFER, texCoordBufferID);
	glVertexAttribPointer(SHADER_ATTR::TEXCOORD, 2, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(SHADER_ATTR::TEXCOORD);

	//glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
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
	cv::Mat img = cv::imread("D:/Trunk/bibleProj/beingthere-data/multicams/snapshot_rgb_3.png");
	glBindTexture(GL_TEXTURE_2D, textureID);

	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	/*glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 640, 480, 0,
		GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*) img.data);*/

	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_RGB, GL_UNSIGNED_BYTE, img.data);

	glBindTexture(GL_TEXTURE_2D, 0);

	return true;
}

bool uploadToBuffer(GLuint& bufferID){
	cv::Mat img = cv::imread("D:/Trunk/bibleProj/beingthere-data/multicams/snapshot_rgb_3.png");

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

	//createBuffer(bufferID);
	//uploadToBuffer(bufferID);
	//readBuffer(bufferID);	

	createArrayBuffer();
	createVertexArray();

	createShader();

	glutReshapeFunc (&ReSizeGLScene);
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glutMainLoop();
	return 0;
}