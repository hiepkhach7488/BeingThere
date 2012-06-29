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

//Interleaved Array
GLuint vertexArrayInterleavedID;
GLuint vertexBufferInterleavedID;
GLuint elementBufferInterleavedID;

GLSLProgram* textureShader = 0;

namespace{
	std::string const SHADER_ROOT_PATH = "D:/Trunk/bibleProj/beingthere/examples/opengl_vbo/";
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

	//glBindVertexArray(vertexArrayID);
	glBindVertexArray(vertexArrayInterleavedID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementBufferID);
	
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
	//glBegin(GL_QUADS);
	//	glTexCoord2f(0,1); glVertex2f(0, 0);
	//	glTexCoord2f(1,1); glVertex2f(1, 0);
	//	glTexCoord2f(1,0); glVertex2f(1, 1);
	//	glTexCoord2f(0,0); glVertex2f(0, 1);	
	//glEnd();
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

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
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, VertexElementSize, ElementData, GL_DYNAMIC_DRAW);
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

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void createVertexArrayInterleaved(){
	//Create Interleaved Buffer
	GLsizei const VertexCount(4);
	GLsizei const VertexBufferInterleavedSize = VertexCount * sizeof(glm::vec2) * 2;
			
	glm::vec2 const VertexData[] = {
		glm::vec2( 0.0f, 0.0f), glm::vec2( 0.0f, 1.0f),
		glm::vec2( 1.0f, 0.0f), glm::vec2( 1.0f, 1.0f),
		glm::vec2( 1.0f, 1.0f), glm::vec2( 1.0f, 0.0f), 
		glm::vec2( 0.0f, 1.0f), glm::vec2( 0.0f, 0.0f)
	};

	GLsizei const VertexElementSize = 6 * sizeof(glm::uint);
	glm::uint const ElementData[] = {
		0, 1, 3,
		1, 3, 2
	};

	glGenBuffers(1, &vertexBufferInterleavedID);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBufferInterleavedID);
    glBufferData(GL_ARRAY_BUFFER, VertexBufferInterleavedSize, VertexData, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, elementBufferID);
	void * ptr = glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
	if(ptr != NULL) {
		memcpy(ptr, ElementData , 6 * sizeof(glm::uint));
		glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
	}

	//Create Interleave Vertex Array Object
	glGenVertexArrays(1, &vertexArrayInterleavedID);
	glBindVertexArray(vertexArrayInterleavedID);

	glBindBuffer(GL_ARRAY_BUFFER, vertexBufferInterleavedID);
		glVertexAttribPointer(SHADER_ATTR::POSITION, 2, GL_FLOAT, GL_FALSE, 2*sizeof(glm::vec2), 0);
		glEnableVertexAttribArray(SHADER_ATTR::POSITION);
		glVertexAttribPointer(SHADER_ATTR::TEXCOORD, 2, GL_FLOAT, GL_FALSE, 2*sizeof(glm::vec2), (const GLvoid*)(sizeof(glm::vec2)));
		glEnableVertexAttribArray(SHADER_ATTR::TEXCOORD);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

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

	createArrayBuffer();
	//createVertexArray();
	createVertexArrayInterleaved();

	createShader();

	glutReshapeFunc (&ReSizeGLScene);
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glutMainLoop();
	return 0;
}