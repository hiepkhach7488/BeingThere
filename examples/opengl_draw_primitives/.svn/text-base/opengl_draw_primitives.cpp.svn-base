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
GLuint textureColorID;
GLuint textureDepthID;

GLSLProgram* shaderProgram = 0;

glm::mat4 MVP;

//Triangle Mesh Template
GLshort *data = 0;
int triangleTemplateVertices;
GLuint triangleTemplateBuffer;
GLuint triangleTemplateVAO;

namespace{
	//std::string const SHADER_ROOT_PATH = "C:/Logs/PCL/beingthere/examples/opengl_pointcloud_renderer/";
	std::string const SHADER_ROOT_PATH = "D:/Trunk/bibleProj/beingthere/examples/opengl_draw_primitives/";
	std::string const VERT_SHADER_SOURCE(SHADER_ROOT_PATH + "shader/draw_primitives.vert");
	std::string const GEOM_SHADER_SOURCE(SHADER_ROOT_PATH + "shader/draw_primitives.geom");
	std::string const FRAG_SHADER_SOURCE(SHADER_ROOT_PATH + "shader/draw_primitives.frag");
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
  glOrtho (0, 4, 0, 4, -1, 1);
  glMatrixMode (GL_MODELVIEW);
}

void updateMVPOrtho(glm::mat4& MVP, float w, float h){
	glm::mat4 Projection = glm::ortho(0.f, w, 0.f, h, -1.f, 1.f);
	glm::mat4 View = glm::mat4(1.f);
	glm::mat4 Model = glm::mat4(1.0f);

	MVP = Projection * View * Model;
}

void drawPoint(){
	glPointSize(5.0);
	glBegin(GL_POINTS);
		glColor3f(1.0, 0.0, 0.0); glVertex2f(1.0, 2.0); 
		glColor3f(0.0, 1.0, 0.0); glVertex2f(1.0, 3.0); 
		glColor3f(0.0, 0.0, 1.0); glVertex2f(2.0, 3.0); 
	glEnd();
}

void drawTriangle(){
	glBegin(GL_TRIANGLES);
		glColor3f(1.0, 0.0, 0.0); glVertex2f(1.0, 2.0); 
		glColor3f(0.0, 1.0, 0.0); glVertex2f(1.0, 3.0); 
		glColor3f(0.0, 0.0, 1.0); glVertex2f(2.0, 3.0); 
	glEnd();
}

void drawTriangleStrip(){
	glBegin(GL_TRIANGLE_STRIP);
		glColor3f(1.0, 0.0, 0.0); glVertex2f(1.0, 2.0); 
		glColor3f(0.0, 1.0, 0.0); glVertex2f(1.0, 3.0); 
		glColor3f(0.0, 0.0, 1.0); glVertex2f(2.0, 3.0); 
		glColor3f(0.0, 0.0, 0.0); glVertex2f(1.0, 4.0); 
	glEnd();
}

void drawLine(){
	glBegin(GL_LINES);
		glColor3f(1.0, 0.0, 0.0); glVertex2f(1.0, 2.0); 
		glColor3f(0.0, 1.0, 0.0); glVertex2f(1.0, 3.0); 
	glEnd();
}

void drawLineStrip(){
	glBegin(GL_LINE_STRIP);
		glColor3f(1.0, 0.0, 0.0); glVertex2f(1.0, 2.0); 
		glColor3f(0.0, 1.0, 0.0); glVertex2f(1.0, 3.0); 
		glColor3f(0.0, 0.0, 1.0); glVertex2f(2.0, 3.0); 
		glColor3f(0.0, 0.0, 0.0); glVertex2f(2.0, 2.0); 
	glEnd();
}

void drawQuad(){
	glBegin(GL_QUADS);
		glColor3f(1.0, 0.0, 0.0); glVertex2f(1.0, 1.0); 
		glColor3f(1.0, 0.0, 0.0); glVertex2f(1.0, 2.0); 
		glColor3f(1.0, 0.0, 0.0); glVertex2f(2.0, 2.0); 
		glColor3f(1.0, 0.0, 0.0); glVertex2f(2.0, 1.0); 
	glEnd();
}

void drawTriangleTemplate(){
	glColor3f(1.0, 0.0, 0.0);
	
	glEnableClientState (GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_SHORT, 0, data);

	//glDrawArrays(GL_TRIANGLES, 0, triangleTemplateVertices);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, triangleTemplateVertices);
	//glDrawArrays(GL_LINE_STRIP, 0, triangleTemplateVertices);
	//glDrawArrays(GL_LINES, 0, triangleTemplateVertices);

	glDisableClientState(GL_VERTEX_ARRAY); 
}

void drawFromBuffer(){
	int size = triangleTemplateVertices*3*sizeof(GLshort);

	//Create Buffer
	glGenBuffers(1, &triangleTemplateBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, triangleTemplateBuffer);
	glBufferData(GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	//Create VAO
	//glGenVertexArrays(1, &triangleTemplateVAO);
	//glBindVertexArray(triangleTemplateVAO);

	glBindBuffer(GL_ARRAY_BUFFER, triangleTemplateBuffer);
	glVertexAttribPointer(0, 3, GL_SHORT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	//glBindBuffer(GL_ARRAY_BUFFER, 0);
	//glBindVertexArray(0);

	//Setup Shader
	shaderProgram->use();
	updateMVPOrtho(MVP, 4.0, 4.0);
	shaderProgram->setUniform("MVP", MVP);

	//Start Drawing
	//glBindVertexArray(triangleTemplateVAO);
	glBindBuffer(GL_ARRAY_BUFFER, triangleTemplateBuffer);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, triangleTemplateVertices);
	//glDrawArrays(GL_LINE_STRIP, 0, triangleTemplateVertices);

}

void glutDisplay (void)
{
	glClear(GL_COLOR_BUFFER_BIT);
	glClearColor(0.3, 0.3, 0.0, 0.0);

	//drawPoint();
	//drawLine();	
	//drawLineStrip();
	//drawTriangle();
	//drawTriangleStrip();
	//drawQuad();
	//drawTriangleTemplate();
	drawFromBuffer();

	glutSwapBuffers();
}

void createShader(){
	shaderProgram = new GLSLProgram();

	shaderProgram->compileShaderFromFile(VERT_SHADER_SOURCE.c_str(), GLSLShader::VERTEX);   cout << shaderProgram->log() << endl;
	shaderProgram->compileShaderFromFile(GEOM_SHADER_SOURCE.c_str(), GLSLShader::GEOMETRY); cout << shaderProgram->log() << endl;
	shaderProgram->compileShaderFromFile(FRAG_SHADER_SOURCE.c_str(), GLSLShader::FRAGMENT); cout << shaderProgram->log() << endl;
	shaderProgram->link(); cout << shaderProgram->log() << endl;
	shaderProgram->validate(); cout << shaderProgram->log() << endl;

	/*shaderProgram->setUniform("texColor", 1);
	shaderProgram->setUniform("texDepth", 0);*/

	//float fx, fy, cx, cy;
	//fx = fy = 532.f;
	//cx = cy = 323.f;

	//shaderProgram->setUniform("fx", fx);
	//shaderProgram->setUniform("fy", fy);
	//shaderProgram->setUniform("cx", cx);
	//shaderProgram->setUniform("cy", cy);

	//shaderProgram->printActiveAttribs();
	//shaderProgram->printActiveUniforms();
}

void createVertexBuffer(){
	int nrow = 4; //480
	int ncol = 4; //640

	triangleTemplateVertices = (nrow-1)*(2*ncol)+(int)(nrow/2);
	int size = triangleTemplateVertices*3*sizeof(GLshort);
	data = (GLshort*)malloc(size);

	int pos = 0;
	for(int j = 0; j < nrow-1; j++) {
		if(j%2==0) {
			for(int k = 0; k < ncol; k++) {
				data[pos++] = k; data[pos++] = j;   data[pos++] = 0; //vertex 1
				data[pos++] = k; data[pos++] = j+1; data[pos++] = 0; //vertex 2
			}
			data[pos++] = ncol-1; data[pos++] = j+1; data[pos++] = 0; //dummy vertex to align rows
		} else {
			for(int k = ncol-1; k >= 0; k--) {
				data[pos++] = k; data[pos++] = j+1; data[pos++] = 0; //vertex 1
				data[pos++] = k; data[pos++] = j;   data[pos++] = 0; //vertex 2
			}
		}			
	}
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
	/*std::string imgColorPath = "C:/Logs/PCL/data/snapshot_rgb_3.png";
	std::string imgDepthPath = "C:/Logs/PCL/data/snapshot_depth_3.png";*/
	std::string imgColorPath = "D:/Trunk/bibleProj/beingthere-data/multicams/snapshot_rgb_3.png";
	std::string imgDepthPath = "D:/Trunk/bibleProj/beingthere-data/multicams/snapshot_depth_3.png";

	createTexture(textureColorID, false);
	uploadToTexture(textureColorID, imgColorPath, false);

	createTexture(textureDepthID, true);
	uploadToTexture(textureDepthID, imgDepthPath, true);

	createVertexBuffer();
	createShader();

	glutReshapeFunc (&ReSizeGLScene);
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glutMainLoop();
	return 0;
}