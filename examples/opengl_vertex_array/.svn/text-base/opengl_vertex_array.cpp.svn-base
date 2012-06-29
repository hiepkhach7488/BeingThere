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
  glOrtho (0, 400, 0, 400, -1, 1);
  glMatrixMode (GL_MODELVIEW);
}

void glutDisplay (void)
{
	glClear(GL_COLOR_BUFFER_BIT);
	static GLint vertices[] = {
		25, 25,
		100, 325,
		175, 25,
		175, 325,
		250, 25,
		325, 325
	};

	static GLfloat colors[] = {
		1.0, 0.2, 0.2,
		0.2, 0.2, 1.0,
		0.8, 1.0, 0.2,
		0.75, 0.75, 0.75,
		0.35, 0.35, 0.35,
		0.5, 0.5, 0.5
	};

	glColor3f(0.0, 1.0, 0.0);
	//glEnableClientState (GL_COLOR_ARRAY);
	glEnableClientState (GL_VERTEX_ARRAY);
	//glColorPointer(3, GL_FLOAT, 0, colors);
	glVertexPointer(2, GL_INT, 0, vertices);
	
	glBegin(GL_TRIANGLES);
		glArrayElement (2);
		glArrayElement (3);
		glArrayElement (5);
	glEnd();

	glDisableClientState(GL_VERTEX_ARRAY);  
    //glDisableClientState(GL_COLOR_ARRAY);

	glutSwapBuffers();
}

void glutDisplay0 (void)
{
	glClear(GL_COLOR_BUFFER_BIT);
	static GLint vertices[] = {
		100, 100,
		200, 100,
		200, 200,
		300, 200,
		300, 100,
		200, 100,
	};

	static GLfloat colors[] = {
		1.0, 0.2, 0.2,
		0.2, 0.2, 1.0,
		0.8, 1.0, 0.2,
		0.75, 0.75, 0.75,
	};

	glColor3f(0.0, 1.0, 0.0);
	//glEnableClientState (GL_COLOR_ARRAY);
	glEnableClientState (GL_VERTEX_ARRAY);
	//glColorPointer(3, GL_FLOAT, 0, colors);
	glVertexPointer(2, GL_INT, 0, vertices);
	
	//glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glDrawArrays(GL_TRIANGLES, 0, 6);
	//glDrawArrays(GL_LINE_STRIP, 0, 6);
	//glDrawArrays(GL_LINES, 0, 6);

	glDisableClientState(GL_VERTEX_ARRAY);  
    //glDisableClientState(GL_COLOR_ARRAY);

	glutSwapBuffers();
}

void glutDisplay1(void){
	glClear(GL_COLOR_BUFFER_BIT);

	static GLfloat intertwined[] ={
		1.0, 0.2, 1.0, 100.0, 100.0, 0.0,
		1.0, 0.2, 0.2, 0.0, 200.0, 0.0,
		1.0, 1.0, 0.2, 100.0, 300.0, 0.0,
		0.2, 1.0, 0.2, 200.0, 300.0, 0.0,
		0.2, 1.0, 1.0, 300.0, 200.0, 0.0,
		0.2, 0.2, 1.0, 200.0, 100.0, 0.0
	};

	glEnableClientState (GL_COLOR_ARRAY);
	glEnableClientState (GL_VERTEX_ARRAY);
	glColorPointer (3, GL_FLOAT, 6 * sizeof(GLfloat), intertwined);
	glVertexPointer(3, GL_FLOAT, 6*sizeof(GLfloat), &intertwined[3]);

	//A Triangle Contain Vertices 2, 3, 5
	//glBegin(GL_TRIANGLES);
	//	glArrayElement (2);
	//	glArrayElement (3);
	//	glArrayElement (5);
	//glEnd();

	//6 is number of vertices - Draw In Sequential
	glDrawArrays(GL_TRIANGLES, 0, 6);

	glDisableClientState(GL_VERTEX_ARRAY);  
    glDisableClientState(GL_COLOR_ARRAY);

	glutSwapBuffers();
}

void glutDisplay2(void){
	glClear(GL_COLOR_BUFFER_BIT);

	static GLfloat intertwined[] ={
		1.0, 0.2, 1.0, 100.0, 100.0, 0.0,
		1.0, 0.2, 0.2, 300.0, 100.0, 0.0,
		1.0, 1.0, 0.2, 100.0, 300.0, 0.0,
		0.2, 1.0, 0.2, 300.0, 300.0, 0.0,
	};

	GLubyte indices[] = {
		0, 1, 2,
		1, 2, 3
	};

	glEnableClientState (GL_COLOR_ARRAY);
	glEnableClientState (GL_VERTEX_ARRAY);
	glColorPointer (3, GL_FLOAT, 6 * sizeof(GLfloat), intertwined);
	glVertexPointer(3, GL_FLOAT, 6*sizeof(GLfloat), &intertwined[3]);

	//glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_BYTE, indices);
	//2 is maximum indices in range (2 is max in 0,1,2 in this case)
	glDrawRangeElements(GL_TRIANGLES, 0, 2, 3, GL_UNSIGNED_BYTE, indices);
	glDrawRangeElements(GL_TRIANGLES, 1, 3, 3, GL_UNSIGNED_BYTE, indices + 3);

	glDisableClientState(GL_VERTEX_ARRAY);  
    glDisableClientState(GL_COLOR_ARRAY);

	glutSwapBuffers();
}

int main(int argc, char* argv[]){
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
	glutInitWindowSize(640, 480);
	glutCreateWindow ("OpenGL Test");

	glewInit();
	
	glutReshapeFunc (&ReSizeGLScene);
	glutKeyboardFunc(glutKeyboard);
	//glutDisplayFunc(glutDisplay);
	glutDisplayFunc(glutDisplay0);
	//glutDisplayFunc(glutDisplay1);
	//glutDisplayFunc(glutDisplay2);

	glutIdleFunc(glutIdle);

	glutMainLoop();
	return 0;
}