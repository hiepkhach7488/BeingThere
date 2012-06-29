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

using std::string;
using namespace std;
using namespace V3D_GPU;

ParametricUndistortionFilter _parametricUndistortionFilter;

void glutIdle (void)
{
	glutPostRedisplay();
}

void glutKeyboard (unsigned char key, int x, int y)
{
	if(key==27) {
		exit(0);
	}

	if(key == ' '){
	}
}

void ReSizeGLScene (int Width, int Height)
{
	glViewport (0, 0, Width, Height);
	glMatrixMode (GL_PROJECTION);										
	glLoadIdentity ();												
	
	glMatrixMode (GL_MODELVIEW);										
	glLoadIdentity ();
}

void glutDisplay (void)
{
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);				
	

	glutSwapBuffers();
}

static void checkForCgError(const char *situation)
{
}

void setupRC(){
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

