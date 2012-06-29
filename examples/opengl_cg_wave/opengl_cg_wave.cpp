#include <stdio.h>   
#include <stdlib.h> 
#include <iostream>
#include <string>

#include <GL/glew.h>
#include <GL/GL.h>
#include <GL/glut.h>

#include <Cg/cg.h>   
#include <Cg/cgGL.h>

#define		TWO_PI 6.2831853071	
#define		SIZE	64													// Defines The Size Of The X/Z Axis Of The Meshs
bool		cg_enable = TRUE, sp;										// Toggle Cg Program On / Off, Space Pressed?
GLfloat		mesh[SIZE][SIZE][3];										// Our Static Mesh
GLfloat		wave_movement = 0.0f;										// Our Variable To Move The Waves Across The Mesh

CGcontext	cgContext;													// A Context To Hold Our Cg Program(s)
CGprogram	cgProgram;													// Our Cg Vertex Program
CGprofile	cgVertexProfile;											// The Profile To Use For Our Vertex Shader
CGparameter	position, color, modelViewMatrix, wave;						// The Parameters Needed For Our Shader

using namespace std;
using std::string;

namespace{
	std::string const CG_SHADER_SOURCE_ROOT= "C:/Logs/PCL/beingthere/examples/opengl_cg_wave/shader/";
	std::string const CG_SHADER_SOURCE_NAME(CG_SHADER_SOURCE_ROOT + "Wave.cg");
}

void glutIdle (void)
{
	glutPostRedisplay();
}

void glutKeyboard (unsigned char key, int x, int y)
{
	if(key==27) {
		cgDestroyContext(cgContext);
		exit(0);
	}

	if(key == ' '){
		cg_enable = !cg_enable;
	}
}

void ReSizeGLScene (int Width, int Height)
{
	glViewport (0, 0, Width, Height);
	glMatrixMode (GL_PROJECTION);										// Select The Projection Matrix
	glLoadIdentity ();													// Reset The Projection Matrix
	gluPerspective (45.0f, (GLfloat)(Width)/(GLfloat)(Height),			// Calculate The Aspect Ratio Of The Window
		0.1f, 100.0f);		
	glMatrixMode (GL_MODELVIEW);										// Select The Modelview Matrix
	glLoadIdentity ();
}

void glutDisplay (void)
{
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);				// Clear Screen And Depth Buffer
	glLoadIdentity ();													// Reset The Modelview Matrix

	// Position The Camera To Look At Our Mesh From A Distance
	gluLookAt(0.0f, 25.0f, -45.0f, 0.0f, 0.0f, 0.0f, 0, 1, 0);

	// Set The Modelview Matrix Of Our Shader To Our OpenGL Modelview Matrix
	cgGLSetStateMatrixParameter(modelViewMatrix, CG_GL_MODELVIEW_PROJECTION_MATRIX, CG_GL_MATRIX_IDENTITY);

	if (cg_enable)
	{
		cgGLEnableProfile(cgVertexProfile);								// Enable Our Vertex Shader Profile

		// Bind Our Vertex Program To The Current State
		cgGLBindProgram(cgProgram);

		// Set The Drawing Color To Light Green (Can Be Changed By Shader, Etc...)
		cgGLSetParameter4f(color, 0.5f, 1.0f, 0.5f, 1.0f);
	}

	// Start Drawing Our Mesh
	for (int x = 0; x < SIZE - 1; x++)
	{
		// Draw A Triangle Strip For Each Column Of Our Mesh
		glBegin(GL_TRIANGLE_STRIP);
		for (int z = 0; z < SIZE - 1; z++)
		{
			// Set The Wave Parameter Of Our Shader To The Incremented Wave Value From Our Main Program
			cgGLSetParameter3f(wave, wave_movement, 1.0f, 1.0f);
			glVertex3f(mesh[x][z][0], mesh[x][z][1], mesh[x][z][2]);		// Draw Vertex
			glVertex3f(mesh[x+1][z][0], mesh[x+1][z][1], mesh[x+1][z][2]);	// Draw Vertex
			wave_movement += 0.00001f;									// Increment Our Wave Movement
			if (wave_movement > TWO_PI)									// Prevent Crashing
				wave_movement = 0.0f;
		}
		glEnd();
	}

	if (cg_enable)
		cgGLDisableProfile(cgVertexProfile);							// Disable Our Vertex Profile

	glFlush ();															// Flush The GL Rendering Pipeline
	glutSwapBuffers();
}

static void checkForCgError(const char *situation)
{
	CGerror error;
	const char *string = cgGetLastErrorString(&error);

	if (error != CG_NO_ERROR) {
		printf("%s: %s: %s\n",
			cgProgram, situation, string);
		if (error == CG_COMPILER_ERROR) {
			printf("%s\n", cgGetLastListing(cgContext));
		}
		exit(1);
	}
}

void setupRC(){
	// Create Our Mesh
	for (int x = 0; x < SIZE; x++)
	{
		for (int z = 0; z < SIZE; z++)
		{
			mesh[x][z][0] = (float) (SIZE / 2) - x;						// We Want To Center Our Mesh Around The Origin
			mesh[x][z][1] = 0.0f;										// Set The Y Values For All Points To 0
			mesh[x][z][2] = (float) (SIZE / 2) - z;						// We Want To Center Our Mesh Around The Origin
		}
	}

	//Create CG Program
	cgContext = cgCreateContext();
	cgVertexProfile = cgGLGetLatestProfile(CG_GL_VERTEX);
	cgGLSetOptimalOptions(cgVertexProfile);

	cgProgram = cgCreateProgramFromFile(cgContext, CG_SOURCE, CG_SHADER_SOURCE_NAME.c_str(), cgVertexProfile, "main", 0);

	cgGLLoadProgram(cgProgram);

	position		= cgGetNamedParameter(cgProgram, "IN.position");
	color			= cgGetNamedParameter(cgProgram, "IN.color");
	wave			= cgGetNamedParameter(cgProgram, "IN.wave");
	modelViewMatrix	= cgGetNamedParameter(cgProgram, "ModelViewProj");

	//Display Setting
	glClearColor (0.0f, 0.0f, 0.0f, 0.5f);								// Black Background
	glClearDepth (1.0f);												// Depth Buffer Setup
	glDepthFunc (GL_LEQUAL);											// The Type Of Depth Testing
	glEnable (GL_DEPTH_TEST);											// Enable Depth Testing
	glShadeModel (GL_SMOOTH);											// Select Smooth Shading
	glHint (GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);					// Set Perspective Calculations To Most Accurate
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);							// Draw Our Mesh In 
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

