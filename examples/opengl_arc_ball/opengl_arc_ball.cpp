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

using namespace std;

//Texture & Buffer
GLuint textureID;
GLuint bufferID;

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

#include "ArcBall.h"
const float PI2 = 2.0*3.1415926535f;								// PI Squared

Matrix4fT   Transform   = {  1.0f,  0.0f,  0.0f,  0.0f,				// NEW: Final Transform
                             0.0f,  1.0f,  0.0f,  0.0f,
                             0.0f,  0.0f,  1.0f,  0.0f,
                             0.0f,  0.0f,  0.0f,  1.0f };

Matrix3fT   LastRot     = {  1.0f,  0.0f,  0.0f,					// NEW: Last Rotation
                             0.0f,  1.0f,  0.0f,
                             0.0f,  0.0f,  1.0f };

Matrix3fT   ThisRot     = {  1.0f,  0.0f,  0.0f,					// NEW: This Rotation
                             0.0f,  1.0f,  0.0f,
                             0.0f,  0.0f,  1.0f };

ArcBallT    ArcBall(640.0f, 480.0f);				                // NEW: ArcBall Instance
Point2fT    MousePt;												// NEW: Current Mouse Point
bool        isClicked  = false;										// NEW: Clicking The Mouse?
bool        isRClicked = false;										// NEW: Clicking The Right Mouse Button?
bool        isDragging = false;					                    // NEW: Dragging The Mouse?

GLUquadricObj *quadratic;

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
  	glViewport (0, 0, (GLsizei)(Width), (GLsizei)(Height));				// Reset The Current Viewport
	glMatrixMode (GL_PROJECTION);										// Select The Projection Matrix
	glLoadIdentity ();													// Reset The Projection Matrix
	gluPerspective (45.0f, (GLfloat)(Width)/(GLfloat)(Height),			// Calculate The Aspect Ratio Of The Window
					1.0f, 100.0f);		
	glMatrixMode (GL_MODELVIEW);										// Select The Modelview Matrix
	glLoadIdentity ();													// Reset The Modelview Matrix

    ArcBall.setBounds((GLfloat)Width, (GLfloat)Height);                 //*NEW* Update mouse bounds for arcball
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

void initialize(){	
	glClearColor (0.0f, 0.0f, 0.0f, 0.5f);							// Black Background
	glClearDepth (1.0f);											// Depth Buffer Setup
	glDepthFunc (GL_LEQUAL);										// The Type Of Depth Testing (Less Or Equal)
	glEnable (GL_DEPTH_TEST);										// Enable Depth Testing
	glShadeModel (GL_FLAT);											// Select Flat Shading (Nice Definition Of Objects)
	glHint (GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);				// Set Perspective Calculations To Most Accurate

	quadratic=gluNewQuadric();										// Create A Pointer To The Quadric Object
	gluQuadricNormals(quadratic, GLU_SMOOTH);						// Create Smooth Normals
	gluQuadricTexture(quadratic, GL_TRUE);							// Create Texture Coords

	glEnable(GL_LIGHT0);											// Enable Default Light
	glEnable(GL_LIGHTING);											// Enable Lighting

	glEnable(GL_COLOR_MATERIAL);									// Enable Color Material	
}

void mouse(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN){
		buttonState |= 1<<button;

		if(buttonState == 4) isRClicked = true;
		if(buttonState == 1) isClicked = true;
	}
	else if (state == GLUT_UP){
		buttonState = 0;
		isClicked = false;
		isRClicked = false;
		isDragging = false;
	}

	int mods = glutGetModifiers();
	if (mods & GLUT_ACTIVE_SHIFT) {
		buttonState = 2;
	} else if (mods & GLUT_ACTIVE_CTRL) {
		buttonState = 3;
	}
}

void motion(int x, int y)
{
	MousePt.s.X = x;
	MousePt.s.Y = y;

	if(!isDragging && isClicked){
		isDragging = true;
		LastRot = ThisRot;										// Set Last Static Rotation To Last Dynamic One
		ArcBall.click(&MousePt);								// Update Start Vector And Prepare For Dragging
	}

	else{
		if (isClicked)												// Still Clicked, So Still Dragging
		{
			Quat4fT     ThisQuat;

			ArcBall.drag(&MousePt, &ThisQuat);						// Update End Vector And Get Rotation As Quaternion
			Matrix3fSetRotationFromQuat4f(&ThisRot, &ThisQuat);		// Convert Quaternion Into Matrix3fT
			Matrix3fMulMatrix3f(&ThisRot, &LastRot);				// Accumulate Last Rotation Into This One
			Matrix4fSetRotationFromMatrix3f(&Transform, &ThisRot);	// Set Our Final Transform's Rotation From This One
		}
		else														// No Longer Dragging
			isDragging = false;
	}

	if(buttonState == 4){
		Matrix3fSetIdentity(&LastRot);								// Reset Rotation
		Matrix3fSetIdentity(&ThisRot);								// Reset Rotation
        Matrix4fSetRotationFromMatrix3f(&Transform, &ThisRot);		// Reset Rotation
	}

	if(buttonState == 2){
		//cout << "Middle Mouse" << endl;
	}

	if(buttonState == 3){
		//cout << "Zoom" << endl;
	}
}

void glutDisplay (void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//Drawing
	glLoadIdentity();												// Reset The Current Modelview Matrix
	glTranslatef(1.5f,0.0f,-6.0f);									// Move Right 1.5 Units And Into The Screen 7.0

	glPushMatrix();													// NEW: Prepare Dynamic Transform
    glMultMatrixf(Transform.M);										// NEW: Apply Dynamic Transform

	glColor3f(1.0f,0.75f,0.75f);
	gluSphere(quadratic,1.3f,20,20);

	glPopMatrix();

	glutSwapBuffers();
}


int main(int argc, char* argv[]){
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
	glutInitWindowSize(640, 480);
	glutCreateWindow ("OpenGL Test");

	glewInit();

	initialize();

	glutReshapeFunc (&ReSizeGLScene);
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);
	
	glutMouseFunc(mouse);
	glutMotionFunc(motion);

	glutMainLoop();
	return 0;
}