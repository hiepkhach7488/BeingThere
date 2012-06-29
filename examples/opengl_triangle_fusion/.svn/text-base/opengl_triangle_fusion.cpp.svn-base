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
#include "calibration_loader.h"
#include "texture_loader.h"
#include "triangle_fusion.h"

using namespace std;

#define MAX_CAM 10

glm::mat4 Projection;
glm::mat4 View;
glm::mat4 Model;
glm::mat4 MVP;

//Mouse - Control
int ox, oy;
int buttonState = 0;
bool keyDown[256];

glm::vec3 cameraRot(0, 0, 0);
glm::vec3 cameraRotLag(cameraRot);

const float inertia = 0.1f * 0.03f;
const float translateSpeed = 0.002f * 300;
const float rotateSpeed = 0.2f;

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


//Shader Option
bool useQualityShader = true;
bool renderCloud = false;

namespace{
	TextureLoader* textureLoader = 0;
	TriangleFusion* triangleFusion = 0;
}

void glutIdle (void)
{
	glutPostRedisplay();
}

int choice = 0;
void glutKeyboard (unsigned char key, int x, int y)
{
	if(key==27) exit(0);
	if(key=='c'){
		useQualityShader = !useQualityShader;
		glutPostRedisplay();
	}

	if(key=='p'){
		renderCloud = !renderCloud;
		glutPostRedisplay();
	}

	if(key == 'n'){
		choice = (choice + 1) % 9;
	}
}

void ReSizeGLScene (int Width, int Height)
{
  glViewport (0, 0, Width, Height);
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();
  glOrtho (0, 1, 0, 1, -1, 1);
  glMatrixMode (GL_MODELVIEW);
}

void updateMVPOrtho(glm::mat4& MVP){
	Projection = glm::ortho(0.f, 640.f, 0.f, 480.f, -1.f, 1.f);
	View = glm::mat4(1.f);
	Model = glm::mat4(1.0f);
	
	MVP = Projection * View * Model;
}

void updateMVPPersp(glm::mat4& MVP){
	Projection = glm::perspective(45.0f, 640.0f / 480.0f, 1.f, 5000.f);
	//View = glm::translate(glm::mat4(1.f), glm::vec3(0.f, 0.f, -500.f));
	View = glm::translate(glm::mat4(1.f), glm::vec3(0.f, 0.f, -600.f));
	Model = glm::mat4(1.0f);
	
	MVP = Projection * View * Model;
}

void setupRC(){
	textureLoader = new TextureLoader();
	triangleFusion = new TriangleFusion();

	//Upload Texture
	textureLoader->loadImageData();
	textureLoader->initTexture();
	textureLoader->uploadToTexture();
	textureLoader->createShader();
	textureLoader->processTextures();

	//Other
	updateMVPPersp(MVP);

	//Initialize Fusion
	triangleFusion->loadCalibrationData();
	triangleFusion->createShaderProgram();
	triangleFusion->createTriangleTemplate();
	triangleFusion->initTextures();

	//Light Params
	float constAtt = 0;	
	float linearAtt = 0;
	float quadAtt = 2.77e-4;
	glLightfv(GL_LIGHT0, GL_CONSTANT_ATTENUATION, &constAtt);
	glLightfv(GL_LIGHT0, GL_LINEAR_ATTENUATION, &linearAtt);
	glLightfv(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, &quadAtt);
}

void glutDisplay (void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	//triangleFusion->renderFromSingle(3, textureLoader->colorTex[3], textureLoader->depthTex[3], MVP);
	
	switch(choice){
	case 0:
		triangleFusion->renderCloudFromSingle(0, textureLoader->colorTex[0], textureLoader->depthTex[0], MVP);
		break;
	case 1:
		triangleFusion->renderCloudFromSingle(0, textureLoader->colorTex[0], textureLoader->processedDepthTex[0], MVP);
		break;
	case 2:
		triangleFusion->renderCloudFromAll(textureLoader->colorTex, textureLoader->depthTex, MVP);
		break;
	case 3:
		triangleFusion->renderCloudFromAll(textureLoader->colorTex, textureLoader->processedDepthTex, MVP);
		break;
	case 4:
		triangleFusion->renderFromSingle(0, textureLoader->colorTex[0], textureLoader->depthTex[0], MVP);
		break;
	case 5:
		triangleFusion->renderFromSingle(0, textureLoader->colorTex[0], textureLoader->processedDepthTex[0], MVP);
		break;
	case 6:
		triangleFusion->renderFromAll(textureLoader->colorTex, textureLoader->depthTex, MVP);
		break;
	case 7:
		triangleFusion->renderFromAll(textureLoader->colorTex, textureLoader->processedDepthTex, MVP);
		break;
	case 8:
		triangleFusion->renderToTempTexture(textureLoader->colorTex, textureLoader->processedDepthTex, MVP);
		triangleFusion->renderQualityTexture();
		break;
	}

	//if(!renderCloud){
	//	if(!useQualityShader)
	//		triangleFusion->renderFromAll(textureLoader->colorTex, textureLoader->processedDepthTex, MVP);
	//	else {
	//		triangleFusion->renderToTempTexture(textureLoader->colorTex, textureLoader->processedDepthTex, MVP);
	//		triangleFusion->renderQualityTexture();
	//	}
	//}
	//else{
	//	if(!useQualityShader){
	//		triangleFusion->renderCloudFromAll(textureLoader->colorTex, textureLoader->depthTex, MVP);
	//	}
	//	else 
	//		triangleFusion->renderCloudFromAll(textureLoader->colorTex, textureLoader->processedDepthTex, MVP);
	//}

	//triangleFusion->renderCloudFromAll(textureLoader->colorTex, textureLoader->depthTex, MVP);
	//triangleFusion->renderCloudFromAll(textureLoader->colorTex, textureLoader->processedDepthTex, MVP);
	//triangleFusion->renderFromSingle(0, textureLoader->colorTex[0], textureLoader->depthTex[0], MVP);

	glDisable(GL_DEPTH_TEST);
	glutSwapBuffers(); 
}


//Mouse, Keyboard Function
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

    ox = x; oy = y;

    glutPostRedisplay();
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

	////////////////////////////////////////////
    float dx, dy;
    dx = (float)(x - ox);
    dy = (float)(y - oy);

	if (buttonState == 1) {
		//Copy Transform Rotation to Model Matrix
		for(int i=0; i<4; ++i)
			for(int j=0; j<4; ++j){
				Model[i][j] = Transform.M[i*4+j];
			}
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

	glewInit(); 

	setupRC();

	glutReshapeFunc (&ReSizeGLScene);
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);

	glutMainLoop();
	return 0;
}