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

GLSLProgram* shaderProgram = 0;

int triangleTemplateVertices;
GLuint triangleTemplateBuffer;

#define MAX_TRIANGLE_DEPTH_CM_PER_DEPTH_METER 2.0
float max_triangle_z_offset = MAX_TRIANGLE_DEPTH_CM_PER_DEPTH_METER;

//Button, Keyboard Function
int ox, oy;
int buttonState = 0;
bool keyDown[256];

//glm::vec3 cameraPos(0, -1, -4);
//glm::vec3 cameraPosLag(cameraPos);
glm::vec3 cameraRot(0, 0, 0);
glm::vec3 cameraRotLag(cameraRot);

const float inertia = 0.1f * 0.03f;
const float translateSpeed = 0.002f * 100;
const float rotateSpeed = 0.2f;
//const float cursorSpeed = 0.01f;
//const float walkSpeed = 0.05f;

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

namespace{
	std::string const SHADER_ROOT_PATH = BEINGTHERE_EXAMPLE_ROOT_DIR + "opengl_geometry_triangle/";
	std::string const VERT_SHADER_SOURCE(SHADER_ROOT_PATH + "shader/geometry_triangle.vert");
	std::string const GEOM_SHADER_SOURCE(SHADER_ROOT_PATH + "shader/geometry_triangle.geom");
	std::string const FRAG_SHADER_SOURCE(SHADER_ROOT_PATH + "shader/geometry_triangle.frag");
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

void updateMVPOrtho(glm::mat4& MVP, float w, float h){
	glm::mat4 Projection = glm::ortho(0.f, w, 0.f, h, -1.f, 1.f);
	glm::mat4 View = glm::mat4(1.f);
	glm::mat4 Model = glm::mat4(1.0f);

	MVP = Projection * View * Model;
}

void updateMVPPersp(glm::mat4& MVP){
	Projection = glm::perspective(45.0f, 640.0f / 480.0f, 1.f, 5000.f);
	View = glm::translate(glm::mat4(1.f), glm::vec3(0.f, 0.f, -500.f));
	Model = glm::mat4(1.0f);
	
	MVP = Projection * View * Model;
}

void drawTriangleOrtho(){
	glClearColor(.3, .3, .3, 0);
	glClear(GL_COLOR_BUFFER_BIT);

	glEnable(GL_TEXTURE_2D);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureDepthID);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, textureColorID);

	shaderProgram->use();
	
	//cout << shaderProgram->log() << endl;
	//shaderProgram->printActiveAttribs();
	//shaderProgram->printActiveUniforms();

	shaderProgram->setUniform("texColor", 1);
	shaderProgram->setUniform("texDepth", 0);

	//updateMVPOrtho(MVP, 640.0, 480.0);
	shaderProgram->setUniform("MVP", MVP);
	shaderProgram->setUniform("max_triangle_z_offset", max_triangle_z_offset);

	float fx, fy, cx, cy;
	fx = fy = 532.f;
	cx = cy = 323.f;

	shaderProgram->setUniform("fx", fx);
	shaderProgram->setUniform("fy", fy);
	shaderProgram->setUniform("cx", cx);
	shaderProgram->setUniform("cy", cy);

	//Drawing
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, triangleTemplateBuffer);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, triangleTemplateVertices);
	
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, 0);	
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, 0);

	glDisable(GL_TEXTURE_2D);
}

void glutDisplay (void)
{
	//Drawing
	drawTriangleOrtho();

	glutSwapBuffers();
}

void createShader(){
	shaderProgram = new GLSLProgram();
	shaderProgram->compileShaderFromFile(VERT_SHADER_SOURCE.c_str(), GLSLShader::VERTEX);   cout << shaderProgram->log() << endl;
	shaderProgram->compileShaderFromFile(GEOM_SHADER_SOURCE.c_str(), GLSLShader::GEOMETRY); cout << shaderProgram->log() << endl;
	shaderProgram->compileShaderFromFile(FRAG_SHADER_SOURCE.c_str(), GLSLShader::FRAGMENT); cout << shaderProgram->log() << endl;
	shaderProgram->link(); cout << shaderProgram->log() << endl;
	shaderProgram->validate(); cout << shaderProgram->log() << endl;
	updateMVPPersp(MVP);
}

void createVertexBuffer(){
	int nrow = 480; //480
	int ncol = 640; //640

	triangleTemplateVertices = (nrow-1)*(2*ncol)+(int)(nrow/2);
	int size = triangleTemplateVertices*3*sizeof(GLshort);
	GLshort *data = (GLshort*)malloc(size);

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

	glGenBuffers(1, &triangleTemplateBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, triangleTemplateBuffer);
	glBufferData(GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_SHORT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);
	//glVertexPointer(3, GL_SHORT, 0, 0);
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
	
	//Load Texture
	std::string imgColorPath = BEINGTHERE_ROOT_DIR + "resources/multi-cam/data/snapshot_rgb_3.png";
	std::string imgDepthPath = BEINGTHERE_ROOT_DIR + "resources/multi-cam/data/snapshot_depth_3.png";
	/*std::string imgColorPath = "D:/Trunk/bibleProj/beingthere-data/multicams/snapshot_rgb_9.png";
	std::string imgDepthPath = "D:/Trunk/bibleProj/beingthere-data/multicams/snapshot_depth_9.png";*/

	createTexture(textureColorID, false);
	uploadToTexture(textureColorID, imgColorPath, false);

	createTexture(textureDepthID, true);
	uploadToTexture(textureDepthID, imgDepthPath, true);

	createVertexBuffer();
	createShader();

	glutReshapeFunc (&ReSizeGLScene);
	glutKeyboardFunc(glutKeyboard);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glutMainLoop();
	return 0;
}


//Triangle Fusion - Old - Keep For Safe
//#include <stdio.h>
//#include <iostream>
//#include <stdlib.h>
//#include <time.h>
//
//#include <GL/glew.h>
//#include <GL/GL.h>
//#include <GL/freeglut.h>
//
//#include <cv.h>
//#include <highgui.h>
//
//#include "glm/glm.hpp"
//#include <glm/gtc/matrix_transform.hpp>
//
//#include "glslprogram.h"
//
//using namespace std;
//
//#define MAX_CAM 10
//
////Texture & Buffer
//GLuint textureColorID;
//GLuint textureDepthID;
//
//namespace SHADER_ATTR{
//	enum type{
//		POSITION = 0,
//		NORMAL = 1,
//		COLOR = 3,
//		TEXCOORD = 4
//	};
//} //end namespace attr
//
//GLSLProgram* progTriangles = 0;
//GLSLProgram* progTrianglesQuality = 0;
//GLSLProgram* progTrianglesPassThruQuality = 0;
//
//glm::mat4 Projection;
//glm::mat4 View;
//glm::mat4 Model;
//glm::mat4 MVP;
//
//int triangleTemplateVertices;
//GLuint triangleTemplateBuffer;
//
//bool zCorrect = 1;
//float zCorrectCoeff[MAX_CAM][3] = {{0,1,0},{0,1,0},{0,1,0},{0,1,0},{0,1,0},{0,1,0},{0,1,0},{0,1,0},{0,1,0},{0,1,0}};
//
//#define MAX_TRIANGLE_DEPTH_CM_PER_DEPTH_METER 2.0
//float max_triangle_z_offset = MAX_TRIANGLE_DEPTH_CM_PER_DEPTH_METER;
//
//GLuint feedbackVertexArray[MAX_CAM];
//GLuint feedbackBuffer[MAX_CAM];
//GLuint query[MAX_CAM];
//
//namespace{
//	//std::string const SHADER_ROOT_PATH = "C:/Logs/PCL/beingthere/examples/opengl_triangle_renderer/";
//	std::string const SHADER_ROOT_PATH = "D:/Trunk/bibleProj/beingthere/examples/opengl_texture_access/";
//	std::string const VERT_SHADER_SOURCE(SHADER_ROOT_PATH + "shader/dummy.vert");
//	std::string const VERT_PASSTHRU_SHADER_SOURCE(SHADER_ROOT_PATH + "shader/triangles_passthru.vert");
//	std::string const GEOM_SHADER_SOURCE(SHADER_ROOT_PATH + "shader/triangles.geom");
//	std::string const FRAG_SHADER_SOURCE(SHADER_ROOT_PATH + "shader/light.frag");
//}
//
//void glutIdle (void)
//{
//	glutPostRedisplay();
//}
//
//void glutKeyboard (unsigned char key, int x, int y)
//{
//	if(key==27) exit(0);
//}
//
//void ReSizeGLScene (int Width, int Height)
//{
//  glViewport (0, 0, Width, Height);
//}
//
//void updateMVPOrtho(glm::mat4& MVP){
//	Projection = glm::ortho(0.f, 640.f, 0.f, 480.f, -1.f, 1.f);
//	View = glm::mat4(1.f);
//	Model = glm::mat4(1.0f);
//	
//	MVP = Projection * View * Model;
//}
//
//void updateMVPPersp(glm::mat4& MVP){
//	Projection = glm::perspective(45.0f, 640.0f / 480.0f, 1.f, 5000.f);
//	View = glm::translate(glm::mat4(1.f), glm::vec3(0.f, 0.f, -500.f));
//	Model = glm::mat4(1.0f);
//	
//	MVP = Projection * View * Model;
//}
//
//void createVertexBuffer(){
//	//Triangle mesh template (VBO)
//	triangleTemplateVertices = 479*(2*640)+240;
//	int size = triangleTemplateVertices*3*sizeof(GLshort);
//	GLshort *data = (GLshort*)malloc(size);
//
//	int pos = 0;
//	for(int j = 0; j < 480-1; j++) {
//		if(j%2==0) {
//			for(int k = 0; k < 640; k++) {
//				data[pos++] = k; data[pos++] = j;   data[pos++] = 0; //vertex 1
//				data[pos++] = k; data[pos++] = j+1; data[pos++] = 0; //vertex 2
//			}
//			data[pos++] = 639; data[pos++] = j+1; data[pos++] = 0; //dummy vertex to align rows
//		} else {
//			for(int k = 639; k >= 0; k--) {
//				data[pos++] = k; data[pos++] = j+1; data[pos++] = 0; //vertex 1
//				data[pos++] = k; data[pos++] = j;   data[pos++] = 0; //vertex 2
//			}
//		}			
//	}
//
//	glGenBuffers(1, &triangleTemplateBuffer);
//	glBindBuffer(GL_ARRAY_BUFFER, triangleTemplateBuffer);
//	glBufferData(GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW);
//	glVertexPointer(3, GL_SHORT, 0, 0);
//
//	//create query used to determine number of triangles drawn during transform feedback
//	int numDev = 10;
//	glGenQueries(numDev, query);
//
//	//Create buffer for transform feedback
//	glGenBuffers(numDev, feedbackBuffer);
//	for(int i = 0; i < numDev; i++) {
//		glBindBuffer(GL_ARRAY_BUFFER, feedbackBuffer[i]);
//		glBufferData(GL_ARRAY_BUFFER, 640*480*2*3*40, NULL, GL_DYNAMIC_DRAW);
//		glBindBuffer(GL_ARRAY_BUFFER, 0);
//	}
//
//	//create vertex array for transform feedback and attach to buffer
//	glGenVertexArrays(numDev, feedbackVertexArray);
//	for(int i = 0; i < numDev; i++) {
//		glBindVertexArray(feedbackVertexArray[i]);
//		glBindBuffer(GL_ARRAY_BUFFER, feedbackBuffer[i]);
//
//		glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 40, (GLvoid*)0);
//		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 40, (GLvoid*)16);
//		glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 40, (GLvoid*)32);
//		glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, 40, (GLvoid*)36);
//
//		glEnableVertexAttribArray(0);
//		glEnableVertexAttribArray(1);
//		glEnableVertexAttribArray(2);
//		glEnableVertexAttribArray(3);
//	}
//
//	glBindBuffer(GL_ARRAY_BUFFER, 0);	
//	glBindVertexArray(0);
//}
//
//void createShader(){
//	progTriangles = new GLSLProgram();
//	progTriangles->compileShaderFromFile(VERT_SHADER_SOURCE.c_str(), GLSLShader::VERTEX);   cout << progTriangles->log() << endl;
//	progTriangles->compileShaderFromFile(GEOM_SHADER_SOURCE.c_str(), GLSLShader::GEOMETRY); cout << progTriangles->log() << endl;	
//	glProgramParameteriARB(progTriangles->getHandle(), GL_GEOMETRY_INPUT_TYPE_EXT,GL_TRIANGLES);
//	glProgramParameteriARB(progTriangles->getHandle(), GL_GEOMETRY_OUTPUT_TYPE_EXT,GL_TRIANGLES);
//	glProgramParameteriARB(progTriangles->getHandle(), GL_GEOMETRY_VERTICES_OUT_EXT,3);
//	progTriangles->link(); cout << progTriangles->log() << endl;	
//	progTriangles->validate();
//
//	progTrianglesQuality = new GLSLProgram();
//	progTrianglesQuality->compileShaderFromFile(VERT_SHADER_SOURCE.c_str(), GLSLShader::VERTEX);   cout << progTrianglesQuality->log() << endl;
//	progTrianglesQuality->compileShaderFromFile(GEOM_SHADER_SOURCE.c_str(), GLSLShader::GEOMETRY); cout << progTrianglesQuality->log() << endl;	
//	progTrianglesQuality->compileShaderFromFile(FRAG_SHADER_SOURCE.c_str(), GLSLShader::FRAGMENT); cout << progTrianglesQuality->log() << endl;	
//	glProgramParameteriARB(progTrianglesQuality->getHandle(), GL_GEOMETRY_INPUT_TYPE_EXT,GL_TRIANGLES);
//	glProgramParameteriARB(progTrianglesQuality->getHandle(), GL_GEOMETRY_OUTPUT_TYPE_EXT,GL_TRIANGLES);
//	glProgramParameteriARB(progTrianglesQuality->getHandle(), GL_GEOMETRY_VERTICES_OUT_EXT,3);
//	GLchar const * ftvaryings[] = {"gl_Position", "gl_TexCoord[0]", "nDotL", "distanceToCam"};
//	glTransformFeedbackVaryings(progTrianglesQuality->getHandle(), 4, ftvaryings, GL_INTERLEAVED_ATTRIBS);
//	progTrianglesQuality->link(); cout << progTrianglesQuality->log() << endl;	
//	progTrianglesQuality->validate(); cout << progTriangles->log() << endl;	
//
//	progTrianglesPassThruQuality = new GLSLProgram();
//	progTrianglesPassThruQuality->compileShaderFromFile(VERT_PASSTHRU_SHADER_SOURCE.c_str(), GLSLShader::VERTEX);   cout << progTriangles->log() << endl;
//	progTrianglesPassThruQuality->compileShaderFromFile(FRAG_SHADER_SOURCE.c_str(), GLSLShader::FRAGMENT); cout << progTriangles->log() << endl;	
//	progTrianglesPassThruQuality->link();
//	progTrianglesPassThruQuality->validate(); cout << progTriangles->log() << endl;	
//}
//
//void setupShaderDraw(GLSLProgram* program) {
//	//Temporary Default Params
//	float fx, fy, cx, cy;
//	fx = fy = 532.f;
//	cx = cy = 323.f;
//	int dev = 0;
//
//	program->setUniform("fx", fx);
//	program->setUniform("fy", fy);
//	program->setUniform("cx", cx);
//	program->setUniform("cy", cy);
//	program->setUniform("zCorrectQuad", zCorrect?zCorrectCoeff[dev][0]:0.0f);
//	program->setUniform("zCorrectLinear", zCorrect?zCorrectCoeff[dev][1]:1.0f);
//	program->setUniform("zCorrectConstant", zCorrect?zCorrectCoeff[dev][2]:0.0f);
//}
//
//void drawTriangles(){
//	//qualityMetricMode = 0
//	//progTriangles->use();
//	progTrianglesQuality->use();
//
//	glEnable(GL_TEXTURE_2D);
//
//	//Setup Active Texture
//	glActiveTexture(GL_TEXTURE0);
//	glBindTexture(GL_TEXTURE_2D, textureColorID);
//	glActiveTexture(GL_TEXTURE1);
//	glBindTexture(GL_TEXTURE_2D, textureDepthID);
//
//	//progTriangles->setUniform("texColor", 0);
//	//progTriangles->setUniform("texDepth", 1);
//	//progTriangles->setUniform("max_triangle_z_offset", max_triangle_z_offset);
//	//progTriangles->setUniform("calcQuality", 0);
//	//updateMVPPersp(MVP);
//	//progTriangles->setUniform("MVP", MVP);
//
//	progTrianglesQuality->setUniform("texColor", 0);
//	progTrianglesQuality->setUniform("texDepth", 1);
//	progTrianglesQuality->setUniform("max_triangle_z_offset", max_triangle_z_offset);
//	progTrianglesQuality->setUniform("calcQuality", 1);
//	updateMVPPersp(MVP);
//	progTrianglesQuality->setUniform("MVP", MVP);
//
//	//setupShaderDraw(progTriangles);
//	setupShaderDraw(progTrianglesQuality);
//
//	int phyDev = 0;
//	glBeginQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN, query[phyDev]); 
//	glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, feedbackBuffer[phyDev]);
//	glBeginTransformFeedback(GL_TRIANGLES);
//
//	//Draw
//	glBindVertexArray(0);
//	glBindBuffer(GL_ARRAY_BUFFER, triangleTemplateBuffer);
//	glDrawArrays(GL_TRIANGLE_STRIP, 0, triangleTemplateVertices);
//
//	glEndTransformFeedback();
//	glEndQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN); 
//
//	GLuint triangles = 0;
//	glGetQueryObjectuiv(query[phyDev], GL_QUERY_RESULT, &triangles);
//
//	cout << "Num:= " << triangles << endl;
//
//	//Deactive Texture
//	glActiveTexture(GL_TEXTURE0);
//	glBindTexture(GL_TEXTURE_2D, 0);	
//	glActiveTexture(GL_TEXTURE1);
//	glBindTexture(GL_TEXTURE_2D, 0);
//
//	glDisable(GL_TEXTURE_2D);
//	glBindTexture(GL_TEXTURE_2D, 0);
//	glUseProgram(0);
//}
//
//void glutDisplay (void)
//{
//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//	drawTriangles(); 
//	glutSwapBuffers(); 
//}
//
//bool createTexture(GLuint& textureID, bool isDepth){
//	glGenTextures(1, &textureID);
//	glBindTexture(GL_TEXTURE_2D, textureID);
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
//
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//	
//	if(!isDepth){
//		unsigned char * dummyTex = (unsigned char *)malloc(640*480*3);
//		memset(dummyTex, 0, 640*480*3);
//		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, dummyTex);
//	}
//	else{
//		unsigned char * dummyTex = (unsigned char *)malloc(640*480*2);
//		memset(dummyTex, 0, 640*480*2);
//		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16, 640, 480, 0, GL_GREEN, GL_UNSIGNED_SHORT, dummyTex);
//	}
//
//	glBindTexture(GL_TEXTURE_2D, 0);
//
//	return true;
//}
//
//bool uploadToTexture(GLuint& textureID, string imgPath, bool isDepth){
//	cv::Mat img = cv::imread(imgPath.c_str(), CV_LOAD_IMAGE_UNCHANGED);
//	
//	glBindTexture(GL_TEXTURE_2D, textureID);
//
//	//glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
//	/*glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 640, 480, 0,
//		GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*) img.data);*/
//
//	if(!isDepth)
//		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_RGB, GL_UNSIGNED_BYTE, img.data);
//	else 
//		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_GREEN, GL_UNSIGNED_SHORT, img.data);
//
//	glBindTexture(GL_TEXTURE_2D, 0);
//
//	return true;
//}
//
//int main(int argc, char* argv[]){
//	glutInit(&argc, argv);
//	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
//	glutInitWindowSize(640, 480);
//	glutCreateWindow ("OpenGL Test");
//
//	glewInit(); 
//	//Load Texture
//	//std::string imgColorPath = "C:/Logs/PCL/data/snapshot_rgb_3.png";
//	//std::string imgDepthPath = "C:/Logs/PCL/data/snapshot_depth_3.png";
//	std::string imgColorPath = "D:/Trunk/bibleProj/beingthere-data/multicams/snapshot_rgb_3.png";
//	std::string imgDepthPath = "D:/Trunk/bibleProj/beingthere-data/multicams/snapshot_depth_3.png";
//
//	createTexture(textureColorID, false);
//	uploadToTexture(textureColorID, imgColorPath, false);
//
//	createTexture(textureDepthID, true);
//	uploadToTexture(textureDepthID, imgDepthPath, true);
//
//	createVertexBuffer();
//	createShader();
//
//	glutReshapeFunc (&ReSizeGLScene);
//	glutKeyboardFunc(glutKeyboard);
//	glutDisplayFunc(glutDisplay);
//	glutIdleFunc(glutIdle);
//
//	glutMainLoop();
//	return 0;
//}