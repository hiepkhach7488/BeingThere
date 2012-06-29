#include <GL/glew.h>
#include <GL/GL.h>
#include <GL/glut.h>

#include "Base/v3d_imageprocessing.h"
#include "Base/v3d_timer.h"
#include "GL/v3d_gpubase.h"
#include "GL/v3d_gpubinarysegmentation.h"
#include <Cg/cg.h>   
#include <Cg/cgGL.h>

#include "v3d_shaders.h"
#include "v3d_common.h"
#include "beingthere_config.h"
#include "v3d_segmentation.h"
#include "v3d_depth_inpainting.h"

#include "glm/glm.hpp"
#include <glm/gtc/matrix_transform.hpp>

using namespace V3D;
//using namespace V3D_GPU;
using namespace beingthere;

void updateMVPPersp(glm::mat4& MVP);
void drawTriangleOrtho();
void mouse(int button, int state, int x, int y);
void motion(int x, int y);
bool uploadToTexture(GLuint& textureID, string imgPath, bool isDepth);

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

V3D_GPU::ImageTexture2D bgColorTex, curColorTex;
V3D_GPU::ImageTexture2D bgDepthTex, curDepthTex;
V3D_GPU::ImageTexture2D tempColorTex, tempDepthTex;

V3D_GPU::RTT_Buffer* seg_buffer;

void glutIdle (void)
{
	glutPostRedisplay();
}

void glutKeyboard (unsigned char key, int x, int y)
{
	if(key==27) {
		exit(0);
	}

	if(key == 's'){

	}
}

void ReSizeGLScene (int Width, int Height)
{
	glViewport (0, 0, Width, Height);
	glMatrixMode (GL_PROJECTION);										
	glLoadIdentity ();												
	glOrtho (0, 1, 0, 1, -1, 1);
	glMatrixMode (GL_MODELVIEW);										
	glLoadIdentity ();
}

void compareTextures(GLuint tex1, GLuint tex2){
	float* depth_data1 = (float*) malloc(640*480*sizeof(float));
	glGetTextureImageEXT(tex1, GL_TEXTURE_2D, 0, GL_RED, GL_FLOAT, depth_data1);

	float* depth_data2 = (float*) malloc(640*480*sizeof(float));
	glGetTextureImageEXT(tex2, GL_TEXTURE_2D, 0, GL_RED, GL_FLOAT, depth_data2);

	for (int y = 0; y < 480; ++y)
		for (int x = 0; x < 640; ++x){
			int idx = y * 640 + x;
			//if(idx % 200 == 0) cout << depth_data1[idx] << endl;
			if(abs(depth_data2[idx] - depth_data1[idx]) > 0.5) cout << depth_data2[idx] << " " << depth_data1[idx] << endl;
		}
}

void testVideoWriter(string imgPath){
	cv::Mat img = cv::imread(imgPath.c_str(), CV_LOAD_IMAGE_UNCHANGED);

	cv::VideoWriter vidWriter("test.avi", CV_FOURCC('P','I','M','1'), 24, cvSize(640, 480), 1); //CV_FOURCC('M','J','P','G')
	
	if(vidWriter.isOpened()) cout << "Opened Sucessfully" << endl;

	for(int i=0; i<3000; ++i)
		vidWriter << img;

	cout << "Finish Writing Video" << endl;
	getchar();
}

void readData(){
	string const DATA_ROOT_PATH = BEINGTHERE_EXAMPLE_ROOT_DIR + "opengl_cg_segmentation/data/old/";

	int w = 640, h = 480;
	Image<uchar> bgColor(w, h, 3), curColor(w, h, 3);;

	readImage<uchar, uchar>(DATA_ROOT_PATH + "bg_color.dat", bgColor, 3);
	readImage<uchar, uchar>(DATA_ROOT_PATH + "cur_color.dat", curColor, 3);

	Image<float> bgDepth(w, h, 1), curDepth(w, h, 1);
	readImage<float, float>(DATA_ROOT_PATH + "bg_depth.dat", bgDepth, 1);
	readImage<float, float>(DATA_ROOT_PATH + "cur_depth.dat", curDepth, 1);

	/*Convert To Meters Unit*/
	for(int i=0; i<480; ++i)
		for(int j=0; j<640; ++j){
			bgDepth(j,i) = bgDepth(j,i) / 1000.f;
			curDepth(j,i) = curDepth(j,i) / 1000.f;
		}

	bgColorTex.allocateID();
	bgColorTex.reserve(640, 480, "rgb=8");
	bgColorTex.overwriteWith(bgColor.begin(0), bgColor.begin(1), bgColor.begin(2));

	curColorTex.allocateID();
	curColorTex.reserve(640, 480, "rgb=8");
	curColorTex.overwriteWith(curColor.begin(0), curColor.begin(1), curColor.begin(2));

	//Test Erode Depth
	//beingthere::erodeDepth(1, curDepth);

	bgDepthTex.allocateID();
	bgDepthTex.reserve(w, h, "r=32f enableTextureRG");
	bgDepthTex.overwriteWith(bgDepth.begin(0), 1);

	curDepthTex.allocateID();
	curDepthTex.reserve(w, h, "r=32f enableTextureRG");
	curDepthTex.overwriteWith(curDepth.begin(0), 1);

	tempDepthTex.allocateID();
	tempDepthTex.reserve(w, h, "r=32f enableTextureRG");

	//Image<ushort> bgShortDepth(w,h,1), curShortDepth(w,h,1);
	//readImage<ushort, float>(DATA_ROOT_PATH + "bg_depth.dat", bgShortDepth, 1);
	//readImage<ushort, float>(DATA_ROOT_PATH + "cur_depth.dat", curShortDepth, 1);

	//GLuint tempTextureDepthID;
	//glGenTextures(1, &tempTextureDepthID);
	//createTexture(tempTextureDepthID, true);

	//glBindTexture(GL_TEXTURE_2D, tempTextureDepthID);
	//glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_GREEN, GL_UNSIGNED_SHORT, curShortDepth.begin(0));
	//glBindTexture(GL_TEXTURE_2D, 0);


	V3dShader* v3dShader = new V3dShader();
	v3dShader->initTextureFBO();
	v3dShader->createShader();

	v3dShader->convertFloatToUshort(curDepthTex.textureID(), textureDepthID);
	v3dShader->convertUshortDepthToRGB(textureDepthID, textureColorID);

	//Test RTT_Buffer
	/*seg_buffer = new V3D_GPU::RTT_Buffer("rg=32f enableTextureRG", "RTT Buffer");
	seg_buffer->allocate(w,h);
	seg_buffer->activate();

	V3dSegmentation* v3d_segmentation = new V3dSegmentation();
	v3d_segmentation->performSegmentation(bgColorTex.textureID(), bgDepthTex.textureID(), curColorTex.textureID(), curDepthTex.textureID(), seg_buffer->textureID());

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glActiveTexture(GL_TEXTURE0);

	v3dShader->maskColor(curColorTex.textureID(), curColorTex.textureID(), seg_buffer->textureID());
	v3dShader->maskDepth(curDepthTex.textureID(), curDepthTex.textureID(), seg_buffer->textureID());

	v3dShader->truncateRGBFromDepth(curColorTex.textureID(), curDepthTex.textureID(), curColorTex.textureID(), 2.5);
	v3dShader->truncateDepth(curDepthTex.textureID(), curDepthTex.textureID(), 2.5);

	GPU_DepthInpainting _depthInpainting;
	_depthInpainting.allocate(640, 480);
	_depthInpainting.initialize(1.0f);
	_depthInpainting.iterate(curDepthTex.textureID(), v3d_segmentation->_edgeWeightBuf.textureID());
	beingthere::copyTexture(_depthInpainting.getResult().getTexture().textureID(), tempDepthTex.textureID());

	v3dShader->convertFloatToUshort(tempDepthTex.textureID(), textureDepthID);*/
	//compareTextures(curDepthTex.textureID(), tempDepthTex.textureID());
}

void glutDisplay (void)
{
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		
	V3D_GPU::setupNormalizedProjection(true);

	//GLuint tempId[] = {curColorTex.textureID(), curDepthTex.textureID()};
	//beingthere::draw2DTextures(tempId, 2);

	glEnable(GL_TEXTURE_2D);

	//glActiveTexture(GL_TEXTURE0);

	//glBindTexture(GL_TEXTURE_2D, seg_buffer->getTexture().textureID());
	glBindTexture(GL_TEXTURE_2D, textureColorID);
	//glBindTexture(GL_TEXTURE_2D, curColorTex.textureID());
	//glBindTexture(GL_TEXTURE_2D, curDepthTex.textureID());

	V3D_GPU::renderNormalizedQuad();

	glBindTexture(GL_TEXTURE_2D, 0);

	glDisable(GL_TEXTURE_2D);

	//drawTriangleOrtho();

	glutSwapBuffers();
}

int main(int argc, char* argv[]){

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
	glutInitWindowSize(640, 480);
	glutCreateWindow ("OpenGL Test");

	glewInit();

	std::string imgColorPath = BEINGTHERE_ROOT_DIR + "resources/multi-cam/data/snapshot_rgb_3.png";
	std::string imgDepthPath = BEINGTHERE_ROOT_DIR + "resources/multi-cam/data/snapshot_depth_3.png";
	createTexture(textureColorID, false);
	//uploadToTexture(textureColorID, imgColorPath, false);

	createTexture(textureDepthID, true);
	uploadToTexture(textureDepthID, imgDepthPath, true);

	//testVideoWriter(imgColorPath);

	createVertexBuffer();
	createShader();

	readData();

	glutReshapeFunc (&ReSizeGLScene);
	glutKeyboardFunc(glutKeyboard);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glutMainLoop();

	return 0;
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

void updateMVPPersp(glm::mat4& MVP){
	Projection = glm::perspective(45.0f, 640.0f / 480.0f, 1.f, 5000.f);
	View = glm::translate(glm::mat4(1.f), glm::vec3(0.f, 0.f, -500.f));
	Model = glm::mat4(1.0f);

	MVP = Projection * View * Model;
}

void drawTriangleOrtho(){
	glClearColor(.3, .3, .3, 0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureDepthID);
	//glBindTexture(GL_TEXTURE_2D, curDepthTex.textureID());
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, curColorTex.textureID());
	//glBindTexture(GL_TEXTURE_2D, textureColorID);

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

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_TEXTURE_2D);
}