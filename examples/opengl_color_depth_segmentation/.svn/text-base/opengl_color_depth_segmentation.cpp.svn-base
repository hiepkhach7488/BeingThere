#include <GL/glew.h>
#include <GL/GL.h>
#include <GL/glut.h>

#include "Base/v3d_imageprocessing.h"
#include "Base/v3d_timer.h"
#include "GL/v3d_gpubase.h"
#include "GL/v3d_gpubinarysegmentation.h"
#include <Cg/cg.h>   
#include <Cg/cgGL.h>

#include "glslprogram.h"
#include "v3d_common.h"
#include "beingthere_config.h"
#include "glm/glm.hpp"
#include <glm/gtc/matrix_transform.hpp>

using namespace V3D;
using namespace beingthere;

GLuint textureColorID;
GLuint textureDepthID;
GLuint tempTextureDepthID;

V3D_GPU::ImageTexture2D bgColorTex, curColorTex;
V3D_GPU::ImageTexture2D bgDepthTex, curDepthTex;

int w = 640, h = 480;
Image<float> bgDepth(w, h, 1), curDepth(w, h, 1);
Image<uchar> bgColor(w, h, 3), curColor(w, h, 3);;
Image<ushort> bgShortDepth(w,h,1), curShortDepth(w,h,1);

GLSLProgram* depthRGBProg = 0;
GLSLProgram* copyConvertProg = 0;
void drawTexture();

void createShader(){
	string const SHADER_ROOT_PATH = BEINGTHERE_EXAMPLE_ROOT_DIR + "opengl_color_depth_segmentation/shader/";

	depthRGBProg = new GLSLProgram();
	depthRGBProg->compileShaderFromFile((SHADER_ROOT_PATH+"vertex.vs").c_str(), GLSLShader::VERTEX); cout << depthRGBProg->log() << endl;
	depthRGBProg->compileShaderFromFile((SHADER_ROOT_PATH+"depth_to_rgb.fs").c_str(), GLSLShader::FRAGMENT); cout << depthRGBProg->log() << endl;
	depthRGBProg->link(); cout << depthRGBProg->log() << endl;
	depthRGBProg->validate(); cout << depthRGBProg->log() << endl;

	copyConvertProg = new GLSLProgram();
	copyConvertProg->compileShaderFromFile((SHADER_ROOT_PATH+"vertex.vs").c_str(), GLSLShader::VERTEX); cout << copyConvertProg->log() << endl;
	copyConvertProg->compileShaderFromFile((SHADER_ROOT_PATH+"convert_texture.fs").c_str(), GLSLShader::FRAGMENT); cout << copyConvertProg->log() << endl;
	copyConvertProg->link(); cout << copyConvertProg->log() << endl;
	copyConvertProg->validate(); cout << copyConvertProg->log() << endl;	
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

void readData(){
	string const DATA_ROOT_PATH = BEINGTHERE_EXAMPLE_ROOT_DIR + "opengl_cg_segmentation/data/";

	readImage<uchar, uchar>(DATA_ROOT_PATH + "bg_color.dat", bgColor, 3);
	readImage<uchar, uchar>(DATA_ROOT_PATH + "cur_color.dat", curColor, 3);

	readImage<float, float>(DATA_ROOT_PATH + "bg_depth.dat", bgDepth, 1);
	readImage<float, float>(DATA_ROOT_PATH + "cur_depth.dat", curDepth, 1);

	bgColorTex.allocateID();
	bgColorTex.reserve(640, 480, "rgb=8");
	bgColorTex.overwriteWith(bgColor.begin(0), bgColor.begin(1), bgColor.begin(2));

	curColorTex.allocateID();
	curColorTex.reserve(640, 480, "rgb=8");
	curColorTex.overwriteWith(curColor.begin(0), curColor.begin(1), curColor.begin(2));

	bgDepthTex.allocateID();
	bgDepthTex.reserve(w, h, "r=32f enableTextureRG");
	bgDepthTex.overwriteWith(bgDepth.begin(0), 1);

	curDepthTex.allocateID();
	curDepthTex.reserve(w, h, "r=32f enableTextureRG");
	curDepthTex.overwriteWith(curDepth.begin(0), 1);

	readImage<ushort, float>(DATA_ROOT_PATH + "bg_depth.dat", bgShortDepth, 1);
	readImage<ushort, float>(DATA_ROOT_PATH + "cur_depth.dat", curShortDepth, 1);

	glGenTextures(1, &tempTextureDepthID);
	createTexture(tempTextureDepthID, true);

	glBindTexture(GL_TEXTURE_2D, tempTextureDepthID);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_GREEN, GL_UNSIGNED_SHORT, curShortDepth.begin(0));
	glBindTexture(GL_TEXTURE_2D, 0);

	drawTexture();

}

void printData(){
	float* depth_data = (float*) malloc(640*480*sizeof(float));
	glGetTextureImageEXT(bgDepthTex.textureID(), GL_TEXTURE_2D, 0, GL_RED, GL_FLOAT, depth_data);

	for (int y = 0; y < 480; ++y)
		for (int x = 0; x < 640; ++x){
			int idx = y * 640 + x;
			if(abs(bgDepth(x,y) - depth_data[idx]) > 5) cout << "Error" << endl;
		}

	ushort* depth_data_short = (ushort*) malloc(640*480*sizeof(ushort));
	glGetTextureImageEXT(tempTextureDepthID, GL_TEXTURE_2D, 0, GL_GREEN, GL_UNSIGNED_SHORT, depth_data_short);

	for (int y = 0; y < 480; ++y)
		for (int x = 0; x < 640; ++x){
			int idx = y * 640 + x;
			if(abs(curShortDepth(x,y) - depth_data_short[idx]) > 5) cout << "Error" << endl;
		}
}

void drawTriangle() {
	glBegin (GL_TRIANGLES); 
	glVertex3i(-1, 3, 0);
	glVertex3i(-1, -1, 0);
	glVertex3i(3, -1, 0);
	glEnd ();
}

void drawDepthToRGB(){
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		
	V3D_GPU::setupNormalizedProjection(true);

	glEnable(GL_TEXTURE_2D);

	depthRGBProg->use();

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, curDepthTex.textureID());

	float minDepth = 700;
	float maxDepth = 2700;

	glUniform1i(glGetUniformLocation(depthRGBProg->getHandle(), "warpedDepthTexture"), 0);
	glUniform1i(glGetUniformLocation(depthRGBProg->getHandle(), "w"), 640);
	glUniform1i(glGetUniformLocation(depthRGBProg->getHandle(), "h"), 480);
	glUniform1f(glGetUniformLocation(depthRGBProg->getHandle(), "minDepth"), minDepth);
	glUniform1f(glGetUniformLocation(depthRGBProg->getHandle(), "maxDepth"), maxDepth);

	drawTriangle();
	glUseProgram(0);

	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);
}

void drawTexture(){
	GLuint depthFBO;
	glGenFramebuffers(1, &depthFBO);
	glBindFramebuffer(GL_FRAMEBUFFER, depthFBO);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tempTextureDepthID, 0);

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		
	V3D_GPU::setupNormalizedProjection(true);

	glEnable(GL_TEXTURE_2D);

	glUseProgram(copyConvertProg->getHandle());
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, curDepthTex.textureID());
	glUniform1i(glGetUniformLocation(copyConvertProg->getHandle(), "inputTexture"), 0);
	
	drawTriangle();

	glDisable(GL_TEXTURE_2D);

	glUseProgram(0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	ushort* depth_data_short = (ushort*) malloc(640*480*sizeof(ushort));
	glGetTextureImageEXT(tempTextureDepthID, GL_TEXTURE_2D, 0, GL_GREEN, GL_UNSIGNED_SHORT, depth_data_short);

	for (int y = 0; y < 480; ++y)
		for (int x = 0; x < 640; ++x){
			int idx = y * 640 + x;
			//if(idx % 300 == 0) cout << depth_data_short[idx] << endl;
			//if(abs(curDepth(x,y) - depth_data_short[idx]) > 0.05) cout << "Error" << endl;
		}
}

void glutDisplay (void)
{
	//drawTexture();
	//drawDepthToRGB();
	
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	
	glEnable(GL_TEXTURE_2D);

	glBindTexture(GL_TEXTURE_2D, tempTextureDepthID );

	V3D_GPU::renderNormalizedQuad();

	glBindTexture(GL_TEXTURE_2D, 0);

	glDisable(GL_TEXTURE_2D);

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
	//uploadToTexture(textureDepthID, imgDepthPath, true);

	createShader();
	readData();
	//printData();

	glutReshapeFunc (&ReSizeGLScene);
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glutMainLoop();

	return 0;
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

