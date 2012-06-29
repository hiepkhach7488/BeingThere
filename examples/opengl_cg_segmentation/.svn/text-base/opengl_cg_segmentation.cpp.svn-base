//#include <stdio.h>   
#include <stdlib.h> 
#include <iostream>
#include <string>

#include <GL/glew.h>
#include <GL/GL.h>
#include <GL/glut.h>

#include <GL/v3d_gpuundistort.h>
#include "GL/v3d_gpubinarysegmentation.h"
#include "Base/v3d_image.h"
#include "fgbg_segmentation.h"
#include "depth_inpainting.h"
#include "Base/v3d_imageprocessing.h"
#include "Base/v3d_timer.h"
#include <float.h>

#include <Cg/cg.h>   
#include <Cg/cgGL.h>

#include "glslprogram.h"

#include <cv.h>
#include <highgui.h>

using std::string;
using namespace std;
using namespace V3D;
using namespace V3D_GPU;

GLSLProgram* identityTextureProg;
FGBG_Segmentation _fgbgSegmentation;
GPU_DepthInpainting _depthInpainting;

void setupRC(){
	//Identity Texture
	string const SHADER_ROOT_PATH = "C:/Program Files (x86)/Winzip/Format/beingthere/examples/opengl_cg_segmentation/shader/";
	//string const SHADER_ROOT_PATH = "D:/Trunk/bibleProj/beingthere/examples/opengl_cg_segmentation/shader/";
	identityTextureProg = new GLSLProgram();
	identityTextureProg->compileShaderFromFile((SHADER_ROOT_PATH+"identity_texture.frag").c_str(), GLSLShader::FRAGMENT); cout << identityTextureProg->log() << endl;
	identityTextureProg->link(); cout << identityTextureProg->log() << endl;
	identityTextureProg->validate(); cout << identityTextureProg->log() << endl;	
}

void glutIdle (void)
{
	glutPostRedisplay();
}

int choice = 0;
void glutKeyboard (unsigned char key, int x, int y)
{
	if(key==27) {
		exit(0);
	}

	if(key == 's'){
	
	}

	if(key == 'n'){
		choice = (choice + 1) % 4;
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

void readFloatImage(string img_path, Image<float>& img){
	ifstream file;
	file.open(img_path.c_str(), ios::in | ios::binary);
	float* depthBuffer = new float[640*480];
	memset(depthBuffer, 0, 640*480*sizeof(float));

	if (!file){
		cout << "No background depth file - please capture background" << endl;
	} else {
		file.read((char*) depthBuffer, 640*480*sizeof(float));	
	}
	file.close();

	for(int i=0; i<480; ++i)
		for(int j=0; j<640; ++j){
			int idx = i * 640 + j;
			img(j, i, 0) = depthBuffer[idx];
		}
}

void readUcharImage(string img_path, Image<unsigned char>& img){
	ifstream file;
	file.open(img_path.c_str(), ios::in | ios::binary);
	float* depthBuffer = new float[640*480];
	memset(depthBuffer, 0, 640*480*sizeof(float));

	if (!file){
		cout << "No background depth file - please capture background" << endl;
	} else {
		file.read((char*) depthBuffer, 640*480*sizeof(float));	
	}
	file.close();

	for(int i=0; i<480; ++i)
		for(int j=0; j<640; ++j){
			int idx = i * 640 + j;
			img(j, i, 0) = ((unsigned char)depthBuffer[idx]) >> 3;
		}
}


void readColorImage(string img_path, Image<unsigned char> &img){
	ifstream file;

	file.open(img_path.c_str(), ios::in | ios::binary);
	
	unsigned char* colorBuffer = new unsigned char[640*480*3];
	memset(colorBuffer, 0, 640*480*3*sizeof(unsigned char));

	if (!file){
		cout << "No background color file - please capture background" << endl;
	} else {
		file.read((char*) colorBuffer, 640*480*3*sizeof(unsigned char));
	}

	file.close();

	//Copy To Image Buffer
	//std::string imgColorPath = "C:/Logs/PCL/data/snapshot_rgb_3.png";
	//cv::Mat image = cv::imread(imgColorPath.c_str(), CV_LOAD_IMAGE_UNCHANGED);

	//unsigned char* rChannel = new unsigned char[640*480];
	//unsigned char* gChannel = new unsigned char[640*480];
	//unsigned char* bChannel = new unsigned char[640*480];

	//unsigned char* imgPointer = image.data;

	//for(int row=0; row<480; ++row)
	//	for(int col=0; col<640; ++col){
	//		int idx = row * 640 * 3 + 3*col;
	//		rChannel[row*640+col] = imgPointer[idx];
	//		gChannel[row*640+col] = imgPointer[idx+1];
	//		bChannel[row*640+col] = imgPointer[idx+2];
	//	}

	//memcpy(img.begin(0), rChannel, 640*480);
	//memcpy(img.begin(1), gChannel, 640*480);
	//memcpy(img.begin(2), bChannel, 640*480);

	for(int i=0; i<480; ++i)
		for(int j=0; j<640; ++j){
			int idx = i * 640 * 3 + 3*j;
			img(j, i, 0) = colorBuffer[idx];
			img(j, i, 1) = colorBuffer[idx + 1];
			img(j, i, 2) = colorBuffer[idx + 2];
		}
}

ImageTexture2D bgColorTex, curColorTex;
ImageTexture2D bgDepthTex, curDepthTex;
ImageTexture2D temDepth8Tex;
ImageTexture2D tempTexture;

void performSegmentation(){
	Cg_ProgramBase::initializeCg();

	_fgbgSegmentation.init(640, 480);
	//_fgbgSegmentation.run(bgColorTex.textureID(), bgDepthTex.textureID(), curColorTex.textureID(), curDepthTex.textureID());

	int w = 640, h = 480;
	RTT_Buffer edgeWeightBuf("rg=32f enableTextureRG", "edgeWeightBuf");
	edgeWeightBuf.allocate(w, h);

	RTT_Buffer costBuf("r=32f enableTextureRG", "costBuf");
	costBuf.allocate(w, h);

	float const alpha = 80.0f;
	_fgbgSegmentation.renderEdgeWeights(alpha, curColorTex.textureID(), edgeWeightBuf);

	Image<float> g(w, h, 3, 1.0f);
	edgeWeightBuf.getTexture().readback(g.begin(0), g.begin(1), g.begin(2));

	tempTexture.allocateID();
	tempTexture.reserve(w, h, "r=32f enableTextureRG");
	tempTexture.overwriteWith(g.begin(0), 1);

	//saveImageChannel(g, 0, "g_x.png");
	//saveImageChannel(g, 1, "g_y.png");

	float const lambda_col = 10.0f;
	float const color_bias = 0.1;
	float const lambda_depth = 1.0f;
	float const depth_threshold = 250.f;

	_fgbgSegmentation.renderSegmentationCost(lambda_col, color_bias, lambda_depth, depth_threshold,
		bgColorTex.textureID(), bgDepthTex.textureID(), curColorTex.textureID(), curDepthTex.textureID(), costBuf);

	Image<float> costs(w, h, 1, 0.0f);
	costBuf.getTexture().readback(costs.begin(0), 1);
	cout << "min(costs) = " << costs.minElement() << " max(costs) = " << costs.maxElement() << endl;
	//tempTexture.allocateID();
	//tempTexture.reserve(w, h, "r=32f enableTextureRG");
	//tempTexture.overwriteWith(costs.begin(0), 1);

	//saveImageChannel(costs, 0, "costs.png");

#if 1
	BinarySegmentationUzawa seg(false);
	//seg.setTimesteps(0.7f, 0.7f);
	//seg.setTimesteps(0.49f, 1.0f);
	seg.setTimesteps(0.249f, 2.0f);
#elif 0
	BinarySegmentationPD seg(false);
	//seg.setTimesteps(0.124f, 1.0f);
	seg.setTimesteps(0.124f*2, 1.0f/2);
#elif 0
	BinarySegmentationFwBw seg(true);
	seg.epsilon = 0.0125f; seg.tau = 0.24f;
#else
	BinarySegmentationRelaxed seg(false);
	seg.theta = 0.125; seg.tau = 0.24f;
#endif
	seg.allocate(w, h);

	Image<float> u(w, h, 1);
	for (int y = 0; y < h; ++y)
		for (int x = 0; x < w; ++x)
			u(x, y) = (costs(x, y) > 0) ? 0.0f : 1.0f;

	//saveImageChannel(u, 0, 0.0f, 1.0f, "u0.png");

	int const nIterations = 50;

	{
		ScopedTimer t("bin seg");
		for (int k = 0; k < 100; ++k)
		{
			seg.initializeIterations();
			seg.setInitialValue(u.begin()); //optional
			//seg.iterate(costTex.textureID(), edgeWeightTex.textureID(), nIterations);
			seg.iterate(costBuf.textureID(), edgeWeightBuf.textureID(), nIterations);
			glFinish();
		}
	}

	seg.getResultBuffer().activate();
	glReadPixels(0, 0, w, h, GL_RED, GL_FLOAT, u.begin(0));
	FrameBufferObject::disableFBORendering();
	//saveImageChannel(u, 0, 0.0f, 1.0f, "u_seg_pd.pgm");

	Image<unsigned char> byteIm(w, h, 1);

	for (int y = 0; y < h; ++y)
		for (int x = 0; x < w; ++x)
		{
			float c = std::max(0.f, std::min(255.f, 255.f * u(x, y, 0)));
			byteIm(x, y) = int(c);
		}

	temDepth8Tex.allocateID();
	temDepth8Tex.reserve(w, h, "r=8 enableTextureRG");
	temDepth8Tex.overwriteWith(byteIm.begin(0), 1);

	_depthInpainting.allocate(640, 480);
	_depthInpainting.initialize(1.f);
	_depthInpainting.iterate(curDepthTex.textureID(), edgeWeightBuf.textureID());
	cout << "Finish" << endl;
}

void readData(){
	string const BACKGROUND_ROOT_PATH("C:/Program Files (x86)/Winzip/Format/beingthere/examples/opengl_cg_segmentation/");
	//string const BACKGROUND_ROOT_PATH("D:/Trunk/bibleProj/beingthere/examples/opengl_cg_segmentation/");
	int w = 640, h = 480;
	Image<unsigned char> bgColor(w, h, 3), curColor(w, h, 3);;
	Image<float> bgDepth(w, h, 1), curDepth(w, h, 1);
	
	readColorImage(BACKGROUND_ROOT_PATH+"data/old/bg_color.dat", bgColor);
	readColorImage(BACKGROUND_ROOT_PATH+"data/old/cur_color.dat", curColor);

	bgColorTex.allocateID();
	bgColorTex.reserve(w, h, "rgb=8");
	bgColorTex.overwriteWith(bgColor.begin(0), bgColor.begin(1), bgColor.begin(2));

	curColorTex.allocateID();
	curColorTex.reserve(w, h, "rgb=8");
	curColorTex.overwriteWith(curColor.begin(0), curColor.begin(1), curColor.begin(2));

	readFloatImage(BACKGROUND_ROOT_PATH+"data/old/bg_depth.dat", bgDepth);
	readFloatImage(BACKGROUND_ROOT_PATH+"data/old/cur_depth.dat", curDepth);

	bgDepthTex.allocateID();
	bgDepthTex.reserve(w, h, "r=32f enableTextureRG");
	bgDepthTex.overwriteWith(bgDepth.begin(0), 1);

	curDepthTex.allocateID();
	curDepthTex.reserve(w, h, "r=32f enableTextureRG");
	curDepthTex.overwriteWith(curDepth.begin(0), 1);

	//Image<unsigned char> tempDepth(w,h,1);
	//readUcharImage(BACKGROUND_ROOT_PATH+"data/cur_depth.dat", tempDepth);
	//temDepth8Tex.allocateID();
	//temDepth8Tex.reserve(w, h, "r=8 enableTextureRG");
	//temDepth8Tex.overwriteWith(tempDepth.begin(0), 1);

	performSegmentation();
	//performSegmentation2();
}


void glutDisplay (void)
{
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);				

	glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);

	identityTextureProg->use();
	identityTextureProg->setUniform("texColor", 0);

	glEnable(GL_TEXTURE_2D);

	glActiveTexture(GL_TEXTURE0);

	switch(choice){
	case 0:
		glBindTexture(GL_TEXTURE_2D, curColorTex.textureID());
		break;
	case 1:
		glBindTexture(GL_TEXTURE_2D, curDepthTex.textureID());
		break;
	case 2:
		glBindTexture(GL_TEXTURE_2D, tempTexture.textureID());
		break;
	case 3:
		//glBindTexture(GL_TEXTURE_2D, temDepth8Tex.textureID());	
		glBindTexture(GL_TEXTURE_2D, _depthInpainting.getResult().getTexture().textureID());
		break;
	}

	//glBindTexture(GL_TEXTURE_2D, _fgbgSegmentation.getResult());

	glBegin(GL_QUADS);
		glTexCoord2f(0,1); glVertex2f(0, 0);
		glTexCoord2f(1,1); glVertex2f(1, 0);
		glTexCoord2f(1,0); glVertex2f(1, 1);
		glTexCoord2f(0,0); glVertex2f(0, 1);	
	glEnd();

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

	setupRC();
	readData();
	
	glutReshapeFunc (&ReSizeGLScene);
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glutMainLoop();
	return 0;
}

