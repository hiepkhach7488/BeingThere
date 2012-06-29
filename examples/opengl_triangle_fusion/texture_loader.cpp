#include "texture_loader.h"

TextureLoader::TextureLoader(){
	colorLoader = new ImageSequence();
	depthLoader = new ImageSequence();

	colorLoader->init(BEINGTHERE_ROOT_DIR+ "resources/multi-cam/data/", ".png", "snapshot_rgb_");
	colorLoader->open();
	colorLoader->captureStart();
	colorLoader->captureNext();

	depthLoader->init(BEINGTHERE_ROOT_DIR+ "resources/multi-cam/data/", ".png", "snapshot_depth_");
	depthLoader->open();
	depthLoader->captureStart();
	depthLoader->captureNext();

	//Shader Program
	progMedianSm = new GLSLProgram();
	progMedianLg = new GLSLProgram();
	progBilateral = new GLSLProgram();
	
	glGenFramebuffers(1, &fbo);
}

TextureLoader::~TextureLoader(){

}

bool TextureLoader::createShader(){
	std::string const SHADER_ROOT_PATH = BEINGTHERE_EXAMPLE_ROOT_DIR + "opengl_triangle_fusion/shader/";

	progMedianSm->compileShaderFromFile((SHADER_ROOT_PATH+"median9.frag").c_str(), GLSLShader::FRAGMENT); cout << progMedianSm->log() << endl;
	progMedianSm->link(); cout << progMedianSm->log() << endl;
	progMedianSm->validate(); cout << progMedianSm->log() << endl;

	progMedianLg->compileShaderFromFile((SHADER_ROOT_PATH+"median25.frag").c_str(), GLSLShader::FRAGMENT); cout << progMedianLg->log() << endl;
	progMedianLg->link(); cout << progMedianLg->log() << endl;
	progMedianLg->validate(); cout << progMedianLg->log() << endl;

	progBilateral->compileShaderFromFile((SHADER_ROOT_PATH+"bilateral.frag").c_str(), GLSLShader::FRAGMENT); cout << progBilateral->log() << endl;
	progBilateral->link(); cout << progBilateral->log() << endl;
	progBilateral->validate(); cout << progBilateral->log() << endl;

	return true;
}

bool TextureLoader::processTextures(){
	for(int i=0; i<NUM_CAM; ++i){
		processDepthTexture(depthTex[i], processedDepthTex[i], lastDepthTex[i]);
		copyTexture(processedDepthTex[i], lastDepthTex[i]);
	}

	return true;
}

void TextureLoader::renderNormalizedQuad(){
	//Push All Setting To Stack
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);
	glPushAttrib(GL_VIEWPORT_BIT);

	glMatrixMode (GL_PROJECTION);
	glPushMatrix();	
	glLoadIdentity ();
	glOrtho (0, 1, 0, 1, -1, 1);
	glMatrixMode (GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glBegin(GL_QUADS);
		glTexCoord2f(0,1); glVertex2f(0, 1);
		glTexCoord2f(1,1); glVertex2f(1, 1);
		glTexCoord2f(1,0); glVertex2f(1, 0);
		glTexCoord2f(0,0); glVertex2f(0, 0);	
	glEnd();

	//Reset All Variables
	glDisable(GL_TEXTURE_2D);
	glEnable(GL_DEPTH_TEST);
	glPopAttrib();
	
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

void TextureLoader::copyTexture(GLuint& const inTex, GLuint& const outTex){
	//Copy Current To Last Depth Tex
	glUseProgram(0);

	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, outTex, 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, inTex);

	renderNormalizedQuad();

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

bool TextureLoader::processDepthTexture(GLuint& const depthTex, GLuint& const processedDepthTex, GLuint& lastDepthTex){
	//Fill Large Holes
	progMedianLg->use(); cout << progMedianLg->log() << endl;

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, depthTex);
	progMedianLg->setUniform("src", 0);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, processedDepthTex, 0);

	renderNormalizedQuad();

	//Fill Small Hole
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, processedDepthTex);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tempDepthTex, 0);
	progMedianSm->use(); cout << progMedianSm->log() << endl;
	progMedianSm->setUniform("src", 0);
	progMedianSm->setUniform("pass", 0);

	renderNormalizedQuad();

	//bilateral filter
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, processedDepthTex, 0);
	
	/*glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, colorTex[devId]);*/
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, lastDepthTex);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, tempDepthTex);

	progBilateral->use(); cout << progBilateral->log() << endl;
	progBilateral->setUniform("depth", 0);
	progBilateral->setUniform("lastDepth", 1);
	//progBilateral->setUniform("color", 2);
	int currentFrame = 0;
	progBilateral->setUniform("init", currentFrame%150==0);

	renderNormalizedQuad();

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	return true;
}

bool TextureLoader::uploadToTexture(){
	for(int i=0; i<NUM_CAM; ++i){
		//Color Tex
		glBindTexture(GL_TEXTURE_2D, colorTex[i]);
			//glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
			/*glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 640, 480, 0,
			GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*) img.data);*/
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_RGB, GL_UNSIGNED_BYTE, colorMat[i].data);
		glBindTexture(GL_TEXTURE_2D, 0);

		//Depth Tex
		glBindTexture(GL_TEXTURE_2D, depthTex[i]);
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_GREEN, GL_UNSIGNED_SHORT, depthMat[i].data);
		glBindTexture(GL_TEXTURE_2D, 0);
	}

	return true;
}

bool TextureLoader::initColorTex(GLuint& const colorTex){
	int xRes = 640; 
	int yRes = 480;

	unsigned char * dummyTexColor = (unsigned char *)malloc(xRes*yRes*3);
	memset(dummyTexColor, 0, xRes*yRes*3);

	//Color Texture
	glBindTexture(GL_TEXTURE_2D, colorTex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, dummyTexColor);

	glBindTexture(GL_TEXTURE_2D, 0);

	free(dummyTexColor);

	return true;
}

bool TextureLoader::initDepthTex(GLuint& const depthTex){
	int xRes = 640; 
	int yRes = 480;

	unsigned char* dummyTexDepth = (unsigned char *) malloc(xRes*yRes*2);
	memset(dummyTexDepth, 0, xRes*yRes*2);

	//Depth Texture
	glBindTexture(GL_TEXTURE_2D, depthTex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16, 640, 480, 0, GL_GREEN, GL_UNSIGNED_SHORT, dummyTexDepth);

	glBindTexture(GL_TEXTURE_2D, 0);

	free(dummyTexDepth);

	return true;
}

bool TextureLoader::initTexture(){
	glGenTextures(NUM_CAM, colorTex);
	glGenTextures(NUM_CAM, depthTex);
	
	glGenTextures(NUM_CAM, processedColorTex);
	glGenTextures(NUM_CAM, processedDepthTex);
	
	glGenTextures(1, &tempColorTex);
	glGenTextures(1, &tempDepthTex);
	initColorTex(tempColorTex);
	initDepthTex(tempDepthTex);

	for(int i=0; i<NUM_CAM; ++i){
		initColorTex(colorTex[i]);
		initColorTex(processedColorTex[i]);
		initColorTex(lastColorTex[i]);

		initDepthTex(depthTex[i]);
		initDepthTex(processedDepthTex[i]);
		initDepthTex(lastDepthTex[i]);
	}

	return true;
}

bool TextureLoader::loadImageData(){
	IplImage* temp = 0; 
	for(int i=0; i<NUM_CAM; ++i){
		temp = colorLoader->image();
		assert(temp != 0);
		colorMat[i] = cv::Mat(temp);

		temp = depthLoader->image();
		assert(temp!=0);
		depthMat[i] = cv::Mat(temp);

		colorLoader->captureNext();
		depthLoader->captureNext();
	}

	return true;
}

