#include "v3d_shaders.h"

V3dShader::V3dShader(){

}

V3dShader::~V3dShader(){

}

void V3dShader::createShader(){
	string const SHADER_ROOT_PATH = BEINGTHERE_EXAMPLE_ROOT_DIR + "opengl_v3d_test/shader/";
	
	colorMaskProg = new GLSLProgram();
	colorMaskProg->compileShaderFromFile((SHADER_ROOT_PATH+"vertex.vs").c_str(), GLSLShader::VERTEX); cout << colorMaskProg->log() << endl;
	colorMaskProg->compileShaderFromFile((SHADER_ROOT_PATH+"mask_color.fs").c_str(), GLSLShader::FRAGMENT); cout << colorMaskProg->log() << endl;
	colorMaskProg->link(); cout << colorMaskProg->log() << endl;
	colorMaskProg->validate(); cout << colorMaskProg->log() << endl;	

	depthMaskProg = new GLSLProgram();
	depthMaskProg->compileShaderFromFile((SHADER_ROOT_PATH+"vertex.vs").c_str(), GLSLShader::VERTEX); cout << depthMaskProg->log() << endl;
	depthMaskProg->compileShaderFromFile((SHADER_ROOT_PATH+"mask_depth.fs").c_str(), GLSLShader::FRAGMENT); cout << depthMaskProg->log() << endl;
	depthMaskProg->link(); cout << depthMaskProg->log() << endl;
	depthMaskProg->validate(); cout << depthMaskProg->log() << endl;	

	borderMaskProg = new GLSLProgram();
	borderMaskProg->compileShaderFromFile((SHADER_ROOT_PATH+"vertex.vs").c_str(), GLSLShader::VERTEX); cout << borderMaskProg->log() << endl;
	borderMaskProg->compileShaderFromFile((SHADER_ROOT_PATH+"mask_color_border.fs").c_str(), GLSLShader::FRAGMENT); cout << borderMaskProg->log() << endl;
	borderMaskProg->link(); cout << borderMaskProg->log() << endl;
	borderMaskProg->validate(); cout << borderMaskProg->log() << endl;	

	floatToUshortProg = new GLSLProgram();
	floatToUshortProg->compileShaderFromFile((SHADER_ROOT_PATH+"vertex.vs").c_str(), GLSLShader::VERTEX); cout << floatToUshortProg->log() << endl;
	floatToUshortProg->compileShaderFromFile((SHADER_ROOT_PATH+"convert_float_to_ushort.fs").c_str(), GLSLShader::FRAGMENT); cout << floatToUshortProg->log() << endl;
	floatToUshortProg->link(); cout << floatToUshortProg->log() << endl;
	floatToUshortProg->validate(); cout << floatToUshortProg->log() << endl;	

	ushortToFloatProg = new GLSLProgram();
	ushortToFloatProg->compileShaderFromFile((SHADER_ROOT_PATH+"vertex.vs").c_str(), GLSLShader::VERTEX); cout << ushortToFloatProg->log() << endl;
	ushortToFloatProg->compileShaderFromFile((SHADER_ROOT_PATH+"convert_ushort_to_float.fs").c_str(), GLSLShader::FRAGMENT); cout << ushortToFloatProg->log() << endl;
	ushortToFloatProg->link(); cout << ushortToFloatProg->log() << endl;
	ushortToFloatProg->validate(); cout << ushortToFloatProg->log() << endl;	

	depthToRGBProg = new GLSLProgram();
	depthToRGBProg->compileShaderFromFile((SHADER_ROOT_PATH+"vertex.vs").c_str(), GLSLShader::VERTEX); cout << depthToRGBProg->log() << endl;
	depthToRGBProg->compileShaderFromFile((SHADER_ROOT_PATH+"depth_to_rgb.fs").c_str(), GLSLShader::FRAGMENT); cout << depthToRGBProg->log() << endl;
	depthToRGBProg->link(); cout << depthToRGBProg->log() << endl;
	depthToRGBProg->validate(); cout << depthToRGBProg->log() << endl;	

	ushortDepthToRGBProg = new GLSLProgram();
	ushortDepthToRGBProg->compileShaderFromFile((SHADER_ROOT_PATH+"vertex.vs").c_str(), GLSLShader::VERTEX); cout << ushortDepthToRGBProg->log() << endl;
	ushortDepthToRGBProg->compileShaderFromFile((SHADER_ROOT_PATH+"ushort_depth_to_rgb.fs").c_str(), GLSLShader::FRAGMENT); cout << ushortDepthToRGBProg->log() << endl;
	ushortDepthToRGBProg->link(); cout << ushortDepthToRGBProg->log() << endl;
	ushortDepthToRGBProg->validate(); cout << ushortDepthToRGBProg->log() << endl;	

	depthTruncationProg = new GLSLProgram();
	depthTruncationProg->compileShaderFromFile((SHADER_ROOT_PATH+"vertex.vs").c_str(), GLSLShader::VERTEX); cout << depthTruncationProg->log() << endl;
	depthTruncationProg->compileShaderFromFile((SHADER_ROOT_PATH+"truncate_depth_float.fs").c_str(), GLSLShader::FRAGMENT); cout << depthTruncationProg->log() << endl;
	depthTruncationProg->link(); cout << depthTruncationProg->log() << endl;
	depthTruncationProg->validate(); cout << depthTruncationProg->log() << endl;

	rgbTruncationProg = new GLSLProgram();
	rgbTruncationProg->compileShaderFromFile((SHADER_ROOT_PATH+"vertex.vs").c_str(), GLSLShader::VERTEX); cout << rgbTruncationProg->log() << endl;
	rgbTruncationProg->compileShaderFromFile((SHADER_ROOT_PATH+"truncate_rgb_from_depth.fs").c_str(), GLSLShader::FRAGMENT); cout << rgbTruncationProg->log() << endl;
	rgbTruncationProg->link(); cout << rgbTruncationProg->log() << endl;
	rgbTruncationProg->validate(); cout << rgbTruncationProg->log() << endl;
}

void V3dShader::initTextureFBO(){
	glGenFramebuffers(1, &colorFBO);

	glGenTextures(1, &tempColorTex);
	glBindTexture(GL_TEXTURE_2D,tempColorTex);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

	glGenFramebuffers(1, &depthFBO);

	glGenTextures(1, &tempDepthTex);
	glBindTexture(GL_TEXTURE_2D, tempDepthTex); 
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, 640, 480 , 0, GL_RED, GL_FLOAT, 0);

	glGenTextures(1, &tempDepthUshortTex);
	glBindTexture(GL_TEXTURE_2D, tempDepthUshortTex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16, 640, 480, 0, GL_GREEN, GL_UNSIGNED_SHORT, 0);
}

void V3dShader::drawTriangle() {
	glBegin (GL_TRIANGLES); 
	glVertex3i(-1, 3, 0);
	glVertex3i(-1, -1, 0);
	glVertex3i(3, -1, 0);
	glEnd ();
}

//dst - 16 bit GREEN texture
void V3dShader::convertFloatToUshort(GLuint srcTex, GLuint dstTex){
	glBindFramebuffer(GL_FRAMEBUFFER, depthFBO);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tempDepthUshortTex, 0);
	
	glPushAttrib(GL_VIEWPORT_BIT);
	glViewport(0, 0, 640, 480);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

	glEnable(GL_TEXTURE_2D);

	glUseProgram(floatToUshortProg->getHandle());
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, srcTex);
	glUniform1i(glGetUniformLocation(floatToUshortProg->getHandle(), "inputTexture"), 0);
	
	drawTriangle();

	glUseProgram(0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, 0);

	glPopAttrib();
	glDisable(GL_TEXTURE_2D);

	beingthere::copyTexture(tempDepthUshortTex, dstTex);
}

void V3dShader::convertUshortToFloat(GLuint srcTex, GLuint dstTex){
	glBindFramebuffer(GL_FRAMEBUFFER, depthFBO);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tempDepthTex, 0);
	
	glPushAttrib(GL_VIEWPORT_BIT);
	glViewport(0, 0, 640, 480);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

	glEnable(GL_TEXTURE_2D);

	glUseProgram(ushortToFloatProg->getHandle());
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, srcTex);
	glUniform1i(glGetUniformLocation(ushortToFloatProg->getHandle(), "inputTexture"), 0);
	
	drawTriangle();

	glUseProgram(0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, 0);

	glPopAttrib();
	glDisable(GL_TEXTURE_2D);
	
	beingthere::copyTexture(tempDepthTex, dstTex);
}


void V3dShader::truncateDepth(GLuint srcTex, GLuint dstTex, float depth_threshold){
	glBindFramebuffer(GL_FRAMEBUFFER, depthFBO);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tempDepthTex, 0);

	glPushAttrib(GL_VIEWPORT_BIT);
	glViewport(0, 0, 640, 480);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

	glEnable(GL_TEXTURE_2D);

	glUseProgram(depthTruncationProg->getHandle());

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, srcTex);
	glUniform1i(glGetUniformLocation(depthTruncationProg->getHandle(), "inputTexture"), 0);
	depthTruncationProg->setUniform("depthThreshold", depth_threshold);

	drawTriangle();

	glUseProgram(0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, 0);

	glPopAttrib();
	glDisable(GL_TEXTURE_2D);

	beingthere::copyTexture(tempDepthTex, dstTex);
}

void V3dShader::truncateRGBFromDepth(GLuint rgbTex, GLuint depthTex, GLuint dstTex, float depth_threshold){
	glBindFramebuffer(GL_FRAMEBUFFER, colorFBO);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tempColorTex, 0);
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		

	glPushAttrib(GL_VIEWPORT_BIT);
	glViewport(0, 0, 640, 480);

	glEnable(GL_TEXTURE_2D);

	rgbTruncationProg->use();

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, rgbTex);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, depthTex);

	glUniform1i(glGetUniformLocation(rgbTruncationProg->getHandle(), "colorTex"), 0);
	glUniform1i(glGetUniformLocation(rgbTruncationProg->getHandle(), "depthTex"), 1);

	glUniform1i(glGetUniformLocation(rgbTruncationProg->getHandle(), "w"), 640);
	glUniform1i(glGetUniformLocation(rgbTruncationProg->getHandle(), "h"), 480);

	rgbTruncationProg->setUniform("depthThreshold", depth_threshold);

	drawTriangle();

	glUseProgram(0);

	glPopAttrib();
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, 0);

	glDisable(GL_TEXTURE_2D);

	beingthere::copyTexture(tempColorTex, dstTex);
}

void V3dShader::convertDepthToRGB(GLuint srcTex, GLuint dstTex, int min_depth, int max_depth){
	glBindFramebuffer(GL_FRAMEBUFFER, depthFBO);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tempDepthTex, 0);
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		

	glPushAttrib(GL_VIEWPORT_BIT);
	glViewport(0, 0, 640, 480);

	glEnable(GL_TEXTURE_2D);

	depthToRGBProg->use();

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, srcTex);

	glUniform1i(glGetUniformLocation(depthToRGBProg->getHandle(), "warpedDepthTexture"), 0);
	glUniform1i(glGetUniformLocation(depthToRGBProg->getHandle(), "w"), 640);
	glUniform1i(glGetUniformLocation(depthToRGBProg->getHandle(), "h"), 480);
	glUniform1f(glGetUniformLocation(depthToRGBProg->getHandle(), "minDepth"), min_depth);
	glUniform1f(glGetUniformLocation(depthToRGBProg->getHandle(), "maxDepth"), max_depth);

	drawTriangle();

	glUseProgram(0);

	glPopAttrib();
	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);

	beingthere::copyTexture(tempDepthTex, dstTex);
}

void V3dShader::convertUshortDepthToRGB(GLuint srcTex, GLuint dstTex){
	glBindFramebuffer(GL_FRAMEBUFFER, colorFBO);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tempColorTex, 0);

	glPushAttrib(GL_VIEWPORT_BIT);
	glViewport(0, 0, 640, 480);
	ushortDepthToRGBProg->use(); cout << ushortDepthToRGBProg->log() << endl;

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, srcTex);

	glUniform1i(glGetUniformLocation(ushortDepthToRGBProg->getHandle(), "inputTex"), 0);

	drawTriangle();

	glUseProgram(0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glPopAttrib();

	glActiveTexture(GL_TEXTURE0);
	beingthere::copyTexture(tempColorTex, dstTex);
}

void V3dShader::maskColor(GLuint srcTex, GLuint dstTex, GLuint maskTex){
	glBindFramebuffer(GL_FRAMEBUFFER, colorFBO);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tempColorTex, 0);

	glPushAttrib(GL_VIEWPORT_BIT);
	glViewport(0, 0, 640, 480);
	colorMaskProg->use(); cout << colorMaskProg->log() << endl;

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, srcTex);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, maskTex);

	glUniform1i(glGetUniformLocation(colorMaskProg->getHandle(), "colorTexture"), 0);
	glUniform1i(glGetUniformLocation(colorMaskProg->getHandle(), "maskTexture"), 1);
	glUniform1i(glGetUniformLocation(colorMaskProg->getHandle(), "w"), 640);
	glUniform1i(glGetUniformLocation(colorMaskProg->getHandle(), "h"), 480);

	drawTriangle();

	glUseProgram(0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glPopAttrib();

	glActiveTexture(GL_TEXTURE0);
	beingthere::copyTexture(tempColorTex, dstTex);
}

void V3dShader::maskDepth(GLuint srcTex, GLuint dstTex, GLuint maskTex){
	glBindFramebuffer(GL_FRAMEBUFFER, depthFBO);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tempDepthTex, 0);

	glPushAttrib(GL_VIEWPORT_BIT);
	glViewport(0, 0, 640, 480);
	
	depthMaskProg->use();

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, srcTex);
	
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, maskTex);

	glUniform1i(glGetUniformLocation(depthMaskProg->getHandle(), "depthTexture"), 0);
	glUniform1i(glGetUniformLocation(depthMaskProg->getHandle(), "maskTexture"), 1);
	glUniform1i(glGetUniformLocation(depthMaskProg->getHandle(), "w"), 640);
	glUniform1i(glGetUniformLocation(depthMaskProg->getHandle(), "h"), 480);

	drawTriangle();

	glUseProgram(0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glPopAttrib();

	glActiveTexture(GL_TEXTURE0);
	beingthere::copyTexture(tempDepthTex, dstTex);
}


void V3dShader::maskColorFromDepth(GLuint srcTex, GLuint dstTex, GLuint depthTex){
	glBindFramebuffer(GL_FRAMEBUFFER, colorFBO);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tempColorTex, 0);

	glPushAttrib(GL_VIEWPORT_BIT);
	glViewport(0, 0, 640, 480);
	borderMaskProg->use(); cout << borderMaskProg->log() << endl;

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, srcTex);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, depthTex);

	glUniform1i(glGetUniformLocation(borderMaskProg->getHandle(), "colorTexture"), 0);
	glUniform1i(glGetUniformLocation(borderMaskProg->getHandle(), "depthTexture"), 1);
	glUniform1i(glGetUniformLocation(borderMaskProg->getHandle(), "w"), 640);
	glUniform1i(glGetUniformLocation(borderMaskProg->getHandle(), "h"), 480);

	drawTriangle();

	glUseProgram(0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glPopAttrib();

	glActiveTexture(GL_TEXTURE0);
	beingthere::copyTexture(tempColorTex, dstTex);
}

