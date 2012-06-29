#include "triangle_fusion.h"

TriangleFusion::TriangleFusion(){
	calibLoader = new CalibrationLoader();
	glGenFramebuffers(1, &fbo);
}

TriangleFusion::~TriangleFusion(){

}

bool TriangleFusion::loadCalibrationData(){
	calibLoader->loadAllCalibrationFiles();
	calibLoader->calculateGlobalCalib();

	for(int i=0; i<NUM_CAM; ++i){
		intrinsic[i] = calibLoader->intrisic[i];
		g_extrinsicR[i] = calibLoader->g_extrinsicR[i];
		g_extrinsicT[i] = calibLoader->g_extrinsicT[i];
		//calibLoader->printMat(g_extrinsicR[i]);
	}

	return true;
}

void TriangleFusion::renderFromSingle(int idx, GLuint colorTex, GLuint depthTex, glm::mat4& const MVP){
	glEnable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, depthTex);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, colorTex);

	float max_triangle_z_offset = 2.f;

	shaderProgram->use();
	shaderProgram->setUniform("texColor", 1);
	shaderProgram->setUniform("texDepth", 0);
	shaderProgram->setUniform("MVP", MVP);
	shaderProgram->setUniform("max_triangle_z_offset", max_triangle_z_offset);

	//Camera Information
	shaderProgram->setUniform("fx", intrinsic[idx].fx);
	shaderProgram->setUniform("fy", intrinsic[idx].fy);
	shaderProgram->setUniform("cx", intrinsic[idx].cx);
	shaderProgram->setUniform("cy", intrinsic[idx].cy);

	//Drawing
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, triangleTemplateBuffer);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, triangleTemplateVertices);

	glUseProgram(0);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, 0);	
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, 0);
	
	glDisable(GL_TEXTURE_2D);
}

void TriangleFusion::renderFromAll(GLuint* colorTex, GLuint* depthTex, glm::mat4& const MVP){
	for(int i=0; i<NUM_CAM; ++i){
		//Setup MVP Matrix
		glm::mat4 ModelMatCM(1.f);
		for(int j=0; j<3; ++j){
			ModelMatCM[0][j] = g_extrinsicR[i].data[j].x;
			ModelMatCM[1][j] = g_extrinsicR[i].data[j].y;
			ModelMatCM[2][j] = g_extrinsicR[i].data[j].z;			
		}

		ModelMatCM[3][0] = g_extrinsicT[i].x * 100.f;
		ModelMatCM[3][1] = g_extrinsicT[i].y * 100.f;
		ModelMatCM[3][2] = g_extrinsicT[i].z * 100.f;

		glm::mat4 G_MVPMat = MVP * ModelMatCM;
		renderFromSingle(i, colorTex[i], depthTex[i], G_MVPMat);
	}
}

void TriangleFusion::renderToTempTexture(GLuint* colorTex, GLuint* depthTex, glm::mat4& const MVP){
	for(int i=0; i<NUM_CAM; ++i){
		//Setup MVP Matrix
		glm::mat4 ModelMatCM(1.f);
		for(int j=0; j<3; ++j){
			ModelMatCM[0][j] = g_extrinsicR[i].data[j].x;
			ModelMatCM[1][j] = g_extrinsicR[i].data[j].y;
			ModelMatCM[2][j] = g_extrinsicR[i].data[j].z;			
		}

		ModelMatCM[3][0] = g_extrinsicT[i].x * 100.f;
		ModelMatCM[3][1] = g_extrinsicT[i].y * 100.f;
		ModelMatCM[3][2] = g_extrinsicT[i].z * 100.f;

		glm::mat4 G_MVPMat = MVP * ModelMatCM;

		//Render To Temp Texture
		glPushAttrib(GL_VIEWPORT_BIT);
		glViewport(0,0,640,480);

		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tempColorTexture[i], 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, tempDepthTexture[i], 0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		renderFromSingle(i, colorTex[i], depthTex[i], G_MVPMat);

		glPopAttrib();
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
	}
}

void TriangleFusion::renderQualityTexture(){
	//Merging And Pos-Processing
	progQuality->use(); cout << progQuality->log() << endl;

	int enabled[NUM_CAM] = {0};

	GLint depthLocArray[NUM_CAM]; 
	GLint colorQualLocArray[NUM_CAM];

	for(int i = 0; i < NUM_CAM; i++) {
		//load color/quality texture
		glActiveTexture(GL_TEXTURE0+2*i);
		glBindTexture(GL_TEXTURE_2D, tempColorTexture[i]);
		colorQualLocArray[i] = 2*i;

		//load depth texture
		glActiveTexture(GL_TEXTURE0+2*i+1);
		glBindTexture(GL_TEXTURE_2D, tempDepthTexture[i]);
		depthLocArray[i] = 2*i+1;

		enabled[i] = 1;
	}

	int camEnabled_location = glGetUniformLocation(progQuality->getHandle(), "camEnabled");
	int depth_location = glGetUniformLocation(progQuality->getHandle(), "texDepth");
	int colorQual_location = glGetUniformLocation(progQuality->getHandle(), "texColorQuality");

	glUniform1iv(camEnabled_location, NUM_CAM, enabled);
	glUniform1iv(colorQual_location, NUM_CAM, colorQualLocArray);
	glUniform1iv(depth_location, NUM_CAM, depthLocArray);
	
	float colorMatchLinear[NUM_CAM*3];
	float colorMatchConstant[NUM_CAM*3];
	for(int i = 0; i < NUM_CAM; i++) {
		for(int j = 0; j < 3; j++) {
			int offset = 3*i+j;
			colorMatchLinear[offset] = 1;
			colorMatchConstant[offset] = 0;
		}		
	}

	int colorMatchLinear_location = glGetUniformLocation(progQuality->getHandle(), "colorMatchLinear");
	int colorMatchConstant_location = glGetUniformLocation(progQuality->getHandle(), "colorMatchConstant");

	glUniform3fv(colorMatchLinear_location, NUM_CAM, colorMatchLinear);
	glUniform3fv(colorMatchConstant_location, NUM_CAM, colorMatchConstant);

	#define Z_NEAR_CM_TRACKED 10
	#define Z_NEAR_CM 1000
	#define Z_FAR_CM 5000
	#define FRONT_SURFACE_TOL_CM_PER_M 6.5

	float zNear = Z_NEAR_CM;
	float zFar = Z_FAR_CM;
	float frontSurfaceTolPerMeter = FRONT_SURFACE_TOL_CM_PER_M;

	progQuality->setUniform("zNear", zNear);
	progQuality->setUniform("zFar", zFar);
	progQuality->setUniform("photometric_merge", 1);
	progQuality->setUniform("closestDepthTolerancePerMeter", frontSurfaceTolPerMeter);

	//Render To Output Texture
	//glPushAttrib(GL_VIEWPORT_BIT);
	//glViewport(0,0,640,480);
	//glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	//glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, outColorTexture, 0);
	//glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, outDepthTexture, 0);

	renderNormalizedQuad();

	//glPopAttrib();
	//glBindFramebuffer(GL_FRAMEBUFFER, 0);
	//glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
	//glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
}

void TriangleFusion::renderFinalResult(){
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, outColorTexture);

	renderNormalizedQuad();
}

void TriangleFusion::renderNormalizedQuad(){
	//Push All Setting To Stack
	//glDisable(GL_DEPTH_TEST);
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
	//glEnable(GL_DEPTH_TEST);
	glPopAttrib();
	
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

void TriangleFusion::renderCloudFromSingle(int idx, GLuint colorTex, GLuint depthTex, glm::mat4& const MVP){
	glEnable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, depthTex);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, colorTex);

	cloudProgram->use();
	cloudProgram->setUniform("texColor", 1);
	cloudProgram->setUniform("texDepth", 0);
	cloudProgram->setUniform("MVP", MVP);

	//Camera Information
	shaderProgram->setUniform("fx", intrinsic[idx].fx);
	shaderProgram->setUniform("fy", intrinsic[idx].fy);
	shaderProgram->setUniform("cx", intrinsic[idx].cx);
	shaderProgram->setUniform("cy", intrinsic[idx].cy);

	//Drawing
	glBegin(GL_POINTS);
	for(int i = 0; i < 640; i++) {
		for(int j = 0; j < 480; j++) {
			glVertex3f(i, j, 0);
		}
	}
	glEnd();

	glUseProgram(0);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, 0);	
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, 0);
	
	glDisable(GL_TEXTURE_2D);
}

void TriangleFusion::renderCloudFromAll(GLuint* colorTex, GLuint* depthTex, glm::mat4& const MVP){
	for(int i=0; i<NUM_CAM; ++i){
		//Setup MVP Matrix
		glm::mat4 ModelMatCM(1.f);
		for(int j=0; j<3; ++j){
			ModelMatCM[0][j] = g_extrinsicR[i].data[j].x;
			ModelMatCM[1][j] = g_extrinsicR[i].data[j].y;
			ModelMatCM[2][j] = g_extrinsicR[i].data[j].z;			
		}

		ModelMatCM[3][0] = g_extrinsicT[i].x * 100.f;
		ModelMatCM[3][1] = g_extrinsicT[i].y * 100.f;
		ModelMatCM[3][2] = g_extrinsicT[i].z * 100.f;

		glm::mat4 G_MVPMat = MVP * ModelMatCM;
		renderCloudFromSingle(i, colorTex[i], depthTex[i], G_MVPMat);
	}
}

bool TriangleFusion::createShaderProgram(){
	std::string const SHADER_ROOT_PATH = BEINGTHERE_EXAMPLE_ROOT_DIR+"opengl_triangle_fusion/";

	//Triangle Shader Program
	std::string const VERT_SHADER_SOURCE(SHADER_ROOT_PATH + "shader/triangle_fusion.vert");
	std::string const GEOM_SHADER_SOURCE(SHADER_ROOT_PATH + "shader/triangle_fusion.geom");
	std::string const FRAG_SHADER_SOURCE(SHADER_ROOT_PATH + "shader/triangle_fusion.frag");

	shaderProgram = new GLSLProgram();
	shaderProgram->compileShaderFromFile(VERT_SHADER_SOURCE.c_str(), GLSLShader::VERTEX);   cout << shaderProgram->log() << endl;
	shaderProgram->compileShaderFromFile(GEOM_SHADER_SOURCE.c_str(), GLSLShader::GEOMETRY); cout << shaderProgram->log() << endl;
	shaderProgram->compileShaderFromFile(FRAG_SHADER_SOURCE.c_str(), GLSLShader::FRAGMENT); cout << shaderProgram->log() << endl;
	shaderProgram->link(); cout << shaderProgram->log() << endl;
	shaderProgram->validate(); cout << shaderProgram->log() << endl;

	//Point Cloud Shader Program
	std::string const CLOUD_VERT_SHADER_SOURCE(SHADER_ROOT_PATH + "shader/point_cloud.vert");
	std::string const CLOUD_FRAG_SHADER_SOURCE(SHADER_ROOT_PATH + "shader/point_cloud.frag");

	cloudProgram = new GLSLProgram();
	cloudProgram->compileShaderFromFile(CLOUD_VERT_SHADER_SOURCE.c_str(), GLSLShader::VERTEX);   cout << cloudProgram->log() << endl;
	cloudProgram->compileShaderFromFile(CLOUD_FRAG_SHADER_SOURCE.c_str(), GLSLShader::FRAGMENT); cout << cloudProgram->log() << endl;
	cloudProgram->link(); cout << cloudProgram->log() << endl;
	cloudProgram->validate(); cout << cloudProgram->log() << endl;

	//Prog Quality For Data Merger
	progQuality = new GLSLProgram();
	progQuality->compileShaderFromFile((SHADER_ROOT_PATH+"shader/qualityMetric.frag").c_str(), GLSLShader::FRAGMENT); cout << progQuality->log() << endl;
	progQuality->link(); cout << progQuality->log() << endl;
	progQuality->validate(); cout << progQuality->log() << endl;

	return true;
}

void TriangleFusion::createTriangleTemplate(){
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

	//No Vertex Array Object - Modified Later
	glGenBuffers(1, &triangleTemplateBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, triangleTemplateBuffer);
	glBufferData(GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_SHORT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);
}

bool TriangleFusion::initTextures(){
	glGenTextures(NUM_CAM, tempColorTexture);
	glGenTextures(NUM_CAM, tempDepthTexture);

	glGenTextures(1, &outColorTexture);
	glGenTextures(1, &outDepthTexture);

	initColorTex(outColorTexture);
	initDepthTex(outDepthTexture);

	for(int i=0; i<NUM_CAM; ++i){
		initColorTex(tempColorTexture[i]);
		initDepthTex(tempDepthTexture[i]);
	}

	return true;
}

bool TriangleFusion::initColorTex(GLuint& const colorTex){
	int xRes = 640; 
	int yRes = 480;

	unsigned char * dummyTexColor = (unsigned char *)malloc(xRes*yRes*4);
	memset(dummyTexColor, 0, xRes*yRes*4);

	//Color Texture
	glBindTexture(GL_TEXTURE_2D, colorTex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 640, 480, 0, GL_RGBA, GL_UNSIGNED_BYTE, dummyTexColor);

	glBindTexture(GL_TEXTURE_2D, 0);

	free(dummyTexColor);

	return true;
}

bool TriangleFusion::initDepthTex(GLuint& const depthTex){
	int xRes = 640; 
	int yRes = 480;

	unsigned char* dummyTexDepth = (unsigned char *) malloc(xRes*yRes*sizeof(GL_FLOAT));
	memset(dummyTexDepth, 0, xRes*yRes*sizeof(GL_FLOAT));

	//Depth Texture
	glBindTexture(GL_TEXTURE_2D, depthTex);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_INTENSITY);

	//Mistake - Change to GL_SHORT Later
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, 640, 480, 0, GL_DEPTH_COMPONENT, GL_FLOAT, dummyTexDepth);

	glBindTexture(GL_TEXTURE_2D, 0);

	free(dummyTexDepth);

	return true;
}