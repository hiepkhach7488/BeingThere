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

using namespace std;

GLuint textureID;
glm::mat4 Projection;
glm::mat4 View;
glm::mat4 Model;
glm::mat4 MVP;

namespace SHADER_ATTR{
	enum type{
		POSITION = 0,
		NORMAL = 1,
		COLOR = 3,
		TEXCOORD = 4
	};
} //end namespace attr

GLSLProgram* TransformProgramName;
GLuint TransformArrayBufferPositionName(0);
GLuint TransformArrayBufferTexCoordName(0);
GLuint TransformVertexArrayName(0);

GLint TransformUniformMVP(0);

GLSLProgram* FeedbackProgramName;
GLuint FeedbackArrayBufferPositionName(0);
GLuint FeedbackArrayBufferTexCoordName(0);
GLuint FeedbackVertexArrayName(0);

GLuint elementBufferID;
GLuint Query(0);


//Interleaved Buffer
GLuint FeedbackArrayBufferInterleavedName;
GLuint FeedbackVertexArrayInterleavedName;

namespace glm{
	typedef struct vec6{
		glm::vec4 position;
		glm::vec2 texCoord;
	};
}

namespace{
	//std::string const SHADER_ROOT_PATH = "D:/Trunk/bibleProj/beingthere/examples/opengl_transform_feedback/";
	std::string const SHADER_ROOT_PATH = "C:/Logs/PCL/beingthere/examples/opengl_transform_feedback/";
	std::string const VERT_SHADER_SOURCE_TRANSFORM(SHADER_ROOT_PATH + "separated/image-2d-transform.vert");
	std::string const VERT_SHADER_SOURCE_FEEDBACK(SHADER_ROOT_PATH + "separated/image-2d-feedback.vert");
	std::string const FRAG_SHADER_SOURCE_FEEDBACK(SHADER_ROOT_PATH + "separated/image-2d-feedback.frag");
}

void printMat(glm::mat4& const mat){
	for(int i=0; i<4; ++i)
		cout << mat[i].x << " " << mat[i].y << " " << mat[i].z << " " << mat[i].w << endl;
}

void glutIdle (void)
{
	glutPostRedisplay();
}

void glutKeyboard (unsigned char key, int x, int y)
{
	if(key==27) exit(0);
	if(key=='p') {
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, FeedbackArrayBufferPositionName);
		glm::vec4 * ptr = (glm::vec4 *) glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_READ_ONLY);

		for(int i=0; i<6; ++i){
			cout << ptr[i].x << " " << ptr[i].y  << " " << ptr[i].z << " " << ptr[i].w << endl;
		}

		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
	}
	if(key=='t'){
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, FeedbackArrayBufferTexCoordName);
		glm::vec2 * ptr = (glm::vec2 *) glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_READ_ONLY);

		for(int i=0; i<6; ++i){
			cout << ptr[i].x << " " << ptr[i].y << endl;
		}

		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
	}

	//Print Interleaved Buffers
	if(key == 'i'){
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, FeedbackArrayBufferInterleavedName);
		glm::vec6* ptr = (glm::vec6*) glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_READ_ONLY);

		for(int i=0; i<6; ++i){
			cout << ptr[i].position.x << " " << ptr[i].position.y << " " << ptr[i].position.z 
				<< " " << ptr[i].position.w << " " << ptr[i].texCoord.x << " " << ptr[i].texCoord.y <<  endl;
		}

		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
	}

	if(key == 'm')
		printMat(MVP);
}

void ReSizeGLScene (int Width, int Height)
{
  glViewport (0, 0, Width, Height);
}

void updateMVPOrtho(glm::mat4& MVP){
	Projection = glm::ortho(0.f, 1.f, 0.f, 1.f, -1.f, 1.f);
	View = glm::mat4(1.f);
	Model = glm::mat4(1.0f);
	
	MVP = Projection * View * Model;
}

void glutDisplay (void)
{
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, textureID);

	updateMVPOrtho(MVP);

	//First Draw
	glEnable(GL_RASTERIZER_DISCARD);
	TransformProgramName->use();
	TransformProgramName->setUniform("MVP", MVP);

	//glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, FeedbackArrayBufferPositionName); 
	//glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 1, FeedbackArrayBufferTexCoordName); 
	
	//Interleaved Buffer Base
	glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, FeedbackArrayBufferInterleavedName); 

	glBindVertexArray(TransformVertexArrayName);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementBufferID);

	glBeginQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN, Query); 
		glBeginTransformFeedback(GL_TRIANGLES);
		//glDrawElementsInstancedBaseVertex(GL_POINTS, 6, GL_UNSIGNED_INT, NULL, 1, 0);
		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);	
		//glDrawArrays(GL_POINTS, 0, 4);
		glEndTransformFeedback();
	glEndQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN); 

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
	glDisable(GL_RASTERIZER_DISCARD);	
	
	GLuint PrimitivesWritten = 0;
	glGetQueryObjectuiv(Query, GL_QUERY_RESULT, &PrimitivesWritten);

	//cout << "Writen:= " << PrimitivesWritten << endl;

	//Second Draw
	FeedbackProgramName->use();
	FeedbackProgramName->setUniform("textureMap", 0);
	//glBindVertexArray(FeedbackVertexArrayName);
	//Interleaved Case
	glBindVertexArray(FeedbackVertexArrayInterleavedName);
		//glDrawArrays(GL_TRIANGLES, 0, 6);
		glDrawArraysInstanced(GL_TRIANGLES, 0, PrimitivesWritten * 3, 1);
		//glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

	glBindVertexArray(0);
	glUseProgram(0);

	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);

	glutSwapBuffers();
}

void createArrayBuffer(){
	//Vertex Info
	GLsizei const VertexCount(4);

	GLsizei const VertexPositionSize = VertexCount * sizeof(glm::vec2);
	glm::vec2 const PositionData[] = {
		glm::vec2( 0.0f, 0.0f),
		glm::vec2( 1.0f, 0.0f),
		glm::vec2( 1.0f, 1.0f), 
		glm::vec2( 0.0f, 1.0f), 
	};

	GLsizei const VertexTexCoordSize = VertexCount * sizeof(glm::vec2);
	glm::vec2 const TextCoordData[] = {
		glm::vec2( 0.0f, 1.0f),
		glm::vec2( 1.0f, 1.0f),
		glm::vec2( 1.0f, 0.0f), 
		glm::vec2( 0.0f, 0.0f), 
	};

	GLsizei const VertexElementSize = 6 * sizeof(glm::uint);
	glm::uint const ElementData[] = {
		0, 1, 3,
		1, 3, 2
	};

	//Indices Buffer
	glGenBuffers(1, &elementBufferID);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementBufferID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, VertexElementSize, ElementData, GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	//Position
	glGenBuffers(1, &TransformArrayBufferPositionName);
	glBindBuffer(GL_ARRAY_BUFFER, TransformArrayBufferPositionName);
	glBufferData(GL_ARRAY_BUFFER, VertexPositionSize, PositionData, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	//Texture Coord
	glGenBuffers(1, &TransformArrayBufferTexCoordName);
	glBindBuffer(GL_ARRAY_BUFFER, TransformArrayBufferTexCoordName);
	glBufferData(GL_ARRAY_BUFFER, VertexTexCoordSize, TextCoordData, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	//Buffers For Output - Position - NOTE: SIZE OF FEEDBACK MUST BE CORRECT
	glGenBuffers(1, &FeedbackArrayBufferPositionName);
	glBindBuffer(GL_ARRAY_BUFFER, FeedbackArrayBufferPositionName);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec4) * 6, NULL, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	//Output - TexCoord
	glGenBuffers(1, &FeedbackArrayBufferTexCoordName);
	glBindBuffer(GL_ARRAY_BUFFER, FeedbackArrayBufferTexCoordName);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec2) * 6, NULL, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	//Interleaved Buffer
	glGenBuffers(1, &FeedbackArrayBufferInterleavedName);
	glBindBuffer(GL_ARRAY_BUFFER, FeedbackArrayBufferInterleavedName);
	glBufferData(GL_ARRAY_BUFFER, (sizeof(glm::vec4)+sizeof(glm::vec2))* 6, NULL, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void createVertexArray(){
	// Build a vertex array object - Transform VAO
	glGenVertexArrays(1, &TransformVertexArrayName);
    glBindVertexArray(TransformVertexArrayName);
		glBindBuffer(GL_ARRAY_BUFFER, TransformArrayBufferPositionName);
		glVertexAttribPointer(SHADER_ATTR::POSITION, 2, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(SHADER_ATTR::POSITION);

		glBindBuffer(GL_ARRAY_BUFFER, TransformArrayBufferTexCoordName);
		glVertexAttribPointer(SHADER_ATTR::TEXCOORD, 2, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(SHADER_ATTR::TEXCOORD);
		
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	// Build a vertex array object - Feedback VAO
	glGenVertexArrays(1, &FeedbackVertexArrayName);
	glBindVertexArray(FeedbackVertexArrayName);
		glBindBuffer(GL_ARRAY_BUFFER, FeedbackArrayBufferPositionName);
		glVertexAttribPointer(SHADER_ATTR::POSITION, 4, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(SHADER_ATTR::POSITION);

		glBindBuffer(GL_ARRAY_BUFFER, FeedbackArrayBufferTexCoordName);
		glVertexAttribPointer(SHADER_ATTR::TEXCOORD, 2, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(SHADER_ATTR::TEXCOORD);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	//FeedbackInterleaved VAO
	glGenVertexArrays(1, &FeedbackVertexArrayInterleavedName);
	glBindVertexArray(FeedbackVertexArrayInterleavedName);
	glBindBuffer(GL_ARRAY_BUFFER, FeedbackArrayBufferInterleavedName);
	glVertexAttribPointer(SHADER_ATTR::POSITION, 4, GL_FLOAT, GL_FALSE, sizeof(glm::vec4)+sizeof(glm::vec2), 0);
	glVertexAttribPointer(SHADER_ATTR::TEXCOORD, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec4)+sizeof(glm::vec2), (const GLvoid*)(sizeof(glm::vec4)));
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glEnableVertexAttribArray(SHADER_ATTR::POSITION);
	glEnableVertexAttribArray(SHADER_ATTR::TEXCOORD);
	glBindVertexArray(0);
}

void createShader(){
	//Transform Program
	TransformProgramName = new GLSLProgram();

	TransformProgramName->compileShaderFromFile(VERT_SHADER_SOURCE_TRANSFORM.c_str(), GLSLShader::VERTEX);
	cout << TransformProgramName->log() << endl;

	GLchar const * Strings[] = {"gl_Position", "VertTexCoord"}; 
	//glTransformFeedbackVaryings(TransformProgramName->getHandle(), 2, Strings, GL_SEPARATE_ATTRIBS); 
	glTransformFeedbackVaryings(TransformProgramName->getHandle(), 2, Strings, GL_INTERLEAVED_ATTRIBS); 

	TransformProgramName->link();
	cout << TransformProgramName->log() << endl;

	TransformProgramName->validate();	
	cout << TransformProgramName->log() << endl;

	//Feedback Program
	FeedbackProgramName = new GLSLProgram();
	FeedbackProgramName->compileShaderFromFile(VERT_SHADER_SOURCE_FEEDBACK.c_str(), GLSLShader::VERTEX);
	cout << TransformProgramName->log() << endl;
	FeedbackProgramName->compileShaderFromFile(FRAG_SHADER_SOURCE_FEEDBACK.c_str(), GLSLShader::FRAGMENT);
	cout << TransformProgramName->log() << endl;

	FeedbackProgramName->link();
	cout << TransformProgramName->log() << endl;

	FeedbackProgramName->validate();	
	cout << TransformProgramName->log() << endl;
}

bool createTexture(GLuint& textureID){
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	unsigned char * dummyTex = (unsigned char *)malloc(640*480*3);
	memset(dummyTex, 0, 640*480*3);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, dummyTex);

	glBindTexture(GL_TEXTURE_2D, 0);

	return true;
}

bool uploadToTexture(GLuint& textureID){
	//cv::Mat img = cv::imread("D:/Trunk/bibleProj/beingthere-data/multicams/snapshot_rgb_3.png");
	cv::Mat img = cv::imread("C:/Logs/PCL/data/snapshot_rgb_3.png");
	glBindTexture(GL_TEXTURE_2D, textureID);

	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	/*glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 640, 480, 0,
		GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*) img.data);*/

	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_RGB, GL_UNSIGNED_BYTE, img.data);

	glBindTexture(GL_TEXTURE_2D, 0);

	return true;
}

int main(int argc, char* argv[]){
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
	glutInitWindowSize(640, 480);
	glutCreateWindow ("OpenGL Test");
	//glutHideWindow();

	glewInit();
	
	glGenQueries(1, &Query);
	createTexture(textureID);
	uploadToTexture(textureID);

	createArrayBuffer();
	createVertexArray();

	createShader();

	glutReshapeFunc (&ReSizeGLScene);
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glutMainLoop();
	return 0;
}