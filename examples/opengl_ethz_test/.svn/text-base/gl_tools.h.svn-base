#ifndef GL_TOOLS_H
#define GL_TOOLS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include "grid_mesh.h"
#include <GL/glew.h>

using namespace std;
#define checkGLErrorsHere() { GLTools::checkGLErrors(__FILE__, __LINE__); }

class GLTools{

public:

	static void pushMatrices();
	static void popMatrices();
	static void drawTriangle();
	static void drawMesh(GridMesh* mesh);

	static void drawText(char* str, int posX, int posY, int windowWidth, int windowHeight);

	static void writeDat(GLuint texId, char* name, int w, int h);
	static void writeTxt(float* buffer, char* name, int w, int h);
	static void writeTxt(GLuint texId, char* name, int w, int h);

	static void writePPMColor(unsigned char* bufferC, char* name, int w, int h);
	static void writePPMColor(GLuint texId, char* name, int w, int h);
	static void writePPMColorF(GLuint texId, char* name, int w, int h);
	static void writePPMColorF(float* bufferF, char* name, int w, int h);
	static void writePPM(unsigned char* buffer, char* name, int w, int h);
	static void writePPM(float* bufferF, char* name, int w, int h);
	static void writePPM(GLuint texId, char* name, int w, int h);

	static void checkGLErrors(char const * file, int line, std::ostream& os = std::cerr);

	static string readFile(const char* filename);
	static void printShaderInfoLog(GLuint shaderID);
	static void printProgramInfoLog(GLuint programID);
	static GLuint createShaderProgram(const char* vsFilename, const char* fsFilename);
	static GLuint createShaderProgram(const char* vsFilename, const char* gsFilename, const char* fsFilename);

	static void drawOrigin();

private:


};


#endif
