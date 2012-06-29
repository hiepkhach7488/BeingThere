#ifndef GLSLPROGRAM_H
#define GLSLPROGRAM_H

#include <GL/glew.h>
#include <GL/GL.h>
#include <GL/freeglut.h>

//#define GL_GLEXT_PROTOTYPES 1
//#include <GL/freeglut.h>
//#include <GL/glext.h>
//#include <GL/glu.h>

#include <string>
using std::string;

#include <glm/glm.hpp>
//using glm::vec2;
//using glm::vec3;
//using glm::vec4;
//using glm::mat4;
//using glm::mat3;

namespace GLSLShader {
    enum GLSLShaderType {
        VERTEX, FRAGMENT, GEOMETRY,
        TESS_CONTROL, TESS_EVALUATION
    };
};

class GLSLProgram
{
private:
    int  handle;
    bool linked;
    string logString;

    int  getUniformLocation(const char * name );
    bool fileExists( const string & fileName );

public:
    GLSLProgram();

    bool   compileShaderFromFile( const char * fileName, GLSLShader::GLSLShaderType type );
    bool   compileShaderFromString( const string & source, GLSLShader::GLSLShaderType type );
    bool   link();
    bool   validate();
    void   use();

    string log();

    int    getHandle();
    bool   isLinked();

    void   bindAttribLocation( GLuint location, const char * name);
    void   bindFragDataLocation( GLuint location, const char * name );

    void   setUniform( const char *name, float x, float y, float z);
    void   setUniform( const char *name, const glm::vec2 & v);
    void   setUniform( const char *name, const glm::vec3 & v);
    void   setUniform( const char *name, const glm::vec4 & v);
    void   setUniform( const char *name, const glm::mat4 & m);
    void   setUniform( const char *name, const glm::mat3 & m);
    void   setUniform( const char *name, float val );
    void   setUniform( const char *name, int val );
    void   setUniform( const char *name, bool val );

    void   printActiveUniforms();
    void   printActiveAttribs();
};

#endif // GLSLPROGRAM_H
