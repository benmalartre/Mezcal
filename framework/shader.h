//--------------------------------------------------------
// Shader
//--------------------------------------------------------
#ifndef _SHADER_H_
#define _SHADER_H_

#include <stdio.h>
#include <string.h>
#include <GL/gl3w.h>

#include "file.h"
#include "glutils.h"

namespace BOB{
    
enum GLSLShaderType
{
    SHADER_VERTEX,
    SHADER_GEOMETRY,
    SHADER_FRAGMENT
};
    
class GLSLShader{
public:
    GLSLShader(){};
    
    void load(std::string fileName);
    void compile(const char* c, GLenum t);
    void compile(GLenum t);
    void outputInfoLog();
    GLuint get(){return _shader;};
    void set(const std::string& code);
private:
    std::string _path;
    GLchar* _code;
    GLenum _type;
    GLuint _shader;
};
    
class GLSLProgram
{
friend GLSLShader;
public:
    GLSLProgram(){};
    GLSLProgram(const char* name);
    GLSLProgram(std::string name);
    GLSLProgram(std::string name,std::string s_vert="",std::string s_frag="");
    ~GLSLProgram();
    void build();
    void outputInfoLog();
    GLuint get(){return _pgm;};
private:
    GLSLShader _vert;
    GLSLShader _geom;
    GLSLShader _frag;
    GLuint _pgm;
    std::string _name;
    
};

}// end namespace BOB
#endif /* _SHADER_H_ */
