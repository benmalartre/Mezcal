//-------------------------------------------------------
// SHADER IMPLEMENTATION
//-------------------------------------------------------
#include "shader.h"
#include <iostream>

namespace BOB{
    void GLSLShader::outputInfoLog()
    {
        char buffer[512];
        glGetShaderInfoLog(_shader, 512, NULL, &buffer[0]);
        std::cout << "COMPILE : " << (std::string)buffer << std::endl;
    }
 
    void GLSLShader::load(std::string fileName)
    {
        File f(fileName);
        if(f.open(FILE_READ))
        {
            
            unsigned long len = f.getFileLength();
            if(len==0)
                std::cout << "[GLSL] File Empty : " << fileName << std::endl;
            else
            {
                
                std::string code = f.readAll();
                _code = (GLchar*) new char[len+1];
                _code[len] = 0;
                memcpy(_code, code.c_str(), len);
            }
            f.close();
        }
        else
            std::cout << "[GLSL] Fail Open File : " << fileName << std::endl;
    }
    
    void GLSLShader::set(const std::string& code)
    {
        unsigned long len = code.length();
        _code = (GLchar*) new char[len+1];
        _code[len] = 0;
        memcpy(_code, code.c_str(), len);

    }
    
    void GLSLShader::compile(const char* code, GLenum type)
    {
        _shader = glCreateShader(type);

        glShaderSource(_shader,1,(GLchar**)code,NULL);
        glCompileShader(_shader);
        
        GLint status;
        glGetShaderiv(_shader,GL_COMPILE_STATUS,&status);
        if(status)
            std::cout << "[GLSLCreateShader] Success Compiling Shader !"<<std::endl;
        else
        {
            std::cout << "[GLSLCreateShader] Fail Compiling Shader !" <<std::endl;
        
        	// Output Info Log
        	outputInfoLog();
        }
    }
    
    void GLSLShader::compile(GLenum type)
    {
        _shader = glCreateShader(type);
        
        glShaderSource(_shader,1,(GLchar**)&_code,NULL);
        glCompileShader(_shader);
        
        GLint status;
        glGetShaderiv(_shader,GL_COMPILE_STATUS,&status);
        if(status)
            std::cout << "[GLSLCreateShader] Success Compiling Shader !"<<std::endl;
        else
        {
            std::cout << "[GLSLCreateShader] Fail Compiling Shader !"<<std::endl;
        	outputInfoLog();
        }
    }

    GLSLProgram::GLSLProgram(std::string name, std::string vertex, std::string fragment)
    {
        _name = name;
        _vert.set(vertex);
        _frag.set(fragment);
        
        build();
    }
    
    GLSLProgram::GLSLProgram(std::string name)
    {
        _name = name;
        std::string filename;
        filename = "glsl/"+name+"_vertex.glsl";
        _vert.load(filename);
        filename = "glsl/"+name+"_fragment.glsl";
        _frag.load(filename);
        
        build();
    }
    
    GLSLProgram::GLSLProgram(const char* name)
    {
        _name = (std::string)name;
        std::string filename;
        filename = "glsl/"+_name+"_vertex.glsl";
        _vert.load(filename);
        filename = "glsl/"+_name+"_fragment.glsl";
        _frag.load(filename);
        
        build();
    }

    
    void GLSLProgram::build()
    {
        _vert.compile(GL_VERTEX_SHADER);
        GLCheckError("Compile Vertex Shader : ");
        
        _frag.compile(GL_FRAGMENT_SHADER);
        GLCheckError("Compile Fragment Shader : ");
        
        _pgm = glCreateProgram();
        GLCheckError("Create Program : ");
        
        glAttachShader(_pgm,_vert.get());
        GLCheckError("Attach Vertex Shader ");
        
        glAttachShader(_pgm,_frag.get());
        GLCheckError("Attach Fragment Shader ");
        
        glBindAttribLocation(_pgm,0,"position");
        
        glLinkProgram(_pgm);
        GLCheckError("Link Program : ");
        
        glUseProgram(_pgm);
        GLCheckError("Use Program : ");
        
        outputInfoLog();

    }
    
    void GLSLProgram::outputInfoLog()
    {
        char buffer[1024];
        GLsizei l;
        glGetProgramInfoLog(_pgm,1024,&l,&buffer[0]);
        std::cout << _name << ":" << (std::string) buffer << std::endl;
    }
    
    GLSLProgram::~GLSLProgram(){};
}
