
// std includes
#include <fstream>

// System includes
#include <glad/glad.h>
#include <fmt/core.h>

// Local includes
#include "shader_manager.hpp"
#include "trace.hpp"

unsigned ShaderManager::getShader( const std::string& VertexFile,
                                   const std::string& FragmentFile ) {
    auto it = ShaderList.find( VertexFile + FragmentFile );
    if ( it != ShaderList.end() ) {
        return it->second;
    }

    GLint success = 0;
    GLint logSize = 0;

    unsigned vertexShader = glCreateShader( GL_VERTEX_SHADER );
    const char* vertexShaderSource = readFile( VertexFile );
    if ( !vertexShaderSource ) {
        return 0;
    }

    glShaderSource( vertexShader, 1, &vertexShaderSource, nullptr );
    glCompileShader( vertexShader );

    glGetShaderiv( vertexShader, GL_COMPILE_STATUS, &success );
    if ( success == GL_FALSE ) {
        glGetShaderiv( vertexShader, GL_INFO_LOG_LENGTH, &logSize );
        GLchar* infoLog = new GLchar[logSize];
        glGetShaderInfoLog( vertexShader, logSize, &logSize, infoLog );
        glDeleteShader( vertexShader );
        Trace::message(
            fmt::format( "Vertex Shader {}: {}\n", VertexFile, infoLog ) );
        delete[] infoLog;
    }

    unsigned fragmentShader = glCreateShader( GL_FRAGMENT_SHADER );
    const char* fragmentShaderSource = readFile( FragmentFile );
    if ( !fragmentShaderSource ) {
        return 0;
    }

    glShaderSource( fragmentShader, 1, &fragmentShaderSource, nullptr );
    glCompileShader( fragmentShader );

    glGetShaderiv( fragmentShader, GL_COMPILE_STATUS, &success );
    if ( success == GL_FALSE ) {
        glGetShaderiv( fragmentShader, GL_INFO_LOG_LENGTH, &logSize );
        GLchar* infoLog = new GLchar[logSize];
        glGetShaderInfoLog( fragmentShader, logSize, &logSize, infoLog );
        glDeleteShader( fragmentShader );
        Trace::message(
            fmt::format( "Fragment Shader {}: {}\n", FragmentFile, infoLog ) );
        delete[] infoLog;
    }

    unsigned shaderProgram = glCreateProgram();
    glAttachShader( shaderProgram, vertexShader );
    glAttachShader( shaderProgram, fragmentShader );
    glLinkProgram( shaderProgram );

    glDeleteShader( vertexShader );
    glDeleteShader( fragmentShader );

    ShaderList[VertexFile + FragmentFile] = shaderProgram;
    ProgramList[shaderProgram] = VertexFile + FragmentFile;

    return shaderProgram;
}

void ShaderManager::detachShader() { glUseProgram( 0 ); }

void ShaderManager::destroyShader( unsigned ShaderID ) {
    glDeleteProgram( ShaderID );

    auto iter = ProgramList.find( ShaderID );
    ShaderList.erase( iter->second );

    ProgramList.erase( ShaderID );
}

const char* ShaderManager::readFile( const std::string& FileName ) {
    auto it = SourceList.find( FileName );
    if ( it != SourceList.end() ) {
        return it->second.c_str();
    }

    std::string& content = SourceList[FileName];

    std::ifstream file( FileName );
    if ( !file.is_open() ) {
        Trace::message( fmt::format( "Failed to open shader {}.", FileName ) );

        SourceList.erase( FileName );
        return nullptr;
    }

    std::string line = "";
    while ( !file.eof() ) {
        getline( file, line );
        content.append( line + "\n" );
    }

    file.close();

    content.append( "\0" );

    return content.c_str();
}

const std::unordered_map< std::string, unsigned >&
ShaderManager::getShaderList() const {
    return ShaderList;
}

ShaderManager& ShaderManager::instance() {
    static ShaderManager shaderManagerInstance;
    return shaderManagerInstance;
}
