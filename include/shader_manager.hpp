
#ifndef SHADER_MANAGER_HPP
#define SHADER_MANAGER_HPP
#pragma once

// std includes
#include <unordered_map>
#include <string>

class ShaderManager {
public:
    unsigned getShader( const std::string& VertexFile,
                        const std::string& FragmentFile );
    void detachShader();
    void destroyShader( unsigned ShaderID );

    const char* readFile( const std::string& FileName );

    const std::unordered_map< std::string, unsigned >& getShaderList() const;

    static ShaderManager& instance();

private:
    std::unordered_map< std::string, unsigned > ShaderList;
    std::unordered_map< unsigned, std::string > ProgramList;
    std::unordered_map< std::string, std::string > SourceList;
};

#endif
