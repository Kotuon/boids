
#ifndef MODEL_MANAGER_HPP
#define MODEL_MANAGER_HPP
#pragma once

// std includes
#include <unordered_map>
#include <string>

// System includes
#include <glad/glad.h>
#include <glm/glm.hpp>

constexpr unsigned VERTEX_LIMIT = 2000;
constexpr unsigned STRIDE = 8;
constexpr unsigned INSTANCE_STRIDE = 3;
constexpr unsigned MAX_INSTANCES = 80000;

struct Mesh {
    Mesh();
    Mesh( std::string ModelFileName_ );

    std::string ModelFileName;
    int NumVertices;
    unsigned VAO;
    unsigned VBO;
    unsigned PositionVBO;
    unsigned AngleVBO;
    bool Instanced;
};

class Model {
public:
    Model( Mesh* NewMesh, unsigned Shader_ );
    Model( Mesh* NewMesh, unsigned RenderMethod_, unsigned Shader_ );
    void draw();

    void setMesh( Mesh* NewMesh );
    Mesh* getMesh() const;

    void setRenderMethod( unsigned RenderMethod_ );
    unsigned getRenderMethod() const;

    void setShader( unsigned NewShader );
    unsigned getShader() const;

private:
    Model();
    Mesh* MeshData;
    unsigned RenderMethod;
    unsigned Shader;
};

class ModelManager {
public:
    Model* getModel( const std::string& ModelFileName, unsigned Shader,
                     bool Instanced );
    Model* getModel( const std::string& ModelFileName, unsigned RenderMethod,
                     unsigned Shader, bool Instanced );

    static ModelManager& instance();

private:
    ModelManager();

    Mesh* getMesh( const std::string& ModelFileName, bool Instanced );

    std::vector< float >* LoadObj( const std::string& ModelFileName );

    void insertData( std::vector< float >& vertices, char* data[3],
                     std::array< glm::vec3, VERTEX_LIMIT >& v,
                     std::array< glm::vec3, VERTEX_LIMIT >& vt,
                     std::array< glm::vec3, VERTEX_LIMIT >& vn );

    std::unordered_map< std::string, std::vector< float > > vertices_list;
};

#endif
