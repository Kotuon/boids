
// std includes
#include <array>

// System includes
#include <glm/glm.hpp>
#include <fmt/core.h>

// Local includes
#include "model_manager.hpp"
#include "trace.hpp"

Mesh::Mesh() {}

Mesh::Mesh( std::string ModelFileName_ ) : ModelFileName( ModelFileName_ ) {}

Model::Model() : MeshData( nullptr ), RenderMethod( GL_TRIANGLES ) {}

Model::Model( Mesh* NewMesh, unsigned Shader )
    : MeshData( NewMesh ), RenderMethod( GL_TRIANGLES ), Shader( Shader ) {}

Model::Model( Mesh* NewMesh, unsigned RenderMethod, unsigned Shader )
    : MeshData( NewMesh ), RenderMethod( RenderMethod ), Shader( Shader ) {}

void Model::draw() {}

void Model::setMesh( Mesh* NewMesh ) { MeshData = NewMesh; }

Mesh* Model::getMesh() const { return MeshData; }

void Model::setRenderMethod( unsigned RenderMethod_ ) {
    RenderMethod = RenderMethod_;
}

unsigned Model::getRenderMethod() const { return RenderMethod; }

void Model::setShader( unsigned NewShader ) { Shader = NewShader; }

unsigned Model::getShader() const { return Shader; }

ModelManager::ModelManager() {}

Model* ModelManager::getModel( const std::string& ModelFileName,
                               unsigned Shader, bool Instanced ) {
    Mesh* MeshData = getMesh( ModelFileName, Instanced );
    Model* model = new Model( MeshData, Shader );

    return model;
}

Model* ModelManager::getModel( const std::string& ModelFileName,
                               unsigned RenderMethod, unsigned Shader,
                               bool Instanced ) {
    Mesh* MeshData = getMesh( ModelFileName, Instanced );
    Model* model = new Model( MeshData, RenderMethod, Shader );

    return model;
}

std::vector< float >*
ModelManager::LoadObj( const std::string& ModelFileName ) {
    auto it = vertices_list.find( ModelFileName );
    if ( it != vertices_list.end() ) {
        return &it->second;
    }

    FILE* file;
    errno_t err = fopen_s( &file, ModelFileName.c_str(), "r" );
    if ( err != 0 ) {
        Trace::message( fmt::format( "Unable to open {}.", ModelFileName ) );
        return nullptr;
    }

    auto vertIter = vertices_list.insert( { ModelFileName, {} } );
    std::vector< float >* verticesList = &( vertIter.first )->second;

    std::array< glm::vec3, VERTEX_LIMIT > v{};
    std::array< glm::vec3, VERTEX_LIMIT > vt{};
    std::array< glm::vec3, VERTEX_LIMIT > vn{};

    int vCount = 0;
    int vnCount = 0;
    int vtCount = 0;

    char line[64];
    char* temp;
    while ( fgets( line, 64, file ) ) {
        char* words[4];
        words[0] = strtok_s( line, " ", &temp );
        for ( int i = 1; i < 4; ++i ) {
            words[i] = strtok_s( nullptr, " ", &temp );
        }

        if ( strcmp( words[0], "v" ) == 0 ) {
            v[vCount++] = { atof( words[1] ), atof( words[2] ),
                            atof( words[3] ) };
        } else if ( strcmp( words[0], "vt" ) == 0 ) {
            vt[vtCount++] = { atof( words[1] ), atof( words[2] ), 0.f };
        } else if ( strcmp( words[0], "vn" ) == 0 ) {
            vn[vnCount++] = { atof( words[1] ), atof( words[2] ),
                              atof( words[3] ) };
        } else if ( strcmp( words[0], "f" ) == 0 ) {
            char* v1[3];
            char* v2[3];
            char* v3[3];

            v1[0] = strtok_s( words[1], "/", &temp );
            v1[1] = strtok_s( nullptr, "/", &temp );
            v1[2] = strtok_s( nullptr, "/", &temp );

            v2[0] = strtok_s( words[2], "/", &temp );
            v2[1] = strtok_s( nullptr, "/", &temp );
            v2[2] = strtok_s( nullptr, "/", &temp );

            v3[0] = strtok_s( words[3], "/", &temp );
            v3[1] = strtok_s( nullptr, "/", &temp );
            v3[2] = strtok_s( nullptr, "/", &temp );

            insertData( *verticesList, v1, v, vt, vn );
            insertData( *verticesList, v2, v, vt, vn );
            insertData( *verticesList, v3, v, vt, vn );
        }
    }

    fclose( file );

    return verticesList;
}

Mesh* ModelManager::getMesh( const std::string& ModelFileName,
                             bool Instanced ) {
    std::vector< float >* vertices = LoadObj( ModelFileName );
    if ( !vertices ) {
        return nullptr;
    }

    Mesh* MeshData = new Mesh( ModelFileName );

    MeshData->Instanced = Instanced;
    MeshData->NumVertices = static_cast< int >( vertices->size() / STRIDE );

    glGenVertexArrays( 1, &( MeshData->VAO ) );
    glBindVertexArray( MeshData->VAO );

    glGenBuffers( 1, &( MeshData->VBO ) );

    glBindBuffer( GL_ARRAY_BUFFER, MeshData->VBO );
    glBufferData( GL_ARRAY_BUFFER,
                  sizeof( float ) * STRIDE * MeshData->NumVertices,
                  vertices->data(), GL_STATIC_DRAW );

    // Position
    glEnableVertexAttribArray( 0 );
    glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, STRIDE * sizeof( float ),
                           ( void* )0 );
    // Normal
    glEnableVertexAttribArray( 1 );
    glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, STRIDE * sizeof( float ),
                           ( void* )12 );
    // Texture
    glEnableVertexAttribArray( 2 );
    glVertexAttribPointer( 2, 2, GL_FLOAT, GL_FALSE, STRIDE * sizeof( float ),
                           ( void* )24 );

    if ( Instanced ) {
        // Position
        glGenBuffers( 1, &( MeshData->PositionVBO ) );
        glBindBuffer( GL_ARRAY_BUFFER, MeshData->PositionVBO );
        glBufferData( GL_ARRAY_BUFFER,
                      sizeof( float ) * INSTANCE_STRIDE * MAX_INSTANCES, NULL,
                      GL_STREAM_DRAW );
        glEnableVertexAttribArray( 3 );
        glVertexAttribPointer( 3, 3, GL_FLOAT, GL_FALSE,
                               INSTANCE_STRIDE * sizeof( float ), ( void* )0 );
        glVertexAttribDivisor( 3, 1 );

        glGenBuffers( 1, &( MeshData->AngleVBO ) );
        glBindBuffer( GL_ARRAY_BUFFER, MeshData->AngleVBO );
        glBufferData( GL_ARRAY_BUFFER, sizeof( float ) * MAX_INSTANCES, NULL,
                      GL_STREAM_DRAW );
        glEnableVertexAttribArray( 4 );
        glVertexAttribPointer( 4, 1, GL_FLOAT, GL_FALSE, sizeof( float ),
                               ( void* )0 );
        glVertexAttribDivisor( 4, 1 );

        // Velocity
        // glGenBuffers( 1, &( MeshData->velocity_VBO ) );
        // glBindBuffer( GL_ARRAY_BUFFER, MeshData->velocity_VBO );
        // glBufferData( GL_ARRAY_BUFFER, sizeof( float ) * MAX_INSTANCES, NULL,
        //               GL_STREAM_DRAW );
        // glEnableVertexAttribArray( 4 );
        // glVertexAttribPointer( 4, 1, GL_FLOAT, GL_FALSE, sizeof( float ),
        //                        ( void* )0 );
        // glVertexAttribDivisor( 4, 1 );
    }

    glBindBuffer( GL_ARRAY_BUFFER, 0 );

    glBindVertexArray( 0 );

    return MeshData;
}

void ModelManager::insertData( std::vector< float >& vertices, char* data[3],
                               std::array< glm::vec3, VERTEX_LIMIT >& v,
                               std::array< glm::vec3, VERTEX_LIMIT >& vt,
                               std::array< glm::vec3, VERTEX_LIMIT >& vn ) {
    vertices.push_back( v[atoi( data[0] ) - 1].x );
    vertices.push_back( v[atoi( data[0] ) - 1].y );
    vertices.push_back( v[atoi( data[0] ) - 1].z );

    vertices.push_back( vn[atoi( data[2] ) - 1].x );
    vertices.push_back( vn[atoi( data[2] ) - 1].y );
    vertices.push_back( vn[atoi( data[2] ) - 1].z );

    vertices.push_back( vt[atoi( data[1] ) - 1].x );
    vertices.push_back( vt[atoi( data[1] ) - 1].y );
}

ModelManager& ModelManager::instance() {
    static ModelManager modelManagerInstance;
    return modelManagerInstance;
}
