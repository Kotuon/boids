
// System headers
#include <fmt/core.h>

// Local headers
#include "profiler.hpp"
#include "engine.hpp"
#include "graphics.hpp"
#include "trace.hpp"
#include "camera.hpp"
#include "shader_manager.hpp"
#include "model_manager.hpp"
#include "editor.hpp"
#include "input.hpp"

#include "boid_manager.hpp"

constexpr int WIDTH = 1920;
constexpr int HEIGHT = 1080;

std::unique_ptr< BoidManager > BoidManagerInstance;

Model* QuadModel;

Engine::Engine() {}

static void drawQuads() {
    std::vector< float > Positions;
    std::vector< float > Angles;

    auto& Quads = BoidManagerInstance->getQuadtree()->getNodes();
    size_t Size = Quads.size();

    for ( auto& Quad : Quads ) {
        Positions.push_back( 0.f );
        Positions.push_back( Quad->Center.y );
        Positions.push_back( Quad->Center.x );

        Angles.push_back( Quad->Size );
    }

    glBindBuffer( GL_ARRAY_BUFFER, QuadModel->getMesh()->PositionVBO );
    glBufferSubData( GL_ARRAY_BUFFER, 0, sizeof( float ) * 3 * Size,
                     Positions.data() );
    glBindBuffer( GL_ARRAY_BUFFER, 0 );

    glBindBuffer( GL_ARRAY_BUFFER, QuadModel->getMesh()->AngleVBO );
    glBufferSubData( GL_ARRAY_BUFFER, 0, sizeof( float ) * Size,
                     Angles.data() );
    glBindBuffer( GL_ARRAY_BUFFER, 0 );

    glUseProgram( QuadModel->getShader() );

    glUniformMatrix4fv(
        glGetUniformLocation( QuadModel->getShader(), "projection" ), 1,
        GL_FALSE, &Graphics::instance().getProjection()[0][0] );

    glUniform1f( glGetUniformLocation( QuadModel->getShader(), "scale" ), 0.f );

    glBindVertexArray( QuadModel->getMesh()->VAO );

    glDrawArraysInstanced( QuadModel->getRenderMethod(), 0,
                           QuadModel->getMesh()->NumVertices,
                           static_cast< GLsizei >( Size ) );

    glUseProgram( 0 );
    glBindVertexArray( 0 );
}

bool Engine::initialize() {
    if ( !Graphics::instance().initialize( WIDTH, HEIGHT ) ) {
        Trace::message( "Graphics falied to initialize." );
        return false;
    }

    if ( !Camera::instance().initialize( glm::vec3( 0.f, 0.f, 60.f ) ) ) {
        Trace::message( "Camera falied to initialize." );
    }

    if ( !Editor::instance().initialize( Graphics::instance().getWindow() ) ) {
        Trace::message( "Editor failed to initialize." );
    }

    BoidManagerInstance =
        std::make_unique< BoidManager >( glm::vec2( -15.f, 15.f ) );

    Graphics::instance().addRenderCallback( &drawQuads );
    Graphics::instance().addRenderCallback(
        std::bind( &BoidManager::draw, BoidManagerInstance.get() ) );

    Editor::instance().addDisplayMenuCallback( std::bind(
        &BoidManager::displayEditorWindow, BoidManagerInstance.get() ) );

    QuadModel = ModelManager::instance().getModel(
        "models/cube.obj", GL_LINE_STRIP,
        ShaderManager::instance().getShader( "shaders/quad_vert.glsl",
                                             "shaders/quad_frag.glsl" ),
        true );

    Time = std::make_unique< TimeManager >();

    IsRunning = true;

    return true;
}

void Engine::update() {
    // Profiler ProfilerInstance( 100000 );

    while ( IsRunning ) {
        Time->update();

        glfwSetWindowTitle( Graphics::instance().getWindow(),
                            fmt::format( "basic window: FPS: {:0.2f}",
                                         1.f / Time->getDeltaTime() )
                                .c_str() );

        // Non-fixed time step update calls
        Input::instance().update();

        // Fixed time step update calls
        while ( Time->needsFixedUpdate() ) {
            // Call fixed updates here

            // BoidManagerInstance->update();
            BoidManagerInstance->updateTree();

            for ( auto& Func : FixedUpdateCallbacks ) {
                Func();
            }
        }

        // Non-fixed time step update calls
        // TODO: will be moved around

        for ( auto& Func : UpdateCallbacks ) {
            Func();
        }

        Camera::instance().update();
        Graphics::instance().update();
    }
}

void Engine::shutdown() { Graphics::instance().shutdown(); }

void Engine::triggerShutdown() { IsRunning = false; }

Engine& Engine::instance() {
    static Engine EngineInstance;
    return EngineInstance;
}
