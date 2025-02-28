
// std includes

// System headers
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <fmt/core.h>
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/matrix_float4x4.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/transform.hpp>

// Local headers
#include "graphics.hpp"
#include "trace.hpp"
#include "engine.hpp"
#include "model_manager.hpp"
#include "camera.hpp"
#include "shader_manager.hpp"
#include "editor.hpp"

static const char* castToString( const unsigned char* Input ) {
    return reinterpret_cast< const char* >( Input );
}

Graphics::Graphics() {}

bool Graphics::initialize() {
    if ( !glfwInit() ) {
        Trace::message( "Could not start GLFW." );
        return false;
    }

    // Set core Window options (adjust version numbers if needed)
    glfwWindowHint( GLFW_CONTEXT_VERSION_MAJOR, 4 );
    glfwWindowHint( GLFW_CONTEXT_VERSION_MINOR, 3 );
    glfwWindowHint( GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE );

    // Enable the GLFW runtime error callback function defined previously.
    glfwSetErrorCallback( Graphics::GLFWErrorCallback );

    // Set additional Window options
    glfwWindowHint( GLFW_RESIZABLE, windowResizable );
    glfwWindowHint( GLFW_SAMPLES, windowSamples ); // MSAA

    // Create Window using GLFW
    Window = glfwCreateWindow( windowWidth, windowHeight, windowTitle.c_str(),
                               nullptr, nullptr );

    // Ensure the Window is set up correctly
    if ( !Window ) {
        Trace::message( "Could not open GLFW Window." );

        glfwTerminate();
        return false;
    }

    // Let the Window be the current OpenGL context and initialise glad
    glfwMakeContextCurrent( Window );
    gladLoadGL();

    Trace::message( fmt::format( "{}: {}",
                                 CastToString( glGetString( GL_VENDOR ) ),
                                 CastToString( glGetString( GL_RENDERER ) ) ) );
    Trace::message( fmt::format( "GLFW\t {}", glfwGetVersionString() ) );
    Trace::message( fmt::format( "OpenGL\t {}",
                                 CastToString( glGetString( GL_VERSION ) ) ) );
    Trace::message( fmt::format(
        "GLSL\t {}",
        CastToString( glGetString( GL_SHADING_LANGUAGE_VERSION ) ) ) );

    // Enable depth (Z) buffer (accept "closest" fragment)
    glEnable( GL_DEPTH_TEST );
    glDepthFunc( GL_LESS );

    // Configure miscellaneous OpenGL settings
    glEnable( GL_CULL_FACE );
    glCullFace( GL_BACK );
    glFrontFace( GL_CCW );

    glEnable( GL_BLEND );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

    glPointSize( 3.0 );

    // Set default colour after clearing the colour buffer
    glClearColor( 0.2f, 0.2f, 0.2f, 1.0f );
    glClearStencil( 0 );

    // Set callbacks
    glfwSetFramebufferSizeCallback( Window, Graphics::FrameBufferSizeCallback );
    glfwSetCursorEnterCallback( Window, Graphics::CursorEnterCallback );
    glfwSetWindowCloseCallback( Window, Input::CloseWindowCallback );

    projection =
        glm::perspective< float >( glm::radians( 45.f ),
                                   static_cast< float >( windowWidth ) /
                                       static_cast< float >( windowHeight ),
                                   0.1f, 100.0f );

    return true;
}

void Graphics::update() {
    glm::mat4 view = Camera::Instance().GetViewMatrix();
    // TODO: setup shaders
    for ( const auto& [key, value] :
          ShaderManager::Instance().GetShaderList() ) {
        glUseProgram( value );
        glUniformMatrix4fv( glGetUniformLocation( value, "view" ), 1, GL_FALSE,
                            &view[0][0] );
        glUseProgram( 0 );
    }

    // Clear colour and depth buffers
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT |
             GL_STENCIL_BUFFER_BIT );

    // Draw your scene here
    for ( auto& func : render_callbacks ) {
        func();
    }

    // Flip buffers
    glfwSwapBuffers( Window );

    // Handle other events
    glfwPollEvents();
}

void Graphics::drawNormal( Model* Model, glm::mat4& Matrix ) {
    glUseProgram( Model->GetShader() );

    glUniformMatrix4fv( glGetUniformLocation( Model->GetShader(), "model" ), 1,
                        GL_FALSE, &Matrix[0][0] );

    glUniformMatrix4fv(
        glGetUniformLocation( Model->GetShader(), "projection" ), 1, GL_FALSE,
        &projection[0][0] );

    glBindVertexArray( Model->GetMesh()->VAO );

    glDrawArrays( Model->GetRenderMethod(), 0, Model->GetMesh()->num_vertices );

    glUseProgram( 0 );

    glBindVertexArray( 0 );
}

void Graphics::shutdown() {
    // Terminate GLFW (no need to call glfwDestroyWindow)
    glfwTerminate();
}

GLFWwindow* Graphics::getWindow() const { return Window; }

void Graphics::frameBufferSizeCallback( GLFWwindow*, int Width, int Height ) {
    glViewport( 0, 0, Width, Height );
}

void Graphics::cursorEnterCallback( GLFWwindow*, int Entered ) {
    if ( Entered ) {
        // The cursor entered the content area of the Window
        // glfwSetInputMode( Graphics::Instance().Window, GLFW_CURSOR,
        // GLFW_CURSOR_DISABLED ); cursorEntered = true;
    } else {
        // The cursor left the content area of the Window
    }
}

void Graphics::GLFWErrorCallback( int Error, const char* Description ) {
    std::string message =
        "GLFW returned an error: " + std::string( Description ) +
        std::to_string( Error );

    Trace::message( message );
}

glm::mat4 Graphics::getProjection() { return projection; }

Graphics& Graphics::instance() {
    static Graphics graphicsInstance;
    return graphicsInstance;
}

void handleKeyboardInput( GLFWwindow* Window ) {
    // Use escape key for terminating the GLFW Window
    if ( glfwGetKey( Window, GLFW_KEY_ESCAPE ) == GLFW_PRESS ) {
        glfwSetWindowShouldClose( Window, GL_TRUE );
        Engine::Instance().TriggerShutdown();
    }
}
