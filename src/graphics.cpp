
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

bool Graphics::initialize( const int Width, const int Height ) {
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
    glfwWindowHint( GLFW_RESIZABLE, false );
    glfwWindowHint( GLFW_SAMPLES, 4 ); // MSAA

    // Create Window using GLFW
    Window = glfwCreateWindow( Width, Height, "", nullptr, nullptr );

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
                                 castToString( glGetString( GL_VENDOR ) ),
                                 castToString( glGetString( GL_RENDERER ) ) ) );
    Trace::message( fmt::format( "GLFW\t {}", glfwGetVersionString() ) );
    Trace::message( fmt::format( "OpenGL\t {}",
                                 castToString( glGetString( GL_VERSION ) ) ) );
    Trace::message( fmt::format(
        "GLSL\t {}",
        castToString( glGetString( GL_SHADING_LANGUAGE_VERSION ) ) ) );

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
    glfwSetFramebufferSizeCallback( Window, Graphics::frameBufferSizeCallback );
    glfwSetCursorEnterCallback( Window, Graphics::cursorEnterCallback );
    glfwSetWindowCloseCallback( Window, Graphics::closeWindowCallback );

    Projection = glm::perspective< float >( glm::radians( 45.f ),
                                            static_cast< float >( Width ) /
                                                static_cast< float >( Height ),
                                            0.1f, 1000.0f );

    // Projection = glm::ortho( 0.f, static_cast< float >( Width ), 0.f,
    //                          static_cast< float >( Height ), 0.1f, 100.f );

    return true;
}

void Graphics::update() {
    glm::mat4 view = Camera::instance().getViewMatrix();

    // TODO: setup shaders
    for ( const auto& [key, value] :
          ShaderManager::instance().getShaderList() ) {
        glUseProgram( value );
        glUniformMatrix4fv( glGetUniformLocation( value, "view" ), 1, GL_FALSE,
                            &view[0][0] );
        glUseProgram( 0 );
    }

    // Clear colour and depth buffers
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT |
             GL_STENCIL_BUFFER_BIT );

    // Draw your scene here
    for ( auto& Func : RenderCallbacks ) {
        Func();
    }

    // Flip buffers
    glfwSwapBuffers( Window );

    // Handle other events
    glfwPollEvents();
}

void Graphics::drawNormal( Model* Model, glm::mat4& Matrix ) {
    glUseProgram( Model->getShader() );

    glUniformMatrix4fv( glGetUniformLocation( Model->getShader(), "model" ), 1,
                        GL_FALSE, &Matrix[0][0] );

    glUniformMatrix4fv(
        glGetUniformLocation( Model->getShader(), "projection" ), 1, GL_FALSE,
        &Projection[0][0] );

    glBindVertexArray( Model->getMesh()->VAO );

    glDrawArrays( Model->getRenderMethod(), 0, Model->getMesh()->NumVertices );

    glUseProgram( 0 );

    glBindVertexArray( 0 );
}

void Graphics::drawTriangle( const glm::vec2 P1, const glm::vec2 P2,
                             const glm::vec2 P3, const glm::vec3 Color ) {
    glBegin( GL_TRIANGLES );
    glColor3f( Color.x, Color.y, Color.z );
    glVertex2f( P1.x, P1.y );
    glVertex2f( P2.x, P2.y );
    glVertex2f( P3.x, P3.y );
    glEnd();
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

void Graphics::closeWindowCallback( GLFWwindow* Window ) {
    glfwSetWindowShouldClose( Window, GL_TRUE );
    Engine::instance().triggerShutdown();
}

glm::mat4 Graphics::getProjection() { return Projection; }

Graphics& Graphics::instance() {
    static Graphics graphicsInstance;
    return graphicsInstance;
}

void handleKeyboardInput( GLFWwindow* Window ) {
    // Use escape key for terminating the GLFW Window
    if ( glfwGetKey( Window, GLFW_KEY_ESCAPE ) == GLFW_PRESS ) {
        glfwSetWindowShouldClose( Window, GL_TRUE );
        Engine::instance().triggerShutdown();
    }
}
