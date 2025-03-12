
// #include <fmt/core.h>
// #include "trace.hpp"

// #include <glad/glad.h>
// #include <GLFW/glfw3.h>

// #include "shader_manager.hpp"

// static const int WIDTH = 1280;
// static const int HEIGHT = 720;

// static void frameBufferSizeCallback( GLFWwindow* Window, int Width,
//                                      int Height );
// static void processInput( GLFWwindow* Window );

// int main( int, char** ) {
//     glfwInit();
//     glfwWindowHint( GLFW_CONTEXT_VERSION_MAJOR, 3 );
//     glfwWindowHint( GLFW_CONTEXT_VERSION_MINOR, 3 );
//     glfwWindowHint( GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE );

//     GLFWwindow* Window =
//         glfwCreateWindow( WIDTH, HEIGHT, "test", nullptr, nullptr );
//     if ( !Window ) {
//         Trace::message( "Failed to create window." );
//         glfwTerminate();
//         return -1;
//     }

//     glfwMakeContextCurrent( Window );
//     glfwSetFramebufferSizeCallback( Window, frameBufferSizeCallback );

//     if ( !gladLoadGLLoader( ( GLADloadproc )glfwGetProcAddress ) ) {
//         Trace::message( "Failed to initialize GLAD." );
//         return -1;
//     }

//     ///////////////////////////////////////////////////////////
//     unsigned shaderProgram = ShaderManager::instance().getShader(
//         "shaders/boid2d_vert.glsl", "shaders/boid2d_frag.glsl" );

//     float verts[] = {
//         0.5f,   0.0f,   0.0f, // top right
//         -0.25f, -0.25f, 0.0f, // bottom right
//         -0.25f, 0.25f,  0.0f  // top left
//     };
//     // unsigned int indices[] = {
//     //     // note that we start from 0!
//     //     0, 1, 2, // first Triangle
//     // };

//     unsigned int VBO, VAO, EBO;
//     glGenVertexArrays( 1, &VAO );
//     glGenBuffers( 1, &VBO );
//     // glGenBuffers( 1, &EBO );
//     glBindVertexArray( VAO );

//     glBindBuffer( GL_ARRAY_BUFFER, VBO );
//     glBufferData( GL_ARRAY_BUFFER, sizeof( verts ), verts, GL_STATIC_DRAW );

//     // glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, EBO );
//     // glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( indices ), indices,
//     //               GL_STATIC_DRAW );

//     glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof( float ),
//                            ( void* )0 );
//     glEnableVertexAttribArray( 0 );

//     glBindBuffer( GL_ARRAY_BUFFER, 0 );
//     glBindVertexArray( 0 );
//     ///////////////////////////////////////////////////////////

//     while ( !glfwWindowShouldClose( Window ) ) {
//         processInput( Window );

//         glClearColor( 0.2f, 0.3f, 0.3f, 1.f );
//         glClear( GL_COLOR_BUFFER_BIT );

//         glUseProgram( shaderProgram );
//         glBindVertexArray( VAO );
//         glDrawElements( GL_TRIANGLES, sizeof( verts ), GL_UNSIGNED_INT, 0 );

//         glfwSwapBuffers( Window );
//         glfwPollEvents();
//     }

//     glfwTerminate();
//     return 0;
// }

// void processInput( GLFWwindow* Window ) {
//     if ( glfwGetKey( Window, GLFW_KEY_ESCAPE ) == GLFW_PRESS )
//         glfwSetWindowShouldClose( Window, true );
// }

// void frameBufferSizeCallback( GLFWwindow*, int Width, int Height ) {
//     glViewport( 0, 0, Width, Height );
// }

#include "crash_handler.hpp"
#include "engine.hpp"

int main( int, char** ) {
    setupDump();

    const bool Result = Engine::instance().initialize();
    if ( !Result ) {
        return EXIT_FAILURE;
    }

    // Main update loop
    Engine::instance().update();

    Engine::instance().shutdown();

    return 0;
}
