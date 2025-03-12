
// System headers
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

// Local includes
#include "input.hpp"
#include "engine.hpp"
#include "graphics.hpp"

Input::Input() {}

void Input::update() {
    GLFWwindow* window = Graphics::instance().getWindow();

    updateMouse( window );

    // Use escape key for terminating the GLFW window
    if ( glfwGetKey( window, GLFW_KEY_ESCAPE ) == GLFW_PRESS ) {
        glfwSetWindowShouldClose( window, GL_TRUE );
        Engine::instance().triggerShutdown();
    }

    for ( auto key : KeyMap ) {
        if ( glfwGetKey( window, key.first ) != GLFW_PRESS ) {
            continue;
        }

        for ( auto func : key.second ) {
            func();
        }
    }

    glm::vec3 wasdInput{ 0.f };
    wasdInput.y =
        glfwGetKey( window, GLFW_KEY_W ) == GLFW_PRESS ? 1.f : wasdInput.y;
    wasdInput.y =
        glfwGetKey( window, GLFW_KEY_S ) == GLFW_PRESS ? -1.f : wasdInput.y;
    wasdInput.x =
        glfwGetKey( window, GLFW_KEY_D ) == GLFW_PRESS ? 1.f : wasdInput.x;
    wasdInput.x =
        glfwGetKey( window, GLFW_KEY_A ) == GLFW_PRESS ? -1.f : wasdInput.x;

    glm::vec2 arrowInput{ 0.f };
    arrowInput.y = glfwGetKey( window, GLFW_KEY_UP ) ? 1.f : arrowInput.y;
    arrowInput.y = glfwGetKey( window, GLFW_KEY_DOWN ) ? -1.f : arrowInput.y;
    arrowInput.x = glfwGetKey( window, GLFW_KEY_RIGHT ) ? 1.f : arrowInput.x;
    arrowInput.x = glfwGetKey( window, GLFW_KEY_LEFT ) ? -1.f : arrowInput.x;

    for ( auto func : WasdCallbacks ) {
        func( wasdInput );
    }

    for ( auto func : ArrowCallbacks ) {
        func( arrowInput );
    }
}

void Input::updateMouse( GLFWwindow* Window ) {
    Mouse.p_pos = Mouse.pos;

    glfwGetCursorPos( Window, &( Mouse.pos.x ), &( Mouse.pos.y ) );
}

Mouse& Input::getMouse() { return Mouse; }

glm::dvec2 Input::getMousePosDx() { return Mouse.pos - Mouse.p_pos; }

void Input::closeWindowCallback( GLFWwindow* Window ) {
    glfwSetWindowShouldClose( Window, GL_TRUE );
    Engine::instance().triggerShutdown();
}

Input& Input::instance() {
    static Input InputInstance;
    return InputInstance;
}

void Input::addWASDCallback( std::function< void( glm::vec3 ) > Func ) {
    WasdCallbacks.push_back( Func );
}

void Input::addArrowCallback( std::function< void( glm::vec2 ) > Func ) {
    ArrowCallbacks.push_back( Func );
}
