
#ifndef GRAPHICS_HPP
#define GRAPHICS_HPP
#pragma once

// std includes
#include <functional>
#include <vector>

// System headers
#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <string>
#include <memory>
#include <glm/glm.hpp>

class Model;

class Graphics {
public:
    bool initialize();
    void update();
    void shutdown();

    void drawNormal( Model* Model, glm::mat4& Matrix );

    GLFWwindow* getWindow() const;

    static void frameBufferSizeCallback( GLFWwindow* Window, int Width,
                                         int Height );
    static void cursorEnterCallback( GLFWwindow* Window, int Entered );

    static void GLFWErrorCallback( int Error, const char* Description );

    template < typename TCallback >
    inline void addRenderCallback( TCallback&& Callback ) {
        RenderCallbacks.push_back( Callback );
    }

    glm::mat4 getProjection();

    static Graphics& instance();

private:
    Graphics();

    std::vector< std::function< void() > > RenderCallbacks;

    GLFWwindow* Window;

    glm::mat4 Projection;
};

// Function for handling keypresses
void handleKeyboardInput( GLFWwindow* Window );

#endif
