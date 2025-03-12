
#ifndef INPUT_HPP
#define INPUT_HPP
#pragma once

// std includes
#include <vector>
#include <unordered_map>

// System includes
#include <glm/glm.hpp>

struct GLFWwindow;

struct Mouse {
    glm::dvec2 pos{ 0.0, 0.0 };
    glm::dvec2 p_pos{ 0.0, 0.0 };
};

class Input {
public:
    void update();

    Mouse& getMouse();
    glm::dvec2 getMousePosDx();

    static void closeWindowCallback( GLFWwindow* Window );

    static Input& instance();

    void addWASDCallback( std::function< void( glm::vec3 ) > );
    void addArrowCallback( std::function< void( glm::vec2 ) > );

    template < typename TCallback >
    inline void addCallback( int Key, TCallback&& Callback ) {
        KeyMap[Key].push_back( Callback );
    }

private:
    Input();

    void updateMouse( GLFWwindow* Window );

    std::vector< std::function< void( glm::vec3 ) > > WasdCallbacks;
    std::vector< std::function< void( glm::vec2 ) > > ArrowCallbacks;

    std::unordered_map< int, std::vector< std::function< void() > > > KeyMap;

    Mouse Mouse;
};

#endif
