
#ifndef EDITOR_HPP
#define EDITOR_HPP
#pragma once

// std includes
#include <functional>
#include <vector>

class Editor {
public:
    bool initialize( void* Window );
    void shutdown();
    void update();

    void render();

    template < typename TCallback >
    inline void addDisplayMenuCallback( TCallback&& Callback ) {
        DisplayMenuCallbacks.insert( DisplayMenuCallbacks.begin(), Callback );
    }

    static void helpMarker( const char* Desc );

    static Editor& instance();

private:
    Editor();

    void displayDockspace();
    void displayMenuBar();

    std::vector< std::function< void() > > DisplayMenuCallbacks;

    bool is_open;
};

#endif
