
#ifndef ENGINE_HPP
#define ENGINE_HPP
#pragma once

// std includes
#include <chrono>
#include <functional>
#include <vector>

#include "time_manager.hpp"

class Engine {
public:
    bool initialize();
    void update();
    void shutdown();

    float getDeltaTime() const;
    float getTotalTime() const;

    float getFixedTimeStep() const;

    void triggerShutdown();

    template < typename TCallback >
    inline void addFixedUpdateCallback( TCallback&& Callback ) {
        FixedUpdateCallbacks.insert( FixedUpdateCallbacks.begin(), Callback );
    }

    template < typename TCallback >
    inline void addUpdateCallback( TCallback&& Callback ) {
        UpdateCallbacks.insert( UpdateCallbacks.begin(), Callback );
    }

    static Engine& instance();

private:
    Engine();

    std::unique_ptr< TimeManager > Time;

    std::vector< std::function< void() > > UpdateCallbacks;
    std::vector< std::function< void() > > FixedUpdateCallbacks;

    bool IsRunning; //!< if main loop is running
};

#endif
