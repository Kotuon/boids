
#ifndef ENGINE_HPP
#define ENGINE_HPP
#pragma once

// std includes
#include <chrono>
#include <functional>
#include <vector>

using namespace std::chrono;

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
        fixed_update_callbacks.insert( fixed_update_callbacks.begin(), Callback );
    }

    template < typename TCallback >
    inline void addUpdateCallback( TCallback&& Callback ) {
        update_callbacks.insert( update_callbacks.begin(), Callback );
    }

    static Engine& instance();

private:
    Engine();

    steady_clock::time_point last_time; //!< last update time
    steady_clock::time_point curr_time; //!< new update time
    steady_clock::duration time_taken;  //!< time between frames

    std::vector< std::function< void() > > update_callbacks;
    std::vector< std::function< void() > > fixed_update_callbacks;

    float delta_time;                                //!< time between frames
    float accumulator;                               //!< amount of unused time for physics update
    float time;                                      //!< total time engine is running
    static constexpr float fixed_time_step{ 0.01f }; //!< fixed time step for physics update
    bool is_running;                                 //!< if main loop is running
};

#endif
