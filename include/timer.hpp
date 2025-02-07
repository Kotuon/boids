
#ifndef TIMER_HPP
#define TIMER_HPP
#pragma once

#include <chrono>
#include <string>
#include "trace.hpp"
#include <fmt/core.h>
#include <source_location>
#include <typeinfo>

struct Timer {
    std::chrono::time_point< std::chrono::steady_clock > Start;
    std::chrono::time_point< std::chrono::steady_clock > End;
    std::chrono::duration< double, std::micro > Duration;

    template < typename TCallback >
    void run( std::string Message, TCallback&& Callback ) {
        start();

        Callback();

        end( Message );
    }

    void start() { Start = std::chrono::steady_clock::now(); }

    void end( std::string Message ) {
        End = std::chrono::steady_clock::now();
        Duration = End - Start;
        Trace::message(
            fmt::format( "{:>24}: {:>6}", Message, Duration.count() ) );
    }
};

#endif
