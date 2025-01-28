
#include <chrono>

#include "raylib.h"

#include <fmt/core.h>

#include "crash_handler.hpp"
#include "profiler.hpp"

constexpr int WIDTH = 1280;
constexpr int HEIGHT = 720;

// Update loop
using namespace std::chrono;
steady_clock::time_point LastTime;
steady_clock::time_point CurrTime;
steady_clock::duration TimeTaken;
float Accumulator;
float Time;
float DeltaTime;
constexpr float FixedDt = 0.02f;

int main( int, char** ) {
    setupDump();

    InitWindow( WIDTH, HEIGHT, "basic window" );

    LastTime = steady_clock::now();
    Accumulator = 0.f;
    Time = 0.f;

    Profiler profiler;

    while ( !WindowShouldClose() ) {
        CurrTime = steady_clock::now();
        TimeTaken = CurrTime - LastTime;
        DeltaTime = static_cast< float >( TimeTaken.count() ) *
                    steady_clock::period::num / steady_clock::period::den;
        LastTime = CurrTime;
        Accumulator += DeltaTime;

        SetWindowTitle(
            fmt::format( "basic window: FPS: {:0.2f}", 1.f / DeltaTime )
                .c_str() );

        while ( Accumulator >= FixedDt ) {
            // Fixed update here
            Accumulator -= FixedDt;
            Time += FixedDt;
        }

        // Frame update here

        BeginDrawing();
        ClearBackground( DARKGRAY );

        EndDrawing();
    }
    CloseWindow();

    return 0;
}
