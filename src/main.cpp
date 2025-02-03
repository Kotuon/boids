
#include <chrono>

#include "raylib.h"

#include <fmt/core.h>

#include "crash_handler.hpp"
#include "profiler.hpp"
#include "time_manager.hpp"
#include "trace.hpp"

constexpr int WIDTH = 1280;
constexpr int HEIGHT = 720;

#include "boid.hpp"
#include "boid_manager.hpp"

int main( int, char** ) {
    setupDump();

    InitWindow( WIDTH, HEIGHT, "basic window" );

    TimeManager Time;

    Profiler Profiler;

    BoidManager BoidManagerInstance( Vector2(
        static_cast< float >( WIDTH ), static_cast< float >( HEIGHT ) ) );

    while ( !WindowShouldClose() ) {
        Time.update();

        SetWindowTitle( fmt::format( "basic window: FPS: {:0.2f}",
                                     1.f / Time.getDeltaTime() )
                            .c_str() );

        while ( Time.needsFixedUpdate() ) {
            // Fixed update here
            BoidManagerInstance.update();
        }

        // Frame update here

        BeginDrawing();
        ClearBackground( DARKGRAY );

        // Draw here
        BoidManagerInstance.draw();

        EndDrawing();
    }
    CloseWindow();

    return 0;
}
