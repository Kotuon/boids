cmake_minimum_required(VERSION 3.5.0)
project(cpp_template VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4 /std:c++latest")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -std=c++20")

    if(NOT WIN32)
        set(GLAD_LIBRARIES dl)
    endif()
endif()

add_subdirectory(libraries/fmt)
add_subdirectory(libraries/traceSystem)
add_subdirectory(libraries/crashHandler)
add_subdirectory(libraries/profiler)

file(GLOB_RECURSE PROJECT_SOURCES
    src/*.cpp
    src/*.cxx
    src/*.cc
    src/*.c
)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})

include_directories(src/
    SYSTEM libraries/fmt/include
    SYSTEM libraries/traceSystem/include
    SYSTEM libraries/crashHandler/include
    SYSTEM libraries/profiler/include
)

target_link_libraries(${PROJECT_NAME}
    fmt::fmt
    traceSystem
    crashHandler
    profiler
)
