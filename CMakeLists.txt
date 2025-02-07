cmake_minimum_required(VERSION 3.5.0)
cmake_path(GET CMAKE_CURRENT_SOURCE_DIR FILENAME ProjectName)
project(${ProjectName} VERSION 0.1.0 LANGUAGES C CXX)

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
add_subdirectory(libraries/timeManager)
add_subdirectory(libraries/threadPool)


# Try to find a locally installed raylib, but don't quit on fail
find_package(raylib 5.0 QUIET)

# This code downloads raylib into a directory called _deps and adds it as a subdirectory, compiling it with the program when running the build command
include(FetchContent)

if(NOT raylib_FOUND)
    FetchContent_Declare(
        raylib
        URL https://github.com/raysan5/raylib/archive/refs/tags/5.0.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP True # This option is not required but suppresses a warning
    )
    FetchContent_MakeAvailable(raylib)
endif()

# We don't want raylib's examples built. This option is picked up by raylib's CMakeLists.txt
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

file(GLOB_RECURSE PROJECT_SOURCES
    src/*.cpp
    src/*.cxx
    src/*.cc
    src/*.c
)

# file(GLOB PROJECT_HEADERS CONFIGURE_DEPENDS libraries/thread_pool/include/*.h)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})

include_directories(include/
    SYSTEM libraries/fmt/include
    SYSTEM libraries/glm
    SYSTEM libraries/eigen
    SYSTEM libraries/traceSystem/include
    SYSTEM libraries/crashHandler/include
    SYSTEM libraries/profiler/include
    SYSTEM libraries/timeManager/include
    SYSTEM libraries/threadPool/include
)

# Link raylib to main
target_link_libraries(${PROJECT_NAME}
    raylib
    fmt::fmt
    traceSystem
    crashHandler
    profiler
    timeManager
    threadPool
)

# Make main find the <raylib.h> header (and others)
target_include_directories(${PROJECT_NAME} PUBLIC "${raylib_SOURCE_DIR}/src")

if(EMSCRIPTEN)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lidbfs.js -s USE_GLFW=3 --shell-file ${CMAKE_CURRENT_LIST_DIR}/web/minshell.html --preload-file ${CMAKE_CURRENT_LIST_DIR}/resources/@resources/ -s GL_ENABLE_GET_PROC_ADDRESS=1")
    set(CMAKE_EXECUTABLE_SUFFIX ".html") # This line is used to set your executable to build with the emscripten html template so that you can directly open it.
endif()
