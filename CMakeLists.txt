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

#
# GLFW options
#
option(GLFW_INSTALL OFF)
option(GLFW_BUILD_DOCS OFF)
option(GLFW_BUILD_EXAMPLES OFF)
option(GLFW_BUILD_TESTS OFF)

#
# Libraries with CMake
#
add_subdirectory(libraries/fmt)
add_subdirectory(libraries/traceSystem)
add_subdirectory(libraries/crashHandler)
add_subdirectory(libraries/profiler)
add_subdirectory(libraries/timeManager)
add_subdirectory(libraries/threadPool)
add_subdirectory(libraries/glfw)

#
# Source files
#
file(GLOB_RECURSE LIBRARY_SOURCES
    libraries/glad/src/glad.c
    libraries/imgui/*.cpp
)
file(GLOB_RECURSE PROJECT_SOURCES
    src/*.cpp
    src/*.cxx
    src/*.cc
    src/*.c
)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES} ${LIBRARY_SOURCES})

include_directories(include/
    SYSTEM libraries/fmt/include
    SYSTEM libraries/glm
    SYSTEM libraries/eigen
    SYSTEM libraries/traceSystem/include
    SYSTEM libraries/crashHandler/include
    SYSTEM libraries/profiler/include
    SYSTEM libraries/timeManager/include
    SYSTEM libraries/threadPool/include
    SYSTEM libraries/imgui
    SYSTEM libraries/glad/include
    SYSTEM libraries/glfw/include
    SYSTEM libraries/glm/include
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
    glfw
    ${GLFW_LIBRARIES}
    ${GLAD_LIBRARIES}
)

# Make main find the <raylib.h> header (and others)
target_include_directories(${PROJECT_NAME} PUBLIC "${raylib_SOURCE_DIR}/src")

if(EMSCRIPTEN)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lidbfs.js -s USE_GLFW=3 --shell-file ${CMAKE_CURRENT_LIST_DIR}/web/minshell.html --preload-file ${CMAKE_CURRENT_LIST_DIR}/resources/@resources/ -s GL_ENABLE_GET_PROC_ADDRESS=1")
    set(CMAKE_EXECUTABLE_SUFFIX ".html") # This line is used to set your executable to build with the emscripten html template so that you can directly open it.
endif()
