cmake_minimum_required(VERSION 3.12.0)
project(PhysSym VERSION 1.0.0)
cmake_policy(SET CMP0072 NEW)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# for Debug
set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Building in Debug mode")
else()
    message(STATUS "Building in Release mode")
endif()

# to autofetch glfw
include(FetchContent)

# Check if GLFW is already installed
find_package(glfw3 QUIET)

if ((NOT glfw3_FOUND) AND MINGW)
    message(WARNING "GLFW not found. Fetching and building GLFW locally. Its better to install GLFW system-wide first.")
    FetchContent_Declare(
        glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG 3.4
    )
    FetchContent_MakeAvailable(glfw)
else()
    message(STATUS "GLFW found.")
endif()

find_package(OpenGL REQUIRED)

set(Boost_MINIMUM_VERSION 1.58)
find_package(Boost 1.58 REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# Find all model subdirectory sources
file(GLOB_RECURSE MODEL_SOURCES
    "${PROJECT_SOURCE_DIR}/src/model/*.cpp"
)

# Find all view subdirectory sources
file(GLOB_RECURSE VIEW_SOURCES
    "${PROJECT_SOURCE_DIR}/src/view/*.cpp"
)

# Find all controller subdirectory sources
file(GLOB_RECURSE CONTROLLER_SOURCES
    "${PROJECT_SOURCE_DIR}/src/controller/*.cpp"
)

# Find all compute subdirectory sources
file(GLOB_RECURSE COMPUTE_SOURCES
    "${PROJECT_SOURCE_DIR}/src/compute/*.cpp"
)

add_executable(PhysSym
    ${PROJECT_SOURCE_DIR}/src/main.cpp
    ${PROJECT_SOURCE_DIR}/src/glad/glad.c
    ${MODEL_SOURCES}
    ${VIEW_SOURCES}
    ${CONTROLLER_SOURCES}
    ${COMPUTE_SOURCES})

target_link_libraries(PhysSym
    glfw
    OpenGL::GL
    ${CMAKE_DL_LIBS} 
    pthread
)
