cmake_minimum_required(VERSION 3.21)
project(TrackerSim)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add source files
file(GLOB SOURCES
    "src/*.h"
    "src/*.cpp"
)

# Libraries
set(CMAKE_PREFIX_PATH "C:/vcpkg/installed/x64-windows/share")
find_package(FFTW3 CONFIG REQUIRED)

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE FFTW3::fftw3)
