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

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Libraries
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/fftw3)

add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/fftw3/libfftw3-3.lib
    COMMAND lib.exe /machine:X64 /def:${CMAKE_CURRENT_SOURCE_DIR}/fftw3/libfftw3-3.def /out:${CMAKE_CURRENT_SOURCE_DIR}/fftw3/libfftw3-3.lib
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/fftw3/libfftw3-3.def
    VERBATIM
)
add_custom_target(fftw3_import ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/fftw3/libfftw3-3.lib)

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/fftw3/libfftw3-3.lib)

# Signal file
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_CURRENT_SOURCE_DIR}/gnss-20170427-L1.1bit.I.bin
    $<TARGET_FILE_DIR:${PROJECT_NAME}>/gnss-20170427-L1.1bit.I.bin)