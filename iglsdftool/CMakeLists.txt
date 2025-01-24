cmake_minimum_required(VERSION 3.16)
project(sdf_generator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Option 1: If libigl is installed in your system
# set(LIBIGL_DIR "/path/to/libigl/installation" CACHE PATH "Path to libigl installation")
# list(APPEND CMAKE_MODULE_PATH ${LIBIGL_DIR}/cmake)

# Option 2: Include libigl as a submodule (recommended)
include(FetchContent)
FetchContent_Declare(
    libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG v2.4.0
)
FetchContent_MakeAvailable(libigl)

# Add your main executable
add_executable(${PROJECT_NAME} main.cpp utils.cpp)

# Add test executable
add_executable(winding_test test_libigl_winding.cpp)

# Link against libigl
target_link_libraries(${PROJECT_NAME} PRIVATE igl::core)
target_link_libraries(winding_test PRIVATE igl::core)

# Enable warnings and compiler options
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG=1)
    target_compile_definitions(winding_test PRIVATE DEBUG=1)
    if(MSVC)
        target_compile_options(${PROJECT_NAME} PRIVATE /Od /Zi)
        target_compile_options(winding_test PRIVATE /Od /Zi)
    else()
        target_compile_options(${PROJECT_NAME} PRIVATE -O0 -g)
        target_compile_options(winding_test PRIVATE -O0 -g)
    endif()
else()
    target_compile_definitions(${PROJECT_NAME} PRIVATE NDEBUG=1)
    target_compile_definitions(winding_test PRIVATE NDEBUG=1)
    if(MSVC)
        target_compile_options(${PROJECT_NAME} PRIVATE /O2)
        target_compile_options(winding_test PRIVATE /O2)
    else()
        target_compile_options(${PROJECT_NAME} PRIVATE -O3)
        target_compile_options(winding_test PRIVATE -O3)
    endif()
endif()