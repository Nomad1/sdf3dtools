cmake_minimum_required(VERSION 3.15)

# On macOS, set compiler before project definition
if(APPLE)
    execute_process(
        COMMAND brew --prefix llvm
        OUTPUT_VARIABLE LLVM_PREFIX
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    
    if(NOT LLVM_PREFIX)
        message(FATAL_ERROR "LLVM not found. Please install it using: brew install llvm libomp")
    endif()

    set(CMAKE_C_COMPILER "${LLVM_PREFIX}/bin/clang")
    set(CMAKE_CXX_COMPILER "${LLVM_PREFIX}/bin/clang++")
    
    # Set OpenMP flags
    set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp")
    set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp")
    set(OpenMP_CXX_LIB_NAMES "omp")
    set(OpenMP_C_LIB_NAMES "omp")
    set(OpenMP_omp_LIBRARY "${LLVM_PREFIX}/lib/libomp.dylib")
    
    include_directories("${LLVM_PREFIX}/include")
    link_directories("${LLVM_PREFIX}/lib")
    
    # Set arm64 architecture for macOS
    set(CMAKE_OSX_ARCHITECTURES "arm64" CACHE STRING "Build architectures for macOS" FORCE)
endif()

project(process_model VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(glm REQUIRED)
find_package(assimp REQUIRED)
find_package(OpenMP REQUIRED)

# Add executable
add_executable(process_model
    main.cpp
    PreparedTriangle.cpp
    TriangleGrid.cpp
)

# Link libraries
target_link_libraries(process_model
    PRIVATE
    glm::glm
    assimp::assimp
    OpenMP::OpenMP_CXX
)

# Enable warnings and compiler options
if(MSVC)
    target_compile_options(process_model PRIVATE /W4)
else()
    target_compile_options(process_model PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Set build type specific flags
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(process_model PRIVATE DEBUG=1)
    if(MSVC)
        target_compile_options(process_model PRIVATE /Od /Zi)
    else()
        target_compile_options(process_model PRIVATE -O0 -g)
    endif()
else()
    target_compile_definitions(process_model PRIVATE NDEBUG=1)
    if(MSVC)
        target_compile_options(process_model PRIVATE /O2)
    else()
        target_compile_options(process_model PRIVATE -O3)
    endif()
endif()