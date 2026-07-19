cmake_minimum_required(VERSION 3.20)
project(ka_jolt_bridge LANGUAGES CXX)

include(FetchContent)
set(JPH_BUILD_SAMPLES OFF CACHE BOOL "" FORCE)
set(JPH_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(JPH_BUILD_PERFORMANCE_TESTS OFF CACHE BOOL "" FORCE)
set(JPH_BUILD_VIEWER OFF CACHE BOOL "" FORCE)
set(JPH_BUILD_HELLO_WORLD OFF CACHE BOOL "" FORCE)
set(JPH_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(JPH_USE_DX12 OFF CACHE BOOL "" FORCE)
set(JPH_USE_VK OFF CACHE BOOL "" FORCE)
set(JPH_USE_MTL OFF CACHE BOOL "" FORCE)
set(JPH_USE_CPU_COMPUTE OFF CACHE BOOL "" FORCE)

set(KA_JOLT_SOURCE_DIR "" CACHE PATH "Optional local Jolt Physics 5.6.0 source checkout")
if(KA_JOLT_SOURCE_DIR)
    if(NOT EXISTS "${KA_JOLT_SOURCE_DIR}/Build/CMakeLists.txt")
        message(FATAL_ERROR "KA_JOLT_SOURCE_DIR must point to a Jolt Physics source checkout")
    endif()
    add_subdirectory("${KA_JOLT_SOURCE_DIR}/Build" "${CMAKE_BINARY_DIR}/jolt" EXCLUDE_FROM_ALL)
    set(KA_JOLT_INCLUDE_ROOT "${KA_JOLT_SOURCE_DIR}")
else()
    FetchContent_Declare(
        JoltPhysics
        GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
        GIT_TAG v5.6.0
        GIT_SHALLOW TRUE
        SOURCE_SUBDIR Build
    )
    FetchContent_MakeAvailable(JoltPhysics)
    set(KA_JOLT_INCLUDE_ROOT "${joltphysics_SOURCE_DIR}")
endif()

add_library(ka_jolt_bridge SHARED ka_jolt_bridge.cpp)
target_compile_features(ka_jolt_bridge PRIVATE cxx_std_17)
target_compile_definitions(ka_jolt_bridge PRIVATE KA_PHYSICS_BRIDGE_EXPORTS)
target_link_libraries(ka_jolt_bridge PRIVATE Jolt)
target_include_directories(ka_jolt_bridge PRIVATE "${KA_JOLT_INCLUDE_ROOT}")
set_target_properties(ka_jolt_bridge PROPERTIES
    OUTPUT_NAME "ka_jolt_bridge"
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    POSITION_INDEPENDENT_CODE ON
)
if(MSVC)
    target_compile_options(ka_jolt_bridge PRIVATE /EHsc)
else()
    target_compile_options(ka_jolt_bridge PRIVATE -fno-strict-aliasing)
endif()
