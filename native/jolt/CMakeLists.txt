cmake_minimum_required(VERSION 3.20)
project(ka_jolt_bridge LANGUAGES CXX)

add_library(ka_jolt_bridge SHARED ka_jolt_bridge_stub.cpp)
target_compile_features(ka_jolt_bridge PRIVATE cxx_std_17)
target_compile_definitions(ka_jolt_bridge PRIVATE KA_PHYSICS_BRIDGE_EXPORTS)
set_target_properties(ka_jolt_bridge PROPERTIES
    OUTPUT_NAME "ka_jolt_bridge"
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
)
