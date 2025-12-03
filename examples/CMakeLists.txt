# Examples for CBS library usage
cmake_minimum_required(VERSION 3.5)
project(cbs_examples)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the CBS library - try local build first, then installed version
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../build/libcbs_lib.so")
    # Use local build
    add_library(cbs_lib SHARED IMPORTED)
    set_target_properties(cbs_lib PROPERTIES
        IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/../build/libcbs_lib.so"
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/../inc"
    )
else()
    # Use installed version
    find_package(cbs_lib REQUIRED)
    set(cbs_lib_target cbs_lib::cbs_lib)
endif()

# Basic usage example
add_executable(basic_usage basic_usage.cpp)
target_link_libraries(basic_usage cbs_lib)

# Advanced configuration example
add_executable(advanced_configuration advanced_configuration.cpp)
target_link_libraries(advanced_configuration cbs_lib)

# Install examples (optional)
install(TARGETS basic_usage advanced_configuration
    DESTINATION bin
)
