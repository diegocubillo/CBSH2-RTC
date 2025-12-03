project(cbs)

cmake_minimum_required (VERSION 3.5)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories("inc")

# Get all source files except driver.cpp
file(GLOB LIB_SOURCES "src/*.cpp")
list(REMOVE_ITEM LIB_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/driver.cpp")

# Create shared library
add_library(cbs_lib SHARED ${LIB_SOURCES})

# Create executable (optional, for testing)
add_executable(cbs src/driver.cpp)

# Find Boost
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)

# Configure library
target_include_directories(cbs_lib PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
    $<INSTALL_INTERFACE:inc>
)
target_include_directories(cbs_lib PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(cbs_lib ${Boost_LIBRARIES})

# Configure executable
target_include_directories(cbs PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(cbs cbs_lib ${Boost_LIBRARIES})

# Install library and headers
install(TARGETS cbs_lib
    EXPORT cbs_lib-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY inc/ DESTINATION include/cbs_lib)

install(EXPORT cbs_lib-targets
    FILE cbs_lib-targets.cmake
    DESTINATION lib/cmake/cbs_lib
)

# Create config file for find_package
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    cbs_lib-config-version.cmake
    VERSION 1.0.0
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/cbs_lib-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/cbs_lib-config.cmake
    INSTALL_DESTINATION lib/cmake/cbs_lib
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/cbs_lib-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/cbs_lib-config-version.cmake
    DESTINATION lib/cmake/cbs_lib
)

# Add tests subdirectory
add_subdirectory(tests)

