cmake_minimum_required(VERSION 3.5)

# Add tests to the main project
option(BUILD_TESTS "Build unit tests" ON)

if(BUILD_TESTS)
    # Find Google Test
    find_package(GTest REQUIRED)
    include(GoogleTest)
    
    # Enable testing
    enable_testing()
    
    # Include directories
    include_directories(${CMAKE_SOURCE_DIR}/inc)
    include_directories(${GTEST_INCLUDE_DIRS})
    
    # Test data directory
    set(TEST_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test_data")
    add_definitions(-DTEST_DATA_DIR="${TEST_DATA_DIR}")
    
    # Create test executable
add_executable(cbs_tests
    main.cpp
    test_coordinates.cpp
    test_basic_planning.cpp
    test_configuration.cpp
    test_map_loading.cpp
    test_format.cpp
)    # Link libraries
    target_link_libraries(cbs_tests 
        cbs_lib
        ${GTEST_LIBRARIES}
        ${GTEST_MAIN_LIBRARIES}
        pthread
    )
    
    # Discover tests
    gtest_discover_tests(cbs_tests)
    
    # Copy test data
    add_custom_command(TARGET cbs_tests POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/test_data
        ${CMAKE_CURRENT_BINARY_DIR}/test_data
    )
endif()
