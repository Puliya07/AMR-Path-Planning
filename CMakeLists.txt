cmake_minimum_required(VERSION 3.10)

# Project name and version
project(AMRPathPlanning VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add compiler flags for optimization and warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")

# Add debug flags for debug builds
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -DDEBUG")

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Include directories
include_directories(src)

# Source files
set(SOURCES
    src/main.cpp
    src/path_planning.cpp
    src/motor_control.cpp
)

# Header files
set(HEADERS
    src/path_planning.h
    src/motor_control.h
)

# Create executable
add_executable(amr_pathfinder ${SOURCES} ${HEADERS})

# Link math library (required for some systems)
target_link_libraries(amr_pathfinder m)

# Add custom target to create output directory
add_custom_target(create_output_dir ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/output
)

# Add dependency
add_dependencies(amr_pathfinder create_output_dir)

# Install targets
install(TARGETS amr_pathfinder
    RUNTIME DESTINATION bin
)

# Install headers for library usage
install(FILES ${HEADERS}
    DESTINATION include/amr_pathfinding
)

# Package configuration
set(CPACK_PACKAGE_NAME "AMRPathPlanning")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Autonomous Mobile Robot Path Planning System")
set(CPACK_PACKAGE_VENDOR "AMR Robotics")
set(CPACK_PACKAGE_CONTACT "pulithathiyasara@gmail.com")

# Include CPack for packaging
include(CPack)

# Enable testing
enable_testing()

# Add custom test (if test files exist)
# add_test(NAME PathPlanningTest COMMAND amr_pathfinder)

# Print build information
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "C++ flags: ${CMAKE_CXX_FLAGS}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")

# Optional: Create a library version as well
add_library(amr_pathfinding_lib STATIC 
    src/path_planning.cpp
    src/motor_control.cpp
)

target_include_directories(amr_pathfinding_lib PUBLIC src)

# Install the library
install(TARGETS amr_pathfinding_lib
    ARCHIVE DESTINATION lib
)