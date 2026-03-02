cmake_minimum_required(VERSION 3.10)

# Set the project name
project(MAPLE)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set the default build type to Release if none is provided
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build (Debug, Release, RelWithDebInfo, MinSizeRel)" FORCE)
endif()
# Specify the possible build types
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")

include_directories(include)
file(GLOB MAPLE_SOURCES "src/*.cpp")
list(REMOVE_ITEM MAPLE_SOURCES "${CMAKE_SOURCE_DIR}/src/main.cpp")
add_executable(maple ${MAPLE_SOURCES} "src/main.cpp")

# Find the NTCore package within allwpilib
# Set options to disable unnecessary components of allwpilib in NetworkTables
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(WITH_JAVA OFF CACHE BOOL "" FORCE)
set(WITH_JAVA_SOURCE OFF CACHE BOOL "" FORCE)
set(WITH_DOCS OFF CACHE BOOL "" FORCE)
set(WITH_CSCORE OFF CACHE BOOL "" FORCE)
set(WITH_NTCORE ON CACHE BOOL "" FORCE)  # Make sure ntcore is enabled
set(WITH_WPIMATH ON CACHE BOOL "" FORCE)
set(WITH_WPIUNITS ON CACHE BOOL "" FORCE)
set(WITH_WPILIB OFF CACHE BOOL "" FORCE)
set(WITH_EXAMPLES OFF CACHE BOOL "" FORCE)
set(WITH_TESTS OFF CACHE BOOL "" FORCE)
set(WITH_GUI OFF CACHE BOOL "" FORCE)
set(WITH_SIMULATION_MODULES OFF CACHE BOOL "" FORCE)
set(WITH_PROTOBUF OFF CACHE BOOL "" FORCE)
add_subdirectory(lib/allwpilib)

# Find OpenCV package
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Find Eigen
include_directories(lib/Eigen)

# Find Mongoose
add_library(mongoose STATIC lib/mongoose/mongoose.c)
target_include_directories(mongoose PUBLIC lib/mongoose)
target_compile_definitions(mongoose PUBLIC MG_MAX_RECV_SIZE=125829120) # 120 * 1024 * 1024, 120MB for .syrup update files
target_compile_definitions(mongoose PRIVATE MG_IO_SIZE=131072) # 128KB chunk sizes

# Recursively copy all web files (html, js, models) to build directory

set(WEB_FILE_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/web")
set(FMAP_FILE_SRC "${CMAKE_CURRENT_SOURCE_DIR}/fmap")

set(WEB_FILE_DST "${CMAKE_CURRENT_BINARY_DIR}")

file(COPY ${WEB_FILE_SRC} DESTINATION ${WEB_FILE_DST})
file(COPY ${FMAP_FILE_SRC} DESTINATION ${WEB_FILE_DST})
#file(GLOB_RECURSE WEB_FILES "${WEB_FILE_SRC}/*")
#
## Iterate over each file
#foreach(WEB_FILE ${WEB_FILES})
#    # Get the relative path of the file w.r.t the source directory
#    file(RELATIVE_PATH REL_PATH "${WEB_FILE_SRC}" "${WEB_FILE}")
#
#    # Determine the output path, preserving the directory structure
#    set(OUTPUT_FILE "${WEB_FILE_DST}/${REL_PATH}")
#
#    # Create the output directory structure if needed
#    get_filename_component(OUTPUT_DIR_FOR_FILE "${OUTPUT_FILE}" DIRECTORY)
#    file(MAKE_DIRECTORY "${OUTPUT_DIR_FOR_FILE}")
#
#    # Copy the file
#    configure_file(
#            "${CONFIG_FILE}" # Input file
#            "${OUTPUT_FILE}" # Output file
#    )
#endforeach()

#configure_file(src/web/index.html ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
#configure_file(src/web/js/three.module.js ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
#configure_file(src/web/js/three.core.js ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
#configure_file(src/web/js/OrbitControls.js ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
#configure_file(src/web/js/GLTFLoader.js ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
#configure_file(src/web/js/BufferGeometryUtils.js ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
#configure_file(src/web/js/quadcopter.gltf ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)

# Add the apriltag library, do not build examples or python wrapper
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_WRAPPER OFF CACHE BOOL "" FORCE)
add_subdirectory(lib/apriltag)

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/logs")

# Yaml parser
add_subdirectory(lib/yaml-cpp)

# Json parser
add_subdirectory(lib/json)

target_link_libraries(maple PUBLIC yaml-cpp::yaml-cpp ${OpenCV_LIBS} ntcore wpimath apriltag PRIVATE nlohmann_json::nlohmann_json mongoose)

# Option to enable or disable tests
option(ENABLE_TESTS "Enable building tests" OFF)

# Fetch GoogleTest if tests are enabled
if(ENABLE_TESTS)

    include(FetchContent)

    # Fetch GoogleTest
    FetchContent_Declare(
            googletest
            # Specify the commit you depend on and update it regularly.
            URL https://github.com/google/googletest/archive/5376968f6948923e2411081fd9372e71a59d8e77.zip
    )
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    # Add the test files
    file(GLOB TEST_SOURCES "test/*.cpp")

    # Create the test executable
    add_executable(runTests ${TEST_SOURCES} ${MAPLE_SOURCES})

    # Link the test executable against gtest and gtest_main
    target_link_libraries(runTests PUBLIC yaml-cpp::yaml-cpp gtest_main ${OpenCV_LIBS} ntcore PRIVATE nlohmann_json::nlohmann_json apriltag mongoose)

    # Enable testing
    enable_testing()

    # Add the tests to CTest
    add_test(NAME runTests COMMAND runTests)
endif()