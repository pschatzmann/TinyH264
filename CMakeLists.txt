# When this file is processed by the ESP-IDF build system (i.e. TinyH264 is
# used as an ESP-IDF component - either dropped into a project's components/
# dir or referenced via EXTRA_COMPONENT_DIRS), register it the IDF way and
# stop: a component CMakeLists.txt must not call project()/
# cmake_minimum_required() itself (that's the enclosing IDF project's job).
# ESP_PLATFORM is the variable ESP-IDF's build system sets for exactly this
# kind of branch - see the IDF docs, "Behind the Scenes" section:
# "The CMake variable ESP_PLATFORM is set to 1 whenever the ESP-IDF build
# system is being used. Tests such as if (ESP_PLATFORM) can be used in
# generic CMake code if special IDF-specific logic is required."
if(ESP_PLATFORM)
    idf_component_register(
        INCLUDE_DIRS "src" "src/common" "src/decoder" "src/encoder"
    )
    return()
endif()

cmake_minimum_required(VERSION 3.14)
project(TinyH264 CXX)

# TinyH264 is header-only (see src/) - the primary distribution mechanism
# is the Arduino Library Manager (library.properties), not CMake. This
# file exists for three things CMake is actually useful for: (1) letting a
# non-Arduino CMake-based project consume the library via
# add_subdirectory() + target_link_libraries(), (2) building/running the
# desktop native test suite (test/native/) - this project's correctness
# oracle, previously only runnable via manual g++ invocations (see
# README's "Testing" section), and (3) - handled by the ESP_PLATFORM
# branch above - being consumed as an ESP-IDF component. None of this is
# needed for, or affects, the Arduino/arduino-cli workflow.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Header-only library target: no .cpp files of its own, so this is an
# INTERFACE target - it only contributes an include path and a C++17
# requirement to whatever links against it.
add_library(TinyH264 INTERFACE)
add_library(TinyH264::TinyH264 ALIAS TinyH264)
target_include_directories(TinyH264 INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common
    ${CMAKE_CURRENT_SOURCE_DIR}/src/decoder
    ${CMAKE_CURRENT_SOURCE_DIR}/src/encoder
)
# Propagated to consumers too: the codebase actually requires C++17 (not
# just this build of it), so a project linking against TinyH264 without
# C++17 enabled should fail to configure rather than fail with confusing
# compiler errors deep inside a header.
target_compile_features(TinyH264 INTERFACE cxx_std_17)

# Only build the test suite when this is the top-level CMake project
# (i.e. not when some other project add_subdirectory()s this one just to
# get the TinyH264 target) - portable equivalent of CMake 3.21's
# PROJECT_IS_TOP_LEVEL, kept for compatibility with the wider range of
# CMake versions this cmake_minimum_required allows.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(TINYH264_IS_TOP_LEVEL TRUE)
else()
    set(TINYH264_IS_TOP_LEVEL FALSE)
endif()
option(TINYH264_BUILD_TESTS "Build the desktop native test suite (test/native/)" ${TINYH264_IS_TOP_LEVEL})

if(TINYH264_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test/native)
endif()
