# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Langtfelt
#
# Builds the reference host against the C ABI.
#
#   cargo build --release -p chipbreaker-abi
#   cmake -S examples/c -B target/example-c -DCHIPBREAKER_ROOT=.
#   cmake --build target/example-c
#
# CI runs exactly these three commands, so an example that stops compiling
# fails the build rather than waiting to be discovered by an evaluator.

cmake_minimum_required(VERSION 3.16)
project(chipbreaker_example CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Where the repository is, relative to wherever cmake was invoked from.
set(CHIPBREAKER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." CACHE PATH
    "Root of the chipbreaker repository")
get_filename_component(CHIPBREAKER_ROOT "${CHIPBREAKER_ROOT}" ABSOLUTE)

set(CHIPBREAKER_TARGET_DIR "${CHIPBREAKER_ROOT}/target/release" CACHE PATH
    "Where cargo put the library")

add_executable(host host.cpp)
target_include_directories(host PRIVATE "${CHIPBREAKER_ROOT}/include")

# The import library on Windows, the shared object elsewhere. Named explicitly
# rather than found by `find_library` so that a missing `cargo build` produces a
# message naming the command to run instead of "library not found".
if(WIN32)
    set(CHIPBREAKER_LIB "${CHIPBREAKER_TARGET_DIR}/chipbreaker.dll.lib")
elseif(APPLE)
    set(CHIPBREAKER_LIB "${CHIPBREAKER_TARGET_DIR}/libchipbreaker.dylib")
else()
    set(CHIPBREAKER_LIB "${CHIPBREAKER_TARGET_DIR}/libchipbreaker.so")
endif()

if(NOT EXISTS "${CHIPBREAKER_LIB}")
    message(FATAL_ERROR
        "${CHIPBREAKER_LIB} does not exist.\n"
        "Build it first:  cargo build --release -p chipbreaker-abi")
endif()

target_link_libraries(host PRIVATE "${CHIPBREAKER_LIB}")

# On Linux the shared object needs to be findable at run time. Setting rpath
# here means the example runs from anywhere without LD_LIBRARY_PATH, which is
# one fewer thing for a quick-start to explain.
if(UNIX AND NOT APPLE)
    set_target_properties(host PROPERTIES
        BUILD_RPATH "${CHIPBREAKER_TARGET_DIR}")
endif()
if(APPLE)
    set_target_properties(host PROPERTIES
        BUILD_RPATH "${CHIPBREAKER_TARGET_DIR}")
endif()

# Windows has no rpath, so the DLL has to sit beside the executable. Copying it
# here rather than explaining it in the quick-start: the first run of the
# reference host otherwise fails with "cannot open shared object file", which
# reads as a broken example rather than as a missing search path.
if(WIN32)
    add_custom_command(TARGET host POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CHIPBREAKER_TARGET_DIR}/chipbreaker.dll"
                "$<TARGET_FILE_DIR:host>"
        COMMENT "copying chipbreaker.dll beside the host")
endif()
