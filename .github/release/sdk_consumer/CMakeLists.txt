# Release check of the C/C++ SDK zip: a separate CMake project finds the unpacked SDK with find_package, builds two
# programs (plain C99) against iai_actuator_driver::shared with the runner's own compiler (MSVC on Windows, gcc on
# Linux) and runs them through ctest:
#   c_smoke      core/tests/c_smoke.c, the live program of this commit
#   c_smoke_1_0  core/abi/1.0/c_smoke.c, the frozen program of ABI 1.0, built with the 1.0 header beside it (not the
#                SDK's): a C program built for 1.0 must still run on this library
# Simulator only: no serial port is opened.
#
#   cmake -S .github/release/sdk_consumer -B build/sdk_consumer -DCMAKE_PREFIX_PATH=<unpacked SDK>
#         -DIAD_SMOKE_SOURCE=<repo>/core/tests/c_smoke.c -DIAD_FROZEN_SMOKE_SOURCE=<repo>/core/abi/1.0/c_smoke.c
#   cmake --build build/sdk_consumer --config Release
#   ctest --test-dir build/sdk_consumer -C Release --output-on-failure

cmake_minimum_required(VERSION 3.21)
project(iad_sdk_consumer LANGUAGES C)

set(IAD_SMOKE_SOURCE "" CACHE FILEPATH "core/tests/c_smoke.c")
if(NOT EXISTS "${IAD_SMOKE_SOURCE}")
    message(FATAL_ERROR "IAD_SMOKE_SOURCE (core/tests/c_smoke.c) not found: '${IAD_SMOKE_SOURCE}'")
endif()
set(IAD_FROZEN_SMOKE_SOURCE "" CACHE FILEPATH "core/abi/1.0/c_smoke.c (iai_actuator_driver.h of ABI 1.0 beside it)")
if(NOT EXISTS "${IAD_FROZEN_SMOKE_SOURCE}")
    message(FATAL_ERROR "IAD_FROZEN_SMOKE_SOURCE (core/abi/1.0/c_smoke.c) not found: '${IAD_FROZEN_SMOKE_SOURCE}'")
endif()
get_filename_component(frozen_dir "${IAD_FROZEN_SMOKE_SOURCE}" DIRECTORY)
if(NOT EXISTS "${frozen_dir}/iai_actuator_driver.h")
    message(FATAL_ERROR "the frozen 1.0 header is not beside IAD_FROZEN_SMOKE_SOURCE: ${frozen_dir}/iai_actuator_driver.h")
endif()

find_package(iai_actuator_driver CONFIG REQUIRED)
message(STATUS "iai_actuator_driver SDK ${iai_actuator_driver_VERSION_STRING} (${iai_actuator_driver_RID}): "
               "${iai_actuator_driver_RUNTIME_FILE}")

enable_testing()

function(iad_smoke name source)
    add_executable(${name} "${source}")
    set_target_properties(${name} PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
    target_link_libraries(${name} PRIVATE iai_actuator_driver::shared)
    if(UNIX)
        target_link_libraries(${name} PRIVATE m)
    endif()
    if(WIN32)
        # the DLL next to the program, the way the SDK's config file tells customers to do it
        add_custom_command(TARGET ${name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_RUNTIME_DLLS:${name}> $<TARGET_FILE_DIR:${name}>
            COMMAND_EXPAND_LISTS)
    endif()
    add_test(NAME ${name} COMMAND ${name})
endfunction()

iad_smoke(c_smoke "${IAD_SMOKE_SOURCE}")
iad_smoke(c_smoke_1_0 "${IAD_FROZEN_SMOKE_SOURCE}")
# the frozen header first, ahead of the SDK's include/ that iai_actuator_driver::shared adds (a quoted include finds
# the file beside the source first anyway; this keeps it so for every compiler)
target_include_directories(c_smoke_1_0 BEFORE PRIVATE "${frozen_dir}")
