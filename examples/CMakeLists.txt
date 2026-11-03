# The C and C++ examples, built against the core in this repository (core/). Unofficial: not an IAI product.
#
#   cmake -S examples -B build/examples -DCMAKE_BUILD_TYPE=Release
#   cmake --build build/examples --config Release
#   build/examples/bin/first_move_c --sim          (Release/ in between with a multi-config generator)
#
# Targets: first_move_c (the C ABI through the shared library), first_move_cpp and multi_axis_cell_cpp (the C++ API
# through the static library). The programs and the shared library land in one folder (bin/), where the Windows loader
# finds the DLL. Also works as add_subdirectory(examples) from a project that already added core/.
cmake_minimum_required(VERSION 3.14)
project(iai_actuator_driver_examples LANGUAGES C CXX)

if(NOT TARGET iai_actuator_driver)
    if(NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    endif()
    set(IAI_ACTUATOR_DRIVER_BUILD_TESTS OFF)
    set(IAI_ACTUATOR_DRIVER_BUILD_CLI OFF)
    set(IAI_ACTUATOR_DRIVER_BUILD_SHARED ON)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../core ${CMAKE_CURRENT_BINARY_DIR}/core)
endif()

# C: the C ABI (include/iai_actuator_driver.h) through the shared library.
if(TARGET iai_actuator_driver_shared)
    add_executable(first_move_c c/first_move.c)
    set_target_properties(first_move_c PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
    target_link_libraries(first_move_c PRIVATE iai_actuator_driver_shared)
    if(UNIX)
        find_package(Threads REQUIRED)
        target_link_libraries(first_move_c PRIVATE Threads::Threads m)
    endif()
endif()

# C++: the C++ API through the static library.
foreach(example IN ITEMS first_move multi_axis_cell)
    add_executable(${example}_cpp cpp/${example}.cpp)
    set_target_properties(${example}_cpp PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
    target_link_libraries(${example}_cpp PRIVATE iai_actuator_driver)
    if(MSVC)
        target_compile_options(${example}_cpp PRIVATE /utf-8)
    endif()
endforeach()
