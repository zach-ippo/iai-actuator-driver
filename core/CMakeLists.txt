# iai_actuator_driver - unofficial C++ driver for IAI actuators over serial Modbus RTU. Not an IAI product.
#
# Standard C++17, no third-party dependencies. Windows (MSVC, MinGW, clang) and Linux (gcc, clang).
#
#   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
#   cmake --build build --config Release
#   ctest --test-dir build -C Release          # the simulator-only tests (never open a real port)
#
# Targets: iai_actuator_driver (static library), ec_tests (tests), iai-actuator-driver-cpp (command line), and with
# -DIAI_ACTUATOR_DRIVER_BUILD_SHARED=ON (or -DBUILD_SHARED_LIBS=ON) iai_actuator_driver_shared: the C ABI
# (include/iai_actuator_driver.h) as iai_actuator_driver.dll / libiai_actuator_driver.so / .dylib, with the C++ runtime
# linked statically where the toolchain allows it (see docs/dev/NATIVE_ABI.md).
# In your own CMake project: add_subdirectory(path/to/core) and target_link_libraries(your_app PRIVATE iai_actuator_driver).

cmake_minimum_required(VERSION 3.14)
# MSVC_RUNTIME_LIBRARY (the static CRT of the shared library) needs this policy before project(); the other targets
# keep the default runtime.
if(POLICY CMP0091)
    cmake_policy(SET CMP0091 NEW)
endif()
project(iai_actuator_driver VERSION 1.2.0 LANGUAGES CXX)

option(IAI_ACTUATOR_DRIVER_BUILD_TESTS "Build the ec_tests executable" ON)
option(IAI_ACTUATOR_DRIVER_BUILD_CLI "Build the iai-actuator-driver-cpp command line" ON)
if(BUILD_SHARED_LIBS)
    set(IAI_ACTUATOR_DRIVER_SHARED_DEFAULT ON)
else()
    set(IAI_ACTUATOR_DRIVER_SHARED_DEFAULT OFF)
endif()
option(IAI_ACTUATOR_DRIVER_BUILD_SHARED "Also build the C ABI as a shared library (iai_actuator_driver_shared)"
       ${IAI_ACTUATOR_DRIVER_SHARED_DEFAULT})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

file(GLOB IAI_ACTUATOR_DRIVER_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB IAI_ACTUATOR_DRIVER_HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/iai_actuator_driver/*.hpp")
list(APPEND IAI_ACTUATOR_DRIVER_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/iai_actuator_driver.h"
                                        "${CMAKE_CURRENT_SOURCE_DIR}/include/iai_actuator_driver_testing.h")

# The versions, read from their sources: the C ABI (include/iai_actuator_driver.h; the shared library's SOVERSION is the
# ABI major, not the product major), the testing header's own number and the core's version text (version.hpp; with a
# pre-release suffix, which project() cannot hold).
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/include/iai_actuator_driver.h" _iad_abi_lines
     REGEX "^#define IAD_ABI_VERSION_(MAJOR|MINOR) [0-9]+")
string(REGEX REPLACE ".*IAD_ABI_VERSION_MAJOR ([0-9]+).*" "\\1" IAD_ABI_VERSION_MAJOR "${_iad_abi_lines}")
string(REGEX REPLACE ".*IAD_ABI_VERSION_MINOR ([0-9]+).*" "\\1" IAD_ABI_VERSION_MINOR "${_iad_abi_lines}")
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/include/iai_actuator_driver_testing.h" _iad_testing_line
     REGEX "^#define IAD_TESTING_ABI_VERSION [0-9]+")
string(REGEX REPLACE ".*IAD_TESTING_ABI_VERSION ([0-9]+).*" "\\1" IAD_TESTING_ABI_VERSION "${_iad_testing_line}")
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/include/iai_actuator_driver/version.hpp" _iad_version_line
     REGEX "kVersion = \"[^\"]+\"")
string(REGEX REPLACE ".*kVersion = \"([^\"]+)\".*" "\\1" IAD_VERSION "${_iad_version_line}")
if(NOT IAD_ABI_VERSION_MAJOR MATCHES "^[0-9]+$" OR NOT IAD_ABI_VERSION_MINOR MATCHES "^[0-9]+$"
   OR NOT IAD_TESTING_ABI_VERSION MATCHES "^[0-9]+$" OR IAD_VERSION STREQUAL "")
    message(FATAL_ERROR "cannot read the ABI versions from include/*.h or kVersion from version.hpp")
endif()

add_library(iai_actuator_driver STATIC ${IAI_ACTUATOR_DRIVER_SOURCES} ${IAI_ACTUATOR_DRIVER_HEADERS})
add_library(iai_actuator_driver::iai_actuator_driver ALIAS iai_actuator_driver)
target_include_directories(iai_actuator_driver
    PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(iai_actuator_driver PUBLIC Threads::Threads)
if(WIN32)
    # winmm: timeBeginPeriod (1 ms sleeps while a port is open); setupapi: serial-port discovery (VID / PID)
    target_link_libraries(iai_actuator_driver PUBLIC winmm setupapi advapi32)
    target_compile_definitions(iai_actuator_driver PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()

if(MSVC)
    target_compile_options(iai_actuator_driver PRIVATE /W4 /utf-8 /permissive- /Zc:__cplusplus)
else()
    target_compile_options(iai_actuator_driver PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

# The C ABI as a shared library. A separate target: the static library above stays what the tests and the CLI link.
# Only the iad_ functions are exported (IAD_BUILD_SHARED makes IAD_API dllexport / visibility default; everything
# else is hidden). The C++ runtime is linked statically where the toolchain allows it, so the library needs no
# runtime of its own on the target PC: MSVC /MT, MinGW -static, Linux -static-libstdc++ -static-libgcc (macOS: the
# system libc++).
if(IAI_ACTUATOR_DRIVER_BUILD_SHARED)
    add_library(iai_actuator_driver_shared SHARED ${IAI_ACTUATOR_DRIVER_SOURCES} ${IAI_ACTUATOR_DRIVER_HEADERS})
    add_library(iai_actuator_driver::shared ALIAS iai_actuator_driver_shared)
    set_target_properties(iai_actuator_driver_shared PROPERTIES
        OUTPUT_NAME iai_actuator_driver
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN ON
        POSITION_INDEPENDENT_CODE ON
        VERSION ${PROJECT_VERSION}
        SOVERSION ${IAD_ABI_VERSION_MAJOR})
    target_include_directories(iai_actuator_driver_shared
        PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_compile_definitions(iai_actuator_driver_shared PRIVATE IAD_BUILD_SHARED)
    target_link_libraries(iai_actuator_driver_shared PRIVATE Threads::Threads)
    # Windows: a version resource with the core's version and the ABI versions (the file's Properties > Details)
    if(WIN32)
        include(CheckLanguage)
        check_language(RC)
        if(CMAKE_RC_COMPILER)
            enable_language(RC)
            string(REGEX MATCH "^[0-9]+\\.[0-9]+\\.[0-9]+" _iad_numbers "${IAD_VERSION}")
            string(REPLACE "." "," IAD_RC_VERSION "${_iad_numbers},0")
            set(IAD_ABI_VERSION "${IAD_ABI_VERSION_MAJOR}.${IAD_ABI_VERSION_MINOR}")
            configure_file("${CMAKE_CURRENT_SOURCE_DIR}/src/iai_actuator_driver.rc.in"
                           "${CMAKE_CURRENT_BINARY_DIR}/iai_actuator_driver.rc" @ONLY)
            target_sources(iai_actuator_driver_shared PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/iai_actuator_driver.rc")
        else()
            message(STATUS "iai_actuator_driver: no resource compiler; the DLL has no version resource")
        endif()
    endif()
    if(WIN32)
        target_link_libraries(iai_actuator_driver_shared PRIVATE winmm setupapi advapi32)
        target_compile_definitions(iai_actuator_driver_shared PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
    endif()
    if(MSVC)
        target_compile_options(iai_actuator_driver_shared PRIVATE /W4 /utf-8 /permissive- /Zc:__cplusplus)
        # the import library would otherwise be iai_actuator_driver.lib, the static library's name
        set_target_properties(iai_actuator_driver_shared PROPERTIES
            ARCHIVE_OUTPUT_NAME iai_actuator_driver_imp
            MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    else()
        target_compile_options(iai_actuator_driver_shared PRIVATE -Wall -Wextra -Wno-unused-parameter)
    endif()
    if(MINGW)
        target_link_options(iai_actuator_driver_shared PRIVATE -static)
    elseif(UNIX AND NOT APPLE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        # --exclude-libs keeps the static C++ runtime's symbols out of the library's exports
        target_link_options(iai_actuator_driver_shared PRIVATE -static-libstdc++ -static-libgcc -Wl,--exclude-libs,ALL)
    endif()
endif()

if(IAI_ACTUATOR_DRIVER_BUILD_CLI)
    file(GLOB IAI_ACTUATOR_DRIVER_CLI_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/cli/*.cpp")
    if(IAI_ACTUATOR_DRIVER_CLI_SOURCES)
        add_executable(iai-actuator-driver-cpp ${IAI_ACTUATOR_DRIVER_CLI_SOURCES})
        target_include_directories(iai-actuator-driver-cpp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/cli ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_link_libraries(iai-actuator-driver-cpp PRIVATE iai_actuator_driver)
    endif()
endif()

if(IAI_ACTUATOR_DRIVER_BUILD_TESTS)
    enable_testing()
    file(GLOB IAI_ACTUATOR_DRIVER_TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp")
    # the CLI tests call cli::run directly: every cli/*.cpp except main.cpp
    file(GLOB IAI_ACTUATOR_DRIVER_CLI_LIB_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/cli/*.cpp")
    list(FILTER IAI_ACTUATOR_DRIVER_CLI_LIB_SOURCES EXCLUDE REGEX ".*/main\\.cpp$")
    add_executable(ec_tests ${IAI_ACTUATOR_DRIVER_TEST_SOURCES} ${IAI_ACTUATOR_DRIVER_CLI_LIB_SOURCES})
    target_include_directories(ec_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/tests ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/cli)
    target_link_libraries(ec_tests PRIVATE iai_actuator_driver)
    add_test(NAME ec_tests COMMAND ec_tests --quiet)
    # The C smoke test of the C ABI: tests/c_smoke.c compiled as plain C99 against the shared library (simulator
    # only). The program and the library land in the same output directory, where the Windows loader finds the DLL.
    if(TARGET iai_actuator_driver_shared)
        enable_language(C)
        add_executable(iad_c_smoke tests/c_smoke.c)
        set_target_properties(iad_c_smoke PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
        target_link_libraries(iad_c_smoke PRIVATE iai_actuator_driver_shared)
        if(UNIX)
            target_link_libraries(iad_c_smoke PRIVATE m)
        endif()
        add_test(NAME iad_c_smoke COMMAND iad_c_smoke)
    endif()
endif()

include(GNUInstallDirs)
install(TARGETS iai_actuator_driver EXPORT iai_actuator_driverTargets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/iai_actuator_driver DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES include/iai_actuator_driver.h include/iai_actuator_driver_testing.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
if(TARGET iai_actuator_driver_shared)
    install(TARGETS iai_actuator_driver_shared EXPORT iai_actuator_driverTargets
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()
install(EXPORT iai_actuator_driverTargets NAMESPACE iai_actuator_driver:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/iai_actuator_driver)
if(TARGET iai-actuator-driver-cpp)
    install(TARGETS iai-actuator-driver-cpp RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
