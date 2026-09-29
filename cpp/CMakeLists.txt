cmake_minimum_required(VERSION 3.16)
project(shensi_can CXX)

# L0 of the replacement controller SDK: the wire codec, the transport seam and the offline
# test bench. No dependencies beyond a C++17 compiler -- deliberately, so this builds on the
# robot's board, on an x86 workstation, and under qemu with nothing installed first.
#
#   cmake -S cpp -B build && cmake --build build && ctest --test-dir build --output-on-failure
#   ./cpp/build.sh          # the same thing, plus a fallback when cmake is absent
#
# Nothing here touches a bus, a robot or the vendor SDK.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(shensi_can STATIC
    src/frame.cpp
    src/wire.cpp
    src/trace.cpp
    src/transport.cpp
    src/fake_transport.cpp)
target_include_directories(shensi_can PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_compile_options(shensi_can PRIVATE -Wall -Wextra -Wpedantic)

option(SHENSI_CAN_BUILD_TESTS "Build the L0 tests" ON)

if(SHENSI_CAN_BUILD_TESTS)
    enable_testing()
    add_executable(shensi_can_tests
        tests/main.cpp
        tests/test_wire.cpp
        tests/test_trace.cpp
        tests/test_transport.cpp)
    target_link_libraries(shensi_can_tests PRIVATE shensi_can)
    target_include_directories(shensi_can_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/tests")
    target_compile_options(shensi_can_tests PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME shensi_can_tests COMMAND shensi_can_tests)
endif()
