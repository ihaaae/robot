# CMake integration for the Juxie SDK tree committed in this repository.
#
#   include(<repo>/cmake/juxie-sdk.cmake)      # or add the repo's cmake/ to CMAKE_MODULE_PATH
#   add_executable(my_app main.cpp)
#   target_link_libraries(my_app PRIVATE Juxie::SDK)
#
# That gives you the include directories, the link directory, the transitive dependency
# search path and an rpath, so the program links and runs without hand-written flags.
#
# The vendor libraries are aarch64 Linux. Build for the robot's board, either natively there
# or by pointing CMake at a cross toolchain:
#
#   cmake -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ \
#         -DCMAKE_SYSTEM_NAME=Linux -DCMAKE_SYSTEM_PROCESSOR=aarch64 ..
#
# Override JUXIE_SDK_ROOT to build against a different copy of the SDK tree.

set(JUXIE_SDK_ROOT
    "${CMAKE_CURRENT_LIST_DIR}/../vendor/sdk/dual-arm-app/0.6.4"
    CACHE PATH "Root of a Juxie SDK tree (the directory holding usr/)")

if(NOT EXISTS "${JUXIE_SDK_ROOT}/usr/lib/libjuxie_controller.so")
    message(FATAL_ERROR
        "JUXIE_SDK_ROOT does not look like a Juxie SDK tree: ${JUXIE_SDK_ROOT}\n"
        "Expected usr/lib/libjuxie_controller.so. Set JUXIE_SDK_ROOT to the directory "
        "that contains usr/.")
endif()

if(NOT TARGET Juxie::SDK)
    # Eigen is a header-only dependency of the public header and is not shipped with the SDK.
    find_package(Eigen3 QUIET NO_MODULE)
    if(TARGET Eigen3::Eigen)
        set(_juxie_eigen Eigen3::Eigen)
    elseif(EXISTS "/usr/include/eigen3/Eigen/Dense")
        add_library(juxie_eigen_headers INTERFACE)
        target_include_directories(juxie_eigen_headers INTERFACE "/usr/include/eigen3")
        set(_juxie_eigen juxie_eigen_headers)
    else()
        message(FATAL_ERROR
            "Eigen headers not found. Install them (for example libeigen3-dev) or set "
            "Eigen3_DIR.")
    endif()

    add_library(Juxie::SDK INTERFACE IMPORTED)
    target_include_directories(Juxie::SDK INTERFACE "${JUXIE_SDK_ROOT}/usr/include")
    target_link_directories(Juxie::SDK INTERFACE "${JUXIE_SDK_ROOT}/usr/lib")
    # libjuxie_controller.so pulls in libexecutor, libbot_servo, libbot_planner,
    # libbot_traj_planner and libbot_kinematics. Without rpath-link the linker cannot find
    # those transitively and reports "undefined reference", which reads like a missing header
    # but is not.
    #
    # At run time those same five libraries must be found too, and none of them carries an
    # RPATH of its own. --disable-new-dtags is what makes the build tree self-sufficient:
    # under the modern DT_RUNPATH tag the loader searches the executable's path only for the
    # libraries the executable itself names, so libjuxie_controller resolves but its own
    # dependency libexecutor does not, and the program dies with "libexecutor.so.3: cannot
    # open shared object file". The older DT_RPATH tag is searched transitively as well.
    # Measured both ways; see the comment on Juxie::SDK in docs/running-on-the-robot.md.
    target_link_options(Juxie::SDK INTERFACE
        "-Wl,-rpath-link,${JUXIE_SDK_ROOT}/usr/lib"
        "-Wl,--disable-new-dtags"
        "-Wl,-rpath,${JUXIE_SDK_ROOT}/usr/lib")
    target_link_libraries(Juxie::SDK INTERFACE juxie_controller ${_juxie_eigen})

    message(STATUS "Juxie::SDK -> ${JUXIE_SDK_ROOT}")
endif()
