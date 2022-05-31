cmake_minimum_required(VERSION 3.10)
project(delay_compensation)

# Example https://github.com/tier4/autoware.universe/blob/b6beeece906024794c708c1d4dc6051cd5e270fc/control/trajectory_follower/CMakeLists.txt#L105
# require that dependencies from package.xml be available

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif ()


#set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Clang tidy
if (TIDY_WITH_CLANG)
    string(
            CONCAT CMAKE_CXX_CLANG_TIDY
            "clang-tidy;"
            "-checks=-*,"
            "bugprone-*,"
            "cert-*,"
            "cppcoreguidelines-*,"
            "clang-analyze-*,"
            "google-*,"
            "hicpp-*,"
            "modernize-*,"
            "performance-*,"
            "readability-*")
endif ()


if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
            -Wall
            -Wextra
            -Wpedantic
            # -Wno-unused-parameter
            # -Wconversion
            -Wdouble-promotion
            # -Wfloat-equal
            -Waddress
            -fvisibility=default # visibility hidden or default is set here.
    )
endif ()


## Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

##  ----------------- Required Packages ---------------------------------------
find_package(Eigen3 REQUIRED)

## ----------------- Library headers and sources -------------------------------
set(CDOB_LIB_INCLUDE_DIRS
        include)

set(CDOB_LIB delay_compensation_lib)

set(CDOB_LIB_SOURCE
        src/delay_observer.cpp
        src/utils_delay_observer/delay_compensation_utils.cpp
        src/qfilters.cpp
        src/utils_delay_observer/delay_compensation_utils.cpp
        src/vehicle_models/vehicle_kinematic_error_model.cpp

        # Integration
        src/integration/integrate_eigen_states.cpp
        )

set(CDOB_LIB_HEADERS
        include/delay_observer.hpp
        include/visibility_control.hpp
        include/utils_delay_observer/delay_compensation_utils.hpp
        include/qfilters.hpp
        include/utils_delay_observer/delay_compensation_utils.hpp
        include/vehicle_models/vehicle_definitions.hpp
        include/vehicle_models/vehicle_kinematic_error_model.hpp

        #Integration
        include/external/eigen_integration_helper.hpp
        include/integration/integrate_eigen_states.hpp
        )

set(CDOB_LIB_INCLUDE_DIRS
        include)


ament_auto_add_library(${CDOB_LIB} SHARED
        ${CDOB_LIB_SOURCE}
        ${CDOB_LIB_HEADERS}
        )

target_link_libraries(${CDOB_LIB}
        ${EIGEN3_LIBRARIES})

install(DIRECTORY include
        DESTINATION include)

install(TARGETS ${CDOB_LIB}
        EXPORT ${CDOB_LIB}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION
        INCLUDES DESTINATION include
        )

autoware_set_compile_options(${CDOB_LIB})

## ----------------- Library Tests----------------------------------------------

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest REQUIRED)

    #    ament_add_gtest(
    #            communication_delay_observer_based_control_tests
    #            src/...
    #            test/src/...)
    #
    #    target_include_directories(communication_delay_observer_based_control_tests PRIVATE include test/include)
    #    target_link_libraries(mpc_nonlinear_tests communication_delay_observer_based_control_tests ...
    #            ... ${EIGEN3_LIBRARIES})
    #    ament_target_dependencies(${CDOB_TEST_NODES} fake_test_node)
    #    autoware_set_compile_options(${CDOB_TEST_NODES})
    #    target_link_libraries(${CDOB_TEST_NODES} ${LATLON_MUXER_NODE} ${LATERAL_CONTROLLER_NODE} ${LONGITUDINAL_CONTROLLER_NODE})

endif ()

# -------------  PROTOTYPES ____________________________
# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE)
# ament_auto_package(INSTALL_TO_SHARE config launch)


#target_compile_options(${CDOB_LIB} PUBLIC "-O0" PRIVATE -Wall)
#ament_auto_package(CONFIG_EXTRAS_POST "${CDOB_LIB}-extras.cmake")
# --------------- Other Installation Macros ------------

##  Usage Examples
ament_auto_add_executable(main_test_classes usage_examples/main_test_classes.cpp)
target_link_libraries(main_test_classes ${CDOB})

## Type erasure example
ament_auto_add_executable(main_type_erasure usage_examples/main_polymorphisim.cpp)
target_link_libraries(main_type_erasure ${CDOB})

## Dummy control signal
ament_auto_add_executable(main_dummy_signal usage_examples/main_dummy_signal.cpp)
target_link_libraries(main_type_erasure ${CDOB})

## ODE integration
ament_auto_add_executable(main_ode usage_examples/main_ode.cpp)
target_link_libraries(main_ode ${CDOB})