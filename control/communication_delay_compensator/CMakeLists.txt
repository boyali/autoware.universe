cmake_minimum_required(VERSION 3.10)
project(communication_delay_compensator)

find_package(autoware_cmake REQUIRED)
autoware_package()


## ----------------- Library headers and sources -------------------------------
set(CDOB_LIB_INCLUDE_DIRS
        include)

set(CDOB_LIB delay_compensation_lib)

set(CDOB_LIB_HEADERS
        include/communication_delay_compensator_core.hpp
        include/visibility_control.hpp
        include/utils_delay_observer/delay_compensation_utils.hpp
        include/qfilters.hpp
        include/utils_delay_observer/delay_compensation_utils.hpp
        include/vehicle_models/vehicle_definitions.hpp
        include/vehicle_models/vehicle_kinematic_error_model.hpp
        include/adaptive_parameter_estimators/adaptive_parameter_estimation.hpp

        #Integration
        include/external/eigen_integration_helper.hpp
        include/integration/integrate_eigen_states.hpp
        )


set(CDOB_LIB_SOURCE
        src/communication_delay_compensator_core.cpp
        src/utils_delay_observer/delay_compensation_utils.cpp
        src/qfilters.cpp
        src/utils_delay_observer/delay_compensation_utils.cpp
        src/vehicle_models/vehicle_kinematic_error_model.cpp
        src/adaptive_parameter_estimators/adaptive_parameter_estimation.cpp

        # Integration
        src/integration/integrate_eigen_states.cpp
        )


set(CDOB_LIB_INCLUDE_DIRS
        include)

ament_auto_add_library(${CDOB_LIB} SHARED
        ${CDOB_LIB_SOURCE}
        ${CDOB_LIB_HEADERS}
        )

target_compile_options(${CDOB_LIB} PRIVATE -Wno-error=old-style-cast)

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
## -----------------  DELAY COMPENSATION NODE ----------------------------------------
set(DELAY_COMPENSATION_NODE communication_delay_compensator)
ament_auto_add_library(${DELAY_COMPENSATION_NODE} SHARED
        include/communication_delay_compensator_node.hpp
        src/communication_delay_compensator_node.cpp
        )

target_link_libraries(${DELAY_COMPENSATION_NODE}
        ${CDOB})

rclcpp_components_register_node(${DELAY_COMPENSATION_NODE}
        PLUGIN "observers::CommunicationDelayCompensatorNode"
        EXECUTABLE ${DELAY_COMPENSATION_NODE}_exe
        )
## ----------------- Library Tests----------------------------------------------

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest REQUIRED)
endif ()

# -------------  PROTOTYPES ____________________________
ament_auto_package(INSTALL_TO_SHARE
        launch
        param)


##  Usage Examples
## Type erasure example
ament_auto_add_executable(main_type_erasure usage_examples/main_polymorphisim.cpp)
target_link_libraries(main_type_erasure ${CDOB})

## Dummy control signal
ament_auto_add_executable(main_qfilter_simulation usage_examples/main_qfilter_simulation.cpp)
target_link_libraries(main_qfilter_simulation ${CDOB})

ament_auto_add_executable(main_vehicle_simulation usage_examples/main_vehicle_simulation.cpp)
target_link_libraries(main_vehicle_simulation ${CDOB})

## ODE integration
ament_auto_add_executable(main_ode usage_examples/main_ode.cpp)
target_link_libraries(main_ode ${CDOB})

## ODE integration
ament_auto_add_executable(main_delay_compensator usage_examples/main_delay_compensator.cpp)
target_link_libraries(main_delay_compensator ${CDOB})

ament_auto_add_executable(main_dc_with_vehicle usage_examples/main_delay_compensator_with_vehicle.cpp)
target_link_libraries(main_dc_with_vehicle ${CDOB})