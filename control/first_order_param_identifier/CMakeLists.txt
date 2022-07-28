cmake_minimum_required(VERSION 3.10)
project(first_order_param_identifier)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
        ${${PROJECT_NAME}_BUILD_DEPENDS}
        ${${PROJECT_NAME}_BUILDTOOL_DEPENDS})


if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
            -Wall
            -Wextra
            -Wpedantic
            -Wno-unused-parameter
            -Wno-unused-variable
            -Wno-conversion
            -Wno-sign-conversion
            -Wdouble-promotion
            # -Wfloat-equal
            -Waddress
            -fvisibility=default) # visibility hidden or default is set here.
endif ()

## Release, Debug, RelWithDebInfo, MinSizeRel
#set(CMAKE_BUILD_TYPE "Release")
#
#if (CMAKE_BUILD_TYPE MATCHES RELEASE)
#    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
#    message("In release mode ...")
#endif ()


## ------------------ PARAM ID CORE LIB ------------------------------------

set(PARAM_ID_LIB_HEADERS
        include/param_id_core.hpp
        )

set(PARAM_ID_LIB_SOURCE
        src/param_id_core.cpp
        )

set(PARAM_ID_LIB_INCLUDE_DIRS
        include)

set(PARAM_ID_LIB first_order_param_identifier_lib)

ament_auto_add_library(${PARAM_ID_LIB} SHARED
        ${PARAM_ID_LIB_SOURCE}
        ${PARAM_ID_LIB_HEADERS}
        )

target_compile_options(${PARAM_ID_LIB} PRIVATE -Wno-error=old-style-cast)

target_link_libraries(${PARAM_ID_LIB}
        ${EIGEN3_LIBRARIES})

install(DIRECTORY include
        DESTINATION include)

install(TARGETS ${PARAM_ID_LIB}
        EXPORT ${PARAM_ID_LIB}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION
        INCLUDES DESTINATION include
        )


## -----------------  PARAM ID NODE ----------------------------------------
set(PARAM_ID_NODE first_order_param_identifier_node)
ament_auto_add_library(${PARAM_ID_NODE} SHARED
        include/node_definitions.hpp
        include/param_id_node.hpp
        src/param_id_node.cpp
        )

target_link_libraries(${PARAM_ID_NODE}
        ${PARAM_ID_LIB})

rclcpp_components_register_node(${PARAM_ID_NODE}
        PLUGIN "sysid::ParameterIdentificationNode"
        EXECUTABLE ${PARAM_ID_NODE}_exe
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

