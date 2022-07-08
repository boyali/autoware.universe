cmake_minimum_required(VERSION 3.10)
project(autoware_control_toolbox)

# FMT
find_package(fmt)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(CMAKE_CXX_STANDARD 17)

## ----------------- Library headers and sources -------------------------------
set(UTILS_SRC
	src/utils_act/act_utils.cpp
	src/utils_act/act_utils_eigen.cpp
	src/utils_act/timekeep.cpp
	src/utils_act/writetopath.cpp)

set(UTILS_INCLUDE_DIRS
	include/utils_act)

set(CONTROL_LIB autoware_control_toolbox_lib)

set(CONTROL_LIB_SOURCE
	src/utils_act/act_utils.cpp
	src/utils_act/act_utils_eigen.cpp
	src/autoware_control_toolbox.cpp
	src/control/tf_algebra.cpp
	src/control/state_space.cpp
	src/control/transfer_functions.cpp
	src/control/balance.cpp
	src/control/act_signal_builder.cpp
	)

set(CONTROL_LIB_HEADERS
	include/control/act_definitions.hpp
	include/autoware_control_toolbox.hpp
	include/utils_act/act_utils.hpp
	include/control/balance.hpp
	include/utils_act/act_utils_eigen.hpp
	include/visibility_control.hpp
	include/control/tf_algebra.hpp
	include/control/state_space.hpp
	include/control/transfer_functions.hpp
	include/control/act_signal_builder.hpp
	)

set(CONTROL_LIB_INCLUDE_DIRS
	include
	include/control
	${UTILS_INCLUDE_DIRS})


## ------------- SPLINES ----------------------------------

set(SPLINE_LIB_SOURCE
	src/splines/interpolating_spline_pcg.cpp
	src/splines/bsplines_smoother.cpp
	src/splines/bsplines_interpolator.cpp
	src/splines/bspline_interpolator_templated.cpp
	${UTILS_SRC}
	)

set(SPLINE_LIB_INCLUDE_DIR
	include/splines
	${UTILS_INCLUDE_DIRS})


add_library(spline_lib ${SPLINE_LIB_SOURCE})
target_link_libraries(spline_lib)
target_include_directories(spline_lib PUBLIC ${SPLINE_LIB_INCLUDE_DIR})

## ------------- Whole LIBRARY COMPILE --------------------

ament_auto_add_library(${CONTROL_LIB} SHARED
	${CONTROL_LIB_SOURCE}
	${CONTROL_LIB_HEADERS}
	${UTILS_SRC}
	)
target_compile_options(${CONTROL_LIB} PRIVATE -Wno-error=old-style-cast)

target_link_libraries(${CONTROL_LIB}
	${EIGEN3_LIBRARIES}
	${Boost_LIBRARIES})

install(DIRECTORY include
	DESTINATION include)

install(TARGETS ${CONTROL_LIB}
	EXPORT ${CONTROL_LIB}
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION
	INCLUDES DESTINATION include
	)

## ----------------- Library Tests----------------------------------------------

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
	find_package(ament_cmake_gtest REQUIRED)

endif()

#ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE)


# -------------------------  PROTOTYPES ____________________________

## CONTROL
ament_auto_add_executable(main_tf usage_examples/control/main_tf.cpp)
ament_auto_add_executable(main_pade usage_examples/control/main_pade.cpp)
ament_auto_add_executable(main_tf2ss usage_examples/control/main_tf2ss.cpp)
ament_auto_add_executable(main_tf_algebra usage_examples/control/main_tf_algebra.cpp)
ament_auto_add_executable(main_balance usage_examples/control/main_balance.cpp)
ament_auto_add_executable(main_signal_generator usage_examples/control/main_signal_generator.cpp)
#target_link_libraries(main_tf fmt::fmt stdc++fs ${AUTOWARE_CONTROL_TOOLBOX_LIB})
#target_link_directories(main_tf PUBLIC ${AUTOWARE_CONTROL_LIB_INCLUDE_DIR})
#target_include_directories(main_tf PRIVATE include include/utils)

## ---------------------- SPLINEs -----------------------------

add_executable(main_spline_pcg usage_examples/splines/main_spline_eigen_pcg.cpp)
target_link_libraries(main_spline_pcg spline_lib fmt::fmt stdc++fs)

add_executable(main_bspline_smoother usage_examples/splines/main_bspline_smoother.cpp)
target_link_libraries(main_bspline_smoother spline_lib fmt::fmt stdc++fs)

add_executable(main_bspline_interp usage_examples/splines/main_bspline_interpolator.cpp)
target_link_libraries(main_bspline_interp spline_lib fmt::fmt stdc++fs)

add_executable(main_bspline_interp_template usage_examples/splines/main_bspline_interpolator_templated.cpp)
target_link_libraries(main_bspline_interp_template spline_lib fmt::fmt stdc++fs)
