cmake_minimum_required(VERSION 3.5)
project(mpc_acado)

set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp -O3")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()


set(ACADO_ENV_SHARED_LIBRARIES /home/hakim/Desktop/Phd/ros2_ws/src/mpc_acado/externals/ACADOtoolkit/build/lib/libacado_toolkit_s.so)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(ACADO REQUIRED )
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
#find_package(Boost REQUIRED COMPONENTS system)


add_library(mpc_solver
  externals/qpoases/SRC/Bounds.cpp
  externals/qpoases/SRC/Constraints.cpp
  externals/qpoases/SRC/CyclingManager.cpp
  externals/qpoases/SRC/Indexlist.cpp
  externals/qpoases/SRC/MessageHandling.cpp
  externals/qpoases/SRC/QProblem.cpp
  externals/qpoases/SRC/QProblemB.cpp
  externals/qpoases/SRC/SubjectTo.cpp
  externals/qpoases/SRC/Utils.cpp
  externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  model/codegen/nmpc_qpoases_interface.cpp
  model/codegen/nmpc_integrator.c
  model/codegen/nmpc_solver.c
  model/codegen/nmpc_auxiliary_functions.c)


target_include_directories(mpc_solver PUBLIC
model/codegen/
externals/qpoases
externals/qpoases/INCLUDE
externals/qpoases/SRC
externals/ACADOtoolkit/acado/code_generation
externals/ACADOtoolkit/acado
externals/ACADOtoolkit/


)

link_directories(${ACADO_ENV_LIBRARY_DIRS})

INCLUDE_DIRECTORIES( . ${ACADO_ENV_INCLUDE_DIRS} )

include_directories(
  include
  ${ACADO_ENV_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()




add_executable(code_gen_drone src/code_generation/code_gen_drone.cpp)
target_link_libraries(code_gen_drone  ${ACADO_ENV_SHARED_LIBRARIES} mpc_solver)
ament_target_dependencies(code_gen_drone  )
set_target_properties(code_gen_drone PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )

add_executable(mpc_acado_node src/mpc_acado_node.cpp)
target_link_libraries(mpc_acado_node mpc_solver ${ACADO_ENV_SHARED_LIBRARIES} yaml-cpp)
ament_target_dependencies(mpc_acado_node rclcpp std_msgs geometry_msgs nav_msgs px4_msgs  )

install(TARGETS
  # getting_started
  code_gen_drone
  mpc_acado_node
	DESTINATION lib/${PROJECT_NAME}
)

