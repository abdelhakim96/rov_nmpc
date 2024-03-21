cmake_minimum_required(VERSION 3.5)
project(rov_mpc)

set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp -O3")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()


set(ACADO_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/externals/ACADOtoolkit/build/lib)


find_library(ACADO_ENV_SHARED_LIBRARIES
             NAMES acado_toolkit_s
             HINTS ${ACADO_LIBRARY_DIR})




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
find_package(ACADO REQUIRED )
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 3.0.1 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
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




add_executable(code_gen_rov src/code_generation/code_gen_rov.cpp)
target_link_libraries(code_gen_rov  ${ACADO_ENV_SHARED_LIBRARIES} mpc_solver)
ament_target_dependencies(code_gen_rov  )
set_target_properties(code_gen_rov PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )

add_executable(rov_mpc_node src/rov_mpc_node.cpp)
target_link_libraries(rov_mpc_node mpc_solver ${ACADO_ENV_SHARED_LIBRARIES} yaml-cpp)
ament_target_dependencies(rov_mpc_node rclcpp std_msgs geometry_msgs nav_msgs  )

install(TARGETS
  # getting_started
  code_gen_rov
  rov_mpc_node
	DESTINATION lib/${PROJECT_NAME}
)

