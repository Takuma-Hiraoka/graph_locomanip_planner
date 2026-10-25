cmake_minimum_required(VERSION 3.16)
project(graph_locomanip_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

set(dependencies
  Eigen3
  pinocchio
  graph_search
  ik_constraint
  ik_constraint_esdf
  ik_constraint_sp
  global_inverse_kinematics_solver
  prioritized_inverse_kinematics_solver
  mujoco_viewer
  Threads)

foreach(dependency ${dependencies})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME}
  src/contact_state.cpp
  src/object_support.cpp
  src/graph_locomanip_planner.cpp
  src/util.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen pinocchio::pinocchio Threads::Threads)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.h")

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()
