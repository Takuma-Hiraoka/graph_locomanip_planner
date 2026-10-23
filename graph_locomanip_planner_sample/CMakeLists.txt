cmake_minimum_required(VERSION 3.16)
project(graph_locomanip_planner_sample)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
find_package(ament_cmake REQUIRED)
set(dependencies
  ament_index_cpp
  graph_locomanip_planner
  ik_constraint
  ik_constraint_esdf
  ik_constraint_vclip
  mujoco_esdf
  mujoco_viewer
  pinocchio
  pinocchio_model_builder)
foreach(dependency ${dependencies})
  find_package(${dependency} REQUIRED)
endforeach()
add_library(${PROJECT_NAME}_common src/g1_common.cpp src/world_common.cpp)
target_include_directories(${PROJECT_NAME}_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_common ${dependencies})
target_link_libraries(${PROJECT_NAME}_common pinocchio::pinocchio)
add_executable(sample0 src/sample0.cpp)
ament_target_dependencies(sample0 ${dependencies})
target_link_libraries(sample0 ${PROJECT_NAME}_common pinocchio::pinocchio)
install(TARGETS ${PROJECT_NAME}_common EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS sample0 DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.h")
install(DIRECTORY unitree_ros DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_package()
