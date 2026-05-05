cmake_minimum_required(VERSION 3.8)
project(pb2025_nav_bringup)

find_package(ament_cmake_auto REQUIRED)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify#include "behaviortree_ros2/bt_topic_pub_action_node.hpp"
    ament_cmake_cpplint
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    launch
    map
    pcd
    config
    behavior_trees
    rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "install" EXCLUDE
  PATTERN "log" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

ament_auto_package()
