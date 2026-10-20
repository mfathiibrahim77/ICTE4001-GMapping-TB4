cmake_minimum_required(VERSION 3.8)
project(tb4_gmapping_nav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz maps
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/slam_eval_recorder.py
  scripts/drive_waypoints.py
  scripts/analyse_run.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
