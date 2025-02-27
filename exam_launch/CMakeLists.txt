cmake_minimum_required(VERSION 3.8)
project(exam_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pub_env_info.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
