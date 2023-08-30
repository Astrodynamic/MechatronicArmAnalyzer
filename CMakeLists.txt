cmake_minimum_required(VERSION 3.10)
project(MechatronicArmAnalyzer  VERSION 1.0.0 LANGUAGES C)

add_executable(
  ${PROJECT_NAME}
  interface/interface.h
  interface/interface.c
  main.c
)

add_subdirectory(model)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/model
  interface
)

target_link_libraries(
  ${PROJECT_NAME}
  PRIVATE
  RobotKinematicsModel
)
