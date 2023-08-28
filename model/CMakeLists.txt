cmake_minimum_required(VERSION 3.5)
project(RobotKinematicsModel VERSION 1.0.0 LANGUAGES C)

set(HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/model.h
)

set(SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/model.c
)

set(BUILD_FILES
  ${HEADERS}
  ${SOURCES}
)

add_library(${PROJECT_NAME}
  STATIC
  ${BUILD_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${CMAKE_PROJECT_NAME})

target_compile_features(${PROJECT_NAME} PRIVATE c_std_11)

target_compile_options(
  ${PROJECT_NAME}
  PRIVATE
  -Wall
  -Werror
  -Wextra
  -Wpedantic
  -fsanitize=address
)

target_link_libraries(
  ${PROJECT_NAME}
  PRIVATE
  -fsanitize=address
)