cmake_minimum_required(VERSION 3.5)
project(RobotKinematicsModel VERSION 1.0.0 LANGUAGES C)

# Создание библиотеки
add_library(${PROJECT_NAME}
  STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/model.h
  ${CMAKE_CURRENT_SOURCE_DIR}/model.c
)

# Установка пути к заголовочным файлам
target_include_directories(
  ${PROJECT_NAME} 
  PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Установка свойств библиотеки
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

target_compile_features(
  ${PROJECT_NAME}
  PRIVATE
  c_std_11
)

# Включение оптимизаций компилятора
target_compile_options(
  ${PROJECT_NAME}
  PRIVATE
  -O3
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

# Линковка с необходимыми библиотеками
target_link_libraries(
  ${PROJECT_NAME}
  PRIVATE
  m
)
