cmake_minimum_required(VERSION 3.14)
project(GameBoy65)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SOURCE_FILES src/main.cpp)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11)
target_include_directories(${PROJECT_NAME} PRIVATE 
  include
)

#target_link_libraries(${PROJECT_NAME} PRIVATE )
