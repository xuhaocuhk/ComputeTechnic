cmake_minimum_required(VERSION 3.7)
project(lego_technic_main)

add_definitions(-std=c++11)

set(SOURCE_FILES main.cpp)
INCLUDE_DIRECTORIES(./lib/eigen)
#INCLUDE_DIRECTORIES(./jsoncpp/json)
#link_directories(./jsoncpp/lib_json)

add_executable(lego_technic_main ${SOURCE_FILES})
#target_link_libraries(lego_technic_main libjsoncpp.a)

include_directories(lego_technic_system)

add_subdirectory(lego_technic_system)
add_subdirectory(lego_technic_test)

target_link_libraries(lego_technic_main lego_technic_system)