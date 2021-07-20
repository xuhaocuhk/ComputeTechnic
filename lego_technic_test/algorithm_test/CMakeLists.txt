include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

add_executable(runAlgorithmTest algorithm_test.cpp)

target_link_libraries(runAlgorithmTest lego_technic_system)
target_link_libraries(runAlgorithmTest gtest gtest_main)