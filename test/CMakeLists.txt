cmake_minimum_required(VERSION 3.13)

# set the project name
project(Test)

enable_testing()
find_package(GTest REQUIRED)

# add the executable
add_executable(gtest_test gtest.cpp)

target_link_libraries(gtest_test GTest::GTest GTest::Main)

#add_test(FailingTest gtest_test)
