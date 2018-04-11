cmake_minimum_required(VERSION 3.1)

# Call cmake with -D TESTS=ON to set this flag to true.
option(TESTS "build tests" OFF)

project(control CXX)

set (CMAKE_CXX_STANDARD 11)

include_directories(include)

if(TESTS)

  enable_testing()

  # sets GTEST_INCLUDE_DIRS and GTEST_LIBRARIES
  find_package( GTest REQUIRED )      

  # adds the gtest include directory
  include_directories( ${GTEST_INCLUDE_DIRS} )

  add_executable(controltests
    ./tests/gtest.cpp
    ./tests/classic-test.cpp)

  target_link_libraries(controltests ${GTEST_LIBRARIES})

  add_test(controltests controltests)

endif()
