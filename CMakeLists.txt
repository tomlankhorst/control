cmake_minimum_required(VERSION 3.1)

# Call cmake with -D TESTS=ON to set this flag to true.
option(TESTS "build tests" OFF)

project(control CXX)

set (CMAKE_CXX_STANDARD 11)

include_directories(include)

if(TESTS)

  # Download and unpack googletest at configure time
  configure_file(cmake/gtest-cmakelists.txt.in googletest-download/CMakeLists.txt)
  execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
	RESULT_VARIABLE result
	WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
  if(result)
	message(FATAL_ERROR "CMake step for googletest failed: ${result}")
  endif()
  execute_process(COMMAND ${CMAKE_COMMAND} --build .
	RESULT_VARIABLE result
	WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
  if(result)
	message(FATAL_ERROR "Build step for googletest failed: ${result}")
  endif()

  # Prevent overriding the parent project's compiler/linker
  # settings on Windows
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)


  # Add googletest directly to our build. This defines
  # the gtest and gtest_main targets.
  add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
	${CMAKE_BINARY_DIR}/googletest-build
	EXCLUDE_FROM_ALL)

  enable_testing()

  include_directories("${gmock_SOURCE_DIR}/include")

  add_executable(controltests
	./tests/gtest.cpp
    tests/pid-test.cpp
    tests/biquad-test.cpp
    tests/biquad-cascade-test.cpp
    tests/prbs-test.cpp
    tests/ss-test.cpp)

  find_package (Eigen3 3.3 REQUIRED)

  target_link_libraries(controltests gtest_main gtest gmock Eigen3::Eigen)

  add_test(controltests controltests)

endif()
