cmake_minimum_required(VERSION 2.6)
project(Asteroid CXX C)

#Use installed library
set(GT_USE_INSTALLED 0)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${PROJECT_SOURCE_DIR}/cmake")
set(CMAKE_BUILD_TYPE "DEBUG")


include_directories(BEFORE "${CMAKE_SOURCE_DIR}")
include(GtsamBuildTypes)

add_definitions(-O0 -Wall -g)

#Wrapping
include(GtsamMatlabWrap)

set(library_deps "")
set(linking_mode "static")

find_package(GTSAM REQUIRED NO_MODULE)
list(APPEND library_deps gtsam-${linking_mode})


find_package(Boost 1.43 COMPONENTS
  serialization
  system
  filesystem
  REQUIRED)

list(APPEND library_deps
  ${Boost_SERIALIZATION_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_FILESYSTEM_LIBRARY})

include_directories(${Boost_INCLUDE_DIR} ${GTSAM_INCLUDE_DIR})

set(ast_lib_source "")
set(ast_tests_source "")
file(GLOB ast_headers "cpp/*.h")
install(FILES ${ast_headers} DESTINATION include/Asteroid)

file(GLOB ast_srcs "cpp/*.cpp")
list(APPEND ast_lib_source ${ast_srcs})


file(GLOB ast_tests_srcs "cpp/tests/*.cpp")
list(APPEND ast_tests_source ${ast_tests_srcs})

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH true)
add_library(${PROJECT_NAME}-shared SHARED ${ast_lib_source})
set_target_properties(${PROJECT_NAME}-shared PROPERTIES 
  OUTPUT_NAME ${PROJECT_NAME}
  CLEAN_DIRECT_OUTPUT 1)

add_library(${PROJECT_NAME}-static STATIC ${ast_lib_source})
set_target_properties(${PROJECT_NAME}-static PROPERTIES 
  OUTPUT_NAME ${PROJECT_NAME}
  CLEAN_DIRECT_OUTPUT 1)

# Build Tests
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})

foreach(test_src_file ${ast_tests_source})
  get_filename_component(test_base ${test_src_file} NAME_WE)
  message(STATUS "Adding test ${test_src_file} with base name ${test_base}" )
  add_executable(${test_base} ${test_src_file})
  target_link_libraries(${test_base} ${PROJECT_NAME}-${linking_mode} ${library_deps} CppUnitLite)
  add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}/${test_base})
  add_custom_target(${test_base}.run ${test_base} ${ARGN})
  add_dependencies(check ${test_base})
endforeach(test_src_file)

install(TARGETS ${PROJECT_NAME}-shared LIBRARY DESTINATION lib)
install(TARGETS ${PROJECT_NAME}-static ARCHIVE DESTINATION lib)

set(GTSAM_BUILD_MEX_BINARY_FLAGS "" CACHE STRING "Extra flags for running mex compilation")
set(AST_TOOLBOX_DIR ../matlab/)
set(WRAP_HEADER_PATH "${GTSAM_DIR}/../../../include")
set(AST_TOOLBOX_FLAGS ${GTSAM_BUILD_MEX_BINARY_FLAGS} -I${Boost_INCLUDE_DIR} -I${MEX_INCLUDE_ROOT} -I${GTSAM_INCLUDE_DIR} -I${WRAP_HEADER_PATH} -I${PROJECT_SOURCE_DIR} -I${PROJECT_SOURCE_DIR}/cpp -Wl,-rpath,${CMAKE_BINARY_DIR}:${CMAKE_INSTALL_PREFIX}/lib)
set(AST_LIBRARY_DEPS gtsam)
set(GTSAM_BUILD_MEX_BIN ON)

wrap_library_generic(Asteroid "${AST_TOOLBOX_FLAGS}" "" "${AST_LIBRARY_DEPS}" "${AST_TOOLBOX_DIR}" "${WRAP_HEADER_PATH}")

#add_custom_target(wrap COMMAND wrap mex ~/School/Asteroid asteroid ../matlab "-I${CMAKE_INSTALL_PREFIX}/include -I${PROJECT_SOURCE_DIR}/cpp -L${CMAKE_INSTALL_PREFIX}/lib -lgtsam -L${CMAKE_BINARY_DIR} -lAsteroid")
