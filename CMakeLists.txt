cmake_minimum_required(VERSION 3.7)
project(indoor_routing)

set(BOOST_ENABLE_CMAKE ON)
set(Boost_USE_STATIC_LIBS ON)

find_package(Boost REQUIRED)
IF (Boost_FOUND)
    INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
    message(${Boost_INCLUDE_DIR})
    include_directories(${Boost_SOURCE_DIR})
ENDIF ()

#indoor_routing:copy_public_include_dir
set("target" "indoor_routing_copy_public_include_dir")
set("${target}__other_srcs" "include")
set("${target}__output" "tests/build/obj/include")
add_custom_command(OUTPUT ${${target}__output}
  COMMAND ${CMAKE_COMMAND} -E copy_directory "include" "tests/build/obj/include"
  DEPENDS  ${${target}__other_srcs}
  WORKING_DIRECTORY "tests/build/"
  COMMENT "Copy ${target}"
  VERBATIM)
add_custom_target("${target}" SOURCES ${${target}__other_srcs} DEPENDS ${${target}__output})


#indoor_routing:indoor_routing
set("target" "indoor_routing")
list(APPEND "${target}__cxx_srcs"
      "src/geometry.cpp"
	    "src/graph.cpp"
	    "src/router.cpp")
add_library("${target}" STATIC ${${target}__cxx_srcs})
# add_dependencies("${target}" boost_geometry)
target_include_directories("${target}" PRIVATE
  "include/"
  "tests/build/")
set_property(TARGET "${target}" PROPERTY CXX_STANDARD 17)
set_target_properties("${target}" PROPERTIES COMPILE_DEFINITIONS "BUILD_FOR_LINUX;_DEBUG;")
set_target_properties("${target}" PROPERTIES COMPILE_FLAGS "-fno-strict-aliasing -funwind-tables -fPIC -pipe -m64 -march=x86-64 -fstack-protector-strong -pthread -O3 -fno-omit-frame-pointer -g2 -fvisibility-inlines-hidden -Wno-undefined-bool-conversion -Wno-tautological-undefined-compare -std=c++17 -frtti -fexceptions ")

#tests:router_test
set("target" "indoor_routing_test")
list(APPEND "${target}__cxx_srcs"
  	"tests/router_test.cpp")
add_executable("${target}" ${${target}__cxx_srcs})
target_include_directories("${target}" PRIVATE
  "tests/include/"
  "tests/build/"
  "include/")
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK, -DBOOST_TEST_MAIN)
set_property(TARGET "${target}" PROPERTY CXX_STANDARD 17)
set_target_properties("${target}" PROPERTIES COMPILE_DEFINITIONS "BUILD_FOR_LINUX;_DEBUG;")
set_target_properties("${target}" PROPERTIES COMPILE_FLAGS "-fno-strict-aliasing -funwind-tables -fPIC -pipe -m64 -march=x86-64 -fstack-protector-strong -pthread -O3 -fno-omit-frame-pointer -g2 -fvisibility-inlines-hidden -Wno-undefined-bool-conversion -Wno-tautological-undefined-compare -std=c++17 -frtti -fexceptions ")
target_link_libraries("${target}" PRIVATE "indoor_routing")
