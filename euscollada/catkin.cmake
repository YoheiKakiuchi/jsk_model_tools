# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(euscollada)

IF($ENV{ROS_DISTRO} STREQUAL "indigo")
find_package(catkin REQUIRED COMPONENTS collada_urdf rospack collada_parser resource_retriever)
ELSE()
find_package(catkin REQUIRED COMPONENTS collada_urdf rospack collada_parser urdfdom resource_retriever)
ENDIF()

catkin_package()

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")

find_package(PkgConfig)
IF($ENV{ROS_DISTRO} STREQUAL "indigo")
pkg_check_modules(urdf_dom urdfdom REQUIRED)
ENDIF()
pkg_check_modules(colladadom collada-dom-150 REQUIRED)
pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)
IF(${yaml_cpp_VERSION} VERSION_GREATER "0.5.0")
## indigo yaml-cpp : 0.5.0 /  hydro yaml-cpp : 0.3.0
  add_definitions("-DUSE_CURRENT_YAML")
ENDIF()
pkg_check_modules(assimpdevel assimp_devel REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${colladadom_INCLUDE_DIRS} ${yaml_cpp_INCLUDE_DIRS} ${assimpdevel_INCLUDE_DIRS} ${urdf_dom_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${assimpdevel_LIBRARY_DIRS})

add_executable(collada2eus_old src/collada2eus.cpp)
target_link_libraries(collada2eus_old ${catkin_LIBRARIES} qhull ${yaml_cpp_LIBRARIES} ${colladadom_LIBRARIES} ${recource_retriever_LIBRARIES})
add_dependencies(collada2eus_old libassimp_devel)

find_package(Boost REQUIRED system)
include_directories(${Boost_INCLUDE_DIR})
add_executable(collada2eus src/collada2eus_urdfmodel.cpp)
target_link_libraries(collada2eus ${catkin_LIBRARIES} qhull ${yaml_cpp_LIBRARIES} ${colladadom_LIBRARIES} ${collada_parser_LIBRARIES} ${recource_retriever_LIBRARIES} ${assimpdevel_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(collada2eus libassimp_devel)

install(TARGETS collada2eus_old collada2eus
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE)


file(GLOB _install_files RELATIVE ${PROJECT_SOURCE_DIR} *.yaml *.sh)
install(FILES ${_install_files}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
