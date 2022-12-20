cmake_minimum_required(VERSION 2.8.3)
project(traj_generator)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")
set (catkin_simple_DIR /home/zhengtong/catkin_simple/devel/share/catkin_simple/cmake)

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    trac_ik_lib
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pkg_nlopt REQUIRED nlopt)

find_package(Boost REQUIRED COMPONENTS date_time)
find_package(orocos_kdl REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    trac_ik_lib
  DEPENDS
    Boost
    orocos_kdl
    pkg_nlopt
)

include_directories(	${EIGEN3_INCLUDE_DIRS} 
			${OSQPEIGEN_INCLUDE_DIRS} 
			${OSQP_INCLUDE_DIRS}
			${catkin_INCLUDE_DIRS}
  			${Boost_INCLUDE_DIRS}
  			${orocos_kdl_INCLUDE_DIRS}
  			${pkg_nlopt_INCLUDE_DIRS}  			
)

catkin_simple()

cs_add_executable(example src/example.cpp src/traj_following.cpp)
cs_add_library(traj_following src/traj_following.cpp)

cs_install()
cs_export()

