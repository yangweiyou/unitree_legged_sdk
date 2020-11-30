cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_sdk_yangwei)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-O3")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
  message(STATUS "Build type not specified: Use Release by default")
endif(NOT CMAKE_BUILD_TYPE)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )

FIND_PACKAGE(Eigen3 3.0.0 REQUIRED)
FIND_PACKAGE(RBDL REQUIRED)

include_directories(include
    ${EIGEN3_INCLUDE_DIR}
    ${RBDL_INCLUDE_DIR}
    $ENV{LEGGED_ROBOT_PATH}/include)

link_directories(lib)
link_directories($ENV{LEGGED_ROBOT_PATH}/build)

set(EXTRA_LIBS -pthread 
               libunitree_legged_sdk_amd64.so 
               lcm
               ${RBDL_LIBRARY}
               ${RBDL_URDFReader_LIBRARY}
               yaml-cpp
               liblegged_robot.so)

# one pc one process
add_executable(test_walk_plan src/test_walk_plan.cpp)
target_link_libraries(test_walk_plan ${EXTRA_LIBS})

# one pc one process
add_executable(whole_body_control src/whole_body_control.cpp)
target_link_libraries(whole_body_control ${EXTRA_LIBS})

# one pc one process
add_executable(recovery src/recovery.cpp)
target_link_libraries(recovery ${EXTRA_LIBS})

# one pc one process
add_executable(zero_torque src/zero_torque.cpp)
target_link_libraries(zero_torque ${EXTRA_LIBS})

# one pc one process
add_executable(example_position examples/example_position.cpp)
target_link_libraries(example_position ${EXTRA_LIBS})

#add_executable(example_velocity examples/example_velocity.cpp)
#target_link_libraries(example_velocity ${EXTRA_LIBS})

#add_executable(example_torque examples/example_torque.cpp)
#target_link_libraries(example_torque ${EXTRA_LIBS})

#add_executable(example_walk examples/example_walk.cpp)
#target_link_libraries(example_walk ${EXTRA_LIBS})

# one pc multi process
#add_executable(lcm_server_high examples/lcm_server_high.cpp)
#target_link_libraries(lcm_server_high ${EXTRA_LIBS})

#add_executable(lcm_server_low examples/lcm_server_low.cpp)
#target_link_libraries(lcm_server_low ${EXTRA_LIBS})

# multi pc
#add_executable(udp_send_test examples/multi_pc_udp_send.cpp)
#target_link_libraries(udp_send_test ${EXTRA_LIBS})

#add_executable(udp_recv_test examples/multi_pc_udp_recv.cpp)
#target_link_libraries(udp_recv_test ${EXTRA_LIBS})

# lcm server
#add_executable(lcm_server examples/lcm_server.cpp)
#target_link_libraries(lcm_server ${EXTRA_LIBS})
