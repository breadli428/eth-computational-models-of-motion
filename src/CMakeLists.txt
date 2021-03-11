cmake_minimum_required(VERSION 3.5)

add_subdirectory(optimization)
add_subdirectory(kinematics)
add_subdirectory(test-a1)

if(CMM_BUILD_GUI)
add_subdirectory(gui)
add_subdirectory(app)
endif(CMM_BUILD_GUI)
