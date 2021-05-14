cmake_minimum_required(VERSION 3.5)

add_subdirectory(helpersLib)
add_subdirectory(optLib)


if(CMM_BUILD_GUI)
add_subdirectory(guiLib)
add_subdirectory(app)
endif(CMM_BUILD_GUI)
