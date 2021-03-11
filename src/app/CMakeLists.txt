cmake_minimum_required(VERSION 3.5)

project(app)

add_executable(linkage-app
    ContourPlot.h
    linkage-app.cpp
)
target_link_libraries(linkage-app
    kinematics
    gui
)
