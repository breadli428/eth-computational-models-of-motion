cmake_minimum_required(VERSION 3.5)

project(app)

find_package(Json REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(fem-app fem-app.cpp)
target_link_libraries(fem-app
    optLib
    guiLib
    femLib
    helpersLib
)

add_executable(manip-app manip-app.cpp)
target_link_libraries(manip-app
    optLib
    guiLib
    femLib
    helpersLib
)
