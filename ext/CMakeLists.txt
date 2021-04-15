
# eigen
find_package(Eigen3 REQUIRED)

# glad
set(OpenGL_GL_PREFERENCE GLVND)
add_subdirectory(glad)

# glfw
set( GLFW_BUILD_EXAMPLES OFF CACHE BOOL  "GLFW lib only" )
set( GLFW_BUILD_TESTS OFF CACHE BOOL  "GLFW lib only" )
set( GLFW_BUILD_DOCS OFF CACHE BOOL  "GLFW lib only" )
set( GLFW_BUILD_INSTALL OFF CACHE BOOL  "GLFW lib only" )
set( GLFW_SRC_DIR ${CMAKE_SOURCE_DIR}/ext/glfw/include)
add_subdirectory(glfw)

# glm
set( GLM_TEST_ENABLE OFF CACHE BOOL "" )
add_subdirectory(glm)
target_include_directories(glm INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/glm>)

# nanovg
set(NANOVG_SRC_DIR ${CMAKE_SOURCE_DIR}/ext/nanovg/src)
file(GLOB sources ${NANOVG_SRC_DIR}/*.h ${NANOVG_SRC_DIR}/*.c)
add_library(nanovg ${sources})
set(NANOVG_SRC_DIR ${NANOVG_SRC_DIR} PARENT_SCOPE)

# imgui
add_subdirectory(imgui)
set( CRL_IMGUI_FONT_FOLDER "\"${CMAKE_CURRENT_SOURCE_DIR}/imgui/misc/fonts\"" CACHE STRING "")

# stb_image
add_subdirectory(stb_image)

# portable file dialog
add_library(portable_file_dialogs INTERFACE)
target_include_directories(portable_file_dialogs INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/portable-file-dialog)
