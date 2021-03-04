cmake_minimum_required(VERSION 3.5)

project(test-a0)

add_executable(${PROJECT_NAME}
    test.cpp
)
target_link_libraries(${PROJECT_NAME}
    nlohmann_json
    math
)
target_compile_definitions(${PROJECT_NAME} PUBLIC CMM_MY_INFO="${CMAKE_CURRENT_LIST_DIR}/../../my-info.json")

add_test(${PROJECT_NAME} "test-a0")
