cmake_minimum_required(VERSION 3.5)

project(test-a1)

add_executable(${PROJECT_NAME}
    test.cpp
    TestResult.h
)
target_link_libraries(${PROJECT_NAME}
    nlohmann_json
    kinematics
)

add_test(${PROJECT_NAME} "test-a1")
