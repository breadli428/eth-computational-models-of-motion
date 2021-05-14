cmake_minimum_required(VERSION 3.5)

project(app)

add_executable(shooting-app shooting-app.cpp)
target_link_libraries(shooting-app
    optLib
    guiLib
    helpersLib
)

add_executable(transcription-app transcription-app.cpp)
target_link_libraries(transcription-app
    optLib
    guiLib
    helpersLib
)

add_executable(challenge-app challenge-app.cpp)
target_link_libraries(challenge-app
    optLib
    guiLib
    helpersLib
)
