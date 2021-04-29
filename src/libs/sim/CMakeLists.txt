cmake_minimum_required(VERSION 3.5)

project(sim)

set(CRL_TARGET_NAME ${PROJECT_NAME})

file(
    GLOB
    CRL_SOURCES #
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h" #
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" #
)

# basic dependencies
list(
    APPEND
    CRL_TARGET_DEPENDENCIES #
    "cmm::utils" #
    "cmm::gui" #
)

# target include dirs
list(APPEND CRL_TARGET_INCLUDE_DIRS #
     PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

# target link libs
list(
    APPEND
    CRL_TARGET_LINK_LIBS #
    PUBLIC
    "cmm::utils" #
    PUBLIC
    "cmm::gui" #
    PUBLIC
    "eigen" #
)

# compile definitions
list(APPEND CRL_COMPILE_DEFINITIONS #
     ""
)

# create target
create_crl_library(
    ${CRL_TARGET_NAME}
    "${CRL_SOURCES}" #
    "${CRL_TARGET_DEPENDENCIES}" #
    "${CRL_TARGET_INCLUDE_DIRS}" #
    "${CRL_TARGET_LINK_LIBS}" #
    "${CRL_COMPILE_DEFINITIONS}"
)