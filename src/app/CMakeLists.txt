cmake_minimum_required(VERSION 3.12)

project(rigidbodies)

file(GLOB CRL_SOURCES #
     "*.h" #
     "*.cpp" #
)

list(
    APPEND
    CRL_TARGET_DEPENDENCIES #
    "cmm::utils" #
    "cmm::gui" #
    "cmm::sim" #
)

list(APPEND CRL_TARGET_INCLUDE_DIRS #
     PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
)

list(
    APPEND
    CRL_TARGET_LINK_LIBS #
    PUBLIC
    "cmm::utils" #
    PUBLIC
    "cmm::gui" #
    PUBLIC
    "cmm::sim" #
)

list(APPEND CRL_COMPILE_DEFINITIONS #
     PUBLIC "CMM_DATA_FOLDER=${CMM_DATA_FOLDER}" #
)

create_crl_app(
    ${PROJECT_NAME}
    "${CRL_SOURCES}" #
    "${CRL_TARGET_DEPENDENCIES}" #
    "${CRL_TARGET_INCLUDE_DIRS}" #
    "${CRL_TARGET_LINK_LIBS}" #
    "${CRL_COMPILE_DEFINITIONS}"
)
