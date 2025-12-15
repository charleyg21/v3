project(FreeRTOS_CLI)
cmake_minimum_required(VERSION 3.16)

add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL
    FreeRTOS_CLI.c
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    $<TARGET_PROPERTY:board,INTERFACE_INCLUDE_DIRECTORIES>
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
    $<TARGET_PROPERTY:board,INTERFACE_COMPILE_DEFINITIONS>
)

add_dependencies(${PROJECT_NAME} board)
